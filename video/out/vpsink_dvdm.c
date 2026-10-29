/*
 * vpsink_dvdm: Dolby Vision DM metadata for the TV-led tunnel picture.
 *
 * This file is part of mpv.
 *
 * mpv is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * mpv is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with mpv.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <string.h>

#include <libdovi/rpu_parser.h>

#include "vpsink_dvdm.h"

#define PACKET_BYTES    128
#define PACKET_BITS     (PACKET_BYTES * 8)
#define REPEATS         3                       // clause 6.4.3
#define FIRST_BODY      119                     // packet_type 0b00 / 0b01
#define NEXT_BODY       121                     // packet_type 0b10 / 0b11
#define HEADER_BYTES    71                      // dm_metadata() before blocks

struct writer {
    uint8_t *p;
    size_t len, cap;
    bool overflow;
};

static void put8(struct writer *w, unsigned v)
{
    if (w->len + 1 > w->cap) {
        w->overflow = true;
        return;
    }
    w->p[w->len++] = v;
}

static void put16(struct writer *w, unsigned v)
{
    put8(w, (v >> 8) & 0xFF);
    put8(w, v & 0xFF);
}

static void put32(struct writer *w, uint32_t v)
{
    put16(w, v >> 16);
    put16(w, v & 0xFFFF);
}

// L9/L10 primaries: the RPU's eight 16-bit CIE xy fields, R G B W.
static void put_primaries(struct writer *w, uint16_t rx, uint16_t ry,
                          uint16_t gx, uint16_t gy, uint16_t bx, uint16_t by,
                          uint16_t wx, uint16_t wy)
{
    put16(w, rx);
    put16(w, ry);
    put16(w, gx);
    put16(w, gy);
    put16(w, bx);
    put16(w, by);
    put16(w, wx);
    put16(w, wy);
}

// L8, L9 and L10 have optional tails; the RPU's ext_block_length (bytes)
// says which groups the file carries, and exactly those go out.
static void put_l8(struct writer *w, const DoviExtMetadataBlockLevel8 *l)
{
    put8(w, l->target_display_index);
    put16(w, l->trim_slope);
    put16(w, l->trim_offset);
    put16(w, l->trim_power);
    put16(w, l->trim_chroma_weight);
    put16(w, l->trim_saturation_gain);
    put16(w, l->ms_weight);
    if (l->length > 10)
        put16(w, l->target_mid_contrast);
    if (l->length > 12)
        put16(w, l->clip_trim);
    if (l->length > 13) {
        put8(w, l->saturation_vector_field0);
        put8(w, l->saturation_vector_field1);
        put8(w, l->saturation_vector_field2);
        put8(w, l->saturation_vector_field3);
        put8(w, l->saturation_vector_field4);
        put8(w, l->saturation_vector_field5);
    }
    if (l->length > 19) {
        put8(w, l->hue_vector_field0);
        put8(w, l->hue_vector_field1);
        put8(w, l->hue_vector_field2);
        put8(w, l->hue_vector_field3);
        put8(w, l->hue_vector_field4);
        put8(w, l->hue_vector_field5);
    }
}

static void put_l9(struct writer *w, const DoviExtMetadataBlockLevel9 *l)
{
    put8(w, l->source_primary_index);
    if (l->length > 1)
        put_primaries(w, l->source_primary_red_x, l->source_primary_red_y,
                      l->source_primary_green_x, l->source_primary_green_y,
                      l->source_primary_blue_x, l->source_primary_blue_y,
                      l->source_primary_white_x, l->source_primary_white_y);
}

static void put_l10(struct writer *w, const DoviExtMetadataBlockLevel10 *l)
{
    put8(w, l->target_display_index);
    put16(w, l->target_max_pq);
    put16(w, l->target_min_pq);
    put8(w, l->target_primary_index);
    if (l->length > 5)
        put_primaries(w, l->target_primary_red_x, l->target_primary_red_y,
                      l->target_primary_green_x, l->target_primary_green_y,
                      l->target_primary_blue_x, l->target_primary_blue_y,
                      l->target_primary_white_x, l->target_primary_white_y);
}

struct blocks {
    struct writer *w;
    struct vpsink_dvdm_info *info;
    size_t at, start;
};

static void block_begin(struct blocks *b, int level)
{
    b->at = b->w->len;
    put32(b->w, 0);                             // ext_block_length
    put8(b->w, level);
    b->start = b->w->len;
}

static void block_end(struct blocks *b, int level)
{
    if (b->w->overflow)
        return;
    uint32_t len = b->w->len - b->start;
    uint8_t *p = b->w->p + b->at;
    p[0] = len >> 24;
    p[1] = len >> 16;
    p[2] = len >> 8;
    p[3] = len;
    b->info->levels[level / 8] |= 1u << (level % 8);
    b->info->blocks++;
}

static void set_error(struct vpsink_dvdm_info *info, const char *msg)
{
    snprintf(info->error, sizeof(info->error), "%s", msg ? msg : "unknown");
}

// dm_metadata(), clause 6.2.1 Table 3, from one parsed RPU.
static size_t build_dm(const DoviVdrDmData *d, uint8_t *body, size_t cap,
                       struct vpsink_dvdm_info *info)
{
    struct writer w = { .p = body, .cap = cap };

    // Bytes the clause fixes are written as it fixes them; the rest are the
    // RPU's integer fields.
    put8(&w, 0x00);
    put8(&w, d->scene_refresh_flag);
    put16(&w, (uint16_t)d->ycc_to_rgb_coef0);
    put16(&w, (uint16_t)d->ycc_to_rgb_coef1);
    put16(&w, (uint16_t)d->ycc_to_rgb_coef2);
    put16(&w, (uint16_t)d->ycc_to_rgb_coef3);
    put16(&w, (uint16_t)d->ycc_to_rgb_coef4);
    put16(&w, (uint16_t)d->ycc_to_rgb_coef5);
    put16(&w, (uint16_t)d->ycc_to_rgb_coef6);
    put16(&w, (uint16_t)d->ycc_to_rgb_coef7);
    put16(&w, (uint16_t)d->ycc_to_rgb_coef8);
    put32(&w, d->ycc_to_rgb_offset0);
    put32(&w, d->ycc_to_rgb_offset1);
    put32(&w, d->ycc_to_rgb_offset2);
    put16(&w, (uint16_t)d->rgb_to_lms_coef0);
    put16(&w, (uint16_t)d->rgb_to_lms_coef1);
    put16(&w, (uint16_t)d->rgb_to_lms_coef2);
    put16(&w, (uint16_t)d->rgb_to_lms_coef3);
    put16(&w, (uint16_t)d->rgb_to_lms_coef4);
    put16(&w, (uint16_t)d->rgb_to_lms_coef5);
    put16(&w, (uint16_t)d->rgb_to_lms_coef6);
    put16(&w, (uint16_t)d->rgb_to_lms_coef7);
    put16(&w, (uint16_t)d->rgb_to_lms_coef8);
    put16(&w, 0xFFFF);                          // reserved_0xff_8bits x2
    for (int i = 0; i < 8; i++)
        put8(&w, 0x00);
    put8(&w, 12);                               // the tunnel carries 12 bits
    put8(&w, d->signal_color_space);
    put8(&w, 0x01);                             // reserved_0x01 (4:2:2)
    put8(&w, 0x01);                             // reserved_0x01
    put16(&w, d->source_min_pq);
    put16(&w, d->source_max_pq);
    put16(&w, d->source_diagonal);
    size_t count_at = w.len;
    put8(&w, 0);                                // num_ext_blocks

    // Clause 6.2 sets no block order: by level, repeats in file order.
    const DoviDmData *m = &d->dm_data;
    struct blocks b = { .w = &w, .info = info };
    if (m->level1) {
        block_begin(&b, 1);
        put16(&w, m->level1->min_pq);
        put16(&w, m->level1->max_pq);
        put16(&w, m->level1->avg_pq);
        block_end(&b, 1);
    }
    for (size_t n = 0; n < m->level2.len; n++) {
        const DoviExtMetadataBlockLevel2 *l = m->level2.list[n];
        block_begin(&b, 2);
        put16(&w, l->target_max_pq);
        put16(&w, l->trim_slope);
        put16(&w, l->trim_offset);
        put16(&w, l->trim_power);
        put16(&w, l->trim_chroma_weight);
        put16(&w, l->trim_saturation_gain);
        put16(&w, (uint16_t)l->ms_weight);      // -1 (unspecified): 0xFFFF
        block_end(&b, 2);
    }
    if (m->level3) {
        block_begin(&b, 3);
        put16(&w, m->level3->min_pq_offset);
        put16(&w, m->level3->max_pq_offset);
        put16(&w, m->level3->avg_pq_offset);
        block_end(&b, 3);
    }
    if (m->level4) {
        block_begin(&b, 4);
        put16(&w, m->level4->anchor_pq);
        put16(&w, m->level4->anchor_power);
        block_end(&b, 4);
    }
    if (m->level5) {
        block_begin(&b, 5);
        put16(&w, m->level5->active_area_left_offset);
        put16(&w, m->level5->active_area_right_offset);
        put16(&w, m->level5->active_area_top_offset);
        put16(&w, m->level5->active_area_bottom_offset);
        block_end(&b, 5);
    }
    if (m->level6) {
        block_begin(&b, 6);
        put16(&w, m->level6->max_display_mastering_luminance);
        put16(&w, m->level6->min_display_mastering_luminance);
        put16(&w, m->level6->max_content_light_level);
        put16(&w, m->level6->max_frame_average_light_level);
        block_end(&b, 6);
    }
    for (size_t n = 0; n < m->level8.len; n++) {
        block_begin(&b, 8);
        put_l8(&w, m->level8.list[n]);
        block_end(&b, 8);
    }
    if (m->level9) {
        block_begin(&b, 9);
        put_l9(&w, m->level9);
        block_end(&b, 9);
    }
    for (size_t n = 0; n < m->level10.len; n++) {
        block_begin(&b, 10);
        put_l10(&w, m->level10.list[n]);
        block_end(&b, 10);
    }
    if (m->level11) {
        const DoviExtMetadataBlockLevel11 *l = m->level11;
        block_begin(&b, 11);
        put8(&w, l->content_type);
        put8(&w, (l->whitepoint & 15) << 4 | (l->reference_mode_flag & 1) << 3);
        put8(&w, l->reserved_byte2);
        put8(&w, l->reserved_byte3);
        block_end(&b, 11);
    }
    if (m->level254) {
        block_begin(&b, 254);
        put8(&w, m->level254->dm_mode);
        put8(&w, m->level254->dm_version_index);
        block_end(&b, 254);
    }
    if (m->level255) {
        const DoviExtMetadataBlockLevel255 *l = m->level255;
        block_begin(&b, 255);
        put8(&w, l->dm_run_mode);
        put8(&w, l->dm_run_version);
        put8(&w, l->dm_debug0);
        put8(&w, l->dm_debug1);
        put8(&w, l->dm_debug2);
        put8(&w, l->dm_debug3);
        block_end(&b, 255);
    }

    if (w.overflow || info->blocks > 254 || w.len > VPSINK_DVDM_MAX_BYTES) {
        set_error(info, "DM body too large");
        return 0;
    }
    body[count_at] = info->blocks;
    return w.len;
}

size_t vpsink_dvdm_build(const uint8_t *rpu, size_t rpu_len, uint8_t *body,
                         size_t cap, struct vpsink_dvdm_info *info)
{
    *info = (struct vpsink_dvdm_info){0};
    if (!rpu || !rpu_len) {
        set_error(info, "no RPU on the frame");
        return 0;
    }
    size_t len = 0;
    DoviRpuOpaque *r = dovi_parse_unspec62_nalu(rpu, rpu_len);
    const char *err = r ? dovi_rpu_get_error(r) : "parse failed";
    const DoviRpuDataHeader *h = err ? NULL : dovi_rpu_get_header(r);
    const DoviVdrDmData *d = NULL;
    if (err) {
        set_error(info, err);
    } else if (!h || !h->vdr_dm_metadata_present_flag) {
        set_error(info, "RPU carries no DM metadata");
    } else if (!(d = dovi_rpu_get_vdr_dm_data(r))) {
        set_error(info, "no vdr_dm_data");
    } else if (d->compressed) {
        // A compressed DM leaves fields to an earlier RPU; not carried yet.
        set_error(info, "compressed DM metadata");
    } else {
        len = build_dm(d, body, cap, info);
    }
    if (d)
        dovi_rpu_free_vdr_dm_data(d);
    if (h)
        dovi_rpu_free_header(h);
    if (r)
        dovi_rpu_free(r);
    return len;
}

// ISO/IEC 13818-1 CRC-32, clause 6.3.4.
static uint32_t crc32_mpeg(const uint8_t *p, size_t n)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint32_t)p[i] << 24;
        for (int b = 0; b < 8; b++)
            crc = crc & 0x80000000u ? (crc << 1) ^ 0x04C11DB7u : crc << 1;
    }
    return crc;
}

static unsigned parity8(unsigned v)
{
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    return v & 1;
}

// One packet bit per pixel (clause 6.4.2): the chroma sample's LSB is set so
// that the 24 bits of the pixel have the bit's parity. In the BGRA8 tunnel
// layout that LSB is bit 4 of B (B = C[3:0] I[3:0]).
static void embed_packet(uint8_t *px, const uint8_t *packet)
{
    for (int bit = 0; bit < PACKET_BITS; bit++, px += 4) {
        unsigned want = (packet[bit / 8] >> (7 - bit % 8)) & 1;
        unsigned b = px[0] & ~0x10u;
        unsigned rest = parity8(b) ^ parity8(px[1]) ^ parity8(px[2]);
        px[0] = b | (want ^ rest) << 4;
    }
}

bool vpsink_dvdm_embed(uint8_t *bgra, int w, int h, const uint8_t *body,
                       size_t len, unsigned id)
{
    if (!len || len > VPSINK_DVDM_MAX_BYTES || w <= 0 || h <= 0)
        return false;
    size_t packets = len <= FIRST_BODY ? 1 :
                     1 + (len - FIRST_BODY + NEXT_BODY - 1) / NEXT_BODY;
    if (packets * REPEATS * PACKET_BITS > (size_t)w * h)
        return false;

    size_t done = 0;
    uint8_t *px = bgra;
    for (size_t n = 0; n < packets; n++) {
        uint8_t packet[PACKET_BYTES] = {0};
        bool first = n == 0, last = n + 1 == packets;
        unsigned type = first ? (last ? 0 : 1) : (last ? 3 : 2);
        // Header, clause 6.3.2: packet_type, metadata_type 0, version 0,
        // no_md 0; affected id = current id; EOS 0.
        packet[0] = type << 6;
        packet[1] = (id & 15) << 4 | (id & 15);
        packet[2] = 0;
        size_t at = 3, room = NEXT_BODY;
        if (first) {
            packet[3] = len >> 8;
            packet[4] = len;
            at = 5;
            room = FIRST_BODY;
        }
        size_t chunk = len - done < room ? len - done : room;
        memcpy(packet + at, body + done, chunk);
        done += chunk;
        uint32_t crc = crc32_mpeg(packet, PACKET_BYTES - 4);
        packet[124] = crc >> 24;
        packet[125] = crc >> 16;
        packet[126] = crc >> 8;
        packet[127] = crc;
        for (int copy = 0; copy < REPEATS; copy++, px += PACKET_BITS * 4)
            embed_packet(px, packet);
    }
    return true;
}
