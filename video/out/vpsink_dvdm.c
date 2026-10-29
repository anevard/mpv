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

#include <math.h>
#include <string.h>

#include <libavutil/dovi_meta.h>

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

// An RPU fixed-point value back to its integer field: num/den * scale.
static int64_t fixed(AVRational q, double scale)
{
    return q.den ? llround((double)q.num * scale / q.den) : 0;
}

// L9/L10 primaries: signed 16-bit, CIE xy * 32767 in the RPU.
static void put_xy(struct writer *w, AVCIExy c)
{
    put16(w, (uint16_t)(int16_t)fixed(c.x, 32767));
    put16(w, (uint16_t)(int16_t)fixed(c.y, 32767));
}

static bool has_primaries(const AVColorPrimariesDesc *d)
{
    return d->prim.r.x.den && d->prim.r.y.den && d->wp.x.den;
}

static void put_primaries(struct writer *w, const AVColorPrimariesDesc *d)
{
    put_xy(w, d->prim.r);
    put_xy(w, d->prim.g);
    put_xy(w, d->prim.b);
    put_xy(w, d->wp);
}

// One extension block's payload, in RPU field order. ffmpeg zeroes fields an
// RPU block leaves out (L8's optional tail, L9/L10 primaries), so a block is
// cut after its last group that carries anything. False: no layout here.
static bool put_payload(struct writer *w, const AVDOVIDmData *e)
{
    switch (e->level) {
    case 1:
        put16(w, e->l1.min_pq);
        put16(w, e->l1.max_pq);
        put16(w, e->l1.avg_pq);
        return true;
    case 2:
        put16(w, e->l2.target_max_pq);
        put16(w, e->l2.trim_slope);
        put16(w, e->l2.trim_offset);
        put16(w, e->l2.trim_power);
        put16(w, e->l2.trim_chroma_weight);
        put16(w, e->l2.trim_saturation_gain);
        put16(w, (uint16_t)e->l2.ms_weight);    // -1 (unspecified): 0xFFFF
        return true;
    case 3:
        put16(w, e->l3.min_pq_offset);
        put16(w, e->l3.max_pq_offset);
        put16(w, e->l3.avg_pq_offset);
        return true;
    case 4:
        put16(w, e->l4.anchor_pq);
        put16(w, e->l4.anchor_power);
        return true;
    case 5:
        put16(w, e->l5.left_offset);
        put16(w, e->l5.right_offset);
        put16(w, e->l5.top_offset);
        put16(w, e->l5.bottom_offset);
        return true;
    case 6:
        put16(w, e->l6.max_luminance);
        put16(w, e->l6.min_luminance);
        put16(w, e->l6.max_cll);
        put16(w, e->l6.max_fall);
        return true;
    case 8: {
        const AVDOVIDmLevel8 *l = &e->l8;
        bool hue = false, sat = false;
        for (int i = 0; i < 6; i++) {
            hue |= l->hue_vector_field[i] != 0;
            sat |= l->saturation_vector_field[i] != 0;
        }
        sat |= hue;
        bool clip = sat || l->clip_trim;
        bool mid = clip || l->target_mid_contrast;
        put8(w, l->target_display_index);
        put16(w, l->trim_slope);
        put16(w, l->trim_offset);
        put16(w, l->trim_power);
        put16(w, l->trim_chroma_weight);
        put16(w, l->trim_saturation_gain);
        put16(w, l->ms_weight);
        if (mid)
            put16(w, l->target_mid_contrast);
        if (clip)
            put16(w, l->clip_trim);
        for (int i = 0; sat && i < 6; i++)
            put8(w, l->saturation_vector_field[i]);
        for (int i = 0; hue && i < 6; i++)
            put8(w, l->hue_vector_field[i]);
        return true;
    }
    case 9:
        put8(w, e->l9.source_primary_index);
        if (has_primaries(&e->l9.source_display_primaries))
            put_primaries(w, &e->l9.source_display_primaries);
        return true;
    case 10:
        put8(w, e->l10.target_display_index);
        put16(w, e->l10.target_max_pq);
        put16(w, e->l10.target_min_pq);
        put8(w, e->l10.target_primary_index);
        if (has_primaries(&e->l10.target_display_primaries))
            put_primaries(w, &e->l10.target_display_primaries);
        return true;
    case 11:
        put8(w, e->l11.content_type);
        put8(w, (e->l11.whitepoint & 15) << 4 |
                (e->l11.reference_mode_flag & 1) << 3);
        put16(w, 0);                            // reserved
        return true;
    case 254:
        put8(w, e->l254.dm_mode);
        put8(w, e->l254.dm_version_index);
        return true;
    case 255:
        put8(w, e->l255.dm_run_mode);
        put8(w, e->l255.dm_run_version);
        for (int i = 0; i < 4; i++)
            put8(w, e->l255.dm_debug[i]);
        return true;
    }
    return false;
}

size_t vpsink_dvdm_build(const struct AVDOVIMetadata *md, uint8_t *body,
                         size_t cap, struct vpsink_dvdm_info *info)
{
    *info = (struct vpsink_dvdm_info){0};
    if (!md)
        return 0;
    const AVDOVIColorMetadata *c = av_dovi_get_color(md);
    struct writer w = { .p = body, .cap = cap };

    // dm_metadata(), clause 6.2.1 Table 3. Bytes the clause fixes are written
    // as it fixes them; the rest come from the RPU.
    put8(&w, 0x00);
    put8(&w, c->scene_refresh_flag);
    for (int i = 0; i < 9; i++)
        put16(&w, (uint16_t)(int16_t)fixed(c->ycc_to_rgb_matrix[i], 1 << 13));
    for (int i = 0; i < 3; i++)
        put32(&w, (uint32_t)fixed(c->ycc_to_rgb_offset[i], 1 << 28));
    for (int i = 0; i < 9; i++)
        put16(&w, (uint16_t)(int16_t)fixed(c->rgb_to_lms_matrix[i], 1 << 14));
    put16(&w, 0xFFFF);                          // reserved_0xff_8bits x2
    for (int i = 0; i < 8; i++)
        put8(&w, 0x00);
    put8(&w, 12);                               // the tunnel carries 12 bits
    put8(&w, c->signal_color_space);
    put8(&w, 0x01);                             // reserved_0x01 (4:2:2)
    put8(&w, 0x01);                             // reserved_0x01
    put16(&w, c->source_min_pq);
    put16(&w, c->source_max_pq);
    put16(&w, c->source_diagonal);
    size_t count_at = w.len;
    put8(&w, 0);                                // num_ext_blocks

    for (int n = 0; n < md->num_ext_blocks && info->blocks < 255; n++) {
        const AVDOVIDmData *e = av_dovi_get_ext(md, n);
        size_t at = w.len;
        put32(&w, 0);                           // ext_block_length
        put8(&w, e->level);
        size_t start = w.len;
        if (!put_payload(&w, e)) {
            w.len = at;
            info->skipped++;
            continue;
        }
        if (w.overflow)
            break;
        uint32_t len = w.len - start;
        body[at] = len >> 24;
        body[at + 1] = len >> 16;
        body[at + 2] = len >> 8;
        body[at + 3] = len;
        info->levels[e->level / 8] |= 1u << (e->level % 8);
        info->blocks++;
    }
    if (w.overflow || w.len > VPSINK_DVDM_MAX_BYTES || count_at + 1 > w.len)
        return 0;
    body[count_at] = info->blocks;
    return w.len;
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
