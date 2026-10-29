/*
 * vo_vpsink: hand each decoded frame to VideoProcessor through shared memory.
 *
 * Behaves like a capture card: each frame becomes visible to the reader at
 * its display time, mpv is never blocked, and a frame with no free slot is
 * dropped and counted. Subtitles (and, if enabled, mpv's OSD message) are
 * never drawn into the picture: each frame carries them as cue images for
 * the reader to draw. With --vo-vpsink-dv, a Dolby Vision frame is composed
 * on a GPU into the BGRA8 tunnel picture a TV-led player sends (vpsink_dv.c),
 * and dropped if it would show late. The contract is vpsink_shm.h.
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
#include <windows.h>
#include <avrt.h>

#include <libavutil/frame.h>
#include <libavutil/mastering_display_metadata.h>

#include "common/common.h"
#include "common/msg.h"
#include "mpv_talloc.h"
#include "options/m_option.h"
#include "options/options.h"
#include "osdep/io.h"
#include "osdep/timer.h"
#include "sub/draw_bmp.h"
#include "sub/osd.h"
#include "video/mp_image.h"
#include "vo.h"
#include "vpsink_dv.h"
#include "vpsink_shm.h"

#define ALIGN_UP(x, a) (((x) + (a) - 1) / (a) * (a))

#ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
#define CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 0x00000002
#endif

// Image subtitles in an HDR film are in the video's colour space. This level
// becomes cue white: BT.2408's graphics white, mpv's "sdr" for
// --image-subs-hdr-peak. Brighter is clipped.
#define CUE_WHITE_NITS  203.0f

// HLG to display light at BT.2100's nominal 1000 cd/m2 display.
#define HLG_PEAK_NITS   1000.0f
#define HLG_GAMMA       1.2f

#define LIN_LUT_SIZE    4096    // encoded value -> linear, one transfer
#define ENC_LUT_SIZE    16384   // linear (0..1) -> sRGB encoded

// One cue as the writer keeps it between frames: rendered and converted once
// per change, copied into every slot while it shows.
struct cue {
    struct mp_draw_sub_cache *cache;
    bool valid;                 // change_id and the pixels below are current
    int64_t change_id;          // of the sub_bitmap_list last rendered
    uint64_t id;                // 0: nothing showing
    struct mp_rect rc;          // bounds in the frame's pixels
    uint8_t *px;                // rc's pixels, BGRA premultiplied, w * 4 a row
};

struct priv {
    char *name;
    bool osd;                   // also hand over mpv's OSD message as a cue
    bool dv;                    // compose Dolby Vision frames as BGRA8 tunnel
                                // pictures (vpsink_dv.c)
    struct vpsink_dv *dvc;      // NULL unless dv

    HANDLE map;
    HANDLE event;
    HANDLE timer;               // high-resolution, for the final wait
    HANDLE avrt;                // MMCSS registration of the VO thread
    struct vpsink_header *hdr;

    int pending;                // slot filled by draw_frame, or -1
    bool discontinuity;         // flag the next published frame
    bool paused;                // VOCTRL_PAUSE seen, no VOCTRL_RESUME since
    uint64_t frame_number;
    int64_t raw_offset;         // mp_raw_time_ns() - mp_time_ns()
    uint64_t picture_capacity;  // bytes of a slot's pixel data for the
                                // picture; the cue area follows

    struct cue sub;
    struct cue msg;
    uint64_t last_cue_id;

    // HDR image-subtitle conversion. lin_lut is for lin_trc: PQ in units of
    // CUE_WHITE_NITS, HLG as scene light (0..1).
    enum pl_color_transfer lin_trc;
    float lin_lut[LIN_LUT_SIZE];
    float enc_lut[ENC_LUT_SIZE];
    bool enc_lut_ready;
};

static struct vpsink_slot *get_slot(struct priv *p, int n)
{
    return (struct vpsink_slot *)((uint8_t *)p->hdr + p->hdr->header_size +
                                  n * p->hdr->slot_stride);
}

static bool slot_cas(struct vpsink_slot *s, int32_t from, int32_t to)
{
    return InterlockedCompareExchange((volatile LONG *)&s->state, to, from) == from;
}

// Take a FREE slot, else the oldest READY one the reader has not taken.
static int acquire_slot(struct priv *p)
{
    for (int n = 0; n < p->hdr->slot_count; n++) {
        if (slot_cas(get_slot(p, n), VPSINK_SLOT_FREE, VPSINK_SLOT_WRITING))
            return n;
    }
    for (;;) {
        int oldest = -1;
        for (int n = 0; n < p->hdr->slot_count; n++) {
            struct vpsink_slot *s = get_slot(p, n);
            if (s->state == VPSINK_SLOT_READY &&
                (oldest < 0 || s->frame_number < get_slot(p, oldest)->frame_number))
                oldest = n;
        }
        if (oldest < 0)
            return -1;
        if (slot_cas(get_slot(p, oldest), VPSINK_SLOT_READY, VPSINK_SLOT_WRITING)) {
            p->hdr->frames_overwritten += 1;
            return oldest;
        }
        // The reader took it first; look again.
    }
}

static void copy_hdr10(struct vpsink_hdr10 *dst, struct mp_image *img)
{
    *dst = (struct vpsink_hdr10){0};
    for (int n = 0; n < img->num_ff_side_data; n++) {
        struct mp_ff_side_data *sd = &img->ff_side_data[n];
        if (sd->type == AV_FRAME_DATA_MASTERING_DISPLAY_METADATA &&
            sd->buf->size >= sizeof(AVMasteringDisplayMetadata))
        {
            const AVMasteringDisplayMetadata *m = (void *)sd->buf->data;
            dst->has_primaries = m->has_primaries;
            dst->has_luminance = m->has_luminance;
            for (int c = 0; c < 3; c++) {
                for (int xy = 0; xy < 2; xy++) {
                    dst->display_primaries[c][xy][0] = m->display_primaries[c][xy].num;
                    dst->display_primaries[c][xy][1] = m->display_primaries[c][xy].den;
                }
            }
            for (int xy = 0; xy < 2; xy++) {
                dst->white_point[xy][0] = m->white_point[xy].num;
                dst->white_point[xy][1] = m->white_point[xy].den;
            }
            dst->min_luminance[0] = m->min_luminance.num;
            dst->min_luminance[1] = m->min_luminance.den;
            dst->max_luminance[0] = m->max_luminance.num;
            dst->max_luminance[1] = m->max_luminance.den;
        } else if (sd->type == AV_FRAME_DATA_CONTENT_LIGHT_LEVEL &&
                   sd->buf->size >= sizeof(AVContentLightMetadata))
        {
            const AVContentLightMetadata *l = (void *)sd->buf->data;
            dst->has_light_level = 1;
            dst->max_cll = l->MaxCLL;
            dst->max_fall = l->MaxFALL;
        }
    }
}

// Everything about the frame but its pixels.
static void fill_fields(struct vo *vo, struct vpsink_slot *s,
                        struct vo_frame *frame, struct mp_image *img)
{
    struct priv *p = vo->priv;
    s->width = img->w;
    s->height = img->h;
    s->sar_num = img->params.p_w;
    s->sar_den = img->params.p_h;
    s->display_time_ns = frame->pts > 0 ? frame->pts + p->raw_offset : 0;
    s->duration_ns = frame->duration >= 0 ? (int64_t)frame->duration : -1;
    s->flags = p->discontinuity ? VPSINK_FLAG_DISCONTINUITY : 0;

    s->tags = (struct vpsink_color_tags){
        .set = img->av_tags.set,
        .primaries = img->av_tags.primaries,
        .trc = img->av_tags.trc,
        .space = img->av_tags.space,
        .range = img->av_tags.range,
        .chroma_location = img->av_tags.chroma_location,
    };
    copy_hdr10(&s->hdr10, img);

    s->pkt_read_ns = img->stamps.pkt_read_ns;
    s->pkt_sent_ns = img->stamps.pkt_sent_ns;
    s->decoded_ns = img->stamps.decoded_ns;
    s->ready_ns = img->stamps.ready_ns;
    s->queued_ns = img->stamps.queued_ns;

    s->dv_upload_end_ns = s->dv_compose_end_ns = s->dv_readback_end_ns = 0;
    s->dv_gpu_compose_ns = s->dv_gpu_pack_ns = 0;
    s->dv_flags = 0;
}

static bool fill_slot(struct vo *vo, struct vpsink_slot *s,
                      struct vo_frame *frame, struct mp_image *img)
{
    struct priv *p = vo->priv;
    uint8_t *data = (uint8_t *)s + p->hdr->slot_data_offset;
    uint64_t used = 0;

    for (int n = 0; n < 2; n++) {
        int row_bytes = mp_image_plane_w(img, n) * img->fmt.bpp[n] / 8;
        int rows = mp_image_plane_h(img, n);
        uint32_t stride = ALIGN_UP(row_bytes, 64);
        if (used + (uint64_t)stride * rows > p->picture_capacity)
            return false;
        s->plane_offset[n] = p->hdr->slot_data_offset + used;
        s->plane_stride[n] = stride;
        s->plane_rows[n] = rows;
        memcpy_pic(data + used, img->planes[n], row_bytes, rows,
                   stride, img->stride[n]);
        used += (uint64_t)stride * rows;
    }

    s->plane_count = 2;
    s->format = img->imgfmt == IMGFMT_P010 ? VPSINK_FMT_P010 : VPSINK_FMT_NV12;
    fill_fields(vo, s, frame, img);
    return true;
}

// Compose a Dolby Vision frame on the GPU straight into the slot as BGRA8.
static bool fill_slot_dv(struct vo *vo, struct vpsink_slot *s,
                         struct vo_frame *frame, struct mp_image *img)
{
    struct priv *p = vo->priv;
    uint64_t stride = VPSINK_DV_BGRA_STRIDE(img->w);
    if (stride * img->h > p->picture_capacity)
        return false;

    struct vpsink_dv_times t;
    if (!vpsink_dv_compose(p->dvc, img, (uint8_t *)s + p->hdr->slot_data_offset, &t))
        return false;

    s->plane_count = 1;
    s->format = VPSINK_FMT_BGRA8;
    s->plane_offset[0] = p->hdr->slot_data_offset;
    s->plane_stride[0] = stride;
    s->plane_rows[0] = img->h;
    s->plane_offset[1] = s->plane_stride[1] = s->plane_rows[1] = 0;
    fill_fields(vo, s, frame, img);

    s->dv_upload_end_ns = t.upload_end_ns;
    s->dv_compose_end_ns = t.compose_end_ns;
    s->dv_readback_end_ns = t.readback_end_ns;
    s->dv_gpu_compose_ns = t.gpu_compose_ns;
    s->dv_gpu_pack_ns = t.gpu_pack_ns;
    s->dv_flags = t.el_composed ? VPSINK_DV_EL_COMPOSED : 0;
    return true;
}

// A frame is late once it would show more than half a frame after its
// display time. 0: no time to judge by (a paused redraw).
static int64_t late_after_ns(struct vo_frame *frame, struct priv *p, bool still)
{
    if (still || frame->pts <= 0)
        return 0;
    return frame->pts + p->raw_offset + MPMAX(frame->duration, 0) / 2;
}

// Encoded value (0..1) to linear for trc: PQ in units of CUE_WHITE_NITS,
// HLG as scene light (0..1; the OOTF needs all three channels).
static float trc_to_linear(enum pl_color_transfer trc, float e)
{
    if (trc == PL_COLOR_TRC_PQ) {
        const float m1 = 2610.0f / 16384, m2 = 2523.0f / 4096 * 128;
        const float c1 = 3424.0f / 4096, c2 = 2413.0f / 4096 * 32,
                    c3 = 2392.0f / 4096 * 32;
        float x = powf(e, 1 / m2);
        float y = powf(MPMAX(x - c1, 0) / (c2 - c3 * x), 1 / m1);
        return y * 10000.0f / CUE_WHITE_NITS;
    }
    const float a = 0.17883277f, b = 0.28466892f, c = 0.55991073f;
    return e <= 0.5f ? e * e / 3 : (expf((e - c) / a) + b) / 12;
}

static float srgb_encode(float l)
{
    return l <= 0.0031308f ? l * 12.92f : 1.055f * powf(l, 1 / 2.4f) - 0.055f;
}

// Convert an image-subtitle cue from an HDR film's colour space to sRGB, in
// place. Each pixel is un-premultiplied, converted and premultiplied again,
// so edges keep their colour.
static void convert_hdr_cue(struct priv *p, struct cue *c,
                            const struct pl_color_space *color)
{
    enum pl_color_transfer trc = color->transfer;
    if (p->lin_trc != trc) {
        for (int n = 0; n < LIN_LUT_SIZE; n++)
            p->lin_lut[n] = trc_to_linear(trc, n / (float)(LIN_LUT_SIZE - 1));
        p->lin_trc = trc;
    }
    if (!p->enc_lut_ready) {
        for (int n = 0; n < ENC_LUT_SIZE; n++)
            p->enc_lut[n] = srgb_encode(n / (float)(ENC_LUT_SIZE - 1));
        p->enc_lut_ready = true;
    }

    enum pl_color_primaries prim = color->primaries ? color->primaries
                                                    : PL_COLOR_PRIM_BT_2020;
    pl_matrix3x3 gamut = pl_get_color_mapping_matrix(
        pl_raw_primaries_get(prim), pl_raw_primaries_get(PL_COLOR_PRIM_BT_709),
        PL_INTENT_RELATIVE_COLORIMETRIC);

    int w = c->rc.x1 - c->rc.x0, h = c->rc.y1 - c->rc.y0;
    for (int y = 0; y < h; y++) {
        uint8_t *px = c->px + (size_t)y * w * 4;
        for (int x = 0; x < w; x++, px += 4) {
            int a = px[3];
            if (!a)
                continue;
            float rgb[3];
            for (int ch = 0; ch < 3; ch++) {
                float e = MPMIN(px[2 - ch] / (float)a, 1.0f);   // B,G,R order
                rgb[ch] = p->lin_lut[(int)(e * (LIN_LUT_SIZE - 1) + 0.5f)];
            }
            if (trc == PL_COLOR_TRC_HLG) {
                float ys = 0.2627f * rgb[0] + 0.6780f * rgb[1] + 0.0593f * rgb[2];
                float k = HLG_PEAK_NITS * powf(ys, HLG_GAMMA - 1) / CUE_WHITE_NITS;
                for (int ch = 0; ch < 3; ch++)
                    rgb[ch] *= k;
            }
            pl_matrix3x3_apply(&gamut, rgb);
            for (int ch = 0; ch < 3; ch++) {
                float l = MPCLAMP(rgb[ch], 0.0f, 1.0f);
                float e = p->enc_lut[(int)(l * (ENC_LUT_SIZE - 1) + 0.5f)];
                px[2 - ch] = (uint8_t)(e * a + 0.5f);
            }
        }
    }
}

// Shrink rc to the pixels of ov that are not fully transparent. draw_bmp
// reports its active area in whole tiles; the reader places by exact bounds.
static struct mp_rect tight_bounds(struct mp_image *ov, struct mp_rect rc)
{
    rc.x0 = MPMAX(rc.x0, 0);
    rc.y0 = MPMAX(rc.y0, 0);
    rc.x1 = MPMIN(rc.x1, ov->w);
    rc.y1 = MPMIN(rc.y1, ov->h);
    struct mp_rect t = {rc.x1, rc.y1, rc.x0, rc.y0};
    for (int y = rc.y0; y < rc.y1; y++) {
        const uint8_t *row = ov->planes[0] + (ptrdiff_t)y * ov->stride[0];
        for (int x = rc.x0; x < rc.x1; x++) {
            if (row[x * 4 + 3]) {
                t.x0 = MPMIN(t.x0, x);
                t.x1 = MPMAX(t.x1, x + 1);
                t.y0 = MPMIN(t.y0, y);
                t.y1 = MPMAX(t.y1, y + 1);
            }
        }
    }
    return t;
}

// Bring cue c up to date for img's time. Renders only when mpv reports a
// change; a changed image gets a new id.
static void render_cue(struct vo *vo, struct cue *c, struct mp_image *img,
                       int draw_flags)
{
    struct priv *p = vo->priv;

    struct sub_bitmap_list *sbs =
        osd_render(vo->osd, osd_res_from_image_params(&img->params), img->pts,
                   draw_flags, mp_draw_sub_formats);
    if (c->valid && sbs->change_id == c->change_id) {
        talloc_free(sbs);
        return;
    }

    // mpv marks image subtitles as in the video's colour space (sd_lavc.c).
    bool video_space = false;
    for (int n = 0; n < sbs->num_items; n++) {
        if (sbs->items[n]->format == SUBBITMAP_BGRA &&
            sbs->items[n]->video_color_space)
            video_space = true;
    }

    if (!c->cache)
        c->cache = mp_draw_sub_alloc(vo, vo->global);
    struct mp_rect act, mod;
    int num_act = 0, num_mod = 0;
    struct mp_image *ov = mp_draw_sub_overlay(c->cache, sbs, &act, 1, &num_act,
                                              &mod, 1, &num_mod);
    int64_t change_id = sbs->change_id;
    talloc_free(sbs);

    c->valid = false;
    c->id = 0;
    if (!ov) {
        MP_WARN(vo, "Cannot render a subtitle or OSD cue.\n");
        return;
    }
    c->valid = true;
    c->change_id = change_id;
    if (!num_act)
        return;

    c->rc = tight_bounds(ov, act);
    int w = c->rc.x1 - c->rc.x0, h = c->rc.y1 - c->rc.y0;
    if (w <= 0 || h <= 0)
        return;
    c->px = talloc_realloc_size(vo, c->px, (size_t)w * h * 4);
    memcpy_pic(c->px, mp_image_pixel_ptr(ov, 0, c->rc.x0, c->rc.y0), w * 4, h,
               w * 4, ov->stride[0]);

    enum pl_color_transfer trc = img->params.color.transfer;
    if (video_space && (trc == PL_COLOR_TRC_PQ || trc == PL_COLOR_TRC_HLG))
        convert_hdr_cue(p, c, &img->params.color);

    c->id = ++p->last_cue_id;
}

// Copy cue c into the slot's cue area at *used, or mark it absent.
static void put_cue(struct priv *p, struct vpsink_slot *s, struct vpsink_cue *dst,
                    struct cue *c, uint64_t *used)
{
    *dst = (struct vpsink_cue){0};
    if (!c->id)
        return;
    uint32_t w = c->rc.x1 - c->rc.x0, h = c->rc.y1 - c->rc.y0;
    uint32_t stride = ALIGN_UP(w * 4, 64);
    if (*used + (uint64_t)stride * h > VPSINK_CUE_AREA_SIZE)
        return;     // only the message can miss: a subtitle always fits
    uint64_t offset = p->hdr->slot_data_offset + p->picture_capacity + *used;
    memcpy_pic((uint8_t *)s + offset, c->px, w * 4, h, stride, w * 4);
    *dst = (struct vpsink_cue){
        .id = c->id,
        .x = c->rc.x0,
        .y = c->rc.y0,
        .w = w,
        .h = h,
        .offset = offset,
        .stride = stride,
    };
    *used += (uint64_t)stride * h;
}

// Bring the subtitle cue (and the OSD message cue, if enabled) up to date for
// img's time and copy them into the slot.
static void draw_cues(struct vo *vo, struct vpsink_slot *s, struct mp_image *img)
{
    struct priv *p = vo->priv;

    s->draw_start_ns = mp_raw_time_ns();
    render_cue(vo, &p->sub, img, OSD_DRAW_SUB_ONLY);
    if (p->osd)
        render_cue(vo, &p->msg, img, OSD_DRAW_OSD_ONLY);
    uint64_t used = 0;
    put_cue(p, s, &s->sub, &p->sub, &used);
    put_cue(p, s, &s->msg, &p->msg, &used);
    s->draw_end_ns = mp_raw_time_ns();
}

static bool draw_frame(struct vo *vo, struct vo_frame *frame)
{
    struct priv *p = vo->priv;
    p->pending = -1;

    // Repeats and redraws are the same picture again: a card would not
    // deliver them as new frames, so neither do we. Except a redraw while
    // paused: it carries a new subtitle or OSD message cue (a track change,
    // a message), so it goes out once as a still frame and shows at once.
    struct mp_image *img = frame->current;
    bool still = frame->still && p->paused;     // only do_redraw sets still
    if (!img || frame->repeat || (frame->redraw && !still))
        return VO_TRUE;

    // Dolby Vision goes out as the tunnel picture a TV-led player sends. A
    // frame already late is not composed at all.
    bool dv = p->dvc && vpsink_dv_frame_ok(img);
    int64_t late_after = dv ? late_after_ns(frame, p, still) : 0;
    if (late_after && (int64_t)mp_raw_time_ns() > late_after) {
        p->hdr->dv_frames_late += 1;
        return VO_TRUE;
    }

    int n = acquire_slot(p);
    if (n < 0) {
        p->hdr->frames_dropped += 1;
        return VO_TRUE;
    }

    struct vpsink_slot *s = get_slot(p, n);
    s->copy_start_ns = mp_raw_time_ns();
    bool filled = dv ? fill_slot_dv(vo, s, frame, img) : fill_slot(vo, s, frame, img);
    s->copy_end_ns = mp_raw_time_ns();
    if (!filled) {
        InterlockedExchange((volatile LONG *)&s->state, VPSINK_SLOT_FREE);
        if (dv)
            p->hdr->dv_compose_failed += 1;
        else
            p->hdr->frames_dropped += 1;
        return VO_TRUE;
    }
    // Composed too late: drop it rather than publish it late.
    if (late_after && s->copy_end_ns > late_after) {
        InterlockedExchange((volatile LONG *)&s->state, VPSINK_SLOT_FREE);
        p->hdr->dv_frames_late += 1;
        return VO_TRUE;
    }
    if (still)
        s->flags |= VPSINK_FLAG_STILL;
    draw_cues(vo, s, img);

    p->pending = n;
    return VO_TRUE;
}

// vo.c calls this at the frame's display time.
static void flip_page(struct vo *vo)
{
    struct priv *p = vo->priv;
    if (p->pending < 0)
        return;

    struct vpsink_slot *s = get_slot(p, p->pending);
    p->pending = -1;
    s->flip_entry_ns = mp_raw_time_ns();

    // vo.c's wait can return up to ~2 ms early (millisecond timed waits on
    // Windows). Never publish before the display time: sleep on a
    // high-resolution timer for most of the rest, then spin the last bit.
    if (s->display_time_ns) {
        int64_t left = s->display_time_ns - (int64_t)mp_raw_time_ns();
        if (left > 500000 && p->timer) {
            LARGE_INTEGER due = { .QuadPart = -(left - 300000) / 100 };
            if (SetWaitableTimer(p->timer, &due, 0, NULL, NULL, FALSE))
                WaitForSingleObject(p->timer, INFINITE);
        }
        while ((int64_t)mp_raw_time_ns() < s->display_time_ns)
            YieldProcessor();
    }

    s->frame_number = ++p->frame_number;
    s->publish_time_ns = mp_raw_time_ns();
    if (!s->display_time_ns)
        s->display_time_ns = s->publish_time_ns;
    if (p->discontinuity) {
        p->hdr->discontinuities += 1;
        p->discontinuity = false;
    }

    bool dv = s->format == VPSINK_FMT_BGRA8;    // the reader owns s after this
    MemoryBarrier();
    InterlockedExchange((volatile LONG *)&s->state, VPSINK_SLOT_READY);

    p->hdr->frames_published += 1;
    if (dv)
        p->hdr->dv_frames_composed += 1;
    p->hdr->mpv_drop_count = vo_get_drop_count(vo);
    SetEvent(p->event);
}

static int query_format(struct vo *vo, int format)
{
    // Only what SOCKET's pass-through takes, so mpv never converts pixels
    // for us. Use hardware decode with copy-back (d3d11va-copy).
    return format == IMGFMT_NV12 || format == IMGFMT_P010;
}

static int reconfig(struct vo *vo, struct mp_image_params *params)
{
    struct priv *p = vo->priv;
    if (params->w > VPSINK_MAX_WIDTH || params->h > VPSINK_MAX_HEIGHT) {
        MP_ERR(vo, "%dx%d is larger than the %ux%u a slot holds.\n",
               params->w, params->h, VPSINK_MAX_WIDTH, VPSINK_MAX_HEIGHT);
        return -1;
    }
    p->discontinuity = true;
    // Size or colour may have changed: render and convert the cues again.
    p->sub.valid = p->msg.valid = false;
    return 0;
}

static void uninit(struct vo *vo)
{
    struct priv *p = vo->priv;
    vpsink_dv_destroy(p->dvc);
    p->dvc = NULL;
    if (p->hdr) {
        if (p->pending >= 0) {
            InterlockedExchange((volatile LONG *)&get_slot(p, p->pending)->state,
                                VPSINK_SLOT_FREE);
        }
        if (p->hdr->writer_pid == GetCurrentProcessId())
            p->hdr->writer_pid = 0;
        UnmapViewOfFile(p->hdr);
    }
    if (p->avrt)
        AvRevertMmThreadCharacteristics(p->avrt);
    p->avrt = NULL;
    if (p->timer)
        CloseHandle(p->timer);
    if (p->event)
        CloseHandle(p->event);
    if (p->map)
        CloseHandle(p->map);
}

static int preinit(struct vo *vo)
{
    struct priv *p = vo->priv;
    p->pending = -1;
    p->discontinuity = true;

    uint64_t raw = mp_raw_time_ns();
    p->raw_offset = raw - mp_time_ns_from_raw_time(raw);

    uint32_t header_size = ALIGN_UP(sizeof(struct vpsink_header), 4096);
    uint32_t slot_data_offset = ALIGN_UP(sizeof(struct vpsink_slot), 4096);
    // Y plane plus half-height UV plane, 2 bytes per sample, rows padded to 64,
    // or a BGRA8 frame, whichever is larger; then the cue area.
    p->picture_capacity = MPMAX(
        (uint64_t)ALIGN_UP(VPSINK_MAX_WIDTH * 2, 64) *
            (VPSINK_MAX_HEIGHT + VPSINK_MAX_HEIGHT / 2),
        VPSINK_DV_BGRA_STRIDE(VPSINK_MAX_WIDTH) * VPSINK_MAX_HEIGHT);
    uint64_t capacity = p->picture_capacity + VPSINK_CUE_AREA_SIZE;
    uint64_t slot_stride = ALIGN_UP(slot_data_offset + capacity, 4096);
    uint64_t total = header_size + VPSINK_SLOT_COUNT * slot_stride;

    const char *name = p->name && p->name[0] ? p->name : "default";
    char *map_name = talloc_asprintf(vo, "Local\\vpsink-%s", name);
    char *event_name = talloc_asprintf(vo, "Local\\vpsink-%s-ready", name);

    p->map = CreateFileMappingW(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE,
                                (DWORD)(total >> 32), (DWORD)total,
                                mp_from_utf8(vo, map_name));
    if (!p->map) {
        MP_ERR(vo, "Cannot create shared memory %s.\n", map_name);
        goto error;
    }
    bool existed = GetLastError() == ERROR_ALREADY_EXISTS;

    p->hdr = MapViewOfFile(p->map, FILE_MAP_ALL_ACCESS, 0, 0, total);
    if (!p->hdr) {
        MP_ERR(vo, "Cannot map shared memory %s.\n", map_name);
        goto error;
    }

    p->event = CreateEventW(NULL, FALSE, FALSE, mp_from_utf8(vo, event_name));
    if (!p->event) {
        MP_ERR(vo, "Cannot create event %s.\n", event_name);
        goto error;
    }

    if (existed) {
        // A reader kept the mapping open across an mpv restart. It must be
        // our layout; slots a dead writer left half-written are freed.
        if (p->hdr->magic != VPSINK_MAGIC || p->hdr->version != VPSINK_VERSION ||
            p->hdr->header_size != header_size || p->hdr->slot_stride != slot_stride)
        {
            MP_ERR(vo, "Shared memory %s exists with another layout.\n", map_name);
            goto error;
        }
        // One writer only: refuse while the recorded writer is still running.
        HANDLE other = p->hdr->writer_pid ?
            OpenProcess(SYNCHRONIZE, FALSE, p->hdr->writer_pid) : NULL;
        if (other) {
            bool alive = WaitForSingleObject(other, 0) == WAIT_TIMEOUT;
            CloseHandle(other);
            if (alive) {
                MP_ERR(vo, "Shared memory %s already has a writer (pid %u).\n",
                       map_name, (unsigned)p->hdr->writer_pid);
                // Not ours: unmap without touching it.
                UnmapViewOfFile(p->hdr);
                p->hdr = NULL;
                goto error;
            }
        }
        for (int n = 0; n < p->hdr->slot_count; n++)
            slot_cas(get_slot(p, n), VPSINK_SLOT_WRITING, VPSINK_SLOT_FREE);
        p->frame_number = 0;
        for (int n = 0; n < p->hdr->slot_count; n++) {
            uint64_t f = get_slot(p, n)->frame_number;
            if (f > p->frame_number)
                p->frame_number = f;
        }
    } else {
        p->hdr->version = VPSINK_VERSION;
        p->hdr->header_size = header_size;
        p->hdr->slot_count = VPSINK_SLOT_COUNT;
        p->hdr->slot_stride = slot_stride;
        p->hdr->slot_data_offset = slot_data_offset;
        p->hdr->slot_data_capacity = capacity;
        LARGE_INTEGER freq;
        QueryPerformanceFrequency(&freq);
        p->hdr->qpc_frequency = freq.QuadPart;
        // The reader checks magic last, so publish it after everything else.
        MemoryBarrier();
        p->hdr->magic = VPSINK_MAGIC;
    }
    p->hdr->writer_pid = GetCurrentProcessId();

    // Windows 10 1803+. Without it flip_page only spins: a plain waitable
    // timer can oversleep by a whole scheduler tick (~15 ms).
    p->timer = CreateWaitableTimerExW(NULL, NULL,
                                      CREATE_WAITABLE_TIMER_HIGH_RESOLUTION,
                                      TIMER_ALL_ACCESS);
    if (!p->timer)
        MP_VERBOSE(vo, "No high-resolution timer; spinning before publish.\n");

    // preinit runs on the VO thread. Register it with MMCSS as mpv's window
    // VOs do (w32_common.c, --vo-mmcss-profile, "Playback" by default).
    // Without it this thread was measured waking 10-50 ms late several times
    // a minute; with it, never more than ~1 ms.
    if (vo->opts->mmcss_profile && vo->opts->mmcss_profile[0]) {
        wchar_t *profile = mp_from_utf8(NULL, vo->opts->mmcss_profile);
        p->avrt = AvSetMmThreadCharacteristicsW(profile, &(DWORD){0});
        talloc_free(profile);
        if (!p->avrt)
            MP_WARN(vo, "Cannot register the VO thread with MMCSS.\n");
    }

    if (p->dv)
        p->dvc = vpsink_dv_create(vo, vo->log);

    MP_VERBOSE(vo, "Shared memory %s, %u slots of %llu bytes.\n", map_name,
               VPSINK_SLOT_COUNT, (unsigned long long)slot_stride);
    return 0;

error:
    uninit(vo);
    p->hdr = NULL;
    p->event = p->map = NULL;
    return -1;
}

static int control(struct vo *vo, uint32_t request, void *data)
{
    struct priv *p = vo->priv;
    switch (request) {
    case VOCTRL_RESET:
        // Seek or flush: the next frame does not follow the last one.
        p->discontinuity = true;
        return VO_TRUE;
    case VOCTRL_PAUSE:
    case VOCTRL_RESUME:
        p->paused = request == VOCTRL_PAUSE;
        return VO_TRUE;
    }
    return VO_NOTIMPL;
}

#define OPT_BASE_STRUCT struct priv
const struct vo_driver video_out_vpsink = {
    .description = "VideoProcessor shared-memory sink",
    .name = "vpsink",
    .preinit = preinit,
    .query_format = query_format,
    .reconfig = reconfig,
    .control = control,
    .draw_frame = draw_frame,
    .flip_page = flip_page,
    .uninit = uninit,
    .priv_size = sizeof(struct priv),
    .options = (const struct m_option[]) {
        {"name", OPT_STRING(name)},
        {"osd", OPT_BOOL(osd)},
        {"dv", OPT_BOOL(dv)},
        {0},
    },
    .options_prefix = "vo-vpsink",
};
