/*
 * vo_vpsink: hand each decoded frame to VideoProcessor through shared memory.
 *
 * Behaves like a capture card: each frame becomes visible to the reader at
 * its display time, mpv is never blocked, and a frame with no free slot is
 * dropped and counted. Subtitles (and, if enabled, mpv's OSD text) are drawn
 * into the slot's copy, never into the decoder's frame. The contract is
 * vpsink_shm.h.
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

#include <windows.h>
#include <avrt.h>

#include <libavutil/frame.h>
#include <libavutil/mastering_display_metadata.h>

#include "common/msg.h"
#include "mpv_talloc.h"
#include "options/m_option.h"
#include "options/options.h"
#include "osdep/io.h"
#include "osdep/timer.h"
#include "sub/osd.h"
#include "video/mp_image.h"
#include "vo.h"
#include "vpsink_shm.h"

#define ALIGN_UP(x, a) (((x) + (a) - 1) / (a) * (a))

#ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
#define CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 0x00000002
#endif

struct priv {
    char *name;
    int sub_nits;               // subtitle white in an HDR frame, cd/m2
    bool osd;                   // also draw mpv's OSD text, not only subtitles

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
        if (used + (uint64_t)stride * rows > p->hdr->slot_data_capacity)
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
    return true;
}

// Draw the subtitles for img's time (and the OSD text, if enabled) into the
// slot's copy of img. draw_bmp touches only the areas that hold them.
static void draw_subs(struct vo *vo, struct vpsink_slot *s, struct mp_image *img)
{
    struct priv *p = vo->priv;

    struct mp_image dst = {0};
    mp_image_set_params(&dst, &img->params);
    for (int n = 0; n < 2; n++) {
        dst.planes[n] = (uint8_t *)s + s->plane_offset[n];
        dst.stride[n] = s->plane_stride[n];
    }

    // draw_bmp converts subtitle colour to the video's with zimg, which puts
    // subtitle white at the frame's mastering peak (video/zimg.c). Give it
    // our level instead; only this drawing sees it, the slot's HDR10
    // metadata is the frame's own. Dynamic metadata is cleared, as
    // mp_image_params_static_equal() does, so it does not rebuild draw_bmp's
    // cache on every frame.
    bool hdr = pl_color_space_is_hdr(&dst.params.color);
    dst.params.repr.dovi = NULL;
    dst.params.color.hdr = (struct pl_hdr_metadata){0};
    if (hdr)
        dst.params.color.hdr.max_luma = p->sub_nits;

    s->draw_start_ns = mp_raw_time_ns();
    osd_draw_on_image(vo->osd, osd_res_from_image_params(&dst.params), img->pts,
                      p->osd ? 0 : OSD_DRAW_SUB_ONLY, &dst);
    s->draw_end_ns = mp_raw_time_ns();
}

static bool draw_frame(struct vo *vo, struct vo_frame *frame)
{
    struct priv *p = vo->priv;
    p->pending = -1;

    // Repeats and redraws are the same picture again: a card would not
    // deliver them as new frames, so neither do we. Except a redraw while
    // paused: it carries new subtitles or OSD text (a track change, a
    // message), so it goes out once as a still frame and shows at once.
    struct mp_image *img = frame->current;
    bool still = frame->still && p->paused;     // only do_redraw sets still
    if (!img || frame->repeat || (frame->redraw && !still))
        return VO_TRUE;

    int n = acquire_slot(p);
    if (n < 0) {
        p->hdr->frames_dropped += 1;
        return VO_TRUE;
    }

    struct vpsink_slot *s = get_slot(p, n);
    s->copy_start_ns = mp_raw_time_ns();
    bool filled = fill_slot(vo, s, frame, img);
    s->copy_end_ns = mp_raw_time_ns();
    if (!filled) {
        InterlockedExchange((volatile LONG *)&s->state, VPSINK_SLOT_FREE);
        p->hdr->frames_dropped += 1;
        return VO_TRUE;
    }
    if (still)
        s->flags |= VPSINK_FLAG_STILL;
    draw_subs(vo, s, img);

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

    MemoryBarrier();
    InterlockedExchange((volatile LONG *)&s->state, VPSINK_SLOT_READY);

    p->hdr->frames_published += 1;
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
    return 0;
}

static void uninit(struct vo *vo)
{
    struct priv *p = vo->priv;
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
    // Y plane plus half-height UV plane, 2 bytes per sample, rows padded to 64.
    uint64_t capacity = (uint64_t)ALIGN_UP(VPSINK_MAX_WIDTH * 2, 64) *
                        (VPSINK_MAX_HEIGHT + VPSINK_MAX_HEIGHT / 2);
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
    .priv_defaults = &(const struct priv) {
        .sub_nits = 50,
    },
    .options = (const struct m_option[]) {
        {"name", OPT_STRING(name)},
        {"sub-nits", OPT_INT(sub_nits), M_RANGE(10, 1000)},
        {"osd", OPT_BOOL(osd)},
        {0},
    },
    .options_prefix = "vo-vpsink",
};
