/*
 * vpsink_dv: compose a Dolby Vision frame into a TV-led tunnel picture for
 * vo_vpsink.
 *
 * The base layer (and the enhancement layer, when the RPU's NLQ is active)
 * is uploaded to vpsink_dv's own libplacebo D3D11 GPU, reshaped by the RPU,
 * NLQ-composed, cut to 12-bit 4:2:2 and laid out as BGRA8 there, then read
 * back into the caller's buffer: the composed picture before the RPU's
 * nonlinear matrix, as a TV-led source sends it over HDMI. The DM packets are
 * written over it afterwards (vpsink_dvdm.c).
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

#ifndef MP_VPSINK_DV_H
#define MP_VPSINK_DV_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

struct mp_log;
struct mp_image;
struct vpsink_dv;

// Bytes per BGRA8 row: 4 a pixel, no padding.
#define VPSINK_DV_BGRA_STRIDE(w) ((uint64_t)(w) * 4)

// Times of one compose, mp_raw_time_ns() clock. GPU durations come from
// GPU timers and lag a few frames; 0 when none has finished yet.
struct vpsink_dv_times {
    int64_t upload_end_ns;      // BL (and EL) handed to the GPU
    int64_t compose_end_ns;     // reshape, NLQ and tunnel pack finished
    int64_t readback_end_ns;    // BGRA8 picture in the caller's buffer
    uint64_t gpu_compose_ns;    // GPU time of the reshape and NLQ pass
    uint64_t gpu_pack_ns;       // GPU time of the 4:2:2 and tunnel pack
    bool el_composed;           // the EL's NLQ residual was added
};

// The GPU is created on the first compose, so files without Dolby Vision
// never open one.
struct vpsink_dv *vpsink_dv_create(void *ta_parent, struct mp_log *log);
void vpsink_dv_destroy(struct vpsink_dv *dv);

// True if img is a Dolby Vision frame vpsink_dv can compose: RPU mapped by
// libplacebo, 10-bit base layer, even width.
bool vpsink_dv_frame_ok(struct mp_image *img);

// Compose img into dst as BGRA8, img->w x img->h, VPSINK_DV_BGRA_STRIDE rows.
// False if the GPU cannot be set up or any step fails; dst is then undefined.
bool vpsink_dv_compose(struct vpsink_dv *dv, struct mp_image *img,
                       uint8_t *dst, struct vpsink_dv_times *times);

#endif
