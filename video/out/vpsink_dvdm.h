/*
 * vpsink_dvdm: Dolby Vision display management (DM) metadata for the
 * TV-led tunnel picture vpsink_dv composes.
 *
 * ETSI GS CCM 001 V1.1.1: clause 6.2 gives the DM byte syntax, 6.3 the
 * 128-byte transmission packets and 6.4 their embedding in the picture. The
 * RPU's fields are carried as they are, each widened to whole bytes (a 12- or
 * 13-bit field becomes two big-endian bytes, an 8-bit field one, padding
 * bits are dropped); that is the clause 6.2 layout of levels 1, 2 and 5, and
 * the same rule carries every other level the RPU has.
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

#ifndef MP_VPSINK_DVDM_H
#define MP_VPSINK_DVDM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Largest DM body VP's tunnel reader accepts.
#define VPSINK_DVDM_MAX_BYTES 0x2F00

struct vpsink_dvdm_info {
    uint8_t levels[32];         // bit n set: extension level n carried
    int blocks;                 // extension blocks carried
    size_t l4_at, l4_len;       // body bytes of the L4 block; 0 if none
    char error[128];            // why no body was built, if none was
};

// Build the DM body for one picture from the frame's raw RPU (ffmpeg's
// AV_FRAME_DATA_DOVI_RPU_BUFFER, an HEVC UNSPEC62 NAL unit), parsed by
// libdovi; each RPU field goes once to its wire bytes. Returns the length in
// bytes, or 0 with info->error set.
size_t vpsink_dvdm_build(const uint8_t *rpu, size_t rpu_len, uint8_t *body,
                         size_t cap, struct vpsink_dvdm_info *info);

// Packetize body (CRC-32, three copies of each packet) and embed it into the
// chroma LSBs of a BGRA8 tunnel picture, w * 4 bytes a row, from the first
// pixel in raster order. id (0..15) names the metadata, which describes
// this picture (affected = current); keep it while the metadata is
// unchanged. False if the picture is too small.
bool vpsink_dvdm_embed(uint8_t *bgra, int w, int h, const uint8_t *body,
                       size_t len, unsigned id);

#endif
