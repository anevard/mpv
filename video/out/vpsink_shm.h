/*
 * vo_vpsink shared-memory contract.
 *
 * mpv (vo_vpsink) is the only writer; one reader (VP's source plugin, or the
 * test reader) takes frames out. Plain C with fixed-width fields so MinGW and
 * MSVC agree on the layout. No mpv, libplacebo or ffmpeg types cross here:
 * colour tags are libavutil's AVCOL_* numbers exactly as the decoder's
 * AVFrame carried them, and HDR10 values are copied field by field.
 *
 * Mapping name:  "Local\\vpsink-<name>"         (CreateFileMapping, pagefile)
 * Event name:    "Local\\vpsink-<name>-ready"   (auto-reset, set on publish)
 *
 * Layout: struct vpsink_header at offset 0, then slot_count slots, each
 * slot_stride bytes apart starting at header_size. A slot is a
 * struct vpsink_slot followed by pixel data at slot_data_offset.
 *
 * Slot protocol (all state changes by interlocked compare-exchange):
 *   writer: FREE or READY -> WRITING (copy pixels and fields)
 *           at display time: WRITING -> READY, then SetEvent
 *   reader: READY -> READING (take the highest frame_number)
 *           when done:  READING -> FREE
 * The writer never waits. A READY slot the reader has not taken may be
 * reused for a newer frame; the reader sees that as a gap in frame_number.
 * If no slot can be taken the frame is dropped and frames_dropped counts it.
 */

#ifndef MP_VPSINK_SHM_H
#define MP_VPSINK_SHM_H

#include <stdint.h>

#define VPSINK_MAGIC        0x4B4E5356u   // "VSNK"
#define VPSINK_VERSION      1u
#define VPSINK_SLOT_COUNT   3u

// Largest frame a slot holds: 4096x2304 P010 (Y plane plus half-height UV).
#define VPSINK_MAX_WIDTH    4096u
#define VPSINK_MAX_HEIGHT   2304u

enum vpsink_slot_state {
    VPSINK_SLOT_FREE    = 0,
    VPSINK_SLOT_WRITING = 1,
    VPSINK_SLOT_READY   = 2,
    VPSINK_SLOT_READING = 3,
};

enum vpsink_format {
    VPSINK_FMT_NONE = 0,
    VPSINK_FMT_NV12 = 1,    // 8-bit 4:2:0, Y plane then interleaved UV plane
    VPSINK_FMT_P010 = 2,    // 10-bit 4:2:0 in the high bits of 16-bit words
};

// vpsink_slot.flags
#define VPSINK_FLAG_DISCONTINUITY   (1u << 0)   // first frame after a seek,
                                                // reset or format change

// Colour tags as the AVFrame carried them (AVCOL_*). set == 0 means the
// frame did not come from the decoder, so no tags are known at all.
struct vpsink_color_tags {
    uint32_t set;
    int32_t  primaries;         // enum AVColorPrimaries
    int32_t  trc;               // enum AVColorTransferCharacteristic
    int32_t  space;             // enum AVColorSpace
    int32_t  range;             // enum AVColorRange
    int32_t  chroma_location;   // enum AVChromaLocation
};

// HDR10 static metadata, copied from AV_FRAME_DATA_MASTERING_DISPLAY_METADATA
// and AV_FRAME_DATA_CONTENT_LIGHT_LEVEL. Rationals stay as num/den pairs.
struct vpsink_hdr10 {
    uint32_t has_primaries;
    uint32_t has_luminance;
    int32_t  display_primaries[3][2][2];    // [R,G,B][x,y][num,den]
    int32_t  white_point[2][2];             // [x,y][num,den]
    int32_t  min_luminance[2];              // [num,den], cd/m2
    int32_t  max_luminance[2];              // [num,den], cd/m2
    uint32_t has_light_level;
    uint32_t max_cll;
    uint32_t max_fall;
    uint32_t reserved;
};

struct vpsink_slot {
    volatile int32_t state;     // enum vpsink_slot_state
    uint32_t flags;             // VPSINK_FLAG_*
    uint64_t frame_number;      // monotonic from 1; never repeats

    // Times are QueryPerformanceCounter converted to ns (as mpv's
    // mp_raw_time_ns()): ticks * 1e9 / qpc_frequency.
    int64_t  display_time_ns;   // when mpv wants the frame on screen
    int64_t  publish_time_ns;   // when the slot actually became READY
    int64_t  duration_ns;       // approximate, from mpv; <0 if unknown

    uint32_t format;            // enum vpsink_format
    uint32_t width;             // pixels, after the decoder's cropping
    uint32_t height;
    uint32_t sar_num;           // sample aspect ratio, 0/0 if unknown
    uint32_t sar_den;
    uint32_t plane_count;       // 2
    uint32_t plane_offset[2];   // from the start of the slot
    uint32_t plane_stride[2];   // bytes per row
    uint32_t plane_rows[2];
    uint32_t reserved0;

    struct vpsink_color_tags tags;
    struct vpsink_hdr10 hdr10;

    // Diagnostics, same clock: when the writer began and finished copying
    // this frame into the slot. A late publish_time_ns with a late
    // copy_start_ns means mpv woke late; with a long copy, the copy was slow.
    int64_t  copy_start_ns;
    int64_t  copy_end_ns;

    uint32_t reserved[12];
};

struct vpsink_header {
    uint32_t magic;             // VPSINK_MAGIC
    uint32_t version;           // VPSINK_VERSION
    uint32_t header_size;       // offset of slot 0
    uint32_t slot_count;
    uint64_t slot_stride;       // bytes between slots
    uint32_t slot_data_offset;  // offset of pixel data inside a slot
    uint32_t reserved0;
    uint64_t slot_data_capacity;
    int64_t  qpc_frequency;     // QueryPerformanceFrequency, for reference
    uint32_t writer_pid;
    uint32_t reserved1;

    // Written by the writer only; 64-bit aligned so reads never tear on x64.
    volatile uint64_t frames_published;
    volatile uint64_t frames_dropped;       // no slot free, or too large
    volatile uint64_t frames_overwritten;   // READY slot reused unread
    volatile uint64_t discontinuities;
    volatile int64_t  mpv_drop_count;       // mpv's own late-frame drops

    uint32_t reserved[32];
};

#endif
