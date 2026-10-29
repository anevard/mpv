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
 * struct vpsink_slot followed by pixel data at slot_data_offset: the
 * picture's planes, then the cue area holding that frame's cues.
 *
 * Cues: mpv's subtitles and its OSD message are not drawn into the picture.
 * Each slot carries the frame's subtitle cue and message cue as images in
 * its cue area, cropped to what they cover, with an id that changes only
 * when the image does. Every frame carries its cues, so a frame the reader
 * skips loses nothing; the reader need copy a cue only when its id changes.
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
#define VPSINK_VERSION      3u
#define VPSINK_SLOT_COUNT   3u

// Largest frame a slot holds: 4096x2304, as P010 (Y plane plus half-height
// UV) or as R210 (4 bytes a pixel; the larger of the two sizes the slot).
#define VPSINK_MAX_WIDTH    4096u
#define VPSINK_MAX_HEIGHT   2304u

// The cue area holds a full-frame cue: VPSINK_MAX_WIDTH x VPSINK_MAX_HEIGHT
// at 4 bytes a pixel.
#define VPSINK_CUE_AREA_SIZE ((uint64_t)VPSINK_MAX_WIDTH * 4u * VPSINK_MAX_HEIGHT)

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
    // One plane, r210 as a DeckLink card delivers it: big-endian 32-bit
    // words, 2 zero bits then 10-bit R, G, B; rows of 64-pixel groups (256
    // bytes). Written only with vo-vpsink-dv: a Dolby Vision RGB tunnel
    // picture (12-bit 4:2:2 in RGB8, HDMI 1.4a 6.5.4), as a TV-led player
    // sends it. A reader that does not know it skips the slot.
    VPSINK_FMT_R210 = 3,
};

// vpsink_slot.flags
#define VPSINK_FLAG_DISCONTINUITY   (1u << 0)   // first frame after a seek,
                                                // reset or format change
#define VPSINK_FLAG_STILL           (1u << 1)   // paused redraw: the same
                                                // picture with new subtitles
                                                // or OSD text

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

// One cue: premultiplied-alpha pixels, bytes B, G, R, A in memory order
// (DXGI_FORMAT_B8G8R8A8_UNORM), sRGB. Subtitles from an HDR film arrive
// converted: 203 cd/m2 is white (1.0), brighter is clipped. The receiver
// chooses the level white is shown at.
struct vpsink_cue {
    uint64_t id;                // 0: no cue on this frame. Otherwise unique
                                // to one image; repeats while it is unchanged
    int32_t  x;                 // bounds in the frame's pixels (width x
    int32_t  y;                 // height, before sample aspect ratio)
    uint32_t w;
    uint32_t h;
    uint32_t offset;            // from the start of the slot
    uint32_t stride;            // bytes per row
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
    uint32_t plane_count;       // 2; 1 for R210
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
    // When vo.c's wait ended and flip_page began. Late here means mpv's
    // wait; between this and publish_time_ns is vo_vpsink's own wait.
    int64_t  flip_entry_ns;
    // The frame's path upstream of SINK, same clock; 0 if not stamped. The
    // packet times are the last packet the decoder took before this frame
    // came out (decoders reorder, so not necessarily this frame's own).
    // Judge each against display_time_ns, not against the stage before.
    // Read late: the demuxer (file or network). Sent late after an early
    // read: the player thread. Ready late after an early send: decode or
    // copy-back. With hardware decode the decoder returns a frame before
    // the GPU finishes it, so decoded_ns follows the send at once and the
    // decode work shows between decoded_ns and ready_ns.
    int64_t  pkt_read_ns;       // demuxer queued the packet
    int64_t  pkt_sent_ns;       // decoder accepted the packet
    int64_t  decoded_ns;        // decoder returned the frame (not finished)
    int64_t  ready_ns;          // frame left the decoder (after copy-back)
    int64_t  queued_ns;         // player queued it to the VO
    // When the writer began and finished the cue step: rendering the
    // subtitles (and the OSD message, if enabled), and copying the cues into
    // the slot. Present on every frame; short when nothing shows.
    int64_t  draw_start_ns;
    int64_t  draw_end_ns;

    struct vpsink_cue sub;      // the subtitle track's cue
    struct vpsink_cue msg;      // mpv's OSD message (vo-vpsink-osd), if any

    // R210 (Dolby Vision) frames only, 0 otherwise; same clock. The compose
    // runs between copy_start_ns and copy_end_ns: upload of the base layer
    // (and enhancement layer), the GPU's reshape, NLQ and r210 pack, then the
    // read-back into this slot. GPU durations come from GPU timers and lag a
    // few frames; 0 until the first one finishes.
    int64_t  dv_upload_end_ns;
    int64_t  dv_compose_end_ns;
    int64_t  dv_readback_end_ns;
    uint64_t dv_gpu_compose_ns; // reshape and NLQ pass
    uint64_t dv_gpu_pack_ns;    // 4:2:2 and r210 pass
    uint32_t dv_flags;          // VPSINK_DV_*
    uint32_t reserved1;
};

// vpsink_slot.dv_flags
#define VPSINK_DV_EL_COMPOSED       (1u << 0)   // FEL residual added (NLQ)

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

    // Dolby Vision (vo-vpsink-dv). A late frame is dropped, never waited
    // for: composing it would hold the VO thread while newer frames queue.
    volatile uint64_t dv_frames_composed;   // R210 frames published
    volatile uint64_t dv_frames_late;       // dropped: past their display time
    volatile uint64_t dv_compose_failed;    // dropped: GPU or format failure

    uint32_t reserved[26];
};

#endif
