/*
 * vpsink_dv: compose a Dolby Vision frame into a TV-led tunnel picture.
 * See vpsink_dv.h.
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

#include <assert.h>
#include <math.h>

#include <libplacebo/config.h>

#include "common/common.h"
#include "common/msg.h"
#include "mpv_talloc.h"
#include "osdep/timer.h"
#include "video/img_format.h"
#include "video/mp_image.h"
#include "vpsink_dv.h"

// Stock libplacebo calls only (PLAN.md "Two libplacebo copies"): the public
// reshape, the public NLQ fields, the D3D11 backend.
#if PL_HAVE_D3D11 && defined(PL_HAVE_DOVI) && PL_API_VER >= 367
#define HAVE_COMPOSER 1
#include <libplacebo/d3d11.h>
#include <libplacebo/dispatch.h>
#include <libplacebo/shaders/colorspace.h>
#include <libplacebo/shaders/custom.h>
#include "video/out/placebo/utils.h"
#else
#define HAVE_COMPOSER 0
#endif

#define MAX_PLANES 3

#if HAVE_COMPOSER

struct layer {
    pl_tex tex[MAX_PLANES];
};

struct vpsink_dv {
    struct mp_log *log;
    pl_log pllog;
    pl_d3d11 d3d11;
    pl_gpu gpu;
    pl_dispatch dp;
    pl_timer tm_compose, tm_pack;
    pl_fmt vfmt;                // vec2 vertex attribute
    bool init_tried, init_ok;
    bool warned;                // a compose failure was logged

    struct layer bl, el;
    pl_tex composed;            // 4:4:4 composed signal, rgba16
    pl_tex packed;              // BGRA8 bytes in rgba8 (byte order = memory)
    uint64_t gpu_compose_ns, gpu_pack_ns;
};

// One plane as the shader samples it: texel = (pos - shift) * ratio, with pos
// in output pixels.
struct plane_map {
    float a[2], b[2];           // normalised: coord = pos * a + b
};

// libplacebo rounds layer and plane ratios to whole factors (renderer.c,
// sample_el); do the same so sub-sampled planes line up identically.
static float round_ratio(float r)
{
    return r >= 1 ? roundf(r) : 1.0f / roundf(1.0f / r);
}

static void map_plane(struct plane_map *m, int pw, int ph, float rx, float ry,
                      float sx, float sy)
{
    m->a[0] = rx / pw;
    m->a[1] = ry / ph;
    m->b[0] = -sx * rx / pw;
    m->b[1] = -sy * ry / ph;
}

static bool init_gpu(struct vpsink_dv *dv)
{
    if (dv->init_tried)
        return dv->init_ok;
    dv->init_tried = true;

    dv->pllog = mppl_log_create(dv, dv->log);
    dv->d3d11 = pl_d3d11_create(dv->pllog, pl_d3d11_params());
    if (!dv->d3d11) {
        MP_ERR(dv, "Cannot create a D3D11 GPU for Dolby Vision composing.\n");
        return false;
    }
    dv->gpu = dv->d3d11->gpu;
    dv->dp = pl_dispatch_create(dv->pllog, dv->gpu);
    dv->vfmt = pl_find_vertex_fmt(dv->gpu, PL_FMT_FLOAT, 2);
    if (!dv->dp || !dv->vfmt) {
        MP_ERR(dv, "The D3D11 GPU cannot run the Dolby Vision composer.\n");
        return false;
    }
    // NULL where the GPU has no timer queries; the stage times then read 0.
    dv->tm_compose = pl_timer_create(dv->gpu);
    dv->tm_pack = pl_timer_create(dv->gpu);
    dv->init_ok = true;
    MP_VERBOSE(dv, "Dolby Vision composer on a D3D11 GPU (libplacebo %s).\n",
               PL_VERSION);
    return true;
}

static void free_layer(pl_gpu gpu, struct layer *l)
{
    for (int i = 0; i < MAX_PLANES; i++)
        pl_tex_destroy(gpu, &l->tex[i]);
}

void vpsink_dv_destroy(struct vpsink_dv *dv)
{
    if (!dv)
        return;
    if (dv->gpu) {
        free_layer(dv->gpu, &dv->bl);
        free_layer(dv->gpu, &dv->el);
        pl_tex_destroy(dv->gpu, &dv->composed);
        pl_tex_destroy(dv->gpu, &dv->packed);
        pl_timer_destroy(dv->gpu, &dv->tm_compose);
        pl_timer_destroy(dv->gpu, &dv->tm_pack);
    }
    pl_dispatch_destroy(&dv->dp);
    pl_d3d11_destroy(&dv->d3d11);
    pl_log_destroy(&dv->pllog);
    talloc_free(dv);
}

struct vpsink_dv *vpsink_dv_create(void *ta_parent, struct mp_log *log)
{
    struct vpsink_dv *dv = talloc_zero(ta_parent, struct vpsink_dv);
    dv->log = log;
    return dv;
}

// What a plane holds and how to normalise its samples to the format's code
// range (code / (2^depth - 1)), as pl_color_repr_normalize does.
struct plane_fmt {
    int num_planes;
    int comps[MAX_PLANES];      // components per plane
    int ids[MAX_PLANES][2];     // 1 = Y, 2 = Cb, 3 = Cr (mp_regular_imgfmt)
    int bytes;                  // per sample
    float scale;                // texture value -> normalised code
};

static bool get_plane_fmt(struct plane_fmt *f, int imgfmt)
{
    struct mp_regular_imgfmt reg;
    if (!mp_get_regular_imgfmt(&reg, imgfmt) ||
        reg.component_type != MP_COMPONENT_TYPE_UINT ||
        (reg.component_size != 1 && reg.component_size != 2) ||
        reg.num_planes < 2 || reg.num_planes > MAX_PLANES ||
        reg.chroma_xs != 1 || reg.chroma_ys != 1)
        return false;
    *f = (struct plane_fmt){ .num_planes = reg.num_planes,
                             .bytes = reg.component_size };
    for (int i = 0; i < reg.num_planes; i++) {
        int n = reg.planes[i].num_components;
        if (n < 1 || n > 2)
            return false;
        f->comps[i] = n;
        for (int c = 0; c < n; c++) {
            int id = reg.planes[i].components[c];
            if (id < 1 || id > 3)
                return false;
            f->ids[i][c] = id;
        }
    }
    int bits = reg.component_size * 8;
    int lsb_pad = MPMAX(reg.component_pad, 0);      // >0: LSB padding (P010)
    int depth = bits - abs(reg.component_pad);
    f->scale = (float)((1 << bits) - 1) / (1 << lsb_pad) / ((1 << depth) - 1);
    return true;
}

static bool upload_layer(struct vpsink_dv *dv, struct layer *l,
                         struct mp_image *img, const struct plane_fmt *f)
{
    for (int i = 0; i < f->num_planes; i++) {
        const char *name = f->comps[i] == 1 ? (f->bytes == 1 ? "r8" : "r16")
                                            : (f->bytes == 1 ? "rg8" : "rg16");
        pl_fmt fmt = pl_find_named_fmt(dv->gpu, name);
        if (!fmt || !(fmt->caps & PL_FMT_CAP_LINEAR))
            return false;
        bool ok = pl_tex_recreate(dv->gpu, &l->tex[i], &(struct pl_tex_params) {
            .w = mp_image_plane_w(img, i),
            .h = mp_image_plane_h(img, i),
            .format = fmt,
            .sampleable = true,
            .host_writable = true,
        });
        ok = ok && pl_tex_upload(dv->gpu, &(struct pl_tex_transfer_params) {
            .tex = l->tex[i],
            .ptr = img->planes[i],
            .row_pitch = img->stride[i],
        });
        if (!ok)
            return false;
    }
    return true;
}

// Append to sh one stage sampling every plane of a layer: bilinear, with the
// chroma siting and (for the EL) the layer siting folded into each plane's
// coordinates. Writes components 1-3 to <out>[0..2], normalised; pre and
// post are GLSL around the sampling, extra the variables post uses.
static bool sample_layer(struct vpsink_dv *dv, pl_shader sh, void *ta,
                         const char *prefix, struct layer *l, struct mp_image *img,
                         const struct plane_fmt *f, int out_w, int out_h,
                         float layer_x, float layer_y, float rlx, float rly,
                         enum pl_shader_sig input,
                         const struct pl_shader_var *extra, int num_extra,
                         const char *pre, const char *out, const char *post)
{
    struct pl_shader_desc descs[MAX_PLANES];
    struct pl_shader_var vars[2 * MAX_PLANES + 1 + 3];
    struct plane_map maps[MAX_PLANES];
    int nv = 0;
    assert(num_extra <= 3);
    for (int i = 0; i < num_extra; i++)
        vars[nv++] = extra[i];

    float cx = 0, cy = 0;
    pl_chroma_location_offset(img->params.chroma_location, &cx, &cy);

    int ref_w = mp_image_plane_w(img, 0), ref_h = mp_image_plane_h(img, 0);
    char *pos = talloc_asprintf(ta, "pos_%s", prefix);
    char *body = talloc_strdup(ta, pre);
    for (int i = 0; i < f->num_planes; i++) {
        int pw = l->tex[i]->params.w, ph = l->tex[i]->params.h;
        float rx = rlx * round_ratio((float)pw / ref_w),
              ry = rly * round_ratio((float)ph / ref_h);
        // Chroma planes carry the siting shift (pl_frame_set_chroma_location).
        bool chroma = pw < ref_w || ph < ref_h;
        float sx = layer_x + (chroma ? cx : 0) / rlx,
              sy = layer_y + (chroma ? cy : 0) / rly;
        map_plane(&maps[i], pw, ph, rx, ry, sx, sy);

        char *tex = talloc_asprintf(ta, "%s%d", prefix, i);
        char *a = talloc_asprintf(ta, "%s%d_a", prefix, i);
        char *b = talloc_asprintf(ta, "%s%d_b", prefix, i);
        descs[i] = (struct pl_shader_desc) {
            .desc = { .name = tex, .type = PL_DESC_SAMPLED_TEX },
            .binding = {
                .object = l->tex[i],
                .address_mode = PL_TEX_ADDRESS_CLAMP,
                .sample_mode = PL_TEX_SAMPLE_LINEAR,
            },
        };
        vars[nv++] = (struct pl_shader_var) { pl_var_vec2(a), maps[i].a };
        vars[nv++] = (struct pl_shader_var) { pl_var_vec2(b), maps[i].b };

        body = talloc_asprintf_append(body,
            "{ vec4 t = texture(%s, %s * %s + %s);\n", tex, pos, a, b);
        for (int c = 0; c < f->comps[i]; c++) {
            body = talloc_asprintf_append(body, "  %s[%d] = t[%d] * %s_scale;\n",
                                          out, f->ids[i][c] - 1, c, prefix);
        }
        body = talloc_strdup_append(body, "}\n");
    }
    body = talloc_strdup_append(body, post);

    char *scale = talloc_asprintf(ta, "%s_scale", prefix);
    vars[nv++] = (struct pl_shader_var) { pl_var_float(scale), &f->scale };

    float corners[4][2] = {{0, 0}, {out_w, 0}, {0, out_h}, {out_w, out_h}};
    struct pl_shader_va va = {
        .attr = { .name = pos, .fmt = dv->vfmt },
        .data = { corners[0], corners[1], corners[2], corners[3] },
    };

    return pl_shader_custom(sh, &(struct pl_custom_shader) {
        .description = prefix,
        .body = body,
        .input = input,
        .output = PL_SHADER_SIG_COLOR,
        .descriptors = descs,
        .num_descriptors = f->num_planes,
        .variables = vars,
        .num_variables = nv,
        .vertex_attribs = &va,
        .num_vertex_attribs = 1,
    });
}

// Reshape and NLQ: the composed signal after the RPU, before its nonlinear
// matrix, as PLAN.md "FEL composer" (b) sets out.
static bool compose_pass(struct vpsink_dv *dv, void *ta, struct mp_image *img,
                         const struct plane_fmt *blf, struct mp_image *el,
                         const struct plane_fmt *elf)
{
    const struct pl_dovi_metadata *dovi = img->params.repr.dovi;
    int w = img->w, h = img->h;

    pl_fmt fmt = pl_find_named_fmt(dv->gpu, "rgba16");
    if (!fmt || !(fmt->caps & PL_FMT_CAP_RENDERABLE))
        return false;
    bool ok = pl_tex_recreate(dv->gpu, &dv->composed, &(struct pl_tex_params) {
        .w = w,
        .h = h,
        .format = fmt,
        .sampleable = true,
        .renderable = true,
    });
    if (!ok)
        return false;

    pl_shader sh = pl_dispatch_begin(dv->dp);
    ok = sample_layer(dv, sh, ta, "bl", &dv->bl, img, blf, w, h, 0, 0, 1, 1,
                      PL_SHADER_SIG_NONE, NULL, 0, "color.a = 1.0;\n", "color", "");
    if (ok)
        pl_shader_dovi_reshape(sh, dovi);

    if (ok && el) {
        // libplacebo's EL layer siting (renderer.c, sample_el): an EL
        // smaller than the BL is left-sited on it.
        int elw = mp_image_plane_w(el, 0), elh = mp_image_plane_h(el, 0);
        float lx = 0, ly = 0, ox, oy;
        pl_chroma_location_offset(PL_CHROMA_LEFT, &ox, &oy);
        if (elw < w)
            lx = ox;
        if (elh < h)
            ly = oy;
        float rlx = round_ratio((float)elw / w), rly = round_ratio((float)elh / h);

        float off[3], slope[3], thr[3];
        for (int c = 0; c < 3; c++) {
            off[c] = dovi->nlq[c].offset;
            slope[c] = dovi->nlq[c].deadzone_slope;
            thr[c] = dovi->nlq[c].deadzone_threshold;
        }
        // LINEAR_DZ dequantisation from the public nlq fields, as libplacebo's
        // own FEL composer applies it.
        struct pl_shader_var nlq[3] = {
            { pl_var_vec3("nlq_off"), off },
            { pl_var_vec3("nlq_slope"), slope },
            { pl_var_vec3("nlq_thr"), thr },
        };
        ok = sample_layer(dv, sh, ta, "el", &dv->el, el, elf, w, h, lx, ly,
                          rlx, rly, PL_SHADER_SIG_COLOR, nlq, 3,
                          "vec3 el = vec3(0.0);\n", "el",
                          "vec3 elc = el - nlq_off;\n"
                          "color.rgb += sign(elc) * (abs(elc) * nlq_slope + nlq_thr);\n");
    }

    if (!ok) {
        pl_dispatch_abort(dv->dp, &sh);
        return false;
    }
    return pl_dispatch_finish(dv->dp, &(struct pl_dispatch_params) {
        .shader = &sh,
        .target = dv->composed,
        .timer = dv->tm_compose,
    });
}

// 4:2:2 and the BGRA8 layout. Each output pixel carries its own 12-bit I and
// one 12-bit chroma sample of its pair (even: P, odd: T), filtered [1 2 1]
// around the even pixel (co-sited, as HDMI 4:2:2). In the RGB8 tunnel
// (HDMI 1.4a 6.5.4, as VP's ReadDolbyVisionTunnelWord reads it):
// G = I[11:4], R = C[11:4], B = C[3:0] I[3:0], written to memory as B, G, R
// and an opaque A, which VP ignores.
static bool pack_pass(struct vpsink_dv *dv, int w, int h)
{
    pl_fmt fmt = pl_find_named_fmt(dv->gpu, "rgba8");
    if (!fmt || !(fmt->caps & PL_FMT_CAP_RENDERABLE) ||
        !(fmt->caps & PL_FMT_CAP_HOST_READABLE))
        return false;
    bool ok = pl_tex_recreate(dv->gpu, &dv->packed, &(struct pl_tex_params) {
        .w = w,
        .h = h,
        .format = fmt,
        .renderable = true,
        .host_readable = true,
    });
    if (!ok)
        return false;

    int last = w - 1;
    float corners[4][2] = {{0, 0}, {w, 0}, {0, h}, {w, h}};
    struct pl_shader_va va = {
        .attr = { .name = "pos_pk", .fmt = dv->vfmt },
        .data = { corners[0], corners[1], corners[2], corners[3] },
    };
    struct pl_shader_desc desc = {
        .desc = { .name = "comp", .type = PL_DESC_SAMPLED_TEX },
        .binding = {
            .object = dv->composed,
            .address_mode = PL_TEX_ADDRESS_CLAMP,
            .sample_mode = PL_TEX_SAMPLE_NEAREST,
        },
    };
    struct pl_shader_var var = { pl_var_int("pk_last"), &last };

    pl_shader sh = pl_dispatch_begin(dv->dp);
    ok = pl_shader_custom(sh, &(struct pl_custom_shader) {
        .description = "tunnel pack",
        .body =
            "ivec2 p = ivec2(pos_pk);\n"
            "int e = p.x - (p.x & 1);\n"
            "vec4 c = texelFetch(comp, p, 0);\n"
            "vec4 ch = 0.25 * (texelFetch(comp, ivec2(max(e - 1, 0), p.y), 0) +\n"
            "                  2.0 * texelFetch(comp, ivec2(e, p.y), 0) +\n"
            "                  texelFetch(comp, ivec2(min(e + 1, pk_last), p.y), 0));\n"
            "float cv = (p.x & 1) == 0 ? ch.g : ch.b;\n"
            "uint I = uint(clamp(c.r, 0.0, 1.0) * 4095.0 + 0.5);\n"
            "uint C = uint(clamp(cv, 0.0, 1.0) * 4095.0 + 0.5);\n"
            "uint r = C >> 4u, g = I >> 4u;\n"
            "uint b = ((C & 15u) << 4u) | (I & 15u);\n"
            "color = vec4(float(b), float(g), float(r), 255.0) / 255.0;\n",
        .input = PL_SHADER_SIG_NONE,
        .output = PL_SHADER_SIG_COLOR,
        .descriptors = &desc,
        .num_descriptors = 1,
        .variables = &var,
        .num_variables = 1,
        .vertex_attribs = &va,
        .num_vertex_attribs = 1,
    });
    if (!ok) {
        pl_dispatch_abort(dv->dp, &sh);
        return false;
    }
    return pl_dispatch_finish(dv->dp, &(struct pl_dispatch_params) {
        .shader = &sh,
        .target = dv->packed,
        .timer = dv->tm_pack,
    });
}

static uint64_t newest_time(pl_gpu gpu, pl_timer t, uint64_t prev)
{
    uint64_t ns;
    while (t && (ns = pl_timer_query(gpu, t)))
        prev = ns;
    return prev;
}

bool vpsink_dv_frame_ok(struct mp_image *img)
{
    return img && img->params.repr.sys == PL_COLOR_SYSTEM_DOLBYVISION &&
           img->params.repr.dovi && img->imgfmt == IMGFMT_P010 &&
           img->w > 0 && !(img->w & 1);
}

bool vpsink_dv_compose(struct vpsink_dv *dv, struct mp_image *img,
                       uint8_t *dst, struct vpsink_dv_times *times)
{
    *times = (struct vpsink_dv_times){0};
    if (!vpsink_dv_frame_ok(img) || !init_gpu(dv))
        return false;

    const struct pl_dovi_metadata *dovi = img->params.repr.dovi;
    struct plane_fmt blf, elf;
    if (!get_plane_fmt(&blf, img->imgfmt))
        return false;

    // The EL matters only when the RPU's NLQ is active (7 FEL); MEL, 8.1 and
    // profile 5 are reshape only (PLAN.md "FEL composer" (6)).
    struct mp_image *el = dovi->nlq_active ? img->enhancement_layer : NULL;
    if (el && !get_plane_fmt(&elf, el->imgfmt)) {
        if (!dv->warned)
            MP_WARN(dv, "Enhancement layer in %s cannot be composed; "
                    "reshaping the base layer only.\n", mp_imgfmt_to_name(el->imgfmt));
        dv->warned = true;
        el = NULL;
    }

    void *ta = talloc_new(NULL);
    bool ok = upload_layer(dv, &dv->bl, img, &blf) &&
              (!el || upload_layer(dv, &dv->el, el, &elf));
    times->upload_end_ns = mp_raw_time_ns();

    ok = ok && compose_pass(dv, ta, img, &blf, el, el ? &elf : NULL) &&
         pack_pass(dv, img->w, img->h);
    talloc_free(ta);
    if (ok) {
        pl_gpu_finish(dv->gpu);
        times->compose_end_ns = mp_raw_time_ns();
        ok = pl_tex_download(dv->gpu, &(struct pl_tex_transfer_params) {
            .tex = dv->packed,
            .ptr = dst,
            .row_pitch = VPSINK_DV_BGRA_STRIDE(img->w),
        });
        times->readback_end_ns = mp_raw_time_ns();
    }

    dv->gpu_compose_ns = newest_time(dv->gpu, dv->tm_compose, dv->gpu_compose_ns);
    dv->gpu_pack_ns = newest_time(dv->gpu, dv->tm_pack, dv->gpu_pack_ns);
    times->gpu_compose_ns = dv->gpu_compose_ns;
    times->gpu_pack_ns = dv->gpu_pack_ns;
    times->el_composed = ok && el;

    if (!ok && !dv->warned) {
        MP_ERR(dv, "Dolby Vision compose failed; the frame is dropped.\n");
        dv->warned = true;
    }
    return ok;
}

#else // HAVE_COMPOSER

struct vpsink_dv { struct mp_log *log; };

struct vpsink_dv *vpsink_dv_create(void *ta_parent, struct mp_log *log)
{
    struct vpsink_dv *dv = talloc_zero(ta_parent, struct vpsink_dv);
    dv->log = log;
    MP_WARN(dv, "Built without the D3D11 libplacebo Dolby Vision composer.\n");
    return dv;
}

void vpsink_dv_destroy(struct vpsink_dv *dv)
{
    talloc_free(dv);
}

bool vpsink_dv_frame_ok(struct mp_image *img)
{
    return false;
}

bool vpsink_dv_compose(struct vpsink_dv *dv, struct mp_image *img,
                       uint8_t *dst, struct vpsink_dv_times *times)
{
    return false;
}

#endif
