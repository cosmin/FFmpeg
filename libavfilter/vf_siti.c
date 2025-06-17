/*
 * Copyright (c) 2021 Boris Baracaldo
 * Copyright (c) 2022 Thilo Borgmann
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * Calculate Spatial Info (SI) and Temporal Info (TI) scores
 */

#include <math.h>

#include "libavutil/imgutils.h"
#include "libavutil/internal.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"

#include "avfilter.h"
#include "filters.h"
#include "video.h"

static const int X_FILTER[9] = {1, 0, -1, 2, 0, -2, 1, 0, -1};

static const int Y_FILTER[9] = {1, 2, 1, 0, 0, 0, -1, -2, -1};

typedef enum { SDR = 0, HDR10, HLG } HdrMode;

typedef enum { UNSPECIFIED = 0, LIMITED = 1, FULL = 2 } ColorRange;

typedef enum { BT1886 = 0, INV_SRGB } EotfFunction;

typedef enum { PQ = 0, PU21 } CalculationDomain;

typedef enum { BANDING = 0, BANDING_GLARE, PEAKS, PEAKS_GLARE } Pu21Mode;

typedef struct SiTiContext {
    const AVClass *class;
    int pixel_depth;
    int width, height;
    uint64_t nb_frames;
    uint8_t *prev_frame;
    float *prev_frame_float; // For float-based TI calculation
    float max_si;
    float max_ti;
    float min_si;
    float min_ti;
    float sum_si;
    float sum_ti;
    float *gradient_matrix;
    float *motion_matrix;
    int full_range;
    int print_summary;

    // New parameters to match Python implementation
    HdrMode hdr_mode;
    int bit_depth;
    ColorRange color_range;
    EotfFunction eotf_function;
    CalculationDomain calculation_domain;
    float l_max;
    float l_min;
    float gamma;
    Pu21Mode pu21_mode;
} SiTiContext;

static const enum AVPixelFormat pix_fmts[] = {
    AV_PIX_FMT_YUV420P,  AV_PIX_FMT_YUV422P,   AV_PIX_FMT_YUVJ420P,
    AV_PIX_FMT_YUVJ422P, AV_PIX_FMT_YUV420P10, AV_PIX_FMT_YUV422P10,
    AV_PIX_FMT_NONE};

static av_cold int init(AVFilterContext *ctx)
{
    SiTiContext *s = ctx->priv;
    s->max_si = 0;
    s->max_ti = 0;

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    SiTiContext *s = ctx->priv;

    if (s->print_summary) {
        float avg_si = s->sum_si / s->nb_frames;
        float avg_ti = s->sum_ti / s->nb_frames;
        av_log(ctx, AV_LOG_INFO,
               "SITI Summary:\nTotal frames: %" PRId64 "\n\n"
               "Spatial Information:\nAverage: %f\nMax: %f\nMin: %f\n\n"
               "Temporal Information:\nAverage: %f\nMax: %f\nMin: %f\n",
               s->nb_frames, avg_si, s->max_si, s->min_si, avg_ti, s->max_ti,
               s->min_ti);
    }

    av_freep(&s->prev_frame);
    av_freep(&s->prev_frame_float);
    av_freep(&s->gradient_matrix);
    av_freep(&s->motion_matrix);
}

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    SiTiContext *s = ctx->priv;
    int max_pixsteps[4];
    size_t pixel_sz;
    size_t data_sz;
    size_t gradient_sz;
    size_t motion_sz;

    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(inlink->format);
    av_image_fill_max_pixsteps(max_pixsteps, NULL, desc);

    av_freep(&s->prev_frame);
    av_freep(&s->gradient_matrix);
    av_freep(&s->motion_matrix);

    s->pixel_depth = max_pixsteps[0];
    s->width = inlink->w;
    s->height = inlink->h;
    pixel_sz = s->pixel_depth == 1 ? sizeof(uint8_t) : sizeof(uint16_t);
    data_sz = s->width * pixel_sz * s->height;

    s->prev_frame = av_malloc(data_sz);
    gradient_sz = (s->width - 2) * sizeof(float) * (s->height - 2);
    s->gradient_matrix = av_malloc(gradient_sz);
    motion_sz = s->width * sizeof(float) * s->height;
    s->motion_matrix = av_malloc(motion_sz);

    if (!s->prev_frame || !s->gradient_matrix || !s->motion_matrix) {
        return AVERROR(ENOMEM);
    }

    return 0;
}

static int is_range_unspecified(AVFrame *frame)
{
    if (frame->format == AV_PIX_FMT_YUVJ420P ||
               frame->format == AV_PIX_FMT_YUVJ422P) {
        return 0;
    }
    return frame->color_range == AVCOL_RANGE_UNSPECIFIED;
}

static int is_full_range(AVFrame *frame)
{
    if (frame->color_range == AVCOL_RANGE_UNSPECIFIED ||
        frame->color_range == AVCOL_RANGE_NB)
        return frame->format == AV_PIX_FMT_YUVJ420P ||
               frame->format == AV_PIX_FMT_YUVJ422P;
    return frame->color_range == AVCOL_RANGE_JPEG;
}

// BT.1886 EOTF implementation
static float eotf_1886(float input, float gamma, float l_min, float l_max)
{
    if (input < 0.0f)
        input = 0.0f;
    if (input > 1.0f)
        input = 1.0f;

    float a =
        powf(powf(l_max, 1.0f / gamma) - powf(l_min, 1.0f / gamma), gamma);
    float b = powf(l_min, 1.0f / gamma) /
              (powf(l_max, 1.0f / gamma) - powf(l_min, 1.0f / gamma));

    return a * powf(fmaxf(input + b, 0.0f), gamma);
}

// Apply display model (EOTF + luminance mapping)
static float apply_display_model(SiTiContext *s, float input)
{
    if (input < 0.0f || input > 1.0f) {
        return 0.0f;
    }

    float linear = eotf_1886(input, s->gamma, 0.0f, 1.0f); // EOTF maps to [0,1]
    return (s->l_max - s->l_min) * linear +
           s->l_min; // Map to physical luminance
}

// PQ OETF implementation
static float oetf_pq(float input)
{
    float m = 78.84375f;
    float n = 0.1593017578125f;
    float c1 = 0.8359375f;
    float c2 = 18.8515625f;
    float c3 = 18.6875f;
    float lm1 = powf(10000.0f, n);
    float lm2 = powf(input, n);

    return powf((c1 * lm1 + c2 * lm2) / (lm1 + c3 * lm2), m);
}

// PU21 OETF implementation
static float oetf_pu21(SiTiContext *s, float input)
{
    float p[7];
    float p_min, p_max;

    switch (s->pu21_mode) {
    case BANDING:
        p[0] = 1.070275272f;
        p[1] = 0.4088273932f;
        p[2] = 0.153224308f;
        p[3] = 0.2520326168f;
        p[4] = 1.063512885f;
        p[5] = 1.14115047f;
        p[6] = 521.4527484f;
        p_min = -1.5580e-07f;
        p_max = 520.4673f;
        break;
    case BANDING_GLARE:
        p[0] = 0.353487901f;
        p[1] = 0.3734658629f;
        p[2] = 8.277049286e-05f;
        p[3] = 0.9062562627f;
        p[4] = 0.09150303166f;
        p[5] = 0.9099517204f;
        p[6] = 596.3148142f;
        p_min = 5.4705e-10f;
        p_max = 595.3939f;
        break;
    case PEAKS:
        p[0] = 1.043882782f;
        p[1] = 0.6459495343f;
        p[2] = 0.3194584211f;
        p[3] = 0.374025247f;
        p[4] = 1.114783422f;
        p[5] = 1.095360363f;
        p[6] = 384.9217577f;
        p_min = 1.3674e-07f;
        p_max = 380.9853f;
        break;
    case PEAKS_GLARE:
        p[0] = 816.885024f;
        p[1] = 1479.463946f;
        p[2] = 0.001253215609f;
        p[3] = 0.9329636822f;
        p[4] = 0.06746643971f;
        p[5] = 1.573435413f;
        p[6] = 419.6006374f;
        p_min = -9.7360e-08f;
        p_max = 407.5066f;
        break;
    default:
        return 0.0f;
    }

    float result = p[6] * (powf((p[0] + p[1] * powf(input, p[3])) /
                                    (1.0f + p[2] * powf(input, p[3])),
                                p[4]) -
                           p[5]);
    return (result - p_min) / (p_max - p_min);
}

static void convolve_sobel(SiTiContext *s, const uint8_t *src, float *dst,
                           int linesize)
{
    double x_conv_sum, y_conv_sum;
    float gradient;
    int ki, kj, index;
    uint16_t data;
    int filter_width = 3;
    int filter_size = filter_width * filter_width;
    int stride = linesize / s->pixel_depth;

#define CONVOLVE(bps)                                                          \
    {                                                                          \
        uint##bps##_t *vsrc = (uint##bps##_t *)src;                            \
        for (int j = 1; j < s->height - 1; j++) {                              \
            for (int i = 1; i < s->width - 1; i++) {                           \
                x_conv_sum = 0.0;                                              \
                y_conv_sum = 0.0;                                              \
                for (int k = 0; k < filter_size; k++) {                        \
                    ki = k % filter_width - 1;                                 \
                    kj = floor(k / filter_width) - 1;                          \
                    index = (j + kj) * stride + (i + ki);                      \
                    data = vsrc[index];                                        \
                    x_conv_sum += data * X_FILTER[k];                          \
                    y_conv_sum += data * Y_FILTER[k];                          \
                }                                                              \
                gradient =                                                     \
                    sqrt(x_conv_sum * x_conv_sum + y_conv_sum * y_conv_sum);   \
                dst[(j - 1) * (s->width - 2) + (i - 1)] = gradient;            \
            }                                                                  \
        }                                                                      \
    }

    if (s->pixel_depth == 2) {
        CONVOLVE(16);
    } else {
        CONVOLVE(8);
    }
}

static void calculate_motion(SiTiContext *s, const uint8_t *curr,
                             float *motion_matrix, int linesize)
{
    int stride = linesize / s->pixel_depth;
    float motion;
    int curr_index, prev_index;
    uint16_t curr_data;

#define CALCULATE(bps)                                                         \
    {                                                                          \
        uint##bps##_t *vsrc = (uint##bps##_t *)curr;                           \
        uint##bps##_t *vdst = (uint##bps##_t *)s->prev_frame;                  \
        for (int j = 0; j < s->height; j++) {                                  \
            for (int i = 0; i < s->width; i++) {                               \
                motion = 0;                                                    \
                curr_index = j * stride + i;                                   \
                prev_index = j * s->width + i;                                 \
                curr_data = vsrc[curr_index];                                  \
                if (s->nb_frames > 1)                                          \
                    motion = curr_data - vdst[prev_index];                     \
                vdst[prev_index] = curr_data;                                  \
                motion_matrix[j * s->width + i] = motion;                      \
            }                                                                  \
        }                                                                      \
    }

    if (s->pixel_depth == 2) {
        CALCULATE(16);
    } else {
        CALCULATE(8);
    }
}

static float std_deviation(float *img_metrics, int width, int height)
{
    int size = height * width;
    double mean = 0.0;
    double sqr_diff = 0;

    for (int j = 0; j < height; j++)
        for (int i = 0; i < width; i++)
            mean += img_metrics[j * width + i];

    mean /= size;

    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {
            float mean_diff = img_metrics[j * width + i] - mean;
            sqr_diff += (mean_diff * mean_diff);
        }
    }
    sqr_diff = sqr_diff / size;
    return sqrt(sqr_diff);
}

// Calculate SI directly on float data (like Python's scipy.ndimage.sobel)
static float calculate_si_float(float *frame_data, int width, int height)
{
    float *sob_x = av_malloc(width * height * sizeof(float));
    float *sob_y = av_malloc(width * height * sizeof(float));
    float *gradient = av_malloc((width - 2) * (height - 2) * sizeof(float));

    if (!sob_x || !sob_y || !gradient) {
        av_freep(&sob_x);
        av_freep(&sob_y);
        av_freep(&gradient);
        return 0.0f;
    }

    // Apply Sobel X filter
    for (int j = 1; j < height - 1; j++) {
        for (int i = 1; i < width - 1; i++) {
            float sum = 0.0f;
            for (int k = 0; k < 9; k++) {
                int ki = k % 3 - 1;
                int kj = k / 3 - 1;
                sum += frame_data[(j + kj) * width + (i + ki)] * X_FILTER[k];
            }
            sob_x[j * width + i] = sum;
        }
    }

    // Apply Sobel Y filter
    for (int j = 1; j < height - 1; j++) {
        for (int i = 1; i < width - 1; i++) {
            float sum = 0.0f;
            for (int k = 0; k < 9; k++) {
                int ki = k % 3 - 1;
                int kj = k / 3 - 1;
                sum += frame_data[(j + kj) * width + (i + ki)] * Y_FILTER[k];
            }
            sob_y[j * width + i] = sum;
        }
    }

    // Calculate gradient magnitude and crop to valid window [1:-1, 1:-1]
    for (int j = 1; j < height - 1; j++) {
        for (int i = 1; i < width - 1; i++) {
            float x_val = sob_x[j * width + i];
            float y_val = sob_y[j * width + i];
            gradient[(j - 1) * (width - 2) + (i - 1)] =
                sqrtf(x_val * x_val + y_val * y_val);
        }
    }

    // Calculate standard deviation
    float result = std_deviation(gradient, width - 2, height - 2);

    av_freep(&sob_x);
    av_freep(&sob_y);
    av_freep(&gradient);

    return result;
}

// Calculate TI directly on float data (like Python's numpy std)
static float calculate_ti_float(float *curr_frame, float *prev_frame, int width,
                                int height)
{
    if (!prev_frame)
        return 0.0f;

    int size = width * height;
    float *diff = av_malloc(size * sizeof(float));

    if (!diff)
        return 0.0f;

    // Calculate frame difference
    for (int i = 0; i < size; i++) {
        diff[i] = curr_frame[i] - prev_frame[i];
    }

    // Calculate standard deviation
    float result = std_deviation(diff, width, height);

    av_freep(&diff);
    return result;
}

static void set_meta(AVDictionary **metadata, const char *key, float d)
{
    char value[128];
    snprintf(value, sizeof(value), "%0.2f", d);
    av_dict_set(metadata, key, value, 0);
}

static int filter_frame(AVFilterLink *inlink, AVFrame *frame)
{
    AVFilterContext *ctx = inlink->dst;
    SiTiContext *s = ctx->priv;
    float si, ti;
    float *normalized_frame = NULL;
    float *transformed_frame = NULL;

    // Determine color range from frame
    int frame_is_full_range = is_full_range(frame);

    // Handle color range determination and warnings
    if (s->color_range == UNSPECIFIED) {
        // Use frame-determined range
        s->full_range = frame_is_full_range;
    } else {
        // User specified a range, check for mismatch and warn
        int user_wants_full = (s->color_range == FULL);
        if (user_wants_full != frame_is_full_range && !is_range_unspecified(frame)) {
            const char *frame_range_str =
                frame_is_full_range ? "full" : "limited";
            const char *user_range_str = user_wants_full ? "full" : "limited";
            av_log(ctx, AV_LOG_WARNING,
                   "Color range mismatch: frame indicates %s range but user "
                   "specified %s range. Using user specification.\n",
                   frame_range_str, user_range_str);
        }
        s->full_range = user_wants_full;
    }

    s->nb_frames++;

    int frame_size = s->width * s->height;
    normalized_frame = av_malloc(frame_size * sizeof(float));
    transformed_frame = av_malloc(frame_size * sizeof(float));

    if (!normalized_frame || !transformed_frame) {
        av_freep(&normalized_frame);
        av_freep(&transformed_frame);
        return AVERROR(ENOMEM);
    }

    // Step 1: Log original frame data
    if (s->nb_frames == 1) {
        float *orig_data = av_malloc(frame_size * sizeof(float));
        if (orig_data) {
            for (int i = 0; i < frame_size; i++) {
                orig_data[i] = ((uint8_t *)frame->data[0])[i];
            }
            av_freep(&orig_data);
        }
    }

    // Step 2: Normalize frame data between 0 and 1 (bit depth normalization)
    for (int i = 0; i < frame_size; i++) {
        normalized_frame[i] = ((uint8_t *)frame->data[0])[i] / 255.0f;
    }

    // Step 3: Handle color range conversion (limited to full range if needed)
    if (s->color_range == FULL || s->full_range) {
        // Frame data (full range, no conversion needed)
    } else {
        // Convert from limited range [16/255, 235/255] to full range [0, 1]
        const float limited_min = 16.0f / 255.0f;
        const float limited_max = 235.0f / 255.0f;
        const float limited_range = limited_max - limited_min;

        for (int i = 0; i < frame_size; i++) {
            float val = normalized_frame[i];
            val = fmaxf(fminf(val, limited_max), limited_min);
            normalized_frame[i] = (val - limited_min) / limited_range;
        }
    }

    // Step 4: Apply display model (EOTF) - maps [0,1] to physical luminance
    for (int i = 0; i < frame_size; i++) {
        transformed_frame[i] = apply_display_model(s, normalized_frame[i]);
    }

    // Step 5: Apply OETF (PQ or PU21) - maps physical luminance to perceptual
    // domain
    if (s->calculation_domain == PQ) {
        for (int i = 0; i < frame_size; i++) {
            transformed_frame[i] = oetf_pq(transformed_frame[i]);
        }
    } else {
        for (int i = 0; i < frame_size; i++) {
            transformed_frame[i] = oetf_pu21(s, transformed_frame[i]);
        }
    }

    // Step 6: Calculate SI and TI directly on the transformed float data (like
    // Python does)
    float si_raw = calculate_si_float(transformed_frame, s->width, s->height);
    float ti_raw = 0.0f;

    if (s->nb_frames > 1) {
        ti_raw = calculate_ti_float(transformed_frame, s->prev_frame_float,
                                    s->width, s->height);
    }

    // Store current frame for next TI calculation
    if (!s->prev_frame_float) {
        s->prev_frame_float = av_malloc(frame_size * sizeof(float));
    }
    if (s->prev_frame_float) {
        memcpy(s->prev_frame_float, transformed_frame,
               frame_size * sizeof(float));
    }

    // Step 7: Normalize SI/TI back to [0, 255] range (like Python does)
    si = si_raw * 255.0f;
    ti = ti_raw * 255.0f;

    // Convert transformed data back to uint8_t format for compatibility (but
    // don't use for SI/TI calc)
    for (int i = 0; i < frame_size; i++) {
        ((uint8_t *)frame->data[0])[i] =
            (uint8_t)(transformed_frame[i] * 255.0f);
    }

    // Print SI and TI values for each frame
    av_log(ctx, AV_LOG_INFO, "Frame %" PRId64 " - SI: %.6f, TI: %.6f\n",
           s->nb_frames, si, ti);

    // Calculate statistics
    s->max_si = fmaxf(si, s->max_si);
    s->max_ti = fmaxf(ti, s->max_ti);
    s->sum_si += si;
    s->sum_ti += ti;
    s->min_si = s->nb_frames == 1 ? si : fminf(si, s->min_si);
    s->min_ti = s->nb_frames == 1 ? ti : fminf(ti, s->min_ti);

    // Set si ti information in frame metadata
    set_meta(&frame->metadata, "lavfi.siti.si", si);
    set_meta(&frame->metadata, "lavfi.siti.ti", ti);

    // Clean up
    av_freep(&normalized_frame);
    av_freep(&transformed_frame);

    return ff_filter_frame(inlink->dst->outputs[0], frame);
}

#define OFFSET(x) offsetof(SiTiContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_FILTERING_PARAM

static const AVOption siti_options[] = {
    {"hdr_mode",
     "HDR mode (SDR, HDR10, HLG)",
     OFFSET(hdr_mode),
     AV_OPT_TYPE_INT,
     {.i64 = 0},
     0,
     2,
     FLAGS},
    {"color_range",
     "Color range (unspecified, limited, full)",
     OFFSET(color_range),
     AV_OPT_TYPE_INT,
     {.i64 = 0},
     0,
     2,
     FLAGS},
    {"eotf_function",
     "EOTF function (BT.1886, Inverse sRGB)",
     OFFSET(eotf_function),
     AV_OPT_TYPE_INT,
     {.i64 = 0},
     0,
     1,
     FLAGS},
    {"calculation_domain",
     "Calculation domain (PQ, PU21)",
     OFFSET(calculation_domain),
     AV_OPT_TYPE_INT,
     {.i64 = 0},
     0,
     1,
     FLAGS},
    {"l_max",
     "Maximum luminance",
     OFFSET(l_max),
     AV_OPT_TYPE_FLOAT,
     {.dbl = 300.0},
     0,
     10000,
     FLAGS},
    {"l_min",
     "Minimum luminance",
     OFFSET(l_min),
     AV_OPT_TYPE_FLOAT,
     {.dbl = 0.1},
     0,
     1000,
     FLAGS},
    {"gamma",
     "Gamma value for BT.1886",
     OFFSET(gamma),
     AV_OPT_TYPE_FLOAT,
     {.dbl = 2.4},
     1,
     3,
     FLAGS},
    {"pu21_mode",
     "PU21 mode (BANDING, BANDING_GLARE, PEAKS, PEAKS_GLARE)",
     OFFSET(pu21_mode),
     AV_OPT_TYPE_INT,
     {.i64 = 0},
     0,
     3,
     FLAGS},
    {"print_summary",
     "Print summary showing average values",
     OFFSET(print_summary),
     AV_OPT_TYPE_BOOL,
     {.i64 = 0},
     0,
     1,
     FLAGS},
    {NULL}};

AVFILTER_DEFINE_CLASS(siti);

static const AVFilterPad avfilter_vf_siti_inputs[] = {
    {
        .name = "default",
        .type = AVMEDIA_TYPE_VIDEO,
        .config_props = config_input,
        .filter_frame = filter_frame,
    },
};

const FFFilter ff_vf_siti = {
    .p.name        = "siti",
    .p.description = NULL_IF_CONFIG_SMALL("Calculate spatial information (SI) and temporal information (TI)."),
    .p.priv_class  = &siti_class,
    .p.flags       = AVFILTER_FLAG_METADATA_ONLY,
    .priv_size     = sizeof(SiTiContext),
    .init          = init,
    .uninit        = uninit,
    FILTER_PIXFMTS_ARRAY(pix_fmts),
    FILTER_INPUTS(avfilter_vf_siti_inputs),
    FILTER_OUTPUTS(ff_video_default_filterpad),
};
