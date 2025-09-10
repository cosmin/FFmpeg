/*
 * Copyright (c) 2021 Boris Baracaldo
 * Copyright (c) 2022 Thilo Borgmann
 * Copyright (c) 2025 Cosmin Stejerean
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
 * Calculate Spatial Information (SI) and Temporal Information (TI) scores.
 *
 * Implements ITU-T Rec. P.910 Annex B spatial and temporal information metrics
 * for video quality assessment.
 */

#include <errno.h>
#include <float.h>
#include <math.h>

#include "libavutil/imgutils.h"
#include "libavutil/internal.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/avstring.h"
#include "libavutil/bprint.h"
#include "libavutil/file_open.h"

#include "avfilter.h"
#include "filters.h"
#include "video.h"

static const int X_FILTER[9] = { 1,  0, -1,  2, 0, -2,  1,  0, -1};
static const int Y_FILTER[9] = { 1,  2,  1,  0, 0,  0, -1, -2, -1};

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

    // New parameters to match updated P.910 standard with HDR support
    HdrMode hdr_mode;
    int bit_depth;
    ColorRange color_range;
    EotfFunction eotf_function;
    CalculationDomain calculation_domain;
    float l_max;
    float l_min;
    float gamma;
    Pu21Mode pu21_mode;
    int legacy;               // Legacy mode flag

    // JSON output support
    char *stats_file_str;
    FILE *stats_file;
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
    s->min_si = FLT_MAX;
    s->min_ti = FLT_MAX;

    // Initialize JSON output if stats file is specified
    if (s->stats_file_str) {
        if (!strcmp(s->stats_file_str, "-")) {
            s->stats_file = stdout;
        } else {
            s->stats_file = avpriv_fopen_utf8(s->stats_file_str, "w");
            if (!s->stats_file) {
                int err = AVERROR(errno);
                av_log(ctx, AV_LOG_ERROR, "Could not open stats file %s: %s\n",
                       s->stats_file_str, av_err2str(err));
                return err;
            }
        }
    }

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    SiTiContext *s = ctx->priv;

    // Always print summary if we have frames
    if (s->nb_frames > 0) {
        float avg_si = s->sum_si / s->nb_frames;
        float avg_ti = s->sum_ti / (s->nb_frames - 1);
        av_log(ctx, AV_LOG_INFO,
               "SITI Summary:\nTotal frames: %" PRId64 "\n\n"
               "Spatial Information:\nAverage: %f\nMax: %f\nMin: %f\n\n"
               "Temporal Information:\nAverage: %f\nMax: %f\nMin: %f\n",
               s->nb_frames, avg_si, s->max_si, s->min_si, avg_ti, s->max_ti,
               s->min_ti);

        // Write JSON summary line if output is enabled
        if (s->stats_file) {
            fprintf(s->stats_file, "{\"summary\":{\"frames\":%" PRId64 ",\"si\":{\"min\":%.15f,\"max\":%.15f,\"avg\":%.15f},\"ti\":{\"min\":%.15f,\"max\":%.15f,\"avg\":%.15f}}}\n",
                    s->nb_frames, s->min_si, s->max_si, avg_si, s->min_ti, s->max_ti, avg_ti);
        }
    }

    // Close JSON file if it was opened
    if (s->stats_file && s->stats_file != stdout) {
        fclose(s->stats_file);
    }

    av_freep(&s->prev_frame_float);
    av_freep(&s->gradient_matrix);
    av_freep(&s->motion_matrix);
}

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    SiTiContext *s = ctx->priv;
    int max_pixsteps[4];
    size_t gradient_sz;
    size_t motion_sz;

    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(inlink->format);
    av_image_fill_max_pixsteps(max_pixsteps, NULL, desc);

    av_freep(&s->gradient_matrix);
    av_freep(&s->motion_matrix);

    s->pixel_depth = max_pixsteps[0];
    s->width = inlink->w;
    s->height = inlink->h;
    s->bit_depth = desc->comp[0].depth;  // Set bit depth from format descriptor

    // Check legacy mode constraints
    if (s->legacy && s->bit_depth != 8) {
        av_log(ctx, AV_LOG_ERROR, "Legacy mode only supports 8-bit input\n");
        return AVERROR(EINVAL);
    }

    gradient_sz = (s->width - 2) * sizeof(float) * (s->height - 2);
    s->gradient_matrix = av_malloc(gradient_sz);
    motion_sz = s->width * sizeof(float) * s->height;
    s->motion_matrix = av_malloc(motion_sz);

    if (!s->gradient_matrix || !s->motion_matrix) {
        return AVERROR(ENOMEM);
    }

    // Write JSON settings line if output is enabled
    if (s->stats_file) {
        // Convert enum values to strings for JSON output
        const char *calc_domain_str = s->calculation_domain == PQ ? "pq" : "pu21";
        const char *hdr_mode_str = s->hdr_mode == SDR ? "sdr" :
                                   (s->hdr_mode == HDR10 ? "hdr10" : "hlg");
        const char *color_range_str = s->color_range == FULL ? "full" :
                                      (s->color_range == LIMITED ? "limited" : "unspecified");
        const char *eotf_str = s->eotf_function == BT1886 ? "bt1886" : "inv_srgb";
        const char *pu21_mode_str = s->pu21_mode == BANDING ? "banding" :
                                    s->pu21_mode == BANDING_GLARE ? "banding_glare" :
                                    s->pu21_mode == PEAKS ? "peaks" : "peaks_glare";

        fprintf(s->stats_file, "{\"settings\":{\"calculation_domain\":\"%s\",\"hdr_mode\":\"%s\",\"bit_depth\":%d,\"color_range\":\"%s\",\"eotf_function\":\"%s\",\"l_max\":%.1f,\"l_min\":%.2f,\"gamma\":%.1f,\"pu21_mode\":\"%s\",\"legacy\":%s,\"version\":\"ffmpeg\",\"width\":%d,\"height\":%d}}\n",
                calc_domain_str, hdr_mode_str, s->bit_depth, color_range_str, eotf_str,
                s->l_max, s->l_min, s->gamma, pu21_mode_str, s->legacy ? "true" : "false",
                s->width, s->height);
        fflush(s->stats_file);
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

static float eotf_1886(float input, float gamma, float l_min, float l_max)
{
    float a, b;

    if (input < 0.0f)
        input = 0.0f;
    if (input > 1.0f)
        input = 1.0f;

    a = powf(powf(l_max, 1.0f / gamma) - powf(l_min, 1.0f / gamma), gamma);
    b = powf(l_min, 1.0f / gamma) /
        (powf(l_max, 1.0f / gamma) - powf(l_min, 1.0f / gamma));

    return a * powf(fmaxf(input + b, 0.0f), gamma);
}

static float eotf_hlg(float input, float l_min, float l_max)
{
    const float a = 0.17883277f;
    const float b = 0.02372241f;
    const float c = 1.00429347f;

    float gamma;
    if (l_max <= 1000.0f) {
        gamma = 1.2f;
    } else {
        gamma = 1.2f + 0.42f * log10f(l_max / 1000.0f);
    }

    float result;
    if (input <= 0.5f) {
        result = powf(input, 2.0f) / 3.0f;
    } else {
        result = expf((input - c) / a) - b;
    }

    // Apply system gamma and luminance mapping
    result = (l_max - l_min) * powf(result, gamma) + l_min;

    return result;
}

static float apply_display_model(SiTiContext *s, float input)
{
    if (input < 0.0f || input > 1.0f) {
        return 0.0f;
    }

    float linear = eotf_1886(input, s->gamma, 0.0f, 1.0f); // EOTF maps to [0,1]
    return (s->l_max - s->l_min) * linear +
           s->l_min; // Map to physical luminance
}

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
        p_min = -0.00000015580f;
        p_max = 520.4673f;
        break;
    case BANDING_GLARE:
        p[0] = 0.353487901f;
        p[1] = 0.3734658629f;
        p[2] = 0.00008277049286f;
        p[3] = 0.9062562627f;
        p[4] = 0.09150303166f;
        p[5] = 0.9099517204f;
        p[6] = 596.3148142f;
        p_min = 0.00000000054705f;
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
        p_min = 0.00000013674f;
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
        p_min = -0.000000097360f;
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

#define NORMALIZE_FRAME(src_type, src_data) do { \
    src_type *src = (src_type *)src_data; \
    if (s->full_range) { \
        /* for full range: simply divide by max value */ \
        for (int i = 0; i < frame_size; i++) { \
            normalized_frame[i] = src[i] / max_value; \
        } \
    } else { \
        /* for limited range: apply (input - min) / (max - min) */ \
        const float range = mpeg_max - mpeg_min; \
        for (int i = 0; i < frame_size; i++) { \
            float val = (float)src[i]; \
            val = fmaxf(fminf(val, mpeg_max), mpeg_min);  /* clamp to valid range */ \
            normalized_frame[i] = (val - mpeg_min) / range; \
        } \
    } \
} while(0)

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

static float calculate_ti_float(float *curr_frame, float *prev_frame, int width,
                                int height)
{
    if (!prev_frame)
        return 0.0f;

    int size = width * height;
    float *diff = av_malloc(size * sizeof(float));

    if (!diff)
        return 0.0f;

    for (int i = 0; i < size; i++) {
        diff[i] = curr_frame[i] - prev_frame[i];
    }

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

    int frame_is_full_range = is_full_range(frame);

    if (s->color_range == UNSPECIFIED) {
        // use specified color range if input is unspecified
        s->full_range = frame_is_full_range;
    } else {
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

    int frame_size = s->width * s->height;
    normalized_frame = av_malloc(frame_size * sizeof(float));
    transformed_frame = av_malloc(frame_size * sizeof(float));

    if (!normalized_frame || !transformed_frame) {
        av_freep(&normalized_frame);
        av_freep(&transformed_frame);
        return AVERROR(ENOMEM);
    }

    // Step 1: Normalize frame data to 0-1 range, accounting for color range
    // Calculate range values based on bit depth
    const float max_value = (1 << s->bit_depth) - 1;  // 255 for 8-bit, 1023 for 10-bit, etc.
    const float mpeg_min = 16.0f * (1 << (s->bit_depth - 8));  // 16 for 8-bit, 64 for 10-bit
    const float mpeg_max = 235.0f * (1 << (s->bit_depth - 8)); // 235 for 8-bit, 940 for 10-bit

    if (s->bit_depth > 8) {
        // 10-bit or higher
        NORMALIZE_FRAME(uint16_t, frame->data[0]);
    } else {
        // 8-bit
        NORMALIZE_FRAME(uint8_t, frame->data[0]);
    }

    // Step 2: Apply transformations based on HDR mode
    if (s->legacy) {
        // In legacy mode, skip all HDR transformations instead convert
        // normalized 0-1 data back to 0-255 range to match legacy SITI calculation
        for (int i = 0; i < frame_size; i++) {
            if (!s->full_range) {
                // Match siti-tools' legacy rounding behavior for limited range
                transformed_frame[i] = roundf(normalized_frame[i] * 255.0f);
            } else {
                transformed_frame[i] = normalized_frame[i] * 255.0f;
            }
        }
    } else if (s->hdr_mode == HDR10) {
        // For HDR10, the data is already in PQ domain - no transformation needed
        // Just copy the normalized data as-is
        for (int i = 0; i < frame_size; i++) {
            transformed_frame[i] = normalized_frame[i];
        }
    } else if (s->hdr_mode == HLG) {
        // For HLG, apply HLG EOTF
        for (int i = 0; i < frame_size; i++) {
            transformed_frame[i] = eotf_hlg(normalized_frame[i], s->l_min, s->l_max);
        }
        // Then apply OETF
        if (s->calculation_domain == PQ) {
            for (int i = 0; i < frame_size; i++) {
                transformed_frame[i] = oetf_pq(transformed_frame[i]);
            }
        } else {
            for (int i = 0; i < frame_size; i++) {
                transformed_frame[i] = oetf_pu21(s, transformed_frame[i]);
            }
        }
    } else {
        // For SDR, apply display model (EOTF)
        for (int i = 0; i < frame_size; i++) {
            transformed_frame[i] = apply_display_model(s, normalized_frame[i]);
        }
        // Then apply OETF
        if (s->calculation_domain == PQ) {
            for (int i = 0; i < frame_size; i++) {
                transformed_frame[i] = oetf_pq(transformed_frame[i]);
            }
        } else {
            for (int i = 0; i < frame_size; i++) {
                transformed_frame[i] = oetf_pu21(s, transformed_frame[i]);
            }
        }
    }

    // Step 3: Calculate SI and TI on the transformed float data
    float si_raw = calculate_si_float(transformed_frame, s->width, s->height);
    float ti_raw = 0.0f;

    if (s->nb_frames > 0) {
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

    // Step 4: Normalize SI/TI back to [0, 255] range (like Python does)
    // In legacy mode, SI/TI values are NOT normalized because they are already in 0-255 range
    if (s->legacy) {
        si = si_raw;
        ti = ti_raw;
    } else {
        si = si_raw * 255.0f;
        ti = ti_raw * 255.0f;
    }

    // Calculate statistics
    s->max_si = fmaxf(si, s->max_si);
    s->max_ti = fmaxf(ti, s->max_ti);
    s->sum_si += si;
    s->sum_ti += ti;
    s->min_si = fminf(si, s->min_si);
    s->min_ti = s->nb_frames > 0 ? fminf(ti, s->min_ti) : s->min_ti;

    // Set si ti information in frame metadata
    set_meta(&frame->metadata, "lavfi.siti.si", si);
    set_meta(&frame->metadata, "lavfi.siti.ti", ti);

    // Write per-frame JSON line if output is enabled
    if (s->stats_file) {
        if (s->nb_frames == 0) {
            // First frame - no TI value
            fprintf(s->stats_file, "{\"frame\":%" PRId64 ",\"si\":%.15f}\n",
                    s->nb_frames, si);
        } else {
            fprintf(s->stats_file, "{\"frame\":%" PRId64 ",\"si\":%.15f,\"ti\":%.15f}\n",
                    s->nb_frames, si, ti);
        }
        fflush(s->stats_file);
    }

    // Clean up
    av_freep(&normalized_frame);
    av_freep(&transformed_frame);

    // Increment frame count at the end
    s->nb_frames++;

    return ff_filter_frame(inlink->dst->outputs[0], frame);
}

#define OFFSET(x) offsetof(SiTiContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_FILTERING_PARAM

static const AVOption siti_options[] = {
    { "hdr_mode", "select HDR mode", OFFSET(hdr_mode), AV_OPT_TYPE_INT,
      {.i64 = SDR}, SDR, HLG, FLAGS, .unit = "hdr_mode" },
        { "sdr",   "standard dynamic range", 0, AV_OPT_TYPE_CONST, {.i64 = SDR},   0, 0, FLAGS, .unit = "hdr_mode" },
        { "hdr10", "HDR10 (PQ transfer)",    0, AV_OPT_TYPE_CONST, {.i64 = HDR10}, 0, 0, FLAGS, .unit = "hdr_mode" },
        { "hlg",   "hybrid log-gamma",       0, AV_OPT_TYPE_CONST, {.i64 = HLG},   0, 0, FLAGS, .unit = "hdr_mode" },
    { "color_range", "select color range", OFFSET(color_range), AV_OPT_TYPE_INT,
      {.i64 = UNSPECIFIED}, UNSPECIFIED, FULL, FLAGS, .unit = "color_range" },
        { "unspecified", "unspecified color range", 0, AV_OPT_TYPE_CONST, {.i64 = UNSPECIFIED}, 0, 0, FLAGS, .unit = "color_range" },
        { "limited",     "limited range",           0, AV_OPT_TYPE_CONST, {.i64 = LIMITED},     0, 0, FLAGS, .unit = "color_range" },
        { "full",        "full range",              0, AV_OPT_TYPE_CONST, {.i64 = FULL},        0, 0, FLAGS, .unit = "color_range" },
    { "eotf_function", "select EOTF function", OFFSET(eotf_function), AV_OPT_TYPE_INT,
      {.i64 = BT1886}, BT1886, INV_SRGB, FLAGS, .unit = "eotf_function" },
        { "bt1886",   "BT.1886 EOTF",         0, AV_OPT_TYPE_CONST, {.i64 = BT1886},   0, 0, FLAGS, .unit = "eotf_function" },
        { "inv_srgb", "inverse sRGB",         0, AV_OPT_TYPE_CONST, {.i64 = INV_SRGB}, 0, 0, FLAGS, .unit = "eotf_function" },
    { "calculation_domain", "select calculation domain", OFFSET(calculation_domain), AV_OPT_TYPE_INT,
      {.i64 = PQ}, PQ, PU21, FLAGS, .unit = "calculation_domain" },
        { "pq",   "perceptual quantizer", 0, AV_OPT_TYPE_CONST, {.i64 = PQ},   0, 0, FLAGS, .unit = "calculation_domain" },
        { "pu21", "PU21 metric",          0, AV_OPT_TYPE_CONST, {.i64 = PU21}, 0, 0, FLAGS, .unit = "calculation_domain" },
    { "l_max", "set maximum luminance", OFFSET(l_max), AV_OPT_TYPE_FLOAT,
      {.dbl = 300.0}, 0.0, 10000.0, FLAGS },
    { "l_min", "set minimum luminance", OFFSET(l_min), AV_OPT_TYPE_FLOAT,
      {.dbl = 0.1}, 0.0, 1000.0, FLAGS },
    { "gamma", "set gamma value for BT.1886", OFFSET(gamma), AV_OPT_TYPE_FLOAT,
      {.dbl = 2.4}, 1.0, 3.0, FLAGS },
    { "pu21_mode", "select PU21 mode", OFFSET(pu21_mode), AV_OPT_TYPE_INT,
      {.i64 = BANDING}, BANDING, PEAKS_GLARE, FLAGS, .unit = "pu21_mode" },
        { "banding",       "banding artifacts",        0, AV_OPT_TYPE_CONST, {.i64 = BANDING},       0, 0, FLAGS, .unit = "pu21_mode" },
        { "banding_glare", "banding with glare",       0, AV_OPT_TYPE_CONST, {.i64 = BANDING_GLARE}, 0, 0, FLAGS, .unit = "pu21_mode" },
        { "peaks",         "peak artifacts",           0, AV_OPT_TYPE_CONST, {.i64 = PEAKS},         0, 0, FLAGS, .unit = "pu21_mode" },
        { "peaks_glare",   "peaks with glare",         0, AV_OPT_TYPE_CONST, {.i64 = PEAKS_GLARE},   0, 0, FLAGS, .unit = "pu21_mode" },
    { "legacy", "use legacy SI/TI calculation (8-bit only, no HDR)", OFFSET(legacy), AV_OPT_TYPE_BOOL,
      {.i64 = 0}, 0, 1, FLAGS },
    { "stats_file", "set file to write SI/TI statistics in line delimited JSON format", OFFSET(stats_file_str), AV_OPT_TYPE_STRING,
      {.str = NULL}, 0, 0, FLAGS },
    { NULL }
};

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
