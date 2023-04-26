/*
 * Copyright (c) 2003-2013 Loren Merritt
 * Copyright (c) 2015 Paul B Mahol
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

/* Computes the Structural Similarity Metric between two video streams.
 * original algorithm:
 * Z. Wang, A. C. Bovik, H. R. Sheikh and E. P. Simoncelli,
 *   "Image quality assessment: From error visibility to structural similarity,"
 *   IEEE Transactions on Image Processing, vol. 13, no. 4, pp. 600-612, Apr. 2004.
 *
 * To improve speed, this implementation uses the standard approximation of
 * overlapped 8x8 block sums, rather than the original gaussian weights.
 */

/*
 * @file
 * Calculate the SSIM between two input videos.
 */

#include "libavutil/avstring.h"
#include "libavutil/file_open.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "avfilter.h"
#include "drawutils.h"
#include "framesync.h"
#include "internal.h"
#include "ssim.h"
#include "filters.h"

typedef struct TAlignContext {
    const AVClass *class;
    FFFrameSync fs;
    uint64_t nb_frames;
    int n_subsample;
} TAlignContext;

#define OFFSET(x) offsetof(TAlignContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM

static const AVOption talign_options[] = {
    {"n_subsample", "Set interval for frame subsampling, will keep 1 in N frames.",     OFFSET(n_subsample), AV_OPT_TYPE_INT, {.i64=1}, 1, UINT_MAX, FLAGS},
    { NULL }
};
FRAMESYNC_DEFINE_CLASS(talign, TAlignContext, fs);

static int do_align(FFFrameSync *fs)
{
    AVFilterContext *ctx = fs->parent;
    TAlignContext *s = ctx->priv;
    AVFrame *master, *ref;
    int ret;

    ret = ff_framesync_dualinput_get(fs, &master, &ref);
    if (ret < 0) {
        av_log(ctx, AV_LOG_VERBOSE, "Failed to get dual frames\n");
        return ret;
    }

    s->nb_frames++;

    if (master && ref) {
        av_log(ctx, AV_LOG_VERBOSE, "Got main frame with PTS %lld\n", master->pts);
        ff_filter_frame(ctx->outputs[0], master);
        ref->pts = master->pts;
        ref->duration = master->duration;
        av_log(ctx, AV_LOG_VERBOSE, "Got ref frame with PTS %lld\n", ref->pts);
        ff_filter_frame(ctx->outputs[1], ref);
    } else {
        av_log(ctx, AV_LOG_VERBOSE, "Only got one of the two frames\n");
        if (ff_inoutlink_check_flow(ctx->inputs[0], ctx->outputs[0]) && ff_inoutlink_check_flow(ctx->inputs[1], ctx->outputs[1])) {
            ff_filter_set_ready(ctx, 100);
        }
        return 0;
    }

    return 0;
}

static av_cold int init(AVFilterContext *ctx)
{
    TAlignContext *s = ctx->priv;
    s->fs.on_event = do_align;
    return 0;
}

static const enum AVPixelFormat pix_fmts[] = {
    AV_PIX_FMT_GRAY8, AV_PIX_FMT_GRAY9, AV_PIX_FMT_GRAY10,
    AV_PIX_FMT_GRAY12, AV_PIX_FMT_GRAY14, AV_PIX_FMT_GRAY16,
    AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUV422P, AV_PIX_FMT_YUV444P,
    AV_PIX_FMT_YUV440P, AV_PIX_FMT_YUV411P, AV_PIX_FMT_YUV410P,
    AV_PIX_FMT_YUVJ411P, AV_PIX_FMT_YUVJ420P, AV_PIX_FMT_YUVJ422P,
    AV_PIX_FMT_YUVJ440P, AV_PIX_FMT_YUVJ444P,
    AV_PIX_FMT_GBRP,
#define PF(suf) AV_PIX_FMT_YUV420##suf,  AV_PIX_FMT_YUV422##suf,  AV_PIX_FMT_YUV444##suf, AV_PIX_FMT_GBR##suf
    PF(P9), PF(P10), PF(P12), PF(P14), PF(P16),
    AV_PIX_FMT_NONE
};

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    TAlignContext *s = ctx->priv;
    AVFilterLink *mainlink = ctx->inputs[0];
    int ret;

    ret = ff_framesync_init_dualinput(&s->fs, ctx);
    if (ret < 0)
        return ret;
    outlink->w = mainlink->w;
    outlink->h = mainlink->h;
    outlink->time_base = mainlink->time_base;
    outlink->sample_aspect_ratio = mainlink->sample_aspect_ratio;
    outlink->frame_rate = mainlink->frame_rate;

    if ((ret = ff_framesync_configure(&s->fs)) < 0)
        return ret;

    outlink->time_base = s->fs.time_base;

    if (av_cmp_q(mainlink->time_base, outlink->time_base) ||
        av_cmp_q(ctx->inputs[1]->time_base, outlink->time_base))
        av_log(ctx, AV_LOG_WARNING, "not matching timebases found between first input: %d/%d and second input %d/%d, results may be incorrect!\n",
               mainlink->time_base.num, mainlink->time_base.den,
               ctx->inputs[1]->time_base.num, ctx->inputs[1]->time_base.den);

    return 0;
}

static int config_output_ref(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    TAlignContext *s = ctx->priv;
    AVFilterLink *mainlink = ctx->inputs[0];
    AVFilterLink *reflink = ctx->inputs[1];
    int ret;

    outlink->w = reflink->w;
    outlink->h = reflink->h;
    outlink->sample_aspect_ratio = reflink->sample_aspect_ratio;

    // copy these from the main link as we are going to synchronize the two
    outlink->time_base = mainlink->time_base;
    outlink->frame_rate = mainlink->frame_rate;
    outlink->time_base = reflink->time_base;
    outlink->frame_rate = reflink->frame_rate;
//    outlink->time_base = s->fs.time_base;

    return 0;
}

static int activate(AVFilterContext *ctx)
{
    TAlignContext *s = ctx->priv;

//    FF_FILTER_FORWARD_STATUS_BACK(ctx->outputs[0], ctx->inputs[0]);
//    FF_FILTER_FORWARD_STATUS_BACK(ctx->outputs[1], ctx->inputs[1]);

    return ff_framesync_activate(&s->fs);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    TAlignContext *s = ctx->priv;
    ff_framesync_uninit(&s->fs);
}

static const AVFilterPad talign_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
    },{
        .name         = "ref",
        .type         = AVMEDIA_TYPE_VIDEO,
    },
};

static const AVFilterPad talign_outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_VIDEO,
        .config_props  = config_output,
    },
    {
        .name         = "ref",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = config_output_ref,
    }
};

const AVFilter ff_vf_talign = {
    .name          = "talign",
    .description   = NULL_IF_CONFIG_SMALL("Frame align a video stream with its reference for subsequent metric calculations"),
    .preinit       = talign_framesync_preinit,
    .init          = init,
    .uninit        = uninit,
    .activate      = activate,
    .priv_size     = sizeof(TAlignContext),
    .priv_class    = &talign_class,
    FILTER_INPUTS(talign_inputs),
    FILTER_OUTPUTS(talign_outputs),
    FILTER_PIXFMTS_ARRAY(pix_fmts),
    .flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL |
                     AVFILTER_FLAG_SLICE_THREADS             |
                     AVFILTER_FLAG_METADATA_ONLY,
};
