/*
 * This copyright notice applies to this file only
 * This Software is distributed under MIT License
 *
 * API software for using RealVideo 11 (RV60) Decoder
 *
 *      * Copyright (c) 2017 Qiang Luo, RealNetworks, Inc. <qluo _at_ realnetworks.com>
 *      * Copyright (c) 2017 Thilo Borgmann <thilo.borgmann _at_ mail.de>
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include "libavutil/imgutils.h"
#include "internal.h"
#include "get_bits.h"

#include "librv11.h"

typedef struct LIBRV11DecContext {
    AVCodecContext *avctx;
    AVFrame        out_frame;
    void           *codec_status;
    uint32_t       input_buf_size;
    uint32_t       input_buf_num;
    uint32_t       output_buf_size;
    uint32_t       output_buf_num;
    uint8_t        *output_buf[RV_NUM_OUT_FRAMES];
    int            frame_index;
    uint32_t       last_frame;
    uint32_t       more_frames;
} LIBRV11DecContext;

static av_cold int librv11dec_init(AVCodecContext *avctx)
{
    LIBRV11DecContext *ctx = avctx->priv_data;
    RVInitParams init_params;
    RVMsgSimple msg;
    HX_RESULT res;
    uint32_t SPO_extra_flags;
    uint32_t version;
    int i;
    ctx->avctx = avctx;

    if (!avctx->extradata || !avctx->extradata_size) {
        return AVERROR_INVALIDDATA;
    }

    SPO_extra_flags = AV_RL32(&avctx->extradata[0]);
    version         = AV_RL32(&avctx->extradata[5]);

    init_params.out_type   = 0;
    init_params.width      = avctx->width;
    init_params.height     = avctx->height;
    init_params.pad_width  = avctx->width;
    init_params.pad_height = avctx->height;
    init_params.pad_to_32  = 0;
    init_params.invariants = SPO_extra_flags;
    init_params.packetize  = 1;
    init_params.version    = version;

    res = RV60toYUV420Init(&init_params, &ctx->codec_status);
    if (res) {
        return res;
    }

    msg.id     = RV_MSG_ID_SMOOTHING_POSTFILTER;
    msg.value1 = RV_MSG_DISABLE;
    msg.value2 = 0;

    res = RV60toYUV420CustomMessage(&msg.id, ctx->codec_status);
    if (res) {
        return res;
    }

    avctx->pix_fmt   = AV_PIX_FMT_YUV420P;
    ctx->last_frame  = 0;
    ctx->more_frames = 0;
    ctx->frame_index = 0;

    ctx->output_buf_size = (avctx->width * avctx->height * 12) >> 3;
    for(i = 0; i < RV_NUM_OUT_FRAMES; i++) {
        ctx->output_buf[i] = av_mallocz(ctx->output_buf_size);
    }
    av_image_fill_arrays(ctx->out_frame.data, ctx->out_frame.linesize, ctx->output_buf[0], AV_PIX_FMT_YUV420P, avctx->width, avctx->height, 1);

    return 0;
}

static int librv11dec_decode_last(AVCodecContext *avctx, void *data,
                                    int *got_frame, AVPacket *avpkt)
{
    LIBRV11DecContext* ctx = avctx->priv_data;
    AVFrame *pict    = data;
    const int stride = avctx->width * avctx->height;
    RVInParams inParams;
    RVOutParams outParams;
    int32_t ret;

    inParams.length       = 0;
    inParams.interpolate  = 0;
    inParams.num_segments = 0;
    inParams.segments     = NULL;
    inParams.timestamp    = 0;
    inParams.flags        = RV_DECODE_MORE_FRAMES;

    pict->data[0] = ctx->output_buf[ctx->frame_index];
    pict->data[1] = ctx->output_buf[ctx->frame_index] + stride;
    pict->data[2] = ctx->output_buf[ctx->frame_index] + stride + (stride >> 2);

    pict->linesize[0] = avctx->width;
    pict->linesize[1] = avctx->width >> 1;
    pict->linesize[2] = avctx->width >> 1;

    ret = RV60toYUV420Transform(NULL, ctx->output_buf[ctx->frame_index], &inParams, &outParams, ctx->codec_status);
    if (ret) {
        *got_frame = 0;
        return ret;
    }

    if(outParams.notes & RV_DECODE_LAST_FRAME) {
        ctx->last_frame = 1;
    }

    pict->pts       = outParams.timestamp;
    pict->pkt_dts   = avpkt->dts;
    pict->width     = avctx->width;
    pict->height    = avctx->height;
    pict->format    = AV_PIX_FMT_YUV420P;
    pict->key_frame = 0;
    pict->pict_type = AV_PICTURE_TYPE_P;

    if (outParams.notes & RV_DECODE_KEY_FRAME) {
        pict->key_frame = 1;
        pict->pict_type = AV_PICTURE_TYPE_I;
    } else if (outParams.notes & RV_DECODE_B_FRAME ||
               outParams.notes & RV_DECODE_FRU_FRAME) {
        pict->key_frame = 0;
        pict->pict_type = AV_PICTURE_TYPE_B;
    }

    *got_frame = !(outParams.notes & RV_DECODE_DONT_DRAW);

    return 0;

}

static int librv11dec_decode_frame(AVCodecContext *avctx, void *data,
                                    int *got_frame, AVPacket *avpkt)
{
    LIBRV11DecContext* ctx = avctx->priv_data;
    const uint8_t *buf = avpkt->data;
    int buf_size       = avpkt->size;
    const int stride   = avctx->width * avctx->height;
    AVFrame *pict      = data;
    int slice_count    = 0;
    int consumed_bytes = 0;
    RVSegment *segment = NULL;
    RVInParams inParams;
    RVOutParams outParams;
    int32_t ret;
    int i;

    ret = ff_get_buffer(avctx, pict, 0);
    if (ret < 0) {
        return ret;
    }

    ctx->frame_index++;
    ctx->frame_index %= RV_NUM_OUT_FRAMES;

    if (!buf_size) { // maybe last frame
        if (ctx->last_frame) {
            *got_frame = 0;
            return 0;
        } else {
            return librv11dec_decode_last(avctx, data, got_frame, avpkt);
        }
    }

    if (!avctx->slice_count) {
        slice_count = (*buf++) + 1;
        segment = av_mallocz(slice_count * sizeof(RVSegment));
        for (i = 0; i < slice_count; i++) {
            segment[i].is_valid = *buf;
            segment[i].offset   = *(buf + 4);
            buf += 8;
        }
        buf_size -= 1 + 8 * slice_count;
    } else {
        slice_count = avctx->slice_count;
    }

    inParams.length       = buf_size;
    inParams.interpolate  = 0;
    inParams.num_segments = slice_count - 1;
    inParams.segments     = segment;
    inParams.timestamp    = avpkt->pts;
    inParams.flags        = 0;

    pict->data[0] = ctx->output_buf[ctx->frame_index];
    pict->data[1] = ctx->output_buf[ctx->frame_index] + stride;
    pict->data[2] = ctx->output_buf[ctx->frame_index] + stride + (stride >> 2);
    pict->linesize[0] = avctx->width;
    pict->linesize[1] = avctx->width >> 1;
    pict->linesize[2] = avctx->width >> 1;

    ret = RV60toYUV420Transform((UCHAR*)buf, ctx->output_buf[ctx->frame_index], &inParams, &outParams, ctx->codec_status);
    if (ret) {
        consumed_bytes = 0;
        *got_frame     = 0;
        return ret;
    }

    if (slice_count) {
        av_freep(&segment);
    }

    consumed_bytes  = avpkt->size;
    *got_frame      = 1;
    pict->key_frame = 0;
    pict->pict_type = AV_PICTURE_TYPE_P;

    if (outParams.notes & RV_DECODE_KEY_FRAME) {
        pict->pict_type = AV_PICTURE_TYPE_I;
        pict->key_frame = 1;
    } else if (outParams.notes & RV_DECODE_B_FRAME ||
               outParams.notes & RV_DECODE_FRU_FRAME) {
        pict->pict_type = AV_PICTURE_TYPE_B;
    } else if (outParams.notes & RV_DECODE_DONT_DRAW) {
        consumed_bytes  = 0;
        *got_frame      = 0;
    }

    pict->pts     = outParams.timestamp;
    pict->pkt_dts = avpkt->dts;
    pict->width   = avctx->width;
    pict->height  = avctx->height;
    pict->format  = AV_PIX_FMT_YUV420P;

    return consumed_bytes;
}

static av_cold int librv11dec_close(AVCodecContext *avctx)
{
    LIBRV11DecContext* ctx = avctx->priv_data;
    int i;

    RV60toYUV420Free(ctx->codec_status);

    for(i = 0; i < RV_NUM_OUT_FRAMES; i++) {
        if (ctx->output_buf[i]) {
            av_freep(&ctx->output_buf[i]);
        }
    }

    return 0;
}

static void librv11dec_flush(AVCodecContext *avctx)
{
    LIBRV11DecContext* ctx = avctx->priv_data;
    uint32_t id = RV_MSG_ID_FLUSH;
    RV60toYUV420HiveMessage(&id, ctx->codec_status);
}

AVCodec ff_librv11_decoder = {
    .name           = "librv11",
    .long_name      = NULL_IF_CONFIG_SMALL("librv11 RealVideo 11 (RV60)"),
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = AV_CODEC_ID_RV60,
    .priv_data_size = sizeof(LIBRV11DecContext),
    .init           = librv11dec_init,
    .decode         = librv11dec_decode_frame,
    .close          = librv11dec_close,
    .flush          = librv11dec_flush,
    .capabilities   = AV_CODEC_CAP_DELAY,
    .wrapper_name   = "librv11",
};
