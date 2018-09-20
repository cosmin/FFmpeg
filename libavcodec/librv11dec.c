/*
 * rv11 decoder
 * Copyright (c) 2017 RealNetworks, Inc. 1501 1st Ave. South, Suite 600, Seattle, WA 98134 U.S.A.
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


#include <stdio.h>

#include "avcodec.h"
#include "libavutil/imgutils.h"
#include "internal.h"
#include "get_bits.h"
#include "librv11dec.h"


#define WK_MULTI_FRAME 1
#define WK_USING_FRAME_IN_DECODER 0
#if WK_MULTI_FRAME
#define OUTPUT_FRAME_NUMBER 6
#endif

typedef struct LIBRMVBPLUSDecContext_struct {
    AVCodecContext *avctx;
    AVFrame out_frame;
    void *codec_status;

    uint32_t input_buf_size;
    uint32_t input_buf_num;
    uint32_t output_buf_size;
    uint32_t output_buf_num;
#if WK_MULTI_FRAME
    uint8_t* output_buf[OUTPUT_FRAME_NUMBER];
    int frame_index;
#else
    uint8_t* output_buf;
#endif
    uint32_t last_frame;
    uint32_t more_frames;
} LIBRMVBPLUSDecContext;

#define RV_MSG_ID_Decoder_Start_Sequence    20

static av_cold int librmvbplus_init(AVCodecContext *avctx)
{
    LIBRMVBPLUSDecContext *s = (LIBRMVBPLUSDecContext*)avctx->priv_data;
    GetBitContext gb;
    uint32_t SPO_extra_flags = 0;
    uint32_t stream_version = 0;
    rv_backend_init_params initParams;
    HX_RESULT result = 0;
    RV_MSG_Simple msg;
#if WK_MULTI_FRAME
    int j = 0;
#endif

    if (!avctx->extradata || !avctx->extradata_size )
    {
        return -1;
    }

    s->avctx = avctx;

    init_get_bits(&gb, avctx->extradata, avctx->extradata_size * 8);
    SPO_extra_flags = get_bits_long(&gb, 32);
    stream_version = get_bits_long(&gb, 32);

    memset(&initParams, 0, sizeof(rv_backend_init_params));
    initParams.usPels = avctx->width;
    initParams.usLines = avctx->height;
    initParams.usPadWidth = avctx->width;
    initParams.usPadHeight = avctx->height;
    initParams.bPacketization = TRUE;
    initParams.ulInvariants = SPO_extra_flags;
    initParams.ulStreamVersion = stream_version;

    s->output_buf_size = (avctx->width * avctx->height * 12) >> 3;

    result = RV60toYUV420Init(&initParams, &s->codec_status);
    if (HXR_OK != result)
    {
        return result;
    }

    avctx->pix_fmt = AV_PIX_FMT_YUV420P;
    s->last_frame = 0;
    s->more_frames = 0;
#if WK_USING_FRAME_IN_DECODER
    s->output_buf = av_mallocz(4);
#else
#if WK_MULTI_FRAME
    for(j = 0; j < OUTPUT_FRAME_NUMBER; j++)
        s->output_buf[j] = av_mallocz(s->output_buf_size);
    av_image_fill_arrays(s->out_frame.data, s->out_frame.linesize, s->output_buf[0], AV_PIX_FMT_YUV420P, avctx->width, avctx->height, 1);
    s->frame_index = 0;
#else
    s->output_buf = av_mallocz(s->output_buf_size);
    av_image_fill_arrays(s->out_frame.data, s->out_frame.linesize, s->output_buf, AV_PIX_FMT_YUV420P, avctx->width, avctx->height, 1);
#endif
#endif

    //Disable post filters
    msg.message_id = RV_MSG_ID_Smoothing_Postfilter;
    msg.value1 = RV_MSG_DISABLE;
    msg.value2 = 0;
    result = RV60toYUV420CustomMessage(&(msg.message_id), (void*)s->codec_status);

    return 0;
}

static int librmvbplus_decode_frame(AVCodecContext *avctx, void *data,
                                    int *got_frame, AVPacket *avpkt)
{
    LIBRMVBPLUSDecContext* s = avctx->priv_data;
    const uint8_t* buf = avpkt->data;
    int buf_size = avpkt->size;
    AVFrame* pict = data;
    int slice_count = 0;
    rv_segment* segment = NULL;
    HX_RESULT ret = HXR_FAIL;
    int consumed_bytes = 0;
    rv_backend_in_params inParams;
    rv_backend_out_params outParams;
#if WK_USING_FRAME_IN_DECODER
    uint8_t * yuv_frame;
#endif
    int* offset;
    int i = 0;

    if ((ret = ff_get_buffer(avctx, pict, 0)) < 0)
        return ret;

#if WK_MULTI_FRAME
    s->frame_index++;
    s->frame_index = (s->frame_index%OUTPUT_FRAME_NUMBER);
#endif
    if (buf_size == 0) // maybe last frame
    {
        if (s->last_frame)
        {
            *got_frame = 0;
            return 0;
        }
        inParams.dataLength = 0;
        inParams.bInterpolateImage = FALSE;
        inParams.numDataSegments = 0;
        inParams.pDataSegments = NULL;
        inParams.timestamp = 0;
        inParams.flags = RV_DECODE_MORE_FRAMES;

#if WK_USING_FRAME_IN_DECODER
        ret = RV60toYUV420Transform(NULL, (s->output_buf), &inParams, &outParams, s->codec_status);
        yuv_frame = (uint8_t * )((s->output_buf[3]<<24)|(s->output_buf[2]<<16)|(s->output_buf[1]<<8)|(s->output_buf[0]));
        pict->data[0] = yuv_frame+64*(avctx->width+128)+64;
        pict->data[1] = yuv_frame + (avctx->width + 128)*(avctx->height + 128) + 32*(avctx->width + 128) + 32;
        pict->data[2] = yuv_frame + (avctx->width + 128)*(avctx->height + 128) + 32*(avctx->width + 128) + 32 + (avctx->width>>1) + 64;
        pict->linesize[0] = avctx->width;
        pict->linesize[1] = avctx->width;
        pict->linesize[2] = avctx->width;
#else
#if WK_MULTI_FRAME
        pict->data[0] = s->output_buf[s->frame_index];
        pict->data[1] = s->output_buf[s->frame_index] + avctx->width * avctx->height;
        pict->data[2] = s->output_buf[s->frame_index] + avctx->width * avctx->height + (avctx->width * avctx->height>>2);
        pict->linesize[0] = avctx->width;
        pict->linesize[1] = (avctx->width>>1);
        pict->linesize[2] = (avctx->width>>1);
        ret = RV60toYUV420Transform(NULL, s->output_buf[s->frame_index], &inParams, &outParams, s->codec_status);
#else
        ret = RV60toYUV420Transform(NULL, s->output_buf, &inParams, &outParams, s->codec_status);
#endif
#endif
        if (ret != HXR_OK)
        {
            *got_frame = 0;
            return ret;
        }
        if(outParams.notes & RV_DECODE_LAST_FRAME)
        {
            s->last_frame = 1;
        }

        av_frame_ref(pict, &s->out_frame);
        pict->pts = outParams.timestamp;
#if FF_API_PKT_PTS
FF_DISABLE_DEPRECATION_WARNINGS
        pict->pkt_pts = avpkt->pts;
FF_ENABLE_DEPRECATION_WARNINGS
#endif
        pict->pkt_dts = avpkt->dts;

        pict->width = avctx->width;
        pict->height = avctx->height;
        pict->format = AV_PIX_FMT_YUV420P;

        pict->key_frame = 0;
        pict->pict_type = AV_PICTURE_TYPE_P;
        if (outParams.notes & RV_DECODE_KEY_FRAME) {
            pict->key_frame = 1;
            pict->pict_type = AV_PICTURE_TYPE_I;
        } else if ((outParams.notes & RV_DECODE_B_FRAME)
                || (outParams.notes & RV_DECODE_FRU_FRAME)) {
            pict->key_frame = 0;
            pict->pict_type = AV_PICTURE_TYPE_B;
        }

        if(outParams.notes & RV_DECODE_DONT_DRAW)
        {
            *got_frame = 0;
        }
        else
        {
            *got_frame = 1;
        }
        return 0; //to end last frame loop
    }
    if (!avctx->slice_count)
    {
        slice_count = (*buf++) + 1;
        //slices_hdr = buf + 4;
        //buf += 8 * slice_count;
        segment = av_mallocz(slice_count * sizeof(rv_segment));
        for (i = 0; i < slice_count; i++)
        {
            segment[i].bIsValid = (*buf);
            buf += 4;
            offset = (int*)buf;
            segment[i].ulOffset = (*offset);
            buf += 4;
        }
        buf_size -= 1 + 8 * slice_count;
    }
    else
    {
        slice_count = avctx->slice_count;
    }

    inParams.dataLength = buf_size;
    inParams.bInterpolateImage = FALSE;
    inParams.numDataSegments = slice_count - 1;
    inParams.pDataSegments = segment;
    inParams.timestamp = avpkt->pts;
    inParams.flags = 0;

#if WK_USING_FRAME_IN_DECODER
    ret = RV60toYUV420Transform((UCHAR*)buf, s->output_buf, &inParams, &outParams, s->codec_status);
    yuv_frame = (uint8_t * )((s->output_buf[3]<<24)|(s->output_buf[2]<<16)|(s->output_buf[1]<<8)|(s->output_buf[0]));
    pict->data[0] = yuv_frame + 64*(avctx->width + 128) + 64;
    pict->data[1] = yuv_frame + (avctx->width + 128) * (avctx->height + 128) + 32*(avctx->width+128) + 32;
    pict->data[2] = yuv_frame + (avctx->width + 128) * (avctx->height + 128) + 32*(avctx->width+128) + 32 + (avctx->width>>1) + 64;
    pict->linesize[0] = avctx->width + 128;
    pict->linesize[1] = avctx->width + 128;
    pict->linesize[2] = avctx->width + 128;
#else
#if WK_MULTI_FRAME
    pict->data[0] = s->output_buf[s->frame_index];
    pict->data[1] = s->output_buf[s->frame_index] + avctx->width * avctx->height;
    pict->data[2] = s->output_buf[s->frame_index] + avctx->width * avctx->height + (avctx->width * avctx->height>>2);
    pict->linesize[0] = avctx->width;
    pict->linesize[1] = (avctx->width>>1);
    pict->linesize[2] = (avctx->width>>1);
    ret = RV60toYUV420Transform((UCHAR*)buf, s->output_buf[s->frame_index], &inParams, &outParams, s->codec_status);
#else
    av_image_fill_arrays(pict->data, pict->linesize, s->output_buf, AV_PIX_FMT_YUV420P, avctx->width, avctx->height, 1);
    ret = RV60toYUV420Transform((UCHAR*)buf, s->output_buf, &inParams, &outParams, s->codec_status);
#endif
#endif
    consumed_bytes = avpkt->size;

    if (slice_count)
        av_free(segment);

    if (ret != HXR_OK)
    {
        consumed_bytes = 0;
        *got_frame = 0;
        return ret;
    }

    *got_frame = 1;

    //set the default value to key_frame and pict_type
    pict->key_frame = 0;
    pict->pict_type = AV_PICTURE_TYPE_P;

    if (outParams.notes & RV_DECODE_KEY_FRAME)
    {
        pict->key_frame = 1;
        *got_frame = 1;
        pict->pict_type = AV_PICTURE_TYPE_I;
    }
    else if ((outParams.notes & RV_DECODE_B_FRAME)
            || (outParams.notes & RV_DECODE_FRU_FRAME))
    {
        pict->key_frame = 0;
        *got_frame = 1;
        pict->pict_type = AV_PICTURE_TYPE_B;
    }
    else if (outParams.notes & RV_DECODE_DONT_DRAW)
    {
        *got_frame = 0;
        consumed_bytes = 0;
    }

    if (SUCCEEDED(ret))
    {
        if (pict)
        {
            pict->pts = outParams.timestamp;
#if FF_API_PKT_PTS
FF_DISABLE_DEPRECATION_WARNINGS
            pict->pkt_pts = avpkt->pts;
FF_ENABLE_DEPRECATION_WARNINGS
#endif
            pict->pkt_dts = avpkt->dts;

            pict->width = avctx->width;
            pict->height = avctx->height;
            pict->format = AV_PIX_FMT_YUV420P;
        }
    }

    return consumed_bytes;
}

static av_cold int librmvbplus_close(AVCodecContext *avctx)
{
    LIBRMVBPLUSDecContext* s = avctx->priv_data;
#if WK_MULTI_FRAME
    int i = 0;
#endif
#if WK_USING_FRAME_IN_DECODER
    av_free(s->output_buf);
#else
#if WK_MULTI_FRAME
    for(i = 0; i < OUTPUT_FRAME_NUMBER; i++)
    {
        if (s->output_buf[i])
        {
            av_free(s->output_buf[i]);
            s->output_buf[i] = NULL;
        }
    }
#else
    if (s->output_buf)
    {
        av_free(s->output_buf);
        s->output_buf = NULL;
    }
#endif
#endif
    return 0;
}

static void librmvbplus_flush(AVCodecContext *avctx)
{
    LIBRMVBPLUSDecContext* s = (LIBRMVBPLUSDecContext*)avctx->priv_data;
    ULONG32 id = RV_MSG_ID_Decoder_Start_Sequence;
    RV60toYUV420HiveMessage(&id, s->codec_status);
}

AVCodec ff_librv11_decoder = {
    .name                  = "librv11",
    .long_name             = NULL_IF_CONFIG_SMALL("librv11 RealVideo 11 RV60"),
    .type                  = AVMEDIA_TYPE_VIDEO,
    .id                    = AV_CODEC_ID_RV60,
    .priv_data_size        = sizeof(LIBRMVBPLUSDecContext),
    .init                  = librmvbplus_init,
    .decode                = librmvbplus_decode_frame,
    .close                 = librmvbplus_close,
    .flush                 = librmvbplus_flush,
    .capabilities          = AV_CODEC_CAP_DELAY,
    .wrapper_name          = "librv11",
};
