/*
 * This copyright notice applies to this file only
 * This Software is distributed under MIT License
 *
 * API software for using RealVideo 11 (RV60) Encoder
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
#include "libavutil/opt.h"

#include "librv11.h"

typedef struct CodedFrameList {
    AVPacket pkt;
    struct CodedFrameList* pNext;
} CodedFrameList;

typedef struct FrameRatioConverter {
    uint32_t    count;
    double      framerate;
    uint8_t*    selected_map;
    uint32_t    framerate_in;
} FrameRatioConverter;

typedef struct LIBRV11EncContext
{
    AVClass    *class;
    void       *codec_ref;
    void       *stream_ref;
    RVEncodeParam rvenc_param;
    uint32_t   codec_4cc;
    uint32_t   frame_size;
    uint8_t*   frame_buf;
    CodedFrameList* coded_frame_list;
    int        bitrate_vbr;           ///< 0:VBRQuality, 1:VBRBitrate
    int        eos;                   ///< set to TURE when input frame was set to NULL
    int        last_packet;           ///< set to 1 when the last codec frame was popped out
    int        seen_packet;
    int64_t    prev_dts;
    FrameRatioConverter fr_convert;
} LIBRV11EncContext;

static int32_t interval_search(uint8_t *buf, const uint32_t buf_size, const uint32_t num_frames)
{
    if (buf_size < num_frames || !num_frames) {
        return -1;
    }

    if (!(buf_size % num_frames)) {
        uint32_t i;
        const uint32_t den = buf_size / num_frames;

        for(i = 0; i < buf_size; i++) {
            if (!((i+1) % den)) {
                buf[i] = 1;
            }
        }
    } else {
        const uint32_t half  = buf_size >> 1;
        const uint32_t left  = num_frames >> 1;
        const uint32_t right = left + (num_frames & 1);

        interval_search(buf, half, left);
        interval_search(buf + half, buf_size - half, right);
    }

    return 0;
}

static av_cold int32_t fr_convert_init(FrameRatioConverter* fr_convert, double fr_in, double fr_max)
{
    if (fr_in < fr_max || fr_in > RV_MAX_INPUT_FRAME_RATE) {
        return -1;
    }

    fr_convert->count        = 0;
    fr_convert->framerate    = fr_max;
    fr_convert->framerate_in = (int32_t)(fr_in + 0.5);
    fr_convert->selected_map = av_mallocz(fr_convert->framerate_in);

    return interval_search(fr_convert->selected_map, fr_convert->framerate_in, (int32_t)(fr_max + 0.5));
}

static av_always_inline int32_t use_frame(FrameRatioConverter *fr_convert)
{
    return fr_convert->selected_map[fr_convert->count];
}

static void next_frame(FrameRatioConverter* fr_convert)
{
    fr_convert->count++;
    if (fr_convert->count >= fr_convert->framerate_in) {
        fr_convert->count = 0;
    }
}

static void coded_frame_add(void *list, CodedFrameList *cx_frame)
{
    CodedFrameList **p = list;

    while(*p) {
        p = &(*p)->pNext;
    }

    *p = cx_frame;
    cx_frame->pNext = NULL;
}

static CodedFrameList* coded_frame_remove_header(CodedFrameList** list)
{
    CodedFrameList *p = *list;

    if (p) {
        *list = p->pNext;
    }

    return p;
}

static av_cold void free_coded_frame(CodedFrameList *cx_frame)
{
    av_packet_unref(&cx_frame->pkt);
    av_freep(&cx_frame);
}

static av_cold void free_frame_list(CodedFrameList *list)
{
    CodedFrameList *p = list;

    while(p) {
        list = list->pNext;
        free_coded_frame(p);
        p = list;
    }
}

static int librv11_num_segments(uint32_t data_length)
{
    int num_packet;

    if (data_length < 1) {
        return -1;
    }

    num_packet = data_length / RV_MAX_PACKET_SIZE;
    if (data_length % RV_MAX_PACKET_SIZE) {
        num_packet++;
    }

    return num_packet;
}

static int librv11_write_frame_header(uint8_t *data, int num_packet)
{
    int i;

    if (num_packet < 1 || !data) {
        return -1;
    }

    data[0] = num_packet -1;

    for (i = 0; i < num_packet; i++) {
        // the value of next 4 byte is the segment index
        AV_WL32(data + 1 + 8 * i, i + 1);
        // continuous 4 byte is the segment offset
        AV_WL32(data + 5 + 8 * i, RV_MAX_PACKET_SIZE * i);
    }

    return 0;
}

static av_cold int CC librv11enc_data_callback(void *stream_ctx, void *stream_ref, HXCodecData* data)
{
    int ret = 0;
    int pict_type;
    int64_t cur_pts;
    int64_t cur_dts;
    AVCodecContext *avctx = (AVCodecContext*) stream_ctx;
    LIBRV11EncContext *enctx = avctx->priv_data;

    if (data && data->length) {
        int numPackets = 0;
        int bufLength;

        CodedFrameList* node = av_mallocz(sizeof(CodedFrameList));
        if (!node) {
            av_log(avctx, AV_LOG_ERROR, "Failed to allocate memory.\n");
            return AVERROR(ENOMEM);
        }

        // for rv11 num_segments should be 1
        if (data->num_segments == 1) {
            numPackets = librv11_num_segments(data->length);
            if (numPackets > 127 || numPackets < 0) {
                av_log(avctx, AV_LOG_WARNING, "Too many packets per frame (%d).\n", numPackets);
            }
        } else {
            av_log(avctx, AV_LOG_WARNING, "Invalid number of segments (%u), should be 1.\n", data->num_segments);
        }

        // allocate extra numPackets*8+1  byte for real video head saving segment information
        bufLength = data->length + numPackets * 8 + 1;
        ret = av_new_packet(&node->pkt, bufLength);
        if (ret < 0) {
            av_log(avctx, AV_LOG_ERROR, "Failed to allocate packet.\n");
            return ret;
        }
        librv11_write_frame_header(node->pkt.data, numPackets);
        memcpy((node->pkt.data + numPackets * 8 + 1), data->data, data->length);

        if (data->flags & HX_KEYFRAME_FLAG) {
            node->pkt.flags |= AV_PKT_FLAG_KEY;
        }

        if (data->last_packet) {
            node->pkt.flags |= RV_PKT_FLAG_END;
        }

        pict_type = ((*(data->data + 1)) >> 6) & 0x03;
        cur_pts = data->timestamp;
        if(!enctx->seen_packet) {
            enctx->seen_packet = 1;
            cur_dts = cur_pts - 1;             // first frame
        } else {
            if (pict_type == 2) {
                cur_dts = cur_pts;             // B-frame
            } else {
                cur_dts = enctx->prev_dts + 1; // ref-frame
            }
        }

        node->pkt.pts   = cur_pts;
        node->pkt.dts   = cur_dts;
        enctx->prev_dts = cur_dts;

        coded_frame_add((void*)(&enctx->coded_frame_list), node);
    }

    return 0;
}

static av_cold int librv11enc_init(AVCodecContext *avctx)
{
    LIBRV11EncContext *enctx = avctx->priv_data;
    HX_RESULT res;
    HXMof hxInput, hxOutput;
    HXFormatVideo mofin, mofout;
    HXCodecInit ci;
    double framerate;
    uint32_t dummy_ui32;
    float dummy_f;

    // Width and height of the video are required to be multiples of 4
    if (avctx->width % 4 || avctx->height % 4) {
        av_log(avctx, AV_LOG_ERROR,
        "Video dimensions have to be multiples of 4 (%dx%d).\n",
        avctx->width, avctx->height);
        return AVERROR(EINVAL);
    }

    res = PNCodec_QueryMediaFormat(NULL, &hxInput, &hxOutput, 0);
    if (FAILED(res)) {
        av_log(avctx, AV_LOG_ERROR, "Failed to query in/out formats.\n");
        return AVERROR_EXTERNAL;
    }

    res = PNCodec_Open(hxOutput.submoftag, &enctx->codec_ref);
    if (FAILED(res)) {
        av_log(avctx, AV_LOG_ERROR, "Failed to open the codec library.\n");
        return AVERROR_EXTERNAL;
    }

    enctx->codec_4cc = hxOutput.submoftag;

    // init the librv11 encoder using default/user parameters
    enctx->rvenc_param.in_width  = avctx->width;
    enctx->rvenc_param.in_height = avctx->height;

    if (!enctx->rvenc_param.out_width) {
        enctx->rvenc_param.out_width = avctx->width;
    }
    if (!enctx->rvenc_param.out_height) {
        enctx->rvenc_param.out_height = avctx->height;
    }

    if (enctx->bitrate_vbr) {
        enctx->rvenc_param.encode_type = ENCODE_VBR_BITRATE;
        if (avctx->flags & AV_CODEC_FLAG_PASS1) {
            enctx->rvenc_param.vbr_quality = 25;
        }
    } else {
        enctx->rvenc_param.encode_type = ENCODE_VBR_QUALITY;
    }

    if (avctx->flags & AV_CODEC_FLAG_PASS1) {
        enctx->rvenc_param.encode_mode = ENCODE_CREATE_ANALYSIS;
    } else if (avctx->flags & AV_CODEC_FLAG_PASS2){
        enctx->rvenc_param.encode_mode = ENCODE_USING_ANALYSIS;
    } else {
        enctx->rvenc_param.encode_mode = ENCODE_SINGLE_PASS;
    }

    enctx->rvenc_param.avg_bitrate = avctx->bit_rate;
    enctx->rvenc_param.max_bitrate = enctx->rvenc_param.avg_bitrate;

    enctx->rvenc_param.max_packet_size = RV_MAX_PACKET_SIZE;

    if (!avctx->framerate.den) {
        return AVERROR(EINVAL);
    }
    framerate = av_q2d(avctx->framerate);

    res = fr_convert_init(&enctx->fr_convert, framerate, enctx->rvenc_param.max_framerate);
    if (res) {
        fr_convert_init(&enctx->fr_convert, framerate, framerate);
    }

    // set input parameters for I420
    memset(&mofin, 0, sizeof(mofin));
    mofin.length     = sizeof(HXFormatVideo);
    mofin.moftag     = HX_MEDIA_VIDEO;
    mofin.submoftag  = HX_YUV420_ID;
    mofin.width      = enctx->rvenc_param.in_width;
    mofin.height     = enctx->rvenc_param.in_height;
    mofin.bit_count  = 12;
    mofin.pad_width  = 0;
    mofin.pad_height = 0;
    mofin.moftag     = HX_MEDIA_VIDEO;

    mofin.fps = (int)(enctx->fr_convert.framerate * (1L << 16) + 0.5);

    // set output parameters
    memset(&mofout, 0, sizeof(mofout));
    mofout.length     = sizeof(mofout);
    mofout.moftag     = HX_MEDIA_VIDEO;
    mofout.submoftag  = enctx->codec_4cc;
    mofout.width      = enctx->rvenc_param.out_width;
    mofout.height     = enctx->rvenc_param.out_height;
    mofout.bit_count  = mofin.bit_count;
    mofout.pad_width  = 0;
    mofout.pad_height = 0;
    mofout.moftag     = HX_MEDIA_VIDEO;
    mofout.fps        = mofin.fps;

    ci.in_mof  = (HXMof*)&mofin;
    ci.out_mof = (HXMof*)&mofout;
    ci.mem_ptr = NULL;

#define SET_RV11_PROP(prop, value) {                                        \
    res = PNStream_SetProperty(enctx->stream_ref, prop, value);             \
    if (FAILED(res)) {                                                      \
        av_log(avctx, AV_LOG_ERROR, "Failed to set property %s.\n", #prop); \
        return AVERROR_EXTERNAL;                                            \
    }                                                                       \
}

    res = PNCodec_StreamOpen(enctx->codec_ref, &enctx->stream_ref, &ci);
    if (FAILED(res)) {
        av_log(avctx, AV_LOG_ERROR, "Failed to open the stream.\n");
        return AVERROR_EXTERNAL;
    }

    if (enctx->bitrate_vbr && !(avctx->flags & AV_CODEC_FLAG_PASS1)) {
        SET_RV11_PROP(SP_BITRATE, &enctx->rvenc_param.avg_bitrate);
    } else {
        SET_RV11_PROP(SP_QUALITY, &enctx->rvenc_param.vbr_quality);
    }

    SET_RV11_PROP(SP_MAXBITRATE, &enctx->rvenc_param.max_bitrate);

    dummy_f = (float)(enctx->rvenc_param.max_framerate);
    SET_RV11_PROP(SP_MAX_FRAMERATE, &dummy_f);

    dummy_ui32 = 0;
    SET_RV11_PROP(SP_CHECK_FRAME_SKIP, &dummy_ui32);

    dummy_ui32 = 0;
    SET_RV11_PROP(SP_LIVE, &dummy_ui32);

    dummy_ui32 = enctx->rvenc_param.max_keyint * 1000;
    SET_RV11_PROP(SP_KEYFRAMERATE, &dummy_ui32);

    SET_RV11_PROP(SP_ECC, &enctx->rvenc_param.loss_protect);
    SET_RV11_PROP(SP_SET_FAST_LEVEL, &enctx->rvenc_param.enc_complexity);

    dummy_ui32 = (uint32_t) enctx->rvenc_param.max_start_latency * 1000;
    SET_RV11_PROP(SP_TARGET_LATENCY, &dummy_ui32);

    if (enctx->rvenc_param.pon > 0) {
        SET_RV11_PROP(SP_SET_PON_START, &enctx->rvenc_param.pon);
    }

    res = PNStream_SetDataCallback(enctx->stream_ref, avctx, NULL, librv11enc_data_callback);
    if (FAILED(res)) {
        av_log(avctx, AV_LOG_ERROR, "Failed to set data callback.\n");
        return AVERROR_EXTERNAL;
    }

    if (avctx->flags & AV_CODEC_FLAG_PASS1) {
        dummy_ui32 = SPCODINGMODE_ANALYSIS;
    } else if (avctx->flags & AV_CODEC_FLAG_PASS2) {
        dummy_ui32 = SPCODINGMODE_FROMFILE;
    } else {
        dummy_ui32 = SPCODINGMODE_ENCODE;
    }
    SET_RV11_PROP(SP_CODING_MODE, &dummy_ui32);

    // if vbr_opt is set to true the encoder can override bitrate settings to remain subjective quality.
    // default is false
    SET_RV11_PROP(SP_RATECONTROL_PLUS, &enctx->rvenc_param.vbr_opt);

    if (avctx->flags & AV_CODEC_FLAG_PASS1 ||
        avctx->flags & AV_CODEC_FLAG_PASS2) {
        if (!enctx->rvenc_param.passlog_file) {
            av_log(avctx, AV_LOG_ERROR, "Invalid passlogfile.\n");
            return AVERROR(EINVAL);
        }
        SET_RV11_PROP(SP_ANALYSIS_FILENAME, (void*)enctx->rvenc_param.passlog_file);
    }

    dummy_ui32 = 1;
    SET_RV11_PROP(SP_CODEC_SETUP_DONE, &dummy_ui32);

    dummy_ui32 = enctx->rvenc_param.max_packet_size;
    res = PNStream_SetOutputPacketSize(enctx->stream_ref, dummy_ui32, dummy_ui32, &dummy_ui32);
    if (FAILED(res)) {
        av_log(avctx, AV_LOG_ERROR, "Failed to set output packet size (%u).\n", dummy_ui32);
        return AVERROR(EINVAL);
    }

    enctx->frame_size = av_image_get_buffer_size(avctx->pix_fmt, avctx->width, avctx->height, 1);

    enctx->frame_buf = av_malloc(enctx->frame_size);
    if (!enctx->frame_buf) {
        av_log(avctx, AV_LOG_ERROR, "Cannot allocate frame buffer.");
        return AVERROR(ENOMEM);
    }

    if (!avctx->extradata_size) {
        uint8_t *q;

        avctx->extradata_size = 14;
        avctx->extradata = av_mallocz(avctx->extradata_size);
        if (!avctx->extradata) {
            avctx->extradata_size = 0;
            return AVERROR(ENOMEM);
        }

        q = avctx->extradata;
        *q++ = 0x01; //spo_extra_flags
        *q++ = 0x08;
        *q++ = 0x10;
        *q++ = 0x00;
        *q++ = 0x40; //codec frontend version (NOT codec version!)
        *q++ = 0x00;
        *q++ = 0x00;
        *q++ = 0x00;
        PNCodec_GetHyperProperty(enctx->codec_ref, q);
    }

    return 0;
}

static int librv11enc_encode(AVCodecContext *avctx, AVPacket*pkt, const AVFrame* frame, int* got_packet)
{
    int ret = 0;
    int nb;
    HXCodecData cd;
    HX_RESULT res;
    LIBRV11EncContext *enctx = avctx->priv_data;
    CodedFrameList* pCodedFrame;

#define RETURN_PACKET(got_p, r) { \
    *got_packet = got_p;          \
    return r;                     \
}

    if (!frame && enctx->last_packet) {
        RETURN_PACKET(0, 0);
    }

    if (frame && enctx->frame_buf && enctx->frame_size > 0) {
        if (use_frame(&enctx->fr_convert)) {
            nb = av_image_copy_to_buffer(enctx->frame_buf, enctx->frame_size,
                                         (const uint8_t * const *)frame->data,
                                         frame->linesize, avctx->pix_fmt,
                                         avctx->width, avctx->height, 1);
            if (nb < 0) {
                RETURN_PACKET(0, nb);
            }

            cd.data         = enctx->frame_buf;
            cd.length       = enctx->frame_size;
            cd.timestamp    = frame->pts;//XXX * 1000 * av_q2d(avctx->time_base);
            cd.flags        = (frame->pict_type == AV_PICTURE_TYPE_I) ? HX_KEYFRAME_FLAG : 0;
            cd.last_packet  = enctx->eos;
            cd.num_segments = 1;
            cd.segments[0].is_valid = 1;
            cd.segments[0].segment_offset = 0;

            res = PNCodec_Input(enctx->codec_ref, &cd);
            if (FAILED(res)) {
                av_log(avctx, AV_LOG_ERROR, "Failed to set input data.\n");
                RETURN_PACKET(0, AVERROR_EXTERNAL);
            }
        }
        next_frame(&enctx->fr_convert);
    } else if (!frame && !enctx->eos) {
        enctx->eos = 1;

        cd.data         = NULL;
        cd.length       = 0;
        cd.timestamp    = 0;
        cd.flags        = 0;
        cd.last_packet  = 1;
        cd.num_segments = 1;
        cd.segments[0].is_valid = 1;
        cd.segments[0].segment_offset = 0;

        res = PNCodec_Input(enctx->codec_ref, &cd);
        if (FAILED(res)) {
            av_log(avctx, AV_LOG_ERROR, "Failed to set input data.\n");
            RETURN_PACKET(0, AVERROR_EXTERNAL);
        }
    }

    pCodedFrame = coded_frame_remove_header(&enctx->coded_frame_list);
    if (pCodedFrame) {
        if (pCodedFrame->pkt.flags & RV_PKT_FLAG_END) {
            pCodedFrame->pkt.flags &= ~RV_PKT_FLAG_END;
            enctx->last_packet = 1;
        }

        ret = av_packet_ref(pkt, &pCodedFrame->pkt);
        if (ret < 0) {
            av_log(enctx, AV_LOG_INFO, "Failed to recieve frame reference.\n");
            free_coded_frame(pCodedFrame);
            RETURN_PACKET(0, ret);
        } else {
            pkt->pts = pCodedFrame->pkt.pts;
            pkt->dts = pCodedFrame->pkt.dts;

            if (pCodedFrame->pkt.flags&AV_PKT_FLAG_KEY) {
                pkt->flags |= AV_PKT_FLAG_KEY;
            }
            free_coded_frame(pCodedFrame);
            RETURN_PACKET(1, 0);
        }
    }

    *got_packet = 0;
    return ret;
}

static av_cold int librv11enc_close(AVCodecContext *avctx)
{
    LIBRV11EncContext *enctx = avctx->priv_data;

    PNStream_Close(enctx->stream_ref);
    PNCodec_Close(enctx->codec_ref);
    enctx->stream_ref = NULL;
    enctx->codec_ref  = NULL;

    free_frame_list(enctx->coded_frame_list);

    av_freep(&enctx->frame_buf);
    av_freep(&enctx->fr_convert.selected_map);
    av_freep(&avctx->extradata);

    return 0;
}

#define OFFSET(x) offsetof(LIBRV11EncContext, x)
#define OFFSETP(x) offsetof(RVEncodeParam, x)
#define OFFSETBASE OFFSET(rvenc_param)
#define VE AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_ENCODING_PARAM

static const AVOption options[] = {
    { "is_lossprotect", "enable loss protection feature",     OFFSETBASE+OFFSETP(loss_protect),      AV_OPT_TYPE_INT,    { .i64 = 0 }, 0,    1, VE },
    { "output_width", "video encoded frame output width",     OFFSETBASE+OFFSETP(out_width),         AV_OPT_TYPE_INT,    { .i64 = 0 }, 0, 4096, VE },
    { "output_height", "video encoded frame output height",   OFFSETBASE+OFFSETP(out_height),        AV_OPT_TYPE_INT,    { .i64 = 0 }, 0, 4096, VE },
    { "rc_strategy", "which ratecontrol method to be used",   OFFSET(bitrate_vbr),                   AV_OPT_TYPE_INT,    { .i64 = 1 }, 0,    1, VE, "rc_strategy" },
    { "bitrate", "", 0,    AV_OPT_TYPE_CONST, {.i64=1}, 0, 0, AV_OPT_FLAG_ENCODING_PARAM, "rc_strategy" },
    { "quality", "", 0,    AV_OPT_TYPE_CONST, {.i64=0}, 0, 0, AV_OPT_FLAG_ENCODING_PARAM, "rc_strategy" },
    { "complexity", "encoding complexity",                    OFFSETBASE+OFFSETP(enc_complexity),    AV_OPT_TYPE_INT,    { .i64 = 75 }, 55, 85, VE, "complexity" },
    { "verylow",  "", 0,  AV_OPT_TYPE_CONST, {.i64=55}, 0, 0, AV_OPT_FLAG_ENCODING_PARAM, "complexity" },
    { "low",      "", 0,  AV_OPT_TYPE_CONST, {.i64=65}, 0, 0, AV_OPT_FLAG_ENCODING_PARAM, "complexity" },
    { "medium",   "", 0,  AV_OPT_TYPE_CONST, {.i64=75}, 0, 0, AV_OPT_FLAG_ENCODING_PARAM, "complexity" },
    { "high",     "", 0,  AV_OPT_TYPE_CONST, {.i64=85}, 0, 0, AV_OPT_FLAG_ENCODING_PARAM, "complexity" },
    { "framerate", "max frame rate value",                    OFFSETBASE+OFFSETP(max_framerate),     AV_OPT_TYPE_INT,    { .i64 = RV_MAX_INPUT_FRAME_RATE }, 0, RV_MAX_INPUT_FRAME_RATE, VE },
    { "max_keyint", "max keyframe interval",                  OFFSETBASE+OFFSETP(max_keyint),        AV_OPT_TYPE_INT,    { .i64 = 5   },   0,   10, VE },
    { "max_latency", "max video latency on start",            OFFSETBASE+OFFSETP(max_start_latency), AV_OPT_TYPE_FLOAT,  { .dbl = 4.0 }, 0.5, 60.0, VE },
    { "vbrquality", "vbr quality value",                      OFFSETBASE+OFFSETP(vbr_quality),       AV_OPT_TYPE_INT,    { .i64 = 60  },   0,  100, VE },
    { "passlogfile", "filename for 2 pass encoding stats",    OFFSETBASE+OFFSETP(passlog_file),      AV_OPT_TYPE_STRING, { .str = "rv11passstats.log" }, 0, 0, VE },
    { "pon", "picture order number",                          OFFSETBASE+OFFSETP(pon),               AV_OPT_TYPE_INT,    { .i64 = 0   },   0, INT_MAX, VE },
    { "vbr_opt", "vbr enabled",                               OFFSETBASE+OFFSETP(vbr_opt),            AV_OPT_TYPE_INT,    { .i64 = 0   },   0,       1, VE, "vbr_opt" },
    { "true",  "", 0,      AV_OPT_TYPE_CONST, {.i64=1}, 0, 0, AV_OPT_FLAG_ENCODING_PARAM, "vbr_opt" },
    { "false", "", 0,      AV_OPT_TYPE_CONST, {.i64=0}, 0, 0, AV_OPT_FLAG_ENCODING_PARAM, "vbr_opt" },
    { NULL },
};

static const AVClass librv11_class = {
    .class_name = "librv11",
    .item_name  = av_default_item_name,
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

AVCodec ff_librv11_encoder = {
    .name           = "librv11",
    .long_name      = NULL_IF_CONFIG_SMALL("librv11 RealVideo 11 (RV60)"),
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = AV_CODEC_ID_RV60,
    .priv_data_size = sizeof(LIBRV11EncContext),
    .init           = librv11enc_init,
    .encode2        = librv11enc_encode,
    .close          = librv11enc_close,
    .capabilities   = AV_CODEC_CAP_DELAY | AV_CODEC_CAP_AUTO_THREADS,
    .pix_fmts       = (const enum AVPixelFormat[]) {
        AV_PIX_FMT_YUV420P,
        AV_PIX_FMT_NONE
    },
    .priv_class     = &librv11_class,
};
