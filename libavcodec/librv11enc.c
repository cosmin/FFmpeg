/*
 * rv11 encoder
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
#ifndef _WIN32
#include <dlfcn.h>
#endif
#include <unistd.h>
#include <inttypes.h>
#include <strings.h>
#include "avcodec.h"
#include "internal.h"
#include "get_bits.h"
#include "libavutil/opt.h"
#include "libavutil/common.h"
#include "libavutil/internal.h"
#include "libavutil/avassert.h"
#include "libavutil/imgutils.h"
#include "librv11util.h"

#if defined(_WIN32)
#include <windows.h>
#define CC __stdcall
#else
#define CC
#endif

#include "librv11enc.h"
#define  ENABLE_RV60_CHANGE 1

#ifndef AV_CODEC_DEFAULT_BITRATE
#define AV_CODEC_DEFAULT_BITRATE 200*1000
#endif

#define AV_PKT_FLAG_STREM_END 0x0080
typedef struct CodedFrameList{
    AVPacket pkt;
    struct CodedFrameList* pNext;
}CodedFrameList;

//This structure was used for frame rate control
#define MAX_INPUT_FRAME_RATE 240
typedef struct FrameRatioConverter{
    uint32_t    ulCount;
    double      dFramerate;
    uint8_t*    pFrameSelectedMap;
    uint32_t    ulInputFrameRate;
}FrameRatioConverter;

typedef struct LIBRMHDEncContext_struct
{
    AVClass *class;
    HXCODEC  hCodecRef;
    HXSTREAM hStreamRef;
    RVENCODE_PARAM rvenc_param;
    HXT_TIME tEncodeRolloverBase;
    uint32_t ulcodec4cc;
    uint32_t inFrameSize;
    uint8_t* inFrameBuff;
    CodedFrameList* coded_frame_list;
    BOOL     bVbrBitrate; //0:VBRQuality;1:VBRBitrate;
    BOOL     bAtEndOfStream; // set to ture when input frame was set to NULL
    BOOL     bLastPacket; // set to true when the last codec frame was poped out
    FrameRatioConverter fr_convert;
    char*    video_mode;
    BOOL     bSeenPacket;
    int64_t  previous_dts;
} LIBRMHDEncContext;

//Frame rate control methods
static int32_t HalfIntervalSearch(uint8_t* pPtr, uint32_t ulBufsize, uint32_t ulNumFrames)
{
    uint32_t ulMidSize = 0;
    uint32_t ulTempNumFrames = 0;
    uint32_t ulLeftNumFrames = 0;
    uint32_t ulRightNumFrames = 0;
    if(ulBufsize < ulNumFrames || ulNumFrames == 0){
        return -1;
    }

    if(ulBufsize%ulNumFrames == 0){
        uint32_t i;
        uint32_t uiDen;
        uiDen = ulBufsize/ulNumFrames;
        for(i=0; i<ulBufsize; i++){
            if((i+1)%uiDen == 0){
                pPtr[i] = 1;
            }
        }
        return 0;
    }else{
        ulTempNumFrames = ulNumFrames/2;
        if(ulTempNumFrames*2 == ulNumFrames){
            ulRightNumFrames = ulTempNumFrames;
            ulLeftNumFrames = ulTempNumFrames;
        }else{
            ulLeftNumFrames = ulTempNumFrames;
            ulRightNumFrames = ulTempNumFrames + 1;
        }
        ulMidSize = ulBufsize/2;
        HalfIntervalSearch(pPtr, ulMidSize, ulLeftNumFrames);
        HalfIntervalSearch(pPtr+ulMidSize, ulBufsize-ulMidSize, ulRightNumFrames);
    }
    return 0;
}

static int32_t InitializeFrConvert(FrameRatioConverter* fr_convert,double dInputFramerate, double dMaxFramerate){
    int32_t ulFramerate = 0;
    int32_t retval = -1;
    if(dInputFramerate < dMaxFramerate || dInputFramerate > MAX_INPUT_FRAME_RATE){
        return -1;
    }
    fr_convert->dFramerate = dMaxFramerate;
    ulFramerate = (int32_t)(dMaxFramerate + 0.5);
    fr_convert->ulInputFrameRate = (int32_t)(dInputFramerate + 0.5);
    fr_convert->ulCount = 0;
    fr_convert->pFrameSelectedMap = (uint8_t*)av_mallocz(fr_convert->ulInputFrameRate);
    retval = HalfIntervalSearch(fr_convert->pFrameSelectedMap, fr_convert->ulInputFrameRate, ulFramerate);
    return retval;
}

static int32_t UseFrame(FrameRatioConverter* fr_convert) {
    return fr_convert->pFrameSelectedMap[fr_convert->ulCount];
}

static void NextFrame(FrameRatioConverter* fr_convert){
    fr_convert->ulCount++;
    if(fr_convert->ulCount >=fr_convert->ulInputFrameRate){
        fr_convert->ulCount = 0;
    }
}

//Frame list manage methods
static void coded_frame_add(void *list, struct CodedFrameList *cx_frame)
{
    struct CodedFrameList **p = list;

    while(*p)
        p = &(*p)->pNext;
    *p = cx_frame;
    cx_frame->pNext = NULL;
}

static CodedFrameList* coded_frame_remove_header(CodedFrameList** list)
{
    struct CodedFrameList *p = *list;
    if(NULL != p){
        *list = p->pNext;
    }
    return p;
}

static av_cold void free_coded_frame(struct CodedFrameList *cx_frame)
{
    av_packet_unref(&cx_frame->pkt);
    av_freep(&cx_frame);
}

static av_cold void free_frame_list(struct CodedFrameList *list)
{
    struct CodedFrameList *p = list;

    while(p)
    {
        list = list->pNext;
        free_coded_frame(p);
        p = list;
    }
}

static av_cold int CC librmhdenc_data_callback(HXSTREAM pFrom,HXSTREAM streamRef,HXCODEC_DATA* pData)
{
    int ret = 0;
    int pict_type;
    int64_t current_pts;
    int64_t current_dts;
    AVCodecContext* avctx = (AVCodecContext*)pFrom;
    LIBRMHDEncContext* pThis = (LIBRMHDEncContext*)(avctx->priv_data);
    if(pData!=NULL)
    {
        if(pData->dataLength!=0)
        {
            int numPackets = 0;
            CodedFrameList* pNode = av_mallocz(sizeof(CodedFrameList));
            int bufLength = 0;

            if(pData->numSegments==1){
                //to rv11 numSegments should be 1
                numPackets = get_segments_number(pData->dataLength);
                if(numPackets>127 || numPackets < 0) {
                    av_log(pFrom,AV_LOG_ERROR,"%s:%u frame size (%u) is too large, numPacket=%d\n",__FILE__, __LINE__, pData->dataLength, numPackets);
                }
            } else {
                av_log(pFrom,AV_LOG_ERROR,"%s:%u to rv11 the number of packet should be 1, but numPacket=%u\n",__FILE__, __LINE__, pData->numSegments);
            }

            //allocate extra numPacket*8+1  byte for real video head saving segment information
            bufLength = pData->dataLength + numPackets*8 +1;
            if(ret=av_new_packet(&pNode->pkt,bufLength)<0)
            {
                av_log(pFrom,AV_LOG_ERROR,"%s alloc_packet2 failed!\n",__FUNCTION__);
                return ret;
            }
            write_rv_frame_head(pNode->pkt.data, numPackets);
            memcpy((pNode->pkt.data+ numPackets*8 +1) ,pData->data,pData->dataLength);

            if(pData->flags&HX_KEYFRAME_FLAG)
            {
                pNode->pkt.flags |= AV_PKT_FLAG_KEY;
            }

            if(pData->lastPacket)
            {
                pNode->pkt.flags |= AV_PKT_FLAG_STREM_END;
            }


#if ENABLE_RV60_CHANGE
            pict_type = ((*(pData->data + 1)) >> 6) & 0x03;
            current_pts = pData->timestamp;
            if(pThis->bSeenPacket == FALSE) {
                /* first frame */
                pThis->bSeenPacket = TRUE;
                current_dts = current_pts - 1;
            } else {
                if (pict_type == 2) {
                    /* B-frame */
                    current_dts = current_pts;
                }
                else {
                    /* reference-frame */
                    current_dts = pThis->previous_dts + 1;
                }
            }
            pNode->pkt.pts = current_pts;
            pNode->pkt.dts = current_dts;
            pThis->previous_dts = current_dts;
#else
            pNode->pkt.dts = pNode->pkt.pts = pData->timestamp*av_q2d(av_inv_q(avctx->time_base))/1000.0;
#endif
            coded_frame_add((void*)(&pThis->coded_frame_list),pNode);

            return HXR_OK;
        }
        else
        {
            //av_log(pFrom,AV_LOG_INFO,"%s get DataLen==0!\n",__FUNCTION__);
        }
    }
    else
    {
        av_log(pFrom,AV_LOG_INFO,"%s get pData==NULL!\n",__FUNCTION__);
    }
    return HXR_OK;
}

#ifndef _WIN32
static const char* lib_name= "librv11enc.so";
#endif
static const char* passlogfile = "rv11passtats.log";

static const char* ksz_encoding_veryhigh_complevel = "veryhigh";
static const char* ksz_encoding_high_complevel     = "high";
static const char* ksz_encoding_medium_complevel   = "medium";
static const char* ksz_encoding_low_complevel      = "low";
static const char* ksz_encoding_verylow_complevel  = "verylow";

static const char* ksz_encoding_typeVBRBitrate= "VBRBitrate";
static const char* ksz_encoding_typeVBRQuality= "VBRQuality";

static const char* ksz_encoding_framerate_mode_sharp  = "sharp";
static const char* ksz_encoding_framerate_mode_smooth  = "smooth";

static av_cold int get_encoding_complevel(const char* pszcompleveldesc)
{
    int ulEncComplevel = 0;
    if(!pszcompleveldesc)
        return -1;

    if(strcasecmp(pszcompleveldesc,ksz_encoding_veryhigh_complevel)==0)
    {
        ulEncComplevel = 85 ;
    }
    else if(strcasecmp(pszcompleveldesc,ksz_encoding_high_complevel)==0)
    {
        ulEncComplevel = 85;
    }
    else if(strcasecmp(pszcompleveldesc,ksz_encoding_medium_complevel)==0)
    {
        ulEncComplevel = 75;
    }
    else if(strcasecmp(pszcompleveldesc,ksz_encoding_low_complevel)==0)
    {
        ulEncComplevel = 65;
    }
    else if(strcasecmp(pszcompleveldesc,ksz_encoding_verylow_complevel)==0)
    {
        ulEncComplevel = 55;
    }
    else
    {
        av_log(NULL,AV_LOG_ERROR,"Invalid encoder parameter to complexity level!\n");
        return -1;
    }
    return ulEncComplevel;
}

static av_cold BOOL is_encoding_type_vbrbitrate(const char* pszencodetypedesc)
{
    if(!pszencodetypedesc)
        return FALSE;

    if(strcasecmp(pszencodetypedesc,ksz_encoding_typeVBRBitrate)==0)
    {
        return TRUE;
    }
    else if(strcasecmp(pszencodetypedesc,ksz_encoding_typeVBRQuality)==0)
    {
        return FALSE;
    }
    else
    {
        av_log(NULL,AV_LOG_ERROR,"Invalid encoder parameter to encoding type!\n");
        return FALSE;
    }

    return FALSE;
}

static av_cold int get_resize_quality(const char* pszresizequality)
{
    if(!pszresizequality)
        return -1;

    if(strcasecmp(pszresizequality,"high")==0)
    {
        return 1;
    }
    else if(strcasecmp(pszresizequality,"fast")==0)
    {
        return 0;
    }
    else
    {
        av_log(NULL,AV_LOG_ERROR,"Invalid encoder parameter to resize quality!\n");
        return -1;
    }

    return 1;
}

#define VGA_RESOLUTION     (640*480)
#define NTSC_RESOLUTION    (720*540)
#define XGA_RESOLUTION     (1024*768)
#define WVGA_RESOLUTION    (1280*768)
#define HDTV_RESOLUTION    (1920*1080)
#define UHD1_RESOLUTION    (3840*2160)
#define UHD2_RESOLUTION    (4096*2160)

static UINT32 get_default_bitrate_by_resolution(RVENCODE_PARAM *param)
{
   UINT32 default_bitrate = 1000000;
   int output_resolution = param->ulInputWidth * param->ulInputHeight;
   if( output_resolution >= VGA_RESOLUTION && output_resolution < NTSC_RESOLUTION)
   {
       default_bitrate = 800000;
   }
   else if(output_resolution >= NTSC_RESOLUTION && output_resolution < XGA_RESOLUTION)
   {
       default_bitrate = 1500000;
   }
   else if(output_resolution >= XGA_RESOLUTION && output_resolution < WVGA_RESOLUTION)
   {
       default_bitrate = 3000000;
   }
   else if(output_resolution >= WVGA_RESOLUTION && output_resolution < HDTV_RESOLUTION)
   {
       default_bitrate = 5000000;
   }
   else if(output_resolution >= HDTV_RESOLUTION && output_resolution < UHD1_RESOLUTION)
   {
       default_bitrate = 5000000;
   }
   else if(output_resolution >= UHD1_RESOLUTION && output_resolution < UHD2_RESOLUTION)
   {
       default_bitrate = 10000000;
   }
   else if(output_resolution >= UHD2_RESOLUTION)
   {
       default_bitrate = 10000000;
   }
   return default_bitrate;
}

static av_cold int librmhdenc_init(AVCodecContext *avctx)
{
    LIBRMHDEncContext *enctx = (LIBRMHDEncContext*)avctx->priv_data;
    HX_RESULT res = HXR_OK;
    ULONG32 cpu_low;
    HX_MOF hxInput,hxOutput;
    HX_FORMAT_VIDEO  mofin;
    HX_FORMAT_VIDEO  mofout;
    float framerate = 30.0;
    HXCODEC_INIT ci;
    float fMaxFramerate = 0;
    uint32_t ulCheckFrameSkip = 0;
    uint32_t ulImageQuality = 50;
    uint32_t isLive = 0;
    UINT32 ulCodingMode = SPCODINGMODE_ENCODE;
    BOOL bDone = TRUE;
    uint32_t ulTargetPacketSize = 0;
    uint8_t  *q = NULL;

    enctx->hCodecRef = NULL;
    enctx->hStreamRef= NULL;
    enctx->inFrameBuff= NULL;
    enctx->coded_frame_list = NULL;
    enctx->bAtEndOfStream = FALSE;
    enctx->bLastPacket        = FALSE;
    enctx->bSeenPacket = FALSE;
    enctx->previous_dts = 0;

    // align the input width&hight to 4 if there are not
    if((avctx->width % 4)!= 0)
    {
        avctx->width = (avctx->width& (~3) );
    }
    if((avctx->height %4) != 0)
    {
        avctx->height = (avctx->height& (~3) );
    }

    res = PNCodec_QueryMediaFormat(NULL,&hxInput,&hxOutput,0);
    if(FAILED(res))
    {
        av_log(avctx,AV_LOG_ERROR,"Error,failed to querymediaformat before PNCodec_Open!\n");
        return HXR_FAIL;
    }

    res = PNCodec_Open(hxOutput.submoftag,&enctx->hCodecRef);
    if(FAILED(res))
    {
        av_log(avctx,AV_LOG_ERROR,"Error,failed to PNCodec_Open!\n");
        return HXR_FAIL;
    }

    if(SUCCEEDED(res))
    {
        enctx->ulcodec4cc = hxOutput.submoftag;
        //init the rv11 encoder default configure params
        enctx->rvenc_param.isLossProtect = FALSE;

        if(avctx->width > 0)
        {
            enctx->rvenc_param.ulInputWidth = avctx->width;
            enctx->rvenc_param.ulOutputWidth = avctx->width;
        }
        if(avctx->height > 0)
        {
            enctx->rvenc_param.ulInputHeight= avctx->height;
            enctx->rvenc_param.ulOutputHeight= avctx->height;
        }

        if(enctx->rvenc_param.szEncodeType)
        {
            enctx->bVbrBitrate = is_encoding_type_vbrbitrate(enctx->rvenc_param.szEncodeType);
            if(enctx->bVbrBitrate)
            {
                enctx->rvenc_param.eEncodeType = ENCODE_VBR_BITRATE;
            }
            else
            {
                enctx->rvenc_param.eEncodeType = ENCODE_VBR_QUALITY;
            }
        }

        if(avctx->flags&AV_CODEC_FLAG_PASS1)
        {
            enctx->rvenc_param.eEncodingMode = ENCODE_CREATE_ANALYSIS;
        }
        else if(avctx->flags&AV_CODEC_FLAG_PASS2)
        {
            enctx->rvenc_param.eEncodingMode = ENCODE_USING_ANALYSIS;
        }
        else
        {
            enctx->rvenc_param.eEncodingMode = ENCODE_SINGLE_PASS;
        }


        if(enctx->rvenc_param.szEncComplevel)
        {
            enctx->rvenc_param.ulEncComplevel = get_encoding_complevel(enctx->rvenc_param.szEncComplevel);
            if(-1 == enctx->rvenc_param.ulEncComplevel)
                enctx->rvenc_param.ulEncComplevel= 75; //medium level encode complex level
        }

        if(enctx->rvenc_param.ulAvgBitrate > enctx->rvenc_param.ulMaxBitrate)
        {
            enctx->rvenc_param.ulMaxBitrate = enctx->rvenc_param.ulAvgBitrate;
        }

        if(enctx->rvenc_param.ulMaxBitrate <= 0 || enctx->rvenc_param.ulAvgBitrate <= 0)
        {
          if(enctx->rvenc_param.ulAvgBitrate >0 && enctx->rvenc_param.ulMaxBitrate <=0)
          {
             enctx->rvenc_param.ulMaxBitrate = enctx->rvenc_param.ulAvgBitrate;

          }
          else if(enctx->rvenc_param.ulMaxBitrate >0 && enctx->rvenc_param.ulAvgBitrate <=0)
          {
             enctx->rvenc_param.ulAvgBitrate = enctx->rvenc_param.ulMaxBitrate;

          }
          else if(enctx->rvenc_param.ulAvgBitrate <=0 && enctx->rvenc_param.ulMaxBitrate <=0 &&avctx->bit_rate <= 200000)//user no set the bitrate,ffmpeg default value 200k
          {
            enctx->rvenc_param.ulMaxBitrate = get_default_bitrate_by_resolution(&enctx->rvenc_param);
            enctx->rvenc_param.ulAvgBitrate = enctx->rvenc_param.ulMaxBitrate;
          }
        }

        //if user set -b:v option, use set -b:v parameter to override -avgrate option settings
        if (avctx->bit_rate > 0 && avctx->bit_rate != AV_CODEC_DEFAULT_BITRATE) {
            enctx->rvenc_param.ulAvgBitrate = avctx->bit_rate;
            enctx->rvenc_param.ulMaxBitrate = avctx->bit_rate;
        }

        enctx->rvenc_param.ulMaxPacketSize= 15000;

        if(enctx->rvenc_param.ulMaxFramerate <= 0)
        {
            enctx->rvenc_param.ulMaxFramerate = MAX_INPUT_FRAME_RATE; /* caps framerate at 240 fps */
        }

        if(enctx->rvenc_param.ulVbrQuality <= 0)
        {
            enctx->rvenc_param.ulVbrQuality = 70;
        }

        if(enctx->bVbrBitrate
           &&(avctx->flags&AV_CODEC_FLAG_PASS1))
        {
            enctx->rvenc_param.ulVbrQuality = 25;
        }

        if(enctx->rvenc_param.szResizeQuality)
        {
            enctx->rvenc_param.ulResizeQuality=get_resize_quality(enctx->rvenc_param.szResizeQuality);
        }

        if(enctx->rvenc_param.ulMaxKeyFrameInterval <= 0)
        {
            enctx->rvenc_param.ulMaxKeyFrameInterval = 5; //default value for rv11 encoder
        }

        //set input video frames for I420
        memset(&mofin,0,sizeof(mofin));
        mofin.cbLength = sizeof(HX_FORMAT_VIDEO);
        mofin.moftag = HX_MEDIA_VIDEO;
        mofin.submoftag = HX_YUV420_ID;
        mofin.uiWidth = enctx->rvenc_param.ulInputWidth;
        mofin.uiHeight= enctx->rvenc_param.ulInputHeight;
        mofin.uiBitCount = 12;
        mofin.uiPadWidth = 0;
        mofin.uiPadHeight= 0;
        mofin.moftag = HX_MEDIA_VIDEO;

        // we should use the input frame rate here
        if(( avctx->framerate.num > 0 ) &&(avctx->framerate.den > 0)){
            framerate = (float)avctx->framerate.num/(float)avctx->framerate.den;
        }

        if(enctx->rvenc_param.ulMaxFramerate > 0 && enctx->rvenc_param.ulMaxFramerate < framerate)
        {
            avctx->framerate.num = enctx->rvenc_param.ulMaxFramerate;
            avctx->framerate.den = 1;
        }

        if(InitializeFrConvert(&enctx->fr_convert,
            framerate,
            enctx->rvenc_param.ulMaxFramerate)){
            InitializeFrConvert(&enctx->fr_convert,framerate,framerate);
        }
        mofin.framesPerSecond =(int) ((enctx->fr_convert.dFramerate) * (1L << 16) + 0.5);

        //set output video frames
        memset(&mofout,0,sizeof(mofout));
        mofout.cbLength = sizeof(mofout);
        mofout.moftag = HX_MEDIA_VIDEO;
        mofout.submoftag = enctx->ulcodec4cc;
        mofout.uiWidth = enctx->rvenc_param.ulOutputWidth;
        mofout.uiHeight= enctx->rvenc_param.ulOutputHeight;
        mofout.uiBitCount = mofin.uiBitCount;
        mofout.uiPadWidth = 0;
        mofout.uiPadHeight= 0;
        mofout.moftag = HX_MEDIA_VIDEO;
        mofout.framesPerSecond = mofin.framesPerSecond;

        ci.pInMof = (HX_MOF*)&mofin;
        ci.pOutMof= (HX_MOF*)&mofout;
        ci.memoryRef = NULL;

        res = PNCodec_SetProperty(enctx->hCodecRef,CP_ENC_SUGGEST_NUMBER_OF_THREAD,&enctx->rvenc_param.ulMaxThreads);
        if(FAILED(res))
        {
          av_log(avctx,AV_LOG_ERROR,"Error,failed to PNCodec_SetProperty CP_ENC_SUGGEST_NUMBER_OF_THREAD!\n");
          return HXR_FAIL;
        }

        if(enctx->rvenc_param.isGPUAccelerate==1)
          res = PNCodec_SetProperty(enctx->hCodecRef,CP_ENC_ENABLE_GPU_ACCELEARATION,NULL);
        else
          res = PNCodec_SetProperty(enctx->hCodecRef,CP_ENC_DISABLE_GPU_ACCELEARATION,NULL);

        if(FAILED(res))
        {
          av_log(avctx,AV_LOG_ERROR,"Error,failed to PNCodec_SetProperty CP_GPU_ACCELERATE!\n");
          return HXR_FAIL;
        }

        res = PNCodec_StreamOpen(enctx->hCodecRef,&enctx->hStreamRef,&ci);
        if(FAILED(res))
        {
            av_log(avctx,AV_LOG_ERROR,"Error,failed to PNCodec_StreamOpen!\n");
            return HXR_FAIL;
        }

        if(enctx->bVbrBitrate&&!(avctx->flags&AV_CODEC_FLAG_PASS1))
        {
            res = PNStream_SetProperty(enctx->hStreamRef,SP_BITRATE,&enctx->rvenc_param.ulAvgBitrate);
            if(FAILED(res))
            {
                av_log(avctx,AV_LOG_ERROR,"Error,failed to PNStream_SetProperty SP_BITRATE!\n");
                return HXR_FAIL;
            }
        }
        else
        {
            res = PNStream_SetProperty(enctx->hStreamRef,SP_QUALITY,&enctx->rvenc_param.ulVbrQuality);
            if(FAILED(res))
            {
                av_log(avctx,AV_LOG_ERROR,"Error,failed to PNStream_SetProperty SP_QUALITY!\n");
                return HXR_FAIL;
            }
        }

        if(SUCCEEDED(res))
        {
            res = PNStream_SetProperty(enctx->hStreamRef,SP_MAXBITRATE,&enctx->rvenc_param.ulMaxBitrate);
            if(FAILED(res))
            {
                av_log(avctx,AV_LOG_ERROR,"Error,failed to PNStream_SetProperty SP_MAXBITRATE!\n");
                return HXR_FAIL;
            }

            fMaxFramerate = enctx->rvenc_param.ulMaxFramerate;
            res = PNStream_SetProperty(enctx->hStreamRef,SP_MAX_FRAMERATE,&fMaxFramerate);
            if(FAILED(res))
            {
                av_log(avctx,AV_LOG_ERROR,"Error,failed to PNStream_SetProperty SP_MAX_FRAMERATE!\n");
                return HXR_FAIL;
            }

            res = PNStream_SetProperty(enctx->hStreamRef,SP_CHECK_FRAME_SKIP,&ulCheckFrameSkip);
            if(FAILED(res))
            {
                av_log(avctx,AV_LOG_ERROR,"Error,failed to PNStream_SetProperty SP_CHECK_FRAME_SKIP!\n");
                return HXR_FAIL;
            }

            //video mode: normal, sharp, smooth, slideshow
            if(!strcasecmp(enctx->video_mode, ksz_encoding_framerate_mode_smooth)){
                ulImageQuality = 100;
            }else if(!strcasecmp(enctx->video_mode, ksz_encoding_framerate_mode_sharp)){
                ulImageQuality = 1;
            }
            res = PNStream_SetProperty(enctx->hStreamRef,SP_QUALITY_MOTION,&ulImageQuality);
            if(FAILED(res))
            {
                av_log(avctx,AV_LOG_ERROR,"Error,failed to PNStream_SetProperty SP_QUALITY_MOTION!\n");
                return HXR_FAIL;
            }

            res = PNStream_SetProperty(enctx->hStreamRef,SP_LIVE,&isLive);

            if(FAILED(res))
            {
                av_log(avctx,AV_LOG_ERROR,"Error,failed to PNStream_SetProperty SP_LIVE!\n");
                return HXR_FAIL;
            }

            if(SUCCEEDED(res))
            {
                uint32_t ulMaxKeyFrameInterval = enctx->rvenc_param.ulMaxKeyFrameInterval*1000;
                res = PNStream_SetProperty(enctx->hStreamRef,SP_KEYFRAMERATE,&ulMaxKeyFrameInterval);
            }

            if(SUCCEEDED(res))
            {
                uint32_t parm = enctx->rvenc_param.isLossProtect==TRUE ? 1 : 0;
                res = PNStream_SetProperty(enctx->hStreamRef,SP_ECC,&parm);
                if(FAILED(res))
                {
                    av_log(avctx,AV_LOG_ERROR,"Error,failed to PNStream_SetProperty SP_ECC!\n");
                    return HXR_FAIL;
                }

                res = PNStream_SetProperty(enctx->hStreamRef,SP_SET_FAST_LEVEL,&enctx->rvenc_param.ulEncComplevel);
                if(FAILED(res))
                {
                    av_log(avctx,AV_LOG_ERROR,"Error,failed to PNStream_SetProperty SP_SET_FAST_LEVEL!\n");
                    return HXR_FAIL;
                }
            }

            if(SUCCEEDED(res))
            {
                //max setup latency 60, min startup latency 0.5, normal latency 4, medium latency 2, cbr low latency 1, vbr low latency 0.5.
                uint32_t ulVBRMaxLatencyInMillsec = (uint32_t)enctx->rvenc_param.lfMaxStartLatency*1000;
                res = PNStream_SetProperty(enctx->hStreamRef,SP_TARGET_LATENCY,&ulVBRMaxLatencyInMillsec);
                if(FAILED(res))
                {
                    av_log(avctx,AV_LOG_ERROR,"Error,failed to PNStream_SetProperty SP_TARGET_LATENCY!\n");
                    return HXR_FAIL;
                }
            }

            if(SUCCEEDED(res))
            {
                res = PNStream_SetProperty(enctx->hStreamRef,SP_RESIZING_ACCURACY,&enctx->rvenc_param.ulResizeQuality);
                if(FAILED(res))
                {
                    av_log(avctx,AV_LOG_ERROR,"Error,failed to PNStream_SetProperty SP_RESIZE_ACCURACY!\n");
                    return HXR_FAIL;
                }
            }

            if(SUCCEEDED(res) && (enctx->rvenc_param.ulPon > 0 /*|| (avctx->resume_transcoding && avctx->nb_vframes>0)*/))
            {
                uint32_t ulPon = 0;
                if(enctx->rvenc_param.ulPon > 0){
                    ulPon = enctx->rvenc_param.ulPon;
                }

                res = PNStream_SetProperty(enctx->hStreamRef,SP_SET_PON_START,&ulPon);
            }

            res = PNStream_SetDataCallback(enctx->hStreamRef,avctx,NULL,librmhdenc_data_callback);

            if(FAILED(res))
            {
                av_log(avctx,AV_LOG_ERROR,"Error,failed to PNStream_SetDataCallback!\n");
                return HXR_FAIL;
            }

            if(avctx->flags&AV_CODEC_FLAG_PASS1)
            {
                ulCodingMode = SPCODINGMODE_ANALYSIS;
            }
            else if(avctx->flags&AV_CODEC_FLAG_PASS2)
            {
                ulCodingMode = SPCODINGMODE_FROMFILE;
            }

            res = PNStream_SetProperty(enctx->hStreamRef,SP_CODING_MODE,&ulCodingMode);
            if(FAILED(res))
            {
                av_log(avctx,AV_LOG_ERROR,"Error,failed to PNStream_SetProperty first SP_CODING_MODE!\n");
                return HXR_FAIL;
            }
        }

        //rate control option feature
        //only available in ABR mode
        //if rate control is 0, bitrate priority, encoder needs to satisfy user's bitrate requirement.
        //if rate control is 1, quality priority, encoder can override user's bitrate settings if it hurts subjective quality.
        //if rate control is 2, only work in single pass,  CBR
        //Default value of the rate control is 0

        if(SUCCEEDED(res) && (enctx->rvenc_param.eEncodeType == ENCODE_VBR_BITRATE))
        {
            uint32_t parm = enctx->rvenc_param.ulVbrbitrateOpt;
            if(avctx->flags&AV_CODEC_FLAG_PASS2 && parm==2)
            {
                av_log(avctx,AV_LOG_WARNING,"Warning! DO NOT set rate control option to 2 in pass2 mode, change it to 0\n");
                parm = 0;
            }
            /*for single pass, set the option to 0 if user sets it to 1*/
            else if(parm==1 && !(avctx->flags&AV_CODEC_FLAG_PASS2) && !(avctx->flags&AV_CODEC_FLAG_PASS1))
            {
                parm = 0;
            }

            res = PNStream_SetProperty(enctx->hStreamRef,SP_RATECONTROL_PLUS,&parm);
            if(FAILED(res))
            {
                av_log(avctx,AV_LOG_ERROR,"Error,failed to PNStream_SetProperty SP_RATECONTROL_PLUS!\n");
                return HXR_FAIL;
            }
        }

        if(avctx->flags&AV_CODEC_FLAG_PASS1||avctx->flags&AV_CODEC_FLAG_PASS2)
        {
            char szPasslogfile[512] = {0};
            if(!enctx->rvenc_param.szPasslogFile)
            {
#ifdef _WIN32
                sprintf(szPasslogfile,"%s",passlogfile);
#else
                sprintf(szPasslogfile,"%s/%s",strLibDir,passlogfile);
#endif
            }
            else
            {
                strcpy(szPasslogfile,enctx->rvenc_param.szPasslogFile);
            }


            res = PNStream_SetProperty(enctx->hStreamRef,SP_ANALYSIS_FILENAME,(void*)szPasslogfile);
            if(FAILED(res))
            {
                av_log(avctx,AV_LOG_ERROR,"Error,failed to PNStream_SetProperty SP_ANALYSIS_FILENAME!\n");
                return HXR_FAIL;
            }
        }

        if(SUCCEEDED(res))
        {
            res = PNStream_SetProperty(enctx->hStreamRef,SP_CODEC_SETUP_DONE,&bDone);
            if(FAILED(res))
            {
                av_log(avctx,AV_LOG_ERROR,"%s failed to setup rv11 video codec!\n",__FUNCTION__);
                return HXR_FAIL;
            }

            ulTargetPacketSize = enctx->rvenc_param.ulMaxPacketSize;
            res = PNStream_SetOutputPacketSize(enctx->hStreamRef,ulTargetPacketSize,ulTargetPacketSize,&ulTargetPacketSize);
        }

        enctx->tEncodeRolloverBase = 0;

#if FF_API_CODED_FRAME
FF_DISABLE_DEPRECATION_WARNINGS
        avctx->coded_frame = av_frame_alloc();
        if(!avctx->coded_frame)
        {
            av_log(avctx,AV_LOG_ERROR,"alloc for coded_frame error!");
            return AVERROR(ENOMEM);
        }
FF_ENABLE_DEPRECATION_WARNINGS
#endif

        enctx->inFrameSize = av_image_get_buffer_size(avctx->pix_fmt, avctx->width, avctx->height, 1);

        enctx->inFrameBuff = av_malloc(enctx->inFrameSize);
        if(!enctx->inFrameBuff)
        {
            av_log(avctx,AV_LOG_ERROR,"alloc for inFrameBuff error!");
            return AVERROR(ENOMEM);
        }
    }

    if(avctx->extradata_size == 0) {
        avctx->extradata_size = 14;
        avctx->extradata = av_mallocz(avctx->extradata_size + AV_INPUT_BUFFER_PADDING_SIZE);
        if (!avctx->extradata) {
            avctx->extradata_size = 0;
            return  AVERROR(ENOMEM);
        }
        q = avctx->extradata;
        *q++ = 0x01; //spo_extra_flags
        *q++ = 0x08;
        *q++ = 0x10;
        *q++ = 0x00;
        *q++ = 0x40; //codec version, RV10=10, rv20=20,rv40=40 rv60=60,
        *q++ = 0x00;
        *q++ = 0x00;
        *q++ = 0x00;
        PNCodec_GetHyperProperty(enctx->hCodecRef, q);
    }

    return 0;
}

static int librmhdenc_encode(AVCodecContext *avctx, AVPacket*pkt, const AVFrame* frame, int* got_packet)
{
    int ret = 0;
    HXCODEC_DATA cd;
    HX_RESULT res = HXR_OK;
    HXT_TIME tStartTime=0;
    BOOL bForceKeyFrame = FALSE;
    LIBRMHDEncContext* enctx = avctx->priv_data;
    CodedFrameList* pCodedFrame;

    if((!frame)&&(enctx->bLastPacket))
    {
        *got_packet = 0;
        return 0;
    }

    if(frame&&enctx->inFrameBuff&&enctx->inFrameSize >0)
    {
        if(UseFrame(&enctx->fr_convert))
        {
            if(ret = av_image_copy_to_buffer(enctx->inFrameBuff, enctx->inFrameSize, (const uint8_t * const *)frame->data,
                      frame->linesize, avctx->pix_fmt, avctx->width, avctx->height, 1) < 0)
            {
                *got_packet = 0;
                return ret;
            }

            tStartTime = frame->pts;

            if(tStartTime-enctx->tEncodeRolloverBase > kMaxUint32)
            {
                av_log(avctx,AV_LOG_INFO,"Detected timestamp rollover!\n");
                enctx->tEncodeRolloverBase += kMaxUint32;
                bForceKeyFrame = TRUE;
            }

            bForceKeyFrame = frame->pict_type==AV_PICTURE_TYPE_I ? TRUE : FALSE;

          if(PNCodec_Input !=NULL)
            {
                cd.data = enctx->inFrameBuff;
                cd.dataLength = enctx->inFrameSize;
                cd.timestamp = tStartTime;
                cd.flags = bForceKeyFrame?HX_KEYFRAME_FLAG:0;
                cd.lastPacket = enctx->bAtEndOfStream;
                cd.numSegments = 1;
                cd.Segments[0].bIsValid = TRUE;
                cd.Segments[0].ulSegmentOffset = 0;

                res = PNCodec_Input(enctx->hCodecRef,&cd);

                if(res != HXR_OK)
                {
                    av_log(avctx, AV_LOG_INFO,"%s fpHXCodec_Input return failed res=%d!\n", __FUNCTION__,res);
                    *got_packet = 0;
                    return -1;
                }
            }
        }
        else
        {
            ret = 0;
        }
        NextFrame(&enctx->fr_convert);
    }else if((NULL == frame)&&(!enctx->bAtEndOfStream)){

        enctx->bAtEndOfStream = TRUE;
        // stream is end
        if(PNCodec_Input !=NULL)
        {
            cd.data = NULL;
            cd.dataLength = 0;
            cd.timestamp = tStartTime;
            cd.flags = bForceKeyFrame?HX_KEYFRAME_FLAG:0;
            cd.lastPacket = TRUE;
            cd.numSegments = 1;
            cd.Segments[0].bIsValid = TRUE;
            cd.Segments[0].ulSegmentOffset = 0;

            res = PNCodec_Input(enctx->hCodecRef,&cd);

            if(res != HXR_OK)
            {
                av_log(avctx, AV_LOG_INFO,"%s fpHXCodec_Input return failed res=%d!\n", __FUNCTION__,res);
                *got_packet = 0;
                return -1;
            }
        }
    }

    pCodedFrame = coded_frame_remove_header(&enctx->coded_frame_list);
    if(pCodedFrame)
    {
        if(pCodedFrame->pkt.flags &  AV_PKT_FLAG_STREM_END){
            pCodedFrame->pkt.flags &= ~AV_PKT_FLAG_STREM_END;
            enctx->bLastPacket = TRUE;
        }

        ret = av_copy_packet(pkt,&pCodedFrame->pkt);
        if(ret <0)
        {
            av_log(enctx,AV_LOG_INFO,"av_copy_packet failed!\n");
            *got_packet = 0;
            free_coded_frame(pCodedFrame);
            return ret;
        }
        else
        {
            pkt->pts = pCodedFrame->pkt.pts;
            pkt->dts = pCodedFrame->pkt.dts;

            if(pCodedFrame->pkt.flags&AV_PKT_FLAG_KEY)
            {
                pkt->flags |= AV_PKT_FLAG_KEY;
#if FF_API_CODED_FRAME
FF_DISABLE_DEPRECATION_WARNINGS
                avctx->coded_frame->pict_type = AV_PICTURE_TYPE_I;
FF_ENABLE_DEPRECATION_WARNINGS
#endif
            }
            *got_packet = 1;
            free_coded_frame(pCodedFrame);
            return 0;
        }
    }

    *got_packet = 0;
    return ret;
}

static av_cold int librmhdenc_close(AVCodecContext *avctx)
{
    LIBRMHDEncContext* enctx = avctx->priv_data;
    if(PNStream_Close)
    {
        PNStream_Close(enctx->hStreamRef);
        enctx->hStreamRef = NULL;
    }

    if(PNCodec_Close)
    {
        PNCodec_Close(enctx->hCodecRef);
        enctx->hCodecRef = NULL;
    }

    if(enctx->inFrameBuff)
    {
        av_free(enctx->inFrameBuff);
        enctx->inFrameBuff = NULL;
    }

    if(enctx->coded_frame_list)
    {
        free_frame_list(enctx->coded_frame_list);
    }

    if(enctx->fr_convert.pFrameSelectedMap)
    {
        av_free(enctx->fr_convert.pFrameSelectedMap);
    }

#if FF_API_CODED_FRAME
FF_DISABLE_DEPRECATION_WARNINGS
    if(avctx->coded_frame)
    {
        av_frame_free(&avctx->coded_frame);
    }
FF_ENABLE_DEPRECATION_WARNINGS
#endif

    if(avctx->extradata){
        av_freep(&avctx->extradata);
    }
    return 0;
}

#define OFFSET(x) offsetof(LIBRMHDEncContext,x)
#define OFFSETP(x) offsetof(RVENCODE_PARAM, x)
#define OFFSETBASE OFFSET(rvenc_param)
#define VE AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_ENCODING_PARAM

static const AVOption options[] =
{
    { "is_lossprotect","enable loss protection feature",OFFSETBASE+OFFSETP(isLossProtect),AV_OPT_TYPE_INT, { .i64 = 0  }, 0,   1, VE  },
    { "gpu_accelerate","enable gpu acceleration",OFFSETBASE+OFFSETP(isGPUAccelerate),AV_OPT_TYPE_INT, { .i64 = 0  }, 0,   1, VE  },
    { "maxthreads","max NO. of threads for encoder",OFFSETBASE+OFFSETP(ulMaxThreads),AV_OPT_TYPE_INT, { .i64 = 0  }, 0, INT_MAX, VE },
    { "rc_strategy","which ratecontrol method to be used",OFFSETBASE+OFFSETP(szEncodeType),AV_OPT_TYPE_STRING, { .str = "VBRBitrate"  }, 0, 0, VE  },
    { "enc_complexity","rv11 encoding complexity",OFFSETBASE+OFFSETP(szEncComplevel),AV_OPT_TYPE_STRING, { .str="medium"  }, 0, 0, VE  },
    { "framerate_mode","which framerate mode to be used",OFFSET(video_mode),AV_OPT_TYPE_STRING, { .str = "normal"  }, 0,  0, VE  },
    { "maxvideobuf","max video buffertime",OFFSETBASE+OFFSETP(lfMaxStartLatency),AV_OPT_TYPE_FLOAT, { .dbl = 4  }, 0.5,  60, VE  },
    { "vbrquality","vbr quality value",OFFSETBASE+OFFSETP(ulVbrQuality),AV_OPT_TYPE_INT, { .i64 = 60  }, 0, 100, VE },
    { "framerate","max frame rate value",OFFSETBASE+OFFSETP(ulMaxFramerate),AV_OPT_TYPE_INT, { .i64 = 0  }, 0, 60, VE },
    { "keyint_max","max keyframe interval",OFFSETBASE+OFFSETP(ulMaxKeyFrameInterval),AV_OPT_TYPE_INT, { .i64 = 5  }, 0,  10, VE },
    { "output_width","video encoded frame output width",OFFSETBASE+OFFSETP(ulOutputWidth),AV_OPT_TYPE_INT, { .i64 = 0  }, 0, 4096, VE },
    { "output_height","video encoded frame output height",OFFSETBASE+OFFSETP(ulOutputHeight),AV_OPT_TYPE_INT, { .i64 = 0  }, 0, 4096, VE },
    { "resize_quality","video encoded frame resize quality",OFFSETBASE+OFFSETP(szResizeQuality),AV_OPT_TYPE_STRING, { .str = "high"  }, 0, 0, VE  },
    { "passlogfile","filename for 2 pass encode stats",OFFSETBASE+OFFSETP(szPasslogFile),AV_OPT_TYPE_STRING, { .str = ""  }, 0, 0, VE  },
    { "avgrate","average bitrate for encoder",OFFSETBASE+OFFSETP(ulAvgBitrate),AV_OPT_TYPE_INT, { .i64 = 0  }, 0, INT_MAX, VE },
    { "maxrate","max bitrate for encoder",OFFSETBASE+OFFSETP(ulMaxBitrate),AV_OPT_TYPE_INT, { .i64 = 0  }, 0, INT_MAX, VE },
    { "pon","picture order number for encoder",OFFSETBASE+OFFSETP(ulPon),AV_OPT_TYPE_INT, { .i64 = 0  }, 0, INT_MAX, VE },
    { "cpulow","cpu low mode on/off",OFFSETBASE+OFFSETP(ulCPULow),AV_OPT_TYPE_INT, { .i64 = 0  }, 0, INT_MAX, VE },
    { "stats", "filename for 2 pass stats", OFFSETBASE+OFFSETP(szPasslogFile), AV_OPT_TYPE_STRING, { 0 },       0, 0, VE},
    { "vbrbitrate_opt","vbr bitrate priority on/off",OFFSETBASE+OFFSETP(ulVbrbitrateOpt),AV_OPT_TYPE_INT, { .i64 = 0  }, 0,   2, VE  },
    { NULL  },
};

static const AVClass rmhdenc_class =
{
    .class_name = "librmhdenc",
    .item_name  = av_default_item_name,
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

AVCodec ff_librv11_encoder =
{
    .name                  = "librv11",
    .long_name             = NULL_IF_CONFIG_SMALL("librv11 RealVideo 11 RV60"),
    .type                  = AVMEDIA_TYPE_VIDEO,
    .id                    = AV_CODEC_ID_RV60,
    .priv_data_size        = sizeof(LIBRMHDEncContext),
    .init                  = librmhdenc_init,
    .encode2               = librmhdenc_encode,
    .close                 = librmhdenc_close,
    .capabilities          = AV_CODEC_CAP_DELAY|AV_CODEC_CAP_AUTO_THREADS,
    .pix_fmts              = (const enum AVPixelFormat[]) {
        AV_PIX_FMT_YUV420P,
        AV_PIX_FMT_NONE
    },
    .priv_class            = &rmhdenc_class,
    .wrapper_name     = "librv11",
};
