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


#ifndef AVCODEC_LIBRV11ENC_H
#define AVCODEC_LIBRV11ENC_H

#if defined( _WIN32 ) || defined( _WINDOWS )
#ifdef __GNUC__
#pragma pack(1)
#else
#pragma pack(1)
//  disable warning on: zero-sized array in struct/union
#pragma warning( disable : 4200 )
#endif
#endif

typedef unsigned char           UINT8;    /* unsigned 8 bit value */
typedef unsigned short int      UINT16;   /* unsigned 16 bit value */
typedef int                     INT32;    /* signed 32 bit value */
typedef unsigned int            UINT32;   /* unsigned 32 bit value */
typedef int                     LONG32;   /* signed 32 bit value */
typedef UINT32                  ULONG32;  /* unsigned 32 bit value */
typedef ULONG32                 UFIXED32; /* FIXED point value  */
typedef UINT8                   UCHAR;    /* unsigned 8 bit value */

#if defined(_WIN32)
typedef __int64			INT64;
typedef unsigned __int64	UINT64;
#else
typedef long long		INT64;
typedef unsigned long long	UINT64;
#endif

typedef ULONG32                 HX_MOFTAG;

#if defined(BOOL)
    typedef BOOL                HXBOOL;
#else
    typedef int                 HXBOOL;
    typedef HXBOOL              BOOL;
#endif

#ifdef TRUE
#undef TRUE
#endif

#ifdef FALSE
#undef FALSE
#endif

#ifndef TRUE
#define TRUE                1
#endif

#ifndef FALSE
#define FALSE               0
#endif

#ifndef NULL
#define NULL                ((void *)0)
#endif

typedef LONG32              HX_RESULT;

typedef void * IHXUnknown;
typedef IHXUnknown* PIHXUnknown;

typedef void *HXCODEC;
typedef void *HXSTREAM;
typedef void *HXMEMORY;
typedef void *HXPACKETBUFF;
typedef INT64   HXT_TIME;

#ifndef _WIN32
    typedef HX_RESULT HRESULT;
#   define MAKE_HRESULT(sev,fac,code)                                           \
        ((HRESULT) (((unsigned long)(sev)<<31) | ((unsigned long)(fac)<<16) |   \
        ((unsigned long)(code))) )
#   define SUCCEEDED(Status) (((unsigned long)(Status)>>31) == 0)
#   define FAILED(Status) (((unsigned long)(Status)>>31) != 0)
#else
#   include <winerror.h>
#endif /* _WIN32 */

#define HXR_FAIL                        MAKE_HRESULT(1,0,0x4005)        // 80004005
#define HXR_OK                          MAKE_HRESULT(0,0,0)             // 00000000

#define HX_MEDIA_AUDIO                 0x4155444FL                     // 'AUDO'
#define HX_MEDIA_VIDEO                 0x5649444FL                     // 'VIDO'

#define HX_YUV420_ID                   0x59555632L                     // 'YUV2'

// Stream property flags
#define SP_INTERPOLATE                  0x00010000
#define SP_SPEED_QUALITY                0x00010001                      // ULONG32: 1=max speed, 100=max quality
#define SP_LIVE                         0x00010002                      // BOOL: TRUE=live source, FALSE=static source
#define SP_BITRATE                      0x00010003                      // ULONG32: bitrate to encode at
#define SP_VARIABLE_FRAME_RATE          0x00010004                      // BOOL: TRUE allow frame rate to vary, FALSE

#define SP_QUALITY_MOTION               0x00010005
#define SP_MAX_FRAMERATE                0x00010010

#define SP_CODING_MODE                  0x00010012

#define SP_CODEC_SETUP_DONE             0x0001001f                      // pass TRUE for this prop when setup of stream properties is done
#define SP_ECC                          0x00010021                      // Sets the percentage of the bitstream dedicated to error correction
#define SP_ANALYSIS_FILENAME            0x00010023
#define SP_MAXBITRATE                   0x00010024
#define SP_KEYFRAMERATE                 0x00010025
#define SP_TARGET_LATENCY               0x00010028
#define SP_RESIZING_ACCURACY            0x0001002c
#define SP_QUALITY                      0x00010031
#define SP_CHECK_FRAME_SKIP             0x00010047
#define SP_SET_FAST_LEVEL               0x00010048
#define SP_SET_PON_START                0x00010049
#define SP_VBVBUFFERSIZE                0x00010050
#define SP_RATECONTROL_PLUS             0x00010052


#define SP_CODEC_ONEPASS_DELAY          0x00020001
#define SP_CODEC_CPU_LOW                0x00020002

/*  SPCODINGMODEs are the allowed values when setting the SP_CODING_MODE param in  */
/*  PNStream_SetProperty  */
#define SPCODINGMODE_VIDEO              0x00000000
#define SPCODINGMODE_POWERPOINT         0x00000001
#define SPCODINGMODE_HIGHDURESS         0x00000002
#define SPCODINGMODE_SLIDESHOW          0x00000003
#define SPCODINGMODE_ANALYSIS           0x00000004
#define SPCODINGMODE_FROMFILE           0x00000005
#define SPCODINGMODE_ENCODE             0x00000006

#define CP_ENC_SUGGEST_NUMBER_OF_THREAD 0x00000000
#define CP_ENC_ENABLE_GPU_ACCELEARATION 0x00000001
#define CP_ENC_DISABLE_GPU_ACCELEARATION 0x00000002

// HX_MOF base Media object format struct
typedef struct tag_HX_MOF
{
    ULONG32            cbLength;                        // size of structure in bytes
    HX_MOFTAG          moftag;                          // identifier of media format
    HX_MOFTAG          submoftag;                       // identifier of object format
} HX_MOF;

// Generic Video HX_MOF struct
typedef struct tag_HX_FORMAT_VIDEO
{
    ULONG32            cbLength;                        // the size of this struct in bytes
    HX_MOFTAG         moftag;                          // always == HX_MEDIA_VIDEO
    HX_MOFTAG          submoftag;                       // video format identifier

    // General attributes of the video stream independent of bitwise encoding
    UINT16             uiWidth;                         // width of the image in pixels
    UINT16             uiHeight;                        // height of the image in pixels
    UINT16             uiBitCount;                      // color depth in bits
    UINT16             uiPadWidth;                      // number of padded columns for codecs that
                                                        // need certian block sizes e.g. 8x8
    UINT16             uiPadHeight;                     // number of padded rows for codecs that
                                                        // need certian block sizes e.g. 8x8
    UFIXED32           framesPerSecond;                 // frames per second
}HX_FORMAT_VIDEO;

// HXCODEC_INIT struct used in HXStream_Open()
typedef struct tag_HXCODEC_INIT
{
    HX_MOF             *pInMof;
    HX_MOF             *pOutMof;
    HXMEMORY           memoryRef;
}HXCODEC_INIT;

// HXCODEC_SETMENTINFO
#ifndef TAG_HXCODEC_SEGMENTINFO
#define TAG_HXCODEC_SEGMENTINFO
typedef struct tag_HXCODEC_SEGMENTINFO
{
    LONG32 bIsValid;
    ULONG32 ulSegmentOffset;
} HXCODEC_SEGMENTINFO;
#define HXCODEC_SEGMENTINFO_SIZE        (8)
#endif

enum EncodeType
{
    ENCODE_CBR,
    ENCODE_VBR_BITRATE,
    ENCODE_VBR_QUALITY,
    ENCODE_VBR_UNCONSTRAINED_BITRATE,
    ENCODE_VBR_UNCONSTRAINED_QUALITY
};

enum EncodingMode
{
    ENCODE_SINGLE_PASS,
    ENCODE_CREATE_ANALYSIS,
    ENCODE_USING_ANALYSIS
};

// HXCODEC_DATA struct used in PNStream_Input() and data_callback functions.
typedef struct tag_HXCODEC_DATA
{
    ULONG32             dataLength;
    UCHAR               *data;
    ULONG32             timestamp;
    UINT16              sequenceNum;
    UINT16              flags;
    BOOL                lastPacket;
    ULONG32             numSegments;
    HXCODEC_SEGMENTINFO Segments[1];
} HXCODEC_DATA;

typedef struct tag_RVENCODE_PARAM
{
    BOOL              isLossProtect;
    BOOL              isGPUAccelerate;
    BOOL              isEnhanceCoding;
    char*             szEncodeType;
    char*             szEncComplevel;
    char*             szResizeQuality;
    char*             szPasslogFile;
    enum EncodeType   eEncodeType;
    enum EncodingMode eEncodingMode;
    UINT32            ulEncComplevel;
    UINT32            ulResizeQuality;
    UINT32            ulVbrQuality;
    UINT32            ulMaxFramerate;
    UINT32            ulMaxKeyFrameInterval;
    UINT32            ulMaxPacketSize;
    UINT32            ulInputWidth;
    UINT32            ulInputHeight;
    UINT32            ulOutputWidth;
    UINT32            ulOutputHeight;
    UINT32            ulAvgBitrate;
    UINT32            ulMaxBitrate;
    UINT32            ulMaxThreads;
    UINT32            ulPon;
    float             lfMaxStartLatency;
    UINT32            ulCPULow;
    UINT32            ulVbrbitrateOpt;
}RVENCODE_PARAM;

#define HX_KEYFRAME_FLAG        0x0002
#define kMaxUint32              0xffffffff

#if defined( _WIN32 ) || defined( _WINDOWS )
#ifdef __GNUC__
#pragma pack()
#else
#pragma pack()
//  Restore warnings
#pragma warning( default : 4200 )
#endif
#endif

#endif /* AVCODEC_LIBRV11ENC_H */
