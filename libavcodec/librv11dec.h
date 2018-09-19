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

#ifndef AVCODEC_LIBRV11DEC_H
#define AVCODEC_LIBRV11DEC_H

typedef unsigned char           UINT8;   /* unsigned 8 bit value */
typedef unsigned short int      UINT16;  /* unsigned 16 bit value */
typedef int                     INT32;   /* signed 32 bit value */
typedef unsigned int            UINT32;  /* unsigned 32 bit value */
typedef int                     LONG32;  /* signed 32 bit value */
typedef UINT32                  ULONG32; /* unsigned 32 bit value */
typedef UINT8                   UCHAR;   /* unsigned 8 bit value */

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

typedef LONG32  HX_RESULT;

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


#define HXR_FAIL                        MAKE_HRESULT(1,0,0x4005)                    // 80004005
#define HXR_OK                          MAKE_HRESULT(0,0,0)                         // 00000000


typedef int (RV60toYUV420Transform)(UCHAR *pRV10Packets,
            UCHAR *pDecodedFrameBuffer, void *pInputParams,
            void *pOutputParams,void *global);
typedef int (RV60toYUV420Init)(void *prv10Init,void **global);
typedef int (RV60toYUV420Free)(void *global);
typedef int (RV60toYUV420CustomMessage)(void *msg_id, void *global);
     // The RV60toYUV420CustomMessage function exposes decoder interfaces that
     // are specific to the "ILVC" decoder.  These interfaces are defined
     // in "ilvcmsg.h".

typedef int (RV60toYUV420HiveMessage)(ULONG32 *msg, void *global);
    // The RV40toYUV420HiveMessage function exposes decoder interfaces that
    // may be applicable to a variety of decoders, not just to "ILVC".
    // The 'msg' parameter points to a ULONG32 that identifies a particular
    // interface or feature.  This ULONG32 is actually the first member in
    // a larger struct, similar to the PIA_Custom_Message_ID usage.
    // See "hivervi.h" for a complete list of supported messages.


/*
 * RV frame struct.
 */
typedef struct rv_segment_struct
{
    HXBOOL bIsValid;
    UINT32 ulOffset;
} rv_segment;


/* */
/* Define formats for representing custom codec messages. */
/* Custom codec messages have semantics unknown to the HIVE/RV layer. */
/* The messages are defined by the underlying codec, and typically documented */
/* for use by applications. */
/* */

/* RV_Custom_Message_ID identifies a specific custom message.  These */
/* identifiers are not guaranteed to be unique among different codecs. */
/* Thus an application using such messages, must have knowledge regarding */
/* which codec is being used, and what custom messages it supports. */
/* A codec will typically distribute a header file that defines its */
/* custom messages. */
/* */
/* Specific custom messages will be passed into a codec via a structure whose */
/* first member is a RV_Custom_Message_ID, and additional parameters */
/* understood by the application and the codec.  The size of each such */
/* custom message structure is variable, depending on the specific message id. */
/* Messages that require no values other than a message id, can be indicated */
/* by using the RV_Custom_Message_ID as is.  For such messages, there is no */
/* need to wrap the message id within another structure. */

typedef UINT32     RV_Custom_Message_ID;

/* RV_MSG_Simple is a structure used to pass custom messages which */
/* take one or two 32-bit values.  The "message_id" member identifies a */
/* particular message, and the "value1" and "value2" members specify the */
/* 32-bit values.  The interpretation of the values depends on the message id. */
/* They could be simple booleans (zero or non-zero), or arbitrary integers. */
/* Some messages will use only value1, and not value2. */

typedef struct {

    RV_Custom_Message_ID       message_id;

    INT32                      value1;
    INT32                      value2;

} RV_MSG_Simple;


/* The RV_MSG_DISABLE, RV_MSG_ENABLE and RV_MSG_GET values may be used as */
/* controls when manipulating boolean-valued custom messages.  Typically, */
/* such messages will employ the RV_MSG_Simple structure.  The message_id */
/* will identify a boolean-valued option being manipulated.  The "value1" */
/* member will be set to one of RV_MSG_DISABLE, RV_MSG_ENABLE or RV_MSG_GET. */
/* Depending on the message id, "value2" may specify additional information */
/* such as a layer number for scalable video sequences. */
/* */
/* When value1 is RV_MSG_DISABLE or RV_MSG_ENABLE, the specified option */
/* will be disabled or enabled, respectively. */
/* When value1 is RV_MSG_GET, the current setting for the option (zero for */
/* disabled and non-zero for enabled) will be returned in "value2". */

#define RV_MSG_DISABLE   0
#define RV_MSG_ENABLE    1
#define RV_MSG_GET       2


/* Decoder postfilters: */
/*  - the smoothing postfilter is designed to remove general compression */
/*    artifacts, mosquito noise, and ringing noise. */
/*  - the annex J deblocking filter when used as a postfilter (annex J */
/*    not encoded in bitstream), removes blocking artifacts (block */
/*    edges) almost as efficiently as when used in-the-loop, i.e. encoded */
/*    in bitstream. */
/*  For the most video quality improvement, both filters should be used. */
/*  However, each filter takes up a certain amount of CPU cycles, and */
/*  for large formats, it may be too computationally expensive to use */
/*  both. */
/*  These filters are more effective, the lower the bitrate. */

/* */
/* Define a custom decoder message for enabling the smoothing postfilter. */
/* */
/* This message must be sent with the RV_MSG_Simple structure. */
/* Its value1 member should be RV_MSG_DISABLE, RV_MSG_ENABLE or RV_MSG_GET. */
/* Its value2 is only used for RV_MSG_GET, in which case it is used to */
/* return the current setting. */
/* */
#define RV_MSG_ID_Smoothing_Postfilter 17


typedef struct rv_backend_init_params_struct
{
    UINT16 usOuttype;
    UINT16 usPels;       /* display width */
    UINT16 usLines;      /* display height */
    UINT16 usPadWidth;   /* number of columns of padding on right to get 16 x 16 block*/
    UINT16 usPadHeight;  /* number of rows of padding on bottom to get 16 x 16 block*/

    UINT16 pad_to_32;   /* to keep struct member alignment independent of */
                        /* compiler options */
    UINT32 ulInvariants;
        /* ulInvariants specifies the invariant picture header bits */
    INT32  bPacketization;
    UINT32 ulStreamVersion;
} rv_backend_init_params;

typedef struct rv_backend_in_params_struct
{
    UINT32     dataLength;
    INT32      bInterpolateImage;
    UINT32     numDataSegments;
    rv_segment *pDataSegments;
    UINT32     flags;
        /* 'flags' should be initialized by the front-end before each */
        /* invocation to decompress a frame.  It is not updated by the decoder. */
        /* */
        /* If it contains RV_DECODE_MORE_FRAMES, it informs the decoder */
        /* that it is being called to extract the second or subsequent */
        /* frame that the decoder is emitting for a given input frame. */
        /* The front-end should set this only in response to seeing */
        /* an RV_DECODE_MORE_FRAMES indication in H263DecoderOutParams. */
        /* */
        /* If it contains RV_DECODE_DONT_DRAW, it informs the decoder */
        /* that it should decode the image (in order to produce a valid */
        /* reference frame for subsequent decoding), but that no image */
        /* should be returned.  This provides a "hurry-up" mechanism. */
    UINT32     timestamp;
} rv_backend_in_params;

typedef struct rv_backend_out_params_struct
{
    UINT32 numFrames;
    UINT32 notes;
        /* 'notes' is assigned by the transform function during each call to */
        /* decompress a frame.  If upon return the notes parameter contains */
        /* the indication RV_DECODE_MORE_FRAMES, then the front-end */
        /* should invoke the decoder again to decompress the same image. */
        /* For this additional invocation, the front-end should first set */
        /* the RV_DECODE_MORE_FRAMES bit in the 'H263DecoderInParams.flags' */
        /* member, to indicate to the decoder that it is being invoked to */
        /* extract the next frame. */
        /* The front-end should continue invoking the decoder until the */
        /* RV_DECODE_MORE_FRAMES bit is not set in the 'notes' member. */
        /* For each invocation to decompress a frame in the same "MORE_FRAMES" */
        /* loop, the front-end should send in the same input image. */
        /* */
        /* If the decoder has no frames to return for display, 'numFrames' will */
        /* be set to zero.  To avoid redundancy, the decoder does *not* set */
        /* the RV_DECODE_DONT_DRAW bit in 'notes' in this case. */


    UINT32 timestamp;
        /* The 'temporal_offset' parameter is used in conjunction with the */
        /* RV_DECODE_MORE_FRAMES note, to assist the front-end in */
        /* determining when to display each returned frame. */
        /* If the decoder sets this to T upon return, the front-end should */
        /* attempt to display the returned image T milliseconds relative to */
        /* the front-end's idea of the presentation time corresponding to */
        /* the input image. */
        /* Be aware that this is a signed value, and will typically be */
        /* negative. */

    UINT32 width;
    UINT32 height;
        /* Width and height of the returned frame. */
        /* This is the width and the height as signalled in the bitstream. */

} rv_backend_out_params;


/* definitions for output parameter notes */

#define RV_DECODE_MORE_FRAMES           0x00000001
#define RV_DECODE_DONT_DRAW             0x00000002
#define RV_DECODE_KEY_FRAME             0x00000004
    /* Indicates that the decompressed image is a key frame. */
    /* Note that enhancement layer EI frames are not key frames, in the */
    /* traditional sense, because they have dependencies on lower layer */
    /* frames. */

#define RV_DECODE_B_FRAME               0x00000008
    /* Indicates that the decompressed image is a B frame. */
    /* At most one of PIA_DDN_KEY_FRAME and PIA_DDN_B_FRAME will be set. */


#define RV_DECODE_FRU_FRAME             0x00000020
    /* Indicates that the decompressed image is a B frame. */
    /* At most one of PIA_DDN_KEY_FRAME and PIA_DDN_B_FRAME will be set. */


#define RV_DECODE_LAST_FRAME            0x00000200
    /* Indicates that the accompanying input frame is the last in the */
    /* current sequence. If input frame is a dummy frame, the decoder */
    /* flushes the latency frame to the output. */


#endif /* AVCODEC_LIBRV11DEC_H */
