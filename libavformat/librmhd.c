/*
 * utils for rmhd format
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

#include "librmhd.h"

const char * const ff_librmhd_metadata[4] = {
    "title",
    "author",
    "copyright",
    "comment"
};

const AVCodecTag ff_librmhd_codec_tags[] = {
    { AV_CODEC_ID_RV10,   MKTAG('R','V','1','0') },
    { AV_CODEC_ID_RV20,   MKTAG('R','V','2','0') },
    { AV_CODEC_ID_RV20,   MKTAG('R','V','T','R') },
    { AV_CODEC_ID_RV30,   MKTAG('R','V','3','0') },
    { AV_CODEC_ID_RV40,   MKTAG('R','V','4','0') },
    { AV_CODEC_ID_RV60,   MKTAG('R','V','6','0') },
    { AV_CODEC_ID_AC3,    MKTAG('d','n','e','t') },
    { AV_CODEC_ID_RA_144, MKTAG('l','p','c','J') },
    { AV_CODEC_ID_RA_288, MKTAG('2','8','_','8') },
    { AV_CODEC_ID_COOK,   MKTAG('c','o','o','k') },
    { AV_CODEC_ID_ATRAC3, MKTAG('a','t','r','c') },
    { AV_CODEC_ID_SIPR,   MKTAG('s','i','p','r') },
    { AV_CODEC_ID_AAC,    MKTAG('r','a','a','c') },
    { AV_CODEC_ID_AAC,    MKTAG('r','a','c','p') },
    { AV_CODEC_ID_RALF,   MKTAG('L','S','D',':') },
    { AV_CODEC_ID_NONE },
};
