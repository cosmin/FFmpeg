/*
 * This copyright notice applies to this file only
 * This Software is distributed under MIT License
 *
 * API software for using RealVideo 11 (RV60) Codec
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

#ifndef AVCODEC_LIBRV11_H
#define AVCODEC_LIBRV11_H

#include <librv11/librv11_sdk.h>

#include "avcodec.h"
#include "libavutil/intreadwrite.h"

#define RV_MAX_INPUT_FRAME_RATE 120
#define RV_NUM_OUT_FRAMES         6

#define XSTR(s) STR(s)
#define STR(s) #s
#define LIBRV11DEC_FILE XSTR(RV_DEC_LIB_FILE)
#define LIBRV11ENC_FILE XSTR(RV_ENC_LIB_FILE)


#endif // AVCODEC_LIBRV11_H
