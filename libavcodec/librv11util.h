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

#ifndef AVCODEC_LIBRV11UTIL_H
#define AVCODEC_LIBRV11UTIL_H

#ifndef MAX_PACKET_SIZE
#define MAX_PACKET_SIZE 15000
#endif

static inline int  get_segments_number(uint32_t data_length )
{
    //to rv11 numSegments should be 1
    int num_packet = 0;
    if(data_length<1){
        return -1;
    }

    num_packet = data_length/MAX_PACKET_SIZE;
    if(data_length%MAX_PACKET_SIZE!=0){
        num_packet++;
    }
    return num_packet;
}

static inline int  write_rv_frame_head(uint8_t *data, int num_packet )
{
    int ret = -1;
    int i = 0;
    if (num_packet <1 || data ==NULL){
        return ret;
    }
    data[0] = num_packet -1;
    for(i = 0; i< num_packet; i++){
        //the value of next 4 byte is the segment index
        AV_WL32(data+1+8*i, (i+1));
        //continuous 4 byte is the segment offset
        AV_WL32(data+5+8*i, (MAX_PACKET_SIZE * i));
    }
    return 0;
}

#endif /* AVCODEC_LIBRV11UTIL_H */
