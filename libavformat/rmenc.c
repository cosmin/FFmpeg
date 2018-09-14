/*
 * "Real" compatible muxer.
 * Copyright (c) 2000, 2001 Fabrice Bellard
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
#include "avformat.h"
#include "avio_internal.h"
#include "rm.h"
#include "libavutil/dict.h"
#include "libavutil/intreadwrite.h"

typedef struct StreamInfo {
    int nb_packets;
    int packet_total_size;
    int packet_max_size;
    /* codec related output */
    int bit_rate;
    AVRational frame_rate;
    int nb_frames;    /* current frame number */
    int total_frames; /* total number of frames */
    int num;
    int32_t deint_id;
    AVCodecParameters *par;
} StreamInfo;

#define MODE_RMVB 0x01
#define MODE_RMHD 0x02

typedef struct RMMuxContext {
    StreamInfo streams[2];
    StreamInfo *audio_stream, *video_stream;
    int data_pos; /* position of the data after the header */
    int mode;
} RMMuxContext;

/* in ms */
#define BUFFER_DURATION 0
/* the header needs at most 7 + 4 + 12 B */
#define MAX_HEADER_SIZE (7 + 4 + 12)
/* UINT16_MAX is the maximal chunk size */
#define MAX_PACKET_SIZE (UINT16_MAX - MAX_HEADER_SIZE)


static void put_str(AVIOContext *s, const char *tag)
{
    avio_wb16(s,strlen(tag));
    while (*tag) {
        avio_w8(s, *tag++);
    }
}

static void put_str8(AVIOContext *s, const char *tag)
{
    avio_w8(s, strlen(tag));
    while (*tag) {
        avio_w8(s, *tag++);
    }
}

static int rv10_write_header(AVFormatContext *ctx,
                             int64_t data_size, int64_t index_pos)
{
    RMMuxContext *rm = ctx->priv_data;
    AVIOContext *s = ctx->pb;
    StreamInfo *stream;
    const char *desc, *mimetype;
    int nb_packets, packet_total_size, packet_max_size, size, packet_avg_size, i, j;
    int bit_rate, v, duration, flags;
    int data_offset;
    AVDictionaryEntry *tag;

    // RMF represent rmvb, RMP represent rmhd
    if(rm->mode & MODE_RMVB) {
        ffio_wfourcc(s,".RMF");
        avio_wb32(s,18); /* header size */
        avio_wb16(s,0);  /* object version */
        avio_wb32(s,0);  /* file version */
        avio_wb32(s,4 + ctx->nb_streams); /* num headers */
        ffio_wfourcc(s,"PROP");
        avio_wb32(s, 50);
        avio_wb16(s, 0);
    } else {
        ffio_wfourcc(s, ".RMP");
        avio_wb32(s,18); /* header size */
        avio_wb16(s,1);  /* object version */
        avio_wb32(s,0);  /* file version */
        avio_wb32(s,5 + ctx->nb_streams); /* num headers */
        ffio_wfourcc(s,"PROP");
        avio_wb32(s, 54);
        avio_wb16(s, 2);
    }

    packet_max_size = 0;
    packet_total_size = 0;
    nb_packets = 0;
    bit_rate = 0;
    duration = 0;
    for(i=0;i<ctx->nb_streams;i++) {
        StreamInfo *stream = &rm->streams[i];
        bit_rate += stream->bit_rate;
        if (stream->packet_max_size > packet_max_size)
            packet_max_size = stream->packet_max_size;
        nb_packets += stream->nb_packets;
        packet_total_size += stream->packet_total_size;
        /* select maximum duration */
        v = av_rescale_q_rnd(stream->total_frames, (AVRational){1000, 1}, stream->frame_rate, AV_ROUND_ZERO);
        if (v > duration)
            duration = v;
    }
    avio_wb32(s, bit_rate); /* max bit rate */
    avio_wb32(s, bit_rate); /* avg bit rate */
    avio_wb32(s, packet_max_size);        /* max packet size */
    if (nb_packets > 0)
        packet_avg_size = packet_total_size / nb_packets;
    else
        packet_avg_size = 0;
    avio_wb32(s, packet_avg_size);        /* avg packet size */
    avio_wb32(s, nb_packets);  /* num packets */
    avio_wb32(s, duration); /* duration */
    avio_wb32(s, BUFFER_DURATION);           /* preroll */
    if(rm->mode & MODE_RMVB) {
        avio_wb32(s, index_pos);             /* index offset */
    } else {
        avio_wb64(s, index_pos);             /* index offset */
    }
    /* computation of data the data offset */
    data_offset = avio_tell(s);
    avio_wb32(s, 0);           /* data offset : will be patched after */
    avio_wb16(s, ctx->nb_streams);    /* num streams */
    flags = 1 | 2; /* save allowed & perfect play */
    if (!(s->seekable & AVIO_SEEKABLE_NORMAL))
        flags |= 4; /* live broadcast */
    avio_wb16(s, flags);

    /* comments */

    ffio_wfourcc(s,"CONT");
    size =  4 * 2 + 10;
    for(i=0; i<FF_ARRAY_ELEMS(ff_rm_metadata); i++) {
        tag = av_dict_get(ctx->metadata, ff_rm_metadata[i], NULL, 0);
        if(tag) size += strlen(tag->value);
    }
    avio_wb32(s,size);
    avio_wb16(s,0);
    for(i=0; i<FF_ARRAY_ELEMS(ff_rm_metadata); i++) {
        tag = av_dict_get(ctx->metadata, ff_rm_metadata[i], NULL, 0);
        put_str(s, tag ? tag->value : "");
    }

    for(i=0;i<ctx->nb_streams;i++) {
        int codec_data_size;

        stream = &rm->streams[i];

        if (stream->par->codec_type == AVMEDIA_TYPE_AUDIO) {
            desc = "The Audio Stream";
            mimetype = "audio/x-pn-realaudio";
            if (stream->par->codec_id == AV_CODEC_ID_AAC) {
                codec_data_size = 78;
                if(stream->par->extradata_size > 0){
                    codec_data_size += stream->par->extradata_size + 1;
                }
            } else {
                codec_data_size = 73;
            }
        } else {
            desc = "The Video Stream";
            mimetype = "video/x-pn-realvideo";
            if(stream->par->codec_id == AV_CODEC_ID_RV10 ||
               stream->par->codec_id == AV_CODEC_ID_RV20 ||
               stream->par->codec_id == AV_CODEC_ID_RV40) {
                codec_data_size = 34;
            } else if(!stream->par->extradata_size &&
                      stream->par->codec_id == AV_CODEC_ID_RV60) {
                codec_data_size = 40;
            } else {
                codec_data_size = 26 + stream->par->extradata_size;
            }
        }

        ffio_wfourcc(s,"MDPR");
        size = 10 + 9 * 4 + strlen(desc) + strlen(mimetype) + codec_data_size;
        avio_wb32(s, size);
        avio_wb16(s, 0);

        avio_wb16(s, i); /* stream number */
        avio_wb32(s, stream->bit_rate); /* max bit rate */
        avio_wb32(s, stream->bit_rate); /* avg bit rate */
        avio_wb32(s, stream->packet_max_size);        /* max packet size */
        if (stream->nb_packets > 0)
            packet_avg_size = stream->packet_total_size /
                stream->nb_packets;
        else
            packet_avg_size = 0;
        avio_wb32(s, packet_avg_size);        /* avg packet size */
        avio_wb32(s, 0);           /* start time */
        avio_wb32(s, BUFFER_DURATION);           /* preroll */
        /* duration */
        if (!(s->seekable & AVIO_SEEKABLE_NORMAL) || !stream->total_frames)
            avio_wb32(s, (int)(3600 * 1000));
        else
            avio_wb32(s, av_rescale_q_rnd(stream->total_frames, (AVRational){1000, 1},  stream->frame_rate, AV_ROUND_ZERO));
        put_str8(s, desc);
        put_str8(s, mimetype);
        avio_wb32(s, codec_data_size);

        if (stream->par->codec_type == AVMEDIA_TYPE_AUDIO) {
            int coded_frame_size, fscode, sample_rate;
            int frame_size = av_get_audio_frame_duration2(stream->par, 0);
            int version = stream->par->codec_id == AV_CODEC_ID_AAC ? 5 : 4;
            sample_rate = stream->par->sample_rate;
            coded_frame_size = (stream->par->bit_rate *
                                frame_size) / (8 * sample_rate);
            /* audio codec info */
            avio_write(s, ".ra", 3);
            avio_w8(s, 0xfd);
            if (version == 5) {
                avio_wb32(s, 0x00050000); /* version */
            } else {
                avio_wb32(s, 0x00040000); /* version */
            }
            ffio_wfourcc(s, ".ra4");
            avio_wb32(s, 0x01b53530); /* stream length */
            if (version == 5) { /* write version */
                avio_wb16(s, 5);
            } else {
                avio_wb16(s, 4);
            }
            avio_wb32(s, 0x39); /* header size */

            switch(sample_rate) {
            case 48000:
            case 24000:
            case 12000:
                fscode = 1;
                break;
            default:
            case 44100:
            case 22050:
            case 11025:
                fscode = 2;
                break;
            case 32000:
            case 16000:
            case 8000:
                fscode = 3;
            }
            avio_wb16(s, fscode); /* codec additional info, for AC-3, seems
                                     to be a frequency code */
            /* special hack to compensate rounding errors... */
            if (coded_frame_size == 557)
                coded_frame_size--;
            avio_wb32(s, coded_frame_size); /* frame length */
            avio_wb32(s, 0x51540); /* unknown */
            avio_wb32(s, stream->par->bit_rate / 8 * 60); /* bytes per minute */
            avio_wb32(s, stream->par->bit_rate / 8 * 60); /* bytes per minute */
            avio_wb16(s, 0x01);
            /* frame length : seems to be very important */
            avio_wb16(s, coded_frame_size);
            if (version == 5) {
                avio_wb16(s, coded_frame_size);
                avio_wb32(s, 0);
                avio_wb32(s, stream->par->sample_rate<<16); /* sample rate */
                avio_wb32(s, stream->par->sample_rate<<16); /* sample rate */
                avio_wb16(s, 0x10);
                avio_wb16(s, stream->par->channels);
                avio_wb32(s, 0x76627266);
                stream->deint_id = DEINT_ID_VBRF;
                if (stream->par->codec_tag) {
                    avio_wl32(s, stream->par->codec_tag);
                } else {
                    av_log(ctx, AV_LOG_ERROR, "Invalid codec tag\n");
                    return AVERROR(EINVAL);
                }
                avio_wb16(s, 0x0100); /* title length set interleaved flag to 1*/
                avio_wb16(s, 0); /* author length */
                if (stream->par->extradata_size > 0) {
                    avio_wb32(s, stream->par->extradata_size + 1); /* tag length */
                    avio_w8(s, 2); /* tag length */
                    for(j = 0; j< stream->par->extradata_size; j++) {
                        avio_w8(s, stream->par->extradata[j]); /* end of header */
                    }
                } else {
                    avio_wb32(s, 0);
                }
            } else {
                avio_wb32(s, 0);
                avio_wb16(s, stream->par->sample_rate); /* sample rate */
                avio_wb32(s, 0x10);
                avio_wb16(s, stream->par->channels);
                put_str8(s, "Int0"); /* codec name */
                if (stream->par->codec_tag) {
                    avio_w8(s, 4); /* tag length */
                    avio_wl32(s, stream->par->codec_tag);
                } else {
                    av_log(ctx, AV_LOG_ERROR, "Invalid codec tag\n");
                    return -1;
                }
                avio_wb16(s, 0); /* title length */
                avio_wb16(s, 0); /* author length */
                avio_wb16(s, 0); /* copyright length */
                avio_w8(s, 0);   /* end of header */
            }
        } else {
            /* video codec info */
            avio_wb32(s,34); /* size */
            ffio_wfourcc(s, "VIDO");
            switch (stream->par->codec_id) {
            case AV_CODEC_ID_RV10:
                ffio_wfourcc(s,"RV10");
                break;
            case AV_CODEC_ID_RV20:
                ffio_wfourcc(s,"RV20");
                break;
            case AV_CODEC_ID_RV30:
                ffio_wfourcc(s,"RV30");
                break;
            case AV_CODEC_ID_RV40:
                ffio_wfourcc(s,"RV40");
                break;
            case AV_CODEC_ID_RV60:
                ffio_wfourcc(s,"RV60");
                break;
            }

            avio_wb16(s, stream->par->width);
            avio_wb16(s, stream->par->height);

            if (stream->frame_rate.num / stream->frame_rate.den > 65535) {
                av_log(s, AV_LOG_ERROR, "Frame rate %d is too high\n", stream->frame_rate.num / stream->frame_rate.den);
                return AVERROR(EINVAL);
            }

            avio_wb16(s, stream->frame_rate.num / stream->frame_rate.den); /* frames per seconds ? */
            avio_wb32(s,0);     /* unknown meaning */

            if(rm->mode & MODE_RMVB) {
                avio_wb16(s, stream->frame_rate.num / stream->frame_rate.den);  /* unknown meaning */
                avio_wb32(s,0);     /* unknown meaning */
                avio_wb16(s, 8);    /* unknown meaning */
                /* Seems to be the codec version: only use basic H.263. The next
                   versions seems to add a differential DC coding as in
                   MPEG... nothing new under the sun. */
                if(stream->par->codec_id == AV_CODEC_ID_RV10)
                    avio_wb32(s,0x10000000);
                else
                    avio_wb32(s,0x20103001);
            } else {
                avio_wb32(s, ((int)av_q2d(stream->frame_rate)*(1<<16)));  /* unknown meaning */
                if (stream->par->codec_id == AV_CODEC_ID_RV10 ||
                    stream->par->codec_id == AV_CODEC_ID_RV20 ||
                    stream->par->codec_id == AV_CODEC_ID_RV40) {
                    avio_wb32(s, 8);    /* unknown meaning */
                    if (stream->par->codec_id == AV_CODEC_ID_RV10) {
                        avio_wb32(s,0x10000000);
                    } else if (stream->par->codec_id == AV_CODEC_ID_RV20) {
                        avio_wb32(s,0x20103001);
                    } else { // AV_CODEC_ID_RV40
                        avio_wb32(s,0x40008000);
                    }
                } else if (stream->par->extradata_size >0) {
                    for (j = 0; j< stream->par->extradata_size; j++) {
                        avio_w8(s, stream->par->extradata[j]); /* end of header */
                    }
                } else if (!stream->par->extradata_size &&
                           stream->par->codec_id == AV_CODEC_ID_RV60) {
                    avio_wb32(s, 0x01081000);
                    avio_wb32(s, 0x40008000);
                    avio_wb16(s, 0x0);
                    avio_wb16(s, stream->par->width);
                    avio_wb16(s, stream->par->height);
                }
            }
        }
    }

    /* patch data offset field */
    rm->data_pos = avio_tell(s);
    if (avio_seek(s, data_offset, SEEK_SET) >= 0) {
        avio_wb32(s, rm->data_pos);
        avio_seek(s, rm->data_pos, SEEK_SET);
    }

    /* data stream */
    ffio_wfourcc(s, "DATA");

    if (rm->mode & MODE_RMVB) {
        avio_wb32(s, data_size + 10 + 8);
        avio_wb16(s, 0);
        avio_wb32(s, nb_packets); /* number of packets */
        avio_wb32(s, 0);          /* next data header */
    } else {
        avio_wb32(s, 0);
        avio_wb16(s, 2);
        avio_wb32(s, nb_packets);     /* number of packets */
        avio_wb64(s, 0);              /* next data header */
        avio_wb64(s, data_size + 30); /* updated_size */
    }

    return 0;
}

static void write_packet_header(AVFormatContext *ctx, StreamInfo *stream,
                                int length, int key_frame, AVPacket *pkt)
{
    int timestamp;
    AVIOContext *s = ctx->pb;
    RMMuxContext *rm = ctx->priv_data;

    stream->nb_packets++;
    stream->packet_total_size += length;
    if (length > stream->packet_max_size)
        stream->packet_max_size =  length;

    avio_wb16(s,0); /* version */
    avio_wb16(s,length + 12);
    avio_wb16(s, stream->num); /* stream number */
    if(pkt && rm->mode & MODE_RMHD) {
        timestamp = pkt->pts;
    } else {
        timestamp = av_rescale_q_rnd(stream->nb_frames, (AVRational){1000, 1}, stream->frame_rate, AV_ROUND_ZERO);
    }
    avio_wb32(s, timestamp); /* timestamp */
    avio_w8(s, 0); /* reserved */
    avio_w8(s, key_frame ? 2 : 0); /* flags */
}

static int rm_write_header(AVFormatContext *s)
{
    RMMuxContext *rm = s->priv_data;
    StreamInfo *stream;
    int n;
    AVCodecParameters *par;

    rm->mode = MODE_RMVB;

    if (s->oformat && !strcmp("rmhd", s->oformat->name)) {
        rm->mode = MODE_RMHD;
    }

    if (s->nb_streams > 2) {
        av_log(s, AV_LOG_ERROR, "At most 2 streams are currently supported for muxing in RM\n");
        return AVERROR_PATCHWELCOME;
    }

    for(n=0;n<s->nb_streams;n++) {
        AVStream *st = s->streams[n];
        int frame_size;

        s->streams[n]->id = n;
        par = s->streams[n]->codecpar;
        stream = &rm->streams[n];
        memset(stream, 0, sizeof(StreamInfo));
        stream->num = n;
        stream->bit_rate = par->bit_rate;
        stream->par = par;

        if (rm->mode & MODE_RMHD) {
            avpriv_set_pts_info(st, 32, 1, 1000);
        }

        switch (par->codec_type) {
        case AVMEDIA_TYPE_AUDIO:
            rm->audio_stream = stream;
            frame_size = av_get_audio_frame_duration2(par, 0);
            stream->frame_rate = (AVRational){par->sample_rate, frame_size};
            /* XXX: dummy values */
            stream->packet_max_size = 1024;
            stream->nb_packets = 0;
            stream->total_frames = stream->nb_packets;
            break;
        case AVMEDIA_TYPE_VIDEO:
            rm->video_stream = stream;
            if (st->codecpar->codec_id == AV_CODEC_ID_RV40 ||
                st->codecpar->codec_id == AV_CODEC_ID_RV60) {
                stream->frame_rate = st->avg_frame_rate;
            } else {
                stream->frame_rate = av_inv_q(st->time_base);
            }
            /* XXX: dummy values */
            stream->packet_max_size = 4096;
            stream->nb_packets = 0;
            stream->total_frames = stream->nb_packets;
            break;
        default:
            return -1;
        }
    }

    if (rv10_write_header(s, 0, 0))
        return AVERROR_INVALIDDATA;
    avio_flush(s->pb);
    return 0;
}

static int rm_write_audio(AVFormatContext *s, const uint8_t *buf,
                          int size, int flags, AVPacket *pkt)
{
    RMMuxContext *rm = s->priv_data;
    AVIOContext *pb = s->pb;
    StreamInfo *stream = rm->audio_stream;
    int i;

    if (stream->par->codec_id == AV_CODEC_ID_AC3) {
        /* for AC-3, the words seem to be reversed */
        write_packet_header(s, stream, size, !!(flags & AV_PKT_FLAG_KEY), pkt);
        for (i = 0; i < size; i += 2) {
            avio_w8(pb, buf[i + 1]);
            avio_w8(pb, buf[i]);
        }
    } else if (stream->deint_id == DEINT_ID_VBRF &&
               stream->par->codec_id == AV_CODEC_ID_AAC) {
        write_packet_header(s, stream, size + 4, !!(flags & AV_PKT_FLAG_KEY), pkt);
        avio_wb16(pb, 0x10); // total frame size
        avio_wb16(pb, size & 0xFFFF); // total frame size
        avio_write(pb, buf, size);
    } else {
        write_packet_header(s, stream, size, !!(flags & AV_PKT_FLAG_KEY), pkt);
        avio_write(pb, buf, size);
    }
    stream->nb_frames++;
    return 0;
}

static int rm_write_video(AVFormatContext *s, const uint8_t *buf,
                          int size, int flags, AVPacket *pkt)
{
    RMMuxContext *rm = s->priv_data;
    AVIOContext *pb = s->pb;
    StreamInfo *stream = rm->video_stream;
    int key_frame = !!(flags & AV_PKT_FLAG_KEY);

    /* XXX: this is incorrect: should be a parameter */

    if (stream->par->codec_id == AV_CODEC_ID_RV30 ||
        stream->par->codec_id == AV_CODEC_ID_RV40 ||
        stream->par->codec_id == AV_CODEC_ID_RV60) {
         const uint8_t* seg_buf = NULL;
         int seg_offset = 0;
         int segments   = buf[0] + 1;
         int total_size = size - (1 + segments * 8);
         int seg_size;
         uint8_t byte1;
         uint8_t byte2;
         int part_val;
         int i;

         for (i = 0; i < segments; i++) {
            seg_buf = buf + 1 + segments * 8 + AV_RL32(buf + 5 + i * 8);
            if (segments - i > 1) {
                seg_size = AV_RL32(buf + 13 + 8 * i) - AV_RL32(buf + 5 + 8 * i);
            } else {
                seg_size = buf + size - seg_buf;
            }

            if ((i + 1) == segments) {
                byte1 = 0x80 | (0x3F & (segments >> 1));
                part_val = seg_size;
            } else {
                byte1 = 0x3F & (segments >> 1);
                part_val = seg_offset;
            }
            byte2 = (0x80 & (segments << 7)) | (0x7f & (i + 1));
            write_packet_header(s, stream, seg_size + 7 + (total_size >= 0x4000) * 2 + (part_val >= 0x4000) * 2, key_frame, pkt);
            avio_w8(pb, byte1);
            avio_w8(pb, byte2);
            if (total_size >= 0x4000) {
                avio_wb32(pb, total_size); // total frame size
            } else {
                avio_wb16(pb, 0x4000 | total_size); // total frame size
            }
            if (part_val >= 0x4000) {
                avio_wb32(pb, part_val); // offset from the start or the end
            } else {
                avio_wb16(pb, 0x4000 | part_val); // offset from the start or the end
            }
            seg_offset += seg_size;
            avio_w8(pb, stream->nb_frames & 0xff);
            avio_write(pb, seg_buf, seg_size);
        }
    }else{
        if (size > MAX_PACKET_SIZE) {
            av_log(s, AV_LOG_ERROR, "Muxing packets larger than 64 kB (%d) is not supported\n", size);
            return AVERROR_PATCHWELCOME;
          }
        write_packet_header(s, stream, size + 7 + (size >= 0x4000)*4, key_frame, pkt);
        // bit 7: '1' if final packet of a frame converted in several packets
        avio_w8(pb, 0x81);
        // bit 7: '1' if I frame. bits 6..0 : sequence number in current
        // frame starting from 1
        if (key_frame) {
            avio_w8(pb, 0x81);
        } else {
            avio_w8(pb, 0x01);
        }
        if (size >= 0x4000) {
            avio_wb32(pb, size); /* total frame size */
            avio_wb32(pb, size); /* offset from the start or the end */
        } else {
            avio_wb16(pb, 0x4000 | size); /* total frame size */
            avio_wb16(pb, 0x4000 | size); /* offset from the start or the end */
        }
        avio_w8(pb, stream->nb_frames & 0xff);

        avio_write(pb, buf, size);
    }

    stream->nb_frames++;
    return 0;
}

static int rm_write_packet(AVFormatContext *s, AVPacket *pkt)
{
    if (s->streams[pkt->stream_index]->codecpar->codec_type ==
        AVMEDIA_TYPE_AUDIO)
        return rm_write_audio(s, pkt->data, pkt->size, pkt->flags, pkt);
    else
        return rm_write_video(s, pkt->data, pkt->size, pkt->flags, pkt);
}

static int rm_write_trailer(AVFormatContext *s)
{
    RMMuxContext *rm = s->priv_data;
    int data_size, index_pos, i, j;
    AVIOContext *pb = s->pb;
    AVIndexEntry *entries          = NULL;
    unsigned char *data_offset_ptr = NULL;
    unsigned int index_data_pos    = 0;
    unsigned int timestamp, n_pkts;
    int64_t offset;

    if (s->pb->seekable & AVIO_SEEKABLE_NORMAL) {
        /* end of file: finish to write header */
        index_pos = avio_tell(pb);
        data_size = index_pos - rm->data_pos;

        /* write index */
        for (i = 0; i < s->nb_streams; i++) {
            AVStream *st = s->streams[i];
            entries      = st->index_entries;
            n_pkts       = st->nb_index_entries;
            if (n_pkts < 1) {
                break;
            }
            if (data_offset_ptr) {
                /* patch data offset field */
                index_data_pos = avio_tell(pb);
                data_offset_ptr[0] = index_data_pos >> 24;
                data_offset_ptr[1] = index_data_pos >> 16;
                data_offset_ptr[2] = index_data_pos >> 8;
                data_offset_ptr[3] = index_data_pos;
            }
            ffio_wfourcc(pb,"INDX");
            if (rm->mode & MODE_RMVB) {
                avio_wb32(pb, n_pkts * 14 + 20);
            } else {
                avio_wb32(pb, n_pkts * 18 + 24);
            }
            avio_wb16(pb, 2);
            avio_wb32(pb, n_pkts);
            avio_wb16(pb, st->id);
            data_offset_ptr = pb->buf_ptr;
            if (rm->mode & MODE_RMVB) {
                avio_wb32(pb,0);
            } else {
                avio_wb64(pb, 0); /* data offset gets written later */
            }

            for (j = 0; j < n_pkts; j++) {
                timestamp = entries[j].timestamp;
                offset    = entries[j].pos;
                avio_wb16(pb, 2);
                avio_wb32(pb, timestamp);
                if (rm->mode & MODE_RMVB) {
                    avio_wb32(pb, (int32_t)offset);
                } else {
                    avio_wb64(pb, offset);
                }
                avio_wb32(pb, 0);
            }
        }

        /* undocumented end header */
        avio_wb32(pb, 0);
        avio_wb32(pb, 0);

        avio_seek(pb, 0, SEEK_SET);
        for(i=0;i<s->nb_streams;i++)
            rm->streams[i].total_frames = rm->streams[i].nb_frames;
        rv10_write_header(s, data_size, 0);
    } else {
        /* undocumented end header */
        avio_wb32(pb, 0);
        avio_wb32(pb, 0);
    }

    return 0;
}


AVOutputFormat ff_rm_muxer = {
    .name              = "rm",
    .long_name         = NULL_IF_CONFIG_SMALL("RealMedia"),
    .mime_type         = "application/vnd.rn-realmedia",
    .extensions        = "rm,ra",
    .priv_data_size    = sizeof(RMMuxContext),
    .audio_codec       = AV_CODEC_ID_AC3,
    .video_codec       = AV_CODEC_ID_RV10,
    .write_header      = rm_write_header,
    .write_packet      = rm_write_packet,
    .write_trailer     = rm_write_trailer,
    .codec_tag         = (const AVCodecTag* const []){ ff_rm_codec_tags, 0 },
};

AVOutputFormat ff_rmhd_muxer = {
    .name              = "rmhd",
    .long_name         = NULL_IF_CONFIG_SMALL("RealMedia HD"),
    .mime_type         = "application/vnd.rn-realmedia",
    .extensions        = "rmhd",
    .priv_data_size    = sizeof(RMMuxContext),
    .audio_codec       = AV_CODEC_ID_AAC,
    .video_codec       = AV_CODEC_ID_RV60,
    .write_header      = rm_write_header,
    .write_packet      = rm_write_packet,
    .write_trailer     = rm_write_trailer,
    .codec_tag         = (const AVCodecTag* const []){ ff_rm_codec_tags, 0 },
};
