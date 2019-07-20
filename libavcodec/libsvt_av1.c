/*
* Scalable Video Technology for AV1 encoder library plugin
*
* Copyright (c) 2018 Intel Corporation
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
* License along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
*/

#include <stdint.h>
#include "EbSvtAv1ErrorCodes.h"
#include "EbSvtAv1Enc.h"

#include "libavutil/common.h"
#include "libavutil/frame.h"
#include "libavutil/opt.h"

#include "internal.h"
#include "avcodec.h"

typedef struct SvtContext {
    AVClass     *class;

    EbSvtAv1EncConfiguration    enc_params;
    EbComponentType            *svt_handle;

    EbBufferHeaderType         *in_buf;
    int                         raw_size;

    int         eos_flag;

    // User options.
    int hierarchical_level;
    int la_depth;
    int enc_mode;
    int rc_mode;
    int scd;
    int qp;

    int forced_idr;

    int aud;

    int tier;
    int level;

    int base_layer_switch_mode;

    int tile_columns;
    int tile_rows;
} SvtContext;

static int error_mapping(EbErrorType svt_ret)
{
    int err;

    switch (svt_ret) {
    case EB_ErrorInsufficientResources:
        err = AVERROR(ENOMEM);
        break;

    case EB_ErrorUndefined:
    case EB_ErrorInvalidComponent:
    case EB_ErrorBadParameter:
        err = AVERROR(EINVAL);
        break;

    case EB_ErrorDestroyThreadFailed:
    case EB_ErrorSemaphoreUnresponsive:
    case EB_ErrorDestroySemaphoreFailed:
    case EB_ErrorCreateMutexFailed:
    case EB_ErrorMutexUnresponsive:
    case EB_ErrorDestroyMutexFailed:
        err = AVERROR_EXTERNAL;
            break;

    case EB_NoErrorEmptyQueue:
        err = AVERROR(EAGAIN);

    case EB_ErrorNone:
        err = 0;
        break;

    default:
        err = AVERROR_UNKNOWN;
    }

    return err;
}

static void free_buffer(SvtContext *svt_enc)
{
    if (svt_enc->in_buf) {
        EbSvtIOFormat *in_data = (EbSvtIOFormat *)svt_enc->in_buf->p_buffer;
        av_freep(&in_data);
        av_freep(&svt_enc->in_buf);
    }
}

static int alloc_buffer(EbSvtAv1EncConfiguration *config, SvtContext *svt_enc)
{
    const int    pack_mode_10bit   =
        (config->encoder_bit_depth > 8) && (config->compressed_ten_bit_format == 0) ? 1 : 0;
    const size_t luma_size_8bit    =
        config->source_width * config->source_height * (1 << pack_mode_10bit);
    const size_t luma_size_10bit   =
        (config->encoder_bit_depth > 8 && pack_mode_10bit == 0) ? luma_size_8bit : 0;

    EbSvtIOFormat *in_data;

    svt_enc->raw_size = (luma_size_8bit + luma_size_10bit) * 3 / 2;

    // allocate buffer for in and out
    svt_enc->in_buf           = av_mallocz(sizeof(*svt_enc->in_buf));
    if (!svt_enc->in_buf)
        goto failed;

    in_data  = av_mallocz(sizeof(*in_data));
    if (!in_data)
        goto failed;
    svt_enc->in_buf->p_buffer  = (unsigned char *)in_data;

    svt_enc->in_buf->size        = sizeof(*svt_enc->in_buf);
    svt_enc->in_buf->p_app_private  = NULL;

    return 0;

failed:
    free_buffer(svt_enc);
    return AVERROR(ENOMEM);
}

static int config_enc_params(EbSvtAv1EncConfiguration *param,
                             AVCodecContext *avctx)
{
    SvtContext *svt_enc = avctx->priv_data;
    int             ret;
    int        ten_bits = 0;

    param->source_width     = avctx->width;
    param->source_height    = avctx->height;

    if (avctx->pix_fmt == AV_PIX_FMT_YUV420P10LE) {
        av_log(avctx, AV_LOG_DEBUG , "Encoder 10 bits depth input\n");
        // Disable Compressed 10-bit format default
        //
        // SVT-AV1 support a compressed 10-bit format allowing the
        // software to achieve a higher speed and channel density levels.
        // The conversion between the 10-bit yuv420p10le and the compressed
        // 10-bit format is a lossless operation. But in FFmpeg, we usually
        // didn't use this format
        param->compressed_ten_bit_format = 0;
        ten_bits = 1;
    }

    // Update param from options
    param->hierarchical_levels     = svt_enc->hierarchical_level;
    param->enc_mode                = svt_enc->enc_mode;
    param->tier                   = svt_enc->tier;
    param->level                  = svt_enc->level;
    param->rate_control_mode        = svt_enc->rc_mode;
    param->scene_change_detection   = svt_enc->scd;
    param->base_layer_switch_mode    = svt_enc->base_layer_switch_mode;
    param->qp                     = svt_enc->qp;


    param->target_bit_rate          = avctx->bit_rate;
    if (avctx->gop_size > 0)
        param->intra_period_length  = avctx->gop_size - 1;

    if (avctx->framerate.num > 0 && avctx->framerate.den > 0) {
        param->frame_rate_numerator     = avctx->framerate.num;
        param->frame_rate_denominator   = avctx->framerate.den * avctx->ticks_per_frame;
    } else {
        param->frame_rate_numerator     = avctx->time_base.den;
        param->frame_rate_denominator   = avctx->time_base.num * avctx->ticks_per_frame;
    }

    if (param->rate_control_mode) {
        param->max_qp_allowed       = avctx->qmax;
        param->min_qp_allowed       = avctx->qmin;
    }

    param->intra_refresh_type       =
        !!(avctx->flags & AV_CODEC_FLAG_CLOSED_GOP) + 1;

    if (svt_enc->la_depth != -1)
        param->look_ahead_distance  = svt_enc->la_depth;

    if (ten_bits) {
        param->encoder_bit_depth        = 10;
    }

    param->tile_columns = svt_enc->tile_columns;
    param->tile_rows = svt_enc->tile_rows;

    if (avctx->thread_count > 0) {
        param->logical_processors = avctx->thread_count;
    }

    ret = alloc_buffer(param, svt_enc);

    return ret;
}

static void read_in_data(EbSvtAv1EncConfiguration *config,
                         const AVFrame *frame,
                         EbBufferHeaderType *headerPtr)
{
    uint8_t is16bit = config->encoder_bit_depth > 8;
    uint64_t luma_size =
        (uint64_t)config->source_width * config->source_height<< is16bit;
    EbSvtIOFormat *in_data = (EbSvtIOFormat *)headerPtr->p_buffer;

    // support yuv420p and yuv420p010
    in_data->luma = frame->data[0];
    in_data->cb   = frame->data[1];
    in_data->cr   = frame->data[2];

    // stride info
    in_data->y_stride  = frame->linesize[0] >> is16bit;
    in_data->cb_stride = frame->linesize[1] >> is16bit;
    in_data->cr_stride = frame->linesize[2] >> is16bit;

    headerPtr->n_filled_len   += luma_size * 3/2u;
}

static av_cold int eb_enc_init(AVCodecContext *avctx)
{
    SvtContext   *svt_enc = avctx->priv_data;
    EbErrorType svt_ret;

    svt_enc->eos_flag = 0;

    svt_ret = eb_init_handle(&svt_enc->svt_handle, svt_enc, &svt_enc->enc_params);
    if (svt_ret != EB_ErrorNone) {
        av_log(avctx, AV_LOG_ERROR, "Error init encoder handle\n");
        goto failed;
    }

    svt_ret = config_enc_params(&svt_enc->enc_params, avctx);
    if (svt_ret != EB_ErrorNone) {
        av_log(avctx, AV_LOG_ERROR, "Error configure encoder parameters\n");
        goto failed_init_handle;
    }

    svt_ret = eb_svt_enc_set_parameter(svt_enc->svt_handle, &svt_enc->enc_params);
    if (svt_ret != EB_ErrorNone) {
        av_log(avctx, AV_LOG_ERROR, "Error setting encoder parameters\n");
        goto failed_init_handle;
    }

    svt_ret = eb_init_encoder(svt_enc->svt_handle);
    if (svt_ret != EB_ErrorNone) {
        av_log(avctx, AV_LOG_ERROR, "Error init encoder\n");
        goto failed_init_handle;
    }

    if (avctx->flags & AV_CODEC_FLAG_GLOBAL_HEADER) {
        EbBufferHeaderType *headerPtr = NULL;

        svt_ret = eb_svt_enc_stream_header(svt_enc->svt_handle, &headerPtr);
        if (svt_ret != EB_ErrorNone) {
            av_log(avctx, AV_LOG_ERROR, "Error when build stream header.\n");
            goto failed_init_enc;
        }

        avctx->extradata_size = headerPtr->n_filled_len;
        avctx->extradata = av_mallocz(avctx->extradata_size + AV_INPUT_BUFFER_PADDING_SIZE);
        if (!avctx->extradata) {
            av_log(avctx, AV_LOG_ERROR,
                   "Cannot allocate AV1 header of size %d.\n", avctx->extradata_size);
            svt_ret = EB_ErrorInsufficientResources;
            goto failed_init_enc;
        }

        memcpy(avctx->extradata, headerPtr->p_buffer, avctx->extradata_size);

        svt_ret = eb_svt_release_enc_stream_header(headerPtr);
        if (svt_ret != EB_ErrorNone) {
            av_log(avctx, AV_LOG_ERROR, "Error when destroy stream header.\n");
            goto failed_init_enc;
        }
    }

    return 0;

failed_init_enc:
    eb_deinit_encoder(svt_enc->svt_handle);
failed_init_handle:
    eb_deinit_handle(svt_enc->svt_handle);
failed:
    free_buffer(svt_enc);
    return error_mapping(svt_ret);
}

static int eb_send_frame(AVCodecContext *avctx, const AVFrame *frame)
{
    SvtContext           *svt_enc = avctx->priv_data;
    EbBufferHeaderType  *headerPtr = svt_enc->in_buf;

    if (!frame) {
        EbBufferHeaderType headerPtrLast;
        headerPtrLast.n_alloc_len   = 0;
        headerPtrLast.n_filled_len  = 0;
        headerPtrLast.n_tick_count  = 0;
        headerPtrLast.p_app_private = NULL;
        headerPtrLast.p_buffer     = NULL;
        headerPtrLast.flags      = EB_BUFFERFLAG_EOS;

        eb_svt_enc_send_picture(svt_enc->svt_handle, &headerPtrLast);
        svt_enc->eos_flag = 1;
        av_log(avctx, AV_LOG_DEBUG, "Finish sending frames!!!\n");
        return 0;
    }

    read_in_data(&svt_enc->enc_params, frame, headerPtr);

    headerPtr->flags       = 0;
    headerPtr->p_app_private  = NULL;
    headerPtr->pts          = frame->pts;
    switch (frame->pict_type) {
    case AV_PICTURE_TYPE_I:
        headerPtr->pic_type = svt_enc->forced_idr > 0 ? EB_AV1_KEY_PICTURE : EB_AV1_INTRA_ONLY_PICTURE;
        break;
    case AV_PICTURE_TYPE_P:
        headerPtr->pic_type = EB_AV1_ALT_REF_PICTURE;
        break;
    case AV_PICTURE_TYPE_B:
        headerPtr->pic_type = EB_AV1_INTER_PICTURE;
        break;
    default:
        headerPtr->pic_type = EB_AV1_INVALID_PICTURE;
        break;
    }
    eb_svt_enc_send_picture(svt_enc->svt_handle, headerPtr);

    return 0;
}

static int eb_receive_packet(AVCodecContext *avctx, AVPacket *pkt)
{
    SvtContext  *svt_enc = avctx->priv_data;
    EbBufferHeaderType   *headerPtr;
    EbErrorType          svt_ret;
    int ret;

    if ((ret = ff_alloc_packet2(avctx, pkt, svt_enc->raw_size, 0)) < 0) {
        av_log(avctx, AV_LOG_ERROR, "Failed to allocate output packet.\n");
        return ret;
    }
    svt_ret = eb_svt_get_packet(svt_enc->svt_handle, &headerPtr, svt_enc->eos_flag);
    if (svt_ret == EB_NoErrorEmptyQueue)
        return AVERROR(EAGAIN);

    memcpy(pkt->data, headerPtr->p_buffer, headerPtr->n_filled_len);
    pkt->size = headerPtr->n_filled_len;
    pkt->pts  = headerPtr->pts;
    pkt->dts  = headerPtr->dts;
    if (headerPtr->pic_type == EB_AV1_KEY_PICTURE)
        pkt->flags |= AV_PKT_FLAG_KEY;
    if (headerPtr->pic_type == EB_AV1_NON_REF_PICTURE)
        pkt->flags |= AV_PKT_FLAG_DISPOSABLE;

    ret = (headerPtr->flags & EB_BUFFERFLAG_EOS) ? AVERROR_EOF : 0;

    eb_svt_release_out_buffer(&headerPtr);

    return ret;
}

static av_cold int eb_enc_close(AVCodecContext *avctx)
{
    SvtContext *svt_enc = avctx->priv_data;

    eb_deinit_encoder(svt_enc->svt_handle);
    eb_deinit_handle(svt_enc->svt_handle);

    free_buffer(svt_enc);

    return 0;
}

#define OFFSET(x) offsetof(SvtContext, x)
#define VE AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_ENCODING_PARAM
static const AVOption options[] = {
    { "aud", "Include AUD", OFFSET(aud),
      AV_OPT_TYPE_BOOL, { .i64 = 0 }, 0, 1, VE },

    { "hielevel", "Hierarchical prediction levels setting", OFFSET(hierarchical_level),
      AV_OPT_TYPE_INT, { .i64 = 4 }, 3, 4, VE , "hielevel"},
        { "flat",   NULL, 0, AV_OPT_TYPE_CONST, { .i64 = 1 },  INT_MIN, INT_MAX, VE, "hielevel" },
        { "2level", NULL, 0, AV_OPT_TYPE_CONST, { .i64 = 2 },  INT_MIN, INT_MAX, VE, "hielevel" },
        { "3level", NULL, 0, AV_OPT_TYPE_CONST, { .i64 = 3 },  INT_MIN, INT_MAX, VE, "hielevel" },
        { "4level", NULL, 0, AV_OPT_TYPE_CONST, { .i64 = 4 },  INT_MIN, INT_MAX, VE, "hielevel" },

    { "la_depth", "Look ahead distance [0, 256]", OFFSET(la_depth),
      AV_OPT_TYPE_INT, { .i64 = -1 }, -1, 256, VE },

    { "preset", "Encoding preset [0, 7]",
      OFFSET(enc_mode), AV_OPT_TYPE_INT, { .i64 = MAX_ENC_PRESET }, 0, MAX_ENC_PRESET, VE },

    { "tier", "Set tier (general_tier_flag)", OFFSET(tier),
      AV_OPT_TYPE_INT, { .i64 = 0 }, 0, 1, VE, "tier" },
        { "main", NULL, 0, AV_OPT_TYPE_CONST, { .i64 = 0 }, 0, 0, VE, "tier" },
        { "high", NULL, 0, AV_OPT_TYPE_CONST, { .i64 = 1 }, 0, 0, VE, "tier" },

    { "level", "Set level (level_idc)", OFFSET(level),
      AV_OPT_TYPE_INT, { .i64 = 0 }, 0, 0xff, VE, "level" },

#define LEVEL(name, value) name, NULL, 0, AV_OPT_TYPE_CONST, \
      { .i64 = value }, 0, 0, VE, "level"
        { LEVEL("1",   10) },
        { LEVEL("2",   20) },
        { LEVEL("2.1", 21) },
        { LEVEL("3",   30) },
        { LEVEL("3.1", 31) },
        { LEVEL("4",   40) },
        { LEVEL("4.1", 41) },
        { LEVEL("5",   50) },
        { LEVEL("5.1", 51) },
        { LEVEL("5.2", 52) },
        { LEVEL("6",   60) },
        { LEVEL("6.1", 61) },
        { LEVEL("6.2", 62) },
#undef LEVEL

    { "rc", "Bit rate control mode", OFFSET(rc_mode),
      AV_OPT_TYPE_INT, { .i64 = 0 }, 0, 2, VE , "rc"},
        { "cqp", NULL, 0, AV_OPT_TYPE_CONST, { .i64 = 0 },  INT_MIN, INT_MAX, VE, "rc" },
        { "abr", NULL, 0, AV_OPT_TYPE_CONST, { .i64 = 1 },  INT_MIN, INT_MAX, VE, "rc" },
        { "vbr", NULL, 0, AV_OPT_TYPE_CONST, { .i64 = 2 },  INT_MIN, INT_MAX, VE, "rc" },

    { "qp", "QP value for intra frames", OFFSET(qp),
      AV_OPT_TYPE_INT, { .i64 = 50 }, 0, 63, VE },

    { "sc_detection", "Scene change detection", OFFSET(scd),
      AV_OPT_TYPE_BOOL, { .i64 = 0 }, 0, 1, VE },

    { "bl_mode", "Random Access Prediction Structure type setting", OFFSET(base_layer_switch_mode),
      AV_OPT_TYPE_BOOL, { .i64 = 0 }, 0, 1, VE },

    { "forced-idr", "If forcing keyframes, force them as IDR frames.", OFFSET(forced_idr),
      AV_OPT_TYPE_BOOL,   { .i64 = 0 }, -1, 1, VE },

    { "tile-columns", "log2 of tile columns", OFFSET(tile_columns),
      AV_OPT_TYPE_INT, { .i64 = 0 }, 0, 6, VE },

    { "tile-rows", "log2 of tile rows", OFFSET(tile_rows),
      AV_OPT_TYPE_INT, { .i64 = 0 }, 0, 6, VE },

    {NULL},
};

static const AVClass class = {
    .class_name = "libsvt_av1",
    .item_name  = av_default_item_name,
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

static const AVCodecDefault eb_enc_defaults[] = {
    { "b",            "7M"    },
    { "g",            "-2"    },
    { "flags",        "-cgop" },
    { "qmin",         "0"     },
    { "qmax",         "63"    },
    { NULL },
};

AVCodec ff_libsvt_av1_encoder = {
    .name           = "libsvt_av1",
    .long_name      = NULL_IF_CONFIG_SMALL("SVT-AV1(Scalable Video Technology for AV1) encoder"),
    .priv_data_size = sizeof(SvtContext),
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = AV_CODEC_ID_AV1,
    .init           = eb_enc_init,
    .send_frame     = eb_send_frame,
    .receive_packet = eb_receive_packet,
    .close          = eb_enc_close,
    .capabilities   = AV_CODEC_CAP_DELAY | AV_CODEC_CAP_AUTO_THREADS,
    .pix_fmts       = (const enum AVPixelFormat[]){ AV_PIX_FMT_YUV420P,
                                                    AV_PIX_FMT_NONE },
    .priv_class     = &class,
    .defaults       = eb_enc_defaults,
    .caps_internal  = FF_CODEC_CAP_INIT_CLEANUP,
    .wrapper_name   = "libsvt_av1",
};
