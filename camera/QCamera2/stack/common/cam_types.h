/* Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __QCAMERA_TYPES_H__
#define __QCAMERA_TYPES_H__

#include <stdint.h>
#include <pthread.h>
#include <inttypes.h>
#include <media/msmb_camera.h>

#define CAM_MAX_NUM_BUFS_PER_STREAM 64
#define MAX_METADATA_PRIVATE_PAYLOAD_SIZE_IN_BYTES 8096
#define AWB_DEBUG_DATA_SIZE               (45000)
#define AEC_DEBUG_DATA_SIZE               (5000)
#define AF_DEBUG_DATA_SIZE                (10000)
#define AF_STATS_DEBUG_DATA_SIZE          (40000)
#define ASD_DEBUG_DATA_SIZE               (100)
#define STATS_BUFFER_DEBUG_DATA_SIZE      (75000)

#define CEILING64(X) (((X) + 0x0003F) & 0xFFFFFFC0)
#define CEILING32(X) (((X) + 0x0001F) & 0xFFFFFFE0)
#define CEILING16(X) (((X) + 0x000F) & 0xFFF0)
#define CEILING4(X)  (((X) + 0x0003) & 0xFFFC)
#define CEILING2(X)  (((X) + 0x0001) & 0xFFFE)

#define MAX_ZOOMS_CNT 91
#define MAX_SIZES_CNT 40
#define MAX_EXP_BRACKETING_LENGTH 32
#define MAX_ROI 5
#define MAX_STREAM_NUM_IN_BUNDLE 6
#define MAX_NUM_STREAMS          8
#define CHROMATIX_SIZE 60000
#define COMMONCHROMATIX_SIZE 45000
#define CPPCHROMATIX_SIZE 36000
#define SWPOSTPROCCHROMATIX_SIZE 36000
#define AFTUNE_SIZE  32768
#define A3CHROMATIX_SIZE 30000
#define MAX_SCALE_SIZES_CNT 8
#define MAX_SAMP_DECISION_CNT     64
#define SENSOR_PHYSICAL_SIZE_CNT  2
#define EXPOSURE_TIME_RANGE_CNT   2
#define BLACK_LEVEL_PATTERN_CNT   4
#define FORWARD_MATRIX_COLS       3
#define FORWARD_MATRIX_ROWS       3
#define COLOR_TRANSFORM_COLS      3
#define COLOR_TRANSFORM_ROWS      3
#define CAL_TRANSFORM_COLS        3
#define CAL_TRANSFORM_ROWS        3

#define MAX_ISP_DATA_SIZE (20*1024)
#define MAX_PP_DATA_SIZE  16384
#define MAX_AE_STATS_DATA_SIZE  1000
#define MAX_AWB_STATS_DATA_SIZE 1000
#define MAX_AF_STATS_DATA_SIZE  1000
#define MAX_ASD_STATS_DATA_SIZE 1000

#define MAX_CAPTURE_BATCH_NUM 32

#define TUNING_DATA_VERSION        6
#define TUNING_SENSOR_DATA_MAX     0x10000 /*(need value from sensor team)*/
#define TUNING_VFE_DATA_MAX        0x10000 /*(need value from vfe team)*/
#define TUNING_CPP_DATA_MAX        0x10000 /*(need value from pproc team)*/
#define TUNING_CAC_DATA_MAX        0x10000 /*(need value from imglib team)*/
#define TUNING_DATA_MAX            (TUNING_SENSOR_DATA_MAX + \
                                   TUNING_VFE_DATA_MAX + TUNING_CPP_DATA_MAX + \
                                   TUNING_CAC_DATA_MAX)

#define TUNING_SENSOR_DATA_OFFSET  0
#define TUNING_VFE_DATA_OFFSET     TUNING_SENSOR_DATA_MAX
#define TUNING_CPP_DATA_OFFSET     (TUNING_SENSOR_DATA_MAX + TUNING_VFE_DATA_MAX)
#define TUNING_CAC_DATA_OFFSET     (TUNING_SENSOR_DATA_MAX + \
                                   TUNING_VFE_DATA_MAX + TUNING_CPP_DATA_MAX)
#define MAX_STATS_DATA_SIZE 4000

#define MAX_AF_BRACKETING_VALUES 5
#define MAX_TEST_PATTERN_CNT     8

#define GPS_PROCESSING_METHOD_SIZE 33

#define MAX_INFLIGHT_REQUESTS  6
#define MIN_INFLIGHT_REQUESTS  3

#define QCAMERA_DUMP_FRM_LOCATION "/data/misc/camera/"
#define QCAMERA_MAX_FILEPATH_LENGTH 64

#define RELCAM_CALIB_ROT_MATRIX_MAX 9
#define RELCAM_CALIB_SURFACE_PARMS_MAX 32
#define RELCAM_CALIB_RESERVED_MAX 62

#define MAX_NUM_CAMERA_PER_BUNDLE    2 /* Max number of cameras per bundle */
#define EXTRA_FRAME_SYNC_BUFFERS     4 /* Extra frame sync buffers in dc mode*/
#define MM_CAMERA_FRAME_SYNC_NODES   EXTRA_FRAME_SYNC_BUFFERS

typedef enum {
    CAM_HAL_V1 = 1,
    CAM_HAL_V3 = 3
} cam_hal_version_t;

typedef enum {
    CAM_STATUS_SUCCESS,       /* Operation Succeded */
    CAM_STATUS_FAILED,        /* Failure in doing operation */
    CAM_STATUS_INVALID_PARM,  /* Inavlid parameter provided */
    CAM_STATUS_NOT_SUPPORTED, /* Parameter/operation not supported */
    CAM_STATUS_ACCEPTED,      /* Parameter accepted */
    CAM_STATUS_MAX,
} cam_status_t;

typedef enum {
    /* back main camera */
    CAM_POSITION_BACK,
    /* front main camera */
    CAM_POSITION_FRONT,
    /* back aux camera */
    CAM_POSITION_BACK_AUX,
    /* front aux camera */
    CAM_POSITION_FRONT_AUX
} cam_position_t;

typedef enum {
    CAM_FLICKER_NONE,
    CAM_FLICKER_50_HZ,
    CAM_FLICKER_60_HZ
} cam_flicker_t;

typedef enum {
    CAM_FORMAT_JPEG = 0,
    CAM_FORMAT_YUV_420_NV12 = 1,
    CAM_FORMAT_YUV_420_NV21,
    CAM_FORMAT_YUV_420_NV21_ADRENO,
    CAM_FORMAT_YUV_420_YV12,
    CAM_FORMAT_YUV_422_NV16,
    CAM_FORMAT_YUV_422_NV61,
    CAM_FORMAT_YUV_420_NV12_VENUS,
    /* Note: For all raw formats, each scanline needs to be 16 bytes aligned */

    /* Packed YUV/YVU raw format, 16 bpp: 8 bits Y and 8 bits UV.
     * U and V are interleaved with Y: YUYV or YVYV */
    CAM_FORMAT_YUV_RAW_8BIT_YUYV,
    CAM_FORMAT_YUV_RAW_8BIT_YVYU,
    CAM_FORMAT_YUV_RAW_8BIT_UYVY,
    CAM_FORMAT_YUV_RAW_8BIT_VYUY,

    /* QCOM RAW formats where data is packed into 64bit word.
     * 8BPP: 1 64-bit word contains 8 pixels p0 - p7, where p0 is
     *       stored at LSB.
     * 10BPP: 1 64-bit word contains 6 pixels p0 - p5, where most
     *       significant 4 bits are set to 0. P0 is stored at LSB.
     * 12BPP: 1 64-bit word contains 5 pixels p0 - p4, where most
     *       significant 4 bits are set to 0. P0 is stored at LSB. */
    CAM_FORMAT_BAYER_QCOM_RAW_8BPP_GBRG,
    CAM_FORMAT_BAYER_QCOM_RAW_8BPP_GRBG,
    CAM_FORMAT_BAYER_QCOM_RAW_8BPP_RGGB,
    CAM_FORMAT_BAYER_QCOM_RAW_8BPP_BGGR,
    CAM_FORMAT_BAYER_QCOM_RAW_10BPP_GBRG,
    CAM_FORMAT_BAYER_QCOM_RAW_10BPP_GRBG,
    CAM_FORMAT_BAYER_QCOM_RAW_10BPP_RGGB,
    CAM_FORMAT_BAYER_QCOM_RAW_10BPP_BGGR,
    CAM_FORMAT_BAYER_QCOM_RAW_12BPP_GBRG,
    CAM_FORMAT_BAYER_QCOM_RAW_12BPP_GRBG,
    CAM_FORMAT_BAYER_QCOM_RAW_12BPP_RGGB,
    CAM_FORMAT_BAYER_QCOM_RAW_12BPP_BGGR,
    /* MIPI RAW formats based on MIPI CSI-2 specifiction.
     * 8BPP: Each pixel occupies one bytes, starting at LSB.
     *       Output with of image has no restrictons.
     * 10BPP: Four pixels are held in every 5 bytes. The output
     *       with of image must be a multiple of 4 pixels.
     * 12BPP: Two pixels are held in every 3 bytes. The output
     *       width of image must be a multiple of 2 pixels. */
    CAM_FORMAT_BAYER_MIPI_RAW_8BPP_GBRG,
    CAM_FORMAT_BAYER_MIPI_RAW_8BPP_GRBG,
    CAM_FORMAT_BAYER_MIPI_RAW_8BPP_RGGB,
    CAM_FORMAT_BAYER_MIPI_RAW_8BPP_BGGR,
    CAM_FORMAT_BAYER_MIPI_RAW_10BPP_GBRG,
    CAM_FORMAT_BAYER_MIPI_RAW_10BPP_GRBG,
    CAM_FORMAT_BAYER_MIPI_RAW_10BPP_RGGB,
    CAM_FORMAT_BAYER_MIPI_RAW_10BPP_BGGR,
    CAM_FORMAT_BAYER_MIPI_RAW_12BPP_GBRG,
    CAM_FORMAT_BAYER_MIPI_RAW_12BPP_GRBG,
    CAM_FORMAT_BAYER_MIPI_RAW_12BPP_RGGB,
    CAM_FORMAT_BAYER_MIPI_RAW_12BPP_BGGR,
    /* Ideal raw formats where image data has gone through black
     * correction, lens rolloff, demux/channel gain, bad pixel
     * correction, and ABF.
     * Ideal raw formats could output any of QCOM_RAW and MIPI_RAW
     * formats, plus plain8 8bbp, plain16 800, plain16 10bpp, and
     * plain 16 12bpp */
    CAM_FORMAT_BAYER_IDEAL_RAW_QCOM_8BPP_GBRG,
    CAM_FORMAT_BAYER_IDEAL_RAW_QCOM_8BPP_GRBG,
    CAM_FORMAT_BAYER_IDEAL_RAW_QCOM_8BPP_RGGB,
    CAM_FORMAT_BAYER_IDEAL_RAW_QCOM_8BPP_BGGR,
    CAM_FORMAT_BAYER_IDEAL_RAW_QCOM_10BPP_GBRG,
    CAM_FORMAT_BAYER_IDEAL_RAW_QCOM_10BPP_GRBG,
    CAM_FORMAT_BAYER_IDEAL_RAW_QCOM_10BPP_RGGB,
    CAM_FORMAT_BAYER_IDEAL_RAW_QCOM_10BPP_BGGR,
    CAM_FORMAT_BAYER_IDEAL_RAW_QCOM_12BPP_GBRG,
    CAM_FORMAT_BAYER_IDEAL_RAW_QCOM_12BPP_GRBG,
    CAM_FORMAT_BAYER_IDEAL_RAW_QCOM_12BPP_RGGB,
    CAM_FORMAT_BAYER_IDEAL_RAW_QCOM_12BPP_BGGR,
    CAM_FORMAT_BAYER_IDEAL_RAW_MIPI_8BPP_GBRG,
    CAM_FORMAT_BAYER_IDEAL_RAW_MIPI_8BPP_GRBG,
    CAM_FORMAT_BAYER_IDEAL_RAW_MIPI_8BPP_RGGB,
    CAM_FORMAT_BAYER_IDEAL_RAW_MIPI_8BPP_BGGR,
    CAM_FORMAT_BAYER_IDEAL_RAW_MIPI_10BPP_GBRG,
    CAM_FORMAT_BAYER_IDEAL_RAW_MIPI_10BPP_GRBG,
    CAM_FORMAT_BAYER_IDEAL_RAW_MIPI_10BPP_RGGB,
    CAM_FORMAT_BAYER_IDEAL_RAW_MIPI_10BPP_BGGR,
    CAM_FORMAT_BAYER_IDEAL_RAW_MIPI_12BPP_GBRG,
    CAM_FORMAT_BAYER_IDEAL_RAW_MIPI_12BPP_GRBG,
    CAM_FORMAT_BAYER_IDEAL_RAW_MIPI_12BPP_RGGB,
    CAM_FORMAT_BAYER_IDEAL_RAW_MIPI_12BPP_BGGR,
    CAM_FORMAT_BAYER_IDEAL_RAW_PLAIN8_8BPP_GBRG,
    CAM_FORMAT_BAYER_IDEAL_RAW_PLAIN8_8BPP_GRBG,
    CAM_FORMAT_BAYER_IDEAL_RAW_PLAIN8_8BPP_RGGB,
    CAM_FORMAT_BAYER_IDEAL_RAW_PLAIN8_8BPP_BGGR,
    CAM_FORMAT_BAYER_IDEAL_RAW_PLAIN16_8BPP_GBRG,
    CAM_FORMAT_BAYER_IDEAL_RAW_PLAIN16_8BPP_GRBG,
    CAM_FORMAT_BAYER_IDEAL_RAW_PLAIN16_8BPP_RGGB,
    CAM_FORMAT_BAYER_IDEAL_RAW_PLAIN16_8BPP_BGGR,
    CAM_FORMAT_BAYER_IDEAL_RAW_PLAIN16_10BPP_GBRG,
    CAM_FORMAT_BAYER_IDEAL_RAW_PLAIN16_10BPP_GRBG,
    CAM_FORMAT_BAYER_IDEAL_RAW_PLAIN16_10BPP_RGGB,
    CAM_FORMAT_BAYER_IDEAL_RAW_PLAIN16_10BPP_BGGR,
    CAM_FORMAT_BAYER_IDEAL_RAW_PLAIN16_12BPP_GBRG,
    CAM_FORMAT_BAYER_IDEAL_RAW_PLAIN16_12BPP_GRBG,
    CAM_FORMAT_BAYER_IDEAL_RAW_PLAIN16_12BPP_RGGB,
    CAM_FORMAT_BAYER_IDEAL_RAW_PLAIN16_12BPP_BGGR,

    /* generic 8-bit raw */
    CAM_FORMAT_JPEG_RAW_8BIT,
    CAM_FORMAT_META_RAW_8BIT,

    /* QCOM RAW formats where data is packed into 64bit word.
     * 14BPP: 1 64-bit word contains 4 pixels p0 - p3, where most
     *       significant 4 bits are set to 0. P0 is stored at LSB.
     */
    CAM_FORMAT_BAYER_QCOM_RAW_14BPP_GBRG,
    CAM_FORMAT_BAYER_QCOM_RAW_14BPP_GRBG,
    CAM_FORMAT_BAYER_QCOM_RAW_14BPP_RGGB,
    CAM_FORMAT_BAYER_QCOM_RAW_14BPP_BGGR,
    /* MIPI RAW formats based on MIPI CSI-2 specifiction.
     * 14 BPPP: 1st byte: P0 [13:6]
     *          2nd byte: P1 [13:6]
     *          3rd byte: P2 [13:6]
     *          4th byte: P3 [13:6]
     *          5th byte: P0 [5:0]
     *          7th byte: P1 [5:0]
     *          8th byte: P2 [5:0]
     *          9th byte: P3 [5:0]
     */
    CAM_FORMAT_BAYER_MIPI_RAW_14BPP_GBRG,
    CAM_FORMAT_BAYER_MIPI_RAW_14BPP_GRBG,
    CAM_FORMAT_BAYER_MIPI_RAW_14BPP_RGGB,
    CAM_FORMAT_BAYER_MIPI_RAW_14BPP_BGGR,
    CAM_FORMAT_BAYER_IDEAL_RAW_QCOM_14BPP_GBRG,
    CAM_FORMAT_BAYER_IDEAL_RAW_QCOM_14BPP_GRBG,
    CAM_FORMAT_BAYER_IDEAL_RAW_QCOM_14BPP_RGGB,
    CAM_FORMAT_BAYER_IDEAL_RAW_QCOM_14BPP_BGGR,
    CAM_FORMAT_BAYER_IDEAL_RAW_MIPI_14BPP_GBRG,
    CAM_FORMAT_BAYER_IDEAL_RAW_MIPI_14BPP_GRBG,
    CAM_FORMAT_BAYER_IDEAL_RAW_MIPI_14BPP_RGGB,
    CAM_FORMAT_BAYER_IDEAL_RAW_MIPI_14BPP_BGGR,
    /* 14BPP: 1st byte: P0 [8:0]
     *        2nd byte: P0 [13:9]
     *        3rd byte: P1 [8:0]
     *        4th byte: P1 [13:9]
     */
    CAM_FORMAT_BAYER_IDEAL_RAW_PLAIN16_14BPP_GBRG,
    CAM_FORMAT_BAYER_IDEAL_RAW_PLAIN16_14BPP_GRBG,
    CAM_FORMAT_BAYER_IDEAL_RAW_PLAIN16_14BPP_RGGB,
    CAM_FORMAT_BAYER_IDEAL_RAW_PLAIN16_14BPP_BGGR,

    CAM_FORMAT_YUV_444_NV24,
    CAM_FORMAT_YUV_444_NV42,

    /* Y plane only, used for FD */
    CAM_FORMAT_Y_ONLY,

    /* UBWC format */
    CAM_FORMAT_YUV_420_NV12_UBWC,

    CAM_FORMAT_YUV_420_NV21_VENUS,

    CAM_FORMAT_MAX
} cam_format_t;

typedef enum {
    /* applies to HAL 1 */
    CAM_STREAM_TYPE_DEFAULT,       /* default stream type */
    CAM_STREAM_TYPE_PREVIEW,       /* preview */
    CAM_STREAM_TYPE_POSTVIEW,      /* postview */
    CAM_STREAM_TYPE_SNAPSHOT,      /* snapshot */
    CAM_STREAM_TYPE_VIDEO,         /* video */

    /* applies to HAL 3 */
    CAM_STREAM_TYPE_IMPL_DEFINED, /* opaque format: could be display, video enc, ZSL YUV */

    /* applies to both HAL 1 and HAL 3 */
    CAM_STREAM_TYPE_METADATA,      /* meta data */
    CAM_STREAM_TYPE_RAW,           /* raw dump from camif */
    CAM_STREAM_TYPE_OFFLINE_PROC,  /* offline process */
    CAM_STREAM_TYPE_PARM,         /* mct internal stream */
    CAM_STREAM_TYPE_ANALYSIS,     /* analysis stream */
    CAM_STREAM_TYPE_CALLBACK,      /* app requested callback */
    CAM_STREAM_TYPE_MAX,
} cam_stream_type_t;

typedef enum {
    CAM_PAD_NONE = 1,
    CAM_PAD_TO_2 = 2,
    CAM_PAD_TO_4 = 4,
    CAM_PAD_TO_WORD = CAM_PAD_TO_4,
    CAM_PAD_TO_8 = 8,
    CAM_PAD_TO_16 = 16,
    CAM_PAD_TO_32 = 32,
    CAM_PAD_TO_64 = 64,
    CAM_PAD_TO_1K = 1024,
    CAM_PAD_TO_2K = 2048,
    CAM_PAD_TO_4K = 4096,
    CAM_PAD_TO_8K = 8192
} cam_pad_format_t;

typedef enum {
    /* followings are per camera */
    CAM_MAPPING_BUF_TYPE_CAPABILITY,  /* mapping camera capability buffer */
    CAM_MAPPING_BUF_TYPE_PARM_BUF,    /* mapping parameters buffer */
    /* this buffer is needed for the payload to be sent with bundling related cameras cmd */
    CAM_MAPPING_BUF_TYPE_SYNC_RELATED_SENSORS_BUF, /* mapping sync buffer.*/

    /* followings are per stream */
    CAM_MAPPING_BUF_TYPE_STREAM_BUF,        /* mapping stream buffers */
    CAM_MAPPING_BUF_TYPE_STREAM_INFO,       /* mapping stream information buffer */
    CAM_MAPPING_BUF_TYPE_OFFLINE_INPUT_BUF, /* mapping offline process input buffer */
    CAM_MAPPING_BUF_TYPE_OFFLINE_META_BUF,  /* mapping offline meta buffer */
    CAM_MAPPING_BUF_TYPE_MISC_BUF,          /* mapping offline miscellaneous buffer */
    CAM_MAPPING_BUF_TYPE_STREAM_USER_BUF,   /* mapping user ptr stream buffers */
    CAM_MAPPING_BUF_TYPE_MAX
} cam_mapping_buf_type;

typedef enum {
    CAM_STREAM_BUF_TYPE_MPLANE,  /* Multiplanar Buffer type */
    CAM_STREAM_BUF_TYPE_USERPTR, /* User specific structure pointer*/
    CAM_STREAM_BUF_TYPE_MAX
} cam_stream_buf_type;

/* values that persist.camera.global.debug can be set to */
/* all camera modules need to map their internal debug levels to this range */
typedef enum {
    CAM_GLBL_DBG_NONE  = 0,
    CAM_GLBL_DBG_ERR   = 1,
    CAM_GLBL_DBG_HIGH  = 2,
    CAM_GLBL_DBG_WARN  = 3,
    CAM_GLBL_DBG_LOW   = 4,
    CAM_GLBL_DBG_DEBUG = 5,
    CAM_GLBL_DBG_INFO  = 6
} cam_global_debug_level_t;

typedef struct {
    cam_mapping_buf_type type;
    uint32_t stream_id;   /* stream id: valid if STREAM_BUF */
    uint32_t frame_idx;   /* frame index: valid if type is STREAM_BUF */
    int32_t plane_idx;    /* planner index. valid if type is STREAM_BUF.
                           * -1 means all planners shanre the same fd;
                           * otherwise, each planner has its own fd */
    uint32_t cookie;      /* could be job_id(uint32_t) to identify mapping job */
    int32_t fd;           /* origin fd */
    size_t size;          /* size of the buffer */
} cam_buf_map_type;

typedef struct {
    uint32_t length;
    cam_buf_map_type buf_maps[CAM_MAX_NUM_BUFS_PER_STREAM];
} cam_buf_map_type_list;

typedef struct {
    cam_mapping_buf_type type;
    uint32_t stream_id;   /* stream id: valid if STREAM_BUF */
    uint32_t frame_idx;   /* frame index: valid if STREAM_BUF or HIST_BUF */
    int32_t plane_idx;    /* planner index. valid if type is STREAM_BUF.
                           * -1 means all planners shanre the same fd;
                           * otherwise, each planner has its own fd */
    uint32_t cookie;      /* could be job_id(uint32_t) to identify unmapping job */
} cam_buf_unmap_type;

typedef struct {
    uint32_t length;
    cam_buf_unmap_type buf_unmaps[CAM_MAX_NUM_BUFS_PER_STREAM];
} cam_buf_unmap_type_list;

typedef enum {
    CAM_MAPPING_TYPE_FD_MAPPING,
    CAM_MAPPING_TYPE_FD_UNMAPPING,
    CAM_MAPPING_TYPE_FD_BUNDLED_MAPPING,
    CAM_MAPPING_TYPE_FD_BUNDLED_UNMAPPING,
    CAM_MAPPING_TYPE_MAX
} cam_mapping_type;

typedef struct {
    cam_mapping_type msg_type;
    union {
        cam_buf_map_type buf_map;
        cam_buf_unmap_type buf_unmap;
        cam_buf_map_type_list buf_map_list;
        cam_buf_unmap_type_list buf_unmap_list;
    } payload;
} cam_sock_packet_t;

typedef enum {
    CAM_MODE_2D = (1<<0),
    CAM_MODE_3D = (1<<1)
} cam_mode_t;

typedef struct {
    uint32_t len;
    uint32_t y_offset;
    uint32_t cbcr_offset;
} cam_sp_len_offset_t;

typedef struct{
    uint32_t len;
    uint32_t offset;
    int32_t offset_x;
    int32_t offset_y;
    int32_t stride;
    int32_t stride_in_bytes;
    int32_t scanline;
    int32_t width;    /* width without padding */
    int32_t height;   /* height without padding */
    int32_t meta_stride;   /*Meta stride*/
    int32_t meta_scanline; /*Meta Scanline*/
    int32_t meta_len;   /*Meta plane length including 4k padding*/
} cam_mp_len_offset_t;

typedef struct {
    uint32_t width_padding;
    uint32_t height_padding;
    uint32_t plane_padding;
    uint32_t min_stride;
    uint32_t min_scanline;
} cam_padding_info_t;

typedef struct {
    uint32_t num_planes;    /*Number of planes in planar buffer*/
    union {
        cam_sp_len_offset_t sp;
        cam_mp_len_offset_t mp[VIDEO_MAX_PLANES];
    };
    uint32_t frame_len;
} cam_frame_len_offset_t;

typedef struct {
    uint8_t frame_buf_cnt;  /*Total plane frames present in 1 batch*/
    uint32_t size;          /*Size of 1 batch buffer. Kernel structure size*/
    long frameInterval;     /*frame interval between each frame*/
} cam_stream_user_buf_info_t;

typedef struct {
    int32_t width;
    int32_t height;
} cam_dimension_t;

typedef struct {
    cam_frame_len_offset_t plane_info;
} cam_stream_buf_plane_info_t;

typedef struct {
    float min_fps;
    float max_fps;
    float video_min_fps;
    float video_max_fps;
} cam_fps_range_t;

typedef struct {
    int32_t min_sensitivity;
    int32_t max_sensitivity;
} cam_sensitivity_range_t;

typedef enum {
    CAM_HFR_MODE_OFF,
    CAM_HFR_MODE_60FPS,
    CAM_HFR_MODE_90FPS,
    CAM_HFR_MODE_120FPS,
    CAM_HFR_MODE_150FPS,
    CAM_HFR_MODE_180FPS,
    CAM_HFR_MODE_210FPS,
    CAM_HFR_MODE_240FPS,
    CAM_HFR_MODE_480FPS,
    CAM_HFR_MODE_MAX
} cam_hfr_mode_t;

typedef struct {
    cam_hfr_mode_t mode;
    cam_dimension_t dim;
    uint8_t frame_skip;
    uint8_t livesnapshot_sizes_tbl_cnt;                     /* livesnapshot sizes table size */
    cam_dimension_t livesnapshot_sizes_tbl[MAX_SIZES_CNT];  /* livesnapshot sizes table */
} cam_hfr_info_t;

typedef enum {
    CAM_WB_MODE_AUTO,
    CAM_WB_MODE_CUSTOM,
    CAM_WB_MODE_INCANDESCENT,
    CAM_WB_MODE_FLUORESCENT,
    CAM_WB_MODE_WARM_FLUORESCENT,
    CAM_WB_MODE_DAYLIGHT,
    CAM_WB_MODE_CLOUDY_DAYLIGHT,
    CAM_WB_MODE_TWILIGHT,
    CAM_WB_MODE_SHADE,
    CAM_WB_MODE_MANUAL,
    CAM_WB_MODE_OFF,
    CAM_WB_MODE_MAX
} cam_wb_mode_type;

typedef enum {
    CAM_ANTIBANDING_MODE_OFF,
    CAM_ANTIBANDING_MODE_60HZ,
    CAM_ANTIBANDING_MODE_50HZ,
    CAM_ANTIBANDING_MODE_AUTO,
    CAM_ANTIBANDING_MODE_AUTO_50HZ,
    CAM_ANTIBANDING_MODE_AUTO_60HZ,
    CAM_ANTIBANDING_MODE_MAX,
} cam_antibanding_mode_type;

/* Enum Type for different ISO Mode supported */
typedef enum {
    CAM_ISO_MODE_AUTO,
    CAM_ISO_MODE_DEBLUR,
    CAM_ISO_MODE_100,
    CAM_ISO_MODE_200,
    CAM_ISO_MODE_400,
    CAM_ISO_MODE_800,
    CAM_ISO_MODE_1600,
    CAM_ISO_MODE_3200,
    CAM_ISO_MODE_MAX
} cam_iso_mode_type;

typedef enum {
    CAM_AEC_MODE_FRAME_AVERAGE,
    CAM_AEC_MODE_CENTER_WEIGHTED,
    CAM_AEC_MODE_SPOT_METERING,
    CAM_AEC_MODE_SMART_METERING,
    CAM_AEC_MODE_USER_METERING,
    CAM_AEC_MODE_SPOT_METERING_ADV,
    CAM_AEC_MODE_CENTER_WEIGHTED_ADV,
    CAM_AEC_MODE_MAX
} cam_auto_exposure_mode_type;

typedef enum {
    CAM_AE_MODE_OFF,
    CAM_AE_MODE_ON,
    CAM_AE_MODE_MAX
} cam_ae_mode_type;

typedef enum {
    CAM_FOCUS_ALGO_AUTO,
    CAM_FOCUS_ALGO_SPOT,
    CAM_FOCUS_ALGO_CENTER_WEIGHTED,
    CAM_FOCUS_ALGO_AVERAGE,
    CAM_FOCUS_ALGO_MAX
} cam_focus_algorithm_type;

/* Auto focus mode */
typedef enum {
    CAM_FOCUS_MODE_OFF,
    CAM_FOCUS_MODE_AUTO,
    CAM_FOCUS_MODE_INFINITY,
    CAM_FOCUS_MODE_MACRO,
    CAM_FOCUS_MODE_FIXED,
    CAM_FOCUS_MODE_EDOF,
    CAM_FOCUS_MODE_CONTINOUS_VIDEO,
    CAM_FOCUS_MODE_CONTINOUS_PICTURE,
    CAM_FOCUS_MODE_MANUAL,
    CAM_FOCUS_MODE_MAX
} cam_focus_mode_type;

typedef enum {
    CAM_MANUAL_FOCUS_MODE_INDEX,
    CAM_MANUAL_FOCUS_MODE_DAC_CODE,
    CAM_MANUAL_FOCUS_MODE_RATIO,
    CAM_MANUAL_FOCUS_MODE_DIOPTER,
    CAM_MANUAL_FOCUS_MODE_MAX
} cam_manual_focus_mode_type;

typedef struct {
    cam_manual_focus_mode_type flag;
    union{
        int32_t af_manual_lens_position_index;
        int32_t af_manual_lens_position_dac;
        int32_t af_manual_lens_position_ratio;
        float af_manual_diopter;
    };
} cam_manual_focus_parm_t;

typedef enum {
    CAM_MANUAL_WB_MODE_CCT,
    CAM_MANUAL_WB_MODE_GAIN,
    CAM_MANUAL_WB_MODE_MAX
} cam_manual_wb_mode_type;

typedef struct {
    float r_gain;
    float g_gain;
    float b_gain;
} cam_awb_gain_t;

typedef struct {
    cam_manual_wb_mode_type type;
    union{
        int32_t cct;
        cam_awb_gain_t gains;
    };
} cam_manual_wb_parm_t;

typedef enum {
    CAM_SCENE_MODE_OFF,
    CAM_SCENE_MODE_AUTO,
    CAM_SCENE_MODE_LANDSCAPE,
    CAM_SCENE_MODE_SNOW,
    CAM_SCENE_MODE_BEACH,
    CAM_SCENE_MODE_SUNSET,
    CAM_SCENE_MODE_NIGHT,
    CAM_SCENE_MODE_PORTRAIT,
    CAM_SCENE_MODE_BACKLIGHT,
    CAM_SCENE_MODE_SPORTS,
    CAM_SCENE_MODE_ANTISHAKE,
    CAM_SCENE_MODE_FLOWERS,
    CAM_SCENE_MODE_CANDLELIGHT,
    CAM_SCENE_MODE_FIREWORKS,
    CAM_SCENE_MODE_PARTY,
    CAM_SCENE_MODE_NIGHT_PORTRAIT,
    CAM_SCENE_MODE_THEATRE,
    CAM_SCENE_MODE_ACTION,
    CAM_SCENE_MODE_AR,
    CAM_SCENE_MODE_FACE_PRIORITY,
    CAM_SCENE_MODE_BARCODE,
    CAM_SCENE_MODE_HDR,
    CAM_SCENE_MODE_MAX
} cam_scene_mode_type;

typedef enum {
    CAM_EFFECT_MODE_OFF,
    CAM_EFFECT_MODE_MONO,
    CAM_EFFECT_MODE_NEGATIVE,
    CAM_EFFECT_MODE_SOLARIZE,
    CAM_EFFECT_MODE_SEPIA,
    CAM_EFFECT_MODE_POSTERIZE,
    CAM_EFFECT_MODE_WHITEBOARD,
    CAM_EFFECT_MODE_BLACKBOARD,
    CAM_EFFECT_MODE_AQUA,
    CAM_EFFECT_MODE_EMBOSS,
    CAM_EFFECT_MODE_SKETCH,
    CAM_EFFECT_MODE_NEON,
    CAM_EFFECT_MODE_MAX
} cam_effect_mode_type;

typedef enum {
    CAM_FLASH_MODE_OFF,
    CAM_FLASH_MODE_AUTO,
    CAM_FLASH_MODE_ON,
    CAM_FLASH_MODE_TORCH,
    CAM_FLASH_MODE_SINGLE,
    CAM_FLASH_MODE_MAX
} cam_flash_mode_t;

// Flash States
typedef enum {
    CAM_FLASH_STATE_UNAVAILABLE,
    CAM_FLASH_STATE_CHARGING,
    CAM_FLASH_STATE_READY,
    CAM_FLASH_STATE_FIRED,
    CAM_FLASH_STATE_PARTIAL,
    CAM_FLASH_STATE_MAX
} cam_flash_state_t;

typedef enum {
    CAM_FLASH_FIRING_LEVEL_0,
    CAM_FLASH_FIRING_LEVEL_1,
    CAM_FLASH_FIRING_LEVEL_2,
    CAM_FLASH_FIRING_LEVEL_3,
    CAM_FLASH_FIRING_LEVEL_4,
    CAM_FLASH_FIRING_LEVEL_5,
    CAM_FLASH_FIRING_LEVEL_6,
    CAM_FLASH_FIRING_LEVEL_7,
    CAM_FLASH_FIRING_LEVEL_8,
    CAM_FLASH_FIRING_LEVEL_9,
    CAM_FLASH_FIRING_LEVEL_10,
    CAM_FLASH_FIRING_LEVEL_MAX
} cam_flash_firing_level_t;


typedef enum {
    CAM_AEC_TRIGGER_IDLE,
    CAM_AEC_TRIGGER_START
} cam_aec_trigger_type_t;

typedef enum {
    CAM_AF_TRIGGER_IDLE,
    CAM_AF_TRIGGER_START,
    CAM_AF_TRIGGER_CANCEL
} cam_af_trigger_type_t;

typedef enum {
    CAM_AE_STATE_INACTIVE,
    CAM_AE_STATE_SEARCHING,
    CAM_AE_STATE_CONVERGED,
    CAM_AE_STATE_LOCKED,
    CAM_AE_STATE_FLASH_REQUIRED,
    CAM_AE_STATE_PRECAPTURE
} cam_ae_state_t;

typedef enum {
    CAM_NOISE_REDUCTION_MODE_OFF,
    CAM_NOISE_REDUCTION_MODE_FAST,
    CAM_NOISE_REDUCTION_MODE_HIGH_QUALITY
} cam_noise_reduction_mode_t;

typedef enum {
    CAM_EDGE_MODE_OFF,
    CAM_EDGE_MODE_FAST,
    CAM_EDGE_MODE_HIGH_QUALITY,
} cam_edge_mode_t;

typedef struct {
   uint8_t edge_mode;
   int32_t sharpness;
} cam_edge_application_t;

typedef enum {
    CAM_BLACK_LEVEL_LOCK_OFF,
    CAM_BLACK_LEVEL_LOCK_ON,
} cam_black_level_lock_t;

typedef enum {
    CAM_LENS_SHADING_MAP_MODE_OFF,
    CAM_LENS_SHADING_MAP_MODE_ON,
} cam_lens_shading_map_mode_t;

typedef enum {
    CAM_LENS_SHADING_MODE_OFF,
    CAM_LENS_SHADING_MODE_FAST,
    CAM_LENS_SHADING_MODE_HIGH_QUALITY,
} cam_lens_shading_mode_t;

typedef enum {
    CAM_FACE_DETECT_MODE_OFF,
    CAM_FACE_DETECT_MODE_SIMPLE,
    CAM_FACE_DETECT_MODE_FULL,
} cam_face_detect_mode_t;

typedef enum {
    CAM_TONEMAP_MODE_CONTRAST_CURVE,
    CAM_TONEMAP_MODE_FAST,
    CAM_TONEMAP_MODE_HIGH_QUALITY,
} cam_tonemap_mode_t;

typedef enum {
  CAM_CDS_MODE_OFF,
  CAM_CDS_MODE_ON,
  CAM_CDS_MODE_AUTO,
  CAM_CDS_MODE_MAX
} cam_cds_mode_type_t;

typedef struct  {
    int32_t left;
    int32_t top;
    int32_t width;
    int32_t height;
} cam_rect_t;

typedef struct  {
    cam_rect_t rect;
    int32_t weight; /* weight of the area, valid for focusing/metering areas */
} cam_area_t;

typedef enum {
    CAM_STREAMING_MODE_CONTINUOUS, /* continous streaming */
    CAM_STREAMING_MODE_BURST,      /* burst streaming */
    CAM_STREAMING_MODE_BATCH,      /* stream frames in batches */
    CAM_STREAMING_MODE_MAX
} cam_streaming_mode_t;

typedef enum {
    IS_TYPE_NONE,
    IS_TYPE_DIS,
    IS_TYPE_GA_DIS,
    IS_TYPE_EIS_1_0,
    IS_TYPE_EIS_2_0,
    IS_TYPE_MAX
} cam_is_type_t;

typedef enum {
    DIS_DISABLE,
    DIS_ENABLE
} cam_dis_mode_t;

typedef enum {
  NON_SECURE,
  SECURE
} cam_stream_secure_t;

#define CAM_REPROCESS_MASK_TYPE_WNR (1<<0)

/* event from server */
typedef enum {
    CAM_EVENT_TYPE_MAP_UNMAP_DONE  = (1<<0),
    CAM_EVENT_TYPE_AUTO_FOCUS_DONE = (1<<1),
    CAM_EVENT_TYPE_ZOOM_DONE       = (1<<2),
    CAM_EVENT_TYPE_DAEMON_DIED     = (1<<3),
    CAM_EVENT_TYPE_INT_TAKE_JPEG   = (1<<4),
    CAM_EVENT_TYPE_INT_TAKE_RAW    = (1<<5),
    CAM_EVENT_TYPE_DAEMON_PULL_REQ = (1<<6),
    CAM_EVENT_TYPE_CAC_DONE        = (1<<7),
    CAM_EVENT_TYPE_MAX
} cam_event_type_t;

typedef enum {
    CAM_EXP_BRACKETING_OFF,
    CAM_EXP_BRACKETING_ON
} cam_bracket_mode;

typedef struct {
    cam_bracket_mode mode;
    char values[MAX_EXP_BRACKETING_LENGTH];  /* user defined values */
} cam_exp_bracketing_t;

typedef struct {
  uint32_t num_frames;
  cam_exp_bracketing_t exp_val;
} cam_hdr_bracketing_info_t;

 typedef struct {
    cam_bracket_mode mode;
    int32_t values;  /* user defined values */
} cam_capture_bracketing_t;

typedef enum {
    CAM_CAPTURE_NORMAL,
    CAM_CAPTURE_FLASH,
    CAM_CAPTURE_BRACKETING,
    CAM_CAPTURE_MAX
} cam_capture_type;

typedef struct {
    int32_t num_frames;     /*Num of frames requested on this quality*/
    cam_capture_type type;  /*type of the capture request*/

    /*union to strore values of capture type*/
    union {
        cam_flash_mode_t flash_mode;
        cam_capture_bracketing_t hdr_mode;
    };
} cam_capture_settings_t;

typedef struct {
    uint32_t num_batch;  /*Number of frames batch requested*/
    cam_capture_settings_t configs[MAX_CAPTURE_BATCH_NUM];
} cam_capture_frame_config_t;

typedef struct {
    uint8_t stepsize;
    uint8_t direction;
    int32_t num_steps;
    uint8_t ttype;
} tune_actuator_t;

typedef struct {
    uint8_t module;
    uint8_t type;
    int32_t value;
} tune_cmd_t;

typedef enum {
    CAM_AEC_ROI_OFF,
    CAM_AEC_ROI_ON
} cam_aec_roi_ctrl_t;

typedef enum {
    CAM_AEC_ROI_BY_INDEX,
    CAM_AEC_ROI_BY_COORDINATE,
} cam_aec_roi_type_t;

typedef struct {
    uint32_t x;
    uint32_t y;
} cam_coordinate_type_t;

typedef struct {
    int32_t numerator;
    int32_t denominator;
} cam_rational_type_t;

typedef struct {
    cam_aec_roi_ctrl_t aec_roi_enable;
    cam_aec_roi_type_t aec_roi_type;
    union {
        cam_coordinate_type_t coordinate[MAX_ROI];
        uint32_t aec_roi_idx[MAX_ROI];
    } cam_aec_roi_position;
} cam_set_aec_roi_t;

typedef struct {
    uint32_t frm_id;
    uint8_t num_roi;
    cam_rect_t roi[MAX_ROI];
    int32_t weight[MAX_ROI];
    uint8_t is_multiwindow;
} cam_roi_info_t;

typedef enum {
    CAM_WAVELET_DENOISE_YCBCR_PLANE,
    CAM_WAVELET_DENOISE_CBCR_ONLY,
    CAM_WAVELET_DENOISE_STREAMLINE_YCBCR,
    CAM_WAVELET_DENOISE_STREAMLINED_CBCR
} cam_denoise_process_type_t;

typedef struct {
    uint8_t denoise_enable;
    cam_denoise_process_type_t process_plates;
} cam_denoise_param_t;

#define CAM_FACE_PROCESS_MASK_DETECTION    (1U<<0)
#define CAM_FACE_PROCESS_MASK_RECOGNITION  (1U<<1)
typedef struct {
    uint32_t fd_mode;          /* mask of face process */
    uint32_t num_fd;
} cam_fd_set_parm_t;

typedef enum {
    QCAMERA_FD_PREVIEW,
    QCAMERA_FD_SNAPSHOT
}qcamera_face_detect_type_t;

typedef enum {
    CAM_FACE_CT_POINT_EYE_L_PUPIL,
    CAM_FACE_CT_POINT_EYE_L_IN,
    CAM_FACE_CT_POINT_EYE_L_OUT,
    CAM_FACE_CT_POINT_EYE_L_UP,
    CAM_FACE_CT_POINT_EYE_L_DOWN,
    CAM_FACE_CT_POINT_EYE_R_PUPIL,
    CAM_FACE_CT_POINT_EYE_R_IN,
    CAM_FACE_CT_POINT_EYE_R_OUT,
    CAM_FACE_CT_POINT_EYE_R_UP,
    CAM_FACE_CT_POINT_EYE_R_DOWN,
    CAM_FACE_CT_POINT_EYE_MAX
} cam_face_ct_point_eye_t;

typedef enum {
    CAM_FACE_CT_POINT_FOREHEAD,
    CAM_FACE_CT_POINT_FOREHEAD_MAX
} cam_face_ct_point_forh_t;

typedef enum {
    CAM_FACE_CT_POINT_NOSE,
    CAM_FACE_CT_POINT_NOSE_TIP,
    CAM_FACE_CT_POINT_NOSE_L,
    CAM_FACE_CT_POINT_NOSE_R,
    CAM_FACE_CT_POINT_NOSE_L_0,
    CAM_FACE_CT_POINT_NOSE_R_0,
    CAM_FACE_CT_POINT_NOSE_L_1,
    CAM_FACE_CT_POINT_NOSE_R_1,
    CAM_FACE_CT_POINT_NOSE_MAX
} cam_face_ct_point_nose_t;

typedef enum {
    CAM_FACE_CT_POINT_MOUTH_L,
    CAM_FACE_CT_POINT_MOUTH_R,
    CAM_FACE_CT_POINT_MOUTH_UP,
    CAM_FACE_CT_POINT_MOUTH_DOWN,
    CAM_FACE_CT_POINT_MOUTH_MAX
} cam_face_ct_point_mouth_t;

typedef enum {
    CAM_FACE_CT_POINT_LIP_UP,
    CAM_FACE_CT_POINT_LIP_DOWN,
    CAM_FACE_CT_POINT_LIP_MAX
} cam_face_ct_point_lip_t;

typedef enum {
    CAM_FACE_CT_POINT_BROW_L_UP,
    CAM_FACE_CT_POINT_BROW_L_DOWN,
    CAM_FACE_CT_POINT_BROW_L_IN,
    CAM_FACE_CT_POINT_BROW_L_OUT,
    CAM_FACE_CT_POINT_BROW_R_UP,
    CAM_FACE_CT_POINT_BROW_R_DOWN,
    CAM_FACE_CT_POINT_BROW_R_IN,
    CAM_FACE_CT_POINT_BROW_R_OUT,
    CAM_FACE_CT_POINT_BROW_MAX
} cam_face_ct_point_brow_t;

typedef enum {
    CAM_FACE_CT_POINT_CHIN,
    CAM_FACE_CT_POINT_CHIN_L,
    CAM_FACE_CT_POINT_CHIN_R,
    CAM_FACE_CT_POINT_CHIN_MAX
} cam_face_ct_point_chin_t;

typedef enum {
    CAM_FACE_CT_POINT_EAR_L_DOWN,
    CAM_FACE_CT_POINT_EAR_R_DOWN,
    CAM_FACE_CT_POINT_EAR_L_UP,
    CAM_FACE_CT_POINT_EAR_R_UP,
    CAM_FACE_CT_POINT_EAR_MAX
} cam_face_ct_point_ear_t;

typedef struct {
  uint8_t is_eye_valid;
  cam_coordinate_type_t contour_eye_pt[CAM_FACE_CT_POINT_EYE_MAX];
  uint8_t is_forehead_valid;
  cam_coordinate_type_t contour_forh_pt[CAM_FACE_CT_POINT_FOREHEAD_MAX];
  uint8_t is_nose_valid;
  cam_coordinate_type_t contour_nose_pt[CAM_FACE_CT_POINT_NOSE_MAX];
  uint8_t is_mouth_valid;
  cam_coordinate_type_t contour_mouth_pt[CAM_FACE_CT_POINT_MOUTH_MAX];
  uint8_t is_lip_valid;
  cam_coordinate_type_t contour_lip_pt[CAM_FACE_CT_POINT_LIP_MAX];
  uint8_t is_brow_valid;
  cam_coordinate_type_t contour_brow_pt[CAM_FACE_CT_POINT_BROW_MAX];
  uint8_t is_chin_valid;
  cam_coordinate_type_t contour_chin_pt[CAM_FACE_CT_POINT_CHIN_MAX];
  uint8_t is_ear_valid;
  cam_coordinate_type_t contour_ear_pt[CAM_FACE_CT_POINT_EAR_MAX];
} cam_face_detect_contour_t;

typedef struct {
    int32_t face_id;            /* unique id for face tracking within view unless view changes */
    int8_t score;              /* score of confidence (0, -100) */
    cam_rect_t face_boundary;  /* boundary of face detected */
    cam_coordinate_type_t left_eye_center;  /* coordinate of center of left eye */
    cam_coordinate_type_t right_eye_center; /* coordinate of center of right eye */
    cam_coordinate_type_t mouth_center;     /* coordinate of center of mouth */
    cam_face_detect_contour_t contour_info; /* face detection contour info */
    uint8_t smile_degree;      /* smile degree (0, -100) */
    uint8_t smile_confidence;  /* smile confidence (0, 100) */
    uint8_t face_recognised;   /* if face is recognised */
    int8_t gaze_angle;         /* -90 -45 0 45 90 for head left to rigth tilt */
    int32_t updown_dir;        /* up down direction (-180, 179) */
    int32_t leftright_dir;     /* left right direction (-180, 179) */
    int32_t roll_dir;          /* roll direction (-180, 179) */
    int8_t left_right_gaze;    /* left right gaze degree (-50, 50) */
    int8_t top_bottom_gaze;    /* up down gaze degree (-50, 50) */
    uint8_t blink_detected;    /* if blink is detected */
    uint8_t left_blink;        /* left eye blink degeree (0, -100) */
    uint8_t right_blink;       /* right eye blink degree (0, - 100) */
} cam_face_detection_info_t;

typedef struct {
    uint32_t frame_id;                         /* frame index of which faces are detected */
    uint8_t num_faces_detected;                /* number of faces detected */
    cam_face_detection_info_t faces[MAX_ROI];  /* detailed information of faces detected */
    qcamera_face_detect_type_t fd_type;        /* face detect for preview or snapshot frame*/
    cam_dimension_t fd_frame_dim;              /* frame dims on which fd is applied */
    uint8_t update_flag;                       /* flag to inform whether HAL needs to send cb
                                                * to app or not */
} cam_face_detection_data_t;

#define CAM_HISTOGRAM_STATS_SIZE 256
typedef struct {
    uint32_t max_hist_value;
    uint32_t hist_buf[CAM_HISTOGRAM_STATS_SIZE]; /* buf holding histogram stats data */
} cam_histogram_data_t;

typedef struct {
    cam_histogram_data_t r_stats;
    cam_histogram_data_t b_stats;
    cam_histogram_data_t gr_stats;
    cam_histogram_data_t gb_stats;
} cam_bayer_hist_stats_t;

typedef enum {
    CAM_HISTOGRAM_TYPE_BAYER,
    CAM_HISTOGRAM_TYPE_YUV
} cam_histogram_type_t;

typedef struct {
    cam_histogram_type_t type;
    union {
        cam_bayer_hist_stats_t bayer_stats;
        cam_histogram_data_t yuv_stats;
    };
} cam_hist_stats_t;

enum cam_focus_distance_index{
  CAM_FOCUS_DISTANCE_NEAR_INDEX,  /* 0 */
  CAM_FOCUS_DISTANCE_OPTIMAL_INDEX,
  CAM_FOCUS_DISTANCE_FAR_INDEX,
  CAM_FOCUS_DISTANCE_MAX_INDEX
};

typedef struct {
  float focus_distance[CAM_FOCUS_DISTANCE_MAX_INDEX];
} cam_focus_distances_info_t;

typedef struct {
    uint32_t scale;
    float diopter;
} cam_focus_pos_info_t ;

typedef struct {
    float focalLengthRatio;
} cam_focal_length_ratio_t;

/* Different autofocus cycle when calling do_autoFocus
 * CAM_AF_COMPLETE_EXISTING_SWEEP: Complete existing sweep
 * if one is ongoing, and lock.
 * CAM_AF_DO_ONE_FULL_SWEEP: Do one full sweep, regardless
 * of the current state, and lock.
 * CAM_AF_START_CONTINUOUS_SWEEP: Start continous sweep.
 * After do_autoFocus, HAL receives an event: CAM_AF_FOCUSED,
 * or CAM_AF_NOT_FOCUSED.
 * cancel_autoFocus stops any lens movement.
 * Each do_autoFocus call only produces 1 FOCUSED/NOT_FOCUSED
 * event, not both.
 */
typedef enum {
    CAM_AF_COMPLETE_EXISTING_SWEEP,
    CAM_AF_DO_ONE_FULL_SWEEP,
    CAM_AF_START_CONTINUOUS_SWEEP
} cam_autofocus_cycle_t;

typedef enum {
    CAM_AF_SCANNING,
    CAM_AF_FOCUSED,
    CAM_AF_NOT_FOCUSED,
    CAM_AF_INACTIVE
} cam_autofocus_state_t;

//Don't change the order of the AF states below. It should match
//with the corresponding enum in frameworks (camera3.h and
//CameraMetadata.java)
typedef enum {
    CAM_AF_STATE_INACTIVE,
    CAM_AF_STATE_PASSIVE_SCAN,
    CAM_AF_STATE_PASSIVE_FOCUSED,
    CAM_AF_STATE_ACTIVE_SCAN,
    CAM_AF_STATE_FOCUSED_LOCKED,
    CAM_AF_STATE_NOT_FOCUSED_LOCKED,
    CAM_AF_STATE_PASSIVE_UNFOCUSED
} cam_af_state_t;

typedef struct {
    cam_af_state_t focus_state;           /* state of focus */
    cam_focus_distances_info_t focus_dist;       /* focus distance */
    cam_focus_mode_type focus_mode;        /* focus mode from backend */
    uint32_t focused_frame_idx;
    int32_t focus_pos;
} cam_auto_focus_data_t;

typedef struct {
  uint32_t is_hdr_scene;
  float    hdr_confidence;
} cam_asd_hdr_scene_data_t;

typedef struct {
    uint32_t stream_id;
    cam_rect_t crop;
    cam_rect_t roi_map;
} cam_stream_crop_info_t;

typedef struct {
    uint8_t num_of_streams;
    cam_stream_crop_info_t crop_info[MAX_NUM_STREAMS];
} cam_crop_data_t;

typedef enum {
    DO_NOT_NEED_FUTURE_FRAME,
    NEED_FUTURE_FRAME,
} cam_prep_snapshot_state_t;

typedef enum {
    CC_RED_GAIN,
    CC_GREEN_RED_GAIN,
    CC_GREEN_BLUE_GAIN,
    CC_BLUE_GAIN,
    CC_GAIN_MAX
} cam_cc_gains_type_t;

typedef struct {
    float gains[CC_GAIN_MAX];
} cam_color_correct_gains_t;

typedef struct {
    // If LED is ON and Burst Num > 1, this is first LED ON frame
    uint32_t min_frame_idx;
    // If LED is ON and Burst Num > 1, this is first LED Off frame after ON
    uint32_t max_frame_idx;
    // Used only when LED Is ON and burst num > 1
    uint32_t num_led_on_frames;
    // Skip count after LED is turned OFF
    uint32_t frame_skip_count;
    // Batch id for each picture request
    uint32_t config_batch_idx;
} cam_frame_idx_range_t;

typedef enum {
  S_NORMAL = 0,
  S_SCENERY,
  S_PORTRAIT,
  S_PORTRAIT_BACKLIGHT,
  S_SCENERY_BACKLIGHT,
  S_BACKLIGHT,
  S_MAX,
} cam_auto_scene_t;

typedef struct {
   uint32_t meta_frame_id;
} cam_meta_valid_t;

typedef enum {
    CAM_SENSOR_RAW,
    CAM_SENSOR_YUV,
    CAM_SENSOR_Y,
} cam_sensor_t;

typedef struct {
    cam_flash_mode_t flash_mode;
    float            aperture_value;
    cam_flash_state_t        flash_state;
    float            focal_length;
    float            f_number;
    int32_t          sensing_method;
    float            crop_factor;
    cam_sensor_t sens_type;
} cam_sensor_params_t;

typedef enum {
    CAM_METERING_MODE_UNKNOWN = 0,
    CAM_METERING_MODE_AVERAGE = 1,
    CAM_METERING_MODE_CENTER_WEIGHTED_AVERAGE = 2,
    CAM_METERING_MODE_SPOT = 3,
    CAM_METERING_MODE_MULTI_SPOT = 4,
    CAM_METERING_MODE_PATTERN = 5,
    CAM_METERING_MODE_PARTIAL = 6,
    CAM_METERING_MODE_OTHER = 255,
} cam_metering_mode_t;

typedef struct {
    float exp_time;
    int32_t iso_value;
    uint32_t flash_needed;
    uint32_t settled;
    cam_wb_mode_type wb_mode;
    uint32_t metering_mode;
    uint32_t exposure_program;
    uint32_t exposure_mode;
    uint32_t scenetype;
    float brightness;
} cam_3a_params_t;

typedef struct {
    uint64_t sw_version_number;
    int32_t aec_debug_data_size;
    char aec_private_debug_data[AEC_DEBUG_DATA_SIZE];
} cam_ae_exif_debug_t;

typedef struct {
    int32_t cct_value;
    cam_awb_gain_t rgb_gains;
} cam_awb_params_t;

typedef struct {
    int32_t awb_debug_data_size;
    char awb_private_debug_data[AWB_DEBUG_DATA_SIZE];
} cam_awb_exif_debug_t;

typedef struct {
    int32_t af_debug_data_size;
    int32_t haf_debug_data_size;
    int32_t tof_debug_data_size;
    int32_t dciaf_debug_data_size;
    int32_t pdaf_debug_data_size;
    char af_private_debug_data[AF_DEBUG_DATA_SIZE];
    int32_t af_stats_buffer_size;
    char af_stats_private_debug_data[AF_STATS_DEBUG_DATA_SIZE];
} cam_af_exif_debug_t;

typedef struct {
    int32_t asd_debug_data_size;
    char asd_private_debug_data[ASD_DEBUG_DATA_SIZE];
} cam_asd_exif_debug_t;

typedef struct {
    int32_t bg_stats_buffer_size;
    int32_t bhist_stats_buffer_size;
    int32_t bg_config_buffer_size;
    char stats_buffer_private_debug_data[STATS_BUFFER_DEBUG_DATA_SIZE];
} cam_stats_buffer_exif_debug_t;

/* 3A version*/
typedef struct {
    uint16_t major_version;
    uint16_t minor_version;
    uint16_t patch_version;
    uint16_t new_feature_des;
} cam_q3a_version_t;

typedef struct {
    uint32_t tuning_data_version;
    size_t tuning_sensor_data_size;
    size_t tuning_vfe_data_size;
    size_t tuning_cpp_data_size;
    size_t tuning_cac_data_size;
    size_t tuning_cac_data_size2;
    size_t tuning_mod3_data_size;
    uint8_t  data[TUNING_DATA_MAX];
}tuning_params_t;

typedef struct {
    int32_t event_type;
    cam_dimension_t dim;
    size_t size;
    char path[QCAMERA_MAX_FILEPATH_LENGTH];
    cam_format_t picture_format;
} cam_int_evt_params_t;

typedef struct {
  uint8_t private_isp_data[MAX_ISP_DATA_SIZE];
} cam_chromatix_lite_isp_t;

typedef struct {
  uint8_t private_pp_data[MAX_PP_DATA_SIZE];
} cam_chromatix_lite_pp_t;

typedef struct {
  uint8_t private_stats_data[MAX_AE_STATS_DATA_SIZE];
} cam_chromatix_lite_ae_stats_t;

typedef struct {
  uint8_t private_stats_data[MAX_AWB_STATS_DATA_SIZE];
} cam_chromatix_lite_awb_stats_t;

typedef struct {
  uint8_t private_stats_data[MAX_AF_STATS_DATA_SIZE];
} cam_chromatix_lite_af_stats_t;

typedef struct {
  uint8_t private_stats_data[MAX_ASD_STATS_DATA_SIZE];
} cam_chromatix_lite_asd_stats_t;

typedef struct {
   uint32_t min_buffers;
   uint32_t max_buffers;
} cam_buffer_info_t;

typedef struct {
    cam_dimension_t stream_sizes[MAX_NUM_STREAMS];
    uint32_t num_streams;
    cam_stream_type_t type[MAX_NUM_STREAMS];
    uint32_t postprocess_mask[MAX_NUM_STREAMS];
    cam_buffer_info_t buffer_info;
    cam_is_type_t is_type;
    cam_hfr_mode_t hfr_mode;
    cam_format_t format[MAX_NUM_STREAMS];
    uint32_t buf_alignment;
    uint32_t min_stride;
    uint32_t min_scanline;
    uint8_t batch_size;
} cam_stream_size_info_t;


typedef enum {
    CAM_INTF_OVERWRITE_MINI_CHROMATIX_OFFLINE,
    CAM_INTF_OVERWRITE_ISP_HW_DATA_OFFLINE,
    CAM_INTF_OVERWRITE_MINI_CHROMATIX_ONLINE,
    CAM_INTF_OVERWRITE_ISP_HW_DATA_ONLINE,
    CAM_INTF_OVERWRITE_MAX,
} cam_intf_overwrite_type_t;

typedef struct {
  uint8_t lds_enabled;
  float rnr_sampling_factor;
} cam_img_hysterisis_info_t;

typedef struct {
  cam_intf_overwrite_type_t overwrite_type;
  char isp_hw_data_list[4096];     /*add upper bound memory, customer to fill*/
  char chromatix_data_overwrite[4096]; /*add bound memory, customer fill*/
} cam_hw_data_overwrite_t;

typedef struct {
    uint32_t num_streams;
    uint32_t streamID[MAX_NUM_STREAMS];
} cam_stream_ID_t;

/*CAC Message posted during pipeline*/
typedef struct {
    uint32_t frame_id;
    int32_t buf_idx;
} cam_cac_info_t;

typedef struct
{
  uint32_t id;            /* Frame ID */
  uint64_t timestamp;    /* Time stamp */
  uint32_t distance_in_mm; /* Distance of object in ROI's in milimeters */
  uint32_t confidence;     /* Confidence on distance from 0(No confidence)to 1024(max) */
  uint32_t status;        /* Status of DCRF library execution call */
  cam_rect_t focused_roi; /* ROI's for which distance is estimated */
  uint32_t focused_x;     /* Focus location X inside ROI with distance estimation */
  uint32_t focused_y;     /* Focus location Y inside ROI with distance estimation */
} cam_dcrf_result_t;

typedef struct {
    uint32_t frame_id;
    uint32_t num_streams;
    uint32_t stream_id[MAX_NUM_STREAMS];
} cam_buf_divert_info_t;

typedef  struct {
    uint8_t is_stats_valid;               /* if histgram data is valid */
    cam_hist_stats_t stats_data;          /* histogram data */

    uint8_t is_faces_valid;               /* if face detection data is valid */
    cam_face_detection_data_t faces_data; /* face detection result */

    uint8_t is_focus_valid;               /* if focus data is valid */
    cam_auto_focus_data_t focus_data;     /* focus data */

    uint8_t is_crop_valid;                /* if crop data is valid */
    cam_crop_data_t crop_data;            /* crop data */

    uint8_t is_prep_snapshot_done_valid;  /* if prep snapshot done is valid */
    cam_prep_snapshot_state_t prep_snapshot_done_state;  /* prepare snapshot done state */

    uint8_t is_cac_valid;                 /* if cac info is valid */
    cam_cac_info_t cac_info;              /* cac info */

    /* Hysterisis data from Img modules */
    uint8_t is_hyst_info_valid;           /* if hyst info is valid */
    cam_img_hysterisis_info_t img_hyst_info; /* hyst info */

    /* if good frame idx range is valid */
    uint8_t is_good_frame_idx_range_valid;
    /* good frame idx range, make sure:
     * 1. good_frame_idx_range.min_frame_idx > current_frame_idx
     * 2. good_frame_idx_range.min_frame_idx - current_frame_idx < 100 */
    cam_frame_idx_range_t good_frame_idx_range;

    uint32_t is_hdr_scene_data_valid;
    cam_asd_hdr_scene_data_t hdr_scene_data;
    uint8_t is_asd_decision_valid;
    cam_auto_scene_t scene; //scene type as decided by ASD

    char private_metadata[MAX_METADATA_PRIVATE_PAYLOAD_SIZE_IN_BYTES];

    /* AE parameters */
    uint8_t is_3a_params_valid;
    cam_3a_params_t cam_3a_params;

    /* AE exif debug parameters */
    uint8_t is_ae_exif_debug_valid;
    cam_ae_exif_debug_t ae_exif_debug_params;

    /* AWB exif debug parameters */
    uint8_t is_awb_exif_debug_valid;
    cam_awb_exif_debug_t awb_exif_debug_params;

    /* AF exif debug parameters */
    uint8_t is_af_exif_debug_valid;
    cam_af_exif_debug_t af_exif_debug_params;

    /* ASD exif debug parameters */
    uint8_t is_asd_exif_debug_valid;
    cam_asd_exif_debug_t asd_exif_debug_params;

    /* Stats buffer exif debug parameters */
    uint8_t is_stats_buffer_exif_debug_valid;
    cam_stats_buffer_exif_debug_t stats_buffer_exif_debug_params;

    /* AWB parameters */
    uint8_t is_awb_params_valid;
    cam_awb_params_t awb_params;

    /* sensor parameters */
    uint8_t is_sensor_params_valid;
    cam_sensor_params_t sensor_params;

    /* Meta valid params */
    uint8_t is_meta_valid;
    cam_meta_valid_t meta_valid_params;

    /*Tuning Data*/
    uint8_t is_tuning_params_valid;
    tuning_params_t tuning_params;

    uint8_t is_chromatix_lite_isp_valid;
    cam_chromatix_lite_isp_t chromatix_lite_isp_data;

    uint8_t is_chromatix_lite_pp_valid;
    cam_chromatix_lite_pp_t chromatix_lite_pp_data;

    uint8_t is_chromatix_lite_ae_stats_valid;
    cam_chromatix_lite_ae_stats_t chromatix_lite_ae_stats_data;

    uint8_t is_chromatix_lite_awb_stats_valid;
    cam_chromatix_lite_awb_stats_t chromatix_lite_awb_stats_data;

    uint8_t is_chromatix_lite_af_stats_valid;
    cam_chromatix_lite_af_stats_t chromatix_lite_af_stats_data;

    uint8_t is_dcrf_result_valid;
    cam_dcrf_result_t dcrf_result;
} cam_metadata_info_t;

typedef enum {
    CAM_INTF_PARM_HAL_VERSION = 0x1,

    /* Overall mode of 3A control routines. We need to have this parameter
     * because not all android.control.* have an OFF option, for example,
     * AE_FPS_Range, aePrecaptureTrigger */
    CAM_INTF_META_MODE,
    /* Whether AE is currently updating the sensor exposure and sensitivity
     * fields */
    CAM_INTF_META_AEC_MODE,
    CAM_INTF_PARM_WHITE_BALANCE,
    CAM_INTF_PARM_FOCUS_MODE,

    /* common between HAL1 and HAL3 */
    CAM_INTF_PARM_ANTIBANDING,
    CAM_INTF_PARM_EXPOSURE_COMPENSATION,
    CAM_INTF_PARM_EV_STEP,
    CAM_INTF_PARM_AEC_LOCK,
    CAM_INTF_PARM_FPS_RANGE,
    CAM_INTF_PARM_AWB_LOCK, /* 10 */
    CAM_INTF_PARM_EFFECT,
    CAM_INTF_PARM_BESTSHOT_MODE,
    CAM_INTF_PARM_DIS_ENABLE,
    CAM_INTF_PARM_LED_MODE,
    CAM_INTF_META_HISTOGRAM,
    CAM_INTF_META_FACE_DETECTION,
    /* Whether optical image stabilization is enabled. */
    CAM_INTF_META_LENS_OPT_STAB_MODE,

    /* specific to HAl1 */
    CAM_INTF_META_AUTOFOCUS_DATA,
    CAM_INTF_PARM_QUERY_FLASH4SNAP,
    CAM_INTF_PARM_EXPOSURE, /* 20 */
    CAM_INTF_PARM_SHARPNESS,
    CAM_INTF_PARM_CONTRAST,
    CAM_INTF_PARM_SATURATION,
    CAM_INTF_PARM_BRIGHTNESS,
    CAM_INTF_PARM_ISO,
    CAM_INTF_PARM_ZOOM,
    CAM_INTF_PARM_ROLLOFF,
    CAM_INTF_PARM_MODE,             /* camera mode */
    CAM_INTF_PARM_AEC_ALGO_TYPE,    /* auto exposure algorithm */
    CAM_INTF_PARM_FOCUS_ALGO_TYPE, /* 30 */ /* focus algorithm */
    CAM_INTF_PARM_AEC_ROI,
    CAM_INTF_PARM_AF_ROI,
    CAM_INTF_PARM_SCE_FACTOR,
    CAM_INTF_PARM_FD,
    CAM_INTF_PARM_MCE,
    CAM_INTF_PARM_HFR,
    CAM_INTF_PARM_REDEYE_REDUCTION,
    CAM_INTF_PARM_WAVELET_DENOISE,
    CAM_INTF_PARM_TEMPORAL_DENOISE,
    CAM_INTF_PARM_HISTOGRAM, /* 40 */
    CAM_INTF_PARM_ASD_ENABLE,
    CAM_INTF_PARM_RECORDING_HINT,
    CAM_INTF_PARM_HDR,
    CAM_INTF_PARM_MAX_DIMENSION,
    CAM_INTF_PARM_RAW_DIMENSION,
    CAM_INTF_PARM_FRAMESKIP,
    CAM_INTF_PARM_ZSL_MODE,  /* indicating if it's running in ZSL mode */
    CAM_INTF_PARM_BURST_NUM,
    CAM_INTF_PARM_RETRO_BURST_NUM,
    CAM_INTF_PARM_BURST_LED_ON_PERIOD, /* 50 */
    CAM_INTF_PARM_HDR_NEED_1X, /* if HDR needs 1x output */
    CAM_INTF_PARM_LOCK_CAF,
    CAM_INTF_PARM_VIDEO_HDR,
    CAM_INTF_PARM_SENSOR_HDR,
    CAM_INTF_PARM_ROTATION,
    CAM_INTF_PARM_SCALE,
    CAM_INTF_PARM_VT, /* indicating if it's a Video Call Apllication */
    CAM_INTF_META_CROP_DATA,
    CAM_INTF_META_PREP_SNAPSHOT_DONE,
    CAM_INTF_META_GOOD_FRAME_IDX_RANGE, /* 60 */
    CAM_INTF_META_ASD_HDR_SCENE_DATA,
    CAM_INTF_META_ASD_SCENE_TYPE,
    CAM_INTF_META_CURRENT_SCENE,
    CAM_INTF_META_AEC_INFO,
    CAM_INTF_META_SENSOR_INFO,
    CAM_INTF_META_ASD_SCENE_CAPTURE_TYPE,
    CAM_INTF_META_CHROMATIX_LITE_ISP,
    CAM_INTF_META_CHROMATIX_LITE_PP,
    CAM_INTF_META_CHROMATIX_LITE_AE,
    CAM_INTF_META_CHROMATIX_LITE_AWB, /* 70 */
    CAM_INTF_META_CHROMATIX_LITE_AF,
    CAM_INTF_META_CHROMATIX_LITE_ASD,
    CAM_INTF_META_EXIF_DEBUG_AE,
    CAM_INTF_META_EXIF_DEBUG_AWB,
    CAM_INTF_META_EXIF_DEBUG_AF,
    CAM_INTF_META_EXIF_DEBUG_ASD,
    CAM_INTF_META_EXIF_DEBUG_STATS,
    CAM_INTF_PARM_GET_CHROMATIX,
    CAM_INTF_PARM_SET_RELOAD_CHROMATIX,
    CAM_INTF_PARM_SET_AUTOFOCUSTUNING, /* 80 */
    CAM_INTF_PARM_GET_AFTUNE,
    CAM_INTF_PARM_SET_RELOAD_AFTUNE,
    CAM_INTF_PARM_SET_VFE_COMMAND,
    CAM_INTF_PARM_SET_PP_COMMAND,
    CAM_INTF_PARM_TINTLESS,
    CAM_INTF_PARM_LONGSHOT_ENABLE,
    CAM_INTF_PARM_RDI_MODE,
    CAM_INTF_PARM_CDS_MODE,
    CAM_INTF_PARM_TONE_MAP_MODE,
    CAM_INTF_PARM_CAPTURE_FRAME_CONFIG,
    CAM_INTF_PARM_DUAL_LED_CALIBRATION,

    /* stream based parameters */
    CAM_INTF_PARM_DO_REPROCESS,
    CAM_INTF_PARM_SET_BUNDLE, /* 90 */
    CAM_INTF_PARM_STREAM_FLIP,
    CAM_INTF_PARM_GET_OUTPUT_CROP,

    CAM_INTF_PARM_EZTUNE_CMD,
    CAM_INTF_PARM_INT_EVT,

    /* specific to HAL3 */
    /* Whether the metadata maps to a valid frame number */
    CAM_INTF_META_FRAME_NUMBER_VALID,
    /* Whether the urgent metadata maps to a valid frame number */
    CAM_INTF_META_URGENT_FRAME_NUMBER_VALID,
    /* Whether the stream buffer corresponding this frame is dropped or not */
    CAM_INTF_META_FRAME_DROPPED,
    /* COLOR CORRECTION.*/
    CAM_INTF_META_COLOR_CORRECT_MODE,
    /* A transform matrix to chromatically adapt pixels in the CIE XYZ (1931)
     * color space from the scene illuminant to the sRGB-standard D65-illuminant. */
    CAM_INTF_META_COLOR_CORRECT_TRANSFORM,
    /*Color channel gains in the Bayer raw domain in the order [RGeGoB]*/
    CAM_INTF_META_COLOR_CORRECT_GAINS, /* 100 */
    /*The best fit color transform matrix calculated by the stats*/
    CAM_INTF_META_PRED_COLOR_CORRECT_TRANSFORM,
    /*The best fit color channels gains calculated by the stats*/
    CAM_INTF_META_PRED_COLOR_CORRECT_GAINS,
    /* CONTROL */
    /* A frame counter set by the framework. Must be maintained unchanged in
     * output frame. */
    CAM_INTF_META_FRAME_NUMBER,
    /* A frame counter set by the framework. Must be maintained unchanged in
     * output frame. */
    CAM_INTF_META_URGENT_FRAME_NUMBER,
    /*Number of streams and size of streams in current configuration*/
    CAM_INTF_META_STREAM_INFO,
    /* List of areas to use for metering */
    CAM_INTF_META_AEC_ROI,
    /* Whether the HAL must trigger precapture metering.*/
    CAM_INTF_META_AEC_PRECAPTURE_TRIGGER,
    /* The ID sent with the latest CAMERA2_TRIGGER_PRECAPTURE_METERING call */
    /* Current state of AE algorithm */
    CAM_INTF_META_AEC_STATE,
    /* List of areas to use for focus estimation */
    CAM_INTF_META_AF_ROI,
    /* Whether the HAL must trigger autofocus. */
    CAM_INTF_META_AF_TRIGGER, /* 110 */
    /* Current state of AF algorithm */
    CAM_INTF_META_AF_STATE,
    /* List of areas to use for illuminant estimation */
    CAM_INTF_META_AWB_REGIONS,
    /* Current state of AWB algorithm */
    CAM_INTF_META_AWB_STATE,
    /*Whether black level compensation is frozen or free to vary*/
    CAM_INTF_META_BLACK_LEVEL_LOCK,
    /* Information to 3A routines about the purpose of this capture, to help
     * decide optimal 3A strategy */
    CAM_INTF_META_CAPTURE_INTENT,
    /* DEMOSAIC */
    /* Controls the quality of the demosaicing processing */
    CAM_INTF_META_DEMOSAIC,
    /* EDGE */
    /* Operation mode for edge enhancement */
    CAM_INTF_META_EDGE_MODE,
    /* Control the amount of edge enhancement applied to the images.*/
    /* 1-10; 10 is maximum sharpening */
    CAM_INTF_META_SHARPNESS_STRENGTH,
    /* FLASH */
    /* Power for flash firing/torch, 10 is max power; 0 is no flash. Linear */
    CAM_INTF_META_FLASH_POWER,
    /* Firing time of flash relative to start of exposure, in nanoseconds*/
    CAM_INTF_META_FLASH_FIRING_TIME, /* 120 */
    /* Current state of the flash unit */
    CAM_INTF_META_FLASH_STATE,
    /* GEOMETRIC */
    /* Operating mode of geometric correction */
    CAM_INTF_META_GEOMETRIC_MODE,
    /* Control the amount of shading correction applied to the images */
    CAM_INTF_META_GEOMETRIC_STRENGTH,
    /* HOT PIXEL */
    /* Set operational mode for hot pixel correction */
    CAM_INTF_META_HOTPIXEL_MODE,
    /* LENS */
    /* Size of the lens aperture */
    CAM_INTF_META_LENS_APERTURE,
    /* State of lens neutral density filter(s) */
    CAM_INTF_META_LENS_FILTERDENSITY,
    /* Lens optical zoom setting */
    CAM_INTF_META_LENS_FOCAL_LENGTH,
    /* Distance to plane of sharpest focus, measured from frontmost surface
     * of the lens */
    CAM_INTF_META_LENS_FOCUS_DISTANCE,
    /* The range of scene distances that are in sharp focus (depth of field) */
    CAM_INTF_META_LENS_FOCUS_RANGE,
    /*Whether the hal needs to output the lens shading map*/
    CAM_INTF_META_LENS_SHADING_MAP_MODE, /* 130 */
    /* Current lens status */
    CAM_INTF_META_LENS_STATE,
    /* NOISE REDUCTION */
    /* Mode of operation for the noise reduction algorithm */
    CAM_INTF_META_NOISE_REDUCTION_MODE,
   /* Control the amount of noise reduction applied to the images.
    * 1-10; 10 is max noise reduction */
    CAM_INTF_META_NOISE_REDUCTION_STRENGTH,
    /* SCALER */
    /* Top-left corner and width of the output region to select from the active
     * pixel array */
    CAM_INTF_META_SCALER_CROP_REGION,
    /* The estimated scene illumination lighting frequency */
    CAM_INTF_META_SCENE_FLICKER,
    /* SENSOR */
    /* Duration each pixel is exposed to light, in nanoseconds */
    CAM_INTF_META_SENSOR_EXPOSURE_TIME,
    /* Duration from start of frame exposure to start of next frame exposure,
     * in nanoseconds */
    CAM_INTF_META_SENSOR_FRAME_DURATION,
    /* Gain applied to image data. Must be implemented through analog gain only
     * if set to values below 'maximum analog sensitivity'. */
    CAM_INTF_META_SENSOR_SENSITIVITY,
    /* Time at start of exposure of first row */
    CAM_INTF_META_SENSOR_TIMESTAMP,
    /* Duration b/w start of first row exposure and the start of last
       row exposure in nanoseconds */
    CAM_INTF_META_SENSOR_ROLLING_SHUTTER_SKEW, /* 140 */
    /* SHADING */
    /* Quality of lens shading correction applied to the image data */
    CAM_INTF_META_SHADING_MODE,
    /* Control the amount of shading correction applied to the images.
     * unitless: 1-10; 10 is full shading compensation */
    CAM_INTF_META_SHADING_STRENGTH,
    /* STATISTICS */
    /* State of the face detector unit */
    CAM_INTF_META_STATS_FACEDETECT_MODE,
    /* Operating mode for histogram generation */
    CAM_INTF_META_STATS_HISTOGRAM_MODE,
    /* Operating mode for sharpness map generation */
    CAM_INTF_META_STATS_SHARPNESS_MAP_MODE,
    /* A 3-channel sharpness map, based on the raw sensor data,
     * If only a monochrome sharpness map is supported, all channels
     * should have the same data
     */
    CAM_INTF_META_STATS_SHARPNESS_MAP,

    /* TONEMAP */
    /* Tone map mode */
    CAM_INTF_META_TONEMAP_MODE,
    /* Table mapping RGB input values to output values */
    CAM_INTF_META_TONEMAP_CURVES,

    CAM_INTF_META_FLASH_MODE,
    /* 2D array of gain factors for each color channel that was used to
     * compensate for lens shading for this frame */
    CAM_INTF_META_LENS_SHADING_MAP, /* 150 */
    CAM_INTF_META_PRIVATE_DATA,
    CAM_INTF_PARM_STATS_DEBUG_MASK,
    CAM_INTF_PARM_STATS_AF_PAAF,
    /* Indicates streams ID of all the requested buffers */
    CAM_INTF_META_STREAM_ID,
    CAM_INTF_PARM_FOCUS_BRACKETING,
    CAM_INTF_PARM_FLASH_BRACKETING,
    CAM_INTF_PARM_GET_IMG_PROP,
    CAM_INTF_META_JPEG_GPS_COORDINATES,
    CAM_INTF_META_JPEG_GPS_PROC_METHODS,
    CAM_INTF_META_JPEG_GPS_TIMESTAMP, /* 160 */
    CAM_INTF_META_JPEG_ORIENTATION,
    CAM_INTF_META_JPEG_QUALITY,
    CAM_INTF_META_JPEG_THUMB_QUALITY,
    CAM_INTF_META_JPEG_THUMB_SIZE,

    CAM_INTF_META_TEST_PATTERN_DATA,
    /* DNG file support */
    CAM_INTF_META_PROFILE_TONE_CURVE,
    CAM_INTF_META_NEUTRAL_COL_POINT,

    /* CAC */
    CAM_INTF_META_CAC_INFO,
    CAM_INTF_PARM_CAC,
    CAM_INTF_META_IMG_HYST_INFO, /* 170 */

    /* trigger for all modules to read the debug/log level properties */
    CAM_INTF_PARM_UPDATE_DEBUG_LEVEL,

    /* OTP : WB gr/gb */
    CAM_INTF_META_OTP_WB_GRGB,
    /* LED override for EZTUNE */
    CAM_INTF_META_LED_MODE_OVERRIDE,
    /* auto lens position info */
    CAM_INTF_META_FOCUS_POSITION,
    /* Manual exposure time */
    CAM_INTF_PARM_EXPOSURE_TIME,
    /* AWB meta data info */
    CAM_INTF_META_AWB_INFO,
    /* Manual lens position info */
    CAM_INTF_PARM_MANUAL_FOCUS_POS,
    /* Manual White balance gains */
    CAM_INTF_PARM_WB_MANUAL,
    /* Offline Data Overwrite */
    CAM_INTF_PARM_HW_DATA_OVERWRITE,
    /* IMG LIB reprocess debug section */
    CAM_INTF_META_IMGLIB, /* cam_intf_meta_imglib_t */ /* 180 */
    /* OEM specific parameters */
    CAM_INTF_PARM_CUSTOM,
    /* parameters added for related cameras */
    /* fetch calibration info for related cam subsystem */
    CAM_INTF_PARM_RELATED_SENSORS_CALIBRATION,
    /* focal length ratio info */
    CAM_INTF_META_AF_FOCAL_LENGTH_RATIO,
    /* crop for binning & FOV adjust */
    CAM_INTF_META_SNAP_CROP_INFO_SENSOR,
    /* crop for trimming edge pixels */
    CAM_INTF_META_SNAP_CROP_INFO_CAMIF,
    /* crop for FOV adjust and zoom */
    CAM_INTF_META_SNAP_CROP_INFO_ISP,
    /* crop for image-stabilization and zoom */
    CAM_INTF_META_SNAP_CROP_INFO_CPP,
    /* parameter for enabling DCRF */
    CAM_INTF_PARM_DCRF,
    /* metadata tag for DCRF info */
    CAM_INTF_META_DCRF,
    /* FLIP mode parameter*/
    CAM_INTF_PARM_FLIP,
    /*Frame divert info from ISP*/
    CAM_INTF_BUF_DIVERT_INFO, /* 190 */
    /*AF state change detected by AF module*/
    CAM_INTF_AF_STATE_TRANSITION, /* 191 */
    /* Param for enabling instant aec*/
    CAM_INTF_PARM_INSTANT_AEC,
    /* Param for updating initial exposure index value*/
    CAM_INTF_PARM_INITIAL_EXPOSURE_INDEX,
    CAM_INTF_PARM_MAX /* 194 */
} cam_intf_parm_type_t;

typedef struct {
    uint32_t forced;
    union {
      uint32_t force_linecount_value;
      float    force_gain_value;
      float    force_snap_exp_value;
      float    force_exp_value;
      uint32_t force_snap_linecount_value;
      float    force_snap_gain_value;
    } u;
} cam_ez_force_params_t;

typedef enum {
    CAM_EZTUNE_CMD_STATUS,
    CAM_EZTUNE_CMD_AEC_ENABLE,
    CAM_EZTUNE_CMD_AWB_ENABLE,
    CAM_EZTUNE_CMD_AF_ENABLE,
    CAM_EZTUNE_CMD_AEC_FORCE_LINECOUNT,
    CAM_EZTUNE_CMD_AEC_FORCE_GAIN,
    CAM_EZTUNE_CMD_AEC_FORCE_EXP,
    CAM_EZTUNE_CMD_AEC_FORCE_SNAP_LC,
    CAM_EZTUNE_CMD_AEC_FORCE_SNAP_GAIN,
    CAM_EZTUNE_CMD_AEC_FORCE_SNAP_EXP,
    CAM_EZTUNE_CMD_AWB_MODE,
    CAM_EZTUNE_CMD_AWB_FORCE_DUAL_LED_IDX,
} cam_eztune_cmd_type_t;

typedef struct {
  cam_eztune_cmd_type_t   cmd;
  union {
    int32_t running;
    int32_t aec_enable;
    int32_t awb_enable;
    int32_t af_enable;
    cam_ez_force_params_t ez_force_param;
    int32_t awb_mode;
    int32_t ez_force_dual_led_idx;
  } u;
} cam_eztune_cmd_data_t;


/*****************************************************************************
 *                 Code for HAL3 data types                                  *
 ****************************************************************************/
typedef enum {
    CAM_INTF_METADATA_MAX
} cam_intf_metadata_type_t;

typedef enum {
    CAM_INTENT_CUSTOM,
    CAM_INTENT_PREVIEW,
    CAM_INTENT_STILL_CAPTURE,
    CAM_INTENT_VIDEO_RECORD,
    CAM_INTENT_VIDEO_SNAPSHOT,
    CAM_INTENT_ZERO_SHUTTER_LAG,
    CAM_INTENT_MAX,
} cam_intent_t;

typedef enum {
    /* Full application control of pipeline. All 3A routines are disabled,
     * no other settings in android.control.* have any effect */
    CAM_CONTROL_OFF,
    /* Use settings for each individual 3A routine. Manual control of capture
     * parameters is disabled. All controls in android.control.* besides sceneMode
     * take effect */
    CAM_CONTROL_AUTO,
    /* Use specific scene mode. Enabling this disables control.aeMode,
     * control.awbMode and control.afMode controls; the HAL must ignore those
     * settings while USE_SCENE_MODE is active (except for FACE_PRIORITY scene mode).
     * Other control entries are still active. This setting can only be used if
     * availableSceneModes != UNSUPPORTED. TODO: Should we remove this and handle this
     * in HAL ?*/
    CAM_CONTROL_USE_SCENE_MODE,
    CAM_CONTROL_MAX
} cam_control_mode_t;

typedef enum {
    /* Use the android.colorCorrection.transform matrix to do color conversion */
    CAM_COLOR_CORRECTION_TRANSFORM_MATRIX,
    /* Must not slow down frame rate relative to raw bayer output */
    CAM_COLOR_CORRECTION_FAST,
    /* Frame rate may be reduced by high quality */
    CAM_COLOR_CORRECTION_HIGH_QUALITY,
} cam_color_correct_mode_t;

typedef enum {
    CAM_COLOR_CORRECTION_ABERRATION_OFF,
    CAM_COLOR_CORRECTION_ABERRATION_FAST,
    CAM_COLOR_CORRECTION_ABERRATION_HIGH_QUALITY,
    CAM_COLOR_CORRECTION_ABERRATION_MAX
} cam_aberration_mode_t;

#define CC_MATRIX_ROWS 3
#define CC_MATRIX_COLS 3

typedef struct {
    /* 3x3 float matrix in row-major order. each element is in range of (0, 1) */
    cam_rational_type_t transform_matrix[CC_MATRIX_ROWS][CC_MATRIX_COLS];
} cam_color_correct_matrix_t;

#define CAM_FOCAL_LENGTHS_MAX     1
#define CAM_APERTURES_MAX         1
#define CAM_FILTER_DENSITIES_MAX  1
#define CAM_MAX_MAP_HEIGHT        6
#define CAM_MAX_MAP_WIDTH         6
#define CAM_MAX_SHADING_MAP_WIDTH 17
#define CAM_MAX_SHADING_MAP_HEIGHT 13
#define CAM_MAX_TONEMAP_CURVE_SIZE    512
#define CAM_MAX_FLASH_BRACKETING    5

typedef struct {
    /* A 1D array of pairs of floats.
     * Mapping a 0-1 input range to a 0-1 output range.
     * The input range must be monotonically increasing with N,
     * and values between entries should be linearly interpolated.
     * For example, if the array is: [0.0, 0.0, 0.3, 0.5, 1.0, 1.0],
     * then the input->output mapping for a few sample points would be:
     * 0 -> 0, 0.15 -> 0.25, 0.3 -> 0.5, 0.5 -> 0.64 */
    float tonemap_points[CAM_MAX_TONEMAP_CURVE_SIZE][2];
} cam_tonemap_curve_t;

typedef struct {
   size_t tonemap_points_cnt;
   cam_tonemap_curve_t curves[3];
} cam_rgb_tonemap_curves;

typedef struct {
   size_t tonemap_points_cnt;
   cam_tonemap_curve_t curve;
} cam_profile_tone_curve;

#define NEUTRAL_COL_POINTS 3

typedef struct {
    cam_rational_type_t neutral_col_point[NEUTRAL_COL_POINTS];
} cam_neutral_col_point_t;

typedef enum {
    OFF,
    FAST,
    QUALITY,
} cam_quality_preference_t;

typedef enum {
    CAM_FLASH_CTRL_OFF,
    CAM_FLASH_CTRL_SINGLE,
    CAM_FLASH_CTRL_TORCH
} cam_flash_ctrl_t;

typedef struct {
    uint8_t frame_dropped; /*  This flag indicates whether any stream buffer is dropped or not */
    cam_stream_ID_t cam_stream_ID; /* if dropped, Stream ID of dropped streams */
} cam_frame_dropped_t;

typedef struct {
    uint8_t ae_mode;
    uint8_t awb_mode;
    uint8_t af_mode;
} cam_scene_mode_overrides_t;

typedef struct {
    int32_t left;
    int32_t top;
    int32_t width;
    int32_t height;
} cam_crop_region_t;

typedef struct {
    /* Estimated sharpness for each region of the input image.
     * Normalized to be between 0 and maxSharpnessMapValue.
     * Higher values mean sharper (better focused) */
    int32_t sharpness[CAM_MAX_MAP_WIDTH][CAM_MAX_MAP_HEIGHT];
} cam_sharpness_map_t;

typedef struct {
   float lens_shading[4*CAM_MAX_SHADING_MAP_HEIGHT*CAM_MAX_SHADING_MAP_WIDTH];
} cam_lens_shading_map_t;

typedef struct {
    int32_t min_value;
    int32_t max_value;
    int32_t def_value;
    int32_t step;
} cam_control_range_t;

#define CAM_QCOM_FEATURE_NONE            0U
#define CAM_QCOM_FEATURE_FACE_DETECTION (1U<<0)
#define CAM_QCOM_FEATURE_DENOISE2D      (1U<<1)
#define CAM_QCOM_FEATURE_CROP           (1U<<2)
#define CAM_QCOM_FEATURE_ROTATION       (1U<<3)
#define CAM_QCOM_FEATURE_FLIP           (1U<<4)
#define CAM_QCOM_FEATURE_HDR            (1U<<5)
#define CAM_QCOM_FEATURE_REGISTER_FACE  (1U<<6)
#define CAM_QCOM_FEATURE_SHARPNESS      (1U<<7)
#define CAM_QCOM_FEATURE_VIDEO_HDR      (1U<<8)
#define CAM_QCOM_FEATURE_CAC            (1U<<9)
#define CAM_QCOM_FEATURE_SCALE          (1U<<10)
#define CAM_QCOM_FEATURE_EFFECT         (1U<<11)
#define CAM_QCOM_FEATURE_UBIFOCUS       (1U<<12)
#define CAM_QCOM_FEATURE_CHROMA_FLASH   (1U<<13)
#define CAM_QCOM_FEATURE_OPTIZOOM       (1U<<14)
#define CAM_QCOM_FEATURE_SENSOR_HDR     (1U<<15)
#define CAM_QCOM_FEATURE_REFOCUS        (1U<<16)
#define CAM_QCOM_FEATURE_CPP_TNR        (1U<<17)
#define CAM_QCOM_FEATURE_RAW_PROCESSING (1U<<18)
#define CAM_QCOM_FEATURE_TRUEPORTRAIT   (1U<<19)
#define CAM_QCOM_FEATURE_LLVD           (1U<<20)
#define CAM_QCOM_FEATURE_DIS20          (1U<<21)
#define CAM_QCOM_FEATURE_STILLMORE      (1U<<22)
#define CAM_QCOM_FEATURE_DCRF           (1U<<23)
#define CAM_QCOM_FEATURE_CDS            (1U<<24)
#define CAM_QCOM_FEATURE_EZTUNE         (1U<<25)
#define CAM_QCOM_FEATURE_DSDN           (1U<<26) //Special CDS in CPP block
#define CAM_QCOM_FEATURE_SW2D           (1U<<27)
#define CAM_QTI_FEATURE_SW_TNR          (1U<<28)
#define CAM_QCOM_FEATURE_MAX            (1U<<29)
#define CAM_QCOM_FEATURE_PP_SUPERSET    (CAM_QCOM_FEATURE_DENOISE2D|CAM_QCOM_FEATURE_CROP|\
                                         CAM_QCOM_FEATURE_ROTATION|CAM_QCOM_FEATURE_SHARPNESS|\
                                         CAM_QCOM_FEATURE_SCALE|CAM_QCOM_FEATURE_CAC|\
                                         CAM_QCOM_FEATURE_EZTUNE)

#define CAM_QCOM_FEATURE_PP_PASS_1      CAM_QCOM_FEATURE_PP_SUPERSET
#define CAM_QCOM_FEATURE_PP_PASS_2      CAM_QCOM_FEATURE_SCALE | CAM_QCOM_FEATURE_CROP;

// Counter clock wise
typedef enum {
    ROTATE_0 = 1<<0,
    ROTATE_90 = 1<<1,
    ROTATE_180 = 1<<2,
    ROTATE_270 = 1<<3,
} cam_rotation_t;

typedef struct {
   cam_rotation_t rotation;         /* jpeg rotation */
   cam_rotation_t device_rotation;  /* device rotation */
   uint32_t streamId;
} cam_rotation_info_t;

typedef enum {
    FLIP_NONE = 0, /* 00b */
    FLIP_H = 1,    /* 01b */
    FLIP_V = 2,    /* 10b */
    FLIP_V_H = 3,  /* 11b */
} cam_flip_t;

typedef struct {
    uint32_t bundle_id;                            /* bundle id */
    uint8_t num_of_streams;                        /* number of streams in the bundle */
    uint32_t stream_ids[MAX_STREAM_NUM_IN_BUNDLE]; /* array of stream ids to be bundled */
} cam_bundle_config_t;

typedef enum {
    CAM_ONLINE_REPROCESS_TYPE,    /* online reprocess, frames from running streams */
    CAM_OFFLINE_REPROCESS_TYPE,   /* offline reprocess, frames from external source */
} cam_reprocess_type_enum_t;

typedef struct {
    uint8_t burst_count;
    uint8_t min_burst_count;
    uint8_t max_burst_count;
} cam_still_more_t;

typedef struct {
    uint8_t burst_count;
    uint8_t output_count;
    uint8_t flash_bracketing[CAM_MAX_FLASH_BRACKETING];
    uint8_t metadata_index;
} cam_chroma_flash_t;

typedef enum {
    CAM_HDR_MODE_SINGLEFRAME,    /* Single frame HDR mode which does only tone mapping */
    CAM_HDR_MODE_MULTIFRAME,     /* Multi frame HDR mode which needs two frames with 0.5x and 2x exposure respectively */
} cam_hdr_mode_enum_t;

typedef struct {
    uint32_t hdr_enable;
    uint32_t hdr_need_1x; /* when CAM_QCOM_FEATURE_HDR enabled, indicate if 1x is needed for output */
    cam_hdr_mode_enum_t hdr_mode;
} cam_hdr_param_t;

typedef struct {
    int32_t output_width;
    int32_t output_height;
} cam_scale_param_t;

typedef struct {
    uint8_t enable;
    uint8_t burst_count;
    uint8_t focus_steps[MAX_AF_BRACKETING_VALUES];
    uint8_t output_count;
    uint32_t meta_max_size;
} cam_af_bracketing_t;

typedef struct {
    uint8_t enable;
    uint8_t burst_count;
} cam_flash_bracketing_t;

typedef struct {
    uint8_t enable;
    uint8_t burst_count;
    uint8_t zoom_threshold;
} cam_opti_zoom_t;

typedef struct {
    size_t meta_max_size;
} cam_true_portrait_t;

typedef enum {
    CAM_FLASH_OFF,
    CAM_FLASH_ON
} cam_flash_value_t;

typedef struct {
    cam_sensor_t sens_type;
    cam_format_t native_format;
} cam_sensor_type_t;

typedef struct {
    uint32_t result;
    uint32_t header_size;
    uint32_t width;
    uint32_t height;
    uint8_t data[0];
} cam_misc_buf_t;

typedef struct {
    uint32_t misc_buffer_index;
} cam_misc_buf_param_t;

typedef struct {
    /* reprocess feature mask */
    uint32_t feature_mask;

    /* individual setting for features to be reprocessed */
    cam_denoise_param_t denoise2d;
    cam_rect_t input_crop;
    cam_rotation_t rotation;
    uint32_t flip;
    int32_t sharpness;
    int32_t effect;
    cam_hdr_param_t hdr_param;
    cam_scale_param_t scale_param;

    uint8_t zoom_level;
    cam_flash_value_t flash_value;
    cam_misc_buf_param_t misc_buf_param;
    uint32_t burst_cnt;
} cam_pp_feature_config_t;

typedef struct {
    uint32_t input_stream_id;
    /* input source stream type */
    cam_stream_type_t input_stream_type;
} cam_pp_online_src_config_t;

typedef struct {
    /* image format */
    cam_format_t input_fmt;

    /* image dimension */
    cam_dimension_t input_dim;

    /* buffer plane information, will be calc based on stream_type, fmt,
       dim, and padding_info(from stream config). Info including:
       offset_x, offset_y, stride, scanline, plane offset */
    cam_stream_buf_plane_info_t input_buf_planes;

    /* number of input reprocess buffers */
    uint8_t num_of_bufs;

    /* input source type */
    cam_stream_type_t input_type;

} cam_pp_offline_src_config_t;

/* reprocess stream input configuration */
typedef struct {
    /* input source config */
    cam_reprocess_type_enum_t pp_type;
    union {
        cam_pp_online_src_config_t online;
        cam_pp_offline_src_config_t offline;
    };

    /* pp feature config */
    cam_pp_feature_config_t pp_feature_config;
} cam_stream_reproc_config_t;

typedef struct {
    uint8_t crop_enabled;
    cam_rect_t input_crop;
} cam_crop_param_t;

typedef struct {
    uint8_t trigger;
    int32_t trigger_id;
} cam_trigger_t;

typedef struct {
    cam_denoise_param_t denoise2d;
    cam_crop_param_t crop;
    uint32_t flip;     /* 0 means no flip */
    uint32_t uv_upsample; /* 0 means no chroma upsampling */
    int32_t sharpness; /* 0 means no sharpness */
    int32_t effect;
    cam_rotation_t rotation;
    cam_rotation_t device_rotation;
} cam_per_frame_pp_config_t;

typedef enum {
    CAM_OPT_STAB_OFF,
    CAM_OPT_STAB_ON,
    CAM_OPT_STAB_MAX
} cam_optical_stab_modes_t;

typedef enum {
    CAM_FILTER_ARRANGEMENT_RGGB,
    CAM_FILTER_ARRANGEMENT_GRBG,
    CAM_FILTER_ARRANGEMENT_GBRG,
    CAM_FILTER_ARRANGEMENT_BGGR,

    /* Sensor is not Bayer; output has 3 16-bit values for each pixel,
     * instead of just 1 16-bit value per pixel.*/
    CAM_FILTER_ARRANGEMENT_RGB,
    /* Sensor is YUV; SW do not have access to actual RAW,
     * output is interleaved UYVY */
    CAM_FILTER_ARRANGEMENT_UYVY,
    CAM_FILTER_ARRANGEMENT_YUYV,
} cam_color_filter_arrangement_t;

typedef enum {
  CAM_AF_LENS_STATE_STATIONARY,
  CAM_AF_LENS_STATE_MOVING,
} cam_af_lens_state_t;

typedef enum {
    CAM_AWB_STATE_INACTIVE,
    CAM_AWB_STATE_SEARCHING,
    CAM_AWB_STATE_CONVERGED,
    CAM_AWB_STATE_LOCKED
} cam_awb_state_t;

typedef enum {
    CAM_FOCUS_UNCALIBRATED,
    CAM_FOCUS_APPROXIMATE,
    CAM_FOCUS_CALIBRATED
} cam_focus_calibration_t;

typedef enum {
    CAM_TEST_PATTERN_OFF,
    CAM_TEST_PATTERN_SOLID_COLOR,
    CAM_TEST_PATTERN_COLOR_BARS,
    CAM_TEST_PATTERN_COLOR_BARS_FADE_TO_GRAY,
    CAM_TEST_PATTERN_PN9,
} cam_test_pattern_mode_t;

typedef struct {
    cam_test_pattern_mode_t mode;
    int32_t r;
    int32_t gr;
    int32_t gb;
    int32_t b;
} cam_test_pattern_data_t;

typedef enum {
    CAM_AWB_D50,
    CAM_AWB_D65,
    CAM_AWB_D75,
    CAM_AWB_A,
    CAM_AWB_CUSTOM_A,
    CAM_AWB_WARM_FLO,
    CAM_AWB_COLD_FLO,
    CAM_AWB_CUSTOM_FLO,
    CAM_AWB_NOON,
    CAM_AWB_CUSTOM_DAYLIGHT,
    CAM_AWB_INVALID_ALL_LIGHT,
} cam_illuminat_t;

typedef enum {
    LEGACY_RAW,
    MIPI_RAW,
} cam_opaque_raw_format_t;

typedef enum {
    CAM_PERF_NORMAL = 0,
    CAM_PERF_HIGH_PERFORMANCE,
} cam_perf_mode_t;

typedef struct {
    float real_gain;
    float lux_idx;
    float exp_time;
} cam_intf_aec_t;

#define CAM_INTF_AEC_DATA_MAX   (10)

typedef struct {
    uint32_t frame_count;
    cam_intf_aec_t aec_data[CAM_INTF_AEC_DATA_MAX];
} cam_intf_meta_imglib_input_aec_t;

typedef struct {
    cam_intf_meta_imglib_input_aec_t meta_imglib_input_aec;
} cam_intf_meta_imglib_t;

/***********************************
* ENUM definition for custom parameter type
************************************/
typedef enum {
    CAM_CUSTOM_PARM_EXAMPLE,
    CAM_CUSTOM_PARM_MAX,
} cam_custom_parm_type;


#endif /* __QCAMERA_TYPES_H__ */
