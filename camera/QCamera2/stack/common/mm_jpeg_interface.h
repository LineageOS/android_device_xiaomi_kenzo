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

#ifndef MM_JPEG_INTERFACE_H_
#define MM_JPEG_INTERFACE_H_
#include "QOMX_JpegExtensions.h"
#include "cam_intf.h"

#define MM_JPEG_MAX_PLANES 3
#define MM_JPEG_MAX_BUF CAM_MAX_NUM_BUFS_PER_STREAM
#define QUANT_SIZE 64
#define QTABLE_MAX 2
#define MM_JPEG_MAX_MPO_IMAGES 2

typedef enum {
  MM_JPEG_FMT_YUV,
  MM_JPEG_FMT_BITSTREAM
} mm_jpeg_format_t;

typedef enum {
  MM_JPEG_TYPE_JPEG,
  MM_JPEG_TYPE_MPO
} mm_jpeg_image_type_t;

typedef struct {
  cam_ae_exif_debug_t ae_debug_params;
  cam_awb_exif_debug_t awb_debug_params;
  cam_af_exif_debug_t af_debug_params;
  cam_asd_exif_debug_t asd_debug_params;
  cam_stats_buffer_exif_debug_t stats_debug_params;
  uint8_t ae_debug_params_valid;
  uint8_t awb_debug_params_valid;
  uint8_t af_debug_params_valid;
  uint8_t asd_debug_params_valid;
  uint8_t stats_debug_params_valid;
} mm_jpeg_debug_exif_params_t;

typedef struct {
  cam_3a_params_t cam_3a_params;
  uint8_t cam_3a_params_valid;
  cam_sensor_params_t sensor_params;
  mm_jpeg_debug_exif_params_t *debug_params;
} mm_jpeg_exif_params_t;

typedef struct {
  /* Indicates if it is a single jpeg or part of a multi picture sequence*/
  mm_jpeg_image_type_t type;

  /*Indicates if image is the primary image in a sequence of images.
  Applicable only to multi picture formats*/
  uint8_t is_primary;

  /*Number of images in the sequence*/
  uint32_t num_of_images;
} mm_jpeg_multi_image_t;

typedef struct {
  uint32_t sequence;          /* for jpeg bit streams, assembling is based on sequence. sequence starts from 0 */
  uint8_t *buf_vaddr;        /* ptr to buf */
  int fd;                    /* fd of buf */
  size_t buf_size;         /* total size of buf (header + image) */
  mm_jpeg_format_t format;   /* buffer format*/
  cam_frame_len_offset_t offset; /* offset of all the planes */
  uint32_t index; /* index used to identify the buffers */
} mm_jpeg_buf_t;

typedef struct {
  uint8_t *buf_vaddr;        /* ptr to buf */
  int fd;                    /* fd of buf */
  size_t buf_filled_len;   /* used for output image. filled by the client */
} mm_jpeg_output_t;

typedef enum {
  MM_JPEG_COLOR_FORMAT_YCRCBLP_H2V2,
  MM_JPEG_COLOR_FORMAT_YCBCRLP_H2V2,
  MM_JPEG_COLOR_FORMAT_YCRCBLP_H2V1,
  MM_JPEG_COLOR_FORMAT_YCBCRLP_H2V1,
  MM_JPEG_COLOR_FORMAT_YCRCBLP_H1V2,
  MM_JPEG_COLOR_FORMAT_YCBCRLP_H1V2,
  MM_JPEG_COLOR_FORMAT_YCRCBLP_H1V1,
  MM_JPEG_COLOR_FORMAT_YCBCRLP_H1V1,
  MM_JPEG_COLOR_FORMAT_MONOCHROME,
  MM_JPEG_COLOR_FORMAT_BITSTREAM_H2V2,
  MM_JPEG_COLOR_FORMAT_BITSTREAM_H2V1,
  MM_JPEG_COLOR_FORMAT_BITSTREAM_H1V2,
  MM_JPEG_COLOR_FORMAT_BITSTREAM_H1V1,
  MM_JPEG_COLOR_FORMAT_MAX
} mm_jpeg_color_format;

typedef enum {
  JPEG_JOB_STATUS_DONE = 0,
  JPEG_JOB_STATUS_ERROR
} jpeg_job_status_t;

typedef void (*jpeg_encode_callback_t)(jpeg_job_status_t status,
  uint32_t client_hdl,
  uint32_t jobId,
  mm_jpeg_output_t *p_output,
  void *userData);

typedef struct {
  /* src img dimension */
  cam_dimension_t src_dim;

  /* jpeg output dimension */
  cam_dimension_t dst_dim;

  /* crop information */
  cam_rect_t crop;
} mm_jpeg_dim_t;

typedef struct {
  /* num of buf in src img */
  uint32_t num_src_bufs;

  /* num of src tmb bufs */
  uint32_t num_tmb_bufs;

  /* num of buf in src img */
  uint32_t num_dst_bufs;

  /* should create thumbnail from main image or not */
  uint32_t encode_thumbnail;

  /* src img bufs */
  mm_jpeg_buf_t src_main_buf[MM_JPEG_MAX_BUF];

  /* this will be used only for bitstream */
  mm_jpeg_buf_t src_thumb_buf[MM_JPEG_MAX_BUF];

  /* this will be used only for bitstream */
  mm_jpeg_buf_t dest_buf[MM_JPEG_MAX_BUF];

  /* mainimage color format */
  mm_jpeg_color_format color_format;

  /* thumbnail color format */
  mm_jpeg_color_format thumb_color_format;

  /* jpeg quality: range 0~100 */
  uint32_t quality;

  /* jpeg thumbnail quality: range 0~100 */
  uint32_t thumb_quality;

  /* buf to exif entries, caller needs to
   * take care of the memory manage with insider ptr */
  QOMX_EXIF_INFO exif_info;

  /*Callback registered to be called after encode*/
  jpeg_encode_callback_t jpeg_cb;

  /*Appdata passed by the user*/
  void* userdata;

  /* thumbnail dimension */
  mm_jpeg_dim_t thumb_dim;

  /* rotation informaiton */
  uint32_t rotation;

  /* thumb rotation informaiton */
  uint32_t thumb_rotation;

  /* main image dimension */
  mm_jpeg_dim_t main_dim;

  /* enable encoder burst mode */
  uint32_t burst_mode;

  /* get memory function ptr */
  int (*get_memory)( omx_jpeg_ouput_buf_t *p_out_buf);

  /* release memory function ptr */
  int (*put_memory)( omx_jpeg_ouput_buf_t *p_out_buf);
} mm_jpeg_encode_params_t;

typedef struct {
  /* num of buf in src img */
  uint32_t num_src_bufs;

  /* num of buf in src img */
  uint32_t num_dst_bufs;

  /* src img bufs */
  mm_jpeg_buf_t src_main_buf[MM_JPEG_MAX_BUF];

  /* this will be used only for bitstream */
  mm_jpeg_buf_t dest_buf[MM_JPEG_MAX_BUF];

  /* color format */
  mm_jpeg_color_format color_format;

  jpeg_encode_callback_t jpeg_cb;

  void* userdata;

} mm_jpeg_decode_params_t;

typedef struct {
  /* active indices of the buffers for encoding */
  int32_t src_index;
  int32_t dst_index;
  uint32_t thumb_index;
  mm_jpeg_dim_t thumb_dim;

  /* rotation informaiton */
  uint32_t rotation;

  /* main image dimension */
  mm_jpeg_dim_t main_dim;

  /*session id*/
  uint32_t session_id;

  /* jpeg output buffer ref count */
  int32_t ref_count;

  /* allocated jpeg output buffer */
  void *alloc_out_buffer;

  /*Metadata stream*/
  metadata_buffer_t *p_metadata;

  /*HAL version*/
  cam_hal_version_t hal_version;

  /* buf to exif entries, caller needs to
   * take care of the memory manage with insider ptr */
  QOMX_EXIF_INFO exif_info;

  /* 3a parameters */
  mm_jpeg_exif_params_t cam_exif_params;

  /* jpeg encoder QTable */
  uint8_t qtable_set[QTABLE_MAX];

  OMX_IMAGE_PARAM_QUANTIZATIONTABLETYPE qtable[QTABLE_MAX];

  /* flag to enable/disable mobicat */
  uint8_t mobicat_mask;

  /*Info associated with multiple image sequence*/
  mm_jpeg_multi_image_t multi_image_info;

  /* work buf */
  mm_jpeg_buf_t work_buf;
} mm_jpeg_encode_job_t;

typedef struct {
  /* active indices of the buffers for encoding */
  int32_t src_index;
  int32_t dst_index;
  uint32_t tmb_dst_index;

  /* rotation informaiton */
  uint32_t rotation;

  /* main image  */
  mm_jpeg_dim_t main_dim;

  /*session id*/
  uint32_t session_id;
} mm_jpeg_decode_job_t;

typedef enum {
  JPEG_JOB_TYPE_ENCODE,
  JPEG_JOB_TYPE_DECODE,
  JPEG_JOB_TYPE_MAX
} mm_jpeg_job_type_t;

typedef struct {
  mm_jpeg_job_type_t job_type;
  union {
    mm_jpeg_encode_job_t encode_job;
    mm_jpeg_decode_job_t decode_job;
  };
} mm_jpeg_job_t;

typedef struct {
  uint32_t w;
  uint32_t h;
} mm_dimension;

typedef struct {
  /*Primary image in the MPO sequence*/
  mm_jpeg_output_t primary_image;

  /*All auxillary images in the sequence*/
  mm_jpeg_output_t aux_images[MM_JPEG_MAX_MPO_IMAGES - 1];

  /*Total number of images in the MPO sequence*/
  int num_of_images;

  /*Output MPO buffer*/
  mm_jpeg_output_t output_buff;

  /*Size of the allocated output buffer*/
  size_t output_buff_size;
} mm_jpeg_mpo_info_t;

typedef struct {
  /* config a job -- async call */
  int (*start_job)(mm_jpeg_job_t* job, uint32_t* job_id);

  /* abort a job -- sync call */
  int (*abort_job)(uint32_t job_id);

  /* create a session */
  int (*create_session)(uint32_t client_hdl,
    mm_jpeg_encode_params_t *p_params, uint32_t *p_session_id);

  /* destroy session */
  int (*destroy_session)(uint32_t session_id);

  /* close a jpeg client -- sync call */
  int (*close) (uint32_t clientHdl);

} mm_jpeg_ops_t;

typedef struct {
  /* config a job -- async call */
  int (*start_job)(mm_jpeg_job_t* job, uint32_t* job_id);

  /* abort a job -- sync call */
  int (*abort_job)(uint32_t job_id);

  /* create a session */
  int (*create_session)(uint32_t client_hdl,
    mm_jpeg_decode_params_t *p_params, uint32_t *p_session_id);

  /* destroy session */
  int (*destroy_session)(uint32_t session_id);

  /* close a jpeg client -- sync call */
  int (*close) (uint32_t clientHdl);
} mm_jpegdec_ops_t;

typedef struct {

  /* Get Mpo size*/
  int (*get_mpo_size)(mm_jpeg_output_t jpeg_buffer[MM_JPEG_MAX_MPO_IMAGES],
    int num_of_images);

  /* Compose MPO*/
  int (*compose_mpo)(mm_jpeg_mpo_info_t *mpo_info);

} mm_jpeg_mpo_ops_t;

/* open a jpeg client -- sync call
 * returns client_handle.
 * failed if client_handle=0
 * jpeg ops tbl and mpo ops tbl will be filled in if open succeeds
 * and jpeg meta data will be cached */
uint32_t jpeg_open(mm_jpeg_ops_t *ops, mm_jpeg_mpo_ops_t *mpo_ops,
  mm_dimension picture_size,
  cam_jpeg_metadata_t *jpeg_metadata);

/* open a jpeg client -- sync call
 * returns client_handle.
 * failed if client_handle=0
 * jpeg ops tbl will be filled in if open succeeds */
uint32_t jpegdec_open(mm_jpegdec_ops_t *ops);

#endif /* MM_JPEG_INTERFACE_H_ */
