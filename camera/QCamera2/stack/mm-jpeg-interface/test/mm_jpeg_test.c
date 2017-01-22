/* Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
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

#include "mm_jpeg_interface.h"
#include "mm_jpeg_ionbuf.h"
#include <sys/time.h>
#include <stdlib.h>

#define MAX_NUM_BUFS (12)

/** DUMP_TO_FILE:
 *  @filename: file name
 *  @p_addr: address of the buffer
 *  @len: buffer length
 *
 *  dump the image to the file
 **/
#define DUMP_TO_FILE(filename, p_addr, len) ({ \
  FILE *fp = fopen(filename, "w+"); \
  if (fp) { \
    fwrite(p_addr, 1, len, fp); \
    fclose(fp); \
  } else { \
    CDBG_ERROR("%s:%d] cannot dump image", __func__, __LINE__); \
  } \
})

static uint32_t g_count = 1U, g_i;

typedef struct {
  mm_jpeg_color_format fmt;
  cam_rational_type_t mult;
  const char *str;
} mm_jpeg_intf_test_colfmt_t;

typedef struct {
  char *filename;
  int width;
  int height;
  char *out_filename;
  uint32_t burst_mode;
  uint32_t min_out_bufs;
  mm_jpeg_intf_test_colfmt_t col_fmt;
  uint32_t encode_thumbnail;
  int tmb_width;
  int tmb_height;
  int main_quality;
  int thumb_quality;
} jpeg_test_input_t;

/* Static constants */
/*  default Luma Qtable */
const uint8_t DEFAULT_QTABLE_0[QUANT_SIZE] = {
  16, 11, 10, 16, 24, 40, 51, 61,
  12, 12, 14, 19, 26, 58, 60, 55,
  14, 13, 16, 24, 40, 57, 69, 56,
  14, 17, 22, 29, 51, 87, 80, 62,
  18, 22, 37, 56, 68, 109, 103, 77,
  24, 35, 55, 64, 81, 104, 113, 92,
  49, 64, 78, 87, 103, 121, 120, 101,
  72, 92, 95, 98, 112, 100, 103, 99
};

/*  default Chroma Qtable */
const uint8_t DEFAULT_QTABLE_1[QUANT_SIZE] = {
  17, 18, 24, 47, 99, 99, 99, 99,
  18, 21, 26, 66, 99, 99, 99, 99,
  24, 26, 56, 99, 99, 99, 99, 99,
  47, 66, 99, 99, 99, 99, 99, 99,
  99, 99, 99, 99, 99, 99, 99, 99,
  99, 99, 99, 99, 99, 99, 99, 99,
  99, 99, 99, 99, 99, 99, 99, 99,
  99, 99, 99, 99, 99, 99, 99, 99
};

typedef struct {
  char *filename[MAX_NUM_BUFS];
  int width;
  int height;
  char *out_filename[MAX_NUM_BUFS];
  pthread_mutex_t lock;
  pthread_cond_t cond;
  buffer_t input[MAX_NUM_BUFS];
  buffer_t output[MAX_NUM_BUFS];
  int use_ion;
  uint32_t handle;
  mm_jpeg_ops_t ops;
  uint32_t job_id[MAX_NUM_BUFS];
  mm_jpeg_encode_params_t params;
  mm_jpeg_job_t job;
  uint32_t session_id;
  uint32_t num_bufs;
  uint32_t min_out_bufs;
  size_t buf_filled_len[MAX_NUM_BUFS];
} mm_jpeg_intf_test_t;



static const mm_jpeg_intf_test_colfmt_t color_formats[] =
{
  { MM_JPEG_COLOR_FORMAT_YCRCBLP_H2V2, {3, 2}, "YCRCBLP_H2V2" },
  { MM_JPEG_COLOR_FORMAT_YCBCRLP_H2V2, {3, 2}, "YCBCRLP_H2V2" },
  { MM_JPEG_COLOR_FORMAT_YCRCBLP_H2V1, {2, 1}, "YCRCBLP_H2V1" },
  { MM_JPEG_COLOR_FORMAT_YCBCRLP_H2V1, {2, 1}, "YCBCRLP_H2V1" },
  { MM_JPEG_COLOR_FORMAT_YCRCBLP_H1V2, {2, 1}, "YCRCBLP_H1V2" },
  { MM_JPEG_COLOR_FORMAT_YCBCRLP_H1V2, {2, 1}, "YCBCRLP_H1V2" },
  { MM_JPEG_COLOR_FORMAT_YCRCBLP_H1V1, {3, 1}, "YCRCBLP_H1V1" },
  { MM_JPEG_COLOR_FORMAT_YCBCRLP_H1V1, {3, 1}, "YCBCRLP_H1V1" }
};

static jpeg_test_input_t jpeg_input[] = {
  { QCAMERA_DUMP_FRM_LOCATION"test_1.yuv", 4000, 3008, QCAMERA_DUMP_FRM_LOCATION"test_1.jpg", 0, 0,
  { MM_JPEG_COLOR_FORMAT_YCRCBLP_H2V2, {3, 2}, "YCRCBLP_H2V2" }, 0, 320, 240, 80, 80}
};

static void mm_jpeg_encode_callback(jpeg_job_status_t status,
  uint32_t client_hdl,
  uint32_t jobId,
  mm_jpeg_output_t *p_output,
  void *userData)
{
  mm_jpeg_intf_test_t *p_obj = (mm_jpeg_intf_test_t *)userData;

  pthread_mutex_lock(&p_obj->lock);

  if (status == JPEG_JOB_STATUS_ERROR) {
    CDBG_ERROR("%s:%d] Encode error", __func__, __LINE__);
  } else {
    int i = 0;
    for (i = 0; p_obj->job_id[i] && (jobId != p_obj->job_id[i]); i++)
      ;
    if (!p_obj->job_id[i]) {
      CDBG_ERROR("%s:%d] Cannot find job ID!!!", __func__, __LINE__);
      goto error;
    }
    CDBG_ERROR("%s:%d] Encode success addr %p len %zu idx %d",
      __func__, __LINE__, p_output->buf_vaddr, p_output->buf_filled_len, i);

    p_obj->buf_filled_len[i] = p_output->buf_filled_len;
    if (p_obj->min_out_bufs) {
      CDBG_ERROR("%s:%d] Saving file%s addr %p len %zu",
          __func__, __LINE__, p_obj->out_filename[i],
          p_output->buf_vaddr, p_output->buf_filled_len);

      DUMP_TO_FILE(p_obj->out_filename[i], p_output->buf_vaddr,
        p_output->buf_filled_len);
    }
  }
  g_i++;

error:

  if (g_i >= g_count) {
    CDBG_ERROR("%s:%d] Signal the thread", __func__, __LINE__);
    pthread_cond_signal(&p_obj->cond);
  }
  pthread_mutex_unlock(&p_obj->lock);
}

int mm_jpeg_test_alloc(buffer_t *p_buffer, int use_pmem)
{
  int ret = 0;
  /*Allocate buffers*/
  if (use_pmem) {
    p_buffer->addr = (uint8_t *)buffer_allocate(p_buffer, 0);
    if (NULL == p_buffer->addr) {
      CDBG_ERROR("%s:%d] Error",__func__, __LINE__);
      return -1;
    }
  } else {
    /* Allocate heap memory */
    p_buffer->addr = (uint8_t *)malloc(p_buffer->size);
    if (NULL == p_buffer->addr) {
      CDBG_ERROR("%s:%d] Error",__func__, __LINE__);
      return -1;
    }
  }
  return ret;
}

void mm_jpeg_test_free(buffer_t *p_buffer)
{
  if (p_buffer->addr == NULL)
    return;

  if (p_buffer->p_pmem_fd >= 0)
    buffer_deallocate(p_buffer);
  else
    free(p_buffer->addr);

  memset(p_buffer, 0x0, sizeof(buffer_t));
}

int mm_jpeg_test_read(mm_jpeg_intf_test_t *p_obj, uint32_t idx)
{
  FILE *fp = NULL;
  size_t file_size = 0;
  fp = fopen(p_obj->filename[idx], "rb");
  if (!fp) {
    CDBG_ERROR("%s:%d] error", __func__, __LINE__);
    return -1;
  }
  fseek(fp, 0, SEEK_END);
  file_size = (size_t)ftell(fp);
  fseek(fp, 0, SEEK_SET);
  CDBG_ERROR("%s:%d] input file size is %zu buf_size %zu",
    __func__, __LINE__, file_size, p_obj->input[idx].size);

  if (p_obj->input[idx].size > file_size) {
    CDBG_ERROR("%s:%d] error", __func__, __LINE__);
    fclose(fp);
    return -1;
  }
  fread(p_obj->input[idx].addr, 1, p_obj->input[idx].size, fp);
  fclose(fp);
  return 0;
}

static int encode_init(jpeg_test_input_t *p_input, mm_jpeg_intf_test_t *p_obj)
{
  int rc = -1;
  size_t size = (size_t)(p_input->width * p_input->height);
  mm_jpeg_encode_params_t *p_params = &p_obj->params;
  mm_jpeg_encode_job_t *p_job_params = &p_obj->job.encode_job;
  uint32_t i = 0;
  uint32_t burst_mode = p_input->burst_mode;
  jpeg_test_input_t *p_in = p_input;

  do {
    p_obj->filename[i] = p_in->filename;
    p_obj->width = p_input->width;
    p_obj->height = p_input->height;
    p_obj->out_filename[i] = p_in->out_filename;
    p_obj->use_ion = 1;
    p_obj->min_out_bufs = p_input->min_out_bufs;

    /* allocate buffers */
    p_obj->input[i].size = size * (size_t)p_input->col_fmt.mult.numerator /
        (size_t)p_input->col_fmt.mult.denominator;
    rc = mm_jpeg_test_alloc(&p_obj->input[i], p_obj->use_ion);
    if (rc) {
      CDBG_ERROR("%s:%d] Error",__func__, __LINE__);
      return -1;
    }


    rc = mm_jpeg_test_read(p_obj, i);
    if (rc) {
      CDBG_ERROR("%s:%d] Error",__func__, __LINE__);
      return -1;
    }

    /* src buffer config*/
    p_params->src_main_buf[i].buf_size = p_obj->input[i].size;
    p_params->src_main_buf[i].buf_vaddr = p_obj->input[i].addr;
    p_params->src_main_buf[i].fd = p_obj->input[i].p_pmem_fd;
    p_params->src_main_buf[i].index = i;
    p_params->src_main_buf[i].format = MM_JPEG_FMT_YUV;
    p_params->src_main_buf[i].offset.mp[0].len = (uint32_t)size;
    p_params->src_main_buf[i].offset.mp[0].stride = p_input->width;
    p_params->src_main_buf[i].offset.mp[0].scanline = p_input->height;
    p_params->src_main_buf[i].offset.mp[1].len = (uint32_t)(size >> 1);

    /* src buffer config*/
    p_params->src_thumb_buf[i].buf_size = p_obj->input[i].size;
    p_params->src_thumb_buf[i].buf_vaddr = p_obj->input[i].addr;
    p_params->src_thumb_buf[i].fd = p_obj->input[i].p_pmem_fd;
    p_params->src_thumb_buf[i].index = i;
    p_params->src_thumb_buf[i].format = MM_JPEG_FMT_YUV;
    p_params->src_thumb_buf[i].offset.mp[0].len = (uint32_t)size;
    p_params->src_thumb_buf[i].offset.mp[0].stride = p_input->width;
    p_params->src_thumb_buf[i].offset.mp[0].scanline = p_input->height;
    p_params->src_thumb_buf[i].offset.mp[1].len = (uint32_t)(size >> 1);


    i++;
  } while((++p_in)->filename);

  p_obj->num_bufs = i;

  pthread_mutex_init(&p_obj->lock, NULL);
  pthread_cond_init(&p_obj->cond, NULL);


  /* set encode parameters */
  p_params->jpeg_cb = mm_jpeg_encode_callback;
  p_params->userdata = p_obj;
  p_params->color_format = p_input->col_fmt.fmt;
  p_params->thumb_color_format = p_input->col_fmt.fmt;

  if (p_obj->min_out_bufs) {
    p_params->num_dst_bufs = 2;
  } else {
    p_params->num_dst_bufs = p_obj->num_bufs;
  }

  for (i = 0; i < (uint32_t)p_params->num_dst_bufs; i++) {
    p_obj->output[i].size = size * 3/2;
    rc = mm_jpeg_test_alloc(&p_obj->output[i], 0);
    if (rc) {
      CDBG_ERROR("%s:%d] Error",__func__, __LINE__);
      return -1;
    }
    /* dest buffer config */
    p_params->dest_buf[i].buf_size = p_obj->output[i].size;
    p_params->dest_buf[i].buf_vaddr = p_obj->output[i].addr;
    p_params->dest_buf[i].fd = p_obj->output[i].p_pmem_fd;
    p_params->dest_buf[i].index = i;
  }


  p_params->num_src_bufs = p_obj->num_bufs;
  p_params->num_tmb_bufs = 0;
  g_count = p_params->num_src_bufs;

  p_params->encode_thumbnail = p_input->encode_thumbnail;
  if (p_params->encode_thumbnail) {
      p_params->num_tmb_bufs = p_obj->num_bufs;
  }
  p_params->quality = (uint32_t)p_input->main_quality;
  p_params->thumb_quality = (uint32_t)p_input->thumb_quality;

  p_job_params->dst_index = 0;
  p_job_params->src_index = 0;
  p_job_params->rotation = 0;

  /* main dimension */
  p_job_params->main_dim.src_dim.width = p_obj->width;
  p_job_params->main_dim.src_dim.height = p_obj->height;
  p_job_params->main_dim.dst_dim.width = p_obj->width;
  p_job_params->main_dim.dst_dim.height = p_obj->height;
  p_job_params->main_dim.crop.top = 0;
  p_job_params->main_dim.crop.left = 0;
  p_job_params->main_dim.crop.width = p_obj->width;
  p_job_params->main_dim.crop.height = p_obj->height;

  p_params->main_dim  = p_job_params->main_dim;

  /* thumb dimension */
  p_job_params->thumb_dim.src_dim.width = p_obj->width;
  p_job_params->thumb_dim.src_dim.height = p_obj->height;
  p_job_params->thumb_dim.dst_dim.width = p_input->tmb_width;
  p_job_params->thumb_dim.dst_dim.height = p_input->tmb_height;
  p_job_params->thumb_dim.crop.top = 0;
  p_job_params->thumb_dim.crop.left = 0;
  p_job_params->thumb_dim.crop.width = 0;
  p_job_params->thumb_dim.crop.height = 0;

  p_params->thumb_dim  = p_job_params->thumb_dim;

  p_job_params->exif_info.numOfEntries = 0;
  p_params->burst_mode = burst_mode;

  /* Qtable */
  p_job_params->qtable[0].eQuantizationTable =
    OMX_IMAGE_QuantizationTableLuma;
  p_job_params->qtable[1].eQuantizationTable =
    OMX_IMAGE_QuantizationTableChroma;
  p_job_params->qtable_set[0] = 1;
  p_job_params->qtable_set[1] = 1;

  for (i = 0; i < QUANT_SIZE; i++) {
    p_job_params->qtable[0].nQuantizationMatrix[i] = DEFAULT_QTABLE_0[i];
    p_job_params->qtable[1].nQuantizationMatrix[i] = DEFAULT_QTABLE_1[i];
  }

  return 0;
}

static int encode_test(jpeg_test_input_t *p_input)
{
  int rc = 0;
  mm_jpeg_intf_test_t jpeg_obj;
  uint32_t i = 0;

  memset(&jpeg_obj, 0x0, sizeof(jpeg_obj));
  rc = encode_init(p_input, &jpeg_obj);
  if (rc) {
    CDBG_ERROR("%s:%d] Error",__func__, __LINE__);
    return -1;
  }

  mm_dimension pic_size;
  memset(&pic_size, 0, sizeof(mm_dimension));
  pic_size.w = (uint32_t)p_input->width;
  pic_size.h = (uint32_t)p_input->height;

  jpeg_obj.handle = jpeg_open(&jpeg_obj.ops, NULL, pic_size, NULL);
  if (jpeg_obj.handle == 0) {
    CDBG_ERROR("%s:%d] Error",__func__, __LINE__);
    goto end;
  }

  rc = jpeg_obj.ops.create_session(jpeg_obj.handle, &jpeg_obj.params,
    &jpeg_obj.job.encode_job.session_id);
  if (jpeg_obj.job.encode_job.session_id == 0) {
    CDBG_ERROR("%s:%d] Error",__func__, __LINE__);
    goto end;
  }

  for (i = 0; i < jpeg_obj.num_bufs; i++) {
    jpeg_obj.job.job_type = JPEG_JOB_TYPE_ENCODE;
    jpeg_obj.job.encode_job.src_index = (int32_t) i;
    jpeg_obj.job.encode_job.dst_index = (int32_t) i;
    jpeg_obj.job.encode_job.thumb_index = (uint32_t) i;

    if (jpeg_obj.params.burst_mode && jpeg_obj.min_out_bufs) {
      jpeg_obj.job.encode_job.dst_index = -1;
    }

    rc = jpeg_obj.ops.start_job(&jpeg_obj.job, &jpeg_obj.job_id[i]);

    if (rc) {
      CDBG_ERROR("%s:%d] Error",__func__, __LINE__);
      goto end;
    }
  }
  jpeg_obj.job_id[i] = 0;

  /*
  usleep(5);
  jpeg_obj.ops.abort_job(jpeg_obj.job_id[0]);
  */
  pthread_mutex_lock(&jpeg_obj.lock);
  pthread_cond_wait(&jpeg_obj.cond, &jpeg_obj.lock);
  pthread_mutex_unlock(&jpeg_obj.lock);


  jpeg_obj.ops.destroy_session(jpeg_obj.job.encode_job.session_id);
  jpeg_obj.ops.close(jpeg_obj.handle);

end:
  for (i = 0; i < jpeg_obj.num_bufs; i++) {
    if (!jpeg_obj.min_out_bufs) {
      // Save output files
      CDBG_ERROR("%s:%d] Saving file%s addr %p len %zu",
          __func__, __LINE__,jpeg_obj.out_filename[i],
          jpeg_obj.output[i].addr, jpeg_obj.buf_filled_len[i]);

      DUMP_TO_FILE(jpeg_obj.out_filename[i], jpeg_obj.output[i].addr,
        jpeg_obj.buf_filled_len[i]);
    }
    mm_jpeg_test_free(&jpeg_obj.input[i]);
    mm_jpeg_test_free(&jpeg_obj.output[i]);
  }
  return 0;
}

#define MAX_FILE_CNT (20)
static int mm_jpeg_test_get_input(int argc, char *argv[],
    jpeg_test_input_t *p_test)
{
  int c;
  size_t in_file_cnt = 0, out_file_cnt = 0, i;
  int idx = 0;
  jpeg_test_input_t *p_test_base = p_test;

  char *in_files[MAX_FILE_CNT];
  char *out_files[MAX_FILE_CNT];

  while ((c = getopt(argc, argv, "-I:O:W:H:F:BTx:y:Q:q:")) != -1) {
    switch (c) {
    case 'B':
      fprintf(stderr, "%-25s\n", "Using burst mode");
      p_test->burst_mode = 1;
      break;
    case 'I':
      for (idx = optind - 1; idx < argc; idx++) {
        if (argv[idx][0] == '-') {
          break;
        }
        in_files[in_file_cnt++] = argv[idx];
      }
      optind = idx -1;

      break;
    case 'O':
      for (idx = optind - 1; idx < argc; idx++) {
        if (argv[idx][0] == '-') {
          break;
        }
        out_files[out_file_cnt++] = argv[idx];
      }
      optind = idx -1;

      break;
    case 'W':
      p_test->width = atoi(optarg);
      fprintf(stderr, "%-25s%d\n", "Width: ", p_test->width);
      break;
    case 'H':
      p_test->height = atoi(optarg);
      fprintf(stderr, "%-25s%d\n", "Height: ", p_test->height);
      break;
    case 'F':
      p_test->col_fmt = color_formats[atoi(optarg)];
      fprintf(stderr, "%-25s%s\n", "Format: ", p_test->col_fmt.str);
      break;
    case 'M':
      p_test->min_out_bufs = 1;
      fprintf(stderr, "%-25s\n", "Using minimum number of output buffers");
      break;
    case 'T':
      p_test->encode_thumbnail = 1;
      fprintf(stderr, "%-25s\n", "Encode thumbnail");
      break;
    case 'x':
      p_test->tmb_width = atoi(optarg);
      fprintf(stderr, "%-25s%d\n", "Tmb Width: ", p_test->tmb_width);
      break;
    case 'y':
      p_test->tmb_height = atoi(optarg);
      fprintf(stderr, "%-25s%d\n", "Tmb Height: ", p_test->tmb_height);
      break;
    case 'Q':
      p_test->main_quality = atoi(optarg);
      fprintf(stderr, "%-25s%d\n", "Main quality: ", p_test->main_quality);
      break;
    case 'q':
      p_test->thumb_quality = atoi(optarg);
      fprintf(stderr, "%-25s%d\n", "Thumb quality: ", p_test->thumb_quality);
      break;
    default:;
    }
  }
  fprintf(stderr, "Infiles: %zu Outfiles: %zu\n", in_file_cnt, out_file_cnt);

  if (in_file_cnt > out_file_cnt) {
    fprintf(stderr, "%-25s\n", "Insufficient number of output files!");
    return 1;
  }

  // Discard the extra out files
  out_file_cnt = in_file_cnt;

  p_test = realloc(p_test, (in_file_cnt + 1) * sizeof(*p_test));
  if (!p_test) {
    CDBG_ERROR("%s:%d] Error",__func__, __LINE__);
    return 1;
  }
  memset(p_test+1, 0, (in_file_cnt) * sizeof(*p_test));

  for (i = 0; i < in_file_cnt; i++, p_test++) {
    memcpy(p_test, p_test_base, sizeof(*p_test));
    p_test->filename = in_files[i];
    p_test->out_filename = out_files[i];
    fprintf(stderr, "Inf: %s Outf: %s\n", in_files[i], out_files[i]);
  }


  return 0;
}

static void mm_jpeg_test_print_usage()
{
  fprintf(stderr, "Usage: program_name [options]\n");
  fprintf(stderr, "Mandatory options:\n");
  fprintf(stderr, "  -I FILE1 [FILE2] [FILEN]\tList of input files\n");
  fprintf(stderr, "  -O FILE1 [FILE2] [FILEN]\tList of output files\n");
  fprintf(stderr, "  -W WIDTH\t\tOutput image width\n");
  fprintf(stderr, "  -H HEIGHT\t\tOutput image height\n");
  fprintf(stderr, "  -F \t\tColor format: \n");
  fprintf(stderr, "\t\t\t\t%s (0), %s (1), %s (2) %s (3)\n"
      "\t\t\t\t%s (4), %s (5), %s (6) %s (7)\n ",
      color_formats[0].str, color_formats[1].str,
      color_formats[2].str, color_formats[3].str,
      color_formats[4].str, color_formats[5].str,
      color_formats[6].str, color_formats[7].str);
  fprintf(stderr, "Optional:\n");
  fprintf(stderr, "  -T \t\Encode thumbnail\n");
  fprintf(stderr, "  -x TMB_WIDTH\t\tThumbnail width\n");
  fprintf(stderr, "  -y TMB_HEIGHT\t\tThumbnail height\n");
  fprintf(stderr, "  -Q MAIN_QUALITY\t\tMain image quality\n");
  fprintf(stderr, "  -q TMB_QUALITY\t\tThumbnail image quality\n");
  fprintf(stderr, "  -B \t\tBurst mode. Utilize both encoder engines on"
          "supported targets\n");
  fprintf(stderr, "  -M \t\tUse minimum number of output buffers \n");
  fprintf(stderr, "\n");
}

/** main:
 *
 *  Arguments:
 *    @argc
 *    @argv
 *
 *  Return:
 *       0 or -ve values
 *
 *  Description:
 *       main function
 *
 **/
int main(int argc, char* argv[])
{
  jpeg_test_input_t *p_test_input;
  int ret = 0;
  if (argc > 1) {
    p_test_input = calloc(2, sizeof(*p_test_input));
    if (!p_test_input) {
      CDBG_ERROR("%s:%d] Error",__func__, __LINE__);
      goto exit;
    }
    memcpy(p_test_input, &jpeg_input[0], sizeof(*p_test_input));
    ret = mm_jpeg_test_get_input(argc, argv, p_test_input);
    if (ret) {
      CDBG_ERROR("%s:%d] Error",__func__, __LINE__);
      goto exit;
    }
  } else {
    mm_jpeg_test_print_usage();
    return 1;
  }
  ret = encode_test(p_test_input);

exit:
  if (!ret) {
    fprintf(stderr, "%-25s\n", "Success!");
  } else {
    fprintf(stderr, "%-25s\n", "Fail!");
  }

  if (argc > 1) {
    if (p_test_input) {
      free(p_test_input);
      p_test_input = NULL;
    }
  }

  return ret;
}


