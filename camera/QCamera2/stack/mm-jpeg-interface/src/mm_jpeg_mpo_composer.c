/* Copyright (c) 2015-2016, The Linux Foundation. All rights reserved.
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

#define ATRACE_TAG ATRACE_TAG_CAMERA

#include <pthread.h>
#include "mm_jpeg_dbg.h"
#include "mm_jpeg_mpo.h"

#define M_APP0    0xe0
#define M_APP1    0xe1
#define M_APP2    0xe2
#define M_EOI     0xd9
#define M_SOI     0xd8

/** READ_LONG:
 *  @b: Buffer start addr
 *  @o: Buffer offset to start reading
 *
 *  Read long value from the specified buff addr at given offset
 **/
#define READ_LONG(b, o)  \
  (uint32_t)(((uint32_t)b[o] << 24) + \
  ((uint32_t)b[o+1] << 16) + \
  ((uint32_t)b[o+2] << 8) + \
  ((uint32_t)b[o+3]))

/** READ_LONG_LITTLE:
 *  @b: Buffer start addr
 *  @o: Buffer offset to start reading
 *
 *  Read long value from the specified buff addr at given offset
 *  in Little Endian
 **/
#define READ_LONG_LITTLE(b, o)  \
  (uint32_t)(((uint32_t)b[o + 3] << 24) + \
  ((uint32_t) b[o + 2] << 16) + \
  ((uint32_t) b[o + 1] << 8) + \
  (uint32_t) b[o]);

/** READ_LONG:
 *  @b: Buffer start addr
 *  @o: Buffer offset to start reading
 *
 *  Read short value from the specified buff addr at given
 *  offset
 **/
#define READ_SHORT(b, o)  \
  (uint16_t) (((uint16_t)b[o] << 8) + \
  (uint16_t) b[o + 1]);

/*Mutex to serializa MPO composition*/
static pthread_mutex_t g_mpo_lock = PTHREAD_MUTEX_INITIALIZER;

/** mm_jpeg_mpo_write_long_little_endian
 *
 *  Arguments:
 *    @buffer_addr: image start addr
 *    @buff_offset: offset in the buffer
 *    @buffer_size: Size of the buffer
 *    @value: Value to write
 *    @overflow : Overflow flag
 *
 *  Return:
 *       None
 *
 *  Description:
 *       Write value at the given offset
 *
 **/
void mm_jpeg_mpo_write_long_little_endian(uint8_t *buff_addr, uint32_t buff_offset,
  uint32_t buffer_size, int value, uint8_t *overflow)
{
  if (buff_offset + 3 >= buffer_size) {
    *overflow = TRUE;
  }

  if (!(*overflow)) {
    buff_addr[buff_offset + 3] = (uint8_t)((value >> 24) & 0xFF);
    buff_addr[buff_offset + 2] = (uint8_t)((value >> 16) & 0xFF);
    buff_addr[buff_offset + 1] = (uint8_t)((value >> 8) & 0xFF);
    buff_addr[buff_offset] = (uint8_t)(value & 0xFF);
  }
}

/** mm_jpeg_mpo_write_long
 *
 *  Arguments:
 *    @buffer_addr: image start addr
 *    @buff_offset: offset in the buffer
 *    @buffer_size: Size of the buffer
 *    @value: Value to write
 *    @overflow : Overflow flag
 *
 *  Return:
 *       None
 *
 *  Description:
 *       Write value at the given offset
 *
 **/
void mm_jpeg_mpo_write_long(uint8_t *buff_addr, uint32_t buff_offset,
  uint32_t buffer_size, int value, uint8_t *overflow)
{
  if ((buff_offset + 3) >= buffer_size) {
    *overflow = TRUE;
  }

  if (!(*overflow)) {
    buff_addr[buff_offset] = (uint8_t)((value >> 24) & 0xFF);
    buff_addr[buff_offset+1] = (uint8_t)((value >> 16) & 0xFF);
    buff_addr[buff_offset+2] = (uint8_t)((value >> 8) & 0xFF);
    buff_addr[buff_offset+3] = (uint8_t)(value & 0xFF);
  }
}

/** mm_jpeg_mpo_get_app_marker
 *
 *  Arguments:
 *    @buffer_addr: Jpeg image start addr
 *    @buffer_size: Size of the Buffer
 *    @app_marker: app_marker to find
 *
 *  Return:
 *       Start offset of the specified app marker
 *
 *  Description:
 *       Gets the start offset of the given app marker
 *
 **/
uint8_t *mm_jpeg_mpo_get_app_marker(uint8_t *buffer_addr, int buffer_size,
  int app_marker)
{
  int32_t byte;
  uint8_t *p_current_addr = NULL, *p_start_offset = NULL;
  uint16_t app_marker_size = 0;

  p_current_addr = buffer_addr;
  do {
    do {
      byte = *(p_current_addr);
      p_current_addr++;
    }
    while ((byte != 0xFF) &&
      (p_current_addr < (buffer_addr + (buffer_size - 1))));

    //If 0xFF is not found at all, break
    if (byte != 0xFF) {
      CDBG("%s %d: 0xFF not found", __func__, __LINE__);
      break;
    }

    //Read the next byte after 0xFF
    byte = *(p_current_addr);
    CDBG("%s %d: Byte %x", __func__, __LINE__, byte);
    if (byte == app_marker) {
      CDBG("%s %d: Byte %x", __func__, __LINE__, byte);
      p_start_offset = ++p_current_addr;
      break;
    } else if (byte != M_SOI) {
      app_marker_size = READ_SHORT(p_current_addr, 1);
      CDBG("%s %d: size %d", __func__, __LINE__, app_marker_size);
      p_current_addr += app_marker_size;
    }
  }
  while ((byte != M_EOI) &&
    (p_current_addr < (buffer_addr + (buffer_size - 1))));

  return p_start_offset;
}

/** mm_jpeg_mpo_get_mp_header
 *
 *  Arguments:
 *    @app2_marker: app2_marker start offset
 *
 *  Return:
 *       Start offset of the MP header
 *
 *  Description:
 *       Get the start offset of the MP header (before the MP
 *       Endian field). All offsets in the MP header need to be
 *       specified wrt this start offset.
 *
 **/
uint8_t *mm_jpeg_mpo_get_mp_header(uint8_t *app2_start_offset)
{
  uint8_t *mp_headr_start_offset = NULL;

  if (app2_start_offset != NULL) {
    mp_headr_start_offset = app2_start_offset + MP_APP2_FIELD_LENGTH_BYTES +
      MP_FORMAT_IDENTIFIER_BYTES;
  }

  return mp_headr_start_offset;
}

/** mm_jpeg_mpo_update_header
 *
 *  Arguments:
 *    @mpo_info: MPO Info
 *
 *  Return:
 *       0 - Success
 *       -1 - otherwise
 *
 *  Description:
 *      Update the MP Index IFD of the first image with info
 *      about about all other images.
 *
 **/
int mm_jpeg_mpo_update_header(mm_jpeg_mpo_info_t *mpo_info)
{
  uint8_t *app2_start_off_addr = NULL, *mp_headr_start_off_addr = NULL;
  uint32_t mp_index_ifd_offset = 0, current_offset = 0, mp_entry_val_offset = 0;
  uint8_t *aux_start_addr = NULL;
  uint8_t overflow_flag = 0;
  int i = 0, rc = -1;
  uint32_t endianess = MPO_LITTLE_ENDIAN, offset_to_nxt_ifd = 8;
  uint16_t ifd_tag_count = 0;

  //Get the addr of the App Marker
  app2_start_off_addr = mm_jpeg_mpo_get_app_marker(
    mpo_info->output_buff.buf_vaddr, mpo_info->primary_image.buf_filled_len, M_APP2);
  if (!app2_start_off_addr) {
    CDBG_ERROR("%s %d:] Cannot find App2 marker. MPO composition failed",
      __func__, __LINE__ );
    return rc;
  }
  CDBG("%s %d:] app2_start_off_addr %p = %x", __func__, __LINE__,
    app2_start_off_addr, *app2_start_off_addr);

  //Get the addr of the MP Headr start offset.
  //All offsets in the MP header are wrt to this addr
  mp_headr_start_off_addr = mm_jpeg_mpo_get_mp_header(app2_start_off_addr);
  if (!mp_headr_start_off_addr) {
    CDBG_ERROR("%s %d:] mp headr start offset is NULL. MPO composition failed",
      __func__, __LINE__ );
    return rc;
  }
  CDBG("%s %d:] mp_headr_start_off_addr %x", __func__, __LINE__,
    *mp_headr_start_off_addr);

  current_offset = mp_headr_start_off_addr - mpo_info->output_buff.buf_vaddr;

  endianess = READ_LONG(mpo_info->output_buff.buf_vaddr, current_offset);
  CDBG("%s %d:] Endianess %d", __func__, __LINE__, endianess);

  //Add offset to first ifd
  current_offset += MP_ENDIAN_BYTES;

  //Read the value to get MP Index IFD.
  if (endianess == MPO_LITTLE_ENDIAN) {
    offset_to_nxt_ifd = READ_LONG_LITTLE(mpo_info->output_buff.buf_vaddr,
      current_offset);
  } else {
    offset_to_nxt_ifd = READ_LONG(mpo_info->output_buff.buf_vaddr,
      current_offset);
  }
  CDBG("%s %d:] offset_to_nxt_ifd %d", __func__, __LINE__, offset_to_nxt_ifd);

  current_offset = ((mp_headr_start_off_addr + offset_to_nxt_ifd) -
    mpo_info->output_buff.buf_vaddr);
  mp_index_ifd_offset = current_offset;
  CDBG("%s %d:] mp_index_ifd_offset %d", __func__, __LINE__,
    mp_index_ifd_offset);

  //Traverse to MP Entry value
  ifd_tag_count = READ_SHORT(mpo_info->output_buff.buf_vaddr, current_offset);
  CDBG("%s %d:] Tag count in MP entry %d", __func__, __LINE__, ifd_tag_count);
  current_offset += MP_INDEX_COUNT_BYTES;

  /* Get MP Entry Value offset - Count * 12 (Each tag is 12 bytes)*/
  current_offset += (ifd_tag_count * 12);
  /*Add Offset to next IFD*/
  current_offset += MP_INDEX_OFFSET_OF_NEXT_IFD_BYTES;

  mp_entry_val_offset = current_offset;
  CDBG("%s %d:] MP Entry value offset %d", __func__, __LINE__,
    mp_entry_val_offset);

  //Update image size for primary image
  current_offset += MP_INDEX_ENTRY_INDIVIDUAL_IMAGE_ATTRIBUTE_BYTES;
  if (endianess == MPO_LITTLE_ENDIAN) {
    mm_jpeg_mpo_write_long_little_endian(mpo_info->output_buff.buf_vaddr,
      current_offset, mpo_info->output_buff_size,
      mpo_info->primary_image.buf_filled_len, &overflow_flag);
  } else {
    mm_jpeg_mpo_write_long(mpo_info->output_buff.buf_vaddr,
      current_offset, mpo_info->output_buff_size,
      mpo_info->primary_image.buf_filled_len, &overflow_flag);
  }

  aux_start_addr = mpo_info->output_buff.buf_vaddr +
    mpo_info->primary_image.buf_filled_len;

  for (i = 0; i < mpo_info->num_of_images - 1; i++) {
    //Go to MP Entry val for each image
    mp_entry_val_offset += MP_INDEX_ENTRY_VALUE_BYTES;
    current_offset = mp_entry_val_offset;

    //Update image size
    current_offset += MP_INDEX_ENTRY_INDIVIDUAL_IMAGE_ATTRIBUTE_BYTES;
    if (endianess == MPO_LITTLE_ENDIAN) {
      mm_jpeg_mpo_write_long_little_endian(mpo_info->output_buff.buf_vaddr,
        current_offset, mpo_info->output_buff_size,
        mpo_info->aux_images[i].buf_filled_len, &overflow_flag);
    } else {
      mm_jpeg_mpo_write_long(mpo_info->output_buff.buf_vaddr,
        current_offset, mpo_info->output_buff_size,
        mpo_info->aux_images[i].buf_filled_len, &overflow_flag);
    }
    CDBG("%s %d:] aux[%d] start_addr %x", __func__, __LINE__, i,
       *aux_start_addr);
    //Update the offset
    current_offset += MP_INDEX_ENTRY_INDIVIDUAL_IMAGE_SIZE_BYTES;
    if (endianess == MPO_LITTLE_ENDIAN) {
      mm_jpeg_mpo_write_long_little_endian(mpo_info->output_buff.buf_vaddr,
        current_offset, mpo_info->output_buff_size,
        aux_start_addr - mp_headr_start_off_addr, &overflow_flag);
    } else {
      mm_jpeg_mpo_write_long(mpo_info->output_buff.buf_vaddr,
        current_offset, mpo_info->output_buff_size,
        aux_start_addr - mp_headr_start_off_addr, &overflow_flag);
    }
    aux_start_addr += mpo_info->aux_images[i].buf_filled_len;
  }
  if (!overflow_flag) {
    rc = 0;
  }
  return rc;
}

/** mm_jpeg_mpo_compose
 *
 *  Arguments:
 *    @mpo_info: MPO Info
 *
 *  Return:
 *       0 - Success
 *      -1 - otherwise
 *
 *  Description:
 *      Compose MPO image from multiple JPEG images
 *
 **/
int mm_jpeg_mpo_compose(mm_jpeg_mpo_info_t *mpo_info)
{
  uint8_t *aux_write_offset = NULL;
  int i = 0, rc = -1;

  pthread_mutex_lock(&g_mpo_lock);

  //Primary image needs to be copied to the o/p buffer if its not already
  if (mpo_info->output_buff.buf_filled_len == 0) {
    if (mpo_info->primary_image.buf_filled_len < mpo_info->output_buff_size) {
      memcpy(mpo_info->output_buff.buf_vaddr, mpo_info->primary_image.buf_vaddr,
        mpo_info->primary_image.buf_filled_len);
      mpo_info->output_buff.buf_filled_len +=
        mpo_info->primary_image.buf_filled_len;
    } else {
      CDBG_ERROR("%s %d: O/P buffer not large enough. MPO composition failed",
        __func__, __LINE__);
      pthread_mutex_unlock(&g_mpo_lock);
      return rc;
    }
  }
  //Append each Aux image to the buffer
  for (i = 0; i < mpo_info->num_of_images - 1; i++) {
    if ((mpo_info->output_buff.buf_filled_len +
      mpo_info->aux_images[i].buf_filled_len) <= mpo_info->output_buff_size) {
      aux_write_offset = mpo_info->output_buff.buf_vaddr +
        mpo_info->output_buff.buf_filled_len;
      memcpy(aux_write_offset, mpo_info->aux_images[i].buf_vaddr,
        mpo_info->aux_images[i].buf_filled_len);
      mpo_info->output_buff.buf_filled_len +=
        mpo_info->aux_images[i].buf_filled_len;
    } else {
      CDBG_ERROR("%s %d: O/P buffer not large enough. MPO composition failed",
          __func__, __LINE__);
      pthread_mutex_unlock(&g_mpo_lock);
      return rc;
    }
  }

  rc = mm_jpeg_mpo_update_header(mpo_info);
  pthread_mutex_unlock(&g_mpo_lock);

  return rc;
}
