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

#include <pthread.h>
#include "mm_jpeg_dbg.h"
#include "mm_jpeg.h"
#include <errno.h>
#include <math.h>


#define LOWER(a)               ((a) & 0xFFFF)
#define UPPER(a)               (((a)>>16) & 0xFFFF)
#define CHANGE_ENDIAN_16(a)  ((0x00FF & ((a)>>8)) | (0xFF00 & ((a)<<8)))
#define ROUND(a) \
        ((a >= 0) ? (uint32_t)(a + 0.5) : (uint32_t)(a - 0.5))


/** addExifEntry:
 *
 *  Arguments:
 *   @exif_info : Exif info struct
 *   @p_session: job session
 *   @tagid   : exif tag ID
 *   @type    : data type
 *   @count   : number of data in uint of its type
 *   @data    : input data ptr
 *
 *  Retrun     : int32_t type of status
 *               0  -- success
 *              none-zero failure code
 *
 *  Description:
 *       Function to add an entry to exif data
 *
 **/
int32_t addExifEntry(QOMX_EXIF_INFO *p_exif_info, exif_tag_id_t tagid,
  exif_tag_type_t type, uint32_t count, void *data)
{
    int32_t rc = 0;
    uint32_t numOfEntries = (uint32_t)p_exif_info->numOfEntries;
    QEXIF_INFO_DATA *p_info_data = p_exif_info->exif_data;
    if(numOfEntries >= MAX_EXIF_TABLE_ENTRIES) {
        ALOGE("%s: Number of entries exceeded limit", __func__);
        return -1;
    }

    p_info_data[numOfEntries].tag_id = tagid;
    p_info_data[numOfEntries].tag_entry.type = type;
    p_info_data[numOfEntries].tag_entry.count = count;
    p_info_data[numOfEntries].tag_entry.copy = 1;
    switch (type) {
    case EXIF_BYTE: {
      if (count > 1) {
        uint8_t *values = (uint8_t *)malloc(count);
        if (values == NULL) {
          ALOGE("%s: No memory for byte array", __func__);
          rc = -1;
        } else {
          memcpy(values, data, count);
          p_info_data[numOfEntries].tag_entry.data._bytes = values;
        }
      } else {
        p_info_data[numOfEntries].tag_entry.data._byte = *(uint8_t *)data;
      }
    }
    break;
    case EXIF_ASCII: {
      char *str = NULL;
      str = (char *)malloc(count + 1);
      if (str == NULL) {
        ALOGE("%s: No memory for ascii string", __func__);
        rc = -1;
      } else {
        memset(str, 0, count + 1);
        memcpy(str, data, count);
        p_info_data[numOfEntries].tag_entry.data._ascii = str;
      }
    }
    break;
    case EXIF_SHORT: {
      if (count > 1) {
        uint16_t *values = (uint16_t *)malloc(count * sizeof(uint16_t));
        if (values == NULL) {
          ALOGE("%s: No memory for short array", __func__);
          rc = -1;
        } else {
          memcpy(values, data, count * sizeof(uint16_t));
          p_info_data[numOfEntries].tag_entry.data._shorts = values;
        }
      } else {
        p_info_data[numOfEntries].tag_entry.data._short = *(uint16_t *)data;
      }
    }
    break;
    case EXIF_LONG: {
      if (count > 1) {
        uint32_t *values = (uint32_t *)malloc(count * sizeof(uint32_t));
        if (values == NULL) {
          ALOGE("%s: No memory for long array", __func__);
          rc = -1;
        } else {
          memcpy(values, data, count * sizeof(uint32_t));
          p_info_data[numOfEntries].tag_entry.data._longs = values;
        }
      } else {
        p_info_data[numOfEntries].tag_entry.data._long = *(uint32_t *)data;
      }
    }
    break;
    case EXIF_RATIONAL: {
      if (count > 1) {
        rat_t *values = (rat_t *)malloc(count * sizeof(rat_t));
        if (values == NULL) {
          ALOGE("%s: No memory for rational array", __func__);
          rc = -1;
        } else {
          memcpy(values, data, count * sizeof(rat_t));
          p_info_data[numOfEntries].tag_entry.data._rats = values;
        }
      } else {
        p_info_data[numOfEntries].tag_entry.data._rat = *(rat_t *)data;
      }
    }
    break;
    case EXIF_UNDEFINED: {
      uint8_t *values = (uint8_t *)malloc(count);
      if (values == NULL) {
        ALOGE("%s: No memory for undefined array", __func__);
        rc = -1;
      } else {
        memcpy(values, data, count);
        p_info_data[numOfEntries].tag_entry.data._undefined = values;
      }
    }
    break;
    case EXIF_SLONG: {
      if (count > 1) {
        int32_t *values = (int32_t *)malloc(count * sizeof(int32_t));
        if (values == NULL) {
          ALOGE("%s: No memory for signed long array", __func__);
          rc = -1;
        } else {
          memcpy(values, data, count * sizeof(int32_t));
          p_info_data[numOfEntries].tag_entry.data._slongs = values;
        }
      } else {
        p_info_data[numOfEntries].tag_entry.data._slong = *(int32_t *)data;
      }
    }
    break;
    case EXIF_SRATIONAL: {
      if (count > 1) {
        srat_t *values = (srat_t *)malloc(count * sizeof(srat_t));
        if (values == NULL) {
          ALOGE("%s: No memory for signed rational array", __func__);
          rc = -1;
        } else {
          memcpy(values, data, count * sizeof(srat_t));
          p_info_data[numOfEntries].tag_entry.data._srats = values;
        }
      } else {
        p_info_data[numOfEntries].tag_entry.data._srat = *(srat_t *)data;
      }
    }
    break;
    }

    // Increase number of entries
    p_exif_info->numOfEntries++;
    return rc;
}

/** releaseExifEntry
 *
 *  Arguments:
 *   @p_exif_data : Exif info struct
 *
 *  Retrun     : int32_t type of status
 *               0  -- success
 *              none-zero failure code
 *
 *  Description:
 *       Function to release an entry from exif data
 *
 **/
int32_t releaseExifEntry(QEXIF_INFO_DATA *p_exif_data)
{
 switch (p_exif_data->tag_entry.type) {
  case EXIF_BYTE: {
    if (p_exif_data->tag_entry.count > 1 &&
      p_exif_data->tag_entry.data._bytes != NULL) {
      free(p_exif_data->tag_entry.data._bytes);
      p_exif_data->tag_entry.data._bytes = NULL;
    }
  }
  break;
  case EXIF_ASCII: {
    if (p_exif_data->tag_entry.data._ascii != NULL) {
      free(p_exif_data->tag_entry.data._ascii);
      p_exif_data->tag_entry.data._ascii = NULL;
    }
  }
  break;
  case EXIF_SHORT: {
    if (p_exif_data->tag_entry.count > 1 &&
      p_exif_data->tag_entry.data._shorts != NULL) {
      free(p_exif_data->tag_entry.data._shorts);
      p_exif_data->tag_entry.data._shorts = NULL;
    }
  }
  break;
  case EXIF_LONG: {
    if (p_exif_data->tag_entry.count > 1 &&
      p_exif_data->tag_entry.data._longs != NULL) {
      free(p_exif_data->tag_entry.data._longs);
      p_exif_data->tag_entry.data._longs = NULL;
    }
  }
  break;
  case EXIF_RATIONAL: {
    if (p_exif_data->tag_entry.count > 1 &&
      p_exif_data->tag_entry.data._rats != NULL) {
      free(p_exif_data->tag_entry.data._rats);
      p_exif_data->tag_entry.data._rats = NULL;
    }
  }
  break;
  case EXIF_UNDEFINED: {
    if (p_exif_data->tag_entry.data._undefined != NULL) {
      free(p_exif_data->tag_entry.data._undefined);
      p_exif_data->tag_entry.data._undefined = NULL;
    }
  }
  break;
  case EXIF_SLONG: {
    if (p_exif_data->tag_entry.count > 1 &&
      p_exif_data->tag_entry.data._slongs != NULL) {
      free(p_exif_data->tag_entry.data._slongs);
      p_exif_data->tag_entry.data._slongs = NULL;
    }
  }
  break;
  case EXIF_SRATIONAL: {
    if (p_exif_data->tag_entry.count > 1 &&
      p_exif_data->tag_entry.data._srats != NULL) {
      free(p_exif_data->tag_entry.data._srats);
      p_exif_data->tag_entry.data._srats = NULL;
    }
  }
  break;
  } /*end of switch*/

  return 0;
}

/** process_sensor_data:
 *
 *  Arguments:
 *   @p_sensor_params : ptr to sensor data
 *
 *  Return     : int32_t type of status
 *               NO_ERROR  -- success
 *              none-zero failure code
 *
 *  Description:
 *       process sensor data
 *
 *  Notes: this needs to be filled for the metadata
 **/
int process_sensor_data(cam_sensor_params_t *p_sensor_params,
  QOMX_EXIF_INFO *exif_info)
{
  int rc = 0;
  rat_t val_rat;

  if (NULL == p_sensor_params) {
    ALOGE("%s %d: Sensor params are null", __func__, __LINE__);
    return 0;
  }

  CDBG("%s:%d] From metadata aperture = %f ", __func__, __LINE__,
    p_sensor_params->aperture_value );

  if (p_sensor_params->aperture_value >= 1.0) {
    double apex_value;
    apex_value = (double)2.0 * log(p_sensor_params->aperture_value) / log(2.0);
    val_rat.num = (uint32_t)(apex_value * 100);
    val_rat.denom = 100;
    rc = addExifEntry(exif_info, EXIFTAGID_APERTURE, EXIF_RATIONAL, 1, &val_rat);
    if (rc) {
      ALOGE("%s:%d]: Error adding Exif Entry", __func__, __LINE__);
    }

    val_rat.num = (uint32_t)(p_sensor_params->aperture_value * 100);
    val_rat.denom = 100;
    rc = addExifEntry(exif_info, EXIFTAGID_F_NUMBER, EXIF_RATIONAL, 1, &val_rat);
    if (rc) {
      ALOGE("%s:%d]: Error adding Exif Entry", __func__, __LINE__);
    }
  }

  /*Flash*/
  short val_short=0;
  int flash_mode_exif, flash_fired;
  if (p_sensor_params->flash_state == CAM_FLASH_STATE_FIRED) {
    flash_fired = 1;
  } else {
    flash_fired = 0;
  }
  CDBG("%s: Flash value %d flash mode %d flash state %d", __func__, val_short,
    p_sensor_params->flash_mode, p_sensor_params->flash_state);

  switch(p_sensor_params->flash_mode) {
  case  CAM_FLASH_MODE_OFF:
    flash_mode_exif = MM_JPEG_EXIF_FLASH_MODE_OFF;
    break;
  case CAM_FLASH_MODE_ON:
    flash_mode_exif = MM_JPEG_EXIF_FLASH_MODE_ON;
    break;
  case CAM_FLASH_MODE_AUTO:
    flash_mode_exif = MM_JPEG_EXIF_FLASH_MODE_AUTO;
    break;
  default:
    flash_mode_exif = MM_JPEG_EXIF_FLASH_MODE_AUTO;
    ALOGE("%s:%d]: Unsupported flash mode", __func__, __LINE__);
  }
  val_short = (short)(flash_fired | (flash_mode_exif << 3));

  rc = addExifEntry(exif_info, EXIFTAGID_FLASH, EXIF_SHORT, 1, &val_short);
  if (rc) {
    ALOGE("%s %d]: Error adding flash exif entry", __func__, __LINE__);
  }
  /* Sensing Method */
  val_short = (short) p_sensor_params->sensing_method;
  rc = addExifEntry(exif_info, EXIFTAGID_SENSING_METHOD, EXIF_SHORT,
    sizeof(val_short)/2, &val_short);
  if (rc) {
    ALOGE("%s:%d]: Error adding flash Exif Entry", __func__, __LINE__);
  }

  /* Focal Length in 35 MM Film */
  val_short = (short)
    ((p_sensor_params->focal_length * p_sensor_params->crop_factor) + 0.5f);
  rc = addExifEntry(exif_info, EXIFTAGID_FOCAL_LENGTH_35MM, EXIF_SHORT,
    1, &val_short);
  if (rc) {
    ALOGE("%s:%d]: Error adding Exif Entry", __func__, __LINE__);
  }

  /* F Number */
  val_rat.num = (uint32_t)(p_sensor_params->f_number * 100);
  val_rat.denom = 100;
  rc = addExifEntry(exif_info, EXIFTAGTYPE_F_NUMBER, EXIF_RATIONAL, 1, &val_rat);
  if (rc) {
    ALOGE("%s:%d]: Error adding Exif Entry", __func__, __LINE__);
  }
  return rc;
}


/** process_3a_data:
 *
 *  Arguments:
 *   @p_3a_params : ptr to 3a data
 *
 *  Return     : int32_t type of status
 *               NO_ERROR  -- success
 *              none-zero failure code
 *
 *  Description:
 *       process 3a data
 *
 *  Notes: this needs to be filled for the metadata
 **/
int process_3a_data(cam_3a_params_t *p_3a_params, QOMX_EXIF_INFO *exif_info)
{
  int rc = 0;
  srat_t val_srat;
  rat_t val_rat;
  double shutter_speed_value;

  if (NULL == p_3a_params) {
    ALOGE("%s %d: 3A params are null", __func__, __LINE__);
    return 0;
  }

  CDBG("%s:%d] exp_time %f, iso_value %d, wb_mode %d", __func__, __LINE__,
    p_3a_params->exp_time, p_3a_params->iso_value, p_3a_params->wb_mode);

  /*Exposure time*/
  if (0.0f >= p_3a_params->exp_time) {
      val_rat.num = 0;
      val_rat.denom = 0;
  } else {
      val_rat.num = 1;
      val_rat.denom = ROUND(1.0/p_3a_params->exp_time);
  }
  CDBG("%s: numer %d denom %d %zd", __func__, val_rat.num, val_rat.denom,
      sizeof(val_rat) / (8));

  rc = addExifEntry(exif_info, EXIFTAGID_EXPOSURE_TIME, EXIF_RATIONAL,
    (sizeof(val_rat)/(8)), &val_rat);
  if (rc) {
    ALOGE("%s:%d]: Error adding Exif Entry Exposure time",
      __func__, __LINE__);
  }

  /* Shutter Speed*/
  if (p_3a_params->exp_time > 0) {
    shutter_speed_value = log10(1/p_3a_params->exp_time)/log10(2);
    val_srat.num = (int32_t)(shutter_speed_value * 1000);
    val_srat.denom = 1000;
  } else {
    val_srat.num = 0;
    val_srat.denom = 0;
  }
  rc = addExifEntry(exif_info, EXIFTAGID_SHUTTER_SPEED, EXIF_SRATIONAL,
    (sizeof(val_srat)/(8)), &val_srat);
  if (rc) {
    ALOGE("%s:%d]: Error adding Exif Entry", __func__, __LINE__);
  }

  /*ISO*/
  short val_short;
  val_short = (short)p_3a_params->iso_value;
  rc = addExifEntry(exif_info, EXIFTAGID_ISO_SPEED_RATING, EXIF_SHORT,
    sizeof(val_short)/2, &val_short);
  if (rc) {
    ALOGE("%s:%d]: Error adding Exif Entry", __func__, __LINE__);
  }

  /*WB mode*/
  if (p_3a_params->wb_mode == CAM_WB_MODE_AUTO)
    val_short = 0;
  else
    val_short = 1;
  rc = addExifEntry(exif_info, EXIFTAGID_WHITE_BALANCE, EXIF_SHORT,
    sizeof(val_short)/2, &val_short);
  if (rc) {
    ALOGE("%s:%d]: Error adding Exif Entry", __func__, __LINE__);
  }

  /* Metering Mode   */
  val_short = (short) p_3a_params->metering_mode;
  rc = addExifEntry(exif_info,EXIFTAGID_METERING_MODE, EXIF_SHORT,
     sizeof(val_short)/2, &val_short);
  if (rc) {
     ALOGE("%s:%d]: Error adding Exif Entry", __func__, __LINE__);
   }

  /*Exposure Program*/
   val_short = (short) p_3a_params->exposure_program;
   rc = addExifEntry(exif_info,EXIFTAGID_EXPOSURE_PROGRAM, EXIF_SHORT,
      sizeof(val_short)/2, &val_short);
   if (rc) {
      ALOGE("%s:%d]: Error adding Exif Entry", __func__, __LINE__);
    }

   /*Exposure Mode */
    val_short = (short) p_3a_params->exposure_mode;
    rc = addExifEntry(exif_info,EXIFTAGID_EXPOSURE_MODE, EXIF_SHORT,
       sizeof(val_short)/2, &val_short);
    if (rc) {
       ALOGE("%s:%d]: Error adding Exif Entry", __func__, __LINE__);
     }

    /*Scenetype*/
     uint8_t val_undef;
     val_undef = (uint8_t) p_3a_params->scenetype;
     rc = addExifEntry(exif_info,EXIFTAGID_SCENE_TYPE, EXIF_UNDEFINED,
        sizeof(val_undef), &val_undef);
     if (rc) {
        ALOGE("%s:%d]: Error adding Exif Entry", __func__, __LINE__);
      }

     CDBG("%s:%d] brightness %f", __func__, __LINE__,
       p_3a_params->brightness);

    /* Brightness Value*/
     val_srat.num = (int32_t) (p_3a_params->brightness * 100.0f);
     val_srat.denom = 100;
     rc = addExifEntry(exif_info,EXIFTAGID_BRIGHTNESS, EXIF_SRATIONAL,
                 (sizeof(val_srat)/(8)), &val_srat);
     if (rc) {
        ALOGE("%s:%d]: Error adding Exif Entry", __func__, __LINE__);
     }

  return rc;
}

/** process_meta_data
 *
 *  Arguments:
 *   @p_meta : ptr to metadata
 *   @exif_info: Exif info struct
 *   @mm_jpeg_exif_params: exif params
 *
 *  Return     : int32_t type of status
 *               NO_ERROR  -- success
 *              none-zero failure code
 *
 *  Description:
 *       Extract exif data from the metadata
 **/
int process_meta_data(metadata_buffer_t *p_meta, QOMX_EXIF_INFO *exif_info,
  mm_jpeg_exif_params_t *p_cam_exif_params, cam_hal_version_t hal_version)
{
  int rc = 0;
  cam_sensor_params_t p_sensor_params;
  cam_3a_params_t p_3a_params;

  memset(&p_3a_params,  0,  sizeof(cam_3a_params_t));
  memset(&p_sensor_params, 0, sizeof(cam_sensor_params_t));

  if (hal_version == CAM_HAL_V1) {
    IF_META_AVAILABLE(cam_3a_params_t, l_3a_params, CAM_INTF_META_AEC_INFO,
        p_meta) {
      p_3a_params = *l_3a_params;
    } else if (p_cam_exif_params) {
      p_3a_params = p_cam_exif_params->cam_3a_params;
    } else {
      p_3a_params.exp_time = 0.0;
      p_3a_params.iso_value = 0;
      p_3a_params.metering_mode = CAM_METERING_MODE_UNKNOWN;
      p_3a_params.exposure_program = 0;
      p_3a_params.exposure_mode = 255;
      p_3a_params.scenetype = 1;
      p_3a_params.brightness = 0.0;
    }

    IF_META_AVAILABLE(int32_t, wb_mode, CAM_INTF_PARM_WHITE_BALANCE, p_meta) {
      p_3a_params.wb_mode = *wb_mode;
    }

    IF_META_AVAILABLE(cam_sensor_params_t, l_sensor_params,
        CAM_INTF_META_SENSOR_INFO, p_meta) {
      p_sensor_params = *l_sensor_params;
    } else if (p_cam_exif_params) {
      p_sensor_params = p_cam_exif_params->sensor_params;
    } else {
      p_sensor_params.focal_length = 0;
      p_sensor_params.f_number = 0;
      p_sensor_params.sensing_method = 2;
      p_sensor_params.crop_factor = 0;
    }
  } else {

    /* Process 3a data */
    IF_META_AVAILABLE(int32_t, iso, CAM_INTF_META_SENSOR_SENSITIVITY, p_meta) {
      p_3a_params.iso_value= *iso;
    } else {
      ALOGE("%s: Cannot extract Iso value", __func__);
    }

    IF_META_AVAILABLE(int64_t, sensor_exposure_time,
        CAM_INTF_META_SENSOR_EXPOSURE_TIME, p_meta) {
      p_3a_params.exp_time =
        (float)((double)(*sensor_exposure_time) / 1000000000.0);
    } else {
      ALOGE("%s: Cannot extract Exp time value", __func__);
    }

    IF_META_AVAILABLE(int32_t, wb_mode, CAM_INTF_PARM_WHITE_BALANCE, p_meta) {
      p_3a_params.wb_mode = *wb_mode;
    } else {
      ALOGE("%s: Cannot extract white balance mode", __func__);
    }

    /* Process sensor data */
    IF_META_AVAILABLE(float, aperture, CAM_INTF_META_LENS_APERTURE, p_meta) {
      p_sensor_params.aperture_value = *aperture;
    } else {
      ALOGE("%s: Cannot extract Aperture value", __func__);
    }

    IF_META_AVAILABLE(uint32_t, flash_mode, CAM_INTF_META_FLASH_MODE, p_meta) {
      p_sensor_params.flash_mode = *flash_mode;
    } else {
      ALOGE("%s: Cannot extract flash mode value", __func__);
    }

    IF_META_AVAILABLE(int32_t, flash_state, CAM_INTF_META_FLASH_STATE, p_meta) {
      p_sensor_params.flash_state = (cam_flash_state_t) *flash_state;
    } else {
      ALOGE("%s: Cannot extract flash state value", __func__);
    }
  }
  if ((hal_version != CAM_HAL_V1) || (p_sensor_params.sens_type != CAM_SENSOR_YUV)) {
    rc = process_3a_data(&p_3a_params, exif_info);
    if (rc) {
      ALOGE("%s %d: Failed to add 3a exif params", __func__, __LINE__);
    }
  }

  rc = process_sensor_data(&p_sensor_params, exif_info);
  if (rc) {
    ALOGE("%s %d: Failed to extract sensor params", __func__, __LINE__);
  }

  if (p_meta) {
    short val_short = 0;

    IF_META_AVAILABLE(cam_auto_scene_t, scene_cap_type,
        CAM_INTF_META_ASD_SCENE_CAPTURE_TYPE, p_meta) {
      val_short = (short) *scene_cap_type;
    }

    rc = addExifEntry(exif_info, EXIFTAGID_SCENE_CAPTURE_TYPE, EXIF_SHORT,
      sizeof(val_short)/2, &val_short);
    if (rc) {
      ALOGE("%s:%d]: Error adding ASD Exif Entry", __func__, __LINE__);
    }
  } else {
    ALOGE("%s:%d]: Error adding ASD Exif Entry, no meta", __func__, __LINE__);
  }
  return rc;
}
