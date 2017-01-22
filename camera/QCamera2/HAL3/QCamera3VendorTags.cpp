/* Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
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

#define LOG_TAG "QCamera3VendorTags"
//#define LOG_NDEBUG 0

#include <hardware/camera3.h>
#include <utils/Log.h>
#include <utils/Errors.h>
#include "QCamera3HWI.h"
#include "QCamera3VendorTags.h"

using namespace android;

namespace qcamera {

const int QCAMERA3_SECTION_COUNT = QCAMERA3_SECTIONS_END - VENDOR_SECTION;

enum qcamera3_ext_tags qcamera3_ext3_section_bounds[QCAMERA3_SECTIONS_END -
    VENDOR_SECTION] = {
        QCAMERA3_PRIVATEDATA_END,
        QCAMERA3_CDS_END,
        QCAMERA3_OPAQUE_RAW_END,
        QCAMERA3_CROP_END,
        QCAMERA3_TUNING_META_DATA_END,
        QCAMERA3_TEMPORAL_DENOISE_END
} ;

typedef struct vendor_tag_info {
    const char *tag_name;
    uint8_t     tag_type;
} vendor_tag_info_t;

const char *qcamera3_ext_section_names[QCAMERA3_SECTIONS_END -
        VENDOR_SECTION] = {
    "org.codeaurora.qcamera3.privatedata",
    "org.codeaurora.qcamera3.CDS",
    "org.codeaurora.qcamera3.opaque_raw",
    "org.codeaurora.qcamera3.crop",
    "org.codeaurora.qcamera3.tuning_meta_data",
    "org.codeaurora.qcamera3.temporal_denoise"
};

vendor_tag_info_t qcamera3_privatedata[QCAMERA3_PRIVATEDATA_END - QCAMERA3_PRIVATEDATA_START] = {
    { "privatedata_reprocess", TYPE_INT32 }
};

vendor_tag_info_t qcamera3_cds[QCAMERA3_CDS_END - QCAMERA3_CDS_START] = {
    { "cds_mode", TYPE_INT32 }
};

vendor_tag_info_t qcamera3_opaque_raw[QCAMERA3_OPAQUE_RAW_END -
        QCAMERA3_OPAQUE_RAW_START] = {
    { "opaque_raw_strides", TYPE_INT32 },
    { "opaque_raw_format", TYPE_BYTE }
};

vendor_tag_info_t qcamera3_crop[QCAMERA3_CROP_END- QCAMERA3_CROP_START] = {
    { "count", TYPE_INT32 },
    { "data", TYPE_INT32},
    { "streamids", TYPE_INT32},
    { "roimap", TYPE_INT32 }
};

vendor_tag_info_t qcamera3_tuning_meta_data[QCAMERA3_TUNING_META_DATA_END -
        QCAMERA3_TUNING_META_DATA_START] = {
    { "tuning_meta_data_blob", TYPE_INT32 }
};

vendor_tag_info_t qcamera3_temporal_denoise[QCAMERA3_TEMPORAL_DENOISE_END -
        QCAMERA3_TEMPORAL_DENOISE_START] = {
    { "enable", TYPE_BYTE },
    { "process_type", TYPE_INT32 }
};

vendor_tag_info_t *qcamera3_tag_info[QCAMERA3_SECTIONS_END -
        VENDOR_SECTION] = {
    qcamera3_privatedata,
    qcamera3_cds,
    qcamera3_opaque_raw,
    qcamera3_crop,
    qcamera3_tuning_meta_data,
    qcamera3_temporal_denoise
};

uint32_t qcamera3_all_tags[] = {
    // QCAMERA3_PRIVATEDATA
    (uint32_t)QCAMERA3_PRIVATEDATA_REPROCESS,

    // QCAMERA3_CDS
    (uint32_t)QCAMERA3_CDS_MODE,

    // QCAMERA3_OPAQUE_RAW
    (uint32_t)QCAMERA3_OPAQUE_RAW_STRIDES,
    (uint32_t)QCAMERA3_OPAQUE_RAW_FORMAT,

    // QCAMERA3_CROP
    (uint32_t)QCAMERA3_CROP_COUNT_REPROCESS,
    (uint32_t)QCAMERA3_CROP_REPROCESS,
    (uint32_t)QCAMERA3_CROP_STREAM_ID_REPROCESS,
    (uint32_t)QCAMERA3_CROP_ROI_MAP_REPROCESS,

    // QCAMERA3_TUNING_META_DATA
    (uint32_t)QCAMERA3_TUNING_META_DATA_BLOB,

    // QCAMERA3_TEMPORAL_DENOISE
    (uint32_t)QCAMERA3_TEMPORAL_DENOISE_ENABLE,
    (uint32_t)QCAMERA3_TEMPORAL_DENOISE_PROCESS_TYPE
};

const vendor_tag_ops_t* QCamera3VendorTags::Ops = NULL;

/*===========================================================================
 * FUNCTION   : get_vendor_tag_ops
 *
 * DESCRIPTION: Get the metadata vendor tag function pointers
 *
 * PARAMETERS :
 *    @ops   : function pointer table to be filled by HAL
 *
 *
 * RETURN     : NONE
 *==========================================================================*/
void QCamera3VendorTags::get_vendor_tag_ops(
                                vendor_tag_ops_t* ops)
{
    ALOGV("%s: E", __func__);

    Ops = ops;

    ops->get_tag_count = get_tag_count;
    ops->get_all_tags = get_all_tags;
    ops->get_section_name = get_section_name;
    ops->get_tag_name = get_tag_name;
    ops->get_tag_type = get_tag_type;
    ops->reserved[0] = NULL;

    ALOGV("%s: X", __func__);
    return;
}

/*===========================================================================
 * FUNCTION   : get_tag_count
 *
 * DESCRIPTION: Get number of vendor tags supported
 *
 * PARAMETERS :
 *    @ops   :  Vendor tag ops data structure
 *
 *
 * RETURN     : Number of vendor tags supported
 *==========================================================================*/

int QCamera3VendorTags::get_tag_count(
                const vendor_tag_ops_t * ops)
{
    size_t count = 0;
    if (ops == Ops)
        count = sizeof(qcamera3_all_tags)/sizeof(qcamera3_all_tags[0]);

    ALOGV("%s: count is %d", __func__, count);
    return (int)count;
}

/*===========================================================================
 * FUNCTION   : get_all_tags
 *
 * DESCRIPTION: Fill array with all supported vendor tags
 *
 * PARAMETERS :
 *    @ops      :  Vendor tag ops data structure
 *    @tag_array:  array of metadata tags
 *
 * RETURN     : Success: the section name of the specific tag
 *              Failure: NULL
 *==========================================================================*/
void QCamera3VendorTags::get_all_tags(
                const vendor_tag_ops_t * ops,
                uint32_t *g_array)
{
    if (ops != Ops)
        return;

    for (size_t i = 0;
            i < sizeof(qcamera3_all_tags)/sizeof(qcamera3_all_tags[0]);
            i++) {
        g_array[i] = qcamera3_all_tags[i];
        CDBG("%s: g_array[%d] is %d", __func__, i, g_array[i]);
    }
}

/*===========================================================================
 * FUNCTION   : get_section_name
 *
 * DESCRIPTION: Get section name for vendor tag
 *
 * PARAMETERS :
 *    @ops   :  Vendor tag ops structure
 *    @tag   :  Vendor specific tag
 *
 *
 * RETURN     : Success: the section name of the specific tag
 *              Failure: NULL
 *==========================================================================*/

const char* QCamera3VendorTags::get_section_name(
                const vendor_tag_ops_t * ops,
                uint32_t tag)
{
    ALOGV("%s: E", __func__);
    if (ops != Ops)
        return NULL;

    const char *ret;
    uint32_t section = tag >> 16;

    if (section < VENDOR_SECTION || section >= QCAMERA3_SECTIONS_END)
        ret = NULL;
    else
        ret = qcamera3_ext_section_names[section - VENDOR_SECTION];

    if (ret)
        ALOGV("%s: section_name[%d] is %s", __func__, tag, ret);
    ALOGV("%s: X", __func__);
    return ret;
}

/*===========================================================================
 * FUNCTION   : get_tag_name
 *
 * DESCRIPTION: Get name of a vendor specific tag
 *
 * PARAMETERS :
 *    @tag   :  Vendor specific tag
 *
 *
 * RETURN     : Success: the name of the specific tag
 *              Failure: NULL
 *==========================================================================*/
const char* QCamera3VendorTags::get_tag_name(
                const vendor_tag_ops_t * ops,
                uint32_t tag)
{
    ALOGV("%s: E", __func__);
    const char *ret;
    uint32_t section = tag >> 16;
    uint32_t section_index = section - VENDOR_SECTION;
    uint32_t tag_index = tag & 0xFFFF;

    if (ops != Ops) {
        ret = NULL;
        goto done;
    }

    if (section < VENDOR_SECTION || section >= QCAMERA3_SECTIONS_END)
        ret = NULL;
    else if (tag >= (uint32_t)qcamera3_ext3_section_bounds[section_index])
        ret = NULL;
    else
        ret = qcamera3_tag_info[section_index][tag_index].tag_name;

    if (ret)
        ALOGV("%s: tag name for tag %d is %s", __func__, tag, ret);
    ALOGV("%s: X", __func__);

done:
    return ret;
}

/*===========================================================================
 * FUNCTION   : get_tag_type
 *
 * DESCRIPTION: Get type of a vendor specific tag
 *
 * PARAMETERS :
 *    @tag   :  Vendor specific tag
 *
 *
 * RETURN     : Success: the type of the specific tag
 *              Failure: -1
 *==========================================================================*/
int QCamera3VendorTags::get_tag_type(
                const vendor_tag_ops_t *ops,
                uint32_t tag)
{
    ALOGV("%s: E", __func__);
    int ret;
    uint32_t section = tag >> 16;
    uint32_t section_index = section - VENDOR_SECTION;
    uint32_t tag_index = tag & 0xFFFF;

    if (ops != Ops) {
        ret = -1;
        goto done;
    }
    if (section < VENDOR_SECTION || section >= QCAMERA3_SECTIONS_END)
        ret = -1;
    else if (tag >= (uint32_t )qcamera3_ext3_section_bounds[section_index])
        ret = -1;
    else
        ret = qcamera3_tag_info[section_index][tag_index].tag_type;

    ALOGV("%s: tag type for tag %d is %d", __func__, tag, ret);
    ALOGV("%s: X", __func__);
done:
    return ret;
}

}; //end namespace qcamera
