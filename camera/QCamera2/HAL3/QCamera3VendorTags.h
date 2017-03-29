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

#ifndef __QCAMERA3VENDORTAGS_H__
#define __QCAMERA3VENDORTAGS_H__

namespace qcamera {

enum qcamera3_ext_section {
    QCAMERA3_PRIVATEDATA = VENDOR_SECTION,
    QCAMERA3_CDS,
    QCAMERA3_OPAQUE_RAW,
    QCAMERA3_CROP,
    QCAMERA3_TUNING_META_DATA,
    QCAMERA3_TEMPORAL_DENOISE,
    QCAMERA3_SECTIONS_END
};

enum qcamera3_ext_section_ranges {
    QCAMERA3_PRIVATEDATA_START = QCAMERA3_PRIVATEDATA << 16,
    QCAMERA3_CDS_START = QCAMERA3_CDS << 16,
    QCAMERA3_OPAQUE_RAW_START = QCAMERA3_OPAQUE_RAW << 16,
    QCAMERA3_CROP_START = QCAMERA3_CROP << 16,
    QCAMERA3_TUNING_META_DATA_START = QCAMERA3_TUNING_META_DATA << 16,
    QCAMERA3_TEMPORAL_DENOISE_START = QCAMERA3_TEMPORAL_DENOISE << 16,
};

enum qcamera3_ext_tags {
    QCAMERA3_PRIVATEDATA_REPROCESS = QCAMERA3_PRIVATEDATA_START,
    QCAMERA3_PRIVATEDATA_END,
    QCAMERA3_CDS_MODE = QCAMERA3_CDS_START,
    QCAMERA3_CDS_END,

    //Property Name:  org.codeaurora.qcamera3.opaque_raw.opaque_raw_strides
    //
    //Type: int32 * n * 3 [public]
    //
    //Description: Distance in bytes from the beginning of one row of opaque
    //raw image data to the beginning of next row.
    //
    //Details: The strides are listed as (raw_width, raw_height, stride)
    //triplets. For each supported raw size, there will be a stride associated
    //with it.
    QCAMERA3_OPAQUE_RAW_STRIDES = QCAMERA3_OPAQUE_RAW_START,

    //Property Name: org.codeaurora.qcamera3.opaque_raw.opaque_raw_format
    //
    //Type: byte(enum) [public]
    //  * LEGACY - The legacy raw format where 8, 10, or 12-bit
    //    raw data is packed into a 64-bit word.
    //  * MIPI - raw format matching the data packing described
    //    in MIPI CSI-2 specification. In memory, the data
    //    is constructed by packing sequentially received pixels
    //    into least significant parts of the words first.
    //    Within each pixel, the least significant bits are also
    //    placed towards the least significant part of the word.
    //
    //Details: Lay out of opaque raw data in memory is decided by two factors:
    //         opaque_raw_format and bit depth (implied by whiteLevel). Below
    //         list illustrates their relationship:
    //  LEGACY8:  P7(7:0) P6(7:0) P5(7:0) P4(7:0) P3(7:0) P2(7:0) P1(7:0) P0(7:0)
    //            8 pixels occupy 8 bytes, no padding needed
    //            min_stride = CEILING8(raw_width)
    // LEGACY10:  0000 P5(9:0) P4(9:0) P3(9:0) P2(9:0) P1(9:0) P0(9:0)
    //            6 pixels occupy 8 bytes, 4 bits padding at MSB
    //            min_stride = (raw_width+5)/6 * 8
    // LEGACY12:  0000 P4(11:0) P3(11:0) P2(11:0) P1(11:0) P0(11:0)
    //            5 pixels occupy 8 bytes, 4 bits padding at MSB
    //            min_stride = (raw_width+4)/5 * 8
    //    MIPI8:  P0(7:0)
    //            1 pixel occupy 1 byte
    //            min_stride = raw_width
    //   MIPI10:  P3(1:0) P2(1:0) P1(1:0) P0(1:0) P3(9:2) P2(9:2) P1(9:2) P0(9:2)
    //            4 pixels occupy 5 bytes
    //            min_stride = (raw_width+3)/4 * 5
    //   MIPI12:  P1(3:0) P0(3:0) P1(11:4) P0(11:4)
    //            2 pixels occupy 3 bytes
    //            min_stride = (raw_width+1)/2 * 3
    //Note that opaque_raw_stride needs to be at least the required minimum
    //stride from the table above. ISP hardware may need more generous stride
    //setting. For example, for LEGACY8, the actual stride may be
    //CEILING16(raw_width) due to bus burst length requirement.
    QCAMERA3_OPAQUE_RAW_FORMAT,
    QCAMERA3_OPAQUE_RAW_END,

    QCAMERA3_CROP_COUNT_REPROCESS = QCAMERA3_CROP_START,
    QCAMERA3_CROP_REPROCESS,
    QCAMERA3_CROP_STREAM_ID_REPROCESS,
    QCAMERA3_CROP_ROI_MAP_REPROCESS,
    QCAMERA3_CROP_END,

    QCAMERA3_TUNING_META_DATA_BLOB = QCAMERA3_TUNING_META_DATA_START,
    QCAMERA3_TUNING_META_DATA_END,

    QCAMERA3_TEMPORAL_DENOISE_ENABLE = QCAMERA3_TEMPORAL_DENOISE_START,
    QCAMERA3_TEMPORAL_DENOISE_PROCESS_TYPE,
    QCAMERA3_TEMPORAL_DENOISE_END
};

// QCAMERA3_OPAQUE_RAW_FORMAT
typedef enum qcamera3_ext_opaque_raw_format {
    QCAMERA3_OPAQUE_RAW_FORMAT_LEGACY,
    QCAMERA3_OPAQUE_RAW_FORMAT_MIPI
} qcamera3_ext_opaque_raw_format_t;

class QCamera3VendorTags {

public:
    static void get_vendor_tag_ops(vendor_tag_ops_t* ops);
    static int get_tag_count(
            const vendor_tag_ops_t *ops);
    static void get_all_tags(
            const vendor_tag_ops_t *ops,
            uint32_t *tag_array);
    static const char* get_section_name(
            const vendor_tag_ops_t *ops,
            uint32_t tag);
    static const char* get_tag_name(
            const vendor_tag_ops_t *ops,
            uint32_t tag);
    static int get_tag_type(
            const vendor_tag_ops_t *ops,
            uint32_t tag);

    static const vendor_tag_ops_t *Ops;
};

}; // namespace qcamera

#endif /* __QCAMERA3VENDORTAGS_H__ */
