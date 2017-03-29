/*Copyright (c) 2015, The Linux Foundation. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of The Linux Foundation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.*/

#ifndef __QMPO_H__
#define __QMPO_H__

#include <stdio.h>
#include <qexif.h>

//Length of MPO header fields
#define MP_APP2_FIELD_LENGTH_BYTES 2
#define MP_FORMAT_IDENTIFIER_BYTES 4
#define MP_ENDIAN_BYTES 4
#define MP_HEADER_OFFSET_TO_FIRST_IFD_BYTES 4
#define MP_INDEX_COUNT_BYTES 2
#define MP_INDEX_VERSION_BYTES 12
#define MP_INDEX_NUMBER_OF_IMAGES_BYTES 12
#define MP_INDEX_ENTRY_BYTES 12
#define MP_INDEX_IMAGE_UNIQUE_ID_LIST_BYTES 12
#define MP_INDEX_TOTAL_CAPURED_FRAMES 12
#define MP_INDEX_OFFSET_OF_NEXT_IFD_BYTES 4
#define MP_INDEX_ENTRY_VALUE_BYTES 16
#define MP_INDEX_ENTRY_INDIVIDUAL_IMAGE_ATTRIBUTE_BYTES 4
#define MP_INDEX_ENTRY_INDIVIDUAL_IMAGE_SIZE_BYTES 4
#define MP_INDEX_ENTRY_INDIVIDUAL_IMAGE_DATA_OFFSET_BYTES 4
#define MP_ATTRIBUTE_COUNT_BYTES 2
#define MP_ATTRIBUTE_OFFSET_OF_NEXT_IFD_BYTES 4
#define MP_TAG_BYTES 12
#define MP_INDIVIDUAL_IMAGE_ID_BYTES 33
#define MP_INDEX_IFD_START 2

#define MPO_BIG_ENDIAN 0x4D4D002A
#define MPO_LITTLE_ENDIAN 0x49492A00

/* MPO Dependent Type */
typedef enum
{
   NON_DEPENDENT_IMAGE    = 0x00000000,   // Non dependent image
   DEPENDENT_CHILD_IMAGE  = 0x40000000,   // Dependent child image flag
   DEPENDENT_PARENT_IMAGE = 0x80000000,   // Dependent parent image flag
   DEPENDENT_MASK         = 0xc0000000,   // Dependent mask
   DEPENDENT_MAX,
} qmpo_dependent_t;

/* MPO Representative Type */
typedef enum
{
  NOT_REPRESENTATIVE_IMAGE = 0x00000000,   // Not a representative image
  REPRESENTATIVE_IMAGE     = 0x20000000,   // Representative image flag
  REPRESENTATIVE_MASK      = 0x20000000,   // Representative mask
  REPRESENTATIVE_MAX,
} qmpo_representative_t;

/* MPO Image Data Format Type */
typedef enum
{
  JPEG                   = 0x00000000,   // Image is in JPEG format
  NON_JPEG               = 0x07000000,   // Image is not JPEG
  IMAGE_DATA_FORMAT_MASK = 0x07000000,   // Image mask
  IMAGE_DATA_FORMAT_MAX,
} qmpo_image_data_format_t;

/* MPO Type */
typedef enum
{
  UNDEFINED              = 0x00000000,   // MP types undefined
  LARGE_TN_CLASS_1       = 0x00010001,   // Large thumbnail class 1 image
  LARGE_TN_CLASS_2       = 0x00010002,   // Large thumbnail class 2 image
  MULTI_VIEW_PANORAMA    = 0x00020001,   // Multi-view Panorama image
  MULTI_VIEW_DISPARITY   = 0x00020002,   // Multi-view Disparity image
  MULTI_VIEW_MULTI_ANGLE = 0x00020003,   // Multi-view Multi-angle image
  BASELINE_PRIMARY       = 0x00030000,   // Baseline MP Primary image
  TYPE_MASK              = 0x00ffffff,   // Type mask
  TYPE_MAX,
} qmpo_type_t;

// MP Format Version
// Use MPOTAGTYPE_MP_F_VERSION as the exif_tag_type (EXIF_UNDEFINED)
// Count should be 4
#define _ID_MP_F_VERSION_FIRST           0xb000
#define MPOTAGID_MP_F_VERSION_FIRST      CONSTRUCT_TAGID(MP_F_VERSION_FIRST, _ID_MP_F_VERSION_FIRST)
#define MPOTAGTYPE_MP_F_VERSION_FIRST    EXIF_UNDEFINED

// Number of Images
// Use MPOTAGTYPE_NUMBER_OF_IMAGES as the exif_tag_type (EXIF_LONG)
// Count should be 1
#define _ID_NUMBER_OF_IMAGES             0xb001
#define MPOTAGID_NUMBER_OF_IMAGES        CONSTRUCT_TAGID(NUMBER_OF_IMAGES, _ID_NUMBER_OF_IMAGES)
#define MPOTAGTYPE_NUMBER_OF_IMAGES      EXIF_LONG

// MP Entry
// Use MPOTAGTYPE_MP_ENTRY as the exif_tag_type (EXIF_UNDEFINED)
// Count should be 16 x NumberOfImages
#define _ID_MP_ENTRY                     0xb002
#define MPOTAGID_MP_ENTRY                CONSTRUCT_TAGID(MP_ENTRY, _ID_MP_ENTRY)
#define MPOTAGTYPE_MP_ENTRY              EXIF_UNDEFINED

// Individual Image Unique ID List
// Use MPOTAGTYPE_IMAGE_UID_LIST as the exif_tag_type (EXIF_UNDEFINED)
// Count should be 33 x NumberOfImages
#define _ID_IMAGE_UID_LIST               0xb003
#define MPOTAGID_IMAGE_UID_LIST          CONSTRUCT_TAGID(IMAGE_UID_LIST, _ID_IMAGE_UID_LIST)
#define MPOTAGTYPE_IMAGE_UID_LIST        EXIF_UNDEFINED

// Total Number of Camptured Frames
// Use MPOTAGTYPE_TOTAL_FRAMES as the exif_tag_type (EXIF_LONG)
// Count should be 1
#define _ID_TOTAL_FRAMES                 0xb004
#define MPOTAGID_TOTAL_FRAMES            CONSTRUCT_TAGID(TOTAL_FRAMES, _ID_TOTAL_FRAMES)
#define MPOTAGTYPE_TOTAL_FRAMES          EXIF_LONG

// MP Format Version
// Use MPOTAGTYPE_MP_F_VERSION as the exif_tag_type (EXIF_UNDEFINED)
// Count should be 4
#define _ID_MP_F_VERSION                 0xb000
#define MPOTAGID_MP_F_VERSION            CONSTRUCT_TAGID(MP_F_VERSION, _ID_MP_F_VERSION)
#define MPOTAGTYPE_MP_F_VERSION          EXIF_UNDEFINED

// MP Individual Image Number
// Use MPOTAGTYPE_MP_INDIVIDUAL_NUM as the exif_tag_type (EXIF_LONG)
// Count should be 1
#define _ID_MP_INDIVIDUAL_NUM            0xb101
#define MPOTAGID_MP_INDIVIDUAL_NUM       CONSTRUCT_TAGID(MP_INDIVIDUAL_NUM, _ID_MP_INDIVIDUAL_NUM)
#define MPOTAGTYPE_MP_INDIVIDUAL_NUM     EXIF_LONG

#endif
