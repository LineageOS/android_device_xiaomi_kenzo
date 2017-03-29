/* Copyright (c) 2016, The Linux Foundation. All rights reserved.
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


/* Macros exposed to gralloc to query camera HAL for gralloc format to be
used for vedor specific camera formats. */

#define PREFERRED_IMPLEMENTATION_DEFINED_CAMERA_FORMAT HAL_PIXEL_FORMAT_YCrCb_420_SP
#define PREFERRED_YCBCR_420_888_CAMERA_FORMAT HAL_PIXEL_FORMAT_YCrCb_420_SP

/* Macros exposed to camera HAL to get the preview and callback stream
formats. Please ensure that if the macros below are changed then the
corresponding change should be done in the above macros and vice versa
to prevent format mismatch between Gralloc and Camera HAL for stream
buffers */

#define PREVIEW_STREAM_FORMAT CAM_FORMAT_YUV_420_NV21
#define CALLBACK_STREAM_FORMAT CAM_FORMAT_YUV_420_NV21
