/* Copyright (c) 2015, The Linux Foundation. All rights reserved.
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

#ifndef __QCAMERA_BUFFERMAPS_H__
#define __QCAMERA_BUFFERMAPS_H__

#include "cam_types.h"

namespace qcamera {

class QCameraBufferMaps {
public:
    QCameraBufferMaps();
    QCameraBufferMaps(const QCameraBufferMaps& pBufferMaps);
    QCameraBufferMaps(const cam_buf_map_type_list& pBufMapList);
    QCameraBufferMaps(cam_mapping_buf_type pType,
            uint32_t pStreamId,
            uint32_t pFrameIndex,
            int32_t pPlaneIndex,
            uint32_t pCookie,
            int32_t pFd,
            size_t pSize);

    ~QCameraBufferMaps();

    QCameraBufferMaps& operator=(const QCameraBufferMaps& pBufferMaps);

    uint32_t enqueue(cam_mapping_buf_type pType,
            uint32_t pStreamId,
            uint32_t pFrameIndex,
            int32_t pPlaneIndex,
            uint32_t pCookie,
            int32_t pFd,
            size_t pSize);

    uint32_t getCamBufMapList(cam_buf_map_type_list& pBufMapList) const;

    static uint32_t makeSingletonBufMapList(cam_mapping_buf_type pType,
            uint32_t pStreamId,
            uint32_t pFrameIndex,
            int32_t pPlaneIndex,
            uint32_t pCookie,
            int32_t pFd,
            size_t pSize,
            cam_buf_map_type_list& pBufMapList);

private:
    cam_buf_map_type_list mBufMapList;
};

}; // namespace qcamera
#endif /* __QCAMERA_BUFFERMAPS_H__ */

