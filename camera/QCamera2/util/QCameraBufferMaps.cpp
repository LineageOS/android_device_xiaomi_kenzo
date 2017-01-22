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

#define LOG_TAG "QCameraBufferMaps"

#include <utils/Errors.h>
#include <utils/Log.h>
#include <string.h>
#include "QCameraBufferMaps.h"

using namespace android;

namespace qcamera {

/*===========================================================================
 * FUNCTION   : QCameraBufferMaps
 *
 * DESCRIPTION: default constructor of QCameraBufferMaps
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *==========================================================================*/
QCameraBufferMaps::QCameraBufferMaps()
{
    memset(&mBufMapList, 0, sizeof(mBufMapList));
}

/*===========================================================================
 * FUNCTION   : QCameraBufferMaps
 *
 * DESCRIPTION: copy constructor of QCameraBufferMaps
 *
 * PARAMETERS :
 *   @pBufferMaps : object to be copied
 *
 * RETURN     : None
 *==========================================================================*/
QCameraBufferMaps::QCameraBufferMaps(const QCameraBufferMaps& pBufferMaps)
{
    memcpy(&mBufMapList, &pBufferMaps.mBufMapList, sizeof(mBufMapList));
}

/*===========================================================================
 * FUNCTION   : QCameraBufferMaps
 *
 * DESCRIPTION: constructor of QCameraBufferMaps
 *
 * PARAMETERS :
 *   @pBufMapList : list of buffer maps
 *
 * RETURN     : None
 *==========================================================================*/
QCameraBufferMaps::QCameraBufferMaps(const cam_buf_map_type_list& pBufMapList)
{
    memcpy(&mBufMapList, &pBufMapList, sizeof(mBufMapList));
}

/*===========================================================================
 * FUNCTION   : QCameraBufferMaps
 *
 * DESCRIPTION: constructor of QCameraBufferMaps
 *
 * PARAMETERS :
 *   @pType   : Type of buffer
 *   @pStreamId : Stream id
 *   @pFrameIndex : Frame index
 *   @pPlaneIndex : Plane index
 *   @pCookie   : Could be job_id to identify mapping job
 *   @pFd   : Origin file descriptor
 *   @pSize   : Size of the buffer
 *
 * RETURN     : None
 *==========================================================================*/
QCameraBufferMaps::QCameraBufferMaps(cam_mapping_buf_type pType,
        uint32_t pStreamId,
        uint32_t pFrameIndex,
        int32_t pPlaneIndex,
        uint32_t pCookie,
        int32_t pFd,
        size_t pSize)
{
    memset(&mBufMapList, 0, sizeof(mBufMapList));
    enqueue(pType, pStreamId, pFrameIndex, pPlaneIndex, pCookie, pFd, pSize);
}

/*===========================================================================
 * FUNCTION   : ~QCameraBufferMaps
 *
 * DESCRIPTION: destructor of QCameraBufferMaps
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *==========================================================================*/
QCameraBufferMaps::~QCameraBufferMaps()
{
}

/*===========================================================================
 * FUNCTION   : operator=
 *
 * DESCRIPTION: assignment operator of QCameraBufferMaps
 *
 * PARAMETERS :
 *   @pBufferMaps : object to be copied
 *
 * RETURN     : *this, with updated contents
 *==========================================================================*/
QCameraBufferMaps& QCameraBufferMaps::operator=(const QCameraBufferMaps& pBufferMaps)
{
    if (&pBufferMaps != this) {
        memcpy(&mBufMapList, &pBufferMaps.mBufMapList, sizeof(mBufMapList));
    }
    return *this;
}

/*===========================================================================
 * FUNCTION   : enqueue
 *
 * DESCRIPTION: Add a buffer map
 *
 * PARAMETERS :
 *   @pType   : Type of buffer
 *   @pStreamId : Stream id
 *   @pFrameIndex : Frame index
 *   @pPlaneIndex : Plane index
 *   @pCookie   : Could be job_id to identify mapping job
 *   @pFd   : Origin file descriptor
 *   @pSize   : Size of the buffer
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
uint32_t QCameraBufferMaps::enqueue(cam_mapping_buf_type pType,
        uint32_t pStreamId,
        uint32_t pFrameIndex,
        int32_t pPlaneIndex,
        uint32_t pCookie,
        int32_t pFd,
        size_t pSize)
{
    uint32_t pos = mBufMapList.length++;
    mBufMapList.buf_maps[pos].type = pType;
    mBufMapList.buf_maps[pos].stream_id = pStreamId;
    mBufMapList.buf_maps[pos].frame_idx = pFrameIndex;
    mBufMapList.buf_maps[pos].plane_idx = pPlaneIndex;
    mBufMapList.buf_maps[pos].cookie = pCookie;
    mBufMapList.buf_maps[pos].fd = pFd;
    mBufMapList.buf_maps[pos].size = pSize;

    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : getCamBufMapList
 *
 * DESCRIPTION: Populate the list
 *
 * PARAMETERS :
 *   @pBufMapList : [output] the list of buffer maps
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
uint32_t QCameraBufferMaps::getCamBufMapList(cam_buf_map_type_list& pBufMapList) const
{
    memcpy(&pBufMapList, &mBufMapList, sizeof(pBufMapList));

    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : makeSingletonBufMapList
 *
 * DESCRIPTION: Create a buffer map list of a single element
 *
 * PARAMETERS :
 *   @pType   : Type of buffer
 *   @pStreamId : Stream id
 *   @pFrameIndex : Frame index
 *   @pPlaneIndex : Plane index
 *   @pCookie   : Could be job_id to identify mapping job
 *   @pFd   : Origin file descriptor
 *   @pSize   : Size of the buffer
 *   @pBufMapList : [output] the list of buffer maps
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
uint32_t QCameraBufferMaps::makeSingletonBufMapList(cam_mapping_buf_type pType,
        uint32_t pStreamId,
        uint32_t pFrameIndex,
        int32_t pPlaneIndex,
        uint32_t pCookie,
        int32_t pFd,
        size_t pSize,
        cam_buf_map_type_list& pBufMapList)
{
    uint32_t rc = NO_ERROR;

    QCameraBufferMaps bufferMaps(pType,
            pStreamId,
            pFrameIndex,
            pPlaneIndex,
            pCookie,
            pFd,
            pSize);
    rc = bufferMaps.getCamBufMapList(pBufMapList);

    return rc;
}

}; // namespace qcamera
