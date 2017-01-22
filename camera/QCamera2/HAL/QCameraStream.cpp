/* Copyright (c) 2012-2015, The Linux Foundataion. All rights reserved.
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

#define LOG_TAG "QCameraStream"

#include <utils/Errors.h>
#include <QComOMXMetadata.h>
#include "QCameraBufferMaps.h"
#include "QCamera2HWI.h"
#include "QCameraStream.h"

#define CAMERA_MIN_ALLOCATED_BUFFERS     3

namespace qcamera {

/*===========================================================================
 * FUNCTION   : get_bufs
 *
 * DESCRIPTION: static function entry to allocate stream buffers
 *
 * PARAMETERS :
 *   @offset     : offset info of stream buffers
 *   @num_bufs   : number of buffers allocated
 *   @initial_reg_flag: flag to indicate if buffer needs to be registered
 *                      at kernel initially
 *   @bufs       : output of allocated buffers
 *   @ops_tbl    : ptr to buf mapping/unmapping ops
 *   @user_data  : user data ptr of ops_tbl
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::get_bufs(
                     cam_frame_len_offset_t *offset,
                     uint8_t *num_bufs,
                     uint8_t **initial_reg_flag,
                     mm_camera_buf_def_t **bufs,
                     mm_camera_map_unmap_ops_tbl_t *ops_tbl,
                     void *user_data)
{
    QCameraStream *stream = reinterpret_cast<QCameraStream *>(user_data);
    if (!stream) {
        ALOGE("getBufs invalid stream pointer");
        return NO_MEMORY;
    }

    if (stream->mStreamInfo != NULL
            && stream->mStreamInfo->streaming_mode == CAM_STREAMING_MODE_BATCH) {
        //Batch Mode. Allocate Butch buffers
        return stream->allocateBatchBufs(offset, num_bufs,
                initial_reg_flag, bufs, ops_tbl);
    } else {
        // Plane Buffer. Allocate plane buffer
        return stream->getBufs(offset, num_bufs,
                initial_reg_flag, bufs, ops_tbl);
    }
}

/*===========================================================================
 * FUNCTION   : get_bufs_deffered
 *
 * DESCRIPTION: static function entry to allocate deffered stream buffers
 *
 * PARAMETERS :
 *   @offset     : offset info of stream buffers
 *   @num_bufs   : number of buffers allocated
 *   @initial_reg_flag: flag to indicate if buffer needs to be registered
 *                      at kernel initially
 *   @bufs       : output of allocated buffers
 *   @ops_tbl    : ptr to buf mapping/unmapping ops
 *   @user_data  : user data ptr of ops_tbl
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::get_bufs_deffered(
        cam_frame_len_offset_t * /* offset */,
        uint8_t *num_bufs,
        uint8_t **initial_reg_flag,
        mm_camera_buf_def_t **bufs,
        mm_camera_map_unmap_ops_tbl_t * ops_tbl,
        void *user_data)
{
    QCameraStream *stream = reinterpret_cast<QCameraStream *>(user_data);

    if (!stream) {
        ALOGE("getBufs invalid stream pointer");
        return NO_MEMORY;
    }

    return stream->getBufsDeferred(NULL /*offset*/, num_bufs, initial_reg_flag, bufs,
            ops_tbl);
}

/*===========================================================================
 * FUNCTION   : put_bufs
 *
 * DESCRIPTION: static function entry to deallocate stream buffers
 *
 * PARAMETERS :
 *   @ops_tbl    : ptr to buf mapping/unmapping ops
 *   @user_data  : user data ptr of ops_tbl
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::put_bufs(
        mm_camera_map_unmap_ops_tbl_t *ops_tbl,
        void *user_data)
{
    QCameraStream *stream = reinterpret_cast<QCameraStream *>(user_data);
    if (!stream) {
        ALOGE("putBufs invalid stream pointer");
        return NO_MEMORY;
    }

    if (stream->mStreamInfo != NULL
            && stream->mStreamInfo->streaming_mode == CAM_STREAMING_MODE_BATCH) {
        //Batch Mode. release  Butch buffers
        return stream->releaseBatchBufs(ops_tbl);
    } else {
        // Plane Buffer. release  plane buffer
        return stream->putBufs(ops_tbl);
    }

}

/*===========================================================================
 * FUNCTION   : put_bufs_deffered
 *
 * DESCRIPTION: static function entry to deallocate deffered stream buffers
 *
 * PARAMETERS :
 *   @ops_tbl    : ptr to buf mapping/unmapping ops
 *   @user_data  : user data ptr of ops_tbl
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::put_bufs_deffered(
        mm_camera_map_unmap_ops_tbl_t * /*ops_tbl */,
        void * /*user_data*/ )
{
    // No op
    // Used for handling buffers with deffered allocation. They are freed separately.
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : invalidate_buf
 *
 * DESCRIPTION: static function entry to invalidate a specific stream buffer
 *
 * PARAMETERS :
 *   @index      : index of the stream buffer to invalidate
 *   @user_data  : user data ptr of ops_tbl
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::invalidate_buf(uint32_t index, void *user_data)
{
    QCameraStream *stream = reinterpret_cast<QCameraStream *>(user_data);
    if (!stream) {
        ALOGE("invalid stream pointer");
        return NO_MEMORY;
    }

    if (stream->mStreamInfo->is_secure == SECURE){
        return 0;
    }

    if (stream->mStreamInfo->streaming_mode == CAM_STREAMING_MODE_BATCH) {
        for (int i = 0; i < stream->mBufDefs[index].user_buf.bufs_used; i++) {
            uint32_t buf_idx = stream->mBufDefs[index].user_buf.buf_idx[i];
            stream->invalidateBuf(buf_idx);
        }
    } else {
        return stream->invalidateBuf(index);
    }

    return 0;
}

/*===========================================================================
 * FUNCTION   : clean_invalidate_buf
 *
 * DESCRIPTION: static function entry to clean invalidate a specific stream buffer
 *
 * PARAMETERS :
 *   @index      : index of the stream buffer to clean invalidate
 *   @user_data  : user data ptr of ops_tbl
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::clean_invalidate_buf(uint32_t index, void *user_data)
{
    QCameraStream *stream = reinterpret_cast<QCameraStream *>(user_data);
    if (!stream) {
        ALOGE("invalid stream pointer");
        return NO_MEMORY;
    }

    if (stream->mStreamInfo->is_secure == SECURE){
        return 0;
    }

    if (stream->mStreamInfo->streaming_mode == CAM_STREAMING_MODE_BATCH) {
        for (int i = 0; i < stream->mBufDefs[index].user_buf.bufs_used; i++) {
            uint32_t buf_idx = stream->mBufDefs[index].user_buf.buf_idx[i];
            stream->cleanInvalidateBuf(buf_idx);
        }
    } else {
        return stream->cleanInvalidateBuf(index);
    }

    return 0;
}

/*===========================================================================
 * FUNCTION   : set_config_ops
 *
 * DESCRIPTION: static function update mm-interface ops functions
 *
 * PARAMETERS :
 *   @ops_tbl    : ptr to buf mapping/unmapping ops
 *   @user_data  : user data ptr of ops_tbl
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::set_config_ops(mm_camera_map_unmap_ops_tbl_t *ops_tbl,
        void *user_data)
{
    QCameraStream *stream = reinterpret_cast<QCameraStream *>(user_data);
    if (!stream) {
        ALOGE("%s: Stream invalid", __func__);
        return NO_MEMORY;
    }

    stream->m_MemOpsTbl = *ops_tbl;
    return 0;
}

/*===========================================================================
 * FUNCTION   : QCameraStream
 *
 * DESCRIPTION: constructor of QCameraStream
 *
 * PARAMETERS :
 *   @allocator  : memory allocator obj
 *   @camHandle  : camera handle
 *   @chId       : channel handle
 *   @camOps     : ptr to camera ops table
 *   @paddingInfo: ptr to padding info
 *   @deffered   : deferred stream
 *   @online_rotation: rotation applied online
 *
 * RETURN     : None
 *==========================================================================*/
QCameraStream::QCameraStream(QCameraAllocator &allocator,
        uint32_t camHandle, uint32_t chId,
        mm_camera_ops_t *camOps, cam_padding_info_t *paddingInfo,
        bool deffered, cam_rotation_t online_rotation):
        mDumpFrame(0),
        mDumpMetaFrame(0),
        mDumpSkipCnt(0),
        mStreamTimestamp(0),
        mCamHandle(camHandle),
        mChannelHandle(chId),
        mHandle(0),
        mCamOps(camOps),
        mStreamInfo(NULL),
        mNumBufs(0),
        mNumPlaneBufs(0),
        mNumBufsNeedAlloc(0),
        mRegFlags(NULL),
        mDataCB(NULL),
        mSYNCDataCB(NULL),
        mUserData(NULL),
        mDataQ(releaseFrameData, this),
        mStreamInfoBuf(NULL),
        mMiscBuf(NULL),
        mStreamBufs(NULL),
        mStreamBatchBufs(NULL),
        mAllocator(allocator),
        mBufDefs(NULL),
        mPlaneBufDefs(NULL),
        mOnlineRotation(online_rotation),
        mStreamBufsAcquired(false),
        m_bActive(false),
        mDynBufAlloc(false),
        mBufAllocPid(0),
        mDefferedAllocation(deffered),
        wait_for_cond(false),
        mAllocTaskId(0),
        mMapTaskId(0)
{
    mMemVtbl.user_data = this;
    if ( !deffered ) {
        mMemVtbl.get_bufs = get_bufs;
        mMemVtbl.put_bufs = put_bufs;
    } else {
        mMemVtbl.get_bufs = get_bufs_deffered;
        mMemVtbl.put_bufs = put_bufs_deffered;
    }
    mMemVtbl.invalidate_buf = invalidate_buf;
    mMemVtbl.clean_invalidate_buf = clean_invalidate_buf;
    mMemVtbl.set_config_ops = set_config_ops;
    memset(&mFrameLenOffset, 0, sizeof(mFrameLenOffset));
    memcpy(&mPaddingInfo, paddingInfo, sizeof(cam_padding_info_t));
    memset(&mCropInfo, 0, sizeof(cam_rect_t));
    memset(&m_MemOpsTbl, 0, sizeof(mm_camera_map_unmap_ops_tbl_t));
    memset(&m_OutputCrop, 0, sizeof(cam_stream_parm_buffer_t));
    memset(&m_ImgProp, 0, sizeof(cam_stream_parm_buffer_t));
    memset(&mAllocTask, 0, sizeof(mAllocTask));
    memset(&mMapTask, 0, sizeof(mMapTask));
    pthread_mutex_init(&mCropLock, NULL);
    pthread_mutex_init(&mParameterLock, NULL);
    pthread_mutex_init(&m_lock, NULL);
    pthread_cond_init(&m_cond, NULL);
}

/*===========================================================================
 * FUNCTION   : ~QCameraStream
 *
 * DESCRIPTION: deconstructor of QCameraStream
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *==========================================================================*/
QCameraStream::~QCameraStream()
{
    pthread_mutex_destroy(&mCropLock);
    pthread_mutex_destroy(&mParameterLock);

    mAllocator.waitForBackgroundTask(mAllocTaskId);
    mAllocator.waitForBackgroundTask(mMapTaskId);

    if (mDefferedAllocation) {
        mStreamBufsAcquired = false;
        releaseBuffs();
    }

    unmapStreamInfoBuf();
    releaseStreamInfoBuf();

    if (mMiscBuf) {
        unMapBuf(mMiscBuf, CAM_MAPPING_BUF_TYPE_MISC_BUF, NULL);
        releaseMiscBuf();
    }

    // delete stream
    if (mHandle > 0) {
        mCamOps->delete_stream(mCamHandle, mChannelHandle, mHandle);
        mHandle = 0;
    }
    pthread_mutex_destroy(&m_lock);
    pthread_cond_destroy(&m_cond);
}

/*===========================================================================
 * FUNCTION   : unmapStreamInfoBuf
 *
 * DESCRIPTION: Unmap stream info buffer
 *
 * PARAMETERS :
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::unmapStreamInfoBuf()
{
    int rc = NO_ERROR;

    if (mStreamInfoBuf != NULL) {
        rc = mCamOps->unmap_stream_buf(mCamHandle,
            mChannelHandle,
            mHandle,
            CAM_MAPPING_BUF_TYPE_STREAM_INFO,
            0,
            -1);

        if (rc < 0) {
            ALOGE("Failed to unmap stream info buffer");
        }
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : releaseMiscBuf
 *
 * DESCRIPTION: Release misc buffers
 *
 * PARAMETERS :
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::releaseMiscBuf()
{
    int rc = NO_ERROR;

    if (mMiscBuf != NULL) {
        mMiscBuf->deallocate();
        delete mMiscBuf;
        mMiscBuf = NULL;
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : releaseStreamInfoBuf
 *
 * DESCRIPTION: Release stream info buffer
 *
 * PARAMETERS :
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::releaseStreamInfoBuf()
{
    int rc = NO_ERROR;

    if (mStreamInfoBuf != NULL) {
        mStreamInfoBuf->deallocate();
        delete mStreamInfoBuf;
        mStreamInfoBuf = NULL;
        mStreamInfo = NULL;
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : deleteStream
 *
 * DESCRIPTION: Deletes a camera stream
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *==========================================================================*/
void QCameraStream::deleteStream()
{
    if (mHandle > 0) {
        acquireStreamBufs();
        releaseBuffs();
        unmapStreamInfoBuf();
        mCamOps->delete_stream(mCamHandle, mChannelHandle, mHandle);
    }
}

/*===========================================================================
 * FUNCTION   : unMapBuf
 *
 * DESCRIPTION: unmaps buffers
 *
 * PARAMETERS :
 *   @heapBuf      : heap buffer handler
 *   @bufType      : buffer type
 *   @ops_tbl    : ptr to buf mapping/unmapping ops
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::unMapBuf(QCameraMemory *Buf,
        cam_mapping_buf_type bufType, mm_camera_map_unmap_ops_tbl_t *ops_tbl)
{
    int32_t rc = NO_ERROR;
    uint8_t cnt;
    ssize_t bufSize = BAD_INDEX;
    uint32_t i;

    cnt = Buf->getCnt();
    for (i = 0; i < cnt; i++) {
        bufSize = Buf->getSize(i);
        if (BAD_INDEX != bufSize) {
            if (m_MemOpsTbl.unmap_ops == NULL ) {
                rc = mCamOps->unmap_stream_buf(mCamHandle, mChannelHandle, mHandle,
                        bufType, i, -1);
            } else {
                rc = m_MemOpsTbl.unmap_ops(i, -1, bufType, m_MemOpsTbl.userdata);
            }
            if (rc < 0) {
                ALOGE("Failed to unmap buffer");
                break;
            }
        } else {
            ALOGE("Failed to retrieve buffer size (bad index)");
            rc = BAD_INDEX;
            break;
        }
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : mapBufs
 *
 * DESCRIPTION: maps buffers
 *
 * PARAMETERS :
 *   @heapBuf      : heap buffer handler
 *   @bufType      : buffer type
 *   @ops_tbl    : ptr to buf mapping/unmapping ops
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::mapBufs(QCameraMemory *Buf,
        cam_mapping_buf_type bufType, mm_camera_map_unmap_ops_tbl_t *ops_tbl)
{
    int32_t rc = NO_ERROR;
    ssize_t bufSize = BAD_INDEX;
    uint32_t i = 0;

    QCameraBufferMaps bufferMaps;
    for (i = 0; i < Buf->getCnt(); i++) {
        ssize_t bufSize = Buf->getSize(i);
        if (BAD_INDEX == bufSize) {
            ALOGE("Failed to retrieve buffer size (bad index)");
            return BAD_INDEX;
        }

        rc = bufferMaps.enqueue(bufType, mHandle, i /*buf index*/, -1 /*plane index*/,
                0 /*cookie*/, Buf->getFd(i), bufSize);

        if (rc < 0) {
            ALOGE("%s: Failed to map buffers", __func__);
            return BAD_INDEX;
        }
    }

    cam_buf_map_type_list bufMapList;
    rc = bufferMaps.getCamBufMapList(bufMapList);
    if (rc < 0) {
        ALOGE("%s: Failed to map buffers", __func__);
        return BAD_INDEX;
    }

    if (m_MemOpsTbl.bundled_map_ops == NULL) {
        rc = mCamOps->map_stream_bufs(mCamHandle, mChannelHandle, &bufMapList);
    } else {
        rc = m_MemOpsTbl.bundled_map_ops(&bufMapList, m_MemOpsTbl.userdata);
    }

    if (rc < 0) {
        ALOGE("Failed to map buffer");
        rc = BAD_INDEX;
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : backgroundAllocate
 *
 * DESCRIPTION: schedule buffers to be allocated in the background
 *
 * PARAMETERS :
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::backgroundAllocate(void *data) {
    QCameraStream *stream = (QCameraStream*)data;
    int32_t rc = stream->allocateBuffers();
    if (rc != NO_ERROR) {
        ALOGE("%s: Error allocating buffers !!!", __func__);
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : backgroundMap
 *
 * DESCRIPTION: map buffers in the background
 *
 * PARAMETERS :
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::backgroundMap(void *data) {
    QCameraStream *stream = (QCameraStream*)data;
    int32_t rc = stream->mapBuffers();
    if (rc != NO_ERROR) {
        ALOGE("%s: Error mapping buffers !!!", __func__);
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : init
 *
 * DESCRIPTION: initialize stream obj
 *
 * PARAMETERS :
 *   @streamInfoBuf: ptr to buf that contains stream info
 *   @miscBuf      : ptr to buf that contains misc bufs
 *   @stream_cb    : stream data notify callback. Can be NULL if not needed
 *   @userdata     : user data ptr
 *   @bDynallocBuf : flag to indicate if buffer allocation can be in 2 steps
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::init(QCameraHeapMemory *streamInfoBuf,
        QCameraHeapMemory *miscBuf,
        uint8_t minNumBuffers,
        stream_cb_routine stream_cb,
        void *userdata,
        bool bDynallocBuf)
{
    int32_t rc = OK;

    // assign and map stream info memory
    mStreamInfoBuf = streamInfoBuf;
    mStreamInfo = reinterpret_cast<cam_stream_info_t *>(mStreamInfoBuf->getPtr(0));
    mNumBufs = minNumBuffers;
    mDynBufAlloc = bDynallocBuf;

    // Calculate buffer size for deffered allocation
    if (mDefferedAllocation) {
        rc = calcOffset(mStreamInfo);
        if (rc < 0) {
            ALOGE("%s : Failed to calculate stream offset", __func__);
            goto done;
        }

        mAllocTask.bgFunction = backgroundAllocate;
        mAllocTask.bgArgs = this;
        mAllocTaskId = mAllocator.scheduleBackgroundTask(&mAllocTask);
        if (mAllocTaskId == 0) {
            ALOGE("%s : Failed to schedule buffer alloction", __func__);
            goto done;
        }
    }

    mHandle = mCamOps->add_stream(mCamHandle, mChannelHandle);
    if (!mHandle) {
        ALOGE("add_stream failed");
        rc = UNKNOWN_ERROR;
        goto done;
    }

    rc = mapBufs(mStreamInfoBuf, CAM_MAPPING_BUF_TYPE_STREAM_INFO, NULL);
    if (rc < 0) {
        ALOGE("Failed to map stream info buffer");
        goto err1;
    }

    mMiscBuf = miscBuf;
    if (miscBuf) {
        rc = mapBufs(mMiscBuf, CAM_MAPPING_BUF_TYPE_MISC_BUF, NULL);
        if (rc < 0) {
            ALOGE("Failed to map miscellaneous buffer");
            releaseMiscBuf();
            goto err1;
        }
    }

    rc = configStream();
    if (rc < 0) {
        ALOGE("%s : Failed to config stream ", __func__);
        goto err1;
    }

    if (mDefferedAllocation) {
        mMapTask.bgFunction = backgroundMap;
        mMapTask.bgArgs = this;
        mMapTaskId = mAllocator.scheduleBackgroundTask(&mMapTask);
        if (mMapTaskId == 0) {
            ALOGE("%s : Failed to schedule buffer alloction", __func__);
            goto done;
        }
    }

    mDataCB = stream_cb;
    mUserData = userdata;
    return 0;

err1:
    mCamOps->delete_stream(mCamHandle, mChannelHandle, mHandle);
    mHandle = 0;
    mNumBufs = 0;
done:
    return rc;
}

/*===========================================================================
 * FUNCTION   : calcOffset
 *
 * DESCRIPTION: calculate frame offset based on format and padding information
 *
 * PARAMETERS :
 *   @streamInfo  : stream information
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t QCameraStream::calcOffset(cam_stream_info_t *streamInfo)
{
    int32_t rc = 0;

    cam_dimension_t dim = streamInfo->dim;
    if (streamInfo->pp_config.feature_mask & CAM_QCOM_FEATURE_ROTATION &&
            streamInfo->stream_type != CAM_STREAM_TYPE_VIDEO) {
        if (streamInfo->pp_config.rotation == ROTATE_90 ||
                streamInfo->pp_config.rotation == ROTATE_270) {
            // rotated by 90 or 270, need to switch width and height
            dim.width = streamInfo->dim.height;
            dim.height = streamInfo->dim.width;
        }
    }

    switch (streamInfo->stream_type) {
    case CAM_STREAM_TYPE_PREVIEW:
    case CAM_STREAM_TYPE_CALLBACK:
        rc = mm_stream_calc_offset_preview(streamInfo,
                &dim,
                &mPaddingInfo,
                &streamInfo->buf_planes);
        break;
    case CAM_STREAM_TYPE_POSTVIEW:
        rc = mm_stream_calc_offset_post_view(streamInfo->fmt,
                &dim,
                &streamInfo->buf_planes);
        break;
    case CAM_STREAM_TYPE_SNAPSHOT:
        rc = mm_stream_calc_offset_snapshot(streamInfo->fmt,
                &dim,
                &mPaddingInfo,
                &streamInfo->buf_planes);
        break;
    case CAM_STREAM_TYPE_OFFLINE_PROC:
        rc = mm_stream_calc_offset_postproc(streamInfo,
                &mPaddingInfo,
                &streamInfo->buf_planes);
        break;
    case CAM_STREAM_TYPE_VIDEO:
        rc = mm_stream_calc_offset_video(streamInfo->fmt,
                &dim, &streamInfo->buf_planes);
        break;
    case CAM_STREAM_TYPE_RAW:
        rc = mm_stream_calc_offset_raw(streamInfo->fmt,
                &dim,
                &mPaddingInfo,
                &streamInfo->buf_planes);
        break;
    case CAM_STREAM_TYPE_ANALYSIS:
        rc = mm_stream_calc_offset_analysis(streamInfo->fmt,
                &dim,
                &mPaddingInfo,
                &streamInfo->buf_planes);
        break;
    case CAM_STREAM_TYPE_METADATA:
        rc = mm_stream_calc_offset_metadata(&dim,
                &mPaddingInfo,
                &streamInfo->buf_planes);
        break;
    default:
        ALOGE("%s: not supported for stream type %d",
                __func__, streamInfo->stream_type);
        rc = -1;
        break;
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : start
 *
 * DESCRIPTION: start stream. Will start main stream thread to handle stream
 *              related ops.
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::start()
{
    int32_t rc = 0;
    mDataQ.init();
    rc = mProcTh.launch(dataProcRoutine, this);
    if (rc == NO_ERROR) {
        m_bActive = true;
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : stop
 *
 * DESCRIPTION: stop stream. Will stop main stream thread
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::stop()
{
    int32_t rc = 0;
    m_bActive = false;
    mAllocator.waitForBackgroundTask(mAllocTaskId);
    mAllocator.waitForBackgroundTask(mMapTaskId);
    rc = mProcTh.exit();
    return rc;
}

/*===========================================================================
 * FUNCTION   : syncRuntimeParams
 *
 * DESCRIPTION: query and sync runtime parameters like output crop
 *              buffer info etc.
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::syncRuntimeParams()
{
    int32_t ret = NO_ERROR;

    memset(&m_OutputCrop, 0, sizeof(cam_stream_parm_buffer_t));
    m_OutputCrop.type = CAM_STREAM_PARAM_TYPE_GET_OUTPUT_CROP;

    ret = getParameter(m_OutputCrop);
    if (ret != NO_ERROR) {
        ALOGE("%s: stream getParameter for output crop failed", __func__);
        return ret;
    }

    memset(&m_ImgProp, 0, sizeof(cam_stream_parm_buffer_t));
    m_ImgProp.type = CAM_STREAM_PARAM_TYPE_GET_IMG_PROP;

    ret = getParameter(m_ImgProp);
    if (ret != NO_ERROR) {
        ALOGE("%s: stream getParameter for image prop failed", __func__);
        return ret;
    }

    return ret;
}

/*===========================================================================
 * FUNCTION   : processZoomDone
 *
 * DESCRIPTION: process zoom done event
 *
 * PARAMETERS :
 *   @previewWindoe : preview window ops table to set preview crop window
 *   @crop_info     : crop info
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::processZoomDone(preview_stream_ops_t *previewWindow,
                                       cam_crop_data_t &crop_info)
{
    int32_t rc = 0;

    if (!m_bActive) {
        ALOGV("%s : Stream not active", __func__);
        return NO_ERROR;
    }

    // get stream param for crop info
    for (int i = 0; i < crop_info.num_of_streams; i++) {
        if (crop_info.crop_info[i].stream_id == mStreamInfo->stream_svr_id) {
            pthread_mutex_lock(&mCropLock);
            mCropInfo = crop_info.crop_info[i].crop;
            pthread_mutex_unlock(&mCropLock);

            // update preview window crop if it's preview/postview stream
            if ( (previewWindow != NULL) &&
                 (mStreamInfo->stream_type == CAM_STREAM_TYPE_PREVIEW ||
                  mStreamInfo->stream_type == CAM_STREAM_TYPE_POSTVIEW) ) {
                rc = previewWindow->set_crop(previewWindow,
                                             mCropInfo.left,
                                             mCropInfo.top,
                                             mCropInfo.width,
                                             mCropInfo.height);
            }
            break;
        }
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : processDataNotify
 *
 * DESCRIPTION: process stream data notify
 *
 * PARAMETERS :
 *   @frame   : stream frame received
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::processDataNotify(mm_camera_super_buf_t *frame)
{
    CDBG("%s:\n", __func__);
    if (mDataQ.enqueue((void *)frame)) {
        return mProcTh.sendCmd(CAMERA_CMD_TYPE_DO_NEXT_JOB, FALSE, FALSE);
    } else {
        CDBG_HIGH("%s: Stream thread is not active, no ops here", __func__);
        bufDone(frame->bufs[0]->buf_idx);
        free(frame);
        return NO_ERROR;
    }
}

/*===========================================================================
 * FUNCTION   : dataNotifySYNCCB
 *
 * DESCRIPTION: This function registered with interface for
 *                        SYNC callback if SYNC callback registered.
 *
 * PARAMETERS :
 *   @recvd_frame   : stream frame received
 *   @userdata      : user data ptr
 *
 * RETURN     : none
 *==========================================================================*/
void QCameraStream::dataNotifySYNCCB(mm_camera_super_buf_t *recvd_frame,
        void *userdata)
{
    CDBG("%s:\n", __func__);
    QCameraStream* stream = (QCameraStream *)userdata;
    if (stream == NULL ||
        recvd_frame == NULL ||
        recvd_frame->bufs[0] == NULL ||
        recvd_frame->bufs[0]->stream_id != stream->getMyHandle()) {
        ALOGE("%s: Not a valid stream to handle buf", __func__);
        return;
    }
    if (stream->mSYNCDataCB != NULL)
        stream->mSYNCDataCB(recvd_frame, stream, stream->mUserData);
    return;
}


/*===========================================================================
 * FUNCTION   : dataNotifyCB
 *
 * DESCRIPTION: callback for data notify. This function is registered with
 *              mm-camera-interface to handle data notify
 *
 * PARAMETERS :
 *   @recvd_frame   : stream frame received
 *   userdata       : user data ptr
 *
 * RETURN     : none
 *==========================================================================*/
void QCameraStream::dataNotifyCB(mm_camera_super_buf_t *recvd_frame,
                                 void *userdata)
{
    CDBG("%s:\n", __func__);
    QCameraStream* stream = (QCameraStream *)userdata;
    if (stream == NULL ||
        recvd_frame == NULL ||
        recvd_frame->bufs[0] == NULL ||
        recvd_frame->bufs[0]->stream_id != stream->getMyHandle()) {
        ALOGE("%s: Not a valid stream to handle buf", __func__);
        return;
    }

    mm_camera_super_buf_t *frame =
        (mm_camera_super_buf_t *)malloc(sizeof(mm_camera_super_buf_t));
    if (frame == NULL) {
        ALOGE("%s: No mem for mm_camera_buf_def_t", __func__);
        stream->bufDone(recvd_frame->bufs[0]->buf_idx);
        return;
    }
    *frame = *recvd_frame;
    stream->processDataNotify(frame);
    return;
}

/*===========================================================================
 * FUNCTION   : dataProcRoutine
 *
 * DESCRIPTION: function to process data in the main stream thread
 *
 * PARAMETERS :
 *   @data    : user data ptr
 *
 * RETURN     : none
 *==========================================================================*/
void *QCameraStream::dataProcRoutine(void *data)
{
    int running = 1;
    int ret;
    QCameraStream *pme = (QCameraStream *)data;
    QCameraCmdThread *cmdThread = &pme->mProcTh;
    cmdThread->setName("CAM_strmDatProc");

    CDBG("%s: E", __func__);
    do {
        do {
            ret = cam_sem_wait(&cmdThread->cmd_sem);
            if (ret != 0 && errno != EINVAL) {
                ALOGE("%s: cam_sem_wait error (%s)",
                      __func__, strerror(errno));
                return NULL;
            }
        } while (ret != 0);

        // we got notified about new cmd avail in cmd queue
        camera_cmd_type_t cmd = cmdThread->getCmd();
        switch (cmd) {
        case CAMERA_CMD_TYPE_DO_NEXT_JOB:
            {
                CDBG_HIGH("%s: Do next job", __func__);
                mm_camera_super_buf_t *frame =
                    (mm_camera_super_buf_t *)pme->mDataQ.dequeue();
                if (NULL != frame) {
                    if (pme->mDataCB != NULL) {
                        pme->mDataCB(frame, pme, pme->mUserData);
                    } else {
                        // no data cb routine, return buf here
                        pme->bufDone(frame->bufs[0]->buf_idx);
                        free(frame);
                    }
                }
            }
            break;
        case CAMERA_CMD_TYPE_EXIT:
            CDBG_HIGH("%s: Exit", __func__);
            /* flush data buf queue */
            pme->mDataQ.flush();
            running = 0;
            break;
        default:
            break;
        }
    } while (running);
    CDBG_HIGH("%s: X", __func__);
    return NULL;
}

/*===========================================================================
 * FUNCTION   : bufDone
 *
 * DESCRIPTION: return stream buffer to kernel
 *
 * PARAMETERS :
 *   @index   : index of buffer to be returned
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::bufDone(uint32_t index)
{
    int32_t rc = NO_ERROR;

    if (index >= mNumBufs || mBufDefs == NULL)
        return BAD_INDEX;

    rc = mCamOps->qbuf(mCamHandle, mChannelHandle, &mBufDefs[index]);
    if (rc < 0)
        return rc;

    return rc;
}

/*===========================================================================
 * FUNCTION   : bufDone
 *
 * DESCRIPTION: return stream buffer to kernel
 *
 * PARAMETERS :
 *   @opaque    : stream frame/metadata buf to be returned
 *   @isMetaData: flag if returned opaque is a metadatabuf or the real frame ptr
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::bufDone(const void *opaque, bool isMetaData)
{
    int32_t rc = NO_ERROR;
    int index;
#ifdef USE_MEDIA_EXTENSIONS
    QCameraVideoMemory *lVideoMem = NULL;
#endif

    if (mStreamInfo != NULL
            && mStreamInfo->streaming_mode == CAM_STREAMING_MODE_BATCH) {
        index = mStreamBatchBufs->getMatchBufIndex(opaque, TRUE);
#ifdef USE_MEDIA_EXTENSIONS
        lVideoMem = (QCameraVideoMemory *)mStreamBatchBufs;
#endif
        if (index == -1 || index >= mNumBufs || mBufDefs == NULL) {
            ALOGE("%s: Cannot find buf for opaque data = %p", __func__, opaque);
            return BAD_INDEX;
        }
        camera_memory_t *video_mem = mStreamBatchBufs->getMemory(index, true);
        if (video_mem != NULL) {
            struct encoder_media_buffer_type * packet =
                    (struct encoder_media_buffer_type *)video_mem->data;
            native_handle_t *nh = const_cast<native_handle_t *>(packet->meta_handle);
            if (NULL != nh) {
               if (native_handle_delete(nh)) {
                   ALOGE("%s: Unable to delete native handle", __func__);
               }
            } else {
               ALOGE("%s : native handle not available", __func__);
            }
        }
    } else {
        index = mStreamBufs->getMatchBufIndex(opaque, isMetaData);
#ifdef USE_MEDIA_EXTENSIONS
        lVideoMem = (QCameraVideoMemory *)mStreamBufs;
#endif
        if (index == -1 || index >= mNumBufs || mBufDefs == NULL) {
            ALOGE("%s: Cannot find buf for opaque data = %p", __func__, opaque);
            return BAD_INDEX;
        }
        CDBG_HIGH("%s: Buffer Index = %d, Frame Idx = %d", __func__, index,
                mBufDefs[index].frame_idx);
    }
#ifdef USE_MEDIA_EXTENSIONS
    //Close and delete duplicated native handle and FD's.
    if (lVideoMem != NULL) {
        rc = lVideoMem->closeNativeHandle(opaque, isMetaData);
        if (rc != NO_ERROR) {
            CDBG_HIGH("Invalid video metadata");
            return rc;
        }
    } else {
        CDBG_HIGH("Possible FD leak. Release recording called after stop");
    }
#endif
    rc = bufDone((uint32_t)index);
    return rc;
}

/*===========================================================================
 * FUNCTION   : getNumQueuedBuf
 *
 * DESCRIPTION: return queued buffer count
 *
 * PARAMETERS : None
 *
 * RETURN     : queued buffer count
 *==========================================================================*/
int32_t QCameraStream::getNumQueuedBuf()
{
    int32_t rc = -1;
    if (mHandle > 0) {
        rc = mCamOps->get_queued_buf_count(mCamHandle, mChannelHandle, mHandle);
    }
    if (rc == -1) {
        ALOGE("%s: stream is not in active state. Invalid operation", __func__);
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : getBufs
 *
 * DESCRIPTION: allocate stream buffers
 *
 * PARAMETERS :
 *   @offset     : offset info of stream buffers
 *   @num_bufs   : number of buffers allocated
 *   @initial_reg_flag: flag to indicate if buffer needs to be registered
 *                      at kernel initially
 *   @bufs       : output of allocated buffers
 *   @ops_tbl    : ptr to buf mapping/unmapping ops
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::getBufs(cam_frame_len_offset_t *offset,
        uint8_t *num_bufs,
        uint8_t **initial_reg_flag,
        mm_camera_buf_def_t **bufs,
        mm_camera_map_unmap_ops_tbl_t *ops_tbl)
{
    int rc = NO_ERROR;
    uint8_t *regFlags;

    if (!ops_tbl) {
        ALOGE("%s: ops_tbl is NULL", __func__);
        return INVALID_OPERATION;
    }

    mFrameLenOffset = *offset;

    uint8_t numBufAlloc = mNumBufs;
    mNumBufsNeedAlloc = 0;
    if (mDynBufAlloc) {
        numBufAlloc = CAMERA_MIN_ALLOCATED_BUFFERS;
        if (numBufAlloc > mNumBufs) {
            mDynBufAlloc = false;
            numBufAlloc = mNumBufs;
        } else {
            mNumBufsNeedAlloc = (uint8_t)(mNumBufs - numBufAlloc);
        }
    }

    /* For some stream types, buffer allocation may have already begun
     * preemptively. If this is the case, we need to wait for the
     * preemptive allocation to complete before proceeding. */
    mAllocator.waitForDeferredAlloc(mStreamInfo->stream_type);

    //Allocate stream buffer
    mStreamBufs = mAllocator.allocateStreamBuf(mStreamInfo->stream_type,
            mFrameLenOffset.frame_len, mFrameLenOffset.mp[0].stride,
            mFrameLenOffset.mp[0].scanline, numBufAlloc);
    if (!mStreamBufs) {
        ALOGE("%s: Failed to allocate stream buffers", __func__);
        return NO_MEMORY;
    }

    mNumBufs = (uint8_t)(numBufAlloc + mNumBufsNeedAlloc);
    uint8_t numBufsToMap = mStreamBufs->getMappable();

    QCameraBufferMaps bufferMaps;
    for (uint32_t i = 0; i < numBufsToMap; i++) {
        ssize_t bufSize = mStreamBufs->getSize(i);
        if (BAD_INDEX == bufSize) {
            ALOGE("Failed to retrieve buffer size (bad index)");
            return INVALID_OPERATION;
        }

        rc = bufferMaps.enqueue(CAM_MAPPING_BUF_TYPE_STREAM_BUF,
                0 /*stream id*/, i /*buf index*/, -1 /*plane index*/,
                0 /*cookie*/, mStreamBufs->getFd(i), bufSize);

        if (rc < 0) {
            ALOGE("%s: Failed to map buffers", __func__);
            return BAD_INDEX;
        }
    }

    cam_buf_map_type_list bufMapList;
    rc = bufferMaps.getCamBufMapList(bufMapList);
    if (rc == NO_ERROR) {
        rc = ops_tbl->bundled_map_ops(&bufMapList, ops_tbl->userdata);
    }
    if (rc < 0) {
        ALOGE("%s: map_stream_buf failed: %d", __func__, rc);
        mStreamBufs->deallocate();
        delete mStreamBufs;
        mStreamBufs = NULL;
        return INVALID_OPERATION;
    }

    //regFlags array is allocated by us, but consumed and freed by mm-camera-interface
    regFlags = (uint8_t *)malloc(sizeof(uint8_t) * mNumBufs);
    if (!regFlags) {
        ALOGE("%s: Out of memory", __func__);
        for (uint32_t i = 0; i < numBufsToMap; i++) {
            ops_tbl->unmap_ops(i, -1, CAM_MAPPING_BUF_TYPE_STREAM_BUF, ops_tbl->userdata);
        }
        mStreamBufs->deallocate();
        delete mStreamBufs;
        mStreamBufs = NULL;
        return NO_MEMORY;
    }
    memset(regFlags, 0, sizeof(uint8_t) * mNumBufs);

    mBufDefs = (mm_camera_buf_def_t *)malloc(mNumBufs * sizeof(mm_camera_buf_def_t));
    if (mBufDefs == NULL) {
        ALOGE("%s: getRegFlags failed %d", __func__, rc);
        for (uint32_t i = 0; i < numBufsToMap; i++) {
            ops_tbl->unmap_ops(i, -1, CAM_MAPPING_BUF_TYPE_STREAM_BUF, ops_tbl->userdata);
        }
        mStreamBufs->deallocate();
        delete mStreamBufs;
        mStreamBufs = NULL;
        free(regFlags);
        regFlags = NULL;
        return INVALID_OPERATION;
    }
    memset(mBufDefs, 0, mNumBufs * sizeof(mm_camera_buf_def_t));
    for (uint32_t i = 0; i < numBufsToMap; i++) {
        mStreamBufs->getBufDef(mFrameLenOffset, mBufDefs[i], i);
    }

    rc = mStreamBufs->getRegFlags(regFlags);
    if (rc < 0) {
        ALOGE("%s: getRegFlags failed %d", __func__, rc);
        for (uint32_t i = 0; i < numBufsToMap; i++) {
            ops_tbl->unmap_ops(i, -1, CAM_MAPPING_BUF_TYPE_STREAM_BUF, ops_tbl->userdata);
        }
        mStreamBufs->deallocate();
        delete mStreamBufs;
        mStreamBufs = NULL;
        free(mBufDefs);
        mBufDefs = NULL;
        free(regFlags);
        regFlags = NULL;
        return INVALID_OPERATION;
    }

    *num_bufs = mNumBufs;
    *initial_reg_flag = regFlags;
    *bufs = mBufDefs;
    CDBG_HIGH("%s: stream type: %d, mRegFlags: 0x%x, numBufs: %d",
            __func__, mStreamInfo->stream_type, regFlags, mNumBufs);

    if (mNumBufsNeedAlloc > 0) {
        pthread_mutex_lock(&m_lock);
        wait_for_cond = TRUE;
        pthread_mutex_unlock(&m_lock);
        CDBG_HIGH("%s: Still need to allocate %d buffers",
              __func__, mNumBufsNeedAlloc);
        // start another thread to allocate the rest of buffers
        pthread_create(&mBufAllocPid,
                       NULL,
                       BufAllocRoutine,
                       this);
        pthread_setname_np(mBufAllocPid, "CAM_strmBuf");
    }

    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : getBufsDeferred
 *
 * DESCRIPTION: allocate deferred stream buffers
 *
 * PARAMETERS :
 *   @offset     : offset info of stream buffers
 *   @num_bufs   : number of buffers allocated
 *   @initial_reg_flag: flag to indicate if buffer needs to be registered
 *                      at kernel initially
 *   @bufs       : output of allocated buffers
 *   @ops_tbl    : ptr to buf mapping/unmapping ops
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::getBufsDeferred(cam_frame_len_offset_t *offset,
        uint8_t *num_bufs,
        uint8_t **initial_reg_flag,
        mm_camera_buf_def_t **bufs,
        mm_camera_map_unmap_ops_tbl_t *ops_tbl)
{
    // wait for allocation
    mAllocator.waitForBackgroundTask(mAllocTaskId);

    if (!mRegFlags || !mBufDefs) {
        ALOGE("%s: reg flags or buf defs uninitialized", __func__);
        return NO_MEMORY;
    }

    *initial_reg_flag   = mRegFlags;
    *num_bufs           = mNumBufs;
    *bufs               = mBufDefs;

    CDBG_HIGH("%s: stream type: %d, mRegFlags: 0x%x, numBufs: %d",
            __func__, getMyType(), mRegFlags, mNumBufs);

    return NO_ERROR;
}
/*===========================================================================
 * FUNCTION   : mapNewBuffer
 *
 * DESCRIPTION: map a new stream buffer
 *
 * PARAMETERS :
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::mapNewBuffer(uint32_t index)
{
    CDBG_HIGH("%s: E - index = %d", __func__, index);

    int rc = NO_ERROR;

    ssize_t bufSize = mStreamBufs->getSize(index);
    if (BAD_INDEX == bufSize) {
        ALOGE("Failed to retrieve buffer size (bad index)");
        return INVALID_OPERATION;
    }

    cam_buf_map_type_list bufMapList;
    rc = QCameraBufferMaps::makeSingletonBufMapList(
            CAM_MAPPING_BUF_TYPE_STREAM_BUF, 0 /*stream id*/, index,
            -1 /*plane index*/, 0 /*cookie*/, mStreamBufs->getFd(index),
            bufSize, bufMapList);

    if (rc == NO_ERROR) {
        rc = m_MemOpsTbl.bundled_map_ops(&bufMapList, m_MemOpsTbl.userdata);
    }
    if (rc < 0) {
        ALOGE("%s: map_stream_buf failed: %d", __func__, rc);
        rc = INVALID_OPERATION;
    } else {
        mStreamBufs->getBufDef(mFrameLenOffset, mBufDefs[index], index);
    }

    CDBG_HIGH("%s: X - rc = %d", __func__, rc);
    return rc;
}

/*===========================================================================
 * FUNCTION   : allocateBuffers
 *
 * DESCRIPTION: allocate stream buffers
 *
 * PARAMETERS :
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::allocateBuffers()
{
    int32_t rc = NO_ERROR;

    mFrameLenOffset = mStreamInfo->buf_planes.plane_info;

    if (mStreamInfo->streaming_mode == CAM_STREAMING_MODE_BATCH) {
        return allocateBatchBufs(&mFrameLenOffset,
                &mNumBufs, &mRegFlags,
                &mBufDefs, NULL);
    }

    /* This allocation is running in the deferred context, so it
     * is safe (and necessary) to assume any preemptive allocation
     * is already complete. Therefore, no need to wait here. */

    uint8_t numBufAlloc = mNumBufs;
    mNumBufsNeedAlloc = 0;
    if (mDynBufAlloc) {
        numBufAlloc = CAMERA_MIN_ALLOCATED_BUFFERS;
        if (numBufAlloc > mNumBufs) {
            mDynBufAlloc = false;
            numBufAlloc = mNumBufs;
        } else {
            mNumBufsNeedAlloc = (uint8_t)(mNumBufs - numBufAlloc);
        }
    }

    //Allocate and map stream info buffer
    mStreamBufs = mAllocator.allocateStreamBuf(mStreamInfo->stream_type,
            mFrameLenOffset.frame_len,
            mFrameLenOffset.mp[0].stride,
            mFrameLenOffset.mp[0].scanline,
            numBufAlloc);

    if (!mStreamBufs) {
        ALOGE("%s: Failed to allocate stream buffers", __func__);
        rc = NO_MEMORY;
    }

    mNumBufs = (uint8_t)(numBufAlloc + mNumBufsNeedAlloc);
    uint8_t numBufsToMap = mStreamBufs->getMappable();

    //regFlags array is allocated by us,
    // but consumed and freed by mm-camera-interface
    mRegFlags = (uint8_t *)malloc(sizeof(uint8_t) * mNumBufs);
    if (!mRegFlags) {
        ALOGE("%s: Out of memory", __func__);
        for (uint32_t i = 0; i < numBufsToMap; i++) {
            unmapBuf(CAM_MAPPING_BUF_TYPE_STREAM_BUF, i, -1, NULL);
        }
        mStreamBufs->deallocate();
        delete mStreamBufs;
        mStreamBufs = NULL;
        return NO_MEMORY;
    }
    memset(mRegFlags, 0, sizeof(uint8_t) * mNumBufs);

    size_t bufDefsSize = mNumBufs * sizeof(mm_camera_buf_def_t);
    mBufDefs = (mm_camera_buf_def_t *)malloc(bufDefsSize);
    if (mBufDefs == NULL) {
        ALOGE("%s: getRegFlags failed %d", __func__, rc);
        for (uint32_t i = 0; i < numBufsToMap; i++) {
            unmapBuf(CAM_MAPPING_BUF_TYPE_STREAM_BUF, i, -1, NULL);
        }
        mStreamBufs->deallocate();
        delete mStreamBufs;
        mStreamBufs = NULL;
        free(mRegFlags);
        mRegFlags = NULL;
        return INVALID_OPERATION;
    }
    memset(mBufDefs, 0, bufDefsSize);
    for (uint32_t i = 0; i < numBufsToMap; i++) {
        mStreamBufs->getBufDef(mFrameLenOffset, mBufDefs[i], i);
    }

    rc = mStreamBufs->getRegFlags(mRegFlags);
    if (rc < 0) {
        ALOGE("%s: getRegFlags failed %d", __func__, rc);
        for (uint32_t i = 0; i < numBufsToMap; i++) {
            unmapBuf(CAM_MAPPING_BUF_TYPE_STREAM_BUF, i, -1, NULL);
        }
        mStreamBufs->deallocate();
        delete mStreamBufs;
        mStreamBufs = NULL;
        free(mBufDefs);
        mBufDefs = NULL;
        free(mRegFlags);
        mRegFlags = NULL;
        return INVALID_OPERATION;
    }

    if (mNumBufsNeedAlloc > 0) {
        pthread_mutex_lock(&m_lock);
        wait_for_cond = TRUE;
        pthread_mutex_unlock(&m_lock);
        CDBG_HIGH("%s: Still need to allocate %d buffers",
              __func__, mNumBufsNeedAlloc);
        // start another thread to allocate the rest of buffers
        pthread_create(&mBufAllocPid,
                       NULL,
                       BufAllocRoutine,
                       this);
        pthread_setname_np(mBufAllocPid, "CAM_strmBufAlloc");
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : mapBuffers
 *
 * DESCRIPTION: map stream buffers
 *
 * PARAMETERS :
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::mapBuffers()
{
    int32_t rc = NO_ERROR;
    QCameraBufferMaps bufferMaps;
    uint8_t numBufsToMap = mStreamBufs->getMappable();
    for (uint32_t i = 0; i < numBufsToMap; i++) {
        ssize_t bufSize = mStreamBufs->getSize(i);
        if (BAD_INDEX != bufSize) {
            rc = bufferMaps.enqueue(CAM_MAPPING_BUF_TYPE_STREAM_BUF, mHandle,
                    i /*buf index*/, -1 /*plane index*/, 0 /*cookie*/,
                    mStreamBufs->getFd(i), bufSize);

            if (rc < 0) {
                ALOGE("%s: Failed to map buffers", __func__);
                rc = BAD_INDEX;
                break;
            }
        } else {
            ALOGE("%s: Bad index %u", __func__, i);
            rc = BAD_INDEX;
            break;
        }
    }

    cam_buf_map_type_list bufMapList;
    if (rc == NO_ERROR) {
        rc = bufferMaps.getCamBufMapList(bufMapList);
    }
    if (rc == NO_ERROR) {
        rc = mapBufs(bufMapList, NULL);
    }

    if (rc < 0) {
        ALOGE("%s: Cleanup after error: %d", __func__, rc);
        mStreamBufs->deallocate();
        delete mStreamBufs;
        mStreamBufs = NULL;
        return INVALID_OPERATION;
    }
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : allocateBatchBufs
 *
 * DESCRIPTION: allocate stream batch buffers and stream buffers
 *
 * PARAMETERS :
 *   @offset     : offset info of stream buffers
 *   @num_bufs   : number of buffers allocated
 *   @initial_reg_flag: flag to indicate if buffer needs to be registered
 *                      at kernel initially
 *   @bufs       : output of allocated buffers
 *   @plane_bufs    : output of allocated plane buffers
  *   @ops_tbl    : ptr to buf mapping/unmapping ops
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::allocateBatchBufs(cam_frame_len_offset_t *offset,
        uint8_t *num_bufs, uint8_t **initial_reg_flag,
        mm_camera_buf_def_t **bufs, mm_camera_map_unmap_ops_tbl_t *ops_tbl)
{
    int rc = NO_ERROR;
    uint8_t *regFlags;
    QCameraBufferMaps bufferMaps;
    QCameraBufferMaps planeBufferMaps;

    mFrameLenOffset = *offset;

    CDBG_HIGH("%s : Batch Buffer allocation stream type = %d", __func__, getMyType());

    //Allocate stream batch buffer
    mStreamBatchBufs = mAllocator.allocateStreamUserBuf (mStreamInfo);
    if (!mStreamBatchBufs) {
        ALOGE("%s: Failed to allocate stream batch buffers", __func__);
        return NO_MEMORY;
    }

    uint8_t numBufsToMap = mStreamBatchBufs->getMappable();

    //map batch buffers
    for (uint32_t i = 0; i < numBufsToMap; i++) {
        rc = bufferMaps.enqueue(CAM_MAPPING_BUF_TYPE_STREAM_USER_BUF,
                0 /*stream id*/, i /*buf index*/, -1 /*plane index*/,
                0 /*cookie*/, mStreamBatchBufs->getFd(i), mNumBufs);

        if (rc < 0) {
            ALOGE("%s: Failed to map buffers", __func__);
            rc = BAD_INDEX;
            break;
        }
    }

    cam_buf_map_type_list bufMapList;
    if (rc == NO_ERROR) {
        rc = bufferMaps.getCamBufMapList(bufMapList);
    }
    if (rc == NO_ERROR) {
        rc = mapBufs(bufMapList, ops_tbl);
    }
    if (rc < 0) {
        ALOGE("Failed to map stream batch buffers");
        mStreamBatchBufs->deallocate();
        delete mStreamBatchBufs;
        mStreamBatchBufs = NULL;
        return NO_MEMORY;
    }

    /*calculate stream Buffer count*/
    mNumPlaneBufs =
            (mNumBufs * mStreamInfo->user_buf_info.frame_buf_cnt);

    /* For some stream types, buffer allocation may have already begun
     * preemptively. If this is the case, we need to wait for the
     * preemptive allocation to complete before proceeding. */
    mAllocator.waitForDeferredAlloc(mStreamInfo->stream_type);

    //Allocate stream buffer
    mStreamBufs = mAllocator.allocateStreamBuf(mStreamInfo->stream_type,
            mFrameLenOffset.frame_len,mFrameLenOffset.mp[0].stride,
            mFrameLenOffset.mp[0].scanline,mNumPlaneBufs);
    if (!mStreamBufs) {
        ALOGE("%s: Failed to allocate stream buffers", __func__);
        rc = NO_MEMORY;
        goto err1;
    }

    //Map plane stream buffers
    for (uint32_t i = 0; i < mNumPlaneBufs; i++) {
        ssize_t bufSize = mStreamBufs->getSize(i);
        if (BAD_INDEX != bufSize) {
            rc = planeBufferMaps.enqueue(CAM_MAPPING_BUF_TYPE_STREAM_BUF,
                    0 /*stream id*/, i /*buf index*/, -1 /*plane index*/,
                    0 /*cookie*/, mStreamBufs->getFd(i), bufSize);

            if (rc < 0) {
                ALOGE("%s: Failed to map buffers", __func__);
                mStreamBufs->deallocate();
                delete mStreamBufs;
                mStreamBufs = NULL;
                rc = INVALID_OPERATION;
                goto err1;
            }
        } else {
            ALOGE("Failed to retrieve buffer size (bad index)");
            mStreamBufs->deallocate();
            delete mStreamBufs;
            mStreamBufs = NULL;
            rc = INVALID_OPERATION;
            goto err1;
        }
    }

    cam_buf_map_type_list planeBufMapList;
    rc = planeBufferMaps.getCamBufMapList(planeBufMapList);
    if (rc == NO_ERROR) {
        rc = mapBufs(planeBufMapList, ops_tbl);
    }

    if (rc < 0) {
        ALOGE("%s: map_stream_buf failed: %d", __func__, rc);
        mStreamBufs->deallocate();
        delete mStreamBufs;
        mStreamBufs = NULL;
        rc = INVALID_OPERATION;
        goto err1;
    }

    CDBG ("%s: BATCH Buf Count = %d, Plane Buf Cnt = %d", __func__,
            mNumBufs, mNumPlaneBufs);

    //regFlags array is allocated by us, but consumed and freed by mm-camera-interface
    regFlags = (uint8_t *)malloc(sizeof(uint8_t) * mNumBufs);
    if (!regFlags) {
        ALOGE("%s: Out of memory", __func__);
        for (uint32_t i = 0; i < mNumPlaneBufs; i++) {
            unmapBuf(CAM_MAPPING_BUF_TYPE_STREAM_BUF, i, -1, ops_tbl);
        }
        mStreamBufs->deallocate();
        delete mStreamBufs;
        mStreamBufs = NULL;
        rc = NO_MEMORY;
        goto err1;
    }
    memset(regFlags, 0, sizeof(uint8_t) * mNumBufs);
    for (uint32_t i = 0; i < mNumBufs; i++) {
        regFlags[i] = 1;
    }

    mBufDefs = (mm_camera_buf_def_t *)malloc(mNumBufs * sizeof(mm_camera_buf_def_t));
    if (mBufDefs == NULL) {
        ALOGE("%s: getRegFlags failed %d", __func__, rc);
        for (uint32_t i = 0; i < mNumPlaneBufs; i++) {
            unmapBuf(CAM_MAPPING_BUF_TYPE_STREAM_BUF, i, -1, ops_tbl);
        }
        mStreamBufs->deallocate();
        delete mStreamBufs;
        mStreamBufs = NULL;
        free(regFlags);
        regFlags = NULL;
        rc = INVALID_OPERATION;
        goto err1;
    }
    memset(mBufDefs, 0, mNumBufs * sizeof(mm_camera_buf_def_t));

    mPlaneBufDefs = (mm_camera_buf_def_t *)
            malloc(mNumPlaneBufs * (sizeof(mm_camera_buf_def_t)));
    if (mPlaneBufDefs == NULL) {
        ALOGE("%s : No Memory", __func__);
        free(regFlags);
        regFlags = NULL;
        free(mBufDefs);
        mBufDefs = NULL;
        for (uint32_t i = 0; i < mNumPlaneBufs; i++) {
            unmapBuf(CAM_MAPPING_BUF_TYPE_STREAM_BUF, i, -1, ops_tbl);
        }
        mStreamBufs->deallocate();
        delete mStreamBufs;
        mStreamBufs = NULL;
        free(regFlags);
        regFlags = NULL;
        rc = INVALID_OPERATION;
        goto err1;
    }
    memset(mPlaneBufDefs, 0,
             mNumPlaneBufs * (sizeof(mm_camera_buf_def_t)));

    for (uint32_t i = 0; i < mStreamInfo->num_bufs; i++) {
        mStreamBatchBufs->getUserBufDef(mStreamInfo->user_buf_info,
                mBufDefs[i], i, mFrameLenOffset, mPlaneBufDefs,
                mStreamBufs);
    }

    *num_bufs = mNumBufs;
    *initial_reg_flag = regFlags;
    *bufs = mBufDefs;
    CDBG_HIGH("%s: stream type: %d, numBufs: %d mNumPlaneBufs: %d",
            __func__, mStreamInfo->stream_type, mNumBufs, mNumPlaneBufs);

    return NO_ERROR;

err1:
    mStreamBatchBufs->deallocate();
    delete mStreamBatchBufs;
    mStreamBatchBufs = NULL;
    return rc;
}


/*===========================================================================
 * FUNCTION   : releaseBuffs
 *
 * DESCRIPTION: method to deallocate stream buffers
 *
 * PARAMETERS :
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::releaseBuffs()
{
    int rc = NO_ERROR;

    if (mBufAllocPid != 0) {
        cond_signal(true);
        CDBG_HIGH("%s: wait for buf allocation thread dead", __func__);
        pthread_join(mBufAllocPid, NULL);
        mBufAllocPid = 0;
        CDBG_HIGH("%s: return from buf allocation thread", __func__);
    }

    if (mStreamInfo->streaming_mode == CAM_STREAMING_MODE_BATCH) {
        return releaseBatchBufs(NULL);
    }

    if (NULL != mBufDefs) {
        uint8_t numBufsToUnmap = mStreamBufs->getMappable();
        for (uint32_t i = 0; i < numBufsToUnmap; i++) {
            rc = unmapBuf(CAM_MAPPING_BUF_TYPE_STREAM_BUF, i, -1, NULL);
            if (rc < 0) {
                ALOGE("%s: map_stream_buf failed: %d", __func__, rc);
            }
        }

        // mBufDefs just keep a ptr to the buffer
        // mm-camera-interface own the buffer, so no need to free
        mBufDefs = NULL;
        memset(&mFrameLenOffset, 0, sizeof(mFrameLenOffset));
    }
    if (!mStreamBufsAcquired && mStreamBufs != NULL) {
        mStreamBufs->deallocate();
        delete mStreamBufs;
        mStreamBufs = NULL;
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : releaseBatchBufs
 *
 * DESCRIPTION: method to deallocate stream buffers and batch buffers
 *
 * PARAMETERS :
 *   @ops_tbl    : ptr to buf mapping/unmapping ops
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code

 *==========================================================================*/
int32_t QCameraStream::releaseBatchBufs(mm_camera_map_unmap_ops_tbl_t *ops_tbl)
{
    int rc = NO_ERROR;

    if (NULL != mPlaneBufDefs) {
        for (uint32_t i = 0; i < mNumPlaneBufs; i++) {
            rc = unmapBuf(CAM_MAPPING_BUF_TYPE_STREAM_BUF, i, -1, ops_tbl);
            if (rc < 0) {
                ALOGE("%s: map_stream_buf failed: %d", __func__, rc);
            }
        }

        // mBufDefs just keep a ptr to the buffer
        // mm-camera-interface own the buffer, so no need to free
        mPlaneBufDefs = NULL;
        memset(&mFrameLenOffset, 0, sizeof(mFrameLenOffset));
        mNumPlaneBufs = 0;
    }

    if ( mStreamBufs != NULL) {
        mStreamBufs->deallocate();
        delete mStreamBufs;
    }

    mBufDefs = NULL;

    if (mStreamBatchBufs != NULL) {
        for (uint8_t i = 0; i < mStreamBatchBufs->getCnt(); i++) {
            unmapBuf(CAM_MAPPING_BUF_TYPE_STREAM_USER_BUF, i, -1, ops_tbl);
        }
        mStreamBatchBufs->deallocate();
        delete mStreamBatchBufs;
        mStreamBatchBufs = NULL;
    }
    return rc;

}

/*===========================================================================
 * FUNCTION   : BufAllocRoutine
 *
 * DESCRIPTION: function to allocate additional stream buffers
 *
 * PARAMETERS :
 *   @data    : user data ptr
 *
 * RETURN     : none
 *==========================================================================*/
void *QCameraStream::BufAllocRoutine(void *data)
{
    QCameraStream *pme = (QCameraStream *)data;
    int32_t rc = NO_ERROR;

    CDBG_HIGH("%s: E", __func__);
    pme->cond_wait();
    if (pme->mNumBufsNeedAlloc > 0) {
        uint8_t numBufAlloc = (uint8_t)(pme->mNumBufs - pme->mNumBufsNeedAlloc);
        rc = pme->mAllocator.allocateMoreStreamBuf(pme->mStreamBufs,
                                                   pme->mFrameLenOffset.frame_len,
                                                   pme->mNumBufsNeedAlloc);
        if (rc != NO_ERROR) {
            ALOGE("%s: Failed to allocate buffers, __func__");
            pme->mNumBufsNeedAlloc = 0;
            return NULL;
        }

        pme->mNumBufsNeedAlloc = 0;
        QCameraBufferMaps bufferMaps;
        for (uint32_t i = numBufAlloc; i < pme->mNumBufs; i++) {
            ssize_t bufSize = pme->mStreamBufs->getSize(i);
            if (BAD_INDEX == bufSize) {
                ALOGE("%s: Failed to retrieve buffer size (bad index)", __func__);
                return NULL;
            }

            rc = bufferMaps.enqueue(CAM_MAPPING_BUF_TYPE_STREAM_BUF,
                    pme->mHandle, i /*buf index*/, -1 /*plane index*/,
                    0 /*cookie*/, pme->mStreamBufs->getFd(i), bufSize);

            if (rc < 0) {
                ALOGE("%s: Failed to map buffers", __func__);
                return NULL;
            }
        }

        cam_buf_map_type_list bufMapList;
        rc = bufferMaps.getCamBufMapList(bufMapList);
        if (rc == NO_ERROR) {
            rc = pme->m_MemOpsTbl.bundled_map_ops(&bufMapList, pme->m_MemOpsTbl.userdata);
        }
        if (rc != 0) {
            ALOGE("%s: Failed to map buffers with return code %d", __func__, rc);
            return NULL;
        }

        for (uint32_t i = numBufAlloc; i < pme->mNumBufs; i++) {
            pme->mStreamBufs->getBufDef(pme->mFrameLenOffset, pme->mBufDefs[i], i);
            pme->mCamOps->qbuf(pme->mCamHandle, pme->mChannelHandle,
                    &pme->mBufDefs[i]);
        }
    }
    CDBG_HIGH("%s: X", __func__);
    return NULL;
}

/*===========================================================================
 * FUNCTION   : cond_signal
 *
 * DESCRIPTION: signal if flag "wait_for_cond" is set
 *
 *==========================================================================*/
void QCameraStream::cond_signal(bool forceExit)
{
    pthread_mutex_lock(&m_lock);
    if(wait_for_cond == TRUE){
        wait_for_cond = FALSE;
        if (forceExit) {
            mNumBufsNeedAlloc = 0;
        }
        pthread_cond_signal(&m_cond);
    }
    pthread_mutex_unlock(&m_lock);
}


/*===========================================================================
 * FUNCTION   : cond_wait
 *
 * DESCRIPTION: wait on if flag "wait_for_cond" is set
 *
 *==========================================================================*/
void QCameraStream::cond_wait()
{
    pthread_mutex_lock(&m_lock);
    while (wait_for_cond == TRUE) {
        pthread_cond_wait(&m_cond, &m_lock);
    }
    pthread_mutex_unlock(&m_lock);
}

/*===========================================================================
 * FUNCTION   : putBufs
 *
 * DESCRIPTION: deallocate stream buffers
 *
 * PARAMETERS :
 *   @ops_tbl    : ptr to buf mapping/unmapping ops
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::putBufs(mm_camera_map_unmap_ops_tbl_t *ops_tbl)
{
    int rc = NO_ERROR;

    if (mBufAllocPid != 0) {
        cond_signal(true);
        CDBG_HIGH("%s: wait for buf allocation thread dead", __func__);
        pthread_join(mBufAllocPid, NULL);
        mBufAllocPid = 0;
        CDBG_HIGH("%s: return from buf allocation thread", __func__);
    }

    uint8_t numBufsToUnmap = mStreamBufs->getMappable();
    for (uint32_t i = 0; i < numBufsToUnmap; i++) {
        rc = ops_tbl->unmap_ops(i, -1, CAM_MAPPING_BUF_TYPE_STREAM_BUF, ops_tbl->userdata);
        if (rc < 0) {
            ALOGE("%s: map_stream_buf failed: %d", __func__, rc);
        }
    }
    mBufDefs = NULL; // mBufDefs just keep a ptr to the buffer
                     // mm-camera-interface own the buffer, so no need to free
    memset(&mFrameLenOffset, 0, sizeof(mFrameLenOffset));
    if ( !mStreamBufsAcquired ) {
        mStreamBufs->deallocate();
        delete mStreamBufs;
        mStreamBufs = NULL;
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : invalidateBuf
 *
 * DESCRIPTION: invalidate a specific stream buffer
 *
 * PARAMETERS :
 *   @index   : index of the buffer to invalidate
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::invalidateBuf(uint32_t index)
{
    return mStreamBufs->invalidateCache(index);
}

/*===========================================================================
 * FUNCTION   : cleanInvalidateBuf
 *
 * DESCRIPTION: clean invalidate a specific stream buffer
 *
 * PARAMETERS :
 *   @index   : index of the buffer to clean invalidate
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::cleanInvalidateBuf(uint32_t index)
{
    return mStreamBufs->cleanInvalidateCache(index);
}

/*===========================================================================
 * FUNCTION   : isTypeOf
 *
 * DESCRIPTION: helper function to determine if the stream is of the queried type
 *
 * PARAMETERS :
 *   @type    : stream type as of queried
 *
 * RETURN     : true/false
 *==========================================================================*/
bool QCameraStream::isTypeOf(cam_stream_type_t type)
{
    if (mStreamInfo != NULL && (mStreamInfo->stream_type == type)) {
        return true;
    } else {
        return false;
    }
}

/*===========================================================================
 * FUNCTION   : isOrignalTypeOf
 *
 * DESCRIPTION: helper function to determine if the original stream is of the
 *              queried type if it's reproc stream
 *
 * PARAMETERS :
 *   @type    : stream type as of queried
 *
 * RETURN     : true/false
 *==========================================================================*/
bool QCameraStream::isOrignalTypeOf(cam_stream_type_t type)
{
    if (mStreamInfo != NULL &&
        mStreamInfo->stream_type == CAM_STREAM_TYPE_OFFLINE_PROC &&
        mStreamInfo->reprocess_config.pp_type == CAM_ONLINE_REPROCESS_TYPE &&
        mStreamInfo->reprocess_config.online.input_stream_type == type) {
        return true;
    } else if (
        mStreamInfo != NULL &&
        mStreamInfo->stream_type == CAM_STREAM_TYPE_OFFLINE_PROC &&
        mStreamInfo->reprocess_config.pp_type == CAM_OFFLINE_REPROCESS_TYPE &&
        mStreamInfo->reprocess_config.offline.input_type == type) {
        return true;
    } else {
        return false;
    }
}

/*===========================================================================
 * FUNCTION   : getMyType
 *
 * DESCRIPTION: return stream type
 *
 * PARAMETERS : none
 *
 * RETURN     : stream type
 *==========================================================================*/
cam_stream_type_t QCameraStream::getMyType()
{
    if (mStreamInfo != NULL) {
        return mStreamInfo->stream_type;
    } else {
        return CAM_STREAM_TYPE_DEFAULT;
    }
}

/*===========================================================================
 * FUNCTION   : getMyOriginalType
 *
 * DESCRIPTION: return stream type
 *
 * PARAMETERS : none
 *
 * RETURN     : stream type
 *==========================================================================*/
cam_stream_type_t QCameraStream::getMyOriginalType()
{
    if (mStreamInfo != NULL) {
        if (mStreamInfo->stream_type == CAM_STREAM_TYPE_OFFLINE_PROC &&
                mStreamInfo->reprocess_config.pp_type == CAM_ONLINE_REPROCESS_TYPE) {
            return mStreamInfo->reprocess_config.online.input_stream_type;
        } else if (mStreamInfo->stream_type == CAM_STREAM_TYPE_OFFLINE_PROC &&
                mStreamInfo->reprocess_config.pp_type == CAM_OFFLINE_REPROCESS_TYPE) {
            return mStreamInfo->reprocess_config.offline.input_type;
        } else {
            return mStreamInfo->stream_type;
        }
    } else {
        return CAM_STREAM_TYPE_DEFAULT;
    }
}

/*===========================================================================
 * FUNCTION   : getFrameOffset
 *
 * DESCRIPTION: query stream buffer frame offset info
 *
 * PARAMETERS :
 *   @offset  : reference to struct to store the queried frame offset info
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::getFrameOffset(cam_frame_len_offset_t &offset)
{
    if (NULL == mStreamInfo) {
        return NO_INIT;
    }

    offset = mFrameLenOffset;
    if ((ROTATE_90 == mOnlineRotation) || (ROTATE_270 == mOnlineRotation)) {
        // Re-calculate frame offset in case of online rotation
        cam_stream_info_t streamInfo = *mStreamInfo;
        getFrameDimension(streamInfo.dim);
        calcOffset(&streamInfo);
        offset = streamInfo.buf_planes.plane_info;
    }

    return 0;
}

/*===========================================================================
 * FUNCTION   : getCropInfo
 *
 * DESCRIPTION: query crop info of the stream
 *
 * PARAMETERS :
 *   @crop    : reference to struct to store the queried crop info
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::getCropInfo(cam_rect_t &crop)
{
    pthread_mutex_lock(&mCropLock);
    crop = mCropInfo;
    pthread_mutex_unlock(&mCropLock);
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : setCropInfo
 *
 * DESCRIPTION: set crop info of the stream
 *
 * PARAMETERS :
 *   @crop    : struct to store new crop info
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::setCropInfo(cam_rect_t crop)
{
    pthread_mutex_lock(&mCropLock);
    mCropInfo = crop;
    pthread_mutex_unlock(&mCropLock);
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : getFrameDimension
 *
 * DESCRIPTION: query stream frame dimension info
 *
 * PARAMETERS :
 *   @dim     : reference to struct to store the queried frame dimension
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::getFrameDimension(cam_dimension_t &dim)
{
    if (mStreamInfo != NULL) {
        if ((ROTATE_90 == mOnlineRotation) || (ROTATE_270 == mOnlineRotation)) {
            dim.width = mStreamInfo->dim.height;
            dim.height = mStreamInfo->dim.width;
        } else {
            dim = mStreamInfo->dim;
        }
        return 0;
    }
    return -1;
}

/*===========================================================================
 * FUNCTION   : getFormat
 *
 * DESCRIPTION: query stream format
 *
 * PARAMETERS :
 *   @fmt     : reference to stream format
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::getFormat(cam_format_t &fmt)
{
    if (mStreamInfo != NULL) {
        fmt = mStreamInfo->fmt;
        return 0;
    }
    return -1;
}

/*===========================================================================
 * FUNCTION   : getMyServerID
 *
 * DESCRIPTION: query server stream ID
 *
 * PARAMETERS : None
 *
 * RETURN     : stream ID from server
 *==========================================================================*/
uint32_t QCameraStream::getMyServerID() {
    if (mStreamInfo != NULL) {
        return mStreamInfo->stream_svr_id;
    } else {
        return 0;
    }
}

/*===========================================================================
 * FUNCTION   : acquireStreamBufs
 *
 * DESCRIPTION: acquire stream buffers and postpone their release.
 *
 * PARAMETERS : None
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::acquireStreamBufs()
{
    mStreamBufsAcquired = true;

    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : mapBuf
 *
 * DESCRIPTION: map stream related buffer to backend server
 *
 * PARAMETERS :
 *   @buf_type : mapping type of buffer
 *   @buf_idx  : index of buffer
 *   @plane_idx: plane index
 *   @fd       : fd of the buffer
 *   @size     : lenght of the buffer
 *   @ops_tbl    : ptr to buf mapping/unmapping ops
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::mapBuf(uint8_t buf_type, uint32_t buf_idx,
        int32_t plane_idx, int fd, size_t size, mm_camera_map_unmap_ops_tbl_t *ops_tbl)
{
    cam_buf_map_type_list bufMapList;
    int32_t rc = QCameraBufferMaps::makeSingletonBufMapList(
           (cam_mapping_buf_type)buf_type, mHandle, buf_idx, plane_idx,
           0 /*cookie*/, fd, size, bufMapList);

    if (rc != NO_ERROR) {
        return rc;
    }

    return mapBufs(bufMapList, ops_tbl);
}

/*===========================================================================
 * FUNCTION   : mapBufs
 *
 * DESCRIPTION: map stream related buffers to backend server
 *
 * PARAMETERS :
 *   @bufMapList : buffer mapping information
 *   @ops_tbl    : ptr to buf mapping/unmapping ops
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/

int32_t QCameraStream::mapBufs(cam_buf_map_type_list bufMapList,
        mm_camera_map_unmap_ops_tbl_t *ops_tbl)
{
    if (m_MemOpsTbl.bundled_map_ops != NULL) {
        return m_MemOpsTbl.bundled_map_ops(&bufMapList, m_MemOpsTbl.userdata);
    } else {
        return mCamOps->map_stream_bufs(mCamHandle, mChannelHandle,
                &bufMapList);
    }

}

/*===========================================================================
 * FUNCTION   : unmapBuf
 *
 * DESCRIPTION: unmap stream related buffer to backend server
 *
 * PARAMETERS :
 *   @buf_type : mapping type of buffer
 *   @buf_idx  : index of buffer
 *   @plane_idx: plane index
 *   @ops_tbl    : ptr to buf mapping/unmapping ops
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::unmapBuf(uint8_t buf_type, uint32_t buf_idx, int32_t plane_idx,
        mm_camera_map_unmap_ops_tbl_t *ops_tbl)
{
    if (ops_tbl != NULL) {
        return ops_tbl->unmap_ops(buf_idx, plane_idx,
                (cam_mapping_buf_type)buf_type, ops_tbl->userdata);
    } else {
        return mCamOps->unmap_stream_buf(mCamHandle, mChannelHandle,
                mHandle, buf_type, buf_idx, plane_idx);
    }
}

/*===========================================================================
 * FUNCTION   : setParameter
 *
 * DESCRIPTION: set stream based parameters
 *
 * PARAMETERS :
 *   @param   : ptr to parameters to be set
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::setParameter(cam_stream_parm_buffer_t &param)
{
    int32_t rc = NO_ERROR;
    pthread_mutex_lock(&mParameterLock);
    mStreamInfo->parm_buf = param;
    rc = mCamOps->set_stream_parms(mCamHandle,
                                   mChannelHandle,
                                   mHandle,
                                   &mStreamInfo->parm_buf);
    if (rc == NO_ERROR) {
        param = mStreamInfo->parm_buf;
    }
    pthread_mutex_unlock(&mParameterLock);
    return rc;
}

/*===========================================================================
 * FUNCTION   : getParameter
 *
 * DESCRIPTION: get stream based parameters
 *
 * PARAMETERS :
 *   @param   : ptr to parameters to be red
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::getParameter(cam_stream_parm_buffer_t &param)
{
    int32_t rc = NO_ERROR;
    pthread_mutex_lock(&mParameterLock);
    mStreamInfo->parm_buf = param;
    rc = mCamOps->get_stream_parms(mCamHandle,
                                   mChannelHandle,
                                   mHandle,
                                   &mStreamInfo->parm_buf);
    if (rc == NO_ERROR) {
        param = mStreamInfo->parm_buf;
    }
    pthread_mutex_unlock(&mParameterLock);
    return rc;
}

/*===========================================================================
 * FUNCTION   : releaseFrameData
 *
 * DESCRIPTION: callback function to release frame data node
 *
 * PARAMETERS :
 *   @data      : ptr to post process input data
 *   @user_data : user data ptr (QCameraReprocessor)
 *
 * RETURN     : None
 *==========================================================================*/
void QCameraStream::releaseFrameData(void *data, void *user_data)
{
    QCameraStream *pme = (QCameraStream *)user_data;
    mm_camera_super_buf_t *frame = (mm_camera_super_buf_t *)data;
    if (NULL != pme) {
        pme->bufDone(frame->bufs[0]->buf_idx);
    }
}

/*===========================================================================
 * FUNCTION   : configStream
 *
 * DESCRIPTION: send stream configuration to back end
 *
 * PARAMETERS :
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraStream::configStream()
{
    int rc = NO_ERROR;

    // Configure the stream
    mm_camera_stream_config_t stream_config;
    stream_config.stream_info = mStreamInfo;
    stream_config.mem_vtbl = mMemVtbl;
    stream_config.stream_cb_sync = NULL;
    stream_config.stream_cb = dataNotifyCB;
    stream_config.padding_info = mPaddingInfo;
    stream_config.userdata = this;
    rc = mCamOps->config_stream(mCamHandle,
                mChannelHandle, mHandle, &stream_config);
    if (rc < 0) {
        ALOGE("Failed to config stream, rc = %d", rc);
        mCamOps->unmap_stream_buf(mCamHandle,
                mChannelHandle,
                mHandle,
                CAM_MAPPING_BUF_TYPE_STREAM_INFO,
                0,
                -1);
        return UNKNOWN_ERROR;
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : setSyncDataCB
 *
 * DESCRIPTION: register callback with mm-interface for this stream
 *
 * PARAMETERS :
       @stream_cb   : Callback function
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              non-zero failure code
 *==========================================================================*/
int32_t QCameraStream::setSyncDataCB(stream_cb_routine data_cb)
{
    if (mCamOps != NULL) {
        mSYNCDataCB = data_cb;
        return mCamOps->register_stream_buf_cb(mCamHandle,
                mChannelHandle, mHandle, dataNotifySYNCCB, MM_CAMERA_STREAM_CB_TYPE_SYNC,
                this);
    }
    ALOGE("%s: Interface handle is NULL", __func__);
    return UNKNOWN_ERROR;
}

}; // namespace qcamera
