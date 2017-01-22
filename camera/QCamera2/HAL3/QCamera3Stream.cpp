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

#define LOG_TAG "QCamera3Stream"
//#define LOG_NDEBUG 0

#include <utils/Log.h>
#include <utils/Errors.h>
#include "QCamera3HWI.h"
#include "QCamera3Stream.h"
#include "QCamera3Channel.h"

using namespace android;

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
int32_t QCamera3Stream::get_bufs(
                     cam_frame_len_offset_t *offset,
                     uint8_t *num_bufs,
                     uint8_t **initial_reg_flag,
                     mm_camera_buf_def_t **bufs,
                     mm_camera_map_unmap_ops_tbl_t *ops_tbl,
                     void *user_data)
{
    QCamera3Stream *stream = reinterpret_cast<QCamera3Stream *>(user_data);
    if (!stream) {
        ALOGE("getBufs invalid stream pointer");
        return NO_MEMORY;
    }
    return stream->getBufs(offset, num_bufs, initial_reg_flag, bufs, ops_tbl);
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
int32_t QCamera3Stream::put_bufs(
                     mm_camera_map_unmap_ops_tbl_t *ops_tbl,
                     void *user_data)
{
    QCamera3Stream *stream = reinterpret_cast<QCamera3Stream *>(user_data);
    if (!stream) {
        ALOGE("putBufs invalid stream pointer");
        return NO_MEMORY;
    }
    return stream->putBufs(ops_tbl);
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
int32_t QCamera3Stream::invalidate_buf(uint32_t index, void *user_data)
{
    QCamera3Stream *stream = reinterpret_cast<QCamera3Stream *>(user_data);
    if (!stream) {
        ALOGE("invalid stream pointer");
        return NO_MEMORY;
    }
    return stream->invalidateBuf(index);
}

/*===========================================================================
 * FUNCTION   : clean_invalidate_buf
 *
 * DESCRIPTION: static function entry to clean and invalidate a specific stream buffer
 *
 * PARAMETERS :
 *   @index      : index of the stream buffer to invalidate
 *   @user_data  : user data ptr of ops_tbl
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3Stream::clean_invalidate_buf(uint32_t index, void *user_data)
{
    QCamera3Stream *stream = reinterpret_cast<QCamera3Stream *>(user_data);
    if (!stream) {
        ALOGE("invalid stream pointer");
        return NO_MEMORY;
    }
    return stream->cleanInvalidateBuf(index);
}

/*===========================================================================
 * FUNCTION   : QCamera3Stream
 *
 * DESCRIPTION: constructor of QCamera3Stream
 *
 * PARAMETERS :
 *   @allocator  : memory allocator obj
 *   @camHandle  : camera handle
 *   @chId       : channel handle
 *   @camOps     : ptr to camera ops table
 *   @paddingInfo: ptr to padding info
 *
 * RETURN     : None
 *==========================================================================*/
QCamera3Stream::QCamera3Stream(uint32_t camHandle,
                             uint32_t chId,
                             mm_camera_ops_t *camOps,
                             cam_padding_info_t *paddingInfo,
                             QCamera3Channel *channel) :
        mCamHandle(camHandle),
        mChannelHandle(chId),
        mHandle(0),
        mCamOps(camOps),
        mStreamInfo(NULL),
        mMemOps(NULL),
        mNumBufs(0),
        mDataCB(NULL),
        mUserData(NULL),
        mDataQ(releaseFrameData, this),
        mStreamInfoBuf(NULL),
        mStreamBufs(NULL),
        mBufDefs(NULL),
        mChannel(channel)
{
    mMemVtbl.user_data = this;
    mMemVtbl.get_bufs = get_bufs;
    mMemVtbl.put_bufs = put_bufs;
    mMemVtbl.invalidate_buf = invalidate_buf;
    mMemVtbl.clean_invalidate_buf = clean_invalidate_buf;
    mMemVtbl.set_config_ops = NULL;
    memset(&mFrameLenOffset, 0, sizeof(mFrameLenOffset));
    memcpy(&mPaddingInfo, paddingInfo, sizeof(cam_padding_info_t));
}

/*===========================================================================
 * FUNCTION   : ~QCamera3Stream
 *
 * DESCRIPTION: deconstructor of QCamera3Stream
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *==========================================================================*/
QCamera3Stream::~QCamera3Stream()
{
    if (mStreamInfoBuf != NULL) {
        int rc = mCamOps->unmap_stream_buf(mCamHandle,
                    mChannelHandle, mHandle, CAM_MAPPING_BUF_TYPE_STREAM_INFO, 0, -1);
        if (rc < 0) {
            ALOGE("Failed to un-map stream info buffer");
        }
        mStreamInfoBuf->deallocate();
        delete mStreamInfoBuf;
        mStreamInfoBuf = NULL;
    }

    // delete stream
    if (mHandle > 0) {
        mCamOps->delete_stream(mCamHandle, mChannelHandle, mHandle);
        mHandle = 0;
    }
}

/*===========================================================================
 * FUNCTION   : init
 *
 * DESCRIPTION: initialize stream obj
 *
 * PARAMETERS :
 *   @streamType     : stream type
 *   @streamFormat   : stream format
 *   @streamDim      : stream dimension
 *   @reprocess_config: reprocess stream input configuration
 *   @minNumBuffers  : minimal buffer count for particular stream type
 *   @stream_cb      : callback handle
 *   @userdata       : user data
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3Stream::init(cam_stream_type_t streamType,
                            cam_format_t streamFormat,
                            cam_dimension_t streamDim,
                            cam_stream_reproc_config_t* reprocess_config,
                            uint8_t minNumBuffers,
                            uint32_t postprocess_mask,
                            cam_is_type_t is_type,
                            hal3_stream_cb_routine stream_cb,
                            void *userdata)
{
    int32_t rc = OK;
    ssize_t bufSize = BAD_INDEX;
    mm_camera_stream_config_t stream_config;

    mHandle = mCamOps->add_stream(mCamHandle, mChannelHandle);
    if (!mHandle) {
        ALOGE("add_stream failed");
        rc = UNKNOWN_ERROR;
        goto done;
    }

    // allocate and map stream info memory
    mStreamInfoBuf = new QCamera3HeapMemory();
    if (mStreamInfoBuf == NULL) {
        ALOGE("%s: no memory for stream info buf obj", __func__);
        rc = -ENOMEM;
        goto err1;
    }
    rc = mStreamInfoBuf->allocate(1, sizeof(cam_stream_info_t), false);
    if (rc < 0) {
        ALOGE("%s: no memory for stream info", __func__);
        rc = -ENOMEM;
        goto err2;
    }

    mStreamInfo =
        reinterpret_cast<cam_stream_info_t *>(mStreamInfoBuf->getPtr(0));
    memset(mStreamInfo, 0, sizeof(cam_stream_info_t));
    mStreamInfo->stream_type = streamType;
    mStreamInfo->fmt = streamFormat;
    mStreamInfo->dim = streamDim;
    mStreamInfo->num_bufs = minNumBuffers;
    mStreamInfo->pp_config.feature_mask = postprocess_mask;
    mStreamInfo->is_type = is_type;
    ALOGV("%s: stream_type is %d, feature_mask is %d",
          __func__, mStreamInfo->stream_type, mStreamInfo->pp_config.feature_mask);

    bufSize = mStreamInfoBuf->getSize(0);
    if (BAD_INDEX != bufSize) {
        rc = mCamOps->map_stream_buf(mCamHandle,
                mChannelHandle, mHandle, CAM_MAPPING_BUF_TYPE_STREAM_INFO,
                0, -1, mStreamInfoBuf->getFd(0), (size_t)bufSize);
        if (rc < 0) {
            ALOGE("Failed to map stream info buffer");
            goto err3;
        }
    } else {
        ALOGE("Failed to retrieve buffer size (bad index)");
        goto err3;
    }

    mNumBufs = minNumBuffers;
    if (reprocess_config != NULL) {
       mStreamInfo->reprocess_config = *reprocess_config;
       mStreamInfo->streaming_mode = CAM_STREAMING_MODE_BURST;
       //mStreamInfo->num_of_burst = reprocess_config->offline.num_of_bufs;
       mStreamInfo->num_of_burst = 1;
       ALOGI("%s: num_of_burst is %d", __func__, mStreamInfo->num_of_burst);
    } else {
       mStreamInfo->streaming_mode = CAM_STREAMING_MODE_CONTINUOUS;
    }

    // Configure the stream
    stream_config.stream_info = mStreamInfo;
    stream_config.mem_vtbl = mMemVtbl;
    stream_config.padding_info = mPaddingInfo;
    stream_config.userdata = this;
    stream_config.stream_cb = dataNotifyCB;
    stream_config.stream_cb_sync = NULL;

    rc = mCamOps->config_stream(mCamHandle,
            mChannelHandle, mHandle, &stream_config);
    if (rc < 0) {
        ALOGE("Failed to config stream, rc = %d", rc);
        goto err4;
    }

    mDataCB = stream_cb;
    mUserData = userdata;
    return 0;

err4:
    mCamOps->unmap_stream_buf(mCamHandle,
            mChannelHandle, mHandle, CAM_MAPPING_BUF_TYPE_STREAM_INFO, 0, -1);
err3:
    mStreamInfoBuf->deallocate();
err2:
    delete mStreamInfoBuf;
    mStreamInfoBuf = NULL;
    mStreamInfo = NULL;
err1:
    mCamOps->delete_stream(mCamHandle, mChannelHandle, mHandle);
    mHandle = 0;
    mNumBufs = 0;
done:
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
int32_t QCamera3Stream::start()
{
    int32_t rc = 0;

    mDataQ.init();
    rc = mProcTh.launch(dataProcRoutine, this);
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
int32_t QCamera3Stream::stop()
{
    int32_t rc = 0;
    rc = mProcTh.exit();
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
int32_t QCamera3Stream::processDataNotify(mm_camera_super_buf_t *frame)
{
    CDBG("%s: E\n", __func__);
    int32_t rc;
    if (mDataQ.enqueue((void *)frame)) {
        rc = mProcTh.sendCmd(CAMERA_CMD_TYPE_DO_NEXT_JOB, FALSE, FALSE);
    } else {
        ALOGD("%s: Stream thread is not active, no ops here", __func__);
        bufDone(frame->bufs[0]->buf_idx);
        free(frame);
        rc = NO_ERROR;
    }
    CDBG("%s: X\n", __func__);
    return rc;
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
void QCamera3Stream::dataNotifyCB(mm_camera_super_buf_t *recvd_frame,
                                 void *userdata)
{
    CDBG("%s: E\n", __func__);
    QCamera3Stream* stream = (QCamera3Stream *)userdata;
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
void *QCamera3Stream::dataProcRoutine(void *data)
{
    int running = 1;
    int ret;
    QCamera3Stream *pme = (QCamera3Stream *)data;
    QCameraCmdThread *cmdThread = &pme->mProcTh;
    cmdThread->setName("cam_stream_proc");

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
                CDBG("%s: Do next job", __func__);
                mm_camera_super_buf_t *frame =
                    (mm_camera_super_buf_t *)pme->mDataQ.dequeue();
                if (NULL != frame) {
                    if (pme->mDataCB != NULL) {
                        pme->mDataCB(frame, pme, pme->mUserData);
                    } else {
                        // no data cb routine, return buf here
                        pme->bufDone(frame->bufs[0]->buf_idx);
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
    CDBG("%s: X", __func__);
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
int32_t QCamera3Stream::bufDone(uint32_t index)
{
    int32_t rc = NO_ERROR;
    Mutex::Autolock lock(mLock);

    if ((index >= mNumBufs) || (mBufDefs == NULL)) {
        return BAD_INDEX;
    }

    if( NULL == mBufDefs[index].mem_info) {
        if (NULL == mMemOps) {
            ALOGE("%s: Camera operations not initialized", __func__);
            return NO_INIT;
        }

        ssize_t bufSize = mStreamBufs->getSize(index);

        if (BAD_INDEX != bufSize) {
            rc = mMemOps->map_ops(index, -1, mStreamBufs->getFd(index),
                    (size_t)bufSize, CAM_MAPPING_BUF_TYPE_STREAM_BUF, mMemOps->userdata);
            if (rc < 0) {
                ALOGE("%s: Failed to map camera buffer %d", __func__, index);
                return rc;
            }

            rc = mStreamBufs->getBufDef(mFrameLenOffset, mBufDefs[index], index);
            if (NO_ERROR != rc) {
                ALOGE("%s: Couldn't find camera buffer definition", __func__);
                mMemOps->unmap_ops(index, -1, CAM_MAPPING_BUF_TYPE_STREAM_BUF, mMemOps->userdata);
                return rc;
            }
        } else {
            ALOGE("Failed to retrieve buffer size (bad index)");
            return INVALID_OPERATION;
        }
    }

    rc = mCamOps->qbuf(mCamHandle, mChannelHandle, &mBufDefs[index]);
    if (rc < 0) {
        return FAILED_TRANSACTION;
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : bufRelease
 *
 * DESCRIPTION: release all resources associated with this buffer
 *
 * PARAMETERS :
 *   @index   : index of buffer to be released
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3Stream::bufRelease(int32_t index)
{
    int32_t rc = NO_ERROR;
    Mutex::Autolock lock(mLock);

    if ((index >= mNumBufs) || (mBufDefs == NULL)) {
        return BAD_INDEX;
    }

    if (NULL != mBufDefs[index].mem_info) {
        if (NULL == mMemOps) {
            ALOGE("%s: Camera operations not initialized", __func__);
            return NO_INIT;
        }

        rc = mMemOps->unmap_ops(index, -1, CAM_MAPPING_BUF_TYPE_STREAM_BUF,
                mMemOps->userdata);
        if (rc < 0) {
            ALOGE("%s: Failed to un-map camera buffer %d", __func__, index);
            return rc;
        }

        mBufDefs[index].mem_info = NULL;
    } else {
        ALOGE("%s: Buffer at index %d not registered", __func__);
        return BAD_INDEX;
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
int32_t QCamera3Stream::getBufs(cam_frame_len_offset_t *offset,
                     uint8_t *num_bufs,
                     uint8_t **initial_reg_flag,
                     mm_camera_buf_def_t **bufs,
                     mm_camera_map_unmap_ops_tbl_t *ops_tbl)
{
    int rc = NO_ERROR;
    uint8_t *regFlags;
    Mutex::Autolock lock(mLock);

    if (!ops_tbl) {
        ALOGE("%s: ops_tbl is NULL", __func__);
        return INVALID_OPERATION;
    }

    mFrameLenOffset = *offset;
    mMemOps = ops_tbl;

    mStreamBufs = mChannel->getStreamBufs(mFrameLenOffset.frame_len);
    if (!mStreamBufs) {
        ALOGE("%s: Failed to allocate stream buffers", __func__);
        return NO_MEMORY;
    }

    uint32_t registeredBuffers = mStreamBufs->getCnt();
    for (uint32_t i = 0; i < registeredBuffers; i++) {
        ssize_t bufSize = mStreamBufs->getSize(i);
        if (BAD_INDEX != bufSize) {
            rc = ops_tbl->map_ops(i, -1, mStreamBufs->getFd(i),
                    (size_t)bufSize, CAM_MAPPING_BUF_TYPE_STREAM_BUF, ops_tbl->userdata);
            if (rc < 0) {
                ALOGE("%s: map_stream_buf failed: %d", __func__, rc);
                for (uint32_t j = 0; j < i; j++) {
                    ops_tbl->unmap_ops(j, -1, CAM_MAPPING_BUF_TYPE_STREAM_BUF, ops_tbl->userdata);
                }
                return INVALID_OPERATION;
            }
        } else {
            ALOGE("Failed to retrieve buffer size (bad index)");
            return INVALID_OPERATION;
        }
    }

    //regFlags array is allocated by us, but consumed and freed by mm-camera-interface
    regFlags = (uint8_t *)malloc(sizeof(uint8_t) * mNumBufs);
    if (!regFlags) {
        ALOGE("%s: Out of memory", __func__);
        for (uint32_t i = 0; i < registeredBuffers; i++) {
            ops_tbl->unmap_ops(i, -1, CAM_MAPPING_BUF_TYPE_STREAM_BUF, ops_tbl->userdata);
        }
        return NO_MEMORY;
    }
    memset(regFlags, 0, sizeof(uint8_t) * mNumBufs);

    mBufDefs = (mm_camera_buf_def_t *)malloc(mNumBufs * sizeof(mm_camera_buf_def_t));
    if (mBufDefs == NULL) {
        ALOGE("%s: Failed to allocate mm_camera_buf_def_t %d", __func__, rc);
        for (uint32_t i = 0; i < registeredBuffers; i++) {
            ops_tbl->unmap_ops(i, -1, CAM_MAPPING_BUF_TYPE_STREAM_BUF, ops_tbl->userdata);
        }
        free(regFlags);
        regFlags = NULL;
        return INVALID_OPERATION;
    }
    memset(mBufDefs, 0, mNumBufs * sizeof(mm_camera_buf_def_t));
    for (uint32_t i = 0; i < registeredBuffers; i++) {
        mStreamBufs->getBufDef(mFrameLenOffset, mBufDefs[i], i);
    }

    rc = mStreamBufs->getRegFlags(regFlags);
    if (rc < 0) {
        ALOGE("%s: getRegFlags failed %d", __func__, rc);
        for (uint32_t i = 0; i < registeredBuffers; i++) {
            ops_tbl->unmap_ops(i, -1, CAM_MAPPING_BUF_TYPE_STREAM_BUF, ops_tbl->userdata);
        }
        free(mBufDefs);
        mBufDefs = NULL;
        free(regFlags);
        regFlags = NULL;
        return INVALID_OPERATION;
    }

    *num_bufs = mNumBufs;
    *initial_reg_flag = regFlags;
    *bufs = mBufDefs;
    return NO_ERROR;
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
int32_t QCamera3Stream::putBufs(mm_camera_map_unmap_ops_tbl_t *ops_tbl)
{
    int rc = NO_ERROR;
    Mutex::Autolock lock(mLock);

    for (uint32_t i = 0; i < mNumBufs; i++) {
        if (NULL != mBufDefs[i].mem_info) {
            rc = ops_tbl->unmap_ops(i, -1, CAM_MAPPING_BUF_TYPE_STREAM_BUF, ops_tbl->userdata);
            if (rc < 0) {
                ALOGE("%s: un-map stream buf failed: %d", __func__, rc);
            }
        }
    }
    mBufDefs = NULL; // mBufDefs just keep a ptr to the buffer
                     // mm-camera-interface own the buffer, so no need to free
    memset(&mFrameLenOffset, 0, sizeof(mFrameLenOffset));
    mChannel->putStreamBufs();

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
int32_t QCamera3Stream::invalidateBuf(uint32_t index)
{
    return mStreamBufs->invalidateCache(index);
}

/*===========================================================================
 * FUNCTION   : cleanInvalidateBuf
 *
 * DESCRIPTION: clean and invalidate a specific stream buffer
 *
 * PARAMETERS :
 *   @index   : index of the buffer to invalidate
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3Stream::cleanInvalidateBuf(uint32_t index)
{
    return mStreamBufs->cleanInvalidateCache(index);
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
int32_t QCamera3Stream::getFrameOffset(cam_frame_len_offset_t &offset)
{
    offset = mFrameLenOffset;
    return 0;
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
int32_t QCamera3Stream::getFrameDimension(cam_dimension_t &dim)
{
    if (mStreamInfo != NULL) {
        dim = mStreamInfo->dim;
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
int32_t QCamera3Stream::getFormat(cam_format_t &fmt)
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
uint32_t QCamera3Stream::getMyServerID() {
    if (mStreamInfo != NULL) {
        return mStreamInfo->stream_svr_id;
    } else {
        return 0;
    }
}

/*===========================================================================
 * FUNCTION   : getMyType
 *
 * DESCRIPTION: query stream type
 *
 * PARAMETERS : None
 *
 * RETURN     : type of stream
 *==========================================================================*/
cam_stream_type_t QCamera3Stream::getMyType() const
{
    if (mStreamInfo != NULL) {
        return mStreamInfo->stream_type;
    } else {
        return CAM_STREAM_TYPE_MAX;
    }
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
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3Stream::mapBuf(uint8_t buf_type, uint32_t buf_idx,
        int32_t plane_idx, int fd, size_t size)
{
    return mCamOps->map_stream_buf(mCamHandle, mChannelHandle,
                                   mHandle, buf_type,
                                   buf_idx, plane_idx,
                                   fd, size);

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
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3Stream::unmapBuf(uint8_t buf_type, uint32_t buf_idx, int32_t plane_idx)
{
    return mCamOps->unmap_stream_buf(mCamHandle, mChannelHandle,
                                     mHandle, buf_type,
                                     buf_idx, plane_idx);
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
int32_t QCamera3Stream::setParameter(cam_stream_parm_buffer_t &param)
{
    int32_t rc = NO_ERROR;
    mStreamInfo->parm_buf = param;
    rc = mCamOps->set_stream_parms(mCamHandle,
                                   mChannelHandle,
                                   mHandle,
                                   &mStreamInfo->parm_buf);
    if (rc == NO_ERROR) {
        param = mStreamInfo->parm_buf;
    }
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
void QCamera3Stream::releaseFrameData(void *data, void *user_data)
{
    QCamera3Stream *pme = (QCamera3Stream *)user_data;
    mm_camera_super_buf_t *frame = (mm_camera_super_buf_t *)data;
    if (NULL != pme) {
        pme->bufDone(frame->bufs[0]->buf_idx);
    }
}

}; // namespace qcamera
