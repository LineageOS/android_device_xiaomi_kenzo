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

#define LOG_TAG "QCamera3PostProc"
//#define LOG_NDEBUG 0

#include <stdlib.h>
#include <utils/Errors.h>

#include "QCamera3PostProc.h"
#include "QCamera3HWI.h"
#include "QCamera3Channel.h"
#include "QCamera3Stream.h"

namespace qcamera {

/*===========================================================================
 * FUNCTION   : QCamera3PostProcessor
 *
 * DESCRIPTION: constructor of QCamera3PostProcessor.
 *
 * PARAMETERS :
 *   @cam_ctrl : ptr to HWI object
 *
 * RETURN     : None
 *==========================================================================*/
QCamera3PostProcessor::QCamera3PostProcessor(QCamera3PicChannel* ch_ctrl)
    : m_parent(ch_ctrl),
      mJpegCB(NULL),
      mJpegUserData(NULL),
      mJpegClientHandle(0),
      mJpegSessionId(0),
      m_bThumbnailNeeded(TRUE),
      m_pReprocChannel(NULL),
      m_inputPPQ(releasePPInputData, this),
      m_inputFWKPPQ(NULL, this),
      m_ongoingPPQ(releaseOngoingPPData, this),
      m_inputJpegQ(releaseJpegData, this),
      m_ongoingJpegQ(releaseJpegData, this),
      m_inputRawQ(releasePPInputData, this),
      m_inputMetaQ(releaseMetadata, this),
      m_jpegSettingsQ(NULL, this)
{
    memset(&mJpegHandle, 0, sizeof(mJpegHandle));
    pthread_mutex_init(&mReprocJobLock, NULL);
}

/*===========================================================================
 * FUNCTION   : ~QCamera3PostProcessor
 *
 * DESCRIPTION: deconstructor of QCamera3PostProcessor.
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *==========================================================================*/
QCamera3PostProcessor::~QCamera3PostProcessor()
{
    pthread_mutex_destroy(&mReprocJobLock);
}

/*===========================================================================
 * FUNCTION   : init
 *
 * DESCRIPTION: initialization of postprocessor
 *
 * PARAMETERS :
 *   @jpeg_cb      : callback to handle jpeg event from mm-camera-interface
 *   @user_data    : user data ptr for jpeg callback
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3PostProcessor::init(QCamera3Memory* mMemory,
                                    jpeg_encode_callback_t jpeg_cb,
                                    uint32_t postprocess_mask,
                                    void *user_data)
{
    ATRACE_CALL();
    mJpegCB = jpeg_cb;
    mJpegUserData = user_data;
    mm_dimension max_size;

    if ((0 > m_parent->m_max_pic_dim.width) ||
            (0 > m_parent->m_max_pic_dim.height)) {
        ALOGE("%s : Negative dimension %dx%d", __func__,
                m_parent->m_max_pic_dim.width, m_parent->m_max_pic_dim.height);
        return BAD_VALUE;
    }

    //set max pic size
    memset(&max_size, 0, sizeof(mm_dimension));
    max_size.w = (uint32_t)m_parent->m_max_pic_dim.width;
    max_size.h = (uint32_t)m_parent->m_max_pic_dim.height;

    mJpegClientHandle = jpeg_open(&mJpegHandle, NULL, max_size, NULL);
    mJpegMem = mMemory;
    if(!mJpegClientHandle) {
        ALOGE("%s : jpeg_open did not work", __func__);
        return UNKNOWN_ERROR;
    }
    mPostProcMask = postprocess_mask;
    m_dataProcTh.launch(dataProcessRoutine, this);

    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : deinit
 *
 * DESCRIPTION: de-initialization of postprocessor
 *
 * PARAMETERS : None
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3PostProcessor::deinit()
{
    m_dataProcTh.exit();

    if (m_pReprocChannel != NULL) {
        m_pReprocChannel->stop();
        delete m_pReprocChannel;
        m_pReprocChannel = NULL;
    }

    if(mJpegClientHandle > 0) {
        int rc = mJpegHandle.close(mJpegClientHandle);
        CDBG_HIGH("%s: Jpeg closed, rc = %d, mJpegClientHandle = %x",
              __func__, rc, mJpegClientHandle);
        mJpegClientHandle = 0;
        memset(&mJpegHandle, 0, sizeof(mJpegHandle));
    }

    mJpegMem = NULL;

    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : start
 *
 * DESCRIPTION: start postprocessor. Data process thread and data notify thread
 *              will be launched.
 *
 * PARAMETERS :
 *   @pMemory       : memory object representing buffers to store JPEG.
 *   @config        : reprocess configuration
 *   @metadata      : metadata for the reprocessing
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *
 * NOTE       : if any reprocess is needed, a reprocess channel/stream
 *              will be started.
 *==========================================================================*/
int32_t QCamera3PostProcessor::start(const reprocess_config_t &config,
        metadata_buffer_t *metadata)
{
    int32_t rc = NO_ERROR;
    QCamera3HardwareInterface* hal_obj = (QCamera3HardwareInterface*)m_parent->mUserData;

    if (hal_obj->needReprocess(mPostProcMask)) {
        if (m_pReprocChannel != NULL) {
            m_pReprocChannel->stop();
            delete m_pReprocChannel;
            m_pReprocChannel = NULL;
        }

        // if reprocess is needed, start reprocess channel
        CDBG("%s: Setting input channel as pInputChannel", __func__);
        m_pReprocChannel = hal_obj->addOfflineReprocChannel(config, m_parent, metadata);
        if (m_pReprocChannel == NULL) {
            ALOGE("%s: cannot add reprocess channel", __func__);
            return UNKNOWN_ERROR;
        }

        rc = m_pReprocChannel->start();
        if (rc != 0) {
            ALOGE("%s: cannot start reprocess channel", __func__);
            delete m_pReprocChannel;
            m_pReprocChannel = NULL;
            return rc;
        }
    }
    m_dataProcTh.sendCmd(CAMERA_CMD_TYPE_START_DATA_PROC, TRUE, FALSE);

    return rc;
}

/*===========================================================================
 * FUNCTION   : stop
 *
 * DESCRIPTION: stop postprocessor. Data process and notify thread will be stopped.
 *
 * PARAMETERS : None
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *
 * NOTE       : reprocess channel will be stopped and deleted if there is any
 *==========================================================================*/
int32_t QCamera3PostProcessor::stop()
{
    m_dataProcTh.sendCmd(CAMERA_CMD_TYPE_STOP_DATA_PROC, TRUE, TRUE);

    if (m_pReprocChannel != NULL) {
        m_pReprocChannel->stop();
        delete m_pReprocChannel;
        m_pReprocChannel = NULL;
    }

    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : getFWKJpegEncodeConfig
 *
 * DESCRIPTION: function to prepare encoding job information
 *
 * PARAMETERS :
 *   @encode_parm   : param to be filled with encoding configuration
 *   @frame         : framework input buffer
 *   @jpeg_settings : jpeg settings to be applied for encoding
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3PostProcessor::getFWKJpegEncodeConfig(
        mm_jpeg_encode_params_t& encode_parm,
        qcamera_fwk_input_pp_data_t *frame,
        jpeg_settings_t *jpeg_settings)
{
    CDBG("%s : E", __func__);
    int32_t ret = NO_ERROR;

    if ((NULL == frame) || (NULL == jpeg_settings)) {
        return BAD_VALUE;
    }

    ssize_t bufSize = mJpegMem->getSize(jpeg_settings->out_buf_index);
    if (BAD_INDEX == bufSize) {
        ALOGE("%s: cannot retrieve buffer size for buffer %u", __func__,
                jpeg_settings->out_buf_index);
        return BAD_VALUE;
    }

    encode_parm.jpeg_cb = mJpegCB;
    encode_parm.userdata = mJpegUserData;

    if (jpeg_settings->thumbnail_size.width > 0 &&
            jpeg_settings->thumbnail_size.height > 0)
        m_bThumbnailNeeded = TRUE;
    else
        m_bThumbnailNeeded = FALSE;
    encode_parm.encode_thumbnail = m_bThumbnailNeeded;

    // get color format
    cam_format_t img_fmt = frame->reproc_config.stream_format;
    encode_parm.color_format = getColorfmtFromImgFmt(img_fmt);

    // get jpeg quality
    encode_parm.quality = jpeg_settings->jpeg_quality;
    if (encode_parm.quality <= 0) {
        encode_parm.quality = 85;
    }

    // get jpeg thumbnail quality
    encode_parm.thumb_quality = jpeg_settings->jpeg_thumb_quality;

    cam_frame_len_offset_t main_offset =
            frame->reproc_config.input_stream_plane_info.plane_info;

    encode_parm.num_src_bufs = 1;
    encode_parm.src_main_buf[0].index = 0;
    encode_parm.src_main_buf[0].buf_size = frame->input_buffer.frame_len;
    encode_parm.src_main_buf[0].buf_vaddr = (uint8_t *) frame->input_buffer.buffer;
    encode_parm.src_main_buf[0].fd = frame->input_buffer.fd;
    encode_parm.src_main_buf[0].format = MM_JPEG_FMT_YUV;
    encode_parm.src_main_buf[0].offset = main_offset;

    //Pass input thumbnail buffer info to encoder.
    //Note: Use main buffer to encode thumbnail
    if (m_bThumbnailNeeded == TRUE) {
        encode_parm.num_tmb_bufs = 1;
        encode_parm.src_thumb_buf[0] = encode_parm.src_main_buf[0];
    }

    //Pass output jpeg buffer info to encoder.
    //mJpegMem is allocated by framework.
    encode_parm.num_dst_bufs = 1;
    encode_parm.dest_buf[0].index = 0;
    encode_parm.dest_buf[0].buf_size = (size_t)bufSize;
    encode_parm.dest_buf[0].buf_vaddr = (uint8_t *)mJpegMem->getPtr(
            jpeg_settings->out_buf_index);
    encode_parm.dest_buf[0].fd = mJpegMem->getFd(
            jpeg_settings->out_buf_index);
    encode_parm.dest_buf[0].format = MM_JPEG_FMT_YUV;
    encode_parm.dest_buf[0].offset = main_offset;

    CDBG("%s : X", __func__);
    return NO_ERROR;

on_error:
    CDBG("%s : X with error %d", __func__, ret);
    return ret;
}

/*===========================================================================
 * FUNCTION   : getJpegEncodeConfig
 *
 * DESCRIPTION: function to prepare encoding job information
 *
 * PARAMETERS :
 *   @encode_parm   : param to be filled with encoding configuration
 *   #main_stream   : stream object where the input buffer comes from
 *   @jpeg_settings : jpeg settings to be applied for encoding
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3PostProcessor::getJpegEncodeConfig(
                mm_jpeg_encode_params_t& encode_parm,
                QCamera3Stream *main_stream,
                jpeg_settings_t *jpeg_settings)
{
    CDBG("%s : E", __func__);
    int32_t ret = NO_ERROR;
    ssize_t bufSize = 0;

    encode_parm.jpeg_cb = mJpegCB;
    encode_parm.userdata = mJpegUserData;

    if (jpeg_settings->thumbnail_size.width > 0 &&
            jpeg_settings->thumbnail_size.height > 0)
        m_bThumbnailNeeded = TRUE;
    else
        m_bThumbnailNeeded = FALSE;
    encode_parm.encode_thumbnail = m_bThumbnailNeeded;

    // get color format
    cam_format_t img_fmt = CAM_FORMAT_YUV_420_NV12;  //default value
    main_stream->getFormat(img_fmt);
    encode_parm.color_format = getColorfmtFromImgFmt(img_fmt);

    // get jpeg quality
    encode_parm.quality = jpeg_settings->jpeg_quality;
    if (encode_parm.quality <= 0) {
        encode_parm.quality = 85;
    }

    // get jpeg thumbnail quality
    encode_parm.thumb_quality = jpeg_settings->jpeg_thumb_quality;

    cam_frame_len_offset_t main_offset;
    memset(&main_offset, 0, sizeof(cam_frame_len_offset_t));
    main_stream->getFrameOffset(main_offset);

    // src buf config
    //Pass input main image buffer info to encoder.
    QCamera3Memory *pStreamMem = main_stream->getStreamBufs();
    if (pStreamMem == NULL) {
        ALOGE("%s: cannot get stream bufs from main stream", __func__);
        ret = BAD_VALUE;
        goto on_error;
    }
    encode_parm.num_src_bufs = MIN(pStreamMem->getCnt(), MM_JPEG_MAX_BUF);
    for (uint32_t i = 0; i < encode_parm.num_src_bufs; i++) {
        if (pStreamMem != NULL) {
            encode_parm.src_main_buf[i].index = i;
            bufSize = pStreamMem->getSize(i);
            if (BAD_INDEX == bufSize) {
            ALOGE("%s: cannot retrieve buffer size for buffer %u", __func__, i);
                ret = BAD_VALUE;
                goto on_error;
            }
            encode_parm.src_main_buf[i].buf_size = (size_t)bufSize;
            encode_parm.src_main_buf[i].buf_vaddr = (uint8_t *)pStreamMem->getPtr(i);
            encode_parm.src_main_buf[i].fd = pStreamMem->getFd(i);
            encode_parm.src_main_buf[i].format = MM_JPEG_FMT_YUV;
            encode_parm.src_main_buf[i].offset = main_offset;
        }
    }

    //Pass input thumbnail buffer info to encoder.
    //Note: Use main buffer to encode thumbnail
    if (m_bThumbnailNeeded == TRUE) {
        pStreamMem = main_stream->getStreamBufs();
        if (pStreamMem == NULL) {
            ALOGE("%s: cannot get stream bufs from thumb stream", __func__);
            ret = BAD_VALUE;
            goto on_error;
        }
        cam_frame_len_offset_t thumb_offset;
        memset(&thumb_offset, 0, sizeof(cam_frame_len_offset_t));
        main_stream->getFrameOffset(thumb_offset);
        encode_parm.num_tmb_bufs = MIN(pStreamMem->getCnt(), MM_JPEG_MAX_BUF);
        for (uint32_t i = 0; i < encode_parm.num_tmb_bufs; i++) {
            if (pStreamMem != NULL) {
                encode_parm.src_thumb_buf[i].index = i;
                bufSize = pStreamMem->getSize(i);
                if (BAD_INDEX == bufSize) {
                    ALOGE("%s: cannot retrieve buffer size for buffer %u", __func__, i);
                    ret = BAD_VALUE;
                    goto on_error;
                }
                encode_parm.src_thumb_buf[i].buf_size = (uint32_t)bufSize;
                encode_parm.src_thumb_buf[i].buf_vaddr = (uint8_t *)pStreamMem->getPtr(i);
                encode_parm.src_thumb_buf[i].fd = pStreamMem->getFd(i);
                encode_parm.src_thumb_buf[i].format = MM_JPEG_FMT_YUV;
                encode_parm.src_thumb_buf[i].offset = thumb_offset;
            }
        }
    }

    //Pass output jpeg buffer info to encoder.
    //mJpegMem is allocated by framework.
    bufSize = mJpegMem->getSize(jpeg_settings->out_buf_index);
    if (BAD_INDEX == bufSize) {
        ALOGE("%s: cannot retrieve buffer size for buffer %u", __func__,
                jpeg_settings->out_buf_index);
        ret = BAD_VALUE;
        goto on_error;
    }
    encode_parm.num_dst_bufs = 1;
    encode_parm.dest_buf[0].index = 0;
    encode_parm.dest_buf[0].buf_size = (size_t)bufSize;
    encode_parm.dest_buf[0].buf_vaddr = (uint8_t *)mJpegMem->getPtr(
            jpeg_settings->out_buf_index);
    encode_parm.dest_buf[0].fd = mJpegMem->getFd(
            jpeg_settings->out_buf_index);
    encode_parm.dest_buf[0].format = MM_JPEG_FMT_YUV;
    encode_parm.dest_buf[0].offset = main_offset;

    CDBG("%s : X", __func__);
    return NO_ERROR;

on_error:
    CDBG("%s : X with error %d", __func__, ret);
    return ret;
}

/*===========================================================================
 * FUNCTION   : processData
 *
 * DESCRIPTION: enqueue data into dataProc thread
 *
 * PARAMETERS :
 *   @frame   : process frame
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *
 * NOTE       : depends on if offline reprocess is needed, received frame will
 *              be sent to either input queue of postprocess or jpeg encoding
 *==========================================================================*/
int32_t QCamera3PostProcessor::processData(mm_camera_super_buf_t *frame)
{
    QCamera3HardwareInterface* hal_obj = (QCamera3HardwareInterface*)m_parent->mUserData;
    pthread_mutex_lock(&mReprocJobLock);
    // enqueue to post proc input queue
    m_inputPPQ.enqueue((void *)frame);
    if (!(m_inputMetaQ.isEmpty())) {
       CDBG("%s: meta queue is not empty, do next job", __func__);
       m_dataProcTh.sendCmd(CAMERA_CMD_TYPE_DO_NEXT_JOB, FALSE, FALSE);
    }
    pthread_mutex_unlock(&mReprocJobLock);

    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : processData
 *
 * DESCRIPTION: enqueue data into dataProc thread
 *
 * PARAMETERS :
 *   @frame   : process frame
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *
 * NOTE       : depends on if offline reprocess is needed, received frame will
 *              be sent to either input queue of postprocess or jpeg encoding
 *==========================================================================*/
int32_t QCamera3PostProcessor::processData(qcamera_fwk_input_pp_data_t *frame)
{
    QCamera3HardwareInterface* hal_obj = (QCamera3HardwareInterface*)m_parent->mUserData;
    if (hal_obj->needReprocess(mPostProcMask)) {
        pthread_mutex_lock(&mReprocJobLock);
        // enqueu to post proc input queue
        m_inputFWKPPQ.enqueue((void *)frame);
        m_dataProcTh.sendCmd(CAMERA_CMD_TYPE_DO_NEXT_JOB, FALSE, FALSE);
        pthread_mutex_unlock(&mReprocJobLock);
    } else {
        jpeg_settings_t *jpeg_settings = (jpeg_settings_t *)m_jpegSettingsQ.dequeue();

        if (jpeg_settings == NULL) {
            ALOGE("%s: Cannot find jpeg settings", __func__);
            return BAD_VALUE;
        }

        CDBG_HIGH("%s: no need offline reprocess, sending to jpeg encoding", __func__);
        qcamera_hal3_jpeg_data_t *jpeg_job =
            (qcamera_hal3_jpeg_data_t *)malloc(sizeof(qcamera_hal3_jpeg_data_t));
        if (jpeg_job == NULL) {
            ALOGE("%s: No memory for jpeg job", __func__);
            return NO_MEMORY;
        }

        memset(jpeg_job, 0, sizeof(qcamera_hal3_jpeg_data_t));
        jpeg_job->fwk_frame = frame;
        jpeg_job->jpeg_settings = jpeg_settings;
        jpeg_job->metadata =
                (metadata_buffer_t *) frame->metadata_buffer.buffer;

        // enqueu to jpeg input queue
        m_inputJpegQ.enqueue((void *)jpeg_job);
        m_dataProcTh.sendCmd(CAMERA_CMD_TYPE_DO_NEXT_JOB, FALSE, FALSE);
    }

    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : processPPMetadata
 *
 * DESCRIPTION: enqueue data into dataProc thread
 *
 * PARAMETERS :
 *   @frame   : process metadata frame received from pic channel
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *
 *==========================================================================*/
int32_t QCamera3PostProcessor::processPPMetadata(mm_camera_super_buf_t *reproc_meta)
{
   pthread_mutex_lock(&mReprocJobLock);
    // enqueue to metadata input queue
    m_inputMetaQ.enqueue((void *)reproc_meta);
    if (!(m_inputPPQ.isEmpty())) {
       CDBG("%s: pp queue is not empty, do next job", __func__);
       m_dataProcTh.sendCmd(CAMERA_CMD_TYPE_DO_NEXT_JOB, FALSE, FALSE);
    } else {
       CDBG("%s: pp queue is empty, not calling do next job", __func__);
    }
    pthread_mutex_unlock(&mReprocJobLock);
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : processJpegSettingData
 *
 * DESCRIPTION: enqueue jpegSetting into dataProc thread
 *
 * PARAMETERS :
 *   @jpeg_settings : jpeg settings data received from pic channel
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *
 *==========================================================================*/
int32_t QCamera3PostProcessor::processJpegSettingData(
        jpeg_settings_t *jpeg_settings)
{
    if (!jpeg_settings) {
        ALOGE("%s: invalid jpeg settings pointer", __func__);
        return -EINVAL;
    }
    return m_jpegSettingsQ.enqueue((void *)jpeg_settings);
}

/*===========================================================================
 * FUNCTION   : processRawData
 *
 * DESCRIPTION: enqueue raw data into dataProc thread
 *
 * PARAMETERS :
 *   @frame   : process frame received from mm-camera-interface
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3PostProcessor::processRawData(mm_camera_super_buf_t *frame)
{
    // enqueu to raw input queue
    m_inputRawQ.enqueue((void *)frame);
    m_dataProcTh.sendCmd(CAMERA_CMD_TYPE_DO_NEXT_JOB, FALSE, FALSE);
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : processPPData
 *
 * DESCRIPTION: process received frame after reprocess.
 *
 * PARAMETERS :
 *   @frame   : received frame from reprocess channel.
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *
 * NOTE       : The frame after reprocess need to send to jpeg encoding.
 *==========================================================================*/
int32_t QCamera3PostProcessor::processPPData(mm_camera_super_buf_t *frame)
{
    qcamera_hal3_pp_data_t *job = (qcamera_hal3_pp_data_t *)m_ongoingPPQ.dequeue();

    if (job == NULL || ((NULL == job->src_frame) && (NULL == job->fwk_src_frame))) {
        ALOGE("%s: Cannot find reprocess job", __func__);
        return BAD_VALUE;
    }
    if (job->jpeg_settings == NULL) {
        ALOGE("%s: Cannot find jpeg settings", __func__);
        return BAD_VALUE;
    }

    qcamera_hal3_jpeg_data_t *jpeg_job =
        (qcamera_hal3_jpeg_data_t *)malloc(sizeof(qcamera_hal3_jpeg_data_t));
    if (jpeg_job == NULL) {
        ALOGE("%s: No memory for jpeg job", __func__);
        return NO_MEMORY;
    }

    memset(jpeg_job, 0, sizeof(qcamera_hal3_jpeg_data_t));
    jpeg_job->src_frame = frame;
    if(frame != job->src_frame)
        jpeg_job->src_reproc_frame = job->src_frame;
    if (NULL == job->fwk_src_frame) {
        jpeg_job->metadata = job->metadata;
    } else {
        jpeg_job->metadata =
                (metadata_buffer_t *) job->fwk_src_frame->metadata_buffer.buffer;
        jpeg_job->fwk_src_buffer = job->fwk_src_frame;
    }
    jpeg_job->src_metadata = job->src_metadata;
    jpeg_job->jpeg_settings = job->jpeg_settings;

    // free pp job buf
    free(job);

    // enqueu reprocessed frame to jpeg input queue
    m_inputJpegQ.enqueue((void *)jpeg_job);

    // wait up data proc thread
    m_dataProcTh.sendCmd(CAMERA_CMD_TYPE_DO_NEXT_JOB, FALSE, FALSE);

    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : findJpegJobByJobId
 *
 * DESCRIPTION: find a jpeg job from ongoing Jpeg queue by its job ID
 *
 * PARAMETERS :
 *   @jobId   : job Id of the job
 *
 * RETURN     : ptr to a jpeg job struct. NULL if not found.
 *
 * NOTE       : Currently only one job is sending to mm-jpeg-interface for jpeg
 *              encoding. Therefore simply dequeue from the ongoing Jpeg Queue
 *              will serve the purpose to find the jpeg job.
 *==========================================================================*/
qcamera_hal3_jpeg_data_t *QCamera3PostProcessor::findJpegJobByJobId(uint32_t jobId)
{
    qcamera_hal3_jpeg_data_t * job = NULL;
    if (jobId == 0) {
        ALOGE("%s: not a valid jpeg jobId", __func__);
        return NULL;
    }

    // currely only one jpeg job ongoing, so simply dequeue the head
    job = (qcamera_hal3_jpeg_data_t *)m_ongoingJpegQ.dequeue();
    return job;
}

/*===========================================================================
 * FUNCTION   : releasePPInputData
 *
 * DESCRIPTION: callback function to release post process input data node
 *
 * PARAMETERS :
 *   @data      : ptr to post process input data
 *   @user_data : user data ptr (QCamera3Reprocessor)
 *
 * RETURN     : None
 *==========================================================================*/
void QCamera3PostProcessor::releasePPInputData(void *data, void *user_data)
{
    QCamera3PostProcessor *pme = (QCamera3PostProcessor *)user_data;
    if (NULL != pme) {
        pme->releaseSuperBuf((mm_camera_super_buf_t *)data);
    }
}

/*===========================================================================
 * FUNCTION   : releaseMetaData
 *
 * DESCRIPTION: callback function to release metadata camera buffer
 *
 * PARAMETERS :
 *   @data      : ptr to post process input data
 *   @user_data : user data ptr (QCamera3Reprocessor)
 *
 * RETURN     : None
 *==========================================================================*/
void QCamera3PostProcessor::releaseMetadata(void *data, void *user_data)
{
    QCamera3PostProcessor *pme = (QCamera3PostProcessor *)user_data;
    if (NULL != pme) {
        pme->m_parent->metadataBufDone((mm_camera_super_buf_t *)data);
    }
}

/*===========================================================================
 * FUNCTION   : releaseJpegData
 *
 * DESCRIPTION: callback function to release jpeg job node
 *
 * PARAMETERS :
 *   @data      : ptr to ongoing jpeg job data
 *   @user_data : user data ptr (QCamera3Reprocessor)
 *
 * RETURN     : None
 *==========================================================================*/
void QCamera3PostProcessor::releaseJpegData(void *data, void *user_data)
{
    QCamera3PostProcessor *pme = (QCamera3PostProcessor *)user_data;
    if (NULL != pme) {
        pme->releaseJpegJobData((qcamera_hal3_jpeg_data_t *)data);
    }
}

/*===========================================================================
 * FUNCTION   : releaseOngoingPPData
 *
 * DESCRIPTION: callback function to release ongoing postprocess job node
 *
 * PARAMETERS :
 *   @data      : ptr to onging postprocess job
 *   @user_data : user data ptr (QCamera3Reprocessor)
 *
 * RETURN     : None
 *==========================================================================*/
void QCamera3PostProcessor::releaseOngoingPPData(void *data, void *user_data)
{
    QCamera3PostProcessor *pme = (QCamera3PostProcessor *)user_data;
    if (NULL != pme) {
        qcamera_hal3_pp_data_t *pp_job = (qcamera_hal3_pp_data_t *)data;
        if (NULL != pp_job->src_frame) {
            pme->releaseSuperBuf(pp_job->src_frame);
            free(pp_job->src_frame);
            if (NULL != pp_job->src_metadata) {
                pme->m_parent->metadataBufDone(pp_job->src_metadata);
                free(pp_job->src_metadata);
            }
            pp_job->src_frame = NULL;
            pp_job->metadata = NULL;
        }

        if (NULL != pp_job->fwk_src_frame) {
            free(pp_job->fwk_src_frame);
            pp_job->fwk_src_frame = NULL;
        }
    }
}

/*===========================================================================
 * FUNCTION   : releaseSuperBuf
 *
 * DESCRIPTION: function to release a superbuf frame by returning back to kernel
 *
 * PARAMETERS :
 *   @super_buf : ptr to the superbuf frame
 *
 * RETURN     : None
 *==========================================================================*/
void QCamera3PostProcessor::releaseSuperBuf(mm_camera_super_buf_t *super_buf)
{
    if (NULL != super_buf) {
        if (m_parent != NULL) {
            m_parent->bufDone(super_buf);
        }
    }
}

/*===========================================================================
 * FUNCTION   : releaseOfflineBuffers
 *
 * DESCRIPTION: function to release/unmap offline buffers if any
 *
 * PARAMETERS : None
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3PostProcessor::releaseOfflineBuffers()
{
    int32_t rc = NO_ERROR;

    if(NULL != m_pReprocChannel) {
        rc = m_pReprocChannel->unmapOfflineBuffers(false);
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : releaseJpegJobData
 *
 * DESCRIPTION: function to release internal resources in jpeg job struct
 *
 * PARAMETERS :
 *   @job     : ptr to jpeg job struct
 *
 * RETURN     : None
 *
 * NOTE       : original source frame need to be queued back to kernel for
 *              future use. Output buf of jpeg job need to be released since
 *              it's allocated for each job. Exif object need to be deleted.
 *==========================================================================*/
void QCamera3PostProcessor::releaseJpegJobData(qcamera_hal3_jpeg_data_t *job)
{
    ATRACE_CALL();
    int32_t rc = NO_ERROR;
    CDBG("%s: E", __func__);
    if (NULL != job) {
        if (NULL != job->src_reproc_frame) {
            free(job->src_reproc_frame);
            job->src_reproc_frame = NULL;
        }

        if (NULL != job->src_frame) {
            if (NULL != m_pReprocChannel) {
                rc = m_pReprocChannel->bufDone(job->src_frame);
                if (NO_ERROR != rc)
                    ALOGE("%s: bufDone error: %d", __func__, rc);
            }
            free(job->src_frame);
            job->src_frame = NULL;
        }

        if (NULL != job->fwk_src_buffer) {
            free(job->fwk_src_buffer);
            job->fwk_src_buffer = NULL;
        } else if (NULL != job->src_metadata) {
            m_parent->metadataBufDone(job->src_metadata);
            free(job->src_metadata);
            job->src_metadata = NULL;
        }

        if (NULL != job->fwk_frame) {
            free(job->fwk_frame);
            job->fwk_frame = NULL;
        }

        if (NULL != job->pJpegExifObj) {
            delete job->pJpegExifObj;
            job->pJpegExifObj = NULL;
        }

        if (NULL != job->jpeg_settings) {
            free(job->jpeg_settings);
            job->jpeg_settings = NULL;
        }
    }
    /* Additional trigger to process any pending jobs in the input queue */
    m_dataProcTh.sendCmd(CAMERA_CMD_TYPE_DO_NEXT_JOB, FALSE, FALSE);
    CDBG("%s: X", __func__);
}

/*===========================================================================
 * FUNCTION   : getColorfmtFromImgFmt
 *
 * DESCRIPTION: function to return jpeg color format based on its image format
 *
 * PARAMETERS :
 *   @img_fmt : image format
 *
 * RETURN     : jpeg color format that can be understandable by omx lib
 *==========================================================================*/
mm_jpeg_color_format QCamera3PostProcessor::getColorfmtFromImgFmt(cam_format_t img_fmt)
{
    switch (img_fmt) {
    case CAM_FORMAT_YUV_420_NV21:
        return MM_JPEG_COLOR_FORMAT_YCRCBLP_H2V2;
    case CAM_FORMAT_YUV_420_NV21_ADRENO:
        return MM_JPEG_COLOR_FORMAT_YCRCBLP_H2V2;
    case CAM_FORMAT_YUV_420_NV12:
    case CAM_FORMAT_YUV_420_NV12_VENUS:
        return MM_JPEG_COLOR_FORMAT_YCBCRLP_H2V2;
    case CAM_FORMAT_YUV_420_YV12:
        return MM_JPEG_COLOR_FORMAT_YCBCRLP_H2V2;
    case CAM_FORMAT_YUV_422_NV61:
        return MM_JPEG_COLOR_FORMAT_YCRCBLP_H2V1;
    case CAM_FORMAT_YUV_422_NV16:
        return MM_JPEG_COLOR_FORMAT_YCBCRLP_H2V1;
    default:
        return MM_JPEG_COLOR_FORMAT_YCRCBLP_H2V2;
    }
}

/*===========================================================================
 * FUNCTION   : getJpegImgTypeFromImgFmt
 *
 * DESCRIPTION: function to return jpeg encode image type based on its image format
 *
 * PARAMETERS :
 *   @img_fmt : image format
 *
 * RETURN     : return jpeg source image format (YUV or Bitstream)
 *==========================================================================*/
mm_jpeg_format_t QCamera3PostProcessor::getJpegImgTypeFromImgFmt(cam_format_t img_fmt)
{
    switch (img_fmt) {
    case CAM_FORMAT_YUV_420_NV21:
    case CAM_FORMAT_YUV_420_NV21_ADRENO:
    case CAM_FORMAT_YUV_420_NV12:
    case CAM_FORMAT_YUV_420_NV12_VENUS:
    case CAM_FORMAT_YUV_420_YV12:
    case CAM_FORMAT_YUV_422_NV61:
    case CAM_FORMAT_YUV_422_NV16:
        return MM_JPEG_FMT_YUV;
    default:
        return MM_JPEG_FMT_YUV;
    }
}

/*===========================================================================
 * FUNCTION   : encodeFWKData
 *
 * DESCRIPTION: function to prepare encoding job information and send to
 *              mm-jpeg-interface to do the encoding job
 *
 * PARAMETERS :
 *   @jpeg_job_data : ptr to a struct saving job related information
 *   @needNewSess   : flag to indicate if a new jpeg encoding session need
 *                    to be created. After creation, this flag will be toggled
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3PostProcessor::encodeFWKData(qcamera_hal3_jpeg_data_t *jpeg_job_data,
        uint8_t &needNewSess)
{
    CDBG("%s : E", __func__);
    int32_t ret = NO_ERROR;
    mm_jpeg_job_t jpg_job;
    uint32_t jobId = 0;
    qcamera_fwk_input_pp_data_t *recvd_frame = NULL;
    metadata_buffer_t *metadata = NULL;
    jpeg_settings_t *jpeg_settings = NULL;
    QCamera3HardwareInterface* hal_obj = NULL;
    bool needJpegRotation = false;

    if (NULL == jpeg_job_data) {
        ALOGE("%s: Invalid jpeg job", __func__);
        return BAD_VALUE;
    }

    recvd_frame = jpeg_job_data->fwk_frame;
    if (NULL == recvd_frame) {
        ALOGE("%s: Invalid input buffer", __func__);
        return BAD_VALUE;
    }

    metadata = jpeg_job_data->metadata;
    if (NULL == metadata) {
        ALOGE("%s: Invalid metadata buffer", __func__);
        return BAD_VALUE;
    }

    jpeg_settings = jpeg_job_data->jpeg_settings;
    if (NULL == jpeg_settings) {
        ALOGE("%s: Invalid jpeg settings buffer", __func__);
        return BAD_VALUE;
    }

    if ((NULL != jpeg_job_data->src_frame) && (NULL != jpeg_job_data->src_frame)) {
        ALOGE("%s: Unsupported case both framework and camera source buffers are invalid!",
                __func__);
        return BAD_VALUE;
    }

    hal_obj = (QCamera3HardwareInterface*)m_parent->mUserData;

    if (mJpegClientHandle <= 0) {
        ALOGE("%s: Error: bug here, mJpegClientHandle is 0", __func__);
        return UNKNOWN_ERROR;
    }

    cam_dimension_t src_dim;
    memset(&src_dim, 0, sizeof(cam_dimension_t));
    src_dim.width = recvd_frame->reproc_config.input_stream_dim.width;
    src_dim.height = recvd_frame->reproc_config.input_stream_dim.height;

    cam_dimension_t dst_dim;
    memset(&dst_dim, 0, sizeof(cam_dimension_t));
    dst_dim.width = recvd_frame->reproc_config.output_stream_dim.width;
    dst_dim.height = recvd_frame->reproc_config.output_stream_dim.height;

    CDBG_HIGH("%s: Need new session?:%d",__func__, needNewSess);
    if (needNewSess) {
        //creating a new session, so we must destroy the old one
        if ( 0 < mJpegSessionId ) {
            ret = mJpegHandle.destroy_session(mJpegSessionId);
            if (ret != NO_ERROR) {
                ALOGE("%s: Error destroying an old jpeg encoding session, id = %d",
                      __func__, mJpegSessionId);
                return ret;
            }
            mJpegSessionId = 0;
        }
        // create jpeg encoding session
        mm_jpeg_encode_params_t encodeParam;
        memset(&encodeParam, 0, sizeof(mm_jpeg_encode_params_t));
        encodeParam.main_dim.src_dim = src_dim;
        encodeParam.main_dim.dst_dim = dst_dim;
        encodeParam.thumb_dim.src_dim = src_dim;
        encodeParam.thumb_dim.dst_dim = jpeg_settings->thumbnail_size;

        getFWKJpegEncodeConfig(encodeParam, recvd_frame, jpeg_settings);
        CDBG_HIGH("%s: #src bufs:%d # tmb bufs:%d #dst_bufs:%d", __func__,
                     encodeParam.num_src_bufs,encodeParam.num_tmb_bufs,encodeParam.num_dst_bufs);

        ret = mJpegHandle.create_session(mJpegClientHandle, &encodeParam, &mJpegSessionId);
        if (ret != NO_ERROR) {
            ALOGE("%s: Error creating a new jpeg encoding session, ret = %d", __func__, ret);
            return ret;
        }
        needNewSess = FALSE;
    }

    // Fill in new job
    memset(&jpg_job, 0, sizeof(mm_jpeg_job_t));
    jpg_job.job_type = JPEG_JOB_TYPE_ENCODE;
    jpg_job.encode_job.session_id = mJpegSessionId;
    jpg_job.encode_job.src_index = 0;
    jpg_job.encode_job.dst_index = 0;

    cam_rect_t crop;
    memset(&crop, 0, sizeof(cam_rect_t));
    //TBD_later - Zoom event removed in stream
    //main_stream->getCropInfo(crop);

    // Set main dim job parameters and handle rotation
    needJpegRotation = hal_obj->needJpegRotation();
    if (!needJpegRotation && (jpeg_settings->jpeg_orientation == 90 ||
            jpeg_settings->jpeg_orientation == 270)) {

        jpg_job.encode_job.main_dim.src_dim.width = src_dim.height;
        jpg_job.encode_job.main_dim.src_dim.height = src_dim.width;

        jpg_job.encode_job.main_dim.dst_dim.width = dst_dim.height;
        jpg_job.encode_job.main_dim.dst_dim.height = dst_dim.width;

        jpg_job.encode_job.main_dim.crop.width = crop.height;
        jpg_job.encode_job.main_dim.crop.height = crop.width;
        jpg_job.encode_job.main_dim.crop.left = crop.top;
        jpg_job.encode_job.main_dim.crop.top = crop.left;
    } else {
        jpg_job.encode_job.main_dim.src_dim = src_dim;
        jpg_job.encode_job.main_dim.dst_dim = dst_dim;
        jpg_job.encode_job.main_dim.crop = crop;
    }

    // get exif data
    QCamera3Exif *pJpegExifObj = m_parent->getExifData(metadata, jpeg_settings);
    jpeg_job_data->pJpegExifObj = pJpegExifObj;
    if (pJpegExifObj != NULL) {
        jpg_job.encode_job.exif_info.exif_data = pJpegExifObj->getEntries();
        jpg_job.encode_job.exif_info.numOfEntries =
            pJpegExifObj->getNumOfEntries();
    }

    // thumbnail dim
    CDBG_HIGH("%s: Thumbnail needed:%d",__func__, m_bThumbnailNeeded);
    if (m_bThumbnailNeeded == TRUE) {
        memset(&crop, 0, sizeof(cam_rect_t));
        jpg_job.encode_job.thumb_dim.dst_dim =
                jpeg_settings->thumbnail_size;

        if (needJpegRotation) {
            jpg_job.encode_job.rotation = (uint32_t)jpeg_settings->jpeg_orientation;
            CDBG_HIGH("%s: jpeg rotation is set to %u", __func__, jpg_job.encode_job.rotation);
        } else if (jpeg_settings->jpeg_orientation  == 90 ||
                jpeg_settings->jpeg_orientation == 270) {
            //swap the thumbnail destination width and height if it has
            //already been rotated
            int temp = jpg_job.encode_job.thumb_dim.dst_dim.width;
            jpg_job.encode_job.thumb_dim.dst_dim.width =
                    jpg_job.encode_job.thumb_dim.dst_dim.height;
            jpg_job.encode_job.thumb_dim.dst_dim.height = temp;
        }
        jpg_job.encode_job.thumb_dim.src_dim = src_dim;
        jpg_job.encode_job.thumb_dim.crop = crop;
        jpg_job.encode_job.thumb_index = 0;
    }

    if (metadata != NULL) {
       //Fill in the metadata passed as parameter
       jpg_job.encode_job.p_metadata = metadata;
    } else {
       ALOGE("%s: Metadata is null", __func__);
    }

    jpg_job.encode_job.hal_version = CAM_HAL_V3;

    //Start jpeg encoding
    ret = mJpegHandle.start_job(&jpg_job, &jobId);
    if (ret == NO_ERROR) {
        // remember job info
        jpeg_job_data->jobId = jobId;
    }

    CDBG("%s : X", __func__);
    return ret;
}

/*===========================================================================
 * FUNCTION   : encodeData
 *
 * DESCRIPTION: function to prepare encoding job information and send to
 *              mm-jpeg-interface to do the encoding job
 *
 * PARAMETERS :
 *   @jpeg_job_data : ptr to a struct saving job related information
 *   @needNewSess   : flag to indicate if a new jpeg encoding session need
 *                    to be created. After creation, this flag will be toggled
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3PostProcessor::encodeData(qcamera_hal3_jpeg_data_t *jpeg_job_data,
                          uint8_t &needNewSess)
{
    ATRACE_CALL();
    CDBG("%s : E", __func__);
    int32_t ret = NO_ERROR;
    mm_jpeg_job_t jpg_job;
    uint32_t jobId = 0;
    QCamera3Stream *main_stream = NULL;
    mm_camera_buf_def_t *main_frame = NULL;
    QCamera3Channel *srcChannel = NULL;
    mm_camera_super_buf_t *recvd_frame = NULL;
    metadata_buffer_t *metadata = NULL;
    jpeg_settings_t *jpeg_settings = NULL;
    QCamera3HardwareInterface* hal_obj = NULL;
    mm_jpeg_debug_exif_params_t *exif_debug_params = NULL;
    if (m_parent != NULL) {
       hal_obj = (QCamera3HardwareInterface*)m_parent->mUserData;
    } else {
       ALOGE("%s: m_parent is NULL, Error",__func__);
       return BAD_VALUE;
    }
    bool needJpegRotation = false;

    recvd_frame = jpeg_job_data->src_frame;
    metadata = jpeg_job_data->metadata;
    jpeg_settings = jpeg_job_data->jpeg_settings;

    CDBG("%s: encoding bufIndex: %u", __func__,
        jpeg_job_data->src_frame->bufs[0]->buf_idx);

    QCamera3Channel *pChannel = NULL;
    // first check picture channel
    if (m_parent->getMyHandle() == recvd_frame->ch_id) {
        pChannel = m_parent;
    }
    // check reprocess channel if not found
    if (pChannel == NULL) {
        if (m_pReprocChannel != NULL &&
            m_pReprocChannel->getMyHandle() == recvd_frame->ch_id) {
            pChannel = m_pReprocChannel;
        }
    }

    srcChannel = pChannel;

    if (srcChannel == NULL) {
        ALOGE("%s: No corresponding channel (ch_id = %d) exist, return here",
              __func__, recvd_frame->ch_id);
        return BAD_VALUE;
    }

    // find snapshot frame and thumnail frame
    //Note: In this version we will receive only snapshot frame.
    for (uint32_t i = 0; i < recvd_frame->num_bufs; i++) {
        QCamera3Stream *srcStream =
            srcChannel->getStreamByHandle(recvd_frame->bufs[i]->stream_id);
        if (srcStream != NULL) {
            switch (srcStream->getMyType()) {
            case CAM_STREAM_TYPE_SNAPSHOT:
            case CAM_STREAM_TYPE_OFFLINE_PROC:
                main_stream = srcStream;
                main_frame = recvd_frame->bufs[i];
                break;
            default:
                break;
            }
        }
    }

    if(NULL == main_frame){
       ALOGE("%s : Main frame is NULL", __func__);
       return BAD_VALUE;
    }

    QCamera3Memory *memObj = (QCamera3Memory *)main_frame->mem_info;
    if (NULL == memObj) {
        ALOGE("%s : Memeory Obj of main frame is NULL", __func__);
        return NO_MEMORY;
    }

    // clean and invalidate cache ops through mem obj of the frame
    memObj->cleanInvalidateCache(main_frame->buf_idx);

    if (mJpegClientHandle <= 0) {
        ALOGE("%s: Error: bug here, mJpegClientHandle is 0", __func__);
        return UNKNOWN_ERROR;
    }
    cam_dimension_t src_dim;
    memset(&src_dim, 0, sizeof(cam_dimension_t));
    main_stream->getFrameDimension(src_dim);

    cam_dimension_t dst_dim;
    memset(&dst_dim, 0, sizeof(cam_dimension_t));
    if (srcChannel->getStreamByIndex(0)) {
       srcChannel->getStreamByIndex(0)->getFrameDimension(dst_dim);
    }

    needJpegRotation = hal_obj->needJpegRotation();
    CDBG_HIGH("%s: Need new session?:%d",__func__, needNewSess);
    if (needNewSess) {
        //creating a new session, so we must destroy the old one
        if ( 0 < mJpegSessionId ) {
            ret = mJpegHandle.destroy_session(mJpegSessionId);
            if (ret != NO_ERROR) {
                ALOGE("%s: Error destroying an old jpeg encoding session, id = %d",
                      __func__, mJpegSessionId);
                return ret;
            }
            mJpegSessionId = 0;
        }
        // create jpeg encoding session
        mm_jpeg_encode_params_t encodeParam;
        memset(&encodeParam, 0, sizeof(mm_jpeg_encode_params_t));
        getJpegEncodeConfig(encodeParam, main_stream, jpeg_settings);
        CDBG_HIGH("%s: #src bufs:%d # tmb bufs:%d #dst_bufs:%d", __func__,
                     encodeParam.num_src_bufs,encodeParam.num_tmb_bufs,encodeParam.num_dst_bufs);
        if (!needJpegRotation &&
            (jpeg_settings->jpeg_orientation == 90 ||
            jpeg_settings->jpeg_orientation == 270)) {
           //swap src width and height, stride and scanline due to rotation
           encodeParam.main_dim.src_dim.width = src_dim.height;
           encodeParam.main_dim.src_dim.height = src_dim.width;
           encodeParam.thumb_dim.src_dim.width = src_dim.height;
           encodeParam.thumb_dim.src_dim.height = src_dim.width;

           int32_t temp = encodeParam.src_main_buf[0].offset.mp[0].stride;
           encodeParam.src_main_buf[0].offset.mp[0].stride =
              encodeParam.src_main_buf[0].offset.mp[0].scanline;
           encodeParam.src_main_buf[0].offset.mp[0].scanline = temp;

           temp = encodeParam.src_thumb_buf[0].offset.mp[0].stride;
           encodeParam.src_thumb_buf[0].offset.mp[0].stride =
              encodeParam.src_thumb_buf[0].offset.mp[0].scanline;
           encodeParam.src_thumb_buf[0].offset.mp[0].scanline = temp;
        } else {
           encodeParam.main_dim.src_dim  = src_dim;
           encodeParam.thumb_dim.src_dim = src_dim;
        }
        encodeParam.main_dim.dst_dim = dst_dim;
        encodeParam.thumb_dim.dst_dim = jpeg_settings->thumbnail_size;
        if (needJpegRotation) {
           encodeParam.rotation = (uint32_t)jpeg_settings->jpeg_orientation;
        }

        ret = mJpegHandle.create_session(mJpegClientHandle, &encodeParam, &mJpegSessionId);
        if (ret != NO_ERROR) {
            ALOGE("%s: Error creating a new jpeg encoding session, ret = %d", __func__, ret);
            return ret;
        }
        needNewSess = FALSE;
    }

    // Fill in new job
    memset(&jpg_job, 0, sizeof(mm_jpeg_job_t));
    jpg_job.job_type = JPEG_JOB_TYPE_ENCODE;
    jpg_job.encode_job.session_id = mJpegSessionId;
    jpg_job.encode_job.src_index = (int32_t)main_frame->buf_idx;
    jpg_job.encode_job.dst_index = 0;

    if (needJpegRotation) {
        jpg_job.encode_job.rotation = (uint32_t)jpeg_settings->jpeg_orientation;
        CDBG("%s: %d: jpeg rotation is set to %d", __func__, __LINE__,
                jpg_job.encode_job.rotation);
    }

    cam_rect_t crop;
    memset(&crop, 0, sizeof(cam_rect_t));
    //TBD_later - Zoom event removed in stream
    //main_stream->getCropInfo(crop);

    // Set main dim job parameters and handle rotation
    if (!needJpegRotation && (jpeg_settings->jpeg_orientation == 90 ||
            jpeg_settings->jpeg_orientation == 270)) {

        jpg_job.encode_job.main_dim.src_dim.width = src_dim.height;
        jpg_job.encode_job.main_dim.src_dim.height = src_dim.width;

        jpg_job.encode_job.main_dim.dst_dim.width = dst_dim.height;
        jpg_job.encode_job.main_dim.dst_dim.height = dst_dim.width;

        jpg_job.encode_job.main_dim.crop.width = crop.height;
        jpg_job.encode_job.main_dim.crop.height = crop.width;
        jpg_job.encode_job.main_dim.crop.left = crop.top;
        jpg_job.encode_job.main_dim.crop.top = crop.left;
    } else {
        jpg_job.encode_job.main_dim.src_dim = src_dim;
        jpg_job.encode_job.main_dim.dst_dim = dst_dim;
        jpg_job.encode_job.main_dim.crop = crop;
    }


    // get exif data
    QCamera3Exif *pJpegExifObj = m_parent->getExifData(metadata, jpeg_settings);
    jpeg_job_data->pJpegExifObj = pJpegExifObj;
    if (pJpegExifObj != NULL) {
        jpg_job.encode_job.exif_info.exif_data = pJpegExifObj->getEntries();
        jpg_job.encode_job.exif_info.numOfEntries =
            pJpegExifObj->getNumOfEntries();
    }

    // thumbnail dim
    CDBG_HIGH("%s: Thumbnail needed:%d",__func__, m_bThumbnailNeeded);
    if (m_bThumbnailNeeded == TRUE) {
        memset(&crop, 0, sizeof(cam_rect_t));
        jpg_job.encode_job.thumb_dim.dst_dim =
                jpeg_settings->thumbnail_size;

      if (!needJpegRotation &&
          (jpeg_settings->jpeg_orientation  == 90 ||
           jpeg_settings->jpeg_orientation == 270)) {
            //swap the thumbnail destination width and height if it has
            //already been rotated
            int temp = jpg_job.encode_job.thumb_dim.dst_dim.width;
            jpg_job.encode_job.thumb_dim.dst_dim.width =
                    jpg_job.encode_job.thumb_dim.dst_dim.height;
            jpg_job.encode_job.thumb_dim.dst_dim.height = temp;

            jpg_job.encode_job.thumb_dim.src_dim.width = src_dim.height;
            jpg_job.encode_job.thumb_dim.src_dim.height = src_dim.width;
        } else {
           jpg_job.encode_job.thumb_dim.src_dim = src_dim;
        }
        jpg_job.encode_job.thumb_dim.crop = crop;
        jpg_job.encode_job.thumb_index = main_frame->buf_idx;
    }

    jpg_job.encode_job.cam_exif_params = hal_obj->get3AExifParams();
    exif_debug_params = jpg_job.encode_job.cam_exif_params.debug_params;

    // Allocate for a local copy of debug parameters
    jpg_job.encode_job.cam_exif_params.debug_params =
            (mm_jpeg_debug_exif_params_t *) malloc (sizeof(mm_jpeg_debug_exif_params_t));
    if (!jpg_job.encode_job.cam_exif_params.debug_params) {
        ALOGE("Out of Memory. Allocation failed for 3A debug exif params");
        return NO_MEMORY;
    }

    jpg_job.encode_job.mobicat_mask = hal_obj->getMobicatMask();

    if (metadata != NULL) {
       //Fill in the metadata passed as parameter
       jpg_job.encode_job.p_metadata = metadata;

       jpg_job.encode_job.p_metadata->is_mobicat_aec_params_valid =
                jpg_job.encode_job.cam_exif_params.cam_3a_params_valid;

       if (jpg_job.encode_job.cam_exif_params.cam_3a_params_valid) {
            jpg_job.encode_job.p_metadata->mobicat_aec_params =
                jpg_job.encode_job.cam_exif_params.cam_3a_params;
       }

       if (exif_debug_params) {
            // Copy debug parameters locally.
           memcpy(jpg_job.encode_job.cam_exif_params.debug_params,
                   exif_debug_params, (sizeof(mm_jpeg_debug_exif_params_t)));
           /* Save a copy of 3A debug params */
            jpg_job.encode_job.p_metadata->is_statsdebug_ae_params_valid =
                    jpg_job.encode_job.cam_exif_params.debug_params->ae_debug_params_valid;
            jpg_job.encode_job.p_metadata->is_statsdebug_awb_params_valid =
                    jpg_job.encode_job.cam_exif_params.debug_params->awb_debug_params_valid;
            jpg_job.encode_job.p_metadata->is_statsdebug_af_params_valid =
                    jpg_job.encode_job.cam_exif_params.debug_params->af_debug_params_valid;
            jpg_job.encode_job.p_metadata->is_statsdebug_asd_params_valid =
                    jpg_job.encode_job.cam_exif_params.debug_params->asd_debug_params_valid;
            jpg_job.encode_job.p_metadata->is_statsdebug_stats_params_valid =
                    jpg_job.encode_job.cam_exif_params.debug_params->stats_debug_params_valid;

            if (jpg_job.encode_job.cam_exif_params.debug_params->ae_debug_params_valid) {
                jpg_job.encode_job.p_metadata->statsdebug_ae_data =
                        jpg_job.encode_job.cam_exif_params.debug_params->ae_debug_params;
            }
            if (jpg_job.encode_job.cam_exif_params.debug_params->awb_debug_params_valid) {
                jpg_job.encode_job.p_metadata->statsdebug_awb_data =
                        jpg_job.encode_job.cam_exif_params.debug_params->awb_debug_params;
            }
            if (jpg_job.encode_job.cam_exif_params.debug_params->af_debug_params_valid) {
                jpg_job.encode_job.p_metadata->statsdebug_af_data =
                        jpg_job.encode_job.cam_exif_params.debug_params->af_debug_params;
            }
            if (jpg_job.encode_job.cam_exif_params.debug_params->asd_debug_params_valid) {
                jpg_job.encode_job.p_metadata->statsdebug_asd_data =
                        jpg_job.encode_job.cam_exif_params.debug_params->asd_debug_params;
            }
            if (jpg_job.encode_job.cam_exif_params.debug_params->stats_debug_params_valid) {
                jpg_job.encode_job.p_metadata->statsdebug_stats_buffer_data =
                        jpg_job.encode_job.cam_exif_params.debug_params->stats_debug_params;
            }
        }
    } else {
       ALOGE("%s: Metadata is null", __func__);
    }

    jpg_job.encode_job.hal_version = CAM_HAL_V3;

    //Start jpeg encoding
    ret = mJpegHandle.start_job(&jpg_job, &jobId);
    if (jpg_job.encode_job.cam_exif_params.debug_params) {
        free(jpg_job.encode_job.cam_exif_params.debug_params);
    }
    if (ret == NO_ERROR) {
        // remember job info
        jpeg_job_data->jobId = jobId;
    }

    CDBG("%s : X", __func__);
    return ret;
}

/*===========================================================================
 * FUNCTION   : dataProcessRoutine
 *
 * DESCRIPTION: data process routine that handles input data either from input
 *              Jpeg Queue to do jpeg encoding, or from input PP Queue to do
 *              reprocess.
 *
 * PARAMETERS :
 *   @data    : user data ptr (QCamera3PostProcessor)
 *
 * RETURN     : None
 *==========================================================================*/
void *QCamera3PostProcessor::dataProcessRoutine(void *data)
{
    int running = 1;
    int ret;
    uint8_t is_active = FALSE;
    uint8_t needNewSess = TRUE;
    mm_camera_super_buf_t *meta_buffer = NULL;
    CDBG("%s: E", __func__);
    QCamera3PostProcessor *pme = (QCamera3PostProcessor *)data;
    QCameraCmdThread *cmdThread = &pme->m_dataProcTh;
    cmdThread->setName("cam_data_proc");

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
        case CAMERA_CMD_TYPE_START_DATA_PROC:
            CDBG_HIGH("%s: start data proc", __func__);
            is_active = TRUE;
            needNewSess = TRUE;

            pme->m_ongoingPPQ.init();
            pme->m_inputJpegQ.init();
            pme->m_inputPPQ.init();
            pme->m_inputFWKPPQ.init();
            pme->m_inputRawQ.init();
            pme->m_inputMetaQ.init();
            cam_sem_post(&cmdThread->sync_sem);

            break;
        case CAMERA_CMD_TYPE_STOP_DATA_PROC:
            {
                CDBG_HIGH("%s: stop data proc", __func__);
                is_active = FALSE;

                // cancel all ongoing jpeg jobs
                qcamera_hal3_jpeg_data_t *jpeg_job =
                    (qcamera_hal3_jpeg_data_t *)pme->m_ongoingJpegQ.dequeue();
                while (jpeg_job != NULL) {
                    pme->mJpegHandle.abort_job(jpeg_job->jobId);

                    pme->releaseJpegJobData(jpeg_job);
                    free(jpeg_job);

                    jpeg_job = (qcamera_hal3_jpeg_data_t *)pme->m_ongoingJpegQ.dequeue();
                }

                // destroy jpeg encoding session
                if ( 0 < pme->mJpegSessionId ) {
                    pme->mJpegHandle.destroy_session(pme->mJpegSessionId);
                    pme->mJpegSessionId = 0;
                }

                needNewSess = TRUE;

                // flush ongoing postproc Queue
                pme->m_ongoingPPQ.flush();

                // flush input jpeg Queue
                pme->m_inputJpegQ.flush();

                // flush input Postproc Queue
                pme->m_inputPPQ.flush();

                // flush framework input Postproc Queue
                pme->m_inputFWKPPQ.flush();

                // flush input raw Queue
                pme->m_inputRawQ.flush();

                pme->m_inputMetaQ.flush();

                // signal cmd is completed
                cam_sem_post(&cmdThread->sync_sem);
            }
            break;
        case CAMERA_CMD_TYPE_DO_NEXT_JOB:
            {
                CDBG_HIGH("%s: Do next job, active is %d", __func__, is_active);
                /* needNewSess is set to TRUE as postproc is not re-STARTed
                 * anymore for every captureRequest */
                needNewSess = TRUE;
                if (is_active == TRUE) {
                    // check if there is any ongoing jpeg jobs
                    if (pme->m_ongoingJpegQ.isEmpty()) {
                       CDBG("%s: ongoing jpeg queue is empty so doing the jpeg job", __func__);
                        // no ongoing jpeg job, we are fine to send jpeg encoding job
                        qcamera_hal3_jpeg_data_t *jpeg_job =
                            (qcamera_hal3_jpeg_data_t *)pme->m_inputJpegQ.dequeue();

                        if (NULL != jpeg_job) {
                            // add into ongoing jpeg job Q
                            pme->m_ongoingJpegQ.enqueue((void *)jpeg_job);

                            if (jpeg_job->fwk_frame) {
                                ret = pme->encodeFWKData(jpeg_job, needNewSess);
                            } else {
                                ret = pme->encodeData(jpeg_job, needNewSess);
                            }
                            if (NO_ERROR != ret) {
                                // dequeue the last one
                                pme->m_ongoingJpegQ.dequeue(false);

                                pme->releaseJpegJobData(jpeg_job);
                                free(jpeg_job);
                            }
                        }
                    }

                    // check if there are any framework pp jobs
                    if (!pme->m_inputFWKPPQ.isEmpty()) {
                        qcamera_fwk_input_pp_data_t *fwk_frame =
                                (qcamera_fwk_input_pp_data_t *) pme->m_inputFWKPPQ.dequeue();
                        if (NULL != fwk_frame) {
                            qcamera_hal3_pp_data_t *pp_job =
                                    (qcamera_hal3_pp_data_t *)malloc(sizeof(qcamera_hal3_pp_data_t));
                            jpeg_settings_t *jpeg_settings =
                                    (jpeg_settings_t *)pme->m_jpegSettingsQ.dequeue();
                            if (pp_job != NULL) {
                                memset(pp_job, 0, sizeof(qcamera_hal3_pp_data_t));
                                pp_job->jpeg_settings = jpeg_settings;
                                if (pme->m_pReprocChannel != NULL) {
                                    if (NO_ERROR != pme->m_pReprocChannel->extractCrop(fwk_frame)) {
                                        ALOGE("%s: Failed to extract output crop", __func__);
                                    }
                                    // add into ongoing PP job Q
                                    pp_job->fwk_src_frame = fwk_frame;
                                    pme->m_ongoingPPQ.enqueue((void *)pp_job);
                                    ret = pme->m_pReprocChannel->doReprocessOffline(fwk_frame);
                                    if (NO_ERROR != ret) {
                                        // remove from ongoing PP job Q
                                        pme->m_ongoingPPQ.dequeue(false);
                                    }
                                } else {
                                    ALOGE("%s: Reprocess channel is NULL", __func__);
                                    ret = -1;
                                }
                            } else {
                                ALOGE("%s: no mem for qcamera_hal3_pp_data_t", __func__);
                                ret = -1;
                            }

                            if (0 != ret) {
                                // free pp_job
                                if (pp_job != NULL) {
                                    free(pp_job);
                                }
                                // free frame
                                if (fwk_frame != NULL) {
                                    free(fwk_frame);
                                }
                            }
                        }
                    }

                    CDBG_HIGH("%s: dequeuing pp frame", __func__);
                    pthread_mutex_lock(&pme->mReprocJobLock);
                    if(!pme->m_inputPPQ.isEmpty() && !pme->m_inputMetaQ.isEmpty()) {
                        mm_camera_super_buf_t *pp_frame =
                            (mm_camera_super_buf_t *)pme->m_inputPPQ.dequeue();
                        meta_buffer =
                            (mm_camera_super_buf_t *)pme->m_inputMetaQ.dequeue();
                        jpeg_settings_t *jpeg_settings =
                           (jpeg_settings_t *)pme->m_jpegSettingsQ.dequeue();
                        pthread_mutex_unlock(&pme->mReprocJobLock);
                        qcamera_hal3_pp_data_t *pp_job =
                            (qcamera_hal3_pp_data_t *)malloc(sizeof(qcamera_hal3_pp_data_t));
                        if (pp_job == NULL) {
                            ALOGE("%s: no mem for qcamera_hal3_pp_data_t",
                                    __func__);
                            ret = -1;
                        } else if (meta_buffer == NULL) {
                            ALOGE("%s: no mem for mm_camera_super_buf_t",
                                    __func__);
                            ret = -1;
                        } else {
                            memset(pp_job, 0, sizeof(qcamera_hal3_pp_data_t));
                            pp_job->src_frame = pp_frame;
                            pp_job->src_metadata = meta_buffer;
                            if (meta_buffer->bufs[0] != NULL) {
                                pp_job->metadata = (metadata_buffer_t *)
                                        meta_buffer->bufs[0]->buffer;
                            }
                            pp_job->jpeg_settings = jpeg_settings;
                            pme->m_ongoingPPQ.enqueue((void *)pp_job);
                            if (pme->m_pReprocChannel != NULL) {
                                mm_camera_buf_def_t *meta_buffer_arg = NULL;
                                meta_buffer_arg = meta_buffer->bufs[0];
                                qcamera_fwk_input_pp_data_t fwk_frame;
                                memset(&fwk_frame, 0, sizeof(qcamera_fwk_input_pp_data_t));
                                ret = pme->m_pReprocChannel->extractFrameCropAndRotation(
                                        pp_frame, meta_buffer_arg,
                                        pp_job->jpeg_settings,
                                        fwk_frame);
                                if (NO_ERROR == ret) {
                                    // add into ongoing PP job Q
                                    ret = pme->m_pReprocChannel->doReprocessOffline(
                                            &fwk_frame);
                                    if (NO_ERROR != ret) {
                                        // remove from ongoing PP job Q
                                        pme->m_ongoingPPQ.dequeue(false);
                                    }
                                }
                            } else {
                                CDBG_HIGH("%s: No reprocess. Calling processPPData directly",
                                    __func__);
                                ret = pme->processPPData(pp_frame);
                            }
                        }

                        if (0 != ret) {
                            // free pp_job
                            if (pp_job != NULL) {
                                free(pp_job);
                            }
                            // free frame
                            if (pp_frame != NULL) {
                                pme->releaseSuperBuf(pp_frame);
                                free(pp_frame);
                            }
                            //free metadata
                            if (NULL != meta_buffer) {
                                pme->m_parent->metadataBufDone(meta_buffer);
                                free(meta_buffer);
                            }
                        }
                    } else {
                        pthread_mutex_unlock(&pme->mReprocJobLock);
                    }
                } else {
                    // not active, simply return buf and do no op
                    qcamera_hal3_jpeg_data_t *jpeg_job =
                        (qcamera_hal3_jpeg_data_t *)pme->m_inputJpegQ.dequeue();
                    if (NULL != jpeg_job) {
                        free(jpeg_job);
                    }
                    mm_camera_super_buf_t *super_buf;
                    super_buf = (mm_camera_super_buf_t *)pme->m_inputRawQ.dequeue();
                    if (NULL != super_buf) {
                        pme->releaseSuperBuf(super_buf);
                        free(super_buf);
                    }
                    super_buf = (mm_camera_super_buf_t *)pme->m_inputPPQ.dequeue();
                    if (NULL != super_buf) {
                        pme->releaseSuperBuf(super_buf);
                        free(super_buf);
                    }
                    mm_camera_super_buf_t *metadata = (mm_camera_super_buf_t *)pme->m_inputMetaQ.dequeue();
                    if (metadata != NULL) {
                        pme->m_parent->metadataBufDone(metadata);
                        free(metadata);
                    }
                    qcamera_fwk_input_pp_data_t *fwk_frame =
                            (qcamera_fwk_input_pp_data_t *) pme->m_inputFWKPPQ.dequeue();
                    if (NULL != fwk_frame) {
                        free(fwk_frame);
                    }
                }
            }
            break;
        case CAMERA_CMD_TYPE_EXIT:
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
 * FUNCTION   : QCamera3Exif
 *
 * DESCRIPTION: constructor of QCamera3Exif
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *==========================================================================*/
QCamera3Exif::QCamera3Exif()
    : m_nNumEntries(0)
{
    memset(m_Entries, 0, sizeof(m_Entries));
}

/*===========================================================================
 * FUNCTION   : ~QCamera3Exif
 *
 * DESCRIPTION: deconstructor of QCamera3Exif. Will release internal memory ptr.
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *==========================================================================*/
QCamera3Exif::~QCamera3Exif()
{
    for (uint32_t i = 0; i < m_nNumEntries; i++) {
        switch (m_Entries[i].tag_entry.type) {
            case EXIF_BYTE:
                {
                    if (m_Entries[i].tag_entry.count > 1 &&
                            m_Entries[i].tag_entry.data._bytes != NULL) {
                        free(m_Entries[i].tag_entry.data._bytes);
                        m_Entries[i].tag_entry.data._bytes = NULL;
                    }
                }
                break;
            case EXIF_ASCII:
                {
                    if (m_Entries[i].tag_entry.data._ascii != NULL) {
                        free(m_Entries[i].tag_entry.data._ascii);
                        m_Entries[i].tag_entry.data._ascii = NULL;
                    }
                }
                break;
            case EXIF_SHORT:
                {
                    if (m_Entries[i].tag_entry.count > 1 &&
                            m_Entries[i].tag_entry.data._shorts != NULL) {
                        free(m_Entries[i].tag_entry.data._shorts);
                        m_Entries[i].tag_entry.data._shorts = NULL;
                    }
                }
                break;
            case EXIF_LONG:
                {
                    if (m_Entries[i].tag_entry.count > 1 &&
                            m_Entries[i].tag_entry.data._longs != NULL) {
                        free(m_Entries[i].tag_entry.data._longs);
                        m_Entries[i].tag_entry.data._longs = NULL;
                    }
                }
                break;
            case EXIF_RATIONAL:
                {
                    if (m_Entries[i].tag_entry.count > 1 &&
                            m_Entries[i].tag_entry.data._rats != NULL) {
                        free(m_Entries[i].tag_entry.data._rats);
                        m_Entries[i].tag_entry.data._rats = NULL;
                    }
                }
                break;
            case EXIF_UNDEFINED:
                {
                    if (m_Entries[i].tag_entry.data._undefined != NULL) {
                        free(m_Entries[i].tag_entry.data._undefined);
                        m_Entries[i].tag_entry.data._undefined = NULL;
                    }
                }
                break;
            case EXIF_SLONG:
                {
                    if (m_Entries[i].tag_entry.count > 1 &&
                            m_Entries[i].tag_entry.data._slongs != NULL) {
                        free(m_Entries[i].tag_entry.data._slongs);
                        m_Entries[i].tag_entry.data._slongs = NULL;
                    }
                }
                break;
            case EXIF_SRATIONAL:
                {
                    if (m_Entries[i].tag_entry.count > 1 &&
                            m_Entries[i].tag_entry.data._srats != NULL) {
                        free(m_Entries[i].tag_entry.data._srats);
                        m_Entries[i].tag_entry.data._srats = NULL;
                    }
                }
                break;
            default:
                ALOGE("%s: Error, Unknown type",__func__);
                break;
        }
    }
}

/*===========================================================================
 * FUNCTION   : addEntry
 *
 * DESCRIPTION: function to add an entry to exif data
 *
 * PARAMETERS :
 *   @tagid   : exif tag ID
 *   @type    : data type
 *   @count   : number of data in uint of its type
 *   @data    : input data ptr
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3Exif::addEntry(exif_tag_id_t tagid,
                              exif_tag_type_t type,
                              uint32_t count,
                              void *data)
{
    int32_t rc = NO_ERROR;
    if(m_nNumEntries >= MAX_HAL3_EXIF_TABLE_ENTRIES) {
        ALOGE("%s: Number of entries exceeded limit", __func__);
        return NO_MEMORY;
    }

    m_Entries[m_nNumEntries].tag_id = tagid;
    m_Entries[m_nNumEntries].tag_entry.type = type;
    m_Entries[m_nNumEntries].tag_entry.count = count;
    m_Entries[m_nNumEntries].tag_entry.copy = 1;
    switch (type) {
        case EXIF_BYTE:
            {
                if (count > 1) {
                    uint8_t *values = (uint8_t *)malloc(count);
                    if (values == NULL) {
                        ALOGE("%s: No memory for byte array", __func__);
                        rc = NO_MEMORY;
                    } else {
                        memcpy(values, data, count);
                        m_Entries[m_nNumEntries].tag_entry.data._bytes = values;
                    }
                } else {
                    m_Entries[m_nNumEntries].tag_entry.data._byte =
                        *(uint8_t *)data;
                }
            }
            break;
        case EXIF_ASCII:
            {
                char *str = NULL;
                str = (char *)malloc(count + 1);
                if (str == NULL) {
                    ALOGE("%s: No memory for ascii string", __func__);
                    rc = NO_MEMORY;
                } else {
                    memset(str, 0, count + 1);
                    memcpy(str, data, count);
                    m_Entries[m_nNumEntries].tag_entry.data._ascii = str;
                }
            }
            break;
        case EXIF_SHORT:
            {
                if (count > 1) {
                    uint16_t *values =
                        (uint16_t *)malloc(count * sizeof(uint16_t));
                    if (values == NULL) {
                        ALOGE("%s: No memory for short array", __func__);
                        rc = NO_MEMORY;
                    } else {
                        memcpy(values, data, count * sizeof(uint16_t));
                        m_Entries[m_nNumEntries].tag_entry.data._shorts =values;
                    }
                } else {
                    m_Entries[m_nNumEntries].tag_entry.data._short =
                        *(uint16_t *)data;
                }
            }
            break;
        case EXIF_LONG:
            {
                if (count > 1) {
                    uint32_t *values =
                        (uint32_t *)malloc(count * sizeof(uint32_t));
                    if (values == NULL) {
                        ALOGE("%s: No memory for long array", __func__);
                        rc = NO_MEMORY;
                    } else {
                        memcpy(values, data, count * sizeof(uint32_t));
                        m_Entries[m_nNumEntries].tag_entry.data._longs = values;
                    }
                } else {
                    m_Entries[m_nNumEntries].tag_entry.data._long =
                        *(uint32_t *)data;
                }
            }
            break;
        case EXIF_RATIONAL:
            {
                if (count > 1) {
                    rat_t *values = (rat_t *)malloc(count * sizeof(rat_t));
                    if (values == NULL) {
                        ALOGE("%s: No memory for rational array", __func__);
                        rc = NO_MEMORY;
                    } else {
                        memcpy(values, data, count * sizeof(rat_t));
                        m_Entries[m_nNumEntries].tag_entry.data._rats = values;
                    }
                } else {
                    m_Entries[m_nNumEntries].tag_entry.data._rat =
                        *(rat_t *)data;
                }
            }
            break;
        case EXIF_UNDEFINED:
            {
                uint8_t *values = (uint8_t *)malloc(count);
                if (values == NULL) {
                    ALOGE("%s: No memory for undefined array", __func__);
                    rc = NO_MEMORY;
                } else {
                    memcpy(values, data, count);
                    m_Entries[m_nNumEntries].tag_entry.data._undefined = values;
                }
            }
            break;
        case EXIF_SLONG:
            {
                if (count > 1) {
                    int32_t *values =
                        (int32_t *)malloc(count * sizeof(int32_t));
                    if (values == NULL) {
                        ALOGE("%s: No memory for signed long array", __func__);
                        rc = NO_MEMORY;
                    } else {
                        memcpy(values, data, count * sizeof(int32_t));
                        m_Entries[m_nNumEntries].tag_entry.data._slongs =values;
                    }
                } else {
                    m_Entries[m_nNumEntries].tag_entry.data._slong =
                        *(int32_t *)data;
                }
            }
            break;
        case EXIF_SRATIONAL:
            {
                if (count > 1) {
                    srat_t *values = (srat_t *)malloc(count * sizeof(srat_t));
                    if (values == NULL) {
                        ALOGE("%s: No memory for sign rational array",__func__);
                        rc = NO_MEMORY;
                    } else {
                        memcpy(values, data, count * sizeof(srat_t));
                        m_Entries[m_nNumEntries].tag_entry.data._srats = values;
                    }
                } else {
                    m_Entries[m_nNumEntries].tag_entry.data._srat =
                        *(srat_t *)data;
                }
            }
            break;
        default:
            ALOGE("%s: Error, Unknown type",__func__);
            break;
    }

    // Increase number of entries
    m_nNumEntries++;
    return rc;
}

}; // namespace qcamera
