/* Copyright (c) 2012-2016, The Linux Foundataion. All rights reserved.
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

#define LOG_TAG "QCamera2HWI"

#include <time.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <utils/Errors.h>
#include <utils/Timers.h>

#ifndef USE_MEDIA_EXTENSIONS
#include <QComOMXMetadata.h>
#endif

#include "QCamera2HWI.h"

namespace qcamera {

/*===========================================================================
 * FUNCTION   : zsl_channel_cb
 *
 * DESCRIPTION: helper function to handle ZSL superbuf callback directly from
 *              mm-camera-interface
 *
 * PARAMETERS :
 *   @recvd_frame : received super buffer
 *   @userdata    : user data ptr
 *
 * RETURN    : None
 *
 * NOTE      : recvd_frame will be released after this call by caller, so if
 *             async operation needed for recvd_frame, it's our responsibility
 *             to save a copy for this variable to be used later.
 *==========================================================================*/
void QCamera2HardwareInterface::zsl_channel_cb(mm_camera_super_buf_t *recvd_frame,
                                               void *userdata)
{
    ATRACE_CALL();
    CDBG_HIGH("[KPI Perf] %s: E",__func__);
    char value[PROPERTY_VALUE_MAX];
    bool dump_raw = false;
    bool dump_yuv = false;
    bool log_matching = false;
    QCamera2HardwareInterface *pme = (QCamera2HardwareInterface *)userdata;
    if (pme == NULL ||
        pme->mCameraHandle == NULL ||
        pme->mCameraHandle->camera_handle != recvd_frame->camera_handle){
       ALOGE("%s: camera obj not valid", __func__);
       return;
    }

    QCameraChannel *pChannel = pme->m_channels[QCAMERA_CH_TYPE_ZSL];
    if (pChannel == NULL ||
        pChannel->getMyHandle() != recvd_frame->ch_id) {
        ALOGE("%s: ZSL channel doesn't exist, return here", __func__);
        return;
    }

    if(pme->mParameters.isSceneSelectionEnabled() &&
            !pme->m_stateMachine.isCaptureRunning()) {
        pme->selectScene(pChannel, recvd_frame);
        pChannel->bufDone(recvd_frame);
        return;
    }

    CDBG_HIGH("%s: [ZSL Retro] Frame CB Unlock : %d, is AEC Locked: %d",
          __func__, recvd_frame->bUnlockAEC, pme->m_bLedAfAecLock);
    if(recvd_frame->bUnlockAEC && pme->m_bLedAfAecLock) {
        qcamera_sm_internal_evt_payload_t *payload =
                (qcamera_sm_internal_evt_payload_t *)malloc(
                        sizeof(qcamera_sm_internal_evt_payload_t));
        if (NULL != payload) {
            memset(payload, 0, sizeof(qcamera_sm_internal_evt_payload_t));
            payload->evt_type = QCAMERA_INTERNAL_EVT_RETRO_AEC_UNLOCK;
            int32_t rc = pme->processEvt(QCAMERA_SM_EVT_EVT_INTERNAL, payload);
            if (rc != NO_ERROR) {
                ALOGE("%s: processEvt for retro AEC unlock failed", __func__);
                free(payload);
                payload = NULL;
            }
        } else {
            ALOGE("%s: No memory for retro AEC event", __func__);
        }
    }

    // Check if retro-active frames are completed and camera is
    // ready to go ahead with LED estimation for regular frames
    if (recvd_frame->bReadyForPrepareSnapshot) {
      // Send an event
      CDBG_HIGH("%s: [ZSL Retro] Ready for Prepare Snapshot, signal ", __func__);
      qcamera_sm_internal_evt_payload_t *payload =
         (qcamera_sm_internal_evt_payload_t *)malloc(sizeof(qcamera_sm_internal_evt_payload_t));
      if (NULL != payload) {
        memset(payload, 0, sizeof(qcamera_sm_internal_evt_payload_t));
        payload->evt_type = QCAMERA_INTERNAL_EVT_READY_FOR_SNAPSHOT;
        int32_t rc = pme->processEvt(QCAMERA_SM_EVT_EVT_INTERNAL, payload);
        if (rc != NO_ERROR) {
          ALOGE("%s: processEvt Ready for Snaphot failed", __func__);
          free(payload);
          payload = NULL;
        }
      } else {
        ALOGE("%s: No memory for prepare signal event detect"
              " qcamera_sm_internal_evt_payload_t", __func__);
      }
    }

    /* indicate the parent that capture is done */
    pme->captureDone();

    // save a copy for the superbuf
    mm_camera_super_buf_t* frame =
               (mm_camera_super_buf_t *)malloc(sizeof(mm_camera_super_buf_t));
    if (frame == NULL) {
        ALOGE("%s: Error allocating memory to save received_frame structure.", __func__);
        pChannel->bufDone(recvd_frame);
        return;
    }
    *frame = *recvd_frame;

    if (recvd_frame->num_bufs > 0) {
        ALOGI("[KPI Perf] %s: superbuf frame_idx %d", __func__,
            recvd_frame->bufs[0]->frame_idx);
    }

    // DUMP RAW if available
    property_get("persist.camera.zsl_raw", value, "0");
    dump_raw = atoi(value) > 0 ? true : false;
    if (dump_raw) {
        for (uint32_t i = 0; i < recvd_frame->num_bufs; i++) {
            if (recvd_frame->bufs[i]->stream_type == CAM_STREAM_TYPE_RAW) {
                mm_camera_buf_def_t * raw_frame = recvd_frame->bufs[i];
                QCameraStream *pStream = pChannel->getStreamByHandle(raw_frame->stream_id);
                if (NULL != pStream) {
                    pme->dumpFrameToFile(pStream, raw_frame, QCAMERA_DUMP_FRM_RAW);
                }
                break;
            }
        }
    }

    // DUMP YUV before reprocess if needed
    property_get("persist.camera.zsl_yuv", value, "0");
    dump_yuv = atoi(value) > 0 ? true : false;
    if (dump_yuv) {
        for (uint32_t i = 0; i < recvd_frame->num_bufs; i++) {
            if (recvd_frame->bufs[i]->stream_type == CAM_STREAM_TYPE_SNAPSHOT) {
                mm_camera_buf_def_t * yuv_frame = recvd_frame->bufs[i];
                QCameraStream *pStream = pChannel->getStreamByHandle(yuv_frame->stream_id);
                if (NULL != pStream) {
                    pme->dumpFrameToFile(pStream, yuv_frame, QCAMERA_DUMP_FRM_SNAPSHOT);
                }
                break;
            }
        }
    }
    //
    // whether need FD Metadata along with Snapshot frame in ZSL mode
    if(pme->needFDMetadata(QCAMERA_CH_TYPE_ZSL)){
        //Need Face Detection result for snapshot frames
        //Get the Meta Data frames
        mm_camera_buf_def_t *pMetaFrame = NULL;
        for (uint32_t i = 0; i < frame->num_bufs; i++) {
            QCameraStream *pStream = pChannel->getStreamByHandle(frame->bufs[i]->stream_id);
            if (pStream != NULL) {
                if (pStream->isTypeOf(CAM_STREAM_TYPE_METADATA)) {
                    pMetaFrame = frame->bufs[i]; //find the metadata
                    break;
                }
            }
        }

        if(pMetaFrame != NULL){
            metadata_buffer_t *pMetaData = (metadata_buffer_t *)pMetaFrame->buffer;
            //send the face detection info
            uint8_t found = 0;
            cam_face_detection_data_t faces_data;
            IF_META_AVAILABLE(cam_face_detection_data_t, p_faces_data,
                    CAM_INTF_META_FACE_DETECTION, pMetaData) {
                faces_data = *p_faces_data;
                found = 1;
            } else {
                memset(&faces_data, 0, sizeof(cam_face_detection_data_t));
            }
            faces_data.fd_type = QCAMERA_FD_SNAPSHOT; //HARD CODE here before MCT can support
            if(!found){
                faces_data.num_faces_detected = 0;
            }else if(faces_data.num_faces_detected > MAX_ROI){
                ALOGE("%s: Invalid number of faces %d",
                    __func__, faces_data.num_faces_detected);
            }
            qcamera_sm_internal_evt_payload_t *payload =
                (qcamera_sm_internal_evt_payload_t *)malloc(sizeof(qcamera_sm_internal_evt_payload_t));
            if (NULL != payload) {
                memset(payload, 0, sizeof(qcamera_sm_internal_evt_payload_t));
                payload->evt_type = QCAMERA_INTERNAL_EVT_FACE_DETECT_RESULT;
                payload->faces_data = faces_data;
                int32_t rc = pme->processEvt(QCAMERA_SM_EVT_EVT_INTERNAL, payload);
                if (rc != NO_ERROR) {
                    ALOGE("%s: processEvt face_detection_result failed", __func__);
                    free(payload);
                    payload = NULL;
                }
            } else {
                ALOGE("%s: No memory for face_detection_result qcamera_sm_internal_evt_payload_t", __func__);
            }
        }
    }

    property_get("persist.camera.dumpmetadata", value, "0");
    int32_t enabled = atoi(value);
    if (enabled) {
        mm_camera_buf_def_t *pMetaFrame = NULL;
        QCameraStream *pStream = NULL;
        for (uint32_t i = 0; i < frame->num_bufs; i++) {
            pStream = pChannel->getStreamByHandle(frame->bufs[i]->stream_id);
            if (pStream != NULL) {
                if (pStream->isTypeOf(CAM_STREAM_TYPE_METADATA)) {
                    pMetaFrame = frame->bufs[i];
                    if (pMetaFrame != NULL &&
                            ((metadata_buffer_t *)pMetaFrame->buffer)->is_tuning_params_valid) {
                        pme->dumpMetadataToFile(pStream, pMetaFrame, (char *) "ZSL_Snapshot");
                    }
                    break;
                }
            }
        }
    }

    property_get("persist.camera.zsl_matching", value, "0");
    log_matching = atoi(value) > 0 ? true : false;
    if (log_matching) {
        CDBG_HIGH("%s : ZSL super buffer contains:", __func__);
        QCameraStream *pStream = NULL;
        for (uint32_t i = 0; i < frame->num_bufs; i++) {
            pStream = pChannel->getStreamByHandle(frame->bufs[i]->stream_id);
            if (pStream != NULL ) {
                CDBG_HIGH("%s: Buffer with V4L index %d frame index %d of type %d Timestamp: %ld %ld ",
                        __func__,
                        frame->bufs[i]->buf_idx,
                        frame->bufs[i]->frame_idx,
                        pStream->getMyType(),
                        frame->bufs[i]->ts.tv_sec,
                        frame->bufs[i]->ts.tv_nsec);
            }
        }
    }

    // Wait on Postproc initialization if needed
    pme->waitDeferredWork(pme->mReprocJob);

    // send to postprocessor
    pme->m_postprocessor.processData(frame);

    CDBG_HIGH("[KPI Perf] %s: X", __func__);
}

/*===========================================================================
 * FUNCTION   : selectScene
 *
 * DESCRIPTION: send a preview callback when a specific selected scene is applied
 *
 * PARAMETERS :
 *   @pChannel: Camera channel
 *   @frame   : Bundled super buffer
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::selectScene(QCameraChannel *pChannel,
        mm_camera_super_buf_t *frame)
{
    mm_camera_buf_def_t *pMetaFrame = NULL;
    QCameraStream *pStream = NULL;
    int32_t rc = NO_ERROR;

    if ((NULL == frame) || (NULL == pChannel)) {
        ALOGE("%s: Invalid scene select input", __func__);
        return BAD_VALUE;
    }

    cam_scene_mode_type selectedScene = mParameters.getSelectedScene();
    if (CAM_SCENE_MODE_MAX == selectedScene) {
        ALOGV("%s: No selected scene", __func__);
        return NO_ERROR;
    }

    for (uint32_t i = 0; i < frame->num_bufs; i++) {
        pStream = pChannel->getStreamByHandle(frame->bufs[i]->stream_id);
        if (pStream != NULL) {
            if (pStream->isTypeOf(CAM_STREAM_TYPE_METADATA)) {
                pMetaFrame = frame->bufs[i];
                break;
            }
        }
    }

    if (NULL == pMetaFrame) {
        ALOGE("%s: No metadata buffer found in scene select super buffer", __func__);
        return NO_INIT;
    }

    metadata_buffer_t *pMetaData = (metadata_buffer_t *)pMetaFrame->buffer;

    IF_META_AVAILABLE(cam_scene_mode_type, scene, CAM_INTF_META_CURRENT_SCENE, pMetaData) {
        if ((*scene == selectedScene) &&
                (mDataCb != NULL) &&
                (msgTypeEnabledWithLock(CAMERA_MSG_PREVIEW_FRAME) > 0)) {
            mm_camera_buf_def_t *preview_frame = NULL;
            for (uint32_t i = 0; i < frame->num_bufs; i++) {
                pStream = pChannel->getStreamByHandle(frame->bufs[i]->stream_id);
                if (pStream != NULL) {
                    if (pStream->isTypeOf(CAM_STREAM_TYPE_PREVIEW)) {
                        preview_frame = frame->bufs[i];
                        break;
                    }
                }
            }
            if (preview_frame) {
                QCameraGrallocMemory *memory = (QCameraGrallocMemory *)preview_frame->mem_info;
                uint32_t idx = preview_frame->buf_idx;
                rc = sendPreviewCallback(pStream, memory, idx);
                if (NO_ERROR != rc) {
                    ALOGE("%s: Error triggering scene select preview callback", __func__);
                } else {
                    mParameters.setSelectedScene(CAM_SCENE_MODE_MAX);
                }
            } else {
                ALOGE("%s: No preview buffer found in scene select super buffer", __func__);
                return NO_INIT;
            }
        }
    } else {
        ALOGE("%s: No current scene metadata!", __func__);
        rc = NO_INIT;
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : capture_channel_cb_routine
 *
 * DESCRIPTION: helper function to handle snapshot superbuf callback directly from
 *              mm-camera-interface
 *
 * PARAMETERS :
 *   @recvd_frame : received super buffer
 *   @userdata    : user data ptr
 *
 * RETURN    : None
 *
 * NOTE      : recvd_frame will be released after this call by caller, so if
 *             async operation needed for recvd_frame, it's our responsibility
 *             to save a copy for this variable to be used later.
*==========================================================================*/
void QCamera2HardwareInterface::capture_channel_cb_routine(mm_camera_super_buf_t *recvd_frame,
                                                           void *userdata)
{
    KPI_ATRACE_CALL();
    char value[PROPERTY_VALUE_MAX];
    CDBG_HIGH("[KPI Perf] %s: E PROFILE_YUV_CB_TO_HAL", __func__);
    bool dump_yuv = false;
    QCamera2HardwareInterface *pme = (QCamera2HardwareInterface *)userdata;
    if (pme == NULL ||
        pme->mCameraHandle == NULL ||
        pme->mCameraHandle->camera_handle != recvd_frame->camera_handle){
        ALOGE("%s: camera obj not valid", __func__);
        return;
    }

    QCameraChannel *pChannel = pme->m_channels[QCAMERA_CH_TYPE_CAPTURE];
    if (pChannel == NULL ||
        pChannel->getMyHandle() != recvd_frame->ch_id) {
        ALOGE("%s: Capture channel doesn't exist, return here", __func__);
        return;
    }

    // save a copy for the superbuf
    mm_camera_super_buf_t* frame =
               (mm_camera_super_buf_t *)malloc(sizeof(mm_camera_super_buf_t));
    if (frame == NULL) {
        ALOGE("%s: Error allocating memory to save received_frame structure.", __func__);
        pChannel->bufDone(recvd_frame);
        return;
    }
    *frame = *recvd_frame;

    // DUMP YUV before reprocess if needed
    property_get("persist.camera.nonzsl.yuv", value, "0");
    dump_yuv = atoi(value) > 0 ? true : false;
    if ( dump_yuv ) {
        for ( uint32_t i= 0 ; i < recvd_frame->num_bufs ; i++ ) {
            if ( recvd_frame->bufs[i]->stream_type == CAM_STREAM_TYPE_SNAPSHOT ) {
                mm_camera_buf_def_t * yuv_frame = recvd_frame->bufs[i];
                QCameraStream *pStream = pChannel->getStreamByHandle(yuv_frame->stream_id);
                if ( NULL != pStream ) {
                    pme->dumpFrameToFile(pStream, yuv_frame, QCAMERA_DUMP_FRM_SNAPSHOT);
                }
                break;
            }
        }
    }

    property_get("persist.camera.dumpmetadata", value, "0");
    int32_t enabled = atoi(value);
    if (enabled) {
        mm_camera_buf_def_t *pMetaFrame = NULL;
        QCameraStream *pStream = NULL;
        for (uint32_t i = 0; i < frame->num_bufs; i++) {
            pStream = pChannel->getStreamByHandle(frame->bufs[i]->stream_id);
            if (pStream != NULL) {
                if (pStream->isTypeOf(CAM_STREAM_TYPE_METADATA)) {
                    pMetaFrame = frame->bufs[i]; //find the metadata
                    if (pMetaFrame != NULL &&
                            ((metadata_buffer_t *)pMetaFrame->buffer)->is_tuning_params_valid) {
                        pme->dumpMetadataToFile(pStream, pMetaFrame, (char *) "Snapshot");
                    }
                    break;
                }
            }
        }
    }

    // Wait on Postproc initialization if needed
    pme->waitDeferredWork(pme->mReprocJob);

    // send to postprocessor
    pme->m_postprocessor.processData(frame);

/* START of test register face image for face authentication */
#ifdef QCOM_TEST_FACE_REGISTER_FACE
    static uint8_t bRunFaceReg = 1;

    if (bRunFaceReg > 0) {
        // find snapshot frame
        QCameraStream *main_stream = NULL;
        mm_camera_buf_def_t *main_frame = NULL;
        for (int i = 0; i < recvd_frame->num_bufs; i++) {
            QCameraStream *pStream =
                pChannel->getStreamByHandle(recvd_frame->bufs[i]->stream_id);
            if (pStream != NULL) {
                if (pStream->isTypeOf(CAM_STREAM_TYPE_SNAPSHOT)) {
                    main_stream = pStream;
                    main_frame = recvd_frame->bufs[i];
                    break;
                }
            }
        }
        if (main_stream != NULL && main_frame != NULL) {
            int32_t faceId = -1;
            cam_pp_offline_src_config_t config;
            memset(&config, 0, sizeof(cam_pp_offline_src_config_t));
            config.num_of_bufs = 1;
            main_stream->getFormat(config.input_fmt);
            main_stream->getFrameDimension(config.input_dim);
            main_stream->getFrameOffset(config.input_buf_planes.plane_info);
            CDBG_HIGH("DEBUG: registerFaceImage E");
            int32_t rc = pme->registerFaceImage(main_frame->buffer, &config, faceId);
            CDBG_HIGH("DEBUG: registerFaceImage X, ret=%d, faceId=%d", rc, faceId);
            bRunFaceReg = 0;
        }
    }

#endif
/* END of test register face image for face authentication */

    CDBG_HIGH("[KPI Perf] %s: X", __func__);
}
#ifdef TARGET_TS_MAKEUP
bool QCamera2HardwareInterface::TsMakeupProcess_Preview(mm_camera_buf_def_t *pFrame,
        QCameraStream * pStream) {
    CDBG("%s begin",__func__);
    bool bRet = false;
    if (pStream == NULL || pFrame == NULL) {
        bRet = false;
        CDBG_HIGH("%s pStream == NULL || pFrame == NULL",__func__);
    } else {
        bRet = TsMakeupProcess(pFrame, pStream, mFaceRect);
    }
    CDBG("%s end bRet = %d ",__func__,bRet);
    return bRet;
}

bool QCamera2HardwareInterface::TsMakeupProcess_Snapshot(mm_camera_buf_def_t *pFrame,
        QCameraStream * pStream) {
    CDBG("%s begin",__func__);
    bool bRet = false;
    if (pStream == NULL || pFrame == NULL) {
        bRet = false;
        CDBG_HIGH("%s pStream == NULL || pFrame == NULL",__func__);
    } else {
        cam_frame_len_offset_t offset;
        memset(&offset, 0, sizeof(cam_frame_len_offset_t));
        pStream->getFrameOffset(offset);

        cam_dimension_t dim;
        pStream->getFrameDimension(dim);

        unsigned char *yBuf  = (unsigned char*)pFrame->buffer;
        unsigned char *uvBuf = yBuf + offset.mp[0].len;
        TSMakeupDataEx inMakeupData;
        inMakeupData.frameWidth  = dim.width;
        inMakeupData.frameHeight = dim.height;
        inMakeupData.yBuf  = yBuf;
        inMakeupData.uvBuf = uvBuf;
        inMakeupData.yStride  = offset.mp[0].stride;
        inMakeupData.uvStride = offset.mp[1].stride;
        CDBG("%s detect begin",__func__);
        TSHandle fd_handle = ts_detectface_create_context();
        if (fd_handle != NULL) {
            cam_format_t fmt;
            pStream->getFormat(fmt);
            int iret = ts_detectface_detectEx(fd_handle, &inMakeupData);
            CDBG("%s ts_detectface_detect iret = %d",__func__,iret);
            if (iret <= 0) {
                bRet = false;
            } else {
                TSRect faceRect;
                memset(&faceRect,-1,sizeof(TSRect));
                iret = ts_detectface_get_face_info(fd_handle, 0, &faceRect, NULL,NULL,NULL);
                CDBG("%s ts_detectface_get_face_info iret=%d,faceRect.left=%ld,"
                        "faceRect.top=%ld,faceRect.right=%ld,faceRect.bottom=%ld"
                        ,__func__,iret,faceRect.left,faceRect.top,faceRect.right,faceRect.bottom);
                bRet = TsMakeupProcess(pFrame,pStream,faceRect);
            }
            ts_detectface_destroy_context(&fd_handle);
            fd_handle = NULL;
        } else {
            CDBG_HIGH("%s fd_handle == NULL",__func__);
        }
        CDBG("%s detect end",__func__);
    }
    CDBG("%s end bRet = %d ",__func__,bRet);
    return bRet;
}

bool QCamera2HardwareInterface::TsMakeupProcess(mm_camera_buf_def_t *pFrame,
        QCameraStream * pStream,TSRect& faceRect) {
    bool bRet = false;
    CDBG("%s begin",__func__);
    if (pStream == NULL || pFrame == NULL) {
        CDBG_HIGH("%s pStream == NULL || pFrame == NULL ",__func__);
        return false;
    }
    pthread_mutex_lock(&m_parm_lock);
    const char* pch_makeup_enable = mParameters.get(QCameraParameters::KEY_TS_MAKEUP);
    pthread_mutex_unlock(&m_parm_lock);
    if (pch_makeup_enable == NULL) {
        CDBG_HIGH("%s pch_makeup_enable = null",__func__);
        return false;
    }
    bool enableMakeUp = (strcmp(pch_makeup_enable,"On") == 0) && (faceRect.left > -1);
    CDBG("%s pch_makeup_enable = %s ",__func__,pch_makeup_enable);
    if (enableMakeUp) {
        cam_dimension_t dim;
        cam_frame_len_offset_t offset;
        pStream->getFrameDimension(dim);
        pStream->getFrameOffset(offset);
        pthread_mutex_lock(&m_parm_lock);
        int whiteLevel = mParameters.getInt(QCameraParameters::KEY_TS_MAKEUP_WHITEN),
        cleanLevel = mParameters.getInt(QCameraParameters::KEY_TS_MAKEUP_CLEAN);
        pthread_mutex_unlock(&(m_parm_lock));
        unsigned char *tempOriBuf = NULL;

        tempOriBuf = (unsigned char*)pFrame->buffer;
        unsigned char *yBuf = tempOriBuf;
        unsigned char *uvBuf = tempOriBuf + offset.mp[0].len;
        unsigned char *tmpBuf = new unsigned char[offset.frame_len];
        if (tmpBuf == NULL) {
            CDBG_HIGH("%s tmpBuf == NULL ",__func__);
            return false;
        }
        TSMakeupDataEx inMakeupData, outMakeupData;
        whiteLevel =  whiteLevel <= 0 ? 0 : (whiteLevel >= 100 ? 100 : whiteLevel);
        cleanLevel =  cleanLevel <= 0 ? 0 : (cleanLevel >= 100 ? 100 : cleanLevel);
        inMakeupData.frameWidth = dim.width;  // NV21 Frame width  > 0
        inMakeupData.frameHeight = dim.height; // NV21 Frame height > 0
        inMakeupData.yBuf =  yBuf; //  Y buffer pointer
        inMakeupData.uvBuf = uvBuf; // VU buffer pointer
        inMakeupData.yStride  = offset.mp[0].stride;
        inMakeupData.uvStride = offset.mp[1].stride;
        outMakeupData.frameWidth = dim.width; // NV21 Frame width  > 0
        outMakeupData.frameHeight = dim.height; // NV21 Frame height > 0
        outMakeupData.yBuf =  tmpBuf; //  Y buffer pointer
        outMakeupData.uvBuf = tmpBuf + offset.mp[0].len; // VU buffer pointer
        outMakeupData.yStride  = offset.mp[0].stride;
        outMakeupData.uvStride = offset.mp[1].stride;
        CDBG("%s: faceRect:left 2:%ld,,right:%ld,,top:%ld,,bottom:%ld,,Level:%dx%d",
            __func__,
            faceRect.left,faceRect.right,faceRect.top,faceRect.bottom,cleanLevel,whiteLevel);
        ts_makeup_skin_beautyEx(&inMakeupData, &outMakeupData, &(faceRect),cleanLevel,whiteLevel);
        memcpy((unsigned char*)pFrame->buffer, tmpBuf, offset.frame_len);
        QCameraMemory *memory = (QCameraMemory *)pFrame->mem_info;
        memory->cleanCache(pFrame->buf_idx);
        if (tmpBuf != NULL) {
            delete[] tmpBuf;
            tmpBuf = NULL;
        }
    }
    CDBG("%s end bRet = %d ",__func__,bRet);
    return bRet;
}
#endif
/*===========================================================================
 * FUNCTION   : postproc_channel_cb_routine
 *
 * DESCRIPTION: helper function to handle postprocess superbuf callback directly from
 *              mm-camera-interface
 *
 * PARAMETERS :
 *   @recvd_frame : received super buffer
 *   @userdata    : user data ptr
 *
 * RETURN    : None
 *
 * NOTE      : recvd_frame will be released after this call by caller, so if
 *             async operation needed for recvd_frame, it's our responsibility
 *             to save a copy for this variable to be used later.
*==========================================================================*/
void QCamera2HardwareInterface::postproc_channel_cb_routine(mm_camera_super_buf_t *recvd_frame,
                                                            void *userdata)
{
    ATRACE_CALL();
    CDBG_HIGH("[KPI Perf] %s: E", __func__);
    QCamera2HardwareInterface *pme = (QCamera2HardwareInterface *)userdata;
    if (pme == NULL ||
        pme->mCameraHandle == NULL ||
        pme->mCameraHandle->camera_handle != recvd_frame->camera_handle){
        ALOGE("%s: camera obj not valid", __func__);
        return;
    }

    // save a copy for the superbuf
    mm_camera_super_buf_t* frame =
               (mm_camera_super_buf_t *)malloc(sizeof(mm_camera_super_buf_t));
    if (frame == NULL) {
        ALOGE("%s: Error allocating memory to save received_frame structure.", __func__);
        return;
    }
    *frame = *recvd_frame;

    // Wait on JPEG create session
    pme->waitDeferredWork(pme->mJpegJob);

    // send to postprocessor
    pme->m_postprocessor.processPPData(frame);

    ATRACE_INT("Camera:Reprocess", 0);
    CDBG_HIGH("[KPI Perf] %s: X", __func__);
}

/*===========================================================================
 * FUNCTION   : synchronous_stream_cb_routine
 *
 * DESCRIPTION: Function to handle STREAM SYNC CALLBACKS
 *
 * PARAMETERS :
 *   @super_frame : received super buffer
 *   @stream      : stream object
 *   @userdata    : user data ptr
 *
 * RETURN    : None
 *
 * NOTE      : This Function is excecuted in mm-interface context.
 *             Avoid adding latency on this thread.
 *==========================================================================*/
void QCamera2HardwareInterface::synchronous_stream_cb_routine(
        mm_camera_super_buf_t *super_frame, QCameraStream * stream,
        void *userdata)
{
    nsecs_t frameTime = 0, mPreviewTimestamp = 0;
    int err = NO_ERROR;

    ATRACE_CALL();
    CDBG_HIGH("[KPI Perf] %s : BEGIN", __func__);
    QCamera2HardwareInterface *pme = (QCamera2HardwareInterface *)userdata;
    QCameraGrallocMemory *memory = NULL;

    if (pme == NULL) {
        ALOGE("%s: Invalid hardware object", __func__);
        return;
    }
    if (super_frame == NULL) {
        ALOGE("%s: Invalid super buffer", __func__);
        return;
    }
    mm_camera_buf_def_t *frame = super_frame->bufs[0];
    if (NULL == frame) {
        ALOGE("%s: Frame is NULL", __func__);
        return;
    }

    if (stream->getMyType() != CAM_STREAM_TYPE_PREVIEW) {
        ALOGE("%s: This is only for PREVIEW stream for now", __func__);
        return;
    }

    if (!pme->needProcessPreviewFrame()) {
        ALOGE("%s: preview is not running, no need to process", __func__);
        return;
    }

#if WINDOW_TIMESTAMP
    frameTime = nsecs_t(frame->ts.tv_sec) * 1000000000LL + frame->ts.tv_nsec;
    if(pme->m_bPreviewStarted) {
        cam_fps_range_t fpsRange = pme->mParameters.getFpsRange();
        nsecs_t previewRate = 0;
        ALOGI("[KPI Perf] %s : PROFILE_FIRST_PREVIEW_FRAME fps = %d", __func__, fpsRange.min_fps);
        pme->m_bPreviewStarted = false ;
        if (fpsRange.min_fps != 0) {
            previewRate = (nsecs_t)(((int)(1000/fpsRange.min_fps) * 1000) * 1000000LL);
        }
        mPreviewTimestamp = frameTime + previewRate;
    } else {
        nsecs_t diff = (nsecs_t)(frameTime - stream->mStreamTimestamp);
        if (diff >= 0) {
            mPreviewTimestamp = frameTime + diff;
        } else {
            ALOGE ("%s: Issue in frame timestamp", __func__);
            mPreviewTimestamp = frameTime;
        }
    }
    // Convert Boottime from camera to Monotime for display if needed.
    // Otherwise, mBootToMonoTimestampOffset value will be 0.
    mPreviewTimestamp = mPreviewTimestamp - pme->mBootToMonoTimestampOffset;
    stream->mStreamTimestamp = frameTime;
#endif
    memory = (QCameraGrallocMemory *)super_frame->bufs[0]->mem_info;

#ifdef TARGET_TS_MAKEUP
    pme->TsMakeupProcess_Preview(frame,stream);
#endif

    // Enqueue  buffer to gralloc.
    uint32_t idx = frame->buf_idx;
    CDBG("%p Enqueue Buffer to display %d frame Time = %lld Display Time = %lld",
            pme, idx, frameTime, mPreviewTimestamp);
    err = memory->enqueueBuffer(idx, mPreviewTimestamp);

    if (err == NO_ERROR) {
        pthread_mutex_lock(&pme->mGrallocLock);
        pme->mEnqueuedBuffers++;
        pthread_mutex_unlock(&pme->mGrallocLock);
    } else {
        ALOGE ("%s: Enqueue Buffer failed", __func__);
    }

    CDBG_HIGH("[KPI Perf] %s : END", __func__);
    return;
}

/*===========================================================================
 * FUNCTION   : preview_stream_cb_routine
 *
 * DESCRIPTION: helper function to handle preview frame from preview stream in
 *              normal case with display.
 *
 * PARAMETERS :
 *   @super_frame : received super buffer
 *   @stream      : stream object
 *   @userdata    : user data ptr
 *
 * RETURN    : None
 *
 * NOTE      : caller passes the ownership of super_frame, it's our
 *             responsibility to free super_frame once it's done. The new
 *             preview frame will be sent to display, and an older frame
 *             will be dequeued from display and needs to be returned back
 *             to kernel for future use.
 *==========================================================================*/
void QCamera2HardwareInterface::preview_stream_cb_routine(mm_camera_super_buf_t *super_frame,
                                                          QCameraStream * stream,
                                                          void *userdata)
{
    KPI_ATRACE_CALL();
    CDBG_HIGH("[KPI Perf] %s : BEGIN", __func__);
    int err = NO_ERROR;
    QCamera2HardwareInterface *pme = (QCamera2HardwareInterface *)userdata;
    QCameraGrallocMemory *memory = (QCameraGrallocMemory *)super_frame->bufs[0]->mem_info;
    uint8_t dequeueCnt = 0;

    if (pme == NULL) {
        ALOGE("%s: Invalid hardware object", __func__);
        free(super_frame);
        return;
    }
    if (memory == NULL) {
        ALOGE("%s: Invalid memory object", __func__);
        free(super_frame);
        return;
    }

    mm_camera_buf_def_t *frame = super_frame->bufs[0];
    if (NULL == frame) {
        ALOGE("%s: preview frame is NLUL", __func__);
        free(super_frame);
        return;
    }

    // For instant capture and for instant AEC, keep track of the frame counter.
    // This count will be used to check against the corresponding bound values.
    if (pme->mParameters.isInstantAECEnabled() ||
            pme->mParameters.isInstantCaptureEnabled()) {
        pme->mInstantAecFrameCount++;
    }

    if (!pme->needProcessPreviewFrame()) {
        ALOGE("%s: preview is not running, no need to process", __func__);
        stream->bufDone(frame->buf_idx);
        free(super_frame);
        return;
    }

    if (pme->needDebugFps()) {
        pme->debugShowPreviewFPS();
    }

    uint32_t idx = frame->buf_idx;

    pme->dumpFrameToFile(stream, frame, QCAMERA_DUMP_FRM_PREVIEW);

    if(pme->m_bPreviewStarted) {
       ALOGI("[KPI Perf] %s : PROFILE_FIRST_PREVIEW_FRAME", __func__);
       pme->m_bPreviewStarted = false ;
    }

    pthread_mutex_lock(&pme->mGrallocLock);
    dequeueCnt = pme->mEnqueuedBuffers;
    pthread_mutex_unlock(&pme->mGrallocLock);

    // Display the buffer.
    CDBG("%p displayBuffer %d E", pme, idx);
    uint8_t numMapped = memory->getMappable();

    for (uint8_t i = 0; i < dequeueCnt; i++) {
        int dequeuedIdx = memory->dequeueBuffer();
        if (dequeuedIdx < 0 || dequeuedIdx >= memory->getCnt()) {
            CDBG_HIGH("%s: Invalid dequeued buffer index %d from display",
                  __func__, dequeuedIdx);
            break;
        } else {
            pthread_mutex_lock(&pme->mGrallocLock);
            pme->mEnqueuedBuffers--;
            pthread_mutex_unlock(&pme->mGrallocLock);
            if (dequeuedIdx >= numMapped) {
                // This buffer has not yet been mapped to the backend
                err = stream->mapNewBuffer((uint32_t)dequeuedIdx);
            }
        }

        if (err < 0) {
            ALOGE("buffer mapping failed %d", err);
        } else {
            // Return dequeued buffer back to driver
            err = stream->bufDone((uint32_t)dequeuedIdx);
            if ( err < 0) {
                ALOGE("stream bufDone failed %d", err);
            }
        }
    }

    // Handle preview data callback
    if (pme->m_channels[QCAMERA_CH_TYPE_CALLBACK] == NULL) {
        if (pme->needSendPreviewCallback() &&
                (!pme->mParameters.isSceneSelectionEnabled())) {
            int32_t rc = pme->sendPreviewCallback(stream, memory, idx);
            if (NO_ERROR != rc) {
                ALOGE("%s: Preview callback was not sent succesfully", __func__);
            }
        }
    }

    free(super_frame);
    CDBG_HIGH("[KPI Perf] %s : END", __func__);
    return;
}

/*===========================================================================
 * FUNCTION   : sendPreviewCallback
 *
 * DESCRIPTION: helper function for triggering preview callbacks
 *
 * PARAMETERS :
 *   @stream    : stream object
 *   @memory    : Stream memory allocator
 *   @idx       : buffer index
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::sendPreviewCallback(QCameraStream *stream,
        QCameraMemory *memory, uint32_t idx)
{
    camera_memory_t *previewMem = NULL;
    camera_memory_t *data = NULL;
    camera_memory_t *dataToApp = NULL;
    size_t previewBufSize = 0;
    size_t previewBufSizeFromCallback = 0;
    cam_dimension_t preview_dim;
    cam_format_t previewFmt;
    int32_t rc = NO_ERROR;
    int32_t yStride = 0;
    int32_t yScanline = 0;
    int32_t uvStride = 0;
    int32_t uvScanline = 0;
    int32_t uStride = 0;
    int32_t uScanline = 0;
    int32_t vStride = 0;
    int32_t vScanline = 0;
    int32_t yStrideToApp = 0;
    int32_t uvStrideToApp = 0;
    int32_t yScanlineToApp = 0;
    int32_t uvScanlineToApp = 0;
    int32_t srcOffset = 0;
    int32_t dstOffset = 0;
    int32_t srcBaseOffset = 0;
    int32_t dstBaseOffset = 0;
    int i;

    if ((NULL == stream) || (NULL == memory)) {
        ALOGE("%s: Invalid preview callback input", __func__);
        return BAD_VALUE;
    }

    cam_stream_info_t *streamInfo =
            reinterpret_cast<cam_stream_info_t *>(stream->getStreamInfoBuf()->getPtr(0));
    if (NULL == streamInfo) {
        ALOGE("%s: Invalid streamInfo", __func__);
        return BAD_VALUE;
    }

    stream->getFrameDimension(preview_dim);
    stream->getFormat(previewFmt);

    /* The preview buffer size in the callback should be
     * (width*height*bytes_per_pixel). As all preview formats we support,
     * use 12 bits per pixel, buffer size = previewWidth * previewHeight * 3/2.
     * We need to put a check if some other formats are supported in future. */
    if ((previewFmt == CAM_FORMAT_YUV_420_NV21) ||
        (previewFmt == CAM_FORMAT_YUV_420_NV12) ||
        (previewFmt == CAM_FORMAT_YUV_420_YV12) ||
        (previewFmt == CAM_FORMAT_YUV_420_NV12_VENUS) ||
        (previewFmt == CAM_FORMAT_YUV_420_NV21_VENUS) ||
        ((isMonoCamera()) && (previewFmt == CAM_FORMAT_Y_ONLY))) {
        if(previewFmt == CAM_FORMAT_YUV_420_YV12) {
            yStride = streamInfo->buf_planes.plane_info.mp[0].stride;
            yScanline = streamInfo->buf_planes.plane_info.mp[0].scanline;
            uStride = streamInfo->buf_planes.plane_info.mp[1].stride;
            uScanline = streamInfo->buf_planes.plane_info.mp[1].scanline;
            vStride = streamInfo->buf_planes.plane_info.mp[2].stride;
            vScanline = streamInfo->buf_planes.plane_info.mp[2].scanline;

            previewBufSize = (size_t)
                    (yStride * yScanline + uStride * uScanline + vStride * vScanline);
            previewBufSizeFromCallback = previewBufSize;
        } else {
            yStride = streamInfo->buf_planes.plane_info.mp[0].stride;
            yScanline = streamInfo->buf_planes.plane_info.mp[0].scanline;
            uvStride = streamInfo->buf_planes.plane_info.mp[1].stride;
            uvScanline = streamInfo->buf_planes.plane_info.mp[1].scanline;

            yStrideToApp = preview_dim.width;
            yScanlineToApp = preview_dim.height;
            uvStrideToApp = yStrideToApp;
            uvScanlineToApp = yScanlineToApp / 2;

            previewBufSize = (size_t)
                    ((yStrideToApp * yScanlineToApp) + (uvStrideToApp * uvScanlineToApp));

            previewBufSizeFromCallback = (size_t)
                    ((yStride * yScanline) + (uvStride * uvScanline));
        }
        if(previewBufSize == previewBufSizeFromCallback) {
            previewMem = mGetMemory(memory->getFd(idx),
                       previewBufSize, 1, mCallbackCookie);
            if (!previewMem || !previewMem->data) {
                ALOGE("%s: mGetMemory failed.\n", __func__);
                return NO_MEMORY;
            } else {
                data = previewMem;
            }
        } else {
            data = memory->getMemory(idx, false);
            dataToApp = mGetMemory(-1, previewBufSize, 1, mCallbackCookie);
            if (!dataToApp || !dataToApp->data) {
                ALOGE("%s: mGetMemory failed.\n", __func__);
                return NO_MEMORY;
            }

            for (i = 0; i < preview_dim.height; i++) {
                srcOffset = i * yStride;
                dstOffset = i * yStrideToApp;

                memcpy((unsigned char *) dataToApp->data + dstOffset,
                        (unsigned char *) data->data + srcOffset,
                        (size_t)yStrideToApp);
            }

            srcBaseOffset = yStride * yScanline;
            dstBaseOffset = yStrideToApp * yScanlineToApp;

            for (i = 0; i < preview_dim.height/2; i++) {
                srcOffset = i * uvStride + srcBaseOffset;
                dstOffset = i * uvStrideToApp + dstBaseOffset;

                memcpy((unsigned char *) dataToApp->data + dstOffset,
                        (unsigned char *) data->data + srcOffset,
                        (size_t)yStrideToApp);
            }
        }
    } else {
        data = memory->getMemory(idx, false);
        ALOGE("%s: Invalid preview format, buffer size in preview callback may be wrong.",
                __func__);
    }
    qcamera_callback_argm_t cbArg;
    memset(&cbArg, 0, sizeof(qcamera_callback_argm_t));
    cbArg.cb_type = QCAMERA_DATA_CALLBACK;
    cbArg.msg_type = CAMERA_MSG_PREVIEW_FRAME;
    if (previewBufSize != 0 && previewBufSizeFromCallback != 0 &&
            previewBufSize == previewBufSizeFromCallback) {
        cbArg.data = data;
    } else {
        cbArg.data = dataToApp;
    }
    if ( previewMem ) {
        cbArg.user_data = previewMem;
        cbArg.release_cb = releaseCameraMemory;
    } else if (dataToApp) {
        cbArg.user_data = dataToApp;
        cbArg.release_cb = releaseCameraMemory;
    }
    cbArg.cookie = this;
    rc = m_cbNotifier.notifyCallback(cbArg);
    if (rc != NO_ERROR) {
        ALOGE("%s: fail sending notification", __func__);
        if (previewMem) {
            previewMem->release(previewMem);
        } else if (dataToApp) {
            dataToApp->release(dataToApp);
        }
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : nodisplay_preview_stream_cb_routine
 *
 * DESCRIPTION: helper function to handle preview frame from preview stream in
 *              no-display case
 *
 * PARAMETERS :
 *   @super_frame : received super buffer
 *   @stream      : stream object
 *   @userdata    : user data ptr
 *
 * RETURN    : None
 *
 * NOTE      : caller passes the ownership of super_frame, it's our
 *             responsibility to free super_frame once it's done.
 *==========================================================================*/
void QCamera2HardwareInterface::nodisplay_preview_stream_cb_routine(
                                                          mm_camera_super_buf_t *super_frame,
                                                          QCameraStream *stream,
                                                          void * userdata)
{
    ATRACE_CALL();
    CDBG_HIGH("[KPI Perf] %s E",__func__);
    QCamera2HardwareInterface *pme = (QCamera2HardwareInterface *)userdata;
    if (pme == NULL ||
        pme->mCameraHandle == NULL ||
        pme->mCameraHandle->camera_handle != super_frame->camera_handle){
        ALOGE("%s: camera obj not valid", __func__);
        // simply free super frame
        free(super_frame);
        return;
    }
    mm_camera_buf_def_t *frame = super_frame->bufs[0];
    if (NULL == frame) {
        ALOGE("%s: preview frame is NULL", __func__);
        free(super_frame);
        return;
    }

    if (!pme->needProcessPreviewFrame()) {
        CDBG_HIGH("%s: preview is not running, no need to process", __func__);
        stream->bufDone(frame->buf_idx);
        free(super_frame);
        return;
    }

    if (pme->needDebugFps()) {
        pme->debugShowPreviewFPS();
    }

    QCameraMemory *previewMemObj = (QCameraMemory *)frame->mem_info;
    camera_memory_t *preview_mem = NULL;
    if (previewMemObj != NULL) {
        preview_mem = previewMemObj->getMemory(frame->buf_idx, false);
    }
    if (NULL != previewMemObj && NULL != preview_mem) {
        pme->dumpFrameToFile(stream, frame, QCAMERA_DUMP_FRM_PREVIEW);

        if ((pme->needProcessPreviewFrame()) &&
                pme->needSendPreviewCallback() &&
                (pme->getRelatedCamSyncInfo()->mode != CAM_MODE_SECONDARY)) {
            qcamera_callback_argm_t cbArg;
            memset(&cbArg, 0, sizeof(qcamera_callback_argm_t));
            cbArg.cb_type = QCAMERA_DATA_CALLBACK;
            cbArg.msg_type = CAMERA_MSG_PREVIEW_FRAME;
            cbArg.data = preview_mem;
            cbArg.user_data = (void *) &frame->buf_idx;
            cbArg.cookie = stream;
            cbArg.release_cb = returnStreamBuffer;
            int32_t rc = pme->m_cbNotifier.notifyCallback(cbArg);
            if (rc != NO_ERROR) {
                ALOGE("%s: fail sending data notify", __func__);
                stream->bufDone(frame->buf_idx);
            }
        } else {
            stream->bufDone(frame->buf_idx);
        }
    }
    free(super_frame);
    CDBG_HIGH("[KPI Perf] %s X",__func__);
}

/*===========================================================================
 * FUNCTION   : rdi_mode_stream_cb_routine
 *
 * DESCRIPTION: helper function to handle RDI frame from preview stream in
 *              rdi mode case
 *
 * PARAMETERS :
 *   @super_frame : received super buffer
 *   @stream      : stream object
 *   @userdata    : user data ptr
 *
 * RETURN    : None
 *
 * NOTE      : caller passes the ownership of super_frame, it's our
 *             responsibility to free super_frame once it's done.
 *==========================================================================*/
void QCamera2HardwareInterface::rdi_mode_stream_cb_routine(
  mm_camera_super_buf_t *super_frame,
  QCameraStream *stream,
  void * userdata)
{
    ATRACE_CALL();
    CDBG_HIGH("RDI_DEBUG %s[%d]: Enter", __func__, __LINE__);
    QCamera2HardwareInterface *pme = (QCamera2HardwareInterface *)userdata;
    if (pme == NULL ||
        pme->mCameraHandle == NULL ||
        pme->mCameraHandle->camera_handle != super_frame->camera_handle){
        ALOGE("%s: camera obj not valid", __func__);
        free(super_frame);
        return;
    }
    mm_camera_buf_def_t *frame = super_frame->bufs[0];
    if (NULL == frame) {
        ALOGE("%s: preview frame is NLUL", __func__);
        goto end;
    }
    if (!pme->needProcessPreviewFrame()) {
        ALOGE("%s: preview is not running, no need to process", __func__);
        stream->bufDone(frame->buf_idx);
        goto end;
    }
    if (pme->needDebugFps()) {
        pme->debugShowPreviewFPS();
    }
    // Non-secure Mode
    if (!pme->isSecureMode()) {
        QCameraMemory *previewMemObj = (QCameraMemory *)frame->mem_info;
        if (NULL == previewMemObj) {
            ALOGE("%s: previewMemObj is NULL", __func__);
            stream->bufDone(frame->buf_idx);
            goto end;
        }

        camera_memory_t *preview_mem = previewMemObj->getMemory(frame->buf_idx, false);
        if (NULL != preview_mem) {
            previewMemObj->cleanCache(frame->buf_idx);
            // Dump RAW frame
            pme->dumpFrameToFile(stream, frame, QCAMERA_DUMP_FRM_RAW);
            // Notify Preview callback frame
            if (pme->needProcessPreviewFrame() &&
                    pme->mDataCb != NULL &&
                    pme->msgTypeEnabledWithLock(CAMERA_MSG_PREVIEW_FRAME) > 0) {
                qcamera_callback_argm_t cbArg;
                memset(&cbArg, 0, sizeof(qcamera_callback_argm_t));
                cbArg.cb_type    = QCAMERA_DATA_CALLBACK;
                cbArg.msg_type   = CAMERA_MSG_PREVIEW_FRAME;
                cbArg.data       = preview_mem;
                cbArg.user_data = (void *) &frame->buf_idx;
                cbArg.cookie     = stream;
                cbArg.release_cb = returnStreamBuffer;
                pme->m_cbNotifier.notifyCallback(cbArg);
            } else {
                ALOGE("%s: preview_mem is NULL", __func__);
                stream->bufDone(frame->buf_idx);
            }
        }
        else {
            ALOGE("%s: preview_mem is NULL", __func__);
            stream->bufDone(frame->buf_idx);
        }
    } else {
        // Secure Mode
        // We will do QCAMERA_NOTIFY_CALLBACK and share FD in case of secure mode
        QCameraMemory *previewMemObj = (QCameraMemory *)frame->mem_info;
        if (NULL == previewMemObj) {
            ALOGE("%s: previewMemObj is NULL", __func__);
            stream->bufDone(frame->buf_idx);
            goto end;
        }

        int fd = previewMemObj->getFd(frame->buf_idx);
        ALOGD("%s: Preview frame fd =%d for index = %d ", __func__, fd, frame->buf_idx);
        if (pme->needProcessPreviewFrame() &&
                pme->mDataCb != NULL &&
                pme->msgTypeEnabledWithLock(CAMERA_MSG_PREVIEW_FRAME) > 0) {
            // Prepare Callback structure
            qcamera_callback_argm_t cbArg;
            memset(&cbArg, 0, sizeof(qcamera_callback_argm_t));
            cbArg.cb_type    = QCAMERA_NOTIFY_CALLBACK;
            cbArg.msg_type   = CAMERA_MSG_PREVIEW_FRAME;
#ifndef VANILLA_HAL
            cbArg.ext1       = CAMERA_FRAME_DATA_FD;
            cbArg.ext2       = fd;
#endif
            cbArg.user_data  = (void *) &frame->buf_idx;
            cbArg.cookie     = stream;
            cbArg.release_cb = returnStreamBuffer;
            pme->m_cbNotifier.notifyCallback(cbArg);
        } else {
            CDBG_HIGH("%s: No need to process preview frame, return buffer", __func__);
            stream->bufDone(frame->buf_idx);
        }
    }
end:
    free(super_frame);
    CDBG_HIGH("RDI_DEBUG %s[%d]: Exit", __func__, __LINE__);
    return;
}

/*===========================================================================
 * FUNCTION   : postview_stream_cb_routine
 *
 * DESCRIPTION: helper function to handle post frame from postview stream
 *
 * PARAMETERS :
 *   @super_frame : received super buffer
 *   @stream      : stream object
 *   @userdata    : user data ptr
 *
 * RETURN    : None
 *
 * NOTE      : caller passes the ownership of super_frame, it's our
 *             responsibility to free super_frame once it's done.
 *==========================================================================*/
void QCamera2HardwareInterface::postview_stream_cb_routine(mm_camera_super_buf_t *super_frame,
                                                           QCameraStream *stream,
                                                           void *userdata)
{
    ATRACE_CALL();
    int err = NO_ERROR;
    QCamera2HardwareInterface *pme = (QCamera2HardwareInterface *)userdata;
    QCameraGrallocMemory *memory = (QCameraGrallocMemory *)super_frame->bufs[0]->mem_info;

    if (pme == NULL) {
        ALOGE("%s: Invalid hardware object", __func__);
        free(super_frame);
        return;
    }
    if (memory == NULL) {
        ALOGE("%s: Invalid memory object", __func__);
        free(super_frame);
        return;
    }

    CDBG_HIGH("[KPI Perf] %s : BEGIN", __func__);

    mm_camera_buf_def_t *frame = super_frame->bufs[0];
    if (NULL == frame) {
        ALOGE("%s: preview frame is NULL", __func__);
        free(super_frame);
        return;
    }

    QCameraMemory *memObj = (QCameraMemory *)frame->mem_info;
    if (NULL != memObj) {
        pme->dumpFrameToFile(stream, frame, QCAMERA_DUMP_FRM_THUMBNAIL);
    }

    // Return buffer back to driver
    err = stream->bufDone(frame->buf_idx);
    if ( err < 0) {
        ALOGE("stream bufDone failed %d", err);
    }

    free(super_frame);
    CDBG_HIGH("[KPI Perf] %s : END", __func__);
    return;
}

/*===========================================================================
 * FUNCTION   : video_stream_cb_routine
 *
 * DESCRIPTION: helper function to handle video frame from video stream
 *
 * PARAMETERS :
 *   @super_frame : received super buffer
 *   @stream      : stream object
 *   @userdata    : user data ptr
 *
 * RETURN    : None
 *
 * NOTE      : caller passes the ownership of super_frame, it's our
 *             responsibility to free super_frame once it's done. video
 *             frame will be sent to video encoder. Once video encoder is
 *             done with the video frame, it will call another API
 *             (release_recording_frame) to return the frame back
 *==========================================================================*/
void QCamera2HardwareInterface::video_stream_cb_routine(mm_camera_super_buf_t *super_frame,
                                                        QCameraStream *stream,
                                                        void *userdata)
{
    ATRACE_CALL();
#ifdef USE_MEDIA_EXTENSIONS
    QCameraVideoMemory *videoMemObj = NULL;
#else
    QCameraMemory *videoMemObj = NULL;
#endif

    CDBG("[KPI Perf] %s : BEGIN", __func__);
    QCamera2HardwareInterface *pme = (QCamera2HardwareInterface *)userdata;
    if (pme == NULL ||
        pme->mCameraHandle == NULL ||
        pme->mCameraHandle->camera_handle != super_frame->camera_handle){
        ALOGE("%s: camera obj not valid", __func__);
        // simply free super frame
        free(super_frame);
        return;
    }

    mm_camera_buf_def_t *frame = super_frame->bufs[0];

    if (pme->needDebugFps()) {
        pme->debugShowVideoFPS();
    }
    if(pme->m_bRecordStarted) {
       ALOGI("[KPI Perf] %s : PROFILE_FIRST_RECORD_FRAME", __func__);
       pme->m_bRecordStarted = false ;
    }
    CDBG("%s: Stream(%d), Timestamp: %ld %ld",
          __func__,
          frame->stream_id,
          frame->ts.tv_sec,
          frame->ts.tv_nsec);

    if (frame->buf_type == CAM_STREAM_BUF_TYPE_MPLANE) {
        nsecs_t timeStamp;
        timeStamp = nsecs_t(frame->ts.tv_sec) * 1000000000LL + frame->ts.tv_nsec;
#ifdef USE_MEDIA_EXTENSIONS
        // For VT usecase, ISP uses AVtimer not CLOCK_BOOTTIME as time source.
        // So do not change video timestamp.
        if (!pme->mParameters.isAVTimerEnabled()) {
            // Convert Boottime from camera to Monotime for video if needed.
            // Otherwise, mBootToMonoTimestampOffset value will be 0.
            timeStamp = timeStamp - pme->mBootToMonoTimestampOffset;
        }
        CDBG("Send Video frame to services/encoder TimeStamp : %lld",
            timeStamp);
        videoMemObj = (QCameraVideoMemory *)frame->mem_info;
#else
        videoMemObj = (QCameraMemory *)frame->mem_info;
#endif
        camera_memory_t *video_mem = NULL;
        if (NULL != videoMemObj) {
            video_mem = videoMemObj->getMemory(frame->buf_idx,
                    (pme->mStoreMetaDataInFrame > 0)? true : false);
            CDBG("Video frame TimeStamp : %lld batch = 0 index=%d",timeStamp,frame->buf_idx);
        }
        if (NULL != videoMemObj && NULL != video_mem) {
            pme->dumpFrameToFile(stream, frame, QCAMERA_DUMP_FRM_VIDEO);
            if ((pme->mDataCbTimestamp != NULL) &&
                pme->msgTypeEnabledWithLock(CAMERA_MSG_VIDEO_FRAME) > 0) {
                qcamera_callback_argm_t cbArg;
                memset(&cbArg, 0, sizeof(qcamera_callback_argm_t));
                cbArg.cb_type = QCAMERA_DATA_TIMESTAMP_CALLBACK;
                cbArg.msg_type = CAMERA_MSG_VIDEO_FRAME;
                cbArg.data = video_mem;
                cbArg.timestamp = timeStamp;
                int32_t rc = pme->m_cbNotifier.notifyCallback(cbArg);
                if (rc != NO_ERROR) {
                    ALOGE("%s: fail sending data notify", __func__);
                    stream->bufDone(frame->buf_idx);
                }
            }
        }
    } else {
#ifdef USE_MEDIA_EXTENSIONS
        videoMemObj = (QCameraVideoMemory *)frame->mem_info;
#else
        videoMemObj = (QCameraMemory *)frame->mem_info;
#endif
        camera_memory_t *video_mem = NULL;
        native_handle_t *nh = NULL;
        int fd_cnt = frame->user_buf.bufs_used;
        if (NULL != videoMemObj) {
            video_mem = videoMemObj->getMemory(frame->buf_idx, true);
#ifdef USE_MEDIA_EXTENSIONS
            nh = videoMemObj->getNativeHandle(frame->buf_idx);
#else
            if (video_mem != NULL) {
                struct encoder_media_buffer_type * packet =
                        (struct encoder_media_buffer_type *)video_mem->data;
                // fd cnt => Number of buffer FD's and buffer for offset, size, timestamp
                packet->meta_handle = native_handle_create(
                        fd_cnt, (VIDEO_METADATA_NUM_INTS * fd_cnt));
                packet->buffer_type = kMetadataBufferTypeCameraSource;
                nh = const_cast<native_handle_t *>(packet->meta_handle);
            } else {
                ALOGE("%s video_mem NULL", __func__);
            }
#endif
        } else {
            ALOGE("%s videoMemObj NULL", __func__);
        }

        if (nh != NULL) {
            nsecs_t timeStamp;
            timeStamp = nsecs_t(frame->ts.tv_sec) * 1000000000LL
                    + frame->ts.tv_nsec;

            for (int i = 0; i < fd_cnt; i++) {
                if (frame->user_buf.buf_idx[i] >= 0) {
                    mm_camera_buf_def_t *plane_frame =
                            &frame->user_buf.plane_buf[frame->user_buf.buf_idx[i]];
                    QCameraVideoMemory *frameobj = (QCameraVideoMemory *)plane_frame->mem_info;
                    nsecs_t frame_ts = nsecs_t(plane_frame->ts.tv_sec) * 1000000000LL
                            + plane_frame->ts.tv_nsec;
                    /*data[0] => FD data[1] => OFFSET data[2] => SIZE data[3] => USAGE
                    data[4] => TIMESTAMP data[5] => FORMAT*/
                    nh->data[i] = frameobj->getFd(plane_frame->buf_idx);
                    nh->data[fd_cnt + i] = 0;
                    nh->data[(2 * fd_cnt) + i] = (int)frameobj->getSize(plane_frame->buf_idx);
                    nh->data[(3 * fd_cnt) + i] = frameobj->getUsage();
                    nh->data[(4 * fd_cnt) + i] = (int)(frame_ts - timeStamp);
                    nh->data[(5 * fd_cnt) + i] = frameobj->getFormat();
                    CDBG("Send Video frames to services/encoder delta : %lld FD = %d index = %d",
                            (frame_ts - timeStamp), plane_frame->fd, plane_frame->buf_idx);
                    pme->dumpFrameToFile(stream, plane_frame, QCAMERA_DUMP_FRM_VIDEO);
                }
            }
            CDBG("Batch buffer TimeStamp : %lld FD = %d index = %d fd_cnt = %d",
                    timeStamp, frame->fd, frame->buf_idx, fd_cnt);
            if ((pme->mDataCbTimestamp != NULL) &&
                        pme->msgTypeEnabledWithLock(CAMERA_MSG_VIDEO_FRAME) > 0) {
                qcamera_callback_argm_t cbArg;
                memset(&cbArg, 0, sizeof(qcamera_callback_argm_t));
                cbArg.cb_type = QCAMERA_DATA_TIMESTAMP_CALLBACK;
                cbArg.msg_type = CAMERA_MSG_VIDEO_FRAME;
                cbArg.data = video_mem;

                // For VT usecase, ISP uses AVtimer not CLOCK_BOOTTIME as time source.
                // So do not change video timestamp.
                if (!pme->mParameters.isAVTimerEnabled()) {
                    // Convert Boottime from camera to Monotime for video if needed.
                    // Otherwise, mBootToMonoTimestampOffset value will be 0.
                    timeStamp = timeStamp - pme->mBootToMonoTimestampOffset;
                }
                CDBG("Final video buffer TimeStamp : %lld ", timeStamp);
                cbArg.timestamp = timeStamp;
                int32_t rc = pme->m_cbNotifier.notifyCallback(cbArg);
                if (rc != NO_ERROR) {
                    ALOGE("%s: fail sending data notify", __func__);
                    stream->bufDone(frame->buf_idx);
                }
            }
        } else {
            ALOGE("%s: No Video Meta Available. Return Buffer", __func__);
            stream->bufDone(super_frame->bufs[0]->buf_idx);
        }
    }
    free(super_frame);
    CDBG("[KPI Perf] %s : END", __func__);
}

/*===========================================================================
 * FUNCTION   : snapshot_channel_cb_routine
 *
 * DESCRIPTION: helper function to handle snapshot frame from snapshot channel
 *
 * PARAMETERS :
 *   @super_frame : received super buffer
 *   @userdata    : user data ptr
 *
 * RETURN    : None
 *
 * NOTE      : recvd_frame will be released after this call by caller, so if
 *             async operation needed for recvd_frame, it's our responsibility
 *             to save a copy for this variable to be used later.
 *==========================================================================*/
void QCamera2HardwareInterface::snapshot_channel_cb_routine(mm_camera_super_buf_t *super_frame,
       void *userdata)
{
    ATRACE_CALL();
    char value[PROPERTY_VALUE_MAX];
    QCameraChannel *pChannel = NULL;

    CDBG_HIGH("[KPI Perf] %s: E", __func__);
    QCamera2HardwareInterface *pme = (QCamera2HardwareInterface *)userdata;
    if (pme == NULL ||
        pme->mCameraHandle == NULL ||
        pme->mCameraHandle->camera_handle != super_frame->camera_handle){
        ALOGE("%s: camera obj not valid", __func__);
        // simply free super frame
        free(super_frame);
        return;
    }

    if (pme->isLowPowerMode()) {
        pChannel = pme->m_channels[QCAMERA_CH_TYPE_VIDEO];
    } else {
        pChannel = pme->m_channels[QCAMERA_CH_TYPE_SNAPSHOT];
    }

    if ((pChannel == NULL) || (pChannel->getMyHandle() != super_frame->ch_id)) {
        ALOGE("%s: Snapshot channel doesn't exist, return here", __func__);
        return;
    }

    property_get("persist.camera.dumpmetadata", value, "0");
    int32_t enabled = atoi(value);
    if (enabled) {
        if (pChannel == NULL ||
            pChannel->getMyHandle() != super_frame->ch_id) {
            ALOGE("%s: Capture channel doesn't exist, return here", __func__);
            return;
        }
        mm_camera_buf_def_t *pMetaFrame = NULL;
        QCameraStream *pStream = NULL;
        for (uint32_t i = 0; i < super_frame->num_bufs; i++) {
            pStream = pChannel->getStreamByHandle(super_frame->bufs[i]->stream_id);
            if (pStream != NULL) {
                if (pStream->isTypeOf(CAM_STREAM_TYPE_METADATA)) {
                    pMetaFrame = super_frame->bufs[i]; //find the metadata
                    if (pMetaFrame != NULL &&
                            ((metadata_buffer_t *)pMetaFrame->buffer)->is_tuning_params_valid) {
                        pme->dumpMetadataToFile(pStream, pMetaFrame, (char *) "Snapshot");
                    }
                    break;
                }
            }
        }
    }

    // save a copy for the superbuf
    mm_camera_super_buf_t* frame = (mm_camera_super_buf_t *)malloc(sizeof(mm_camera_super_buf_t));
    if (frame == NULL) {
        ALOGE("%s: Error allocating memory to save received_frame structure.",
                __func__);
        pChannel->bufDone(super_frame);
        return;
    }
    *frame = *super_frame;

    pme->m_postprocessor.processData(frame);

    CDBG_HIGH("[KPI Perf] %s: X", __func__);
}

/*===========================================================================
 * FUNCTION   : raw_stream_cb_routine
 *
 * DESCRIPTION: helper function to handle raw dump frame from raw stream
 *
 * PARAMETERS :
 *   @super_frame : received super buffer
 *   @stream      : stream object
 *   @userdata    : user data ptr
 *
 * RETURN    : None
 *
 * NOTE      : caller passes the ownership of super_frame, it's our
 *             responsibility to free super_frame once it's done. For raw
 *             frame, there is no need to send to postprocessor for jpeg
 *             encoding. this function will play shutter and send the data
 *             callback to upper layer. Raw frame buffer will be returned
 *             back to kernel, and frame will be free after use.
 *==========================================================================*/
void QCamera2HardwareInterface::raw_stream_cb_routine(mm_camera_super_buf_t * super_frame,
                                                      QCameraStream * /*stream*/,
                                                      void * userdata)
{
    ATRACE_CALL();
    CDBG_HIGH("[KPI Perf] %s : BEGIN", __func__);
    QCamera2HardwareInterface *pme = (QCamera2HardwareInterface *)userdata;
    if (pme == NULL ||
        pme->mCameraHandle == NULL ||
        pme->mCameraHandle->camera_handle != super_frame->camera_handle){
        ALOGE("%s: camera obj not valid", __func__);
        // simply free super frame
        free(super_frame);
        return;
    }

    pme->m_postprocessor.processRawData(super_frame);
    CDBG_HIGH("[KPI Perf] %s : END", __func__);
}

/*===========================================================================
 * FUNCTION   : preview_raw_stream_cb_routine
 *
 * DESCRIPTION: helper function to handle raw frame during standard preview
 *
 * PARAMETERS :
 *   @super_frame : received super buffer
 *   @stream      : stream object
 *   @userdata    : user data ptr
 *
 * RETURN    : None
 *
 * NOTE      : caller passes the ownership of super_frame, it's our
 *             responsibility to free super_frame once it's done.
 *==========================================================================*/
void QCamera2HardwareInterface::preview_raw_stream_cb_routine(mm_camera_super_buf_t * super_frame,
                                                              QCameraStream * stream,
                                                              void * userdata)
{
    ATRACE_CALL();
    CDBG_HIGH("[KPI Perf] %s : BEGIN", __func__);
    char value[PROPERTY_VALUE_MAX];
    bool dump_preview_raw = false, dump_video_raw = false;

    QCamera2HardwareInterface *pme = (QCamera2HardwareInterface *)userdata;
    if (pme == NULL ||
        pme->mCameraHandle == NULL ||
        pme->mCameraHandle->camera_handle != super_frame->camera_handle){
        ALOGE("%s: camera obj not valid", __func__);
        // simply free super frame
        free(super_frame);
        return;
    }

    mm_camera_buf_def_t *raw_frame = super_frame->bufs[0];

    if (raw_frame != NULL) {
        property_get("persist.camera.preview_raw", value, "0");
        dump_preview_raw = atoi(value) > 0 ? true : false;
        property_get("persist.camera.video_raw", value, "0");
        dump_video_raw = atoi(value) > 0 ? true : false;
        if (dump_preview_raw || (pme->mParameters.getRecordingHintValue()
                && dump_video_raw)) {
            pme->dumpFrameToFile(stream, raw_frame, QCAMERA_DUMP_FRM_RAW);
        }
        stream->bufDone(raw_frame->buf_idx);
    }
    free(super_frame);

    CDBG_HIGH("[KPI Perf] %s : END", __func__);
}

/*===========================================================================
 * FUNCTION   : snapshot_raw_stream_cb_routine
 *
 * DESCRIPTION: helper function to handle raw frame during standard capture
 *
 * PARAMETERS :
 *   @super_frame : received super buffer
 *   @stream      : stream object
 *   @userdata    : user data ptr
 *
 * RETURN    : None
 *
 * NOTE      : caller passes the ownership of super_frame, it's our
 *             responsibility to free super_frame once it's done.
 *==========================================================================*/
void QCamera2HardwareInterface::snapshot_raw_stream_cb_routine(mm_camera_super_buf_t * super_frame,
                                                               QCameraStream * stream,
                                                               void * userdata)
{
    ATRACE_CALL();
    CDBG_HIGH("[KPI Perf] %s : BEGIN", __func__);
    char value[PROPERTY_VALUE_MAX];
    bool dump_raw = false;

    QCamera2HardwareInterface *pme = (QCamera2HardwareInterface *)userdata;
    if (pme == NULL ||
        pme->mCameraHandle == NULL ||
        pme->mCameraHandle->camera_handle != super_frame->camera_handle){
        ALOGE("%s: camera obj not valid", __func__);
        // simply free super frame
        free(super_frame);
        return;
    }

    property_get("persist.camera.snapshot_raw", value, "0");
    dump_raw = atoi(value) > 0 ? true : false;

    for (uint32_t i = 0; i < super_frame->num_bufs; i++) {
        if (super_frame->bufs[i]->stream_type == CAM_STREAM_TYPE_RAW) {
            mm_camera_buf_def_t * raw_frame = super_frame->bufs[i];
            if (NULL != stream) {
                if (dump_raw) {
                    pme->dumpFrameToFile(stream, raw_frame, QCAMERA_DUMP_FRM_RAW);
                }
                stream->bufDone(super_frame->bufs[i]->buf_idx);
            }
            break;
        }
    }

    free(super_frame);

    CDBG_HIGH("[KPI Perf] %s : END", __func__);
}

/*===========================================================================
 * FUNCTION   : updateMetadata
 *
 * DESCRIPTION: Frame related parameter can be updated here
 *
 * PARAMETERS :
 *   @pMetaData : pointer to metadata buffer
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::updateMetadata(metadata_buffer_t *pMetaData)
{
    int32_t rc = NO_ERROR;

    if (pMetaData == NULL) {
        ALOGE("%s: Null Metadata buffer", __func__);
        return rc;
    }

    // Sharpness
    cam_edge_application_t edge_application;
    memset(&edge_application, 0x00, sizeof(cam_edge_application_t));
    edge_application.sharpness = mParameters.getSharpness();
    if (edge_application.sharpness != 0) {
        edge_application.edge_mode = CAM_EDGE_MODE_FAST;
    } else {
        edge_application.edge_mode = CAM_EDGE_MODE_OFF;
    }
    ADD_SET_PARAM_ENTRY_TO_BATCH(pMetaData,
            CAM_INTF_META_EDGE_MODE, edge_application);

    //Effect
    int32_t prmEffect = mParameters.getEffect();
    ADD_SET_PARAM_ENTRY_TO_BATCH(pMetaData, CAM_INTF_PARM_EFFECT, prmEffect);

    //flip
    int32_t prmFlip = mParameters.getFlipMode(CAM_STREAM_TYPE_SNAPSHOT);
    ADD_SET_PARAM_ENTRY_TO_BATCH(pMetaData, CAM_INTF_PARM_FLIP, prmFlip);

    //denoise
    uint8_t prmDenoise = (uint8_t)mParameters.isWNREnabled();
    ADD_SET_PARAM_ENTRY_TO_BATCH(pMetaData,
            CAM_INTF_META_NOISE_REDUCTION_MODE, prmDenoise);

    //rotation & device rotation
    uint32_t prmRotation = mParameters.getJpegRotation();
    cam_rotation_info_t rotation_info;
    if (prmRotation == 0) {
       rotation_info.rotation = ROTATE_0;
    } else if (prmRotation == 90) {
       rotation_info.rotation = ROTATE_90;
    } else if (prmRotation == 180) {
       rotation_info.rotation = ROTATE_180;
    } else if (prmRotation == 270) {
       rotation_info.rotation = ROTATE_270;
    }

    uint32_t device_rotation = mParameters.getDeviceRotation();
    if (device_rotation == 0) {
        rotation_info.device_rotation = ROTATE_0;
    } else if (device_rotation == 90) {
        rotation_info.device_rotation = ROTATE_90;
    } else if (device_rotation == 180) {
        rotation_info.device_rotation = ROTATE_180;
    } else if (device_rotation == 270) {
        rotation_info.device_rotation = ROTATE_270;
    } else {
        rotation_info.device_rotation = ROTATE_0;
    }

    ADD_SET_PARAM_ENTRY_TO_BATCH(pMetaData, CAM_INTF_PARM_ROTATION, rotation_info);

    //CPP CDS
    int32_t prmCDSMode = mParameters.getCDSMode();
    ADD_SET_PARAM_ENTRY_TO_BATCH(pMetaData,
            CAM_INTF_PARM_CDS_MODE, prmCDSMode);

    return rc;
}

/*===========================================================================
 * FUNCTION   : metadata_stream_cb_routine
 *
 * DESCRIPTION: helper function to handle metadata frame from metadata stream
 *
 * PARAMETERS :
 *   @super_frame : received super buffer
 *   @stream      : stream object
 *   @userdata    : user data ptr
 *
 * RETURN    : None
 *
 * NOTE      : caller passes the ownership of super_frame, it's our
 *             responsibility to free super_frame once it's done. Metadata
 *             could have valid entries for face detection result or
 *             histogram statistics information.
 *==========================================================================*/
void QCamera2HardwareInterface::metadata_stream_cb_routine(mm_camera_super_buf_t * super_frame,
                                                           QCameraStream * stream,
                                                           void * userdata)
{
    ATRACE_CALL();
    CDBG("[KPI Perf] %s : BEGIN", __func__);
    QCamera2HardwareInterface *pme = (QCamera2HardwareInterface *)userdata;
    if (pme == NULL ||
        pme->mCameraHandle == NULL ||
        pme->mCameraHandle->camera_handle != super_frame->camera_handle){
        ALOGE("%s: camera obj not valid", __func__);
        // simply free super frame
        free(super_frame);
        return;
    }

    mm_camera_buf_def_t *frame = super_frame->bufs[0];
    metadata_buffer_t *pMetaData = (metadata_buffer_t *)frame->buffer;
    if(pme->m_stateMachine.isNonZSLCaptureRunning()&&
       !pme->mLongshotEnabled) {
       //Make shutter call back in non ZSL mode once raw frame is received from VFE.
       pme->playShutter();
    }

    if (pMetaData->is_tuning_params_valid && pme->mParameters.getRecordingHintValue() == true) {
        //Dump Tuning data for video
        pme->dumpMetadataToFile(stream,frame,(char *)"Video");
    }

    IF_META_AVAILABLE(cam_hist_stats_t, stats_data, CAM_INTF_META_HISTOGRAM, pMetaData) {
        // process histogram statistics info
        qcamera_sm_internal_evt_payload_t *payload =
            (qcamera_sm_internal_evt_payload_t *)
                malloc(sizeof(qcamera_sm_internal_evt_payload_t));
        if (NULL != payload) {
            memset(payload, 0, sizeof(qcamera_sm_internal_evt_payload_t));
            payload->evt_type = QCAMERA_INTERNAL_EVT_HISTOGRAM_STATS;
            payload->stats_data = *stats_data;
            int32_t rc = pme->processEvt(QCAMERA_SM_EVT_EVT_INTERNAL, payload);
            if (rc != NO_ERROR) {
                ALOGE("%s: processEvt histogram failed", __func__);
                free(payload);
                payload = NULL;

            }
        } else {
            ALOGE("%s: No memory for histogram qcamera_sm_internal_evt_payload_t", __func__);
        }
    }

    IF_META_AVAILABLE(cam_face_detection_data_t, faces_data,
            CAM_INTF_META_FACE_DETECTION, pMetaData) {
        if (faces_data->num_faces_detected > MAX_ROI) {
            ALOGE("%s: Invalid number of faces %d",
                __func__, faces_data->num_faces_detected);
        } else {
            // process face detection result
            if (faces_data->num_faces_detected)
                CDBG_HIGH("[KPI Perf] %s: PROFILE_NUMBER_OF_FACES_DETECTED %d",
                    __func__,faces_data->num_faces_detected);
            faces_data->fd_type = QCAMERA_FD_PREVIEW; //HARD CODE here before MCT can support
            qcamera_sm_internal_evt_payload_t *payload = (qcamera_sm_internal_evt_payload_t *)
                malloc(sizeof(qcamera_sm_internal_evt_payload_t));
            if (NULL != payload) {
                memset(payload, 0, sizeof(qcamera_sm_internal_evt_payload_t));
                payload->evt_type = QCAMERA_INTERNAL_EVT_FACE_DETECT_RESULT;
                payload->faces_data = *faces_data;
                int32_t rc = pme->processEvt(QCAMERA_SM_EVT_EVT_INTERNAL, payload);
                if (rc != NO_ERROR) {
                    ALOGE("%s: processEvt face detection failed", __func__);
                    free(payload);
                    payload = NULL;
                }
            } else {
                ALOGE("%s: No memory for face detect qcamera_sm_internal_evt_payload_t", __func__);
            }
        }
    }

    IF_META_AVAILABLE(uint32_t, afState, CAM_INTF_META_AF_STATE, pMetaData) {
        uint8_t forceAFUpdate = FALSE;
        //1. Earlier HAL used to rely on AF done flags set in metadata to generate callbacks to
        //upper layers. But in scenarios where metadata drops especially which contain important
        //AF information, APP will wait indefinitely for focus result resulting in capture hang.
        //2. HAL can check for AF state transitions to generate AF state callbacks to upper layers.
        //This will help overcome metadata drop issue with the earlier approach.
        //3. But sometimes AF state transitions can happen so fast within same metadata due to
        //which HAL will receive only the final AF state. HAL may perceive this as no change in AF
        //state depending on the state transitions happened (for example state A -> B -> A).
        //4. To overcome the drawbacks of both the approaches, we go for a hybrid model in which
        //we check state transition at both HAL level and AF module level. We rely on
        //'state transition' meta field set by AF module for the state transition detected by it.
        IF_META_AVAILABLE(uint8_t, stateChange, CAM_INTF_AF_STATE_TRANSITION, pMetaData) {
            forceAFUpdate = *stateChange;
        }
        //This is a special scenario in which when scene modes like landscape are selected, AF mode
        //gets changed to INFINITY at backend, but HAL will not be aware of it. Also, AF state in
        //such cases will be set to CAM_AF_STATE_INACTIVE by backend. So, detect the AF mode
        //change here and trigger AF callback @ processAutoFocusEvent().
        IF_META_AVAILABLE(uint32_t, afFocusMode, CAM_INTF_PARM_FOCUS_MODE, pMetaData) {
            if (((cam_focus_mode_type)(*afFocusMode) == CAM_FOCUS_MODE_INFINITY) &&
                    pme->mActiveAF){
                forceAFUpdate = TRUE;
            }
        }
        if ((pme->m_currentFocusState != (*afState)) || forceAFUpdate) {
            pme->m_currentFocusState = (cam_af_state_t)(*afState);
            qcamera_sm_internal_evt_payload_t *payload = (qcamera_sm_internal_evt_payload_t *)
                    malloc(sizeof(qcamera_sm_internal_evt_payload_t));
            if (NULL != payload) {
                memset(payload, 0, sizeof(qcamera_sm_internal_evt_payload_t));
                payload->evt_type = QCAMERA_INTERNAL_EVT_FOCUS_UPDATE;
                payload->focus_data.focus_state = (cam_af_state_t)(*afState);
                payload->focus_data.focused_frame_idx = frame->frame_idx;
                IF_META_AVAILABLE(float, focusDistance,
                        CAM_INTF_META_LENS_FOCUS_DISTANCE, pMetaData) {
                    payload->focus_data.focus_dist.
                    focus_distance[CAM_FOCUS_DISTANCE_OPTIMAL_INDEX] = *focusDistance;
                }
                IF_META_AVAILABLE(float, focusRange, CAM_INTF_META_LENS_FOCUS_RANGE, pMetaData) {
                    payload->focus_data.focus_dist.
                            focus_distance[CAM_FOCUS_DISTANCE_NEAR_INDEX] = focusRange[0];
                    payload->focus_data.focus_dist.
                            focus_distance[CAM_FOCUS_DISTANCE_FAR_INDEX] = focusRange[1];
                }
                IF_META_AVAILABLE(uint32_t, focusMode, CAM_INTF_PARM_FOCUS_MODE, pMetaData) {
                    payload->focus_data.focus_mode = (cam_focus_mode_type)(*focusMode);
                }
                int32_t rc = pme->processEvt(QCAMERA_SM_EVT_EVT_INTERNAL, payload);
                if (rc != NO_ERROR) {
                    ALOGE("%s: processEvt focus failed", __func__);
                    free(payload);
                    payload = NULL;
                }
            } else {
                ALOGE("%s: No memory for focus qcamera_sm_internal_evt_payload_t", __func__);
            }
        }
    }

    IF_META_AVAILABLE(cam_crop_data_t, crop_data, CAM_INTF_META_CROP_DATA, pMetaData) {
        if (crop_data->num_of_streams > MAX_NUM_STREAMS) {
            ALOGE("%s: Invalid num_of_streams %d in crop_data", __func__,
                crop_data->num_of_streams);
        } else {
            qcamera_sm_internal_evt_payload_t *payload =
                (qcamera_sm_internal_evt_payload_t *)
                    malloc(sizeof(qcamera_sm_internal_evt_payload_t));
            if (NULL != payload) {
                memset(payload, 0, sizeof(qcamera_sm_internal_evt_payload_t));
                payload->evt_type = QCAMERA_INTERNAL_EVT_CROP_INFO;
                payload->crop_data = *crop_data;
                int32_t rc = pme->processEvt(QCAMERA_SM_EVT_EVT_INTERNAL, payload);
                if (rc != NO_ERROR) {
                    ALOGE("%s: processEvt crop info failed", __func__);
                    free(payload);
                    payload = NULL;

                }
            } else {
                ALOGE("%s: No memory for prep_snapshot qcamera_sm_internal_evt_payload_t",
                    __func__);
            }
        }
    }

    IF_META_AVAILABLE(int32_t, prep_snapshot_done_state,
            CAM_INTF_META_PREP_SNAPSHOT_DONE, pMetaData) {
        qcamera_sm_internal_evt_payload_t *payload =
        (qcamera_sm_internal_evt_payload_t *)malloc(sizeof(qcamera_sm_internal_evt_payload_t));
        if (NULL != payload) {
            memset(payload, 0, sizeof(qcamera_sm_internal_evt_payload_t));
            payload->evt_type = QCAMERA_INTERNAL_EVT_PREP_SNAPSHOT_DONE;
            payload->prep_snapshot_state = (cam_prep_snapshot_state_t)*prep_snapshot_done_state;
            int32_t rc = pme->processEvt(QCAMERA_SM_EVT_EVT_INTERNAL, payload);
            if (rc != NO_ERROR) {
                ALOGE("%s: processEvt prep_snapshot failed", __func__);
                free(payload);
                payload = NULL;

            }
        } else {
            ALOGE("%s: No memory for prep_snapshot qcamera_sm_internal_evt_payload_t", __func__);
        }
    }

    IF_META_AVAILABLE(cam_asd_hdr_scene_data_t, hdr_scene_data,
            CAM_INTF_META_ASD_HDR_SCENE_DATA, pMetaData) {
        CDBG_HIGH("%s: hdr_scene_data: %d %f\n", __func__,
                hdr_scene_data->is_hdr_scene, hdr_scene_data->hdr_confidence);
        //Handle this HDR meta data only if capture is not in process
        if (!pme->m_stateMachine.isCaptureRunning()) {
            qcamera_sm_internal_evt_payload_t *payload =
                    (qcamera_sm_internal_evt_payload_t *)
                    malloc(sizeof(qcamera_sm_internal_evt_payload_t));
            if (NULL != payload) {
                memset(payload, 0, sizeof(qcamera_sm_internal_evt_payload_t));
                payload->evt_type = QCAMERA_INTERNAL_EVT_HDR_UPDATE;
                payload->hdr_data = *hdr_scene_data;
                int32_t rc = pme->processEvt(QCAMERA_SM_EVT_EVT_INTERNAL, payload);
                if (rc != NO_ERROR) {
                    ALOGE("%s: processEvt hdr update failed", __func__);
                    free(payload);
                    payload = NULL;
                }
            } else {
                ALOGE("%s: No memory for hdr update qcamera_sm_internal_evt_payload_t",
                        __func__);
            }
        }
    }

    IF_META_AVAILABLE(int32_t, scene, CAM_INTF_META_ASD_SCENE_TYPE, pMetaData) {
        qcamera_sm_internal_evt_payload_t *payload =
            (qcamera_sm_internal_evt_payload_t *)malloc(sizeof(qcamera_sm_internal_evt_payload_t));
        if (NULL != payload) {
            memset(payload, 0, sizeof(qcamera_sm_internal_evt_payload_t));
            payload->evt_type = QCAMERA_INTERNAL_EVT_ASD_UPDATE;
            payload->asd_data = (cam_auto_scene_t)*scene;
            int32_t rc = pme->processEvt(QCAMERA_SM_EVT_EVT_INTERNAL, payload);
            if (rc != NO_ERROR) {
                ALOGE("%s: processEvt asd_update failed", __func__);
                free(payload);
                payload = NULL;
            }
        } else {
            ALOGE("%s: No memory for asd_update qcamera_sm_internal_evt_payload_t", __func__);
        }
    }

    IF_META_AVAILABLE(cam_awb_params_t, awb_params, CAM_INTF_META_AWB_INFO, pMetaData) {
        CDBG_HIGH("%s, metadata for awb params.", __func__);
        qcamera_sm_internal_evt_payload_t *payload =
                (qcamera_sm_internal_evt_payload_t *)
                malloc(sizeof(qcamera_sm_internal_evt_payload_t));
        if (NULL != payload) {
            memset(payload, 0, sizeof(qcamera_sm_internal_evt_payload_t));
            payload->evt_type = QCAMERA_INTERNAL_EVT_AWB_UPDATE;
            payload->awb_data = *awb_params;
            int32_t rc = pme->processEvt(QCAMERA_SM_EVT_EVT_INTERNAL, payload);
            if (rc != NO_ERROR) {
                ALOGE("%s: processEvt awb_update failed", __func__);
                free(payload);
                payload = NULL;
            }
        } else {
            ALOGE("%s: No memory for awb_update qcamera_sm_internal_evt_payload_t", __func__);
        }
    }

    IF_META_AVAILABLE(uint32_t, flash_mode, CAM_INTF_META_FLASH_MODE, pMetaData) {
        pme->mExifParams.sensor_params.flash_mode = (cam_flash_mode_t)*flash_mode;
    }

    IF_META_AVAILABLE(int32_t, flash_state, CAM_INTF_META_FLASH_STATE, pMetaData) {
        pme->mExifParams.sensor_params.flash_state = (cam_flash_state_t) *flash_state;
    }

    IF_META_AVAILABLE(float, aperture_value, CAM_INTF_META_LENS_APERTURE, pMetaData) {
        pme->mExifParams.sensor_params.aperture_value = *aperture_value;
    }

    IF_META_AVAILABLE(cam_3a_params_t, ae_params, CAM_INTF_META_AEC_INFO, pMetaData) {
        pme->mExifParams.cam_3a_params = *ae_params;
        pme->mExifParams.cam_3a_params_valid = TRUE;
        pme->mFlashNeeded = ae_params->flash_needed;
        pthread_mutex_lock(&pme->m_parm_lock);
        pme->mExifParams.cam_3a_params.brightness = (float) pme->mParameters.getBrightness();
        pthread_mutex_unlock(&pme->m_parm_lock);
        qcamera_sm_internal_evt_payload_t *payload =
                (qcamera_sm_internal_evt_payload_t *)
                malloc(sizeof(qcamera_sm_internal_evt_payload_t));
        if (NULL != payload) {
            memset(payload, 0, sizeof(qcamera_sm_internal_evt_payload_t));
            payload->evt_type = QCAMERA_INTERNAL_EVT_AE_UPDATE;
            payload->ae_data = *ae_params;
            int32_t rc = pme->processEvt(QCAMERA_SM_EVT_EVT_INTERNAL, payload);
            if (rc != NO_ERROR) {
                ALOGE("%s: processEvt ae_update failed", __func__);
                free(payload);
                payload = NULL;
            }
        } else {
            ALOGE("%s: No memory for ae_update qcamera_sm_internal_evt_payload_t", __func__);
        }
    }

    IF_META_AVAILABLE(int32_t, wb_mode, CAM_INTF_PARM_WHITE_BALANCE, pMetaData) {
        pme->mExifParams.cam_3a_params.wb_mode = (cam_wb_mode_type) *wb_mode;
    }

    IF_META_AVAILABLE(cam_sensor_params_t, sensor_params, CAM_INTF_META_SENSOR_INFO, pMetaData) {
        pme->mExifParams.sensor_params = *sensor_params;
    }

    IF_META_AVAILABLE(cam_ae_exif_debug_t, ae_exif_debug_params,
            CAM_INTF_META_EXIF_DEBUG_AE, pMetaData) {
        if (pme->mExifParams.debug_params) {
            pme->mExifParams.debug_params->ae_debug_params = *ae_exif_debug_params;
            pme->mExifParams.debug_params->ae_debug_params_valid = TRUE;
        }
    }

    IF_META_AVAILABLE(cam_awb_exif_debug_t, awb_exif_debug_params,
            CAM_INTF_META_EXIF_DEBUG_AWB, pMetaData) {
        if (pme->mExifParams.debug_params) {
            pme->mExifParams.debug_params->awb_debug_params = *awb_exif_debug_params;
            pme->mExifParams.debug_params->awb_debug_params_valid = TRUE;
        }
    }

    IF_META_AVAILABLE(cam_af_exif_debug_t, af_exif_debug_params,
            CAM_INTF_META_EXIF_DEBUG_AF, pMetaData) {
        if (pme->mExifParams.debug_params) {
            pme->mExifParams.debug_params->af_debug_params = *af_exif_debug_params;
            pme->mExifParams.debug_params->af_debug_params_valid = TRUE;
        }
    }

    IF_META_AVAILABLE(cam_asd_exif_debug_t, asd_exif_debug_params,
            CAM_INTF_META_EXIF_DEBUG_ASD, pMetaData) {
        if (pme->mExifParams.debug_params) {
            pme->mExifParams.debug_params->asd_debug_params = *asd_exif_debug_params;
            pme->mExifParams.debug_params->asd_debug_params_valid = TRUE;
        }
    }

    IF_META_AVAILABLE(cam_stats_buffer_exif_debug_t, stats_exif_debug_params,
            CAM_INTF_META_EXIF_DEBUG_STATS, pMetaData) {
        if (pme->mExifParams.debug_params) {
            pme->mExifParams.debug_params->stats_debug_params = *stats_exif_debug_params;
            pme->mExifParams.debug_params->stats_debug_params_valid = TRUE;
        }
    }

    IF_META_AVAILABLE(uint32_t, led_mode, CAM_INTF_META_LED_MODE_OVERRIDE, pMetaData) {
        qcamera_sm_internal_evt_payload_t *payload =
                (qcamera_sm_internal_evt_payload_t *)
                malloc(sizeof(qcamera_sm_internal_evt_payload_t));
        if (NULL != payload) {
            memset(payload, 0, sizeof(qcamera_sm_internal_evt_payload_t));
            payload->evt_type = QCAMERA_INTERNAL_EVT_LED_MODE_OVERRIDE;
            payload->led_data = (cam_flash_mode_t)*led_mode;
            int32_t rc = pme->processEvt(QCAMERA_SM_EVT_EVT_INTERNAL, payload);
            if (rc != NO_ERROR) {
                ALOGE("%s: processEvt led mode override failed", __func__);
                free(payload);
                payload = NULL;
            }
        } else {
            ALOGE("%s: No memory for focus qcamera_sm_internal_evt_payload_t", __func__);
        }
    }

    cam_edge_application_t edge_application;
    memset(&edge_application, 0x00, sizeof(cam_edge_application_t));
    edge_application.sharpness = pme->mParameters.getSharpness();
    if (edge_application.sharpness != 0) {
        edge_application.edge_mode = CAM_EDGE_MODE_FAST;
    } else {
        edge_application.edge_mode = CAM_EDGE_MODE_OFF;
    }
    ADD_SET_PARAM_ENTRY_TO_BATCH(pMetaData, CAM_INTF_META_EDGE_MODE, edge_application);

    IF_META_AVAILABLE(cam_focus_pos_info_t, cur_pos_info,
            CAM_INTF_META_FOCUS_POSITION, pMetaData) {
        qcamera_sm_internal_evt_payload_t *payload =
            (qcamera_sm_internal_evt_payload_t *)malloc(sizeof(qcamera_sm_internal_evt_payload_t));
        if (NULL != payload) {
            memset(payload, 0, sizeof(qcamera_sm_internal_evt_payload_t));
            payload->evt_type = QCAMERA_INTERNAL_EVT_FOCUS_POS_UPDATE;
            payload->focus_pos = *cur_pos_info;
            int32_t rc = pme->processEvt(QCAMERA_SM_EVT_EVT_INTERNAL, payload);
            if (rc != NO_ERROR) {
                ALOGE("%s: processEvt focus_pos_update failed", __func__);
                free(payload);
                payload = NULL;
            }
        } else {
            ALOGE("%s: No memory for focus_pos_update qcamera_sm_internal_evt_payload_t", __func__);
        }
    }

    stream->bufDone(frame->buf_idx);
    free(super_frame);

    CDBG("[KPI Perf] %s : END", __func__);
}

/*===========================================================================
 * FUNCTION   : reprocess_stream_cb_routine
 *
 * DESCRIPTION: helper function to handle reprocess frame from reprocess stream
                (after reprocess, e.g., ZSL snapshot frame after WNR if
 *              WNR is enabled)
 *
 * PARAMETERS :
 *   @super_frame : received super buffer
 *   @stream      : stream object
 *   @userdata    : user data ptr
 *
 * RETURN    : None
 *
 * NOTE      : caller passes the ownership of super_frame, it's our
 *             responsibility to free super_frame once it's done. In this
 *             case, reprocessed frame need to be passed to postprocessor
 *             for jpeg encoding.
 *==========================================================================*/
void QCamera2HardwareInterface::reprocess_stream_cb_routine(mm_camera_super_buf_t * super_frame,
                                                            QCameraStream * /*stream*/,
                                                            void * userdata)
{
    ATRACE_CALL();
    CDBG_HIGH("[KPI Perf] %s: E", __func__);
    QCamera2HardwareInterface *pme = (QCamera2HardwareInterface *)userdata;
    if (pme == NULL ||
        pme->mCameraHandle == NULL ||
        pme->mCameraHandle->camera_handle != super_frame->camera_handle){
        ALOGE("%s: camera obj not valid", __func__);
        // simply free super frame
        free(super_frame);
        return;
    }

    pme->m_postprocessor.processPPData(super_frame);

    CDBG_HIGH("[KPI Perf] %s: X", __func__);
}

/*===========================================================================
 * FUNCTION   : callback_stream_cb_routine
 *
 * DESCRIPTION: function to process CALBACK stream data
                           Frame will processed and sent to framework
 *
 * PARAMETERS :
 *   @super_frame : received super buffer
 *   @stream      : stream object
 *   @userdata    : user data ptr
 *
 * RETURN    : None
 *==========================================================================*/
void QCamera2HardwareInterface::callback_stream_cb_routine(mm_camera_super_buf_t *super_frame,
        QCameraStream *stream, void *userdata)
{
    ATRACE_CALL();
    CDBG_HIGH("[KPI Perf] %s: E", __func__);
    QCamera2HardwareInterface *pme = (QCamera2HardwareInterface *)userdata;

    if (pme == NULL ||
            pme->mCameraHandle == NULL ||
            pme->mCameraHandle->camera_handle != super_frame->camera_handle) {
        ALOGE("%s: camera obj not valid", __func__);
        // simply free super frame
        free(super_frame);
        return;
    }

    mm_camera_buf_def_t *frame = super_frame->bufs[0];
    if (NULL == frame) {
        ALOGE("%s: preview callback frame is NULL", __func__);
        free(super_frame);
        return;
    }

    if (!pme->needProcessPreviewFrame()) {
        CDBG_HIGH("%s: preview is not running, no need to process", __func__);
        stream->bufDone(frame->buf_idx);
        free(super_frame);
        return;
    }

    QCameraMemory *previewMemObj = (QCameraMemory *)frame->mem_info;
    // Handle preview data callback
    if (pme->mDataCb != NULL &&
            (pme->msgTypeEnabledWithLock(CAMERA_MSG_PREVIEW_FRAME) > 0) &&
            (!pme->mParameters.isSceneSelectionEnabled())) {
        int32_t rc = pme->sendPreviewCallback(stream, previewMemObj, frame->buf_idx);
        if (NO_ERROR != rc) {
            ALOGE("%s: Preview callback was not sent succesfully", __func__);
        }
    }
    stream->bufDone(frame->buf_idx);
    free(super_frame);
    CDBG_HIGH("[KPI Perf] %s: X", __func__);
}

/*===========================================================================
 * FUNCTION   : dumpFrameToFile
 *
 * DESCRIPTION: helper function to dump jpeg into file for debug purpose.
 *
 * PARAMETERS :
 *    @data : data ptr
 *    @size : length of data buffer
 *    @index : identifier for data
 *
 * RETURN     : None
 *==========================================================================*/
void QCamera2HardwareInterface::dumpJpegToFile(const void *data,
        size_t size, uint32_t index)
{
    char value[PROPERTY_VALUE_MAX];
    property_get("persist.camera.dumpimg", value, "0");
    uint32_t enabled = (uint32_t) atoi(value);
    uint32_t frm_num = 0;
    uint32_t skip_mode = 0;

    char buf[32];
    cam_dimension_t dim;
    memset(buf, 0, sizeof(buf));
    memset(&dim, 0, sizeof(dim));

    if(((enabled & QCAMERA_DUMP_FRM_JPEG) && data) ||
        ((true == m_bIntJpegEvtPending) && data)) {
        frm_num = ((enabled & 0xffff0000) >> 16);
        if(frm_num == 0) {
            frm_num = 10; //default 10 frames
        }
        if(frm_num > 256) {
            frm_num = 256; //256 buffers cycle around
        }
        skip_mode = ((enabled & 0x0000ff00) >> 8);
        if(skip_mode == 0) {
            skip_mode = 1; //no-skip
        }

        if( mDumpSkipCnt % skip_mode == 0) {
            if((frm_num == 256) && (mDumpFrmCnt >= frm_num)) {
                // reset frame count if cycling
                mDumpFrmCnt = 0;
            }
            if (mDumpFrmCnt <= frm_num) {
                snprintf(buf, sizeof(buf), QCAMERA_DUMP_FRM_LOCATION "%d_%d.jpg",
                        mDumpFrmCnt, index);
                if (true == m_bIntJpegEvtPending) {
                    strlcpy(m_BackendFileName, buf, sizeof(buf));
                    mBackendFileSize = size;
                }

                int file_fd = open(buf, O_RDWR | O_CREAT, 0777);
                if (file_fd >= 0) {
                    ssize_t written_len = write(file_fd, data, size);
                    fchmod(file_fd, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
                    CDBG_HIGH("%s: written number of bytes %zd\n",
                            __func__, written_len);
                    close(file_fd);
                } else {
                    ALOGE("%s: fail t open file for image dumping", __func__);
                }
                if (false == m_bIntJpegEvtPending) {
                    mDumpFrmCnt++;
                }
            }
        }
        mDumpSkipCnt++;
    }
}


void QCamera2HardwareInterface::dumpMetadataToFile(QCameraStream *stream,
                                                   mm_camera_buf_def_t *frame,char *type)
{
    char value[PROPERTY_VALUE_MAX];
    uint32_t frm_num = 0;
    metadata_buffer_t *metadata = (metadata_buffer_t *)frame->buffer;
    property_get("persist.camera.dumpmetadata", value, "0");
    uint32_t enabled = (uint32_t) atoi(value);
    if (stream == NULL) {
        CDBG_HIGH("No op");
        return;
    }

    uint32_t dumpFrmCnt = stream->mDumpMetaFrame;
    if(enabled){
        frm_num = ((enabled & 0xffff0000) >> 16);
        if (frm_num == 0) {
            frm_num = 10; //default 10 frames
        }
        if (frm_num > 256) {
            frm_num = 256; //256 buffers cycle around
        }
        if ((frm_num == 256) && (dumpFrmCnt >= frm_num)) {
            // reset frame count if cycling
            dumpFrmCnt = 0;
        }
        CDBG_HIGH("dumpFrmCnt= %u, frm_num = %u", dumpFrmCnt, frm_num);
        if (dumpFrmCnt < frm_num) {
            char timeBuf[128];
            char buf[32];
            memset(buf, 0, sizeof(buf));
            memset(timeBuf, 0, sizeof(timeBuf));
            time_t current_time;
            struct tm * timeinfo;
            time (&current_time);
            timeinfo = localtime (&current_time);
            if (NULL != timeinfo) {
                strftime(timeBuf, sizeof(timeBuf),
                        QCAMERA_DUMP_FRM_LOCATION "%Y%m%d%H%M%S", timeinfo);
            }
            String8 filePath(timeBuf);
            snprintf(buf, sizeof(buf), "%um_%s_%d.bin", dumpFrmCnt, type, frame->frame_idx);
            filePath.append(buf);
            int file_fd = open(filePath.string(), O_RDWR | O_CREAT, 0777);
            if (file_fd >= 0) {
                ssize_t written_len = 0;
                metadata->tuning_params.tuning_data_version = TUNING_DATA_VERSION;
                void *data = (void *)((uint8_t *)&metadata->tuning_params.tuning_data_version);
                written_len += write(file_fd, data, sizeof(uint32_t));
                data = (void *)((uint8_t *)&metadata->tuning_params.tuning_sensor_data_size);
                CDBG_HIGH("tuning_sensor_data_size %d",(int)(*(int *)data));
                written_len += write(file_fd, data, sizeof(uint32_t));
                data = (void *)((uint8_t *)&metadata->tuning_params.tuning_vfe_data_size);
                CDBG_HIGH("tuning_vfe_data_size %d",(int)(*(int *)data));
                written_len += write(file_fd, data, sizeof(uint32_t));
                data = (void *)((uint8_t *)&metadata->tuning_params.tuning_cpp_data_size);
                CDBG_HIGH("tuning_cpp_data_size %d",(int)(*(int *)data));
                written_len += write(file_fd, data, sizeof(uint32_t));
                data = (void *)((uint8_t *)&metadata->tuning_params.tuning_cac_data_size);
                CDBG_HIGH("tuning_cac_data_size %d",(int)(*(int *)data));
                written_len += write(file_fd, data, sizeof(uint32_t));
                data = (void *)((uint8_t *)&metadata->tuning_params.tuning_cac_data_size2);
                CDBG_HIGH("%s < skrajago >tuning_cac_data_size %d",__func__,(int)(*(int *)data));
                written_len += write(file_fd, data, sizeof(uint32_t));
                size_t total_size = metadata->tuning_params.tuning_sensor_data_size;
                data = (void *)((uint8_t *)&metadata->tuning_params.data);
                written_len += write(file_fd, data, total_size);
                total_size = metadata->tuning_params.tuning_vfe_data_size;
                data = (void *)((uint8_t *)&metadata->tuning_params.data[TUNING_VFE_DATA_OFFSET]);
                written_len += write(file_fd, data, total_size);
                total_size = metadata->tuning_params.tuning_cpp_data_size;
                data = (void *)((uint8_t *)&metadata->tuning_params.data[TUNING_CPP_DATA_OFFSET]);
                written_len += write(file_fd, data, total_size);
                total_size = metadata->tuning_params.tuning_cac_data_size;
                data = (void *)((uint8_t *)&metadata->tuning_params.data[TUNING_CAC_DATA_OFFSET]);
                written_len += write(file_fd, data, total_size);
                close(file_fd);
            }else {
                ALOGE("%s: fail t open file for image dumping", __func__);
            }
            dumpFrmCnt++;
        }
    }
    stream->mDumpMetaFrame = dumpFrmCnt;
}
/*===========================================================================
 * FUNCTION   : dumpFrameToFile
 *
 * DESCRIPTION: helper function to dump frame into file for debug purpose.
 *
 * PARAMETERS :
 *    @data : data ptr
 *    @size : length of data buffer
 *    @index : identifier for data
 *    @dump_type : type of the frame to be dumped. Only such
 *                 dump type is enabled, the frame will be
 *                 dumped into a file.
 *
 * RETURN     : None
 *==========================================================================*/
void QCamera2HardwareInterface::dumpFrameToFile(QCameraStream *stream,
        mm_camera_buf_def_t *frame, uint32_t dump_type)
{
    char value[PROPERTY_VALUE_MAX];
    property_get("persist.camera.dumpimg", value, "0");
    uint32_t enabled = (uint32_t) atoi(value);
    uint32_t frm_num = 0;
    uint32_t skip_mode = 0;

    if (NULL == stream) {
        ALOGE("%s stream object is null", __func__);
        return;
    }

    uint32_t dumpFrmCnt = stream->mDumpFrame;

    if (true == m_bIntRawEvtPending) {
        enabled = QCAMERA_DUMP_FRM_RAW;
    }

    if((enabled & QCAMERA_DUMP_FRM_MASK_ALL)) {
        if((enabled & dump_type) && stream && frame) {
            frm_num = ((enabled & 0xffff0000) >> 16);
            if(frm_num == 0) {
                frm_num = 10; //default 10 frames
            }
            if(frm_num > 256) {
                frm_num = 256; //256 buffers cycle around
            }
            skip_mode = ((enabled & 0x0000ff00) >> 8);
            if(skip_mode == 0) {
                skip_mode = 1; //no-skip
            }
            if(stream->mDumpSkipCnt == 0)
                stream->mDumpSkipCnt = 1;

            if( stream->mDumpSkipCnt % skip_mode == 0) {
                if((frm_num == 256) && (dumpFrmCnt >= frm_num)) {
                    // reset frame count if cycling
                    dumpFrmCnt = 0;
                }
                if (dumpFrmCnt <= frm_num) {
                    char buf[32];
                    char timeBuf[128];
                    time_t current_time;
                    struct tm * timeinfo;

                    memset(timeBuf, 0, sizeof(timeBuf));

                    time (&current_time);
                    timeinfo = localtime (&current_time);
                    memset(buf, 0, sizeof(buf));

                    cam_dimension_t dim;
                    memset(&dim, 0, sizeof(dim));
                    stream->getFrameDimension(dim);

                    cam_frame_len_offset_t offset;
                    memset(&offset, 0, sizeof(cam_frame_len_offset_t));
                    stream->getFrameOffset(offset);

                    if (NULL != timeinfo) {
                        strftime(timeBuf, sizeof(timeBuf),
                                QCAMERA_DUMP_FRM_LOCATION "%Y%m%d%H%M%S", timeinfo);
                    }
                    String8 filePath(timeBuf);
                    switch (dump_type) {
                    case QCAMERA_DUMP_FRM_PREVIEW:
                        {
                            snprintf(buf, sizeof(buf), "%dp_%dx%d_%d.yuv",
                                    dumpFrmCnt, dim.width, dim.height, frame->frame_idx);
                        }
                        break;
                    case QCAMERA_DUMP_FRM_THUMBNAIL:
                        {
                            snprintf(buf, sizeof(buf), "%dt_%dx%d_%d.yuv",
                                    dumpFrmCnt, dim.width, dim.height, frame->frame_idx);
                        }
                        break;
                    case QCAMERA_DUMP_FRM_SNAPSHOT:
                        {
                            mParameters.getStreamDimension(CAM_STREAM_TYPE_SNAPSHOT, dim);
                            snprintf(buf, sizeof(buf), "%ds_%dx%d_%d.yuv",
                                    dumpFrmCnt, dim.width, dim.height, frame->frame_idx);
                        }
                        break;
                    case QCAMERA_DUMP_FRM_VIDEO:
                        {
                            snprintf(buf, sizeof(buf), "%dv_%dx%d_%d.yuv",
                                    dumpFrmCnt, dim.width, dim.height, frame->frame_idx);
                        }
                        break;
                    case QCAMERA_DUMP_FRM_RAW:
                        {
                            mParameters.getStreamDimension(CAM_STREAM_TYPE_RAW, dim);
                            snprintf(buf, sizeof(buf), "%dr_%dx%d_%d.raw",
                                    dumpFrmCnt, dim.width, dim.height, frame->frame_idx);
                        }
                        break;
                    case QCAMERA_DUMP_FRM_JPEG:
                        {
                            mParameters.getStreamDimension(CAM_STREAM_TYPE_SNAPSHOT, dim);
                            snprintf(buf, sizeof(buf), "%dj_%dx%d_%d.yuv",
                                    dumpFrmCnt, dim.width, dim.height, frame->frame_idx);
                        }
                        break;
                    default:
                        ALOGE("%s: Not supported for dumping stream type %d",
                              __func__, dump_type);
                        return;
                    }

                    filePath.append(buf);
                    int file_fd = open(filePath.string(), O_RDWR | O_CREAT, 0777);
                    ssize_t written_len = 0;
                    if (file_fd >= 0) {
                        void *data = NULL;

                        fchmod(file_fd, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
                        for (uint32_t i = 0; i < offset.num_planes; i++) {
                            uint32_t index = offset.mp[i].offset;
                            if (i > 0) {
                                index += offset.mp[i-1].len;
                            }
                            for (int j = 0; j < offset.mp[i].height; j++) {
                                data = (void *)((uint8_t *)frame->buffer + index);
                                written_len += write(file_fd, data,
                                        (size_t)offset.mp[i].width);
                                index += (uint32_t)offset.mp[i].stride;
                            }
                        }

                        CDBG_HIGH("%s: written number of bytes %ld\n",
                            __func__, written_len);
                        close(file_fd);
                    } else {
                        ALOGE("%s: fail t open file for image dumping", __func__);
                    }
                    if (true == m_bIntRawEvtPending) {
                        strlcpy(m_BackendFileName, filePath.string(), QCAMERA_MAX_FILEPATH_LENGTH);
                        mBackendFileSize = (size_t)written_len;
                    } else {
                        dumpFrmCnt++;
                    }
                }
            }
            stream->mDumpSkipCnt++;
        }
    } else {
        dumpFrmCnt = 0;
    }
    stream->mDumpFrame = dumpFrmCnt;
}

/*===========================================================================
 * FUNCTION   : debugShowVideoFPS
 *
 * DESCRIPTION: helper function to log video frame FPS for debug purpose.
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *==========================================================================*/
void QCamera2HardwareInterface::debugShowVideoFPS()
{
    mVFrameCount++;
    nsecs_t now = systemTime();
    nsecs_t diff = now - mVLastFpsTime;
    if (diff > ms2ns(250)) {
        mVFps = (((double)(mVFrameCount - mVLastFrameCount)) *
                (double)(s2ns(1))) / (double)diff;
        CDBG_HIGH("Video Frames Per Second: %.4f Cam ID = %d", mVFps, mCameraId);
        mVLastFpsTime = now;
        mVLastFrameCount = mVFrameCount;
    }
}

/*===========================================================================
 * FUNCTION   : debugShowPreviewFPS
 *
 * DESCRIPTION: helper function to log preview frame FPS for debug purpose.
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *==========================================================================*/
void QCamera2HardwareInterface::debugShowPreviewFPS()
{
    mPFrameCount++;
    nsecs_t now = systemTime();
    nsecs_t diff = now - mPLastFpsTime;
    if (diff > ms2ns(250)) {
        mPFps = (((double)(mPFrameCount - mPLastFrameCount)) *
                (double)(s2ns(1))) / (double)diff;
        CDBG_HIGH("[KPI Perf] %s: PROFILE_PREVIEW_FRAMES_PER_SECOND : %.4f Cam ID = %d",
                __func__, mPFps, mCameraId);
        mPLastFpsTime = now;
        mPLastFrameCount = mPFrameCount;
    }
}

/*===========================================================================
 * FUNCTION   : ~QCameraCbNotifier
 *
 * DESCRIPTION: Destructor for exiting the callback context.
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *==========================================================================*/
QCameraCbNotifier::~QCameraCbNotifier()
{
}

/*===========================================================================
 * FUNCTION   : exit
 *
 * DESCRIPTION: exit notify thread.
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *==========================================================================*/
void QCameraCbNotifier::exit()
{
    mActive = false;
    mProcTh.exit();
}

/*===========================================================================
 * FUNCTION   : releaseNotifications
 *
 * DESCRIPTION: callback for releasing data stored in the callback queue.
 *
 * PARAMETERS :
 *   @data      : data to be released
 *   @user_data : context data
 *
 * RETURN     : None
 *==========================================================================*/
void QCameraCbNotifier::releaseNotifications(void *data, void *user_data)
{
    qcamera_callback_argm_t *arg = ( qcamera_callback_argm_t * ) data;

    if ( ( NULL != arg ) && ( NULL != user_data ) ) {
        if ( arg->release_cb ) {
            arg->release_cb(arg->user_data, arg->cookie, FAILED_TRANSACTION);
        }
    }
}

/*===========================================================================
 * FUNCTION   : matchSnapshotNotifications
 *
 * DESCRIPTION: matches snapshot data callbacks
 *
 * PARAMETERS :
 *   @data      : data to match
 *   @user_data : context data
 *
 * RETURN     : bool match
 *              true - match found
 *              false- match not found
 *==========================================================================*/
bool QCameraCbNotifier::matchSnapshotNotifications(void *data,
                                                   void */*user_data*/)
{
    qcamera_callback_argm_t *arg = ( qcamera_callback_argm_t * ) data;
    if ( NULL != arg ) {
        if ( QCAMERA_DATA_SNAPSHOT_CALLBACK == arg->cb_type ) {
            return true;
        }
    }

    return false;
}

/*===========================================================================
 * FUNCTION   : matchPreviewNotifications
 *
 * DESCRIPTION: matches preview data callbacks
 *
 * PARAMETERS :
 *   @data      : data to match
 *   @user_data : context data
 *
 * RETURN     : bool match
 *              true - match found
 *              false- match not found
 *==========================================================================*/
bool QCameraCbNotifier::matchPreviewNotifications(void *data,
        void */*user_data*/)
{
    qcamera_callback_argm_t *arg = ( qcamera_callback_argm_t * ) data;
    if (NULL != arg) {
        if ((QCAMERA_DATA_CALLBACK == arg->cb_type) &&
                (CAMERA_MSG_PREVIEW_FRAME == arg->msg_type)) {
            return true;
        }
    }

    return false;
}

#ifdef USE_MEDIA_EXTENSIONS
/*===========================================================================
* FUNCTION   : matchTimestampNotifications
*
* DESCRIPTION: matches timestamp data callbacks
*
* PARAMETERS :
*   @data      : data to match
*   @user_data : context data
*
* RETURN     : bool match
*              true - match found
*              false- match not found
*==========================================================================*/
bool QCameraCbNotifier::matchTimestampNotifications(void *data, void * /*user_data*/)
{
    qcamera_callback_argm_t *arg = ( qcamera_callback_argm_t * ) data;
    if (NULL != arg) {
        if ((QCAMERA_DATA_TIMESTAMP_CALLBACK == arg->cb_type) &&
            (CAMERA_MSG_VIDEO_FRAME == arg->msg_type)) {
            return true;
        }
    }
    return false;
}
#endif

/*===========================================================================
 * FUNCTION   : cbNotifyRoutine
 *
 * DESCRIPTION: callback thread which interfaces with the upper layers
 *              given input commands.
 *
 * PARAMETERS :
 *   @data    : context data
 *
 * RETURN     : None
 *==========================================================================*/
void * QCameraCbNotifier::cbNotifyRoutine(void * data)
{
    int running = 1;
    int ret;
    QCameraCbNotifier *pme = (QCameraCbNotifier *)data;
    QCameraCmdThread *cmdThread = &pme->mProcTh;
    cmdThread->setName("CAM_cbNotify");
    uint8_t isSnapshotActive = FALSE;
    bool longShotEnabled = false;
    uint32_t numOfSnapshotExpected = 0;
    uint32_t numOfSnapshotRcvd = 0;
    int32_t cbStatus = NO_ERROR;

    CDBG("%s: E", __func__);
    do {
        do {
            ret = cam_sem_wait(&cmdThread->cmd_sem);
            if (ret != 0 && errno != EINVAL) {
                CDBG("%s: cam_sem_wait error (%s)",
                           __func__, strerror(errno));
                return NULL;
            }
        } while (ret != 0);

        camera_cmd_type_t cmd = cmdThread->getCmd();
        CDBG("%s: get cmd %d", __func__, cmd);
        switch (cmd) {
        case CAMERA_CMD_TYPE_START_DATA_PROC:
            {
                isSnapshotActive = TRUE;
                numOfSnapshotExpected = pme->mParent->numOfSnapshotsExpected();
                longShotEnabled = pme->mParent->isLongshotEnabled();
                ALOGI("%s: Num Snapshots Expected = %d",
                  __func__, numOfSnapshotExpected);
                numOfSnapshotRcvd = 0;
            }
            break;
        case CAMERA_CMD_TYPE_STOP_DATA_PROC:
            {
                pme->mDataQ.flushNodes(matchSnapshotNotifications);
                isSnapshotActive = FALSE;

                numOfSnapshotExpected = 0;
                numOfSnapshotRcvd = 0;
            }
            break;
        case CAMERA_CMD_TYPE_DO_NEXT_JOB:
            {
                qcamera_callback_argm_t *cb =
                    (qcamera_callback_argm_t *)pme->mDataQ.dequeue();
                cbStatus = NO_ERROR;
                if (NULL != cb) {
                    CDBG("%s: cb type %d received",
                          __func__,
                          cb->cb_type);

                    if (pme->mParent->msgTypeEnabledWithLock(cb->msg_type)) {
                        switch (cb->cb_type) {
                        case QCAMERA_NOTIFY_CALLBACK:
                            {
                                if (cb->msg_type == CAMERA_MSG_FOCUS) {
                                    KPI_ATRACE_INT("Camera:AutoFocus", 0);
                                    CDBG_HIGH("[KPI Perf] %s : PROFILE_SENDING_FOCUS_EVT_TO APP",
                                        __func__);
                                }
                                if (pme->mNotifyCb) {
                                    pme->mNotifyCb(cb->msg_type,
                                                  cb->ext1,
                                                  cb->ext2,
                                                  pme->mCallbackCookie);
                                } else {
                                    ALOGE("%s : notify callback not set!",
                                          __func__);
                                }
                                if (cb->release_cb) {
                                    cb->release_cb(cb->user_data, cb->cookie,
                                            cbStatus);
                                }
                            }
                            break;
                        case QCAMERA_DATA_CALLBACK:
                            {
                                if (pme->mDataCb) {
                                    pme->mDataCb(cb->msg_type,
                                                 cb->data,
                                                 cb->index,
                                                 cb->metadata,
                                                 pme->mCallbackCookie);
                                } else {
                                    ALOGE("%s : data callback not set!",
                                          __func__);
                                }
                                if (cb->release_cb) {
                                    cb->release_cb(cb->user_data, cb->cookie,
                                            cbStatus);
                                }
                            }
                            break;
                        case QCAMERA_DATA_TIMESTAMP_CALLBACK:
                            {
                                if(pme->mDataCbTimestamp) {
                                    pme->mDataCbTimestamp(cb->timestamp,
                                                          cb->msg_type,
                                                          cb->data,
                                                          cb->index,
                                                          pme->mCallbackCookie);
                                } else {
                                    ALOGE("%s:data cb with tmp not set!",
                                          __func__);
                                }
                                if (cb->release_cb) {
                                    cb->release_cb(cb->user_data, cb->cookie,
                                            cbStatus);
                                }
                            }
                            break;
                        case QCAMERA_DATA_SNAPSHOT_CALLBACK:
                            {
                                if (TRUE == isSnapshotActive && pme->mDataCb ) {
                                    if (!longShotEnabled) {
                                        numOfSnapshotRcvd++;
                                        ALOGI("%s: [ZSL Retro] Num Snapshots Received = %d", __func__,
                                                numOfSnapshotRcvd);
                                        if (numOfSnapshotExpected > 0 &&
                                           (numOfSnapshotExpected == numOfSnapshotRcvd)) {
                                            ALOGI("%s: [ZSL Retro] Expected snapshot received = %d",
                                                    __func__, numOfSnapshotRcvd);
                                            // notify HWI that snapshot is done
                                            pme->mParent->processSyncEvt(QCAMERA_SM_EVT_SNAPSHOT_DONE,
                                                                         NULL);
                                        }
                                    }
                                    if (pme->mJpegCb) {
                                        ALOGI("%s: Calling JPEG Callback!! for camera %d"
                                                "release_data %p",
                                                "frame_idx %d",
                                                __func__, pme->mParent->getCameraId(),
                                                cb->user_data,
                                                cb->frame_index);
                                        pme->mJpegCb(cb->msg_type, cb->data,
                                                cb->index, cb->metadata,
                                                pme->mJpegCallbackCookie,
                                                cb->frame_index, cb->release_cb,
                                                cb->cookie, cb->user_data);
                                        // incase of non-null Jpeg cb we transfer
                                        // ownership of buffer to muxer. hence
                                        // release_cb should not be called
                                        // muxer will release after its done with
                                        // processing the buffer
                                    }
                                    else if(pme->mDataCb){
                                        pme->mDataCb(cb->msg_type, cb->data, cb->index,
                                                cb->metadata, pme->mCallbackCookie);
                                        if (cb->release_cb) {
                                            cb->release_cb(cb->user_data, cb->cookie,
                                                    cbStatus);
                                        }
                                    }
                                }
                            }
                            break;
                        default:
                            {
                                ALOGE("%s : invalid cb type %d",
                                      __func__,
                                      cb->cb_type);
                                cbStatus = BAD_VALUE;
                                if (cb->release_cb) {
                                    cb->release_cb(cb->user_data, cb->cookie,
                                            cbStatus);
                                }
                            }
                            break;
                        };
                    } else {
                        ALOGE("%s : cb message type %d not enabled!",
                              __func__,
                              cb->msg_type);
                        cbStatus = INVALID_OPERATION;
                        if (cb->release_cb) {
                            cb->release_cb(cb->user_data, cb->cookie, cbStatus);
                        }
                    }
                delete cb;
                } else {
                    ALOGE("%s: invalid cb type passed", __func__);
                }
            }
            break;
        case CAMERA_CMD_TYPE_EXIT:
            {
                running = 0;
                pme->mDataQ.flush();
            }
            break;
        default:
            break;
        }
    } while (running);
    CDBG("%s: X", __func__);

    return NULL;
}

/*===========================================================================
 * FUNCTION   : notifyCallback
 *
 * DESCRIPTION: Enqueus pending callback notifications for the upper layers.
 *
 * PARAMETERS :
 *   @cbArgs  : callback arguments
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraCbNotifier::notifyCallback(qcamera_callback_argm_t &cbArgs)
{
    if (!mActive) {
        ALOGE("%s: notify thread is not active", __func__);
        return UNKNOWN_ERROR;
    }

    qcamera_callback_argm_t *cbArg = new qcamera_callback_argm_t();
    if (NULL == cbArg) {
        ALOGE("%s: no mem for qcamera_callback_argm_t", __func__);
        return NO_MEMORY;
    }
    memset(cbArg, 0, sizeof(qcamera_callback_argm_t));
    *cbArg = cbArgs;

    if (mDataQ.enqueue((void *)cbArg)) {
        return mProcTh.sendCmd(CAMERA_CMD_TYPE_DO_NEXT_JOB, FALSE, FALSE);
    } else {
        ALOGE("%s: Error adding cb data into queue", __func__);
        delete cbArg;
        return UNKNOWN_ERROR;
    }
}

/*===========================================================================
 * FUNCTION   : setCallbacks
 *
 * DESCRIPTION: Initializes the callback functions, which would be used for
 *              communication with the upper layers and launches the callback
 *              context in which the callbacks will occur.
 *
 * PARAMETERS :
 *   @notifyCb          : notification callback
 *   @dataCb            : data callback
 *   @dataCbTimestamp   : data with timestamp callback
 *   @callbackCookie    : callback context data
 *
 * RETURN     : None
 *==========================================================================*/
void QCameraCbNotifier::setCallbacks(camera_notify_callback notifyCb,
                                     camera_data_callback dataCb,
                                     camera_data_timestamp_callback dataCbTimestamp,
                                     void *callbackCookie)
{
    if ( ( NULL == mNotifyCb ) &&
         ( NULL == mDataCb ) &&
         ( NULL == mDataCbTimestamp ) &&
         ( NULL == mCallbackCookie ) ) {
        mNotifyCb = notifyCb;
        mDataCb = dataCb;
        mDataCbTimestamp = dataCbTimestamp;
        mCallbackCookie = callbackCookie;
        mActive = true;
        mProcTh.launch(cbNotifyRoutine, this);
    } else {
        ALOGE("%s : Camera callback notifier already initialized!",
              __func__);
    }
}

/*===========================================================================
 * FUNCTION   : setJpegCallBacks
 *
 * DESCRIPTION: Initializes the JPEG callback function, which would be used for
 *              communication with the upper layers and launches the callback
 *              context in which the callbacks will occur.
 *
 * PARAMETERS :
 *   @jpegCb          : notification callback
 *   @callbackCookie    : callback context data
 *
 * RETURN     : None
 *==========================================================================*/
void QCameraCbNotifier::setJpegCallBacks(
        jpeg_data_callback jpegCb, void *callbackCookie)
{
    CDBG_HIGH("%s: Setting JPEG Callback notifier", __func__);
    mJpegCb        = jpegCb;
    mJpegCallbackCookie  = callbackCookie;
}

/*===========================================================================
 * FUNCTION   : flushPreviewNotifications
 *
 * DESCRIPTION: flush all pending preview notifications
 *              from the notifier queue
 *
 * PARAMETERS : None
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraCbNotifier::flushPreviewNotifications()
{
    if (!mActive) {
        ALOGE("%s: notify thread is not active", __func__);
        return UNKNOWN_ERROR;
    }

    mDataQ.flushNodes(matchPreviewNotifications);

    return NO_ERROR;
}

#ifdef USE_MEDIA_EXTENSIONS

/*===========================================================================
* FUNCTION   : flushVideoNotifications
*
* DESCRIPTION: flush all pending video notifications
*              from the notifier queue
*
* PARAMETERS : None
*
* RETURN     : int32_t type of status
*              NO_ERROR  -- success
*              none-zero failure code
*==========================================================================*/
int32_t QCameraCbNotifier::flushVideoNotifications()
{
    if (!mActive) {
        ALOGE("notify thread is not active");
        return UNKNOWN_ERROR;
    }
    mDataQ.flushNodes(matchTimestampNotifications);
    return NO_ERROR;
}

#endif

/*===========================================================================
 * FUNCTION   : startSnapshots
 *
 * DESCRIPTION: Enables snapshot mode
 *
 * PARAMETERS : None
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraCbNotifier::startSnapshots()
{
    return mProcTh.sendCmd(CAMERA_CMD_TYPE_START_DATA_PROC, FALSE, TRUE);
}

/*===========================================================================
 * FUNCTION   : stopSnapshots
 *
 * DESCRIPTION: Disables snapshot processing mode
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *==========================================================================*/
void QCameraCbNotifier::stopSnapshots()
{
    mProcTh.sendCmd(CAMERA_CMD_TYPE_STOP_DATA_PROC, FALSE, TRUE);
}

}; // namespace qcamera
