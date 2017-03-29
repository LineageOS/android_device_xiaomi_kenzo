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

#include <utils/Log.h>
#include <cutils/properties.h>
#include <hardware/camera.h>
#include <stdio.h>
#include <stdlib.h>
#include <utils/Errors.h>
#include <gralloc_priv.h>
#include <gui/Surface.h>
#include <binder/Parcel.h>
#include <binder/IServiceManager.h>
#include <utils/RefBase.h>
#include <QServiceUtils.h>
#include <dlfcn.h>

#include "QCamera2HWI.h"
#include "QCameraBufferMaps.h"
#include "QCameraMem.h"

#define MAP_TO_DRIVER_COORDINATE(val, base, scale, offset) \
  ((int32_t)val * (int32_t)scale / (int32_t)base + (int32_t)offset)
#define CAMERA_MIN_STREAMING_BUFFERS     3
#define EXTRA_ZSL_PREVIEW_STREAM_BUF     2
#define CAMERA_MIN_JPEG_ENCODING_BUFFERS 2
#define CAMERA_MIN_VIDEO_BUFFERS         9
#define CAMERA_MIN_CALLBACK_BUFFERS      5
#define CAMERA_LONGSHOT_STAGES           4
#define CAMERA_MIN_VIDEO_BATCH_BUFFERS   6
#define CAMERA_ISP_PING_PONG_BUFFERS     2
#define MIN_UNDEQUEUED_BUFFERS           1 // This is required if preview window is not set

#define HDR_CONFIDENCE_THRESHOLD 0.4

// Very long wait, just to be sure we don't deadlock
#define CAMERA_DEFERRED_THREAD_TIMEOUT 5000000000 // 5 seconds
#define CAMERA_MIN_METADATA_BUFFERS 10 // Need at least 10 for ZSL snapshot
#define CAMERA_INITIAL_MAPPABLE_PREVIEW_BUFFERS 5

namespace qcamera {

extern cam_capability_t *gCamCapability[MM_CAMERA_MAX_NUM_SENSORS];
extern pthread_mutex_t gCamLock;
volatile uint32_t gCamHalLogLevel = 1;
extern uint8_t gNumCameraSessions;
uint32_t QCamera2HardwareInterface::sNextJobId = 1;

camera_device_ops_t QCamera2HardwareInterface::mCameraOps = {
    set_preview_window:         QCamera2HardwareInterface::set_preview_window,
    set_callbacks:              QCamera2HardwareInterface::set_CallBacks,
    enable_msg_type:            QCamera2HardwareInterface::enable_msg_type,
    disable_msg_type:           QCamera2HardwareInterface::disable_msg_type,
    msg_type_enabled:           QCamera2HardwareInterface::msg_type_enabled,

    start_preview:              QCamera2HardwareInterface::start_preview,
    stop_preview:               QCamera2HardwareInterface::stop_preview,
    preview_enabled:            QCamera2HardwareInterface::preview_enabled,
    store_meta_data_in_buffers: QCamera2HardwareInterface::store_meta_data_in_buffers,

    start_recording:            QCamera2HardwareInterface::start_recording,
    stop_recording:             QCamera2HardwareInterface::stop_recording,
    recording_enabled:          QCamera2HardwareInterface::recording_enabled,
    release_recording_frame:    QCamera2HardwareInterface::release_recording_frame,

    auto_focus:                 QCamera2HardwareInterface::auto_focus,
    cancel_auto_focus:          QCamera2HardwareInterface::cancel_auto_focus,

    take_picture:               QCamera2HardwareInterface::take_picture,
    cancel_picture:             QCamera2HardwareInterface::cancel_picture,

    set_parameters:             QCamera2HardwareInterface::set_parameters,
    get_parameters:             QCamera2HardwareInterface::get_parameters,
    put_parameters:             QCamera2HardwareInterface::put_parameters,
    send_command:               QCamera2HardwareInterface::send_command,

    release:                    QCamera2HardwareInterface::release,
    dump:                       QCamera2HardwareInterface::dump,
};

/*===========================================================================
 * FUNCTION   : set_preview_window
 *
 * DESCRIPTION: set preview window.
 *
 * PARAMETERS :
 *   @device  : ptr to camera device struct
 *   @window  : window ops table
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::set_preview_window(struct camera_device *device,
        struct preview_stream_ops *window)
{
    ATRACE_CALL();
    int rc = NO_ERROR;
    QCamera2HardwareInterface *hw =
        reinterpret_cast<QCamera2HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("%s: NULL camera device", __func__);
        return BAD_VALUE;
    }
    CDBG("%s: E camera id %d window = %p", __func__, hw->getCameraId(), window);

    hw->lockAPI();
    qcamera_api_result_t apiResult;
    rc = hw->processAPI(QCAMERA_SM_EVT_SET_PREVIEW_WINDOW, (void *)window);
    if (rc == NO_ERROR) {
        hw->waitAPIResult(QCAMERA_SM_EVT_SET_PREVIEW_WINDOW, &apiResult);
        rc = apiResult.status;
    }
    hw->unlockAPI();
    CDBG("%s: X camera id %d", __func__, hw->getCameraId());

    return rc;
}

/*===========================================================================
 * FUNCTION   : set_CallBacks
 *
 * DESCRIPTION: set callbacks for notify and data
 *
 * PARAMETERS :
 *   @device     : ptr to camera device struct
 *   @notify_cb  : notify cb
 *   @data_cb    : data cb
 *   @data_cb_timestamp  : video data cd with timestamp
 *   @get_memory : ops table for request gralloc memory
 *   @user       : user data ptr
 *
 * RETURN     : none
 *==========================================================================*/
void QCamera2HardwareInterface::set_CallBacks(struct camera_device *device,
        camera_notify_callback notify_cb,
        camera_data_callback data_cb,
        camera_data_timestamp_callback data_cb_timestamp,
        camera_request_memory get_memory,
        void *user)
{
    ATRACE_CALL();
    QCamera2HardwareInterface *hw =
        reinterpret_cast<QCamera2HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("NULL camera device");
        return;
    }
    CDBG("%s: E camera id %d", __func__, hw->getCameraId());

    qcamera_sm_evt_setcb_payload_t payload;
    payload.notify_cb = notify_cb;
    payload.data_cb = data_cb;
    payload.data_cb_timestamp = data_cb_timestamp;
    payload.get_memory = get_memory;
    payload.user = user;

    hw->lockAPI();
    qcamera_api_result_t apiResult;
    int32_t rc = hw->processAPI(QCAMERA_SM_EVT_SET_CALLBACKS, (void *)&payload);
    if (rc == NO_ERROR) {
        hw->waitAPIResult(QCAMERA_SM_EVT_SET_CALLBACKS, &apiResult);
    }
    hw->unlockAPI();
    CDBG("%s: X camera id %d", __func__, hw->getCameraId());

}

/*===========================================================================
 * FUNCTION   : enable_msg_type
 *
 * DESCRIPTION: enable certain msg type
 *
 * PARAMETERS :
 *   @device     : ptr to camera device struct
 *   @msg_type   : msg type mask
 *
 * RETURN     : none
 *==========================================================================*/
void QCamera2HardwareInterface::enable_msg_type(struct camera_device *device, int32_t msg_type)
{
    ATRACE_CALL();
    QCamera2HardwareInterface *hw =
        reinterpret_cast<QCamera2HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("NULL camera device");
        return;
    }
    CDBG("%s: E camera id %d", __func__, hw->getCameraId());

    hw->lockAPI();
    qcamera_api_result_t apiResult;
    int32_t rc = hw->processAPI(QCAMERA_SM_EVT_ENABLE_MSG_TYPE, (void *)&msg_type);
    if (rc == NO_ERROR) {
        hw->waitAPIResult(QCAMERA_SM_EVT_ENABLE_MSG_TYPE, &apiResult);
    }
    hw->unlockAPI();
    CDBG("%s: X camera id %d", __func__, hw->getCameraId());

}

/*===========================================================================
 * FUNCTION   : disable_msg_type
 *
 * DESCRIPTION: disable certain msg type
 *
 * PARAMETERS :
 *   @device     : ptr to camera device struct
 *   @msg_type   : msg type mask
 *
 * RETURN     : none
 *==========================================================================*/
void QCamera2HardwareInterface::disable_msg_type(struct camera_device *device, int32_t msg_type)
{
    ATRACE_CALL();
    QCamera2HardwareInterface *hw =
        reinterpret_cast<QCamera2HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("NULL camera device");
        return;
    }
    CDBG("%s: E camera id %d", __func__, hw->getCameraId());

    hw->lockAPI();
    qcamera_api_result_t apiResult;
    int32_t rc = hw->processAPI(QCAMERA_SM_EVT_DISABLE_MSG_TYPE, (void *)&msg_type);
    if (rc == NO_ERROR) {
        hw->waitAPIResult(QCAMERA_SM_EVT_DISABLE_MSG_TYPE, &apiResult);
    }
    hw->unlockAPI();
    CDBG("%s: X camera id %d", __func__, hw->getCameraId());

}

/*===========================================================================
 * FUNCTION   : msg_type_enabled
 *
 * DESCRIPTION: if certain msg type is enabled
 *
 * PARAMETERS :
 *   @device     : ptr to camera device struct
 *   @msg_type   : msg type mask
 *
 * RETURN     : 1 -- enabled
 *              0 -- not enabled
 *==========================================================================*/
int QCamera2HardwareInterface::msg_type_enabled(struct camera_device *device, int32_t msg_type)
{
    ATRACE_CALL();
    int ret = NO_ERROR;
    QCamera2HardwareInterface *hw =
        reinterpret_cast<QCamera2HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("NULL camera device");
        return BAD_VALUE;
    }
    CDBG("%s: E camera id %d", __func__, hw->getCameraId());

    hw->lockAPI();
    qcamera_api_result_t apiResult;
    ret = hw->processAPI(QCAMERA_SM_EVT_MSG_TYPE_ENABLED, (void *)&msg_type);
    if (ret == NO_ERROR) {
        hw->waitAPIResult(QCAMERA_SM_EVT_MSG_TYPE_ENABLED, &apiResult);
        ret = apiResult.enabled;
    }
    hw->unlockAPI();
    CDBG("%s: X camera id %d", __func__, hw->getCameraId());

   return ret;
}

/*===========================================================================
 * FUNCTION   : prepare_preview
 *
 * DESCRIPTION: prepare preview
 *
 * PARAMETERS :
 *   @device  : ptr to camera device struct
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::prepare_preview(struct camera_device *device)
{
    ATRACE_CALL();
    int ret = NO_ERROR;
    QCamera2HardwareInterface *hw =
        reinterpret_cast<QCamera2HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("NULL camera device");
        return BAD_VALUE;
    }
    ALOGI("[KPI Perf] %s: E PROFILE_PREPARE_PREVIEW camera id %d",
            __func__, hw->getCameraId());
    hw->lockAPI();
    qcamera_api_result_t apiResult;
    qcamera_sm_evt_enum_t evt = QCAMERA_SM_EVT_PREPARE_PREVIEW;
    ret = hw->processAPI(evt, NULL);
    if (ret == NO_ERROR) {
        hw->waitAPIResult(evt, &apiResult);
        ret = apiResult.status;
    }
    hw->unlockAPI();
    ALOGI("[KPI Perf] %s: X", __func__);
    return ret;
}


/*===========================================================================
 * FUNCTION   : start_preview
 *
 * DESCRIPTION: start preview
 *
 * PARAMETERS :
 *   @device  : ptr to camera device struct
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::start_preview(struct camera_device *device)
{
    KPI_ATRACE_CALL();
    int ret = NO_ERROR;
    QCamera2HardwareInterface *hw =
        reinterpret_cast<QCamera2HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("NULL camera device");
        return BAD_VALUE;
    }
    ALOGI("[KPI Perf] %s: E PROFILE_START_PREVIEW camera id %d",
            __func__, hw->getCameraId());
    hw->lockAPI();
    qcamera_api_result_t apiResult;
    qcamera_sm_evt_enum_t evt = QCAMERA_SM_EVT_START_PREVIEW;
    if (hw->isNoDisplayMode()) {
        evt = QCAMERA_SM_EVT_START_NODISPLAY_PREVIEW;
    }
    ret = hw->processAPI(evt, NULL);
    if (ret == NO_ERROR) {
        hw->waitAPIResult(evt, &apiResult);
        ret = apiResult.status;
    }
    hw->unlockAPI();
    hw->m_bPreviewStarted = true;
    ALOGI("[KPI Perf] %s: X", __func__);
    return ret;
}

/*===========================================================================
 * FUNCTION   : stop_preview
 *
 * DESCRIPTION: stop preview
 *
 * PARAMETERS :
 *   @device  : ptr to camera device struct
 *
 * RETURN     : none
 *==========================================================================*/
void QCamera2HardwareInterface::stop_preview(struct camera_device *device)
{
    KPI_ATRACE_CALL();
    QCamera2HardwareInterface *hw =
        reinterpret_cast<QCamera2HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("NULL camera device");
        return;
    }
    ALOGI("[KPI Perf] %s: E PROFILE_STOP_PREVIEW camera id %d",
            __func__, hw->getCameraId());
    hw->lockAPI();
    qcamera_api_result_t apiResult;
    int32_t ret = hw->processAPI(QCAMERA_SM_EVT_STOP_PREVIEW, NULL);
    if (ret == NO_ERROR) {
        hw->waitAPIResult(QCAMERA_SM_EVT_STOP_PREVIEW, &apiResult);
    }
    hw->unlockAPI();
    ALOGI("[KPI Perf] %s: X", __func__);
}

/*===========================================================================
 * FUNCTION   : preview_enabled
 *
 * DESCRIPTION: if preview is running
 *
 * PARAMETERS :
 *   @device  : ptr to camera device struct
 *
 * RETURN     : 1 -- running
 *              0 -- not running
 *==========================================================================*/
int QCamera2HardwareInterface::preview_enabled(struct camera_device *device)
{
    ATRACE_CALL();
    int ret = NO_ERROR;
    QCamera2HardwareInterface *hw =
        reinterpret_cast<QCamera2HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("NULL camera device");
        return BAD_VALUE;
    }
    CDBG("%s: E camera id %d", __func__, hw->getCameraId());

    hw->lockAPI();
    qcamera_api_result_t apiResult;
    ret = hw->processAPI(QCAMERA_SM_EVT_PREVIEW_ENABLED, NULL);
    if (ret == NO_ERROR) {
        hw->waitAPIResult(QCAMERA_SM_EVT_PREVIEW_ENABLED, &apiResult);
        ret = apiResult.enabled;
    }

    //if preview enabled, can enable preview callback send
    if(apiResult.enabled) {
        hw->mParameters.setPreviewCallbackNeeded(true);
    }
    hw->unlockAPI();
    CDBG("%s: X camera id %d", __func__, hw->getCameraId());

    return ret;
}

/*===========================================================================
 * FUNCTION   : store_meta_data_in_buffers
 *
 * DESCRIPTION: if need to store meta data in buffers for video frame
 *
 * PARAMETERS :
 *   @device  : ptr to camera device struct
 *   @enable  : flag if enable
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::store_meta_data_in_buffers(
                struct camera_device *device, int enable)
{
    int ret = NO_ERROR;
    QCamera2HardwareInterface *hw =
        reinterpret_cast<QCamera2HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("NULL camera device");
        return BAD_VALUE;
    }
    CDBG("%s: E camera id %d", __func__, hw->getCameraId());

    hw->lockAPI();
    qcamera_api_result_t apiResult;
    ret = hw->processAPI(QCAMERA_SM_EVT_STORE_METADATA_IN_BUFS, (void *)&enable);
    if (ret == NO_ERROR) {
        hw->waitAPIResult(QCAMERA_SM_EVT_STORE_METADATA_IN_BUFS, &apiResult);
        ret = apiResult.status;
    }
    hw->unlockAPI();
    CDBG("%s: X camera id %d", __func__, hw->getCameraId());

    return ret;
}

/*===========================================================================
 * FUNCTION   : restart_start_preview
 *
 * DESCRIPTION: start preview as part of the restart preview
 *
 * PARAMETERS :
 *   @device  : ptr to camera device struct
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::restart_start_preview(struct camera_device *device)
{
    ATRACE_CALL();
    int ret = NO_ERROR;
    QCamera2HardwareInterface *hw =
        reinterpret_cast<QCamera2HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("NULL camera device");
        return BAD_VALUE;
    }
    CDBG_HIGH("%s: E camera id %d", __func__, hw->getCameraId());
    hw->lockAPI();
    qcamera_api_result_t apiResult;

    if (hw->getRelatedCamSyncInfo()->sync_control == CAM_SYNC_RELATED_SENSORS_ON) {
        ret = hw->processAPI(QCAMERA_SM_EVT_RESTART_START_PREVIEW, NULL);
        if (ret == NO_ERROR) {
            hw->waitAPIResult(QCAMERA_SM_EVT_RESTART_START_PREVIEW, &apiResult);
            ret = apiResult.status;
        }
    } else {
        ALOGE("%s is not supposed to be called in single-camera mode", __func__);
        ret = INVALID_OPERATION;
    }
    // Preview restart done, update the mPreviewRestartNeeded flag to false.
    hw->mPreviewRestartNeeded = false;
    hw->unlockAPI();
    CDBG_HIGH("%s: X camera id %d", __func__, hw->getCameraId());

    return ret;
}

/*===========================================================================
 * FUNCTION   : restart_stop_preview
 *
 * DESCRIPTION: stop preview as part of the restart preview
 *
 * PARAMETERS :
 *   @device  : ptr to camera device struct
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::restart_stop_preview(struct camera_device *device)
{
    ATRACE_CALL();
    int ret = NO_ERROR;
    QCamera2HardwareInterface *hw =
        reinterpret_cast<QCamera2HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("NULL camera device");
        return BAD_VALUE;
    }
    CDBG_HIGH("%s: E camera id %d", __func__, hw->getCameraId());
    hw->lockAPI();
    qcamera_api_result_t apiResult;

    if (hw->getRelatedCamSyncInfo()->sync_control == CAM_SYNC_RELATED_SENSORS_ON) {
        ret = hw->processAPI(QCAMERA_SM_EVT_RESTART_STOP_PREVIEW, NULL);
        if (ret == NO_ERROR) {
            hw->waitAPIResult(QCAMERA_SM_EVT_RESTART_STOP_PREVIEW, &apiResult);
            ret = apiResult.status;
        }
    } else {
        ALOGE("%s is not supposed to be called in single-camera mode", __func__);
        ret = INVALID_OPERATION;
    }

    hw->unlockAPI();
    CDBG_HIGH("%s: X camera id %d", __func__, hw->getCameraId());

    return ret;
}

/*===========================================================================
 * FUNCTION   : pre_start_recording
 *
 * DESCRIPTION: prepare for the start recording
 *
 * PARAMETERS :
 *   @device  : ptr to camera device struct
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::pre_start_recording(struct camera_device *device)
{
    ATRACE_CALL();
    int ret = NO_ERROR;
    QCamera2HardwareInterface *hw =
        reinterpret_cast<QCamera2HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("NULL camera device");
        return BAD_VALUE;
    }
    ALOGI("[KPI Perf] %s: E PROFILE_PRE_START_RECORDING camera id %d",
            __func__, hw->getCameraId());
    hw->lockAPI();
    qcamera_api_result_t apiResult;
    ret = hw->processAPI(QCAMERA_SM_EVT_PRE_START_RECORDING, NULL);
    if (ret == NO_ERROR) {
        hw->waitAPIResult(QCAMERA_SM_EVT_PRE_START_RECORDING, &apiResult);
        ret = apiResult.status;
    }
    hw->unlockAPI();
    ALOGI("[KPI Perf] %s: X", __func__);
    return ret;
}

/*===========================================================================
 * FUNCTION   : start_recording
 *
 * DESCRIPTION: start recording
 *
 * PARAMETERS :
 *   @device  : ptr to camera device struct
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::start_recording(struct camera_device *device)
{
    ATRACE_CALL();
    int ret = NO_ERROR;
    QCamera2HardwareInterface *hw =
        reinterpret_cast<QCamera2HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("NULL camera device");
        return BAD_VALUE;
    }
    ALOGI("[KPI Perf] %s: E PROFILE_START_RECORDING camera id %d",
            __func__, hw->getCameraId());
    // Give HWI control to call pre_start_recording in single camera mode.
    // In dual-cam mode, this control belongs to muxer.
    if (hw->getRelatedCamSyncInfo()->sync_control != CAM_SYNC_RELATED_SENSORS_ON) {
        ret = pre_start_recording(device);
        if (ret != NO_ERROR) {
            ALOGE("%s: pre_start_recording failed with ret = %d",__func__,ret);
            return ret;
        }
    }

    hw->lockAPI();
    qcamera_api_result_t apiResult;
    ret = hw->processAPI(QCAMERA_SM_EVT_START_RECORDING, NULL);
    if (ret == NO_ERROR) {
        hw->waitAPIResult(QCAMERA_SM_EVT_START_RECORDING, &apiResult);
        ret = apiResult.status;
    }
    hw->unlockAPI();
    hw->m_bRecordStarted = true;
    ALOGI("[KPI Perf] %s: X", __func__);
    return ret;
}

/*===========================================================================
 * FUNCTION   : stop_recording
 *
 * DESCRIPTION: stop recording
 *
 * PARAMETERS :
 *   @device  : ptr to camera device struct
 *
 * RETURN     : none
 *==========================================================================*/
void QCamera2HardwareInterface::stop_recording(struct camera_device *device)
{
    ATRACE_CALL();
    QCamera2HardwareInterface *hw =
        reinterpret_cast<QCamera2HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("NULL camera device");
        return;
    }
    ALOGI("[KPI Perf] %s: E PROFILE_STOP_RECORDING camera id %d",
            __func__, hw->getCameraId());
    hw->lockAPI();
    qcamera_api_result_t apiResult;
    int32_t ret = hw->processAPI(QCAMERA_SM_EVT_STOP_RECORDING, NULL);
    if (ret == NO_ERROR) {
        hw->waitAPIResult(QCAMERA_SM_EVT_STOP_RECORDING, &apiResult);
    }
    hw->unlockAPI();
    ALOGI("[KPI Perf] %s: X", __func__);
}

/*===========================================================================
 * FUNCTION   : recording_enabled
 *
 * DESCRIPTION: if recording is running
 *
 * PARAMETERS :
 *   @device  : ptr to camera device struct
 *
 * RETURN     : 1 -- running
 *              0 -- not running
 *==========================================================================*/
int QCamera2HardwareInterface::recording_enabled(struct camera_device *device)
{
    ATRACE_CALL();
    int ret = NO_ERROR;
    QCamera2HardwareInterface *hw =
        reinterpret_cast<QCamera2HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("NULL camera device");
        return BAD_VALUE;
    }
    CDBG("%s: E camera id %d", __func__, hw->getCameraId());
    hw->lockAPI();
    qcamera_api_result_t apiResult;
    ret = hw->processAPI(QCAMERA_SM_EVT_RECORDING_ENABLED, NULL);
    if (ret == NO_ERROR) {
        hw->waitAPIResult(QCAMERA_SM_EVT_RECORDING_ENABLED, &apiResult);
        ret = apiResult.enabled;
    }
    hw->unlockAPI();
    CDBG("%s: X camera id %d", __func__, hw->getCameraId());

    return ret;
}

/*===========================================================================
 * FUNCTION   : release_recording_frame
 *
 * DESCRIPTION: return recording frame back
 *
 * PARAMETERS :
 *   @device  : ptr to camera device struct
 *   @opaque  : ptr to frame to be returned
 *
 * RETURN     : none
 *==========================================================================*/
void QCamera2HardwareInterface::release_recording_frame(
            struct camera_device *device, const void *opaque)
{
    ATRACE_CALL();
    int32_t ret = NO_ERROR;
    QCamera2HardwareInterface *hw =
        reinterpret_cast<QCamera2HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("NULL camera device");
        return;
    }
    if (!opaque) {
        ALOGE("%s: Error!! Frame info is NULL", __func__);
        return;
    }
    CDBG("%s: E camera id %d", __func__, hw->getCameraId());

    hw->lockAPI();
    qcamera_api_result_t apiResult;
    ret = hw->processAPI(QCAMERA_SM_EVT_RELEASE_RECORIDNG_FRAME, (void *)opaque);
    if (ret == NO_ERROR) {
        hw->waitAPIResult(QCAMERA_SM_EVT_RELEASE_RECORIDNG_FRAME, &apiResult);
    }
    hw->unlockAPI();
    CDBG("%s: X camera id %d", __func__, hw->getCameraId());
}

/*===========================================================================
 * FUNCTION   : auto_focus
 *
 * DESCRIPTION: start auto focus
 *
 * PARAMETERS :
 *   @device  : ptr to camera device struct
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::auto_focus(struct camera_device *device)
{
    KPI_ATRACE_INT("Camera:AutoFocus", 1);
    int ret = NO_ERROR;
    QCamera2HardwareInterface *hw =
        reinterpret_cast<QCamera2HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("NULL camera device");
        return BAD_VALUE;
    }
    CDBG_HIGH("[KPI Perf] %s : E PROFILE_AUTO_FOCUS camera id %d",
            __func__, hw->getCameraId());
    hw->lockAPI();
    qcamera_api_result_t apiResult;
    ret = hw->processAPI(QCAMERA_SM_EVT_START_AUTO_FOCUS, NULL);
    if (ret == NO_ERROR) {
        hw->waitAPIResult(QCAMERA_SM_EVT_START_AUTO_FOCUS, &apiResult);
        ret = apiResult.status;
    }
    hw->unlockAPI();
    CDBG_HIGH("[KPI Perf] %s : X", __func__);

    return ret;
}

/*===========================================================================
 * FUNCTION   : cancel_auto_focus
 *
 * DESCRIPTION: cancel auto focus
 *
 * PARAMETERS :
 *   @device  : ptr to camera device struct
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::cancel_auto_focus(struct camera_device *device)
{
    ATRACE_CALL();
    int ret = NO_ERROR;
    QCamera2HardwareInterface *hw =
        reinterpret_cast<QCamera2HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("NULL camera device");
        return BAD_VALUE;
    }
    CDBG_HIGH("[KPI Perf] %s : E PROFILE_CANCEL_AUTO_FOCUS camera id %d",
            __func__, hw->getCameraId());
    hw->lockAPI();
    qcamera_api_result_t apiResult;
    ret = hw->processAPI(QCAMERA_SM_EVT_STOP_AUTO_FOCUS, NULL);
    if (ret == NO_ERROR) {
        hw->waitAPIResult(QCAMERA_SM_EVT_STOP_AUTO_FOCUS, &apiResult);
        ret = apiResult.status;
    }
    hw->unlockAPI();
    CDBG_HIGH("[KPI Perf] %s : X", __func__);
    return ret;
}

/*===========================================================================
 * FUNCTION   : pre_take_picture
 *
 * DESCRIPTION: pre take picture, restart preview if necessary.
 *
 * PARAMETERS :
 *   @device  : ptr to camera device struct
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::pre_take_picture(struct camera_device *device)
{
    ATRACE_CALL();
    int ret = NO_ERROR;
    QCamera2HardwareInterface *hw =
        reinterpret_cast<QCamera2HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("NULL camera device");
        return BAD_VALUE;
    }
    ALOGI("[KPI Perf] %s: E PROFILE_PRE_TAKE_PICTURE camera id %d",
            __func__, hw->getCameraId());
    hw->lockAPI();
    qcamera_api_result_t apiResult;
    ret = hw->processAPI(QCAMERA_SM_EVT_PRE_TAKE_PICTURE, NULL);
    if (ret == NO_ERROR) {
        hw->waitAPIResult(QCAMERA_SM_EVT_PRE_TAKE_PICTURE, &apiResult);
        ret = apiResult.status;
    }
    hw->unlockAPI();
    ALOGI("[KPI Perf] %s: X", __func__);
    return ret;
}

/*===========================================================================
 * FUNCTION   : take_picture
 *
 * DESCRIPTION: take picture
 *
 * PARAMETERS :
 *   @device  : ptr to camera device struct
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::take_picture(struct camera_device *device)
{
    KPI_ATRACE_CALL();
    int ret = NO_ERROR;
    QCamera2HardwareInterface *hw =
        reinterpret_cast<QCamera2HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("NULL camera device");
        return BAD_VALUE;
    }
    ALOGI("[KPI Perf] %s: E PROFILE_TAKE_PICTURE camera id %d",
            __func__, hw->getCameraId());
    qcamera_api_result_t apiResult;

   /** Added support for Retro-active Frames:
     *  takePicture() is called before preparing Snapshot to indicate the
     *  mm-camera-channel to pick up legacy frames even
     *  before LED estimation is triggered.
     */

    CDBG_HIGH("%s: numRetroSnap %d, isLiveSnap %d, isZSL %d, isHDR %d",
       __func__, hw->mParameters.getNumOfRetroSnapshots(),
       hw->isLiveSnapshot(), hw->isZSLMode(), hw->isHDRMode());

    // Check for Retro-active Frames
    if ((hw->mParameters.getNumOfRetroSnapshots() > 0) &&
        !hw->isLiveSnapshot() && hw->isZSLMode() &&
        !hw->isHDRMode() && !hw->isLongshotEnabled() &&
        (hw->getRelatedCamSyncInfo()->sync_control != CAM_SYNC_RELATED_SENSORS_ON)) {
        // Set Retro Picture Mode
        hw->setRetroPicture(1);
        hw->m_bLedAfAecLock = 0;
        CDBG_HIGH("%s: [ZSL Retro] mode", __func__);

        // Give HWI control to call pre_take_picture in single camera mode.
        // In dual-cam mode, this control belongs to muxer.
        if (hw->getRelatedCamSyncInfo()->sync_control != CAM_SYNC_RELATED_SENSORS_ON) {
            ret = pre_take_picture(device);
            if (ret != NO_ERROR) {
                ALOGE("%s: pre_take_picture failed with ret = %d",__func__,ret);
                return ret;
            }
        }

        /* Call take Picture for total number of snapshots required.
             This includes the number of retro frames and normal frames */
        hw->lockAPI();
        ret = hw->processAPI(QCAMERA_SM_EVT_TAKE_PICTURE, NULL);
        if (ret == NO_ERROR) {
          // Wait for retro frames, before calling prepare snapshot
          CDBG_HIGH("%s:[ZSL Retro] Wait for Retro frames to be done", __func__);
          hw->waitAPIResult(QCAMERA_SM_EVT_TAKE_PICTURE, &apiResult);
            ret = apiResult.status;
        }
        /* Unlock API since it is acquired in prepare snapshot seperately */
        hw->unlockAPI();

        /* Prepare snapshot in case LED needs to be flashed */
        CDBG_HIGH("%s: [ZSL Retro]  Start Prepare Snapshot", __func__);
        ret = hw->prepare_snapshot(device);
    }
    else {
        hw->setRetroPicture(0);
        // Check if prepare snapshot is done
        if (!hw->mPrepSnapRun) {
            // Ignore the status from prepare_snapshot
            hw->prepare_snapshot(device);
        }

        // Give HWI control to call pre_take_picture in single camera mode.
        // In dual-cam mode, this control belongs to muxer.
        if (hw->getRelatedCamSyncInfo()->sync_control != CAM_SYNC_RELATED_SENSORS_ON) {
            ret = pre_take_picture(device);
            if (ret != NO_ERROR) {
                ALOGE("%s: pre_take_picture failed with ret = %d",__func__,ret);
                return ret;
            }
        }

        // Regardless what the result value for prepare_snapshot,
        // go ahead with capture anyway. Just like the way autofocus
        // is handled in capture case
        /* capture */
        CDBG_HIGH("%s: [ZSL Retro] Capturing normal frames", __func__);
        hw->lockAPI();
        ret = hw->processAPI(QCAMERA_SM_EVT_TAKE_PICTURE, NULL);
        if (ret == NO_ERROR) {
          hw->waitAPIResult(QCAMERA_SM_EVT_TAKE_PICTURE, &apiResult);
            ret = apiResult.status;
        }
        hw->unlockAPI();
        if (!hw->isLongshotEnabled()){
            // For longshot mode, we prepare snapshot only once
            hw->mPrepSnapRun = false;
         }
    }
    ALOGI("[KPI Perf] %s: X", __func__);
    return ret;
}

/*===========================================================================
 * FUNCTION   : cancel_picture
 *
 * DESCRIPTION: cancel current take picture request
 *
 * PARAMETERS :
 *   @device  : ptr to camera device struct
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::cancel_picture(struct camera_device *device)
{
    ATRACE_CALL();
    int ret = NO_ERROR;
    QCamera2HardwareInterface *hw =
        reinterpret_cast<QCamera2HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("NULL camera device");
        return BAD_VALUE;
    }
    CDBG_HIGH("[KPI Perf] %s: E PROFILE_CANCEL_PICTURE camera id %d",
            __func__, hw->getCameraId());
    hw->lockAPI();
    qcamera_api_result_t apiResult;
    ret = hw->processAPI(QCAMERA_SM_EVT_CANCEL_PICTURE, NULL);
    if (ret == NO_ERROR) {
        hw->waitAPIResult(QCAMERA_SM_EVT_CANCEL_PICTURE, &apiResult);
        ret = apiResult.status;
    }
    hw->unlockAPI();
    CDBG_HIGH("[KPI Perf] %s: X camera id %d", __func__, hw->getCameraId());

    return ret;
}

/*===========================================================================
 * FUNCTION   : set_parameters
 *
 * DESCRIPTION: set camera parameters
 *
 * PARAMETERS :
 *   @device  : ptr to camera device struct
 *   @parms   : string of packed parameters
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::set_parameters(struct camera_device *device,
                                              const char *parms)
{
    ATRACE_CALL();
    int ret = NO_ERROR;
    QCamera2HardwareInterface *hw =
        reinterpret_cast<QCamera2HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("NULL camera device");
        return BAD_VALUE;
    }
    CDBG("%s: E camera id %d", __func__, hw->getCameraId());
    hw->lockAPI();
    qcamera_api_result_t apiResult;
    ret = hw->processAPI(QCAMERA_SM_EVT_SET_PARAMS, (void *)parms);
    if (ret == NO_ERROR) {
        hw->waitAPIResult(QCAMERA_SM_EVT_SET_PARAMS, &apiResult);
        ret = apiResult.status;
    }

    // Give HWI control to commit parameters (restart if necessary) only
    // in single camera mode. In dual-cam mode, this control belongs to muxer.
    if (ret == NO_ERROR &&
        hw->getRelatedCamSyncInfo()->sync_control != CAM_SYNC_RELATED_SENSORS_ON) {

        CDBG("%s: committing param change", __func__);
        ret = hw->processAPI(QCAMERA_SM_EVT_COMMIT_PARAMS, NULL);
        if (ret == NO_ERROR) {
            hw->waitAPIResult(QCAMERA_SM_EVT_COMMIT_PARAMS, &apiResult);
            ret = apiResult.status;
        }
    }

    hw->unlockAPI();
    CDBG("%s: E camera id %d", __func__, hw->getCameraId());

    return ret;
}

int QCamera2HardwareInterface::preview_restart_needed(
          struct camera_device *device, int &needRestart)
{
    ATRACE_CALL();
    int ret = NO_ERROR;
    QCamera2HardwareInterface *hw =
        reinterpret_cast<QCamera2HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("NULL camera device");
        return BAD_VALUE;
    }
    CDBG("%s: E camera id %d", __func__, hw->getCameraId());

    hw->lockAPI();
    qcamera_api_result_t apiResult;
    ret = hw->processAPI(QCAMERA_SM_EVT_PREVIEW_RESTART_NEENED, NULL);
    if (ret == NO_ERROR) {
        hw->waitAPIResult(QCAMERA_SM_EVT_PREVIEW_RESTART_NEENED, &apiResult);
        needRestart = apiResult.enabled;
        ret = apiResult.status;
    }
    hw->unlockAPI();
    CDBG("%s: X camera id %d", __func__, hw->getCameraId());

    return ret;
}

/*===========================================================================
 * FUNCTION   : commit_parameters_stop_preview
 *
 * DESCRIPTION: commit camera parameters, stop preview if necessary
 *
 * PARAMETERS :
 *   @device  : ptr to camera device struct
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::commit_parameters_stop_preview(
                     struct camera_device *device, int needRestart)
{
    ATRACE_CALL();
    int ret = NO_ERROR;
    QCamera2HardwareInterface *hw =
        reinterpret_cast<QCamera2HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("NULL camera device");
        return BAD_VALUE;
    }
    CDBG("%s: E camera id %d", __func__, hw->getCameraId());
    hw->lockAPI();
    qcamera_api_result_t apiResult;

    if (hw->getRelatedCamSyncInfo()->sync_control == CAM_SYNC_RELATED_SENSORS_ON) {
        ret = hw->processAPI(QCAMERA_SM_EVT_COMMIT_STOP_PREVIEW, (void *)(&needRestart));
        if (ret == NO_ERROR) {
            hw->waitAPIResult(QCAMERA_SM_EVT_COMMIT_STOP_PREVIEW, &apiResult);
            ret = apiResult.status;
        }
    } else {
        ALOGE("%s is not supposed to be called in single-camera mode", __func__);
        ret = INVALID_OPERATION;
    }

    hw->unlockAPI();
    CDBG("%s: X camera id %d", __func__, hw->getCameraId());

    return ret;
}

/*===========================================================================
 * FUNCTION   : commit_parameters_start_preview
 *
 * DESCRIPTION: commit camera parameters, start preview if necessary
 *
 * PARAMETERS :
 *   @device  : ptr to camera device struct
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::commit_parameters_start_preview(
                 struct camera_device *device, int needRestart)
{
    ATRACE_CALL();
    int ret = NO_ERROR;
    QCamera2HardwareInterface *hw =
        reinterpret_cast<QCamera2HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("NULL camera device");
        return BAD_VALUE;
    }
    CDBG("%s: E camera id %d", __func__, hw->getCameraId());
    hw->lockAPI();
    qcamera_api_result_t apiResult;

    if (hw->getRelatedCamSyncInfo()->sync_control == CAM_SYNC_RELATED_SENSORS_ON) {
        ret = hw->processAPI(QCAMERA_SM_EVT_COMMIT_START_PREVIEW, (void *)(&needRestart));
        if (ret == NO_ERROR) {
            hw->waitAPIResult(QCAMERA_SM_EVT_COMMIT_START_PREVIEW, &apiResult);
            ret = apiResult.status;
        }
    } else {
        ALOGE("%s is not supposed to be called in single-camera mode", __func__);
        ret = INVALID_OPERATION;
    }

    hw->unlockAPI();
    CDBG("%s: X camera id %d", __func__, hw->getCameraId());

    return ret;
}

/*===========================================================================
 * FUNCTION   : get_parameters
 *
 * DESCRIPTION: query camera parameters
 *
 * PARAMETERS :
 *   @device  : ptr to camera device struct
 *
 * RETURN     : packed parameters in a string
 *==========================================================================*/
char* QCamera2HardwareInterface::get_parameters(struct camera_device *device)
{
    ATRACE_CALL();
    char *ret = NULL;
    QCamera2HardwareInterface *hw =
        reinterpret_cast<QCamera2HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("NULL camera device");
        return NULL;
    }
    CDBG("%s: E camera id %d", __func__, hw->getCameraId());
    hw->lockAPI();
    qcamera_api_result_t apiResult;
    int32_t rc = hw->processAPI(QCAMERA_SM_EVT_GET_PARAMS, NULL);
    if (rc == NO_ERROR) {
        hw->waitAPIResult(QCAMERA_SM_EVT_GET_PARAMS, &apiResult);
        ret = apiResult.params;
    }
    hw->unlockAPI();
    CDBG("%s: E camera id %d", __func__, hw->getCameraId());

    return ret;
}

/*===========================================================================
 * FUNCTION   : put_parameters
 *
 * DESCRIPTION: return camera parameters string back to HAL
 *
 * PARAMETERS :
 *   @device  : ptr to camera device struct
 *   @parm    : ptr to parameter string to be returned
 *
 * RETURN     : none
 *==========================================================================*/
void QCamera2HardwareInterface::put_parameters(struct camera_device *device,
                                               char *parm)
{
    ATRACE_CALL();
    QCamera2HardwareInterface *hw =
        reinterpret_cast<QCamera2HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("NULL camera device");
        return;
    }
    CDBG("%s: E camera id %d", __func__, hw->getCameraId());
    hw->lockAPI();
    qcamera_api_result_t apiResult;
    int32_t ret = hw->processAPI(QCAMERA_SM_EVT_PUT_PARAMS, (void *)parm);
    if (ret == NO_ERROR) {
        hw->waitAPIResult(QCAMERA_SM_EVT_PUT_PARAMS, &apiResult);
    }
    hw->unlockAPI();
    CDBG("%s: E camera id %d", __func__, hw->getCameraId());
}

/*===========================================================================
 * FUNCTION   : send_command
 *
 * DESCRIPTION: command to be executed
 *
 * PARAMETERS :
 *   @device  : ptr to camera device struct
 *   @cmd     : cmd to be executed
 *   @arg1    : ptr to optional argument1
 *   @arg2    : ptr to optional argument2
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::send_command(struct camera_device *device,
                                            int32_t cmd,
                                            int32_t arg1,
                                            int32_t arg2)
{
    ATRACE_CALL();
    int ret = NO_ERROR;
    QCamera2HardwareInterface *hw =
        reinterpret_cast<QCamera2HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("NULL camera device");
        return BAD_VALUE;
    }
    CDBG("%s: E camera id %d", __func__, hw->getCameraId());

    qcamera_sm_evt_command_payload_t payload;
    memset(&payload, 0, sizeof(qcamera_sm_evt_command_payload_t));
    payload.cmd = cmd;
    payload.arg1 = arg1;
    payload.arg2 = arg2;
    hw->lockAPI();
    qcamera_api_result_t apiResult;
    ret = hw->processAPI(QCAMERA_SM_EVT_SEND_COMMAND, (void *)&payload);
    if (ret == NO_ERROR) {
        hw->waitAPIResult(QCAMERA_SM_EVT_SEND_COMMAND, &apiResult);
        ret = apiResult.status;
    }
    hw->unlockAPI();
    CDBG("%s: E camera id %d", __func__, hw->getCameraId());

    return ret;
}

/*===========================================================================
 * FUNCTION   : send_command_restart
 *
 * DESCRIPTION: restart if necessary after a send_command
 *
 * PARAMETERS :
 *   @device  : ptr to camera device struct
 *   @cmd     : cmd to be executed
 *   @arg1    : ptr to optional argument1
 *   @arg2    : ptr to optional argument2
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::send_command_restart(struct camera_device *device,
        int32_t cmd,
        int32_t arg1,
        int32_t arg2)
{
    ATRACE_CALL();
    int ret = NO_ERROR;
    QCamera2HardwareInterface *hw =
            reinterpret_cast<QCamera2HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("NULL camera device");
        return BAD_VALUE;
    }

    qcamera_sm_evt_command_payload_t payload;
    memset(&payload, 0, sizeof(qcamera_sm_evt_command_payload_t));
    payload.cmd = cmd;
    payload.arg1 = arg1;
    payload.arg2 = arg2;
    hw->lockAPI();
    qcamera_api_result_t apiResult;
    ret = hw->processAPI(QCAMERA_SM_EVT_SEND_COMMAND_RESTART, (void *)&payload);
    if (ret == NO_ERROR) {
        hw->waitAPIResult(QCAMERA_SM_EVT_SEND_COMMAND_RESTART, &apiResult);
        ret = apiResult.status;
    }
    hw->unlockAPI();
    CDBG("%s: E camera id %d", __func__, hw->getCameraId());

    return ret;
}

/*===========================================================================
 * FUNCTION   : release
 *
 * DESCRIPTION: release camera resource
 *
 * PARAMETERS :
 *   @device  : ptr to camera device struct
 *
 * RETURN     : none
 *==========================================================================*/
void QCamera2HardwareInterface::release(struct camera_device *device)
{
    ATRACE_CALL();
    QCamera2HardwareInterface *hw =
        reinterpret_cast<QCamera2HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("NULL camera device");
        return;
    }
    CDBG("%s: E camera id %d", __func__, hw->getCameraId());
    hw->lockAPI();
    qcamera_api_result_t apiResult;
    int32_t ret = hw->processAPI(QCAMERA_SM_EVT_RELEASE, NULL);
    if (ret == NO_ERROR) {
        hw->waitAPIResult(QCAMERA_SM_EVT_RELEASE, &apiResult);
    }
    hw->unlockAPI();
    CDBG("%s: E camera id %d", __func__, hw->getCameraId());
}

/*===========================================================================
 * FUNCTION   : dump
 *
 * DESCRIPTION: dump camera status
 *
 * PARAMETERS :
 *   @device  : ptr to camera device struct
 *   @fd      : fd for status to be dumped to
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::dump(struct camera_device *device, int fd)
{
    int ret = NO_ERROR;

    //Log level property is read when "adb shell dumpsys media.camera" is
    //called so that the log level can be controlled without restarting
    //media server
    getLogLevel();
    QCamera2HardwareInterface *hw =
        reinterpret_cast<QCamera2HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("NULL camera device");
        return BAD_VALUE;
    }
    CDBG("%s: E camera id %d", __func__, hw->getCameraId());
    hw->lockAPI();
    qcamera_api_result_t apiResult;
    ret = hw->processAPI(QCAMERA_SM_EVT_DUMP, (void *)&fd);
    if (ret == NO_ERROR) {
        hw->waitAPIResult(QCAMERA_SM_EVT_DUMP, &apiResult);
        ret = apiResult.status;
    }
    hw->unlockAPI();
    CDBG("%s: E camera id %d", __func__, hw->getCameraId());

    return ret;
}

/*===========================================================================
 * FUNCTION   : close_camera_device
 *
 * DESCRIPTION: close camera device
 *
 * PARAMETERS :
 *   @device  : ptr to camera device struct
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::close_camera_device(hw_device_t *hw_dev)
{
    KPI_ATRACE_CALL();
    int ret = NO_ERROR;
    QCamera2HardwareInterface *hw =
        reinterpret_cast<QCamera2HardwareInterface *>(
            reinterpret_cast<camera_device_t *>(hw_dev)->priv);
    if (!hw) {
        ALOGE("%s: NULL camera device", __func__);
        return BAD_VALUE;
    }
    ALOGI("[KPI Perf] %s: E camera id %d",__func__, hw->getCameraId());
    delete hw;
    ALOGI("[KPI Perf] %s: X",__func__);
    return ret;
}

/*===========================================================================
 * FUNCTION   : register_face_image
 *
 * DESCRIPTION: register a face image into imaging lib for face authenticatio/
 *              face recognition
 *
 * PARAMETERS :
 *   @device  : ptr to camera device struct
 *   @img_ptr : ptr to image buffer
 *   @config  : ptr to config about input image, i.e., format, dimension, and etc.
 *
 * RETURN     : >=0 unique ID of face registerd.
 *              <0  failure.
 *==========================================================================*/
int QCamera2HardwareInterface::register_face_image(struct camera_device *device,
                                                   void *img_ptr,
                                                   cam_pp_offline_src_config_t *config)
{
    ATRACE_CALL();
    int ret = NO_ERROR;
    QCamera2HardwareInterface *hw =
        reinterpret_cast<QCamera2HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("NULL camera device");
        return BAD_VALUE;
    }
    CDBG("%s: E camera id %d", __func__, hw->getCameraId());
    qcamera_sm_evt_reg_face_payload_t payload;
    memset(&payload, 0, sizeof(qcamera_sm_evt_reg_face_payload_t));
    payload.img_ptr = img_ptr;
    payload.config = config;
    hw->lockAPI();
    qcamera_api_result_t apiResult;
    ret = hw->processAPI(QCAMERA_SM_EVT_REG_FACE_IMAGE, (void *)&payload);
    if (ret == NO_ERROR) {
        hw->waitAPIResult(QCAMERA_SM_EVT_REG_FACE_IMAGE, &apiResult);
        ret = apiResult.handle;
    }
    hw->unlockAPI();
    CDBG("%s: E camera id %d", __func__, hw->getCameraId());

    return ret;
}

/*===========================================================================
 * FUNCTION   : prepare_snapshot
 *
 * DESCRIPTION: prepares hardware for snapshot
 *
 * PARAMETERS :
 *   @device  : ptr to camera device struct
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::prepare_snapshot(struct camera_device *device)
{
    ATRACE_CALL();
    int ret = NO_ERROR;
    QCamera2HardwareInterface *hw =
        reinterpret_cast<QCamera2HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("NULL camera device");
        return BAD_VALUE;
    }
    if (hw->isLongshotEnabled() && hw->mPrepSnapRun == true) {
        // For longshot mode, we prepare snapshot only once
        ALOGI("%s: prepare snapshot only once ", __func__);
        return NO_ERROR;
    }
    ALOGI("[KPI Perf] %s: E PROFILE_PREPARE_SNAPSHOT camera id %d",
            __func__, hw->getCameraId());
    hw->lockAPI();
    qcamera_api_result_t apiResult;

    /* Prepare snapshot in case LED needs to be flashed */
    if (hw->mFlashNeeded || hw->mParameters.isChromaFlashEnabled()) {
        CDBG_HIGH("%s: mFlashNeeded: %d ChromaFlash :%d",
                __func__, hw->mFlashNeeded,
                hw->mParameters.isChromaFlashEnabled());
        /* Prepare snapshot in case LED needs to be flashed */
        ret = hw->processAPI(QCAMERA_SM_EVT_PREPARE_SNAPSHOT, NULL);
        if (ret == NO_ERROR) {
          hw->waitAPIResult(QCAMERA_SM_EVT_PREPARE_SNAPSHOT, &apiResult);
            ret = apiResult.status;
        }
        hw->mPrepSnapRun = true;
    }
    hw->unlockAPI();
    ALOGI("[KPI Perf] %s: X, ret: %d", __func__, ret);
    return ret;
}

/*===========================================================================
 * FUNCTION   : QCamera2HardwareInterface
 *
 * DESCRIPTION: constructor of QCamera2HardwareInterface
 *
 * PARAMETERS :
 *   @cameraId  : camera ID
 *
 * RETURN     : none
 *==========================================================================*/
QCamera2HardwareInterface::QCamera2HardwareInterface(uint32_t cameraId)
    : mCameraId(cameraId),
      mCameraHandle(NULL),
      mCameraOpened(false),
      m_bRelCamCalibValid(false),
      mPreviewWindow(NULL),
      mMsgEnabled(0),
      mStoreMetaDataInFrame(0),
      mJpegCb(NULL),
      mCallbackCookie(NULL),
      mJpegCallbackCookie(NULL),
      m_bMpoEnabled(TRUE),
      m_stateMachine(this),
      m_smThreadActive(true),
      m_postprocessor(this),
      m_thermalAdapter(QCameraThermalAdapter::getInstance()),
      m_cbNotifier(this),
      m_bPreviewStarted(false),
      m_bRecordStarted(false),
      m_currentFocusState(CAM_AF_STATE_INACTIVE),
      m_pPowerModule(NULL),
      mDumpFrmCnt(0U),
      mDumpSkipCnt(0U),
      mThermalLevel(QCAMERA_THERMAL_NO_ADJUSTMENT),
      mActiveAF(false),
      m_HDRSceneEnabled(false),
      mLongshotEnabled(false),
      m_max_pic_width(0),
      m_max_pic_height(0),
      mLiveSnapshotThread(0),
      mIntPicThread(0),
      mFlashNeeded(false),
      mDeviceRotation(0U),
      mCaptureRotation(0U),
      mJpegExifRotation(0U),
      mUseJpegExifRotation(false),
      mIs3ALocked(false),
      mPrepSnapRun(false),
      mZoomLevel(0),
      mPreviewRestartNeeded(false),
      mVFrameCount(0),
      mVLastFrameCount(0),
      mVLastFpsTime(0),
      mVFps(0),
      mPFrameCount(0),
      mPLastFrameCount(0),
      mPLastFpsTime(0),
      mPFps(0),
      mInstantAecFrameCount(0),
      m_bIntJpegEvtPending(false),
      m_bIntRawEvtPending(false),
      mPostviewJob(0),
      mReprocJob(0),
      mJpegJob(0),
      mMetadataAllocJob(0),
      mInitPProcJob(0),
      mParamAllocJob(0),
      mParamInitJob(0),
      mOutputCount(0),
      mInputCount(0),
      mAdvancedCaptureConfigured(false),
      mHDRBracketingEnabled(false),
      mNumPreviewFaces(-1),
      mJpegClientHandle(0),
      mJpegHandleOwner(false),
      mCACDoneReceived(false),
      mMetadataMem(NULL),
      mBootToMonoTimestampOffset(0)
{
#ifdef TARGET_TS_MAKEUP
    memset(&mFaceRect, -1, sizeof(mFaceRect));
#endif
    getLogLevel();
    ATRACE_CALL();
    mCameraDevice.common.tag = HARDWARE_DEVICE_TAG;
    mCameraDevice.common.version = HARDWARE_DEVICE_API_VERSION(1, 0);
    mCameraDevice.common.close = close_camera_device;
    mCameraDevice.ops = &mCameraOps;
    mCameraDevice.priv = this;

    pthread_mutex_init(&m_lock, NULL);
    pthread_cond_init(&m_cond, NULL);

    m_apiResultList = NULL;

    pthread_mutex_init(&m_evtLock, NULL);
    pthread_cond_init(&m_evtCond, NULL);
    memset(&m_evtResult, 0, sizeof(qcamera_api_result_t));

    pthread_mutex_init(&m_parm_lock, NULL);

    pthread_mutex_init(&m_int_lock, NULL);
    pthread_cond_init(&m_int_cond, NULL);

    memset(m_channels, 0, sizeof(m_channels));

    memset(&mExifParams, 0, sizeof(mm_jpeg_exif_params_t));

    memset(m_BackendFileName, 0, QCAMERA_MAX_FILEPATH_LENGTH);

#ifdef HAS_MULTIMEDIA_HINTS
    if (hw_get_module(POWER_HARDWARE_MODULE_ID, (const hw_module_t **)&m_pPowerModule)) {
        ALOGE("%s: %s module not found", __func__, POWER_HARDWARE_MODULE_ID);
    }
#endif

    memset(mDefOngoingJobs, 0, sizeof(mDefOngoingJobs));
    memset(&mJpegMetadata, 0, sizeof(mJpegMetadata));
    memset(&mJpegHandle, 0, sizeof(mJpegHandle));
    memset(&mJpegMpoHandle, 0, sizeof(mJpegMpoHandle));

    mDeferredWorkThread.launch(deferredWorkRoutine, this);
    mDeferredWorkThread.sendCmd(CAMERA_CMD_TYPE_START_DATA_PROC, FALSE, FALSE);

    //Load and read GPU library.
    lib_surface_utils = NULL;
    LINK_get_surface_pixel_alignment = NULL;
    mSurfaceStridePadding = CAM_PAD_TO_32;
    lib_surface_utils = dlopen("libadreno_utils.so", RTLD_NOW);
    if (lib_surface_utils) {
        *(void **)&LINK_get_surface_pixel_alignment =
                dlsym(lib_surface_utils, "get_gpu_pixel_alignment");
         if (LINK_get_surface_pixel_alignment) {
             mSurfaceStridePadding = LINK_get_surface_pixel_alignment();
         }
         dlclose(lib_surface_utils);
    }

    pthread_mutex_init(&mGrallocLock, NULL);
    mEnqueuedBuffers = 0;
}

/*===========================================================================
 * FUNCTION   : ~QCamera2HardwareInterface
 *
 * DESCRIPTION: destructor of QCamera2HardwareInterface
 *
 * PARAMETERS : none
 *
 * RETURN     : none
 *==========================================================================*/
QCamera2HardwareInterface::~QCamera2HardwareInterface()
{
    CDBG_HIGH("%s: E", __func__);
    mDeferredWorkThread.sendCmd(CAMERA_CMD_TYPE_STOP_DATA_PROC, TRUE, TRUE);
    mDeferredWorkThread.exit();

    if (mMetadataMem != NULL) {
        delete mMetadataMem;
        mMetadataMem = NULL;
    }

    lockAPI();
    m_smThreadActive = false;
    unlockAPI();
    m_stateMachine.releaseThread();
    closeCamera();
    pthread_mutex_destroy(&m_lock);
    pthread_cond_destroy(&m_cond);
    pthread_mutex_destroy(&m_evtLock);
    pthread_cond_destroy(&m_evtCond);
    pthread_mutex_destroy(&m_parm_lock);
    pthread_mutex_destroy(&m_int_lock);
    pthread_cond_destroy(&m_int_cond);
    pthread_mutex_destroy(&mGrallocLock);
    CDBG_HIGH("%s: X", __func__);
}

/*===========================================================================
 * FUNCTION   : deferPPInit
 *
 * DESCRIPTION: Queue postproc init task to deferred thread
 *
 * PARAMETERS : none
 *
 * RETURN     : uint32_t job id of pproc init job
 *              0  -- failure
 *==========================================================================*/
uint32_t QCamera2HardwareInterface::deferPPInit()
{
    // init pproc
    DeferWorkArgs args;
    DeferPProcInitArgs pprocInitArgs;

    memset(&args, 0, sizeof(DeferWorkArgs));
    memset(&pprocInitArgs, 0, sizeof(DeferPProcInitArgs));

    pprocInitArgs.jpeg_cb = jpegEvtHandle;
    pprocInitArgs.user_data = this;
    args.pprocInitArgs = pprocInitArgs;

    return queueDeferredWork(CMD_DEF_PPROC_INIT,
            args);
}

/*===========================================================================
 * FUNCTION   : openCamera
 *
 * DESCRIPTION: open camera
 *
 * PARAMETERS :
 *   @hw_device  : double ptr for camera device struct
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::openCamera(struct hw_device_t **hw_device)
{
    KPI_ATRACE_CALL();
    int rc = NO_ERROR;
    if (mCameraOpened) {
        *hw_device = NULL;
        return PERMISSION_DENIED;
    }
    ALOGI("[KPI Perf] %s: E PROFILE_OPEN_CAMERA camera id %d",
        __func__,mCameraId);
    rc = openCamera();
    if (rc == NO_ERROR){
        *hw_device = &mCameraDevice.common;
        if (m_thermalAdapter.init(this) != 0) {
          ALOGE("Init thermal adapter failed");
        }
    }
    else
        *hw_device = NULL;

    ALOGI("[KPI Perf] %s: X PROFILE_OPEN_CAMERA camera id %d, rc: %d",
        __func__,mCameraId, rc);

    return rc;
}

/*===========================================================================
 * FUNCTION   : openCamera
 *
 * DESCRIPTION: open camera
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::openCamera()
{
    size_t i;
    int32_t rc = NO_ERROR;
    char value[PROPERTY_VALUE_MAX];

    if (mCameraHandle) {
        ALOGE("Failure: Camera already opened");
        return ALREADY_EXISTS;
    }

    // alloc param buffer
    DeferWorkArgs args;
    memset(&args, 0, sizeof(args));
    mParamAllocJob = queueDeferredWork(CMD_DEF_PARAM_ALLOC, args);

    if (gCamCapability[mCameraId] != NULL) {
        // allocate metadata buffers
        DeferWorkArgs args;
        DeferMetadataAllocArgs metadataAllocArgs;

        memset(&args, 0, sizeof(args));
        memset(&metadataAllocArgs, 0, sizeof(metadataAllocArgs));

        uint32_t padding =
                gCamCapability[mCameraId]->padding_info.plane_padding;
        metadataAllocArgs.size = PAD_TO_SIZE(sizeof(metadata_buffer_t),
                padding);
        metadataAllocArgs.bufferCnt = CAMERA_MIN_METADATA_BUFFERS;
        args.metadataAllocArgs = metadataAllocArgs;

        mMetadataAllocJob = queueDeferredWork(CMD_DEF_METADATA_ALLOC, args);
        if (mMetadataAllocJob == 0) {
            CDBG_HIGH("%s: Failed to allocate param buffer", __func__);
        }

        rc = camera_open((uint8_t)mCameraId, &mCameraHandle);
        if (rc) {
            ALOGE("camera_open failed. rc = %d, mCameraHandle = %p", rc, mCameraHandle);
            return rc;
        }

        mCameraHandle->ops->register_event_notify(mCameraHandle->camera_handle,
                camEvtHandle,
                (void *) this);
    } else {
        CDBG_HIGH("%s: Capabilities are not initialized. Initializing them now.", __func__);

        rc = camera_open((uint8_t)mCameraId, &mCameraHandle);
        if (rc) {
            ALOGE("camera_open failed. rc = %d, mCameraHandle = %p", rc, mCameraHandle);
            return rc;
        }

        if(NO_ERROR != initCapabilities(mCameraId,mCameraHandle)) {
            ALOGE("initCapabilities failed.");
            rc = UNKNOWN_ERROR;
            goto error_exit;
        }

        mCameraHandle->ops->register_event_notify(mCameraHandle->camera_handle,
                camEvtHandle,
                (void *) this);
    }

    // Init params in the background
    // 1. It's safe to queue init job, even if alloc job is not yet complete.
    // It will be queued to the same thread, so the alloc is guaranteed to
    // finish first.
    // 2. However, it is not safe to begin param init until after camera is
    // open. That is why we wait until after camera open completes to schedule
    // this task.
    memset(&args, 0, sizeof(args));
    mParamInitJob = queueDeferredWork(CMD_DEF_PARAM_INIT, args);

    mCameraOpened = true;

    //Notify display HAL that a camera session is active.
    //But avoid calling the same during bootup because camera service might open/close
    //cameras at boot time during its initialization and display service will also internally
    //wait for camera service to initialize first while calling this display API, resulting in a
    //deadlock situation. Since boot time camera open/close calls are made only to fetch
    //capabilities, no need of this display bw optimization.
    //Use "service.bootanim.exit" property to know boot status.
    property_get("service.bootanim.exit", value, "0");
    if (atoi(value) == 1) {
        pthread_mutex_lock(&gCamLock);
        if (gNumCameraSessions++ == 0) {
            setCameraLaunchStatus(true);
        }
        pthread_mutex_unlock(&gCamLock);
    }

    // Setprop to decide the time source (whether boottime or monotonic).
    // By default, use monotonic time.
    property_get("persist.camera.time.monotonic", value, "1");
    mBootToMonoTimestampOffset = 0;
    if (atoi(value) == 1) {
        // if monotonic is set, then need to use time in monotonic.
        // So, Measure the clock offset between BOOTTIME and MONOTONIC
        // The clock domain source for ISP is BOOTTIME and
        // for Video/display is MONOTONIC
        // The below offset is used to convert from clock domain of other subsystem
        // (video/hardware composer) to that of camera. Assumption is that this
        // offset won't change during the life cycle of the camera device. In other
        // words, camera device shouldn't be open during CPU suspend.
        mBootToMonoTimestampOffset = getBootToMonoTimeOffset();
    }
    CDBG_HIGH("mBootToMonoTimestampOffset = %lld", mBootToMonoTimestampOffset);

    return NO_ERROR;
error_exit:
    if(mJpegClientHandle) {
        deinitJpegHandle();
    }
    mCameraHandle->ops->close_camera(mCameraHandle->camera_handle);
    mCameraHandle = NULL;
    return rc;

}

/*===========================================================================
 * FUNCTION   : bundleRelatedCameras
 *
 * DESCRIPTION: bundle cameras to enable syncing of cameras
 *
 * PARAMETERS :
 *   @sync        :indicates whether syncing is On or Off
 *   @sessionid  :session id for other camera session
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::bundleRelatedCameras(bool syncOn,
            uint32_t sessionid)
{
    CDBG("%s: bundleRelatedCameras sync %d with sessionid %d", __func__,
            syncOn, sessionid);

    pthread_mutex_lock(&m_parm_lock);
    int32_t rc = mParameters.bundleRelatedCameras(syncOn, sessionid);
    if (rc != NO_ERROR) {
        ALOGE("%s: bundleRelatedCameras failed %d", __func__, rc);
        pthread_mutex_unlock(&m_parm_lock);
        return rc;
    }
    pthread_mutex_unlock(&m_parm_lock);
    return rc;
}

/*===========================================================================
 * FUNCTION   : getCameraSessionId
 *
 * DESCRIPTION: gets the backend session Id of this HWI instance
 *
 * PARAMETERS :
 *   @sessionid  : pointer to the output session id
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::getCameraSessionId(uint32_t* session_id)
{
    int32_t rc = NO_ERROR;

    if(session_id != NULL) {
        rc = mCameraHandle->ops->get_session_id(mCameraHandle->camera_handle,
                session_id);
        CDBG("%s: Getting Camera Session Id %d", __func__, *session_id);
    } else {
        ALOGE("%s:Session Id is Null", __func__);
        return UNKNOWN_ERROR;
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : getRelatedCamSyncInfo
 *
 * DESCRIPTION:returns the related cam sync info for this HWI instance
 *
 * PARAMETERS :none
 *
 * RETURN     : const pointer to cam_sync_related_sensors_event_info_t
 *==========================================================================*/
const cam_sync_related_sensors_event_info_t*
        QCamera2HardwareInterface::getRelatedCamSyncInfo(void)
{
    return mParameters.getRelatedCamSyncInfo();
}

/*===========================================================================
 * FUNCTION   : setRelatedCamSyncInfo
 *
 * DESCRIPTION:sets the related cam sync info for this HWI instance
 *
 * PARAMETERS :
 *   @info  : ptr to related cam info parameters
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::setRelatedCamSyncInfo(
        cam_sync_related_sensors_event_info_t* info)
{
    if(info) {
        return mParameters.setRelatedCamSyncInfo(info);
    } else {
        return BAD_TYPE;
    }
}

/*===========================================================================
 * FUNCTION   : getMpoComposition
 *
 * DESCRIPTION:function to retrieve whether Mpo composition should be enabled
 *                    or not
 *
 * PARAMETERS :none
 *
 * RETURN     : bool indicates whether mpo composition is enabled or not
 *==========================================================================*/
bool QCamera2HardwareInterface::getMpoComposition(void)
{
    CDBG_HIGH("%s: MpoComposition:%d ", __func__, m_bMpoEnabled);
    return m_bMpoEnabled;
}

/*===========================================================================
 * FUNCTION   : setMpoComposition
 *
 * DESCRIPTION:set if Mpo composition should be enabled for this HWI instance
 *
 * PARAMETERS :
 *   @enable  : indicates whether Mpo composition enabled or not
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::setMpoComposition(bool enable)
{
    // By default set Mpo composition to disable
    m_bMpoEnabled = false;

    // Enable Mpo composition only if
    // 1) frame sync is ON between two cameras and
    // 2) any advanced features are not enabled (AOST features) and
    // 3) not in recording mode (for liveshot case)
    // 4) flash is not needed
    if ((getRelatedCamSyncInfo()->sync_control == CAM_SYNC_RELATED_SENSORS_ON) &&
            !mParameters.isAdvCamFeaturesEnabled() &&
            !mParameters.getRecordingHintValue() &&
            !mFlashNeeded &&
            !isLongshotEnabled()) {
        m_bMpoEnabled = enable;
        CDBG_HIGH("%s: MpoComposition:%d ", __func__, m_bMpoEnabled);
        return NO_ERROR;
    } else {
        return BAD_TYPE;
    }
}

/*===========================================================================
 * FUNCTION   : getRecordingHintValue
 *
 * DESCRIPTION:function to retrieve recording hint value
 *
 * PARAMETERS :none
 *
 * RETURN     : bool indicates whether recording hint is enabled or not
 *==========================================================================*/
bool QCamera2HardwareInterface::getRecordingHintValue(void)
{
    return mParameters.getRecordingHintValue();
}

/*===========================================================================
 * FUNCTION   : setRecordingHintValue
 *
 * DESCRIPTION:set recording hint value
 *
 * PARAMETERS :
 *   @enable  : video hint value
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::setRecordingHintValue(int32_t value)
{
    int32_t rc = NO_ERROR;
    pthread_mutex_lock(&m_parm_lock);
    rc = mParameters.updateRecordingHintValue(value);
    pthread_mutex_unlock(&m_parm_lock);
    return rc;
}

/*===========================================================================
 * FUNCTION   : closeCamera
 *
 * DESCRIPTION: close camera
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::closeCamera()
{
    int rc = NO_ERROR;
    int i;
    char value[PROPERTY_VALUE_MAX];
    CDBG_HIGH("%s: E", __func__);
    if (!mCameraOpened) {
        return NO_ERROR;
    }
    ALOGI("[KPI Perf] %s: E PROFILE_CLOSE_CAMERA camera id %d",
        __func__, mCameraId);

    pthread_mutex_lock(&m_parm_lock);

    // set open flag to false
    mCameraOpened = false;

    // Reset Stream config info
    mParameters.setStreamConfigure(false, false, true);

    // deinit Parameters
    mParameters.deinit();

    pthread_mutex_unlock(&m_parm_lock);

    // exit notifier
    m_cbNotifier.exit();

    // stop and deinit postprocessor
    waitDeferredWork(mReprocJob);
    // Close the JPEG session
    waitDeferredWork(mJpegJob);
    m_postprocessor.stop();
    deinitJpegHandle();
    m_postprocessor.deinit();
    mInitPProcJob = 0; // reset job id, so pproc can be reinited later

    m_thermalAdapter.deinit();

    // delete all channels if not already deleted
    for (i = 0; i < QCAMERA_CH_TYPE_MAX; i++) {
        if (m_channels[i] != NULL) {
            m_channels[i]->stop();
            delete m_channels[i];
            m_channels[i] = NULL;
        }
    }

    //free all pending api results here
    if(m_apiResultList != NULL) {
        api_result_list *apiResultList = m_apiResultList;
        api_result_list *apiResultListNext;
        while (apiResultList != NULL) {
            apiResultListNext = apiResultList->next;
            free(apiResultList);
            apiResultList = apiResultListNext;
        }
    }

    rc = mCameraHandle->ops->close_camera(mCameraHandle->camera_handle);
    mCameraHandle = NULL;

    //Notify display HAL that there is no active camera session
    //but avoid calling the same during bootup. Refer to openCamera
    //for more details.
    property_get("service.bootanim.exit", value, "0");
    if (atoi(value) == 1) {
        pthread_mutex_lock(&gCamLock);
        if (--gNumCameraSessions == 0) {
            setCameraLaunchStatus(false);
        }
        pthread_mutex_unlock(&gCamLock);
    }

    if (mExifParams.debug_params) {
        free(mExifParams.debug_params);
        mExifParams.debug_params = NULL;
    }
    ALOGI("[KPI Perf] %s: X PROFILE_CLOSE_CAMERA camera id %d, rc: %d",
        __func__, mCameraId, rc);

    return rc;
}

#define DATA_PTR(MEM_OBJ,INDEX) MEM_OBJ->getPtr( INDEX )

/*===========================================================================
 * FUNCTION   : initCapabilities
 *
 * DESCRIPTION: initialize camera capabilities in static data struct
 *
 * PARAMETERS :
 *   @cameraId  : camera Id
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::initCapabilities(uint32_t cameraId,
        mm_camera_vtbl_t *cameraHandle)
{
    ATRACE_CALL();
    int rc = NO_ERROR;
    QCameraHeapMemory *capabilityHeap = NULL;

    /* Allocate memory for capability buffer */
    capabilityHeap = new QCameraHeapMemory(QCAMERA_ION_USE_CACHE);
    rc = capabilityHeap->allocate(1, sizeof(cam_capability_t), NON_SECURE);
    if(rc != OK) {
        ALOGE("%s: No memory for cappability", __func__);
        goto allocate_failed;
    }

    /* Map memory for capability buffer */
    memset(DATA_PTR(capabilityHeap,0), 0, sizeof(cam_capability_t));

    cam_buf_map_type_list bufMapList;
    rc = QCameraBufferMaps::makeSingletonBufMapList(
            CAM_MAPPING_BUF_TYPE_CAPABILITY,
            0 /*stream id*/, 0 /*buffer index*/, -1 /*plane index*/,
            0 /*cookie*/, capabilityHeap->getFd(0), sizeof(cam_capability_t),
            bufMapList);

    if (rc == NO_ERROR) {
        rc = cameraHandle->ops->map_bufs(cameraHandle->camera_handle,
                &bufMapList);
    }

    if(rc < 0) {
        ALOGE("%s: failed to map capability buffer", __func__);
        goto map_failed;
    }

    /* Query Capability */
    rc = cameraHandle->ops->query_capability(cameraHandle->camera_handle);
    if(rc < 0) {
        ALOGE("%s: failed to query capability",__func__);
        goto query_failed;
    }
    gCamCapability[cameraId] =
            (cam_capability_t *)malloc(sizeof(cam_capability_t));

    if (!gCamCapability[cameraId]) {
        ALOGE("%s: out of memory", __func__);
        goto query_failed;
    }
    memcpy(gCamCapability[cameraId], DATA_PTR(capabilityHeap,0),
                                        sizeof(cam_capability_t));

    rc = NO_ERROR;

query_failed:
    cameraHandle->ops->unmap_buf(cameraHandle->camera_handle,
                            CAM_MAPPING_BUF_TYPE_CAPABILITY);
map_failed:
    capabilityHeap->deallocate();
    delete capabilityHeap;
allocate_failed:
    return rc;
}

/*===========================================================================
 * FUNCTION   : getCapabilities
 *
 * DESCRIPTION: query camera capabilities
 *
 * PARAMETERS :
 *   @cameraId  : camera Id
 *   @info      : camera info struct to be filled in with camera capabilities
 *
 * RETURN     : int type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::getCapabilities(uint32_t cameraId,
        struct camera_info *info, cam_sync_type_t *p_cam_type)
{
    ATRACE_CALL();
    int rc = NO_ERROR;
    struct  camera_info *p_info = NULL;
    pthread_mutex_lock(&gCamLock);
    p_info = get_cam_info(cameraId, p_cam_type);
    p_info->device_version = CAMERA_DEVICE_API_VERSION_1_0;
    p_info->static_camera_characteristics = NULL;
    memcpy(info, p_info, sizeof (struct camera_info));
    pthread_mutex_unlock(&gCamLock);
    return rc;
}

/*===========================================================================
 * FUNCTION   : getCamHalCapabilities
 *
 * DESCRIPTION: get the HAL capabilities structure
 *
 * PARAMETERS :
 *   @cameraId  : camera Id
 *
 * RETURN     : capability structure of respective camera
 *
 *==========================================================================*/
cam_capability_t* QCamera2HardwareInterface::getCamHalCapabilities()
{
    return gCamCapability[mCameraId];
}

/*===========================================================================
 * FUNCTION   : getBufNumRequired
 *
 * DESCRIPTION: return number of stream buffers needed for given stream type
 *
 * PARAMETERS :
 *   @stream_type  : type of stream
 *
 * RETURN     : number of buffers needed
 *==========================================================================*/
uint8_t QCamera2HardwareInterface::getBufNumRequired(cam_stream_type_t stream_type)
{
    int bufferCnt = 0;
    int minCaptureBuffers = mParameters.getNumOfSnapshots();
    char value[PROPERTY_VALUE_MAX];
    bool raw_yuv = false;
    int persist_cnt = 0;

    int zslQBuffers = mParameters.getZSLQueueDepth();

    int minCircularBufNum = mParameters.getMaxUnmatchedFramesInQueue() +
                            CAMERA_MIN_JPEG_ENCODING_BUFFERS;

    int maxStreamBuf = minCaptureBuffers + mParameters.getMaxUnmatchedFramesInQueue() +
                       mParameters.getNumOfExtraHDRInBufsIfNeeded() -
                       mParameters.getNumOfExtraHDROutBufsIfNeeded() +
                       mParameters.getNumOfExtraBuffersForImageProc() +
                       EXTRA_ZSL_PREVIEW_STREAM_BUF;

    int minUndequeCount = 0;
    if (!isNoDisplayMode()) {
        if(mPreviewWindow != NULL) {
            if (mPreviewWindow->get_min_undequeued_buffer_count(mPreviewWindow,&minUndequeCount)
                != 0) {
                ALOGE("get_min_undequeued_buffer_count  failed");
                //TODO: hardcoded because MIN_UNDEQUEUED_BUFFERS not defined
                //minUndequeCount = BufferQueue::MIN_UNDEQUEUED_BUFFERS;
                minUndequeCount = MIN_UNDEQUEUED_BUFFERS;
            }
        } else {
            //preview window might not be set at this point. So, query directly
            //from BufferQueue implementation of gralloc buffers.
            //minUndequeCount = BufferQueue::MIN_UNDEQUEUED_BUFFERS;
            //hardcoded because MIN_UNDEQUEUED_BUFFERS not defined. REVISIT
            minUndequeCount = MIN_UNDEQUEUED_BUFFERS;
        }
        if (minUndequeCount != MIN_UNDEQUEUED_BUFFERS) {
            // minUndequeCount from valid preview window != hardcoded MIN_UNDEQUEUED_BUFFERS
            // and so change the MACRO as per minUndequeCount
            ALOGE("%s: ***ERROR Case*** minUndequeCount(%d) != hardcoded value(%d)",
                    __func__, minUndequeCount, MIN_UNDEQUEUED_BUFFERS);
        }
    }

    // Get buffer count for the particular stream type
    switch (stream_type) {
    case CAM_STREAM_TYPE_PREVIEW:
        {
            if (mParameters.isZSLMode()) {
                // We need to add two extra streming buffers to add
                // flexibility in forming matched super buf in ZSL queue.
                // with number being 'zslQBuffers + minCircularBufNum'
                // we see preview buffers sometimes get dropped at CPP
                // and super buf is not forming in ZSL Q for long time.

                bufferCnt = zslQBuffers + minCircularBufNum +
                        mParameters.getNumOfExtraBuffersForImageProc() +
                        mParameters.getNumOfExtraBuffersForPreview() +
                        mParameters.getNumOfExtraHDRInBufsIfNeeded();
            } else {
                bufferCnt = CAMERA_MIN_STREAMING_BUFFERS +
                        mParameters.getMaxUnmatchedFramesInQueue() +
                        mParameters.getNumOfExtraBuffersForPreview();
            }
            // ISP allocates native preview buffers and so reducing same from HAL allocation
            if (bufferCnt > CAMERA_ISP_PING_PONG_BUFFERS )
                bufferCnt -= CAMERA_ISP_PING_PONG_BUFFERS;

            // Add the display minUndequeCount count on top of camera requirement
            bufferCnt += minUndequeCount;

            property_get("persist.camera.preview_yuv", value, "0");
            persist_cnt = atoi(value);
            if ((persist_cnt < CAM_MAX_NUM_BUFS_PER_STREAM)
                    && (bufferCnt < persist_cnt)) {
                bufferCnt = persist_cnt;
            }
        }
        break;
    case CAM_STREAM_TYPE_POSTVIEW:
        {
            bufferCnt = minCaptureBuffers +
                        mParameters.getMaxUnmatchedFramesInQueue() +
                        mParameters.getNumOfExtraHDRInBufsIfNeeded() -
                        mParameters.getNumOfExtraHDROutBufsIfNeeded() +
                        mParameters.getNumOfExtraBuffersForImageProc();

            if (bufferCnt > maxStreamBuf) {
                bufferCnt = maxStreamBuf;
            }
            bufferCnt += minUndequeCount;
        }
        break;
    case CAM_STREAM_TYPE_SNAPSHOT:
        {
            if (mParameters.isZSLMode() || mLongshotEnabled) {
                if ((minCaptureBuffers == 1 || mParameters.isUbiRefocus()) &&
                        !mLongshotEnabled) {
                    // Single ZSL snapshot case
                    bufferCnt = zslQBuffers + CAMERA_MIN_STREAMING_BUFFERS +
                            mParameters.getNumOfExtraBuffersForImageProc();
                }
                else {
                    // ZSL Burst or Longshot case
                    bufferCnt = zslQBuffers + minCircularBufNum +
                            mParameters.getNumOfExtraBuffersForImageProc();
                }
                if (getSensorType() == CAM_SENSOR_YUV && bufferCnt > CAMERA_ISP_PING_PONG_BUFFERS) {
                    //ISP allocates native buffers in YUV case
                    bufferCnt -= CAMERA_ISP_PING_PONG_BUFFERS;
                }
            } else {
                bufferCnt = minCaptureBuffers +
                            mParameters.getNumOfExtraHDRInBufsIfNeeded() -
                            mParameters.getNumOfExtraHDROutBufsIfNeeded() +
                            mParameters.getNumOfExtraBuffersForImageProc();

                if (bufferCnt > maxStreamBuf) {
                    bufferCnt = maxStreamBuf;
                }
            }
        }
        break;
    case CAM_STREAM_TYPE_RAW:
        property_get("persist.camera.raw_yuv", value, "0");
        raw_yuv = atoi(value) > 0 ? true : false;

        if (isRdiMode() || raw_yuv) {
            CDBG_HIGH("RDI_DEBUG %s[%d]: CAM_STREAM_TYPE_RAW",
              __func__, __LINE__);
            bufferCnt = zslQBuffers + minCircularBufNum;
        } else if (mParameters.isZSLMode()) {
            bufferCnt = zslQBuffers + minCircularBufNum;
            if (getSensorType() == CAM_SENSOR_YUV && bufferCnt > CAMERA_ISP_PING_PONG_BUFFERS) {
                //ISP allocates native buffers in YUV case
                bufferCnt -= CAMERA_ISP_PING_PONG_BUFFERS;
            }

        } else {
            bufferCnt = minCaptureBuffers +
                        mParameters.getNumOfExtraHDRInBufsIfNeeded() -
                        mParameters.getNumOfExtraHDROutBufsIfNeeded() +
                        mParameters.getNumOfExtraBuffersForImageProc();

            if (bufferCnt > maxStreamBuf) {
                bufferCnt = maxStreamBuf;
            }
        }

        property_get("persist.camera.preview_raw", value, "0");
        persist_cnt = atoi(value);
        if ((persist_cnt < CAM_MAX_NUM_BUFS_PER_STREAM)
                && (bufferCnt < persist_cnt)) {
            bufferCnt = persist_cnt;
        }
        property_get("persist.camera.video_raw", value, "0");
        persist_cnt = atoi(value);
        if ((persist_cnt < CAM_MAX_NUM_BUFS_PER_STREAM)
                && (bufferCnt < persist_cnt)) {
            bufferCnt = persist_cnt;
        }

        break;
    case CAM_STREAM_TYPE_VIDEO:
        {
            if (mParameters.getBufBatchCount()) {
                bufferCnt = CAMERA_MIN_VIDEO_BATCH_BUFFERS;
            } else {
                bufferCnt = CAMERA_MIN_VIDEO_BUFFERS;
            }

            bufferCnt += mParameters.getNumOfExtraBuffersForVideo();
            //if its 4K encoding usecase, then add extra buffer
            cam_dimension_t dim;
            mParameters.getStreamDimension(CAM_STREAM_TYPE_VIDEO, dim);
            if (is4k2kResolution(&dim)) {
                 //get additional buffer count
                 property_get("vidc.enc.dcvs.extra-buff-count", value, "0");
                 bufferCnt += atoi(value);
            }
            ALOGI("Buffer count is %d, width / height (%d/%d) ", bufferCnt, dim.width, dim.height);
        }
        break;
    case CAM_STREAM_TYPE_METADATA:
        {
            if (mParameters.isZSLMode()) {
                // MetaData buffers should be >= (Preview buffers-minUndequeCount)
                bufferCnt = zslQBuffers + minCircularBufNum +
                            mParameters.getNumOfExtraHDRInBufsIfNeeded() -
                            mParameters.getNumOfExtraHDROutBufsIfNeeded() +
                            mParameters.getNumOfExtraBuffersForImageProc() +
                            EXTRA_ZSL_PREVIEW_STREAM_BUF;
            } else {
                bufferCnt = minCaptureBuffers +
                            mParameters.getNumOfExtraHDRInBufsIfNeeded() -
                            mParameters.getNumOfExtraHDROutBufsIfNeeded() +
                            mParameters.getMaxUnmatchedFramesInQueue() +
                            CAMERA_MIN_STREAMING_BUFFERS +
                            mParameters.getNumOfExtraBuffersForImageProc();

                if (bufferCnt > zslQBuffers + minCircularBufNum) {
                    bufferCnt = zslQBuffers + minCircularBufNum;
                }
            }
            if (CAMERA_MIN_METADATA_BUFFERS > bufferCnt) {
                bufferCnt = CAMERA_MIN_METADATA_BUFFERS;
            }
        }
        break;
    case CAM_STREAM_TYPE_OFFLINE_PROC:
        {
            bufferCnt = minCaptureBuffers;
            // One of the ubifocus buffers is miscellaneous buffer
            if (mParameters.isUbiRefocus()) {
                bufferCnt -= 1;
            }
            if (mLongshotEnabled) {
                bufferCnt = mParameters.getLongshotStages();
            }
        }
        break;
    case CAM_STREAM_TYPE_CALLBACK:
        bufferCnt = CAMERA_MIN_CALLBACK_BUFFERS;
        break;
    case CAM_STREAM_TYPE_ANALYSIS:
    case CAM_STREAM_TYPE_DEFAULT:
    case CAM_STREAM_TYPE_MAX:
    default:
        bufferCnt = 0;
        break;
    }

    CDBG("%s: Buffer count = %d for stream type = %d",__func__, bufferCnt, stream_type);
    if (CAM_MAX_NUM_BUFS_PER_STREAM < bufferCnt) {
        ALOGE("%s: Buffer count %d for stream type %d exceeds limit %d",
                __func__, bufferCnt, stream_type, CAM_MAX_NUM_BUFS_PER_STREAM);
        return CAM_MAX_NUM_BUFS_PER_STREAM;
    }

    return (uint8_t)bufferCnt;
}

/*===========================================================================
 * FUNCTION   : allocateStreamBuf
 *
 * DESCRIPTION: alocate stream buffers
 *
 * PARAMETERS :
 *   @stream_type  : type of stream
 *   @size         : size of buffer
 *   @stride       : stride of buffer
 *   @scanline     : scanline of buffer
 *   @bufferCnt    : [IN/OUT] minimum num of buffers to be allocated.
 *                   could be modified during allocation if more buffers needed
 *
 * RETURN     : ptr to a memory obj that holds stream buffers.
 *              NULL if failed
 *==========================================================================*/
QCameraMemory *QCamera2HardwareInterface::allocateStreamBuf(
        cam_stream_type_t stream_type, size_t size, int stride, int scanline,
        uint8_t &bufferCnt)
{
    int rc = NO_ERROR;
    QCameraMemory *mem = NULL;
    bool bCachedMem = QCAMERA_ION_USE_CACHE;
    bool bPoolMem = false;
    char value[PROPERTY_VALUE_MAX];
    property_get("persist.camera.mem.usepool", value, "1");
    if (atoi(value) == 1) {
        bPoolMem = true;
    }

    // Allocate stream buffer memory object
    switch (stream_type) {
    case CAM_STREAM_TYPE_PREVIEW:
        {
            if (isNoDisplayMode()) {
                mem = new QCameraStreamMemory(mGetMemory,
                        bCachedMem,
                        (bPoolMem) ? &m_memoryPool : NULL,
                        stream_type);
            } else {
                cam_dimension_t dim;
                int minFPS, maxFPS;
                QCameraGrallocMemory *grallocMemory =
                    new QCameraGrallocMemory(mGetMemory);

                mParameters.getStreamDimension(stream_type, dim);
                /* we are interested only in maxfps here */
                mParameters.getPreviewFpsRange(&minFPS, &maxFPS);
                int usage = 0;
                if(mParameters.isUBWCEnabled()) {
                    cam_format_t fmt;
                    mParameters.getStreamFormat(CAM_STREAM_TYPE_PREVIEW,fmt);
                    if (fmt == CAM_FORMAT_YUV_420_NV12_UBWC) {
                        usage = GRALLOC_USAGE_PRIVATE_ALLOC_UBWC ;
                    }
                }
                if (grallocMemory) {
                    grallocMemory->setMappable(
                            CAMERA_INITIAL_MAPPABLE_PREVIEW_BUFFERS);
                    grallocMemory->setWindowInfo(mPreviewWindow,
                            dim.width,dim.height, stride, scanline,
                            mParameters.getPreviewHalPixelFormat(),
                            maxFPS, usage,
                            isMonoCamera() ? CAM_FORMAT_Y_ONLY : 0);
                    pthread_mutex_lock(&mGrallocLock);
                    if (bufferCnt > CAMERA_INITIAL_MAPPABLE_PREVIEW_BUFFERS) {
                        mEnqueuedBuffers = (bufferCnt -
                                CAMERA_INITIAL_MAPPABLE_PREVIEW_BUFFERS);
                    } else {
                        mEnqueuedBuffers = 0;
                    }
                    pthread_mutex_unlock(&mGrallocLock);
                }
                mem = grallocMemory;
            }
        }
        break;
    case CAM_STREAM_TYPE_POSTVIEW:
        {
            if (isNoDisplayMode() || isPreviewRestartEnabled()) {
                mem = new QCameraStreamMemory(mGetMemory, bCachedMem);
            } else {
                cam_dimension_t dim;
                int minFPS, maxFPS;
                QCameraGrallocMemory *grallocMemory =
                        new QCameraGrallocMemory(mGetMemory);

                mParameters.getStreamDimension(stream_type, dim);
                /* we are interested only in maxfps here */
                mParameters.getPreviewFpsRange(&minFPS, &maxFPS);
                if (grallocMemory) {
                    grallocMemory->setWindowInfo(mPreviewWindow, dim.width,
                            dim.height, stride, scanline,
                            mParameters.getPreviewHalPixelFormat(), maxFPS);
                }
                mem = grallocMemory;
            }
        }
        break;
    case CAM_STREAM_TYPE_ANALYSIS:
    case CAM_STREAM_TYPE_SNAPSHOT:
    case CAM_STREAM_TYPE_RAW:
    case CAM_STREAM_TYPE_OFFLINE_PROC:
        mem = new QCameraStreamMemory(mGetMemory,
                bCachedMem,
                (bPoolMem) ? &m_memoryPool : NULL,
                stream_type);
        break;
    case CAM_STREAM_TYPE_METADATA:
        {
            if (mMetadataMem == NULL) {
                mem = new QCameraMetadataStreamMemory(QCAMERA_ION_USE_CACHE);
            } else {
                mem = mMetadataMem;
                mMetadataMem = NULL;

                int32_t numAdditionalBuffers = bufferCnt - mem->getCnt();
                if (numAdditionalBuffers > 0) {
                    rc = mem->allocateMore(numAdditionalBuffers, size);
                    if (rc != NO_ERROR) {
                        ALOGE("%s: Failed to allocate additional buffers, "
                                "but attempting to proceed.", __func__);
                    }
                }
                bufferCnt = mem->getCnt();
                // The memory is already allocated  and initialized, so
                // simply return here.
                return mem;
            }
        }
        break;
    case CAM_STREAM_TYPE_VIDEO:
        {
            //Use uncached allocation by default
            if (mParameters.isVideoBuffersCached() || mParameters.isSeeMoreEnabled() ||
                    mParameters.isHighQualityNoiseReductionMode()) {
                bCachedMem = QCAMERA_ION_USE_CACHE;
            }
            else {
                bCachedMem = QCAMERA_ION_USE_NOCACHE;
            }
            CDBG_HIGH("%s: %s video buf allocated ", __func__,
                    (bCachedMem == 0) ? "Uncached" : "Cached" );
            QCameraVideoMemory *videoMemory =
                    new QCameraVideoMemory(mGetMemory, bCachedMem);

            int usage = 0;
            cam_format_t fmt;
            mParameters.getStreamFormat(CAM_STREAM_TYPE_VIDEO,fmt);
            if (mParameters.isUBWCEnabled() && (fmt == CAM_FORMAT_YUV_420_NV12_UBWC)) {
                usage = private_handle_t::PRIV_FLAGS_UBWC_ALIGNED;
            }
            videoMemory->setVideoInfo(usage, fmt);
            mem = videoMemory;
        }
        break;
    case CAM_STREAM_TYPE_CALLBACK:
        mem = new QCameraStreamMemory(mGetMemory,
                bCachedMem,
                (bPoolMem) ? &m_memoryPool : NULL,
                stream_type);
        break;
    case CAM_STREAM_TYPE_DEFAULT:
    case CAM_STREAM_TYPE_MAX:
    default:
        break;
    }
    if (!mem) {
        return NULL;
    }

    if (bufferCnt > 0) {
        if (mParameters.isSecureMode() &&
            (stream_type == CAM_STREAM_TYPE_RAW) &&
            (mParameters.isRdiMode())) {
            ALOGD("%s: Allocating %d secure buffers of size %d ", __func__, bufferCnt, size);
            rc = mem->allocate(bufferCnt, size, SECURE);
        } else {
            rc = mem->allocate(bufferCnt, size, NON_SECURE);
        }
        if (rc < 0) {
            delete mem;
            return NULL;
        }
        bufferCnt = mem->getCnt();
    }
    return mem;
}

/*===========================================================================
 * FUNCTION   : allocateMoreStreamBuf
 *
 * DESCRIPTION: alocate more stream buffers from the memory object
 *
 * PARAMETERS :
 *   @mem_obj      : memory object ptr
 *   @size         : size of buffer
 *   @bufferCnt    : [IN/OUT] additional number of buffers to be allocated.
 *                   output will be the number of total buffers
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::allocateMoreStreamBuf(
        QCameraMemory *mem_obj, size_t size, uint8_t &bufferCnt)
{
    int rc = NO_ERROR;

    if (bufferCnt > 0) {
        rc = mem_obj->allocateMore(bufferCnt, size);
        bufferCnt = mem_obj->getCnt();
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : allocateMiscBuf
 *
 * DESCRIPTION: alocate miscellaneous buffer
 *
 * PARAMETERS :
 *   @streamInfo  : stream info
 *
 * RETURN     : ptr to a memory obj that holds stream info buffer.
 *              NULL if failed
 *==========================================================================*/
QCameraHeapMemory *QCamera2HardwareInterface::allocateMiscBuf(
        cam_stream_info_t *streamInfo)
{
    int rc = NO_ERROR;
    uint8_t bufNum = 0;
    size_t bufSize = 0;
    QCameraHeapMemory *miscBuf = NULL;
    uint32_t feature_mask =
            streamInfo->reprocess_config.pp_feature_config.feature_mask;

    switch (streamInfo->stream_type) {
    case CAM_STREAM_TYPE_OFFLINE_PROC:
        if (CAM_QCOM_FEATURE_TRUEPORTRAIT & feature_mask) {
            bufNum = 1;
            bufSize = mParameters.getTPMaxMetaSize();
        } else if (CAM_QCOM_FEATURE_REFOCUS & feature_mask) {
            bufNum = 1;
            bufSize = mParameters.getRefocusMaxMetaSize();
        }
        break;
    default:
        break;
    }

    if (bufNum && bufSize) {
        miscBuf = new QCameraHeapMemory(QCAMERA_ION_USE_CACHE);

        if (!miscBuf) {
            ALOGE("%s: Unable to allocate miscBuf object", __func__);
            return NULL;
        }

        rc = miscBuf->allocate(bufNum, bufSize, NON_SECURE);
        if (rc < 0) {
            ALOGE("%s: Failed to allocate misc buffer memory", __func__);
            delete miscBuf;
            return NULL;
        }
    }

    return miscBuf;
}

/*===========================================================================
 * FUNCTION   : allocateStreamInfoBuf
 *
 * DESCRIPTION: alocate stream info buffer
 *
 * PARAMETERS :
 *   @stream_type  : type of stream
 *
 * RETURN     : ptr to a memory obj that holds stream info buffer.
 *              NULL if failed
 *==========================================================================*/
QCameraHeapMemory *QCamera2HardwareInterface::allocateStreamInfoBuf(
        cam_stream_type_t stream_type)
{
    int rc = NO_ERROR;
    char value[PROPERTY_VALUE_MAX];
    bool raw_yuv = false;

    QCameraHeapMemory *streamInfoBuf = new QCameraHeapMemory(QCAMERA_ION_USE_CACHE);
    if (!streamInfoBuf) {
        ALOGE("allocateStreamInfoBuf: Unable to allocate streamInfo object");
        return NULL;
    }

    rc = streamInfoBuf->allocate(1, sizeof(cam_stream_info_t), NON_SECURE);
    if (rc < 0) {
        ALOGE("allocateStreamInfoBuf: Failed to allocate stream info memory");
        delete streamInfoBuf;
        return NULL;
    }

    cam_stream_info_t *streamInfo = (cam_stream_info_t *)streamInfoBuf->getPtr(0);
    memset(streamInfo, 0, sizeof(cam_stream_info_t));
    streamInfo->stream_type = stream_type;
    rc = mParameters.getStreamFormat(stream_type, streamInfo->fmt);
    if (isMonoCamera() && stream_type == CAM_STREAM_TYPE_PREVIEW) {
        // Display does not have support for Y ONLY.
        // For Mono camera, stream format info for backend will be updated as Y ONLY
        // Display will be updated as YCbCr, and CbCr data will be
        // initialized to 128 to get the grey scale image.
        streamInfo->fmt = CAM_FORMAT_Y_ONLY;
    }
    rc = mParameters.getStreamDimension(stream_type, streamInfo->dim);
    rc = mParameters.getStreamRotation(stream_type, streamInfo->pp_config, streamInfo->dim);
    streamInfo->num_bufs = getBufNumRequired(stream_type);
    streamInfo->streaming_mode = CAM_STREAMING_MODE_CONTINUOUS;
    streamInfo->is_secure = NON_SECURE;
    CDBG_HIGH("%s: stream_type %d, stream format %d,stream dimension %dx%d, num_bufs %d\n",
           __func__, stream_type, streamInfo->fmt, streamInfo->dim.width,
           streamInfo->dim.height, streamInfo->num_bufs);
    switch (stream_type) {
    case CAM_STREAM_TYPE_SNAPSHOT:
        if ((mParameters.isZSLMode() && mParameters.getRecordingHintValue() != true) ||
            mLongshotEnabled) {
            streamInfo->streaming_mode = CAM_STREAMING_MODE_CONTINUOUS;
        } else {
            streamInfo->streaming_mode = CAM_STREAMING_MODE_BURST;
            streamInfo->num_of_burst = (uint8_t)
                    (mParameters.getNumOfSnapshots()
                        + mParameters.getNumOfExtraHDRInBufsIfNeeded()
                        - mParameters.getNumOfExtraHDROutBufsIfNeeded()
                        + mParameters.getNumOfExtraBuffersForImageProc());
        }
        break;
    case CAM_STREAM_TYPE_RAW:
        property_get("persist.camera.raw_yuv", value, "0");
        raw_yuv = atoi(value) > 0 ? true : false;
        if ((mParameters.isZSLMode() || isRdiMode() || raw_yuv) &&
                !mParameters.getofflineRAW()) {
            CDBG_HIGH("RDI_DEBUG %s[%d]: CAM_STREAM_TYPE_RAW",
              __func__, __LINE__);
            streamInfo->streaming_mode = CAM_STREAMING_MODE_CONTINUOUS;
        } else {
            streamInfo->streaming_mode = CAM_STREAMING_MODE_BURST;
            streamInfo->num_of_burst = mParameters.getNumOfSnapshots();
        }
        if (mParameters.isSecureMode() && mParameters.isRdiMode()) {
            streamInfo->is_secure = SECURE;
        } else {
            streamInfo->is_secure = NON_SECURE;
        }
        break;
    case CAM_STREAM_TYPE_POSTVIEW:
        if (mLongshotEnabled) {
            streamInfo->streaming_mode = CAM_STREAMING_MODE_CONTINUOUS;
        } else {
            streamInfo->streaming_mode = CAM_STREAMING_MODE_BURST;
            streamInfo->num_of_burst = (uint8_t)(mParameters.getNumOfSnapshots()
                + mParameters.getNumOfExtraHDRInBufsIfNeeded()
                - mParameters.getNumOfExtraHDROutBufsIfNeeded()
                + mParameters.getNumOfExtraBuffersForImageProc());
        }
        break;
    case CAM_STREAM_TYPE_VIDEO:
        streamInfo->dis_enable = mParameters.isDISEnabled();
        if (mParameters.getBufBatchCount()) {
            //Update stream info structure with batch mode info
            streamInfo->streaming_mode = CAM_STREAMING_MODE_BATCH;
            streamInfo->user_buf_info.frame_buf_cnt = mParameters.getBufBatchCount();
            streamInfo->user_buf_info.size =
                    (uint32_t)(sizeof(struct msm_camera_user_buf_cont_t));
            cam_fps_range_t pFpsRange;
            mParameters.getHfrFps(pFpsRange);
            streamInfo->user_buf_info.frameInterval =
                    (long)((1000/pFpsRange.video_max_fps) * 1000);
            CDBG_HIGH("%s: Video Batch Count = %d, interval = %d", __func__,
                    streamInfo->user_buf_info.frame_buf_cnt,
                    streamInfo->user_buf_info.frameInterval);
        }
    case CAM_STREAM_TYPE_PREVIEW:
        if (mParameters.getRecordingHintValue()) {
            const char* dis_param = mParameters.get(QCameraParameters::KEY_QC_DIS);
            bool disEnabled = (dis_param != NULL)
                    && !strcmp(dis_param,QCameraParameters::VALUE_ENABLE);
            if(disEnabled) {
                streamInfo->is_type = mParameters.getISType();
            } else {
                streamInfo->is_type = IS_TYPE_NONE;
            }
        }
        if (mParameters.isSecureMode()) {
            streamInfo->is_secure = SECURE;
        }
        break;
    case CAM_STREAM_TYPE_ANALYSIS:
        streamInfo->noFrameExpected = 1;
        break;
    default:
        break;
    }

    // Update feature mask
    mParameters.updatePpFeatureMask(stream_type);

    // Get feature mask
    mParameters.getStreamPpMask(stream_type, streamInfo->pp_config.feature_mask);

    // Update pp config
    if (streamInfo->pp_config.feature_mask & CAM_QCOM_FEATURE_FLIP) {
        int flipMode = mParameters.getFlipMode(stream_type);
        if (flipMode > 0) {
            streamInfo->pp_config.flip = (uint32_t)flipMode;
        }
    }
    if (streamInfo->pp_config.feature_mask & CAM_QCOM_FEATURE_SHARPNESS) {
        streamInfo->pp_config.sharpness = mParameters.getInt(QCameraParameters::KEY_QC_SHARPNESS);
    }
    if (streamInfo->pp_config.feature_mask & CAM_QCOM_FEATURE_EFFECT) {
        streamInfo->pp_config.effect = mParameters.getEffectValue();
    }

    if (streamInfo->pp_config.feature_mask & CAM_QCOM_FEATURE_DENOISE2D) {
        streamInfo->pp_config.denoise2d.denoise_enable = 1;
        streamInfo->pp_config.denoise2d.process_plates =
                mParameters.getDenoiseProcessPlate(CAM_INTF_PARM_WAVELET_DENOISE);
    }

    if (!((needReprocess()) && (CAM_STREAM_TYPE_SNAPSHOT == stream_type ||
            CAM_STREAM_TYPE_RAW == stream_type))) {
        if (gCamCapability[mCameraId]->qcom_supported_feature_mask &
                CAM_QCOM_FEATURE_CROP)
            streamInfo->pp_config.feature_mask |= CAM_QCOM_FEATURE_CROP;
        if (gCamCapability[mCameraId]->qcom_supported_feature_mask &
                CAM_QCOM_FEATURE_SCALE)
            streamInfo->pp_config.feature_mask |= CAM_QCOM_FEATURE_SCALE;
    }

    CDBG_HIGH("%s: allocateStreamInfoBuf: stream type: %d, pp_mask: 0x%x",
            __func__, stream_type, streamInfo->pp_config.feature_mask);

    return streamInfoBuf;
}

/*===========================================================================
 * FUNCTION   : allocateStreamUserBuf
 *
 * DESCRIPTION: allocate user ptr for stream buffers
 *
 * PARAMETERS :
 *   @streamInfo  : stream info structure
 *
 * RETURN     : ptr to a memory obj that holds stream info buffer.
 *                    NULL if failed

 *==========================================================================*/
QCameraMemory *QCamera2HardwareInterface::allocateStreamUserBuf(
        cam_stream_info_t *streamInfo)
{
    int rc = NO_ERROR;
    QCameraMemory *mem = NULL;
    int bufferCnt = 0;
    int size = 0;

    if (streamInfo->streaming_mode != CAM_STREAMING_MODE_BATCH) {
        ALOGE("%s: Stream is not in BATCH mode. Invalid Stream", __func__);
        return NULL;
    }

    // Allocate stream user buffer memory object
    switch (streamInfo->stream_type) {
    case CAM_STREAM_TYPE_VIDEO: {
        QCameraVideoMemory *video_mem = new QCameraVideoMemory(
                mGetMemory, FALSE, CAM_STREAM_BUF_TYPE_USERPTR);
        video_mem->allocateMeta(streamInfo->num_bufs);
        mem = static_cast<QCameraMemory *>(video_mem);
    }
    break;

    case CAM_STREAM_TYPE_PREVIEW:
    case CAM_STREAM_TYPE_POSTVIEW:
    case CAM_STREAM_TYPE_ANALYSIS:
    case CAM_STREAM_TYPE_SNAPSHOT:
    case CAM_STREAM_TYPE_RAW:
    case CAM_STREAM_TYPE_METADATA:
    case CAM_STREAM_TYPE_OFFLINE_PROC:
    case CAM_STREAM_TYPE_CALLBACK:
        ALOGE("%s: Stream type Not supported.for BATCH processing", __func__);
    break;

    case CAM_STREAM_TYPE_DEFAULT:
    case CAM_STREAM_TYPE_MAX:
    default:
        break;
    }
    if (!mem) {
        ALOGE("%s: Failed to allocate mem", __func__);
        return NULL;
    }

    /*Size of this buffer will be number of batch buffer */
    size = PAD_TO_SIZE((streamInfo->num_bufs * streamInfo->user_buf_info.size),
            CAM_PAD_TO_4K);

    CDBG_HIGH("%s: Allocating BATCH Buffer count = %d", __func__, streamInfo->num_bufs);

    if (size > 0) {
        // Allocating one buffer for all batch buffers
        rc = mem->allocate(1, size, NON_SECURE);
        if (rc < 0) {
            delete mem;
            return NULL;
        }
    }
    return mem;
}


/*===========================================================================
 * FUNCTION   : waitForDeferredAlloc
 *
 * DESCRIPTION: Wait for deferred allocation, if applicable
 *              (applicable only for metadata buffers so far)
 *
 * PARAMETERS :
 *   @stream_type  : type of stream to (possibly) wait for
 *
 * RETURN     : None
 *==========================================================================*/
void QCamera2HardwareInterface::waitForDeferredAlloc(cam_stream_type_t stream_type)
{
    if (stream_type == CAM_STREAM_TYPE_METADATA) {
        waitDeferredWork(mMetadataAllocJob);
    }
}


/*===========================================================================
 * FUNCTION   : setPreviewWindow
 *
 * DESCRIPTION: set preview window impl
 *
 * PARAMETERS :
 *   @window  : ptr to window ops table struct
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::setPreviewWindow(
        struct preview_stream_ops *window)
{
    mPreviewWindow = window;
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : setCallBacks
 *
 * DESCRIPTION: set callbacks impl
 *
 * PARAMETERS :
 *   @notify_cb  : notify cb
 *   @data_cb    : data cb
 *   @data_cb_timestamp : data cb with time stamp
 *   @get_memory : request memory ops table
 *   @user       : user data ptr
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::setCallBacks(camera_notify_callback notify_cb,
                                            camera_data_callback data_cb,
                                            camera_data_timestamp_callback data_cb_timestamp,
                                            camera_request_memory get_memory,
                                            void *user)
{
    mNotifyCb        = notify_cb;
    mDataCb          = data_cb;
    mDataCbTimestamp = data_cb_timestamp;
    mGetMemory       = get_memory;
    mCallbackCookie  = user;
    m_cbNotifier.setCallbacks(notify_cb, data_cb, data_cb_timestamp, user);
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : setJpegCallBacks
 *
 * DESCRIPTION: set JPEG callbacks impl
 *
 * PARAMETERS :
 *   @jpegCb  : Jpeg callback method
 *   @callbackCookie    : callback cookie
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
void QCamera2HardwareInterface::setJpegCallBacks(jpeg_data_callback jpegCb,
                                            void *callbackCookie)
{
    CDBG_HIGH("%s: camera id %d", __func__, getCameraId());
    mJpegCb        = jpegCb;
    mJpegCallbackCookie  = callbackCookie;
    m_cbNotifier.setJpegCallBacks(mJpegCb, mJpegCallbackCookie);
}

/*===========================================================================
 * FUNCTION   : enableMsgType
 *
 * DESCRIPTION: enable msg type impl
 *
 * PARAMETERS :
 *   @msg_type  : msg type mask to be enabled
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::enableMsgType(int32_t msg_type)
{
    int32_t rc = NO_ERROR;

    if (mParameters.isUBWCEnabled()) {
        /*Need Special CALLBACK stream incase application requesting for
              Preview callback  in UBWC case*/
        if (!(msgTypeEnabled(CAMERA_MSG_PREVIEW_FRAME)) &&
                (msg_type & CAMERA_MSG_PREVIEW_FRAME)) {
            if (m_channels[QCAMERA_CH_TYPE_CALLBACK] != NULL) {
                rc = startChannel(QCAMERA_CH_TYPE_CALLBACK);
                if (rc != NO_ERROR) {
                    ALOGE("%s : START Callback Channel failed", __func__);
                }
            }
        }
    }

    mMsgEnabled |= msg_type;
    CDBG_HIGH("%s (0x%x) : mMsgEnabled = 0x%x", __func__, msg_type , mMsgEnabled );
    return rc;
}

/*===========================================================================
 * FUNCTION   : disableMsgType
 *
 * DESCRIPTION: disable msg type impl
 *
 * PARAMETERS :
 *   @msg_type  : msg type mask to be disabled
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::disableMsgType(int32_t msg_type)
{
    int32_t rc = NO_ERROR;

    if (mParameters.isUBWCEnabled()) {
        /*STOP CALLBACK STREAM*/
        if ((msgTypeEnabled(CAMERA_MSG_PREVIEW_FRAME)) &&
                (msg_type & CAMERA_MSG_PREVIEW_FRAME)) {
            if (m_channels[QCAMERA_CH_TYPE_CALLBACK] != NULL) {
                rc = stopChannel(QCAMERA_CH_TYPE_CALLBACK);
                if (rc != NO_ERROR) {
                    ALOGE("%s : STOP Callback Channel failed", __func__);
                }
            }
        }
    }

    mMsgEnabled &= ~msg_type;
    CDBG_HIGH("%s (0x%x) : mMsgEnabled = 0x%x", __func__, msg_type , mMsgEnabled );
    return rc;
}

/*===========================================================================
 * FUNCTION   : msgTypeEnabled
 *
 * DESCRIPTION: impl to determine if certain msg_type is enabled
 *
 * PARAMETERS :
 *   @msg_type  : msg type mask
 *
 * RETURN     : 0 -- not enabled
 *              none 0 -- enabled
 *==========================================================================*/
int QCamera2HardwareInterface::msgTypeEnabled(int32_t msg_type)
{
    return (mMsgEnabled & msg_type);
}

/*===========================================================================
 * FUNCTION   : msgTypeEnabledWithLock
 *
 * DESCRIPTION: impl to determine if certain msg_type is enabled with lock
 *
 * PARAMETERS :
 *   @msg_type  : msg type mask
 *
 * RETURN     : 0 -- not enabled
 *              none 0 -- enabled
 *==========================================================================*/
int QCamera2HardwareInterface::msgTypeEnabledWithLock(int32_t msg_type)
{
    int enabled = 0;
    lockAPI();
    enabled = mMsgEnabled & msg_type;
    unlockAPI();
    return enabled;
}

/*===========================================================================
 * FUNCTION   : startPreview
 *
 * DESCRIPTION: start preview impl
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::startPreview()
{
    KPI_ATRACE_CALL();
    int32_t rc = NO_ERROR;
    CDBG_HIGH("%s: E", __func__);
    updateThermalLevel((void *)&mThermalLevel);
    // start preview stream
    if (mParameters.isZSLMode() && mParameters.getRecordingHintValue() != true) {
        rc = startChannel(QCAMERA_CH_TYPE_ZSL);
    } else {
        rc = startChannel(QCAMERA_CH_TYPE_PREVIEW);
    }

    if (rc == NO_ERROR && (msgTypeEnabled(CAMERA_MSG_PREVIEW_FRAME))
            && (m_channels[QCAMERA_CH_TYPE_CALLBACK] != NULL)) {
        rc = startChannel(QCAMERA_CH_TYPE_CALLBACK);
        if (rc != NO_ERROR) {
            ALOGE("%s: failed to start callback stream", __func__);
            stopChannel(QCAMERA_CH_TYPE_ZSL);
            stopChannel(QCAMERA_CH_TYPE_PREVIEW);
        }
    }

    updatePostPreviewParameters();

    // if job id is non-zero, that means the postproc init job is already
    // pending or complete
    if (mInitPProcJob == 0) {
        mInitPProcJob = deferPPInit();
        if (mInitPProcJob == 0) {
            ALOGE("%s: Unable to initialize postprocessor, mCameraHandle = %p",
                    __func__, mCameraHandle);
            rc = UNKNOWN_ERROR;
        }
    }

    mParameters.setPreviewCallbackNeeded(true);
    CDBG_HIGH("%s: X", __func__);
    return rc;
}

int32_t QCamera2HardwareInterface::updatePostPreviewParameters() {
    // Enable OIS only in Camera mode and 4k2k camcoder mode
    int32_t rc = NO_ERROR;
    pthread_mutex_lock(&m_parm_lock);
    rc = mParameters.updateOisValue(1);
    pthread_mutex_unlock(&m_parm_lock);
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : stopPreview
 *
 * DESCRIPTION: stop preview impl
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::stopPreview()
{
    KPI_ATRACE_CALL();
    CDBG_HIGH("%s: E", __func__);
    mNumPreviewFaces = -1;
    mActiveAF = false;
    // stop preview stream
    stopChannel(QCAMERA_CH_TYPE_CALLBACK);
    stopChannel(QCAMERA_CH_TYPE_ZSL);
    stopChannel(QCAMERA_CH_TYPE_PREVIEW);

    m_cbNotifier.flushPreviewNotifications();
    //add for ts makeup
#ifdef TARGET_TS_MAKEUP
    ts_makeup_finish();
#endif
    // delete all channels from preparePreview
    unpreparePreview();
    CDBG_HIGH("%s: X", __func__);
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : storeMetaDataInBuffers
 *
 * DESCRIPTION: enable store meta data in buffers for video frames impl
 *
 * PARAMETERS :
 *   @enable  : flag if need enable
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::storeMetaDataInBuffers(int enable)
{
    mStoreMetaDataInFrame = enable;
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : preStartRecording
 *
 * DESCRIPTION: Prepare start recording impl
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::preStartRecording()
{
    int32_t rc = NO_ERROR;
    CDBG_HIGH("%s: E", __func__);
    if (mParameters.getRecordingHintValue() == false) {

        // Give HWI control to restart preview only in single camera mode.
        // In dual-cam mode, this control belongs to muxer.
        if (getRelatedCamSyncInfo()->sync_control != CAM_SYNC_RELATED_SENSORS_ON) {
            CDBG_HIGH("%s: start recording when hint is false, stop preview first", __func__);
            stopPreview();

            // Set recording hint to TRUE
            pthread_mutex_lock(&m_parm_lock);
            mParameters.updateRecordingHintValue(TRUE);
            pthread_mutex_unlock(&m_parm_lock);
            rc = preparePreview();
            if (rc == NO_ERROR) {
                rc = startPreview();
            }
        }
        else
        {
            // For dual cam mode, update the flag mPreviewRestartNeeded to true
            // Restart control will be handled by muxer.
            mPreviewRestartNeeded = true;
        }
    }

    CDBG_HIGH("%s: X", __func__);
    return rc;
}

/*===========================================================================
 * FUNCTION   : startRecording
 *
 * DESCRIPTION: start recording impl
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::startRecording()
{
    int32_t rc = NO_ERROR;
    CDBG_HIGH("%s: E", __func__);

    //link meta stream with video channel if low power mode.
    if (isLowPowerMode()) {
        // Find and try to link a metadata stream from preview channel
        QCameraChannel *pMetaChannel = NULL;
        QCameraStream *pMetaStream = NULL;
        QCameraChannel *pVideoChannel = m_channels[QCAMERA_CH_TYPE_VIDEO];

        if (m_channels[QCAMERA_CH_TYPE_PREVIEW] != NULL) {
            pMetaChannel = m_channels[QCAMERA_CH_TYPE_PREVIEW];
            uint32_t streamNum = pMetaChannel->getNumOfStreams();
            QCameraStream *pStream = NULL;
            for (uint32_t i = 0 ; i < streamNum ; i++ ) {
                pStream = pMetaChannel->getStreamByIndex(i);
                if ((NULL != pStream) &&
                        (CAM_STREAM_TYPE_METADATA == pStream->getMyType())) {
                    pMetaStream = pStream;
                    break;
                }
            }
        }

        if ((NULL != pMetaChannel) && (NULL != pMetaStream)) {
            rc = pVideoChannel->linkStream(pMetaChannel, pMetaStream);
            if (NO_ERROR != rc) {
                CDBG_HIGH("%s : Metadata stream link failed %d", __func__, rc);
            }
        }
    }

    if (rc == NO_ERROR) {
        rc = startChannel(QCAMERA_CH_TYPE_VIDEO);
    }

#ifdef HAS_MULTIMEDIA_HINTS
    if (rc == NO_ERROR) {
        if (m_pPowerModule) {
            if (m_pPowerModule->powerHint) {
                m_pPowerModule->powerHint(m_pPowerModule, POWER_HINT_VIDEO_ENCODE, (void *)"state=1");
            }
        }
    }
#endif
    CDBG_HIGH("%s: X", __func__);
    return rc;
}

/*===========================================================================
 * FUNCTION   : stopRecording
 *
 * DESCRIPTION: stop recording impl
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::stopRecording()
{
    CDBG_HIGH("%s: E", __func__);
    int rc = stopChannel(QCAMERA_CH_TYPE_VIDEO);

#ifdef USE_MEDIA_EXTENSIONS
    m_cbNotifier.flushVideoNotifications();
#endif

#ifdef HAS_MULTIMEDIA_HINTS
    if (m_pPowerModule) {
        if (m_pPowerModule->powerHint) {
            m_pPowerModule->powerHint(m_pPowerModule, POWER_HINT_VIDEO_ENCODE, (void *)"state=0");
        }
    }
#endif
    CDBG_HIGH("%s: X", __func__);
    return rc;
}

/*===========================================================================
 * FUNCTION   : releaseRecordingFrame
 *
 * DESCRIPTION: return video frame impl
 *
 * PARAMETERS :
 *   @opaque  : ptr to video frame to be returned
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::releaseRecordingFrame(const void * opaque)
{
    int32_t rc = UNKNOWN_ERROR;
    QCameraVideoChannel *pChannel =
        (QCameraVideoChannel *)m_channels[QCAMERA_CH_TYPE_VIDEO];
    CDBG("%s: opaque data = %p", __func__,opaque);
    if(pChannel != NULL) {
        rc = pChannel->releaseFrame(opaque, mStoreMetaDataInFrame > 0);
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : autoFocus
 *
 * DESCRIPTION: start auto focus impl
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::autoFocus()
{
    int rc = NO_ERROR;
    cam_focus_mode_type focusMode = mParameters.getFocusMode();
    CDBG_HIGH("[AF_DBG] %s: focusMode=%d, m_currentFocusState=%d",
            __func__, focusMode, m_currentFocusState);

    switch (focusMode) {
    case CAM_FOCUS_MODE_AUTO:
    case CAM_FOCUS_MODE_MACRO:
    case CAM_FOCUS_MODE_CONTINOUS_VIDEO:
    case CAM_FOCUS_MODE_CONTINOUS_PICTURE:
        mActiveAF = true;
        rc = mCameraHandle->ops->do_auto_focus(mCameraHandle->camera_handle);
        break;
    case CAM_FOCUS_MODE_INFINITY:
    case CAM_FOCUS_MODE_FIXED:
    case CAM_FOCUS_MODE_EDOF:
    default:
        ALOGE("%s: No ops in focusMode (%d)", __func__, focusMode);
        rc = sendEvtNotify(CAMERA_MSG_FOCUS, true, 0);
        break;
    }

    if (NO_ERROR != rc) {
        mActiveAF = false;
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : cancelAutoFocus
 *
 * DESCRIPTION: cancel auto focus impl
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::cancelAutoFocus()
{
    int rc = NO_ERROR;
    cam_focus_mode_type focusMode = mParameters.getFocusMode();

    switch (focusMode) {
    case CAM_FOCUS_MODE_AUTO:
    case CAM_FOCUS_MODE_MACRO:
    case CAM_FOCUS_MODE_CONTINOUS_VIDEO:
    case CAM_FOCUS_MODE_CONTINOUS_PICTURE:
        mActiveAF = false;
        rc = mCameraHandle->ops->cancel_auto_focus(mCameraHandle->camera_handle);
        break;
    case CAM_FOCUS_MODE_INFINITY:
    case CAM_FOCUS_MODE_FIXED:
    case CAM_FOCUS_MODE_EDOF:
    default:
        CDBG("%s: No ops in focusMode (%d)", __func__, focusMode);
        break;
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : processUFDumps
 *
 * DESCRIPTION: process UF jpeg dumps for refocus support
 *
 * PARAMETERS :
 *   @evt     : payload of jpeg event, including information about jpeg encoding
 *              status, jpeg size and so on.
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *
 * NOTE       : none
 *==========================================================================*/
bool QCamera2HardwareInterface::processUFDumps(qcamera_jpeg_evt_payload_t *evt)
{
   bool ret = true;
   if (mParameters.isUbiRefocus()) {
       int index = (int)getOutputImageCount();
       bool allFocusImage = (index == ((int)mParameters.getRefocusOutputCount() - 1));
       char name[FILENAME_MAX];

       camera_memory_t *jpeg_mem = NULL;
       omx_jpeg_ouput_buf_t *jpeg_out = NULL;
       size_t dataLen;
       uint8_t *dataPtr;
       waitDeferredWork(mInitPProcJob);
       if (!m_postprocessor.getJpegMemOpt()) {
           dataLen = evt->out_data.buf_filled_len;
           dataPtr = evt->out_data.buf_vaddr;
       } else {
           jpeg_out  = (omx_jpeg_ouput_buf_t*) evt->out_data.buf_vaddr;
           if (!jpeg_out) {
              ALOGE("%s:%d] Null pointer detected",  __func__, __LINE__);
              return false;
           }
           jpeg_mem = (camera_memory_t *)jpeg_out->mem_hdl;
           if (!jpeg_mem) {
              ALOGE("%s:%d] Null pointer detected",  __func__, __LINE__);
              return false;
           }
           dataPtr = (uint8_t *)jpeg_mem->data;
           dataLen = jpeg_mem->size;
       }

       if (allFocusImage)  {
           snprintf(name, sizeof(name), "AllFocusImage");
           index = -1;
       } else {
           snprintf(name, sizeof(name), "%d", 0);
       }
       CAM_DUMP_TO_FILE(QCAMERA_DUMP_FRM_LOCATION"ubifocus", name, index, "jpg",
           dataPtr, dataLen);
       CDBG("%s:%d] Dump the image %d %d allFocusImage %d", __func__, __LINE__,
           getOutputImageCount(), index, allFocusImage);
       setOutputImageCount(getOutputImageCount() + 1);
       if (!allFocusImage) {
           ret = false;
       }
   }
   return ret;
}

/*===========================================================================
 * FUNCTION   : unconfigureAdvancedCapture
 *
 * DESCRIPTION: unconfigure Advanced Capture.
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::unconfigureAdvancedCapture()
{
    int32_t rc = NO_ERROR;

    if (mAdvancedCaptureConfigured) {

        mAdvancedCaptureConfigured = false;

        if(mIs3ALocked) {
            mParameters.set3ALock(QCameraParameters::VALUE_FALSE);
            mIs3ALocked = false;
        }
        if (mParameters.isHDREnabled() || mParameters.isAEBracketEnabled()) {
            rc = mParameters.setToneMapMode(true, true);
            if (rc != NO_ERROR) {
                CDBG_HIGH("%s: Failed to enable tone map during HDR/AEBracketing", __func__);
            }
            mHDRBracketingEnabled = false;
            rc = mParameters.stopAEBracket();
        } else if (mParameters.isChromaFlashEnabled() || mFlashNeeded) {
            rc = mParameters.resetFrameCapture(TRUE);
        } else if (mParameters.isUbiFocusEnabled() || mParameters.isUbiRefocus()) {
            rc = configureAFBracketing(false);
        } else if (mParameters.isOptiZoomEnabled()) {
            rc = mParameters.setAndCommitZoom(mZoomLevel);
        } else if (mParameters.isStillMoreEnabled()) {
            cam_still_more_t stillmore_config = mParameters.getStillMoreSettings();
            stillmore_config.burst_count = 0;
            mParameters.setStillMoreSettings(stillmore_config);

            /* If SeeMore is running, it will handle re-enabling tone map */
            if (!mParameters.isSeeMoreEnabled() && !mParameters.isLTMForSeeMoreEnabled()) {
                rc = mParameters.setToneMapMode(true, true);
                if (rc != NO_ERROR) {
                    CDBG_HIGH("%s: Failed to enable tone map during StillMore", __func__);
                }
            }

            /* Re-enable Tintless */
            mParameters.setTintless(true);
        } else {
            ALOGE("%s: No Advanced Capture feature enabled!! ", __func__);
            rc = BAD_VALUE;
        }
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : configureAdvancedCapture
 *
 * DESCRIPTION: configure Advanced Capture.
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::configureAdvancedCapture()
{
    CDBG_HIGH("%s: E",__func__);
    int32_t rc = NO_ERROR;

    rc = mParameters.checkFeatureConcurrency();
    if (rc != NO_ERROR) {
        ALOGE("%s: Cannot support Advanced capture modes", __func__);
        return rc;
    }

    setOutputImageCount(0);
    mInputCount = 0;

    if (getRelatedCamSyncInfo()->mode == CAM_MODE_SECONDARY) {
        // no Advance capture settings for Aux camera
        CDBG_HIGH("%s: X Secondary Camera, no need to process!! ", __func__);
        return rc;
    }

    pthread_mutex_lock(&m_parm_lock);

    /* Temporarily stop display only if not in stillmore livesnapshot */
    if (!(mParameters.isStillMoreEnabled() &&
            mParameters.isSeeMoreEnabled())) {
        mParameters.setDisplayFrame(FALSE);
    }

    if (mParameters.isUbiFocusEnabled() || mParameters.isUbiRefocus()) {
        rc = configureAFBracketing();
    } else if (mParameters.isOptiZoomEnabled()) {
        rc = configureOptiZoom();
    } else if(mParameters.isHDREnabled()) {
        rc = configureHDRBracketing();
        if (mHDRBracketingEnabled) {
            rc = mParameters.setToneMapMode(false, true);
            if (rc != NO_ERROR) {
                CDBG_HIGH("%s: Failed to disable tone map during HDR", __func__);
            }
        }
    } else if (mParameters.isAEBracketEnabled()) {
        rc = mParameters.setToneMapMode(false, true);
        if (rc != NO_ERROR) {
            CDBG_HIGH("%s: Failed to disable tone map during AEBracketing", __func__);
        }
        rc = configureAEBracketing();
    } else if (mParameters.isStillMoreEnabled()) {
        rc = configureStillMore();
    } else if (mParameters.isChromaFlashEnabled() || mFlashNeeded) {
        rc = mParameters.configFrameCapture(TRUE);
    } else {
        ALOGE("%s: No Advanced Capture feature enabled!! ", __func__);
        rc = BAD_VALUE;
    }

    if (NO_ERROR == rc) {
        mAdvancedCaptureConfigured = true;
    } else {
        mAdvancedCaptureConfigured = false;
    }

    pthread_mutex_unlock(&m_parm_lock);
    CDBG_HIGH("%s: X",__func__);
    return rc;
}

/*===========================================================================
 * FUNCTION   : configureAFBracketing
 *
 * DESCRIPTION: configure AF Bracketing.
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::configureAFBracketing(bool enable)
{
    CDBG_HIGH("%s: E",__func__);
    int32_t rc = NO_ERROR;
    cam_af_bracketing_t *af_bracketing_need;

    if (mParameters.isUbiRefocus()) {
        af_bracketing_need =
                &gCamCapability[mCameraId]->refocus_af_bracketing_need;
    } else {
        af_bracketing_need =
                &gCamCapability[mCameraId]->ubifocus_af_bracketing_need;
    }

    //Enable AF Bracketing.
    cam_af_bracketing_t afBracket;
    memset(&afBracket, 0, sizeof(cam_af_bracketing_t));
    afBracket.enable = enable;
    afBracket.burst_count = af_bracketing_need->burst_count;

    for(int8_t i = 0; i < MAX_AF_BRACKETING_VALUES; i++) {
        afBracket.focus_steps[i] = af_bracketing_need->focus_steps[i];
        CDBG_HIGH("%s: focus_step[%d] = %d", __func__, i, afBracket.focus_steps[i]);
    }
    //Send cmd to backend to set AF Bracketing for Ubi Focus.
    rc = mParameters.commitAFBracket(afBracket);
    if ( NO_ERROR != rc ) {
        ALOGE("%s: cannot configure AF bracketing", __func__);
        return rc;
    }
    if (enable) {
        mParameters.set3ALock(QCameraParameters::VALUE_TRUE);
        mIs3ALocked = true;
    }
    CDBG_HIGH("%s: X",__func__);
    return rc;
}

/*===========================================================================
 * FUNCTION   : configureHDRBracketing
 *
 * DESCRIPTION: configure HDR Bracketing.
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::configureHDRBracketing()
{
    CDBG_HIGH("%s: E",__func__);
    int32_t rc = NO_ERROR;

    cam_hdr_bracketing_info_t& hdrBracketingSetting =
            gCamCapability[mCameraId]->hdr_bracketing_setting;

    // 'values' should be in "idx1,idx2,idx3,..." format
    uint32_t hdrFrameCount =
            hdrBracketingSetting.num_frames;
    CDBG_HIGH("%s : HDR values %d, %d frame count: %u",
          __func__,
          (int8_t) hdrBracketingSetting.exp_val.values[0],
          (int8_t) hdrBracketingSetting.exp_val.values[1],
          hdrFrameCount);

    // Enable AE Bracketing for HDR
    cam_exp_bracketing_t aeBracket;
    memset(&aeBracket, 0, sizeof(cam_exp_bracketing_t));
    aeBracket.mode =
        hdrBracketingSetting.exp_val.mode;

    if (aeBracket.mode == CAM_EXP_BRACKETING_ON) {
        mHDRBracketingEnabled = true;
    }

    String8 tmp("");
    for (uint32_t i = 0; i < hdrFrameCount; i++) {
        tmp.appendFormat("%d",
            (int8_t) hdrBracketingSetting.exp_val.values[i]);
        tmp.append(",");
    }
    if (mParameters.isHDR1xFrameEnabled()
        && mParameters.isHDR1xExtraBufferNeeded()) {
            tmp.appendFormat("%d", 0);
            tmp.append(",");
    }

    if( !tmp.isEmpty() &&
        ( MAX_EXP_BRACKETING_LENGTH > tmp.length() ) ) {
        //Trim last comma
        memset(aeBracket.values, '\0', MAX_EXP_BRACKETING_LENGTH);
        memcpy(aeBracket.values, tmp.string(), tmp.length() - 1);
    }

    CDBG_HIGH("%s : HDR config values %s",
          __func__,
          aeBracket.values);
    rc = mParameters.setHDRAEBracket(aeBracket);
    if ( NO_ERROR != rc ) {
        ALOGE("%s: cannot configure HDR bracketing", __func__);
        return rc;
    }
    CDBG_HIGH("%s: X",__func__);
    return rc;
}

/*===========================================================================
 * FUNCTION   : configureAEBracketing
 *
 * DESCRIPTION: configure AE Bracketing.
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::configureAEBracketing()
{
    CDBG_HIGH("%s: E",__func__);
    int32_t rc = NO_ERROR;

    rc = mParameters.setAEBracketing();
    if ( NO_ERROR != rc ) {
        ALOGE("%s: cannot configure AE bracketing", __func__);
        return rc;
    }
    CDBG_HIGH("%s: X",__func__);
    return rc;
}

/*===========================================================================
 * FUNCTION   : configureOptiZoom
 *
 * DESCRIPTION: configure Opti Zoom.
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::configureOptiZoom()
{
    int32_t rc = NO_ERROR;

    //store current zoom level.
    mZoomLevel = mParameters.getParmZoomLevel();

    //set zoom level to 1x;
    mParameters.setAndCommitZoom(0);

    mParameters.set3ALock(QCameraParameters::VALUE_TRUE);

    mIs3ALocked = true;

    return rc;
}

/*===========================================================================
 * FUNCTION   : configureStillMore
 *
 * DESCRIPTION: configure StillMore.
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::configureStillMore()
{
    int32_t rc = NO_ERROR;
    uint8_t burst_cnt = 0;
    cam_still_more_t stillmore_config;
    cam_still_more_t stillmore_cap;

    /* Disable Tone Map. If seemore is enabled, it will handle disabling it. */
    if (!mParameters.isSeeMoreEnabled() && !mParameters.isLTMForSeeMoreEnabled()) {
        rc = mParameters.setToneMapMode(false, true);
        if (rc != NO_ERROR) {
            CDBG_HIGH("%s: Failed to disable tone map during StillMore", __func__);
        }
    }

    /* Lock 3A */
    mParameters.set3ALock(QCameraParameters::VALUE_TRUE);
    mIs3ALocked = true;

    /* Disable Tintless */
    mParameters.setTintless(false);

    /* Configure burst count based on user input */
    char prop[PROPERTY_VALUE_MAX];
    property_get("persist.camera.imglib.stillmore", prop, "0");
    burst_cnt = (uint32_t)atoi(prop);

    /* In the case of liveshot, burst should be 1 */
    if (mParameters.isSeeMoreEnabled()) {
        burst_cnt = 1;
    }

    /* Validate burst count */
    stillmore_cap = mParameters.getStillMoreCapability();
    if ((burst_cnt < stillmore_cap.min_burst_count) ||
            (burst_cnt > stillmore_cap.max_burst_count)) {
        burst_cnt = stillmore_cap.max_burst_count;
    }

    memset(&stillmore_config, 0, sizeof(cam_still_more_t));
    stillmore_config.burst_count = burst_cnt;
    mParameters.setStillMoreSettings(stillmore_config);

    CDBG_HIGH("%s: Stillmore burst %d", __func__, burst_cnt);

    return rc;
}

/*===========================================================================
 * FUNCTION   : stopAdvancedCapture
 *
 * DESCRIPTION: stops advanced capture based on capture type
 *
 * PARAMETERS :
 *   @pChannel : channel.
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::stopAdvancedCapture(
        QCameraPicChannel *pChannel)
{
    CDBG_HIGH("%s: stop bracketig",__func__);
    int32_t rc = NO_ERROR;

    if(mParameters.isUbiFocusEnabled() || mParameters.isUbiRefocus()) {
        rc = pChannel->stopAdvancedCapture(MM_CAMERA_AF_BRACKETING);
    } else if (mParameters.isChromaFlashEnabled() || mFlashNeeded) {
        rc = pChannel->stopAdvancedCapture(MM_CAMERA_FRAME_CAPTURE);
    } else if(mParameters.isHDREnabled()
            || mParameters.isAEBracketEnabled()) {
        rc = pChannel->stopAdvancedCapture(MM_CAMERA_AE_BRACKETING);
    } else if (mParameters.isOptiZoomEnabled()) {
        rc = pChannel->stopAdvancedCapture(MM_CAMERA_ZOOM_1X);
    } else if (mParameters.isStillMoreEnabled()) {
        CDBG_HIGH("%s: stopAdvancedCapture not needed for StillMore", __func__);
    } else {
        ALOGE("%s: No Advanced Capture feature enabled!",__func__);
        rc = BAD_VALUE;
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : startAdvancedCapture
 *
 * DESCRIPTION: starts advanced capture based on capture type
 *
 * PARAMETERS :
 *   @pChannel : channel.
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::startAdvancedCapture(
        QCameraPicChannel *pChannel)
{
    CDBG_HIGH("%s: Start bracketing",__func__);
    int32_t rc = NO_ERROR;

    if(mParameters.isUbiFocusEnabled() || mParameters.isUbiRefocus()) {
        rc = pChannel->startAdvancedCapture(MM_CAMERA_AF_BRACKETING);
    } else if (mParameters.isOptiZoomEnabled()) {
        rc = pChannel->startAdvancedCapture(MM_CAMERA_ZOOM_1X);
    } else if (mParameters.isStillMoreEnabled()) {
        CDBG_HIGH("%s: startAdvancedCapture not needed for StillMore", __func__);
    } else if (mParameters.isHDREnabled()
            || mParameters.isAEBracketEnabled()) {
        rc = pChannel->startAdvancedCapture(MM_CAMERA_AE_BRACKETING);
    } else if (mParameters.isChromaFlashEnabled() || mFlashNeeded) {
        cam_capture_frame_config_t config = mParameters.getCaptureFrameConfig();
        rc = pChannel->startAdvancedCapture(MM_CAMERA_FRAME_CAPTURE, &config);
    } else {
        ALOGE("%s: No Advanced Capture feature enabled!",__func__);
        rc = BAD_VALUE;
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : preTakePicture
 *
 * DESCRIPTION: Prepare take picture impl, Restarts preview if necessary
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::preTakePicture()
{
    int32_t rc = NO_ERROR;
    CDBG_HIGH("%s: E", __func__);
    if (mParameters.getRecordingHintValue() == true) {

        // Give HWI control to restart preview only in single camera mode.
        // In dual-cam mode, this control belongs to muxer.
        if (getRelatedCamSyncInfo()->sync_control != CAM_SYNC_RELATED_SENSORS_ON) {
            CDBG_HIGH("%s: restart preview if rec hint is true and preview is running", __func__);
            stopPreview();
            pthread_mutex_lock(&m_parm_lock);
            mParameters.updateRecordingHintValue(FALSE);
            pthread_mutex_unlock(&m_parm_lock);
            // start preview again
            rc = preparePreview();
            if (rc == NO_ERROR) {
                rc = startPreview();
                if (rc != NO_ERROR) {
                    unpreparePreview();
                }
            }
        }
        else
        {
            // For dual cam mode, update the flag mPreviewRestartNeeded to true
            // Restart control will be handled by muxer.
            mPreviewRestartNeeded = true;
        }
    }

    CDBG_HIGH("%s: X", __func__);
    return rc;
}

/*===========================================================================
 * FUNCTION   : takePicture
 *
 * DESCRIPTION: take picture impl
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::takePicture()
{
    int rc = NO_ERROR;

    // Get total number for snapshots (retro + regular)
    uint8_t numSnapshots = mParameters.getNumOfSnapshots();
    uint8_t numAdvancedSnapshot = mParameters.getBurstCountForAdvancedCapture();
    // Get number of retro-active snapshots
    uint8_t numRetroSnapshots = mParameters.getNumOfRetroSnapshots();
    CDBG_HIGH("%s: E", __func__);

    //Set rotation value from user settings as Jpeg rotation
    //to configure back-end modules.
    mParameters.setJpegRotation(mParameters.getRotation());

    // Check if retro-active snapshots are not enabled
    if (!isRetroPicture() || !mParameters.isZSLMode()) {
      numRetroSnapshots = 0;
      CDBG_HIGH("%s: [ZSL Retro] Reset retro snaphot count to zero", __func__);
    }
    if (mParameters.isUbiFocusEnabled() ||
            mParameters.isUbiRefocus() ||
            mParameters.isOptiZoomEnabled() ||
            mParameters.isHDREnabled() ||
            mParameters.isChromaFlashEnabled() ||
            mParameters.isAEBracketEnabled() ||
            mParameters.isStillMoreEnabled()) {
        rc = configureAdvancedCapture();
        if (rc == NO_ERROR) {
            numSnapshots = numAdvancedSnapshot;
        }
    }
    CDBG_HIGH("%s: [ZSL Retro] numSnapshots = %d, numRetroSnapshots = %d",
          __func__, numSnapshots, numRetroSnapshots);

    if (mParameters.isZSLMode()) {
        QCameraPicChannel *pZSLChannel =
            (QCameraPicChannel *)m_channels[QCAMERA_CH_TYPE_ZSL];
        if (NULL != pZSLChannel) {

            rc = configureOnlineRotation(*pZSLChannel);
            if (rc != NO_ERROR) {
                ALOGE("%s: online rotation failed", __func__);
                return rc;
            }

            // start postprocessor
            DeferWorkArgs args;
            memset(&args, 0, sizeof(DeferWorkArgs));

            args.pprocArgs = pZSLChannel;

            // No need to wait for mInitPProcJob here, because it was
            // queued in startPreview, and will definitely be processed before
            // mReprocJob can begin.
            mReprocJob = queueDeferredWork(CMD_DEF_PPROC_START,
                    args);
            if (mReprocJob == 0) {
                ALOGE("%s: Failure: Unable to start pproc", __func__);
                return UNKNOWN_ERROR;
            }
            // Create JPEG session
            mJpegJob = queueDeferredWork(CMD_DEF_CREATE_JPEG_SESSION,
                    args);
            if (mJpegJob == 0) {
                ALOGE("%s: Failure: Unable to create jpeg session", __func__);
                return UNKNOWN_ERROR;
            }

            if (mParameters.isUbiFocusEnabled() ||
                    mParameters.isUbiRefocus() ||
                    mParameters.isOptiZoomEnabled() ||
                    mParameters.isHDREnabled() ||
                    mParameters.isChromaFlashEnabled() ||
                    mParameters.isAEBracketEnabled() ||
                    mParameters.isStillMoreEnabled()) {
                rc = startAdvancedCapture(pZSLChannel);
                if (rc != NO_ERROR) {
                    ALOGE("%s: cannot start zsl advanced capture", __func__);
                    return rc;
                }
            }
            if (mLongshotEnabled && mPrepSnapRun) {
                mCameraHandle->ops->start_zsl_snapshot(
                        mCameraHandle->camera_handle,
                        pZSLChannel->getMyHandle());
            }
            // If frame sync is ON and it is a SECONDARY camera,
            // we do not need to send the take picture command to interface
            // It will be handled along with PRIMARY camera takePicture request
            mm_camera_req_buf_t buf;
            memset(&buf, 0x0, sizeof(buf));
            if ((getRelatedCamSyncInfo()->is_frame_sync_enabled) &&
                    m_bMpoEnabled) {
                if (getRelatedCamSyncInfo()->mode == CAM_MODE_PRIMARY) {
                    buf.type = MM_CAMERA_REQ_FRAME_SYNC_BUF;
                    buf.num_buf_requested = numSnapshots;
                    rc = pZSLChannel->takePicture(&buf);
                    if (rc != NO_ERROR) {
                        ALOGE("%s: FS_DBG cannot take ZSL picture, stop pproc",
                                __func__);
                        waitDeferredWork(mReprocJob);
                        waitDeferredWork(mJpegJob);
                        m_postprocessor.stop();
                        return rc;
                    }
                    ALOGI("%s: PRIMARY camera: send frame sync takePicture!!",
                            __func__);
                }
            } else {
                buf.type = MM_CAMERA_REQ_SUPER_BUF;
                buf.num_buf_requested = numSnapshots;
                buf.num_retro_buf_requested = numRetroSnapshots;
                rc = pZSLChannel->takePicture(&buf);
                if (rc != NO_ERROR) {
                    ALOGE("%s: cannot take ZSL picture, stop pproc", __func__);
                    waitDeferredWork(mReprocJob);
                    waitDeferredWork(mJpegJob);
                    m_postprocessor.stop();
                    return rc;
                }
            }
        } else {
            ALOGE("%s: ZSL channel is NULL", __func__);
            return UNKNOWN_ERROR;
        }
    } else {

        // start snapshot
        if (mParameters.isJpegPictureFormat() ||
            mParameters.isNV16PictureFormat() ||
            mParameters.isNV21PictureFormat()) {

            if (!isLongshotEnabled()) {

                // normal capture case
                // need to stop preview channel
                stopChannel(QCAMERA_CH_TYPE_PREVIEW);
                delChannel(QCAMERA_CH_TYPE_PREVIEW);

                rc = declareSnapshotStreams();
                if (NO_ERROR != rc) {
                    return rc;
                }

                rc = addCaptureChannel();
            } else {
                // normal capture case
                // need to stop preview channel

                stopChannel(QCAMERA_CH_TYPE_PREVIEW);
                delChannel(QCAMERA_CH_TYPE_PREVIEW);

                rc = declareSnapshotStreams();
                if (NO_ERROR != rc) {
                    return rc;
                }

                rc = addCaptureChannel();
            }

            if ((rc == NO_ERROR) &&
                (NULL != m_channels[QCAMERA_CH_TYPE_CAPTURE])) {

                if (!mParameters.getofflineRAW()) {
                    rc = configureOnlineRotation(
                        *m_channels[QCAMERA_CH_TYPE_CAPTURE]);
                    if (rc != NO_ERROR) {
                        ALOGE("%s: online rotation failed", __func__);
                        delChannel(QCAMERA_CH_TYPE_CAPTURE);
                        return rc;
                    }
                }

                DeferWorkArgs args;
                memset(&args, 0, sizeof(DeferWorkArgs));

                args.pprocArgs = m_channels[QCAMERA_CH_TYPE_CAPTURE];

                // No need to wait for mInitPProcJob here, because it was
                // queued in startPreview, and will definitely be processed before
                // mReprocJob can begin.
                mReprocJob = queueDeferredWork(CMD_DEF_PPROC_START,
                        args);
                if (mReprocJob == 0) {
                    ALOGE("%s: Failure: Unable to start pproc", __func__);
                    return UNKNOWN_ERROR;
                }

                // Create JPEG session
                mJpegJob = queueDeferredWork(CMD_DEF_CREATE_JPEG_SESSION,
                        args);

                // start catpure channel
                rc =  m_channels[QCAMERA_CH_TYPE_CAPTURE]->start();
                if (rc != NO_ERROR) {
                    ALOGE("%s: cannot start capture channel", __func__);
                    waitDeferredWork(mReprocJob);
                    waitDeferredWork(mJpegJob);
                    delChannel(QCAMERA_CH_TYPE_CAPTURE);
                    return rc;
                }

                QCameraPicChannel *pCapChannel =
                    (QCameraPicChannel *)m_channels[QCAMERA_CH_TYPE_CAPTURE];
                if (NULL != pCapChannel) {
                    if (mParameters.isUbiFocusEnabled() ||
                            mParameters.isUbiRefocus() ||
                            mParameters.isChromaFlashEnabled()) {
                        rc = startAdvancedCapture(pCapChannel);
                        if (rc != NO_ERROR) {
                            ALOGE("%s: cannot start advanced capture", __func__);
                            return rc;
                        }
                    }
                }
                if ( mLongshotEnabled ) {
                    rc = longShot();
                    if (NO_ERROR != rc) {
                        waitDeferredWork(mReprocJob);
                        waitDeferredWork(mJpegJob);
                        delChannel(QCAMERA_CH_TYPE_CAPTURE);
                        return rc;
                    }
                }
            } else {
                ALOGE("%s: cannot add capture channel", __func__);
                delChannel(QCAMERA_CH_TYPE_CAPTURE);
                return rc;
            }
        } else {

            stopChannel(QCAMERA_CH_TYPE_PREVIEW);
            delChannel(QCAMERA_CH_TYPE_PREVIEW);

            pthread_mutex_lock(&m_parm_lock);
            rc = mParameters.updateRAW(gCamCapability[mCameraId]->raw_dim[0]);
            pthread_mutex_unlock(&m_parm_lock);
            if (NO_ERROR != rc) {
                ALOGE("%s: Raw dimension update failed %d", __func__, rc);
                return rc;
            }

            rc = declareSnapshotStreams();
            if (NO_ERROR != rc) {
                ALOGE("%s: RAW stream info configuration failed %d",
                        __func__,
                        rc);
                return rc;
            }

            rc = addRawChannel();
            if (rc == NO_ERROR) {
                // start postprocessor
                waitDeferredWork(mInitPProcJob);
                rc = m_postprocessor.start(m_channels[QCAMERA_CH_TYPE_RAW]);
                if (rc != NO_ERROR) {
                    ALOGE("%s: cannot start postprocessor", __func__);
                    delChannel(QCAMERA_CH_TYPE_RAW);
                    return rc;
                }

                rc = startChannel(QCAMERA_CH_TYPE_RAW);
                if (rc != NO_ERROR) {
                    ALOGE("%s: cannot start raw channel", __func__);
                    m_postprocessor.stop();
                    delChannel(QCAMERA_CH_TYPE_RAW);
                    return rc;
                }
            } else {
                ALOGE("%s: cannot add raw channel", __func__);
                return rc;
            }
        }
    }

    //When take picture, stop sending preview callbacks to APP
    mParameters.setPreviewCallbackNeeded(false);
    CDBG_HIGH("%s: X", __func__);
    return rc;
}

/*===========================================================================
 * FUNCTION   : configureOnlineRotation
 *
 * DESCRIPTION: Configure backend with expected rotation for snapshot stream
 *
 * PARAMETERS :
 *    @ch     : Channel containing a snapshot stream
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::configureOnlineRotation(QCameraChannel &ch)
{
    int rc = NO_ERROR;
    uint32_t streamId = 0;
    QCameraStream *pStream = NULL;

    for (uint8_t i = 0; i < ch.getNumOfStreams(); i++) {
        QCameraStream *stream = ch.getStreamByIndex(i);
        if ((NULL != stream) &&
                (CAM_STREAM_TYPE_SNAPSHOT == stream->getMyType())) {
            pStream = stream;
            break;
        }
    }

    if (NULL == pStream) {
        ALOGE("%s: No snapshot stream found!", __func__);
        return BAD_VALUE;
    }

    streamId = pStream->getMyServerID();
    // Update online rotation configuration
    pthread_mutex_lock(&m_parm_lock);
    rc = mParameters.addOnlineRotation(mParameters.getJpegRotation(), streamId,
            mParameters.getDeviceRotation());
    if (rc != NO_ERROR) {
        ALOGE("%s: addOnlineRotation failed %d", __func__, rc);
        pthread_mutex_unlock(&m_parm_lock);
        return rc;
    }
    pthread_mutex_unlock(&m_parm_lock);

    return rc;
}

/*===========================================================================
 * FUNCTION   : declareSnapshotStreams
 *
 * DESCRIPTION: Configure backend with expected snapshot streams
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::declareSnapshotStreams()
{
    int rc = NO_ERROR;

    // Update stream info configuration
    pthread_mutex_lock(&m_parm_lock);
    rc = mParameters.setStreamConfigure(true, mLongshotEnabled, false);
    if (rc != NO_ERROR) {
        ALOGE("%s: setStreamConfigure failed %d", __func__, rc);
        pthread_mutex_unlock(&m_parm_lock);
        return rc;
    }
    pthread_mutex_unlock(&m_parm_lock);

    return rc;
}

/*===========================================================================
 * FUNCTION   : longShot
 *
 * DESCRIPTION: Queue one more ZSL frame
 *              in the longshot pipe.
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::longShot()
{
    int32_t rc = NO_ERROR;
    uint8_t numSnapshots = mParameters.getNumOfSnapshots();
    QCameraPicChannel *pChannel = NULL;

    if (mParameters.isZSLMode()) {
        pChannel = (QCameraPicChannel *)m_channels[QCAMERA_CH_TYPE_ZSL];
    } else {
        pChannel = (QCameraPicChannel *)m_channels[QCAMERA_CH_TYPE_CAPTURE];
    }

    if (NULL != pChannel) {
        mm_camera_req_buf_t buf;
        memset(&buf, 0x0, sizeof(buf));
        buf.type = MM_CAMERA_REQ_SUPER_BUF;
        buf.num_buf_requested = numSnapshots;
        rc = pChannel->takePicture(&buf);
    } else {
        ALOGE(" %s : Capture channel not initialized!", __func__);
        rc = NO_INIT;
        goto end;
    }

end:
    return rc;
}

/*===========================================================================
 * FUNCTION   : stopCaptureChannel
 *
 * DESCRIPTION: Stops capture channel
 *
 * PARAMETERS :
 *   @destroy : Set to true to stop and delete camera channel.
 *              Set to false to only stop capture channel.
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::stopCaptureChannel(bool destroy)
{
    int rc = NO_ERROR;
    if (mParameters.isJpegPictureFormat() ||
        mParameters.isNV16PictureFormat() ||
        mParameters.isNV21PictureFormat()) {
        rc = stopChannel(QCAMERA_CH_TYPE_CAPTURE);
        if (destroy && (NO_ERROR == rc)) {
            // Destroy camera channel but dont release context
            waitDeferredWork(mJpegJob);
            rc = delChannel(QCAMERA_CH_TYPE_CAPTURE, false);
        }
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : cancelPicture
 *
 * DESCRIPTION: cancel picture impl
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::cancelPicture()
{
    waitDeferredWork(mReprocJob);
    waitDeferredWork(mJpegJob);

    //stop post processor
    m_postprocessor.stop();

    pthread_mutex_lock(&m_parm_lock);
    unconfigureAdvancedCapture();
    pthread_mutex_unlock(&m_parm_lock);

    mParameters.setDisplayFrame(TRUE);

    if (mParameters.isZSLMode()) {
        QCameraPicChannel *pZSLChannel =
            (QCameraPicChannel *)m_channels[QCAMERA_CH_TYPE_ZSL];
        if (NULL != pZSLChannel) {
            stopAdvancedCapture(pZSLChannel);
            pZSLChannel->cancelPicture();
        }
    } else {

        // normal capture case
        if (mParameters.isJpegPictureFormat() ||
            mParameters.isNV16PictureFormat() ||
            mParameters.isNV21PictureFormat()) {
            stopChannel(QCAMERA_CH_TYPE_CAPTURE);
            delChannel(QCAMERA_CH_TYPE_CAPTURE);
        } else {
            stopChannel(QCAMERA_CH_TYPE_RAW);
            delChannel(QCAMERA_CH_TYPE_RAW);
        }
    }

    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : captureDone
 *
 * DESCRIPTION: Function called when the capture is completed before encoding
 *
 * PARAMETERS : none
 *
 * RETURN     : none
 *==========================================================================*/
void QCamera2HardwareInterface::captureDone()
{
    qcamera_sm_internal_evt_payload_t *payload =
       (qcamera_sm_internal_evt_payload_t *)
       malloc(sizeof(qcamera_sm_internal_evt_payload_t));
    if (NULL != payload) {
        memset(payload, 0, sizeof(qcamera_sm_internal_evt_payload_t));
        payload->evt_type = QCAMERA_INTERNAL_EVT_ZSL_CAPTURE_DONE;
        int32_t rc = processEvt(QCAMERA_SM_EVT_EVT_INTERNAL, payload);
        if (rc != NO_ERROR) {
            ALOGE("%s: processEvt ZSL capture done failed", __func__);
            free(payload);
            payload = NULL;
        }
    } else {
        ALOGE("%s: No memory for ZSL capture done event", __func__);
    }
}

/*===========================================================================
 * FUNCTION   : Live_Snapshot_thread
 *
 * DESCRIPTION: Seperate thread for taking live snapshot during recording
 *
 * PARAMETERS : @data - pointer to QCamera2HardwareInterface class object
 *
 * RETURN     : none
 *==========================================================================*/
void* Live_Snapshot_thread (void* data)
{

    QCamera2HardwareInterface *hw = reinterpret_cast<QCamera2HardwareInterface *>(data);
    if (!hw) {
        ALOGE("take_picture_thread: NULL camera device");
        return (void *)BAD_VALUE;
    }
    hw->takeLiveSnapshot_internal();
    return (void* )NULL;
}

/*===========================================================================
 * FUNCTION   : Int_Pic_thread
 *
 * DESCRIPTION: Seperate thread for taking snapshot triggered by camera backend
 *
 * PARAMETERS : @data - pointer to QCamera2HardwareInterface class object
 *
 * RETURN     : none
 *==========================================================================*/
void* Int_Pic_thread (void* data)
{
    int rc = NO_ERROR;

    QCamera2HardwareInterface *hw = reinterpret_cast<QCamera2HardwareInterface *>(data);

    if (!hw) {
        ALOGE("take_picture_thread: NULL camera device");
        return (void *)BAD_VALUE;
    }

    bool JpegMemOpt = false;
    char raw_format[PROPERTY_VALUE_MAX];

    memset(raw_format, 0, sizeof(raw_format));

    rc = hw->takeBackendPic_internal(&JpegMemOpt, &raw_format[0]);
    if (rc == NO_ERROR) {
        hw->checkIntPicPending(JpegMemOpt, &raw_format[0]);
    } else {
        //Snapshot attempt not successful, we need to do cleanup here
        hw->clearIntPendingEvents();
    }

    return (void* )NULL;
}

/*===========================================================================
 * FUNCTION   : takeLiveSnapshot
 *
 * DESCRIPTION: take live snapshot during recording
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::takeLiveSnapshot()
{
    int rc = NO_ERROR;
    rc= pthread_create(&mLiveSnapshotThread, NULL, Live_Snapshot_thread, (void *) this);
    if (!rc) {
        pthread_setname_np(mLiveSnapshotThread, "CAM_liveSnap");
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : takePictureInternal
 *
 * DESCRIPTION: take snapshot triggered by backend
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::takePictureInternal()
{
    int rc = NO_ERROR;
    rc= pthread_create(&mIntPicThread, NULL, Int_Pic_thread, (void *) this);
    if (!rc) {
        pthread_setname_np(mIntPicThread, "CAM_IntPic");
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : checkIntPicPending
 *
 * DESCRIPTION: timed wait for jpeg completion event, and send
 *                        back completion event to backend
 *
 * PARAMETERS : none
 *
 * RETURN     : none
 *==========================================================================*/
void QCamera2HardwareInterface::checkIntPicPending(bool JpegMemOpt, char *raw_format)
{
    bool bSendToBackend = true;
    cam_int_evt_params_t params;
    int rc = NO_ERROR;

    struct timespec   ts;
    struct timeval    tp;
    gettimeofday(&tp, NULL);
    ts.tv_sec  = tp.tv_sec + 5;
    ts.tv_nsec = tp.tv_usec * 1000;

    if (true == m_bIntJpegEvtPending ||
        (true == m_bIntRawEvtPending)) {
        //Waiting in HAL for snapshot taken notification
        pthread_mutex_lock(&m_int_lock);
        rc = pthread_cond_timedwait(&m_int_cond, &m_int_lock, &ts);
        if (ETIMEDOUT == rc || 0x0 == m_BackendFileName[0]) {
            //Hit a timeout, or some spurious activity
            bSendToBackend = false;
        }

        if (true == m_bIntJpegEvtPending) {
            params.event_type = 0;
            mParameters.getStreamFormat(CAM_STREAM_TYPE_SNAPSHOT, params.picture_format);
        } else if (true == m_bIntRawEvtPending) {
            params.event_type = 1;
            mParameters.getStreamFormat(CAM_STREAM_TYPE_RAW, params.picture_format);
        }
        pthread_mutex_unlock(&m_int_lock);

        if (true == m_bIntJpegEvtPending) {
            //Attempting to restart preview after taking JPEG snapshot
            lockAPI();
            rc = processAPI(QCAMERA_SM_EVT_SNAPSHOT_DONE, NULL);
            unlockAPI();
            m_postprocessor.setJpegMemOpt(JpegMemOpt);
        } else if (true == m_bIntRawEvtPending) {
            //Attempting to restart preview after taking RAW snapshot
            stopChannel(QCAMERA_CH_TYPE_RAW);
            delChannel(QCAMERA_CH_TYPE_RAW);
            //restoring the old raw format
            property_set("persist.camera.raw.format", raw_format);
        }

        if (true == bSendToBackend) {
            //send event back to server with the file path
            params.dim = m_postprocessor.m_dst_dim;
            memcpy(&params.path[0], &m_BackendFileName[0], QCAMERA_MAX_FILEPATH_LENGTH);
            memset(&m_BackendFileName[0], 0x0, QCAMERA_MAX_FILEPATH_LENGTH);
            params.size = mBackendFileSize;
            pthread_mutex_lock(&m_parm_lock);
            rc = mParameters.setIntEvent(params);
            pthread_mutex_unlock(&m_parm_lock);
        }

        clearIntPendingEvents();
    }

    return;
}

/*===========================================================================
 * FUNCTION   : takeBackendPic_internal
 *
 * DESCRIPTION: take snapshot triggered by backend
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::takeBackendPic_internal(bool *JpegMemOpt, char *raw_format)
{
    int rc = NO_ERROR;
    qcamera_api_result_t apiResult;

    lockAPI();
    //Set rotation value from user settings as Jpeg rotation
    //to configure back-end modules.
    mParameters.setJpegRotation(mParameters.getRotation());

    setRetroPicture(0);
    /* Prepare snapshot in case LED needs to be flashed */
    if (mFlashNeeded == 1 || mParameters.isChromaFlashEnabled()) {
        // Start Preparing for normal Frames
        CDBG_HIGH("%s: Start Prepare Snapshot", __func__);
        /* Prepare snapshot in case LED needs to be flashed */
        rc = processAPI(QCAMERA_SM_EVT_PREPARE_SNAPSHOT, NULL);
        if (rc == NO_ERROR) {
            waitAPIResult(QCAMERA_SM_EVT_PREPARE_SNAPSHOT, &apiResult);
            rc = apiResult.status;
            CDBG_HIGH("%s: Prep Snapshot done", __func__);
        }
        mPrepSnapRun = true;
    }
    unlockAPI();

    if (true == m_bIntJpegEvtPending) {
        //Attempting to take JPEG snapshot
        waitDeferredWork(mInitPProcJob);
        *JpegMemOpt = m_postprocessor.getJpegMemOpt();
        m_postprocessor.setJpegMemOpt(false);

        /* capture */
        lockAPI();
        CDBG_HIGH("%s: Capturing internal snapshot", __func__);
        rc = processAPI(QCAMERA_SM_EVT_TAKE_PICTURE, NULL);
        if (rc == NO_ERROR) {
            waitAPIResult(QCAMERA_SM_EVT_TAKE_PICTURE, &apiResult);
            rc = apiResult.status;
        }
        unlockAPI();
    } else if (true == m_bIntRawEvtPending) {
        //Attempting to take RAW snapshot
        (void)JpegMemOpt;
        stopPreview();

        //getting the existing raw format type
        property_get("persist.camera.raw.format", raw_format, "17");
        //setting it to a default know value for this task
        property_set("persist.camera.raw.format", "18");

        rc = addRawChannel();
        if (rc == NO_ERROR) {
            // start postprocessor
            waitDeferredWork(mInitPProcJob);
            rc = m_postprocessor.start(m_channels[QCAMERA_CH_TYPE_RAW]);
            if (rc != NO_ERROR) {
                ALOGE("%s: cannot start postprocessor", __func__);
                delChannel(QCAMERA_CH_TYPE_RAW);
                return rc;
            }

            rc = startChannel(QCAMERA_CH_TYPE_RAW);
            if (rc != NO_ERROR) {
                ALOGE("%s: cannot start raw channel", __func__);
                m_postprocessor.stop();
                delChannel(QCAMERA_CH_TYPE_RAW);
                return rc;
            }
        } else {
            ALOGE("%s: cannot add raw channel", __func__);
            return rc;
        }
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : clearIntPendingEvents
 *
 * DESCRIPTION: clear internal pending events pertaining to backend
 *                        snapshot requests
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
void QCamera2HardwareInterface::clearIntPendingEvents()
{
    int rc = NO_ERROR;

    if (true == m_bIntRawEvtPending) {
        preparePreview();
        startPreview();
    }
    if (true == m_bIntJpegEvtPending) {
        if (false == mParameters.isZSLMode()) {
            lockAPI();
            rc = processAPI(QCAMERA_SM_EVT_START_PREVIEW, NULL);
            unlockAPI();
        }
    }

    pthread_mutex_lock(&m_int_lock);
    if (true == m_bIntJpegEvtPending) {
        m_bIntJpegEvtPending = false;
    } else if (true == m_bIntRawEvtPending) {
        m_bIntRawEvtPending = false;
    }
    pthread_mutex_unlock(&m_int_lock);
    return;
}

/*===========================================================================
 * FUNCTION   : takeLiveSnapshot_internal
 *
 * DESCRIPTION: take live snapshot during recording
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::takeLiveSnapshot_internal()
{
    int rc = NO_ERROR;

    QCameraChannel *pChannel = NULL;

    //Set rotation value from user settings as Jpeg rotation
    //to configure back-end modules.
    mParameters.setJpegRotation(mParameters.getRotation());

    // Configure advanced capture
    if (mParameters.isUbiFocusEnabled() ||
            mParameters.isUbiRefocus() ||
            mParameters.isOptiZoomEnabled() ||
            mParameters.isHDREnabled() ||
            mParameters.isChromaFlashEnabled() ||
            mParameters.isAEBracketEnabled() ||
            mParameters.isStillMoreEnabled()) {
        rc = configureAdvancedCapture();
        if (rc != NO_ERROR) {
            CDBG_HIGH("%s: configureAdvancedCapture unsuccessful", __func__);
        }
    }

    if (isLowPowerMode()) {
        pChannel = m_channels[QCAMERA_CH_TYPE_VIDEO];
    } else {
        pChannel = m_channels[QCAMERA_CH_TYPE_SNAPSHOT];
    }

    if (NULL == pChannel) {
        ALOGE("%s: Snapshot/Video channel not initialized", __func__);
        rc = NO_INIT;
        goto end;
    }

    // start post processor
    waitDeferredWork(mInitPProcJob);
    rc = m_postprocessor.start(pChannel);

    if (NO_ERROR != rc) {
        ALOGE("%s: Post-processor start failed %d", __func__, rc);
        goto end;
    }

    if (isLowPowerMode()) {
        mm_camera_req_buf_t buf;
        memset(&buf, 0x0, sizeof(buf));
        buf.type = MM_CAMERA_REQ_SUPER_BUF;
        buf.num_buf_requested = 1;
        rc = ((QCameraVideoChannel*)pChannel)->takePicture(&buf);
        goto end;
    }

    //Disable reprocess for 4K liveshot case
    if (!mParameters.is4k2kVideoResolution()) {
        rc = configureOnlineRotation(*m_channels[QCAMERA_CH_TYPE_SNAPSHOT]);
        if (rc != NO_ERROR) {
            ALOGE("%s: online rotation failed", __func__);
            m_postprocessor.stop();
            return rc;
        }
    }
    // start snapshot channel
    if ((rc == NO_ERROR) && (NULL != pChannel)) {
        // Do not link metadata stream for 4K2k resolution
        // as CPP processing would be done on snapshot stream and not
        // reprocess stream
        if (!mParameters.is4k2kVideoResolution()) {
            // Find and try to link a metadata stream from preview channel
            QCameraChannel *pMetaChannel = NULL;
            QCameraStream *pMetaStream = NULL;

            if (m_channels[QCAMERA_CH_TYPE_PREVIEW] != NULL) {
                pMetaChannel = m_channels[QCAMERA_CH_TYPE_PREVIEW];
                uint32_t streamNum = pMetaChannel->getNumOfStreams();
                QCameraStream *pStream = NULL;
                for (uint32_t i = 0 ; i < streamNum ; i++ ) {
                    pStream = pMetaChannel->getStreamByIndex(i);
                    if ((NULL != pStream) &&
                            (CAM_STREAM_TYPE_METADATA == pStream->getMyType())) {
                        pMetaStream = pStream;
                        break;
                    }
                }
            }

            if ((NULL != pMetaChannel) && (NULL != pMetaStream)) {
                rc = pChannel->linkStream(pMetaChannel, pMetaStream);
                if (NO_ERROR != rc) {
                    ALOGE("%s : Metadata stream link failed %d", __func__, rc);
                }
            }
        }

        rc = pChannel->start();
    }

end:
    if (rc != NO_ERROR) {
        rc = processAPI(QCAMERA_SM_EVT_CANCEL_PICTURE, NULL);
        rc = sendEvtNotify(CAMERA_MSG_ERROR, UNKNOWN_ERROR, 0);
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : cancelLiveSnapshot
 *
 * DESCRIPTION: cancel current live snapshot request
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::cancelLiveSnapshot()
{
    int rc = NO_ERROR;

    pthread_mutex_lock(&m_parm_lock);
    unconfigureAdvancedCapture();
    pthread_mutex_unlock(&m_parm_lock);

    if (mLiveSnapshotThread != 0) {
        pthread_join(mLiveSnapshotThread,NULL);
        mLiveSnapshotThread = 0;
    }

    //stop post processor
    m_postprocessor.stop();

    // stop snapshot channel
    rc = stopChannel(QCAMERA_CH_TYPE_SNAPSHOT);

    return rc;
}

/*===========================================================================
 * FUNCTION   : getParameters
 *
 * DESCRIPTION: get parameters impl
 *
 * PARAMETERS : none
 *
 * RETURN     : a string containing parameter pairs
 *==========================================================================*/
char* QCamera2HardwareInterface::getParameters()
{
    char* strParams = NULL;
    String8 str("");

    int cur_width, cur_height;
    pthread_mutex_lock(&m_parm_lock);
    //Need take care Scale picture size
    if(mParameters.m_reprocScaleParam.isScaleEnabled() &&
        mParameters.m_reprocScaleParam.isUnderScaling()){
        int scale_width, scale_height;

        mParameters.m_reprocScaleParam.getPicSizeFromAPK(scale_width,scale_height);
        mParameters.getPictureSize(&cur_width, &cur_height);

        String8 pic_size("");
        char buffer[32];
        snprintf(buffer, sizeof(buffer), "%dx%d", scale_width, scale_height);
        pic_size.append(buffer);
        mParameters.set(CameraParameters::KEY_PICTURE_SIZE, pic_size);
    }

    str = mParameters.flatten( );
    strParams = (char *)malloc(sizeof(char)*(str.length()+1));
    if(strParams != NULL){
        memset(strParams, 0, sizeof(char)*(str.length()+1));
        strlcpy(strParams, str.string(), str.length()+1);
        strParams[str.length()] = 0;
    }

    if(mParameters.m_reprocScaleParam.isScaleEnabled() &&
        mParameters.m_reprocScaleParam.isUnderScaling()){
        //need set back picture size
        String8 pic_size("");
        char buffer[32];
        snprintf(buffer, sizeof(buffer), "%dx%d", cur_width, cur_height);
        pic_size.append(buffer);
        mParameters.set(CameraParameters::KEY_PICTURE_SIZE, pic_size);
    }
    pthread_mutex_unlock(&m_parm_lock);
    return strParams;
}

/*===========================================================================
 * FUNCTION   : putParameters
 *
 * DESCRIPTION: put parameters string impl
 *
 * PARAMETERS :
 *   @parms   : parameters string to be released
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::putParameters(char *parms)
{
    free(parms);
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : sendCommand
 *
 * DESCRIPTION: send command impl
 *
 * PARAMETERS :
 *   @command : command to be executed
 *   @arg1    : optional argument 1
 *   @arg2    : optional argument 2
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::sendCommand(int32_t command,
        int32_t &arg1, int32_t &arg2)
{
    int rc = NO_ERROR;

    switch (command) {
#ifndef VANILLA_HAL
    case CAMERA_CMD_LONGSHOT_ON:
        arg1 = arg2 = 0;
        // Longshot can only be enabled when image capture
        // is not active.
        if ( !m_stateMachine.isCaptureRunning() ) {
            CDBG_HIGH("%s: Longshot Enabled", __func__);
            mLongshotEnabled = true;
            pthread_mutex_lock(&m_parm_lock);
            rc = mParameters.setLongshotEnable(mLongshotEnabled);
            pthread_mutex_unlock(&m_parm_lock);

            // Due to recent buffer count optimizations
            // ZSL might run with considerably less buffers
            // when not in longshot mode. Preview needs to
            // restart in this case.
            if (isZSLMode() && m_stateMachine.isPreviewRunning()) {
                QCameraChannel *pChannel = NULL;
                QCameraStream *pSnapStream = NULL;
                pChannel = m_channels[QCAMERA_CH_TYPE_ZSL];
                if (NULL != pChannel) {
                    QCameraStream *pStream = NULL;
                    for (uint32_t i = 0; i < pChannel->getNumOfStreams(); i++) {
                        pStream = pChannel->getStreamByIndex(i);
                        if (pStream != NULL) {
                            if (pStream->isTypeOf(CAM_STREAM_TYPE_SNAPSHOT)) {
                                pSnapStream = pStream;
                                break;
                            }
                        }
                    }
                    if (NULL != pSnapStream) {
                        uint8_t required = 0;
                        required = getBufNumRequired(CAM_STREAM_TYPE_SNAPSHOT);
                        if (pSnapStream->getBufferCount() < required) {
                            // We restart here, to reset the FPS and no
                            // of buffers as per the requirement of longshot usecase.
                            arg1 = QCAMERA_SM_EVT_RESTART_PERVIEW;
                            if (getRelatedCamSyncInfo()->sync_control ==
                                    CAM_SYNC_RELATED_SENSORS_ON) {
                                arg2 = QCAMERA_SM_EVT_DELAYED_RESTART;
                            }
                        }
                    }
                }
            }
            //
            mPrepSnapRun = false;
            mCACDoneReceived = FALSE;
        } else {
            rc = NO_INIT;
        }
        break;
    case CAMERA_CMD_LONGSHOT_OFF:
        if ( mLongshotEnabled && m_stateMachine.isCaptureRunning() ) {
            cancelPicture();
            processEvt(QCAMERA_SM_EVT_SNAPSHOT_DONE, NULL);
            QCameraChannel *pZSLChannel = m_channels[QCAMERA_CH_TYPE_ZSL];
            if (isZSLMode() && (NULL != pZSLChannel) && mPrepSnapRun) {
                mCameraHandle->ops->stop_zsl_snapshot(
                        mCameraHandle->camera_handle,
                        pZSLChannel->getMyHandle());
            }
        }
        mPrepSnapRun = false;
        CDBG_HIGH("%s: Longshot Disabled", __func__);
        mLongshotEnabled = false;
        pthread_mutex_lock(&m_parm_lock);
        rc = mParameters.setLongshotEnable(mLongshotEnabled);
        pthread_mutex_unlock(&m_parm_lock);
        mCACDoneReceived = FALSE;
        break;
    case CAMERA_CMD_HISTOGRAM_ON:
    case CAMERA_CMD_HISTOGRAM_OFF:
        rc = setHistogram(command == CAMERA_CMD_HISTOGRAM_ON? true : false);
        CDBG_HIGH("%s: Histogram -> %s", __func__,
              mParameters.isHistogramEnabled() ? "Enabled" : "Disabled");
        break;
#endif
    case CAMERA_CMD_START_FACE_DETECTION:
    case CAMERA_CMD_STOP_FACE_DETECTION:
        mParameters.setFaceDetectionOption(command == CAMERA_CMD_START_FACE_DETECTION? true : false);
        rc = setFaceDetection(command == CAMERA_CMD_START_FACE_DETECTION? true : false);
        CDBG_HIGH("%s: FaceDetection -> %s", __func__,
              mParameters.isFaceDetectionEnabled() ? "Enabled" : "Disabled");
        break;
#ifndef VANILLA_HAL
    case CAMERA_CMD_HISTOGRAM_SEND_DATA:
#endif
    default:
        rc = NO_ERROR;
        break;
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : registerFaceImage
 *
 * DESCRIPTION: register face image impl
 *
 * PARAMETERS :
 *   @img_ptr : ptr to image buffer
 *   @config  : ptr to config struct about input image info
 *   @faceID  : [OUT] face ID to uniquely identifiy the registered face image
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::registerFaceImage(void *img_ptr,
                                                 cam_pp_offline_src_config_t *config,
                                                 int32_t &faceID)
{
    int rc = NO_ERROR;
    faceID = -1;

    if (img_ptr == NULL || config == NULL) {
        ALOGE("%s: img_ptr or config is NULL", __func__);
        return BAD_VALUE;
    }

    // allocate ion memory for source image
    QCameraHeapMemory *imgBuf = new QCameraHeapMemory(QCAMERA_ION_USE_CACHE);
    if (imgBuf == NULL) {
        ALOGE("%s: Unable to new heap memory obj for image buf", __func__);
        return NO_MEMORY;
    }

    rc = imgBuf->allocate(1, config->input_buf_planes.plane_info.frame_len, NON_SECURE);
    if (rc < 0) {
        ALOGE("%s: Unable to allocate heap memory for image buf", __func__);
        delete imgBuf;
        return NO_MEMORY;
    }

    void *pBufPtr = imgBuf->getPtr(0);
    if (pBufPtr == NULL) {
        ALOGE("%s: image buf is NULL", __func__);
        imgBuf->deallocate();
        delete imgBuf;
        return NO_MEMORY;
    }
    memcpy(pBufPtr, img_ptr, config->input_buf_planes.plane_info.frame_len);

    cam_pp_feature_config_t pp_feature;
    memset(&pp_feature, 0, sizeof(cam_pp_feature_config_t));
    pp_feature.feature_mask = CAM_QCOM_FEATURE_REGISTER_FACE;
    QCameraReprocessChannel *pChannel =
        addOfflineReprocChannel(*config, pp_feature, NULL, NULL);

    if (pChannel == NULL) {
        ALOGE("%s: fail to add offline reprocess channel", __func__);
        imgBuf->deallocate();
        delete imgBuf;
        return UNKNOWN_ERROR;
    }

    rc = pChannel->start();
    if (rc != NO_ERROR) {
        ALOGE("%s: Cannot start reprocess channel", __func__);
        imgBuf->deallocate();
        delete imgBuf;
        delete pChannel;
        return rc;
    }

    ssize_t bufSize = imgBuf->getSize(0);
    if (BAD_INDEX != bufSize) {
        rc = pChannel->doReprocess(imgBuf->getFd(0), (size_t)bufSize, faceID);
    } else {
        ALOGE("Failed to retrieve buffer size (bad index)");
        return UNKNOWN_ERROR;
    }

    // done with register face image, free imgbuf and delete reprocess channel
    imgBuf->deallocate();
    delete imgBuf;
    imgBuf = NULL;
    pChannel->stop();
    delete pChannel;
    pChannel = NULL;

    return rc;
}

/*===========================================================================
 * FUNCTION   : release
 *
 * DESCRIPTION: release camera resource impl
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::release()
{
    // stop and delete all channels
    for (int i = 0; i <QCAMERA_CH_TYPE_MAX ; i++) {
        if (m_channels[i] != NULL) {
            stopChannel((qcamera_ch_type_enum_t)i);
            delChannel((qcamera_ch_type_enum_t)i);
        }
    }

    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : dump
 *
 * DESCRIPTION: camera status dump impl
 *
 * PARAMETERS :
 *   @fd      : fd for the buffer to be dumped with camera status
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::dump(int fd)
{
    dprintf(fd, "\n Camera HAL information Begin \n");
    dprintf(fd, "Camera ID: %d \n", mCameraId);
    dprintf(fd, "StoreMetaDataInFrame: %d \n", mStoreMetaDataInFrame);
    dprintf(fd, "\n Configuration: %s", mParameters.dump().string());
    dprintf(fd, "\n State Information: %s", m_stateMachine.dump().string());
    dprintf(fd, "\n Camera HAL information End \n");

    /* send UPDATE_DEBUG_LEVEL to the backend so that they can read the
       debug level property */
    pthread_mutex_lock(&m_parm_lock);
    mParameters.updateDebugLevel();
    pthread_mutex_unlock(&m_parm_lock);
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : processAPI
 *
 * DESCRIPTION: process API calls from upper layer
 *
 * PARAMETERS :
 *   @api         : API to be processed
 *   @api_payload : ptr to API payload if any
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::processAPI(qcamera_sm_evt_enum_t api, void *api_payload)
{
    int ret = DEAD_OBJECT;

    if (m_smThreadActive) {
        ret = m_stateMachine.procAPI(api, api_payload);
    }

    return ret;
}

/*===========================================================================
 * FUNCTION   : processEvt
 *
 * DESCRIPTION: process Evt from backend via mm-camera-interface
 *
 * PARAMETERS :
 *   @evt         : event type to be processed
 *   @evt_payload : ptr to event payload if any
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::processEvt(qcamera_sm_evt_enum_t evt, void *evt_payload)
{
    return m_stateMachine.procEvt(evt, evt_payload);
}

/*===========================================================================
 * FUNCTION   : processSyncEvt
 *
 * DESCRIPTION: process synchronous Evt from backend
 *
 * PARAMETERS :
 *   @evt         : event type to be processed
 *   @evt_payload : ptr to event payload if any
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::processSyncEvt(qcamera_sm_evt_enum_t evt, void *evt_payload)
{
    int rc = NO_ERROR;

    pthread_mutex_lock(&m_evtLock);
    rc =  processEvt(evt, evt_payload);
    if (rc == NO_ERROR) {
        memset(&m_evtResult, 0, sizeof(qcamera_api_result_t));
        while (m_evtResult.request_api != evt) {
            pthread_cond_wait(&m_evtCond, &m_evtLock);
        }
        rc =  m_evtResult.status;
    }
    pthread_mutex_unlock(&m_evtLock);

    return rc;
}

/*===========================================================================
 * FUNCTION   : evtHandle
 *
 * DESCRIPTION: Function registerd to mm-camera-interface to handle backend events
 *
 * PARAMETERS :
 *   @camera_handle : event type to be processed
 *   @evt           : ptr to event
 *   @user_data     : user data ptr
 *
 * RETURN     : none
 *==========================================================================*/
void QCamera2HardwareInterface::camEvtHandle(uint32_t /*camera_handle*/,
                                          mm_camera_event_t *evt,
                                          void *user_data)
{
    QCamera2HardwareInterface *obj = (QCamera2HardwareInterface *)user_data;
    if (obj && evt) {
        mm_camera_event_t *payload =
            (mm_camera_event_t *)malloc(sizeof(mm_camera_event_t));
        if (NULL != payload) {
            *payload = *evt;
            //peek into the event, if this is an eztune event from server,
            //then we don't need to post it to the SM Qs, we shud directly
            //spawn a thread and get the job done (jpeg or raw snapshot)
            switch (payload->server_event_type) {
                case CAM_EVENT_TYPE_INT_TAKE_JPEG:
                    //Received JPEG trigger from eztune
                    if (false == obj->m_bIntJpegEvtPending) {
                        pthread_mutex_lock(&obj->m_int_lock);
                        obj->m_bIntJpegEvtPending = true;
                        pthread_mutex_unlock(&obj->m_int_lock);
                        obj->takePictureInternal();
                    }
                    free(payload);
                    break;
                case CAM_EVENT_TYPE_INT_TAKE_RAW:
                    //Received RAW trigger from eztune
                    if (false == obj->m_bIntRawEvtPending) {
                        pthread_mutex_lock(&obj->m_int_lock);
                        obj->m_bIntRawEvtPending = true;
                        pthread_mutex_unlock(&obj->m_int_lock);
                        obj->takePictureInternal();
                    }
                    free(payload);
                    break;
                case CAM_EVENT_TYPE_DAEMON_DIED:
                    {
                        Mutex::Autolock l(obj->mDefLock);
                        obj->mDefCond.broadcast();
                        CDBG_HIGH("%s: broadcast mDefCond signal\n", __func__);
                    }
                default:
                    obj->processEvt(QCAMERA_SM_EVT_EVT_NOTIFY, payload);
                    break;
            }
        }
    } else {
        ALOGE("%s: NULL user_data", __func__);
    }
}

/*===========================================================================
 * FUNCTION   : jpegEvtHandle
 *
 * DESCRIPTION: Function registerd to mm-jpeg-interface to handle jpeg events
 *
 * PARAMETERS :
 *   @status    : status of jpeg job
 *   @client_hdl: jpeg client handle
 *   @jobId     : jpeg job Id
 *   @p_ouput   : ptr to jpeg output result struct
 *   @userdata  : user data ptr
 *
 * RETURN     : none
 *==========================================================================*/
void QCamera2HardwareInterface::jpegEvtHandle(jpeg_job_status_t status,
                                              uint32_t /*client_hdl*/,
                                              uint32_t jobId,
                                              mm_jpeg_output_t *p_output,
                                              void *userdata)
{
    QCamera2HardwareInterface *obj = (QCamera2HardwareInterface *)userdata;
    if (obj) {
        qcamera_jpeg_evt_payload_t *payload =
            (qcamera_jpeg_evt_payload_t *)malloc(sizeof(qcamera_jpeg_evt_payload_t));
        if (NULL != payload) {
            memset(payload, 0, sizeof(qcamera_jpeg_evt_payload_t));
            payload->status = status;
            payload->jobId = jobId;
            if (p_output != NULL) {
                payload->out_data = *p_output;
            }
            obj->processEvt(QCAMERA_SM_EVT_JPEG_EVT_NOTIFY, payload);
        }
    } else {
        ALOGE("%s: NULL user_data", __func__);
    }
}

/*===========================================================================
 * FUNCTION   : thermalEvtHandle
 *
 * DESCRIPTION: routine to handle thermal event notification
 *
 * PARAMETERS :
 *   @level      : thermal level
 *   @userdata   : userdata passed in during registration
 *   @data       : opaque data from thermal client
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::thermalEvtHandle(
        qcamera_thermal_level_enum_t *level, void *userdata, void *data)
{
    if (!mCameraOpened) {
        CDBG_HIGH("%s: Camera is not opened, no need to handle thermal evt", __func__);
        return NO_ERROR;
    }

    // Make sure thermal events are logged
    CDBG_HIGH("%s: level = %d, userdata = %p, data = %p",
        __func__, *level, userdata, data);
    //We don't need to lockAPI, waitAPI here. QCAMERA_SM_EVT_THERMAL_NOTIFY
    // becomes an aync call. This also means we can only pass payload
    // by value, not by address.
    return processAPI(QCAMERA_SM_EVT_THERMAL_NOTIFY, (void *)level);
}

/*===========================================================================
 * FUNCTION   : sendEvtNotify
 *
 * DESCRIPTION: send event notify to notify thread
 *
 * PARAMETERS :
 *   @msg_type: msg type to be sent
 *   @ext1    : optional extension1
 *   @ext2    : optional extension2
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::sendEvtNotify(int32_t msg_type,
                                                 int32_t ext1,
                                                 int32_t ext2)
{
    qcamera_callback_argm_t cbArg;
    memset(&cbArg, 0, sizeof(qcamera_callback_argm_t));
    cbArg.cb_type = QCAMERA_NOTIFY_CALLBACK;
    cbArg.msg_type = msg_type;
    cbArg.ext1 = ext1;
    cbArg.ext2 = ext2;
    return m_cbNotifier.notifyCallback(cbArg);
}

/*===========================================================================
 * FUNCTION   : processAEInfo
 *
 * DESCRIPTION: process AE updates
 *
 * PARAMETERS :
 *   @ae_params: current AE parameters
 *
 * RETURN     : None
 *==========================================================================*/
int32_t QCamera2HardwareInterface::processAEInfo(cam_3a_params_t &ae_params)
{
    pthread_mutex_lock(&m_parm_lock);
    mParameters.updateAEInfo(ae_params);
    if (mParameters.isInstantAECEnabled()) {
        // Reset Instant AEC info only if instant aec enabled.
        bool bResetInstantAec = false;
        if (ae_params.settled)
        {
            // If AEC settled, reset instant AEC
            bResetInstantAec = true;
        } else if (mParameters.isInstantCaptureEnabled() &&
                mInstantAecFrameCount >= mParameters.getAecFrameBoundValue()) {
            // if AEC not settled, and instant capture enabled,
            // reset instant AEC only when frame count is
            // more or equal to AEC frame bound value.
            bResetInstantAec = true;
        } else if (mParameters.isInstantAECEnabled() &&
                mInstantAecFrameCount >= mParameters.getAecSkipDisplayFrameBound()) {
            // if AEC not settled, and only instant AEC enabled,
            // reset instant AEC only when frame count is
            // more or equal to AEC skip display frame bound value.
            bResetInstantAec = true;
        }

        if (bResetInstantAec) {
            CDBG("setting instant AEC to false");
            mParameters.setInstantAEC(false, true);
            mInstantAecFrameCount = 0;
        }
    }
    pthread_mutex_unlock(&m_parm_lock);
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : processFocusPositionInfo
 *
 * DESCRIPTION: process AF updates
 *
 * PARAMETERS :
 *   @cur_pos_info: current lens position
 *
 * RETURN     : None
 *==========================================================================*/
int32_t QCamera2HardwareInterface::processFocusPositionInfo(cam_focus_pos_info_t &cur_pos_info)
{
    pthread_mutex_lock(&m_parm_lock);
    mParameters.updateCurrentFocusPosition(cur_pos_info);
    pthread_mutex_unlock(&m_parm_lock);
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : processAutoFocusEvent
 *
 * DESCRIPTION: process auto focus event
 *
 * PARAMETERS :
 *   @focus_data: struct containing auto focus result info
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::processAutoFocusEvent(cam_auto_focus_data_t &focus_data)
{
    int32_t ret = NO_ERROR;
    CDBG_HIGH("%s: E",__func__);

    if (getRelatedCamSyncInfo()->mode == CAM_MODE_SECONDARY) {
        // Ignore focus updates
        CDBG_HIGH("%s: X Secondary Camera, no need to process!! ", __func__);
        return ret;
    }
    cam_focus_mode_type focusMode = mParameters.getFocusMode();
    CDBG_HIGH("[AF_DBG] %s: focusMode=%d, focusState=%d",
            __func__, focusMode, focus_data.focus_state);

    switch (focusMode) {
    case CAM_FOCUS_MODE_AUTO:
    case CAM_FOCUS_MODE_MACRO:
        // ignore AF event if AF was already cancelled meanwhile
        if (!mActiveAF) {
            break;
        }
        // If the HAL focus mode is different from AF INFINITY focus mode, send event to app
        if ((focus_data.focus_mode == CAM_FOCUS_MODE_INFINITY) &&
                (focus_data.focus_state == CAM_AF_STATE_INACTIVE)) {
            ret = sendEvtNotify(CAMERA_MSG_FOCUS, true, 0);
            mActiveAF = false; // reset the mActiveAF in this special case
            break;
        }

        //while transitioning from CAF->Auto/Macro, we might receive CAF related
        //events (PASSIVE_*) due to timing. Ignore such events if any.
        if ((focus_data.focus_state == CAM_AF_STATE_PASSIVE_SCAN) ||
                (focus_data.focus_state == CAM_AF_STATE_PASSIVE_FOCUSED) ||
                (focus_data.focus_state == CAM_AF_STATE_PASSIVE_UNFOCUSED)) {
            break;
        }

        //This is just an intermediate update to HAL indicating focus is in progress. No need
        //to send this event to app. Same applies to INACTIVE state as well.
        if ((focus_data.focus_state == CAM_AF_STATE_ACTIVE_SCAN) ||
                (focus_data.focus_state == CAM_AF_STATE_INACTIVE)) {
            break;
        }
        // update focus distance
        pthread_mutex_lock(&m_parm_lock);
        mParameters.updateFocusDistances(&focus_data.focus_dist);
        pthread_mutex_unlock(&m_parm_lock);

        //flush any old snapshot frames in ZSL Q which are not focused.
        if ((CAM_AF_STATE_FOCUSED_LOCKED == focus_data.focus_state) &&
                mParameters.isZSLMode()) {
            QCameraPicChannel *pZSLChannel =
                    (QCameraPicChannel *)m_channels[QCAMERA_CH_TYPE_ZSL];
            if (NULL != pZSLChannel) {
                //flush the zsl-buffer
                uint32_t flush_frame_idx = focus_data.focused_frame_idx;
                CDBG("%s, flush the zsl-buffer before frame = %u.", __func__, flush_frame_idx);
                pZSLChannel->flushSuperbuffer(flush_frame_idx);
            }
        }

        //send event to app finally
        ret = sendEvtNotify(CAMERA_MSG_FOCUS,
                            (focus_data.focus_state == CAM_AF_STATE_FOCUSED_LOCKED), 0);
        break;
    case CAM_FOCUS_MODE_CONTINOUS_VIDEO:
    case CAM_FOCUS_MODE_CONTINOUS_PICTURE:

        // If the HAL focus mode is different from AF INFINITY focus mode, send event to app
        if ((focus_data.focus_mode == CAM_FOCUS_MODE_INFINITY) &&
                (focus_data.focus_state == CAM_AF_STATE_INACTIVE)) {
            ret = sendEvtNotify(CAMERA_MSG_FOCUS, false, 0);
            mActiveAF = false; // reset the mActiveAF in this special case
            break;
        }

        //If AutoFocus() is triggered while in CAF mode, ignore all CAF events (PASSIVE_*) and
        //process/wait for only ACTIVE_* events.
        if (((focus_data.focus_state == CAM_AF_STATE_PASSIVE_FOCUSED) ||
                (focus_data.focus_state == CAM_AF_STATE_PASSIVE_UNFOCUSED) ||
                (focus_data.focus_state == CAM_AF_STATE_PASSIVE_SCAN)) && mActiveAF) {
            break;
        }

        //These are the AF states for which we need to send notification to app in CAF mode.
        //This includes both regular CAF (PASSIVE) events as well as ACTIVE events ( in case
        //AF is triggered while in CAF mode)
        if ((focus_data.focus_state == CAM_AF_STATE_PASSIVE_FOCUSED) ||
                (focus_data.focus_state == CAM_AF_STATE_PASSIVE_UNFOCUSED) ||
                (focus_data.focus_state == CAM_AF_STATE_FOCUSED_LOCKED) ||
                (focus_data.focus_state == CAM_AF_STATE_NOT_FOCUSED_LOCKED)) {

            // update focus distance
            pthread_mutex_lock(&m_parm_lock);
            mParameters.updateFocusDistances(&focus_data.focus_dist);
            pthread_mutex_unlock(&m_parm_lock);

            if ((focusMode == CAM_FOCUS_MODE_CONTINOUS_PICTURE) &&
                    (CAM_AF_STATE_FOCUSED_LOCKED == focus_data.focus_state) &&
                    mParameters.isZSLMode()) {
                QCameraPicChannel *pZSLChannel =
                        (QCameraPicChannel *)m_channels[QCAMERA_CH_TYPE_ZSL];
                if (NULL != pZSLChannel) {
                    //flush the zsl-buffer
                    uint32_t flush_frame_idx = focus_data.focused_frame_idx;
                    CDBG("%s, flush the zsl-buffer before frame = %u.", __func__, flush_frame_idx);
                    pZSLChannel->flushSuperbuffer(flush_frame_idx);
                }
            }

            ret = sendEvtNotify(CAMERA_MSG_FOCUS,
                    ((focus_data.focus_state == CAM_AF_STATE_PASSIVE_FOCUSED) ||
                    (focus_data.focus_state == CAM_AF_STATE_FOCUSED_LOCKED)), 0);
        }
        ret = sendEvtNotify(CAMERA_MSG_FOCUS_MOVE,
                (focus_data.focus_state == CAM_AF_STATE_PASSIVE_SCAN), 0);
        break;
    case CAM_FOCUS_MODE_INFINITY:
    case CAM_FOCUS_MODE_FIXED:
    case CAM_FOCUS_MODE_EDOF:
    default:
        CDBG_HIGH("%s: no ops for autofocus event in focusmode %d", __func__, focusMode);
        break;
    }

    //Reset mActiveAF once we receive focus done event
    if ((focus_data.focus_state == CAM_AF_STATE_FOCUSED_LOCKED) ||
            (focus_data.focus_state == CAM_AF_STATE_NOT_FOCUSED_LOCKED)) {
        mActiveAF = false;
    }

    CDBG_HIGH("%s: X",__func__);
    return ret;
}

/*===========================================================================
 * FUNCTION   : processZoomEvent
 *
 * DESCRIPTION: process zoom event
 *
 * PARAMETERS :
 *   @crop_info : crop info as a result of zoom operation
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::processZoomEvent(cam_crop_data_t &crop_info)
{
    int32_t ret = NO_ERROR;

    for (int i = 0; i < QCAMERA_CH_TYPE_MAX; i++) {
        if (m_channels[i] != NULL) {
            ret = m_channels[i]->processZoomDone(mPreviewWindow, crop_info);
        }
    }
    return ret;
}

/*===========================================================================
 * FUNCTION   : processZSLCaptureDone
 *
 * DESCRIPTION: process ZSL capture done events
 *
 * PARAMETERS : None
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::processZSLCaptureDone()
{
    int rc = NO_ERROR;

    pthread_mutex_lock(&m_parm_lock);
    if (++mInputCount >= mParameters.getBurstCountForAdvancedCapture()) {
        rc = unconfigureAdvancedCapture();
    }
    pthread_mutex_unlock(&m_parm_lock);

    return rc;
}

/*===========================================================================
 * FUNCTION   : processRetroAECUnlock
 *
 * DESCRIPTION: process retro burst AEC unlock events
 *
 * PARAMETERS : None
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::processRetroAECUnlock()
{
    int rc = NO_ERROR;

    CDBG_HIGH("%s : [ZSL Retro] LED assisted AF Release AEC Lock", __func__);
    pthread_mutex_lock(&m_parm_lock);
    rc = mParameters.setAecLock("false");
    if (NO_ERROR != rc) {
        ALOGE("%s: Error setting AEC lock", __func__);
        pthread_mutex_unlock(&m_parm_lock);
        return rc;
    }

    rc = mParameters.commitParameters();
    if (NO_ERROR != rc) {
        ALOGE("%s: Error during camera parameter commit", __func__);
    } else {
        m_bLedAfAecLock = FALSE;
    }

    pthread_mutex_unlock(&m_parm_lock);

    return rc;
}

/*===========================================================================
 * FUNCTION   : processHDRData
 *
 * DESCRIPTION: process HDR scene events
 *
 * PARAMETERS :
 *   @hdr_scene : HDR scene event data
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::processHDRData(cam_asd_hdr_scene_data_t hdr_scene)
{
    int rc = NO_ERROR;

#ifndef VANILLA_HAL
    if (hdr_scene.is_hdr_scene &&
      (hdr_scene.hdr_confidence > HDR_CONFIDENCE_THRESHOLD) &&
      mParameters.isAutoHDREnabled()) {
        m_HDRSceneEnabled = true;
    } else {
        m_HDRSceneEnabled = false;
    }
    pthread_mutex_lock(&m_parm_lock);
    mParameters.setHDRSceneEnable(m_HDRSceneEnabled);
    pthread_mutex_unlock(&m_parm_lock);

    if ( msgTypeEnabled(CAMERA_MSG_META_DATA) ) {

        size_t data_len = sizeof(int);
        size_t buffer_len = 1 *sizeof(int)       //meta type
                          + 1 *sizeof(int)       //data len
                          + 1 *sizeof(int);      //data
        camera_memory_t *hdrBuffer = mGetMemory(-1,
                                                 buffer_len,
                                                 1,
                                                 mCallbackCookie);
        if ( NULL == hdrBuffer ) {
            ALOGE("%s: Not enough memory for auto HDR data",
                  __func__);
            return NO_MEMORY;
        }

        int *pHDRData = (int *)hdrBuffer->data;
        if (pHDRData == NULL) {
            ALOGE("%s: memory data ptr is NULL", __func__);
            return UNKNOWN_ERROR;
        }

        pHDRData[0] = CAMERA_META_DATA_HDR;
        pHDRData[1] = (int)data_len;
        pHDRData[2] = m_HDRSceneEnabled;

        qcamera_callback_argm_t cbArg;
        memset(&cbArg, 0, sizeof(qcamera_callback_argm_t));
        cbArg.cb_type = QCAMERA_DATA_CALLBACK;
        cbArg.msg_type = CAMERA_MSG_META_DATA;
        cbArg.data = hdrBuffer;
        cbArg.user_data = hdrBuffer;
        cbArg.cookie = this;
        cbArg.release_cb = releaseCameraMemory;
        rc = m_cbNotifier.notifyCallback(cbArg);
        if (rc != NO_ERROR) {
            ALOGE("%s: fail sending auto HDR notification", __func__);
            hdrBuffer->release(hdrBuffer);
        }
    }

    CDBG_HIGH("%s : hdr_scene_data: processHDRData: %d %f",
          __func__,
          hdr_scene.is_hdr_scene,
          hdr_scene.hdr_confidence);

#endif
  return rc;
}

/*===========================================================================
 * FUNCTION   : transAwbMetaToParams
 *
 * DESCRIPTION: translate awb params from metadata callback to QCameraParameters
 *
 * PARAMETERS :
 *   @awb_params : awb params from metadata callback
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::transAwbMetaToParams(cam_awb_params_t &awb_params)
{
    pthread_mutex_lock(&m_parm_lock);
    mParameters.updateAWBParams(awb_params);
    pthread_mutex_unlock(&m_parm_lock);
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : processPrepSnapshotDone
 *
 * DESCRIPTION: process prep snapshot done event
 *
 * PARAMETERS :
 *   @prep_snapshot_state  : state of prepare snapshot done. In other words,
 *                           i.e. whether need future frames for capture.
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::processPrepSnapshotDoneEvent(
                        cam_prep_snapshot_state_t prep_snapshot_state)
{
    int32_t ret = NO_ERROR;

    if (m_channels[QCAMERA_CH_TYPE_ZSL] &&
        prep_snapshot_state == NEED_FUTURE_FRAME) {
        CDBG_HIGH("%s: already handled in mm-camera-intf, no ops here", __func__);
        if (isRetroPicture()) {
            pthread_mutex_lock(&m_parm_lock);
            mParameters.setAecLock("true");
            mParameters.commitParameters();
            pthread_mutex_unlock(&m_parm_lock);
            m_bLedAfAecLock = TRUE;
        }
    }
    return ret;
}

/*===========================================================================
 * FUNCTION   : processASDUpdate
 *
 * DESCRIPTION: process ASD update event
 *
 * PARAMETERS :
 *   @scene: selected scene mode
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::processASDUpdate(cam_auto_scene_t scene)
{
    //set ASD parameter
    pthread_mutex_lock(&m_parm_lock);
    mParameters.set(QCameraParameters::KEY_SELECTED_AUTO_SCENE, mParameters.getASDStateString(scene));
    pthread_mutex_unlock(&m_parm_lock);

    size_t data_len = sizeof(cam_auto_scene_t);
    size_t buffer_len = 1 *sizeof(int)       //meta type
                      + 1 *sizeof(int)       //data len
                      + data_len;            //data
    camera_memory_t *asdBuffer = mGetMemory(-1,
                                             buffer_len,
                                             1,
                                             mCallbackCookie);
    if ( NULL == asdBuffer ) {
        ALOGE("%s: Not enough memory for histogram data", __func__);
        return NO_MEMORY;
    }

    int *pASDData = (int *)asdBuffer->data;
    if (pASDData == NULL) {
        ALOGE("%s: memory data ptr is NULL", __func__);
        return UNKNOWN_ERROR;
    }

#ifndef VANILLA_HAL
    pASDData[0] = CAMERA_META_DATA_ASD;
    pASDData[1] = (int)data_len;
    pASDData[2] = scene;

    qcamera_callback_argm_t cbArg;
    memset(&cbArg, 0, sizeof(qcamera_callback_argm_t));
    cbArg.cb_type = QCAMERA_DATA_CALLBACK;
    cbArg.msg_type = CAMERA_MSG_META_DATA;
    cbArg.data = asdBuffer;
    cbArg.user_data = asdBuffer;
    cbArg.cookie = this;
    cbArg.release_cb = releaseCameraMemory;
    int32_t rc = m_cbNotifier.notifyCallback(cbArg);
    if (rc != NO_ERROR) {
        ALOGE("%s: fail sending notification", __func__);
        asdBuffer->release(asdBuffer);
    }
#endif
    return NO_ERROR;

}

/*===========================================================================
 * FUNCTION   : processJpegNotify
 *
 * DESCRIPTION: process jpeg event
 *
 * PARAMETERS :
 *   @jpeg_evt: ptr to jpeg event payload
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::processJpegNotify(qcamera_jpeg_evt_payload_t *jpeg_evt)
{
    return m_postprocessor.processJpegEvt(jpeg_evt);
}

/*===========================================================================
 * FUNCTION   : lockAPI
 *
 * DESCRIPTION: lock to process API
 *
 * PARAMETERS : none
 *
 * RETURN     : none
 *==========================================================================*/
void QCamera2HardwareInterface::lockAPI()
{
    pthread_mutex_lock(&m_lock);
}

/*===========================================================================
 * FUNCTION   : waitAPIResult
 *
 * DESCRIPTION: wait for API result coming back. This is a blocking call, it will
 *              return only cerntain API event type arrives
 *
 * PARAMETERS :
 *   @api_evt : API event type
 *
 * RETURN     : none
 *==========================================================================*/
void QCamera2HardwareInterface::waitAPIResult(qcamera_sm_evt_enum_t api_evt,
        qcamera_api_result_t *apiResult)
{
    CDBG("%s: wait for API result of evt (%d)", __func__, api_evt);
    int resultReceived = 0;
    while  (!resultReceived) {
        pthread_cond_wait(&m_cond, &m_lock);
        if (m_apiResultList != NULL) {
            api_result_list *apiResultList = m_apiResultList;
            api_result_list *apiResultListPrevious = m_apiResultList;
            while (apiResultList != NULL) {
                if (apiResultList->result.request_api == api_evt) {
                    resultReceived = 1;
                    *apiResult = apiResultList->result;
                    apiResultListPrevious->next = apiResultList->next;
                    if (apiResultList == m_apiResultList) {
                        m_apiResultList = apiResultList->next;
                    }
                    free(apiResultList);
                    break;
                }
                else {
                    apiResultListPrevious = apiResultList;
                    apiResultList = apiResultList->next;
                }
            }
        }
    }
    CDBG("%s: return (%d) from API result wait for evt (%d)",
          __func__, apiResult->status, api_evt);
}


/*===========================================================================
 * FUNCTION   : unlockAPI
 *
 * DESCRIPTION: API processing is done, unlock
 *
 * PARAMETERS : none
 *
 * RETURN     : none
 *==========================================================================*/
void QCamera2HardwareInterface::unlockAPI()
{
    pthread_mutex_unlock(&m_lock);
}

/*===========================================================================
 * FUNCTION   : signalAPIResult
 *
 * DESCRIPTION: signal condition viarable that cerntain API event type arrives
 *
 * PARAMETERS :
 *   @result  : API result
 *
 * RETURN     : none
 *==========================================================================*/
void QCamera2HardwareInterface::signalAPIResult(qcamera_api_result_t *result)
{

    pthread_mutex_lock(&m_lock);
    api_result_list *apiResult = (api_result_list *)malloc(sizeof(api_result_list));
    if (apiResult == NULL) {
        ALOGE("%s: ERROR: malloc for api result failed", __func__);
        ALOGE("%s: ERROR: api thread will wait forever fot this lost result", __func__);
        goto malloc_failed;
    }
    apiResult->result = *result;
    apiResult->next = NULL;
    if (m_apiResultList == NULL) m_apiResultList = apiResult;
    else {
        api_result_list *apiResultList = m_apiResultList;
        while(apiResultList->next != NULL) apiResultList = apiResultList->next;
        apiResultList->next = apiResult;
    }
malloc_failed:
    pthread_cond_broadcast(&m_cond);
    pthread_mutex_unlock(&m_lock);
}

/*===========================================================================
 * FUNCTION   : signalEvtResult
 *
 * DESCRIPTION: signal condition variable that certain event was processed
 *
 * PARAMETERS :
 *   @result  : Event result
 *
 * RETURN     : none
 *==========================================================================*/
void QCamera2HardwareInterface::signalEvtResult(qcamera_api_result_t *result)
{
    pthread_mutex_lock(&m_evtLock);
    m_evtResult = *result;
    pthread_cond_signal(&m_evtCond);
    pthread_mutex_unlock(&m_evtLock);
}

int32_t QCamera2HardwareInterface::prepareRawStream(QCameraChannel *curChannel)
{
    int32_t rc = NO_ERROR;
    cam_dimension_t str_dim,max_dim;
    QCameraChannel *pChannel;

    max_dim.width = 0;
    max_dim.height = 0;

    for (int j = 0; j < QCAMERA_CH_TYPE_MAX; j++) {
        if (m_channels[j] != NULL) {
            pChannel = m_channels[j];
            for (uint8_t i = 0; i < pChannel->getNumOfStreams(); i++) {
                QCameraStream *pStream = pChannel->getStreamByIndex(i);
                if (pStream != NULL) {
                    if (pStream->isTypeOf(CAM_STREAM_TYPE_METADATA)) {
                        continue;
                    }
                    pStream->getFrameDimension(str_dim);
                    if (str_dim.width > max_dim.width) {
                        max_dim.width = str_dim.width;
                    }
                    if (str_dim.height > max_dim.height) {
                        max_dim.height = str_dim.height;
                    }
                }
            }
        }
    }

    for (uint8_t i = 0; i < curChannel->getNumOfStreams(); i++) {
        QCameraStream *pStream = curChannel->getStreamByIndex(i);
        if (pStream != NULL) {
            if (pStream->isTypeOf(CAM_STREAM_TYPE_METADATA)) {
                continue;
            }
            pStream->getFrameDimension(str_dim);
            if (str_dim.width > max_dim.width) {
                max_dim.width = str_dim.width;
            }
            if (str_dim.height > max_dim.height) {
                max_dim.height = str_dim.height;
            }
        }
    }
    pthread_mutex_lock(&m_parm_lock);
    rc = mParameters.updateRAW(max_dim);
    pthread_mutex_unlock(&m_parm_lock);
    return rc;
}
/*===========================================================================
 * FUNCTION   : addStreamToChannel
 *
 * DESCRIPTION: add a stream into a channel
 *
 * PARAMETERS :
 *   @pChannel   : ptr to channel obj
 *   @streamType : type of stream to be added
 *   @streamCB   : callback of stream
 *   @userData   : user data ptr to callback
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::addStreamToChannel(QCameraChannel *pChannel,
                                                      cam_stream_type_t streamType,
                                                      stream_cb_routine streamCB,
                                                      void *userData)
{
    int32_t rc = NO_ERROR;

    if (streamType == CAM_STREAM_TYPE_RAW) {
        prepareRawStream(pChannel);
    }
    QCameraHeapMemory *pStreamInfo = allocateStreamInfoBuf(streamType);
    if (pStreamInfo == NULL) {
        ALOGE("%s: no mem for stream info buf", __func__);
        return NO_MEMORY;
    }
    uint8_t minStreamBufNum = getBufNumRequired(streamType);
    bool bDynAllocBuf = false;
    if (isZSLMode() && streamType == CAM_STREAM_TYPE_SNAPSHOT) {
        bDynAllocBuf = true;
    }

    cam_padding_info_t *padding_info = NULL;

    if (streamType == CAM_STREAM_TYPE_ANALYSIS) {
        padding_info =
                &gCamCapability[mCameraId]->analysis_padding_info;
    } else {
        padding_info =
                &gCamCapability[mCameraId]->padding_info;
    }

    bool deferAllocation = needDeferred(streamType);
    CDBG("%s: deferAllocation = %d bDynAllocBuf = %d, stream type = %d",__func__,
            deferAllocation, bDynAllocBuf, streamType);
    rc = pChannel->addStream(*this,
            pStreamInfo,
            NULL,
            minStreamBufNum,
            padding_info,
            streamCB, userData,
            bDynAllocBuf,
            deferAllocation);

    if (rc != NO_ERROR) {
        ALOGE("%s: add stream type (%d) failed, ret = %d",
              __func__, streamType, rc);
        return rc;
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : addPreviewChannel
 *
 * DESCRIPTION: add a preview channel that contains a preview stream
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::addPreviewChannel()
{
    int32_t rc = NO_ERROR;
    QCameraChannel *pChannel = NULL;
    char value[PROPERTY_VALUE_MAX];
    bool raw_yuv = false;


    if (m_channels[QCAMERA_CH_TYPE_PREVIEW] != NULL) {
        // if we had preview channel before, delete it first
        delete m_channels[QCAMERA_CH_TYPE_PREVIEW];
        m_channels[QCAMERA_CH_TYPE_PREVIEW] = NULL;
    }

    pChannel = new QCameraChannel(mCameraHandle->camera_handle,
                                  mCameraHandle->ops);
    if (NULL == pChannel) {
        ALOGE("%s: no mem for preview channel", __func__);
        return NO_MEMORY;
    }

    // preview only channel, don't need bundle attr and cb
    rc = pChannel->init(NULL, NULL, NULL);
    if (rc != NO_ERROR) {
        ALOGE("%s: init preview channel failed, ret = %d", __func__, rc);
        return rc;
    }

    // meta data stream always coexists with preview if applicable
    rc = addStreamToChannel(pChannel, CAM_STREAM_TYPE_METADATA,
                            metadata_stream_cb_routine, this);
    if (rc != NO_ERROR) {
        ALOGE("%s: add metadata stream failed, ret = %d", __func__, rc);
        return rc;
    }
    /* For Mono Sensor as we are not linking the pproc analysis stream
       will not be supported */
    if (((mParameters.getDcrf() == true)
            || (mParameters.getRecordingHintValue() != true)
            || (mParameters.isFDInVideoEnabled()))
            && (!mParameters.isSecureMode()) &&
            (getSensorType() != CAM_SENSOR_Y)) {
        rc = addStreamToChannel(pChannel, CAM_STREAM_TYPE_ANALYSIS,
                NULL, this);
        if (rc != NO_ERROR) {
            ALOGE("%s: add Analysis stream failed, ret = %d", __func__, rc);
            return rc;
        }
    }

    if (isRdiMode()) {
        CDBG_HIGH("RDI_DEBUG %s[%d]: Add stream to channel", __func__, __LINE__);
        rc = addStreamToChannel(pChannel, CAM_STREAM_TYPE_RAW,
                                rdi_mode_stream_cb_routine, this);
    } else {
        if (isNoDisplayMode()) {
            rc = addStreamToChannel(pChannel, CAM_STREAM_TYPE_PREVIEW,
                                    nodisplay_preview_stream_cb_routine, this);
        } else {
            rc = addStreamToChannel(pChannel, CAM_STREAM_TYPE_PREVIEW,
                                    preview_stream_cb_routine, this);
            pChannel->setStreamSyncCB(CAM_STREAM_TYPE_PREVIEW,
                    synchronous_stream_cb_routine);
        }
    }

    property_get("persist.camera.raw_yuv", value, "0");
    raw_yuv = atoi(value) > 0 ? true : false;
    if ( raw_yuv ) {
        rc = addStreamToChannel(pChannel,CAM_STREAM_TYPE_RAW,
                preview_raw_stream_cb_routine,this);
        if ( rc != NO_ERROR ) {
            ALOGE("%s: add raw stream failed, ret = %d", __FUNCTION__, rc);
            delete pChannel;
            return rc;
        }
    }

    if (rc != NO_ERROR) {
        ALOGE("%s: add preview stream failed, ret = %d", __func__, rc);
        delete pChannel;
        return rc;
    }

    m_channels[QCAMERA_CH_TYPE_PREVIEW] = pChannel;
    return rc;
}

/*===========================================================================
 * FUNCTION   : addVideoChannel
 *
 * DESCRIPTION: add a video channel that contains a video stream
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::addVideoChannel()
{
    int32_t rc = NO_ERROR;
    QCameraVideoChannel *pChannel = NULL;

    if (m_channels[QCAMERA_CH_TYPE_VIDEO] != NULL) {
        // if we had video channel before, delete it first
        delete m_channels[QCAMERA_CH_TYPE_VIDEO];
        m_channels[QCAMERA_CH_TYPE_VIDEO] = NULL;
    }

    pChannel = new QCameraVideoChannel(mCameraHandle->camera_handle,
                                       mCameraHandle->ops);
    if (NULL == pChannel) {
        ALOGE("%s: no mem for video channel", __func__);
        return NO_MEMORY;
    }

    if (isLowPowerMode()) {
        mm_camera_channel_attr_t attr;
        memset(&attr, 0, sizeof(mm_camera_channel_attr_t));
        attr.notify_mode = MM_CAMERA_SUPER_BUF_NOTIFY_BURST;
        attr.look_back = 0; //wait for future frame for liveshot
        attr.post_frame_skip = mParameters.getZSLBurstInterval();
        attr.water_mark = 1; //hold min buffers possible in Q
        attr.max_unmatched_frames = mParameters.getMaxUnmatchedFramesInQueue();
        rc = pChannel->init(&attr, snapshot_channel_cb_routine, this);
    } else {
        // preview only channel, don't need bundle attr and cb
        rc = pChannel->init(NULL, NULL, NULL);
    }

    if (rc != 0) {
        ALOGE("%s: init video channel failed, ret = %d", __func__, rc);
        delete pChannel;
        return rc;
    }

    rc = addStreamToChannel(pChannel, CAM_STREAM_TYPE_VIDEO,
                            video_stream_cb_routine, this);
    if (rc != NO_ERROR) {
        ALOGE("%s: add video stream failed, ret = %d", __func__, rc);
        delete pChannel;
        return rc;
    }

    m_channels[QCAMERA_CH_TYPE_VIDEO] = pChannel;
    return rc;
}

/*===========================================================================
 * FUNCTION   : addSnapshotChannel
 *
 * DESCRIPTION: add a snapshot channel that contains a snapshot stream
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 * NOTE       : Add this channel for live snapshot usecase. Regular capture will
 *              use addCaptureChannel.
 *==========================================================================*/
int32_t QCamera2HardwareInterface::addSnapshotChannel()
{
    int32_t rc = NO_ERROR;
    QCameraChannel *pChannel = NULL;

    if (m_channels[QCAMERA_CH_TYPE_SNAPSHOT] != NULL) {
        // if we had ZSL channel before, delete it first
        delete m_channels[QCAMERA_CH_TYPE_SNAPSHOT];
        m_channels[QCAMERA_CH_TYPE_SNAPSHOT] = NULL;
    }

    pChannel = new QCameraChannel(mCameraHandle->camera_handle,
                                  mCameraHandle->ops);
    if (NULL == pChannel) {
        ALOGE("%s: no mem for snapshot channel", __func__);
        return NO_MEMORY;
    }

    mm_camera_channel_attr_t attr;
    memset(&attr, 0, sizeof(mm_camera_channel_attr_t));
    attr.notify_mode = MM_CAMERA_SUPER_BUF_NOTIFY_CONTINUOUS;
    attr.look_back = mParameters.getZSLBackLookCount();
    attr.post_frame_skip = mParameters.getZSLBurstInterval();
    attr.water_mark = mParameters.getZSLQueueDepth();
    attr.max_unmatched_frames = mParameters.getMaxUnmatchedFramesInQueue();
    attr.priority = MM_CAMERA_SUPER_BUF_PRIORITY_LOW;
    rc = pChannel->init(&attr, snapshot_channel_cb_routine, this);
    if (rc != NO_ERROR) {
        ALOGE("%s: init snapshot channel failed, ret = %d", __func__, rc);
        delete pChannel;
        return rc;
    }

    rc = addStreamToChannel(pChannel, CAM_STREAM_TYPE_SNAPSHOT,
            NULL, NULL);
    if (rc != NO_ERROR) {
        ALOGE("%s: add snapshot stream failed, ret = %d", __func__, rc);
        delete pChannel;
        return rc;
    }

    m_channels[QCAMERA_CH_TYPE_SNAPSHOT] = pChannel;
    return rc;
}

/*===========================================================================
 * FUNCTION   : addRawChannel
 *
 * DESCRIPTION: add a raw channel that contains a raw image stream
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::addRawChannel()
{
    int32_t rc = NO_ERROR;
    QCameraChannel *pChannel = NULL;

    if (m_channels[QCAMERA_CH_TYPE_RAW] != NULL) {
        // if we had raw channel before, delete it first
        delete m_channels[QCAMERA_CH_TYPE_RAW];
        m_channels[QCAMERA_CH_TYPE_RAW] = NULL;
    }

    pChannel = new QCameraChannel(mCameraHandle->camera_handle,
                                  mCameraHandle->ops);
    if (NULL == pChannel) {
        ALOGE("%s: no mem for raw channel", __func__);
        return NO_MEMORY;
    }

    rc = pChannel->init(NULL, NULL, NULL);
    if (rc != NO_ERROR) {
        ALOGE("%s: init raw channel failed, ret = %d", __func__, rc);
        delete pChannel;
        return rc;
    }

    // meta data stream always coexists with snapshot in regular RAW capture case
    rc = addStreamToChannel(pChannel, CAM_STREAM_TYPE_METADATA,
                            metadata_stream_cb_routine, this);
    if (rc != NO_ERROR) {
        ALOGE("%s: add metadata stream failed, ret = %d", __func__, rc);
        delete pChannel;
        return rc;
    }

    rc = addStreamToChannel(pChannel, CAM_STREAM_TYPE_RAW,
                            raw_stream_cb_routine, this);
    if (rc != NO_ERROR) {
        ALOGE("%s: add snapshot stream failed, ret = %d", __func__, rc);
        delete pChannel;
        return rc;
    }
    m_channels[QCAMERA_CH_TYPE_RAW] = pChannel;
    return rc;
}

/*===========================================================================
 * FUNCTION   : addZSLChannel
 *
 * DESCRIPTION: add a ZSL channel that contains a preview stream and
 *              a snapshot stream
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::addZSLChannel()
{
    int32_t rc = NO_ERROR;
    QCameraPicChannel *pChannel = NULL;
    char value[PROPERTY_VALUE_MAX];
    bool raw_yuv = false;

    if (m_channels[QCAMERA_CH_TYPE_ZSL] != NULL) {
        // if we had ZSL channel before, delete it first
        delete m_channels[QCAMERA_CH_TYPE_ZSL];
        m_channels[QCAMERA_CH_TYPE_ZSL] = NULL;
    }

    pChannel = new QCameraPicChannel(mCameraHandle->camera_handle,
                                     mCameraHandle->ops);
    if (NULL == pChannel) {
        ALOGE("%s: no mem for ZSL channel", __func__);
        return NO_MEMORY;
    }

    // ZSL channel, init with bundle attr and cb
    mm_camera_channel_attr_t attr;
    memset(&attr, 0, sizeof(mm_camera_channel_attr_t));
    if (mParameters.isSceneSelectionEnabled()) {
        attr.notify_mode = MM_CAMERA_SUPER_BUF_NOTIFY_CONTINUOUS;
    } else {
        attr.notify_mode = MM_CAMERA_SUPER_BUF_NOTIFY_BURST;
    }
    attr.look_back = mParameters.getZSLBackLookCount();
    attr.post_frame_skip = mParameters.getZSLBurstInterval();
    attr.water_mark = mParameters.getZSLQueueDepth();
    attr.max_unmatched_frames = mParameters.getMaxUnmatchedFramesInQueue();
    attr.instant_capture_enabled = (uint8_t)mParameters.isInstantCaptureEnabled();
    attr.aec_frame_bound = (uint8_t)mParameters.getAecFrameBoundValue();

    //Enabled matched queue
    if (getRelatedCamSyncInfo()->is_frame_sync_enabled) {
        CDBG_HIGH("%s: Enabling frame sync for dual camera, camera Id: %d",
                __func__, mCameraId);
        attr.enable_frame_sync = 1;
    }
    rc = pChannel->init(&attr,
                        zsl_channel_cb,
                        this);
    if (rc != 0) {
        ALOGE("%s: init ZSL channel failed, ret = %d", __func__, rc);
        delete pChannel;
        return rc;
    }

    // meta data stream always coexists with preview if applicable
    rc = addStreamToChannel(pChannel, CAM_STREAM_TYPE_METADATA,
                            metadata_stream_cb_routine, this);
    if (rc != NO_ERROR) {
        ALOGE("%s: add metadata stream failed, ret = %d", __func__, rc);
        delete pChannel;
        return rc;
    }

    if (isNoDisplayMode()) {
        rc = addStreamToChannel(pChannel, CAM_STREAM_TYPE_PREVIEW,
                                nodisplay_preview_stream_cb_routine, this);
    } else {
        rc = addStreamToChannel(pChannel, CAM_STREAM_TYPE_PREVIEW,
                                preview_stream_cb_routine, this);
        pChannel->setStreamSyncCB(CAM_STREAM_TYPE_PREVIEW,
                synchronous_stream_cb_routine);
    }
    if (rc != NO_ERROR) {
        ALOGE("%s: add preview stream failed, ret = %d", __func__, rc);
        delete pChannel;
        return rc;
    }

    rc = addStreamToChannel(pChannel, CAM_STREAM_TYPE_SNAPSHOT,
                            NULL, this);
    if (rc != NO_ERROR) {
        ALOGE("%s: add snapshot stream failed, ret = %d", __func__, rc);
        delete pChannel;
        return rc;
    }

    if (!mParameters.isSecureMode() &&
        (getSensorType() != CAM_SENSOR_Y)) {
        rc = addStreamToChannel(pChannel, CAM_STREAM_TYPE_ANALYSIS,
                NULL, this);
        if (rc != NO_ERROR) {
            ALOGE("%s: add Analysis stream failed, ret = %d", __func__, rc);
            delete pChannel;
            return rc;
        }
    }

    property_get("persist.camera.raw_yuv", value, "0");
    raw_yuv = atoi(value) > 0 ? true : false;
    if ( raw_yuv ) {
        rc = addStreamToChannel(pChannel,
                                CAM_STREAM_TYPE_RAW,
                                NULL,
                                this);
        if (rc != NO_ERROR) {
            ALOGE("%s: add raw stream failed, ret = %d", __func__, rc);
            delete pChannel;
            return rc;
        }
    }

    m_channels[QCAMERA_CH_TYPE_ZSL] = pChannel;
    return rc;
}

/*===========================================================================
 * FUNCTION   : addCaptureChannel
 *
 * DESCRIPTION: add a capture channel that contains a snapshot stream
 *              and a postview stream
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 * NOTE       : Add this channel for regular capture usecase.
 *              For Live snapshot usecase, use addSnapshotChannel.
 *==========================================================================*/
int32_t QCamera2HardwareInterface::addCaptureChannel()
{
    int32_t rc = NO_ERROR;
    QCameraPicChannel *pChannel = NULL;
    char value[PROPERTY_VALUE_MAX];
    bool raw_yuv = false;

    if (m_channels[QCAMERA_CH_TYPE_CAPTURE] != NULL) {
        delete m_channels[QCAMERA_CH_TYPE_CAPTURE];
        m_channels[QCAMERA_CH_TYPE_CAPTURE] = NULL;
    }

    pChannel = new QCameraPicChannel(mCameraHandle->camera_handle,
                                  mCameraHandle->ops);
    if (NULL == pChannel) {
        ALOGE("%s: no mem for capture channel", __func__);
        return NO_MEMORY;
    }

    // Capture channel, only need snapshot and postview streams start together
    mm_camera_channel_attr_t attr;
    memset(&attr, 0, sizeof(mm_camera_channel_attr_t));
    if ( mLongshotEnabled ) {
        attr.notify_mode = MM_CAMERA_SUPER_BUF_NOTIFY_BURST;
        attr.look_back = mParameters.getZSLBackLookCount();
        attr.water_mark = mParameters.getZSLQueueDepth();
    } else {
        attr.notify_mode = MM_CAMERA_SUPER_BUF_NOTIFY_CONTINUOUS;
    }
    attr.max_unmatched_frames = mParameters.getMaxUnmatchedFramesInQueue();

    rc = pChannel->init(&attr,
                        capture_channel_cb_routine,
                        this);
    if (rc != NO_ERROR) {
        ALOGE("%s: init capture channel failed, ret = %d", __func__, rc);
        return rc;
    }

    // meta data stream always coexists with snapshot in regular capture case
    rc = addStreamToChannel(pChannel, CAM_STREAM_TYPE_METADATA,
                            metadata_stream_cb_routine, this);
    if (rc != NO_ERROR) {
        ALOGE("%s: add metadata stream failed, ret = %d", __func__, rc);
        return rc;
    }

    if (!mLongshotEnabled) {
        rc = addStreamToChannel(pChannel, CAM_STREAM_TYPE_POSTVIEW,
                                NULL, this);

        if (rc != NO_ERROR) {
            ALOGE("%s: add postview stream failed, ret = %d", __func__, rc);
            return rc;
        }
    } else {
        rc = addStreamToChannel(pChannel, CAM_STREAM_TYPE_PREVIEW,
                                preview_stream_cb_routine, this);

        if (rc != NO_ERROR) {
            ALOGE("%s: add preview stream failed, ret = %d", __func__, rc);
            return rc;
        }
        pChannel->setStreamSyncCB(CAM_STREAM_TYPE_PREVIEW,
                synchronous_stream_cb_routine);
    }

    if (!mParameters.getofflineRAW()) {
        rc = addStreamToChannel(pChannel, CAM_STREAM_TYPE_SNAPSHOT,
                NULL, this);
        if (rc != NO_ERROR) {
            ALOGE("%s: add snapshot stream failed, ret = %d", __func__, rc);
            return rc;
        }
    }
    property_get("persist.camera.raw_yuv", value, "0");
    raw_yuv = atoi(value) > 0 ? true : false;
    if ( raw_yuv ) {
        if (!mParameters.getofflineRAW()) {
            rc = addStreamToChannel(pChannel,
                    CAM_STREAM_TYPE_RAW,
                    snapshot_raw_stream_cb_routine,
                    this);
        } else {
            rc = addStreamToChannel(pChannel,
                    CAM_STREAM_TYPE_RAW,
                    NULL,
                    this);
        }
        if (rc != NO_ERROR) {
            ALOGE("%s: add raw stream failed, ret = %d", __func__, rc);
            return rc;
        }
    }

    m_channels[QCAMERA_CH_TYPE_CAPTURE] = pChannel;
    return rc;
}

/*===========================================================================
 * FUNCTION   : addMetaDataChannel
 *
 * DESCRIPTION: add a meta data channel that contains a metadata stream
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::addMetaDataChannel()
{
    int32_t rc = NO_ERROR;
    QCameraChannel *pChannel = NULL;

    if (m_channels[QCAMERA_CH_TYPE_METADATA] != NULL) {
        delete m_channels[QCAMERA_CH_TYPE_METADATA];
        m_channels[QCAMERA_CH_TYPE_METADATA] = NULL;
    }

    pChannel = new QCameraChannel(mCameraHandle->camera_handle,
                                  mCameraHandle->ops);
    if (NULL == pChannel) {
        ALOGE("%s: no mem for metadata channel", __func__);
        return NO_MEMORY;
    }

    rc = pChannel->init(NULL,
                        NULL,
                        NULL);
    if (rc != NO_ERROR) {
        ALOGE("%s: init metadata channel failed, ret = %d", __func__, rc);
        delete pChannel;
        return rc;
    }

    rc = addStreamToChannel(pChannel, CAM_STREAM_TYPE_METADATA,
                            metadata_stream_cb_routine, this);
    if (rc != NO_ERROR) {
        ALOGE("%s: add metadata stream failed, ret = %d", __func__, rc);
        delete pChannel;
        return rc;
    }

    m_channels[QCAMERA_CH_TYPE_METADATA] = pChannel;
    return rc;
}

/*===========================================================================
 * FUNCTION   : addCallbackChannel
 *
 * DESCRIPTION: add a callback channel that contains a callback stream
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::addCallbackChannel()
{
    int32_t rc = NO_ERROR;
    QCameraChannel *pChannel = NULL;

    if (m_channels[QCAMERA_CH_TYPE_CALLBACK] != NULL) {
        delete m_channels[QCAMERA_CH_TYPE_CALLBACK];
        m_channels[QCAMERA_CH_TYPE_CALLBACK] = NULL;
    }

    pChannel = new QCameraChannel(mCameraHandle->camera_handle,
            mCameraHandle->ops);
    if (NULL == pChannel) {
        ALOGE("%s: no mem for callback channel", __func__);
        return NO_MEMORY;
    }

    rc = pChannel->init(NULL, NULL, this);
    if (rc != NO_ERROR) {
        ALOGE("%s: init callback channel failed, ret = %d",
                __func__, rc);
        delete pChannel;
        return rc;
    }

    rc = addStreamToChannel(pChannel, CAM_STREAM_TYPE_CALLBACK,
            callback_stream_cb_routine, this);
    if (rc != NO_ERROR) {
        ALOGE("%s: add callback stream failed, ret = %d", __func__, rc);
        delete pChannel;
        return rc;
    }

    m_channels[QCAMERA_CH_TYPE_CALLBACK] = pChannel;
    return rc;
}


/*===========================================================================
 * FUNCTION   : addAnalysisChannel
 *
 * DESCRIPTION: add a analysis channel that contains a analysis stream
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::addAnalysisChannel()
{
    int32_t rc = NO_ERROR;
    QCameraChannel *pChannel = NULL;

    if (m_channels[QCAMERA_CH_TYPE_ANALYSIS] != NULL) {
        delete m_channels[QCAMERA_CH_TYPE_ANALYSIS];
        m_channels[QCAMERA_CH_TYPE_ANALYSIS] = NULL;
    }

    pChannel = new QCameraChannel(mCameraHandle->camera_handle,
                                  mCameraHandle->ops);
    if (NULL == pChannel) {
        ALOGE("%s: no mem for metadata channel", __func__);
        return NO_MEMORY;
    }

    rc = pChannel->init(NULL, NULL, this);
    if (rc != NO_ERROR) {
        ALOGE("%s: init Analysis channel failed, ret = %d", __func__, rc);
        delete pChannel;
        return rc;
    }

    rc = addStreamToChannel(pChannel, CAM_STREAM_TYPE_ANALYSIS,
                            NULL, this);
    if (rc != NO_ERROR) {
        ALOGE("%s: add Analysis stream failed, ret = %d", __func__, rc);
        delete pChannel;
        return rc;
    }

    m_channels[QCAMERA_CH_TYPE_ANALYSIS] = pChannel;
    return rc;
}


/*===========================================================================
 * FUNCTION   : getPPConfig
 *
 * DESCRIPTION: get Post processing configaration data
 *
 * PARAMETERS :
 * @pp config:  pp config structure pointer,
 * @curCount:  current pp pass count
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::getPPConfig(cam_pp_feature_config_t &pp_config, int curCount)
{
    int32_t rc = NO_ERROR;

    if ( curCount != mParameters.getReprocCount() ) {
        ALOGW("%s : Multi pass enabled. Total Pass = %d, cur Pass = %d", __func__,
                mParameters.getReprocCount(), curCount);
    }

    CDBG_HIGH("%s: Supported pproc feature mask = %x", __func__,
            gCamCapability[mCameraId]->qcom_supported_feature_mask);
    uint32_t feature_mask = gCamCapability[mCameraId]->qcom_supported_feature_mask;
    int32_t zoomLevel = mParameters.getParmZoomLevel();
    uint32_t rotation = mParameters.getJpegRotation();
    int32_t effect = mParameters.getEffectValue();

    switch(curCount) {
        case 1:
            //Configure feature mask for first pass of reprocessing
            //check if any effects are enabled
            if ((CAM_EFFECT_MODE_OFF != effect) &&
                (feature_mask & CAM_QCOM_FEATURE_EFFECT)) {
                pp_config.feature_mask |= CAM_QCOM_FEATURE_EFFECT;
                pp_config.effect = effect;
            }

            //check for features that need to be enabled by default like sharpness
            //(if supported by hw).
            if ((feature_mask & CAM_QCOM_FEATURE_SHARPNESS) &&
                !mParameters.isOptiZoomEnabled()) {
                pp_config.feature_mask |= CAM_QCOM_FEATURE_SHARPNESS;
                pp_config.sharpness = mParameters.getInt(QCameraParameters::KEY_QC_SHARPNESS);
            }

            //check if zoom is enabled
            if ((zoomLevel > 0) && (feature_mask & CAM_QCOM_FEATURE_CROP)) {
                pp_config.feature_mask |= CAM_QCOM_FEATURE_CROP;
            }

            if (mParameters.isWNREnabled() &&
                (feature_mask & CAM_QCOM_FEATURE_DENOISE2D)) {
                pp_config.feature_mask |= CAM_QCOM_FEATURE_DENOISE2D;
                pp_config.denoise2d.denoise_enable = 1;
                pp_config.denoise2d.process_plates =
                        mParameters.getDenoiseProcessPlate(CAM_INTF_PARM_WAVELET_DENOISE);
            }

            if (isCACEnabled()) {
                pp_config.feature_mask |= CAM_QCOM_FEATURE_CAC;
            }

            //check if rotation is required
            if ((feature_mask & CAM_QCOM_FEATURE_ROTATION) && (rotation > 0)) {
                pp_config.feature_mask |= CAM_QCOM_FEATURE_ROTATION;
                if (rotation == 0) {
                    pp_config.rotation = ROTATE_0;
                } else if (rotation == 90) {
                    pp_config.rotation = ROTATE_90;
                } else if (rotation == 180) {
                    pp_config.rotation = ROTATE_180;
                } else if (rotation == 270) {
                    pp_config.rotation = ROTATE_270;
                }
            }

            if (mParameters.isHDREnabled()){
                pp_config.feature_mask |= CAM_QCOM_FEATURE_HDR;
                pp_config.hdr_param.hdr_enable = 1;
                pp_config.hdr_param.hdr_need_1x = mParameters.isHDR1xFrameEnabled();
                pp_config.hdr_param.hdr_mode = CAM_HDR_MODE_MULTIFRAME;
            } else {
                pp_config.feature_mask &= ~CAM_QCOM_FEATURE_HDR;
                pp_config.hdr_param.hdr_enable = 0;
            }

            //check if scaling is enabled
            if ((feature_mask & CAM_QCOM_FEATURE_SCALE) &&
                mParameters.m_reprocScaleParam.isScaleEnabled() &&
                mParameters.m_reprocScaleParam.isUnderScaling()){
                pp_config.feature_mask |= CAM_QCOM_FEATURE_SCALE;
                mParameters.m_reprocScaleParam.getPicSizeFromAPK(
                        pp_config.scale_param.output_width,
                        pp_config.scale_param.output_height);
            }

            if(mParameters.isUbiFocusEnabled()) {
                pp_config.feature_mask |= CAM_QCOM_FEATURE_UBIFOCUS;
            } else {
                pp_config.feature_mask &= ~CAM_QCOM_FEATURE_UBIFOCUS;
            }

            if(mParameters.isUbiRefocus()) {
                pp_config.feature_mask |= CAM_QCOM_FEATURE_REFOCUS;
                pp_config.misc_buf_param.misc_buffer_index = 0;
            } else {
                pp_config.feature_mask &= ~CAM_QCOM_FEATURE_REFOCUS;
            }

            if(mParameters.isChromaFlashEnabled()) {
                pp_config.feature_mask |= CAM_QCOM_FEATURE_CHROMA_FLASH;
                pp_config.flash_value = CAM_FLASH_ON;
            } else {
                pp_config.feature_mask &= ~CAM_QCOM_FEATURE_CHROMA_FLASH;
            }

            if(mParameters.isOptiZoomEnabled() && (0 <= zoomLevel)) {
                pp_config.feature_mask |= CAM_QCOM_FEATURE_OPTIZOOM;
                pp_config.zoom_level = (uint8_t) zoomLevel;
            } else {
                pp_config.feature_mask &= ~CAM_QCOM_FEATURE_OPTIZOOM;
            }

            if (mParameters.getofflineRAW()) {
                pp_config.feature_mask |= CAM_QCOM_FEATURE_RAW_PROCESSING;
            }

            if (mParameters.isTruePortraitEnabled()) {
                pp_config.feature_mask |= CAM_QCOM_FEATURE_TRUEPORTRAIT;
                pp_config.misc_buf_param.misc_buffer_index = 0;
            } else {
                pp_config.feature_mask &= ~CAM_QCOM_FEATURE_TRUEPORTRAIT;
            }

            if(mParameters.isStillMoreEnabled()) {
                pp_config.feature_mask |= CAM_QCOM_FEATURE_STILLMORE;
            } else {
                pp_config.feature_mask &= ~CAM_QCOM_FEATURE_STILLMORE;
            }

            if (mParameters.getCDSMode() != CAM_CDS_MODE_OFF) {
                if (feature_mask & CAM_QCOM_FEATURE_DSDN) {
                    pp_config.feature_mask |= CAM_QCOM_FEATURE_DSDN;
                } else {
                    pp_config.feature_mask |= CAM_QCOM_FEATURE_CDS;
                }
            }

            if (curCount != mParameters.getReprocCount()) {
                pp_config.feature_mask &= ~CAM_QCOM_FEATURE_PP_PASS_2;
                pp_config.feature_mask &= ~CAM_QCOM_FEATURE_ROTATION;
                pp_config.feature_mask &= ~CAM_QCOM_FEATURE_CDS;
                pp_config.feature_mask &= ~CAM_QCOM_FEATURE_DSDN;
                pp_config.feature_mask |= CAM_QCOM_FEATURE_CROP;
            } else {
                pp_config.feature_mask |= CAM_QCOM_FEATURE_SCALE;
            }

            cam_dimension_t thumb_src_dim;
            cam_dimension_t thumb_dst_dim;
            mParameters.getThumbnailSize(&(thumb_dst_dim.width), &(thumb_dst_dim.height));
            mParameters.getStreamDimension(CAM_STREAM_TYPE_POSTVIEW,thumb_src_dim);
            if ((thumb_dst_dim.width != thumb_src_dim.width) ||
                    (thumb_dst_dim.height != thumb_src_dim.height)) {
                if (thumb_dst_dim.width != 0 && thumb_dst_dim.height != 0) {
                    pp_config.feature_mask |= CAM_QCOM_FEATURE_CROP;
                }
            }

            break;

        case 2:
            //Configure feature mask for second pass of reprocessing
            pp_config.feature_mask |= CAM_QCOM_FEATURE_PP_PASS_2;
            if ((feature_mask & CAM_QCOM_FEATURE_ROTATION) && (rotation > 0)) {
                pp_config.feature_mask |= CAM_QCOM_FEATURE_ROTATION;
                if (rotation == 0) {
                    pp_config.rotation = ROTATE_0;
                } else if (rotation == 90) {
                    pp_config.rotation = ROTATE_90;
                } else if (rotation == 180) {
                    pp_config.rotation = ROTATE_180;
                } else if (rotation == 270) {
                    pp_config.rotation = ROTATE_270;
                }
            }
            if (mParameters.getCDSMode() != CAM_CDS_MODE_OFF) {
                if (feature_mask & CAM_QCOM_FEATURE_DSDN) {
                    pp_config.feature_mask |= CAM_QCOM_FEATURE_DSDN;
                } else {
                    pp_config.feature_mask |= CAM_QCOM_FEATURE_CDS;
                }
            }
            break;

    }
    CDBG_HIGH("%s: pproc feature mask set = %x pass count = %d",
            __func__, pp_config.feature_mask,curCount);
    return rc;
}

/*===========================================================================
 * FUNCTION   : addReprocChannel
 *
 * DESCRIPTION: add a reprocess channel that will do reprocess on frames
 *              coming from input channel
 *
 * PARAMETERS :
 *   @pInputChannel : ptr to input channel whose frames will be post-processed
 *
 * RETURN     : Ptr to the newly created channel obj. NULL if failed.
 *==========================================================================*/
QCameraReprocessChannel *QCamera2HardwareInterface::addReprocChannel(
                                                      QCameraChannel *pInputChannel)
{
    int32_t rc = NO_ERROR;
    QCameraReprocessChannel *pChannel = NULL;

    if (pInputChannel == NULL) {
        ALOGE("%s: input channel obj is NULL", __func__);
        return NULL;
    }

    pChannel = new QCameraReprocessChannel(mCameraHandle->camera_handle,
                                           mCameraHandle->ops);
    if (NULL == pChannel) {
        ALOGE("%s: no mem for reprocess channel", __func__);
        return NULL;
    }

    // Capture channel, only need snapshot and postview streams start together
    mm_camera_channel_attr_t attr;
    memset(&attr, 0, sizeof(mm_camera_channel_attr_t));
    attr.notify_mode = MM_CAMERA_SUPER_BUF_NOTIFY_CONTINUOUS;
    attr.max_unmatched_frames = mParameters.getMaxUnmatchedFramesInQueue();
    rc = pChannel->init(&attr,
                        postproc_channel_cb_routine,
                        this);
    if (rc != NO_ERROR) {
        ALOGE("%s: init reprocess channel failed, ret = %d", __func__, rc);
        delete pChannel;
        return NULL;
    }

    // pp feature config
    cam_pp_feature_config_t pp_config;
    memset(&pp_config, 0, sizeof(cam_pp_feature_config_t));

    rc = getPPConfig(pp_config, mParameters.getCurPPCount());
    if (rc != NO_ERROR){
        ALOGE("%s: Error while creating PP config",__func__);
        delete pChannel;
        return NULL;
    }

    uint8_t minStreamBufNum = getBufNumRequired(CAM_STREAM_TYPE_OFFLINE_PROC);

    //WNR and HDR happen inline. No extra buffers needed.
    uint32_t temp_feature_mask = pp_config.feature_mask;
    temp_feature_mask &= ~CAM_QCOM_FEATURE_HDR;
    if (temp_feature_mask && mParameters.isHDREnabled()) {
        minStreamBufNum = (uint8_t)(1 + mParameters.getNumOfExtraHDRInBufsIfNeeded());
    }

    if (mParameters.isStillMoreEnabled()) {
        cam_still_more_t stillmore_config = mParameters.getStillMoreSettings();
        pp_config.burst_cnt = stillmore_config.burst_count;
        CDBG_HIGH("%s: Stillmore burst %d", __func__, pp_config.burst_cnt);

        // getNumOfExtraBuffersForImageProc returns 1 less buffer assuming
        // number of capture is already added. In the case of liveshot,
        // stillmore burst is 1. This is to account for the premature decrement
        if (mParameters.getNumOfExtraBuffersForImageProc() == 0) {
            minStreamBufNum += 1;
        }
    }

    // Add non inplace image lib buffers only when ppproc is present,
    // becuase pproc is non inplace and input buffers for img lib
    // are output for pproc and this number of extra buffers is required
    // If pproc is not there, input buffers for imglib are from snapshot stream
    uint8_t imglib_extra_bufs = mParameters.getNumOfExtraBuffersForImageProc();
    if (temp_feature_mask && imglib_extra_bufs) {
        // 1 is added because getNumOfExtraBuffersForImageProc returns extra
        // buffers assuming number of capture is already added
        minStreamBufNum = (uint8_t)(minStreamBufNum + imglib_extra_bufs + 1);
    }

    //Mask out features that are already processed in snapshot stream.
    uint32_t snapshot_feature_mask = 0;
    mParameters.getStreamPpMask(CAM_STREAM_TYPE_SNAPSHOT, snapshot_feature_mask);

    pp_config.feature_mask &= ~snapshot_feature_mask;
    ALOGI("%s: Snapshot feature mask: 0x%x, reproc feature mask: 0x%x", __func__,
            snapshot_feature_mask, pp_config.feature_mask);

    bool offlineReproc = isRegularCapture();
    cam_padding_info_t* paddingInfo = &gCamCapability[mCameraId]->padding_info;
    rc = pChannel->addReprocStreamsFromSource(*this,
                                              pp_config,
                                              pInputChannel,
                                              minStreamBufNum,
                                              mParameters.getNumOfSnapshots(),
                                              paddingInfo,
                                              mParameters,
                                              mLongshotEnabled,
                                              offlineReproc);
    if (rc != NO_ERROR) {
        delete pChannel;
        return NULL;
    }

    return pChannel;
}

/*===========================================================================
 * FUNCTION   : addOfflineReprocChannel
 *
 * DESCRIPTION: add a offline reprocess channel contains one reproc stream,
 *              that will do reprocess on frames coming from external images
 *
 * PARAMETERS :
 *   @img_config  : offline reporcess image info
 *   @pp_feature  : pp feature config
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
QCameraReprocessChannel *QCamera2HardwareInterface::addOfflineReprocChannel(
                                            cam_pp_offline_src_config_t &img_config,
                                            cam_pp_feature_config_t &pp_feature,
                                            stream_cb_routine stream_cb,
                                            void *userdata)
{
    int32_t rc = NO_ERROR;
    QCameraReprocessChannel *pChannel = NULL;

    pChannel = new QCameraReprocessChannel(mCameraHandle->camera_handle,
                                           mCameraHandle->ops);
    if (NULL == pChannel) {
        ALOGE("%s: no mem for reprocess channel", __func__);
        return NULL;
    }

    rc = pChannel->init(NULL, NULL, NULL);
    if (rc != NO_ERROR) {
        ALOGE("%s: init reprocess channel failed, ret = %d", __func__, rc);
        delete pChannel;
        return NULL;
    }

    QCameraHeapMemory *pStreamInfo = allocateStreamInfoBuf(CAM_STREAM_TYPE_OFFLINE_PROC);
    if (pStreamInfo == NULL) {
        ALOGE("%s: no mem for stream info buf", __func__);
        delete pChannel;
        return NULL;
    }

    cam_stream_info_t *streamInfoBuf = (cam_stream_info_t *)pStreamInfo->getPtr(0);
    memset(streamInfoBuf, 0, sizeof(cam_stream_info_t));
    streamInfoBuf->stream_type = CAM_STREAM_TYPE_OFFLINE_PROC;
    streamInfoBuf->fmt = img_config.input_fmt;
    streamInfoBuf->dim = img_config.input_dim;
    streamInfoBuf->buf_planes = img_config.input_buf_planes;
    streamInfoBuf->streaming_mode = CAM_STREAMING_MODE_BURST;
    streamInfoBuf->num_of_burst = img_config.num_of_bufs;

    streamInfoBuf->reprocess_config.pp_type = CAM_OFFLINE_REPROCESS_TYPE;
    streamInfoBuf->reprocess_config.offline = img_config;
    streamInfoBuf->reprocess_config.pp_feature_config = pp_feature;

    rc = pChannel->addStream(*this,
            pStreamInfo, NULL, img_config.num_of_bufs,
            &gCamCapability[mCameraId]->padding_info,
            stream_cb, userdata, false);

    if (rc != NO_ERROR) {
        ALOGE("%s: add reprocess stream failed, ret = %d", __func__, rc);
        delete pChannel;
        return NULL;
    }

    return pChannel;
}

/*===========================================================================
 * FUNCTION   : addChannel
 *
 * DESCRIPTION: add a channel by its type
 *
 * PARAMETERS :
 *   @ch_type : channel type
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::addChannel(qcamera_ch_type_enum_t ch_type)
{
    int32_t rc = UNKNOWN_ERROR;
    switch (ch_type) {
    case QCAMERA_CH_TYPE_ZSL:
        rc = addZSLChannel();
        break;
    case QCAMERA_CH_TYPE_CAPTURE:
        rc = addCaptureChannel();
        break;
    case QCAMERA_CH_TYPE_PREVIEW:
        rc = addPreviewChannel();
        break;
    case QCAMERA_CH_TYPE_VIDEO:
        rc = addVideoChannel();
        break;
    case QCAMERA_CH_TYPE_SNAPSHOT:
        rc = addSnapshotChannel();
        break;
    case QCAMERA_CH_TYPE_RAW:
        rc = addRawChannel();
        break;
    case QCAMERA_CH_TYPE_METADATA:
        rc = addMetaDataChannel();
        break;
    case QCAMERA_CH_TYPE_CALLBACK:
        rc = addCallbackChannel();
        break;
    case QCAMERA_CH_TYPE_ANALYSIS:
        rc = addAnalysisChannel();
        break;
    default:
        break;
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : delChannel
 *
 * DESCRIPTION: delete a channel by its type
 *
 * PARAMETERS :
 *   @ch_type : channel type
 *   @destroy : delete context as well
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::delChannel(qcamera_ch_type_enum_t ch_type,
                                              bool destroy)
{
    if (m_channels[ch_type] != NULL) {
        if (destroy) {
            delete m_channels[ch_type];
            m_channels[ch_type] = NULL;
        } else {
            m_channels[ch_type]->deleteChannel();
        }
    }

    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : startChannel
 *
 * DESCRIPTION: start a channel by its type
 *
 * PARAMETERS :
 *   @ch_type : channel type
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::startChannel(qcamera_ch_type_enum_t ch_type)
{
    int32_t rc = UNKNOWN_ERROR;
    if (m_channels[ch_type] != NULL) {
        rc = m_channels[ch_type]->start();
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : stopChannel
 *
 * DESCRIPTION: stop a channel by its type
 *
 * PARAMETERS :
 *   @ch_type : channel type
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::stopChannel(qcamera_ch_type_enum_t ch_type)
{
    int32_t rc = UNKNOWN_ERROR;
    if (m_channels[ch_type] != NULL) {
        rc = m_channels[ch_type]->stop();
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : preparePreview
 *
 * DESCRIPTION: add channels needed for preview
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::preparePreview()
{
    ATRACE_CALL();
    int32_t rc = NO_ERROR;

    pthread_mutex_lock(&m_parm_lock);
    rc = mParameters.setStreamConfigure(false, false, false);
    if (rc != NO_ERROR) {
        ALOGE("%s: setStreamConfigure failed %d", __func__, rc);
        pthread_mutex_unlock(&m_parm_lock);
        return rc;
    }
    pthread_mutex_unlock(&m_parm_lock);

    if (mParameters.isZSLMode() && mParameters.getRecordingHintValue() != true) {
        rc = addChannel(QCAMERA_CH_TYPE_ZSL);
        if (rc != NO_ERROR) {
            ALOGE("%s[%d]: failed!! rc = %d", __func__, __LINE__, rc);
            return rc;
        }

        if (mParameters.isUBWCEnabled()) {
            cam_format_t fmt;
            mParameters.getStreamFormat(CAM_STREAM_TYPE_PREVIEW, fmt);
            if (fmt == CAM_FORMAT_YUV_420_NV12_UBWC) {
                rc = addChannel(QCAMERA_CH_TYPE_CALLBACK);
                if (rc != NO_ERROR) {
                    delChannel(QCAMERA_CH_TYPE_ZSL);
                    ALOGE("%s[%d]:failed!! rc = %d", __func__, __LINE__, rc);
                    return rc;
                }
            }
        }
    } else {
        bool recordingHint = mParameters.getRecordingHintValue();
        if(!isRdiMode() && recordingHint) {
            //stop face detection,longshot,etc if turned ON in Camera mode
            int32_t arg; //dummy arg
#ifndef VANILLA_HAL
            if (isLongshotEnabled()) {
                sendCommand(CAMERA_CMD_LONGSHOT_OFF, arg, arg);
            }
            if (mParameters.isFaceDetectionEnabled()
                    && (!mParameters.isFDInVideoEnabled())) {
                sendCommand(CAMERA_CMD_STOP_FACE_DETECTION, arg, arg);
            }
            if (mParameters.isHistogramEnabled()) {
                sendCommand(CAMERA_CMD_HISTOGRAM_OFF, arg, arg);
            }
#endif
            //Don't create snapshot channel for liveshot, if low power mode is set.
            //Use video stream instead.
            if (!isLowPowerMode()) {
               rc = addChannel(QCAMERA_CH_TYPE_SNAPSHOT);
               if (rc != NO_ERROR) {
                   return rc;
               }
            }

            rc = addChannel(QCAMERA_CH_TYPE_VIDEO);
            if (rc != NO_ERROR) {
                delChannel(QCAMERA_CH_TYPE_SNAPSHOT);
                ALOGE("%s[%d]:failed!! rc = %d", __func__, __LINE__, rc);
                return rc;
            }
        }

        rc = addChannel(QCAMERA_CH_TYPE_PREVIEW);
        if (!isRdiMode() && (rc != NO_ERROR)) {
            if (recordingHint) {
                delChannel(QCAMERA_CH_TYPE_SNAPSHOT);
                delChannel(QCAMERA_CH_TYPE_VIDEO);
            }
        }

        if (mParameters.isUBWCEnabled() && !recordingHint) {
            cam_format_t fmt;
            mParameters.getStreamFormat(CAM_STREAM_TYPE_PREVIEW, fmt);
            if (fmt == CAM_FORMAT_YUV_420_NV12_UBWC) {
                rc = addChannel(QCAMERA_CH_TYPE_CALLBACK);
                if (rc != NO_ERROR) {
                    delChannel(QCAMERA_CH_TYPE_PREVIEW);
                    if (!isRdiMode()) {
                        delChannel(QCAMERA_CH_TYPE_SNAPSHOT);
                        delChannel(QCAMERA_CH_TYPE_VIDEO);
                    }
                    ALOGE("%s[%d]:failed!! rc = %d", __func__, __LINE__, rc);
                    return rc;
                }
            }
        }

        if (NO_ERROR != rc) {
            delChannel(QCAMERA_CH_TYPE_PREVIEW);
            ALOGE("%s[%d]:failed!! rc = %d", __func__, __LINE__, rc);
        }
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : unpreparePreview
 *
 * DESCRIPTION: delete channels for preview
 *
 * PARAMETERS : none
 *
 * RETURN     : none
 *==========================================================================*/
void QCamera2HardwareInterface::unpreparePreview()
{
    delChannel(QCAMERA_CH_TYPE_ZSL);
    delChannel(QCAMERA_CH_TYPE_PREVIEW);
    delChannel(QCAMERA_CH_TYPE_VIDEO);
    delChannel(QCAMERA_CH_TYPE_SNAPSHOT);
    delChannel(QCAMERA_CH_TYPE_CALLBACK);
}

/*===========================================================================
 * FUNCTION   : playShutter
 *
 * DESCRIPTION: send request to play shutter sound
 *
 * PARAMETERS : none
 *
 * RETURN     : none
 *==========================================================================*/
void QCamera2HardwareInterface::playShutter(){
     if (mNotifyCb == NULL ||
         msgTypeEnabledWithLock(CAMERA_MSG_SHUTTER) == 0){
         CDBG("%s: shutter msg not enabled or NULL cb", __func__);
         return;
     }
     CDBG_HIGH("%s: CAMERA_MSG_SHUTTER ", __func__);
     qcamera_callback_argm_t cbArg;
     memset(&cbArg, 0, sizeof(qcamera_callback_argm_t));
     cbArg.cb_type = QCAMERA_NOTIFY_CALLBACK;
     cbArg.msg_type = CAMERA_MSG_SHUTTER;
     cbArg.ext1 = 0;
     cbArg.ext2 = false;
     m_cbNotifier.notifyCallback(cbArg);
}

/*===========================================================================
 * FUNCTION   : getChannelByHandle
 *
 * DESCRIPTION: return a channel by its handle
 *
 * PARAMETERS :
 *   @channelHandle : channel handle
 *
 * RETURN     : a channel obj if found, NULL if not found
 *==========================================================================*/
QCameraChannel *QCamera2HardwareInterface::getChannelByHandle(uint32_t channelHandle)
{
    for(int i = 0; i < QCAMERA_CH_TYPE_MAX; i++) {
        if (m_channels[i] != NULL &&
            m_channels[i]->getMyHandle() == channelHandle) {
            return m_channels[i];
        }
    }

    return NULL;
}
/*===========================================================================
 * FUNCTION   : needPreviewFDCallback
 *
 * DESCRIPTION: decides if needPreviewFDCallback
 *
 * PARAMETERS :
 *   @fd_data : number of faces
 *
 * RETURN     : bool type of status
 *              true  -- success
 *              fale -- failure code
 *==========================================================================*/
bool QCamera2HardwareInterface::needPreviewFDCallback(uint8_t num_faces)
{
    if (num_faces == 0 && mNumPreviewFaces == 0) {
        return false;
    }

    return true;
}

/*===========================================================================
 * FUNCTION   : processFaceDetectionReuslt
 *
 * DESCRIPTION: process face detection reuslt
 *
 * PARAMETERS :
 *   @fd_data : ptr to face detection result struct
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::processFaceDetectionResult(cam_face_detection_data_t *fd_data)
{
    if (!mParameters.isFaceDetectionEnabled()) {
        CDBG_HIGH("%s: FaceDetection not enabled, no ops here", __func__);
        return NO_ERROR;
    }

    qcamera_face_detect_type_t fd_type = fd_data->fd_type;
    if ((NULL == mDataCb) ||
        (fd_type == QCAMERA_FD_PREVIEW && !msgTypeEnabled(CAMERA_MSG_PREVIEW_METADATA)) ||
        (!needPreviewFDCallback(fd_data->num_faces_detected))
#ifndef VANILLA_HAL
        || (fd_type == QCAMERA_FD_SNAPSHOT && !msgTypeEnabled(CAMERA_MSG_META_DATA))
#endif
        ) {
        CDBG_HIGH("%s: metadata msgtype not enabled, no ops here", __func__);
        return NO_ERROR;
    }

    if ((fd_type == QCAMERA_FD_PREVIEW) && (fd_data->update_flag == FALSE)) {
        // Don't send callback to app if this is skipped by fd at backend
        return NO_ERROR;
    }

    cam_dimension_t display_dim;
    mParameters.getStreamDimension(CAM_STREAM_TYPE_PREVIEW, display_dim);
    if (display_dim.width <= 0 || display_dim.height <= 0) {
        ALOGE("%s: Invalid preview width or height (%d x %d)",
              __func__, display_dim.width, display_dim.height);
        return UNKNOWN_ERROR;
    }

    // process face detection result
    // need separate face detection in preview or snapshot type
    size_t faceResultSize = 0;
    size_t data_len = 0;
    if(fd_type == QCAMERA_FD_PREVIEW){
        //fd for preview frames
        faceResultSize = sizeof(camera_frame_metadata_t);
        faceResultSize += sizeof(camera_face_t) * MAX_ROI;
    }else if(fd_type == QCAMERA_FD_SNAPSHOT){
#ifndef VANILLA_HAL
        // fd for snapshot frames
        //check if face is detected in this frame
        if(fd_data->num_faces_detected > 0){
            data_len = sizeof(camera_frame_metadata_t) +
                         sizeof(camera_face_t) * fd_data->num_faces_detected;
        }else{
            //no face
            data_len = 0;
        }
#endif
        faceResultSize = 1 *sizeof(int)    //meta data type
                       + 1 *sizeof(int)    // meta data len
                       + data_len;         //data
    }

    camera_memory_t *faceResultBuffer = mGetMemory(-1,
                                                   faceResultSize,
                                                   1,
                                                   mCallbackCookie);
    if ( NULL == faceResultBuffer ) {
        ALOGE("%s: Not enough memory for face result data",
              __func__);
        return NO_MEMORY;
    }

    unsigned char *pFaceResult = ( unsigned char * ) faceResultBuffer->data;
    memset(pFaceResult, 0, faceResultSize);
    unsigned char *faceData = NULL;
    if(fd_type == QCAMERA_FD_PREVIEW){
        faceData = pFaceResult;
        mNumPreviewFaces = fd_data->num_faces_detected;
    }else if(fd_type == QCAMERA_FD_SNAPSHOT){
#ifndef VANILLA_HAL
        //need fill meta type and meta data len first
        int *data_header = (int* )pFaceResult;
        data_header[0] = CAMERA_META_DATA_FD;
        data_header[1] = (int)data_len;

        if(data_len <= 0){
            //if face is not valid or do not have face, return
            qcamera_callback_argm_t cbArg;
            memset(&cbArg, 0, sizeof(qcamera_callback_argm_t));
            cbArg.cb_type = QCAMERA_DATA_CALLBACK;
            cbArg.msg_type = CAMERA_MSG_META_DATA;
            cbArg.data = faceResultBuffer;
            cbArg.user_data = faceResultBuffer;
            cbArg.cookie = this;
            cbArg.release_cb = releaseCameraMemory;
            int32_t rc = m_cbNotifier.notifyCallback(cbArg);
            if (rc != NO_ERROR) {
                ALOGE("%s: fail sending notification", __func__);
                faceResultBuffer->release(faceResultBuffer);
            }
            return rc;
        }
#endif
        faceData = pFaceResult + 2 *sizeof(int); //skip two int length
    }

    camera_frame_metadata_t *roiData = (camera_frame_metadata_t * ) faceData;
    camera_face_t *faces = (camera_face_t *) ( faceData + sizeof(camera_frame_metadata_t) );

    roiData->number_of_faces = fd_data->num_faces_detected;
    roiData->faces = faces;
    if (roiData->number_of_faces > 0) {
        for (int i = 0; i < roiData->number_of_faces; i++) {
            faces[i].id = fd_data->faces[i].face_id;
            faces[i].score = fd_data->faces[i].score;

            // left
            faces[i].rect[0] =
                MAP_TO_DRIVER_COORDINATE(fd_data->faces[i].face_boundary.left, display_dim.width, 2000, -1000);

            // top
            faces[i].rect[1] =
                MAP_TO_DRIVER_COORDINATE(fd_data->faces[i].face_boundary.top, display_dim.height, 2000, -1000);

            // right
            faces[i].rect[2] = faces[i].rect[0] +
                MAP_TO_DRIVER_COORDINATE(fd_data->faces[i].face_boundary.width, display_dim.width, 2000, 0);

             // bottom
            faces[i].rect[3] = faces[i].rect[1] +
                MAP_TO_DRIVER_COORDINATE(fd_data->faces[i].face_boundary.height, display_dim.height, 2000, 0);

            // Center of left eye
            faces[i].left_eye[0] =
                MAP_TO_DRIVER_COORDINATE(fd_data->faces[i].left_eye_center.x, display_dim.width, 2000, -1000);

            faces[i].left_eye[1] =
                MAP_TO_DRIVER_COORDINATE(fd_data->faces[i].left_eye_center.y, display_dim.height, 2000, -1000);

            // Center of right eye
            faces[i].right_eye[0] =
                MAP_TO_DRIVER_COORDINATE(fd_data->faces[i].right_eye_center.x, display_dim.width, 2000, -1000);

            faces[i].right_eye[1] =
                MAP_TO_DRIVER_COORDINATE(fd_data->faces[i].right_eye_center.y, display_dim.height, 2000, -1000);

            // Center of mouth
            faces[i].mouth[0] =
                MAP_TO_DRIVER_COORDINATE(fd_data->faces[i].mouth_center.x, display_dim.width, 2000, -1000);

            faces[i].mouth[1] =
                MAP_TO_DRIVER_COORDINATE(fd_data->faces[i].mouth_center.y, display_dim.height, 2000, -1000);

#ifndef VANILLA_HAL
#ifdef TARGET_TS_MAKEUP
            mFaceRect.left = fd_data->faces[i].face_boundary.left;
            mFaceRect.top = fd_data->faces[i].face_boundary.top;
            mFaceRect.right = fd_data->faces[i].face_boundary.width+mFaceRect.left;
            mFaceRect.bottom = fd_data->faces[i].face_boundary.height+mFaceRect.top;
#endif
            faces[i].smile_degree = fd_data->faces[i].smile_degree;
            faces[i].smile_score = fd_data->faces[i].smile_confidence;
            faces[i].blink_detected = fd_data->faces[i].blink_detected;
            faces[i].face_recognised = fd_data->faces[i].face_recognised;
            faces[i].gaze_angle = fd_data->faces[i].gaze_angle;

            faces[i].updown_dir = fd_data->faces[i].updown_dir;
            faces[i].leftright_dir = fd_data->faces[i].leftright_dir;
            faces[i].roll_dir = fd_data->faces[i].roll_dir;

            faces[i].leye_blink = fd_data->faces[i].left_blink;
            faces[i].reye_blink = fd_data->faces[i].right_blink;
            faces[i].left_right_gaze = fd_data->faces[i].left_right_gaze;
            faces[i].top_bottom_gaze = fd_data->faces[i].top_bottom_gaze;
#endif

        }
    }
    else{
#ifdef TARGET_TS_MAKEUP
        memset(&mFaceRect,-1,sizeof(mFaceRect));
#endif
    }
    qcamera_callback_argm_t cbArg;
    memset(&cbArg, 0, sizeof(qcamera_callback_argm_t));
    cbArg.cb_type = QCAMERA_DATA_CALLBACK;
    if(fd_type == QCAMERA_FD_PREVIEW){
        cbArg.msg_type = CAMERA_MSG_PREVIEW_METADATA;
    }
#ifndef VANILLA_HAL
    else if(fd_type == QCAMERA_FD_SNAPSHOT){
        cbArg.msg_type = CAMERA_MSG_META_DATA;
    }
#endif
    cbArg.data = faceResultBuffer;
    cbArg.metadata = roiData;
    cbArg.user_data = faceResultBuffer;
    cbArg.cookie = this;
    cbArg.release_cb = releaseCameraMemory;
    int32_t rc = m_cbNotifier.notifyCallback(cbArg);
    if (rc != NO_ERROR) {
        ALOGE("%s: fail sending notification", __func__);
        faceResultBuffer->release(faceResultBuffer);
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : releaseCameraMemory
 *
 * DESCRIPTION: releases camera memory objects
 *
 * PARAMETERS :
 *   @data    : buffer to be released
 *   @cookie  : context data
 *   @cbStatus: callback status
 *
 * RETURN     : None
 *==========================================================================*/
void QCamera2HardwareInterface::releaseCameraMemory(void *data,
                                                    void */*cookie*/,
                                                    int32_t /*cbStatus*/)
{
    camera_memory_t *mem = ( camera_memory_t * ) data;
    if ( NULL != mem ) {
        mem->release(mem);
    }
}

/*===========================================================================
 * FUNCTION   : returnStreamBuffer
 *
 * DESCRIPTION: returns back a stream buffer
 *
 * PARAMETERS :
 *   @data    : buffer to be released
 *   @cookie  : context data
 *   @cbStatus: callback status
 *
 * RETURN     : None
 *==========================================================================*/
void QCamera2HardwareInterface::returnStreamBuffer(void *data,
                                                   void *cookie,
                                                   int32_t /*cbStatus*/)
{
    QCameraStream *stream = ( QCameraStream * ) cookie;
    int idx = *((int *)data);
    if ((NULL != stream) && (0 <= idx)) {
        stream->bufDone((uint32_t)idx);
    } else {
        ALOGE("%s: Cannot return buffer %d %p", __func__, idx, cookie);
    }
}

/*===========================================================================
 * FUNCTION   : processHistogramStats
 *
 * DESCRIPTION: process histogram stats
 *
 * PARAMETERS :
 *   @hist_data : ptr to histogram stats struct
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::processHistogramStats(cam_hist_stats_t &stats_data)
{
#ifndef VANILLA_HAL
    if (!mParameters.isHistogramEnabled()) {
        CDBG_HIGH("%s: Histogram not enabled, no ops here", __func__);
        return NO_ERROR;
    }

    camera_memory_t *histBuffer = mGetMemory(-1,
                                             sizeof(cam_histogram_data_t),
                                             1,
                                             mCallbackCookie);
    if ( NULL == histBuffer ) {
        ALOGE("%s: Not enough memory for histogram data",
              __func__);
        return NO_MEMORY;
    }

    cam_histogram_data_t *pHistData = (cam_histogram_data_t *)histBuffer->data;
    if (pHistData == NULL) {
        ALOGE("%s: memory data ptr is NULL", __func__);
        return UNKNOWN_ERROR;
    }

    switch (stats_data.type) {
    case CAM_HISTOGRAM_TYPE_BAYER:
        *pHistData = stats_data.bayer_stats.gb_stats;
        break;
    case CAM_HISTOGRAM_TYPE_YUV:
        *pHistData = stats_data.yuv_stats;
        break;
    }

    qcamera_callback_argm_t cbArg;
    memset(&cbArg, 0, sizeof(qcamera_callback_argm_t));
    cbArg.cb_type = QCAMERA_DATA_CALLBACK;
    cbArg.msg_type = CAMERA_MSG_STATS_DATA;
    cbArg.data = histBuffer;
    cbArg.user_data = histBuffer;
    cbArg.cookie = this;
    cbArg.release_cb = releaseCameraMemory;
    int32_t rc = m_cbNotifier.notifyCallback(cbArg);
    if (rc != NO_ERROR) {
        ALOGE("%s: fail sending notification", __func__);
        histBuffer->release(histBuffer);
    }
#endif
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : calcThermalLevel
 *
 * DESCRIPTION: Calculates the target fps range depending on
 *              the thermal level.
 *
 * PARAMETERS :
 *   @level    : received thermal level
 *   @minFPS   : minimum configured fps range
 *   @maxFPS   : maximum configured fps range
 *   @adjustedRange : target fps range
 *   @skipPattern : target skip pattern
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::calcThermalLevel(
            qcamera_thermal_level_enum_t level,
            const int minFPSi,
            const int maxFPSi,
            cam_fps_range_t &adjustedRange,
            enum msm_vfe_frame_skip_pattern &skipPattern)
{
    const float minFPS = (float)minFPSi;
    const float maxFPS = (float)maxFPSi;

    // Initialize video fps to preview fps
    float minVideoFps = minFPS, maxVideoFps = maxFPS;
    cam_fps_range_t videoFps;
    // If HFR mode, update video fps accordingly
    if(isHFRMode()) {
        mParameters.getHfrFps(videoFps);
        minVideoFps = videoFps.video_min_fps;
        maxVideoFps = videoFps.video_max_fps;
    }

    CDBG_HIGH("%s: level: %d, preview minfps %f, preview maxfpS %f, "
              "video minfps %f, video maxfpS %f",
            __func__, level, minFPS, maxFPS, minVideoFps, maxVideoFps);

    switch(level) {
    case QCAMERA_THERMAL_NO_ADJUSTMENT:
        {
            adjustedRange.min_fps = minFPS / 1000.0f;
            adjustedRange.max_fps = maxFPS / 1000.0f;
            adjustedRange.video_min_fps = minVideoFps / 1000.0f;
            adjustedRange.video_max_fps = maxVideoFps / 1000.0f;
            skipPattern = NO_SKIP;
        }
        break;
    case QCAMERA_THERMAL_SLIGHT_ADJUSTMENT:
        {
            adjustedRange.min_fps = minFPS / 1000.0f;
            adjustedRange.max_fps = maxFPS / 1000.0f;
            adjustedRange.min_fps -= 0.1f * adjustedRange.min_fps;
            adjustedRange.max_fps -= 0.1f * adjustedRange.max_fps;
            adjustedRange.video_min_fps = minVideoFps / 1000.0f;
            adjustedRange.video_max_fps = maxVideoFps / 1000.0f;
            adjustedRange.video_min_fps -= 0.1f * adjustedRange.video_min_fps;
            adjustedRange.video_max_fps -= 0.1f * adjustedRange.video_max_fps;
            if ( adjustedRange.min_fps < 1 ) {
                adjustedRange.min_fps = 1;
            }
            if ( adjustedRange.max_fps < 1 ) {
                adjustedRange.max_fps = 1;
            }
            if ( adjustedRange.video_min_fps < 1 ) {
                adjustedRange.video_min_fps = 1;
            }
            if ( adjustedRange.video_max_fps < 1 ) {
                adjustedRange.video_max_fps = 1;
            }
            skipPattern = EVERY_2FRAME;
        }
        break;
    case QCAMERA_THERMAL_BIG_ADJUSTMENT:
        {
            adjustedRange.min_fps = minFPS / 1000.0f;
            adjustedRange.max_fps = maxFPS / 1000.0f;
            adjustedRange.min_fps -= 0.2f * adjustedRange.min_fps;
            adjustedRange.max_fps -= 0.2f * adjustedRange.max_fps;
            adjustedRange.video_min_fps = minVideoFps / 1000.0f;
            adjustedRange.video_max_fps = maxVideoFps / 1000.0f;
            adjustedRange.video_min_fps -= 0.2f * adjustedRange.video_min_fps;
            adjustedRange.video_max_fps -= 0.2f * adjustedRange.video_max_fps;
            if ( adjustedRange.min_fps < 1 ) {
                adjustedRange.min_fps = 1;
            }
            if ( adjustedRange.max_fps < 1 ) {
                adjustedRange.max_fps = 1;
            }
            if ( adjustedRange.video_min_fps < 1 ) {
                adjustedRange.video_min_fps = 1;
            }
            if ( adjustedRange.video_max_fps < 1 ) {
                adjustedRange.video_max_fps = 1;
            }
            skipPattern = EVERY_4FRAME;
        }
        break;
    case QCAMERA_THERMAL_MAX_ADJUSTMENT:
        {
            // Stop Preview?
            // Set lowest min FPS for now
            adjustedRange.min_fps = minFPS/1000.0f;
            adjustedRange.max_fps = minFPS/1000.0f;
            cam_capability_t *capability = gCamCapability[mCameraId];
            for (size_t i = 0;
                     i < capability->fps_ranges_tbl_cnt;
                     i++) {
                if (capability->fps_ranges_tbl[i].min_fps <
                        adjustedRange.min_fps) {
                    adjustedRange.min_fps =
                            capability->fps_ranges_tbl[i].min_fps;
                    adjustedRange.max_fps = adjustedRange.min_fps;
                }
            }
            skipPattern = MAX_SKIP;
            adjustedRange.video_min_fps = adjustedRange.min_fps;
            adjustedRange.video_max_fps = adjustedRange.max_fps;
        }
        break;
    case QCAMERA_THERMAL_SHUTDOWN:
        {
            // send error notify
            ALOGE("%s: Received shutdown thermal level. Closing camera", __func__);
            sendEvtNotify(CAMERA_MSG_ERROR, CAMERA_ERROR_SERVER_DIED, 0);
        }
        break;
    default:
        {
            ALOGE("%s: Invalid thermal level %d", __func__, level);
            return BAD_VALUE;
        }
        break;
    }
    if (level >= QCAMERA_THERMAL_NO_ADJUSTMENT && level <= QCAMERA_THERMAL_MAX_ADJUSTMENT) {
        if (mParameters.getRecordingHintValue() == true) {
            adjustedRange.min_fps = minFPS / 1000.0f;
            adjustedRange.max_fps = maxFPS / 1000.0f;
            adjustedRange.video_min_fps = minVideoFps / 1000.0f;
            adjustedRange.video_max_fps = maxVideoFps / 1000.0f;
            skipPattern = NO_SKIP;
            CDBG_HIGH("%s: No FPS mitigation in camcorder mode", __func__);
        }
        CDBG_HIGH("%s: Thermal level %d, FPS [%3.2f,%3.2f, %3.2f,%3.2f], frameskip %d",
                __func__, level, adjustedRange.min_fps, adjustedRange.max_fps,
                adjustedRange.video_min_fps, adjustedRange.video_max_fps,skipPattern);
    }

    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : recalcFPSRange
 *
 * DESCRIPTION: adjust the configured fps range regarding
 *              the last thermal level.
 *
 * PARAMETERS :
 *   @minFPS   : minimum configured fps range
 *   @maxFPS   : maximum configured fps range
 *   @adjustedRange : target fps range
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::recalcFPSRange(int &minFPS, int &maxFPS,
        cam_fps_range_t &adjustedRange)
{
    enum msm_vfe_frame_skip_pattern skipPattern;
    calcThermalLevel(mThermalLevel,
                     minFPS,
                     maxFPS,
                     adjustedRange,
                     skipPattern);
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : updateThermalLevel
 *
 * DESCRIPTION: update thermal level depending on thermal events
 *
 * PARAMETERS :
 *   @level   : thermal level
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::updateThermalLevel(void *thermal_level)
{
    int ret = NO_ERROR;
    cam_fps_range_t adjustedRange;
    int minFPS, maxFPS;
    enum msm_vfe_frame_skip_pattern skipPattern;
    qcamera_thermal_level_enum_t level = *(qcamera_thermal_level_enum_t *)thermal_level;

    pthread_mutex_lock(&m_parm_lock);

    if (!mCameraOpened) {
        CDBG_HIGH("%s: Camera is not opened, no need to update camera parameters", __func__);
        pthread_mutex_unlock(&m_parm_lock);
        return NO_ERROR;
    }

    mParameters.getPreviewFpsRange(&minFPS, &maxFPS);
    qcamera_thermal_mode thermalMode = mParameters.getThermalMode();
    calcThermalLevel(level, minFPS, maxFPS, adjustedRange, skipPattern);
    mThermalLevel = level;

    if (thermalMode == QCAMERA_THERMAL_ADJUST_FPS)
        ret = mParameters.adjustPreviewFpsRange(&adjustedRange);
    else if (thermalMode == QCAMERA_THERMAL_ADJUST_FRAMESKIP)
        ret = mParameters.setFrameSkip(skipPattern);
    else
        ALOGE("%s: Incorrect thermal mode %d", __func__, thermalMode);

    pthread_mutex_unlock(&m_parm_lock);

    return ret;

}

/*===========================================================================
 * FUNCTION   : updateParameters
 *
 * DESCRIPTION: update parameters
 *
 * PARAMETERS :
 *   @parms       : input parameters string
 *   @needRestart : output, flag to indicate if preview restart is needed
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2HardwareInterface::updateParameters(const char *parms, bool &needRestart)
{
    int rc = NO_ERROR;

    pthread_mutex_lock(&m_parm_lock);
    String8 str = String8(parms);
    QCameraParameters param(str);
    rc =  mParameters.updateParameters(param, needRestart);

    // update stream based parameter settings
    for (int i = 0; i < QCAMERA_CH_TYPE_MAX; i++) {
        if (m_channels[i] != NULL) {
            m_channels[i]->UpdateStreamBasedParameters(mParameters);
        }
    }
    pthread_mutex_unlock(&m_parm_lock);

    return rc;
}

/*===========================================================================
 * FUNCTION   : commitParameterChanges
 *
 * DESCRIPTION: commit parameter changes to the backend to take effect
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 * NOTE       : This function must be called after updateParameters.
 *              Otherwise, no change will be passed to backend to take effect.
 *==========================================================================*/
int QCamera2HardwareInterface::commitParameterChanges()
{
    int rc = NO_ERROR;
    pthread_mutex_lock(&m_parm_lock);
    rc = mParameters.commitParameters();
    if (rc == NO_ERROR) {
        // update number of snapshot based on committed parameters setting
        rc = mParameters.setNumOfSnapshot();
    }
    pthread_mutex_unlock(&m_parm_lock);
    return rc;
}

/*===========================================================================
 * FUNCTION   : needDebugFps
 *
 * DESCRIPTION: if fps log info need to be printed out
 *
 * PARAMETERS : none
 *
 * RETURN     : true: need print out fps log
 *              false: no need to print out fps log
 *==========================================================================*/
bool QCamera2HardwareInterface::needDebugFps()
{
    bool needFps = false;
    pthread_mutex_lock(&m_parm_lock);
    needFps = mParameters.isFpsDebugEnabled();
    pthread_mutex_unlock(&m_parm_lock);
    return needFps;
}

/*===========================================================================
 * FUNCTION   : isCACEnabled
 *
 * DESCRIPTION: if CAC is enabled
 *
 * PARAMETERS : none
 *
 * RETURN     : true: needed
 *              false: no need
 *==========================================================================*/
bool QCamera2HardwareInterface::isCACEnabled()
{
    char prop[PROPERTY_VALUE_MAX];
    memset(prop, 0, sizeof(prop));
    property_get("persist.camera.feature.cac", prop, "0");
    int enableCAC = atoi(prop);
    return enableCAC == 1;
}

/*===========================================================================
 * FUNCTION   : is4k2kResolution
 *
 * DESCRIPTION: if resolution is 4k x 2k or true 4k x 2k
 *
 * PARAMETERS : none
 *
 * RETURN     : true: needed
 *              false: no need
 *==========================================================================*/
bool QCamera2HardwareInterface::is4k2kResolution(cam_dimension_t* resolution)
{
   bool enabled = false;
   if ((resolution->width == 4096 && resolution->height == 2160) ||
       (resolution->width == 3840 && resolution->height == 2160) ) {
      enabled = true;
   }
   return enabled;
}

/*===========================================================================
 * FUNCTION   : isPreviewRestartEnabled
 *
 * DESCRIPTION: Check whether preview should be restarted automatically
 *              during image capture.
 *
 * PARAMETERS : none
 *
 * RETURN     : true: needed
 *              false: no need
 *==========================================================================*/
bool QCamera2HardwareInterface::isPreviewRestartEnabled()
{
    char prop[PROPERTY_VALUE_MAX];
    memset(prop, 0, sizeof(prop));
    property_get("persist.camera.feature.restart", prop, "0");
    int earlyRestart = atoi(prop);
    return earlyRestart == 1;
}

/*===========================================================================
 * FUNCTION   : needReprocess
 *
 * DESCRIPTION: if reprocess is needed
 *
 * PARAMETERS : none
 *
 * RETURN     : true: needed
 *              false: no need
 *==========================================================================*/
bool QCamera2HardwareInterface::needReprocess()
{
    bool needReprocess = false;
    pthread_mutex_lock(&m_parm_lock);

    if (!mParameters.isJpegPictureFormat() &&
        !mParameters.isNV21PictureFormat()) {
        // RAW image, no need to reprocess
        pthread_mutex_unlock(&m_parm_lock);
        return false;
    }

    //Disable reprocess for 4K liveshot case but enable if lowpower mode
    if (mParameters.is4k2kVideoResolution() && mParameters.getRecordingHintValue()
            && !isLowPowerMode()) {
        pthread_mutex_unlock(&m_parm_lock);
        return false;
    }

    // pp feature config
    cam_pp_feature_config_t pp_config;
    memset(&pp_config, 0, sizeof(cam_pp_feature_config_t));

    //Decide whether to do reprocess or not based on
    //ppconfig obtained in the first pass.
    getPPConfig(pp_config, 1);

    if (pp_config.feature_mask > 0) {
        needReprocess = true;
    }

    CDBG_HIGH("%s: needReprocess %s", __func__, needReprocess ? "true" : "false");
    pthread_mutex_unlock(&m_parm_lock);
    return needReprocess;
}


/*===========================================================================
 * FUNCTION   : needRotationReprocess
 *
 * DESCRIPTION: if rotation needs to be done by reprocess in pp
 *
 * PARAMETERS : none
 *
 * RETURN     : true: needed
 *              false: no need
 *==========================================================================*/
bool QCamera2HardwareInterface::needRotationReprocess()
{
    pthread_mutex_lock(&m_parm_lock);
    if (!mParameters.isJpegPictureFormat() &&
        !mParameters.isNV21PictureFormat()) {
        // RAW image, no need to reprocess
        pthread_mutex_unlock(&m_parm_lock);
        return false;
    }

    //Disable reprocess for 4K liveshot case
    if (mParameters.is4k2kVideoResolution() && mParameters.getRecordingHintValue()
            && !isLowPowerMode()) {
        //Disable reprocess for 4K liveshot case
        pthread_mutex_unlock(&m_parm_lock);
        return false;
    }

    if ((gCamCapability[mCameraId]->qcom_supported_feature_mask &
            CAM_QCOM_FEATURE_ROTATION) > 0 &&
            (mParameters.getJpegRotation() > 0)) {
        // current rotation is not zero, and pp has the capability to process rotation
        CDBG_HIGH("%s: need to do reprocess for rotation=%d",
                __func__, mParameters.getJpegRotation());
        pthread_mutex_unlock(&m_parm_lock);
        return true;
    }

    pthread_mutex_unlock(&m_parm_lock);
    return false;
}

/*===========================================================================
 * FUNCTION   : getThumbnailSize
 *
 * DESCRIPTION: get user set thumbnail size
 *
 * PARAMETERS :
 *   @dim     : output of thumbnail dimension
 *
 * RETURN     : none
 *==========================================================================*/
void QCamera2HardwareInterface::getThumbnailSize(cam_dimension_t &dim)
{
    pthread_mutex_lock(&m_parm_lock);
    mParameters.getThumbnailSize(&dim.width, &dim.height);
    pthread_mutex_unlock(&m_parm_lock);
}

/*===========================================================================
 * FUNCTION   : getJpegQuality
 *
 * DESCRIPTION: get user set jpeg quality
 *
 * PARAMETERS : none
 *
 * RETURN     : jpeg quality setting
 *==========================================================================*/
uint32_t QCamera2HardwareInterface::getJpegQuality()
{
    uint32_t quality = 0;
    pthread_mutex_lock(&m_parm_lock);
    quality =  mParameters.getJpegQuality();
    pthread_mutex_unlock(&m_parm_lock);
    return quality;
}

/*===========================================================================
 * FUNCTION   : getExifData
 *
 * DESCRIPTION: get exif data to be passed into jpeg encoding
 *
 * PARAMETERS : none
 *
 * RETURN     : exif data from user setting and GPS
 *==========================================================================*/
QCameraExif *QCamera2HardwareInterface::getExifData()
{
    QCameraExif *exif = new QCameraExif();
    if (exif == NULL) {
        ALOGE("%s: No memory for QCameraExif", __func__);
        return NULL;
    }

    int32_t rc = NO_ERROR;

    pthread_mutex_lock(&m_parm_lock);

    // add exif entries
    String8 dateTime(""), subSecTime("");
    rc = mParameters.getExifDateTime(dateTime, subSecTime);
    if(rc == NO_ERROR) {
        exif->addEntry(EXIFTAGID_DATE_TIME, EXIF_ASCII,
                (uint32_t)(dateTime.length() + 1), (void *)dateTime.string());
        exif->addEntry(EXIFTAGID_EXIF_DATE_TIME_ORIGINAL, EXIF_ASCII,
                (uint32_t)(dateTime.length() + 1), (void *)dateTime.string());
        exif->addEntry(EXIFTAGID_EXIF_DATE_TIME_DIGITIZED, EXIF_ASCII,
                (uint32_t)(dateTime.length() + 1), (void *)dateTime.string());
        exif->addEntry(EXIFTAGID_SUBSEC_TIME, EXIF_ASCII,
                (uint32_t)(subSecTime.length() + 1), (void *)subSecTime.string());
        exif->addEntry(EXIFTAGID_SUBSEC_TIME_ORIGINAL, EXIF_ASCII,
                (uint32_t)(subSecTime.length() + 1), (void *)subSecTime.string());
        exif->addEntry(EXIFTAGID_SUBSEC_TIME_DIGITIZED, EXIF_ASCII,
                (uint32_t)(subSecTime.length() + 1), (void *)subSecTime.string());
    } else {
        ALOGE("%s: getExifDateTime failed", __func__);
    }

    rat_t focalLength;
    rc = mParameters.getExifFocalLength(&focalLength);
    if (rc == NO_ERROR) {
        exif->addEntry(EXIFTAGID_FOCAL_LENGTH,
                       EXIF_RATIONAL,
                       1,
                       (void *)&(focalLength));
    } else {
        ALOGE("%s: getExifFocalLength failed", __func__);
    }

    uint16_t isoSpeed = mParameters.getExifIsoSpeed();
    if (getSensorType() != CAM_SENSOR_YUV) {
        exif->addEntry(EXIFTAGID_ISO_SPEED_RATING,
                       EXIF_SHORT,
                       1,
                       (void *)&(isoSpeed));
    }

    char gpsProcessingMethod[EXIF_ASCII_PREFIX_SIZE + GPS_PROCESSING_METHOD_SIZE];
    uint32_t count = 0;
    rc = mParameters.getExifGpsProcessingMethod(gpsProcessingMethod, count);
    if(rc == NO_ERROR) {
        exif->addEntry(EXIFTAGID_GPS_PROCESSINGMETHOD,
                       EXIF_ASCII,
                       count,
                       (void *)gpsProcessingMethod);
    } else {
        ALOGE("%s: getExifGpsProcessingMethod failed", __func__);
    }

    rat_t latitude[3];
    char latRef[2];
    rc = mParameters.getExifLatitude(latitude, latRef);
    if(rc == NO_ERROR) {
        exif->addEntry(EXIFTAGID_GPS_LATITUDE,
                       EXIF_RATIONAL,
                       3,
                       (void *)latitude);
        exif->addEntry(EXIFTAGID_GPS_LATITUDE_REF,
                       EXIF_ASCII,
                       2,
                       (void *)latRef);
    } else {
        ALOGE("%s: getExifLatitude failed", __func__);
    }

    rat_t longitude[3];
    char lonRef[2];
    rc = mParameters.getExifLongitude(longitude, lonRef);
    if(rc == NO_ERROR) {
        exif->addEntry(EXIFTAGID_GPS_LONGITUDE,
                       EXIF_RATIONAL,
                       3,
                       (void *)longitude);

        exif->addEntry(EXIFTAGID_GPS_LONGITUDE_REF,
                       EXIF_ASCII,
                       2,
                       (void *)lonRef);
    } else {
        ALOGE("%s: getExifLongitude failed", __func__);
    }

    rat_t altitude;
    char altRef;
    rc = mParameters.getExifAltitude(&altitude, &altRef);
    if(rc == NO_ERROR) {
        exif->addEntry(EXIFTAGID_GPS_ALTITUDE,
                       EXIF_RATIONAL,
                       1,
                       (void *)&(altitude));

        exif->addEntry(EXIFTAGID_GPS_ALTITUDE_REF,
                       EXIF_BYTE,
                       1,
                       (void *)&altRef);
    } else {
        ALOGE("%s: getExifAltitude failed", __func__);
    }

    char gpsDateStamp[20];
    rat_t gpsTimeStamp[3];
    rc = mParameters.getExifGpsDateTimeStamp(gpsDateStamp, 20, gpsTimeStamp);
    if(rc == NO_ERROR) {
        exif->addEntry(EXIFTAGID_GPS_DATESTAMP,
                       EXIF_ASCII,
                       (uint32_t)(strlen(gpsDateStamp) + 1),
                       (void *)gpsDateStamp);

        exif->addEntry(EXIFTAGID_GPS_TIMESTAMP,
                       EXIF_RATIONAL,
                       3,
                       (void *)gpsTimeStamp);
    } else {
        ALOGE("%s: getExifGpsDataTimeStamp failed", __func__);
    }

    char value[PROPERTY_VALUE_MAX];
    if (property_get("persist.sys.exif.make", value, "") > 0 ||
            property_get("ro.product.manufacturer", value, "QCOM-AA") > 0) {
        exif->addEntry(EXIFTAGID_MAKE,
                EXIF_ASCII, strlen(value) + 1, (void *)value);
    } else {
        ALOGE("%s: getExifMaker failed", __func__);
    }

    if (property_get("persist.sys.exif.model", value, "") > 0 ||
            property_get("ro.product.model", value, "QCAM-AA") > 0) {
        exif->addEntry(EXIFTAGID_MODEL,
                EXIF_ASCII, strlen(value) + 1, (void *)value);
    } else {
        ALOGE("%s: getExifModel failed", __func__);
    }

    if (property_get("ro.build.description", value, "QCAM-AA") > 0) {
        exif->addEntry(EXIFTAGID_SOFTWARE, EXIF_ASCII,
                (uint32_t)(strlen(value) + 1), (void *)value);
    } else {
        ALOGE("%s: getExifSoftware failed", __func__);
    }

    if (mParameters.useJpegExifRotation()) {
        int16_t orientation;
        switch (mParameters.getJpegExifRotation()) {
        case 0:
            orientation = 1;
            break;
        case 90:
            orientation = 6;
            break;
        case 180:
            orientation = 3;
            break;
        case 270:
            orientation = 8;
            break;
        default:
            orientation = 1;
            break;
        }
        exif->addEntry(EXIFTAGID_ORIENTATION,
                EXIF_SHORT,
                1,
                (void *)&orientation);
        exif->addEntry(EXIFTAGID_TN_ORIENTATION,
                EXIF_SHORT,
                1,
                (void *)&orientation);
    }

    pthread_mutex_unlock(&m_parm_lock);
    return exif;
}

/*===========================================================================
 * FUNCTION   : setHistogram
 *
 * DESCRIPTION: set if histogram should be enabled
 *
 * PARAMETERS :
 *   @histogram_en : bool flag if histogram should be enabled
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::setHistogram(bool histogram_en)
{
    int32_t rc = NO_ERROR;
    pthread_mutex_lock(&m_parm_lock);
    rc = mParameters.setHistogram(histogram_en);
    pthread_mutex_unlock(&m_parm_lock);
    return rc;
}

/*===========================================================================
 * FUNCTION   : setFaceDetection
 *
 * DESCRIPTION: set if face detection should be enabled
 *
 * PARAMETERS :
 *   @enabled : bool flag if face detection should be enabled
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::setFaceDetection(bool enabled)
{
    int32_t rc = NO_ERROR;
    pthread_mutex_lock(&m_parm_lock);
    rc = mParameters.setFaceDetection(enabled, true);
    pthread_mutex_unlock(&m_parm_lock);
    return rc;
}

/*===========================================================================
 * FUNCTION   : isCaptureShutterEnabled
 *
 * DESCRIPTION: Check whether shutter should be triggered immediately after
 *              capture
 *
 * PARAMETERS :
 *
 * RETURN     : true - regular capture
 *              false - other type of capture
 *==========================================================================*/
bool QCamera2HardwareInterface::isCaptureShutterEnabled()
{
    char prop[PROPERTY_VALUE_MAX];
    memset(prop, 0, sizeof(prop));
    property_get("persist.camera.feature.shutter", prop, "0");
    int enableShutter = atoi(prop);
    return enableShutter == 1;
}

/*===========================================================================
 * FUNCTION   : needProcessPreviewFrame
 *
 * DESCRIPTION: returns whether preview frame need to be displayed
 *
 * PARAMETERS :
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
bool QCamera2HardwareInterface::needProcessPreviewFrame()
{
    return m_stateMachine.isPreviewRunning()
            && mParameters.isDisplayFrameNeeded()
            && !mParameters.isInstantAECEnabled();
};

/*===========================================================================
 * FUNCTION   : needSendPreviewCallback
 *
 * DESCRIPTION: returns whether preview frame need to callback to APP
 *
 * PARAMETERS :
 *
 *  RETURN     : true - need preview frame callbck
 *              false - not send preview frame callback
 *==========================================================================*/
bool QCamera2HardwareInterface::needSendPreviewCallback()
{
    return m_stateMachine.isPreviewRunning()
            && (mDataCb != NULL)
            && (msgTypeEnabledWithLock(CAMERA_MSG_PREVIEW_FRAME) > 0)
            && mParameters.isPreviewCallbackNeeded();
};

/*===========================================================================
 * FUNCTION   : prepareHardwareForSnapshot
 *
 * DESCRIPTION: prepare hardware for snapshot, such as LED
 *
 * PARAMETERS :
 *   @afNeeded: flag indicating if Auto Focus needs to be done during preparation
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::prepareHardwareForSnapshot(int32_t afNeeded)
{
    ATRACE_CALL();
    CDBG_HIGH("[KPI Perf] %s: Prepare hardware such as LED",__func__);
    return mCameraHandle->ops->prepare_snapshot(mCameraHandle->camera_handle,
                                                afNeeded);
}

/*===========================================================================
 * FUNCTION   : needFDMetadata
 *
 * DESCRIPTION: check whether we need process Face Detection metadata in this chanel
 *
 * PARAMETERS :
 *   @channel_type: channel type
 *
  * RETURN     : true: needed
 *              false: no need
 *==========================================================================*/
bool QCamera2HardwareInterface::needFDMetadata(qcamera_ch_type_enum_t channel_type)
{
    //Note: Currently we only process ZSL channel
    bool value = false;
    if(channel_type == QCAMERA_CH_TYPE_ZSL){
        //check if FD requirement is enabled
        if(mParameters.isSnapshotFDNeeded() &&
           mParameters.isFaceDetectionEnabled()){
            value = true;
            CDBG_HIGH("%s: Face Detection metadata is required in ZSL mode.", __func__);
        }
    }

    return value;
}

/*===========================================================================
 * FUNCTION   : deferredWorkRoutine
 *
 * DESCRIPTION: data process routine that executes deferred tasks
 *
 * PARAMETERS :
 *   @data    : user data ptr (QCamera2HardwareInterface)
 *
 * RETURN     : None
 *==========================================================================*/
void *QCamera2HardwareInterface::deferredWorkRoutine(void *obj)
{
    int running = 1;
    int ret;
    uint8_t is_active = FALSE;

    QCamera2HardwareInterface *pme = (QCamera2HardwareInterface *)obj;
    QCameraCmdThread *cmdThread = &pme->mDeferredWorkThread;
    cmdThread->setName("CAM_defrdWrk");

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
            break;
        case CAMERA_CMD_TYPE_STOP_DATA_PROC:
            CDBG_HIGH("%s: stop data proc", __func__);
            is_active = FALSE;
            // signal cmd is completed
            cam_sem_post(&cmdThread->sync_sem);
            break;
        case CAMERA_CMD_TYPE_DO_NEXT_JOB:
            {
                DefWork *dw =
                    reinterpret_cast<DefWork *>(pme->mCmdQueue.dequeue());

                if ( NULL == dw ) {
                    ALOGE("%s : Invalid deferred work", __func__);
                    break;
                }

                switch( dw->cmd ) {
                case CMD_DEF_ALLOCATE_BUFF:
                    {
                        QCameraChannel * pChannel = dw->args.allocArgs.ch;

                        if ( NULL == pChannel ) {
                            ALOGE("%s : Invalid deferred work channel",
                                    __func__);
                            break;
                        }

                        cam_stream_type_t streamType = dw->args.allocArgs.type;
                        CDBG_HIGH("%s: Deferred buffer allocation started for stream type: %d",
                                __func__, streamType);

                        uint32_t iNumOfStreams = pChannel->getNumOfStreams();
                        QCameraStream *pStream = NULL;
                        for ( uint32_t i = 0; i < iNumOfStreams; ++i) {
                            pStream = pChannel->getStreamByIndex(i);

                            if ( NULL == pStream ) {
                                break;
                            }

                            if ( pStream->isTypeOf(streamType)) {
                                if ( pStream->allocateBuffers() ) {
                                    ALOGE("%s: Error allocating buffers !!!",
                                            __func__);
                                }
                                break;
                            }
                        }
                    }
                    break;
                case CMD_DEF_PPROC_START:
                    {
                        QCameraChannel * pChannel = dw->args.pprocArgs;
                        assert(pChannel);

                        if (pme->m_postprocessor.start(pChannel) != NO_ERROR) {
                            ALOGE("%s: cannot start postprocessor", __func__);
                            pme->sendEvtNotify(CAMERA_MSG_ERROR, CAMERA_ERROR_UNKNOWN, 0);
                        }
                    }
                    break;
                case CMD_DEF_METADATA_ALLOC:
                    {
                        pme->mMetadataMem = new QCameraMetadataStreamMemory(
                                QCAMERA_ION_USE_CACHE);

                        if (pme->mMetadataMem == NULL) {
                            ALOGE("%s: Unable to allocate metadata buffers",
                                    __func__);
                        } else {
                            int32_t rc = pme->mMetadataMem->allocate(
                                    dw->args.metadataAllocArgs.bufferCnt,
                                    dw->args.metadataAllocArgs.size,
                                    NON_SECURE);
                            if (rc < 0) {
                                delete pme->mMetadataMem;
                                pme->mMetadataMem = NULL;
                            }
                        }
                     }
                     break;
                case CMD_DEF_CREATE_JPEG_SESSION:
                    {
                        QCameraChannel * pChannel = dw->args.pprocArgs;
                        assert(pChannel);

                        if (pme->m_postprocessor.createJpegSession(pChannel) != NO_ERROR) {
                            ALOGE("%s: cannot create JPEG session", __func__);
                            pme->sendEvtNotify(CAMERA_MSG_ERROR, CAMERA_ERROR_UNKNOWN, 0);
                        }
                    }
                    break;
                case CMD_DEF_PPROC_INIT:
                    {
                        int32_t rc = NO_ERROR;
                        int32_t l_curr_width = 0;
                        int32_t l_curr_height = 0;
                        pme->m_max_pic_width = 0;
                        pme->m_max_pic_height = 0;

                        jpeg_encode_callback_t jpegEvtHandle =
                                dw->args.pprocInitArgs.jpeg_cb;
                        void* user_data = dw->args.pprocInitArgs.user_data;
                        QCameraPostProcessor *postProcessor =
                                &(pme->m_postprocessor);
                        uint32_t cameraId = pme->mCameraId;
                        cam_capability_t *capability =
                                gCamCapability[cameraId];
                        cam_padding_info_t padding_info;
                        cam_padding_info_t& cam_capability_padding_info =
                                capability->padding_info;

                        for(uint32_t i = 0;
                                i < capability->picture_sizes_tbl_cnt - 1;
                                i++) {
                            l_curr_width =
                                    capability->picture_sizes_tbl[i].width;
                            l_curr_height =
                                    capability->picture_sizes_tbl[i].height;

                            if ((l_curr_width * l_curr_height) >
                                    (int32_t)(pme->m_max_pic_width *
                                    pme->m_max_pic_height)) {
                                pme->m_max_pic_width = l_curr_width;
                                pme->m_max_pic_height = l_curr_height;
                            }
                        }

                        if(!pme->mJpegClientHandle) {
                            rc = pme->initJpegHandle();
                            if (rc != NO_ERROR) {
                                ALOGE("%s: Error!! creating JPEG handle failed", __func__);
                                break;
                            }
                        }
                        CDBG_HIGH("%s: mJpegClientHandle : %d", __func__, pme->mJpegClientHandle);

                        rc = postProcessor->setJpegHandle(&pme->mJpegHandle, &pme->mJpegMpoHandle,
                                pme->mJpegClientHandle);
                        if (rc != 0) {
                            ALOGE("%s: Error!! set JPEG handle failed", __func__);
                            break;
                        }

                        /* get max pic size for jpeg work buf calculation*/
                        rc = postProcessor->init(jpegEvtHandle, user_data);

                        if (rc != NO_ERROR) {
                            ALOGE("%s: cannot init postprocessor", __func__);
                            pme->mCameraHandle->ops->close_camera(
                                    pme->mCameraHandle->camera_handle);
                            pme->mCameraHandle = NULL;
                            break;
                        }

                        // update padding info from jpeg
                        postProcessor->getJpegPaddingReq(padding_info);
                        if (cam_capability_padding_info.width_padding <
                                padding_info.width_padding) {
                            cam_capability_padding_info.width_padding =
                                    padding_info.width_padding;
                        }
                        if (cam_capability_padding_info.height_padding <
                                padding_info.height_padding) {
                            cam_capability_padding_info.height_padding =
                                    padding_info.height_padding;
                        }
                        if (cam_capability_padding_info.plane_padding <
                                padding_info.plane_padding) {
                            cam_capability_padding_info.plane_padding =
                                    padding_info.plane_padding;
                        }
                    }
                    break;
                case CMD_DEF_PARAM_ALLOC:
                    {
                        pthread_mutex_lock(&pme->m_parm_lock);
                        pme->mParameters.allocate();
                        pthread_mutex_unlock(&pme->m_parm_lock);
                    }
                    break;
                case CMD_DEF_PARAM_INIT:
                    {
                        int32_t rc = NO_ERROR;
                        uint32_t camId = pme->mCameraId;
                        cam_capability_t * cap = gCamCapability[camId];

                        if (pme->mCameraHandle == NULL) {
                            ALOGE("%s: Camera handle is null", __func__);
                            break;
                        }

                        // Now PostProc need calibration data as initialization
                        // time for jpeg_open and calibration data is a
                        // get param for now, so params needs to be initialized
                        // before postproc init
                        pthread_mutex_lock(&pme->m_parm_lock);
                        pme->mParameters.init(cap,
                                pme->mCameraHandle,
                                pme);
                        rc = pme->mParameters.getRelatedCamCalibration(
                            &(pme->mJpegMetadata.otp_calibration_data));
                        CDBG("%s: Dumping Calibration Data Version Id %f rc %d",
                                __func__,
                                pme->mJpegMetadata.otp_calibration_data.calibration_format_version,
                                rc);
                        if (rc != 0) {
                            pthread_mutex_unlock(&pme->m_parm_lock);
                            ALOGE("getRelatedCamCalibration failed");
                            pme->mCameraHandle->ops->close_camera(
                                pme->mCameraHandle->camera_handle);
                            pme->mCameraHandle = NULL;
                            break;
                        }
                        pme->m_bRelCamCalibValid = true;
                        pme->mJpegMetadata.sensor_mount_angle  =
                            cap->sensor_mount_angle;
                        pme->mJpegMetadata.default_sensor_flip = FLIP_NONE;

                        pme->mParameters.setMinPpMask(
                            cap->qcom_supported_feature_mask);
                        pthread_mutex_unlock(&pme->m_parm_lock);
                        pme->mExifParams.debug_params =
                                (mm_jpeg_debug_exif_params_t *)
                                malloc(sizeof(mm_jpeg_debug_exif_params_t));
                        if (!pme->mExifParams.debug_params) {
                            ALOGE("%s: Out of Memory. Allocation failed for "
                                    "3A debug exif params", __func__);
                            pme->mCameraHandle->ops->close_camera(
                                pme->mCameraHandle->camera_handle);
                            pme->mCameraHandle = NULL;
                            break;
                        }
                        memset(pme->mExifParams.debug_params, 0,
                                sizeof(mm_jpeg_debug_exif_params_t));
                    }
                    break;
                case CMD_DEF_GENERIC:
                    {
                        BackgroundTask *bgTask = dw->args.genericArgs;
                        bgTask->bgFunction(bgTask->bgArgs);
                    }
                    break;
                default:
                    ALOGE("%s[%d]:  Incorrect command : %d",
                            __func__,
                            __LINE__,
                            dw->cmd);
                }

                pme->dequeueDeferredWork(dw);
            }
            break;
        case CAMERA_CMD_TYPE_EXIT:
            running = 0;
            break;
        default:
            break;
        }
    } while (running);

    return NULL;
}

/*===========================================================================
 * FUNCTION   : queueDeferredWork
 *
 * DESCRIPTION: function which queues deferred tasks
 *
 * PARAMETERS :
 *   @cmd     : deferred task
 *   @args    : deferred task arguments
 *
 * RETURN     : job id of deferred job
 *            : 0 in case of error
 *==========================================================================*/
uint32_t QCamera2HardwareInterface::queueDeferredWork(DeferredWorkCmd cmd,
                                                      DeferWorkArgs args)
{
    Mutex::Autolock l(mDefLock);
    for (uint32_t i = 0; i < MAX_ONGOING_JOBS; ++i) {
        if (mDefOngoingJobs[i] == 0) {
            DefWork *dw = new DefWork(cmd, sNextJobId, args);
            if (mCmdQueue.enqueue(dw)) {
                mDefOngoingJobs[i] = sNextJobId++;
                if (sNextJobId == 0) { // handle overflow
                    sNextJobId = 1;
                }
                mDeferredWorkThread.sendCmd(CAMERA_CMD_TYPE_DO_NEXT_JOB,
                        FALSE,
                        FALSE);
                return mDefOngoingJobs[i];
            } else {
                CDBG("%s: Command queue not active! cmd = %d", __func__, cmd);
                delete dw;
                return 0;
            }
        }
    }
    return 0;
}

/*===========================================================================
 * FUNCTION   : initJpegHandle
 *
 * DESCRIPTION: Opens JPEG client and gets a handle.
 *                     Sends Dual cam calibration info if present
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::initJpegHandle() {
    // Check if JPEG client handle is present
    CDBG_HIGH("%s: E", __func__);
    if(!mJpegClientHandle) {
        mm_dimension max_size = {0, 0};
        //set max pic size
        max_size.w = m_max_pic_width;
        max_size.h = m_max_pic_height;
        if (getRelatedCamSyncInfo()->sync_control == CAM_SYNC_RELATED_SENSORS_ON) {
            if (m_bRelCamCalibValid) {
                mJpegClientHandle = jpeg_open(&mJpegHandle, &mJpegMpoHandle,
                        max_size, &mJpegMetadata);
            } else {
                mJpegClientHandle =  jpeg_open(&mJpegHandle, &mJpegMpoHandle,
                        max_size, NULL);
            }
        } else {
            mJpegClientHandle = jpeg_open(&mJpegHandle, NULL, max_size, NULL);
        }
        if (!mJpegClientHandle) {
            ALOGE("%s: Error !! jpeg_open failed!! ", __func__);
            return UNKNOWN_ERROR;
        }
        // Set JPEG initialized as true to signify that this camera
        // has initialized the handle
        mJpegHandleOwner = true;
    }
    CDBG_HIGH("%s: X mJpegHandleOwner: %d, mJpegClientHandle: %d camera id: %d",
            __func__, mJpegHandleOwner, mJpegClientHandle, mCameraId);
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : deinitJpegHandle
 *
 * DESCRIPTION: Closes JPEG client using handle
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::deinitJpegHandle() {
    int32_t rc = NO_ERROR;
    CDBG_HIGH("%s: E", __func__);
    // Check if JPEG client handle is present and inited by this camera
    if(mJpegHandleOwner && mJpegClientHandle) {
        rc = mJpegHandle.close(mJpegClientHandle);
        if (rc != NO_ERROR) {
            ALOGE("%s: Error!! Closing mJpegClientHandle: %d failed",
                    __func__, mJpegClientHandle);
        }
        memset(&mJpegHandle, 0, sizeof(mJpegHandle));
        memset(&mJpegMpoHandle, 0, sizeof(mJpegMpoHandle));
        mJpegHandleOwner = false;
    }
    mJpegClientHandle = 0;
    CDBG_HIGH("%s: X rc = %d", __func__, rc);
    return rc;
}

/*===========================================================================
 * FUNCTION   : setJpegHandleInfo
 *
 * DESCRIPTION: sets JPEG client handle info
 *
 * PARAMETERS:
 *                  @ops                    : JPEG ops
 *                  @mpo_ops             : Jpeg MPO ops
 *                  @pJpegClientHandle : o/p Jpeg Client Handle
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::setJpegHandleInfo(mm_jpeg_ops_t *ops,
        mm_jpeg_mpo_ops_t *mpo_ops, uint32_t pJpegClientHandle) {

    if (pJpegClientHandle && ops && mpo_ops) {
        CDBG_HIGH("%s: Setting JPEG client handle %d", __func__,
                pJpegClientHandle);
        memcpy(&mJpegHandle, ops, sizeof(mm_jpeg_ops_t));
        memcpy(&mJpegMpoHandle, mpo_ops, sizeof(mm_jpeg_mpo_ops_t));
        mJpegClientHandle = pJpegClientHandle;
        return NO_ERROR;
    }
    else {
        CDBG_HIGH("%s: Error!! No Handle found: %d", __func__,
                pJpegClientHandle);
        return BAD_VALUE;
    }
}

/*===========================================================================
 * FUNCTION   : getJpegHandleInfo
 *
 * DESCRIPTION: gets JPEG client handle info
 *
 * PARAMETERS:
 *                  @ops                    : JPEG ops
 *                  @mpo_ops             : Jpeg MPO ops
 *                  @pJpegClientHandle : o/p Jpeg Client Handle
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::getJpegHandleInfo(mm_jpeg_ops_t *ops,
        mm_jpeg_mpo_ops_t *mpo_ops, uint32_t *pJpegClientHandle) {
    waitDeferredWork(mInitPProcJob);

    // Copy JPEG ops if present
    if (ops && mpo_ops && pJpegClientHandle) {
        memcpy(ops, &mJpegHandle, sizeof(mm_jpeg_ops_t));
        memcpy(mpo_ops, &mJpegMpoHandle, sizeof(mm_jpeg_mpo_ops_t));
        *pJpegClientHandle = mJpegClientHandle;
        CDBG_HIGH("%s: Getting JPEG client handle %d", __func__,
                pJpegClientHandle);
        return NO_ERROR;
    } else {
        return BAD_VALUE;
    }
}

/*===========================================================================
 * FUNCTION   : queueDeferredWork
 *
 * DESCRIPTION: function which dequeues deferred tasks
 *
 * PARAMETERS :
 *   @dw      : deferred work
 *   @args    : deferred task arguments
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
uint32_t QCamera2HardwareInterface::dequeueDeferredWork(DefWork* dw)
{
    Mutex::Autolock l(mDefLock);
    for (uint32_t i = 0; i < MAX_ONGOING_JOBS; i++) {
        if (mDefOngoingJobs[i] == dw->id) {
            mDefOngoingJobs[i] = 0;
            delete dw;
            mDefCond.broadcast();
            return NO_ERROR;
        }
    }

    return UNKNOWN_ERROR;
}

/*===========================================================================
 * FUNCTION   : checkDeferredWork
 *
 * DESCRIPTION: checks if a deferred task is in progress
 *
 * PARAMETERS :
 *   @job_id  : deferred task id
 *
 * RETURN     : true if the task exists, otherwise false
 *
 * PRECONDITION : mDefLock is held by current thread
 *==========================================================================*/
bool QCamera2HardwareInterface::checkDeferredWork(uint32_t &job_id)
{
    for (uint32_t i = 0; i < MAX_ONGOING_JOBS; i++) {
        if (mDefOngoingJobs[i] == job_id) {
            return true;
        }
    }

    return false;
}

/*===========================================================================
 * FUNCTION   : waitDeferredWork
 *
 * DESCRIPTION: waits for a deferred task to finish
 *
 * PARAMETERS :
 *   @job_id  : deferred task id
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::waitDeferredWork(uint32_t &job_id)
{
    int32_t rc = NO_ERROR;
    Mutex::Autolock l(mDefLock);

    if (job_id == 0) {
        ALOGE("%s: Invalid job id %d", __func__, job_id);
        return NO_ERROR;
    }

    while ( checkDeferredWork(job_id) == true ) {
        mDefCond.waitRelative(mDefLock, CAMERA_DEFERRED_THREAD_TIMEOUT);
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : scheduleBackgroundTask
 *
 * DESCRIPTION: Run a requested task in the deferred thread
 *
 * PARAMETERS :
 *   @bgTask  : Task to perform in the background
 *
 * RETURN     : job id of deferred job
 *            : 0 in case of error
 *==========================================================================*/
uint32_t QCamera2HardwareInterface::scheduleBackgroundTask(BackgroundTask* bgTask)
{
    DeferWorkArgs args;
    memset(&args, 0, sizeof(DeferWorkArgs));
    args.genericArgs = bgTask;

    return queueDeferredWork(CMD_DEF_GENERIC, args);
}

/*===========================================================================
 * FUNCTION   : waitForBackgroundTask
 *
 * DESCRIPTION: Wait for a background task to complete
 *
 * PARAMETERS :
 *   @taskId  : Task id to wait for
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera2HardwareInterface::waitForBackgroundTask(uint32_t& taskId)
{
    return waitDeferredWork(taskId);
}

/*===========================================================================
 * FUNCTION   : needDeferedAllocation
 *
 * DESCRIPTION: Function to decide background task for streams
 *
 * PARAMETERS :
 *   @stream_type  : stream type
 *
 * RETURN     : true - if background task is needed
 *              false -  if background task is NOT needed
 *==========================================================================*/
bool QCamera2HardwareInterface::needDeferred(cam_stream_type_t stream_type)
{
    if ((stream_type == CAM_STREAM_TYPE_PREVIEW && mPreviewWindow == NULL)
            || (stream_type == CAM_STREAM_TYPE_ANALYSIS)) {
        return FALSE;
    }

    if ((stream_type == CAM_STREAM_TYPE_PREVIEW)
            || (stream_type == CAM_STREAM_TYPE_METADATA)
            || (stream_type == CAM_STREAM_TYPE_RAW)
            || (stream_type == CAM_STREAM_TYPE_POSTVIEW)) {
        return TRUE;
    }

    if ((stream_type == CAM_STREAM_TYPE_SNAPSHOT)
            && (!mParameters.getRecordingHintValue())){
        return TRUE;
    }

    if (stream_type == CAM_STREAM_TYPE_VIDEO) {
        return FALSE;
    }
    return FALSE;
}

/*===========================================================================
 * FUNCTION   : isRegularCapture
 *
 * DESCRIPTION: Check configuration for regular catpure
 *
 * PARAMETERS :
 *
 * RETURN     : true - regular capture
 *              false - other type of capture
 *==========================================================================*/
bool QCamera2HardwareInterface::isRegularCapture()
{
    bool ret = false;

    if (numOfSnapshotsExpected() == 1 &&
        !isLongshotEnabled() &&
        !mParameters.isHDREnabled() &&
        !mParameters.getRecordingHintValue() &&
        !isZSLMode() && !mParameters.getofflineRAW()) {
            ret = true;
    }
    return ret;
}

/*===========================================================================
 * FUNCTION   : getLogLevel
 *
 * DESCRIPTION: Reads the log level property into a variable
 *
 * PARAMETERS :
 *   None
 *
 * RETURN     :
 *   None
 *==========================================================================*/
void QCamera2HardwareInterface::getLogLevel()
{
    char prop[PROPERTY_VALUE_MAX];
    uint32_t globalLogLevel = 0;

    property_get("persist.camera.hal.debug", prop, "0");
    int val = atoi(prop);
    if (0 <= val) {
        gCamHalLogLevel = (uint32_t)val;
    }
    property_get("persist.camera.global.debug", prop, "0");
    val = atoi(prop);
    if (0 <= val) {
        globalLogLevel = (uint32_t)val;
    }

    property_get("persist.camera.kpi.debug", prop, "1");
    gKpiDebugLevel = atoi(prop);

    /* Highest log level among hal.logs and global.logs is selected */
    if (gCamHalLogLevel < globalLogLevel)
        gCamHalLogLevel = globalLogLevel;

    return;
}

/*===========================================================================
 * FUNCTION   : getSensorType
 *
 * DESCRIPTION: Returns the type of sensor being used whether YUV or Bayer
 *
 * PARAMETERS :
 *   None
 *
 * RETURN     : Type of sensor - bayer or YUV
 *
 *==========================================================================*/
cam_sensor_t QCamera2HardwareInterface::getSensorType()
{
    return gCamCapability[mCameraId]->sensor_type.sens_type;
}

/*===========================================================================
 * FUNCTION   : isLowPowerMode
 *
 * DESCRIPTION: Returns TRUE if low power mode settings are to be applied for video recording
 *
 * PARAMETERS :
 *   None
 *
 * RETURN     : TRUE/FALSE
 *
 *==========================================================================*/
bool QCamera2HardwareInterface::isLowPowerMode()
{
    cam_dimension_t dim;
    mParameters.getStreamDimension(CAM_STREAM_TYPE_VIDEO, dim);

    char prop[PROPERTY_VALUE_MAX];
    property_get("camera.lowpower.record.enable", prop, "0");
    int enable = atoi(prop);

    //Enable low power mode if :
    //1. Video resolution is 2k (2048x1080) or above and
    //2. camera.lowpower.record.enable is set

    bool isLowpower = mParameters.getRecordingHintValue() && enable
            && ((dim.width * dim.height) >= (2048 * 1080));
    return isLowpower;
}

}; // namespace qcamera
