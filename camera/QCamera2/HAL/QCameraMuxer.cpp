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

#define ALOG_NIDEBUG 0
#define LOG_TAG "QCameraMuxer"
#include <utils/Log.h>
#include <utils/threads.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <binder/IMemory.h>
#include <binder/MemoryBase.h>
#include <binder/MemoryHeapBase.h>
#include <utils/RefBase.h>

#include "QCameraMuxer.h"
#include "QCamera2HWI.h"
#include "QCameraPostProc.h"

#include <sys/stat.h>
#include <utils/Errors.h>
#include <utils/Trace.h>
#include <utils/Timers.h>

/* Muxer implementation */

using namespace android;
namespace qcamera {

QCameraMuxer *gMuxer = NULL;

//Error Check Macros
#define CHECK_MUXER() \
    if (!gMuxer) { \
        ALOGE("%s[%d]: Error getting muxer ", __func__, __LINE__); \
        return; \
    } \

#define CHECK_MUXER_ERROR() \
    if (!gMuxer) { \
        ALOGE("%s[%d]: Error getting muxer ", __func__, __LINE__); \
        return -ENODEV; \
    } \

#define CHECK_CAMERA(pCam) \
    if (!pCam) { \
        ALOGE("%s[%d]: Error getting physical camera", __func__, __LINE__); \
        return; \
    } \

#define CHECK_CAMERA_ERROR(pCam) \
    if (!pCam) { \
        ALOGE("%s[%d]: Error getting physical camera", __func__, __LINE__); \
        return -ENODEV; \
    } \

#define CHECK_HWI(hwi) \
    if (!hwi) { \
        ALOGE("%s[%d]: Error !! HWI not found!!", __func__, __LINE__); \
        return; \
    } \

#define CHECK_HWI_ERROR(hwi) \
    if (!hwi) { \
        ALOGE("%s[%d]: Error !! HWI not found!!", __func__, __LINE__); \
        return -ENODEV; \
    } \


/*===========================================================================
 * FUNCTION         : getCameraMuxer
 *
 * DESCRIPTION     : Creates Camera Muxer if not created
 *
 * PARAMETERS:
 *   @pMuxer               : Pointer to retrieve Camera Muxer
 *   @num_of_cameras  : Number of Physical Cameras on device
 *
 * RETURN             :  NONE
 *==========================================================================*/
void QCameraMuxer::getCameraMuxer(
        QCameraMuxer** pMuxer, uint32_t num_of_cameras)
{
    *pMuxer = NULL;
    if (!gMuxer) {
        gMuxer = new QCameraMuxer(num_of_cameras);
    }
    CHECK_MUXER();
    *pMuxer = gMuxer;
    CDBG_HIGH("%s[%d]: gMuxer: %p ", __func__, __LINE__, gMuxer);
    return;
}

/*===========================================================================
 * FUNCTION         : QCameraMuxer
 *
 * DESCRIPTION     : QCameraMuxer Constructor
 *
 * PARAMETERS:
 *   @num_of_cameras  : Number of Physical Cameras on device
 *
 *==========================================================================*/
QCameraMuxer::QCameraMuxer(uint32_t num_of_cameras)
    : mJpegClientHandle(0),
      m_pPhyCamera(NULL),
      m_pLogicalCamera(NULL),
      m_pCallbacks(NULL),
      m_bDualCameraEnabled(FALSE),
      m_bAuxCameraExposed(FALSE),
      m_nPhyCameras(num_of_cameras),
      m_nLogicalCameras(0),
      m_MainJpegQ(releaseJpegInfo, this),
      m_AuxJpegQ(releaseJpegInfo, this),
      m_pRelCamMpoJpeg(NULL),
      m_pMpoCallbackCookie(NULL),
      m_pJpegCallbackCookie(NULL),
      m_bDumpImages(FALSE),
      m_bMpoEnabled(TRUE),
      m_bFrameSyncEnabled(FALSE),
      m_bRecordingHintInternallySet(FALSE)
{
    setupLogicalCameras();
    memset(&mJpegOps, 0, sizeof(mJpegOps));
    memset(&mJpegMpoOps, 0, sizeof(mJpegMpoOps));
    memset(&mGetMemoryCb, 0, sizeof(mGetMemoryCb));
    memset(&mDataCb, 0, sizeof(mDataCb));

    // initialize mutex for MPO composition
    pthread_mutex_init(&m_JpegLock, NULL);
    // launch MPO composition thread
    m_ComposeMpoTh.launch(composeMpoRoutine, this);

    //Check whether dual camera images need to be dumped
    char prop[PROPERTY_VALUE_MAX];
    property_get("persist.camera.dual.camera.dump", prop, "0");
    m_bDumpImages = atoi(prop);
    CDBG_HIGH("%s: dualCamera dump images:%d ", __func__, m_bDumpImages);
}

/*===========================================================================
 * FUNCTION         : ~QCameraMuxer
 *
 * DESCRIPTION     : QCameraMuxer Desctructor
 *
 *==========================================================================*/
QCameraMuxer::~QCameraMuxer() {
    if (m_pLogicalCamera) {
        delete [] m_pLogicalCamera;
        m_pLogicalCamera = NULL;
    }
    if (m_pPhyCamera) {
        delete [] m_pPhyCamera;
        m_pPhyCamera = NULL;
    }

    if (NULL != m_pRelCamMpoJpeg) {
        m_pRelCamMpoJpeg->release(m_pRelCamMpoJpeg);
        m_pRelCamMpoJpeg = NULL;
    }
    // flush Jpeg Queues
    m_MainJpegQ.flush();
    m_AuxJpegQ.flush();

    // stop and exit MPO composition thread
    m_ComposeMpoTh.sendCmd(CAMERA_CMD_TYPE_STOP_DATA_PROC, TRUE, FALSE);
    m_ComposeMpoTh.exit();

    pthread_mutex_destroy(&m_JpegLock);
}

/*===========================================================================
 * FUNCTION         : get_number_of_cameras
 *
 * DESCRIPTION     : Provide number of Logical Cameras
 *
 * RETURN             :  Number of logical Cameras
 *==========================================================================*/
int QCameraMuxer::get_number_of_cameras()
{
    return gMuxer->getNumberOfCameras();
}

/*===========================================================================
 * FUNCTION         : get_camera_info
 *
 * DESCRIPTION     : get logical camera info
 *
 * PARAMETERS:
 *   @camera_id     : Logical Camera ID
 *   @info              : Logical Main Camera Info
 *
 * RETURN     :
 *              NO_ERROR  : success
 *              ENODEV : Camera not found
 *              other: non-zero failure code
 *==========================================================================*/
int QCameraMuxer::get_camera_info(int camera_id, struct camera_info *info)
{
    int rc = NO_ERROR;
    CDBG_HIGH("%s: E", __func__);
    cam_sync_type_t type;
    if ((camera_id < 0) || (camera_id >= gMuxer->getNumberOfCameras())) {
        ALOGE("%s : Camera id %d not found!", __func__, camera_id);
        return -ENODEV;
    }
    if(info) {
        rc = gMuxer->getCameraInfo(camera_id, info, &type);
    }
    CDBG_HIGH("%s: X, rc: %d", __func__, rc);
    return rc;
}


/*===========================================================================
 * FUNCTION         : set_callbacks
 *
 * DESCRIPTION     : Not Implemented
 *
 * PARAMETERS:
 *   @callbacks      : Camera Module Callbacks
 *
 * RETURN     :
 *              NO_ERROR  : success
 *              other: non-zero failure code
 *==========================================================================*/
int QCameraMuxer::set_callbacks(const camera_module_callbacks_t *callbacks)
{
    // Not implemented
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : camera_device_open
 *
 * DESCRIPTION: static function to open a camera device by its ID
 *
 * PARAMETERS :
 *   @modue: hw module
 *   @id : camera ID
 *   @hw_device : ptr to struct storing camera hardware device info
 *
 * RETURN     :
 *              NO_ERROR  : success
 *              BAD_VALUE : Invalid Camera ID
 *              other: non-zero failure code
 *==========================================================================*/
int QCameraMuxer::camera_device_open(
        const struct hw_module_t *module, const char *id,
        struct hw_device_t **hw_device)
{
    int rc = NO_ERROR;
    CDBG_HIGH("%s[%d]: id= %d", __func__, __LINE__, atoi(id));
    if (!id) {
        ALOGE("%s: Invalid camera id", __func__);
        return BAD_VALUE;
    }

    rc =  gMuxer->cameraDeviceOpen(atoi(id), hw_device);
    CDBG_HIGH("%s[%d]: id= %d, rc: %d", __func__, __LINE__, atoi(id), rc);
    return rc;
}

/*===========================================================================
 * FUNCTION   : open_legacy
 *
 * DESCRIPTION: static function to open a camera device by its ID
 *
 * PARAMETERS :
 *   @modue: hw module
 *   @id : camera ID
 *   @halVersion: hal version
 *   @hw_device : ptr to struct storing camera hardware device info
 *
 * RETURN     :
 *              NO_ERROR  : success
 *              BAD_VALUE : Invalid Camera ID
 *              other: non-zero failure code
 *==========================================================================*/
int QCameraMuxer::open_legacy(const struct hw_module_t* module,
        const char* id, uint32_t halVersion, struct hw_device_t** hw_device)
{
    int rc = NO_ERROR;
    CDBG_HIGH("%s[%d]: id= %d", __func__, __LINE__, atoi(id));
    if (!id) {
        ALOGE("%s: Invalid camera id", __func__);
        return BAD_VALUE;
    }

    rc =  gMuxer->cameraDeviceOpen(atoi(id), hw_device);
    CDBG_HIGH("%s[%d]: id= %d, rc: %d", __func__, __LINE__, atoi(id), rc);
    return rc;
}

/*===========================================================================
 * FUNCTION   : set_preview_window
 *
 * DESCRIPTION: Set Preview window for main camera
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *   @window: Preview window ops
 *
 * RETURN     :
 *              NO_ERROR  : success
 *              other: non-zero failure code
 *==========================================================================*/
int QCameraMuxer::set_preview_window(struct camera_device * device,
        struct preview_stream_ops *window)
{
    int rc = NO_ERROR;
    CHECK_MUXER_ERROR();
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA_ERROR(cam);

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);
        rc = hwi->set_preview_window(pCam->dev, window);
        if (rc != NO_ERROR) {
            ALOGE("%s: Error!! setting preview window", __func__);
            return rc;
        }
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : set_callBacks
 *
 * DESCRIPTION: Set Framework callbacks to notify various frame data asynchronously
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *   @notify_cb: Notification callback
 *   @data_cb: data callback
 *   @data_cb_timestamp: data timestamp callback
 *   @get_memory: callback to obtain memory
 *   @user : userdata
 *
 * RETURN : None
 *==========================================================================*/
void QCameraMuxer::set_callBacks(struct camera_device * device,
        camera_notify_callback notify_cb,
        camera_data_callback data_cb,
        camera_data_timestamp_callback data_cb_timestamp,
        camera_request_memory get_memory,
        void *user)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER();
    int rc = NO_ERROR;
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA(cam);

    // Set callbacks to HWI
    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI(hwi);

        hwi->set_CallBacks(pCam->dev, notify_cb, data_cb, data_cb_timestamp,
                get_memory, user);

        // Set JPG callbacks
        // sending the physical camera description with the Jpeg callback
        // this will be retrieved in callbacks to get the cam instance
        // delivering JPEGs
        hwi->setJpegCallBacks(jpeg_data_callback, (void*)pCam);

        if (pCam->mode == CAM_MODE_PRIMARY) {
            rc = gMuxer->setMainJpegCallbackCookie((void*)(pCam));
            if(rc != NO_ERROR) {
                ALOGE("%s: Error setting Jpeg callback cookie", __func__);
            }
        }
    }
    // Store callback in Muxer to send data callbacks
    rc = gMuxer->setDataCallback(data_cb);
    if(rc != NO_ERROR) {
        ALOGE("%s: Error setting data callback", __func__);
    }
    // memory callback stored to allocate memory for MPO buffer
    rc = gMuxer->setMemoryCallback(get_memory);
    if(rc != NO_ERROR) {
        ALOGE("%s: Error setting memory callback", __func__);
    }
    // actual user callback cookie is saved in Muxer
    // this will be used to deliver final MPO callback to the framework
    rc = gMuxer->setMpoCallbackCookie(user);
    if(rc != NO_ERROR) {
        ALOGE("%s: Error setting mpo cookie", __func__);
    }

    CDBG_HIGH("%s: X", __func__);

}

/*===========================================================================
 * FUNCTION   : enable_msg_type
 *
 * DESCRIPTION: Enable msg_type to send callbacks
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *   @msg_type: callback Message type to be enabled
 *
 * RETURN : None
 *==========================================================================*/
void QCameraMuxer::enable_msg_type(struct camera_device * device, int32_t msg_type)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER();
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA(cam);

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA(pCam);
        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI(hwi);
        hwi->enable_msg_type(pCam->dev, msg_type);
    }
    CDBG_HIGH("%s: X", __func__);
}

/*===========================================================================
 * FUNCTION   : disable_msg_type
 *
 * DESCRIPTION: disable msg_type to send callbacks
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *   @msg_type: callback Message type to be disabled
 *
 * RETURN : None
 *==========================================================================*/
void QCameraMuxer::disable_msg_type(struct camera_device * device, int32_t msg_type)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER();
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA(cam);

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA(pCam);
        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI(hwi);
        hwi->disable_msg_type(pCam->dev, msg_type);
    }
    CDBG_HIGH("%s: X", __func__);
}

/*===========================================================================
 * FUNCTION   : msg_type_enabled
 *
 * DESCRIPTION: Check if message type enabled
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *   @msg_type: message type
 *
 * RETURN : true/false
 *==========================================================================*/
int QCameraMuxer::msg_type_enabled(struct camera_device * device, int32_t msg_type)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER_ERROR();
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA_ERROR(cam);

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);

        if (pCam->mode == CAM_MODE_PRIMARY) {
            return hwi->msg_type_enabled(pCam->dev, msg_type);
        }
    }
    CDBG_HIGH("%s: X", __func__);
    return false;
}

/*===========================================================================
 * FUNCTION   : start_preview
 *
 * DESCRIPTION: Starts logical camera preview
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *
 * RETURN     :
 *              NO_ERROR  : success
 *              other: non-zero failure code
 *==========================================================================*/
int QCameraMuxer::start_preview(struct camera_device * device)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER_ERROR();
    int rc = NO_ERROR;
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA_ERROR(cam);

    // prepare preview first for all cameras
    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);

        rc = hwi->prepare_preview(pCam->dev);
        if (rc != NO_ERROR) {
            ALOGE("%s: Error preparing preview !! ", __func__);
            return rc;
        }
    }

    if (cam->numCameras > 1) {
        uint sessionId = 0;
        // Set up sync for camera sessions
        for (uint32_t i = 0; i < cam->numCameras; i++) {
            pCam = gMuxer->getPhysicalCamera(cam, i);
            CHECK_CAMERA_ERROR(pCam);

            QCamera2HardwareInterface *hwi = pCam->hwi;
            CHECK_HWI_ERROR(hwi);

            if(pCam->mode == CAM_MODE_PRIMARY) {
                // bundle primary cam with all aux cameras
                for (uint32_t j = 0; j < cam->numCameras; j++) {
                    if (j == cam->nPrimaryPhyCamIndex) {
                        continue;
                    }
                    sessionId = cam->sId[j];
                    CDBG_HIGH("%s: Related cam id: %d, server id: %d sync ON"
                            " related session_id %d", __func__,
                            cam->pId[i], cam->sId[i], sessionId);
                    rc = hwi->bundleRelatedCameras(true, sessionId);
                    if (rc != NO_ERROR) {
                        ALOGE("%s: Error Bundling physical cameras !! ", __func__);
                        return rc;
                    }
                }
            }

            if (pCam->mode == CAM_MODE_SECONDARY) {
                // bundle all aux cam with primary cams
                sessionId = cam->sId[cam->nPrimaryPhyCamIndex];
                CDBG_HIGH("%s: Related cam id: %d, server id: %d sync ON"
                        " related session_id %d", __func__,
                        cam->pId[i], cam->sId[i], sessionId);
                rc = hwi->bundleRelatedCameras(true, sessionId);
                if (rc != NO_ERROR) {
                    ALOGE("%s: Error Bundling physical cameras !! ", __func__);
                    return rc;
                }
            }
        }

        // Remember Sync is ON
        cam->bSyncOn = true;
    }
    // Start Preview for all cameras
    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);
        rc = hwi->start_preview(pCam->dev);
        if (rc != NO_ERROR) {
            ALOGE("%s: Error starting preview !! ", __func__);
            return rc;
        }
    }
    CDBG_HIGH("%s: X", __func__);
    return rc;
}

/*===========================================================================
 * FUNCTION   : stop_preview
 *
 * DESCRIPTION: Stops logical camera preview
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *
 * RETURN     : None
 *==========================================================================*/
void QCameraMuxer::stop_preview(struct camera_device * device)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER();
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA(cam);

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI(hwi);

        QCamera2HardwareInterface::stop_preview(pCam->dev);
    }

    //Flush JPEG Queues. Nodes in Main and Aux JPEGQ are not valid after preview stopped.
    gMuxer->m_MainJpegQ.flush();
    gMuxer->m_AuxJpegQ.flush();

    CDBG_HIGH("%s: X", __func__);
}

/*===========================================================================
 * FUNCTION   : preview_enabled
 *
 * DESCRIPTION: Checks preview enabled
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *
 * RETURN     : true/false
 *==========================================================================*/
int QCameraMuxer::preview_enabled(struct camera_device * device)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER_ERROR();
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA_ERROR(cam);

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);

        if (pCam->mode == CAM_MODE_PRIMARY) {
            return hwi->preview_enabled(pCam->dev);
        }
    }
    CDBG_HIGH("%s: X", __func__);
    return false;
}

/*===========================================================================
 * FUNCTION   : store_meta_data_in_buffers
 *
 * DESCRIPTION: Stores metadata in buffers
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *   @enable: Enable/disable metadata
 *
 * RETURN     :
 *              NO_ERROR  : success
 *              other: non-zero failure code
 *==========================================================================*/
int QCameraMuxer::store_meta_data_in_buffers(struct camera_device * device, int enable)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER_ERROR();
    int rc = NO_ERROR;
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA_ERROR(cam);

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);

        rc = hwi->store_meta_data_in_buffers(pCam->dev, enable);
        if (rc != NO_ERROR) {
            ALOGE("%s: Error storing metat data !! ", __func__);
            return rc;
        }
    }
    CDBG_HIGH("%s: X", __func__);
    return rc;
}

/*===========================================================================
 * FUNCTION   : start_recording
 *
 * DESCRIPTION: Starts recording on camcorder
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *
 * RETURN     :
 *              NO_ERROR  : success
 *              other: non-zero failure code
 *==========================================================================*/
int QCameraMuxer::start_recording(struct camera_device * device)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER_ERROR();
    int rc = NO_ERROR;
    bool previewRestartNeeded = false;
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA_ERROR(cam);

    // In cases where recording hint is not set, hwi->start_recording will
    // internally restart the preview.
    // To take the preview restart control in muxer,
    // 1. call pre_start_recording first
    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);

        rc = hwi->pre_start_recording(pCam->dev);
        if (rc != NO_ERROR) {
            ALOGE("%s: Error preparing recording start!! ", __func__);
            return rc;
        }
    }

    // 2. Check if preview restart is needed. Check all cameras.
    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);

        if (hwi->isPreviewRestartNeeded()) {
            previewRestartNeeded = hwi->isPreviewRestartNeeded();
            break;
        }
    }

    if (previewRestartNeeded) {
        // 3. if preview restart needed. stop the preview first
        for (uint32_t i = 0; i < cam->numCameras; i++) {
            pCam = gMuxer->getPhysicalCamera(cam, i);
            CHECK_CAMERA_ERROR(pCam);

            QCamera2HardwareInterface *hwi = pCam->hwi;
            CHECK_HWI_ERROR(hwi);

            rc = hwi->restart_stop_preview(pCam->dev);
            if (rc != NO_ERROR) {
                ALOGE("%s: Error in restart stop preview!! ", __func__);
                return rc;
            }
        }

        //4. Update the recording hint value to TRUE
        for (uint32_t i = 0; i < cam->numCameras; i++) {
            pCam = gMuxer->getPhysicalCamera(cam, i);
            CHECK_CAMERA_ERROR(pCam);

            QCamera2HardwareInterface *hwi = pCam->hwi;
            CHECK_HWI_ERROR(hwi);

            rc = hwi->setRecordingHintValue(TRUE);
            if (rc != NO_ERROR) {
                ALOGE("%s: Error in setting recording hint value!! ", __func__);
                return rc;
            }
            gMuxer->m_bRecordingHintInternallySet = TRUE;
        }

        // 5. start the preview
        for (uint32_t i = 0; i < cam->numCameras; i++) {
            pCam = gMuxer->getPhysicalCamera(cam, i);
            CHECK_CAMERA_ERROR(pCam);

            QCamera2HardwareInterface *hwi = pCam->hwi;
            CHECK_HWI_ERROR(hwi);

            rc = hwi->restart_start_preview(pCam->dev);
            if (rc != NO_ERROR) {
                ALOGE("%s: Error in restart start preview!! ", __func__);
                return rc;
            }
        }
    }

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);

        if (pCam->mode == CAM_MODE_PRIMARY) {
            rc = hwi->start_recording(pCam->dev);
            if (rc != NO_ERROR) {
                ALOGE("%s: Error starting recording!! ", __func__);
            }
            break;
        }
    }
    CDBG_HIGH("%s: X", __func__);
    return rc;
}

/*===========================================================================
 * FUNCTION   : stop_recording
 *
 * DESCRIPTION: Stops recording on camcorder
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *
 * RETURN     : None
 *==========================================================================*/
void QCameraMuxer::stop_recording(struct camera_device * device)
{
    int rc = NO_ERROR;
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER();
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA(cam);

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI(hwi);

        if (pCam->mode == CAM_MODE_PRIMARY) {
            QCamera2HardwareInterface::stop_recording(pCam->dev);
            break;
        }
    }

    // If recording hint is set internally to TRUE,
    // we need to set it to FALSE.
    // preview restart is needed in between
    if (gMuxer->m_bRecordingHintInternallySet) {
        // stop the preview first
        for (uint32_t i = 0; i < cam->numCameras; i++) {
            pCam = gMuxer->getPhysicalCamera(cam, i);
            CHECK_CAMERA(pCam);

            QCamera2HardwareInterface *hwi = pCam->hwi;
            CHECK_HWI(hwi);

            rc = hwi->restart_stop_preview(pCam->dev);
            if (rc != NO_ERROR) {
                ALOGE("%s: Error in restart stop preview!! ", __func__);
                return;
            }
        }

        // Update the recording hint value to FALSE
        for (uint32_t i = 0; i < cam->numCameras; i++) {
            pCam = gMuxer->getPhysicalCamera(cam, i);
            CHECK_CAMERA(pCam);

            QCamera2HardwareInterface *hwi = pCam->hwi;
            CHECK_HWI(hwi);

            rc = hwi->setRecordingHintValue(FALSE);
            if (rc != NO_ERROR) {
                ALOGE("%s: Error in setting recording hint value!! ", __func__);
                return;
            }
            gMuxer->m_bRecordingHintInternallySet = FALSE;
        }

        // start the preview
        for (uint32_t i = 0; i < cam->numCameras; i++) {
            pCam = gMuxer->getPhysicalCamera(cam, i);
            CHECK_CAMERA(pCam);

            QCamera2HardwareInterface *hwi = pCam->hwi;
            CHECK_HWI(hwi);

            rc = hwi->restart_start_preview(pCam->dev);
            if (rc != NO_ERROR) {
                ALOGE("%s: Error in restart start preview!! ", __func__);
                return;
            }
        }
    }
    CDBG_HIGH("%s: X", __func__);
}

/*===========================================================================
 * FUNCTION   : recording_enabled
 *
 * DESCRIPTION: Checks for recording enabled
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *
 * RETURN     : true/false
 *==========================================================================*/
int QCameraMuxer::recording_enabled(struct camera_device * device)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER_ERROR();
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA_ERROR(cam);

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);

        if (pCam->mode == CAM_MODE_PRIMARY) {
            return hwi->recording_enabled(pCam->dev);
        }
    }
    CDBG_HIGH("%s: X", __func__);
    return false;
}

/*===========================================================================
 * FUNCTION   : release_recording_frame
 *
 * DESCRIPTION: Release the recording frame
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *   @opaque: Frame to be released
 *
  * RETURN     : None
 *==========================================================================*/
void QCameraMuxer::release_recording_frame(struct camera_device * device,
                const void *opaque)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER();
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA(cam);

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI(hwi);

        if (pCam->mode == CAM_MODE_PRIMARY) {
            QCamera2HardwareInterface::release_recording_frame(pCam->dev, opaque);
            break;
        }
    }
    CDBG_HIGH("%s: X", __func__);
}

/*===========================================================================
 * FUNCTION   : auto_focus
 *
 * DESCRIPTION: Performs auto focus on camera
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *
 * RETURN     :
 *              NO_ERROR  : success
 *              other: non-zero failure code
 *==========================================================================*/
int QCameraMuxer::auto_focus(struct camera_device * device)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER_ERROR();
    int rc = NO_ERROR;
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA_ERROR(cam);

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);
        // Call auto focus on main camera
        if (pCam->mode == CAM_MODE_PRIMARY) {
            rc = QCamera2HardwareInterface::auto_focus(pCam->dev);
            if (rc != NO_ERROR) {
                ALOGE("%s: Error auto focusing !! ", __func__);
                return rc;
            }
            break;
        }
    }
    CDBG_HIGH("%s: X", __func__);
    return rc;
}

/*===========================================================================
 * FUNCTION   : cancel_auto_focus
 *
 * DESCRIPTION: Cancels auto focus
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *
 * RETURN     :
 *              NO_ERROR  : success
 *              other: non-zero failure code
 *==========================================================================*/
int QCameraMuxer::cancel_auto_focus(struct camera_device * device)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER_ERROR();
    int rc = NO_ERROR;
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA_ERROR(cam);

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);
        // Cancel auto focus on primary camera
        if (pCam->mode == CAM_MODE_PRIMARY) {
            rc = QCamera2HardwareInterface::cancel_auto_focus(pCam->dev);
            if (rc != NO_ERROR) {
                ALOGE("%s: Error cancelling auto focus !! ", __func__);
                return rc;
            }
            break;
        }
    }
    CDBG_HIGH("%s: X", __func__);
    return rc;
}

/*===========================================================================
 * FUNCTION   : take_picture
 *
 * DESCRIPTION: Take snapshots on device
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *
 * RETURN     :
 *              NO_ERROR  : success
 *              other: non-zero failure code
 *==========================================================================*/
int QCameraMuxer::take_picture(struct camera_device * device)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER_ERROR();
    int rc = NO_ERROR;
    bool previewRestartNeeded = false;
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA_ERROR(cam);

    char prop[PROPERTY_VALUE_MAX];
    property_get("persist.camera.dual.camera.mpo", prop, "1");
    gMuxer->m_bMpoEnabled = atoi(prop);
    //If only one Physical Camera included in Logical, disable MPO
    int numOfAcitvePhyCam = 0;
    gMuxer->getActiveNumOfPhyCam(cam, numOfAcitvePhyCam);
    if (gMuxer->m_bMpoEnabled && numOfAcitvePhyCam <= 1) {
        gMuxer->m_bMpoEnabled = 0;
    }
    CDBG_HIGH("%s: dualCamera MPO Enabled:%d ", __func__, gMuxer->m_bMpoEnabled);

    if (!gMuxer->mJpegClientHandle) {
        // set up jpeg handles
        pCam = gMuxer->getPhysicalCamera(cam, 0);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);

        rc = hwi->getJpegHandleInfo(&gMuxer->mJpegOps, &gMuxer->mJpegMpoOps,
                &gMuxer->mJpegClientHandle);
        if (rc != NO_ERROR) {
            ALOGE("%s: Error retrieving jpeg handle!", __func__);
            return rc;
        }

        for (uint32_t i = 1; i < cam->numCameras; i++) {
            pCam = gMuxer->getPhysicalCamera(cam, i);
            CHECK_CAMERA_ERROR(pCam);

            QCamera2HardwareInterface *hwi = pCam->hwi;
            CHECK_HWI_ERROR(hwi);

            rc = hwi->setJpegHandleInfo(&gMuxer->mJpegOps, &gMuxer->mJpegMpoOps,
                    gMuxer->mJpegClientHandle);
            if (rc != NO_ERROR) {
                ALOGE("%s: Error setting jpeg handle %d!", __func__, i);
                return rc;
            }
        }
    }

    // prepare snapshot for main camera
    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);

        if (pCam->mode == CAM_MODE_PRIMARY) {
            rc = hwi->prepare_snapshot(pCam->dev);
            if (rc != NO_ERROR) {
                ALOGE("%s: Error preparing for snapshot !! ", __func__);
                return rc;
            }
        }
        // set Mpo composition for each session
        rc = hwi->setMpoComposition(gMuxer->m_bMpoEnabled);
        //disable MPO if AOST features are enabled
        if (rc != NO_ERROR) {
            gMuxer->m_bMpoEnabled = 0;
            rc = NO_ERROR;
        }
    }

    // initialize Jpeg Queues
    gMuxer->m_MainJpegQ.init();
    gMuxer->m_AuxJpegQ.init();
    gMuxer->m_ComposeMpoTh.sendCmd(
            CAMERA_CMD_TYPE_START_DATA_PROC, FALSE, FALSE);

    // In cases where recording hint is set, preview is running,
    // hwi->take_picture will internally restart the preview.
    // To take the preview restart control in muxer,
    // 1. call pre_take_picture first
    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);

        // no need to call pre_take_pic on Aux if not MPO (for AOST,liveshot...etc.)
        if ( (gMuxer->m_bMpoEnabled == 1) || (pCam->mode == CAM_MODE_PRIMARY) ) {
            rc = hwi->pre_take_picture(pCam->dev);
            if (rc != NO_ERROR) {
                ALOGE("%s: Error preparing take_picture!! ", __func__);
                return rc;
            }
        }
    }

    // 2. Check if preview restart is needed. Check all cameras.
    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);

        if (hwi->isPreviewRestartNeeded()) {
            previewRestartNeeded = hwi->isPreviewRestartNeeded();
            break;
        }
    }

    if (previewRestartNeeded) {
        // 3. if preview restart needed. stop the preview first
        for (uint32_t i = 0; i < cam->numCameras; i++) {
            pCam = gMuxer->getPhysicalCamera(cam, i);
            CHECK_CAMERA_ERROR(pCam);

            QCamera2HardwareInterface *hwi = pCam->hwi;
            CHECK_HWI_ERROR(hwi);

            rc = hwi->restart_stop_preview(pCam->dev);
            if (rc != NO_ERROR) {
                ALOGE("%s: Error in restart stop preview!! ", __func__);
                return rc;
            }
        }

        //4. Update the recording hint value to FALSE
        for (uint32_t i = 0; i < cam->numCameras; i++) {
            pCam = gMuxer->getPhysicalCamera(cam, i);
            CHECK_CAMERA_ERROR(pCam);

            QCamera2HardwareInterface *hwi = pCam->hwi;
            CHECK_HWI_ERROR(hwi);

            rc = hwi->setRecordingHintValue(FALSE);
            if (rc != NO_ERROR) {
                ALOGE("%s: Error in setting recording hint value!! ", __func__);
                return rc;
            }
        }

        // 5. start the preview
        for (uint32_t i = 0; i < cam->numCameras; i++) {
            pCam = gMuxer->getPhysicalCamera(cam, i);
            CHECK_CAMERA_ERROR(pCam);

            QCamera2HardwareInterface *hwi = pCam->hwi;
            CHECK_HWI_ERROR(hwi);

            rc = hwi->restart_start_preview(pCam->dev);
            if (rc != NO_ERROR) {
                ALOGE("%s: Error in restart start preview!! ", __func__);
                return rc;
            }
        }
    }

    // As frame sync for dual cameras is enabled, the take picture call
    // for secondary camera is handled only till HAL level to init corresponding
    // pproc channel and update statemachine.
    // This call is forwarded to mm-camera-intf only for primary camera
    // Primary camera should receive the take picture call after all secondary
    // camera statemachines are updated
    for (int32_t i = cam->numCameras-1 ; i >= 0; i--) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);

        // no need to call take_pic on Aux if not MPO (for AOST)
        if ( (gMuxer->m_bMpoEnabled == 1) || (pCam->mode == CAM_MODE_PRIMARY) ) {
            rc = QCamera2HardwareInterface::take_picture(pCam->dev);
            if (rc != NO_ERROR) {
                ALOGE("%s: Error taking picture !! ", __func__);
                return rc;
            }
        }
    }
    CDBG_HIGH("%s: X", __func__);
    return rc;
}

/*===========================================================================
 * FUNCTION   : cancel_picture
 *
 * DESCRIPTION: Cancel the take picture call
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *
 * RETURN     :
 *              NO_ERROR  : success
 *              other: non-zero failure code
 *==========================================================================*/
int QCameraMuxer::cancel_picture(struct camera_device * device)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER_ERROR();
    int rc = NO_ERROR;
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA_ERROR(cam);

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);

        rc = QCamera2HardwareInterface::cancel_picture(pCam->dev);
        if (rc != NO_ERROR) {
            ALOGE("%s: Error cancelling picture !! ", __func__);
            return rc;
        }
    }
    gMuxer->m_ComposeMpoTh.sendCmd(CAMERA_CMD_TYPE_STOP_DATA_PROC, FALSE, FALSE);
    // flush Jpeg Queues
    gMuxer->m_MainJpegQ.flush();
    gMuxer->m_AuxJpegQ.flush();

    CDBG_HIGH("%s: X", __func__);
    return rc;
}

/*===========================================================================
 * FUNCTION   : set_parameters
 *
 * DESCRIPTION: Sets the parameters on camera
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *   @parms : Parameters to be set on camera
 *
 * RETURN     :
 *              NO_ERROR  : success
 *              other: non-zero failure code
 *==========================================================================*/
int QCameraMuxer::set_parameters(struct camera_device * device,
        const char *parms)

{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER_ERROR();
    int rc = NO_ERROR;
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA_ERROR(cam);
    int previewRestartNeeded = 0;

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);

        rc = QCamera2HardwareInterface::set_parameters(pCam->dev, parms);
        if (rc != NO_ERROR) {
            ALOGE("%s: Error setting parameters !! ", __func__);
            return rc;
        }
    }

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);

        rc = QCamera2HardwareInterface::preview_restart_needed(pCam->dev,
                                                    previewRestartNeeded);
        if (rc != NO_ERROR) {
            ALOGE("%s: Error get restart status rc=%d!! ", __func__, rc);
            return rc;
        }
        if (previewRestartNeeded) {
            CDBG_HIGH("%s: Need Restart Preview from Muxer.", __func__);
            break;
        }
    }

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);

        CDBG("%s: stopping preview for cam %d", __func__, i);
        rc = QCamera2HardwareInterface::commit_parameters_stop_preview(
                                          pCam->dev, previewRestartNeeded);
        if (rc != NO_ERROR) {
            ALOGE("%s: Error committing parameters rc=%d!! ", __func__, rc);
            return rc;
        }
    }

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);

        CDBG("%s: starting preview for cam %d", __func__, i);
        rc = QCamera2HardwareInterface::commit_parameters_start_preview(
                                          pCam->dev, previewRestartNeeded);
        if (rc != NO_ERROR) {
            ALOGE("%s: Error committing parameters rc=%d!! ", __func__, rc);
            return rc;
        }
    }

    CDBG_HIGH("%s: X", __func__);
    return rc;
}

/*===========================================================================
 * FUNCTION   : get_parameters
 *
 * DESCRIPTION: Gets the parameters on camera
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *
 * RETURN     : Parameter string or NULL
 *==========================================================================*/
char* QCameraMuxer::get_parameters(struct camera_device * device)
{
    CDBG_HIGH("%s: E", __func__);

    if (!gMuxer)
        return NULL;

    char* ret = NULL;
    int rc = NO_ERROR;
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    if (!cam) {
        ALOGE("%s: Error getting logical camera", __func__);
        return NULL;
    }

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        if (!pCam) {
            ALOGE("%s: Error getting physical camera", __func__);
            return NULL;
        }
        QCamera2HardwareInterface *hwi = pCam->hwi;
        if (!hwi) {
            ALOGE("%s: Allocation of hardware interface failed", __func__);
            return NULL;
        }
        if (pCam->mode == CAM_MODE_PRIMARY) {
            // Get only primary camera parameters
            ret = QCamera2HardwareInterface::get_parameters(pCam->dev);
            break;
        }
    }

    CDBG_HIGH("%s: X", __func__);
    return ret;
}

/*===========================================================================
 * FUNCTION   : put_parameters
 *
 * DESCRIPTION: Puts parameters on camera
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *   @parm : parameters
 *
 * RETURN     : None
 *==========================================================================*/
void QCameraMuxer::put_parameters(struct camera_device * device, char *parm)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER();
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA(cam);

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI(hwi);

        if (pCam->mode == CAM_MODE_PRIMARY) {
            // Parameters are not used in HWI and hence freed
            QCamera2HardwareInterface::put_parameters(pCam->dev, parm);
            break;
        }
    }
    CDBG_HIGH("%s: X", __func__);
}

/*===========================================================================
 * FUNCTION   : send_command
 *
 * DESCRIPTION: Send command to camera
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *   @cmd : Command
 *   @arg1/arg2 : command arguments
 *
 * RETURN     :
 *              NO_ERROR  : success
 *              other: non-zero failure code
 *==========================================================================*/
int QCameraMuxer::send_command(struct camera_device * device,
        int32_t cmd, int32_t arg1, int32_t arg2)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER_ERROR();
    int rc = NO_ERROR;
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA_ERROR(cam);

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);

        rc = QCamera2HardwareInterface::send_command(pCam->dev, cmd, arg1, arg2);
        if (rc != NO_ERROR) {
            ALOGE("%s: Error sending command !! ", __func__);
            return rc;
        }
    }

        switch (cmd) {
#ifndef VANILLA_HAL
        case CAMERA_CMD_LONGSHOT_ON:
            for (uint32_t i = 0; i < cam->numCameras; i++) {
                pCam = gMuxer->getPhysicalCamera(cam, i);
                CHECK_CAMERA_ERROR(pCam);

                QCamera2HardwareInterface *hwi = pCam->hwi;
                CHECK_HWI_ERROR(hwi);

                rc = QCamera2HardwareInterface::send_command_restart(pCam->dev,
                        cmd, arg1, arg2);
                if (rc != NO_ERROR) {
                    ALOGE("%s: Error sending command restart !! ", __func__);
                    return rc;
                }
            }
        break;
        case CAMERA_CMD_LONGSHOT_OFF:
            gMuxer->m_ComposeMpoTh.sendCmd(CAMERA_CMD_TYPE_STOP_DATA_PROC,
                    FALSE, FALSE);
            // flush Jpeg Queues
            gMuxer->m_MainJpegQ.flush();
            gMuxer->m_AuxJpegQ.flush();
        break;
#endif
        default:
            // do nothing
            rc = NO_ERROR;
        break;
        }

    CDBG_HIGH("%s: X", __func__);
    return rc;
}

/*===========================================================================
 * FUNCTION   : release
 *
 * DESCRIPTION: Release the camera
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *
 * RETURN     : None
 *==========================================================================*/
void QCameraMuxer::release(struct camera_device * device)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER();
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA(cam);

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI(hwi);

        QCamera2HardwareInterface::release(pCam->dev);
    }
    CDBG_HIGH("%s: X", __func__);
}

/*===========================================================================
 * FUNCTION   : dump
 *
 * DESCRIPTION: Dump the camera info
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *   @fd : fd
 *
 * RETURN     :
 *              NO_ERROR  : success
 *              other: non-zero failure code
 *==========================================================================*/
int QCameraMuxer::dump(struct camera_device * device, int fd)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER_ERROR();
    int rc = NO_ERROR;
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA_ERROR(cam);

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);

        rc = QCamera2HardwareInterface::dump(pCam->dev, fd);
        if (rc != NO_ERROR) {
            ALOGE("%s: Error dumping", __func__);
            return rc;
        }
    }
    CDBG_HIGH("%s: X", __func__);
    return rc;
}

/*===========================================================================
 * FUNCTION   : close_camera_device
 *
 * DESCRIPTION: Close the camera
 *
 * PARAMETERS :
 *   @hw_dev : camera hardware device info
 *
 * RETURN     :
 *              NO_ERROR  : success
 *              other: non-zero failure code
 *==========================================================================*/
int QCameraMuxer::close_camera_device(hw_device_t *hw_dev)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER_ERROR();
    int rc = NO_ERROR;
    qcamera_physical_descriptor_t *pCam = NULL;
    camera_device_t *cam_dev = (camera_device_t*)hw_dev;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(cam_dev);
    CHECK_CAMERA_ERROR(cam);

    // Unlink camera sessions
    if (cam->bSyncOn) {
        if (cam->numCameras > 1) {
            uint sessionId = 0;
            // unbundle primary camera with all aux cameras
            for (uint32_t i = 0; i < cam->numCameras; i++) {
                pCam = gMuxer->getPhysicalCamera(cam, i);
                CHECK_CAMERA_ERROR(pCam);

                QCamera2HardwareInterface *hwi = pCam->hwi;
                CHECK_HWI_ERROR(hwi);

                if(pCam->mode == CAM_MODE_PRIMARY) {
                    // bundle primary cam with all aux cameras
                    for (uint32_t j = 0; j < cam->numCameras; j++) {
                        if (j == cam->nPrimaryPhyCamIndex) {
                            continue;
                        }
                        sessionId = cam->sId[j];
                        CDBG_HIGH("%s: Related cam id: %d, server id: %d sync OFF"
                                " related session_id %d", __func__,
                                cam->pId[i], cam->sId[i], sessionId);
                        rc = hwi->bundleRelatedCameras(false, sessionId);
                        if (rc != NO_ERROR) {
                            ALOGE("%s: Error Bundling physical cameras !! ", __func__);
                            break;
                        }
                    }
                }

                if (pCam->mode == CAM_MODE_SECONDARY) {
                    // unbundle all aux cam with primary cams
                    sessionId = cam->sId[cam->nPrimaryPhyCamIndex];
                    CDBG_HIGH("%s: Related cam id: %d, server id: %d sync OFF"
                            " related session_id %d", __func__,
                            cam->pId[i], cam->sId[i], sessionId);
                    rc = hwi->bundleRelatedCameras(false, sessionId);
                    if (rc != NO_ERROR) {
                        ALOGE("%s: Error Bundling physical cameras !! ", __func__);
                        break;
                    }
                }
            }
        }
        cam->bSyncOn = false;
    }

    // Attempt to close all cameras regardless of unbundle results
    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        hw_device_t *dev = (hw_device_t*)(pCam->dev);
        CDBG_HIGH("%s: hw device %x, hw %x", __func__, dev, pCam->hwi);

        rc = QCamera2HardwareInterface::close_camera_device(dev);
        if (rc != NO_ERROR) {
            ALOGE("%s: Error closing camera", __func__);
        }
        pCam->hwi = NULL;
        pCam->dev = NULL;
    }

    // Reset JPEG client handle
    gMuxer->setJpegHandle(0);
    CDBG_HIGH("%s: X, rc: %d", __func__, rc);
    return rc;
}

/*===========================================================================
 * FUNCTION         : setupLogicalCameras
 *
 * DESCRIPTION     : Creates Camera Muxer if not created
 *
 * RETURN     :
 *              NO_ERROR  : success
 *              other: non-zero failure code
 *==========================================================================*/
int QCameraMuxer::setupLogicalCameras()
{
    int rc = NO_ERROR;
    char prop[PROPERTY_VALUE_MAX];
    int i = 0;
    camera_info info;
    int primaryType = CAM_TYPE_MAIN;

    CDBG_HIGH("%s[%d] E: rc = %d", __func__, __LINE__, rc);
    // Signifies whether AUX camera has to be exposed as physical camera
    property_get("persist.camera.aux.camera", prop, "0");
    m_bAuxCameraExposed = atoi(prop);

    // Signifies whether AUX camera needs to be swapped
    property_get("persist.camera.auxcamera.swap", prop, "0");
    int swapAux = atoi(prop);
    if (swapAux != 0) {
        primaryType = CAM_TYPE_AUX;
    }

    // Check for number of camera present on device
    if (!m_nPhyCameras || (m_nPhyCameras > MM_CAMERA_MAX_NUM_SENSORS)) {
        ALOGE("%s: Error!! Invalid number of cameras: %d",
                __func__, m_nPhyCameras);
        return BAD_VALUE;
    }

    m_pPhyCamera = new qcamera_physical_descriptor_t[m_nPhyCameras];
    if (!m_pPhyCamera) {
        ALOGE("%s: Error allocating camera info buffer!!",__func__);
        return NO_MEMORY;
    }
    memset(m_pPhyCamera, 0x00,
            (m_nPhyCameras * sizeof(qcamera_physical_descriptor_t)));
    uint32_t cameraId = 0;
    m_nLogicalCameras = 0;

    // Enumerate physical cameras and logical
    for (i = 0; i < m_nPhyCameras ; i++, cameraId++) {
        camera_info *info = &m_pPhyCamera[i].cam_info;
        rc = QCamera2HardwareInterface::getCapabilities(cameraId,
                info, &m_pPhyCamera[i].type);
        m_pPhyCamera[i].id = cameraId;
        m_pPhyCamera[i].device_version = CAMERA_DEVICE_API_VERSION_1_0;
        m_pPhyCamera[i].mode = CAM_MODE_PRIMARY;

        if (!m_bAuxCameraExposed && (m_pPhyCamera[i].type != primaryType)) {
            m_pPhyCamera[i].mode = CAM_MODE_SECONDARY;
            CDBG_HIGH("%s[%d]: Camera ID: %d, Aux Camera, type: %d, facing: %d",
                    __func__, __LINE__, cameraId, m_pPhyCamera[i].type,
                    m_pPhyCamera[i].cam_info.facing);
        }
        else {
            m_nLogicalCameras++;
            CDBG_HIGH("%s[%d]: Camera ID: %d, Main Camera, type: %d, facing: %d",
                    __func__, __LINE__, cameraId, m_pPhyCamera[i].type,
                    m_pPhyCamera[i].cam_info.facing);
        }
    }

    if (!m_nLogicalCameras) {
        // No Main camera detected, return from here
        ALOGE("%s: Error !!!! detecting main camera!!",__func__);
        delete [] m_pPhyCamera;
        m_pPhyCamera = NULL;
        return -ENODEV;
    }
    // Allocate Logical Camera descriptors
    m_pLogicalCamera = new qcamera_logical_descriptor_t[m_nLogicalCameras];
    if (!m_pLogicalCamera) {
        ALOGE("%s: Error !!!! allocating camera info buffer!!",__func__);
        delete [] m_pPhyCamera;
        m_pPhyCamera = NULL;
        return  NO_MEMORY;
    }
    memset(m_pLogicalCamera, 0x00,
            (m_nLogicalCameras * sizeof(qcamera_logical_descriptor_t)));
    // Assign MAIN cameras for each logical camera
    int index = 0;
    for (i = 0; i < m_nPhyCameras ; i++) {
        if (m_pPhyCamera[i].mode == CAM_MODE_PRIMARY) {
            m_pLogicalCamera[index].nPrimaryPhyCamIndex = 0;
            m_pLogicalCamera[index].id = index;
            m_pLogicalCamera[index].device_version = CAMERA_DEVICE_API_VERSION_1_0;
            m_pLogicalCamera[index].pId[0] = i;
            m_pLogicalCamera[index].type[0] = CAM_TYPE_MAIN;
            m_pLogicalCamera[index].mode[0] = CAM_MODE_PRIMARY;
            m_pLogicalCamera[index].facing = m_pPhyCamera[i].cam_info.facing;
            m_pLogicalCamera[index].numCameras++;
            CDBG_HIGH("%s[%d]: Logical Main Camera ID: %d, facing: %d,"
                    "Phy Id: %d type: %d mode: %d",
                    __func__, __LINE__, m_pLogicalCamera[index].id,
                    m_pLogicalCamera[index].facing,
                    m_pLogicalCamera[index].pId[0],
                    m_pLogicalCamera[index].type[0],
                    m_pLogicalCamera[index].mode[0]);

            index++;
        }
    }
    //Now assign AUX cameras to logical camera
    for (i = 0; i < m_nPhyCameras ; i++) {
        if (m_pPhyCamera[i].mode == CAM_MODE_SECONDARY) {
            for (int j = 0; j < m_nLogicalCameras; j++) {
                int n = m_pLogicalCamera[j].numCameras;
                ///@note n can only be 1 at this point
                if ((n < MAX_NUM_CAMERA_PER_BUNDLE) &&
                        (m_pLogicalCamera[j].facing ==
                        m_pPhyCamera[i].cam_info.facing)) {
                    m_pLogicalCamera[j].pId[n] = i;
                    m_pLogicalCamera[j].type[n] = CAM_TYPE_AUX;
                    m_pLogicalCamera[j].mode[n] = CAM_MODE_SECONDARY;
                    m_pLogicalCamera[j].numCameras++;
                    CDBG_HIGH("%s[%d]: Aux %d for Logical Camera ID: %d,"
                        "aux phy id:%d, type: %d mode: %d",
                        __func__, __LINE__, n, j, m_pLogicalCamera[j].pId[n],
                        m_pLogicalCamera[j].type[n], m_pLogicalCamera[j].mode[n]);
                }
            }
        }
    }
    //Print logical and physical camera tables
    for (i = 0; i < m_nLogicalCameras ; i++) {
        for (uint8_t j = 0; j < m_pLogicalCamera[i].numCameras; j++) {
            CDBG_HIGH("%s[%d]: Logical Camera ID: %d, index: %d, "
                    "facing: %d, Phy Id: %d type: %d mode: %d",
                    __func__, __LINE__, i, j, m_pLogicalCamera[i].facing,
                    m_pLogicalCamera[i].pId[j], m_pLogicalCamera[i].type[j],
                    m_pLogicalCamera[i].mode[j]);
        }
    }
    CDBG_HIGH("%s[%d] X: rc = %d", __func__, __LINE__, rc);
    return rc;
}

/*===========================================================================
 * FUNCTION   : getNumberOfCameras
 *
 * DESCRIPTION: query number of logical cameras detected
 *
 * RETURN     : number of cameras detected
 *==========================================================================*/
int QCameraMuxer::getNumberOfCameras()
{
    return m_nLogicalCameras;
}

/*===========================================================================
 * FUNCTION   : getCameraInfo
 *
 * DESCRIPTION: query camera information with its ID
 *
 * PARAMETERS :
 *   @camera_id : camera ID
 *   @info      : ptr to camera info struct
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCameraMuxer::getCameraInfo(int camera_id,
        struct camera_info *info, cam_sync_type_t *p_cam_type)
{
    int rc = NO_ERROR;
    CDBG_HIGH("%s: E, camera_id = %d", __func__, camera_id);
    cam_sync_type_t cam_type = CAM_TYPE_MAIN;

    if (!m_nLogicalCameras || (camera_id >= m_nLogicalCameras) ||
            !info || (camera_id < 0)) {
        ALOGE("%s : m_nLogicalCameras: %d, camera id: %d", __func__,
                m_nLogicalCameras, camera_id);
        return -ENODEV;
    }

    if (!m_pLogicalCamera || !m_pPhyCamera) {
        ALOGE("%s : Error! Cameras not initialized!", __func__);
        return NO_INIT;
    }
    uint32_t phy_id =
            m_pLogicalCamera[camera_id].pId[
            m_pLogicalCamera[camera_id].nPrimaryPhyCamIndex];
    rc = QCamera2HardwareInterface::getCapabilities(phy_id, info, &cam_type);
    CDBG_HIGH("%s: X", __func__);
    return rc;
}

/*===========================================================================
 * FUNCTION   : setCallbacks
 *
 * DESCRIPTION: set callback functions to send asynchronous notifications to
 *              frameworks.
 *
 * PARAMETERS :
 *   @callbacks : callback function pointer
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraMuxer::setCallbacks(const camera_module_callbacks_t *callbacks)
{
    if(callbacks) {
        m_pCallbacks = callbacks;
        return NO_ERROR;
    } else {
        return BAD_TYPE;
    }
}

/*===========================================================================
 * FUNCTION   : setDataCallback
 *
 * DESCRIPTION: set data callback function for snapshots
 *
 * PARAMETERS :
 *   @data_cb : callback function pointer
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraMuxer::setDataCallback(camera_data_callback data_cb)
{
    if(data_cb) {
        mDataCb = data_cb;
        return NO_ERROR;
    } else {
        return BAD_TYPE;
    }
}

/*===========================================================================
 * FUNCTION   : setMemoryCallback
 *
 * DESCRIPTION: set get memory callback for memory allocations
 *
 * PARAMETERS :
 *   @get_memory : callback function pointer
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraMuxer::setMemoryCallback(camera_request_memory get_memory)
{
    if(get_memory) {
        mGetMemoryCb = get_memory;
        return NO_ERROR;
    } else {
        return BAD_TYPE;
    }
}

/*===========================================================================
 * FUNCTION   : setMpoCallbackCookie
 *
 * DESCRIPTION: set mpo callback cookie. will be used for sending final MPO callbacks
 *                     to framework
 *
 * PARAMETERS :
 *   @mpoCbCookie : callback function pointer
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraMuxer::setMpoCallbackCookie(void* mpoCbCookie)
{
    if(mpoCbCookie) {
        m_pMpoCallbackCookie = mpoCbCookie;
        return NO_ERROR;
    } else {
        return BAD_TYPE;
    }
}

/*===========================================================================
 * FUNCTION   : getMpoCallbackCookie
 *
 * DESCRIPTION: gets the mpo callback cookie. will be used for sending final MPO callbacks
 *                     to framework
 *
 * PARAMETERS :none
 *
 * RETURN     :void ptr to the mpo callback cookie
 *==========================================================================*/
void* QCameraMuxer::getMpoCallbackCookie(void)
{
    return m_pMpoCallbackCookie;
}

/*===========================================================================
 * FUNCTION   : setMainJpegCallbackCookie
 *
 * DESCRIPTION: set jpeg callback cookie.
 *                     set to phy cam instance of the primary related cam instance
 *
 * PARAMETERS :
 *   @jpegCbCookie : ptr to jpeg cookie
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraMuxer::setMainJpegCallbackCookie(void* jpegCbCookie)
{
    if(jpegCbCookie) {
        m_pJpegCallbackCookie = jpegCbCookie;
        return NO_ERROR;
    } else {
        return BAD_TYPE;
    }
}

/*===========================================================================
 * FUNCTION   : getMainJpegCallbackCookie
 *
 * DESCRIPTION: gets the jpeg callback cookie for primary related cam instance
 *                     set to phy cam instance of the primary related cam instance
 *
 * PARAMETERS :none
 *
 * RETURN     :void ptr to the jpeg callback cookie
 *==========================================================================*/
void* QCameraMuxer::getMainJpegCallbackCookie(void)
{
    return m_pJpegCallbackCookie;
}

/*===========================================================================
 * FUNCTION   : cameraDeviceOpen
 *
 * DESCRIPTION: open a camera device with its ID
 *
 * PARAMETERS :
 *   @camera_id : camera ID
 *   @hw_device : ptr to struct storing camera hardware device info
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCameraMuxer::cameraDeviceOpen(int camera_id,
        struct hw_device_t **hw_device)
{
    int rc = NO_ERROR;
    uint32_t phyId = 0;
    qcamera_logical_descriptor_t *cam = NULL;

    if (camera_id < 0 || camera_id >= m_nLogicalCameras) {
        ALOGE("%s : Camera id %d not found!", __func__, camera_id);
        return -ENODEV;
    }

    if ( NULL == m_pLogicalCamera) {
        ALOGE("%s : Hal descriptor table is not initialized!", __func__);
        return NO_INIT;
    }

    char prop[PROPERTY_VALUE_MAX];
    property_get("persist.camera.dc.frame.sync", prop, "1");
    m_bFrameSyncEnabled = atoi(prop);

    // Get logical camera
    cam = &m_pLogicalCamera[camera_id];

    if (m_pLogicalCamera[camera_id].device_version ==
            CAMERA_DEVICE_API_VERSION_1_0) {
        // HW Dev Holders
        hw_device_t *hw_dev[cam->numCameras];

        if (m_pPhyCamera[cam->pId[0]].type != CAM_TYPE_MAIN) {
            ALOGE("%s: Physical camera at index 0 is not main!", __func__);
            return UNKNOWN_ERROR;
        }

        // Open all physical cameras
        for (uint32_t i = 0; i < cam->numCameras; i++) {
            phyId = cam->pId[i];
            QCamera2HardwareInterface *hw =
                    new QCamera2HardwareInterface((uint32_t)phyId);
            if (!hw) {
                ALOGE("%s: Allocation of hardware interface failed", __func__);
                return NO_MEMORY;
            }
            hw_dev[i] = NULL;

            // Make Camera HWI aware of its mode
            cam_sync_related_sensors_event_info_t info;
            info.sync_control = CAM_SYNC_RELATED_SENSORS_ON;
            info.mode = m_pPhyCamera[phyId].mode;
            info.type = m_pPhyCamera[phyId].type;
            info.is_frame_sync_enabled = m_bFrameSyncEnabled;
            rc = hw->setRelatedCamSyncInfo(&info);
            if (rc != NO_ERROR) {
                ALOGE("%s: setRelatedCamSyncInfo failed %d", __func__, rc);
                delete hw;
                return rc;
            }

            rc = hw->openCamera(&hw_dev[i]);
            if (rc != NO_ERROR) {
                delete hw;
                return rc;
            }
            hw->getCameraSessionId(&m_pPhyCamera[phyId].camera_server_id);
            m_pPhyCamera[phyId].dev = reinterpret_cast<camera_device_t*>(hw_dev[i]);
            m_pPhyCamera[phyId].hwi = hw;
            cam->sId[i] = m_pPhyCamera[phyId].camera_server_id;
            CDBG_HIGH("%s: camera id %d server id : %d hw device %x, hw %x",
                    __func__, phyId, cam->sId[i], hw_dev[i], hw);
        }
    } else {
        ALOGE("%s: Device version for camera id %d invalid %d",
                __func__, camera_id, m_pLogicalCamera[camera_id].device_version);
        return BAD_VALUE;
    }

    cam->dev.common.tag = HARDWARE_DEVICE_TAG;
    cam->dev.common.version = HARDWARE_DEVICE_API_VERSION(1, 0);
    cam->dev.common.close = close_camera_device;
    cam->dev.ops = &mCameraMuxerOps;
    cam->dev.priv = (void*)cam;
    *hw_device = &cam->dev.common;
    return rc;
}


/*===========================================================================
 * FUNCTION   : getLogicalCamera
 *
 * DESCRIPTION: Get logical camera descriptor
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *
 * RETURN     : logical camera descriptor or NULL
 *==========================================================================*/
qcamera_logical_descriptor_t* QCameraMuxer::getLogicalCamera(
        struct camera_device * device)
{
    if(device && device->priv){
        return (qcamera_logical_descriptor_t*)(device->priv);
    }
    return NULL;
}

/*===========================================================================
 * FUNCTION   : getPhysicalCamera
 *
 * DESCRIPTION: Get physical camera descriptor
 *
 * PARAMETERS :
 *   @log_cam : Logical camera descriptor
 *   @index : physical camera index
 *
 * RETURN     : physical camera descriptor or NULL
 *==========================================================================*/
qcamera_physical_descriptor_t* QCameraMuxer::getPhysicalCamera(
        qcamera_logical_descriptor_t* log_cam, uint32_t index)
{
    if(!log_cam){
        return NULL;
    }
    return &m_pPhyCamera[log_cam->pId[index]];
}

/*===========================================================================
 * FUNCTION   : getActiveNumOfPhyCam
 *
 * DESCRIPTION: Get active physical camera number in Logical Camera
 *
 * PARAMETERS :
 *   @log_cam :   Logical camera descriptor
 *   @numOfAcitvePhyCam :  number of active physical camera in Logical Camera.
 *
 * RETURN     :
 *                NO_ERROR  : success
 *                ENODEV : Camera not found
 *                other: non-zero failure code
 *==========================================================================*/
int32_t QCameraMuxer::getActiveNumOfPhyCam(
        qcamera_logical_descriptor_t* log_cam, int& numOfAcitvePhyCam)
{
    CHECK_CAMERA_ERROR(log_cam);

    numOfAcitvePhyCam = log_cam->numCameras;
    return NO_ERROR;
}


/*===========================================================================
 * FUNCTION   : sendEvtNotify
 *
 * DESCRIPTION: send event notify to HWI for error callbacks
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
int32_t QCameraMuxer::sendEvtNotify(int32_t msg_type, int32_t ext1,
        int32_t ext2)
{
    CDBG_HIGH("%s: E", __func__);
    int rc = NO_ERROR;

    CHECK_MUXER_ERROR();

    qcamera_physical_descriptor_t *pCam = NULL;
    pCam = (qcamera_physical_descriptor_t*)(gMuxer->getMainJpegCallbackCookie());

    CHECK_CAMERA_ERROR(pCam);

    QCamera2HardwareInterface *hwi = pCam->hwi;
    CHECK_HWI_ERROR(hwi);

    CDBG_HIGH("%s: X", __func__);
    return pCam->hwi->sendEvtNotify(msg_type, ext1, ext2);
}

/*===========================================================================
 * FUNCTION   : composeMpo
 *
 * DESCRIPTION: Composition of the 2 MPOs
 *
 * PARAMETERS : none
 *   @main_Jpeg: pointer to info to Main Jpeg
 *   @aux_Jpeg : pointer to info to Aux JPEG
 *
  * RETURN : none
 *==========================================================================*/
void QCameraMuxer::composeMpo(cam_compose_jpeg_info_t* main_Jpeg,
        cam_compose_jpeg_info_t* aux_Jpeg)
{
    CDBG_HIGH("%s: E Main Jpeg %p Aux Jpeg %p", __func__, main_Jpeg, aux_Jpeg);

    CHECK_MUXER();
    if(main_Jpeg == NULL || aux_Jpeg == NULL) {
        ALOGE("%s : input buffers invalid, ret = NO_MEMORY", __func__);
        gMuxer->sendEvtNotify(CAMERA_MSG_ERROR, UNKNOWN_ERROR, 0);
        return;
    }

    pthread_mutex_lock(&m_JpegLock);

    m_pRelCamMpoJpeg = mGetMemoryCb(-1, main_Jpeg->buffer->size +
            aux_Jpeg->buffer->size, 1, m_pMpoCallbackCookie);
    if (NULL == m_pRelCamMpoJpeg) {
        ALOGE("%s : getMemory for mpo, ret = NO_MEMORY", __func__);
        gMuxer->sendEvtNotify(CAMERA_MSG_ERROR, UNKNOWN_ERROR, 0);
        pthread_mutex_unlock(&m_JpegLock);
        return;
    }

    // fill all structures to send for composition
    mm_jpeg_mpo_info_t mpo_compose_info;
    mpo_compose_info.num_of_images = 2;
    mpo_compose_info.primary_image.buf_filled_len = main_Jpeg->buffer->size;
    mpo_compose_info.primary_image.buf_vaddr =
            (uint8_t*)(main_Jpeg->buffer->data);
    mpo_compose_info.aux_images[0].buf_filled_len = aux_Jpeg->buffer->size;
    mpo_compose_info.aux_images[0].buf_vaddr =
            (uint8_t*)(aux_Jpeg->buffer->data);
    mpo_compose_info.output_buff.buf_vaddr =
            (uint8_t*)m_pRelCamMpoJpeg->data;
    mpo_compose_info.output_buff.buf_filled_len = 0;
    mpo_compose_info.output_buff_size = main_Jpeg->buffer->size +
            aux_Jpeg->buffer->size;

    CDBG("%s: MPO buffer size %d\n"
            "expected size %d, mpo_compose_info.output_buff_size %d",
            __func__, m_pRelCamMpoJpeg->size,
            main_Jpeg->buffer->size + aux_Jpeg->buffer->size,
            mpo_compose_info.output_buff_size);

    CDBG("%s: MPO primary buffer filled lengths\n"
            "mpo_compose_info.primary_image.buf_filled_len %d\n"
            "mpo_compose_info.primary_image.buf_vaddr %p", __func__,
            mpo_compose_info.primary_image.buf_filled_len,
            mpo_compose_info.primary_image.buf_vaddr);

    CDBG("%s: MPO aux buffer filled lengths\n"
            "mpo_compose_info.aux_images[0].buf_filled_len %d"
            "mpo_compose_info.aux_images[0].buf_vaddr %p", __func__,
            mpo_compose_info.aux_images[0].buf_filled_len,
            mpo_compose_info.aux_images[0].buf_vaddr);

    if(m_bDumpImages) {
        CDBG("%s: Dumping Main Image for MPO", __func__);
        char buf_main[QCAMERA_MAX_FILEPATH_LENGTH];
        memset(buf_main, 0, sizeof(buf_main));
        snprintf(buf_main, sizeof(buf_main),
                QCAMERA_DUMP_FRM_LOCATION "Main.jpg");

        int file_fd_main = open(buf_main, O_RDWR | O_CREAT, 0777);
        if (file_fd_main >= 0) {
            ssize_t written_len = write(file_fd_main,
                    mpo_compose_info.primary_image.buf_vaddr,
                    mpo_compose_info.primary_image.buf_filled_len);
            fchmod(file_fd_main, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
            CDBG("%s: written number of bytes for main Image %zd\n",
                    __func__, written_len);
            close(file_fd_main);
        }

        CDBG("%s: Dumping Aux Image for MPO", __func__);
        char buf_aux[QCAMERA_MAX_FILEPATH_LENGTH];
        memset(buf_aux, 0, sizeof(buf_aux));
        snprintf(buf_aux, sizeof(buf_aux),
                QCAMERA_DUMP_FRM_LOCATION "Aux.jpg");

        int file_fd_aux = open(buf_aux, O_RDWR | O_CREAT, 0777);
        if (file_fd_aux >= 0) {
            ssize_t written_len = write(file_fd_aux,
                    mpo_compose_info.aux_images[0].buf_vaddr,
                    mpo_compose_info.aux_images[0].buf_filled_len);
            fchmod(file_fd_aux, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
            CDBG("%s: written number of bytes for Aux Image %zd\n",
                    __func__, written_len);
            close(file_fd_aux);
        }
    }

    int32_t rc = mJpegMpoOps.compose_mpo(&mpo_compose_info);
    CDBG("%s: Compose mpo returned %d", __func__, rc);

    if(rc != NO_ERROR) {
        ALOGE("%s : ComposeMpo failed, ret = %d", __func__, rc);
        gMuxer->sendEvtNotify(CAMERA_MSG_ERROR, UNKNOWN_ERROR, 0);
        pthread_mutex_unlock(&m_JpegLock);
        return;
    }

    if(m_bDumpImages) {
        char buf_mpo[QCAMERA_MAX_FILEPATH_LENGTH];
        memset(buf_mpo, 0, sizeof(buf_mpo));
        snprintf(buf_mpo, sizeof(buf_mpo),
                QCAMERA_DUMP_FRM_LOCATION "Composed.MPO");

        int file_fd_mpo = open(buf_mpo, O_RDWR | O_CREAT, 0777);
        if (file_fd_mpo >= 0) {
            ssize_t written_len = write(file_fd_mpo,
                    m_pRelCamMpoJpeg->data,
                    m_pRelCamMpoJpeg->size);
            fchmod(file_fd_mpo, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
            CDBG("%s: written number of bytes for MPO Image %zd\n",
                    __func__, written_len);
            close(file_fd_mpo);
        }
    }

    mDataCb(main_Jpeg->msg_type,
            m_pRelCamMpoJpeg,
            main_Jpeg->index,
            main_Jpeg->metadata,
            m_pMpoCallbackCookie);

    if (NULL != m_pRelCamMpoJpeg) {
        m_pRelCamMpoJpeg->release(m_pRelCamMpoJpeg);
        m_pRelCamMpoJpeg = NULL;
    }

    pthread_mutex_unlock(&m_JpegLock);
    CDBG_HIGH("%s: X", __func__);
    return;
}

/*===========================================================================
 * FUNCTION   : matchFrameId
 *
 * DESCRIPTION: function to match frame ids within queue nodes
 *
 * PARAMETERS :
 *   @data: pointer to queue node to be matched for condition
 *   @user_data: caller can add more info here
 *   @match_data : value to be matched against
 *
 * RETURN     : true or false based on whether match was successful or not
 *==========================================================================*/
bool QCameraMuxer::matchFrameId(void *data, void *user_data,
        void *match_data)
{
    CDBG_HIGH("%s: E", __func__);

    if (!data || !match_data) {
        return false;
    }

    cam_compose_jpeg_info_t * node = (cam_compose_jpeg_info_t *) data;
    uint32_t frame_idx = *((uint32_t *) match_data);
    CDBG_HIGH("%s: X", __func__);
    return node->frame_idx == frame_idx;
}

/*===========================================================================
 * FUNCTION   : findPreviousJpegs
 *
 * DESCRIPTION: Finds Jpegs in the queue with index less than delivered one
 *
 * PARAMETERS :
 *   @data: pointer to queue node to be matched for condition
 *   @user_data: caller can add more info here
 *   @match_data : value to be matched against
 *
 * RETURN     : true or false based on whether match was successful or not
 *==========================================================================*/
bool QCameraMuxer::findPreviousJpegs(void *data, void *user_data,
        void *match_data)
{
    CDBG_HIGH("%s: E", __func__);

    if (!data || !match_data) {
        return false;
    }
    cam_compose_jpeg_info_t * node = (cam_compose_jpeg_info_t *) data;
    uint32_t frame_idx = *((uint32_t *) match_data);
    CDBG_HIGH("%s: X", __func__);
    return node->frame_idx < frame_idx;
}

/*===========================================================================
 * FUNCTION   : releaseJpegInfo
 *
 * DESCRIPTION: callback function for the release of individual nodes
 *                     in the JPEG queues.
 *
 * PARAMETERS :
 *   @data      : ptr to the data to be released
 *   @user_data : caller can add more info here
 *
 * RETURN     : None
 *==========================================================================*/
void QCameraMuxer::releaseJpegInfo(void *data, void *user_data)
{
    CDBG_HIGH("%s: E", __func__);

    cam_compose_jpeg_info_t *jpegInfo = (cam_compose_jpeg_info_t *)data;
    if(jpegInfo && jpegInfo->release_cb) {
        if (jpegInfo->release_data != NULL) {
            jpegInfo->release_cb(jpegInfo->release_data,
                    jpegInfo->release_cookie,
                    NO_ERROR);
        }
    }
    CDBG_HIGH("%s: X", __func__);
}

/*===========================================================================
 * FUNCTION   : composeMpoRoutine
 *
 * DESCRIPTION: specialized thread for MPO composition
 *
 * PARAMETERS :
 *   @data   : pointer to the thread owner
 *
 * RETURN     : void* to thread
 *==========================================================================*/
void* QCameraMuxer::composeMpoRoutine(void *data)
{
    CDBG_HIGH("%s: E", __func__);
    if (!gMuxer) {
        ALOGE("%s[%d]: Error getting muxer ", __func__, __LINE__);
        return NULL;
    }

    int running = 1;
    int ret;
    uint8_t is_active = FALSE;
    QCameraCmdThread *cmdThread = &gMuxer->m_ComposeMpoTh;
    cmdThread->setName("CAM_ComposeMpo");
    char saveName[PROPERTY_VALUE_MAX];

    do {
        do {
            ret = cam_sem_wait(&cmdThread->cmd_sem);
            if (ret != 0 && errno != EINVAL) {
                ALOGE("%s: cam_sem_wait error (%s)", __func__, strerror(errno));
                return NULL;
            }
        } while (ret != 0);

        // we got notified about new cmd avail in cmd queue
        camera_cmd_type_t cmd = cmdThread->getCmd();
        switch (cmd) {
        case CAMERA_CMD_TYPE_START_DATA_PROC:
            {
                CDBG_HIGH("%s: start ComposeMpo processing", __func__);
                is_active = TRUE;

                // signal cmd is completed
                cam_sem_post(&cmdThread->sync_sem);
            }
            break;
        case CAMERA_CMD_TYPE_STOP_DATA_PROC:
            {
                CDBG_HIGH("%s: stop ComposeMpo processing", __func__);
                is_active = FALSE;

                // signal cmd is completed
                cam_sem_post(&cmdThread->sync_sem);
            }
            break;
        case CAMERA_CMD_TYPE_DO_NEXT_JOB:
            {
                if (is_active == TRUE) {
                    CDBG_HIGH("%s: Mpo Composition Requested", __func__);
                    cam_compose_jpeg_info_t *main_jpeg_node = NULL;
                    cam_compose_jpeg_info_t *aux_jpeg_node = NULL;
                    bool foundMatch = false;
                    while (!gMuxer->m_MainJpegQ.isEmpty() &&
                            !gMuxer->m_AuxJpegQ.isEmpty()) {
                        main_jpeg_node = (cam_compose_jpeg_info_t *)
                                gMuxer->m_MainJpegQ.dequeue();
                        if (main_jpeg_node) {
                            CDBG("%s: main_jpeg_node found frame idx %d"
                                    "ptr %p buffer_ptr %p buffer_size %d",
                                    __func__, main_jpeg_node->frame_idx,
                                    main_jpeg_node,
                                    main_jpeg_node->buffer->data,
                                    main_jpeg_node->buffer->size);
                            // find matching aux node in Aux Jpeg Queue
                            aux_jpeg_node =
                                    (cam_compose_jpeg_info_t *) gMuxer->
                                    m_AuxJpegQ.dequeue();
                            if (aux_jpeg_node) {
                                CDBG("%s: aux_jpeg_node found frame idx %d"
                                        "ptr %p buffer_ptr %p buffer_size %d",
                                        __func__, aux_jpeg_node->frame_idx,
                                        aux_jpeg_node,
                                        aux_jpeg_node->buffer->data,
                                        aux_jpeg_node->buffer->size);
                                foundMatch = true;
                                // start MPO composition
                                gMuxer->composeMpo(main_jpeg_node,
                                        aux_jpeg_node);
                            }
                        }
                        if (main_jpeg_node) {
                            if ( main_jpeg_node->release_cb ) {
                                main_jpeg_node->release_cb(
                                        main_jpeg_node->release_data,
                                        main_jpeg_node->release_cookie,
                                        NO_ERROR);
                            }
                            free(main_jpeg_node);
                        } else {
                            CDBG_HIGH("%s: Mpo Match not found", __func__);
                        }
                        if (aux_jpeg_node) {
                            if ( aux_jpeg_node->release_cb ) {
                                aux_jpeg_node->release_cb(
                                        aux_jpeg_node->release_data,
                                        aux_jpeg_node->release_cookie,
                                        NO_ERROR);
                            }
                            free(aux_jpeg_node);
                        } else {
                            CDBG_HIGH("%s: Mpo Match not found", __func__);
                        }
                    }
                }
            break;
            }
        case CAMERA_CMD_TYPE_EXIT:
            CDBG_HIGH("%s : ComposeMpo thread exit", __func__);
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
 * FUNCTION   : jpeg_data_callback
 *
 * DESCRIPTION: JPEG data callback for snapshot
 *
 * PARAMETERS :
 *   @msg_type : callback msg type
 *   @data : data ptr of the buffer
 *   @index : index of the frame
 *   @metadata : metadata associated with the buffer
 *   @user : callback cookie returned back to the user
 *   @frame_idx : frame index for matching frames
 *   @release_cb : callback function for releasing the data memory
 *   @release_cookie : cookie for the release callback function
 *   @release_data :pointer indicating what needs to be released
 *
 * RETURN : none
 *==========================================================================*/
void QCameraMuxer::jpeg_data_callback(int32_t msg_type,
           const camera_memory_t *data, unsigned int index,
           camera_frame_metadata_t *metadata, void *user,
           uint32_t frame_idx, camera_release_callback release_cb,
           void *release_cookie, void *release_data)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER();

    if(data != NULL) {
        CDBG_HIGH("%s: jpeg received: data %p size %d data ptr %p frameIdx %d",
                __func__, data, data->size, data->data, frame_idx);
        int rc = gMuxer->storeJpeg(((qcamera_physical_descriptor_t*)(user))->type,
                msg_type, data, index, metadata, user, frame_idx, release_cb,
                release_cookie, release_data);
        if(rc != NO_ERROR) {
            gMuxer->sendEvtNotify(CAMERA_MSG_ERROR, UNKNOWN_ERROR, 0);
        }
    } else {
        gMuxer->sendEvtNotify(CAMERA_MSG_ERROR, UNKNOWN_ERROR, 0);
    }
    CDBG_HIGH("%s: X", __func__);
    return;
}

/*===========================================================================
 * FUNCTION   : storeJpeg
 *
 * DESCRIPTION: Stores jpegs from multiple related cam instances into a common Queue
 *
 * PARAMETERS :
 *   @cam_type : indicates whether main or aux camera sent the Jpeg callback
 *   @msg_type : callback msg type
 *   @data : data ptr of the buffer
 *   @index : index of the frame
 *   @metadata : metadata associated with the buffer
 *   @user : callback cookie returned back to the user
 *   @frame_idx : frame index for matching frames
 *   @release_cb : callback function for releasing the data memory
 *   @release_cookie : cookie for the release callback function
 *   @release_data :pointer indicating what needs to be released
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraMuxer::storeJpeg(cam_sync_type_t cam_type,
        int32_t msg_type, const camera_memory_t *data, unsigned int index,
        camera_frame_metadata_t *metadata, void *user,uint32_t frame_idx,
        camera_release_callback release_cb, void *release_cookie,
        void *release_data)
{
    CDBG_HIGH("%s: E jpeg received: data %p size %d data ptr %p frameIdx %d",
            __func__, data, data->size, data->data, frame_idx);

    CHECK_MUXER_ERROR();

    if (!m_bMpoEnabled) {
        if (cam_type == CAM_TYPE_MAIN) {
            // send data callback only incase of main camera
            // aux image is ignored and released back
            mDataCb(msg_type,
                    data,
                    index,
                    metadata,
                    m_pMpoCallbackCookie);
        }
        if (release_cb) {
            release_cb(release_data, release_cookie, NO_ERROR);
        }
        CDBG_HIGH("%s: X", __func__);
        return NO_ERROR;
    }

    cam_compose_jpeg_info_t* pJpegFrame =
            (cam_compose_jpeg_info_t*)malloc(sizeof(cam_compose_jpeg_info_t));
    if (!pJpegFrame) {
        ALOGE("%s: Allocation failed for MPO nodes", __func__);
        return NO_MEMORY;
    }
    memset(pJpegFrame, 0, sizeof(*pJpegFrame));

    pJpegFrame->msg_type = msg_type;
    pJpegFrame->buffer = const_cast<camera_memory_t*>(data);
    pJpegFrame->index = index;
    pJpegFrame->metadata = metadata;
    pJpegFrame->user = user;
    pJpegFrame->valid = true;
    pJpegFrame->frame_idx = frame_idx;
    pJpegFrame->release_cb = release_cb;
    pJpegFrame->release_cookie = release_cookie;
    pJpegFrame->release_data = release_data;
    if(cam_type == CAM_TYPE_MAIN) {
        if (m_MainJpegQ.enqueue((void *)pJpegFrame)) {
            CDBG("%s:Main FrameIdx %d", __func__, pJpegFrame->frame_idx);
            if (m_MainJpegQ.getCurrentSize() > 0) {
                CDBG("%s: Trigger Compose", __func__);
                m_ComposeMpoTh.sendCmd(CAMERA_CMD_TYPE_DO_NEXT_JOB, FALSE, FALSE);
            }
        } else {
            ALOGE("%s: Enqueue Failed for Main Jpeg Q", __func__);
            if ( pJpegFrame->release_cb ) {
                // release other buffer also here
                pJpegFrame->release_cb(
                        pJpegFrame->release_data,
                        pJpegFrame->release_cookie,
                        NO_ERROR);
            }
            free(pJpegFrame);
            pJpegFrame = NULL;
            return NO_MEMORY;
        }

    } else {
        if (m_AuxJpegQ.enqueue((void *)pJpegFrame)) {
            CDBG("%s:Aux FrameIdx %d", __func__, pJpegFrame->frame_idx);
            if (m_AuxJpegQ.getCurrentSize() > 0) {
                CDBG("%s: Trigger Compose", __func__);
                m_ComposeMpoTh.sendCmd(CAMERA_CMD_TYPE_DO_NEXT_JOB, FALSE, FALSE);
            }
        } else {
            ALOGE("%s: Enqueue Failed for Aux Jpeg Q", __func__);
            if ( pJpegFrame->release_cb ) {
                // release other buffer also here
                pJpegFrame->release_cb(
                        pJpegFrame->release_data,
                        pJpegFrame->release_cookie,
                        NO_ERROR);
            }
            free(pJpegFrame);
            pJpegFrame = NULL;
            return NO_MEMORY;
        }
    }
    CDBG_HIGH("%s: X", __func__);

    return NO_ERROR;
}


// Muxer Ops
camera_device_ops_t QCameraMuxer::mCameraMuxerOps = {
    set_preview_window:         QCameraMuxer::set_preview_window,
    set_callbacks:              QCameraMuxer::set_callBacks,
    enable_msg_type:            QCameraMuxer::enable_msg_type,
    disable_msg_type:           QCameraMuxer::disable_msg_type,
    msg_type_enabled:           QCameraMuxer::msg_type_enabled,

    start_preview:              QCameraMuxer::start_preview,
    stop_preview:               QCameraMuxer::stop_preview,
    preview_enabled:            QCameraMuxer::preview_enabled,
    store_meta_data_in_buffers: QCameraMuxer::store_meta_data_in_buffers,

    start_recording:            QCameraMuxer::start_recording,
    stop_recording:             QCameraMuxer::stop_recording,
    recording_enabled:          QCameraMuxer::recording_enabled,
    release_recording_frame:    QCameraMuxer::release_recording_frame,

    auto_focus:                 QCameraMuxer::auto_focus,
    cancel_auto_focus:          QCameraMuxer::cancel_auto_focus,

    take_picture:               QCameraMuxer::take_picture,
    cancel_picture:             QCameraMuxer::cancel_picture,

    set_parameters:             QCameraMuxer::set_parameters,
    get_parameters:             QCameraMuxer::get_parameters,
    put_parameters:             QCameraMuxer::put_parameters,
    send_command:               QCameraMuxer::send_command,

    release:                    QCameraMuxer::release,
    dump:                       QCameraMuxer::dump,
};


}; // namespace android
