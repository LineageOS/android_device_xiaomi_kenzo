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

#define LOG_TAG "QCamera2Factory"
//#define LOG_NDEBUG 0

#include <stdlib.h>
#include <utils/Log.h>
#include <utils/Errors.h>
#include <hardware/camera.h>
#include <hardware/camera3.h>

#include "HAL/QCamera2HWI.h"
#include "HAL3/QCamera3HWI.h"
#include "QCamera2Factory.h"
#include "QCameraMuxer.h"

using namespace android;

namespace qcamera {

QCamera2Factory *gQCamera2Factory = NULL;
QCameraMuxer *gQCameraMuxer = NULL;
pthread_mutex_t gCamLock = PTHREAD_MUTEX_INITIALIZER;
//Total number of cameras opened simultaneously.
//This variable updation is protected by gCamLock.
uint8_t gNumCameraSessions = 0;

volatile uint32_t gKpiDebugLevel = 1;

/*===========================================================================
 * FUNCTION   : QCamera2Factory
 *
 * DESCRIPTION: default constructor of QCamera2Factory
 *
 * PARAMETERS : none
 *
 * RETURN     : None
 *==========================================================================*/
QCamera2Factory::QCamera2Factory()
{
    camera_info info;
    mHalDescriptors = NULL;
    mCallbacks = NULL;
    mNumOfCameras = get_num_of_cameras();
    int bDualCamera = 0;
    char propDefault[PROPERTY_VALUE_MAX];
    char prop[PROPERTY_VALUE_MAX];
    property_get("persist.camera.HAL3.enabled", prop, "1");
    int isHAL3Enabled = atoi(prop);

    // Signifies whether system has to enable dual camera mode
    snprintf(propDefault, PROPERTY_VALUE_MAX, "%d", isDualCamAvailable(isHAL3Enabled));
    property_get("persist.camera.dual.camera", prop, propDefault);
    bDualCamera = atoi(prop);
    CDBG_HIGH("%s[%d]: dualCamera:%d ", __func__, __LINE__, bDualCamera);

    if(bDualCamera) {
        ALOGI("%s[%d]: Enabling QCamera Muxer", __func__, __LINE__);
        if (!gQCameraMuxer) {
            QCameraMuxer::getCameraMuxer(&gQCameraMuxer, mNumOfCameras);
            if (!gQCameraMuxer) {
                ALOGE("%s: Error !! Failed to get QCameraMuxer", __func__);
            }
        }
    }
    if (!gQCameraMuxer && (mNumOfCameras > 0) &&
            (mNumOfCameras <= MM_CAMERA_MAX_NUM_SENSORS)) {
        mHalDescriptors = new hal_desc[mNumOfCameras];
        if ( NULL != mHalDescriptors) {
            uint32_t cameraId = 0;

            for (int i = 0; i < mNumOfCameras ; i++, cameraId++) {
                mHalDescriptors[i].cameraId = cameraId;
                // Set Device version to 3.x when both HAL3 is enabled & its BAYER sensor
                if (isHAL3Enabled && !(is_yuv_sensor(cameraId))) {
                    mHalDescriptors[i].device_version =
                            CAMERA_DEVICE_API_VERSION_3_0;
                } else {
                    mHalDescriptors[i].device_version =
                            CAMERA_DEVICE_API_VERSION_1_0;
                }
                //Query camera at this point in order
                //to avoid any delays during subsequent
                //calls to 'getCameraInfo()'
                getCameraInfo(i, &info);
            }
        } else {
            ALOGE("%s: Not enough resources to allocate HAL descriptor table!",
                  __func__);
        }
    } else {
        ALOGI("%s: %d camera devices detected!", __func__, mNumOfCameras);
    }
}

/*===========================================================================
 * FUNCTION   : ~QCamera2Factory
 *
 * DESCRIPTION: deconstructor of QCamera2Factory
 *
 * PARAMETERS : none
 *
 * RETURN     : None
 *==========================================================================*/
QCamera2Factory::~QCamera2Factory()
{
    if ( NULL != mHalDescriptors ) {
        delete [] mHalDescriptors;
    }
    if (gQCameraMuxer) {
        delete gQCameraMuxer;
        gQCameraMuxer = NULL;
    }
}

/*===========================================================================
 * FUNCTION   : get_number_of_cameras
 *
 * DESCRIPTION: static function to query number of cameras detected
 *
 * PARAMETERS : none
 *
 * RETURN     : number of cameras detected
 *==========================================================================*/
int QCamera2Factory::get_number_of_cameras()
{
    int numCameras = 0;
    int rc = NO_ERROR;

    if (!gQCamera2Factory) {
        gQCamera2Factory = new QCamera2Factory();
        if (!gQCamera2Factory) {
            ALOGE("%s: Failed to allocate Camera2Factory object", __func__);
            return 0;
        }
    }

    if(gQCameraMuxer)
        numCameras = gQCameraMuxer->get_number_of_cameras();
    else
        numCameras = gQCamera2Factory->getNumberOfCameras();

    CDBG_HIGH("%s: num of cameras: %d", __func__, numCameras);
    return numCameras;
}

/*===========================================================================
 * FUNCTION   : get_camera_info
 *
 * DESCRIPTION: static function to query camera information with its ID
 *
 * PARAMETERS :
 *   @camera_id : camera ID
 *   @info      : ptr to camera info struct
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2Factory::get_camera_info(int camera_id, struct camera_info *info)
{
    int rc = NO_ERROR;

    if(gQCameraMuxer)
        rc = gQCameraMuxer->get_camera_info(camera_id, info);
    else
        rc =  gQCamera2Factory->getCameraInfo(camera_id, info);

    return rc;
}

/*===========================================================================
 * FUNCTION   : set_callbacks
 *
 * DESCRIPTION: static function to set callbacks function to camera module
 *
 * PARAMETERS :
 *   @callbacks : ptr to callback functions
 *
 * RETURN     : NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2Factory::set_callbacks(const camera_module_callbacks_t *callbacks)
{
    int rc = NO_ERROR;
    if(gQCameraMuxer)
        rc = gQCameraMuxer->set_callbacks(callbacks);
    else
        rc =  gQCamera2Factory->setCallbacks(callbacks);

    return rc;
}

/*===========================================================================
 * FUNCTION   : open_legacy
 *
 * DESCRIPTION: Function to open older hal version implementation
 *
 * PARAMETERS :
 *   @hw_device : ptr to struct storing camera hardware device info
 *   @camera_id : camera ID
 *   @halVersion: Based on camera_module_t.common.module_api_version
 *
 * RETURN     : 0  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2Factory::open_legacy(const struct hw_module_t* module,
            const char* id, uint32_t halVersion, struct hw_device_t** device)
{
    int rc = NO_ERROR;
    if (module != &HAL_MODULE_INFO_SYM.common) {
        ALOGE("Invalid module. Trying to open %p, expect %p",
            module, &HAL_MODULE_INFO_SYM.common);
        return INVALID_OPERATION;
    }
    if (!id) {
        ALOGE("Invalid camera id");
        return BAD_VALUE;
    }
    if(gQCameraMuxer)
        rc =  gQCameraMuxer->open_legacy(module, id, halVersion, device);
    else
        rc =  gQCamera2Factory->openLegacy(atoi(id), halVersion, device);

    return rc;
}

/*===========================================================================
 * FUNCTION   : getNumberOfCameras
 *
 * DESCRIPTION: query number of cameras detected
 *
 * PARAMETERS : none
 *
 * RETURN     : number of cameras detected
 *==========================================================================*/
int QCamera2Factory::getNumberOfCameras()
{
    return mNumOfCameras;
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
int QCamera2Factory::getCameraInfo(int camera_id, struct camera_info *info)
{
    int rc;
    cam_sync_type_t cam_type = CAM_TYPE_MAIN;

    if (!mNumOfCameras || camera_id >= mNumOfCameras || !info ||
        (camera_id < 0)) {
        ALOGE("%s: Error getting camera info!! mNumOfCameras = %d,"
                "camera_id = %d, info = %p",
                __func__, mNumOfCameras, camera_id, info);
        return -ENODEV;
    }

    if ( NULL == mHalDescriptors ) {
        ALOGE("%s : Hal descriptor table is not initialized!", __func__);
        return NO_INIT;
    }

    if ( mHalDescriptors[camera_id].device_version ==
            CAMERA_DEVICE_API_VERSION_3_0 ) {
        rc = QCamera3HardwareInterface::getCamInfo(
                mHalDescriptors[camera_id].cameraId, info);
    } else if (mHalDescriptors[camera_id].device_version ==
            CAMERA_DEVICE_API_VERSION_1_0) {
        rc = QCamera2HardwareInterface::getCapabilities(
                mHalDescriptors[camera_id].cameraId, info, &cam_type);
    } else {
        ALOGE("%s: Device version for camera id %d invalid %d",
              __func__,
              camera_id,
              mHalDescriptors[camera_id].device_version);
        return BAD_VALUE;
    }

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
 * RETURN     :
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2Factory::setCallbacks(const camera_module_callbacks_t *callbacks)
{
    int rc = NO_ERROR;
    mCallbacks = callbacks;
    return rc;
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
int QCamera2Factory::cameraDeviceOpen(int camera_id,
                    struct hw_device_t **hw_device)
{
    int rc = NO_ERROR;
    if (camera_id < 0 || camera_id >= mNumOfCameras)
        return -ENODEV;

    if ( NULL == mHalDescriptors ) {
        ALOGE("%s : Hal descriptor table is not initialized!", __func__);
        return NO_INIT;
    }

    ALOGI("%s: Open camera id %d API version %d", __func__,
            camera_id, mHalDescriptors[camera_id].device_version);

    if ( mHalDescriptors[camera_id].device_version == CAMERA_DEVICE_API_VERSION_3_0 ) {
        QCamera3HardwareInterface *hw = new QCamera3HardwareInterface(mHalDescriptors[camera_id].cameraId,
                mCallbacks);
        if (!hw) {
            ALOGE("Allocation of hardware interface failed");
            return NO_MEMORY;
        }
        rc = hw->openCamera(hw_device);
        if (rc != 0) {
            delete hw;
        }
    } else if (mHalDescriptors[camera_id].device_version == CAMERA_DEVICE_API_VERSION_1_0) {
        QCamera2HardwareInterface *hw = new QCamera2HardwareInterface((uint32_t)camera_id);
        if (!hw) {
            ALOGE("Allocation of hardware interface failed");
            return NO_MEMORY;
        }
        rc = hw->openCamera(hw_device);
        if (rc != NO_ERROR) {
            delete hw;
        }
    } else {
        ALOGE("%s: Device version for camera id %d invalid %d",
              __func__,
              camera_id,
              mHalDescriptors[camera_id].device_version);
        return BAD_VALUE;
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : camera_device_open
 *
 * DESCRIPTION: static function to open a camera device by its ID
 *
 * PARAMETERS :
 *   @camera_id : camera ID
 *   @hw_device : ptr to struct storing camera hardware device info
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2Factory::camera_device_open(
    const struct hw_module_t *module, const char *id,
    struct hw_device_t **hw_device)
{
    int rc = NO_ERROR;
    if (module != &HAL_MODULE_INFO_SYM.common) {
        ALOGE("Invalid module. Trying to open %p, expect %p",
            module, &HAL_MODULE_INFO_SYM.common);
        return INVALID_OPERATION;
    }
    if (!id) {
        ALOGE("Invalid camera id");
        return BAD_VALUE;
    }

    if(gQCameraMuxer)
        rc =  gQCameraMuxer->camera_device_open(module, id, hw_device);
    else
        rc = gQCamera2Factory->cameraDeviceOpen(atoi(id), hw_device);

    return rc;
}

struct hw_module_methods_t QCamera2Factory::mModuleMethods = {
    open: QCamera2Factory::camera_device_open,
};

/*===========================================================================
 * FUNCTION   : openLegacy
 *
 * DESCRIPTION: Function to open older hal version implementation
 *
 * PARAMETERS :
 *   @camera_id : camera ID
 *   @halVersion: Based on camera_module_t.common.module_api_version
 *   @hw_device : ptr to struct storing camera hardware device info
 *
 * RETURN     : 0  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera2Factory::openLegacy(
        int32_t cameraId, uint32_t halVersion, struct hw_device_t** hw_device)
{
    int rc = NO_ERROR;

    ALOGI(":%s openLegacy halVersion: %d", __func__, halVersion);
    //Assumption: all cameras can support legacy API version
    if (cameraId < 0 || cameraId >= gQCamera2Factory->getNumberOfCameras())
        return -ENODEV;

    switch(halVersion)
    {
        case CAMERA_DEVICE_API_VERSION_1_0:
        {
            QCamera2HardwareInterface *hw =
                new QCamera2HardwareInterface((uint32_t)cameraId);
            if (!hw) {
                ALOGE("%s: Allocation of hardware interface failed", __func__);
                return NO_MEMORY;
            }
            rc = hw->openCamera(hw_device);
            if (rc != NO_ERROR) {
                delete hw;
            }
            break;
        }
        default:
            ALOGE("%s: Device API version: %d for camera id %d invalid",
                __func__, halVersion, cameraId);
            return BAD_VALUE;
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : isDualCamAvailable
 *
 * DESCRIPTION: Function to check whether we have dual Camera HW available
 *
 * PARAMETERS :
 *   @hal3Enabled : HAL3 enable flag
 *
 * RETURN     : bool - true : have Dual Camera HW available
 *                           false : not have Dual Camera HW available
 *==========================================================================*/
bool QCamera2Factory::isDualCamAvailable(int hal3Enabled)
{
    bool rc = FALSE;
    int i = 0;
    camera_info info;
    cam_sync_type_t cam_type = CAM_TYPE_MAIN;

    for (i = 0; i < mNumOfCameras; i++) {
        if (!hal3Enabled) {
            QCamera2HardwareInterface::getCapabilities(i, &info, &cam_type);
        }

        if(cam_type == CAM_TYPE_AUX) {
            CDBG_HIGH("%s: Have Dual Camera HW Avaiable.", __func__);
            rc = TRUE;
            break;
        }
    }

    return rc;
}

}; // namespace qcamera

