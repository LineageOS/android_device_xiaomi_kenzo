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

#define LOG_TAG "QCamera3HWI"
//#define LOG_NDEBUG 0

#define __STDC_LIMIT_MACROS
#include <cutils/properties.h>
#include <hardware/camera3.h>
#include <camera/CameraMetadata.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <stdint.h>
#include <utils/Log.h>
#include <utils/Errors.h>
#include <ui/Fence.h>
#include <gralloc_priv.h>
#include "QCamera3HWI.h"
#include "QCamera3Mem.h"
#include "QCamera3Channel.h"
#include "QCamera3PostProc.h"
#include "QCamera3VendorTags.h"
#include <cutils/properties.h>
#include <dlfcn.h>

#include <binder/Parcel.h>
#include <binder/IServiceManager.h>
#include <utils/RefBase.h>
#include <QServiceUtils.h>

using namespace android;

namespace qcamera {

#define DATA_PTR(MEM_OBJ,INDEX) MEM_OBJ->getPtr( INDEX )

#define TIME_SOURCE ANDROID_SENSOR_INFO_TIMESTAMP_SOURCE_UNKNOWN

#define EMPTY_PIPELINE_DELAY 2
#define PARTIAL_RESULT_COUNT 2
#define FRAME_SKIP_DELAY     0

#define MAX_VALUE_8BIT ((1<<8)-1)
#define MAX_VALUE_10BIT ((1<<10)-1)
#define MAX_VALUE_12BIT ((1<<12)-1)

#define VIDEO_4K_WIDTH  3840
#define VIDEO_4K_HEIGHT 2160

#define MAX_RAW_STREAMS        1
#define MAX_STALLING_STREAMS   1
#define MAX_PROCESSED_STREAMS  3
#define REGIONS_TUPLE_COUNT    5

#define FLUSH_TIMEOUT 3

#define METADATA_MAP_SIZE(MAP) (sizeof(MAP)/sizeof(MAP[0]))

#define CAM_QCOM_FEATURE_PP_SUPERSET_HAL3   ( CAM_QCOM_FEATURE_DENOISE2D |\
                                              CAM_QCOM_FEATURE_CROP |\
                                              CAM_QCOM_FEATURE_ROTATION |\
                                              CAM_QCOM_FEATURE_SHARPNESS |\
                                              CAM_QCOM_FEATURE_SCALE |\
                                              CAM_QCOM_FEATURE_CAC |\
                                              CAM_QCOM_FEATURE_CDS )

cam_capability_t *gCamCapability[MM_CAMERA_MAX_NUM_SENSORS];
const camera_metadata_t *gStaticMetadata[MM_CAMERA_MAX_NUM_SENSORS];
extern pthread_mutex_t gCamLock;
volatile uint32_t gCamHal3LogLevel = 0;
extern uint8_t gNumCameraSessions;

const QCamera3HardwareInterface::QCameraPropMap QCamera3HardwareInterface::CDS_MAP [] = {
    {"On",  CAM_CDS_MODE_ON},
    {"Off", CAM_CDS_MODE_OFF},
    {"Auto",CAM_CDS_MODE_AUTO}
};

const QCamera3HardwareInterface::QCameraMap<
        camera_metadata_enum_android_control_effect_mode_t,
        cam_effect_mode_type> QCamera3HardwareInterface::EFFECT_MODES_MAP[] = {
    { ANDROID_CONTROL_EFFECT_MODE_OFF,       CAM_EFFECT_MODE_OFF },
    { ANDROID_CONTROL_EFFECT_MODE_MONO,       CAM_EFFECT_MODE_MONO },
    { ANDROID_CONTROL_EFFECT_MODE_NEGATIVE,   CAM_EFFECT_MODE_NEGATIVE },
    { ANDROID_CONTROL_EFFECT_MODE_SOLARIZE,   CAM_EFFECT_MODE_SOLARIZE },
    { ANDROID_CONTROL_EFFECT_MODE_SEPIA,      CAM_EFFECT_MODE_SEPIA },
    { ANDROID_CONTROL_EFFECT_MODE_POSTERIZE,  CAM_EFFECT_MODE_POSTERIZE },
    { ANDROID_CONTROL_EFFECT_MODE_WHITEBOARD, CAM_EFFECT_MODE_WHITEBOARD },
    { ANDROID_CONTROL_EFFECT_MODE_BLACKBOARD, CAM_EFFECT_MODE_BLACKBOARD },
    { ANDROID_CONTROL_EFFECT_MODE_AQUA,       CAM_EFFECT_MODE_AQUA }
};

const QCamera3HardwareInterface::QCameraMap<
        camera_metadata_enum_android_control_awb_mode_t,
        cam_wb_mode_type> QCamera3HardwareInterface::WHITE_BALANCE_MODES_MAP[] = {
    { ANDROID_CONTROL_AWB_MODE_OFF,             CAM_WB_MODE_OFF },
    { ANDROID_CONTROL_AWB_MODE_AUTO,            CAM_WB_MODE_AUTO },
    { ANDROID_CONTROL_AWB_MODE_INCANDESCENT,    CAM_WB_MODE_INCANDESCENT },
    { ANDROID_CONTROL_AWB_MODE_FLUORESCENT,     CAM_WB_MODE_FLUORESCENT },
    { ANDROID_CONTROL_AWB_MODE_WARM_FLUORESCENT,CAM_WB_MODE_WARM_FLUORESCENT},
    { ANDROID_CONTROL_AWB_MODE_DAYLIGHT,        CAM_WB_MODE_DAYLIGHT },
    { ANDROID_CONTROL_AWB_MODE_CLOUDY_DAYLIGHT, CAM_WB_MODE_CLOUDY_DAYLIGHT },
    { ANDROID_CONTROL_AWB_MODE_TWILIGHT,        CAM_WB_MODE_TWILIGHT },
    { ANDROID_CONTROL_AWB_MODE_SHADE,           CAM_WB_MODE_SHADE }
};

const QCamera3HardwareInterface::QCameraMap<
        camera_metadata_enum_android_control_scene_mode_t,
        cam_scene_mode_type> QCamera3HardwareInterface::SCENE_MODES_MAP[] = {
    { ANDROID_CONTROL_SCENE_MODE_FACE_PRIORITY,  CAM_SCENE_MODE_FACE_PRIORITY },
    { ANDROID_CONTROL_SCENE_MODE_ACTION,         CAM_SCENE_MODE_ACTION },
    { ANDROID_CONTROL_SCENE_MODE_PORTRAIT,       CAM_SCENE_MODE_PORTRAIT },
    { ANDROID_CONTROL_SCENE_MODE_LANDSCAPE,      CAM_SCENE_MODE_LANDSCAPE },
    { ANDROID_CONTROL_SCENE_MODE_NIGHT,          CAM_SCENE_MODE_NIGHT },
    { ANDROID_CONTROL_SCENE_MODE_NIGHT_PORTRAIT, CAM_SCENE_MODE_NIGHT_PORTRAIT },
    { ANDROID_CONTROL_SCENE_MODE_THEATRE,        CAM_SCENE_MODE_THEATRE },
    { ANDROID_CONTROL_SCENE_MODE_BEACH,          CAM_SCENE_MODE_BEACH },
    { ANDROID_CONTROL_SCENE_MODE_SNOW,           CAM_SCENE_MODE_SNOW },
    { ANDROID_CONTROL_SCENE_MODE_SUNSET,         CAM_SCENE_MODE_SUNSET },
    { ANDROID_CONTROL_SCENE_MODE_STEADYPHOTO,    CAM_SCENE_MODE_ANTISHAKE },
    { ANDROID_CONTROL_SCENE_MODE_FIREWORKS ,     CAM_SCENE_MODE_FIREWORKS },
    { ANDROID_CONTROL_SCENE_MODE_SPORTS ,        CAM_SCENE_MODE_SPORTS },
    { ANDROID_CONTROL_SCENE_MODE_PARTY,          CAM_SCENE_MODE_PARTY },
    { ANDROID_CONTROL_SCENE_MODE_CANDLELIGHT,    CAM_SCENE_MODE_CANDLELIGHT },
    { ANDROID_CONTROL_SCENE_MODE_BARCODE,        CAM_SCENE_MODE_BARCODE}
};

const QCamera3HardwareInterface::QCameraMap<
        camera_metadata_enum_android_control_af_mode_t,
        cam_focus_mode_type> QCamera3HardwareInterface::FOCUS_MODES_MAP[] = {
    { ANDROID_CONTROL_AF_MODE_OFF,                CAM_FOCUS_MODE_OFF },
    { ANDROID_CONTROL_AF_MODE_OFF,                CAM_FOCUS_MODE_FIXED },
    { ANDROID_CONTROL_AF_MODE_AUTO,               CAM_FOCUS_MODE_AUTO },
    { ANDROID_CONTROL_AF_MODE_MACRO,              CAM_FOCUS_MODE_MACRO },
    { ANDROID_CONTROL_AF_MODE_EDOF,               CAM_FOCUS_MODE_EDOF },
    { ANDROID_CONTROL_AF_MODE_CONTINUOUS_PICTURE, CAM_FOCUS_MODE_CONTINOUS_PICTURE },
    { ANDROID_CONTROL_AF_MODE_CONTINUOUS_VIDEO,   CAM_FOCUS_MODE_CONTINOUS_VIDEO }
};

const QCamera3HardwareInterface::QCameraMap<
        camera_metadata_enum_android_color_correction_aberration_mode_t,
        cam_aberration_mode_t> QCamera3HardwareInterface::COLOR_ABERRATION_MAP[] = {
    { ANDROID_COLOR_CORRECTION_ABERRATION_MODE_OFF,
            CAM_COLOR_CORRECTION_ABERRATION_OFF },
    { ANDROID_COLOR_CORRECTION_ABERRATION_MODE_FAST,
            CAM_COLOR_CORRECTION_ABERRATION_FAST },
    { ANDROID_COLOR_CORRECTION_ABERRATION_MODE_HIGH_QUALITY,
            CAM_COLOR_CORRECTION_ABERRATION_HIGH_QUALITY },
};

const QCamera3HardwareInterface::QCameraMap<
        camera_metadata_enum_android_control_ae_antibanding_mode_t,
        cam_antibanding_mode_type> QCamera3HardwareInterface::ANTIBANDING_MODES_MAP[] = {
    { ANDROID_CONTROL_AE_ANTIBANDING_MODE_OFF,  CAM_ANTIBANDING_MODE_OFF },
    { ANDROID_CONTROL_AE_ANTIBANDING_MODE_50HZ, CAM_ANTIBANDING_MODE_50HZ },
    { ANDROID_CONTROL_AE_ANTIBANDING_MODE_60HZ, CAM_ANTIBANDING_MODE_60HZ },
    { ANDROID_CONTROL_AE_ANTIBANDING_MODE_AUTO, CAM_ANTIBANDING_MODE_AUTO }
};

const QCamera3HardwareInterface::QCameraMap<
        camera_metadata_enum_android_control_ae_mode_t,
        cam_flash_mode_t> QCamera3HardwareInterface::AE_FLASH_MODE_MAP[] = {
    { ANDROID_CONTROL_AE_MODE_OFF,                  CAM_FLASH_MODE_OFF },
    { ANDROID_CONTROL_AE_MODE_ON,                   CAM_FLASH_MODE_OFF },
    { ANDROID_CONTROL_AE_MODE_ON_AUTO_FLASH,        CAM_FLASH_MODE_AUTO},
    { ANDROID_CONTROL_AE_MODE_ON_ALWAYS_FLASH,      CAM_FLASH_MODE_ON  },
    { ANDROID_CONTROL_AE_MODE_ON_AUTO_FLASH_REDEYE, CAM_FLASH_MODE_AUTO}
};

const QCamera3HardwareInterface::QCameraMap<
        camera_metadata_enum_android_flash_mode_t,
        cam_flash_mode_t> QCamera3HardwareInterface::FLASH_MODES_MAP[] = {
    { ANDROID_FLASH_MODE_OFF,    CAM_FLASH_MODE_OFF  },
    { ANDROID_FLASH_MODE_SINGLE, CAM_FLASH_MODE_SINGLE },
    { ANDROID_FLASH_MODE_TORCH,  CAM_FLASH_MODE_TORCH }
};

const QCamera3HardwareInterface::QCameraMap<
        camera_metadata_enum_android_statistics_face_detect_mode_t,
        cam_face_detect_mode_t> QCamera3HardwareInterface::FACEDETECT_MODES_MAP[] = {
    { ANDROID_STATISTICS_FACE_DETECT_MODE_OFF,    CAM_FACE_DETECT_MODE_OFF     },
    { ANDROID_STATISTICS_FACE_DETECT_MODE_SIMPLE, CAM_FACE_DETECT_MODE_SIMPLE  },
    { ANDROID_STATISTICS_FACE_DETECT_MODE_FULL,   CAM_FACE_DETECT_MODE_FULL    }
};

const QCamera3HardwareInterface::QCameraMap<
        camera_metadata_enum_android_lens_info_focus_distance_calibration_t,
        cam_focus_calibration_t> QCamera3HardwareInterface::FOCUS_CALIBRATION_MAP[] = {
    { ANDROID_LENS_INFO_FOCUS_DISTANCE_CALIBRATION_UNCALIBRATED,
      CAM_FOCUS_UNCALIBRATED },
    { ANDROID_LENS_INFO_FOCUS_DISTANCE_CALIBRATION_APPROXIMATE,
      CAM_FOCUS_APPROXIMATE },
    { ANDROID_LENS_INFO_FOCUS_DISTANCE_CALIBRATION_CALIBRATED,
      CAM_FOCUS_CALIBRATED }
};

const QCamera3HardwareInterface::QCameraMap<
        camera_metadata_enum_android_lens_state_t,
        cam_af_lens_state_t> QCamera3HardwareInterface::LENS_STATE_MAP[] = {
    { ANDROID_LENS_STATE_STATIONARY,    CAM_AF_LENS_STATE_STATIONARY},
    { ANDROID_LENS_STATE_MOVING,        CAM_AF_LENS_STATE_MOVING}
};

const int32_t available_thumbnail_sizes[] = {0, 0,
                                             176, 144,
                                             320, 240,
                                             432, 288,
                                             480, 288,
                                             512, 288,
                                             512, 384};

const QCamera3HardwareInterface::QCameraMap<
        camera_metadata_enum_android_sensor_test_pattern_mode_t,
        cam_test_pattern_mode_t> QCamera3HardwareInterface::TEST_PATTERN_MAP[] = {
    { ANDROID_SENSOR_TEST_PATTERN_MODE_OFF,          CAM_TEST_PATTERN_OFF   },
    { ANDROID_SENSOR_TEST_PATTERN_MODE_SOLID_COLOR,  CAM_TEST_PATTERN_SOLID_COLOR },
    { ANDROID_SENSOR_TEST_PATTERN_MODE_COLOR_BARS,   CAM_TEST_PATTERN_COLOR_BARS },
    { ANDROID_SENSOR_TEST_PATTERN_MODE_COLOR_BARS_FADE_TO_GRAY, CAM_TEST_PATTERN_COLOR_BARS_FADE_TO_GRAY },
    { ANDROID_SENSOR_TEST_PATTERN_MODE_PN9,          CAM_TEST_PATTERN_PN9 },
};

/* Since there is no mapping for all the options some Android enum are not listed.
 * Also, the order in this list is important because while mapping from HAL to Android it will
 * traverse from lower to higher index which means that for HAL values that are map to different
 * Android values, the traverse logic will select the first one found.
 */
const QCamera3HardwareInterface::QCameraMap<
        camera_metadata_enum_android_sensor_reference_illuminant1_t,
        cam_illuminat_t> QCamera3HardwareInterface::REFERENCE_ILLUMINANT_MAP[] = {
    { ANDROID_SENSOR_REFERENCE_ILLUMINANT1_FLUORESCENT, CAM_AWB_WARM_FLO},
    { ANDROID_SENSOR_REFERENCE_ILLUMINANT1_DAYLIGHT_FLUORESCENT, CAM_AWB_CUSTOM_DAYLIGHT },
    { ANDROID_SENSOR_REFERENCE_ILLUMINANT1_COOL_WHITE_FLUORESCENT, CAM_AWB_COLD_FLO },
    { ANDROID_SENSOR_REFERENCE_ILLUMINANT1_STANDARD_A, CAM_AWB_A },
    { ANDROID_SENSOR_REFERENCE_ILLUMINANT1_D55, CAM_AWB_NOON },
    { ANDROID_SENSOR_REFERENCE_ILLUMINANT1_D65, CAM_AWB_D65 },
    { ANDROID_SENSOR_REFERENCE_ILLUMINANT1_D75, CAM_AWB_D75 },
    { ANDROID_SENSOR_REFERENCE_ILLUMINANT1_D50, CAM_AWB_D50 },
    { ANDROID_SENSOR_REFERENCE_ILLUMINANT1_ISO_STUDIO_TUNGSTEN, CAM_AWB_CUSTOM_A},
    { ANDROID_SENSOR_REFERENCE_ILLUMINANT1_DAYLIGHT, CAM_AWB_D50 },
    { ANDROID_SENSOR_REFERENCE_ILLUMINANT1_TUNGSTEN, CAM_AWB_A },
    { ANDROID_SENSOR_REFERENCE_ILLUMINANT1_FINE_WEATHER, CAM_AWB_D50 },
    { ANDROID_SENSOR_REFERENCE_ILLUMINANT1_CLOUDY_WEATHER, CAM_AWB_D65 },
    { ANDROID_SENSOR_REFERENCE_ILLUMINANT1_SHADE, CAM_AWB_D75 },
    { ANDROID_SENSOR_REFERENCE_ILLUMINANT1_DAY_WHITE_FLUORESCENT, CAM_AWB_CUSTOM_DAYLIGHT },
    { ANDROID_SENSOR_REFERENCE_ILLUMINANT1_WHITE_FLUORESCENT, CAM_AWB_COLD_FLO},
};

camera3_device_ops_t QCamera3HardwareInterface::mCameraOps = {
    initialize:                         QCamera3HardwareInterface::initialize,
    configure_streams:                  QCamera3HardwareInterface::configure_streams,
    register_stream_buffers:            NULL,
    construct_default_request_settings: QCamera3HardwareInterface::construct_default_request_settings,
    process_capture_request:            QCamera3HardwareInterface::process_capture_request,
    get_metadata_vendor_tag_ops:        NULL,
    dump:                               QCamera3HardwareInterface::dump,
    flush:                              QCamera3HardwareInterface::flush,
    reserved:                           {0},
};

/*===========================================================================
 * FUNCTION   : QCamera3HardwareInterface
 *
 * DESCRIPTION: constructor of QCamera3HardwareInterface
 *
 * PARAMETERS :
 *   @cameraId  : camera ID
 *
 * RETURN     : none
 *==========================================================================*/
QCamera3HardwareInterface::QCamera3HardwareInterface(uint32_t cameraId,
        const camera_module_callbacks_t *callbacks)
    : mCameraId(cameraId),
      mCameraHandle(NULL),
      mCameraOpened(false),
      mCameraInitialized(false),
      mCallbackOps(NULL),
      mInputStream(NULL),
      mMetadataChannel(NULL),
      mPictureChannel(NULL),
      mRawChannel(NULL),
      mSupportChannel(NULL),
      mAnalysisChannel(NULL),
      mRawDumpChannel(NULL),
      mFirstRequest(false),
      mFirstConfiguration(true),
      mFlush(false),
      mFlushPerf(false),
      mParamHeap(NULL),
      mParameters(NULL),
      mPrevParameters(NULL),
      m_bIsVideo(false),
      m_bIs4KVideo(false),
      m_bEisSupportedSize(false),
      m_bEisEnable(false),
      m_MobicatMask(0),
      mMinProcessedFrameDuration(0),
      mMinJpegFrameDuration(0),
      mMinRawFrameDuration(0),
      m_pPowerModule(NULL),
      mMetaFrameCount(0U),
      mUpdateDebugLevel(false),
      mCallbacks(callbacks),
      mCaptureIntent(0),
      mCacMode(0),
      mBootToMonoTimestampOffset(0)
{
    getLogLevel();
    mCameraDevice.common.tag = HARDWARE_DEVICE_TAG;
    mCameraDevice.common.version = CAMERA_DEVICE_API_VERSION_3_2;
    mCameraDevice.common.close = close_camera_device;
    mCameraDevice.ops = &mCameraOps;
    mCameraDevice.priv = this;
    gCamCapability[cameraId]->version = CAM_HAL_V3;
    // TODO: hardcode for now until mctl add support for min_num_pp_bufs
    //TBD - To see if this hardcoding is needed. Check by printing if this is filled by mctl to 3
    gCamCapability[cameraId]->min_num_pp_bufs = 3;

    pthread_cond_init(&mBuffersCond, NULL);

    pthread_cond_init(&mRequestCond, NULL);
    mPendingRequest = 0;
    mCurrentRequestId = -1;
    pthread_mutex_init(&mMutex, NULL);

    for (size_t i = 0; i < CAMERA3_TEMPLATE_COUNT; i++)
        mDefaultMetadata[i] = NULL;

#ifdef HAS_MULTIMEDIA_HINTS
    if (hw_get_module(POWER_HARDWARE_MODULE_ID, (const hw_module_t **)&m_pPowerModule)) {
        ALOGE("%s: %s module not found", __func__, POWER_HARDWARE_MODULE_ID);
    }
#endif

    char prop[PROPERTY_VALUE_MAX];
    memset(prop, 0, sizeof(prop));
    property_get("persist.camera.raw.dump", prop, "0");
    mEnableRawDump = atoi(prop);
    if (mEnableRawDump)
        CDBG("%s: Raw dump from Camera HAL enabled", __func__);

    memset(prop, 0, sizeof(prop));
    property_get("persist.camera.tnr.preview", prop, "0");
    m_bTnrEnabled = (uint8_t)atoi(prop);

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
    memset(&mFpsRange, 0, sizeof(cam_fps_range_t));
}

/*===========================================================================
 * FUNCTION   : ~QCamera3HardwareInterface
 *
 * DESCRIPTION: destructor of QCamera3HardwareInterface
 *
 * PARAMETERS : none
 *
 * RETURN     : none
 *==========================================================================*/
QCamera3HardwareInterface::~QCamera3HardwareInterface()
{
    CDBG("%s: E", __func__);
    /* We need to stop all streams before deleting any stream */


    if (mRawDumpChannel) {
        mRawDumpChannel->stop();
    }

    // NOTE: 'camera3_stream_t *' objects are already freed at
    //        this stage by the framework
    for (List<stream_info_t *>::iterator it = mStreamInfo.begin();
        it != mStreamInfo.end(); it++) {
        QCamera3Channel *channel = (*it)->channel;
        if (channel) {
            channel->stop();
        }
    }
    if (mSupportChannel)
        mSupportChannel->stop();

    if (mAnalysisChannel) {
        mAnalysisChannel->stop();
    }

    /* Turn off video hint */
    updatePowerHint(m_bIsVideo, false);

    for (List<stream_info_t *>::iterator it = mStreamInfo.begin();
        it != mStreamInfo.end(); it++) {
        QCamera3Channel *channel = (*it)->channel;
        if (channel)
            delete channel;
        free (*it);
    }
    if (mSupportChannel) {
        delete mSupportChannel;
        mSupportChannel = NULL;
    }

    if (mAnalysisChannel) {
        delete mAnalysisChannel;
        mAnalysisChannel = NULL;
    }
    if (mRawDumpChannel) {
        delete mRawDumpChannel;
        mRawDumpChannel = NULL;
    }
    mPictureChannel = NULL;

    /* Clean up all channels */
    if (mCameraInitialized) {
        if (mMetadataChannel) {
            mMetadataChannel->stop();
            delete mMetadataChannel;
            mMetadataChannel = NULL;
        }
        if(!mFirstConfiguration){
            //send the last unconfigure
            cam_stream_size_info_t stream_config_info;
            memset(&stream_config_info, 0, sizeof(cam_stream_size_info_t));
            stream_config_info.buffer_info.min_buffers = MIN_INFLIGHT_REQUESTS;
            stream_config_info.buffer_info.max_buffers = MAX_INFLIGHT_REQUESTS;
            clear_metadata_buffer(mParameters);
            ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_META_STREAM_INFO,
                    stream_config_info);
            int rc = mCameraHandle->ops->set_parms(mCameraHandle->camera_handle, mParameters);
            if (rc < 0) {
                ALOGE("%s: set_parms failed for unconfigure", __func__);
            }
        }
        deinitParameters();
    }

    if (mCameraOpened)
        closeCamera();

    mPendingBuffersMap.mPendingBufferList.clear();
    mPendingRequestsList.clear();
    mPendingReprocessResultList.clear();

    for (size_t i = 0; i < CAMERA3_TEMPLATE_COUNT; i++)
        if (mDefaultMetadata[i])
            free_camera_metadata(mDefaultMetadata[i]);

    pthread_cond_destroy(&mRequestCond);

    pthread_cond_destroy(&mBuffersCond);

    pthread_mutex_destroy(&mMutex);
    CDBG("%s: X", __func__);
}

/*===========================================================================
 * FUNCTION   : camEvtHandle
 *
 * DESCRIPTION: Function registered to mm-camera-interface to handle events
 *
 * PARAMETERS :
 *   @camera_handle : interface layer camera handle
 *   @evt           : ptr to event
 *   @user_data     : user data ptr
 *
 * RETURN     : none
 *==========================================================================*/
void QCamera3HardwareInterface::camEvtHandle(uint32_t /*camera_handle*/,
                                          mm_camera_event_t *evt,
                                          void *user_data)
{
    QCamera3HardwareInterface *obj = (QCamera3HardwareInterface *)user_data;
    if (obj && evt) {
        switch(evt->server_event_type) {
            case CAM_EVENT_TYPE_DAEMON_DIED:
                ALOGE("%s: Fatal, camera daemon died", __func__);
                //close the camera backend
                if (obj->mCameraHandle && obj->mCameraHandle->camera_handle
                        && obj->mCameraHandle->ops) {
                    obj->mCameraHandle->ops->error_close_camera(obj->mCameraHandle->camera_handle);
                } else {
                    ALOGE("%s: Could not close camera on error because the handle or ops is NULL",
                            __func__);
                }
                camera3_notify_msg_t notify_msg;
                memset(&notify_msg, 0, sizeof(camera3_notify_msg_t));
                notify_msg.type = CAMERA3_MSG_ERROR;
                notify_msg.message.error.error_code = CAMERA3_MSG_ERROR_DEVICE;
                notify_msg.message.error.error_stream = NULL;
                notify_msg.message.error.frame_number = 0;
                obj->mCallbackOps->notify(obj->mCallbackOps, &notify_msg);
                break;

            case CAM_EVENT_TYPE_DAEMON_PULL_REQ:
                CDBG("%s: HAL got request pull from Daemon", __func__);
                pthread_mutex_lock(&obj->mMutex);
                obj->mWokenUpByDaemon = true;
                obj->unblockRequestIfNecessary();
                pthread_mutex_unlock(&obj->mMutex);
                break;

            default:
                CDBG_HIGH("%s: Warning: Unhandled event %d", __func__,
                        evt->server_event_type);
                break;
        }
    } else {
        ALOGE("%s: NULL user_data/evt", __func__);
    }
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
int QCamera3HardwareInterface::openCamera(struct hw_device_t **hw_device)
{
    int rc = 0;
    if (mCameraOpened) {
        *hw_device = NULL;
        return PERMISSION_DENIED;
    }
    ALOGI("[KPI Perf] %s: E PROFILE_OPEN_CAMERA camera id %d",
            __func__, mCameraId);

    rc = openCamera();
    if (rc == 0) {
        *hw_device = &mCameraDevice.common;
    } else
        *hw_device = NULL;

    ALOGI("[KPI Perf] %s: X PROFILE_OPEN_CAMERA camera id %d, rc: %d",
            __func__, mCameraId, rc);

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
int QCamera3HardwareInterface::openCamera()
{
    int rc = 0;
    char value[PROPERTY_VALUE_MAX];

    ATRACE_CALL();
    if (mCameraHandle) {
        ALOGE("Failure: Camera already opened");
        return ALREADY_EXISTS;
    }
    rc = camera_open((uint8_t)mCameraId, &mCameraHandle);
    if (rc) {
        ALOGE("camera_open failed. rc = %d, mCameraHandle = %p", rc, mCameraHandle);
        return rc;
    }
    if (!mCameraHandle) {
        ALOGE("camera_open failed. mCameraHandle = %p", mCameraHandle);
        return -ENODEV;
    }

    mCameraOpened = true;

    rc = mCameraHandle->ops->register_event_notify(mCameraHandle->camera_handle,
            camEvtHandle, (void *)this);

    if (rc < 0) {
        ALOGE("%s: Error, failed to register event callback", __func__);
        /* Not closing camera here since it is already handled in destructor */
        return FAILED_TRANSACTION;
    }

    mExifParams.debug_params =
            (mm_jpeg_debug_exif_params_t *) malloc (sizeof(mm_jpeg_debug_exif_params_t));
    if (mExifParams.debug_params) {
        memset(mExifParams.debug_params, 0, sizeof(mm_jpeg_debug_exif_params_t));
    } else {
        ALOGE("%s: Out of Memory. Allocation failed for 3A debug exif params", __func__);
        return NO_MEMORY;
    }
    mFirstConfiguration = true;

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
        // for display is MONOTONIC
        // The below offset is used to convert from clock domain of other subsystem
        // (hardware composer) to that of camera. Assumption is that this
        // offset won't change during the life cycle of the camera device. In other
        // words, camera device shouldn't be open during CPU suspend.
        mBootToMonoTimestampOffset = getBootToMonoTimeOffset();
    }
    CDBG_HIGH("mBootToMonoTimestampOffset = %lld", mBootToMonoTimestampOffset);

    return NO_ERROR;
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
int QCamera3HardwareInterface::closeCamera()
{
    ATRACE_CALL();
    int rc = NO_ERROR;
    char value[PROPERTY_VALUE_MAX];

    rc = mCameraHandle->ops->close_camera(mCameraHandle->camera_handle);
    mCameraHandle = NULL;
    mCameraOpened = false;

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

    return rc;
}

/*===========================================================================
 * FUNCTION   : initialize
 *
 * DESCRIPTION: Initialize frameworks callback functions
 *
 * PARAMETERS :
 *   @callback_ops : callback function to frameworks
 *
 * RETURN     :
 *
 *==========================================================================*/
int QCamera3HardwareInterface::initialize(
        const struct camera3_callback_ops *callback_ops)
{
    ATRACE_CALL();
    int rc;

    pthread_mutex_lock(&mMutex);

    rc = initParameters();
    if (rc < 0) {
        ALOGE("%s: initParamters failed %d", __func__, rc);
       goto err1;
    }
    mCallbackOps = callback_ops;

    pthread_mutex_unlock(&mMutex);
    mCameraInitialized = true;
    return 0;

err1:
    pthread_mutex_unlock(&mMutex);
    return rc;
}

/*===========================================================================
 * FUNCTION   : validateStreamDimensions
 *
 * DESCRIPTION: Check if the configuration requested are those advertised
 *
 * PARAMETERS :
 *   @stream_list : streams to be configured
 *
 * RETURN     :
 *
 *==========================================================================*/
int QCamera3HardwareInterface::validateStreamDimensions(
        camera3_stream_configuration_t *streamList)
{
    int rc = NO_ERROR;
    int32_t available_processed_sizes[MAX_SIZES_CNT * 2];
    size_t count = 0;

    /*
    * Loop through all streams requested in configuration
    * Check if unsupported sizes have been requested on any of them
    */
    for (size_t j = 0; j < streamList->num_streams; j++) {
        bool sizeFound = false;
        size_t jpeg_sizes_cnt = 0;
        camera3_stream_t *newStream = streamList->streams[j];

        /*
        * Sizes are different for each type of stream format check against
        * appropriate table.
        */
        switch (newStream->format) {
        case ANDROID_SCALER_AVAILABLE_FORMATS_RAW16:
        case ANDROID_SCALER_AVAILABLE_FORMATS_RAW_OPAQUE:
        case HAL_PIXEL_FORMAT_RAW10:
            count = MIN(gCamCapability[mCameraId]->supported_raw_dim_cnt, MAX_SIZES_CNT);
            for (size_t i = 0; i < count; i++) {
                if ((gCamCapability[mCameraId]->raw_dim[i].width == (int32_t)newStream->width) &&
                        (gCamCapability[mCameraId]->raw_dim[i].height == (int32_t)newStream->height)) {
                    sizeFound = true;
                    break;
                }
            }
            break;
        case HAL_PIXEL_FORMAT_BLOB:
            count = MIN(gCamCapability[mCameraId]->picture_sizes_tbl_cnt, MAX_SIZES_CNT);
            /* Verify set size against generated sizes table */
            for (size_t i = 0; i < count; i++) {
                if (((int32_t)newStream->width ==
                            gCamCapability[mCameraId]->picture_sizes_tbl[i].width) &&
                        ((int32_t)newStream->height ==
                                gCamCapability[mCameraId]->picture_sizes_tbl[i].height)) {
                    sizeFound = true;
                    break;
                }
            }
            break;


        case HAL_PIXEL_FORMAT_YCbCr_420_888:
        case HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED:
        default:
            /* ZSL stream will be full active array size validate that*/
            if (newStream->stream_type == CAMERA3_STREAM_BIDIRECTIONAL) {
                if (((int32_t)newStream->width ==
                            gCamCapability[mCameraId]->active_array_size.width) &&
                        ((int32_t)newStream->height ==
                                gCamCapability[mCameraId]->active_array_size.height)) {
                    sizeFound = true;
                }
                /* We could potentially break here to enforce ZSL stream
                 * set from frameworks always has full active array size
                 * but it is not clear from spec if framework will always
                 * follow that, also we have logic to override to full array
                 * size, so keeping this logic lenient at the moment.
                 */
            }

            /* Non ZSL stream still need to conform to advertised sizes*/
            count = MIN(gCamCapability[mCameraId]->picture_sizes_tbl_cnt,
                    MAX_SIZES_CNT);
            for (size_t i = 0; i < count; i++) {
                if (((int32_t)newStream->width ==
                            gCamCapability[mCameraId]->picture_sizes_tbl[i].width) &&
                        ((int32_t)newStream->height ==
                                gCamCapability[mCameraId]->picture_sizes_tbl[i].height)) {
                    sizeFound = true;
                break;
                }
            }
            break;
        } /* End of switch(newStream->format) */

        /* We error out even if a single stream has unsupported size set */
        if (!sizeFound) {
            ALOGE("%s: Error: Unsupported size of %d x %d requested for stream type:%d",
                    __func__, newStream->width, newStream->height, newStream->format);
            rc = -EINVAL;
            break;
        }
    } /* End of for each stream */
    return rc;
}

/*==============================================================================
 * FUNCTION   : isSupportChannelNeeded
 *
 * DESCRIPTION: Simple heuristic func to determine if support channels is needed
 *
 * PARAMETERS :
 *   @stream_list : streams to be configured
 *
 * RETURN     : Boolen true/false decision
 *
 *==========================================================================*/
bool QCamera3HardwareInterface::isSupportChannelNeeded(camera3_stream_configuration_t *streamList)
{
    uint32_t i;

    /* Dummy stream needed if only raw or jpeg streams present */
    for (i = 0;i < streamList->num_streams;i++) {
        switch(streamList->streams[i]->format) {
            case HAL_PIXEL_FORMAT_RAW_OPAQUE:
            case HAL_PIXEL_FORMAT_RAW10:
            case HAL_PIXEL_FORMAT_RAW16:
            case HAL_PIXEL_FORMAT_BLOB:
                break;
            default:
                return false;
        }
    }
    return true;
}

/*==============================================================================
 * FUNCTION   : getSensorOutputSize
 *
 * DESCRIPTION: Get sensor output size based on current stream configuratoin
 *
 * PARAMETERS :
 *   @sensor_dim : sensor output dimension (output)
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *
 *==========================================================================*/
int32_t QCamera3HardwareInterface::getSensorOutputSize(cam_dimension_t &sensor_dim,
                                                             CameraMetadata *frame_settings)
{
    int32_t rc = NO_ERROR;

    cam_dimension_t max_dim = {0, 0};
    for (uint32_t i = 0; i < mStreamConfigInfo.num_streams; i++) {
        if (mStreamConfigInfo.stream_sizes[i].width > max_dim.width)
            max_dim.width = mStreamConfigInfo.stream_sizes[i].width;
        if (mStreamConfigInfo.stream_sizes[i].height > max_dim.height)
            max_dim.height = mStreamConfigInfo.stream_sizes[i].height;
    }

    clear_metadata_buffer(mParameters);

    if (frame_settings) {
        rc = setHalFpsRange(*frame_settings, mParameters);
        if (rc != NO_ERROR) {
            ALOGE("%s: setHalFpsRange failed", __func__);
        }
    }

    rc = ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_PARM_MAX_DIMENSION,
            max_dim);
    if (rc != NO_ERROR) {
        ALOGE("%s:Failed to update table for CAM_INTF_PARM_MAX_DIMENSION", __func__);
        return rc;
    }

    rc = mCameraHandle->ops->set_parms(mCameraHandle->camera_handle, mParameters);
    if (rc != NO_ERROR) {
        ALOGE("%s: Failed to set CAM_INTF_PARM_MAX_DIMENSION", __func__);
        return rc;
    }

    clear_metadata_buffer(mParameters);
    ADD_GET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_PARM_RAW_DIMENSION);

    rc = mCameraHandle->ops->get_parms(mCameraHandle->camera_handle,
            mParameters);
    if (rc != NO_ERROR) {
        ALOGE("%s: Failed to get CAM_INTF_PARM_RAW_DIMENSION", __func__);
        return rc;
    }

    READ_PARAM_ENTRY(mParameters, CAM_INTF_PARM_RAW_DIMENSION, sensor_dim);
    ALOGI("%s: sensor output dimension = %d x %d", __func__, sensor_dim.width, sensor_dim.height);

    return rc;
}

/*==============================================================================
 * FUNCTION   : updatePowerHint
 *
 * DESCRIPTION: update power hint based on whether it's video mode or not.
 *
 * PARAMETERS :
 *   @bWasVideo : whether video mode before the switch
 *   @bIsVideo  : whether new mode is video or not.
 *
 * RETURN     : NULL
 *
 *==========================================================================*/
void QCamera3HardwareInterface::updatePowerHint(bool bWasVideo, bool bIsVideo)
{
#ifdef HAS_MULTIMEDIA_HINTS
    if (bWasVideo == bIsVideo)
        return;

    if (m_pPowerModule && m_pPowerModule->powerHint) {
        if (bIsVideo)
            m_pPowerModule->powerHint(m_pPowerModule,
                    POWER_HINT_VIDEO_ENCODE, (void *)"state=1");
        else
            m_pPowerModule->powerHint(m_pPowerModule,
                    POWER_HINT_VIDEO_ENCODE, (void *)"state=0");
     }
#endif
}

/*===========================================================================
 * FUNCTION   : configureStreams
 *
 * DESCRIPTION: Reset HAL camera device processing pipeline and set up new input
 *              and output streams.
 *
 * PARAMETERS :
 *   @stream_list : streams to be configured
 *
 * RETURN     :
 *
 *==========================================================================*/
int QCamera3HardwareInterface::configureStreams(
        camera3_stream_configuration_t *streamList)
{
    ATRACE_CALL();
    int rc = 0;
    bool bWasVideo = m_bIsVideo;

    // Sanity check stream_list
    if (streamList == NULL) {
        ALOGE("%s: NULL stream configuration", __func__);
        return BAD_VALUE;
    }
    if (streamList->streams == NULL) {
        ALOGE("%s: NULL stream list", __func__);
        return BAD_VALUE;
    }

    if (streamList->num_streams < 1) {
        ALOGE("%s: Bad number of streams requested: %d", __func__,
                streamList->num_streams);
        return BAD_VALUE;
    }

    if (streamList->num_streams >= MAX_NUM_STREAMS) {
        ALOGE("%s: Maximum number of streams %d exceeded: %d", __func__,
                MAX_NUM_STREAMS, streamList->num_streams);
        return BAD_VALUE;
    }

    /* first invalidate all the steams in the mStreamList
     * if they appear again, they will be validated */
    for (List<stream_info_t*>::iterator it = mStreamInfo.begin();
            it != mStreamInfo.end(); it++) {
        QCamera3Channel *channel = (QCamera3Channel*)(*it)->stream->priv;
        channel->stop();
        (*it)->status = INVALID;
    }

    if (mRawDumpChannel) {
        mRawDumpChannel->stop();
        delete mRawDumpChannel;
        mRawDumpChannel = NULL;
    }

    if (mSupportChannel)
        mSupportChannel->stop();

    if (mAnalysisChannel) {
        mAnalysisChannel->stop();
    }
    if (mMetadataChannel) {
        /* If content of mStreamInfo is not 0, there is metadata stream */
        mMetadataChannel->stop();
    }

    pthread_mutex_lock(&mMutex);

    /* Check whether we have video stream */
    m_bIs4KVideo = false;
    m_bIsVideo = false;
    m_bEisSupportedSize = false;
    bool isZsl = false;
    uint32_t videoWidth = 0U;
    uint32_t videoHeight = 0U;
    size_t rawStreamCnt = 0;
    size_t stallStreamCnt = 0;
    size_t processedStreamCnt = 0;
    // Number of streams on ISP encoder path
    size_t numStreamsOnEncoder = 0;
    cam_dimension_t maxViewfinderSize;
    bool bJpegExceeds4K = false;
    bool bUseCommonFeatureMask = false;
    bool bSmallJpegSize = false;
    uint8_t maxDownscaleFactor = 1;
    uint32_t commonFeatureMask = 0;

    //@todo Remove fullFeatureMask and possibly m_bTnrEnabled once CPP checks
    //      both feature mask and param for TNR enable.
    uint32_t fullFeatureMask = CAM_QCOM_FEATURE_PP_SUPERSET_HAL3;
    if (gCamCapability[mCameraId]->qcom_supported_feature_mask
            & CAM_QCOM_FEATURE_DSDN) {
        //Use CPP CDS incase h/w supports it.
        fullFeatureMask &= ~CAM_QCOM_FEATURE_CDS;
        fullFeatureMask |= CAM_QCOM_FEATURE_DSDN;
    }
    if (m_bTnrEnabled != 0)
    {
        fullFeatureMask |= CAM_QCOM_FEATURE_CPP_TNR;
        // TNR and CDS cannot be enabled at the same time. Unmask CDS feature.
        fullFeatureMask &= ~CAM_QCOM_FEATURE_CDS;
        fullFeatureMask &= ~CAM_QCOM_FEATURE_DSDN;
    }
    maxViewfinderSize = gCamCapability[mCameraId]->max_viewfinder_size;

    cam_padding_info_t padding_info = gCamCapability[mCameraId]->padding_info;

    uint32_t minWidth;
    uint32_t minHeight;
    /*EIS configuration*/
    bool eisSupported = false;
    bool oisSupported = false;
    int32_t margin_index = -1;
    uint8_t eis_prop_set;
    uint32_t maxEisWidth = 0;
    uint32_t maxEisHeight = 0;
    int32_t hal_version = CAM_HAL_V3;

    size_t count = IS_TYPE_MAX;
    count = MIN(gCamCapability[mCameraId]->supported_is_types_cnt, count);
    for (size_t i = 0; i < count; i++) {
        if (gCamCapability[mCameraId]->supported_is_types[i] == IS_TYPE_EIS_2_0) {
            eisSupported = true;
            margin_index = (int32_t)i;
            break;
        }
    }

    count = CAM_OPT_STAB_MAX;
    count = MIN(gCamCapability[mCameraId]->optical_stab_modes_count, count);
    for (size_t i = 0; i < count; i++) {
        if (gCamCapability[mCameraId]->optical_stab_modes[i] ==  CAM_OPT_STAB_ON) {
            oisSupported = true;
            break;
        }
    }

    if (eisSupported) {
        maxEisWidth = (uint32_t)
            ((gCamCapability[mCameraId]->active_array_size.width * 1.0) /
            (1+ gCamCapability[mCameraId]->supported_is_type_margins[margin_index]));
         maxEisHeight = (uint32_t)
            ((gCamCapability[mCameraId]->active_array_size.height * 1.0) /
            (1+ gCamCapability[mCameraId]->supported_is_type_margins[margin_index]));
    }

    /* EIS setprop control */
    char eis_prop[PROPERTY_VALUE_MAX];
    memset(eis_prop, 0, sizeof(eis_prop));
    property_get("camera.eis.enable", eis_prop, "0");
    eis_prop_set = (uint8_t)atoi(eis_prop);

    m_bEisEnable = eis_prop_set && (!oisSupported && eisSupported);

    /* stream configurations */
    for (size_t i = 0; i < streamList->num_streams; i++) {
        camera3_stream_t *newStream = streamList->streams[i];
        ALOGI("%s: stream[%d] type = %d, format = %d, width = %d, height = %d",
                __func__, i, newStream->stream_type, newStream->format,
                newStream->width, newStream->height);
        if (newStream->stream_type == CAMERA3_STREAM_BIDIRECTIONAL &&
                newStream->format == HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED){
            isZsl = true;
        }
        if (newStream->format == HAL_PIXEL_FORMAT_BLOB) {
            if (newStream->width > VIDEO_4K_WIDTH ||
                    newStream->height > VIDEO_4K_HEIGHT)
                bJpegExceeds4K = true;
        }

        if ((HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED == newStream->format) &&
                (newStream->usage & private_handle_t::PRIV_FLAGS_VIDEO_ENCODER)) {
            m_bIsVideo = true;
            if ((VIDEO_4K_WIDTH <= newStream->width) &&
                    (VIDEO_4K_HEIGHT <= newStream->height)) {
                videoWidth = newStream->width;
                videoHeight = newStream->height;
                m_bIs4KVideo = true;
            }
            m_bEisSupportedSize = (newStream->width <= maxEisWidth) &&
                                  (newStream->height <= maxEisHeight);
        }
        if (newStream->stream_type == CAMERA3_STREAM_BIDIRECTIONAL ||
                newStream->stream_type == CAMERA3_STREAM_OUTPUT) {
            switch (newStream->format) {
            case HAL_PIXEL_FORMAT_BLOB:
                stallStreamCnt++;
                if (((int32_t)newStream->width > maxViewfinderSize.width) ||
                        ((int32_t)newStream->height > maxViewfinderSize.height)) {
                    commonFeatureMask |= CAM_QCOM_FEATURE_NONE;
                    numStreamsOnEncoder++;
                }
                assert(gCamCapability[mCameraId]->max_downscale_factor > 0);
                maxDownscaleFactor = gCamCapability[mCameraId]->max_downscale_factor;
                minWidth = gCamCapability[mCameraId]->active_array_size.width/maxDownscaleFactor;
                minHeight = gCamCapability[mCameraId]->active_array_size.height/maxDownscaleFactor;
                // Set bSmallJpegSize to link the cpp if new resolution is < VFE downscale
                if ( (newStream->width < minWidth) ||
                        (newStream->height < minHeight)) {
                    CDBG("%s: Setting small jpeg size to true", __func__);
                    bSmallJpegSize = true;
                }
                break;
            case HAL_PIXEL_FORMAT_RAW10:
            case HAL_PIXEL_FORMAT_RAW_OPAQUE:
            case HAL_PIXEL_FORMAT_RAW16:
                rawStreamCnt++;
                break;
            case HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED:
                processedStreamCnt++;
                if (((int32_t)newStream->width > maxViewfinderSize.width) ||
                        ((int32_t)newStream->height > maxViewfinderSize.height)) {
                    if (newStream->stream_type == CAMERA3_STREAM_BIDIRECTIONAL) {
                        commonFeatureMask |= CAM_QCOM_FEATURE_NONE;
                    } else {
                        commonFeatureMask |= fullFeatureMask;
                    }
                    numStreamsOnEncoder++;
                }
                break;
            case HAL_PIXEL_FORMAT_YCbCr_420_888:
            default:
                processedStreamCnt++;
                if (((int32_t)newStream->width > maxViewfinderSize.width) ||
                        ((int32_t)newStream->height > maxViewfinderSize.height)) {
                    commonFeatureMask |= fullFeatureMask;
                    numStreamsOnEncoder++;
                }
                break;
            }

        }
    }

    CDBG_HIGH("%s: max viewfinder width %d height %d isZsl %d",  __func__, maxViewfinderSize.width,
            maxViewfinderSize.height, isZsl);
    CDBG_HIGH("%s: numStreamsOnEncoder %d, processedStreamCnt %d, stallcnt %d bSmallJpegSize %d",
            __func__, numStreamsOnEncoder, processedStreamCnt, stallStreamCnt, bSmallJpegSize);

    /* Check if num_streams is sane */
    if (stallStreamCnt > MAX_STALLING_STREAMS ||
            rawStreamCnt > MAX_RAW_STREAMS ||
            processedStreamCnt > MAX_PROCESSED_STREAMS) {
        ALOGE("%s: Invalid stream configu: stall: %d, raw: %d, processed %d",
                __func__, stallStreamCnt, rawStreamCnt, processedStreamCnt);
        pthread_mutex_unlock(&mMutex);
        return -EINVAL;
    }
    /* Check whether we have zsl stream or 4k video case */
    if (isZsl && m_bIsVideo) {
        ALOGE("%s: Currently invalid configuration ZSL&Video!", __func__);
        pthread_mutex_unlock(&mMutex);
        return -EINVAL;
    }
    /* Check if stream sizes are sane */
    if (numStreamsOnEncoder > 2) {
        ALOGE("%s: Number of streams on ISP encoder path exceeds limits of 2",
                __func__);
        pthread_mutex_unlock(&mMutex);
        return -EINVAL;
    } else if (1 < numStreamsOnEncoder){
        bUseCommonFeatureMask = true;
        CDBG_HIGH("%s: Multiple streams above max viewfinder size, common mask needed",
                __func__);
    }
    /* Check if BLOB size is greater than 4k in 4k recording case */
    if (m_bIs4KVideo && bJpegExceeds4K) {
        ALOGE("%s: HAL doesn't support Blob size greater than 4k in 4k recording",
                __func__);
        pthread_mutex_unlock(&mMutex);
        return -EINVAL;
    }

    rc = validateStreamDimensions(streamList);
    if (rc != NO_ERROR) {
        ALOGE("%s: Invalid stream configuration requested!", __func__);
        pthread_mutex_unlock(&mMutex);
        return rc;
    }

    camera3_stream_t *inputStream = NULL;
    camera3_stream_t *jpegStream = NULL;
    for (size_t i = 0; i < streamList->num_streams; i++) {
        camera3_stream_t *newStream = streamList->streams[i];
        CDBG_HIGH("%s: newStream type = %d, stream format = %d stream size : %d x %d",
                __func__, newStream->stream_type, newStream->format,
                 newStream->width, newStream->height);
        //if the stream is in the mStreamList validate it
        bool stream_exists = false;
        for (List<stream_info_t*>::iterator it=mStreamInfo.begin();
                it != mStreamInfo.end(); it++) {
            if ((*it)->stream == newStream) {
                QCamera3Channel *channel =
                    (QCamera3Channel*)(*it)->stream->priv;
                stream_exists = true;
                delete channel;
                (*it)->status = VALID;
                (*it)->stream->priv = NULL;
                (*it)->channel = NULL;
            }
        }
        if (!stream_exists) {
            //new stream
            stream_info_t* stream_info;
            stream_info = (stream_info_t* )malloc(sizeof(stream_info_t));
            if (!stream_info) {
               ALOGE("%s: Could not allocate stream info", __func__);
               rc = -ENOMEM;
               pthread_mutex_unlock(&mMutex);
               return rc;
            }
            stream_info->stream = newStream;
            stream_info->status = VALID;
            stream_info->channel = NULL;
            mStreamInfo.push_back(stream_info);
        }
        if (newStream->stream_type == CAMERA3_STREAM_INPUT
                || newStream->stream_type == CAMERA3_STREAM_BIDIRECTIONAL ) {
            if (inputStream != NULL) {
                ALOGE("%s: Multiple input streams requested!", __func__);
                pthread_mutex_unlock(&mMutex);
                return BAD_VALUE;
            }
            inputStream = newStream;
        }
        if (newStream->format == HAL_PIXEL_FORMAT_BLOB) {
            jpegStream = newStream;
        }
    }
    mInputStream = inputStream;

    cleanAndSortStreamInfo();
    if (mMetadataChannel) {
        delete mMetadataChannel;
        mMetadataChannel = NULL;
    }
    if (mSupportChannel) {
        delete mSupportChannel;
        mSupportChannel = NULL;
    }

    if (mAnalysisChannel) {
        delete mAnalysisChannel;
        mAnalysisChannel = NULL;
    }

    //Create metadata channel and initialize it
    mMetadataChannel = new QCamera3MetadataChannel(mCameraHandle->camera_handle,
                    mCameraHandle->ops, captureResultCb,
                    &padding_info, CAM_QCOM_FEATURE_NONE, this);
    if (mMetadataChannel == NULL) {
        ALOGE("%s: failed to allocate metadata channel", __func__);
        rc = -ENOMEM;
        pthread_mutex_unlock(&mMutex);
        return rc;
    }
    rc = mMetadataChannel->initialize(IS_TYPE_NONE);
    if (rc < 0) {
        ALOGE("%s: metadata channel initialization failed", __func__);
        delete mMetadataChannel;
        mMetadataChannel = NULL;
        pthread_mutex_unlock(&mMutex);
        return rc;
    }

    /* Create analysis stream to support FD */
    mAnalysisChannel = new QCamera3SupportChannel(
            mCameraHandle->camera_handle,
            mCameraHandle->ops,
            &gCamCapability[mCameraId]->analysis_padding_info,
            fullFeatureMask,
            CAM_STREAM_TYPE_ANALYSIS,
            &gCamCapability[mCameraId]->analysis_recommended_res,
            (gCamCapability[mCameraId]->analysis_recommended_format
            == CAM_FORMAT_Y_ONLY ? CAM_FORMAT_Y_ONLY
            : CAM_FORMAT_YUV_420_NV21),
            this);
    if (!mAnalysisChannel) {
        ALOGE("%s: Analysis channel cannot be created", __func__);
        pthread_mutex_unlock(&mMutex);
        return -ENOMEM;
    }

    if (isSupportChannelNeeded(streamList)) {
        mSupportChannel = new QCamera3SupportChannel(
                mCameraHandle->camera_handle,
                mCameraHandle->ops,
                &padding_info,
                fullFeatureMask,
                CAM_STREAM_TYPE_CALLBACK,
                &QCamera3SupportChannel::kDim,
                CAM_FORMAT_YUV_420_NV21,
                this);
        if (!mSupportChannel) {
            ALOGE("%s: dummy channel cannot be created", __func__);
            pthread_mutex_unlock(&mMutex);
            return -ENOMEM;
        }
    }

    bool isRawStreamRequested = false;
    memset(&mStreamConfigInfo, 0, sizeof(cam_stream_size_info_t));
    /* Allocate channel objects for the requested streams */
    for (size_t i = 0; i < streamList->num_streams; i++) {
        camera3_stream_t *newStream = streamList->streams[i];
        uint32_t stream_usage = newStream->usage;
        mStreamConfigInfo.stream_sizes[i].width = (int32_t)newStream->width;
        mStreamConfigInfo.stream_sizes[i].height = (int32_t)newStream->height;
        if (newStream->stream_type == CAMERA3_STREAM_BIDIRECTIONAL &&
            newStream->format == HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED && jpegStream){
            //for zsl stream the size is jpeg stream size
            mStreamConfigInfo.stream_sizes[i].width = (int32_t)jpegStream->width;
            mStreamConfigInfo.stream_sizes[i].height = (int32_t)jpegStream->height;
            mStreamConfigInfo.type[i] = CAM_STREAM_TYPE_SNAPSHOT;
            // link the cpp if below conditions are met
            if (bSmallJpegSize) {
                mStreamConfigInfo.postprocess_mask[i] = fullFeatureMask;
                mStreamConfigInfo.postprocess_mask[i] &= ~CAM_QCOM_FEATURE_CDS;
            } else {
                mStreamConfigInfo.postprocess_mask[i] = CAM_QCOM_FEATURE_NONE;
            }
        } else {
            //for non zsl streams find out the format
            switch (newStream->format) {
            case HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED :
            {
                char feature_mask_value[PROPERTY_VALUE_MAX];
                uint32_t feature_mask;
                int args_converted;
                int property_len;

                property_len = property_get("persist.camera.hal3.prv.feature",
                        feature_mask_value, "0");
                if ((property_len > 2) && (feature_mask_value[0] == '0') &&
                        (feature_mask_value[1] == 'x')) {
                    args_converted = sscanf(feature_mask_value, "0x%x", &feature_mask);
                } else {
                    args_converted = sscanf(feature_mask_value, "%d", &feature_mask);
                }
                if (1 != args_converted) {
                    feature_mask = 0;
                    ALOGE("%s: Wrong feature mask setting: %s", __func__, feature_mask_value);
                }

                if (stream_usage & private_handle_t::PRIV_FLAGS_VIDEO_ENCODER) {
                    mStreamConfigInfo.type[i] = CAM_STREAM_TYPE_VIDEO;
                } else {
                    mStreamConfigInfo.type[i] = CAM_STREAM_TYPE_PREVIEW;
                    padding_info.width_padding = mSurfaceStridePadding;
                    padding_info.height_padding = CAM_PAD_TO_2;
                }
                mStreamConfigInfo.postprocess_mask[i] = fullFeatureMask;
                mStreamConfigInfo.postprocess_mask[i] |= feature_mask;
            }
            break;
            case HAL_PIXEL_FORMAT_YCbCr_420_888:
                mStreamConfigInfo.type[i] = CAM_STREAM_TYPE_CALLBACK;
                mStreamConfigInfo.postprocess_mask[i] = fullFeatureMask;
                padding_info.width_padding = mSurfaceStridePadding;
                padding_info.height_padding = CAM_PAD_TO_2;
            break;
            case HAL_PIXEL_FORMAT_BLOB:
                mStreamConfigInfo.type[i] = CAM_STREAM_TYPE_SNAPSHOT;
                // link the cpp if below conditions are met
                if ((m_bIs4KVideo && !isZsl) || bSmallJpegSize) {
                    mStreamConfigInfo.postprocess_mask[i] = fullFeatureMask;
                    mStreamConfigInfo.postprocess_mask[i] &= ~CAM_QCOM_FEATURE_CDS;
                } else {
                    if (bUseCommonFeatureMask &&
                            (((int32_t)newStream->width > maxViewfinderSize.width) ||
                                    ((int32_t)newStream->height > maxViewfinderSize.height))) {
                        mStreamConfigInfo.postprocess_mask[i] = commonFeatureMask;
                    } else {
                        mStreamConfigInfo.postprocess_mask[i] = CAM_QCOM_FEATURE_NONE;
                    }
                }
                if (m_bIs4KVideo) {
                    mStreamConfigInfo.stream_sizes[i].width = (int32_t)videoWidth;
                    mStreamConfigInfo.stream_sizes[i].height = (int32_t)videoHeight;
                }
                break;
            case HAL_PIXEL_FORMAT_RAW_OPAQUE:
            case HAL_PIXEL_FORMAT_RAW16:
            case HAL_PIXEL_FORMAT_RAW10:
                mStreamConfigInfo.type[i] = CAM_STREAM_TYPE_RAW;
                isRawStreamRequested = true;
                break;
            default:
                mStreamConfigInfo.type[i] = CAM_STREAM_TYPE_DEFAULT;
                mStreamConfigInfo.postprocess_mask[i] = CAM_QCOM_FEATURE_NONE;
                break;
            }
        }

        if (newStream->priv == NULL) {
            //New stream, construct channel
            switch (newStream->stream_type) {
            case CAMERA3_STREAM_INPUT:
                newStream->usage = GRALLOC_USAGE_HW_CAMERA_READ;
                break;
            case CAMERA3_STREAM_BIDIRECTIONAL:
                newStream->usage = GRALLOC_USAGE_HW_CAMERA_READ |
                    GRALLOC_USAGE_HW_CAMERA_WRITE;
                break;
            case CAMERA3_STREAM_OUTPUT:
                /* For video encoding stream, set read/write rarely
                 * flag so that they may be set to un-cached */
                if (newStream->usage & GRALLOC_USAGE_HW_VIDEO_ENCODER)
                    newStream->usage =
                         (GRALLOC_USAGE_SW_READ_RARELY |
                         GRALLOC_USAGE_SW_WRITE_RARELY |
                         GRALLOC_USAGE_HW_CAMERA_WRITE);
                else
                    newStream->usage = GRALLOC_USAGE_HW_CAMERA_WRITE;
                break;
            default:
                ALOGE("%s: Invalid stream_type %d", __func__, newStream->stream_type);
                break;
            }

            if (newStream->format == HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED &&
                    newStream->stream_type == CAMERA3_STREAM_BIDIRECTIONAL &&
                    jpegStream) {
                QCamera3Channel *channel = NULL;
                channel = new QCamera3RegularChannel(mCameraHandle->camera_handle,
                        mCameraHandle->ops, captureResultCb,
                        &padding_info,
                        this,
                        newStream,
                        (cam_stream_type_t) mStreamConfigInfo.type[i],
                        mStreamConfigInfo.postprocess_mask[i],
                        jpegStream->width, jpegStream->height);
                if (channel == NULL) {
                    ALOGE("%s: allocation of channel failed", __func__);
                    pthread_mutex_unlock(&mMutex);
                    return -ENOMEM;
                }
                newStream->max_buffers = channel->getNumBuffers();
                newStream->priv = channel;
            } else if (newStream->stream_type == CAMERA3_STREAM_OUTPUT) {
                QCamera3Channel *channel = NULL;
                switch (newStream->format) {
                case HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED:
                case HAL_PIXEL_FORMAT_YCbCr_420_888:
                    channel = new QCamera3RegularChannel(mCameraHandle->camera_handle,
                            mCameraHandle->ops, captureResultCb,
                            &padding_info,
                            this,
                            newStream,
                            (cam_stream_type_t) mStreamConfigInfo.type[i],
                            mStreamConfigInfo.postprocess_mask[i]);
                    if (channel == NULL) {
                        ALOGE("%s: allocation of channel failed", __func__);
                        pthread_mutex_unlock(&mMutex);
                        return -ENOMEM;
                    }
                    newStream->max_buffers = channel->getNumBuffers();
                    newStream->priv = channel;
                    break;
                case HAL_PIXEL_FORMAT_RAW_OPAQUE:
                case HAL_PIXEL_FORMAT_RAW16:
                case HAL_PIXEL_FORMAT_RAW10:
                    mRawChannel = new QCamera3RawChannel(
                            mCameraHandle->camera_handle,
                            mCameraHandle->ops, captureResultCb,
                            &padding_info,
                            this, newStream, CAM_QCOM_FEATURE_NONE,
                            (newStream->format == HAL_PIXEL_FORMAT_RAW16));
                    if (mRawChannel == NULL) {
                        ALOGE("%s: allocation of raw channel failed", __func__);
                        pthread_mutex_unlock(&mMutex);
                        return -ENOMEM;
                    }
                    newStream->max_buffers = mRawChannel->getNumBuffers();
                    newStream->priv = (QCamera3Channel*)mRawChannel;
                    break;
                case HAL_PIXEL_FORMAT_BLOB:
                    // Max live snapshot inflight buffer is 1. This is to mitigate
                    // frame drop issues for video snapshot. The more buffers being
                    // allocated, the more frame drops there are.
                    mPictureChannel = new QCamera3PicChannel(mCameraHandle->camera_handle,
                            mCameraHandle->ops, captureResultCb,
                            &padding_info, this, newStream,
                            mStreamConfigInfo.postprocess_mask[i],
                            m_bIs4KVideo, mMetadataChannel,
                            (m_bIsVideo ? 1 : MAX_INFLIGHT_REQUESTS));
                    if (mPictureChannel == NULL) {
                        ALOGE("%s: allocation of channel failed", __func__);
                        pthread_mutex_unlock(&mMutex);
                        return -ENOMEM;
                    }
                    newStream->priv = (QCamera3Channel*)mPictureChannel;
                    newStream->max_buffers = mPictureChannel->getNumBuffers();
                    break;

                default:
                    ALOGE("%s: not a supported format 0x%x", __func__, newStream->format);
                    break;
                }
            }

            QCamera3Channel *channel = (QCamera3Channel*) newStream->priv;
            if (channel != NULL && channel->isUBWCEnabled()) {
                cam_format_t fmt =
                        channel->getStreamDefaultFormat(mStreamConfigInfo.type[i]);
                if(fmt == CAM_FORMAT_YUV_420_NV12_UBWC) {
                    newStream->usage |= GRALLOC_USAGE_PRIVATE_ALLOC_UBWC;
                }
            }

            for (List<stream_info_t*>::iterator it=mStreamInfo.begin();
                    it != mStreamInfo.end(); it++) {
                if ((*it)->stream == newStream) {
                    (*it)->channel = (QCamera3Channel*) newStream->priv;
                    break;
                }
            }
        } else {
            // Channel already exists for this stream
            // Do nothing for now
        }
        padding_info = gCamCapability[mCameraId]->padding_info;
    }

    if (mPictureChannel && m_bIs4KVideo) {
        mPictureChannel->overrideYuvSize(videoWidth, videoHeight);
    }

    //RAW DUMP channel
    if (mEnableRawDump && isRawStreamRequested == false){
        cam_dimension_t rawDumpSize;
        rawDumpSize = getMaxRawSize(mCameraId);
        mRawDumpChannel = new QCamera3RawDumpChannel(mCameraHandle->camera_handle,
                                  mCameraHandle->ops,
                                  rawDumpSize,
                                  &padding_info,
                                  this, CAM_QCOM_FEATURE_NONE);
        if (!mRawDumpChannel) {
            ALOGE("%s: Raw Dump channel cannot be created", __func__);
            pthread_mutex_unlock(&mMutex);
            return -ENOMEM;
        }
    }


    mStreamConfigInfo.num_streams = streamList->num_streams;

    if (mAnalysisChannel) {
        mStreamConfigInfo.stream_sizes[mStreamConfigInfo.num_streams] =
                gCamCapability[mCameraId]->analysis_recommended_res;
        mStreamConfigInfo.type[mStreamConfigInfo.num_streams] =
                CAM_STREAM_TYPE_ANALYSIS;
        mStreamConfigInfo.postprocess_mask[mStreamConfigInfo.num_streams] =
                CAM_QCOM_FEATURE_FACE_DETECTION;
        mStreamConfigInfo.num_streams++;
    }

    if (mSupportChannel) {
        mStreamConfigInfo.stream_sizes[mStreamConfigInfo.num_streams] =
                QCamera3SupportChannel::kDim;
        mStreamConfigInfo.type[mStreamConfigInfo.num_streams] =
                CAM_STREAM_TYPE_CALLBACK;
        mStreamConfigInfo.postprocess_mask[mStreamConfigInfo.num_streams] =
                fullFeatureMask;
        mStreamConfigInfo.num_streams++;
    }

    if (mRawDumpChannel) {
        cam_dimension_t rawSize;
        rawSize = getMaxRawSize(mCameraId);
        mStreamConfigInfo.stream_sizes[mStreamConfigInfo.num_streams] =
                rawSize;
        mStreamConfigInfo.type[mStreamConfigInfo.num_streams] =
                CAM_STREAM_TYPE_RAW;
        mStreamConfigInfo.postprocess_mask[mStreamConfigInfo.num_streams] =
                CAM_QCOM_FEATURE_NONE;
        mStreamConfigInfo.num_streams++;
    }
    mStreamConfigInfo.buffer_info.min_buffers = MIN_INFLIGHT_REQUESTS;
    mStreamConfigInfo.buffer_info.max_buffers = MAX_INFLIGHT_REQUESTS;

    /* Initialize mPendingRequestInfo and mPendnigBuffersMap */
    mPendingRequestsList.clear();
    mPendingFrameDropList.clear();
    // Initialize/Reset the pending buffers list
    mPendingBuffersMap.num_buffers = 0;
    mPendingBuffersMap.mPendingBufferList.clear();
    mPendingReprocessResultList.clear();

    mFirstRequest = true;
    //Get min frame duration for this streams configuration
    deriveMinFrameDuration();

    /* Turn on video hint only if video stream is configured */
    updatePowerHint(bWasVideo, m_bIsVideo);

    pthread_mutex_unlock(&mMutex);
    return rc;
}

/*===========================================================================
 * FUNCTION   : validateCaptureRequest
 *
 * DESCRIPTION: validate a capture request from camera service
 *
 * PARAMETERS :
 *   @request : request from framework to process
 *
 * RETURN     :
 *
 *==========================================================================*/
int QCamera3HardwareInterface::validateCaptureRequest(
                    camera3_capture_request_t *request)
{
    ssize_t idx = 0;
    const camera3_stream_buffer_t *b;
    CameraMetadata meta;

    /* Sanity check the request */
    if (request == NULL) {
        ALOGE("%s: NULL capture request", __func__);
        return BAD_VALUE;
    }

    if (request->settings == NULL && mFirstRequest) {
        /*settings cannot be null for the first request*/
        return BAD_VALUE;
    }

    uint32_t frameNumber = request->frame_number;
    if (request->input_buffer != NULL &&
            request->input_buffer->stream != mInputStream) {
        ALOGE("%s: Request %d: Input buffer not from input stream!",
                __FUNCTION__, frameNumber);
        return BAD_VALUE;
    }
    if (request->num_output_buffers < 1 || request->output_buffers == NULL) {
        ALOGE("%s: Request %d: No output buffers provided!",
                __FUNCTION__, frameNumber);
        return BAD_VALUE;
    }
    if (request->num_output_buffers >= MAX_NUM_STREAMS) {
        ALOGE("%s: Number of buffers %d equals or is greater than maximum number of streams!",
                __func__, request->num_output_buffers, MAX_NUM_STREAMS);
        return BAD_VALUE;
    }
    if (request->input_buffer != NULL) {
        b = request->input_buffer;
        QCamera3Channel *channel =
            static_cast<QCamera3Channel*>(b->stream->priv);
        if (channel == NULL) {
            ALOGE("%s: Request %d: Buffer %ld: Unconfigured stream!",
                    __func__, frameNumber, (long)idx);
            return BAD_VALUE;
        }
        if (b->status != CAMERA3_BUFFER_STATUS_OK) {
            ALOGE("%s: Request %d: Buffer %ld: Status not OK!",
                    __func__, frameNumber, (long)idx);
            return BAD_VALUE;
        }
        if (b->release_fence != -1) {
            ALOGE("%s: Request %d: Buffer %ld: Has a release fence!",
                    __func__, frameNumber, (long)idx);
            return BAD_VALUE;
        }
        if (b->buffer == NULL) {
            ALOGE("%s: Request %d: Buffer %ld: NULL buffer handle!",
                    __func__, frameNumber, (long)idx);
            return BAD_VALUE;
        }
    }

    // Validate all buffers
    b = request->output_buffers;
    do {
        QCamera3Channel *channel =
                static_cast<QCamera3Channel*>(b->stream->priv);
        if (channel == NULL) {
            ALOGE("%s: Request %d: Buffer %ld: Unconfigured stream!",
                    __func__, frameNumber, (long)idx);
            return BAD_VALUE;
        }
        if (b->status != CAMERA3_BUFFER_STATUS_OK) {
            ALOGE("%s: Request %d: Buffer %ld: Status not OK!",
                    __func__, frameNumber, (long)idx);
            return BAD_VALUE;
        }
        if (b->release_fence != -1) {
            ALOGE("%s: Request %d: Buffer %ld: Has a release fence!",
                    __func__, frameNumber, (long)idx);
            return BAD_VALUE;
        }
        if (b->buffer == NULL) {
            ALOGE("%s: Request %d: Buffer %ld: NULL buffer handle!",
                    __func__, frameNumber, (long)idx);
            return BAD_VALUE;
        }
        if (*(b->buffer) == NULL) {
            ALOGE("%s: Request %d: Buffer %ld: NULL private handle!",
                    __func__, frameNumber, (long)idx);
            return BAD_VALUE;
        }
        idx++;
        b = request->output_buffers + idx;
    } while (idx < (ssize_t)request->num_output_buffers);

    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : deriveMinFrameDuration
 *
 * DESCRIPTION: derive mininum processed, jpeg, and raw frame durations based
 *              on currently configured streams.
 *
 * PARAMETERS : NONE
 *
 * RETURN     : NONE
 *
 *==========================================================================*/
void QCamera3HardwareInterface::deriveMinFrameDuration()
{
    int32_t maxJpegDim, maxProcessedDim, maxRawDim;

    maxJpegDim = 0;
    maxProcessedDim = 0;
    maxRawDim = 0;

    // Figure out maximum jpeg, processed, and raw dimensions
    for (List<stream_info_t*>::iterator it = mStreamInfo.begin();
        it != mStreamInfo.end(); it++) {

        // Input stream doesn't have valid stream_type
        if ((*it)->stream->stream_type == CAMERA3_STREAM_INPUT)
            continue;

        int32_t dimension = (int32_t)((*it)->stream->width * (*it)->stream->height);
        if ((*it)->stream->format == HAL_PIXEL_FORMAT_BLOB) {
            if (dimension > maxJpegDim)
                maxJpegDim = dimension;
        } else if ((*it)->stream->format == HAL_PIXEL_FORMAT_RAW_OPAQUE ||
                (*it)->stream->format == HAL_PIXEL_FORMAT_RAW10 ||
                (*it)->stream->format == HAL_PIXEL_FORMAT_RAW16) {
            if (dimension > maxRawDim)
                maxRawDim = dimension;
        } else {
            if (dimension > maxProcessedDim)
                maxProcessedDim = dimension;
        }
    }

    size_t count = MIN(gCamCapability[mCameraId]->supported_raw_dim_cnt,
            MAX_SIZES_CNT);

    //Assume all jpeg dimensions are in processed dimensions.
    if (maxJpegDim > maxProcessedDim)
        maxProcessedDim = maxJpegDim;
    //Find the smallest raw dimension that is greater or equal to jpeg dimension
    if (maxProcessedDim > maxRawDim) {
        maxRawDim = INT32_MAX;

        for (size_t i = 0; i < count; i++) {
            int32_t dimension = gCamCapability[mCameraId]->raw_dim[i].width *
                    gCamCapability[mCameraId]->raw_dim[i].height;
            if (dimension >= maxProcessedDim && dimension < maxRawDim)
                maxRawDim = dimension;
        }
    }

    //Find minimum durations for processed, jpeg, and raw
    for (size_t i = 0; i < count; i++) {
        if (maxRawDim == gCamCapability[mCameraId]->raw_dim[i].width *
                gCamCapability[mCameraId]->raw_dim[i].height) {
            mMinRawFrameDuration = gCamCapability[mCameraId]->raw_min_duration[i];
            break;
        }
    }
    count = MIN(gCamCapability[mCameraId]->picture_sizes_tbl_cnt, MAX_SIZES_CNT);
    for (size_t i = 0; i < count; i++) {
        if (maxProcessedDim ==
                gCamCapability[mCameraId]->picture_sizes_tbl[i].width *
                gCamCapability[mCameraId]->picture_sizes_tbl[i].height) {
            mMinProcessedFrameDuration = gCamCapability[mCameraId]->picture_min_duration[i];
            mMinJpegFrameDuration = gCamCapability[mCameraId]->picture_min_duration[i];
            break;
        }
    }
}

/*===========================================================================
 * FUNCTION   : getMinFrameDuration
 *
 * DESCRIPTION: get minimum frame draution based on the current maximum frame durations
 *              and current request configuration.
 *
 * PARAMETERS : @request: requset sent by the frameworks
 *
 * RETURN     : min farme duration for a particular request
 *
 *==========================================================================*/
int64_t QCamera3HardwareInterface::getMinFrameDuration(const camera3_capture_request_t *request)
{
    bool hasJpegStream = false;
    bool hasRawStream = false;
    for (uint32_t i = 0; i < request->num_output_buffers; i ++) {
        const camera3_stream_t *stream = request->output_buffers[i].stream;
        if (stream->format == HAL_PIXEL_FORMAT_BLOB)
            hasJpegStream = true;
        else if (stream->format == HAL_PIXEL_FORMAT_RAW_OPAQUE ||
                stream->format == HAL_PIXEL_FORMAT_RAW10 ||
                stream->format == HAL_PIXEL_FORMAT_RAW16)
            hasRawStream = true;
    }

    if (!hasJpegStream)
        return MAX(mMinRawFrameDuration, mMinProcessedFrameDuration);
    else
        return MAX(MAX(mMinRawFrameDuration, mMinProcessedFrameDuration), mMinJpegFrameDuration);
}

/*===========================================================================
 * FUNCTION   : handlePendingReprocResults
 *
 * DESCRIPTION: check and notify on any pending reprocess results
 *
 * PARAMETERS :
 *   @frame_number   : Pending request frame number
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3HardwareInterface::handlePendingReprocResults(uint32_t frame_number)
{
    for (List<PendingReprocessResult>::iterator j = mPendingReprocessResultList.begin();
            j != mPendingReprocessResultList.end(); j++) {
        if (j->frame_number == frame_number) {
            mCallbackOps->notify(mCallbackOps, &j->notify_msg);

            CDBG("%s: Delayed reprocess notify %d", __func__,
                    frame_number);

            for (List<PendingRequestInfo>::iterator k = mPendingRequestsList.begin();
                k != mPendingRequestsList.end(); k++) {

                if (k->frame_number == j->frame_number) {
                    CDBG("%s: Found reprocess frame number %d in pending reprocess List "
                            "Take it out!!", __func__,
                            k->frame_number);

                    camera3_capture_result result;
                    memset(&result, 0, sizeof(camera3_capture_result));
                    result.frame_number = frame_number;
                    result.num_output_buffers = 1;
                    result.output_buffers =  &j->buffer;
                    result.input_buffer = k->input_buffer;
                    result.result = k->settings;
                    result.partial_result = PARTIAL_RESULT_COUNT;
                    mCallbackOps->process_capture_result(mCallbackOps, &result);

                    mPendingRequestsList.erase(k);
                    mPendingRequest--;
                    break;
                }
            }
            mPendingReprocessResultList.erase(j);
            break;
        }
    }
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : handleMetadataWithLock
 *
 * DESCRIPTION: Handles metadata buffer callback with mMutex lock held.
 *
 * PARAMETERS : @metadata_buf: metadata buffer
 *
 * RETURN     :
 *
 *==========================================================================*/
void QCamera3HardwareInterface::handleMetadataWithLock(
    mm_camera_super_buf_t *metadata_buf)
{
    ATRACE_CALL();
    if (mFlushPerf) {
        //during flush do not send metadata from this thread
        CDBG("%s: not sending metadata during flush",
                            __func__);
        mMetadataChannel->bufDone(metadata_buf);
        free(metadata_buf);
        return;
    }

    //not in flush
    metadata_buffer_t *metadata = (metadata_buffer_t *)metadata_buf->bufs[0]->buffer;
    int32_t frame_number_valid, urgent_frame_number_valid;
    uint32_t frame_number, urgent_frame_number;
    int64_t capture_time;

    // Convert Boottime from camera to Monotime.
    uint8_t timestampSource = TIME_SOURCE;
    nsecs_t timeOffset = mBootToMonoTimestampOffset;
    if (ANDROID_SENSOR_INFO_TIMESTAMP_SOURCE_UNKNOWN != timestampSource)
        timeOffset = 0;

    int32_t *p_frame_number_valid =
            POINTER_OF_META(CAM_INTF_META_FRAME_NUMBER_VALID, metadata);
    uint32_t *p_frame_number = POINTER_OF_META(CAM_INTF_META_FRAME_NUMBER, metadata);
    int64_t *p_capture_time = POINTER_OF_META(CAM_INTF_META_SENSOR_TIMESTAMP, metadata);
    int32_t *p_urgent_frame_number_valid =
            POINTER_OF_META(CAM_INTF_META_URGENT_FRAME_NUMBER_VALID, metadata);
    uint32_t *p_urgent_frame_number =
            POINTER_OF_META(CAM_INTF_META_URGENT_FRAME_NUMBER, metadata);
    IF_META_AVAILABLE(cam_frame_dropped_t, p_cam_frame_drop, CAM_INTF_META_FRAME_DROPPED,
            metadata) {
        CDBG("%s: Dropped frame info for frame_number_valid %d, frame_number %d",
                __func__, *p_frame_number_valid, *p_frame_number);
    }

    if ((NULL == p_frame_number_valid) || (NULL == p_frame_number) || (NULL == p_capture_time) ||
            (NULL == p_urgent_frame_number_valid) || (NULL == p_urgent_frame_number)) {
        ALOGE("%s: Invalid metadata", __func__);
        mMetadataChannel->bufDone(metadata_buf);
        free(metadata_buf);
        goto done_metadata;
    } else {
        frame_number_valid = *p_frame_number_valid;
        frame_number = *p_frame_number;
        capture_time = *p_capture_time - timeOffset;
        urgent_frame_number_valid = *p_urgent_frame_number_valid;
        urgent_frame_number = *p_urgent_frame_number;
    }

    if (urgent_frame_number_valid) {
        CDBG("%s: valid urgent frame_number = %u, capture_time = %lld",
          __func__, urgent_frame_number, capture_time);

        //Recieved an urgent Frame Number, handle it
        //using partial results
        for (List<PendingRequestInfo>::iterator i =
            mPendingRequestsList.begin(); i != mPendingRequestsList.end(); i++) {
            CDBG("%s: Iterator Frame = %d urgent frame = %d",
                __func__, i->frame_number, urgent_frame_number);

            if (i->frame_number < urgent_frame_number &&
                i->partial_result_cnt == 0) {
                ALOGE("%s: Error: HAL missed urgent metadata for frame number %d",
                    __func__, i->frame_number);
            }

            if (i->frame_number == urgent_frame_number &&
                     i->bUrgentReceived == 0) {

                camera3_capture_result_t result;
                memset(&result, 0, sizeof(camera3_capture_result_t));

                i->partial_result_cnt++;
                i->bUrgentReceived = 1;
                // Extract 3A metadata
                result.result =
                    translateCbUrgentMetadataToResultMetadata(metadata);
                // Populate metadata result
                result.frame_number = urgent_frame_number;
                result.num_output_buffers = 0;
                result.output_buffers = NULL;
                result.partial_result = i->partial_result_cnt;

                mCallbackOps->process_capture_result(mCallbackOps, &result);
                CDBG("%s: urgent frame_number = %u, capture_time = %lld",
                     __func__, result.frame_number, capture_time);
                free_camera_metadata((camera_metadata_t *)result.result);
                break;
            }
        }
    }

    if (!frame_number_valid) {
        CDBG("%s: Not a valid normal frame number, used as SOF only", __func__);
        mMetadataChannel->bufDone(metadata_buf);
        free(metadata_buf);
        goto done_metadata;
    }
    CDBG("%s: valid frame_number = %u, capture_time = %lld", __func__,
            frame_number, capture_time);

    // Go through the pending requests info and send shutter/results to frameworks
    for (List<PendingRequestInfo>::iterator i = mPendingRequestsList.begin();
        i != mPendingRequestsList.end() && i->frame_number <= frame_number;) {
        camera3_capture_result_t result;
        memset(&result, 0, sizeof(camera3_capture_result_t));

        CDBG("%s: frame_number in the list is %u", __func__, i->frame_number);
        i->partial_result_cnt++;
        result.partial_result = i->partial_result_cnt;

        // Flush out all entries with less or equal frame numbers.
        mPendingRequest--;

        // Check whether any stream buffer corresponding to this is dropped or not
        // If dropped, then send the ERROR_BUFFER for the corresponding stream
        // The API does not expect a blob buffer to be dropped
        if (p_cam_frame_drop && p_cam_frame_drop->frame_dropped) {
            /* Clear notify_msg structure */
            camera3_notify_msg_t notify_msg;
            memset(&notify_msg, 0, sizeof(camera3_notify_msg_t));
            for (List<RequestedBufferInfo>::iterator j = i->buffers.begin();
                    j != i->buffers.end(); j++) {
               if (j->stream->format != HAL_PIXEL_FORMAT_BLOB) {
                   QCamera3Channel *channel = (QCamera3Channel *)j->stream->priv;
                   uint32_t streamID = channel->getStreamID(channel->getStreamTypeMask());
                   for (uint32_t k = 0; k < p_cam_frame_drop->cam_stream_ID.num_streams; k++) {
                       if (streamID == p_cam_frame_drop->cam_stream_ID.streamID[k]) {
                           // Send Error notify to frameworks with CAMERA3_MSG_ERROR_BUFFER
                           CDBG("%s: Start of reporting error frame#=%u, streamID=%u",
                                   __func__, i->frame_number, streamID);
                           notify_msg.type = CAMERA3_MSG_ERROR;
                           notify_msg.message.error.frame_number = i->frame_number;
                           notify_msg.message.error.error_code = CAMERA3_MSG_ERROR_BUFFER ;
                           notify_msg.message.error.error_stream = j->stream;
                           mCallbackOps->notify(mCallbackOps, &notify_msg);
                           CDBG("%s: End of reporting error frame#=%u, streamID=%u",
                                  __func__, i->frame_number, streamID);
                           PendingFrameDropInfo PendingFrameDrop;
                           PendingFrameDrop.frame_number=i->frame_number;
                           PendingFrameDrop.stream_ID = streamID;
                           // Add the Frame drop info to mPendingFrameDropList
                           mPendingFrameDropList.push_back(PendingFrameDrop);
                      }
                   }
               }
            }
        }

        // Send empty metadata with already filled buffers for dropped metadata
        // and send valid metadata with already filled buffers for current metadata
        if (i->frame_number < frame_number) {
            /* Clear notify_msg structure */
            camera3_notify_msg_t notify_msg;
            memset(&notify_msg, 0, sizeof(camera3_notify_msg_t));

            notify_msg.type = CAMERA3_MSG_SHUTTER;
            notify_msg.message.shutter.frame_number = i->frame_number;
            notify_msg.message.shutter.timestamp = (uint64_t)capture_time -
                    (frame_number - i->frame_number) * NSEC_PER_33MSEC;
            mCallbackOps->notify(mCallbackOps, &notify_msg);
            i->timestamp = (nsecs_t)notify_msg.message.shutter.timestamp;
            CDBG("%s: Support notification !!!! notify frame_number = %u, capture_time = %llu",
                    __func__, i->frame_number, notify_msg.message.shutter.timestamp);

            CameraMetadata dummyMetadata;
            dummyMetadata.update(ANDROID_SENSOR_TIMESTAMP,
                    &i->timestamp, 1);
            dummyMetadata.update(ANDROID_REQUEST_ID,
                    &(i->request_id), 1);
            result.result = dummyMetadata.release();
        } else {
            /* Clear notify_msg structure */
            camera3_notify_msg_t notify_msg;
            memset(&notify_msg, 0, sizeof(camera3_notify_msg_t));

            // Send shutter notify to frameworks
            notify_msg.type = CAMERA3_MSG_SHUTTER;
            notify_msg.message.shutter.frame_number = i->frame_number;
            notify_msg.message.shutter.timestamp = (uint64_t)capture_time;
            mCallbackOps->notify(mCallbackOps, &notify_msg);

            i->timestamp = capture_time;

            result.result = translateFromHalMetadata(metadata,
                    i->timestamp, i->request_id, i->jpegMetadata, i->pipeline_depth,
                    i->capture_intent, i->fwkCacMode);

            saveExifParams(metadata);

            if (i->blob_request) {
                {
                    //Dump tuning metadata if enabled and available
                    char prop[PROPERTY_VALUE_MAX];
                    memset(prop, 0, sizeof(prop));
                    property_get("persist.camera.dumpmetadata", prop, "0");
                    int32_t enabled = atoi(prop);
                    if (enabled && metadata->is_tuning_params_valid) {
                        dumpMetadataToFile(metadata->tuning_params,
                               mMetaFrameCount,
                               enabled,
                               "Snapshot",
                               frame_number);
                    }
                }
                mPictureChannel->queueReprocMetadata(metadata_buf);
            } else {
                // Return metadata buffer
                mMetadataChannel->bufDone(metadata_buf);
                free(metadata_buf);
            }
        }
        if (!result.result) {
            ALOGE("%s: metadata is NULL", __func__);
        }
        result.frame_number = i->frame_number;
        result.num_output_buffers = 0;
        result.output_buffers = NULL;
        for (List<RequestedBufferInfo>::iterator j = i->buffers.begin();
                    j != i->buffers.end(); j++) {
            if (j->buffer) {
                result.num_output_buffers++;
            }
        }

        if (result.num_output_buffers > 0) {
            camera3_stream_buffer_t *result_buffers =
                new camera3_stream_buffer_t[result.num_output_buffers];
            if (!result_buffers) {
                ALOGE("%s: Fatal error: out of memory", __func__);
            }
            size_t result_buffers_idx = 0;
            for (List<RequestedBufferInfo>::iterator j = i->buffers.begin();
                    j != i->buffers.end(); j++) {
                if (j->buffer) {
                    for (List<PendingFrameDropInfo>::iterator m = mPendingFrameDropList.begin();
                            m != mPendingFrameDropList.end(); m++) {
                        QCamera3Channel *channel = (QCamera3Channel *)j->buffer->stream->priv;
                        uint32_t streamID = channel->getStreamID(channel->getStreamTypeMask());
                        if((m->stream_ID == streamID) && (m->frame_number==frame_number)) {
                            j->buffer->status=CAMERA3_BUFFER_STATUS_ERROR;
                            CDBG("%s: Stream STATUS_ERROR frame_number=%u, streamID=%u",
                                  __func__, frame_number, streamID);
                            m = mPendingFrameDropList.erase(m);
                            break;
                        }
                    }

                    for (List<PendingBufferInfo>::iterator k =
                      mPendingBuffersMap.mPendingBufferList.begin();
                      k != mPendingBuffersMap.mPendingBufferList.end(); k++) {
                      if (k->buffer == j->buffer->buffer) {
                        CDBG("%s: Found buffer %p in pending buffer List "
                              "for frame %u, Take it out!!", __func__,
                               k->buffer, k->frame_number);
                        mPendingBuffersMap.num_buffers--;
                        k = mPendingBuffersMap.mPendingBufferList.erase(k);
                        break;
                      }
                    }

                    result_buffers[result_buffers_idx++] = *(j->buffer);
                    free(j->buffer);
                    j->buffer = NULL;
                }
            }
            result.output_buffers = result_buffers;
            mCallbackOps->process_capture_result(mCallbackOps, &result);
            CDBG("%s: meta frame_number = %u, capture_time = %lld",
                    __func__, result.frame_number, i->timestamp);
            free_camera_metadata((camera_metadata_t *)result.result);
            delete[] result_buffers;
        } else {
            mCallbackOps->process_capture_result(mCallbackOps, &result);
            CDBG("%s: meta frame_number = %u, capture_time = %lld",
                        __func__, result.frame_number, i->timestamp);
            free_camera_metadata((camera_metadata_t *)result.result);
        }
        // erase the element from the list
        i = mPendingRequestsList.erase(i);

        if (!mPendingReprocessResultList.empty()) {
            handlePendingReprocResults(frame_number + 1);
        }
    }

done_metadata:
    for (List<PendingRequestInfo>::iterator i = mPendingRequestsList.begin();
              i != mPendingRequestsList.end() ;i++) {
        i->pipeline_depth++;
    }
    unblockRequestIfNecessary();
}

/*===========================================================================
 * FUNCTION   : handleBufferWithLock
 *
 * DESCRIPTION: Handles image buffer callback with mMutex lock held.
 *
 * PARAMETERS : @buffer: image buffer for the callback
 *              @frame_number: frame number of the image buffer
 *
 * RETURN     :
 *
 *==========================================================================*/
void QCamera3HardwareInterface::handleBufferWithLock(
    camera3_stream_buffer_t *buffer, uint32_t frame_number)
{
    ATRACE_CALL();
    if (mFlushPerf) {
        // flush case
        //go through the pending buffers and mark them as returned.
        CDBG("%s: Handle buffer with lock called during flush", __func__);
        for (List<PendingBufferInfo>::iterator i =
                mPendingBuffersMap.mPendingBufferList.begin();
                i != mPendingBuffersMap.mPendingBufferList.end(); i++) {
            if (i->buffer == buffer->buffer) {
                mPendingBuffersMap.num_buffers--;
                CDBG("%s: Found Frame buffer, updated num_buffers %d, ",
                        __func__, mPendingBuffersMap.num_buffers);
                break;
            }
        }
        if (mPendingBuffersMap.num_buffers == 0) {
            //signal the flush()
            CDBG("%s: All buffers returned to HAL continue flush", __func__);
            pthread_cond_signal(&mBuffersCond);
        }
        return;
    }

    //not in flush
    // If the frame number doesn't exist in the pending request list,
    // directly send the buffer to the frameworks, and update pending buffers map
    // Otherwise, book-keep the buffer.
    List<PendingRequestInfo>::iterator i = mPendingRequestsList.begin();
    while (i != mPendingRequestsList.end() && i->frame_number != frame_number){
        i++;
    }
    if (i == mPendingRequestsList.end()) {
        // Verify all pending requests frame_numbers are greater
        for (List<PendingRequestInfo>::iterator j = mPendingRequestsList.begin();
                j != mPendingRequestsList.end(); j++) {
            if (j->frame_number < frame_number) {
                ALOGE("%s: Error: pending frame number %d is smaller than %d",
                        __func__, j->frame_number, frame_number);
            }
        }
        camera3_capture_result_t result;
        memset(&result, 0, sizeof(camera3_capture_result_t));
        result.result = NULL;
        result.frame_number = frame_number;
        result.num_output_buffers = 1;
        result.partial_result = 0;
        for (List<PendingFrameDropInfo>::iterator m = mPendingFrameDropList.begin();
                m != mPendingFrameDropList.end(); m++) {
            QCamera3Channel *channel = (QCamera3Channel *)buffer->stream->priv;
            uint32_t streamID = channel->getStreamID(channel->getStreamTypeMask());
            if((m->stream_ID == streamID) && (m->frame_number==frame_number) ) {
                buffer->status=CAMERA3_BUFFER_STATUS_ERROR;
                CDBG("%s: Stream STATUS_ERROR frame_number=%d, streamID=%d",
                        __func__, frame_number, streamID);
                m = mPendingFrameDropList.erase(m);
                break;
            }
        }
        result.output_buffers = buffer;
        CDBG("%s: result frame_number = %d, buffer = %p",
                __func__, frame_number, buffer->buffer);

        for (List<PendingBufferInfo>::iterator k =
                mPendingBuffersMap.mPendingBufferList.begin();
                k != mPendingBuffersMap.mPendingBufferList.end(); k++ ) {
            if (k->buffer == buffer->buffer) {
                CDBG("%s: Found Frame buffer, take it out from list",
                        __func__);

                mPendingBuffersMap.num_buffers--;
                k = mPendingBuffersMap.mPendingBufferList.erase(k);
                break;
            }
        }
        CDBG("%s: mPendingBuffersMap.num_buffers = %d",
            __func__, mPendingBuffersMap.num_buffers);

        mCallbackOps->process_capture_result(mCallbackOps, &result);
    } else {
        if (i->input_buffer) {
            CameraMetadata settings;
            camera3_notify_msg_t notify_msg;
            memset(&notify_msg, 0, sizeof(camera3_notify_msg_t));
            nsecs_t capture_time = systemTime(CLOCK_MONOTONIC);
            if(i->settings) {
                settings = i->settings;
                if (settings.exists(ANDROID_SENSOR_TIMESTAMP)) {
                    capture_time = settings.find(ANDROID_SENSOR_TIMESTAMP).data.i64[0];
                } else {
                    ALOGE("%s: No timestamp in input settings! Using current one.",
                            __func__);
                }
            } else {
                ALOGE("%s: Input settings missing!", __func__);
            }

            notify_msg.type = CAMERA3_MSG_SHUTTER;
            notify_msg.message.shutter.frame_number = frame_number;
            notify_msg.message.shutter.timestamp = (uint64_t)capture_time;

            sp<Fence> releaseFence = new Fence(i->input_buffer->release_fence);
            int32_t rc = releaseFence->wait(Fence::TIMEOUT_NEVER);
            if (rc != OK) {
                ALOGE("%s: input buffer fence wait failed %d", __func__, rc);
            }

            for (List<PendingBufferInfo>::iterator k =
                    mPendingBuffersMap.mPendingBufferList.begin();
                    k != mPendingBuffersMap.mPendingBufferList.end(); k++ ) {
                if (k->buffer == buffer->buffer) {
                    CDBG("%s: Found Frame buffer, take it out from list",
                            __func__);

                    mPendingBuffersMap.num_buffers--;
                    k = mPendingBuffersMap.mPendingBufferList.erase(k);
                    break;
                }
            }
            CDBG("%s: mPendingBuffersMap.num_buffers = %d",
                __func__, mPendingBuffersMap.num_buffers);

            bool notifyNow = true;
            for (List<PendingRequestInfo>::iterator j = mPendingRequestsList.begin();
                    j != mPendingRequestsList.end(); j++) {
                if (j->frame_number < frame_number) {
                    notifyNow = false;
                    break;
                }
            }

            if (notifyNow) {
                camera3_capture_result result;
                memset(&result, 0, sizeof(camera3_capture_result));
                result.frame_number = frame_number;
                result.result = i->settings;
                result.input_buffer = i->input_buffer;
                result.num_output_buffers = 1;
                result.output_buffers = buffer;
                result.partial_result = PARTIAL_RESULT_COUNT;

                mCallbackOps->notify(mCallbackOps, &notify_msg);
                mCallbackOps->process_capture_result(mCallbackOps, &result);
                CDBG("%s: Notify reprocess now %d!", __func__, frame_number);
                i = mPendingRequestsList.erase(i);
                mPendingRequest--;
            } else {
                // Cache reprocess result for later
                PendingReprocessResult pendingResult;
                memset(&pendingResult, 0, sizeof(PendingReprocessResult));
                pendingResult.notify_msg = notify_msg;
                pendingResult.buffer = *buffer;
                pendingResult.frame_number = frame_number;
                mPendingReprocessResultList.push_back(pendingResult);
                CDBG("%s: Cache reprocess result %d!", __func__, frame_number);
            }
        } else {
            for (List<RequestedBufferInfo>::iterator j = i->buffers.begin();
                j != i->buffers.end(); j++) {
                if (j->stream == buffer->stream) {
                    if (j->buffer != NULL) {
                        ALOGE("%s: Error: buffer is already set", __func__);
                    } else {
                        j->buffer = (camera3_stream_buffer_t *)malloc(
                            sizeof(camera3_stream_buffer_t));
                        *(j->buffer) = *buffer;
                        CDBG("%s: cache buffer %p at result frame_number %d",
                            __func__, buffer, frame_number);
                    }
                }
            }
        }
    }
}

/*===========================================================================
 * FUNCTION   : unblockRequestIfNecessary
 *
 * DESCRIPTION: Unblock capture_request if max_buffer hasn't been reached. Note
 *              that mMutex is held when this function is called.
 *
 * PARAMETERS :
 *
 * RETURN     :
 *
 *==========================================================================*/
void QCamera3HardwareInterface::unblockRequestIfNecessary()
{
   // Unblock process_capture_request
   pthread_cond_signal(&mRequestCond);
}

/*===========================================================================
 * FUNCTION   : processCaptureRequest
 *
 * DESCRIPTION: process a capture request from camera service
 *
 * PARAMETERS :
 *   @request : request from framework to process
 *
 * RETURN     :
 *
 *==========================================================================*/
int QCamera3HardwareInterface::processCaptureRequest(
                    camera3_capture_request_t *request)
{
    ATRACE_CALL();
    int rc = NO_ERROR;
    int32_t request_id;
    CameraMetadata meta;

    pthread_mutex_lock(&mMutex);

    rc = validateCaptureRequest(request);
    if (rc != NO_ERROR) {
        ALOGE("%s: incoming request is not valid", __func__);
        pthread_mutex_unlock(&mMutex);
        return rc;
    }

    meta = request->settings;

    // For first capture request, send capture intent, and
    // stream on all streams
    if (mFirstRequest) {
        // send an unconfigure to the backend so that the isp
        // resources are deallocated
        if (!mFirstConfiguration) {
            cam_stream_size_info_t stream_config_info;
            int32_t hal_version = CAM_HAL_V3;
            memset(&stream_config_info, 0, sizeof(cam_stream_size_info_t));
            stream_config_info.buffer_info.min_buffers =
                    MIN_INFLIGHT_REQUESTS;
            stream_config_info.buffer_info.max_buffers =
                    MAX_INFLIGHT_REQUESTS;
            clear_metadata_buffer(mParameters);
            ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters,
                    CAM_INTF_PARM_HAL_VERSION, hal_version);
            ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters,
                    CAM_INTF_META_STREAM_INFO, stream_config_info);
            rc = mCameraHandle->ops->set_parms(mCameraHandle->camera_handle,
                    mParameters);
            if (rc < 0) {
                ALOGE("%s: set_parms for unconfigure failed", __func__);
                pthread_mutex_unlock(&mMutex);
                return rc;
            }
        }

        /* get eis information for stream configuration */
        cam_is_type_t is_type;
        char is_type_value[PROPERTY_VALUE_MAX];
        property_get("camera.is_type", is_type_value, "0");
        is_type = static_cast<cam_is_type_t>(atoi(is_type_value));

        if (meta.exists(ANDROID_CONTROL_CAPTURE_INTENT)) {
            int32_t hal_version = CAM_HAL_V3;
            uint8_t captureIntent =
                meta.find(ANDROID_CONTROL_CAPTURE_INTENT).data.u8[0];
            mCaptureIntent = captureIntent;
            clear_metadata_buffer(mParameters);
            ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_PARM_HAL_VERSION, hal_version);
            ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_META_CAPTURE_INTENT, captureIntent);
        }

        //If EIS is enabled, turn it on for video
        bool setEis = m_bEisEnable && m_bEisSupportedSize &&
            ((mCaptureIntent ==  CAMERA3_TEMPLATE_VIDEO_RECORD) ||
             (mCaptureIntent == CAMERA3_TEMPLATE_VIDEO_SNAPSHOT));
        int32_t vsMode;
        vsMode = (setEis)? DIS_ENABLE: DIS_DISABLE;
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_PARM_DIS_ENABLE, vsMode)) {
            rc = BAD_VALUE;
        }

        //IS type will be 0 unless EIS is supported. If EIS is supported
        //it could either be 1 or 4 depending on the stream and video size
        if (setEis){
            if (!m_bEisSupportedSize) {
                is_type = IS_TYPE_DIS;
            } else {
                is_type = IS_TYPE_EIS_2_0;
            }
        }

        if (mCaptureIntent == CAMERA3_TEMPLATE_VIDEO_RECORD) {
            mStreamConfigInfo.is_type = is_type;
        } else {
            mStreamConfigInfo.is_type = IS_TYPE_NONE;
        }

        ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters,
                CAM_INTF_META_STREAM_INFO, mStreamConfigInfo);
        int32_t tintless_value = 1;
        ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters,
                CAM_INTF_PARM_TINTLESS, tintless_value);

        setMobicat();

        /* Set fps and hfr mode while sending meta stream info so that sensor
         * can configure appropriate streaming mode */
        if (meta.exists(ANDROID_CONTROL_AE_TARGET_FPS_RANGE)) {
            rc = setHalFpsRange(meta, mParameters);
            if (rc != NO_ERROR) {
                ALOGE("%s: setHalFpsRange failed", __func__);
            }
        }
        if (meta.exists(ANDROID_CONTROL_MODE)) {
            uint8_t metaMode = meta.find(ANDROID_CONTROL_MODE).data.u8[0];
            rc = extractSceneMode(meta, metaMode, mParameters);
            if (rc != NO_ERROR) {
                ALOGE("%s: extractSceneMode failed", __func__);
            }
        }
        /*set the capture intent, hal version, tintless, stream info,
         *and disenable parameters to the backend*/
        rc = mCameraHandle->ops->set_parms(mCameraHandle->camera_handle,
                    mParameters);
        if (rc < 0) {
            ALOGE("%s: set_parms failed for hal version, stream info", __func__);
        }

        cam_dimension_t sensor_dim;
        memset(&sensor_dim, 0, sizeof(sensor_dim));
        rc = getSensorOutputSize(sensor_dim);
        if (rc != NO_ERROR) {
            ALOGE("%s: Failed to get sensor output size", __func__);
            pthread_mutex_unlock(&mMutex);
            return rc;
        }

        mCropRegionMapper.update(gCamCapability[mCameraId]->active_array_size.width,
                gCamCapability[mCameraId]->active_array_size.height,
                sensor_dim.width, sensor_dim.height);

        for (size_t i = 0; i < request->num_output_buffers; i++) {
            const camera3_stream_buffer_t& output = request->output_buffers[i];
            QCamera3Channel *channel = (QCamera3Channel *)output.stream->priv;
            /*for livesnapshot stream is_type will be DIS*/
            if (setEis && output.stream->format == HAL_PIXEL_FORMAT_BLOB) {
                rc = channel->registerBuffer(output.buffer, IS_TYPE_DIS);
            } else {
                rc = channel->registerBuffer(output.buffer, is_type);
            }
            if (rc < 0) {
                ALOGE("%s: registerBuffer failed",
                        __func__);
                pthread_mutex_unlock(&mMutex);
                return -ENODEV;
            }
        }

        //First initialize all streams
        for (List<stream_info_t *>::iterator it = mStreamInfo.begin();
            it != mStreamInfo.end(); it++) {
            QCamera3Channel *channel = (QCamera3Channel *)(*it)->stream->priv;
            if (setEis && (*it)->stream->format == HAL_PIXEL_FORMAT_BLOB) {
                rc = channel->initialize(IS_TYPE_DIS);
            } else {
                rc = channel->initialize(is_type);
            }
            if (NO_ERROR != rc) {
                ALOGE("%s : Channel initialization failed %d", __func__, rc);
                pthread_mutex_unlock(&mMutex);
                return rc;
            }
        }

        if (mRawDumpChannel) {
            rc = mRawDumpChannel->initialize(is_type);
            if (rc != NO_ERROR) {
                ALOGE("%s: Error: Raw Dump Channel init failed", __func__);
                pthread_mutex_unlock(&mMutex);
                return rc;
            }
        }
        if (mSupportChannel) {
            rc = mSupportChannel->initialize(is_type);
            if (rc < 0) {
                ALOGE("%s: Support channel initialization failed", __func__);
                pthread_mutex_unlock(&mMutex);
                return rc;
            }
        }
        if (mAnalysisChannel) {
            rc = mAnalysisChannel->initialize(is_type);
            if (rc < 0) {
                ALOGE("%s: Analysis channel initialization failed", __func__);
                pthread_mutex_unlock(&mMutex);
                return rc;
            }
        }

        //Then start them.
        CDBG_HIGH("%s: Start META Channel", __func__);
        rc = mMetadataChannel->start();
        if (rc < 0) {
            ALOGE("%s: META channel start failed", __func__);
            pthread_mutex_unlock(&mMutex);
            return rc;
        }

        if (mAnalysisChannel) {
            rc = mAnalysisChannel->start();
            if (rc < 0) {
                ALOGE("%s: Analysis channel start failed", __func__);
                mMetadataChannel->stop();
                pthread_mutex_unlock(&mMutex);
                return rc;
            }
        }

        if (mSupportChannel) {
            rc = mSupportChannel->start();
            if (rc < 0) {
                ALOGE("%s: Support channel start failed", __func__);
                mMetadataChannel->stop();
                /* Although support and analysis are mutually exclusive today
                   adding it in anycase for future proofing */
                if (mAnalysisChannel) {
                    mAnalysisChannel->stop();
                }
                pthread_mutex_unlock(&mMutex);
                return rc;
            }
        }
        for (List<stream_info_t *>::iterator it = mStreamInfo.begin();
            it != mStreamInfo.end(); it++) {
            QCamera3Channel *channel = (QCamera3Channel *)(*it)->stream->priv;
            CDBG_HIGH("%s: Start Regular Channel mask=%d", __func__, channel->getStreamTypeMask());
            rc = channel->start();
            if (rc < 0) {
                ALOGE("%s: channel start failed", __func__);
                pthread_mutex_unlock(&mMutex);
                return rc;
            }
        }

        if (mRawDumpChannel) {
            CDBG("%s: Starting raw dump stream",__func__);
            rc = mRawDumpChannel->start();
            if (rc != NO_ERROR) {
                ALOGE("%s: Error Starting Raw Dump Channel", __func__);
                for (List<stream_info_t *>::iterator it = mStreamInfo.begin();
                      it != mStreamInfo.end(); it++) {
                    QCamera3Channel *channel =
                        (QCamera3Channel *)(*it)->stream->priv;
                    ALOGE("%s: Stopping Regular Channel mask=%d", __func__,
                        channel->getStreamTypeMask());
                    channel->stop();
                }
                if (mSupportChannel)
                    mSupportChannel->stop();
                if (mAnalysisChannel) {
                    mAnalysisChannel->stop();
                }
                mMetadataChannel->stop();
                pthread_mutex_unlock(&mMutex);
                return rc;
            }
        }
        mWokenUpByDaemon = false;
        mPendingRequest = 0;
        mFirstConfiguration = false;
    }

    if (!mFirstRequest && meta.exists(ANDROID_CONTROL_AE_TARGET_FPS_RANGE)) {
        cam_dimension_t sensor_dim;
        cam_fps_range_t fps_range;
        fps_range.min_fps = (float)
                meta.find(ANDROID_CONTROL_AE_TARGET_FPS_RANGE).data.i32[0];
        fps_range.max_fps = (float)
                meta.find(ANDROID_CONTROL_AE_TARGET_FPS_RANGE).data.i32[1];
        if ((fps_range.min_fps != mFpsRange.min_fps) ||
                (fps_range.max_fps != mFpsRange.max_fps)) {
            memset(&sensor_dim, 0, sizeof(sensor_dim));
            rc = getSensorOutputSize(sensor_dim, &meta);
            if (rc != NO_ERROR) {
                ALOGE("%s: Failed to get sensor output size", __func__);
                pthread_mutex_unlock(&mMutex);
                return rc;
            }
            mCropRegionMapper.update(gCamCapability[mCameraId]->active_array_size.width,
                    gCamCapability[mCameraId]->active_array_size.height,
                    sensor_dim.width, sensor_dim.height);
        }
    }

    uint32_t frameNumber = request->frame_number;
    cam_stream_ID_t streamID;

    if (mFlushPerf) {
        //we cannot accept any requests during flush
        ALOGE("%s: process_capture_request cannot proceed during flush", __func__);
        pthread_mutex_unlock(&mMutex);
        return NO_ERROR; //should return an error
    }

    if (meta.exists(ANDROID_REQUEST_ID)) {
        request_id = meta.find(ANDROID_REQUEST_ID).data.i32[0];
        mCurrentRequestId = request_id;
        CDBG("%s: Received request with id: %d",__func__, request_id);
    } else if (mFirstRequest || mCurrentRequestId == -1){
        ALOGE("%s: Unable to find request id field, \
                & no previous id available", __func__);
        pthread_mutex_unlock(&mMutex);
        return NAME_NOT_FOUND;
    } else {
        CDBG("%s: Re-using old request id", __func__);
        request_id = mCurrentRequestId;
    }

    CDBG("%s: %d, num_output_buffers = %d input_buffer = %p frame_number = %d",
                                    __func__, __LINE__,
                                    request->num_output_buffers,
                                    request->input_buffer,
                                    frameNumber);
    // Acquire all request buffers first
    streamID.num_streams = 0;
    int blob_request = 0;
    uint32_t snapshotStreamId = 0;
    for (size_t i = 0; i < request->num_output_buffers; i++) {
        const camera3_stream_buffer_t& output = request->output_buffers[i];
        QCamera3Channel *channel = (QCamera3Channel *)output.stream->priv;
        sp<Fence> acquireFence = new Fence(output.acquire_fence);

        if (output.stream->format == HAL_PIXEL_FORMAT_BLOB) {
            //Call function to store local copy of jpeg data for encode params.
            blob_request = 1;
            snapshotStreamId = channel->getStreamID(channel->getStreamTypeMask());
        }

        rc = acquireFence->wait(Fence::TIMEOUT_NEVER);
        if (rc != OK) {
            ALOGE("%s: fence wait failed %d", __func__, rc);
            pthread_mutex_unlock(&mMutex);
            return rc;
        }

        streamID.streamID[streamID.num_streams] =
            channel->getStreamID(channel->getStreamTypeMask());
        streamID.num_streams++;


    }

    if (blob_request && mRawDumpChannel) {
        CDBG("%s: Trigger Raw based on blob request if Raw dump is enabled", __func__);
        streamID.streamID[streamID.num_streams] =
            mRawDumpChannel->getStreamID(mRawDumpChannel->getStreamTypeMask());
        streamID.num_streams++;
    }

    if(request->input_buffer == NULL) {
       rc = setFrameParameters(request, streamID, blob_request, snapshotStreamId);
        if (rc < 0) {
            ALOGE("%s: fail to set frame parameters", __func__);
            pthread_mutex_unlock(&mMutex);
            return rc;
        }
    } else {
        sp<Fence> acquireFence = new Fence(request->input_buffer->acquire_fence);

        rc = acquireFence->wait(Fence::TIMEOUT_NEVER);
        if (rc != OK) {
            ALOGE("%s: input buffer fence wait failed %d", __func__, rc);
            pthread_mutex_unlock(&mMutex);
            return rc;
        }
    }

    /* Update pending request list and pending buffers map */
    PendingRequestInfo pendingRequest;
    pendingRequest.frame_number = frameNumber;
    pendingRequest.num_buffers = request->num_output_buffers;
    pendingRequest.request_id = request_id;
    pendingRequest.blob_request = blob_request;
    pendingRequest.timestamp = 0;
    pendingRequest.bUrgentReceived = 0;

    pendingRequest.input_buffer = request->input_buffer;
    pendingRequest.settings = request->settings;
    pendingRequest.pipeline_depth = 0;
    pendingRequest.partial_result_cnt = 0;
    extractJpegMetadata(pendingRequest.jpegMetadata, request);

    //extract capture intent
    if (meta.exists(ANDROID_CONTROL_CAPTURE_INTENT)) {
        mCaptureIntent =
                meta.find(ANDROID_CONTROL_CAPTURE_INTENT).data.u8[0];
    }
    pendingRequest.capture_intent = mCaptureIntent;

    //extract CAC info
    if (meta.exists(ANDROID_COLOR_CORRECTION_ABERRATION_MODE)) {
        mCacMode =
                meta.find(ANDROID_COLOR_CORRECTION_ABERRATION_MODE).data.u8[0];
    }
    pendingRequest.fwkCacMode = mCacMode;

    for (size_t i = 0; i < request->num_output_buffers; i++) {
        RequestedBufferInfo requestedBuf;
        requestedBuf.stream = request->output_buffers[i].stream;
        requestedBuf.buffer = NULL;
        pendingRequest.buffers.push_back(requestedBuf);

        // Add to buffer handle the pending buffers list
        PendingBufferInfo bufferInfo;
        bufferInfo.frame_number = frameNumber;
        bufferInfo.buffer = request->output_buffers[i].buffer;
        bufferInfo.stream = request->output_buffers[i].stream;
        mPendingBuffersMap.mPendingBufferList.push_back(bufferInfo);
        mPendingBuffersMap.num_buffers++;
        QCamera3Channel *channel = (QCamera3Channel *)bufferInfo.stream->priv;
        CDBG("%s: frame = %d, buffer = %p, streamTypeMask = %d, stream format = %d",
                __func__, frameNumber, bufferInfo.buffer,
                channel->getStreamTypeMask(), bufferInfo.stream->format);
    }
    CDBG("%s: mPendingBuffersMap.num_buffers = %d",
          __func__, mPendingBuffersMap.num_buffers);

    mPendingRequestsList.push_back(pendingRequest);

    if (mFlush) {
        pthread_mutex_unlock(&mMutex);
        return NO_ERROR;
    }

    // Notify metadata channel we receive a request
    mMetadataChannel->request(NULL, frameNumber);

    // Call request on other streams
    for (size_t i = 0; i < request->num_output_buffers; i++) {
        const camera3_stream_buffer_t& output = request->output_buffers[i];
        QCamera3Channel *channel = (QCamera3Channel *)output.stream->priv;

        if (channel == NULL) {
            ALOGE("%s: invalid channel pointer for stream", __func__);
            continue;
        }

        if (output.stream->format == HAL_PIXEL_FORMAT_BLOB) {
            QCamera3RegularChannel* inputChannel = NULL;
            CDBG("%s: snapshot request with output buffer %p, input buffer %p, frame_number %d",
                     __func__, output.buffer, request->input_buffer, frameNumber);
            if(request->input_buffer != NULL){

                //Try to get the internal format
                inputChannel = (QCamera3RegularChannel*)
                    request->input_buffer->stream->priv;
                if(inputChannel == NULL ){
                    ALOGE("%s: failed to get input channel handle", __func__);
                    pthread_mutex_unlock(&mMutex);
                    return NO_INIT;
                }
                rc = setReprocParameters(request, &mRreprocMeta, snapshotStreamId);
                if (NO_ERROR == rc) {
                    rc = channel->request(output.buffer, frameNumber,
                            request->input_buffer, &mRreprocMeta);
                    if (rc < 0) {
                        ALOGE("%s: Fail to request on picture channel", __func__);
                        pthread_mutex_unlock(&mMutex);
                        return rc;
                    }
                } else {
                    ALOGE("%s: fail to set reproc parameters", __func__);
                    pthread_mutex_unlock(&mMutex);
                    return rc;
                }
            } else{
                 if (!request->settings) {
                   rc = channel->request(output.buffer, frameNumber,
                               NULL, mPrevParameters);
                 } else {
                    rc = channel->request(output.buffer, frameNumber,
                               NULL, mParameters);
                 }
            }
        } else {
            CDBG("%s: %d, request with buffer %p, frame_number %d", __func__,
                __LINE__, output.buffer, frameNumber);
           rc = channel->request(output.buffer, frameNumber);
        }
        if (rc < 0)
            ALOGE("%s: request failed", __func__);
    }

    if(request->input_buffer == NULL) {
        /*set the parameters to backend*/
        rc = mCameraHandle->ops->set_parms(mCameraHandle->camera_handle, mParameters);
        if (rc < 0) {
            ALOGE("%s: set_parms failed", __func__);
        }
    }

    mFirstRequest = false;
    // Added a timed condition wait
    struct timespec ts;
    uint8_t isValidTimeout = 1;
    rc = clock_gettime(CLOCK_REALTIME, &ts);
    if (rc < 0) {
      isValidTimeout = 0;
      ALOGE("%s: Error reading the real time clock!!", __func__);
    }
    else {
      // Make timeout as 5 sec for request to be honored
      ts.tv_sec += 5;
    }
    //Block on conditional variable

    mPendingRequest++;
    while (mPendingRequest >= MIN_INFLIGHT_REQUESTS) {
        if (!isValidTimeout) {
            CDBG("%s: Blocking on conditional wait", __func__);
            pthread_cond_wait(&mRequestCond, &mMutex);
        }
        else {
            CDBG("%s: Blocking on timed conditional wait", __func__);
            rc = pthread_cond_timedwait(&mRequestCond, &mMutex, &ts);
            if (rc == ETIMEDOUT) {
                rc = -ENODEV;
                ALOGE("%s: Unblocked on timeout!!!!", __func__);
                break;
            }
        }
        CDBG("%s: Unblocked", __func__);
        if (mWokenUpByDaemon) {
            mWokenUpByDaemon = false;
            if (mPendingRequest < MAX_INFLIGHT_REQUESTS)
                break;
        }
    }
    pthread_mutex_unlock(&mMutex);

    return rc;
}

/*===========================================================================
 * FUNCTION   : dump
 *
 * DESCRIPTION:
 *
 * PARAMETERS :
 *
 *
 * RETURN     :
 *==========================================================================*/
void QCamera3HardwareInterface::dump(int fd)
{
    pthread_mutex_lock(&mMutex);
    dprintf(fd, "\n Camera HAL3 information Begin \n");

    dprintf(fd, "\nNumber of pending requests: %zu \n",
        mPendingRequestsList.size());
    dprintf(fd, "-------+-------------------+-------------+----------+---------------------\n");
    dprintf(fd, " Frame | Number of Buffers |   Req Id:   | Blob Req | Input buffer present\n");
    dprintf(fd, "-------+-------------------+-------------+----------+---------------------\n");
    for(List<PendingRequestInfo>::iterator i = mPendingRequestsList.begin();
        i != mPendingRequestsList.end(); i++) {
        dprintf(fd, " %5d | %17d | %11d | %8d | %p \n",
        i->frame_number, i->num_buffers, i->request_id, i->blob_request,
        i->input_buffer);
    }
    dprintf(fd, "\nPending buffer map: Number of buffers: %u\n",
                mPendingBuffersMap.num_buffers);
    dprintf(fd, "-------+------------------\n");
    dprintf(fd, " Frame | Stream type mask \n");
    dprintf(fd, "-------+------------------\n");
    for(List<PendingBufferInfo>::iterator i =
        mPendingBuffersMap.mPendingBufferList.begin();
        i != mPendingBuffersMap.mPendingBufferList.end(); i++) {
        QCamera3Channel *channel = (QCamera3Channel *)(i->stream->priv);
        dprintf(fd, " %5d | %11d \n",
                i->frame_number, channel->getStreamTypeMask());
    }
    dprintf(fd, "-------+------------------\n");

    dprintf(fd, "\nPending frame drop list: %zu\n",
        mPendingFrameDropList.size());
    dprintf(fd, "-------+-----------\n");
    dprintf(fd, " Frame | Stream ID \n");
    dprintf(fd, "-------+-----------\n");
    for(List<PendingFrameDropInfo>::iterator i = mPendingFrameDropList.begin();
        i != mPendingFrameDropList.end(); i++) {
        dprintf(fd, " %5d | %9d \n",
            i->frame_number, i->stream_ID);
    }
    dprintf(fd, "-------+-----------\n");

    dprintf(fd, "\n Camera HAL3 information End \n");

    /* use dumpsys media.camera as trigger to send update debug level event */
    mUpdateDebugLevel = true;
    pthread_mutex_unlock(&mMutex);
    return;
}

/*===========================================================================
 * FUNCTION   : flush
 *
 * DESCRIPTION:
 *
 * PARAMETERS :
 *
 *
 * RETURN     :
 *==========================================================================*/
int QCamera3HardwareInterface::flush()
{
    ATRACE_CALL();
    unsigned int frameNum = 0;
    camera3_capture_result_t result;
    camera3_stream_buffer_t *pStream_Buf = NULL;
    FlushMap flushMap;

    CDBG("%s: Unblocking Process Capture Request", __func__);
    pthread_mutex_lock(&mMutex);
    mFlush = true;
    pthread_mutex_unlock(&mMutex);

    memset(&result, 0, sizeof(camera3_capture_result_t));

    // Stop the Streams/Channels
    for (List<stream_info_t *>::iterator it = mStreamInfo.begin();
        it != mStreamInfo.end(); it++) {
        QCamera3Channel *channel = (QCamera3Channel *)(*it)->stream->priv;
        channel->stop();
        (*it)->status = INVALID;
    }

    if (mSupportChannel) {
        mSupportChannel->stop();
    }
    if (mAnalysisChannel) {
        mAnalysisChannel->stop();
    }
    if (mRawDumpChannel) {
        mRawDumpChannel->stop();
    }
    if (mMetadataChannel) {
        /* If content of mStreamInfo is not 0, there is metadata stream */
        mMetadataChannel->stop();
    }

    // Mutex Lock
    pthread_mutex_lock(&mMutex);

    // Unblock process_capture_request
    mPendingRequest = 0;
    pthread_cond_signal(&mRequestCond);

    List<PendingRequestInfo>::iterator i = mPendingRequestsList.begin();
    frameNum = i->frame_number;
    CDBG("%s: Oldest frame num on  mPendingRequestsList = %d",
      __func__, frameNum);

    // Go through the pending buffers and group them depending
    // on frame number
    for (List<PendingBufferInfo>::iterator k =
            mPendingBuffersMap.mPendingBufferList.begin();
            k != mPendingBuffersMap.mPendingBufferList.end();) {

        if (k->frame_number < frameNum) {
            ssize_t idx = flushMap.indexOfKey(k->frame_number);
            if (idx == NAME_NOT_FOUND) {
                Vector<PendingBufferInfo> pending;
                pending.add(*k);
                flushMap.add(k->frame_number, pending);
            } else {
                Vector<PendingBufferInfo> &pending =
                        flushMap.editValueFor(k->frame_number);
                pending.add(*k);
            }

            mPendingBuffersMap.num_buffers--;
            k = mPendingBuffersMap.mPendingBufferList.erase(k);
        } else {
            k++;
        }
    }

    for (size_t iFlush = 0; iFlush < flushMap.size(); iFlush++) {
        uint32_t frame_number = flushMap.keyAt(iFlush);
        const Vector<PendingBufferInfo> &pending = flushMap.valueAt(iFlush);

        // Send Error notify to frameworks for each buffer for which
        // metadata buffer is already sent
        CDBG("%s: Sending ERROR BUFFER for frame %d number of buffer %d",
          __func__, frame_number, pending.size());

        pStream_Buf = new camera3_stream_buffer_t[pending.size()];
        if (NULL == pStream_Buf) {
            ALOGE("%s: No memory for pending buffers array", __func__);
            pthread_mutex_unlock(&mMutex);
            return NO_MEMORY;
        }
        memset(pStream_Buf, 0, sizeof(camera3_stream_buffer_t)*pending.size());

        for (size_t j = 0; j < pending.size(); j++) {
            const PendingBufferInfo &info = pending.itemAt(j);
            camera3_notify_msg_t notify_msg;
            memset(&notify_msg, 0, sizeof(camera3_notify_msg_t));
            notify_msg.type = CAMERA3_MSG_ERROR;
            notify_msg.message.error.error_code = CAMERA3_MSG_ERROR_BUFFER;
            notify_msg.message.error.error_stream = info.stream;
            notify_msg.message.error.frame_number = frame_number;
            pStream_Buf[j].acquire_fence = -1;
            pStream_Buf[j].release_fence = -1;
            pStream_Buf[j].buffer = info.buffer;
            pStream_Buf[j].status = CAMERA3_BUFFER_STATUS_ERROR;
            pStream_Buf[j].stream = info.stream;
            mCallbackOps->notify(mCallbackOps, &notify_msg);
            CDBG("%s: notify frame_number = %d stream %p", __func__,
                    frame_number, info.stream);
        }

        result.result = NULL;
        result.frame_number = frame_number;
        result.num_output_buffers = (uint32_t)pending.size();
        result.output_buffers = pStream_Buf;
        mCallbackOps->process_capture_result(mCallbackOps, &result);

        delete [] pStream_Buf;
    }

    CDBG("%s:Sending ERROR REQUEST for all pending requests", __func__);

    flushMap.clear();
    for (List<PendingBufferInfo>::iterator k =
            mPendingBuffersMap.mPendingBufferList.begin();
            k != mPendingBuffersMap.mPendingBufferList.end();) {
        ssize_t idx = flushMap.indexOfKey(k->frame_number);
        if (idx == NAME_NOT_FOUND) {
            Vector<PendingBufferInfo> pending;
            pending.add(*k);
            flushMap.add(k->frame_number, pending);
        } else {
            Vector<PendingBufferInfo> &pending =
                    flushMap.editValueFor(k->frame_number);
            pending.add(*k);
        }

        mPendingBuffersMap.num_buffers--;
        k = mPendingBuffersMap.mPendingBufferList.erase(k);
    }

    // Go through the pending requests info and send error request to framework
    for (size_t iFlush = 0; iFlush < flushMap.size(); iFlush++) {
        uint32_t frame_number = flushMap.keyAt(iFlush);
        const Vector<PendingBufferInfo> &pending = flushMap.valueAt(iFlush);
        CDBG("%s:Sending ERROR REQUEST for frame %d",
              __func__, frame_number);

        // Send shutter notify to frameworks
        camera3_notify_msg_t notify_msg;
        memset(&notify_msg, 0, sizeof(camera3_notify_msg_t));
        notify_msg.type = CAMERA3_MSG_ERROR;
        notify_msg.message.error.error_code = CAMERA3_MSG_ERROR_REQUEST;
        notify_msg.message.error.error_stream = NULL;
        notify_msg.message.error.frame_number = frame_number;
        mCallbackOps->notify(mCallbackOps, &notify_msg);

        pStream_Buf = new camera3_stream_buffer_t[pending.size()];
        if (NULL == pStream_Buf) {
            ALOGE("%s: No memory for pending buffers array", __func__);
            pthread_mutex_unlock(&mMutex);
            return NO_MEMORY;
        }
        memset(pStream_Buf, 0, sizeof(camera3_stream_buffer_t)*pending.size());

        for (size_t j = 0; j < pending.size(); j++) {
            const PendingBufferInfo &info = pending.itemAt(j);
            pStream_Buf[j].acquire_fence = -1;
            pStream_Buf[j].release_fence = -1;
            pStream_Buf[j].buffer = info.buffer;
            pStream_Buf[j].status = CAMERA3_BUFFER_STATUS_ERROR;
            pStream_Buf[j].stream = info.stream;
        }

        result.num_output_buffers = (uint32_t)pending.size();
        result.output_buffers = pStream_Buf;
        result.result = NULL;
        result.frame_number = frame_number;
        mCallbackOps->process_capture_result(mCallbackOps, &result);
        delete [] pStream_Buf;
    }

    /* Reset pending buffer list and requests list */
    mPendingRequestsList.clear();
    /* Reset pending frame Drop list and requests list */
    mPendingFrameDropList.clear();

    flushMap.clear();
    mPendingBuffersMap.num_buffers = 0;
    mPendingBuffersMap.mPendingBufferList.clear();
    mPendingReprocessResultList.clear();
    CDBG("%s: Cleared all the pending buffers ", __func__);

    mFlush = false;

    // Start the Streams/Channels
    int rc = NO_ERROR;
    if (mMetadataChannel) {
        /* If content of mStreamInfo is not 0, there is metadata stream */
        rc = mMetadataChannel->start();
        if (rc < 0) {
            ALOGE("%s: META channel start failed", __func__);
            pthread_mutex_unlock(&mMutex);
            return rc;
        }
    }
    for (List<stream_info_t *>::iterator it = mStreamInfo.begin();
        it != mStreamInfo.end(); it++) {
        QCamera3Channel *channel = (QCamera3Channel *)(*it)->stream->priv;
        rc = channel->start();
        if (rc < 0) {
            ALOGE("%s: channel start failed", __func__);
            pthread_mutex_unlock(&mMutex);
            return rc;
        }
    }
    if (mAnalysisChannel) {
        mAnalysisChannel->start();
    }
    if (mSupportChannel) {
        rc = mSupportChannel->start();
        if (rc < 0) {
            ALOGE("%s: Support channel start failed", __func__);
            pthread_mutex_unlock(&mMutex);
            return rc;
        }
    }
    if (mRawDumpChannel) {
        rc = mRawDumpChannel->start();
        if (rc < 0) {
            ALOGE("%s: RAW dump channel start failed", __func__);
            pthread_mutex_unlock(&mMutex);
            return rc;
        }
    }

    pthread_mutex_unlock(&mMutex);

    return 0;
}

/*===========================================================================
 * FUNCTION   : flushPerf
 *
 * DESCRIPTION: This is the performance optimization version of flush that does
 *              not use stream off, rather flushes the system
 *
 * PARAMETERS :
 *
 *
 * RETURN     : 0 : success
 *              -EINVAL: input is malformed (device is not valid)
 *              -ENODEV: if the device has encountered a serious error
 *==========================================================================*/
int QCamera3HardwareInterface::flushPerf()
{
    ATRACE_CALL();
    int32_t rc = 0;
    struct timespec timeout;
    unsigned int frameNum = 0;
    bool timed_wait = false;
    camera3_stream_buffer_t *pStream_Buf = NULL;
    FlushMap flushMap;

    pthread_mutex_lock(&mMutex);
    mFlushPerf = true;

    /* send the flush event to the backend */
    rc = mCameraHandle->ops->flush(mCameraHandle->camera_handle);
    if (rc < 0) {
        ALOGE("%s: Error in flush: IOCTL failure", __func__);
        mFlushPerf = false;
        pthread_mutex_unlock(&mMutex);
        return -ENODEV;
    }

    if (mPendingBuffersMap.num_buffers == 0) {
        CDBG("%s: No pending buffers in the HAL, return flush");
        mFlushPerf = false;
        pthread_mutex_unlock(&mMutex);
        return rc;
    }

    /* wait on a signal that buffers were received */
    rc = clock_gettime(CLOCK_REALTIME, &timeout);
    if (rc < 0) {
        ALOGE("%s: Error reading the real time clock, cannot use timed wait",
                __func__);
    } else {
        timeout.tv_sec += FLUSH_TIMEOUT;
        timed_wait = true;
    }

    //Block on conditional variable
    while (mPendingBuffersMap.num_buffers != 0) {
        CDBG("%s: Waiting on mBuffersCond", __func__);
        if (!timed_wait) {
            rc = pthread_cond_wait(&mBuffersCond, &mMutex);
            if (rc != 0) {
                 ALOGE("%s: pthread_cond_wait failed due to rc = %s", __func__,
                        strerror(rc));
                 break;
            }
        } else {
            rc = pthread_cond_timedwait(&mBuffersCond, &mMutex, &timeout);
            if (rc != 0) {
                ALOGE("%s: pthread_cond_timedwait failed due to rc = %s", __func__,
                            strerror(rc));
                break;
            }
        }
    }
    if (rc != 0) {
        mFlushPerf = false;
        pthread_mutex_unlock(&mMutex);
        return -ENODEV;
    }

    CDBG("%s: Received buffers, now safe to return them", __func__);

    List<PendingRequestInfo>::iterator i = mPendingRequestsList.begin();
    frameNum = i->frame_number;
    CDBG("%s: Oldest frame num on  mPendingRequestsList = %d",
      __func__, frameNum);

    // Go through the pending buffers and group them depending
    // on frame number
    for (List<PendingBufferInfo>::iterator k =
            mPendingBuffersMap.mPendingBufferList.begin();
            k != mPendingBuffersMap.mPendingBufferList.end();) {

        if (k->frame_number < frameNum) {
            ssize_t idx = flushMap.indexOfKey(k->frame_number);
            if (idx == NAME_NOT_FOUND) {
                Vector<PendingBufferInfo> pending;
                pending.add(*k);
                flushMap.add(k->frame_number, pending);
            } else {
                Vector<PendingBufferInfo> &pending =
                        flushMap.editValueFor(k->frame_number);
                pending.add(*k);
            }
            k = mPendingBuffersMap.mPendingBufferList.erase(k);
        } else {
            k++;
        }
    }

    for (size_t iFlush = 0; iFlush < flushMap.size(); iFlush++) {
        camera3_capture_result_t result;
        uint32_t frame_number = flushMap.keyAt(iFlush);
        const Vector<PendingBufferInfo> &pending = flushMap.valueAt(iFlush);

        // Send Error notify to frameworks for each buffer for which
        // metadata buffer is already sent
        CDBG("%s: Sending ERROR BUFFER for frame %d number of buffer %d",
          __func__, frame_number, pending.size());

        pStream_Buf = new camera3_stream_buffer_t[pending.size()];
        if (NULL == pStream_Buf) {
            ALOGE("%s: No memory for pending buffers array", __func__);
            pthread_mutex_unlock(&mMutex);
            return NO_MEMORY;
        }
        memset(pStream_Buf, 0, sizeof(camera3_stream_buffer_t)*pending.size());

        for (size_t j = 0; j < pending.size(); j++) {
            const PendingBufferInfo &info = pending.itemAt(j);
            camera3_notify_msg_t notify_msg;
            memset(&notify_msg, 0, sizeof(camera3_notify_msg_t));
            notify_msg.type = CAMERA3_MSG_ERROR;
            notify_msg.message.error.error_code = CAMERA3_MSG_ERROR_BUFFER;
            notify_msg.message.error.error_stream = info.stream;
            notify_msg.message.error.frame_number = frame_number;
            pStream_Buf[j].acquire_fence = -1;
            pStream_Buf[j].release_fence = -1;
            pStream_Buf[j].buffer = info.buffer;
            pStream_Buf[j].status = CAMERA3_BUFFER_STATUS_ERROR;
            pStream_Buf[j].stream = info.stream;
            mCallbackOps->notify(mCallbackOps, &notify_msg);
            CDBG("%s: notify frame_number = %d stream %p", __func__,
                    frame_number, info.stream);
        }

        result.result = NULL;
        result.frame_number = frame_number;
        result.num_output_buffers = (uint32_t)pending.size();
        result.output_buffers = pStream_Buf;
        mCallbackOps->process_capture_result(mCallbackOps, &result);

        delete [] pStream_Buf;
    }

    CDBG("%s:Sending ERROR REQUEST for all pending requests", __func__);

    flushMap.clear();
    for (List<PendingBufferInfo>::iterator k =
            mPendingBuffersMap.mPendingBufferList.begin();
            k != mPendingBuffersMap.mPendingBufferList.end();) {
        ssize_t idx = flushMap.indexOfKey(k->frame_number);
        if (idx == NAME_NOT_FOUND) {
            Vector<PendingBufferInfo> pending;
            pending.add(*k);
            flushMap.add(k->frame_number, pending);
        } else {
            Vector<PendingBufferInfo> &pending =
                    flushMap.editValueFor(k->frame_number);
            pending.add(*k);
        }
        k = mPendingBuffersMap.mPendingBufferList.erase(k);
    }

    // Go through the pending requests info and send error request to framework
    for (size_t iFlush = 0; iFlush < flushMap.size(); iFlush++) {
        camera3_capture_result_t result;
        uint32_t frame_number = flushMap.keyAt(iFlush);
        const Vector<PendingBufferInfo> &pending = flushMap.valueAt(iFlush);
        CDBG("%s:Sending ERROR REQUEST for frame %d",
              __func__, frame_number);

        // Send shutter notify to frameworks
        camera3_notify_msg_t notify_msg;
        memset(&notify_msg, 0, sizeof(camera3_notify_msg_t));
        notify_msg.type = CAMERA3_MSG_ERROR;
        notify_msg.message.error.error_code = CAMERA3_MSG_ERROR_REQUEST;
        notify_msg.message.error.error_stream = NULL;
        notify_msg.message.error.frame_number = frame_number;
        mCallbackOps->notify(mCallbackOps, &notify_msg);

        pStream_Buf = new camera3_stream_buffer_t[pending.size()];
        if (NULL == pStream_Buf) {
            ALOGE("%s: No memory for pending buffers array", __func__);
            pthread_mutex_unlock(&mMutex);
            return NO_MEMORY;
        }
        memset(pStream_Buf, 0, sizeof(camera3_stream_buffer_t)*pending.size());

        for (size_t j = 0; j < pending.size(); j++) {
            const PendingBufferInfo &info = pending.itemAt(j);
            pStream_Buf[j].acquire_fence = -1;
            pStream_Buf[j].release_fence = -1;
            pStream_Buf[j].buffer = info.buffer;
            pStream_Buf[j].status = CAMERA3_BUFFER_STATUS_ERROR;
            pStream_Buf[j].stream = info.stream;
        }

        result.num_output_buffers = (uint32_t)pending.size();
        result.output_buffers = pStream_Buf;
        result.result = NULL;
        result.frame_number = frame_number;
        mCallbackOps->process_capture_result(mCallbackOps, &result);
        delete [] pStream_Buf;
    }

    /* Reset pending buffer list and requests list */
    mPendingRequestsList.clear();
    /* Reset pending frame Drop list and requests list */
    mPendingFrameDropList.clear();

    flushMap.clear();
    mPendingBuffersMap.num_buffers = 0;
    mPendingBuffersMap.mPendingBufferList.clear();
    mPendingReprocessResultList.clear();
    CDBG("%s: Cleared all the pending buffers ", __func__);

    //unblock process_capture_request
    mPendingRequest = 0;
    unblockRequestIfNecessary();

    mFlushPerf = false;
    pthread_mutex_unlock(&mMutex);
    return rc;
}

/*===========================================================================
 * FUNCTION   : captureResultCb
 *
 * DESCRIPTION: Callback handler for all capture result
 *              (streams, as well as metadata)
 *
 * PARAMETERS :
 *   @metadata : metadata information
 *   @buffer   : actual gralloc buffer to be returned to frameworks.
 *               NULL if metadata.
 *
 * RETURN     : NONE
 *==========================================================================*/
void QCamera3HardwareInterface::captureResultCb(mm_camera_super_buf_t *metadata_buf,
                camera3_stream_buffer_t *buffer, uint32_t frame_number)
{
    pthread_mutex_lock(&mMutex);
    if (metadata_buf)
        handleMetadataWithLock(metadata_buf);
    else
        handleBufferWithLock(buffer, frame_number);
    pthread_mutex_unlock(&mMutex);
    return;
}

/*===========================================================================
 * FUNCTION   : lookupFwkName
 *
 * DESCRIPTION: In case the enum is not same in fwk and backend
 *              make sure the parameter is correctly propogated
 *
 * PARAMETERS  :
 *   @arr      : map between the two enums
 *   @len      : len of the map
 *   @hal_name : name of the hal_parm to map
 *
 * RETURN     : int type of status
 *              fwk_name  -- success
 *              none-zero failure code
 *==========================================================================*/
template <typename halType, class mapType> int lookupFwkName(const mapType *arr,
        size_t len, halType hal_name)
{

    for (size_t i = 0; i < len; i++) {
        if (arr[i].hal_name == hal_name) {
            return arr[i].fwk_name;
        }
    }

    /* Not able to find matching framework type is not necessarily
     * an error case. This happens when mm-camera supports more attributes
     * than the frameworks do */
    CDBG_HIGH("%s: Cannot find matching framework type", __func__);
    return NAME_NOT_FOUND;
}

/*===========================================================================
 * FUNCTION   : lookupHalName
 *
 * DESCRIPTION: In case the enum is not same in fwk and backend
 *              make sure the parameter is correctly propogated
 *
 * PARAMETERS  :
 *   @arr      : map between the two enums
 *   @len      : len of the map
 *   @fwk_name : name of the hal_parm to map
 *
 * RETURN     : int32_t type of status
 *              hal_name  -- success
 *              none-zero failure code
 *==========================================================================*/
template <typename fwkType, class mapType> int lookupHalName(const mapType *arr,
        size_t len, fwkType fwk_name)
{
    for (size_t i = 0; i < len; i++) {
        if (arr[i].fwk_name == fwk_name) {
            return arr[i].hal_name;
        }
    }

    ALOGE("%s: Cannot find matching hal type fwk_name=%d", __func__, fwk_name);
    return NAME_NOT_FOUND;
}

/*===========================================================================
 * FUNCTION   : lookupProp
 *
 * DESCRIPTION: lookup a value by its name
 *
 * PARAMETERS :
 *   @arr     : map between the two enums
 *   @len     : size of the map
 *   @name    : name to be looked up
 *
 * RETURN     : Value if found
 *              CAM_CDS_MODE_MAX if not found
 *==========================================================================*/
template <class mapType> cam_cds_mode_type_t lookupProp(const mapType *arr,
        size_t len, const char *name)
{
    if (name) {
        for (size_t i = 0; i < len; i++) {
            if (!strcmp(arr[i].desc, name)) {
                return arr[i].val;
            }
        }
    }
    return CAM_CDS_MODE_MAX;
}

/*===========================================================================
 *
 * DESCRIPTION:
 *
 * PARAMETERS :
 *   @metadata : metadata information from callback
 *   @timestamp: metadata buffer timestamp
 *   @request_id: request id
 *   @jpegMetadata: additional jpeg metadata
 *
 * RETURN     : camera_metadata_t*
 *              metadata in a format specified by fwk
 *==========================================================================*/
camera_metadata_t*
QCamera3HardwareInterface::translateFromHalMetadata(
                                 metadata_buffer_t *metadata,
                                 nsecs_t timestamp,
                                 int32_t request_id,
                                 const CameraMetadata& jpegMetadata,
                                 uint8_t pipeline_depth,
                                 uint8_t capture_intent,
                                 uint8_t fwk_cacMode)
{
    CameraMetadata camMetadata;
    camera_metadata_t *resultMetadata;

    if (jpegMetadata.entryCount())
        camMetadata.append(jpegMetadata);

    camMetadata.update(ANDROID_SENSOR_TIMESTAMP, &timestamp, 1);
    camMetadata.update(ANDROID_REQUEST_ID, &request_id, 1);
    camMetadata.update(ANDROID_REQUEST_PIPELINE_DEPTH, &pipeline_depth, 1);
    camMetadata.update(ANDROID_CONTROL_CAPTURE_INTENT, &capture_intent, 1);

    IF_META_AVAILABLE(uint32_t, frame_number, CAM_INTF_META_FRAME_NUMBER, metadata) {
        int64_t fwk_frame_number = *frame_number;
        camMetadata.update(ANDROID_SYNC_FRAME_NUMBER, &fwk_frame_number, 1);
    }

    IF_META_AVAILABLE(cam_fps_range_t, float_range, CAM_INTF_PARM_FPS_RANGE, metadata) {
        int32_t fps_range[2];
        fps_range[0] = (int32_t)float_range->min_fps;
        fps_range[1] = (int32_t)float_range->max_fps;
        camMetadata.update(ANDROID_CONTROL_AE_TARGET_FPS_RANGE,
                                      fps_range, 2);
        CDBG("%s: urgent Metadata : ANDROID_CONTROL_AE_TARGET_FPS_RANGE [%d, %d]",
            __func__, fps_range[0], fps_range[1]);
    }

    IF_META_AVAILABLE(int32_t, expCompensation, CAM_INTF_PARM_EXPOSURE_COMPENSATION, metadata) {
        camMetadata.update(ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION, expCompensation, 1);
    }

    /* HFR and BEST_MODE need to be both available to derive SCENE_MODE
     * Framework sets scenemode to indicate HFR and hence corresponding
     * translatation is required from hfr mode to scenemode */
    int32_t hfrMode = CAM_HFR_MODE_OFF;
    uint32_t sceneMode = CAM_SCENE_MODE_OFF;

    IF_META_AVAILABLE(int32_t, pHfrMode, CAM_INTF_PARM_HFR, metadata) {
        hfrMode = *pHfrMode;
    }
    IF_META_AVAILABLE(uint32_t, pBestshotMode, CAM_INTF_PARM_BESTSHOT_MODE, metadata) {
        uint8_t fwkSceneMode;
        sceneMode = *pBestshotMode;

        if ((hfrMode != CAM_HFR_MODE_OFF) && (hfrMode < CAM_HFR_MODE_MAX))
            fwkSceneMode = ANDROID_CONTROL_SCENE_MODE_HIGH_SPEED_VIDEO;
        else {
            fwkSceneMode =
                    (uint8_t)lookupFwkName(SCENE_MODES_MAP,
                    sizeof(SCENE_MODES_MAP)/
                    sizeof(SCENE_MODES_MAP[0]), sceneMode);
        }
        camMetadata.update(ANDROID_CONTROL_SCENE_MODE,
                &fwkSceneMode, 1);
        CDBG("%s: urgent Metadata : ANDROID_CONTROL_SCENE_MODE: %d",
                __func__, fwkSceneMode);
    }

    IF_META_AVAILABLE(uint32_t, ae_lock, CAM_INTF_PARM_AEC_LOCK, metadata) {
        uint8_t fwk_ae_lock = (uint8_t) *ae_lock;
        camMetadata.update(ANDROID_CONTROL_AE_LOCK, &fwk_ae_lock, 1);
    }

    IF_META_AVAILABLE(uint32_t, awb_lock, CAM_INTF_PARM_AWB_LOCK, metadata) {
        uint8_t fwk_awb_lock = (uint8_t) *awb_lock;
        camMetadata.update(ANDROID_CONTROL_AWB_LOCK, &fwk_awb_lock, 1);
    }

    IF_META_AVAILABLE(uint32_t, color_correct_mode, CAM_INTF_META_COLOR_CORRECT_MODE, metadata) {
        uint8_t fwk_color_correct_mode = (uint8_t) *color_correct_mode;
        camMetadata.update(ANDROID_COLOR_CORRECTION_MODE, &fwk_color_correct_mode, 1);
    }

    IF_META_AVAILABLE(cam_edge_application_t, edgeApplication,
            CAM_INTF_META_EDGE_MODE, metadata) {
        uint8_t edgeStrength = (uint8_t) edgeApplication->sharpness;
        camMetadata.update(ANDROID_EDGE_MODE, &(edgeApplication->edge_mode), 1);
        camMetadata.update(ANDROID_EDGE_STRENGTH, &edgeStrength, 1);
    }

    IF_META_AVAILABLE(uint32_t, flashPower, CAM_INTF_META_FLASH_POWER, metadata) {
        uint8_t fwk_flashPower = (uint8_t) *flashPower;
        camMetadata.update(ANDROID_FLASH_FIRING_POWER, &fwk_flashPower, 1);
    }

    IF_META_AVAILABLE(int64_t, flashFiringTime, CAM_INTF_META_FLASH_FIRING_TIME, metadata) {
        camMetadata.update(ANDROID_FLASH_FIRING_TIME, flashFiringTime, 1);
    }

    IF_META_AVAILABLE(int32_t, flashState, CAM_INTF_META_FLASH_STATE, metadata) {
        if (0 <= *flashState) {
            uint8_t fwk_flashState = (uint8_t) *flashState;
            if (!gCamCapability[mCameraId]->flash_available) {
                fwk_flashState = ANDROID_FLASH_STATE_UNAVAILABLE;
            }
            camMetadata.update(ANDROID_FLASH_STATE, &fwk_flashState, 1);
        }
    }

    IF_META_AVAILABLE(uint32_t, flashMode, CAM_INTF_META_FLASH_MODE, metadata) {
        int val = lookupFwkName(FLASH_MODES_MAP, METADATA_MAP_SIZE(FLASH_MODES_MAP), *flashMode);
        if (NAME_NOT_FOUND != val) {
            uint8_t fwk_flashMode = (uint8_t)val;
            camMetadata.update(ANDROID_FLASH_MODE, &fwk_flashMode, 1);
        }
    }

    IF_META_AVAILABLE(uint32_t, hotPixelMode, CAM_INTF_META_HOTPIXEL_MODE, metadata) {
        uint8_t fwk_hotPixelMode = (uint8_t) *hotPixelMode;
        camMetadata.update(ANDROID_HOT_PIXEL_MODE, &fwk_hotPixelMode, 1);
    }

    IF_META_AVAILABLE(float, lensAperture, CAM_INTF_META_LENS_APERTURE, metadata) {
        camMetadata.update(ANDROID_LENS_APERTURE , lensAperture, 1);
    }

    IF_META_AVAILABLE(float, filterDensity, CAM_INTF_META_LENS_FILTERDENSITY, metadata) {
        camMetadata.update(ANDROID_LENS_FILTER_DENSITY , filterDensity, 1);
    }

    IF_META_AVAILABLE(float, focalLength, CAM_INTF_META_LENS_FOCAL_LENGTH, metadata) {
        camMetadata.update(ANDROID_LENS_FOCAL_LENGTH, focalLength, 1);
    }

    IF_META_AVAILABLE(uint32_t, opticalStab, CAM_INTF_META_LENS_OPT_STAB_MODE, metadata) {
        uint8_t fwk_opticalStab = (uint8_t) *opticalStab;
        camMetadata.update(ANDROID_LENS_OPTICAL_STABILIZATION_MODE, &fwk_opticalStab, 1);
    }

    /*EIS is currently not hooked up to the app, so set the mode to OFF*/
    uint8_t vsMode = ANDROID_CONTROL_VIDEO_STABILIZATION_MODE_OFF;
    camMetadata.update(ANDROID_CONTROL_VIDEO_STABILIZATION_MODE, &vsMode, 1);

    IF_META_AVAILABLE(uint32_t, noiseRedMode, CAM_INTF_META_NOISE_REDUCTION_MODE, metadata) {
        uint8_t fwk_noiseRedMode = (uint8_t) *noiseRedMode;
        camMetadata.update(ANDROID_NOISE_REDUCTION_MODE, &fwk_noiseRedMode, 1);
    }

    IF_META_AVAILABLE(uint32_t, noiseRedStrength, CAM_INTF_META_NOISE_REDUCTION_STRENGTH, metadata) {
        uint8_t fwk_noiseRedStrength = (uint8_t) *noiseRedStrength;
        camMetadata.update(ANDROID_NOISE_REDUCTION_STRENGTH, &fwk_noiseRedStrength, 1);
    }

    IF_META_AVAILABLE(cam_crop_region_t, hScalerCropRegion,
            CAM_INTF_META_SCALER_CROP_REGION, metadata) {
        int32_t scalerCropRegion[4];
        scalerCropRegion[0] = hScalerCropRegion->left;
        scalerCropRegion[1] = hScalerCropRegion->top;
        scalerCropRegion[2] = hScalerCropRegion->width;
        scalerCropRegion[3] = hScalerCropRegion->height;

        // Adjust crop region from sensor output coordinate system to active
        // array coordinate system.
        mCropRegionMapper.toActiveArray(scalerCropRegion[0], scalerCropRegion[1],
                scalerCropRegion[2], scalerCropRegion[3]);

        camMetadata.update(ANDROID_SCALER_CROP_REGION, scalerCropRegion, 4);
    }

    IF_META_AVAILABLE(int64_t, sensorExpTime, CAM_INTF_META_SENSOR_EXPOSURE_TIME, metadata) {
        CDBG("%s: sensorExpTime = %lld", __func__, *sensorExpTime);
        camMetadata.update(ANDROID_SENSOR_EXPOSURE_TIME , sensorExpTime, 1);
    }

    IF_META_AVAILABLE(int64_t, sensorFameDuration,
            CAM_INTF_META_SENSOR_FRAME_DURATION, metadata) {
        CDBG("%s: sensorFameDuration = %lld", __func__, *sensorFameDuration);
        camMetadata.update(ANDROID_SENSOR_FRAME_DURATION, sensorFameDuration, 1);
    }

    IF_META_AVAILABLE(int64_t, sensorRollingShutterSkew,
            CAM_INTF_META_SENSOR_ROLLING_SHUTTER_SKEW, metadata) {
        CDBG("%s: sensorRollingShutterSkew = %lld", __func__, *sensorRollingShutterSkew);
        camMetadata.update(ANDROID_SENSOR_ROLLING_SHUTTER_SKEW,
                sensorRollingShutterSkew, 1);
    }

    IF_META_AVAILABLE(int32_t, sensorSensitivity, CAM_INTF_META_SENSOR_SENSITIVITY, metadata) {
        CDBG("%s: sensorSensitivity = %d", __func__, *sensorSensitivity);
        camMetadata.update(ANDROID_SENSOR_SENSITIVITY, sensorSensitivity, 1);

        //calculate the noise profile based on sensitivity
        double noise_profile_S = computeNoiseModelEntryS(*sensorSensitivity);
        double noise_profile_O = computeNoiseModelEntryO(*sensorSensitivity);
        double noise_profile[2 * gCamCapability[mCameraId]->num_color_channels];
        for (int i = 0; i < 2 * gCamCapability[mCameraId]->num_color_channels; i += 2) {
            noise_profile[i]   = noise_profile_S;
            noise_profile[i+1] = noise_profile_O;
        }
        CDBG("%s: noise model entry (S, O) is (%f, %f)", __func__,
                noise_profile_S, noise_profile_O);
        camMetadata.update(ANDROID_SENSOR_NOISE_PROFILE, noise_profile,
                (size_t) (2 * gCamCapability[mCameraId]->num_color_channels));
    }

    IF_META_AVAILABLE(uint32_t, shadingMode, CAM_INTF_META_SHADING_MODE, metadata) {
        uint8_t fwk_shadingMode = (uint8_t) *shadingMode;
        camMetadata.update(ANDROID_SHADING_MODE, &fwk_shadingMode, 1);
    }

    IF_META_AVAILABLE(uint32_t, faceDetectMode, CAM_INTF_META_STATS_FACEDETECT_MODE, metadata) {
        int val = lookupFwkName(FACEDETECT_MODES_MAP, METADATA_MAP_SIZE(FACEDETECT_MODES_MAP),
                *faceDetectMode);
        if (NAME_NOT_FOUND != val) {
            uint8_t fwk_faceDetectMode = (uint8_t)val;
            camMetadata.update(ANDROID_STATISTICS_FACE_DETECT_MODE, &fwk_faceDetectMode, 1);
            if (fwk_faceDetectMode != ANDROID_STATISTICS_FACE_DETECT_MODE_OFF) {
                IF_META_AVAILABLE(cam_face_detection_data_t, faceDetectionInfo,
                            CAM_INTF_META_FACE_DETECTION, metadata) {
                    uint8_t numFaces = MIN(
                            faceDetectionInfo->num_faces_detected, MAX_ROI);
                    int32_t faceIds[MAX_ROI];
                    uint8_t faceScores[MAX_ROI];
                    int32_t faceRectangles[MAX_ROI * 4];
                    int32_t faceLandmarks[MAX_ROI * 6];
                    size_t j = 0, k = 0;

                    for (size_t i = 0; i < numFaces; i++) {
                        faceScores[i] = (uint8_t)faceDetectionInfo->faces[i].score;
                        // Adjust crop region from sensor output coordinate system to active
                        // array coordinate system.
                        cam_rect_t& rect = faceDetectionInfo->faces[i].face_boundary;
                        mCropRegionMapper.toActiveArray(rect.left, rect.top,
                                 rect.width, rect.height);

                        convertToRegions(faceDetectionInfo->faces[i].face_boundary,
                                 faceRectangles+j, -1);

                         // Map the co-ordinate sensor output coordinate system to active
                         // array coordinate system.
                         cam_face_detection_info_t& face = faceDetectionInfo->faces[i];
                         mCropRegionMapper.toActiveArray(face.left_eye_center.x,
                                 face.left_eye_center.y);
                         mCropRegionMapper.toActiveArray(face.right_eye_center.x,
                                 face.right_eye_center.y);
                         mCropRegionMapper.toActiveArray(face.mouth_center.x,
                                 face.mouth_center.y);

                         convertLandmarks(faceDetectionInfo->faces[i], faceLandmarks+k);
                         j+= 4;
                         k+= 6;
                    }
                    if (numFaces <= 0) {
                        memset(faceIds, 0, sizeof(int32_t) * MAX_ROI);
                        memset(faceScores, 0, sizeof(uint8_t) * MAX_ROI);
                        memset(faceRectangles, 0, sizeof(int32_t) * MAX_ROI * 4);
                        memset(faceLandmarks, 0, sizeof(int32_t) * MAX_ROI * 6);
                    }

                    camMetadata.update(ANDROID_STATISTICS_FACE_SCORES, faceScores,
                            numFaces);
                    camMetadata.update(ANDROID_STATISTICS_FACE_RECTANGLES,
                            faceRectangles, numFaces * 4U);
                    if (fwk_faceDetectMode ==
                            ANDROID_STATISTICS_FACE_DETECT_MODE_FULL) {
                        camMetadata.update(ANDROID_STATISTICS_FACE_IDS, faceIds, numFaces);
                        camMetadata.update(ANDROID_STATISTICS_FACE_LANDMARKS,
                                faceLandmarks, numFaces * 6U);
                    }
                }
            }
        }
    }

    IF_META_AVAILABLE(uint32_t, histogramMode, CAM_INTF_META_STATS_HISTOGRAM_MODE, metadata) {
        uint8_t fwk_histogramMode = (uint8_t) *histogramMode;
        camMetadata.update(ANDROID_STATISTICS_HISTOGRAM_MODE, &fwk_histogramMode, 1);
    }

    IF_META_AVAILABLE(uint32_t, sharpnessMapMode,
            CAM_INTF_META_STATS_SHARPNESS_MAP_MODE, metadata) {
        uint8_t fwk_sharpnessMapMode = (uint8_t) *sharpnessMapMode;
        camMetadata.update(ANDROID_STATISTICS_SHARPNESS_MAP_MODE, &fwk_sharpnessMapMode, 1);
    }

    IF_META_AVAILABLE(cam_sharpness_map_t, sharpnessMap,
            CAM_INTF_META_STATS_SHARPNESS_MAP, metadata) {
        camMetadata.update(ANDROID_STATISTICS_SHARPNESS_MAP, (int32_t *)sharpnessMap->sharpness,
                CAM_MAX_MAP_WIDTH * CAM_MAX_MAP_HEIGHT * 3);
    }

    IF_META_AVAILABLE(cam_lens_shading_map_t, lensShadingMap,
            CAM_INTF_META_LENS_SHADING_MAP, metadata) {
        size_t map_height = MIN((size_t)gCamCapability[mCameraId]->lens_shading_map_size.height,
                CAM_MAX_SHADING_MAP_HEIGHT);
        size_t map_width = MIN((size_t)gCamCapability[mCameraId]->lens_shading_map_size.width,
                CAM_MAX_SHADING_MAP_WIDTH);
        camMetadata.update(ANDROID_STATISTICS_LENS_SHADING_MAP,
                lensShadingMap->lens_shading, 4U * map_width * map_height);
    }

    IF_META_AVAILABLE(uint32_t, toneMapMode, CAM_INTF_META_TONEMAP_MODE, metadata) {
        uint8_t fwk_toneMapMode = (uint8_t) *toneMapMode;
        camMetadata.update(ANDROID_TONEMAP_MODE, &fwk_toneMapMode, 1);
    }

    IF_META_AVAILABLE(cam_rgb_tonemap_curves, tonemap, CAM_INTF_META_TONEMAP_CURVES, metadata) {
        //Populate CAM_INTF_META_TONEMAP_CURVES
        /* ch0 = G, ch 1 = B, ch 2 = R*/
        if (tonemap->tonemap_points_cnt > CAM_MAX_TONEMAP_CURVE_SIZE) {
            ALOGE("%s: Fatal: tonemap_points_cnt %d exceeds max value of %d",
                    __func__, tonemap->tonemap_points_cnt,
                    CAM_MAX_TONEMAP_CURVE_SIZE);
            tonemap->tonemap_points_cnt = CAM_MAX_TONEMAP_CURVE_SIZE;
        }

        camMetadata.update(ANDROID_TONEMAP_CURVE_GREEN,
                        &tonemap->curves[0].tonemap_points[0][0],
                        tonemap->tonemap_points_cnt * 2);

        camMetadata.update(ANDROID_TONEMAP_CURVE_BLUE,
                        &tonemap->curves[1].tonemap_points[0][0],
                        tonemap->tonemap_points_cnt * 2);

        camMetadata.update(ANDROID_TONEMAP_CURVE_RED,
                        &tonemap->curves[2].tonemap_points[0][0],
                        tonemap->tonemap_points_cnt * 2);
    }

    IF_META_AVAILABLE(cam_color_correct_gains_t, colorCorrectionGains,
            CAM_INTF_META_COLOR_CORRECT_GAINS, metadata) {
        camMetadata.update(ANDROID_COLOR_CORRECTION_GAINS, colorCorrectionGains->gains,
                CC_GAINS_COUNT);
    }

    IF_META_AVAILABLE(cam_color_correct_matrix_t, colorCorrectionMatrix,
            CAM_INTF_META_COLOR_CORRECT_TRANSFORM, metadata) {
        camMetadata.update(ANDROID_COLOR_CORRECTION_TRANSFORM,
                (camera_metadata_rational_t *)(void *)colorCorrectionMatrix->transform_matrix,
                CC_MATRIX_COLS * CC_MATRIX_ROWS);
    }

    IF_META_AVAILABLE(cam_profile_tone_curve, toneCurve,
            CAM_INTF_META_PROFILE_TONE_CURVE, metadata) {
        if (toneCurve->tonemap_points_cnt > CAM_MAX_TONEMAP_CURVE_SIZE) {
            ALOGE("%s: Fatal: tonemap_points_cnt %d exceeds max value of %d",
                    __func__, toneCurve->tonemap_points_cnt,
                    CAM_MAX_TONEMAP_CURVE_SIZE);
            toneCurve->tonemap_points_cnt = CAM_MAX_TONEMAP_CURVE_SIZE;
        }
        camMetadata.update(ANDROID_SENSOR_PROFILE_TONE_CURVE,
                (float*)toneCurve->curve.tonemap_points,
                toneCurve->tonemap_points_cnt * 2);
    }

    IF_META_AVAILABLE(cam_color_correct_gains_t, predColorCorrectionGains,
            CAM_INTF_META_PRED_COLOR_CORRECT_GAINS, metadata) {
        camMetadata.update(ANDROID_STATISTICS_PREDICTED_COLOR_GAINS,
                predColorCorrectionGains->gains, 4);
    }

    IF_META_AVAILABLE(cam_color_correct_matrix_t, predColorCorrectionMatrix,
            CAM_INTF_META_PRED_COLOR_CORRECT_TRANSFORM, metadata) {
        camMetadata.update(ANDROID_STATISTICS_PREDICTED_COLOR_TRANSFORM,
                (camera_metadata_rational_t *)(void *)predColorCorrectionMatrix->transform_matrix,
                CC_MATRIX_ROWS * CC_MATRIX_COLS);
    }

    IF_META_AVAILABLE(float, otpWbGrGb, CAM_INTF_META_OTP_WB_GRGB, metadata) {
        camMetadata.update(ANDROID_SENSOR_GREEN_SPLIT, otpWbGrGb, 1);
    }

    IF_META_AVAILABLE(uint32_t, blackLevelLock, CAM_INTF_META_BLACK_LEVEL_LOCK, metadata) {
        uint8_t fwk_blackLevelLock = (uint8_t) *blackLevelLock;
        camMetadata.update(ANDROID_BLACK_LEVEL_LOCK, &fwk_blackLevelLock, 1);
    }

    IF_META_AVAILABLE(uint32_t, sceneFlicker, CAM_INTF_META_SCENE_FLICKER, metadata) {
        uint8_t fwk_sceneFlicker = (uint8_t) *sceneFlicker;
        camMetadata.update(ANDROID_STATISTICS_SCENE_FLICKER, &fwk_sceneFlicker, 1);
    }

    IF_META_AVAILABLE(uint32_t, effectMode, CAM_INTF_PARM_EFFECT, metadata) {
        int val = lookupFwkName(EFFECT_MODES_MAP, METADATA_MAP_SIZE(EFFECT_MODES_MAP),
                *effectMode);
        if (NAME_NOT_FOUND != val) {
            uint8_t fwk_effectMode = (uint8_t)val;
            camMetadata.update(ANDROID_CONTROL_EFFECT_MODE, &fwk_effectMode, 1);
        }
    }

    IF_META_AVAILABLE(cam_test_pattern_data_t, testPatternData,
            CAM_INTF_META_TEST_PATTERN_DATA, metadata) {
        int32_t fwk_testPatternMode = lookupFwkName(TEST_PATTERN_MAP,
                METADATA_MAP_SIZE(TEST_PATTERN_MAP), testPatternData->mode);
        if (NAME_NOT_FOUND != fwk_testPatternMode) {
            camMetadata.update(ANDROID_SENSOR_TEST_PATTERN_MODE, &fwk_testPatternMode, 1);
        }
        int32_t fwk_testPatternData[4];
        fwk_testPatternData[0] = testPatternData->r;
        fwk_testPatternData[3] = testPatternData->b;
        switch (gCamCapability[mCameraId]->color_arrangement) {
        case CAM_FILTER_ARRANGEMENT_RGGB:
        case CAM_FILTER_ARRANGEMENT_GRBG:
            fwk_testPatternData[1] = testPatternData->gr;
            fwk_testPatternData[2] = testPatternData->gb;
            break;
        case CAM_FILTER_ARRANGEMENT_GBRG:
        case CAM_FILTER_ARRANGEMENT_BGGR:
            fwk_testPatternData[2] = testPatternData->gr;
            fwk_testPatternData[1] = testPatternData->gb;
            break;
        default:
            ALOGE("%s: color arrangement %d is not supported", __func__,
                gCamCapability[mCameraId]->color_arrangement);
            break;
        }
        camMetadata.update(ANDROID_SENSOR_TEST_PATTERN_DATA, fwk_testPatternData, 4);
    }

    IF_META_AVAILABLE(double, gps_coords, CAM_INTF_META_JPEG_GPS_COORDINATES, metadata) {
        camMetadata.update(ANDROID_JPEG_GPS_COORDINATES, gps_coords, 3);
    }

    IF_META_AVAILABLE(uint8_t, gps_methods, CAM_INTF_META_JPEG_GPS_PROC_METHODS, metadata) {
        String8 str((const char *)gps_methods);
        camMetadata.update(ANDROID_JPEG_GPS_PROCESSING_METHOD, str);
    }

    IF_META_AVAILABLE(int64_t, gps_timestamp, CAM_INTF_META_JPEG_GPS_TIMESTAMP, metadata) {
        camMetadata.update(ANDROID_JPEG_GPS_TIMESTAMP, gps_timestamp, 1);
    }

    IF_META_AVAILABLE(int32_t, jpeg_orientation, CAM_INTF_META_JPEG_ORIENTATION, metadata) {
        camMetadata.update(ANDROID_JPEG_ORIENTATION, jpeg_orientation, 1);
    }

    IF_META_AVAILABLE(uint32_t, jpeg_quality, CAM_INTF_META_JPEG_QUALITY, metadata) {
        uint8_t fwk_jpeg_quality = (uint8_t) *jpeg_quality;
        camMetadata.update(ANDROID_JPEG_QUALITY, &fwk_jpeg_quality, 1);
    }

    IF_META_AVAILABLE(uint32_t, thumb_quality, CAM_INTF_META_JPEG_THUMB_QUALITY, metadata) {
        uint8_t fwk_thumb_quality = (uint8_t) *thumb_quality;
        camMetadata.update(ANDROID_JPEG_THUMBNAIL_QUALITY, &fwk_thumb_quality, 1);
    }

    IF_META_AVAILABLE(cam_dimension_t, thumb_size, CAM_INTF_META_JPEG_THUMB_SIZE, metadata) {
        int32_t fwk_thumb_size[2];
        fwk_thumb_size[0] = thumb_size->width;
        fwk_thumb_size[1] = thumb_size->height;
        camMetadata.update(ANDROID_JPEG_THUMBNAIL_SIZE, fwk_thumb_size, 2);
    }

    IF_META_AVAILABLE(int32_t, privateData, CAM_INTF_META_PRIVATE_DATA, metadata) {
        camMetadata.update(QCAMERA3_PRIVATEDATA_REPROCESS,
                privateData,
                MAX_METADATA_PRIVATE_PAYLOAD_SIZE_IN_BYTES / sizeof(int32_t));
    }

    if (metadata->is_tuning_params_valid) {
        uint8_t tuning_meta_data_blob[sizeof(tuning_params_t)];
        uint8_t *data = (uint8_t *)&tuning_meta_data_blob[0];
        metadata->tuning_params.tuning_data_version = TUNING_DATA_VERSION;


        memcpy(data, ((uint8_t *)&metadata->tuning_params.tuning_data_version),
                sizeof(uint32_t));
        data += sizeof(uint32_t);

        memcpy(data, ((uint8_t *)&metadata->tuning_params.tuning_sensor_data_size),
                sizeof(uint32_t));
        CDBG("tuning_sensor_data_size %d",(int)(*(int *)data));
        data += sizeof(uint32_t);

        memcpy(data, ((uint8_t *)&metadata->tuning_params.tuning_vfe_data_size),
                sizeof(uint32_t));
        CDBG("tuning_vfe_data_size %d",(int)(*(int *)data));
        data += sizeof(uint32_t);

        memcpy(data, ((uint8_t *)&metadata->tuning_params.tuning_cpp_data_size),
                sizeof(uint32_t));
        CDBG("tuning_cpp_data_size %d",(int)(*(int *)data));
        data += sizeof(uint32_t);

        memcpy(data, ((uint8_t *)&metadata->tuning_params.tuning_cac_data_size),
                sizeof(uint32_t));
        CDBG("tuning_cac_data_size %d",(int)(*(int *)data));
        data += sizeof(uint32_t);

        metadata->tuning_params.tuning_mod3_data_size = 0;
        memcpy(data, ((uint8_t *)&metadata->tuning_params.tuning_mod3_data_size),
                sizeof(uint32_t));
        CDBG("tuning_mod3_data_size %d",(int)(*(int *)data));
        data += sizeof(uint32_t);

        size_t count = MIN(metadata->tuning_params.tuning_sensor_data_size,
                TUNING_SENSOR_DATA_MAX);
        memcpy(data, ((uint8_t *)&metadata->tuning_params.data),
                count);
        data += count;

        count = MIN(metadata->tuning_params.tuning_vfe_data_size,
                TUNING_VFE_DATA_MAX);
        memcpy(data, ((uint8_t *)&metadata->tuning_params.data[TUNING_VFE_DATA_OFFSET]),
                count);
        data += count;

        count = MIN(metadata->tuning_params.tuning_cpp_data_size,
                TUNING_CPP_DATA_MAX);
        memcpy(data, ((uint8_t *)&metadata->tuning_params.data[TUNING_CPP_DATA_OFFSET]),
                count);
        data += count;

        count = MIN(metadata->tuning_params.tuning_cac_data_size,
                TUNING_CAC_DATA_MAX);
        memcpy(data, ((uint8_t *)&metadata->tuning_params.data[TUNING_CAC_DATA_OFFSET]),
                count);
        data += count;

        camMetadata.update(QCAMERA3_TUNING_META_DATA_BLOB,
                (int32_t *)(void *)tuning_meta_data_blob,
                (size_t)(data-tuning_meta_data_blob) / sizeof(uint32_t));
    }

    IF_META_AVAILABLE(cam_neutral_col_point_t, neuColPoint,
            CAM_INTF_META_NEUTRAL_COL_POINT, metadata) {
        camMetadata.update(ANDROID_SENSOR_NEUTRAL_COLOR_POINT,
                (camera_metadata_rational_t *)(void *)neuColPoint->neutral_col_point,
                NEUTRAL_COL_POINTS);
    }

    IF_META_AVAILABLE(uint32_t, shadingMapMode, CAM_INTF_META_LENS_SHADING_MAP_MODE, metadata) {
        uint8_t fwk_shadingMapMode = (uint8_t) *shadingMapMode;
        camMetadata.update(ANDROID_STATISTICS_LENS_SHADING_MAP_MODE, &fwk_shadingMapMode, 1);
    }

    IF_META_AVAILABLE(cam_area_t, hAeRegions, CAM_INTF_META_AEC_ROI, metadata) {
        int32_t aeRegions[REGIONS_TUPLE_COUNT];
        // Adjust crop region from sensor output coordinate system to active
        // array coordinate system.
        mCropRegionMapper.toActiveArray(hAeRegions->rect.left, hAeRegions->rect.top,
                hAeRegions->rect.width, hAeRegions->rect.height);

        convertToRegions(hAeRegions->rect, aeRegions, hAeRegions->weight);
        camMetadata.update(ANDROID_CONTROL_AE_REGIONS, aeRegions,
                REGIONS_TUPLE_COUNT);
        CDBG("%s: Metadata : ANDROID_CONTROL_AE_REGIONS: FWK: [%d,%d,%d,%d] HAL: [%d,%d,%d,%d]",
                __func__, aeRegions[0], aeRegions[1], aeRegions[2], aeRegions[3],
                hAeRegions->rect.left, hAeRegions->rect.top, hAeRegions->rect.width,
                hAeRegions->rect.height);
    }

    IF_META_AVAILABLE(cam_area_t, hAfRegions, CAM_INTF_META_AF_ROI, metadata) {
        /*af regions*/
        int32_t afRegions[REGIONS_TUPLE_COUNT];
        // Adjust crop region from sensor output coordinate system to active
        // array coordinate system.
        mCropRegionMapper.toActiveArray(hAfRegions->rect.left, hAfRegions->rect.top,
                hAfRegions->rect.width, hAfRegions->rect.height);

        convertToRegions(hAfRegions->rect, afRegions, hAfRegions->weight);
        camMetadata.update(ANDROID_CONTROL_AF_REGIONS, afRegions,
                REGIONS_TUPLE_COUNT);
        CDBG("%s: Metadata : ANDROID_CONTROL_AF_REGIONS: FWK: [%d,%d,%d,%d] HAL: [%d,%d,%d,%d]",
                __func__, afRegions[0], afRegions[1], afRegions[2], afRegions[3],
                hAfRegions->rect.left, hAfRegions->rect.top, hAfRegions->rect.width,
                hAfRegions->rect.height);
    }

    IF_META_AVAILABLE(uint32_t, hal_ab_mode, CAM_INTF_PARM_ANTIBANDING, metadata) {
        int val = lookupFwkName(ANTIBANDING_MODES_MAP, METADATA_MAP_SIZE(ANTIBANDING_MODES_MAP),
                *hal_ab_mode);
        if (NAME_NOT_FOUND != val) {
            uint8_t fwk_ab_mode = (uint8_t)val;
            camMetadata.update(ANDROID_CONTROL_AE_ANTIBANDING_MODE, &fwk_ab_mode, 1);
        }
    }

    IF_META_AVAILABLE(uint32_t, bestshotMode, CAM_INTF_PARM_BESTSHOT_MODE, metadata) {
        int val = lookupFwkName(SCENE_MODES_MAP,
                METADATA_MAP_SIZE(SCENE_MODES_MAP), *bestshotMode);
        if (NAME_NOT_FOUND != val) {
            uint8_t fwkBestshotMode = (uint8_t)val;
            camMetadata.update(ANDROID_CONTROL_SCENE_MODE, &fwkBestshotMode, 1);
            CDBG("%s: Metadata : ANDROID_CONTROL_SCENE_MODE", __func__);
        } else {
            CDBG_HIGH("%s: Metadata not found : ANDROID_CONTROL_SCENE_MODE", __func__);
        }
    }

    IF_META_AVAILABLE(uint32_t, mode, CAM_INTF_META_MODE, metadata) {
         uint8_t fwk_mode = (uint8_t) *mode;
         camMetadata.update(ANDROID_CONTROL_MODE, &fwk_mode, 1);
    }

    /* Constant metadata values to be update*/
    uint8_t hotPixelModeFast = ANDROID_HOT_PIXEL_MODE_FAST;
    camMetadata.update(ANDROID_HOT_PIXEL_MODE, &hotPixelModeFast, 1);

    uint8_t hotPixelMapMode = ANDROID_STATISTICS_HOT_PIXEL_MAP_MODE_OFF;
    camMetadata.update(ANDROID_STATISTICS_HOT_PIXEL_MAP_MODE, &hotPixelMapMode, 1);

    int32_t hotPixelMap[2];
    camMetadata.update(ANDROID_STATISTICS_HOT_PIXEL_MAP, &hotPixelMap[0], 0);

    // CDS
    IF_META_AVAILABLE(int32_t, cds, CAM_INTF_PARM_CDS_MODE, metadata) {
        camMetadata.update(QCAMERA3_CDS_MODE, cds, 1);
    }

    // TNR
    IF_META_AVAILABLE(cam_denoise_param_t, tnr, CAM_INTF_PARM_TEMPORAL_DENOISE, metadata) {
        uint8_t tnr_enable       = tnr->denoise_enable;
        int32_t tnr_process_type = (int32_t)tnr->process_plates;

        camMetadata.update(QCAMERA3_TEMPORAL_DENOISE_ENABLE, &tnr_enable, 1);
        camMetadata.update(QCAMERA3_TEMPORAL_DENOISE_PROCESS_TYPE, &tnr_process_type, 1);
    }

    // Reprocess crop data
    IF_META_AVAILABLE(cam_crop_data_t, crop_data, CAM_INTF_META_CROP_DATA, metadata) {
        uint8_t cnt = crop_data->num_of_streams;
        if ((0 < cnt) && (cnt < MAX_NUM_STREAMS)) {
            int rc = NO_ERROR;
            int32_t *crop = new int32_t[cnt*4];
            if (NULL == crop) {
                rc = NO_MEMORY;
            }

            int32_t *crop_stream_ids = new int32_t[cnt];
            if (NULL == crop_stream_ids) {
                rc = NO_MEMORY;
            }

            Vector<int32_t> roi_map;

            if (NO_ERROR == rc) {
                int32_t steams_found = 0;
                for (size_t i = 0; i < cnt; i++) {
                    for (List<stream_info_t *>::iterator it = mStreamInfo.begin();
                        it != mStreamInfo.end(); it++) {
                        QCamera3Channel *channel = (QCamera3Channel *)(*it)->stream->priv;
                        if (NULL != channel) {
                            if (crop_data->crop_info[i].stream_id ==
                                    channel->mStreams[0]->getMyServerID()) {
                                crop[steams_found*4] = crop_data->crop_info[i].crop.left;
                                crop[steams_found*4 + 1] = crop_data->crop_info[i].crop.top;
                                crop[steams_found*4 + 2] = crop_data->crop_info[i].crop.width;
                                crop[steams_found*4 + 3] = crop_data->crop_info[i].crop.height;
                                // In a more general case we may want to generate
                                // unique id depending on width, height, stream, private
                                // data etc.
#ifdef __LP64__
                                // Using XORed value of lower and upper halves as ID
                                crop_stream_ids[steams_found] = (int32_t)
                                        ((((int64_t)(*it)->stream) & 0x0000FFFF) ^
                                                (((int64_t)(*it)->stream) >> 0x20 & 0x0000FFFF));
#else
                                // FIXME: Although using data address as ID doesn't guarantee
                                // that all IDs will be unique, we are keeping existing nostrum
                                // for now till found better solution.
                                crop_stream_ids[steams_found] = (int32_t)(*it)->stream;
#endif
                                steams_found++;
                                roi_map.add(crop_data->crop_info[i].roi_map.left);
                                roi_map.add(crop_data->crop_info[i].roi_map.top);
                                roi_map.add(crop_data->crop_info[i].roi_map.width);
                                roi_map.add(crop_data->crop_info[i].roi_map.height);
                                CDBG("%s: Adding reprocess crop data for stream %p %dx%d, %dx%d",
                                        __func__,
                                        (*it)->stream,
                                        crop_data->crop_info[i].crop.left,
                                        crop_data->crop_info[i].crop.top,
                                        crop_data->crop_info[i].crop.width,
                                        crop_data->crop_info[i].crop.height);
                                CDBG("%s: Adding reprocess crop roi map for stream %p %dx%d, %dx%d",
                                        __func__,
                                        (*it)->stream,
                                        crop_data->crop_info[i].roi_map.left,
                                        crop_data->crop_info[i].roi_map.top,
                                        crop_data->crop_info[i].roi_map.width,
                                        crop_data->crop_info[i].roi_map.height);
                                break;
                            }
                        }
                    }
                }

                camMetadata.update(QCAMERA3_CROP_COUNT_REPROCESS,
                        &steams_found, 1);
                camMetadata.update(QCAMERA3_CROP_REPROCESS,
                        crop, (size_t)(steams_found * 4));
                camMetadata.update(QCAMERA3_CROP_STREAM_ID_REPROCESS,
                        crop_stream_ids, (size_t)steams_found);
                if (roi_map.array()) {
                    camMetadata.update(QCAMERA3_CROP_ROI_MAP_REPROCESS,
                            roi_map.array(), roi_map.size());
                }
            }

            if (crop) {
                delete [] crop;
            }
            if (crop_stream_ids) {
                delete [] crop_stream_ids;
            }
        } else {
            // mm-qcamera-daemon only posts crop_data for streams
            // not linked to pproc. So no valid crop metadata is not
            // necessarily an error case.
            CDBG("%s: No valid crop metadata entries", __func__);
        }
    }

    if (gCamCapability[mCameraId]->aberration_modes_count == 0) {
        // Regardless of CAC supports or not, CTS is expecting the CAC result to be non NULL and
        // so hardcoding the CAC result to OFF mode.
        uint8_t fwkCacMode = ANDROID_COLOR_CORRECTION_ABERRATION_MODE_OFF;
        camMetadata.update(ANDROID_COLOR_CORRECTION_ABERRATION_MODE, &fwkCacMode, 1);
    } else {
        IF_META_AVAILABLE(cam_aberration_mode_t, cacMode, CAM_INTF_PARM_CAC, metadata) {
            int val = lookupFwkName(COLOR_ABERRATION_MAP, METADATA_MAP_SIZE(COLOR_ABERRATION_MAP),
                    *cacMode);
            if (NAME_NOT_FOUND != val) {
                uint8_t resultCacMode = (uint8_t)val;
                // check whether CAC result from CB is equal to Framework set CAC mode
                // If not equal then set the CAC mode came in corresponding request
                if (fwk_cacMode != resultCacMode) {
                    resultCacMode = fwk_cacMode;
                }
                CDBG("%s: fwk_cacMode=%d resultCacMode=%d", __func__, fwk_cacMode, resultCacMode);
                camMetadata.update(ANDROID_COLOR_CORRECTION_ABERRATION_MODE, &resultCacMode, 1);
            } else {
                ALOGE("%s: Invalid CAC camera parameter: %d", __func__, *cacMode);
            }
        }
    }

    resultMetadata = camMetadata.release();
    return resultMetadata;
}

/*===========================================================================
 * FUNCTION   : saveExifParams
 *
 * DESCRIPTION:
 *
 * PARAMETERS :
 *   @metadata : metadata information from callback
 *
 * RETURN     : none
 *
 *==========================================================================*/
void QCamera3HardwareInterface::saveExifParams(metadata_buffer_t *metadata)
{
    IF_META_AVAILABLE(cam_ae_exif_debug_t, ae_exif_debug_params,
            CAM_INTF_META_EXIF_DEBUG_AE, metadata) {
        if (mExifParams.debug_params) {
            mExifParams.debug_params->ae_debug_params = *ae_exif_debug_params;
            mExifParams.debug_params->ae_debug_params_valid = TRUE;
        }
    }
    IF_META_AVAILABLE(cam_awb_exif_debug_t,awb_exif_debug_params,
            CAM_INTF_META_EXIF_DEBUG_AWB, metadata) {
        if (mExifParams.debug_params) {
            mExifParams.debug_params->awb_debug_params = *awb_exif_debug_params;
            mExifParams.debug_params->awb_debug_params_valid = TRUE;
        }
    }
    IF_META_AVAILABLE(cam_af_exif_debug_t,af_exif_debug_params,
            CAM_INTF_META_EXIF_DEBUG_AF, metadata) {
        if (mExifParams.debug_params) {
            mExifParams.debug_params->af_debug_params = *af_exif_debug_params;
            mExifParams.debug_params->af_debug_params_valid = TRUE;
        }
    }
    IF_META_AVAILABLE(cam_asd_exif_debug_t, asd_exif_debug_params,
            CAM_INTF_META_EXIF_DEBUG_ASD, metadata) {
        if (mExifParams.debug_params) {
            mExifParams.debug_params->asd_debug_params = *asd_exif_debug_params;
            mExifParams.debug_params->asd_debug_params_valid = TRUE;
        }
    }
    IF_META_AVAILABLE(cam_stats_buffer_exif_debug_t,stats_exif_debug_params,
            CAM_INTF_META_EXIF_DEBUG_STATS, metadata) {
        if (mExifParams.debug_params) {
            mExifParams.debug_params->stats_debug_params = *stats_exif_debug_params;
            mExifParams.debug_params->stats_debug_params_valid = TRUE;
        }
    }
}

/*===========================================================================
 * FUNCTION   : get3AExifParams
 *
 * DESCRIPTION:
 *
 * PARAMETERS : none
 *
 *
 * RETURN     : mm_jpeg_exif_params_t
 *
 *==========================================================================*/
mm_jpeg_exif_params_t QCamera3HardwareInterface::get3AExifParams()
{
    return mExifParams;
}

/*===========================================================================
 * FUNCTION   : translateCbUrgentMetadataToResultMetadata
 *
 * DESCRIPTION:
 *
 * PARAMETERS :
 *   @metadata : metadata information from callback
 *
 * RETURN     : camera_metadata_t*
 *              metadata in a format specified by fwk
 *==========================================================================*/
camera_metadata_t*
QCamera3HardwareInterface::translateCbUrgentMetadataToResultMetadata
                                (metadata_buffer_t *metadata)
{
    CameraMetadata camMetadata;
    camera_metadata_t *resultMetadata;

    IF_META_AVAILABLE(uint32_t, afState, CAM_INTF_META_AF_STATE, metadata) {
        uint8_t fwk_afState = (uint8_t) *afState;
        camMetadata.update(ANDROID_CONTROL_AF_STATE, &fwk_afState, 1);
        CDBG("%s: urgent Metadata : ANDROID_CONTROL_AF_STATE %u", __func__, *afState);
    }

    IF_META_AVAILABLE(float, focusDistance, CAM_INTF_META_LENS_FOCUS_DISTANCE, metadata) {
        camMetadata.update(ANDROID_LENS_FOCUS_DISTANCE , focusDistance, 1);
    }

    IF_META_AVAILABLE(float, focusRange, CAM_INTF_META_LENS_FOCUS_RANGE, metadata) {
        camMetadata.update(ANDROID_LENS_FOCUS_RANGE , focusRange, 2);
    }

    IF_META_AVAILABLE(uint32_t, whiteBalanceState, CAM_INTF_META_AWB_STATE, metadata) {
        uint8_t fwk_whiteBalanceState = (uint8_t) *whiteBalanceState;
        camMetadata.update(ANDROID_CONTROL_AWB_STATE, &fwk_whiteBalanceState, 1);
        CDBG("%s: urgent Metadata : ANDROID_CONTROL_AWB_STATE %u", __func__, *whiteBalanceState);
    }

    IF_META_AVAILABLE(cam_trigger_t, aecTrigger, CAM_INTF_META_AEC_PRECAPTURE_TRIGGER, metadata) {
        camMetadata.update(ANDROID_CONTROL_AE_PRECAPTURE_TRIGGER,
                &aecTrigger->trigger, 1);
        camMetadata.update(ANDROID_CONTROL_AE_PRECAPTURE_ID,
                &aecTrigger->trigger_id, 1);
        CDBG("%s: urgent Metadata : CAM_INTF_META_AEC_PRECAPTURE_TRIGGER: %d",
                __func__, aecTrigger->trigger);
        CDBG("%s: urgent Metadata : ANDROID_CONTROL_AE_PRECAPTURE_ID: %d", __func__,
                aecTrigger->trigger_id);
    }

    IF_META_AVAILABLE(uint32_t, ae_state, CAM_INTF_META_AEC_STATE, metadata) {
        uint8_t fwk_ae_state = (uint8_t) *ae_state;
        camMetadata.update(ANDROID_CONTROL_AE_STATE, &fwk_ae_state, 1);
        CDBG("%s: urgent Metadata : ANDROID_CONTROL_AE_STATE %u", __func__, *ae_state);
    }

    IF_META_AVAILABLE(uint32_t, focusMode, CAM_INTF_PARM_FOCUS_MODE, metadata) {
        int val = lookupFwkName(FOCUS_MODES_MAP, METADATA_MAP_SIZE(FOCUS_MODES_MAP), *focusMode);
        if (NAME_NOT_FOUND != val) {
            uint8_t fwkAfMode = (uint8_t)val;
            camMetadata.update(ANDROID_CONTROL_AF_MODE, &fwkAfMode, 1);
            CDBG("%s: urgent Metadata : ANDROID_CONTROL_AF_MODE %d", __func__, val);
        } else {
            CDBG_HIGH("%s: urgent Metadata not found : ANDROID_CONTROL_AF_MODE %d", __func__,
                    val);
        }
    }

    IF_META_AVAILABLE(cam_trigger_t, af_trigger, CAM_INTF_META_AF_TRIGGER, metadata) {
        camMetadata.update(ANDROID_CONTROL_AF_TRIGGER,
                &af_trigger->trigger, 1);
        CDBG("%s: urgent Metadata : CAM_INTF_META_AF_TRIGGER = %d",
                __func__, af_trigger->trigger);
        camMetadata.update(ANDROID_CONTROL_AF_TRIGGER_ID, &af_trigger->trigger_id, 1);
        CDBG("%s: urgent Metadata : ANDROID_CONTROL_AF_TRIGGER_ID = %d", __func__,
                af_trigger->trigger_id);
    }

    IF_META_AVAILABLE(int32_t, whiteBalance, CAM_INTF_PARM_WHITE_BALANCE, metadata) {
        int val = lookupFwkName(WHITE_BALANCE_MODES_MAP,
                METADATA_MAP_SIZE(WHITE_BALANCE_MODES_MAP), *whiteBalance);
        if (NAME_NOT_FOUND != val) {
            uint8_t fwkWhiteBalanceMode = (uint8_t)val;
            camMetadata.update(ANDROID_CONTROL_AWB_MODE, &fwkWhiteBalanceMode, 1);
            CDBG("%s: urgent Metadata : ANDROID_CONTROL_AWB_MODE %d", __func__, val);
        } else {
            CDBG_HIGH("%s: urgent Metadata not found : ANDROID_CONTROL_AWB_MODE", __func__);
        }
    }

    uint8_t fwk_aeMode = ANDROID_CONTROL_AE_MODE_OFF;
    uint32_t aeMode = CAM_AE_MODE_MAX;
    int32_t flashMode = CAM_FLASH_MODE_MAX;
    int32_t redeye = -1;
    IF_META_AVAILABLE(uint32_t, pAeMode, CAM_INTF_META_AEC_MODE, metadata) {
        aeMode = *pAeMode;
    }
    IF_META_AVAILABLE(int32_t, pFlashMode, CAM_INTF_PARM_LED_MODE, metadata) {
        flashMode = *pFlashMode;
    }
    IF_META_AVAILABLE(int32_t, pRedeye, CAM_INTF_PARM_REDEYE_REDUCTION, metadata) {
        redeye = *pRedeye;
    }

    if (1 == redeye) {
        fwk_aeMode = ANDROID_CONTROL_AE_MODE_ON_AUTO_FLASH_REDEYE;
        camMetadata.update(ANDROID_CONTROL_AE_MODE, &fwk_aeMode, 1);
    } else if ((CAM_FLASH_MODE_AUTO == flashMode) || (CAM_FLASH_MODE_ON == flashMode)) {
        int val = lookupFwkName(AE_FLASH_MODE_MAP, METADATA_MAP_SIZE(AE_FLASH_MODE_MAP),
                flashMode);
        if (NAME_NOT_FOUND != val) {
            fwk_aeMode = (uint8_t)val;
            camMetadata.update(ANDROID_CONTROL_AE_MODE, &fwk_aeMode, 1);
        } else {
            ALOGE("%s: Unsupported flash mode %d", __func__, flashMode);
        }
    } else if (aeMode == CAM_AE_MODE_ON) {
        fwk_aeMode = ANDROID_CONTROL_AE_MODE_ON;
        camMetadata.update(ANDROID_CONTROL_AE_MODE, &fwk_aeMode, 1);
    } else if (aeMode == CAM_AE_MODE_OFF) {
        fwk_aeMode = ANDROID_CONTROL_AE_MODE_OFF;
        camMetadata.update(ANDROID_CONTROL_AE_MODE, &fwk_aeMode, 1);
    } else {
        ALOGE("%s: Not enough info to deduce ANDROID_CONTROL_AE_MODE redeye:%d, "
              "flashMode:%d, aeMode:%u!!!",
                __func__, redeye, flashMode, aeMode);
    }

    IF_META_AVAILABLE(cam_af_lens_state_t, lensState, CAM_INTF_META_LENS_STATE, metadata) {
        uint8_t fwk_lensState = *lensState;
        camMetadata.update(ANDROID_LENS_STATE , &fwk_lensState, 1);
    }

    resultMetadata = camMetadata.release();
    return resultMetadata;
}

/*===========================================================================
 * FUNCTION   : dumpMetadataToFile
 *
 * DESCRIPTION: Dumps tuning metadata to file system
 *
 * PARAMETERS :
 *   @meta           : tuning metadata
 *   @dumpFrameCount : current dump frame count
 *   @enabled        : Enable mask
 *
 *==========================================================================*/
void QCamera3HardwareInterface::dumpMetadataToFile(tuning_params_t &meta,
                                                   uint32_t &dumpFrameCount,
                                                   bool enabled,
                                                   const char *type,
                                                   uint32_t frameNumber)
{
    uint32_t frm_num = 0;

    //Some sanity checks
    if (meta.tuning_sensor_data_size > TUNING_SENSOR_DATA_MAX) {
        ALOGE("%s : Tuning sensor data size bigger than expected %d: %d",
              __func__,
              meta.tuning_sensor_data_size,
              TUNING_SENSOR_DATA_MAX);
        return;
    }

    if (meta.tuning_vfe_data_size > TUNING_VFE_DATA_MAX) {
        ALOGE("%s : Tuning VFE data size bigger than expected %d: %d",
              __func__,
              meta.tuning_vfe_data_size,
              TUNING_VFE_DATA_MAX);
        return;
    }

    if (meta.tuning_cpp_data_size > TUNING_CPP_DATA_MAX) {
        ALOGE("%s : Tuning CPP data size bigger than expected %d: %d",
              __func__,
              meta.tuning_cpp_data_size,
              TUNING_CPP_DATA_MAX);
        return;
    }

    if (meta.tuning_cac_data_size > TUNING_CAC_DATA_MAX) {
        ALOGE("%s : Tuning CAC data size bigger than expected %d: %d",
              __func__,
              meta.tuning_cac_data_size,
              TUNING_CAC_DATA_MAX);
        return;
    }
    //

    if(enabled){
        char timeBuf[FILENAME_MAX];
        char buf[FILENAME_MAX];
        memset(buf, 0, sizeof(buf));
        memset(timeBuf, 0, sizeof(timeBuf));
        time_t current_time;
        struct tm * timeinfo;
        time (&current_time);
        timeinfo = localtime (&current_time);
        if (timeinfo != NULL) {
            strftime (timeBuf, sizeof(timeBuf),
                    QCAMERA_DUMP_FRM_LOCATION"%Y%m%d%H%M%S", timeinfo);
        }
        String8 filePath(timeBuf);
        snprintf(buf,
                sizeof(buf),
                "%dm_%s_%d.bin",
                dumpFrameCount,
                type,
                frameNumber);
        filePath.append(buf);
        int file_fd = open(filePath.string(), O_RDWR | O_CREAT, 0777);
        if (file_fd >= 0) {
            ssize_t written_len = 0;
            meta.tuning_data_version = TUNING_DATA_VERSION;
            void *data = (void *)((uint8_t *)&meta.tuning_data_version);
            written_len += write(file_fd, data, sizeof(uint32_t));
            data = (void *)((uint8_t *)&meta.tuning_sensor_data_size);
            CDBG("tuning_sensor_data_size %d",(int)(*(int *)data));
            written_len += write(file_fd, data, sizeof(uint32_t));
            data = (void *)((uint8_t *)&meta.tuning_vfe_data_size);
            CDBG("tuning_vfe_data_size %d",(int)(*(int *)data));
            written_len += write(file_fd, data, sizeof(uint32_t));
            data = (void *)((uint8_t *)&meta.tuning_cpp_data_size);
            CDBG("tuning_cpp_data_size %d",(int)(*(int *)data));
            written_len += write(file_fd, data, sizeof(uint32_t));
            data = (void *)((uint8_t *)&meta.tuning_cac_data_size);
            CDBG("tuning_cac_data_size %d",(int)(*(int *)data));
            written_len += write(file_fd, data, sizeof(uint32_t));
            meta.tuning_mod3_data_size = 0;
            data = (void *)((uint8_t *)&meta.tuning_mod3_data_size);
            CDBG("tuning_mod3_data_size %d",(int)(*(int *)data));
            written_len += write(file_fd, data, sizeof(uint32_t));
            size_t total_size = meta.tuning_sensor_data_size;
            data = (void *)((uint8_t *)&meta.data);
            written_len += write(file_fd, data, total_size);
            total_size = meta.tuning_vfe_data_size;
            data = (void *)((uint8_t *)&meta.data[TUNING_VFE_DATA_OFFSET]);
            written_len += write(file_fd, data, total_size);
            total_size = meta.tuning_cpp_data_size;
            data = (void *)((uint8_t *)&meta.data[TUNING_CPP_DATA_OFFSET]);
            written_len += write(file_fd, data, total_size);
            total_size = meta.tuning_cac_data_size;
            data = (void *)((uint8_t *)&meta.data[TUNING_CAC_DATA_OFFSET]);
            written_len += write(file_fd, data, total_size);
            close(file_fd);
        }else {
            ALOGE("%s: fail to open file for metadata dumping", __func__);
        }
    }
}

/*===========================================================================
 * FUNCTION   : cleanAndSortStreamInfo
 *
 * DESCRIPTION: helper method to clean up invalid streams in stream_info,
 *              and sort them such that raw stream is at the end of the list
 *              This is a workaround for camera daemon constraint.
 *
 * PARAMETERS : None
 *
 *==========================================================================*/
void QCamera3HardwareInterface::cleanAndSortStreamInfo()
{
    List<stream_info_t *> newStreamInfo;

    /*clean up invalid streams*/
    for (List<stream_info_t*>::iterator it=mStreamInfo.begin();
            it != mStreamInfo.end();) {
        if(((*it)->status) == INVALID){
            QCamera3Channel *channel = (QCamera3Channel*)(*it)->stream->priv;
            delete channel;
            free(*it);
            it = mStreamInfo.erase(it);
        } else {
            it++;
        }
    }

    // Move preview/video/callback/snapshot streams into newList
    for (List<stream_info_t *>::iterator it = mStreamInfo.begin();
            it != mStreamInfo.end();) {
        if ((*it)->stream->format != HAL_PIXEL_FORMAT_RAW_OPAQUE &&
                (*it)->stream->format != HAL_PIXEL_FORMAT_RAW10 &&
                (*it)->stream->format != HAL_PIXEL_FORMAT_RAW16) {
            newStreamInfo.push_back(*it);
            it = mStreamInfo.erase(it);
        } else
            it++;
    }
    // Move raw streams into newList
    for (List<stream_info_t *>::iterator it = mStreamInfo.begin();
            it != mStreamInfo.end();) {
        newStreamInfo.push_back(*it);
        it = mStreamInfo.erase(it);
    }

    mStreamInfo = newStreamInfo;
}

/*===========================================================================
 * FUNCTION   : extractJpegMetadata
 *
 * DESCRIPTION: helper method to extract Jpeg metadata from capture request.
 *              JPEG metadata is cached in HAL, and return as part of capture
 *              result when metadata is returned from camera daemon.
 *
 * PARAMETERS : @jpegMetadata: jpeg metadata to be extracted
 *              @request:      capture request
 *
 *==========================================================================*/
void QCamera3HardwareInterface::extractJpegMetadata(
        CameraMetadata& jpegMetadata,
        const camera3_capture_request_t *request)
{
    CameraMetadata frame_settings;
    frame_settings = request->settings;

    if (frame_settings.exists(ANDROID_JPEG_GPS_COORDINATES))
        jpegMetadata.update(ANDROID_JPEG_GPS_COORDINATES,
                frame_settings.find(ANDROID_JPEG_GPS_COORDINATES).data.d,
                frame_settings.find(ANDROID_JPEG_GPS_COORDINATES).count);

    if (frame_settings.exists(ANDROID_JPEG_GPS_PROCESSING_METHOD))
        jpegMetadata.update(ANDROID_JPEG_GPS_PROCESSING_METHOD,
                frame_settings.find(ANDROID_JPEG_GPS_PROCESSING_METHOD).data.u8,
                frame_settings.find(ANDROID_JPEG_GPS_PROCESSING_METHOD).count);

    if (frame_settings.exists(ANDROID_JPEG_GPS_TIMESTAMP))
        jpegMetadata.update(ANDROID_JPEG_GPS_TIMESTAMP,
                frame_settings.find(ANDROID_JPEG_GPS_TIMESTAMP).data.i64,
                frame_settings.find(ANDROID_JPEG_GPS_TIMESTAMP).count);

    if (frame_settings.exists(ANDROID_JPEG_ORIENTATION))
        jpegMetadata.update(ANDROID_JPEG_ORIENTATION,
                frame_settings.find(ANDROID_JPEG_ORIENTATION).data.i32,
                frame_settings.find(ANDROID_JPEG_ORIENTATION).count);

    if (frame_settings.exists(ANDROID_JPEG_QUALITY))
        jpegMetadata.update(ANDROID_JPEG_QUALITY,
                frame_settings.find(ANDROID_JPEG_QUALITY).data.u8,
                frame_settings.find(ANDROID_JPEG_QUALITY).count);

    if (frame_settings.exists(ANDROID_JPEG_THUMBNAIL_QUALITY))
        jpegMetadata.update(ANDROID_JPEG_THUMBNAIL_QUALITY,
                frame_settings.find(ANDROID_JPEG_THUMBNAIL_QUALITY).data.u8,
                frame_settings.find(ANDROID_JPEG_THUMBNAIL_QUALITY).count);

    if (frame_settings.exists(ANDROID_JPEG_THUMBNAIL_SIZE)) {
        int32_t thumbnail_size[2];
        thumbnail_size[0] = frame_settings.find(ANDROID_JPEG_THUMBNAIL_SIZE).data.i32[0];
        thumbnail_size[1] = frame_settings.find(ANDROID_JPEG_THUMBNAIL_SIZE).data.i32[1];
        #ifndef USE_L_MR1
        if (frame_settings.exists(ANDROID_JPEG_ORIENTATION)) {
            int32_t orientation =
                    frame_settings.find(ANDROID_JPEG_ORIENTATION).data.i32[0];
            if ((orientation == 90) || (orientation == 270)) {
                //swap thumbnail dimensions for rotations 90 and 270 in jpeg metadata.
                int32_t temp;
                temp = thumbnail_size[0];
                thumbnail_size[0] = thumbnail_size[1];
                thumbnail_size[1] = temp;
            }
        }
        #endif
        jpegMetadata.update(ANDROID_JPEG_THUMBNAIL_SIZE,
                thumbnail_size,
                frame_settings.find(ANDROID_JPEG_THUMBNAIL_SIZE).count);
    }
}

/*===========================================================================
 * FUNCTION   : convertToRegions
 *
 * DESCRIPTION: helper method to convert from cam_rect_t into int32_t array
 *
 * PARAMETERS :
 *   @rect   : cam_rect_t struct to convert
 *   @region : int32_t destination array
 *   @weight : if we are converting from cam_area_t, weight is valid
 *             else weight = -1
 *
 *==========================================================================*/
void QCamera3HardwareInterface::convertToRegions(cam_rect_t rect,
        int32_t *region, int weight)
{
    region[0] = rect.left;
    region[1] = rect.top;
    region[2] = rect.left + rect.width;
    region[3] = rect.top + rect.height;
    if (weight > -1) {
        region[4] = weight;
    }
}

/*===========================================================================
 * FUNCTION   : convertFromRegions
 *
 * DESCRIPTION: helper method to convert from array to cam_rect_t
 *
 * PARAMETERS :
 *   @rect   : cam_rect_t struct to convert
 *   @region : int32_t destination array
 *   @weight : if we are converting from cam_area_t, weight is valid
 *             else weight = -1
 *
 *==========================================================================*/
void QCamera3HardwareInterface::convertFromRegions(cam_area_t &roi,
        const camera_metadata_t *settings, uint32_t tag)
{
    CameraMetadata frame_settings;
    frame_settings = settings;
    int32_t x_min = frame_settings.find(tag).data.i32[0];
    int32_t y_min = frame_settings.find(tag).data.i32[1];
    int32_t x_max = frame_settings.find(tag).data.i32[2];
    int32_t y_max = frame_settings.find(tag).data.i32[3];
    roi.weight = frame_settings.find(tag).data.i32[4];
    roi.rect.left = x_min;
    roi.rect.top = y_min;
    roi.rect.width = x_max - x_min;
    roi.rect.height = y_max - y_min;
}

/*===========================================================================
 * FUNCTION   : resetIfNeededROI
 *
 * DESCRIPTION: helper method to reset the roi if it is greater than scaler
 *              crop region
 *
 * PARAMETERS :
 *   @roi       : cam_area_t struct to resize
 *   @scalerCropRegion : cam_crop_region_t region to compare against
 *
 *
 *==========================================================================*/
bool QCamera3HardwareInterface::resetIfNeededROI(cam_area_t* roi,
                                                 const cam_crop_region_t* scalerCropRegion)
{
    int32_t roi_x_max = roi->rect.width + roi->rect.left;
    int32_t roi_y_max = roi->rect.height + roi->rect.top;
    int32_t crop_x_max = scalerCropRegion->width + scalerCropRegion->left;
    int32_t crop_y_max = scalerCropRegion->height + scalerCropRegion->top;

    /* According to spec weight = 0 is used to indicate roi needs to be disabled
     * without having this check the calculations below to validate if the roi
     * is inside scalar crop region will fail resulting in the roi not being
     * reset causing algorithm to continue to use stale roi window
     */
    if (roi->weight == 0) {
        return true;
    }

    if ((roi_x_max < scalerCropRegion->left) ||
        // right edge of roi window is left of scalar crop's left edge
        (roi_y_max < scalerCropRegion->top)  ||
        // bottom edge of roi window is above scalar crop's top edge
        (roi->rect.left > crop_x_max) ||
        // left edge of roi window is beyond(right) of scalar crop's right edge
        (roi->rect.top > crop_y_max)){
        // top edge of roi windo is above scalar crop's top edge
        return false;
    }
    if (roi->rect.left < scalerCropRegion->left) {
        roi->rect.left = scalerCropRegion->left;
    }
    if (roi->rect.top < scalerCropRegion->top) {
        roi->rect.top = scalerCropRegion->top;
    }
    if (roi_x_max > crop_x_max) {
        roi_x_max = crop_x_max;
    }
    if (roi_y_max > crop_y_max) {
        roi_y_max = crop_y_max;
    }
    roi->rect.width = roi_x_max - roi->rect.left;
    roi->rect.height = roi_y_max - roi->rect.top;
    return true;
}

/*===========================================================================
 * FUNCTION   : convertLandmarks
 *
 * DESCRIPTION: helper method to extract the landmarks from face detection info
 *
 * PARAMETERS :
 *   @face   : cam_rect_t struct to convert
 *   @landmarks : int32_t destination array
 *
 *
 *==========================================================================*/
void QCamera3HardwareInterface::convertLandmarks(cam_face_detection_info_t face, int32_t *landmarks)
{
    landmarks[0] = (int32_t)face.left_eye_center.x;
    landmarks[1] = (int32_t)face.left_eye_center.y;
    landmarks[2] = (int32_t)face.right_eye_center.x;
    landmarks[3] = (int32_t)face.right_eye_center.y;
    landmarks[4] = (int32_t)face.mouth_center.x;
    landmarks[5] = (int32_t)face.mouth_center.y;
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
int QCamera3HardwareInterface::initCapabilities(uint32_t cameraId)
{
    int rc = 0;
    mm_camera_vtbl_t *cameraHandle = NULL;
    QCamera3HeapMemory *capabilityHeap = NULL;

    rc = camera_open((uint8_t)cameraId, &cameraHandle);
    if (rc) {
        ALOGE("%s: camera_open failed. rc = %d", __func__, rc);
        goto open_failed;
    }
    if (!cameraHandle) {
        ALOGE("%s: camera_open failed. cameraHandle = %p", __func__, cameraHandle);
        goto open_failed;
    }

    capabilityHeap = new QCamera3HeapMemory();
    if (capabilityHeap == NULL) {
        ALOGE("%s: creation of capabilityHeap failed", __func__);
        goto heap_creation_failed;
    }
    /* Allocate memory for capability buffer */
    rc = capabilityHeap->allocate(1, sizeof(cam_capability_t), false);
    if(rc != OK) {
        ALOGE("%s: No memory for cappability", __func__);
        goto allocate_failed;
    }

    /* Map memory for capability buffer */
    memset(DATA_PTR(capabilityHeap,0), 0, sizeof(cam_capability_t));
    rc = cameraHandle->ops->map_buf(cameraHandle->camera_handle,
                                CAM_MAPPING_BUF_TYPE_CAPABILITY,
                                capabilityHeap->getFd(0),
                                sizeof(cam_capability_t));
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
    gCamCapability[cameraId] = (cam_capability_t *)malloc(sizeof(cam_capability_t));
    if (!gCamCapability[cameraId]) {
        ALOGE("%s: out of memory", __func__);
        goto query_failed;
    }
    memcpy(gCamCapability[cameraId], DATA_PTR(capabilityHeap,0),
                                        sizeof(cam_capability_t));
    rc = 0;

query_failed:
    cameraHandle->ops->unmap_buf(cameraHandle->camera_handle,
                            CAM_MAPPING_BUF_TYPE_CAPABILITY);
map_failed:
    capabilityHeap->deallocate();
allocate_failed:
    delete capabilityHeap;
heap_creation_failed:
    cameraHandle->ops->close_camera(cameraHandle->camera_handle);
    cameraHandle = NULL;
open_failed:
    return rc;
}

/*===========================================================================
 * FUNCTION   : initParameters
 *
 * DESCRIPTION: initialize camera parameters
 *
 * PARAMETERS :
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCamera3HardwareInterface::initParameters()
{
    int rc = 0;

    //Allocate Set Param Buffer
    mParamHeap = new QCamera3HeapMemory();
    rc = mParamHeap->allocate(1, sizeof(metadata_buffer_t), false);
    if(rc != OK) {
        rc = NO_MEMORY;
        ALOGE("Failed to allocate SETPARM Heap memory");
        delete mParamHeap;
        mParamHeap = NULL;
        return rc;
    }

    //Map memory for parameters buffer
    rc = mCameraHandle->ops->map_buf(mCameraHandle->camera_handle,
            CAM_MAPPING_BUF_TYPE_PARM_BUF,
            mParamHeap->getFd(0),
            sizeof(metadata_buffer_t));
    if(rc < 0) {
        ALOGE("%s:failed to map SETPARM buffer",__func__);
        rc = FAILED_TRANSACTION;
        mParamHeap->deallocate();
        delete mParamHeap;
        mParamHeap = NULL;
        return rc;
    }

    mParameters = (metadata_buffer_t *) DATA_PTR(mParamHeap,0);

    mPrevParameters = (metadata_buffer_t *)malloc(sizeof(metadata_buffer_t));
    return rc;
}

/*===========================================================================
 * FUNCTION   : deinitParameters
 *
 * DESCRIPTION: de-initialize camera parameters
 *
 * PARAMETERS :
 *
 * RETURN     : NONE
 *==========================================================================*/
void QCamera3HardwareInterface::deinitParameters()
{
    mCameraHandle->ops->unmap_buf(mCameraHandle->camera_handle,
            CAM_MAPPING_BUF_TYPE_PARM_BUF);

    mParamHeap->deallocate();
    delete mParamHeap;
    mParamHeap = NULL;

    mParameters = NULL;

    free(mPrevParameters);
    mPrevParameters = NULL;
}

/*===========================================================================
 * FUNCTION   : calcMaxJpegSize
 *
 * DESCRIPTION: Calculates maximum jpeg size supported by the cameraId
 *
 * PARAMETERS :
 *
 * RETURN     : max_jpeg_size
 *==========================================================================*/
size_t QCamera3HardwareInterface::calcMaxJpegSize(uint32_t camera_id)
{
    size_t max_jpeg_size = 0;
    size_t temp_width, temp_height;
    size_t count = MIN(gCamCapability[camera_id]->picture_sizes_tbl_cnt,
            MAX_SIZES_CNT);
    for (size_t i = 0; i < count; i++) {
        temp_width = (size_t)gCamCapability[camera_id]->picture_sizes_tbl[i].width;
        temp_height = (size_t)gCamCapability[camera_id]->picture_sizes_tbl[i].height;
        if (temp_width * temp_height > max_jpeg_size ) {
            max_jpeg_size = temp_width * temp_height;
        }
    }
    max_jpeg_size = max_jpeg_size * 3/2 + sizeof(camera3_jpeg_blob_t);
    return max_jpeg_size;
}

/*===========================================================================
 * FUNCTION   : getMaxRawSize
 *
 * DESCRIPTION: Fetches maximum raw size supported by the cameraId
 *
 * PARAMETERS :
 *
 * RETURN     : Largest supported Raw Dimension
 *==========================================================================*/
cam_dimension_t QCamera3HardwareInterface::getMaxRawSize(uint32_t camera_id)
{
    int max_width = 0;
    cam_dimension_t maxRawSize;

    memset(&maxRawSize, 0, sizeof(cam_dimension_t));
    for (size_t i = 0; i < gCamCapability[camera_id]->supported_raw_dim_cnt; i++) {
        if (max_width < gCamCapability[camera_id]->raw_dim[i].width) {
            max_width = gCamCapability[camera_id]->raw_dim[i].width;
            maxRawSize = gCamCapability[camera_id]->raw_dim[i];
        }
    }
    return maxRawSize;
}


/*===========================================================================
 * FUNCTION   : calcMaxJpegDim
 *
 * DESCRIPTION: Calculates maximum jpeg dimension supported by the cameraId
 *
 * PARAMETERS :
 *
 * RETURN     : max_jpeg_dim
 *==========================================================================*/
cam_dimension_t QCamera3HardwareInterface::calcMaxJpegDim()
{
    cam_dimension_t max_jpeg_dim;
    cam_dimension_t curr_jpeg_dim;
    max_jpeg_dim.width = 0;
    max_jpeg_dim.height = 0;
    curr_jpeg_dim.width = 0;
    curr_jpeg_dim.height = 0;
    for (size_t i = 0; i < gCamCapability[mCameraId]->picture_sizes_tbl_cnt; i++) {
        curr_jpeg_dim.width = gCamCapability[mCameraId]->picture_sizes_tbl[i].width;
        curr_jpeg_dim.height = gCamCapability[mCameraId]->picture_sizes_tbl[i].height;
        if (curr_jpeg_dim.width * curr_jpeg_dim.height >
            max_jpeg_dim.width * max_jpeg_dim.height ) {
            max_jpeg_dim.width = curr_jpeg_dim.width;
            max_jpeg_dim.height = curr_jpeg_dim.height;
        }
    }
    return max_jpeg_dim;
}


/*===========================================================================
 * FUNCTION   : initStaticMetadata
 *
 * DESCRIPTION: initialize the static metadata
 *
 * PARAMETERS :
 *   @cameraId  : camera Id
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              non-zero failure code
 *==========================================================================*/
int QCamera3HardwareInterface::initStaticMetadata(uint32_t cameraId)
{
    int rc = 0;
    CameraMetadata staticInfo;
    size_t count = 0;
    bool limitedDevice = false;
    int64_t m_MinDurationBoundNs = 50000000; // 50 ms, 20 fps
    char prop[PROPERTY_VALUE_MAX];
    /* If sensor is YUV sensor (no raw support) or if per-frame control is not
     * guaranteed or if min fps of max resolution is less than 20 fps, its
     * advertised as limited device*/
    limitedDevice = gCamCapability[cameraId]->no_per_frame_control_support ||
            (CAM_SENSOR_YUV == gCamCapability[cameraId]->sensor_type.sens_type) ||
            (gCamCapability[cameraId]->picture_min_duration[0] > m_MinDurationBoundNs);

    uint8_t supportedHwLvl = limitedDevice ?
            ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL_LIMITED :
            ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL_FULL;

    staticInfo.update(ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL,
            &supportedHwLvl, 1);

    bool facingBack = gCamCapability[cameraId]->position == CAM_POSITION_BACK;
    /*HAL 3 only*/
    staticInfo.update(ANDROID_LENS_INFO_MINIMUM_FOCUS_DISTANCE,
                    &gCamCapability[cameraId]->min_focus_distance, 1);

    staticInfo.update(ANDROID_LENS_INFO_HYPERFOCAL_DISTANCE,
                    &gCamCapability[cameraId]->hyper_focal_distance, 1);

    /*should be using focal lengths but sensor doesn't provide that info now*/
    staticInfo.update(ANDROID_LENS_INFO_AVAILABLE_FOCAL_LENGTHS,
                      &gCamCapability[cameraId]->focal_length,
                      1);

    staticInfo.update(ANDROID_LENS_INFO_AVAILABLE_APERTURES,
                      gCamCapability[cameraId]->apertures,
                      gCamCapability[cameraId]->apertures_count);

    staticInfo.update(ANDROID_LENS_INFO_AVAILABLE_FILTER_DENSITIES,
                gCamCapability[cameraId]->filter_densities,
                gCamCapability[cameraId]->filter_densities_count);


    staticInfo.update(ANDROID_LENS_INFO_AVAILABLE_OPTICAL_STABILIZATION,
                      (uint8_t *)gCamCapability[cameraId]->optical_stab_modes,
                      gCamCapability[cameraId]->optical_stab_modes_count);

    #ifdef USE_L_MR1
    staticInfo.update(ANDROID_LENS_POSITION,
                      gCamCapability[cameraId]->lens_position,
                      sizeof(gCamCapability[cameraId]->lens_position)/ sizeof(float));
    #endif

    int32_t lens_shading_map_size[] = {gCamCapability[cameraId]->lens_shading_map_size.width,
            gCamCapability[cameraId]->lens_shading_map_size.height};
    staticInfo.update(ANDROID_LENS_INFO_SHADING_MAP_SIZE,
                      lens_shading_map_size,
                      sizeof(lens_shading_map_size)/sizeof(int32_t));

    staticInfo.update(ANDROID_SENSOR_INFO_PHYSICAL_SIZE,
            gCamCapability[cameraId]->sensor_physical_size, SENSOR_PHYSICAL_SIZE_CNT);

    staticInfo.update(ANDROID_SENSOR_INFO_EXPOSURE_TIME_RANGE,
            gCamCapability[cameraId]->exposure_time_range, EXPOSURE_TIME_RANGE_CNT);

    staticInfo.update(ANDROID_SENSOR_INFO_MAX_FRAME_DURATION,
            &gCamCapability[cameraId]->max_frame_duration, 1);

    camera_metadata_rational baseGainFactor = {
            gCamCapability[cameraId]->base_gain_factor.numerator,
            gCamCapability[cameraId]->base_gain_factor.denominator};
    staticInfo.update(ANDROID_SENSOR_BASE_GAIN_FACTOR,
                      &baseGainFactor, 1);

    staticInfo.update(ANDROID_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT,
                     (uint8_t *)&gCamCapability[cameraId]->color_arrangement, 1);

    int32_t pixel_array_size[] = {gCamCapability[cameraId]->pixel_array_size.width,
            gCamCapability[cameraId]->pixel_array_size.height};
    staticInfo.update(ANDROID_SENSOR_INFO_PIXEL_ARRAY_SIZE,
                      pixel_array_size, sizeof(pixel_array_size)/sizeof(pixel_array_size[0]));

    int32_t active_array_size[] = {gCamCapability[cameraId]->active_array_size.left,
                                                gCamCapability[cameraId]->active_array_size.top,
                                                gCamCapability[cameraId]->active_array_size.width,
                                                gCamCapability[cameraId]->active_array_size.height};
    staticInfo.update(ANDROID_SENSOR_INFO_ACTIVE_ARRAY_SIZE,
                      active_array_size, sizeof(active_array_size)/sizeof(active_array_size[0]));

    staticInfo.update(ANDROID_SENSOR_INFO_WHITE_LEVEL,
            &gCamCapability[cameraId]->white_level, 1);

    staticInfo.update(ANDROID_SENSOR_BLACK_LEVEL_PATTERN,
            gCamCapability[cameraId]->black_level_pattern, BLACK_LEVEL_PATTERN_CNT);

    staticInfo.update(ANDROID_FLASH_INFO_CHARGE_DURATION,
                      &gCamCapability[cameraId]->flash_charge_duration, 1);

    staticInfo.update(ANDROID_TONEMAP_MAX_CURVE_POINTS,
                      &gCamCapability[cameraId]->max_tone_map_curve_points, 1);

    uint8_t timestampSource = TIME_SOURCE;
    staticInfo.update(ANDROID_SENSOR_INFO_TIMESTAMP_SOURCE,
            &timestampSource, 1);

    staticInfo.update(ANDROID_STATISTICS_INFO_HISTOGRAM_BUCKET_COUNT,
                      &gCamCapability[cameraId]->histogram_size, 1);

    staticInfo.update(ANDROID_STATISTICS_INFO_MAX_HISTOGRAM_COUNT,
            &gCamCapability[cameraId]->max_histogram_count, 1);

    int32_t sharpness_map_size[] = {gCamCapability[cameraId]->sharpness_map_size.width,
            gCamCapability[cameraId]->sharpness_map_size.height};

    staticInfo.update(ANDROID_STATISTICS_INFO_SHARPNESS_MAP_SIZE,
            sharpness_map_size, sizeof(sharpness_map_size)/sizeof(int32_t));

    staticInfo.update(ANDROID_STATISTICS_INFO_MAX_SHARPNESS_MAP_VALUE,
            &gCamCapability[cameraId]->max_sharpness_map_value, 1);

    int32_t scalar_formats[] = {
            ANDROID_SCALER_AVAILABLE_FORMATS_RAW_OPAQUE,
            ANDROID_SCALER_AVAILABLE_FORMATS_RAW16,
            ANDROID_SCALER_AVAILABLE_FORMATS_YCbCr_420_888,
            ANDROID_SCALER_AVAILABLE_FORMATS_BLOB,
            HAL_PIXEL_FORMAT_RAW10,
            HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED};
    size_t scalar_formats_count = sizeof(scalar_formats) / sizeof(int32_t);
    staticInfo.update(ANDROID_SCALER_AVAILABLE_FORMATS,
                      scalar_formats,
                      scalar_formats_count);

    int32_t available_processed_sizes[MAX_SIZES_CNT * 2];
    count = MIN(gCamCapability[cameraId]->picture_sizes_tbl_cnt, MAX_SIZES_CNT);
    makeTable(gCamCapability[cameraId]->picture_sizes_tbl,
            count, MAX_SIZES_CNT, available_processed_sizes);
    staticInfo.update(ANDROID_SCALER_AVAILABLE_PROCESSED_SIZES,
            available_processed_sizes, count * 2);

    int32_t available_raw_sizes[MAX_SIZES_CNT * 2];
    count = MIN(gCamCapability[cameraId]->supported_raw_dim_cnt, MAX_SIZES_CNT);
    makeTable(gCamCapability[cameraId]->raw_dim,
            count, MAX_SIZES_CNT, available_raw_sizes);
    staticInfo.update(ANDROID_SCALER_AVAILABLE_RAW_SIZES,
            available_raw_sizes, count * 2);

    int32_t available_fps_ranges[MAX_SIZES_CNT * 2];
    count = MIN(gCamCapability[cameraId]->fps_ranges_tbl_cnt, MAX_SIZES_CNT);
    makeFPSTable(gCamCapability[cameraId]->fps_ranges_tbl,
            count, MAX_SIZES_CNT, available_fps_ranges);
    staticInfo.update(ANDROID_CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES,
            available_fps_ranges, count * 2);

    camera_metadata_rational exposureCompensationStep = {
            gCamCapability[cameraId]->exp_compensation_step.numerator,
            gCamCapability[cameraId]->exp_compensation_step.denominator};
    staticInfo.update(ANDROID_CONTROL_AE_COMPENSATION_STEP,
                      &exposureCompensationStep, 1);

    uint8_t availableVstabModes[] = {ANDROID_CONTROL_VIDEO_STABILIZATION_MODE_OFF};
    staticInfo.update(ANDROID_CONTROL_AVAILABLE_VIDEO_STABILIZATION_MODES,
                      availableVstabModes, sizeof(availableVstabModes));

    /*HAL 1 and HAL 3 common*/
    uint32_t zoomSteps = gCamCapability[cameraId]->zoom_ratio_tbl_cnt;
    uint32_t maxZoomStep = gCamCapability[cameraId]->zoom_ratio_tbl[zoomSteps - 1];
    uint32_t minZoomStep = 100; //as per HAL1/API1 spec
    float maxZoom = maxZoomStep/minZoomStep;
    staticInfo.update(ANDROID_SCALER_AVAILABLE_MAX_DIGITAL_ZOOM,
            &maxZoom, 1);

    uint8_t croppingType = ANDROID_SCALER_CROPPING_TYPE_FREEFORM;
    staticInfo.update(ANDROID_SCALER_CROPPING_TYPE, &croppingType, 1);

    int32_t max3aRegions[3] = {/*AE*/1,/*AWB*/ 0,/*AF*/ 1};
    if (gCamCapability[cameraId]->supported_focus_modes_cnt == 1)
        max3aRegions[2] = 0; /* AF not supported */
    staticInfo.update(ANDROID_CONTROL_MAX_REGIONS,
            max3aRegions, 3);

    /* 0: OFF, 1: OFF+SIMPLE, 2: OFF+FULL, 3: OFF+SIMPLE+FULL */
    memset(prop, 0, sizeof(prop));
    property_get("persist.camera.facedetect", prop, "1");
    uint8_t supportedFaceDetectMode = (uint8_t)atoi(prop);
    CDBG("%s: Support face detection mode: %d",
            __func__, supportedFaceDetectMode);

    int32_t maxFaces = gCamCapability[cameraId]->max_num_roi;
    Vector<uint8_t> availableFaceDetectModes;
    availableFaceDetectModes.add(ANDROID_STATISTICS_FACE_DETECT_MODE_OFF);
    if (supportedFaceDetectMode == 1) {
        availableFaceDetectModes.add(ANDROID_STATISTICS_FACE_DETECT_MODE_SIMPLE);
    } else if (supportedFaceDetectMode == 2) {
        availableFaceDetectModes.add(ANDROID_STATISTICS_FACE_DETECT_MODE_FULL);
    } else if (supportedFaceDetectMode == 3) {
        availableFaceDetectModes.add(ANDROID_STATISTICS_FACE_DETECT_MODE_SIMPLE);
        availableFaceDetectModes.add(ANDROID_STATISTICS_FACE_DETECT_MODE_FULL);
    } else {
        maxFaces = 0;
    }
    staticInfo.update(ANDROID_STATISTICS_INFO_AVAILABLE_FACE_DETECT_MODES,
            availableFaceDetectModes.array(),
            availableFaceDetectModes.size());
    staticInfo.update(ANDROID_STATISTICS_INFO_MAX_FACE_COUNT,
            (int32_t *)&maxFaces, 1);

    int32_t exposureCompensationRange[] = {gCamCapability[cameraId]->exposure_compensation_min,
                                           gCamCapability[cameraId]->exposure_compensation_max};
    staticInfo.update(ANDROID_CONTROL_AE_COMPENSATION_RANGE,
            exposureCompensationRange,
            sizeof(exposureCompensationRange)/sizeof(int32_t));

    uint8_t lensFacing = (facingBack) ?
            ANDROID_LENS_FACING_BACK : ANDROID_LENS_FACING_FRONT;
    staticInfo.update(ANDROID_LENS_FACING, &lensFacing, 1);

    staticInfo.update(ANDROID_JPEG_AVAILABLE_THUMBNAIL_SIZES,
                      available_thumbnail_sizes,
                      sizeof(available_thumbnail_sizes)/sizeof(int32_t));

    /*all sizes will be clubbed into this tag*/
    count = MIN(gCamCapability[cameraId]->picture_sizes_tbl_cnt, MAX_SIZES_CNT);
    /*android.scaler.availableStreamConfigurations*/
    size_t max_stream_configs_size = count * scalar_formats_count * 4;
    int32_t available_stream_configs[max_stream_configs_size];
    size_t idx = 0;
    for (size_t j = 0; j < scalar_formats_count; j++) {
        switch (scalar_formats[j]) {
        case ANDROID_SCALER_AVAILABLE_FORMATS_RAW16:
        case ANDROID_SCALER_AVAILABLE_FORMATS_RAW_OPAQUE:
        case HAL_PIXEL_FORMAT_RAW10:
            for (size_t i = 0; i < gCamCapability[cameraId]->supported_raw_dim_cnt; i++) {
                available_stream_configs[idx] = scalar_formats[j];
                available_stream_configs[idx+1] =
                    gCamCapability[cameraId]->raw_dim[i].width;
                available_stream_configs[idx+2] =
                    gCamCapability[cameraId]->raw_dim[i].height;
                available_stream_configs[idx+3] =
                    ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT;
                idx+=4;
            }
            break;
        case HAL_PIXEL_FORMAT_BLOB:
            for (size_t i = 0; i < MIN(MAX_SIZES_CNT,
                    gCamCapability[cameraId]->picture_sizes_tbl_cnt); i++) {
                available_stream_configs[idx] = scalar_formats[j];
                available_stream_configs[idx+1] =
                        gCamCapability[cameraId]->picture_sizes_tbl[i].width;
                available_stream_configs[idx+2] =
                        gCamCapability[cameraId]->picture_sizes_tbl[i].height;
                available_stream_configs[idx+3] = ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT;
                idx+=4;
            }
            break;
        default:
            for (size_t i = 0; i < gCamCapability[cameraId]->picture_sizes_tbl_cnt; i++) {
                available_stream_configs[idx] = scalar_formats[j];
                available_stream_configs[idx+1] =
                    gCamCapability[cameraId]->picture_sizes_tbl[i].width;
                available_stream_configs[idx+2] =
                    gCamCapability[cameraId]->picture_sizes_tbl[i].height;
                available_stream_configs[idx+3] =
                    ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT;
                idx+=4;
            }


            break;
        }
    }
    staticInfo.update(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS,
                      available_stream_configs, idx);
    static const uint8_t hotpixelMode = ANDROID_HOT_PIXEL_MODE_FAST;
    staticInfo.update(ANDROID_HOT_PIXEL_MODE, &hotpixelMode, 1);

    static const uint8_t hotPixelMapMode = ANDROID_STATISTICS_HOT_PIXEL_MAP_MODE_OFF;
    staticInfo.update(ANDROID_STATISTICS_HOT_PIXEL_MAP_MODE, &hotPixelMapMode, 1);

    /* android.scaler.availableMinFrameDurations */
    int64_t available_min_durations[max_stream_configs_size];
    idx = 0;
    for (size_t j = 0; j < scalar_formats_count; j++) {
        switch (scalar_formats[j]) {
        case ANDROID_SCALER_AVAILABLE_FORMATS_RAW16:
        case ANDROID_SCALER_AVAILABLE_FORMATS_RAW_OPAQUE:
        case HAL_PIXEL_FORMAT_RAW10:
            for (size_t i = 0; i < gCamCapability[cameraId]->supported_raw_dim_cnt; i++) {
                available_min_durations[idx] = scalar_formats[j];
                available_min_durations[idx+1] =
                    gCamCapability[cameraId]->raw_dim[i].width;
                available_min_durations[idx+2] =
                    gCamCapability[cameraId]->raw_dim[i].height;
                available_min_durations[idx+3] =
                    gCamCapability[cameraId]->raw_min_duration[i];
                idx+=4;
            }
            break;
        default:
            for (size_t i = 0; i < gCamCapability[cameraId]->picture_sizes_tbl_cnt; i++) {
                available_min_durations[idx] = scalar_formats[j];
                available_min_durations[idx+1] =
                    gCamCapability[cameraId]->picture_sizes_tbl[i].width;
                available_min_durations[idx+2] =
                    gCamCapability[cameraId]->picture_sizes_tbl[i].height;
                available_min_durations[idx+3] =
                    gCamCapability[cameraId]->picture_min_duration[i];
                idx+=4;
            }
            break;
        }
    }
    staticInfo.update(ANDROID_SCALER_AVAILABLE_MIN_FRAME_DURATIONS,
                      &available_min_durations[0], idx);

    Vector<int32_t> available_hfr_configs;
    for (size_t i = 0; i < gCamCapability[cameraId]->hfr_tbl_cnt; i++) {
        int32_t fps = 0;
        switch (gCamCapability[cameraId]->hfr_tbl[i].mode) {
        case CAM_HFR_MODE_60FPS:
            fps = 60;
            break;
        case CAM_HFR_MODE_90FPS:
            fps = 90;
            break;
        case CAM_HFR_MODE_120FPS:
            fps = 120;
            break;
        case CAM_HFR_MODE_150FPS:
            fps = 150;
            break;
        case CAM_HFR_MODE_180FPS:
            fps = 180;
            break;
        case CAM_HFR_MODE_210FPS:
            fps = 210;
            break;
        case CAM_HFR_MODE_240FPS:
            fps = 240;
            break;
        case CAM_HFR_MODE_480FPS:
            fps = 480;
            break;
        case CAM_HFR_MODE_OFF:
        case CAM_HFR_MODE_MAX:
        default:
            break;
        }

        if (fps > 0) {
            /* (width, height, fps_min, fps_max) */
            available_hfr_configs.add(gCamCapability[cameraId]->hfr_tbl[i].dim.width);
            available_hfr_configs.add(gCamCapability[cameraId]->hfr_tbl[i].dim.height);
            available_hfr_configs.add(fps);
            available_hfr_configs.add(fps);
       }
    }
    //Advertise HFR capability only if the property is set
    memset(prop, 0, sizeof(prop));
    property_get("persist.camera.hal3hfr.enable", prop, "0");
    uint8_t hfrEnable = (uint8_t)atoi(prop);

    if(hfrEnable && available_hfr_configs.array()) {
        staticInfo.update(ANDROID_CONTROL_AVAILABLE_HIGH_SPEED_VIDEO_CONFIGURATIONS,
                available_hfr_configs.array(), available_hfr_configs.size());
    }

    int32_t max_jpeg_size = (int32_t)calcMaxJpegSize(cameraId);
    staticInfo.update(ANDROID_JPEG_MAX_SIZE,
                      &max_jpeg_size, 1);

    uint8_t avail_effects[CAM_EFFECT_MODE_MAX];
    size_t size = 0;
    count = CAM_EFFECT_MODE_MAX;
    count = MIN(gCamCapability[cameraId]->supported_effects_cnt, count);
    for (size_t i = 0; i < count; i++) {
        int val = lookupFwkName(EFFECT_MODES_MAP, METADATA_MAP_SIZE(EFFECT_MODES_MAP),
                gCamCapability[cameraId]->supported_effects[i]);
        if (NAME_NOT_FOUND != val) {
            avail_effects[size] = (uint8_t)val;
            size++;
        }
    }
    staticInfo.update(ANDROID_CONTROL_AVAILABLE_EFFECTS,
                      avail_effects,
                      size);

    /* '+1' for HighSpeedVideo scenemode. Backend does not have a High speed
        video scene mode. If HFR is supported, add HSV scenemode */
    uint8_t avail_scene_modes[CAM_SCENE_MODE_MAX + 1];
    uint8_t supported_indexes[CAM_SCENE_MODE_MAX + 1];
    size_t supported_scene_modes_cnt = 0;
    count = CAM_SCENE_MODE_MAX;
    count = MIN(gCamCapability[cameraId]->supported_scene_modes_cnt, count);
    for (size_t i = 0; i < count; i++) {
        if (gCamCapability[cameraId]->supported_scene_modes[i] !=
                CAM_SCENE_MODE_OFF) {
            int val = lookupFwkName(SCENE_MODES_MAP,
                    METADATA_MAP_SIZE(SCENE_MODES_MAP),
                    gCamCapability[cameraId]->supported_scene_modes[i]);
            if (NAME_NOT_FOUND != val) {
                avail_scene_modes[supported_scene_modes_cnt] = (uint8_t)val;
                supported_indexes[supported_scene_modes_cnt] = (uint8_t)i;
                supported_scene_modes_cnt++;
            }
        }
    }
    uint8_t scene_mode_overrides[(CAM_SCENE_MODE_MAX + 1) * 3];
    makeOverridesList(gCamCapability[cameraId]->scene_mode_overrides,
                      supported_scene_modes_cnt,
                      CAM_SCENE_MODE_MAX,
                      scene_mode_overrides,
                      supported_indexes,
                      cameraId);

    if (hfrEnable && gCamCapability[cameraId]->hfr_tbl_cnt > 0) {
        avail_scene_modes[supported_scene_modes_cnt] =
                ANDROID_CONTROL_SCENE_MODE_HIGH_SPEED_VIDEO;
        scene_mode_overrides[3 * supported_scene_modes_cnt] =
                ANDROID_CONTROL_AE_MODE_ON;
        scene_mode_overrides[3 * supported_scene_modes_cnt + 1] =
                ANDROID_CONTROL_AWB_MODE_AUTO;
        scene_mode_overrides[3 * supported_scene_modes_cnt + 2] =
                ANDROID_CONTROL_AF_MODE_CONTINUOUS_VIDEO;
        supported_scene_modes_cnt++;
    }

    if (supported_scene_modes_cnt == 0) {
        supported_scene_modes_cnt = 1;
        avail_scene_modes[0] = ANDROID_CONTROL_SCENE_MODE_DISABLED;
    }

    staticInfo.update(ANDROID_CONTROL_AVAILABLE_SCENE_MODES,
                      avail_scene_modes,
                      supported_scene_modes_cnt);
    staticInfo.update(ANDROID_CONTROL_SCENE_MODE_OVERRIDES,
            scene_mode_overrides, supported_scene_modes_cnt * 3);

    uint8_t avail_antibanding_modes[CAM_ANTIBANDING_MODE_MAX];
    size = 0;
    count = CAM_ANTIBANDING_MODE_MAX;
    count = MIN(gCamCapability[cameraId]->supported_antibandings_cnt, count);
    for (size_t i = 0; i < count; i++) {
        int val = lookupFwkName(ANTIBANDING_MODES_MAP, METADATA_MAP_SIZE(ANTIBANDING_MODES_MAP),
                gCamCapability[cameraId]->supported_antibandings[i]);
        if (NAME_NOT_FOUND != val) {
            avail_antibanding_modes[size] = (uint8_t)val;
            size++;
        }

    }
    staticInfo.update(ANDROID_CONTROL_AE_AVAILABLE_ANTIBANDING_MODES,
                      avail_antibanding_modes,
                      size);

    uint8_t avail_abberation_modes[] = {
            ANDROID_COLOR_CORRECTION_ABERRATION_MODE_OFF,
            ANDROID_COLOR_CORRECTION_ABERRATION_MODE_FAST,
            ANDROID_COLOR_CORRECTION_ABERRATION_MODE_HIGH_QUALITY};
    count = CAM_COLOR_CORRECTION_ABERRATION_MAX;
    count = MIN(gCamCapability[cameraId]->aberration_modes_count, count);
    if (0 == count) {
        //  If no aberration correction modes are available for a device, this advertise OFF mode
        size = 1;
    } else {
        // If count is not zero then atleast one among the FAST or HIGH quality is supported
        // So, advertize all 3 modes if atleast any one mode is supported as per the
        // new M requirement
        size = 3;
    }
    staticInfo.update(ANDROID_COLOR_CORRECTION_AVAILABLE_ABERRATION_MODES,
            avail_abberation_modes,
            size);

    uint8_t avail_af_modes[CAM_FOCUS_MODE_MAX];
    size = 0;
    count = CAM_FOCUS_MODE_MAX;
    count = MIN(gCamCapability[cameraId]->supported_focus_modes_cnt, count);
    for (size_t i = 0; i < count; i++) {
        int val = lookupFwkName(FOCUS_MODES_MAP, METADATA_MAP_SIZE(FOCUS_MODES_MAP),
                gCamCapability[cameraId]->supported_focus_modes[i]);
        if (NAME_NOT_FOUND != val) {
            avail_af_modes[size] = (uint8_t)val;
            size++;
        }
    }
    staticInfo.update(ANDROID_CONTROL_AF_AVAILABLE_MODES,
                      avail_af_modes,
                      size);

    uint8_t avail_awb_modes[CAM_WB_MODE_MAX];
    size = 0;
    count = CAM_WB_MODE_MAX;
    count = MIN(gCamCapability[cameraId]->supported_white_balances_cnt, count);
    for (size_t i = 0; i < count; i++) {
        int val = lookupFwkName(WHITE_BALANCE_MODES_MAP,
                METADATA_MAP_SIZE(WHITE_BALANCE_MODES_MAP),
                gCamCapability[cameraId]->supported_white_balances[i]);
        if (NAME_NOT_FOUND != val) {
            avail_awb_modes[size] = (uint8_t)val;
            size++;
        }
    }
    staticInfo.update(ANDROID_CONTROL_AWB_AVAILABLE_MODES,
                      avail_awb_modes,
                      size);

    uint8_t available_flash_levels[CAM_FLASH_FIRING_LEVEL_MAX];
    count = CAM_FLASH_FIRING_LEVEL_MAX;
    count = MIN(gCamCapability[cameraId]->supported_flash_firing_level_cnt,
            count);
    for (size_t i = 0; i < count; i++) {
        available_flash_levels[i] =
                gCamCapability[cameraId]->supported_firing_levels[i];
    }
    staticInfo.update(ANDROID_FLASH_FIRING_POWER,
            available_flash_levels, count);

    uint8_t flashAvailable;
    if (gCamCapability[cameraId]->flash_available)
        flashAvailable = ANDROID_FLASH_INFO_AVAILABLE_TRUE;
    else
        flashAvailable = ANDROID_FLASH_INFO_AVAILABLE_FALSE;
    staticInfo.update(ANDROID_FLASH_INFO_AVAILABLE,
            &flashAvailable, 1);

    Vector<uint8_t> avail_ae_modes;
    count = CAM_AE_MODE_MAX;
    count = MIN(gCamCapability[cameraId]->supported_ae_modes_cnt, count);
    for (size_t i = 0; i < count; i++) {
        avail_ae_modes.add(gCamCapability[cameraId]->supported_ae_modes[i]);
    }
    if (flashAvailable) {
        avail_ae_modes.add(ANDROID_CONTROL_AE_MODE_ON_AUTO_FLASH);
        avail_ae_modes.add(ANDROID_CONTROL_AE_MODE_ON_ALWAYS_FLASH);
        avail_ae_modes.add(ANDROID_CONTROL_AE_MODE_ON_AUTO_FLASH_REDEYE);
    }
    staticInfo.update(ANDROID_CONTROL_AE_AVAILABLE_MODES,
                      avail_ae_modes.array(),
                      avail_ae_modes.size());

    int32_t sensitivity_range[2];
    sensitivity_range[0] = gCamCapability[cameraId]->sensitivity_range.min_sensitivity;
    sensitivity_range[1] = gCamCapability[cameraId]->sensitivity_range.max_sensitivity;
    staticInfo.update(ANDROID_SENSOR_INFO_SENSITIVITY_RANGE,
                      sensitivity_range,
                      sizeof(sensitivity_range) / sizeof(int32_t));

    staticInfo.update(ANDROID_SENSOR_MAX_ANALOG_SENSITIVITY,
                      &gCamCapability[cameraId]->max_analog_sensitivity,
                      1);

    int32_t sensor_orientation = (int32_t)gCamCapability[cameraId]->sensor_mount_angle;
    staticInfo.update(ANDROID_SENSOR_ORIENTATION,
                      &sensor_orientation,
                      1);

    int32_t max_output_streams[] = {
            MAX_STALLING_STREAMS,
            MAX_PROCESSED_STREAMS,
            MAX_RAW_STREAMS};
    staticInfo.update(ANDROID_REQUEST_MAX_NUM_OUTPUT_STREAMS,
            max_output_streams,
            sizeof(max_output_streams)/sizeof(max_output_streams[0]));

    uint8_t avail_leds = 0;
    staticInfo.update(ANDROID_LED_AVAILABLE_LEDS,
                      &avail_leds, 0);

    uint8_t focus_dist_calibrated;
    int val = lookupFwkName(FOCUS_CALIBRATION_MAP, METADATA_MAP_SIZE(FOCUS_CALIBRATION_MAP),
            gCamCapability[cameraId]->focus_dist_calibrated);
    if (NAME_NOT_FOUND != val) {
        focus_dist_calibrated = (uint8_t)val;
        staticInfo.update(ANDROID_LENS_INFO_FOCUS_DISTANCE_CALIBRATION,
                     &focus_dist_calibrated, 1);
    }

    int32_t avail_testpattern_modes[MAX_TEST_PATTERN_CNT];
    size = 0;
    count = MIN(gCamCapability[cameraId]->supported_test_pattern_modes_cnt,
            MAX_TEST_PATTERN_CNT);
    for (size_t i = 0; i < count; i++) {
        int testpatternMode = lookupFwkName(TEST_PATTERN_MAP, METADATA_MAP_SIZE(TEST_PATTERN_MAP),
                gCamCapability[cameraId]->supported_test_pattern_modes[i]);
        if (NAME_NOT_FOUND != testpatternMode) {
            avail_testpattern_modes[size] = testpatternMode;
            size++;
        }
    }
    staticInfo.update(ANDROID_SENSOR_AVAILABLE_TEST_PATTERN_MODES,
                      avail_testpattern_modes,
                      size);

    uint8_t max_pipeline_depth = (uint8_t)(MAX_INFLIGHT_REQUESTS + EMPTY_PIPELINE_DELAY + FRAME_SKIP_DELAY);
    staticInfo.update(ANDROID_REQUEST_PIPELINE_MAX_DEPTH,
                      &max_pipeline_depth,
                      1);

    int32_t partial_result_count = PARTIAL_RESULT_COUNT;
    staticInfo.update(ANDROID_REQUEST_PARTIAL_RESULT_COUNT,
                      &partial_result_count,
                       1);

    Vector<uint8_t> available_capabilities;
    available_capabilities.add(ANDROID_REQUEST_AVAILABLE_CAPABILITIES_BACKWARD_COMPATIBLE);
    available_capabilities.add(ANDROID_REQUEST_AVAILABLE_CAPABILITIES_MANUAL_SENSOR);
    available_capabilities.add(ANDROID_REQUEST_AVAILABLE_CAPABILITIES_MANUAL_POST_PROCESSING);
    available_capabilities.add(ANDROID_REQUEST_AVAILABLE_CAPABILITIES_READ_SENSOR_SETTINGS);
#ifdef USE_L_MR1
    /* Adding this check for advertising burst capabilities only where min fps for max
     * resolution is >= 20 to fix CTS issue */
    if (gCamCapability[cameraId]->picture_min_duration[0] <= m_MinDurationBoundNs)
#endif
    {
        available_capabilities.add(ANDROID_REQUEST_AVAILABLE_CAPABILITIES_BURST_CAPTURE);
    }
    if (CAM_SENSOR_YUV != gCamCapability[cameraId]->sensor_type.sens_type) {
        available_capabilities.add(ANDROID_REQUEST_AVAILABLE_CAPABILITIES_RAW);
    }
    staticInfo.update(ANDROID_REQUEST_AVAILABLE_CAPABILITIES,
            available_capabilities.array(),
            available_capabilities.size());

    int32_t max_input_streams = 0;
    staticInfo.update(ANDROID_REQUEST_MAX_NUM_INPUT_STREAMS,
                      &max_input_streams,
                      1);

    int32_t io_format_map[] = {};
    staticInfo.update(ANDROID_SCALER_AVAILABLE_INPUT_OUTPUT_FORMATS_MAP,
                      io_format_map, 0);

    int32_t max_latency = ANDROID_SYNC_MAX_LATENCY_PER_FRAME_CONTROL;
    staticInfo.update(ANDROID_SYNC_MAX_LATENCY,
                      &max_latency,
                      1);

#ifdef USE_L_MR1
    float optical_axis_angle[2];
    optical_axis_angle[0] = 0; //need to verify
    optical_axis_angle[1] = 0; //need to verify
    staticInfo.update(ANDROID_LENS_OPTICAL_AXIS_ANGLE,
                      optical_axis_angle,
                      2);
#endif

    uint8_t available_hot_pixel_modes[] = {ANDROID_HOT_PIXEL_MODE_FAST,
#ifndef USE_L_MR1
                                           ANDROID_HOT_PIXEL_MODE_HIGH_QUALITY
#endif
                                           };
    staticInfo.update(ANDROID_HOT_PIXEL_AVAILABLE_HOT_PIXEL_MODES,
            available_hot_pixel_modes,
            sizeof(available_hot_pixel_modes)/sizeof(available_hot_pixel_modes[0]));

    uint8_t available_edge_modes[] = {ANDROID_EDGE_MODE_OFF,
                                      ANDROID_EDGE_MODE_FAST,
#ifndef USE_L_MR1
                                      ANDROID_EDGE_MODE_HIGH_QUALITY
#endif
                                      };
    staticInfo.update(ANDROID_EDGE_AVAILABLE_EDGE_MODES,
            available_edge_modes,
            sizeof(available_edge_modes)/sizeof(available_edge_modes[0]));

    uint8_t available_noise_red_modes[] = {ANDROID_NOISE_REDUCTION_MODE_OFF,
                                           ANDROID_NOISE_REDUCTION_MODE_FAST,
#ifndef USE_L_MR1
                                           ANDROID_NOISE_REDUCTION_MODE_HIGH_QUALITY
#endif
                                           };
    staticInfo.update(ANDROID_NOISE_REDUCTION_AVAILABLE_NOISE_REDUCTION_MODES,
            available_noise_red_modes,
            sizeof(available_noise_red_modes)/sizeof(available_noise_red_modes[0]));

    uint8_t available_tonemap_modes[] = {ANDROID_TONEMAP_MODE_CONTRAST_CURVE,
                                         ANDROID_TONEMAP_MODE_FAST,
#ifndef USE_L_MR1
                                         ANDROID_TONEMAP_MODE_HIGH_QUALITY
#endif
                                         };
    staticInfo.update(ANDROID_TONEMAP_AVAILABLE_TONE_MAP_MODES,
            available_tonemap_modes,
            sizeof(available_tonemap_modes)/sizeof(available_tonemap_modes[0]));

    uint8_t available_hot_pixel_map_modes[] = {ANDROID_STATISTICS_HOT_PIXEL_MAP_MODE_OFF};
    staticInfo.update(ANDROID_STATISTICS_INFO_AVAILABLE_HOT_PIXEL_MAP_MODES,
            available_hot_pixel_map_modes,
            sizeof(available_hot_pixel_map_modes)/sizeof(available_hot_pixel_map_modes[0]));

    val = lookupFwkName(REFERENCE_ILLUMINANT_MAP, METADATA_MAP_SIZE(REFERENCE_ILLUMINANT_MAP),
            gCamCapability[cameraId]->reference_illuminant1);
    if (NAME_NOT_FOUND != val) {
        uint8_t fwkReferenceIlluminant = (uint8_t)val;
        staticInfo.update(ANDROID_SENSOR_REFERENCE_ILLUMINANT1, &fwkReferenceIlluminant, 1);
    }

    val = lookupFwkName(REFERENCE_ILLUMINANT_MAP, METADATA_MAP_SIZE(REFERENCE_ILLUMINANT_MAP),
            gCamCapability[cameraId]->reference_illuminant2);
    if (NAME_NOT_FOUND != val) {
        uint8_t fwkReferenceIlluminant = (uint8_t)val;
        staticInfo.update(ANDROID_SENSOR_REFERENCE_ILLUMINANT2, &fwkReferenceIlluminant, 1);
    }

    staticInfo.update(ANDROID_SENSOR_FORWARD_MATRIX1, (camera_metadata_rational_t *)
            (void *)gCamCapability[cameraId]->forward_matrix1,
            FORWARD_MATRIX_COLS * FORWARD_MATRIX_ROWS);

    staticInfo.update(ANDROID_SENSOR_FORWARD_MATRIX2, (camera_metadata_rational_t *)
            (void *)gCamCapability[cameraId]->forward_matrix2,
            FORWARD_MATRIX_COLS * FORWARD_MATRIX_ROWS);

    staticInfo.update(ANDROID_SENSOR_COLOR_TRANSFORM1, (camera_metadata_rational_t *)
            (void *)gCamCapability[cameraId]->color_transform1,
            COLOR_TRANSFORM_COLS * COLOR_TRANSFORM_ROWS);

    staticInfo.update(ANDROID_SENSOR_COLOR_TRANSFORM2, (camera_metadata_rational_t *)
            (void *)gCamCapability[cameraId]->color_transform2,
            COLOR_TRANSFORM_COLS * COLOR_TRANSFORM_ROWS);

    staticInfo.update(ANDROID_SENSOR_CALIBRATION_TRANSFORM1, (camera_metadata_rational_t *)
            (void *)gCamCapability[cameraId]->calibration_transform1,
            CAL_TRANSFORM_COLS * CAL_TRANSFORM_ROWS);

    staticInfo.update(ANDROID_SENSOR_CALIBRATION_TRANSFORM2, (camera_metadata_rational_t *)
            (void *)gCamCapability[cameraId]->calibration_transform2,
            CAL_TRANSFORM_COLS * CAL_TRANSFORM_ROWS);

    int32_t request_keys_basic[] = {ANDROID_COLOR_CORRECTION_MODE,
       ANDROID_COLOR_CORRECTION_TRANSFORM, ANDROID_COLOR_CORRECTION_GAINS,
       ANDROID_COLOR_CORRECTION_ABERRATION_MODE,
       ANDROID_CONTROL_AE_ANTIBANDING_MODE, ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION,
       ANDROID_CONTROL_AE_LOCK, ANDROID_CONTROL_AE_MODE,
       ANDROID_CONTROL_AE_REGIONS, ANDROID_CONTROL_AE_TARGET_FPS_RANGE,
       ANDROID_CONTROL_AE_PRECAPTURE_TRIGGER, ANDROID_CONTROL_AF_MODE,
       ANDROID_CONTROL_AF_TRIGGER, ANDROID_CONTROL_AWB_LOCK,
       ANDROID_CONTROL_AWB_MODE, ANDROID_CONTROL_CAPTURE_INTENT,
       ANDROID_CONTROL_EFFECT_MODE, ANDROID_CONTROL_MODE,
       ANDROID_CONTROL_SCENE_MODE, ANDROID_CONTROL_VIDEO_STABILIZATION_MODE,
       ANDROID_DEMOSAIC_MODE, ANDROID_EDGE_MODE, ANDROID_EDGE_STRENGTH,
       ANDROID_FLASH_FIRING_POWER, ANDROID_FLASH_FIRING_TIME, ANDROID_FLASH_MODE,
       ANDROID_JPEG_GPS_COORDINATES,
       ANDROID_JPEG_GPS_PROCESSING_METHOD, ANDROID_JPEG_GPS_TIMESTAMP,
       ANDROID_JPEG_ORIENTATION, ANDROID_JPEG_QUALITY, ANDROID_JPEG_THUMBNAIL_QUALITY,
       ANDROID_JPEG_THUMBNAIL_SIZE, ANDROID_LENS_APERTURE, ANDROID_LENS_FILTER_DENSITY,
       ANDROID_LENS_FOCAL_LENGTH, ANDROID_LENS_FOCUS_DISTANCE,
       ANDROID_LENS_OPTICAL_STABILIZATION_MODE, ANDROID_NOISE_REDUCTION_MODE,
       ANDROID_NOISE_REDUCTION_STRENGTH, ANDROID_REQUEST_ID, ANDROID_REQUEST_TYPE,
       ANDROID_SCALER_CROP_REGION, ANDROID_SENSOR_EXPOSURE_TIME,
       ANDROID_SENSOR_FRAME_DURATION, ANDROID_HOT_PIXEL_MODE,
       ANDROID_STATISTICS_HOT_PIXEL_MAP_MODE,
       ANDROID_SENSOR_SENSITIVITY, ANDROID_SHADING_MODE,
       ANDROID_SHADING_STRENGTH, ANDROID_STATISTICS_FACE_DETECT_MODE,
       ANDROID_STATISTICS_HISTOGRAM_MODE, ANDROID_STATISTICS_SHARPNESS_MAP_MODE,
       ANDROID_STATISTICS_LENS_SHADING_MAP_MODE, ANDROID_TONEMAP_CURVE_BLUE,
       ANDROID_TONEMAP_CURVE_GREEN, ANDROID_TONEMAP_CURVE_RED, ANDROID_TONEMAP_MODE,
       ANDROID_BLACK_LEVEL_LOCK };

    size_t request_keys_cnt =
            sizeof(request_keys_basic)/sizeof(request_keys_basic[0]);
    Vector<int32_t> available_request_keys;
    available_request_keys.appendArray(request_keys_basic, request_keys_cnt);
    if (gCamCapability[cameraId]->supported_focus_modes_cnt > 1) {
        available_request_keys.add(ANDROID_CONTROL_AF_REGIONS);
    }
    staticInfo.update(ANDROID_REQUEST_AVAILABLE_REQUEST_KEYS,
            available_request_keys.array(), available_request_keys.size());

    int32_t result_keys_basic[] = {ANDROID_COLOR_CORRECTION_TRANSFORM,
       ANDROID_COLOR_CORRECTION_GAINS, ANDROID_CONTROL_AE_MODE, ANDROID_CONTROL_AE_REGIONS,
       ANDROID_CONTROL_AE_STATE, ANDROID_CONTROL_AF_MODE,
       ANDROID_CONTROL_AF_STATE, ANDROID_CONTROL_AWB_MODE,
       ANDROID_CONTROL_AWB_STATE, ANDROID_CONTROL_MODE, ANDROID_EDGE_MODE,
       ANDROID_FLASH_FIRING_POWER, ANDROID_FLASH_FIRING_TIME, ANDROID_FLASH_MODE,
       ANDROID_FLASH_STATE, ANDROID_JPEG_GPS_COORDINATES, ANDROID_JPEG_GPS_PROCESSING_METHOD,
       ANDROID_JPEG_GPS_TIMESTAMP, ANDROID_JPEG_ORIENTATION, ANDROID_JPEG_QUALITY,
       ANDROID_JPEG_THUMBNAIL_QUALITY, ANDROID_JPEG_THUMBNAIL_SIZE, ANDROID_LENS_APERTURE,
       ANDROID_LENS_FILTER_DENSITY, ANDROID_LENS_FOCAL_LENGTH, ANDROID_LENS_FOCUS_DISTANCE,
       ANDROID_LENS_FOCUS_RANGE, ANDROID_LENS_STATE, ANDROID_LENS_OPTICAL_STABILIZATION_MODE,
       ANDROID_NOISE_REDUCTION_MODE, ANDROID_REQUEST_ID,
       ANDROID_SCALER_CROP_REGION, ANDROID_SHADING_MODE, ANDROID_SENSOR_EXPOSURE_TIME,
       ANDROID_SENSOR_FRAME_DURATION, ANDROID_SENSOR_SENSITIVITY,
       ANDROID_SENSOR_TIMESTAMP, ANDROID_SENSOR_NEUTRAL_COLOR_POINT,
       ANDROID_SENSOR_PROFILE_TONE_CURVE, ANDROID_BLACK_LEVEL_LOCK, ANDROID_TONEMAP_CURVE_BLUE,
       ANDROID_TONEMAP_CURVE_GREEN, ANDROID_TONEMAP_CURVE_RED, ANDROID_TONEMAP_MODE,
       ANDROID_STATISTICS_FACE_DETECT_MODE, ANDROID_STATISTICS_HISTOGRAM_MODE,
       ANDROID_STATISTICS_SHARPNESS_MAP, ANDROID_STATISTICS_SHARPNESS_MAP_MODE,
       ANDROID_STATISTICS_PREDICTED_COLOR_GAINS, ANDROID_STATISTICS_PREDICTED_COLOR_TRANSFORM,
       ANDROID_STATISTICS_SCENE_FLICKER, ANDROID_STATISTICS_FACE_RECTANGLES,
       ANDROID_STATISTICS_FACE_SCORES};
    size_t result_keys_cnt =
            sizeof(result_keys_basic)/sizeof(result_keys_basic[0]);

    Vector<int32_t> available_result_keys;
    available_result_keys.appendArray(result_keys_basic, result_keys_cnt);
    if (gCamCapability[cameraId]->supported_focus_modes_cnt > 1) {
        available_result_keys.add(ANDROID_CONTROL_AF_REGIONS);
    }
    if (CAM_SENSOR_RAW == gCamCapability[cameraId]->sensor_type.sens_type) {
        available_result_keys.add(ANDROID_SENSOR_NOISE_PROFILE);
        available_result_keys.add(ANDROID_SENSOR_GREEN_SPLIT);
    }
    if (supportedFaceDetectMode == 1) {
        available_result_keys.add(ANDROID_STATISTICS_FACE_RECTANGLES);
        available_result_keys.add(ANDROID_STATISTICS_FACE_SCORES);
    } else if ((supportedFaceDetectMode == 2) ||
             (supportedFaceDetectMode == 3)) {
        available_result_keys.add(ANDROID_STATISTICS_FACE_IDS);
        available_result_keys.add(ANDROID_STATISTICS_FACE_LANDMARKS);
    }
    staticInfo.update(ANDROID_REQUEST_AVAILABLE_RESULT_KEYS,
            available_result_keys.array(), available_result_keys.size());

    int32_t available_characteristics_keys[] = {ANDROID_CONTROL_AE_AVAILABLE_ANTIBANDING_MODES,
       ANDROID_CONTROL_AE_AVAILABLE_MODES, ANDROID_CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES,
       ANDROID_CONTROL_AE_COMPENSATION_RANGE, ANDROID_CONTROL_AE_COMPENSATION_STEP,
       ANDROID_CONTROL_AF_AVAILABLE_MODES, ANDROID_CONTROL_AVAILABLE_EFFECTS,
       ANDROID_COLOR_CORRECTION_AVAILABLE_ABERRATION_MODES,
       ANDROID_SCALER_CROPPING_TYPE,
       ANDROID_SYNC_MAX_LATENCY,
       ANDROID_SENSOR_INFO_TIMESTAMP_SOURCE,
       ANDROID_CONTROL_AVAILABLE_SCENE_MODES,
       ANDROID_CONTROL_AVAILABLE_VIDEO_STABILIZATION_MODES,
       ANDROID_CONTROL_AWB_AVAILABLE_MODES, ANDROID_CONTROL_MAX_REGIONS,
       ANDROID_CONTROL_SCENE_MODE_OVERRIDES,ANDROID_FLASH_INFO_AVAILABLE,
       ANDROID_FLASH_INFO_CHARGE_DURATION, ANDROID_JPEG_AVAILABLE_THUMBNAIL_SIZES,
       ANDROID_JPEG_MAX_SIZE, ANDROID_LENS_INFO_AVAILABLE_APERTURES,
       ANDROID_LENS_INFO_AVAILABLE_FILTER_DENSITIES,
       ANDROID_LENS_INFO_AVAILABLE_FOCAL_LENGTHS,
       ANDROID_LENS_INFO_AVAILABLE_OPTICAL_STABILIZATION,
       ANDROID_LENS_INFO_HYPERFOCAL_DISTANCE, ANDROID_LENS_INFO_MINIMUM_FOCUS_DISTANCE,
       ANDROID_LENS_INFO_SHADING_MAP_SIZE, ANDROID_LENS_INFO_FOCUS_DISTANCE_CALIBRATION,
       ANDROID_LENS_FACING,
#ifdef USE_L_MR1
       ANDROID_LENS_OPTICAL_AXIS_ANGLE,ANDROID_LENS_POSITION,
#endif
       ANDROID_REQUEST_MAX_NUM_OUTPUT_STREAMS, ANDROID_REQUEST_MAX_NUM_INPUT_STREAMS,
       ANDROID_REQUEST_PIPELINE_MAX_DEPTH, ANDROID_REQUEST_AVAILABLE_CAPABILITIES,
       ANDROID_REQUEST_AVAILABLE_REQUEST_KEYS, ANDROID_REQUEST_AVAILABLE_RESULT_KEYS,
       ANDROID_REQUEST_AVAILABLE_CHARACTERISTICS_KEYS, ANDROID_REQUEST_PARTIAL_RESULT_COUNT,
       ANDROID_SCALER_AVAILABLE_MAX_DIGITAL_ZOOM,
       ANDROID_SCALER_AVAILABLE_INPUT_OUTPUT_FORMATS_MAP,
       ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS,
       /*ANDROID_SCALER_AVAILABLE_STALL_DURATIONS,*/
       ANDROID_SCALER_AVAILABLE_MIN_FRAME_DURATIONS, ANDROID_SENSOR_FORWARD_MATRIX1,
       ANDROID_SENSOR_REFERENCE_ILLUMINANT1, ANDROID_SENSOR_REFERENCE_ILLUMINANT2,
       ANDROID_SENSOR_FORWARD_MATRIX2, ANDROID_SENSOR_COLOR_TRANSFORM1,
       ANDROID_SENSOR_COLOR_TRANSFORM2, ANDROID_SENSOR_CALIBRATION_TRANSFORM1,
       ANDROID_SENSOR_CALIBRATION_TRANSFORM2, ANDROID_SENSOR_INFO_ACTIVE_ARRAY_SIZE,
       ANDROID_SENSOR_INFO_SENSITIVITY_RANGE, ANDROID_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT,
       ANDROID_SENSOR_INFO_EXPOSURE_TIME_RANGE, ANDROID_SENSOR_INFO_MAX_FRAME_DURATION,
       ANDROID_SENSOR_INFO_PHYSICAL_SIZE, ANDROID_SENSOR_INFO_PIXEL_ARRAY_SIZE,
       ANDROID_SENSOR_INFO_WHITE_LEVEL, ANDROID_SENSOR_BASE_GAIN_FACTOR,
       ANDROID_SENSOR_BLACK_LEVEL_PATTERN, ANDROID_SENSOR_MAX_ANALOG_SENSITIVITY,
       ANDROID_SENSOR_ORIENTATION, ANDROID_SENSOR_AVAILABLE_TEST_PATTERN_MODES,
       ANDROID_STATISTICS_INFO_AVAILABLE_FACE_DETECT_MODES,
       ANDROID_STATISTICS_INFO_HISTOGRAM_BUCKET_COUNT,
       ANDROID_STATISTICS_INFO_MAX_FACE_COUNT, ANDROID_STATISTICS_INFO_MAX_HISTOGRAM_COUNT,
       ANDROID_STATISTICS_INFO_MAX_SHARPNESS_MAP_VALUE,
       ANDROID_STATISTICS_INFO_SHARPNESS_MAP_SIZE, ANDROID_HOT_PIXEL_AVAILABLE_HOT_PIXEL_MODES,
       ANDROID_EDGE_AVAILABLE_EDGE_MODES,
       ANDROID_NOISE_REDUCTION_AVAILABLE_NOISE_REDUCTION_MODES,
       ANDROID_TONEMAP_AVAILABLE_TONE_MAP_MODES,
       ANDROID_STATISTICS_INFO_AVAILABLE_HOT_PIXEL_MAP_MODES,
       ANDROID_TONEMAP_MAX_CURVE_POINTS, ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL };
    staticInfo.update(ANDROID_REQUEST_AVAILABLE_CHARACTERISTICS_KEYS,
                      available_characteristics_keys,
                      sizeof(available_characteristics_keys)/sizeof(int32_t));

    /*available stall durations depend on the hw + sw and will be different for different devices */
    /*have to add for raw after implementation*/
    int32_t stall_formats[] = {HAL_PIXEL_FORMAT_BLOB, ANDROID_SCALER_AVAILABLE_FORMATS_RAW16};
    size_t stall_formats_count = sizeof(stall_formats)/sizeof(int32_t);

    count = MIN(gCamCapability[cameraId]->picture_sizes_tbl_cnt, MAX_SIZES_CNT);
    size_t raw_count = MIN(gCamCapability[cameraId]->supported_raw_dim_cnt,
            MAX_SIZES_CNT);
    size_t available_stall_size = count * 4;
    int64_t available_stall_durations[available_stall_size];
    idx = 0;
    for (uint32_t j = 0; j < stall_formats_count; j++) {
       if (stall_formats[j] == HAL_PIXEL_FORMAT_BLOB) {
          for (uint32_t i = 0; i < count; i++) {
             available_stall_durations[idx]   = stall_formats[j];
             available_stall_durations[idx+1] = gCamCapability[cameraId]->picture_sizes_tbl[i].width;
             available_stall_durations[idx+2] = gCamCapability[cameraId]->picture_sizes_tbl[i].height;
             available_stall_durations[idx+3] = gCamCapability[cameraId]->jpeg_stall_durations[i];
             idx+=4;
          }
       } else {
          for (uint32_t i = 0; i < raw_count; i++) {
             available_stall_durations[idx]   = stall_formats[j];
             available_stall_durations[idx+1] = gCamCapability[cameraId]->raw_dim[i].width;
             available_stall_durations[idx+2] = gCamCapability[cameraId]->raw_dim[i].height;
             available_stall_durations[idx+3] = gCamCapability[cameraId]->raw16_stall_durations[i];
             idx+=4;
          }
       }
    }
    staticInfo.update(ANDROID_SCALER_AVAILABLE_STALL_DURATIONS,
                      available_stall_durations,
                      idx);
    //QCAMERA3_OPAQUE_RAW
    uint8_t raw_format = QCAMERA3_OPAQUE_RAW_FORMAT_LEGACY;
    cam_format_t fmt = CAM_FORMAT_BAYER_QCOM_RAW_10BPP_GBRG;
    switch (gCamCapability[cameraId]->opaque_raw_fmt) {
    case LEGACY_RAW:
        if (gCamCapability[cameraId]->white_level == MAX_VALUE_8BIT)
            fmt = CAM_FORMAT_BAYER_QCOM_RAW_8BPP_GBRG;
        else if (gCamCapability[cameraId]->white_level == MAX_VALUE_10BIT)
            fmt = CAM_FORMAT_BAYER_QCOM_RAW_10BPP_GBRG;
        else if (gCamCapability[cameraId]->white_level == MAX_VALUE_12BIT)
            fmt = CAM_FORMAT_BAYER_QCOM_RAW_12BPP_GBRG;
        raw_format = QCAMERA3_OPAQUE_RAW_FORMAT_LEGACY;
        break;
    case MIPI_RAW:
        if (gCamCapability[cameraId]->white_level == MAX_VALUE_8BIT)
            fmt = CAM_FORMAT_BAYER_MIPI_RAW_8BPP_GBRG;
        else if (gCamCapability[cameraId]->white_level == MAX_VALUE_10BIT)
            fmt = CAM_FORMAT_BAYER_MIPI_RAW_10BPP_GBRG;
        else if (gCamCapability[cameraId]->white_level == MAX_VALUE_12BIT)
            fmt = CAM_FORMAT_BAYER_MIPI_RAW_12BPP_GBRG;
        raw_format = QCAMERA3_OPAQUE_RAW_FORMAT_MIPI;
        break;
    default:
        ALOGE("%s: unknown opaque_raw_format %d", __func__,
                gCamCapability[cameraId]->opaque_raw_fmt);
        break;
    }
    staticInfo.update(QCAMERA3_OPAQUE_RAW_FORMAT, &raw_format, 1);

    int32_t strides[3*raw_count];
    for (size_t i = 0; i < raw_count; i++) {
        cam_stream_buf_plane_info_t buf_planes;
        strides[i*3] = gCamCapability[cameraId]->raw_dim[i].width;
        strides[i*3+1] = gCamCapability[cameraId]->raw_dim[i].height;
        mm_stream_calc_offset_raw(fmt, &gCamCapability[cameraId]->raw_dim[i],
            &gCamCapability[cameraId]->padding_info, &buf_planes);
        strides[i*3+2] = buf_planes.plane_info.mp[0].stride;
    }
    staticInfo.update(QCAMERA3_OPAQUE_RAW_STRIDES, strides,
            3*raw_count);

    gStaticMetadata[cameraId] = staticInfo.release();
    return rc;
}

/*===========================================================================
 * FUNCTION   : makeTable
 *
 * DESCRIPTION: make a table of sizes
 *
 * PARAMETERS :
 *
 *
 *==========================================================================*/
void QCamera3HardwareInterface::makeTable(cam_dimension_t* dimTable, size_t size,
        size_t max_size, int32_t *sizeTable)
{
    size_t j = 0;
    if (size > max_size) {
       size = max_size;
    }
    for (size_t i = 0; i < size; i++) {
        sizeTable[j] = dimTable[i].width;
        sizeTable[j+1] = dimTable[i].height;
        j+=2;
    }
}

/*===========================================================================
 * FUNCTION   : makeFPSTable
 *
 * DESCRIPTION: make a table of fps ranges
 *
 * PARAMETERS :
 *
 *==========================================================================*/
void QCamera3HardwareInterface::makeFPSTable(cam_fps_range_t* fpsTable, size_t size,
        size_t max_size, int32_t *fpsRangesTable)
{
    size_t j = 0;
    if (size > max_size) {
       size = max_size;
    }
    for (size_t i = 0; i < size; i++) {
        fpsRangesTable[j] = (int32_t)fpsTable[i].min_fps;
        fpsRangesTable[j+1] = (int32_t)fpsTable[i].max_fps;
        j+=2;
    }
}

/*===========================================================================
 * FUNCTION   : makeOverridesList
 *
 * DESCRIPTION: make a list of scene mode overrides
 *
 * PARAMETERS :
 *
 *
 *==========================================================================*/
void QCamera3HardwareInterface::makeOverridesList(
        cam_scene_mode_overrides_t* overridesTable, size_t size, size_t max_size,
        uint8_t *overridesList, uint8_t *supported_indexes, uint32_t camera_id)
{
    /*daemon will give a list of overrides for all scene modes.
      However we should send the fwk only the overrides for the scene modes
      supported by the framework*/
    size_t j = 0;
    if (size > max_size) {
       size = max_size;
    }
    size_t focus_count = CAM_FOCUS_MODE_MAX;
    focus_count = MIN(gCamCapability[camera_id]->supported_focus_modes_cnt,
            focus_count);
    for (size_t i = 0; i < size; i++) {
        bool supt = false;
        size_t index = supported_indexes[i];
        overridesList[j] = gCamCapability[camera_id]->flash_available ?
                ANDROID_CONTROL_AE_MODE_ON_AUTO_FLASH : ANDROID_CONTROL_AE_MODE_ON;
        int val = lookupFwkName(WHITE_BALANCE_MODES_MAP,
                METADATA_MAP_SIZE(WHITE_BALANCE_MODES_MAP),
                overridesTable[index].awb_mode);
        if (NAME_NOT_FOUND != val) {
            overridesList[j+1] = (uint8_t)val;
        }
        uint8_t focus_override = overridesTable[index].af_mode;
        for (size_t k = 0; k < focus_count; k++) {
           if (gCamCapability[camera_id]->supported_focus_modes[k] == focus_override) {
              supt = true;
              break;
           }
        }
        if (supt) {
            val = lookupFwkName(FOCUS_MODES_MAP, METADATA_MAP_SIZE(FOCUS_MODES_MAP),
                    focus_override);
            if (NAME_NOT_FOUND != val) {
                overridesList[j+2] = (uint8_t)val;
            }
        } else {
           overridesList[j+2] = ANDROID_CONTROL_AF_MODE_OFF;
        }
        j+=3;
    }
}

/*===========================================================================
 * FUNCTION   : filterJpegSizes
 *
 * DESCRIPTION: Returns the supported jpeg sizes based on the max dimension that
 *              could be downscaled to
 *
 * PARAMETERS :
 *
 * RETURN     : length of jpegSizes array
 *==========================================================================*/

size_t QCamera3HardwareInterface::filterJpegSizes(int32_t *jpegSizes, int32_t *processedSizes,
        size_t processedSizesCnt, size_t maxCount, cam_rect_t active_array_size,
        uint8_t downscale_factor)
{
    if (0 == downscale_factor) {
        downscale_factor = 1;
    }

    int32_t min_width = active_array_size.width / downscale_factor;
    int32_t min_height = active_array_size.height / downscale_factor;
    size_t jpegSizesCnt = 0;
    if (processedSizesCnt > maxCount) {
        processedSizesCnt = maxCount;
    }
    for (size_t i = 0; i < processedSizesCnt; i+=2) {
        if (processedSizes[i] >= min_width && processedSizes[i+1] >= min_height) {
            jpegSizes[jpegSizesCnt] = processedSizes[i];
            jpegSizes[jpegSizesCnt+1] = processedSizes[i+1];
            jpegSizesCnt += 2;
        }
    }
    return jpegSizesCnt;
}

/*===========================================================================
 * FUNCTION   : getPreviewHalPixelFormat
 *
 * DESCRIPTION: convert the format to type recognized by framework
 *
 * PARAMETERS : format : the format from backend
 *
 ** RETURN    : format recognized by framework
 *
 *==========================================================================*/
int32_t QCamera3HardwareInterface::getScalarFormat(int32_t format)
{
    int32_t halPixelFormat;

    switch (format) {
    case CAM_FORMAT_YUV_420_NV12:
        halPixelFormat = HAL_PIXEL_FORMAT_YCbCr_420_SP;
        break;
    case CAM_FORMAT_YUV_420_NV21:
        halPixelFormat = HAL_PIXEL_FORMAT_YCrCb_420_SP;
        break;
    case CAM_FORMAT_YUV_420_NV21_ADRENO:
        halPixelFormat = HAL_PIXEL_FORMAT_YCrCb_420_SP_ADRENO;
        break;
    case CAM_FORMAT_YUV_420_YV12:
        halPixelFormat = HAL_PIXEL_FORMAT_YV12;
        break;
    case CAM_FORMAT_YUV_422_NV16:
    case CAM_FORMAT_YUV_422_NV61:
    default:
        halPixelFormat = HAL_PIXEL_FORMAT_YCrCb_420_SP;
        break;
    }
    return halPixelFormat;
}

/*===========================================================================
 * FUNCTION   : computeNoiseModelEntryS
 *
 * DESCRIPTION: function to map a given sensitivity to the S noise
 *              model parameters in the DNG noise model.
 *
 * PARAMETERS : sens : the sensor sensitivity
 *
 ** RETURN    : S (sensor amplification) noise
 *
 *==========================================================================*/
double QCamera3HardwareInterface::computeNoiseModelEntryS(int32_t sens) {
    double s = gCamCapability[mCameraId]->gradient_S * sens +
            gCamCapability[mCameraId]->offset_S;
    return ((s < 0.0) ? 0.0 : s);
}

/*===========================================================================
 * FUNCTION   : computeNoiseModelEntryO
 *
 * DESCRIPTION: function to map a given sensitivity to the O noise
 *              model parameters in the DNG noise model.
 *
 * PARAMETERS : sens : the sensor sensitivity
 *
 ** RETURN    : O (sensor readout) noise
 *
 *==========================================================================*/
double QCamera3HardwareInterface::computeNoiseModelEntryO(int32_t sens) {
    double o = gCamCapability[mCameraId]->gradient_O * sens +
            gCamCapability[mCameraId]->offset_O;
    return ((o < 0.0) ? 0.0 : o);
}

/*===========================================================================
 * FUNCTION   : getSensorSensitivity
 *
 * DESCRIPTION: convert iso_mode to an integer value
 *
 * PARAMETERS : iso_mode : the iso_mode supported by sensor
 *
 ** RETURN    : sensitivity supported by sensor
 *
 *==========================================================================*/
int32_t QCamera3HardwareInterface::getSensorSensitivity(int32_t iso_mode)
{
    int32_t sensitivity;

    switch (iso_mode) {
    case CAM_ISO_MODE_100:
        sensitivity = 100;
        break;
    case CAM_ISO_MODE_200:
        sensitivity = 200;
        break;
    case CAM_ISO_MODE_400:
        sensitivity = 400;
        break;
    case CAM_ISO_MODE_800:
        sensitivity = 800;
        break;
    case CAM_ISO_MODE_1600:
        sensitivity = 1600;
        break;
    default:
        sensitivity = -1;
        break;
    }
    return sensitivity;
}

/*===========================================================================
 * FUNCTION   : getCamInfo
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
int QCamera3HardwareInterface::getCamInfo(uint32_t cameraId,
        struct camera_info *info)
{
    ATRACE_CALL();
    int rc = 0;

    pthread_mutex_lock(&gCamLock);
    if (NULL == gCamCapability[cameraId]) {
        rc = initCapabilities(cameraId);
        if (rc < 0) {
            pthread_mutex_unlock(&gCamLock);
            return rc;
        }
    }

    if (NULL == gStaticMetadata[cameraId]) {
        rc = initStaticMetadata(cameraId);
        if (rc < 0) {
            pthread_mutex_unlock(&gCamLock);
            return rc;
        }
    }

    switch(gCamCapability[cameraId]->position) {
    case CAM_POSITION_BACK:
        info->facing = CAMERA_FACING_BACK;
        break;

    case CAM_POSITION_FRONT:
        info->facing = CAMERA_FACING_FRONT;
        break;

    default:
        ALOGE("%s:Unknown position type for camera id:%d", __func__, cameraId);
        rc = -1;
        break;
    }


    info->orientation = (int)gCamCapability[cameraId]->sensor_mount_angle;
    info->device_version = CAMERA_DEVICE_API_VERSION_3_2;
    info->static_camera_characteristics = gStaticMetadata[cameraId];

    pthread_mutex_unlock(&gCamLock);

    return rc;
}

/*===========================================================================
 * FUNCTION   : translateCapabilityToMetadata
 *
 * DESCRIPTION: translate the capability into camera_metadata_t
 *
 * PARAMETERS : type of the request
 *
 *
 * RETURN     : success: camera_metadata_t*
 *              failure: NULL
 *
 *==========================================================================*/
camera_metadata_t* QCamera3HardwareInterface::translateCapabilityToMetadata(int type)
{
    if (mDefaultMetadata[type] != NULL) {
        return mDefaultMetadata[type];
    }
    //first time we are handling this request
    //fill up the metadata structure using the wrapper class
    CameraMetadata settings;
    //translate from cam_capability_t to camera_metadata_tag_t
    static const uint8_t requestType = ANDROID_REQUEST_TYPE_CAPTURE;
    settings.update(ANDROID_REQUEST_TYPE, &requestType, 1);
    int32_t defaultRequestID = 0;
    settings.update(ANDROID_REQUEST_ID, &defaultRequestID, 1);

    /* OIS disable */
    char ois_prop[PROPERTY_VALUE_MAX];
    memset(ois_prop, 0, sizeof(ois_prop));
    property_get("persist.camera.ois.disable", ois_prop, "0");
    uint8_t ois_disable = (uint8_t)atoi(ois_prop);

    /* Force video to use OIS */
    char videoOisProp[PROPERTY_VALUE_MAX];
    memset(videoOisProp, 0, sizeof(videoOisProp));
    property_get("persist.camera.ois.video", videoOisProp, "1");
    uint8_t forceVideoOis = (uint8_t)atoi(videoOisProp);

    uint8_t controlIntent = 0;
    uint8_t focusMode;
    uint8_t vsMode;
    uint8_t optStabMode;
    uint8_t cacMode;
    bool highQualityModeEntryAvailable = FALSE;
    bool fastModeEntryAvailable = FALSE;
    uint8_t edge_mode = ANDROID_EDGE_MODE_FAST;
    uint8_t noise_red_mode = ANDROID_NOISE_REDUCTION_MODE_FAST;
    uint8_t tonemap_mode = ANDROID_TONEMAP_MODE_FAST;
    vsMode = ANDROID_CONTROL_VIDEO_STABILIZATION_MODE_OFF;
    switch (type) {
      case CAMERA3_TEMPLATE_PREVIEW:
        controlIntent = ANDROID_CONTROL_CAPTURE_INTENT_PREVIEW;
        focusMode = ANDROID_CONTROL_AF_MODE_CONTINUOUS_PICTURE;
        optStabMode = ANDROID_LENS_OPTICAL_STABILIZATION_MODE_ON;
        break;
      case CAMERA3_TEMPLATE_STILL_CAPTURE:
        controlIntent = ANDROID_CONTROL_CAPTURE_INTENT_STILL_CAPTURE;
        focusMode = ANDROID_CONTROL_AF_MODE_CONTINUOUS_PICTURE;
        optStabMode = ANDROID_LENS_OPTICAL_STABILIZATION_MODE_ON;
        cacMode = ANDROID_COLOR_CORRECTION_ABERRATION_MODE_OFF;
        // Order of priority for default CAC is HIGH Quality -> FAST -> OFF
        for (size_t i = 0; i < gCamCapability[mCameraId]->aberration_modes_count; i++) {
            if (gCamCapability[mCameraId]->aberration_modes[i] ==
                    CAM_COLOR_CORRECTION_ABERRATION_HIGH_QUALITY) {
                highQualityModeEntryAvailable = TRUE;
            } else if (gCamCapability[mCameraId]->aberration_modes[i] ==
                    CAM_COLOR_CORRECTION_ABERRATION_FAST) {
                fastModeEntryAvailable = TRUE;
            }
        }
        if (highQualityModeEntryAvailable) {
            cacMode = ANDROID_COLOR_CORRECTION_ABERRATION_MODE_HIGH_QUALITY;
        } else if (fastModeEntryAvailable) {
            cacMode = ANDROID_COLOR_CORRECTION_ABERRATION_MODE_FAST;
        }
        #ifndef USE_L_MR1
        edge_mode = ANDROID_EDGE_MODE_HIGH_QUALITY;
        noise_red_mode = ANDROID_NOISE_REDUCTION_MODE_HIGH_QUALITY;
        tonemap_mode = ANDROID_TONEMAP_MODE_HIGH_QUALITY;
        #endif
        settings.update(ANDROID_COLOR_CORRECTION_ABERRATION_MODE, &cacMode, 1);
        break;
      case CAMERA3_TEMPLATE_VIDEO_RECORD:
        controlIntent = ANDROID_CONTROL_CAPTURE_INTENT_VIDEO_RECORD;
        focusMode = ANDROID_CONTROL_AF_MODE_CONTINUOUS_VIDEO;
        optStabMode = ANDROID_LENS_OPTICAL_STABILIZATION_MODE_OFF;
        if (forceVideoOis)
            optStabMode = ANDROID_LENS_OPTICAL_STABILIZATION_MODE_ON;
        break;
      case CAMERA3_TEMPLATE_VIDEO_SNAPSHOT:
        controlIntent = ANDROID_CONTROL_CAPTURE_INTENT_VIDEO_SNAPSHOT;
        focusMode = ANDROID_CONTROL_AF_MODE_CONTINUOUS_VIDEO;
        optStabMode = ANDROID_LENS_OPTICAL_STABILIZATION_MODE_OFF;
        if (forceVideoOis)
            optStabMode = ANDROID_LENS_OPTICAL_STABILIZATION_MODE_ON;
        break;
      case CAMERA3_TEMPLATE_ZERO_SHUTTER_LAG:
        controlIntent = ANDROID_CONTROL_CAPTURE_INTENT_ZERO_SHUTTER_LAG;
        focusMode = ANDROID_CONTROL_AF_MODE_CONTINUOUS_PICTURE;
        optStabMode = ANDROID_LENS_OPTICAL_STABILIZATION_MODE_ON;
        break;
      case CAMERA3_TEMPLATE_MANUAL:
        controlIntent = ANDROID_CONTROL_CAPTURE_INTENT_MANUAL;
        focusMode = ANDROID_CONTROL_AF_MODE_OFF;
        optStabMode = ANDROID_LENS_OPTICAL_STABILIZATION_MODE_OFF;
        break;
      default:
        controlIntent = ANDROID_CONTROL_CAPTURE_INTENT_CUSTOM;
        optStabMode = ANDROID_LENS_OPTICAL_STABILIZATION_MODE_OFF;
        break;
    }
    settings.update(ANDROID_CONTROL_CAPTURE_INTENT, &controlIntent, 1);
    settings.update(ANDROID_CONTROL_VIDEO_STABILIZATION_MODE, &vsMode, 1);
    if (gCamCapability[mCameraId]->supported_focus_modes_cnt == 1) {
        focusMode = ANDROID_CONTROL_AF_MODE_OFF;
    }
    settings.update(ANDROID_CONTROL_AF_MODE, &focusMode, 1);

    if (gCamCapability[mCameraId]->optical_stab_modes_count == 1 &&
            gCamCapability[mCameraId]->optical_stab_modes[0] == CAM_OPT_STAB_ON)
        optStabMode = ANDROID_LENS_OPTICAL_STABILIZATION_MODE_ON;
    else if ((gCamCapability[mCameraId]->optical_stab_modes_count == 1 &&
            gCamCapability[mCameraId]->optical_stab_modes[0] == CAM_OPT_STAB_OFF)
            || ois_disable)
        optStabMode = ANDROID_LENS_OPTICAL_STABILIZATION_MODE_OFF;
    settings.update(ANDROID_LENS_OPTICAL_STABILIZATION_MODE, &optStabMode, 1);

    settings.update(ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION,
            &gCamCapability[mCameraId]->exposure_compensation_default, 1);

    static const uint8_t aeLock = ANDROID_CONTROL_AE_LOCK_OFF;
    settings.update(ANDROID_CONTROL_AE_LOCK, &aeLock, 1);

    static const uint8_t awbLock = ANDROID_CONTROL_AWB_LOCK_OFF;
    settings.update(ANDROID_CONTROL_AWB_LOCK, &awbLock, 1);

    static const uint8_t awbMode = ANDROID_CONTROL_AWB_MODE_AUTO;
    settings.update(ANDROID_CONTROL_AWB_MODE, &awbMode, 1);

    static const uint8_t controlMode = ANDROID_CONTROL_MODE_AUTO;
    settings.update(ANDROID_CONTROL_MODE, &controlMode, 1);

    static const uint8_t effectMode = ANDROID_CONTROL_EFFECT_MODE_OFF;
    settings.update(ANDROID_CONTROL_EFFECT_MODE, &effectMode, 1);

    static const uint8_t sceneMode = ANDROID_CONTROL_SCENE_MODE_FACE_PRIORITY;
    settings.update(ANDROID_CONTROL_SCENE_MODE, &sceneMode, 1);

    static const uint8_t aeMode = ANDROID_CONTROL_AE_MODE_ON;
    settings.update(ANDROID_CONTROL_AE_MODE, &aeMode, 1);

    /*flash*/
    static const uint8_t flashMode = ANDROID_FLASH_MODE_OFF;
    settings.update(ANDROID_FLASH_MODE, &flashMode, 1);

    static const uint8_t flashFiringLevel = CAM_FLASH_FIRING_LEVEL_4;
    settings.update(ANDROID_FLASH_FIRING_POWER,
            &flashFiringLevel, 1);

    /* lens */
    float default_aperture = gCamCapability[mCameraId]->apertures[0];
    settings.update(ANDROID_LENS_APERTURE, &default_aperture, 1);

    if (gCamCapability[mCameraId]->filter_densities_count) {
        float default_filter_density = gCamCapability[mCameraId]->filter_densities[0];
        settings.update(ANDROID_LENS_FILTER_DENSITY, &default_filter_density,
                        gCamCapability[mCameraId]->filter_densities_count);
    }

    float default_focal_length = gCamCapability[mCameraId]->focal_length;
    settings.update(ANDROID_LENS_FOCAL_LENGTH, &default_focal_length, 1);

    float default_focus_distance = 0;
    settings.update(ANDROID_LENS_FOCUS_DISTANCE, &default_focus_distance, 1);

    static const uint8_t demosaicMode = ANDROID_DEMOSAIC_MODE_FAST;
    settings.update(ANDROID_DEMOSAIC_MODE, &demosaicMode, 1);

    static const uint8_t hotpixelMode = ANDROID_HOT_PIXEL_MODE_FAST;
    settings.update(ANDROID_HOT_PIXEL_MODE, &hotpixelMode, 1);

    static const int32_t testpatternMode = ANDROID_SENSOR_TEST_PATTERN_MODE_OFF;
    settings.update(ANDROID_SENSOR_TEST_PATTERN_MODE, &testpatternMode, 1);

    static const uint8_t faceDetectMode = ANDROID_STATISTICS_FACE_DETECT_MODE_FULL;
    settings.update(ANDROID_STATISTICS_FACE_DETECT_MODE, &faceDetectMode, 1);

    static const uint8_t histogramMode = ANDROID_STATISTICS_HISTOGRAM_MODE_OFF;
    settings.update(ANDROID_STATISTICS_HISTOGRAM_MODE, &histogramMode, 1);

    static const uint8_t sharpnessMapMode = ANDROID_STATISTICS_SHARPNESS_MAP_MODE_OFF;
    settings.update(ANDROID_STATISTICS_SHARPNESS_MAP_MODE, &sharpnessMapMode, 1);

    static const uint8_t hotPixelMapMode = ANDROID_STATISTICS_HOT_PIXEL_MAP_MODE_OFF;
    settings.update(ANDROID_STATISTICS_HOT_PIXEL_MAP_MODE, &hotPixelMapMode, 1);

    static const uint8_t lensShadingMode = ANDROID_STATISTICS_LENS_SHADING_MAP_MODE_OFF;
    settings.update(ANDROID_STATISTICS_LENS_SHADING_MAP_MODE, &lensShadingMode, 1);

    static const uint8_t blackLevelLock = ANDROID_BLACK_LEVEL_LOCK_OFF;
    settings.update(ANDROID_BLACK_LEVEL_LOCK, &blackLevelLock, 1);

    /* Exposure time(Update the Min Exposure Time)*/
    int64_t default_exposure_time = gCamCapability[mCameraId]->exposure_time_range[0];
    settings.update(ANDROID_SENSOR_EXPOSURE_TIME, &default_exposure_time, 1);

    /* frame duration */
    static const int64_t default_frame_duration = NSEC_PER_33MSEC;
    settings.update(ANDROID_SENSOR_FRAME_DURATION, &default_frame_duration, 1);

    /* sensitivity */
    static const int32_t default_sensitivity = 100;
    settings.update(ANDROID_SENSOR_SENSITIVITY, &default_sensitivity, 1);

    /*edge mode*/
    settings.update(ANDROID_EDGE_MODE, &edge_mode, 1);

    /*noise reduction mode*/
    settings.update(ANDROID_NOISE_REDUCTION_MODE, &noise_red_mode, 1);

    /*color correction mode*/
    static const uint8_t color_correct_mode = ANDROID_COLOR_CORRECTION_MODE_FAST;
    settings.update(ANDROID_COLOR_CORRECTION_MODE, &color_correct_mode, 1);

    /*transform matrix mode*/
    settings.update(ANDROID_TONEMAP_MODE, &tonemap_mode, 1);

    uint8_t edge_strength = (uint8_t)gCamCapability[mCameraId]->sharpness_ctrl.def_value;
    settings.update(ANDROID_EDGE_STRENGTH, &edge_strength, 1);

    int32_t scaler_crop_region[4];
    scaler_crop_region[0] = 0;
    scaler_crop_region[1] = 0;
    scaler_crop_region[2] = gCamCapability[mCameraId]->active_array_size.width;
    scaler_crop_region[3] = gCamCapability[mCameraId]->active_array_size.height;
    settings.update(ANDROID_SCALER_CROP_REGION, scaler_crop_region, 4);

    static const uint8_t antibanding_mode = ANDROID_CONTROL_AE_ANTIBANDING_MODE_AUTO;
    settings.update(ANDROID_CONTROL_AE_ANTIBANDING_MODE, &antibanding_mode, 1);

    /*focus distance*/
    float focus_distance = 0.0;
    settings.update(ANDROID_LENS_FOCUS_DISTANCE, &focus_distance, 1);

    /*target fps range: use maximum range for picture, and maximum fixed range for video*/
    float max_range = 0.0;
    float max_fixed_fps = 0.0;
    int32_t fps_range[2] = {0, 0};
    for (uint32_t i = 0; i < gCamCapability[mCameraId]->fps_ranges_tbl_cnt;
            i++) {
        float range = gCamCapability[mCameraId]->fps_ranges_tbl[i].max_fps -
            gCamCapability[mCameraId]->fps_ranges_tbl[i].min_fps;
        if (type == CAMERA3_TEMPLATE_PREVIEW ||
                type == CAMERA3_TEMPLATE_STILL_CAPTURE ||
                type == CAMERA3_TEMPLATE_ZERO_SHUTTER_LAG) {
            if (range > max_range) {
                fps_range[0] =
                    (int32_t)gCamCapability[mCameraId]->fps_ranges_tbl[i].min_fps;
                fps_range[1] =
                    (int32_t)gCamCapability[mCameraId]->fps_ranges_tbl[i].max_fps;
                max_range = range;
            }
        } else {
            if (range < 0.01 && max_fixed_fps <
                    gCamCapability[mCameraId]->fps_ranges_tbl[i].max_fps) {
                fps_range[0] =
                    (int32_t)gCamCapability[mCameraId]->fps_ranges_tbl[i].min_fps;
                fps_range[1] =
                    (int32_t)gCamCapability[mCameraId]->fps_ranges_tbl[i].max_fps;
                max_fixed_fps = gCamCapability[mCameraId]->fps_ranges_tbl[i].max_fps;
            }
        }
    }
    settings.update(ANDROID_CONTROL_AE_TARGET_FPS_RANGE, fps_range, 2);

    /*precapture trigger*/
    uint8_t precapture_trigger = ANDROID_CONTROL_AE_PRECAPTURE_TRIGGER_IDLE;
    settings.update(ANDROID_CONTROL_AE_PRECAPTURE_TRIGGER, &precapture_trigger, 1);

    /*af trigger*/
    uint8_t af_trigger = ANDROID_CONTROL_AF_TRIGGER_IDLE;
    settings.update(ANDROID_CONTROL_AF_TRIGGER, &af_trigger, 1);

    /* ae & af regions */
    int32_t active_region[] = {
            gCamCapability[mCameraId]->active_array_size.left,
            gCamCapability[mCameraId]->active_array_size.top,
            gCamCapability[mCameraId]->active_array_size.left +
                    gCamCapability[mCameraId]->active_array_size.width,
            gCamCapability[mCameraId]->active_array_size.top +
                    gCamCapability[mCameraId]->active_array_size.height,
            0};
    settings.update(ANDROID_CONTROL_AE_REGIONS, active_region,
            sizeof(active_region) / sizeof(active_region[0]));
    settings.update(ANDROID_CONTROL_AF_REGIONS, active_region,
            sizeof(active_region) / sizeof(active_region[0]));

    /* black level lock */
    uint8_t blacklevel_lock = ANDROID_BLACK_LEVEL_LOCK_OFF;
    settings.update(ANDROID_BLACK_LEVEL_LOCK, &blacklevel_lock, 1);

    /* face detect mode */
    uint8_t facedetect_mode = ANDROID_STATISTICS_FACE_DETECT_MODE_OFF;
    settings.update(ANDROID_STATISTICS_FACE_DETECT_MODE, &facedetect_mode, 1);

    /* lens shading map mode */
    uint8_t shadingmap_mode = ANDROID_STATISTICS_LENS_SHADING_MAP_MODE_OFF;
    if (CAM_SENSOR_RAW == gCamCapability[mCameraId]->sensor_type.sens_type) {
        shadingmap_mode = ANDROID_STATISTICS_LENS_SHADING_MAP_MODE_ON;
    }
    settings.update(ANDROID_STATISTICS_LENS_SHADING_MAP_MODE, &shadingmap_mode, 1);

    //special defaults for manual template
    if (type == CAMERA3_TEMPLATE_MANUAL) {
        static const uint8_t manualControlMode = ANDROID_CONTROL_MODE_OFF;
        settings.update(ANDROID_CONTROL_MODE, &manualControlMode, 1);

        static const uint8_t manualFocusMode = ANDROID_CONTROL_AF_MODE_OFF;
        settings.update(ANDROID_CONTROL_AF_MODE, &manualFocusMode, 1);

        static const uint8_t manualAeMode = ANDROID_CONTROL_AE_MODE_OFF;
        settings.update(ANDROID_CONTROL_AE_MODE, &manualAeMode, 1);

        static const uint8_t manualAwbMode = ANDROID_CONTROL_AWB_MODE_OFF;
        settings.update(ANDROID_CONTROL_AWB_MODE, &manualAwbMode, 1);

        static const uint8_t manualTonemapMode = ANDROID_TONEMAP_MODE_FAST;
        settings.update(ANDROID_TONEMAP_MODE, &manualTonemapMode, 1);

        static const uint8_t manualColorCorrectMode = ANDROID_COLOR_CORRECTION_MODE_TRANSFORM_MATRIX;
        settings.update(ANDROID_COLOR_CORRECTION_MODE, &manualColorCorrectMode, 1);
    }

    /* TNR default */
    uint8_t tnr_enable       = m_bTnrEnabled;
    int32_t tnr_process_type = (int32_t)getTemporalDenoiseProcessPlate();
    settings.update(QCAMERA3_TEMPORAL_DENOISE_ENABLE, &tnr_enable, 1);
    settings.update(QCAMERA3_TEMPORAL_DENOISE_PROCESS_TYPE, &tnr_process_type, 1);
    CDBG("%s: default TNR enable %d, process plate %d", __func__, tnr_enable, tnr_process_type);

    /* CDS default */
    char prop[PROPERTY_VALUE_MAX];
    memset(prop, 0, sizeof(prop));
    property_get("persist.camera.CDS", prop, "Auto");
    cam_cds_mode_type_t cds_mode = CAM_CDS_MODE_AUTO;
    cds_mode = lookupProp(CDS_MAP, METADATA_MAP_SIZE(CDS_MAP), prop);
    if (CAM_CDS_MODE_MAX == cds_mode) {
        cds_mode = CAM_CDS_MODE_AUTO;
    }
    //@note: force cds mode to be OFF when TNR is enabled.
    if (m_bTnrEnabled == true) {
        CDBG_HIGH("%s: default CDS mode %d is forced to be OFF because TNR is enabled.",
                __func__, cds_mode);
        cds_mode = CAM_CDS_MODE_OFF;
    }
    int32_t mode = cds_mode;
    settings.update(QCAMERA3_CDS_MODE, &mode, 1);
    mDefaultMetadata[type] = settings.release();

    return mDefaultMetadata[type];
}

/*===========================================================================
 * FUNCTION   : setFrameParameters
 *
 * DESCRIPTION: set parameters per frame as requested in the metadata from
 *              framework
 *
 * PARAMETERS :
 *   @request   : request that needs to be serviced
 *   @streamID : Stream ID of all the requested streams
 *   @blob_request: Whether this request is a blob request or not
 *
 * RETURN     : success: NO_ERROR
 *              failure:
 *==========================================================================*/
int QCamera3HardwareInterface::setFrameParameters(
                    camera3_capture_request_t *request,
                    cam_stream_ID_t streamID,
                    int blob_request,
                    uint32_t snapshotStreamId)
{
    /*translate from camera_metadata_t type to parm_type_t*/
    int rc = 0;
    int32_t hal_version = CAM_HAL_V3;

    clear_metadata_buffer(mParameters);
    if (ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_PARM_HAL_VERSION, hal_version)) {
        ALOGE("%s: Failed to set hal version in the parameters", __func__);
        return BAD_VALUE;
    }

    /*we need to update the frame number in the parameters*/
    if (ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_META_FRAME_NUMBER,
            request->frame_number)) {
        ALOGE("%s: Failed to set the frame number in the parameters", __func__);
        return BAD_VALUE;
    }

    /* Update stream id of all the requested buffers */
    if (ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_META_STREAM_ID, streamID)) {
        ALOGE("%s: Failed to set stream type mask in the parameters", __func__);
        return BAD_VALUE;
    }

    if (mUpdateDebugLevel) {
        uint32_t dummyDebugLevel = 0;
        /* The value of dummyDebugLevel is irrelavent. On
         * CAM_INTF_PARM_UPDATE_DEBUG_LEVEL, read debug property */
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_PARM_UPDATE_DEBUG_LEVEL,
                dummyDebugLevel)) {
            ALOGE("%s: Failed to set UPDATE_DEBUG_LEVEL", __func__);
            return BAD_VALUE;
        }
        mUpdateDebugLevel = false;
    }

    if(request->settings != NULL){
        rc = translateToHalMetadata(request, mParameters, snapshotStreamId);
        if (blob_request)
            memcpy(mPrevParameters, mParameters, sizeof(metadata_buffer_t));
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : setReprocParameters
 *
 * DESCRIPTION: Translate frameworks metadata to HAL metadata structure, and
 *              return it.
 *
 * PARAMETERS :
 *   @request   : request that needs to be serviced
 *
 * RETURN     : success: NO_ERROR
 *              failure:
 *==========================================================================*/
int32_t QCamera3HardwareInterface::setReprocParameters(
        camera3_capture_request_t *request, metadata_buffer_t *reprocParam,
        uint32_t snapshotStreamId)
{
    /*translate from camera_metadata_t type to parm_type_t*/
    int rc = 0;

    if (NULL == request->settings){
        ALOGE("%s: Reprocess settings cannot be NULL", __func__);
        return BAD_VALUE;
    }

    if (NULL == reprocParam) {
        ALOGE("%s: Invalid reprocessing metadata buffer", __func__);
        return BAD_VALUE;
    }
    clear_metadata_buffer(reprocParam);

    /*we need to update the frame number in the parameters*/
    if (ADD_SET_PARAM_ENTRY_TO_BATCH(reprocParam, CAM_INTF_META_FRAME_NUMBER,
            request->frame_number)) {
        ALOGE("%s: Failed to set the frame number in the parameters", __func__);
        return BAD_VALUE;
    }

    rc = translateToHalMetadata(request, reprocParam, snapshotStreamId);
    if (rc < 0) {
        ALOGE("%s: Failed to translate reproc request", __func__);
        return rc;
    }

    CameraMetadata frame_settings;
    frame_settings = request->settings;
    if (frame_settings.exists(QCAMERA3_CROP_COUNT_REPROCESS) &&
            frame_settings.exists(QCAMERA3_CROP_REPROCESS) &&
            frame_settings.exists(QCAMERA3_CROP_STREAM_ID_REPROCESS)) {
        int32_t *crop_count =
                frame_settings.find(QCAMERA3_CROP_COUNT_REPROCESS).data.i32;
        int32_t *crop_data =
                frame_settings.find(QCAMERA3_CROP_REPROCESS).data.i32;
        int32_t *crop_stream_ids =
                frame_settings.find(QCAMERA3_CROP_STREAM_ID_REPROCESS).data.i32;
        int32_t *roi_map =
                frame_settings.find(QCAMERA3_CROP_ROI_MAP_REPROCESS).data.i32;
        if ((0 < *crop_count) && (*crop_count < MAX_NUM_STREAMS)) {
            bool found = false;
            int32_t i;
            for (i = 0; i < *crop_count; i++) {
#ifdef __LP64__
                int32_t id = (int32_t)
                        ((((int64_t)request->input_buffer->stream) & 0x0000FFFF) ^
                                (((int64_t)request->input_buffer->stream) >> 0x20 & 0x0000FFFF));
#else
                int32_t id = (int32_t) request->input_buffer->stream;
#endif
                if (crop_stream_ids[i] == id) {
                    found = true;
                    break;
                }
            }

            if (found) {
                cam_crop_data_t crop_meta;
                size_t roi_map_idx = i*4;
                size_t crop_info_idx = i*4;
                memset(&crop_meta, 0, sizeof(cam_crop_data_t));
                crop_meta.num_of_streams = 1;
                crop_meta.crop_info[0].crop.left   = crop_data[crop_info_idx++];
                crop_meta.crop_info[0].crop.top    = crop_data[crop_info_idx++];
                crop_meta.crop_info[0].crop.width  = crop_data[crop_info_idx++];
                crop_meta.crop_info[0].crop.height = crop_data[crop_info_idx++];

                crop_meta.crop_info[0].roi_map.left =
                        roi_map[roi_map_idx++];
                crop_meta.crop_info[0].roi_map.top =
                        roi_map[roi_map_idx++];
                crop_meta.crop_info[0].roi_map.width =
                        roi_map[roi_map_idx++];
                crop_meta.crop_info[0].roi_map.height =
                        roi_map[roi_map_idx++];

                if (ADD_SET_PARAM_ENTRY_TO_BATCH(reprocParam, CAM_INTF_META_CROP_DATA, crop_meta)) {
                    rc = BAD_VALUE;
                }
                CDBG("%s: Found reprocess crop data for stream %p %dx%d, %dx%d",
                        __func__,
                        request->input_buffer->stream,
                        crop_meta.crop_info[0].crop.left,
                        crop_meta.crop_info[0].crop.top,
                        crop_meta.crop_info[0].crop.width,
                        crop_meta.crop_info[0].crop.height);
                CDBG("%s: Found reprocess roi map data for stream %p %dx%d, %dx%d",
                        __func__,
                        request->input_buffer->stream,
                        crop_meta.crop_info[0].roi_map.left,
                        crop_meta.crop_info[0].roi_map.top,
                        crop_meta.crop_info[0].roi_map.width,
                        crop_meta.crop_info[0].roi_map.height);
            } else {
                ALOGE("%s: No matching reprocess input stream found!", __func__);
            }
        } else {
            ALOGE("%s: Invalid reprocess crop count %d!", __func__, *crop_count);
        }
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : setHalFpsRange
 *
 * DESCRIPTION: set FPS range parameter
 *
 *
 * PARAMETERS :
 *   @settings    : Metadata from framework
 *   @hal_metadata: Metadata buffer
 *
 *
 * RETURN     : success: NO_ERROR
 *              failure:
 *==========================================================================*/
int32_t QCamera3HardwareInterface::setHalFpsRange(const CameraMetadata &settings,
        metadata_buffer_t *hal_metadata)
{
    int32_t rc = NO_ERROR;
    cam_fps_range_t fps_range;
    fps_range.min_fps = (float)
            settings.find(ANDROID_CONTROL_AE_TARGET_FPS_RANGE).data.i32[0];
    fps_range.max_fps = (float)
            settings.find(ANDROID_CONTROL_AE_TARGET_FPS_RANGE).data.i32[1];
    fps_range.video_min_fps = fps_range.min_fps;
    fps_range.video_max_fps = fps_range.max_fps;
    if (ADD_SET_PARAM_ENTRY_TO_BATCH(hal_metadata, CAM_INTF_PARM_FPS_RANGE, fps_range)) {
        rc = BAD_VALUE;
    }
    if (rc == NO_ERROR) {
        mFpsRange = fps_range;
    }
    CDBG("%s: fps: [%f %f] vid_fps: [%f %f]", __func__, fps_range.min_fps,
            fps_range.max_fps, fps_range.video_min_fps, fps_range.video_max_fps);
    return rc;
}

/*===========================================================================
 * FUNCTION   : translateToHalMetadata
 *
 * DESCRIPTION: read from the camera_metadata_t and change to parm_type_t
 *
 *
 * PARAMETERS :
 *   @request  : request sent from framework
 *
 *
 * RETURN     : success: NO_ERROR
 *              failure:
 *==========================================================================*/
int QCamera3HardwareInterface::translateToHalMetadata
                                  (const camera3_capture_request_t *request,
                                   metadata_buffer_t *hal_metadata,
                                   uint32_t snapshotStreamId)
{
    int rc = 0;
    CameraMetadata frame_settings;
    frame_settings = request->settings;

    /* Do not change the order of the following list unless you know what you are
     * doing.
     * The order is laid out in such a way that parameters in the front of the table
     * may be used to override the parameters later in the table. Examples are:
     * 1. META_MODE should precede AEC/AWB/AF MODE
     * 2. AEC MODE should preced EXPOSURE_TIME/SENSITIVITY/FRAME_DURATION
     * 3. AWB_MODE should precede COLOR_CORRECTION_MODE
     * 4. Any mode should precede it's corresponding settings
     */
    if (frame_settings.exists(ANDROID_CONTROL_MODE)) {
        uint8_t metaMode = frame_settings.find(ANDROID_CONTROL_MODE).data.u8[0];
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(hal_metadata, CAM_INTF_META_MODE, metaMode)) {
            rc = BAD_VALUE;
        }
        rc = extractSceneMode(frame_settings, metaMode, hal_metadata);
        if (rc != NO_ERROR) {
            ALOGE("%s: extractSceneMode failed", __func__);
        }
    }

    if (frame_settings.exists(ANDROID_CONTROL_AE_MODE)) {
        uint8_t fwk_aeMode =
            frame_settings.find(ANDROID_CONTROL_AE_MODE).data.u8[0];
        uint8_t aeMode;
        int32_t redeye;

        if (fwk_aeMode == ANDROID_CONTROL_AE_MODE_OFF ) {
            aeMode = CAM_AE_MODE_OFF;
        } else {
            aeMode = CAM_AE_MODE_ON;
        }
        if (fwk_aeMode == ANDROID_CONTROL_AE_MODE_ON_AUTO_FLASH_REDEYE) {
            redeye = 1;
        } else {
            redeye = 0;
        }

        int val = lookupHalName(AE_FLASH_MODE_MAP, METADATA_MAP_SIZE(AE_FLASH_MODE_MAP),
                fwk_aeMode);
        if (NAME_NOT_FOUND != val) {
            int32_t flashMode = (int32_t)val;
            ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_PARM_LED_MODE, flashMode);
            ADD_SET_PARAM_ENTRY_TO_BATCH(hal_metadata, CAM_INTF_PARM_LED_MODE, flashMode);
        }

        ADD_SET_PARAM_ENTRY_TO_BATCH(hal_metadata, CAM_INTF_META_AEC_MODE, aeMode);
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(hal_metadata, CAM_INTF_PARM_REDEYE_REDUCTION, redeye)) {
            rc = BAD_VALUE;
        }
    }

    if (frame_settings.exists(ANDROID_CONTROL_AWB_MODE)) {
        uint8_t fwk_whiteLevel = frame_settings.find(ANDROID_CONTROL_AWB_MODE).data.u8[0];
        int val = lookupHalName(WHITE_BALANCE_MODES_MAP, METADATA_MAP_SIZE(WHITE_BALANCE_MODES_MAP),
                fwk_whiteLevel);
        if (NAME_NOT_FOUND != val) {
            uint8_t whiteLevel = (uint8_t)val;
            if (ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_PARM_WHITE_BALANCE, whiteLevel)) {
                rc = BAD_VALUE;
            }
        }
    }

    if (frame_settings.exists(ANDROID_COLOR_CORRECTION_ABERRATION_MODE)) {
        uint8_t fwk_cacMode =
                frame_settings.find(
                        ANDROID_COLOR_CORRECTION_ABERRATION_MODE).data.u8[0];
        int val = lookupHalName(COLOR_ABERRATION_MAP, METADATA_MAP_SIZE(COLOR_ABERRATION_MAP),
                fwk_cacMode);
        if (NAME_NOT_FOUND != val) {
            cam_aberration_mode_t cacMode = (cam_aberration_mode_t) val;
            bool entryAvailable = FALSE;
            // Check whether Frameworks set CAC mode is supported in device or not
            for (size_t i = 0; i < gCamCapability[mCameraId]->aberration_modes_count; i++) {
                if (gCamCapability[mCameraId]->aberration_modes[i] == cacMode) {
                    entryAvailable = TRUE;
                    break;
                }
            }
            CDBG("%s: FrameworksCacMode=%d entryAvailable=%d", __func__, cacMode, entryAvailable);
            // If entry not found then set the device supported mode instead of frameworks mode i.e,
            // Only HW ISP CAC + NO SW CAC : Advertise all 3 with High doing same as fast by ISP
            // NO HW ISP CAC + Only SW CAC : Advertise all 3 with Fast doing the same as OFF
            if (entryAvailable == FALSE) {
                if (gCamCapability[mCameraId]->aberration_modes_count == 0) {
                    cacMode = CAM_COLOR_CORRECTION_ABERRATION_OFF;
                } else {
                    if (cacMode == CAM_COLOR_CORRECTION_ABERRATION_HIGH_QUALITY) {
                        // High is not supported and so set the FAST as spec say's underlying
                        // device implementation can be the same for both modes.
                        cacMode = CAM_COLOR_CORRECTION_ABERRATION_FAST;
                    } else if (cacMode == CAM_COLOR_CORRECTION_ABERRATION_FAST) {
                        // Fast is not supported and so we cannot set HIGH or FAST but choose OFF
                        // in order to avoid the fps drop due to high quality
                        cacMode = CAM_COLOR_CORRECTION_ABERRATION_OFF;
                    } else {
                        cacMode = CAM_COLOR_CORRECTION_ABERRATION_OFF;
                    }
                }
            }
            CDBG("%s: Final cacMode is %d", __func__, cacMode);
            if (ADD_SET_PARAM_ENTRY_TO_BATCH(hal_metadata, CAM_INTF_PARM_CAC, cacMode)) {
                rc = BAD_VALUE;
            }
        } else {
            ALOGE("%s: Invalid framework CAC mode: %d", __func__, fwk_cacMode);
        }
    }

    if (frame_settings.exists(ANDROID_CONTROL_AF_MODE)) {
        uint8_t fwk_focusMode = frame_settings.find(ANDROID_CONTROL_AF_MODE).data.u8[0];
        int val = lookupHalName(FOCUS_MODES_MAP, METADATA_MAP_SIZE(FOCUS_MODES_MAP),
                fwk_focusMode);
        if (NAME_NOT_FOUND != val) {
            uint8_t focusMode = (uint8_t)val;
            CDBG("%s: set focus mode %d", __func__, focusMode);
            if (ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_PARM_FOCUS_MODE, focusMode)) {
                rc = BAD_VALUE;
            }
        }
    }

    if (frame_settings.exists(ANDROID_LENS_FOCUS_DISTANCE)) {
        float focalDistance = frame_settings.find(ANDROID_LENS_FOCUS_DISTANCE).data.f[0];
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_META_LENS_FOCUS_DISTANCE,
                focalDistance)) {
            rc = BAD_VALUE;
        }
    }

    if (frame_settings.exists(ANDROID_CONTROL_AE_ANTIBANDING_MODE)) {
        uint8_t fwk_antibandingMode =
                frame_settings.find(ANDROID_CONTROL_AE_ANTIBANDING_MODE).data.u8[0];
        int val = lookupHalName(ANTIBANDING_MODES_MAP,
                METADATA_MAP_SIZE(ANTIBANDING_MODES_MAP), fwk_antibandingMode);
        if (NAME_NOT_FOUND != val) {
            uint32_t hal_antibandingMode = (uint32_t)val;
            if (ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_PARM_ANTIBANDING,
                    hal_antibandingMode)) {
                rc = BAD_VALUE;
            }
        }
    }

    if (frame_settings.exists(ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION)) {
        int32_t expCompensation = frame_settings.find(
                ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION).data.i32[0];
        if (expCompensation < gCamCapability[mCameraId]->exposure_compensation_min)
            expCompensation = gCamCapability[mCameraId]->exposure_compensation_min;
        if (expCompensation > gCamCapability[mCameraId]->exposure_compensation_max)
            expCompensation = gCamCapability[mCameraId]->exposure_compensation_max;
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(hal_metadata, CAM_INTF_PARM_EXPOSURE_COMPENSATION,
                expCompensation)) {
            rc = BAD_VALUE;
        }
    }

    if (frame_settings.exists(ANDROID_CONTROL_AE_LOCK)) {
        uint8_t aeLock = frame_settings.find(ANDROID_CONTROL_AE_LOCK).data.u8[0];
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(hal_metadata, CAM_INTF_PARM_AEC_LOCK, aeLock)) {
            rc = BAD_VALUE;
        }
    }
    if (frame_settings.exists(ANDROID_CONTROL_AE_TARGET_FPS_RANGE)) {
        rc = setHalFpsRange(frame_settings, hal_metadata);
        if (rc != NO_ERROR) {
            ALOGE("%s: setHalFpsRange failed", __func__);
        }
    }

    if (frame_settings.exists(ANDROID_CONTROL_AWB_LOCK)) {
        uint8_t awbLock = frame_settings.find(ANDROID_CONTROL_AWB_LOCK).data.u8[0];
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_PARM_AWB_LOCK, awbLock)) {
            rc = BAD_VALUE;
        }
    }

    if (frame_settings.exists(ANDROID_CONTROL_EFFECT_MODE)) {
        uint8_t fwk_effectMode = frame_settings.find(ANDROID_CONTROL_EFFECT_MODE).data.u8[0];
        int val = lookupHalName(EFFECT_MODES_MAP, METADATA_MAP_SIZE(EFFECT_MODES_MAP),
                fwk_effectMode);
        if (NAME_NOT_FOUND != val) {
            uint8_t effectMode = (uint8_t)val;
            if (ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_PARM_EFFECT, effectMode)) {
                rc = BAD_VALUE;
            }
        }
    }

    if (frame_settings.exists(ANDROID_COLOR_CORRECTION_MODE)) {
        uint8_t colorCorrectMode = frame_settings.find(ANDROID_COLOR_CORRECTION_MODE).data.u8[0];
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_META_COLOR_CORRECT_MODE,
                colorCorrectMode)) {
            rc = BAD_VALUE;
        }
    }

    if (frame_settings.exists(ANDROID_COLOR_CORRECTION_GAINS)) {
        cam_color_correct_gains_t colorCorrectGains;
        for (size_t i = 0; i < CC_GAINS_COUNT; i++) {
            colorCorrectGains.gains[i] =
                    frame_settings.find(ANDROID_COLOR_CORRECTION_GAINS).data.f[i];
        }
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_META_COLOR_CORRECT_GAINS,
                colorCorrectGains)) {
            rc = BAD_VALUE;
        }
    }

    if (frame_settings.exists(ANDROID_COLOR_CORRECTION_TRANSFORM)) {
        cam_color_correct_matrix_t colorCorrectTransform;
        cam_rational_type_t transform_elem;
        size_t num = 0;
        for (size_t i = 0; i < CC_MATRIX_ROWS; i++) {
           for (size_t j = 0; j < CC_MATRIX_COLS; j++) {
              transform_elem.numerator =
                 frame_settings.find(ANDROID_COLOR_CORRECTION_TRANSFORM).data.r[num].numerator;
              transform_elem.denominator =
                 frame_settings.find(ANDROID_COLOR_CORRECTION_TRANSFORM).data.r[num].denominator;
              colorCorrectTransform.transform_matrix[i][j] = transform_elem;
              num++;
           }
        }
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(hal_metadata, CAM_INTF_META_COLOR_CORRECT_TRANSFORM,
                colorCorrectTransform)) {
            rc = BAD_VALUE;
        }
    }

    cam_trigger_t aecTrigger;
    aecTrigger.trigger = CAM_AEC_TRIGGER_IDLE;
    aecTrigger.trigger_id = -1;
    if (frame_settings.exists(ANDROID_CONTROL_AE_PRECAPTURE_TRIGGER)&&
        frame_settings.exists(ANDROID_CONTROL_AE_PRECAPTURE_ID)) {
        aecTrigger.trigger =
            frame_settings.find(ANDROID_CONTROL_AE_PRECAPTURE_TRIGGER).data.u8[0];
        aecTrigger.trigger_id =
            frame_settings.find(ANDROID_CONTROL_AE_PRECAPTURE_ID).data.i32[0];
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(hal_metadata, CAM_INTF_META_AEC_PRECAPTURE_TRIGGER,
                aecTrigger)) {
            rc = BAD_VALUE;
        }
        CDBG("%s: precaptureTrigger: %d precaptureTriggerID: %d", __func__,
                aecTrigger.trigger, aecTrigger.trigger_id);
    }

    /*af_trigger must come with a trigger id*/
    if (frame_settings.exists(ANDROID_CONTROL_AF_TRIGGER) &&
        frame_settings.exists(ANDROID_CONTROL_AF_TRIGGER_ID)) {
        cam_trigger_t af_trigger;
        af_trigger.trigger =
            frame_settings.find(ANDROID_CONTROL_AF_TRIGGER).data.u8[0];
        af_trigger.trigger_id =
            frame_settings.find(ANDROID_CONTROL_AF_TRIGGER_ID).data.i32[0];
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(hal_metadata, CAM_INTF_META_AF_TRIGGER, af_trigger)) {
            rc = BAD_VALUE;
        }
        CDBG("%s: AfTrigger: %d AfTriggerID: %d", __func__,
                af_trigger.trigger, af_trigger.trigger_id);
    }

    if (frame_settings.exists(ANDROID_DEMOSAIC_MODE)) {
        int32_t demosaic = frame_settings.find(ANDROID_DEMOSAIC_MODE).data.u8[0];
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_META_DEMOSAIC, demosaic)) {
            rc = BAD_VALUE;
        }
    }

    if (frame_settings.exists(ANDROID_EDGE_MODE)) {
        cam_edge_application_t edge_application;
        edge_application.edge_mode = frame_settings.find(ANDROID_EDGE_MODE).data.u8[0];
        if (edge_application.edge_mode == CAM_EDGE_MODE_OFF) {
            edge_application.sharpness = 0;
        } else {
            if (frame_settings.exists(ANDROID_EDGE_STRENGTH)) {
                uint8_t edgeStrength = frame_settings.find(ANDROID_EDGE_STRENGTH).data.u8[0];
                edge_application.sharpness = (int32_t)edgeStrength;
            } else {
                edge_application.sharpness = gCamCapability[mCameraId]->sharpness_ctrl.def_value; //default
            }
        }
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(hal_metadata, CAM_INTF_META_EDGE_MODE, edge_application)) {
            rc = BAD_VALUE;
        }
    }

    if (frame_settings.exists(ANDROID_FLASH_MODE)) {
        int32_t respectFlashMode = 1;
        if (frame_settings.exists(ANDROID_CONTROL_AE_MODE)) {
            uint8_t fwk_aeMode =
                frame_settings.find(ANDROID_CONTROL_AE_MODE).data.u8[0];
            if (fwk_aeMode > ANDROID_CONTROL_AE_MODE_ON) {
                respectFlashMode = 0;
                CDBG_HIGH("%s: AE Mode controls flash, ignore android.flash.mode",
                    __func__);
            }
        }
        if (respectFlashMode) {
            int val = lookupHalName(FLASH_MODES_MAP, METADATA_MAP_SIZE(FLASH_MODES_MAP),
                    (int)frame_settings.find(ANDROID_FLASH_MODE).data.u8[0]);
            CDBG_HIGH("%s: flash mode after mapping %d", __func__, val);
            // To check: CAM_INTF_META_FLASH_MODE usage
            if (NAME_NOT_FOUND != val) {
                uint8_t flashMode = (uint8_t)val;
                if (ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_PARM_LED_MODE, flashMode)) {
                    rc = BAD_VALUE;
                }
            }
        }
    }

    if (frame_settings.exists(ANDROID_FLASH_FIRING_POWER)) {
        uint8_t flashPower = frame_settings.find(ANDROID_FLASH_FIRING_POWER).data.u8[0];
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_META_FLASH_POWER, flashPower)) {
            rc = BAD_VALUE;
        }
    }

    if (frame_settings.exists(ANDROID_FLASH_FIRING_TIME)) {
        int64_t flashFiringTime = frame_settings.find(ANDROID_FLASH_FIRING_TIME).data.i64[0];
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(hal_metadata, CAM_INTF_META_FLASH_FIRING_TIME,
                flashFiringTime)) {
            rc = BAD_VALUE;
        }
    }

    if (frame_settings.exists(ANDROID_HOT_PIXEL_MODE)) {
        uint8_t hotPixelMode = frame_settings.find(ANDROID_HOT_PIXEL_MODE).data.u8[0];
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(hal_metadata, CAM_INTF_META_HOTPIXEL_MODE,
                hotPixelMode)) {
            rc = BAD_VALUE;
        }
    }

    if (frame_settings.exists(ANDROID_LENS_APERTURE)) {
        float lensAperture = frame_settings.find( ANDROID_LENS_APERTURE).data.f[0];
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_META_LENS_APERTURE,
                lensAperture)) {
            rc = BAD_VALUE;
        }
    }

    if (frame_settings.exists(ANDROID_LENS_FILTER_DENSITY)) {
        float filterDensity = frame_settings.find(ANDROID_LENS_FILTER_DENSITY).data.f[0];
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_META_LENS_FILTERDENSITY,
                filterDensity)) {
            rc = BAD_VALUE;
        }
    }

    if (frame_settings.exists(ANDROID_LENS_FOCAL_LENGTH)) {
        float focalLength = frame_settings.find(ANDROID_LENS_FOCAL_LENGTH).data.f[0];
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_META_LENS_FOCAL_LENGTH, focalLength)) {
            rc = BAD_VALUE;
        }
    }

    if (frame_settings.exists(ANDROID_LENS_OPTICAL_STABILIZATION_MODE)) {
        uint8_t optStabMode =
                frame_settings.find(ANDROID_LENS_OPTICAL_STABILIZATION_MODE).data.u8[0];
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_META_LENS_OPT_STAB_MODE, optStabMode)) {
            rc = BAD_VALUE;
        }
    }

    if (frame_settings.exists(ANDROID_NOISE_REDUCTION_MODE)) {
        uint8_t noiseRedMode = frame_settings.find(ANDROID_NOISE_REDUCTION_MODE).data.u8[0];
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_META_NOISE_REDUCTION_MODE,
                noiseRedMode)) {
            rc = BAD_VALUE;
        }
    }

    if (frame_settings.exists(ANDROID_NOISE_REDUCTION_STRENGTH)) {
        uint8_t noiseRedStrength =
                frame_settings.find(ANDROID_NOISE_REDUCTION_STRENGTH).data.u8[0];
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_META_NOISE_REDUCTION_STRENGTH,
                noiseRedStrength)) {
            rc = BAD_VALUE;
        }
    }

    cam_crop_region_t scalerCropRegion;
    bool scalerCropSet = false;
    if (frame_settings.exists(ANDROID_SCALER_CROP_REGION)) {
        scalerCropRegion.left = frame_settings.find(ANDROID_SCALER_CROP_REGION).data.i32[0];
        scalerCropRegion.top = frame_settings.find(ANDROID_SCALER_CROP_REGION).data.i32[1];
        scalerCropRegion.width = frame_settings.find(ANDROID_SCALER_CROP_REGION).data.i32[2];
        scalerCropRegion.height = frame_settings.find(ANDROID_SCALER_CROP_REGION).data.i32[3];

        // Map coordinate system from active array to sensor output.
        mCropRegionMapper.toSensor(scalerCropRegion.left, scalerCropRegion.top,
                scalerCropRegion.width, scalerCropRegion.height);

        if (ADD_SET_PARAM_ENTRY_TO_BATCH(hal_metadata, CAM_INTF_META_SCALER_CROP_REGION,
                scalerCropRegion)) {
            rc = BAD_VALUE;
        }
        scalerCropSet = true;
    }

    if (frame_settings.exists(ANDROID_SENSOR_EXPOSURE_TIME)) {
        int64_t sensorExpTime =
                frame_settings.find(ANDROID_SENSOR_EXPOSURE_TIME).data.i64[0];
        CDBG("%s: setting sensorExpTime %lld", __func__, sensorExpTime);
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(hal_metadata, CAM_INTF_META_SENSOR_EXPOSURE_TIME,
                sensorExpTime)) {
            rc = BAD_VALUE;
        }
    }

    if (frame_settings.exists(ANDROID_SENSOR_FRAME_DURATION)) {
        int64_t sensorFrameDuration =
                frame_settings.find(ANDROID_SENSOR_FRAME_DURATION).data.i64[0];
        int64_t minFrameDuration = getMinFrameDuration(request);
        sensorFrameDuration = MAX(sensorFrameDuration, minFrameDuration);
        if (sensorFrameDuration > gCamCapability[mCameraId]->max_frame_duration)
            sensorFrameDuration = gCamCapability[mCameraId]->max_frame_duration;
        CDBG("%s: clamp sensorFrameDuration to %lld", __func__, sensorFrameDuration);
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(hal_metadata, CAM_INTF_META_SENSOR_FRAME_DURATION,
                sensorFrameDuration)) {
            rc = BAD_VALUE;
        }
    }

    if (frame_settings.exists(ANDROID_SENSOR_SENSITIVITY)) {
        int32_t sensorSensitivity = frame_settings.find(ANDROID_SENSOR_SENSITIVITY).data.i32[0];
        if (sensorSensitivity < gCamCapability[mCameraId]->sensitivity_range.min_sensitivity)
                sensorSensitivity = gCamCapability[mCameraId]->sensitivity_range.min_sensitivity;
        if (sensorSensitivity > gCamCapability[mCameraId]->sensitivity_range.max_sensitivity)
                sensorSensitivity = gCamCapability[mCameraId]->sensitivity_range.max_sensitivity;
        CDBG("%s: clamp sensorSensitivity to %d", __func__, sensorSensitivity);
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(hal_metadata, CAM_INTF_META_SENSOR_SENSITIVITY,
                sensorSensitivity)) {
            rc = BAD_VALUE;
        }
    }

    if (frame_settings.exists(ANDROID_SHADING_MODE)) {
        uint8_t shadingMode = frame_settings.find(ANDROID_SHADING_MODE).data.u8[0];
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_META_SHADING_MODE, shadingMode)) {
            rc = BAD_VALUE;
        }
    }

    if (frame_settings.exists(ANDROID_SHADING_STRENGTH)) {
        uint8_t shadingStrength = frame_settings.find(ANDROID_SHADING_STRENGTH).data.u8[0];
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_META_SHADING_STRENGTH,
                shadingStrength)) {
            rc = BAD_VALUE;
        }
    }


    if (frame_settings.exists(ANDROID_STATISTICS_FACE_DETECT_MODE)) {
        uint8_t fwk_facedetectMode =
                frame_settings.find(ANDROID_STATISTICS_FACE_DETECT_MODE).data.u8[0];
        int val = lookupHalName(FACEDETECT_MODES_MAP, METADATA_MAP_SIZE(FACEDETECT_MODES_MAP),
                fwk_facedetectMode);
        if (NAME_NOT_FOUND != val) {
            uint8_t facedetectMode = (uint8_t)val;
            if (ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_META_STATS_FACEDETECT_MODE,
                    facedetectMode)) {
                rc = BAD_VALUE;
            }
        }
    }

    if (frame_settings.exists(ANDROID_STATISTICS_HISTOGRAM_MODE)) {
        uint8_t histogramMode =
                frame_settings.find(ANDROID_STATISTICS_HISTOGRAM_MODE).data.u8[0];
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_META_STATS_HISTOGRAM_MODE,
                histogramMode)) {
            rc = BAD_VALUE;
        }
    }

    if (frame_settings.exists(ANDROID_STATISTICS_SHARPNESS_MAP_MODE)) {
        uint8_t sharpnessMapMode =
                frame_settings.find(ANDROID_STATISTICS_SHARPNESS_MAP_MODE).data.u8[0];
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_META_STATS_SHARPNESS_MAP_MODE,
                sharpnessMapMode)) {
            rc = BAD_VALUE;
        }
    }

    if (frame_settings.exists(ANDROID_TONEMAP_MODE)) {
        uint8_t tonemapMode =
                frame_settings.find(ANDROID_TONEMAP_MODE).data.u8[0];
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_META_TONEMAP_MODE, tonemapMode)) {
            rc = BAD_VALUE;
        }
    }
    /* Tonemap curve channels ch0 = G, ch 1 = B, ch 2 = R */
    /*All tonemap channels will have the same number of points*/
    if (frame_settings.exists(ANDROID_TONEMAP_CURVE_GREEN) &&
        frame_settings.exists(ANDROID_TONEMAP_CURVE_BLUE) &&
        frame_settings.exists(ANDROID_TONEMAP_CURVE_RED)) {
        cam_rgb_tonemap_curves tonemapCurves;
        tonemapCurves.tonemap_points_cnt = frame_settings.find(ANDROID_TONEMAP_CURVE_GREEN).count/2;
        if (tonemapCurves.tonemap_points_cnt > CAM_MAX_TONEMAP_CURVE_SIZE) {
            ALOGE("%s: Fatal: tonemap_points_cnt %d exceeds max value of %d",
                    __func__, tonemapCurves.tonemap_points_cnt,
                    CAM_MAX_TONEMAP_CURVE_SIZE);
            tonemapCurves.tonemap_points_cnt = CAM_MAX_TONEMAP_CURVE_SIZE;
        }

        /* ch0 = G*/
        size_t point = 0;
        cam_tonemap_curve_t tonemapCurveGreen;
        for (size_t i = 0; i < tonemapCurves.tonemap_points_cnt; i++) {
            for (size_t j = 0; j < 2; j++) {
               tonemapCurveGreen.tonemap_points[i][j] =
                  frame_settings.find(ANDROID_TONEMAP_CURVE_GREEN).data.f[point];
               point++;
            }
        }
        tonemapCurves.curves[0] = tonemapCurveGreen;

        /* ch 1 = B */
        point = 0;
        cam_tonemap_curve_t tonemapCurveBlue;
        for (size_t i = 0; i < tonemapCurves.tonemap_points_cnt; i++) {
            for (size_t j = 0; j < 2; j++) {
               tonemapCurveBlue.tonemap_points[i][j] =
                  frame_settings.find(ANDROID_TONEMAP_CURVE_BLUE).data.f[point];
               point++;
            }
        }
        tonemapCurves.curves[1] = tonemapCurveBlue;

        /* ch 2 = R */
        point = 0;
        cam_tonemap_curve_t tonemapCurveRed;
        for (size_t i = 0; i < tonemapCurves.tonemap_points_cnt; i++) {
            for (size_t j = 0; j < 2; j++) {
               tonemapCurveRed.tonemap_points[i][j] =
                  frame_settings.find(ANDROID_TONEMAP_CURVE_RED).data.f[point];
               point++;
            }
        }
        tonemapCurves.curves[2] = tonemapCurveRed;

        if (ADD_SET_PARAM_ENTRY_TO_BATCH(hal_metadata, CAM_INTF_META_TONEMAP_CURVES,
                tonemapCurves)) {
            rc = BAD_VALUE;
        }
    }

    if (frame_settings.exists(ANDROID_CONTROL_CAPTURE_INTENT)) {
        uint8_t captureIntent = frame_settings.find(ANDROID_CONTROL_CAPTURE_INTENT).data.u8[0];
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_META_CAPTURE_INTENT,
                captureIntent)) {
            rc = BAD_VALUE;
        }
    }

    if (frame_settings.exists(ANDROID_BLACK_LEVEL_LOCK)) {
        uint8_t blackLevelLock = frame_settings.find(ANDROID_BLACK_LEVEL_LOCK).data.u8[0];
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_META_BLACK_LEVEL_LOCK,
                blackLevelLock)) {
            rc = BAD_VALUE;
        }
    }

    if (frame_settings.exists(ANDROID_STATISTICS_LENS_SHADING_MAP_MODE)) {
        uint8_t lensShadingMapMode =
                frame_settings.find(ANDROID_STATISTICS_LENS_SHADING_MAP_MODE).data.u8[0];
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_META_LENS_SHADING_MAP_MODE,
                lensShadingMapMode)) {
            rc = BAD_VALUE;
        }
    }

    if (frame_settings.exists(ANDROID_CONTROL_AE_REGIONS)) {
        cam_area_t roi;
        bool reset = true;
        convertFromRegions(roi, request->settings, ANDROID_CONTROL_AE_REGIONS);

        // Map coordinate system from active array to sensor output.
        mCropRegionMapper.toSensor(roi.rect.left, roi.rect.top, roi.rect.width,
                roi.rect.height);

        if (scalerCropSet) {
            reset = resetIfNeededROI(&roi, &scalerCropRegion);
        }
        if (reset && ADD_SET_PARAM_ENTRY_TO_BATCH(hal_metadata, CAM_INTF_META_AEC_ROI, roi)) {
            rc = BAD_VALUE;
        }
    }

    if (frame_settings.exists(ANDROID_CONTROL_AF_REGIONS)) {
        cam_area_t roi;
        bool reset = true;
        convertFromRegions(roi, request->settings, ANDROID_CONTROL_AF_REGIONS);

        // Map coordinate system from active array to sensor output.
        mCropRegionMapper.toSensor(roi.rect.left, roi.rect.top, roi.rect.width,
                roi.rect.height);

        if (scalerCropSet) {
            reset = resetIfNeededROI(&roi, &scalerCropRegion);
        }
        if (reset && ADD_SET_PARAM_ENTRY_TO_BATCH(hal_metadata, CAM_INTF_META_AF_ROI, roi)) {
            rc = BAD_VALUE;
        }
    }

    // CDS
    if (frame_settings.exists(QCAMERA3_CDS_MODE)) {
        int32_t *cds = frame_settings.find(QCAMERA3_CDS_MODE).data.i32;
        if ((CAM_CDS_MODE_MAX <= (*cds)) || (0 > (*cds))) {
            ALOGE("%s: Invalid CDS mode %d!", __func__, *cds);
        } else {
            if (ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_PARM_CDS_MODE, *cds)) {
                rc = BAD_VALUE;
            }
        }
    }

    // TNR
    if (frame_settings.exists(QCAMERA3_TEMPORAL_DENOISE_ENABLE) &&
        frame_settings.exists(QCAMERA3_TEMPORAL_DENOISE_PROCESS_TYPE)) {
        cam_denoise_param_t tnr;
        tnr.denoise_enable = frame_settings.find(QCAMERA3_TEMPORAL_DENOISE_ENABLE).data.u8[0];
        tnr.process_plates =
            (cam_denoise_process_type_t)frame_settings.find(
            QCAMERA3_TEMPORAL_DENOISE_PROCESS_TYPE).data.i32[0];

        if (ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters, CAM_INTF_PARM_TEMPORAL_DENOISE, tnr)) {
            rc = BAD_VALUE;
        }
    }

    if (frame_settings.exists(ANDROID_SENSOR_TEST_PATTERN_MODE)) {
        int32_t fwk_testPatternMode =
                frame_settings.find(ANDROID_SENSOR_TEST_PATTERN_MODE).data.i32[0];
        int testPatternMode = lookupHalName(TEST_PATTERN_MAP,
                METADATA_MAP_SIZE(TEST_PATTERN_MAP), fwk_testPatternMode);

        if (NAME_NOT_FOUND != testPatternMode) {
            cam_test_pattern_data_t testPatternData;
            memset(&testPatternData, 0, sizeof(testPatternData));
            testPatternData.mode = (cam_test_pattern_mode_t)testPatternMode;
            if (testPatternMode == CAM_TEST_PATTERN_SOLID_COLOR &&
                    frame_settings.exists(ANDROID_SENSOR_TEST_PATTERN_DATA)) {
                int32_t *fwk_testPatternData =
                        frame_settings.find(ANDROID_SENSOR_TEST_PATTERN_DATA).data.i32;
                testPatternData.r = fwk_testPatternData[0];
                testPatternData.b = fwk_testPatternData[3];
                switch (gCamCapability[mCameraId]->color_arrangement) {
                    case CAM_FILTER_ARRANGEMENT_RGGB:
                    case CAM_FILTER_ARRANGEMENT_GRBG:
                        testPatternData.gr = fwk_testPatternData[1];
                        testPatternData.gb = fwk_testPatternData[2];
                        break;
                    case CAM_FILTER_ARRANGEMENT_GBRG:
                    case CAM_FILTER_ARRANGEMENT_BGGR:
                        testPatternData.gr = fwk_testPatternData[2];
                        testPatternData.gb = fwk_testPatternData[1];
                        break;
                    default:
                        ALOGE("%s: color arrangement %d is not supported", __func__,
                                gCamCapability[mCameraId]->color_arrangement);
                        break;
                }
            }
            if (ADD_SET_PARAM_ENTRY_TO_BATCH(hal_metadata, CAM_INTF_META_TEST_PATTERN_DATA,
                    testPatternData)) {
                rc = BAD_VALUE;
            }
        } else {
            ALOGE("%s: Invalid framework sensor test pattern mode %d", __func__,
                    fwk_testPatternMode);
        }
    }

    if (frame_settings.exists(ANDROID_JPEG_GPS_COORDINATES)) {
        size_t count = 0;
        camera_metadata_entry_t gps_coords = frame_settings.find(ANDROID_JPEG_GPS_COORDINATES);
        ADD_SET_PARAM_ARRAY_TO_BATCH(hal_metadata, CAM_INTF_META_JPEG_GPS_COORDINATES,
                gps_coords.data.d, gps_coords.count, count);
        if (gps_coords.count != count) {
            rc = BAD_VALUE;
        }
    }

    if (frame_settings.exists(ANDROID_JPEG_GPS_PROCESSING_METHOD)) {
        char gps_methods[GPS_PROCESSING_METHOD_SIZE];
        size_t count = 0;
        const char *gps_methods_src = (const char *)
                frame_settings.find(ANDROID_JPEG_GPS_PROCESSING_METHOD).data.u8;
        memset(gps_methods, '\0', sizeof(gps_methods));
        strlcpy(gps_methods, gps_methods_src, sizeof(gps_methods));
        ADD_SET_PARAM_ARRAY_TO_BATCH(hal_metadata, CAM_INTF_META_JPEG_GPS_PROC_METHODS,
                gps_methods, GPS_PROCESSING_METHOD_SIZE, count);
        if (GPS_PROCESSING_METHOD_SIZE != count) {
            rc = BAD_VALUE;
        }
    }

    if (frame_settings.exists(ANDROID_JPEG_GPS_TIMESTAMP)) {
        int64_t gps_timestamp = frame_settings.find(ANDROID_JPEG_GPS_TIMESTAMP).data.i64[0];
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(hal_metadata, CAM_INTF_META_JPEG_GPS_TIMESTAMP,
                gps_timestamp)) {
            rc = BAD_VALUE;
        }
    }

    if (frame_settings.exists(ANDROID_JPEG_ORIENTATION)) {
        int32_t orientation = frame_settings.find(ANDROID_JPEG_ORIENTATION).data.i32[0];
        cam_rotation_info_t rotation_info;
        if (orientation == 0) {
           rotation_info.rotation = ROTATE_0;
        } else if (orientation == 90) {
           rotation_info.rotation = ROTATE_90;
        } else if (orientation == 180) {
           rotation_info.rotation = ROTATE_180;
        } else if (orientation == 270) {
           rotation_info.rotation = ROTATE_270;
        }
        rotation_info.streamId = snapshotStreamId;
        ADD_SET_PARAM_ENTRY_TO_BATCH(hal_metadata, CAM_INTF_META_JPEG_ORIENTATION, orientation);
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(hal_metadata, CAM_INTF_PARM_ROTATION, rotation_info)) {
            rc = BAD_VALUE;
        }
    }

    if (frame_settings.exists(ANDROID_JPEG_QUALITY)) {
        uint32_t quality = (uint32_t) frame_settings.find(ANDROID_JPEG_QUALITY).data.u8[0];
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(hal_metadata, CAM_INTF_META_JPEG_QUALITY, quality)) {
            rc = BAD_VALUE;
        }
    }

    if (frame_settings.exists(ANDROID_JPEG_THUMBNAIL_QUALITY)) {
        uint32_t thumb_quality = (uint32_t)
                frame_settings.find(ANDROID_JPEG_THUMBNAIL_QUALITY).data.u8[0];
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(hal_metadata, CAM_INTF_META_JPEG_THUMB_QUALITY,
                thumb_quality)) {
            rc = BAD_VALUE;
        }
    }

    if (frame_settings.exists(ANDROID_JPEG_THUMBNAIL_SIZE)) {
        cam_dimension_t dim;
        dim.width = frame_settings.find(ANDROID_JPEG_THUMBNAIL_SIZE).data.i32[0];
        dim.height = frame_settings.find(ANDROID_JPEG_THUMBNAIL_SIZE).data.i32[1];
        if (ADD_SET_PARAM_ENTRY_TO_BATCH(hal_metadata, CAM_INTF_META_JPEG_THUMB_SIZE, dim)) {
            rc = BAD_VALUE;
        }
    }

    // Internal metadata
    if (frame_settings.exists(QCAMERA3_PRIVATEDATA_REPROCESS)) {
        size_t count = 0;
        camera_metadata_entry_t privatedata = frame_settings.find(QCAMERA3_PRIVATEDATA_REPROCESS);
        ADD_SET_PARAM_ARRAY_TO_BATCH(hal_metadata, CAM_INTF_META_PRIVATE_DATA,
                privatedata.data.i32, privatedata.count, count);
        if (privatedata.count != count) {
            rc = BAD_VALUE;
        }
    }

    // EV step
    if (ADD_SET_PARAM_ENTRY_TO_BATCH(hal_metadata, CAM_INTF_PARM_EV_STEP,
            gCamCapability[mCameraId]->exp_compensation_step)) {
        rc = BAD_VALUE;
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : captureResultCb
 *
 * DESCRIPTION: Callback handler for all channels (streams, as well as metadata)
 *
 * PARAMETERS :
 *   @frame  : frame information from mm-camera-interface
 *   @buffer : actual gralloc buffer to be returned to frameworks. NULL if metadata.
 *   @userdata: userdata
 *
 * RETURN     : NONE
 *==========================================================================*/
void QCamera3HardwareInterface::captureResultCb(mm_camera_super_buf_t *metadata,
                camera3_stream_buffer_t *buffer,
                uint32_t frame_number, void *userdata)
{
    QCamera3HardwareInterface *hw = (QCamera3HardwareInterface *)userdata;
    if (hw == NULL) {
        ALOGE("%s: Invalid hw %p", __func__, hw);
        return;
    }

    hw->captureResultCb(metadata, buffer, frame_number);
    return;
}


/*===========================================================================
 * FUNCTION   : initialize
 *
 * DESCRIPTION: Pass framework callback pointers to HAL
 *
 * PARAMETERS :
 *
 *
 * RETURN     : Success : 0
 *              Failure: -ENODEV
 *==========================================================================*/

int QCamera3HardwareInterface::initialize(const struct camera3_device *device,
                                  const camera3_callback_ops_t *callback_ops)
{
    CDBG("%s: E", __func__);
    QCamera3HardwareInterface *hw =
        reinterpret_cast<QCamera3HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("%s: NULL camera device", __func__);
        return -ENODEV;
    }

    int rc = hw->initialize(callback_ops);
    CDBG("%s: X", __func__);
    return rc;
}

/*===========================================================================
 * FUNCTION   : configure_streams
 *
 * DESCRIPTION:
 *
 * PARAMETERS :
 *
 *
 * RETURN     : Success: 0
 *              Failure: -EINVAL (if stream configuration is invalid)
 *                       -ENODEV (fatal error)
 *==========================================================================*/

int QCamera3HardwareInterface::configure_streams(
        const struct camera3_device *device,
        camera3_stream_configuration_t *stream_list)
{
    CDBG("%s: E", __func__);
    QCamera3HardwareInterface *hw =
        reinterpret_cast<QCamera3HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("%s: NULL camera device", __func__);
        return -ENODEV;
    }
    int rc = hw->configureStreams(stream_list);
    CDBG("%s: X", __func__);
    return rc;
}

/*===========================================================================
 * FUNCTION   : construct_default_request_settings
 *
 * DESCRIPTION: Configure a settings buffer to meet the required use case
 *
 * PARAMETERS :
 *
 *
 * RETURN     : Success: Return valid metadata
 *              Failure: Return NULL
 *==========================================================================*/
const camera_metadata_t* QCamera3HardwareInterface::
    construct_default_request_settings(const struct camera3_device *device,
                                        int type)
{

    CDBG("%s: E", __func__);
    camera_metadata_t* fwk_metadata = NULL;
    QCamera3HardwareInterface *hw =
        reinterpret_cast<QCamera3HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("%s: NULL camera device", __func__);
        return NULL;
    }

    fwk_metadata = hw->translateCapabilityToMetadata(type);

    CDBG("%s: X", __func__);
    return fwk_metadata;
}

/*===========================================================================
 * FUNCTION   : process_capture_request
 *
 * DESCRIPTION:
 *
 * PARAMETERS :
 *
 *
 * RETURN     :
 *==========================================================================*/
int QCamera3HardwareInterface::process_capture_request(
                    const struct camera3_device *device,
                    camera3_capture_request_t *request)
{
    CDBG("%s: E", __func__);
    QCamera3HardwareInterface *hw =
        reinterpret_cast<QCamera3HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("%s: NULL camera device", __func__);
        return -EINVAL;
    }

    int rc = hw->processCaptureRequest(request);
    CDBG("%s: X", __func__);
    return rc;
}

/*===========================================================================
 * FUNCTION   : dump
 *
 * DESCRIPTION:
 *
 * PARAMETERS :
 *
 *
 * RETURN     :
 *==========================================================================*/

void QCamera3HardwareInterface::dump(
                const struct camera3_device *device, int fd)
{
    /* Log level property is read when "adb shell dumpsys media.camera" is
       called so that the log level can be controlled without restarting
       the media server */
    getLogLevel();

    CDBG("%s: E", __func__);
    QCamera3HardwareInterface *hw =
        reinterpret_cast<QCamera3HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("%s: NULL camera device", __func__);
        return;
    }

    hw->dump(fd);
    CDBG("%s: X", __func__);
    return;
}

/*===========================================================================
 * FUNCTION   : flush
 *
 * DESCRIPTION:
 *
 * PARAMETERS :
 *
 *
 * RETURN     :
 *==========================================================================*/

int QCamera3HardwareInterface::flush(
                const struct camera3_device *device)
{
    int rc;
    CDBG("%s: E", __func__);
    QCamera3HardwareInterface *hw =
        reinterpret_cast<QCamera3HardwareInterface *>(device->priv);
    if (!hw) {
        ALOGE("%s: NULL camera device", __func__);
        return -EINVAL;
    }

    rc = hw->flush();
    CDBG("%s: X", __func__);
    return rc;
}

/*===========================================================================
 * FUNCTION   : close_camera_device
 *
 * DESCRIPTION:
 *
 * PARAMETERS :
 *
 *
 * RETURN     :
 *==========================================================================*/
int QCamera3HardwareInterface::close_camera_device(struct hw_device_t* device)
{
    int ret = NO_ERROR;
    QCamera3HardwareInterface *hw =
        reinterpret_cast<QCamera3HardwareInterface *>(
            reinterpret_cast<camera3_device_t *>(device)->priv);
    if (!hw) {
        ALOGE("NULL camera device");
        return BAD_VALUE;
    }
    ALOGI("[KPI Perf] %s: E camera id %d",__func__, hw->mCameraId);
    delete hw;
    ALOGI("[KPI Perf] %s: X",__func__);
    return ret;
}

/*===========================================================================
 * FUNCTION   : getWaveletDenoiseProcessPlate
 *
 * DESCRIPTION: query wavelet denoise process plate
 *
 * PARAMETERS : None
 *
 * RETURN     : WNR prcocess plate value
 *==========================================================================*/
cam_denoise_process_type_t QCamera3HardwareInterface::getWaveletDenoiseProcessPlate()
{
    char prop[PROPERTY_VALUE_MAX];
    memset(prop, 0, sizeof(prop));
    property_get("persist.denoise.process.plates", prop, "0");
    int processPlate = atoi(prop);
    switch(processPlate) {
    case 0:
        return CAM_WAVELET_DENOISE_YCBCR_PLANE;
    case 1:
        return CAM_WAVELET_DENOISE_CBCR_ONLY;
    case 2:
        return CAM_WAVELET_DENOISE_STREAMLINE_YCBCR;
    case 3:
        return CAM_WAVELET_DENOISE_STREAMLINED_CBCR;
    default:
        return CAM_WAVELET_DENOISE_STREAMLINE_YCBCR;
    }
}


/*===========================================================================
 * FUNCTION   : getTemporalDenoiseProcessPlate
 *
 * DESCRIPTION: query temporal denoise process plate
 *
 * PARAMETERS : None
 *
 * RETURN     : TNR prcocess plate value
 *==========================================================================*/
cam_denoise_process_type_t QCamera3HardwareInterface::getTemporalDenoiseProcessPlate()
{
    char prop[PROPERTY_VALUE_MAX];
    memset(prop, 0, sizeof(prop));
    property_get("persist.tnr.process.plates", prop, "0");
    int processPlate = atoi(prop);
    switch(processPlate) {
    case 0:
        return CAM_WAVELET_DENOISE_YCBCR_PLANE;
    case 1:
        return CAM_WAVELET_DENOISE_CBCR_ONLY;
    case 2:
        return CAM_WAVELET_DENOISE_STREAMLINE_YCBCR;
    case 3:
        return CAM_WAVELET_DENOISE_STREAMLINED_CBCR;
    default:
        return CAM_WAVELET_DENOISE_STREAMLINE_YCBCR;
    }
}


/*===========================================================================
 * FUNCTION   : extractSceneMode
 *
 * DESCRIPTION: Extract scene mode from frameworks set metadata
 *
 * PARAMETERS :
 *      @frame_settings: CameraMetadata reference
 *      @metaMode: ANDROID_CONTORL_MODE
 *      @hal_metadata: hal metadata structure
 *
 * RETURN     : None
 *==========================================================================*/
int32_t QCamera3HardwareInterface::extractSceneMode(
        const CameraMetadata &frame_settings, uint8_t metaMode,
        metadata_buffer_t *hal_metadata)
{
    int32_t sceneMode, hfrMode;
    int32_t rc = NO_ERROR;

    if (metaMode == ANDROID_CONTROL_MODE_USE_SCENE_MODE) {
        camera_metadata_ro_entry entry = frame_settings.find(ANDROID_CONTROL_SCENE_MODE);
        if (0 == entry.count)
            return rc;

        uint8_t fwk_sceneMode = entry.data.u8[0];

        if (fwk_sceneMode != ANDROID_CONTROL_SCENE_MODE_HIGH_SPEED_VIDEO) {
            sceneMode = lookupHalName(SCENE_MODES_MAP,
                    sizeof(SCENE_MODES_MAP)/sizeof(SCENE_MODES_MAP[0]),
                    fwk_sceneMode);
            hfrMode = CAM_HFR_MODE_OFF;
        } else {
            if (!frame_settings.exists(ANDROID_CONTROL_AE_TARGET_FPS_RANGE)) {
                CDBG("%s: No valid TARGET_FPS_RANGE in setting.", __func__);
                return rc;
            }
            int32_t min_fps =
                frame_settings.find(ANDROID_CONTROL_AE_TARGET_FPS_RANGE).data.i32[0];
            int32_t max_fps =
                frame_settings.find(ANDROID_CONTROL_AE_TARGET_FPS_RANGE).data.i32[1];
            if (min_fps != max_fps) {
                ALOGE("%s: for HIGH_SPEED_VIDEO, min_fps and max_fps should be same",
                        __func__);
                return BAD_VALUE;
            }
            switch (min_fps) {
            case 60:
                hfrMode = CAM_HFR_MODE_60FPS;
                break;
            case 90:
                hfrMode = CAM_HFR_MODE_90FPS;
                break;
            case 120:
                hfrMode = CAM_HFR_MODE_120FPS;
                break;
            case 150:
                hfrMode = CAM_HFR_MODE_150FPS;
                break;
            case 180:
                hfrMode = CAM_HFR_MODE_180FPS;
                break;
            case 210:
                hfrMode = CAM_HFR_MODE_210FPS;
                break;
            case 240:
                hfrMode = CAM_HFR_MODE_240FPS;
                break;
            case 480:
                hfrMode = CAM_HFR_MODE_480FPS;
                break;
            default:
                hfrMode = CAM_HFR_MODE_OFF;
                break;
            }
            sceneMode = CAM_SCENE_MODE_OFF;
        }
    } else {
       sceneMode = CAM_SCENE_MODE_OFF;
       hfrMode = CAM_HFR_MODE_OFF;
    }

    if (ADD_SET_PARAM_ENTRY_TO_BATCH(hal_metadata, CAM_INTF_PARM_HFR,
            hfrMode)) {
        rc = BAD_VALUE;
    }
    if (ADD_SET_PARAM_ENTRY_TO_BATCH(hal_metadata, CAM_INTF_PARM_BESTSHOT_MODE,
            sceneMode)) {
        rc = BAD_VALUE;
    }
    CDBG("%s: sceneMode: %d hfrMode: %d", __func__, sceneMode, hfrMode);

    return rc;
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
bool QCamera3HardwareInterface::needRotationReprocess()
{
    if ((gCamCapability[mCameraId]->qcom_supported_feature_mask & CAM_QCOM_FEATURE_ROTATION) > 0) {
        // current rotation is not zero, and pp has the capability to process rotation
        CDBG_HIGH("%s: need do reprocess for rotation", __func__);
        return true;
    }

    return false;
}

/*===========================================================================
 * FUNCTION   : needReprocess
 *
 * DESCRIPTION: if reprocess in needed
 *
 * PARAMETERS : none
 *
 * RETURN     : true: needed
 *              false: no need
 *==========================================================================*/
bool QCamera3HardwareInterface::needReprocess(uint32_t postprocess_mask)
{
    if (gCamCapability[mCameraId]->qcom_supported_feature_mask > 0) {
        // TODO: add for ZSL HDR later
        // pp module has min requirement for zsl reprocess, or WNR in ZSL mode
        if(postprocess_mask == CAM_QCOM_FEATURE_NONE){
            CDBG_HIGH("%s: need do reprocess for ZSL WNR or min PP reprocess", __func__);
            return true;
        } else {
            CDBG_HIGH("%s: already post processed frame", __func__);
            return false;
        }
    }
    return needRotationReprocess();
}

/*===========================================================================
 * FUNCTION   : needJpegRotation
 *
 * DESCRIPTION: if rotation from jpeg is needed
 *
 * PARAMETERS : none
 *
 * RETURN     : true: needed
 *              false: no need
 *==========================================================================*/
bool QCamera3HardwareInterface::needJpegRotation()
{
   /*If the pp does not have the ability to do rotation, enable jpeg rotation*/
    if (!(gCamCapability[mCameraId]->qcom_supported_feature_mask & CAM_QCOM_FEATURE_ROTATION)) {
       CDBG("%s: Need Jpeg to do the rotation", __func__);
       return true;
    }
    return false;
}

/*===========================================================================
 * FUNCTION   : addOfflineReprocChannel
 *
 * DESCRIPTION: add a reprocess channel that will do reprocess on frames
 *              coming from input channel
 *
 * PARAMETERS :
 *   @config  : reprocess configuration
 *
 *
 * RETURN     : Ptr to the newly created channel obj. NULL if failed.
 *==========================================================================*/
QCamera3ReprocessChannel *QCamera3HardwareInterface::addOfflineReprocChannel(
        const reprocess_config_t &config, QCamera3PicChannel *picChHandle,
        metadata_buffer_t *metadata)
{
    int32_t rc = NO_ERROR;
    QCamera3ReprocessChannel *pChannel = NULL;

    pChannel = new QCamera3ReprocessChannel(mCameraHandle->camera_handle,
            mCameraHandle->ops, NULL, config.padding, CAM_QCOM_FEATURE_NONE, this, picChHandle);
    if (NULL == pChannel) {
        ALOGE("%s: no mem for reprocess channel", __func__);
        return NULL;
    }

    rc = pChannel->initialize(IS_TYPE_NONE);
    if (rc != NO_ERROR) {
        ALOGE("%s: init reprocess channel failed, ret = %d", __func__, rc);
        delete pChannel;
        return NULL;
    }

    // pp feature config
    cam_pp_feature_config_t pp_config;
    memset(&pp_config, 0, sizeof(cam_pp_feature_config_t));

    pp_config.feature_mask |= CAM_QCOM_FEATURE_PP_SUPERSET_HAL3;
    if (gCamCapability[mCameraId]->qcom_supported_feature_mask
            & CAM_QCOM_FEATURE_DSDN) {
        //Use CPP CDS incase h/w supports it.
        pp_config.feature_mask &= ~CAM_QCOM_FEATURE_CDS;
        pp_config.feature_mask |= CAM_QCOM_FEATURE_DSDN;
    }

    rc = pChannel->addReprocStreamsFromSource(pp_config,
            config,
            IS_TYPE_NONE,
            mMetadataChannel);

    if (rc != NO_ERROR) {
        delete pChannel;
        return NULL;
    }
    return pChannel;
}

/*===========================================================================
 * FUNCTION   : getMobicatMask
 *
 * DESCRIPTION: returns mobicat mask
 *
 * PARAMETERS : none
 *
 * RETURN     : mobicat mask
 *
 *==========================================================================*/
uint8_t QCamera3HardwareInterface::getMobicatMask()
{
    return m_MobicatMask;
}

/*===========================================================================
 * FUNCTION   : setMobicat
 *
 * DESCRIPTION: set Mobicat on/off.
 *
 * PARAMETERS :
 *   @params  : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3HardwareInterface::setMobicat()
{
    char value [PROPERTY_VALUE_MAX];
    property_get("persist.camera.mobicat", value, "0");
    int32_t ret = NO_ERROR;
    uint8_t enableMobi = (uint8_t)atoi(value);

    if (enableMobi) {
        tune_cmd_t tune_cmd;
        tune_cmd.type = SET_RELOAD_CHROMATIX;
        tune_cmd.module = MODULE_ALL;
        tune_cmd.value = TRUE;
        ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters,
                CAM_INTF_PARM_SET_VFE_COMMAND,
                tune_cmd);

        ADD_SET_PARAM_ENTRY_TO_BATCH(mParameters,
                CAM_INTF_PARM_SET_PP_COMMAND,
                tune_cmd);
    }
    m_MobicatMask = enableMobi;

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
void QCamera3HardwareInterface::getLogLevel()
{
    char prop[PROPERTY_VALUE_MAX];
    uint32_t globalLogLevel = 0;

    property_get("persist.camera.hal.debug", prop, "0");
    int val = atoi(prop);
    if (0 <= val) {
        gCamHal3LogLevel = (uint32_t)val;
    }

    property_get("persist.camera.kpi.debug", prop, "1");
    gKpiDebugLevel = atoi(prop);

    property_get("persist.camera.global.debug", prop, "0");
    val = atoi(prop);
    if (0 <= val) {
        globalLogLevel = (uint32_t)val;
    }

    /* Highest log level among hal.logs and global.logs is selected */
    if (gCamHal3LogLevel < globalLogLevel)
        gCamHal3LogLevel = globalLogLevel;

    return;
}

}; //end namespace qcamera
