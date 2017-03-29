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

#ifndef __QCAMERA3HARDWAREINTERFACE_H__
#define __QCAMERA3HARDWAREINTERFACE_H__

#include <pthread.h>
#include <utils/List.h>
#include <utils/KeyedVector.h>
#include <hardware/camera3.h>
#include <camera/CameraMetadata.h>
#include "QCameraTrace.h"
#include "QCamera3HALHeader.h"
#include "QCamera3Channel.h"
#include "QCamera3CropRegionMapper.h"

#include <hardware/power.h>

extern "C" {
#include <mm_camera_interface.h>
#include <mm_jpeg_interface.h>
}
#ifdef CDBG
#undef CDBG
#endif //#ifdef CDBG
#define CDBG(fmt, args...) ALOGD_IF(gCamHal3LogLevel >= 2, fmt, ##args)

#ifdef CDBG_HIGH
#undef CDBG_HIGH
#endif //#ifdef CDBG_HIGH
#define CDBG_HIGH(fmt, args...) ALOGD_IF(gCamHal3LogLevel >= 1, fmt, ##args)

using namespace android;

namespace qcamera {

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/* Time related macros */
#define NSEC_PER_SEC 1000000000LLU
#define NSEC_PER_USEC 1000LLU
#define NSEC_PER_33MSEC 33000000LLU

typedef enum {
    SET_ENABLE,
    SET_CONTROLENABLE,
    SET_RELOAD_CHROMATIX,
    SET_STATUS,
} optype_t;

#define MODULE_ALL 0


extern volatile uint32_t gCamHal3LogLevel;

class QCamera3MetadataChannel;
class QCamera3PicChannel;
class QCamera3HeapMemory;
class QCamera3Exif;

typedef struct {
    camera3_stream_t *stream;
    camera3_stream_buffer_set_t buffer_set;
    stream_status_t status;
    int registered;
    QCamera3Channel *channel;
} stream_info_t;

class QCamera3HardwareInterface {
public:
    /* static variable and functions accessed by camera service */
    static camera3_device_ops_t mCameraOps;
    static int initialize(const struct camera3_device *,
                const camera3_callback_ops_t *callback_ops);
    static int configure_streams(const struct camera3_device *,
                camera3_stream_configuration_t *stream_list);
    static const camera_metadata_t* construct_default_request_settings(
                                const struct camera3_device *, int type);
    static int process_capture_request(const struct camera3_device *,
                                camera3_capture_request_t *request);

    static void dump(const struct camera3_device *, int fd);
    static int flush(const struct camera3_device *);
    static int close_camera_device(struct hw_device_t* device);

public:
    QCamera3HardwareInterface(uint32_t cameraId,
            const camera_module_callbacks_t *callbacks);
    virtual ~QCamera3HardwareInterface();
    static void camEvtHandle(uint32_t camera_handle, mm_camera_event_t *evt,
                                          void *user_data);
    int openCamera(struct hw_device_t **hw_device);
    camera_metadata_t* translateCapabilityToMetadata(int type);

    static int getCamInfo(uint32_t cameraId, struct camera_info *info);
    static int initCapabilities(uint32_t cameraId);
    static int initStaticMetadata(uint32_t cameraId);
    static void makeTable(cam_dimension_t *dimTable, size_t size,
            size_t max_size, int32_t *sizeTable);
    static void makeFPSTable(cam_fps_range_t *fpsTable, size_t size,
            size_t max_size, int32_t *fpsRangesTable);
    static void makeOverridesList(cam_scene_mode_overrides_t *overridesTable,
            size_t size, size_t max_size, uint8_t *overridesList,
            uint8_t *supported_indexes, uint32_t camera_id);
    static size_t filterJpegSizes(int32_t *jpegSizes, int32_t *processedSizes,
            size_t processedSizesCnt, size_t maxCount, cam_rect_t active_array_size,
            uint8_t downscale_factor);
    static void convertToRegions(cam_rect_t rect, int32_t* region, int weight);
    static void convertFromRegions(cam_area_t &roi, const camera_metadata_t *settings,
                                   uint32_t tag);
    static bool resetIfNeededROI(cam_area_t* roi, const cam_crop_region_t* scalerCropRegion);
    static void convertLandmarks(cam_face_detection_info_t face, int32_t* landmarks);
    static int32_t getScalarFormat(int32_t format);
    static int32_t getSensorSensitivity(int32_t iso_mode);

    double computeNoiseModelEntryS(int32_t sensitivity);
    double computeNoiseModelEntryO(int32_t sensitivity);

    static void captureResultCb(mm_camera_super_buf_t *metadata,
                camera3_stream_buffer_t *buffer, uint32_t frame_number,
                void *userdata);

    int initialize(const camera3_callback_ops_t *callback_ops);
    int configureStreams(camera3_stream_configuration_t *stream_list);
    int processCaptureRequest(camera3_capture_request_t *request);
    void dump(int fd);
    int flush();
    int flushPerf();

    int setFrameParameters(camera3_capture_request_t *request,
            cam_stream_ID_t streamID, int blob_request, uint32_t snapshotStreamId);
    int32_t setReprocParameters(camera3_capture_request_t *request,
            metadata_buffer_t *reprocParam, uint32_t snapshotStreamId);
    int translateToHalMetadata(const camera3_capture_request_t *request,
            metadata_buffer_t *parm, uint32_t snapshotStreamId);
    camera_metadata_t* translateCbUrgentMetadataToResultMetadata (
                             metadata_buffer_t *metadata);
    camera_metadata_t* translateFromHalMetadata(metadata_buffer_t *metadata,
                            nsecs_t timestamp, int32_t request_id,
                            const CameraMetadata& jpegMetadata, uint8_t pipeline_depth,
                            uint8_t capture_intent, uint8_t fwk_cacMode);
    int initParameters();
    void deinitParameters();
    QCamera3ReprocessChannel *addOfflineReprocChannel(const reprocess_config_t &config,
            QCamera3PicChannel *picChHandle, metadata_buffer_t *metadata);
    bool needRotationReprocess();
    bool needReprocess(uint32_t postprocess_mask);
    bool needJpegRotation();
    cam_denoise_process_type_t getWaveletDenoiseProcessPlate();
    cam_denoise_process_type_t getTemporalDenoiseProcessPlate();

    void captureResultCb(mm_camera_super_buf_t *metadata,
                camera3_stream_buffer_t *buffer, uint32_t frame_number);
    cam_dimension_t calcMaxJpegDim();
    bool needOnlineRotation();
    uint32_t getJpegQuality();
    QCamera3Exif *getExifData();
    mm_jpeg_exif_params_t get3AExifParams();
    uint8_t getMobicatMask();

    template <typename fwkType, typename halType> struct QCameraMap {
        fwkType fwk_name;
        halType hal_name;
    };

    typedef struct {
        const char *const desc;
        cam_cds_mode_type_t val;
    } QCameraPropMap;


private:

    int openCamera();
    int closeCamera();
    static size_t calcMaxJpegSize(uint32_t camera_id);
    cam_dimension_t getMaxRawSize(uint32_t camera_id);

    int validateCaptureRequest(camera3_capture_request_t *request);
    int validateStreamDimensions(camera3_stream_configuration_t *streamList);
    void deriveMinFrameDuration();
    int32_t handlePendingReprocResults(uint32_t frame_number);
    int64_t getMinFrameDuration(const camera3_capture_request_t *request);
    void handleMetadataWithLock(mm_camera_super_buf_t *metadata_buf);
    void handleBufferWithLock(camera3_stream_buffer_t *buffer,
            uint32_t frame_number);
    void unblockRequestIfNecessary();
    void dumpMetadataToFile(tuning_params_t &meta, uint32_t &dumpFrameCount,
            bool enabled, const char *type, uint32_t frameNumber);
    static void getLogLevel();

    void cleanAndSortStreamInfo();
    void extractJpegMetadata(CameraMetadata& jpegMetadata,
            const camera3_capture_request_t *request);

    bool isSupportChannelNeeded(camera3_stream_configuration_t *streamList);
    int32_t setMobicat();

    int32_t getSensorOutputSize(cam_dimension_t &sensor_dim, CameraMetadata *frame_settings = NULL);
    int32_t setHalFpsRange(const CameraMetadata &settings,
            metadata_buffer_t *hal_metadata);
    int32_t extractSceneMode(const CameraMetadata &frame_settings, uint8_t metaMode,
            metadata_buffer_t *hal_metadata);

    void updatePowerHint(bool bWasVideo, bool bIsVideo);

    camera3_device_t   mCameraDevice;
    uint32_t           mCameraId;
    mm_camera_vtbl_t  *mCameraHandle;
    bool               mCameraOpened;
    bool               mCameraInitialized;
    camera_metadata_t *mDefaultMetadata[CAMERA3_TEMPLATE_COUNT];
    const camera3_callback_ops_t *mCallbackOps;

    camera3_stream_t *mInputStream;
    QCamera3MetadataChannel *mMetadataChannel;
    QCamera3PicChannel *mPictureChannel;
    QCamera3RawChannel *mRawChannel;
    QCamera3SupportChannel *mSupportChannel;
    QCamera3SupportChannel *mAnalysisChannel;
    QCamera3RawDumpChannel *mRawDumpChannel;

    void saveExifParams(metadata_buffer_t *metadata);
    mm_jpeg_exif_params_t mExifParams;

     //First request yet to be processed after configureStreams
    bool mFirstRequest;
    bool mFirstConfiguration;
    bool mFlush;
    bool mFlushPerf;
    bool mEnableRawDump;
    QCamera3HeapMemory *mParamHeap;
    metadata_buffer_t* mParameters;
    metadata_buffer_t* mPrevParameters;
    bool m_bIsVideo;
    bool m_bIs4KVideo;
    bool m_bEisSupportedSize;
    bool m_bEisEnable;
    uint8_t m_MobicatMask;
    uint8_t m_bTnrEnabled;
    uint8_t mSupportedFaceDetectMode;

    /* Data structure to store pending request */
    typedef struct {
        camera3_stream_t *stream;
        camera3_stream_buffer_t *buffer;
    } RequestedBufferInfo;
    typedef struct {
        uint32_t frame_number;
        uint32_t num_buffers;
        int32_t request_id;
        List<RequestedBufferInfo> buffers;
        int blob_request;
        uint8_t bUrgentReceived;
        nsecs_t timestamp;
        camera3_stream_buffer_t *input_buffer;
        const camera_metadata_t *settings;
        CameraMetadata jpegMetadata;
        uint8_t pipeline_depth;
        uint32_t partial_result_cnt;
        uint8_t capture_intent;
        uint8_t fwkCacMode;
    } PendingRequestInfo;
    typedef struct {
        uint32_t frame_number;
        uint32_t stream_ID;
    } PendingFrameDropInfo;

    // Store the Pending buffers for Flushing
    typedef struct {
        // Frame number pertaining to the buffer
        uint32_t frame_number;
        camera3_stream_t *stream;
        // Buffer handle
        buffer_handle_t *buffer;
    } PendingBufferInfo;

    typedef struct {
        // Total number of buffer requests pending
        uint32_t num_buffers;
        // List of pending buffers
        List<PendingBufferInfo> mPendingBufferList;
    } PendingBuffersMap;

    typedef struct {
        camera3_notify_msg_t notify_msg;
        camera3_stream_buffer_t buffer;
        uint32_t frame_number;
    } PendingReprocessResult;

    typedef KeyedVector<uint32_t, Vector<PendingBufferInfo> > FlushMap;

    List<PendingReprocessResult> mPendingReprocessResultList;
    List<PendingRequestInfo> mPendingRequestsList;
    List<PendingFrameDropInfo> mPendingFrameDropList;
    PendingBuffersMap mPendingBuffersMap;
    pthread_cond_t mRequestCond;
    int mPendingRequest;
    bool mWokenUpByDaemon;
    int32_t mCurrentRequestId;
    cam_stream_size_info_t mStreamConfigInfo;

    //mutex for serialized access to camera3_device_ops_t functions
    pthread_mutex_t mMutex;

    //condition used to signal flush after buffers have returned
    pthread_cond_t mBuffersCond;

    List<stream_info_t*> mStreamInfo;

    int64_t mMinProcessedFrameDuration;
    int64_t mMinJpegFrameDuration;
    int64_t mMinRawFrameDuration;
    power_module_t *m_pPowerModule;   // power module

    uint32_t mMetaFrameCount;
    bool    mUpdateDebugLevel;
    const camera_module_callbacks_t *mCallbacks;

    uint8_t mCaptureIntent;
    uint8_t mCacMode;
    metadata_buffer_t mRreprocMeta; //scratch meta buffer

    /* sensor output size with current stream configuration */
    QCamera3CropRegionMapper mCropRegionMapper;

    static const QCameraMap<camera_metadata_enum_android_control_effect_mode_t,
            cam_effect_mode_type> EFFECT_MODES_MAP[];
    static const QCameraMap<camera_metadata_enum_android_control_awb_mode_t,
            cam_wb_mode_type> WHITE_BALANCE_MODES_MAP[];
    static const QCameraMap<camera_metadata_enum_android_control_scene_mode_t,
            cam_scene_mode_type> SCENE_MODES_MAP[];
    static const QCameraMap<camera_metadata_enum_android_control_af_mode_t,
            cam_focus_mode_type> FOCUS_MODES_MAP[];
    static const QCameraMap<camera_metadata_enum_android_color_correction_aberration_mode_t,
            cam_aberration_mode_t> COLOR_ABERRATION_MAP[];
    static const QCameraMap<camera_metadata_enum_android_control_ae_antibanding_mode_t,
            cam_antibanding_mode_type> ANTIBANDING_MODES_MAP[];
    static const QCameraMap<camera_metadata_enum_android_lens_state_t,
            cam_af_lens_state_t> LENS_STATE_MAP[];
    static const QCameraMap<camera_metadata_enum_android_control_ae_mode_t,
            cam_flash_mode_t> AE_FLASH_MODE_MAP[];
    static const QCameraMap<camera_metadata_enum_android_flash_mode_t,
            cam_flash_mode_t> FLASH_MODES_MAP[];
    static const QCameraMap<camera_metadata_enum_android_statistics_face_detect_mode_t,
            cam_face_detect_mode_t> FACEDETECT_MODES_MAP[];
    static const QCameraMap<camera_metadata_enum_android_lens_info_focus_distance_calibration_t,
            cam_focus_calibration_t> FOCUS_CALIBRATION_MAP[];
    static const QCameraMap<camera_metadata_enum_android_sensor_test_pattern_mode_t,
            cam_test_pattern_mode_t> TEST_PATTERN_MAP[];
    static const QCameraMap<camera_metadata_enum_android_sensor_reference_illuminant1_t,
            cam_illuminat_t> REFERENCE_ILLUMINANT_MAP[];

    static const QCameraPropMap CDS_MAP[];

    //GPU library to read buffer padding details.
    void *lib_surface_utils;
    int (*LINK_get_surface_pixel_alignment)();
    uint32_t mSurfaceStridePadding;
    cam_fps_range_t mFpsRange;
    //The offset between BOOTTIME and MONOTONIC timestamps
    nsecs_t mBootToMonoTimestampOffset;
};

}; // namespace qcamera

#endif /* __QCAMERA2HARDWAREINTERFACE_H__ */
