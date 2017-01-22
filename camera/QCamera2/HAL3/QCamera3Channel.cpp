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

#define LOG_TAG "QCamera3Channel"
//#define LOG_NDEBUG 0
#include <fcntl.h>
#include <stdlib.h>
#include <cstdlib>
#include <stdio.h>
#include <string.h>
#include <hardware/camera3.h>
#include <system/camera_metadata.h>
#include <gralloc_priv.h>
#include <utils/Log.h>
#include <utils/Errors.h>
#include <cutils/properties.h>
#include "QCamera3Channel.h"
#include "QCamera3HWI.h"

using namespace android;


namespace qcamera {
static const char ExifAsciiPrefix[] =
    { 0x41, 0x53, 0x43, 0x49, 0x49, 0x0, 0x0, 0x0 };          // "ASCII\0\0\0"
static const char ExifUndefinedPrefix[] =
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };   // "\0\0\0\0\0\0\0\0"

#define EXIF_ASCII_PREFIX_SIZE           8   //(sizeof(ExifAsciiPrefix))
#define FOCAL_LENGTH_DECIMAL_PRECISION   100

/*===========================================================================
 * FUNCTION   : QCamera3Channel
 *
 * DESCRIPTION: constrcutor of QCamera3Channel
 *
 * PARAMETERS :
 *   @cam_handle : camera handle
 *   @cam_ops    : ptr to camera ops table
 *
 * RETURN     : none
 *==========================================================================*/
QCamera3Channel::QCamera3Channel(uint32_t cam_handle,
                               mm_camera_ops_t *cam_ops,
                               channel_cb_routine cb_routine,
                               cam_padding_info_t *paddingInfo,
                               uint32_t postprocess_mask,
                               void *userData, uint32_t numBuffers)
{
    m_camHandle = cam_handle;
    m_camOps = cam_ops;
    m_bIsActive = false;

    m_handle = 0;
    m_numStreams = 0;
    memset(mStreams, 0, sizeof(mStreams));
    mUserData = userData;

    mStreamInfoBuf = NULL;
    mChannelCB = cb_routine;
    mPaddingInfo = *paddingInfo;

    mPostProcMask = postprocess_mask;

    mIsType = IS_TYPE_NONE;
    mNumBuffers = numBuffers;
    dumpFrmCnt = 0;
}

/*===========================================================================
 * FUNCTION   : QCamera3Channel
 *
 * DESCRIPTION: default constrcutor of QCamera3Channel
 *
 * PARAMETERS : none
 *
 * RETURN     : none
 *==========================================================================*/
QCamera3Channel::QCamera3Channel()
{
    m_camHandle = 0;
    m_camOps = NULL;
    m_bIsActive = false;

    m_handle = 0;
    m_numStreams = 0;
    memset(mStreams, 0, sizeof(mStreams));
    mUserData = NULL;

    mStreamInfoBuf = NULL;
    mChannelCB = NULL;
    memset(&mPaddingInfo, 0, sizeof(cam_padding_info_t));

    mPostProcMask = 0;
}

/*===========================================================================
 * FUNCTION   : ~QCamera3Channel
 *
 * DESCRIPTION: destructor of QCamera3Channel
 *
 * PARAMETERS : none
 *
 * RETURN     : none
 *==========================================================================*/
QCamera3Channel::~QCamera3Channel()
{
    if (m_bIsActive)
        stop();

    for (uint32_t i = 0; i < m_numStreams; i++) {
        if (mStreams[i] != NULL) {
            delete mStreams[i];
            mStreams[i] = 0;
        }
    }
    if (m_handle) {
        m_camOps->delete_channel(m_camHandle, m_handle);
        ALOGE("%s: deleting channel %d", __func__, m_handle);
        m_handle = 0;
    }
    m_numStreams = 0;
}

/*===========================================================================
 * FUNCTION   : init
 *
 * DESCRIPTION: initialization of channel
 *
 * PARAMETERS :
 *   @attr    : channel bundle attribute setting
 *   @dataCB  : data notify callback
 *   @userData: user data ptr
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3Channel::init(mm_camera_channel_attr_t *attr,
                             mm_camera_buf_notify_t dataCB)
{
    m_handle = m_camOps->add_channel(m_camHandle,
                                      attr,
                                      dataCB,
                                      this);
    if (m_handle == 0) {
        ALOGE("%s: Add channel failed", __func__);
        return UNKNOWN_ERROR;
    }
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : addStream
 *
 * DESCRIPTION: add a stream into channel
 *
 * PARAMETERS :
 *   @streamType     : stream type
 *   @streamFormat   : stream format
 *   @streamDim      : stream dimension
 *   @minStreamBufNum : minimal buffer count for particular stream type
 *   @postprocessMask : post-proccess feature mask
 *   @isType         : type of image stabilization required on the stream
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3Channel::addStream(cam_stream_type_t streamType,
                                  cam_format_t streamFormat,
                                  cam_dimension_t streamDim,
                                  uint8_t minStreamBufNum,
                                  uint32_t postprocessMask,
                                  cam_is_type_t isType)
{
    int32_t rc = NO_ERROR;

    if (m_numStreams >= 1) {
        ALOGE("%s: Only one stream per channel supported in v3 Hal", __func__);
        return BAD_VALUE;
    }

    if (m_numStreams >= MAX_STREAM_NUM_IN_BUNDLE) {
        ALOGE("%s: stream number (%d) exceeds max limit (%d)",
              __func__, m_numStreams, MAX_STREAM_NUM_IN_BUNDLE);
        return BAD_VALUE;
    }
    QCamera3Stream *pStream = new QCamera3Stream(m_camHandle,
                                               m_handle,
                                               m_camOps,
                                               &mPaddingInfo,
                                               this);
    if (pStream == NULL) {
        ALOGE("%s: No mem for Stream", __func__);
        return NO_MEMORY;
    }

    rc = pStream->init(streamType, streamFormat, streamDim, NULL, minStreamBufNum,
                       postprocessMask, isType, streamCbRoutine, this);
    if (rc == 0) {
        mStreams[m_numStreams] = pStream;
        m_numStreams++;
    } else {
        delete pStream;
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : start
 *
 * DESCRIPTION: start channel, which will start all streams belong to this channel
 *
 * PARAMETERS :
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3Channel::start()
{
    ATRACE_CALL();
    int32_t rc = NO_ERROR;

    if (m_numStreams > 1) {
        ALOGE("%s: bundle not supported", __func__);
    } else if (m_numStreams == 0) {
        return NO_INIT;
    }

    if(m_bIsActive) {
        ALOGD("%s: Attempt to start active channel", __func__);
        return rc;
    }

    for (uint32_t i = 0; i < m_numStreams; i++) {
        if (mStreams[i] != NULL) {
            mStreams[i]->start();
        }
    }
    rc = m_camOps->start_channel(m_camHandle, m_handle);

    if (rc != NO_ERROR) {
        for (uint32_t i = 0; i < m_numStreams; i++) {
            if (mStreams[i] != NULL) {
                mStreams[i]->stop();
            }
        }
    } else {
        m_bIsActive = true;
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : stop
 *
 * DESCRIPTION: stop a channel, which will stop all streams belong to this channel
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3Channel::stop()
{
    ATRACE_CALL();
    int32_t rc = NO_ERROR;
    if(!m_bIsActive) {
        ALOGE("%s: Attempt to stop inactive channel", __func__);
        return rc;
    }

    for (uint32_t i = 0; i < m_numStreams; i++) {
        if (mStreams[i] != NULL) {
            mStreams[i]->stop();
        }
    }

    rc = m_camOps->stop_channel(m_camHandle, m_handle);

    m_bIsActive = false;
    return rc;
}

/*===========================================================================
 * FUNCTION   : bufDone
 *
 * DESCRIPTION: return a stream buf back to kernel
 *
 * PARAMETERS :
 *   @recvd_frame  : stream buf frame to be returned
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3Channel::bufDone(mm_camera_super_buf_t *recvd_frame)
{
    int32_t rc = NO_ERROR;
    for (uint32_t i = 0; i < recvd_frame->num_bufs; i++) {
         if (recvd_frame->bufs[i] != NULL) {
             for (uint32_t j = 0; j < m_numStreams; j++) {
                 if (mStreams[j] != NULL &&
                     mStreams[j]->getMyHandle() == recvd_frame->bufs[i]->stream_id) {
                     rc = mStreams[j]->bufDone(recvd_frame->bufs[i]->buf_idx);
                     break; // break loop j
                 }
             }
         }
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : getStreamTypeMask
 *
 * DESCRIPTION: Get bit mask of all stream types in this channel
 *
 * PARAMETERS : None
 *
 * RETURN     : Bit mask of all stream types in this channel
 *==========================================================================*/
uint32_t QCamera3Channel::getStreamTypeMask()
{
    uint32_t mask = 0;
    for (uint32_t i = 0; i < m_numStreams; i++) {
       mask |= (1U << mStreams[i]->getMyType());
    }
    return mask;
}

/*===========================================================================
 * FUNCTION   : getStreamID
 *
 * DESCRIPTION: Get StreamID of requested stream type
 *
 * PARAMETERS : streamMask
 *
 * RETURN     : Stream ID
 *==========================================================================*/
uint32_t QCamera3Channel::getStreamID(uint32_t streamMask)
{
    uint32_t streamID = 0;
    for (uint32_t i = 0; i < m_numStreams; i++) {
        if (streamMask == (uint32_t )(0x1 << mStreams[i]->getMyType())) {
            streamID = mStreams[i]->getMyServerID();
            break;
        }
    }
    return streamID;
}

/*===========================================================================
 * FUNCTION   : getStreamByHandle
 *
 * DESCRIPTION: return stream object by stream handle
 *
 * PARAMETERS :
 *   @streamHandle : stream handle
 *
 * RETURN     : stream object. NULL if not found
 *==========================================================================*/
QCamera3Stream *QCamera3Channel::getStreamByHandle(uint32_t streamHandle)
{
    for (uint32_t i = 0; i < m_numStreams; i++) {
        if (mStreams[i] != NULL && mStreams[i]->getMyHandle() == streamHandle) {
            return mStreams[i];
        }
    }
    return NULL;
}

/*===========================================================================
 * FUNCTION   : getStreamByIndex
 *
 * DESCRIPTION: return stream object by index
 *
 * PARAMETERS :
 *   @streamHandle : stream handle
 *
 * RETURN     : stream object. NULL if not found
 *==========================================================================*/
QCamera3Stream *QCamera3Channel::getStreamByIndex(uint32_t index)
{
    if (index < m_numStreams) {
        return mStreams[index];
    }
    return NULL;
}

/*===========================================================================
 * FUNCTION   : streamCbRoutine
 *
 * DESCRIPTION: callback routine for stream
 *
 * PARAMETERS :
 *   @streamHandle : stream handle
 *
 * RETURN     : stream object. NULL if not found
 *==========================================================================*/
void QCamera3Channel::streamCbRoutine(mm_camera_super_buf_t *super_frame,
                QCamera3Stream *stream, void *userdata)
{
    QCamera3Channel *channel = (QCamera3Channel *)userdata;
    if (channel == NULL) {
        ALOGE("%s: invalid channel pointer", __func__);
        return;
    }
    channel->streamCbRoutine(super_frame, stream);
}

/*===========================================================================
 * FUNCTION   : dumpYUV
 *
 * DESCRIPTION: function to dump the YUV data from ISP/pproc
 *
 * PARAMETERS :
 *   @frame   : frame to be dumped
 *   @dim     : dimension of the stream
 *   @offset  : offset of the data
 *   @name    : 1 if it is ISP output/pproc input, 2 if it is pproc output
 *
 * RETURN  :
 *==========================================================================*/
void QCamera3Channel::dumpYUV(mm_camera_buf_def_t *frame, cam_dimension_t dim,
        cam_frame_len_offset_t offset, uint8_t dump_type)
{
    char buf[FILENAME_MAX];
    memset(buf, 0, sizeof(buf));
    static int counter = 0;
    char prop[PROPERTY_VALUE_MAX];
    property_get("persist.camera.dumpimg", prop, "0");
    mYUVDump = (uint8_t) atoi(prop);
    if (mYUVDump & dump_type) {
        frm_num = ((mYUVDump & 0xffff0000) >> 16);
        if (frm_num == 0) {
            frm_num = 10;
        }
        if (frm_num > 256) {
            frm_num = 256;
        }
        skip_mode = ((mYUVDump & 0x0000ff00) >> 8);
        if (skip_mode == 0) {
            skip_mode = 1;
        }
        if (mDumpSkipCnt == 0) {
            mDumpSkipCnt = 1;
        }
        if (mDumpSkipCnt % skip_mode == 0) {
            if (dumpFrmCnt <= frm_num) {
                /* Note that the image dimension will be the unrotated stream dimension.
                * If you feel that the image would have been rotated during reprocess
                * then swap the dimensions while opening the file
                * */
                switch (dump_type) {
                    case QCAMERA_DUMP_FRM_PREVIEW:
                        snprintf(buf, sizeof(buf), QCAMERA_DUMP_FRM_LOCATION"p_%d_%d_%dx%d.yuv",
                            counter, frame->frame_idx, dim.width, dim.height);
                    break;
                    case QCAMERA_DUMP_FRM_VIDEO:
                        snprintf(buf, sizeof(buf), QCAMERA_DUMP_FRM_LOCATION"v_%d_%d_%dx%d.yuv",
                            counter, frame->frame_idx, dim.width, dim.height);
                    break;
                    case QCAMERA_DUMP_FRM_SNAPSHOT:
                        snprintf(buf, sizeof(buf), QCAMERA_DUMP_FRM_LOCATION"s_%d_%d_%dx%d.yuv",
                            counter, frame->frame_idx, dim.width, dim.height);
                    break;
                    case QCAMERA_DUMP_FRM_OFFLINE_PROC:
                        snprintf(buf, sizeof(buf), QCAMERA_DUMP_FRM_LOCATION"o_%d_%d_%dx%d.yuv",
                            counter, frame->frame_idx, dim.width, dim.height);
                    break;
                    default :
                        ALOGE("%s: dumping not enabled for stream type %d",__func__,dump_type);
                    break;
                }
                counter++;
                int file_fd = open(buf, O_RDWR | O_CREAT, 0777);
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
                    dumpFrmCnt++;
                    close(file_fd);
                } else {
                    ALOGE("%s: failed to open file to dump image", __func__);
                }
            }
        } else {
            mDumpSkipCnt++;
        }
    }
}
/*===========================================================================
 * FUNCTION   : isUBWCEnabled
 *
 * DESCRIPTION: Function to get UBWC hardware support.
 *
 * PARAMETERS : None
 *
 * RETURN     : TRUE -- UBWC format supported
 *              FALSE -- UBWC is not supported.
 *==========================================================================*/
bool QCamera3Channel::isUBWCEnabled()
{
#ifdef UBWC_PRESENT
    char value[PROPERTY_VALUE_MAX];
    int disable = false;
    bool ubwc_enabled = TRUE;

    property_get("debug.gralloc.gfx_ubwc_disable", value, "0");
    disable = atoi(value);
    if (disable) {
        ubwc_enabled = FALSE;
    }
    return ubwc_enabled;
#else
    return FALSE;
#endif
}

/*===========================================================================
 * FUNCTION   : getStreamDefaultFormat
 *
 * DESCRIPTION: return default buffer format for the stream
 *
 * PARAMETERS : type : Stream type
 *
 ** RETURN    : format for stream type
 *
 *==========================================================================*/
cam_format_t QCamera3Channel::getStreamDefaultFormat(cam_stream_type_t type)
{
    cam_format_t streamFormat;

    switch (type) {
    case CAM_STREAM_TYPE_PREVIEW:
        if (isUBWCEnabled()) {
            char prop[PROPERTY_VALUE_MAX];
            int pFormat;
            memset(prop, 0, sizeof(prop));
            property_get("persist.camera.preview.ubwc", prop, "1");
            pFormat = atoi(prop);
            if (pFormat == 1) {
                streamFormat = CAM_FORMAT_YUV_420_NV12_UBWC;
            } else {
                streamFormat = CAM_FORMAT_YUV_420_NV21;
            }
        } else {
            streamFormat = CAM_FORMAT_YUV_420_NV21;
        }
        break;
    case CAM_STREAM_TYPE_VIDEO:
        if (isUBWCEnabled()) {
            char prop[PROPERTY_VALUE_MAX];
            int pFormat;
            memset(prop, 0, sizeof(prop));
            property_get("persist.camera.video.ubwc", prop, "1");
            pFormat = atoi(prop);
            if (pFormat == 1) {
                streamFormat = CAM_FORMAT_YUV_420_NV12_UBWC;
            } else {
                streamFormat = CAM_FORMAT_YUV_420_NV12_VENUS;
            }
        } else {
#if VENUS_PRESENT
        streamFormat = CAM_FORMAT_YUV_420_NV12_VENUS;
#else
        streamFormat = CAM_FORMAT_YUV_420_NV12;
#endif
        }
        break;
    case CAM_STREAM_TYPE_SNAPSHOT:
        streamFormat = CAM_FORMAT_YUV_420_NV21;
        break;
    case CAM_STREAM_TYPE_CALLBACK:
        streamFormat = CAM_FORMAT_YUV_420_NV21;
        break;
    case CAM_STREAM_TYPE_RAW:
        streamFormat = CAM_FORMAT_BAYER_MIPI_RAW_10BPP_GBRG;
        break;
    default:
        streamFormat = CAM_FORMAT_YUV_420_NV21;
        break;
    }
    return streamFormat;
}

/*===========================================================================
 * FUNCTION   : QCamera3RegularChannel
 *
 * DESCRIPTION: constructor of QCamera3RegularChannel
 *
 * PARAMETERS :
 *   @cam_handle : camera handle
 *   @cam_ops    : ptr to camera ops table
 *   @cb_routine : callback routine to frame aggregator
 *   @stream     : camera3_stream_t structure
 *   @stream_type: Channel stream type
 *   @postprocess_mask: feature mask for postprocessing
 *   @numBuffers : number of max dequeued buffers
 *
 * RETURN     : none
 *==========================================================================*/
QCamera3RegularChannel::QCamera3RegularChannel(uint32_t cam_handle,
                    mm_camera_ops_t *cam_ops,
                    channel_cb_routine cb_routine,
                    cam_padding_info_t *paddingInfo,
                    void *userData,
                    camera3_stream_t *stream,
                    cam_stream_type_t stream_type,
                    uint32_t postprocess_mask,
                    uint32_t numBuffers) :
                        QCamera3Channel(cam_handle, cam_ops, cb_routine,
                                paddingInfo, postprocess_mask, userData,
                                numBuffers),
                        mFrameCount(0),
                        mLastFrameCount(0),
                        mLastFpsTime(0),
                        mCamera3Stream(stream),
                        mNumBufs(0),
                        mStreamType(stream_type),
                        mWidth(stream->width),
                        mHeight(stream->height)
{
    char prop[PROPERTY_VALUE_MAX];
    property_get("persist.debug.sf.showfps", prop, "0");
    mDebugFPS = (uint8_t) atoi(prop);
}

/*===========================================================================
 * FUNCTION   : QCamera3RegularChannel
 *
 * DESCRIPTION: constructor of QCamera3RegularChannel
 *
 * PARAMETERS :
 *   @cam_handle  : camera handle
 *   @cam_ops     : ptr to camera ops table
 *   @cb_routine  : callback routine to frame aggregator
 *   @padding_info: padding information for stream
 *   @userData    : pointer to hal object
 *   @stream      : camera3_stream_t structure
 *   @stream_type : Channel stream type
 *   @postprocess_mask: bit mask for postprocessing
 *   @width       : width overriding camera3_stream_t::width
 *   @height      : height overriding camera3_stream_t::height
 *   @numBuffers  : number of maximum dequeued buffers`
 *
 * RETURN     : none
 *==========================================================================*/
QCamera3RegularChannel::QCamera3RegularChannel(uint32_t cam_handle,
                    mm_camera_ops_t *cam_ops,
                    channel_cb_routine cb_routine,
                    cam_padding_info_t *paddingInfo,
                    void *userData,
                    camera3_stream_t *stream,
                    cam_stream_type_t stream_type,
                    uint32_t postprocess_mask,
                    uint32_t width, uint32_t height,
                    uint32_t numBuffers) :
                        QCamera3Channel(cam_handle, cam_ops, cb_routine,
                                paddingInfo, postprocess_mask, userData,
                                numBuffers),
                        mFrameCount(0),
                        mLastFrameCount(0),
                        mLastFpsTime(0),
                        mCamera3Stream(stream),
                        mNumBufs(0),
                        mStreamType(stream_type),
                        mWidth(width),
                        mHeight(height)
{
    char prop[PROPERTY_VALUE_MAX];
    property_get("persist.debug.sf.showfps", prop, "0");
    mDebugFPS = (uint8_t) atoi(prop);
}

/*===========================================================================
 * FUNCTION   : ~QCamera3RegularChannel
 *
 * DESCRIPTION: destructor of QCamera3RegularChannel
 *
 * PARAMETERS : none
 *
 * RETURN     : none
 *==========================================================================*/
QCamera3RegularChannel::~QCamera3RegularChannel()
{
}

/*===========================================================================
 * FUNCTION   : initialize
 *
 * DESCRIPTION: Initialize and add camera channel & stream
 *
 * PARAMETERS :
 *    @isType : type of image stabilization required on this stream
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/

int32_t QCamera3RawChannel::initialize(cam_is_type_t isType)
{
    return QCamera3RegularChannel::initialize(isType);
}
int32_t QCamera3RegularChannel::initialize(cam_is_type_t isType)
{
    ATRACE_CALL();
    int32_t rc = NO_ERROR;
    cam_format_t streamFormat;
    cam_dimension_t streamDim;

    if (NULL == mCamera3Stream) {
        ALOGE("%s: Camera stream uninitialized", __func__);
        return NO_INIT;
    }

    if (1 <= m_numStreams) {
        // Only one stream per channel supported in v3 Hal
        return NO_ERROR;
    }

    rc = init(NULL, NULL);
    if (rc < 0) {
        ALOGE("%s: init failed", __func__);
        return rc;
    }

    mNumBufs = CAM_MAX_NUM_BUFS_PER_STREAM;
    mIsType  = isType;

    if (mCamera3Stream->format == HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED) {
        streamFormat = getStreamDefaultFormat(mStreamType);
    } else if(mCamera3Stream->format == HAL_PIXEL_FORMAT_YCbCr_420_888) {
        streamFormat = getStreamDefaultFormat(CAM_STREAM_TYPE_CALLBACK);
    } else if (mCamera3Stream->format == HAL_PIXEL_FORMAT_RAW_OPAQUE ||
         mCamera3Stream->format == HAL_PIXEL_FORMAT_RAW10 ||
         mCamera3Stream->format == HAL_PIXEL_FORMAT_RAW16) {
         // Bayer pattern doesn't matter here.
         // All CAMIF raw format uses 10bit.
         streamFormat = CAM_FORMAT_BAYER_MIPI_RAW_10BPP_GBRG;
    } else {
        //TODO: Fail for other types of streams for now
        ALOGE("%s: format is not IMPLEMENTATION_DEFINED or flexible", __func__);
        return -EINVAL;
    }

    streamDim.width = (int32_t)mWidth;
    streamDim.height = (int32_t)mHeight;

    rc = QCamera3Channel::addStream(mStreamType,
            streamFormat,
            streamDim,
            mNumBufs,
            mPostProcMask,
            mIsType);

    return rc;
}

/*===========================================================================
* FUNCTION   : start
*
* DESCRIPTION: start a regular channel
*
* PARAMETERS :
*
* RETURN     : int32_t type of status
*              NO_ERROR  -- success
*              none-zero failure code
*==========================================================================*/
int32_t QCamera3RegularChannel::start()
{
    ATRACE_CALL();
    int32_t rc = NO_ERROR;

    if (0 < mMemory.getCnt()) {
        rc = QCamera3Channel::start();
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : request
 *
 * DESCRIPTION: process a request from camera service. Stream on if ncessary.
 *
 * PARAMETERS :
 *   @buffer  : buffer to be filled for this request
 *
 * RETURN     : 0 on a success start of capture
 *              -EINVAL on invalid input
 *              -ENODEV on serious error
 *==========================================================================*/
int32_t QCamera3RegularChannel::request(buffer_handle_t *buffer, uint32_t frameNumber)
{
    ATRACE_CALL();
    //FIX ME: Return buffer back in case of failures below.

    int32_t rc = NO_ERROR;
    int index;

    if (NULL == buffer) {
        ALOGE("%s: Invalid buffer in channel request", __func__);
        return BAD_VALUE;
    }

    if(!m_bIsActive) {
        rc = registerBuffer(buffer, mIsType);
        if (NO_ERROR != rc) {
            ALOGE("%s: On-the-fly buffer registration failed %d",
                    __func__, rc);
            return rc;
        }

        rc = start();
        if (NO_ERROR != rc) {
            return rc;
        }
    } else {
        CDBG("%s: Request on an existing stream",__func__);
    }

    index = mMemory.getMatchBufIndex((void*)buffer);
    if(index < 0) {
        rc = registerBuffer(buffer, mIsType);
        if (NO_ERROR != rc) {
            ALOGE("%s: On-the-fly buffer registration failed %d",
                    __func__, rc);
            return rc;
        }

        index = mMemory.getMatchBufIndex((void*)buffer);
        if (index < 0) {
            ALOGE("%s: Could not find object among registered buffers",
                    __func__);
            return DEAD_OBJECT;
        }
    }

    rc = mStreams[0]->bufDone((uint32_t)index);
    if(rc != NO_ERROR) {
        ALOGE("%s: Failed to Q new buffer to stream",__func__);
        return rc;
    }

    rc = mMemory.markFrameNumber((uint32_t)index, frameNumber);
    return rc;
}

/*===========================================================================
 * FUNCTION   : registerBuffer
 *
 * DESCRIPTION: register streaming buffer to the channel object
 *
 * PARAMETERS :
 *   @buffer     : buffer to be registered
 *   @isType : type of image stabilization required on this stream
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3RegularChannel::registerBuffer(buffer_handle_t *buffer, cam_is_type_t isType)
{
    ATRACE_CALL();
    int rc = 0;
    mIsType = isType;
    cam_stream_type_t streamType;

    if (0 == m_numStreams) {
        rc = initialize(mIsType);
        if (rc != NO_ERROR) {
            ALOGE("%s: Couldn't initialize camera stream %d",
                    __func__, rc);
            return rc;
        }
    }

    if (((uint32_t)mMemory.getCnt() + 1) > mNumBufs) {
        ALOGE("%s: Trying to register more buffers than initially requested",
                __func__);
        return BAD_VALUE;
    }

    streamType = mStreams[0]->getMyType();
    rc = mMemory.registerBuffer(buffer, streamType);
    if (ALREADY_EXISTS == rc) {
        return NO_ERROR;
    } else if (NO_ERROR != rc) {
        ALOGE("%s: Buffer %p couldn't be registered %d", __func__, buffer, rc);
        return rc;
    }

    return rc;
}

void QCamera3RegularChannel::streamCbRoutine(
                            mm_camera_super_buf_t *super_frame,
                            QCamera3Stream *stream)
{
    ATRACE_CALL();
    //FIXME Q Buf back in case of error?
    uint8_t frameIndex;
    buffer_handle_t *resultBuffer;
    int32_t resultFrameNumber;
    camera3_stream_buffer_t result;
    cam_dimension_t dim;
    cam_frame_len_offset_t offset;

    memset(&dim, 0, sizeof(dim));
    memset(&offset, 0, sizeof(cam_frame_len_offset_t));

    if (NULL == stream) {
        ALOGE("%s: Invalid stream", __func__);
        return;
    }

    if(!super_frame) {
         ALOGE("%s: Invalid Super buffer",__func__);
         return;
    }

    if(super_frame->num_bufs != 1) {
         ALOGE("%s: Multiple streams are not supported",__func__);
         return;
    }
    if(super_frame->bufs[0] == NULL ) {
         ALOGE("%s: Error, Super buffer frame does not contain valid buffer",
                  __func__);
         return;
    }

    frameIndex = (uint8_t)super_frame->bufs[0]->buf_idx;
    if(frameIndex >= mNumBufs) {
         ALOGE("%s: Error, Invalid index for buffer",__func__);
         stream->bufDone(frameIndex);
         return;
    }

    if (mDebugFPS) {
        showDebugFPS(stream->getMyType());
    }
    stream->getFrameDimension(dim);
    stream->getFrameOffset(offset);
    if (stream->getMyType() == CAM_STREAM_TYPE_PREVIEW) {
        dumpYUV(super_frame->bufs[0], dim, offset, QCAMERA_DUMP_FRM_PREVIEW);
    } else if (stream->getMyType() == CAM_STREAM_TYPE_VIDEO) {
        dumpYUV(super_frame->bufs[0], dim, offset, QCAMERA_DUMP_FRM_VIDEO);
    }

    ////Use below data to issue framework callback
    resultBuffer = (buffer_handle_t *)mMemory.getBufferHandle(frameIndex);
    resultFrameNumber = mMemory.getFrameNumber(frameIndex);

    result.stream = mCamera3Stream;
    result.buffer = resultBuffer;
    result.status = CAMERA3_BUFFER_STATUS_OK;
    result.acquire_fence = -1;
    result.release_fence = -1;
    int32_t rc = stream->bufRelease(frameIndex);
    if (NO_ERROR != rc) {
        ALOGE("%s: Error %d releasing stream buffer %d",
                __func__, rc, frameIndex);
    }

    rc = mMemory.unregisterBuffer(frameIndex);
    if (NO_ERROR != rc) {
        ALOGE("%s: Error %d unregistering stream buffer %d",
                __func__, rc, frameIndex);
    }

    if (0 <= resultFrameNumber) {
        mChannelCB(NULL, &result, (uint32_t)resultFrameNumber, mUserData);
    } else {
        ALOGE("%s: Bad brame number", __func__);
    }
    free(super_frame);
    return;
}

QCamera3Memory* QCamera3RegularChannel::getStreamBufs(uint32_t /*len*/)
{
    return &mMemory;
}

void QCamera3RegularChannel::putStreamBufs()
{
    mMemory.unregisterBuffers();
}

/*===========================================================================
 * FUNCTION   : showDebugFPS
 *
 * DESCRIPTION: Function to log the fps for preview, video, callback and raw
 *              streams
 *
 * PARAMETERS : Stream type
 *
 * RETURN  : None
 *==========================================================================*/
void QCamera3RegularChannel::showDebugFPS(int32_t streamType)
{
    double fps = 0;
    mFrameCount++;
    nsecs_t now = systemTime();
    nsecs_t diff = now - mLastFpsTime;
    if (diff > ms2ns(250)) {
        fps = (((double)(mFrameCount - mLastFrameCount)) *
                (double)(s2ns(1))) / (double)diff;
        switch(streamType) {
            case CAM_STREAM_TYPE_PREVIEW:
                CDBG_HIGH("%s: PROFILE_PREVIEW_FRAMES_PER_SECOND : %.4f",
                        __func__, fps);
                break;
            case CAM_STREAM_TYPE_VIDEO:
                CDBG_HIGH("%s: PROFILE_VIDEO_FRAMES_PER_SECOND : %.4f",
                        __func__, fps);
                break;
            case CAM_STREAM_TYPE_CALLBACK:
                CDBG_HIGH("%s: PROFILE_CALLBACK_FRAMES_PER_SECOND : %.4f",
                        __func__, fps);
                break;
            case CAM_STREAM_TYPE_RAW:
                CDBG_HIGH("%s: PROFILE_RAW_FRAMES_PER_SECOND : %.4f",
                        __func__, fps);
                break;
            default:
                CDBG_HIGH("%s: logging not supported for the stream",
                         __func__);
                break;
        }
        mLastFpsTime = now;
        mLastFrameCount = mFrameCount;
    }
}

QCamera3MetadataChannel::QCamera3MetadataChannel(uint32_t cam_handle,
                    mm_camera_ops_t *cam_ops,
                    channel_cb_routine cb_routine,
                    cam_padding_info_t *paddingInfo,
                    uint32_t postprocess_mask,
                    void *userData, uint32_t numBuffers) :
                        QCamera3Channel(cam_handle, cam_ops,
                                cb_routine, paddingInfo, postprocess_mask,
                                userData, numBuffers),
                        mMemory(NULL)
{
}

QCamera3MetadataChannel::~QCamera3MetadataChannel()
{
    if (m_bIsActive)
        stop();

    if (mMemory) {
        mMemory->deallocate();
        delete mMemory;
        mMemory = NULL;
    }
}

int32_t QCamera3MetadataChannel::initialize(cam_is_type_t isType)
{
    ATRACE_CALL();
    int32_t rc;
    cam_dimension_t streamDim;

    if (mMemory || m_numStreams > 0) {
        ALOGE("%s: metadata channel already initialized", __func__);
        return -EINVAL;
    }

    rc = init(NULL, NULL);
    if (rc < 0) {
        ALOGE("%s: init failed", __func__);
        return rc;
    }

    streamDim.width = (int32_t)sizeof(metadata_buffer_t),
    streamDim.height = 1;

    mIsType = isType;
    rc = QCamera3Channel::addStream(CAM_STREAM_TYPE_METADATA, CAM_FORMAT_MAX,
            streamDim, (uint8_t)mNumBuffers, mPostProcMask, mIsType);
    if (rc < 0) {
        ALOGE("%s: addStream failed", __func__);
    }
    return rc;
}

int32_t QCamera3MetadataChannel::request(buffer_handle_t * /*buffer*/,
                                                uint32_t /*frameNumber*/)
{
    if (!m_bIsActive) {
        return start();
    }
    else
        return 0;
}

void QCamera3MetadataChannel::streamCbRoutine(
                        mm_camera_super_buf_t *super_frame,
                        QCamera3Stream * /*stream*/)
{
    ATRACE_NAME("metadata_stream_cb_routine");
    uint32_t requestNumber = 0;
    if (super_frame == NULL || super_frame->num_bufs != 1) {
        ALOGE("%s: super_frame is not valid", __func__);
        return;
    }
    mChannelCB(super_frame, NULL, requestNumber, mUserData);
}

QCamera3Memory* QCamera3MetadataChannel::getStreamBufs(uint32_t len)
{
    int rc;
    if (len < sizeof(metadata_buffer_t)) {
        ALOGE("%s: Metadata buffer size less than structure %d vs %d",
                __func__,
                len,
                sizeof(metadata_buffer_t));
        return NULL;
    }
    mMemory = new QCamera3HeapMemory();
    if (!mMemory) {
        ALOGE("%s: unable to create metadata memory", __func__);
        return NULL;
    }
    rc = mMemory->allocate(MIN_STREAMING_BUFFER_NUM, len, true);
    if (rc < 0) {
        ALOGE("%s: unable to allocate metadata memory", __func__);
        delete mMemory;
        mMemory = NULL;
        return NULL;
    }
    clear_metadata_buffer((metadata_buffer_t*)mMemory->getPtr(0));
    return mMemory;
}

void QCamera3MetadataChannel::putStreamBufs()
{
    mMemory->deallocate();
    delete mMemory;
    mMemory = NULL;
}
/*************************************************************************************/
// RAW Channel related functions
QCamera3RawChannel::QCamera3RawChannel(uint32_t cam_handle,
                    mm_camera_ops_t *cam_ops,
                    channel_cb_routine cb_routine,
                    cam_padding_info_t *paddingInfo,
                    void *userData,
                    camera3_stream_t *stream,
                    uint32_t postprocess_mask,
                    bool raw_16, uint32_t numBuffers) :
                        QCamera3RegularChannel(cam_handle, cam_ops,
                                cb_routine, paddingInfo, userData, stream,
                                CAM_STREAM_TYPE_RAW, postprocess_mask, numBuffers),
                        mIsRaw16(raw_16)
{
    char prop[PROPERTY_VALUE_MAX];
    property_get("persist.camera.raw.debug.dump", prop, "0");
    mRawDump = atoi(prop);
}

QCamera3RawChannel::~QCamera3RawChannel()
{
}

void QCamera3RawChannel::streamCbRoutine(
                        mm_camera_super_buf_t *super_frame,
                        QCamera3Stream * stream)
{
    ATRACE_CALL();
    /* Move this back down once verified */
    if (mRawDump)
        dumpRawSnapshot(super_frame->bufs[0]);

    if (mIsRaw16) {
        if (getStreamDefaultFormat(CAM_STREAM_TYPE_RAW) ==
                CAM_FORMAT_BAYER_MIPI_RAW_10BPP_GBRG)
            convertMipiToRaw16(super_frame->bufs[0]);
        else
            convertLegacyToRaw16(super_frame->bufs[0]);
    }

    //Make sure cache coherence because extra processing is done
    mMemory.cleanInvalidateCache(super_frame->bufs[0]->buf_idx);

    QCamera3RegularChannel::streamCbRoutine(super_frame, stream);
    return;
}

void QCamera3RawChannel::dumpRawSnapshot(mm_camera_buf_def_t *frame)
{
   QCamera3Stream *stream = getStreamByIndex(0);
   if (stream != NULL) {
       char buf[FILENAME_MAX];
       memset(buf, 0, sizeof(buf));
       cam_dimension_t dim;
       memset(&dim, 0, sizeof(dim));
       stream->getFrameDimension(dim);

       cam_frame_len_offset_t offset;
       memset(&offset, 0, sizeof(cam_frame_len_offset_t));
       stream->getFrameOffset(offset);
       snprintf(buf, sizeof(buf), QCAMERA_DUMP_FRM_LOCATION"r_%d_%dx%d.raw",
                frame->frame_idx, offset.mp[0].stride, offset.mp[0].scanline);

       int file_fd = open(buf, O_RDWR| O_CREAT, 0644);
       if (file_fd >= 0) {
          ssize_t written_len = write(file_fd, frame->buffer, frame->frame_len);
          ALOGE("%s: written number of bytes %zd", __func__, written_len);
          close(file_fd);
       } else {
          ALOGE("%s: failed to open file to dump image", __func__);
       }
   } else {
       ALOGE("%s: Could not find stream", __func__);
   }

}

void QCamera3RawChannel::convertLegacyToRaw16(mm_camera_buf_def_t *frame)
{
    // Convert image buffer from Opaque raw format to RAW16 format
    // 10bit Opaque raw is stored in the format of:
    // 0000 - p5 - p4 - p3 - p2 - p1 - p0
    // where p0 to p5 are 6 pixels (each is 10bit)_and most significant
    // 4 bits are 0s. Each 64bit word contains 6 pixels.

  QCamera3Stream *stream = getStreamByIndex(0);
  if (stream != NULL) {
      cam_dimension_t dim;
      memset(&dim, 0, sizeof(dim));
      stream->getFrameDimension(dim);

      cam_frame_len_offset_t offset;
      memset(&offset, 0, sizeof(cam_frame_len_offset_t));
      stream->getFrameOffset(offset);

      uint32_t raw16_stride = ((uint32_t)dim.width + 15U) & ~15U;
      uint16_t* raw16_buffer = (uint16_t *)frame->buffer;

      // In-place format conversion.
      // Raw16 format always occupy more memory than opaque raw10.
      // Convert to Raw16 by iterating through all pixels from bottom-right
      // to top-left of the image.
      // One special notes:
      // 1. Cross-platform raw16's stride is 16 pixels.
      // 2. Opaque raw10's stride is 6 pixels, and aligned to 16 bytes.
      for (int32_t ys = dim.height - 1; ys >= 0; ys--) {
          uint32_t y = (uint32_t)ys;
          uint64_t* row_start = (uint64_t *)frame->buffer +
                  y * (uint32_t)offset.mp[0].stride_in_bytes / 8;
          for (int32_t xs = dim.width - 1; xs >= 0; xs--) {
              uint32_t x = (uint32_t)xs;
              uint16_t raw16_pixel = 0x3FF & (row_start[x/6] >> (10*(x%6)));
              raw16_buffer[y*raw16_stride+x] = raw16_pixel;
          }
      }
  } else {
      ALOGE("%s: Could not find stream", __func__);
  }

}

void QCamera3RawChannel::convertMipiToRaw16(mm_camera_buf_def_t *frame)
{
    // Convert image buffer from mipi10 raw format to RAW16 format
    // mipi10 opaque raw is stored in the format of:
    // P3(1:0) P2(1:0) P1(1:0) P0(1:0) P3(9:2) P2(9:2) P1(9:2) P0(9:2)
    // 4 pixels occupy 5 bytes, no padding needed

    QCamera3Stream *stream = getStreamByIndex(0);
    if (stream != NULL) {
        cam_dimension_t dim;
        memset(&dim, 0, sizeof(dim));
        stream->getFrameDimension(dim);

        cam_frame_len_offset_t offset;
        memset(&offset, 0, sizeof(cam_frame_len_offset_t));
        stream->getFrameOffset(offset);

        uint32_t raw16_stride = ((uint32_t)dim.width + 15U) & ~15U;
        uint16_t* raw16_buffer = (uint16_t *)frame->buffer;

        // In-place format conversion.
        // Raw16 format always occupy more memory than opaque raw10.
        // Convert to Raw16 by iterating through all pixels from bottom-right
        // to top-left of the image.
        // One special notes:
        // 1. Cross-platform raw16's stride is 16 pixels.
        // 2. mipi raw10's stride is 4 pixels, and aligned to 16 bytes.
        for (int32_t ys = dim.height - 1; ys >= 0; ys--) {
            uint32_t y = (uint32_t)ys;
            uint8_t* row_start = (uint8_t *)frame->buffer +
                    y * (uint32_t)offset.mp[0].stride_in_bytes;
            for (int32_t xs = dim.width - 1; xs >= 0; xs--) {
                uint32_t x = (uint32_t)xs;
                uint8_t upper_8bit = row_start[5*(x/4)+x%4];
                uint8_t lower_2bit = ((row_start[5*(x/4)+4] >> (x%4)) & 0x3);
                uint16_t raw16_pixel =
                        (uint16_t)(((uint16_t)upper_8bit)<<2 |
                        (uint16_t)lower_2bit);
                raw16_buffer[y*raw16_stride+x] = raw16_pixel;
            }
        }
    } else {
        ALOGE("%s: Could not find stream", __func__);
    }

}


/*************************************************************************************/
// RAW Dump Channel related functions

/*===========================================================================
 * FUNCTION   : QCamera3RawDumpChannel
 *
 * DESCRIPTION: Constructor for RawDumpChannel
 *
 * PARAMETERS :
 *   @cam_handle    : Handle for Camera
 *   @cam_ops       : Function pointer table
 *   @rawDumpSize   : Dimensions for the Raw stream
 *   @paddinginfo   : Padding information for stream
 *   @userData      : Cookie for parent
 *   @pp mask       : PP feature mask for this stream
 *   @numBuffers    : number of max dequeued buffers
 *
 * RETURN           : NA
 *==========================================================================*/
QCamera3RawDumpChannel::QCamera3RawDumpChannel(uint32_t cam_handle,
                    mm_camera_ops_t *cam_ops,
                    cam_dimension_t rawDumpSize,
                    cam_padding_info_t *paddingInfo,
                    void *userData,
                    uint32_t postprocess_mask, uint32_t numBuffers) :
                        QCamera3Channel(cam_handle, cam_ops, NULL,
                                paddingInfo, postprocess_mask,
                                userData, numBuffers),
                        mDim(rawDumpSize),
                        mMemory(NULL)
{
    char prop[PROPERTY_VALUE_MAX];
    property_get("persist.camera.raw.dump", prop, "0");
    mRawDump = atoi(prop);
}

/*===========================================================================
 * FUNCTION   : QCamera3RawDumpChannel
 *
 * DESCRIPTION: Destructor for RawDumpChannel
 *
 * PARAMETERS :
 *
 * RETURN           : NA
 *==========================================================================*/

QCamera3RawDumpChannel::~QCamera3RawDumpChannel()
{
}

/*===========================================================================
 * FUNCTION   : dumpRawSnapshot
 *
 * DESCRIPTION: Helper function to dump Raw frames
 *
 * PARAMETERS :
 *  @frame      : stream buf frame to be dumped
 *
 *  RETURN      : NA
 *==========================================================================*/
void QCamera3RawDumpChannel::dumpRawSnapshot(mm_camera_buf_def_t *frame)
{
    QCamera3Stream *stream = getStreamByIndex(0);
    if (stream != NULL) {
        char buf[FILENAME_MAX];
        struct timeval tv;
        struct tm timeinfo_data;
        struct tm *timeinfo;

        cam_dimension_t dim;
        memset(&dim, 0, sizeof(dim));
        stream->getFrameDimension(dim);

        cam_frame_len_offset_t offset;
        memset(&offset, 0, sizeof(cam_frame_len_offset_t));
        stream->getFrameOffset(offset);

        gettimeofday(&tv, NULL);
        timeinfo = localtime_r(&tv.tv_sec, &timeinfo_data);

        if (NULL != timeinfo) {
            memset(buf, 0, sizeof(buf));
            snprintf(buf, sizeof(buf),
                    QCAMERA_DUMP_FRM_LOCATION
                    "%04d-%02d-%02d-%02d-%02d-%02d-%06ld_%d_%dx%d.raw",
                    timeinfo->tm_year + 1900, timeinfo->tm_mon + 1,
                    timeinfo->tm_mday, timeinfo->tm_hour,
                    timeinfo->tm_min, timeinfo->tm_sec,tv.tv_usec,
                    frame->frame_idx, dim.width, dim.height);

            int file_fd = open(buf, O_RDWR| O_CREAT, 0777);
            if (file_fd >= 0) {
                ssize_t written_len =
                        write(file_fd, frame->buffer, offset.frame_len);
                CDBG("%s: written number of bytes %zd", __func__, written_len);
                close(file_fd);
            } else {
                ALOGE("%s: failed to open file to dump image", __func__);
            }
        } else {
            ALOGE("%s: localtime_r() error", __func__);
        }
    } else {
        ALOGE("%s: Could not find stream", __func__);
    }

}

/*===========================================================================
 * FUNCTION   : streamCbRoutine
 *
 * DESCRIPTION: Callback routine invoked for each frame generated for
 *              Rawdump channel
 *
 * PARAMETERS :
 *   @super_frame  : stream buf frame generated
 *   @stream       : Underlying Stream object cookie
 *
 * RETURN          : NA
 *==========================================================================*/
void QCamera3RawDumpChannel::streamCbRoutine(mm_camera_super_buf_t *super_frame,
                                                QCamera3Stream *stream)
{
    CDBG("%s: E",__func__);
    if (super_frame == NULL || super_frame->num_bufs != 1) {
        ALOGE("%s: super_frame is not valid", __func__);
        return;
    }

    if (mRawDump)
        dumpRawSnapshot(super_frame->bufs[0]);

    bufDone(super_frame);
    free(super_frame);
}

/*===========================================================================
 * FUNCTION   : getStreamBufs
 *
 * DESCRIPTION: Callback function provided to interface to get buffers.
 *
 * PARAMETERS :
 *   @len       : Length of each buffer to be allocated
 *
 * RETURN     : NULL on buffer allocation failure
 *              QCamera3Memory object on sucess
 *==========================================================================*/
QCamera3Memory* QCamera3RawDumpChannel::getStreamBufs(uint32_t len)
{
    int rc;
    mMemory = new QCamera3HeapMemory();

    if (!mMemory) {
        ALOGE("%s: unable to create heap memory", __func__);
        return NULL;
    }
    rc = mMemory->allocate(mNumBuffers, (size_t)len, true);
    if (rc < 0) {
        ALOGE("%s: unable to allocate heap memory", __func__);
        delete mMemory;
        mMemory = NULL;
        return NULL;
    }
    return mMemory;
}

/*===========================================================================
 * FUNCTION   : putStreamBufs
 *
 * DESCRIPTION: Callback function provided to interface to return buffers.
 *              Although no handles are actually returned, implicitl assumption
 *              that interface will no longer use buffers and channel can
 *              deallocated if necessary.
 *
 * PARAMETERS : NA
 *
 * RETURN     : NA
 *==========================================================================*/
void QCamera3RawDumpChannel::putStreamBufs()
{
    mMemory->deallocate();
    delete mMemory;
    mMemory = NULL;
}

/*===========================================================================
 * FUNCTION : request
 *
 * DESCRIPTION: Request function used as trigger
 *
 * PARAMETERS :
 * @recvd_frame : buffer- this will be NULL since this is internal channel
 * @frameNumber : Undefined again since this is internal stream
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3RawDumpChannel::request(buffer_handle_t * /*buffer*/,
                                                uint32_t /*frameNumber*/)
{
    if (!m_bIsActive) {
        return QCamera3Channel::start();
    }
    else
        return 0;
}

/*===========================================================================
 * FUNCTION : intialize
 *
 * DESCRIPTION: Initializes channel params and creates underlying stream
 *
 * PARAMETERS :
 *    @isType : type of image stabilization required on this stream
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3RawDumpChannel::initialize(cam_is_type_t isType)
{
    int32_t rc;

    rc = init(NULL, NULL);
    if (rc < 0) {
        ALOGE("%s: init failed", __func__);
        return rc;
    }
    mIsType = isType;
    rc = QCamera3Channel::addStream(CAM_STREAM_TYPE_RAW,
        CAM_FORMAT_BAYER_MIPI_RAW_10BPP_GBRG, mDim, (uint8_t)mNumBuffers,
        mPostProcMask, mIsType);
    if (rc < 0) {
        ALOGE("%s: addStream failed", __func__);
    }
    return rc;
}
/*************************************************************************************/

/*===========================================================================
 * FUNCTION   : jpegEvtHandle
 *
 * DESCRIPTION: Function registerd to mm-jpeg-interface to handle jpeg events.
                Construct result payload and call mChannelCb to deliver buffer
                to framework.
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
void QCamera3PicChannel::jpegEvtHandle(jpeg_job_status_t status,
                                              uint32_t /*client_hdl*/,
                                              uint32_t jobId,
                                              mm_jpeg_output_t *p_output,
                                              void *userdata)
{
    ATRACE_CALL();
    buffer_handle_t *resultBuffer = NULL;
    buffer_handle_t *jpegBufferHandle = NULL;
    int resultStatus = CAMERA3_BUFFER_STATUS_OK;
    camera3_stream_buffer_t result;
    camera3_jpeg_blob_t jpegHeader;

    QCamera3PicChannel *obj = (QCamera3PicChannel *)userdata;
    if (obj) {
        //Construct payload for process_capture_result. Call mChannelCb

        qcamera_hal3_jpeg_data_t *job = obj->m_postprocessor.findJpegJobByJobId(jobId);

        if ((job == NULL) || (status == JPEG_JOB_STATUS_ERROR)) {
            ALOGE("%s: Error in jobId: (%d) with status: %d", __func__, jobId, status);
            resultStatus = CAMERA3_BUFFER_STATUS_ERROR;
        }

        if (NULL != job) {
            uint32_t bufIdx = (uint32_t)job->jpeg_settings->out_buf_index;
            CDBG("%s: jpeg out_buf_index: %d", __func__, bufIdx);

            //Construct jpeg transient header of type camera3_jpeg_blob_t
            //Append at the end of jpeg image of buf_filled_len size

            jpegHeader.jpeg_blob_id = CAMERA3_JPEG_BLOB_ID;
            if (JPEG_JOB_STATUS_DONE == status) {
                jpegHeader.jpeg_size = (uint32_t)p_output->buf_filled_len;
                char* jpeg_buf = (char *)p_output->buf_vaddr;

                ssize_t maxJpegSize = -1;

                // Gralloc buffer may have additional padding for 4K page size
                // Follow size guidelines based on spec since framework relies
                // on that to reach end of buffer and with it the header

                //Handle same as resultBuffer, but for readablity
                jpegBufferHandle =
                        (buffer_handle_t *)obj->mMemory.getBufferHandle(bufIdx);

                if (NULL != jpegBufferHandle) {
                    maxJpegSize = ((private_handle_t*)(*jpegBufferHandle))->width;
                    if (maxJpegSize > obj->mMemory.getSize(bufIdx)) {
                        maxJpegSize = obj->mMemory.getSize(bufIdx);
                    }

                    size_t jpeg_eof_offset =
                            (size_t)(maxJpegSize - (ssize_t)sizeof(jpegHeader));
                    char *jpeg_eof = &jpeg_buf[jpeg_eof_offset];
                    memcpy(jpeg_eof, &jpegHeader, sizeof(jpegHeader));
                    obj->mMemory.cleanInvalidateCache(bufIdx);
                } else {
                    ALOGE("%s: JPEG buffer not found and index: %d",
                            __func__,
                            bufIdx);
                    resultStatus = CAMERA3_BUFFER_STATUS_ERROR;
                }
            }

            ////Use below data to issue framework callback
            resultBuffer =
                    (buffer_handle_t *)obj->mMemory.getBufferHandle(bufIdx);
            int32_t resultFrameNumber = obj->mMemory.getFrameNumber(bufIdx);
            int32_t rc = obj->mMemory.unregisterBuffer(bufIdx);
            if (NO_ERROR != rc) {
                ALOGE("%s: Error %d unregistering stream buffer %d",
                    __func__, rc, bufIdx);
            }

            result.stream = obj->mCamera3Stream;
            result.buffer = resultBuffer;
            result.status = resultStatus;
            result.acquire_fence = -1;
            result.release_fence = -1;

            // Release any snapshot buffers before calling
            // the user callback. The callback can potentially
            // unblock pending requests to snapshot stream.
            int32_t snapshotIdx = -1;
            mm_camera_super_buf_t* src_frame = NULL;

            if (job->src_reproc_frame)
                src_frame = job->src_reproc_frame;
            else
                src_frame = job->src_frame;

            if (src_frame) {
                if (obj->mStreams[0]->getMyHandle() ==
                        src_frame->bufs[0]->stream_id) {
                    snapshotIdx = (int32_t)src_frame->bufs[0]->buf_idx;
                } else {
                    ALOGE("%s: Snapshot stream id %d and source frame %d don't match!",
                            __func__, obj->mStreams[0]->getMyHandle(),
                            src_frame->bufs[0]->stream_id);
                }
            }
            if (0 <= snapshotIdx) {
                Mutex::Autolock lock(obj->mFreeBuffersLock);
                obj->mFreeBufferList.push_back((uint32_t)snapshotIdx);
            } else {
                ALOGE("%s: Snapshot buffer not found!", __func__);
            }

            CDBG("%s: Issue Callback", __func__);
            obj->mChannelCB(NULL,
                    &result,
                    (uint32_t)resultFrameNumber,
                    obj->mUserData);

            // release internal data for jpeg job
            if ((NULL != job->fwk_frame) || (NULL != job->fwk_src_buffer)) {
                obj->mOfflineMetaMemory.deallocate();
                obj->mOfflineMemory.unregisterBuffers();
            }
            obj->m_postprocessor.releaseOfflineBuffers();
            obj->m_postprocessor.releaseJpegJobData(job);
            free(job);
        }

        return;
        // }
    } else {
        ALOGE("%s: Null userdata in jpeg callback", __func__);
    }
}

QCamera3PicChannel::QCamera3PicChannel(uint32_t cam_handle,
                    mm_camera_ops_t *cam_ops,
                    channel_cb_routine cb_routine,
                    cam_padding_info_t *paddingInfo,
                    void *userData,
                    camera3_stream_t *stream,
                    uint32_t postprocess_mask,
                    bool is4KVideo,
                    QCamera3Channel *metadataChannel,
                    uint32_t numBuffers) :
                        QCamera3Channel(cam_handle, cam_ops, cb_routine,
                        paddingInfo, postprocess_mask, userData, numBuffers),
                        m_postprocessor(this),
                        mCamera3Stream(stream),
                        mNumBufsRegistered(CAM_MAX_NUM_BUFS_PER_STREAM),
                        mNumSnapshotBufs(0),
                        mCurrentBufIndex(0U),
                        mPostProcStarted(false),
                        mInputBufferConfig(false),
                        mYuvMemory(NULL),
                        m_pMetaChannel(metadataChannel),
                        mMetaFrame(NULL)
{
    QCamera3HardwareInterface* hal_obj = (QCamera3HardwareInterface*)mUserData;
    m_max_pic_dim = hal_obj->calcMaxJpegDim();
    mYuvWidth = stream->width;
    mYuvHeight = stream->height;
    mStreamType = CAM_STREAM_TYPE_SNAPSHOT;
    // Use same pixelformat for 4K video case
    mStreamFormat = is4KVideo ?
            getStreamDefaultFormat(CAM_STREAM_TYPE_VIDEO)
            :getStreamDefaultFormat(CAM_STREAM_TYPE_SNAPSHOT);
    int32_t rc = m_postprocessor.init(&mMemory, jpegEvtHandle, mPostProcMask,
            this);
    if (rc != 0) {
        ALOGE("Init Postprocessor failed");
    }
}

/*===========================================================================
 * FUNCTION   : stop
 *
 * DESCRIPTION: stop pic channel, which will stop all streams within, including
 *              the reprocessing channel in postprocessor and YUV stream.
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3PicChannel::stop()
{
    int32_t rc = NO_ERROR;
    if(!m_bIsActive) {
        ALOGE("%s: Attempt to stop inactive channel",__func__);
        return rc;
    }

    m_postprocessor.stop();
    mPostProcStarted = false;
    rc |= QCamera3Channel::stop();
    return rc;
}

QCamera3PicChannel::~QCamera3PicChannel()
{
   stop();

   int32_t rc = m_postprocessor.deinit();
   if (rc != 0) {
       ALOGE("De-init Postprocessor failed");
   }

   if (0 < mOfflineMetaMemory.getCnt()) {
       mOfflineMetaMemory.deallocate();
   }
   if (0 < mOfflineMemory.getCnt()) {
       mOfflineMemory.unregisterBuffers();
   }
}

int32_t QCamera3PicChannel::initialize(cam_is_type_t isType)
{
    int32_t rc = NO_ERROR;
    cam_dimension_t streamDim;
    cam_stream_type_t streamType;
    cam_format_t streamFormat;
    mm_camera_channel_attr_t attr;

    if (NULL == mCamera3Stream) {
        ALOGE("%s: Camera stream uninitialized", __func__);
        return NO_INIT;
    }

    if (1 <= m_numStreams) {
        // Only one stream per channel supported in v3 Hal
        return NO_ERROR;
    }

    memset(&attr, 0, sizeof(mm_camera_channel_attr_t));
    attr.notify_mode = MM_CAMERA_SUPER_BUF_NOTIFY_BURST;
    attr.look_back = 1;
    attr.post_frame_skip = 1;
    attr.water_mark = 1;
    attr.max_unmatched_frames = 1;

    rc = init(&attr, NULL);
    if (rc < 0) {
        ALOGE("%s: init failed", __func__);
        return rc;
    }
    mIsType = isType;
    streamType = mStreamType;
    streamFormat = mStreamFormat;
    streamDim.width = (int32_t)mYuvWidth;
    streamDim.height = (int32_t)mYuvHeight;

    mNumSnapshotBufs = mCamera3Stream->max_buffers;
    rc = QCamera3Channel::addStream(streamType, streamFormat, streamDim,
            (uint8_t)mCamera3Stream->max_buffers, mPostProcMask, mIsType);

    Mutex::Autolock lock(mFreeBuffersLock);
    mFreeBufferList.clear();
    for (uint32_t i = 0; i < mCamera3Stream->max_buffers; i++) {
        mFreeBufferList.push_back(i);
    }

    return rc;
}

int32_t QCamera3PicChannel::request(buffer_handle_t *buffer,
        uint32_t frameNumber,
        camera3_stream_buffer_t *pInputBuffer,
        metadata_buffer_t *metadata)
{
    ATRACE_CALL();
    //FIX ME: Return buffer back in case of failures below.

    int32_t rc = NO_ERROR;

    reprocess_config_t reproc_cfg;
    memset(&reproc_cfg, 0, sizeof(reprocess_config_t));
    reproc_cfg.padding = &mPaddingInfo;
    //to ensure a big enough buffer size set the height and width
    //padding to max(height padding, width padding)
    if (reproc_cfg.padding->height_padding > reproc_cfg.padding->width_padding) {
       reproc_cfg.padding->width_padding = reproc_cfg.padding->height_padding;
    } else {
       reproc_cfg.padding->height_padding = reproc_cfg.padding->width_padding;
    }

    reproc_cfg.input_stream_dim.width = (int32_t)mYuvWidth;
    reproc_cfg.input_stream_dim.height = (int32_t)mYuvHeight;
    if (NULL == pInputBuffer)
       reproc_cfg.src_channel = this;

    reproc_cfg.output_stream_dim.width = (int32_t)mCamera3Stream->width;
    reproc_cfg.output_stream_dim.height = (int32_t)mCamera3Stream->height;
    reproc_cfg.stream_type = mStreamType;
    reproc_cfg.stream_format = mStreamFormat;
    rc = mm_stream_calc_offset_snapshot(mStreamFormat, &reproc_cfg.input_stream_dim,
            reproc_cfg.padding, &reproc_cfg.input_stream_plane_info);
    if (rc != 0) {
        ALOGE("%s: Snapshot stream plane info calculation failed!", __func__);
        return rc;
    }

    // Picture stream has already been started before any request comes in
    if (!m_bIsActive) {
        ALOGE("%s: Channel not started!!", __func__);
        return NO_INIT;
    }

    int index = mMemory.getMatchBufIndex((void*)buffer);

    if(index < 0) {
        rc = registerBuffer(buffer, mIsType);
        if (NO_ERROR != rc) {
            ALOGE("%s: On-the-fly buffer registration failed %d",
                    __func__, rc);
            return rc;
        }

        index = mMemory.getMatchBufIndex((void*)buffer);
        if (index < 0) {
            ALOGE("%s: Could not find object among registered buffers",__func__);
            return DEAD_OBJECT;
        }
    }
    CDBG("%s: buffer index %d, frameNumber: %u", __func__, index, frameNumber);

    rc = mMemory.markFrameNumber((uint32_t)index, frameNumber);

    //Start the postprocessor for jpeg encoding. Pass mMemory as destination buffer
    mCurrentBufIndex = (uint32_t)index;

    // Start postprocessor
    // This component needs to be re-configured
    // once we switch from input(framework) buffer
    // reprocess to standard capture!
    bool restartNeeded = ((!mInputBufferConfig) != (NULL != pInputBuffer));
    if((!mPostProcStarted) || restartNeeded) {
        m_postprocessor.start(reproc_cfg, metadata);
        mPostProcStarted = true;
        mInputBufferConfig = (NULL == pInputBuffer);
    }

    // Queue jpeg settings
    rc = queueJpegSetting((uint32_t)index, metadata);

    if (pInputBuffer == NULL) {
        Mutex::Autolock lock(mFreeBuffersLock);
        if (!mFreeBufferList.empty()) {
            List<uint32_t>::iterator it = mFreeBufferList.begin();
            uint32_t freeBuffer = *it;
            mStreams[0]->bufDone(freeBuffer);
            mFreeBufferList.erase(it);
        } else {
            ALOGE("%s: No snapshot buffers available!", __func__);
            rc = NOT_ENOUGH_DATA;
        }
    } else {
        if (0 < mOfflineMetaMemory.getCnt()) {
            mOfflineMetaMemory.deallocate();
        }
        if (0 < mOfflineMemory.getCnt()) {
            mOfflineMemory.unregisterBuffers();
        }

        int input_index = mOfflineMemory.getMatchBufIndex((void*)pInputBuffer->buffer);
        if(input_index < 0) {
            rc = mOfflineMemory.registerBuffer(pInputBuffer->buffer, mStreamType);
            if (NO_ERROR != rc) {
                ALOGE("%s: On-the-fly input buffer registration failed %d",
                        __func__, rc);
                return rc;
            }

            input_index = mOfflineMemory.getMatchBufIndex((void*)pInputBuffer->buffer);
            if (input_index < 0) {
                ALOGE("%s: Could not find object among registered buffers",__func__);
                return DEAD_OBJECT;
            }
        }
        qcamera_fwk_input_pp_data_t *src_frame = NULL;
        src_frame = (qcamera_fwk_input_pp_data_t *)malloc(
                sizeof(qcamera_fwk_input_pp_data_t));
        if (src_frame == NULL) {
            ALOGE("%s: No memory for src frame", __func__);
            return NO_MEMORY;
        }
        memset(src_frame, 0, sizeof(qcamera_fwk_input_pp_data_t));
        src_frame->src_frame = *pInputBuffer;
        rc = mOfflineMemory.getBufDef(reproc_cfg.input_stream_plane_info.plane_info,
                src_frame->input_buffer, (uint32_t)input_index);
        if (rc != 0) {
            free(src_frame);
            return rc;
        }
        dumpYUV(&src_frame->input_buffer, reproc_cfg.input_stream_dim,
                reproc_cfg.input_stream_plane_info.plane_info, QCAMERA_DUMP_FRM_SNAPSHOT);
        cam_dimension_t dim = {(int)sizeof(metadata_buffer_t), 1};
        cam_stream_buf_plane_info_t meta_planes;
        rc = mm_stream_calc_offset_metadata(&dim, &mPaddingInfo, &meta_planes);
        if (rc != 0) {
            ALOGE("%s: Metadata stream plane info calculation failed!", __func__);
            free(src_frame);
            return rc;
        }

        rc = mOfflineMetaMemory.allocate(1, sizeof(metadata_buffer_t), false);
        if (NO_ERROR != rc) {
            ALOGE("%s: Couldn't allocate offline metadata buffer!", __func__);
            free(src_frame);
            return rc;
        }
        mm_camera_buf_def_t meta_buf;
        cam_frame_len_offset_t offset = meta_planes.plane_info;
        rc = mOfflineMetaMemory.getBufDef(offset, meta_buf, 0);
        if (NO_ERROR != rc) {
            free(src_frame);
            return rc;
        }
        memcpy(meta_buf.buffer, metadata, sizeof(metadata_buffer_t));
        src_frame->metadata_buffer = meta_buf;
        src_frame->reproc_config = reproc_cfg;

        CDBG_HIGH("%s: Post-process started", __func__);
        CDBG_HIGH("%s: Issue call to reprocess", __func__);

        m_postprocessor.processData(src_frame);
    }
    return rc;
}


/*===========================================================================
 * FUNCTION : metadataBufDone
 *
 * DESCRIPTION: Buffer done method for a metadata buffer
 *
 * PARAMETERS :
 * @recvd_frame : received metadata frame
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3PicChannel::metadataBufDone(mm_camera_super_buf_t *recvd_frame)
{
    int32_t rc = NO_ERROR;;
    if ((NULL == m_pMetaChannel) || (NULL == recvd_frame)) {
        ALOGE("%s: Metadata channel or metadata buffer invalid", __func__);
        return BAD_VALUE;
    }

    rc = ((QCamera3MetadataChannel*)m_pMetaChannel)->bufDone(recvd_frame);

    return rc;
}

/*===========================================================================
 * FUNCTION   : dataNotifyCB
 *
 * DESCRIPTION: Channel Level callback used for super buffer data notify.
 *              This function is registered with mm-camera-interface to handle
 *              data notify
 *
 * PARAMETERS :
 *   @recvd_frame   : stream frame received
 *   userdata       : user data ptr
 *
 * RETURN     : none
 *==========================================================================*/
void QCamera3PicChannel::dataNotifyCB(mm_camera_super_buf_t *recvd_frame,
                                 void *userdata)
{
    ATRACE_CALL();
    CDBG("%s: E\n", __func__);
    QCamera3PicChannel *channel = (QCamera3PicChannel *)userdata;

    if (channel == NULL) {
        ALOGE("%s: invalid channel pointer", __func__);
        return;
    }

    if(channel->m_numStreams != 1) {
        ALOGE("%s: Error: Bug: This callback assumes one stream per channel",__func__);
        return;
    }


    if(channel->mStreams[0] == NULL) {
        ALOGE("%s: Error: Invalid Stream object",__func__);
        return;
    }

    channel->QCamera3PicChannel::streamCbRoutine(recvd_frame, channel->mStreams[0]);

    CDBG("%s: X\n", __func__);
    return;
}

/*===========================================================================
 * FUNCTION   : registerBuffer
 *
 * DESCRIPTION: register streaming buffer to the channel object
 *
 * PARAMETERS :
 *   @buffer     : buffer to be registered
 *   @isType     : type of image stabilization required on this channel
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3PicChannel::registerBuffer(buffer_handle_t *buffer, cam_is_type_t isType)
{
    int rc = 0;
    mIsType = isType;
    if ((uint32_t)mMemory.getCnt() > (mNumBufsRegistered - 1)) {
        ALOGE("%s: Trying to register more buffers than initially requested",
                __func__);
        return BAD_VALUE;
    }

    if (0 == m_numStreams) {
        rc = initialize(mIsType);
        if (rc != NO_ERROR) {
            ALOGE("%s: Couldn't initialize camera stream %d",
                    __func__, rc);
            return rc;
        }
    }

    rc = mMemory.registerBuffer(buffer, mStreamType);
    if (ALREADY_EXISTS == rc) {
        return NO_ERROR;
    } else if (NO_ERROR != rc) {
        ALOGE("%s: Buffer %p couldn't be registered %d", __func__, buffer, rc);
        return rc;
    }

    CDBG("%s: X",__func__);

    return rc;
}

void QCamera3PicChannel::streamCbRoutine(mm_camera_super_buf_t *super_frame,
                            QCamera3Stream *stream)
{
    ATRACE_CALL();
    //TODO
    //Used only for getting YUV. Jpeg callback will be sent back from channel
    //directly to HWI. Refer to func jpegEvtHandle

    //Got the yuv callback. Calling yuv callback handler in PostProc
    uint8_t frameIndex;
    mm_camera_super_buf_t* frame = NULL;
    cam_dimension_t dim;
    cam_frame_len_offset_t offset;

    memset(&dim, 0, sizeof(dim));
    memset(&offset, 0, sizeof(cam_frame_len_offset_t));

    if(!super_frame) {
         ALOGE("%s: Invalid Super buffer",__func__);
         return;
    }

    if(super_frame->num_bufs != 1) {
         ALOGE("%s: Multiple streams are not supported",__func__);
         return;
    }
    if(super_frame->bufs[0] == NULL ) {
         ALOGE("%s: Error, Super buffer frame does not contain valid buffer",
                  __func__);
         return;
    }

    frameIndex = (uint8_t)super_frame->bufs[0]->buf_idx;
    CDBG("%s: recvd buf_idx: %u for further processing",
        __func__, (uint32_t)frameIndex);
    if(frameIndex >= mNumSnapshotBufs) {
         ALOGE("%s: Error, Invalid index for buffer",__func__);
         if(stream) {
             Mutex::Autolock lock(mFreeBuffersLock);
             mFreeBufferList.push_back(frameIndex);
             stream->bufDone(frameIndex);
         }
         return;
    }

    frame = (mm_camera_super_buf_t *)malloc(sizeof(mm_camera_super_buf_t));
    if (frame == NULL) {
       ALOGE("%s: Error allocating memory to save received_frame structure.",
                                                                    __func__);
       if(stream) {
           Mutex::Autolock lock(mFreeBuffersLock);
           mFreeBufferList.push_back(frameIndex);
           stream->bufDone(frameIndex);
       }
       return;
    }
    *frame = *super_frame;
    stream->getFrameDimension(dim);
    stream->getFrameOffset(offset);
    dumpYUV(frame->bufs[0], dim, offset, QCAMERA_DUMP_FRM_SNAPSHOT);

    m_postprocessor.processData(frame);
    free(super_frame);
    return;
}

QCamera3Memory* QCamera3PicChannel::getStreamBufs(uint32_t len)
{
    int rc = 0;

    mYuvMemory = new QCamera3HeapMemory();
    if (!mYuvMemory) {
        ALOGE("%s: unable to create metadata memory", __func__);
        return NULL;
    }

    //Queue YUV buffers in the beginning mQueueAll = true
    rc = mYuvMemory->allocate(mCamera3Stream->max_buffers, len, false);
    if (rc < 0) {
        ALOGE("%s: unable to allocate metadata memory", __func__);
        delete mYuvMemory;
        mYuvMemory = NULL;
        return NULL;
    }
    return mYuvMemory;
}

void QCamera3PicChannel::putStreamBufs()
{
    mMemory.unregisterBuffers();

    mYuvMemory->deallocate();
    delete mYuvMemory;
    mYuvMemory = NULL;
}

int32_t QCamera3PicChannel::queueReprocMetadata(mm_camera_super_buf_t *metadata)
{
    return m_postprocessor.processPPMetadata(metadata);
}

int32_t QCamera3PicChannel::queueJpegSetting(uint32_t index, metadata_buffer_t *metadata)
{
    jpeg_settings_t *settings =
            (jpeg_settings_t *)malloc(sizeof(jpeg_settings_t));

    if (!settings) {
        ALOGE("%s: out of memory allocating jpeg_settings", __func__);
        return -ENOMEM;
    }

    memset(settings, 0, sizeof(jpeg_settings_t));

    settings->out_buf_index = index;

    settings->jpeg_orientation = 0;
    IF_META_AVAILABLE(int32_t, orientation, CAM_INTF_META_JPEG_ORIENTATION, metadata) {
        settings->jpeg_orientation = *orientation;
    }

    settings->jpeg_quality = 85;
    IF_META_AVAILABLE(uint32_t, quality1, CAM_INTF_META_JPEG_QUALITY, metadata) {
        settings->jpeg_quality = (uint8_t) *quality1;
    }

    IF_META_AVAILABLE(uint32_t, quality2, CAM_INTF_META_JPEG_THUMB_QUALITY, metadata) {
        settings->jpeg_thumb_quality = (uint8_t) *quality2;
    }

    IF_META_AVAILABLE(cam_dimension_t, dimension, CAM_INTF_META_JPEG_THUMB_SIZE, metadata) {
        settings->thumbnail_size = *dimension;
    }

    settings->gps_timestamp_valid = 0;
    IF_META_AVAILABLE(int64_t, timestamp, CAM_INTF_META_JPEG_GPS_TIMESTAMP, metadata) {
        settings->gps_timestamp = *timestamp;
        settings->gps_timestamp_valid = 1;
    }

    settings->gps_coordinates_valid = 0;
    IF_META_AVAILABLE(double, coordinates, CAM_INTF_META_JPEG_GPS_COORDINATES, metadata) {
        memcpy(settings->gps_coordinates, coordinates, 3*sizeof(double));
        settings->gps_coordinates_valid = 1;
    }

    IF_META_AVAILABLE(uint8_t, proc_methods, CAM_INTF_META_JPEG_GPS_PROC_METHODS, metadata) {
        memset(settings->gps_processing_method, 0,
                sizeof(settings->gps_processing_method));
        strlcpy(settings->gps_processing_method, (const char *)proc_methods,
                sizeof(settings->gps_processing_method));
    }

    return m_postprocessor.processJpegSettingData(settings);
}

/*===========================================================================
 * FUNCTION   : getRational
 *
 * DESCRIPTION: compose rational struct
 *
 * PARAMETERS :
 *   @rat     : ptr to struct to store rational info
 *   @num     :num of the rational
 *   @denom   : denom of the rational
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t getRational(rat_t *rat, int num, int denom)
{
    if ((0 > num) || (0 > denom)) {
        ALOGE("%s: Negative values", __func__);
        return BAD_VALUE;
    }
    if (NULL == rat) {
        ALOGE("%s: NULL rat input", __func__);
        return BAD_VALUE;
    }
    rat->num = (uint32_t)num;
    rat->denom = (uint32_t)denom;
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : parseGPSCoordinate
 *
 * DESCRIPTION: parse GPS coordinate string
 *
 * PARAMETERS :
 *   @coord_str : [input] coordinate string
 *   @coord     : [output]  ptr to struct to store coordinate
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int parseGPSCoordinate(const char *coord_str, rat_t* coord)
{
    if(coord == NULL) {
        ALOGE("%s: error, invalid argument coord == NULL", __func__);
        return BAD_VALUE;
    }
    double degF = atof(coord_str);
    if (degF < 0) {
        degF = -degF;
    }
    double minF = (degF - (int) degF) * 60;
    double secF = (minF - (int) minF) * 60;

    getRational(&coord[0], (int)degF, 1);
    getRational(&coord[1], (int)minF, 1);
    getRational(&coord[2], (int)(secF * 10000), 10000);
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : getExifDateTime
 *
 * DESCRIPTION: query exif date time
 *
 * PARAMETERS :
 *   @dateTime   : string to store exif date time
 *   @subsecTime : string to store exif subsec time
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t getExifDateTime(String8 &dateTime, String8 &subsecTime)
{
    int32_t ret = NO_ERROR;

    //get time and date from system
    struct timeval tv;
    struct tm timeinfo_data;

    int res = gettimeofday(&tv, NULL);
    if (0 == res) {
        struct tm *timeinfo = localtime_r(&tv.tv_sec, &timeinfo_data);
        if (NULL != timeinfo) {
            //Write datetime according to EXIF Spec
            //"YYYY:MM:DD HH:MM:SS" (20 chars including \0)
            dateTime = String8::format("%04d:%02d:%02d %02d:%02d:%02d",
                    timeinfo->tm_year + 1900, timeinfo->tm_mon + 1,
                    timeinfo->tm_mday, timeinfo->tm_hour,
                    timeinfo->tm_min, timeinfo->tm_sec);
            //Write subsec according to EXIF Sepc
            subsecTime = String8::format("%06ld", tv.tv_usec);
        } else {
            ALOGE("%s: localtime_r() error", __func__);
            ret = UNKNOWN_ERROR;
        }
    } else if (-1 == res) {
        ALOGE("%s: gettimeofday() error: %s", __func__, strerror(errno));
        ret = UNKNOWN_ERROR;
    } else {
        ALOGE("%s: gettimeofday() unexpected return code: %d", __func__, res);
        ret = UNKNOWN_ERROR;
    }

    return ret;
}

/*===========================================================================
 * FUNCTION   : getExifFocalLength
 *
 * DESCRIPTION: get exif focal lenght
 *
 * PARAMETERS :
 *   @focalLength : ptr to rational strcut to store focal lenght
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t getExifFocalLength(rat_t *focalLength, float value)
{
    int focalLengthValue =
        (int)(value * FOCAL_LENGTH_DECIMAL_PRECISION);
    return getRational(focalLength, focalLengthValue, FOCAL_LENGTH_DECIMAL_PRECISION);
}

/*===========================================================================
  * FUNCTION   : getExifExpTimeInfo
  *
  * DESCRIPTION: get exif exposure time information
  *
  * PARAMETERS :
  *   @expoTimeInfo     : expousure time value
  * RETURN     : nt32_t type of status
  *              NO_ERROR  -- success
  *              none-zero failure code
  *==========================================================================*/
int32_t getExifExpTimeInfo(rat_t *expoTimeInfo, int64_t value)
{

    int64_t cal_exposureTime;
    if (value != 0)
        cal_exposureTime = value;
    else
        cal_exposureTime = 60;

    return getRational(expoTimeInfo, 1, (int)cal_exposureTime);
}

/*===========================================================================
 * FUNCTION   : getExifGpsProcessingMethod
 *
 * DESCRIPTION: get GPS processing method
 *
 * PARAMETERS :
 *   @gpsProcessingMethod : string to store GPS process method
 *   @count               : lenght of the string
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t getExifGpsProcessingMethod(char *gpsProcessingMethod,
                                   uint32_t &count, char* value)
{
    if(value != NULL) {
        memcpy(gpsProcessingMethod, ExifAsciiPrefix, EXIF_ASCII_PREFIX_SIZE);
        count = EXIF_ASCII_PREFIX_SIZE;
        strlcpy(gpsProcessingMethod + EXIF_ASCII_PREFIX_SIZE,
                value,
                GPS_PROCESSING_METHOD_SIZE);
        count += (uint32_t)strlen(value);
        gpsProcessingMethod[count++] = '\0'; // increase 1 for the last NULL char
        return NO_ERROR;
    } else {
        return BAD_VALUE;
    }
}

/*===========================================================================
 * FUNCTION   : getExifLatitude
 *
 * DESCRIPTION: get exif latitude
 *
 * PARAMETERS :
 *   @latitude : ptr to rational struct to store latitude info
 *   @ladRef   : charater to indicate latitude reference
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t getExifLatitude(rat_t *latitude,
                                           char *latRef, double value)
{
    char str[30];
    snprintf(str, sizeof(str), "%f", value);
    if(str != NULL) {
        parseGPSCoordinate(str, latitude);

        //set Latitude Ref
        float latitudeValue = strtof(str, 0);
        if(latitudeValue < 0.0f) {
            latRef[0] = 'S';
        } else {
            latRef[0] = 'N';
        }
        latRef[1] = '\0';
        return NO_ERROR;
    }else{
        return BAD_VALUE;
    }
}

/*===========================================================================
 * FUNCTION   : getExifLongitude
 *
 * DESCRIPTION: get exif longitude
 *
 * PARAMETERS :
 *   @longitude : ptr to rational struct to store longitude info
 *   @lonRef    : charater to indicate longitude reference
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t getExifLongitude(rat_t *longitude,
                                            char *lonRef, double value)
{
    char str[30];
    snprintf(str, sizeof(str), "%f", value);
    if(str != NULL) {
        parseGPSCoordinate(str, longitude);

        //set Longitude Ref
        float longitudeValue = strtof(str, 0);
        if(longitudeValue < 0.0f) {
            lonRef[0] = 'W';
        } else {
            lonRef[0] = 'E';
        }
        lonRef[1] = '\0';
        return NO_ERROR;
    }else{
        return BAD_VALUE;
    }
}

/*===========================================================================
 * FUNCTION   : getExifAltitude
 *
 * DESCRIPTION: get exif altitude
 *
 * PARAMETERS :
 *   @altitude : ptr to rational struct to store altitude info
 *   @altRef   : charater to indicate altitude reference
 *   @argValue : altitude value
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t getExifAltitude(rat_t *altitude, char *altRef, double argValue)
{
    char str[30];
    snprintf(str, sizeof(str), "%f", argValue);
    if (str != NULL) {
        double value = atof(str);
        *altRef = 0;
        if(value < 0){
            *altRef = 1;
            value = -value;
        }
        return getRational(altitude, (int)(value * 1000), 1000);
    } else {
        return BAD_VALUE;
    }
}

/*===========================================================================
 * FUNCTION   : getExifGpsDateTimeStamp
 *
 * DESCRIPTION: get exif GPS date time stamp
 *
 * PARAMETERS :
 *   @gpsDateStamp : GPS date time stamp string
 *   @bufLen       : length of the string
 *   @gpsTimeStamp : ptr to rational struct to store time stamp info
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t getExifGpsDateTimeStamp(char *gpsDateStamp,
                                           uint32_t bufLen,
                                           rat_t *gpsTimeStamp, int64_t value)
{
    char str[30];
    snprintf(str, sizeof(str), "%lld", (long long int)value);
    if(str != NULL) {
        time_t unixTime = (time_t)atol(str);
        struct tm *UTCTimestamp = gmtime(&unixTime);
        if (UTCTimestamp != NULL) {
            strftime(gpsDateStamp, bufLen, "%Y:%m:%d", UTCTimestamp);

            getRational(&gpsTimeStamp[0], UTCTimestamp->tm_hour, 1);
            getRational(&gpsTimeStamp[1], UTCTimestamp->tm_min, 1);
            getRational(&gpsTimeStamp[2], UTCTimestamp->tm_sec, 1);
            return NO_ERROR;
        } else {
            ALOGE("%s: Could not get the timestamp", __func__);
            return BAD_VALUE;
        }
    } else {
        return BAD_VALUE;
    }
}

int32_t getExifExposureValue(srat_t* exposure_val, int32_t exposure_comp,
                             cam_rational_type_t step)
{
    exposure_val->num = exposure_comp * step.numerator;
    exposure_val->denom = step.denominator;
    return 0;
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
QCamera3Exif *QCamera3PicChannel::getExifData(metadata_buffer_t *metadata,
        jpeg_settings_t *jpeg_settings)
{
    QCamera3Exif *exif = new QCamera3Exif();
    if (exif == NULL) {
        ALOGE("%s: No memory for QCamera3Exif", __func__);
        return NULL;
    }

    int32_t rc = NO_ERROR;
    uint32_t count = 0;

    // add exif entries
    String8 dateTime;
    String8 subsecTime;
    rc = getExifDateTime(dateTime, subsecTime);
    if (rc == NO_ERROR) {
        exif->addEntry(EXIFTAGID_DATE_TIME, EXIF_ASCII,
                (uint32_t)(dateTime.length() + 1), (void *)dateTime.string());
        exif->addEntry(EXIFTAGID_EXIF_DATE_TIME_ORIGINAL, EXIF_ASCII,
                (uint32_t)(dateTime.length() + 1), (void *)dateTime.string());
        exif->addEntry(EXIFTAGID_EXIF_DATE_TIME_DIGITIZED, EXIF_ASCII,
                (uint32_t)(dateTime.length() + 1), (void *)dateTime.string());
        exif->addEntry(EXIFTAGID_SUBSEC_TIME, EXIF_ASCII,
                (uint32_t)(subsecTime.length() + 1), (void *)subsecTime.string());
        exif->addEntry(EXIFTAGID_SUBSEC_TIME_ORIGINAL, EXIF_ASCII,
                (uint32_t)(subsecTime.length() + 1), (void *)subsecTime.string());
        exif->addEntry(EXIFTAGID_SUBSEC_TIME_DIGITIZED, EXIF_ASCII,
                (uint32_t)(subsecTime.length() + 1), (void *)subsecTime.string());
    } else {
        ALOGE("%s: getExifDateTime failed", __func__);
    }

    if (metadata != NULL) {
        IF_META_AVAILABLE(float,
                focal_length,
                CAM_INTF_META_LENS_FOCAL_LENGTH,
                metadata) {
            rat_t focalLength;
            rc = getExifFocalLength(&focalLength, *focal_length);
            if (rc == NO_ERROR) {
                exif->addEntry(EXIFTAGID_FOCAL_LENGTH,
                        EXIF_RATIONAL,
                        1,
                        (void *)&(focalLength));
            } else {
                ALOGE("%s: getExifFocalLength failed", __func__);
            }
        }

        IF_META_AVAILABLE(int32_t, isoSpeed, CAM_INTF_META_SENSOR_SENSITIVITY,
                metadata) {
            int16_t fwk_isoSpeed = (int16_t) *isoSpeed;
            exif->addEntry(EXIFTAGID_ISO_SPEED_RATING,
                    EXIF_SHORT,
                    1,
                    (void *) &(fwk_isoSpeed));
        }

        IF_META_AVAILABLE(int64_t, sensor_exposure_time,
                CAM_INTF_META_SENSOR_EXPOSURE_TIME, metadata) {
            rat_t sensorExpTime;
            rc = getExifExpTimeInfo(&sensorExpTime, *sensor_exposure_time);
            if (rc == NO_ERROR){
                exif->addEntry(EXIFTAGID_EXPOSURE_TIME,
                        EXIF_RATIONAL,
                        1,
                        (void *)&(isoSpeed));
            }
        }

        char* jpeg_gps_processing_method = jpeg_settings->gps_processing_method;
        if (strlen(jpeg_gps_processing_method) > 0) {
            char gpsProcessingMethod[EXIF_ASCII_PREFIX_SIZE +
                    GPS_PROCESSING_METHOD_SIZE];
            count = 0;
            rc = getExifGpsProcessingMethod(gpsProcessingMethod,
                    count,
                    jpeg_gps_processing_method);
            if(rc == NO_ERROR) {
                exif->addEntry(EXIFTAGID_GPS_PROCESSINGMETHOD,
                        EXIF_ASCII,
                        count,
                        (void *)gpsProcessingMethod);
            } else {
                ALOGE("%s: getExifGpsProcessingMethod failed", __func__);
            }
        }

        if (jpeg_settings->gps_coordinates_valid) {

            //latitude
            rat_t latitude[3];
            char latRef[2];
            rc = getExifLatitude(latitude, latRef,
                    jpeg_settings->gps_coordinates[0]);
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

            //longitude
            rat_t longitude[3];
            char lonRef[2];
            rc = getExifLongitude(longitude, lonRef,
                    jpeg_settings->gps_coordinates[1]);
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

            //altitude
            rat_t altitude;
            char altRef;
            rc = getExifAltitude(&altitude, &altRef,
                    jpeg_settings->gps_coordinates[2]);
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
        }

        if (jpeg_settings->gps_timestamp_valid) {

            char gpsDateStamp[20];
            rat_t gpsTimeStamp[3];
            rc = getExifGpsDateTimeStamp(gpsDateStamp, 20, gpsTimeStamp,
                    jpeg_settings->gps_timestamp);
            if(rc == NO_ERROR) {
                exif->addEntry(EXIFTAGID_GPS_DATESTAMP, EXIF_ASCII,
                        (uint32_t)(strlen(gpsDateStamp) + 1),
                        (void *)gpsDateStamp);

                exif->addEntry(EXIFTAGID_GPS_TIMESTAMP,
                        EXIF_RATIONAL,
                        3,
                        (void *)gpsTimeStamp);
            } else {
                ALOGE("%s: getExifGpsDataTimeStamp failed", __func__);
            }
        }

        IF_META_AVAILABLE(int32_t,
                exposure_comp,
                CAM_INTF_PARM_EXPOSURE_COMPENSATION,
                metadata) {
            IF_META_AVAILABLE(cam_rational_type_t,
                    comp_step,
                    CAM_INTF_PARM_EV_STEP,
                    metadata) {
                srat_t exposure_val;
                rc = getExifExposureValue(&exposure_val,
                        *exposure_comp,
                        *comp_step);
                if(rc == NO_ERROR) {
                    exif->addEntry(EXIFTAGID_EXPOSURE_BIAS_VALUE,
                            EXIF_SRATIONAL,
                            1,
                            (void *)(&exposure_val));
                } else {
                    ALOGE("%s: getExifExposureValue failed ", __func__);
                }
            }
        }
    } else {
        ALOGE("%s: no metadata provided ", __func__);
    }

    char value[PROPERTY_VALUE_MAX];
    if (property_get("ro.product.manufacturer", value, "QCOM-AA") > 0) {
        exif->addEntry(EXIFTAGID_MAKE, EXIF_ASCII,
                (uint32_t)(strlen(value) + 1), (void *)value);
    } else {
        ALOGE("%s: getExifMaker failed", __func__);
    }

    if (property_get("ro.product.model", value, "QCAM-AA") > 0) {
        exif->addEntry(EXIFTAGID_MODEL, EXIF_ASCII,
                (uint32_t)(strlen(value) + 1), (void *)value);
    } else {
        ALOGE("%s: getExifModel failed", __func__);
    }

    if (property_get("ro.build.description", value, "QCAM-AA") > 0) {
        exif->addEntry(EXIFTAGID_SOFTWARE, EXIF_ASCII,
                (uint32_t)(strlen(value) + 1), (void *)value);
    } else {
        ALOGE("%s: getExifSoftware failed", __func__);
    }

    return exif;
}

void QCamera3PicChannel::overrideYuvSize(uint32_t width, uint32_t height)
{
   mYuvWidth = width;
   mYuvHeight = height;
}

/*===========================================================================
 * FUNCTION   : QCamera3ReprocessChannel
 *
 * DESCRIPTION: constructor of QCamera3ReprocessChannel
 *
 * PARAMETERS :
 *   @cam_handle : camera handle
 *   @cam_ops    : ptr to camera ops table
 *   @pp_mask    : post-proccess feature mask
 *
 * RETURN     : none
 *==========================================================================*/
QCamera3ReprocessChannel::QCamera3ReprocessChannel(uint32_t cam_handle,
                                                 mm_camera_ops_t *cam_ops,
                                                 channel_cb_routine cb_routine,
                                                 cam_padding_info_t *paddingInfo,
                                                 uint32_t postprocess_mask,
                                                 void *userData, void *ch_hdl) :
    QCamera3Channel(cam_handle, cam_ops, cb_routine, paddingInfo, postprocess_mask,
                    userData, ((QCamera3PicChannel *)ch_hdl)->getNumBuffers()),
    picChHandle(ch_hdl),
    mOfflineBuffersIndex(-1),
    m_pSrcChannel(NULL),
    m_pMetaChannel(NULL),
    mMemory(NULL)
{
    memset(mSrcStreamHandles, 0, sizeof(mSrcStreamHandles));
    mOfflineMetaIndex = (int32_t) (mNumBuffers -1);
}


/*===========================================================================
 * FUNCTION   : QCamera3ReprocessChannel
 *
 * DESCRIPTION: constructor of QCamera3ReprocessChannel
 *
 * PARAMETERS :
 *   @cam_handle : camera handle
 *   @cam_ops    : ptr to camera ops table
 *   @pp_mask    : post-proccess feature mask
 *
 * RETURN     : none
 *==========================================================================*/
int32_t QCamera3ReprocessChannel::initialize(cam_is_type_t isType)
{
    int32_t rc = NO_ERROR;
    mm_camera_channel_attr_t attr;

    memset(&attr, 0, sizeof(mm_camera_channel_attr_t));
    attr.notify_mode = MM_CAMERA_SUPER_BUF_NOTIFY_CONTINUOUS;
    attr.max_unmatched_frames = 1;

    rc = init(&attr, NULL);
    if (rc < 0) {
        ALOGE("%s: init failed", __func__);
    }
    mIsType = isType;
    return rc;
}


/*===========================================================================
 * FUNCTION   : QCamera3ReprocessChannel
 *
 * DESCRIPTION: constructor of QCamera3ReprocessChannel
 *
 * PARAMETERS :
 *   @cam_handle : camera handle
 *   @cam_ops    : ptr to camera ops table
 *   @pp_mask    : post-proccess feature mask
 *
 * RETURN     : none
 *==========================================================================*/
void QCamera3ReprocessChannel::streamCbRoutine(mm_camera_super_buf_t *super_frame,
                                  QCamera3Stream *stream)
{
    //Got the pproc data callback. Now send to jpeg encoding
    uint8_t frameIndex;
    mm_camera_super_buf_t* frame = NULL;
    QCamera3PicChannel *obj = (QCamera3PicChannel *)picChHandle;
    cam_dimension_t dim;
    cam_frame_len_offset_t offset;

    memset(&dim, 0, sizeof(dim));
    memset(&offset, 0, sizeof(cam_frame_len_offset_t));

    if(!super_frame) {
         ALOGE("%s: Invalid Super buffer",__func__);
         return;
    }

    if(super_frame->num_bufs != 1) {
         ALOGE("%s: Multiple streams are not supported",__func__);
         return;
    }
    if(super_frame->bufs[0] == NULL ) {
         ALOGE("%s: Error, Super buffer frame does not contain valid buffer",
                  __func__);
         return;
    }

    frameIndex = (uint8_t)super_frame->bufs[0]->buf_idx;
    frame = (mm_camera_super_buf_t *)malloc(sizeof(mm_camera_super_buf_t));
    if (frame == NULL) {
       ALOGE("%s: Error allocating memory to save received_frame structure.",
                                                                    __func__);
       if(stream) {
           stream->bufDone(frameIndex);
       }
       return;
    }
    CDBG("%s: bufIndex: %u recvd from post proc",
        __func__, (uint32_t)frameIndex);
    *frame = *super_frame;
    stream->getFrameDimension(dim);
    stream->getFrameOffset(offset);
    dumpYUV(frame->bufs[0], dim, offset, QCAMERA_DUMP_FRM_OFFLINE_PROC);
    obj->m_postprocessor.processPPData(frame);
    free(super_frame);
    return;
}

/*===========================================================================
 * FUNCTION   : QCamera3ReprocessChannel
 *
 * DESCRIPTION: default constructor of QCamera3ReprocessChannel
 *
 * PARAMETERS : none
 *
 * RETURN     : none
 *==========================================================================*/
QCamera3ReprocessChannel::QCamera3ReprocessChannel() :
    m_pSrcChannel(NULL),
    m_pMetaChannel(NULL)
{
}

/*===========================================================================
 * FUNCTION   : getStreamBufs
 *
 * DESCRIPTION: register the buffers of the reprocess channel
 *
 * PARAMETERS : none
 *
 * RETURN     : QCamera3Memory *
 *==========================================================================*/
QCamera3Memory* QCamera3ReprocessChannel::getStreamBufs(uint32_t len)
{
   int rc = 0;

    mMemory = new QCamera3HeapMemory();
    if (!mMemory) {
        ALOGE("%s: unable to create reproc memory", __func__);
        return NULL;
    }

    rc = mMemory->allocate(mNumBuffers, len, true);
    if (rc < 0) {
        ALOGE("%s: unable to allocate reproc memory", __func__);
        delete mMemory;
        mMemory = NULL;
        return NULL;
    }
    return mMemory;
}

/*===========================================================================
 * FUNCTION   : getStreamBufs
 *
 * DESCRIPTION: register the buffers of the reprocess channel
 *
 * PARAMETERS : none
 *
 * RETURN     :
 *==========================================================================*/
void QCamera3ReprocessChannel::putStreamBufs()
{
    mMemory->deallocate();
    delete mMemory;
    mMemory = NULL;
}

/*===========================================================================
 * FUNCTION   : ~QCamera3ReprocessChannel
 *
 * DESCRIPTION: destructor of QCamera3ReprocessChannel
 *
 * PARAMETERS : none
 *
 * RETURN     : none
 *==========================================================================*/
QCamera3ReprocessChannel::~QCamera3ReprocessChannel()
{
}

/*===========================================================================
 * FUNCTION   : getStreamBySrcHandle
 *
 * DESCRIPTION: find reprocess stream by its source stream handle
 *
 * PARAMETERS :
 *   @srcHandle : source stream handle
 *
 * RETURN     : ptr to reprocess stream if found. NULL if not found
 *==========================================================================*/
QCamera3Stream * QCamera3ReprocessChannel::getStreamBySrcHandle(uint32_t srcHandle)
{
    QCamera3Stream *pStream = NULL;

    for (uint32_t i = 0; i < m_numStreams; i++) {
        if (mSrcStreamHandles[i] == srcHandle) {
            pStream = mStreams[i];
            break;
        }
    }
    return pStream;
}

/*===========================================================================
 * FUNCTION   : getSrcStreamBySrcHandle
 *
 * DESCRIPTION: find source stream by source stream handle
 *
 * PARAMETERS :
 *   @srcHandle : source stream handle
 *
 * RETURN     : ptr to reprocess stream if found. NULL if not found
 *==========================================================================*/
QCamera3Stream * QCamera3ReprocessChannel::getSrcStreamBySrcHandle(uint32_t srcHandle)
{
    QCamera3Stream *pStream = NULL;

    if (NULL == m_pSrcChannel) {
        return NULL;
    }

    for (uint32_t i = 0; i < m_numStreams; i++) {
        if (mSrcStreamHandles[i] == srcHandle) {
            pStream = m_pSrcChannel->getStreamByIndex(i);
            break;
        }
    }
    return pStream;
}

/*===========================================================================
 * FUNCTION   : stop
 *
 * DESCRIPTION: stop channel
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3ReprocessChannel::stop()
{
    unmapOfflineBuffers(true);

    return QCamera3Channel::stop();
}

/*===========================================================================
 * FUNCTION   : unmapOfflineBuffers
 *
 * DESCRIPTION: Unmaps offline buffers
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3ReprocessChannel::unmapOfflineBuffers(bool all)
{
    int rc = NO_ERROR;
    if (!mOfflineBuffers.empty()) {
        QCamera3Stream *stream = NULL;
        List<OfflineBuffer>::iterator it = mOfflineBuffers.begin();
        for (; it != mOfflineBuffers.end(); it++) {
           stream = (*it).stream;
           if (NULL != stream) {
               rc = stream->unmapBuf((*it).type,
                                     (*it).index,
                                        -1);
               if (NO_ERROR != rc) {
                   ALOGE("%s: Error during offline buffer unmap %d",
                         __func__, rc);
               }
               CDBG("%s: Unmapped buffer with index %d", __func__, (*it).index);
           }
           if (!all) {
               mOfflineBuffers.erase(it);
               break;
           }
        }
        if (all) {
           mOfflineBuffers.clear();
        }
    }

    if (!mOfflineMetaBuffers.empty()) {
        QCamera3Stream *stream = NULL;
        List<OfflineBuffer>::iterator it = mOfflineMetaBuffers.begin();
        for (; it != mOfflineMetaBuffers.end(); it++) {
           stream = (*it).stream;
           if (NULL != stream) {
               rc = stream->unmapBuf((*it).type,
                                     (*it).index,
                                        -1);
               if (NO_ERROR != rc) {
                   ALOGE("%s: Error during offline buffer unmap %d",
                         __func__, rc);
               }
               CDBG("%s: Unmapped meta buffer with index %d", __func__, (*it).index);
           }
           if (!all) {
               mOfflineMetaBuffers.erase(it);
               break;
           }
        }
        if (all) {
           mOfflineMetaBuffers.clear();
        }
    }
    return rc;
}


/*===========================================================================
 * FUNCTION   : extractFrameAndRotation
 *
 * DESCRIPTION: Extract output rotation and frame data if present
 *
 * PARAMETERS :
 *   @frame     : input frame from source stream
 *   meta_buffer: metadata buffer
 *   @metadata  : corresponding metadata
 *   @fwk_frame :
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3ReprocessChannel::extractFrameCropAndRotation(mm_camera_super_buf_t *frame,
        mm_camera_buf_def_t *meta_buffer, jpeg_settings_t *jpeg_settings,
        qcamera_fwk_input_pp_data_t &fwk_frame)
{
    int32_t rc = NO_ERROR;
    QCamera3HardwareInterface* hal_obj = (QCamera3HardwareInterface*)mUserData;
    if ((NULL == meta_buffer) || (NULL == frame) || (NULL == jpeg_settings) ||
            (NULL == hal_obj)) {
        return BAD_VALUE;
    }

    metadata_buffer_t *meta = (metadata_buffer_t *)meta_buffer->buffer;
    if (NULL == meta) {
        return BAD_VALUE;
    }

    for (uint32_t i = 0; i < frame->num_bufs; i++) {
        QCamera3Stream *pStream = getStreamBySrcHandle(frame->bufs[i]->stream_id);
        QCamera3Stream *pSrcStream = getSrcStreamBySrcHandle(frame->bufs[i]->stream_id);

        if (pStream != NULL && pSrcStream != NULL) {
            // Find rotation info for reprocess stream
            cam_rotation_info_t rotation_info;
            memset(&rotation_info, 0, sizeof(rotation_info));
            if (jpeg_settings->jpeg_orientation == 0) {
               rotation_info.rotation = ROTATE_0;
            } else if (jpeg_settings->jpeg_orientation == 90) {
               rotation_info.rotation = ROTATE_90;
            } else if (jpeg_settings->jpeg_orientation == 180) {
               rotation_info.rotation = ROTATE_180;
            } else if (jpeg_settings->jpeg_orientation == 270) {
               rotation_info.rotation = ROTATE_270;
            }
            rotation_info.streamId = mStreams[0]->getMyServerID();
            ADD_SET_PARAM_ENTRY_TO_BATCH(meta, CAM_INTF_PARM_ROTATION, rotation_info);

            // Find and insert crop info for reprocess stream
            IF_META_AVAILABLE(cam_crop_data_t, crop_data, CAM_INTF_META_CROP_DATA, meta) {
                if (MAX_NUM_STREAMS > crop_data->num_of_streams) {
                    for (int j = 0; j < crop_data->num_of_streams; j++) {
                        if (crop_data->crop_info[j].stream_id ==
                                pSrcStream->getMyServerID()) {

                            // Store crop/roi information for offline reprocess
                            // in the reprocess stream slot
                            crop_data->crop_info[crop_data->num_of_streams].crop =
                                    crop_data->crop_info[j].crop;
                            crop_data->crop_info[crop_data->num_of_streams].roi_map =
                                    crop_data->crop_info[j].roi_map;
                            crop_data->crop_info[crop_data->num_of_streams].stream_id =
                                    mStreams[0]->getMyServerID();
                            crop_data->num_of_streams++;

                            CDBG("%s: Reprocess stream server id: %d",
                                    __func__, mStreams[0]->getMyServerID());
                            CDBG("%s: Found offline reprocess crop %dx%d %dx%d",
                                    __func__,
                                    crop_data->crop_info[j].crop.left,
                                    crop_data->crop_info[j].crop.top,
                                    crop_data->crop_info[j].crop.width,
                                    crop_data->crop_info[j].crop.height);
                            CDBG("%s: Found offline reprocess roimap %dx%d %dx%d",
                                    __func__,
                                    crop_data->crop_info[j].roi_map.left,
                                    crop_data->crop_info[j].roi_map.top,
                                    crop_data->crop_info[j].roi_map.width,
                                    crop_data->crop_info[j].roi_map.height);

                            break;
                        }
                    }
                } else {
                    ALOGE("%s: No space to add reprocess stream crop/roi information",
                            __func__);
                }
            }

            fwk_frame.input_buffer = *frame->bufs[i];
            fwk_frame.metadata_buffer = *meta_buffer;
            break;
        } else {
            ALOGE("%s: Source/Re-process streams are invalid", __func__);
            rc |= BAD_VALUE;
        }
    }

    return rc;
}

/*===========================================================================
* FUNCTION : extractCrop
*
* DESCRIPTION: Extract framework output crop if present
*
* PARAMETERS :
* @frame : input frame for reprocessing
*
* RETURN : int32_t type of status
* NO_ERROR -- success
* none-zero failure code
*==========================================================================*/
int32_t QCamera3ReprocessChannel::extractCrop(qcamera_fwk_input_pp_data_t *frame)
{
    if (NULL == frame) {
        ALOGE("%s: Incorrect input frame", __func__);
        return BAD_VALUE;
    }


    if (NULL == frame->metadata_buffer.buffer) {
        ALOGE("%s: No metadata available", __func__);
        return BAD_VALUE;
    }

    // Find and insert crop info for reprocess stream
    metadata_buffer_t *meta = (metadata_buffer_t *) frame->metadata_buffer.buffer;
    IF_META_AVAILABLE(cam_crop_data_t, crop_data, CAM_INTF_META_CROP_DATA, meta) {
        if (1 == crop_data->num_of_streams) {
            // Store crop/roi information for offline reprocess
            // in the reprocess stream slot
            crop_data->crop_info[crop_data->num_of_streams].crop =
                    crop_data->crop_info[0].crop;
            crop_data->crop_info[crop_data->num_of_streams].roi_map =
                    crop_data->crop_info[0].roi_map;
            crop_data->crop_info[crop_data->num_of_streams].stream_id =
                    mStreams[0]->getMyServerID();
            crop_data->num_of_streams++;

            CDBG("%s: Reprocess stream server id: %d",
                    __func__, mStreams[0]->getMyServerID());
            CDBG("%s: Found offline reprocess crop %dx%d %dx%d", __func__,
                    crop_data->crop_info[0].crop.left,
                    crop_data->crop_info[0].crop.top,
                    crop_data->crop_info[0].crop.width,
                    crop_data->crop_info[0].crop.height);
            CDBG("%s: Found offline reprocess roi map %dx%d %dx%d", __func__,
                    crop_data->crop_info[0].roi_map.left,
                    crop_data->crop_info[0].roi_map.top,
                    crop_data->crop_info[0].roi_map.width,
                    crop_data->crop_info[0].roi_map.height);
        } else {
            ALOGE("%s: Incorrect number of offline crop data entries %d",
                    __func__,
                    crop_data->num_of_streams);
            return BAD_VALUE;
        }
    } else {
        CDBG_HIGH("%s: Crop data not present", __func__);
    }

    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : doReprocessOffline
 *
 * DESCRIPTION: request to do a reprocess on the frame
 *
 * PARAMETERS :
 *   @frame     : input frame for reprocessing
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
 int32_t QCamera3ReprocessChannel::doReprocessOffline(qcamera_fwk_input_pp_data_t *frame)
{
    int32_t rc = 0;
    OfflineBuffer mappedBuffer;

    if (m_numStreams < 1) {
        ALOGE("%s: No reprocess stream is created", __func__);
        return -1;
    }

    if (NULL == frame) {
        ALOGE("%s: Incorrect input frame", __func__);
        return BAD_VALUE;
    }

    if (NULL == frame->metadata_buffer.buffer) {
        ALOGE("%s: No metadata available", __func__);
        return BAD_VALUE;
    }

    if (NULL == frame->input_buffer.buffer) {
        ALOGE("%s: No input buffer available", __func__);
        return BAD_VALUE;
    }

    if ((0 == m_numStreams) || (NULL == mStreams[0])) {
        ALOGE("%s: Reprocess stream not initialized!", __func__);
        return NO_INIT;
    }

    QCamera3Stream *pStream = mStreams[0];
    int32_t max_idx = (int32_t) (mNumBuffers - 1);
    //loop back the indices if max burst count reached
    if (mOfflineBuffersIndex == max_idx) {
       mOfflineBuffersIndex = -1;
    }
    uint32_t buf_idx = (uint32_t)(mOfflineBuffersIndex + 1);
    rc = pStream->mapBuf(
            CAM_MAPPING_BUF_TYPE_OFFLINE_INPUT_BUF,
            buf_idx, -1,
            frame->input_buffer.fd, frame->input_buffer.frame_len);
    if (NO_ERROR == rc) {
        mappedBuffer.index = buf_idx;
        mappedBuffer.stream = pStream;
        mappedBuffer.type = CAM_MAPPING_BUF_TYPE_OFFLINE_INPUT_BUF;
        mOfflineBuffers.push_back(mappedBuffer);
        mOfflineBuffersIndex = (int32_t)buf_idx;
        CDBG("%s: Mapped buffer with index %d", __func__, mOfflineBuffersIndex);
    }

    max_idx = (int32_t) ((mNumBuffers * 2) - 1);
    //loop back the indices if max burst count reached
    if (mOfflineMetaIndex == max_idx) {
       mOfflineMetaIndex = (int32_t) (mNumBuffers - 1);
    }
    uint32_t meta_buf_idx = (uint32_t)(mOfflineMetaIndex + 1);
    rc |= pStream->mapBuf(
            CAM_MAPPING_BUF_TYPE_OFFLINE_META_BUF,
            meta_buf_idx, -1,
            frame->metadata_buffer.fd, frame->metadata_buffer.frame_len);
    if (NO_ERROR == rc) {
        mappedBuffer.index = meta_buf_idx;
        mappedBuffer.stream = pStream;
        mappedBuffer.type = CAM_MAPPING_BUF_TYPE_OFFLINE_META_BUF;
        mOfflineMetaBuffers.push_back(mappedBuffer);
        mOfflineMetaIndex = (int32_t)meta_buf_idx;
        CDBG("%s: Mapped meta buffer with index %d", __func__, mOfflineMetaIndex);
    }

    if (rc == NO_ERROR) {
        cam_stream_parm_buffer_t param;
        memset(&param, 0, sizeof(cam_stream_parm_buffer_t));
        param.type = CAM_STREAM_PARAM_TYPE_DO_REPROCESS;
        param.reprocess.buf_index = buf_idx;
        param.reprocess.frame_idx = frame->input_buffer.frame_idx;
        param.reprocess.meta_present = 1;
        param.reprocess.meta_buf_index = meta_buf_idx;
        rc = pStream->setParameter(param);
        if (rc != NO_ERROR) {
            ALOGE("%s: stream setParameter for reprocess failed", __func__);
        }
    } else {
        ALOGE("%s: Input buffer memory map failed: %d", __func__, rc);
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : doReprocess
 *
 * DESCRIPTION: request to do a reprocess on the frame
 *
 * PARAMETERS :
 *   @buf_fd     : fd to the input buffer that needs reprocess
 *   @buf_lenght : length of the input buffer
 *   @ret_val    : result of reprocess.
 *                 Example: Could be faceID in case of register face image.
 *   @meta_frame : metadata frame.
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3ReprocessChannel::doReprocess(int buf_fd, size_t buf_length,
        int32_t &ret_val, mm_camera_super_buf_t *meta_frame)
{
    int32_t rc = 0;
    if (m_numStreams < 1) {
        ALOGE("%s: No reprocess stream is created", __func__);
        return -1;
    }
    if (meta_frame == NULL) {
        ALOGE("%s: Did not get corresponding metadata in time", __func__);
        return -1;
    }

    uint8_t buf_idx = 0;
    for (uint32_t i = 0; i < m_numStreams; i++) {
        rc = mStreams[i]->mapBuf(CAM_MAPPING_BUF_TYPE_OFFLINE_INPUT_BUF,
                                 buf_idx, -1,
                                 buf_fd, buf_length);

        if (rc == NO_ERROR) {
            cam_stream_parm_buffer_t param;
            memset(&param, 0, sizeof(cam_stream_parm_buffer_t));
            param.type = CAM_STREAM_PARAM_TYPE_DO_REPROCESS;
            param.reprocess.buf_index = buf_idx;
            param.reprocess.meta_present = 1;
            param.reprocess.meta_stream_handle = m_pMetaChannel->mStreams[0]->getMyServerID();
            param.reprocess.meta_buf_index = meta_frame->bufs[0]->buf_idx;
            rc = mStreams[i]->setParameter(param);
            if (rc == NO_ERROR) {
                ret_val = param.reprocess.ret_val;
            }
            mStreams[i]->unmapBuf(CAM_MAPPING_BUF_TYPE_OFFLINE_INPUT_BUF,
                                  buf_idx, -1);
        }
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : addReprocStreamsFromSource
 *
 * DESCRIPTION: add reprocess streams from input source channel
 *
 * PARAMETERS :
 *   @config         : pp feature configuration
 *   @src_config     : source reprocess configuration
 *   @isType         : type of image stabilization required on this stream
 *   @pMetaChannel   : ptr to metadata channel to get corresp. metadata
 *
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3ReprocessChannel::addReprocStreamsFromSource(cam_pp_feature_config_t &pp_config,
        const reprocess_config_t &src_config , cam_is_type_t is_type,
        QCamera3Channel *pMetaChannel)
{
    int32_t rc = 0;
    cam_stream_reproc_config_t reprocess_config;
    cam_stream_type_t streamType;

    cam_dimension_t streamDim = src_config.output_stream_dim;

    if (NULL != src_config.src_channel) {
        QCamera3Stream *pSrcStream = src_config.src_channel->getStreamByIndex(0);
        if (pSrcStream == NULL) {
           ALOGE("%s: source channel doesn't have a stream", __func__);
           return BAD_VALUE;
        }
        mSrcStreamHandles[m_numStreams] = pSrcStream->getMyHandle();
    }

    streamType = CAM_STREAM_TYPE_OFFLINE_PROC;
    reprocess_config.pp_type = CAM_OFFLINE_REPROCESS_TYPE;

    reprocess_config.offline.input_fmt = src_config.stream_format;
    reprocess_config.offline.input_dim = src_config.input_stream_dim;
    reprocess_config.offline.input_buf_planes.plane_info =
            src_config.input_stream_plane_info.plane_info;
    reprocess_config.offline.num_of_bufs = (uint8_t)mNumBuffers;
    reprocess_config.offline.input_type = src_config.stream_type;

    reprocess_config.pp_feature_config = pp_config;
    QCamera3Stream *pStream = new QCamera3Stream(m_camHandle,
            m_handle,
            m_camOps,
            &mPaddingInfo,
            (QCamera3Channel*)this);
    if (pStream == NULL) {
        ALOGE("%s: No mem for Stream", __func__);
        return NO_MEMORY;
    }

    rc = pStream->init(streamType, src_config.stream_format,
            streamDim, &reprocess_config,
            (uint8_t)mNumBuffers,
            reprocess_config.pp_feature_config.feature_mask,
            is_type,
            QCamera3Channel::streamCbRoutine, this);

    if (rc == 0) {
        mStreams[m_numStreams] = pStream;
        m_numStreams++;
    } else {
        ALOGE("%s: failed to create reprocess stream", __func__);
        delete pStream;
    }

    if (rc == NO_ERROR) {
        m_pSrcChannel = src_config.src_channel;
        m_pMetaChannel = pMetaChannel;
    }
    mm_camera_req_buf_t buf;
    memset(&buf, 0x0, sizeof(buf));
    buf.type = MM_CAMERA_REQ_SUPER_BUF;
    buf.num_buf_requested = 1;
    if(m_camOps->request_super_buf(m_camHandle,m_handle, &buf) < 0) {
        ALOGE("%s: Request for super buffer failed",__func__);
    }
    return rc;
}

cam_dimension_t QCamera3SupportChannel::kDim = {640, 480};

QCamera3SupportChannel::QCamera3SupportChannel(uint32_t cam_handle,
                    mm_camera_ops_t *cam_ops,
                    cam_padding_info_t *paddingInfo,
                    uint32_t postprocess_mask,
                    cam_stream_type_t streamType,
                    cam_dimension_t *dim,
                    cam_format_t streamFormat,
                    void *userData, uint32_t numBuffers) :
                        QCamera3Channel(cam_handle, cam_ops,
                                NULL, paddingInfo, postprocess_mask,
                                userData, numBuffers),
                        mMemory(NULL)
{
    memcpy(&mDim, dim, sizeof(cam_dimension_t));
    mStreamType = streamType;
    mStreamFormat = streamFormat;
}

QCamera3SupportChannel::~QCamera3SupportChannel()
{
    if (m_bIsActive)
        stop();

    if (mMemory) {
        mMemory->deallocate();
        delete mMemory;
        mMemory = NULL;
    }
}

int32_t QCamera3SupportChannel::initialize(cam_is_type_t isType)
{
    int32_t rc;

    if (mMemory || m_numStreams > 0) {
        ALOGE("%s: metadata channel already initialized", __func__);
        return -EINVAL;
    }

    rc = init(NULL, NULL);
    if (rc < 0) {
        ALOGE("%s: init failed", __func__);
        return rc;
    }
    mIsType = isType;
    rc = QCamera3Channel::addStream(mStreamType,
        mStreamFormat, mDim, MIN_STREAMING_BUFFER_NUM,
        mPostProcMask, mIsType);
    if (rc < 0) {
        ALOGE("%s: addStream failed", __func__);
    }
    return rc;
}

int32_t QCamera3SupportChannel::request(buffer_handle_t * /*buffer*/,
                                                uint32_t /*frameNumber*/)
{
    return NO_ERROR;
}

void QCamera3SupportChannel::streamCbRoutine(
                        mm_camera_super_buf_t *super_frame,
                        QCamera3Stream * /*stream*/)
{
    if (super_frame == NULL || super_frame->num_bufs != 1) {
        ALOGE("%s: super_frame is not valid", __func__);
        return;
    }
    bufDone(super_frame);
    free(super_frame);
}

QCamera3Memory* QCamera3SupportChannel::getStreamBufs(uint32_t len)
{
    int rc;

    mMemory = new QCamera3HeapMemory();
    if (!mMemory) {
        ALOGE("%s: unable to create heap memory", __func__);
        return NULL;
    }
    rc = mMemory->allocate(MIN_STREAMING_BUFFER_NUM, len, true);
    if (rc < 0) {
        ALOGE("%s: unable to allocate heap memory", __func__);
        delete mMemory;
        mMemory = NULL;
        return NULL;
    }
    return mMemory;
}

void QCamera3SupportChannel::putStreamBufs()
{
    mMemory->deallocate();
    delete mMemory;
    mMemory = NULL;
}

}; // namespace qcamera
