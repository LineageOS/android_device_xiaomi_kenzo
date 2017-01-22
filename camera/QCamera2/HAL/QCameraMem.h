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

#ifndef __QCAMERA2HWI_MEM_H__
#define __QCAMERA2HWI_MEM_H__

#include <hardware/camera.h>
#include <utils/Mutex.h>
#include <utils/List.h>
#include <utils/Timers.h>

//Media depedancies
#include "OMX_QCOMExtns.h"
//Display dependencies
#include <qdMetaData.h>

extern "C" {
#include <sys/types.h>
#include <linux/msm_ion.h>
#include <mm_camera_interface.h>
}

//OFFSET, SIZE, USAGE, TIMESTAMP, FORMAT
#define VIDEO_METADATA_NUM_INTS 5

#ifdef USE_MEDIA_EXTENSIONS
//Note that this macro might have already been
//defined in OMX_QCOMExtns.h, in which case
//the local value below will not be used.
#ifndef VIDEO_METADATA_NUM_COMMON_INTS
#define VIDEO_METADATA_NUM_COMMON_INTS   1
#endif
#endif

namespace qcamera {

class QCameraMemoryPool;

// Base class for all memory types. Abstract.
class QCameraMemory {

public:
    int cleanCache(uint32_t index)
    {
        return cacheOps(index, ION_IOC_CLEAN_CACHES);
    }
    int invalidateCache(uint32_t index)
    {
        return cacheOps(index, ION_IOC_INV_CACHES);
    }
    int cleanInvalidateCache(uint32_t index)
    {
        return cacheOps(index, ION_IOC_CLEAN_INV_CACHES);
    }
    int getFd(uint32_t index) const;
    ssize_t getSize(uint32_t index) const;
    uint8_t getCnt() const;
    virtual uint8_t getMappable() const;

    virtual int allocate(uint8_t count, size_t size, uint32_t is_secure) = 0;
    virtual void deallocate() = 0;
    virtual int allocateMore(uint8_t count, size_t size) = 0;
    virtual int cacheOps(uint32_t index, unsigned int cmd) = 0;
    virtual int getRegFlags(uint8_t *regFlags) const = 0;
    virtual camera_memory_t *getMemory(uint32_t index,
            bool metadata) const = 0;
    virtual int getMatchBufIndex(const void *opaque, bool metadata) const = 0;
    virtual void *getPtr(uint32_t index) const= 0;

    QCameraMemory(bool cached,
                  QCameraMemoryPool *pool = NULL,
                  cam_stream_type_t streamType = CAM_STREAM_TYPE_DEFAULT,
                  cam_stream_buf_type buf_Type = CAM_STREAM_BUF_TYPE_MPLANE);
    virtual ~QCameraMemory();
    virtual void reset();

    void getBufDef(const cam_frame_len_offset_t &offset,
            mm_camera_buf_def_t &bufDef, uint32_t index) const;

    int32_t getUserBufDef(const cam_stream_user_buf_info_t &buf_info,
            mm_camera_buf_def_t &bufDef, uint32_t index,
            const cam_frame_len_offset_t &plane_offset,
            mm_camera_buf_def_t *planebufDef, QCameraMemory *bufs) const;

protected:

    friend class QCameraMemoryPool;

    struct QCameraMemInfo {
        int fd;
        int main_ion_fd;
        ion_user_handle_t handle;
        size_t size;
        bool cached;
        unsigned int heap_id;
    };

    int alloc(int count, size_t size, unsigned int heap_id,
            uint32_t is_secure);
    void dealloc();
    static int allocOneBuffer(struct QCameraMemInfo &memInfo,
            unsigned int heap_id, size_t size, bool cached, uint32_t is_secure);
    static void deallocOneBuffer(struct QCameraMemInfo &memInfo);
    int cacheOpsInternal(uint32_t index, unsigned int cmd, void *vaddr);

    bool m_bCached;
    uint8_t mBufferCount;
    struct QCameraMemInfo mMemInfo[MM_CAMERA_MAX_NUM_FRAMES];
    QCameraMemoryPool *mMemoryPool;
    cam_stream_type_t mStreamType;
    cam_stream_buf_type mBufType;
};

class QCameraMemoryPool {

public:

    QCameraMemoryPool();
    virtual ~QCameraMemoryPool();

    int allocateBuffer(struct QCameraMemory::QCameraMemInfo &memInfo,
            unsigned int heap_id, size_t size, bool cached,
            cam_stream_type_t streamType, uint32_t is_secure);
    void releaseBuffer(struct QCameraMemory::QCameraMemInfo &memInfo,
            cam_stream_type_t streamType);
    void clear();

protected:

    int findBufferLocked(struct QCameraMemory::QCameraMemInfo &memInfo,
            unsigned int heap_id, size_t size, bool cached,
            cam_stream_type_t streamType);

    android::List<QCameraMemory::QCameraMemInfo> mPools[CAM_STREAM_TYPE_MAX];
    pthread_mutex_t mLock;
};

// Internal heap memory is used for memories used internally
// They are allocated from /dev/ion.
class QCameraHeapMemory : public QCameraMemory {
public:
    QCameraHeapMemory(bool cached);
    virtual ~QCameraHeapMemory();

    virtual int allocate(uint8_t count, size_t size, uint32_t is_secure);
    virtual int allocateMore(uint8_t count, size_t size);
    virtual void deallocate();
    virtual int cacheOps(uint32_t index, unsigned int cmd);
    virtual int getRegFlags(uint8_t *regFlags) const;
    virtual camera_memory_t *getMemory(uint32_t index, bool metadata) const;
    virtual int getMatchBufIndex(const void *opaque, bool metadata) const;
    virtual void *getPtr(uint32_t index) const;

private:
    void *mPtr[MM_CAMERA_MAX_NUM_FRAMES];
};

class QCameraMetadataStreamMemory : public QCameraHeapMemory {
public:
    QCameraMetadataStreamMemory(bool cached);
    virtual ~QCameraMetadataStreamMemory();

    virtual int getRegFlags(uint8_t *regFlags) const;
};

// Externel heap memory is used for memories shared with
// framework. They are allocated from /dev/ion or gralloc.
class QCameraStreamMemory : public QCameraMemory {
public:
    QCameraStreamMemory(camera_request_memory getMemory,
                        bool cached,
                        QCameraMemoryPool *pool = NULL,
                        cam_stream_type_t streamType = CAM_STREAM_TYPE_DEFAULT,
                        cam_stream_buf_type buf_Type = CAM_STREAM_BUF_TYPE_MPLANE);
    virtual ~QCameraStreamMemory();

    virtual int allocate(uint8_t count, size_t size, uint32_t is_secure);
    virtual int allocateMore(uint8_t count, size_t size);
    virtual void deallocate();
    virtual int cacheOps(uint32_t index, unsigned int cmd);
    virtual int getRegFlags(uint8_t *regFlags) const;
    virtual camera_memory_t *getMemory(uint32_t index, bool metadata) const;
    virtual int getMatchBufIndex(const void *opaque, bool metadata) const;
    virtual void *getPtr(uint32_t index) const;

protected:
    camera_request_memory mGetMemory;
    camera_memory_t *mCameraMemory[MM_CAMERA_MAX_NUM_FRAMES];
};

// Externel heap memory is used for memories shared with
// framework. They are allocated from /dev/ion or gralloc.
class QCameraVideoMemory : public QCameraStreamMemory {
public:
    QCameraVideoMemory(camera_request_memory getMemory, bool cached,
            cam_stream_buf_type bufType = CAM_STREAM_BUF_TYPE_MPLANE);
    virtual ~QCameraVideoMemory();

    virtual int allocate(uint8_t count, size_t size, uint32_t is_secure);
    virtual int allocateMore(uint8_t count, size_t size);
    virtual void deallocate();
    virtual camera_memory_t *getMemory(uint32_t index, bool metadata) const;
    virtual int getMatchBufIndex(const void *opaque, bool metadata) const;
    int allocateMeta(uint8_t buf_cnt);
    void deallocateMeta();
    void setVideoInfo(int usage, cam_format_t format);
    int convCamtoOMXFormat(cam_format_t format);
    int getUsage(){return mUsage;};
    int getFormat(){return mFormat;};
#ifdef USE_MEDIA_EXTENSIONS
    native_handle_t *getNativeHandle(uint32_t index, bool metadata = true);
    static int closeNativeHandle(const void *data);
    int closeNativeHandle(const void *data, bool metadata);
#endif
private:
    camera_memory_t *mMetadata[MM_CAMERA_MAX_NUM_FRAMES];
    uint8_t mMetaBufCount;
    int mUsage, mFormat;
#ifdef USE_MEDIA_EXTENSIONS
    native_handle_t *mNativeHandle[MM_CAMERA_MAX_NUM_FRAMES];
#endif
};


// Gralloc Memory is acquired from preview window
class QCameraGrallocMemory : public QCameraMemory {
    enum {
        BUFFER_NOT_OWNED,
        BUFFER_OWNED,
    };
public:
    QCameraGrallocMemory(camera_request_memory getMemory);
    void setNativeWindow(preview_stream_ops_t *anw);
    virtual ~QCameraGrallocMemory();

    virtual int allocate(uint8_t count, size_t size, uint32_t is_secure);
    virtual int allocateMore(uint8_t count, size_t size);
    virtual void deallocate();
    virtual int cacheOps(uint32_t index, unsigned int cmd);
    virtual int getRegFlags(uint8_t *regFlags) const;
    virtual camera_memory_t *getMemory(uint32_t index, bool metadata) const;
    virtual int getMatchBufIndex(const void *opaque, bool metadata) const;
    virtual void *getPtr(uint32_t index) const;
    virtual void setMappable(uint8_t mappable);
    virtual uint8_t getMappable() const;

    void setWindowInfo(preview_stream_ops_t *window, int width, int height,
        int stride, int scanline, int format, int maxFPS, int usage = 0);
    // Enqueue/display buffer[index] onto the native window,
    // and dequeue one buffer from it.
    // Returns the buffer index of the dequeued buffer.
    int displayBuffer(uint32_t index);
    void setMaxFPS(int maxFPS);
    int32_t enqueueBuffer(uint32_t index, nsecs_t timeStamp = 0);
    int32_t dequeueBuffer();

private:
    buffer_handle_t *mBufferHandle[MM_CAMERA_MAX_NUM_FRAMES];
    int mLocalFlag[MM_CAMERA_MAX_NUM_FRAMES];
    struct private_handle_t *mPrivateHandle[MM_CAMERA_MAX_NUM_FRAMES];
    preview_stream_ops_t *mWindow;
    int mWidth, mHeight, mFormat, mStride, mScanline, mUsage, mMaxFPS;
    camera_request_memory mGetMemory;
    camera_memory_t *mCameraMemory[MM_CAMERA_MAX_NUM_FRAMES];
    int mMinUndequeuedBuffers;
    enum ColorSpace_t mColorSpace;
    uint8_t mMappableBuffers;
    pthread_mutex_t mLock;
    uint8_t mEnqueuedBuffers;
};

}; // namespace qcamera

#endif /* __QCAMERA2HWI_MEM_H__ */
