/* Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
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

#include <ctype.h>
#include <cutils/properties.h>
#include <fcntl.h>
#include <dlfcn.h>
#include <linux/msm_ion.h>
#include <sys/mman.h>

#include "mm_qcamera_dbg.h"
#include "mm_qcamera_app.h"

static pthread_mutex_t app_mutex;
static int thread_status = 0;
static pthread_cond_t app_cond_v;

#define MM_QCAMERA_APP_NANOSEC_SCALE 1000000000

int mm_camera_app_timedwait(uint8_t seconds)
{
    int rc = 0;
    pthread_mutex_lock(&app_mutex);
    if(FALSE == thread_status) {
        struct timespec tw;
        memset(&tw, 0, sizeof tw);
        tw.tv_sec = 0;
        tw.tv_nsec = time(0) + seconds * MM_QCAMERA_APP_NANOSEC_SCALE;

        rc = pthread_cond_timedwait(&app_cond_v, &app_mutex,&tw);
        thread_status = FALSE;
    }
    pthread_mutex_unlock(&app_mutex);
    return rc;
}

int mm_camera_app_wait()
{
    int rc = 0;
    pthread_mutex_lock(&app_mutex);
    if(FALSE == thread_status){
        pthread_cond_wait(&app_cond_v, &app_mutex);
    }
    thread_status = FALSE;
    pthread_mutex_unlock(&app_mutex);
    return rc;
}

void mm_camera_app_done()
{
  pthread_mutex_lock(&app_mutex);
  thread_status = TRUE;
  pthread_cond_signal(&app_cond_v);
  pthread_mutex_unlock(&app_mutex);
}

int mm_app_load_hal(mm_camera_app_t *my_cam_app)
{
    memset(&my_cam_app->hal_lib, 0, sizeof(hal_interface_lib_t));
    my_cam_app->hal_lib.ptr = dlopen("libmmcamera_interface.so", RTLD_NOW);
    my_cam_app->hal_lib.ptr_jpeg = dlopen("libmmjpeg_interface.so", RTLD_NOW);
    if (!my_cam_app->hal_lib.ptr || !my_cam_app->hal_lib.ptr_jpeg) {
        CDBG_ERROR("%s Error opening HAL library %s\n", __func__, dlerror());
        return -MM_CAMERA_E_GENERAL;
    }
    *(void **)&(my_cam_app->hal_lib.get_num_of_cameras) =
        dlsym(my_cam_app->hal_lib.ptr, "get_num_of_cameras");
    *(void **)&(my_cam_app->hal_lib.mm_camera_open) =
        dlsym(my_cam_app->hal_lib.ptr, "camera_open");
    *(void **)&(my_cam_app->hal_lib.jpeg_open) =
        dlsym(my_cam_app->hal_lib.ptr_jpeg, "jpeg_open");

    if (my_cam_app->hal_lib.get_num_of_cameras == NULL ||
        my_cam_app->hal_lib.mm_camera_open == NULL ||
        my_cam_app->hal_lib.jpeg_open == NULL) {
        CDBG_ERROR("%s Error loading HAL sym %s\n", __func__, dlerror());
        return -MM_CAMERA_E_GENERAL;
    }

    my_cam_app->num_cameras = my_cam_app->hal_lib.get_num_of_cameras();
    CDBG("%s: num_cameras = %d\n", __func__, my_cam_app->num_cameras);

    return MM_CAMERA_OK;
}

int mm_app_allocate_ion_memory(mm_camera_app_buf_t *buf, unsigned int ion_type)
{
    int rc = MM_CAMERA_OK;
    struct ion_handle_data handle_data;
    struct ion_allocation_data alloc;
    struct ion_fd_data ion_info_fd;
    int main_ion_fd = -1;
    void *data = NULL;

    main_ion_fd = open("/dev/ion", O_RDONLY);
    if (main_ion_fd <= 0) {
        CDBG_ERROR("Ion dev open failed %s\n", strerror(errno));
        goto ION_OPEN_FAILED;
    }

    memset(&alloc, 0, sizeof(alloc));
    alloc.len = buf->mem_info.size;
    /* to make it page size aligned */
    alloc.len = (alloc.len + 4095U) & (~4095U);
    alloc.align = 4096;
    alloc.flags = ION_FLAG_CACHED;
    alloc.heap_id_mask = ION_HEAP(ION_SYSTEM_HEAP_ID);
    rc = ioctl(main_ion_fd, ION_IOC_ALLOC, &alloc);
    if (rc < 0) {
        CDBG_ERROR("ION allocation failed %s with rc = %d \n",strerror(errno), rc);
        goto ION_ALLOC_FAILED;
    }

    memset(&ion_info_fd, 0, sizeof(ion_info_fd));
    ion_info_fd.handle = alloc.handle;
    rc = ioctl(main_ion_fd, ION_IOC_SHARE, &ion_info_fd);
    if (rc < 0) {
        CDBG_ERROR("ION map failed %s\n", strerror(errno));
        goto ION_MAP_FAILED;
    }

    data = mmap(NULL,
                alloc.len,
                PROT_READ  | PROT_WRITE,
                MAP_SHARED,
                ion_info_fd.fd,
                0);

    if (data == MAP_FAILED) {
        CDBG_ERROR("ION_MMAP_FAILED: %s (%d)\n", strerror(errno), errno);
        goto ION_MAP_FAILED;
    }
    buf->mem_info.main_ion_fd = main_ion_fd;
    buf->mem_info.fd = ion_info_fd.fd;
    buf->mem_info.handle = ion_info_fd.handle;
    buf->mem_info.size = alloc.len;
    buf->mem_info.data = data;
    return MM_CAMERA_OK;

ION_MAP_FAILED:
    memset(&handle_data, 0, sizeof(handle_data));
    handle_data.handle = ion_info_fd.handle;
    ioctl(main_ion_fd, ION_IOC_FREE, &handle_data);
ION_ALLOC_FAILED:
    close(main_ion_fd);
ION_OPEN_FAILED:
    return -MM_CAMERA_E_GENERAL;
}

int mm_app_deallocate_ion_memory(mm_camera_app_buf_t *buf)
{
  struct ion_handle_data handle_data;
  int rc = 0;

  rc = munmap(buf->mem_info.data, buf->mem_info.size);

  if (buf->mem_info.fd >= 0) {
      close(buf->mem_info.fd);
      buf->mem_info.fd = -1;
  }

  if (buf->mem_info.main_ion_fd >= 0) {
      memset(&handle_data, 0, sizeof(handle_data));
      handle_data.handle = buf->mem_info.handle;
      ioctl(buf->mem_info.main_ion_fd, ION_IOC_FREE, &handle_data);
      close(buf->mem_info.main_ion_fd);
      buf->mem_info.main_ion_fd = -1;
  }
  return rc;
}

/* cmd = ION_IOC_CLEAN_CACHES, ION_IOC_INV_CACHES, ION_IOC_CLEAN_INV_CACHES */
int mm_app_cache_ops(mm_camera_app_meminfo_t *mem_info,
                     int cmd)
{
    struct ion_flush_data cache_inv_data;
    struct ion_custom_data custom_data;
    int ret = MM_CAMERA_OK;

#ifdef USE_ION
    if (NULL == mem_info) {
        CDBG_ERROR("%s: mem_info is NULL, return here", __func__);
        return -MM_CAMERA_E_GENERAL;
    }

    memset(&cache_inv_data, 0, sizeof(cache_inv_data));
    memset(&custom_data, 0, sizeof(custom_data));
    cache_inv_data.vaddr = mem_info->data;
    cache_inv_data.fd = mem_info->fd;
    cache_inv_data.handle = mem_info->handle;
    cache_inv_data.length = (unsigned int)mem_info->size;
    custom_data.cmd = (unsigned int)cmd;
    custom_data.arg = (unsigned long)&cache_inv_data;

    CDBG("addr = %p, fd = %d, handle = %lx length = %d, ION Fd = %d",
         cache_inv_data.vaddr, cache_inv_data.fd,
         (unsigned long)cache_inv_data.handle, cache_inv_data.length,
         mem_info->main_ion_fd);
    if(mem_info->main_ion_fd >= 0) {
        if(ioctl(mem_info->main_ion_fd, ION_IOC_CUSTOM, &custom_data) < 0) {
            ALOGE("%s: Cache Invalidate failed\n", __func__);
            ret = -MM_CAMERA_E_GENERAL;
        }
    }
#endif

    return ret;
}

void mm_app_dump_frame(mm_camera_buf_def_t *frame,
                       char *name,
                       char *ext,
                       uint32_t frame_idx)
{
    char file_name[FILENAME_MAX];
    int file_fd;
    int i;
    int offset = 0;
    if ( frame != NULL) {
        snprintf(file_name, sizeof(file_name),
                QCAMERA_DUMP_FRM_LOCATION"%s_%04d.%s", name, frame_idx, ext);
        file_fd = open(file_name, O_RDWR | O_CREAT, 0777);
        if (file_fd < 0) {
            CDBG_ERROR("%s: cannot open file %s \n", __func__, file_name);
        } else {
            for (i = 0; i < frame->planes_buf.num_planes; i++) {
                CDBG("%s: saving file from address: %p, data offset: %d, "
                     "length: %d \n", __func__, frame->buffer,
                    frame->planes_buf.planes[i].data_offset, frame->planes_buf.planes[i].length);
                write(file_fd,
                      (uint8_t *)frame->buffer + offset,
                      frame->planes_buf.planes[i].length);
                offset += (int)frame->planes_buf.planes[i].length;
            }

            close(file_fd);
            CDBG("dump %s", file_name);
        }
    }
}

void mm_app_dump_jpeg_frame(const void * data, size_t size, char* name,
        char* ext, uint32_t index)
{
    char buf[FILENAME_MAX];
    int file_fd;
    if ( data != NULL) {
        snprintf(buf, sizeof(buf),
                QCAMERA_DUMP_FRM_LOCATION"test/%s_%u.%s", name, index, ext);
        CDBG("%s: %s size =%zu, jobId=%u", __func__, buf, size, index);
        file_fd = open(buf, O_RDWR | O_CREAT, 0777);
        write(file_fd, data, size);
        close(file_fd);
    }
}

int mm_app_alloc_bufs(mm_camera_app_buf_t* app_bufs,
                      cam_frame_len_offset_t *frame_offset_info,
                      uint8_t num_bufs,
                      uint8_t is_streambuf,
                      size_t multipleOf)
{
    uint32_t i, j;
    unsigned int ion_type = 0x1 << CAMERA_ION_FALLBACK_HEAP_ID;

    if (is_streambuf) {
        ion_type |= 0x1 << CAMERA_ION_HEAP_ID;
    }

    for (i = 0; i < num_bufs ; i++) {
        if ( 0 < multipleOf ) {
            size_t m = frame_offset_info->frame_len / multipleOf;
            if ( ( frame_offset_info->frame_len % multipleOf ) != 0 ) {
                m++;
            }
            app_bufs[i].mem_info.size = m * multipleOf;
        } else {
            app_bufs[i].mem_info.size = frame_offset_info->frame_len;
        }
        mm_app_allocate_ion_memory(&app_bufs[i], ion_type);

        app_bufs[i].buf.buf_idx = i;
        app_bufs[i].buf.planes_buf.num_planes = (int8_t)frame_offset_info->num_planes;
        app_bufs[i].buf.fd = app_bufs[i].mem_info.fd;
        app_bufs[i].buf.frame_len = app_bufs[i].mem_info.size;
        app_bufs[i].buf.buffer = app_bufs[i].mem_info.data;
        app_bufs[i].buf.mem_info = (void *)&app_bufs[i].mem_info;

        /* Plane 0 needs to be set seperately. Set other planes
             * in a loop. */
        app_bufs[i].buf.planes_buf.planes[0].length = frame_offset_info->mp[0].len;
        app_bufs[i].buf.planes_buf.planes[0].m.userptr =
            (long unsigned int)app_bufs[i].buf.fd;
        app_bufs[i].buf.planes_buf.planes[0].data_offset = frame_offset_info->mp[0].offset;
        app_bufs[i].buf.planes_buf.planes[0].reserved[0] = 0;
        for (j = 1; j < (uint8_t)frame_offset_info->num_planes; j++) {
            app_bufs[i].buf.planes_buf.planes[j].length = frame_offset_info->mp[j].len;
            app_bufs[i].buf.planes_buf.planes[j].m.userptr =
                (long unsigned int)app_bufs[i].buf.fd;
            app_bufs[i].buf.planes_buf.planes[j].data_offset = frame_offset_info->mp[j].offset;
            app_bufs[i].buf.planes_buf.planes[j].reserved[0] =
                app_bufs[i].buf.planes_buf.planes[j-1].reserved[0] +
                app_bufs[i].buf.planes_buf.planes[j-1].length;
        }
    }
    CDBG("%s: X", __func__);
    return MM_CAMERA_OK;
}

int mm_app_release_bufs(uint8_t num_bufs,
                        mm_camera_app_buf_t* app_bufs)
{
    int i, rc = MM_CAMERA_OK;

    CDBG("%s: E", __func__);

    for (i = 0; i < num_bufs; i++) {
        rc = mm_app_deallocate_ion_memory(&app_bufs[i]);
    }
    memset(app_bufs, 0, num_bufs * sizeof(mm_camera_app_buf_t));
    CDBG("%s: X", __func__);
    return rc;
}

int mm_app_stream_initbuf(cam_frame_len_offset_t *frame_offset_info,
                          uint8_t *num_bufs,
                          uint8_t **initial_reg_flag,
                          mm_camera_buf_def_t **bufs,
                          mm_camera_map_unmap_ops_tbl_t *ops_tbl,
                          void *user_data)
{
    mm_camera_stream_t *stream = (mm_camera_stream_t *)user_data;
    mm_camera_buf_def_t *pBufs = NULL;
    uint8_t *reg_flags = NULL;
    int i, rc;

    stream->offset = *frame_offset_info;

    CDBG("%s: alloc buf for stream_id %d, len=%d, num planes: %d, offset: %d",
         __func__,
         stream->s_id,
         frame_offset_info->frame_len,
         frame_offset_info->num_planes,
         frame_offset_info->mp[1].offset);

    if (stream->num_of_bufs > CAM_MAX_NUM_BUFS_PER_STREAM)
        stream->num_of_bufs = CAM_MAX_NUM_BUFS_PER_STREAM;

    pBufs = (mm_camera_buf_def_t *)malloc(sizeof(mm_camera_buf_def_t) * stream->num_of_bufs);
    reg_flags = (uint8_t *)malloc(sizeof(uint8_t) * stream->num_of_bufs);
    if (pBufs == NULL || reg_flags == NULL) {
        CDBG_ERROR("%s: No mem for bufs", __func__);
        if (pBufs != NULL) {
            free(pBufs);
        }
        if (reg_flags != NULL) {
            free(reg_flags);
        }
        return -1;
    }

    rc = mm_app_alloc_bufs(&stream->s_bufs[0],
                           frame_offset_info,
                           stream->num_of_bufs,
                           1,
                           stream->multipleOf);

    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: mm_stream_alloc_bufs err = %d", __func__, rc);
        free(pBufs);
        free(reg_flags);
        return rc;
    }

    for (i = 0; i < stream->num_of_bufs; i++) {
        /* mapping stream bufs first */
        pBufs[i] = stream->s_bufs[i].buf;
        reg_flags[i] = 1;
        rc = ops_tbl->map_ops(pBufs[i].buf_idx,
                              -1,
                              pBufs[i].fd,
                              (uint32_t)pBufs[i].frame_len,
                              CAM_MAPPING_BUF_TYPE_STREAM_BUF, ops_tbl->userdata);
        if (rc != MM_CAMERA_OK) {
            CDBG_ERROR("%s: mapping buf[%d] err = %d", __func__, i, rc);
            break;
        }
    }

    if (rc != MM_CAMERA_OK) {
        int j;
        for (j=0; j>i; j++) {
            ops_tbl->unmap_ops(pBufs[j].buf_idx, -1,
                    CAM_MAPPING_BUF_TYPE_STREAM_BUF, ops_tbl->userdata);
        }
        mm_app_release_bufs(stream->num_of_bufs, &stream->s_bufs[0]);
        free(pBufs);
        free(reg_flags);
        return rc;
    }

    *num_bufs = stream->num_of_bufs;
    *bufs = pBufs;
    *initial_reg_flag = reg_flags;

    CDBG("%s: X",__func__);
    return rc;
}

int32_t mm_app_stream_deinitbuf(mm_camera_map_unmap_ops_tbl_t *ops_tbl,
                                void *user_data)
{
    mm_camera_stream_t *stream = (mm_camera_stream_t *)user_data;
    int i;

    for (i = 0; i < stream->num_of_bufs ; i++) {
        /* mapping stream bufs first */
        ops_tbl->unmap_ops(stream->s_bufs[i].buf.buf_idx, -1,
                CAM_MAPPING_BUF_TYPE_STREAM_BUF, ops_tbl->userdata);
    }

    mm_app_release_bufs(stream->num_of_bufs, &stream->s_bufs[0]);

    CDBG("%s: X",__func__);
    return 0;
}

int32_t mm_app_stream_clean_invalidate_buf(uint32_t index, void *user_data)
{
    mm_camera_stream_t *stream = (mm_camera_stream_t *)user_data;
    return mm_app_cache_ops(&stream->s_bufs[index].mem_info,
      ION_IOC_CLEAN_INV_CACHES);
}

int32_t mm_app_stream_invalidate_buf(uint32_t index, void *user_data)
{
    mm_camera_stream_t *stream = (mm_camera_stream_t *)user_data;
    return mm_app_cache_ops(&stream->s_bufs[index].mem_info, ION_IOC_INV_CACHES);
}

static void notify_evt_cb(uint32_t camera_handle,
                          mm_camera_event_t *evt,
                          void *user_data)
{
    mm_camera_test_obj_t *test_obj =
        (mm_camera_test_obj_t *)user_data;
    if (test_obj == NULL || test_obj->cam->camera_handle != camera_handle) {
        CDBG_ERROR("%s: Not a valid test obj", __func__);
        return;
    }

    CDBG("%s:E evt = %d", __func__, evt->server_event_type);
    switch (evt->server_event_type) {
       case CAM_EVENT_TYPE_AUTO_FOCUS_DONE:
           CDBG("%s: rcvd auto focus done evt", __func__);
           break;
       case CAM_EVENT_TYPE_ZOOM_DONE:
           CDBG("%s: rcvd zoom done evt", __func__);
           break;
       default:
           break;
    }

    CDBG("%s:X", __func__);
}

int mm_app_open(mm_camera_app_t *cam_app,
                int cam_id,
                mm_camera_test_obj_t *test_obj)
{
    int32_t rc = 0;
    cam_frame_len_offset_t offset_info;

    CDBG("%s:BEGIN\n", __func__);

    rc = cam_app->hal_lib.mm_camera_open((uint8_t)cam_id, &(test_obj->cam));
    if(rc || !test_obj->cam) {
        CDBG_ERROR("%s:dev open error. rc = %d, vtbl = %p\n", __func__, rc, test_obj->cam);
        return -MM_CAMERA_E_GENERAL;
    }

    CDBG("Open Camera id = %d handle = %d", cam_id, test_obj->cam->camera_handle);

    /* alloc ion mem for capability buf */
    memset(&offset_info, 0, sizeof(offset_info));
    offset_info.frame_len = sizeof(cam_capability_t);

    rc = mm_app_alloc_bufs(&test_obj->cap_buf,
                           &offset_info,
                           1,
                           0,
                           0);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s:alloc buf for capability error\n", __func__);
        goto error_after_cam_open;
    }

    /* mapping capability buf */
    rc = test_obj->cam->ops->map_buf(test_obj->cam->camera_handle,
                                     CAM_MAPPING_BUF_TYPE_CAPABILITY,
                                     test_obj->cap_buf.mem_info.fd,
                                     test_obj->cap_buf.mem_info.size);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s:map for capability error\n", __func__);
        goto error_after_cap_buf_alloc;
    }

    /* alloc ion mem for getparm buf */
    memset(&offset_info, 0, sizeof(offset_info));
    offset_info.frame_len = sizeof(parm_buffer_t);
    rc = mm_app_alloc_bufs(&test_obj->parm_buf,
                           &offset_info,
                           1,
                           0,
                           0);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s:alloc buf for getparm_buf error\n", __func__);
        goto error_after_cap_buf_map;
    }

    /* mapping getparm buf */
    rc = test_obj->cam->ops->map_buf(test_obj->cam->camera_handle,
                                     CAM_MAPPING_BUF_TYPE_PARM_BUF,
                                     test_obj->parm_buf.mem_info.fd,
                                     test_obj->parm_buf.mem_info.size);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s:map getparm_buf error\n", __func__);
        goto error_after_getparm_buf_alloc;
    }
    test_obj->params_buffer = (parm_buffer_t*) test_obj->parm_buf.mem_info.data;
    CDBG_HIGH("\n%s params_buffer=%p\n",__func__,test_obj->params_buffer);

    rc = test_obj->cam->ops->register_event_notify(test_obj->cam->camera_handle,
                                                   notify_evt_cb,
                                                   test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: failed register_event_notify", __func__);
        rc = -MM_CAMERA_E_GENERAL;
        goto error_after_getparm_buf_map;
    }

    rc = test_obj->cam->ops->query_capability(test_obj->cam->camera_handle);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: failed query_capability", __func__);
        rc = -MM_CAMERA_E_GENERAL;
        goto error_after_getparm_buf_map;
    }
    memset(&test_obj->jpeg_ops, 0, sizeof(mm_jpeg_ops_t));
    mm_dimension pic_size;
    memset(&pic_size, 0, sizeof(mm_dimension));
    pic_size.w = 4000;
    pic_size.h = 3000;
    test_obj->jpeg_hdl = cam_app->hal_lib.jpeg_open(&test_obj->jpeg_ops, NULL, pic_size, NULL);
    if (test_obj->jpeg_hdl == 0) {
        CDBG_ERROR("%s: jpeg lib open err", __func__);
        rc = -MM_CAMERA_E_GENERAL;
        goto error_after_getparm_buf_map;
    }

    return rc;

error_after_getparm_buf_map:
    test_obj->cam->ops->unmap_buf(test_obj->cam->camera_handle,
                                  CAM_MAPPING_BUF_TYPE_PARM_BUF);
error_after_getparm_buf_alloc:
    mm_app_release_bufs(1, &test_obj->parm_buf);
error_after_cap_buf_map:
    test_obj->cam->ops->unmap_buf(test_obj->cam->camera_handle,
                                  CAM_MAPPING_BUF_TYPE_CAPABILITY);
error_after_cap_buf_alloc:
    mm_app_release_bufs(1, &test_obj->cap_buf);
error_after_cam_open:
    test_obj->cam->ops->close_camera(test_obj->cam->camera_handle);
    test_obj->cam = NULL;
    return rc;
}

int init_batch_update(parm_buffer_t *p_table)
{
    int rc = MM_CAMERA_OK;
    CDBG_HIGH("\nEnter %s\n",__func__);
    int32_t hal_version = CAM_HAL_V1;

    memset(p_table, 0, sizeof(parm_buffer_t));
    if(ADD_SET_PARAM_ENTRY_TO_BATCH(p_table, CAM_INTF_PARM_HAL_VERSION, hal_version)) {
        rc = -1;
    }

    return rc;
}

int commit_set_batch(mm_camera_test_obj_t *test_obj)
{
    int rc = MM_CAMERA_OK;
    int i = 0;

    for(i = 0; i < CAM_INTF_PARM_MAX; i++){
        if(test_obj->params_buffer->is_valid[i])
            break;
    }
    if (i < CAM_INTF_PARM_MAX) {
        CDBG_HIGH("\n set_param p_buffer =%p\n",test_obj->params_buffer);
        rc = test_obj->cam->ops->set_parms(test_obj->cam->camera_handle, test_obj->params_buffer);
    }
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: cam->ops->set_parms failed !!", __func__);
    }
    return rc;
}

int mm_app_close(mm_camera_test_obj_t *test_obj)
{
    int32_t rc = MM_CAMERA_OK;

    if (test_obj == NULL || test_obj->cam ==NULL) {
        CDBG_ERROR("%s: cam not opened", __func__);
        return -MM_CAMERA_E_GENERAL;
    }

    /* unmap capability buf */
    rc = test_obj->cam->ops->unmap_buf(test_obj->cam->camera_handle,
                                       CAM_MAPPING_BUF_TYPE_CAPABILITY);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: unmap capability buf failed, rc=%d", __func__, rc);
    }

    /* unmap parm buf */
    rc = test_obj->cam->ops->unmap_buf(test_obj->cam->camera_handle,
                                       CAM_MAPPING_BUF_TYPE_PARM_BUF);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: unmap setparm buf failed, rc=%d", __func__, rc);
    }

    rc = test_obj->cam->ops->close_camera(test_obj->cam->camera_handle);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: close camera failed, rc=%d", __func__, rc);
    }
    test_obj->cam = NULL;

    /* close jpeg client */
    if (test_obj->jpeg_hdl && test_obj->jpeg_ops.close) {
        rc = test_obj->jpeg_ops.close(test_obj->jpeg_hdl);
        test_obj->jpeg_hdl = 0;
        if (rc != MM_CAMERA_OK) {
            CDBG_ERROR("%s: close jpeg failed, rc=%d", __func__, rc);
        }
    }

    /* dealloc capability buf */
    rc = mm_app_release_bufs(1, &test_obj->cap_buf);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: release capability buf failed, rc=%d", __func__, rc);
    }

    /* dealloc parm buf */
    rc = mm_app_release_bufs(1, &test_obj->parm_buf);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: release setparm buf failed, rc=%d", __func__, rc);
    }

    return MM_CAMERA_OK;
}

mm_camera_channel_t * mm_app_add_channel(mm_camera_test_obj_t *test_obj,
                                         mm_camera_channel_type_t ch_type,
                                         mm_camera_channel_attr_t *attr,
                                         mm_camera_buf_notify_t channel_cb,
                                         void *userdata)
{
    uint32_t ch_id = 0;
    mm_camera_channel_t *channel = NULL;

    ch_id = test_obj->cam->ops->add_channel(test_obj->cam->camera_handle,
                                            attr,
                                            channel_cb,
                                            userdata);
    if (ch_id == 0) {
        CDBG_ERROR("%s: add channel failed", __func__);
        return NULL;
    }
    channel = &test_obj->channels[ch_type];
    channel->ch_id = ch_id;
    return channel;
}

int mm_app_del_channel(mm_camera_test_obj_t *test_obj,
                       mm_camera_channel_t *channel)
{
    test_obj->cam->ops->delete_channel(test_obj->cam->camera_handle,
                                       channel->ch_id);
    memset(channel, 0, sizeof(mm_camera_channel_t));
    return MM_CAMERA_OK;
}

mm_camera_stream_t * mm_app_add_stream(mm_camera_test_obj_t *test_obj,
                                       mm_camera_channel_t *channel)
{
    mm_camera_stream_t *stream = NULL;
    int rc = MM_CAMERA_OK;
    cam_frame_len_offset_t offset_info;

    stream = &(channel->streams[channel->num_streams++]);
    stream->s_id = test_obj->cam->ops->add_stream(test_obj->cam->camera_handle,
                                                  channel->ch_id);
    if (stream->s_id == 0) {
        CDBG_ERROR("%s: add stream failed", __func__);
        return NULL;
    }

    stream->multipleOf = test_obj->slice_size;

    /* alloc ion mem for stream_info buf */
    memset(&offset_info, 0, sizeof(offset_info));
    offset_info.frame_len = sizeof(cam_stream_info_t);

    rc = mm_app_alloc_bufs(&stream->s_info_buf,
                           &offset_info,
                           1,
                           0,
                           0);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s:alloc buf for stream_info error\n", __func__);
        test_obj->cam->ops->delete_stream(test_obj->cam->camera_handle,
                                          channel->ch_id,
                                          stream->s_id);
        stream->s_id = 0;
        return NULL;
    }

    /* mapping streaminfo buf */
    rc = test_obj->cam->ops->map_stream_buf(test_obj->cam->camera_handle,
                                            channel->ch_id,
                                            stream->s_id,
                                            CAM_MAPPING_BUF_TYPE_STREAM_INFO,
                                            0,
                                            -1,
                                            stream->s_info_buf.mem_info.fd,
                                            (uint32_t)stream->s_info_buf.mem_info.size);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s:map setparm_buf error\n", __func__);
        mm_app_deallocate_ion_memory(&stream->s_info_buf);
        test_obj->cam->ops->delete_stream(test_obj->cam->camera_handle,
                                          channel->ch_id,
                                          stream->s_id);
        stream->s_id = 0;
        return NULL;
    }

    return stream;
}

int mm_app_del_stream(mm_camera_test_obj_t *test_obj,
                      mm_camera_channel_t *channel,
                      mm_camera_stream_t *stream)
{
    test_obj->cam->ops->unmap_stream_buf(test_obj->cam->camera_handle,
                                         channel->ch_id,
                                         stream->s_id,
                                         CAM_MAPPING_BUF_TYPE_STREAM_INFO,
                                         0,
                                         -1);
    mm_app_deallocate_ion_memory(&stream->s_info_buf);
    test_obj->cam->ops->delete_stream(test_obj->cam->camera_handle,
                                      channel->ch_id,
                                      stream->s_id);
    memset(stream, 0, sizeof(mm_camera_stream_t));
    return MM_CAMERA_OK;
}

mm_camera_channel_t *mm_app_get_channel_by_type(mm_camera_test_obj_t *test_obj,
                                                mm_camera_channel_type_t ch_type)
{
    return &test_obj->channels[ch_type];
}

int mm_app_config_stream(mm_camera_test_obj_t *test_obj,
                         mm_camera_channel_t *channel,
                         mm_camera_stream_t *stream,
                         mm_camera_stream_config_t *config)
{
    return test_obj->cam->ops->config_stream(test_obj->cam->camera_handle,
                                             channel->ch_id,
                                             stream->s_id,
                                             config);
}

int mm_app_start_channel(mm_camera_test_obj_t *test_obj,
                         mm_camera_channel_t *channel)
{
    return test_obj->cam->ops->start_channel(test_obj->cam->camera_handle,
                                             channel->ch_id);
}

int mm_app_stop_channel(mm_camera_test_obj_t *test_obj,
                        mm_camera_channel_t *channel)
{
    return test_obj->cam->ops->stop_channel(test_obj->cam->camera_handle,
                                            channel->ch_id);
}

int initBatchUpdate(mm_camera_test_obj_t *test_obj)
{
    int32_t hal_version = CAM_HAL_V1;

    parm_buffer_t *parm_buf = ( parm_buffer_t * ) test_obj->parm_buf.mem_info.data;
    memset(parm_buf, 0, sizeof(parm_buffer_t));
    ADD_SET_PARAM_ENTRY_TO_BATCH(test_obj->parm_buf.mem_info.data,
            CAM_INTF_PARM_HAL_VERSION, hal_version);

    return MM_CAMERA_OK;
}

int commitSetBatch(mm_camera_test_obj_t *test_obj)
{
    int rc = MM_CAMERA_OK;
    int i = 0;

    parm_buffer_t *p_table = ( parm_buffer_t * ) test_obj->parm_buf.mem_info.data;
    for(i = 0; i < CAM_INTF_PARM_MAX; i++){
        if(p_table->is_valid[i])
            break;
    }
    if (i < CAM_INTF_PARM_MAX) {
        rc = test_obj->cam->ops->set_parms(test_obj->cam->camera_handle, p_table);
    }
    return rc;
}


int commitGetBatch(mm_camera_test_obj_t *test_obj)
{
    int rc = MM_CAMERA_OK;
    int i = 0;
    parm_buffer_t *p_table = ( parm_buffer_t * ) test_obj->parm_buf.mem_info.data;
    for(i = 0; i < CAM_INTF_PARM_MAX; i++){
        if(p_table->is_valid[i])
            break;
    }
    if (i < CAM_INTF_PARM_MAX) {
        rc = test_obj->cam->ops->get_parms(test_obj->cam->camera_handle, p_table);
    }
    return rc;
}

int setAecLock(mm_camera_test_obj_t *test_obj, int value)
{
    int rc = MM_CAMERA_OK;

    rc = initBatchUpdate(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch camera parameter update failed\n", __func__);
        goto ERROR;
    }

    if (ADD_SET_PARAM_ENTRY_TO_BATCH(test_obj->parm_buf.mem_info.data,
            CAM_INTF_PARM_AEC_LOCK, (uint32_t)value)) {
        CDBG_ERROR("%s: AEC Lock parameter not added to batch\n", __func__);
        rc = -1;
        goto ERROR;
    }

    rc = commitSetBatch(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch parameters commit failed\n", __func__);
        goto ERROR;
    }

ERROR:
    return rc;
}

int setAwbLock(mm_camera_test_obj_t *test_obj, int value)
{
    int rc = MM_CAMERA_OK;

    rc = initBatchUpdate(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch camera parameter update failed\n", __func__);
        goto ERROR;
    }

    if (ADD_SET_PARAM_ENTRY_TO_BATCH(test_obj->parm_buf.mem_info.data,
            CAM_INTF_PARM_AWB_LOCK, (uint32_t)value)) {
        CDBG_ERROR("%s: AWB Lock parameter not added to batch\n", __func__);
        rc = -1;
        goto ERROR;
    }

    rc = commitSetBatch(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch parameters commit failed\n", __func__);
        goto ERROR;
    }

ERROR:
    return rc;
}


int set3Acommand(mm_camera_test_obj_t *test_obj, cam_eztune_cmd_data_t *value)
{
    int rc = MM_CAMERA_OK;

    rc = initBatchUpdate(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch camera parameter update failed\n", __func__);
        goto ERROR;
    }

    if (ADD_SET_PARAM_ENTRY_TO_BATCH(test_obj->parm_buf.mem_info.data,
            CAM_INTF_PARM_EZTUNE_CMD, *value)) {
        CDBG_ERROR("%s: CAM_INTF_PARM_EZTUNE_CMD parameter not added to batch\n", __func__);
        rc = -1;
        goto ERROR;
    }

    rc = commitSetBatch(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch parameters commit failed\n", __func__);
        goto ERROR;
    }

ERROR:
    return rc;
}

int setAutoFocusTuning(mm_camera_test_obj_t *test_obj, tune_actuator_t *value)
{
    int rc = MM_CAMERA_OK;

    rc = initBatchUpdate(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch camera parameter update failed\n", __func__);
        goto ERROR;
    }

    if (ADD_SET_PARAM_ENTRY_TO_BATCH(test_obj->parm_buf.mem_info.data,
            CAM_INTF_PARM_SET_AUTOFOCUSTUNING, *value)) {
        CDBG_ERROR("%s: AutoFocus Tuning not added to batch\n", __func__);
        rc = -1;
        goto ERROR;
    }

    rc = commitSetBatch(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch parameters commit failed\n", __func__);
        goto ERROR;
    }

ERROR:
    return rc;
}

int setVfeCommand(mm_camera_test_obj_t *test_obj, tune_cmd_t *value)
{
    int rc = MM_CAMERA_OK;

    rc = initBatchUpdate(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch camera parameter update failed\n", __func__);
        goto ERROR;
    }

    if (ADD_SET_PARAM_ENTRY_TO_BATCH(test_obj->parm_buf.mem_info.data,
            CAM_INTF_PARM_SET_VFE_COMMAND, *value)) {
        CDBG_ERROR("%s: VFE Command not added to batch\n", __func__);
        rc = -1;
        goto ERROR;
    }

    rc = commitSetBatch(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch parameters commit failed\n", __func__);
        goto ERROR;
    }

ERROR:
    return rc;
}

int setmetainfoCommand(mm_camera_test_obj_t *test_obj, cam_stream_size_info_t *value)
{
    int rc = MM_CAMERA_OK;

    rc = initBatchUpdate(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch camera parameter update failed\n", __func__);
        goto ERROR;
    }

    if (ADD_SET_PARAM_ENTRY_TO_BATCH(test_obj->parm_buf.mem_info.data,
            CAM_INTF_META_STREAM_INFO, *value)) {
        CDBG_ERROR("%s: PP Command not added to batch\n", __func__);
        rc = -1;
        goto ERROR;
    }

    rc = commitSetBatch(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch parameters commit failed\n", __func__);
        goto ERROR;
    }

ERROR:
    return rc;
}


int setPPCommand(mm_camera_test_obj_t *test_obj, tune_cmd_t *value)
{
    int rc = MM_CAMERA_OK;

    rc = initBatchUpdate(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch camera parameter update failed\n", __func__);
        goto ERROR;
    }

    if (ADD_SET_PARAM_ENTRY_TO_BATCH(test_obj->parm_buf.mem_info.data,
            CAM_INTF_PARM_SET_PP_COMMAND, *value)) {
        CDBG_ERROR("%s: PP Command not added to batch\n", __func__);
        rc = -1;
        goto ERROR;
    }

    rc = commitSetBatch(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch parameters commit failed\n", __func__);
        goto ERROR;
    }

ERROR:
    return rc;
}

int setFocusMode(mm_camera_test_obj_t *test_obj, cam_focus_mode_type mode)
{
    int rc = MM_CAMERA_OK;

    rc = initBatchUpdate(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch camera parameter update failed\n", __func__);
        goto ERROR;
    }

    uint32_t value = mode;

    if (ADD_SET_PARAM_ENTRY_TO_BATCH(test_obj->parm_buf.mem_info.data,
            CAM_INTF_PARM_FOCUS_MODE, value)) {
        CDBG_ERROR("%s: Focus mode parameter not added to batch\n", __func__);
        rc = -1;
        goto ERROR;
    }

    rc = commitSetBatch(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch parameters commit failed\n", __func__);
        goto ERROR;
    }

ERROR:
    return rc;
}

int setEVCompensation(mm_camera_test_obj_t *test_obj, int ev)
{
    int rc = MM_CAMERA_OK;

    cam_capability_t *camera_cap = NULL;

    camera_cap = (cam_capability_t *) test_obj->cap_buf.mem_info.data;
    if ( (ev >= camera_cap->exposure_compensation_min) &&
         (ev <= camera_cap->exposure_compensation_max) ) {

        rc = initBatchUpdate(test_obj);
        if (rc != MM_CAMERA_OK) {
            CDBG_ERROR("%s: Batch camera parameter update failed\n", __func__);
            goto ERROR;
        }

        if (ADD_SET_PARAM_ENTRY_TO_BATCH(test_obj->parm_buf.mem_info.data,
                CAM_INTF_PARM_EXPOSURE_COMPENSATION, ev)) {
            CDBG_ERROR("%s: EV compensation parameter not added to batch\n", __func__);
            rc = -1;
            goto ERROR;
        }

        rc = commitSetBatch(test_obj);
        if (rc != MM_CAMERA_OK) {
            CDBG_ERROR("%s: Batch parameters commit failed\n", __func__);
            goto ERROR;
        }

        CDBG_ERROR("%s: EV compensation set to: %d", __func__, ev);
    } else {
        CDBG_ERROR("%s: Invalid EV compensation", __func__);
        return -EINVAL;
    }

ERROR:
    return rc;
}

int setAntibanding(mm_camera_test_obj_t *test_obj, cam_antibanding_mode_type antibanding)
{
    int rc = MM_CAMERA_OK;

    rc = initBatchUpdate(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch camera parameter update failed\n", __func__);
        goto ERROR;
    }

    if (ADD_SET_PARAM_ENTRY_TO_BATCH(test_obj->parm_buf.mem_info.data,
            CAM_INTF_PARM_ANTIBANDING, antibanding)) {
        CDBG_ERROR("%s: Antibanding parameter not added to batch\n", __func__);
        rc = -1;
        goto ERROR;
    }

    rc = commitSetBatch(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch parameters commit failed\n", __func__);
        goto ERROR;
    }

    CDBG_ERROR("%s: Antibanding set to: %d", __func__, (int)antibanding);

ERROR:
    return rc;
}

int setWhiteBalance(mm_camera_test_obj_t *test_obj, cam_wb_mode_type mode)
{
    int rc = MM_CAMERA_OK;

    rc = initBatchUpdate(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch camera parameter update failed\n", __func__);
        goto ERROR;
    }

    if (ADD_SET_PARAM_ENTRY_TO_BATCH(test_obj->parm_buf.mem_info.data,
            CAM_INTF_PARM_WHITE_BALANCE, mode)) {
        CDBG_ERROR("%s: White balance parameter not added to batch\n", __func__);
        rc = -1;
        goto ERROR;
    }

    rc = commitSetBatch(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch parameters commit failed\n", __func__);
        goto ERROR;
    }

    CDBG_ERROR("%s: White balance set to: %d", __func__, (int)mode);

ERROR:
    return rc;
}

int setExposureMetering(mm_camera_test_obj_t *test_obj, cam_auto_exposure_mode_type mode)
{
    int rc = MM_CAMERA_OK;

    rc = initBatchUpdate(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch camera parameter update failed\n", __func__);
        goto ERROR;
    }

    if (ADD_SET_PARAM_ENTRY_TO_BATCH(test_obj->parm_buf.mem_info.data,
            CAM_INTF_PARM_EXPOSURE, mode)) {
        CDBG_ERROR("%s: Exposure metering parameter not added to batch\n", __func__);
        rc = -1;
        goto ERROR;
    }

    rc = commitSetBatch(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch parameters commit failed\n", __func__);
        goto ERROR;
    }

    CDBG_ERROR("%s: Exposure metering set to: %d", __func__, (int)mode);

ERROR:
    return rc;
}

int setBrightness(mm_camera_test_obj_t *test_obj, int brightness)
{
    int rc = MM_CAMERA_OK;

    rc = initBatchUpdate(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch camera parameter update failed\n", __func__);
        goto ERROR;
    }

    if (ADD_SET_PARAM_ENTRY_TO_BATCH(test_obj->parm_buf.mem_info.data,
            CAM_INTF_PARM_BRIGHTNESS, brightness)) {
        CDBG_ERROR("%s: Brightness parameter not added to batch\n", __func__);
        rc = -1;
        goto ERROR;
    }

    rc = commitSetBatch(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch parameters commit failed\n", __func__);
        goto ERROR;
    }

    CDBG_ERROR("%s: Brightness set to: %d", __func__, brightness);

ERROR:
    return rc;
}

int setContrast(mm_camera_test_obj_t *test_obj, int contrast)
{
    int rc = MM_CAMERA_OK;

    rc = initBatchUpdate(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch camera parameter update failed\n", __func__);
        goto ERROR;
    }

    if (ADD_SET_PARAM_ENTRY_TO_BATCH(test_obj->parm_buf.mem_info.data,
            CAM_INTF_PARM_CONTRAST, contrast)) {
        CDBG_ERROR("%s: Contrast parameter not added to batch\n", __func__);
        rc = -1;
        goto ERROR;
    }

    rc = commitSetBatch(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch parameters commit failed\n", __func__);
        goto ERROR;
    }

    CDBG_ERROR("%s: Contrast set to: %d", __func__, contrast);

ERROR:
    return rc;
}

int setTintless(mm_camera_test_obj_t *test_obj, int tintless)
{
    int rc = MM_CAMERA_OK;

    rc = initBatchUpdate(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch camera parameter update failed\n", __func__);
        goto ERROR;
    }

    if (ADD_SET_PARAM_ENTRY_TO_BATCH(test_obj->parm_buf.mem_info.data,
            CAM_INTF_PARM_TINTLESS, tintless)) {
        CDBG_ERROR("%s: Tintless parameter not added to batch\n", __func__);
        rc = -1;
        goto ERROR;
    }

    rc = commitSetBatch(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch parameters commit failed\n", __func__);
        goto ERROR;
    }

    CDBG_ERROR("%s:  set Tintless to: %d", __func__, tintless);

ERROR:
    return rc;
}

int setSaturation(mm_camera_test_obj_t *test_obj, int saturation)
{
    int rc = MM_CAMERA_OK;

    rc = initBatchUpdate(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch camera parameter update failed\n", __func__);
        goto ERROR;
    }

    if (ADD_SET_PARAM_ENTRY_TO_BATCH(test_obj->parm_buf.mem_info.data,
            CAM_INTF_PARM_SATURATION, saturation)) {
        CDBG_ERROR("%s: Saturation parameter not added to batch\n", __func__);
        rc = -1;
        goto ERROR;
    }

    rc = commitSetBatch(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch parameters commit failed\n", __func__);
        goto ERROR;
    }

    CDBG_ERROR("%s: Saturation set to: %d", __func__, saturation);

ERROR:
    return rc;
}

int setSharpness(mm_camera_test_obj_t *test_obj, int sharpness)
{
    int rc = MM_CAMERA_OK;

    rc = initBatchUpdate(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch camera parameter update failed\n", __func__);
        goto ERROR;
    }

    if (ADD_SET_PARAM_ENTRY_TO_BATCH(test_obj->parm_buf.mem_info.data,
            CAM_INTF_PARM_SHARPNESS, sharpness)) {
        CDBG_ERROR("%s: Sharpness parameter not added to batch\n", __func__);
        rc = -1;
        goto ERROR;
    }

    rc = commitSetBatch(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch parameters commit failed\n", __func__);
        goto ERROR;
    }

    test_obj->reproc_sharpness = sharpness;
    CDBG_ERROR("%s: Sharpness set to: %d", __func__, sharpness);

ERROR:
    return rc;
}

int setISO(mm_camera_test_obj_t *test_obj, cam_iso_mode_type iso)
{
    int rc = MM_CAMERA_OK;

    rc = initBatchUpdate(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch camera parameter update failed\n", __func__);
        goto ERROR;
    }

    if (ADD_SET_PARAM_ENTRY_TO_BATCH(test_obj->parm_buf.mem_info.data,
            CAM_INTF_PARM_ISO, iso)) {
        CDBG_ERROR("%s: ISO parameter not added to batch\n", __func__);
        rc = -1;
        goto ERROR;
    }

    rc = commitSetBatch(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch parameters commit failed\n", __func__);
        goto ERROR;
    }

    CDBG_ERROR("%s: ISO set to: %d", __func__, (int)iso);

ERROR:
    return rc;
}

int setZoom(mm_camera_test_obj_t *test_obj, int zoom)
{
    int rc = MM_CAMERA_OK;

    rc = initBatchUpdate(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch camera parameter update failed\n", __func__);
        goto ERROR;
    }

    if (ADD_SET_PARAM_ENTRY_TO_BATCH(test_obj->parm_buf.mem_info.data,
            CAM_INTF_PARM_ZOOM, zoom)) {
        CDBG_ERROR("%s: Zoom parameter not added to batch\n", __func__);
        rc = -1;
        goto ERROR;
    }

    rc = commitSetBatch(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch parameters commit failed\n", __func__);
        goto ERROR;
    }

    CDBG_ERROR("%s: Zoom set to: %d", __func__, zoom);

ERROR:
    return rc;
}

int setFPSRange(mm_camera_test_obj_t *test_obj, cam_fps_range_t range)
{
    int rc = MM_CAMERA_OK;

    rc = initBatchUpdate(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch camera parameter update failed\n", __func__);
        goto ERROR;
    }

    if (ADD_SET_PARAM_ENTRY_TO_BATCH(test_obj->parm_buf.mem_info.data,
            CAM_INTF_PARM_FPS_RANGE, range)) {
        CDBG_ERROR("%s: FPS range parameter not added to batch\n", __func__);
        rc = -1;
        goto ERROR;
    }

    rc = commitSetBatch(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch parameters commit failed\n", __func__);
        goto ERROR;
    }

    CDBG_ERROR("%s: FPS Range set to: [%5.2f:%5.2f]",
                __func__,
                range.min_fps,
                range.max_fps);

ERROR:
    return rc;
}

int setScene(mm_camera_test_obj_t *test_obj, cam_scene_mode_type scene)
{
    int rc = MM_CAMERA_OK;

    rc = initBatchUpdate(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch camera parameter update failed\n", __func__);
        goto ERROR;
    }

    if (ADD_SET_PARAM_ENTRY_TO_BATCH(test_obj->parm_buf.mem_info.data,
            CAM_INTF_PARM_BESTSHOT_MODE, scene)) {
        CDBG_ERROR("%s: Scene parameter not added to batch\n", __func__);
        rc = -1;
        goto ERROR;
    }

    rc = commitSetBatch(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch parameters commit failed\n", __func__);
        goto ERROR;
    }

    CDBG_ERROR("%s: Scene set to: %d", __func__, (int)scene);

ERROR:
    return rc;
}

int setFlash(mm_camera_test_obj_t *test_obj, cam_flash_mode_t flash)
{
    int rc = MM_CAMERA_OK;

    rc = initBatchUpdate(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch camera parameter update failed\n", __func__);
        goto ERROR;
    }

    if (ADD_SET_PARAM_ENTRY_TO_BATCH(test_obj->parm_buf.mem_info.data,
            CAM_INTF_PARM_LED_MODE, flash)) {
        CDBG_ERROR("%s: Flash parameter not added to batch\n", __func__);
        rc = -1;
        goto ERROR;
    }

    rc = commitSetBatch(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch parameters commit failed\n", __func__);
        goto ERROR;
    }

    CDBG_ERROR("%s: Flash set to: %d", __func__, (int)flash);

ERROR:
    return rc;
}

int setWNR(mm_camera_test_obj_t *test_obj, uint8_t enable)
{
    int rc = MM_CAMERA_OK;

    rc = initBatchUpdate(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch camera parameter update failed\n", __func__);
        goto ERROR;
    }

    cam_denoise_param_t param;
    memset(&param, 0, sizeof(cam_denoise_param_t));
    param.denoise_enable = enable;
    param.process_plates = CAM_WAVELET_DENOISE_YCBCR_PLANE;

    if (ADD_SET_PARAM_ENTRY_TO_BATCH(test_obj->parm_buf.mem_info.data,
            CAM_INTF_PARM_WAVELET_DENOISE, param)) {
        CDBG_ERROR("%s: WNR enabled parameter not added to batch\n", __func__);
        rc = -1;
        goto ERROR;
    }

    rc = commitSetBatch(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch parameters commit failed\n", __func__);
        goto ERROR;
    }


    test_obj->reproc_wnr = param;
    CDBG_ERROR("%s: WNR enabled: %d", __func__, enable);

ERROR:
    return rc;
}


/** tuneserver_capture
 *    @lib_handle: the camera handle object
 *    @dim: snapshot dimensions
 *
 *  makes JPEG capture
 *
 *  Return: >=0 on success, -1 on failure.
 **/
int tuneserver_capture(mm_camera_lib_handle *lib_handle,
                       mm_camera_lib_snapshot_params *dim)
{
    int rc = 0;

    printf("Take jpeg snapshot\n");
    if ( lib_handle->stream_running ) {

        if ( lib_handle->test_obj.zsl_enabled) {
            if ( NULL != dim) {
                if ( ( lib_handle->test_obj.buffer_width != dim->width) ||
                     ( lib_handle->test_obj.buffer_height = dim->height ) ) {

                    lib_handle->test_obj.buffer_width = dim->width;
                    lib_handle->test_obj.buffer_height = dim->height;

                    rc = mm_camera_lib_stop_stream(lib_handle);
                    if (rc != MM_CAMERA_OK) {
                        CDBG_ERROR("%s: mm_camera_lib_stop_stream() err=%d\n",
                                   __func__, rc);
                        goto EXIT;
                    }

                    rc = mm_camera_lib_start_stream(lib_handle);
                    if (rc != MM_CAMERA_OK) {
                        CDBG_ERROR("%s: mm_camera_lib_start_stream() err=%d\n",
                                   __func__, rc);
                        goto EXIT;
                    }
                }

            }

            lib_handle->test_obj.encodeJpeg = 1;

            mm_camera_app_wait();
        } else {
            // For standard 2D capture streaming has to be disabled first
            rc = mm_camera_lib_stop_stream(lib_handle);
            if (rc != MM_CAMERA_OK) {
                CDBG_ERROR("%s: mm_camera_lib_stop_stream() err=%d\n",
                         __func__, rc);
                goto EXIT;
            }

            if ( NULL != dim ) {
                lib_handle->test_obj.buffer_width = dim->width;
                lib_handle->test_obj.buffer_height = dim->height;
            }
            rc = mm_app_start_capture(&lib_handle->test_obj, 1);
            if (rc != MM_CAMERA_OK) {
                CDBG_ERROR("%s: mm_app_start_capture() err=%d\n",
                         __func__, rc);
                goto EXIT;
            }

            mm_camera_app_wait();

            rc = mm_app_stop_capture(&lib_handle->test_obj);
            if (rc != MM_CAMERA_OK) {
                CDBG_ERROR("%s: mm_app_stop_capture() err=%d\n",
                         __func__, rc);
                goto EXIT;
            }

            // Restart streaming after capture is done
            rc = mm_camera_lib_start_stream(lib_handle);
            if (rc != MM_CAMERA_OK) {
                CDBG_ERROR("%s: mm_camera_lib_start_stream() err=%d\n",
                         __func__, rc);
                goto EXIT;
            }
        }
    }

EXIT:

    return rc;
}

int mm_app_start_regression_test(int run_tc)
{
    int rc = MM_CAMERA_OK;
    mm_camera_app_t my_cam_app;

    CDBG("\nCamera Test Application\n");
    memset(&my_cam_app, 0, sizeof(mm_camera_app_t));

    rc = mm_app_load_hal(&my_cam_app);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: mm_app_load_hal failed !!", __func__);
        return rc;
    }

    if(run_tc) {
        rc = mm_app_unit_test_entry(&my_cam_app);
        return rc;
    }
#if 0
    if(run_dual_tc) {
        printf("\tRunning Dual camera test engine only\n");
        rc = mm_app_dual_test_entry(&my_cam_app);
        printf("\t Dual camera engine. EXIT(%d)!!!\n", rc);
        exit(rc);
    }
#endif
    return rc;
}

int32_t mm_camera_load_tuninglibrary(mm_camera_tuning_lib_params_t *tuning_param)
{
  void *(*tuning_open_lib)(void) = NULL;

  CDBG("%s  %d\n", __func__, __LINE__);
  tuning_param->lib_handle = dlopen("libmmcamera_tuning.so", RTLD_NOW);
  if (!tuning_param->lib_handle) {
    CDBG_ERROR("%s Failed opening libmmcamera_tuning.so\n", __func__);
    return -EINVAL;
  }

  *(void **)&tuning_open_lib  = dlsym(tuning_param->lib_handle,
    "open_tuning_lib");
  if (!tuning_open_lib) {
    CDBG_ERROR("%s Failed symbol libmmcamera_tuning.so\n", __func__);
    return -EINVAL;
  }

  if (tuning_param->func_tbl) {
    CDBG_ERROR("%s already loaded tuninglib..", __func__);
    return 0;
  }

  tuning_param->func_tbl = (mm_camera_tune_func_t *)tuning_open_lib();
  if (!tuning_param->func_tbl) {
    CDBG_ERROR("%s Failed opening library func table ptr\n", __func__);
    return -EINVAL;
  }

  CDBG("%s  %d\n", __func__, __LINE__);
  return 0;
}

int mm_camera_lib_open(mm_camera_lib_handle *handle, int cam_id)
{
    int rc = MM_CAMERA_OK;

    if ( NULL == handle ) {
        CDBG_ERROR(" %s : Invalid handle", __func__);
        rc = MM_CAMERA_E_INVALID_INPUT;
        goto EXIT;
    }

    memset(handle, 0, sizeof(mm_camera_lib_handle));
    rc = mm_app_load_hal(&handle->app_ctx);
    if( MM_CAMERA_OK != rc ) {
        CDBG_ERROR("%s:mm_app_init err\n", __func__);
        goto EXIT;
    }

    handle->test_obj.buffer_width = DEFAULT_PREVIEW_WIDTH;
    handle->test_obj.buffer_height = DEFAULT_PREVIEW_HEIGHT;
    handle->test_obj.buffer_format = DEFAULT_SNAPSHOT_FORMAT;
    handle->current_params.stream_width = DEFAULT_SNAPSHOT_WIDTH;
    handle->current_params.stream_height = DEFAULT_SNAPSHOT_HEIGHT;
    handle->current_params.af_mode = CAM_FOCUS_MODE_AUTO; // Default to auto focus mode
    rc = mm_app_open(&handle->app_ctx, (uint8_t)cam_id, &handle->test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s:mm_app_open() cam_idx=%d, err=%d\n",
                   __func__, cam_id, rc);
        goto EXIT;
    }

    //rc = mm_app_initialize_fb(&handle->test_obj);
    rc = MM_CAMERA_OK;
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: mm_app_initialize_fb() cam_idx=%d, err=%d\n",
                   __func__, cam_id, rc);
        goto EXIT;
    }

EXIT:

    return rc;
}

int mm_camera_lib_start_stream(mm_camera_lib_handle *handle)
{
    int rc = MM_CAMERA_OK;
    cam_capability_t camera_cap;

    if ( NULL == handle ) {
        CDBG_ERROR(" %s : Invalid handle", __func__);
        rc = MM_CAMERA_E_INVALID_INPUT;
        goto EXIT;
    }

    if ( handle->test_obj.zsl_enabled ) {
        rc = mm_app_start_preview_zsl(&handle->test_obj);
        if (rc != MM_CAMERA_OK) {
            CDBG_ERROR("%s: mm_app_start_preview_zsl() err=%d\n",
                       __func__, rc);
            goto EXIT;
        }
    } else {
        handle->test_obj.enable_reproc = ENABLE_REPROCESSING;
        rc = mm_app_start_preview(&handle->test_obj);
        if (rc != MM_CAMERA_OK) {
            CDBG_ERROR("%s: mm_app_start_preview() err=%d\n",
                       __func__, rc);
            goto EXIT;
        }
    }

    // Configure focus mode after stream starts
    rc = mm_camera_lib_get_caps(handle, &camera_cap);
    if ( MM_CAMERA_OK != rc ) {
      CDBG_ERROR("%s:mm_camera_lib_get_caps() err=%d\n", __func__, rc);
      return -1;
    }
    if (camera_cap.supported_focus_modes_cnt == 1 &&
      camera_cap.supported_focus_modes[0] == CAM_FOCUS_MODE_FIXED) {
      CDBG("focus not supported");
      handle->test_obj.focus_supported = 0;
      handle->current_params.af_mode = CAM_FOCUS_MODE_FIXED;
    } else {
      handle->test_obj.focus_supported = 1;
    }
    rc = setFocusMode(&handle->test_obj, handle->current_params.af_mode);
    if (rc != MM_CAMERA_OK) {
      CDBG_ERROR("%s:autofocus error\n", __func__);
      goto EXIT;
    }
    handle->stream_running = 1;

EXIT:
    return rc;
}

int mm_camera_lib_stop_stream(mm_camera_lib_handle *handle)
{
    int rc = MM_CAMERA_OK;

    if ( NULL == handle ) {
        CDBG_ERROR(" %s : Invalid handle", __func__);
        rc = MM_CAMERA_E_INVALID_INPUT;
        goto EXIT;
    }

    if ( handle->test_obj.zsl_enabled ) {
        rc = mm_app_stop_preview_zsl(&handle->test_obj);
        if (rc != MM_CAMERA_OK) {
            CDBG_ERROR("%s: mm_app_stop_preview_zsl() err=%d\n",
                       __func__, rc);
            goto EXIT;
        }
    } else {
        rc = mm_app_stop_preview(&handle->test_obj);
        if (rc != MM_CAMERA_OK) {
            CDBG_ERROR("%s: mm_app_stop_preview() err=%d\n",
                       __func__, rc);
            goto EXIT;
        }
    }

    handle->stream_running = 0;

EXIT:
    return rc;
}

int mm_camera_lib_get_caps(mm_camera_lib_handle *handle,
                           cam_capability_t *caps)
{
    int rc = MM_CAMERA_OK;

    if ( NULL == handle ) {
        CDBG_ERROR(" %s : Invalid handle", __func__);
        rc = MM_CAMERA_E_INVALID_INPUT;
        goto EXIT;
    }

    if ( NULL == caps ) {
        CDBG_ERROR(" %s : Invalid capabilities structure", __func__);
        rc = MM_CAMERA_E_INVALID_INPUT;
        goto EXIT;
    }

    *caps = *( (cam_capability_t *) handle->test_obj.cap_buf.mem_info.data );

EXIT:

    return rc;
}


int mm_camera_lib_send_command(mm_camera_lib_handle *handle,
                               mm_camera_lib_commands cmd,
                               void *in_data, void *out_data)
{
    uint32_t width, height;
    int rc = MM_CAMERA_OK;
    cam_capability_t *camera_cap = NULL;
    mm_camera_lib_snapshot_params *dim = NULL;

    if ( NULL == handle ) {
        CDBG_ERROR(" %s : Invalid handle", __func__);
        rc = MM_CAMERA_E_INVALID_INPUT;
        goto EXIT;
    }

    camera_cap = (cam_capability_t *) handle->test_obj.cap_buf.mem_info.data;

    switch(cmd) {
        case MM_CAMERA_LIB_FPS_RANGE:
            if ( NULL != in_data ) {
                cam_fps_range_t range = *(( cam_fps_range_t * )in_data);
                rc = setFPSRange(&handle->test_obj, range);
                if (rc != MM_CAMERA_OK) {
                        CDBG_ERROR("%s: setFPSRange() err=%d\n",
                                   __func__, rc);
                        goto EXIT;
                }
            }
            break;
        case MM_CAMERA_LIB_FLASH:
            if ( NULL != in_data ) {
                cam_flash_mode_t flash = *(( int * )in_data);
                rc = setFlash(&handle->test_obj, flash);
                if (rc != MM_CAMERA_OK) {
                        CDBG_ERROR("%s: setFlash() err=%d\n",
                                   __func__, rc);
                        goto EXIT;
                }
            }
            break;
        case MM_CAMERA_LIB_BESTSHOT:
            if ( NULL != in_data ) {
                cam_scene_mode_type scene = *(( int * )in_data);
                rc = setScene(&handle->test_obj, scene);
                if (rc != MM_CAMERA_OK) {
                        CDBG_ERROR("%s: setScene() err=%d\n",
                                   __func__, rc);
                        goto EXIT;
                }
            }
            break;
        case MM_CAMERA_LIB_ZOOM:
            if ( NULL != in_data ) {
                int zoom = *(( int * )in_data);
                rc = setZoom(&handle->test_obj, zoom);
                if (rc != MM_CAMERA_OK) {
                        CDBG_ERROR("%s: setZoom() err=%d\n",
                                   __func__, rc);
                        goto EXIT;
                }
            }
            break;
        case MM_CAMERA_LIB_ISO:
            if ( NULL != in_data ) {
                cam_iso_mode_type iso = *(( int * )in_data);
                rc = setISO(&handle->test_obj, iso);
                if (rc != MM_CAMERA_OK) {
                        CDBG_ERROR("%s: setISO() err=%d\n",
                                   __func__, rc);
                        goto EXIT;
                }
            }
            break;
        case MM_CAMERA_LIB_SHARPNESS:
            if ( NULL != in_data ) {
                int sharpness = *(( int * )in_data);
                rc = setSharpness(&handle->test_obj, sharpness);
                if (rc != MM_CAMERA_OK) {
                        CDBG_ERROR("%s: setSharpness() err=%d\n",
                                   __func__, rc);
                        goto EXIT;
                }
            }
            break;
        case MM_CAMERA_LIB_SATURATION:
            if ( NULL != in_data ) {
                int saturation = *(( int * )in_data);
                rc = setSaturation(&handle->test_obj, saturation);
                if (rc != MM_CAMERA_OK) {
                        CDBG_ERROR("%s: setSaturation() err=%d\n",
                                   __func__, rc);
                        goto EXIT;
                }
            }
            break;
        case MM_CAMERA_LIB_CONTRAST:
            if ( NULL != in_data ) {
                int contrast = *(( int * )in_data);
                rc = setContrast(&handle->test_obj, contrast);
                if (rc != MM_CAMERA_OK) {
                        CDBG_ERROR("%s: setContrast() err=%d\n",
                                   __func__, rc);
                        goto EXIT;
                }
            }
            break;
        case MM_CAMERA_LIB_SET_TINTLESS:
            if ( NULL != in_data ) {
                int tintless = *(( int * )in_data);
                rc = setTintless(&handle->test_obj, tintless);
                if (rc != MM_CAMERA_OK) {
                        CDBG_ERROR("%s: enlabe/disable:%d tintless() err=%d\n",
                                   __func__, tintless, rc);
                        goto EXIT;
                }
            }
            break;
        case MM_CAMERA_LIB_BRIGHTNESS:
            if ( NULL != in_data ) {
                int brightness = *(( int * )in_data);
                rc = setBrightness(&handle->test_obj, brightness);
                if (rc != MM_CAMERA_OK) {
                        CDBG_ERROR("%s: setBrightness() err=%d\n",
                                   __func__, rc);
                        goto EXIT;
                }
            }
            break;
        case MM_CAMERA_LIB_EXPOSURE_METERING:
            if ( NULL != in_data ) {
                cam_auto_exposure_mode_type exp = *(( int * )in_data);
                rc = setExposureMetering(&handle->test_obj, exp);
                if (rc != MM_CAMERA_OK) {
                        CDBG_ERROR("%s: setExposureMetering() err=%d\n",
                                   __func__, rc);
                        goto EXIT;
                }
            }
            break;
        case MM_CAMERA_LIB_WB:
            if ( NULL != in_data ) {
                cam_wb_mode_type wb = *(( int * )in_data);
                rc = setWhiteBalance(&handle->test_obj, wb);
                if (rc != MM_CAMERA_OK) {
                        CDBG_ERROR("%s: setWhiteBalance() err=%d\n",
                                   __func__, rc);
                        goto EXIT;
                }
            }
            break;
        case MM_CAMERA_LIB_ANTIBANDING:
            if ( NULL != in_data ) {
                int antibanding = *(( int * )in_data);
                rc = setAntibanding(&handle->test_obj, antibanding);
                if (rc != MM_CAMERA_OK) {
                        CDBG_ERROR("%s: setAntibanding() err=%d\n",
                                   __func__, rc);
                        goto EXIT;
                }
            }
            break;
        case MM_CAMERA_LIB_EV:
            if ( NULL != in_data ) {
                int ev = *(( int * )in_data);
                rc = setEVCompensation(&handle->test_obj, ev);
                if (rc != MM_CAMERA_OK) {
                        CDBG_ERROR("%s: setEVCompensation() err=%d\n",
                                   __func__, rc);
                        goto EXIT;
                }
            }
            break;
        case MM_CAMERA_LIB_ZSL_ENABLE:
            if ( NULL != in_data) {
                int enable_zsl = *(( int * )in_data);
                if ( ( enable_zsl != handle->test_obj.zsl_enabled ) &&
                        handle->stream_running ) {
                    rc = mm_camera_lib_stop_stream(handle);
                    if (rc != MM_CAMERA_OK) {
                        CDBG_ERROR("%s: mm_camera_lib_stop_stream() err=%d\n",
                                   __func__, rc);
                        goto EXIT;
                    }
                    handle->test_obj.zsl_enabled = enable_zsl;
                    rc = mm_camera_lib_start_stream(handle);
                    if (rc != MM_CAMERA_OK) {
                        CDBG_ERROR("%s: mm_camera_lib_start_stream() err=%d\n",
                                   __func__, rc);
                        goto EXIT;
                    }
                } else {
                    handle->test_obj.zsl_enabled = enable_zsl;
                }
            }
            break;
        case MM_CAMERA_LIB_RAW_CAPTURE:

            if ( 0 == handle->stream_running ) {
                CDBG_ERROR(" %s : Streaming is not enabled!", __func__);
                rc = MM_CAMERA_E_INVALID_OPERATION;
                goto EXIT;
            }

            rc = mm_camera_lib_stop_stream(handle);
            if (rc != MM_CAMERA_OK) {
                CDBG_ERROR("%s: mm_camera_lib_stop_stream() err=%d\n",
                           __func__, rc);
                goto EXIT;
            }

            width = handle->test_obj.buffer_width;
            height = handle->test_obj.buffer_height;
            handle->test_obj.buffer_width =
                    (uint32_t)camera_cap->raw_dim[0].width;
            handle->test_obj.buffer_height =
                    (uint32_t)camera_cap->raw_dim[0].height;
            handle->test_obj.buffer_format = DEFAULT_RAW_FORMAT;
            CDBG_ERROR("%s: MM_CAMERA_LIB_RAW_CAPTURE %dx%d\n",
                       __func__,
                       camera_cap->raw_dim[0].width,
                       camera_cap->raw_dim[0].height);
            rc = mm_app_start_capture_raw(&handle->test_obj, 1);
            if (rc != MM_CAMERA_OK) {
                CDBG_ERROR("%s: mm_app_start_capture() err=%d\n",
                           __func__, rc);
                goto EXIT;
            }

            mm_camera_app_wait();

            rc = mm_app_stop_capture_raw(&handle->test_obj);
            if (rc != MM_CAMERA_OK) {
                CDBG_ERROR("%s: mm_app_stop_capture() err=%d\n",
                           __func__, rc);
                goto EXIT;
            }

            handle->test_obj.buffer_width = width;
            handle->test_obj.buffer_height = height;
            handle->test_obj.buffer_format = DEFAULT_SNAPSHOT_FORMAT;
            rc = mm_camera_lib_start_stream(handle);
            if (rc != MM_CAMERA_OK) {
                CDBG_ERROR("%s: mm_camera_lib_start_stream() err=%d\n",
                           __func__, rc);
                goto EXIT;
            }

            break;

        case MM_CAMERA_LIB_JPEG_CAPTURE:
            if ( 0 == handle->stream_running ) {
                CDBG_ERROR(" %s : Streaming is not enabled!", __func__);
                rc = MM_CAMERA_E_INVALID_OPERATION;
                goto EXIT;
            }

            if ( NULL != in_data ) {
                dim = ( mm_camera_lib_snapshot_params * ) in_data;
            }

            rc = tuneserver_capture(handle, dim);
            if (rc != MM_CAMERA_OK) {
                CDBG_ERROR("%s:capture error %d\n", __func__, rc);
                goto EXIT;
            }
            break;

        case MM_CAMERA_LIB_SET_FOCUS_MODE: {
            cam_focus_mode_type mode = *((cam_focus_mode_type *)in_data);
            handle->current_params.af_mode = mode;
            rc = setFocusMode(&handle->test_obj, mode);
            if (rc != MM_CAMERA_OK) {
              CDBG_ERROR("%s:autofocus error\n", __func__);
              goto EXIT;
            }
            break;
        }

        case MM_CAMERA_LIB_DO_AF:
            if (handle->test_obj.focus_supported) {
              rc = handle->test_obj.cam->ops->do_auto_focus(handle->test_obj.cam->camera_handle);
              if (rc != MM_CAMERA_OK) {
                CDBG_ERROR("%s:autofocus error\n", __func__);
                goto EXIT;
              }
              /*Waiting for Auto Focus Done Call Back*/
              mm_camera_app_wait();
            }
            break;

        case MM_CAMERA_LIB_CANCEL_AF:
            rc = handle->test_obj.cam->ops->cancel_auto_focus(handle->test_obj.cam->camera_handle);
            if (rc != MM_CAMERA_OK) {
                CDBG_ERROR("%s:autofocus error\n", __func__);
                goto EXIT;
            }

            break;

        case MM_CAMERA_LIB_LOCK_AWB:
            rc = setAwbLock(&handle->test_obj, 1);
            if (rc != MM_CAMERA_OK) {
                CDBG_ERROR("%s: AWB locking failed\n", __func__);
                goto EXIT;
            }
            break;

        case MM_CAMERA_LIB_UNLOCK_AWB:
            rc = setAwbLock(&handle->test_obj, 0);
            if (rc != MM_CAMERA_OK) {
                CDBG_ERROR("%s: AE unlocking failed\n", __func__);
                goto EXIT;
            }
            break;

        case MM_CAMERA_LIB_LOCK_AE:
            rc = setAecLock(&handle->test_obj, 1);
            if (rc != MM_CAMERA_OK) {
                CDBG_ERROR("%s: AE locking failed\n", __func__);
                goto EXIT;
            }
            break;

        case MM_CAMERA_LIB_UNLOCK_AE:
            rc = setAecLock(&handle->test_obj, 0);
            if (rc != MM_CAMERA_OK) {
                CDBG_ERROR("%s: AE unlocking failed\n", __func__);
                goto EXIT;
            }
            break;

       case MM_CAMERA_LIB_SET_3A_COMMAND: {
          rc = set3Acommand(&handle->test_obj, (cam_eztune_cmd_data_t *)in_data);
          if (rc != MM_CAMERA_OK) {
            CDBG_ERROR("%s:3A set command error\n", __func__);
            goto EXIT;
          }
          break;
        }

       case MM_CAMERA_LIB_SET_AUTOFOCUS_TUNING: {
           rc = setAutoFocusTuning(&handle->test_obj, in_data);
           if (rc != MM_CAMERA_OK) {
             CDBG_ERROR("%s: Set AF tuning failed\n", __func__);
             goto EXIT;
           }
           break;
       }

       case MM_CAMERA_LIB_SET_VFE_COMMAND: {
           rc = setVfeCommand(&handle->test_obj, in_data);
           if (rc != MM_CAMERA_OK) {
             CDBG_ERROR("%s: Set vfe command failed\n", __func__);
             goto EXIT;
           }
           break;
       }

       case MM_CAMERA_LIB_SET_POSTPROC_COMMAND: {
           rc = setPPCommand(&handle->test_obj, in_data);
           if (rc != MM_CAMERA_OK) {
             CDBG_ERROR("%s: Set pp command failed\n", __func__);
             goto EXIT;
           }
           break;
       }

        case MM_CAMERA_LIB_WNR_ENABLE: {
            rc = setWNR(&handle->test_obj, *((uint8_t *)in_data));
            if ( rc != MM_CAMERA_OK) {
                CDBG_ERROR("%s: Set wnr enable failed\n", __func__);
                goto EXIT;
            }
        }

      case MM_CAMERA_LIB_NO_ACTION:
        default:
            break;
    };

EXIT:

    return rc;
}
int mm_camera_lib_number_of_cameras(mm_camera_lib_handle *handle)
{
    int rc = 0;

    if ( NULL == handle ) {
        CDBG_ERROR(" %s : Invalid handle", __func__);
        goto EXIT;
    }

    rc = handle->app_ctx.num_cameras;

EXIT:

    return rc;
}

int mm_camera_lib_close(mm_camera_lib_handle *handle)
{
    int rc = MM_CAMERA_OK;

    if ( NULL == handle ) {
        CDBG_ERROR(" %s : Invalid handle", __func__);
        rc = MM_CAMERA_E_INVALID_INPUT;
        goto EXIT;
    }

    //rc = mm_app_close_fb(&handle->test_obj);
    rc = MM_CAMERA_OK;
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s:mm_app_close_fb() err=%d\n",
                   __func__, rc);
        goto EXIT;
    }

    rc = mm_app_close(&handle->test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s:mm_app_close() err=%d\n",
                   __func__, rc);
        goto EXIT;
    }

EXIT:
    return rc;
}

int mm_camera_lib_set_preview_usercb(
   mm_camera_lib_handle *handle, cam_stream_user_cb cb)
{
    if (handle->test_obj.user_preview_cb != NULL) {
        CDBG_ERROR("%s, already set preview callbacks\n", __func__);
        return -1;
    }
    handle->test_obj.user_preview_cb = *cb;
    return 0;
}

int mm_app_set_preview_fps_range(mm_camera_test_obj_t *test_obj,
                        cam_fps_range_t *fpsRange)
{
    int rc = MM_CAMERA_OK;
    CDBG_HIGH("%s: preview fps range: min=%f, max=%f.", __func__,
        fpsRange->min_fps, fpsRange->max_fps);
    rc = setFPSRange(test_obj, *fpsRange);

    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: add_parm_entry_tobatch failed !!", __func__);
        return rc;
    }

    return rc;
}

int mm_app_set_face_detection(mm_camera_test_obj_t *test_obj,
        cam_fd_set_parm_t *fd_set_parm)
{
    int rc = MM_CAMERA_OK;

    if (test_obj == NULL || fd_set_parm == NULL) {
        CDBG_ERROR("%s, invalid params!", __func__);
        return MM_CAMERA_E_INVALID_INPUT;
    }

    CDBG_HIGH("%s: mode = %d, num_fd = %d", __func__,
          fd_set_parm->fd_mode, fd_set_parm->num_fd);

    rc = initBatchUpdate(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch camera parameter update failed\n", __func__);
        goto ERROR;
    }

    if (ADD_SET_PARAM_ENTRY_TO_BATCH(test_obj->parm_buf.mem_info.data,
        CAM_INTF_PARM_FD, *fd_set_parm)) {
        CDBG_ERROR("%s: FD parameter not added to batch\n", __func__);
        rc = -1;
        goto ERROR;
    }

    rc = commitSetBatch(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch parameters commit failed\n", __func__);
        goto ERROR;
    }

ERROR:
    return rc;
}

int mm_app_set_flash_mode(mm_camera_test_obj_t *test_obj,
        cam_flash_mode_t flashMode)
{
    int rc = MM_CAMERA_OK;

    if (test_obj == NULL) {
        CDBG_ERROR("%s, invalid params!", __func__);
        return MM_CAMERA_E_INVALID_INPUT;
    }

    CDBG_HIGH("%s: mode = %d", __func__, (int)flashMode);

    rc = initBatchUpdate(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch camera parameter update failed\n", __func__);
        goto ERROR;
    }

    if (ADD_SET_PARAM_ENTRY_TO_BATCH(test_obj->parm_buf.mem_info.data,
        CAM_INTF_PARM_LED_MODE, flashMode)) {
        CDBG_ERROR("%s: Flash mode parameter not added to batch\n", __func__);
        rc = -1;
        goto ERROR;
    }

    rc = commitSetBatch(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch parameters commit failed\n", __func__);
        goto ERROR;
    }

ERROR:
    return rc;
}

int mm_app_set_metadata_usercb(mm_camera_test_obj_t *test_obj,
                        cam_stream_user_cb usercb)
{
    if (test_obj == NULL || usercb == NULL) {
        CDBG_ERROR("%s, invalid params!", __func__);
        return MM_CAMERA_E_INVALID_INPUT;
    }

    CDBG_HIGH("%s, set user metadata callback, addr: %p\n", __func__, usercb);

    if (test_obj->user_metadata_cb != NULL) {
        CDBG_HIGH("%s, already set user metadata callback", __func__);
    }
    test_obj->user_metadata_cb = usercb;

    return 0;
}


