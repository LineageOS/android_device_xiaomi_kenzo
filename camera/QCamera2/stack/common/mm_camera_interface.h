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

#ifndef __MM_CAMERA_INTERFACE_H__
#define __MM_CAMERA_INTERFACE_H__
#include <linux/msm_ion.h>
#include <linux/videodev2.h>
#include <media/msmb_camera.h>
#include <utils/Timers.h>
#include "cam_intf.h"
#include "cam_queue.h"

#define MM_CAMERA_MAX_NUM_SENSORS MSM_MAX_CAMERA_SENSORS
#define MM_CAMERA_MAX_NUM_FRAMES CAM_MAX_NUM_BUFS_PER_STREAM
/* num of channels allowed in a camera obj */
#define MM_CAMERA_CHANNEL_MAX 16

#define PAD_TO_SIZE(size, padding) \
        ((size + (typeof(size))(padding - 1)) & \
        (typeof(size))(~(padding - 1)))

/** CAM_DUMP_TO_FILE:
 *  @filename: file name
 *  @name:filename
 *  @index: index of the file
 *  @extn: file extension
 *  @p_addr: address of the buffer
 *  @len: buffer length
 *
 *  dump the image to the file
 **/
#define CAM_DUMP_TO_FILE(path, name, index, extn, p_addr, len) ({ \
  size_t rc = 0; \
  char filename[FILENAME_MAX]; \
  if (index >= 0) \
    snprintf(filename, FILENAME_MAX, "%s/%s%d.%s", path, name, index, extn); \
  else \
    snprintf(filename, FILENAME_MAX, "%s/%s.%s", path, name, extn); \
  FILE *fp = fopen(filename, "w+"); \
  if (fp) { \
    rc = fwrite(p_addr, 1, len, fp); \
    ALOGE("%s:%d] written size %d", __func__, __LINE__, len); \
    fclose(fp); \
  } else { \
    ALOGE("%s:%d] open %s failed", __func__, __LINE__, filename); \
  } \
})

/* Declaring Buffer structure */
struct mm_camera_buf_def;

/** mm_camera_plane_def_t : structure for frame plane info
*    @num_planes : num of planes for the frame buffer, to be
*               filled during mem allocation
*    @planes : plane info for the frame buffer, to be filled
*               during mem allocation
**/
typedef struct {
    int8_t num_planes;
    struct v4l2_plane planes[VIDEO_MAX_PLANES];
} mm_camera_plane_buf_def_t;

/** mm_camera_user_buf_def_t : structure for frame plane info
*    @num_buffers : num of buffers in this user defined structure
*    @bufs_used : actual number of buffer filled
*    @buf_in_use : flag to notify buffer usage status.
*    @plane_buf : Plane buffer array pointer.
**/
typedef struct {
    uint8_t num_buffers;
    uint8_t bufs_used;     /*Num of Buffer filled by Kernel*/
    uint8_t buf_in_use;  /* Container buffer is freed to fill*/
    int32_t buf_idx[MSM_CAMERA_MAX_USER_BUFF_CNT];
    struct mm_camera_buf_def *plane_buf;
} mm_camera_user_buf_def_t;

/** mm_camera_buf_def_t: structure for stream frame buf
*    @stream_id : stream handler to uniquely identify a stream
*               object
*    @buf_idx : index of the buf within the stream bufs, to be
*               filled during mem allocation
*    @timespec_ts : time stamp, to be filled when DQBUF is
*                 called
*    @frame_idx : frame sequence num, to be filled when DQBUF
*    @plane_buf  : Frame plane definition
*    @fd : file descriptor of the frame buffer, to be filled
*        during mem allocation
*    @buffer : pointer to the frame buffer, to be filled during
*            mem allocation
*    @frame_len : length of the whole frame, to be filled during
*               mem allocation
*    @mem_info : user specific pointer to additional mem info
**/
typedef struct mm_camera_buf_def {
    uint32_t stream_id;
    cam_stream_type_t stream_type;
    cam_stream_buf_type buf_type;
    uint32_t buf_idx;
    uint8_t is_uv_subsampled;
    struct timespec ts;
    uint32_t frame_idx;
    union {
        mm_camera_plane_buf_def_t planes_buf;
        mm_camera_user_buf_def_t user_buf;
    };
    int fd;
    void *buffer;
    size_t frame_len;
    void *mem_info;
} mm_camera_buf_def_t;

/** mm_camera_super_buf_t: super buf structure for bundled
*   stream frames
*    @camera_handle : camera handler to uniquely identify
*              a camera object
*    @ch_id : channel handler to uniquely ideentify a channel
*           object
*    @num_bufs : number of buffers in the super buf, should not
*              exceeds MAX_STREAM_NUM_IN_BUNDLE
*    @bufs : array of buffers in the bundle
**/
typedef struct {
    uint32_t camera_handle;
    uint32_t ch_id;
    uint32_t num_bufs;
    uint8_t bUnlockAEC;
    uint8_t bReadyForPrepareSnapshot;
    mm_camera_buf_def_t* bufs[MAX_STREAM_NUM_IN_BUNDLE];
} mm_camera_super_buf_t;

/** mm_camera_req_buf_type_t
* Request type for super buf from channel
**/
typedef enum {
    MM_CAMERA_REQ_SUPER_BUF,
    MM_CAMERA_REQ_FRAME_SYNC_BUF
} mm_camera_req_buf_type_t;

/** mm_camera_req_buf_t: Attributes for super buf request
*
*    @type : type of super buf requested
*    @num_buf_requested : num of super bufs requested
*    @num_retro_buf_requested : number of retro bufs requested
*    @primary_only : specifies if only primary camera frame for a dual
*     camera is requested
**/
typedef struct {
    mm_camera_req_buf_type_t type;
    uint32_t num_buf_requested;
    uint32_t num_retro_buf_requested;
    uint8_t primary_only;
} mm_camera_req_buf_t;

/** mm_camera_event_t: structure for event
*    @server_event_type : event type from serer
*    @status : status of an event, value could be
*              CAM_STATUS_SUCCESS
*              CAM_STATUS_FAILED
**/
typedef struct {
    cam_event_type_t server_event_type;
    uint32_t status;
} mm_camera_event_t;

/** mm_camera_event_notify_t: function definition for event
*   notify handling
*    @camera_handle : camera handler
*    @evt : pointer to an event struct
*    @user_data: user data pointer
**/
typedef void (*mm_camera_event_notify_t)(uint32_t camera_handle,
                                         mm_camera_event_t *evt,
                                         void *user_data);

/** mm_camera_buf_notify_t: function definition for frame notify
*   handling
*    @mm_camera_super_buf_t : received frame buffers
*    @user_data: user data pointer
**/
typedef void (*mm_camera_buf_notify_t) (mm_camera_super_buf_t *bufs,
                                        void *user_data);

/** map_stream_buf_op_t: function definition for operation of
*   mapping stream buffers via domain socket
*    @frame_idx : buffer index within stream buffers
*    @plane_idx    : plane index. If all planes share the same
*                   fd, plane_idx = -1; otherwise, plean_idx is
*                   the index to plane (0..num_of_planes)
*    @fd : file descriptor of the stream buffer
*    @size: size of the stream buffer
*    @userdata : user data pointer
**/
typedef int32_t (*map_stream_buf_op_t) (uint32_t frame_idx,
                                        int32_t plane_idx,
                                        int fd,
                                        size_t size,
                                        cam_mapping_buf_type type,
                                        void *userdata);

typedef int32_t (*map_stream_bufs_op_t) (const cam_buf_map_type_list *buf_map_list,
                                         void *userdata);

/** unmap_stream_buf_op_t: function definition for operation of
*                          unmapping stream buffers via domain
*                          socket
*    @frame_idx : buffer index within stream buffers
*    @plane_idx : plane index. If all planes share the same
*                 fd, plane_idx = -1; otherwise, plean_idx is
*                 the index to plane (0..num_of_planes)
*    @userdata : user data pointer
**/
typedef int32_t (*unmap_stream_buf_op_t) (uint32_t frame_idx,
                                          int32_t plane_idx,
                                          cam_mapping_buf_type type,
                                          void *userdata);

/** mm_camera_map_unmap_ops_tbl_t: virtual table
*                      for mapping/unmapping stream buffers via
*                      domain socket
*    @map_ops : operation for mapping
*    @unmap_ops : operation for unmapping
*    @userdata: user data pointer
**/
typedef struct {
    map_stream_buf_op_t map_ops;
    map_stream_bufs_op_t bundled_map_ops;
    unmap_stream_buf_op_t unmap_ops;
    void *userdata;
} mm_camera_map_unmap_ops_tbl_t;

/** mm_camera_stream_mem_vtbl_t: virtual table for stream
*                      memory allocation and deallocation
*    @get_bufs : function definition for allocating
*                stream buffers
*    @put_bufs : function definition for deallocating
*                stream buffers
*    @user_data: user data pointer
**/
typedef struct {
  void *user_data;
  int32_t (*set_config_ops) (mm_camera_map_unmap_ops_tbl_t *ops_tbl,
          void *user_data);
  int32_t (*get_bufs) (cam_frame_len_offset_t *offset,
                       uint8_t *num_bufs,
                       uint8_t **initial_reg_flag,
                       mm_camera_buf_def_t **bufs,
                       mm_camera_map_unmap_ops_tbl_t *ops_tbl,
                       void *user_data);
  int32_t (*put_bufs) (mm_camera_map_unmap_ops_tbl_t *ops_tbl,
                       void *user_data);
  int32_t (*invalidate_buf)(uint32_t index, void *user_data);
  int32_t (*clean_invalidate_buf)(uint32_t index, void *user_data);
} mm_camera_stream_mem_vtbl_t;

/** mm_camera_stream_config_t: structure for stream
*                              configuration
*    @stream_info : pointer to a stream info structure
*    @padding_info: padding info obtained from querycapability
*    @mem_tbl : memory operation table for
*              allocating/deallocating stream buffers
*    @stream_cb_sync : SYNC callback handling stream frame notify
*    @stream_cb : ASYNC callback handling stream frame notify
*    @userdata : user data pointer
**/
typedef struct {
    cam_stream_info_t *stream_info;
    cam_padding_info_t padding_info;
    mm_camera_stream_mem_vtbl_t mem_vtbl;
    mm_camera_buf_notify_t stream_cb_sync;
    mm_camera_buf_notify_t stream_cb;
    void *userdata;
} mm_camera_stream_config_t;

/** mm_camera_super_buf_notify_mode_t: enum for super uffer
*                                      notification mode
*    @MM_CAMERA_SUPER_BUF_NOTIFY_BURST :
*       ZSL use case: get burst of frames
*    @MM_CAMERA_SUPER_BUF_NOTIFY_CONTINUOUS :
*       get continuous frames: when the super buf is ready
*       dispatch it to HAL
**/
typedef enum {
    MM_CAMERA_SUPER_BUF_NOTIFY_BURST = 0,
    MM_CAMERA_SUPER_BUF_NOTIFY_CONTINUOUS,
    MM_CAMERA_SUPER_BUF_NOTIFY_MAX
} mm_camera_super_buf_notify_mode_t;

/** mm_camera_super_buf_priority_t: enum for super buffer
*                                   matching priority
*    @MM_CAMERA_SUPER_BUF_PRIORITY_NORMAL :
*       Save the frame no matter focused or not. Currently only
*       this type is supported.
*    @MM_CAMERA_SUPER_BUF_PRIORITY_FOCUS :
*       only queue the frame that is focused. Will enable meta
*       data header to carry focus info
*    @MM_CAMERA_SUPER_BUF_PRIORITY_EXPOSURE_BRACKETING :
*       after shutter, only queue matched exposure index
**/
typedef enum {
    MM_CAMERA_SUPER_BUF_PRIORITY_NORMAL = 0,
    MM_CAMERA_SUPER_BUF_PRIORITY_FOCUS,
    MM_CAMERA_SUPER_BUF_PRIORITY_EXPOSURE_BRACKETING,
    MM_CAMERA_SUPER_BUF_PRIORITY_LOW,/* Bundled metadata frame may not match*/
    MM_CAMERA_SUPER_BUF_PRIORITY_MAX
} mm_camera_super_buf_priority_t;

/** mm_camera_advanced_capture_t: enum for advanced capture type.
*    @MM_CAMERA_AF_BRACKETING :
*       to enable AF Bracketig.
*    @MM_CAMERA_AE_BRACKETING :
*       to enable AF Bracketing.
*    @MM_CAMERA_FLASH_BRACKETING :
*       to enable Flash Bracketing.
*    @MM_CAMERA_ZOOM_1X :
*       to enable zoom 1x capture request
**/
typedef enum {
   MM_CAMERA_AF_BRACKETING = 0,
   MM_CAMERA_AE_BRACKETING,
   MM_CAMERA_FLASH_BRACKETING,
   MM_CAMERA_ZOOM_1X,
   MM_CAMERA_FRAME_CAPTURE,
} mm_camera_advanced_capture_t;

/** mm_camera_stream_cb_type: enum for stream buffer callback type.
*    @MM_CAMERA_STREAM_CB_TYPE_ASYNC :
*       callback is async type. buffer process done in client thread context
*    @MM_CAMERA_STREAM_CB_TYPE_SYNC :
*       callback is sync type. buffer process done interface thread context
**/
typedef enum {
    MM_CAMERA_STREAM_CB_TYPE_ASYNC,
    MM_CAMERA_STREAM_CB_TYPE_SYNC,
} mm_camera_stream_cb_type;


/** mm_camera_channel_attr_t: structure for defining channel
*                             attributes
*    @notify_mode : notify mode: burst or continuous
*    @water_mark : queue depth. Only valid for burst mode
*    @look_back : look back how many frames from last buf.
*                 Only valid for burst mode
*    @post_frame_skip : after send first frame to HAL, how many
*                     frames needing to be skipped for next
*                     delivery. Only valid for burst mode
*    @max_unmatched_frames : max number of unmatched frames in
*                     queue
*    @enable_frame_sync: Enables frame sync for dual camera
*    @priority : save matched priority frames only
*    @instant_capture_enabled : flag to indicate whether
*                     instant capture enabled or not.
*    @aec_frame_bound : Number of frames, camera interface will wait for
*                     getting the instant capture frame.
**/
typedef struct {
    mm_camera_super_buf_notify_mode_t notify_mode;
    uint8_t water_mark;
    uint8_t look_back;
    uint8_t post_frame_skip;
    uint8_t max_unmatched_frames;
    uint8_t enable_frame_sync;
    mm_camera_super_buf_priority_t priority;
    uint8_t instant_capture_enabled;
    uint8_t aec_frame_bound;
} mm_camera_channel_attr_t;

typedef struct {
    /** query_capability: fucntion definition for querying static
     *                    camera capabilities
     *    @camera_handle : camer handler
     *  Return value: 0 -- success
     *                -1 -- failure
     *  Note: would assume cam_capability_t is already mapped
     **/
    int32_t (*query_capability) (uint32_t camera_handle);

    /** register_event_notify: fucntion definition for registering
     *                         for event notification
     *    @camera_handle : camer handler
     *    @evt_cb : callback for event notify
     *    @user_data : user data poiner
     *  Return value: 0 -- success
     *                -1 -- failure
     **/
    int32_t (*register_event_notify) (uint32_t camera_handle,
                                      mm_camera_event_notify_t evt_cb,
                                      void *user_data);

    /** close_camera: fucntion definition for closing a camera
     *    @camera_handle : camer handler
     *  Return value: 0 -- success
     *                -1 -- failure
     **/
    int32_t (*close_camera) (uint32_t camera_handle);


    /** error_close_camera: function definition for closing
     *                      the camera backend on an unrecoverable
     *                      error
     *    @camera_handle : camera handler
     *  Return value: 0 -- success
     *                -1 -- failure
     **/
    int32_t (*error_close_camera) (uint32_t camera_handle);

    /** map_buf: fucntion definition for mapping a camera buffer
     *           via domain socket
     *    @camera_handle : camer handler
     *    @buf_type : type of mapping buffers, can be value of
     *                CAM_MAPPING_BUF_TYPE_CAPABILITY
     *                CAM_MAPPING_BUF_TYPE_SETPARM_BUF
     *                CAM_MAPPING_BUF_TYPE_GETPARM_BUF
     *    @fd : file descriptor of the stream buffer
     *    @size :  size of the stream buffer
     *  Return value: 0 -- success
     *                -1 -- failure
     **/
    int32_t (*map_buf) (uint32_t camera_handle,
                        uint8_t buf_type,
                        int fd,
                        size_t size);

    /** map_bufs: function definition for mapping multiple camera buffers
     *           via domain socket
     *    @camera_handle : camera handler
     *    @buf_map_list : list of buffers to map
     *  Return value: 0 -- success
     *                -1 -- failure
     **/
    int32_t (*map_bufs) (uint32_t camera_handle,
                         const cam_buf_map_type_list *buf_map_list);

    /** unmap_buf: fucntion definition for unmapping a camera buffer
     *           via domain socket
     *    @camera_handle : camer handler
     *    @buf_type : type of mapping buffers, can be value of
     *                CAM_MAPPING_BUF_TYPE_CAPABILITY
     *                CAM_MAPPING_BUF_TYPE_SETPARM_BUF
     *                CAM_MAPPING_BUF_TYPE_GETPARM_BUF
     *  Return value: 0 -- success
     *                -1 -- failure
     **/
    int32_t (*unmap_buf) (uint32_t camera_handle,
                          uint8_t buf_type);

    /** set_parms: fucntion definition for setting camera
     *             based parameters to server
     *    @camera_handle : camer handler
     *    @parms : batch for parameters to be set, stored in
     *               parm_buffer_t
     *  Return value: 0 -- success
     *                -1 -- failure
     *  Note: would assume parm_buffer_t is already mapped, and
     *       according parameter entries to be set are filled in the
     *       buf before this call
     **/
    int32_t (*set_parms) (uint32_t camera_handle,
                          parm_buffer_t *parms);

    /** get_parms: fucntion definition for querying camera
     *             based parameters from server
     *    @camera_handle : camer handler
     *    @parms : batch for parameters to be queried, stored in
     *               parm_buffer_t
     *  Return value: 0 -- success
     *                -1 -- failure
     *  Note: would assume parm_buffer_t is already mapped, and
     *       according parameter entries to be queried are filled in
     *       the buf before this call
     **/
    int32_t (*get_parms) (uint32_t camera_handle,
                          parm_buffer_t *parms);

    /** do_auto_focus: fucntion definition for performing auto focus
     *    @camera_handle : camer handler
     *  Return value: 0 -- success
     *                -1 -- failure
     *  Note: if this call success, we will always assume there will
     *        be an auto_focus event following up.
     **/
    int32_t (*do_auto_focus) (uint32_t camera_handle);

    /** cancel_auto_focus: fucntion definition for cancelling
     *                     previous auto focus request
     *    @camera_handle : camer handler
    *  Return value: 0 -- success
    *                -1 -- failure
     **/
    int32_t (*cancel_auto_focus) (uint32_t camera_handle);

    /** prepare_snapshot: fucntion definition for preparing hardware
     *                    for snapshot.
     *    @camera_handle : camer handler
     *    @do_af_flag    : flag indicating if AF needs to be done
     *                     0 -- no AF needed
     *                     1 -- AF needed
     *  Return value: 0 -- success
     *                -1 -- failure
     **/
    int32_t (*prepare_snapshot) (uint32_t camera_handle,
                                 int32_t do_af_flag);

    /** start_zsl_snapshot: function definition for starting
     *                    zsl snapshot.
     *    @camera_handle : camer handler
     *    @ch_id         : channel id
     *  Return value: 0 -- success
     *                -1 -- failure
     **/
    int32_t (*start_zsl_snapshot) (uint32_t camera_handle, uint32_t ch_id);

    /** stop_zsl_snapshot: function definition for stopping
     *                    zsl snapshot.
     *    @camera_handle : camer handler
     *    @ch_id         : channel id
     *  Return value: 0 -- success
     *                -1 -- failure
     **/
    int32_t (*stop_zsl_snapshot) (uint32_t camera_handle, uint32_t ch_id);

    /** add_channel: fucntion definition for adding a channel
     *    @camera_handle : camer handler
     *    @ch_id : channel handler
     *    @attr : pointer to channel attribute structure
     *    @channel_cb : callbak to handle bundled super buffer
     *    @userdata : user data pointer
     *  Return value: channel id, zero is invalid ch_id
     * Note: attr, channel_cb, and userdata can be NULL if no
     *       superbufCB is needed
     **/
    uint32_t (*add_channel) (uint32_t camera_handle,
                             mm_camera_channel_attr_t *attr,
                             mm_camera_buf_notify_t channel_cb,
                             void *userdata);

    /** delete_channel: fucntion definition for deleting a channel
     *    @camera_handle : camer handler
     *    @ch_id : channel handler
     *  Return value: 0 -- success
     *                -1 -- failure
     **/
    int32_t (*delete_channel) (uint32_t camera_handle,
                               uint32_t ch_id);

    /** get_bundle_info: function definition for querying bundle
     *  info of the channel
     *    @camera_handle : camera handler
     *    @ch_id         : channel handler
     *    @bundle_info   : bundle info to be filled in
     *  Return value: 0 -- success
     *                -1 -- failure
     **/
    int32_t (*get_bundle_info) (uint32_t camera_handle,
                                uint32_t ch_id,
                                cam_bundle_config_t *bundle_info);

    /** add_stream: fucntion definition for adding a stream
     *    @camera_handle : camer handler
     *    @ch_id : channel handler
     *  Return value: stream_id. zero is invalid stream_id
     **/
    uint32_t (*add_stream) (uint32_t camera_handle,
                            uint32_t ch_id);

    /** delete_stream: fucntion definition for deleting a stream
     *    @camera_handle : camer handler
     *    @ch_id : channel handler
     *    @stream_id : stream handler
     *  Return value: 0 -- success
     *                -1 -- failure
     **/
    int32_t (*delete_stream) (uint32_t camera_handle,
                              uint32_t ch_id,
                              uint32_t stream_id);

    /** link_stream: function definition for linking a stream
     *    @camera_handle : camera handle
     *    @ch_id : channel handle from which the stream originates
     *    @stream_id : stream handle
     *    @linked_ch_id: channel handle in which the stream will be linked
     *  Return value: 0 -- success
     *                -1 -- failure
     **/
    int32_t (*link_stream) (uint32_t camera_handle,
          uint32_t ch_id,
          uint32_t stream_id,
          uint32_t linked_ch_id);

    /** config_stream: fucntion definition for configuring a stream
     *    @camera_handle : camer handler
     *    @ch_id : channel handler
     *    @stream_id : stream handler
     *    @confid : pointer to a stream configuration structure
     *  Return value: 0 -- success
     *                -1 -- failure
     **/
    int32_t (*config_stream) (uint32_t camera_handle,
                              uint32_t ch_id,
                              uint32_t stream_id,
                              mm_camera_stream_config_t *config);

    /** map_stream_buf: fucntion definition for mapping
     *                 stream buffer via domain socket
     *    @camera_handle : camer handler
     *    @ch_id : channel handler
     *    @stream_id : stream handler
     *    @buf_type : type of mapping buffers, can be value of
     *             CAM_MAPPING_BUF_TYPE_STREAM_BUF
     *             CAM_MAPPING_BUF_TYPE_STREAM_INFO
     *             CAM_MAPPING_BUF_TYPE_OFFLINE_INPUT_BUF
     *    @buf_idx : buffer index within the stream buffers
     *    @plane_idx : plane index. If all planes share the same fd,
     *               plane_idx = -1; otherwise, plean_idx is the
     *               index to plane (0..num_of_planes)
     *    @fd : file descriptor of the stream buffer
     *    @size :  size of the stream buffer
     *  Return value: 0 -- success
     *                -1 -- failure
     **/
    int32_t (*map_stream_buf) (uint32_t camera_handle,
                               uint32_t ch_id,
                               uint32_t stream_id,
                               uint8_t buf_type,
                               uint32_t buf_idx,
                               int32_t plane_idx,
                               int fd,
                               size_t size);

    /** map_stream_bufs: function definition for mapping multiple
     *                 stream buffers via domain socket
     *    @camera_handle : camera handler
     *    @ch_id : channel handler
     *    @buf_map_list : list of buffers to map
     *  Return value: 0 -- success
     *                -1 -- failure
     **/
    int32_t (*map_stream_bufs) (uint32_t camera_handle,
                                uint32_t ch_id,
                                const cam_buf_map_type_list *buf_map_list);

    /** unmap_stream_buf: fucntion definition for unmapping
     *                 stream buffer via domain socket
     *    @camera_handle : camer handler
     *    @ch_id : channel handler
     *    @stream_id : stream handler
     *    @buf_type : type of mapping buffers, can be value of
     *             CAM_MAPPING_BUF_TYPE_STREAM_BUF
     *             CAM_MAPPING_BUF_TYPE_STREAM_INFO
     *             CAM_MAPPING_BUF_TYPE_OFFLINE_INPUT_BUF
     *    @buf_idx : buffer index within the stream buffers
     *    @plane_idx : plane index. If all planes share the same fd,
     *               plane_idx = -1; otherwise, plean_idx is the
     *               index to plane (0..num_of_planes)
     *  Return value: 0 -- success
     *                -1 -- failure
     **/
    int32_t (*unmap_stream_buf) (uint32_t camera_handle,
                                 uint32_t ch_id,
                                 uint32_t stream_id,
                                 uint8_t buf_type,
                                 uint32_t buf_idx,
                                 int32_t plane_idx);

    /** set_stream_parms: fucntion definition for setting stream
     *                    specific parameters to server
     *    @camera_handle : camer handler
     *    @ch_id : channel handler
     *    @stream_id : stream handler
     *    @parms : batch for parameters to be set
     *  Return value: 0 -- success
     *                -1 -- failure
     *  Note: would assume parm buffer is already mapped, and
     *       according parameter entries to be set are filled in the
     *       buf before this call
     **/
    int32_t (*set_stream_parms) (uint32_t camera_handle,
                                 uint32_t ch_id,
                                 uint32_t s_id,
                                 cam_stream_parm_buffer_t *parms);

    /** get_stream_parms: fucntion definition for querying stream
     *                    specific parameters from server
     *    @camera_handle : camer handler
     *    @ch_id : channel handler
     *    @stream_id : stream handler
     *    @parms : batch for parameters to be queried
     *  Return value: 0 -- success
     *                -1 -- failure
     *  Note: would assume parm buffer is already mapped, and
     *       according parameter entries to be queried are filled in
     *       the buf before this call
     **/
    int32_t (*get_stream_parms) (uint32_t camera_handle,
                                 uint32_t ch_id,
                                 uint32_t s_id,
                                 cam_stream_parm_buffer_t *parms);

    /** start_channel: fucntion definition for starting a channel
     *    @camera_handle : camer handler
     *    @ch_id : channel handler
     *  Return value: 0 -- success
     *                -1 -- failure
     * This call will start all streams belongs to the channel
     **/
    int32_t (*start_channel) (uint32_t camera_handle,
                              uint32_t ch_id);

    /** stop_channel: fucntion definition for stopping a channel
     *    @camera_handle : camer handler
     *    @ch_id : channel handler
     *  Return value: 0 -- success
     *                -1 -- failure
     * This call will stop all streams belongs to the channel
     **/
    int32_t (*stop_channel) (uint32_t camera_handle,
                             uint32_t ch_id);

    /** qbuf: fucntion definition for queuing a frame buffer back to
     *        kernel for reuse
     *    @camera_handle : camer handler
     *    @ch_id : channel handler
     *    @buf : a frame buffer to be queued back to kernel
     *  Return value: 0 -- success
     *                -1 -- failure
     **/
    int32_t (*qbuf) (uint32_t camera_handle,
                     uint32_t ch_id,
                     mm_camera_buf_def_t *buf);

    /** get_queued_buf_count: fucntion definition for querying queued buf count
     *    @camera_handle : camer handler
     *    @ch_id : channel handler
     *    @stream_id : stream handler
     *  Return value: queued buf count
     **/
    int32_t (*get_queued_buf_count) (uint32_t camera_handle,
            uint32_t ch_id,
            uint32_t stream_id);

    /** request_super_buf: fucntion definition for requesting frames
     *                     from superbuf queue in burst mode
     *    @camera_handle : camer handler
     *    @ch_id : channel handler
     *    @buf : provides info related to the super buf request
     *  Return value: 0 -- success
     *                -1 -- failure
     **/
    int32_t (*request_super_buf) (uint32_t camera_handle,
                                  uint32_t ch_id,
                                  mm_camera_req_buf_t *buf);

    /** cancel_super_buf_request: fucntion definition for canceling
     *                     frames dispatched from superbuf queue in
     *                     burst mode
     *    @camera_handle : camer handler
     *    @ch_id : channel handler
     *  Return value: 0 -- success
     *                -1 -- failure
     **/
    int32_t (*cancel_super_buf_request) (uint32_t camera_handle,
                                         uint32_t ch_id);

    /** flush_super_buf_queue: function definition for flushing out
     *                     all frames in the superbuf queue up to frame_idx,
     *                     even if frames with frame_idx come in later than
     *                     this call.
     *    @camera_handle : camer handler
     *    @ch_id : channel handler
     *    @frame_idx : frame index up until which all superbufs are flushed
     *  Return value: 0 -- success
     *                -1 -- failure
     **/
    int32_t (*flush_super_buf_queue) (uint32_t camera_handle,
                                      uint32_t ch_id, uint32_t frame_idx);

    /** configure_notify_mode: function definition for configuring the
     *                         notification mode of channel
     *    @camera_handle : camera handler
     *    @ch_id : channel handler
     *    @notify_mode : notification mode
     *  Return value: 0 -- success
     *                -1 -- failure
     **/
    int32_t (*configure_notify_mode) (uint32_t camera_handle,
                                      uint32_t ch_id,
                                      mm_camera_super_buf_notify_mode_t notify_mode);

   /** process_advanced_capture: function definition for start/stop advanced capture
     *                    for snapshot.
     *    @camera_handle : camera handle
     *    @ch_id : channel handler
     *    @type :  advanced capture type.
     *    @trigger    : flag indicating if advanced capture needs to be done
     *                     0 -- stop advanced capture
     *                     1 -- start advanced capture
     *    @in_value: Input value. Configaration
     *  Return value: 0 -- success
     *                -1 -- failure
     **/
    int32_t (*process_advanced_capture) (uint32_t camera_handle,
             uint32_t ch_id, mm_camera_advanced_capture_t type,
             int8_t start_flag, void *in_value);

   /** get_session_id: gets the backend session id from the kernel
     *    @camera_handle : camera handle
     *    @sessionid : session id to be retrieved
     *     Return value: 0 -- success
     *                -1 -- failure
     *  Note: if this call succeeds, we will get a valid session id
     **/
    int32_t (*get_session_id) (uint32_t camera_handle,
            uint32_t* sessionid);

    /** sync_related_sensors: sends sync cmd
      *    @camera_handle : camera handle
      *    @related_cam_info : related cam info to be sent to server
      *     Return value: 0 -- success
      *                -1 -- failure
      *  Note: if this call succeeds, we will get linking established in back end
      **/
     int32_t (*sync_related_sensors) (uint32_t camera_handle,
            cam_sync_related_sensors_event_info_t*
            related_cam_info);
    /** flush: function definition for flush
     *  @camera_handle: camera handler
     *  Return value: 0 -- success
     *               -1 -- failure
     **/
    int32_t (*flush) (uint32_t camera_handle);

   /** register_stream_buf_cb: fucntion definition for registering special stream callbacks
     *    @camera_handle : camer handler
     *    @ch_id : channel handler
     *    @stream_id : stream handler
     *    @buf_cb : callback function pointer
     *    @cb_type : Callback type SYNC/ASYNC
     *    @userdata : user data pointer
     *    Return value: 0 -- success
     *                -       1 -- failure
     **/
    int32_t (*register_stream_buf_cb) (uint32_t camera_handle,
            uint32_t ch_id, uint32_t stream_id, mm_camera_buf_notify_t buf_cb,
            mm_camera_stream_cb_type cb_type, void *userdata);
} mm_camera_ops_t;

/** mm_camera_vtbl_t: virtual table for camera operations
*    @camera_handle : camera handler which uniquely identifies a
*                   camera object
*    @ops : API call table
**/
typedef struct {
    uint32_t camera_handle;
    mm_camera_ops_t *ops;
} mm_camera_vtbl_t;

/* return number of cameras */
uint8_t get_num_of_cameras();

/* return reference pointer of camera vtbl */
int32_t camera_open(uint8_t camera_idx, mm_camera_vtbl_t **camera_obj);

/* helper functions */
int32_t mm_stream_calc_offset_preview(cam_stream_info_t *stream_info,
        cam_dimension_t *dim,
        cam_padding_info_t *padding,
        cam_stream_buf_plane_info_t *buf_planes);

int32_t mm_stream_calc_offset_post_view(cam_format_t fmt,
        cam_dimension_t *dim,
        cam_stream_buf_plane_info_t *buf_planes);

int32_t mm_stream_calc_offset_snapshot(cam_format_t fmt,
        cam_dimension_t *dim,
        cam_padding_info_t *padding,
        cam_stream_buf_plane_info_t *buf_planes);

int32_t mm_stream_calc_offset_raw(cam_format_t fmt,
        cam_dimension_t *dim,
        cam_padding_info_t *padding,
        cam_stream_buf_plane_info_t *buf_planes);

int32_t mm_stream_calc_offset_video(cam_format_t fmt,
        cam_dimension_t *dim,
        cam_stream_buf_plane_info_t *buf_planes);

int32_t mm_stream_calc_offset_metadata(cam_dimension_t *dim,
        cam_padding_info_t *padding,
        cam_stream_buf_plane_info_t *buf_planes);

int32_t mm_stream_calc_offset_postproc(cam_stream_info_t *stream_info,
        cam_padding_info_t *padding,
        cam_stream_buf_plane_info_t *buf_planes);

int32_t mm_stream_calc_offset_analysis(cam_format_t fmt,
        cam_dimension_t *dim,
        cam_padding_info_t *padding,
        cam_stream_buf_plane_info_t *buf_planes);

uint32_t mm_stream_calc_lcm (int32_t num1, int32_t num2);

struct camera_info *get_cam_info(uint32_t camera_id, cam_sync_type_t *pCamType);

uint8_t is_yuv_sensor(uint32_t camera_id);

nsecs_t getBootToMonoTimeOffset();
#endif /*__MM_CAMERA_INTERFACE_H__*/
