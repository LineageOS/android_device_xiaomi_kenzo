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

#ifndef __MM_CAMERA_H__
#define __MM_CAMERA_H__

#include <cam_semaphore.h>

#include "mm_camera_interface.h"
#include <hardware/camera.h>
#include <utils/Timers.h>

/**********************************************************************************
* Data structure declare
***********************************************************************************/
/* num of callbacks allowed for an event type */
#define MM_CAMERA_EVT_ENTRY_MAX 4
/* num of data callbacks allowed in a stream obj */
#define MM_CAMERA_STREAM_BUF_CB_MAX 4
/* num of data poll threads allowed in a channel obj */
#define MM_CAMERA_CHANNEL_POLL_THREAD_MAX 1

#define MM_CAMERA_DEV_NAME_LEN 32
#define MM_CAMERA_DEV_OPEN_TRIES 2
#define MM_CAMERA_DEV_OPEN_RETRY_SLEEP 20
#define THREAD_NAME_SIZE 15

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))

struct mm_channel;
struct mm_stream;
struct mm_camera_obj;

typedef enum
{
    MM_CAMERA_CMD_TYPE_DATA_CB,    /* dataB CMD */
    MM_CAMERA_CMD_TYPE_EVT_CB,     /* evtCB CMD */
    MM_CAMERA_CMD_TYPE_EXIT,       /* EXIT */
    MM_CAMERA_CMD_TYPE_REQ_DATA_CB,/* request data */
    MM_CAMERA_CMD_TYPE_SUPER_BUF_DATA_CB,    /* superbuf dataB CMD */
    MM_CAMERA_CMD_TYPE_CONFIG_NOTIFY, /* configure notify mode */
    MM_CAMERA_CMD_TYPE_START_ZSL, /* start zsl snapshot for channel */
    MM_CAMERA_CMD_TYPE_STOP_ZSL, /* stop zsl snapshot for channel */
    MM_CAMERA_CMD_TYPE_FLUSH_QUEUE, /* flush queue */
    MM_CAMERA_CMD_TYPE_GENERAL,  /* general cmd */
    MM_CAMERA_CMD_TYPE_MAX
} mm_camera_cmdcb_type_t;

typedef struct {
    uint32_t stream_id;
    uint32_t frame_idx;
    uint32_t flags;
    mm_camera_buf_def_t *buf; /* ref to buf */
} mm_camera_buf_info_t;

typedef enum {
    MM_CAMERA_GENERIC_CMD_TYPE_AE_BRACKETING,
    MM_CAMERA_GENERIC_CMD_TYPE_AF_BRACKETING,
    MM_CAMERA_GENERIC_CMD_TYPE_FLASH_BRACKETING,
    MM_CAMERA_GENERIC_CMD_TYPE_ZOOM_1X,
    MM_CAMERA_GENERIC_CMD_TYPE_CAPTURE_SETTING,
} mm_camera_generic_cmd_type_t;

typedef struct {
    mm_camera_generic_cmd_type_t type;
    uint32_t payload[32];
    union {
        cam_capture_frame_config_t frame_config;
    };
} mm_camera_generic_cmd_t;

typedef struct {
    uint32_t frame_idx;
    cam_stream_type_t stream_type;
} mm_camera_flush_cmd_t;

typedef struct {
    mm_camera_cmdcb_type_t cmd_type;
    union {
        mm_camera_buf_info_t buf;    /* frame buf if dataCB */
        mm_camera_event_t evt;       /* evt if evtCB */
        mm_camera_super_buf_t superbuf; /* superbuf if superbuf dataCB*/
        mm_camera_req_buf_t req_buf; /* num of buf requested */
        mm_camera_flush_cmd_t flush_cmd; /* frame idx boundary for flush superbuf queue*/
        mm_camera_super_buf_notify_mode_t notify_mode; /* notification mode */
        mm_camera_generic_cmd_t gen_cmd;
    } u;
} mm_camera_cmdcb_t;

typedef void (*mm_camera_cmd_cb_t)(mm_camera_cmdcb_t * cmd_cb, void* user_data);

typedef struct {
    uint8_t is_active;     /*indicates whether thread is active or not */
    cam_queue_t cmd_queue; /* cmd queue (queuing dataCB, asyncCB, or exitCMD) */
    pthread_t cmd_pid;           /* cmd thread ID */
    cam_semaphore_t cmd_sem;     /* semaphore for cmd thread */
    cam_semaphore_t sync_sem;     /* semaphore for synchronization with cmd thread */
    mm_camera_cmd_cb_t cb;       /* cb for cmd */
    void* user_data;             /* user_data for cb */
    char threadName[THREAD_NAME_SIZE];
} mm_camera_cmd_thread_t;

typedef enum {
    MM_CAMERA_POLL_TYPE_EVT,
    MM_CAMERA_POLL_TYPE_DATA,
    MM_CAMERA_POLL_TYPE_MAX
} mm_camera_poll_thread_type_t;

/* function ptr defined for poll notify CB,
 * registered at poll thread with poll fd */
typedef void (*mm_camera_poll_notify_t)(void *user_data);

typedef struct {
    int32_t fd;
    mm_camera_poll_notify_t notify_cb;
    uint32_t handler;
    void* user_data;
} mm_camera_poll_entry_t;

typedef struct {
    mm_camera_poll_thread_type_t poll_type;
    /* array to store poll fd and cb info
     * for MM_CAMERA_POLL_TYPE_EVT, only index 0 is valid;
     * for MM_CAMERA_POLL_TYPE_DATA, depends on valid stream fd */
    mm_camera_poll_entry_t poll_entries[MAX_STREAM_NUM_IN_BUNDLE];
    int32_t pfds[2];
    pthread_t pid;
    int32_t state;
    int timeoutms;
    uint32_t cmd;
    struct pollfd poll_fds[MAX_STREAM_NUM_IN_BUNDLE + 1];
    uint8_t num_fds;
    pthread_mutex_t mutex;
    pthread_cond_t cond_v;
    int32_t status;
    char threadName[THREAD_NAME_SIZE];
    //void *my_obj;
} mm_camera_poll_thread_t;

/* mm_stream */
typedef enum {
    MM_STREAM_STATE_NOTUSED = 0,      /* not used */
    MM_STREAM_STATE_INITED,           /* inited  */
    MM_STREAM_STATE_ACQUIRED,         /* acquired, fd opened  */
    MM_STREAM_STATE_CFG,              /* fmt & dim configured */
    MM_STREAM_STATE_BUFFED,           /* buf allocated */
    MM_STREAM_STATE_REG,              /* buf regged, stream off */
    MM_STREAM_STATE_ACTIVE,           /* active */
    MM_STREAM_STATE_MAX
} mm_stream_state_type_t;

typedef enum {
    MM_STREAM_EVT_ACQUIRE,
    MM_STREAM_EVT_RELEASE,
    MM_STREAM_EVT_SET_FMT,
    MM_STREAM_EVT_GET_BUF,
    MM_STREAM_EVT_PUT_BUF,
    MM_STREAM_EVT_REG_BUF,
    MM_STREAM_EVT_UNREG_BUF,
    MM_STREAM_EVT_START,
    MM_STREAM_EVT_STOP,
    MM_STREAM_EVT_QBUF,
    MM_STREAM_EVT_SET_PARM,
    MM_STREAM_EVT_GET_PARM,
    MM_STREAM_EVT_DO_ACTION,
    MM_STREAM_EVT_GET_QUEUED_BUF_COUNT,
    MM_STREAM_EVT_MAX
} mm_stream_evt_type_t;

typedef struct {
    mm_camera_buf_notify_t cb;
    void *user_data;
    /* cb_count = -1: infinite
     * cb_count > 0: register only for required times */
    int8_t cb_count;
    mm_camera_stream_cb_type cb_type;
} mm_stream_data_cb_t;

typedef struct {
    /* buf reference count */
    uint8_t buf_refcnt;

    /* This flag is to indicate if after allocation,
     * the corresponding buf needs to qbuf into kernel
     * (e.g. for preview usecase, display needs to hold two bufs,
     * so no need to qbuf these two bufs initially) */
    uint8_t initial_reg_flag;

    /* indicate if buf is in kernel(1) or client(0) */
    uint8_t in_kernel;
    /*indicate if this buffer is mapped to daemon*/
    uint8_t is_mapped;
} mm_stream_buf_status_t;

typedef struct mm_stream {
    uint32_t my_hdl; /* local stream id */
    uint32_t server_stream_id; /* stream id from server */
    int32_t fd;
    mm_stream_state_type_t state;

    /* stream info*/
    cam_stream_info_t *stream_info;

    /* padding info */
    cam_padding_info_t padding_info;

    /* offset */
    cam_frame_len_offset_t frame_offset;

    pthread_mutex_t cmd_lock; /* lock to protect cmd_thread */
    mm_camera_cmd_thread_t cmd_thread;

    /* dataCB registered on this stream obj */
    pthread_mutex_t cb_lock; /* cb lock to protect buf_cb */
    mm_stream_data_cb_t buf_cb[MM_CAMERA_STREAM_BUF_CB_MAX];

    /* stream buffer management */
    pthread_mutex_t buf_lock;
    uint8_t buf_num; /* num of buffers allocated */
    mm_camera_buf_def_t* buf; /* ptr to buf array */
    mm_stream_buf_status_t buf_status[CAM_MAX_NUM_BUFS_PER_STREAM]; /* ptr to buf status array */

    uint8_t plane_buf_num; /* num of plane buffers allocated  Used only in Batch mode*/
    mm_camera_buf_def_t *plane_buf; /*Pointer to plane buffer array Used only in Batch mode */
    int32_t cur_buf_idx; /* Current container buffer active filling. Used only in Batch mode*/
    uint8_t cur_bufs_staged; /*Number of plane buf freed by HAL for this usr buf*/


    /* reference to parent channel_obj */
    struct mm_channel* ch_obj;

    uint8_t is_bundled; /* flag if stream is bundled */

    /* reference to linked channel_obj */
    struct mm_channel* linked_obj;
    struct mm_stream * linked_stream; /* original stream */
    uint8_t is_linked; /* flag if stream is linked */

    mm_camera_stream_mem_vtbl_t mem_vtbl; /* mem ops tbl */

    mm_camera_map_unmap_ops_tbl_t map_ops;

    int8_t queued_buffer_count;

    /*latest timestamp of this stream frame received & last frameID*/
    uint32_t prev_frameID;
    nsecs_t prev_timestamp;

    /* Need to wait for buffer mapping before stream-on*/
    pthread_cond_t buf_cond;
} mm_stream_t;

/* mm_channel */
typedef enum {
    MM_CHANNEL_STATE_NOTUSED = 0,   /* not used */
    MM_CHANNEL_STATE_STOPPED,       /* stopped */
    MM_CHANNEL_STATE_ACTIVE,        /* active, at least one stream active */
    MM_CHANNEL_STATE_PAUSED,        /* paused */
    MM_CHANNEL_STATE_MAX
} mm_channel_state_type_t;

typedef enum {
    MM_CHANNEL_EVT_ADD_STREAM,
    MM_CHANNEL_EVT_DEL_STREAM,
    MM_CHANNEL_EVT_LINK_STREAM,
    MM_CHANNEL_EVT_CONFIG_STREAM,
    MM_CHANNEL_EVT_GET_BUNDLE_INFO,
    MM_CHANNEL_EVT_START,
    MM_CHANNEL_EVT_STOP,
    MM_CHANNEL_EVT_PAUSE,
    MM_CHANNEL_EVT_RESUME,
    MM_CHANNEL_EVT_REQUEST_SUPER_BUF,
    MM_CHANNEL_EVT_CANCEL_REQUEST_SUPER_BUF,
    MM_CHANNEL_EVT_FLUSH_SUPER_BUF_QUEUE,
    MM_CHANNEL_EVT_CONFIG_NOTIFY_MODE,
    MM_CHANNEL_EVT_START_ZSL_SNAPSHOT,
    MM_CHANNEL_EVT_STOP_ZSL_SNAPSHOT,
    MM_CHANNEL_EVT_MAP_STREAM_BUF,
    MM_CHANNEL_EVT_UNMAP_STREAM_BUF,
    MM_CHANNEL_EVT_SET_STREAM_PARM,
    MM_CHANNEL_EVT_GET_STREAM_PARM,
    MM_CHANNEL_EVT_DO_STREAM_ACTION,
    MM_CHANNEL_EVT_DELETE,
    MM_CHANNEL_EVT_AF_BRACKETING,
    MM_CHANNEL_EVT_AE_BRACKETING,
    MM_CHANNEL_EVT_FLASH_BRACKETING,
    MM_CHANNEL_EVT_ZOOM_1X,
    MM_CAMERA_EVT_CAPTURE_SETTING,
    MM_CHANNEL_EVT_GET_STREAM_QUEUED_BUF_COUNT,
    MM_CHANNEL_EVT_MAP_STREAM_BUFS,
    MM_CHANNEL_EVT_REG_STREAM_BUF_CB
} mm_channel_evt_type_t;

typedef struct {
    uint32_t stream_id;
    mm_camera_stream_config_t *config;
} mm_evt_paylod_config_stream_t;

typedef struct {
    uint32_t stream_id;
    cam_stream_parm_buffer_t *parms;
} mm_evt_paylod_set_get_stream_parms_t;

typedef struct {
    uint32_t stream_id;
    void *actions;
} mm_evt_paylod_do_stream_action_t;

typedef struct {
    uint32_t stream_id;
    mm_stream_data_cb_t buf_cb;
} mm_evt_paylod_reg_stream_buf_cb;


typedef struct {
    uint8_t num_of_bufs;
    mm_camera_buf_info_t super_buf[MAX_STREAM_NUM_IN_BUNDLE];
    uint8_t matched;
    uint8_t expected;
    uint32_t frame_idx;
} mm_channel_queue_node_t;

typedef struct {
    cam_queue_t que;
    uint8_t num_streams;
    /* container for bundled stream handlers */
    uint32_t bundled_streams[MAX_STREAM_NUM_IN_BUNDLE];
    mm_camera_channel_attr_t attr;
    uint32_t expected_frame_id;
    uint32_t match_cnt;
    uint32_t expected_frame_id_without_led;
    uint32_t led_on_start_frame_id;
    uint32_t led_off_start_frame_id;
    uint32_t led_on_num_frames;
    uint32_t once;
    uint32_t frame_skip_count;
    uint32_t nomatch_frame_id;
    uint32_t frame_num_for_instant_capture;
} mm_channel_queue_t;

typedef struct {
    uint8_t is_active; /* flag to indicate if bundle is valid */
    /* queue to store bundled super buffers */
    mm_channel_queue_t superbuf_queue;
    mm_camera_buf_notify_t super_buf_notify_cb;
    void *user_data;
} mm_channel_bundle_t;

/* Nodes used for frame sync */
typedef struct {
    /* Frame idx */
    uint32_t frame_idx;
    /* Frame present for corresponding channel*/
    uint32_t frame_valid[MAX_NUM_CAMERA_PER_BUNDLE];
    /* Frame present in all channels*/
    uint32_t matched;
} mm_channel_sync_node_t;

/* Frame sync information */
typedef struct {
    /* Number of camera channels that need to be synced*/
    uint8_t num_cam;
    /* position of the next node to be updated */
    uint8_t pos;
    /* circular node array used to store frame information */
    mm_channel_sync_node_t node[MM_CAMERA_FRAME_SYNC_NODES];
    /* Channel corresponding to each camera */
    struct mm_channel *ch_obj[MAX_NUM_CAMERA_PER_BUNDLE];
    /* Cb corresponding to each camera */
    mm_camera_buf_notify_t cb[MAX_NUM_CAMERA_PER_BUNDLE];
} mm_channel_frame_sync_info_t;

/* Node information for multiple superbuf callbacks
*  This can be used to batch nodes before sending to upper layer */
typedef struct {
    /* Number of nodes to be sent*/
    uint8_t num_nodes;
    /* queue node information*/
    mm_channel_queue_node_t *node[MAX_NUM_CAMERA_PER_BUNDLE];
    /* channel information*/
    struct mm_channel *ch_obj[MAX_NUM_CAMERA_PER_BUNDLE];
} mm_channel_node_info_t;

typedef enum {
    MM_CHANNEL_BRACKETING_STATE_OFF,
    MM_CHANNEL_BRACKETING_STATE_WAIT_GOOD_FRAME_IDX,
    MM_CHANNEL_BRACKETING_STATE_ACTIVE,
} mm_channel_bracketing_state_t;

typedef struct mm_channel {
    uint32_t my_hdl;
    mm_channel_state_type_t state;
    pthread_mutex_t ch_lock; /* channel lock */

    /* stream bundle info in the channel */
    mm_channel_bundle_t bundle;

    /* num of pending suferbuffers */
    uint32_t pending_cnt;
    uint32_t pending_retro_cnt;
    mm_camera_req_buf_type_t req_type;
    uint32_t bWaitForPrepSnapshotDone;
    uint32_t unLockAEC;
    /* num of pending suferbuffers */
    uint8_t stopZslSnapshot;

    /* cmd thread for superbuffer dataCB and async stop*/
    mm_camera_cmd_thread_t cmd_thread;

    /* cb thread for sending data cb */
    mm_camera_cmd_thread_t cb_thread;

    /* data poll thread
    * currently one data poll thread per channel
    * could extended to support one data poll thread per stream in the channel */
    mm_camera_poll_thread_t poll_thread[MM_CAMERA_CHANNEL_POLL_THREAD_MAX];

    /* container for all streams in channel */
    mm_stream_t streams[MAX_STREAM_NUM_IN_BUNDLE];

    /* reference to parent cam_obj */
    struct mm_camera_obj* cam_obj;

    /* manual zsl snapshot control */
    uint8_t manualZSLSnapshot;

    /* control for zsl led */
    uint8_t startZSlSnapshotCalled;
    uint8_t needLEDFlash;
    mm_channel_bracketing_state_t bracketingState;
    uint8_t isFlashBracketingEnabled;
    uint8_t isZoom1xFrameRequested;
    uint32_t burstSnapNum;
    char threadName[THREAD_NAME_SIZE];

    /*Frame capture configaration*/
    uint8_t isConfigCapture;
    uint8_t cur_capture_idx;

    /*Buffer diverted*/
    uint8_t diverted_frame_id;
    uint32_t sessionid;

    uint8_t capture_frame_id[MAX_CAPTURE_BATCH_NUM];
    cam_capture_frame_config_t frameConfig;
} mm_channel_t;

typedef struct {
    mm_channel_t *ch;
    uint32_t stream_id;
} mm_camera_stream_link_t;

/* struct to store information about pp cookie*/
typedef struct {
    uint32_t cam_hdl;
    uint32_t ch_hdl;
    uint32_t stream_hdl;
    mm_channel_queue_node_t* super_buf;
} mm_channel_pp_info_t;

/* mm_camera */
typedef struct {
    mm_camera_event_notify_t evt_cb;
    void *user_data;
} mm_camera_evt_entry_t;

typedef struct {
    mm_camera_evt_entry_t evt[MM_CAMERA_EVT_ENTRY_MAX];
    /* reg_count <=0: infinite
     * reg_count > 0: register only for required times */
    int reg_count;
} mm_camera_evt_obj_t;

typedef struct mm_camera_obj {
    uint32_t my_hdl;
    int ref_count;
    int32_t ctrl_fd;
    int32_t ds_fd; /* domain socket fd */
    pthread_mutex_t cam_lock;
    pthread_mutex_t cb_lock; /* lock for evt cb */
    mm_channel_t ch[MM_CAMERA_CHANNEL_MAX];
    mm_camera_evt_obj_t evt;
    mm_camera_poll_thread_t evt_poll_thread; /* evt poll thread */
    mm_camera_cmd_thread_t evt_thread;       /* thread for evt CB */
    mm_camera_vtbl_t vtbl;

    pthread_mutex_t evt_lock;
    pthread_cond_t evt_cond;
    mm_camera_event_t evt_rcvd;

    pthread_mutex_t msg_lock; /* lock for sending msg through socket */
    uint32_t sessionid; /* Camera server session id */
} mm_camera_obj_t;

typedef struct {
    int8_t num_cam;
    char video_dev_name[MM_CAMERA_MAX_NUM_SENSORS][MM_CAMERA_DEV_NAME_LEN];
    mm_camera_obj_t *cam_obj[MM_CAMERA_MAX_NUM_SENSORS];
    struct camera_info info[MM_CAMERA_MAX_NUM_SENSORS];
    cam_sync_type_t cam_type[MM_CAMERA_MAX_NUM_SENSORS];
    cam_sync_mode_t cam_mode[MM_CAMERA_MAX_NUM_SENSORS];
    uint8_t is_yuv[MM_CAMERA_MAX_NUM_SENSORS]; // 1=CAM_SENSOR_YUV, 0=CAM_SENSOR_RAW
} mm_camera_ctrl_t;

typedef enum {
    mm_camera_async_call,
    mm_camera_sync_call
} mm_camera_call_type_t;

/**********************************************************************************
* external function declare
***********************************************************************************/
/* utility functions */
/* set int32_t value */
extern int32_t mm_camera_util_s_ctrl(int32_t fd,
                                     uint32_t id,
                                     int32_t *value);

/* get int32_t value */
extern int32_t mm_camera_util_g_ctrl(int32_t fd,
                                     uint32_t id,
                                     int32_t *value);

/* send msg throught domain socket for fd mapping */
extern int32_t mm_camera_util_sendmsg(mm_camera_obj_t *my_obj,
                                      void *msg,
                                      size_t buf_size,
                                      int sendfd);

/* send msg through domain socket for bundled fd mapping */
extern int32_t mm_camera_util_bundled_sendmsg(mm_camera_obj_t *my_obj,
                                              void *msg,
                                              size_t buf_size,
                                              int sendfds[CAM_MAX_NUM_BUFS_PER_STREAM],
                                              int numfds);

/* Check if hardware target is A family */
uint8_t mm_camera_util_chip_is_a_family(void);

/* mm-camera */
extern int32_t mm_camera_open(mm_camera_obj_t *my_obj);
extern int32_t mm_camera_close(mm_camera_obj_t *my_obj);
extern int32_t mm_camera_close_fd(mm_camera_obj_t *my_obj);
extern int32_t mm_camera_register_event_notify(mm_camera_obj_t *my_obj,
                                               mm_camera_event_notify_t evt_cb,
                                               void * user_data);
extern int32_t mm_camera_qbuf(mm_camera_obj_t *my_obj,
                              uint32_t ch_id,
                              mm_camera_buf_def_t *buf);
extern int32_t mm_camera_get_queued_buf_count(mm_camera_obj_t *my_obj,
        uint32_t ch_id, uint32_t stream_id);
extern int32_t mm_camera_query_capability(mm_camera_obj_t *my_obj);
extern int32_t mm_camera_set_parms(mm_camera_obj_t *my_obj,
                                   parm_buffer_t *parms);
extern int32_t mm_camera_get_parms(mm_camera_obj_t *my_obj,
                                   parm_buffer_t *parms);
extern int32_t mm_camera_map_buf(mm_camera_obj_t *my_obj,
                                 uint8_t buf_type,
                                 int fd,
                                 size_t size);
extern int32_t mm_camera_map_bufs(mm_camera_obj_t *my_obj,
                                  const cam_buf_map_type_list *buf_map_list);
extern int32_t mm_camera_unmap_buf(mm_camera_obj_t *my_obj,
                                   uint8_t buf_type);
extern int32_t mm_camera_do_auto_focus(mm_camera_obj_t *my_obj);
extern int32_t mm_camera_cancel_auto_focus(mm_camera_obj_t *my_obj);
extern int32_t mm_camera_prepare_snapshot(mm_camera_obj_t *my_obj,
                                          int32_t do_af_flag);
extern int32_t mm_camera_start_zsl_snapshot(mm_camera_obj_t *my_obj);
extern int32_t mm_camera_stop_zsl_snapshot(mm_camera_obj_t *my_obj);
extern int32_t mm_camera_flush(mm_camera_obj_t *my_obj);
extern int32_t mm_camera_start_zsl_snapshot_ch(mm_camera_obj_t *my_obj,
        uint32_t ch_id);
extern int32_t mm_camera_stop_zsl_snapshot_ch(mm_camera_obj_t *my_obj,
        uint32_t ch_id);
extern uint32_t mm_camera_add_channel(mm_camera_obj_t *my_obj,
                                      mm_camera_channel_attr_t *attr,
                                      mm_camera_buf_notify_t channel_cb,
                                      void *userdata);
extern int32_t mm_camera_del_channel(mm_camera_obj_t *my_obj,
                                     uint32_t ch_id);
extern int32_t mm_camera_get_bundle_info(mm_camera_obj_t *my_obj,
                                         uint32_t ch_id,
                                         cam_bundle_config_t *bundle_info);
extern uint32_t mm_camera_add_stream(mm_camera_obj_t *my_obj,
                                     uint32_t ch_id);
extern int32_t mm_camera_del_stream(mm_camera_obj_t *my_obj,
                                    uint32_t ch_id,
                                    uint32_t stream_id);
extern uint32_t mm_camera_link_stream(mm_camera_obj_t *my_obj,
        uint32_t ch_id,
        uint32_t stream_id,
        uint32_t linked_ch_id);

extern int32_t mm_camera_reg_stream_buf_cb(mm_camera_obj_t *my_obj,
        uint32_t ch_id, uint32_t stream_id, mm_camera_buf_notify_t buf_cb,
        mm_camera_stream_cb_type cb_type, void *userdata);

extern int32_t mm_camera_config_stream(mm_camera_obj_t *my_obj,
                                       uint32_t ch_id,
                                       uint32_t stream_id,
                                       mm_camera_stream_config_t *config);
extern int32_t mm_camera_start_channel(mm_camera_obj_t *my_obj,
                                       uint32_t ch_id);
extern int32_t mm_camera_stop_channel(mm_camera_obj_t *my_obj,
                                      uint32_t ch_id);
extern int32_t mm_camera_request_super_buf(mm_camera_obj_t *my_obj,
        uint32_t ch_id, mm_camera_req_buf_t *buf);
extern int32_t mm_camera_cancel_super_buf_request(mm_camera_obj_t *my_obj,
                                                  uint32_t ch_id);
extern int32_t mm_camera_flush_super_buf_queue(mm_camera_obj_t *my_obj,
                                               uint32_t ch_id,
                                               uint32_t frame_idx);
extern int32_t mm_camera_config_channel_notify(mm_camera_obj_t *my_obj,
                                               uint32_t ch_id,
                                               mm_camera_super_buf_notify_mode_t notify_mode);
extern int32_t mm_camera_set_stream_parms(mm_camera_obj_t *my_obj,
                                          uint32_t ch_id,
                                          uint32_t s_id,
                                          cam_stream_parm_buffer_t *parms);
extern int32_t mm_camera_get_stream_parms(mm_camera_obj_t *my_obj,
                                          uint32_t ch_id,
                                          uint32_t s_id,
                                          cam_stream_parm_buffer_t *parms);
extern int32_t mm_camera_register_event_notify_internal(mm_camera_obj_t *my_obj,
                                                        mm_camera_event_notify_t evt_cb,
                                                        void * user_data);
extern int32_t mm_camera_map_stream_buf(mm_camera_obj_t *my_obj,
                                        uint32_t ch_id,
                                        uint32_t stream_id,
                                        uint8_t buf_type,
                                        uint32_t buf_idx,
                                        int32_t plane_idx,
                                        int fd,
                                        size_t size);
extern int32_t mm_camera_map_stream_bufs(mm_camera_obj_t *my_obj,
                                         uint32_t ch_id,
                                         const cam_buf_map_type_list *buf_map_list);
extern int32_t mm_camera_unmap_stream_buf(mm_camera_obj_t *my_obj,
                                          uint32_t ch_id,
                                          uint32_t stream_id,
                                          uint8_t buf_type,
                                          uint32_t buf_idx,
                                          int32_t plane_idx);
extern int32_t mm_camera_do_stream_action(mm_camera_obj_t *my_obj,
                                          uint32_t ch_id,
                                          uint32_t stream_id,
                                          void *actions);
extern int32_t mm_camera_get_session_id(mm_camera_obj_t *my_obj,
                                        uint32_t* sessionid);
extern int32_t mm_camera_sync_related_sensors(mm_camera_obj_t *my_obj,
                                   cam_sync_related_sensors_event_info_t *parms);

/* mm_channel */
extern int32_t mm_channel_fsm_fn(mm_channel_t *my_obj,
                                 mm_channel_evt_type_t evt,
                                 void * in_val,
                                 void * out_val);
extern int32_t mm_channel_init(mm_channel_t *my_obj,
                               mm_camera_channel_attr_t *attr,
                               mm_camera_buf_notify_t channel_cb,
                               void *userdata);
/* qbuf is a special case that not going through state machine.
 * This is to avoid deadlock when trying to aquire ch_lock,
 * from the context of dataCB, but async stop is holding ch_lock */
extern int32_t mm_channel_qbuf(mm_channel_t *my_obj,
                               mm_camera_buf_def_t *buf);
/* mm_stream */
extern int32_t mm_stream_fsm_fn(mm_stream_t *my_obj,
                                mm_stream_evt_type_t evt,
                                void * in_val,
                                void * out_val);
/* Function to register special callback for stream buffer*/
extern int32_t mm_stream_reg_buf_cb(mm_stream_t *my_obj,
        mm_stream_data_cb_t val);
extern int32_t mm_stream_map_buf(mm_stream_t *my_obj,
                                 uint8_t buf_type,
                                 uint32_t frame_idx,
                                 int32_t plane_idx,
                                 int fd,
                                 size_t size);
extern int32_t mm_stream_map_bufs(mm_stream_t *my_obj,
                                  const cam_buf_map_type_list *buf_map_list);
extern int32_t mm_stream_unmap_buf(mm_stream_t *my_obj,
                                   uint8_t buf_type,
                                   uint32_t frame_idx,
                                   int32_t plane_idx);


/* utiltity fucntion declared in mm-camera-inteface2.c
 * and need be used by mm-camera and below*/
uint32_t mm_camera_util_generate_handler(uint8_t index);
const char * mm_camera_util_get_dev_name(uint32_t cam_handler);
uint8_t mm_camera_util_get_index_by_handler(uint32_t handler);

/* poll/cmd thread functions */
extern int32_t mm_camera_poll_thread_launch(
                                mm_camera_poll_thread_t * poll_cb,
                                mm_camera_poll_thread_type_t poll_type);
extern int32_t mm_camera_poll_thread_release(mm_camera_poll_thread_t *poll_cb);
extern int32_t mm_camera_poll_thread_add_poll_fd(
                                mm_camera_poll_thread_t * poll_cb,
                                uint32_t handler,
                                int32_t fd,
                                mm_camera_poll_notify_t nofity_cb,
                                void *userdata,
                                mm_camera_call_type_t);
extern int32_t mm_camera_poll_thread_del_poll_fd(
                                mm_camera_poll_thread_t * poll_cb,
                                uint32_t handler,
                                mm_camera_call_type_t);
extern int32_t mm_camera_poll_thread_commit_updates(
        mm_camera_poll_thread_t * poll_cb);
extern int32_t mm_camera_cmd_thread_launch(
                                mm_camera_cmd_thread_t * cmd_thread,
                                mm_camera_cmd_cb_t cb,
                                void* user_data);
extern int32_t mm_camera_cmd_thread_name(const char* name);
extern int32_t mm_camera_cmd_thread_release(mm_camera_cmd_thread_t * cmd_thread);

extern int32_t mm_camera_channel_advanced_capture(mm_camera_obj_t *my_obj,
        uint32_t ch_id, mm_camera_advanced_capture_t type,
        uint32_t trigger, void *in_value);
#endif /* __MM_CAMERA_H__ */
