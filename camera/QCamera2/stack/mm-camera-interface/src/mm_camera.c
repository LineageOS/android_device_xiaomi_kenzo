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

#include <pthread.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <poll.h>
#include <cutils/properties.h>
#include <stdlib.h>

#include <cam_semaphore.h>

#include "mm_camera_dbg.h"
#include "mm_camera_sock.h"
#include "mm_camera_interface.h"
#include "mm_camera.h"

#define SET_PARM_BIT32(parm, parm_arr) \
    (parm_arr[parm/32] |= (1<<(parm%32)))

#define GET_PARM_BIT32(parm, parm_arr) \
    ((parm_arr[parm/32]>>(parm%32))& 0x1)

#define WAIT_TIMEOUT 3

/* internal function declare */
int32_t mm_camera_evt_sub(mm_camera_obj_t * my_obj,
                          uint8_t reg_flag);
int32_t mm_camera_enqueue_evt(mm_camera_obj_t *my_obj,
                              mm_camera_event_t *event);

/*===========================================================================
 * FUNCTION   : mm_camera_util_get_channel_by_handler
 *
 * DESCRIPTION: utility function to get a channel object from its handle
 *
 * PARAMETERS :
 *   @cam_obj: ptr to a camera object
 *   @handler: channel handle
 *
 * RETURN     : ptr to a channel object.
 *              NULL if failed.
 *==========================================================================*/
mm_channel_t * mm_camera_util_get_channel_by_handler(
                                    mm_camera_obj_t * cam_obj,
                                    uint32_t handler)
{
    int i;
    mm_channel_t *ch_obj = NULL;
    for(i = 0; i < MM_CAMERA_CHANNEL_MAX; i++) {
        if (handler == cam_obj->ch[i].my_hdl) {
            ch_obj = &cam_obj->ch[i];
            break;
        }
    }
    return ch_obj;
}

/*===========================================================================
 * FUNCTION   : mm_camera_util_chip_is_a_family
 *
 * DESCRIPTION: utility function to check if the host is A family chip
 *
 * PARAMETERS :
 *
 * RETURN     : TRUE if A family.
 *              FALSE otherwise.
 *==========================================================================*/
uint8_t mm_camera_util_chip_is_a_family(void)
{
#ifdef USE_A_FAMILY
    return TRUE;
#else
    return FALSE;
#endif
}

/*===========================================================================
 * FUNCTION   : mm_camera_dispatch_app_event
 *
 * DESCRIPTION: dispatch event to apps who regitster for event notify
 *
 * PARAMETERS :
 *   @cmd_cb: ptr to a struct storing event info
 *   @user_data: user data ptr (camera object)
 *
 * RETURN     : none
 *==========================================================================*/
static void mm_camera_dispatch_app_event(mm_camera_cmdcb_t *cmd_cb,
                                         void* user_data)
{
    int i;
    mm_camera_event_t *event = &cmd_cb->u.evt;
    mm_camera_obj_t * my_obj = (mm_camera_obj_t *)user_data;
    if (NULL != my_obj) {
        mm_camera_cmd_thread_name(my_obj->evt_thread.threadName);
        pthread_mutex_lock(&my_obj->cb_lock);
        for(i = 0; i < MM_CAMERA_EVT_ENTRY_MAX; i++) {
            if(my_obj->evt.evt[i].evt_cb) {
                my_obj->evt.evt[i].evt_cb(
                    my_obj->my_hdl,
                    event,
                    my_obj->evt.evt[i].user_data);
            }
        }
        pthread_mutex_unlock(&my_obj->cb_lock);
    }
}

/*===========================================================================
 * FUNCTION   : mm_camera_event_notify
 *
 * DESCRIPTION: callback to handle event notify from kernel. This call will
 *              dequeue event from kernel.
 *
 * PARAMETERS :
 *   @user_data: user data ptr (camera object)
 *
 * RETURN     : none
 *==========================================================================*/
static void mm_camera_event_notify(void* user_data)
{
    struct v4l2_event ev;
    struct msm_v4l2_event_data *msm_evt = NULL;
    int rc;
    mm_camera_event_t evt;
    memset(&evt, 0, sizeof(mm_camera_event_t));

    mm_camera_obj_t *my_obj = (mm_camera_obj_t*)user_data;
    if (NULL != my_obj) {
        /* read evt */
        memset(&ev, 0, sizeof(ev));
        rc = ioctl(my_obj->ctrl_fd, VIDIOC_DQEVENT, &ev);

        if (rc >= 0 && ev.id == MSM_CAMERA_MSM_NOTIFY) {
            msm_evt = (struct msm_v4l2_event_data *)ev.u.data;
            switch (msm_evt->command) {
            case CAM_EVENT_TYPE_DAEMON_PULL_REQ:
                evt.server_event_type = CAM_EVENT_TYPE_DAEMON_PULL_REQ;
                mm_camera_enqueue_evt(my_obj, &evt);
                break;
            case CAM_EVENT_TYPE_MAP_UNMAP_DONE:
                pthread_mutex_lock(&my_obj->evt_lock);
                my_obj->evt_rcvd.server_event_type = msm_evt->command;
                my_obj->evt_rcvd.status = msm_evt->status;
                pthread_cond_signal(&my_obj->evt_cond);
                pthread_mutex_unlock(&my_obj->evt_lock);
                break;
            case CAM_EVENT_TYPE_INT_TAKE_JPEG:
            case CAM_EVENT_TYPE_INT_TAKE_RAW:
                {
                    evt.server_event_type = msm_evt->command;
                    mm_camera_enqueue_evt(my_obj, &evt);
                }
                break;
            case MSM_CAMERA_PRIV_SHUTDOWN:
                {
                    CDBG_ERROR("%s: Camera Event DAEMON DIED received", __func__);
                    evt.server_event_type = CAM_EVENT_TYPE_DAEMON_DIED;
                    mm_camera_enqueue_evt(my_obj, &evt);
                }
                break;
            case CAM_EVENT_TYPE_CAC_DONE:
                {
                    evt.server_event_type = CAM_EVENT_TYPE_CAC_DONE;
                    mm_camera_enqueue_evt(my_obj, &evt);
                }
                break;
            default:
                break;
            }
        }
    }
}

/*===========================================================================
 * FUNCTION   : mm_camera_enqueue_evt
 *
 * DESCRIPTION: enqueue received event into event queue to be processed by
 *              event thread.
 *
 * PARAMETERS :
 *   @my_obj   : ptr to a camera object
 *   @event    : event to be queued
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_camera_enqueue_evt(mm_camera_obj_t *my_obj,
                              mm_camera_event_t *event)
{
    int32_t rc = 0;
    mm_camera_cmdcb_t *node = NULL;

    node = (mm_camera_cmdcb_t *)malloc(sizeof(mm_camera_cmdcb_t));
    if (NULL != node) {
        memset(node, 0, sizeof(mm_camera_cmdcb_t));
        node->cmd_type = MM_CAMERA_CMD_TYPE_EVT_CB;
        node->u.evt = *event;

        /* enqueue to evt cmd thread */
        cam_queue_enq(&(my_obj->evt_thread.cmd_queue), node);
        /* wake up evt cmd thread */
        cam_sem_post(&(my_obj->evt_thread.cmd_sem));
    } else {
        CDBG_ERROR("%s: No memory for mm_camera_node_t", __func__);
        rc = -1;
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_open
 *
 * DESCRIPTION: open a camera
 *
 * PARAMETERS :
 *   @my_obj   : ptr to a camera object
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_camera_open(mm_camera_obj_t *my_obj)
{
    char dev_name[MM_CAMERA_DEV_NAME_LEN];
    int32_t rc = 0;
    int8_t n_try=MM_CAMERA_DEV_OPEN_TRIES;
    uint8_t sleep_msec=MM_CAMERA_DEV_OPEN_RETRY_SLEEP;
    int cam_idx = 0;
    const char *dev_name_value = NULL;
    char prop[PROPERTY_VALUE_MAX];
    uint32_t globalLogLevel = 0;
    int l_errno = 0;

    property_get("persist.camera.hal.debug", prop, "0");
    int val = atoi(prop);
    if (0 <= val) {
        gMmCameraIntfLogLevel = (uint32_t)val;
    }
    property_get("persist.camera.global.debug", prop, "0");
    val = atoi(prop);
    if (0 <= val) {
        globalLogLevel = (uint32_t)val;
    }

    /* Highest log level among hal.logs and global.logs is selected */
    if (gMmCameraIntfLogLevel < globalLogLevel)
        gMmCameraIntfLogLevel = globalLogLevel;

    CDBG("%s:  begin\n", __func__);

    if (NULL == my_obj) {
        goto on_error;
    }
    dev_name_value = mm_camera_util_get_dev_name(my_obj->my_hdl);
    if (NULL == dev_name_value) {
        goto on_error;
    }
    snprintf(dev_name, sizeof(dev_name), "/dev/%s",
             dev_name_value);
    sscanf(dev_name, "/dev/video%d", &cam_idx);
    CDBG("%s: dev name = %s, cam_idx = %d", __func__, dev_name, cam_idx);

    do{
        n_try--;
        my_obj->ctrl_fd = open(dev_name, O_RDWR | O_NONBLOCK);
        l_errno = errno;
        CDBG("%s:  ctrl_fd = %d, errno == %d", __func__, my_obj->ctrl_fd, l_errno);
        if((my_obj->ctrl_fd >= 0) || (l_errno != EIO) || (n_try <= 0 )) {
            CDBG("%s:  opened, break out while loop", __func__);
            break;
        }
        CDBG("%s:failed with I/O error retrying after %d milli-seconds",
             __func__, sleep_msec);
        usleep(sleep_msec * 1000U);
    }while (n_try > 0);

    if (my_obj->ctrl_fd < 0) {
        CDBG_ERROR("%s: cannot open control fd of '%s' (%s)\n",
                 __func__, dev_name, strerror(l_errno));
        if (l_errno == EBUSY)
            rc = -EUSERS;
        else
            rc = -1;
        goto on_error;
    }

    /* open domain socket*/
    n_try = MM_CAMERA_DEV_OPEN_TRIES;
    do {
        n_try--;
        my_obj->ds_fd = mm_camera_socket_create(cam_idx, MM_CAMERA_SOCK_TYPE_UDP);
        l_errno = errno;
        CDBG("%s:  ds_fd = %d, errno = %d", __func__, my_obj->ds_fd, l_errno);
        if((my_obj->ds_fd >= 0) || (n_try <= 0 )) {
            CDBG("%s:  opened, break out while loop", __func__);
            break;
        }
        CDBG("%s:failed with I/O error retrying after %d milli-seconds",
             __func__, sleep_msec);
        usleep(sleep_msec * 1000U);
    } while (n_try > 0);

    if (my_obj->ds_fd < 0) {
        CDBG_ERROR("%s: cannot open domain socket fd of '%s'(%s)\n",
                 __func__, dev_name, strerror(l_errno));
        rc = -1;
        goto on_error;
    }
    pthread_mutex_init(&my_obj->msg_lock, NULL);

    pthread_mutex_init(&my_obj->cb_lock, NULL);
    pthread_mutex_init(&my_obj->evt_lock, NULL);
    pthread_cond_init(&my_obj->evt_cond, NULL);

    CDBG("%s : Launch evt Thread in Cam Open",__func__);
    snprintf(my_obj->evt_thread.threadName, THREAD_NAME_SIZE, "CAM_Dispatch");
    mm_camera_cmd_thread_launch(&my_obj->evt_thread,
                                mm_camera_dispatch_app_event,
                                (void *)my_obj);

    /* launch event poll thread
     * we will add evt fd into event poll thread upon user first register for evt */
    CDBG("%s : Launch evt Poll Thread in Cam Open", __func__);
    snprintf(my_obj->evt_poll_thread.threadName, THREAD_NAME_SIZE, "CAM_evntPoll");
    mm_camera_poll_thread_launch(&my_obj->evt_poll_thread,
                                 MM_CAMERA_POLL_TYPE_EVT);
    mm_camera_evt_sub(my_obj, TRUE);

    /* unlock cam_lock, we need release global intf_lock in camera_open(),
     * in order not block operation of other Camera in dual camera use case.*/
    pthread_mutex_unlock(&my_obj->cam_lock);
    CDBG("%s:  end (rc = %d)\n", __func__, rc);
    return rc;

on_error:

    if (NULL == dev_name_value) {
        CDBG_ERROR("%s: Invalid device name\n", __func__);
        rc = -1;
    }

    if (NULL == my_obj) {
        CDBG_ERROR("%s: Invalid camera object\n", __func__);
        rc = -1;
    } else {
        if (my_obj->ctrl_fd >= 0) {
            close(my_obj->ctrl_fd);
            my_obj->ctrl_fd = -1;
        }
        if (my_obj->ds_fd >= 0) {
            mm_camera_socket_close(my_obj->ds_fd);
            my_obj->ds_fd = -1;
        }
    }

    /* unlock cam_lock, we need release global intf_lock in camera_open(),
     * in order not block operation of other Camera in dual camera use case.*/
    pthread_mutex_unlock(&my_obj->cam_lock);
    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_close
 *
 * DESCRIPTION: enqueue received event into event queue to be processed by
 *              event thread.
 *
 * PARAMETERS :
 *   @my_obj   : ptr to a camera object
 *   @event    : event to be queued
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_camera_close(mm_camera_obj_t *my_obj)
{
    CDBG("%s : unsubscribe evt", __func__);
    mm_camera_evt_sub(my_obj, FALSE);

    CDBG("%s : Close evt Poll Thread in Cam Close",__func__);
    mm_camera_poll_thread_release(&my_obj->evt_poll_thread);

    CDBG("%s : Close evt cmd Thread in Cam Close",__func__);
    mm_camera_cmd_thread_release(&my_obj->evt_thread);

    if(my_obj->ctrl_fd >= 0) {
        close(my_obj->ctrl_fd);
        my_obj->ctrl_fd = -1;
    }
    if(my_obj->ds_fd >= 0) {
        mm_camera_socket_close(my_obj->ds_fd);
        my_obj->ds_fd = -1;
    }
    pthread_mutex_destroy(&my_obj->msg_lock);

    pthread_mutex_destroy(&my_obj->cb_lock);
    pthread_mutex_destroy(&my_obj->evt_lock);
    pthread_cond_destroy(&my_obj->evt_cond);

    pthread_mutex_unlock(&my_obj->cam_lock);
    return 0;
}

/*===========================================================================
 * FUNCTION   : mm_camera_close_fd
 *
 * DESCRIPTION: close the ctrl_fd and socket fd in case of an error so that
 *              the backend will close
 *              Do NOT close or release any HAL resources since a close_camera
 *              has not been called yet.
 * PARAMETERS :
 *   @my_obj   : ptr to a camera object
 *   @event    : event to be queued
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_camera_close_fd(mm_camera_obj_t *my_obj)
{
    if(my_obj->ctrl_fd >= 0) {
        close(my_obj->ctrl_fd);
        my_obj->ctrl_fd = -1;
    }
    if(my_obj->ds_fd >= 0) {
        mm_camera_socket_close(my_obj->ds_fd);
        my_obj->ds_fd = -1;
    }
    pthread_mutex_unlock(&my_obj->cam_lock);
    return 0;
}

/*===========================================================================
 * FUNCTION   : mm_camera_register_event_notify_internal
 *
 * DESCRIPTION: internal implementation for registering callback for event notify.
 *
 * PARAMETERS :
 *   @my_obj   : ptr to a camera object
 *   @evt_cb   : callback to be registered to handle event notify
 *   @user_data: user data ptr
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_camera_register_event_notify_internal(mm_camera_obj_t *my_obj,
                                                 mm_camera_event_notify_t evt_cb,
                                                 void * user_data)
{
    int i;
    int rc = -1;
    mm_camera_evt_obj_t *evt_array = NULL;

    pthread_mutex_lock(&my_obj->cb_lock);
    evt_array = &my_obj->evt;
    if(evt_cb) {
        /* this is reg case */
        for(i = 0; i < MM_CAMERA_EVT_ENTRY_MAX; i++) {
            if(evt_array->evt[i].user_data == NULL) {
                evt_array->evt[i].evt_cb = evt_cb;
                evt_array->evt[i].user_data = user_data;
                evt_array->reg_count++;
                rc = 0;
                break;
            }
        }
    } else {
        /* this is unreg case */
        for(i = 0; i < MM_CAMERA_EVT_ENTRY_MAX; i++) {
            if(evt_array->evt[i].user_data == user_data) {
                evt_array->evt[i].evt_cb = NULL;
                evt_array->evt[i].user_data = NULL;
                evt_array->reg_count--;
                rc = 0;
                break;
            }
        }
    }

    pthread_mutex_unlock(&my_obj->cb_lock);
    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_register_event_notify
 *
 * DESCRIPTION: registering a callback for event notify.
 *
 * PARAMETERS :
 *   @my_obj   : ptr to a camera object
 *   @evt_cb   : callback to be registered to handle event notify
 *   @user_data: user data ptr
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_camera_register_event_notify(mm_camera_obj_t *my_obj,
                                        mm_camera_event_notify_t evt_cb,
                                        void * user_data)
{
    int rc = -1;
    rc = mm_camera_register_event_notify_internal(my_obj,
                                                  evt_cb,
                                                  user_data);
    pthread_mutex_unlock(&my_obj->cam_lock);
    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_qbuf
 *
 * DESCRIPTION: enqueue buffer back to kernel
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *   @ch_id        : channel handle
 *   @buf          : buf ptr to be enqueued
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_camera_qbuf(mm_camera_obj_t *my_obj,
                       uint32_t ch_id,
                       mm_camera_buf_def_t *buf)
{
    int rc = -1;
    mm_channel_t * ch_obj = NULL;
    ch_obj = mm_camera_util_get_channel_by_handler(my_obj, ch_id);

    pthread_mutex_unlock(&my_obj->cam_lock);

    /* we always assume qbuf will be done before channel/stream is fully stopped
     * because qbuf is done within dataCB context
     * in order to avoid deadlock, we are not locking ch_lock for qbuf */
    if (NULL != ch_obj) {
        rc = mm_channel_qbuf(ch_obj, buf);
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_get_queued_buf_count
 *
 * DESCRIPTION: return queued buffer count
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *   @ch_id        : channel handle
 *   @stream_id : stream id
 *
 * RETURN     : queued buffer count
 *==========================================================================*/
int32_t mm_camera_get_queued_buf_count(mm_camera_obj_t *my_obj,
        uint32_t ch_id, uint32_t stream_id)
{
    int rc = -1;
    mm_channel_t * ch_obj = NULL;
    uint32_t payload;
    ch_obj = mm_camera_util_get_channel_by_handler(my_obj, ch_id);
    payload = stream_id;

    if (NULL != ch_obj) {
        pthread_mutex_lock(&ch_obj->ch_lock);
        pthread_mutex_unlock(&my_obj->cam_lock);
        rc = mm_channel_fsm_fn(ch_obj,
                MM_CHANNEL_EVT_GET_STREAM_QUEUED_BUF_COUNT,
                (void *)&payload,
                NULL);
    } else {
        pthread_mutex_unlock(&my_obj->cam_lock);
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_query_capability
 *
 * DESCRIPTION: query camera capability
 *
 * PARAMETERS :
 *   @my_obj: camera object
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_camera_query_capability(mm_camera_obj_t *my_obj)
{
    int32_t rc = 0;
    struct v4l2_capability cap;

    /* get camera capabilities */
    memset(&cap, 0, sizeof(cap));
    rc = ioctl(my_obj->ctrl_fd, VIDIOC_QUERYCAP, &cap);
    if (rc != 0) {
        CDBG_ERROR("%s: cannot get camera capabilities, rc = %d\n", __func__, rc);
    }

    pthread_mutex_unlock(&my_obj->cam_lock);
    return rc;

}

/*===========================================================================
 * FUNCTION   : mm_camera_set_parms
 *
 * DESCRIPTION: set parameters per camera
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *   @parms        : ptr to a param struct to be set to server
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 * NOTE       : Assume the parms struct buf is already mapped to server via
 *              domain socket. Corresponding fields of parameters to be set
 *              are already filled in by upper layer caller.
 *==========================================================================*/
int32_t mm_camera_set_parms(mm_camera_obj_t *my_obj,
                            parm_buffer_t *parms)
{
    int32_t rc = -1;
    int32_t value = 0;
    if (parms !=  NULL) {
        rc = mm_camera_util_s_ctrl(my_obj->ctrl_fd, CAM_PRIV_PARM, &value);
    }
    pthread_mutex_unlock(&my_obj->cam_lock);
    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_get_parms
 *
 * DESCRIPTION: get parameters per camera
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *   @parms        : ptr to a param struct to be get from server
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 * NOTE       : Assume the parms struct buf is already mapped to server via
 *              domain socket. Parameters to be get from server are already
 *              filled in by upper layer caller. After this call, corresponding
 *              fields of requested parameters will be filled in by server with
 *              detailed information.
 *==========================================================================*/
int32_t mm_camera_get_parms(mm_camera_obj_t *my_obj,
                            parm_buffer_t *parms)
{
    int32_t rc = -1;
    int32_t value = 0;
    if (parms != NULL) {
        rc = mm_camera_util_g_ctrl(my_obj->ctrl_fd, CAM_PRIV_PARM, &value);
    }
    pthread_mutex_unlock(&my_obj->cam_lock);
    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_do_auto_focus
 *
 * DESCRIPTION: performing auto focus
 *
 * PARAMETERS :
 *   @camera_handle: camera handle
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 * NOTE       : if this call success, we will always assume there will
 *              be an auto_focus event following up.
 *==========================================================================*/
int32_t mm_camera_do_auto_focus(mm_camera_obj_t *my_obj)
{
    int32_t rc = -1;
    int32_t value = 0;
    rc = mm_camera_util_s_ctrl(my_obj->ctrl_fd, CAM_PRIV_DO_AUTO_FOCUS, &value);
    pthread_mutex_unlock(&my_obj->cam_lock);
    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_cancel_auto_focus
 *
 * DESCRIPTION: cancel auto focus
 *
 * PARAMETERS :
 *   @camera_handle: camera handle
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_camera_cancel_auto_focus(mm_camera_obj_t *my_obj)
{
    int32_t rc = -1;
    int32_t value = 0;
    rc = mm_camera_util_s_ctrl(my_obj->ctrl_fd, CAM_PRIV_CANCEL_AUTO_FOCUS, &value);
    pthread_mutex_unlock(&my_obj->cam_lock);
    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_prepare_snapshot
 *
 * DESCRIPTION: prepare hardware for snapshot
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *   @do_af_flag   : flag indicating if AF is needed
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_camera_prepare_snapshot(mm_camera_obj_t *my_obj,
                                   int32_t do_af_flag)
{
    int32_t rc = -1;
    int32_t value = do_af_flag;
    rc = mm_camera_util_s_ctrl(my_obj->ctrl_fd, CAM_PRIV_PREPARE_SNAPSHOT, &value);
    pthread_mutex_unlock(&my_obj->cam_lock);
    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_start_zsl_snapshot
 *
 * DESCRIPTION: start zsl snapshot
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_camera_start_zsl_snapshot(mm_camera_obj_t *my_obj)
{
    int32_t rc = -1;
    int32_t value = 0;

    rc = mm_camera_util_s_ctrl(my_obj->ctrl_fd,
             CAM_PRIV_START_ZSL_SNAPSHOT, &value);
    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_stop_zsl_snapshot
 *
 * DESCRIPTION: stop zsl capture
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_camera_stop_zsl_snapshot(mm_camera_obj_t *my_obj)
{
    int32_t rc = -1;
    int32_t value;
    rc = mm_camera_util_s_ctrl(my_obj->ctrl_fd,
             CAM_PRIV_STOP_ZSL_SNAPSHOT, &value);
    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_flush
 *
 * DESCRIPTION: flush the current camera state and buffers
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_camera_flush(mm_camera_obj_t *my_obj)
{
    int32_t rc = -1;
    int32_t value;
    rc = mm_camera_util_s_ctrl(my_obj->ctrl_fd,
            CAM_PRIV_FLUSH, &value);
    pthread_mutex_unlock(&my_obj->cam_lock);
    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_add_channel
 *
 * DESCRIPTION: add a channel
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *   @attr         : bundle attribute of the channel if needed
 *   @channel_cb   : callback function for bundle data notify
 *   @userdata     : user data ptr
 *
 * RETURN     : uint32_t type of channel handle
 *              0  -- invalid channel handle, meaning the op failed
 *              >0 -- successfully added a channel with a valid handle
 * NOTE       : if no bundle data notify is needed, meaning each stream in the
 *              channel will have its own stream data notify callback, then
 *              attr, channel_cb, and userdata can be NULL. In this case,
 *              no matching logic will be performed in channel for the bundling.
 *==========================================================================*/
uint32_t mm_camera_add_channel(mm_camera_obj_t *my_obj,
                               mm_camera_channel_attr_t *attr,
                               mm_camera_buf_notify_t channel_cb,
                               void *userdata)
{
    mm_channel_t *ch_obj = NULL;
    uint8_t ch_idx = 0;
    uint32_t ch_hdl = 0;

    for(ch_idx = 0; ch_idx < MM_CAMERA_CHANNEL_MAX; ch_idx++) {
        if (MM_CHANNEL_STATE_NOTUSED == my_obj->ch[ch_idx].state) {
            ch_obj = &my_obj->ch[ch_idx];
            break;
        }
    }

    if (NULL != ch_obj) {
        /* initialize channel obj */
        memset(ch_obj, 0, sizeof(mm_channel_t));
        ch_hdl = mm_camera_util_generate_handler(ch_idx);
        ch_obj->my_hdl = ch_hdl;
        ch_obj->state = MM_CHANNEL_STATE_STOPPED;
        ch_obj->cam_obj = my_obj;
        pthread_mutex_init(&ch_obj->ch_lock, NULL);
        ch_obj->sessionid = my_obj->sessionid;
        mm_channel_init(ch_obj, attr, channel_cb, userdata);
    }

    pthread_mutex_unlock(&my_obj->cam_lock);

    return ch_hdl;
}

/*===========================================================================
 * FUNCTION   : mm_camera_del_channel
 *
 * DESCRIPTION: delete a channel by its handle
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *   @ch_id        : channel handle
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 * NOTE       : all streams in the channel should be stopped already before
 *              this channel can be deleted.
 *==========================================================================*/
int32_t mm_camera_del_channel(mm_camera_obj_t *my_obj,
                              uint32_t ch_id)
{
    int32_t rc = -1;
    mm_channel_t * ch_obj =
        mm_camera_util_get_channel_by_handler(my_obj, ch_id);

    if (NULL != ch_obj) {
        pthread_mutex_lock(&ch_obj->ch_lock);
        pthread_mutex_unlock(&my_obj->cam_lock);

        rc = mm_channel_fsm_fn(ch_obj,
                               MM_CHANNEL_EVT_DELETE,
                               NULL,
                               NULL);

        pthread_mutex_destroy(&ch_obj->ch_lock);
        memset(ch_obj, 0, sizeof(mm_channel_t));
    } else {
        pthread_mutex_unlock(&my_obj->cam_lock);
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_get_bundle_info
 *
 * DESCRIPTION: query bundle info of the channel
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *   @ch_id        : channel handle
 *   @bundle_info  : bundle info to be filled in
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 * NOTE       : all streams in the channel should be stopped already before
 *              this channel can be deleted.
 *==========================================================================*/
int32_t mm_camera_get_bundle_info(mm_camera_obj_t *my_obj,
                                  uint32_t ch_id,
                                  cam_bundle_config_t *bundle_info)
{
    int32_t rc = -1;
    mm_channel_t * ch_obj =
        mm_camera_util_get_channel_by_handler(my_obj, ch_id);

    if (NULL != ch_obj) {
        pthread_mutex_lock(&ch_obj->ch_lock);
        pthread_mutex_unlock(&my_obj->cam_lock);

        rc = mm_channel_fsm_fn(ch_obj,
                               MM_CHANNEL_EVT_GET_BUNDLE_INFO,
                               (void *)bundle_info,
                               NULL);
    } else {
        pthread_mutex_unlock(&my_obj->cam_lock);
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_link_stream
 *
 * DESCRIPTION: link a stream into a channel
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *   @ch_id        : channel handle
 *   @stream_id    : stream that will be linked
 *   @linked_ch_id : channel in which the stream will be linked
 *
 * RETURN     : uint32_t type of stream handle
 *              0  -- invalid stream handle, meaning the op failed
 *              >0 -- successfully linked a stream with a valid handle
 *==========================================================================*/
uint32_t mm_camera_link_stream(mm_camera_obj_t *my_obj,
        uint32_t ch_id,
        uint32_t stream_id,
        uint32_t linked_ch_id)
{
    uint32_t s_hdl = 0;
    mm_channel_t * ch_obj =
            mm_camera_util_get_channel_by_handler(my_obj, linked_ch_id);
    mm_channel_t * owner_obj =
            mm_camera_util_get_channel_by_handler(my_obj, ch_id);

    if ((NULL != ch_obj) && (NULL != owner_obj)) {
        pthread_mutex_lock(&ch_obj->ch_lock);
        pthread_mutex_unlock(&my_obj->cam_lock);

        mm_camera_stream_link_t stream_link;
        memset(&stream_link, 0, sizeof(mm_camera_stream_link_t));
        stream_link.ch = owner_obj;
        stream_link.stream_id = stream_id;
        mm_channel_fsm_fn(ch_obj,
                          MM_CHANNEL_EVT_LINK_STREAM,
                          (void*)&stream_link,
                          (void*)&s_hdl);
    } else {
        pthread_mutex_unlock(&my_obj->cam_lock);
    }

    return s_hdl;
}

/*===========================================================================
 * FUNCTION   : mm_camera_add_stream
 *
 * DESCRIPTION: add a stream into a channel
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *   @ch_id        : channel handle
 *
 * RETURN     : uint32_t type of stream handle
 *              0  -- invalid stream handle, meaning the op failed
 *              >0 -- successfully added a stream with a valid handle
 *==========================================================================*/
uint32_t mm_camera_add_stream(mm_camera_obj_t *my_obj,
                              uint32_t ch_id)
{
    uint32_t s_hdl = 0;
    mm_channel_t * ch_obj =
        mm_camera_util_get_channel_by_handler(my_obj, ch_id);

    if (NULL != ch_obj) {
        pthread_mutex_lock(&ch_obj->ch_lock);
        pthread_mutex_unlock(&my_obj->cam_lock);

        mm_channel_fsm_fn(ch_obj,
                          MM_CHANNEL_EVT_ADD_STREAM,
                          NULL,
                          (void *)&s_hdl);
    } else {
        pthread_mutex_unlock(&my_obj->cam_lock);
    }

    return s_hdl;
}

/*===========================================================================
 * FUNCTION   : mm_camera_del_stream
 *
 * DESCRIPTION: delete a stream by its handle
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *   @ch_id        : channel handle
 *   @stream_id    : stream handle
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 * NOTE       : stream should be stopped already before it can be deleted.
 *==========================================================================*/
int32_t mm_camera_del_stream(mm_camera_obj_t *my_obj,
                             uint32_t ch_id,
                             uint32_t stream_id)
{
    int32_t rc = -1;
    mm_channel_t * ch_obj =
        mm_camera_util_get_channel_by_handler(my_obj, ch_id);

    if (NULL != ch_obj) {
        pthread_mutex_lock(&ch_obj->ch_lock);
        pthread_mutex_unlock(&my_obj->cam_lock);

        rc = mm_channel_fsm_fn(ch_obj,
                               MM_CHANNEL_EVT_DEL_STREAM,
                               (void *)&stream_id,
                               NULL);
    } else {
        pthread_mutex_unlock(&my_obj->cam_lock);
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_start_zsl_snapshot_ch
 *
 * DESCRIPTION: starts zsl snapshot for specific channel
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *   @ch_id        : channel handle
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_camera_start_zsl_snapshot_ch(mm_camera_obj_t *my_obj,
        uint32_t ch_id)
{
    int32_t rc = -1;
    mm_channel_t * ch_obj =
        mm_camera_util_get_channel_by_handler(my_obj, ch_id);

    if (NULL != ch_obj) {
        pthread_mutex_lock(&ch_obj->ch_lock);
        pthread_mutex_unlock(&my_obj->cam_lock);

        rc = mm_channel_fsm_fn(ch_obj,
                               MM_CHANNEL_EVT_START_ZSL_SNAPSHOT,
                               NULL,
                               NULL);
    } else {
        pthread_mutex_unlock(&my_obj->cam_lock);
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_stop_zsl_snapshot_ch
 *
 * DESCRIPTION: stops zsl snapshot for specific channel
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *   @ch_id        : channel handle
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_camera_stop_zsl_snapshot_ch(mm_camera_obj_t *my_obj,
        uint32_t ch_id)
{
    int32_t rc = -1;
    mm_channel_t * ch_obj =
        mm_camera_util_get_channel_by_handler(my_obj, ch_id);

    if (NULL != ch_obj) {
        pthread_mutex_lock(&ch_obj->ch_lock);
        pthread_mutex_unlock(&my_obj->cam_lock);

        rc = mm_channel_fsm_fn(ch_obj,
                               MM_CHANNEL_EVT_STOP_ZSL_SNAPSHOT,
                               NULL,
                               NULL);
    } else {
        pthread_mutex_unlock(&my_obj->cam_lock);
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_config_stream
 *
 * DESCRIPTION: configure a stream
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *   @ch_id        : channel handle
 *   @stream_id    : stream handle
 *   @config       : stream configuration
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_camera_config_stream(mm_camera_obj_t *my_obj,
                                uint32_t ch_id,
                                uint32_t stream_id,
                                mm_camera_stream_config_t *config)
{
    int32_t rc = -1;
    mm_channel_t * ch_obj =
        mm_camera_util_get_channel_by_handler(my_obj, ch_id);
    mm_evt_paylod_config_stream_t payload;

    if (NULL != ch_obj) {
        pthread_mutex_lock(&ch_obj->ch_lock);
        pthread_mutex_unlock(&my_obj->cam_lock);

        memset(&payload, 0, sizeof(mm_evt_paylod_config_stream_t));
        payload.stream_id = stream_id;
        payload.config = config;
        rc = mm_channel_fsm_fn(ch_obj,
                               MM_CHANNEL_EVT_CONFIG_STREAM,
                               (void *)&payload,
                               NULL);
    } else {
        pthread_mutex_unlock(&my_obj->cam_lock);
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_start_channel
 *
 * DESCRIPTION: start a channel, which will start all streams in the channel
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *   @ch_id        : channel handle
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_camera_start_channel(mm_camera_obj_t *my_obj, uint32_t ch_id)
{
    int32_t rc = -1;
    mm_channel_t * ch_obj =
        mm_camera_util_get_channel_by_handler(my_obj, ch_id);

    if (NULL != ch_obj) {
        pthread_mutex_lock(&ch_obj->ch_lock);
        pthread_mutex_unlock(&my_obj->cam_lock);

        rc = mm_channel_fsm_fn(ch_obj,
                               MM_CHANNEL_EVT_START,
                               NULL,
                               NULL);
    } else {
        pthread_mutex_unlock(&my_obj->cam_lock);
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_stop_channel
 *
 * DESCRIPTION: stop a channel, which will stop all streams in the channel
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *   @ch_id        : channel handle
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_camera_stop_channel(mm_camera_obj_t *my_obj,
                               uint32_t ch_id)
{
    int32_t rc = 0;
    mm_channel_t * ch_obj =
        mm_camera_util_get_channel_by_handler(my_obj, ch_id);

    if (NULL != ch_obj) {
        pthread_mutex_lock(&ch_obj->ch_lock);
        pthread_mutex_unlock(&my_obj->cam_lock);

        rc = mm_channel_fsm_fn(ch_obj,
                               MM_CHANNEL_EVT_STOP,
                               NULL,
                               NULL);
    } else {
        pthread_mutex_unlock(&my_obj->cam_lock);
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_request_super_buf
 *
 * DESCRIPTION: for burst mode in bundle, reuqest certain amount of matched
 *              frames from superbuf queue
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *   @ch_id        : channel handle
 *   @num_buf_requested : number of matched frames needed
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_camera_request_super_buf(mm_camera_obj_t *my_obj,
        uint32_t ch_id, mm_camera_req_buf_t *buf)
{
    int32_t rc = -1;
    mm_channel_t * ch_obj =
        mm_camera_util_get_channel_by_handler(my_obj, ch_id);

    if ((NULL != ch_obj) && (buf != NULL)) {
        pthread_mutex_lock(&ch_obj->ch_lock);
        pthread_mutex_unlock(&my_obj->cam_lock);

        rc = mm_channel_fsm_fn(ch_obj, MM_CHANNEL_EVT_REQUEST_SUPER_BUF,
                (void *)buf, NULL);
    } else {
        pthread_mutex_unlock(&my_obj->cam_lock);
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_cancel_super_buf_request
 *
 * DESCRIPTION: for burst mode in bundle, cancel the reuqest for certain amount
 *              of matched frames from superbuf queue
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *   @ch_id        : channel handle
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_camera_cancel_super_buf_request(mm_camera_obj_t *my_obj, uint32_t ch_id)
{
    int32_t rc = -1;
    mm_channel_t * ch_obj =
        mm_camera_util_get_channel_by_handler(my_obj, ch_id);

    if (NULL != ch_obj) {
        pthread_mutex_lock(&ch_obj->ch_lock);
        pthread_mutex_unlock(&my_obj->cam_lock);

        rc = mm_channel_fsm_fn(ch_obj,
                               MM_CHANNEL_EVT_CANCEL_REQUEST_SUPER_BUF,
                               NULL,
                               NULL);
    } else {
        pthread_mutex_unlock(&my_obj->cam_lock);
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_flush_super_buf_queue
 *
 * DESCRIPTION: flush out all frames in the superbuf queue
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *   @ch_id        : channel handle
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_camera_flush_super_buf_queue(mm_camera_obj_t *my_obj, uint32_t ch_id,
                                                             uint32_t frame_idx)
{
    int32_t rc = -1;
    mm_channel_t * ch_obj =
        mm_camera_util_get_channel_by_handler(my_obj, ch_id);

    if (NULL != ch_obj) {
        pthread_mutex_lock(&ch_obj->ch_lock);
        pthread_mutex_unlock(&my_obj->cam_lock);

        rc = mm_channel_fsm_fn(ch_obj,
                               MM_CHANNEL_EVT_FLUSH_SUPER_BUF_QUEUE,
                               (void *)&frame_idx,
                               NULL);
    } else {
        pthread_mutex_unlock(&my_obj->cam_lock);
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_config_channel_notify
 *
 * DESCRIPTION: configures the channel notification mode
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *   @ch_id        : channel handle
 *   @notify_mode  : notification mode
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_camera_config_channel_notify(mm_camera_obj_t *my_obj,
                                        uint32_t ch_id,
                                        mm_camera_super_buf_notify_mode_t notify_mode)
{
    int32_t rc = -1;
    mm_channel_t * ch_obj =
        mm_camera_util_get_channel_by_handler(my_obj, ch_id);

    if (NULL != ch_obj) {
        pthread_mutex_lock(&ch_obj->ch_lock);
        pthread_mutex_unlock(&my_obj->cam_lock);

        rc = mm_channel_fsm_fn(ch_obj,
                               MM_CHANNEL_EVT_CONFIG_NOTIFY_MODE,
                               (void *)&notify_mode,
                               NULL);
    } else {
        pthread_mutex_unlock(&my_obj->cam_lock);
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_set_stream_parms
 *
 * DESCRIPTION: set parameters per stream
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *   @ch_id        : channel handle
 *   @s_id         : stream handle
 *   @parms        : ptr to a param struct to be set to server
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 * NOTE       : Assume the parms struct buf is already mapped to server via
 *              domain socket. Corresponding fields of parameters to be set
 *              are already filled in by upper layer caller.
 *==========================================================================*/
int32_t mm_camera_set_stream_parms(mm_camera_obj_t *my_obj,
                                   uint32_t ch_id,
                                   uint32_t s_id,
                                   cam_stream_parm_buffer_t *parms)
{
    int32_t rc = -1;
    mm_evt_paylod_set_get_stream_parms_t payload;
    mm_channel_t * ch_obj =
        mm_camera_util_get_channel_by_handler(my_obj, ch_id);

    if (NULL != ch_obj) {
        pthread_mutex_lock(&ch_obj->ch_lock);
        pthread_mutex_unlock(&my_obj->cam_lock);

        memset(&payload, 0, sizeof(payload));
        payload.stream_id = s_id;
        payload.parms = parms;

        rc = mm_channel_fsm_fn(ch_obj,
                               MM_CHANNEL_EVT_SET_STREAM_PARM,
                               (void *)&payload,
                               NULL);
    } else {
        pthread_mutex_unlock(&my_obj->cam_lock);
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_get_stream_parms
 *
 * DESCRIPTION: get parameters per stream
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *   @ch_id        : channel handle
 *   @s_id         : stream handle
 *   @parms        : ptr to a param struct to be get from server
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 * NOTE       : Assume the parms struct buf is already mapped to server via
 *              domain socket. Parameters to be get from server are already
 *              filled in by upper layer caller. After this call, corresponding
 *              fields of requested parameters will be filled in by server with
 *              detailed information.
 *==========================================================================*/
int32_t mm_camera_get_stream_parms(mm_camera_obj_t *my_obj,
                                   uint32_t ch_id,
                                   uint32_t s_id,
                                   cam_stream_parm_buffer_t *parms)
{
    int32_t rc = -1;
    mm_evt_paylod_set_get_stream_parms_t payload;
    mm_channel_t * ch_obj =
        mm_camera_util_get_channel_by_handler(my_obj, ch_id);

    if (NULL != ch_obj) {
        pthread_mutex_lock(&ch_obj->ch_lock);
        pthread_mutex_unlock(&my_obj->cam_lock);

        memset(&payload, 0, sizeof(payload));
        payload.stream_id = s_id;
        payload.parms = parms;

        rc = mm_channel_fsm_fn(ch_obj,
                               MM_CHANNEL_EVT_GET_STREAM_PARM,
                               (void *)&payload,
                               NULL);
    } else {
        pthread_mutex_unlock(&my_obj->cam_lock);
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_do_stream_action
 *
 * DESCRIPTION: request server to perform stream based action. Maybe removed later
 *              if the functionality is included in mm_camera_set_parms
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *   @ch_id        : channel handle
 *   @s_id         : stream handle
 *   @actions      : ptr to an action struct buf to be performed by server
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 * NOTE       : Assume the action struct buf is already mapped to server via
 *              domain socket. Actions to be performed by server are already
 *              filled in by upper layer caller.
 *==========================================================================*/
int32_t mm_camera_do_stream_action(mm_camera_obj_t *my_obj,
                                   uint32_t ch_id,
                                   uint32_t stream_id,
                                   void *actions)
{
    int32_t rc = -1;
    mm_evt_paylod_do_stream_action_t payload;
    mm_channel_t * ch_obj =
        mm_camera_util_get_channel_by_handler(my_obj, ch_id);

    if (NULL != ch_obj) {
        pthread_mutex_lock(&ch_obj->ch_lock);
        pthread_mutex_unlock(&my_obj->cam_lock);

        memset(&payload, 0, sizeof(payload));
        payload.stream_id = stream_id;
        payload.actions = actions;

        rc = mm_channel_fsm_fn(ch_obj,
                               MM_CHANNEL_EVT_DO_STREAM_ACTION,
                               (void*)&payload,
                               NULL);
    } else {
        pthread_mutex_unlock(&my_obj->cam_lock);
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_map_stream_buf
 *
 * DESCRIPTION: mapping stream buffer via domain socket to server
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *   @ch_id        : channel handle
 *   @s_id         : stream handle
 *   @buf_type     : type of buffer to be mapped. could be following values:
 *                   CAM_MAPPING_BUF_TYPE_STREAM_BUF
 *                   CAM_MAPPING_BUF_TYPE_STREAM_INFO
 *                   CAM_MAPPING_BUF_TYPE_OFFLINE_INPUT_BUF
 *   @buf_idx      : index of buffer within the stream buffers, only valid if
 *                   buf_type is CAM_MAPPING_BUF_TYPE_STREAM_BUF or
 *                   CAM_MAPPING_BUF_TYPE_OFFLINE_INPUT_BUF
 *   @plane_idx    : plane index. If all planes share the same fd,
 *                   plane_idx = -1; otherwise, plean_idx is the
 *                   index to plane (0..num_of_planes)
 *   @fd           : file descriptor of the buffer
 *   @size         : size of the buffer
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_camera_map_stream_buf(mm_camera_obj_t *my_obj,
                                 uint32_t ch_id,
                                 uint32_t stream_id,
                                 uint8_t buf_type,
                                 uint32_t buf_idx,
                                 int32_t plane_idx,
                                 int fd,
                                 size_t size)
{
    int32_t rc = -1;
    cam_buf_map_type payload;
    mm_channel_t * ch_obj =
        mm_camera_util_get_channel_by_handler(my_obj, ch_id);

    if (NULL != ch_obj) {
        pthread_mutex_lock(&ch_obj->ch_lock);
        pthread_mutex_unlock(&my_obj->cam_lock);

        memset(&payload, 0, sizeof(payload));
        payload.stream_id = stream_id;
        payload.type = buf_type;
        payload.frame_idx = buf_idx;
        payload.plane_idx = plane_idx;
        payload.fd = fd;
        payload.size = size;
        rc = mm_channel_fsm_fn(ch_obj,
                               MM_CHANNEL_EVT_MAP_STREAM_BUF,
                               (void*)&payload,
                               NULL);
    } else {
        pthread_mutex_unlock(&my_obj->cam_lock);
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_map_stream_bufs
 *
 * DESCRIPTION: mapping stream buffers via domain socket to server
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *   @ch_id        : channel handle
 *   @buf_map_list : list of buffers to be mapped
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_camera_map_stream_bufs(mm_camera_obj_t *my_obj,
                                  uint32_t ch_id,
                                  const cam_buf_map_type_list *buf_map_list)
{
    int32_t rc = -1;
    cam_buf_map_type_list payload;
    mm_channel_t * ch_obj =
        mm_camera_util_get_channel_by_handler(my_obj, ch_id);

    if (NULL != ch_obj) {
        pthread_mutex_lock(&ch_obj->ch_lock);
        pthread_mutex_unlock(&my_obj->cam_lock);

        memcpy(&payload, buf_map_list, sizeof(payload));
        rc = mm_channel_fsm_fn(ch_obj,
                               MM_CHANNEL_EVT_MAP_STREAM_BUFS,
                               (void*)&payload,
                               NULL);
    } else {
        pthread_mutex_unlock(&my_obj->cam_lock);
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_unmap_stream_buf
 *
 * DESCRIPTION: unmapping stream buffer via domain socket to server
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *   @ch_id        : channel handle
 *   @s_id         : stream handle
 *   @buf_type     : type of buffer to be mapped. could be following values:
 *                   CAM_MAPPING_BUF_TYPE_STREAM_BUF
 *                   CAM_MAPPING_BUF_TYPE_STREAM_INFO
 *                   CAM_MAPPING_BUF_TYPE_OFFLINE_INPUT_BUF
 *   @buf_idx      : index of buffer within the stream buffers, only valid if
 *                   buf_type is CAM_MAPPING_BUF_TYPE_STREAM_BUF or
 *                   CAM_MAPPING_BUF_TYPE_OFFLINE_INPUT_BUF
 *   @plane_idx    : plane index. If all planes share the same fd,
 *                   plane_idx = -1; otherwise, plean_idx is the
 *                   index to plane (0..num_of_planes)
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_camera_unmap_stream_buf(mm_camera_obj_t *my_obj,
                                   uint32_t ch_id,
                                   uint32_t stream_id,
                                   uint8_t buf_type,
                                   uint32_t buf_idx,
                                   int32_t plane_idx)
{
    int32_t rc = -1;
    cam_buf_unmap_type payload;
    mm_channel_t * ch_obj =
        mm_camera_util_get_channel_by_handler(my_obj, ch_id);

    if (NULL != ch_obj) {
        pthread_mutex_lock(&ch_obj->ch_lock);
        pthread_mutex_unlock(&my_obj->cam_lock);

        memset(&payload, 0, sizeof(payload));
        payload.stream_id = stream_id;
        payload.type = buf_type;
        payload.frame_idx = buf_idx;
        payload.plane_idx = plane_idx;
        rc = mm_channel_fsm_fn(ch_obj,
                               MM_CHANNEL_EVT_UNMAP_STREAM_BUF,
                               (void*)&payload,
                               NULL);
    } else {
        pthread_mutex_unlock(&my_obj->cam_lock);
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_evt_sub
 *
 * DESCRIPTION: subscribe/unsubscribe event notify from kernel
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *   @reg_flag     : 1 -- subscribe ; 0 -- unsubscribe
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_camera_evt_sub(mm_camera_obj_t * my_obj,
                          uint8_t reg_flag)
{
    int32_t rc = 0;
    struct v4l2_event_subscription sub;

    memset(&sub, 0, sizeof(sub));
    sub.type = MSM_CAMERA_V4L2_EVENT_TYPE;
    sub.id = MSM_CAMERA_MSM_NOTIFY;
    if(FALSE == reg_flag) {
        /* unsubscribe */
        rc = ioctl(my_obj->ctrl_fd, VIDIOC_UNSUBSCRIBE_EVENT, &sub);
        if (rc < 0) {
            CDBG_ERROR("%s: unsubscribe event rc = %d", __func__, rc);
            return rc;
        }
        /* remove evt fd from the polling thraed when unreg the last event */
        rc = mm_camera_poll_thread_del_poll_fd(&my_obj->evt_poll_thread,
                                               my_obj->my_hdl,
                                               mm_camera_sync_call);
    } else {
        rc = ioctl(my_obj->ctrl_fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
        if (rc < 0) {
            CDBG_ERROR("%s: subscribe event rc = %d", __func__, rc);
            return rc;
        }
        /* add evt fd to polling thread when subscribe the first event */
        rc = mm_camera_poll_thread_add_poll_fd(&my_obj->evt_poll_thread,
                                               my_obj->my_hdl,
                                               my_obj->ctrl_fd,
                                               mm_camera_event_notify,
                                               (void*)my_obj,
                                               mm_camera_sync_call);
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_util_wait_for_event
 *
 * DESCRIPTION: utility function to wait for certain events
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *   @evt_mask     : mask for events to be waited. Any of event in the mask would
 *                   trigger the wait to end
 *   @status       : status of the event
 *
 * RETURN     : none
 *==========================================================================*/
void mm_camera_util_wait_for_event(mm_camera_obj_t *my_obj,
                                   uint32_t evt_mask,
                                   uint32_t *status)
{
    int rc = 0;
    struct timespec ts;

    pthread_mutex_lock(&my_obj->evt_lock);
    while (!(my_obj->evt_rcvd.server_event_type & evt_mask)) {
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += WAIT_TIMEOUT;
        rc = pthread_cond_timedwait(&my_obj->evt_cond, &my_obj->evt_lock, &ts);
        if (rc == ETIMEDOUT) {
            ALOGE("%s pthread_cond_timedwait success\n", __func__);
            break;
        }
    }
    *status = my_obj->evt_rcvd.status;
    /* reset local storage for recieved event for next event */
    memset(&my_obj->evt_rcvd, 0, sizeof(mm_camera_event_t));
    pthread_mutex_unlock(&my_obj->evt_lock);
}

/*===========================================================================
 * FUNCTION   : mm_camera_util_bundled_sendmsg
 *
 * DESCRIPTION: utility function to send bundled msg via domain socket
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *   @msg          : message to be sent
 *   @buf_size     : size of the message to be sent
 *   @sendfds      : array of file descriptors to be sent
 *   @numfds       : number of file descriptors to be sent
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_camera_util_bundled_sendmsg(mm_camera_obj_t *my_obj,
                                       void *msg,
                                       size_t buf_size,
                                       int sendfds[CAM_MAX_NUM_BUFS_PER_STREAM],
                                       int numfds)
{
    int32_t rc = -1;
    uint32_t status;

    /* need to lock msg_lock, since sendmsg until response back is deemed as one operation*/
    pthread_mutex_lock(&my_obj->msg_lock);
    if(mm_camera_socket_bundle_sendmsg(my_obj->ds_fd, msg, buf_size, sendfds, numfds) > 0) {
        /* wait for event that mapping/unmapping is done */
        mm_camera_util_wait_for_event(my_obj, CAM_EVENT_TYPE_MAP_UNMAP_DONE, &status);
        if (MSM_CAMERA_STATUS_SUCCESS == status) {
            rc = 0;
        }
    }
    pthread_mutex_unlock(&my_obj->msg_lock);
    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_util_sendmsg
 *
 * DESCRIPTION: utility function to send msg via domain socket
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *   @msg          : message to be sent
 *   @buf_size     : size of the message to be sent
 *   @sendfd       : >0 if any file descriptor need to be passed across process
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_camera_util_sendmsg(mm_camera_obj_t *my_obj,
                               void *msg,
                               size_t buf_size,
                               int sendfd)
{
    int32_t rc = -1;
    uint32_t status;

    /* need to lock msg_lock, since sendmsg until reposonse back is deemed as one operation*/
    pthread_mutex_lock(&my_obj->msg_lock);
    if(mm_camera_socket_sendmsg(my_obj->ds_fd, msg, buf_size, sendfd) > 0) {
        /* wait for event that mapping/unmapping is done */
        mm_camera_util_wait_for_event(my_obj, CAM_EVENT_TYPE_MAP_UNMAP_DONE, &status);
        if (MSM_CAMERA_STATUS_SUCCESS == status) {
            rc = 0;
        }
    }
    pthread_mutex_unlock(&my_obj->msg_lock);
    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_map_buf
 *
 * DESCRIPTION: mapping camera buffer via domain socket to server
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *   @buf_type     : type of buffer to be mapped. could be following values:
 *                   CAM_MAPPING_BUF_TYPE_CAPABILITY
 *                   CAM_MAPPING_BUF_TYPE_SETPARM_BUF
 *                   CAM_MAPPING_BUF_TYPE_GETPARM_BUF
 *   @fd           : file descriptor of the buffer
 *   @size         : size of the buffer
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_camera_map_buf(mm_camera_obj_t *my_obj,
                          uint8_t buf_type,
                          int fd,
                          size_t size)
{
    int32_t rc = 0;
    cam_sock_packet_t packet;
    memset(&packet, 0, sizeof(cam_sock_packet_t));
    packet.msg_type = CAM_MAPPING_TYPE_FD_MAPPING;
    packet.payload.buf_map.type = buf_type;
    packet.payload.buf_map.fd = fd;
    packet.payload.buf_map.size = size;
    rc = mm_camera_util_sendmsg(my_obj,
                                &packet,
                                sizeof(cam_sock_packet_t),
                                fd);
    pthread_mutex_unlock(&my_obj->cam_lock);
    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_map_bufs
 *
 * DESCRIPTION: mapping camera buffers via domain socket to server
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *   @buf_map_list : list of buffers to be mapped
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_camera_map_bufs(mm_camera_obj_t *my_obj,
                           const cam_buf_map_type_list* buf_map_list)
{
    int32_t rc = 0;
    cam_sock_packet_t packet;
    memset(&packet, 0, sizeof(cam_sock_packet_t));
    packet.msg_type = CAM_MAPPING_TYPE_FD_BUNDLED_MAPPING;

    memcpy(&packet.payload.buf_map_list, buf_map_list,
           sizeof(packet.payload.buf_map_list));

    int sendfds[CAM_MAX_NUM_BUFS_PER_STREAM];
    uint32_t numbufs = packet.payload.buf_map_list.length;
    uint32_t i;
    for (i = 0; i < numbufs; i++) {
        sendfds[i] = packet.payload.buf_map_list.buf_maps[i].fd;
    }

    for (i = numbufs; i < CAM_MAX_NUM_BUFS_PER_STREAM; i++) {
        packet.payload.buf_map_list.buf_maps[i].fd = -1;
        sendfds[i] = -1;
    }

    rc = mm_camera_util_bundled_sendmsg(my_obj,
                                        &packet,
                                        sizeof(cam_sock_packet_t),
                                        sendfds,
                                        numbufs);

    pthread_mutex_unlock(&my_obj->cam_lock);
    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_unmap_buf
 *
 * DESCRIPTION: unmapping camera buffer via domain socket to server
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *   @buf_type     : type of buffer to be mapped. could be following values:
 *                   CAM_MAPPING_BUF_TYPE_CAPABILITY
 *                   CAM_MAPPING_BUF_TYPE_SETPARM_BUF
 *                   CAM_MAPPING_BUF_TYPE_GETPARM_BUF
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_camera_unmap_buf(mm_camera_obj_t *my_obj,
                            uint8_t buf_type)
{
    int32_t rc = 0;
    cam_sock_packet_t packet;
    memset(&packet, 0, sizeof(cam_sock_packet_t));
    packet.msg_type = CAM_MAPPING_TYPE_FD_UNMAPPING;
    packet.payload.buf_unmap.type = buf_type;
    rc = mm_camera_util_sendmsg(my_obj,
                                &packet,
                                sizeof(cam_sock_packet_t),
                                -1);
    pthread_mutex_unlock(&my_obj->cam_lock);
    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_util_s_ctrl
 *
 * DESCRIPTION: utility function to send v4l2 ioctl for s_ctrl
 *
 * PARAMETERS :
 *   @fd      : file descritpor for sending ioctl
 *   @id      : control id
 *   @value   : value of the ioctl to be sent
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_camera_util_s_ctrl(int32_t fd,  uint32_t id, int32_t *value)
{
    int rc = 0;
    struct v4l2_control control;

    memset(&control, 0, sizeof(control));
    control.id = id;
    if (value != NULL) {
        control.value = *value;
    }
    rc = ioctl(fd, VIDIOC_S_CTRL, &control);

    CDBG("%s: fd=%d, S_CTRL, id=0x%x, value = %p, rc = %d\n",
         __func__, fd, id, value, rc);
    if (value != NULL) {
        *value = control.value;
    }
    return (rc >= 0)? 0 : -1;
}

/*===========================================================================
 * FUNCTION   : mm_camera_util_g_ctrl
 *
 * DESCRIPTION: utility function to send v4l2 ioctl for g_ctrl
 *
 * PARAMETERS :
 *   @fd      : file descritpor for sending ioctl
 *   @id      : control id
 *   @value   : value of the ioctl to be sent
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_camera_util_g_ctrl( int32_t fd, uint32_t id, int32_t *value)
{
    int rc = 0;
    struct v4l2_control control;

    memset(&control, 0, sizeof(control));
    control.id = id;
    if (value != NULL) {
        control.value = *value;
    }
    rc = ioctl(fd, VIDIOC_G_CTRL, &control);
    CDBG("%s: fd=%d, G_CTRL, id=0x%x, rc = %d\n", __func__, fd, id, rc);
    if (value != NULL) {
        *value = control.value;
    }
    return (rc >= 0)? 0 : -1;
}

/*===========================================================================
 * FUNCTION   : mm_camera_channel_advanced_capture
 *
 * DESCRIPTION: sets the channel advanced capture
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *   @ch_id        : channel handle
  *   @type : advanced capture type.
 *   @start_flag  : flag to indicate start/stop
  *   @in_value  : input configaration
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_camera_channel_advanced_capture(mm_camera_obj_t *my_obj,
            uint32_t ch_id, mm_camera_advanced_capture_t type,
            uint32_t trigger, void *in_value)
{
    CDBG("%s: E type = %d",__func__, type);
    int32_t rc = -1;
    mm_channel_t * ch_obj =
        mm_camera_util_get_channel_by_handler(my_obj, ch_id);

    if (NULL != ch_obj) {
        pthread_mutex_lock(&ch_obj->ch_lock);
        pthread_mutex_unlock(&my_obj->cam_lock);
        switch (type) {
            case MM_CAMERA_AF_BRACKETING:
                rc = mm_channel_fsm_fn(ch_obj,
                                       MM_CHANNEL_EVT_AF_BRACKETING,
                                       (void *)&trigger,
                                       NULL);
                break;
            case MM_CAMERA_AE_BRACKETING:
                rc = mm_channel_fsm_fn(ch_obj,
                                       MM_CHANNEL_EVT_AE_BRACKETING,
                                       (void *)&trigger,
                                       NULL);
                break;
            case MM_CAMERA_FLASH_BRACKETING:
                rc = mm_channel_fsm_fn(ch_obj,
                                       MM_CHANNEL_EVT_FLASH_BRACKETING,
                                       (void *)&trigger,
                                       NULL);
                break;
            case MM_CAMERA_ZOOM_1X:
                rc = mm_channel_fsm_fn(ch_obj,
                                       MM_CHANNEL_EVT_ZOOM_1X,
                                       (void *)&trigger,
                                       NULL);
                break;
            case MM_CAMERA_FRAME_CAPTURE:
                rc = mm_channel_fsm_fn(ch_obj,
                                       MM_CAMERA_EVT_CAPTURE_SETTING,
                                       (void *)in_value,
                                       NULL);
                break;
            default:
                break;
        }

    } else {
        pthread_mutex_unlock(&my_obj->cam_lock);
    }

    CDBG("%s: X",__func__);
    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_get_session_id
 *
 * DESCRIPTION: get the session identity
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *   @sessionid: pointer to the output session id
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 * NOTE       : if this call succeeds, we will get a valid session id
 *==========================================================================*/
int32_t mm_camera_get_session_id(mm_camera_obj_t *my_obj,
        uint32_t* sessionid)
{
    int32_t rc = -1;
    int32_t value = 0;
    if(sessionid != NULL) {
        rc = mm_camera_util_g_ctrl(my_obj->ctrl_fd,
                MSM_CAMERA_PRIV_G_SESSION_ID, &value);
        CDBG("%s: fd=%d, get_session_id, id=0x%x, value = %d, rc = %d\n",
                __func__, my_obj->ctrl_fd, MSM_CAMERA_PRIV_G_SESSION_ID,
                value, rc);
        *sessionid = value;
        my_obj->sessionid = value;
    }

    pthread_mutex_unlock(&my_obj->cam_lock);
    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_sync_related_sensors
 *
 * DESCRIPTION: send sync cmd
 *
 * PARAMETERS :
 *   @my_obj       : camera object
 *   @parms        : ptr to the related cam info to be sent to server
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 * NOTE       : Assume the sync struct buf is already mapped to server via
 *              domain socket. Corresponding fields of parameters to be set
 *              are already filled in by upper layer caller.
 *==========================================================================*/
int32_t mm_camera_sync_related_sensors(mm_camera_obj_t *my_obj,
        cam_sync_related_sensors_event_info_t* parms)
{
    int32_t rc = -1;
    int32_t value = 0;
    if (parms !=  NULL) {
        rc = mm_camera_util_s_ctrl(my_obj->ctrl_fd,
                CAM_PRIV_SYNC_RELATED_SENSORS, &value);
    }
    pthread_mutex_unlock(&my_obj->cam_lock);
    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_camera_reg_stream_buf_cb
 *
 * DESCRIPTION: Register callback for stream buffer
 *
 * PARAMETERS :
 *   @my_obj    : camera object
 *   @ch_id     : channel handle
 *   @stream_id : stream that will be linked
 *   @buf_cb    : special callback needs to be registered for stream buffer
 *   @cb_type   : Callback type SYNC/ASYNC
 *   @userdata  : user data pointer
 *
 * RETURN    : int32_t type of status
 *             0  -- success
 *             1 --  failure
 *==========================================================================*/
int32_t mm_camera_reg_stream_buf_cb(mm_camera_obj_t *my_obj,
        uint32_t ch_id, uint32_t stream_id, mm_camera_buf_notify_t stream_cb,
        mm_camera_stream_cb_type cb_type, void *userdata)
{
    int rc = 0;
    mm_stream_t *stream = NULL;
    mm_stream_data_cb_t buf_cb;
    mm_channel_t * ch_obj =
            mm_camera_util_get_channel_by_handler(my_obj, ch_id);

    if (NULL != ch_obj) {
        pthread_mutex_lock(&ch_obj->ch_lock);
        pthread_mutex_unlock(&my_obj->cam_lock);

        memset(&buf_cb, 0, sizeof(mm_stream_data_cb_t));
        buf_cb.cb = stream_cb;
        buf_cb.cb_count = -1;
        buf_cb.cb_type = cb_type;
        buf_cb.user_data = userdata;

        mm_evt_paylod_reg_stream_buf_cb payload;
        memset(&payload, 0, sizeof(mm_evt_paylod_reg_stream_buf_cb));
        payload.buf_cb = buf_cb;
        payload.stream_id = stream_id;
        mm_channel_fsm_fn(ch_obj,
                MM_CHANNEL_EVT_REG_STREAM_BUF_CB,
                (void*)&payload, NULL);
    } else {
        pthread_mutex_unlock(&my_obj->cam_lock);
    }
    return rc;
}

