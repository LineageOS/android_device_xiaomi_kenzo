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
#include <cam_semaphore.h>

#include "mm_camera_dbg.h"
#include "mm_camera_interface.h"
#include "mm_camera.h"

extern mm_camera_obj_t* mm_camera_util_get_camera_by_handler(uint32_t cam_handler);
extern mm_channel_t * mm_camera_util_get_channel_by_handler(mm_camera_obj_t * cam_obj,
                                                            uint32_t handler);
/* Static frame sync info used between different camera channels*/
static mm_channel_frame_sync_info_t fs = { .num_cam =0, .pos = 0};
/* Frame sync info access lock */
static pthread_mutex_t fs_lock = PTHREAD_MUTEX_INITIALIZER;

/* internal function declare goes here */
int32_t mm_channel_qbuf(mm_channel_t *my_obj,
                        mm_camera_buf_def_t *buf);
int32_t mm_channel_init(mm_channel_t *my_obj,
                        mm_camera_channel_attr_t *attr,
                        mm_camera_buf_notify_t channel_cb,
                        void *userdata);
void mm_channel_release(mm_channel_t *my_obj);
uint32_t mm_channel_add_stream(mm_channel_t *my_obj);
int32_t mm_channel_del_stream(mm_channel_t *my_obj,
                                   uint32_t stream_id);
uint32_t mm_channel_link_stream(mm_channel_t *my_obj,
        mm_camera_stream_link_t *stream_link);
int32_t mm_channel_config_stream(mm_channel_t *my_obj,
                                 uint32_t stream_id,
                                 mm_camera_stream_config_t *config);
int32_t mm_channel_get_bundle_info(mm_channel_t *my_obj,
                                   cam_bundle_config_t *bundle_info);
int32_t mm_channel_start(mm_channel_t *my_obj);
int32_t mm_channel_stop(mm_channel_t *my_obj);
int32_t mm_channel_request_super_buf(mm_channel_t *my_obj,
        mm_camera_req_buf_t *buf);
int32_t mm_channel_cancel_super_buf_request(mm_channel_t *my_obj);
int32_t mm_channel_flush_super_buf_queue(mm_channel_t *my_obj,
                                         uint32_t frame_idx,
                                         cam_stream_type_t stream_type);
int32_t mm_channel_config_notify_mode(mm_channel_t *my_obj,
                                      mm_camera_super_buf_notify_mode_t notify_mode);
int32_t mm_channel_start_zsl_snapshot(mm_channel_t *my_obj);
int32_t mm_channel_stop_zsl_snapshot(mm_channel_t *my_obj);
int32_t mm_channel_superbuf_flush(mm_channel_t* my_obj,
        mm_channel_queue_t * queue, cam_stream_type_t cam_type);
int32_t mm_channel_set_stream_parm(mm_channel_t *my_obj,
                                   mm_evt_paylod_set_get_stream_parms_t *payload);
int32_t mm_channel_get_queued_buf_count(mm_channel_t *my_obj,
        uint32_t stream_id);

int32_t mm_channel_get_stream_parm(mm_channel_t *my_obj,
                                   mm_evt_paylod_set_get_stream_parms_t *payload);
int32_t mm_channel_do_stream_action(mm_channel_t *my_obj,
                                    mm_evt_paylod_do_stream_action_t *payload);
int32_t mm_channel_map_stream_buf(mm_channel_t *my_obj,
                                  cam_buf_map_type *payload);
int32_t mm_channel_map_stream_bufs(mm_channel_t *my_obj,
                                   cam_buf_map_type_list *payload);
int32_t mm_channel_unmap_stream_buf(mm_channel_t *my_obj,
                                    cam_buf_unmap_type *payload);

/* state machine function declare */
int32_t mm_channel_fsm_fn_notused(mm_channel_t *my_obj,
                          mm_channel_evt_type_t evt,
                          void * in_val,
                          void * out_val);
int32_t mm_channel_fsm_fn_stopped(mm_channel_t *my_obj,
                          mm_channel_evt_type_t evt,
                          void * in_val,
                          void * out_val);
int32_t mm_channel_fsm_fn_active(mm_channel_t *my_obj,
                          mm_channel_evt_type_t evt,
                          void * in_val,
                          void * out_val);
int32_t mm_channel_fsm_fn_paused(mm_channel_t *my_obj,
                          mm_channel_evt_type_t evt,
                          void * in_val,
                          void * out_val);

/* channel super queue functions */
int32_t mm_channel_superbuf_queue_init(mm_channel_queue_t * queue);
int32_t mm_channel_superbuf_queue_deinit(mm_channel_queue_t * queue);
int32_t mm_channel_superbuf_comp_and_enqueue(mm_channel_t *ch_obj,
                                             mm_channel_queue_t * queue,
                                             mm_camera_buf_info_t *buf);
mm_channel_queue_node_t* mm_channel_superbuf_dequeue(
        mm_channel_queue_t * queue, mm_channel_t *ch_obj);
int32_t mm_channel_superbuf_bufdone_overflow(mm_channel_t *my_obj,
                                             mm_channel_queue_t *queue);
int32_t mm_channel_superbuf_skip(mm_channel_t *my_obj,
                                 mm_channel_queue_t *queue);

static int32_t mm_channel_proc_general_cmd(mm_channel_t *my_obj,
                                           mm_camera_generic_cmd_t *p_gen_cmd);
int32_t mm_channel_superbuf_flush_matched(mm_channel_t* my_obj,
                                          mm_channel_queue_t * queue);

/* Start of Frame Sync util methods */
void mm_frame_sync_reset();
int32_t mm_frame_sync_register_channel(mm_channel_t *ch_obj);
int32_t mm_frame_sync_unregister_channel(mm_channel_t *ch_obj);
int32_t mm_frame_sync_add(uint32_t frame_id, mm_channel_t *ch_obj);
int32_t mm_frame_sync_remove(uint32_t frame_id);
uint32_t mm_frame_sync_find_matched(uint8_t oldest);
int8_t mm_frame_sync_find_frame_index(uint32_t frame_id);
void mm_frame_sync_lock_queues();
void mm_frame_sync_unlock_queues();
void mm_channel_node_qbuf(mm_channel_t *ch_obj, mm_channel_queue_node_t *node);
/* End of Frame Sync Util methods */
void mm_channel_send_super_buf(mm_channel_node_info_t *info);
mm_channel_queue_node_t* mm_channel_superbuf_dequeue_frame_internal(
        mm_channel_queue_t * queue, uint32_t frame_idx);
uint8_t mm_channel_check_aec(mm_channel_queue_node_t *node);

/*===========================================================================
 * FUNCTION   : mm_channel_util_get_stream_by_handler
 *
 * DESCRIPTION: utility function to get a stream object from its handle
 *
 * PARAMETERS :
 *   @cam_obj: ptr to a channel object
 *   @handler: stream handle
 *
 * RETURN     : ptr to a stream object.
 *              NULL if failed.
 *==========================================================================*/
mm_stream_t * mm_channel_util_get_stream_by_handler(
                                    mm_channel_t * ch_obj,
                                    uint32_t handler)
{
    int i;
    mm_stream_t *s_obj = NULL;
    for(i = 0; i < MAX_STREAM_NUM_IN_BUNDLE; i++) {
        if ((MM_STREAM_STATE_NOTUSED != ch_obj->streams[i].state) &&
            (handler == ch_obj->streams[i].my_hdl)) {
            s_obj = &ch_obj->streams[i];
            break;
        }
    }
    return s_obj;
}

/*===========================================================================
 * FUNCTION   : mm_channel_dispatch_super_buf
 *
 * DESCRIPTION: dispatch super buffer of bundle to registered user
 *
 * PARAMETERS :
 *   @cmd_cb  : ptr storing matched super buf information
 *   @userdata: user data ptr
 *
 * RETURN     : none
 *==========================================================================*/
static void mm_channel_dispatch_super_buf(mm_camera_cmdcb_t *cmd_cb,
                                          void* user_data)
{
    mm_channel_t * my_obj = (mm_channel_t *)user_data;

    if (NULL == my_obj) {
        return;
    }

    if (MM_CAMERA_CMD_TYPE_SUPER_BUF_DATA_CB != cmd_cb->cmd_type) {
        CDBG_ERROR("%s: Wrong cmd_type (%d) for super buf dataCB",
                   __func__, cmd_cb->cmd_type);
        return;
    }

    if (my_obj->bundle.super_buf_notify_cb) {
        my_obj->bundle.super_buf_notify_cb(&cmd_cb->u.superbuf, my_obj->bundle.user_data);
    }
}

/*===========================================================================
 * FUNCTION   : mm_channel_process_stream_buf
 *
 * DESCRIPTION: handle incoming buffer from stream in a bundle. In this function,
 *              matching logic will be performed on incoming stream frames.
 *              Will depends on the bundle attribute, either storing matched frames
 *              in the superbuf queue, or sending matched superbuf frames to upper
 *              layer through registered callback.
 *
 * PARAMETERS :
 *   @cmd_cb  : ptr storing matched super buf information
 *   @userdata: user data ptr
 *
 * RETURN     : none
 *==========================================================================*/
static void mm_channel_process_stream_buf(mm_camera_cmdcb_t * cmd_cb,
                                          void *user_data)
{
    mm_camera_super_buf_notify_mode_t notify_mode;
    mm_channel_queue_node_t *node = NULL;
    mm_channel_t *ch_obj = (mm_channel_t *)user_data;
    uint32_t i = 0;

    if (NULL == ch_obj) {
        return;
    }
    if (MM_CAMERA_CMD_TYPE_DATA_CB  == cmd_cb->cmd_type) {
        /* comp_and_enqueue */
        mm_channel_superbuf_comp_and_enqueue(
                        ch_obj,
                        &ch_obj->bundle.superbuf_queue,
                        &cmd_cb->u.buf);
    } else if (MM_CAMERA_CMD_TYPE_REQ_DATA_CB  == cmd_cb->cmd_type) {
        /* skip frames if needed */
        ch_obj->pending_cnt = cmd_cb->u.req_buf.num_buf_requested;
        ch_obj->pending_retro_cnt = cmd_cb->u.req_buf.num_retro_buf_requested;
        ch_obj->req_type = cmd_cb->u.req_buf.type;
        ch_obj->bWaitForPrepSnapshotDone = 0;

        CDBG_HIGH("%s: pending cnt (%d), retro count (%d)"
                "req_type (%d) is_primary (%d)",
                __func__, ch_obj->pending_cnt, ch_obj->pending_retro_cnt,
                ch_obj->req_type, cmd_cb->u.req_buf.primary_only);
        if (!ch_obj->pending_cnt || (ch_obj->pending_retro_cnt > ch_obj->pending_cnt)) {
          ch_obj->pending_retro_cnt = ch_obj->pending_cnt;
        }
        if (ch_obj->pending_retro_cnt > 0) {
          ALOGV("%s: [ZSL Retro] Resetting need Led Flash!!!",
              __func__);
          ch_obj->needLEDFlash = 0;
        }
        ch_obj->stopZslSnapshot = 0;
        ch_obj->unLockAEC = 0;

        mm_channel_superbuf_skip(ch_obj, &ch_obj->bundle.superbuf_queue);

    } else if (MM_CAMERA_CMD_TYPE_START_ZSL == cmd_cb->cmd_type) {
            ch_obj->manualZSLSnapshot = TRUE;
            mm_camera_start_zsl_snapshot(ch_obj->cam_obj);
    } else if (MM_CAMERA_CMD_TYPE_STOP_ZSL == cmd_cb->cmd_type) {
            ch_obj->manualZSLSnapshot = FALSE;
            mm_camera_stop_zsl_snapshot(ch_obj->cam_obj);
    } else if (MM_CAMERA_CMD_TYPE_CONFIG_NOTIFY == cmd_cb->cmd_type) {
           ch_obj->bundle.superbuf_queue.attr.notify_mode = cmd_cb->u.notify_mode;
    } else if (MM_CAMERA_CMD_TYPE_FLUSH_QUEUE  == cmd_cb->cmd_type) {
        ch_obj->bundle.superbuf_queue.expected_frame_id = cmd_cb->u.flush_cmd.frame_idx;
        mm_channel_superbuf_flush(ch_obj,
                &ch_obj->bundle.superbuf_queue, cmd_cb->u.flush_cmd.stream_type);
        cam_sem_post(&(ch_obj->cmd_thread.sync_sem));
        return;
    } else if (MM_CAMERA_CMD_TYPE_GENERAL == cmd_cb->cmd_type) {
        CDBG_HIGH("%s:%d] MM_CAMERA_CMD_TYPE_GENERAL", __func__, __LINE__);
        switch (cmd_cb->u.gen_cmd.type) {
            case MM_CAMERA_GENERIC_CMD_TYPE_AE_BRACKETING:
            case MM_CAMERA_GENERIC_CMD_TYPE_AF_BRACKETING: {
                uint32_t start = cmd_cb->u.gen_cmd.payload[0];
                CDBG_HIGH("%s:%d] MM_CAMERA_GENERIC_CMDTYPE_AF_BRACKETING %u",
                    __func__, __LINE__, start);
                mm_channel_superbuf_flush(ch_obj,
                        &ch_obj->bundle.superbuf_queue, CAM_STREAM_TYPE_DEFAULT);

                if (start) {
                    CDBG_HIGH("%s:%d] need AE bracketing, start zsl snapshot",
                        __func__, __LINE__);
                    ch_obj->bracketingState = MM_CHANNEL_BRACKETING_STATE_WAIT_GOOD_FRAME_IDX;
                } else {
                    ch_obj->bracketingState = MM_CHANNEL_BRACKETING_STATE_OFF;
                }
            }
                break;
            case MM_CAMERA_GENERIC_CMD_TYPE_FLASH_BRACKETING: {
                uint32_t start = cmd_cb->u.gen_cmd.payload[0];
                CDBG_HIGH("%s:%d] MM_CAMERA_GENERIC_CMDTYPE_FLASH_BRACKETING %u",
                    __func__, __LINE__, start);
                mm_channel_superbuf_flush(ch_obj,
                        &ch_obj->bundle.superbuf_queue, CAM_STREAM_TYPE_DEFAULT);

                if (start) {
                    CDBG_HIGH("%s:%d] need flash bracketing",
                        __func__, __LINE__);
                    ch_obj->isFlashBracketingEnabled = TRUE;
                } else {
                    ch_obj->isFlashBracketingEnabled = FALSE;
                }
            }
                break;
            case MM_CAMERA_GENERIC_CMD_TYPE_ZOOM_1X: {
                uint32_t start = cmd_cb->u.gen_cmd.payload[0];
                CDBG_HIGH("%s:%d] MM_CAMERA_GENERIC_CMD_TYPE_ZOOM_1X %u",
                    __func__, __LINE__, start);
                mm_channel_superbuf_flush(ch_obj,
                        &ch_obj->bundle.superbuf_queue, CAM_STREAM_TYPE_DEFAULT);

                if (start) {
                    CDBG_HIGH("%s:%d] need zoom 1x frame",
                        __func__, __LINE__);
                    ch_obj->isZoom1xFrameRequested = TRUE;
                } else {
                    ch_obj->isZoom1xFrameRequested = FALSE;
                }
            }
                break;
            case MM_CAMERA_GENERIC_CMD_TYPE_CAPTURE_SETTING: {
                uint32_t start = cmd_cb->u.gen_cmd.payload[0];
                CDBG_HIGH("%s:%d] MM_CAMERA_GENERIC_CMD_TYPE_CAPTURE_SETTING %u",
                    __func__, __LINE__, start);

                if (start) {
                    ch_obj->frameConfig = cmd_cb->u.gen_cmd.frame_config;
                    CDBG_HIGH("%s:%d] Capture setting Batch Count %d",
                            __func__, __LINE__, ch_obj->frameConfig.num_batch);
                    for (i = 0; i < ch_obj->frameConfig.num_batch; i++) {
                        CDBG("capture setting frame = %d type = %d",
                                i,ch_obj->frameConfig.configs[i].type);
                    }
                    ch_obj->isConfigCapture = TRUE;
                } else {
                    ch_obj->isConfigCapture = FALSE;
                    memset(&ch_obj->frameConfig, 0, sizeof(cam_capture_frame_config_t));
                }
                ch_obj->cur_capture_idx = 0;
                memset(ch_obj->capture_frame_id, 0, sizeof(uint8_t) * MAX_CAPTURE_BATCH_NUM);
                break;
            }
            default:
                CDBG_ERROR("%s:%d] Error: Invalid command", __func__, __LINE__);
                break;
        }
    }
    notify_mode = ch_obj->bundle.superbuf_queue.attr.notify_mode;

    if ((ch_obj->pending_cnt > 0)
            && (ch_obj->manualZSLSnapshot == FALSE)
            && (ch_obj->startZSlSnapshotCalled == FALSE)
            && (ch_obj->needLEDFlash == TRUE)
            && (ch_obj->isConfigCapture)) {
        /* Need to Flush the queue and trigger frame config */
        mm_channel_superbuf_flush(ch_obj,
                &ch_obj->bundle.superbuf_queue, CAM_STREAM_TYPE_DEFAULT);
        CDBG_HIGH("%s: TRIGGER frame config capture", __func__);
        mm_camera_start_zsl_snapshot(ch_obj->cam_obj);
        ch_obj->startZSlSnapshotCalled = TRUE;
        ch_obj->burstSnapNum = ch_obj->pending_cnt;
        ch_obj->bWaitForPrepSnapshotDone = 0;
    } else if ((ch_obj->pending_cnt > 0)
        && ( (ch_obj->needLEDFlash == TRUE) ||
        (MM_CHANNEL_BRACKETING_STATE_OFF != ch_obj->bracketingState))
        && (ch_obj->manualZSLSnapshot == FALSE)
        && ch_obj->startZSlSnapshotCalled == FALSE) {

        CDBG_HIGH("%s: need flash, start zsl snapshot", __func__);
        mm_camera_start_zsl_snapshot(ch_obj->cam_obj);
        ch_obj->startZSlSnapshotCalled = TRUE;
        ch_obj->burstSnapNum = ch_obj->pending_cnt;
        ch_obj->bWaitForPrepSnapshotDone = 0;
    } else if (((ch_obj->pending_cnt == 0) || (ch_obj->stopZslSnapshot == 1))
            && (ch_obj->manualZSLSnapshot == FALSE)
            && (ch_obj->startZSlSnapshotCalled == TRUE)) {
        CDBG_HIGH("%s: Got picture cancelled, stop zsl snapshot", __func__);
        mm_camera_stop_zsl_snapshot(ch_obj->cam_obj);
        // Unlock AEC
        ch_obj->startZSlSnapshotCalled = FALSE;
        ch_obj->needLEDFlash = FALSE;
        ch_obj->burstSnapNum = 0;
        ch_obj->stopZslSnapshot = 0;
        ch_obj->bWaitForPrepSnapshotDone = 0;
        ch_obj->unLockAEC = 1;
        ch_obj->bracketingState = MM_CHANNEL_BRACKETING_STATE_OFF;
        ch_obj->isConfigCapture = FALSE;
    }
    /* bufdone for overflowed bufs */
    mm_channel_superbuf_bufdone_overflow(ch_obj, &ch_obj->bundle.superbuf_queue);

    CDBG("%s: Super Buffer received, pending_cnt=%d",
        __func__, ch_obj->pending_cnt);
    /* dispatch frame if pending_cnt>0 or is in continuous streaming mode */

    CDBG("%s: [ZSL Retro] Out loop pending cnt (%d), retro count (%d)",
          __func__, ch_obj->pending_cnt, ch_obj->pending_retro_cnt);
    while (((ch_obj->pending_cnt > 0) ||
             (MM_CAMERA_SUPER_BUF_NOTIFY_CONTINUOUS == notify_mode)) &&
             (!ch_obj->bWaitForPrepSnapshotDone)) {

      CDBG_HIGH("%s: [ZSL Retro] In loop pending cnt (%d), req type (%d)",
            __func__, ch_obj->pending_cnt, ch_obj->req_type);
        /* dequeue */
        uint32_t match_frame = 0;
        mm_channel_node_info_t info;
        memset(&info, 0x0, sizeof(info));
        if (ch_obj->req_type == MM_CAMERA_REQ_FRAME_SYNC_BUF) {
            // Lock the Queues
            mm_frame_sync_lock_queues();
            uint32_t match_frame = mm_frame_sync_find_matched(FALSE);
            if (match_frame) {
                uint8_t j = 0;
                for (j = 0; j < MAX_NUM_CAMERA_PER_BUNDLE; j++) {
                    if (fs.ch_obj[j]) {
                        mm_channel_queue_t *ch_queue =
                                &fs.ch_obj[j]->bundle.superbuf_queue;
                        if (ch_queue == NULL) {
                            CDBG_HIGH("%s: Channel queue is NULL", __func__);
                            break;
                        }
                        node = mm_channel_superbuf_dequeue_frame_internal(
                                ch_queue, match_frame);
                        if (node != NULL) {
                            info.ch_obj[info.num_nodes] = fs.ch_obj[j];
                            info.node[info.num_nodes] = node;
                            info.num_nodes++;
                            CDBG_HIGH("%s: Added ch(%p) to node ,num nodes %d",
                                    __func__, fs.ch_obj[j], info.num_nodes);
                        }
                    }
                }
                mm_frame_sync_remove(match_frame);
                ALOGI("%s: match frame %d", __func__, match_frame);
                if (info.num_nodes != fs.num_cam) {
                    ALOGI("%s: num node %d != num cam (%d) Debug this",
                            __func__, info.num_nodes, fs.num_cam);
                    uint8_t j = 0;
                    // free super buffers from various nodes
                    for (j = 0; j < info.num_nodes; j++) {
                        if (info.node[j]) {
                            mm_channel_node_qbuf(info.ch_obj[j], info.node[j]);
                            free(info.node[j]);
                        }
                    }
                    //we should not use it as matched dual camera frames
                    info.num_nodes = 0;
                }
            }
            mm_frame_sync_unlock_queues();
        } else {
           node = mm_channel_superbuf_dequeue(&ch_obj->bundle.superbuf_queue, ch_obj);
           if (node != NULL) {
               if (ch_obj->isConfigCapture &&
                        (node->frame_idx < ch_obj->capture_frame_id[ch_obj->cur_capture_idx])) {
                   uint8_t i;
                   for (i = 0; i < node->num_of_bufs; i++) {
                       mm_channel_qbuf(ch_obj, node->super_buf[i].buf);
                   }
                   free(node);
               } else {
                   if (ch_obj->bundle.superbuf_queue.attr.instant_capture_enabled) {
                       // If instant capture enabled, wait until the AEC is settled
                       // check if AEC is settled or waited more than the aec frame bound.
                       if (!mm_channel_check_aec(node) &&
                                (ch_obj->bundle.superbuf_queue.frame_num_for_instant_capture <
                                ch_obj->bundle.superbuf_queue.attr.aec_frame_bound)) {
                           uint8_t i;
                           for (i = 0; i < node->num_of_bufs; i++) {
                               mm_channel_qbuf(ch_obj, node->super_buf[i].buf);
                           }
                           ch_obj->bundle.superbuf_queue.frame_num_for_instant_capture++;
                           free(node);
                       } else {
                           info.num_nodes = 1;
                           info.ch_obj[0] = ch_obj;
                           info.node[0] = node;
                           ch_obj->bundle.superbuf_queue.frame_num_for_instant_capture = 0;
                       }
                   } else {
                       info.num_nodes = 1;
                       info.ch_obj[0] = ch_obj;
                       info.node[0] = node;
                   }
               }
            }
        }
        if (info.num_nodes > 0) {
             uint8_t bReady = 0;

            /* decrease pending_cnt */
            if (MM_CAMERA_SUPER_BUF_NOTIFY_BURST == notify_mode) {
                ch_obj->pending_cnt--;
                if (ch_obj->pending_retro_cnt > 0) {
                  if (ch_obj->pending_retro_cnt == 1) {
                    ch_obj->bWaitForPrepSnapshotDone = 1;
                  }
                  ch_obj->pending_retro_cnt--;
                }
                CDBG_HIGH("%s: [ZSL Retro] Super Buffer received, Call client callback,"
                    "pending_cnt=%d", __func__, ch_obj->pending_cnt);

                if (((ch_obj->pending_cnt == 0) ||
                      (ch_obj->stopZslSnapshot == 1)) &&
                      (ch_obj->manualZSLSnapshot == FALSE) &&
                       ch_obj->startZSlSnapshotCalled == TRUE) {
                    CDBG_HIGH("%s: [ZSL Retro] Received all frames, stop zsl snapshot",
                            __func__);
                    mm_camera_stop_zsl_snapshot(ch_obj->cam_obj);
                    ch_obj->startZSlSnapshotCalled = FALSE;
                    ch_obj->burstSnapNum = 0;
                    ch_obj->stopZslSnapshot = 0;
                    ch_obj->unLockAEC = 1;
                    ch_obj->needLEDFlash = FALSE;
                    ch_obj->bracketingState = MM_CHANNEL_BRACKETING_STATE_OFF;
                    ch_obj->isConfigCapture = FALSE;
                }

                if (ch_obj->isConfigCapture) {
                    if (ch_obj->frameConfig.configs[ch_obj->cur_capture_idx].num_frames != 0) {
                        ch_obj->frameConfig.configs[ch_obj->cur_capture_idx].num_frames--;
                    } else {
                        CDBG_HIGH("Invalid frame config batch index %d max batch = %d",
                                ch_obj->cur_capture_idx, ch_obj->frameConfig.num_batch);
                    }

                    if (ch_obj->frameConfig.configs[ch_obj->cur_capture_idx].num_frames == 0) {
                        //Received all frames for current batch
                        ch_obj->cur_capture_idx++;
                        ch_obj->bundle.superbuf_queue.expected_frame_id =
                                ch_obj->capture_frame_id[ch_obj->cur_capture_idx];
                    } else {
                        CDBG_HIGH("Need %d frames more for batch %d",
                                ch_obj->frameConfig.configs[ch_obj->cur_capture_idx].num_frames,
                                ch_obj->cur_capture_idx);
                    }
                }
            }
            /* dispatch superbuf */
            mm_channel_send_super_buf(&info);
        } else {
            /* no superbuf avail, break the loop */
            break;
        }
    }
}

/*===========================================================================
 * FUNCTION   : mm_channel_send_super_buf
 *
 * DESCRIPTION: Send super buffers to HAL
 *
 * PARAMETERS :
 *   @info  : Info of super buffers to be sent in callback
 *
 * RETURN     : None
 *==========================================================================*/
void mm_channel_send_super_buf(mm_channel_node_info_t *info)
{
    if (!info || !info->num_nodes){
        CDBG_ERROR("%s: X Error!! Info invalid", __func__);
        return;
    }
    mm_channel_queue_node_t *node = NULL;

    CDBG_HIGH("%s: num nodes %d to send", __func__, info->num_nodes);
    uint32_t idx = 0;
    mm_channel_t *ch_obj = NULL;
    for (idx = 0; idx < info->num_nodes; idx++) {
        node = info->node[idx];
        ch_obj = info->ch_obj[idx];
        if ((ch_obj) && (NULL != ch_obj->bundle.super_buf_notify_cb) && node) {
            mm_camera_cmdcb_t* cb_node = NULL;
            CDBG("%s: Send superbuf to HAL, pending_cnt=%d",
                    __func__, ch_obj->pending_cnt);
            /* send cam_sem_post to wake up cb thread to dispatch super buffer */
            cb_node = (mm_camera_cmdcb_t *)malloc(sizeof(mm_camera_cmdcb_t));
            if (NULL != cb_node) {
                memset(cb_node, 0, sizeof(mm_camera_cmdcb_t));
                cb_node->cmd_type = MM_CAMERA_CMD_TYPE_SUPER_BUF_DATA_CB;
                cb_node->u.superbuf.num_bufs = node->num_of_bufs;
                uint8_t i = 0;
                for (i = 0; i < node->num_of_bufs; i++) {
                    cb_node->u.superbuf.bufs[i] = node->super_buf[i].buf;
                }
                cb_node->u.superbuf.camera_handle = ch_obj->cam_obj->my_hdl;
                cb_node->u.superbuf.ch_id = ch_obj->my_hdl;
                cb_node->u.superbuf.bReadyForPrepareSnapshot =
                        ch_obj->bWaitForPrepSnapshotDone;
                if (ch_obj->unLockAEC == 1) {
                    cb_node->u.superbuf.bUnlockAEC = 1;
                    CDBG_HIGH("%s: Unlocking AEC", __func__);
                    ch_obj->unLockAEC = 0;
                }
                /* enqueue to cb thread */
                cam_queue_enq(&(ch_obj->cb_thread.cmd_queue), cb_node);
                /* wake up cb thread */
                cam_sem_post(&(ch_obj->cb_thread.cmd_sem));
                CDBG_HIGH("%s: Sent super buf for node[%d] ", __func__, idx);

            } else {
                CDBG_ERROR("%s: No memory for mm_camera_node_t", __func__);
                /* buf done with the unused super buf */
                uint8_t i = 0;
                for (i = 0; i < node->num_of_bufs; i++) {
                    mm_channel_qbuf(ch_obj, node->super_buf[i].buf);
                }
            }
            free(node);
        } else if ((ch_obj != NULL) && (node != NULL)) {
            /* buf done with the unused super buf */
            uint8_t i;
            for (i = 0; i < node->num_of_bufs; i++) {
                mm_channel_qbuf(ch_obj, node->super_buf[i].buf);
            }
            free(node);
        } else {
            CDBG_ERROR("%s: node is NULL, debug this", __func__);
        }
    }
}

/*===========================================================================
 * FUNCTION   : mm_channel_reg_stream_buf_cb
 *
 * DESCRIPTION: Register callback for stream buffer
 *
 * PARAMETERS :
 *   @my_obj     : Channel object
 *   @stream_id  : stream that will be linked
 *   @buf_cb     : special callback needs to be registered for stream buffer
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 --  failure
 *==========================================================================*/
int32_t mm_channel_reg_stream_buf_cb (mm_channel_t* my_obj,
        uint32_t stream_id, mm_stream_data_cb_t buf_cb)
{
    int32_t rc = -1;
    mm_stream_t* s_obj = mm_channel_util_get_stream_by_handler(my_obj,
            stream_id);

    if (NULL != s_obj) {
        if (s_obj->ch_obj != my_obj) {
            /* No op. on linked streams */
            return 0;
        }
        rc = mm_stream_reg_buf_cb(s_obj, buf_cb);
    }

    return rc;

}

/*===========================================================================
 * FUNCTION   : mm_channel_fsm_fn
 *
 * DESCRIPTION: channel finite state machine entry function. Depends on channel
 *              state, incoming event will be handled differently.
 *
 * PARAMETERS :
 *   @my_obj   : ptr to a channel object
 *   @evt      : channel event to be processed
 *   @in_val   : input event payload. Can be NULL if not needed.
 *   @out_val  : output payload, Can be NULL if not needed.
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_channel_fsm_fn(mm_channel_t *my_obj,
                          mm_channel_evt_type_t evt,
                          void * in_val,
                          void * out_val)
{
    int32_t rc = -1;

    CDBG("%s : E state = %d", __func__, my_obj->state);
    switch (my_obj->state) {
    case MM_CHANNEL_STATE_NOTUSED:
        rc = mm_channel_fsm_fn_notused(my_obj, evt, in_val, out_val);
        break;
    case MM_CHANNEL_STATE_STOPPED:
        rc = mm_channel_fsm_fn_stopped(my_obj, evt, in_val, out_val);
        break;
    case MM_CHANNEL_STATE_ACTIVE:
        rc = mm_channel_fsm_fn_active(my_obj, evt, in_val, out_val);
        break;
    case MM_CHANNEL_STATE_PAUSED:
        rc = mm_channel_fsm_fn_paused(my_obj, evt, in_val, out_val);
        break;
    default:
        CDBG("%s: Not a valid state (%d)", __func__, my_obj->state);
        break;
    }

    /* unlock ch_lock */
    pthread_mutex_unlock(&my_obj->ch_lock);
    CDBG("%s : X rc = %d", __func__, rc);
    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_channel_fsm_fn_notused
 *
 * DESCRIPTION: channel finite state machine function to handle event
 *              in NOT_USED state.
 *
 * PARAMETERS :
 *   @my_obj   : ptr to a channel object
 *   @evt      : channel event to be processed
 *   @in_val   : input event payload. Can be NULL if not needed.
 *   @out_val  : output payload, Can be NULL if not needed.
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_channel_fsm_fn_notused(mm_channel_t *my_obj,
                                  mm_channel_evt_type_t evt,
                                  void * in_val,
                                  void * out_val)
{
    int32_t rc = -1;

    switch (evt) {
    default:
        CDBG_ERROR("%s: invalid state (%d) for evt (%d), in(%p), out(%p)",
                   __func__, my_obj->state, evt, in_val, out_val);
        break;
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_channel_fsm_fn_stopped
 *
 * DESCRIPTION: channel finite state machine function to handle event
 *              in STOPPED state.
 *
 * PARAMETERS :
 *   @my_obj   : ptr to a channel object
 *   @evt      : channel event to be processed
 *   @in_val   : input event payload. Can be NULL if not needed.
 *   @out_val  : output payload, Can be NULL if not needed.
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_channel_fsm_fn_stopped(mm_channel_t *my_obj,
                                  mm_channel_evt_type_t evt,
                                  void * in_val,
                                  void * out_val)
{
    int32_t rc = 0;
    CDBG("%s : E evt = %d", __func__, evt);
    switch (evt) {
    case MM_CHANNEL_EVT_ADD_STREAM:
        {
            uint32_t s_hdl = 0;
            s_hdl = mm_channel_add_stream(my_obj);
            *((uint32_t*)out_val) = s_hdl;
            rc = 0;
        }
        break;
    case MM_CHANNEL_EVT_LINK_STREAM:
        {
            mm_camera_stream_link_t *stream_link = NULL;
            uint32_t s_hdl = 0;
            stream_link = (mm_camera_stream_link_t *) in_val;
            s_hdl = mm_channel_link_stream(my_obj, stream_link);
            *((uint32_t*)out_val) = s_hdl;
            rc = 0;
        }
        break;
    case MM_CHANNEL_EVT_DEL_STREAM:
        {
            uint32_t s_id = *((uint32_t *)in_val);
            rc = mm_channel_del_stream(my_obj, s_id);
        }
        break;
    case MM_CHANNEL_EVT_START:
        {
            rc = mm_channel_start(my_obj);
            /* first stream started in stopped state
             * move to active state */
            if (0 == rc) {
                my_obj->state = MM_CHANNEL_STATE_ACTIVE;
            }
        }
        break;
    case MM_CHANNEL_EVT_CONFIG_STREAM:
        {
            mm_evt_paylod_config_stream_t *payload =
                (mm_evt_paylod_config_stream_t *)in_val;
            rc = mm_channel_config_stream(my_obj,
                                          payload->stream_id,
                                          payload->config);
        }
        break;
    case MM_CHANNEL_EVT_GET_BUNDLE_INFO:
        {
            cam_bundle_config_t *payload =
                (cam_bundle_config_t *)in_val;
            rc = mm_channel_get_bundle_info(my_obj, payload);
        }
        break;
    case MM_CHANNEL_EVT_DELETE:
        {
            mm_channel_release(my_obj);
            rc = 0;
        }
        break;
    case MM_CHANNEL_EVT_SET_STREAM_PARM:
        {
            mm_evt_paylod_set_get_stream_parms_t *payload =
                (mm_evt_paylod_set_get_stream_parms_t *)in_val;
            rc = mm_channel_set_stream_parm(my_obj, payload);
        }
        break;
    case MM_CHANNEL_EVT_GET_STREAM_QUEUED_BUF_COUNT:
        {
            uint32_t stream_id = *((uint32_t *)in_val);
            rc = mm_channel_get_queued_buf_count(my_obj, stream_id);
        }
        break;
    case MM_CHANNEL_EVT_GET_STREAM_PARM:
        {
            mm_evt_paylod_set_get_stream_parms_t *payload =
                (mm_evt_paylod_set_get_stream_parms_t *)in_val;
            rc = mm_channel_get_stream_parm(my_obj, payload);
        }
        break;
    case MM_CHANNEL_EVT_DO_STREAM_ACTION:
        {
            mm_evt_paylod_do_stream_action_t *payload =
                (mm_evt_paylod_do_stream_action_t *)in_val;
            rc = mm_channel_do_stream_action(my_obj, payload);
        }
        break;
    case MM_CHANNEL_EVT_MAP_STREAM_BUF:
        {
            cam_buf_map_type *payload =
                (cam_buf_map_type *)in_val;
            rc = mm_channel_map_stream_buf(my_obj, payload);
        }
        break;
    case MM_CHANNEL_EVT_MAP_STREAM_BUFS:
        {
            cam_buf_map_type_list *payload =
                (cam_buf_map_type_list *)in_val;
            rc = mm_channel_map_stream_bufs(my_obj, payload);
        }
        break;
    case MM_CHANNEL_EVT_UNMAP_STREAM_BUF:
        {
            cam_buf_unmap_type *payload =
                (cam_buf_unmap_type *)in_val;
            rc = mm_channel_unmap_stream_buf(my_obj, payload);
        }
        break;
    case MM_CHANNEL_EVT_REG_STREAM_BUF_CB:
        {
            mm_evt_paylod_reg_stream_buf_cb *payload =
                (mm_evt_paylod_reg_stream_buf_cb *)in_val;
            rc = mm_channel_reg_stream_buf_cb (my_obj,
                    payload->stream_id, payload->buf_cb);
        }
        break;
    default:
        CDBG_ERROR("%s: invalid state (%d) for evt (%d)",
                   __func__, my_obj->state, evt);
        break;
    }
    CDBG("%s : E rc = %d", __func__, rc);
    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_channel_fsm_fn_active
 *
 * DESCRIPTION: channel finite state machine function to handle event
 *              in ACTIVE state.
 *
 * PARAMETERS :
 *   @my_obj   : ptr to a channel object
 *   @evt      : channel event to be processed
 *   @in_val   : input event payload. Can be NULL if not needed.
 *   @out_val  : output payload, Can be NULL if not needed.
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_channel_fsm_fn_active(mm_channel_t *my_obj,
                          mm_channel_evt_type_t evt,
                          void * in_val,
                          void * out_val)
{
    int32_t rc = 0;

    CDBG("%s : E evt = %d", __func__, evt);
    switch (evt) {
    case MM_CHANNEL_EVT_STOP:
        {
            rc = mm_channel_stop(my_obj);
            my_obj->state = MM_CHANNEL_STATE_STOPPED;
        }
        break;
    case MM_CHANNEL_EVT_REQUEST_SUPER_BUF:
        {
            mm_camera_req_buf_t *payload =
                    (mm_camera_req_buf_t *)in_val;
            rc = mm_channel_request_super_buf(my_obj, payload);
        }
        break;
    case MM_CHANNEL_EVT_CANCEL_REQUEST_SUPER_BUF:
        {
            rc = mm_channel_cancel_super_buf_request(my_obj);
        }
        break;
    case MM_CHANNEL_EVT_FLUSH_SUPER_BUF_QUEUE:
        {
            uint32_t frame_idx = *((uint32_t *)in_val);
            rc = mm_channel_flush_super_buf_queue(my_obj, frame_idx, CAM_STREAM_TYPE_DEFAULT);
        }
        break;
    case MM_CHANNEL_EVT_START_ZSL_SNAPSHOT:
        {
            rc = mm_channel_start_zsl_snapshot(my_obj);
        }
        break;
    case MM_CHANNEL_EVT_STOP_ZSL_SNAPSHOT:
        {
            rc = mm_channel_stop_zsl_snapshot(my_obj);
        }
        break;
    case MM_CHANNEL_EVT_CONFIG_NOTIFY_MODE:
        {
            mm_camera_super_buf_notify_mode_t notify_mode =
                *((mm_camera_super_buf_notify_mode_t *)in_val);
            rc = mm_channel_config_notify_mode(my_obj, notify_mode);
        }
        break;
    case MM_CHANNEL_EVT_SET_STREAM_PARM:
        {
            mm_evt_paylod_set_get_stream_parms_t *payload =
                (mm_evt_paylod_set_get_stream_parms_t *)in_val;
            rc = mm_channel_set_stream_parm(my_obj, payload);
        }
        break;
    case MM_CHANNEL_EVT_GET_STREAM_QUEUED_BUF_COUNT:
        {
            uint32_t stream_id = *((uint32_t *)in_val);
            rc = mm_channel_get_queued_buf_count(my_obj, stream_id);
        }
        break;
    case MM_CHANNEL_EVT_GET_STREAM_PARM:
        {
            mm_evt_paylod_set_get_stream_parms_t *payload =
                (mm_evt_paylod_set_get_stream_parms_t *)in_val;
            rc = mm_channel_get_stream_parm(my_obj, payload);
        }
        break;
    case MM_CHANNEL_EVT_DO_STREAM_ACTION:
        {
            mm_evt_paylod_do_stream_action_t *payload =
                (mm_evt_paylod_do_stream_action_t *)in_val;
            rc = mm_channel_do_stream_action(my_obj, payload);
        }
        break;
    case MM_CHANNEL_EVT_MAP_STREAM_BUF:
        {
            cam_buf_map_type *payload =
                (cam_buf_map_type *)in_val;
            if (payload != NULL) {
                uint8_t type = payload->type;
                if ((type == CAM_MAPPING_BUF_TYPE_OFFLINE_INPUT_BUF) ||
                        (type == CAM_MAPPING_BUF_TYPE_OFFLINE_META_BUF)) {
                    rc = mm_channel_map_stream_buf(my_obj, payload);
                }
            } else {
                CDBG_ERROR("%s: cannot map regualr stream buf in active state", __func__);
            }
        }
        break;
    case MM_CHANNEL_EVT_MAP_STREAM_BUFS:
        {
            cam_buf_map_type_list *payload =
                (cam_buf_map_type_list *)in_val;
            if ((payload != NULL) && (payload->length > 0)) {
                uint8_t type = payload->buf_maps[0].type;
                if ((type == CAM_MAPPING_BUF_TYPE_OFFLINE_INPUT_BUF) ||
                        (type == CAM_MAPPING_BUF_TYPE_OFFLINE_META_BUF)) {
                    rc = mm_channel_map_stream_bufs(my_obj, payload);
                }
            } else {
                CDBG_ERROR("%s: cannot map regualr stream buf in active state", __func__);
            }
        }
        break;
    case MM_CHANNEL_EVT_UNMAP_STREAM_BUF:
        {
            cam_buf_unmap_type *payload =
                (cam_buf_unmap_type *)in_val;
            if (payload != NULL) {
                uint8_t type = payload->type;
                if ((type == CAM_MAPPING_BUF_TYPE_OFFLINE_INPUT_BUF) ||
                        (type == CAM_MAPPING_BUF_TYPE_OFFLINE_META_BUF)) {
                    rc = mm_channel_unmap_stream_buf(my_obj, payload);
                }
            } else {
                CDBG_ERROR("%s: cannot unmap regualr stream buf in active state", __func__);
            }
        }
        break;
    case MM_CHANNEL_EVT_AF_BRACKETING:
        {
            CDBG_HIGH("MM_CHANNEL_EVT_AF_BRACKETING");
            uint32_t start_flag = *((uint32_t *)in_val);
            mm_camera_generic_cmd_t gen_cmd;
            gen_cmd.type = MM_CAMERA_GENERIC_CMD_TYPE_AF_BRACKETING;
            gen_cmd.payload[0] = start_flag;
            rc = mm_channel_proc_general_cmd(my_obj, &gen_cmd);
        }
        break;
    case MM_CHANNEL_EVT_AE_BRACKETING:
        {
            CDBG_HIGH("MM_CHANNEL_EVT_AE_BRACKETING");
            uint32_t start_flag = *((uint32_t *)in_val);
            mm_camera_generic_cmd_t gen_cmd;
            gen_cmd.type = MM_CAMERA_GENERIC_CMD_TYPE_AE_BRACKETING;
            gen_cmd.payload[0] = start_flag;
            rc = mm_channel_proc_general_cmd(my_obj, &gen_cmd);
        }
        break;
    case MM_CHANNEL_EVT_FLASH_BRACKETING:
        {
            CDBG_HIGH("MM_CHANNEL_EVT_FLASH_BRACKETING");
            uint32_t start_flag = *((uint32_t *)in_val);
            mm_camera_generic_cmd_t gen_cmd;
            gen_cmd.type = MM_CAMERA_GENERIC_CMD_TYPE_FLASH_BRACKETING;
            gen_cmd.payload[0] = start_flag;
            rc = mm_channel_proc_general_cmd(my_obj, &gen_cmd);
        }
        break;
    case MM_CHANNEL_EVT_ZOOM_1X:
        {
            CDBG_HIGH("MM_CHANNEL_EVT_ZOOM_1X");
            uint32_t start_flag = *((uint32_t *)in_val);
            mm_camera_generic_cmd_t gen_cmd;
            gen_cmd.type = MM_CAMERA_GENERIC_CMD_TYPE_ZOOM_1X;
            gen_cmd.payload[0] = start_flag;
            rc = mm_channel_proc_general_cmd(my_obj, &gen_cmd);
        }
        break;
    case MM_CAMERA_EVT_CAPTURE_SETTING:
        {
            mm_camera_generic_cmd_t gen_cmd;
            cam_capture_frame_config_t *input;
            gen_cmd.type = MM_CAMERA_GENERIC_CMD_TYPE_CAPTURE_SETTING;
            CDBG_HIGH("MM_CAMERA_EVT_CAPTURE_SETTING");
            if (in_val == NULL) {
                gen_cmd.payload[0] = 0;
                memset(&gen_cmd.frame_config, 0, sizeof(cam_capture_frame_config_t));
            } else {
                gen_cmd.payload[0] = 1;
                input = (cam_capture_frame_config_t *)in_val;
                gen_cmd.frame_config = *input;
            }
            rc = mm_channel_proc_general_cmd(my_obj, &gen_cmd);
        }
        break;
    case MM_CHANNEL_EVT_REG_STREAM_BUF_CB:
        {
            mm_evt_paylod_reg_stream_buf_cb *payload =
                (mm_evt_paylod_reg_stream_buf_cb *)in_val;
            rc = mm_channel_reg_stream_buf_cb (my_obj,
                    payload->stream_id, payload->buf_cb);
        }
        break;
     default:
        CDBG_ERROR("%s: invalid state (%d) for evt (%d), in(%p), out(%p)",
                   __func__, my_obj->state, evt, in_val, out_val);
        break;
    }
    CDBG("%s : X rc = %d", __func__, rc);
    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_channel_fsm_fn_paused
 *
 * DESCRIPTION: channel finite state machine function to handle event
 *              in PAUSED state.
 *
 * PARAMETERS :
 *   @my_obj   : ptr to a channel object
 *   @evt      : channel event to be processed
 *   @in_val   : input event payload. Can be NULL if not needed.
 *   @out_val  : output payload, Can be NULL if not needed.
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_channel_fsm_fn_paused(mm_channel_t *my_obj,
                          mm_channel_evt_type_t evt,
                          void * in_val,
                          void * out_val)
{
    int32_t rc = 0;

    /* currently we are not supporting pause/resume channel */
    CDBG_ERROR("%s: invalid state (%d) for evt (%d), in(%p), out(%p)",
               __func__, my_obj->state, evt, in_val, out_val);

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_channel_init
 *
 * DESCRIPTION: initialize a channel
 *
 * PARAMETERS :
 *   @my_obj       : channel object be to initialized
 *   @attr         : bundle attribute of the channel if needed
 *   @channel_cb   : callback function for bundle data notify
 *   @userdata     : user data ptr
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 * NOTE       : if no bundle data notify is needed, meaning each stream in the
 *              channel will have its own stream data notify callback, then
 *              attr, channel_cb, and userdata can be NULL. In this case,
 *              no matching logic will be performed in channel for the bundling.
 *==========================================================================*/
int32_t mm_channel_init(mm_channel_t *my_obj,
                        mm_camera_channel_attr_t *attr,
                        mm_camera_buf_notify_t channel_cb,
                        void *userdata)
{
    int32_t rc = 0;

    my_obj->bundle.super_buf_notify_cb = channel_cb;
    my_obj->bundle.user_data = userdata;
    if (NULL != attr) {
        my_obj->bundle.superbuf_queue.attr = *attr;
    }

    CDBG("%s : Launch data poll thread in channel open", __func__);
    snprintf(my_obj->threadName, THREAD_NAME_SIZE, "CAM_dataPoll");
    mm_camera_poll_thread_launch(&my_obj->poll_thread[0],
                                 MM_CAMERA_POLL_TYPE_DATA);

    /* change state to stopped state */
    my_obj->state = MM_CHANNEL_STATE_STOPPED;
    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_channel_release
 *
 * DESCRIPTION: release a channel resource. Channel state will move to UNUSED
 *              state after this call.
 *
 * PARAMETERS :
 *   @my_obj       : channel object
 *
 * RETURN     : none
 *==========================================================================*/
void mm_channel_release(mm_channel_t *my_obj)
{
    /* stop data poll thread */
    mm_camera_poll_thread_release(&my_obj->poll_thread[0]);

    /* change state to notused state */
    my_obj->state = MM_CHANNEL_STATE_NOTUSED;
}

/*===========================================================================
 * FUNCTION   : mm_channel_link_stream
 *
 * DESCRIPTION: link a stream from external channel into this channel
 *
 * PARAMETERS :
 *   @my_obj  : channel object
 *   @stream_link  : channel and stream to be linked
 *
 * RETURN     : uint32_t type of stream handle
 *              0  -- invalid stream handle, meaning the op failed
 *              >0 -- successfully added a stream with a valid handle
 *==========================================================================*/
uint32_t mm_channel_link_stream(mm_channel_t *my_obj,
        mm_camera_stream_link_t *stream_link)
{
    uint8_t idx = 0;
    uint32_t s_hdl = 0;
    mm_stream_t *stream_obj = NULL;
    mm_stream_t *stream = NULL;

    if (NULL == stream_link) {
        CDBG_ERROR("%s : Invalid stream link", __func__);
        return 0;
    }

    stream = mm_channel_util_get_stream_by_handler(stream_link->ch,
            stream_link->stream_id);
    if (NULL == stream) {
        return 0;
    }

    /* check available stream */
    for (idx = 0; idx < MAX_STREAM_NUM_IN_BUNDLE; idx++) {
        if (MM_STREAM_STATE_NOTUSED == my_obj->streams[idx].state) {
            stream_obj = &my_obj->streams[idx];
            break;
        }
    }
    if (NULL == stream_obj) {
        CDBG_ERROR("%s: streams reach max, no more stream allowed to add",
                __func__);
        return s_hdl;
    }

    /* initialize stream object */
    *stream_obj = *stream;
    stream_obj->linked_stream = stream;
    s_hdl = stream->my_hdl;

    CDBG("%s : stream handle = %d", __func__, s_hdl);
    return s_hdl;
}

/*===========================================================================
 * FUNCTION   : mm_channel_add_stream
 *
 * DESCRIPTION: add a stream into the channel
 *
 * PARAMETERS :
 *   @my_obj       : channel object
 *
 * RETURN     : uint32_t type of stream handle
 *              0  -- invalid stream handle, meaning the op failed
 *              >0 -- successfully added a stream with a valid handle
 *==========================================================================*/
uint32_t mm_channel_add_stream(mm_channel_t *my_obj)
{
    int32_t rc = 0;
    uint8_t idx = 0;
    uint32_t s_hdl = 0;
    mm_stream_t *stream_obj = NULL;

    CDBG("%s : E", __func__);
    /* check available stream */
    for (idx = 0; idx < MAX_STREAM_NUM_IN_BUNDLE; idx++) {
        if (MM_STREAM_STATE_NOTUSED == my_obj->streams[idx].state) {
            stream_obj = &my_obj->streams[idx];
            break;
        }
    }
    if (NULL == stream_obj) {
        CDBG_ERROR("%s: streams reach max, no more stream allowed to add", __func__);
        return s_hdl;
    }

    /* initialize stream object */
    memset(stream_obj, 0, sizeof(mm_stream_t));
    stream_obj->fd = -1;
    stream_obj->my_hdl = mm_camera_util_generate_handler(idx);
    stream_obj->ch_obj = my_obj;
    pthread_mutex_init(&stream_obj->buf_lock, NULL);
    pthread_mutex_init(&stream_obj->cb_lock, NULL);
    pthread_mutex_init(&stream_obj->cmd_lock, NULL);
    pthread_cond_init(&stream_obj->buf_cond, NULL);
    memset(stream_obj->buf_status, 0,
            sizeof(stream_obj->buf_status));
    stream_obj->state = MM_STREAM_STATE_INITED;

    /* acquire stream */
    rc = mm_stream_fsm_fn(stream_obj, MM_STREAM_EVT_ACQUIRE, NULL, NULL);
    if (0 == rc) {
        s_hdl = stream_obj->my_hdl;
    } else {
        /* error during acquire, de-init */
        pthread_cond_destroy(&stream_obj->buf_cond);
        pthread_mutex_destroy(&stream_obj->buf_lock);
        pthread_mutex_destroy(&stream_obj->cb_lock);
        pthread_mutex_destroy(&stream_obj->cmd_lock);
        memset(stream_obj, 0, sizeof(mm_stream_t));
    }
    CDBG("%s : stream handle = %d", __func__, s_hdl);
    return s_hdl;
}

/*===========================================================================
 * FUNCTION   : mm_channel_del_stream
 *
 * DESCRIPTION: delete a stream from the channel bu its handle
 *
 * PARAMETERS :
 *   @my_obj       : channel object
 *   @stream_id    : stream handle
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 * NOTE       : assume steam is stooped before it can be deleted
 *==========================================================================*/
int32_t mm_channel_del_stream(mm_channel_t *my_obj,
                              uint32_t stream_id)
{
    int rc = -1;
    mm_stream_t * stream_obj = NULL;
    stream_obj = mm_channel_util_get_stream_by_handler(my_obj, stream_id);

    if (NULL == stream_obj) {
        CDBG_ERROR("%s :Invalid Stream Object for stream_id = %d",
                   __func__, stream_id);
        return rc;
    }

    if (stream_obj->ch_obj != my_obj) {
        /* Only unlink stream */
        pthread_mutex_lock(&stream_obj->linked_stream->buf_lock);
        stream_obj->linked_stream->is_linked = 0;
        stream_obj->linked_stream->linked_obj = NULL;
        pthread_mutex_unlock(&stream_obj->linked_stream->buf_lock);
        memset(stream_obj, 0, sizeof(mm_stream_t));

        return 0;
    }

    rc = mm_stream_fsm_fn(stream_obj,
                          MM_STREAM_EVT_RELEASE,
                          NULL,
                          NULL);

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_channel_config_stream
 *
 * DESCRIPTION: configure a stream
 *
 * PARAMETERS :
 *   @my_obj       : channel object
 *   @stream_id    : stream handle
 *   @config       : stream configuration
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_channel_config_stream(mm_channel_t *my_obj,
                                   uint32_t stream_id,
                                   mm_camera_stream_config_t *config)
{
    int rc = -1;
    mm_stream_t * stream_obj = NULL;
    CDBG("%s : E stream ID = %d", __func__, stream_id);
    stream_obj = mm_channel_util_get_stream_by_handler(my_obj, stream_id);

    if (NULL == stream_obj) {
        CDBG_ERROR("%s :Invalid Stream Object for stream_id = %d", __func__, stream_id);
        return rc;
    }

    if (stream_obj->ch_obj != my_obj) {
        /* No op. on linked streams */
        return 0;
    }

    /* set stream fmt */
    rc = mm_stream_fsm_fn(stream_obj,
                          MM_STREAM_EVT_SET_FMT,
                          (void *)config,
                          NULL);
    CDBG("%s : X rc = %d",__func__,rc);
    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_channel_get_bundle_info
 *
 * DESCRIPTION: query bundle info of the channel, which should include all
 *              streams within this channel
 *
 * PARAMETERS :
 *   @my_obj       : channel object
 *   @bundle_info  : bundle info to be filled in
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_channel_get_bundle_info(mm_channel_t *my_obj,
                                   cam_bundle_config_t *bundle_info)
{
    int i;
    mm_stream_t *s_obj = NULL;
    cam_stream_type_t stream_type = CAM_STREAM_TYPE_DEFAULT;
    int32_t rc = 0;

    memset(bundle_info, 0, sizeof(cam_bundle_config_t));
    bundle_info->bundle_id = my_obj->my_hdl;
    bundle_info->num_of_streams = 0;
    for (i = 0; i < MAX_STREAM_NUM_IN_BUNDLE; i++) {
        if (my_obj->streams[i].my_hdl > 0) {
            s_obj = mm_channel_util_get_stream_by_handler(my_obj,
                                                          my_obj->streams[i].my_hdl);
            if (NULL != s_obj) {
                stream_type = s_obj->stream_info->stream_type;
                if ((CAM_STREAM_TYPE_METADATA != stream_type) &&
                        (s_obj->ch_obj == my_obj)) {
                    bundle_info->stream_ids[bundle_info->num_of_streams++] =
                                                        s_obj->server_stream_id;
                }
            } else {
                CDBG_ERROR("%s: cannot find stream obj (%d) by handler (%d)",
                           __func__, i, my_obj->streams[i].my_hdl);
                rc = -1;
                break;
            }
        }
    }
    if (rc != 0) {
        /* error, reset to 0 */
        memset(bundle_info, 0, sizeof(cam_bundle_config_t));
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_channel_start
 *
 * DESCRIPTION: start a channel, which will start all streams in the channel
 *
 * PARAMETERS :
 *   @my_obj       : channel object
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_channel_start(mm_channel_t *my_obj)
{
    int32_t rc = 0;
    int i = 0, j = 0;
    mm_stream_t *s_objs[MAX_STREAM_NUM_IN_BUNDLE] = {NULL};
    uint8_t num_streams_to_start = 0;
    uint8_t num_streams_in_bundle_queue = 0;
    mm_stream_t *s_obj = NULL;
    int meta_stream_idx = 0;
    cam_stream_type_t stream_type = CAM_STREAM_TYPE_DEFAULT;

    for (i = 0; i < MAX_STREAM_NUM_IN_BUNDLE; i++) {
        if (my_obj->streams[i].my_hdl > 0) {
            s_obj = mm_channel_util_get_stream_by_handler(my_obj,
                                                          my_obj->streams[i].my_hdl);
            if (NULL != s_obj) {
                stream_type = s_obj->stream_info->stream_type;
                /* remember meta data stream index */
                if ((stream_type == CAM_STREAM_TYPE_METADATA) &&
                        (s_obj->ch_obj == my_obj)) {
                    meta_stream_idx = num_streams_to_start;
                }
                s_objs[num_streams_to_start++] = s_obj;

                if (!s_obj->stream_info->noFrameExpected) {
                    num_streams_in_bundle_queue++;
                }
            }
        }
    }

    if (meta_stream_idx > 0 ) {
        /* always start meta data stream first, so switch the stream object with the first one */
        s_obj = s_objs[0];
        s_objs[0] = s_objs[meta_stream_idx];
        s_objs[meta_stream_idx] = s_obj;
    }

    if (NULL != my_obj->bundle.super_buf_notify_cb) {
        /* need to send up cb, therefore launch thread */
        /* init superbuf queue */
        mm_channel_superbuf_queue_init(&my_obj->bundle.superbuf_queue);
        my_obj->bundle.superbuf_queue.num_streams = num_streams_in_bundle_queue;
        my_obj->bundle.superbuf_queue.expected_frame_id = 0;
        my_obj->bundle.superbuf_queue.expected_frame_id_without_led = 0;
        my_obj->bundle.superbuf_queue.led_off_start_frame_id = 0;
        my_obj->bundle.superbuf_queue.led_on_start_frame_id = 0;
        my_obj->bundle.superbuf_queue.led_on_num_frames = 0;
        my_obj->bundle.superbuf_queue.frame_num_for_instant_capture = 0;

        for (i = 0; i < num_streams_to_start; i++) {
            /* Only bundle streams that belong to the channel */
            if(!(s_objs[i]->stream_info->noFrameExpected)) {
                if (s_objs[i]->ch_obj == my_obj) {
                    /* set bundled flag to streams */
                    s_objs[i]->is_bundled = 1;
                }
                my_obj->bundle.superbuf_queue.bundled_streams[j++] = s_objs[i]->my_hdl;
            }
        }

        /* launch cb thread for dispatching super buf through cb */
        snprintf(my_obj->cb_thread.threadName, THREAD_NAME_SIZE, "CAM_SuperBuf");
        mm_camera_cmd_thread_launch(&my_obj->cb_thread,
                                    mm_channel_dispatch_super_buf,
                                    (void*)my_obj);

        /* launch cmd thread for super buf dataCB */
        snprintf(my_obj->cmd_thread.threadName, THREAD_NAME_SIZE, "CAM_SuperBufCB");
        mm_camera_cmd_thread_launch(&my_obj->cmd_thread,
                                    mm_channel_process_stream_buf,
                                    (void*)my_obj);

        /* set flag to TRUE */
        my_obj->bundle.is_active = TRUE;
    }

    /* link any streams first before starting the rest of the streams */
    for (i = 0; i < num_streams_to_start; i++) {
        if (s_objs[i]->ch_obj != my_obj) {
            pthread_mutex_lock(&s_objs[i]->linked_stream->buf_lock);
            s_objs[i]->linked_stream->linked_obj = my_obj;
            s_objs[i]->linked_stream->is_linked = 1;
            pthread_mutex_unlock(&s_objs[i]->linked_stream->buf_lock);
            continue;
        }
    }

    for (i = 0; i < num_streams_to_start; i++) {
        if (s_objs[i]->ch_obj != my_obj) {
            continue;
        }
        /* all streams within a channel should be started at the same time */
        if (s_objs[i]->state == MM_STREAM_STATE_ACTIVE) {
            CDBG_ERROR("%s: stream already started idx(%d)", __func__, i);
            rc = -1;
            break;
        }

        /* allocate buf */
        rc = mm_stream_fsm_fn(s_objs[i],
                              MM_STREAM_EVT_GET_BUF,
                              NULL,
                              NULL);
        if (0 != rc) {
            CDBG_ERROR("%s: get buf failed at idx(%d)", __func__, i);
            break;
        }

        /* reg buf */
        rc = mm_stream_fsm_fn(s_objs[i],
                              MM_STREAM_EVT_REG_BUF,
                              NULL,
                              NULL);
        if (0 != rc) {
            CDBG_ERROR("%s: reg buf failed at idx(%d)", __func__, i);
            break;
        }

        /* start stream */
        rc = mm_stream_fsm_fn(s_objs[i],
                              MM_STREAM_EVT_START,
                              NULL,
                              NULL);
        if (0 != rc) {
            CDBG_ERROR("%s: start stream failed at idx(%d)", __func__, i);
            break;
        }
    }

    /* error handling */
    if (0 != rc) {
        /* unlink the streams first */
        for (j = 0; j < num_streams_to_start; j++) {
            if (s_objs[j]->ch_obj != my_obj) {
                pthread_mutex_lock(&s_objs[j]->linked_stream->buf_lock);
                s_objs[j]->linked_stream->is_linked = 0;
                s_objs[j]->linked_stream->linked_obj = NULL;
                pthread_mutex_unlock(&s_objs[j]->linked_stream->buf_lock);

                if (TRUE == my_obj->bundle.is_active) {
                    mm_channel_flush_super_buf_queue(my_obj, 0,
                            s_objs[i]->stream_info->stream_type);
                }
                memset(s_objs[j], 0, sizeof(mm_stream_t));
                continue;
            }
        }

        for (j = 0; j <= i; j++) {
            if ((NULL == s_objs[j]) || (s_objs[j]->ch_obj != my_obj)) {
                continue;
            }
            /* stop streams*/
            mm_stream_fsm_fn(s_objs[j],
                             MM_STREAM_EVT_STOP,
                             NULL,
                             NULL);

            /* unreg buf */
            mm_stream_fsm_fn(s_objs[j],
                             MM_STREAM_EVT_UNREG_BUF,
                             NULL,
                             NULL);

            /* put buf back */
            mm_stream_fsm_fn(s_objs[j],
                             MM_STREAM_EVT_PUT_BUF,
                             NULL,
                             NULL);
        }

        /* destroy super buf cmd thread */
        if (TRUE == my_obj->bundle.is_active) {
            /* first stop bundle thread */
            mm_camera_cmd_thread_release(&my_obj->cmd_thread);
            mm_camera_cmd_thread_release(&my_obj->cb_thread);

            /* deinit superbuf queue */
            mm_channel_superbuf_queue_deinit(&my_obj->bundle.superbuf_queue);

            /* memset bundle info */
            memset(&my_obj->bundle, 0, sizeof(mm_channel_bundle_t));
        }
    }
    my_obj->bWaitForPrepSnapshotDone = 0;
    if (my_obj->bundle.superbuf_queue.attr.enable_frame_sync) {
        CDBG_HIGH("%s: registering Channel obj %p", __func__, my_obj);
        mm_frame_sync_register_channel(my_obj);
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_channel_stop
 *
 * DESCRIPTION: stop a channel, which will stop all streams in the channel
 *
 * PARAMETERS :
 *   @my_obj       : channel object
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_channel_stop(mm_channel_t *my_obj)
{
    int32_t rc = 0;
    int i;
    mm_stream_t *s_objs[MAX_STREAM_NUM_IN_BUNDLE] = {NULL};
    uint8_t num_streams_to_stop = 0;
    mm_stream_t *s_obj = NULL;
    int meta_stream_idx = 0;
    cam_stream_type_t stream_type = CAM_STREAM_TYPE_DEFAULT;

    if (my_obj->bundle.superbuf_queue.attr.enable_frame_sync) {
        mm_frame_sync_unregister_channel(my_obj);
    }

    for (i = 0; i < MAX_STREAM_NUM_IN_BUNDLE; i++) {
        if (my_obj->streams[i].my_hdl > 0) {
            s_obj = mm_channel_util_get_stream_by_handler(my_obj,
                                                          my_obj->streams[i].my_hdl);
            if (NULL != s_obj) {
                if (s_obj->ch_obj == my_obj) {
                    stream_type = s_obj->stream_info->stream_type;
                    /* remember meta data stream index */
                    if (stream_type == CAM_STREAM_TYPE_METADATA) {
                        meta_stream_idx = num_streams_to_stop;
                    }
                }
                s_objs[num_streams_to_stop++] = s_obj;
            }
        }
    }

    if (meta_stream_idx < num_streams_to_stop - 1 ) {
        /* always stop meta data stream last, so switch the stream object with the last one */
        s_obj = s_objs[num_streams_to_stop - 1];
        s_objs[num_streams_to_stop - 1] = s_objs[meta_stream_idx];
        s_objs[meta_stream_idx] = s_obj;
    }

    for (i = 0; i < num_streams_to_stop; i++) {
        /* stream that are linked to this channel should not be stopped */
        if (s_objs[i]->ch_obj != my_obj) {
            continue;
        }

        /* stream off */
        mm_stream_fsm_fn(s_objs[i],
                         MM_STREAM_EVT_STOP,
                         NULL,
                         NULL);

        /* unreg buf at kernel */
        mm_stream_fsm_fn(s_objs[i],
                         MM_STREAM_EVT_UNREG_BUF,
                         NULL,
                         NULL);
    }

    for (i = 0; i < num_streams_to_stop; i++) {
        if (s_objs[i]->ch_obj != my_obj) {
            /* Only unlink stream */
            pthread_mutex_lock(&s_objs[i]->linked_stream->buf_lock);
            s_objs[i]->linked_stream->is_linked = 0;
            s_objs[i]->linked_stream->linked_obj = NULL;
            pthread_mutex_unlock(&s_objs[i]->linked_stream->buf_lock);

            if (TRUE == my_obj->bundle.is_active) {
                mm_channel_flush_super_buf_queue(my_obj, 0, s_objs[i]->stream_info->stream_type);
            }
            break;
        } else {
            continue;
        }
    }

    /* destroy super buf cmd thread */
    if (TRUE == my_obj->bundle.is_active) {
        /* first stop bundle thread */
        mm_camera_cmd_thread_release(&my_obj->cmd_thread);
        mm_camera_cmd_thread_release(&my_obj->cb_thread);

        /* deinit superbuf queue */
        mm_channel_superbuf_queue_deinit(&my_obj->bundle.superbuf_queue);
    }

    /* since all streams are stopped, we are safe to
     * release all buffers allocated in stream */
    for (i = 0; i < num_streams_to_stop; i++) {
        if (s_objs[i]->ch_obj != my_obj) {
            continue;
        }
        /* put buf back */
        mm_stream_fsm_fn(s_objs[i],
                         MM_STREAM_EVT_PUT_BUF,
                         NULL,
                         NULL);
    }

    for (i = 0; i < num_streams_to_stop; i++) {
        if (s_objs[i]->ch_obj != my_obj) {
            memset(s_objs[i], 0, sizeof(mm_stream_t));
        } else {
            continue;
        }
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_channel_request_super_buf
 *
 * DESCRIPTION: for burst mode in bundle, reuqest certain amount of matched
 *              frames from superbuf queue
 *
 * PARAMETERS :
 *   @my_obj       : channel object
 *   @num_buf_requested : number of matched frames needed
 *   @num_retro_buf_requested : number of retro frames needed
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_channel_request_super_buf(mm_channel_t *my_obj,
        mm_camera_req_buf_t *buf)
{
    int32_t rc = 0;
    mm_camera_cmdcb_t* node = NULL;

    if(!buf) {
        CDBG_ERROR("%s: Request info buf is NULL", __func__);
        return -1;
    }

    /* set pending_cnt
     * will trigger dispatching super frames if pending_cnt > 0 */
    /* send cam_sem_post to wake up cmd thread to dispatch super buffer */
    node = (mm_camera_cmdcb_t *)malloc(sizeof(mm_camera_cmdcb_t));
    if (NULL != node) {
        memset(node, 0, sizeof(mm_camera_cmdcb_t));
        node->cmd_type = MM_CAMERA_CMD_TYPE_REQ_DATA_CB;
        node->u.req_buf = *buf;

        /* enqueue to cmd thread */
        cam_queue_enq(&(my_obj->cmd_thread.cmd_queue), node);

        /* wake up cmd thread */
        cam_sem_post(&(my_obj->cmd_thread.cmd_sem));
    } else {
        CDBG_ERROR("%s: No memory for mm_camera_node_t", __func__);
        rc = -1;
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_channel_cancel_super_buf_request
 *
 * DESCRIPTION: for burst mode in bundle, cancel the reuqest for certain amount
 *              of matched frames from superbuf queue
 *
 * PARAMETERS :
 *   @my_obj       : channel object
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_channel_cancel_super_buf_request(mm_channel_t *my_obj)
{
    int32_t rc = 0;
    /* reset pending_cnt */
    mm_camera_req_buf_t buf;
    memset(&buf, 0x0, sizeof(buf));
    buf.type = MM_CAMERA_REQ_SUPER_BUF;
    buf.num_buf_requested = 0;
    rc = mm_channel_request_super_buf(my_obj, &buf);
    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_channel_flush_super_buf_queue
 *
 * DESCRIPTION: flush superbuf queue
 *
 * PARAMETERS :
 *   @my_obj  : channel object
 *   @frame_idx : frame idx until which to flush all superbufs
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_channel_flush_super_buf_queue(mm_channel_t *my_obj, uint32_t frame_idx,
                                                     cam_stream_type_t stream_type)
{
    int32_t rc = 0;
    mm_camera_cmdcb_t* node = NULL;

    node = (mm_camera_cmdcb_t *)malloc(sizeof(mm_camera_cmdcb_t));
    if (NULL != node) {
        memset(node, 0, sizeof(mm_camera_cmdcb_t));
        node->cmd_type = MM_CAMERA_CMD_TYPE_FLUSH_QUEUE;
        node->u.flush_cmd.frame_idx = frame_idx;
        node->u.flush_cmd.stream_type = stream_type;

        /* enqueue to cmd thread */
        cam_queue_enq(&(my_obj->cmd_thread.cmd_queue), node);

        /* wake up cmd thread */
        cam_sem_post(&(my_obj->cmd_thread.cmd_sem));

        /* wait for ack from cmd thread */
        cam_sem_wait(&(my_obj->cmd_thread.sync_sem));
    } else {
        CDBG_ERROR("%s: No memory for mm_camera_node_t", __func__);
        rc = -1;
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_channel_config_notify_mode
 *
 * DESCRIPTION: configure notification mode
 *
 * PARAMETERS :
 *   @my_obj  : channel object
 *   @notify_mode : notification mode
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_channel_config_notify_mode(mm_channel_t *my_obj,
                                      mm_camera_super_buf_notify_mode_t notify_mode)
{
    int32_t rc = 0;
    mm_camera_cmdcb_t* node = NULL;

    node = (mm_camera_cmdcb_t *)malloc(sizeof(mm_camera_cmdcb_t));
    if (NULL != node) {
        memset(node, 0, sizeof(mm_camera_cmdcb_t));
        node->u.notify_mode = notify_mode;
        node->cmd_type = MM_CAMERA_CMD_TYPE_CONFIG_NOTIFY;

        /* enqueue to cmd thread */
        cam_queue_enq(&(my_obj->cmd_thread.cmd_queue), node);

        /* wake up cmd thread */
        cam_sem_post(&(my_obj->cmd_thread.cmd_sem));
    } else {
        CDBG_ERROR("%s: No memory for mm_camera_node_t", __func__);
        rc = -1;
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_channel_start_zsl_snapshot
 *
 * DESCRIPTION: start zsl snapshot
 *
 * PARAMETERS :
 *   @my_obj  : channel object
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_channel_start_zsl_snapshot(mm_channel_t *my_obj)
{
    int32_t rc = 0;
    mm_camera_cmdcb_t* node = NULL;

    node = (mm_camera_cmdcb_t *)malloc(sizeof(mm_camera_cmdcb_t));
    if (NULL != node) {
        memset(node, 0, sizeof(mm_camera_cmdcb_t));
        node->cmd_type = MM_CAMERA_CMD_TYPE_START_ZSL;

        /* enqueue to cmd thread */
        cam_queue_enq(&(my_obj->cmd_thread.cmd_queue), node);

        /* wake up cmd thread */
        cam_sem_post(&(my_obj->cmd_thread.cmd_sem));
    } else {
        CDBG_ERROR("%s: No memory for mm_camera_node_t", __func__);
        rc = -1;
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_channel_stop_zsl_snapshot
 *
 * DESCRIPTION: stop zsl snapshot
 *
 * PARAMETERS :
 *   @my_obj  : channel object
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_channel_stop_zsl_snapshot(mm_channel_t *my_obj)
{
    int32_t rc = 0;
    mm_camera_cmdcb_t* node = NULL;

    node = (mm_camera_cmdcb_t *)malloc(sizeof(mm_camera_cmdcb_t));
    if (NULL != node) {
        memset(node, 0, sizeof(mm_camera_cmdcb_t));
        node->cmd_type = MM_CAMERA_CMD_TYPE_STOP_ZSL;

        /* enqueue to cmd thread */
        cam_queue_enq(&(my_obj->cmd_thread.cmd_queue), node);

        /* wake up cmd thread */
        cam_sem_post(&(my_obj->cmd_thread.cmd_sem));
    } else {
        CDBG_ERROR("%s: No memory for mm_camera_node_t", __func__);
        rc = -1;
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_channel_qbuf
 *
 * DESCRIPTION: enqueue buffer back to kernel
 *
 * PARAMETERS :
 *   @my_obj       : channel object
 *   @buf          : buf ptr to be enqueued
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_channel_qbuf(mm_channel_t *my_obj,
                        mm_camera_buf_def_t *buf)
{
    int32_t rc = -1;
    mm_stream_t* s_obj = mm_channel_util_get_stream_by_handler(my_obj, buf->stream_id);

    if (NULL != s_obj) {
        if (s_obj->ch_obj != my_obj) {
            /* Redirect to linked stream */
            rc = mm_stream_fsm_fn(s_obj->linked_stream,
                    MM_STREAM_EVT_QBUF,
                    (void *)buf,
                    NULL);
        } else {
            rc = mm_stream_fsm_fn(s_obj,
                    MM_STREAM_EVT_QBUF,
                    (void *)buf,
                    NULL);
        }
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_channel_get_queued_buf_count
 *
 * DESCRIPTION: return queued buffer count
 *
 * PARAMETERS :
 *   @my_obj       : channel object
 *   @stream_id    : steam_id
 *
 * RETURN     : queued buffer count
 *==========================================================================*/
int32_t mm_channel_get_queued_buf_count(mm_channel_t *my_obj, uint32_t stream_id)
{
    int32_t rc = -1;
    mm_stream_t* s_obj = mm_channel_util_get_stream_by_handler(my_obj, stream_id);

    if (NULL != s_obj) {
        if (s_obj->ch_obj != my_obj) {
            /* Redirect to linked stream */
            rc = mm_stream_fsm_fn(s_obj->linked_stream,
                    MM_STREAM_EVT_GET_QUEUED_BUF_COUNT,
                    NULL,
                    NULL);
        } else {
            rc = mm_stream_fsm_fn(s_obj,
                    MM_STREAM_EVT_GET_QUEUED_BUF_COUNT,
                    NULL,
                    NULL);
        }
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_channel_set_stream_parms
 *
 * DESCRIPTION: set parameters per stream
 *
 * PARAMETERS :
 *   @my_obj       : channel object
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
int32_t mm_channel_set_stream_parm(mm_channel_t *my_obj,
                                   mm_evt_paylod_set_get_stream_parms_t *payload)
{
    int32_t rc = -1;
    mm_stream_t* s_obj = mm_channel_util_get_stream_by_handler(my_obj,
                                                               payload->stream_id);
    if (NULL != s_obj) {
        if (s_obj->ch_obj != my_obj) {
            /* No op. on linked streams */
            return 0;
        }

        rc = mm_stream_fsm_fn(s_obj,
                              MM_STREAM_EVT_SET_PARM,
                              (void *)payload,
                              NULL);
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_channel_get_stream_parms
 *
 * DESCRIPTION: get parameters per stream
 *
 * PARAMETERS :
 *   @my_obj       : channel object
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
int32_t mm_channel_get_stream_parm(mm_channel_t *my_obj,
                                   mm_evt_paylod_set_get_stream_parms_t *payload)
{
    int32_t rc = -1;
    mm_stream_t* s_obj = mm_channel_util_get_stream_by_handler(my_obj,
                                                               payload->stream_id);
    if (NULL != s_obj) {
        if (s_obj->ch_obj != my_obj) {
            /* No op. on linked streams */
            return 0;
        }

        rc = mm_stream_fsm_fn(s_obj,
                              MM_STREAM_EVT_GET_PARM,
                              (void *)payload,
                              NULL);
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_channel_do_stream_action
 *
 * DESCRIPTION: request server to perform stream based action. Maybe removed later
 *              if the functionality is included in mm_camera_set_parms
 *
 * PARAMETERS :
 *   @my_obj       : channel object
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
int32_t mm_channel_do_stream_action(mm_channel_t *my_obj,
                                   mm_evt_paylod_do_stream_action_t *payload)
{
    int32_t rc = -1;
    mm_stream_t* s_obj = mm_channel_util_get_stream_by_handler(my_obj,
                                                               payload->stream_id);
    if (NULL != s_obj) {
        if (s_obj->ch_obj != my_obj) {
            /* No op. on linked streams */
            return 0;
        }

        rc = mm_stream_fsm_fn(s_obj,
                              MM_STREAM_EVT_DO_ACTION,
                              (void *)payload,
                              NULL);
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_channel_map_stream_buf
 *
 * DESCRIPTION: mapping stream buffer via domain socket to server
 *
 * PARAMETERS :
 *   @my_obj       : channel object
 *   @payload      : ptr to payload for mapping
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_channel_map_stream_buf(mm_channel_t *my_obj,
                                  cam_buf_map_type *payload)
{
    int32_t rc = -1;
    mm_stream_t* s_obj = mm_channel_util_get_stream_by_handler(my_obj,
                                                               payload->stream_id);
    if (NULL != s_obj) {
        if (s_obj->ch_obj != my_obj) {
            /* No op. on linked streams */
            return 0;
        }

        rc = mm_stream_map_buf(s_obj,
                               payload->type,
                               payload->frame_idx,
                               payload->plane_idx,
                               payload->fd,
                               payload->size);
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_channel_map_stream_bufs
 *
 * DESCRIPTION: mapping stream buffers via domain socket to server
 *
 * PARAMETERS :
 *   @my_obj       : channel object
 *   @payload      : ptr to payload for mapping
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_channel_map_stream_bufs(mm_channel_t *my_obj,
                                   cam_buf_map_type_list *payload)
{
    int32_t rc = -1;
    if ((payload == NULL) || (payload->length == 0)) {
        return rc;
    }

    mm_stream_t* s_obj = mm_channel_util_get_stream_by_handler(my_obj,
                                                               payload->buf_maps[0].stream_id);
    if (NULL != s_obj) {
        if (s_obj->ch_obj != my_obj) {
            /* No op. on linked streams */
            return 0;
        }

        rc = mm_stream_map_bufs(s_obj, payload);
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_channel_unmap_stream_buf
 *
 * DESCRIPTION: unmapping stream buffer via domain socket to server
 *
 * PARAMETERS :
 *   @my_obj       : channel object
 *   @payload      : ptr to unmap payload
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_channel_unmap_stream_buf(mm_channel_t *my_obj,
                                    cam_buf_unmap_type *payload)
{
    int32_t rc = -1;
    mm_stream_t* s_obj = mm_channel_util_get_stream_by_handler(my_obj,
                                                               payload->stream_id);
    if (NULL != s_obj) {
        if (s_obj->ch_obj != my_obj) {
            /* No op. on linked streams */
            return 0;
        }

        rc = mm_stream_unmap_buf(s_obj, payload->type,
                                 payload->frame_idx, payload->plane_idx);
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_channel_superbuf_queue_init
 *
 * DESCRIPTION: initialize superbuf queue in the channel
 *
 * PARAMETERS :
 *   @queue   : ptr to superbuf queue to be initialized
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_channel_superbuf_queue_init(mm_channel_queue_t * queue)
{
    return cam_queue_init(&queue->que);
}

/*===========================================================================
 * FUNCTION   : mm_channel_superbuf_queue_deinit
 *
 * DESCRIPTION: deinitialize superbuf queue in the channel
 *
 * PARAMETERS :
 *   @queue   : ptr to superbuf queue to be deinitialized
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_channel_superbuf_queue_deinit(mm_channel_queue_t * queue)
{
    return cam_queue_deinit(&queue->que);
}

/*===========================================================================
 * FUNCTION   : mm_channel_util_seq_comp_w_rollover
 *
 * DESCRIPTION: utility function to handle sequence number comparison with rollover
 *
 * PARAMETERS :
 *   @v1      : first value to be compared
 *   @v2      : second value to be compared
 *
 * RETURN     : int8_t type of comparison result
 *              >0  -- v1 larger than v2
 *              =0  -- vi equal to v2
 *              <0  -- v1 smaller than v2
 *==========================================================================*/
int8_t mm_channel_util_seq_comp_w_rollover(uint32_t v1,
                                           uint32_t v2)
{
    int8_t ret = 0;

    /* TODO: need to handle the case if v2 roll over to 0 */
    if (v1 > v2) {
        ret = 1;
    } else if (v1 < v2) {
        ret = -1;
    }

    return ret;
}

uint8_t mm_channel_check_aec(mm_channel_queue_node_t *node)
{
    uint8_t i = 0;
    const metadata_buffer_t *metadata = NULL;
    uint8_t is_settled = 0;
    for (i = 0; i < node->num_of_bufs; i++) {
        if (node->super_buf[i].buf->stream_type == CAM_STREAM_TYPE_METADATA) {
            metadata = (const metadata_buffer_t *)node->super_buf[i].buf->buffer;
            break;
        }
    }

    if (i == node->num_of_bufs) {
        CDBG_ERROR("%s: no metadata stream , ignore is_settled",
                   __func__);
        is_settled = 1;
    } else if (NULL == metadata) {
        CDBG_ERROR("%s: NULL metadata buffer for metadata stream",
                   __func__);
    } else {
        IF_META_AVAILABLE(const cam_3a_params_t, ae_params, CAM_INTF_META_AEC_INFO, metadata) {
            is_settled = ae_params->settled;
        }
    }
    CDBG("%s: is_settled %d", __func__ ,is_settled);
    return is_settled;
}

/*===========================================================================
 * FUNCTION   : mm_channel_handle_metadata
 *
 * DESCRIPTION: Handle frame matching logic change due to metadata
 *
 * PARAMETERS :
 *   @ch_obj  : channel object
 *   @queue   : superbuf queue
 *   @buf_info: new buffer from stream
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_channel_handle_metadata(
                        mm_channel_t* ch_obj,
                        mm_channel_queue_t * queue,
                        mm_camera_buf_info_t *buf_info)
{

    int rc = 0 ;
    mm_stream_t* stream_obj = NULL;
    stream_obj = mm_channel_util_get_stream_by_handler(ch_obj,
                buf_info->stream_id);
    uint8_t is_prep_snapshot_done_valid = 0;
    uint8_t is_good_frame_idx_range_valid = 0;
    int32_t prep_snapshot_done_state = 0;
    cam_frame_idx_range_t good_frame_idx_range;
    uint8_t is_crop_1x_found = 0;
    uint32_t snapshot_stream_id = 0;
    uint32_t i;
    /* Set expected frame id to a future frame idx, large enough to wait
    * for good_frame_idx_range, and small enough to still capture an image */
    const uint32_t max_future_frame_offset = 100U;

    memset(&good_frame_idx_range, 0, sizeof(good_frame_idx_range));

    if (NULL == stream_obj) {
        CDBG_ERROR("%s: Invalid Stream Object for stream_id = %d",
                   __func__, buf_info->stream_id);
        rc = -1;
        goto end;
    }
    if (NULL == stream_obj->stream_info) {
        CDBG_ERROR("%s: NULL stream info for stream_id = %d",
                    __func__, buf_info->stream_id);
        rc = -1;
        goto end;
    }

    if ((CAM_STREAM_TYPE_METADATA == stream_obj->stream_info->stream_type) &&
            (stream_obj->ch_obj == ch_obj)) {
        const metadata_buffer_t *metadata;
        metadata = (const metadata_buffer_t *)buf_info->buf->buffer;

        if (NULL == metadata) {
            CDBG_ERROR("%s: NULL metadata buffer for metadata stream",
                       __func__);
            rc = -1;
            goto end;
        }
        CDBG("%s: E , expected frame id: %d", __func__, queue->expected_frame_id);

        IF_META_AVAILABLE(const int32_t, p_prep_snapshot_done_state,
                CAM_INTF_META_PREP_SNAPSHOT_DONE, metadata) {
            prep_snapshot_done_state = *p_prep_snapshot_done_state;
            is_prep_snapshot_done_valid = 1;
            CDBG_HIGH("%s: prepare snapshot done valid ", __func__);
        }
        IF_META_AVAILABLE(const cam_frame_idx_range_t, p_good_frame_idx_range,
                CAM_INTF_META_GOOD_FRAME_IDX_RANGE, metadata) {
            good_frame_idx_range = *p_good_frame_idx_range;
            is_good_frame_idx_range_valid = 1;
            CDBG_HIGH("%s: good_frame_idx_range : min: %d, max: %d , num frames = %d",
                __func__, good_frame_idx_range.min_frame_idx,
                good_frame_idx_range.max_frame_idx, good_frame_idx_range.num_led_on_frames);
        }
        IF_META_AVAILABLE(const cam_crop_data_t, p_crop_data,
                CAM_INTF_META_CROP_DATA, metadata) {
            cam_crop_data_t crop_data = *p_crop_data;

            for (i = 0; i < ARRAY_SIZE(ch_obj->streams); i++) {
                if (MM_STREAM_STATE_NOTUSED == ch_obj->streams[i].state) {
                    continue;
                }
                if (CAM_STREAM_TYPE_SNAPSHOT ==
                    ch_obj->streams[i].stream_info->stream_type) {
                    snapshot_stream_id = ch_obj->streams[i].server_stream_id;
                    break;
                }
            }

            for (i=0; i<crop_data.num_of_streams; i++) {
                if (snapshot_stream_id == crop_data.crop_info[i].stream_id) {
                    if (!crop_data.crop_info[i].crop.left &&
                            !crop_data.crop_info[i].crop.top) {
                        is_crop_1x_found = 1;
                        break;
                    }
                }
            }
        }

        IF_META_AVAILABLE(const cam_buf_divert_info_t, p_divert_info,
                CAM_INTF_BUF_DIVERT_INFO, metadata) {
            cam_buf_divert_info_t divert_info = *p_divert_info;
            if (divert_info.frame_id >= buf_info->frame_idx) {
                ch_obj->diverted_frame_id = divert_info.frame_id;
            } else {
                ch_obj->diverted_frame_id = 0;
            }
        }

        if (ch_obj->isZoom1xFrameRequested) {
            if (is_crop_1x_found) {
                ch_obj->isZoom1xFrameRequested = 0;
                queue->expected_frame_id = buf_info->frame_idx + 1;
            } else {
                queue->expected_frame_id += max_future_frame_offset;
                /* Flush unwanted frames */
                mm_channel_superbuf_flush_matched(ch_obj, queue);
            }
            goto end;
        }

        if (is_prep_snapshot_done_valid) {
            ch_obj->bWaitForPrepSnapshotDone = 0;
            if (prep_snapshot_done_state == NEED_FUTURE_FRAME) {
                queue->expected_frame_id += max_future_frame_offset;
                CDBG_HIGH("%s: [ZSL Retro] NEED_FUTURE_FRAME, expected frame id = %d ",
                        __func__,  queue->expected_frame_id);

                mm_channel_superbuf_flush(ch_obj,
                        queue, CAM_STREAM_TYPE_DEFAULT);

                ch_obj->needLEDFlash = TRUE;
            } else {
                ch_obj->needLEDFlash = FALSE;
            }
        }
        if (is_good_frame_idx_range_valid) {
            if (good_frame_idx_range.min_frame_idx > queue->expected_frame_id) {
                CDBG_HIGH("%s: [ZSL Retro] min_frame_idx %d is greater than expected_frame_id %d",
                        __func__, good_frame_idx_range.min_frame_idx, queue->expected_frame_id);
            }
            queue->expected_frame_id =
                good_frame_idx_range.min_frame_idx;
             if((ch_obj->needLEDFlash == TRUE) && (ch_obj->burstSnapNum > 1)) {
                queue->led_on_start_frame_id =
                good_frame_idx_range.min_frame_idx;
                queue->led_off_start_frame_id =
                good_frame_idx_range.max_frame_idx;
                queue->once = 0;
                queue->led_on_num_frames =
                  good_frame_idx_range.num_led_on_frames;
                queue->frame_skip_count = good_frame_idx_range.frame_skip_count;
                CDBG("%s: [ZSL Retro] Need Flash, expected frame id = %d,"
                        " led_on start = %d, led off start = %d, led on frames = %d ",
                        __func__,   queue->expected_frame_id, queue->led_on_start_frame_id,
                        queue->led_off_start_frame_id, queue->led_on_num_frames);
            } else {
                CDBG("%s: [ZSL Retro]No flash, expected frame id = %d ",
                        __func__, queue->expected_frame_id);
            }
        } else if ((MM_CHANNEL_BRACKETING_STATE_WAIT_GOOD_FRAME_IDX == ch_obj->bracketingState) &&
                !is_prep_snapshot_done_valid) {
            /* Flush unwanted frames */
            mm_channel_superbuf_flush_matched(ch_obj, queue);
            queue->expected_frame_id += max_future_frame_offset;
        }
        if (ch_obj->isFlashBracketingEnabled &&
            is_good_frame_idx_range_valid) {
            /* Flash bracketing needs two frames, with & without led flash.
            * in valid range min frame is with led flash and max frame is
            * without led flash */
            queue->expected_frame_id =
                good_frame_idx_range.min_frame_idx;
            /* max frame is without led flash */
            queue->expected_frame_id_without_led =
                good_frame_idx_range.max_frame_idx;

        } else if (is_good_frame_idx_range_valid) {
            if (good_frame_idx_range.min_frame_idx >
                queue->expected_frame_id) {
                CDBG_HIGH("%s: min_frame_idx %d is greater than expected_frame_id %d",
                        __func__, good_frame_idx_range.min_frame_idx,
                        queue->expected_frame_id);
            }
            queue->expected_frame_id =
                    good_frame_idx_range.min_frame_idx;

            ch_obj->bracketingState = MM_CHANNEL_BRACKETING_STATE_ACTIVE;
        }

        if (ch_obj->isConfigCapture && is_good_frame_idx_range_valid
                && (good_frame_idx_range.config_batch_idx < ch_obj->frameConfig.num_batch)) {

            CDBG_HIGH("Frame Config: Expcted ID = %d batch index = %d",
                    good_frame_idx_range.min_frame_idx, good_frame_idx_range.config_batch_idx);
            ch_obj->capture_frame_id[good_frame_idx_range.config_batch_idx] =
                    good_frame_idx_range.min_frame_idx;

            if (ch_obj->cur_capture_idx == good_frame_idx_range.config_batch_idx) {
                queue->expected_frame_id =
                        good_frame_idx_range.min_frame_idx;
            } else {
                queue->expected_frame_id =
                        ch_obj->capture_frame_id[ch_obj->cur_capture_idx];
            }
        }

        if ((ch_obj->burstSnapNum > 1) && (ch_obj->needLEDFlash == TRUE)
            && !ch_obj->isFlashBracketingEnabled
            && (MM_CHANNEL_BRACKETING_STATE_OFF == ch_obj->bracketingState)
            && !ch_obj->isConfigCapture) {
            if((buf_info->frame_idx >= queue->led_off_start_frame_id)
                    &&  !queue->once) {
                CDBG("%s: [ZSL Retro]Burst snap num = %d ",
                        __func__, ch_obj->burstSnapNum);
                // Skip frames from LED OFF frame to get a good frame
                queue->expected_frame_id = queue->led_off_start_frame_id +
                        queue->frame_skip_count;
                queue->once = 1;
                ch_obj->stopZslSnapshot = 1;
                ch_obj->needLEDFlash = FALSE;
                CDBG("%s:[ZSL Retro]Reached max led on frames = %d , expected id = %d",
                        __func__, buf_info->frame_idx, queue->expected_frame_id);
         }
       }
    }
end:
    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_channel_superbuf_comp_and_enqueue
 *
 * DESCRIPTION: implementation for matching logic for superbuf
 *
 * PARAMETERS :
 *   @ch_obj  : channel object
 *   @queue   : superbuf queue
 *   @buf_info: new buffer from stream
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_channel_superbuf_comp_and_enqueue(
                        mm_channel_t* ch_obj,
                        mm_channel_queue_t *queue,
                        mm_camera_buf_info_t *buf_info)
{
    cam_node_t* node = NULL;
    struct cam_list *head = NULL;
    struct cam_list *pos = NULL;
    mm_channel_queue_node_t* super_buf = NULL;
    uint8_t buf_s_idx, i, found_super_buf, unmatched_bundles;
    struct cam_list *last_buf, *insert_before_buf, *last_buf_ptr;

    CDBG("%s: E", __func__);

    for (buf_s_idx = 0; buf_s_idx < queue->num_streams; buf_s_idx++) {
        if (buf_info->stream_id == queue->bundled_streams[buf_s_idx]) {
            break;
        }
    }

    if (buf_s_idx == queue->num_streams) {
        CDBG_ERROR("%s: buf from stream (%d) not bundled", __func__, buf_info->stream_id);
        return -1;
    }

    if(buf_info->frame_idx == 0) {
        mm_channel_qbuf(ch_obj, buf_info->buf);
        return 0;
    }

    if (mm_channel_handle_metadata(ch_obj, queue, buf_info) < 0) {
        mm_channel_qbuf(ch_obj, buf_info->buf);
        return -1;
    }

    if (mm_channel_util_seq_comp_w_rollover(buf_info->frame_idx,
                                            queue->expected_frame_id) < 0) {
        CDBG_HIGH("%s: incoming buf id(%d) is older than expected buf id(%d), will discard it",
                __func__, buf_info->frame_idx, queue->expected_frame_id);
        mm_channel_qbuf(ch_obj, buf_info->buf);
        return 0;
    }

    if((queue->nomatch_frame_id != 0)
            && (queue->nomatch_frame_id > buf_info->frame_idx)
            && (buf_info->buf->stream_type == CAM_STREAM_TYPE_METADATA)) {
        /*Incoming metadata is older than expected*/
        mm_channel_qbuf(ch_obj, buf_info->buf);
        return 0;
    }

    /* comp */
    pthread_mutex_lock(&queue->que.lock);
    head = &queue->que.head.list;
    /* get the last one in the queue which is possibly having no matching */
    pos = head->next;

    found_super_buf = 0;
    unmatched_bundles = 0;
    last_buf = NULL;
    insert_before_buf = NULL;
    last_buf_ptr = NULL;

    while (pos != head) {
        node = member_of(pos, cam_node_t, list);
        super_buf = (mm_channel_queue_node_t*)node->data;

        if (NULL != super_buf) {
            if (super_buf->matched) {
                /* find a matched super buf, move to next one */
                pos = pos->next;
                continue;
            } else if ( buf_info->frame_idx == super_buf->frame_idx
                    /*Pick metadata greater than available frameID*/
                    || ((queue->nomatch_frame_id != 0)
                    && (queue->nomatch_frame_id <= buf_info->frame_idx)
                    && (super_buf->super_buf[buf_s_idx].frame_idx == 0)
                    && (buf_info->buf->stream_type == CAM_STREAM_TYPE_METADATA))
                    /*Pick available metadata closest to frameID*/
                    || ((queue->attr.priority == MM_CAMERA_SUPER_BUF_PRIORITY_LOW)
                    && (buf_info->buf->stream_type != CAM_STREAM_TYPE_METADATA)
                    && (super_buf->super_buf[buf_s_idx].frame_idx == 0)
                    && (super_buf->frame_idx > buf_info->frame_idx))){
                /*super buffer frame IDs matching OR In low priority bundling
                metadata frameID greater than avialbale super buffer frameID  OR
                metadata frame closest to incoming frameID will be bundled*/
                found_super_buf = 1;
                queue->nomatch_frame_id = 0;
                break;
            } else {
                unmatched_bundles++;
                if ( NULL == last_buf ) {
                    if ( super_buf->frame_idx < buf_info->frame_idx ) {
                        last_buf = pos;
                    }
                }
                if ( NULL == insert_before_buf ) {
                    if ( super_buf->frame_idx > buf_info->frame_idx ) {
                        insert_before_buf = pos;
                    }
                }
                pos = pos->next;
            }
        }
    }

    if ( found_super_buf ) {
        if(super_buf->super_buf[buf_s_idx].frame_idx != 0) {
            //This can cause frame drop. We are overwriting same memory.
            pthread_mutex_unlock(&queue->que.lock);
            //CDBG_FATAL("FATAL: frame is already in camera ZSL queue");
            CDBG_ERROR("***FATAL: frame is already in camera ZSL queue***");
            mm_channel_qbuf(ch_obj, buf_info->buf);
            return 0;
        }

        /*Insert incoming buffer to super buffer*/
        super_buf->super_buf[buf_s_idx] = *buf_info;

        /* check if superbuf is all matched */
        super_buf->matched = 1;
        for (i=0; i < super_buf->num_of_bufs; i++) {
            if (super_buf->super_buf[i].frame_idx == 0) {
                super_buf->matched = 0;
                break;
            }
        }

        if (super_buf->matched) {
            if(ch_obj->isFlashBracketingEnabled) {
               queue->expected_frame_id =
                   queue->expected_frame_id_without_led;
               if (buf_info->frame_idx >=
                       queue->expected_frame_id_without_led) {
                   ch_obj->isFlashBracketingEnabled = FALSE;
               }
            } else {
               queue->expected_frame_id = buf_info->frame_idx
                                          + queue->attr.post_frame_skip;
            }

            super_buf->expected = FALSE;

            CDBG("%s: curr = %d, skip = %d , Expected Frame ID: %d",
                    __func__, buf_info->frame_idx,
                    queue->attr.post_frame_skip, queue->expected_frame_id);

            queue->match_cnt++;
            if (ch_obj->bundle.superbuf_queue.attr.enable_frame_sync) {
                pthread_mutex_lock(&fs_lock);
                mm_frame_sync_add(buf_info->frame_idx, ch_obj);
                pthread_mutex_unlock(&fs_lock);
            }
            /* Any older unmatched buffer need to be released */
            if ( last_buf ) {
                while ( last_buf != pos ) {
                    node = member_of(last_buf, cam_node_t, list);
                    super_buf = (mm_channel_queue_node_t*)node->data;
                    if (NULL != super_buf) {
                        for (i=0; i<super_buf->num_of_bufs; i++) {
                            if (super_buf->super_buf[i].frame_idx != 0) {
                                mm_channel_qbuf(ch_obj, super_buf->super_buf[i].buf);
                            }
                        }
                        queue->que.size--;
                        last_buf = last_buf->next;
                        cam_list_del_node(&node->list);
                        free(node);
                        free(super_buf);
                    } else {
                        CDBG_ERROR(" %s : Invalid superbuf in queue!", __func__);
                        break;
                    }
                }
            }
        }else {
            if (ch_obj->diverted_frame_id == buf_info->frame_idx) {
                super_buf->expected = TRUE;
                ch_obj->diverted_frame_id = 0;
            }
        }
    } else {
        if ((queue->attr.max_unmatched_frames < unmatched_bundles)
                && ( NULL == last_buf )) {
            /* incoming frame is older than the last bundled one */
            mm_channel_qbuf(ch_obj, buf_info->buf);
        } else {
            last_buf_ptr = last_buf;

            /* Loop to remove unmatched frames */
            while ((queue->attr.max_unmatched_frames < unmatched_bundles)
                    && (last_buf_ptr != NULL && last_buf_ptr != pos)) {
                node = member_of(last_buf_ptr, cam_node_t, list);
                super_buf = (mm_channel_queue_node_t*)node->data;
                if (NULL != super_buf && super_buf->expected == FALSE
                        && (&node->list != insert_before_buf)) {
                    for (i=0; i<super_buf->num_of_bufs; i++) {
                        if (super_buf->super_buf[i].frame_idx != 0) {
                            mm_channel_qbuf(ch_obj, super_buf->super_buf[i].buf);
                        }
                    }
                    queue->que.size--;
                    cam_list_del_node(&node->list);
                    free(node);
                    free(super_buf);
                    unmatched_bundles--;
                }
                last_buf_ptr = last_buf_ptr->next;
            }

            if (queue->attr.max_unmatched_frames < unmatched_bundles) {
                node = member_of(last_buf, cam_node_t, list);
                super_buf = (mm_channel_queue_node_t*)node->data;
                for (i=0; i<super_buf->num_of_bufs; i++) {
                    if (super_buf->super_buf[i].frame_idx != 0) {
                        mm_channel_qbuf(ch_obj, super_buf->super_buf[i].buf);
                    }
                }
                queue->que.size--;
                cam_list_del_node(&node->list);
                free(node);
                free(super_buf);
            }

            /* insert the new frame at the appropriate position. */

            mm_channel_queue_node_t *new_buf = NULL;
            cam_node_t* new_node = NULL;

            new_buf = (mm_channel_queue_node_t*)malloc(sizeof(mm_channel_queue_node_t));
            new_node = (cam_node_t*)malloc(sizeof(cam_node_t));
            if (NULL != new_buf && NULL != new_node) {
                memset(new_buf, 0, sizeof(mm_channel_queue_node_t));
                memset(new_node, 0, sizeof(cam_node_t));
                new_node->data = (void *)new_buf;
                new_buf->num_of_bufs = queue->num_streams;
                new_buf->super_buf[buf_s_idx] = *buf_info;
                new_buf->frame_idx = buf_info->frame_idx;

                if (ch_obj->diverted_frame_id == buf_info->frame_idx) {
                    new_buf->expected = TRUE;
                    ch_obj->diverted_frame_id = 0;
                }

                /* enqueue */
                if ( insert_before_buf ) {
                    cam_list_insert_before_node(&new_node->list, insert_before_buf);
                } else {
                    cam_list_add_tail_node(&new_node->list, &queue->que.head.list);
                }
                queue->que.size++;

                if(queue->num_streams == 1) {
                    new_buf->matched = 1;
                    new_buf->expected = FALSE;
                    queue->expected_frame_id = buf_info->frame_idx + queue->attr.post_frame_skip;
                    queue->match_cnt++;
                    if (ch_obj->bundle.superbuf_queue.attr.enable_frame_sync) {
                        pthread_mutex_lock(&fs_lock);
                        mm_frame_sync_add(buf_info->frame_idx, ch_obj);
                        pthread_mutex_unlock(&fs_lock);
                    }
                }

                if ((queue->attr.priority == MM_CAMERA_SUPER_BUF_PRIORITY_LOW)
                        && (buf_info->buf->stream_type != CAM_STREAM_TYPE_METADATA)) {
                    CDBG_ERROR ("%s : No metadata matching for frame = %d",
                            __func__, buf_info->frame_idx);
                    queue->nomatch_frame_id = buf_info->frame_idx;
                }
            } else {
                /* No memory */
                if (NULL != new_buf) {
                    free(new_buf);
                }
                if (NULL != new_node) {
                    free(new_node);
                }
                /* qbuf the new buf since we cannot enqueue */
                mm_channel_qbuf(ch_obj, buf_info->buf);
            }
        }
    }

    pthread_mutex_unlock(&queue->que.lock);
    CDBG("%s: X", __func__);
    return 0;
}

/*===========================================================================
 * FUNCTION   : mm_channel_superbuf_dequeue_internal
 *
 * DESCRIPTION: internal implementation for dequeue from the superbuf queue
 *
 * PARAMETERS :
 *   @queue   : superbuf queue
 *   @matched_only : if dequeued buf should be matched
 *   @ch_obj  : channel object
 *
 * RETURN     : ptr to a node from superbuf queue
 *==========================================================================*/
mm_channel_queue_node_t* mm_channel_superbuf_dequeue_internal(
        mm_channel_queue_t * queue,
        uint8_t matched_only, mm_channel_t *ch_obj)
{
    cam_node_t* node = NULL;
    struct cam_list *head = NULL;
    struct cam_list *pos = NULL;
    mm_channel_queue_node_t* super_buf = NULL;

    head = &queue->que.head.list;
    pos = head->next;
    if (pos != head) {
        /* get the first node */
        node = member_of(pos, cam_node_t, list);
        super_buf = (mm_channel_queue_node_t*)node->data;
        if ( (NULL != super_buf) &&
             (matched_only == TRUE) &&
             (super_buf->matched == FALSE) ) {
            /* require to dequeue matched frame only, but this superbuf is not matched,
               simply set return ptr to NULL */
            super_buf = NULL;
        }
        if (NULL != super_buf) {
            /* remove from the queue */
            cam_list_del_node(&node->list);
            queue->que.size--;
            if (super_buf->matched == TRUE) {
                queue->match_cnt--;
                if (ch_obj->bundle.superbuf_queue.attr.enable_frame_sync) {
                    pthread_mutex_lock(&fs_lock);
                    mm_frame_sync_remove(super_buf->frame_idx);
                    pthread_mutex_unlock(&fs_lock);
                }
            }
            free(node);
        }
    }

    return super_buf;
}

/*===========================================================================
 * FUNCTION   : mm_channel_superbuf_dequeue_frame_internal
 *
 * DESCRIPTION: internal implementation for dequeue based on frame index
 *                     from the superbuf queue
 *
 * PARAMETERS :
 *   @queue       : superbuf queue
 *   @frame_idx  : frame index to be dequeued
 *
 * RETURN     : ptr to a node from superbuf queue with matched frame index
 *                : NULL if not found
 *==========================================================================*/
mm_channel_queue_node_t* mm_channel_superbuf_dequeue_frame_internal(
        mm_channel_queue_t * queue, uint32_t frame_idx)
{
    cam_node_t* node = NULL;
    struct cam_list *head = NULL;
    struct cam_list *pos = NULL;
    mm_channel_queue_node_t* super_buf = NULL;

    if (!queue) {
        CDBG_ERROR("%s: queue is NULL", __func__);
        return NULL;
    }

    head = &queue->que.head.list;
    pos = head->next;
    CDBG_HIGH("%s: Searching for match frame %d", __func__, frame_idx);
    while ((pos != head) && (pos != NULL)) {
        /* get the first node */
        node = member_of(pos, cam_node_t, list);
        super_buf = (mm_channel_queue_node_t*)node->data;
        if (super_buf && super_buf->matched &&
                (super_buf->frame_idx == frame_idx)) {
            /* remove from the queue */
            cam_list_del_node(&node->list);
            queue->que.size--;
            queue->match_cnt--;
            CDBG_HIGH("%s: Found match frame %d", __func__, frame_idx);
            free(node);
            break;
        }
        else {
            CDBG_HIGH("%s: match frame not found %d", __func__, frame_idx);
            super_buf = NULL;
        }
        pos = pos->next;
    }
    return super_buf;
}


/*===========================================================================
 * FUNCTION   : mm_channel_superbuf_dequeue
 *
 * DESCRIPTION: dequeue from the superbuf queue
 *
 * PARAMETERS :
 *   @queue   : superbuf queue
 *   @ch_obj  : channel object
 *
 * RETURN     : ptr to a node from superbuf queue
 *==========================================================================*/
mm_channel_queue_node_t* mm_channel_superbuf_dequeue(
        mm_channel_queue_t * queue, mm_channel_t *ch_obj)
{
    mm_channel_queue_node_t* super_buf = NULL;

    pthread_mutex_lock(&queue->que.lock);
    super_buf = mm_channel_superbuf_dequeue_internal(queue, TRUE, ch_obj);
    pthread_mutex_unlock(&queue->que.lock);

    return super_buf;
}

/*===========================================================================
 * FUNCTION   : mm_channel_superbuf_bufdone_overflow
 *
 * DESCRIPTION: keep superbuf queue no larger than watermark set by upper layer
 *              via channel attribute
 *
 * PARAMETERS :
 *   @my_obj  : channel object
 *   @queue   : superbuf queue
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_channel_superbuf_bufdone_overflow(mm_channel_t* my_obj,
                                             mm_channel_queue_t * queue)
{
    int32_t rc = 0, i;
    mm_channel_queue_node_t* super_buf = NULL;
    if (MM_CAMERA_SUPER_BUF_NOTIFY_CONTINUOUS == queue->attr.notify_mode) {
        /* for continuous streaming mode, no overflow is needed */
        return 0;
    }

    CDBG("%s: before match_cnt=%d, water_mark=%d",
         __func__, queue->match_cnt, queue->attr.water_mark);
    /* bufdone overflowed bufs */
    pthread_mutex_lock(&queue->que.lock);
    while (queue->match_cnt > queue->attr.water_mark) {
        super_buf = mm_channel_superbuf_dequeue_internal(queue, TRUE, my_obj);
        if (NULL != super_buf) {
            for (i=0; i<super_buf->num_of_bufs; i++) {
                if (NULL != super_buf->super_buf[i].buf) {
                    mm_channel_qbuf(my_obj, super_buf->super_buf[i].buf);
                }
            }
            free(super_buf);
        }
    }
    pthread_mutex_unlock(&queue->que.lock);
    CDBG("%s: after match_cnt=%d, water_mark=%d",
         __func__, queue->match_cnt, queue->attr.water_mark);

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_channel_superbuf_skip
 *
 * DESCRIPTION: depends on the lookback configuration of the channel attribute,
 *              unwanted superbufs will be removed from the superbuf queue.
 *
 * PARAMETERS :
 *   @my_obj  : channel object
 *   @queue   : superbuf queue
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_channel_superbuf_skip(mm_channel_t* my_obj,
                                 mm_channel_queue_t * queue)
{
    int32_t rc = 0, i;
    mm_channel_queue_node_t* super_buf = NULL;
    if (MM_CAMERA_SUPER_BUF_NOTIFY_CONTINUOUS == queue->attr.notify_mode) {
        /* for continuous streaming mode, no skip is needed */
        return 0;
    }

    /* bufdone overflowed bufs */
    pthread_mutex_lock(&queue->que.lock);
    while (queue->match_cnt > queue->attr.look_back) {
        super_buf = mm_channel_superbuf_dequeue_internal(queue, TRUE, my_obj);
        if (NULL != super_buf) {
            for (i=0; i<super_buf->num_of_bufs; i++) {
                if (NULL != super_buf->super_buf[i].buf) {
                    mm_channel_qbuf(my_obj, super_buf->super_buf[i].buf);
                }
            }
            free(super_buf);
        }
    }
    pthread_mutex_unlock(&queue->que.lock);

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_channel_superbuf_flush
 *
 * DESCRIPTION: flush the superbuf queue.
 *
 * PARAMETERS :
 *   @my_obj  : channel object
 *   @queue   : superbuf queue
 *   @cam_type: flush only particular type (default flushes all)
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_channel_superbuf_flush(mm_channel_t* my_obj,
        mm_channel_queue_t * queue, cam_stream_type_t cam_type)
{
    int32_t rc = 0, i;
    mm_channel_queue_node_t* super_buf = NULL;
    cam_stream_type_t stream_type = CAM_STREAM_TYPE_DEFAULT;

    /* bufdone bufs */
    pthread_mutex_lock(&queue->que.lock);
    super_buf = mm_channel_superbuf_dequeue_internal(queue, FALSE, my_obj);
    while (super_buf != NULL) {
        for (i=0; i<super_buf->num_of_bufs; i++) {
            if (NULL != super_buf->super_buf[i].buf) {
                stream_type = super_buf->super_buf[i].buf->stream_type;
                if ((CAM_STREAM_TYPE_DEFAULT == cam_type) ||
                        (cam_type == stream_type)) {
                    mm_channel_qbuf(my_obj, super_buf->super_buf[i].buf);
                }
            }
        }
        free(super_buf);
        super_buf = mm_channel_superbuf_dequeue_internal(queue, FALSE, my_obj);
    }
    pthread_mutex_unlock(&queue->que.lock);

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_channel_proc_general_cmd
 *
 * DESCRIPTION: process general command
 *
 * PARAMETERS :
 *   @my_obj  : channel object
 *   @notify_mode : notification mode
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_channel_proc_general_cmd(mm_channel_t *my_obj,
                                      mm_camera_generic_cmd_t *p_gen_cmd)
{
    CDBG("%s: E",__func__);
    int32_t rc = 0;
    mm_camera_cmdcb_t* node = NULL;

    node = (mm_camera_cmdcb_t *)malloc(sizeof(mm_camera_cmdcb_t));
    if (NULL != node) {
        memset(node, 0, sizeof(mm_camera_cmdcb_t));
        node->u.gen_cmd = *p_gen_cmd;
        node->cmd_type = MM_CAMERA_CMD_TYPE_GENERAL;

        /* enqueue to cmd thread */
        cam_queue_enq(&(my_obj->cmd_thread.cmd_queue), node);

        /* wake up cmd thread */
        cam_sem_post(&(my_obj->cmd_thread.cmd_sem));
    } else {
        CDBG_ERROR("%s: No memory for mm_camera_node_t", __func__);
        rc = -1;
    }
    CDBG("%s: X",__func__);

    return rc;
}

/*===========================================================================
 * FUNCTION   : mm_channel_superbuf_flush_matched
 *
 * DESCRIPTION: flush matched buffers from the superbuf queue.
 *
 * PARAMETERS :
 *   @my_obj  : channel object
 *   @queue   : superbuf queue
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_channel_superbuf_flush_matched(mm_channel_t* my_obj,
                                  mm_channel_queue_t * queue)
{
    int32_t rc = 0, i;
    mm_channel_queue_node_t* super_buf = NULL;

    /* bufdone bufs */
    pthread_mutex_lock(&queue->que.lock);
    super_buf = mm_channel_superbuf_dequeue_internal(queue, TRUE, my_obj);
    while (super_buf != NULL) {
        for (i=0; i<super_buf->num_of_bufs; i++) {
            if (NULL != super_buf->super_buf[i].buf) {
                mm_channel_qbuf(my_obj, super_buf->super_buf[i].buf);
            }
        }
        free(super_buf);
        super_buf = mm_channel_superbuf_dequeue_internal(queue, TRUE, my_obj);
    }
    pthread_mutex_unlock(&queue->que.lock);

    return rc;
}


/*===========================================================================
 * FUNCTION   : mm_frame_sync_reset
 *
 * DESCRIPTION: Reset Frame sync info
 *
 * RETURN     : None
 *==========================================================================*/
void mm_frame_sync_reset() {
    memset(&fs, 0x0, sizeof(fs));
    CDBG("%s: Reset Done", __func__);
}

/*===========================================================================
 * FUNCTION   : mm_frame_sync_register_channel
 *
 * DESCRIPTION: Register Channel for frame sync
 *
 * PARAMETERS :
 *   @ch_obj  : channel object
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_frame_sync_register_channel(mm_channel_t *ch_obj) {
    // Lock frame sync info
    pthread_mutex_lock(&fs_lock);
    if ((fs.num_cam >= MAX_NUM_CAMERA_PER_BUNDLE) || (!ch_obj)) {
        CDBG_ERROR("%s: DBG_FS Error!! num cam(%d) is out of range ",
                __func__, fs.num_cam);
        pthread_mutex_unlock(&fs_lock);
        return -1;
    }
    if (fs.num_cam == 0) {
        CDBG_HIGH("%s: First channel registering!!", __func__);
        mm_frame_sync_reset();
    }
    uint8_t i = 0;
    for (i = 0; i < MAX_NUM_CAMERA_PER_BUNDLE; i++) {
        if (fs.ch_obj[i] == NULL) {
            fs.ch_obj[i] = ch_obj;
            fs.cb[i] = ch_obj->bundle.super_buf_notify_cb;
            fs.num_cam++;
            CDBG("%s: DBG_FS index %d", __func__, i);
            break;
        }
    }
    if (i >= MAX_NUM_CAMERA_PER_BUNDLE) {
        CDBG_HIGH("%s: X, DBG_FS Cannot register channel!!", __func__);
        pthread_mutex_unlock(&fs_lock);
        return -1;
    }
    CDBG_HIGH("%s: num_cam %d ", __func__, fs.num_cam);
    pthread_mutex_unlock(&fs_lock);
    return 0;
}

/*===========================================================================
 * FUNCTION   : mm_frame_sync_unregister_channel
 *
 * DESCRIPTION: un-register Channel for frame sync
 *
 * PARAMETERS :
 *   @ch_obj  : channel object
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_frame_sync_unregister_channel(mm_channel_t *ch_obj) {
    uint8_t i = 0;
    // Lock frame sync info
    pthread_mutex_lock(&fs_lock);
    if (!fs.num_cam || !ch_obj) {
        CDBG_HIGH("%s: X, DBG_FS: channel not found  !!", __func__);
        // Lock frame sync info
        pthread_mutex_unlock(&fs_lock);
        return -1;
    }
    for (i = 0; i < MAX_NUM_CAMERA_PER_BUNDLE; i++) {
        if (fs.ch_obj[i] == ch_obj) {
            CDBG("%s: found ch_obj at i (%d) ", __func__, i);
            break;
        }
    }
    if (i < MAX_NUM_CAMERA_PER_BUNDLE) {
        CDBG("%s: remove channel info ", __func__);
        fs.ch_obj[i] = NULL;
        fs.cb[i] = NULL;
        fs.num_cam--;
    } else {
        CDBG("%s: DBG_FS Channel not found ", __func__);
    }
    if (fs.num_cam == 0) {
        mm_frame_sync_reset();
    }
    CDBG_HIGH("%s: X, fs.num_cam %d", __func__, fs.num_cam);
    pthread_mutex_unlock(&fs_lock);
    return 0;
}


/*===========================================================================
 * FUNCTION   : mm_frame_sync_add
 *
 * DESCRIPTION: Add frame info into frame sync nodes
 *
 * PARAMETERS :
 *   @frame_id  : frame id to be added
 *   @ch_obj  : channel object
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_frame_sync_add(uint32_t frame_id, mm_channel_t *ch_obj) {

    CDBG("%s: E, frame id %d ch_obj %p", __func__, frame_id, ch_obj);
    if (!frame_id || !ch_obj) {
        CDBG_HIGH("%s: X, DBG_FS Error, cannot add sync frame !!", __func__);
        return -1;
    }

    int8_t ch_idx = -1;
    uint8_t i = 0;
    for (i = 0; i < MAX_NUM_CAMERA_PER_BUNDLE; i++) {
        if (fs.ch_obj[i] == ch_obj) {
            ch_idx = i;
            CDBG("%s: ch id %d ", __func__, ch_idx);
            break;
        }
    }
    if (ch_idx < 0) {
        CDBG_HIGH("%s: X, DBG_FS ch not found!!", __func__);
        return -1;
    }
    int8_t index = mm_frame_sync_find_frame_index(frame_id);
    if ((index >= 0) && (index < MM_CAMERA_FRAME_SYNC_NODES)) {
        fs.node[index].frame_valid[ch_idx] = 1;
    } else if (index < 0) {
        if (fs.pos >= MM_CAMERA_FRAME_SYNC_NODES) {
            fs.pos = 0;
        }
        index = fs.pos;
        memset(&fs.node[index], 0x00, sizeof(mm_channel_sync_node_t));
        fs.pos++;
        fs.node[index].frame_idx = frame_id;
        fs.node[index].frame_valid[ch_idx] = 1;
        if (fs.num_cam == 1) {
            CDBG("%s: Single camera frame %d , matched ", __func__, frame_id);
            fs.node[index].matched = 1;
        }
    }
    uint8_t frames_valid = 0;
    if (!fs.node[index].matched) {
        for (i = 0; i < MAX_NUM_CAMERA_PER_BUNDLE; i++) {
            if (fs.node[index].frame_valid[i]) {
                frames_valid++;
            }
        }
        if (frames_valid == fs.num_cam) {
            fs.node[index].matched = 1;
            CDBG("%s: dual camera frame %d , matched ",
                    __func__, frame_id);
        }
    }
    return 0;
}

/*===========================================================================
 * FUNCTION   : mm_frame_sync_remove
 *
 * DESCRIPTION: Remove frame info from frame sync nodes
 *
 * PARAMETERS :
 *   @frame_id  : frame id to be removed
 *
 * RETURN     : int32_t type of status
 *              0  -- success
 *              -1 -- failure
 *==========================================================================*/
int32_t mm_frame_sync_remove(uint32_t frame_id) {
    int8_t index = -1;

    CDBG("%s: E, frame_id %d", __func__, frame_id);
    if (!frame_id) {
        CDBG("%s: X, DBG_FS frame id invalid", __func__);
        return -1;
    }

    index = mm_frame_sync_find_frame_index(frame_id);
    if ((index >= 0) && (index < MM_CAMERA_FRAME_SYNC_NODES)) {
        CDBG("%s: Removing sync frame %d", __func__, frame_id);
        memset(&fs.node[index], 0x00, sizeof(mm_channel_sync_node_t));
    }
    CDBG("%s: X ", __func__);
    return 0;
}

/*===========================================================================
 * FUNCTION   : mm_frame_sync_find_matched
 *
 * DESCRIPTION: Find  a matched sync frame from the node array
 *
 * PARAMETERS :
 *   @oldest  : If enabled, find oldest matched frame.,
 *                  If not enabled, get the first matched frame found
 *
 * RETURN     : unt32_t type of status
 *              0  -- If no matched frames found
 *              frame index: inf matched frame found
 *==========================================================================*/
uint32_t mm_frame_sync_find_matched(uint8_t oldest) {
    CDBG_HIGH("%s: E, oldest %d ", __func__, oldest);
    uint8_t i = 0;
    uint32_t frame_idx = 0;
    uint32_t curr_frame_idx = 0;
    for (i = 0; i < MM_CAMERA_FRAME_SYNC_NODES; i++) {
        if (fs.node[i].matched) {
            curr_frame_idx = fs.node[i].frame_idx;
            if (!frame_idx) {
                frame_idx = curr_frame_idx;
            }
            if (!oldest) {
                break;
            } else if (frame_idx > curr_frame_idx) {
                frame_idx = curr_frame_idx;
            }
        }
    }
    CDBG_HIGH("%s: X, oldest %d frame idx %d", __func__, oldest, frame_idx);
    return frame_idx;
}

/*===========================================================================
 * FUNCTION   : mm_frame_sync_find_frame_index
 *
 * DESCRIPTION: Find sync frame index if present
 *
 * PARAMETERS :
 *   @frame_id  : frame id to be searched
 *
 * RETURN     : int8_t type of status
 *              -1  -- If desired frame not found
 *              index: node array index if frame is found
 *==========================================================================*/
int8_t mm_frame_sync_find_frame_index(uint32_t frame_id) {

    CDBG("%s: E, frame_id %d", __func__, frame_id);
    int8_t index = -1, i = 0;
    for (i = 0; i < MM_CAMERA_FRAME_SYNC_NODES; i++) {
        if (fs.node[i].frame_idx == frame_id) {
            index = i;
            break;
        }
    }
    CDBG("%s: X index :%d", __func__, index);
    return index;
}

/*===========================================================================
 * FUNCTION   : mm_frame_sync_lock_queues
 *
 * DESCRIPTION: Lock all channel queues present in node info
 *
 * RETURN     : None
 *==========================================================================*/
void mm_frame_sync_lock_queues() {
    uint8_t j = 0;
    ALOGI("%s: E ", __func__);
    for (j = 0; j < MAX_NUM_CAMERA_PER_BUNDLE; j++) {
        if (fs.ch_obj[j]) {
            mm_channel_queue_t *ch_queue =
                    &fs.ch_obj[j]->bundle.superbuf_queue;
            if (ch_queue) {
                pthread_mutex_lock(&ch_queue->que.lock);
                ALOGI("%s: Done locking fs.ch_obj[%d] ", __func__, j);
            }
        }
    }
    ALOGI("%s: Locking fs ", __func__);
    pthread_mutex_lock(&fs_lock);
    ALOGI("%s: X ", __func__);
}

/*===========================================================================
 * FUNCTION   : mm_frame_sync_unlock_queues
 *
 * DESCRIPTION: Unlock all channel queues
 *
 * RETURN     : None
 *==========================================================================*/
void mm_frame_sync_unlock_queues() {
    // Unlock all queues
    uint8_t j = 0;
    ALOGI("%s: E ", __func__);
    pthread_mutex_unlock(&fs_lock);
    ALOGI("%s: Done unlocking fs ", __func__);
    for (j = 0; j < MAX_NUM_CAMERA_PER_BUNDLE; j++) {
        if (fs.ch_obj[j]) {
            mm_channel_queue_t *ch_queue =
                    &fs.ch_obj[j]->bundle.superbuf_queue;
            if (ch_queue) {
                pthread_mutex_unlock(&ch_queue->que.lock);
                ALOGI("%s: Done unlocking fs.ch_obj[%d] ", __func__, j);
            }
        }
    }
    ALOGI("%s: X ", __func__);
}

/*===========================================================================
 * FUNCTION   : mm_channel_node_qbuf
 *
 * DESCRIPTION: qbuf all buffers in a node
 *
 * PARAMETERS :
 *   @ch_obj  : Channel info
 *   @node    : node to qbuf
 *
 * RETURN     : None
 *==========================================================================*/
void mm_channel_node_qbuf(mm_channel_t *ch_obj, mm_channel_queue_node_t *node) {
    uint8_t i;
    if (!ch_obj || !node) {
        return;
    }
    for (i = 0; i < node->num_of_bufs; i++) {
        mm_channel_qbuf(ch_obj, node->super_buf[i].buf);
    }
    return;
}
