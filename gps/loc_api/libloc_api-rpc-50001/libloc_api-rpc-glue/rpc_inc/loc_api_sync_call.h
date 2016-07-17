/* Copyright (c) 2011, The Linux Foundation. All rights reserved.
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
 */

#ifndef LOC_API_CB_SYNC_H
#define LOC_API_CB_SYNC_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "loc_api_rpc_glue.h"
#define LOC_SYNC_CALL_SLOTS_MAX 8

typedef struct {
   pthread_mutex_t                lock;

   /* Client ID */
   rpc_loc_client_handle_type     loc_handle;

   /* Callback waiting conditional variable */
   pthread_cond_t                 loc_cb_arrived_cond;

   /* Callback waiting data block, protected by loc_cb_data_mutex */
   boolean                        in_use;
   boolean                        signal_sent;
   boolean                        not_available;
   rpc_loc_event_mask_type        loc_cb_wait_event_mask;        /* event to wait for */
   rpc_loc_ioctl_e_type           ioctl_type;                    /* ioctl to wait for */
   rpc_loc_event_payload_u_type   loc_cb_received_payload;       /* received payload */
   rpc_loc_event_mask_type        loc_cb_received_event_mask;    /* received event   */
} loc_sync_call_slot_s_type;

typedef struct {
   int                            num_of_slots;
   loc_sync_call_slot_s_type      slots[LOC_SYNC_CALL_SLOTS_MAX];
} loc_sync_call_slot_array_s_type;

/* Init function */
void loc_api_sync_call_init();

/* Destroy function */
void loc_api_sync_call_destroy();

/* Process Loc API callbacks to wake up blocked user threads */
void loc_api_callback_process_sync_call(
      rpc_loc_client_handle_type            loc_handle,             /* handle of the client */
      rpc_loc_event_mask_type               loc_event,              /* event mask           */
      const rpc_loc_event_payload_u_type*   loc_event_payload       /* payload              */
);

/* Reentrant synchronous IOCTL call, using Loc API return code */
int loc_api_sync_ioctl
(
      rpc_loc_client_handle_type           handle,
      rpc_loc_ioctl_e_type                 ioctl_type,
      rpc_loc_ioctl_data_u_type*           ioctl_data_ptr,
      uint32                               timeout_msec,
      rpc_loc_ioctl_callback_s_type       *cb_data_ptr
);

#ifdef __cplusplus
}
#endif

#endif /* LOC_API_CB_SYNC_H */
