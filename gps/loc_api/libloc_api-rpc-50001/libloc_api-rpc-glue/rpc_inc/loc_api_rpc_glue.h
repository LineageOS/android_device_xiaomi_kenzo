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

#ifndef LOC_API_RPC_GLUE_H
#define LOC_API_RPC_GLUE_H

/* Include RPC headers */
#ifdef USE_LOCAL_RPC
#include "rpc_inc/loc_api_common.h"
#include "rpc_inc/loc_api.h"
#include "rpc_inc/loc_api_cb.h"
#endif

#ifdef USE_QCOM_AUTO_RPC
#include "loc_api_rpcgen_rpc.h"
#include "loc_api_rpcgen_common_rpc.h"
#include "loc_api_rpcgen_cb_rpc.h"
#endif

/* Boolean */
/* Other data types in comdef.h are defined in rpc stubs, so fix it here */
typedef unsigned char boolean;
#define TRUE 1
#define FALSE 0

#include "loc_api_fixup.h"
#include "loc_api_sync_call.h"
#include <rpc/clnt.h>

#ifdef __cplusplus
extern "C"
{
#endif

extern int loc_api_glue_init(void);
extern int loc_api_null(void);

typedef int32 (loc_event_cb_f_type)(
    void*                                 userData,
    rpc_loc_client_handle_type            loc_handle,             /* handle of the client */
    rpc_loc_event_mask_type               loc_event,              /* event mask           */
    const rpc_loc_event_payload_u_type*   loc_event_payload       /* payload              */
);

typedef void (loc_reset_notif_cb_f_type)(
    void*                                 userData,
    CLIENT*                               clnt,
    enum rpc_reset_event                  event
);

extern rpc_loc_client_handle_type loc_open(
    rpc_loc_event_mask_type       event_reg_mask,
    loc_event_cb_f_type           *event_callback,
    loc_reset_notif_cb_f_type     *rpc_global_cb,
    void*                         userData
);

extern int32 loc_close
(
      rpc_loc_client_handle_type handle
);

extern void loc_clear
(
      rpc_loc_client_handle_type handle
);

extern int32 loc_start_fix
(
      rpc_loc_client_handle_type handle
);

extern int32 loc_stop_fix
(
      rpc_loc_client_handle_type handle
);

extern int32 loc_ioctl
(
      rpc_loc_client_handle_type           handle,
      rpc_loc_ioctl_e_type                 ioctl_type,
      rpc_loc_ioctl_data_u_type*           ioctl_data
);

extern int loc_eng_ioctl
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

#endif /* LOC_API_RPC_GLUE_H */
