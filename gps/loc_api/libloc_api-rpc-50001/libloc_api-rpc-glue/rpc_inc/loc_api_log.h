/* Copyright (c) 2011 The Linux Foundation. All rights reserved.
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

#ifndef LOC_API_LOG_H
#define LOC_API_LOG_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <ctype.h>
#include "loc_api_rpcgen_common_rpc.h"

extern int loc_callback_log(
      rpc_loc_event_mask_type               loc_event,              /* event mask           */
      const rpc_loc_event_payload_u_type*   loc_event_payload       /* payload              */
);

extern const char* loc_get_event_atl_open_name(rpc_loc_server_request_e_type loc_event_atl_open);
extern const char* loc_get_event_name(rpc_loc_event_mask_type loc_event_mask);
extern const char* loc_get_ioctl_type_name(rpc_loc_ioctl_e_type ioctl_type);
extern const char* loc_get_ioctl_status_name(uint32 status);
extern const char* loc_get_sess_status_name(rpc_loc_session_status_e_type status);
extern const char* loc_get_engine_state_name(rpc_loc_engine_state_e_type state);
extern const char* loc_get_fix_session_state_name(rpc_loc_fix_session_state_e_type state);
extern const char* loc_get_rpc_reset_event_name(enum rpc_reset_event event);

#ifdef __cplusplus
}
#endif

#endif /* LOC_API_LOG_H */
