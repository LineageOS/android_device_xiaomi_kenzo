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

#include "rpc/rpc.h"

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

#include "rpc_inc/loc_api_fixup.h"
#include "loc_apicb_appinit.h"

#define RPC_FUNC_VERSION_BASE(a,b) a ## b
#define RPC_CB_FUNC_VERS(a,b) RPC_FUNC_VERSION_BASE(a,b)

static SVCXPRT* svrPort = NULL;

extern void RPC_CB_FUNC_VERS(loc_apicbprog_,LOC_APICBVERS_0001)(struct svc_req *rqstp, register SVCXPRT *transp);

int loc_apicb_app_init(void)
{

  /* Register a callback server to use the loc_apicbprog_* function  */
  if (svrPort == NULL) {
        svrPort = svcrtr_create();
  }
  if (!svrPort) return -1;

  xprt_register(svrPort);
  if(svc_register(svrPort, LOC_APICBPROG, LOC_APICBVERS_0001, RPC_CB_FUNC_VERS(loc_apicbprog_,LOC_APICBVERS_0001),0))
  {
     return 0;
  }
  else
  {
    return -1;
  }
}

void loc_apicb_app_deinit(void)
{
   if (svrPort == NULL)
   {
      return;
   }

   svc_unregister(svrPort, LOC_APICBPROG, LOC_APICBVERS_0001);
   xprt_unregister(svrPort);
   svc_destroy(svrPort);
   svrPort = NULL;
}

