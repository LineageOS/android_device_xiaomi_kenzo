/* 
Copyright (c) 2013, The Linux Foundation. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of The Linux Foundation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*!
	@file
	IPACM_EvtDispatcher.h

	@brief
	This file implements the IPAM event dispatcher definitions

	@Author

*/
#ifndef IPACM_EvtDispatcher_H
#define IPACM_EvtDispatcher_H

#include <stdio.h>
#include <IPACM_CmdQueue.h>
#include "IPACM_Defs.h"
#include "IPACM_Listener.h"

/* queue */
typedef struct _cmd_evts
{
	ipa_cm_event_id event;
	IPACM_Listener *obj;
	//int ipa_interface_index;
	_cmd_evts *next;
}  cmd_evts;



class IPACM_EvtDispatcher
{
public:

	/* api for all iface instances to register events */
	static int registr(ipa_cm_event_id event, IPACM_Listener *obj);

	/* api for all iface instances to de-register events */
	static int deregistr(IPACM_Listener *obj);

	static int PostEvt(ipacm_cmd_q_data *);
	static void ProcessEvt(ipacm_cmd_q_data *);

private:
	static cmd_evts *head;
};

#endif /* IPACM_EvtDispatcher_H */
