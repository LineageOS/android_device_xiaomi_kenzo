/* 
Copyright (c) 2013-2016, The Linux Foundation. All rights reserved.

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
	IPACM_EvtDispatcher.cpp

	@brief
	This file implements the IPAM event dispatcher functionality

	@Author

*/
#include <string.h>
#include <pthread.h>
#include <IPACM_EvtDispatcher.h>
#include <IPACM_Neighbor.h>
#include "IPACM_CmdQueue.h"
#include "IPACM_Defs.h"


extern pthread_mutex_t mutex;
extern pthread_cond_t  cond_var;

cmd_evts *IPACM_EvtDispatcher::head = NULL;
extern uint32_t ipacm_event_stats[IPACM_EVENT_MAX];

int IPACM_EvtDispatcher::PostEvt
(
	 ipacm_cmd_q_data *data
)
{
	Message *item = NULL;
	MessageQueue *MsgQueue = NULL;

	if(data->event < IPA_EXTERNAL_EVENT_MAX)
	{
		IPACMDBG("Insert event into external queue.\n");
		MsgQueue = MessageQueue::getInstanceExternal();
	}
	else
	{
		IPACMDBG("Insert event into internal queue.\n");
		MsgQueue = MessageQueue::getInstanceInternal();
	}
	if(MsgQueue == NULL)
	{
		IPACMERR("unable to retrieve MsgQueue instance\n");
		return IPACM_FAILURE;
	}

	item = new Message();
	if(item == NULL)
	{
		IPACMERR("unable to create new message item\n");
		return IPACM_FAILURE;
	}

	item->evt.callback_ptr = IPACM_EvtDispatcher::ProcessEvt;
	memcpy(&item->evt.data, data, sizeof(ipacm_cmd_q_data));

	if(pthread_mutex_lock(&mutex) != 0)
	{
		IPACMERR("unable to lock the mutex\n");
		return IPACM_FAILURE;
	}

	IPACMDBG("Enqueing item\n");
	MsgQueue->enqueue(item);
	IPACMDBG("Enqueued item %p\n", item);

	if(pthread_cond_signal(&cond_var) != 0)
	{
		IPACMDBG("unable to lock the mutex\n");
		/* Release the mutex before you return failure */
		if(pthread_mutex_unlock(&mutex) != 0)
		{
			IPACMERR("unable to unlock the mutex\n");
			return IPACM_FAILURE;
		}
		return IPACM_FAILURE;
	}

	if(pthread_mutex_unlock(&mutex) != 0)
	{
		IPACMERR("unable to unlock the mutex\n");
		return IPACM_FAILURE;
	}

	return IPACM_SUCCESS;
}

void IPACM_EvtDispatcher::ProcessEvt(ipacm_cmd_q_data *data)
{

	cmd_evts *tmp = head, tmp1;

	if(head == NULL)
	{
		IPACMDBG("Queue is empty\n");
	}

	while(tmp != NULL)
	{
	        memcpy(&tmp1, tmp, sizeof(tmp1));
		if(data->event == tmp1.event)
		{
			ipacm_event_stats[data->event]++;
			tmp1.obj->event_callback(data->event, data->evt_data);
			IPACMDBG(" Find matched registered events\n");
		}
	        tmp = tmp1.next;
	}

	IPACMDBG(" Finished process events\n");
			
	if(data->evt_data != NULL)
	{
		IPACMDBG("free the event:%d data: %p\n", data->event, data->evt_data);
		free(data->evt_data);
	}
	return;
}

int IPACM_EvtDispatcher::registr(ipa_cm_event_id event, IPACM_Listener *obj)
{
	cmd_evts *tmp = head,*nw;

	nw = (cmd_evts *)malloc(sizeof(cmd_evts));
	if(nw != NULL)
	{
		nw->event = event;
		nw->obj = obj;
		nw->next = NULL;
	}
	else
	{
		return IPACM_FAILURE;
	}

	if(head == NULL)
	{
		head = nw;
	}
	else
	{
		while(tmp->next)
		{
			tmp = tmp->next;
		}
		tmp->next = nw;
	}
	return IPACM_SUCCESS;
}


int IPACM_EvtDispatcher::deregistr(IPACM_Listener *param)
{
	cmd_evts *tmp = head,*tmp1,*prev = head;

	while(tmp != NULL)
	{
		if(tmp->obj == param)
		{
			tmp1 = tmp;
			if(tmp == head)
			{
				head = head->next;
			}
			else if(tmp->next == NULL)
			{
				prev->next = NULL;
			}
			else
			{
				prev->next = tmp->next;
			}

			tmp = tmp->next;
			free(tmp1);
		}
		else
		{
			prev = tmp;
			tmp = tmp->next;
		}
	}
	return IPACM_SUCCESS;
}
