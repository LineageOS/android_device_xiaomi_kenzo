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
	IPACM_CmdQueue.cpp

	@brief
	This file implements the IPAM Comment Queue functionality

	@Author 
   Sunil

*/
#include <string.h>
#include "IPACM_CmdQueue.h"
#include "IPACM_Log.h"
#include "IPACM_Iface.h"

pthread_mutex_t mutex    = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  cond_var = PTHREAD_COND_INITIALIZER;

MessageQueue* MessageQueue::inst_internal = NULL;
MessageQueue* MessageQueue::inst_external = NULL;

MessageQueue* MessageQueue::getInstanceInternal()
{
	if(inst_internal == NULL)
	{
		inst_internal = new MessageQueue();
		if(inst_internal == NULL)
		{
			IPACMERR("unable to create internal Message Queue instance\n");
			return NULL;
		}
	}

	return inst_internal;
}

MessageQueue* MessageQueue::getInstanceExternal()
{
	if(inst_external == NULL)
	{
		inst_external = new MessageQueue();
		if(inst_external == NULL)
		{
			IPACMERR("unable to create external Message Queue instance\n");
			return NULL;
		}
	}

	return inst_external;
}

void MessageQueue::enqueue(Message *item)
{
	if(!Head)
	{
		Tail = item;
		Head = item;
	}
	else
	{
		if(Tail == NULL)
		{
			IPACMDBG("Tail is null\n");
			Head->setnext(item);
		}
		else
		{
			Tail->setnext(item);
		}
		Tail = item;
	}
}


Message* MessageQueue::dequeue(void)
{
	if(Head == NULL)
	{
		return NULL;
	}
	else
	{
		Message *tmp = Head;
		Head = Head->getnext();

		return tmp;
	}
}


void* MessageQueue::Process(void *param)
{
	MessageQueue *MsgQueueInternal = NULL;
	MessageQueue *MsgQueueExternal = NULL;
	Message *item = NULL;
	IPACMDBG("MessageQueue::Process()\n");

	MsgQueueInternal = MessageQueue::getInstanceInternal();
	if(MsgQueueInternal == NULL)
	{
		IPACMERR("unable to start internal cmd queue process\n");
		return NULL;
	}

	MsgQueueExternal = MessageQueue::getInstanceExternal();
	if(MsgQueueExternal == NULL)
	{
		IPACMERR("unable to start external cmd queue process\n");
		return NULL;
	}

	while(1)
	{
		if(pthread_mutex_lock(&mutex) != 0)
		{
			IPACMERR("unable to lock the mutex\n");
			return NULL;
		}

		item = MsgQueueInternal->dequeue();
		if(item == NULL)
		{
			item = MsgQueueExternal->dequeue();
			if(item)
			{
				IPACMDBG("Get event %s from external queue.\n",
					IPACM_Iface::ipacmcfg->getEventName(item->evt.data.event));
			}
		}
		else
		{
			IPACMDBG("Get event %s from internal queue.\n",
				IPACM_Iface::ipacmcfg->getEventName(item->evt.data.event));
		}

		if(item == NULL)
		{
			IPACMDBG("Waiting for Message\n");

			if(pthread_cond_wait(&cond_var, &mutex) != 0)
			{
				IPACMERR("unable to lock the mutex\n");

				if(pthread_mutex_unlock(&mutex) != 0)
				{
					IPACMERR("unable to unlock the mutex\n");
					return NULL;
				}

				return NULL;
			}

			if(pthread_mutex_unlock(&mutex) != 0)
			{
				IPACMERR("unable to unlock the mutex\n");
				return NULL;
			}

		}
		else
		{
			if(pthread_mutex_unlock(&mutex) != 0)
			{
				IPACMERR("unable to unlock the mutex\n");
				return NULL;
			}

			IPACMDBG("Processing item %p event ID: %d\n",item,item->evt.data.event);
			item->evt.callback_ptr(&item->evt.data);
			delete item;
			item = NULL;
		}

	} /* Go forever until a termination indication is received */

}
