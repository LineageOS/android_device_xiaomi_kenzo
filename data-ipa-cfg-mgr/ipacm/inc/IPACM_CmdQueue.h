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
	IPACM_CmdQueue.h

	@brief
	This file implements the IPAM Comment Queue definitions

	@Author

*/
#ifndef IPA_CONNTRACK_MESSAGE_H
#define IPA_CONNTRACK_MESSAGE_H

#include <pthread.h>
#include "IPACM_Defs.h"



/*---------------------------------------------------------------------------
	 Event data required by IPA_CM
---------------------------------------------------------------------------*/


typedef struct _ipacm_cmd_q_data {
	ipa_cm_event_id event;
	void *evt_data;
}ipacm_cmd_q_data;

typedef struct cmd_s
{
	void (*callback_ptr)(ipacm_cmd_q_data *);
	ipacm_cmd_q_data data;
}cmd_t;

class Message
{
private:
	Message *m_next;

public:
	cmd_t evt;

	Message()
	{
		m_next = NULL;
		evt.callback_ptr = NULL;
	}
	~Message() { }
	void setnext(Message *item) { m_next = item; }
	Message* getnext()       { return m_next; }
};

class MessageQueue
{

private:
	Message *Head;
	Message *Tail;
	Message* dequeue(void);
	static MessageQueue *inst;

	MessageQueue()
	{
		Head = NULL;
		Tail = NULL;
	}

public:

	~MessageQueue() { }
	void enqueue(Message *item);

	static void* Process(void *);
	static MessageQueue* getInstance();

};

#endif  /* IPA_CONNTRACK_MESSAGE_H */

