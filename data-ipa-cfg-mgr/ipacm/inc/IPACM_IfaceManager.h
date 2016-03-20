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
	IPACM_IfaceManager.h

	@brief
	This file implements the IPAM iface_manager definitions

	@Author
	Skylar Chang

*/
#ifndef IPACM_IFACEMANAGER_H
#define IPACM_IFACEMANAGER_H

#include <stdio.h>
#include <IPACM_CmdQueue.h>

#include "IPACM_Routing.h"
#include "IPACM_Filtering.h"
#include "IPACM_Listener.h"
#include "IPACM_Iface.h"

#define IPA_MAX_NUM_NEIGHBOR_CLIENTS  17
#define IPA_INSTANCE_NOT_FOUND  0
#define IPA_INSTANCE_FOUND  1

/* queue */
typedef struct _iface_instances
{
    /* Linux interface id */
	int ipa_if_index;
	IPACM_Listener *obj;
	_iface_instances *next;
}  iface_instances;


class IPACM_IfaceManager : public IPACM_Listener
{

public:

  IPACM_IfaceManager();

  void event_callback(ipa_cm_event_id event,
                      void *data);

  /* api for all iface instances to de-register instances */
  static int deregistr(IPACM_Listener *param);


private:
	int create_iface_instance(ipacm_ifacemgr_data *);

    /* api to register instances */
	int registr(int ipa_if_index, IPACM_Listener *obj);

	int SearchInstance(int ipa_if_index);

	static iface_instances *head;

};

#endif /* IPACM_IFACEMANAGER_H */
