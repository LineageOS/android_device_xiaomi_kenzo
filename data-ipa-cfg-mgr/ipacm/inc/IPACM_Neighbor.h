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
	IPACM_Neighbor.h

	@brief
	This file implements the functionality of handling IPACM Neighbor events.

	@Author
	Skylar Chang

*/
#ifndef IPACM_NEIGHBOR_H
#define IPACM_NEIGHBOR_H

#include <stdio.h>
#include <IPACM_CmdQueue.h>
#include <linux/msm_ipa.h>
#include "IPACM_Routing.h"
#include "IPACM_Filtering.h"
#include "IPACM_Listener.h"
#include "IPACM_Iface.h"

#define IPA_MAX_NUM_NEIGHBOR_CLIENTS  100

struct ipa_neighbor_client
{
	uint8_t mac_addr[6];
	int iface_index;
	uint32_t v4_addr;
	int ipa_if_num;
};

class IPACM_Neighbor : public IPACM_Listener
{

public:

	IPACM_Neighbor();

	void event_callback(ipa_cm_event_id event,
											void *data);

private:

	int num_neighbor_client;

	int circular_index;

	ipa_neighbor_client neighbor_client[IPA_MAX_NUM_NEIGHBOR_CLIENTS];

};

#endif /* IPACM_NEIGHBOR_H */
