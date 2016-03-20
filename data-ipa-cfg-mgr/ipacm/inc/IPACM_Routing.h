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
	IPACM_Routing.cpp

	@brief
	This file implements the IPACM routing functionality.

	@Author
	Skylar Chang

*/


#ifndef IPACM_ROUTING_H
#define IPACM_ROUTING_H

#include <stdint.h>
#include <linux/msm_ipa.h>
#include <IPACM_Defs.h>

using namespace std;

class IPACM_Routing
{
public:
	IPACM_Routing();
	~IPACM_Routing();

	bool AddRoutingRule(struct ipa_ioc_add_rt_rule *ruleTable);
	bool DeleteRoutingRule(struct ipa_ioc_del_rt_rule *ruleTable);

	bool Commit(enum ipa_ip_type ip);
	bool Reset(enum ipa_ip_type ip);

	bool GetRoutingTable(struct ipa_ioc_get_rt_tbl *routingTable);
	bool PutRoutingTable(uint32_t routingTableHandle);

	bool DeviceNodeIsOpened();
	bool DeleteRoutingHdl(uint32_t rt_rule_hdl, ipa_ip_type ip);

	bool ModifyRoutingRule(struct ipa_ioc_mdfy_rt_rule *);

private:
	static const char *DEVICE_NAME;
	int m_fd; /* File descriptor of the IPA device node /dev/ipa */
};

#endif //IPACM_ROUTING_H

