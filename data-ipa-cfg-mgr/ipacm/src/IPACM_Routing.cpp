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

*/

#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>

#include "IPACM_Routing.h"
#include <IPACM_Log.h>

const char *IPACM_Routing::DEVICE_NAME = "/dev/ipa";

IPACM_Routing::IPACM_Routing()
{
	m_fd = open(DEVICE_NAME, O_RDWR);
	if (0 == m_fd)
	{
		IPACMERR("Failed opening %s.\n", DEVICE_NAME);
	}
}

IPACM_Routing::~IPACM_Routing()
{
	close(m_fd);
}

bool IPACM_Routing::DeviceNodeIsOpened()
{
	int res = fcntl(m_fd, F_GETFL);

	if (m_fd > 0 && res >= 0) return true;
	else return false;

}

bool IPACM_Routing::AddRoutingRule(struct ipa_ioc_add_rt_rule *ruleTable)
{
	int retval = 0, cnt=0;
	bool isInvalid = false;

	if (!DeviceNodeIsOpened())
	{
		IPACMERR("Device is not opened\n");
		return false;
	}

	for(cnt=0; cnt<ruleTable->num_rules; cnt++)
	{
		if(ruleTable->rules[cnt].rule.dst > IPA_CLIENT_MAX)
		{
			IPACMERR("Invalid dst pipe, Rule:%d  dst_pipe:%d\n", cnt, ruleTable->rules[cnt].rule.dst);
			isInvalid = true;
		}
	}

	if(isInvalid)
	{
		return false;
	}

	retval = ioctl(m_fd, IPA_IOC_ADD_RT_RULE, ruleTable);
	if (retval)
	{
		IPACMERR("Failed adding routing rule %p\n", ruleTable);
		return false;
	}

	for(cnt=0; cnt<ruleTable->num_rules; cnt++)
	{
		IPACMDBG("Rule:%d  dst_pipe:%d\n", cnt, ruleTable->rules[cnt].rule.dst);
	}

	IPACMDBG_H("Added routing rule %p\n", ruleTable);
	return true;
}

bool IPACM_Routing::DeleteRoutingRule(struct ipa_ioc_del_rt_rule *ruleTable)
{
	int retval = 0;

	if (!DeviceNodeIsOpened()) return false;

	retval = ioctl(m_fd, IPA_IOC_DEL_RT_RULE, ruleTable);
	if (retval)
	{
		IPACMERR("Failed deleting routing rule table %p\n", ruleTable);
		return false;
	}

	IPACMDBG_H("Deleted routing rule %p\n", ruleTable);
	return true;
}

bool IPACM_Routing::Commit(enum ipa_ip_type ip)
{
	int retval = 0;

	if (!DeviceNodeIsOpened()) return false;

	retval = ioctl(m_fd, IPA_IOC_COMMIT_RT, ip);
	if (retval)
	{
		IPACMERR("Failed commiting routing rules.\n");
		return false;
	}

	IPACMDBG_H("Commited routing rules to IPA HW.\n");
	return true;
}

bool IPACM_Routing::Reset(enum ipa_ip_type ip)
{
	int retval = 0;

	if (!DeviceNodeIsOpened()) return false;

	retval = ioctl(m_fd, IPA_IOC_RESET_RT, ip);
	retval |= ioctl(m_fd, IPA_IOC_COMMIT_RT, ip);
	if (retval)
	{
		IPACMERR("Failed resetting routing block.\n");
		return false;
	}

	IPACMDBG_H("Reset command issued to IPA routing block.\n");
	return true;
}

bool IPACM_Routing::GetRoutingTable(struct ipa_ioc_get_rt_tbl *routingTable)
{
	int retval = 0;

	if (!DeviceNodeIsOpened()) return false;

	retval = ioctl(m_fd, IPA_IOC_GET_RT_TBL, routingTable);
	if (retval)
	{
		IPACMERR("IPA_IOCTL_GET_RT_TBL ioctl failed, routingTable =0x%p, retval=0x%x.\n", routingTable, retval);
		return false;
	}

	IPACMDBG_H("IPA_IOCTL_GET_RT_TBL ioctl issued to IPA routing block.\n");
	return true;
}

bool IPACM_Routing::PutRoutingTable(uint32_t routingTableHandle)
{
	int retval = 0;

	if (!DeviceNodeIsOpened()) return false;

	retval = ioctl(m_fd, IPA_IOC_PUT_RT_TBL, routingTableHandle);
	if (retval)
	{
		IPACMERR("IPA_IOCTL_PUT_RT_TBL ioctl failed.\n");
		return false;
	}

	IPACMDBG_H("IPA_IOCTL_PUT_RT_TBL ioctl issued to IPA routing block.\n");
	return true;
}

bool IPACM_Routing::DeleteRoutingHdl(uint32_t rt_rule_hdl, ipa_ip_type ip)
{
	const uint8_t NUM_RULES = 1;
	struct ipa_ioc_del_rt_rule *rt_rule;
	struct ipa_rt_rule_del *rt_rule_entry;
	bool res = true;
	int len = 0;

	if (rt_rule_hdl == 0)
	{
		IPACMERR(" No route handle passed. Ignoring it\n");
		return res;
	}

	len = (sizeof(struct ipa_ioc_del_rt_rule)) + (NUM_RULES * sizeof(struct ipa_rt_rule_del));
	rt_rule = (struct ipa_ioc_del_rt_rule *)malloc(len);
	if (rt_rule == NULL)
	{
		IPACMERR("unable to allocate memory for del route rule\n");
		return false;
	}

	memset(rt_rule, 0, len);
	rt_rule->commit = 1;
	rt_rule->num_hdls = NUM_RULES;
	rt_rule->ip = ip;

	rt_rule_entry = &rt_rule->hdl[0];
	rt_rule_entry->status = -1;
	rt_rule_entry->hdl = rt_rule_hdl;

	IPACMDBG_H("Deleting Route hdl:(0x%x) with ip type: %d\n", rt_rule_entry->hdl, ip);
	if ((false == DeleteRoutingRule(rt_rule)) ||
			(rt_rule_entry->status))
	{
		PERROR("Routing rule deletion failed!\n");
		goto fail;
		res = false;
	}

fail:
	free(rt_rule);

	return res;
}

bool IPACM_Routing::ModifyRoutingRule(struct ipa_ioc_mdfy_rt_rule *mdfyRules)
{
	int retval = 0, cnt;

	if (!DeviceNodeIsOpened())
	{
		IPACMERR("Device is not opened\n");
		return false;
	}

	retval = ioctl(m_fd, IPA_IOC_MDFY_RT_RULE, mdfyRules);
	if (retval)
	{
		IPACMERR("Failed modifying routing rules %p\n", mdfyRules);
		return false;
	}

	for(cnt=0; cnt<mdfyRules->num_rules; cnt++)
	{
		if(mdfyRules->rules[cnt].status != 0)
		{
			IPACMERR("Unable to modify rule: %d\n", cnt);
		}
	}

	IPACMDBG_H("Modified routing rules %p\n", mdfyRules);
	return true;
}
