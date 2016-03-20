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
	IPACM_Filtering.cpp

	@brief
	This file implements the IPACM filtering functionality.

	@Author
	Skylar Chang

*/
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>

#include "IPACM_Filtering.h"
#include <IPACM_Log.h>
#include "IPACM_Defs.h"


const char *IPACM_Filtering::DEVICE_NAME = "/dev/ipa";

IPACM_Filtering::IPACM_Filtering()
{
	fd = open(DEVICE_NAME, O_RDWR);
	if (fd < 0)
	{
		IPACMERR("Failed opening %s.\n", DEVICE_NAME);
	}
}

IPACM_Filtering::~IPACM_Filtering()
{
	close(fd);
}

bool IPACM_Filtering::DeviceNodeIsOpened()
{
	return fd;
}

bool IPACM_Filtering::AddFilteringRule(struct ipa_ioc_add_flt_rule const *ruleTable)
{
	int retval = 0;

	IPACMDBG("Printing filter add attributes\n");
	IPACMDBG("ip type: %d\n", ruleTable->ip);
	IPACMDBG("Number of rules: %d\n", ruleTable->num_rules);
	IPACMDBG("End point: %d and global value: %d\n", ruleTable->ep, ruleTable->global);
	IPACMDBG("commit value: %d\n", ruleTable->commit);
	for (int cnt=0; cnt<ruleTable->num_rules; cnt++)
	{
		IPACMDBG("Filter rule:%d attrib mask: 0x%x\n",
						 cnt, 
						 ruleTable->rules[cnt].rule.attrib.attrib_mask);
	}

	retval = ioctl(fd, IPA_IOC_ADD_FLT_RULE, ruleTable);
	if (retval != 0)
	{
		IPACMERR("Failed adding Filtering rule %p\n", ruleTable);
		PERROR("unable to add filter rule:");

		for (int cnt = 0; cnt < ruleTable->num_rules; cnt++)
		{
			if (ruleTable->rules[cnt].status != 0)
			{
				IPACMERR("Adding Filter rule:%d failed with status:%d\n",
								 cnt, ruleTable->rules[cnt].status);
			}
		}
		return false;
	}

	for (int cnt = 0; cnt<ruleTable->num_rules; cnt++)
	{
		if(ruleTable->rules[cnt].status != 0)
		{
			IPACMERR("Adding Filter rule:%d failed with status:%d\n",
							 cnt, ruleTable->rules[cnt].status);
		}
	}

	IPACMDBG("Added Filtering rule %p\n", ruleTable);
	return true;
}

bool IPACM_Filtering::DeleteFilteringRule(struct ipa_ioc_del_flt_rule *ruleTable)
{
	int retval = 0;

	retval = ioctl(fd, IPA_IOC_DEL_FLT_RULE, ruleTable);
	if (retval != 0)
	{
		IPACMERR("Failed deleting Filtering rule %p\n", ruleTable);
		return false;
	}

	IPACMDBG("Deleted Filtering rule %p\n", ruleTable);
	return true;
}

bool IPACM_Filtering::Commit(enum ipa_ip_type ip)
{
	int retval = 0;

	retval = ioctl(fd, IPA_IOC_COMMIT_FLT, ip);
	if (retval != 0)
	{
		IPACMERR("failed committing Filtering rules.\n");
		return false;
	}

	IPACMDBG("Committed Filtering rules to IPA HW.\n");
	return true;
}

bool IPACM_Filtering::Reset(enum ipa_ip_type ip)
{
	int retval = 0;

	retval = ioctl(fd, IPA_IOC_RESET_FLT, ip);
	retval |= ioctl(fd, IPA_IOC_COMMIT_FLT, ip);
	if (retval)
	{
		IPACMERR("failed resetting Filtering block.\n");
		return false;
	}

	IPACMDBG("Reset command issued to IPA Filtering block.\n");
	return true;
}

bool IPACM_Filtering::DeleteFilteringHdls
(
	 uint32_t *flt_rule_hdls,
	 ipa_ip_type ip,
	 uint8_t num_rules
)
{
	struct ipa_ioc_del_flt_rule *flt_rule;
	bool res = true;
	int len = 0, cnt = 0;
        const uint8_t UNIT_RULES = 1;

	len = (sizeof(struct ipa_ioc_del_flt_rule)) + (UNIT_RULES * sizeof(struct ipa_flt_rule_del));
	flt_rule = (struct ipa_ioc_del_flt_rule *)malloc(len);
	if (flt_rule == NULL)
	{
		IPACMERR("unable to allocate memory for del filter rule\n");
		return false;
	}

	for (cnt = 0; cnt < num_rules; cnt++)
	{
	    memset(flt_rule, 0, len);
	    flt_rule->commit = 1;
	    flt_rule->num_hdls = UNIT_RULES;
	    flt_rule->ip = ip;

	    if (flt_rule_hdls[cnt] == 0)
	    {
		   IPACMERR("invalid filter handle passed, ignoring it: %d\n", cnt)
	    }
            else
	    {

		   flt_rule->hdl[0].status = -1;
		   flt_rule->hdl[0].hdl = flt_rule_hdls[cnt];
		   IPACMDBG("Deleting filter hdl:(0x%x) with ip type: %d\n", flt_rule_hdls[cnt], ip);

	           if (DeleteFilteringRule(flt_rule) == false)
	           {
		        PERROR("Filter rule deletion failed!\n");
		        res = false;
		        goto fail;
	           }
		   else
	           {

		        if (flt_rule->hdl[0].status != 0)
		        {
			     IPACMERR("Filter rule hdl 0x%x deletion failed with error:%d\n",
		        					 flt_rule->hdl[0].hdl, flt_rule->hdl[0].status);
			     res = false;
			     goto fail;
		        }
		   
		   }	   
	    }
	}

fail:
	free(flt_rule);

	return res;
}

bool IPACM_Filtering::AddWanDLFilteringRule(struct ipa_ioc_add_flt_rule const *rule_table_v4, struct ipa_ioc_add_flt_rule const * rule_table_v6, uint8_t mux_id)
{
	int ret = 0, cnt, num_rules = 0, pos = 0;
	ipa_install_fltr_rule_req_msg_v01 qmi_rule_msg;

	int fd_wwan_ioctl = open(WWAN_QMI_IOCTL_DEVICE_NAME, O_RDWR);
	if(fd_wwan_ioctl < 0)
	{
		IPACMERR("Failed to open %s.\n",WWAN_QMI_IOCTL_DEVICE_NAME);
		return false;
	}

	if(rule_table_v4 != NULL)
	{
		num_rules += rule_table_v4->num_rules;
		IPACMDBG_H("Get %d WAN DL IPv4 filtering rules.\n", rule_table_v4->num_rules);
	}
	if(rule_table_v6 != NULL)
	{
		num_rules += rule_table_v6->num_rules;
		IPACMDBG_H("Get %d WAN DL IPv6 filtering rules.\n", rule_table_v6->num_rules);
	}

	if(num_rules > QMI_IPA_MAX_FILTERS_V01)
	{
		IPACMERR("The number of filtering rules exceed limit.\n");
		close(fd_wwan_ioctl);
		return false;
	}
	else
	{
		memset(&qmi_rule_msg, 0, sizeof(qmi_rule_msg));

		if (num_rules > 0)
		{
			qmi_rule_msg.filter_spec_list_valid = true;
		}
		else
		{
			qmi_rule_msg.filter_spec_list_valid = false;
		}
		qmi_rule_msg.filter_spec_list_len = num_rules;
		qmi_rule_msg.source_pipe_index_valid = 0;

		IPACMDBG_H("Get %d WAN DL filtering rules in total.\n", num_rules);
		
		if(rule_table_v4 != NULL)
		{
			for(cnt = rule_table_v4->num_rules - 1; cnt >= 0; cnt--)
			{
				if (pos < QMI_IPA_MAX_FILTERS_V01)
				{
				qmi_rule_msg.filter_spec_list[pos].filter_spec_identifier = pos;
				qmi_rule_msg.filter_spec_list[pos].ip_type = QMI_IPA_IP_TYPE_V4_V01;
				qmi_rule_msg.filter_spec_list[pos].filter_action = GetQmiFilterAction(rule_table_v4->rules[cnt].rule.action);
				qmi_rule_msg.filter_spec_list[pos].is_routing_table_index_valid = 1;
				qmi_rule_msg.filter_spec_list[pos].route_table_index = rule_table_v4->rules[cnt].rule.rt_tbl_idx;
				qmi_rule_msg.filter_spec_list[pos].is_mux_id_valid = 1;
				qmi_rule_msg.filter_spec_list[pos].mux_id = mux_id;

				memcpy(&qmi_rule_msg.filter_spec_list[pos].filter_rule, &rule_table_v4->rules[cnt].rule.eq_attrib, 
					sizeof(struct ipa_filter_rule_type_v01));
				pos++;
			}
				else
				{
					IPACMERR(" QMI only support max %d rules, current (%d)\n ",QMI_IPA_MAX_FILTERS_V01, pos);
				}
			}
		}

		if(rule_table_v6 != NULL)
		{
			for(cnt = rule_table_v6->num_rules - 1; cnt >= 0; cnt--)
			{
				if (pos < QMI_IPA_MAX_FILTERS_V01)
				{
				qmi_rule_msg.filter_spec_list[pos].filter_spec_identifier = pos;
				qmi_rule_msg.filter_spec_list[pos].ip_type = QMI_IPA_IP_TYPE_V6_V01;
				qmi_rule_msg.filter_spec_list[pos].filter_action = GetQmiFilterAction(rule_table_v6->rules[cnt].rule.action);
				qmi_rule_msg.filter_spec_list[pos].is_routing_table_index_valid = 1;
				qmi_rule_msg.filter_spec_list[pos].route_table_index = rule_table_v6->rules[cnt].rule.rt_tbl_idx;
				qmi_rule_msg.filter_spec_list[pos].is_mux_id_valid = 1;
				qmi_rule_msg.filter_spec_list[pos].mux_id = mux_id;

				memcpy(&qmi_rule_msg.filter_spec_list[pos].filter_rule, &rule_table_v6->rules[cnt].rule.eq_attrib, 
					sizeof(struct ipa_filter_rule_type_v01));
				pos++;
			}
				else
				{
					IPACMERR(" QMI only support max %d rules, current (%d)\n ",QMI_IPA_MAX_FILTERS_V01, pos);
				}
			}
		}

		ret = ioctl(fd_wwan_ioctl, WAN_IOC_ADD_FLT_RULE, &qmi_rule_msg);
		if (ret != 0)
		{
			IPACMERR("Failed adding Filtering rule %p with ret %d\n ", &qmi_rule_msg, ret);
			close(fd_wwan_ioctl);
			return false;
		}
	}
	IPACMDBG("Added Filtering rule %p\n", &qmi_rule_msg);
	close(fd_wwan_ioctl);
	return true;
}

bool IPACM_Filtering::SendFilteringRuleIndex(struct ipa_fltr_installed_notif_req_msg_v01* table)
{
	int ret = 0;
	int fd_wwan_ioctl = open(WWAN_QMI_IOCTL_DEVICE_NAME, O_RDWR);
	if(fd_wwan_ioctl < 0)
	{
		IPACMERR("Failed to open %s.\n",WWAN_QMI_IOCTL_DEVICE_NAME);
		return false;
	}

	ret = ioctl(fd_wwan_ioctl, WAN_IOC_ADD_FLT_RULE_INDEX, table);
	if (ret != 0)
	{
		IPACMERR("Failed adding filtering rule index %p with ret %d\n", table, ret);
		close(fd_wwan_ioctl);
		return false;
	}

	IPACMDBG("Added Filtering rule index %p\n", table);
	close(fd_wwan_ioctl);
	return true;
}

ipa_filter_action_enum_v01 IPACM_Filtering::GetQmiFilterAction(ipa_flt_action action)
{
	switch(action)
	{
	case IPA_PASS_TO_ROUTING:
		return QMI_IPA_FILTER_ACTION_ROUTING_V01;

	case IPA_PASS_TO_SRC_NAT:
		return QMI_IPA_FILTER_ACTION_SRC_NAT_V01;

	case IPA_PASS_TO_DST_NAT:
		return QMI_IPA_FILTER_ACTION_DST_NAT_V01;

	case IPA_PASS_TO_EXCEPTION:
		return QMI_IPA_FILTER_ACTION_EXCEPTION_V01;

	default:
		return IPA_FILTER_ACTION_ENUM_MAX_ENUM_VAL_V01;
	}
}

bool IPACM_Filtering::ModifyFilteringRule(struct ipa_ioc_mdfy_flt_rule* ruleTable)
{
	int i, ret = 0;

	IPACMDBG("Printing filtering add attributes\n");
	IPACMDBG("IP type: %d Number of rules: %d commit value: %d\n", ruleTable->ip, ruleTable->num_rules, ruleTable->commit);

	for (i=0; i<ruleTable->num_rules; i++)
	{
		IPACMDBG("Filter rule:%d attrib mask: 0x%x\n", i, ruleTable->rules[i].rule.attrib.attrib_mask);
	}

	ret = ioctl(fd, IPA_IOC_MDFY_FLT_RULE, ruleTable);
	if (ret != 0)
	{
		IPACMERR("Failed modifying filtering rule %p\n", ruleTable);

		for (i = 0; i < ruleTable->num_rules; i++)
		{
			if (ruleTable->rules[i].status != 0)
			{
				IPACMERR("Modifying filter rule %d failed\n", i);
			}
		}
		return false;
	}

	IPACMDBG("Modified filtering rule %p\n", ruleTable);
	return true;
}

