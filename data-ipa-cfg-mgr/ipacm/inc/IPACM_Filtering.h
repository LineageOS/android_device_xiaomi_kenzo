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
	IPACM_Filtering.h

	@brief
	This file implements the IPACM filtering definitions

	@Author
	Skylar Chang

*/

#ifndef IPACM_FILTERING_H
#define IPACM_FILTERING_H

#include <stdint.h>
#include <linux/msm_ipa.h>
#include <IPACM_Defs.h>
#include <linux/rmnet_ipa_fd_ioctl.h>

class IPACM_Filtering
{
public:
	IPACM_Filtering();
	~IPACM_Filtering();
	bool AddFilteringRule(struct ipa_ioc_add_flt_rule const *ruleTable);
	bool AddFilteringRuleAfter(struct ipa_ioc_add_flt_rule_after const *ruleTable);
	bool DeleteFilteringRule(struct ipa_ioc_del_flt_rule *ruleTable);
	bool Commit(enum ipa_ip_type ip);
	bool Reset(enum ipa_ip_type ip);
	bool DeviceNodeIsOpened();
	bool DeleteFilteringHdls(uint32_t *flt_rule_hdls,
													 ipa_ip_type ip,
													 uint8_t num_rules);

	bool AddWanDLFilteringRule(struct ipa_ioc_add_flt_rule const *rule_table_v4, struct ipa_ioc_add_flt_rule const * rule_table_v6, uint8_t mux_id);
	bool SendFilteringRuleIndex(struct ipa_fltr_installed_notif_req_msg_v01* table);
	bool ModifyFilteringRule(struct ipa_ioc_mdfy_flt_rule* ruleTable);
	ipa_filter_action_enum_v01 GetQmiFilterAction(ipa_flt_action action);

private:
	static const char *DEVICE_NAME;
	int fd; /* File descriptor of the IPA device node /dev/ipa */
};

#endif //IPACM_FILTERING_H

