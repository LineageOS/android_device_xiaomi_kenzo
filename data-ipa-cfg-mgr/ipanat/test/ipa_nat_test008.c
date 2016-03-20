/*
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of The Linux Foundation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*=========================================================================*/
/*!
	@file
	ipa_nat_test008.c

	@brief
	Verify the following scenario:
	1. Add ipv4 table
	2. add 2 distinct rules
	3. delete first followed by second
	4. Delete ipv4 table
*/
/*=========================================================================*/

#include "ipa_nat_test.h"
#include "ipa_nat_drv.h"

int ipa_nat_test008(int total_entries, u32 tbl_hdl, u8 sep)
{
	int ret;
	u32 rule_hdl, rule_hdl1;
	ipa_nat_ipv4_rule ipv4_rule, ipv4_rule1;

	u32 pub_ip_add = 0x011617c0;   /* "192.23.22.1" */

	ipv4_rule.target_ip = 0xC1171601; /* 193.23.22.1 */
	ipv4_rule.target_port = 1234;
	ipv4_rule.private_ip = 0xC2171601; /* 194.23.22.1 */
	ipv4_rule.private_port = 5678;
	ipv4_rule.protocol = IPPROTO_TCP;
	ipv4_rule.public_port = 9050;
	ipv4_rule1.target_ip = 0xC1171602; /* 193.23.22.2 */
	ipv4_rule1.target_port = 1234;
	ipv4_rule1.private_ip = 0xC2171602; /* 194.23.22.2 */
	ipv4_rule1.private_port = 5678;
	ipv4_rule1.protocol = IPPROTO_TCP;
	ipv4_rule1.public_port = 9050;

	IPADBG("%s():\n",__FUNCTION__);

	if(sep)
	{
		ret = ipa_nat_add_ipv4_tbl(pub_ip_add, total_entries, &tbl_hdl);
		CHECK_ERR(ret);
	}

	ret = ipa_nat_add_ipv4_rule(tbl_hdl, &ipv4_rule, &rule_hdl);
	CHECK_ERR(ret);

	ret = ipa_nat_add_ipv4_rule(tbl_hdl, &ipv4_rule1, &rule_hdl1);
	CHECK_ERR(ret);

	ret = ipa_nat_del_ipv4_rule(tbl_hdl, rule_hdl);
	CHECK_ERR(ret);

	ret = ipa_nat_del_ipv4_rule(tbl_hdl, rule_hdl1);
	CHECK_ERR(ret);

	if(sep)
	{
		ret = ipa_nat_del_ipv4_tbl(tbl_hdl);
		CHECK_ERR(ret);
	}
	return 0;
}
