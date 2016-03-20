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
	ipa_nat_test000.c

	@brief
	Verify the following scenario:
	1. Add ipv4 table
	2. Delete ipv4 table
*/
/*===========================================================================*/

#include "ipa_nat_test.h"
#include "ipa_nat_drv.h"

int ipa_nat_test000(int total_entries, u32 tbl_hdl, u8 sep)
{

	int ret;
	u32 pub_ip_add = 0x011617c0;   /* "192.23.22.1" */

	ret = ipa_nat_add_ipv4_tbl(pub_ip_add, total_entries, &tbl_hdl);
	if (0 != ret)
	{
		IPAERR("unable to create ipv4 nat table and returning Error:%d\n", ret);
		return -1;
	}
	IPADBG("create nat ipv4 table successfully() \n");

	IPADBG("calling ipa_nat_del_ipv4_tbl() \n");
	ret = ipa_nat_del_ipv4_tbl(tbl_hdl);
	if (0 != ret)
	{
		IPAERR("Unable to delete ipv4 nat table %d\n", ret);
		return -1;
	}
	IPADBG("deleted ipv4 nat table successfully. Test passed \n");

	return 0;
}
