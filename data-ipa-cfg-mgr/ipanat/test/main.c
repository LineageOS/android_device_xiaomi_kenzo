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


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "ipa_nat_drv.h"
#include "ipa_nat_drvi.h"
#include "ipa_nat_test.h"

extern struct ipa_nat_cache ipv4_nat_cache;

int chk_for_loop(u32 tbl_hdl)
{
	struct ipa_nat_rule *tbl_ptr;
	struct ipa_nat_indx_tbl_rule *indx_tbl_ptr;
	int cnt;
	uint16_t cur_entry;

	if (IPA_NAT_INVALID_NAT_ENTRY == tbl_hdl ||
			tbl_hdl > IPA_NAT_MAX_IP4_TBLS) {
		IPAERR("invalid table handle passed \n");
		return -EINVAL;
	}

	IPADBG("checking ipv4 rules:\n");
	tbl_ptr = (struct ipa_nat_rule *)
			ipv4_nat_cache.ip4_tbl[tbl_hdl-1].ipv4_rules_addr;
	for (cnt = 0;
		cnt < ipv4_nat_cache.ip4_tbl[tbl_hdl - 1].table_entries;
		cnt++) {
		if (Read16BitFieldValue(tbl_ptr[cnt].ip_cksm_enbl,ENABLE_FIELD)) {
			if(Read16BitFieldValue(tbl_ptr[cnt].nxt_indx_pub_port,
							NEXT_INDEX_FIELD) == cnt)
			{
				IPAERR("Infinite loop detected, entry\n");
				ipa_nati_print_rule(&tbl_ptr[cnt], cnt);
				return -EINVAL;
			}
		}
	}

	/* Print ipv4 expansion rules */
	IPADBG("checking ipv4 active expansion rules:\n");
	tbl_ptr = (struct ipa_nat_rule *)
			ipv4_nat_cache.ip4_tbl[tbl_hdl-1].ipv4_expn_rules_addr;
	for (cnt = 0;
		cnt <= ipv4_nat_cache.ip4_tbl[tbl_hdl - 1].expn_table_entries;
		cnt++) {
		if (Read16BitFieldValue(tbl_ptr[cnt].ip_cksm_enbl,
								ENABLE_FIELD)) {
			cur_entry =
				cnt + ipv4_nat_cache.ip4_tbl[tbl_hdl - 1].table_entries;
			if (Read16BitFieldValue(tbl_ptr[cnt].nxt_indx_pub_port,
							NEXT_INDEX_FIELD) == cur_entry)
			{
				IPAERR("Infinite loop detected\n");
				ipa_nati_print_rule(&tbl_ptr[cnt],
					(cnt + ipv4_nat_cache.ip4_tbl[tbl_hdl - 1].table_entries));
				return -EINVAL;
			}
		}
	}

	/* Print ipv4 index rules */
	IPADBG("checking ipv4 index active rules: \n");
	indx_tbl_ptr = (struct ipa_nat_indx_tbl_rule *)
			ipv4_nat_cache.ip4_tbl[tbl_hdl-1].index_table_addr;
	for (cnt = 0;
		 cnt < ipv4_nat_cache.ip4_tbl[tbl_hdl - 1].table_entries;
			 cnt++) {
		if (Read16BitFieldValue(indx_tbl_ptr[cnt].tbl_entry_nxt_indx,
							INDX_TBL_TBL_ENTRY_FIELD)) {
			if (Read16BitFieldValue(indx_tbl_ptr[cnt].tbl_entry_nxt_indx,
							INDX_TBL_NEXT_INDEX_FILED) == cnt)
			{
				IPAERR("Infinite loop detected\n");
				ipa_nati_print_index_rule(&indx_tbl_ptr[cnt], cnt, 0);
				return -EINVAL;
			}
		}
	}

	/* Print ipv4 index expansion rules */
	IPADBG("Checking ipv4 index expansion active rules: \n");
	indx_tbl_ptr = (struct ipa_nat_indx_tbl_rule *)
			ipv4_nat_cache.ip4_tbl[tbl_hdl-1].index_table_expn_addr;
	for (cnt = 0;
		cnt <= ipv4_nat_cache.ip4_tbl[tbl_hdl - 1].expn_table_entries;
			 cnt++) {
		if (Read16BitFieldValue(indx_tbl_ptr[cnt].tbl_entry_nxt_indx,
							INDX_TBL_TBL_ENTRY_FIELD)) {
			cur_entry =
				cnt + ipv4_nat_cache.ip4_tbl[tbl_hdl - 1].table_entries;
			if (Read16BitFieldValue(indx_tbl_ptr[cnt].tbl_entry_nxt_indx,
							INDX_TBL_NEXT_INDEX_FILED) == cur_entry)
			{
				IPAERR("Infinite loop detected\n");
				ipa_nati_print_index_rule(&indx_tbl_ptr[cnt],
					(cnt + ipv4_nat_cache.ip4_tbl[tbl_hdl - 1].table_entries),
				ipv4_nat_cache.ip4_tbl[tbl_hdl - 1].index_expn_table_meta[cnt].prev_index);
				return -EINVAL;
			}
		}
	}
	return 0;
}

uint8_t is_base_entry_valid(u32 tbl_hdl, u16 entry)
{
	struct ipa_nat_rule *tbl_ptr;

	if (entry >
		ipv4_nat_cache.ip4_tbl[tbl_hdl - 1].table_entries)
	{
		tbl_ptr = (struct ipa_nat_rule *)
				ipv4_nat_cache.ip4_tbl[tbl_hdl-1].ipv4_expn_rules_addr;
		entry -=
			ipv4_nat_cache.ip4_tbl[tbl_hdl - 1].table_entries;
	}
	else
	{
		tbl_ptr = (struct ipa_nat_rule *)
				ipv4_nat_cache.ip4_tbl[tbl_hdl-1].ipv4_rules_addr;
	}
	return (Read16BitFieldValue(tbl_ptr[entry].ip_cksm_enbl,
							ENABLE_FIELD));
}

uint8_t is_index_entry_valid(u32 tbl_hdl, u16 entry)
{
	struct ipa_nat_indx_tbl_rule *tbl_ptr;

	if (entry >
		ipv4_nat_cache.ip4_tbl[tbl_hdl - 1].table_entries)
	{
		tbl_ptr = (struct ipa_nat_indx_tbl_rule *)
				ipv4_nat_cache.ip4_tbl[tbl_hdl-1].index_table_expn_addr;
		entry -=
			ipv4_nat_cache.ip4_tbl[tbl_hdl - 1].table_entries;
	}
	else
	{
		tbl_ptr = (struct ipa_nat_indx_tbl_rule *)
				ipv4_nat_cache.ip4_tbl[tbl_hdl-1].index_table_addr;
	}
	if (Read16BitFieldValue(tbl_ptr[entry].tbl_entry_nxt_indx,
						INDX_TBL_TBL_ENTRY_FIELD)) {
		return 1;
	}
	else
	{
		return 0;
	}
}

int chk_for_validity(u32 tbl_hdl)
{
	struct ipa_nat_rule *tbl_ptr;
	struct ipa_nat_indx_tbl_rule *indx_tbl_ptr;
	uint16_t nxt_index, prv_index;
	int cnt;

	if (IPA_NAT_INVALID_NAT_ENTRY == tbl_hdl ||
			tbl_hdl > IPA_NAT_MAX_IP4_TBLS) {
		IPAERR("invalid table handle passed \n");
		return -EINVAL;
	}

	/* Validate base table next_indx and prev_indx values */
	IPADBG("Validating ipv4 active rules: \n");
	tbl_ptr = (struct ipa_nat_rule *)
			ipv4_nat_cache.ip4_tbl[tbl_hdl-1].ipv4_rules_addr;
	for (cnt = 0;
		cnt < ipv4_nat_cache.ip4_tbl[tbl_hdl - 1].table_entries;
			 cnt++) {
		if (Read16BitFieldValue(tbl_ptr[cnt].ip_cksm_enbl,
						ENABLE_FIELD)) {
			nxt_index =
			Read16BitFieldValue(tbl_ptr[cnt].nxt_indx_pub_port,
						NEXT_INDEX_FIELD);
			if (!is_base_entry_valid(tbl_hdl, nxt_index)) {
				IPAERR("Invalid next index found, entry:%d\n", cnt);
			}
		}
	}

	IPADBG("Validating ipv4 expansion active rules: \n");
	tbl_ptr = (struct ipa_nat_rule *)
			ipv4_nat_cache.ip4_tbl[tbl_hdl-1].ipv4_expn_rules_addr;
	for (cnt = 0;
		cnt <= ipv4_nat_cache.ip4_tbl[tbl_hdl - 1].expn_table_entries;
			 cnt++) {
		if (Read16BitFieldValue(tbl_ptr[cnt].ip_cksm_enbl,
							ENABLE_FIELD)) {
			/* Validate next index */
			nxt_index =
				Read16BitFieldValue(tbl_ptr[cnt].nxt_indx_pub_port,
									NEXT_INDEX_FIELD);
			if (!is_base_entry_valid(tbl_hdl, nxt_index)) {
				IPAERR("Invalid next index found, entry:%d\n", cnt);
			}
			/* Validate previous index */
			prv_index =
				Read16BitFieldValue(tbl_ptr[cnt].sw_spec_params,
						SW_SPEC_PARAM_PREV_INDEX_FIELD);
			if (!is_base_entry_valid(tbl_hdl, prv_index)) {
				IPAERR("Invalid Previous index found, entry:%d\n", cnt);
			}
		}
	}

	IPADBG("Validating ipv4 index active rules: \n");
	indx_tbl_ptr = (struct ipa_nat_indx_tbl_rule *)
				ipv4_nat_cache.ip4_tbl[tbl_hdl-1].index_table_addr;
	for (cnt = 0;
		cnt < ipv4_nat_cache.ip4_tbl[tbl_hdl - 1].table_entries;
			 cnt++) {
		if (Read16BitFieldValue(indx_tbl_ptr[cnt].tbl_entry_nxt_indx,
							INDX_TBL_TBL_ENTRY_FIELD)) {
			nxt_index =
				Read16BitFieldValue(indx_tbl_ptr[cnt].tbl_entry_nxt_indx,
							INDX_TBL_NEXT_INDEX_FILED);
			if (!is_index_entry_valid(tbl_hdl, nxt_index)) {
				IPAERR("Invalid next index found, entry:%d\n", cnt);
			}
		}
	}

	IPADBG("Validating ipv4 index expansion active rules: \n");
	indx_tbl_ptr = (struct ipa_nat_indx_tbl_rule *)
	ipv4_nat_cache.ip4_tbl[tbl_hdl-1].index_table_expn_addr;
	for (cnt = 0;
		cnt <= ipv4_nat_cache.ip4_tbl[tbl_hdl - 1].expn_table_entries;
			 cnt++) {
		if (Read16BitFieldValue(indx_tbl_ptr[cnt].tbl_entry_nxt_indx,
								INDX_TBL_TBL_ENTRY_FIELD)) {
			/* Validate next index*/
			nxt_index =
				Read16BitFieldValue(indx_tbl_ptr[cnt].tbl_entry_nxt_indx,
								INDX_TBL_NEXT_INDEX_FILED);
			if (!is_index_entry_valid(tbl_hdl, nxt_index)) {
				IPAERR("Invalid next index found, entry:%d\n", cnt);
			}

			/* Validate previous index*/
			prv_index =
				ipv4_nat_cache.ip4_tbl[tbl_hdl-1].index_expn_table_meta[cnt].prev_index;

			if (!is_index_entry_valid(tbl_hdl, prv_index)) {
				IPAERR("Invalid Previous index found, entry:%d\n", cnt);
			}
		}
	}

	return 0;
}

int ipa_nat_validate_ipv4_table(u32 tbl_hdl)
{
	int ret = 0;

	ret = chk_for_loop(tbl_hdl);
	if (ret)
		return ret;
	ret = chk_for_validity(tbl_hdl);

	return ret;
}

int main(int argc, char* argv[])
{
	int exec = 0, pass = 0, ret;
	int cnt, nt=1;
	int total_entries = 100;
	u8 sep = 0;
	u32 tbl_hdl = 0;
	u32 pub_ip_add = 0x011617c0;   /* "192.23.22.1" */

	IPADBG("ipa_nat_testing user space nat driver\n");

	if (argc == 4)
	{
		if (!strncmp(argv[1], "reg", 3))
		{
			nt = atoi(argv[2]);
			total_entries = atoi(argv[3]);
			IPADBG("Reg: %d, Nat Entries: %d\n", nt, total_entries);
		}
		else if (!strncmp(argv[1], "sep", 3))
		{
			sep = 1;
			nt = atoi(argv[2]);
			total_entries = atoi(argv[3]);
		}
	}
	else if (argc == 3)
	{
		if (!strncmp(argv[1], "inotify", 7))
		{
			ipa_nat_test021(total_entries, atoi(argv[2]));
			return 0;
		}
		else if (!strncmp(argv[1], "sep", 3))
		{
			sep = 1;
			total_entries = atoi(argv[2]);
		}
	}
	else if (argc == 2)
	{
		total_entries = atoi(argv[1]);
		IPADBG("Nat Entries: %d\n", total_entries);
	}


	for (cnt=0; cnt<nt; cnt++)
	{
		IPADBG("%s():Executing %d time \n",__FUNCTION__, cnt);

		if (!sep)
		{
			ret = ipa_nat_add_ipv4_tbl(pub_ip_add, total_entries, &tbl_hdl);
			CHECK_ERR(ret);
		}

		if (sep)
		{
			IPADBG("\n\nExecuting ipa_nat_test00%d\n", exec);
			ret = ipa_nat_test000(total_entries, tbl_hdl, sep);
			if (!ret)
			{
				pass++;
			}
			else
			{
				IPAERR("ipa_nat_test00%d Fail\n", exec);
			}
			exec++;

			IPADBG("\n\nExecuting ipa_nat_test00%d\n", exec);
			ret = ipa_nat_test001(total_entries, tbl_hdl, sep);
			if (!ret)
			{
				pass++;
			}
			else
			{
				IPAERR("ipa_nat_test00%d Fail\n", exec);
			}
			exec++;
		}

		IPADBG("\n\nExecuting ipa_nat_test00%d\n", exec);
		ret = ipa_nat_test002(total_entries, tbl_hdl, sep);
		if (!ret)
		{
			pass++;
		}
		else
		{
			IPAERR("ipa_nat_test00%d Fail\n", exec);
		}
		exec++;

		if (sep)
		{
			IPADBG("\n\nExecuting ipa_nat_test00%d\n", exec);
			ret = ipa_nat_test003(total_entries, tbl_hdl, sep);
			if (!ret)
			{
				pass++;
			}
			else
			{
				IPAERR("ipa_nat_test00%d Fail\n", exec);
			}
			exec++;

			IPADBG("\n\nExecuting ipa_nat_test00%d\n", exec);
			ret = ipa_nat_test004(total_entries, tbl_hdl, sep);
			if (!ret)
			{
				pass++;
			}
			else
			{
				IPAERR("ipa_nat_test00%d Fail\n", exec);
			}
			exec++;

			IPADBG("\n\nExecuting ipa_nat_test00%d\n", exec);
			ret = ipa_nat_test005(total_entries, tbl_hdl, sep);
			if (!ret)
			{
				pass++;
			}
			else
			{
				IPAERR("ipa_nat_test00%d Fail\n", exec);
			}
			exec++;
		}

		IPADBG("\n\nExecuting ipa_nat_test00%d\n", exec);
		ret = ipa_nat_test006(total_entries, tbl_hdl, sep);
		if (!ret)
		{
			pass++;
		}
		else
		{
			IPAERR("ipa_nat_test00%d Fail\n", exec);
		}
		exec++;

		IPADBG("\n\nExecuting ipa_nat_test00%d\n", exec);
		ret = ipa_nat_test007(total_entries, tbl_hdl, sep);
		if (!ret)
		{
			pass++;
		}
		else
		{
			IPAERR("ipa_nat_test00%d Fail\n", exec);
		}
		exec++;

		IPADBG("\n\nExecuting ipa_nat_test00%d\n", exec);
		ret = ipa_nat_test008(total_entries, tbl_hdl, sep);
		if (!ret)
		{
			pass++;
		}
		else
		{
			IPAERR("ipa_nat_test00%d Fail\n", exec);
		}
		exec++;

		IPADBG("\n\nExecuting ipa_nat_test00%d\n", exec);
		ret = ipa_nat_test009(total_entries, tbl_hdl, sep);
		if (!ret)
		{
			pass++;
		}
		else
		{
			IPAERR("ipa_nat_test00%d Fail\n", exec);
		}
		exec++;

		if (total_entries >= IPA_NAT_TEST_PRE_COND_TE)
		{
			IPADBG("\n\nExecuting ipa_nat_test0%d\n", exec);
			ret = ipa_nat_test010(total_entries, tbl_hdl, sep);
			if (!ret)
			{
				pass++;
			}
			else
			{
				IPAERR("ipa_nat_test0%d Fail\n", exec);
			}
			exec++;

			IPADBG("\n\nExecuting ipa_nat_test0%d\n", exec);
			ret = ipa_nat_test011(total_entries, tbl_hdl, sep);
			if (!ret)
			{
				pass++;
			}
			else
			{
				IPAERR("ipa_nat_test0%d Fail\n", exec);
			}
			exec++;

			IPADBG("\n\nExecuting ipa_nat_test0%d\n", exec);
			ret = ipa_nat_test012(total_entries, tbl_hdl, sep);
			if (!ret)
			{
				pass++;
			}
			else
			{
				IPAERR("ipa_nat_test0%d Fail\n", exec);
			}
			exec++;

			IPADBG("\n\nExecuting ipa_nat_test0%d\n", exec);
			ret = ipa_nat_test013(total_entries, tbl_hdl, sep);
			if (!ret)
			{
				pass++;
			}
			else
			{
				IPAERR("ipa_nat_test0%d Fail\n", exec);
			}
			exec++;

			IPADBG("\n\nExecuting ipa_nat_test0%d\n", exec);
			ret = ipa_nat_test014(total_entries, tbl_hdl, sep);
			if (!ret)
			{
				pass++;
			}
			else
			{
				IPAERR("ipa_nat_test0%d Fail\n", exec);
			}
			exec++;

			IPADBG("\n\nExecuting ipa_nat_test0%d\n", exec);
			ret = ipa_nat_test015(total_entries, tbl_hdl, sep);
			if (!ret)
			{
				pass++;
			}
			else
			{
				IPAERR("ipa_nat_test0%d Fail\n", exec);
			}
			exec++;

			IPADBG("\n\nExecuting ipa_nat_test0%d\n", exec);
			ret = ipa_nat_test016(total_entries, tbl_hdl, sep);
			if (!ret)
			{
				pass++;
			}
			else
			{
				IPAERR("ipa_nat_test0%d Fail\n", exec);
			}
			exec++;

			IPADBG("\n\nExecuting ipa_nat_test0%d\n", exec);
			ret = ipa_nat_test017(total_entries, tbl_hdl, sep);
			if (!ret)
			{
				pass++;
			}
			else
			{
				IPAERR("ipa_nat_test0%d Fail\n", exec);
			}
			exec++;

			IPADBG("\n\nExecuting ipa_nat_test0%d\n", exec);
			ret = ipa_nat_test018(total_entries, tbl_hdl, sep);
			if (!ret)
			{
				pass++;
			}
			else
			{
				IPAERR("ipa_nat_test0%d Fail\n", exec);
			}
			exec++;

			IPADBG("\n\nExecuting ipa_nat_test0%d\n", exec);
			ret = ipa_nat_test019(total_entries, tbl_hdl, sep);
			if (!ret)
			{
				pass++;
			}
			else
			{
				IPAERR("ipa_nat_test0%d Fail\n", exec);
			}
			exec++;

			IPADBG("\n\nExecuting ipa_nat_test0%d\n", exec);
			ret = ipa_nat_test020(total_entries, tbl_hdl, sep);
			if (!ret)
			{
				pass++;
			}
			else
			{
				IPAERR("ipa_nat_test0%d Fail\n", exec);
			}
			exec++;

			IPADBG("\n\nExecuting ipa_nat_test0%d\n", exec);
			ret = ipa_nat_test022(total_entries, tbl_hdl, sep);
			if (!ret)
			{
				pass++;
			}
			else
			{
				IPAERR("ipa_nat_test0%d Fail\n", exec);
			}
			exec++;
		}

		if (!sep)
		{
			ret = ipa_nat_del_ipv4_tbl(tbl_hdl);
			CHECK_ERR(ret);
		}
	}
	/*=======  Printing Results ==========*/
	IPADBG("Total ipa_nat Tests Run:%d, Pass:%d, Fail:%d\n",exec, pass, exec-pass);
	return 0;
}
