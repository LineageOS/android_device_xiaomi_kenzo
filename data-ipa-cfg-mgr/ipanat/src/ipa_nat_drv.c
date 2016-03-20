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

#include "ipa_nat_drv.h"
#include "ipa_nat_drvi.h"

/**
 * ipa_nat_add_ipv4_tbl() - create ipv4 nat table
 * @public_ip_addr: [in] public ipv4 address
 * @number_of_entries: [in]  number of nat entries
 * @table_handle: [out] Handle of new ipv4 nat table
 *
 * To create new ipv4 nat table
 *
 * Returns:	0  On Success, negative on failure
 */
int ipa_nat_add_ipv4_tbl(uint32_t public_ip_addr,
		uint16_t number_of_entries,
		uint32_t *tbl_hdl)
{
  int ret;

  if (NULL == tbl_hdl || 0 == number_of_entries) {
    IPAERR("Invalid parameters \n");
    return -EINVAL;
  }

  ret = ipa_nati_add_ipv4_tbl(public_ip_addr,
								number_of_entries,
								tbl_hdl);
  if (ret != 0) {
    IPAERR("unable to add table \n");
    return -EINVAL;
  }
  IPADBG("Returning table handle 0x%x\n", *tbl_hdl);

  return ret;
} /* __ipa_nat_add_ipv4_tbl() */

/**
 * ipa_nat_del_ipv4_tbl() - delete ipv4 table
 * @table_handle: [in] Handle of ipv4 nat table
 *
 * To delete given ipv4 nat table
 *
 * Returns:	0  On Success, negative on failure
 */
int ipa_nat_del_ipv4_tbl(uint32_t tbl_hdl)
{
  if (IPA_NAT_INVALID_NAT_ENTRY == tbl_hdl ||
      tbl_hdl > IPA_NAT_MAX_IP4_TBLS) {
    IPAERR("invalid table handle passed \n");
    return -EINVAL;
  }
  IPADBG("Passed Table Handle: 0x%x\n", tbl_hdl);

  return ipa_nati_del_ipv4_table(tbl_hdl);
}

/**
 * ipa_nat_add_ipv4_rule() - to insert new ipv4 rule
 * @table_handle: [in] handle of ipv4 nat table
 * @rule: [in]  Pointer to new rule
 * @rule_handle: [out] Return the handle to rule
 *
 * To insert new ipv4 nat rule into ipv4 nat table
 *
 * Returns:	0  On Success, negative on failure
 */
int ipa_nat_add_ipv4_rule(uint32_t tbl_hdl,
		const ipa_nat_ipv4_rule *clnt_rule,
		uint32_t *rule_hdl)
{
  int result = -EINVAL;

  if (IPA_NAT_INVALID_NAT_ENTRY == tbl_hdl ||
      tbl_hdl > IPA_NAT_MAX_IP4_TBLS || NULL == rule_hdl ||
      NULL == clnt_rule) {
    IPAERR("invalide table handle passed \n");
    return result;
  }
  IPADBG("Passed Table handle: 0x%x\n", tbl_hdl);

  if (ipa_nati_add_ipv4_rule(tbl_hdl, clnt_rule, rule_hdl) != 0) {
		return result;
	}

  IPADBG("returning rule handle 0x%x\n", *rule_hdl);
  return 0;
}


/**
 * ipa_nat_del_ipv4_rule() - to delete ipv4 nat rule
 * @table_handle: [in] handle of ipv4 nat table
 * @rule_handle: [in] ipv4 nat rule handle
 *
 * To insert new ipv4 nat rule into ipv4 nat table
 *
 * Returns:	0  On Success, negative on failure
 */
int ipa_nat_del_ipv4_rule(uint32_t tbl_hdl,
		uint32_t rule_hdl)
{
  int result = -EINVAL;

  if (IPA_NAT_INVALID_NAT_ENTRY == tbl_hdl ||
      IPA_NAT_INVALID_NAT_ENTRY == rule_hdl) {
    IPAERR("invalide parameters\n");
    return result;
  }
  IPADBG("Passed Table: 0x%x and rule handle 0x%x\n", tbl_hdl, rule_hdl);

  result = ipa_nati_del_ipv4_rule(tbl_hdl, rule_hdl);
  if (result) {
    IPAERR("unable to delete rule from hw \n");
    return result;
  }

  return 0;
}

/**
 * ipa_nat_query_timestamp() - to query timestamp
 * @table_handle: [in] handle of ipv4 nat table
 * @rule_handle: [in] ipv4 nat rule handle
 * @time_stamp: [out] time stamp of rule
 *
 * To retrieve the timestamp that lastly the
 * nat rule was accessed
 *
 * Returns:	0  On Success, negative on failure
 */
int ipa_nat_query_timestamp(uint32_t  tbl_hdl,
		uint32_t  rule_hdl,
		uint32_t  *time_stamp)
{

  if (0 == tbl_hdl || tbl_hdl > IPA_NAT_MAX_IP4_TBLS ||
      NULL == time_stamp) {
    IPAERR("invalid parameters passed \n");
    return -EINVAL;
  }
  IPADBG("Passed Table: 0x%x and rule handle 0x%x\n", tbl_hdl, rule_hdl);

  return ipa_nati_query_timestamp(tbl_hdl, rule_hdl, time_stamp);
}


