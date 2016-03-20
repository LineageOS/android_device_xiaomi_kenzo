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

#include "string.h"  /* memset */
#include "stdlib.h"  /* free, malloc */
#include "stdint.h"  /* uint32_t */

/**
 * struct ipa_nat_ipv4_rule - To hold ipv4 nat rule
 * @target_ip: destination ip address
 * @private_ip: private ip address
 * @target_port: destination port
 * @private_port: private port
 * @protocol: protocol of rule (tcp/udp)
 */
typedef struct {
	uint32_t target_ip;
	uint32_t private_ip;
	uint16_t target_port;
	uint16_t private_port;
	uint16_t public_port;
	uint8_t  protocol;
} ipa_nat_ipv4_rule;

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
				uint32_t *table_handle);

/**
 * ipa_nat_del_ipv4_tbl() - delete ipv4 table
 * @table_handle: [in] Handle of ipv4 nat table
 *
 * To delete given ipv4 nat table
 *
 * Returns:	0  On Success, negative on failure
 */
int ipa_nat_del_ipv4_tbl(uint32_t table_handle);

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
int ipa_nat_add_ipv4_rule(uint32_t table_handle,
				const ipa_nat_ipv4_rule * rule,
				uint32_t *rule_handle);

/**
 * ipa_nat_del_ipv4_rule() - to delete ipv4 nat rule
 * @table_handle: [in] handle of ipv4 nat table
 * @rule_handle: [in] ipv4 nat rule handle
 *
 * To insert new ipv4 nat rule into ipv4 nat table
 *
 * Returns:	0  On Success, negative on failure
 */
int ipa_nat_del_ipv4_rule(uint32_t table_handle,
				uint32_t rule_handle);


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
int ipa_nat_query_timestamp(uint32_t  table_handle,
				uint32_t  rule_handle,
				uint32_t  *time_stamp);

