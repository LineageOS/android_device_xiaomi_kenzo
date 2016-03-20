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

#ifndef IPA_NAT_DRVI_H
#define IPA_NAT_DRVI_H

#include <unistd.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <linux/msm_ipa.h>
#include <netinet/in.h>
#include <sys/inotify.h>
#include <errno.h>

#include "ipa_nat_logi.h"

#define NAT_DUMP

/*======= IMPLEMENTATION related data structures and functions ======= */
#ifdef IPA_ON_R3PC
#define NAT_MMAP_MEM_SIZE (2 * 1024UL * 1024UL - 1)
#endif

#define IPA_DEV_NAME       "/dev/ipa"
#define NAT_DEV_DIR        "/dev"
#define NAT_DEV_NAME       "ipaNatTable"
#define NAT_DEV_FULL_NAME  "/dev/ipaNatTable"

#define IPA_NAT_TABLE_VALID 1
#define IPA_NAT_MAX_IP4_TBLS   1
#define IPA_NAT_BASE_TABLE_PERCENTAGE       .8
#define IPA_NAT_EXPANSION_TABLE_PERCENTAGE  .2

#define IPA_NAT_NUM_OF_BASE_TABLES      2
#define IPA_NAT_UNUSED_BASE_ENTRIES     2

#define IPA_NAT_RULE_FLAG_FIELD_OFFSET        18
#define IPA_NAT_RULE_NEXT_FIELD_OFFSET        8
#define IPA_NAT_RULE_PROTO_FIELD_OFFSET       22

#define IPA_NAT_INDEX_RULE_NEXT_FIELD_OFFSET       2
#define IPA_NAT_INDEX_RULE_NAT_INDEX_FIELD_OFFSET  0

#define IPA_NAT_RULE_FLAG_FIELD_SIZE       2
#define IPA_NAT_RULE_NEXTFIELD_FIELD_SIZE  2

#define IPA_NAT_FLAG_ENABLE_BIT_MASK  0x8000
#define IPA_NAT_FLAG_DISABLE_BIT_MASK 0x0000

#define IPA_NAT_FLAG_ENABLE_BIT  1
#define IPA_NAT_FLAG_DISABLE_BIT 0

#define IPA_NAT_INVALID_PROTO_FIELD_VALUE 0xFF00
#define IPA_NAT_INVALID_PROTO_FIELD_CMP   0xFF

#define IPA_NAT_INVALID_INDEX 0xFF
#define IPA_NAT_INVALID_NAT_ENTRY 0x0

#define INDX_TBL_ENTRY_SIZE_IN_BITS  16

/* ----------- Rule id -----------------------

   ------------------------------------------------
   |  3bits   |    12 bits       |     1 bit      |
   ------------------------------------------------
   | reserved | index into table |  0 - base      |
   |          |                  |  1 - expansion |
   ------------------------------------------------

*/
#define IPA_NAT_RULE_HDL_TBL_TYPE_BITS        0x1
#define IPA_NAT_RULE_HDL_TBL_TYPE_MASK        0x1

/* ----------- sw specif parameter -----
   ------------------------------------
   |     16 bits     |     16 bits    |
   ------------------------------------
   |  index table    |  prev index    |
   |     entry       |                |
   ------------------------------------
-----------------------------------------*/
#define IPA_NAT_SW_PARAM_PREV_INDX_BYTE       0
#define IPA_NAT_SW_PARAM_INDX_TBL_ENTRY_BYTE  1

typedef enum {
	IPA_NAT_BASE_TBL        = 0,
	IPA_NAT_EXPN_TBL        = 1,
	IPA_NAT_INDX_TBL        = 2,
	IPA_NAT_INDEX_EXPN_TBL  = 3,
} nat_table_type;

typedef enum {
	NEXT_INDEX_FIELD,
	PUBLIC_PORT_FILED,
	PRIVATE_PORT_FIELD,
	TARGET_PORT_FIELD,
	IP_CHKSUM_FIELD,
	ENABLE_FIELD,
	TIME_STAMP_FIELD,
	PROTOCOL_FIELD,
	TCP_UDP_CHKSUM_FIELD,
	SW_SPEC_PARAM_PREV_INDEX_FIELD,
	SW_SPEC_PARAM_INDX_TBL_ENTRY_FIELD,
	INDX_TBL_TBL_ENTRY_FIELD,
	INDX_TBL_NEXT_INDEX_FILED
} ipa_nat_rule_field_type;

/*
	---------------------------------------------
	|     3      |    2    |    1    |    0      |
	---------------------------------------------
	| Public Port(2B)     | Next Index(2B)       |
	---------------------------------------------
*/
typedef struct {
	uint32_t next_index:16;
	uint32_t public_port:16;
} next_index_pub_port;


/*
	---------------------------------------------
	|     3      |    2    |    1    |    0      |
	---------------------------------------------
  |       Flags(2B)     | IP check sum Diff(2B)|
	|EN|FIN|Resv |        |                      |
	---------------------------------------------
*/
typedef struct {
	uint32_t ip_chksum:16;
	uint32_t rsvd1:14;
	uint32_t redirect:1;
	uint32_t enable:1;
} ipcksum_enbl;


/*
	---------------------------------------
	|   7    |    6    |   5    |    4    |
	---------------------------------------
  | Proto   |      TimeStamp(3B)        |
	| (1B)    |                           |
	---------------------------------------
*/
typedef struct {
	uint32_t time_stamp:24;
	uint32_t protocol:8;
} time_stamp_proto;


/*
	---------------------------------------------
	|     3      |    2    |    1    |    0      |
	---------------------------------------------
  |       next_index     | Table entry         |
	----------------------------------------------
*/
typedef struct {
	uint16_t tbl_entry;
	uint16_t next_index;
} tbl_ent_nxt_indx;

/*--------------------------------------------------
   32 bit sw_spec_params is interpreted as follows
   ------------------------------------
   |     16 bits     |     16 bits    |
   ------------------------------------
   |  index table    |  prev index    |
   |     entry       |                |
	 ------------------------------------
--------------------------------------------------*/
typedef struct {
	uint16_t prev_index;
	uint16_t index_table_entry;
} sw_spec_params;

/*------------------------  NAT Table Entry  ---------------------------------------

  -----------------------------------------------------------------------------------
  |   7    |    6    |   5    |    4    |     3      |    2    |    1    |    0      |
  -----------------------------------------------------------------------------------
  |             Target IP(4B)           |             Private IP(4B)                 |
  -----------------------------------------------------------------------------------
  |Target Port(2B)   | Private Port(2B) | Public Port(2B)     | Next Index(2B)       |
  -----------------------------------------------------------------------------------
  | Proto   |      TimeStamp(3B)        |       Flags(2B)     | IP check sum Diff(2B)|
  | (1B)    |                           |EN|FIN|Resv |        |                      |
  -----------------------------------------------------------------------------------
  | TCP/UDP checksum |  Reserved(2B)    |    SW Specific Parameters(4B)              |
  |    diff (2B)                        |                                            |
  -----------------------------------------------------------------------------------

  Dont change below structure definition.
  It should be same as above(little endian order)
  -------------------------------------------------------------------------------*/
struct ipa_nat_rule {
	uint64_t private_ip:32;
	uint64_t target_ip:32;

	uint64_t nxt_indx_pub_port:32;
	uint64_t private_port:16;
	uint64_t target_port:16;

	uint64_t ip_cksm_enbl:32;
	uint64_t ts_proto:32;

  /*--------------------------------------------------
   32 bit sw_spec_params is interpreted as follows
   ------------------------------------
   |     16 bits     |     16 bits    |
   ------------------------------------
   |  index table    |  prev index    |
   |     entry       |                |
	 ------------------------------------
  --------------------------------------------------*/
	uint64_t sw_spec_params:32;

	uint64_t rsvd2:16;
	uint64_t tcp_udp_chksum:16;
};

struct ipa_nat_sw_rule {
	uint64_t private_ip:32;
	uint64_t target_ip:32;

	uint64_t next_index:16;
	uint64_t public_port:16;
	uint64_t private_port:16;
	uint64_t target_port:16;

	uint64_t ip_chksum:16;
	uint64_t rsvd1:14;
	uint64_t redirect:1;
	uint64_t enable:1;
	uint64_t time_stamp:24;
	uint64_t protocol:8;

  /*--------------------------------------------------
   32 bit sw_spec_params is interpreted as follows
   ------------------------------------
   |     16 bits     |     16 bits    |
   ------------------------------------
   |  index table    |  prev index    |
   |     entry       |                |
   ------------------------------------
  --------------------------------------------------*/
	uint64_t prev_index:16;
	uint64_t indx_tbl_entry:16;
	uint64_t rsvd2:16;
	uint64_t tcp_udp_chksum:16;
};
#define IPA_NAT_TABLE_ENTRY_SIZE        32
#define IPA_NAT_INDEX_TABLE_ENTRY_SIZE  4

struct ipa_nat_indx_tbl_rule {
	uint32_t tbl_entry_nxt_indx;
};

struct ipa_nat_sw_indx_tbl_rule {
	uint16_t tbl_entry;
	uint16_t next_index;
};

struct ipa_nat_indx_tbl_meta_info {
	uint16_t prev_index;
};

struct ipa_nat_ip4_table_cache {
	uint8_t valid;
	uint32_t public_addr;

	int nat_fd;
	int size;
	uint32_t tbl_addr_offset;
	char table_name[IPA_RESOURCE_NAME_MAX];

	char  *ipv4_rules_addr;
	char  *index_table_addr;
	uint16_t   table_entries;

	char *ipv4_expn_rules_addr;
	char *index_table_expn_addr;
	uint16_t  expn_table_entries;

	struct ipa_nat_indx_tbl_meta_info *index_expn_table_meta;

	uint16_t *rule_id_array;
#ifdef IPA_ON_R3PC
	uint32_t mmap_offset;
#endif

	uint16_t cur_tbl_cnt;
	uint16_t cur_expn_tbl_cnt;
};

struct ipa_nat_cache {
	struct ipa_nat_ip4_table_cache ip4_tbl[IPA_NAT_MAX_IP4_TBLS];
	int ipa_fd;
	uint8_t table_cnt;
};

struct ipa_nat_indx_tbl_sw_rule {
	uint16_t tbl_entry;
	uint16_t next_index;
	uint16_t prev_index;
};

typedef enum {
	IPA_NAT_DEL_TYPE_ONLY_ONE,
	IPA_NAT_DEL_TYPE_HEAD,
	IPA_NAT_DEL_TYPE_MIDDLE,
	IPA_NAT_DEL_TYPE_LAST,
} del_type;

/**
 * ipa_nati_parse_ipv4_rule_hdl() - prase rule handle
 * @tbl_hdl:	[in] nat table rule
 * @rule_hdl: [in] nat rule handle
 * @expn_tbl: [out] expansion table or not
 * @tbl_entry: [out] index into table
 *
 * Parse the rule handle to retrieve the nat table
 * type and entry of nat table
 *
 * Returns:	None
 */
void ipa_nati_parse_ipv4_rule_hdl(uint8_t tbl_hdl,
				uint16_t rule_hdl,
				uint8_t *expn_tbl,
				uint16_t *tbl_entry);

/**
 * ipa_nati_make_rule_hdl() - makes nat rule handle
 * @tbl_hdl: [in] nat table handle
 * @tbl_entry: [in]  nat table entry
 *
 * Calculate the nat rule handle which from
 * nat entry which will be returned to client of
 * nat driver
 *
 * Returns:	>0 nat rule handle
 */
uint16_t ipa_nati_make_rule_hdl(uint16_t tbl_hdl,
				uint16_t tbl_entry);

uint32_t ipa_nati_get_index_entry_offset(
				struct ipa_nat_ip4_table_cache*,
				nat_table_type tbl_type,
				uint16_t indx_tbl_entry);
uint32_t ipa_nati_get_entry_offset(
				struct ipa_nat_ip4_table_cache*,
				nat_table_type tbl_type,
				uint16_t  tbl_entry);

int ipa_nati_add_ipv4_tbl(uint32_t public_ip_addr,
				uint16_t number_of_entries,
				uint32_t *table_hanle);

int ipa_nati_alloc_table(uint16_t number_of_entries,
				struct ipa_ioc_nat_alloc_mem *mem,
				uint16_t*, uint16_t*);

int ipa_nati_update_cache(struct ipa_ioc_nat_alloc_mem *,
				uint32_t public_ip_addr,
				uint16_t tbl_entries,
				uint16_t expn_tbl_entries);

int ipa_nati_del_ipv4_table(uint32_t tbl_hdl);
int ipa_nati_reset_ipv4_table(uint32_t tbl_hdl);
int ipa_nati_post_ipv4_init_cmd(uint8_t tbl_index);

int ipa_nati_query_timestamp(uint32_t  tbl_hdl,
				uint32_t  rule_hdl,
				uint32_t  *time_stamp);

int ipa_nati_add_ipv4_rule(uint32_t tbl_hdl,
				const ipa_nat_ipv4_rule *clnt_rule,
				uint32_t *rule_hdl);

int ipa_nati_generate_rule(uint32_t tbl_hdl,
				const ipa_nat_ipv4_rule *clnt_rule,
				struct ipa_nat_sw_rule *rule,
				struct ipa_nat_indx_tbl_sw_rule *index_sw_rule,
				uint16_t *tbl_entry,
				uint16_t *indx_tbl_entry);

uint16_t ipa_nati_expn_tbl_free_entry(struct ipa_nat_rule *expn_tbl,
				uint16_t size);

uint16_t ipa_nati_generate_tbl_rule(const ipa_nat_ipv4_rule *clnt_rule,
				struct ipa_nat_sw_rule *sw_rule,
				struct ipa_nat_ip4_table_cache *tbl_ptr);

uint16_t ipa_nati_generate_index_rule(const ipa_nat_ipv4_rule *clnt_rule,
				struct ipa_nat_indx_tbl_sw_rule *sw_rule,
				struct ipa_nat_ip4_table_cache *tbl_ptr);

uint16_t ipa_nati_index_expn_get_free_entry(struct ipa_nat_indx_tbl_rule *tbl,
				uint16_t size);

void ipa_nati_copy_ipv4_rule_to_hw(
				struct ipa_nat_ip4_table_cache *ipv4_cache,
				struct ipa_nat_sw_rule *rule,
				uint16_t entry, uint8_t tbl_index);

void ipa_nati_copy_ipv4_index_rule_to_hw(
				struct ipa_nat_ip4_table_cache *ipv4_cache,
				struct ipa_nat_indx_tbl_sw_rule *indx_sw_rule,
				uint16_t entry, uint8_t tbl_index);

void ipa_nati_write_next_index(uint8_t tbl_indx,
				nat_table_type tbl_type,
				uint16_t value,
				uint32_t offset);

int ipa_nati_post_ipv4_dma_cmd(uint8_t tbl_indx,
				uint16_t entry);

int ipa_nati_del_ipv4_rule(uint32_t tbl_hdl,
				uint32_t rule_hdl);

int ipa_nati_post_del_dma_cmd(uint8_t tbl_indx,
				uint16_t tbl_entry,
				uint8_t expn_tbl,
				del_type rule_pos);

void ipa_nati_find_index_rule_pos(
				struct ipa_nat_ip4_table_cache *cache_ptr,
				uint16_t tbl_entry,
				del_type *rule_pos);

void ipa_nati_del_dead_ipv4_head_nodes(uint8_t tbl_indx);
void ipa_nati_find_rule_pos(struct ipa_nat_ip4_table_cache *cache_ptr,
				uint8_t expn_tbl,
				uint16_t tbl_entry,
				del_type *rule_pos);
void ipa_nati_del_dead_ipv4_head_nodes(uint8_t tbl_indx);

uint16_t Read16BitFieldValue(uint32_t param,
				ipa_nat_rule_field_type fld_type);

/* ========================================================
								Debug functions
   ========================================================*/
#ifdef NAT_DUMP
void ipa_nati_print_rule(struct ipa_nat_rule*, uint32_t);
void ipa_nat_dump_ipv4_table(uint32_t);
void ipa_nati_print_index_rule(struct ipa_nat_indx_tbl_rule*,
				uint32_t, uint16_t);
int ipa_nati_query_nat_rules(uint32_t, nat_table_type);
#endif

#endif /* #ifndef IPA_NAT_DRVI_H */
