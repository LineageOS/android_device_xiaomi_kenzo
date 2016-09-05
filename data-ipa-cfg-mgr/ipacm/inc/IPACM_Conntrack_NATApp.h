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
#ifndef IPACM_CONNTRACK_NATAPP_H
#define IPACM_CONNTRACK_NATAPP_H

#include <string.h>  /* for stderror */
#include <stdlib.h>
#include <cstdio>  /* for perror */

#include "IPACM_Config.h"
#include "IPACM_Xml.h"

extern "C"
{
#include <libnetfilter_conntrack/libnetfilter_conntrack.h>
#include <ipa_nat_drv.h>
}

#define MAX_TEMP_ENTRIES 25

#define IPACM_TCP_FULL_FILE_NAME  "/proc/sys/net/ipv4/netfilter/ip_conntrack_tcp_timeout_established"
#define IPACM_UDP_FULL_FILE_NAME   "/proc/sys/net/ipv4/netfilter/ip_conntrack_udp_timeout_stream"

typedef struct _nat_table_entry
{
	uint32_t private_ip;
	uint16_t private_port;

	uint32_t target_ip;
	uint16_t target_port;

	uint32_t public_ip;
	uint16_t public_port;

	u_int8_t  protocol;
	uint32_t timestamp;

	bool dst_nat;
	bool enabled;
	uint32_t rule_hdl;

}nat_table_entry;

#define CHK_TBL_HDL()  if(nat_table_hdl == 0){ return -1; }

class NatApp
{
private:

	static NatApp *pInstance;

	nat_table_entry *cache;
	nat_table_entry temp[MAX_TEMP_ENTRIES];
	uint32_t pub_ip_addr;
	uint32_t pub_ip_addr_pre;
	uint32_t nat_table_hdl;

	int curCnt, max_entries;

	ipacm_alg *pALGPorts;
	uint16_t nALGPort;

	uint32_t tcp_timeout;
	uint32_t udp_timeout;

	uint32_t PwrSaveIfs[IPA_MAX_NUM_WIFI_CLIENTS];

	struct nf_conntrack *ct;
	struct nfct_handle *ct_hdl;

	NatApp();
	int Init();

	void UpdateCTUdpTs(nat_table_entry *, uint32_t);
	bool ChkForDup(const nat_table_entry *);
	bool isAlgPort(uint8_t, uint16_t);
	void Reset();
	bool isPwrSaveIf(uint32_t);

public:
	static NatApp* GetInstance();

	int AddTable(uint32_t);
	uint32_t GetTableHdl(uint32_t);
	int DeleteTable(uint32_t);

	int AddEntry(const nat_table_entry *);
	int DeleteEntry(const nat_table_entry *);

	void UpdateUDPTimeStamp();

	int UpdatePwrSaveIf(uint32_t);
	int ResetPwrSaveIf(uint32_t);
	int DelEntriesOnClntDiscon(uint32_t);
	int DelEntriesOnSTAClntDiscon(uint32_t);

	void Read_TcpUdp_Timeout(void);

	void AddTempEntry(const nat_table_entry *);
	void CacheEntry(const nat_table_entry *);
	void DeleteTempEntry(const nat_table_entry *);
	void FlushTempEntries(uint32_t, bool, bool isDummy = false);
};



#endif /* IPACM_CONNTRACK_NATAPP_H */
