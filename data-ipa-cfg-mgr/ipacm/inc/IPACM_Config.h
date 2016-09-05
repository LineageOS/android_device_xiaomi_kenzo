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
	IPACM_Config.h

	@brief
	This file implements the IPACM Configuration from XML file

	@Author
	Skylar Chang

*/
#ifndef IPACM_CONFIG_H
#define IPACM_CONFIG_H

#include "IPACM_Defs.h"
#include "IPACM_Xml.h"
#include "IPACM_EvtDispatcher.h"

typedef struct
{
  char iface_name[IPA_IFACE_NAME_LEN];
}NatIfaces;

/* for IPACM rm dependency use*/
typedef struct _ipa_rm_client
{
    ipa_rm_resource_name producer_rm1;
    ipa_rm_resource_name consumer_rm1;
    ipa_rm_resource_name producer_rm2;
    ipa_rm_resource_name consumer_rm2;
    bool producer1_up;            /* only monitor producer_rm1, not monitor producer_rm2 */
    bool consumer1_up;            /* only monitor consumer_rm1, not monitor consumer_rm2 */
    bool rm_set;                  /* once producer1_up and consumer1_up, will add bi-directional dependency */
    bool rx_bypass_ipa;          /* support WLAN may not register RX-property, should not add dependency */
}ipa_rm_client;

#define MAX_NUM_EXT_PROPS 25

/* used to hold extended properties */
typedef struct
{
	uint8_t num_ext_props;
	ipa_ioc_ext_intf_prop prop[MAX_NUM_EXT_PROPS];
} ipacm_ext_prop;

/* iface */
class IPACM_Config
{
public:

	/* IPACM ipa_client map to rm_resource*/
	ipa_rm_resource_name ipa_client_rm_map_tbl[IPA_CLIENT_MAX];

	/* IPACM monitored rm_depency table */
	ipa_rm_client ipa_rm_tbl[IPA_MAX_RM_ENTRY];

	/* IPACM rm_depency a2 endpoint check*/
	int ipa_rm_a2_check;

	/* Store interested interface and their configuration from XML file */
	ipa_ifi_dev_name_t *iface_table;

	/* Store interested ALG port from XML file */
	ipacm_alg *alg_table;

	/* Store private subnet configuration from XML file */
	ipa_private_subnet private_subnet_table[IPA_MAX_PRIVATE_SUBNET_ENTRIES];

	/* Store the non nat iface names */
	NatIfaces *pNatIfaces;

	/* Store the bridge iface names */
	char ipa_virtual_iface_name[IPA_IFACE_NAME_LEN];

	/* Store the number of interface IPACM read from XML file */
	int ipa_num_ipa_interfaces;

	int ipa_num_private_subnet;

	int ipa_num_alg_ports;

	int ipa_nat_max_entries;

	bool ipacm_odu_router_mode;

	bool ipacm_odu_enable;

	bool ipacm_odu_embms_enable;

	int ipa_nat_iface_entries;

	/* Store the total number of wlan guest ap configured */
	int ipa_num_wlan_guest_ap;

	/* Max valid rm entry */
	int ipa_max_valid_rm_entry;

	/* Store SW-enable or not */
	bool ipa_sw_rt_enable;

	/* Store bridge mode or not */
	bool ipa_bridge_enable;

	/* Store bridge netdev mac */
	uint8_t bridge_mac[IPA_MAC_ADDR_SIZE];

	/* Store the flt rule count for each producer client*/
	int flt_rule_count_v4[IPA_CLIENT_CONS - IPA_CLIENT_PROD];
	int flt_rule_count_v6[IPA_CLIENT_CONS - IPA_CLIENT_PROD];

	/* IPACM routing table name for v4/v6 */
	struct ipa_ioc_get_rt_tbl rt_tbl_lan_v4, rt_tbl_wan_v4, rt_tbl_default_v4, rt_tbl_v6, rt_tbl_wan_v6;
	struct ipa_ioc_get_rt_tbl rt_tbl_wan_dl;
	struct ipa_ioc_get_rt_tbl rt_tbl_odu_v4, rt_tbl_odu_v6;

	bool isMCC_Mode;

	/* To return the instance */
	static IPACM_Config* GetInstance();

	const char* getEventName(ipa_cm_event_id event_id);

	inline void increaseFltRuleCount(int index, ipa_ip_type iptype, int increment)
	{
		if((index >= IPA_CLIENT_CONS - IPA_CLIENT_PROD) || (index < 0))
		{
			IPACMERR("Index is out of range: %d.\n", index);
			return;
		}
		if(iptype == IPA_IP_v4)
		{
			flt_rule_count_v4[index] += increment;
			IPACMDBG_H("Now num of v4 flt rules on client %d is %d.\n", index, flt_rule_count_v4[index]);
		}
		else
		{
			flt_rule_count_v6[index] += increment;
			IPACMDBG_H("Now num of v6 flt rules on client %d is %d.\n", index, flt_rule_count_v6[index]);
		}
		return;
	}

	inline void decreaseFltRuleCount(int index, ipa_ip_type iptype, int decrement)
	{
		if((index >= IPA_CLIENT_CONS - IPA_CLIENT_PROD) || (index < 0))
		{
			IPACMERR("Index is out of range: %d.\n", index);
			return;
		}
		if(iptype == IPA_IP_v4)
		{
			flt_rule_count_v4[index] -= decrement;
			IPACMDBG_H("Now num of v4 flt rules on client %d is %d.\n", index, flt_rule_count_v4[index]);
		}
		else
		{
			flt_rule_count_v6[index] -= decrement;
			IPACMDBG_H("Now num of v6 flt rules on client %d is %d.\n", index, flt_rule_count_v6[index]);
		}
		return;
	}

	inline int getFltRuleCount(int index, ipa_ip_type iptype)
	{
		if((index >= IPA_CLIENT_CONS - IPA_CLIENT_PROD) || (index < 0))
		{
			IPACMERR("Index is out of range: %d.\n", index);
			return -1;
		}
		if(iptype == IPA_IP_v4)
		{
			return flt_rule_count_v4[index];
		}
		else
		{
			return flt_rule_count_v6[index];
		}
	}

	inline int GetAlgPortCnt()
	{
		return ipa_num_alg_ports;
	}

	int GetAlgPorts(int nPorts, ipacm_alg *pAlgPorts);

	inline int GetNatMaxEntries(void)
	{
		return ipa_nat_max_entries;
	}

	inline int GetNatIfacesCnt()
	{
		return ipa_nat_iface_entries;
	}
	int GetNatIfaces(int nPorts, NatIfaces *ifaces);

	/* for IPACM resource manager dependency usage */
	void AddRmDepend(ipa_rm_resource_name rm1,bool rx_bypass_ipa);

	void DelRmDepend(ipa_rm_resource_name rm1);

	int AddNatIfaces(char *dev_name);

	int DelNatIfaces(char *dev_name);

	inline void SetQmapId(uint8_t id)
	{
		qmap_id = id;
	}

	inline uint8_t GetQmapId()
	{
		return qmap_id;
	}

	int SetExtProp(ipa_ioc_query_intf_ext_props *prop);

	ipacm_ext_prop* GetExtProp(ipa_ip_type ip_type);

	int DelExtProp(ipa_ip_type ip_type);

	int Init(void);

	inline bool isPrivateSubnet(uint32_t ip_addr)
	{
		for(int cnt=0; cnt<ipa_num_private_subnet; cnt++)
		{
			if(private_subnet_table[cnt].subnet_addr ==
				 (private_subnet_table[cnt].subnet_mask & ip_addr))
			{
				return true;
			}
		}

		return false;
	}
#ifdef FEATURE_IPA_ANDROID
	inline bool AddPrivateSubnet(uint32_t ip_addr, int ipa_if_index)
	{
		ipacm_cmd_q_data evt_data;
		ipacm_event_data_fid *data_fid;
		uint32_t subnet_mask = ~0;
		for(int cnt=0; cnt<ipa_num_private_subnet; cnt++)
		{
			if(private_subnet_table[cnt].subnet_addr == ip_addr)
			{
				IPACMDBG("Already has private subnet_addr as: 0x%x in entry(%d) \n", ip_addr, cnt);
				return true;
			}
		}

		if(ipa_num_private_subnet < IPA_MAX_PRIVATE_SUBNET_ENTRIES)
		{
			IPACMDBG("Add IPACM private subnet_addr as: 0x%x in entry(%d) \n", ip_addr, ipa_num_private_subnet);
			private_subnet_table[ipa_num_private_subnet].subnet_addr = ip_addr;
			private_subnet_table[ipa_num_private_subnet].subnet_mask = (subnet_mask >> 8) << 8;
			ipa_num_private_subnet++;

			/* IPACM private subnet set changes */
			data_fid = (ipacm_event_data_fid *)malloc(sizeof(ipacm_event_data_fid));
			if(data_fid == NULL)
			{
				IPACMERR("unable to allocate memory for event data_fid\n");
				return IPACM_FAILURE;
			}
			data_fid->if_index = ipa_if_index; // already ipa index, not fid index
			evt_data.event = IPA_PRIVATE_SUBNET_CHANGE_EVENT;
			evt_data.evt_data = data_fid;

			/* Insert IPA_PRIVATE_SUBNET_CHANGE_EVENT to command queue */
			IPACM_EvtDispatcher::PostEvt(&evt_data);
			return true;
		}
		IPACMERR("IPACM private subnet_addr overflow, total entry(%d)\n", ipa_num_private_subnet);
		return false;
	}

	inline bool DelPrivateSubnet(uint32_t ip_addr, int ipa_if_index)
	{
		ipacm_cmd_q_data evt_data;
		ipacm_event_data_fid *data_fid;
		for(int cnt=0; cnt<ipa_num_private_subnet; cnt++)
		{
			if(private_subnet_table[cnt].subnet_addr == ip_addr)
			{
				IPACMDBG("Found private subnet_addr as: 0x%x in entry(%d) \n", ip_addr, cnt);
				for (; cnt < ipa_num_private_subnet - 1; cnt++)
				{
					private_subnet_table[cnt].subnet_addr = private_subnet_table[cnt+1].subnet_addr;
				}
				ipa_num_private_subnet = ipa_num_private_subnet - 1;

				/* IPACM private subnet set changes */
				data_fid = (ipacm_event_data_fid *)malloc(sizeof(ipacm_event_data_fid));
				if(data_fid == NULL)
				{
					IPACMERR("unable to allocate memory for event data_fid\n");
					return IPACM_FAILURE;
				}
				data_fid->if_index = ipa_if_index; // already ipa index, not fid index
				evt_data.event = IPA_PRIVATE_SUBNET_CHANGE_EVENT;
				evt_data.evt_data = data_fid;

				/* Insert IPA_PRIVATE_SUBNET_CHANGE_EVENT to command queue */
				IPACM_EvtDispatcher::PostEvt(&evt_data);
				return true;
			}
		}
		IPACMDBG("can't find private subnet_addr as: 0x%x \n", ip_addr);
		return false;
	}
#endif /* defined(FEATURE_IPA_ANDROID)*/

	static const char *DEVICE_NAME_ODU;

private:
	static IPACM_Config *pInstance;
	static const char *DEVICE_NAME;
	IPACM_Config(void);
	int m_fd; /* File descriptor of the IPA device node /dev/ipa */
	uint8_t qmap_id;
	ipacm_ext_prop ext_prop_v4;
	ipacm_ext_prop ext_prop_v6;
};

#endif /* IPACM_CONFIG */
