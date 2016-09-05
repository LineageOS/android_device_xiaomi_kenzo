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
		IPACM_Config.cpp

		@brief
		This file implements the IPACM Configuration from XML file

		@Author
		Skylar Chang

*/
#include <IPACM_Config.h>
#include <IPACM_Log.h>
#include <IPACM_Iface.h>
#include <sys/ioctl.h>
#include <fcntl.h>

IPACM_Config *IPACM_Config::pInstance = NULL;
const char *IPACM_Config::DEVICE_NAME = "/dev/ipa";
const char *IPACM_Config::DEVICE_NAME_ODU = "/dev/odu_ipa_bridge";

#define __stringify(x...) #x

const char *ipacm_event_name[] = {
	__stringify(IPA_CFG_CHANGE_EVENT),                     /* NULL */
	__stringify(IPA_PRIVATE_SUBNET_CHANGE_EVENT),          /* ipacm_event_data_fid */
	__stringify(IPA_FIREWALL_CHANGE_EVENT),                /* NULL */
	__stringify(IPA_LINK_UP_EVENT),                        /* ipacm_event_data_fid */
	__stringify(IPA_LINK_DOWN_EVENT),                      /* ipacm_event_data_fid */
	__stringify(IPA_USB_LINK_UP_EVENT),                    /* ipacm_event_data_fid */
	__stringify(IPA_BRIDGE_LINK_UP_EVENT),                 /* ipacm_event_data_all */
	__stringify(IPA_WAN_EMBMS_LINK_UP_EVENT),              /* ipacm_event_data_mac */
	__stringify(IPA_ADDR_ADD_EVENT),                       /* ipacm_event_data_addr */
	__stringify(IPA_ADDR_DEL_EVENT),                       /* no use */
	__stringify(IPA_ROUTE_ADD_EVENT),                      /* ipacm_event_data_addr */
	__stringify(IPA_ROUTE_DEL_EVENT),                      /* ipacm_event_data_addr */
	__stringify(IPA_WAN_UPSTREAM_ROUTE_ADD_EVENT),         /* ipacm_event_data_fid */
	__stringify(IPA_WAN_UPSTREAM_ROUTE_DEL_EVENT),         /* ipacm_event_data_fid */
	__stringify(IPA_WLAN_AP_LINK_UP_EVENT),                /* ipacm_event_data_mac */
	__stringify(IPA_WLAN_STA_LINK_UP_EVENT),               /* ipacm_event_data_mac */
	__stringify(IPA_WLAN_LINK_DOWN_EVENT),                 /* ipacm_event_data_mac */
	__stringify(IPA_WLAN_CLIENT_ADD_EVENT),                /* ipacm_event_data_mac */
	__stringify(IPA_WLAN_CLIENT_ADD_EVENT_EX),             /* ipacm_event_data_wlan_ex */
	__stringify(IPA_WLAN_CLIENT_DEL_EVENT),                /* ipacm_event_data_mac */
	__stringify(IPA_WLAN_CLIENT_POWER_SAVE_EVENT),         /* ipacm_event_data_mac */
	__stringify(IPA_WLAN_CLIENT_RECOVER_EVENT),            /* ipacm_event_data_mac */
	__stringify(IPA_NEW_NEIGH_EVENT),                      /* ipacm_event_data_all */
	__stringify(IPA_DEL_NEIGH_EVENT),                      /* ipacm_event_data_all */
	__stringify(IPA_NEIGH_CLIENT_IP_ADDR_ADD_EVENT),       /* ipacm_event_data_all */
	__stringify(IPA_NEIGH_CLIENT_IP_ADDR_DEL_EVENT),       /* ipacm_event_data_all */
	__stringify(IPA_SW_ROUTING_ENABLE),                    /* NULL */
	__stringify(IPA_SW_ROUTING_DISABLE),                   /* NULL */
	__stringify(IPA_PROCESS_CT_MESSAGE),                   /* ipacm_ct_evt_data */
	__stringify(IPA_PROCESS_CT_MESSAGE_V6),                /* ipacm_ct_evt_data */
	__stringify(IPA_LAN_TO_LAN_NEW_CONNECTION),            /* ipacm_event_connection */
	__stringify(IPA_LAN_TO_LAN_DEL_CONNECTION),            /* ipacm_event_connection */
	__stringify(IPA_WLAN_SWITCH_TO_SCC),                   /* No Data */
	__stringify(IPA_WLAN_SWITCH_TO_MCC),                   /* No Data */
	__stringify(IPA_CRADLE_WAN_MODE_SWITCH),               /* ipacm_event_cradle_wan_mode */
	__stringify(IPA_WAN_XLAT_CONNECT_EVENT),               /* ipacm_event_data_fid */
	__stringify(IPA_TETHERING_STATS_UPDATE_EVENT),         /* ipacm_event_data_fid */
	__stringify(IPA_NETWORK_STATS_UPDATE_EVENT),           /* ipacm_event_data_fid */
	__stringify(IPA_EXTERNAL_EVENT_MAX),
	__stringify(IPA_HANDLE_WAN_UP),                        /* ipacm_event_iface_up  */
	__stringify(IPA_HANDLE_WAN_DOWN),                      /* ipacm_event_iface_up  */
	__stringify(IPA_HANDLE_WAN_UP_V6),                     /* NULL */
	__stringify(IPA_HANDLE_WAN_DOWN_V6),                   /* NULL */
	__stringify(IPA_HANDLE_WAN_UP_TETHER),                 /* ipacm_event_iface_up_tehter */
	__stringify(IPA_HANDLE_WAN_DOWN_TETHER),               /* ipacm_event_iface_up_tehter */
	__stringify(IPA_HANDLE_WAN_UP_V6_TETHER),              /* ipacm_event_iface_up_tehter */
	__stringify(IPA_HANDLE_WAN_DOWN_V6_TETHER),            /* ipacm_event_iface_up_tehter */
	__stringify(IPA_HANDLE_WLAN_UP),                       /* ipacm_event_iface_up */
	__stringify(IPA_HANDLE_LAN_UP),                        /* ipacm_event_iface_up */
	__stringify(IPA_ETH_BRIDGE_IFACE_UP),                  /* ipacm_event_eth_bridge*/
	__stringify(IPA_ETH_BRIDGE_IFACE_DOWN),                /* ipacm_event_eth_bridge*/
	__stringify(IPA_ETH_BRIDGE_CLIENT_ADD),                /* ipacm_event_eth_bridge*/
	__stringify(IPA_ETH_BRIDGE_CLIENT_DEL),                /* ipacm_event_eth_bridge*/
	__stringify(IPA_ETH_BRIDGE_WLAN_SCC_MCC_SWITCH),       /* ipacm_event_eth_bridge*/
	__stringify(IPA_LAN_DELETE_SELF),                      /* ipacm_event_data_fid */
	__stringify(IPACM_EVENT_MAX),
};

IPACM_Config::IPACM_Config()
{
	iface_table = NULL;
	alg_table = NULL;
	memset(&ipa_client_rm_map_tbl, 0, sizeof(ipa_client_rm_map_tbl));
	memset(&ipa_rm_tbl, 0, sizeof(ipa_rm_tbl));
	ipa_rm_a2_check=0;
	ipacm_odu_enable = false;
	ipacm_odu_router_mode = false;
	ipa_num_wlan_guest_ap = 0;

	ipa_num_ipa_interfaces = 0;
	ipa_num_private_subnet = 0;
	ipa_num_alg_ports = 0;
	ipa_nat_max_entries = 0;
	ipa_nat_iface_entries = 0;
	ipa_sw_rt_enable = false;
	ipa_bridge_enable = false;
	isMCC_Mode = false;
	ipa_max_valid_rm_entry = 0;

	memset(&rt_tbl_default_v4, 0, sizeof(rt_tbl_default_v4));
	memset(&rt_tbl_lan_v4, 0, sizeof(rt_tbl_lan_v4));
	memset(&rt_tbl_wan_v4, 0, sizeof(rt_tbl_wan_v4));
	memset(&rt_tbl_v6, 0, sizeof(rt_tbl_v6));
	memset(&rt_tbl_wan_v6, 0, sizeof(rt_tbl_wan_v6));
	memset(&rt_tbl_wan_dl, 0, sizeof(rt_tbl_wan_dl));
	memset(&rt_tbl_odu_v4, 0, sizeof(rt_tbl_odu_v4));
	memset(&rt_tbl_odu_v6, 0, sizeof(rt_tbl_odu_v6));

	memset(&ext_prop_v4, 0, sizeof(ext_prop_v4));
	memset(&ext_prop_v6, 0, sizeof(ext_prop_v6));

	qmap_id = ~0;

	memset(flt_rule_count_v4, 0, (IPA_CLIENT_CONS - IPA_CLIENT_PROD)*sizeof(int));
	memset(flt_rule_count_v6, 0, (IPA_CLIENT_CONS - IPA_CLIENT_PROD)*sizeof(int));
	memset(bridge_mac, 0, IPA_MAC_ADDR_SIZE*sizeof(uint8_t));

	IPACMDBG_H(" create IPACM_Config constructor\n");
	return;
}

int IPACM_Config::Init(void)
{
	/* Read IPACM Config file */
	char	IPACM_config_file[IPA_MAX_FILE_LEN];
	IPACM_conf_t	*cfg;
	cfg = (IPACM_conf_t *)malloc(sizeof(IPACM_conf_t));
	if(cfg == NULL)
	{
		IPACMERR("Unable to allocate cfg memory.\n");
		return IPACM_FAILURE;
	}
	uint32_t subnet_addr;
	uint32_t subnet_mask;
	int i, ret = IPACM_SUCCESS;
	struct in_addr in_addr_print;

	m_fd = open(DEVICE_NAME, O_RDWR);
	if (0 > m_fd)
	{
		IPACMERR("Failed opening %s.\n", DEVICE_NAME);
	}
	strncpy(IPACM_config_file, "/etc/IPACM_cfg.xml", sizeof(IPACM_config_file));

	IPACMDBG_H("\n IPACM XML file is %s \n", IPACM_config_file);
	if (IPACM_SUCCESS == ipacm_read_cfg_xml(IPACM_config_file, cfg))
	{
		IPACMDBG_H("\n IPACM XML read OK \n");
	}
	else
	{
		IPACMERR("\n IPACM XML read failed \n");
		ret = IPACM_FAILURE;
		goto fail;
	}

	/* Check wlan AP-AP access mode configuration */
	if (cfg->num_wlan_guest_ap == 2)
	{
		IPACMDBG_H("IPACM_Config::Both wlan APs can not be configured in guest ap mode. \n");
		IPACMDBG_H("IPACM_Config::configure both APs in full access mode or at least one in guest ap mode. \n");
		ret = IPACM_FAILURE;
		goto fail;
	}
	/* Construct IPACM Iface table */
	ipa_num_ipa_interfaces = cfg->iface_config.num_iface_entries;
	if (iface_table != NULL)
	{
		free(iface_table);
		iface_table = NULL;
		IPACMDBG_H("RESET IPACM_Config::iface_table\n");
	}
	iface_table = (ipa_ifi_dev_name_t *)calloc(ipa_num_ipa_interfaces,
					sizeof(ipa_ifi_dev_name_t));
	if(iface_table == NULL)
	{
		IPACMERR("Unable to allocate iface_table memory.\n");
		ret = IPACM_FAILURE;
		goto fail;
	}

	for (i = 0; i < cfg->iface_config.num_iface_entries; i++)
	{
		strncpy(iface_table[i].iface_name, cfg->iface_config.iface_entries[i].iface_name, sizeof(iface_table[i].iface_name));
		iface_table[i].if_cat = cfg->iface_config.iface_entries[i].if_cat;
		iface_table[i].if_mode = cfg->iface_config.iface_entries[i].if_mode;
		iface_table[i].wlan_mode = cfg->iface_config.iface_entries[i].wlan_mode;
		IPACMDBG_H("IPACM_Config::iface_table[%d] = %s, cat=%d, mode=%d wlan-mode=%d \n", i, iface_table[i].iface_name,
				iface_table[i].if_cat, iface_table[i].if_mode, iface_table[i].wlan_mode);
		/* copy bridge interface name to ipacmcfg */
		if( iface_table[i].if_cat == VIRTUAL_IF)
		{
			strlcpy(ipa_virtual_iface_name, iface_table[i].iface_name, sizeof(ipa_virtual_iface_name));
			IPACMDBG_H("ipa_virtual_iface_name(%s) \n", ipa_virtual_iface_name);
		}
	}

	/* Construct IPACM Private_Subnet table */
	memset(&private_subnet_table, 0, sizeof(private_subnet_table));
	ipa_num_private_subnet = cfg->private_subnet_config.num_subnet_entries;

	for (i = 0; i < cfg->private_subnet_config.num_subnet_entries; i++)
	{
		memcpy(&private_subnet_table[i].subnet_addr,
					 &cfg->private_subnet_config.private_subnet_entries[i].subnet_addr,
					 sizeof(cfg->private_subnet_config.private_subnet_entries[i].subnet_addr));

		memcpy(&private_subnet_table[i].subnet_mask,
					 &cfg->private_subnet_config.private_subnet_entries[i].subnet_mask,
					 sizeof(cfg->private_subnet_config.private_subnet_entries[i].subnet_mask));

		subnet_addr = htonl(private_subnet_table[i].subnet_addr);
		memcpy(&in_addr_print,&subnet_addr,sizeof(in_addr_print));
		IPACMDBG_H("%dst::private_subnet_table= %s \n ", i,
						 inet_ntoa(in_addr_print));

		subnet_mask =  htonl(private_subnet_table[i].subnet_mask);
		memcpy(&in_addr_print,&subnet_mask,sizeof(in_addr_print));
		IPACMDBG_H("%dst::private_subnet_table= %s \n ", i,
						 inet_ntoa(in_addr_print));
	}

	/* Construct IPACM ALG table */
	ipa_num_alg_ports = cfg->alg_config.num_alg_entries;
	if (alg_table != NULL)
	{
		free(alg_table);
		alg_table = NULL;
		IPACMDBG_H("RESET IPACM_Config::alg_table \n");
	}
	alg_table = (ipacm_alg *)calloc(ipa_num_alg_ports,
				sizeof(ipacm_alg));
	if(alg_table == NULL)
	{
		IPACMERR("Unable to allocate alg_table memory.\n");
		ret = IPACM_FAILURE;
		free(iface_table);
		goto fail;;
	}
	for (i = 0; i < cfg->alg_config.num_alg_entries; i++)
	{
		alg_table[i].protocol = cfg->alg_config.alg_entries[i].protocol;
		alg_table[i].port = cfg->alg_config.alg_entries[i].port;
		IPACMDBG_H("IPACM_Config::ipacm_alg[%d] = %d, port=%d\n", i, alg_table[i].protocol, alg_table[i].port);
	}

	ipa_nat_max_entries = cfg->nat_max_entries;
	IPACMDBG_H("Nat Maximum Entries %d\n", ipa_nat_max_entries);

	/* Find ODU is either router mode or bridge mode*/
	ipacm_odu_enable = cfg->odu_enable;
	ipacm_odu_router_mode = cfg->router_mode_enable;
	ipacm_odu_embms_enable = cfg->odu_embms_enable;
	IPACMDBG_H("ipacm_odu_enable %d\n", ipacm_odu_enable);
	IPACMDBG_H("ipacm_odu_mode %d\n", ipacm_odu_router_mode);
	IPACMDBG_H("ipacm_odu_embms_enable %d\n", ipacm_odu_embms_enable);
	ipa_num_wlan_guest_ap = cfg->num_wlan_guest_ap;
	IPACMDBG_H("ipa_num_wlan_guest_ap %d\n",ipa_num_wlan_guest_ap);

	/* Allocate more non-nat entries if the monitored iface dun have Tx/Rx properties */
	if (pNatIfaces != NULL)
	{
		free(pNatIfaces);
		pNatIfaces = NULL;
		IPACMDBG_H("RESET IPACM_Config::pNatIfaces \n");
	}
	ipa_nat_iface_entries = 0;
	pNatIfaces = (NatIfaces *)calloc(ipa_num_ipa_interfaces, sizeof(NatIfaces));
	if (pNatIfaces == NULL)
	{
		IPACMERR("unable to allocate nat ifaces\n");
		ret = IPACM_FAILURE;
		free(iface_table);
		free(alg_table);
		goto fail;
	}

	/* Construct the routing table ictol name in iface static member*/
	rt_tbl_default_v4.ip = IPA_IP_v4;
	strncpy(rt_tbl_default_v4.name, V4_DEFAULT_ROUTE_TABLE_NAME, sizeof(rt_tbl_default_v4.name));

	rt_tbl_lan_v4.ip = IPA_IP_v4;
	strncpy(rt_tbl_lan_v4.name, V4_LAN_ROUTE_TABLE_NAME, sizeof(rt_tbl_lan_v4.name));

	rt_tbl_wan_v4.ip = IPA_IP_v4;
	strncpy(rt_tbl_wan_v4.name, V4_WAN_ROUTE_TABLE_NAME, sizeof(rt_tbl_wan_v4.name));

	rt_tbl_v6.ip = IPA_IP_v6;
	strncpy(rt_tbl_v6.name, V6_COMMON_ROUTE_TABLE_NAME, sizeof(rt_tbl_v6.name));

	rt_tbl_wan_v6.ip = IPA_IP_v6;
	strncpy(rt_tbl_wan_v6.name, V6_WAN_ROUTE_TABLE_NAME, sizeof(rt_tbl_wan_v6.name));

	rt_tbl_odu_v4.ip = IPA_IP_v4;
	strncpy(rt_tbl_odu_v4.name, V4_ODU_ROUTE_TABLE_NAME, sizeof(rt_tbl_odu_v4.name));

	rt_tbl_odu_v6.ip = IPA_IP_v6;
	strncpy(rt_tbl_odu_v6.name, V6_ODU_ROUTE_TABLE_NAME, sizeof(rt_tbl_odu_v6.name));

	rt_tbl_wan_dl.ip = IPA_IP_MAX;
	strncpy(rt_tbl_wan_dl.name, WAN_DL_ROUTE_TABLE_NAME, sizeof(rt_tbl_wan_dl.name));

	/* Construct IPACM ipa_client map to rm_resource table */
	ipa_client_rm_map_tbl[IPA_CLIENT_WLAN1_PROD]= IPA_RM_RESOURCE_WLAN_PROD;
	ipa_client_rm_map_tbl[IPA_CLIENT_USB_PROD]= IPA_RM_RESOURCE_USB_PROD;
	ipa_client_rm_map_tbl[IPA_CLIENT_A5_WLAN_AMPDU_PROD]= IPA_RM_RESOURCE_HSIC_PROD;
	ipa_client_rm_map_tbl[IPA_CLIENT_A2_EMBEDDED_PROD]= IPA_RM_RESOURCE_Q6_PROD;
	ipa_client_rm_map_tbl[IPA_CLIENT_A2_TETHERED_PROD]= IPA_RM_RESOURCE_Q6_PROD;
	ipa_client_rm_map_tbl[IPA_CLIENT_APPS_LAN_WAN_PROD]= IPA_RM_RESOURCE_Q6_PROD;
	ipa_client_rm_map_tbl[IPA_CLIENT_WLAN1_CONS]= IPA_RM_RESOURCE_WLAN_CONS;
	ipa_client_rm_map_tbl[IPA_CLIENT_WLAN2_CONS]= IPA_RM_RESOURCE_WLAN_CONS;
	ipa_client_rm_map_tbl[IPA_CLIENT_WLAN3_CONS]= IPA_RM_RESOURCE_WLAN_CONS;
	ipa_client_rm_map_tbl[IPA_CLIENT_WLAN4_CONS]= IPA_RM_RESOURCE_WLAN_CONS;
	ipa_client_rm_map_tbl[IPA_CLIENT_USB_CONS]= IPA_RM_RESOURCE_USB_CONS;
	ipa_client_rm_map_tbl[IPA_CLIENT_A2_EMBEDDED_CONS]= IPA_RM_RESOURCE_Q6_CONS;
	ipa_client_rm_map_tbl[IPA_CLIENT_A2_TETHERED_CONS]= IPA_RM_RESOURCE_Q6_CONS;
	ipa_client_rm_map_tbl[IPA_CLIENT_APPS_WAN_CONS]= IPA_RM_RESOURCE_Q6_CONS;
	ipa_client_rm_map_tbl[IPA_CLIENT_ODU_PROD]= IPA_RM_RESOURCE_ODU_ADAPT_PROD;
	ipa_client_rm_map_tbl[IPA_CLIENT_ODU_EMB_CONS]= IPA_RM_RESOURCE_ODU_ADAPT_CONS;
	ipa_client_rm_map_tbl[IPA_CLIENT_ODU_TETH_CONS]= IPA_RM_RESOURCE_ODU_ADAPT_CONS;

	/* Create the entries which IPACM wants to add dependencies on */
	ipa_rm_tbl[0].producer_rm1 = IPA_RM_RESOURCE_WLAN_PROD;
	ipa_rm_tbl[0].consumer_rm1 = IPA_RM_RESOURCE_Q6_CONS;
	ipa_rm_tbl[0].producer_rm2 = IPA_RM_RESOURCE_Q6_PROD;
	ipa_rm_tbl[0].consumer_rm2 = IPA_RM_RESOURCE_WLAN_CONS;

	ipa_rm_tbl[1].producer_rm1 = IPA_RM_RESOURCE_USB_PROD;
	ipa_rm_tbl[1].consumer_rm1 = IPA_RM_RESOURCE_Q6_CONS;
	ipa_rm_tbl[1].producer_rm2 = IPA_RM_RESOURCE_Q6_PROD;
	ipa_rm_tbl[1].consumer_rm2 = IPA_RM_RESOURCE_USB_CONS;

	ipa_rm_tbl[2].producer_rm1 = IPA_RM_RESOURCE_WLAN_PROD;
	ipa_rm_tbl[2].consumer_rm1 = IPA_RM_RESOURCE_USB_CONS;
	ipa_rm_tbl[2].producer_rm2 = IPA_RM_RESOURCE_USB_PROD;
	ipa_rm_tbl[2].consumer_rm2 = IPA_RM_RESOURCE_WLAN_CONS;

	ipa_rm_tbl[3].producer_rm1 = IPA_RM_RESOURCE_ODU_ADAPT_PROD;
	ipa_rm_tbl[3].consumer_rm1 = IPA_RM_RESOURCE_Q6_CONS;
	ipa_rm_tbl[3].producer_rm2 = IPA_RM_RESOURCE_Q6_PROD;
	ipa_rm_tbl[3].consumer_rm2 = IPA_RM_RESOURCE_ODU_ADAPT_CONS;

	ipa_rm_tbl[4].producer_rm1 = IPA_RM_RESOURCE_WLAN_PROD;
	ipa_rm_tbl[4].consumer_rm1 = IPA_RM_RESOURCE_ODU_ADAPT_CONS;
	ipa_rm_tbl[4].producer_rm2 = IPA_RM_RESOURCE_ODU_ADAPT_PROD;
	ipa_rm_tbl[4].consumer_rm2 = IPA_RM_RESOURCE_WLAN_CONS;

	ipa_rm_tbl[5].producer_rm1 = IPA_RM_RESOURCE_ODU_ADAPT_PROD;
	ipa_rm_tbl[5].consumer_rm1 = IPA_RM_RESOURCE_USB_CONS;
	ipa_rm_tbl[5].producer_rm2 = IPA_RM_RESOURCE_USB_PROD;
	ipa_rm_tbl[5].consumer_rm2 = IPA_RM_RESOURCE_ODU_ADAPT_CONS;
	ipa_max_valid_rm_entry = 6; /* max is IPA_MAX_RM_ENTRY (6)*/

	IPACMDBG_H(" depend MAP-0 rm index %d to rm index: %d \n", IPA_RM_RESOURCE_WLAN_PROD, IPA_RM_RESOURCE_Q6_CONS);
	IPACMDBG_H(" depend MAP-1 rm index %d to rm index: %d \n", IPA_RM_RESOURCE_USB_PROD, IPA_RM_RESOURCE_Q6_CONS);
	IPACMDBG_H(" depend MAP-2 rm index %d to rm index: %d \n", IPA_RM_RESOURCE_WLAN_PROD, IPA_RM_RESOURCE_USB_CONS);
	IPACMDBG_H(" depend MAP-3 rm index %d to rm index: %d \n", IPA_RM_RESOURCE_ODU_ADAPT_PROD, IPA_RM_RESOURCE_Q6_CONS);
	IPACMDBG_H(" depend MAP-4 rm index %d to rm index: %d \n", IPA_RM_RESOURCE_WLAN_PROD, IPA_RM_RESOURCE_ODU_ADAPT_CONS);
	IPACMDBG_H(" depend MAP-5 rm index %d to rm index: %d \n", IPA_RM_RESOURCE_ODU_ADAPT_PROD, IPA_RM_RESOURCE_USB_CONS);

fail:
	if (cfg != NULL)
	{
		free(cfg);
		cfg = NULL;
	}

	return ret;
}

IPACM_Config* IPACM_Config::GetInstance()
{
	int res = IPACM_SUCCESS;

	if (pInstance == NULL)
	{
		pInstance = new IPACM_Config();

		res = pInstance->Init();
		if (res != IPACM_SUCCESS)
		{
			delete pInstance;
			IPACMERR("unable to initialize config instance\n");
			return NULL;
		}
	}

	return pInstance;
}

int IPACM_Config::GetAlgPorts(int nPorts, ipacm_alg *pAlgPorts)
{
	if (nPorts <= 0 || pAlgPorts == NULL)
	{
		IPACMERR("Invalid input\n");
		return -1;
	}

	for (int cnt = 0; cnt < nPorts; cnt++)
	{
		pAlgPorts[cnt].protocol = alg_table[cnt].protocol;
		pAlgPorts[cnt].port = alg_table[cnt].port;
	}

	return 0;
}

int IPACM_Config::GetNatIfaces(int nIfaces, NatIfaces *pIfaces)
{
	if (nIfaces <= 0 || pIfaces == NULL)
	{
		IPACMERR("Invalid input\n");
		return -1;
	}

	for (int cnt=0; cnt<nIfaces; cnt++)
	{
		memcpy(pIfaces[cnt].iface_name,
					 pNatIfaces[cnt].iface_name,
					 sizeof(pIfaces[cnt].iface_name));
	}

	return 0;
}


int IPACM_Config::AddNatIfaces(char *dev_name)
{
	int i;
	/* Check if this iface already in NAT-iface*/
	for(i = 0; i < ipa_nat_iface_entries; i++)
	{
		if(strncmp(dev_name,
							 pNatIfaces[i].iface_name,
							 sizeof(pNatIfaces[i].iface_name)) == 0)
		{
			IPACMDBG("Interface (%s) is add to nat iface already\n", dev_name);
				return 0;
		}
	}

	IPACMDBG_H("Add iface %s to NAT-ifaces, origin it has %d nat ifaces\n",
					          dev_name, ipa_nat_iface_entries);
	ipa_nat_iface_entries++;

	if (ipa_nat_iface_entries < ipa_num_ipa_interfaces)
	{
		memcpy(pNatIfaces[ipa_nat_iface_entries - 1].iface_name,
					 dev_name, IPA_IFACE_NAME_LEN);

		IPACMDBG_H("Add Nat IfaceName: %s ,update nat-ifaces number: %d\n",
						 pNatIfaces[ipa_nat_iface_entries - 1].iface_name,
						 ipa_nat_iface_entries);
	}

	return 0;
}

int IPACM_Config::DelNatIfaces(char *dev_name)
{
	int i = 0;
	IPACMDBG_H("Del iface %s from NAT-ifaces, origin it has %d nat ifaces\n",
					 dev_name, ipa_nat_iface_entries);

	for (i = 0; i < ipa_nat_iface_entries; i++)
	{
		if (strcmp(dev_name, pNatIfaces[i].iface_name) == 0)
		{
			IPACMDBG_H("Find Nat IfaceName: %s ,previous nat-ifaces number: %d\n",
							 pNatIfaces[i].iface_name, ipa_nat_iface_entries);

			/* Reset the matched entry */
			memset(pNatIfaces[i].iface_name, 0, IPA_IFACE_NAME_LEN);

			for (; i < ipa_nat_iface_entries - 1; i++)
			{
				memcpy(pNatIfaces[i].iface_name,
							 pNatIfaces[i + 1].iface_name, IPA_IFACE_NAME_LEN);

				/* Reset the copied entry */
				memset(pNatIfaces[i + 1].iface_name, 0, IPA_IFACE_NAME_LEN);
			}
			ipa_nat_iface_entries--;
			IPACMDBG_H("Update nat-ifaces number: %d\n", ipa_nat_iface_entries);
			return 0;
		}
	}

	IPACMDBG_H("Can't find Nat IfaceName: %s with total nat-ifaces number: %d\n",
					    dev_name, ipa_nat_iface_entries);
	return 0;
}

/* for IPACM resource manager dependency usage
   add either Tx or Rx ipa_rm_resource_name and
   also indicate that endpoint property if valid */
void IPACM_Config::AddRmDepend(ipa_rm_resource_name rm1,bool rx_bypass_ipa)
{
	int retval = 0;
	struct ipa_ioc_rm_dependency dep;

	IPACMDBG_H(" Got rm add-depend index : %d \n", rm1);
	/* ipa_rm_a2_check: IPA_RM_RESOURCE_Q6_CONS*/
	if(rm1 == IPA_RM_RESOURCE_Q6_CONS)
	{
		ipa_rm_a2_check+=1;
		IPACMDBG_H("got %d times default RT routing from A2 \n", ipa_rm_a2_check);
	}

	for(int i=0;i<ipa_max_valid_rm_entry;i++)
	{
		if(rm1 == ipa_rm_tbl[i].producer_rm1)
		{
			ipa_rm_tbl[i].producer1_up = true;
			/* entry1's producer actually dun have registered Rx-property */
			ipa_rm_tbl[i].rx_bypass_ipa = rx_bypass_ipa;
			IPACMDBG_H("Matched RM_table entry: %d's producer_rm1 with non_rx_prop: %d \n", i,ipa_rm_tbl[i].rx_bypass_ipa);

			if(ipa_rm_tbl[i].consumer1_up == true && ipa_rm_tbl[i].rm_set == false)
			{
				IPACMDBG_H("SETUP RM_table entry %d's bi-direction dependency  \n", i);
				/* add bi-directional dependency*/
				if(ipa_rm_tbl[i].rx_bypass_ipa)
				{
					IPACMDBG_H("Skip ADD entry %d's dependency between WLAN-Pro: %d, Con: %d \n", i, ipa_rm_tbl[i].producer_rm1,ipa_rm_tbl[i].consumer_rm1);
				}
				else
				{
					memset(&dep, 0, sizeof(dep));
					dep.resource_name = ipa_rm_tbl[i].producer_rm1;
					dep.depends_on_name = ipa_rm_tbl[i].consumer_rm1;
					retval = ioctl(m_fd, IPA_IOC_RM_ADD_DEPENDENCY, &dep);
					IPACMDBG_H("ADD entry %d's dependency between Pro: %d, Con: %d \n", i,dep.resource_name,dep.depends_on_name);
					if (retval)
					{
						IPACMERR("Failed adding dependecny for RM_table entry %d's bi-direction dependency (error:%d) \n", i,retval);
					}
				}
				memset(&dep, 0, sizeof(dep));
				dep.resource_name = ipa_rm_tbl[i].producer_rm2;
				dep.depends_on_name = ipa_rm_tbl[i].consumer_rm2;
				retval = ioctl(m_fd, IPA_IOC_RM_ADD_DEPENDENCY, &dep);
				IPACMDBG_H("ADD entry %d's dependency between Pro: %d, Con: %d \n", i,dep.resource_name,dep.depends_on_name);
				if (retval)
				{
					IPACMERR("Failed adding dependecny for RM_table entry %d's bi-direction dependency (error:%d)  \n", i,retval);
				}
				ipa_rm_tbl[i].rm_set = true;
			}
			else
			{
				IPACMDBG_H("Not SETUP RM_table entry %d: prod_up:%d, cons_up:%d, rm_set: %d \n", i,ipa_rm_tbl[i].producer1_up, ipa_rm_tbl[i].consumer1_up, ipa_rm_tbl[i].rm_set);
			}
		}

		if(rm1 == ipa_rm_tbl[i].consumer_rm1)
		{
			ipa_rm_tbl[i].consumer1_up = true;
			IPACMDBG_H("Matched RM_table entry: %d's consumer_rm1 \n", i);

			if(ipa_rm_tbl[i].producer1_up == true && ipa_rm_tbl[i].rm_set == false)
			{
				IPACMDBG_H("SETUP RM_table entry %d's bi-direction dependency  \n", i);
				/* add bi-directional dependency*/
				if(ipa_rm_tbl[i].rx_bypass_ipa)
				{
					IPACMDBG_H("Skip ADD entry %d's dependency between WLAN-Pro: %d, Con: %d \n", i, ipa_rm_tbl[i].producer_rm1,ipa_rm_tbl[i].consumer_rm1);
				}
				else
				{
					memset(&dep, 0, sizeof(dep));
					dep.resource_name = ipa_rm_tbl[i].producer_rm1;
					dep.depends_on_name = ipa_rm_tbl[i].consumer_rm1;
					retval = ioctl(m_fd, IPA_IOC_RM_ADD_DEPENDENCY, &dep);
					IPACMDBG_H("ADD entry %d's dependency between Pro: %d, Con: %d \n", i,dep.resource_name,dep.depends_on_name);
					if (retval)
					{
						IPACMERR("Failed adding dependecny for RM_table entry %d's bi-direction dependency (error:%d)  \n", i,retval);
					}
				}

				memset(&dep, 0, sizeof(dep));
				dep.resource_name = ipa_rm_tbl[i].producer_rm2;
				dep.depends_on_name = ipa_rm_tbl[i].consumer_rm2;
				retval = ioctl(m_fd, IPA_IOC_RM_ADD_DEPENDENCY, &dep);
				IPACMDBG_H("ADD entry %d's dependency between Pro: %d, Con: %d \n", i,dep.resource_name,dep.depends_on_name);
				if (retval)
				{
					IPACMERR("Failed adding dependecny for RM_table entry %d's bi-direction dependency (error:%d)  \n", i,retval);
				}
				ipa_rm_tbl[i].rm_set = true;
			}
			else
			{
				IPACMDBG_H("Not SETUP RM_table entry %d: prod_up:%d, cons_up:%d, rm_set: %d \n", i,ipa_rm_tbl[i].producer1_up, ipa_rm_tbl[i].consumer1_up, ipa_rm_tbl[i].rm_set);
			}
	   }
   }
   return ;
}

/* for IPACM resource manager dependency usage
   delete either Tx or Rx ipa_rm_resource_name */

void IPACM_Config::DelRmDepend(ipa_rm_resource_name rm1)
{
	int retval = 0;
	struct ipa_ioc_rm_dependency dep;

	IPACMDBG_H(" Got rm del-depend index : %d \n", rm1);
	/* ipa_rm_a2_check: IPA_RM_RESOURCE_Q6_CONS*/
	if(rm1 == IPA_RM_RESOURCE_Q6_CONS)
	{
		ipa_rm_a2_check-=1;
		IPACMDBG_H("Left %d times default RT routing from A2 \n", ipa_rm_a2_check);
	}

	for(int i=0;i<ipa_max_valid_rm_entry;i++)
	{

		if(rm1 == ipa_rm_tbl[i].producer_rm1)
		{
			if(ipa_rm_tbl[i].rm_set == true)
			{
				IPACMDBG_H("Matched RM_table entry: %d's producer_rm1 and dependency is up \n", i);
				ipa_rm_tbl[i].rm_set = false;

				/* delete bi-directional dependency*/
				if(ipa_rm_tbl[i].rx_bypass_ipa)
				{
					IPACMDBG_H("Skip DEL entry %d's dependency between WLAN-Pro: %d, Con: %d \n", i, ipa_rm_tbl[i].producer_rm1,ipa_rm_tbl[i].consumer_rm1);
				}
				else
				{
					memset(&dep, 0, sizeof(dep));
					dep.resource_name = ipa_rm_tbl[i].producer_rm1;
					dep.depends_on_name = ipa_rm_tbl[i].consumer_rm1;
					retval = ioctl(m_fd, IPA_IOC_RM_DEL_DEPENDENCY, &dep);
					IPACMDBG_H("Delete entry %d's dependency between Pro: %d, Con: %d \n", i,dep.resource_name,dep.depends_on_name);
					if (retval)
					{
						IPACMERR("Failed deleting dependecny for RM_table entry %d's bi-direction dependency (error:%d) \n", i,retval);
					}
				}
				memset(&dep, 0, sizeof(dep));
				dep.resource_name = ipa_rm_tbl[i].producer_rm2;
				dep.depends_on_name = ipa_rm_tbl[i].consumer_rm2;
				retval = ioctl(m_fd, IPA_IOC_RM_DEL_DEPENDENCY, &dep);
				IPACMDBG_H("Delete entry %d's dependency between Pro: %d, Con: %d \n", i,dep.resource_name,dep.depends_on_name);
				if (retval)
				{
					IPACMERR("Failed deleting dependecny for RM_table entry %d's bi-direction dependency (error:%d) \n", i,retval);
				}
			}
			ipa_rm_tbl[i].producer1_up = false;
			ipa_rm_tbl[i].rx_bypass_ipa = false;
		}
		if(rm1 == ipa_rm_tbl[i].consumer_rm1)
		{
			/* ipa_rm_a2_check: IPA_RM_RESOURCE_!6_CONS*/
			if(ipa_rm_tbl[i].consumer_rm1 == IPA_RM_RESOURCE_Q6_CONS && ipa_rm_a2_check == 1)
			{
				IPACMDBG_H(" still have %d default RT routing from A2 \n", ipa_rm_a2_check);
				continue;
			}

			if(ipa_rm_tbl[i].rm_set == true)
			{
				IPACMDBG_H("Matched RM_table entry: %d's consumer_rm1 and dependency is up \n", i);
				ipa_rm_tbl[i].rm_set = false;
				/* delete bi-directional dependency*/
				if(ipa_rm_tbl[i].rx_bypass_ipa)
				{
					IPACMDBG_H("Skip DEL entry %d's dependency between WLAN-Pro: %d, Con: %d \n", i, ipa_rm_tbl[i].producer_rm1,ipa_rm_tbl[i].consumer_rm1);
				}
				else
				{
					memset(&dep, 0, sizeof(dep));
					dep.resource_name = ipa_rm_tbl[i].producer_rm1;
					dep.depends_on_name = ipa_rm_tbl[i].consumer_rm1;
					retval = ioctl(m_fd, IPA_IOC_RM_DEL_DEPENDENCY, &dep);
					IPACMDBG_H("Delete entry %d's dependency between Pro: %d, Con: %d \n", i,dep.resource_name,dep.depends_on_name);
					if (retval)
					{
						IPACMERR("Failed deleting dependecny for RM_table entry %d's bi-direction dependency (error:%d) \n", i,retval);
					}
				}

				memset(&dep, 0, sizeof(dep));
				dep.resource_name = ipa_rm_tbl[i].producer_rm2;
				dep.depends_on_name = ipa_rm_tbl[i].consumer_rm2;
				retval = ioctl(m_fd, IPA_IOC_RM_DEL_DEPENDENCY, &dep);
				IPACMDBG_H("Delete entry %d's dependency between Pro: %d, Con: %d \n", i,dep.resource_name,dep.depends_on_name);
				if (retval)
				{
					IPACMERR("Failed deleting dependecny for RM_table entry %d's bi-direction dependency (error:%d) \n", i,retval);
				}
			}
			ipa_rm_tbl[i].consumer1_up = false;
		}
	}
	return ;
}

int IPACM_Config::SetExtProp(ipa_ioc_query_intf_ext_props *prop)
{
	int i, num;

	if(prop == NULL || prop->num_ext_props <= 0)
	{
		IPACMERR("There is no extended property!\n");
		return IPACM_FAILURE;
	}

	num = prop->num_ext_props;
	for(i=0; i<num; i++)
	{
		if(prop->ext[i].ip == IPA_IP_v4)
		{
			if(ext_prop_v4.num_ext_props >= MAX_NUM_EXT_PROPS)
			{
				IPACMERR("IPv4 extended property table is full!\n");
				continue;
			}
			memcpy(&ext_prop_v4.prop[ext_prop_v4.num_ext_props], &prop->ext[i], sizeof(struct ipa_ioc_ext_intf_prop));
			ext_prop_v4.num_ext_props++;
		}
		else if(prop->ext[i].ip == IPA_IP_v6)
		{
			if(ext_prop_v6.num_ext_props >= MAX_NUM_EXT_PROPS)
			{
				IPACMERR("IPv6 extended property table is full!\n");
				continue;
			}
			memcpy(&ext_prop_v6.prop[ext_prop_v6.num_ext_props], &prop->ext[i], sizeof(struct ipa_ioc_ext_intf_prop));
			ext_prop_v6.num_ext_props++;
		}
		else
		{
			IPACMERR("The IP type is not expected!\n");
			return IPACM_FAILURE;
		}
	}

	IPACMDBG_H("Set extended property succeeded.\n");

	return IPACM_SUCCESS;
}

ipacm_ext_prop* IPACM_Config::GetExtProp(ipa_ip_type ip_type)
{
	if(ip_type == IPA_IP_v4)
		return &ext_prop_v4;
	else if(ip_type == IPA_IP_v6)
		return &ext_prop_v6;
	else
	{
		IPACMERR("Failed to get extended property: the IP version is neither IPv4 nor IPv6!\n");
		return NULL;
	}
}

int IPACM_Config::DelExtProp(ipa_ip_type ip_type)
{
	if(ip_type != IPA_IP_v6)
	{
		memset(&ext_prop_v4, 0, sizeof(ext_prop_v4));
	}

	if(ip_type != IPA_IP_v4)
	{
		memset(&ext_prop_v6, 0, sizeof(ext_prop_v6));
	}

	return IPACM_SUCCESS;
}

const char* IPACM_Config::getEventName(ipa_cm_event_id event_id)
{
	if(event_id >= sizeof(ipacm_event_name)/sizeof(ipacm_event_name[0]))
	{
		IPACMERR("Event name array is not consistent with event array!\n");
		return NULL;
	}

	return ipacm_event_name[event_id];
}
