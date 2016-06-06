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
/*!
@file
IPACM_Wlan.cpp

@brief
This file implements the WLAN iface functionality.

@Author
Skylar Chang

*/

#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <IPACM_Wlan.h>
#include <IPACM_Netlink.h>
#include <fcntl.h>
#include <sys/inotify.h>
#include <IPACM_Wan.h>
#include <IPACM_Lan.h>
#include <IPACM_IfaceManager.h>
#include <IPACM_ConntrackListener.h>


/* static member to store the number of total wifi clients within all APs*/
int IPACM_Wlan::total_num_wifi_clients = 0;

uint32_t* IPACM_Wlan::dummy_flt_rule_hdl_v4 = NULL;
uint32_t* IPACM_Wlan::dummy_flt_rule_hdl_v6 = NULL;
int IPACM_Wlan::num_wlan_ap_iface = 0;

lan2lan_flt_rule_hdl IPACM_Wlan::self_client_flt_rule_hdl_v4[IPA_LAN_TO_LAN_MAX_WLAN_CLIENT];
lan2lan_flt_rule_hdl IPACM_Wlan::self_client_flt_rule_hdl_v6[IPA_LAN_TO_LAN_MAX_WLAN_CLIENT];

lan2lan_flt_rule_hdl IPACM_Wlan::lan_client_flt_rule_hdl_v4[IPA_LAN_TO_LAN_MAX_LAN_CLIENT];
lan2lan_flt_rule_hdl IPACM_Wlan::lan_client_flt_rule_hdl_v6[IPA_LAN_TO_LAN_MAX_LAN_CLIENT];

IPACM_Wlan::IPACM_Wlan(int iface_index) : IPACM_Lan(iface_index)
{
#define WLAN_AMPDU_DEFAULT_FILTER_RULES 3

	wlan_ap_index = IPACM_Wlan::num_wlan_ap_iface;
	if(wlan_ap_index < 0 || wlan_ap_index > 1)
	{
		IPACMERR("Wlan_ap_index is not correct: %d, not creating instance.\n", wlan_ap_index);
		if (tx_prop != NULL)
		{
			free(tx_prop);
		}
		if (rx_prop != NULL)
		{
			free(rx_prop);
		}
		if (iface_query != NULL)
		{
			free(iface_query);
		}
		delete this;
		return;
	}

	num_wifi_client = 0;
	header_name_count = 0;
	wlan_client = NULL;

	if(iface_query != NULL)
	{
		wlan_client_len = (sizeof(ipa_wlan_client)) + (iface_query->num_tx_props * sizeof(wlan_client_rt_hdl));
		wlan_client = (ipa_wlan_client *)calloc(IPA_MAX_NUM_WIFI_CLIENTS, wlan_client_len);
		if (wlan_client == NULL)
		{
			IPACMERR("unable to allocate memory\n");
			return;
		}
		IPACMDBG_H("index:%d constructor: Tx properties:%d\n", iface_index, iface_query->num_tx_props);
	}
	Nat_App = NatApp::GetInstance();
	if (Nat_App == NULL)
	{
		IPACMERR("unable to get Nat App instance \n");
		return;
	}

#ifdef FEATURE_ETH_BRIDGE_LE
	exp_index_v4 = IPV4_DEFAULT_FILTERTING_RULES + IPACM_Iface::ipacmcfg->ipa_num_private_subnet
			+ IPA_LAN_TO_LAN_MAX_WLAN_CLIENT + IPA_LAN_TO_LAN_MAX_LAN_CLIENT + NUM_IPV4_ICMP_FLT_RULE;
	exp_index_v6 = IPV6_DEFAULT_FILTERTING_RULES + IPA_LAN_TO_LAN_MAX_WLAN_CLIENT + IPA_LAN_TO_LAN_MAX_LAN_CLIENT
			+ NUM_IPV6_PREFIX_FLT_RULE + NUM_IPV6_ICMP_FLT_RULE;
#else
#ifndef CT_OPT
	exp_index_v4 = 2*(IPV4_DEFAULT_FILTERTING_RULES + MAX_OFFLOAD_PAIR + IPACM_Iface::ipacmcfg->ipa_num_private_subnet) + NUM_IPV4_ICMP_FLT_RULE;
	exp_index_v6 = 2*(IPV6_DEFAULT_FILTERTING_RULES + MAX_OFFLOAD_PAIR) + NUM_IPV6_PREFIX_FLT_RULE + NUM_IPV6_ICMP_FLT_RULE;
#else
	exp_index_v4 = 2*(IPV4_DEFAULT_FILTERTING_RULES + NUM_TCP_CTL_FLT_RULE + MAX_OFFLOAD_PAIR + IPACM_Iface::ipacmcfg->ipa_num_private_subnet) + NUM_IPV4_ICMP_FLT_RULE;
	exp_index_v6 = 2*(IPV6_DEFAULT_FILTERTING_RULES + NUM_TCP_CTL_FLT_RULE + MAX_OFFLOAD_PAIR) + NUM_IPV6_PREFIX_FLT_RULE + NUM_IPV6_ICMP_FLT_RULE;
#endif
#ifdef FEATURE_IPA_ANDROID
	exp_index_v4 = exp_index_v4 + 2 * (IPA_MAX_PRIVATE_SUBNET_ENTRIES - IPACM_Iface::ipacmcfg->ipa_num_private_subnet);
#endif
#endif

	IPACM_Wlan::num_wlan_ap_iface++;
	IPACMDBG_H("Now the number of wlan AP iface is %d\n", IPACM_Wlan::num_wlan_ap_iface);
	add_dummy_flt_rule();

	is_guest_ap = false;

	memset(eth_bridge_lan_client_flt_info, 0, IPA_LAN_TO_LAN_MAX_LAN_CLIENT * sizeof(eth_bridge_client_flt_info));
	lan_client_flt_info_count = 0;
	eth_bridge_wlan_client_rt_from_lan_info_v4 = NULL;
	eth_bridge_wlan_client_rt_from_lan_info_v6 = NULL;
	eth_bridge_wlan_client_rt_from_wlan_info_v4 = NULL;
	eth_bridge_wlan_client_rt_from_wlan_info_v6 = NULL;
	if(tx_prop != NULL)
	{
#ifdef FEATURE_ETH_BRIDGE_LE
		client_rt_info_size_v4 = sizeof(eth_bridge_client_rt_info) + each_client_rt_rule_count_v4 * sizeof(uint32_t);
		eth_bridge_wlan_client_rt_from_lan_info_v4 = (eth_bridge_client_rt_info*)calloc(IPA_LAN_TO_LAN_MAX_WLAN_CLIENT, client_rt_info_size_v4);
		eth_bridge_wlan_client_rt_from_wlan_info_v4 = (eth_bridge_client_rt_info*)calloc(IPA_LAN_TO_LAN_MAX_WLAN_CLIENT, client_rt_info_size_v4);

		client_rt_info_size_v6 = sizeof(eth_bridge_client_rt_info) + each_client_rt_rule_count_v6 * sizeof(uint32_t);
		eth_bridge_wlan_client_rt_from_lan_info_v6 = (eth_bridge_client_rt_info*)calloc(IPA_LAN_TO_LAN_MAX_WLAN_CLIENT, client_rt_info_size_v6);
		eth_bridge_wlan_client_rt_from_wlan_info_v6 = (eth_bridge_client_rt_info*)calloc(IPA_LAN_TO_LAN_MAX_WLAN_CLIENT, client_rt_info_size_v6);
#endif
	}
	wlan_client_rt_from_lan_info_count_v4 = 0;
	wlan_client_rt_from_lan_info_count_v6 = 0;
	wlan_client_rt_from_wlan_info_count_v4 = 0;
	wlan_client_rt_from_wlan_info_count_v6 = 0;
#ifdef FEATURE_ETH_BRIDGE_LE
	if(iface_query != NULL)
	{
		if(IPACM_Iface::ipacmcfg->iface_table[ipa_if_num].if_cat == WLAN_IF && tx_prop != NULL)
		{
			if(IPACM_Lan::wlan_hdr_type != IPA_HDR_L2_NONE && tx_prop->tx[0].hdr_l2_type != IPACM_Lan::wlan_hdr_type)
			{
				IPACMERR("The WLAN header format is not consistent! Now header format is %d.\n", tx_prop->tx[0].hdr_l2_type);
			}
			else
			{
				if(wlan_ap_index == 0)
				{
					if(eth_bridge_get_hdr_template_hdl(&IPACM_Lan::wlan_hdr_template_hdl) == IPACM_FAILURE)
					{
						IPACMERR("Failed to setup wlan hdr template.\n");
					}
					else
					{
						IPACM_Lan::wlan_hdr_type = tx_prop->tx[0].hdr_l2_type;
						add_hdr_proc_ctx();
					}
				}
			}
			if (IPACM_Iface::ipacmcfg->iface_table[ipa_if_num].wlan_mode == INTERNET)
			{
				is_guest_ap = true;
			}
			IPACMDBG_H("%s: guest ap enable: %d \n",
				IPACM_Iface::ipacmcfg->iface_table[ipa_if_num].iface_name, is_guest_ap);
		}
	}
#endif

#ifdef FEATURE_IPA_ANDROID
	/* set the IPA-client pipe enum */
	if(IPACM_Iface::ipacmcfg->iface_table[ipa_if_num].if_cat == WLAN_IF)
	{
		handle_tethering_client(false, IPACM_CLIENT_WLAN);
	}
#endif
	return;
}


IPACM_Wlan::~IPACM_Wlan()
{
	IPACM_EvtDispatcher::deregistr(this);
	IPACM_IfaceManager::deregistr(this);
	return;
}

void IPACM_Wlan::event_callback(ipa_cm_event_id event, void *param)
{
	if(is_active == false && event != IPA_LAN_DELETE_SELF)
	{
		IPACMDBG_H("The interface is no longer active, return.\n");
		return;
	}

	int ipa_interface_index;
	int wlan_index;
	ipacm_ext_prop* ext_prop;
	ipacm_event_iface_up* data_wan;
	ipacm_event_iface_up_tehter* data_wan_tether;

	switch (event)
	{

	case IPA_WLAN_LINK_DOWN_EVENT:
		{
			ipacm_event_data_fid *data = (ipacm_event_data_fid *)param;
			ipa_interface_index = iface_ipa_index_query(data->if_index);
			if (ipa_interface_index == ipa_if_num)
			{
				IPACMDBG_H("Received IPA_WLAN_LINK_DOWN_EVENT\n");
				handle_down_evt();
				/* reset the AP-iface category to unknown */
				IPACM_Iface::ipacmcfg->iface_table[ipa_if_num].if_cat=UNKNOWN_IF;
				IPACM_Iface::ipacmcfg->DelNatIfaces(dev_name); // delete NAT-iface
				IPACM_Wlan::total_num_wifi_clients = (IPACM_Wlan::total_num_wifi_clients) - \
                                                                     (num_wifi_client);
				return;
			}
		}
		break;

	case IPA_PRIVATE_SUBNET_CHANGE_EVENT:
		{
			ipacm_event_data_fid *data = (ipacm_event_data_fid *)param;
			/* internel event: data->if_index is ipa_if_index */
			if (data->if_index == ipa_if_num)
			{
				IPACMDBG_H("Received IPA_PRIVATE_SUBNET_CHANGE_EVENT from itself posting, ignore\n");
				return;
			}
			else
			{
				IPACMDBG_H("Received IPA_PRIVATE_SUBNET_CHANGE_EVENT from other LAN iface \n");
#ifdef FEATURE_IPA_ANDROID
				handle_private_subnet_android(IPA_IP_v4);
#endif
				IPACMDBG_H(" delete old private subnet rules, use new sets \n");
				return;
			}
		}
		break;

	case IPA_LAN_DELETE_SELF:
	{
		ipacm_event_data_fid *data = (ipacm_event_data_fid *)param;
		if(data->if_index == ipa_if_num)
		{
			IPACM_Wlan::num_wlan_ap_iface--;
			IPACMDBG_H("Now the number of wlan AP iface is %d\n", IPACM_Wlan::num_wlan_ap_iface);
			del_dummy_flt_rule();

			IPACMDBG_H("Received IPA_LAN_DELETE_SELF event.\n");
			IPACMDBG_H("ipa_WLAN (%s):ipa_index (%d) instance close \n", IPACM_Iface::ipacmcfg->iface_table[ipa_if_num].iface_name, ipa_if_num);
			delete this;
		}
		break;
	}

	case IPA_ADDR_ADD_EVENT:
		{
			ipacm_event_data_addr *data = (ipacm_event_data_addr *)param;
			ipa_interface_index = iface_ipa_index_query(data->if_index);

			if ( (data->iptype == IPA_IP_v4 && data->ipv4_addr == 0) ||
					 (data->iptype == IPA_IP_v6 &&
						data->ipv6_addr[0] == 0 && data->ipv6_addr[1] == 0 &&
					  data->ipv6_addr[2] == 0 && data->ipv6_addr[3] == 0) )
			{
				IPACMDBG_H("Invalid address, ignore IPA_ADDR_ADD_EVENT event\n");
				return;
			}

			if (ipa_interface_index == ipa_if_num)
			{
				/* check v4 not setup before, v6 can have 2 iface ip */
				if( ((data->iptype != ip_type) && (ip_type != IPA_IP_MAX))
				    || ((data->iptype==IPA_IP_v6) && (num_dft_rt_v6!=MAX_DEFAULT_v6_ROUTE_RULES)))
				{
				  IPACMDBG_H("Got IPA_ADDR_ADD_EVENT ip-family:%d, v6 num %d: \n",data->iptype,num_dft_rt_v6);
					/* Post event to NAT */
					if (data->iptype == IPA_IP_v4)
					{
						ipacm_cmd_q_data evt_data;
						ipacm_event_iface_up *info;

						info = (ipacm_event_iface_up *)
							 malloc(sizeof(ipacm_event_iface_up));
						if (info == NULL)
						{
							IPACMERR("Unable to allocate memory\n");
							return;
						}

						memcpy(info->ifname, dev_name, IF_NAME_LEN);
						info->ipv4_addr = data->ipv4_addr;
						info->addr_mask = IPACM_Iface::ipacmcfg->private_subnet_table[0].subnet_mask;

						evt_data.event = IPA_HANDLE_WLAN_UP;
						evt_data.evt_data = (void *)info;

						/* Insert IPA_HANDLE_WLAN_UP to command queue */
						IPACMDBG_H("posting IPA_HANDLE_WLAN_UP for IPv4 with below information\n");
						IPACMDBG_H("IPv4 address:0x%x, IPv4 address mask:0x%x\n",
										 info->ipv4_addr, info->addr_mask);
						IPACM_EvtDispatcher::PostEvt(&evt_data);
					}
					if(handle_addr_evt(data) == IPACM_FAILURE)
					{
						return;
					}
					if ((data->iptype == IPA_IP_v4) && (wlan_ap_index == 0))
					{
						IPACM_Lan::install_ipv4_icmp_flt_rule();
					}
					if ((num_dft_rt_v6 == 1) && (data->iptype == IPA_IP_v6) && (wlan_ap_index == 0))
					{
						install_ipv6_icmp_flt_rule();
					}

#ifdef FEATURE_IPA_ANDROID
					add_dummy_private_subnet_flt_rule(data->iptype);
					handle_private_subnet_android(data->iptype);
#else
					if(wlan_ap_index == 0)
					{
						handle_private_subnet(data->iptype);
					}
#endif

					if (IPACM_Wan::isWanUP(ipa_if_num))
					{
						if(data->iptype == IPA_IP_v4 || data->iptype == IPA_IP_MAX)
						{
							if(IPACM_Wan::backhaul_is_sta_mode == false)
							{
								ext_prop = IPACM_Iface::ipacmcfg->GetExtProp(IPA_IP_v4);
								IPACM_Lan::handle_wan_up_ex(ext_prop, IPA_IP_v4,
												IPACM_Wan::getXlat_Mux_Id());
							}
							else
							{
								IPACM_Lan::handle_wan_up(IPA_IP_v4);
							}
						}
					}

					if(IPACM_Wan::isWanUP_V6(ipa_if_num))
					{
						if((data->iptype == IPA_IP_v6 || data->iptype == IPA_IP_MAX) && num_dft_rt_v6 == 1)
						{
							if(wlan_ap_index == 0) //install ipv6 prefix rule only once
							{
								install_ipv6_prefix_flt_rule(IPACM_Wan::backhaul_ipv6_prefix);
							}
							if(IPACM_Wan::backhaul_is_sta_mode == false)
							{
								ext_prop = IPACM_Iface::ipacmcfg->GetExtProp(IPA_IP_v6);
								IPACM_Lan::handle_wan_up_ex(ext_prop, IPA_IP_v6, 0);
							}
							else
							{
								IPACM_Lan::handle_wan_up(IPA_IP_v6);
							}
						}
					}

					IPACMDBG_H("posting IPA_HANDLE_WLAN_UP:Finished checking wan_up\n");
					/* checking if SW-RT_enable */
					if (IPACM_Iface::ipacmcfg->ipa_sw_rt_enable == true)
					{
						/* handle software routing enable event*/
						IPACMDBG_H("IPA_SW_ROUTING_ENABLE for iface: %s \n",IPACM_Iface::ipacmcfg->iface_table[ipa_if_num].iface_name);
						handle_software_routing_enable();
					}
				}
			}
		}
		break;
#ifdef FEATURE_IPA_ANDROID
	case IPA_HANDLE_WAN_UP_TETHER:
		IPACMDBG_H("Received IPA_HANDLE_WAN_UP_TETHER event\n");

		data_wan_tether = (ipacm_event_iface_up_tehter*)param;
		if(data_wan_tether == NULL)
		{
			IPACMERR("No event data is found.\n");
			return;
		}
		IPACMDBG_H("Backhaul is sta mode?%d, if_index_tether:%d tether_if_name:%s\n", data_wan_tether->is_sta,
					data_wan_tether->if_index_tether,
					IPACM_Iface::ipacmcfg->iface_table[data_wan_tether->if_index_tether].iface_name);
		if (data_wan_tether->if_index_tether == ipa_if_num)
		{
			if(ip_type == IPA_IP_v4 || ip_type == IPA_IP_MAX)
			{
				if(data_wan_tether->is_sta == false)
				{
					ext_prop = IPACM_Iface::ipacmcfg->GetExtProp(IPA_IP_v4);
					IPACM_Lan::handle_wan_up_ex(ext_prop, IPA_IP_v4, 0);
				}
				else
				{
					IPACM_Lan::handle_wan_up(IPA_IP_v4);
				}
			}
		}
		break;

	case IPA_HANDLE_WAN_UP_V6_TETHER:
		IPACMDBG_H("Received IPA_HANDLE_WAN_UP_V6_TETHER event\n");

		data_wan_tether = (ipacm_event_iface_up_tehter*)param;
		if(data_wan_tether == NULL)
		{
			IPACMERR("No event data is found.\n");
			return;
		}
		IPACMDBG_H("Backhaul is sta mode?%d, if_index_tether:%d tether_if_name:%s\n", data_wan_tether->is_sta,
					data_wan_tether->if_index_tether,
					IPACM_Iface::ipacmcfg->iface_table[data_wan_tether->if_index_tether].iface_name);
		if (data_wan_tether->if_index_tether == ipa_if_num)
		{
			if(ip_type == IPA_IP_v6 || ip_type == IPA_IP_MAX)
			{
				if(wlan_ap_index == 0) //install ipv6 prefix rule only once
				{
					install_ipv6_prefix_flt_rule(data_wan_tether->ipv6_prefix);
				}
				if(data_wan_tether->is_sta == false)
				{
					ext_prop = IPACM_Iface::ipacmcfg->GetExtProp(IPA_IP_v6);
					IPACM_Lan::handle_wan_up_ex(ext_prop, IPA_IP_v6, 0);
				}
				else
				{
					IPACM_Lan::handle_wan_up(IPA_IP_v6);
				}
			}
		}
		break;

	case IPA_HANDLE_WAN_DOWN_TETHER:
		IPACMDBG_H("Received IPA_HANDLE_WAN_DOWN_TETHER event\n");
		data_wan_tether = (ipacm_event_iface_up_tehter*)param;
		if(data_wan_tether == NULL)
		{
			IPACMERR("No event data is found.\n");
			return;
		}
		IPACMDBG_H("Backhaul is sta mode?%d, if_index_tether:%d tether_if_name:%s\n", data_wan_tether->is_sta,
					data_wan_tether->if_index_tether,
					IPACM_Iface::ipacmcfg->iface_table[data_wan_tether->if_index_tether].iface_name);
		if (data_wan_tether->if_index_tether == ipa_if_num)
		{
			if(data_wan_tether->is_sta == false && wlan_ap_index > 0)
			{
				IPACMDBG_H("This is not the first AP instance and not STA mode, ignore WAN_DOWN event.\n");
				return;
			}
			if (rx_prop != NULL)
			{
				if(ip_type == IPA_IP_v4 || ip_type == IPA_IP_MAX)
				{
					handle_wan_down(data_wan_tether->is_sta);
				}
			}
		}
		break;

	case IPA_HANDLE_WAN_DOWN_V6_TETHER:
		IPACMDBG_H("Received IPA_HANDLE_WAN_DOWN_V6_TETHER event\n");
		data_wan_tether = (ipacm_event_iface_up_tehter*)param;
		if(data_wan_tether == NULL)
		{
			IPACMERR("No event data is found.\n");
			return;
		}
		IPACMDBG_H("Backhaul is sta mode?%d, if_index_tether:%d tether_if_name:%s\n", data_wan_tether->is_sta,
					data_wan_tether->if_index_tether,
					IPACM_Iface::ipacmcfg->iface_table[data_wan_tether->if_index_tether].iface_name);
		if (data_wan_tether->if_index_tether == ipa_if_num)
		{
			/* clean up v6 RT rules*/
			IPACMDBG_H("Received IPA_WAN_V6_DOWN in WLAN-instance and need clean up client IPv6 address \n");
			/* reset wifi-client ipv6 rt-rules */
			handle_wlan_client_reset_rt(IPA_IP_v6);

			if(data_wan_tether->is_sta == false && wlan_ap_index > 0)
			{
				IPACMDBG_H("This is not the first AP instance and not STA mode, ignore WAN_DOWN event.\n");
				return;
			}
			if (rx_prop != NULL)
			{
				if(ip_type == IPA_IP_v6 || ip_type == IPA_IP_MAX)
				{
					handle_wan_down_v6(data_wan_tether->is_sta);
				}
			}
		}
		break;
#else
	case IPA_HANDLE_WAN_UP:
		IPACMDBG_H("Received IPA_HANDLE_WAN_UP event\n");

		data_wan = (ipacm_event_iface_up*)param;
		if(data_wan == NULL)
		{
			IPACMERR("No event data is found.\n");
			return;
		}
		IPACMDBG_H("Backhaul is sta mode?%d\n", data_wan->is_sta);
		if(ip_type == IPA_IP_v4 || ip_type == IPA_IP_MAX)
		{
			if(data_wan->is_sta == false)
			{
				ext_prop = IPACM_Iface::ipacmcfg->GetExtProp(IPA_IP_v4);
				IPACM_Lan::handle_wan_up_ex(ext_prop, IPA_IP_v4, data_wan->xlat_mux_id);
			}
			else
			{
				IPACM_Lan::handle_wan_up(IPA_IP_v4);
			}
		}
		break;

	case IPA_HANDLE_WAN_UP_V6:
		IPACMDBG_H("Received IPA_HANDLE_WAN_UP_V6 event\n");

		data_wan = (ipacm_event_iface_up*)param;
		if(data_wan == NULL)
		{
			IPACMERR("No event data is found.\n");
			return;
		}
		IPACMDBG_H("Backhaul is sta mode?%d\n", data_wan->is_sta);
		if(ip_type == IPA_IP_v6 || ip_type == IPA_IP_MAX)
		{
			if(wlan_ap_index == 0) //install ipv6 prefix rule only once
			{
				install_ipv6_prefix_flt_rule(data_wan->ipv6_prefix);
			}
			if(data_wan->is_sta == false)
			{
				ext_prop = IPACM_Iface::ipacmcfg->GetExtProp(IPA_IP_v6);
				IPACM_Lan::handle_wan_up_ex(ext_prop, IPA_IP_v6, 0);
			}
			else
			{
				IPACM_Lan::handle_wan_up(IPA_IP_v6);
			}
		}
		break;

	case IPA_HANDLE_WAN_DOWN:
		IPACMDBG_H("Received IPA_HANDLE_WAN_DOWN event\n");
		data_wan = (ipacm_event_iface_up*)param;
		if(data_wan == NULL)
		{
			IPACMERR("No event data is found.\n");
			return;
		}
		IPACMDBG_H("Backhaul is sta mode?%d\n", data_wan->is_sta);
		if(data_wan->is_sta == false && wlan_ap_index > 0)
		{
			IPACMDBG_H("This is not the first AP instance and not STA mode, ignore WAN_DOWN event.\n");
			return;
		}
		if (rx_prop != NULL)
		{
			if(ip_type == IPA_IP_v4 || ip_type == IPA_IP_MAX)
			{
				handle_wan_down(data_wan->is_sta);
			}
		}
		break;

	case IPA_HANDLE_WAN_DOWN_V6:
		IPACMDBG_H("Received IPA_HANDLE_WAN_DOWN_V6 event\n");
		data_wan = (ipacm_event_iface_up*)param;
		if(data_wan == NULL)
		{
			IPACMERR("No event data is found.\n");
			return;
		}
		/* clean up v6 RT rules*/
		IPACMDBG_H("Received IPA_WAN_V6_DOWN in WLAN-instance and need clean up client IPv6 address \n");
		/* reset wifi-client ipv6 rt-rules */
		handle_wlan_client_reset_rt(IPA_IP_v6);
		IPACMDBG_H("Backhaul is sta mode ? %d\n", data_wan->is_sta);
		if(data_wan->is_sta == false && wlan_ap_index > 0)
		{
			IPACMDBG_H("This is not the first AP instance and not STA mode, ignore WAN_DOWN event.\n");
			return;
		}
		if (rx_prop != NULL)
		{
			if(ip_type == IPA_IP_v6 || ip_type == IPA_IP_MAX)
			{
				handle_wan_down_v6(data_wan->is_sta);
			}
		}
		break;
#endif

	case IPA_WLAN_CLIENT_ADD_EVENT_EX:
		{
			ipacm_event_data_wlan_ex *data = (ipacm_event_data_wlan_ex *)param;
			ipa_interface_index = iface_ipa_index_query(data->if_index);
			if (ipa_interface_index == ipa_if_num)
			{
#ifdef FEATURE_ETH_BRIDGE_LE
				int i;
				for(i=0; i<data->num_of_attribs; i++)
				{
					if(data->attribs[i].attrib_type == WLAN_HDR_ATTRIB_MAC_ADDR)
					{
						if(IPACM_Lan::wlan_to_wlan_hdr_proc_ctx.valid == true)
						{
							eth_bridge_add_wlan_client_rt_rule(data->attribs[i].u.mac_addr, SRC_WLAN, IPA_IP_v4);
							eth_bridge_add_wlan_client_rt_rule(data->attribs[i].u.mac_addr, SRC_WLAN, IPA_IP_v6);
						}
						if(ip_type == IPA_IP_v4 || ip_type == IPA_IP_MAX)
						{
							eth_bridge_add_self_client_flt_rule(data->attribs[i].u.mac_addr, IPA_IP_v4);
						}
						if(ip_type == IPA_IP_v6 || ip_type == IPA_IP_MAX)
						{
							eth_bridge_add_self_client_flt_rule(data->attribs[i].u.mac_addr, IPA_IP_v6);
						}
						if (is_guest_ap == false)
						{
							if(IPACM_Lan::lan_to_wlan_hdr_proc_ctx.valid == true)
							{
								eth_bridge_add_wlan_client_rt_rule(data->attribs[i].u.mac_addr, SRC_LAN, IPA_IP_v4);
								eth_bridge_add_wlan_client_rt_rule(data->attribs[i].u.mac_addr, SRC_LAN, IPA_IP_v6);
							}
							eth_bridge_post_lan_client_event(data->attribs[i].u.mac_addr, IPA_ETH_BRIDGE_WLAN_CLIENT_ADD_EVENT);
						}
						eth_bridge_add_wlan_client(data->attribs[i].u.mac_addr, ipa_if_num);
						break;
					}
				}
#endif
				IPACMDBG_H("Received IPA_WLAN_CLIENT_ADD_EVENT\n");
				handle_wlan_client_init_ex(data);
			}
		}
		break;

	case IPA_WLAN_CLIENT_DEL_EVENT:
		{
			ipacm_event_data_mac *data = (ipacm_event_data_mac *)param;
			ipa_interface_index = iface_ipa_index_query(data->if_index);
			if (ipa_interface_index == ipa_if_num)
			{
				IPACMDBG_H("Received IPA_WLAN_CLIENT_DEL_EVENT\n");
#ifdef FEATURE_ETH_BRIDGE_LE
				eth_bridge_del_self_client_flt_rule(data->mac_addr);
				eth_bridge_del_wlan_client_rt_rule(data->mac_addr, SRC_WLAN);
				if (is_guest_ap == false)
				{
					if(IPACM_Lan::lan_to_wlan_hdr_proc_ctx.valid == true)
					{
						eth_bridge_del_wlan_client_rt_rule(data->mac_addr, SRC_LAN);
					}
					eth_bridge_post_lan_client_event(data->mac_addr, IPA_ETH_BRIDGE_WLAN_CLIENT_DEL_EVENT);
				}
				eth_bridge_del_wlan_client(data->mac_addr);
#endif
				/* support lan2lan ipa-HW feature*/
				handle_lan2lan_msg_post(data->mac_addr, IPA_LAN_CLIENT_DISCONNECT, IPA_IP_v4);
				handle_lan2lan_msg_post(data->mac_addr, IPA_LAN_CLIENT_DISCONNECT, IPA_IP_v6);
				handle_wlan_client_down_evt(data->mac_addr);
			}
		}
		break;

	case IPA_WLAN_CLIENT_POWER_SAVE_EVENT:
		{
			ipacm_event_data_mac *data = (ipacm_event_data_mac *)param;
			ipa_interface_index = iface_ipa_index_query(data->if_index);
			if (ipa_interface_index == ipa_if_num)
			{
				IPACMDBG_H("Received IPA_WLAN_CLIENT_POWER_SAVE_EVENT\n");
				/* support lan2lan ipa-HW feature*/
				handle_lan2lan_msg_post(data->mac_addr, IPA_LAN_CLIENT_POWER_SAVE, IPA_IP_v4);
				handle_lan2lan_msg_post(data->mac_addr, IPA_LAN_CLIENT_POWER_SAVE, IPA_IP_v6);
				handle_wlan_client_pwrsave(data->mac_addr);
			}
		}
		break;

	case IPA_WLAN_CLIENT_RECOVER_EVENT:
		{
			ipacm_event_data_mac *data = (ipacm_event_data_mac *)param;
			ipa_interface_index = iface_ipa_index_query(data->if_index);
			if (ipa_interface_index == ipa_if_num)
			{
				IPACMDBG_H("Received IPA_WLAN_CLIENT_RECOVER_EVENT\n");
				/* support lan2lan ipa-HW feature*/
				handle_lan2lan_msg_post(data->mac_addr, IPA_LAN_CLIENT_POWER_RECOVER, IPA_IP_v4);
				handle_lan2lan_msg_post(data->mac_addr, IPA_LAN_CLIENT_POWER_RECOVER, IPA_IP_v6);

				wlan_index = get_wlan_client_index(data->mac_addr);
				if ((wlan_index != IPACM_INVALID_INDEX) &&
						(get_client_memptr(wlan_client, wlan_index)->power_save_set == true))
				{

					IPACMDBG_H("change wlan client out of  power safe mode \n");
					get_client_memptr(wlan_client, wlan_index)->power_save_set = false;

					/* First add route rules and then nat rules */
					if(get_client_memptr(wlan_client, wlan_index)->ipv4_set == true) /* for ipv4 */
					{
						     IPACMDBG_H("recover client index(%d):ipv4 address: 0x%x\n",
										 wlan_index,
										 get_client_memptr(wlan_client, wlan_index)->v4_addr);

						IPACMDBG_H("Adding Route Rules\n");
						handle_wlan_client_route_rule(data->mac_addr, IPA_IP_v4);
						IPACMDBG_H("Adding Nat Rules\n");
						Nat_App->ResetPwrSaveIf(get_client_memptr(wlan_client, wlan_index)->v4_addr);
					}

					if(get_client_memptr(wlan_client, wlan_index)->ipv6_set != 0) /* for ipv6 */
					{
						handle_wlan_client_route_rule(data->mac_addr, IPA_IP_v6);
					}
				}
			}
		}
		break;

	case IPA_NEIGH_CLIENT_IP_ADDR_ADD_EVENT:
		{
			ipacm_event_data_all *data = (ipacm_event_data_all *)param;
			ipa_interface_index = iface_ipa_index_query(data->if_index);
			if (ipa_interface_index == ipa_if_num)
			{
				IPACMDBG_H("Received IPA_NEIGH_CLIENT_IP_ADDR_ADD_EVENT\n");
				if (handle_wlan_client_ipaddr(data) == IPACM_FAILURE)
				{
					return;
				}
				/* support lan2lan ipa-hw feature */
				handle_lan2lan_client_active(data, IPA_LAN_CLIENT_ACTIVE);

				handle_wlan_client_route_rule(data->mac_addr, data->iptype);
				if (data->iptype == IPA_IP_v4)
				{
					/* Add NAT rules after ipv4 RT rules are set */
					CtList->HandleNeighIpAddrAddEvt(data);
					//Nat_App->ResetPwrSaveIf(data->ipv4_addr);
				}
			}
		}
		break;

		/* handle software routing enable event, iface will update softwarerouting_act to true*/
	case IPA_SW_ROUTING_ENABLE:
		IPACMDBG_H("Received IPA_SW_ROUTING_ENABLE\n");
		IPACM_Iface::handle_software_routing_enable();
		break;

		/* handle software routing disable event, iface will update softwarerouting_act to false*/
	case IPA_SW_ROUTING_DISABLE:
		IPACMDBG_H("Received IPA_SW_ROUTING_DISABLE\n");
		IPACM_Iface::handle_software_routing_disable();
		break;

	case IPA_ETH_BRIDGE_LAN_CLIENT_ADD_EVENT:
		{
			IPACMDBG_H("Received IPA_ETH_BRIDGE_LAN_CLIENT_ADD_EVENT event.\n");
			ipacm_event_data_mac* mac = (ipacm_event_data_mac*)param;
			if(mac != NULL)
			{
				if(ip_type == IPA_IP_v4 || ip_type == IPA_IP_MAX)
				{
					eth_bridge_add_lan_client_flt_rule(mac->mac_addr, IPA_IP_v4);
				}
				if(ip_type == IPA_IP_v6 || ip_type == IPA_IP_MAX)
				{
					eth_bridge_add_lan_client_flt_rule(mac->mac_addr, IPA_IP_v6);
				}
			}
			else
			{
				IPACMERR("Event MAC is empty.\n");
			}
		}
		break;

	case IPA_ETH_BRIDGE_LAN_CLIENT_DEL_EVENT:
		{
			IPACMDBG_H("Received IPA_ETH_BRIDGE_LAN_CLIENT_DEL_EVENT event.\n");
			ipacm_event_data_mac* mac = (ipacm_event_data_mac*)param;
			if(mac != NULL)
			{
				if(wlan_ap_index == 0)
				{
					if(eth_bridge_del_lan_client_flt_rule(mac->mac_addr) == IPACM_FAILURE)
					{
						IPACMDBG_H("Failed to delete lan client MAC based flt rule.\n");
					}
				}
			}
			else
			{
				IPACMERR("Event MAC is empty.\n");
			}
		}
		break;

	case IPA_ETH_BRIDGE_HDR_PROC_CTX_SET_EVENT:
	{
		IPACMDBG_H("Received IPA_ETH_BRIDGE_HDR_PROC_CTX_SET_EVENT event.\n");
		int i;
		ipacm_event_data_if_cat* cat = (ipacm_event_data_if_cat*)param;
		if(cat == NULL)
		{
			IPACMERR("Event data is empty.\n");
			return;
		}
		if(cat->if_cat != LAN_IF && cat->if_cat != ODU_IF)
		{
			IPACMDBG_H("The event was not sent by LAN interface, ignore.\n");
			return;
		}
		if (IPACM_Lan::is_usb_up == true && IPACM_Lan::is_cpe_up == true)
		{
			IPACMDBG_H("USB and CPE both are up, lan-wlan routing rules are already installed. \n");
			return;
		}
		for(i=0; i<IPACM_Lan::num_wlan_client; i++)
		{
			if(IPACM_Lan::eth_bridge_wlan_client[i].ipa_if_num == ipa_if_num)
			{
				eth_bridge_add_wlan_client_rt_rule(IPACM_Lan::eth_bridge_wlan_client[i].mac, SRC_LAN, IPA_IP_v4);
				eth_bridge_add_wlan_client_rt_rule(IPACM_Lan::eth_bridge_wlan_client[i].mac, SRC_LAN, IPA_IP_v6);
			}
		}
	}
	break;

	case IPA_ETH_BRIDGE_HDR_PROC_CTX_UNSET_EVENT:
	{
		IPACMDBG_H("Received IPA_ETH_BRIDGE_HDR_PROC_CTX_UNSET_EVENT event.\n");
		int i;
		ipacm_event_data_if_cat* cat = (ipacm_event_data_if_cat*)param;
		if(cat == NULL)
		{
			IPACMERR("Event data is empty.\n");
			return;
		}
		if(cat->if_cat != LAN_IF && cat->if_cat != ODU_IF)
		{
			IPACMDBG_H("The event was not sent by LAN interface, ignore.\n");
			return;
		}
		if (IPACM_Lan::is_usb_up == true || IPACM_Lan::is_cpe_up == true)
		{
			IPACMDBG_H("USB or CPE is still up, so keep lan-wlan routing rule. \n");
			return;
		}
		for(i=0; i<IPACM_Lan::num_wlan_client; i++)
		{
			if(IPACM_Lan::eth_bridge_wlan_client[i].ipa_if_num == ipa_if_num)
			{
				eth_bridge_del_wlan_client_rt_rule(IPACM_Lan::eth_bridge_wlan_client[i].mac, SRC_LAN);
			}
		}
	}
	break;

	case IPA_WLAN_SWITCH_TO_SCC:
		IPACMDBG_H("Received IPA_WLAN_SWITCH_TO_SCC\n");
		if(ip_type == IPA_IP_MAX)
		{
			handle_SCC_MCC_switch(IPA_IP_v4);
			handle_SCC_MCC_switch(IPA_IP_v6);
			eth_bridge_handle_wlan_SCC_MCC_switch(IPA_IP_v4);
			eth_bridge_handle_wlan_SCC_MCC_switch(IPA_IP_v6);
		}
		else
		{
			handle_SCC_MCC_switch(ip_type);
			eth_bridge_handle_wlan_SCC_MCC_switch(ip_type);
		}
		break;

	case IPA_WLAN_SWITCH_TO_MCC:
		IPACMDBG_H("Received IPA_WLAN_SWITCH_TO_MCC\n");
		if(ip_type == IPA_IP_MAX)
		{
			handle_SCC_MCC_switch(IPA_IP_v4);
			handle_SCC_MCC_switch(IPA_IP_v6);
			eth_bridge_handle_wlan_SCC_MCC_switch(IPA_IP_v4);
			eth_bridge_handle_wlan_SCC_MCC_switch(IPA_IP_v6);
		}
		else
		{
			handle_SCC_MCC_switch(ip_type);
			eth_bridge_handle_wlan_SCC_MCC_switch(ip_type);
		}
		break;

	case IPA_CRADLE_WAN_MODE_SWITCH:
	{
		IPACMDBG_H("Received IPA_CRADLE_WAN_MODE_SWITCH event.\n");
		ipacm_event_cradle_wan_mode* wan_mode = (ipacm_event_cradle_wan_mode*)param;
		if(wan_mode == NULL)
		{
			IPACMERR("Event data is empty.\n");
			return;
		}

		if(wan_mode->cradle_wan_mode == BRIDGE)
		{
			handle_cradle_wan_mode_switch(true);
		}
		else
		{
			handle_cradle_wan_mode_switch(false);
		}
	}
	break;
	case IPA_CFG_CHANGE_EVENT:
	{
		int i;
		IPACMDBG_H("Received IPA_CFG_CHANGE_EVENT event for %s with new wlan-mode: %s old wlan-mode: %s",
				IPACM_Iface::ipacmcfg->iface_table[ipa_if_num].iface_name,
				(IPACM_Iface::ipacmcfg->iface_table[ipa_if_num].wlan_mode == 0) ? "full" : "internet",
				(is_guest_ap == true) ? "internet" : "full");
		/* Add Natting iface to IPACM_Config if there is  Rx/Tx property */
		if (rx_prop != NULL || tx_prop != NULL)
		{
			IPACMDBG_H(" Has rx/tx properties registered for iface %s, add for NATTING \n", dev_name);
			IPACM_Iface::ipacmcfg->AddNatIfaces(dev_name);
		}

		if (is_guest_ap == true && (IPACM_Iface::ipacmcfg->iface_table[ipa_if_num].wlan_mode == FULL))
		{
			is_guest_ap = false;
			IPACMDBG_H("wlan mode is switched to full access mode. \n");
			eth_bridge_handle_wlan_mode_switch();
		}
		else if (is_guest_ap == false && (IPACM_Iface::ipacmcfg->iface_table[ipa_if_num].wlan_mode == INTERNET))
		{
			is_guest_ap = true;
			IPACMDBG_H("wlan mode is switched to internet only access mode. \n");
			eth_bridge_handle_wlan_mode_switch();
		}
		else
		{
			IPACMDBG_H("No change in %s access mode. \n",
					IPACM_Iface::ipacmcfg->iface_table[ipa_if_num].iface_name);

			/* Handle the WLAN filtering rule */
			for (i=0; i<IPACM_Lan::num_wlan_client; i++)
			{
				if (IPACM_Lan::eth_bridge_wlan_client[i].ipa_if_num == ipa_if_num)
				{
					eth_bridge_modify_wlan_client_flt_rule(IPACM_Lan::eth_bridge_wlan_client[i].mac, DST_WLAN, IPA_IP_v4);
					eth_bridge_modify_wlan_client_flt_rule(IPACM_Lan::eth_bridge_wlan_client[i].mac, DST_WLAN, IPA_IP_v6);
				}
			}
			/* Handle the LAN filtering rule */
			if (wlan_ap_index == 0 && is_guest_ap == false)
			{
				IPACMDBG_H("Modify LAN clients filtering rules. \n");
				for (i=0; i<IPACM_Lan::num_lan_client; i++)
				{
					eth_bridge_modify_wlan_client_flt_rule(IPACM_Lan::eth_bridge_lan_client[i].mac, DST_LAN, IPA_IP_v4);
					eth_bridge_modify_wlan_client_flt_rule(IPACM_Lan::eth_bridge_lan_client[i].mac, DST_LAN, IPA_IP_v6);
				}
			}
			IPACMDBG_H("wlan access mode switch is successful. \n");
		}
	}
	break;
	case IPA_TETHERING_STATS_UPDATE_EVENT:
	{
		IPACMDBG_H("Received IPA_TETHERING_STATS_UPDATE_EVENT event.\n");
		if (IPACM_Wan::isWanUP(ipa_if_num) || IPACM_Wan::isWanUP_V6(ipa_if_num))
		{
			if(IPACM_Wan::backhaul_is_sta_mode == false) /* LTE */
			{
				ipa_get_data_stats_resp_msg_v01 *data = (ipa_get_data_stats_resp_msg_v01 *)param;
				if (data->ipa_stats_type != QMI_IPA_STATS_TYPE_PIPE_V01)
				{
					IPACMERR("not valid pipe stats\n");
					return;
				}
				handle_tethering_stats_event(data);
			};
		}
	}
	break;
	default:
		break;
	}
	return;
}

/*Configure the initial filter rules */
int IPACM_Wlan::init_fl_rule(ipa_ip_type iptype)
{
	int res = IPACM_SUCCESS, len, offset;
	struct ipa_flt_rule_mdfy flt_rule;
	struct ipa_ioc_mdfy_flt_rule* pFilteringTable;

	/* update the iface ip-type to be IPA_IP_v4, IPA_IP_v6 or both*/
	if (iptype == IPA_IP_v4)
	{
		if ((ip_type == IPA_IP_v4) || (ip_type == IPA_IP_MAX))
		{
			IPACMDBG_H("Interface(%s:%d) already in ip-type %d\n", dev_name, ipa_if_num, ip_type);
			return res;
		}

		if (ip_type == IPA_IP_v6)
		{
			ip_type = IPA_IP_MAX;
		}
		else
		{
			ip_type = IPA_IP_v4;
		}
		IPACMDBG_H("Interface(%s:%d) now ip-type is %d\n", dev_name, ipa_if_num, ip_type);
	}
	else
	{
		if ((ip_type == IPA_IP_v6) || (ip_type == IPA_IP_MAX))
		{
			IPACMDBG_H("Interface(%s:%d) already in ip-type %d\n", dev_name, ipa_if_num, ip_type);
			return res;
		}

		if (ip_type == IPA_IP_v4)
		{
			ip_type = IPA_IP_MAX;
		}
		else
		{
			ip_type = IPA_IP_v6;
		}

		IPACMDBG_H("Interface(%s:%d) now ip-type is %d\n", dev_name, ipa_if_num, ip_type);
	}

    /* ADD corresponding ipa_rm_resource_name of RX-endpoint before adding all IPV4V6 FT-rules */
	if(rx_prop != NULL)
	{
		IPACMDBG_H("dev %s add producer dependency\n", dev_name);
		IPACMDBG_H("depend Got pipe %d rm index : %d \n", rx_prop->rx[0].src_pipe, IPACM_Iface::ipacmcfg->ipa_client_rm_map_tbl[rx_prop->rx[0].src_pipe]);
		IPACM_Iface::ipacmcfg->AddRmDepend(IPACM_Iface::ipacmcfg->ipa_client_rm_map_tbl[rx_prop->rx[0].src_pipe],false);
		IPACMDBG_H("Add producer dependency from %s with registered rx-prop\n", dev_name);
	}
	else
	{
		/* Adding the check if no Rx property registered, no filter rules will be added */
		IPACMDBG_H("No rx properties registered for iface %s\n", dev_name);
		return IPACM_SUCCESS;
	}
#ifdef FEATURE_ETH_BRIDGE_LE
	if(wlan_ap_index != 0)
	{
		IPACMDBG_H("Install frag/multicast/broadcast rules only for the first AP.\n");
		return IPACM_SUCCESS;
	}
#endif

	/* construct ipa_ioc_add_flt_rule with default filter rules */
	if (iptype == IPA_IP_v4)
	{
		if(IPACM_Wlan::dummy_flt_rule_hdl_v4 == NULL)
		{
			IPACMERR("Dummy ipv4 flt rule has not been installed.\n");
			return IPACM_FAILURE;
		}
#ifdef FEATURE_ETH_BRIDGE_LE
		offset = 0;
#else
#ifndef CT_OPT
		offset = wlan_ap_index * (IPV4_DEFAULT_FILTERTING_RULES + MAX_OFFLOAD_PAIR + IPACM_Iface::ipacmcfg->ipa_num_private_subnet);
#else
		offset = wlan_ap_index * (IPV4_DEFAULT_FILTERTING_RULES + NUM_TCP_CTL_FLT_RULE + MAX_OFFLOAD_PAIR + IPACM_Iface::ipacmcfg->ipa_num_private_subnet)
				+ NUM_TCP_CTL_FLT_RULE;
#endif
#endif

#ifdef FEATURE_IPA_ANDROID
		offset = offset + wlan_ap_index * (IPA_MAX_PRIVATE_SUBNET_ENTRIES - IPACM_Iface::ipacmcfg->ipa_num_private_subnet);
#endif
		len = sizeof(struct ipa_ioc_mdfy_flt_rule) + (IPV4_DEFAULT_FILTERTING_RULES * sizeof(struct ipa_flt_rule_mdfy));
		pFilteringTable = (struct ipa_ioc_mdfy_flt_rule *)calloc(1, len);
		if (!pFilteringTable)
		{
			IPACMERR("Error Locate ipa_ioc_mdfy_flt_rule memory...\n");
			return IPACM_FAILURE;
		}
		memset(pFilteringTable, 0, len);

		pFilteringTable->commit = 1;
		pFilteringTable->ip = iptype;
		pFilteringTable->num_rules = (uint8_t)IPV4_DEFAULT_FILTERTING_RULES;

		memset(&flt_rule, 0, sizeof(struct ipa_flt_rule_mdfy));

		flt_rule.status = -1;

		flt_rule.rule.retain_hdr = 1;
		flt_rule.rule.to_uc = 0;
		flt_rule.rule.action = IPA_PASS_TO_EXCEPTION;
		flt_rule.rule.eq_attrib_type = 0;

		/* Configuring Fragment Filtering Rule */
		IPACMDBG_H("rx property attrib mask:0x%x\n", rx_prop->rx[0].attrib.attrib_mask);
		memcpy(&flt_rule.rule.attrib, &rx_prop->rx[0].attrib, sizeof(flt_rule.rule.attrib));
#ifdef FEATURE_ETH_BRIDGE_LE
		/* remove meta data mask */
		flt_rule.rule.attrib.attrib_mask &= ~((uint32_t)IPA_FLT_META_DATA);
#endif
		flt_rule.rule.attrib.attrib_mask |= IPA_FLT_FRAGMENT;
		flt_rule.rule_hdl = IPACM_Wlan::dummy_flt_rule_hdl_v4[offset];
		memcpy(&(pFilteringTable->rules[0]), &flt_rule, sizeof(struct ipa_flt_rule_mdfy));

		/* Configuring Multicast Filtering Rule */
		memcpy(&flt_rule.rule.attrib, &rx_prop->rx[0].attrib, sizeof(flt_rule.rule.attrib));
#ifdef FEATURE_ETH_BRIDGE_LE
		/* remove meta data mask */
		flt_rule.rule.attrib.attrib_mask &= ~((uint32_t)IPA_FLT_META_DATA);
#endif
		flt_rule.rule.attrib.attrib_mask |= IPA_FLT_DST_ADDR;
		flt_rule.rule.attrib.u.v4.dst_addr_mask = 0xF0000000;
		flt_rule.rule.attrib.u.v4.dst_addr = 0xE0000000;
		flt_rule.rule_hdl = IPACM_Wlan::dummy_flt_rule_hdl_v4[offset+1];
		memcpy(&(pFilteringTable->rules[1]), &flt_rule, sizeof(struct ipa_flt_rule_mdfy));

		/* Configuring Broadcast Filtering Rule */
		flt_rule.rule.attrib.u.v4.dst_addr_mask = 0xFFFFFFFF;
		flt_rule.rule.attrib.u.v4.dst_addr = 0xFFFFFFFF;
		flt_rule.rule_hdl = IPACM_Wlan::dummy_flt_rule_hdl_v4[offset+2];
		memcpy(&(pFilteringTable->rules[2]), &flt_rule, sizeof(struct ipa_flt_rule_mdfy));

		if (false == m_filtering.ModifyFilteringRule(pFilteringTable))
		{
			IPACMERR("Failed to modify default ipv4 filtering rules.\n");
			res = IPACM_FAILURE;
			goto fail;
		}
		else
		{
			/* copy filter hdls */
			for (int i = 0; i < IPV4_DEFAULT_FILTERTING_RULES; i++)
			{
				if (pFilteringTable->rules[i].status == 0)
				{
					dft_v4fl_rule_hdl[i] = pFilteringTable->rules[i].rule_hdl;
					IPACMDBG_H("Default v4 filter Rule %d HDL:0x%x\n", i, dft_v4fl_rule_hdl[i]);
				}
				else
				{
					IPACMERR("Failed adding default v4 Filtering rule %d\n", i);
				}
			}
		}
	}
	else
	{
		if(IPACM_Wlan::dummy_flt_rule_hdl_v6 == NULL)
		{
			IPACMERR("Dummy ipv6 flt rule has not been installed.\n");
			return IPACM_FAILURE;
		}
#ifdef FEATURE_ETH_BRIDGE_LE
		offset = IPA_LAN_TO_LAN_MAX_WLAN_CLIENT + IPA_LAN_TO_LAN_MAX_LAN_CLIENT;
#else
#ifndef CT_OPT
		offset = wlan_ap_index * (IPV6_DEFAULT_FILTERTING_RULES + MAX_OFFLOAD_PAIR) + MAX_OFFLOAD_PAIR;
#else
		offset = wlan_ap_index * (IPV6_DEFAULT_FILTERTING_RULES + NUM_TCP_CTL_FLT_RULE + MAX_OFFLOAD_PAIR)
				+ NUM_TCP_CTL_FLT_RULE + MAX_OFFLOAD_PAIR;
#endif
#endif

		len = sizeof(struct ipa_ioc_mdfy_flt_rule) + (IPV6_DEFAULT_FILTERTING_RULES * sizeof(struct ipa_flt_rule_mdfy));
		pFilteringTable = (struct ipa_ioc_mdfy_flt_rule *)calloc(1, len);
		if (!pFilteringTable)
		{
			IPACMERR("Error Locate ipa_ioc_mdfy_flt_rule memory...\n");
			return IPACM_FAILURE;
		}
		memset(pFilteringTable, 0, len);

		pFilteringTable->commit = 1;
		pFilteringTable->ip = iptype;
		pFilteringTable->num_rules = (uint8_t)IPV6_DEFAULT_FILTERTING_RULES;

		memset(&flt_rule, 0, sizeof(struct ipa_flt_rule_mdfy));

		flt_rule.status = -1;

		flt_rule.rule.retain_hdr = 1;
		flt_rule.rule.to_uc = 0;
		flt_rule.rule.action = IPA_PASS_TO_EXCEPTION;
		flt_rule.rule.eq_attrib_type = 0;

		/* Configuring Multicast Filtering Rule */
		memcpy(&flt_rule.rule.attrib, &rx_prop->rx[0].attrib, sizeof(flt_rule.rule.attrib));
#ifdef FEATURE_ETH_BRIDGE_LE
		/* remove meta data mask */
		flt_rule.rule.attrib.attrib_mask &= ~((uint32_t)IPA_FLT_META_DATA);
#endif
		flt_rule.rule.attrib.attrib_mask |= IPA_FLT_DST_ADDR;
		flt_rule.rule.attrib.u.v6.dst_addr_mask[0] = 0xFF000000;
		flt_rule.rule.attrib.u.v6.dst_addr_mask[1] = 0x00000000;
		flt_rule.rule.attrib.u.v6.dst_addr_mask[2] = 0x00000000;
		flt_rule.rule.attrib.u.v6.dst_addr_mask[3] = 0x00000000;
		flt_rule.rule.attrib.u.v6.dst_addr[0] = 0XFF000000;
		flt_rule.rule.attrib.u.v6.dst_addr[1] = 0x00000000;
		flt_rule.rule.attrib.u.v6.dst_addr[2] = 0x00000000;
		flt_rule.rule.attrib.u.v6.dst_addr[3] = 0X00000000;
		flt_rule.rule_hdl = IPACM_Wlan::dummy_flt_rule_hdl_v6[offset];
		memcpy(&(pFilteringTable->rules[0]), &flt_rule, sizeof(struct ipa_flt_rule_mdfy));

		/* Configuring fe80::/10 Link-Scoped Unicast Filtering Rule */
		flt_rule.rule.attrib.u.v6.dst_addr_mask[0] = 0XFFC00000;
		flt_rule.rule.attrib.u.v6.dst_addr_mask[1] = 0x00000000;
		flt_rule.rule.attrib.u.v6.dst_addr_mask[2] = 0x00000000;
		flt_rule.rule.attrib.u.v6.dst_addr_mask[3] = 0x00000000;
		flt_rule.rule.attrib.u.v6.dst_addr[0] = 0xFE800000;
		flt_rule.rule.attrib.u.v6.dst_addr[1] = 0x00000000;
		flt_rule.rule.attrib.u.v6.dst_addr[2] = 0x00000000;
		flt_rule.rule.attrib.u.v6.dst_addr[3] = 0X00000000;
		flt_rule.rule_hdl = IPACM_Wlan::dummy_flt_rule_hdl_v6[offset+1];
		memcpy(&(pFilteringTable->rules[1]), &flt_rule, sizeof(struct ipa_flt_rule_mdfy));

		/* Configuring fec0::/10 Reserved by IETF Filtering Rule */
		flt_rule.rule.attrib.u.v6.dst_addr_mask[0] = 0XFFC00000;
		flt_rule.rule.attrib.u.v6.dst_addr_mask[1] = 0x00000000;
		flt_rule.rule.attrib.u.v6.dst_addr_mask[2] = 0x00000000;
		flt_rule.rule.attrib.u.v6.dst_addr_mask[3] = 0x00000000;
		flt_rule.rule.attrib.u.v6.dst_addr[0] = 0xFEC00000;
		flt_rule.rule.attrib.u.v6.dst_addr[1] = 0x00000000;
		flt_rule.rule.attrib.u.v6.dst_addr[2] = 0x00000000;
		flt_rule.rule.attrib.u.v6.dst_addr[3] = 0X00000000;
		flt_rule.rule_hdl = IPACM_Wlan::dummy_flt_rule_hdl_v6[offset+2];
		memcpy(&(pFilteringTable->rules[2]), &flt_rule, sizeof(struct ipa_flt_rule_mdfy));

#ifdef FEATURE_IPA_ANDROID
		memset(&flt_rule, 0, sizeof(struct ipa_flt_rule_mdfy));

		flt_rule.status = -1;

		flt_rule.rule.retain_hdr = 1;
		flt_rule.rule.to_uc = 0;
		flt_rule.rule.action = IPA_PASS_TO_EXCEPTION;
		flt_rule.rule.eq_attrib_type = 1;

		flt_rule.rule.eq_attrib.rule_eq_bitmap = 0;

		if(rx_prop->rx[0].attrib.attrib_mask & IPA_FLT_META_DATA)
		{
			flt_rule.rule.eq_attrib.rule_eq_bitmap |= (1<<14);
			flt_rule.rule.eq_attrib.metadata_meq32_present = 1;
			flt_rule.rule.eq_attrib.metadata_meq32.offset = 0;
			flt_rule.rule.eq_attrib.metadata_meq32.value = rx_prop->rx[0].attrib.meta_data;
			flt_rule.rule.eq_attrib.metadata_meq32.mask = rx_prop->rx[0].attrib.meta_data_mask;
		}

		flt_rule.rule.eq_attrib.rule_eq_bitmap |= (1<<1);
		flt_rule.rule.eq_attrib.protocol_eq_present = 1;
		flt_rule.rule.eq_attrib.protocol_eq = IPACM_FIREWALL_IPPROTO_TCP;

		flt_rule.rule.eq_attrib.rule_eq_bitmap |= (1<<8);
		flt_rule.rule.eq_attrib.num_ihl_offset_meq_32 = 1;
		flt_rule.rule.eq_attrib.ihl_offset_meq_32[0].offset = 12;

		/* add TCP FIN rule*/
		flt_rule.rule.eq_attrib.ihl_offset_meq_32[0].value = (((uint32_t)1)<<TCP_FIN_SHIFT);
		flt_rule.rule.eq_attrib.ihl_offset_meq_32[0].mask = (((uint32_t)1)<<TCP_FIN_SHIFT);
		flt_rule.rule_hdl = IPACM_Wlan::dummy_flt_rule_hdl_v6[offset+3];
		memcpy(&(pFilteringTable->rules[3]), &flt_rule, sizeof(struct ipa_flt_rule_mdfy));

		/* add TCP SYN rule*/
		flt_rule.rule.eq_attrib.ihl_offset_meq_32[0].value = (((uint32_t)1)<<TCP_SYN_SHIFT);
		flt_rule.rule.eq_attrib.ihl_offset_meq_32[0].mask = (((uint32_t)1)<<TCP_SYN_SHIFT);
		flt_rule.rule_hdl = IPACM_Wlan::dummy_flt_rule_hdl_v6[offset+4];
		memcpy(&(pFilteringTable->rules[4]), &flt_rule, sizeof(struct ipa_flt_rule_mdfy));

		/* add TCP RST rule*/
		flt_rule.rule.eq_attrib.ihl_offset_meq_32[0].value = (((uint32_t)1)<<TCP_RST_SHIFT);
		flt_rule.rule.eq_attrib.ihl_offset_meq_32[0].mask = (((uint32_t)1)<<TCP_RST_SHIFT);
		flt_rule.rule_hdl = IPACM_Wlan::dummy_flt_rule_hdl_v6[offset+5];
		memcpy(&(pFilteringTable->rules[5]), &flt_rule, sizeof(struct ipa_flt_rule_mdfy));
#endif

		if (m_filtering.ModifyFilteringRule(pFilteringTable) == false)
		{
			IPACMERR("Failed to modify default ipv6 filtering rules.\n");
			res = IPACM_FAILURE;
			goto fail;
		}
		else
		{
			for (int i = 0; i < IPV6_DEFAULT_FILTERTING_RULES; i++)
			{
				if (pFilteringTable->rules[i].status == 0)
				{
					dft_v6fl_rule_hdl[i] = pFilteringTable->rules[i].rule_hdl;
					IPACMDBG_H("Default v6 Filter Rule %d HDL:0x%x\n", i, dft_v6fl_rule_hdl[i]);
				}
				else
				{
					IPACMERR("Failing adding v6 default IPV6 rule %d\n", i);
				}
			}
		}
	}

fail:
	free(pFilteringTable);
	return res;
}

int IPACM_Wlan::add_dummy_lan2lan_flt_rule(ipa_ip_type iptype)
{
	if(rx_prop == NULL)
	{
		IPACMDBG_H("There is no rx_prop for iface %s, not able to add dummy lan2lan filtering rule.\n", dev_name);
		return IPACM_FAILURE;
	}

	int offset;
	if(iptype == IPA_IP_v4)
	{
		if(IPACM_Wlan::dummy_flt_rule_hdl_v4 == NULL)
		{
			IPACMERR("Dummy ipv4 flt rule has not been installed.\n");
			return IPACM_FAILURE;
		}

#ifndef CT_OPT
		offset = wlan_ap_index * (IPV4_DEFAULT_FILTERTING_RULES + MAX_OFFLOAD_PAIR + IPACM_Iface::ipacmcfg->ipa_num_private_subnet)
						+ IPV4_DEFAULT_FILTERTING_RULES;
#else
		offset = wlan_ap_index * (IPV4_DEFAULT_FILTERTING_RULES + NUM_TCP_CTL_FLT_RULE + MAX_OFFLOAD_PAIR + IPACM_Iface::ipacmcfg->ipa_num_private_subnet)
						+ NUM_TCP_CTL_FLT_RULE + IPV4_DEFAULT_FILTERTING_RULES;
#endif

#ifdef FEATURE_IPA_ANDROID
		offset = offset + wlan_ap_index * (IPA_MAX_PRIVATE_SUBNET_ENTRIES - IPACM_Iface::ipacmcfg->ipa_num_private_subnet);
#endif
		for (int i = 0; i < MAX_OFFLOAD_PAIR; i++)
		{
			lan2lan_flt_rule_hdl_v4[i].rule_hdl = IPACM_Wlan::dummy_flt_rule_hdl_v4[offset+i];
			lan2lan_flt_rule_hdl_v4[i].valid = false;
			IPACMDBG_H("Lan2lan v4 flt rule %d hdl:0x%x\n", i, lan2lan_flt_rule_hdl_v4[i].rule_hdl);
		}
	}
	else if(iptype == IPA_IP_v6)
	{
		if(IPACM_Wlan::dummy_flt_rule_hdl_v6 == NULL)
		{
			IPACMERR("Dummy ipv6 flt rule has not been installed.\n");
			return IPACM_FAILURE;
		}

#ifndef CT_OPT
		offset = wlan_ap_index * (IPV6_DEFAULT_FILTERTING_RULES + MAX_OFFLOAD_PAIR);
#else
		offset = wlan_ap_index * (IPV6_DEFAULT_FILTERTING_RULES + NUM_TCP_CTL_FLT_RULE + MAX_OFFLOAD_PAIR)
						+ NUM_TCP_CTL_FLT_RULE;
#endif

		for (int i = 0; i < MAX_OFFLOAD_PAIR; i++)
		{
			lan2lan_flt_rule_hdl_v6[i].rule_hdl = IPACM_Wlan::dummy_flt_rule_hdl_v6[offset+i];
			lan2lan_flt_rule_hdl_v6[i].valid = false;
			IPACMDBG_H("Lan2lan v6 flt rule %d hdl:0x%x\n", i, lan2lan_flt_rule_hdl_v6[i].rule_hdl);
		}
	}
	else
	{
		IPACMERR("IP type is not expected.\n");
		return IPACM_FAILURE;
	}

	return IPACM_SUCCESS;
}

/* configure private subnet filter rules*/
int IPACM_Wlan::handle_private_subnet(ipa_ip_type iptype)
{
	int i, len, res = IPACM_SUCCESS, offset;
	struct ipa_flt_rule_mdfy flt_rule;
	struct ipa_ioc_mdfy_flt_rule* pFilteringTable;

	if (rx_prop == NULL)
	{
		IPACMDBG_H("No rx properties registered for iface %s\n", dev_name);
		return IPACM_SUCCESS;
	}

	if (iptype == IPA_IP_v4)
	{
		if(IPACM_Wlan::dummy_flt_rule_hdl_v4 == NULL)
		{
			IPACMERR("Dummy ipv4 flt rule has not been installed.\n");
			return IPACM_FAILURE;
		}
#ifdef FEATURE_ETH_BRIDGE_LE
		offset = IPV4_DEFAULT_FILTERTING_RULES + IPACM_Iface::ipacmcfg->ipa_num_private_subnet + IPA_LAN_TO_LAN_MAX_WLAN_CLIENT + IPA_LAN_TO_LAN_MAX_LAN_CLIENT;
#else
#ifndef CT_OPT
		offset = wlan_ap_index * (IPV4_DEFAULT_FILTERTING_RULES + MAX_OFFLOAD_PAIR + IPACM_Iface::ipacmcfg->ipa_num_private_subnet)
				+ IPV4_DEFAULT_FILTERTING_RULES + MAX_OFFLOAD_PAIR;
#else
		offset = wlan_ap_index * (IPV4_DEFAULT_FILTERTING_RULES + NUM_TCP_CTL_FLT_RULE + MAX_OFFLOAD_PAIR + IPACM_Iface::ipacmcfg->ipa_num_private_subnet)
				+ IPV4_DEFAULT_FILTERTING_RULES + NUM_TCP_CTL_FLT_RULE + MAX_OFFLOAD_PAIR;
#endif
#endif

		len = sizeof(struct ipa_ioc_mdfy_flt_rule) + (IPACM_Iface::ipacmcfg->ipa_num_private_subnet) * sizeof(struct ipa_flt_rule_mdfy);
		pFilteringTable = (struct ipa_ioc_mdfy_flt_rule*)malloc(len);
		if (!pFilteringTable)
		{
			IPACMERR("Failed to allocate ipa_ioc_mdfy_flt_rule memory...\n");
			return IPACM_FAILURE;
		}
		memset(pFilteringTable, 0, len);

		pFilteringTable->commit = 1;
		pFilteringTable->ip = iptype;
		pFilteringTable->num_rules = (uint8_t)IPACM_Iface::ipacmcfg->ipa_num_private_subnet;

		/* Make LAN-traffic always go A5, use default IPA-RT table */
		if (false == m_routing.GetRoutingTable(&IPACM_Iface::ipacmcfg->rt_tbl_default_v4))
		{
			IPACMERR("Failed to get routing table handle.\n");
			res = IPACM_FAILURE;
			goto fail;
		}

		memset(&flt_rule, 0, sizeof(struct ipa_flt_rule_mdfy));
		flt_rule.status = -1;

		flt_rule.rule.retain_hdr = 1;
		flt_rule.rule.to_uc = 0;
		flt_rule.rule.action = IPA_PASS_TO_ROUTING;
		flt_rule.rule.eq_attrib_type = 0;
		flt_rule.rule.rt_tbl_hdl = IPACM_Iface::ipacmcfg->rt_tbl_default_v4.hdl;
		IPACMDBG_H("Private filter rule use table: %s\n",IPACM_Iface::ipacmcfg->rt_tbl_default_v4.name);

		memcpy(&flt_rule.rule.attrib, &rx_prop->rx[0].attrib, sizeof(flt_rule.rule.attrib));
		flt_rule.rule.attrib.attrib_mask |= IPA_FLT_DST_ADDR;
#ifdef FEATURE_ETH_BRIDGE_LE
		/* remove meta data mask */
		flt_rule.rule.attrib.attrib_mask &= ~((uint32_t)IPA_FLT_META_DATA);
#endif

		for (i = 0; i < (IPACM_Iface::ipacmcfg->ipa_num_private_subnet); i++)
		{
			flt_rule.rule_hdl = IPACM_Wlan::dummy_flt_rule_hdl_v4[offset+i];
			flt_rule.rule.attrib.u.v4.dst_addr_mask = IPACM_Iface::ipacmcfg->private_subnet_table[i].subnet_mask;
			flt_rule.rule.attrib.u.v4.dst_addr = IPACM_Iface::ipacmcfg->private_subnet_table[i].subnet_addr;
			memcpy(&(pFilteringTable->rules[i]), &flt_rule, sizeof(struct ipa_flt_rule_mdfy));
		}

		if (false == m_filtering.ModifyFilteringRule(pFilteringTable))
		{
			IPACMERR("Failed to modify private subnet filtering rules.\n");
			res = IPACM_FAILURE;
			goto fail;
		}

		/* copy filter rule hdls */
		for (i = 0; i < IPACM_Iface::ipacmcfg->ipa_num_private_subnet; i++)
		{
			private_fl_rule_hdl[i] = pFilteringTable->rules[i].rule_hdl;
		}
	}
	else
	{
		return IPACM_SUCCESS;
	}
fail:
	free(pFilteringTable);
	return res;
}

/* install UL filter rule from Q6 */
int IPACM_Wlan::handle_uplink_filter_rule(ipacm_ext_prop *prop, ipa_ip_type iptype, uint8_t xlat_mux_id)
{
	ipa_flt_rule_add flt_rule_entry;
	int len = 0, cnt, ret = IPACM_SUCCESS, index;
	ipa_ioc_add_flt_rule *pFilteringTable;
	ipa_fltr_installed_notif_req_msg_v01 flt_index;
	int fd, i;
	uint32_t value = 0;

	IPACMDBG_H("Set extended property rules in WLAN\n");

	if (rx_prop == NULL)
	{
		IPACMDBG_H("No rx properties registered for iface %s\n", dev_name);
		return IPACM_SUCCESS;
	}

	if(prop == NULL || prop->num_ext_props <= 0)
	{
		IPACMDBG_H("No extended property.\n");
		return IPACM_SUCCESS;
	}

	if(wlan_ap_index > 0)
	{
		IPACMDBG_H("This is not the first WLAN AP, do not install modem UL rules.\n");
		return IPACM_SUCCESS;
	}

	fd = open(IPA_DEVICE_NAME, O_RDWR);
	if (0 == fd)
	{
		IPACMERR("Failed opening %s.\n", IPA_DEVICE_NAME);
		return IPACM_FAILURE;
	}
	if (prop->num_ext_props > MAX_WAN_UL_FILTER_RULES)
	{
		IPACMERR("number of modem UL rules > MAX_WAN_UL_FILTER_RULES, aborting...\n");
		close(fd);
		return IPACM_FAILURE;
	}

	memset(&flt_index, 0, sizeof(flt_index));
	flt_index.source_pipe_index = ioctl(fd, IPA_IOC_QUERY_EP_MAPPING, rx_prop->rx[0].src_pipe);
	flt_index.install_status = IPA_QMI_RESULT_SUCCESS_V01;
#ifndef FEATURE_IPA_V3
	flt_index.filter_index_list_len = prop->num_ext_props;
#else  /* defined (FEATURE_IPA_V3) */
	flt_index.rule_id_valid = 1;
	flt_index.rule_id_len = prop->num_ext_props;
#endif
	flt_index.embedded_pipe_index_valid = 1;
	flt_index.embedded_pipe_index = ioctl(fd, IPA_IOC_QUERY_EP_MAPPING, IPA_CLIENT_APPS_LAN_WAN_PROD);
	flt_index.retain_header_valid = 1;
	flt_index.retain_header = 0;
	flt_index.embedded_call_mux_id_valid = 1;
	flt_index.embedded_call_mux_id = IPACM_Iface::ipacmcfg->GetQmapId();
#ifndef FEATURE_IPA_V3
	IPACMDBG_H("flt_index: src pipe: %d, num of rules: %d, ebd pipe: %d, mux id: %d\n",
		flt_index.source_pipe_index, flt_index.filter_index_list_len, flt_index.embedded_pipe_index, flt_index.embedded_call_mux_id);
#else /* defined (FEATURE_IPA_V3) */
	IPACMDBG_H("flt_index: src pipe: %d, num of rules: %d, ebd pipe: %d, mux id: %d\n",
		flt_index.source_pipe_index, flt_index.rule_id_len, flt_index.embedded_pipe_index, flt_index.embedded_call_mux_id);
#endif
	len = sizeof(struct ipa_ioc_add_flt_rule) + prop->num_ext_props * sizeof(struct ipa_flt_rule_add);
	pFilteringTable = (struct ipa_ioc_add_flt_rule*)malloc(len);
	if (pFilteringTable == NULL)
	{
		IPACMERR("Error Locate ipa_flt_rule_add memory...\n");
		close(fd);
		return IPACM_FAILURE;
	}
	memset(pFilteringTable, 0, len);

	pFilteringTable->commit = 1;
	pFilteringTable->ep = rx_prop->rx[0].src_pipe;
	pFilteringTable->global = false;
	pFilteringTable->ip = iptype;
	pFilteringTable->num_rules = prop->num_ext_props;

	memset(&flt_rule_entry, 0, sizeof(struct ipa_flt_rule_add)); // Zero All Fields
	flt_rule_entry.at_rear = 1;
#ifdef FEATURE_IPA_V3
	if (flt_rule_entry.rule.eq_attrib.ipv4_frag_eq_present)
		flt_rule_entry.at_rear = 0;
#endif
	flt_rule_entry.flt_rule_hdl = -1;
	flt_rule_entry.status = -1;

	flt_rule_entry.rule.retain_hdr = 0;
	flt_rule_entry.rule.to_uc = 0;
	flt_rule_entry.rule.eq_attrib_type = 1;
	if(iptype == IPA_IP_v4)
		flt_rule_entry.rule.action = IPA_PASS_TO_SRC_NAT;
	else if(iptype == IPA_IP_v6)
		flt_rule_entry.rule.action = IPA_PASS_TO_ROUTING;
	else
	{
		IPACMERR("IP type is not expected.\n");
		ret = IPACM_FAILURE;
		goto fail;
	}

	index = IPACM_Iface::ipacmcfg->getFltRuleCount(rx_prop->rx[0].src_pipe, iptype);

#ifndef FEATURE_IPA_ANDROID
	if(iptype == IPA_IP_v4 && index != exp_index_v4)
	{
		IPACMDBG_DMESG("### WARNING ### num flt rules for IPv4 on client %d is not expected: %d expected value: %d",
			rx_prop->rx[0].src_pipe, index, exp_index_v4);
	}
	if(iptype == IPA_IP_v6 && index != exp_index_v6)
	{
		IPACMDBG_DMESG("### WARNING ### num flt rules for IPv6 on client %d is not expected: %d expected value: %d",
			rx_prop->rx[0].src_pipe, index, exp_index_v6);
	}
#endif

	for(cnt=0; cnt<prop->num_ext_props; cnt++)
	{
		memcpy(&flt_rule_entry.rule.eq_attrib,
					 &prop->prop[cnt].eq_attrib,
					 sizeof(prop->prop[cnt].eq_attrib));
		flt_rule_entry.rule.rt_tbl_idx = prop->prop[cnt].rt_tbl_idx;

		/* Handle XLAT configuration */
		if ((iptype == IPA_IP_v4) && prop->prop[cnt].is_xlat_rule && (xlat_mux_id != 0))
		{
			/* fill the value of meta-data */
			value = xlat_mux_id;
			flt_rule_entry.rule.eq_attrib.metadata_meq32_present = 1;
			flt_rule_entry.rule.eq_attrib.metadata_meq32.offset = 0;
			flt_rule_entry.rule.eq_attrib.metadata_meq32.value = (value & 0xFF) << 16;
			flt_rule_entry.rule.eq_attrib.metadata_meq32.mask = 0x00FF0000;
			IPACMDBG_H("xlat meta-data is modified for rule: %d has index: %d with xlat_mux_id: %d\n",
					cnt, index, xlat_mux_id);
                }
#ifdef FEATURE_IPA_V3
		flt_rule_entry.rule.hashable = prop->prop[cnt].is_rule_hashable;
		flt_rule_entry.rule.rule_id = prop->prop[cnt].rule_id;
#endif
		memcpy(&pFilteringTable->rules[cnt], &flt_rule_entry, sizeof(flt_rule_entry));

		IPACMDBG_H("Modem UL filtering rule %d has index %d\n", cnt, index);
#ifndef FEATURE_IPA_V3
		flt_index.filter_index_list[cnt].filter_index = index;
		flt_index.filter_index_list[cnt].filter_handle = prop->prop[cnt].filter_hdl;
#else /* defined (FEATURE_IPA_V3) */
		flt_index.rule_id[cnt] = prop->prop[cnt].rule_id;
#endif
		index++;
	}

	if(false == m_filtering.SendFilteringRuleIndex(&flt_index))
	{
		IPACMERR("Error sending filtering rule index, aborting...\n");
		ret = IPACM_FAILURE;
		goto fail;
	}

	if(false == m_filtering.AddFilteringRule(pFilteringTable))
	{
		IPACMERR("Error Adding RuleTable to Filtering, aborting...\n");
		ret = IPACM_FAILURE;
		goto fail;
	}
	else
	{
		if(iptype == IPA_IP_v4)
		{
			for(i=0; i<pFilteringTable->num_rules; i++)
			{
				wan_ul_fl_rule_hdl_v4[num_wan_ul_fl_rule_v4] = pFilteringTable->rules[i].flt_rule_hdl;
				num_wan_ul_fl_rule_v4++;
			}
			IPACM_Iface::ipacmcfg->increaseFltRuleCount(rx_prop->rx[0].src_pipe, iptype, pFilteringTable->num_rules);
		}
		else if(iptype == IPA_IP_v6)
		{
			for(i=0; i<pFilteringTable->num_rules; i++)
			{
				wan_ul_fl_rule_hdl_v6[num_wan_ul_fl_rule_v6] = pFilteringTable->rules[i].flt_rule_hdl;
				num_wan_ul_fl_rule_v6++;
			}
			IPACM_Iface::ipacmcfg->increaseFltRuleCount(rx_prop->rx[0].src_pipe, iptype, pFilteringTable->num_rules);
		}
		else
		{
			IPACMERR("IP type is not expected.\n");
			goto fail;
		}
	}

fail:
	free(pFilteringTable);
	close(fd);
	return ret;
}

/* handle wifi client initial,copy all partial headers (tx property) */
int IPACM_Wlan::handle_wlan_client_init_ex(ipacm_event_data_wlan_ex *data)
{

#define WLAN_IFACE_INDEX_LEN 2

	int res = IPACM_SUCCESS, len = 0, i, evt_size;
	char index[WLAN_IFACE_INDEX_LEN];
	struct ipa_ioc_copy_hdr sCopyHeader;
	struct ipa_ioc_add_hdr *pHeaderDescriptor = NULL;
        uint32_t cnt;

	/* start of adding header */
	IPACMDBG_H("Wifi client number for this iface: %d & total number of wlan clients: %d\n",
                 num_wifi_client,IPACM_Wlan::total_num_wifi_clients);

	if ((num_wifi_client >= IPA_MAX_NUM_WIFI_CLIENTS) ||
			(IPACM_Wlan::total_num_wifi_clients >= IPA_MAX_NUM_WIFI_CLIENTS))
	{
		IPACMERR("Reached maximum number of wlan clients\n");
		return IPACM_FAILURE;
	}

	IPACMDBG_H("Wifi client number: %d\n", num_wifi_client);

	/* add header to IPA */
	if(tx_prop != NULL)
	{
		len = sizeof(struct ipa_ioc_add_hdr) + (1 * sizeof(struct ipa_hdr_add));
		pHeaderDescriptor = (struct ipa_ioc_add_hdr *)calloc(1, len);
		if (pHeaderDescriptor == NULL)
		{
			IPACMERR("calloc failed to allocate pHeaderDescriptor\n");
			return IPACM_FAILURE;
		}

		evt_size = sizeof(ipacm_event_data_wlan_ex) + data->num_of_attribs * sizeof(struct ipa_wlan_hdr_attrib_val);
		get_client_memptr(wlan_client, num_wifi_client)->p_hdr_info = (ipacm_event_data_wlan_ex*)malloc(evt_size);
		memcpy(get_client_memptr(wlan_client, num_wifi_client)->p_hdr_info, data, evt_size);

		/* copy partial header for v4*/
		for (cnt=0; cnt<tx_prop->num_tx_props; cnt++)
		{
			if(tx_prop->tx[cnt].ip==IPA_IP_v4)
			{
				IPACMDBG_H("Got partial v4-header name from %d tx props\n", cnt);
				memset(&sCopyHeader, 0, sizeof(sCopyHeader));
				memcpy(sCopyHeader.name,
							 tx_prop->tx[cnt].hdr_name,
							 sizeof(sCopyHeader.name));

				IPACMDBG_H("header name: %s in tx:%d\n", sCopyHeader.name,cnt);
				if (m_header.CopyHeader(&sCopyHeader) == false)
				{
					PERROR("ioctl copy header failed");
					res = IPACM_FAILURE;
					goto fail;
				}

				IPACMDBG_H("header length: %d, paritial: %d\n", sCopyHeader.hdr_len, sCopyHeader.is_partial);
				if (sCopyHeader.hdr_len > IPA_HDR_MAX_SIZE)
				{
					IPACMERR("header oversize\n");
					res = IPACM_FAILURE;
					goto fail;
				}
				else
				{
					memcpy(pHeaderDescriptor->hdr[0].hdr,
								 sCopyHeader.hdr,
								 sCopyHeader.hdr_len);
				}

				for(i = 0; i < data->num_of_attribs; i++)
				{
					if(data->attribs[i].attrib_type == WLAN_HDR_ATTRIB_MAC_ADDR)
					{
						memcpy(get_client_memptr(wlan_client, num_wifi_client)->mac,
								data->attribs[i].u.mac_addr,
								sizeof(get_client_memptr(wlan_client, num_wifi_client)->mac));

						/* copy client mac_addr to partial header */
						memcpy(&pHeaderDescriptor->hdr[0].hdr[data->attribs[i].offset],
									 get_client_memptr(wlan_client, num_wifi_client)->mac,
									 IPA_MAC_ADDR_SIZE);
					}
					else if(data->attribs[i].attrib_type == WLAN_HDR_ATTRIB_STA_ID)
					{
						/* copy client id to header */
						memcpy(&pHeaderDescriptor->hdr[0].hdr[data->attribs[i].offset],
									&data->attribs[i].u.sta_id, sizeof(data->attribs[i].u.sta_id));
					}
					else
					{
						IPACMDBG_H("The attribute type is not expected!\n");
					}
				}

				pHeaderDescriptor->commit = true;
				pHeaderDescriptor->num_hdrs = 1;

				memset(pHeaderDescriptor->hdr[0].name, 0,
							 sizeof(pHeaderDescriptor->hdr[0].name));

				snprintf(index,sizeof(index), "%d", ipa_if_num);
				strlcpy(pHeaderDescriptor->hdr[0].name, index, sizeof(pHeaderDescriptor->hdr[0].name));
				pHeaderDescriptor->hdr[0].name[IPA_RESOURCE_NAME_MAX-1] = '\0';

				if (strlcat(pHeaderDescriptor->hdr[0].name, IPA_WLAN_PARTIAL_HDR_NAME_v4, sizeof(pHeaderDescriptor->hdr[0].name)) > IPA_RESOURCE_NAME_MAX)
				{
					IPACMERR(" header name construction failed exceed length (%d)\n", strlen(pHeaderDescriptor->hdr[0].name));
					res = IPACM_FAILURE;
					goto fail;
				}
				snprintf(index,sizeof(index), "%d", header_name_count);
				if (strlcat(pHeaderDescriptor->hdr[0].name, index, sizeof(pHeaderDescriptor->hdr[0].name)) > IPA_RESOURCE_NAME_MAX)
				{
					IPACMERR(" header name construction failed exceed length (%d)\n", strlen(pHeaderDescriptor->hdr[0].name));
					res = IPACM_FAILURE;
					goto fail;
				}


				pHeaderDescriptor->hdr[0].hdr_len = sCopyHeader.hdr_len;
				pHeaderDescriptor->hdr[0].hdr_hdl = -1;
				pHeaderDescriptor->hdr[0].is_partial = 0;
				pHeaderDescriptor->hdr[0].status = -1;

				if (m_header.AddHeader(pHeaderDescriptor) == false ||
						pHeaderDescriptor->hdr[0].status != 0)
				{
					IPACMERR("ioctl IPA_IOC_ADD_HDR failed: %d\n", pHeaderDescriptor->hdr[0].status);
					res = IPACM_FAILURE;
					goto fail;
				}

				get_client_memptr(wlan_client, num_wifi_client)->hdr_hdl_v4 = pHeaderDescriptor->hdr[0].hdr_hdl;
				IPACMDBG_H("client(%d) v4 full header name:%s header handle:(0x%x)\n",
								 num_wifi_client,
								 pHeaderDescriptor->hdr[0].name,
								 get_client_memptr(wlan_client, num_wifi_client)->hdr_hdl_v4);
				get_client_memptr(wlan_client, num_wifi_client)->ipv4_header_set=true;
				break;
			}
		}

		/* copy partial header for v6*/
		for (cnt=0; cnt<tx_prop->num_tx_props; cnt++)
		{
			if(tx_prop->tx[cnt].ip==IPA_IP_v6)
			{
				IPACMDBG_H("Got partial v6-header name from %d tx props\n", cnt);
				memset(&sCopyHeader, 0, sizeof(sCopyHeader));
				memcpy(sCopyHeader.name,
							 tx_prop->tx[cnt].hdr_name,
							 sizeof(sCopyHeader.name));

				IPACMDBG_H("header name: %s in tx:%d\n", sCopyHeader.name,cnt);
				if (m_header.CopyHeader(&sCopyHeader) == false)
				{
					PERROR("ioctl copy header failed");
					res = IPACM_FAILURE;
					goto fail;
				}

				IPACMDBG_H("header length: %d, paritial: %d\n", sCopyHeader.hdr_len, sCopyHeader.is_partial);
				if (sCopyHeader.hdr_len > IPA_HDR_MAX_SIZE)
				{
					IPACMERR("header oversize\n");
					res = IPACM_FAILURE;
					goto fail;
				}
				else
				{
					memcpy(pHeaderDescriptor->hdr[0].hdr,
								 sCopyHeader.hdr,
								 sCopyHeader.hdr_len);
				}

				for(i = 0; i < data->num_of_attribs; i++)
				{
					if(data->attribs[i].attrib_type == WLAN_HDR_ATTRIB_MAC_ADDR)
					{
						memcpy(get_client_memptr(wlan_client, num_wifi_client)->mac,
								data->attribs[i].u.mac_addr,
								sizeof(get_client_memptr(wlan_client, num_wifi_client)->mac));

						/* copy client mac_addr to partial header */
						memcpy(&pHeaderDescriptor->hdr[0].hdr[data->attribs[i].offset],
								get_client_memptr(wlan_client, num_wifi_client)->mac,
								IPA_MAC_ADDR_SIZE);
					}
					else if (data->attribs[i].attrib_type == WLAN_HDR_ATTRIB_STA_ID)
					{
						/* copy client id to header */
						memcpy(&pHeaderDescriptor->hdr[0].hdr[data->attribs[i].offset],
								&data->attribs[i].u.sta_id, sizeof(data->attribs[i].u.sta_id));
					}
					else
					{
						IPACMDBG_H("The attribute type is not expected!\n");
					}
				}

				pHeaderDescriptor->commit = true;
				pHeaderDescriptor->num_hdrs = 1;

				memset(pHeaderDescriptor->hdr[0].name, 0,
							 sizeof(pHeaderDescriptor->hdr[0].name));

				snprintf(index,sizeof(index), "%d", ipa_if_num);
				strlcpy(pHeaderDescriptor->hdr[0].name, index, sizeof(pHeaderDescriptor->hdr[0].name));
				pHeaderDescriptor->hdr[0].name[IPA_RESOURCE_NAME_MAX-1] = '\0';
				if (strlcat(pHeaderDescriptor->hdr[0].name, IPA_WLAN_PARTIAL_HDR_NAME_v6, sizeof(pHeaderDescriptor->hdr[0].name)) > IPA_RESOURCE_NAME_MAX)
				{
					IPACMERR(" header name construction failed exceed length (%d)\n", strlen(pHeaderDescriptor->hdr[0].name));
					res = IPACM_FAILURE;
					goto fail;
				}

				snprintf(index,sizeof(index), "%d", header_name_count);
				if (strlcat(pHeaderDescriptor->hdr[0].name, index, sizeof(pHeaderDescriptor->hdr[0].name)) > IPA_RESOURCE_NAME_MAX)
				{
					IPACMERR(" header name construction failed exceed length (%d)\n", strlen(pHeaderDescriptor->hdr[0].name));
					res = IPACM_FAILURE;
					goto fail;
				}

				pHeaderDescriptor->hdr[0].hdr_len = sCopyHeader.hdr_len;
				pHeaderDescriptor->hdr[0].hdr_hdl = -1;
				pHeaderDescriptor->hdr[0].is_partial = 0;
				pHeaderDescriptor->hdr[0].status = -1;

				if (m_header.AddHeader(pHeaderDescriptor) == false ||
						pHeaderDescriptor->hdr[0].status != 0)
				{
					IPACMERR("ioctl IPA_IOC_ADD_HDR failed: %d\n", pHeaderDescriptor->hdr[0].status);
					res = IPACM_FAILURE;
					goto fail;
				}

				get_client_memptr(wlan_client, num_wifi_client)->hdr_hdl_v6 = pHeaderDescriptor->hdr[0].hdr_hdl;
				IPACMDBG_H("client(%d) v6 full header name:%s header handle:(0x%x)\n",
								 num_wifi_client,
								 pHeaderDescriptor->hdr[0].name,
											 get_client_memptr(wlan_client, num_wifi_client)->hdr_hdl_v6);

				get_client_memptr(wlan_client, num_wifi_client)->ipv6_header_set=true;
				break;
			}
		}

		/* initialize wifi client*/
		get_client_memptr(wlan_client, num_wifi_client)->route_rule_set_v4 = false;
		get_client_memptr(wlan_client, num_wifi_client)->route_rule_set_v6 = 0;
		get_client_memptr(wlan_client, num_wifi_client)->ipv4_set = false;
		get_client_memptr(wlan_client, num_wifi_client)->ipv6_set = 0;
		get_client_memptr(wlan_client, num_wifi_client)->power_save_set=false;
		num_wifi_client++;
		header_name_count++; //keep increasing header_name_count
		IPACM_Wlan::total_num_wifi_clients++;
		res = IPACM_SUCCESS;
		IPACMDBG_H("Wifi client number: %d\n", num_wifi_client);
	}
	else
	{
		return res;
	}

fail:
	free(pHeaderDescriptor);
	return res;
}

/*handle wifi client */
int IPACM_Wlan::handle_wlan_client_ipaddr(ipacm_event_data_all *data)
{
	int clnt_indx;
	int v6_num;

	IPACMDBG_H("number of wifi clients: %d\n", num_wifi_client);
	IPACMDBG_H(" event MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
					 data->mac_addr[0],
					 data->mac_addr[1],
					 data->mac_addr[2],
					 data->mac_addr[3],
					 data->mac_addr[4],
					 data->mac_addr[5]);

	clnt_indx = get_wlan_client_index(data->mac_addr);

		if (clnt_indx == IPACM_INVALID_INDEX)
		{
			IPACMERR("wlan client not found/attached \n");
			return IPACM_FAILURE;
		}

	IPACMDBG_H("Ip-type received %d\n", data->iptype);
	if (data->iptype == IPA_IP_v4)
	{
		IPACMDBG_H("ipv4 address: 0x%x\n", data->ipv4_addr);
		if (data->ipv4_addr != 0) /* not 0.0.0.0 */
		{
			if (get_client_memptr(wlan_client, clnt_indx)->ipv4_set == false)
			{
				get_client_memptr(wlan_client, clnt_indx)->v4_addr = data->ipv4_addr;
				get_client_memptr(wlan_client, clnt_indx)->ipv4_set = true;
			}
			else
			{
			   /* check if client got new IPv4 address*/
			   if(data->ipv4_addr == get_client_memptr(wlan_client, clnt_indx)->v4_addr)
			   {
			     IPACMDBG_H("Already setup ipv4 addr for client:%d, ipv4 address didn't change\n", clnt_indx);
				 return IPACM_FAILURE;
			   }
			   else
			   {
			     IPACMDBG_H("ipv4 addr for client:%d is changed \n", clnt_indx);
				 /* delete NAT rules first */
				 CtList->HandleNeighIpAddrDelEvt(get_client_memptr(wlan_client, clnt_indx)->v4_addr);
			     delete_default_qos_rtrules(clnt_indx,IPA_IP_v4);
		         get_client_memptr(wlan_client, clnt_indx)->route_rule_set_v4 = false;
			     get_client_memptr(wlan_client, clnt_indx)->v4_addr = data->ipv4_addr;
			}
		}
	}
	else
	{
		    IPACMDBG_H("Invalid client IPv4 address \n");
		    return IPACM_FAILURE;
		}
	}
	else
	{
		if ((data->ipv6_addr[0] != 0) || (data->ipv6_addr[1] != 0) ||
				(data->ipv6_addr[2] != 0) || (data->ipv6_addr[3] || 0)) /* check if all 0 not valid ipv6 address */
		{
		   IPACMDBG_H("ipv6 address: 0x%x:%x:%x:%x\n", data->ipv6_addr[0], data->ipv6_addr[1], data->ipv6_addr[2], data->ipv6_addr[3]);
                   if(get_client_memptr(wlan_client, clnt_indx)->ipv6_set < IPV6_NUM_ADDR)
		   {

		       for(v6_num=0;v6_num < get_client_memptr(wlan_client, clnt_indx)->ipv6_set;v6_num++)
	               {
			      if( data->ipv6_addr[0] == get_client_memptr(wlan_client, clnt_indx)->v6_addr[v6_num][0] &&
			           data->ipv6_addr[1] == get_client_memptr(wlan_client, clnt_indx)->v6_addr[v6_num][1] &&
			  	        data->ipv6_addr[2]== get_client_memptr(wlan_client, clnt_indx)->v6_addr[v6_num][2] &&
			  	         data->ipv6_addr[3] == get_client_memptr(wlan_client, clnt_indx)->v6_addr[v6_num][3])
			      {
			  	    IPACMDBG_H("Already see this ipv6 addr for client:%d\n", clnt_indx);
			  	    return IPACM_FAILURE; /* not setup the RT rules*/
			  		break;
			      }
		       }

		       /* not see this ipv6 before for wifi client*/
			   get_client_memptr(wlan_client, clnt_indx)->v6_addr[get_client_memptr(wlan_client, clnt_indx)->ipv6_set][0] = data->ipv6_addr[0];
			   get_client_memptr(wlan_client, clnt_indx)->v6_addr[get_client_memptr(wlan_client, clnt_indx)->ipv6_set][1] = data->ipv6_addr[1];
			   get_client_memptr(wlan_client, clnt_indx)->v6_addr[get_client_memptr(wlan_client, clnt_indx)->ipv6_set][2] = data->ipv6_addr[2];
			   get_client_memptr(wlan_client, clnt_indx)->v6_addr[get_client_memptr(wlan_client, clnt_indx)->ipv6_set][3] = data->ipv6_addr[3];
			   get_client_memptr(wlan_client, clnt_indx)->ipv6_set++;
		    }
		    else
		    {
		         IPACMDBG_H("Already got 3 ipv6 addr for client:%d\n", clnt_indx);
			 return IPACM_FAILURE; /* not setup the RT rules*/
		    }
		}
	}

	return IPACM_SUCCESS;
}

/*handle wifi client routing rule*/
int IPACM_Wlan::handle_wlan_client_route_rule(uint8_t *mac_addr, ipa_ip_type iptype)
{
	struct ipa_ioc_add_rt_rule *rt_rule;
	struct ipa_rt_rule_add *rt_rule_entry;
	uint32_t tx_index;
	int wlan_index,v6_num;
	const int NUM = 1;

	if(tx_prop == NULL)
	{
		IPACMDBG_H("No rx properties registered for iface %s\n", dev_name);
		return IPACM_SUCCESS;
	}

	IPACMDBG_H("Received mac_addr MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
			mac_addr[0], mac_addr[1], mac_addr[2],
			mac_addr[3], mac_addr[4], mac_addr[5]);

	wlan_index = get_wlan_client_index(mac_addr);
	if (wlan_index == IPACM_INVALID_INDEX)
	{
		IPACMDBG_H("wlan client not found/attached \n");
		return IPACM_SUCCESS;
	}

	/* during power_save mode, even receive IP_ADDR_ADD, not setting RT rules*/
	if (get_client_memptr(wlan_client, wlan_index)->power_save_set == true)
	{
		IPACMDBG_H("wlan client is in power safe mode \n");
		return IPACM_SUCCESS;
	}

	if (iptype==IPA_IP_v4)
	{
		IPACMDBG_H("wlan client index: %d, ip-type: %d, ipv4_set:%d, ipv4_rule_set:%d \n", wlan_index, iptype,
				get_client_memptr(wlan_client, wlan_index)->ipv4_set,
				get_client_memptr(wlan_client, wlan_index)->route_rule_set_v4);
	}
	else
	{
		IPACMDBG_H("wlan client index: %d, ip-type: %d, ipv6_set:%d, ipv6_rule_num:%d \n", wlan_index, iptype,
				get_client_memptr(wlan_client, wlan_index)->ipv6_set,
				get_client_memptr(wlan_client, wlan_index)->route_rule_set_v6);
	}


	/* Add default  Qos routing rules if not set yet */
	if ((iptype == IPA_IP_v4
				&& get_client_memptr(wlan_client, wlan_index)->route_rule_set_v4 == false
				&& get_client_memptr(wlan_client, wlan_index)->ipv4_set == true)
			|| (iptype == IPA_IP_v6
				&& get_client_memptr(wlan_client, wlan_index)->route_rule_set_v6 < get_client_memptr(wlan_client, wlan_index)->ipv6_set
			   ))
	{
		rt_rule = (struct ipa_ioc_add_rt_rule *)
			calloc(1, sizeof(struct ipa_ioc_add_rt_rule) +
					NUM * sizeof(struct ipa_rt_rule_add));

		if (rt_rule == NULL)
		{
			PERROR("Error Locate ipa_ioc_add_rt_rule memory...\n");
			return IPACM_FAILURE;
		}

		rt_rule->commit = 1;
		rt_rule->num_rules = (uint8_t)NUM;
		rt_rule->ip = iptype;


		for (tx_index = 0; tx_index < iface_query->num_tx_props; tx_index++)
		{

			if(iptype != tx_prop->tx[tx_index].ip)
			{
				IPACMDBG_H("Tx:%d, ip-type: %d conflict ip-type: %d no RT-rule added\n",
						tx_index, tx_prop->tx[tx_index].ip,iptype);
				continue;
			}

			rt_rule_entry = &rt_rule->rules[0];
			rt_rule_entry->at_rear = 0;

			if (iptype == IPA_IP_v4)
			{
				IPACMDBG_H("client index(%d):ipv4 address: 0x%x\n", wlan_index,
						get_client_memptr(wlan_client, wlan_index)->v4_addr);

				IPACMDBG_H("client(%d): v4 header handle:(0x%x)\n",
						wlan_index,
						get_client_memptr(wlan_client, wlan_index)->hdr_hdl_v4);
				strlcpy(rt_rule->rt_tbl_name,
						IPACM_Iface::ipacmcfg->rt_tbl_lan_v4.name,
						sizeof(rt_rule->rt_tbl_name));
				rt_rule->rt_tbl_name[IPA_RESOURCE_NAME_MAX-1] = '\0';

				if(IPACM_Iface::ipacmcfg->isMCC_Mode)
				{
					IPACMDBG_H("In MCC mode, use alt dst pipe: %d\n",
							tx_prop->tx[tx_index].alt_dst_pipe);
					rt_rule_entry->rule.dst = tx_prop->tx[tx_index].alt_dst_pipe;
				}
				else
				{
					rt_rule_entry->rule.dst = tx_prop->tx[tx_index].dst_pipe;
				}

				memcpy(&rt_rule_entry->rule.attrib,
						&tx_prop->tx[tx_index].attrib,
						sizeof(rt_rule_entry->rule.attrib));
				rt_rule_entry->rule.attrib.attrib_mask |= IPA_FLT_DST_ADDR;
				rt_rule_entry->rule.hdr_hdl = get_client_memptr(wlan_client, wlan_index)->hdr_hdl_v4;
				rt_rule_entry->rule.attrib.u.v4.dst_addr = get_client_memptr(wlan_client, wlan_index)->v4_addr;
				rt_rule_entry->rule.attrib.u.v4.dst_addr_mask = 0xFFFFFFFF;
				if (false == m_routing.AddRoutingRule(rt_rule))
				{
					IPACMERR("Routing rule addition failed!\n");
					free(rt_rule);
					return IPACM_FAILURE;
				}

				/* copy ipv4 RT hdl */
				get_client_memptr(wlan_client, wlan_index)->wifi_rt_hdl[tx_index].wifi_rt_rule_hdl_v4 =
					rt_rule->rules[0].rt_rule_hdl;
				IPACMDBG_H("tx:%d, rt rule hdl=%x ip-type: %d\n", tx_index,
						get_client_memptr(wlan_client, wlan_index)->wifi_rt_hdl[tx_index].wifi_rt_rule_hdl_v4, iptype);
			}
			else
			{
				for(v6_num = get_client_memptr(wlan_client, wlan_index)->route_rule_set_v6;v6_num < get_client_memptr(wlan_client, wlan_index)->ipv6_set;v6_num++)
				{
					IPACMDBG_H("client(%d): v6 header handle:(0x%x)\n",
							wlan_index,
							get_client_memptr(wlan_client, wlan_index)->hdr_hdl_v6);

					/* v6 LAN_RT_TBL */
					strlcpy(rt_rule->rt_tbl_name,
							IPACM_Iface::ipacmcfg->rt_tbl_v6.name,
							sizeof(rt_rule->rt_tbl_name));
					rt_rule->rt_tbl_name[IPA_RESOURCE_NAME_MAX-1] = '\0';
					/* Support QCMAP LAN traffic feature, send to A5 */
					rt_rule_entry->rule.dst = iface_query->excp_pipe;
					memset(&rt_rule_entry->rule.attrib, 0, sizeof(rt_rule_entry->rule.attrib));
					rt_rule_entry->rule.hdr_hdl = 0;
					rt_rule_entry->rule.attrib.attrib_mask |= IPA_FLT_DST_ADDR;
					rt_rule_entry->rule.attrib.u.v6.dst_addr[0] = get_client_memptr(wlan_client, wlan_index)->v6_addr[v6_num][0];
					rt_rule_entry->rule.attrib.u.v6.dst_addr[1] = get_client_memptr(wlan_client, wlan_index)->v6_addr[v6_num][1];
					rt_rule_entry->rule.attrib.u.v6.dst_addr[2] = get_client_memptr(wlan_client, wlan_index)->v6_addr[v6_num][2];
					rt_rule_entry->rule.attrib.u.v6.dst_addr[3] = get_client_memptr(wlan_client, wlan_index)->v6_addr[v6_num][3];
					rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[0] = 0xFFFFFFFF;
					rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[1] = 0xFFFFFFFF;
					rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[2] = 0xFFFFFFFF;
					rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[3] = 0xFFFFFFFF;
					if (false == m_routing.AddRoutingRule(rt_rule))
					{
						IPACMERR("Routing rule addition failed!\n");
						free(rt_rule);
						return IPACM_FAILURE;
					}

					get_client_memptr(wlan_client, wlan_index)->wifi_rt_hdl[tx_index].wifi_rt_rule_hdl_v6[v6_num] = rt_rule->rules[0].rt_rule_hdl;
					IPACMDBG_H("tx:%d, rt rule hdl=%x ip-type: %d\n", tx_index,
							get_client_memptr(wlan_client, wlan_index)->wifi_rt_hdl[tx_index].wifi_rt_rule_hdl_v6[v6_num], iptype);

					/*Copy same rule to v6 WAN RT TBL*/
					strlcpy(rt_rule->rt_tbl_name,
							IPACM_Iface::ipacmcfg->rt_tbl_wan_v6.name,
							sizeof(rt_rule->rt_tbl_name));
					rt_rule->rt_tbl_name[IPA_RESOURCE_NAME_MAX-1] = '\0';
					/* Downlink traffic from Wan iface, directly through IPA */
					if(IPACM_Iface::ipacmcfg->isMCC_Mode)
					{
						IPACMDBG_H("In MCC mode, use alt dst pipe: %d\n",
								tx_prop->tx[tx_index].alt_dst_pipe);
						rt_rule_entry->rule.dst = tx_prop->tx[tx_index].alt_dst_pipe;
					}
					else
					{
						rt_rule_entry->rule.dst = tx_prop->tx[tx_index].dst_pipe;
					}
					memcpy(&rt_rule_entry->rule.attrib,
							&tx_prop->tx[tx_index].attrib,
							sizeof(rt_rule_entry->rule.attrib));
					rt_rule_entry->rule.hdr_hdl = get_client_memptr(wlan_client, wlan_index)->hdr_hdl_v6;
					rt_rule_entry->rule.attrib.attrib_mask |= IPA_FLT_DST_ADDR;
					rt_rule_entry->rule.attrib.u.v6.dst_addr[0] = get_client_memptr(wlan_client, wlan_index)->v6_addr[v6_num][0];
					rt_rule_entry->rule.attrib.u.v6.dst_addr[1] = get_client_memptr(wlan_client, wlan_index)->v6_addr[v6_num][1];
					rt_rule_entry->rule.attrib.u.v6.dst_addr[2] = get_client_memptr(wlan_client, wlan_index)->v6_addr[v6_num][2];
					rt_rule_entry->rule.attrib.u.v6.dst_addr[3] = get_client_memptr(wlan_client, wlan_index)->v6_addr[v6_num][3];
					rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[0] = 0xFFFFFFFF;
					rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[1] = 0xFFFFFFFF;
					rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[2] = 0xFFFFFFFF;
					rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[3] = 0xFFFFFFFF;
					if (false == m_routing.AddRoutingRule(rt_rule))
					{
						IPACMERR("Routing rule addition failed!\n");
						free(rt_rule);
						return IPACM_FAILURE;
					}

					get_client_memptr(wlan_client, wlan_index)->wifi_rt_hdl[tx_index].wifi_rt_rule_hdl_v6_wan[v6_num] = rt_rule->rules[0].rt_rule_hdl;

					IPACMDBG_H("tx:%d, rt rule hdl=%x ip-type: %d\n", tx_index,
							get_client_memptr(wlan_client, wlan_index)->wifi_rt_hdl[tx_index].wifi_rt_rule_hdl_v6_wan[v6_num], iptype);
				}
			}

		} /* end of for loop */

		free(rt_rule);

		if (iptype == IPA_IP_v4)
		{
			get_client_memptr(wlan_client, wlan_index)->route_rule_set_v4 = true;
		}
		else
		{
			get_client_memptr(wlan_client, wlan_index)->route_rule_set_v6 = get_client_memptr(wlan_client, wlan_index)->ipv6_set;
		}
	}

	return IPACM_SUCCESS;
}

/*handle wifi client power-save mode*/
int IPACM_Wlan::handle_wlan_client_pwrsave(uint8_t *mac_addr)
{
	int clt_indx;
	IPACMDBG_H("wlan->handle_wlan_client_pwrsave();\n");

	clt_indx = get_wlan_client_index(mac_addr);
	if (clt_indx == IPACM_INVALID_INDEX)
	{
		IPACMDBG_H("wlan client not attached\n");
		return IPACM_SUCCESS;
	}

        if (get_client_memptr(wlan_client, clt_indx)->power_save_set == false)
	{
		/* First reset nat rules and then route rules */
	    if(get_client_memptr(wlan_client, clt_indx)->ipv4_set == true)
	    {
			IPACMDBG_H("Deleting Nat Rules\n");
			Nat_App->UpdatePwrSaveIf(get_client_memptr(wlan_client, clt_indx)->v4_addr);
 	     }

		IPACMDBG_H("Deleting default qos Route Rules\n");
		delete_default_qos_rtrules(clt_indx, IPA_IP_v4);
		delete_default_qos_rtrules(clt_indx, IPA_IP_v6);
                get_client_memptr(wlan_client, clt_indx)->power_save_set = true;
	}
	else
	{
		IPACMDBG_H("wlan client already in power-save mode\n");
	}
    return IPACM_SUCCESS;
}

/*handle wifi client del mode*/
int IPACM_Wlan::handle_wlan_client_down_evt(uint8_t *mac_addr)
{
	int clt_indx;
	uint32_t tx_index;
	int num_wifi_client_tmp = num_wifi_client;
	int num_v6;

	IPACMDBG_H("total client: %d\n", num_wifi_client_tmp);

	clt_indx = get_wlan_client_index(mac_addr);
	if (clt_indx == IPACM_INVALID_INDEX)
	{
		IPACMDBG_H("wlan client not attached\n");
		return IPACM_SUCCESS;
	}

	/* First reset nat rules and then route rules */
	if(get_client_memptr(wlan_client, clt_indx)->ipv4_set == true)
	{
	        IPACMDBG_H("Clean Nat Rules for ipv4:0x%x\n", get_client_memptr(wlan_client, clt_indx)->v4_addr);
			CtList->HandleNeighIpAddrDelEvt(get_client_memptr(wlan_client, clt_indx)->v4_addr);
 	}

	if (delete_default_qos_rtrules(clt_indx, IPA_IP_v4))
	{
		IPACMERR("unbale to delete v4 default qos route rules for index: %d\n", clt_indx);
		return IPACM_FAILURE;
	}

	if (delete_default_qos_rtrules(clt_indx, IPA_IP_v6))
	{
		IPACMERR("unbale to delete v6 default qos route rules for indexn: %d\n", clt_indx);
		return IPACM_FAILURE;
	}

	/* Delete wlan client header */
	if(get_client_memptr(wlan_client, clt_indx)->ipv4_header_set == true)
	{
	if (m_header.DeleteHeaderHdl(get_client_memptr(wlan_client, clt_indx)->hdr_hdl_v4)
			== false)
	{
		return IPACM_FAILURE;
	}
		get_client_memptr(wlan_client, clt_indx)->ipv4_header_set = false;
	}

	if(get_client_memptr(wlan_client, clt_indx)->ipv6_header_set == true)
	{
	if (m_header.DeleteHeaderHdl(get_client_memptr(wlan_client, clt_indx)->hdr_hdl_v6)
			== false)
	{
		return IPACM_FAILURE;
	}
		get_client_memptr(wlan_client, clt_indx)->ipv6_header_set = false;
	}

	/* Reset ip_set to 0*/
	get_client_memptr(wlan_client, clt_indx)->ipv4_set = false;
	get_client_memptr(wlan_client, clt_indx)->ipv6_set = 0;
	get_client_memptr(wlan_client, clt_indx)->ipv4_header_set = false;
	get_client_memptr(wlan_client, clt_indx)->ipv6_header_set = false;
	get_client_memptr(wlan_client, clt_indx)->route_rule_set_v4 = false;
	get_client_memptr(wlan_client, clt_indx)->route_rule_set_v6 = 0;
	free(get_client_memptr(wlan_client, clt_indx)->p_hdr_info);

	for (; clt_indx < num_wifi_client_tmp - 1; clt_indx++)
	{
		get_client_memptr(wlan_client, clt_indx)->p_hdr_info = get_client_memptr(wlan_client, (clt_indx + 1))->p_hdr_info;

		memcpy(get_client_memptr(wlan_client, clt_indx)->mac,
					 get_client_memptr(wlan_client, (clt_indx + 1))->mac,
					 sizeof(get_client_memptr(wlan_client, clt_indx)->mac));

		get_client_memptr(wlan_client, clt_indx)->hdr_hdl_v4 = get_client_memptr(wlan_client, (clt_indx + 1))->hdr_hdl_v4;
		get_client_memptr(wlan_client, clt_indx)->hdr_hdl_v6 = get_client_memptr(wlan_client, (clt_indx + 1))->hdr_hdl_v6;
		get_client_memptr(wlan_client, clt_indx)->v4_addr = get_client_memptr(wlan_client, (clt_indx + 1))->v4_addr;

		get_client_memptr(wlan_client, clt_indx)->ipv4_set = get_client_memptr(wlan_client, (clt_indx + 1))->ipv4_set;
		get_client_memptr(wlan_client, clt_indx)->ipv6_set = get_client_memptr(wlan_client, (clt_indx + 1))->ipv6_set;
		get_client_memptr(wlan_client, clt_indx)->ipv4_header_set = get_client_memptr(wlan_client, (clt_indx + 1))->ipv4_header_set;
		get_client_memptr(wlan_client, clt_indx)->ipv6_header_set = get_client_memptr(wlan_client, (clt_indx + 1))->ipv6_header_set;

		get_client_memptr(wlan_client, clt_indx)->route_rule_set_v4 = get_client_memptr(wlan_client, (clt_indx + 1))->route_rule_set_v4;
		get_client_memptr(wlan_client, clt_indx)->route_rule_set_v6 = get_client_memptr(wlan_client, (clt_indx + 1))->route_rule_set_v6;

                for(num_v6=0;num_v6< get_client_memptr(wlan_client, clt_indx)->ipv6_set;num_v6++)
	        {
		    get_client_memptr(wlan_client, clt_indx)->v6_addr[num_v6][0] = get_client_memptr(wlan_client, (clt_indx + 1))->v6_addr[num_v6][0];
		    get_client_memptr(wlan_client, clt_indx)->v6_addr[num_v6][1] = get_client_memptr(wlan_client, (clt_indx + 1))->v6_addr[num_v6][1];
		    get_client_memptr(wlan_client, clt_indx)->v6_addr[num_v6][2] = get_client_memptr(wlan_client, (clt_indx + 1))->v6_addr[num_v6][2];
		    get_client_memptr(wlan_client, clt_indx)->v6_addr[num_v6][3] = get_client_memptr(wlan_client, (clt_indx + 1))->v6_addr[num_v6][3];
                }

		for (tx_index = 0; tx_index < iface_query->num_tx_props; tx_index++)
		{
			get_client_memptr(wlan_client, clt_indx)->wifi_rt_hdl[tx_index].wifi_rt_rule_hdl_v4 =
				 get_client_memptr(wlan_client, (clt_indx + 1))->wifi_rt_hdl[tx_index].wifi_rt_rule_hdl_v4;

			for(num_v6=0;num_v6< get_client_memptr(wlan_client, clt_indx)->route_rule_set_v6;num_v6++)
			{
			  get_client_memptr(wlan_client, clt_indx)->wifi_rt_hdl[tx_index].wifi_rt_rule_hdl_v6[num_v6] =
			   	 get_client_memptr(wlan_client, (clt_indx + 1))->wifi_rt_hdl[tx_index].wifi_rt_rule_hdl_v6[num_v6];
			  get_client_memptr(wlan_client, clt_indx)->wifi_rt_hdl[tx_index].wifi_rt_rule_hdl_v6_wan[num_v6] =
			   	 get_client_memptr(wlan_client, (clt_indx + 1))->wifi_rt_hdl[tx_index].wifi_rt_rule_hdl_v6_wan[num_v6];
		    }
		}
	}

	IPACMDBG_H(" %d wifi client deleted successfully \n", num_wifi_client);
	num_wifi_client = num_wifi_client - 1;
	IPACM_Wlan::total_num_wifi_clients = IPACM_Wlan::total_num_wifi_clients - 1;
	IPACMDBG_H(" Number of wifi client: %d\n", num_wifi_client);

	return IPACM_SUCCESS;
}

/*handle wlan iface down event*/
int IPACM_Wlan::handle_down_evt()
{
	int res = IPACM_SUCCESS, i;

	IPACMDBG_H("WLAN ip-type: %d \n", ip_type);
	/* no iface address up, directly close iface*/
	if (ip_type == IPACM_IP_NULL)
	{
		IPACMERR("Invalid iptype: 0x%x\n", ip_type);
		goto fail;
	}
#ifdef FEATURE_ETH_BRIDGE_LE
	if(wlan_ap_index == 0)
	{
		IPACM_Lan::wlan_hdr_type = IPA_HDR_L2_NONE;
		IPACM_Lan::wlan_hdr_template_hdl = 0;
		del_hdr_proc_ctx();
	}
#endif

	/* delete wan filter rule */
	if (IPACM_Wan::isWanUP(ipa_if_num) && rx_prop != NULL)
	{
		IPACMDBG_H("LAN IF goes down, backhaul type %d\n", IPACM_Wan::backhaul_is_sta_mode);
		IPACM_Lan::handle_wan_down(IPACM_Wan::backhaul_is_sta_mode);
	}

	if (IPACM_Wan::isWanUP_V6(ipa_if_num) && rx_prop != NULL)
	{
		IPACMDBG_H("LAN IF goes down, backhaul type %d\n", IPACM_Wan::backhaul_is_sta_mode);
		IPACM_Lan::handle_wan_down_v6(IPACM_Wan::backhaul_is_sta_mode);
	}
	IPACMDBG_H("finished deleting wan filtering rules\n ");

	/* Delete v4 filtering rules */
	if (ip_type != IPA_IP_v6 && rx_prop != NULL)
	{
		/* delete IPv4 icmp filter rules */
		if(wlan_ap_index == 0)
		{
			if(m_filtering.DeleteFilteringHdls(ipv4_icmp_flt_rule_hdl, IPA_IP_v4, NUM_IPV4_ICMP_FLT_RULE) == false)
			{
				IPACMERR("Error Deleting ICMPv4 Filtering Rule, aborting...\n");
				res = IPACM_FAILURE;
				goto fail;
			}
			IPACM_Iface::ipacmcfg->decreaseFltRuleCount(rx_prop->rx[0].src_pipe, IPA_IP_v4, NUM_IPV4_ICMP_FLT_RULE);
		}
#ifdef FEATURE_ETH_BRIDGE_LE
		if(wlan_ap_index == 0)
		{
			/* delete default filter rules */
			for(i=0; i<IPV4_DEFAULT_FILTERTING_RULES; i++)
			{
				if(reset_to_dummy_flt_rule(IPA_IP_v4, dft_v4fl_rule_hdl[i]) == IPACM_FAILURE)
				{
					IPACMERR("Error deleting dft IPv4 flt rules.\n");
					res = IPACM_FAILURE;
					goto fail;
				}
			}
		}
#else
		/* delete default filter rules */
		for(i=0; i<IPV4_DEFAULT_FILTERTING_RULES; i++)
		{
			if(reset_to_dummy_flt_rule(IPA_IP_v4, dft_v4fl_rule_hdl[i]) == IPACM_FAILURE)
			{
				IPACMERR("Error deleting dft IPv4 flt rules.\n");
				res = IPACM_FAILURE;
				goto fail;
			}
		}
#endif
		IPACMDBG_H("Deleted default v4 filter rules successfully.\n");
#ifndef FEATURE_ETH_BRIDGE_LE
#ifdef CT_OPT
		IPACMDBG_H("Delete tcp control flt rules.\n");
		/* Delete tcp control flt rules */
		for(i=0; i<NUM_TCP_CTL_FLT_RULE; i++)
		{
			if(reset_to_dummy_flt_rule(IPA_IP_v4, tcp_ctl_flt_rule_hdl_v4[i]) == IPACM_FAILURE)
			{
				res = IPACM_FAILURE;
				goto fail;
			}
		}
#endif
		IPACMDBG_H("Delete lan2lan v4 flt rules.\n");
		/* delete lan2lan ipv4 flt rules */
		for(i=0; i<MAX_OFFLOAD_PAIR; i++)
		{
			if(reset_to_dummy_flt_rule(IPA_IP_v4, lan2lan_flt_rule_hdl_v4[i].rule_hdl) == IPACM_FAILURE)
			{
				IPACMERR("Error deleting lan2lan IPv4 flt rules.\n");
				res = IPACM_FAILURE;
				goto fail;
			}
		}
#endif

		IPACMDBG_H("Delete private v4 filter rules\n");
		/* delete private-ipv4 filter rules */
#ifdef FEATURE_IPA_ANDROID
		for(i=0; i<IPA_MAX_PRIVATE_SUBNET_ENTRIES; i++)
		{
			if(reset_to_dummy_flt_rule(IPA_IP_v4, private_fl_rule_hdl[i]) == IPACM_FAILURE)
			{
				IPACMERR("Error deleting private subnet IPv4 flt rules.\n");
				res = IPACM_FAILURE;
				goto fail;
			}
		}
#else
		if(wlan_ap_index == 0)
		{
			for(i=0; i<IPACM_Iface::ipacmcfg->ipa_num_private_subnet; i++)
			{
				if(reset_to_dummy_flt_rule(IPA_IP_v4, private_fl_rule_hdl[i]) == IPACM_FAILURE)
				{
					IPACMERR("Error deleting private subnet IPv4 flt rules.\n");
					res = IPACM_FAILURE;
					goto fail;
				}
			}
		}
		IPACMDBG_H("Deleted private subnet v4 filter rules successfully.\n");
#endif
	}

	/* Delete v6 filtering rules */
	if (ip_type != IPA_IP_v4 && rx_prop != NULL)
	{
		/* delete icmp filter rules */
		if(wlan_ap_index == 0)
		{
			if(m_filtering.DeleteFilteringHdls(ipv6_icmp_flt_rule_hdl, IPA_IP_v6, NUM_IPV6_ICMP_FLT_RULE) == false)
			{
				IPACMERR("Error Deleting ICMPv6 Filtering Rule, aborting...\n");
				res = IPACM_FAILURE;
				goto fail;
			}
			IPACM_Iface::ipacmcfg->decreaseFltRuleCount(rx_prop->rx[0].src_pipe, IPA_IP_v6, NUM_IPV6_ICMP_FLT_RULE);
		}
		IPACMDBG_H("Delete default %d v6 filter rules\n", IPV6_DEFAULT_FILTERTING_RULES);
		/* delete default filter rules */
#ifdef FEATURE_ETH_BRIDGE_LE
		if(wlan_ap_index == 0)
		{
			for(i=0; i<IPV6_DEFAULT_FILTERTING_RULES; i++)
			{
				if(reset_to_dummy_flt_rule(IPA_IP_v6, dft_v6fl_rule_hdl[i]) == IPACM_FAILURE)
				{
					res = IPACM_FAILURE;
					goto fail;
				}
			}
		}
#else
		for(i=0; i<IPV6_DEFAULT_FILTERTING_RULES; i++)
		{
			if(reset_to_dummy_flt_rule(IPA_IP_v6, dft_v6fl_rule_hdl[i]) == IPACM_FAILURE)
			{
				res = IPACM_FAILURE;
				goto fail;
			}
		}
#endif
		IPACMDBG_H("Deleted default v6 filter rules successfully.\n");

#ifndef FEATURE_ETH_BRIDGE_LE
#ifdef CT_OPT
		IPACMDBG_H("Delete tcp control flt rules.\n");
		/* Delete tcp control flt rules */
		for(i=0; i<NUM_TCP_CTL_FLT_RULE; i++)
		{
			if(reset_to_dummy_flt_rule(IPA_IP_v6, tcp_ctl_flt_rule_hdl_v6[i]) == IPACM_FAILURE)
			{
				res = IPACM_FAILURE;
				goto fail;
			}
		}
#endif
		IPACMDBG_H("Delete lan2lan v6 flt rules.\n");
		/* delete lan2lan ipv4 flt rules */
		for(i=0; i<MAX_OFFLOAD_PAIR; i++)
		{
			if(reset_to_dummy_flt_rule(IPA_IP_v6, lan2lan_flt_rule_hdl_v6[i].rule_hdl) == IPACM_FAILURE)
			{
				IPACMERR("Error deleting lan2lan IPv4 flt rules.\n");
				res = IPACM_FAILURE;
				goto fail;
			}
		}
#endif
	}
	IPACMDBG_H("finished delete filtering rules\n ");

	/* Delete default v4 RT rule */
	if (ip_type != IPA_IP_v6)
	{
		IPACMDBG_H("Delete default v4 routing rules\n");
		if (m_routing.DeleteRoutingHdl(dft_rt_rule_hdl[0], IPA_IP_v4)
				== false)
		{
			IPACMERR("Routing rule deletion failed!\n");
			res = IPACM_FAILURE;
			goto fail;
		}
	}

	/* Delete default v6 RT rule */
	if (ip_type != IPA_IP_v4)
	{
		IPACMDBG_H("Delete default v6 routing rules\n");
		/* May have multiple ipv6 iface-RT rules */
		for (i = 0; i < 2*num_dft_rt_v6; i++)
		{
			if (m_routing.DeleteRoutingHdl(dft_rt_rule_hdl[MAX_DEFAULT_v4_ROUTE_RULES+i], IPA_IP_v6)
					== false)
			{
				IPACMERR("Routing rule deletion failed!\n");
				res = IPACM_FAILURE;
				goto fail;
			}
		}
	}
	IPACMDBG_H("finished deleting default RT rules\n ");

	/* check software routing fl rule hdl */
	if (softwarerouting_act == true && rx_prop != NULL )
	{
		IPACMDBG_H("Delete sw routing filtering rules\n");
		IPACM_Iface::handle_software_routing_disable();
	}
	IPACMDBG_H("finished delete software-routing filtering rules\n ");


	/* clean wifi-client header, routing rules */
	/* clean wifi client rule*/
	IPACMDBG_H("left %d wifi clients need to be deleted \n ", num_wifi_client);

	for (i = 0; i < num_wifi_client; i++)
	{
#ifdef FEATURE_ETH_BRIDGE_LE
		eth_bridge_del_self_client_flt_rule(get_client_memptr(wlan_client, i)->mac);
		eth_bridge_del_wlan_client_rt_rule(get_client_memptr(wlan_client, i)->mac, SRC_WLAN);
		eth_bridge_del_wlan_client(get_client_memptr(wlan_client, i)->mac);
		if (is_guest_ap == false)
		{
			eth_bridge_del_wlan_client_rt_rule(get_client_memptr(wlan_client, i)->mac, SRC_LAN);
			eth_bridge_post_lan_client_event(get_client_memptr(wlan_client, i)->mac, IPA_ETH_BRIDGE_WLAN_CLIENT_DEL_EVENT);
		}
#endif
		/* First reset nat rules and then route rules */
		if(get_client_memptr(wlan_client, i)->ipv4_set == true)
		{
	        IPACMDBG_H("Clean Nat Rules for ipv4:0x%x\n", get_client_memptr(wlan_client, i)->v4_addr);
			CtList->HandleNeighIpAddrDelEvt(get_client_memptr(wlan_client, i)->v4_addr);
		}

		if (delete_default_qos_rtrules(i, IPA_IP_v4))
		{
			IPACMERR("unbale to delete v4 default qos route rules for index: %d\n", i);
			res = IPACM_FAILURE;
			goto fail;
		}

		if (delete_default_qos_rtrules(i, IPA_IP_v6))
		{
			IPACMERR("unbale to delete v6 default qos route rules for index: %d\n", i);
			res = IPACM_FAILURE;
			goto fail;
		}

		IPACMDBG_H("Delete %d client header\n", num_wifi_client);

		handle_lan2lan_msg_post(get_client_memptr(wlan_client, i)->mac, IPA_LAN_CLIENT_DISCONNECT, IPA_IP_v4);
		handle_lan2lan_msg_post(get_client_memptr(wlan_client, i)->mac, IPA_LAN_CLIENT_DISCONNECT, IPA_IP_v6);

		if(get_client_memptr(wlan_client, i)->ipv4_header_set == true)
		{
			if (m_header.DeleteHeaderHdl(get_client_memptr(wlan_client, i)->hdr_hdl_v4)
				== false)
			{
				res = IPACM_FAILURE;
				goto fail;
			}
		}

		if(get_client_memptr(wlan_client, i)->ipv6_header_set == true)
		{
			if (m_header.DeleteHeaderHdl(get_client_memptr(wlan_client, i)->hdr_hdl_v6)
					== false)
			{
				res = IPACM_FAILURE;
				goto fail;
			}
		}
	} /* end of for loop */

	/* free the wlan clients cache */
	IPACMDBG_H("Free wlan clients cache\n");

	/* Delete private subnet*/
#ifdef FEATURE_IPA_ANDROID
	if (ip_type != IPA_IP_v6)
	{
		IPACMDBG_H("current IPACM private subnet_addr number(%d)\n", IPACM_Iface::ipacmcfg->ipa_num_private_subnet);
		IPACMDBG_H(" Delete IPACM private subnet_addr as: 0x%x \n", if_ipv4_subnet);
		if(IPACM_Iface::ipacmcfg->DelPrivateSubnet(if_ipv4_subnet, ipa_if_num) == false)
		{
			IPACMERR(" can't Delete IPACM private subnet_addr as: 0x%x \n", if_ipv4_subnet);
		}
	}
	/* reset the IPA-client pipe enum */
	handle_tethering_client(true, IPACM_CLIENT_WLAN);
#endif /* defined(FEATURE_IPA_ANDROID)*/

fail:
	/* Delete corresponding ipa_rm_resource_name of RX-endpoint after delete all IPV4V6 FT-rule */
	if (rx_prop != NULL)
	{
		IPACMDBG_H("dev %s add producer dependency\n", dev_name);
		IPACMDBG_H("depend Got pipe %d rm index : %d \n", rx_prop->rx[0].src_pipe, IPACM_Iface::ipacmcfg->ipa_client_rm_map_tbl[rx_prop->rx[0].src_pipe]);
		IPACM_Iface::ipacmcfg->DelRmDepend(IPACM_Iface::ipacmcfg->ipa_client_rm_map_tbl[rx_prop->rx[0].src_pipe]);
		free(rx_prop);
	}

	for (i = 0; i < num_wifi_client; i++)
	{
		if(get_client_memptr(wlan_client, i)->p_hdr_info != NULL)
		{
			free(get_client_memptr(wlan_client, i)->p_hdr_info);
		}
	}
	if(wlan_client != NULL)
	{
		free(wlan_client);
	}
	if (tx_prop != NULL)
	{
		free(tx_prop);
	}

	if (iface_query != NULL)
	{
		free(iface_query);
	}
#ifdef FEATURE_ETH_BRIDGE_LE
	if(eth_bridge_lan_client_rt_from_lan_info_v4 != NULL)
	{
		free(eth_bridge_lan_client_rt_from_lan_info_v4);
	}
	if(eth_bridge_lan_client_rt_from_lan_info_v6 != NULL)
	{
		free(eth_bridge_lan_client_rt_from_lan_info_v6);
	}
	if(eth_bridge_lan_client_rt_from_wlan_info_v4 != NULL)
	{
		free(eth_bridge_lan_client_rt_from_wlan_info_v4);
	}
	if(eth_bridge_lan_client_rt_from_wlan_info_v6 != NULL)
	{
		free(eth_bridge_lan_client_rt_from_wlan_info_v6);
	}
	if(eth_bridge_wlan_client_rt_from_lan_info_v4 != NULL)
	{
		free(eth_bridge_wlan_client_rt_from_lan_info_v4);
	}
	if(eth_bridge_wlan_client_rt_from_lan_info_v6 != NULL)
	{
		free(eth_bridge_wlan_client_rt_from_lan_info_v6);
	}
	if(eth_bridge_wlan_client_rt_from_wlan_info_v4 != NULL)
	{
		free(eth_bridge_wlan_client_rt_from_wlan_info_v4);
	}
	if(eth_bridge_wlan_client_rt_from_wlan_info_v6 != NULL)
	{
		free(eth_bridge_wlan_client_rt_from_wlan_info_v6);
	}
#endif
	is_active = false;
	post_del_self_evt();

	return res;
}

/*handle reset wifi-client rt-rules */
int IPACM_Wlan::handle_wlan_client_reset_rt(ipa_ip_type iptype)
{
	int i, res = IPACM_SUCCESS;

	/* clean wifi-client routing rules */
	IPACMDBG_H("left %d wifi clients to reset ip-type(%d) rules \n ", num_wifi_client, iptype);

	for (i = 0; i < num_wifi_client; i++)
	{
		/* Reset RT rules */
		res = delete_default_qos_rtrules(i, iptype);
		if (res != IPACM_SUCCESS)
		{
			IPACMERR("Failed to delete old iptype(%d) rules.\n", iptype);
			return res;
		}
		/* Pass info to LAN2LAN module */
		res = handle_lan2lan_msg_post(get_client_memptr(wlan_client, i)->mac, IPA_LAN_CLIENT_DISCONNECT, iptype);
		if (res != IPACM_SUCCESS)
		{
			IPACMERR("Failed to posting delete old iptype(%d) address.\n", iptype);
			return res;
		}
		/* Reset ip-address */
		if(iptype == IPA_IP_v4)
		{
			get_client_memptr(wlan_client, i)->ipv4_set = false;
		}
		else
		{
			get_client_memptr(wlan_client, i)->ipv6_set = 0;
		}
	} /* end of for loop */
	return res;
}

/*handle lan2lan internal mesg posting*/
int IPACM_Wlan::handle_lan2lan_msg_post(uint8_t *mac_addr, ipa_cm_event_id event,ipa_ip_type iptype)
{
	int client_index;
	client_index = get_wlan_client_index(mac_addr);
	if (client_index == IPACM_INVALID_INDEX)
	{
		IPACMDBG_H("wlan client not attached\n");
		return IPACM_SUCCESS;
	}

	ipacm_event_lan_client* lan_client;
	ipacm_cmd_q_data evt_data;
	if((get_client_memptr(wlan_client, client_index)->ipv4_set == true)
		&& (iptype == IPA_IP_v4)) /* handle ipv4 case*/
	{
		if(ip_type != IPA_IP_v4 && ip_type != IPA_IP_MAX)
		{
			IPACMERR("Client has IPv4 addr but iface does not have IPv4 up.\n");
			return IPACM_FAILURE;
		}

		lan_client = (ipacm_event_lan_client*)malloc(sizeof(ipacm_event_lan_client));
		if(lan_client == NULL)
		{
			IPACMERR("Unable to allocate memory.\n");
			return IPACM_FAILURE;
		}
		memset(lan_client, 0, sizeof(ipacm_event_lan_client));

		lan_client->iptype = IPA_IP_v4;
		lan_client->ipv4_addr = get_client_memptr(wlan_client, client_index)->v4_addr;
		lan_client->p_iface = this;

		memset(&evt_data, 0, sizeof(evt_data));
		evt_data.event = event;
		evt_data.evt_data = (void*)lan_client;

		IPACMDBG_H("Posting event: %d\n",event);
		IPACM_EvtDispatcher::PostEvt(&evt_data);
	}

	if((get_client_memptr(wlan_client, client_index)->ipv6_set > 0)
		&& (iptype == IPA_IP_v6)) /* handle v6 case: may be multiple v6 addr */
	{
		if(ip_type != IPA_IP_v6 && ip_type != IPA_IP_MAX)
		{
			IPACMERR("Client has IPv6 addr but iface does not have IPv6 up.\n");
			return IPACM_FAILURE;
		}
		int i;

		for(i=0; i<get_client_memptr(wlan_client, client_index)->ipv6_set; i++)
		{
			lan_client = (ipacm_event_lan_client*)malloc(sizeof(ipacm_event_lan_client));
			if(lan_client == NULL)
			{
				IPACMERR("Unable to allocate memory.\n");
				return IPACM_FAILURE;
			}
			memset(lan_client, 0, sizeof(ipacm_event_lan_client));

			lan_client->iptype = IPA_IP_v6;
			memcpy(lan_client->ipv6_addr, get_client_memptr(wlan_client, client_index)->v6_addr[i], 4*sizeof(uint32_t));
			lan_client->p_iface = this;

			memset(&evt_data, 0, sizeof(evt_data));
			evt_data.event = event;
			evt_data.evt_data = (void*)lan_client;

			IPACMDBG_H("Posting event: %d\n",event);
			IPACM_EvtDispatcher::PostEvt(&evt_data);
		}
	}
	return IPACM_SUCCESS;
}

int IPACM_Wlan::add_lan2lan_hdr(ipa_ip_type iptype, uint8_t* src_mac, uint8_t* dst_mac, uint32_t* hdr_hdl)
{
	if(tx_prop == NULL)
	{
		IPACMERR("There is no tx_prop, cannot add header.\n");
		return IPACM_FAILURE;
	}
	if(src_mac == NULL || dst_mac == NULL)
	{
		IPACMERR("Either src_mac or dst_mac is null, cannot add header.\n");
		return IPACM_FAILURE;
	}
	if(hdr_hdl == NULL)
	{
		IPACMERR("Header handle is empty.\n");
		return IPACM_FAILURE;
	}

	int i, j, k, len;
	int res = IPACM_SUCCESS;
	char index[4];
	struct ipa_ioc_copy_hdr sCopyHeader;
	struct ipa_ioc_add_hdr *pHeader;

	IPACMDBG_H("Get lan2lan header request, src_mac: 0x%02x%02x%02x%02x%02x%02x dst_mac: 0x%02x%02x%02x%02x%02x%02x\n",
			src_mac[0], src_mac[1], src_mac[2], src_mac[3], src_mac[4], src_mac[5], dst_mac[0], dst_mac[1],
			dst_mac[2], dst_mac[3], dst_mac[4], dst_mac[5]);

	len = sizeof(struct ipa_ioc_add_hdr) + sizeof(struct ipa_hdr_add);
	pHeader = (struct ipa_ioc_add_hdr *)malloc(len);
	if (pHeader == NULL)
	{
		IPACMERR("Failed to allocate header\n");
		return IPACM_FAILURE;
	}
	memset(pHeader, 0, len);

	if(iptype == IPA_IP_v4)
	{		/* copy partial header for v4*/
		for(i=0; i<tx_prop->num_tx_props; i++)
		{
			if(tx_prop->tx[i].ip == IPA_IP_v4)
			{
				IPACMDBG_H("Got v4-header name from %d tx props\n", i);
				memset(&sCopyHeader, 0, sizeof(sCopyHeader));
				memcpy(sCopyHeader.name, tx_prop->tx[i].hdr_name, sizeof(sCopyHeader.name));

				IPACMDBG_H("Header name: %s\n", sCopyHeader.name);
				if(m_header.CopyHeader(&sCopyHeader) == false)
				{
					IPACMERR("Copy header failed\n");
					res = IPACM_FAILURE;
					goto fail;
				}

				IPACMDBG_H("Header length: %d, paritial: %d\n", sCopyHeader.hdr_len, sCopyHeader.is_partial);
				if (sCopyHeader.hdr_len > IPA_HDR_MAX_SIZE)
				{
					IPACMERR("Header oversize\n");
					res = IPACM_FAILURE;
					goto fail;
				}
				else
				{
					memcpy(pHeader->hdr[0].hdr, sCopyHeader.hdr, sCopyHeader.hdr_len);
				}

				for(j=0; j<num_wifi_client; j++)	//Add src/dst mac to the header
				{
					if(memcmp(dst_mac, get_client_memptr(wlan_client, j)->mac, IPA_MAC_ADDR_SIZE) == 0)
					{
						break;
					}
				}
				if(j == num_wifi_client)
				{
					IPACMERR("Not able to find the wifi client from mac addr.\n");
					res = IPACM_FAILURE;
					goto fail;
				}
				else
				{
					IPACMDBG_H("Find wifi client at position %d\n", j);
					for(k = 0; k < get_client_memptr(wlan_client, j)->p_hdr_info->num_of_attribs; k++)
					{
						if(get_client_memptr(wlan_client, j)->p_hdr_info->attribs[k].attrib_type == WLAN_HDR_ATTRIB_MAC_ADDR)
						{
							memcpy(&pHeader->hdr[0].hdr[get_client_memptr(wlan_client, j)->p_hdr_info->attribs[k].offset],
									dst_mac, IPA_MAC_ADDR_SIZE);
							memcpy(&pHeader->hdr[0].hdr[get_client_memptr(wlan_client, j)->p_hdr_info->attribs[k].offset + IPA_MAC_ADDR_SIZE],
									src_mac, IPA_MAC_ADDR_SIZE);
						}
						else if(get_client_memptr(wlan_client, j)->p_hdr_info->attribs[k].attrib_type == WLAN_HDR_ATTRIB_STA_ID)
						{
							memcpy(&pHeader->hdr[0].hdr[get_client_memptr(wlan_client, j)->p_hdr_info->attribs[k].offset],
									&get_client_memptr(wlan_client, j)->p_hdr_info->attribs[k].u.sta_id,
									sizeof(get_client_memptr(wlan_client, j)->p_hdr_info->attribs[k].u.sta_id));
						}
						else
						{
							IPACMDBG_H("The attribute type is not expected!\n");
						}
					}
				}

				pHeader->commit = true;
				pHeader->num_hdrs = 1;

				memset(pHeader->hdr[0].name, 0, sizeof(pHeader->hdr[0].name));
				strlcpy(pHeader->hdr[0].name, IPA_LAN_TO_LAN_WLAN_HDR_NAME_V4, sizeof(pHeader->hdr[0].name));
				pHeader->hdr[0].name[IPA_RESOURCE_NAME_MAX-1] = '\0';
				for(j=0; j<MAX_OFFLOAD_PAIR; j++)
				{
					if( lan2lan_hdr_hdl_v4[j].valid == false)
					{
						IPACMDBG_H("Construct lan2lan hdr with index %d.\n", j);
						break;
					}
				}
				if(j == MAX_OFFLOAD_PAIR)
				{
					IPACMERR("Failed to find an available hdr index.\n");
					res = IPACM_FAILURE;
					goto fail;
				}
				lan2lan_hdr_hdl_v4[j].valid = true;
				snprintf(index,sizeof(index), "%d", j);

				if (strlcat(pHeader->hdr[0].name, index, sizeof(pHeader->hdr[0].name)) > IPA_RESOURCE_NAME_MAX)
				{
					IPACMERR(" header name construction failed exceed length (%d)\n", strlen(pHeader->hdr[0].name));
					res = IPACM_FAILURE;
					goto fail;
				}

				pHeader->hdr[0].hdr_len = sCopyHeader.hdr_len;
				pHeader->hdr[0].is_partial = 0;
				pHeader->hdr[0].hdr_hdl = -1;
				pHeader->hdr[0].status = -1;

				if (m_header.AddHeader(pHeader) == false || pHeader->hdr[0].status != 0)
				{
					IPACMERR("Ioctl IPA_IOC_ADD_HDR failed with status: %d\n", pHeader->hdr[0].status);
					res = IPACM_FAILURE;
					goto fail;
				}
				IPACMDBG_H("Installed v4 full header %s header handle 0x%08x\n", pHeader->hdr[0].name,
							pHeader->hdr[0].hdr_hdl);
				*hdr_hdl = pHeader->hdr[0].hdr_hdl;
				lan2lan_hdr_hdl_v4[j].hdr_hdl = pHeader->hdr[0].hdr_hdl;
				break;
			}
		}
	}
	else if(iptype == IPA_IP_v6)
	{		/* copy partial header for v6*/
		for(i=0; i<tx_prop->num_tx_props; i++)
		{
			if(tx_prop->tx[i].ip == IPA_IP_v6)
			{
				IPACMDBG_H("Got v6-header name from %d tx props\n", i);
				memset(&sCopyHeader, 0, sizeof(sCopyHeader));
				memcpy(sCopyHeader.name, tx_prop->tx[i].hdr_name, sizeof(sCopyHeader.name));

				IPACMDBG_H("Header name: %s\n", sCopyHeader.name);
				if(m_header.CopyHeader(&sCopyHeader) == false)
				{
					IPACMERR("Copy header failed\n");
					res = IPACM_FAILURE;
					goto fail;
				}

				IPACMDBG_H("Header length: %d, paritial: %d\n", sCopyHeader.hdr_len, sCopyHeader.is_partial);
				if (sCopyHeader.hdr_len > IPA_HDR_MAX_SIZE)
				{
					IPACMERR("Header oversize\n");
					res = IPACM_FAILURE;
					goto fail;
				}
				else
				{
					memcpy(pHeader->hdr[0].hdr, sCopyHeader.hdr, sCopyHeader.hdr_len);
				}

				for(j=0; j<num_wifi_client; j++)	//Add src/dst mac to the header
				{
					if(memcmp(dst_mac, get_client_memptr(wlan_client, j)->mac, IPA_MAC_ADDR_SIZE) == 0)
					{
						break;
					}
				}
				if(j == num_wifi_client)
				{
					IPACMERR("Not able to find the wifi client from mac addr.\n");
					res = IPACM_FAILURE;
					goto fail;
				}
				else
				{
					IPACMDBG_H("Find wifi client at position %d\n", j);
					for(k = 0; k < get_client_memptr(wlan_client, j)->p_hdr_info->num_of_attribs; k++)
					{
						if(get_client_memptr(wlan_client, j)->p_hdr_info->attribs[k].attrib_type == WLAN_HDR_ATTRIB_MAC_ADDR)
						{
							memcpy(&pHeader->hdr[0].hdr[get_client_memptr(wlan_client, j)->p_hdr_info->attribs[k].offset],
									dst_mac, IPA_MAC_ADDR_SIZE);
							memcpy(&pHeader->hdr[0].hdr[get_client_memptr(wlan_client, j)->p_hdr_info->attribs[k].offset + IPA_MAC_ADDR_SIZE],
									src_mac, IPA_MAC_ADDR_SIZE);
						}
						else if(get_client_memptr(wlan_client, j)->p_hdr_info->attribs[k].attrib_type == WLAN_HDR_ATTRIB_STA_ID)
						{
							memcpy(&pHeader->hdr[0].hdr[get_client_memptr(wlan_client, j)->p_hdr_info->attribs[k].offset],
									&get_client_memptr(wlan_client, j)->p_hdr_info->attribs[k].u.sta_id,
									sizeof(get_client_memptr(wlan_client, j)->p_hdr_info->attribs[k].u.sta_id));
						}
						else
						{
							IPACMDBG_H("The attribute type is not expected!\n");
						}
					}
				}

				pHeader->commit = true;
				pHeader->num_hdrs = 1;

				memset(pHeader->hdr[0].name, 0, sizeof(pHeader->hdr[0].name));
				strlcpy(pHeader->hdr[0].name, IPA_LAN_TO_LAN_WLAN_HDR_NAME_V6, sizeof(pHeader->hdr[0].name));
				pHeader->hdr[0].name[IPA_RESOURCE_NAME_MAX-1] = '\0';

				for(j=0; j<MAX_OFFLOAD_PAIR; j++)
				{
					if( lan2lan_hdr_hdl_v6[j].valid == false)
					{
						IPACMDBG_H("Construct lan2lan hdr with index %d.\n", j);
						break;
					}
				}
				if(j == MAX_OFFLOAD_PAIR)
				{
					IPACMERR("Failed to find an available hdr index.\n");
					res = IPACM_FAILURE;
					goto fail;
				}
				lan2lan_hdr_hdl_v6[j].valid = true;
				snprintf(index,sizeof(index), "%d", j);

				if (strlcat(pHeader->hdr[0].name, index, sizeof(pHeader->hdr[0].name)) > IPA_RESOURCE_NAME_MAX)
				{
					IPACMERR(" header name construction failed exceed length (%d)\n", strlen(pHeader->hdr[0].name));
					res = IPACM_FAILURE;
					goto fail;
				}
				pHeader->hdr[0].hdr_len = sCopyHeader.hdr_len;
				pHeader->hdr[0].is_partial = 0;
				pHeader->hdr[0].hdr_hdl = -1;
				pHeader->hdr[0].status = -1;

				if (m_header.AddHeader(pHeader) == false || pHeader->hdr[0].status != 0)
				{
					IPACMERR("Ioctl IPA_IOC_ADD_HDR failed with status: %d\n", pHeader->hdr[0].status);
					res = IPACM_FAILURE;
					goto fail;
				}
				IPACMDBG_H("Installed v6 full header %s header handle 0x%08x\n", pHeader->hdr[0].name,
							pHeader->hdr[0].hdr_hdl);
				*hdr_hdl = pHeader->hdr[0].hdr_hdl;
				lan2lan_hdr_hdl_v6[j].hdr_hdl = pHeader->hdr[0].hdr_hdl;
				break;
			}
		}
	}
	else
	{
		IPACMERR("IP type is not expected.\n");
	}

fail:
	free(pHeader);
	return res;
}

/* add dummy filtering rules for WLAN AP-AP mode support */
void IPACM_Wlan::add_dummy_flt_rule()
{
	int num_v4_dummy_rule, num_v6_dummy_rule;

	if(IPACM_Wlan::num_wlan_ap_iface == 1)
	{
		if(IPACM_Wlan::dummy_flt_rule_hdl_v4 != NULL || IPACM_Wlan::dummy_flt_rule_hdl_v6 != NULL)
		{
			IPACMERR("Either v4 or v6 dummy filtering rule handle is not empty.\n");
			return;
		}
#ifdef FEATURE_ETH_BRIDGE_LE
		num_v4_dummy_rule = IPV4_DEFAULT_FILTERTING_RULES + IPA_LAN_TO_LAN_MAX_WLAN_CLIENT + IPA_LAN_TO_LAN_MAX_LAN_CLIENT + IPACM_Iface::ipacmcfg->ipa_num_private_subnet;
		num_v6_dummy_rule = IPV6_DEFAULT_FILTERTING_RULES + IPA_LAN_TO_LAN_MAX_WLAN_CLIENT + IPA_LAN_TO_LAN_MAX_LAN_CLIENT;
#else
#ifndef CT_OPT
		num_v4_dummy_rule = 2*(IPV4_DEFAULT_FILTERTING_RULES + MAX_OFFLOAD_PAIR + IPACM_Iface::ipacmcfg->ipa_num_private_subnet);
		num_v6_dummy_rule = 2*(IPV6_DEFAULT_FILTERTING_RULES + MAX_OFFLOAD_PAIR);
#else
		num_v4_dummy_rule = 2*(IPV4_DEFAULT_FILTERTING_RULES + NUM_TCP_CTL_FLT_RULE + MAX_OFFLOAD_PAIR + IPACM_Iface::ipacmcfg->ipa_num_private_subnet);
		num_v6_dummy_rule = 2*(IPV6_DEFAULT_FILTERTING_RULES + NUM_TCP_CTL_FLT_RULE + MAX_OFFLOAD_PAIR);
#endif
#ifdef FEATURE_IPA_ANDROID
		num_v4_dummy_rule = num_v4_dummy_rule - 2* IPACM_Iface::ipacmcfg->ipa_num_private_subnet + 2 * IPA_MAX_PRIVATE_SUBNET_ENTRIES;
#endif
#endif

		IPACM_Wlan::dummy_flt_rule_hdl_v4 = (uint32_t*)malloc(num_v4_dummy_rule * sizeof(uint32_t));
		if(IPACM_Wlan::dummy_flt_rule_hdl_v4 == NULL)
		{
			IPACMERR("Failed to allocate memory.\n");
			return;
		}
		IPACM_Wlan::dummy_flt_rule_hdl_v6 = (uint32_t*)malloc(num_v6_dummy_rule * sizeof(uint32_t));
		if(IPACM_Wlan::dummy_flt_rule_hdl_v6 == NULL)
		{
			IPACMERR("Failed to allocate memory.\n");
			free(IPACM_Wlan::dummy_flt_rule_hdl_v4);
			IPACM_Wlan::dummy_flt_rule_hdl_v4 = NULL;
			return;
		}
		memset(IPACM_Wlan::dummy_flt_rule_hdl_v4, 0, num_v4_dummy_rule * sizeof(uint32_t));
		memset(IPACM_Wlan::dummy_flt_rule_hdl_v6, 0, num_v6_dummy_rule * sizeof(uint32_t));

		install_dummy_flt_rule(IPA_IP_v4, num_v4_dummy_rule);
		install_dummy_flt_rule(IPA_IP_v6, num_v6_dummy_rule);
	}
	return;
}

/* install dummy filtering rules for WLAN AP-AP mode support */
int IPACM_Wlan::install_dummy_flt_rule(ipa_ip_type iptype, int num_rule)
{
	if(rx_prop == NULL)
	{
		IPACMDBG_H("There is no rx_prop for iface %s, not able to add dummy filtering rule.\n", dev_name);
		return IPACM_FAILURE;
	}

	int i, len, res = IPACM_SUCCESS;
	struct ipa_flt_rule_add flt_rule;
	ipa_ioc_add_flt_rule* pFilteringTable;

	len = sizeof(struct ipa_ioc_add_flt_rule) + num_rule * sizeof(struct ipa_flt_rule_add);

	pFilteringTable = (struct ipa_ioc_add_flt_rule *)malloc(len);
	if (pFilteringTable == NULL)
	{
		IPACMERR("Error allocate flt table memory...\n");
		return IPACM_FAILURE;
	}
	memset(pFilteringTable, 0, len);

	pFilteringTable->commit = 1;
	pFilteringTable->ep = rx_prop->rx[0].src_pipe;
	pFilteringTable->global = false;
	pFilteringTable->ip = iptype;
	pFilteringTable->num_rules = num_rule;

	memset(&flt_rule, 0, sizeof(struct ipa_flt_rule_add));

	flt_rule.rule.retain_hdr = 0;
	flt_rule.at_rear = true;
	flt_rule.flt_rule_hdl = -1;
	flt_rule.status = -1;
	flt_rule.rule.action = IPA_PASS_TO_EXCEPTION;
	memcpy(&flt_rule.rule.attrib, &rx_prop->rx[0].attrib,
			sizeof(flt_rule.rule.attrib));

	if(iptype == IPA_IP_v4)
	{
		flt_rule.rule.attrib.attrib_mask = IPA_FLT_SRC_ADDR | IPA_FLT_DST_ADDR;
		flt_rule.rule.attrib.u.v4.src_addr_mask = ~0;
		flt_rule.rule.attrib.u.v4.src_addr = ~0;
		flt_rule.rule.attrib.u.v4.dst_addr_mask = ~0;
		flt_rule.rule.attrib.u.v4.dst_addr = ~0;

		for(i=0; i<num_rule; i++)
		{
			memcpy(&(pFilteringTable->rules[i]), &flt_rule, sizeof(struct ipa_flt_rule_add));
		}

		if (false == m_filtering.AddFilteringRule(pFilteringTable))
		{
			IPACMERR("Error adding dummy ipv4 flt rule\n");
			res = IPACM_FAILURE;
			goto fail;
		}
		else
		{
			IPACM_Iface::ipacmcfg->increaseFltRuleCount(rx_prop->rx[0].src_pipe, iptype, num_rule);
			/* copy filter rule hdls */
			for (int i = 0; i < num_rule; i++)
			{
				if (pFilteringTable->rules[i].status == 0)
				{
					IPACM_Wlan::dummy_flt_rule_hdl_v4[i] = pFilteringTable->rules[i].flt_rule_hdl;
					IPACMDBG("Dummy v4 flt rule %d hdl:0x%x\n", i, IPACM_Wlan::dummy_flt_rule_hdl_v4[i]);
				}
				else
				{
					IPACMERR("Failed adding dummy v4 flt rule %d\n", i);
					res = IPACM_FAILURE;
					goto fail;
				}
			}
		}
	}
	else if(iptype == IPA_IP_v6)
	{
		flt_rule.rule.attrib.attrib_mask = IPA_FLT_SRC_ADDR | IPA_FLT_DST_ADDR;
		flt_rule.rule.attrib.u.v6.src_addr_mask[0] = ~0;
		flt_rule.rule.attrib.u.v6.src_addr_mask[1] = ~0;
		flt_rule.rule.attrib.u.v6.src_addr_mask[2] = ~0;
		flt_rule.rule.attrib.u.v6.src_addr_mask[3] = ~0;
		flt_rule.rule.attrib.u.v6.src_addr[0] = ~0;
		flt_rule.rule.attrib.u.v6.src_addr[1] = ~0;
		flt_rule.rule.attrib.u.v6.src_addr[2] = ~0;
		flt_rule.rule.attrib.u.v6.src_addr[3] = ~0;
		flt_rule.rule.attrib.u.v6.dst_addr_mask[0] = ~0;
		flt_rule.rule.attrib.u.v6.dst_addr_mask[1] = ~0;
		flt_rule.rule.attrib.u.v6.dst_addr_mask[2] = ~0;
		flt_rule.rule.attrib.u.v6.dst_addr_mask[3] = ~0;
		flt_rule.rule.attrib.u.v6.dst_addr[0] = ~0;
		flt_rule.rule.attrib.u.v6.dst_addr[1] = ~0;
		flt_rule.rule.attrib.u.v6.dst_addr[2] = ~0;
		flt_rule.rule.attrib.u.v6.dst_addr[3] = ~0;

		for(i=0; i<num_rule; i++)
		{
			memcpy(&(pFilteringTable->rules[i]), &flt_rule, sizeof(struct ipa_flt_rule_add));
		}

		if (false == m_filtering.AddFilteringRule(pFilteringTable))
		{
			IPACMERR("Error adding dummy ipv6 flt rule\n");
			res = IPACM_FAILURE;
			goto fail;
		}
		else
		{
			IPACM_Iface::ipacmcfg->increaseFltRuleCount(rx_prop->rx[0].src_pipe, iptype, num_rule);
			/* copy filter rule hdls */
			for (int i = 0; i < num_rule; i++)
			{
				if (pFilteringTable->rules[i].status == 0)
				{
					IPACM_Wlan::dummy_flt_rule_hdl_v6[i] = pFilteringTable->rules[i].flt_rule_hdl;
					IPACMDBG("Lan2lan v6 flt rule %d hdl:0x%x\n", i, IPACM_Wlan::dummy_flt_rule_hdl_v6[i]);
				}
				else
				{
					IPACMERR("Failed adding v6 flt rule %d\n", i);
					res = IPACM_FAILURE;
					goto fail;
				}
			}
		}
	}
	else
	{
		IPACMERR("IP type is not expected.\n");
		goto fail;
	}

fail:
	free(pFilteringTable);
	return res;
}

/* delete dummy flt rule for WLAN AP-AP mode support*/
void IPACM_Wlan::del_dummy_flt_rule()
{
	int num_v4_dummy_rule, num_v6_dummy_rule;

	if(IPACM_Wlan::num_wlan_ap_iface == 0)
	{
		if(IPACM_Wlan::dummy_flt_rule_hdl_v4 == NULL || IPACM_Wlan::dummy_flt_rule_hdl_v4 == NULL)
		{
			IPACMERR("Either v4 or v6 dummy flt rule is empty.\n");
			return;
		}
#ifndef CT_OPT
		num_v4_dummy_rule = 2*(IPV4_DEFAULT_FILTERTING_RULES + MAX_OFFLOAD_PAIR + IPACM_Iface::ipacmcfg->ipa_num_private_subnet);
		num_v6_dummy_rule = 2*(IPV6_DEFAULT_FILTERTING_RULES + MAX_OFFLOAD_PAIR);
#else
		num_v4_dummy_rule = 2*(IPV4_DEFAULT_FILTERTING_RULES + NUM_TCP_CTL_FLT_RULE + MAX_OFFLOAD_PAIR + IPACM_Iface::ipacmcfg->ipa_num_private_subnet);
		num_v6_dummy_rule = 2*(IPV6_DEFAULT_FILTERTING_RULES + NUM_TCP_CTL_FLT_RULE + MAX_OFFLOAD_PAIR);
#endif
#ifdef FEATURE_IPA_ANDROID
		num_v4_dummy_rule = num_v4_dummy_rule - 2* IPACM_Iface::ipacmcfg->ipa_num_private_subnet + 2 * IPA_MAX_PRIVATE_SUBNET_ENTRIES;
#endif

#ifdef FEATURE_ETH_BRIDGE_LE
		num_v4_dummy_rule = IPV4_DEFAULT_FILTERTING_RULES + IPA_LAN_TO_LAN_MAX_WLAN_CLIENT + IPA_LAN_TO_LAN_MAX_LAN_CLIENT + IPACM_Iface::ipacmcfg->ipa_num_private_subnet;
		num_v6_dummy_rule = IPV6_DEFAULT_FILTERTING_RULES + IPA_LAN_TO_LAN_MAX_WLAN_CLIENT + IPA_LAN_TO_LAN_MAX_LAN_CLIENT;
#endif

		if(m_filtering.DeleteFilteringHdls(IPACM_Wlan::dummy_flt_rule_hdl_v4, IPA_IP_v4, num_v4_dummy_rule) == false)
		{
			IPACMERR("Failed to delete ipv4 dummy flt rules.\n");
			return;
		}
		IPACM_Iface::ipacmcfg->decreaseFltRuleCount(rx_prop->rx[0].src_pipe, IPA_IP_v4, num_v4_dummy_rule);
		if(m_filtering.DeleteFilteringHdls(IPACM_Wlan::dummy_flt_rule_hdl_v6, IPA_IP_v6, num_v6_dummy_rule) == false)
		{
			IPACMERR("Failed to delete ipv6 dummy flt rules.\n");
			return;
		}
		IPACM_Iface::ipacmcfg->decreaseFltRuleCount(rx_prop->rx[0].src_pipe, IPA_IP_v6, num_v6_dummy_rule);

		free(IPACM_Wlan::dummy_flt_rule_hdl_v4);
		IPACM_Wlan::dummy_flt_rule_hdl_v4 = NULL;
		free(IPACM_Wlan::dummy_flt_rule_hdl_v6);
		IPACM_Wlan::dummy_flt_rule_hdl_v6 = NULL;
#ifdef FEATURE_ETH_BRIDGE_LE
		memset(self_client_flt_rule_hdl_v4, 0, IPA_LAN_TO_LAN_MAX_WLAN_CLIENT * sizeof(lan2lan_flt_rule_hdl));
		memset(self_client_flt_rule_hdl_v6, 0, IPA_LAN_TO_LAN_MAX_WLAN_CLIENT * sizeof(lan2lan_flt_rule_hdl));
		memset(lan_client_flt_rule_hdl_v4, 0, IPA_LAN_TO_LAN_MAX_LAN_CLIENT * sizeof(lan2lan_flt_rule_hdl));
		memset(lan_client_flt_rule_hdl_v6, 0, IPA_LAN_TO_LAN_MAX_LAN_CLIENT * sizeof(lan2lan_flt_rule_hdl));
#endif
	}
	return;
}

/* install TCP control filter rules */
void IPACM_Wlan::install_tcp_ctl_flt_rule(ipa_ip_type iptype)
{
	if (rx_prop == NULL)
	{
		IPACMDBG_H("No rx properties registered for iface %s\n", dev_name);
		return;
	}

	int i, len, res = IPACM_SUCCESS, offset;
	struct ipa_flt_rule_mdfy flt_rule;
	struct ipa_ioc_mdfy_flt_rule* pFilteringTable;

	if (iptype == IPA_IP_v4)
	{
		if(IPACM_Wlan::dummy_flt_rule_hdl_v4 == NULL)
		{
			IPACMERR("Dummy ipv4 flt rule has not been installed.\n");
			return;
		}
		offset = wlan_ap_index * (IPV4_DEFAULT_FILTERTING_RULES + NUM_TCP_CTL_FLT_RULE + MAX_OFFLOAD_PAIR + IPACM_Iface::ipacmcfg->ipa_num_private_subnet);
#ifdef FEATURE_IPA_ANDROID
		offset = offset + wlan_ap_index * (IPA_MAX_PRIVATE_SUBNET_ENTRIES - IPACM_Iface::ipacmcfg->ipa_num_private_subnet);
#endif
	}
	else
	{
		if(IPACM_Wlan::dummy_flt_rule_hdl_v6 == NULL)
		{
			IPACMERR("Dummy ipv6 flt rule has not been installed.\n");
			return;
		}
		offset = wlan_ap_index * (IPV6_DEFAULT_FILTERTING_RULES + NUM_TCP_CTL_FLT_RULE + MAX_OFFLOAD_PAIR);
	}

	len = sizeof(struct ipa_ioc_mdfy_flt_rule) + NUM_TCP_CTL_FLT_RULE * sizeof(struct ipa_flt_rule_mdfy);
	pFilteringTable = (struct ipa_ioc_mdfy_flt_rule*)malloc(len);
	if (!pFilteringTable)
	{
		IPACMERR("Failed to allocate ipa_ioc_mdfy_flt_rule memory...\n");
		return;
	}
	memset(pFilteringTable, 0, len);

	pFilteringTable->commit = 1;
	pFilteringTable->ip = iptype;
	pFilteringTable->num_rules = NUM_TCP_CTL_FLT_RULE;

	memset(&flt_rule, 0, sizeof(struct ipa_flt_rule_mdfy));
	flt_rule.status = -1;

	flt_rule.rule.retain_hdr = 1;
	flt_rule.rule.to_uc = 0;
	flt_rule.rule.action = IPA_PASS_TO_EXCEPTION;
	flt_rule.rule.eq_attrib_type = 1;

	flt_rule.rule.eq_attrib.rule_eq_bitmap = 0;

	if(rx_prop->rx[0].attrib.attrib_mask & IPA_FLT_META_DATA)
	{
		flt_rule.rule.eq_attrib.rule_eq_bitmap |= (1<<14);
		flt_rule.rule.eq_attrib.metadata_meq32_present = 1;
		flt_rule.rule.eq_attrib.metadata_meq32.offset = 0;
		flt_rule.rule.eq_attrib.metadata_meq32.value = rx_prop->rx[0].attrib.meta_data;
		flt_rule.rule.eq_attrib.metadata_meq32.mask = rx_prop->rx[0].attrib.meta_data_mask;
	}

	flt_rule.rule.eq_attrib.rule_eq_bitmap |= (1<<1);
	flt_rule.rule.eq_attrib.protocol_eq_present = 1;
	flt_rule.rule.eq_attrib.protocol_eq = IPACM_FIREWALL_IPPROTO_TCP;

	/* add TCP FIN rule*/
	flt_rule.rule.eq_attrib.rule_eq_bitmap |= (1<<8);
	flt_rule.rule.eq_attrib.ihl_offset_meq_32[0].offset = 12;
	flt_rule.rule.eq_attrib.ihl_offset_meq_32[0].value = (((uint32_t)1)<<TCP_FIN_SHIFT);
	flt_rule.rule.eq_attrib.ihl_offset_meq_32[0].mask = (((uint32_t)1)<<TCP_FIN_SHIFT);
	flt_rule.rule.eq_attrib.num_ihl_offset_meq_32 = 1;
	if(iptype == IPA_IP_v4)
	{
		flt_rule.rule_hdl = IPACM_Wlan::dummy_flt_rule_hdl_v4[offset];
	}
	else
	{
		flt_rule.rule_hdl = IPACM_Wlan::dummy_flt_rule_hdl_v6[offset];
	}
	memcpy(&(pFilteringTable->rules[0]), &flt_rule, sizeof(struct ipa_flt_rule_mdfy));

	/* add TCP SYN rule*/
	flt_rule.rule.eq_attrib.ihl_offset_meq_32[0].value = (((uint32_t)1)<<TCP_SYN_SHIFT);
	flt_rule.rule.eq_attrib.ihl_offset_meq_32[0].mask = (((uint32_t)1)<<TCP_SYN_SHIFT);
	if(iptype == IPA_IP_v4)
	{
		flt_rule.rule_hdl = IPACM_Wlan::dummy_flt_rule_hdl_v4[offset+1];
	}
	else
	{
		flt_rule.rule_hdl = IPACM_Wlan::dummy_flt_rule_hdl_v6[offset+1];
	}
	memcpy(&(pFilteringTable->rules[1]), &flt_rule, sizeof(struct ipa_flt_rule_mdfy));

	/* add TCP RST rule*/
	flt_rule.rule.eq_attrib.ihl_offset_meq_32[0].value = (((uint32_t)1)<<TCP_RST_SHIFT);
	flt_rule.rule.eq_attrib.ihl_offset_meq_32[0].mask = (((uint32_t)1)<<TCP_RST_SHIFT);
	if(iptype == IPA_IP_v4)
	{
		flt_rule.rule_hdl = IPACM_Wlan::dummy_flt_rule_hdl_v4[offset+2];
	}
	else
	{
		flt_rule.rule_hdl = IPACM_Wlan::dummy_flt_rule_hdl_v6[offset+2];
	}
	memcpy(&(pFilteringTable->rules[2]), &flt_rule, sizeof(struct ipa_flt_rule_mdfy));

	if (false == m_filtering.ModifyFilteringRule(pFilteringTable))
	{
		IPACMERR("Failed to modify tcp control filtering rules.\n");
		goto fail;
	}
	else
	{
		if(iptype == IPA_IP_v4)
		{
			for(i=0; i<NUM_TCP_CTL_FLT_RULE; i++)
			{
				tcp_ctl_flt_rule_hdl_v4[i] = pFilteringTable->rules[i].rule_hdl;
			}
		}
		else
		{
			for(i=0; i<NUM_TCP_CTL_FLT_RULE; i++)
			{
				tcp_ctl_flt_rule_hdl_v6[i] = pFilteringTable->rules[i].rule_hdl;
			}
		}
	}

fail:
	free(pFilteringTable);
	return;
}

int IPACM_Wlan::add_dummy_private_subnet_flt_rule(ipa_ip_type iptype)
{
	if(rx_prop == NULL)
	{
		IPACMDBG_H("There is no rx_prop for iface %s, not able to add dummy lan2lan filtering rule.\n", dev_name);
		return IPACM_FAILURE;
	}

	int offset;
	if(iptype == IPA_IP_v4)
	{
		if(IPACM_Wlan::dummy_flt_rule_hdl_v4 == NULL)
		{
			IPACMERR("Dummy ipv4 flt rule has not been installed.\n");
			return IPACM_FAILURE;
		}

#ifndef CT_OPT
		offset = wlan_ap_index * (IPV4_DEFAULT_FILTERTING_RULES + MAX_OFFLOAD_PAIR + IPACM_Iface::ipacmcfg->ipa_num_private_subnet)
						+ IPV4_DEFAULT_FILTERTING_RULES + MAX_OFFLOAD_PAIR;
#else
		offset = wlan_ap_index * (IPV4_DEFAULT_FILTERTING_RULES + NUM_TCP_CTL_FLT_RULE + MAX_OFFLOAD_PAIR + IPACM_Iface::ipacmcfg->ipa_num_private_subnet)
						+ IPV4_DEFAULT_FILTERTING_RULES + MAX_OFFLOAD_PAIR + NUM_TCP_CTL_FLT_RULE;
#endif

#ifdef FEATURE_IPA_ANDROID
		offset = offset + wlan_ap_index * (IPA_MAX_PRIVATE_SUBNET_ENTRIES - IPACM_Iface::ipacmcfg->ipa_num_private_subnet);
#endif
		for (int i = 0; i < IPA_MAX_PRIVATE_SUBNET_ENTRIES; i++)
		{
			private_fl_rule_hdl[i] = IPACM_Wlan::dummy_flt_rule_hdl_v4[offset+i];
			IPACMDBG_H("Private subnet v4 flt rule %d hdl:0x%x\n", i, private_fl_rule_hdl[i]);
		}
	}
	return IPACM_SUCCESS;
}

int IPACM_Wlan::eth_bridge_handle_dummy_wlan_client_flt_rule(ipa_ip_type iptype)
{
	int i, offset;
	if(wlan_ap_index == 0)
	{
		if(iptype == IPA_IP_v4)
		{
			offset = IPV4_DEFAULT_FILTERTING_RULES;
			for(i=0; i<IPA_LAN_TO_LAN_MAX_WLAN_CLIENT; i++)
			{
				self_client_flt_rule_hdl_v4[i].rule_hdl = IPACM_Wlan::dummy_flt_rule_hdl_v4[offset+i];
				self_client_flt_rule_hdl_v4[i].valid = true;
			}
		}
		else
		{
			offset = 0;
			for(i=0; i<IPA_LAN_TO_LAN_MAX_WLAN_CLIENT; i++)
			{
				self_client_flt_rule_hdl_v6[i].rule_hdl = IPACM_Wlan::dummy_flt_rule_hdl_v6[offset+i];
				self_client_flt_rule_hdl_v6[i].valid = true;
			}
		}
		IPACMDBG_H("Get %d flt rule hdls for wlan clients ip type: %d.\n", IPA_LAN_TO_LAN_MAX_WLAN_CLIENT, iptype);
	}
	return IPACM_SUCCESS;
}

int IPACM_Wlan::eth_bridge_handle_dummy_lan_client_flt_rule(ipa_ip_type iptype)
{
	int i, offset;
	if(wlan_ap_index == 0)
	{
		if(iptype == IPA_IP_v4)
		{
			offset = IPV4_DEFAULT_FILTERTING_RULES + IPA_LAN_TO_LAN_MAX_WLAN_CLIENT;
			for(i=0; i<IPA_LAN_TO_LAN_MAX_LAN_CLIENT; i++)
			{
				lan_client_flt_rule_hdl_v4[i].rule_hdl = IPACM_Wlan::dummy_flt_rule_hdl_v4[offset+i];
				lan_client_flt_rule_hdl_v4[i].valid = true;
			}
		}
		else
		{
			offset = IPA_LAN_TO_LAN_MAX_WLAN_CLIENT;
			for(i=0; i<IPA_LAN_TO_LAN_MAX_LAN_CLIENT; i++)
			{
				lan_client_flt_rule_hdl_v6[i].rule_hdl = IPACM_Wlan::dummy_flt_rule_hdl_v6[offset+i];
				lan_client_flt_rule_hdl_v6[i].valid = true;
			}
		}
		IPACMDBG_H("Get %d flt rule hdls for lan clients ip type: %d.\n", IPA_LAN_TO_LAN_MAX_LAN_CLIENT, iptype);
	}
	return IPACM_SUCCESS;
}

int IPACM_Wlan::eth_bridge_add_lan_client_flt_rule(uint8_t* mac, ipa_ip_type iptype)
{
	int i, len, res = IPACM_SUCCESS, client_position;
	struct ipa_flt_rule_mdfy flt_rule;
	struct ipa_ioc_mdfy_flt_rule* pFilteringTable = NULL;
	bool client_is_found = false;

	if (rx_prop == NULL)
	{
		IPACMDBG_H("No rx properties registered for iface %s\n", dev_name);
		return IPACM_FAILURE;
	}
	if(mac == NULL)
	{
		IPACMERR("MAC address is empty.\n");
		return IPACM_FAILURE;
	}
	if(is_guest_ap)
	{
		IPACMDBG_H("This is guest AP WLAN interface index %d, ignore.\n", wlan_ap_index);
		return IPACM_SUCCESS;
	}
	if(wlan_ap_index != 0)
	{
		IPACMDBG_H("This is WLAN interface index %d, ignore.\n", wlan_ap_index);
		return IPACM_SUCCESS;
	}

	for(i=0; i<lan_client_flt_info_count; i++)
	{
		if(memcmp(eth_bridge_lan_client_flt_info[i].mac, mac, sizeof(eth_bridge_lan_client_flt_info[i].mac)) == 0)
		{
			client_is_found = true;
			client_position = i;
			if( (iptype == IPA_IP_v4 && eth_bridge_lan_client_flt_info[i].flt_rule_set_v4 == true)
				|| (iptype == IPA_IP_v6 && eth_bridge_lan_client_flt_info[i].flt_rule_set_v6 == true))
			{
				IPACMDBG_H("Flt rule for iptype %d has been set.\n", iptype);
				return IPACM_SUCCESS;
			}
			break;
		}
	}

	if(client_is_found == false && lan_client_flt_info_count == IPA_LAN_TO_LAN_MAX_LAN_CLIENT)
	{
		IPACMDBG_H("The lan client flt table is already full.\n");
		return IPACM_FAILURE;
	}

	len = sizeof(struct ipa_ioc_mdfy_flt_rule) + sizeof(struct ipa_flt_rule_mdfy);
	pFilteringTable = (struct ipa_ioc_mdfy_flt_rule*)malloc(len);
	if (!pFilteringTable)
	{
		IPACMERR("Failed to allocate ipa_ioc_mdfy_flt_rule memory...\n");
		return IPACM_FAILURE;
	}
	memset(pFilteringTable, 0, len);

	IPACMDBG_H("Receive LAN client MAC 0x%02x%02x%02x%02x%02x%02x.\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

	/* add mac based rule on flt table */
	pFilteringTable->commit = 1;
	pFilteringTable->ip = iptype;
	pFilteringTable->num_rules = 1;

	/* point to LAN-WLAN routing table */
	memset(&flt_rule, 0, sizeof(struct ipa_flt_rule_mdfy));
	flt_rule.status = -1;

	flt_rule.rule.retain_hdr = 0;
	flt_rule.rule.to_uc = 0;
	flt_rule.rule.action = IPA_PASS_TO_ROUTING;
	flt_rule.rule.eq_attrib_type = 0;

	if(iptype == IPA_IP_v4)
	{
		if (false == m_routing.GetRoutingTable(&IPACM_Iface::ipacmcfg->rt_tbl_eth_bridge_lan_wlan_v4))
		{
			IPACMERR("Failed to get routing table handle.\n");
			res = IPACM_FAILURE;
			goto fail;
		}
		flt_rule.rule.rt_tbl_hdl = IPACM_Iface::ipacmcfg->rt_tbl_eth_bridge_lan_wlan_v4.hdl;
		IPACMDBG_H("WLAN->LAN IPv4 filter rule use table: %s\n",IPACM_Iface::ipacmcfg->rt_tbl_eth_bridge_lan_wlan_v4.name);
	}
	else
	{
		if (false == m_routing.GetRoutingTable(&IPACM_Iface::ipacmcfg->rt_tbl_eth_bridge_lan_wlan_v6))
		{
			IPACMERR("Failed to get routing table handle.\n");
			res = IPACM_FAILURE;
			goto fail;
		}
		flt_rule.rule.rt_tbl_hdl = IPACM_Iface::ipacmcfg->rt_tbl_eth_bridge_lan_wlan_v6.hdl;
		IPACMDBG_H("WLAN->LAN IPv6 filter rule use table: %s\n",IPACM_Iface::ipacmcfg->rt_tbl_eth_bridge_lan_wlan_v6.name);
	}

	memcpy(&flt_rule.rule.attrib, &rx_prop->rx[0].attrib, sizeof(flt_rule.rule.attrib));

	/* Install meta-data if self or other ap is guest ap */
	if ((is_guest_ap == false &&  IPACM_Wlan::num_wlan_ap_iface == 1) ||
			IPACM_Iface::ipacmcfg->ipa_num_wlan_guest_ap == 0)
	{
		flt_rule.rule.attrib.attrib_mask &= ~((uint32_t)IPA_FLT_META_DATA);	//remove meta data mask
	}

	if(IPACM_Lan::wlan_hdr_type == IPA_HDR_L2_ETHERNET_II)
	{
		flt_rule.rule.attrib.attrib_mask |= IPA_FLT_MAC_DST_ADDR_ETHER_II;
	}
	else if(IPACM_Lan::wlan_hdr_type == IPA_HDR_L2_802_3)
	{
		flt_rule.rule.attrib.attrib_mask |= IPA_FLT_MAC_DST_ADDR_802_3;
	}
	else
	{
		IPACMERR("WLAN hdr type is not expected.\n");
		res = IPACM_FAILURE;
		goto fail;
	}
	memcpy(flt_rule.rule.attrib.dst_mac_addr, mac, sizeof(flt_rule.rule.attrib.dst_mac_addr));
	memset(flt_rule.rule.attrib.dst_mac_addr_mask, 0xFF, sizeof(flt_rule.rule.attrib.dst_mac_addr_mask));

	if(iptype == IPA_IP_v4)
	{
		for(i=0; i<IPA_LAN_TO_LAN_MAX_LAN_CLIENT; i++)
		{
			if(lan_client_flt_rule_hdl_v4[i].valid == true)
			{
				flt_rule.rule_hdl = lan_client_flt_rule_hdl_v4[i].rule_hdl;
				lan_client_flt_rule_hdl_v4[i].valid = false;
				break;
			}
		}
		if(i == IPA_LAN_TO_LAN_MAX_LAN_CLIENT)
		{
			IPACMDBG_H("Cannot find a valid flt rule hdl.\n");
			res = IPACM_FAILURE;
			goto fail;
		}
	}
	else
	{
		for(i=0; i<IPA_LAN_TO_LAN_MAX_LAN_CLIENT; i++)
		{
			if(lan_client_flt_rule_hdl_v6[i].valid == true)
			{
				flt_rule.rule_hdl = lan_client_flt_rule_hdl_v6[i].rule_hdl;
				lan_client_flt_rule_hdl_v6[i].valid = false;
				break;
			}
		}
		if(i == IPA_LAN_TO_LAN_MAX_LAN_CLIENT)
		{
			IPACMDBG_H("Cannot find a valid flt rule hdl.\n");
			res = IPACM_FAILURE;
			goto fail;
		}
	}

	memcpy(&(pFilteringTable->rules[0]), &flt_rule, sizeof(struct ipa_flt_rule_mdfy));
	if (false == m_filtering.ModifyFilteringRule(pFilteringTable))
	{
		IPACMERR("Failed to add wlan client filtering rules.\n");
		res = IPACM_FAILURE;
		goto fail;
	}

	if(client_is_found == false)
	{
		client_position = lan_client_flt_info_count;
		lan_client_flt_info_count++;
	}

	memcpy(eth_bridge_lan_client_flt_info[client_position].mac, mac, sizeof(eth_bridge_lan_client_flt_info[client_position].mac));
	if(iptype == IPA_IP_v4)
	{
		eth_bridge_lan_client_flt_info[client_position].flt_rule_set_v4 = true;
		eth_bridge_lan_client_flt_info[client_position].flt_rule_hdl_v4 = lan_client_flt_rule_hdl_v4[i].rule_hdl;
	}
	else
	{
		eth_bridge_lan_client_flt_info[client_position].flt_rule_set_v6 = true;
		eth_bridge_lan_client_flt_info[client_position].flt_rule_hdl_v6 = lan_client_flt_rule_hdl_v6[i].rule_hdl;
	}

fail:
	free(pFilteringTable);
	return res;
}

int IPACM_Wlan::eth_bridge_del_lan_client_flt_rule(uint8_t* mac)
{
	if(mac == NULL)
	{
		IPACMERR("Client MAC address is empty.\n");
		return IPACM_FAILURE;
	}

	IPACMDBG_H("Receive LAN client MAC 0x%02x%02x%02x%02x%02x%02x.\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

	int i, j, res = IPACM_SUCCESS;
	for(i=0; i<lan_client_flt_info_count; i++)
	{
		if(memcmp(eth_bridge_lan_client_flt_info[i].mac, mac, sizeof(eth_bridge_lan_client_flt_info[i].mac)) == 0)
		{
			break;
		}
	}

	if(i == lan_client_flt_info_count)
	{
		IPACMERR("Do not find the lan client.\n");
		return IPACM_FAILURE;
	}

	if(eth_bridge_lan_client_flt_info[i].flt_rule_set_v4 == true)
	{
		if(reset_to_dummy_flt_rule(IPA_IP_v4, eth_bridge_lan_client_flt_info[i].flt_rule_hdl_v4) == IPACM_SUCCESS)
		{
			for(j=0; j<IPA_LAN_TO_LAN_MAX_LAN_CLIENT; j++)
			{
				if(lan_client_flt_rule_hdl_v4[j].rule_hdl == eth_bridge_lan_client_flt_info[i].flt_rule_hdl_v4)
				{
					lan_client_flt_rule_hdl_v4[j].valid = true;
					break;
				}
			}
			if(j == IPA_LAN_TO_LAN_MAX_LAN_CLIENT)
			{
				IPACMERR("Not finding the rule handle in handle pool.\n");
				return IPACM_FAILURE;
			}
		}
		else
		{
			IPACMERR("Failed to delete the lan client specific flt rule.\n");
			return IPACM_FAILURE;
		}
	}
	if(eth_bridge_lan_client_flt_info[i].flt_rule_set_v6 == true)
	{
		if(reset_to_dummy_flt_rule(IPA_IP_v6, eth_bridge_lan_client_flt_info[i].flt_rule_hdl_v6) == IPACM_SUCCESS)
		{
			for(j=0; j<IPA_LAN_TO_LAN_MAX_LAN_CLIENT; j++)
			{
				if(lan_client_flt_rule_hdl_v6[j].rule_hdl == eth_bridge_lan_client_flt_info[i].flt_rule_hdl_v6)
				{
					lan_client_flt_rule_hdl_v6[j].valid = true;
					break;
				}
			}
			if(j == IPA_LAN_TO_LAN_MAX_LAN_CLIENT)
			{
				IPACMERR("Not finding the rule handle in handle pool.\n");
				return IPACM_FAILURE;
			}
		}
		else
		{
			IPACMERR("Failed to delete the lan client specific flt rule.\n");
			return IPACM_FAILURE;
		}
	}

	for(j=i+1; j<lan_client_flt_info_count; j++)
	{
		memcpy(&(eth_bridge_lan_client_flt_info[j-1]), &(eth_bridge_lan_client_flt_info[j]), sizeof(eth_bridge_client_flt_info));
	}
	memset(&(eth_bridge_lan_client_flt_info[lan_client_flt_info_count-1]), 0, sizeof(eth_bridge_client_flt_info));
	lan_client_flt_info_count--;

	return res;
}

int IPACM_Wlan::eth_bridge_add_self_client_flt_rule(uint8_t* mac, ipa_ip_type iptype)
{
	int i, len, res = IPACM_SUCCESS, client_position;
	struct ipa_flt_rule_mdfy flt_rule;
	struct ipa_ioc_mdfy_flt_rule* pFilteringTable = NULL;
	bool client_is_found = false;

	if (rx_prop == NULL)
	{
		IPACMDBG_H("No rx properties registered for iface %s\n", dev_name);
		return IPACM_FAILURE;
	}
	if(mac == NULL)
	{
		IPACMERR("MAC address is empty.\n");
		return IPACM_FAILURE;
	}
	if(IPACM_Lan::wlan_to_wlan_hdr_proc_ctx.valid == false)
	{
		IPACMDBG_H("WLAN to WLAN hdr proc ctx has not been set, don't add WLAN client specific flt rule.\n");
		return IPACM_FAILURE;
	}

	for(i=0; i<wlan_client_flt_info_count; i++)
	{
		if(memcmp(eth_bridge_wlan_client_flt_info[i].mac, mac, sizeof(eth_bridge_wlan_client_flt_info[i].mac)) == 0)
		{
			client_is_found = true;
			client_position = i;
			if( (iptype == IPA_IP_v4 && eth_bridge_wlan_client_flt_info[i].flt_rule_set_v4 == true)
				|| (iptype == IPA_IP_v6 && eth_bridge_wlan_client_flt_info[i].flt_rule_set_v6 == true))
			{
				IPACMDBG_H("Flt rule for iptype %d has been set.\n", iptype);
				return IPACM_SUCCESS;
			}
			break;
		}
	}

	if(client_is_found == false && wlan_client_flt_info_count == IPA_LAN_TO_LAN_MAX_WLAN_CLIENT)
	{
		IPACMDBG_H("The wlan client flt table is already full.\n");
		return IPACM_FAILURE;
	}

	len = sizeof(struct ipa_ioc_mdfy_flt_rule) + sizeof(struct ipa_flt_rule_mdfy);
	pFilteringTable = (struct ipa_ioc_mdfy_flt_rule*)malloc(len);
	if (!pFilteringTable)
	{
		IPACMERR("Failed to allocate ipa_ioc_mdfy_flt_rule memory...\n");
		return IPACM_FAILURE;
	}
	memset(pFilteringTable, 0, len);

	IPACMDBG_H("Receive WLAN client MAC 0x%02x%02x%02x%02x%02x%02x.\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

	/* add mac based rule on IPv4 table */
	pFilteringTable->commit = 1;
	pFilteringTable->ip = iptype;
	pFilteringTable->num_rules = 1;

	/* point to LAN-WLAN routing table */
	memset(&flt_rule, 0, sizeof(struct ipa_flt_rule_mdfy));
	flt_rule.status = -1;

	flt_rule.rule.retain_hdr = 0;
	flt_rule.rule.to_uc = 0;
	flt_rule.rule.action = IPA_PASS_TO_ROUTING;
	flt_rule.rule.eq_attrib_type = 0;

	if(iptype == IPA_IP_v4)
	{
		if (false == m_routing.GetRoutingTable(&IPACM_Iface::ipacmcfg->rt_tbl_eth_bridge_wlan_wlan_v4))
		{
			IPACMERR("Failed to get routing table handle.\n");
			res = IPACM_FAILURE;
			goto fail;
		}
		flt_rule.rule.rt_tbl_hdl = IPACM_Iface::ipacmcfg->rt_tbl_eth_bridge_wlan_wlan_v4.hdl;
		IPACMDBG_H("WLAN->WLAN IPv4 filter rule use table: %s\n",IPACM_Iface::ipacmcfg->rt_tbl_eth_bridge_wlan_wlan_v4.name);
	}
	else
	{
		if (false == m_routing.GetRoutingTable(&IPACM_Iface::ipacmcfg->rt_tbl_eth_bridge_wlan_wlan_v6))
		{
			IPACMERR("Failed to get routing table handle.\n");
			res = IPACM_FAILURE;
			goto fail;
		}
		flt_rule.rule.rt_tbl_hdl = IPACM_Iface::ipacmcfg->rt_tbl_eth_bridge_wlan_wlan_v6.hdl;
		IPACMDBG_H("WLAN->WLAN IPv4 filter rule use table: %s\n",IPACM_Iface::ipacmcfg->rt_tbl_eth_bridge_wlan_wlan_v6.name);
	}

	memcpy(&flt_rule.rule.attrib, &rx_prop->rx[0].attrib, sizeof(flt_rule.rule.attrib));

	/* Install meta-data if self or other ap is guest ap */
	if ((is_guest_ap == false &&  IPACM_Wlan::num_wlan_ap_iface == 1) ||
		IPACM_Iface::ipacmcfg->ipa_num_wlan_guest_ap == 0)
	{
		flt_rule.rule.attrib.attrib_mask &= ~((uint32_t)IPA_FLT_META_DATA);	//remove meta data mask
	}

	if(IPACM_Lan::wlan_hdr_type == IPA_HDR_L2_ETHERNET_II)
	{
		flt_rule.rule.attrib.attrib_mask |= IPA_FLT_MAC_DST_ADDR_ETHER_II;
	}
	else if(IPACM_Lan::wlan_hdr_type == IPA_HDR_L2_802_3)
	{
		flt_rule.rule.attrib.attrib_mask |= IPA_FLT_MAC_DST_ADDR_802_3;
	}
	else
	{
		IPACMERR("WLAN hdr type is not expected.\n");
		res = IPACM_FAILURE;
		goto fail;
	}
	memcpy(flt_rule.rule.attrib.dst_mac_addr, mac, sizeof(flt_rule.rule.attrib.dst_mac_addr));
	memset(flt_rule.rule.attrib.dst_mac_addr_mask, 0xFF, sizeof(flt_rule.rule.attrib.dst_mac_addr_mask));

	if(iptype == IPA_IP_v4)
	{
		for(i=0; i<IPA_LAN_TO_LAN_MAX_WLAN_CLIENT; i++)
		{
			if(self_client_flt_rule_hdl_v4[i].valid == true)
			{
				flt_rule.rule_hdl = self_client_flt_rule_hdl_v4[i].rule_hdl;
				self_client_flt_rule_hdl_v4[i].valid = false;
				break;
			}
		}
		if(i == IPA_LAN_TO_LAN_MAX_WLAN_CLIENT)
		{
			IPACMDBG_H("Cannot find a valid flt rule hdl.\n");
			res = IPACM_FAILURE;
			goto fail;
		}
	}
	else
	{
		for(i=0; i<IPA_LAN_TO_LAN_MAX_WLAN_CLIENT; i++)
		{
			if(self_client_flt_rule_hdl_v6[i].valid == true)
			{
				flt_rule.rule_hdl = self_client_flt_rule_hdl_v6[i].rule_hdl;
				self_client_flt_rule_hdl_v6[i].valid = false;
				break;
			}
		}
		if(i == IPA_LAN_TO_LAN_MAX_WLAN_CLIENT)
		{
			IPACMDBG_H("Cannot find a valid flt rule hdl.\n");
			res = IPACM_FAILURE;
			goto fail;
		}
	}

	memcpy(&(pFilteringTable->rules[0]), &flt_rule, sizeof(struct ipa_flt_rule_mdfy));
	if (false == m_filtering.ModifyFilteringRule(pFilteringTable))
	{
		IPACMERR("Failed to add self client filtering rules.\n");
		res = IPACM_FAILURE;
		goto fail;
	}

	if(client_is_found == false)
	{
		client_position = wlan_client_flt_info_count;
		wlan_client_flt_info_count++;
	}

	memcpy(eth_bridge_wlan_client_flt_info[client_position].mac, mac, sizeof(eth_bridge_wlan_client_flt_info[client_position].mac));
	if(iptype == IPA_IP_v4)
	{
		eth_bridge_wlan_client_flt_info[client_position].flt_rule_set_v4 = true;
		eth_bridge_wlan_client_flt_info[client_position].flt_rule_hdl_v4 = self_client_flt_rule_hdl_v4[i].rule_hdl;
	}
	else
	{
		eth_bridge_wlan_client_flt_info[client_position].flt_rule_set_v6 = true;
		eth_bridge_wlan_client_flt_info[client_position].flt_rule_hdl_v6 = self_client_flt_rule_hdl_v6[i].rule_hdl;
	}

fail:
	free(pFilteringTable);
	return res;
}

int IPACM_Wlan::eth_bridge_del_self_client_flt_rule(uint8_t* mac)
{
	if(mac == NULL)
	{
		IPACMERR("Client MAC address is empty.\n");
		return IPACM_FAILURE;
	}

	IPACMDBG_H("Receive WLAN client MAC 0x%02x%02x%02x%02x%02x%02x.\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

	int i, j, res = IPACM_SUCCESS;
	for(i=0; i<wlan_client_flt_info_count; i++)
	{
		if(memcmp(eth_bridge_wlan_client_flt_info[i].mac, mac, sizeof(eth_bridge_wlan_client_flt_info[i].mac)) == 0)
		{
			break;
		}
	}

	if(i == wlan_client_flt_info_count)
	{
		IPACMERR("Do not find the wlan client.\n");
		return IPACM_FAILURE;
	}

	if(eth_bridge_wlan_client_flt_info[i].flt_rule_set_v4 == true)
	{
		if(reset_to_dummy_flt_rule(IPA_IP_v4, eth_bridge_wlan_client_flt_info[i].flt_rule_hdl_v4) == IPACM_SUCCESS)
		{
			for(j=0; j<IPA_LAN_TO_LAN_MAX_WLAN_CLIENT; j++)
			{
				if(self_client_flt_rule_hdl_v4[j].rule_hdl == eth_bridge_wlan_client_flt_info[i].flt_rule_hdl_v4)
				{
					self_client_flt_rule_hdl_v4[j].valid = true;
					break;
				}
			}
			if(j == IPA_LAN_TO_LAN_MAX_WLAN_CLIENT)
			{
				IPACMERR("Not finding the rule handle in handle pool.\n");
				return IPACM_FAILURE;
			}
		}
		else
		{
			IPACMERR("Failed to delete the wlan client specific flt rule.\n");
			return IPACM_FAILURE;
		}
	}
	if(eth_bridge_wlan_client_flt_info[i].flt_rule_set_v6 == true)
	{
		if(reset_to_dummy_flt_rule(IPA_IP_v6, eth_bridge_wlan_client_flt_info[i].flt_rule_hdl_v6) == IPACM_SUCCESS)
		{
			for(j=0; j<IPA_LAN_TO_LAN_MAX_WLAN_CLIENT; j++)
			{
				if(self_client_flt_rule_hdl_v6[j].rule_hdl == eth_bridge_wlan_client_flt_info[i].flt_rule_hdl_v6)
				{
					self_client_flt_rule_hdl_v6[j].valid = true;
					break;
				}
			}
			if(j == IPA_LAN_TO_LAN_MAX_WLAN_CLIENT)
			{
				IPACMERR("Not finding the rule handle in handle pool.\n");
				return IPACM_FAILURE;
			}
		}
		else
		{
			IPACMERR("Failed to delete the wlan client specific flt rule.\n");
			return IPACM_FAILURE;
		}
	}

	for(j=i+1; j<wlan_client_flt_info_count; j++)
	{
		memcpy(&(eth_bridge_wlan_client_flt_info[j-1]), &(eth_bridge_wlan_client_flt_info[j]), sizeof(eth_bridge_client_flt_info));
	}
	memset(&(eth_bridge_wlan_client_flt_info[wlan_client_flt_info_count-1]), 0, sizeof(eth_bridge_client_flt_info));
	wlan_client_flt_info_count--;

	return res;
}

int IPACM_Wlan::eth_bridge_install_cache_lan_client_flt_rule(ipa_ip_type iptype)
{
	int i;

	IPACMDBG_H("There are %d lan clients cached.\n", IPACM_Lan::num_lan_client);
	for(i=0; i<IPACM_Lan::num_lan_client; i++)
	{
		eth_bridge_add_lan_client_flt_rule(IPACM_Lan::eth_bridge_lan_client[i].mac, iptype);
	}
	return IPACM_SUCCESS;
}

int IPACM_Wlan::eth_bridge_install_cache_wlan_client_flt_rule(ipa_ip_type iptype)
{
	int i;

	IPACMDBG_H("There are %d wlan clients cached.\n", IPACM_Lan::num_wlan_client);
	for(i=0; i<IPACM_Lan::num_wlan_client; i++)
	{
		eth_bridge_add_self_client_flt_rule(IPACM_Lan::eth_bridge_wlan_client[i].mac, iptype);
	}
	return IPACM_SUCCESS;
}

int IPACM_Wlan::eth_bridge_add_wlan_client_rt_rule(uint8_t* mac, eth_bridge_src_iface src, ipa_ip_type iptype)
{
	if(tx_prop == NULL)
	{
		IPACMDBG_H("Tx prop is empty, not adding routing rule.\n");
		return IPACM_SUCCESS;
	}
	if(mac == NULL)
	{
		IPACMERR("Client MAC address is empty.\n");
		return IPACM_FAILURE;
	}

	IPACMDBG_H("Receive WLAN client MAC 0x%02x%02x%02x%02x%02x%02x src_iface: %d .\n",
			mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], src);

	if(iptype == IPA_IP_v4)
	{
		if( (src == SRC_WLAN && wlan_client_rt_from_wlan_info_count_v4 == IPA_LAN_TO_LAN_MAX_WLAN_CLIENT)
			|| (src == SRC_LAN && wlan_client_rt_from_lan_info_count_v4 == IPA_LAN_TO_LAN_MAX_WLAN_CLIENT))
		{
			IPACMDBG_H("WLAN client number has reached maximum.\n");
			return IPACM_FAILURE;
		}
	}
	else
	{
		if( (src == SRC_WLAN && wlan_client_rt_from_wlan_info_count_v6 == IPA_LAN_TO_LAN_MAX_WLAN_CLIENT)
			|| (src == SRC_LAN && wlan_client_rt_from_lan_info_count_v6 == IPA_LAN_TO_LAN_MAX_WLAN_CLIENT))
		{
			IPACMDBG_H("WLAN client number has reached maximum.\n");
			return IPACM_FAILURE;
		}
	}
	if( (src == SRC_WLAN && IPACM_Lan::wlan_to_wlan_hdr_proc_ctx.valid == false)
		|| (src == SRC_LAN && IPACM_Lan::lan_to_wlan_hdr_proc_ctx.valid == false) )
	{
		IPACMDBG_H("Hdr proc ctx has not been set for source %d, don't add WLAN client routing rule.\n", src);
		return IPACM_FAILURE;
	}

	int i, len, res = IPACM_SUCCESS;
	struct ipa_ioc_add_rt_rule* rt_rule_table = NULL;
	struct ipa_rt_rule_add rt_rule;
	int position, num_rt_rule;

	if(src == SRC_WLAN)
	{
		if(iptype == IPA_IP_v4)
		{
			for(i=0; i<wlan_client_rt_from_wlan_info_count_v4; i++)
			{
				if(memcmp(eth_bridge_get_client_rt_info_ptr(i, SRC_WLAN, iptype)->mac, mac, sizeof(eth_bridge_get_client_rt_info_ptr(i, SRC_WLAN, iptype)->mac)) == 0)
				{
					IPACMDBG_H("The client's routing rule was added before.\n");
					return IPACM_SUCCESS;
				}
			}
			memcpy(eth_bridge_get_client_rt_info_ptr(wlan_client_rt_from_wlan_info_count_v4, src, iptype)->mac, mac,
					sizeof(eth_bridge_get_client_rt_info_ptr(wlan_client_rt_from_wlan_info_count_v4, src, iptype)->mac));
		}
		else
		{
			for(i=0; i<wlan_client_rt_from_wlan_info_count_v6; i++)
			{
				if(memcmp(eth_bridge_get_client_rt_info_ptr(i, SRC_WLAN, iptype)->mac, mac, sizeof(eth_bridge_get_client_rt_info_ptr(i, SRC_WLAN, iptype)->mac)) == 0)
				{
					IPACMDBG_H("The client's routing rule was added before.\n");
					return IPACM_SUCCESS;
				}
			}
			memcpy(eth_bridge_get_client_rt_info_ptr(wlan_client_rt_from_wlan_info_count_v6, src, iptype)->mac, mac,
					sizeof(eth_bridge_get_client_rt_info_ptr(wlan_client_rt_from_wlan_info_count_v6, src, iptype)->mac));
		}
	}
	else
	{
		if(iptype == IPA_IP_v4)
		{
			for(i=0; i<wlan_client_rt_from_lan_info_count_v4; i++)
			{
				if(memcmp(eth_bridge_get_client_rt_info_ptr(i, SRC_LAN, iptype)->mac, mac, sizeof(eth_bridge_get_client_rt_info_ptr(i, SRC_LAN, iptype)->mac)) == 0)
				{
					IPACMDBG_H("The client's routing rule was added before.\n");
					return IPACM_SUCCESS;
				}
			}
			memcpy(eth_bridge_get_client_rt_info_ptr(wlan_client_rt_from_lan_info_count_v4, src, iptype)->mac, mac,
					sizeof(eth_bridge_get_client_rt_info_ptr(wlan_client_rt_from_lan_info_count_v4, src, iptype)->mac));
		}
		else
		{
			for(i=0; i<wlan_client_rt_from_lan_info_count_v6; i++)
			{
				if(memcmp(eth_bridge_get_client_rt_info_ptr(i, SRC_LAN, iptype)->mac, mac, sizeof(eth_bridge_get_client_rt_info_ptr(i, SRC_LAN, iptype)->mac)) == 0)
				{
					IPACMDBG_H("The client's routing rule was added before.\n");
					return IPACM_SUCCESS;
				}
			}
			memcpy(eth_bridge_get_client_rt_info_ptr(wlan_client_rt_from_lan_info_count_v6, src, iptype)->mac, mac,
					sizeof(eth_bridge_get_client_rt_info_ptr(wlan_client_rt_from_lan_info_count_v6, src, iptype)->mac));
		}
	}

	if(iptype == IPA_IP_v4)
	{
		num_rt_rule = each_client_rt_rule_count_v4;
	}
	else
	{
		num_rt_rule = each_client_rt_rule_count_v6;
	}

	len = sizeof(ipa_ioc_add_rt_rule) + num_rt_rule * sizeof(ipa_rt_rule_add);
	rt_rule_table = (ipa_ioc_add_rt_rule*)malloc(len);
	if(rt_rule_table == NULL)
	{
		IPACMERR("Failed to allocate memory.\n");
		return IPACM_FAILURE;
	}
	memset(rt_rule_table, 0, len);

	rt_rule_table->commit = 1;
	rt_rule_table->ip = iptype;
	rt_rule_table->num_rules = num_rt_rule;
	if(src == SRC_WLAN)
	{
		if(iptype == IPA_IP_v4)
		{
			strlcpy(rt_rule_table->rt_tbl_name, IPACM_Iface::ipacmcfg->rt_tbl_eth_bridge_wlan_wlan_v4.name, sizeof(rt_rule_table->rt_tbl_name));
		}
		else
		{
			strlcpy(rt_rule_table->rt_tbl_name, IPACM_Iface::ipacmcfg->rt_tbl_eth_bridge_wlan_wlan_v6.name, sizeof(rt_rule_table->rt_tbl_name));
		}
	}
	else
	{
		if(iptype == IPA_IP_v4)
		{
			strlcpy(rt_rule_table->rt_tbl_name, IPACM_Iface::ipacmcfg->rt_tbl_eth_bridge_lan_wlan_v4.name, sizeof(rt_rule_table->rt_tbl_name));
		}
		else
		{
			strlcpy(rt_rule_table->rt_tbl_name, IPACM_Iface::ipacmcfg->rt_tbl_eth_bridge_lan_wlan_v6.name, sizeof(rt_rule_table->rt_tbl_name));
		}
	}
	rt_rule_table->rt_tbl_name[IPA_RESOURCE_NAME_MAX-1] = '\0';
	memset(&rt_rule, 0, sizeof(ipa_rt_rule_add));
	rt_rule.at_rear = false;
	rt_rule.status = -1;
	rt_rule.rt_rule_hdl = -1;

	rt_rule.rule.hdr_hdl = 0;
	if(src == SRC_WLAN)
	{
		rt_rule.rule.hdr_proc_ctx_hdl = IPACM_Lan::wlan_to_wlan_hdr_proc_ctx.proc_ctx_hdl;
	}
	else
	{
		rt_rule.rule.hdr_proc_ctx_hdl = IPACM_Lan::lan_to_wlan_hdr_proc_ctx.proc_ctx_hdl;
	}
	position = 0;
	for(i=0; i<iface_query->num_tx_props; i++)
	{
		if(tx_prop->tx[i].ip == iptype)
		{
			if(position >= num_rt_rule)
			{
				IPACMERR("Number of routing rules already exceeds limit.\n");
				res = IPACM_FAILURE;
				goto fail;
			}
			/* Handle MCC Mode case */
			if (IPACM_Iface::ipacmcfg->isMCC_Mode == true)
			{
				IPACMDBG_H("In MCC mode, use alt dst pipe: %d\n",
						tx_prop->tx[i].alt_dst_pipe);
				rt_rule.rule.dst = tx_prop->tx[i].alt_dst_pipe;
			}
			else
			{
				rt_rule.rule.dst = tx_prop->tx[i].dst_pipe;
			}

			memcpy(&rt_rule.rule.attrib, &tx_prop->tx[i].attrib, sizeof(rt_rule.rule.attrib));
			if(src == SRC_WLAN)	//src is WLAN means packet is from WLAN
			{
				if(IPACM_Lan::wlan_hdr_type == IPA_HDR_L2_ETHERNET_II)
				{
					rt_rule.rule.attrib.attrib_mask |= IPA_FLT_MAC_DST_ADDR_ETHER_II;
				}
				else
				{
					rt_rule.rule.attrib.attrib_mask |= IPA_FLT_MAC_DST_ADDR_802_3;
				}
			}
			else	//packet is from LAN
			{
				if(IPACM_Lan::lan_hdr_type == IPA_HDR_L2_ETHERNET_II)
				{
					rt_rule.rule.attrib.attrib_mask |= IPA_FLT_MAC_DST_ADDR_ETHER_II;
				}
				else
				{
					rt_rule.rule.attrib.attrib_mask |= IPA_FLT_MAC_DST_ADDR_802_3;
				}
			}
			memcpy(rt_rule.rule.attrib.dst_mac_addr, mac, sizeof(rt_rule.rule.attrib.dst_mac_addr));
			memset(rt_rule.rule.attrib.dst_mac_addr_mask, 0xFF, sizeof(rt_rule.rule.attrib.dst_mac_addr_mask));

			memcpy(&(rt_rule_table->rules[position]), &rt_rule, sizeof(rt_rule_table->rules[position]));
			position++;
		}
	}
	if(false == m_routing.AddRoutingRule(rt_rule_table))
	{
		IPACMERR("Routing rule addition failed!\n");
		res = IPACM_FAILURE;
		goto fail;
	}
	else
	{
		if(src == SRC_WLAN)
		{
			for(i=0; i<num_rt_rule; i++)
			{
				if(iptype == IPA_IP_v4)
				{
					eth_bridge_get_client_rt_info_ptr(wlan_client_rt_from_wlan_info_count_v4, src, iptype)->rt_rule_hdl[i] = rt_rule_table->rules[i].rt_rule_hdl;
				}
				else
				{
					eth_bridge_get_client_rt_info_ptr(wlan_client_rt_from_wlan_info_count_v6, src, iptype)->rt_rule_hdl[i] = rt_rule_table->rules[i].rt_rule_hdl;
				}
			}
			if(iptype == IPA_IP_v4)
			{
				wlan_client_rt_from_wlan_info_count_v4++;
				IPACMDBG_H("Now the number of IPv4 rt rule on wlan-wlan rt table is %d.\n", wlan_client_rt_from_wlan_info_count_v4);
			}
			else
			{
				wlan_client_rt_from_wlan_info_count_v6++;
				IPACMDBG_H("Now the number of IPv6 rt rule on wlan-wlan rt table is %d.\n", wlan_client_rt_from_wlan_info_count_v6);
			}
		}
		else
		{
			for(i=0; i<num_rt_rule; i++)
			{
				if(iptype == IPA_IP_v4)
				{
					eth_bridge_get_client_rt_info_ptr(wlan_client_rt_from_lan_info_count_v4, src, iptype)->rt_rule_hdl[i] = rt_rule_table->rules[i].rt_rule_hdl;
				}
				else
				{
					eth_bridge_get_client_rt_info_ptr(wlan_client_rt_from_lan_info_count_v6, src, iptype)->rt_rule_hdl[i] = rt_rule_table->rules[i].rt_rule_hdl;
				}
			}
			if(iptype == IPA_IP_v4)
			{
				wlan_client_rt_from_lan_info_count_v4++;
				IPACMDBG_H("Now the number of IPv4 rt rule on lan-wlan rt table is %d.\n", wlan_client_rt_from_lan_info_count_v4);
			}
			else
			{
				wlan_client_rt_from_lan_info_count_v6++;
				IPACMDBG_H("Now the number of IPv6 rt rule on lan-wlan rt table is %d.\n", wlan_client_rt_from_lan_info_count_v6);
			}
		}
	}

fail:
	if(rt_rule_table != NULL)
	{
		free(rt_rule_table);
	}
	return res;
}

int IPACM_Wlan::eth_bridge_del_wlan_client_rt_rule(uint8_t* mac, eth_bridge_src_iface src)
{
	if(tx_prop == NULL)
	{
		IPACMDBG_H("Tx prop is empty, not deleting routing rule.\n");
		return IPACM_SUCCESS;
	}
	if(mac == NULL)
	{
		IPACMERR("Client MAC address is empty.\n");
		return IPACM_FAILURE;
	}

	IPACMDBG_H("Receive WLAN client MAC 0x%02x%02x%02x%02x%02x%02x.\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

	int i, position;

	/* first delete the rt rules from IPv4 rt table*/
	if(src == SRC_WLAN)
	{
		for(i=0; i<wlan_client_rt_from_wlan_info_count_v4; i++)
		{
			if(memcmp(eth_bridge_get_client_rt_info_ptr(i, src, IPA_IP_v4)->mac, mac, sizeof(eth_bridge_get_client_rt_info_ptr(i, src, IPA_IP_v4)->mac)) == 0)
			{
				position = i;
				IPACMDBG_H("The client is found at position %d.\n", position);
				break;
			}
		}
		if(i == wlan_client_rt_from_wlan_info_count_v4)
		{
			IPACMERR("The client is not found.\n");
			return IPACM_FAILURE;
		}
	}
	else
	{
		for(i=0; i<wlan_client_rt_from_lan_info_count_v4; i++)
		{
			if(memcmp(eth_bridge_get_client_rt_info_ptr(i, src, IPA_IP_v4)->mac, mac, sizeof(eth_bridge_get_client_rt_info_ptr(i, src, IPA_IP_v4)->mac)) == 0)
			{
				position = i;
				IPACMDBG_H("The client is found at position %d.\n", position);
				break;
			}
		}
		if(i == wlan_client_rt_from_lan_info_count_v4)
		{
			IPACMERR("The client is not found.\n");
			return IPACM_FAILURE;
		}
	}

	for(i=0; i<each_client_rt_rule_count_v4; i++)
	{
		if(m_routing.DeleteRoutingHdl(eth_bridge_get_client_rt_info_ptr(position, src, IPA_IP_v4)->rt_rule_hdl[i], IPA_IP_v4) == false)
		{
			IPACMERR("Failed to delete routing rule %d.\n", i);
			return IPACM_FAILURE;
		}
	}

	if(src == SRC_WLAN)
	{
		for(i=position+1; i<wlan_client_rt_from_wlan_info_count_v4; i++)
		{
			memcpy(eth_bridge_get_client_rt_info_ptr(i-1, src, IPA_IP_v4), eth_bridge_get_client_rt_info_ptr(i, src, IPA_IP_v4), client_rt_info_size_v4);
		}
		memset(eth_bridge_get_client_rt_info_ptr(wlan_client_rt_from_wlan_info_count_v4-1, src, IPA_IP_v4), 0, client_rt_info_size_v4);
		wlan_client_rt_from_wlan_info_count_v4--;
		IPACMDBG_H("Now the number of IPv4 rt rule from wlan info is %d.\n", wlan_client_rt_from_wlan_info_count_v4);
	}
	else
	{
		for(i=position+1; i<wlan_client_rt_from_lan_info_count_v4; i++)
		{
			memcpy(eth_bridge_get_client_rt_info_ptr(i-1, src, IPA_IP_v4), eth_bridge_get_client_rt_info_ptr(i, src, IPA_IP_v4), client_rt_info_size_v4);
		}
		memset(eth_bridge_get_client_rt_info_ptr(wlan_client_rt_from_lan_info_count_v4-1, src, IPA_IP_v4), 0, client_rt_info_size_v4);
		wlan_client_rt_from_lan_info_count_v4--;
		IPACMDBG_H("Now the number of IPv4 rt rule from lan info is %d.\n", wlan_client_rt_from_lan_info_count_v4);
	}

	/*delete rt rules from IPv6 rt table */
	if(src == SRC_WLAN)
	{
		for(i=0; i<wlan_client_rt_from_wlan_info_count_v6; i++)
		{
			if(memcmp(eth_bridge_get_client_rt_info_ptr(i, src, IPA_IP_v6)->mac, mac, sizeof(eth_bridge_get_client_rt_info_ptr(i, src, IPA_IP_v6)->mac)) == 0)
			{
				position = i;
				IPACMDBG_H("The client is found at position %d.\n", position);
				break;
			}
		}
		if(i == wlan_client_rt_from_wlan_info_count_v6)
		{
			IPACMERR("The client is not found.\n");
			return IPACM_FAILURE;
		}
	}
	else
	{
		for(i=0; i<wlan_client_rt_from_lan_info_count_v6; i++)
		{
			if(memcmp(eth_bridge_get_client_rt_info_ptr(i, src, IPA_IP_v6)->mac, mac, sizeof(eth_bridge_get_client_rt_info_ptr(i, src, IPA_IP_v6)->mac)) == 0)
			{
				position = i;
				IPACMDBG_H("The client is found at position %d.\n", position);
				break;
			}
		}
		if(i == wlan_client_rt_from_lan_info_count_v6)
		{
			IPACMERR("The client is not found.\n");
			return IPACM_FAILURE;
		}
	}

	for(i=0; i<each_client_rt_rule_count_v6; i++)
	{
		if(m_routing.DeleteRoutingHdl(eth_bridge_get_client_rt_info_ptr(position, src, IPA_IP_v6)->rt_rule_hdl[i], IPA_IP_v6) == false)
		{
			IPACMERR("Failed to delete routing rule %d.\n", i);
			return IPACM_FAILURE;
		}
	}

	if(src == SRC_WLAN)
	{
		for(i=position+1; i<wlan_client_rt_from_wlan_info_count_v6; i++)
		{
			memcpy(eth_bridge_get_client_rt_info_ptr(i-1, src, IPA_IP_v6), eth_bridge_get_client_rt_info_ptr(i, src, IPA_IP_v6), client_rt_info_size_v6);
		}
		memset(eth_bridge_get_client_rt_info_ptr(wlan_client_rt_from_wlan_info_count_v6-1, src, IPA_IP_v6), 0, client_rt_info_size_v6);
		wlan_client_rt_from_wlan_info_count_v6--;
		IPACMDBG_H("Now the number of IPv6 rt rule from wlan info is %d.\n", wlan_client_rt_from_wlan_info_count_v6);
	}
	else
	{
		for(i=position+1; i<wlan_client_rt_from_lan_info_count_v6; i++)
		{
			memcpy(eth_bridge_get_client_rt_info_ptr(i-1, src, IPA_IP_v6), eth_bridge_get_client_rt_info_ptr(i, src, IPA_IP_v6), client_rt_info_size_v6);
		}
		memset(eth_bridge_get_client_rt_info_ptr(wlan_client_rt_from_lan_info_count_v6-1, src, IPA_IP_v6), 0, client_rt_info_size_v6);
		wlan_client_rt_from_lan_info_count_v6--;
		IPACMDBG_H("Now the number of IPv6 rt rule from lan info is %d.\n", wlan_client_rt_from_lan_info_count_v6);
	}

	return IPACM_SUCCESS;
}

eth_bridge_client_rt_info* IPACM_Wlan::eth_bridge_get_client_rt_info_ptr(uint8_t index, eth_bridge_src_iface src, ipa_ip_type iptype)
{
	void* result;
	if(src == SRC_WLAN)
	{
		if(iptype == IPA_IP_v4)
		{
			result = (void*)((void*)eth_bridge_wlan_client_rt_from_wlan_info_v4 + index * client_rt_info_size_v4);
		}
		else
		{
			result = (void*)((void*)eth_bridge_wlan_client_rt_from_wlan_info_v6 + index * client_rt_info_size_v6);
		}
	}
	else
	{
		if(iptype == IPA_IP_v4)
		{
			result = (void*)((void*)eth_bridge_wlan_client_rt_from_lan_info_v4 + index * client_rt_info_size_v4);
		}
		else
		{
			result = (void*)((void*)eth_bridge_wlan_client_rt_from_lan_info_v6 + index * client_rt_info_size_v6);
		}
	}
	return (eth_bridge_client_rt_info*)result;
}

void IPACM_Wlan::eth_bridge_add_wlan_client(uint8_t* mac, int if_num)
{
	if(IPACM_Lan::num_wlan_client == IPA_LAN_TO_LAN_MAX_WLAN_CLIENT)
	{
		IPACMDBG_H("WLAN client table is already full.\n");
		return;
	}

	if(mac == NULL)
	{
		IPACMERR("Mac address is empty.\n");
		return;
	}

	int i;
	for(i=0; i<IPACM_Lan::num_wlan_client; i++)
	{
		if(memcmp(IPACM_Lan::eth_bridge_wlan_client[i].mac, mac, sizeof(IPACM_Lan::eth_bridge_wlan_client[i].mac)) == 0)
		{
			IPACMDBG_H("The wlan client mac has been added before at position %d.\n", i);
			return;
		}
	}

	memcpy(IPACM_Lan::eth_bridge_wlan_client[IPACM_Lan::num_wlan_client].mac, mac, sizeof(IPACM_Lan::eth_bridge_wlan_client[IPACM_Lan::num_wlan_client].mac));
	IPACM_Lan::eth_bridge_wlan_client[IPACM_Lan::num_wlan_client].ipa_if_num = if_num;
	IPACM_Lan::num_wlan_client++;
	IPACMDBG_H("Now the total num of wlan clients is %d", IPACM_Lan::num_wlan_client);
	return;
}

void IPACM_Wlan::eth_bridge_del_wlan_client(uint8_t* mac)
{
	if(mac == NULL)
	{
		IPACMERR("Mac address is empty.\n");
		return;
	}

	int i, j;
	for(i=0; i<IPACM_Lan::num_wlan_client; i++)
	{
		if(memcmp(IPACM_Lan::eth_bridge_wlan_client[i].mac, mac, sizeof(IPACM_Lan::eth_bridge_wlan_client[i].mac)) == 0)
		{
			IPACMDBG_H("Found WLAN client at position %d.\n", i);
			break;
		}
	}

	if(i == IPACM_Lan::num_wlan_client)
	{
		IPACMDBG_H("Not finding the WLAN client.\n");
		return;
	}

	for(j=i+1; j<IPACM_Lan::num_wlan_client; j++)
	{
		memcpy(IPACM_Lan::eth_bridge_wlan_client[j-1].mac, IPACM_Lan::eth_bridge_wlan_client[j].mac, sizeof(IPACM_Lan::eth_bridge_wlan_client[j].mac));
		IPACM_Lan::eth_bridge_wlan_client[j-1].ipa_if_num = IPACM_Lan::eth_bridge_wlan_client[j].ipa_if_num;
	}
	IPACM_Lan::num_wlan_client--;
	IPACMDBG_H("Now the total num of wlan clients is %d", IPACM_Lan::num_wlan_client);
	return;
}

void IPACM_Wlan::handle_SCC_MCC_switch(ipa_ip_type iptype)
{
	struct ipa_ioc_mdfy_rt_rule *rt_rule = NULL;
	struct ipa_rt_rule_mdfy *rt_rule_entry;
	uint32_t tx_index;
	int wlan_index, v6_num;
	const int NUM = 1;
	int num_wifi_client_tmp = IPACM_Wlan::num_wifi_client;
	bool isAdded = false;

	if (tx_prop == NULL)
	{
		IPACMDBG_H("No rx properties registered for iface %s\n", dev_name);
		return;
	}

	if (rt_rule == NULL)
	{
		rt_rule = (struct ipa_ioc_mdfy_rt_rule *)
			calloc(1, sizeof(struct ipa_ioc_mdfy_rt_rule) +
					NUM * sizeof(struct ipa_rt_rule_mdfy));

		if (rt_rule == NULL)
		{
			PERROR("Error Locate ipa_ioc_mdfy_rt_rule memory...\n");
			return;
		}

		rt_rule->commit = 0;
		rt_rule->num_rules = NUM;
		rt_rule->ip = iptype;
	}
	rt_rule_entry = &rt_rule->rules[0];

	/* modify ipv4 routing rule */
	if (iptype == IPA_IP_v4)
	{
		for (wlan_index = 0; wlan_index < num_wifi_client_tmp; wlan_index++)
		{
			IPACMDBG_H("wlan client index: %d, ip-type: %d, ipv4_set:%d, ipv4_rule_set:%d \n",
					wlan_index, iptype,
					get_client_memptr(wlan_client, wlan_index)->ipv4_set,
					get_client_memptr(wlan_client, wlan_index)->route_rule_set_v4);

			if (get_client_memptr(wlan_client, wlan_index)->power_save_set == true ||
					get_client_memptr(wlan_client, wlan_index)->route_rule_set_v4 == false)
			{
				IPACMDBG_H("client %d route rules not set\n", wlan_index);
				continue;
			}

			IPACMDBG_H("Modify client %d route rule\n", wlan_index);
			for (tx_index = 0; tx_index < iface_query->num_tx_props; tx_index++)
			{
				if (iptype != tx_prop->tx[tx_index].ip)
				{
					IPACMDBG_H("Tx:%d, ip-type: %d ip-type not matching: %d ignore\n",
							tx_index, tx_prop->tx[tx_index].ip, iptype);
					continue;
				}

				IPACMDBG_H("client index(%d):ipv4 address: 0x%x\n", wlan_index,
						get_client_memptr(wlan_client, wlan_index)->v4_addr);

				IPACMDBG_H("client(%d): v4 header handle:(0x%x)\n",
						wlan_index,
						get_client_memptr(wlan_client, wlan_index)->hdr_hdl_v4);

				if (IPACM_Iface::ipacmcfg->isMCC_Mode)
				{
					IPACMDBG_H("In MCC mode, use alt dst pipe: %d\n",
							tx_prop->tx[tx_index].alt_dst_pipe);
					rt_rule_entry->rule.dst = tx_prop->tx[tx_index].alt_dst_pipe;
				}
				else
				{
					rt_rule_entry->rule.dst = tx_prop->tx[tx_index].dst_pipe;
				}

				memcpy(&rt_rule_entry->rule.attrib,
						&tx_prop->tx[tx_index].attrib,
						sizeof(rt_rule_entry->rule.attrib));

				rt_rule_entry->rule.attrib.attrib_mask |= IPA_FLT_DST_ADDR;
				rt_rule_entry->rule.hdr_hdl = get_client_memptr(wlan_client, wlan_index)->hdr_hdl_v4;

				rt_rule_entry->rule.attrib.u.v4.dst_addr = get_client_memptr(wlan_client, wlan_index)->v4_addr;
				rt_rule_entry->rule.attrib.u.v4.dst_addr_mask = 0xFFFFFFFF;

				IPACMDBG_H("tx:%d, rt rule hdl=%x ip-type: %d\n", tx_index,
						get_client_memptr(wlan_client, wlan_index)->wifi_rt_hdl[tx_index].wifi_rt_rule_hdl_v4, iptype);

				rt_rule_entry->rt_rule_hdl =
					get_client_memptr(wlan_client, wlan_index)->wifi_rt_hdl[tx_index].wifi_rt_rule_hdl_v4;

				if (false == m_routing.ModifyRoutingRule(rt_rule))
				{
					IPACMERR("Routing rule modify failed!\n");
					free(rt_rule);
					return;
				}
				isAdded = true;
			}

		}
	}

	/* modify ipv6 routing rule */
	if (iptype == IPA_IP_v6)
	{
		for (wlan_index = 0; wlan_index < num_wifi_client_tmp; wlan_index++)
		{

			IPACMDBG_H("wlan client index: %d, ip-type: %d, ipv6_set:%d, ipv6_rule_num:%d \n", wlan_index, iptype,
					get_client_memptr(wlan_client, wlan_index)->ipv6_set,
					get_client_memptr(wlan_client, wlan_index)->route_rule_set_v6);

			if (get_client_memptr(wlan_client, wlan_index)->power_save_set == true ||
					(get_client_memptr(wlan_client, wlan_index)->route_rule_set_v6 <
					 get_client_memptr(wlan_client, wlan_index)->ipv6_set) )
			{
				IPACMDBG_H("client %d route rules not set\n", wlan_index);
				continue;
			}

			IPACMDBG_H("Modify client %d route rule\n", wlan_index);
			for (tx_index = 0; tx_index < iface_query->num_tx_props; tx_index++)
			{
				if (iptype != tx_prop->tx[tx_index].ip)
				{
					IPACMDBG_H("Tx:%d, ip-type: %d ip-type not matching: %d Ignore\n",
							tx_index, tx_prop->tx[tx_index].ip, iptype);
					continue;
				}

				for (v6_num = get_client_memptr(wlan_client, wlan_index)->route_rule_set_v6;
						v6_num < get_client_memptr(wlan_client, wlan_index)->ipv6_set;
						v6_num++)
				{

					IPACMDBG_H("client(%d): v6 header handle:(0x%x)\n",
							wlan_index,
							get_client_memptr(wlan_client, wlan_index)->hdr_hdl_v6);

					if (IPACM_Iface::ipacmcfg->isMCC_Mode)
					{
						IPACMDBG_H("In MCC mode, use alt dst pipe: %d\n",
								tx_prop->tx[tx_index].alt_dst_pipe);
						rt_rule_entry->rule.dst = tx_prop->tx[tx_index].alt_dst_pipe;
					}
					else
					{
						rt_rule_entry->rule.dst = tx_prop->tx[tx_index].dst_pipe;
					}

					memcpy(&rt_rule_entry->rule.attrib,
							&tx_prop->tx[tx_index].attrib,
							sizeof(rt_rule_entry->rule.attrib));

					rt_rule_entry->rule.hdr_hdl = get_client_memptr(wlan_client, wlan_index)->hdr_hdl_v6;
					rt_rule_entry->rule.attrib.attrib_mask |= IPA_FLT_DST_ADDR;

					rt_rule_entry->rule.attrib.u.v6.dst_addr[0] = get_client_memptr(wlan_client, wlan_index)->v6_addr[v6_num][0];
					rt_rule_entry->rule.attrib.u.v6.dst_addr[1] = get_client_memptr(wlan_client, wlan_index)->v6_addr[v6_num][1];
					rt_rule_entry->rule.attrib.u.v6.dst_addr[2] = get_client_memptr(wlan_client, wlan_index)->v6_addr[v6_num][2];
					rt_rule_entry->rule.attrib.u.v6.dst_addr[3] = get_client_memptr(wlan_client, wlan_index)->v6_addr[v6_num][3];
					rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[0] = 0xFFFFFFFF;
					rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[1] = 0xFFFFFFFF;
					rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[2] = 0xFFFFFFFF;
					rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[3] = 0xFFFFFFFF;

					rt_rule_entry->rt_rule_hdl =
						get_client_memptr(wlan_client, wlan_index)->wifi_rt_hdl[tx_index].wifi_rt_rule_hdl_v6_wan[v6_num];

					if (false == m_routing.ModifyRoutingRule(rt_rule))
					{
						IPACMERR("Routing rule modify failed!\n");
						free(rt_rule);
						return;
					}
					isAdded = true;
				}
			}

		}
	}


	if (isAdded)
	{
		if (false == m_routing.Commit(iptype))
		{
			IPACMERR("Routing rule modify commit failed!\n");
			free(rt_rule);
			return;
		}

		IPACMDBG("Routing rule modified successfully \n");
	}

	if(rt_rule)
	{
		free(rt_rule);
	}
	return;
}

void IPACM_Wlan::eth_bridge_handle_wlan_SCC_MCC_switch(ipa_ip_type iptype)
{

	for (int i= 0; i < IPACM_Lan::num_wlan_client; i++)
	{
		if (IPACM_Lan::eth_bridge_wlan_client[i].ipa_if_num == ipa_if_num)
		{
			if (IPACM_Lan::wlan_to_wlan_hdr_proc_ctx.valid == true)
			{
				if (eth_bridge_modify_wlan_rt_rule(IPACM_Lan::eth_bridge_wlan_client[i].mac, SRC_WLAN, iptype) == IPACM_FAILURE)
				{
					IPACMDBG_H("SCC/MCC switch is failed for iptype: %d src_iface: %d \n", iptype, SRC_WLAN);
					return;
				}
			}
			if (IPACM_Lan::lan_to_wlan_hdr_proc_ctx.valid == true)
			{
				if (eth_bridge_modify_wlan_rt_rule(IPACM_Lan::eth_bridge_wlan_client[i].mac, SRC_LAN, iptype) == IPACM_FAILURE)
				{
					IPACMDBG_H("SCC/MCC switch is failed for iptype: %d src_iface: %d \n", iptype, SRC_LAN);
					return;
				}
			}
		}
	}

	IPACMDBG_H("SCC/MCC switch is successful for iptype: %d\n", iptype);
}

int IPACM_Wlan::eth_bridge_modify_wlan_rt_rule(uint8_t* mac, eth_bridge_src_iface src_iface, ipa_ip_type iptype)
{
	struct ipa_ioc_mdfy_rt_rule *rt_rule = NULL;
	struct ipa_rt_rule_mdfy *rt_rule_entry;
	uint32_t index = 0, num_rt_rule = 0, position;

	if (tx_prop == NULL)
	{
		IPACMDBG_H("No tx properties \n");
		return IPACM_FAILURE;
	}

	if (mac == NULL)
	{
		IPACMERR("Client MAC address is empty.\n");
		return IPACM_FAILURE;
	}

	IPACMDBG_H("Receive WLAN client MAC 0x%02x%02x%02x%02x%02x%02x. src_iface: %d\n",
			mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], src_iface);

	if (iptype == IPA_IP_v4)
	{
		num_rt_rule = each_client_rt_rule_count_v4;
	}
	else
	{
		num_rt_rule = each_client_rt_rule_count_v6;
	}

	if (src_iface == SRC_WLAN)
	{
		if (iptype == IPA_IP_v4)
		{
			for (index = 0; index < wlan_client_rt_from_wlan_info_count_v4; index++)
			{
				if (memcmp(eth_bridge_get_client_rt_info_ptr(index, src_iface, IPA_IP_v4)->mac, mac,
					sizeof(eth_bridge_get_client_rt_info_ptr(index, src_iface, IPA_IP_v4)->mac)) == 0)
				{
					position = index;
					IPACMDBG_H("The client is found at position %d.\n", position);
					break;
				}
			}
			if (index == wlan_client_rt_from_wlan_info_count_v4)
			{
				IPACMERR("The client is not found.\n");
				return IPACM_FAILURE;
			}
		}
		else
		{
			for (index =0 ; index < wlan_client_rt_from_wlan_info_count_v6; index++)
			{
				if (memcmp(eth_bridge_get_client_rt_info_ptr(index, src_iface, IPA_IP_v6)->mac, mac,
					sizeof(eth_bridge_get_client_rt_info_ptr(index, src_iface, IPA_IP_v6)->mac)) == 0)
				{
					position = index;
					IPACMDBG_H("The client is found at position %d.\n", position);
					break;
				}
			}
			if (index == wlan_client_rt_from_wlan_info_count_v6)
			{
				IPACMERR("The client is not found.\n");
				return IPACM_FAILURE;
			}
		}
	}
	else
	{
		if (iptype == IPA_IP_v4)
		{
			for (index = 0; index < wlan_client_rt_from_lan_info_count_v4; index++)
			{
				if (memcmp(eth_bridge_get_client_rt_info_ptr(index, src_iface, IPA_IP_v4)->mac, mac,
					sizeof(eth_bridge_get_client_rt_info_ptr(index, src_iface, IPA_IP_v4)->mac)) == 0)
				{
					position = index;
					IPACMDBG_H("The client is found at position %d.\n", position);
					break;
				}
			}
			if (index == wlan_client_rt_from_lan_info_count_v4)
			{
				IPACMERR("The client is not found.\n");
				return IPACM_FAILURE;
			}
		}
		else
		{
			for (index = 0; index < wlan_client_rt_from_lan_info_count_v6; index++)
			{
				if (memcmp(eth_bridge_get_client_rt_info_ptr(index, src_iface, IPA_IP_v6)->mac, mac,
					sizeof(eth_bridge_get_client_rt_info_ptr(index, src_iface, IPA_IP_v6)->mac)) == 0)
				{
					position = index;
					IPACMDBG_H("The client is found at position %d.\n", position);
					break;
				}
			}
			if (index == wlan_client_rt_from_lan_info_count_v6)
			{
				IPACMERR("The client is not found.\n");
				return IPACM_FAILURE;
			}
		}
	}

	rt_rule = (struct ipa_ioc_mdfy_rt_rule *)
			calloc(1, sizeof(struct ipa_ioc_mdfy_rt_rule) +
			(num_rt_rule) * sizeof(struct ipa_rt_rule_mdfy));

	if (rt_rule == NULL)
	{
		IPACMERR("Unable to allocate memory for modify rt rule\n");
		return IPACM_FAILURE;
	}
	IPACMDBG("Allocated memory for %d rules successfully\n", num_rt_rule);

	rt_rule->commit = 1;
	rt_rule->num_rules = 0;
	rt_rule->ip = iptype;

	for (index = 0; index < tx_prop->num_tx_props; index++)
	{
		if (tx_prop->tx[index].ip == iptype)
		{
			if (rt_rule->num_rules >= num_rt_rule)
			{
				IPACMERR("Number of routing rules exceeds limit.\n");
				free(rt_rule);
				return IPACM_FAILURE;
			}

			rt_rule_entry = &rt_rule->rules[rt_rule->num_rules];

			if (IPACM_Iface::ipacmcfg->isMCC_Mode)
			{
				IPACMDBG_H("In MCC mode, use alt dst pipe: %d\n",
						tx_prop->tx[index].alt_dst_pipe);
				rt_rule_entry->rule.dst = tx_prop->tx[index].alt_dst_pipe;
			}
			else
			{
				rt_rule_entry->rule.dst = tx_prop->tx[index].dst_pipe;
			}

			rt_rule_entry->rule.hdr_hdl = 0;

			if (src_iface == SRC_WLAN)
			{
				rt_rule_entry->rule.hdr_proc_ctx_hdl =
						IPACM_Lan::wlan_to_wlan_hdr_proc_ctx.proc_ctx_hdl;
			}
			else
			{
				rt_rule_entry->rule.hdr_proc_ctx_hdl =
						IPACM_Lan::lan_to_wlan_hdr_proc_ctx.proc_ctx_hdl;
			}

			memcpy(&rt_rule_entry->rule.attrib,
					&tx_prop->tx[index].attrib,
					sizeof(rt_rule_entry->rule.attrib));

			if (src_iface == SRC_WLAN)	//src is WLAN means packet is from WLAN
			{
				if (IPACM_Lan::wlan_hdr_type == IPA_HDR_L2_ETHERNET_II)
				{
					rt_rule_entry->rule.attrib.attrib_mask |= IPA_FLT_MAC_DST_ADDR_ETHER_II;
				}
				else
				{
					rt_rule_entry->rule.attrib.attrib_mask |= IPA_FLT_MAC_DST_ADDR_802_3;
				}
			}
			else	//packet is from LAN
			{
				if (IPACM_Lan::lan_hdr_type == IPA_HDR_L2_ETHERNET_II)
				{
					rt_rule_entry->rule.attrib.attrib_mask |= IPA_FLT_MAC_DST_ADDR_ETHER_II;
				}
				else
				{
					rt_rule_entry->rule.attrib.attrib_mask |= IPA_FLT_MAC_DST_ADDR_802_3;
				}
			}
			memcpy(rt_rule_entry->rule.attrib.dst_mac_addr, mac,
					sizeof(rt_rule_entry->rule.attrib.dst_mac_addr));
			memset(rt_rule_entry->rule.attrib.dst_mac_addr_mask, 0xFF,
					sizeof(rt_rule_entry->rule.attrib.dst_mac_addr_mask));

			rt_rule_entry->rt_rule_hdl =
					eth_bridge_get_client_rt_info_ptr(position, src_iface, iptype)->rt_rule_hdl[rt_rule->num_rules];
			IPACMDBG_H("tx:%d, rt rule hdl=%x ip-type: %d\n", index,
				eth_bridge_get_client_rt_info_ptr(position, src_iface, iptype)->rt_rule_hdl[rt_rule->num_rules], iptype);

			rt_rule->num_rules++;
		}
	}

	if (rt_rule->num_rules > 0)
	{
		if (false == m_routing.ModifyRoutingRule(rt_rule))
		{
			IPACMERR("Routing rule modify failed!\n");
			free(rt_rule);
			return IPACM_FAILURE;
		}
		if (false == m_routing.Commit(iptype))
		{
			IPACMERR("Routing rule modify commit failed!\n");
			free(rt_rule);
			return IPACM_FAILURE;
		}
		IPACMDBG("Routing rule modified successfully \n");
	}

	if (rt_rule)
	{
		free(rt_rule);
	}
	return IPACM_SUCCESS;
}

void IPACM_Wlan::eth_bridge_handle_wlan_mode_switch()
{
	int i;

	for (i=0; i<IPACM_Lan::num_wlan_client; i++)
	{
		if (IPACM_Lan::eth_bridge_wlan_client[i].ipa_if_num == ipa_if_num)
		{
			eth_bridge_modify_wlan_client_flt_rule(IPACM_Lan::eth_bridge_wlan_client[i].mac, DST_WLAN, IPA_IP_v4);
			eth_bridge_modify_wlan_client_flt_rule(IPACM_Lan::eth_bridge_wlan_client[i].mac, DST_WLAN, IPA_IP_v6);

			if(IPACM_Lan::lan_to_wlan_hdr_proc_ctx.valid == true)
			{
				if (is_guest_ap == true)
				{
					eth_bridge_del_wlan_client_rt_rule(IPACM_Lan::eth_bridge_wlan_client[i].mac, SRC_LAN);
					eth_bridge_post_lan_client_event(IPACM_Lan::eth_bridge_wlan_client[i].mac, IPA_ETH_BRIDGE_WLAN_CLIENT_DEL_EVENT);
				}
				else
				{
					eth_bridge_add_wlan_client_rt_rule(IPACM_Lan::eth_bridge_wlan_client[i].mac, SRC_LAN, IPA_IP_v4);
					eth_bridge_add_wlan_client_rt_rule(IPACM_Lan::eth_bridge_wlan_client[i].mac, SRC_LAN, IPA_IP_v6);
					eth_bridge_post_lan_client_event(IPACM_Lan::eth_bridge_wlan_client[i].mac, IPA_ETH_BRIDGE_WLAN_CLIENT_ADD_EVENT);
				}
			}
		}
	}
}


int IPACM_Wlan::eth_bridge_modify_wlan_client_flt_rule(uint8_t* mac, eth_bridge_dst_iface dst_iface, ipa_ip_type iptype)
{

	int index, len, res = IPACM_SUCCESS, client_position;
	struct ipa_flt_rule_mdfy flt_rule;
	struct ipa_ioc_mdfy_flt_rule* pFilteringTable = NULL;

	if (rx_prop == NULL)
	{
		IPACMDBG_H("No rx properties registered for iface %s\n", dev_name);
		return IPACM_FAILURE;
	}
	if (mac == NULL)
	{
		IPACMERR("MAC address is empty.\n");
		return IPACM_FAILURE;
	}
	IPACMDBG_H("Received client MAC 0x%02x%02x%02x%02x%02x%02x. dst_iface: %d \n",
			mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], dst_iface);

	if (dst_iface == DST_WLAN && IPACM_Lan::wlan_to_wlan_hdr_proc_ctx.valid == false)
	{
		IPACMDBG_H("WLAN to WLAN hdr proc ctx has not been set, don't modify client specific flt rule.\n");
		return IPACM_FAILURE;
	}
	if (dst_iface == DST_LAN && IPACM_Lan::lan_to_wlan_hdr_proc_ctx.valid == false)
	{
		IPACMDBG_H("WLAN to LAN hdr proc ctx has not been set, don't modify client specific flt rule.\n");
		return IPACM_FAILURE;
	}

	if (dst_iface == DST_WLAN)
	{
		for (index=0; index<wlan_client_flt_info_count; index++)
		{
			if(memcmp(eth_bridge_wlan_client_flt_info[index].mac, mac, sizeof(eth_bridge_wlan_client_flt_info[index].mac)) == 0)
			{
				client_position = index;
				IPACMDBG_H("The client is found at position %d.\n", client_position);
				break;
			}
		}
		if(index == wlan_client_flt_info_count)
		{
			IPACMDBG_H("The wlan client is not found.\n");
			return IPACM_FAILURE;
		}
	}
	else
	{
		for(index=0; index<lan_client_flt_info_count; index++)
		{
			if(memcmp(eth_bridge_lan_client_flt_info[index].mac, mac, sizeof(eth_bridge_lan_client_flt_info[index].mac)) == 0)
			{
				client_position = index;
				IPACMDBG_H("The client is found at position %d.\n", client_position);
				break;
			}
		}
		if(index == lan_client_flt_info_count)
		{
			IPACMDBG_H("The lan client is not found.\n");
			return IPACM_FAILURE;
		}
	}

	len = sizeof(struct ipa_ioc_mdfy_flt_rule) + sizeof(struct ipa_flt_rule_mdfy);
	pFilteringTable = (struct ipa_ioc_mdfy_flt_rule*)malloc(len);
	if (!pFilteringTable)
	{
		IPACMERR("Failed to allocate ipa_ioc_mdfy_flt_rule memory...\n");
		return IPACM_FAILURE;
	}
	memset(pFilteringTable, 0, len);

	/* add mac based rule on IPv4 table */
	pFilteringTable->commit = 1;
	pFilteringTable->ip = iptype;
	pFilteringTable->num_rules = 1;

	/* point to WLAN-WLAN routing table */
	memset(&flt_rule, 0, sizeof(struct ipa_flt_rule_mdfy));
	flt_rule.status = -1;
	flt_rule.rule.retain_hdr = 0;
	flt_rule.rule.to_uc = 0;
	flt_rule.rule.action = IPA_PASS_TO_ROUTING;
	flt_rule.rule.eq_attrib_type = 0;

	if (dst_iface == DST_WLAN)
	{
		if(iptype == IPA_IP_v4)
		{
			if (false == m_routing.GetRoutingTable(&IPACM_Iface::ipacmcfg->rt_tbl_eth_bridge_wlan_wlan_v4))
			{
				IPACMERR("Failed to get routing table handle.\n");
				res = IPACM_FAILURE;
				goto fail;
			}
			flt_rule.rule.rt_tbl_hdl = IPACM_Iface::ipacmcfg->rt_tbl_eth_bridge_wlan_wlan_v4.hdl;
			IPACMDBG_H("WLAN->WLAN IPv4 filter rule use table: %s\n",IPACM_Iface::ipacmcfg->rt_tbl_eth_bridge_wlan_wlan_v4.name);
		}
		else
		{
			if (false == m_routing.GetRoutingTable(&IPACM_Iface::ipacmcfg->rt_tbl_eth_bridge_wlan_wlan_v6))
			{
				IPACMERR("Failed to get routing table handle.\n");
				res = IPACM_FAILURE;
				goto fail;
			}
			flt_rule.rule.rt_tbl_hdl = IPACM_Iface::ipacmcfg->rt_tbl_eth_bridge_wlan_wlan_v6.hdl;
			IPACMDBG_H("WLAN->WLAN IPv6 filter rule use table: %s\n",IPACM_Iface::ipacmcfg->rt_tbl_eth_bridge_wlan_wlan_v6.name);
		}
	}
	else
	{
		if(iptype == IPA_IP_v4)
		{
			if (false == m_routing.GetRoutingTable(&IPACM_Iface::ipacmcfg->rt_tbl_eth_bridge_lan_wlan_v4))
			{
				IPACMERR("Failed to get routing table handle.\n");
				res = IPACM_FAILURE;
				goto fail;
			}
			flt_rule.rule.rt_tbl_hdl = IPACM_Iface::ipacmcfg->rt_tbl_eth_bridge_lan_wlan_v4.hdl;
			IPACMDBG_H("WLAN->LAN IPv4 filter rule use table: %s\n",IPACM_Iface::ipacmcfg->rt_tbl_eth_bridge_lan_wlan_v4.name);
		}
		else
		{
			if (false == m_routing.GetRoutingTable(&IPACM_Iface::ipacmcfg->rt_tbl_eth_bridge_lan_wlan_v6))
			{
				IPACMERR("Failed to get routing table handle.\n");
				res = IPACM_FAILURE;
			}
			flt_rule.rule.rt_tbl_hdl = IPACM_Iface::ipacmcfg->rt_tbl_eth_bridge_lan_wlan_v6.hdl;
			IPACMDBG_H("WLAN->LAN IPv6 filter rule use table: %s\n",IPACM_Iface::ipacmcfg->rt_tbl_eth_bridge_lan_wlan_v6.name);
		}
	}

	memcpy(&flt_rule.rule.attrib, &rx_prop->rx[0].attrib, sizeof(flt_rule.rule.attrib));

	/* Install meta-data if self or other ap is guest ap */
	if ((is_guest_ap == false &&  IPACM_Wlan::num_wlan_ap_iface == 1) ||
			IPACM_Iface::ipacmcfg->ipa_num_wlan_guest_ap == 0)
	{
		flt_rule.rule.attrib.attrib_mask &= ~((uint32_t)IPA_FLT_META_DATA); //remove meta data mask
	}

	if(IPACM_Lan::wlan_hdr_type == IPA_HDR_L2_ETHERNET_II)
	{
		flt_rule.rule.attrib.attrib_mask |= IPA_FLT_MAC_DST_ADDR_ETHER_II;
	}
	else if(IPACM_Lan::wlan_hdr_type == IPA_HDR_L2_802_3)
	{
		flt_rule.rule.attrib.attrib_mask |= IPA_FLT_MAC_DST_ADDR_802_3;
	}
	else
	{
		IPACMERR("WLAN hdr type is not expected.\n");
		res = IPACM_FAILURE;
		goto fail;
	}
	memcpy(flt_rule.rule.attrib.dst_mac_addr, mac, sizeof(flt_rule.rule.attrib.dst_mac_addr));
	memset(flt_rule.rule.attrib.dst_mac_addr_mask, 0xFF, sizeof(flt_rule.rule.attrib.dst_mac_addr_mask));

	if (dst_iface == DST_WLAN)
	{
		if(iptype == IPA_IP_v4)
		{
			flt_rule.rule_hdl = eth_bridge_wlan_client_flt_info[client_position].flt_rule_hdl_v4;
		}
		else
		{
			flt_rule.rule_hdl = eth_bridge_wlan_client_flt_info[client_position].flt_rule_hdl_v6;
		}
	}
	else
	{
		if(iptype == IPA_IP_v4)
		{
			flt_rule.rule_hdl = eth_bridge_lan_client_flt_info[client_position].flt_rule_hdl_v4;
		}
		else
		{
			flt_rule.rule_hdl = eth_bridge_lan_client_flt_info[client_position].flt_rule_hdl_v6;
		}
	}
	memcpy(&(pFilteringTable->rules[0]), &flt_rule, sizeof(struct ipa_flt_rule_mdfy));
	if (false == m_filtering.ModifyFilteringRule(pFilteringTable))
	{
		IPACMERR("Failed to modify wlan client filtering rule.\n");
		res = IPACM_FAILURE;
		goto fail;
	}
fail:
	free(pFilteringTable);
	return res;
}
