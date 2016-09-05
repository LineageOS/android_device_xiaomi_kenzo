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
	IPACM_IfaceManager.cpp

	@brief
	This file implements the IPAM iface_manager functionality.

	@Author
	Skylar Chang

*/
#include <string.h>
#include <sys/ioctl.h>

#include <IPACM_IfaceManager.h>
#include <IPACM_EvtDispatcher.h>
#include <IPACM_Defs.h>
#include <IPACM_Wlan.h>
#include <IPACM_Lan.h>
#include <IPACM_Wan.h>
#include <IPACM_Iface.h>
#include <IPACM_Log.h>

iface_instances *IPACM_IfaceManager::head = NULL;

IPACM_IfaceManager::IPACM_IfaceManager()
{
	IPACM_EvtDispatcher::registr(IPA_CFG_CHANGE_EVENT, this); 		// register for IPA_CFG_CHANGE event
	IPACM_EvtDispatcher::registr(IPA_LINK_UP_EVENT, this);
	IPACM_EvtDispatcher::registr(IPA_WLAN_AP_LINK_UP_EVENT, this);  // register for wlan AP-iface
#ifndef FEATURE_IPA_ANDROID
	IPACM_EvtDispatcher::registr(IPA_WLAN_STA_LINK_UP_EVENT, this); // register for wlan STA-iface
	/* only MDM targets support device on bridge mode */
	IPACM_EvtDispatcher::registr(IPA_BRIDGE_LINK_UP_EVENT, this); 	// register for IPA_BRIDGE_LINK_UP_EVENT event
#endif /* not defined(FEATURE_IPA_ANDROID)*/
	IPACM_EvtDispatcher::registr(IPA_USB_LINK_UP_EVENT, this); // register for USB-iface
	IPACM_EvtDispatcher::registr(IPA_WAN_EMBMS_LINK_UP_EVENT, this);  // register for wan eMBMS-iface
	return;
}

void IPACM_IfaceManager::event_callback(ipa_cm_event_id event, void *param)
{
	int ipa_interface_index;
	ipacm_event_data_fid *evt_data = (ipacm_event_data_fid *)param;
	ipacm_event_data_mac *StaData = (ipacm_event_data_mac *)param;
	ipacm_event_data_all *data_all = (ipacm_event_data_all *)param;
	ipacm_ifacemgr_data ifmgr_data = {0};

	switch(event)
	{
		case IPA_CFG_CHANGE_EVENT:
				IPACMDBG_H(" RESET IPACM_cfg \n");
				IPACM_Iface::ipacmcfg->Init();
			break;
		case IPA_BRIDGE_LINK_UP_EVENT:
			IPACMDBG_H(" Save the bridge0 mac info in IPACM_cfg \n");
			ipa_interface_index = IPACM_Iface::iface_ipa_index_query(data_all->if_index);
			/* check if iface is bridge interface*/
			if (strcmp(IPACM_Iface::ipacmcfg->ipa_virtual_iface_name, IPACM_Iface::ipacmcfg->iface_table[ipa_interface_index].iface_name) == 0)
			{
				IPACM_Iface::ipacmcfg->ipa_bridge_enable = true;
				memcpy(IPACM_Iface::ipacmcfg->bridge_mac,
								data_all->mac_addr,
								sizeof(IPACM_Iface::ipacmcfg->bridge_mac));
				IPACMDBG_H("cached bridge0 MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
						 IPACM_Iface::ipacmcfg->bridge_mac[0], IPACM_Iface::ipacmcfg->bridge_mac[1], IPACM_Iface::ipacmcfg->bridge_mac[2],
						 IPACM_Iface::ipacmcfg->bridge_mac[3], IPACM_Iface::ipacmcfg->bridge_mac[4], IPACM_Iface::ipacmcfg->bridge_mac[5]);
			}
			break;
		case IPA_LINK_UP_EVENT:
			IPACMDBG_H("Recieved IPA_LINK_UP_EVENT event: link up %d: \n", evt_data->if_index);
			ipa_interface_index = IPACM_Iface::iface_ipa_index_query(evt_data->if_index);
			/* LTE-backhaul */
			if(IPACM_Iface::ipacmcfg->iface_table[ipa_interface_index].if_cat == EMBMS_IF)
			{
				IPACMDBG("WAN-EMBMS (%s) link already up, iface: %d: \n", IPACM_Iface::ipacmcfg->iface_table[ipa_interface_index].iface_name,evt_data->if_index);
			}
			else if(IPACM_Iface::ipacmcfg->iface_table[ipa_interface_index].if_cat == WAN_IF)
			{
				IPACMDBG_H("WAN-LTE (%s) link up, iface: %d: \n", IPACM_Iface::ipacmcfg->iface_table[ipa_interface_index].iface_name,evt_data->if_index);
				ifmgr_data.if_index = evt_data->if_index;
				ifmgr_data.if_type = Q6_WAN;
				create_iface_instance(&ifmgr_data);
			}
			break;

		case IPA_USB_LINK_UP_EVENT:
			IPACMDBG_H("Recieved IPA_USB_LINK_UP_EVENT event: link up %d: \n", evt_data->if_index);
			ipa_interface_index = IPACM_Iface::iface_ipa_index_query(evt_data->if_index);
			/* check if it's WAN_IF */
			if(IPACM_Iface::ipacmcfg->iface_table[ipa_interface_index].if_cat == WAN_IF)
			{
				/* usb-backhaul using sta_mode ECM_WAN*/
				IPACMDBG_H("WAN-usb (%s) link up, iface: %d: \n", IPACM_Iface::ipacmcfg->iface_table[ipa_interface_index].iface_name, evt_data->if_index);
				ifmgr_data.if_index = evt_data->if_index;
				ifmgr_data.if_type = ECM_WAN;
				create_iface_instance(&ifmgr_data);
			}
			else
			{
				ifmgr_data.if_index = evt_data->if_index;
				ifmgr_data.if_type = Q6_WAN;
				create_iface_instance(&ifmgr_data);
			}
			break;

		case IPA_WLAN_AP_LINK_UP_EVENT:
			ipa_interface_index = IPACM_Iface::iface_ipa_index_query(evt_data->if_index);
			/* change iface category from unknown to WLAN_IF */
			if(IPACM_Iface::ipacmcfg->iface_table[ipa_interface_index].if_cat == UNKNOWN_IF)
			{
				IPACM_Iface::ipacmcfg->iface_table[ipa_interface_index].if_cat = WLAN_IF;
				IPACMDBG_H("WLAN AP (%s) link up, iface: %d: \n", IPACM_Iface::ipacmcfg->iface_table[ipa_interface_index].iface_name,evt_data->if_index);
				ifmgr_data.if_index = evt_data->if_index;
				ifmgr_data.if_type = Q6_WAN;
				create_iface_instance(&ifmgr_data);
			}
			else
			{
				IPACMDBG_H("iface %s already up and act as %d mode: \n",IPACM_Iface::ipacmcfg->iface_table[ipa_interface_index].iface_name,IPACM_Iface::ipacmcfg->iface_table[ipa_interface_index].if_cat);
			}
			break;

		case IPA_WLAN_STA_LINK_UP_EVENT:
			ipa_interface_index = IPACM_Iface::iface_ipa_index_query(StaData->if_index);
			/* change iface category from unknown to WAN_IF */
			if(IPACM_Iface::ipacmcfg->iface_table[ipa_interface_index].if_cat == UNKNOWN_IF)
			{
				/* wlan-backhaul using sta_mode WLAN_WAN */
				IPACM_Iface::ipacmcfg->iface_table[ipa_interface_index].if_cat = WAN_IF;
				IPACMDBG_H("WLAN STA (%s) link up, iface: %d: \n",
				IPACM_Iface::ipacmcfg->iface_table[ipa_interface_index].iface_name, StaData->if_index);

				ifmgr_data.if_index = StaData->if_index;
				ifmgr_data.if_type = WLAN_WAN;
				memcpy(ifmgr_data.mac_addr, StaData->mac_addr, sizeof(ifmgr_data.mac_addr));
				create_iface_instance(&ifmgr_data);
			}
			else
			{
				IPACMDBG_H("iface %s already up and act as %d mode: \n",
				IPACM_Iface::ipacmcfg->iface_table[ipa_interface_index].iface_name,
						IPACM_Iface::ipacmcfg->iface_table[ipa_interface_index].if_cat);
			}
			break;

		/* Add new instance open for eMBMS iface and wan iface */
		case IPA_WAN_EMBMS_LINK_UP_EVENT:
			ipa_interface_index = IPACM_Iface::iface_ipa_index_query(evt_data->if_index);
			/* change iface category from unknown to EMBMS_IF */
			if ((IPACM_Iface::ipacmcfg->ipacm_odu_enable == true) && (IPACM_Iface::ipacmcfg->ipacm_odu_embms_enable == true))
			{
				IPACMDBG(" ODU-mode enable or not (%d) \n",IPACM_Iface::ipacmcfg->ipacm_odu_enable);
				if(IPACM_Iface::ipacmcfg->iface_table[ipa_interface_index].if_cat == WAN_IF)
				{
					IPACM_Iface::ipacmcfg->iface_table[ipa_interface_index].if_cat=EMBMS_IF;
					IPACMDBG("WAN eMBMS (%s) link up, iface: %d: \n", IPACM_Iface::ipacmcfg->iface_table[ipa_interface_index].iface_name,evt_data->if_index);
					ifmgr_data.if_index = StaData->if_index;
					ifmgr_data.if_type = Q6_WAN;
					create_iface_instance(&ifmgr_data);
				}
				else
				{
					IPACMDBG("iface %s already up and act as %d mode: \n",IPACM_Iface::ipacmcfg->iface_table[ipa_interface_index].iface_name,IPACM_Iface::ipacmcfg->iface_table[ipa_interface_index].if_cat);
				}
			}
			break;

		default:
			break;
	}
	return;
}

int IPACM_IfaceManager::create_iface_instance(ipacm_ifacemgr_data *param)
{
	int if_index = param->if_index;
	ipacm_wan_iface_type is_sta_mode = param->if_type;

	int ipa_interface_index;
	ipa_interface_index = IPACM_Iface::iface_ipa_index_query(if_index);

	if(ipa_interface_index == INVALID_IFACE)
	{
			IPACMDBG_H("Unhandled interface received, fid: %d\n",if_index);
			return IPACM_SUCCESS;
	}

	/* check if duplicate instance*/
	if(SearchInstance(ipa_interface_index) == IPA_INSTANCE_NOT_FOUND)
	{
		/* IPA_INSTANCE_NOT_FOUND */
		switch(IPACM_Iface::ipacmcfg->iface_table[ipa_interface_index].if_cat)
		{

		case LAN_IF:
			{
				IPACMDBG_H("Creating Lan interface\n");
				IPACM_Lan *lan = new IPACM_Lan(ipa_interface_index);
				IPACM_EvtDispatcher::registr(IPA_ADDR_ADD_EVENT, lan);
				//IPACM_EvtDispatcher::registr(IPA_ROUTE_ADD_EVENT, lan);
				//IPACM_EvtDispatcher::registr(IPA_ROUTE_DEL_EVENT, lan);
				IPACM_EvtDispatcher::registr(IPA_NEIGH_CLIENT_IP_ADDR_ADD_EVENT, lan);
				IPACM_EvtDispatcher::registr(IPA_NEIGH_CLIENT_IP_ADDR_DEL_EVENT, lan);
				IPACM_EvtDispatcher::registr(IPA_SW_ROUTING_ENABLE, lan);
				IPACM_EvtDispatcher::registr(IPA_SW_ROUTING_DISABLE, lan);
#ifdef FEATURE_IPA_ANDROID
				IPACM_EvtDispatcher::registr(IPA_HANDLE_WAN_UP_TETHER, lan);
				IPACM_EvtDispatcher::registr(IPA_HANDLE_WAN_UP_V6_TETHER, lan);
				IPACM_EvtDispatcher::registr(IPA_HANDLE_WAN_DOWN_TETHER, lan);
				IPACM_EvtDispatcher::registr(IPA_HANDLE_WAN_DOWN_V6_TETHER, lan);
#else
				IPACM_EvtDispatcher::registr(IPA_HANDLE_WAN_UP, lan);
				IPACM_EvtDispatcher::registr(IPA_HANDLE_WAN_UP_V6, lan);
				IPACM_EvtDispatcher::registr(IPA_HANDLE_WAN_DOWN, lan);
				IPACM_EvtDispatcher::registr(IPA_HANDLE_WAN_DOWN_V6, lan);
#endif
				IPACM_EvtDispatcher::registr(IPA_CFG_CHANGE_EVENT, lan); 				// register for IPA_CFG_CHANGE event
				IPACM_EvtDispatcher::registr(IPA_PRIVATE_SUBNET_CHANGE_EVENT, lan); 	// register for IPA_PRIVATE_SUBNET_CHANGE_EVENT event
#ifdef FEATURE_IPA_ANDROID
				IPACM_EvtDispatcher::registr(IPA_TETHERING_STATS_UPDATE_EVENT, lan);
#endif
				IPACM_EvtDispatcher::registr(IPA_CRADLE_WAN_MODE_SWITCH, lan);
				IPACM_EvtDispatcher::registr(IPA_LINK_DOWN_EVENT, lan);
				/* IPA_LAN_DELETE_SELF should be always last */
				IPACM_EvtDispatcher::registr(IPA_LAN_DELETE_SELF, lan);
				IPACMDBG_H("ipa_LAN (%s):ipa_index (%d) instance open/registr ok\n", lan->dev_name, lan->ipa_if_num);
				registr(ipa_interface_index, lan);
				/* solve the new_addr comes earlier issue */
                                IPACM_Iface::iface_addr_query(if_index);
			}
			break;

		case ETH_IF:
			{
				IPACMDBG_H("Creating ETH interface in router mode\n");
				IPACM_Lan *ETH = new IPACM_Lan(ipa_interface_index);
				IPACM_EvtDispatcher::registr(IPA_ADDR_ADD_EVENT, ETH);
				IPACM_EvtDispatcher::registr(IPA_NEIGH_CLIENT_IP_ADDR_ADD_EVENT, ETH);
				IPACM_EvtDispatcher::registr(IPA_SW_ROUTING_ENABLE, ETH);
				IPACM_EvtDispatcher::registr(IPA_SW_ROUTING_DISABLE, ETH);
				IPACM_EvtDispatcher::registr(IPA_HANDLE_WAN_UP, ETH);
				IPACM_EvtDispatcher::registr(IPA_HANDLE_WAN_UP_V6, ETH);
				IPACM_EvtDispatcher::registr(IPA_HANDLE_WAN_DOWN, ETH);
				IPACM_EvtDispatcher::registr(IPA_HANDLE_WAN_DOWN_V6, ETH);
				IPACM_EvtDispatcher::registr(IPA_CRADLE_WAN_MODE_SWITCH, ETH);
				IPACM_EvtDispatcher::registr(IPA_LINK_DOWN_EVENT, ETH);
				/* IPA_LAN_DELETE_SELF should be always last */
				IPACM_EvtDispatcher::registr(IPA_LAN_DELETE_SELF, ETH);
				IPACMDBG_H("ipa_LAN (%s):ipa_index (%d) instance open/registr ok\n", ETH->dev_name, ETH->ipa_if_num);
				registr(ipa_interface_index, ETH);
				/* solve the new_addr comes earlier issue */
				IPACM_Iface::iface_addr_query(if_index);
			}
			break;

		case ODU_IF:
			{
				if(IPACM_Iface::ipacmcfg->ipacm_odu_router_mode == true)
				{
					IPACMDBG_H("Creating ODU interface in router mode\n");
					IPACM_Lan *odu = new IPACM_Lan(ipa_interface_index);
					IPACM_EvtDispatcher::registr(IPA_ADDR_ADD_EVENT, odu);
					IPACM_EvtDispatcher::registr(IPA_NEIGH_CLIENT_IP_ADDR_ADD_EVENT, odu);
					IPACM_EvtDispatcher::registr(IPA_NEIGH_CLIENT_IP_ADDR_DEL_EVENT, odu);
					IPACM_EvtDispatcher::registr(IPA_SW_ROUTING_ENABLE, odu);
					IPACM_EvtDispatcher::registr(IPA_SW_ROUTING_DISABLE, odu);
					IPACM_EvtDispatcher::registr(IPA_HANDLE_WAN_UP, odu);
					IPACM_EvtDispatcher::registr(IPA_HANDLE_WAN_UP_V6, odu);
					IPACM_EvtDispatcher::registr(IPA_HANDLE_WAN_DOWN, odu);
					IPACM_EvtDispatcher::registr(IPA_HANDLE_WAN_DOWN_V6, odu);
					IPACM_EvtDispatcher::registr(IPA_CRADLE_WAN_MODE_SWITCH, odu);
					IPACM_EvtDispatcher::registr(IPA_LINK_DOWN_EVENT, odu);
					/* IPA_LAN_DELETE_SELF should be always last */
					IPACM_EvtDispatcher::registr(IPA_LAN_DELETE_SELF, odu);
					IPACMDBG_H("ipa_LAN (%s):ipa_index (%d) instance open/registr ok\n", odu->dev_name, odu->ipa_if_num);
					registr(ipa_interface_index, odu);
					/* solve the new_addr comes earlier issue */
					IPACM_Iface::iface_addr_query(if_index);
				}
				else
				{
					IPACMDBG_H("Creating ODU interface in bridge mode\n");
					IPACM_Lan *odu = new IPACM_Lan(ipa_interface_index);
					IPACM_EvtDispatcher::registr(IPA_ADDR_ADD_EVENT, odu);
					IPACM_EvtDispatcher::registr(IPA_NEIGH_CLIENT_IP_ADDR_ADD_EVENT, odu);
					IPACM_EvtDispatcher::registr(IPA_SW_ROUTING_ENABLE, odu);
					IPACM_EvtDispatcher::registr(IPA_SW_ROUTING_DISABLE, odu);
					IPACM_EvtDispatcher::registr(IPA_LINK_DOWN_EVENT, odu);
					/* IPA_LAN_DELETE_SELF should be always last */
					IPACM_EvtDispatcher::registr(IPA_LAN_DELETE_SELF, odu);
					IPACMDBG_H("ipa_LAN (%s):ipa_index (%d) instance open/registr ok\n", odu->dev_name, odu->ipa_if_num);
					registr(ipa_interface_index, odu);
					/* solve the new_addr comes earlier issue */
					IPACM_Iface::iface_addr_query(if_index);
				}
			}
			break;

		case WLAN_IF:
			{
				IPACMDBG_H("Creating WLan interface\n");
				IPACM_Wlan *wl = new IPACM_Wlan(ipa_interface_index);
				IPACM_EvtDispatcher::registr(IPA_ADDR_ADD_EVENT, wl);
				IPACM_EvtDispatcher::registr(IPA_ROUTE_DEL_EVENT, wl);
				IPACM_EvtDispatcher::registr(IPA_WLAN_CLIENT_ADD_EVENT, wl);
				IPACM_EvtDispatcher::registr(IPA_WLAN_CLIENT_ADD_EVENT_EX, wl);
				IPACM_EvtDispatcher::registr(IPA_WLAN_CLIENT_DEL_EVENT, wl);
				IPACM_EvtDispatcher::registr(IPA_WLAN_CLIENT_POWER_SAVE_EVENT, wl);
				IPACM_EvtDispatcher::registr(IPA_WLAN_CLIENT_RECOVER_EVENT, wl);
				IPACM_EvtDispatcher::registr(IPA_NEIGH_CLIENT_IP_ADDR_ADD_EVENT, wl);
				IPACM_EvtDispatcher::registr(IPA_SW_ROUTING_ENABLE, wl);
				IPACM_EvtDispatcher::registr(IPA_SW_ROUTING_DISABLE, wl);
#ifdef FEATURE_IPA_ANDROID
				IPACM_EvtDispatcher::registr(IPA_HANDLE_WAN_UP_TETHER, wl);
				IPACM_EvtDispatcher::registr(IPA_HANDLE_WAN_UP_V6_TETHER, wl);
				IPACM_EvtDispatcher::registr(IPA_HANDLE_WAN_DOWN_TETHER, wl);
				IPACM_EvtDispatcher::registr(IPA_HANDLE_WAN_DOWN_V6_TETHER, wl);
#else
				IPACM_EvtDispatcher::registr(IPA_HANDLE_WAN_UP, wl);
				IPACM_EvtDispatcher::registr(IPA_HANDLE_WAN_UP_V6, wl);
				IPACM_EvtDispatcher::registr(IPA_HANDLE_WAN_DOWN, wl);
				IPACM_EvtDispatcher::registr(IPA_HANDLE_WAN_DOWN_V6, wl);
#endif
				IPACM_EvtDispatcher::registr(IPA_PRIVATE_SUBNET_CHANGE_EVENT, wl); 	// register for IPA_PRIVATE_SUBNET_CHANGE_EVENT event
#ifdef FEATURE_ETH_BRIDGE_LE
				IPACM_EvtDispatcher::registr(IPA_CFG_CHANGE_EVENT, wl);
#endif
				IPACM_EvtDispatcher::registr(IPA_CRADLE_WAN_MODE_SWITCH, wl);
				IPACM_EvtDispatcher::registr(IPA_WLAN_LINK_DOWN_EVENT, wl);
#ifndef FEATURE_IPA_ANDROID
				IPACM_EvtDispatcher::registr(IPA_WLAN_SWITCH_TO_SCC, wl);
				IPACM_EvtDispatcher::registr(IPA_WLAN_SWITCH_TO_MCC, wl);
#else
				IPACM_EvtDispatcher::registr(IPA_TETHERING_STATS_UPDATE_EVENT, wl);
#endif
				/* IPA_LAN_DELETE_SELF should be always last */
				IPACM_EvtDispatcher::registr(IPA_LAN_DELETE_SELF, wl);
				IPACMDBG_H("ipa_WLAN (%s):ipa_index (%d) instance open/registr ok\n", wl->dev_name, wl->ipa_if_num);
				registr(ipa_interface_index, wl);
				/* solve the new_addr comes earlier issue */
	            IPACM_Iface::iface_addr_query(if_index);
			}
			break;

		case WAN_IF:
			{
				if((IPACM_Iface::ipacmcfg->ipacm_odu_enable == false) || (IPACM_Iface::ipacmcfg->ipacm_odu_router_mode == true))
				{
					IPACMDBG_H("Creating Wan interface\n");
					IPACM_Wan *w;
					if(is_sta_mode == WLAN_WAN)
					{
						w = new IPACM_Wan(ipa_interface_index, is_sta_mode, param->mac_addr);
					}
					else
					{
						w = new IPACM_Wan(ipa_interface_index, is_sta_mode, NULL);
					}
					IPACM_EvtDispatcher::registr(IPA_ADDR_ADD_EVENT, w);
#ifdef FEATURE_IPA_ANDROID
					IPACM_EvtDispatcher::registr(IPA_WAN_UPSTREAM_ROUTE_ADD_EVENT, w);
					IPACM_EvtDispatcher::registr(IPA_WAN_UPSTREAM_ROUTE_DEL_EVENT, w);
					if(is_sta_mode == Q6_WAN)
					{
						IPACM_EvtDispatcher::registr(IPA_NETWORK_STATS_UPDATE_EVENT, w);
					};
#else/* defined(FEATURE_IPA_ANDROID) */
					IPACM_EvtDispatcher::registr(IPA_ROUTE_ADD_EVENT, w);
					IPACM_EvtDispatcher::registr(IPA_ROUTE_DEL_EVENT, w);
#endif /* not defined(FEATURE_IPA_ANDROID)*/
					IPACM_EvtDispatcher::registr(IPA_FIREWALL_CHANGE_EVENT, w);
					IPACM_EvtDispatcher::registr(IPA_NEIGH_CLIENT_IP_ADDR_ADD_EVENT, w);
					IPACM_EvtDispatcher::registr(IPA_SW_ROUTING_ENABLE, w);
					IPACM_EvtDispatcher::registr(IPA_SW_ROUTING_DISABLE, w);
					IPACM_EvtDispatcher::registr(IPA_CFG_CHANGE_EVENT, w); 		// register for IPA_CFG_CHANGE event
					IPACM_EvtDispatcher::registr(IPA_WAN_XLAT_CONNECT_EVENT, w);
					if(is_sta_mode == WLAN_WAN)
					{
						IPACM_EvtDispatcher::registr(IPA_WLAN_LINK_DOWN_EVENT, w); // for STA mode
#ifndef FEATURE_IPA_ANDROID
						IPACM_EvtDispatcher::registr(IPA_WLAN_SWITCH_TO_SCC, w);
						IPACM_EvtDispatcher::registr(IPA_WLAN_SWITCH_TO_MCC, w);
#endif
					}
					else
					{
						IPACM_EvtDispatcher::registr(IPA_LINK_DOWN_EVENT, w);
					}

					IPACMDBG_H("ipa_WAN (%s):ipa_index (%d) instance open/registr ok\n", w->dev_name, w->ipa_if_num);
					registr(ipa_interface_index, w);
					/* solve the new_addr comes earlier issue */
					IPACM_Iface::iface_addr_query(if_index);
				}
			}
			break;

	    /* WAN-eMBMS instance */
		case EMBMS_IF:
			{
				IPACMDBG("Creating Wan-eMBSM interface\n");
				IPACM_Wan *embms = new IPACM_Wan(ipa_interface_index, is_sta_mode, NULL);
				IPACM_EvtDispatcher::registr(IPA_LINK_DOWN_EVENT, embms);
				IPACMDBG("ipa_WAN (%s):ipa_index (%d) instance open/registr ok\n", embms->dev_name, embms->ipa_if_num);
				registr(ipa_interface_index, embms);
			}
			break;

		default:
			IPACMDBG_H("Unhandled interface category received iface name: %s, category: %d\n",
			            IPACM_Iface::ipacmcfg->iface_table[ipa_interface_index].iface_name,
						       IPACM_Iface::ipacmcfg->iface_table[ipa_interface_index].if_cat);
			return IPACM_SUCCESS;
		}
	}
	return IPACM_SUCCESS;
}


int IPACM_IfaceManager::registr(int ipa_if_index, IPACM_Listener *obj)
{
	iface_instances *tmp = head,*nw;

	nw = (iface_instances *)malloc(sizeof(iface_instances));
	if(nw != NULL)
	{
		nw->ipa_if_index = ipa_if_index;
		nw->obj = obj;
		nw->next = NULL;
	}
	else
	{
		return IPACM_FAILURE;
	}

	if(head == NULL)
	{
		head = nw;
	}
	else
	{
		while(tmp->next)
		{
			tmp = tmp->next;
		}
		tmp->next = nw;
	}
	return IPACM_SUCCESS;
}

int IPACM_IfaceManager::deregistr(IPACM_Listener *param)
{
	iface_instances *tmp = head,*tmp1,*prev = head;

	while(tmp != NULL)
	{
		if(tmp->obj == param)
		{
			tmp1 = tmp;
			if(tmp == head)
			{
				head = head->next;
			}
			else if(tmp->next == NULL)
			{
				prev->next = NULL;
			}
			else
			{
				prev->next = tmp->next;
			}

			tmp = tmp->next;
			free(tmp1);
		}
		else
		{
			prev = tmp;
			tmp = tmp->next;
		}
	}
	return IPACM_SUCCESS;
}


int IPACM_IfaceManager::SearchInstance(int ipa_if_index)
{

	iface_instances *tmp = head;

	while(tmp != NULL)
	{
		if(ipa_if_index == tmp->ipa_if_index)
		{
			IPACMDBG_H("Find existed iface-instance name: %s\n",
							 IPACM_Iface::ipacmcfg->iface_table[ipa_if_index].iface_name);
			return IPA_INSTANCE_FOUND;
		}
		tmp = tmp->next;
	}

	IPACMDBG_H("No existed iface-instance name: %s,\n",
					 IPACM_Iface::ipacmcfg->iface_table[ipa_if_index].iface_name);

	return IPA_INSTANCE_NOT_FOUND;
}
