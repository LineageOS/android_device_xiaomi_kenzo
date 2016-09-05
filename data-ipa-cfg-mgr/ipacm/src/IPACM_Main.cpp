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
	IPACM_Main.cpp

	@brief
	This file implements the IPAM functionality.

	@Author
	Skylar Chang

*/
/******************************************************************************

                      IPCM_MAIN.C

******************************************************************************/

#include <sys/socket.h>
#include <signal.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <linux/if.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <fcntl.h>
#include <sys/inotify.h>
#include <stdlib.h>
#include <signal.h>
#include "linux/ipa_qmi_service_v01.h"

#include "IPACM_CmdQueue.h"
#include "IPACM_EvtDispatcher.h"
#include "IPACM_Defs.h"
#include "IPACM_Neighbor.h"
#include "IPACM_IfaceManager.h"
#include "IPACM_Log.h"

#include "IPACM_ConntrackListener.h"
#include "IPACM_ConntrackClient.h"
#include "IPACM_Netlink.h"

/* not defined(FEATURE_IPA_ANDROID)*/
#ifndef FEATURE_IPA_ANDROID
#include "IPACM_LanToLan.h"
#endif

#define IPA_DRIVER  "/dev/ipa"

#define IPACM_FIREWALL_FILE_NAME    "mobileap_firewall.xml"
#define IPACM_CFG_FILE_NAME    "IPACM_cfg.xml"
#ifdef FEATURE_IPA_ANDROID
#define IPACM_PID_FILE "/data/misc/ipa/ipacm.pid"
#define IPACM_DIR_NAME     "/data"
#else/* defined(FEATURE_IPA_ANDROID) */
#define IPACM_PID_FILE "/etc/ipacm.pid"
#define IPACM_DIR_NAME     "/etc"
#endif /* defined(NOT FEATURE_IPA_ANDROID)*/
#define IPACM_NAME "ipacm"

#define INOTIFY_EVENT_SIZE  (sizeof(struct inotify_event))
#define INOTIFY_BUF_LEN     (INOTIFY_EVENT_SIZE + 2*sizeof(IPACM_FIREWALL_FILE_NAME))

#define IPA_DRIVER_WLAN_EVENT_MAX_OF_ATTRIBS  3
#define IPA_DRIVER_WLAN_EVENT_SIZE  (sizeof(struct ipa_wlan_msg_ex)+ IPA_DRIVER_WLAN_EVENT_MAX_OF_ATTRIBS*sizeof(ipa_wlan_hdr_attrib_val))
#define IPA_DRIVER_PIPE_STATS_EVENT_SIZE  (sizeof(struct ipa_get_data_stats_resp_msg_v01))
#define IPA_DRIVER_WLAN_META_MSG    (sizeof(struct ipa_msg_meta))
#define IPA_DRIVER_WLAN_BUF_LEN     (IPA_DRIVER_PIPE_STATS_EVENT_SIZE + IPA_DRIVER_WLAN_META_MSG)

uint32_t ipacm_event_stats[IPACM_EVENT_MAX];
bool ipacm_logging = true;

void ipa_is_ipacm_running(void);
int ipa_get_if_index(char *if_name, int *if_index);

/* start netlink socket monitor*/
void* netlink_start(void *param)
{
	ipa_nl_sk_fd_set_info_t sk_fdset;
	int ret_val = 0;
	memset(&sk_fdset, 0, sizeof(ipa_nl_sk_fd_set_info_t));
	IPACMDBG_H("netlink starter memset sk_fdset succeeds\n");
	ret_val = ipa_nl_listener_init(NETLINK_ROUTE, (RTMGRP_IPV4_ROUTE | RTMGRP_IPV6_ROUTE | RTMGRP_LINK |
																										RTMGRP_IPV4_IFADDR | RTMGRP_IPV6_IFADDR | RTMGRP_NEIGH |
																										RTNLGRP_IPV6_PREFIX),
																 &sk_fdset, ipa_nl_recv_msg);

	if (ret_val != IPACM_SUCCESS)
	{
		IPACMERR("Failed to initialize IPA netlink event listener\n");
		return NULL;
	}

	return NULL;
}

/* start firewall-rule monitor*/
void* firewall_monitor(void *param)
{
	int length;
	int wd;
	char buffer[INOTIFY_BUF_LEN];
	int inotify_fd;
	ipacm_cmd_q_data evt_data;
	uint32_t mask = IN_MODIFY | IN_MOVE;

	inotify_fd = inotify_init();
	if (inotify_fd < 0)
	{
		PERROR("inotify_init");
	}

	IPACMDBG_H("Waiting for nofications in dir %s with mask: 0x%x\n", IPACM_DIR_NAME, mask);

	wd = inotify_add_watch(inotify_fd,
												 IPACM_DIR_NAME,
												 mask);

	while (1)
	{
		length = read(inotify_fd, buffer, INOTIFY_BUF_LEN);
		if (length < 0)
		{
			IPACMERR("inotify read() error return length: %d and mask: 0x%x\n", length, mask);
			continue;
		}

		struct inotify_event* event;
		event = (struct inotify_event*)malloc(length);
		if(event == NULL)
		{
			IPACMERR("Failed to allocate memory.\n");
			return NULL;
		}
		memset(event, 0, length);
		memcpy(event, buffer, length);

		if (event->len > 0)
		{
			if ( (event->mask & IN_MODIFY) || (event->mask & IN_MOVE))
			{
				if (event->mask & IN_ISDIR)
				{
					IPACMDBG_H("The directory %s was 0x%x\n", event->name, event->mask);
				}
				else if (!strncmp(event->name, IPACM_FIREWALL_FILE_NAME, event->len)) // firewall_rule change
				{
					IPACMDBG_H("File \"%s\" was 0x%x\n", event->name, event->mask);
					IPACMDBG_H("The interested file %s .\n", IPACM_FIREWALL_FILE_NAME);

					evt_data.event = IPA_FIREWALL_CHANGE_EVENT;
					evt_data.evt_data = NULL;

					/* Insert IPA_FIREWALL_CHANGE_EVENT to command queue */
					IPACM_EvtDispatcher::PostEvt(&evt_data);
				}
				else if (!strncmp(event->name, IPACM_CFG_FILE_NAME, event->len)) // IPACM_configuration change
				{
					IPACMDBG_H("File \"%s\" was 0x%x\n", event->name, event->mask);
					IPACMDBG_H("The interested file %s .\n", IPACM_CFG_FILE_NAME);

					evt_data.event = IPA_CFG_CHANGE_EVENT;
					evt_data.evt_data = NULL;

					/* Insert IPA_FIREWALL_CHANGE_EVENT to command queue */
					IPACM_EvtDispatcher::PostEvt(&evt_data);
				}
			}
			IPACMDBG_H("Received monitoring event %s.\n", event->name);
		}
		free(event);
	}

	(void)inotify_rm_watch(inotify_fd, wd);
	(void)close(inotify_fd);
	return NULL;
}


/* start IPACM wan-driver notifier */
void* ipa_driver_msg_notifier(void *param)
{
	int length, fd, cnt;
	char buffer[IPA_DRIVER_WLAN_BUF_LEN];
	struct ipa_msg_meta event_hdr;
	struct ipa_ecm_msg event_ecm;
	struct ipa_wan_msg event_wan;
	struct ipa_wlan_msg_ex event_ex_o;
	struct ipa_wlan_msg *event_wlan=NULL;
	struct ipa_wlan_msg_ex *event_ex= NULL;
	struct ipa_get_data_stats_resp_msg_v01 event_data_stats;
	struct ipa_get_apn_data_stats_resp_msg_v01 event_network_stats;

	ipacm_cmd_q_data evt_data;
	ipacm_event_data_mac *data = NULL;
	ipacm_event_data_fid *data_fid = NULL;
	ipacm_event_data_iptype *data_iptype = NULL;
	ipacm_event_data_wlan_ex *data_ex;
	ipa_get_data_stats_resp_msg_v01 *data_tethering_stats = NULL;
	ipa_get_apn_data_stats_resp_msg_v01 *data_network_stats = NULL;

	ipacm_cmd_q_data new_neigh_evt;
	ipacm_event_data_all* new_neigh_data;

	fd = open(IPA_DRIVER, O_RDWR);
	if (fd < 0)
	{
		IPACMERR("Failed opening %s.\n", IPA_DRIVER);
		return NULL;
	}

	while (1)
	{
		IPACMDBG_H("Waiting for nofications from IPA driver \n");
		memset(buffer, 0, sizeof(buffer));
		memset(&evt_data, 0, sizeof(evt_data));
		memset(&new_neigh_evt, 0, sizeof(ipacm_cmd_q_data));
		new_neigh_data = NULL;
		data = NULL;
		data_fid = NULL;
		data_tethering_stats = NULL;
		data_network_stats = NULL;

		length = read(fd, buffer, IPA_DRIVER_WLAN_BUF_LEN);
		if (length < 0)
		{
			PERROR("didn't read IPA_driver correctly");
			continue;
		}

		memcpy(&event_hdr, buffer,sizeof(struct ipa_msg_meta));
		IPACMDBG_H("Message type: %d\n", event_hdr.msg_type);
		IPACMDBG_H("Event header length received: %d\n",event_hdr.msg_len);

		/* Insert WLAN_DRIVER_EVENT to command queue */
		switch (event_hdr.msg_type)
		{

		case SW_ROUTING_ENABLE:
			IPACMDBG_H("Received SW_ROUTING_ENABLE\n");
			evt_data.event = IPA_SW_ROUTING_ENABLE;
			IPACMDBG_H("Not supported anymore\n");
			continue;

		case SW_ROUTING_DISABLE:
			IPACMDBG_H("Received SW_ROUTING_DISABLE\n");
			evt_data.event = IPA_SW_ROUTING_DISABLE;
			IPACMDBG_H("Not supported anymore\n");
			continue;

		case WLAN_AP_CONNECT:
			event_wlan = (struct ipa_wlan_msg *) (buffer + sizeof(struct ipa_msg_meta));
			IPACMDBG_H("Received WLAN_AP_CONNECT name: %s\n",event_wlan->name);
			IPACMDBG_H("AP Mac Address %02x:%02x:%02x:%02x:%02x:%02x\n",
							 event_wlan->mac_addr[0], event_wlan->mac_addr[1], event_wlan->mac_addr[2],
							 event_wlan->mac_addr[3], event_wlan->mac_addr[4], event_wlan->mac_addr[5]);
                        data_fid = (ipacm_event_data_fid *)malloc(sizeof(ipacm_event_data_fid));
			if(data_fid == NULL)
			{
				IPACMERR("unable to allocate memory for event_wlan data_fid\n");
				return NULL;
			}
			ipa_get_if_index(event_wlan->name, &(data_fid->if_index));
			evt_data.event = IPA_WLAN_AP_LINK_UP_EVENT;
			evt_data.evt_data = data_fid;
			break;

		case WLAN_AP_DISCONNECT:
			event_wlan = (struct ipa_wlan_msg *)(buffer + sizeof(struct ipa_msg_meta));
			IPACMDBG_H("Received WLAN_AP_DISCONNECT name: %s\n",event_wlan->name);
			IPACMDBG_H("AP Mac Address %02x:%02x:%02x:%02x:%02x:%02x\n",
							 event_wlan->mac_addr[0], event_wlan->mac_addr[1], event_wlan->mac_addr[2],
							 event_wlan->mac_addr[3], event_wlan->mac_addr[4], event_wlan->mac_addr[5]);
                        data_fid = (ipacm_event_data_fid *)malloc(sizeof(ipacm_event_data_fid));
			if(data_fid == NULL)
			{
				IPACMERR("unable to allocate memory for event_wlan data_fid\n");
				return NULL;
			}
			ipa_get_if_index(event_wlan->name, &(data_fid->if_index));
			evt_data.event = IPA_WLAN_LINK_DOWN_EVENT;
			evt_data.evt_data = data_fid;
			break;
		case WLAN_STA_CONNECT:
			event_wlan = (struct ipa_wlan_msg *)(buffer + sizeof(struct ipa_msg_meta));
			IPACMDBG_H("Received WLAN_STA_CONNECT name: %s\n",event_wlan->name);
			IPACMDBG_H("STA Mac Address %02x:%02x:%02x:%02x:%02x:%02x\n",
							 event_wlan->mac_addr[0], event_wlan->mac_addr[1], event_wlan->mac_addr[2],
							 event_wlan->mac_addr[3], event_wlan->mac_addr[4], event_wlan->mac_addr[5]);
			data = (ipacm_event_data_mac *)malloc(sizeof(ipacm_event_data_mac));
			if(data == NULL)
			{
				IPACMERR("unable to allocate memory for event_wlan data_fid\n");
				return NULL;
			}
			memcpy(data->mac_addr,
				 event_wlan->mac_addr,
				 sizeof(event_wlan->mac_addr));
			ipa_get_if_index(event_wlan->name, &(data->if_index));
			evt_data.event = IPA_WLAN_STA_LINK_UP_EVENT;
			evt_data.evt_data = data;
			break;

		case WLAN_STA_DISCONNECT:
			event_wlan = (struct ipa_wlan_msg *)(buffer + sizeof(struct ipa_msg_meta));
			IPACMDBG_H("Received WLAN_STA_DISCONNECT name: %s\n",event_wlan->name);
			IPACMDBG_H("STA Mac Address %02x:%02x:%02x:%02x:%02x:%02x\n",
							 event_wlan->mac_addr[0], event_wlan->mac_addr[1], event_wlan->mac_addr[2],
							 event_wlan->mac_addr[3], event_wlan->mac_addr[4], event_wlan->mac_addr[5]);
                        data_fid = (ipacm_event_data_fid *)malloc(sizeof(ipacm_event_data_fid));
			if(data_fid == NULL)
			{
				IPACMERR("unable to allocate memory for event_wlan data_fid\n");
				return NULL;
			}
			ipa_get_if_index(event_wlan->name, &(data_fid->if_index));
			evt_data.event = IPA_WLAN_LINK_DOWN_EVENT;
			evt_data.evt_data = data_fid;
			break;

		case WLAN_CLIENT_CONNECT:
			event_wlan = (struct ipa_wlan_msg *)(buffer + sizeof(struct ipa_msg_meta));
			IPACMDBG_H("Received WLAN_CLIENT_CONNECT\n");
			IPACMDBG_H("Mac Address %02x:%02x:%02x:%02x:%02x:%02x\n",
							 event_wlan->mac_addr[0], event_wlan->mac_addr[1], event_wlan->mac_addr[2],
							 event_wlan->mac_addr[3], event_wlan->mac_addr[4], event_wlan->mac_addr[5]);
		        data = (ipacm_event_data_mac *)malloc(sizeof(ipacm_event_data_mac));
		        if (data == NULL)
		        {
		    	        IPACMERR("unable to allocate memory for event_wlan data\n");
		    	        return NULL;
		        }
			memcpy(data->mac_addr,
						 event_wlan->mac_addr,
						 sizeof(event_wlan->mac_addr));
			ipa_get_if_index(event_wlan->name, &(data->if_index));
		        evt_data.event = IPA_WLAN_CLIENT_ADD_EVENT;
			evt_data.evt_data = data;
			break;

		case WLAN_CLIENT_CONNECT_EX:
			IPACMDBG_H("Received WLAN_CLIENT_CONNECT_EX\n");

			memcpy(&event_ex_o, buffer + sizeof(struct ipa_msg_meta),sizeof(struct ipa_wlan_msg_ex));
			if(event_ex_o.num_of_attribs > IPA_DRIVER_WLAN_EVENT_MAX_OF_ATTRIBS)
			{
				IPACMERR("buffer size overflow\n");
				return NULL;
			}
			length = sizeof(ipa_wlan_msg_ex)+ event_ex_o.num_of_attribs * sizeof(ipa_wlan_hdr_attrib_val);
			IPACMDBG_H("num_of_attribs %d, length %d\n", event_ex_o.num_of_attribs, length);
			event_ex = (ipa_wlan_msg_ex *)malloc(length);
			if(event_ex == NULL )
			{
				IPACMERR("Unable to allocate memory\n");
				return NULL;
			}
			memcpy(event_ex, buffer + sizeof(struct ipa_msg_meta), length);
			data_ex = (ipacm_event_data_wlan_ex *)malloc(sizeof(ipacm_event_data_wlan_ex) + event_ex_o.num_of_attribs * sizeof(ipa_wlan_hdr_attrib_val));
		    if (data_ex == NULL)
		    {
				IPACMERR("unable to allocate memory for event data\n");
		    	return NULL;
		    }
			data_ex->num_of_attribs = event_ex->num_of_attribs;

			memcpy(data_ex->attribs,
						event_ex->attribs,
						event_ex->num_of_attribs * sizeof(ipa_wlan_hdr_attrib_val));

			ipa_get_if_index(event_ex->name, &(data_ex->if_index));
			evt_data.event = IPA_WLAN_CLIENT_ADD_EVENT_EX;
			evt_data.evt_data = data_ex;

			/* Construct new_neighbor msg with netdev device internally */
			new_neigh_data = (ipacm_event_data_all*)malloc(sizeof(ipacm_event_data_all));
			if(new_neigh_data == NULL)
			{
				IPACMERR("Failed to allocate memory.\n");
				return NULL;
			}
			memset(new_neigh_data, 0, sizeof(ipacm_event_data_all));
			new_neigh_data->iptype = IPA_IP_v6;
			for(cnt = 0; cnt < event_ex->num_of_attribs; cnt++)
			{
				if(event_ex->attribs[cnt].attrib_type == WLAN_HDR_ATTRIB_MAC_ADDR)
				{
					memcpy(new_neigh_data->mac_addr, event_ex->attribs[cnt].u.mac_addr, sizeof(new_neigh_data->mac_addr));
					IPACMDBG_H("Mac Address %02x:%02x:%02x:%02x:%02x:%02x\n",
								 event_ex->attribs[cnt].u.mac_addr[0], event_ex->attribs[cnt].u.mac_addr[1], event_ex->attribs[cnt].u.mac_addr[2],
								 event_ex->attribs[cnt].u.mac_addr[3], event_ex->attribs[cnt].u.mac_addr[4], event_ex->attribs[cnt].u.mac_addr[5]);
				}
				else if(event_ex->attribs[cnt].attrib_type == WLAN_HDR_ATTRIB_STA_ID)
				{
					IPACMDBG_H("Wlan client id %d\n",event_ex->attribs[cnt].u.sta_id);
				}
				else
				{
					IPACMDBG_H("Wlan message has unexpected type!\n");
				}
			}
			new_neigh_data->if_index = data_ex->if_index;
			new_neigh_evt.evt_data = (void*)new_neigh_data;
			new_neigh_evt.event = IPA_NEW_NEIGH_EVENT;
			free(event_ex);
			break;

		case WLAN_CLIENT_DISCONNECT:
			IPACMDBG_H("Received WLAN_CLIENT_DISCONNECT\n");
			event_wlan = (struct ipa_wlan_msg *)(buffer + sizeof(struct ipa_msg_meta));
			IPACMDBG_H("Mac Address %02x:%02x:%02x:%02x:%02x:%02x\n",
							 event_wlan->mac_addr[0], event_wlan->mac_addr[1], event_wlan->mac_addr[2],
							 event_wlan->mac_addr[3], event_wlan->mac_addr[4], event_wlan->mac_addr[5]);
		        data = (ipacm_event_data_mac *)malloc(sizeof(ipacm_event_data_mac));
		        if (data == NULL)
		        {
		    	        IPACMERR("unable to allocate memory for event_wlan data\n");
		    	        return NULL;
		        }
			memcpy(data->mac_addr,
						 event_wlan->mac_addr,
						 sizeof(event_wlan->mac_addr));
			ipa_get_if_index(event_wlan->name, &(data->if_index));
			evt_data.event = IPA_WLAN_CLIENT_DEL_EVENT;
			evt_data.evt_data = data;
			break;

		case WLAN_CLIENT_POWER_SAVE_MODE:
			IPACMDBG_H("Received WLAN_CLIENT_POWER_SAVE_MODE\n");
			event_wlan = (struct ipa_wlan_msg *)(buffer + sizeof(struct ipa_msg_meta));
			IPACMDBG_H("Mac Address %02x:%02x:%02x:%02x:%02x:%02x\n",
							 event_wlan->mac_addr[0], event_wlan->mac_addr[1], event_wlan->mac_addr[2],
							 event_wlan->mac_addr[3], event_wlan->mac_addr[4], event_wlan->mac_addr[5]);
		        data = (ipacm_event_data_mac *)malloc(sizeof(ipacm_event_data_mac));
		        if (data == NULL)
		        {
		    	        IPACMERR("unable to allocate memory for event_wlan data\n");
		    	        return NULL;
		        }
			memcpy(data->mac_addr,
						 event_wlan->mac_addr,
						 sizeof(event_wlan->mac_addr));
			ipa_get_if_index(event_wlan->name, &(data->if_index));
			evt_data.event = IPA_WLAN_CLIENT_POWER_SAVE_EVENT;
			evt_data.evt_data = data;
			break;

		case WLAN_CLIENT_NORMAL_MODE:
			IPACMDBG_H("Received WLAN_CLIENT_NORMAL_MODE\n");
			event_wlan = (struct ipa_wlan_msg *)(buffer + sizeof(struct ipa_msg_meta));
			IPACMDBG_H("Mac Address %02x:%02x:%02x:%02x:%02x:%02x\n",
							 event_wlan->mac_addr[0], event_wlan->mac_addr[1], event_wlan->mac_addr[2],
							 event_wlan->mac_addr[3], event_wlan->mac_addr[4], event_wlan->mac_addr[5]);
		        data = (ipacm_event_data_mac *)malloc(sizeof(ipacm_event_data_mac));
		        if (data == NULL)
		        {
		    	       IPACMERR("unable to allocate memory for event_wlan data\n");
		    	       return NULL;
		        }
			memcpy(data->mac_addr,
						 event_wlan->mac_addr,
						 sizeof(event_wlan->mac_addr));
			ipa_get_if_index(event_wlan->name, &(data->if_index));
			evt_data.evt_data = data;
			evt_data.event = IPA_WLAN_CLIENT_RECOVER_EVENT;
			break;

		case ECM_CONNECT:
			memcpy(&event_ecm, buffer + sizeof(struct ipa_msg_meta), sizeof(struct ipa_ecm_msg));
			IPACMDBG_H("Received ECM_CONNECT name: %s\n",event_ecm.name);
			data_fid = (ipacm_event_data_fid *)malloc(sizeof(ipacm_event_data_fid));
			if(data_fid == NULL)
			{
				IPACMERR("unable to allocate memory for event_ecm data_fid\n");
				return NULL;
			}
			data_fid->if_index = event_ecm.ifindex;
			evt_data.event = IPA_USB_LINK_UP_EVENT;
			evt_data.evt_data = data_fid;
			break;

		case ECM_DISCONNECT:
			memcpy(&event_ecm, buffer + sizeof(struct ipa_msg_meta), sizeof(struct ipa_ecm_msg));
			IPACMDBG_H("Received ECM_DISCONNECT name: %s\n",event_ecm.name);
			data_fid = (ipacm_event_data_fid *)malloc(sizeof(ipacm_event_data_fid));
			if(data_fid == NULL)
			{
				IPACMERR("unable to allocate memory for event_ecm data_fid\n");
				return NULL;
			}
			data_fid->if_index = event_ecm.ifindex;
			evt_data.event = IPA_LINK_DOWN_EVENT;
			evt_data.evt_data = data_fid;
			break;
		/* Add for 8994 Android case */
		case WAN_UPSTREAM_ROUTE_ADD:
			memcpy(&event_wan, buffer + sizeof(struct ipa_msg_meta), sizeof(struct ipa_wan_msg));
			IPACMDBG_H("Received WAN_UPSTREAM_ROUTE_ADD name: %s, tethered name: %s\n", event_wan.upstream_ifname, event_wan.tethered_ifname);
			data_iptype = (ipacm_event_data_iptype *)malloc(sizeof(ipacm_event_data_iptype));
			if(data_iptype == NULL)
			{
				IPACMERR("unable to allocate memory for event_ecm data_iptype\n");
				return NULL;
			}
			ipa_get_if_index(event_wan.upstream_ifname, &(data_iptype->if_index));
			ipa_get_if_index(event_wan.tethered_ifname, &(data_iptype->if_index_tether));
			data_iptype->iptype = event_wan.ip;
			IPACMDBG_H("Received WAN_UPSTREAM_ROUTE_ADD: fid(%d) tether_fid(%d) ip-type(%d)\n", data_iptype->if_index,
					data_iptype->if_index_tether, data_iptype->iptype);
			evt_data.event = IPA_WAN_UPSTREAM_ROUTE_ADD_EVENT;
			evt_data.evt_data = data_iptype;
			break;
		case WAN_UPSTREAM_ROUTE_DEL:
			memcpy(&event_wan, buffer + sizeof(struct ipa_msg_meta), sizeof(struct ipa_wan_msg));
			IPACMDBG_H("Received WAN_UPSTREAM_ROUTE_DEL name: %s, tethered name: %s\n", event_wan.upstream_ifname, event_wan.tethered_ifname);
			data_iptype = (ipacm_event_data_iptype *)malloc(sizeof(ipacm_event_data_iptype));
			if(data_iptype == NULL)
			{
				IPACMERR("unable to allocate memory for event_ecm data_iptype\n");
				return NULL;
			}
			ipa_get_if_index(event_wan.upstream_ifname, &(data_iptype->if_index));
			ipa_get_if_index(event_wan.tethered_ifname, &(data_iptype->if_index_tether));
			data_iptype->iptype = event_wan.ip;
			IPACMDBG_H("Received WAN_UPSTREAM_ROUTE_DEL: fid(%d) ip-type(%d)\n", data_iptype->if_index, data_iptype->iptype);
			evt_data.event = IPA_WAN_UPSTREAM_ROUTE_DEL_EVENT;
			evt_data.evt_data = data_iptype;
			break;
		/* End of adding for 8994 Android case */

		/* Add for embms case */
		case WAN_EMBMS_CONNECT:
			memcpy(&event_wan, buffer + sizeof(struct ipa_msg_meta), sizeof(struct ipa_wan_msg));
			IPACMDBG("Received WAN_EMBMS_CONNECT name: %s\n",event_wan.upstream_ifname);
			data_fid = (ipacm_event_data_fid *)malloc(sizeof(ipacm_event_data_fid));
			if(data_fid == NULL)
			{
				IPACMERR("unable to allocate memory for event data_fid\n");
				return NULL;
			}
			ipa_get_if_index(event_wan.upstream_ifname, &(data_fid->if_index));
			evt_data.event = IPA_WAN_EMBMS_LINK_UP_EVENT;
			evt_data.evt_data = data_fid;
			break;

		case WLAN_SWITCH_TO_SCC:
			IPACMDBG_H("Received WLAN_SWITCH_TO_SCC\n");
		case WLAN_WDI_ENABLE:
			IPACMDBG_H("Received WLAN_WDI_ENABLE\n");
			if (IPACM_Iface::ipacmcfg->isMCC_Mode == true)
			{
				IPACM_Iface::ipacmcfg->isMCC_Mode = false;
				evt_data.event = IPA_WLAN_SWITCH_TO_SCC;
				break;
			}
			continue;
		case WLAN_SWITCH_TO_MCC:
			IPACMDBG_H("Received WLAN_SWITCH_TO_MCC\n");
		case WLAN_WDI_DISABLE:
			IPACMDBG_H("Received WLAN_WDI_DISABLE\n");
			if (IPACM_Iface::ipacmcfg->isMCC_Mode == false)
			{
				IPACM_Iface::ipacmcfg->isMCC_Mode = true;
				evt_data.event = IPA_WLAN_SWITCH_TO_MCC;
				break;
			}
			continue;

		case WAN_XLAT_CONNECT:
			memcpy(&event_wan, buffer + sizeof(struct ipa_msg_meta),
				sizeof(struct ipa_wan_msg));
			IPACMDBG_H("Received WAN_XLAT_CONNECT name: %s\n",
					event_wan.upstream_ifname);

			/* post IPA_LINK_UP_EVENT event
			 * may be WAN interface is not up
			*/
			data_fid = (ipacm_event_data_fid *)calloc(1, sizeof(ipacm_event_data_fid));
			if(data_fid == NULL)
			{
				IPACMERR("unable to allocate memory for xlat event\n");
				return NULL;
			}
			ipa_get_if_index(event_wan.upstream_ifname, &(data_fid->if_index));
			evt_data.event = IPA_LINK_UP_EVENT;
			evt_data.evt_data = data_fid;
			IPACMDBG_H("Posting IPA_LINK_UP_EVENT event:%d\n", evt_data.event);
			IPACM_EvtDispatcher::PostEvt(&evt_data);

			/* post IPA_WAN_XLAT_CONNECT_EVENT event */
			memset(&evt_data, 0, sizeof(evt_data));
			data_fid = (ipacm_event_data_fid *)calloc(1, sizeof(ipacm_event_data_fid));
			if(data_fid == NULL)
			{
				IPACMERR("unable to allocate memory for xlat event\n");
				return NULL;
			}
			ipa_get_if_index(event_wan.upstream_ifname, &(data_fid->if_index));
			evt_data.event = IPA_WAN_XLAT_CONNECT_EVENT;
			evt_data.evt_data = data_fid;
			IPACMDBG_H("Posting IPA_WAN_XLAT_CONNECT_EVENT event:%d\n", evt_data.event);
			break;

		case IPA_TETHERING_STATS_UPDATE_STATS:
			memcpy(&event_data_stats, buffer + sizeof(struct ipa_msg_meta), sizeof(struct ipa_get_data_stats_resp_msg_v01));
			data_tethering_stats = (ipa_get_data_stats_resp_msg_v01 *)malloc(sizeof(struct ipa_get_data_stats_resp_msg_v01));
			if(data_tethering_stats == NULL)
			{
				IPACMERR("unable to allocate memory for event data_tethering_stats\n");
				return NULL;
			}
			memcpy(data_tethering_stats,
					 &event_data_stats,
						 sizeof(struct ipa_get_data_stats_resp_msg_v01));
			IPACMDBG("Received IPA_TETHERING_STATS_UPDATE_STATS ipa_stats_type: %d\n",data_tethering_stats->ipa_stats_type);
			IPACMDBG("Received %d UL, %d DL pipe stats\n",data_tethering_stats->ul_src_pipe_stats_list_len, data_tethering_stats->dl_dst_pipe_stats_list_len);
			evt_data.event = IPA_TETHERING_STATS_UPDATE_EVENT;
			evt_data.evt_data = data_tethering_stats;
			break;

		case IPA_TETHERING_STATS_UPDATE_NETWORK_STATS:
			memcpy(&event_network_stats, buffer + sizeof(struct ipa_msg_meta), sizeof(struct ipa_get_apn_data_stats_resp_msg_v01));
			data_network_stats = (ipa_get_apn_data_stats_resp_msg_v01 *)malloc(sizeof(ipa_get_apn_data_stats_resp_msg_v01));
			if(data_network_stats == NULL)
			{
				IPACMERR("unable to allocate memory for event data_network_stats\n");
				return NULL;
			}
			memcpy(data_network_stats,
					 &event_network_stats,
						 sizeof(struct ipa_get_apn_data_stats_resp_msg_v01));
			IPACMDBG("Received %d apn network stats \n", data_network_stats->apn_data_stats_list_len);
			evt_data.event = IPA_NETWORK_STATS_UPDATE_EVENT;
			evt_data.evt_data = data_network_stats;
			break;

		default:
			IPACMDBG_H("Unhandled message type: %d\n", event_hdr.msg_type);
			continue;

		}
		/* finish command queue */
		IPACMDBG_H("Posting event:%d\n", evt_data.event);
		IPACM_EvtDispatcher::PostEvt(&evt_data);
		/* push new_neighbor with netdev device internally */
		if(new_neigh_data != NULL)
		{
			IPACMDBG_H("Internally post event IPA_NEW_NEIGH_EVENT\n");
			IPACM_EvtDispatcher::PostEvt(&new_neigh_evt);
		}
	}

	(void)close(fd);
	return NULL;
}

void IPACM_Sig_Handler(int sig)
{
	int cnt;
	ipacm_cmd_q_data evt_data;

	printf("Received Signal: %d\n", sig);
	memset(&evt_data, 0, sizeof(evt_data));

	switch(sig)
	{
		case SIGUSR1:
			IPACMDBG_H("Received SW_ROUTING_ENABLE request \n");
			evt_data.event = IPA_SW_ROUTING_ENABLE;
			IPACM_Iface::ipacmcfg->ipa_sw_rt_enable = true;
			break;

		case SIGUSR2:
			IPACMDBG_H("Received SW_ROUTING_DISABLE request \n");
			evt_data.event = IPA_SW_ROUTING_DISABLE;
			IPACM_Iface::ipacmcfg->ipa_sw_rt_enable = false;
			break;
	}
	/* finish command queue */
	IPACMDBG_H("Posting event:%d\n", evt_data.event);
	IPACM_EvtDispatcher::PostEvt(&evt_data);
	return;
}

void RegisterForSignals(void)
{

	signal(SIGUSR1, IPACM_Sig_Handler);
	signal(SIGUSR2, IPACM_Sig_Handler);
}


int main(int argc, char **argv)
{
	int ret;
	pthread_t netlink_thread = 0, monitor_thread = 0, ipa_driver_thread = 0;
	pthread_t cmd_queue_thread = 0;

	/* check if ipacm is already running or not */
	ipa_is_ipacm_running();

	IPACMDBG_H("In main()\n");
	IPACM_Neighbor *neigh = new IPACM_Neighbor();
	IPACM_IfaceManager *ifacemgr = new IPACM_IfaceManager();

#ifdef FEATURE_ETH_BRIDGE_LE
	IPACM_LanToLan* lan2lan = new IPACM_LanToLan();
#endif

	IPACM_ConntrackClient *cc = IPACM_ConntrackClient::GetInstance();
	CtList = new IPACM_ConntrackListener();

	IPACMDBG_H("Staring IPA main\n");
	IPACMDBG_H("ipa_cmdq_successful\n");


	RegisterForSignals();

	if (IPACM_SUCCESS == cmd_queue_thread)
	{
		ret = pthread_create(&cmd_queue_thread, NULL, MessageQueue::Process, NULL);
		if (IPACM_SUCCESS != ret)
		{
			IPACMERR("unable to command queue thread\n");
			return ret;
		}
		IPACMDBG_H("created command queue thread\n");
		if(pthread_setname_np(cmd_queue_thread, "cmd queue process") != 0)
		{
			IPACMERR("unable to set thread name\n");
		}
	}

	if (IPACM_SUCCESS == netlink_thread)
	{
		ret = pthread_create(&netlink_thread, NULL, netlink_start, NULL);
		if (IPACM_SUCCESS != ret)
		{
			IPACMERR("unable to create netlink thread\n");
			return ret;
		}
		IPACMDBG_H("created netlink thread\n");
		if(pthread_setname_np(netlink_thread, "netlink socket") != 0)
		{
			IPACMERR("unable to set thread name\n");
		}
	}

	/* Enable Firewall support only on MDM targets */
#ifndef FEATURE_IPA_ANDROID
	if (IPACM_SUCCESS == monitor_thread)
	{
		ret = pthread_create(&monitor_thread, NULL, firewall_monitor, NULL);
		if (IPACM_SUCCESS != ret)
		{
			IPACMERR("unable to create monitor thread\n");
			return ret;
		}
		IPACMDBG_H("created firewall monitor thread\n");
		if(pthread_setname_np(monitor_thread, "firewall cfg process") != 0)
		{
			IPACMERR("unable to set thread name\n");
		}
	}
#endif

	if (IPACM_SUCCESS == ipa_driver_thread)
	{
		ret = pthread_create(&ipa_driver_thread, NULL, ipa_driver_msg_notifier, NULL);
		if (IPACM_SUCCESS != ret)
		{
			IPACMERR("unable to create ipa_driver_wlan thread\n");
			return ret;
		}
		IPACMDBG_H("created ipa_driver_wlan thread\n");
		if(pthread_setname_np(ipa_driver_thread, "ipa driver ntfy") != 0)
		{
			IPACMERR("unable to set thread name\n");
		}
	}

	pthread_join(cmd_queue_thread, NULL);
	pthread_join(netlink_thread, NULL);
	pthread_join(monitor_thread, NULL);
	pthread_join(ipa_driver_thread, NULL);
	return IPACM_SUCCESS;
}

/*===========================================================================
		FUNCTION  ipa_is_ipacm_running
===========================================================================*/
/*!
@brief
  Determine whether there's already an IPACM process running, if so, terminate
  the current one

@return
	None

@note

- Dependencies
		- None

- Side Effects
		- None
*/
/*=========================================================================*/

void ipa_is_ipacm_running(void) {

	int fd;
	struct flock lock;
	int retval;

	fd = open(IPACM_PID_FILE, O_RDWR | O_CREAT, 0600);
	if ( fd <= 0 )
	{
		IPACMERR("Failed to open %s, error is %d - %s\n",
				 IPACM_PID_FILE, errno, strerror(errno));
		exit(0);
	}

	/*
	 * Getting an exclusive Write lock on the file, if it fails,
	 * it means that another instance of IPACM is running and it
	 * got the lock before us.
	 */
	memset(&lock, 0, sizeof(lock));
	lock.l_type = F_WRLCK;
	retval = fcntl(fd, F_SETLK, &lock);

	if (retval != 0)
	{
		retval = fcntl(fd, F_GETLK, &lock);
		if (retval == 0)
		{
			IPACMERR("Unable to get lock on file %s (my PID %d), PID %d already has it\n",
					 IPACM_PID_FILE, getpid(), lock.l_pid);
			close(fd);
			exit(0);
		}
	}
	else
	{
		IPACMERR("PID %d is IPACM main process\n", getpid());
	}

	return;
}

/*===========================================================================
		FUNCTION  ipa_get_if_index
===========================================================================*/
/*!
@brief
  get ipa interface index by given the interface name

@return
	IPACM_SUCCESS or IPA_FALUIRE

@note

- Dependencies
		- None

- Side Effects
		- None
*/
/*=========================================================================*/
int ipa_get_if_index
(
	 char *if_name,
	 int *if_index
	 )
{
	int fd;
	struct ifreq ifr;

	if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
	{
		PERROR("get interface index socket create failed");
		return IPACM_FAILURE;
	}

	memset(&ifr, 0, sizeof(struct ifreq));

	(void)strlcpy(ifr.ifr_name, if_name, sizeof(ifr.ifr_name));

	if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0)
	{
		IPACMERR("call_ioctl_on_dev: ioctl failed: can't find device %s",if_name);
		*if_index = -1;
		close(fd);
		return IPACM_FAILURE;
	}

	*if_index = ifr.ifr_ifindex;
	close(fd);
	return IPACM_SUCCESS;
}
