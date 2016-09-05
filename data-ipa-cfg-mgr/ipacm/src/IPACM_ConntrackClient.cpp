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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include "IPACM_Iface.h"
#include "IPACM_ConntrackListener.h"
#include "IPACM_ConntrackClient.h"
#include "IPACM_Log.h"

#define LO_NAME "lo"

extern IPACM_EvtDispatcher cm_dis;
extern void ParseCTMessage(struct nf_conntrack *ct);

IPACM_ConntrackClient *IPACM_ConntrackClient::pInstance = NULL;
IPACM_ConntrackListener *CtList = NULL;

/* ================================
		 Local Function Definitions
		 =================================
*/
IPACM_ConntrackClient::IPACM_ConntrackClient()
{
	IPACMDBG("\n");

	tcp_hdl = NULL;
	udp_hdl = NULL;
	tcp_filter = NULL;
	udp_filter = NULL;
}

IPACM_ConntrackClient* IPACM_ConntrackClient::GetInstance()
{
	if(pInstance == NULL)
	{
		pInstance = new IPACM_ConntrackClient();

		pInstance->udp_filter = nfct_filter_create();
		if(pInstance->udp_filter == NULL)
		{
			IPACMERR("unable to create UDP filter\n");
			delete pInstance;
			return NULL;
		}
		IPACMDBG("Created UDP filter\n");

		pInstance->tcp_filter = nfct_filter_create();
		if(pInstance->tcp_filter == NULL)
		{
			IPACMERR("unable to create TCP filter\n");
			delete pInstance;
			return NULL;
		}
		IPACMDBG("Created TCP filter\n");
	}

	return pInstance;
}

int IPACM_ConntrackClient::IPAConntrackEventCB
(
	 enum nf_conntrack_msg_type type,
	 struct nf_conntrack *ct,
	 void *data
	 )
{
	ipacm_cmd_q_data evt_data;
	ipacm_ct_evt_data *ct_data;
	uint8_t ip_type = 0;

	IPACMDBG("Event callback called with msgtype: %d\n",type);

	/* Retrieve ip type */
	ip_type = nfct_get_attr_u8(ct, ATTR_REPL_L3PROTO);

#ifndef CT_OPT
	if(AF_INET6 == ip_type)
	{
		IPACMDBG("Ignoring ipv6(%d) connections\n", ip_type);
		goto IGNORE;
	}

#endif

	ct_data = (ipacm_ct_evt_data *)malloc(sizeof(ipacm_ct_evt_data));
	if(ct_data == NULL)
	{
		IPACMERR("unable to allocate memory \n");
		goto IGNORE;
	}

	ct_data->ct = ct;
	ct_data->type = type;

	evt_data.event = IPA_PROCESS_CT_MESSAGE;
	evt_data.evt_data = (void *)ct_data;

#ifdef CT_OPT
	if(AF_INET6 == ip_type)
	{
		evt_data.event = IPA_PROCESS_CT_MESSAGE_V6;
	}
#endif

	if(0 != IPACM_EvtDispatcher::PostEvt(&evt_data))
	{
		IPACMERR("Error sending Conntrack message to processing thread!\n");
		free(ct_data);
		goto IGNORE;
	}

/* NFCT_CB_STOLEN means that the conntrack object is not released after the
	 callback That must be manually done later when the object is no longer needed. */
	return NFCT_CB_STOLEN;

IGNORE:
	nfct_destroy(ct);
	return NFCT_CB_STOLEN;

}

int IPACM_ConntrackClient::IPA_Conntrack_Filters_Ignore_Bridge_Addrs
(
	 struct nfct_filter *filter
)
{
	int fd;
	fd = socket(AF_INET, SOCK_DGRAM, 0);
	if(fd < 0)
	{
		PERROR("unable to open socket");
		return -1;
	}

	int ret;
	uint32_t ipv4_addr;
	struct ifreq ifr;

	/* retrieve bridge interface ipv4 address */
	memset(&ifr, 0, sizeof(struct ifreq));
	ifr.ifr_addr.sa_family = AF_INET;
	(void)strncpy(ifr.ifr_name, IPACM_Iface::ipacmcfg->ipa_virtual_iface_name, sizeof(ifr.ifr_name));
	IPACMDBG("bridge interface name (%s)\n", ifr.ifr_name);

	ret = ioctl(fd, SIOCGIFADDR, &ifr);
	if (ret < 0)
	{
		IPACMERR("unable to retrieve (%s) interface address\n",ifr.ifr_name);
		close(fd);
		return -1;
	}
	IPACMDBG("Interface (%s) address %s\n", ifr.ifr_name, inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr));
	ipv4_addr = ntohl(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr.s_addr);
	close(fd);

	/* ignore whatever is destined to or originates from broadcast ip address */
	struct nfct_filter_ipv4 filter_ipv4;

	filter_ipv4.addr = ipv4_addr;
	filter_ipv4.mask = 0xffffffff;

	nfct_filter_set_logic(filter,
												NFCT_FILTER_DST_IPV4,
												NFCT_FILTER_LOGIC_NEGATIVE);

	nfct_filter_add_attr(filter, NFCT_FILTER_DST_IPV4, &filter_ipv4);

	nfct_filter_set_logic(filter,
												NFCT_FILTER_SRC_IPV4,
												NFCT_FILTER_LOGIC_NEGATIVE);

	nfct_filter_add_attr(filter, NFCT_FILTER_SRC_IPV4, &filter_ipv4);

  return 0;
}

int IPACM_ConntrackClient::IPA_Conntrack_Filters_Ignore_Local_Iface
(
	 struct nfct_filter *filter,
	 ipacm_event_iface_up *param
)
{
	struct nfct_filter_ipv4 filter_ipv4;

	filter_ipv4.addr = param->ipv4_addr;
	filter_ipv4.mask = 0xffffffff;

	/* ignore whatever is destined to local interfaces */
	IPACMDBG("Ignore connections destinated to interface %s", param->ifname);
	iptodot("with ipv4 address", param->ipv4_addr);
	nfct_filter_set_logic(filter,
												NFCT_FILTER_DST_IPV4,
												NFCT_FILTER_LOGIC_NEGATIVE);

	nfct_filter_add_attr(filter, NFCT_FILTER_DST_IPV4, &filter_ipv4);

	IPACMDBG("Ignore connections orignated from interface %s", param->ifname);
	iptodot("with ipv4 address", filter_ipv4.addr);
	nfct_filter_set_logic(filter,
												NFCT_FILTER_SRC_IPV4,
												NFCT_FILTER_LOGIC_NEGATIVE);

	nfct_filter_add_attr(filter, NFCT_FILTER_SRC_IPV4, &filter_ipv4);

	/* Retrieve broadcast address */
	/* Intialize with 255.255.255.255 */
	uint32_t bc_ip_addr = 0xFFFFFFFF;

	/* calculate broadcast address from addr and addr_mask */
	bc_ip_addr = (bc_ip_addr & (~param->addr_mask));
	bc_ip_addr = (bc_ip_addr | (param->ipv4_addr & param->addr_mask));

	/* netfitler expecting in host-byte order */
	filter_ipv4.addr = bc_ip_addr;
	filter_ipv4.mask = 0xffffffff;

	iptodot("with broadcast address", filter_ipv4.addr);
	nfct_filter_set_logic(filter,
												NFCT_FILTER_DST_IPV4,
												NFCT_FILTER_LOGIC_NEGATIVE);

	nfct_filter_add_attr(filter, NFCT_FILTER_DST_IPV4, &filter_ipv4);

	return 0;
}

/* Function which sets up filters to ignore
		 connections to and from local interfaces */
int IPACM_ConntrackClient::IPA_Conntrack_Filters_Ignore_Local_Addrs
(
	 struct nfct_filter *filter
)
{
	struct nfct_filter_ipv4 filter_ipv4;

	/* ignore whatever is destined to or originates from broadcast ip address */
	filter_ipv4.addr = 0xffffffff;
	filter_ipv4.mask = 0xffffffff;

	nfct_filter_set_logic(filter,
												NFCT_FILTER_DST_IPV4,
												NFCT_FILTER_LOGIC_NEGATIVE);

	nfct_filter_add_attr(filter, NFCT_FILTER_DST_IPV4, &filter_ipv4);

	nfct_filter_set_logic(filter,
												NFCT_FILTER_SRC_IPV4,
												NFCT_FILTER_LOGIC_NEGATIVE);

	nfct_filter_add_attr(filter, NFCT_FILTER_SRC_IPV4, &filter_ipv4);

	return 0;
} /* IPA_Conntrack_Filters_Ignore_Local_Addrs() */

/* Initialize TCP Filter */
int IPACM_ConntrackClient::IPA_Conntrack_TCP_Filter_Init(void)
{
	int ret = 0;
	IPACM_ConntrackClient *pClient;

	IPACMDBG("\n");

	pClient = IPACM_ConntrackClient::GetInstance();
	if(pClient == NULL)
	{
		IPACMERR("unable to get conntrack client instance\n");
		return -1;
	}

	ret = nfct_filter_set_logic(pClient->tcp_filter,
															NFCT_FILTER_L4PROTO,
															NFCT_FILTER_LOGIC_POSITIVE);
	if(ret == -1)
	{
		IPACMERR("Unable to set filter logic\n");
		return -1;
	}

	/* set protocol filters as tcp and udp */
	nfct_filter_add_attr_u32(pClient->tcp_filter, NFCT_FILTER_L4PROTO, IPPROTO_TCP);


	struct nfct_filter_proto tcp_proto_state;
	tcp_proto_state.proto = IPPROTO_TCP;
	tcp_proto_state.state = TCP_CONNTRACK_ESTABLISHED;

	ret = nfct_filter_set_logic(pClient->tcp_filter,
															NFCT_FILTER_L4PROTO_STATE,
															NFCT_FILTER_LOGIC_POSITIVE);
	if(ret == -1)
	{
		IPACMERR("unable to set filter logic\n");
		return -1;
	}
	nfct_filter_add_attr(pClient->tcp_filter,
											 NFCT_FILTER_L4PROTO_STATE,
											 &tcp_proto_state);


	tcp_proto_state.proto = IPPROTO_TCP;
	tcp_proto_state.state = TCP_CONNTRACK_FIN_WAIT;
	ret = nfct_filter_set_logic(pClient->tcp_filter,
															NFCT_FILTER_L4PROTO_STATE,
															NFCT_FILTER_LOGIC_POSITIVE);
	if(ret == -1)
	{
		IPACMERR("unable to set filter logic\n");
		return -1;
	}

	nfct_filter_add_attr(pClient->tcp_filter,
											 NFCT_FILTER_L4PROTO_STATE,
											 &tcp_proto_state);
	return 0;
}


/* Initialize UDP Filter */
int IPACM_ConntrackClient::IPA_Conntrack_UDP_Filter_Init(void)
{
	int ret = 0;
	IPACM_ConntrackClient *pClient = IPACM_ConntrackClient::GetInstance();
	if(pClient == NULL)
	{
		IPACMERR("unable to get conntrack client instance\n");
		return -1;
	}

	ret = nfct_filter_set_logic(pClient->udp_filter,
															NFCT_FILTER_L4PROTO,
															NFCT_FILTER_LOGIC_POSITIVE);
	if(ret == -1)
	{
		IPACMERR("unable to set filter logic\n");
	}
	/* set protocol filters as tcp and udp */
	nfct_filter_add_attr_u32(pClient->udp_filter, NFCT_FILTER_L4PROTO, IPPROTO_UDP);

	return 0;
}

void* IPACM_ConntrackClient::UDPConnTimeoutUpdate(void *ptr)
{

	NatApp *nat_inst = NULL;
#ifdef IPACM_DEBUG
	IPACMDBG("\n");
#endif

	nat_inst = NatApp::GetInstance();
	if(nat_inst == NULL)
	{
		IPACMERR("unable to create nat instance\n");
		return NULL;
	}

	while(1)
	{
		nat_inst->UpdateUDPTimeStamp();
		sleep(UDP_TIMEOUT_UPDATE);
	} /* end of while(1) loop */

#ifdef IPACM_DEBUG
	IPACMDBG("Returning from %s() %d\n", __FUNCTION__, __LINE__);
#endif

	return NULL;
}

/* Thread to initialize TCP Conntrack Filters*/
void* IPACM_ConntrackClient::TCPRegisterWithConnTrack(void *)
{
	int ret;
	IPACM_ConntrackClient *pClient;
	unsigned subscrips = 0;

	IPACMDBG("\n");

	pClient = IPACM_ConntrackClient::GetInstance();
	if(pClient == NULL)
	{
		IPACMERR("unable to get conntrack client instance\n");
		return NULL;
	}

	subscrips = (NF_NETLINK_CONNTRACK_UPDATE | NF_NETLINK_CONNTRACK_DESTROY);
#ifdef CT_OPT
	subscrips |= NF_NETLINK_CONNTRACK_NEW;
#endif

	pClient->tcp_hdl = nfct_open(CONNTRACK, subscrips);
	if(pClient->tcp_hdl == NULL)
	{
		PERROR("nfct_open\n");
		return NULL;
	}

	/* Initialize the filter */
	ret = IPA_Conntrack_TCP_Filter_Init();
	if(ret == -1)
	{
		IPACMERR("Unable to initliaze TCP Filter\n");
		return NULL;
	}

	/* Attach the filter to net filter handler */
	ret = nfct_filter_attach(nfct_fd(pClient->tcp_hdl), pClient->tcp_filter);
	if(ret == -1)
	{
		IPACMDBG("unable to attach TCP filter\n");
		return NULL;
	}

	/* Register callback with netfilter handler */
	IPACMDBG_H("tcp handle:%p, fd:%d\n", pClient->tcp_hdl, nfct_fd(pClient->tcp_hdl));
#ifndef CT_OPT
	nfct_callback_register(pClient->tcp_hdl,
			(nf_conntrack_msg_type)	(NFCT_T_UPDATE | NFCT_T_DESTROY | NFCT_T_NEW),
						IPAConntrackEventCB, NULL);
#else
	nfct_callback_register(pClient->tcp_hdl, (nf_conntrack_msg_type) NFCT_T_ALL, IPAConntrackEventCB, NULL);
#endif

	/* Block to catch events from net filter connection track */
	/* nfct_catch() receives conntrack events from kernel-space, by default it
			 blocks waiting for events. */
	IPACMDBG("Waiting for events\n");

	ret = nfct_catch(pClient->tcp_hdl);
	if(ret == -1)
	{
		IPACMERR("(%d)(%s)\n", ret, strerror(errno));
		return NULL;
	}

	IPACMDBG("Exit from tcp thread\n");

	/* destroy the filter.. this will not detach the filter */
	nfct_filter_destroy(pClient->tcp_filter);
	pClient->tcp_filter = NULL;

	/* de-register the callback */
	nfct_callback_unregister(pClient->tcp_hdl);
	/* close the handle */
	nfct_close(pClient->tcp_hdl);
	pClient->tcp_hdl = NULL;

	pthread_exit(NULL);
	return NULL;
}

/* Thread to initialize UDP Conntrack Filters*/
void* IPACM_ConntrackClient::UDPRegisterWithConnTrack(void *)
{
	int ret;
	IPACM_ConntrackClient *pClient = NULL;

	IPACMDBG("\n");

	pClient = IPACM_ConntrackClient::GetInstance();
	if(pClient == NULL)
	{
		IPACMERR("unable to retrieve instance of conntrack client\n");
		return NULL;
	}

	pClient->udp_hdl = nfct_open(CONNTRACK,
					(NF_NETLINK_CONNTRACK_NEW | NF_NETLINK_CONNTRACK_DESTROY));
	if(pClient->udp_hdl == NULL)
	{
		PERROR("nfct_open\n");
		return NULL;
	}

	/* Initialize Filter */
	ret = IPA_Conntrack_UDP_Filter_Init();
	if(-1 == ret)
	{
		IPACMDBG("Unable to initalize udp filters\n");
		return NULL;
	}

	/* Attach the filter to net filter handler */
	ret = nfct_filter_attach(nfct_fd(pClient->udp_hdl), pClient->udp_filter);
	if(ret == -1)
	{
		IPACMDBG("unable to attach the filter\n");
		return NULL;
	}

	/* Register callback with netfilter handler */
	IPACMDBG_H("udp handle:%p, fd:%d\n", pClient->udp_hdl, nfct_fd(pClient->udp_hdl));
	nfct_callback_register(pClient->udp_hdl,
			(nf_conntrack_msg_type)(NFCT_T_NEW | NFCT_T_DESTROY),
			IPAConntrackEventCB,
			NULL);

	/* Block to catch events from net filter connection track */
ctcatch:
	ret = nfct_catch(pClient->udp_hdl);
	if(ret == -1)
	{
		IPACMDBG("(%d)(%s)\n", ret, strerror(errno));
		return NULL;
	}
	else
	{
		IPACMDBG("ctcatch ret:%d\n", ret);
		goto ctcatch;
	}

	IPACMDBG("Exit from udp thread with ret: %d\n", ret);

	/* destroy the filter.. this will not detach the filter */
	nfct_filter_destroy(pClient->udp_filter);
	pClient->udp_filter = NULL;

	/* de-register the callback */
	nfct_callback_unregister(pClient->udp_hdl);
	/* close the handle */
	nfct_close(pClient->udp_hdl);
	pClient->udp_hdl = NULL;

	pthread_exit(NULL);
	return NULL;
}

void IPACM_ConntrackClient::UpdateUDPFilters(void *param, bool isWan)
{
	static bool isIgnore = false;
	int ret = 0;
	IPACM_ConntrackClient *pClient = NULL;

	pClient = IPACM_ConntrackClient::GetInstance();
	if(pClient == NULL)
	{
		IPACMERR("unable to retrieve conntrack client instance\n");
		return;
	}

	if(pClient->udp_filter == NULL)
	{
		 return;
	}

	if(!isWan)
	{
		IPA_Conntrack_Filters_Ignore_Local_Iface(pClient->udp_filter,
																		 (ipacm_event_iface_up *)param);

		if(!isIgnore)
		{
			IPA_Conntrack_Filters_Ignore_Bridge_Addrs(pClient->udp_filter);
			IPA_Conntrack_Filters_Ignore_Local_Addrs(pClient->udp_filter);
			isIgnore = true;
		}
	}

	/* Attach the filter to udp handle */
	if(pClient->udp_hdl != NULL)
	{
		IPACMDBG("attaching the filter to udp handle\n");
		ret = nfct_filter_attach(nfct_fd(pClient->udp_hdl), pClient->udp_filter);
		if(ret == -1)
		{
			PERROR("unable to attach the filter to udp handle\n");
			IPACMERR("udp handle:%p, fd:%d Error: %d\n",pClient->udp_hdl, nfct_fd(pClient->udp_hdl), ret);
			return;
		}
	}

	return;
}

void IPACM_ConntrackClient::UpdateTCPFilters(void *param, bool isWan)
{
	static bool isIgnore = false;
	int ret = 0;
	IPACM_ConntrackClient *pClient = NULL;

	pClient = IPACM_ConntrackClient::GetInstance();
	if(pClient == NULL)
	{
		IPACMERR("unable to retrieve conntrack client instance\n");
		return;
	}

	if(pClient->tcp_filter == NULL)
		return;

	if(!isWan)
	{
		IPA_Conntrack_Filters_Ignore_Local_Iface(pClient->tcp_filter,
																	(ipacm_event_iface_up *)param);

		if(!isIgnore)
		{
			IPA_Conntrack_Filters_Ignore_Bridge_Addrs(pClient->udp_filter);
			IPA_Conntrack_Filters_Ignore_Local_Addrs(pClient->udp_filter);
			isIgnore = true;
		}
	}

	/* Attach the filter to tcp handle */
	if(pClient->tcp_hdl != NULL)
	{
		IPACMDBG("attaching the filter to tcp handle\n");
		ret = nfct_filter_attach(nfct_fd(pClient->tcp_hdl), pClient->tcp_filter);
		if(ret == -1)
		{
			PERROR("unable to attach the filter to tcp handle\n");
			IPACMERR("tcp handle:%p, fd:%d Error: %d\n",pClient->tcp_hdl, nfct_fd(pClient->tcp_hdl), ret);
			return;
		}
	}

  return;
}

