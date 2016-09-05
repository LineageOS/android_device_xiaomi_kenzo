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

#include <sys/ioctl.h>
#include <net/if.h>

#include "IPACM_ConntrackListener.h"
#include "IPACM_ConntrackClient.h"
#include "IPACM_EvtDispatcher.h"

IPACM_ConntrackListener::IPACM_ConntrackListener()
{
	 IPACMDBG("\n");

	 isNatThreadStart = false;
	 isCTReg = false;
	 WanUp = false;
	 nat_inst = NatApp::GetInstance();

	 NatIfaceCnt = 0;
	 StaClntCnt = 0;
	 pNatIfaces = NULL;
	 pConfig = IPACM_Config::GetInstance();;

	 memset(nat_iface_ipv4_addr, 0, sizeof(nat_iface_ipv4_addr));
	 memset(nonnat_iface_ipv4_addr, 0, sizeof(nonnat_iface_ipv4_addr));
	 memset(sta_clnt_ipv4_addr, 0, sizeof(sta_clnt_ipv4_addr));

	 IPACM_EvtDispatcher::registr(IPA_HANDLE_WAN_UP, this);
	 IPACM_EvtDispatcher::registr(IPA_HANDLE_WAN_DOWN, this);
	 IPACM_EvtDispatcher::registr(IPA_PROCESS_CT_MESSAGE, this);
	 IPACM_EvtDispatcher::registr(IPA_PROCESS_CT_MESSAGE_V6, this);
	 IPACM_EvtDispatcher::registr(IPA_HANDLE_WLAN_UP, this);
	 IPACM_EvtDispatcher::registr(IPA_HANDLE_LAN_UP, this);
	 IPACM_EvtDispatcher::registr(IPA_NEIGH_CLIENT_IP_ADDR_ADD_EVENT, this);
	 IPACM_EvtDispatcher::registr(IPA_NEIGH_CLIENT_IP_ADDR_DEL_EVENT, this);

#ifdef CT_OPT
	 p_lan2lan = IPACM_LanToLan::getLan2LanInstance();
#endif
}

void IPACM_ConntrackListener::event_callback(ipa_cm_event_id evt,
						void *data)
{
	 ipacm_event_iface_up *wan_down = NULL;

	 if(data == NULL)
	 {
		 IPACMERR("Invalid Data\n");
		 return;
	 }

	 switch(evt)
	 {
	 case IPA_PROCESS_CT_MESSAGE:
			IPACMDBG("Received IPA_PROCESS_CT_MESSAGE event\n");
			ProcessCTMessage(data);
			break;

#ifdef CT_OPT
	 case IPA_PROCESS_CT_MESSAGE_V6:
			IPACMDBG("Received IPA_PROCESS_CT_MESSAGE_V6 event\n");
			ProcessCTV6Message(data);
			break;
#endif

	 case IPA_HANDLE_WAN_UP:
			IPACMDBG_H("Received IPA_HANDLE_WAN_UP event\n");
			CreateConnTrackThreads();
			if(!isWanUp())
			{
				TriggerWANUp(data);
			}
			break;

	 case IPA_HANDLE_WAN_DOWN:
			IPACMDBG_H("Received IPA_HANDLE_WAN_DOWN event\n");
			wan_down = (ipacm_event_iface_up *)data;
			if(isWanUp())
			{
				TriggerWANDown(wan_down->ipv4_addr);
			}
			break;

	/* if wlan or lan comes up after wan interface, modify
		 tcp/udp filters to ignore local wlan or lan connections */
	 case IPA_HANDLE_WLAN_UP:
	 case IPA_HANDLE_LAN_UP:
			IPACMDBG_H("Received event: %d with ifname: %s and address: 0x%x\n",
							 evt, ((ipacm_event_iface_up *)data)->ifname,
							 ((ipacm_event_iface_up *)data)->ipv4_addr);
			CreateConnTrackThreads();
			IPACM_ConntrackClient::UpdateUDPFilters(data, false);
			IPACM_ConntrackClient::UpdateTCPFilters(data, false);
			break;

	 case IPA_NEIGH_CLIENT_IP_ADDR_ADD_EVENT:
		 IPACMDBG("Received IPA_NEIGH_CLIENT_IP_ADDR_ADD_EVENT event\n");
		 HandleNonNatIPAddr(data, true);
		 break;

	 case IPA_NEIGH_CLIENT_IP_ADDR_DEL_EVENT:
		 IPACMDBG("Received IPA_NEIGH_CLIENT_IP_ADDR_DEL_EVENT event\n");
		 HandleNonNatIPAddr(data, false);
		 break;

	 default:
			IPACMDBG("Ignore cmd %d\n", evt);
			break;
	 }
}

int IPACM_ConntrackListener::CheckNatIface(
   ipacm_event_data_all *data, bool *NatIface)
{
	int fd = 0, len = 0, cnt, i, j;
	struct ifreq ifr;
	*NatIface = false;

	if (data->ipv4_addr == 0 || data->iptype != IPA_IP_v4)
	{
		IPACMDBG("Ignoring\n");
		return IPACM_FAILURE;
	}

	IPACMDBG("Received interface index %d with ip type: %d", data->if_index, data->iptype);
	iptodot(" and ipv4 address", data->ipv4_addr);

	if (pConfig == NULL)
	{
		pConfig = IPACM_Config::GetInstance();
		if (pConfig == NULL)
		{
			IPACMERR("Unable to get Config instance\n");
			return IPACM_FAILURE;
		}
	}

	cnt = pConfig->GetNatIfacesCnt();
	NatIfaceCnt = cnt;
	IPACMDBG("Total Nat ifaces: %d\n", NatIfaceCnt);
	if (pNatIfaces != NULL)
	{
		free(pNatIfaces);
		pNatIfaces = NULL;
	}

	len = (sizeof(NatIfaces) * NatIfaceCnt);
	pNatIfaces = (NatIfaces *)malloc(len);
	if (pNatIfaces == NULL)
	{
		IPACMERR("Unable to allocate memory for non nat ifaces\n");
		return IPACM_FAILURE;
	}

	memset(pNatIfaces, 0, len);
	if (pConfig->GetNatIfaces(NatIfaceCnt, pNatIfaces) != 0)
	{
		IPACMERR("Unable to retrieve non nat ifaces\n");
		return IPACM_FAILURE;
	}

	/* Search/Configure linux interface-index and map it to IPA interface-index */
	if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
	{
		PERROR("get interface name socket create failed");
		return IPACM_FAILURE;
	}

	memset(&ifr, 0, sizeof(struct ifreq));
	ifr.ifr_ifindex = data->if_index;
	if (ioctl(fd, SIOCGIFNAME, &ifr) < 0)
	{
		PERROR("call_ioctl_on_dev: ioctl failed:");
		close(fd);
		return IPACM_FAILURE;
	}
	close(fd);

	for (i = 0; i < NatIfaceCnt; i++)
	{
		if (strncmp(ifr.ifr_name,
					pNatIfaces[i].iface_name,
					sizeof(pNatIfaces[i].iface_name)) == 0)
		{
			/* copy the ipv4 address to filter out downlink connections
				 ignore downlink after listening connection event from
				 conntrack as it is not destinated to private ip address */
			IPACMDBG("Interface (%s) is nat\n", ifr.ifr_name);
			for (j = 0; j < MAX_IFACE_ADDRESS; j++)
			{
				/* check if duplicate NAT ip */
				if (nat_iface_ipv4_addr[j] == data->ipv4_addr)
				{
					*NatIface = true;
					return IPACM_SUCCESS;
				}

				if (nat_iface_ipv4_addr[j] == 0)
				{
					nat_iface_ipv4_addr[j] = data->ipv4_addr;
					IPACMDBG_H("Nating connections of Interface (%s), entry (%d) ",
						pNatIfaces[i].iface_name, j);
					iptodot("with ipv4 address: ", nat_iface_ipv4_addr[j]);

					*NatIface = true;
					return IPACM_SUCCESS;
				}
			}
		}
	}

	return IPACM_SUCCESS;
}

void IPACM_ConntrackListener::HandleNonNatIPAddr(
   void *inParam, bool AddOp)
{
	ipacm_event_data_all *data = (ipacm_event_data_all *)inParam;
	bool NatIface = false;
	int cnt, ret;

	/* Handle only non nat ifaces, NAT iface should be handle
	   separately to avoid race conditions between route/nat
	   rules add/delete operations */
	if (AddOp)
	{
		ret = CheckNatIface(data, &NatIface);
		if (!NatIface && ret == IPACM_SUCCESS)
		{
			/* Cache the non nat iface ip address */
			for (cnt = 0; cnt < MAX_IFACE_ADDRESS; cnt++)
			{
				if (nonnat_iface_ipv4_addr[cnt] == 0)
				{
					nonnat_iface_ipv4_addr[cnt] = data->ipv4_addr;
					/* Add dummy nat rule for non nat ifaces */
					nat_inst->FlushTempEntries(data->ipv4_addr, true, true);
					return;
				}
			}
		}
	}
	else
	{
		/* for delete operation */
		for (cnt = 0; cnt < MAX_IFACE_ADDRESS; cnt++)
		{
			if (nonnat_iface_ipv4_addr[cnt] == data->ipv4_addr)
			{
				IPACMDBG("Reseting ct filters, entry (%d) ", cnt);
				iptodot("with ipv4 address", nonnat_iface_ipv4_addr[cnt]);
				nonnat_iface_ipv4_addr[cnt] = 0;
				nat_inst->FlushTempEntries(data->ipv4_addr, false);
				nat_inst->DelEntriesOnClntDiscon(data->ipv4_addr);
				return;
			}
		}

	}

	return;
}

void IPACM_ConntrackListener::HandleNeighIpAddrAddEvt(
   ipacm_event_data_all *data)
{
	bool NatIface = false;
	int j, ret;

	ret = CheckNatIface(data, &NatIface);
	if (NatIface && ret == IPACM_SUCCESS)
	{
		for (j = 0; j < MAX_IFACE_ADDRESS; j++)
		{
			/* check if duplicate NAT ip */
			if (nat_iface_ipv4_addr[j] == data->ipv4_addr)
				break;

			/* Cache the new nat iface address */
			if (nat_iface_ipv4_addr[j] == 0)
			{
				nat_iface_ipv4_addr[j] = data->ipv4_addr;
				iptodot("Nating connections of addr: ", nat_iface_ipv4_addr[j]);
				break;
			}
		}

		/* Add the cached temp entries to NAT table */
		if (j != MAX_IFACE_ADDRESS)
		{
			nat_inst->ResetPwrSaveIf(data->ipv4_addr);
			nat_inst->FlushTempEntries(data->ipv4_addr, true);
		}
	}
	return;
}

void IPACM_ConntrackListener::HandleNeighIpAddrDelEvt(
   uint32_t ipv4_addr)
{
	int cnt;

	if(ipv4_addr == 0)
	{
		IPACMDBG("Ignoring\n");
		return;
	}

	iptodot("HandleNeighIpAddrDelEvt(): Received ip addr", ipv4_addr);
	for(cnt = 0; cnt<MAX_IFACE_ADDRESS; cnt++)
	{
		if (nat_iface_ipv4_addr[cnt] == ipv4_addr)
		{
			IPACMDBG("Reseting ct nat iface, entry (%d) ", cnt);
			iptodot("with ipv4 address", nat_iface_ipv4_addr[cnt]);
			nat_iface_ipv4_addr[cnt] = 0;
			nat_inst->FlushTempEntries(ipv4_addr, false);
			nat_inst->DelEntriesOnClntDiscon(ipv4_addr);
		}
	}

	return;
}

void IPACM_ConntrackListener::TriggerWANUp(void *in_param)
{
	 ipacm_event_iface_up *wanup_data = (ipacm_event_iface_up *)in_param;

	 IPACMDBG_H("Recevied below information during wanup,\n");
	 IPACMDBG_H("if_name:%s, ipv4_address:0x%x\n",
						wanup_data->ifname, wanup_data->ipv4_addr);

	 if(wanup_data->ipv4_addr == 0)
	 {
		 IPACMERR("Invalid ipv4 address,ignoring IPA_HANDLE_WAN_UP event\n");
		 return;
	 }

	 WanUp = true;
	 isStaMode = wanup_data->is_sta;
	 IPACMDBG("isStaMode: %d\n", isStaMode);

	 wan_ipaddr = wanup_data->ipv4_addr;
	 memcpy(wan_ifname, wanup_data->ifname, sizeof(wan_ifname));

	 if(nat_inst != NULL)
	 {
		 nat_inst->AddTable(wanup_data->ipv4_addr);
	 }

	 IPACMDBG("creating nat threads\n");
	 CreateNatThreads();
}

int IPACM_ConntrackListener::CreateConnTrackThreads(void)
{
	int ret;
	pthread_t tcp_thread = 0, udp_thread = 0;

	if(isCTReg == false)
	{
		if(!tcp_thread)
		{
			ret = pthread_create(&tcp_thread, NULL, IPACM_ConntrackClient::TCPRegisterWithConnTrack, NULL);
			if(0 != ret)
			{
				IPACMERR("unable to create TCP conntrack event listner thread\n");
				PERROR("unable to create TCP conntrack\n");
				return -1;
			}

			IPACMDBG("created TCP conntrack event listner thread\n");
			if(pthread_setname_np(tcp_thread, "tcp ct listener") != 0)
			{
				IPACMERR("unable to set thread name\n");
			}
		}

		if(!udp_thread)
		{
			ret = pthread_create(&udp_thread, NULL, IPACM_ConntrackClient::UDPRegisterWithConnTrack, NULL);
			if(0 != ret)
			{
				IPACMERR("unable to create UDP conntrack event listner thread\n");
				PERROR("unable to create UDP conntrack\n");
				goto error;
			}

			IPACMDBG("created UDP conntrack event listner thread\n");
			if(pthread_setname_np(udp_thread, "udp ct listener") != 0)
			{
				IPACMERR("unable to set thread name\n");
			}
		}

		isCTReg = true;
	}

	return 0;

error:
	return -1;
}
int IPACM_ConntrackListener::CreateNatThreads(void)
{
	int ret;
	pthread_t udpcto_thread = 0;

	if(isNatThreadStart == false)
	{
		if(!udpcto_thread)
		{
			ret = pthread_create(&udpcto_thread, NULL, IPACM_ConntrackClient::UDPConnTimeoutUpdate, NULL);
			if(0 != ret)
			{
				IPACMERR("unable to create udp conn timeout thread\n");
				PERROR("unable to create udp conn timeout\n");
				goto error;
			}

			IPACMDBG("created upd conn timeout thread\n");
			if(pthread_setname_np(udpcto_thread, "udp conn timeout") != 0)
			{
				IPACMERR("unable to set thread name\n");
			}
		}

		isNatThreadStart = true;
	}
	return 0;

error:
	return -1;
}

void IPACM_ConntrackListener::TriggerWANDown(uint32_t wan_addr)
{
	 IPACMDBG_H("Deleting ipv4 nat table with");
	 IPACMDBG_H(" public ip address(0x%x): %d.%d.%d.%d\n", wan_addr,
		    ((wan_addr>>24) & 0xFF), ((wan_addr>>16) & 0xFF), 
		    ((wan_addr>>8) & 0xFF), (wan_addr & 0xFF));
	 
	 WanUp = false;

	 if(nat_inst != NULL)
	 {
		 nat_inst->DeleteTable(wan_addr);
	 }
}


void ParseCTMessage(struct nf_conntrack *ct)
{
	 uint32_t status, timeout;
	 IPACMDBG("Printing conntrack parameters\n");

	 iptodot("ATTR_IPV4_SRC = ATTR_ORIG_IPV4_SRC:", nfct_get_attr_u32(ct, ATTR_ORIG_IPV4_SRC));
	 iptodot("ATTR_IPV4_DST = ATTR_ORIG_IPV4_DST:", nfct_get_attr_u32(ct, ATTR_ORIG_IPV4_DST));
	 IPACMDBG("ATTR_PORT_SRC = ATTR_ORIG_PORT_SRC: 0x%x\n", nfct_get_attr_u16(ct, ATTR_ORIG_PORT_SRC));
	 IPACMDBG("ATTR_PORT_DST = ATTR_ORIG_PORT_DST: 0x%x\n", nfct_get_attr_u16(ct, ATTR_ORIG_PORT_DST));

	 iptodot("ATTR_REPL_IPV4_SRC:", nfct_get_attr_u32(ct, ATTR_REPL_IPV4_SRC));
	 iptodot("ATTR_REPL_IPV4_DST:", nfct_get_attr_u32(ct, ATTR_REPL_IPV4_DST));
	 IPACMDBG("ATTR_REPL_PORT_SRC: 0x%x\n", nfct_get_attr_u16(ct, ATTR_REPL_PORT_SRC));
	 IPACMDBG("ATTR_REPL_PORT_DST: 0x%x\n", nfct_get_attr_u16(ct, ATTR_REPL_PORT_DST));

	 iptodot("ATTR_SNAT_IPV4:", nfct_get_attr_u32(ct, ATTR_SNAT_IPV4));
	 iptodot("ATTR_DNAT_IPV4:", nfct_get_attr_u32(ct, ATTR_DNAT_IPV4));
	 IPACMDBG("ATTR_SNAT_PORT: 0x%x\n", nfct_get_attr_u16(ct, ATTR_SNAT_PORT));
	 IPACMDBG("ATTR_DNAT_PORT: 0x%x\n", nfct_get_attr_u16(ct, ATTR_DNAT_PORT));

	 IPACMDBG("ATTR_MARK: 0x%x\n", nfct_get_attr_u32(ct, ATTR_MARK));
	 IPACMDBG("ATTR_USE: 0x%x\n", nfct_get_attr_u32(ct, ATTR_USE));
	 IPACMDBG("ATTR_ID: 0x%x\n", nfct_get_attr_u32(ct, ATTR_ID));

	 status = nfct_get_attr_u32(ct, ATTR_STATUS);
	 IPACMDBG("ATTR_STATUS: 0x%x\n", status);

	 timeout = nfct_get_attr_u32(ct, ATTR_TIMEOUT);
	 IPACMDBG("ATTR_TIMEOUT: 0x%x\n", timeout);

	 if(IPS_SRC_NAT & status)
	 {
			IPACMDBG("IPS_SRC_NAT set\n");
	 }

	 if(IPS_DST_NAT & status)
	 {
			IPACMDBG("IPS_DST_NAT set\n");
	 }

	 if(IPS_SRC_NAT_DONE & status)
	 {
			IPACMDBG("IPS_SRC_NAT_DONE set\n");
	 }

	 if(IPS_DST_NAT_DONE & status)
	 {
			IPACMDBG(" IPS_DST_NAT_DONE set\n");
	 }

	 IPACMDBG("\n");
	 return;
}

void ParseCTV6Message(struct nf_conntrack *ct)
{
	 uint32_t status, timeout;
	 struct nfct_attr_grp_ipv6 orig_params;
	 uint8_t l4proto, tcp_flags, tcp_state;

	 IPACMDBG("Printing conntrack parameters\n");

	 nfct_get_attr_grp(ct, ATTR_GRP_ORIG_IPV6, (void *)&orig_params);
	 IPACMDBG("Orig src_v6_addr: 0x%08x%08x%08x%08x\n", orig_params.src[0], orig_params.src[1],
                	orig_params.src[2], orig_params.src[3]);
	IPACMDBG("Orig dst_v6_addr: 0x%08x%08x%08x%08x\n", orig_params.dst[0], orig_params.dst[1],
                	orig_params.dst[2], orig_params.dst[3]);

	 IPACMDBG("ATTR_PORT_SRC = ATTR_ORIG_PORT_SRC: 0x%x\n", nfct_get_attr_u16(ct, ATTR_ORIG_PORT_SRC));
	 IPACMDBG("ATTR_PORT_DST = ATTR_ORIG_PORT_DST: 0x%x\n", nfct_get_attr_u16(ct, ATTR_ORIG_PORT_DST));

	 IPACMDBG("ATTR_MARK: 0x%x\n", nfct_get_attr_u32(ct, ATTR_MARK));
	 IPACMDBG("ATTR_USE: 0x%x\n", nfct_get_attr_u32(ct, ATTR_USE));
	 IPACMDBG("ATTR_ID: 0x%x\n", nfct_get_attr_u32(ct, ATTR_ID));

	 timeout = nfct_get_attr_u32(ct, ATTR_TIMEOUT);
	 IPACMDBG("ATTR_TIMEOUT: 0x%x\n", timeout);

	 status = nfct_get_attr_u32(ct, ATTR_STATUS);
	 IPACMDBG("ATTR_STATUS: 0x%x\n", status);

	 l4proto = nfct_get_attr_u8(ct, ATTR_ORIG_L4PROTO);
	 IPACMDBG("ATTR_ORIG_L4PROTO: 0x%x\n", l4proto);
	 if(l4proto == IPPROTO_TCP)
	 {
		tcp_state = nfct_get_attr_u8(ct, ATTR_TCP_STATE);
		IPACMDBG("ATTR_TCP_STATE: 0x%x\n", tcp_state);

		tcp_flags =  nfct_get_attr_u8(ct, ATTR_TCP_FLAGS_ORIG);
		IPACMDBG("ATTR_TCP_FLAGS_ORIG: 0x%x\n", tcp_flags);
	 }

	 IPACMDBG("\n");
	 return;
}

#ifdef CT_OPT
void IPACM_ConntrackListener::ProcessCTV6Message(void *param)
{
	ipacm_ct_evt_data *evt_data = (ipacm_ct_evt_data *)param;
	u_int8_t l4proto = 0;
	uint32_t status = 0;
	struct nf_conntrack *ct = evt_data->ct;

#ifdef IPACM_DEBUG
	 char buf[1024];

	 /* Process message and generate ioctl call to kernel thread */
	 nfct_snprintf(buf, sizeof(buf), evt_data->ct,
								 evt_data->type, NFCT_O_PLAIN, NFCT_OF_TIME);
	 IPACMDBG("%s\n", buf);
	 IPACMDBG("\n");
	 ParseCTV6Message(ct);
#endif

	if(p_lan2lan == NULL)
	{
		IPACMERR("Lan2Lan Instance is null\n");
		goto IGNORE;
	}

	status = nfct_get_attr_u32(ct, ATTR_STATUS);
	if((IPS_DST_NAT & status) || (IPS_SRC_NAT & status))
	{
		IPACMDBG("Either Destination or Source nat flag Set\n");
		goto IGNORE;
	}

	l4proto = nfct_get_attr_u8(ct, ATTR_ORIG_L4PROTO);
	if(IPPROTO_UDP != l4proto && IPPROTO_TCP != l4proto)
	{
		 IPACMDBG("Received unexpected protocl %d conntrack message\n", l4proto);
		 goto IGNORE;
	}

	IPACMDBG("Neither Destination nor Source nat flag Set\n");
	struct nfct_attr_grp_ipv6 orig_params;
	nfct_get_attr_grp(ct, ATTR_GRP_ORIG_IPV6, (void *)&orig_params);

	ipacm_event_connection lan2lan_conn;
	lan2lan_conn.iptype = IPA_IP_v6;
	memcpy(lan2lan_conn.src_ipv6_addr, orig_params.src,
				 sizeof(lan2lan_conn.src_ipv6_addr));
    IPACMDBG("Before convert, src_v6_addr: 0x%08x%08x%08x%08x\n", lan2lan_conn.src_ipv6_addr[0], lan2lan_conn.src_ipv6_addr[1],
                	lan2lan_conn.src_ipv6_addr[2], lan2lan_conn.src_ipv6_addr[3]);
    for(int cnt=0; cnt<4; cnt++)
	{
	   lan2lan_conn.src_ipv6_addr[cnt] = ntohl(lan2lan_conn.src_ipv6_addr[cnt]);
	}
	IPACMDBG("After convert src_v6_addr: 0x%08x%08x%08x%08x\n", lan2lan_conn.src_ipv6_addr[0], lan2lan_conn.src_ipv6_addr[1],
                	lan2lan_conn.src_ipv6_addr[2], lan2lan_conn.src_ipv6_addr[3]);

	memcpy(lan2lan_conn.dst_ipv6_addr, orig_params.dst,
				 sizeof(lan2lan_conn.dst_ipv6_addr));
	IPACMDBG("Before convert, dst_ipv6_addr: 0x%08x%08x%08x%08x\n", lan2lan_conn.dst_ipv6_addr[0], lan2lan_conn.dst_ipv6_addr[1],
                	lan2lan_conn.dst_ipv6_addr[2], lan2lan_conn.dst_ipv6_addr[3]);
    for(int cnt=0; cnt<4; cnt++)
	{
	   lan2lan_conn.dst_ipv6_addr[cnt] = ntohl(lan2lan_conn.dst_ipv6_addr[cnt]);
	}
	IPACMDBG("After convert, dst_ipv6_addr: 0x%08x%08x%08x%08x\n", lan2lan_conn.dst_ipv6_addr[0], lan2lan_conn.dst_ipv6_addr[1],
                	lan2lan_conn.dst_ipv6_addr[2], lan2lan_conn.dst_ipv6_addr[3]);

	if(((IPPROTO_UDP == l4proto) && (NFCT_T_NEW == evt_data->type)) ||
		 ((IPPROTO_TCP == l4proto) &&
			(nfct_get_attr_u8(ct, ATTR_TCP_STATE) == TCP_CONNTRACK_ESTABLISHED))
		 )
	{
			p_lan2lan->handle_new_connection(&lan2lan_conn);
	}
	else if((IPPROTO_UDP == l4proto && NFCT_T_DESTROY == evt_data->type) ||
					(IPPROTO_TCP == l4proto &&
					 nfct_get_attr_u8(ct, ATTR_TCP_STATE) == TCP_CONNTRACK_FIN_WAIT))
	{
			p_lan2lan->handle_del_connection(&lan2lan_conn);
	}

IGNORE:
	/* Cleanup item that was allocated during the original CT callback */
	nfct_destroy(ct);
	return;
}
#endif

void IPACM_ConntrackListener::ProcessCTMessage(void *param)
{
	 ipacm_ct_evt_data *evt_data = (ipacm_ct_evt_data *)param;
	 u_int8_t l4proto = 0;

#ifdef IPACM_DEBUG
	 char buf[1024];
	 unsigned int out_flags;

	 /* Process message and generate ioctl call to kernel thread */
	 out_flags = (NFCT_OF_SHOW_LAYER3 | NFCT_OF_TIME | NFCT_OF_ID);
	 nfct_snprintf(buf, sizeof(buf), evt_data->ct,
								 evt_data->type, NFCT_O_PLAIN, out_flags);
	 IPACMDBG_H("%s\n", buf);

	 ParseCTMessage(evt_data->ct);
#endif

	 l4proto = nfct_get_attr_u8(evt_data->ct, ATTR_ORIG_L4PROTO);
	 if(IPPROTO_UDP != l4proto && IPPROTO_TCP != l4proto)
	 {
			IPACMDBG("Received unexpected protocl %d conntrack message\n", l4proto);
	 }
	 else
	 {
			ProcessTCPorUDPMsg(evt_data->ct, evt_data->type, l4proto);
	 }

	 /* Cleanup item that was allocated during the original CT callback */
	 nfct_destroy(evt_data->ct);
	 return;
}

bool IPACM_ConntrackListener::AddIface(
   nat_table_entry *rule, bool *isTempEntry)
{
	int cnt;

	*isTempEntry = false;
	/* check whether nat iface or not */
	for (cnt = 0; cnt < MAX_IFACE_ADDRESS; cnt++)
	{
		if (nat_iface_ipv4_addr[cnt] != 0)
		{
			if (rule->private_ip == nat_iface_ipv4_addr[cnt] ||
				rule->target_ip == nat_iface_ipv4_addr[cnt])
			{
				IPACMDBG("matched nat_iface_ipv4_addr entry(%d)\n", cnt);
				iptodot("AddIface(): Nat entry match with ip addr",
						nat_iface_ipv4_addr[cnt]);
				return true;
			}
		}
	}

	/* check whether non nat iface or not, on Nat iface
	   add dummy rule by copying public ip to private ip */
	for (cnt = 0; cnt < MAX_IFACE_ADDRESS; cnt++)
	{
		if (nonnat_iface_ipv4_addr[cnt] != 0)
		{
			if (rule->private_ip == nonnat_iface_ipv4_addr[cnt] ||
				rule->target_ip == nonnat_iface_ipv4_addr[cnt])
			{
				IPACMDBG("matched non_nat_iface_ipv4_addr entry(%d)\n", cnt);
				iptodot("AddIface(): Non Nat entry match with ip addr",
						nat_iface_ipv4_addr[cnt]);

				rule->private_ip = rule->public_ip;
				rule->private_port = rule->public_port;
				return true;
			}
		}
	}

	IPACMDBG_H("Not mtaching with non-nat ifaces\n");
	if(pConfig == NULL)
	{
		pConfig = IPACM_Config::GetInstance();
		if(pConfig == NULL)
		{
			IPACMERR("Unable to get Config instance\n");
			return false;
		}
	}

	if (pConfig->isPrivateSubnet(rule->private_ip) ||
		pConfig->isPrivateSubnet(rule->target_ip))
	{
		IPACMDBG("Matching with Private subnet\n");
		*isTempEntry = true;
		return true;
	}

	return false;
}

void IPACM_ConntrackListener::AddORDeleteNatEntry(const nat_entry_bundle *input)
{
	u_int8_t tcp_state;

	if (nat_inst == NULL)
	{
		IPACMERR("Nat instance is NULL, unable to add or delete\n");
		return;
	}

	IPACMDBG_H("Below Nat Entry will either be added or deleted\n");
	iptodot("AddORDeleteNatEntry(): target ip or dst ip",
			input->rule->target_ip);
	IPACMDBG("target port or dst port: 0x%x Decimal:%d\n",
			 input->rule->target_port, input->rule->target_port);
	iptodot("AddORDeleteNatEntry(): private ip or src ip",
			input->rule->private_ip);
	IPACMDBG("private port or src port: 0x%x, Decimal:%d\n",
			 input->rule->private_port, input->rule->private_port);
	IPACMDBG("public port or reply dst port: 0x%x, Decimal:%d\n",
			 input->rule->public_port, input->rule->public_port);
	IPACMDBG("Protocol: %d, destination nat flag: %d\n",
			 input->rule->protocol, input->rule->dst_nat);

	if (IPPROTO_TCP == input->rule->protocol)
	{
		tcp_state = nfct_get_attr_u8(input->ct, ATTR_TCP_STATE);
		if (TCP_CONNTRACK_ESTABLISHED == tcp_state)
		{
			IPACMDBG("TCP state TCP_CONNTRACK_ESTABLISHED(%d)\n", tcp_state);
			if (!CtList->isWanUp())
			{
				IPACMDBG("Wan is not up, cache connections\n");
				nat_inst->CacheEntry(input->rule);
			}
			else if (input->isTempEntry)
			{
				nat_inst->AddTempEntry(input->rule);
			}
			else
			{
				nat_inst->AddEntry(input->rule);
			}
		}
		else if (TCP_CONNTRACK_FIN_WAIT == tcp_state ||
				   input->type == NFCT_T_DESTROY)
		{
			IPACMDBG("TCP state TCP_CONNTRACK_FIN_WAIT(%d) "
					 "or type NFCT_T_DESTROY(%d)\n", tcp_state, input->type);

			nat_inst->DeleteEntry(input->rule);
			nat_inst->DeleteTempEntry(input->rule);
		}
		else
		{
			IPACMDBG("Ignore tcp state: %d and type: %d\n",
					 tcp_state, input->type);
		}

	}
	else if (IPPROTO_UDP == input->rule->protocol)
	{
		if (NFCT_T_NEW == input->type)
		{
			IPACMDBG("New UDP connection at time %ld\n", time(NULL));
			if (!CtList->isWanUp())
			{
				IPACMDBG("Wan is not up, cache connections\n");
				nat_inst->CacheEntry(input->rule);
			}
			else if (input->isTempEntry)
			{
				nat_inst->AddTempEntry(input->rule);
			}
			else
			{
				nat_inst->AddEntry(input->rule);
			}
		}
		else if (NFCT_T_DESTROY == input->type)
		{
			IPACMDBG("UDP connection close at time %ld\n", time(NULL));
			nat_inst->DeleteEntry(input->rule);
			nat_inst->DeleteTempEntry(input->rule);
		}
	}

	return;
}

void IPACM_ConntrackListener::PopulateTCPorUDPEntry(
	 struct nf_conntrack *ct,
	 uint32_t status,
	 nat_table_entry *rule)
{
	if (IPS_DST_NAT == status)
	{
		IPACMDBG("Destination NAT\n");
		rule->dst_nat = true;

		IPACMDBG("Parse reply tuple\n");
		rule->target_ip = nfct_get_attr_u32(ct, ATTR_ORIG_IPV4_SRC);
		rule->target_ip = ntohl(rule->target_ip);
		iptodot("PopulateTCPorUDPEntry(): target ip", rule->target_ip);

		/* Retriev target/dst port */
		rule->target_port = nfct_get_attr_u16(ct, ATTR_ORIG_PORT_SRC);
		rule->target_port = ntohs(rule->target_port);
		if (0 == rule->target_port)
		{
			IPACMDBG("unable to retrieve target port\n");
		}

		rule->public_port = nfct_get_attr_u16(ct, ATTR_ORIG_PORT_DST);
		rule->public_port = ntohs(rule->public_port);

		/* Retriev src/private ip address */
		rule->private_ip = nfct_get_attr_u32(ct, ATTR_REPL_IPV4_SRC);
		rule->private_ip = ntohl(rule->private_ip);
		iptodot("PopulateTCPorUDPEntry(): private ip", rule->private_ip);
		if (0 == rule->private_ip)
		{
			IPACMDBG("unable to retrieve private ip address\n");
		}

		/* Retriev src/private port */
		rule->private_port = nfct_get_attr_u16(ct, ATTR_REPL_PORT_SRC);
		rule->private_port = ntohs(rule->private_port);
		if (0 == rule->private_port)
		{
			IPACMDBG("unable to retrieve private port\n");
		}
	}
	else if (IPS_SRC_NAT == status)
	{
		IPACMDBG("Source NAT\n");
		rule->dst_nat = false;

		/* Retriev target/dst ip address */
		IPACMDBG("Parse source tuple\n");
		rule->target_ip = nfct_get_attr_u32(ct, ATTR_ORIG_IPV4_DST);
		rule->target_ip = ntohl(rule->target_ip);
		iptodot("PopulateTCPorUDPEntry(): target ip", rule->target_ip);
		if (0 == rule->target_ip)
		{
			IPACMDBG("unable to retrieve target ip address\n");
		}
		/* Retriev target/dst port */
		rule->target_port = nfct_get_attr_u16(ct, ATTR_ORIG_PORT_DST);
		rule->target_port = ntohs(rule->target_port);
		if (0 == rule->target_port)
		{
			IPACMDBG("unable to retrieve target port\n");
		}

		/* Retriev public port */
		rule->public_port = nfct_get_attr_u16(ct, ATTR_REPL_PORT_DST);
		rule->public_port = ntohs(rule->public_port);
		if (0 == rule->public_port)
		{
			IPACMDBG("unable to retrieve public port\n");
		}

		/* Retriev src/private ip address */
		rule->private_ip = nfct_get_attr_u32(ct, ATTR_ORIG_IPV4_SRC);
		rule->private_ip = ntohl(rule->private_ip);
		iptodot("PopulateTCPorUDPEntry(): private ip", rule->private_ip);
		if (0 == rule->private_ip)
		{
			IPACMDBG("unable to retrieve private ip address\n");
		}

		/* Retriev src/private port */
		rule->private_port = nfct_get_attr_u16(ct, ATTR_ORIG_PORT_SRC);
		rule->private_port = ntohs(rule->private_port);
		if (0 == rule->private_port)
		{
			IPACMDBG("unable to retrieve private port\n");
		}
	}

	return;
}

#ifdef CT_OPT
void IPACM_ConntrackListener::HandleLan2Lan(struct nf_conntrack *ct,
	enum nf_conntrack_msg_type type,
	 nat_table_entry *rule)
{
	ipacm_event_connection lan2lan_conn = { 0 };

	if (p_lan2lan == NULL)
	{
		IPACMERR("Lan2Lan Instance is null\n");
		return;
	}

	lan2lan_conn.iptype = IPA_IP_v4;
	lan2lan_conn.src_ipv4_addr = orig_src_ip;
	lan2lan_conn.dst_ipv4_addr = orig_dst_ip;

	if (((IPPROTO_UDP == rule->protocol) && (NFCT_T_NEW == type)) ||
		((IPPROTO_TCP == rule->protocol) && (nfct_get_attr_u8(ct, ATTR_TCP_STATE) == TCP_CONNTRACK_ESTABLISHED)))
	{
		p_lan2lan->handle_new_connection(&lan2lan_conn);
	}
	else if ((IPPROTO_UDP == rule->protocol && NFCT_T_DESTROY == type) ||
			   (IPPROTO_TCP == rule->protocol &&
				nfct_get_attr_u8(ct, ATTR_TCP_STATE) == TCP_CONNTRACK_FIN_WAIT))
	{
		p_lan2lan->handle_del_connection(&lan2lan_conn);
	}
}
#endif

void IPACM_ConntrackListener::CheckSTAClient(
   const nat_table_entry *rule, bool *isTempEntry)
{
	int nCnt;

	/* Check whether target is in STA client list or not
      if not ignore the connection */
	 if(!isStaMode || (StaClntCnt == 0))
	 {
		return;
	 }

	 if((sta_clnt_ipv4_addr[0] & STA_CLNT_SUBNET_MASK) !=
		 (rule->target_ip & STA_CLNT_SUBNET_MASK))
	 {
		IPACMDBG("STA client subnet mask not matching\n");
		return;
	 }

	 IPACMDBG("StaClntCnt %d\n", StaClntCnt);
	 for(nCnt = 0; nCnt < StaClntCnt; nCnt++)
	 {
		IPACMDBG("Comparing trgt_ip 0x%x with sta clnt ip: 0x%x\n",
			 rule->target_ip, sta_clnt_ipv4_addr[nCnt]);
		if(rule->target_ip == sta_clnt_ipv4_addr[nCnt])
		{
			IPACMDBG("Match index %d\n", nCnt);
			return;
		}
	 }

	IPACMDBG_H("Not matching with STA Clnt Ip Addrs 0x%x\n",
		rule->target_ip);
	*isTempEntry = true;
}

/* conntrack send in host order and ipa expects in host order */
void IPACM_ConntrackListener::ProcessTCPorUDPMsg(
	 struct nf_conntrack *ct,
	 enum nf_conntrack_msg_type type,
	 u_int8_t l4proto)
{
	 nat_table_entry rule;
	 uint32_t status = 0;
	 uint32_t orig_src_ip, orig_dst_ip;
	 bool isAdd = false;

	 nat_entry_bundle nat_entry;
	 nat_entry.isTempEntry = false;
	 nat_entry.ct = ct;
	 nat_entry.type = type;

 	 memset(&rule, 0, sizeof(rule));
	 IPACMDBG("Received type:%d with proto:%d\n", type, l4proto);
	 status = nfct_get_attr_u32(ct, ATTR_STATUS);

	 /* Retrieve Protocol */
	 rule.protocol = nfct_get_attr_u8(ct, ATTR_REPL_L4PROTO);

	 if(IPS_DST_NAT & status)
	 {
		 status = IPS_DST_NAT;
	 }
	 else if(IPS_SRC_NAT & status)
	 {
		 status = IPS_SRC_NAT;
	 }
	 else
	 {
		 IPACMDBG("Neither Destination nor Source nat flag Set\n");
		 orig_src_ip = nfct_get_attr_u32(ct, ATTR_ORIG_IPV4_SRC);
		 orig_src_ip = ntohl(orig_src_ip);
		 if(orig_src_ip == 0)
		 {
			 IPACMERR("unable to retrieve orig src ip address\n");
			 return;
		 }

		 orig_dst_ip = nfct_get_attr_u32(ct, ATTR_ORIG_IPV4_DST);
		 orig_dst_ip = ntohl(orig_dst_ip);
		 if(orig_dst_ip == 0)
		 {
			 IPACMERR("unable to retrieve orig dst ip address\n");
			 return;
		 }

		if(orig_src_ip == wan_ipaddr)
		{
			IPACMDBG("orig src ip:0x%x equal to wan ip\n",orig_src_ip);
			status = IPS_SRC_NAT;
		}
		else if(orig_dst_ip == wan_ipaddr)
		{
			IPACMDBG("orig Dst IP:0x%x equal to wan ip\n",orig_dst_ip);
			status = IPS_DST_NAT;
		}
		else
		{
			IPACMDBG_H("Neither orig src ip:0x%x Nor orig Dst IP:0x%x equal to wan ip:0x%x\n",
					   orig_src_ip, orig_dst_ip, wan_ipaddr);

#ifdef CT_OPT
			HandleLan2Lan(ct, type, &rule);
#endif
			return;
		}
	 }

	 if(IPS_DST_NAT == status || IPS_SRC_NAT == status)
	 {
		 PopulateTCPorUDPEntry(ct, status, &rule);
		 rule.public_ip = wan_ipaddr;
	 }
	 else
	 {
		 IPACMDBG("Neither source Nor destination nat\n");
		 goto IGNORE;
	 }

	 if (rule.private_ip != wan_ipaddr)
	 {
		 isAdd = AddIface(&rule, &nat_entry.isTempEntry);
		 if (!isAdd)
		 {
			 goto IGNORE;
		 }
	 }
	 else
	 {
		 if (isStaMode)
		 {
			 IPACMDBG("In STA mode, ignore connections destinated to STA interface\n");
			 goto IGNORE;
		 }

		 IPACMDBG("For embedded connections add dummy nat rule\n");
		 IPACMDBG("Change private port %d to %d\n",
				  rule.private_port, rule.public_port);
		 rule.private_port = rule.public_port;
	 }

	 CheckSTAClient(&rule, &nat_entry.isTempEntry);
	 nat_entry.rule = &rule;
	 AddORDeleteNatEntry(&nat_entry);
	 return;

IGNORE:
	IPACMDBG_H("ignoring below Nat Entry\n");
	iptodot("ProcessTCPorUDPMsg(): target ip or dst ip", rule.target_ip);
	IPACMDBG("target port or dst port: 0x%x Decimal:%d\n", rule.target_port, rule.target_port);
	iptodot("ProcessTCPorUDPMsg(): private ip or src ip", rule.private_ip);
	IPACMDBG("private port or src port: 0x%x, Decimal:%d\n", rule.private_port, rule.private_port);
	IPACMDBG("public port or reply dst port: 0x%x, Decimal:%d\n", rule.public_port, rule.public_port);
	IPACMDBG("Protocol: %d, destination nat flag: %d\n", rule.protocol, rule.dst_nat);
	return;
}

void IPACM_ConntrackListener::HandleSTAClientAddEvt(uint32_t clnt_ip_addr)
{
	 int cnt;
	 IPACMDBG_H("Received STA client 0x%x\n", clnt_ip_addr);

	 if(StaClntCnt >= MAX_STA_CLNT_IFACES)
	 {
		IPACMDBG("Max STA client reached, ignore 0x%x\n", clnt_ip_addr);
		return;
	 }

	 for(cnt=0; cnt<MAX_STA_CLNT_IFACES; cnt++)
	 {
		if(sta_clnt_ipv4_addr[cnt] != 0 &&
		 sta_clnt_ipv4_addr[cnt] == clnt_ip_addr)
		{
			IPACMDBG("Ignoring duplicate one 0x%x\n", clnt_ip_addr);
			break;
		}

		if(sta_clnt_ipv4_addr[cnt] == 0)
		{
			IPACMDBG("Adding STA client 0x%x at Index: %d\n",
					clnt_ip_addr, cnt);
			sta_clnt_ipv4_addr[cnt] = clnt_ip_addr;
			StaClntCnt++;
			IPACMDBG("STA client cnt %d\n", StaClntCnt);
			break;
		}

	 }

	 nat_inst->FlushTempEntries(clnt_ip_addr, true);
	 return;
}

void IPACM_ConntrackListener::HandleSTAClientDelEvt(uint32_t clnt_ip_addr)
{
	 int cnt;
	 IPACMDBG_H("Received STA client 0x%x\n", clnt_ip_addr);

	 for(cnt=0; cnt<MAX_STA_CLNT_IFACES; cnt++)
	 {
		if(sta_clnt_ipv4_addr[cnt] != 0 &&
		 sta_clnt_ipv4_addr[cnt] == clnt_ip_addr)
		{
			IPACMDBG("Deleting STA client 0x%x at index: %d\n",
					clnt_ip_addr, cnt);
			sta_clnt_ipv4_addr[cnt] = 0;
			nat_inst->DelEntriesOnSTAClntDiscon(clnt_ip_addr);
			StaClntCnt--;
			IPACMDBG("STA client cnt %d\n", StaClntCnt);
			break;
		}
	 }

	 nat_inst->FlushTempEntries(clnt_ip_addr, false);
   return;
}
