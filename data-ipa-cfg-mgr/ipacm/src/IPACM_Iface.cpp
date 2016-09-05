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
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.Z
*/
/*!
  @file
  IPACM_Iface.cpp

  @brief
  This file implements the basis Iface functionality.

  @Author
  Skylar Chang

*/
#include <fcntl.h>
#include <sys/ioctl.h>
#include <IPACM_Netlink.h>
#include <IPACM_Iface.h>
#include <IPACM_Lan.h>
#include <IPACM_Wan.h>
#include <IPACM_Wlan.h>
#include <string.h>

extern "C"
{
#include <ifaddrs.h>
}


const char *IPACM_Iface::DEVICE_NAME = "/dev/ipa";
IPACM_Routing IPACM_Iface::m_routing;
IPACM_Filtering IPACM_Iface::m_filtering;
IPACM_Header IPACM_Iface::m_header;

IPACM_Config *IPACM_Iface::ipacmcfg = IPACM_Config::GetInstance();

IPACM_Iface::IPACM_Iface(int iface_index)
{
	ip_type = IPACM_IP_NULL; /* initially set invalid */
	num_dft_rt_v6 = 0;
	softwarerouting_act = false;
	ipa_if_num = iface_index;
	ipa_if_cate = IPACM_Iface::ipacmcfg->iface_table[iface_index].if_cat;

	iface_query = NULL;
	tx_prop = NULL;
	rx_prop = NULL;

	memcpy(dev_name,
				 IPACM_Iface::ipacmcfg->iface_table[iface_index].iface_name,
				 sizeof(IPACM_Iface::ipacmcfg->iface_table[iface_index].iface_name));

	memset(dft_v4fl_rule_hdl, 0, sizeof(dft_v4fl_rule_hdl));
	memset(dft_v6fl_rule_hdl, 0, sizeof(dft_v6fl_rule_hdl));

	memset(dft_rt_rule_hdl, 0, sizeof(dft_rt_rule_hdl));
	memset(software_routing_fl_rule_hdl, 0, sizeof(software_routing_fl_rule_hdl));
	memset(ipv6_addr, 0, sizeof(ipv6_addr));

	query_iface_property();
	IPACMDBG_H(" create iface-index(%d) constructor\n", ipa_if_num);
	return;
}

/* software routing enable */
int IPACM_Iface::handle_software_routing_enable(void)
{

	int res = IPACM_SUCCESS;
	struct ipa_flt_rule_add flt_rule_entry;
	ipa_ioc_add_flt_rule *m_pFilteringTable;

	IPACMDBG("\n");
	if (softwarerouting_act == true)
	{
		IPACMDBG("already setup software_routing rule for (%s)iface ip-family %d\n",
						     IPACM_Iface::ipacmcfg->iface_table[ipa_if_num].iface_name, ip_type);
		return IPACM_SUCCESS;
	}

	if(rx_prop == NULL)
	{
		IPACMDBG("No rx properties registered for iface %s\n", dev_name);
		return IPACM_SUCCESS;
	}

	m_pFilteringTable = (struct ipa_ioc_add_flt_rule *)
		 calloc(1,
						sizeof(struct ipa_ioc_add_flt_rule) +
						1 * sizeof(struct ipa_flt_rule_add)
						);
	if (!m_pFilteringTable)
	{
		IPACMERR("Error Locate ipa_flt_rule_add memory...\n");
		return IPACM_FAILURE;
	}

	m_pFilteringTable->commit = 1;
	m_pFilteringTable->ep = rx_prop->rx[0].src_pipe;
	m_pFilteringTable->global = false;
	m_pFilteringTable->num_rules = (uint8_t)1;


	/* Configuring Software-Routing Filtering Rule */
	memset(&flt_rule_entry, 0, sizeof(struct ipa_flt_rule_add));

	flt_rule_entry.at_rear = false;
	flt_rule_entry.flt_rule_hdl = -1;
	flt_rule_entry.status = -1;
	flt_rule_entry.rule.action = IPA_PASS_TO_EXCEPTION;
#ifdef FEATURE_IPA_V3
	flt_rule_entry.rule.hashable = true;
#endif
	memcpy(&flt_rule_entry.rule.attrib,
				 &rx_prop->rx[0].attrib,
				 sizeof(flt_rule_entry.rule.attrib));

	memcpy(&(m_pFilteringTable->rules[0]), &flt_rule_entry, sizeof(struct ipa_flt_rule_add));

	/* check iface is v4 or v6 or both*/
//	if (ip_type == IPA_IP_MAX)
//	{
		/* handle v4 */
		m_pFilteringTable->ip = IPA_IP_v4;
		if (false == m_filtering.AddFilteringRule(m_pFilteringTable))
		{
			IPACMERR("Error Adding Filtering rule, aborting...\n");
			res = IPACM_FAILURE;
			goto fail;
		}
		else if (m_pFilteringTable->rules[0].status)
		{
			IPACMERR("adding flt rule failed status=0x%x\n", m_pFilteringTable->rules[0].status);
			res = IPACM_FAILURE;
			goto fail;
		}

		IPACM_Iface::ipacmcfg->increaseFltRuleCount(rx_prop->rx[0].src_pipe, IPA_IP_v4, 1);
		IPACMDBG("soft-routing flt rule hdl0=0x%x\n", m_pFilteringTable->rules[0].flt_rule_hdl);
		/* copy filter hdls */
		software_routing_fl_rule_hdl[0] = m_pFilteringTable->rules[0].flt_rule_hdl;


		/* handle v6*/
		m_pFilteringTable->ip = IPA_IP_v6;
		if (false == m_filtering.AddFilteringRule(m_pFilteringTable))
		{
			IPACMERR("Error Adding Filtering rule, aborting...\n");
			res = IPACM_FAILURE;
			goto fail;
		}
		else if (m_pFilteringTable->rules[0].status)
		{
			IPACMDBG("adding flt rule failed status=0x%x\n", m_pFilteringTable->rules[0].status);
			res = IPACM_FAILURE;
			goto fail;
		}

		IPACM_Iface::ipacmcfg->increaseFltRuleCount(rx_prop->rx[0].src_pipe, IPA_IP_v6, 1);
		IPACMDBG("soft-routing flt rule hdl0=0x%x\n", m_pFilteringTable->rules[0].flt_rule_hdl);
		/* copy filter hdls */
		software_routing_fl_rule_hdl[1] = m_pFilteringTable->rules[0].flt_rule_hdl;
		softwarerouting_act = true;
#if 0
	}
	else
	{
		if (ip_type == IPA_IP_v4)
		{
			m_pFilteringTable->ip = IPA_IP_v4;
		}
		else
		{
			m_pFilteringTable->ip = IPA_IP_v6;
		}

		if (false == m_filtering.AddFilteringRule(m_pFilteringTable))
		{
			IPACMERR("Error Adding Filtering rule, aborting...\n");
			res = IPACM_FAILURE;
			goto fail;
		}
		else if (m_pFilteringTable->rules[0].status)
		{
			IPACMERR("adding flt rule failed status=0x%x\n", m_pFilteringTable->rules[0].status);
			res = IPACM_FAILURE;
			goto fail;
		}

		IPACM_Iface::ipacmcfg->increaseFltRuleCount(rx_prop->rx[0].src_pipe, ip_type, 1);
		IPACMDBG("soft-routing flt rule hdl0=0x%x\n", m_pFilteringTable->rules[0].flt_rule_hdl);
		/* copy filter hdls */
		if (ip_type == IPA_IP_v4)
		{
			software_routing_fl_rule_hdl[0] = m_pFilteringTable->rules[0].flt_rule_hdl;
		}
		else
		{
			software_routing_fl_rule_hdl[1] = m_pFilteringTable->rules[0].flt_rule_hdl;
		}
		softwarerouting_act = true;
	}
#endif

fail:
	free(m_pFilteringTable);

	return res;
}

/* software routing disable */
int IPACM_Iface::handle_software_routing_disable(void)
{
	int res = IPACM_SUCCESS;
	ipa_ip_type ip;
	uint32_t flt_hdl;

	if (rx_prop == NULL)
	{
		IPACMDBG("No rx properties registered for iface %s\n", dev_name);
		return IPACM_SUCCESS;
	}

	if (softwarerouting_act == false)
	{
		IPACMDBG("already delete software_routing rule for (%s)iface ip-family %d\n", IPACM_Iface::ipacmcfg->iface_table[ipa_if_num].iface_name, ip_type);
		return IPACM_SUCCESS;
	}

//	if (ip_type == IPA_IP_MAX)
//	{
		/* ipv4 case */
		if (m_filtering.DeleteFilteringHdls(&software_routing_fl_rule_hdl[0],
																				IPA_IP_v4, 1) == false)
		{
			IPACMERR("Error Adding Filtering rule, aborting...\n");
			res = IPACM_FAILURE;
			goto fail;
		}
		IPACM_Iface::ipacmcfg->decreaseFltRuleCount(rx_prop->rx[0].src_pipe, IPA_IP_v4, 1);

		/* ipv6 case */
		if (m_filtering.DeleteFilteringHdls(&software_routing_fl_rule_hdl[1],
																				IPA_IP_v6, 1) == false)
		{
			IPACMERR("Error Adding Filtering rule, aborting...\n");
			res = IPACM_FAILURE;
			goto fail;
		}
		IPACM_Iface::ipacmcfg->decreaseFltRuleCount(rx_prop->rx[0].src_pipe, IPA_IP_v6, 1);
		softwarerouting_act = false;
#if 0
	}
	else
	{
		if (ip_type == IPA_IP_v4)
		{
			ip = IPA_IP_v4;
		}
		else
		{
			ip = IPA_IP_v6;
		}


		if (ip_type == IPA_IP_v4)
		{
			flt_hdl = software_routing_fl_rule_hdl[0];
		}
		else
		{
			flt_hdl = software_routing_fl_rule_hdl[1];
		}

		if (m_filtering.DeleteFilteringHdls(&flt_hdl, ip, 1) == false)
		{
			IPACMERR("Error Adding Filtering rule, aborting...\n");
			res = IPACM_FAILURE;
			goto fail;
		}
		IPACM_Iface::ipacmcfg->decreaseFltRuleCount(rx_prop->rx[0].src_pipe, ip, 1);
		softwarerouting_act = false;
	}
#endif

fail:
	return res;
}

/* Query ipa_interface_index by given linux interface_index */
int IPACM_Iface::iface_ipa_index_query
(
	 int interface_index
)
{
	int fd;
	int link = INVALID_IFACE;
	int i = 0;
	struct ifreq ifr;


	if(IPACM_Iface::ipacmcfg->iface_table == NULL)
	{
		IPACMERR("Iface table in IPACM_Config is not available.\n");
		return link;
	}

	/* Search known linux interface-index and map to IPA interface-index*/
	for (i = 0; i < IPACM_Iface::ipacmcfg->ipa_num_ipa_interfaces; i++)
	{
		if (interface_index == IPACM_Iface::ipacmcfg->iface_table[i].netlink_interface_index)
		{
			link = i;
			IPACMDBG("Interface (%s) found: linux(%d) ipa(%d) \n",
							 IPACM_Iface::ipacmcfg->iface_table[i].iface_name,
							 IPACM_Iface::ipacmcfg->iface_table[i].netlink_interface_index,
							 link);
			return link;
			break;
		}
	}

	/* Search/Configure linux interface-index and map it to IPA interface-index */
	if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
	{
		PERROR("get interface name socket create failed");
		return IPACM_FAILURE;
	}

	memset(&ifr, 0, sizeof(struct ifreq));

	ifr.ifr_ifindex = interface_index;
	IPACMDBG_H("Interface index %d\n", interface_index);

	if (ioctl(fd, SIOCGIFNAME, &ifr) < 0)
	{
		PERROR("call_ioctl_on_dev: ioctl failed:");
		close(fd);
		return IPACM_FAILURE;
	}
	close(fd);

	IPACMDBG_H("Received interface name %s\n", ifr.ifr_name);
	for (i = 0; i < IPACM_Iface::ipacmcfg->ipa_num_ipa_interfaces; i++)
	{
		if (strncmp(ifr.ifr_name,
								IPACM_Iface::ipacmcfg->iface_table[i].iface_name,
								sizeof(IPACM_Iface::ipacmcfg->iface_table[i].iface_name)) == 0)
		{
			IPACMDBG_H("Interface (%s) linux(%d) mapped to ipa(%d) \n", ifr.ifr_name,
							 IPACM_Iface::ipacmcfg->iface_table[i].netlink_interface_index, i);

			link = i;
			IPACM_Iface::ipacmcfg->iface_table[i].netlink_interface_index = interface_index;
			break;
		}
	}

	return link;
}

/* Query ipa_interface ipv4_addr by given linux interface_index */
void IPACM_Iface::iface_addr_query
(
	 int interface_index
)
{
	int fd;
	struct ifreq ifr;
	struct ifaddrs *myaddrs, *ifa;
	ipacm_cmd_q_data evt_data;
	ipacm_event_data_addr *data_addr;
	struct in_addr iface_ipv4;

	/* use linux interface-index to find interface name */
	if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
	{
		PERROR("get interface name socket create failed");
		return ;
	}

	memset(&ifr, 0, sizeof(struct ifreq));

	ifr.ifr_ifindex = interface_index;
	IPACMDBG_H("Interface index %d\n", interface_index);

	if (ioctl(fd, SIOCGIFNAME, &ifr) < 0)
	{
		PERROR("call_ioctl_on_dev: ioctl failed:");
		close(fd);
		return ;
	}
	IPACMDBG_H("Interface index %d name: %s\n", interface_index,ifr.ifr_name);
	close(fd);

	/* query ipv4/v6 address */
    if(getifaddrs(&myaddrs) != 0)
	{
        IPACMERR("getifaddrs");
		return ;
	}

    for (ifa = myaddrs; ifa != NULL; ifa = ifa->ifa_next)
    {
        if (ifa->ifa_addr == NULL)
            continue;
        if (!(ifa->ifa_flags & IFF_UP))
            continue;

		if(strcmp(ifr.ifr_name,ifa->ifa_name) == 0) // find current iface
		{
			IPACMDBG_H("Internal post new_addr event for iface %s\n", ifa->ifa_name);
			switch (ifa->ifa_addr->sa_family)
			{
				case AF_INET:
				{
					struct sockaddr_in *s4 = (struct sockaddr_in *)ifa->ifa_addr;
					IPACMDBG_H("ipv4 address %s\n",inet_ntoa(s4->sin_addr));
					iface_ipv4 = s4->sin_addr;
					/* post new_addr event to command queue */
					data_addr = (ipacm_event_data_addr *)malloc(sizeof(ipacm_event_data_addr));
					if(data_addr == NULL)
					{
						IPACMERR("unable to allocate memory for event data_addr\n");
						freeifaddrs(myaddrs);
						return ;
					}
					data_addr->iptype = IPA_IP_v4;
					data_addr->if_index = interface_index;
					data_addr->ipv4_addr = 	iface_ipv4.s_addr;
					data_addr->ipv4_addr = ntohl(data_addr->ipv4_addr);
					IPACMDBG_H("Posting IPA_ADDR_ADD_EVENT with if index:%d, ipv4 addr:0x%x\n",
						data_addr->if_index,
						data_addr->ipv4_addr);

					evt_data.event = IPA_ADDR_ADD_EVENT;
					evt_data.evt_data = data_addr;
					IPACM_EvtDispatcher::PostEvt(&evt_data);
					break;
				}

				case AF_INET6:
				{
					struct sockaddr_in6 *s6 = (struct sockaddr_in6 *)ifa->ifa_addr;
					/* post new_addr event to command queue */
					data_addr = (ipacm_event_data_addr *)malloc(sizeof(ipacm_event_data_addr));
					if(data_addr == NULL)
					{
						IPACMERR("unable to allocate memory for event data_addr\n");
						freeifaddrs(myaddrs);
						return ;
					}
					data_addr->iptype = IPA_IP_v6;
					data_addr->if_index = interface_index;
					memcpy(data_addr->ipv6_addr,
									&s6->sin6_addr,
									sizeof(data_addr->ipv6_addr));
					data_addr->ipv6_addr[0] = ntohl(data_addr->ipv6_addr[0]);
					data_addr->ipv6_addr[1] = ntohl(data_addr->ipv6_addr[1]);
					data_addr->ipv6_addr[2] = ntohl(data_addr->ipv6_addr[2]);
					data_addr->ipv6_addr[3] = ntohl(data_addr->ipv6_addr[3]);
					IPACMDBG_H("Posting IPA_ADDR_ADD_EVENT with if index:%d, ipv6 addr:0x%x:%x:%x:%x\n",
							data_addr->if_index,
							data_addr->ipv6_addr[0], data_addr->ipv6_addr[1], data_addr->ipv6_addr[2], data_addr->ipv6_addr[3]);

					evt_data.event = IPA_ADDR_ADD_EVENT;
					evt_data.evt_data = data_addr;
					IPACM_EvtDispatcher::PostEvt(&evt_data);
					break;
				}

				default:
					continue;
			}
		}
	}
    freeifaddrs(myaddrs);
	return ;
}

/*Query the IPA endpoint property */
int IPACM_Iface::query_iface_property(void)
{
	int res = IPACM_SUCCESS, fd = 0;
	uint32_t cnt=0;

	fd = open(DEVICE_NAME, O_RDWR);
	IPACMDBG("iface query-property \n");
	if (0 == fd)
	{
		IPACMERR("Failed opening %s.\n", DEVICE_NAME);
		return IPACM_FAILURE;
	}

	iface_query = (struct ipa_ioc_query_intf *)
		 calloc(1, sizeof(struct ipa_ioc_query_intf));
	if(iface_query == NULL)
	{
		IPACMERR("Unable to allocate iface_query memory.\n");
		close(fd);
		return IPACM_FAILURE;
	}
	IPACMDBG_H("iface name %s\n", dev_name);
	memcpy(iface_query->name, dev_name, sizeof(dev_name));

	if (ioctl(fd, IPA_IOC_QUERY_INTF, iface_query) < 0)
	{
		PERROR("ioctl IPA_IOC_QUERY_INTF failed\n");
		/* iface_query memory will free when iface-down*/
		res = IPACM_FAILURE;
	}

	if(iface_query->num_tx_props > 0)
	{
		tx_prop = (struct ipa_ioc_query_intf_tx_props *)
			 calloc(1, sizeof(struct ipa_ioc_query_intf_tx_props) +
							iface_query->num_tx_props * sizeof(struct ipa_ioc_tx_intf_prop));
		if(tx_prop == NULL)
		{
			IPACMERR("Unable to allocate tx_prop memory.\n");
			close(fd);
			return IPACM_FAILURE;
		}
		memcpy(tx_prop->name, dev_name, sizeof(tx_prop->name));
		tx_prop->num_tx_props = iface_query->num_tx_props;

		if (ioctl(fd, IPA_IOC_QUERY_INTF_TX_PROPS, tx_prop) < 0)
		{
			PERROR("ioctl IPA_IOC_QUERY_INTF_TX_PROPS failed\n");
			/* tx_prop memory will free when iface-down*/
			res = IPACM_FAILURE;
		}

		if (res != IPACM_FAILURE)
		{
			for (cnt = 0; cnt < tx_prop->num_tx_props; cnt++)
			{
				IPACMDBG_H("Tx(%d):attrib-mask:0x%x, ip-type: %d, dst_pipe: %d, alt_dst_pipe: %d, header: %s\n",
						cnt, tx_prop->tx[cnt].attrib.attrib_mask,
						tx_prop->tx[cnt].ip, tx_prop->tx[cnt].dst_pipe,
						tx_prop->tx[cnt].alt_dst_pipe,
						tx_prop->tx[cnt].hdr_name);

				if (tx_prop->tx[cnt].dst_pipe == 0)
				{
					IPACMERR("Tx(%d): wrong tx property: dst_pipe: 0.\n", cnt);
					close(fd);
					return IPACM_FAILURE;
				}
				if (tx_prop->tx[cnt].alt_dst_pipe == 0 &&
					((memcmp(dev_name, "wlan0", sizeof("wlan0")) == 0) ||
					(memcmp(dev_name, "wlan1", sizeof("wlan1")) == 0)))
				{
					IPACMERR("Tx(%d): wrong tx property: alt_dst_pipe: 0. \n", cnt);
					close(fd);
					return IPACM_FAILURE;
				}

			}
		}

	}

	if (iface_query->num_rx_props > 0)
	{
		rx_prop = (struct ipa_ioc_query_intf_rx_props *)
			 calloc(1, sizeof(struct ipa_ioc_query_intf_rx_props) +
							iface_query->num_rx_props * sizeof(struct ipa_ioc_rx_intf_prop));
		if(rx_prop == NULL)
		{
			IPACMERR("Unable to allocate rx_prop memory.\n");
			close(fd);
			return IPACM_FAILURE;
		}
		memcpy(rx_prop->name, dev_name,
				 sizeof(rx_prop->name));
		rx_prop->num_rx_props = iface_query->num_rx_props;

		if (ioctl(fd, IPA_IOC_QUERY_INTF_RX_PROPS, rx_prop) < 0)
		{
			PERROR("ioctl IPA_IOC_QUERY_INTF_RX_PROPS failed\n");
			/* rx_prop memory will free when iface-down*/
			res = IPACM_FAILURE;
		}

		if (res != IPACM_FAILURE)
		{
			for (cnt = 0; cnt < rx_prop->num_rx_props; cnt++)
			{
				IPACMDBG_H("Rx(%d):attrib-mask:0x%x, ip-type: %d, src_pipe: %d\n",
								 cnt, rx_prop->rx[cnt].attrib.attrib_mask, rx_prop->rx[cnt].ip, rx_prop->rx[cnt].src_pipe);
			}
		}
	}

	/* Add Natting iface to IPACM_Config if there is  Rx/Tx property */
	if (rx_prop != NULL || tx_prop != NULL)
	{
		IPACMDBG_H(" Has rx/tx properties registered for iface %s, add for NATTING \n", dev_name);
        IPACM_Iface::ipacmcfg->AddNatIfaces(dev_name);
	}

	close(fd);
	return res;
}

/*Configure the initial filter rules */
int IPACM_Iface::init_fl_rule(ipa_ip_type iptype)
{

	int res = IPACM_SUCCESS, len = 0;
	struct ipa_flt_rule_add flt_rule_entry;
	ipa_ioc_add_flt_rule *m_pFilteringTable;

  /* Adding this hack because WLAN may not registered for Rx-endpoint, other ifaces will always have*/
	const char *dev_wlan0="wlan0";
	const char *dev_wlan1="wlan1";
	const char *dev_ecm0="ecm0";

	/* update the iface ip-type to be IPA_IP_v4, IPA_IP_v6 or both*/
	if (iptype == IPA_IP_v4)
	{

		if ((ip_type == IPA_IP_v4) || (ip_type == IPA_IP_MAX))
		{
			IPACMDBG_H(" interface(%s:%d) already in ip-type %d\n", dev_name, ipa_if_num, ip_type);
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

		IPACMDBG_H(" interface(%s:%d) now ip-type is %d\n", dev_name, ipa_if_num, ip_type);
	}
	else
	{

		if ((ip_type == IPA_IP_v6) || (ip_type == IPA_IP_MAX))
		{
			IPACMDBG_H(" interface(%s:%d) already in ip-type %d\n", dev_name, ipa_if_num, ip_type);
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

		IPACMDBG_H(" interface(%s:%d) now ip-type is %d\n", dev_name, ipa_if_num, ip_type);
	}

    /* ADD corresponding ipa_rm_resource_name of RX-endpoint before adding all IPV4V6 FT-rules */
	if((IPACM_Iface::ipacmcfg->iface_table[ipa_if_num].if_cat== WAN_IF) || (IPACM_Iface::ipacmcfg->iface_table[ipa_if_num].if_cat== EMBMS_IF))
	{
		IPACMDBG_H(" NOT add producer dependency on dev %s with registered rx-prop cat:%d \n", dev_name, IPACM_Iface::ipacmcfg->iface_table[ipa_if_num].if_cat);
	}
	else
	{
		if(rx_prop != NULL)
		{
			IPACMDBG_H("dev %s add producer dependency\n", dev_name);
			IPACMDBG_H("depend Got pipe %d rm index : %d \n", rx_prop->rx[0].src_pipe, IPACM_Iface::ipacmcfg->ipa_client_rm_map_tbl[rx_prop->rx[0].src_pipe]);
			IPACM_Iface::ipacmcfg->AddRmDepend(IPACM_Iface::ipacmcfg->ipa_client_rm_map_tbl[rx_prop->rx[0].src_pipe],false);
		}
		else
		{
			/* only wlan may take software-path, not register Rx-property*/
			if(strcmp(dev_name,dev_wlan0) == 0 || strcmp(dev_name,dev_wlan1) == 0)
			{
				IPACMDBG_H("dev %s add producer dependency\n", dev_name);
				IPACMDBG_H("depend Got piperm index : %d \n", IPA_RM_RESOURCE_HSIC_PROD);
				IPACM_Iface::ipacmcfg->AddRmDepend(IPA_RM_RESOURCE_HSIC_PROD,true);
			}
			if(strcmp(dev_name,dev_ecm0) == 0)
			{
				IPACMDBG_H("dev %s add producer dependency\n", dev_name);
				IPACMDBG_H("depend Got piperm index : %d \n", IPA_RM_RESOURCE_USB_PROD);
				IPACM_Iface::ipacmcfg->AddRmDepend(IPA_RM_RESOURCE_USB_PROD,true);
			}
		}
	}
	if (rx_prop == NULL)
	{
		IPACMDBG_H("No rx properties registered for iface %s\n", dev_name);
		return IPACM_SUCCESS;
	}

	/* construct ipa_ioc_add_flt_rule with default filter rules */
	if (iptype == IPA_IP_v4)
	{
		len = sizeof(struct ipa_ioc_add_flt_rule) +
			 (IPV4_DEFAULT_FILTERTING_RULES * sizeof(struct ipa_flt_rule_add));

		m_pFilteringTable = (struct ipa_ioc_add_flt_rule *)calloc(1, len);
		if (!m_pFilteringTable)
		{
			IPACMERR("Error Locate ipa_flt_rule_add memory...\n");
			return IPACM_FAILURE;
		}

		m_pFilteringTable->commit = 1;
		m_pFilteringTable->ep = rx_prop->rx[0].src_pipe;
		m_pFilteringTable->global = false;
		m_pFilteringTable->ip = iptype;
		m_pFilteringTable->num_rules = (uint8_t)IPV4_DEFAULT_FILTERTING_RULES;

		/* Configuring Fragment Filtering Rule */
		memset(&flt_rule_entry, 0, sizeof(struct ipa_flt_rule_add));

		flt_rule_entry.rule.retain_hdr = 1;
		flt_rule_entry.at_rear = true;
		flt_rule_entry.flt_rule_hdl = -1;
		flt_rule_entry.status = -1;
		flt_rule_entry.rule.action = IPA_PASS_TO_EXCEPTION;
#ifdef FEATURE_IPA_V3
		flt_rule_entry.at_rear = false;
		flt_rule_entry.rule.hashable = false;
#endif
		IPACMDBG_H("rx property attrib mask:0x%x\n", rx_prop->rx[0].attrib.attrib_mask);
		memcpy(&flt_rule_entry.rule.attrib,
					 &rx_prop->rx[0].attrib,
					 sizeof(flt_rule_entry.rule.attrib));

		flt_rule_entry.rule.attrib.attrib_mask |= IPA_FLT_FRAGMENT;
		memcpy(&(m_pFilteringTable->rules[0]), &flt_rule_entry, sizeof(struct ipa_flt_rule_add));

		/* Configuring Multicast Filtering Rule */
		memcpy(&flt_rule_entry.rule.attrib,
					 &rx_prop->rx[0].attrib,
					 sizeof(flt_rule_entry.rule.attrib));
		flt_rule_entry.rule.attrib.attrib_mask |= IPA_FLT_DST_ADDR;
		flt_rule_entry.rule.attrib.u.v4.dst_addr_mask = 0xF0000000;
		flt_rule_entry.rule.attrib.u.v4.dst_addr = 0xE0000000;
#ifdef FEATURE_IPA_V3
		flt_rule_entry.at_rear = true;
		flt_rule_entry.rule.hashable = true;
#endif
		memcpy(&(m_pFilteringTable->rules[1]), &flt_rule_entry, sizeof(struct ipa_flt_rule_add));

		/* Configuring Broadcast Filtering Rule */
		flt_rule_entry.rule.attrib.u.v4.dst_addr_mask = 0xFFFFFFFF;
		flt_rule_entry.rule.attrib.u.v4.dst_addr = 0xFFFFFFFF;
#ifdef FEATURE_IPA_V3
		flt_rule_entry.at_rear = true;
		flt_rule_entry.rule.hashable = true;
#endif
		memcpy(&(m_pFilteringTable->rules[2]), &flt_rule_entry, sizeof(struct ipa_flt_rule_add));

		if (false == m_filtering.AddFilteringRule(m_pFilteringTable))
		{
			IPACMERR("Error Adding Filtering rule, aborting...\n");
			res = IPACM_FAILURE;
			goto fail;
		}
		else
		{
			IPACM_Iface::ipacmcfg->increaseFltRuleCount(rx_prop->rx[0].src_pipe, IPA_IP_v4, IPV4_DEFAULT_FILTERTING_RULES);
			/* copy filter hdls */
			for (int i = 0; i < IPV4_DEFAULT_FILTERTING_RULES; i++)
			{
				if (m_pFilteringTable->rules[i].status == 0)
				{
					dft_v4fl_rule_hdl[i] = m_pFilteringTable->rules[i].flt_rule_hdl;
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
		len = sizeof(struct ipa_ioc_add_flt_rule) +
			 (IPV6_DEFAULT_FILTERTING_RULES * sizeof(struct ipa_flt_rule_add));

		m_pFilteringTable = (struct ipa_ioc_add_flt_rule *)calloc(1, len);
		if (!m_pFilteringTable)
		{
			IPACMERR("Error Locate ipa_flt_rule_add memory...\n");
			return IPACM_FAILURE;
		}

		m_pFilteringTable->commit = 1;
		m_pFilteringTable->ep = rx_prop->rx[0].src_pipe;
		m_pFilteringTable->global = false;
		m_pFilteringTable->ip = iptype;
		m_pFilteringTable->num_rules = (uint8_t)IPV6_DEFAULT_FILTERTING_RULES;

		memset(&flt_rule_entry, 0, sizeof(struct ipa_flt_rule_add));

		flt_rule_entry.rule.retain_hdr = 1;
		flt_rule_entry.at_rear = true;
		flt_rule_entry.flt_rule_hdl = -1;
		flt_rule_entry.status = -1;
		flt_rule_entry.rule.action = IPA_PASS_TO_EXCEPTION;
		/* Configuring Multicast Filtering Rule */
		memcpy(&flt_rule_entry.rule.attrib,
					 &rx_prop->rx[0].attrib,
					 sizeof(flt_rule_entry.rule.attrib));
		flt_rule_entry.rule.attrib.attrib_mask |= IPA_FLT_DST_ADDR;
		flt_rule_entry.rule.attrib.u.v6.dst_addr_mask[0] = 0xFF000000;
		flt_rule_entry.rule.attrib.u.v6.dst_addr_mask[1] = 0x00000000;
		flt_rule_entry.rule.attrib.u.v6.dst_addr_mask[2] = 0x00000000;
		flt_rule_entry.rule.attrib.u.v6.dst_addr_mask[3] = 0x00000000;
		flt_rule_entry.rule.attrib.u.v6.dst_addr[0] = 0XFF000000;
		flt_rule_entry.rule.attrib.u.v6.dst_addr[1] = 0x00000000;
		flt_rule_entry.rule.attrib.u.v6.dst_addr[2] = 0x00000000;
		flt_rule_entry.rule.attrib.u.v6.dst_addr[3] = 0X00000000;
#ifdef FEATURE_IPA_V3
		flt_rule_entry.at_rear = true;
		flt_rule_entry.rule.hashable = true;
#endif
		memcpy(&(m_pFilteringTable->rules[0]), &flt_rule_entry, sizeof(struct ipa_flt_rule_add));

		/* Configuring fe80::/10 Link-Scoped Unicast Filtering Rule */
		flt_rule_entry.rule.attrib.u.v6.dst_addr_mask[0] = 0XFFC00000;
		flt_rule_entry.rule.attrib.u.v6.dst_addr_mask[1] = 0x00000000;
		flt_rule_entry.rule.attrib.u.v6.dst_addr_mask[2] = 0x00000000;
		flt_rule_entry.rule.attrib.u.v6.dst_addr_mask[3] = 0x00000000;
		flt_rule_entry.rule.attrib.u.v6.dst_addr[0] = 0xFE800000;
		flt_rule_entry.rule.attrib.u.v6.dst_addr[1] = 0x00000000;
		flt_rule_entry.rule.attrib.u.v6.dst_addr[2] = 0x00000000;
		flt_rule_entry.rule.attrib.u.v6.dst_addr[3] = 0X00000000;
#ifdef FEATURE_IPA_V3
		flt_rule_entry.at_rear = true;
		flt_rule_entry.rule.hashable = true;
#endif
		memcpy(&(m_pFilteringTable->rules[1]), &flt_rule_entry, sizeof(struct ipa_flt_rule_add));

		/* Configuring fec0::/10 Reserved by IETF Filtering Rule */
		flt_rule_entry.rule.attrib.u.v6.dst_addr_mask[0] = 0XFFC00000;
		flt_rule_entry.rule.attrib.u.v6.dst_addr_mask[1] = 0x00000000;
		flt_rule_entry.rule.attrib.u.v6.dst_addr_mask[2] = 0x00000000;
		flt_rule_entry.rule.attrib.u.v6.dst_addr_mask[3] = 0x00000000;
		flt_rule_entry.rule.attrib.u.v6.dst_addr[0] = 0xFEC00000;
		flt_rule_entry.rule.attrib.u.v6.dst_addr[1] = 0x00000000;
		flt_rule_entry.rule.attrib.u.v6.dst_addr[2] = 0x00000000;
		flt_rule_entry.rule.attrib.u.v6.dst_addr[3] = 0X00000000;
#ifdef FEATURE_IPA_V3
		flt_rule_entry.at_rear = true;
		flt_rule_entry.rule.hashable = true;
#endif
		memcpy(&(m_pFilteringTable->rules[2]), &flt_rule_entry, sizeof(struct ipa_flt_rule_add));

#ifdef FEATURE_IPA_ANDROID
		IPACMDBG_H("Add TCP ctrl rules: total num %d\n", IPV6_DEFAULT_FILTERTING_RULES);
		memset(&flt_rule_entry, 0, sizeof(struct ipa_flt_rule_add));

		flt_rule_entry.at_rear = true;
		flt_rule_entry.flt_rule_hdl = -1;
		flt_rule_entry.status = -1;

		flt_rule_entry.rule.retain_hdr = 1;
		flt_rule_entry.rule.to_uc = 0;
		flt_rule_entry.rule.action = IPA_PASS_TO_EXCEPTION;
		flt_rule_entry.rule.eq_attrib_type = 1;
		flt_rule_entry.rule.eq_attrib.rule_eq_bitmap = 0;

		if(rx_prop->rx[0].attrib.attrib_mask & IPA_FLT_META_DATA)
		{
			flt_rule_entry.rule.eq_attrib.rule_eq_bitmap |= (1<<14);
			flt_rule_entry.rule.eq_attrib.metadata_meq32_present = 1;
			flt_rule_entry.rule.eq_attrib.metadata_meq32.offset = 0;
			flt_rule_entry.rule.eq_attrib.metadata_meq32.value = rx_prop->rx[0].attrib.meta_data;
			flt_rule_entry.rule.eq_attrib.metadata_meq32.mask = rx_prop->rx[0].attrib.meta_data_mask;
		}

		flt_rule_entry.rule.eq_attrib.rule_eq_bitmap |= (1<<1);
		flt_rule_entry.rule.eq_attrib.protocol_eq_present = 1;
		flt_rule_entry.rule.eq_attrib.protocol_eq = IPACM_FIREWALL_IPPROTO_TCP;

		flt_rule_entry.rule.eq_attrib.rule_eq_bitmap |= (1<<8);
		flt_rule_entry.rule.eq_attrib.num_ihl_offset_meq_32 = 1;
		flt_rule_entry.rule.eq_attrib.ihl_offset_meq_32[0].offset = 12;

		/* add TCP FIN rule*/
		flt_rule_entry.rule.eq_attrib.ihl_offset_meq_32[0].value = (((uint32_t)1)<<TCP_FIN_SHIFT);
		flt_rule_entry.rule.eq_attrib.ihl_offset_meq_32[0].mask = (((uint32_t)1)<<TCP_FIN_SHIFT);
		memcpy(&(m_pFilteringTable->rules[3]), &flt_rule_entry, sizeof(struct ipa_flt_rule_add));

		/* add TCP SYN rule*/
		flt_rule_entry.rule.eq_attrib.ihl_offset_meq_32[0].value = (((uint32_t)1)<<TCP_SYN_SHIFT);
		flt_rule_entry.rule.eq_attrib.ihl_offset_meq_32[0].mask = (((uint32_t)1)<<TCP_SYN_SHIFT);
		memcpy(&(m_pFilteringTable->rules[4]), &flt_rule_entry, sizeof(struct ipa_flt_rule_add));

		/* add TCP RST rule*/
		flt_rule_entry.rule.eq_attrib.ihl_offset_meq_32[0].value = (((uint32_t)1)<<TCP_RST_SHIFT);
		flt_rule_entry.rule.eq_attrib.ihl_offset_meq_32[0].mask = (((uint32_t)1)<<TCP_RST_SHIFT);
		memcpy(&(m_pFilteringTable->rules[5]), &flt_rule_entry, sizeof(struct ipa_flt_rule_add));
#endif
		if (m_filtering.AddFilteringRule(m_pFilteringTable) == false)
		{
			IPACMERR("Error Adding Filtering rule, aborting...\n");
			res = IPACM_FAILURE;
			goto fail;
		}
		else
		{
			IPACM_Iface::ipacmcfg->increaseFltRuleCount(rx_prop->rx[0].src_pipe, IPA_IP_v6, IPV6_DEFAULT_FILTERTING_RULES);
			/* copy filter hdls */
			for (int i = 0;
					 i < IPV6_DEFAULT_FILTERTING_RULES;
					 i++)
			{
				if (m_pFilteringTable->rules[i].status == 0)
				{
					dft_v6fl_rule_hdl[i] = m_pFilteringTable->rules[i].flt_rule_hdl;
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
	free(m_pFilteringTable);

	return res;
}

/*  get ipa interface name */
int IPACM_Iface::ipa_get_if_index
(
  char * if_name,
  int * if_index
)
{
  int fd;
  struct ifreq ifr;

  if((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
  {
    IPACMERR("get interface index socket create failed \n");
    return IPACM_FAILURE;
  }

  memset(&ifr, 0, sizeof(struct ifreq));
  (void)strncpy(ifr.ifr_name, if_name, sizeof(ifr.ifr_name));
  IPACMDBG_H("interface name (%s)\n", if_name);

  if (ioctl(fd,SIOCGIFINDEX , &ifr) < 0)
  {
    IPACMERR("call_ioctl_on_dev: ioctl failed, interface name (%s):\n", ifr.ifr_name);
    close(fd);
    return IPACM_FAILURE;
  }

  *if_index = ifr.ifr_ifindex;
  IPACMDBG_H("Interface index %d\n", *if_index);
  close(fd);
  return IPACM_SUCCESS;
}
