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
		IPACM_iface.h

		@brief
		This file implements the basis Iface definitions.

		@Author
		Skylar Chang

*/
#ifndef IPACM_IFACE_H
#define IPACM_IFACE_H

#include <stdio.h>
#include <IPACM_CmdQueue.h>
#include <linux/msm_ipa.h>
#include "IPACM_Routing.h"
#include "IPACM_Filtering.h"
#include "IPACM_Header.h"
#include "IPACM_EvtDispatcher.h"
#include "IPACM_Xml.h"
#include "IPACM_Log.h"
#include "IPACM_Config.h"
#include "IPACM_Defs.h"
#include <string.h>

/* current support 2 ipv6-address*/
#define MAX_DEFAULT_v4_ROUTE_RULES  1
#define MAX_DEFAULT_v6_ROUTE_RULES  2
#define IPV4_DEFAULT_FILTERTING_RULES 3

#ifdef FEATURE_IPA_ANDROID
#define IPV6_DEFAULT_FILTERTING_RULES 6
#else
#define IPV6_DEFAULT_FILTERTING_RULES 3
#endif

#define IPV6_DEFAULT_LAN_FILTERTING_RULES 1
#define IPV6_NUM_ADDR 3
#define MAX_SOFTWAREROUTING_FILTERTING_RULES 2
#define INVALID_IFACE -1

/* iface */
class IPACM_Iface :public IPACM_Listener
{
public:

	/* Static class for reading IPACM configuration from XML file*/
	static IPACM_Config *ipacmcfg;

	/* IPACM interface id */
	int ipa_if_num;

	/* IPACM interface category */
	ipacm_iface_type ipa_if_cate;

	/* IPACM interface name */
	char dev_name[IF_NAME_LEN];

	/* IPACM interface iptype v4, v6 or both */
	ipa_ip_type ip_type;

	/* IPACM interface v6 ip-address*/
	uint32_t ipv6_addr[MAX_DEFAULT_v6_ROUTE_RULES][4];

	uint32_t software_routing_fl_rule_hdl[MAX_SOFTWAREROUTING_FILTERTING_RULES];

	bool softwarerouting_act;

	/* IPACM number of default route rules for ipv6*/
	int num_dft_rt_v6;

	uint32_t dft_v4fl_rule_hdl[IPV4_DEFAULT_FILTERTING_RULES];
	uint32_t dft_v6fl_rule_hdl[IPV6_DEFAULT_FILTERTING_RULES + IPV6_DEFAULT_LAN_FILTERTING_RULES];
	/* create additional set of v6 RT-rules in Wanv6RT table*/
	uint32_t dft_rt_rule_hdl[MAX_DEFAULT_v4_ROUTE_RULES+2*MAX_DEFAULT_v6_ROUTE_RULES];

	ipa_ioc_query_intf *iface_query;
	ipa_ioc_query_intf_tx_props *tx_prop;
	ipa_ioc_query_intf_rx_props *rx_prop;

	virtual int handle_down_evt() = 0;

	virtual int handle_addr_evt(ipacm_event_data_addr *data) = 0;

	IPACM_Iface(int iface_index);

	virtual void event_callback(ipa_cm_event_id event,
															void *data) = 0;

	/* Query ipa_interface_index by given linux interface_index */
	static int iface_ipa_index_query(int interface_index);

	/* Query ipa_interface ipv4_addr by given linux interface_index */
	static void iface_addr_query(int interface_index);

	/*Query the IPA endpoint property */
	int query_iface_property(void);

	/*Configure the initial filter rules */
	virtual int init_fl_rule(ipa_ip_type iptype);

	/* Get interface index */
	virtual int ipa_get_if_index(char * if_name, int * if_index);

	static IPACM_Routing m_routing;
	static IPACM_Filtering m_filtering;
	static IPACM_Header m_header;

	/* software routing enable */
	virtual int handle_software_routing_enable(void);

	/* software routing disable */
	virtual int handle_software_routing_disable(void);

private:

	static const char *DEVICE_NAME;
};

#endif /* IPACM_IFACE_H */
