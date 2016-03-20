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
  IPACM_Xml.h

  @brief
  This file implements the XML specific parsing functionality.

  @Author
  Skylar Chang/Shihuan Liu

*/
#ifndef IPACM_XML_H
#define IPACM_XML_H

#include <linux/msm_ipa.h>
#include "IPACM_Defs.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <stdint.h>
#include <libxml/parser.h>
#include <libxml/tree.h>
#ifdef __cplusplus
extern "C" {
#endif

  
#define IPACM_ASSERT(a)                                     \
if (!(a)) {                                                 \
	fprintf(stderr, "%s, %d: assertion (a) failed!",    \
	__FILE__,                                           \
	__LINE__);                                          \
	abort();                                            \
}
  
/* Max allowed size of the XML file (2 MB) */
#define IPACM_XML_MAX_FILESIZE               (2 << 20)
#define IPACM_MAX_FIREWALL_ENTRIES            50
#define IPACM_IPV6_ADDR_LEN                   16

/* Defines for clipping space or space & quotes (single, double) */
#define IPACM_XML_CLIP_SPACE         " "
#define IPACM_XML_CLIP_SPACE_QUOTES  " '\""

#define MAX_XML_STR_LEN                 120
  
/* IPA Config Entries */
#define system_TAG                           "system"
#define ODU_TAG                              "ODUCFG"
#define ODUMODE_TAG                          "OduMode"
#define ODUEMBMS_OFFLOAD_TAG                 "eMBMS_offload"
#define ODU_ROUTER_TAG                       "router"
#define ODU_BRIDGE_TAG                       "bridge"
#define IPACMCFG_TAG                         "IPACM"
#define IPACMIFACECFG_TAG                    "IPACMIface"
#define IFACE_TAG                            "Iface"
#define NAME_TAG                             "Name"
#define CATEGORY_TAG                         "Category"
#define MODE_TAG                             "Mode"
#define IPACMPRIVATESUBNETCFG_TAG            "IPACMPrivateSubnet"
#define SUBNET_TAG                           "Subnet"
#define SUBNETADDRESS_TAG                    "SubnetAddress"
#define SUBNETMASK_TAG                       "SubnetMask"
#define WANIF_TAG                            "WAN"
#define LANIF_TAG                            "LAN"
#define WLANIF_TAG                           "WLAN"
#define WLAN_FULL_MODE_TAG                   "full"
#define WLAN_INTERNET_MODE_TAG               "internet"
#define WLAN_MODE_TAG                        "WlanMode"
#define VIRTUALIF_TAG                        "VIRTUAL"
#define UNKNOWNIF_TAG                        "UNKNOWN"
#define ODUIF_TAG                            "ODU"
#define EMBMSIF_TAG                          "EMBMS"
#define ETHIF_TAG                            "ETH"
#define IFACE_ROUTER_MODE_TAG                "ROUTER"
#define IFACE_BRIDGE_MODE_TAG                "BRIDGE"
#define IPACMALG_TAG                         "IPACMALG"
#define ALG_TAG                              "ALG"
#define Protocol_TAG                         "Protocol"
#define Port_TAG                             "Port"
#define TCP_PROTOCOL_TAG                     "TCP"
#define UDP_PROTOCOL_TAG                     "UDP"

/* FIREWALL Config Entries */
#define Firewall_TAG                         "Firewall"
#define MobileAPFirewallCfg_TAG              "MobileAPFirewallCfg"
#define FirewallEnabled_TAG                  "FirewallEnabled"
#define FirewallPktsAllowed_TAG              "FirewallPktsAllowed"

#define IPFamily_TAG                         "IPFamily"
#define IPV4SourceAddress_TAG                "IPV4SourceAddress"
#define IPV4SourceIPAddress_TAG              "IPV4SourceIPAddress"
#define IPV4SourceSubnetMask_TAG             "IPV4SourceSubnetMask"

#define IPV4DestinationAddress_TAG           "IPV4DestinationAddress"
#define IPV4DestinationIPAddress_TAG         "IPV4DestinationIPAddress"
#define IPV4DestinationSubnetMask_TAG        "IPV4DestinationSubnetMask"

#define IPV4TypeOfService_TAG                "IPV4TypeOfService"
#define TOSValue_TAG                         "TOSValue"
#define TOSMask_TAG                          "TOSMask"

#define IPV4NextHeaderProtocol_TAG           "IPV4NextHeaderProtocol"

#define IPV6SourceAddress_TAG                "IPV6SourceAddress"
#define IPV6SourceIPAddress_TAG              "IPV6SourceIPAddress"
#define IPV6SourcePrefix_TAG                 "IPV6SourcePrefix"

#define IPV6DestinationAddress_TAG           "IPV6DestinationAddress"
#define IPV6DestinationIPAddress_TAG         "IPV6DestinationIPAddress"
#define IPV6DestinationPrefix_TAG            "IPV6DestinationPrefix"

#define IPV6TrafficClass_TAG                 "IPV6TrafficClass"
#define TrfClsValue_TAG                      "TrfClsValue"
#define TrfClsMask_TAG                       "TrfClsMask"

#define IPV6NextHeaderProtocol_TAG           "IPV6NextHeaderProtocol"

#define TCPSource_TAG                        "TCPSource"
#define TCPSourcePort_TAG                    "TCPSourcePort"
#define TCPSourceRange_TAG                   "TCPSourceRange"

#define TCPDestination_TAG                   "TCPDestination"
#define TCPDestinationPort_TAG               "TCPDestinationPort"
#define TCPDestinationRange_TAG              "TCPDestinationRange"

#define UDPSource_TAG                        "UDPSource"
#define UDPSourcePort_TAG                    "UDPSourcePort"
#define UDPSourceRange_TAG                   "UDPSourceRange"

#define UDPDestination_TAG                   "UDPDestination"
#define UDPDestinationPort_TAG               "UDPDestinationPort"
#define UDPDestinationRange_TAG              "UDPDestinationRange"

#define ICMPType_TAG                         "ICMPType"
#define ICMPCode_TAG                         "ICMPCode"

#define ESP_TAG                              "ESP"
#define ESPSPI_TAG                           "ESPSPI"

#define TCP_UDPSource_TAG                    "TCP_UDPSource"
#define TCP_UDPSourcePort_TAG                "TCP_UDPSourcePort"
#define TCP_UDPSourceRange_TAG               "TCP_UDPSourceRange"

#define TCP_UDPDestination_TAG               "TCP_UDPDestination"
#define TCP_UDPDestinationPort_TAG           "TCP_UDPDestinationPort"
#define TCP_UDPDestinationRange_TAG          "TCP_UDPDestinationRange"

#define IPACMNat_TAG                         "IPACMNAT"
#define NAT_MaxEntries_TAG                   "MaxNatEntries"

/*---------------------------------------------------------------------------
      IP protocol numbers - use in dss_socket() to identify protocols.
      Also contains the extension header types for IPv6.
---------------------------------------------------------------------------*/
typedef enum
{
	IPACM_FIREWALL_IPV6_BASE_HDR        = 4,                               /* IPv6 Base Header           */
	IPACM_FIREWALL_IPPROTO_HOP_BY_HOP_OPT_HDR = 0,                         /* Hop-by-hop Option Header   */
	IPACM_FIREWALL_IPPROTO_ICMP         = 1,                               /* ICMP protocol */
	IPACM_FIREWALL_IPPROTO_IGMP         = 2,                               /* IGMP protocol */
	IPACM_FIREWALL_IPPROTO_IP           = IPACM_FIREWALL_IPV6_BASE_HDR,    /* IPv4          */
	IPACM_FIREWALL_IPPROTO_TCP          = 6,                               /* TCP Protocol */
	IPACM_FIREWALL_IPPROTO_UDP          = 17,                              /* UDP Protocol */
	IPACM_FIREWALL_IPPROTO_IPV6         = 41,                              /* IPv6                       */
	IPACM_FIREWALL_IPPROTO_ROUTING_HDR  = 43,                              /* Routing Header             */
	IPACM_FIREWALL_IPPROTO_FRAG_HDR     = 44,                              /* Fragmentation Header       */
	IPACM_FIREWALL_IPPROTO_GRE          = 47,                              /* GRE Protocol */
	IPACM_FIREWALL_IPPROTO_ESP          = 50,                              /* ESP Protocol */
	IPACM_FIREWALL_IPPROTO_AH           = 51,                              /* Authentication Header      */
	IPACM_FIREWALL_IPPROTO_ICMP6        = 58,                              /* ICMPv6                     */
	IPACM_FIREWALL_NO_NEXT_HDR          = 59,                              /* No Next Header for IPv6    */
	IPACM_FIREWALL_IPPROTO_DEST_OPT_HDR = 60,                              /* Destination Options Header */
	IPACM_FIREWALL_IPPROTO_MOBILITY_HDR = 135,                             /* Mobility Header            */
	IPACM_FIREWALL_IPPROTO_TCP_UDP      = 253                              /* Unspecified protocol used for IPACM */
} ipacm_firewall_ip_protocol_enum_type;

/* define as mobileap firewall rule format*/
typedef enum
{
	IP_V4 = 4,
	IP_V6 = 6
} firewall_ip_version_enum;
  
/*---------------------------------------------------------------------------
           Extended FireWall Entry Configuration.
---------------------------------------------------------------------------*/
typedef struct
{
	struct ipa_rule_attrib attrib;
	firewall_ip_version_enum  ip_vsn;
} IPACM_extd_firewall_entry_conf_t;


/*---------------------------------------------------------------------------
           Extended FireWall configuration.
---------------------------------------------------------------------------*/
typedef union
{
	IPACM_extd_firewall_entry_conf_t extd_firewall_entry;
} IPACM_extd_firewall_conf_t;


typedef struct
{
	char firewall_config_file[IPA_MAX_FILE_LEN];
	uint8_t  num_extd_firewall_entries;
	IPACM_extd_firewall_entry_conf_t extd_firewall_entries[IPACM_MAX_FIREWALL_ENTRIES];
	bool rule_action_accept;
	bool firewall_enable;
} IPACM_firewall_conf_t;
  


typedef struct
{
	uint8_t num_iface_entries;
	ipa_ifi_dev_name_t iface_entries[IPA_MAX_IFACE_ENTRIES];
} ipacm_iface_conf_t;

typedef struct
{
	uint8_t num_subnet_entries;
	ipa_private_subnet private_subnet_entries[IPA_MAX_PRIVATE_SUBNET_ENTRIES];
} ipacm_private_subnet_conf_t;

typedef struct
{
	uint8_t protocol;
	uint16_t port;
} ipacm_alg;

typedef struct
{
	uint8_t num_alg_entries;
	ipacm_alg alg_entries[IPA_MAX_ALG_ENTRIES];
} ipacm_alg_conf_t;

 
typedef struct  _IPACM_conf_t
{
	ipacm_iface_conf_t iface_config;
	ipacm_private_subnet_conf_t private_subnet_config;
	ipacm_alg_conf_t alg_config;
	int nat_max_entries;
	bool odu_enable;
	bool router_mode_enable;
	bool odu_embms_enable;
	int num_wlan_guest_ap;
} IPACM_conf_t;  

/* This function read IPACM XML configuration*/
int ipacm_read_cfg_xml
(
	char *xml_file,                              /* Filename and path     */
	IPACM_conf_t *config                         /* Mobile AP config data */
);

/* This function reads QCMAP Firewall XML and store in IPACM Firewall stucture */
int IPACM_read_firewall_xml
(
	char *xml_file,                                 /* Filename and path     */
	IPACM_firewall_conf_t *config                   /* Mobile AP config data */
);

#ifdef __cplusplus
}
#endif

#endif //IPACM_XML
