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
   IPACM_Xml.cpp

  @brief
   This file implements the XML specific parsing functionality.

  @Author
   Skylar Chang/Shihuan Liu
*/

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "IPACM_Xml.h"
#include "IPACM_Log.h"
#include "IPACM_Netlink.h"

static char* IPACM_read_content_element
(
	 xmlNode* element
);

static int32_t IPACM_util_icmp_string
(
	 const char* xml_str,
	 const char* str
);

static int ipacm_cfg_xml_parse_tree
(
	 xmlNode* xml_node,
	 IPACM_conf_t *config
);

static int IPACM_firewall_xml_parse_tree
(
	 xmlNode* xml_node,
	 IPACM_firewall_conf_t *config
);

/*Reads content (stored as child) of the element */
static char* IPACM_read_content_element
(
	 xmlNode* element
)
{
	xmlNode* child_ptr;

	for (child_ptr  = element->children;
			 child_ptr != NULL;
			 child_ptr  = child_ptr->next)
	{
		if (child_ptr->type == XML_TEXT_NODE)
		{
			return (char*)child_ptr->content;
		}
	}
	return NULL;
}

/* insensitive comparison of a libxml's string (xml_str) and a regular string (str)*/
static int32_t IPACM_util_icmp_string
(
	 const char* xml_str,
	 const char* str
)
{
	int32_t ret = -1;

	if (NULL != xml_str && NULL != str)
	{
		uint32_t len1 = strlen(str);
		uint32_t len2 = strlen(xml_str);
		/* If the lengths match, do the string comparison */
		if (len1 == len2)
		{
			ret = strncasecmp(xml_str, str, len1);
		}
	}

	return ret;
}

/* This function read IPACM XML and populate the IPA CM Cfg */
int ipacm_read_cfg_xml(char *xml_file, IPACM_conf_t *config)
{
	xmlDocPtr doc = NULL;
	xmlNode* root = NULL;
	int ret_val = IPACM_SUCCESS;

	/* Invoke the XML parser and obtain the parse tree */
	doc = xmlReadFile(xml_file, "UTF-8", XML_PARSE_NOBLANKS);
	if (doc == NULL) {
		IPACMDBG_H("IPACM_xml_parse: libxml returned parse error!\n");
		return IPACM_FAILURE;
	}

	/*Get the root of the tree*/
	root = xmlDocGetRootElement(doc);

	memset(config, 0, sizeof(IPACM_conf_t));

	/* parse the xml tree returned by libxml */
	ret_val = ipacm_cfg_xml_parse_tree(root, config);

	if (ret_val != IPACM_SUCCESS)
	{
		IPACMDBG_H("IPACM_xml_parse: ipacm_cfg_xml_parse_tree returned parse error!\n");
	}

	/* Free up the libxml's parse tree */
	xmlFreeDoc(doc);

	return ret_val;
}

/* This function traverses the xml tree*/
static int ipacm_cfg_xml_parse_tree
(
	 xmlNode* xml_node,
	 IPACM_conf_t *config
)
{
	int32_t ret_val = IPACM_SUCCESS;
	int str_size;
	char* content;
	char content_buf[MAX_XML_STR_LEN];

	if (NULL == xml_node)
		return ret_val;
	while ( xml_node != NULL &&
				 ret_val == IPACM_SUCCESS)
	{
		switch (xml_node->type)
		{
		case XML_ELEMENT_NODE:
			{
				if (IPACM_util_icmp_string((char*)xml_node->name, system_TAG) == 0 ||
						IPACM_util_icmp_string((char*)xml_node->name, ODU_TAG) == 0 ||
						IPACM_util_icmp_string((char*)xml_node->name, IPACMCFG_TAG) == 0 ||
						IPACM_util_icmp_string((char*)xml_node->name, IPACMIFACECFG_TAG) == 0 ||
						IPACM_util_icmp_string((char*)xml_node->name, IFACE_TAG) == 0 ||
						IPACM_util_icmp_string((char*)xml_node->name, IPACMPRIVATESUBNETCFG_TAG) == 0 ||
						IPACM_util_icmp_string((char*)xml_node->name, SUBNET_TAG) == 0 ||
						IPACM_util_icmp_string((char*)xml_node->name, IPACMALG_TAG) == 0 ||
						IPACM_util_icmp_string((char*)xml_node->name, ALG_TAG) == 0 ||
						IPACM_util_icmp_string((char*)xml_node->name, IPACMNat_TAG) == 0)
				{
					if (0 == IPACM_util_icmp_string((char*)xml_node->name, IFACE_TAG))
					{
						/* increase iface entry number */
						config->iface_config.num_iface_entries++;
					}

					if (0 == IPACM_util_icmp_string((char*)xml_node->name, SUBNET_TAG))
					{
						/* increase iface entry number */
						config->private_subnet_config.num_subnet_entries++;
					}

					if (0 == IPACM_util_icmp_string((char*)xml_node->name, ALG_TAG))
					{
						/* increase iface entry number */
						config->alg_config.num_alg_entries++;
					}
					/* go to child */
					ret_val = ipacm_cfg_xml_parse_tree(xml_node->children, config);
				}
				else if (IPACM_util_icmp_string((char*)xml_node->name, ODUMODE_TAG) == 0)
				{
					IPACMDBG_H("inside ODU-XML\n");
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						if (0 == strncasecmp(content_buf, ODU_ROUTER_TAG, str_size))
						{
							config->router_mode_enable = true;
							IPACMDBG_H("router-mode enable %d\n", config->router_mode_enable);
						}
						else if (0 == strncasecmp(content_buf, ODU_BRIDGE_TAG, str_size))
						{
							config->router_mode_enable = false;
							IPACMDBG_H("router-mode enable %d\n", config->router_mode_enable);
						}
					}
				}
				else if (IPACM_util_icmp_string((char*)xml_node->name, ODUEMBMS_OFFLOAD_TAG) == 0)
				{
					IPACMDBG_H("inside ODU-XML\n");
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						if (atoi(content_buf))
						{
							config->odu_embms_enable = true;
							IPACMDBG_H("router-mode enable %d buf(%d)\n", config->odu_embms_enable, atoi(content_buf));
						}
						else
						{
							config->odu_embms_enable = false;
							IPACMDBG_H("router-mode enable %d buf(%d)\n", config->odu_embms_enable, atoi(content_buf));
						}
					}
				}
				else if (IPACM_util_icmp_string((char*)xml_node->name, NAME_TAG) == 0)
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						strlcpy(config->iface_config.iface_entries[config->iface_config.num_iface_entries - 1].iface_name, content_buf, str_size+1);
						IPACMDBG_H("Name %s\n", config->iface_config.iface_entries[config->iface_config.num_iface_entries - 1].iface_name);
					}
				}
				else if (IPACM_util_icmp_string((char*)xml_node->name, CATEGORY_TAG) == 0)
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						if (0 == strncasecmp(content_buf, WANIF_TAG, str_size))
						{
							config->iface_config.iface_entries[config->iface_config.num_iface_entries - 1].if_cat = WAN_IF;
							IPACMDBG_H("Category %d\n", config->iface_config.iface_entries[config->iface_config.num_iface_entries - 1].if_cat);
						}
						else if (0 == strncasecmp(content_buf, LANIF_TAG, str_size))
						{
							config->iface_config.iface_entries[config->iface_config.num_iface_entries - 1].if_cat = LAN_IF;
							IPACMDBG_H("Category %d\n", config->iface_config.iface_entries[config->iface_config.num_iface_entries - 1].if_cat);
						}
						else if (0 == strncasecmp(content_buf, WLANIF_TAG, str_size))
						{
							config->iface_config.iface_entries[config->iface_config.num_iface_entries - 1].if_cat = WLAN_IF;
							IPACMDBG_H("Category %d\n", config->iface_config.iface_entries[config->iface_config.num_iface_entries - 1].if_cat);
						}
						else  if (0 == strncasecmp(content_buf, VIRTUALIF_TAG, str_size))
						{
							config->iface_config.iface_entries[config->iface_config.num_iface_entries - 1].if_cat = VIRTUAL_IF;
							IPACMDBG_H("Category %d\n", config->iface_config.iface_entries[config->iface_config.num_iface_entries - 1].if_cat);
						}
						else  if (0 == strncasecmp(content_buf, UNKNOWNIF_TAG, str_size))
						{
							config->iface_config.iface_entries[config->iface_config.num_iface_entries - 1].if_cat = UNKNOWN_IF;
							IPACMDBG_H("Category %d\n", config->iface_config.iface_entries[config->iface_config.num_iface_entries - 1].if_cat);
						}
						else  if (0 == strncasecmp(content_buf, ETHIF_TAG, str_size))
						{
							config->iface_config.iface_entries[config->iface_config.num_iface_entries - 1].if_cat = ETH_IF;
							IPACMDBG_H("Category %d\n", config->iface_config.iface_entries[config->iface_config.num_iface_entries - 1].if_cat);
						}
						else  if (0 == strncasecmp(content_buf, ODUIF_TAG, str_size))
						{
							config->iface_config.iface_entries[config->iface_config.num_iface_entries - 1].if_cat = ODU_IF;
							IPACMDBG("Category %d\n", config->iface_config.iface_entries[config->iface_config.num_iface_entries - 1].if_cat);
						}
					}
				}
				else if (IPACM_util_icmp_string((char*)xml_node->name, MODE_TAG) == 0)
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						if (0 == strncasecmp(content_buf, IFACE_ROUTER_MODE_TAG, str_size))
						{
							config->iface_config.iface_entries[config->iface_config.num_iface_entries - 1].if_mode = ROUTER;
							IPACMDBG_H("Iface mode %d\n", config->iface_config.iface_entries[config->iface_config.num_iface_entries - 1].if_mode);
						}
						else  if (0 == strncasecmp(content_buf, IFACE_BRIDGE_MODE_TAG, str_size))
						{
							config->iface_config.iface_entries[config->iface_config.num_iface_entries - 1].if_mode = BRIDGE;
							IPACMDBG_H("Iface mode %d\n", config->iface_config.iface_entries[config->iface_config.num_iface_entries - 1].if_mode);
						}
					}
				}
				else if (IPACM_util_icmp_string((char*)xml_node->name, WLAN_MODE_TAG) == 0)
				{
					IPACMDBG_H("Inside WLAN-XML\n");
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);

						if (0 == strncasecmp(content_buf, WLAN_FULL_MODE_TAG, str_size))
						{
							config->iface_config.iface_entries[config->iface_config.num_iface_entries - 1].wlan_mode = FULL;
							IPACMDBG_H("Wlan-mode full(%d)\n",
									config->iface_config.iface_entries[config->iface_config.num_iface_entries - 1].wlan_mode);
						}
						else  if (0 == strncasecmp(content_buf, WLAN_INTERNET_MODE_TAG, str_size))
						{
							config->iface_config.iface_entries[config->iface_config.num_iface_entries - 1].wlan_mode = INTERNET;
							config->num_wlan_guest_ap++;
							IPACMDBG_H("Wlan-mode internet(%d)\n",
									config->iface_config.iface_entries[config->iface_config.num_iface_entries - 1].wlan_mode);
						}
					}
				}
				else if (IPACM_util_icmp_string((char*)xml_node->name, SUBNETADDRESS_TAG) == 0)
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						content_buf[MAX_XML_STR_LEN-1] = '\0';
						config->private_subnet_config.private_subnet_entries[config->private_subnet_config.num_subnet_entries - 1].subnet_addr
							 = ntohl(inet_addr(content_buf));
						IPACMDBG_H("subnet_addr: %s \n", content_buf);
					}
				}
				else if (IPACM_util_icmp_string((char*)xml_node->name, SUBNETMASK_TAG) == 0)
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						content_buf[MAX_XML_STR_LEN-1] = '\0';
						config->private_subnet_config.private_subnet_entries[config->private_subnet_config.num_subnet_entries - 1].subnet_mask
							 = ntohl(inet_addr(content_buf));
						IPACMDBG_H("subnet_mask: %s \n", content_buf);
					}
				}
				else if (IPACM_util_icmp_string((char*)xml_node->name, Protocol_TAG) == 0)
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						content_buf[MAX_XML_STR_LEN-1] = '\0';

						if (0 == strncasecmp(content_buf, TCP_PROTOCOL_TAG, str_size))
						{
							config->alg_config.alg_entries[config->alg_config.num_alg_entries - 1].protocol = IPPROTO_TCP;
							IPACMDBG_H("Protocol %s: %d\n",
									content_buf, config->alg_config.alg_entries[config->alg_config.num_alg_entries - 1].protocol);
						}
						else if (0 == strncasecmp(content_buf, UDP_PROTOCOL_TAG, str_size))
						{
							config->alg_config.alg_entries[config->alg_config.num_alg_entries - 1].protocol = IPPROTO_UDP;
							IPACMDBG_H("Protocol %s: %d\n",
									content_buf, config->alg_config.alg_entries[config->alg_config.num_alg_entries - 1].protocol);
						}
					}
				}
				else if (IPACM_util_icmp_string((char*)xml_node->name, Port_TAG) == 0)
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						config->alg_config.alg_entries[config->alg_config.num_alg_entries - 1].port
							 = atoi(content_buf);
						IPACMDBG_H("port %d\n", config->alg_config.alg_entries[config->alg_config.num_alg_entries - 1].port);
					}
				}
				else if (IPACM_util_icmp_string((char*)xml_node->name, NAT_MaxEntries_TAG) == 0)
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						config->nat_max_entries = atoi(content_buf);
						IPACMDBG_H("Nat Table Max Entries %d\n", config->nat_max_entries);
					}
				}
			}
			break;
		default:
			break;
		}
		/* go to sibling */
		xml_node = xml_node->next;
	} /* end while */
	return ret_val;
}

/* This function read QCMAP CM Firewall XML and populate the QCMAP CM Cfg */
int IPACM_read_firewall_xml(char *xml_file, IPACM_firewall_conf_t *config)
{
	xmlDocPtr doc = NULL;
	xmlNode* root = NULL;
	int ret_val;

	IPACM_ASSERT(xml_file != NULL);
	IPACM_ASSERT(config != NULL);

	/* invoke the XML parser and obtain the parse tree */
	doc = xmlReadFile(xml_file, "UTF-8", XML_PARSE_NOBLANKS);
	if (doc == NULL) {
		IPACMDBG_H("IPACM_xml_parse: libxml returned parse error\n");
		return IPACM_FAILURE;
	}
	/*get the root of the tree*/
	root = xmlDocGetRootElement(doc);

	/* parse the xml tree returned by libxml*/
	ret_val = IPACM_firewall_xml_parse_tree(root, config);

	if (ret_val != IPACM_SUCCESS)
	{
		IPACMDBG_H("IPACM_xml_parse: ipacm_firewall_xml_parse_tree returned parse error!\n");
	}

	/* free the tree */
	xmlFreeDoc(doc);

	return ret_val;
}


/* This function traverses the firewall xml tree */
static int IPACM_firewall_xml_parse_tree
(
	 xmlNode* xml_node,
	 IPACM_firewall_conf_t *config
)
{
	int mask_value_v6, mask_index;
	int32_t ret_val = IPACM_SUCCESS;
	char *content;
	int str_size;
	char content_buf[MAX_XML_STR_LEN];
	struct in6_addr ip6_addr;

	IPACM_ASSERT(config != NULL);

	if (NULL == xml_node)
		return ret_val;

	while ( xml_node != NULL &&
				 ret_val == IPACM_SUCCESS)
	{
		switch (xml_node->type)
		{

		case XML_ELEMENT_NODE:
			{
				if (0 == IPACM_util_icmp_string((char*)xml_node->name, system_TAG) ||
						0 == IPACM_util_icmp_string((char*)xml_node->name, MobileAPFirewallCfg_TAG) ||
						0 == IPACM_util_icmp_string((char*)xml_node->name, Firewall_TAG) ||
						0 == IPACM_util_icmp_string((char*)xml_node->name, FirewallEnabled_TAG)  ||
						0 == IPACM_util_icmp_string((char*)xml_node->name, FirewallPktsAllowed_TAG))
				{
					if (0 == IPACM_util_icmp_string((char*)xml_node->name, Firewall_TAG))
					{
						/* increase firewall entry num */
						config->num_extd_firewall_entries++;
					}

					if (0 == IPACM_util_icmp_string((char*)xml_node->name, FirewallPktsAllowed_TAG))
					{
						/* setup action of matched rules */
					    content = IPACM_read_content_element(xml_node);
					    if (content)
					    {
						        str_size = strlen(content);
						        memset(content_buf, 0, sizeof(content_buf));
						        memcpy(content_buf, (void *)content, str_size);
							if (atoi(content_buf)==1)
							{
								config->rule_action_accept = true;
							}
							else
							{
								config->rule_action_accept = false;
							}
							IPACMDBG_H(" Allow traffic which matches rules ?:%d\n",config->rule_action_accept);
					    }
				        }

					if (0 == IPACM_util_icmp_string((char*)xml_node->name, FirewallEnabled_TAG))
					{
						/* setup if firewall enable or not */
					    content = IPACM_read_content_element(xml_node);
					    if (content)
					    {
						        str_size = strlen(content);
						        memset(content_buf, 0, sizeof(content_buf));
						        memcpy(content_buf, (void *)content, str_size);
							if (atoi(content_buf)==1)
							{
								config->firewall_enable = true;
							}
						        else
							{
								config->firewall_enable = false;
							}
							IPACMDBG_H(" Firewall Enable?:%d\n", config->firewall_enable);
				            }
					}
					/* go to child */
					ret_val = IPACM_firewall_xml_parse_tree(xml_node->children, config);
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, IPFamily_TAG))
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						config->extd_firewall_entries[config->num_extd_firewall_entries - 1].ip_vsn
							 = (firewall_ip_version_enum)atoi(content_buf);
						IPACMDBG_H("\n IP family type is %d \n",
								config->extd_firewall_entries[config->num_extd_firewall_entries - 1].ip_vsn);
					}
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, IPV4SourceAddress_TAG))
				{
					config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.attrib_mask |= IPA_FLT_SRC_ADDR;
					/* go to child */
					ret_val = IPACM_firewall_xml_parse_tree(xml_node->children, config);
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, IPV4SourceIPAddress_TAG))
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						content_buf[MAX_XML_STR_LEN-1] = '\0';
						config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v4.src_addr
							 = ntohl(inet_addr(content_buf));
						IPACMDBG_H("IPv4 source address is: %s \n", content_buf);
					}
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, IPV4SourceSubnetMask_TAG))
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						content_buf[MAX_XML_STR_LEN-1] = '\0';
						config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v4.src_addr_mask
							 = ntohl(inet_addr(content_buf));
						IPACMDBG_H("IPv4 source subnet mask is: %s \n", content_buf);
					}
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, IPV4DestinationAddress_TAG))
				{
					config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.attrib_mask |= IPA_FLT_DST_ADDR;
					/* go to child */
					ret_val = IPACM_firewall_xml_parse_tree(xml_node->children, config);
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, IPV4DestinationIPAddress_TAG))
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						content_buf[MAX_XML_STR_LEN-1] = '\0';
						config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v4.dst_addr
							 = ntohl(inet_addr(content_buf));
						IPACMDBG_H("IPv4 destination address is: %s \n", content_buf);
					}
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, IPV4DestinationSubnetMask_TAG))
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						content_buf[MAX_XML_STR_LEN-1] = '\0';
						if (content_buf > 0)
						{
							config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v4.dst_addr_mask
								 = ntohl(inet_addr(content_buf));
							IPACMDBG_H("IPv4 destination subnet mask is: %s \n", content_buf);
						}
					}
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, IPV4TypeOfService_TAG))
				{
					config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.attrib_mask |= IPA_FLT_TOS;
					/* go to child */
					ret_val = IPACM_firewall_xml_parse_tree(xml_node->children, config);
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, TOSValue_TAG))
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v4.tos
							 = atoi(content_buf);
						IPACMDBG_H("\n IPV4 TOS val is %d \n",
										 config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v4.tos);
					}
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, TOSMask_TAG))
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v4.tos
							 &= atoi(content_buf);
						IPACMDBG_H("\n IPv4 TOS mask is %d \n",
								config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v4.tos);
					}
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, IPV4NextHeaderProtocol_TAG))
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.attrib_mask |= IPA_FLT_PROTOCOL;
						config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v4.protocol = atoi(content_buf);
						IPACMDBG_H("\n IPv4 next header prot is %d \n",
								 config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v4.protocol);
					}
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, IPV6SourceAddress_TAG))
				{
					config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.attrib_mask |=
						 IPA_FLT_SRC_ADDR;
					/* go to child */
					ret_val = IPACM_firewall_xml_parse_tree(xml_node->children, config);
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, IPV6SourceIPAddress_TAG))
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						inet_pton(AF_INET6, content_buf, &ip6_addr);
						memcpy(config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v6.src_addr,
									 ip6_addr.s6_addr, IPACM_IPV6_ADDR_LEN * sizeof(uint8_t));
						config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v6.src_addr[0]=ntohl(config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v6.src_addr[0]);
						config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v6.src_addr[1]=ntohl(config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v6.src_addr[1]);
						config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v6.src_addr[2]=ntohl(config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v6.src_addr[2]);
						config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v6.src_addr[3]=ntohl(config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v6.src_addr[3]);

						IPACMDBG_H("\n ipv6 source addr is %d \n ",
								config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v6.src_addr[0]);
					}
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, IPV6SourcePrefix_TAG))
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						mask_value_v6 = atoi(content_buf);
						for (mask_index = 0; mask_index < 4; mask_index++)
						{
							if (mask_value_v6 >= 32)
							{
								mask_v6(32, &(config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v6.src_addr_mask[mask_index]));
								mask_value_v6 -= 32;
							}
							else
							{
								mask_v6(mask_value_v6, &(config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v6.src_addr_mask[mask_index]));
								mask_value_v6 = 0;
							}
						}
						IPACMDBG_H("\n ipv6 source prefix is %d \n", atoi(content_buf));
					}
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, IPV6DestinationAddress_TAG))
				{
					config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.attrib_mask |=
						 IPA_FLT_DST_ADDR;
					/* go to child */
					ret_val = IPACM_firewall_xml_parse_tree(xml_node->children, config);
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, IPV6DestinationIPAddress_TAG))
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						inet_pton(AF_INET6, content_buf, &ip6_addr);
						memcpy(config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v6.dst_addr,
									 ip6_addr.s6_addr, IPACM_IPV6_ADDR_LEN * sizeof(uint8_t));
						config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v6.dst_addr[0]=ntohl(config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v6.dst_addr[0]);
						config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v6.dst_addr[1]=ntohl(config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v6.dst_addr[1]);
						config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v6.dst_addr[2]=ntohl(config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v6.dst_addr[2]);
						config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v6.dst_addr[3]=ntohl(config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v6.dst_addr[3]);
						IPACMDBG_H("\n ipv6 dest addr is %d \n",
								 config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v6.dst_addr[0]);
					}
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, IPV6DestinationPrefix_TAG))
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						mask_value_v6 = atoi(content_buf);
						for (mask_index = 0; mask_index < 4; mask_index++)
						{
							if (mask_value_v6 >= 32)
							{
								mask_v6(32, &(config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v6.dst_addr_mask[mask_index]));
								mask_value_v6 -= 32;
							}
							else
							{
								mask_v6(mask_value_v6, &(config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v6.dst_addr_mask[mask_index]));
								mask_value_v6 = 0;
							}
						}
						IPACMDBG_H("\n ipv6 dest prefix is %d \n", atoi(content_buf));
					}
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, IPV6TrafficClass_TAG))
				{
					config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.attrib_mask |= IPA_FLT_TC;
					/* go to child */
					ret_val = IPACM_firewall_xml_parse_tree(xml_node->children, config);
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, TrfClsValue_TAG))
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v6.tc
							 = atoi(content_buf);
						IPACMDBG_H("\n ipv6 trf class val is %d \n",
								 config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v6.tc);
					}
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, TrfClsMask_TAG))
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v6.tc
							 &= atoi(content_buf);
						IPACMDBG_H("\n ipv6 trf class mask is %d \n", atoi(content_buf));
					}
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, IPV6NextHeaderProtocol_TAG))
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.attrib_mask |= IPA_FLT_NEXT_HDR;
						config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v6.next_hdr
							 = atoi(content_buf);
						IPACMDBG_H("\n ipv6 next header protocol is %d \n",
								 config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.u.v6.next_hdr);
					}
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, TCPSource_TAG))
				{
					/* go to child */
					ret_val = IPACM_firewall_xml_parse_tree(xml_node->children, config);
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, TCPSourcePort_TAG))
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.src_port
							 = atoi(content_buf);
					}
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, TCPSourceRange_TAG))
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						if (atoi(content_buf) != 0)
						{
							config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.attrib_mask |= IPA_FLT_SRC_PORT_RANGE;
							config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.src_port_lo
								= config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.src_port;
							config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.src_port_hi
								= config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.src_port + atoi(content_buf);
							config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.src_port = 0;
							IPACMDBG_H("\n tcp source port from %d to %d \n",
									config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.src_port_lo,
									config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.src_port_hi);
						}
						else
						{
							config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.attrib_mask |= IPA_FLT_SRC_PORT;
							IPACMDBG_H("\n tcp source port= %d \n",
									config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.src_port);
						}
					}
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, TCPDestination_TAG))
				{
					/* go to child */
					ret_val = IPACM_firewall_xml_parse_tree(xml_node->children, config);
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, TCPDestinationPort_TAG))
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.dst_port
							 = atoi(content_buf);
					}
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, TCPDestinationRange_TAG))
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						if(atoi(content_buf)!=0)
						{
							config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.attrib_mask |= IPA_FLT_DST_PORT_RANGE;
							config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.dst_port_lo
								= config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.dst_port;
							config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.dst_port_hi
								= config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.dst_port + atoi(content_buf);
							config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.dst_port = 0;
							IPACMDBG_H("\n tcp dest port from %d to %d \n",
									config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.dst_port_lo,
									config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.dst_port_hi);
						}
						else
						{
							config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.attrib_mask |= IPA_FLT_DST_PORT;
							IPACMDBG_H("\n tcp dest port= %d \n",
									config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.dst_port);
						}
					}
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, UDPSource_TAG))
				{
					/* go to child */
					ret_val = IPACM_firewall_xml_parse_tree(xml_node->children, config);
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, UDPSourcePort_TAG))
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.src_port
							 = atoi(content_buf);
					}
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, UDPSourceRange_TAG))
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						if(atoi(content_buf)!=0)
						{
							config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.attrib_mask |= IPA_FLT_SRC_PORT_RANGE;
 							config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.src_port_lo
								= config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.src_port;
							config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.src_port_hi
								= config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.src_port + atoi(content_buf);
							config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.src_port = 0;
							IPACMDBG_H("\n udp source port from %d to %d \n",
									config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.src_port_lo,
									config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.src_port_hi);
						}
						else
						{
							config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.attrib_mask |= IPA_FLT_SRC_PORT;
							IPACMDBG_H("\n udp source port= %d \n",
									config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.src_port);
						}
					}
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, UDPDestination_TAG))
				{
					/* go to child */
					ret_val = IPACM_firewall_xml_parse_tree(xml_node->children, config);
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, UDPDestinationPort_TAG))
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.dst_port
							 = atoi(content_buf);
					}
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, UDPDestinationRange_TAG))
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						if(atoi(content_buf)!=0)
						{
							config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.attrib_mask |= IPA_FLT_DST_PORT_RANGE;
							config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.dst_port_lo
								= config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.dst_port;
							config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.dst_port_hi
								= config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.dst_port + atoi(content_buf);
							config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.dst_port = 0;
							IPACMDBG_H("\n UDP dest port from %d to %d \n",
									config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.dst_port_lo,
									config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.dst_port_hi);
						}
						else
						{
							config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.attrib_mask |= IPA_FLT_DST_PORT;
							IPACMDBG_H("\n UDP dest port= %d \n",
									config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.dst_port);
						}
					}
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, ICMPType_TAG))
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.type = atoi(content_buf);
						config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.attrib_mask |= IPA_FLT_TYPE;
						IPACMDBG_H("\n icmp type is %d \n",
								 config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.type);
					}
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, ICMPCode_TAG))
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.code = atoi(content_buf);
						config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.attrib_mask |= IPA_FLT_CODE;
						IPACMDBG_H("\n icmp code is %d \n",
								 config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.code);
					}
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, ESPSPI_TAG))
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.spi = atoi(content_buf);
						config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.attrib_mask |= IPA_FLT_SPI;
						IPACMDBG_H("\n esp spi is %d \n",
								config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.spi);
					}
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, TCP_UDPSource_TAG))
				{
					/* go to child */
					ret_val = IPACM_firewall_xml_parse_tree(xml_node->children, config);
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, TCP_UDPSourcePort_TAG))
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content,str_size);
						config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.src_port
							 = atoi(content_buf);
					}
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, TCP_UDPSourceRange_TAG))
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						if(atoi(content_buf)!=0)
						{
							config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.attrib_mask |= IPA_FLT_SRC_PORT_RANGE;
							config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.src_port_lo
								= config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.src_port;
							config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.src_port_hi
								= config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.src_port + atoi(content_buf);
							config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.src_port = 0;
							IPACMDBG_H("\n tcp_udp source port from %d to %d \n",
									config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.src_port_lo,
									config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.src_port_hi);
						}
						else
						{
							config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.attrib_mask |= IPA_FLT_SRC_PORT;
							IPACMDBG_H("\n tcp_udp source port= %d \n",
									config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.src_port);

						}
					}
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, TCP_UDPDestination_TAG))
				{
					ret_val = IPACM_firewall_xml_parse_tree(xml_node->children, config);
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, TCP_UDPDestinationPort_TAG))
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.dst_port
							 = atoi(content_buf);
					}
				}
				else if (0 == IPACM_util_icmp_string((char*)xml_node->name, TCP_UDPDestinationRange_TAG))
				{
					content = IPACM_read_content_element(xml_node);
					if (content)
					{
						str_size = strlen(content);
						memset(content_buf, 0, sizeof(content_buf));
						memcpy(content_buf, (void *)content, str_size);
						if(atoi(content_buf)!=0)
						{
							config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.attrib_mask |= IPA_FLT_DST_PORT_RANGE;
							config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.dst_port_lo
								= config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.dst_port;
							config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.dst_port_hi
								= config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.dst_port + atoi(content_buf);
							config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.dst_port = 0;
							IPACMDBG_H("\n tcp_udp dest port from %d to %d \n",
								config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.dst_port_lo,
								config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.dst_port_hi);
						}
						else
						{
							config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.attrib_mask |= IPA_FLT_DST_PORT;
							IPACMDBG_H("\n tcp_udp dest port= %d \n",
									config->extd_firewall_entries[config->num_extd_firewall_entries - 1].attrib.dst_port);
						}
					}
				}
			}
			break;

		default:
			break;
		}
		/* go to sibling */
		xml_node = xml_node->next;
	} /* end while */
	return ret_val;
}
