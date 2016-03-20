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
	IPA_Netlink.h

	@brief
	IPACM Netlink Messaging Implementation File

	@Author
	Skylar Chang

*/
#ifndef IPACM_NETLINK_H
#define IPACM_NETLINK_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <pthread.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <linux/if.h>
#include <linux/if_addr.h>
#include <linux/rtnetlink.h>
#include <linux/netlink.h>
#include <netinet/in.h>
#include "IPACM_Defs.h"

#define MAX_NUM_OF_FD 10
#define IPA_NL_MSG_MAX_LEN (2048)

/*--------------------------------------------------------------------------- 
	 Type representing enumeration of NetLink event indication messages
---------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------- 
	 Types representing parsed NetLink message
---------------------------------------------------------------------------*/
#define IPA_NLA_PARAM_NONE        (0x0000)
#define IPA_NLA_PARAM_PREFIXADDR  (0x0001)
#define IPA_NLA_PARAM_LOCALADDR   (0x0002)
#define IPA_NLA_PARAM_LABELNAME   (0x0004)
#define IPA_NLA_PARAM_BCASTADDR   (0x0008)
#define IPA_NLA_PARAM_ACASTADDR   (0x0010)
#define IPA_NLA_PARAM_MCASTADDR   (0x0020)
#define IPA_NLA_PARAM_CACHEINFO   (0x0080)
#define IPA_NLA_PARAM_PROTOINFO   (0x0100)
#define IPA_NLA_PARAM_FLAGS       (0x0200)

#define IPA_RTA_PARAM_NONE        (0x0000)
#define IPA_RTA_PARAM_DST         (0x0001)
#define IPA_RTA_PARAM_SRC         (0x0002)
#define IPA_RTA_PARAM_GATEWAY     (0x0004)
#define IPA_RTA_PARAM_IIF         (0x0008)
#define IPA_RTA_PARAM_OIF         (0x0010)
#define IPA_RTA_PARAM_CACHEINFO   (0x0020)
#define IPA_RTA_PARAM_PRIORITY    (0x0080)
#define IPA_RTA_PARAM_METRICS     (0x0100)


/*--------------------------------------------------------------------------- 
	 Type representing function callback registered with a socket listener 
	 thread for reading from a socket on receipt of an incoming message
---------------------------------------------------------------------------*/
typedef int (*ipa_sock_thrd_fd_read_f)(int fd);

typedef enum
{
	IPA_INIT = 0,
	IPA_LINK_UP_WAIT,
	IPA_LINK_UP,
	IPA_LINK_DOWN_WAIT,
	IPA_LINK_DOWN
} ipa_nl_state_e;

typedef struct
{
	int sk_fd;
	ipa_sock_thrd_fd_read_f read_func;
} ipa_nl_sk_fd_map_info_t;

typedef struct
{
	ipa_nl_sk_fd_map_info_t sk_fds[MAX_NUM_OF_FD];
	fd_set fdset;
	int num_fd;
	int max_fd;
} ipa_nl_sk_fd_set_info_t;

typedef struct
{
	int                 sk_fd;       /* socket descriptor */
	struct sockaddr_nl  sk_addr_loc; /* local address of socket */
} ipa_nl_sk_info_t;

typedef struct ipa_nl_addr_s {
	struct sockaddr_storage        ip_addr;
	unsigned int                   mask;
} ipa_nl_addr_t;

typedef struct ipa_nl_proto_info_s {
	unsigned int                    param_mask;
	unsigned int                    flags;
	struct ifla_cacheinfo           cache_info;
} ipa_nl_proto_info_t;

typedef struct
{
	struct ifinfomsg  metainfo;                   /* from header */
} ipa_nl_link_info_t;



typedef struct ipa_nl_addr_info_s {
	struct ifaddrmsg                metainfo;     /* from header */
	struct                                      /* attributes  */
	{
		unsigned int                  param_mask;
		unsigned char                 label_name[IF_NAME_LEN];
		struct sockaddr_storage       prefix_addr;
		struct sockaddr_storage       local_addr;
		struct sockaddr_storage       bcast_addr;
		struct sockaddr_storage       acast_addr;
		struct sockaddr_storage       mcast_addr;
	} attr_info;
} ipa_nl_addr_info_t;


typedef struct ipa_nl_neigh_info_s {
	struct ndmsg                metainfo;     /* from header */
	struct                                  /* attributes  */
	{
		unsigned int                param_mask;
		struct sockaddr_storage     local_addr;
		struct  sockaddr            lladdr_hwaddr;
	} attr_info;
} ipa_nl_neigh_info_t;



typedef struct ipa_nl_route_info_s {
	struct rtmsg                    metainfo;     /* from header */
	struct                                      /* attributes  */
	{
		unsigned int                  param_mask;
		struct sockaddr_storage       dst_addr;
		struct sockaddr_storage       src_addr;
		struct sockaddr_storage       gateway_addr;
		struct sockaddr_storage       mark_addr;
		struct rta_cacheinfo          cache_info;
		__u32		iif_index;                      /* Link index  */
		__u32		oif_index;                      /* Link index  */
		__u32       priority;
		__u32       metrics;
		ipa_nl_proto_info_t        proto_info;
	} attr_info;
} ipa_nl_route_info_t;

#define IPA_FLOW_TYPE_INVALID      (-1)

typedef struct
{
	unsigned int type;
	bool link_event;
	/* Optional parameters */
	ipa_nl_link_info_t      nl_link_info;
	ipa_nl_addr_info_t      nl_addr_info;
	ipa_nl_neigh_info_t      nl_neigh_info;
	ipa_nl_route_info_t      nl_route_info;
} ipa_nl_msg_t;

/* Initialization routine for listener on NetLink sockets interface */
int ipa_nl_listener_init
(
	 unsigned int nl_type,
	 unsigned int nl_groups,
	 ipa_nl_sk_fd_set_info_t *sk_fdset,
	 ipa_sock_thrd_fd_read_f read_f
	 );

/*  Virtual function registered to receive incoming messages over the NETLINK routing socket*/
int ipa_nl_recv_msg(int fd);

/* map mask value for ipv6 */
int mask_v6(int index, uint32_t *mask);

#ifdef __cplusplus
}
#endif

#endif /* IPACM_NETLINK_H */
