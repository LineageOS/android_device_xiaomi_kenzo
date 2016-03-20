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

#ifndef IPACM_CONNTRACK_FILTER_H
#define IPACM_CONNTRACK_FILTER_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <errno.h>

#include "IPACM_ConntrackClient.h"
#include "IPACM_CmdQueue.h"
#include "IPACM_Conntrack_NATApp.h"
#include "IPACM_EvtDispatcher.h"
#include "IPACM_Defs.h"

#ifndef IPACM_DEBUG
#define IPACM_DEBUG
#endif

extern "C"
{
#include <libnetfilter_conntrack/libnetfilter_conntrack.h>
#include <libnetfilter_conntrack/libnetfilter_conntrack_tcp.h>
#include <sys/inotify.h>
}

using namespace std;

#define UDP_TIMEOUT_UPDATE 20
#define BROADCAST_IPV4_ADDR 0xFFFFFFFF

class IPACM_ConntrackClient
{

private:
   static IPACM_ConntrackClient *pInstance;

   struct nfct_handle *tcp_hdl;
   struct nfct_handle *udp_hdl;
   struct nfct_filter *tcp_filter;
   struct nfct_filter *udp_filter;
   static int IPA_Conntrack_Filters_Ignore_Local_Addrs(struct nfct_filter *filter);
   static int IPA_Conntrack_Filters_Ignore_Bridge_Addrs(struct nfct_filter *filter);
   static int IPA_Conntrack_Filters_Ignore_Local_Iface(struct nfct_filter *, ipacm_event_iface_up *);
   IPACM_ConntrackClient();

public:
   static int IPAConntrackEventCB(enum nf_conntrack_msg_type type,
                                  struct nf_conntrack *ct,
                                  void *data);

   static int IPA_Conntrack_UDP_Filter_Init(void);
   static int IPA_Conntrack_TCP_Filter_Init(void);
   static void* TCPRegisterWithConnTrack(void *);
   static void* UDPRegisterWithConnTrack(void *);
   static void* UDPConnTimeoutUpdate(void *);

   static void UpdateUDPFilters(void *, bool);
   static void UpdateTCPFilters(void *, bool);
   static void Read_TcpUdp_Timeout(char *in, int len);

   static IPACM_ConntrackClient* GetInstance();

#ifdef IPACM_DEBUG
#define iptodot(X,Y) \
		 IPACMLOG(" %s(0x%x): %d.%d.%d.%d\n", X, Y, ((Y>>24) & 0xFF), ((Y>>16) & 0xFF), ((Y>>8) & 0xFF), (Y & 0xFF));
#endif

#define log_nat(A,B,C,D,E,F) \
		IPACMDBG_H("protocol %d Private IP: %d.%d.%d.%d\t Target IP: %d.%d.%d.%d\t private port: %d public port: %d %s",A,((B>>24) & 0xFF), ((B>>16) & 0xFF), ((B>>8) & 0xFF), (B & 0xFF), ((C>>24) & 0xFF), ((C>>16) & 0xFF),((C>>8) & 0xFF),(C & 0xFF),D,E,F);

};

#endif  /* IPACM_CONNTRACK_FILTER_H */
