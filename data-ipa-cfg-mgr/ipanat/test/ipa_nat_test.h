/*
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of The Linux Foundation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*===========================================================================

                     INCLUDE FILES FOR MODULE

===========================================================================*/
#include "stdint.h"  /* uint32_t */
#include "stdio.h"
#include <netinet/in.h> /* for proto definitions */

#define u32 uint32_t
#define u16 uint16_t
#define u8  uint8_t

/*============ Preconditions to run NAT Test cases =========*/
#define IPA_NAT_TEST_PRE_COND_TE  20

#define CHECK_ERR1(x, tbl_hdl) \
  if(ipa_nat_validate_ipv4_table(tbl_hdl)) { \
    if(sep) {\
       ipa_nat_del_ipv4_tbl(tbl_hdl); \
     }\
    return -1;\
  }\
  if(x) { \
    IPAERR("%d\n", ret); \
    if(sep) {\
      ipa_nat_del_ipv4_tbl(tbl_hdl); \
     }\
     return -1; \
  }

#define CHECK_ERR(x) if(x) { \
    IPAERR("%d\n", ret); \
    return -1;\
 }

#if 0
#define CHECK_ERR(x) if(x) { \
    IPAERR("%d\n", ret); \
    if(sep) {\
      ipa_nat_del_ipv4_tbl(tbl_hdl); \
    }\
    return -1;\
 }
#endif

#define IPADBG(fmt, args...) printf(" %s:%d " fmt, __FUNCTION__, __LINE__, ## args)
#define IPAERR(fmt, args...) printf(" %s:%d " fmt, __FUNCTION__, __LINE__, ## args)

#define NAT_DUMP
int ipa_nat_validate_ipv4_table(u32);

int ipa_nat_test000(int, u32, u8);
int ipa_nat_test001(int, u32, u8);
int ipa_nat_test002(int, u32, u8);
int ipa_nat_test003(int, u32, u8);
int ipa_nat_test004(int, u32, u8);
int ipa_nat_test005(int, u32, u8);
int ipa_nat_test006(int, u32, u8);
int ipa_nat_test007(int, u32, u8);
int ipa_nat_test008(int, u32, u8);
int ipa_nat_test009(int, u32, u8);
int ipa_nat_test010(int, u32, u8);
int ipa_nat_test011(int, u32, u8);
int ipa_nat_test012(int, u32, u8);
int ipa_nat_test013(int, u32, u8);
int ipa_nat_test014(int, u32, u8);
int ipa_nat_test015(int, u32, u8);
int ipa_nat_test016(int, u32, u8);
int ipa_nat_test017(int, u32, u8);
int ipa_nat_test018(int, u32, u8);
int ipa_nat_test019(int, u32, u8);
int ipa_nat_test020(int, u32, u8);
int ipa_nat_test021(int, int);
int ipa_nat_test022(int, u32, u8);
