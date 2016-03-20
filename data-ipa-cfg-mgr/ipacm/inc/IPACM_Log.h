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
	IPACM_log.h

	@brief
	This file implements the IPAM log functionality.

	@Author
	Skylar Chang

*/

#ifndef IPACM_LOG_H
#define IPACM_LOG_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <string.h>
#include <syslog.h>

#define MAX_BUF_LEN 256

#ifdef FEATURE_IPA_ANDROID
#define IPACMLOG_FILE "/dev/socket/ipacm_log_file"
#else/* defined(FEATURE_IPA_ANDROID) */
#define IPACMLOG_FILE "/etc/ipacm_log_file"
#endif /* defined(NOT FEATURE_IPA_ANDROID)*/

typedef struct ipacm_log_buffer_s {
	char	user_data[MAX_BUF_LEN];
} ipacm_log_buffer_t;

void ipacm_log_send( void * user_data);

static char buffer_send[MAX_BUF_LEN];
static char dmesg_cmd[MAX_BUF_LEN];

#define IPACMDBG_DMESG(fmt, ...) memset(buffer_send, 0, MAX_BUF_LEN);\
							     snprintf(buffer_send,MAX_BUF_LEN,"%s:%d %s: " fmt, __FILE__,  __LINE__, __FUNCTION__, ##__VA_ARGS__);\
								 memset(dmesg_cmd, 0, MAX_BUF_LEN);\
								 snprintf(dmesg_cmd, MAX_BUF_LEN, "echo %s > /dev/kmsg", buffer_send);\
								 system(dmesg_cmd);
#ifdef DEBUG
#define PERROR(fmt)   memset(buffer_send, 0, MAX_BUF_LEN);\
					  snprintf(buffer_send,MAX_BUF_LEN,"%s:%d %s()", __FILE__, __LINE__, __FUNCTION__);\
					  ipacm_log_send (buffer_send); \
                      perror(fmt);
#define IPACMERR(fmt, ...)	memset(buffer_send, 0, MAX_BUF_LEN);\
							snprintf(buffer_send,MAX_BUF_LEN,"ERR: %s:%d %s() " fmt, __FILE__,  __LINE__, __FUNCTION__, ##__VA_ARGS__);\
							ipacm_log_send (buffer_send);\
							printf("ERR: %s:%d %s() " fmt, __FILE__,  __LINE__, __FUNCTION__, ##__VA_ARGS__);
#define IPACMDBG_H(fmt, ...) memset(buffer_send, 0, MAX_BUF_LEN);\
							 snprintf(buffer_send,MAX_BUF_LEN,"%s:%d %s() " fmt, __FILE__,  __LINE__, __FUNCTION__, ##__VA_ARGS__);\
							 ipacm_log_send (buffer_send);\
							 printf("%s:%d %s() " fmt, __FILE__,  __LINE__, __FUNCTION__, ##__VA_ARGS__);
#else
#define PERROR(fmt)   perror(fmt)
#define IPACMERR(fmt, ...)   printf("ERR: %s:%d %s() " fmt, __FILE__,  __LINE__, __FUNCTION__, ##__VA_ARGS__);
#define IPACMDBG_H(fmt, ...) printf("%s:%d %s() " fmt, __FILE__,  __LINE__, __FUNCTION__, ##__VA_ARGS__);
#endif
#define IPACMDBG(fmt, ...)	printf("%s:%d %s() " fmt, __FILE__,  __LINE__, __FUNCTION__, ##__VA_ARGS__);
#define IPACMLOG(fmt, ...)  printf(fmt, ##__VA_ARGS__);

#ifdef __cplusplus
}
#endif

#endif /* IPACM_LOG_H */
