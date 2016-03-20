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
	IPACM_log.cpp

	@brief
	This file implements the IPAM log functionality.

	@Author
	Skylar Chang

*/
#include "IPACM_Log.h"
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <asm/types.h>
#include <linux/if.h>
#include <sys/un.h>
#include <errno.h>
#include <IPACM_Defs.h>

void logmessage(int log_level)
{
	return;
}

/* start IPACMDIAG socket*/
int create_socket(unsigned int *sockfd)
{

  if ((*sockfd = socket(AF_UNIX, SOCK_DGRAM, 0)) == IPACM_FAILURE)
  {
    perror("Error creating ipacm_log socket\n");
    return IPACM_FAILURE;
  }

  if(fcntl(*sockfd, F_SETFD, FD_CLOEXEC) < 0)
  {
    perror("Couldn't set ipacm_log Close on Exec\n");
  }

  return IPACM_SUCCESS;
}

void ipacm_log_send( void * user_data)
{
	ipacm_log_buffer_t ipacm_log_buffer;
	int numBytes=0, len;
	struct sockaddr_un ipacmlog_socket;
	static unsigned int ipacm_log_sockfd = 0;

	if(ipacm_log_sockfd == 0)
	{
		/* start ipacm_log socket */
		if(create_socket(&ipacm_log_sockfd) < 0)
		{
			printf("unable to create ipacm_log socket\n");
			return;
		}
		printf("create ipacm_log socket successfully\n");
	}
	ipacmlog_socket.sun_family = AF_UNIX;
	strcpy(ipacmlog_socket.sun_path, IPACMLOG_FILE);
	len = strlen(ipacmlog_socket.sun_path) + sizeof(ipacmlog_socket.sun_family);

	memcpy(ipacm_log_buffer.user_data, user_data, MAX_BUF_LEN);

	//printf("send : %s\n", ipacm_log_buffer.user_data);
	if ((numBytes = sendto(ipacm_log_sockfd, (void *)&ipacm_log_buffer, sizeof(ipacm_log_buffer.user_data), 0,
			(struct sockaddr *)&ipacmlog_socket, len)) == -1)
	{
		printf("Send Failed(%d) %s \n",errno,strerror(errno));
		return;
	}
	return;
}
