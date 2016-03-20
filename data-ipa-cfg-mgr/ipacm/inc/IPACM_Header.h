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

/*
 * IPACM_Header.h
 *
 *  Created on: Jun 20, 2012
 *      Author: tatias
 */

//////////////////////////////////////////////////////////////////////////////////

#ifndef IPACM_HEADER_H
#define IPACM_HEADER_H

#include <stdint.h>
#include "linux/msm_ipa.h"

//////////////////////////////////////////////////////////////////////////////////

class IPACM_Header
{
private:
	int m_fd;
public:
	bool AddHeader(struct ipa_ioc_add_hdr   *pHeaderTable);
	bool DeleteHeader(struct ipa_ioc_del_hdr *pHeaderTable);
	bool GetHeaderHandle(struct ipa_ioc_get_hdr *pHeaderStruct);
	bool CopyHeader(struct ipa_ioc_copy_hdr *pCopyHeaderStruct);
	bool Commit();
	bool Reset();
	bool DeleteHeaderHdl(uint32_t hdr_hdl);
	bool AddHeaderProcCtx(struct ipa_ioc_add_hdr_proc_ctx* pHeader);
	bool DeleteHeaderProcCtx(uint32_t hdl);

	IPACM_Header();
	~IPACM_Header();
	bool DeviceNodeIsOpened();
};


#endif


