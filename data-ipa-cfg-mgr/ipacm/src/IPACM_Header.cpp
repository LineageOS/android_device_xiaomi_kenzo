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
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>

#include "IPACM_Header.h"
#include "IPACM_Log.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////

//All interaction through the driver are made through this inode.
static const char *DEVICE_NAME = "/dev/ipa";

/////////////////////////////////////////////////////////////////////////////////////////////////////////

IPACM_Header::IPACM_Header()
{
	m_fd = open(DEVICE_NAME, O_RDWR);
	if (-1 == m_fd)
	{
		IPACMERR("Failed to open %s in IPACM_Header test application constructor.\n", DEVICE_NAME);
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

IPACM_Header::~IPACM_Header()
{
	if (-1 != m_fd)
	{
		close(m_fd);
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

bool IPACM_Header::DeviceNodeIsOpened()
{
	return (-1 != m_fd);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

bool IPACM_Header::AddHeader(struct ipa_ioc_add_hdr *pHeaderTableToAdd)
{
	int nRetVal = 0;
	//call the Driver ioctl in order to add header
	nRetVal = ioctl(m_fd, IPA_IOC_ADD_HDR, pHeaderTableToAdd);
	IPACMDBG("return value: %d\n", nRetVal);
	return (-1 != nRetVal);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

bool IPACM_Header::DeleteHeader(struct ipa_ioc_del_hdr *pHeaderTableToDelete)
{
	int nRetVal = 0;
	//call the Driver ioctl in order to remove header
	nRetVal = ioctl(m_fd, IPA_IOC_DEL_HDR, pHeaderTableToDelete);
	IPACMDBG("return value: %d\n", nRetVal);
	return (-1 != nRetVal);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

bool IPACM_Header::Commit()
{
	int nRetVal = 0;
	nRetVal = ioctl(m_fd, IPA_IOC_COMMIT_HDR);
	IPACMDBG("return value: %d\n", nRetVal);
	return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

bool IPACM_Header::Reset()
{
	int nRetVal = 0;

	nRetVal = ioctl(m_fd, IPA_IOC_RESET_HDR);
	nRetVal |= ioctl(m_fd, IPA_IOC_COMMIT_HDR);
	IPACMDBG("return value: %d\n", nRetVal);
	return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

bool IPACM_Header::GetHeaderHandle(struct ipa_ioc_get_hdr *pHeaderStruct)
{
	int retval = 0;

	if (!DeviceNodeIsOpened()) return false;

	retval = ioctl(m_fd, IPA_IOC_GET_HDR, pHeaderStruct);
	if (retval)
	{
		IPACMERR("IPA_IOC_GET_HDR ioctl failed, routingTable =0x%p, retval=0x%x.\n", pHeaderStruct, retval);
		return false;
	}

	IPACMDBG("IPA_IOC_GET_HDR ioctl issued to IPA header insertion block.\n");
	return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

bool IPACM_Header::CopyHeader(struct ipa_ioc_copy_hdr *pCopyHeaderStruct)
{
	int retval = 0;

	if (!DeviceNodeIsOpened()) return false;

	retval = ioctl(m_fd, IPA_IOC_COPY_HDR, pCopyHeaderStruct);
	if (retval)
	{
		IPACMERR("IPA_IOC_COPY_HDR ioctl failed, retval=0x%x.\n", retval);
		return false;
	}

	IPACMDBG("IPA_IOC_COPY_HDR ioctl issued to IPA header insertion block.\n");
	return true;
}

bool IPACM_Header::DeleteHeaderHdl(uint32_t hdr_hdl)
{
	const uint8_t NUM_HDLS = 1;
	struct ipa_ioc_del_hdr *pHeaderDescriptor = NULL;
	struct ipa_hdr_del *hd_rule_entry;
	int len = 0;
	bool res = true;

	if (hdr_hdl == 0)
	{
		IPACMERR("Invalid header handle passed. Ignoring it\n");
		return false;
	}

	len = (sizeof(struct ipa_ioc_del_hdr)) + (NUM_HDLS * sizeof(struct ipa_hdr_del));
	pHeaderDescriptor = (struct ipa_ioc_del_hdr *)malloc(len);
	if (pHeaderDescriptor == NULL)
	{
		IPACMERR("Unable to allocate memory for del header\n");
		return false;
	}

	memset(pHeaderDescriptor, 0, len);
	pHeaderDescriptor->commit = true;
	pHeaderDescriptor->num_hdls = NUM_HDLS;
	hd_rule_entry = &pHeaderDescriptor->hdl[0];

	hd_rule_entry->hdl = hdr_hdl;
	hd_rule_entry->status = -1;

	IPACMDBG("Deleting Header hdl:(%x)\n", hd_rule_entry->hdl);
	if ((false == DeleteHeader(pHeaderDescriptor)) ||
			(hd_rule_entry->status))
	{
	    IPACMERR("Header hdl:(%x) deletion failed!  status: %d\n", hd_rule_entry->hdl,hd_rule_entry->status);
		res = false;
		goto fail;
	}

	IPACMDBG_H("Deleted Header hdl:(%x) successfully\n", hd_rule_entry->hdl);

fail:
	free(pHeaderDescriptor);

	return res;

}

bool IPACM_Header::AddHeaderProcCtx(struct ipa_ioc_add_hdr_proc_ctx* pHeader)
{
	int ret = 0;
	//call the Driver ioctl to add header processing context
	ret = ioctl(m_fd, IPA_IOC_ADD_HDR_PROC_CTX, pHeader);
	return (ret == 0);
}

bool IPACM_Header::DeleteHeaderProcCtx(uint32_t hdl)
{
	int len, ret;
	struct ipa_ioc_del_hdr_proc_ctx* pHeaderTable = NULL;

	len = sizeof(struct ipa_ioc_del_hdr_proc_ctx) + sizeof(struct ipa_hdr_proc_ctx_del);
	pHeaderTable = (struct ipa_ioc_del_hdr_proc_ctx*)malloc(len);
	if(pHeaderTable == NULL)
	{
		IPACMERR("Failed to allocate buffer.\n");
		return false;
	}
	memset(pHeaderTable, 0, len);

	pHeaderTable->commit = 1;
	pHeaderTable->num_hdls = 1;
	pHeaderTable->hdl[0].hdl = hdl;

	ret = ioctl(m_fd, IPA_IOC_DEL_HDR_PROC_CTX, pHeaderTable);
	if(ret != 0)
	{
		IPACMERR("Failed to delete hdr proc ctx: return value %d, status %d\n",
			ret, pHeaderTable->hdl[0].status);
	}
	free(pHeaderTable);
	return (ret == 0);
}

