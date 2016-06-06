AM_CPPFLAGS = -I./../inc \
	      -I$(top_srcdir)/ipanat/inc \
	      ${LIBXML_CFLAGS}
AM_CPPFLAGS += -Wall -Wundef -Wno-trigraphs
AM_CPPFLAGS	+= -DDEBUG -g -DFEATURE_ETH_BRIDGE_LE
AM_CPPFLAGS += -DFEATURE_IPA_V3

ipacm_SOURCES =	IPACM_Main.cpp \
		IPACM_Conntrack_NATApp.cpp\
		IPACM_ConntrackClient.cpp \
		IPACM_ConntrackListener.cpp \
		IPACM_EvtDispatcher.cpp \
		IPACM_Config.cpp \
		IPACM_CmdQueue.cpp \
		IPACM_Log.cpp \
		IPACM_Filtering.cpp \
		IPACM_Routing.cpp \
		IPACM_Header.cpp \
		IPACM_Lan.cpp \
		IPACM_Iface.cpp \
		IPACM_Wlan.cpp \
		IPACM_Wan.cpp \
		IPACM_IfaceManager.cpp \
		IPACM_Neighbor.cpp \
		IPACM_Netlink.cpp \
		IPACM_Xml.cpp \
		IPACM_LanToLan.cpp

bin_PROGRAMS  =  ipacm

requiredlibs =  ${LIBXML_LIB} -lxml2 -lpthread -lnetfilter_conntrack -lnfnetlink\
               ../../ipanat/src/libipanat.la

ipacm_LDADD =  $(requiredlibs)

AM_CPPFLAGS += "-std=c++0x"

LOCAL_MODULE := libipanat
LOCAL_PRELINK_MODULE := false
include $(BUILD_SHARED_LIBRARY)

etcdir = ${sysconfdir}
etc_SCRIPTS = IPACM_cfg.xml

init_ddir = ${sysconfdir}/init.d
init_d_SCRIPTS = start_ipacm_le
