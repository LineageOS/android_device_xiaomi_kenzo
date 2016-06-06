LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_C_INCLUDES := $(LOCAL_PATH)/../src
LOCAL_C_INCLUDES += $(LOCAL_PATH)/../inc
LOCAL_C_INCLUDES += $(LOCAL_PATH)/../../ipanat/inc
ifeq ($(call is-platform-sdk-version-at-least,20),true)
LOCAL_C_INCLUDES += external/icu/icu4c/source/common
else
LOCAL_C_INCLUDES += external/icu4c/common
endif
LOCAL_C_INCLUDES += external/dhcpcd
LOCAL_C_INCLUDES += external/libxml2/include
LOCAL_C_INCLUDES += external/libnetfilter_conntrack/include
LOCAL_C_INCLUDES += external/libnfnetlink/include

LOCAL_C_INCLUDES += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include
LOCAL_ADDITIONAL_DEPENDENCIES := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr


LOCAL_CFLAGS := -v
LOCAL_CFLAGS += -DFEATURE_IPA_ANDROID
ifneq (,$(filter userdebug eng, $(TARGET_BUILD_VARIANT)))
LOCAL_CFLAGS += -DDEBUG
endif

ifeq ($(TARGET_BOARD_PLATFORM),msm8998)
LOCAL_CFLAGS += -DFEATURE_IPA_V3
endif

filetoadd = bionic/libc/kernel/arch-arm/asm/posix_types.h
LOCAL_CFLAGS += $(shell if [ -a $(filetoadd) ] ; then echo -include $(filetoadd) ; fi ;)
filetoadd = bionic/libc/kernel/arch-arm/asm/byteorder.h
LOCAL_CFLAGS += $(shell if [ -a $(filetoadd) ] ; then echo -include $(filetoadd) ; fi ;)

LOCAL_SRC_FILES := IPACM_Main.cpp \
		IPACM_EvtDispatcher.cpp \
		IPACM_Config.cpp \
		IPACM_CmdQueue.cpp \
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
		IPACM_Conntrack_NATApp.cpp\
		IPACM_ConntrackClient.cpp \
		IPACM_ConntrackListener.cpp \
                IPACM_Log.cpp

LOCAL_MODULE := ipacm
LOCAL_CLANG := false
LOCAL_MODULE_TAGS := optional

LOCAL_SHARED_LIBRARIES := libipanat
LOCAL_SHARED_LIBRARIES += libxml2
LOCAL_SHARED_LIBRARIES += libnfnetlink
LOCAL_SHARED_LIBRARIES += libnetfilter_conntrack
LOCAL_SHARED_LIBRARIES += libdhcpcd
include $(BUILD_EXECUTABLE)

################################################################################

define ADD_TEST

include $(CLEAR_VARS)
LOCAL_MODULE       := $1
LOCAL_SRC_FILES    := $1
LOCAL_MODULE_CLASS := ipacm
LOCAL_MODULE_TAGS  := debug
LOCAL_MODULE_PATH  := $(TARGET_OUT_ETC)
include $(BUILD_PREBUILT)

endef

include $(CLEAR_VARS)
LOCAL_MODULE := IPACM_cfg.xml
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_PATH := $(TARGET_OUT_ETC)
LOCAL_MODULE_TAGS := optional
LOCAL_SRC_FILES := $(LOCAL_MODULE)
LOCAL_MODULE_OWNER := ipacm
include $(BUILD_PREBUILT)
