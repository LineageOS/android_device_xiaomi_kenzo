LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

# functions
LOC_RPCGEN_APIS_PATH := $(TARGET_OUT_INTERMEDIATES)/loc_api/libloc_api_rpcgen_intermediates
LOC_RPCGEN_APIS_PATH_FL := ../../../../../$(TARGET_OUT_INTERMEDIATES)/loc_api/libloc_api_rpcgen_intermediates

LOCAL_MODULE := libloc_api_rpcgen
LOCAL_MODULE_OWNER := qcom

LOCAL_SHARED_LIBRARIES := \
    librpc \
    libcommondefs

LOCAL_SRC_FILES += \
    src/loc_api_rpcgen_cb_xdr.c \
    src/loc_api_rpcgen_common_xdr.c \
    src/loc_api_rpcgen_cb_svc.c \
    src/loc_api_rpcgen_clnt.c \
    src/loc_api_rpcgen_xdr.c

LOCAL_C_INCLUDES += hardware/msm7k/librpc
LOCAL_C_INCLUDES += $(LOC_RPCGEN_APIS_PATH)/../../SHARED_LIBRARIES/libcommondefs_intermediates/inc
LOCAL_C_INCLUDES += $(LOCAL_PATH)/inc
LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/libcommondefs/rpcgen/inc

LOCAL_COPY_HEADERS_TO := loc_api/rpcgen/inc
LOCAL_COPY_HEADERS := inc/loc_api_rpcgen_rpc.h
LOCAL_COPY_HEADERS += inc/loc_api_rpcgen_common_rpc.h
LOCAL_COPY_HEADERS += inc/loc_api_rpcgen_cb_rpc.h
LOCAL_COPY_HEADERS += inc/loc_apicb_appinit.h

LOCAL_LDLIBS += -lpthread
LOCAL_PRELINK_MODULE := false
include $(BUILD_STATIC_LIBRARY)
