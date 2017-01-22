LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
    qcamera_test.cpp \

LOCAL_SHARED_LIBRARIES:= \
    libdl \
    libui \
    libutils \
    libcutils \
    libbinder \
    libmedia \
    libui \
    libgui \
    libcamera_client \
    libskia \
    libstagefright \
    libstagefright_foundation \

ifneq (1,$(filter 1,$(shell echo "$$(( $(PLATFORM_SDK_VERSION) >= 18 ))" )))

LOCAL_SHARED_LIBRARIES += \
    libmedia_native \

LOCAL_32_BIT_ONLY := $(BOARD_QTI_CAMERA_32BIT_ONLY)
LOCAL_CFLAGS += -DUSE_JB_MR1

endif

LOCAL_C_INCLUDES += \
    frameworks/base/include/ui \
    frameworks/base/include/surfaceflinger \
    frameworks/base/include/camera \
    frameworks/base/include/media \
    external/skia/include/core \
    external/skia/include/images \
    $(TARGET_OUT_HEADERS)/qcom/display \
    device/xiaomi/kenzo/camera/QCamera2/stack/common \
    frameworks/av/include/media/stagefright \
    frameworks/native/include/media/openmax \
    $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include

LOCAL_ADDITIONAL_DEPENDENCIES := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr
LOCAL_MODULE:= camera_test
LOCAL_MODULE_TAGS:= tests

LOCAL_CFLAGS += -Wall -Wextra -Werror -Wno-unused-parameter
LOCAL_CFLAGS += -O0
LOCAL_CLANG := false

ifeq (1,$(filter 1,$(shell echo "$$(( $(PLATFORM_SDK_VERSION) >= 20 ))" )))

LOCAL_CFLAGS += -DUSE_SDK_20_OR_HIGHER

endif

ifeq (1,$(filter 1,$(shell echo "$$(( $(PLATFORM_SDK_VERSION) <= 22 ))" )))
LOCAL_CFLAGS += -DUSE_L_MR1
endif

include $(BUILD_EXECUTABLE)
