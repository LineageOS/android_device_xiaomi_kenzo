BOARD_PLATFORM_LIST := msm8916
BOARD_PLATFORM_LIST += msm8909
ifneq ($(call is-board-platform-in-list,$(BOARD_PLATFORM_LIST)),true)
ifneq (,$(filter $(QCOM_BOARD_PLATFORMS),$(TARGET_BOARD_PLATFORM)))
ifneq (, $(filter aarch64 arm arm64, $(TARGET_ARCH)))

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_C_INCLUDES := $(LOCAL_PATH)/
LOCAL_C_INCLUDES += $(LOCAL_PATH)/../../ipanat/inc

LOCAL_C_INCLUDES += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include
LOCAL_ADDITIONAL_DEPENDENCIES := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr

LOCAL_MODULE := ipa_nat_test
LOCAL_SRC_FILES := ipa_nat_test000.c \
		ipa_nat_test001.c \
		ipa_nat_test002.c \
		ipa_nat_test003.c \
		ipa_nat_test004.c \
		ipa_nat_test005.c \
		ipa_nat_test006.c \
		ipa_nat_test007.c \
		ipa_nat_test008.c \
		ipa_nat_test009.c \
		ipa_nat_test010.c \
		ipa_nat_test011.c \
		ipa_nat_test012.c \
		ipa_nat_test013.c \
		ipa_nat_test014.c \
		ipa_nat_test015.c \
		ipa_nat_test016.c \
		ipa_nat_test017.c \
		ipa_nat_test018.c \
		ipa_nat_test019.c \
		ipa_nat_test020.c \
		ipa_nat_test021.c \
		ipa_nat_test022.c \
		main.c


LOCAL_SHARED_LIBRARIES := libipanat

LOCAL_MODULE_TAGS := debug
LOCAL_MODULE_PATH := $(TARGET_OUT_DATA)/kernel-tests/ip_accelerator

include $(BUILD_EXECUTABLE)

endif # $(TARGET_ARCH)
endif
endif