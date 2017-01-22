common_deps :=
kernel_includes :=

ifeq ($(call is-vendor-board-platform,QCOM),true)
ifeq ($(TARGET_COMPILE_WITH_MSM_KERNEL),true)
    common_deps += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr
    kernel_includes += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include
endif
endif
