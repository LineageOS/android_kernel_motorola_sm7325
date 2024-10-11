DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := mmi_parallel_charger_iio.ko
LOCAL_MODULE_TAGS := optional

ifeq ($(DLKM_INSTALL_TO_VENDOR_OUT),true)
LOCAL_MODULE_PATH := $(TARGET_OUT_VENDOR)/lib/modules/
else
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
endif
KBUILD_OPTIONS += MODULE_KERNEL_VERSION=$(TARGET_KERNEL_VERSION)
include $(DLKM_DIR)/AndroidKernelModule.mk
