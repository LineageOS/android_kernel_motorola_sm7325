DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)


include $(CLEAR_VARS)
LOCAL_MODULE := pt.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk

include $(CLEAR_VARS)
LOCAL_MODULE :=pt_debug.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk

include $(CLEAR_VARS)
LOCAL_MODULE := pt_loader.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk

include $(CLEAR_VARS)
LOCAL_MODULE := pt_device_access.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk

include $(CLEAR_VARS)
LOCAL_MODULE := cypsoc_picoleaf.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)

include $(DLKM_DIR)/AndroidKernelModule.mk