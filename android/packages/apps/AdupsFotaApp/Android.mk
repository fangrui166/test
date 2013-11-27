#adupsfota
ifeq ($(strip $(ADUPS_FOTA_SUPPORT)), true)
BASE_PATH := $(call my-dir)
include $(CLEAR_VARS)

include $(BASE_PATH)/AdupsFota/Android.mk
include $(BASE_PATH)/AdupsFotaReboot/Android.mk
endif
