LOCAL_PATH := $(call my-dir)

#----------------------------------------------------------------------
# Compile (L)ittle (K)ernel bootloader and the nandwrite utility
#----------------------------------------------------------------------
ifneq ($(strip $(TARGET_NO_BOOTLOADER)),true)

# Compile
include bootable/bootloader/lk/AndroidBoot.mk

$(INSTALLED_BOOTLOADER_MODULE) : $(TARGET_BOOTLOADER) | $(ACP)
	$(transform-prebuilt-to-target)
$(BUILT_TARGET_FILES_PACKAGE): $(INSTALLED_BOOTLOADER_MODULE)

droidcore: $(INSTALLED_BOOTLOADER_MODULE)
# Copy nandwrite utility to target out directory
$(INSTALLED_NANDWRITE_TARGET) : $(TARGET_NANDWRITE) | $(ACP)
	$(transform-prebuilt-to-target)
endif

#----------------------------------------------------------------------
# Compile Linux Kernel
#----------------------------------------------------------------------
ifeq ($(KERNEL_DEFCONFIG),)
    KERNEL_DEFCONFIG := $(TARGET_DEVICE)_defconfig
endif

include kernel/AndroidKernel.mk

$(INSTALLED_KERNEL_TARGET) : $(TARGET_PREBUILT_KERNEL) | $(ACP)
	$(transform-prebuilt-to-target)

#----------------------------------------------------------------------
# Key mappings
#----------------------------------------------------------------------
include $(CLEAR_VARS)
LOCAL_SRC_FILES := surf_keypad.kcm
LOCAL_MODULE_TAGS := optional
include $(BUILD_KEY_CHAR_MAP)

include $(CLEAR_VARS)
LOCAL_MODULE := 7x27a_kp.kcm
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := ETC
LOCAL_SRC_FILES    := $(LOCAL_MODULE)
LOCAL_MODULE_PATH  := $(TARGET_OUT_KEYCHARS)
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE       := atmel_mxt_ts.kl
LOCAL_MODULE_TAGS  := optional
LOCAL_MODULE_CLASS := ETC
LOCAL_SRC_FILES    := $(LOCAL_MODULE)
LOCAL_MODULE_PATH  := $(TARGET_OUT_KEYLAYOUT)
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE       := ft5x06_ts.kl
LOCAL_MODULE_TAGS  := optional
LOCAL_MODULE_CLASS := ETC
LOCAL_SRC_FILES    := $(LOCAL_MODULE)
LOCAL_MODULE_PATH  := $(TARGET_OUT_KEYLAYOUT)
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE       := surf_keypad.kl
LOCAL_MODULE_TAGS  := optional
LOCAL_MODULE_CLASS := ETC
LOCAL_SRC_FILES    := $(LOCAL_MODULE)
LOCAL_MODULE_PATH  := $(TARGET_OUT_KEYLAYOUT)
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE       := 7k_handset.kl
LOCAL_MODULE_TAGS  := optional
LOCAL_MODULE_CLASS := ETC
LOCAL_SRC_FILES    := $(LOCAL_MODULE)
LOCAL_MODULE_PATH  := $(TARGET_OUT_KEYLAYOUT)
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE       := 7x27a_kp.kl
LOCAL_MODULE_TAGS  := optional
LOCAL_MODULE_CLASS := ETC
LOCAL_SRC_FILES    := $(LOCAL_MODULE)
LOCAL_MODULE_PATH  := $(TARGET_OUT_KEYLAYOUT)
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE       := vold.fstab
LOCAL_MODULE_TAGS  := optional
LOCAL_MODULE_CLASS := ETC
LOCAL_SRC_FILES    := $(LOCAL_MODULE)
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE       := vold.emmc.fstab
LOCAL_MODULE_TAGS  := optional
LOCAL_MODULE_CLASS := ETC
LOCAL_SRC_FILES    := $(LOCAL_MODULE)
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE       := fstab.msm7627a
LOCAL_MODULE_TAGS  := optional
LOCAL_MODULE_CLASS := ETC
LOCAL_SRC_FILES    := $(LOCAL_MODULE)
LOCAL_MODULE_PATH  := $(TARGET_ROOT_OUT)
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE       := fstab.nand.msm7627a
LOCAL_MODULE_TAGS  := optional
LOCAL_MODULE_CLASS := ETC
LOCAL_SRC_FILES    := $(LOCAL_MODULE)
LOCAL_MODULE_PATH  := $(TARGET_ROOT_OUT)
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE       := fstab.qcom
LOCAL_MODULE_TAGS  := optional
LOCAL_MODULE_CLASS := ETC
LOCAL_SRC_FILES    := $(LOCAL_MODULE)
LOCAL_MODULE_PATH  := $(TARGET_ROOT_OUT)
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE       := init.target.rc
LOCAL_MODULE_TAGS  := optional
LOCAL_MODULE_CLASS := ETC
LOCAL_SRC_FILES    := $(LOCAL_MODULE)
LOCAL_MODULE_PATH  := $(TARGET_ROOT_OUT)
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE       := init.qcom.ril.path.sh
LOCAL_MODULE_TAGS  := optional
LOCAL_MODULE_CLASS := ETC
LOCAL_SRC_FILES    := $(LOCAL_MODULE)
LOCAL_MODULE_PATH  := $(TARGET_ROOT_OUT)
include $(BUILD_PREBUILT)


include $(CLEAR_VARS)
LOCAL_MODULE       := init.qcom.composition_type.sh
LOCAL_MODULE_TAGS  := optional
LOCAL_MODULE_CLASS := ETC
LOCAL_SRC_FILES    := $(LOCAL_MODULE)
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE       := init.target.8x25.sh
LOCAL_MODULE_TAGS  := optional
LOCAL_MODULE_CLASS := ETC
LOCAL_SRC_FILES    := $(LOCAL_MODULE)
include $(BUILD_PREBUILT)


include $(CLEAR_VARS)
LOCAL_MODULE       := init.qcom.thermald_conf.sh
LOCAL_MODULE_TAGS  := optional
LOCAL_MODULE_CLASS := ETC
LOCAL_SRC_FILES    := $(LOCAL_MODULE)
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE       := init.qcom.set_dpi.sh
LOCAL_MODULE_TAGS  := optional
LOCAL_MODULE_CLASS := ETC
LOCAL_SRC_FILES    := $(LOCAL_MODULE)
include $(BUILD_PREBUILT)

ifeq ($(findstring true,$(BOARD_HAS_ATH_WLAN) $(BOARD_HAS_QCOM_WLAN)),true)
include $(CLEAR_VARS)
LOCAL_MODULE       := wpa_supplicant.conf
LOCAL_MODULE_TAGS  := optional
LOCAL_MODULE_CLASS := ETC
LOCAL_SRC_FILES    := $(LOCAL_MODULE)
LOCAL_MODULE_PATH  := $(TARGET_OUT_ETC)/wifi
include $(BUILD_PREBUILT)
endif

ifeq ($(strip $(BOARD_HAS_QCOM_WLAN)),true)
#include $(CLEAR_VARS)
#LOCAL_MODULE       := hostapd_default.conf
#LOCAL_MODULE_TAGS  := optional
#LOCAL_MODULE_CLASS := ETC
#LOCAL_MODULE_PATH  := $(TARGET_OUT_PERSIST)/qcom/softap
#LOCAL_SRC_FILES    := hostapd.conf
#include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE       := hostapd.conf
LOCAL_MODULE_TAGS  := optional
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_PATH  := $(TARGET_OUT_DATA)/misc/wifi
LOCAL_SRC_FILES    := hostapd.conf
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE       := hostapd.accept
LOCAL_MODULE_TAGS  := optional
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_PATH  := $(TARGET_OUT_DATA)/hostapd
LOCAL_SRC_FILES    := hostapd.accept
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE       := hostapd.deny
LOCAL_MODULE_TAGS  := optional
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_PATH  := $(TARGET_OUT_DATA)/hostapd
LOCAL_SRC_FILES    := hostapd.deny
include $(BUILD_PREBUILT)

endif
#----------------------------------------------------------------------
# Radio image
#----------------------------------------------------------------------
ifeq ($(ADD_RADIO_FILES), true)
radio_dir := $(PRODUCT_OUT)/radio
ifeq ($(ADUPS_FOTA_SUPPORT), true)
MP_PATH := $(LOCAL_PATH)/../../../modem_images

$(shell mkdir -p $(radio_dir))
$(shell cat $(MP_PATH)/oemsblhd.mbn > $(radio_dir)/OEMSBL.enc;cat $(MP_PATH)/oemsblhdtail.mbn >> $(radio_dir)/OEMSBL.enc;cat $(MP_PATH)/oemsbl.mbn >> $(radio_dir)/OEMSBL.enc)
$(shell cat $(PRODUCT_OUT)/emmc_appsboothd.mbn > $(radio_dir)/APPSBOOT.enc;cat $(MP_PATH)/emmc_appsboothdtail.mbn >> $(radio_dir)/APPSBOOT.enc;cat $(PRODUCT_OUT)/emmc_appsboot.mbn > $(radio_dir)/APPSBOOT.enc)
$(shell cat $(MP_PATH)/fat.bin > $(radio_dir)/fat.enc)
$(shell cat $(MP_PATH)/qcsblhd_cfgdata.mbn > $(radio_dir)/QCSBLHD.enc)
$(shell cat $(MP_PATH)/qcsbl.mbn > $(radio_dir)/QCSBL.enc)
else
MP_PATH := $(PRODUCT_OUT)/modem_images

$(shell mkdir -p $(radio_dir))
$(shell mkdir -p $(MP_PATH))
$(shell rar x -y -inul $(RELEASE_DIR_MODEM)/$(BUILD_NUMBER_MODEM)/*.rar $(MP_PATH))
$(shell cat $(MP_PATH)/qcsblhd_cfgdata.mbn > $(radio_dir)/QCSBLHD.enc)
$(shell cat $(MP_PATH)/qcsbl.mbn > $(radio_dir)/QCSBL.enc)
$(shell cat $(MP_PATH)/oemsblhd.mbn > $(radio_dir)/OEMSBL.enc;cat $(MP_PATH)/oemsbl.mbn >> $(radio_dir)/OEMSBL.enc)
$(shell cat $(MP_PATH)/amsshd.mbn > $(radio_dir)/AMSS.enc;cat $(MP_PATH)/amss.mbn >> $(radio_dir)/AMSS.enc)
$(shell echo SSRSKMLY > $(radio_dir)/build_version.enc)
endif
RADIO_FILES := $(shell cd $(radio_dir) ; find . -iname '*.ENC')
$(foreach f, $(RADIO_FILES), \
    $(call add-radio-file,radio/$(f)))
endif

#----------------------------------------------------------------------
# extra images
#----------------------------------------------------------------------
include device/qcom/common/generate_extra_images.mk
#----------------------------------------------------------------------
# dmesg dump cmm file
#----------------------------------------------------------------------

DMESG_DUMP := $(PRODUCT_OUT)/dmesg_dump.cmm
droidcore : $(DMESG_DUMP)
$(DMESG_DUMP) : kernel
	echo "d.save.binary * " \
    $(shell echo $$[0x$(shell cat $(PRODUCT_OUT)/obj/KERNEL_OBJ/System.map|grep __log_buf|awk '{print $$1}') - 0xc0000000 + 0x200000]|awk '{printf "0x%x",$$0}')"++0x20000" > $(DMESG_DUMP)
