USE_CAMERA_STUB := true

# inherit from the proprietary version
-include vendor/bn/nookcolor/BoardConfigVendor.mk

TARGET_NO_BOOTLOADER := true
TARGET_BOARD_PLATFORM := unknown
TARGET_CPU_ABI := armeabi
TARGET_BOOTLOADER_BOARD_NAME := nookcolor

BOARD_KERNEL_CMDLINE := no_console_suspend=1 console=null
BOARD_KERNEL_BASE := 0x20000000
BOARD_PAGE_SIZE := 0x00000800

# fix this up by examining /proc/mtd on a running device
##BOARD_BOOTIMAGE_PARTITION_SIZE := 0x00380000
##BOARD_RECOVERYIMAGE_PARTITION_SIZE := 0x00480000
##BOARD_SYSTEMIMAGE_PARTITION_SIZE := 0x08c60000
##BOARD_USERDATAIMAGE_PARTITION_SIZE := 0x105c0000
##BOARD_FLASH_BLOCK_SIZE := 131072

TARGET_PREBUILT_KERNEL := device/bn/nookcolor/kernel

# ignore boot and recovery
BOARD_RECOVERY_IGNORE_BOOTABLES := true

# custom recovery ui
BOARD_CUSTOM_RECOVERY_KEYMAPPING := ../../device/bn/nookcolor/recovery/recovery_ui.c

# we dont' have one of these
BOARD_HAS_NO_MISC_PARTITION := true

# partition scheme and options
BOARD_SYSTEM_DEVICE := /dev/block/mmcblk0p5
BOARD_SYSTEM_FILESYSTEM := ext2
BOARD_SYSTEM_FILESYSTEM_OPTIONS := nocheck
BOARD_DATA_DEVICE := /dev/block/mmcblk0p6
BOARD_DATA_FILESYSTEM := ext3
BOARD_DATA_FILESYSTEM_OPTIONS := nocheck,nosuid,nodev
BOARD_CACHE_DEVICE := /dev/block/mmcblk0p7
BOARD_CACHE_FILESYSTEM := ext3
BOARD_CACHE_FILESYSTEM_OPTIONS := nocheck,nosuid,nodev
BOARD_SDCARD_DEVICE_PRIMARY := /dev/block/mmcblk1p1
BOARD_SDCARD_DEVICE_SECONDARY := /dev/block/mmcblk1
BOARD_SDEXT_DEVICE := /dev/block/mmcblk1p2
