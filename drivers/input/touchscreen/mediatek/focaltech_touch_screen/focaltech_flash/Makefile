#drivers/input/touchscreen/mediatek/focaltech_ts/focaltech_flash
#
#Makefile for test module
#
   
PROJECT := $(subst full_,,$(TARGET_PRODUCT))
ccflags-y += -I$(srctree)/tinno/$(PROJECT)/touchpanel/focaltech_touch_screen/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/$(CONFIG_TOUCHSCREEN_MTK_FTS_DIRECTORY)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
ccflags-y += -I$(srctree)/drivers/base

obj-$(CONFIG_MTK_FOCALTECH_TOUCH_TS) += focaltech_upgrade_ft5x46.o
obj-$(CONFIG_MTK_FOCALTECH_TOUCH_TS) += focaltech_upgrade_ft5822.o
obj-$(CONFIG_MTK_FOCALTECH_TOUCH_TS) += focaltech_upgrade_ft6336gu.o
obj-$(CONFIG_MTK_FOCALTECH_TOUCH_TS) += focaltech_upgrade_ft8006.o
obj-$(CONFIG_MTK_FOCALTECH_TOUCH_TS) += focaltech_upgrade_ft8606.o
obj-$(CONFIG_MTK_FOCALTECH_TOUCH_TS) += focaltech_upgrade_ft8607.o
obj-$(CONFIG_MTK_FOCALTECH_TOUCH_TS) += focaltech_upgrade_ft8716.o
obj-$(CONFIG_MTK_FOCALTECH_TOUCH_TS) += focaltech_upgrade_ft8736.o
obj-$(CONFIG_MTK_FOCALTECH_TOUCH_TS) += focaltech_upgrade_ft8006u.o
obj-$(CONFIG_MTK_FOCALTECH_TOUCH_TS) += focaltech_upgrade_ft5422u.o
