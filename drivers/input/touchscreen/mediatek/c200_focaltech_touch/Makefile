# Linux driver folder
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/$(CONFIG_TOUCHSCREEN_MTK_FTS_DIRECTORY)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
ccflags-y += -I$(srctree)/drivers/base
ccflags-y += -I$(srctree)/drivers/misc/mediatek/hwmon/include

obj-$(CONFIG_MTK_FOCALTECH_TOUCH_C200)    +=  focaltech_ex_fun.o
obj-$(CONFIG_MTK_FOCALTECH_TOUCH_C200)    +=  focaltech_flash.o
obj-$(CONFIG_MTK_FOCALTECH_TOUCH_C200)    +=  focaltech_gesture.o
obj-$(CONFIG_MTK_FOCALTECH_TOUCH_C200)    +=  focaltech_core.o
obj-$(CONFIG_MTK_FOCALTECH_TOUCH_C200)    +=  focaltech_esdcheck.o
obj-$(CONFIG_MTK_FOCALTECH_TOUCH_C200)    +=  focaltech_sensor.o
obj-$(CONFIG_MTK_FOCALTECH_TOUCH_C200)    +=  focaltech_ex_mode.o
obj-$(CONFIG_MTK_FOCALTECH_TOUCH_C200)    +=  focaltech_proximity.o
obj-$(CONFIG_MTK_FOCALTECH_TOUCH_C200)    +=  focaltech_i2c.o
obj-$(CONFIG_MTK_FOCALTECH_TOUCH_C200)    +=  focaltech_point_report_check.o
obj-$(CONFIG_MTK_FOCALTECH_TOUCH_C200) +=     focaltech_flash/
#obj-$(CONFIG_MTK_FOCALTECH_TOUCH_C200) +=     focaltech_test/
