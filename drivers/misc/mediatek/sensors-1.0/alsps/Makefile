# In case the platform does NOT support this type of sensors
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include
# guomingyi add
obj-$(CONFIG_SIMULATOR_PS_SUPPORT) += tp_simulator_ps/
obj-y   +=  alsps.o
obj-y   +=  alsps_factory.o
obj-y   +=  aal_control.o

obj-$(CONFIG_MTK_ALSPSHUB)      +=  alspshub/
obj-$(CONFIG_MTK_CM36558)	+=  cm36558/
obj-$(CONFIG_MTK_EPL259X)	+=  epl259x/
obj-$(CONFIG_MTK_LTR578)	+=  ltr578/
obj-$(CONFIG_MTK_EPL259X_P130)	+=  epl259x_p130/
