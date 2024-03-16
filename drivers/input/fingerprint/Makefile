#
# Makefile for the fingerprint drivers.
#

ifeq ($(CONFIG_BBK_ET713),y)
et713-objs := et713_platform.o
obj-$(CONFIG_BBK_ET713) := et713.o
endif

ifeq ($(CONFIG_INPUT_FINGERPRINT_FPC1540),y)
obj-$(CONFIG_INPUT_FINGERPRINT_FPC1540) += fpc1020_platform_tee.o
endif
