#
# Makefile for lge touch core driver
#

# Each configuration option enables a list of files.

ifdef CONFIG_LGE_TOUCH_CORE_MTK
	subdir-ccflags-y += -Werror
	subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
	subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
endif

# Touch Core Driver Common Header Code..!!
ccflags-y += -I$(srctree)/drivers/input/touchscreen/lge_sub/

obj-$(CONFIG_LGE_TOUCH_CORE_BASE)	+= touch_sub_core.o
obj-$(CONFIG_LGE_TOUCH_CORE_BASE)	+= touch_sub_core_sysfs.o
obj-$(CONFIG_LGE_TOUCH_CORE_BASE)	+= touch_sub_common.o
obj-$(CONFIG_LGE_TOUCH_CORE_BASE)	+= touch_sub_i2c.o
obj-$(CONFIG_LGE_TOUCH_CORE_BASE)	+= touch_sub_spi.o
obj-$(CONFIG_LGE_TOUCH_CORE_BASE)	+= touch_sub_model.o
obj-$(CONFIG_LGE_TOUCH_CORE_BASE)	+= touch_sub_notify.o

obj-$(CONFIG_LGE_TOUCH_CORE_QCT)	+= touch_sub_hwif.o
obj-$(CONFIG_LGE_TOUCH_CORE_MTK)	+= touch_sub_hwif_mtk.o
obj-$(CONFIG_LGE_TOUCH_SUB_PEN)		+= touch_sub_pen.o
obj-$(CONFIG_LGE_TOUCH_SUB_DEX)		+= touch_sub_dex.o
obj-$(CONFIG_LGE_TOUCH_CORE_BASE)	+= lgsic/
