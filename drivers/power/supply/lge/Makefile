ifeq ($(CONFIG_LGE_PM_ACTM_V2), y)
obj-$(CONFIG_LGE_PM_ACTM_V2) += adaptive-charging-thermal-v2.o
else
obj-$(CONFIG_LGE_PM_ACTM) += adaptive-charging-thermal.o
endif
obj-y		+= charging-ceiling.o
obj-y		+= protection-battemp.o
obj-y		+= protection-batvolt.o
obj-y		+= unified-bootmode.o
obj-y		+= unified-nodes.o
obj-y		+= unified-sysfs.o
obj-y		+= veneer-psy.o
obj-y		+= veneer-uevent.o
obj-y		+= veneer-voter.o
ifneq ($(CONFIG_LGE_PM_CCD), y)
obj-y		+= charging-time-v3.o
obj-y		+= protection-showcase.o
obj-y		+= protection-usbio.o
endif
obj-$(CONFIG_LGE_PM_QNOVO_QNS) += qns_system.o
obj-$(CONFIG_LGE_PM_QNOVO_QNS_V2) += qns_system_v2.o