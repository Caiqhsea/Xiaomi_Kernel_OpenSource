# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_LC_CHARGER_MANAGER)    += charger_policy/
obj-$(CONFIG_LC_CHARGER_CLASS)      += charger_class/
obj-$(CONFIG_LC_CHARGER_IC)         += charger_ic/
obj-y                               += fuelgauge/
obj-y                               += battery_secret/

ifeq ($(CONFIG_FACTORY_BUILD), y)
KBUILD_CFLAGS += -DKERNEL_FACTORY_BUILD=1
else
subdir-ccflags-$(CONFIG_XM_SMART_CHG) += -DCONFIG_XIAOMI_SMART_CHG=1
endif

