obj-$(CONFIG_SM_FUELGAUGE)				+= fg_sm5602.o
fg_sm5602-objs := sm5602_fg.o
ifeq ($(CONFIG_LC_ADAPTIVE_POWEROFF_VOLTAGE),m)
fg_sm5602-objs += lc_adaptive_poweroff_voltage.o
endif
obj-$(CONFIG_CHARGE_PARTITION)			+= charger_partition.o
