# charger_policy/Makefile
export KBUILD_EXTRA_SYMBOLS += $(BSP_MODULES_OUT)/battery_secret/Module.symvers

obj-$(CONFIG_LC_CHARGER_MANAGER)        += main_chg.o
main_chg-objs:= lc_charger_manager.o lc_jeita.o lc_charge_sysfs.o lc_charge_class_sysfs.o 
ifneq ($(CONFIG_FACTORY_BUILD), y)
main_chg-objs-$(CONFIG_XM_SMART_CHG)	+= xm_smart_chg.o
endif
main_chg-objs 	+= $(main_chg-objs-y)

obj-$(CONFIG_LC_CHARGER_MANAGER)        += ../common/lc_voter.o
obj-$(CONFIG_LC_CHARGER_MANAGER)        += ../common/lc_notify.o
obj-$(CONFIG_LC_CP_POLICY)              += lc_cp_policy.o
obj-$(CONFIG_LC_PD_POLICY)              += lc_pd_adapter.o
obj-$(CONFIG_PD_BATTERY_SECRET)             += xm_pd_adapter.o
obj-$(CONFIG_LC_QC_POLICY)              += lc_qc_policy.o
ifneq ($(CONFIG_FACTORY_BUILD), y)
obj-$(CONFIG_XM_DFX_CHARGER)              += xm_dfx_chg.o
endif
