// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2023 Xiaomi Inc.
 * Author Tianye<tianye9@xiaomi.com>
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/printk.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/kthread.h>
// #include <linux/power/sprd_charger_manager.h>
#include "xm_dfx_chg.h"
#include "lc_charger_manager.h"
#include "../battery_secret/battery_secret_class.h"
#include "../fuelgauge/sm5602_fg.h"

#define DFX_CHARGE_WORK_TIME        5000  // 5s
#define DFX_FAST_CHARGE_WORK_TIME   1000  // 1s
#define DFX_DISCHARGE_WORK_TIME     10000 // 10s
#define DELTA_REPORT_NS      (300 * 1000) // 5min

#define IS_BETWEEN(val, lval, rval)	((val >= lval) ? ((val <= rval) ? true :  false) : \
								((val >= rval) ? true : false))
#if IS_ENABLED(CONFIG_MIEV)
#include "miev/mievent.h"
#include <linux/string.h>
#endif

struct xm_dfs_info *dfs_info = NULL;
#define INVALID_VALUE 		0xFFFF

static struct device *xm_dfx_chg_dev = NULL;
static struct dfx_data_struct dfx_data;
static struct dfx_data_struct *dfx_data_p = &dfx_data;

static const char *const xm_dfx_chg_report_text[][2] = { \
	{"DEFAULT_NAME", "DEFAULT_TEXT"}, \
	{"chgErrInfo", "PdAuthFail"}, \
	{"chgErrInfo", "CpEnFail"}, \
	{"chgErrInfo", "NoneStandartChg"}, \
	{"chgErrInfo", "CorrosionDischarge"}, \
	{"chgErrInfo", "LpdDetected"}, \
	{"chgErrInfo", "CpVbusOvp"}, \
	{"chgErrInfo", "CpIbusOcp"}, \
	{"chgErrInfo", "CpVbatOvp"}, \
	{"chgErrInfo", "CpIbatOcp"}, \
	{"chgStatInfo", "chgBattCycle"}, \
	{"chgErrInfo", "SocNotFull"}, \
	{"chgStatInfo", "SmartEnduraTrig"}, \
	{"chgStatInfo", "SmartNaviTrig"}, \
	{"chgErrInfo", "FgI2cErr"}, \
	{"chgErrInfo", "CpI2cErr"}, \
	{"chgErrInfo", "BattLinkerAbsent"}, \
	{"chgErrInfo", "NotChgInLowTemp"}, \
	{"chgErrInfo", "NotChgInHighTemp"}, \
	{"chgErrInfo", "SmartEnduraSocErr"}, \
	{"chgErrInfo", "SmartNaviSocErr"}, \
	{"chgErrInfo", "BattAuthFail"}, \
	{"chgStatInfo", "TbatHot"}, \
	{"chgStatInfo", "TbatCold"},
};

static struct xm_dfs_evt_condition dfs_evt_conds[] = {
	{ .evt = CHG_DFX_PD_AUTH_ERR, // not use
	  .is_first_report = 1,
	  .support_boot_report = false,
	  .require_charge_stat = true,
	  .max_report_times = 1,
	  .report_times = 0 },
	{ .evt = CHG_DFX_CP_ENABLE_FAIL, // not use
	  .is_first_report = 1,
	  .support_boot_report = false,
	  .require_charge_stat = true,
	  .max_report_times = 3,
	  .report_times = 0 },
	{ .evt = CHG_DFX_NONE_STANDARD_CHG,
	  .is_first_report = 1,
	  .support_boot_report = false,
	  .require_charge_stat = true,
	  .max_report_times = 1,
	  .report_times = 0 },
	{ .evt = CHG_DFX_RP_SHORT_VBUS_DETECTED, // not use
	  .is_first_report = 1,
	  .support_boot_report = false,
	  .require_charge_stat = true,
	  .max_report_times = 1,
	  .report_times = 0 },
	{ .evt = CHG_DFX_LPD_DETECTED, // not use
	  .is_first_report = 1,
	  .support_boot_report = false,
	  .require_charge_stat = true,
	  .max_report_times = 1,
	  .report_times = 0 },
	{ .evt = CHG_DFX_CP_VBUS_OVP, // not use
	  .is_first_report = 1,
	  .support_boot_report = true,
	  .require_charge_stat = true,
	  .max_report_times = 3,
	  .report_times = 0	},
	{ .evt = CHG_DFX_CP_IBUS_OCP, // not use
	  .is_first_report = 1,
	  .support_boot_report = false,
	  .require_charge_stat = true,
	  .max_report_times = 3,
	  .report_times = 0	},
	{ .evt = CHG_DFX_CP_VBAT_OVP, // not use
	  .is_first_report = 1,
	  .support_boot_report = false,
	  .require_charge_stat = true,
	  .max_report_times = 3,
	  .report_times = 0	},
	{ .evt = CHG_DFX_CP_IBAT_OCP, // not use
	  .is_first_report = 1,
	  .support_boot_report = false,
	  .require_charge_stat = true,
	  .max_report_times = 3,
	  .report_times = 0	},
	{ .evt = CHG_DFX_BATT_CYCLE_COUNT,
	  .is_first_report = 1,
	  .support_boot_report = false,
	  .require_charge_stat = false,
	  .max_report_times = 1,
	  .report_times = 0 },
	{ .evt = CHG_DFX_UISOC_NOT_FULL,
	  .is_first_report = 1,
	  .support_boot_report = false,
	  .require_charge_stat = true,
	  .max_report_times = 1,
	  .report_times = 0 },
	{ .evt = CHG_DFX_SMART_ENDURANCE_TRIGGERED,
	  .is_first_report = 1,
	  .support_boot_report = false,
	  .require_charge_stat = true,
	  .max_report_times = 1,
	  .report_times = 0 },
	{ .evt = CHG_DFX_NAVIGATION,
	  .is_first_report = 1,
	  .support_boot_report = false,
	  .require_charge_stat = true,
	  .max_report_times = 1,
	  .report_times = 0 },
	{ .evt = CHG_DFX_FG_IIC_ERR, // not use
	  .is_first_report = 1,
	  .support_boot_report = false,
	  .require_charge_stat = true,
	  .max_report_times = 1,
	  .report_times = 0 },
	{ .evt = CHG_DFX_CP_ABSENT, // not use
	  .is_first_report = 1,
	  .support_boot_report = true,
	  .require_charge_stat = true,
	  .max_report_times = 1,
	  .report_times = 0 },
	{ .evt = CHG_DFX_BATT_LINKER_ABSENT,
	  .is_first_report = 1,
	  .support_boot_report = false,
	  .require_charge_stat = true,
	  .max_report_times = 1,
	  .report_times = 0 },
	{ .evt = CHG_DFX_LOW_TEMP_DISCHARGING,
	  .is_first_report = 1,
	  .support_boot_report = true,
	  .require_charge_stat = true,
	  .max_report_times = 3,
	  .report_times = 0 },
	{ .evt = CHG_DFX_HIGH_TEMP_DISCHARGING,
	  .is_first_report = 1,
	  .support_boot_report = true,
	  .require_charge_stat = true,
	  .max_report_times = 3,
	  .report_times = 0 },
	{ .evt = CHG_DFX_SMART_ENDURANCE_SOC_ERR,
	  .is_first_report = 1,
	  .support_boot_report = true,
	  .require_charge_stat = true,
	  .max_report_times = 1,
	  .report_times = 0 },
	{ .evt = CHG_DFX_NAVIGATION_OVER_SOC,
	  .is_first_report = 1,
	  .support_boot_report = true,
	  .require_charge_stat = true,
	  .max_report_times = 1,
	  .report_times = 0 },
	{ .evt = CHG_DFX_BATT_AUTH_ERR,
	  .is_first_report = 1,
	  .support_boot_report = false,
	  .require_charge_stat = true,
	  .max_report_times = 1,
	  .report_times = 0	},
	{ .evt = CHG_DFX_BATTERY_TEMP_HIGH,
	  .is_first_report = 1,
	  .support_boot_report = true,
	  .require_charge_stat = false,
	  .max_report_times = 1000,
	  .report_times = 0 },
	{ .evt = CHG_DFX_BATTERY_TEMP_LOW,
	  .is_first_report = 1,
	  .support_boot_report = true,
	  .require_charge_stat = false,
	  .max_report_times = 1000,
	  .report_times = 0 },
};

void disable_not_support_evt(struct xm_dfs_info *info)
{
	clear_bit(CHG_DFX_PD_AUTH_ERR, &info->evt_en_mask);
	clear_bit(CHG_DFX_CP_ENABLE_FAIL, &info->evt_en_mask);
	clear_bit(CHG_DFX_RP_SHORT_VBUS_DETECTED, &info->evt_en_mask);
	clear_bit(CHG_DFX_LPD_DETECTED, &info->evt_en_mask);
	clear_bit(CHG_DFX_CP_VBUS_OVP, &info->evt_en_mask);
	clear_bit(CHG_DFX_CP_IBUS_OCP, &info->evt_en_mask);
	clear_bit(CHG_DFX_CP_VBAT_OVP, &info->evt_en_mask);
	clear_bit(CHG_DFX_CP_IBAT_OCP, &info->evt_en_mask);
	clear_bit(CHG_DFX_CP_ABSENT, &info->evt_en_mask);
	return;
}

static int get_charger_online(int *charging)
{
	int ret = 0;
	const char *chg_psy_name = "usb";
	struct power_supply *chg_psy = NULL;
	union power_supply_propval val;
	chg_psy = power_supply_get_by_name(chg_psy_name);
	if (!chg_psy) {
		dfx_err("get %s fail! \n", chg_psy_name);
		return -EINVAL;
	}
	ret = power_supply_get_property(chg_psy, POWER_SUPPLY_PROP_ONLINE, &val);
	if (!ret) {
		*charging = !!val.intval;
	}
	return ret;
}

void mievent_upload(int miev_code, char *miev_param, ...)
{
#if IS_ENABLED(CONFIG_MIEV)
	va_list arg;
	char buffer[128] = {
		0
	}; //miev_param length should be less than 128 bytes.
	char *p1, *p2, *key, *value;
	int intval;
	struct misight_mievent *event = cdev_tevent_alloc(miev_code);
	va_start(arg, miev_param);
	memcpy(buffer, miev_param, strlen(miev_param));
	p1 = buffer;
	while (p1 && *p1 != '\0') {
		p2 = strsep(&p1, ",");
		while (p2 && *p2 != '\0') {
			key = strsep(&p2, ":");
			if (key) {
				dfx_err("[CHG_DFX] key:%s\n", key);
			} else {
				dfx_err("[CHG_DFX] none key\n");
				key = "None";
			}
			value = strsep(&p2, ":");
			if (value) {
				dfx_err("[CHG_DFX] value:%s\n", value);
			} else {
				dfx_err("[CHG_DFX] none value\n");
				value = "None";
			}
		}
		dfx_info("code:%d key:%s value:%s \n", miev_code, key, value);
		if (kstrtoint(value, 10, &intval) != 0) {
			cdev_tevent_add_str(event, key, value);
		} else {
			cdev_tevent_add_int(event, key, intval);
		}
	}
	cdev_tevent_write(event);
	cdev_tevent_destroy(event);
#endif
	return;
}

#define DATA_LEN_MAX 256
void xm_handle_dfx_report(u8 type, bool flag)
{
	char data[DATA_LEN_MAX] = { 0 };
	int len = 0;
	if (type < CHG_DFX_MAX_INDEX) {
		dfx_info("CHG:DFX: report %s\n",
			xm_dfx_chg_report_text[type][1]);
		len += scnprintf((data + len), (DATA_LEN_MAX - len), "%s:%s",
				 xm_dfx_chg_report_text[type][0],
				 xm_dfx_chg_report_text[type][1]);
	} else {
		dfx_err("CHG:DFX: unknown type to report\n");
		return;
	}
	switch (type) {
	case CHG_DFX_NONE_STANDARD_CHG:
		mievent_upload(DFX_ID_CHG_NONE_STANDARD_CHG, data);
		break;
	case CHG_DFX_BATT_CYCLE_COUNT:
		len += scnprintf((data + len), (DATA_LEN_MAX - len), ",%s:%d",
				 "cycleCnt", dfx_data_p->data_batt.cycle);
		mievent_upload(DFX_ID_CHG_BATTERY_CYCLECOUNT, data);
		break;
	case CHG_DFX_UISOC_NOT_FULL:
		len += scnprintf((data + len), (DATA_LEN_MAX - len), ",%s:%d",
				 "vbat", dfx_data_p->data_batt.vbat);
		len += scnprintf((data + len), (DATA_LEN_MAX - len), ",%s:%d",
				 "soc", dfx_data_p->data_batt.uisoc);
		len += scnprintf((data + len), (DATA_LEN_MAX - len), ",%s:%d",
				 "rsoc", dfx_data_p->data_batt.rawsoc);
		mievent_upload(DFX_ID_CHG_UISOC_NOT_FULL, data);
		break;
	case CHG_DFX_SMART_ENDURANCE_TRIGGERED:
		if (flag) {
			len += scnprintf((data + len), (DATA_LEN_MAX - len), ",%s:%d",
				 "soc", dfx_data_p->data_batt.uisoc);
		}
		mievent_upload(DFX_ID_CHG_SMART_ENDURANCE_TRIGGERED, data);
		break;
	case CHG_DFX_NAVIGATION:
		if (flag) {
			len += scnprintf((data + len), (DATA_LEN_MAX - len), ",%s:%d",
				 "soc", dfx_data_p->data_batt.uisoc);
		}
		mievent_upload(DFX_ID_CHG_SMART_NAVIGATION_TRIGGERED, data);
		break;
	case CHG_DFX_FG_IIC_ERR:
		len += scnprintf((data + len), (DATA_LEN_MAX - len), ",%s:%d",
				 "soc", dfx_data_p->data_batt.uisoc);
		mievent_upload(DFX_ID_CHG_BATT_IIC_ERR, data);
		break;
	case CHG_DFX_BATT_LINKER_ABSENT:
		mievent_upload(DFX_ID_CHG_BATT_LINKER_ABSENT, data);
		break;
	case CHG_DFX_BATT_AUTH_ERR:
		mievent_upload(DFX_ID_CHG_BATTERY_AUTH_FAIL, data);
		break;
	case CHG_DFX_LOW_TEMP_DISCHARGING:
		len += scnprintf((data + len), (DATA_LEN_MAX - len), ",%s:%d",
				 "tbat", dfx_data_p->data_batt.tbat_x10);
		mievent_upload(DFX_ID_CHG_LOW_TEMP_DISCHARGING, data);
		break;
	case CHG_DFX_HIGH_TEMP_DISCHARGING:
		len += scnprintf((data + len), (DATA_LEN_MAX - len), ",%s:%d",
				 "tbat", dfx_data_p->data_batt.tbat_x10);
		mievent_upload(DFX_ID_CHG_HIGH_TEMP_DISCHARGING, data);
		break;
	case CHG_DFX_SMART_ENDURANCE_SOC_ERR:
		len += scnprintf((data + len), (DATA_LEN_MAX - len), ",%s:%d",
				 "soc", dfx_data_p->data_batt.uisoc);
		mievent_upload(DFX_ID_CHG_SMART_ENDURANCE_SOC_ERR, data);
		break;
	case CHG_DFX_NAVIGATION_OVER_SOC:
		len += scnprintf((data + len), (DATA_LEN_MAX - len), ",%s:%d",
				 "soc", dfx_data_p->data_batt.uisoc);
		mievent_upload(DFX_ID_SMART_NAVIGATION_SOC_ERR, data);
		break;
	case CHG_DFX_BATTERY_TEMP_HIGH:
		len += scnprintf((data + len), (DATA_LEN_MAX - len), ",%s:%d",
				 "tbat", dfx_data_p->data_batt.tbat_x10);
		len += scnprintf((data + len), (DATA_LEN_MAX - len), ",%s:%d",
				 "tbatMax",
				 dfx_data_p->data_batt.tbat_max_x10 / 10);
		len += scnprintf((data + len), (DATA_LEN_MAX - len), ",%s:%d",
				 "isCharging",
				 dfx_data_p->data_batt.is_charging);
		len += scnprintf((data + len), (DATA_LEN_MAX - len), ",%s:%d",
				 "tboard", dfx_data_p->tboard_x1000);
		mievent_upload(DFX_ID_CHG_BATTERY_TEMP_HOT, data);
		break;
	case CHG_DFX_BATTERY_TEMP_LOW:
		len += scnprintf((data + len), (DATA_LEN_MAX - len), ",%s:%d",
				 "tbat", dfx_data_p->data_batt.tbat_x10);
		len += scnprintf((data + len), (DATA_LEN_MAX - len), ",%s:%d",
				 "tbatMin",
				 dfx_data_p->data_batt.tbat_min_x10 / 10);
		len += scnprintf((data + len), (DATA_LEN_MAX - len), ",%s:%d",
				 "isCharging",
				 dfx_data_p->data_batt.is_charging);
		len += scnprintf((data + len), (DATA_LEN_MAX - len), ",%s:%d",
				 "tboard", dfx_data_p->tboard_x1000);
		mievent_upload(DFX_ID_CHG_BATTERY_TEMP_COLD, data);
		break;
	default:
		dfx_err("CHG:DFX: unknown type to report\n");
	}
	return;
}
EXPORT_SYMBOL(xm_handle_dfx_report);

void charger_plug_out_init(void)
{
	int i;
	dfx_info("Start. \n");
	dfs_info->evt_dfs_type = 0;
	dfs_info->evt_en_mask = ~0UL;
	dfs_info->charger_online = 0;
	disable_not_support_evt(dfs_info);
	for (i = 0; i < ARRAY_SIZE(dfs_evt_conds); i++) {
		dfs_evt_conds[i].report_times = 0;
	}
}

int is_condition_met(struct xm_dfs_info *info, int evt)
{
	int ret = 0;
	int i, cycle_count;
	struct xm_dfs_evt_condition *cond = NULL;
	dfx_info(" evt:%d Start. \n", evt);
	for (i = 0; i < ARRAY_SIZE(dfs_evt_conds); i++) {
		cond = &dfs_evt_conds[i];
		if (cond->evt == evt) {
			dfx_info("evt:%d condition: %d %d %d %d %d  \n", evt, cond->is_first_report,
				cond->support_boot_report, cond->require_charge_stat,
				cond->max_report_times, cond->report_times);
			if (cond->support_boot_report && cond->is_first_report) {
				cond->is_first_report = 0;
				if (info->charger_online) {
					cond->report_times++;
				}
				ret = 1;
			} else if (cond->require_charge_stat && info->charger_online) {
				ret = 1;
				cond->report_times++;
			} else if (!cond->require_charge_stat) {
				if (cond->evt == CHG_DFX_BATT_CYCLE_COUNT) {
					cycle_count = dfx_data_p->data_batt.cycle;
					if (cycle_count > 0 && cycle_count % 100 == 0 ) {
						ret = 1;
					}
				} else {
					ret = 1;
					cond->report_times++;
				}
			}
			if (cond->report_times >= cond->max_report_times) {
				clear_bit(evt, &info->evt_en_mask);
			}
			break;
		}
	}
	if (i >= ARRAY_SIZE(dfs_evt_conds)) {
		dfx_err("Can not match the condition for evet:%d , disable it!\n", evt);
		clear_bit(evt, &info->evt_en_mask);
	}
	dfx_info(" evt:%d conds_met:%d end. \n", evt, ret);
	return ret;
}

void xm_dfs_work_report_event(struct xm_dfs_info *info)
{
	int evt = 0;
	dfx_info("start. \n");
	mutex_lock(&info->lock);
	for (evt = 0; evt < CHG_DFX_MAX_INDEX; evt++) {
		if (test_bit(evt, &info->evt_dfs_type) && test_bit(evt, &info->evt_en_mask)) {
			if (is_condition_met(info, evt)) {
				xm_handle_dfx_report(evt, true);
			}
			clear_bit(evt, &dfs_info->evt_dfs_type);
		}
	}
	mutex_unlock(&info->lock);
}

static int xm_dfs_chg_get_charger_enabled(void)
{
	int ret = 0;
	int state = 0, status = 0;
	int enabled = 0;
	struct charger_manager *cm = NULL;

	cm = power_supply_get_drvdata(dfx_data_p->batt_psy);
	if (IS_ERR_OR_NULL(cm)) {
		dfx_info("get charger cm failed !\n");
		goto out;
	}

	ret = charger_get_chg_status(cm->charger, &state, &status);
	if (ret < 0) {
		dfx_info("failed to get chg status prop\n");
		goto out;
	}
	enabled = status;
out:
	return enabled;
}

static int xm_dfs_chg_get_batt_temp(struct power_supply *psy)
{
	int ret = 0;
	union power_supply_propval val = {0,};
	int temp = 250;

	if (IS_ERR_OR_NULL(psy)) {
		dfx_info("psy is NULL !\n");
		goto out;
	}

	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_TEMP, &val);
	if (ret) {
		dfx_info("get charger temp failed !\n");
		goto out;
	}
	temp = val.intval;

out:
	return temp;
}

static int xm_dfs_chg_get_cyclecount(struct power_supply *psy)
{
	int ret = 0;
	union power_supply_propval val = {0,};
	int cyclecount = 0;

	if (IS_ERR_OR_NULL(psy)) {
		dfx_info("psy is NULL !\n");
		goto out;
	}

	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CYCLE_COUNT, &val);
	if (ret) {
		dfx_info("get charger cyclecount failed !\n");
		goto out;
	}
	cyclecount = val.intval;

out:
	return cyclecount;
}

static int xm_dfs_chg_get_board_temp(struct power_supply *psy)
{
	int ret = 0;
	union power_supply_propval val = {0,};
	int temp = 25000;

	if (IS_ERR_OR_NULL(psy)) {
		dfx_info("psy is NULL !\n");
		goto out;
	}

	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_TEMP_AMBIENT, &val);
	if (ret) {
		dfx_info("get charger board temp failed !\n");
		goto out;
	}
	temp = val.intval;

out:
	return temp;
}

static int xm_dfs_chg_get_batt_status(struct power_supply *psy)
{
	int ret = 0;
	union power_supply_propval val = {0,};
	int status = 0;

	if (IS_ERR_OR_NULL(psy)) {
		dfx_info("psy is NULL !\n");
		goto out;
	}

	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_STATUS, &val);
	if (ret) {
		dfx_info("get charger status failed !\n");
		goto out;
	}
	status = val.intval;

out:
	return status;
}

static int xm_dfs_chg_get_batt_capacity(struct power_supply *psy)
{
	int ret = 0;
	union power_supply_propval val = {0,};
	int capacity = 0;

	if (IS_ERR_OR_NULL(psy)) {
		dfx_info("psy is NULL !\n");
		goto out;
	}

	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &val);
	if (ret) {
		dfx_info("get charger capacity failed !\n");
		goto out;
	}
	capacity = val.intval;

out:
	return capacity;
}

static int xm_dfs_chg_get_raw_soc(void)
{
	struct power_supply *psy_bms = NULL;
	struct sm_fg_chip *sm;
	int raw_soc = 0;

	psy_bms = power_supply_get_by_name("bms");
	if (IS_ERR_OR_NULL(psy_bms)) {
		dfx_info("psy_bms is NULL !\n");
		goto out;
	}

	sm = power_supply_get_drvdata(psy_bms);
	if (IS_ERR_OR_NULL(sm)) {
		dfx_info("sm is NULL !\n");
		goto out;
	}
	raw_soc = sm->batt_soc;

out:
	return raw_soc;
}

static int xm_dfs_chg_get_batt_voltage(struct power_supply *psy)
{
	int ret = 0;
	union power_supply_propval val = {0,};
	int volt = 0;

	if (IS_ERR_OR_NULL(psy)) {
		dfx_info("psy is NULL !\n");
		goto out;
	}

	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	if (ret) {
		dfx_info("get charger volt failed !\n");
		goto out;
	}
	volt = val.intval;

out:
	return volt;
}

static int xm_dfs_chg_get_batt_present(struct power_supply *psy)
{
	int ret = 0;
	union power_supply_propval val = {0,};
	int present = 0;

	if (IS_ERR_OR_NULL(psy)) {
		dfx_info("psy is NULL !\n");
		goto out;
	}

	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_PRESENT, &val);
	if (ret) {
		dfx_info("get charger present failed !\n");
		goto out;
	}
	present = val.intval;

out:
	return present;
}

static int xm_dfs_chg_get_real_type(void)
{
	enum vbus_type real_type = 0;
	struct power_supply *usb_psy = NULL;
	struct charger_manager *manager = NULL;
	int ret;

	usb_psy = power_supply_get_by_name("usb");
	if (IS_ERR_OR_NULL(usb_psy)) {
		dfx_info("usb_psy is NULL !\n");
		goto out;
	}
	manager = power_supply_get_drvdata(usb_psy);
	if (IS_ERR_OR_NULL(manager)) {
		dfx_info("manager is NULL !\n");
		goto out;
	}
	ret = charger_get_vbus_type(manager->charger, &real_type);
	dfx_info("charger_get_vbus_type=%d\n", real_type);

out:
	return real_type;
}

static void dfx_cyclecount_check(void)
{
	static int count = 0;

	count = dfx_data_p->data_batt.cycle   / 100;
	if ((dfx_data_p->data_batt.cycle > 0) && (dfx_data_p->data_batt.cycle % 100 == 0) &&
			(dfx_data_p->dfx_cycle_count_flag < count)) {
		dfx_data_p->dfx_cycle_count_flag++;
		xm_handle_dfx_report(CHG_DFX_BATT_CYCLE_COUNT, true);
		dfx_err("batt cyclecout, send dfx report, count :%d val: %d\n",
			dfx_data_p->data_batt.cycle, dfx_data_p->dfx_cycle_count_flag);
	}
	return;
}

static void dfx_none_standard_check(void)
{
	if (dfx_data_p->data_batt.real_type == 5 && dfs_info->charger_online == 1) {//float
		xm_handle_dfx_report(CHG_DFX_NONE_STANDARD_CHG, true);
		dfx_err("none standard, send dfx report, val: %d\n",dfx_data_p->data_batt.real_type);
	}
	return;
}

static void dfx_uisoc_not_full_check(void)
{
	static int pre_battery_status = -1;

 	if(pre_battery_status == POWER_SUPPLY_STATUS_FULL ||
			dfx_data_p->data_batt.status != POWER_SUPPLY_STATUS_FULL) {
		pre_battery_status = dfx_data_p->data_batt.status;
		return;
	}
	pre_battery_status = dfx_data_p->data_batt.status;

	if (dfx_data_p->data_batt.uisoc != 100) {
		xm_handle_dfx_report(CHG_DFX_UISOC_NOT_FULL, true);
		dfx_err("vbat:%d uisoc:%d rawsoc:%d\n",dfx_data_p->data_batt.vbat,
			dfx_data_p->data_batt.uisoc, dfx_data_p->data_batt.rawsoc);
	}
	return;
}

/* dfx temp */
#define COLD_ZONE_LOW           0
#define COLD_ZONE_HIGH          100
#define HOT_ZONE_LOW            480
#define HOT_ZONE_HIGH           550

static void dfx_temp_check(void)
{
	if (dfx_data_p->data_batt.is_charging == POWER_SUPPLY_STATUS_DISCHARGING && dfs_info->charger_online == 1) {
		if (IS_BETWEEN(dfx_data_p->data_batt.tbat_x10, COLD_ZONE_LOW, COLD_ZONE_HIGH)) {
			xm_handle_dfx_report(CHG_DFX_LOW_TEMP_DISCHARGING, true);
			dfx_err("low temp dischg, send dfx report, val: %d\n", dfx_data_p->data_batt.tbat_x10);
		} else if (IS_BETWEEN(dfx_data_p->data_batt.tbat_x10, HOT_ZONE_LOW, HOT_ZONE_HIGH)) {
			xm_handle_dfx_report(CHG_DFX_HIGH_TEMP_DISCHARGING, true);
			dfx_err("high temp dischg, send dfx report, val: %d\n", dfx_data_p->data_batt.tbat_x10);
		}
	}
	if (dfx_data_p->data_batt.tbat_x10 < -100) {
		xm_handle_dfx_report(CHG_DFX_BATTERY_TEMP_LOW, true);
		dfx_err("low temp, send dfx report, val: %d\n", dfx_data_p->data_batt.tbat_x10);
	} else if (dfx_data_p->data_batt.tbat_x10 > HOT_ZONE_HIGH) {
		xm_handle_dfx_report(CHG_DFX_BATTERY_TEMP_HIGH, true);
		dfx_err("high temp, send dfx report, val: %d\n", dfx_data_p->data_batt.tbat_x10);
	}
	return;
}

static void dfx_batt_absent_check(void)
{
	int present = 0;
	present = xm_dfs_chg_get_batt_present(dfx_data_p->batt_psy);
	if (present == 0) {
		xm_handle_dfx_report(CHG_DFX_BATT_LINKER_ABSENT, true);
		dfx_err("batt absent, send dfx report, present: %d\n", present);
	}
	return;
}

static void dfx_batt_auth_check(void)
{
	const char *chip_name_cmdline = find_secret_attr_by_name("chip_name");
	if(chip_name_cmdline == NULL)
		return;

	dfx_info("batt auth, secret name: %s\n", chip_name_cmdline);
	if (strcmp(chip_name_cmdline, "unsupported") != 0 && lc_is_battery_auth_success() == 0) {
		xm_handle_dfx_report(CHG_DFX_BATT_AUTH_ERR, true);
		dfx_err("batt auth, send dfx report\n");
	}
}

static void xm_dfx_monitor_work(struct work_struct *work)
{

	static ktime_t pre_report_time = 0;

	if (dfx_data_p->dfx_work_delay == 0)
		dfx_data_p->dfx_work_delay = DELTA_REPORT_NS;

	dfx_info("%s start\n", __func__);
	if (pre_report_time == 0) {
		pre_report_time = ktime_get();
	}

	if (!dfs_info) {
		dfx_err("dfs_info is not ready!\n");
		return ;
	}

	if (IS_ERR_OR_NULL(dfx_data_p->batt_psy)) {
		dfx_data_p->batt_psy = power_supply_get_by_name("battery");
		if(IS_ERR_OR_NULL(dfx_data_p->batt_psy))
			goto out;
		dfx_err("dfx_data_p get batt_psy pass\n");
	}

	if (dfx_data_p->batt_psy) {
		dfx_data_p->data_batt.uisoc = xm_dfs_chg_get_batt_capacity(dfx_data_p->batt_psy);
		dfx_data_p->data_batt.vbat = xm_dfs_chg_get_batt_voltage(dfx_data_p->batt_psy);
		dfx_data_p->data_batt.cycle = xm_dfs_chg_get_cyclecount(dfx_data_p->batt_psy);
		dfx_data_p->data_batt.rawsoc = xm_dfs_chg_get_raw_soc();
		dfx_data_p->data_batt.tbat_x10 = xm_dfs_chg_get_batt_temp(dfx_data_p->batt_psy);
		dfx_data_p->data_batt.is_charging = xm_dfs_chg_get_charger_enabled();
		dfx_data_p->data_batt.status = xm_dfs_chg_get_batt_status(dfx_data_p->batt_psy);
		dfx_data_p->tboard_x1000 = xm_dfs_chg_get_board_temp(dfx_data_p->batt_psy);
		dfx_data_p->data_batt.real_type = xm_dfs_chg_get_real_type();
	} else {
		goto out;
	}

	get_charger_online(&dfs_info->charger_online);
	dfx_cyclecount_check();
	dfx_none_standard_check();
	dfx_uisoc_not_full_check();
	dfx_temp_check();
	dfx_batt_absent_check();
	dfx_batt_auth_check();

out:
	schedule_delayed_work(&dfx_data_p->dfx_monitor_work, msecs_to_jiffies(dfx_data_p->dfx_work_delay));

	return;
}

static int init_xm_dfs_info(void)
{
	dfx_info("Start. \n");

	dfs_info = kmalloc(sizeof(*dfs_info), GFP_KERNEL);
	if (!dfs_info) {
		dfx_err("%s failed to alloc memory for dfs_info", __func__);
		return -ENOMEM;
	}

	mutex_init(&dfs_info->lock);
	charger_plug_out_init();
	INIT_DELAYED_WORK(&dfx_data_p->dfx_monitor_work, xm_dfx_monitor_work);

	dfx_data_p->dfx_work_delay = DELTA_REPORT_NS;
	schedule_delayed_work(&dfx_data_p->dfx_monitor_work, msecs_to_jiffies(dfx_data_p->dfx_work_delay));
	return 0;
}

static void deinit_xm_dfs_info(void)
{
	cancel_delayed_work_sync(&dfx_data_p->dfx_monitor_work);

	if (dfs_info && dfs_info->task) {
		kthread_stop(dfs_info->task);
		dfs_info->task = NULL;
	}
	xm_dfx_chg_dev = NULL;
}

/******************* Module Init ***********************************/
int xm_smart_chg_init(void)
{
	init_xm_dfs_info();
	return 0;
}

static void __exit xm_smart_chg_exit(void)
{
	deinit_xm_dfs_info();
}

late_initcall(xm_smart_chg_init);
module_exit(xm_smart_chg_exit);

MODULE_DESCRIPTION("XM SMART CHARGE Driver");
MODULE_LICENSE("GPL");
MODULE_SOFTDEP("lc charger");
