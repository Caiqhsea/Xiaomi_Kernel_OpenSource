// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2019 The Linux Foundation. All rights reserved.
 */

#define pr_fmt(fmt) "lc_jeita: %s: " fmt, __func__

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/slab.h>

#include "lc_jeita.h"
#include "../lc_printk.h"
#ifdef TAG
#undef TAG
#define  TAG "[LC_CHG][jeita]"
#endif

#define MAX_STRING_SIZE            128

static struct lc_jeita_info *the_chip;
static bool warm_stop_charge;
static bool warm_vbat_high;
#if 0
int cc_cv_cycle_count[CC_CV_CYCLE_COUNT_MAX] = {0,100,300,800};

/*ffc cycle*/
struct step_jeita_cycle_cfg jeita_FFC_cfg_15_45[CC_CV_CYCLE_COUNT_MAX] = {{3600,4250,4560,6000,5400},{3600,4200,4540,6000,5400},{3600,4170,4520,6000,5400},{3600,4150,4510,4800,4320}};

/*nomal cycle  20°C-35°C*/
struct step_jeita_cycle_cfg jeita_nomal1_fcc_cfg[CC_CV_CYCLE_COUNT_MAX] = {{3600,4250,4490,6000,5400},{3600,4250,4480,6000,5400},{3600,4250,4470,6000,5400},{3600,4250,4450,4800,4320}};
/*35°C-40°C*/
struct step_jeita_cycle_cfg jeita_nomal2_fcc_cfg[CC_CV_CYCLE_COUNT_MAX] = {{3600,4250,4480,6000,5400},{3600,4250,4470,6000,5400},{3600,4250,4460,6000,5400},{3600,4250,4440,4800,4320}};
/*40°C-45°C*/
struct step_jeita_cycle_cfg jeita_nomal3_fcc_cfg[CC_CV_CYCLE_COUNT_MAX] = {{3600,4250,4470,6000,5400},{3600,4250,4460,6000,5400},{3600,4250,4450,6000,5400},{3600,4250,4430,4800,4320}};

/*other nomal cycle*/
struct step_jeita_normal_cfg jeita_nomal1_cfg[CC_CV_CYCLE_COUNT_MAX] = {{4490,5400},{4480,5400},{4470,5400},{4450,4320}};
struct step_jeita_normal_cfg jeita_nomal2_cfg[CC_CV_CYCLE_COUNT_MAX] = {{4480,5400},{4470,5400},{4460,5400},{4440,4320}};
struct step_jeita_normal_cfg jeita_nomal3_cfg[CC_CV_CYCLE_COUNT_MAX] = {{4470,5400},{4460,5400},{4450,5400},{4430,4320}};
#endif
/*cycle count end*/
bool get_warm_stop_charge_state(void)
{
	return warm_stop_charge;
}
EXPORT_SYMBOL(get_warm_stop_charge_state);

static bool is_batt_available(struct lc_jeita_info *chip)
{
	if (!chip->batt_psy)
		chip->batt_psy = power_supply_get_by_name("battery");

	if (!chip->batt_psy)
		return false;

	return true;
}

static bool is_bms_available(struct lc_jeita_info *chip)
{
	if (!chip->bms_psy)
		chip->bms_psy = power_supply_get_by_name("bms");

	if (!chip->bms_psy)
		return false;

	return true;
}

static bool is_input_present(struct lc_jeita_info *chip)
{
	int rc = 0, input_present = 0;
	union power_supply_propval pval = {0, };

	if (!chip->usb_psy)
		chip->usb_psy = power_supply_get_by_name("usb");
	if (chip->usb_psy) {
		rc = power_supply_get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_ONLINE, &pval);
		if (rc < 0)
			lc_err("Couldn't read USB Present status, rc=%d\n", rc);
		else
			input_present |= pval.intval;
	}

	if (input_present)
		return true;

	return false;
}

static int lc_jeita_get_temp_level(struct lc_jeita_info *chip, int temp)
{
	int i = 0;

	for (i = 0; i < JEITA_TEMP_MAX; i++) {
		if (temp < chip->jeita_temp_level[i])
			break;
	}
	if (i == JEITA_TEMP_MAX)
		return JEITA_TEMP_HOT;
	return i;
}

static int lc_jeita_get_index_new(int bounds[], int value, int len) {
	int i = 0;

	if (len < 2)
		return -2; //length of attr is short

	if (value < bounds[0])
		return -1;
	if (value >= bounds[len - 1])
		return len - 1;

	for (i = 0; i < len - 1; i++) {
		if (value >= bounds[i] && value < bounds[i + 1])
			return i;
	}

	return -2; //cannot reach here
}

static int lc_jeita_get_temp_index_new(struct lc_jeita_info *chip, int temp)
{
	if (IS_ERR_OR_NULL(chip->jeita_temp_bounds))
		return PTR_ERR(chip->jeita_temp_bounds);

	return lc_jeita_get_index_new(chip->jeita_temp_bounds,
			temp, chip->jeita_temp_bounds_len);
}

static int lc_jeita_get_cycle_index_new(struct lc_jeita_info *chip, int cyclecount)
{
	if (IS_ERR_OR_NULL(chip->jeita_cc_bounds))
		return PTR_ERR(chip->jeita_cc_bounds);

	return lc_jeita_get_index_new(chip->jeita_cc_bounds,
			cyclecount, chip->jeita_cc_bounds_len);
}

static int handle_jeita(struct lc_jeita_info *chip)
{
	union power_supply_propval pval = {0, };
	int ret = 0;
	int cycle_index = 0;
	int jeita_temp_index = 0;
	int temp_now, vol_now;
	int eu_mode = -EINVAL;
	int curr_offset = 0;
	static bool cold_curr_lmt = false;
	int cyclecount;
	int fv_temp = 0;
	int fcc_temp = 0;
	struct charger_manager *manager;
	int capacity;
	int vol_ocv;
	//int normal_fv = 0;
	//int ffc_fv =0;

	if (!is_batt_available(chip)) {
		pr_err("failed to get batt psy\n");
		return 0;
	}

#if IS_ENABLED(CONFIG_PD_BATTERY_SECRET)
	chip->pd_adapter = get_adapter_by_name("pd_adapter");
	if (!chip->pd_adapter)
		lc_err("failed to pd_adapter\n");
	else
		chip->pd_verifed = chip->pd_adapter->verifed;
#endif

	if (chip->fuel_gauge == NULL)
		chip->fuel_gauge = fuel_gauge_find_dev_by_name("fuel_gauge");
	if (IS_ERR_OR_NULL(chip->fuel_gauge)) {
		lc_err("failed to get fuel_gauge\n");
	}

	if (chip->charger == NULL)
		chip->charger = charger_find_dev_by_name("primary_chg");
	if (IS_ERR_OR_NULL(chip->charger)) {
		lc_err("failed to get main charger\n");
		return 0;
	}

	manager = (struct charger_manager *)power_supply_get_drvdata(chip->batt_psy);
	if (IS_ERR_OR_NULL(manager)) {
		lc_err("manager is_err_or_null\n");
		return 0;
	}

	ret = power_supply_get_property(chip->batt_psy, POWER_SUPPLY_PROP_STATUS, &pval);
	if (ret < 0) {
		lc_err("Couldn't read batt temp, ret=%d\n", ret);
	}
	if (pval.intval == POWER_SUPPLY_STATUS_DISCHARGING) {
		vote(chip->fv_votable, SDP_JEITA_VOTER, false, 0);
		cold_curr_lmt = false;
		return 0;
	}

	ret = power_supply_get_property(chip->batt_psy, POWER_SUPPLY_PROP_TEMP, &pval);
	if (ret < 0) {
		lc_err("Couldn't read batt temp, ret=%d\n", ret);
	}
	temp_now = pval.intval;

	ret = power_supply_get_property(chip->batt_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &pval);
	if (ret < 0) {
		lc_err("Couldn't read batt voltage_now, ret=%d\n", ret);
	}
	vol_now = pval.intval / 1000;

	ret = power_supply_get_property(chip->bms_psy, POWER_SUPPLY_PROP_VOLTAGE_OCV, &pval);
	if (ret < 0) {
		lc_err("Couldn't read batt voltage_ocv, ret=%d\n", ret);
	}
	vol_ocv = pval.intval / 1000;

	if (temp_now < 0) {
		charger_set_rechg_volt(chip->charger, LOW_TEMP_RECHG_OFFSET);
	} else {
		charger_set_rechg_volt(chip->charger, NOR_TEMP_RECHG_OFFSET);
	}

	ret = power_supply_get_property(chip->batt_psy, POWER_SUPPLY_PROP_CAPACITY, &pval);
	if (ret < 0) {
		lc_err("Couldn't read batt capacity, ret=%d\n", ret);
	}
	capacity = pval.intval;

	charger_is_charge_done(chip->charger, &(chip->is_charge_done));
#if 0
	//vbat < 4.2V and -10 < temp <0，ichg = 1A
	if (chip->jeita_index == 0 && (vol_now < NAGETIVE_10_TO_0_VOL_4200)) {
		if (cold_curr_lmt) { //if limited,need vbat below 4.1V,than set ichg = 1A
			if ((vol_now < (NAGETIVE_10_TO_0_VOL_4200 - COLD_RECHG_VOLT_OFFSET))) {
				curr_offset = chip->under_4200_curr_offset;
				cold_curr_lmt = false;
			}
		} else
			curr_offset = chip->under_4200_curr_offset;
	} else { //vbat >= 4.2V, ichg = 0.7A
		curr_offset = 0;
		cold_curr_lmt = true;
	}
#endif
	chip->iterm_curr = chip->iterm;
	if (manager->vbus_type == VBUS_TYPE_SDP) {
		chip->iterm_curr = 200;
	}
	if (chip->iterm_curr >= 500 && chip->temp_level <= JEITA_TEMP_GOOD)
		chip->iterm_curr -= 200;
#if 0
	/*FFC: iterm + fv*/
	if (fastcharge_mode) {
		if (chip->jeita_index == INDEX_20_to_35) {
			i = 0 ;
		} else if (chip->jeita_index == INDEX_35_to_40) {
			i = 1;
		} else if (chip->jeita_index == INDEX_40_to_45) {
			i = 2;
		}
		if (battery_id == 1) {
			chip->iterm_ffc = jeita_ffc_iterm_cfg[i].bat1_iterm;
		} else if (battery_id == 2) {
			chip->iterm_ffc = jeita_ffc_iterm_cfg[i].bat2_iterm;
		}else if (battery_id == 3) {
			chip->iterm_ffc = jeita_ffc_iterm_cfg[i].bat3_iterm;
		}
		if (chip->jeita_index == INDEX_20_to_35) {
			chip->iterm_curr = chip->iterm_ffc;
		} else if (chip->jeita_index == INDEX_35_to_40) {
			chip->iterm_curr = chip->iterm_ffc;
		} else if (chip->jeita_index == INDEX_40_to_45) {
			chip->iterm_curr = chip->iterm_ffc;
		}
		if (chip->is_charge_done)
			chip->fv = normal_fv;
	}
#endif
	vote(chip->iterm_votable, ITER_VOTER, true, chip->iterm_curr);

	/* config fcc fv by cycle_count */
	ret = power_supply_get_property(chip->batt_psy, POWER_SUPPLY_PROP_CYCLE_COUNT, &pval);
	if (ret < 0) {
		lc_err("Couldn't read batt voltage_now, ret=%d\n", ret);
	}
	cyclecount = pval.intval;

	cycle_index = lc_jeita_get_cycle_index_new(chip, cyclecount);
	if (cycle_index < 0) {
		lc_err("cycle is %d, not correct, usb default cycle\n", cyclecount);
		cycle_index = 0;
	} else if (cycle_index >= chip->jeita_cc_bounds_len) {
		lc_err("cycle is %d, out of range, usb max cycle\n", cyclecount);
		cycle_index = chip->jeita_cc_bounds_len - 1;
	}
	switch (chip->temp_level) {
		case JEITA_TEMP_COLD:
			chip->temp_level = lc_jeita_get_temp_level(chip, temp_now - COLD_RECHG_TEMP_OFFSET);
			jeita_temp_index = lc_jeita_get_temp_index_new(chip, temp_now - COLD_RECHG_TEMP_OFFSET);
			break;
		case JEITA_TEMP_COOL:
		case JEITA_TEMP_GOOD:
			chip->temp_level = lc_jeita_get_temp_level(chip, temp_now + WARM_STPCHG_TEMP_OFFSET);
			jeita_temp_index = lc_jeita_get_temp_index_new(chip, temp_now + WARM_STPCHG_TEMP_OFFSET);
			break;
		case JEITA_TEMP_WARM:
			chip->temp_level = lc_jeita_get_temp_level(chip, temp_now + WARM_RECHG_TEMP_OFFSET);
			if (chip->temp_level == JEITA_TEMP_HOT) {
				chip->temp_level = lc_jeita_get_temp_level(chip, temp_now);
				jeita_temp_index = lc_jeita_get_temp_index_new(chip, temp_now);
			} else
				jeita_temp_index = lc_jeita_get_temp_index_new(chip, temp_now + WARM_RECHG_TEMP_OFFSET);
			break;
		case JEITA_TEMP_HOT:
			chip->temp_level = lc_jeita_get_temp_level(chip, temp_now + HOT_RECHG_TEMP_OFFSET);
			jeita_temp_index = lc_jeita_get_temp_index_new(chip, temp_now + HOT_RECHG_TEMP_OFFSET);
			break;
		default:
			chip->temp_level = lc_jeita_get_temp_level(chip, temp_now);
			jeita_temp_index = lc_jeita_get_temp_index_new(chip, temp_now);
			break;
	}

	if (chip->temp_level <= JEITA_TEMP_COLD || chip->temp_level >= JEITA_TEMP_HOT) {
		lc_err("too %s to stop charge.\n", (chip->temp_level == JEITA_TEMP_COLD) ? "COLD":"HOT");
		vote(chip->total_fcc_votable, JEITA_VOTER, true, 0);
		goto out;
	}
	if (jeita_temp_index >= 0 && jeita_temp_index < chip->jeita_temp_bounds_len)
	{
		eu_mode = get_eu_mode();
		lc_info("eu_mode = %d \n", eu_mode);
		if (eu_mode == true) {
			/* TBD */
			fcc_temp = chip->jeita_cc_fcc_cfg[cycle_index * chip->jeita_temp_bounds_len + jeita_temp_index];
			fv_temp = chip->jeita_cc_fv_cfg[cycle_index * chip->jeita_temp_bounds_len + jeita_temp_index];
		} else {
			fcc_temp = chip->jeita_cc_fcc_cfg[cycle_index * chip->jeita_temp_bounds_len + jeita_temp_index];
			fv_temp = chip->jeita_cc_fv_cfg[cycle_index * chip->jeita_temp_bounds_len + jeita_temp_index];
		}
		//limit voltage stop charging in high temp(factory)
		if (chip->temp_level > JEITA_TEMP_GOOD)
			fv_temp -= WARM_STPCHG_VOLT_OFFSET;
		if(chip->fv_votable)
			vote(chip->fv_votable, JEITA_VOTER, true, fv_temp);

		if (manager->vbus_type == VBUS_TYPE_SDP){
                	fv_temp -= 20;
                	if(chip->fv_votable)
				vote(chip->fv_votable, SDP_JEITA_VOTER, true, fv_temp);
                }

		if (chip->temp_level > JEITA_TEMP_GOOD &&
				vol_ocv >= fv_temp - (warm_vbat_high ? WARM_RECHG_VOLT_OFFSET : 0)) {
			lc_err("voltage_ocv is higher than warm fv:%d.\n", vol_ocv);
			vote(chip->total_fcc_votable, JEITA_VOTER, true, 0);
			warm_vbat_high = true;
			goto out;
		}
		warm_vbat_high = false;
		vote(chip->total_fcc_votable, JEITA_VOTER, true, fcc_temp);
	} else {
		//unlikely reach here
		lc_info("temp is out range of cycle_temp\n");
	}
	lc_info("jeita_fcc = %d, jeita_fv = %d, jeita_iterm = %d vol_now = %d, vol_ocv = %d\n",
			fcc_temp, fv_temp, chip->iterm_curr, vol_now, vol_ocv);

	fv_temp = get_effective_result(chip->fv_votable);
	if (fv_temp < 0) {
		lc_err("failed to get fv_votable\n");
	} else {
		if (chip->is_charge_done && chip->fv < fv_temp && vol_now < fv_temp) {
			charger_set_chg(chip->charger, false);
			/* recharge delay */
			msleep(50);
			charger_set_chg(chip->charger, true);
			lc_info("recharge due to fv change \n");
		}
	}
out:
	fcc_temp = get_effective_result(chip->total_fcc_votable);
	fv_temp = get_effective_result(chip->fv_votable);
	chip->fv = fv_temp;

	if (chip->temp_level > JEITA_TEMP_GOOD &&
			vol_now >= (chip->fv - WARM_RECHG_VOLT_OFFSET)) {
		warm_stop_charge = true;
	} else {
		warm_stop_charge = false;
	}

	lc_info("current_fcc = %d, current_fv = %d, jeita_iterm = %d\n", fcc_temp, fv_temp, chip->iterm_curr);
	lc_info("temp_level = %d cycle_index = %d  jeita_temp_index = %d\n", chip->temp_level, cycle_index, jeita_temp_index);
	lc_info("pd_verifed = %d, curr_offset = %d, is_charge_done = %d ibat = %d\n",
				chip->pd_verifed, curr_offset, chip->is_charge_done, manager->ibat / 1000);
	return 0;
}

static void status_change_work(struct work_struct *work)
{
	struct lc_jeita_info*chip = container_of(work,
			struct lc_jeita_info, status_change_work.work);
	int rc = 0;

	if (!is_batt_available(chip)|| !is_bms_available(chip))
		goto exit_work;

	rc = handle_jeita(chip);
	if (rc < 0)
		lc_err("Couldn't handle sw jeita rc = %d\n", rc);

	if (! is_input_present(chip)) {
		vote(chip->main_icl_votable, JEITA_VOTER, false, 0);
	}

exit_work:
	__pm_relax(chip->lc_jeita_ws);
}

static int jeita_notifier_call(struct notifier_block *nb,
		unsigned long ev, void *v)
{
	struct power_supply *psy = v;
	struct lc_jeita_info*chip = container_of(nb, struct lc_jeita_info, nb);

	if (ev != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;

	if (!strcmp(psy->desc->name, "usb")) {
		__pm_stay_awake(chip->lc_jeita_ws);
		schedule_delayed_work(&chip->status_change_work, 0);
	}

	return NOTIFY_OK;
}

static int jeita_register_notifier(struct lc_jeita_info *chip)
{
	int rc;

	chip->nb.notifier_call = jeita_notifier_call;
	rc = power_supply_reg_notifier(&chip->nb);
	if (rc < 0) {
		lc_err("Couldn't register psy notifier rc = %d\n", rc);
		return rc;
	}

	return 0;
}

#define DEFALUT_JEITA_CONFIG_ID	0
#define MAX_BATTERY_NUM 	5
static bool lc_parse_jeita_dt(struct device_node *node, struct lc_jeita_info *chip)
{
	int total_length = 0;
	bool ret = 0;
	int battery_id = DEFALUT_JEITA_CONFIG_ID;
	char prop_name[MAX_STRING_SIZE] = {'\0'};
	int jeita_batt_map[MAX_BATTERY_NUM] = {0};

	if (chip->fuel_gauge == NULL)
		chip->fuel_gauge = fuel_gauge_find_dev_by_name("fuel_gauge");
	if (IS_ERR_OR_NULL(chip->fuel_gauge)) {
		lc_err("failed to get fuel_gauge\n");
	} else {
		battery_id = fuel_gauge_get_battery_id(chip->fuel_gauge);
		if (battery_id < 0) {
			lc_err("failed to get battery_id, use default battery_id\n");
			battery_id = DEFALUT_JEITA_CONFIG_ID;
		}
	}

	total_length = of_property_count_elems_of_size(node, "jeita-batt-map", sizeof(u32));
	if (total_length < 0) {
		lc_err("failed to read total_length of jeita_batt_map\n");
		chip->jeita_cfg_id = DEFALUT_JEITA_CONFIG_ID;
	} else {
		ret |= of_property_read_u32_array(node, "jeita-batt-map", (u32 *)jeita_batt_map, total_length);
		if (ret) {
			lc_err("failed to parse jeita_batt-map\n");
			chip->jeita_cfg_id = DEFALUT_JEITA_CONFIG_ID;
		} else if (battery_id < total_length)
			chip->jeita_cfg_id = jeita_batt_map[battery_id];
		else
			chip->jeita_cfg_id = DEFALUT_JEITA_CONFIG_ID;
	}

	lc_info("use jeita cfg %d\n",chip->jeita_cfg_id);
	snprintf(prop_name, MAX_STRING_SIZE, "jeita-temp-level-%d", chip->jeita_cfg_id);
	ret |= of_property_read_u32_array(node, prop_name, (u32 *)chip->jeita_temp_level, JEITA_TEMP_MAX);
	if (ret) {
		lc_err("failed to parse jeita_jeita_temp_level\n");
		return 0;
	}

/*
	snprintf(prop_name, MAX_STRING_SIZE, "jeita_cv_temp_step-%d", chip->jeita_cfg_id);
	ret |= of_property_read_u32(node, prop_name, &chip->jeita_cv_temp_len);
	if (ret) {
		lc_err("failed to parse jeita_cv_temp_step\n");
		return 0;
	}

	chip->jeita_fcc_cfg = devm_kmalloc_array(chip->dev, chip->jeita_cv_temp_len,
			sizeof(struct step_jeita_cfg), GFP_KERNEL);
	if (IS_ERR_OR_NULL(chip->jeita_fcc_cfg)) {
		lc_err("failed to malloc jeita_fcc_cfg, ret = %ld!\n", PTR_ERR(chip->jeita_fcc_cfg));
	} else {
		snprintf(prop_name, MAX_STRING_SIZE,"jeita_fcc_cfg-%d", chip->jeita_cfg_id);
		ret |= of_property_read_u32_array(node, prop_name, (u32 *)chip->jeita_fcc_cfg,
				(chip->jeita_cv_temp_len * sizeof(struct step_jeita_cfg))/sizeof(u32));
		if (ret) {
			lc_err("failed to parse jeita_fcc_cfg\n");
			return 0;
		}
	}

	chip->jeita_fv_cfg = devm_kmalloc_array(chip->dev, chip->jeita_cv_temp_len,
			sizeof(struct step_jeita_cfg), GFP_KERNEL);
	if (IS_ERR_OR_NULL(chip->jeita_fv_cfg)) {
		lc_err("failed to malloc jeita_fv_cfg, ret = %ld!\n", PTR_ERR(chip->jeita_fv_cfg));
	} else {
		snprintf(prop_name, MAX_STRING_SIZE,"jeita_fv_cfg-%d", chip->jeita_cfg_id);
		ret |= of_property_read_u32_array(node, prop_name, (u32 *)chip->jeita_fv_cfg,
				(chip->jeita_cv_temp_len * sizeof(struct step_jeita_cfg))/sizeof(u32));
		if (ret) {
			lc_err("failed to parse jeita_fv_cfg\n");
			return 0;
		}
	}

	for (i = 0; i < chip->jeita_cv_temp_len; i++)
		lc_info("[jeita_fcc_cfg]%d %d %d [jeita_fv_cfg]%d %d %d\n",
					chip->jeita_fcc_cfg[i].low_threshold, chip->jeita_fcc_cfg[i].high_threshold, chip->jeita_fcc_cfg[i].value,
					chip->jeita_fv_cfg[i].low_threshold, chip->jeita_fv_cfg[i].high_threshold, chip->jeita_fv_cfg[i].value);
*/
	ret |= of_property_read_u32(node, "iterm", &chip->iterm);

	ret |= of_property_read_u32(node, "under_4200_curr_offset", &chip->under_4200_curr_offset);
	ret |= of_property_read_u32(node, "fv_offset_15_to_35", &chip->fv_offset_15_to_35);
	ret |= of_property_read_u32(node, "fv_offset_35_to_48", &chip->fv_offset_35_to_48);

	return !ret;
}

static bool lc_parse_cyclecount_dt(struct device_node *node, struct lc_jeita_info *chip)
{
	bool ret = 0;
	int i = 0, j = 0;
	char cfg_str[MAX_STRING_SIZE] = {'\0'};
	char prop_name[MAX_STRING_SIZE] = {'\0'};

	snprintf(prop_name, MAX_STRING_SIZE, "jeita-cyclecount-bounds-%d", chip->jeita_cfg_id);
	chip->jeita_cc_bounds_len = of_property_count_elems_of_size(node, prop_name, sizeof(u32));
	if (!chip->jeita_cc_bounds_len) {
		lc_err("jeita-cyclecount-bounds is null\n");
		return false;
	}
	chip->jeita_cc_bounds = devm_kmalloc_array(chip->dev, chip->jeita_cc_bounds_len, sizeof(int), GFP_KERNEL);
	if (IS_ERR_OR_NULL(chip->jeita_cc_bounds)) {
		lc_err("failed to malloc chip->cc_bounds, len=%d, ret=%ld!\n",
				chip->jeita_cc_bounds_len, PTR_ERR(chip->jeita_cc_bounds));
		return false;
	}
	ret |= of_property_read_u32_array(node, prop_name, (u32 *)chip->jeita_cc_bounds, chip->jeita_cc_bounds_len);
	if (ret) {
		lc_err("failed to parse cyclecount-bounds\n");
		return false;
	}

	snprintf(prop_name, MAX_STRING_SIZE, "jeita-temp-bounds-%d", chip->jeita_cfg_id);
	chip->jeita_temp_bounds_len = of_property_count_elems_of_size(node, prop_name, sizeof(u32));
	if (!chip->jeita_temp_bounds_len) {
		lc_err("failed to parse jeita-temp-bounds\n");
		return false;
	}
	chip->jeita_temp_bounds = devm_kmalloc_array(chip->dev, chip->jeita_temp_bounds_len, sizeof(int), GFP_KERNEL);
	if (IS_ERR_OR_NULL(chip->jeita_temp_bounds)) {
		lc_err("failed to malloc chip->jeita_temp_bounds, len=%d, ret=%ld!\n",
				chip->jeita_temp_bounds_len, PTR_ERR(chip->jeita_temp_bounds));
		return false;
	}
	ret |= of_property_read_u32_array(node, prop_name, (u32 *)chip->jeita_temp_bounds, chip->jeita_temp_bounds_len);
	if (ret) {
		lc_err("failed to parse jeita-temp-bounds\n");
		return false;
	}

	chip->jeita_cc_fcc_cfg = devm_kmalloc_array(chip->dev,
			chip->jeita_temp_bounds_len * chip->jeita_cc_bounds_len, sizeof(int), GFP_KERNEL);
	if (IS_ERR_OR_NULL(chip->jeita_cc_fcc_cfg)) {
		lc_err("failed to malloc chip->jeita_cc_fcc_cfg, ret=%ld!\n", PTR_ERR(chip->jeita_cc_fcc_cfg));
		return false;
	}
	snprintf(prop_name, MAX_STRING_SIZE, "jeita-cyclecount-fcc-cfg-%d", chip->jeita_cfg_id);
	ret |= of_property_read_u32_array(node, prop_name,
			(u32 *)chip->jeita_cc_fcc_cfg, chip->jeita_temp_bounds_len * chip->jeita_cc_bounds_len);
	if (ret) {
		lc_err("failed to parse jeita-cyclecount-fcc-cfg\n");
		return false;
	} else {
		for (i = 0; i < chip->jeita_cc_bounds_len; i++) {
			snprintf(cfg_str, MAX_STRING_SIZE,"jeita_cyclecount_fcc_cfg:");
			for (j = 0; j < chip->jeita_temp_bounds_len; j++) {
				snprintf(cfg_str, MAX_STRING_SIZE,"%s %d",
					cfg_str, chip->jeita_cc_fcc_cfg[i * chip->jeita_temp_bounds_len + j]);
			}
			lc_info("%s.\n", cfg_str);
		}
	}

	chip->jeita_cc_fv_cfg = devm_kmalloc_array(chip->dev,
			chip->jeita_temp_bounds_len * chip->jeita_cc_bounds_len, sizeof(int), GFP_KERNEL);
	if (IS_ERR_OR_NULL(chip->jeita_cc_fv_cfg)) {
		lc_err("failed to malloc chip->jeita_cc_fv_cfg, ret=%ld!\n", PTR_ERR(chip->jeita_cc_fv_cfg));
		return false;
	}
	snprintf(prop_name, MAX_STRING_SIZE, "jeita-cyclecount-fv-cfg-%d", chip->jeita_cfg_id);
	ret |= of_property_read_u32_array(node, prop_name,
			(u32 *)chip->jeita_cc_fv_cfg, chip->jeita_temp_bounds_len * chip->jeita_cc_bounds_len);
	if (ret) {
		lc_err("failed to parse jeita-cyclecount-fv-cfg\n");
		return false;
	} else {
		for (i = 0; i < chip->jeita_cc_bounds_len; i++) {
			snprintf(cfg_str, MAX_STRING_SIZE,"jeita_cyclecount_fv_cfg:");
			for (j = 0; j < chip->jeita_temp_bounds_len; j++) {
				snprintf(cfg_str, MAX_STRING_SIZE,"%s %d",
					cfg_str, chip->jeita_cc_fv_cfg[i * chip->jeita_temp_bounds_len + j]);
			}
			lc_info("%s.\n", cfg_str);
		}
	}

	return !ret;
}

static bool lc_parse_ffc_dt(struct device_node *node, struct lc_jeita_info *chip)
{
	int total_length = 0;
	int i = 0;
	bool ret = 0;
	char prop_name[MAX_STRING_SIZE];

	if (of_property_read_bool(node, "step_chg_en"))
		chip->step_chg_en = true;
	else {
		chip->step_chg_en = false;
		lc_err("not support step chg\n");
		return !ret;
	}

	snprintf(prop_name, MAX_STRING_SIZE,"step_chg_cfg_cycle-%d", chip->jeita_cfg_id);
	total_length = of_property_count_elems_of_size(node, prop_name, sizeof(u32));
	if (total_length < 0) {
		lc_err("failed to read total_length of step_chg_cfg_cycle\n");
		return 0;
	}

	ret |= of_property_read_u32_array(node, prop_name, (u32 *)chip->step_chg_cfg, total_length);
	if (ret)
	{
		lc_err("failed to parse step_chg_cfg_cycle\n");
		return false;
	}

	for (i = 0; i < total_length; i++)
		lc_info("[STEP_CHG] [step_chg_cfg_cycle]%d %d %d\n",
					chip->step_chg_cfg[i].low_threshold, chip->step_chg_cfg[i].high_threshold, chip->step_chg_cfg[i].value);

	return !ret;
}
struct lc_jeita_info *get_jeita_info(void){
	return the_chip;
}
int lc_jeita_init(struct device *dev)
{
	struct device_node *node = dev->of_node;
	struct device_node *step_jeita_node = NULL;
	struct lc_jeita_info *chip = NULL;

	int rc = 0;

	if (node) {
		step_jeita_node = of_find_node_by_name(node, "step_jeita");
	}

	if (the_chip) {
		lc_err("Already initialized\n");
		return -EINVAL;
	}

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->lc_jeita_ws = wakeup_source_register(dev, "lc_jeita");
	if (!chip->lc_jeita_ws)
		return -EINVAL;

	chip->dev = dev;
	chip->temp_level = JEITA_TEMP_GOOD;

	rc = lc_parse_jeita_dt(step_jeita_node, chip);
	if(!rc)
		lc_err("lc_parse_jeita_dt failed\n");

	rc = lc_parse_cyclecount_dt(step_jeita_node, chip);
	if(!rc)
		lc_err("lc_parse_cyclecount_dt failed\n");

	rc = lc_parse_ffc_dt(step_jeita_node, chip);
	if(!rc)
		lc_err("lc_parse_ffc_dt failed\n");

	chip->total_fcc_votable = find_votable("TOTAL_FCC");
	if (!chip->total_fcc_votable)
		lc_err("find TOTAL_FCC voltable failed\n");

	chip->fv_votable = find_votable("MAIN_FV");
	if (!chip->fv_votable)
		lc_err("find MAIN_FV voltable failed\n");

	chip->iterm_votable = find_votable("MAIN_ITERM");
	if (!chip->iterm_votable)
		lc_err("find MAIN_FV voltable failed\n");

	chip->charger = charger_find_dev_by_name("primary_chg");
	if (chip->charger == NULL) {
		lc_err("failed get charger\n");
	}

	INIT_DELAYED_WORK(&chip->status_change_work, status_change_work);

	rc = jeita_register_notifier(chip);
	if (rc < 0) {
		lc_err("Couldn't register psy notifier rc = %d\n", rc);
		goto release_wakeup_source;
	}

	the_chip = chip;

	lc_info("lc_jeita_init success\n");

	return 0;

release_wakeup_source:
	wakeup_source_unregister(chip->lc_jeita_ws);
	return rc;
}

void lc_jeita_deinit(void)
{
	struct lc_jeita_info *chip = the_chip;

	if (!chip)
		return;

	cancel_delayed_work_sync(&chip->status_change_work);
	power_supply_unreg_notifier(&chip->nb);
	wakeup_source_unregister(chip->lc_jeita_ws);
	the_chip = NULL;
}
