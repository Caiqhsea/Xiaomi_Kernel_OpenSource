/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2017-2019 The Linux Foundation. All rights reserved.
 */

#ifndef __LC_JEITA_H__
#define __LC_JEITA_H__

#include "../common/lc_voter.h"
#include "lc_cp_policy.h"
#if IS_ENABLED(CONFIG_PD_BATTERY_SECRET)
#include "../charger_class/xm_adapter_class.h"
#endif
#include "../charger_class/lc_fg_class.h"
#include "../charger_class/lc_charger_class.h"
#include "../fuelgauge/charger_partition.h"
#include "lc_charger_manager.h"

#define STEP_JEITA_TUPLE_NUM		9

#define COLD_RECHG_TEMP_OFFSET		20
#define COLD_RECHG_VOLT_OFFSET		100
#define HOT_RECHG_TEMP_OFFSET		20

#ifdef CONFIG_FACTORY_BUILD
#define WARM_STPCHG_TEMP_OFFSET		20
#define WARM_STPCHG_VOLT_OFFSET		100
#define WARM_RECHG_VOLT_OFFSET		200
#define WARM_RECHG_TEMP_OFFSET		(WARM_STPCHG_TEMP_OFFSET + 20)
#else
#define WARM_STPCHG_TEMP_OFFSET		0	// high-temp cutoff compensation
#define WARM_STPCHG_VOLT_OFFSET		0	// high-temp cutoff compensation
#define WARM_RECHG_TEMP_OFFSET		20	// high-temp recharge compensation
#define WARM_RECHG_VOLT_OFFSET		100	// high-temp recharge compensation
#endif

#define NORMAL_TERM_DELTA_CV        0
#define LOW_TEMP_RECHG_OFFSET       200
#define NOR_TEMP_RECHG_OFFSET       100

#define is_between(left, right, value) \
		(((left) >= (right) && (left) >= (value) \
			&& (value) >= (right)) \
		|| ((left) <= (right) && (left) <= (value) \
			&& (value) <= (right)))

#define POWER_REPLENISH_VOTER "POWER_REPLENISH_VOTER"
#define FCC_STEPPER_VOTER		"FCC_STEPPER_VOTER"
#define REPLENISH_LOOP_WAIT_S (3 * 1000)
#define REPLENISH_START_WAIT_S (300 * 1000)

struct lc_jeita_info *get_jeita_info(void);
int lc_jeita_init(struct device *dev);

void lc_jeita_deinit(void);
bool get_warm_stop_charge_state(void);
#ifdef CONFIG_LC_CP_POLICY
extern struct chargerpump_policy *g_policy;
#endif

enum jeita_temp_level {
	JEITA_TEMP_COLD,
	JEITA_TEMP_COOL,
	JEITA_TEMP_GOOD,
	JEITA_TEMP_WARM,
	JEITA_TEMP_HOT,
	JEITA_TEMP_MAX,
};

struct step_jeita_cfg {
	int low_threshold;
	int high_threshold;
	int value;
};
/*cycle count start*/
struct step_jeita_cycle_cfg {
	int vbat1;
	int vbat2;
	int vbat3;
	int cur_val1;
	int cur_val2;
};

struct step_jeita_normal_cfg {
	int cv_val;
	int cur_val;
};

struct step_jeita_ffc_iterm_cfg {
	int bat1_iterm;   /*NVT*/
	int bat2_iterm;  /*cos*/
	int bat3_iterm;  /*SWD*/
};

struct lc_jeita_info {
	struct device		*dev;
	ktime_t			jeita_last_update_time;
	bool			config_is_read;
	bool			sw_jeita_cfg_valid;
	bool			batt_missing;
	bool			taper_fcc;
	bool			step_chg_en;

	int			*jeita_cc_bounds;
	int			jeita_cc_bounds_len;
	int			*jeita_temp_bounds;
	int			jeita_temp_bounds_len;
	int			*jeita_cc_fcc_cfg;
	int			*jeita_cc_fv_cfg;

	int			jeita_cv_temp_len;

	int 			temp_level;
	int			jeita_temp_level[JEITA_TEMP_MAX];
	int			jeita_cfg_id;
	int			get_config_retry_count;
	int			fv;
	int			fv_offset_15_to_35;
	int			fv_offset_35_to_48;
	int			iterm;
	int			iterm_ffc;
	int			iterm_curr;
	int			under_4200_curr_offset;

	bool		pd_verifed;
	bool		is_charge_done;

	struct wakeup_source	*lc_jeita_ws;
	struct step_jeita_cfg step_chg_cfg[STEP_JEITA_TUPLE_NUM];

	/* voter add here */
	struct votable *main_fcc_votable;
	struct votable *fv_votable;
	struct votable *main_icl_votable;
	struct votable *iterm_votable;
	struct votable *total_fcc_votable;

	/* psy add here */
	struct power_supply	*batt_psy;
	struct power_supply	*bms_psy;
	struct power_supply	*usb_psy;
	#if IS_ENABLED(CONFIG_PD_BATTERY_SECRET)
	struct adapter_device *pd_adapter;
	#endif
	struct fuel_gauge_dev *fuel_gauge;
	struct charger_dev *charger;

	struct delayed_work	status_change_work;
	struct delayed_work	get_config_work;

	struct notifier_block nb;
};
#endif /* __LC_JEITA_H__ */
