/*
 * include/linux/power/sm5602_fg.h
 *
 * Copyright (C) 2018 SiliconMitus
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
#ifndef SM5602_FG_H
#define SM5602_FG_H

#include "../charger_class/lc_fg_class.h"
#include "../charger_class/lc_charger_class.h"
#include "../battery_secret/battery_secret_class.h"
#include "../battery_secret/battery_secret_logic.h"
#include "../battery_secret/secret_common.h"
#include "../lc_printk.h"

#ifndef POWER_SUPPLY_PROP_SOH
#define POWER_SUPPLY_PROP_SOH     128
#endif

#define FG_INIT_MARK              0xA000

#define FG_PARAM_UNLOCK_CODE      0x3700
#define FG_PARAM_LOCK_CODE        0x0000
#define FG_TABLE_LEN              0x18
#define FG_ADD_TABLE_LEN          0x8
#define FG_INIT_B_LEN             0x7
#define FG_TABLE_MAX_LEN          0x18

#define ENABLE_EN_TEMP_IN         0x0200
#define ENABLE_EN_TEMP_EX         0x0400
#define ENABLE_EN_BATT_DET        0x0800
#define ENABLE_IOCV_MAN_MODE      0x1000
#define ENABLE_FORCED_SLEEP       0x2000
#define ENABLE_SLEEPMODE_EN       0x4000
#define ENABLE_SHUTDOWN           0x8000

#define ENABLE_EN_SHUTDOWN        0x0001

/* REG */
#define FG_REG_SOC_CYCLE          0x0B
#define FG_REG_SOC_CYCLE_CFG      0x15
#define FG_REG_BATT_ID		  0x1F
#define FG_REG_ALPHA              0x20
#define FG_REG_BETA               0x21
#define FG_REG_RS                 0x24
#define FG_REG_RS_1               0x25
#define FG_REG_RS_2               0x26
#define FG_REG_RS_3               0x27
#define FG_REG_RS_0               0x29
#define FG_REG_END_V_IDX          0x2F
#define FG_REG_START_LB_V         0x30
#define FG_REG_START_CB_V         0x38
#define FG_REG_START_LB_I         0x40
#define FG_REG_START_CB_I         0x48
#define FG_REG_VOLT_CAL           0x70
#define FG_REG_CURR_IN_OFFSET     0x75
#define FG_REG_CURR_IN_SLOPE      0x76
#define FG_REG_RMC                0x84
#define FG_REG_MISC2              0x93

#define FG_REG_SRADDR             0x8C
#define FG_REG_SRDATA             0x8D
#define FG_REG_SWADDR             0x8E
#define FG_REG_SWDATA             0x8F

#define FG_REG_AGING_CTRL         0x9C

#define FG_TEMP_TABLE_CNT_MAX     0x65

#define FG_SRAM_SOH				  0x71

#define I2C_ERROR_COUNT_MAX       0x5

#define FG_PARAM_VERION           0x1F
#define FG_USER_RESERVED       	  0x1E

#define INIT_CHECK_MASK           0x0010
#define DISABLE_RE_INIT           0x0010

/******************************************************************/
#define    SOC_SMOOTH_TRACKING
//#define    FG_ENABLE_IRQ
#define      ENABLE_IOCV_ADJ
#define      ENABLE_INSPECTION_TABLE
#define      ENABLE_TEMBASE_ZDSCON
#define      ENABLE_TEMP_AVG

//#define  SOC_SMOOTH_TRACKING    ENABLE_WAIT_SOC_FULL
#define    ENABLE_LTIM_ACT
#define      ENABLE_HCIC_ACT
#define    ENABLE_HCRSM_MODE
//#define    ENABLE_FIT_ZR_NCUR
#define      CHECK_ABNORMAL_TABLE
//#define          DELAY_SOC_ZERO
//#define    ENABLE_FCHG_VSOC_MODE
#define      STATIC_SCALE_SOC
#define FFC_SMOOTH_LEN                  4

#define FG_MONITOR_DELAY_3S             3000
#define FG_MONITOR_DELAY_5S             5000
#define FG_MONITOR_DELAY_10S            10000
#define FG_MONITOR_DELAY_30S            30000
/******************************************************************/

#ifdef ENABLE_TEMP_AVG
#define CHANGE_TEMP_TIME_LIMIT_1      1 //1sec
#define CHANGE_TEMP_TIME_LIMIT_3      3 //3sec
#define CHANGE_TEMP_TIME_LIMIT_5      5 //5sec
#define BATT_TEMP_AVG_SAMPLES         8
#endif

/******************************************************************/
enum sm_fg_reg_idx {
        SM_FG_REG_DEVICE_ID = 0,
        SM_FG_REG_CNTL,
        SM_FG_REG_INT,
        SM_FG_REG_INT_MASK,
        SM_FG_REG_STATUS,
        SM_FG_REG_SOC,
        SM_FG_REG_OCV,
        SM_FG_REG_VOLTAGE,
        SM_FG_REG_CURRENT,
        SM_FG_REG_TEMPERATURE_IN,
        SM_FG_REG_TEMPERATURE_EX,
        SM_FG_REG_V_L_ALARM,
        SM_FG_REG_V_H_ALARM,
        SM_FG_REG_A_H_ALARM,
        SM_FG_REG_T_IN_H_ALARM,
        SM_FG_REG_SOC_L_ALARM,
        SM_FG_REG_FG_OP_STATUS,
        SM_FG_REG_TOPOFFSOC,
        SM_FG_REG_PARAM_CTRL,
        SM_FG_REG_SHUTDOWN,
        SM_FG_REG_VIT_PERIOD,
        SM_FG_REG_CURRENT_RATE,
        SM_FG_REG_BAT_CAP,
        SM_FG_REG_CURR_OFFSET,
        SM_FG_REG_CURR_SLOPE,
        SM_FG_REG_MISC,
        SM_FG_REG_RESET,
        SM_FG_REG_RSNS_SEL,
        SM_FG_REG_VOL_COMP,
        NUM_REGS,
};

static u8 sm5602_regs[NUM_REGS] = {
        0x00, /* DEVICE_ID */
        0x01, /* CNTL */
        0x02, /* INT */
        0x03, /* INT_MASK */
        0x04, /* STATUS */
        0x05, /* SOC */
        0x06, /* OCV */
        0x07, /* VOLTAGE */
        0x08, /* CURRENT */
        0x09, /* TEMPERATURE_IN */
        0x0A, /* TEMPERATURE_EX */
        0x0C, /* V_L_ALARM */
        0x0D, /* V_H_ALARM */
        0x0E, /* A_H_ALARM */
        0x0F, /* T_IN_H_ALARM */
        0x10, /* SOC_L_ALARM */
        0x11, /* FG_OP_STATUS */
        0x12, /* TOPOFFSOC */
        0x13, /* PARAM_CTRL */
        0x14, /* SHUTDOWN */
        0x1A, /* VIT_PERIOD */
        0x1B, /* CURRENT_RATE */
        0x62, /* BAT_CAP */
        0x73, /* CURR_OFFSET */
        0x74, /* CURR_SLOPE */
        0x90, /* MISC */
        0x91, /* RESET */
        0x95, /* RSNS_SEL */
        0x96, /* VOL_COMP */
};

enum sm_fg_device {
        SM5602,
};

enum sm_fg_temperature_type {
        TEMPERATURE_IN = 0,
        TEMPERATURE_EX,
        TEMPERATURE_3RD,
};

enum battery_table_type {
        BATTERY_TABLE0 = 0,
        BATTERY_TABLE1,
        BATTERY_TABLE2,
        BATTERY_TABLE_MAX,
};

#ifdef SOC_SMOOTH_TRACKING
#define BATT_MA_AVG_SAMPLES     8
struct batt_params {
        bool                    update_now;
        int                             batt_raw_soc;
        int                             batt_soc;
        int                             samples_num;
        int                             samples_index;
        int                             batt_ma_avg_samples[BATT_MA_AVG_SAMPLES];
        int                             batt_ma_avg;
        int                             batt_ma_prev;
        int                             batt_ma;
        int                             batt_mv;
        int                             batt_temp;
	ktime_t				last_soc_change_time;
};
#endif

#ifdef ENABLE_TEMP_AVG

struct batt_temp_params {
        bool            update_now;
        int                     batt_raw_temp;
        int                     batt_temp;
        int                     samples_num;
        int                     samples_index;
        int                     batt_temp_avg_samples[BATT_TEMP_AVG_SAMPLES];
        int                     batt_temp_avg;
        int                     batt_temp_prev;
};
#endif

struct last_read_time {
	ktime_t rsoc_read_time;
	ktime_t temp_read_time;
	ktime_t vbat_read_time;
	ktime_t ibat_read_time;
};

struct recover_params {
	int rsoc;
	int temp;
	int vbat;
	int curr;
	int fcc;
	int rmc;
	int soh;
	int cycle;
};

struct sm_fg_chip {
        struct device           *dev;
        struct i2c_client       *client;
        struct mutex i2c_rw_lock; /* I2C Read/Write Lock */
        struct mutex data_lock; /* Data Lock */
        u8 chip;
        u8 regs[NUM_REGS];
        int     batt_id;
        int gpio_int;

        /* Status Tracking */
        bool batt_present;
        bool batt_fc;   /* Battery Full Condition */
        bool batt_ot;   /* Battery Over Temperature */
        bool batt_ut;   /* Battery Under Temperature */
        bool batt_soc1; /* SOC Low */
        bool batt_socp; /* SOC Poor */
        bool batt_dsg;  /* Discharge Condition*/
        struct wakeup_source *bms_wakelock;
        int     batt_soc;
        int batt_ocv;
        int batt_fcc;   /* Full charge capacity */
        int     batt_volt;
        int     aver_batt_volt;
        int     batt_temp;
        int     batt_curr;
        int batt_rmc;
        int is_charging;        /* Charging informaion from charger IC */
        int batt_soc_cycle; /* Battery SOC cycle */
        int batt_soh; /* Battery SOH */
        int cycle_count_new;
        int last_batt_soh;
        int last_batt_cycle;
        int last_chip_soh;
        int last_chip_cycle;
        int topoff_soc;
        int topoff_margin;
        int top_off;
        int iocv_error_count;
#ifdef SOC_SMOOTH_TRACKING
        bool    soc_reporting_ready;
#endif
#ifdef SHUTDOWN_DELAY
        bool    shutdown_delay_enable;
        bool    shutdown_delay;
#endif
	/* previous battery voltage current*/
        int p_batt_voltage;
        int p_batt_current;
        int p_report_soc;

        /* DT */
        bool en_temp_ex;
        bool en_temp_in;
        bool en_temp_3rd;
        bool en_batt_det;
        bool iocv_man_mode;
        int aging_ctrl;
        int batt_rsns;  /* Sensing resistor value */
        int cycle_cfg;
        int fg_irq_set;
        int low_soc1;
        int low_soc2;
        int v_l_alarm;
        int v_h_alarm;
        int battery_table_num;
        int misc;
        int misc2;
        int batt_v_max;
        int min_cap;
        u32 common_param_version;
        int t_l_alarm_in;
        int t_h_alarm_in;
        u32 t_l_alarm_ex;
        u32 t_h_alarm_ex;
        int rpara;
        int curr_voffset;
        int curr_vslope;

	/* Battery Data */
        int battery_table[BATTERY_TABLE_MAX][FG_TABLE_LEN];
        signed short battery_temp_table[FG_TEMP_TABLE_CNT_MAX]; /* Degree */
        int alpha;
        int beta;
        int rs;
        int rs_value[4];
        int vit_period;
        int mix_value;
        const char              *battery_type;
        int volt_cal;
        int curr_offset;
        int curr_slope;
        int cap;
        int n_tem_poff;
        int n_tem_poff_offset;
        int batt_max_voltage_uv;
        int temp_std;
        int en_high_fg_temp_offset;
        int high_fg_temp_offset_denom;
        int high_fg_temp_offset_fact;
        int en_low_fg_temp_offset;
        int low_fg_temp_offset_denom;
        int low_fg_temp_offset_fact;
        int en_high_fg_temp_cal;
        int high_fg_temp_p_cal_denom;
        int high_fg_temp_p_cal_fact;
        int high_fg_temp_n_cal_denom;
        int high_fg_temp_n_cal_fact;
        int en_low_fg_temp_cal;
        int low_fg_temp_p_cal_denom;
        int low_fg_temp_p_cal_fact;
        int low_fg_temp_n_cal_denom;
        int low_fg_temp_n_cal_fact;
        int     en_high_temp_cal;
        int high_temp_p_cal_denom;
        int high_temp_p_cal_fact;
        int high_temp_n_cal_denom;
        int high_temp_n_cal_fact;
        int en_low_temp_cal;
        int low_temp_p_cal_denom;
        int low_temp_p_cal_fact;
        int low_temp_n_cal_denom;
        int low_temp_n_cal_fact;
        u32 battery_param_version;

#ifdef ENABLE_NTC_COMPENSATION_1
        int rtrace;
#endif

        struct delayed_work monitor_work;

#ifdef DELAY_SOC_ZERO
        struct delayed_work delay_soc_zero_work;
#endif

        struct delayed_work update_delay_work;

#ifdef SOC_SMOOTH_TRACKING
        int charge_full;
#endif
        //unsigned long last_update;

#ifdef DELAY_SOC_ZERO
        int en_delay_soc_zero;
        int recover_delay_soc_zero;
#endif

#ifdef STATIC_SCALE_SOC
        int en_static_scale_soc;
        int static_scale_soc_base_min;
        int static_scale_soc_base_max;
        int off_current_min;
        int off_current_max;
        int rsoc_delta_scale_limit;
        int     last_rsoc;
        int static_scale_soc_base_value;
        int static_scale_soc_rise_count;
#endif

	/* Debug */
        int     skip_reads;
        int     skip_writes;
        int fake_soc;
        int fake_temp;
        int fake_cycle_count;
        struct dentry *debug_root;
        struct power_supply *batt_psy;
        struct power_supply *charger_psy;
        struct power_supply *fg_psy;
#ifdef SOC_SMOOTH_TRACKING
        struct batt_params      param;
#endif
#ifdef ENABLE_TEMP_AVG
        struct batt_temp_params temp_param;
#endif
      	int monitor_delay;
        int rsoc_smooth;
	int charging_status;
	int dev_id;
        bool authenticate;
        int input_suspend;
	int ui_soc;
        int raw_soc;
	int last_soc;
	int rsoc;
        int ch;
	int mtbf_current;
	bool chip_ok;
	bool first_flag;
	bool fast_chg;
	int batt_id_volt;
        bool shipmode;
	int pmic_fv;

	struct recover_params recover;
	struct last_read_time last_rt;

	struct iio_channel	*bat_id_vol;
	struct iio_chan_spec    *iio_chan;
	struct iio_channel      *int_iio_chans;
	struct iio_channel	**nopmi_chg_iio;
	struct iio_dev  *indio_dev;
	struct iio_channel	**main_iio;

        struct mutex rw_lock;
        u8 mi_infoC[32];
        int mi_infoC_valid;
        u8 bat_sn[32];

        struct fuel_gauge_dev *fuel_gauge;
        struct charger_dev *charger;
        struct class qcom_batt_class;
        struct w1_data *w1sec_info;
};

enum nopmi_chg_iio_channels {
  	NOPMI_CHG_USB_REAL_TYPE,
};

static const char * const nopmi_chg_iio_chan_name[] = {
	[NOPMI_CHG_USB_REAL_TYPE] = "usb_real_type",
};

enum main_iio_channels {
	MAIN_CHARGE_DONE,
};

static const char * const main_iio_chan_name[] = {
	[MAIN_CHARGE_DONE] = "charge_done",
};

enum bq_fg_mac_cmd {
	FG_MAC_CMD_CTRL_STATUS	= 0x0000,
	FG_MAC_CMD_DEV_TYPE	= 0x0001,
	FG_MAC_CMD_FW_VER	= 0x0002,
	FG_MAC_CMD_HW_VER	= 0x0003,
	FG_MAC_CMD_IF_SIG	= 0x0004,
	FG_MAC_CMD_CHEM_ID	= 0x0006,
	FG_MAC_CMD_SHUTDOWN	= 0x0010,
	FG_MAC_CMD_GAUGING	= 0x0021,
	FG_MAC_CMD_SEAL		= 0x0030,
	FG_MAC_CMD_TO_FULL	= 0x0031,
	FG_MAC_CMD_FASTCHARGE_EN = 0x003E,
	FG_MAC_CMD_FASTCHARGE_DIS = 0x003F,
	FG_MAC_CMD_DEV_RESET	= 0x0041,
	FG_MAC_CMD_DEVICE_NAME	= 0x004A,
	FG_MAC_CMD_DEVICE_CHEM	= 0x004B,
	FG_MAC_CMD_MANU_NAME	= 0x004C,
	FG_MAC_CMD_MANU_DATE    = 0x004D,
	FG_MAC_CMD_BATT_SERL    = 0x004E,
	FG_MAC_CMD_CHARGING_STATUS = 0x0055,
	FG_MAC_CMD_LIFETIME1	= 0x0060,
	FG_MAC_CMD_LIFETIME3	= 0x0062,
	FG_MAC_CMD_NTC_TEMP     = 0x006A,
	FG_MAC_CMD_MANU_INFO	= 0x0070,
	FG_MAC_CMD_DASTATUS1	= 0x0071,
	FG_MAC_CMD_ITSTATUS1	= 0x0073,
	FG_MAC_CMD_QMAX		= 0x0075,
	FG_MAC_CMD_FCC_SOH	= 0x0077,
	FG_MAC_CMD_MANU_INFOC = 0x007B,
	FG_MAC_CMD_RA_TABLE	= 0x40C0,
	FG_MAC_CMD_OPR_STAT	= 0x0054,
	FG_MAC_CMD_FG_STAT  = 0x4440,
};

struct r_item{
	int batt_id;
	int bat_resistance_id;
	const char* manufacturer;
	const char* batt_type;
};

static const struct r_item r_items_param[] = {
	//100k
	{
		.batt_id = 0,
		.manufacturer = "UNKNOWN",
		.bat_resistance_id = 100000,
		.batt_type = "UNKNOWN",
	},

	//330k
	{
		.batt_id = 1,
		.manufacturer = "SWD",
		.bat_resistance_id = 330000,
		.batt_type = "XN56_SWD_330K_7600mAh",
	},

	//68k
	{
		.batt_id = 2,
		.manufacturer = "NVT",
		.bat_resistance_id = 68000,
		.batt_type = "XN56_NVT_68K_7600mAh",
	},

	//68k
	{
		.batt_id = 3,
		.manufacturer = "SWD_EEA",
		.bat_resistance_id = 330000,
		.batt_type = "XN56_SWD_330K_7600mAh",
	},

	//330k
	{
		.batt_id = 4,
		.manufacturer = "NVT_EEA",
		.bat_resistance_id = 68000,
		.batt_type = "XN56_NVT_68K_7600mAh",
	},
};

static const struct r_item *r_items_param_fg = r_items_param;
/******************************************************************/
#endif /* SM5602_FG_H */
