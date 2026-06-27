/*
 * miufs_cld.h
 *
 *  Created on: 2020-10-20
 *  Author: shane
 */

#ifndef __DRIVERS_SCSI_UFS_MI_CLD_H__
#define __DRIVERS_SCSI_UFS_MI_CLD_H__
#include "../mi_ufshcd.h"

#define INFO_MSG(msg, args...)		pr_info("%s:%d info: " msg "\n", \
					       __func__, __LINE__, ##args)
#define ERR_MSG(msg, args...)		pr_err("%s:%d err: " msg "\n", \
					       __func__, __LINE__, ##args)
#define WARN_MSG(msg, args...)		pr_warn("%s:%d warn: " msg "\n", \
					       __func__, __LINE__, ##args)


#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/blktrace_api.h>
#include <linux/blkdev.h>
#include <linux/bitfield.h>
#include <linux/ktime.h>
#include <scsi/scsi_cmnd.h>
#include <linux/workqueue.h>
#include "../../../block/blk.h"

#define CLD_TRIGGER_WORKER_DELAY_MS_DEFAULT	2000
#define CLD_TRIGGER_WORKER_DELAY_MS_MIN		100
#define CLD_TRIGGER_WORKER_DELAY_MS_MAX		10000
#define CLD_TRIGGER_WORKER_TIMEOUT_MS		(420 * 1000)
#define CLD_AUTO_HIBERN8_DISABLE  (FIELD_PREP(UFSHCI_AHIBERN8_TIMER_MASK, 0) | \
				   FIELD_PREP(UFSHCI_AHIBERN8_SCALE_MASK, 3))

enum UFSCLD_STATE {
	CLD_NEED_INIT = 0,
	CLD_PRESENT = 1,
	CLD_SUSPEND = 2,
	CLD_FAILED = -2,
	CLD_RESET = -3,
};

enum {
	CLD_OP_DISABLE	= 0,
	CLD_OP_ANALYZE	= 1,
	CLD_OP_EXECUTE	= 2,
	CLD_OP_MAX
};

enum {
	CLD_NOT_REQUIRED	= 0,
	CLD_REQUIRED		= 1
};

enum CLD_STATUS {
	CLD_STATUS_NA = 0,
	CLD_STATUS_IDLE = 1,
	CLD_STATUS_PROGRESSING = 2,
};

enum CLD_LEVEL {
	CLD_LEV_NA	= 0,
	CLD_LEV_CLEAN	= 1,
	CLD_LEV_WARN	= 2,
	CLD_LEV_CRITICAL	= 3,
};

enum {
	AUTO_HIBERN8_DISABLE	= 0,
	AUTO_HIBERN8_ENABLE	= 1,
};

struct vendor_ops {
	uint8_t vendor_id[8];
	uint8_t auto_hibern8_enable;
	struct ufscld_ops *cld_ops;
};

struct ufscld_dev {
	struct ufs_hba *hba;
	unsigned int cld_trigger;   /* default value is false */
	struct delayed_work cld_trigger_work;
	unsigned int cld_trigger_delay;

	bool is_auto_enabled;
	ktime_t start_time_stamp;

	atomic_t cld_state;
	/* for sysfs */
	struct kobject kobj;
	struct mutex sysfs_lock;
	struct ufscld_sysfs_entry *sysfs_entries;

	struct ufscld_ops *cld_ops;
	struct vendor_ops *vendor_ops;
	/* for debug */
	bool cld_debug;

	bool block_suspend;

	u32 ahit;			/* to restore ahit value */
};

struct ufscld_ops {
	int (*cld_get_frag_level)(struct ufscld_dev *cld, int *frag_level);
	int (*cld_set_trigger)(struct ufscld_dev *cld, u32 trigger);
	int (*cld_get_trigger)(struct ufscld_dev *cld, u32 *trigger);
	int (*cld_operation_status)(struct ufscld_dev *cld, int *status);
};

struct ufscld_sysfs_entry {
	struct attribute attr;
	ssize_t (*show)(struct ufscld_dev *cld, char *buf);
	ssize_t (*store)(struct ufscld_dev *cld, const char *buf, size_t count);
};

extern struct ufscld_dev *cld;
int ufscld_get_state(struct ufscld_dev *hba);
void ufscld_set_state(struct ufscld_dev *hba, int state);
bool ufscld_schedule_delayed_work(struct delayed_work *dwork,unsigned long delay);
void ufscld_set_init_state(struct ufs_hba *hba);
void ufscld_init(struct ufs_hba *hba);
void ufscld_reset(struct ufs_hba *hba);
void ufscld_reset_host(struct ufs_hba *hba);
void ufscld_remove(struct ufs_hba *hba);
void ufscld_on_idle(struct ufs_hba *hba);
int ufscld_trigger_on(struct ufscld_dev *cld);
int ufscld_trigger_off(struct ufscld_dev *cld);
int ufscld_get_frag_level(struct ufscld_dev *cld, int *frag_level);
int ufscld_is_not_present(struct ufscld_dev *cld);
int ufscld_get_operation_status(struct ufscld_dev *cld, int *op_status);
void ufscld_block_enter_suspend(struct ufscld_dev *cld);
void ufscld_allow_enter_suspend(struct ufscld_dev *cld);
void ufscld_auto_hibern8_enable(struct ufscld_dev *cld, unsigned int val);
int ufscld_hold_runtime_pm(struct ufscld_dev *cld);
void ufscld_release_runtime_pm(struct ufscld_dev *cld);
void ufscld_suspend(struct ufscld_dev *cld);
void ufscld_resume(struct ufscld_dev *cld);

#endif /* DRIVERS_SCSI_UFS_MI_CLD_H_ */