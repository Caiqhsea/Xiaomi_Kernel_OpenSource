/*
 * miufs_cld1.c
 *
 *  Created on: 2020-10-20
 *  Author: shane
 */

#include "ufscld.h"
#include "ufscld_sysfs.h"


struct ufscld_dev *cld;

struct standard_inquiry {
	uint8_t vendor_id[8];
	uint8_t product_id[16];
	uint8_t product_rev[4];
};

#define CLD_DEBUG(cld, msg, args...)					\
	do { if (cld->cld_debug)					\
		pr_err("%40s:%3d [%01d%02d%02d] " msg "\n",		\
		       __func__, __LINE__,				\
			   cld->cld_trigger,				\
		       atomic_read(&cld->hba->dev->power.usage_count),\
			   cld->hba->clk_gating.active_reqs, ##args);	\
	} while (0)


extern struct ufscld_ops hg_cld_ops;
extern struct ufscld_ops samsung_cld_ops;
extern struct ufscld_ops ymtc_cld_ops;
extern struct ufscld_ops hynix_cld_ops;

struct vendor_ops cld_ops_arry[] = {
		{"SAMSUNG", AUTO_HIBERN8_ENABLE, &samsung_cld_ops},
  		{"SKhynix",AUTO_HIBERN8_DISABLE, &hynix_cld_ops},
		{"HG", AUTO_HIBERN8_DISABLE, &hg_cld_ops},
		{"XBSTOR", AUTO_HIBERN8_DISABLE, &ymtc_cld_ops},
  		{"YMTC", AUTO_HIBERN8_DISABLE, &ymtc_cld_ops},
};

int ufscld_init_ops(struct ufscld_dev *cld)
{
	struct standard_inquiry stdinq = {};
	int ret = -1;
	int i = 0;
	struct ufs_hba *hba = cld->hba;

	memcpy(&stdinq, hba->sdev_ufs_device->inquiry + 8, sizeof(stdinq));
	if(strncmp((char *)stdinq.product_id, "HBL19064G0CHBC",strlen("HBL19064G0CHBC")) == 0){				//64G HG操作函数和XB一样,不能使用其他的HG的flag
        	INFO_MSG("64G HG HBL19064G0CHBC USE OPS AS XB");
        	for (i = 0; i < sizeof(cld_ops_arry)/sizeof(cld_ops_arry[0]); i++) {
		if (strncmp("XBSTOR", (char *)cld_ops_arry[i].vendor_id, strlen((char *)cld_ops_arry[i].vendor_id)) == 0) {
			cld->cld_ops = cld_ops_arry[i].cld_ops;
			cld->vendor_ops = &cld_ops_arry[i];
			return 0;
			}
		}
        }
	for (i = 0; i < sizeof(cld_ops_arry)/sizeof(cld_ops_arry[0]); i++) {
		if (strncmp((char *)stdinq.vendor_id, (char *)cld_ops_arry[i].vendor_id, strlen((char *)cld_ops_arry[i].vendor_id)) == 0) {
			cld->cld_ops = cld_ops_arry[i].cld_ops;
			cld->vendor_ops = &cld_ops_arry[i];
			return 0;
		}
	}
	return ret;
}

int ufscld_is_not_present(struct ufscld_dev *cld)
{
	enum UFSCLD_STATE cur_state = ufscld_get_state(cld);

	if (cur_state != CLD_PRESENT) {
		INFO_MSG("cld_state != CLD_PRESENT (%d)", cur_state);
		return -ENODEV;
	}
	return 0;
}

inline int ufscld_get_state(struct ufscld_dev *cld)
{
	if(!cld){
		INFO_MSG("cld is not init, please check.");
		return CLD_NEED_INIT;
	}
	else
		return atomic_read(&cld->cld_state);
}

inline void ufscld_set_state(struct ufscld_dev *cld, int state)
{
	atomic_set(&cld->cld_state, state);
}

inline bool ufscld_schedule_delayed_work(struct delayed_work *dwork,
                                       unsigned long delay)
{
	return queue_delayed_work(system_freezable_wq, dwork, delay);
}

/*
 * Lock status: cld_sysfs lock was held when called.
 */
void ufscld_auto_hibern8_enable(struct ufscld_dev *cld,
				       unsigned int val)
{
	struct ufs_hba *hba = cld->hba;
	unsigned long flags;
	u32 reg;

	val = !!val;

	/* Update auto hibern8 timer value if supported */
	if (!ufshcd_is_auto_hibern8_supported(hba))
		return;

	ufshcd_rpm_get_sync(hba);
	ufshcd_hold(hba, false);
	ufshcd_scsi_block_requests(hba);
	/* wait for all the outstanding requests to finish */
	ufshcd_wait_for_doorbell_clr(hba, U64_MAX);
	spin_lock_irqsave(hba->host->host_lock, flags);

	reg = ufshcd_readl(hba, REG_AUTO_HIBERNATE_IDLE_TIMER);
	INFO_MSG("ahit-reg 0x%X", reg);

	if (val ^ (reg != 0)) {
		if (val) {
			hba->ahit = cld->ahit;
		} else {
			/*
			 * Store current ahit value.
			 * We don't know who set the ahit value to different
			 * from the initial value
			 */
			cld->ahit = reg;
			hba->ahit = 0;
		}

		ufshcd_writel(hba, hba->ahit, REG_AUTO_HIBERNATE_IDLE_TIMER);

		/* Make sure the timer gets applied before further operations */
		mb();

		INFO_MSG("[Before] is_auto_enabled %d", cld->is_auto_enabled);
		cld->is_auto_enabled = val;

		reg = ufshcd_readl(hba, REG_AUTO_HIBERNATE_IDLE_TIMER);
		INFO_MSG("[After] is_auto_enabled %d ahit-reg 0x%X",
			 cld->is_auto_enabled, reg);
	} else {
		INFO_MSG("is_auto_enabled %d. so it does not changed",
			 cld->is_auto_enabled);
	}

	spin_unlock_irqrestore(hba->host->host_lock, flags);
	ufshcd_scsi_unblock_requests(hba);
	ufshcd_release(hba);
	pm_runtime_put_sync(hba->dev);
}

void ufscld_block_enter_suspend(struct ufscld_dev *cld)
{
	struct ufs_hba *hba = cld->hba;
	unsigned long flags;

	if (unlikely(cld->block_suspend))
		return;

	cld->block_suspend = true;

	pm_runtime_get_sync(hba->dev);
	ufshcd_hold(hba, false);

	spin_lock_irqsave(hba->host->host_lock, flags);
	CLD_DEBUG(cld,
		  "dev->power.usage_count %d hba->clk_gating.active_reqs %d",
		  atomic_read(&hba->dev->power.usage_count),
		  hba->clk_gating.active_reqs);
	spin_unlock_irqrestore(hba->host->host_lock, flags);
}

void ufscld_allow_enter_suspend(struct ufscld_dev *cld)
{
	struct ufs_hba *hba = cld->hba;
	unsigned long flags;

	if (unlikely(!cld->block_suspend))
		return;

	cld->block_suspend = false;

	ufshcd_release(hba);
	pm_runtime_mark_last_busy(hba->dev);
	pm_runtime_put_noidle(hba->dev);

	spin_lock_irqsave(hba->host->host_lock, flags);
	CLD_DEBUG(cld,
		  "dev->power.usage_count %d hba->clk_gating.active_reqs %d",
		  atomic_read(&hba->dev->power.usage_count),
		  hba->clk_gating.active_reqs);
	spin_unlock_irqrestore(hba->host->host_lock, flags);
}

int ufscld_get_frag_level(struct ufscld_dev *cld, int *frag_level)
{

	int ret;

	ret = cld->cld_ops->cld_get_frag_level(cld, frag_level);

	return ret;
}

int ufscld_get_operation_status(struct ufscld_dev *cld, int *op_status)
{

	int ret;

	ret = cld->cld_ops->cld_operation_status(cld, (u32 *)op_status);

	return ret;
}

static int ufscld_issue_enable(struct ufscld_dev *cld)
{
	if (cld->cld_ops->cld_set_trigger(cld, 1))
		return -EINVAL;
	return 0;
}

static int ufscld_issue_disable(struct ufscld_dev *cld)
{
	int frag_level;

	if (cld->cld_ops->cld_set_trigger(cld, 0))
		return -EINVAL;
	if (cld->cld_ops->cld_get_frag_level(cld, &frag_level))
		return -EINVAL;

	CLD_DEBUG(cld, "Frag_lv %d \n", frag_level);

	return 0;
}

static void trigger_uevent(struct ufscld_dev *cld, int trigger)
{
	char *cld_trigger_on[2]  = { "CLD_TRIGGER=ON", NULL };
	char *cld_trigger_off[2] = { "CLD_TRIGGER=OFF", NULL };

	if (trigger) {
		kobject_uevent_env(&cld->hba->dev->kobj, KOBJ_CHANGE, cld_trigger_on);
		INFO_MSG("UFSCLD: sent uevent %s\n", cld_trigger_on[0]);
	} else {
		kobject_uevent_env(&cld->hba->dev->kobj, KOBJ_CHANGE, cld_trigger_off);
		INFO_MSG("UFSCLD: sent uevent %s\n", cld_trigger_off[0]);
	}
}

/*
 * If the return value is not err, pm_runtime_put_noidle() must be called once.
 * IMPORTANT : ufshid_hold_runtime_pm() & ufshid_release_runtime_pm() pair.
 */
int ufscld_hold_runtime_pm(struct ufscld_dev *cld)
{
	struct ufs_hba *hba = cld->hba;

	if (ufscld_get_state(cld) == CLD_SUSPEND) {
		/* Check that device was suspended by System PM */
		ufshcd_rpm_get_sync(cld->hba);

		/* If it success, device was suspended by Runtime PM */
		if (ufscld_get_state(cld) == CLD_PRESENT &&
		    hba->curr_dev_pwr_mode == UFS_ACTIVE_PWR_MODE &&
		    hba->uic_link_state == UIC_LINK_ACTIVE_STATE)
			goto resume_success;

		INFO_MSG("RPM resume failed. Maybe it was SPM suspend");
		INFO_MSG("UFS state (POWER = %d LINK = %d)",
			 hba->curr_dev_pwr_mode, hba->uic_link_state);

		pm_runtime_mark_last_busy(hba->dev);
		ufshcd_rpm_put_sync(cld->hba);
		return -ENODEV;
	}

	if (ufscld_is_not_present(cld))
		return -ENODEV;

	ufshcd_rpm_get_sync(cld->hba);

resume_success:
	return 0;
}

inline void ufscld_release_runtime_pm(struct ufscld_dev *cld)
{
	struct ufs_hba *hba = cld->hba;

	pm_runtime_mark_last_busy(hba->dev);
	ufshcd_rpm_put_sync(cld->hba);
}

/*
 * Lock status: cld_sysfs lock was held when called.
 */
int ufscld_trigger_on(struct ufscld_dev *cld)
{
	int ret;
	if (unlikely(cld->cld_trigger))
		return 0;

	ret = ufscld_hold_runtime_pm(cld);
	if (ret)
		return ret;

	cld->cld_trigger = true;

	ufscld_block_enter_suspend(cld);

	if (cld->vendor_ops->auto_hibern8_enable) {
		ufscld_auto_hibern8_enable(cld, 0);
	}

	cld->start_time_stamp = ktime_to_ms(ktime_get());
	CLD_DEBUG(cld, "at %d trigger 0 -> 1", cld->start_time_stamp);
	INFO_MSG("ufscld_trigger_on: at %d trigger 0 -> 1", cld->start_time_stamp);
	ufscld_schedule_delayed_work(&cld->cld_trigger_work, 0);

	ufscld_release_runtime_pm(cld);

	return 0;
}

/*
 * Lock status: cld_sysfs lock was held when called.
 */
int ufscld_trigger_off(struct ufscld_dev *cld)
{
	int ret;
	if (unlikely(!cld->cld_trigger))
		return 0;

	ret = ufscld_hold_runtime_pm(cld);
	if (ret)
		return ret;

	cld->cld_trigger = false;
	CLD_DEBUG(cld, "cld_trigger 1 -> 0");
	INFO_MSG("ufscld_trigger_off: at %d trigger 1 -> 0", cld->start_time_stamp);
	ufscld_issue_disable(cld);

	if (cld->vendor_ops->auto_hibern8_enable) {
		ufscld_auto_hibern8_enable(cld, 1);
	}

	ufscld_allow_enter_suspend(cld);

	trigger_uevent(cld, 0);

	ufscld_release_runtime_pm(cld);

	return 0;
}



static void ufscld_trigger_work_fn(struct work_struct *dwork)
{
	struct ufscld_dev *cld;
	int ret;
	enum CLD_STATUS op_status;
	ktime_t work_delta_time = 0;

	cld = container_of(dwork, struct ufscld_dev, cld_trigger_work.work);


	if (ufscld_is_not_present(cld)) {
		ERR_MSG("ufscld state is = %d\n", cld->cld_state);
		return;
	}

	ret = ufscld_hold_runtime_pm(cld);
	if (ret)
		goto pm_put;

	if(!cld->block_suspend) {
		ufscld_block_enter_suspend(cld);
		if (cld->vendor_ops->auto_hibern8_enable)
			ufscld_auto_hibern8_enable(cld, 0);
	}

	ufscld_get_operation_status(cld, (int *)&op_status);
	CLD_DEBUG(cld, "cld status %d\n", op_status);

	mutex_lock(&cld->sysfs_lock);
	if (!cld->cld_trigger) { // if host manual stop cld
		CLD_DEBUG(cld, "cld_trigger == false, return");
		mutex_unlock(&cld->sysfs_lock);
		goto pm_put;
	}
	ret = ufscld_issue_enable(cld);
	if (ret) {
		ERR_MSG("ufscld_issue_enable failed ret=%d\n", ret);
	}
	work_delta_time = ktime_to_ms(ktime_get()) - cld->start_time_stamp;
	INFO_MSG("ufscld status %d, work_delta_time %lld ms\n", op_status, work_delta_time);
	if (CLD_STATUS_IDLE == op_status || work_delta_time >= CLD_TRIGGER_WORKER_TIMEOUT_MS) { //cld entry idel or timeout
	WARN_MSG("CLD off with status (%d), duration (%d ms)", op_status, work_delta_time);
		ufscld_trigger_off(cld);
		mutex_unlock(&cld->sysfs_lock);
		goto pm_put;
	} else if (CLD_STATUS_PROGRESSING == op_status) {
		CLD_DEBUG(cld, "cld_REQUIRED, so sched (%d ms)",
				cld->cld_trigger_delay);
	} else {
		CLD_DEBUG(cld, "issue_cld ERR(%X), so resched for retry",
			  ret);
	}

	mutex_unlock(&cld->sysfs_lock);

	ufscld_schedule_delayed_work(&cld->cld_trigger_work,
			      msecs_to_jiffies(cld->cld_trigger_delay));

pm_put:
	ufscld_release_runtime_pm(cld);

	CLD_DEBUG(cld, "end cld_trigger_work_fn");
}

void ufscld_suspend(struct ufscld_dev *cld)
{
	if (!cld)
		return;

	if (unlikely(cld->cld_trigger))
		ERR_MSG("cld_trigger was set to block the suspend. so weird");
	ufscld_set_state(cld, CLD_SUSPEND);

	cancel_delayed_work_sync(&cld->cld_trigger_work);
}

void ufscld_resume(struct ufscld_dev *cld)
{
	if (!cld)
		return;

	if (unlikely(cld->cld_trigger))
		ERR_MSG("cld_trigger need to off");
	ufscld_set_state(cld, CLD_PRESENT);
}

/*
 * this function is called in irq context.
 * so cancel_delayed_work_sync() do not use due to waiting.
 */
void ufscld_on_idle(struct ufs_hba *hba)
{
	if(!cld)
		return;

	if (!cld->cld_trigger)
		return;// cld already done or not triggered.

	if (ufscld_get_state(cld) != CLD_PRESENT || hba->outstanding_reqs) {
		return; // HID is not avalible or this is not the last cmd.
	}

	if (!cld->cld_ops) // cld no supported
		return;
	/*
	 * When cld_trigger_work will be scheduled,
	 * check cld_trigger under sysfs_lock.
	 */

	if (delayed_work_pending(&cld->cld_trigger_work))
		cancel_delayed_work(&cld->cld_trigger_work);

	ufscld_schedule_delayed_work(&cld->cld_trigger_work, msecs_to_jiffies(1000));// if there is no new cmd in 1S.
}

void ufscld_init(struct ufs_hba *hba)
{
	int ret = 0;

	cld = kzalloc(sizeof(struct ufscld_dev), GFP_KERNEL);

	cld->hba = hba;

	cld->cld_trigger = false;

	ret = ufscld_init_ops(cld);
	if (ret) {
		ERR_MSG("CLD get cld ops fail.\n");
		return;
	} else {
		INFO_MSG("CLD init ops succeed\n");
	}

	cld->cld_trigger_delay = CLD_TRIGGER_WORKER_DELAY_MS_DEFAULT;
	INIT_DELAYED_WORK(&cld->cld_trigger_work, ufscld_trigger_work_fn);

	cld->cld_debug = false;
	cld->block_suspend = false;
	cld->start_time_stamp = 0;

	/* Save default Auto-Hibernate Idle Timer register value */
	cld->ahit = hba->ahit;

	/* If HCI supports auto hibern8, UFS Driver use it default */
	if (ufshcd_is_auto_hibern8_supported(cld->hba))
		cld->is_auto_enabled = true;
	else
		cld->is_auto_enabled = false;

	ret = ufscld_create_sysfs(cld);
	if (ret) {
		ERR_MSG("ufscld sysfs create failed.\n");
		ufscld_set_state(cld, CLD_FAILED);
		kfree(cld);
		return;
	}

	INFO_MSG("UFS cld create sysfs finished.\n");

	ufscld_set_state(cld, CLD_PRESENT);

}

void ufscld_remove(struct ufs_hba *hba)
{
	if(!cld)
		return;

	ufscld_set_state(cld, CLD_FAILED);

	cancel_delayed_work_sync(&cld->cld_trigger_work);

	mutex_lock(&cld->sysfs_lock);
	ufscld_allow_enter_suspend(cld);
	ufscld_trigger_off(cld);
	ufscld_remove_sysfs(cld);
	mutex_unlock(&cld->sysfs_lock);

	kfree(cld);

	INFO_MSG("CLD released\n");
}
