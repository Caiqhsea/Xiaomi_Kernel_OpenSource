
#include <linux/reboot.h>
#include <asm-generic/unaligned.h>
#include "../mi_ufshcd.h"
#include "../mi_ufs.h"
#include "../../ufs/ufs-qcom.h"
//extern int ufs_ffu_reboot_reason_reboot(void *ptr);
int check_device_wb_sup(struct ufs_hba *hba, const u8 *desc_buf)
{
	int err = 0;
	u8 lun;
	u32 d_lu_wb_buf_alloc;
	u32 ext_ufs_feature;
	struct ufs_dev_info *dev_info = &hba->dev_info;
	if (!ufshcd_is_wb_allowed(hba)) {
		err = -1;
		goto wb_check_fail;
	}
	ext_ufs_feature = get_unaligned_be32(
		desc_buf + DEVICE_DESC_PARAM_EXT_UFS_FEATURE_SUP);
	if (!(ext_ufs_feature & UFS_DEV_WRITE_BOOSTER_SUP)) {
		err = -2;
		goto wb_check_fail;
	}
	dev_info->wb_buffer_type = desc_buf[DEVICE_DESC_PARAM_WB_TYPE];
	dev_info->b_presrv_uspc_en =
		desc_buf[DEVICE_DESC_PARAM_WB_PRESRV_USRSPC_EN];
	if (dev_info->wb_buffer_type == WB_BUF_MODE_SHARED) {
		if (!get_unaligned_be32(
			    desc_buf +
			    DEVICE_DESC_PARAM_WB_SHARED_ALLOC_UNITS)) {
			err = -3;
			goto wb_check_fail;
		}
	} else {
		for (lun = 0; lun < UFS_UPIU_MAX_WB_LUN_ID; lun++) {
			d_lu_wb_buf_alloc = 0;
			ufshcd_read_unit_desc_param(
				hba, lun, UNIT_DESC_PARAM_WB_BUF_ALLOC_UNITS,
				(u8 *)&d_lu_wb_buf_alloc,
				sizeof(d_lu_wb_buf_alloc));
			if (d_lu_wb_buf_alloc) {
				dev_info->wb_dedicated_lu = lun;
				break;
			}
		}
		if (!d_lu_wb_buf_alloc) {
			err = -4;
			goto wb_check_fail;
		}
	}
wb_check_fail:
	return err;
}
int mi_check_wb(struct ufs_hba *hba)
{
	int err = 0;
	u8 *desc_buf;
	desc_buf = kzalloc(QUERY_DESC_MAX_SIZE, GFP_KERNEL);
	if (!desc_buf) {
		err = -ENOMEM;
		goto out;
	}
	err = ufshcd_read_desc_param(hba, QUERY_DESC_IDN_DEVICE, 0, 0,
					desc_buf, QUERY_DESC_MAX_SIZE);
	if (err) {
		dev_err(hba->dev, "%s: Failed reading Device Desc. err = %d\n",
			__func__, err);
		goto out;
	}
	err = check_device_wb_sup(hba, desc_buf);
	if (err) {
		dev_err(hba->dev,
			"%s: mi_ufs check_device_wb_sup fail. err = %d.\n",
			__func__, err);
#if IS_ENABLED(CONFIG_FACTORY_BUILD)
		dev_err(hba->dev,
			"%s: phone is not enable write booster feature, factory image can not turn on!\n",
			__func__);
		dev_err(hba->dev,
			"%s: If you have any question, please contact memory group!\n",
			__func__);
		//ufs_ffu_reboot_reason_reboot("bootloader");
                if (desc_buf) {
                  kfree(desc_buf);
                  desc_buf = NULL;
                }
		BUG_ON(1);
#endif
		goto out;
	}
out:
        if (desc_buf) {
          kfree(desc_buf);
          desc_buf = NULL;
        }
	return err;
}

