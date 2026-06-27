/*
 * hg_cld.c
 *
 *  Created on: 2023-9-20
 *      Author: Ying
 */
#include "ufscld.h"
#include "../mi_ufshcd.h"

#define QUERY_FLAG_IDN_HG_HID_ENABLE           0x13
#define QUERY_ATTR_IDN_HG_HID_LEVEL            0x34
#define HG_DEFRAG_MENTION_STATUS_COUNT         4

enum HG_DEFRAG_MENTION_STATUS {
	HID_MENTION_STATUS_GRAY = 0x0,
	HID_MENTION_STATUS_GREEN = 0x01,
	HID_MENTION_STATUS_YELLOW = 0x02,
	HID_MENTION_STATUS_RED = 0x03,
};

int hg_get_frag_level(struct ufscld_dev *cld, int *frag_level)
{
	struct ufs_hba *hba = cld->hba;
	int ret = 0, attr = -1;
	ret = ufshcd_query_attr_retry(hba, UPIU_QUERY_OPCODE_READ_ATTR,
			(enum attr_idn)QUERY_ATTR_IDN_HG_HID_LEVEL, 0, 0, &attr);
	if (ret) {
		ERR_MSG("read hg cld level fail,ret = %d", ret);
		return ret;
	}

	if (attr < HG_DEFRAG_MENTION_STATUS_COUNT) {
		if (attr == HID_MENTION_STATUS_GRAY) {
			*frag_level = CLD_LEV_CLEAN;
		} else if (attr == HID_MENTION_STATUS_GREEN) {
			*frag_level = CLD_LEV_WARN;
		} else if (attr == HID_MENTION_STATUS_YELLOW ||
			attr == HID_MENTION_STATUS_RED) {
			*frag_level = CLD_LEV_CRITICAL;
		}
	} else {
		ERR_MSG("hg cld unknown level, attr = %d", attr);
		return -1;
	}

	return 0;
}

int hg_cld_set_trigger(struct ufscld_dev *cld, u32 trigger)
{
	struct ufs_hba *hba = cld->hba;
	int ret = -1;

	if (trigger) {
		ret = ufshcd_query_flag_retry(hba, UPIU_QUERY_OPCODE_SET_FLAG,
			(enum flag_idn)QUERY_FLAG_IDN_HG_HID_ENABLE, 0, NULL);
	} else {
		ret = ufshcd_query_flag_retry(hba, UPIU_QUERY_OPCODE_CLEAR_FLAG,
			(enum flag_idn)QUERY_FLAG_IDN_HG_HID_ENABLE, 0, NULL);
	}

	return ret;
}

int hg_cld_get_trigger(struct ufscld_dev *cld, u32 *trigger)
{
	struct ufs_hba *hba = cld->hba;
	int ret = -1;
	ret = ufshcd_query_flag_retry(hba, UPIU_QUERY_OPCODE_READ_FLAG,
			(enum flag_idn)QUERY_FLAG_IDN_HG_HID_ENABLE, 0, (bool *)trigger);

	return ret;
}

int hg_cld_operation_status(struct ufscld_dev *cld, int *op_status)
{
	enum CLD_LEVEL frag_level;
	int ret;

	ret = hg_get_frag_level(cld, (int *)&frag_level);
	if (ret)
		ERR_MSG("hg cld get frag level failed, ret=%d\n", ret);

	if (frag_level == CLD_LEV_CLEAN) { // if cld was done
		*op_status = CLD_STATUS_IDLE;
	} else if (frag_level == CLD_LEV_WARN ||
			frag_level == CLD_LEV_CRITICAL) {
		*op_status = CLD_STATUS_PROGRESSING;
	} else {
		*op_status = CLD_STATUS_NA;
	}

	return 0;
}

struct ufscld_ops hg_cld_ops = {
	.cld_get_frag_level = hg_get_frag_level,
	.cld_set_trigger = hg_cld_set_trigger,
	.cld_get_trigger = hg_cld_get_trigger,
	.cld_operation_status = hg_cld_operation_status
};