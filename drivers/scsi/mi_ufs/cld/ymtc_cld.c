/*
 * ymtc_cld.c
 *
 *  Created on: 2023-9-20
 *      Author: Ying
 */
#include <linux/string.h>
#include "ufscld.h"
#include "../mi_ufshcd.h"

#define QUERY_ATTR_IDN_YMTC_HID_WDSIZE			0x12
#define QUERY_FLAG_IDN_YMTC_UFS31_HID_ENABLE		0x13
#define QUERY_FLAG_IDN_YMTC_UFS22_HID_ENABLE		0x14
#define QUERY_ATTR_IDN_YMTC_HID_LEVEL            0x34
#define QUERY_ATTR_IDN_YMTC_HID_DEPROGES          0x35
#define QUERY_DESC_OFFSET_YMTC_HID_SUPPORT         0x4f
#define YMTC_DEFRAG_MENTION_STATUS_COUNT         4
#define INQUIRY_VENDOR_SIZE		8
#define INQUIRY_PRODUCT_SIZE		16
#define QUERY_FLAG_IDN_CLD_ENABLE 0x35
#define QUERY_ATTR_IDN_CLD_HIDSTATE 0x39
#define QUERY_ATTR_IDN_CLD_AVISIZE 0x36
#define QUERY_ATTR_IDN_CLD_HIDSIZE 0x37

enum YMTC_STATE {
	HID_IDEL = 0x0,
	HID_ANALYZING = 0x01,
	HID_DEFRAG_REQUIRED = 0x02,
	HID_DEFRAGMENTING = 0x03,
	HID_DAFRAG_COMPLETED = 0x04,
	HID_DEFRAG_UNREQUIRED = 0x05,
};

enum YMTC_DEFRAG_MENTION_STATUS {
	HID_MENTION_STATUS_GRAY = 0x0,
	HID_MENTION_STATUS_GREEN = 0x01,
	HID_MENTION_STATUS_YELLOW = 0x02,
	HID_MENTION_STATUS_RED = 0x03,
};

struct vendor_info_checklist {
	uint8_t vendor_id[INQUIRY_VENDOR_SIZE + 1];
	uint8_t product_id[INQUIRY_PRODUCT_SIZE+ 1];
	int fDefragEn;
};

static struct vendor_info_checklist vdr_update_checklist[] = {
	{"HG", "HBL19064G0CHBC", 	// 64GB ufs2.2
	QUERY_FLAG_IDN_YMTC_UFS22_HID_ENABLE},
	{"XBSTOR", "XBUSC1A17A8TG1", 	// 64GB ufs2.2
	QUERY_FLAG_IDN_YMTC_UFS22_HID_ENABLE},
	{"YMTC", "YMUS8B5TH1A1C1", 	// 128GB ufs2.2
	QUERY_FLAG_IDN_CLD_ENABLE},
};

int ymtc_cld_enable_flag(struct ufscld_dev *cld)
{
	int i, ret = 0;
	char *parse_inquiry = NULL;
	struct vendor_info_checklist  stdinq = {};
	struct ufs_hba *hba = cld->hba;
	if (cld->hba != NULL) {
		parse_inquiry = hba->sdev_ufs_device->inquiry + INQUIRY_VENDOR_SIZE;
		memcpy(stdinq.vendor_id, parse_inquiry, INQUIRY_VENDOR_SIZE);
		parse_inquiry += INQUIRY_VENDOR_SIZE;
		memcpy(stdinq.product_id, parse_inquiry, INQUIRY_PRODUCT_SIZE);
	} else {
		return ret;
	}
	for(i = 0; i < sizeof(vdr_update_checklist)/sizeof(vdr_update_checklist[0]); i++) {
		if (strncmp((char *)stdinq.vendor_id, (char *)vdr_update_checklist[i].vendor_id, strlen(vdr_update_checklist[i].vendor_id)) == 0
				&& strncmp((char *)stdinq.product_id, (char *)vdr_update_checklist[i].product_id, strlen(vdr_update_checklist[i].product_id)) == 0) {
				return vdr_update_checklist[i].fDefragEn;
		}
	}

	return ret;
}

static int ymtc_cld_get_avisize(struct ufscld_dev *cld, u32 *avisize)
{
	struct ufs_hba *hba = cld->hba;
	int ret = 0;
	*avisize = 0;
	ret = ufshcd_query_attr_retry(hba, UPIU_QUERY_OPCODE_READ_ATTR, (enum attr_idn)QUERY_ATTR_IDN_CLD_AVISIZE, 0, 0, avisize);
	return ret;
}

static int ymtc_cld_set_hidsize(struct ufscld_dev *cld, u32 hidsize)
{
	struct ufs_hba *hba = cld->hba;
	int ret = 0;
	ret = ufshcd_query_attr_retry(hba, UPIU_QUERY_OPCODE_WRITE_ATTR, (enum attr_idn)QUERY_ATTR_IDN_CLD_HIDSIZE, 0, 0, &hidsize);
	return ret;
}

static int ymtc_cld_get_hid_state(struct ufscld_dev *cld, int *hidstate)
{
	struct ufs_hba *hba = cld->hba;
	int ret = 0;
	*hidstate = 0;
	ret = ufshcd_query_attr_retry(hba, UPIU_QUERY_OPCODE_READ_ATTR, (enum attr_idn)QUERY_ATTR_IDN_CLD_HIDSTATE, 0, 0, hidstate);
	return ret;
}

int ymtc_get_frag_level(struct ufscld_dev *cld, int *frag_level)
{
	struct ufs_hba *hba = cld->hba;
	int ret = 0, attr = -1;
	int hidstate = 0;
	u32 aylenable = 1;
	int fIdn_enable = 0;
	fIdn_enable = ymtc_cld_enable_flag(cld);
	if(QUERY_FLAG_IDN_CLD_ENABLE != fIdn_enable){
		ret = ufshcd_query_attr_retry(hba, UPIU_QUERY_OPCODE_READ_ATTR,
				(enum attr_idn)QUERY_ATTR_IDN_YMTC_HID_LEVEL, 0, 0, &attr);
		if (ret) {
			ERR_MSG("%s:read cld level fail, ret = %d", __func__, ret);
			return ret;
		}

		if (attr < YMTC_DEFRAG_MENTION_STATUS_COUNT) {
			if (attr == HID_MENTION_STATUS_GRAY) {
				*frag_level = CLD_LEV_CLEAN;
			} else if (attr == HID_MENTION_STATUS_GREEN) {
				*frag_level = CLD_LEV_WARN;
			} else if (attr == HID_MENTION_STATUS_YELLOW ||
				attr == HID_MENTION_STATUS_RED) {
				*frag_level = CLD_LEV_CRITICAL;
			}
		} else {
			ERR_MSG("%s:unknown cld level, attr = %d", __func__, attr);
			return -1;
		}
	}else{
		ret = ymtc_cld_get_hid_state(cld, &hidstate);
		if (ret) {
			ERR_MSG("get hidstate failed ret=%d\n", ret);
			return ret;
		}

		if(hidstate == HID_IDEL || hidstate == HID_DAFRAG_COMPLETED || hidstate == HID_DEFRAG_UNREQUIRED) {
			ufshcd_query_attr_retry(hba, UPIU_QUERY_OPCODE_WRITE_ATTR, (enum attr_idn)QUERY_FLAG_IDN_CLD_ENABLE, 0, 0, &aylenable);
			*frag_level = CLD_LEV_CLEAN;
		} else if(hidstate == HID_DEFRAG_REQUIRED) {
			*frag_level = CLD_LEV_CRITICAL;
		} else if(hidstate == HID_ANALYZING || hidstate == HID_DEFRAGMENTING) {
			*frag_level = CLD_LEV_WARN;
		} else {
			pr_info("ymtc cld unknown level %d\n", hidstate);
			ret = -1;
			return ret;
		}
		
	}

	return 0;
}

int ymtc_cld_set_trigger(struct ufscld_dev *cld, u32 trigger)
{
	struct ufs_hba *hba = cld->hba;
	int ret = -1;
	int fIdn_enable = 0;
  	u32 avisize = 0;
	u32 dfdisable = 0, dfenable = 2;
	fIdn_enable = ymtc_cld_enable_flag(cld);
	if (!fIdn_enable) {
		ERR_MSG("%s:read cld set fail, fIdn_enable = %d", __func__, fIdn_enable);
		return ret;
	}
	if(QUERY_FLAG_IDN_CLD_ENABLE != fIdn_enable){
		if (trigger) {
			ret = ufshcd_query_flag_retry(hba, UPIU_QUERY_OPCODE_SET_FLAG,
				(enum flag_idn)fIdn_enable, 0, NULL);
		} else {
			ret = ufshcd_query_flag_retry(hba, UPIU_QUERY_OPCODE_CLEAR_FLAG,
				(enum flag_idn)fIdn_enable, 0, NULL);
		}
	}else{
		if(trigger) {
		ret = ymtc_cld_get_avisize(cld, &avisize);
		if(!ret) {
			if(ymtc_cld_set_hidsize(cld, avisize)) {
				ERR_MSG("set hidsize failed\n");
				return -1;
			}
		}
			ret = ufshcd_query_attr_retry(hba, UPIU_QUERY_OPCODE_WRITE_ATTR, (enum attr_idn)QUERY_FLAG_IDN_CLD_ENABLE, 0, 0, &dfenable);
		} else {
			ret = ufshcd_query_attr_retry(hba, UPIU_QUERY_OPCODE_WRITE_ATTR, (enum 		attr_idn)QUERY_FLAG_IDN_CLD_ENABLE, 0, 0, &dfdisable);
		}
	}
	return ret;
}

int ymtc_cld_get_trigger(struct ufscld_dev *cld, u32 *trigger)
{
	struct ufs_hba *hba = cld->hba;
	int ret = -1;
	int fIdn_enable = 0;
	fIdn_enable = ymtc_cld_enable_flag(cld);
		if (!fIdn_enable) {
			ERR_MSG("%s:read cld get fail, fIdn_enable = %d", __func__, fIdn_enable);
			return ret;
		}
	if(QUERY_FLAG_IDN_CLD_ENABLE != fIdn_enable){
		ret = ufshcd_query_flag_retry(hba, UPIU_QUERY_OPCODE_READ_FLAG,
				(enum flag_idn)fIdn_enable, 0, (bool *)trigger);
	}else{
		ret = ufshcd_query_attr_retry(hba, UPIU_QUERY_OPCODE_READ_ATTR, (enum attr_idn)QUERY_FLAG_IDN_CLD_ENABLE, 0, 0, trigger);
	}

	return ret;
}

int ymtc_cld_operation_status(struct ufscld_dev *cld, int *op_status)
{
	enum CLD_LEVEL frag_level;
	int ret;
	ret = ymtc_get_frag_level(cld, (int *)&frag_level);
	if (ret)
		ERR_MSG("%s:get cld frag level failed, ret=%d\n", __func__, ret);
	if (frag_level == CLD_LEV_CLEAN) {
		*op_status = CLD_STATUS_IDLE;
	} else if (frag_level == CLD_LEV_WARN ||
			frag_level == CLD_LEV_CRITICAL) {
		*op_status = CLD_STATUS_PROGRESSING;
	} else {
		*op_status = CLD_STATUS_NA;
	}

	return 0;
}

struct ufscld_ops ymtc_cld_ops = {
	.cld_get_frag_level = ymtc_get_frag_level,
	.cld_set_trigger = ymtc_cld_set_trigger,
	.cld_get_trigger = ymtc_cld_get_trigger,
	.cld_operation_status = ymtc_cld_operation_status
};