

#ifndef __SM5602_IIO_H
#define __SM5602_IIO_H

#include <linux/iio/iio.h>
#include <dt-bindings/iio/qti_power_supply_iio.h>
#include <linux/qti_power_supply.h>

struct sm5602_iio_channels {
	const char *datasheet_name;
	int channel_num;
	enum iio_chan_type type;
	long info_mask;
};

#define SM5602_IIO_CHAN(_name, _num, _type, _mask)		\
	{						\
		.datasheet_name = _name,		\
		.channel_num = _num,			\
		.type = _type,				\
		.info_mask = _mask,			\
	},

#define SM5602_CHAN_CURRENT(_name, _num)			\
	SM5602_IIO_CHAN(_name, _num, IIO_CURRENT,		\
		BIT(IIO_CHAN_INFO_PROCESSED))

static const struct sm5602_iio_channels sm5602_iio_psy_channels[] = {
	SM5602_CHAN_CURRENT("resistance_id", PSY_IIO_RESISTANCE_ID)
	SM5602_CHAN_CURRENT("fg_batt_id", PSY_IIO_BATT_ID)
};

#endif

