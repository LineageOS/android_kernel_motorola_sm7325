/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2020 Texas Instruments Incorporated - http://www.sc.com/ */

#ifndef SC8541_IIO_H
#define SC8541_IIO_H


#include <linux/iio/iio.h>
#include <dt-bindings/iio/qti_power_supply_iio.h>


struct sc8541_iio_channels {
	const char *datasheet_name;
	int channel_num;
	enum iio_chan_type type;
	long info_mask;
};

enum {
	MMI_DISABLE_ADC = 0,
	MMI_ENABLE_ADC,
};

#define SC8541_IIO_CHAN(_name, _num, _type, _mask)		\
	{						\
		.datasheet_name = _name,		\
		.channel_num = _num,			\
		.type = _type,				\
		.info_mask = _mask,			\
	},

#define SC8541_CHAN_INDEX(_name, _num)			\
	SC8541_IIO_CHAN(_name, _num, IIO_INDEX,		\
		BIT(IIO_CHAN_INFO_PROCESSED))

#define SC8541_CHAN_CUR(_name, _num)			\
	SC8541_IIO_CHAN(_name, _num, IIO_CURRENT,		\
		BIT(IIO_CHAN_INFO_PROCESSED))

#define SC8541_CHAN_VOLT(_name, _num)			\
	SC8541_IIO_CHAN(_name, _num, IIO_VOLTAGE,		\
		BIT(IIO_CHAN_INFO_PROCESSED))

static const struct sc8541_iio_channels sc8541_iio_psy_channels[] = {
	SC8541_CHAN_INDEX("sc8541_cp_enabled", PSY_IIO_CP_ENABLE)
	SC8541_CHAN_INDEX("sc8541_online", PSY_IIO_ONLINE)
	SC8541_CHAN_CUR("sc8541_current_now", PSY_IIO_CURRENT_NOW)
	SC8541_CHAN_VOLT("sc8541_voltage_now", PSY_IIO_VOLTAGE_NOW)
	SC8541_CHAN_INDEX("sc8541_input_current_now", PSY_IIO_MMI_CP_INPUT_CURRENT_NOW)
	SC8541_CHAN_INDEX("sc8541_input_voltage_settled", PSY_IIO_MMI_CP_INPUT_VOLTAGE_NOW)
	SC8541_CHAN_INDEX("sc8541_cp_status1", PSY_IIO_CP_STATUS1)
	SC8541_CHAN_INDEX("sc8541_cp_clear_error", PSY_IIO_CP_CLEAR_ERROR)
};

#endif /* SC8541_IIO_H */
