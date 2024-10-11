// SPDX-License-Identifier: GPL-2.0
// BQ25980 Battery Charger Driver
// Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio/consumer.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/iio/consumer.h>

#include "sc8541_reg.h"
#include "sc8541_mmi_iio.h"
#include "nu2115_reg.h"

//undef because of pinctrl
#define CONFIG_INTERRUPT_AS_GPIO

union sc8541_alarm_status {
	struct {
	unsigned char bat_ovp_alarm:1;
	unsigned char bat_ocp_alarm:1;
	unsigned char bus_ovp_alarm:1;
	unsigned char bus_ocp_alarm:1;
	unsigned char bat_therm_alarm:1;
	unsigned char bus_therm_alarm:1;
	unsigned char die_therm_alarm:1;
	unsigned char bat_ucp_alarm:1;
	}bits;
	unsigned char status;
};

union sc8541_fault_status {
	struct {
	unsigned char bat_ovp_fault:1;
	unsigned char bat_ocp_fault:1;
	unsigned char bus_ovp_fault:1;
	unsigned char bus_ocp_fault:1;
	unsigned char bat_therm_fault:1;
	unsigned char bus_therm_fault:1;
	unsigned char die_therm_fault:1;
	unsigned char :1;
	}bits;
	unsigned char status;
};

struct sc8541_state {
	bool dischg;
	bool ovp;
	bool ocp;
	bool wdt;
	bool tflt;
	bool online;
	bool ce;
	bool hiz;
	bool bypass;
	bool cp_switch;//add cp_switch state

	u32 vbat_adc;
	u32 vsys_adc;
	u32 ibat_adc;
	u32 fault_status;
};

enum sc_work_mode {
	SC_STANDALONE,
	SC_SLAVE,
	SC_MASTER,
};

#define SC_MODE_COUNT 3
#define SC8541_PART_NO 0x41
#define NU2115_PART_NO 0x90

enum sc_device_id {
	SC8541 = 8,
};

enum sc_compatible_id {
	SC8541_STANDALONE,
	SC8541_SLAVE,
	SC8541_MASTER,
};

struct sc8541_chip_info {

	int model_id;

	const struct regmap_config *regmap_config;

	const struct reg_default *reg_init_values;

	int busocp_sc_def;
	int busocp_byp_def;
	int busocp_sc_max;
	int busocp_byp_max;
	int busocp_sc_min;
	int busocp_byp_min;
	int busocp_step;
	int busocp_offset;

	int busovp_sc_def;
	int busovp_byp_def;
	int busovp_sc_step;

	int busovp_sc_offset;
	int busovp_byp_step;
	int busovp_byp_offset;
	int busovp_sc_min;
	int busovp_sc_max;
	int busovp_byp_min;
	int busovp_byp_max;

	int batovp_def;
	int batovp_max;
	int batovp_min;
	int batovp_step;
	int batovp_offset;

	int batocp_def;
	int batocp_max;

	int vac_sc_ovp;
	int vac_byp_ovp;

	int adc_curr_step;
	int adc_vbat_volt_step;
	int adc_vbus_volt_step;
	int adc_vbus_volt_offset;
	int adc_vout_volt_step;
	int adc_vout_volt_offset;

};

struct sc8541_init_data {
	u32 ichg;
	u32 bypass_ilim;
	u32 sc_ilim;
	u32 vreg;
	u32 iterm;
	u32 iprechg;
	u32 bypass_vlim;
	u32 sc_vlim;
	u32 ichg_max;
	u32 vreg_max;
};

struct sc8541_device {
	struct i2c_client *client;
	struct device *dev;
	struct power_supply *charger;
	struct power_supply *battery;
	struct mutex lock;
	struct regmap *regmap;

	char model_name[I2C_NAME_SIZE];

	struct sc8541_init_data init_data;
	const struct sc8541_chip_info *chip_info;
	struct sc8541_state state;
	int watchdog_timer;
	int mode;
	int device_id;
	struct power_supply_desc psy_desc;
	struct pinctrl *irq_pinctrl;
	bool usb_present;
	int irq_counts;
	bool irq_waiting;
	bool irq_disabled;
	bool resume_completed;
	struct mutex irq_complete;

	union sc8541_alarm_status alarm_status;
	union sc8541_fault_status fault_status;
	struct iio_dev *indio_dev;
	struct iio_chan_spec *iio_chan;
	struct iio_channel *int_iio_chans;
	struct iio_channel **ext_iio_chans;

	int reg_addr;
	int reg_data;
	int part_no;
	int nu2115_addr;
};

static struct reg_default sc8541_reg_init_val[] = {
	{SC8541_BATOVP,	        0x4a},//0x47:4550mV 0x4a:4580mv
	{SC8541_BATOVP_ALM,	0x42},//0x3f:4470mV 0x42:4500mv
	{SC8541_BATOCP,	        0xEE},//0xEE:disable for dual //0x46:7000mA for standalone
	{SC8541_BATOCP_ALM,	0x7F},//0x7F:12700mA
	{SC8541_CHRGR_CFG_1,	0x00},
	{SC8541_CHRGR_CTRL_1,	0x49},
	{SC8541_BUSOVP,	        0x64},//0X50:11000mV 0x64:12000mv
	{SC8541_BUSOVP_ALM,	0x50},//0X46:10500mV 0X50:11000mV
	{SC8541_BUSOCP,	        0x0C},//0X0c:4000mA
	{SC8541_REG_09,	        0x00},
	{SC8541_TEMP_CONTROL,	0x2C},
	{SC8541_TDIE_ALM,	0x78},//0x78:85C
	{SC8541_TSBUS_FLT,	0x15},
	{SC8541_TSBAT_FLG,	0x15},
	{SC8541_VAC_CONTROL,	0x6c},//0xb4:14v*2 for vacovp
	{SC8541_CHRGR_CTRL_2,	0x00},
	{SC8541_CHRGR_CTRL_3,	0x94},//0x94:watchdog disable 5s,500kHz
	{SC8541_CHRGR_CTRL_4,	0xf1},//5m oum battery sense resister & ss_timeout is 10s
	{SC8541_CHRGR_CTRL_5,	0x60},

	{SC8541_MASK1,		0x00},
	{SC8541_MASK2,		0x00},
	{SC8541_MASK3,		0x00},
	{SC8541_MASK4,		0x00},
	{SC8541_MASK5,		0x00},

	{SC8541_ADC_CONTROL1,	0x00},//disable ADC_EN as default
	{SC8541_ADC_CONTROL2,	0x06}, //0x26: enable vac1 vac2 adc and vout adc

};

static struct reg_default nu2115_reg_init_val[] = {
	{NU2115_BATOVP,	    0x45},//0x47:4550mV 0x45:4580mv
	{NU2115_BATOVP_ALM,	0x80},//0x3f:4470mV 0x42:4500mv
	{NU2115_BATOCP,	    0xDA},//0xDA:disable for dual  11A//0x46:7000mA for standalone
	{NU2115_BATOCP_ALM,	0x80},//0x6B:12700mA
	{NU2115_BATUCP_ALM,	0x80},//0x28:default 2a
	{NU2115_AC1PROT,	0x06},//0x06:13000mv
	{NU2115_AC2PROT,	0x06},//0x06:13000mv
	{NU2115_BUSOVP,	    0x2D},//0x2D:10500mv
	{NU2115_BUSOVP_ALM,	0x80},//0X32:11000mV
	{NU2115_BUSOCP,	    0x06},//0X06:4000mA
	{NU2115_BUSOCP_ALM,	0x8C},//0X8C:4000mA disable
	{NU2115_CON_STAT,	0x00},
	{NU2115_CTRL_REG,	0x36},//0x36:watchdog disable 5s,600kHz
	{NU2115_CHGCTRL,	0x07},//default
	{NU2115_INT_STAT,	0x00},//default mean {NU2115_STAT1, 0x0}
	{NU2115_INT_FLAG,	0x00},//default
	{NU2115_INT_MASK,	0x00},//default
	{NU2115_FLT_MASK,	0x00},//default
	{NU2115_ADC_CTRL,	0x00},//default mean {NU2115_ADC_CONTROL1,	0x00}
	{NU2115_ADC_FN_DIS,	0x8F},//0x06:TSBUS TSBAT mean {NU2115_ADC_CONTROL2,	0x06}
	{NU2115_TSBUS_FLT,	0x15},
	{NU2115_TSBAT_FLG,	0x15},
	{NU2115_TDIE_ALM,	0x48},//0x48:60C
	{NU2115_IBUS_UCP,	0xE2},
	{NU2115_VAC12PRET,	0x90},
	{NU2115_ACDRV12_CTRL,   0x40},
	{NU2115_P2VOUT_UOVP,    0x70},
	{NU2115_DEGLITC_REG,    0x0D},
	{NU2115_CP_OPTION,      0x00},
	{NU2115_CP_OPTION1,     0xC0},
	{NU2115_CP_OPTION2,     0x27},
};

static struct reg_default sc8541_reg_defs[] = {
	{SC8541_BATOVP,       0x4a},
	{SC8541_BATOVP_ALM,   0x42},
	{SC8541_BATOCP,       0x51},
	{SC8541_BATOCP_ALM,   0x50},
	{SC8541_CHRGR_CFG_1,  0x0},
	{SC8541_CHRGR_CTRL_1, 0x0},
	{SC8541_BUSOVP,       0x26},
	{SC8541_BUSOVP_ALM,   0x22},
	{SC8541_BUSOCP,       0xD},
	{SC8541_REG_09,       0x0},
	{SC8541_TEMP_CONTROL, 0x00},
	{SC8541_TDIE_ALM,     0xC8},
	{SC8541_TSBUS_FLT,    0x15},
	{SC8541_TSBAT_FLG,    0x15},
	{SC8541_VAC_CONTROL,  0x0},
	{SC8541_CHRGR_CTRL_2, 0x0},
	{SC8541_CHRGR_CTRL_3, 0x20},
	{SC8541_CHRGR_CTRL_4, 0x1D},
	{SC8541_CHRGR_CTRL_5, 0x18},
	{SC8541_STAT1,        0x0},
	{SC8541_STAT2,        0x0},
	{SC8541_STAT3,        0x0},
	{SC8541_STAT4,        0x0},
	{SC8541_STAT5,        0x0},
	{SC8541_FLAG1,        0x0},
	{SC8541_FLAG2,        0x0},
	{SC8541_FLAG3,        0x0},
	{SC8541_FLAG4,        0x0},
	{SC8541_FLAG5,        0x0},
	{SC8541_MASK1,        0x0},
	{SC8541_MASK2,        0x0},
	{SC8541_MASK3,        0x0},
	{SC8541_MASK4,        0x0},
	{SC8541_MASK5,        0x0},
	{SC8541_DEVICE_INFO,  0x41},
	{SC8541_ADC_CONTROL1, 0x0},
	{SC8541_ADC_CONTROL2, 0x0},
	{SC8541_IBUS_ADC_LSB, 0x0},
	{SC8541_IBUS_ADC_MSB, 0x0},
	{SC8541_VBUS_ADC_LSB, 0x0},
	{SC8541_VBUS_ADC_MSB, 0x0},
	{SC8541_VAC1_ADC_LSB, 0x0},
	{SC8541_VAC2_ADC_LSB, 0x0},
	{SC8541_VOUT_ADC_LSB, 0x0},
	{SC8541_VBAT_ADC_LSB, 0x0},
	{SC8541_IBAT_ADC_MSB, 0x0},
	{SC8541_IBAT_ADC_LSB, 0x0},
	{SC8541_TSBUS_ADC_LSB,0x0},
	{SC8541_TSBAT_ADC_LSB,0x0},
	{SC8541_TDIE_ADC_LSB, 0x0},
	{SC8541_DEGLITCH_TIME,0x0},
	{SC8541_CHRGR_CTRL_6, 0x0},
};

static struct reg_default nu2115_reg_defs[] = {
	{NU2115_BATOVP,        0x37},
	{NU2115_BATOVP_ALM,    0x80},
	{NU2115_BATOCP,        0xDA},
	{NU2115_BATOCP_ALM,    0x80},
	{NU2115_BATUCP_ALM,    0x80},
	{NU2115_AC1PROT,       0x06},
	{NU2115_AC2PROT,       0x06},
	{NU2115_BUSOVP,        0x2D},
	{NU2115_BUSOVP_ALM,    0x80},
	{NU2115_BUSOCP,        0x01},
	{NU2115_BUSOCP_ALM,    0x8C},
	{NU2115_VOUTOVP,       0x00},
	{NU2115_CON_STAT,      0x00},
	{NU2115_CTRL_REG,      0x20},
	{NU2115_CHGCTRL,       0x07},
	{NU2115_INT_STAT,      0x00},
	{NU2115_INT_FLAG,      0x00},
	{NU2115_INT_MASK,      0x00},
	{NU2115_FLT_STAT,      0x00},
	{NU2115_FLT_FLAG,      0x00},
	{NU2115_FLT_MASK,      0x00},
	{NU2115_ADC_CTRL,      0x00},
	{NU2115_ADC_FN_DIS,    0x8F},
	{NU2115_IBUS_ADC_MSB,  0x00},
	{NU2115_IBUS_ADC_LSB,  0x00},
	{NU2115_VBUS_ADC_MSB,  0x00},
	{NU2115_VBUS_ADC_LSB,  0x00},
	{NU2115_VAC1_ADC_MSB,  0x00},
	{NU2115_VAC1_ADC_LSB,  0x00},
	{NU2115_VAC2_ADC_MSB,  0x00},
	{NU2115_VAC2_ADC_LSB,  0x00},
	{NU2115_VOUT_ADC_MSB,  0x00},
	{NU2115_VOUT_ADC_LSB,  0x00},
	{NU2115_VBAT_ADC_MSB,  0x00},
	{NU2115_VBAT_ADC_LSB,  0x00},
	{NU2115_IBAT_ADC_MSB,  0x00},
	{NU2115_IBAT_ADC_LSB,  0x00},
	{NU2115_TSBUS_ADC_MSB, 0x00},
	{NU2115_TSBUS_ADC_LSB, 0x00},
	{NU2115_TSBAT_ADC_MSB, 0x00},
	{NU2115_TSBAT_ADC_LSB, 0x00},
	{NU2115_TDIE_ADC_MSB,  0x00},
	{NU2115_TDIE_ADC_LSB,  0x00},
	{NU2115_TSBUS_FLT,     0x15},
	{NU2115_TSBAT_FLG,     0x15},
	{NU2115_TDIE_ALM,      0xC3},
	{NU2115_IBUS_UCP,      0xE2},
	{NU2115_VAC12PRET,     0x01},
	{NU2115_ACDRV12_CTRL,  0x80},
	{NU2115_DEV_INFO,      0x90},
	{NU2115_P2VOUT_UOVP,   0x70},
	{NU2115_DEGLITC_REG,   0x0D},
	{NU2115_CP_OPTION,     0x00},
	{NU2115_CP_OPTION1,    0xC0},
	{NU2115_CP_OPTION2,    0x27},
};

static int sc8541_reg_init(struct sc8541_device *bq);

static void dump_all_reg(struct sc8541_device *bq)
{
	int ret;
	unsigned int val;
	int addr;

	for (addr = 0x00; addr <= 0x37; addr++) {
		ret = regmap_read(bq->regmap, addr, &val);
		if (!ret)
			dev_err(bq->dev, "[%s] Reg[%02X] = 0x%02X\n", bq->model_name, addr, val);
	}
}

static int sc8541_set_adc_enable(struct sc8541_device *bq, bool enable)
{
	int ret;

	dev_notice(bq->dev, "%s %d", __FUNCTION__, enable);

	if (bq->part_no == NU2115_PART_NO) {
		if (enable) {
			ret = regmap_update_bits(bq->regmap, NU2115_ADC_CTRL,
					NU2115_ADC_EN, NU2115_ADC_EN);
			/* when adc coolect after 20ms */
			msleep(20);
		} else
			ret = regmap_update_bits(bq->regmap, NU2115_ADC_CTRL,
					NU2115_ADC_EN, 0);
	} else {
		if (enable)
			ret = regmap_update_bits(bq->regmap, SC8541_ADC_CONTROL1,
					SC8541_ADC_EN, SC8541_ADC_EN);
		else
			ret = regmap_update_bits(bq->regmap, SC8541_ADC_CONTROL1,
					SC8541_ADC_EN, 0);
	}
	return ret;
}

static int sc8541_get_const_charge_curr(struct sc8541_device *bq)
{
	unsigned int batocp_reg_code;
	unsigned int curr_value;
	int ret;

	if (bq->part_no == NU2115_PART_NO) {
		ret = regmap_read(bq->regmap, NU2115_BATOCP, &batocp_reg_code);
		if (ret)
			return ret;

		curr_value = (batocp_reg_code & NU2115_BATOCP_MASK) *
								NU2115_BATOCP_STEP_uA;

	} else {
		ret = regmap_read(bq->regmap, SC8541_BATOCP, &batocp_reg_code);
		if (ret)
			return ret;

		curr_value = (batocp_reg_code & SC8541_BATOCP_MASK) *
							SC8541_BATOCP_STEP_uA;
	}
	return curr_value;
}

static int sc8541_get_const_charge_volt(struct sc8541_device *bq)
{
	unsigned int batovp_reg_code;
	unsigned int volt_value;
	int ret;
	if (bq->part_no == NU2115_PART_NO) {
		ret = regmap_read(bq->regmap, NU2115_BATOVP, &batovp_reg_code);
		if (ret)
			return ret;

		volt_value = ((batovp_reg_code * bq->chip_info->batovp_step) +
				bq->chip_info->batovp_offset);
	} else {
		ret = regmap_read(bq->regmap, SC8541_BATOVP, &batovp_reg_code);
		if (ret)
			return ret;

		volt_value = ((batovp_reg_code * bq->chip_info->batovp_step) +
				bq->chip_info->batovp_offset);
	}
	return volt_value;
}

static int sc8541_set_chg_en(struct sc8541_device *bq, bool en_chg)
{
	int ret;

	if (bq->part_no == NU2115_PART_NO) {
		if (en_chg) {
			ret = regmap_update_bits(bq->regmap, NU2115_P2VOUT_UOVP,
						NU2115_PMID2VOUT_OVP, NU2115_PMID2VOUT_OVP12P5);

			ret = regmap_update_bits(bq->regmap, NU2115_CHGCTRL,
						NU2115_CHG_EN, NU2115_CHG_EN);
			/* Set PMID2OVP 12.5% to 7.5% after 30ms */
			msleep(30);
			ret = regmap_update_bits(bq->regmap, NU2115_P2VOUT_UOVP,
						NU2115_PMID2VOUT_OVP, NU2115_PMID2VOUT_OVP7P5);

		} else {
			ret = regmap_update_bits(bq->regmap, NU2115_CHGCTRL,
					NU2115_CHG_EN, en_chg);
		}

		if (ret)
			return ret;
	} else {
		if (en_chg)
			ret = regmap_update_bits(bq->regmap, SC8541_CHRGR_CTRL_2,
						SC8541_CHG_EN, SC8541_CHG_EN);
		else
			ret = regmap_update_bits(bq->regmap, SC8541_CHRGR_CTRL_2,
						SC8541_CHG_EN, en_chg);
		if (ret)
			return ret;
	}

	bq->state.ce = en_chg;
	return 0;
}

static int sc8541_is_chg_en(struct sc8541_device *bq, bool *en_chg)
{
	int ret;
	unsigned int chg_ctrl_2;
	unsigned int stat5;

	if (bq->part_no == NU2115_PART_NO) {	
		ret = regmap_read(bq->regmap, NU2115_CHGCTRL, &chg_ctrl_2);
		if (ret)
			return ret;

		ret = regmap_read(bq->regmap, NU2115_CON_STAT, &stat5);
		if (ret)
			return ret;

		*en_chg = (!!(chg_ctrl_2 & NU2115_CHG_EN) &
			 !!(stat5 & NU2115_SWITCHING_STAT));
	} else {
		ret = regmap_read(bq->regmap, SC8541_CHRGR_CTRL_2, &chg_ctrl_2);
		if (ret)
			return ret;

		ret = regmap_read(bq->regmap, SC8541_STAT5, &stat5);
		if (ret)
			return ret;

		*en_chg = (!!(chg_ctrl_2 & SC8541_CHG_EN) &
			 !!(stat5 & SC8541_SWITCHING_STAT));
	}

	return 0;
}

static int sc8541_set_otg_en(struct sc8541_device *bq, bool en_otg)
{
	int ret = 0;

	if (bq->part_no != NU2115_PART_NO) {
		dev_err(bq->dev," ic is not NU2115 charger!");
		return ret;
	}

	if (en_otg) {
		ret = regmap_update_bits(bq->regmap, NU2115_VAC12PRET,
						NU2115_EN_OTG, NU2115_EN_OTG);
		ret += regmap_update_bits(bq->regmap, NU2115_ACDRV12_CTRL,
						NU2115_EN_ACRDV2, NU2115_EN_ACRDV2);
	} else {
		ret = regmap_update_bits(bq->regmap, NU2115_VAC12PRET,
						NU2115_EN_OTG, en_otg);
		ret += regmap_update_bits(bq->regmap, NU2115_ACDRV12_CTRL,
						NU2115_EN_ACRDV2, NU2115_EN_ACRDV2);
	}

	if (ret)
		return ret;

	return 0;
}

static int sc8541_get_adc_ibus(struct sc8541_device *bq)
{
	int ibus_adc_lsb, ibus_adc_msb;
	u16 ibus_adc;
	int ret;

	if (bq->part_no == NU2115_PART_NO) {
		ret = regmap_read(bq->regmap, NU2115_IBUS_ADC_MSB, &ibus_adc_msb);
		if (ret)
			return ret;

		ret = regmap_read(bq->regmap, NU2115_IBUS_ADC_LSB, &ibus_adc_lsb);
		if (ret)
			return ret;

		ibus_adc = (ibus_adc_msb << 8) | ibus_adc_lsb;

		if (ibus_adc_msb & NU2115_ADC_POLARITY_BIT)
			ibus_adc = ((ibus_adc ^ 0xffff) + 1);//mA
	} else {
		ret = regmap_read(bq->regmap, SC8541_IBUS_ADC_MSB, &ibus_adc_msb);
		if (ret)
			return ret;

		ret = regmap_read(bq->regmap, SC8541_IBUS_ADC_LSB, &ibus_adc_lsb);
		if (ret)
			return ret;

		ibus_adc = (ibus_adc_msb << 8) | ibus_adc_lsb;

		if (ibus_adc_msb & SC8541_ADC_POLARITY_BIT)
			return (((ibus_adc ^ 0xffff) + 1) * bq->chip_info->adc_curr_step) /1000;//mA

		ibus_adc = (ibus_adc * bq->chip_info->adc_curr_step) /1000; //mA
	}
	return ibus_adc;
}

static int sc8541_get_adc_vbus(struct sc8541_device *bq)
{
	int vbus_adc_lsb, vbus_adc_msb;
	u16 vbus_adc;
	int ret;

	if (bq->part_no == NU2115_PART_NO) {
		ret = regmap_read(bq->regmap, NU2115_VBUS_ADC_MSB, &vbus_adc_msb);
		if (ret)
			return ret;

		ret = regmap_read(bq->regmap, NU2115_VBUS_ADC_LSB, &vbus_adc_lsb);
		if (ret)
			return ret;

		vbus_adc = (vbus_adc_msb << 8) | vbus_adc_lsb;

		if (vbus_adc_msb & NU2115_ADC_POLARITY_BIT)
			vbus_adc = ((vbus_adc ^ 0xffff) + 1);//mA
	} else {
		ret = regmap_read(bq->regmap, SC8541_VBUS_ADC_MSB, &vbus_adc_msb);
		if (ret)
			return ret;

		ret = regmap_read(bq->regmap, SC8541_VBUS_ADC_LSB, &vbus_adc_lsb);
		if (ret)
			return ret;

		vbus_adc = (vbus_adc_msb << 8) | vbus_adc_lsb;

		vbus_adc = (bq->chip_info->adc_vbus_volt_offset + vbus_adc * bq->chip_info->adc_vbus_volt_step /10) /1000;//mV
	}

	return vbus_adc;
}

static int sc8541_get_ibat_adc(struct sc8541_device *bq)
{
	int ret;
	int ibat_adc_lsb, ibat_adc_msb;
	int ibat_adc;

	if (bq->part_no == NU2115_PART_NO) {
		ret = regmap_read(bq->regmap, NU2115_IBAT_ADC_MSB, &ibat_adc_msb);
		if (ret)
			return ret;

		ret = regmap_read(bq->regmap, NU2115_IBAT_ADC_LSB, &ibat_adc_lsb);
		if (ret)
			return ret;

		ibat_adc = (ibat_adc_msb << 8) | ibat_adc_lsb;

		if (ibat_adc_msb & NU2115_ADC_POLARITY_BIT)
			ibat_adc = ((ibat_adc ^ 0xffff) + 1);//mA
	} else {
		ret = regmap_read(bq->regmap, SC8541_IBAT_ADC_MSB, &ibat_adc_msb);
		if (ret)
			return ret;

		ret = regmap_read(bq->regmap, SC8541_IBAT_ADC_LSB, &ibat_adc_lsb);
		if (ret)
			return ret;

		ibat_adc = (ibat_adc_msb << 8) | ibat_adc_lsb;

		if (ibat_adc_msb & SC8541_ADC_POLARITY_BIT)
			return (((ibat_adc ^ 0xffff) + 1) * SC8541_ADC_CURR_STEP_uA) /1000;//mA
		ibat_adc = (ibat_adc * SC8541_ADC_CURR_STEP_uA) /1000; //mA
	}
	return ibat_adc;
}

static int sc8541_get_adc_vbat(struct sc8541_device *bq)
{
	int vsys_adc_lsb, vsys_adc_msb;
	u16 vsys_adc;
	int ret;

	if (bq->part_no == NU2115_PART_NO) {
		ret = regmap_read(bq->regmap, NU2115_VBAT_ADC_MSB, &vsys_adc_msb);
		if (ret)
			return ret;

		ret = regmap_read(bq->regmap, NU2115_VBAT_ADC_LSB, &vsys_adc_lsb);
		if (ret)
			return ret;

		vsys_adc = (vsys_adc_msb << 8) | vsys_adc_lsb;

		if (vsys_adc_msb & NU2115_ADC_POLARITY_BIT)
			vsys_adc = ((vsys_adc ^ 0xffff) + 1);//mA
	} else {
		ret = regmap_read(bq->regmap, SC8541_VBAT_ADC_MSB, &vsys_adc_msb);
		if (ret)
			return ret;

		ret = regmap_read(bq->regmap, SC8541_VBAT_ADC_LSB, &vsys_adc_lsb);
		if (ret)
			return ret;

		vsys_adc = (vsys_adc_msb << 8) | vsys_adc_lsb;

		vsys_adc = (vsys_adc * bq->chip_info->adc_vbat_volt_step / 10) /1000;//mV
	}

	return vsys_adc;
}

static int sc8541_get_state(struct sc8541_device *bq,
				struct sc8541_state *state)
{
	unsigned int chg_ctrl_2;
	unsigned int stat1;
	unsigned int stat2;
	unsigned int stat3;
	unsigned int stat4;
	unsigned int stat5;
	unsigned int ibat_adc_msb;
	int ret;

	ret = regmap_read(bq->regmap, SC8541_STAT1, &stat1);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, SC8541_STAT2, &stat2);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, SC8541_STAT3, &stat3);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, SC8541_STAT4, &stat4);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, SC8541_STAT5, &stat5);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, SC8541_CHRGR_CTRL_2, &chg_ctrl_2);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, SC8541_IBAT_ADC_MSB, &ibat_adc_msb);
	if (ret)
		return ret;

	state->dischg = ibat_adc_msb & SC8541_ADC_POLARITY_BIT;
	state->ovp = (stat1 & SC8541_STAT1_OVP_MASK) |
		(stat3 & SC8541_STAT3_OVP_MASK);
	state->ocp = (stat1 & SC8541_STAT1_OCP_MASK) |
		(stat2 & SC8541_STAT2_OCP_MASK);
	state->tflt = stat4 & SC8541_STAT4_TFLT_MASK;
	state->wdt = stat4 & SC8541_WD_STAT;
	state->online = stat3 & SC8541_PRESENT_MASK;
	state->ce = chg_ctrl_2 & SC8541_CHG_EN;
	state->hiz = chg_ctrl_2 & SC8541_EN_HIZ;
	state->bypass = chg_ctrl_2 & SC8541_EN_BYPASS;
	state->cp_switch = stat5 & SC8541_STAT5_CP_SWITCH_MASK;

	bq->alarm_status.bits.bat_ovp_alarm = stat1 & SC8541_STAT1_BAT_OVP_ALM_MASK;
	bq->alarm_status.bits.bat_ocp_alarm = stat1 & SC8541_STAT1_BAT_OCP_ALM_MASK;
	bq->alarm_status.bits.bus_ovp_alarm = stat1 & SC8541_STAT1_BUS_OVP_ALM_MASK;
	bq->alarm_status.bits.bus_ocp_alarm = stat2 & SC8541_STAT2_BUS_OCP_ALM_MASK;
	bq->alarm_status.bits.bat_therm_alarm = stat4 & SC8541_STAT4_TSBUS_TSBAT_ALM_MASK;
	bq->alarm_status.bits.bus_therm_alarm = stat4 & SC8541_STAT4_TSBUS_TSBAT_ALM_MASK;
	bq->alarm_status.bits.die_therm_alarm = stat4 & SC8541_STAT4_TDIE_ALM_MASK;
	bq->alarm_status.bits.bat_ucp_alarm = stat1 & SC8541_STAT1_BAT_UCP_ALM_MASK;

	bq->fault_status.bits.bat_ovp_fault = stat1 & SC8541_STAT1_BAT_OVP_MASK;
	bq->fault_status.bits.bat_ocp_fault = stat1 & SC8541_STAT1_BAT_OCP_MASK;
	bq->fault_status.bits.bus_ovp_fault = stat1 & SC8541_STAT1_BUS_OVP_MASK;
	bq->fault_status.bits.bus_ocp_fault = stat2 & SC8541_STAT2_BUS_OCP_MASK;
	bq->fault_status.bits.bat_therm_fault = stat4 & SC8541_STAT4_TSBAT_FLT_MASK;
	bq->fault_status.bits.bus_therm_fault = stat4 & SC8541_STAT4_TSBUS_FLT_MASK;
	bq->fault_status.bits.die_therm_fault = stat4 & SC8541_STAT4_TDIE_FLT_MASK;

	return 0;
}

static int nu2115_get_state(struct sc8541_device *bq,
				struct sc8541_state *state)
{
	unsigned int chg_ctrl;
	unsigned int ac1_ovp;
	unsigned int ac2_ovp;
	unsigned int flt_stat;
	unsigned int ac_online;
	unsigned int bus_online;
	unsigned int ibat_adc_msb;
	unsigned int alm_stat;
	unsigned int flt_flag;
	unsigned int stat5;
	int ret;

	ret = regmap_read(bq->regmap, NU2115_AC1PROT, &ac1_ovp);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, NU2115_AC2PROT, &ac2_ovp);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, NU2115_FLT_STAT, &flt_stat);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, NU2115_VAC12PRET, &ac_online);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, NU2115_CP_OPTION2, &bus_online);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, NU2115_CHGCTRL, &chg_ctrl);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, NU2115_IBAT_ADC_MSB, &ibat_adc_msb);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, NU2115_INT_STAT, &alm_stat);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, NU2115_FLT_FLAG, &flt_flag);
	if (ret)
		return ret;
	/* delay 150ms for NU2115_CON_STAT out after setting cp enable */
	msleep(150);

	ret = regmap_read(bq->regmap, NU2115_CON_STAT, &stat5);
		if (ret)
			return ret;

	state->dischg = ibat_adc_msb & NU2115_ADC_POLARITY_BIT;
 	state->ovp = (ac1_ovp & NU2115_AC_OVP_MASK) | (ac2_ovp & NU2115_AC_OVP_MASK)
	        | (flt_stat & NU2115_BUS_OVP_MASK) | (flt_stat & NU2115_BAT_OVP_MASK);
	state->ocp = (flt_stat & NU2115_BUS_OCP_MASK) | (flt_stat & NU2115_BAT_OCP_MASK);
	state->tflt = flt_stat & NU2115_TFLT_MASK;
	state->online = (ac_online & NU2115_AC_PRESENT_MASK) | (bus_online & NU2115_BUS_PRESENT_MASK);
	state->ce = chg_ctrl & NU2115_CHG_EN;
	state->bypass = chg_ctrl & NU2115_EN_BYPASS;
	state->cp_switch = stat5 & NU2115_SWITCHING_STAT;

	bq->alarm_status.bits.bat_ovp_alarm = alm_stat & NU2115_STAT1_BAT_OVP_ALM_MASK;
	bq->alarm_status.bits.bat_ocp_alarm = alm_stat & NU2115_STAT1_BAT_OCP_ALM_MASK;
	bq->alarm_status.bits.bus_ovp_alarm = alm_stat & NU2115_STAT1_BUS_OVP_ALM_MASK;
	bq->alarm_status.bits.bus_ocp_alarm = alm_stat & NU2115_STAT2_BUS_OCP_ALM_MASK;
	bq->alarm_status.bits.bat_therm_alarm = flt_stat & NU2115_STAT4_TSBUS_TSBAT_ALM_MASK;
	bq->alarm_status.bits.bus_therm_alarm = flt_stat & NU2115_STAT4_TSBUS_TSBAT_ALM_MASK;
	bq->alarm_status.bits.die_therm_alarm = flt_stat & NU2115_STAT4_TDIE_ALM_MASK;
	bq->alarm_status.bits.bat_ucp_alarm = alm_stat & NU2115_STAT1_BAT_UCP_ALM_MASK;

	bq->fault_status.bits.bat_ovp_fault = flt_stat & NU2115_STAT1_BAT_OVP_MASK;
	bq->fault_status.bits.bat_ocp_fault = flt_stat & NU2115_STAT1_BAT_OCP_MASK;
	bq->fault_status.bits.bus_ovp_fault = flt_stat & NU2115_STAT1_BUS_OVP_MASK;
	bq->fault_status.bits.bus_ocp_fault = flt_stat & NU2115_STAT2_BUS_OCP_MASK;
	bq->fault_status.bits.bat_therm_fault = flt_flag & NU2115_STAT4_TSBAT_FLT_MASK;
	bq->fault_status.bits.bus_therm_fault = flt_flag & NU2115_STAT4_TSBUS_FLT_MASK;
	bq->fault_status.bits.die_therm_fault = flt_flag & NU2115_STAT4_TDIE_FLT_MASK;

	return 0;
}

static int sc8541_set_present(struct sc8541_device *bq, bool present)
{
	bq->usb_present = present;

	if (present)
		sc8541_reg_init(bq);
	return 0;
}

static int sc8541_set_charger_property(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *val)
{
	struct sc8541_device *bq = power_supply_get_drvdata(psy);
	int ret = -EINVAL;

	switch (prop) {
	case POWER_SUPPLY_PROP_PRESENT:
		ret = sc8541_set_present(bq, !!val->intval);
		pr_info("[%s] set present :%d ret=%d\n", bq->model_name, val->intval,ret);
		//dev_err(bq->dev,"%s,POWER_SUPPLY_PROP_PRESENT prop:%d,value:%d\n",__func__,prop,val->intval);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int nu2115_get_health_property(struct sc8541_device *bq, union power_supply_propval *val)
{
	int ret = 0;
	unsigned int ac1_ovp;
	unsigned int ac2_ovp;
	unsigned int flt_stat;
	unsigned int ibat_adc_msb;
	struct sc8541_state state;	

	val->intval = POWER_SUPPLY_HEALTH_GOOD;

	ret = regmap_read(bq->regmap, NU2115_AC1PROT, &ac1_ovp);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, NU2115_AC2PROT, &ac2_ovp);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, NU2115_FLT_STAT, &flt_stat);
	if (ret)
		return ret;
		
	ret = regmap_read(bq->regmap, NU2115_IBAT_ADC_MSB, &ibat_adc_msb);
	if (ret)
		return ret;

	state.ovp = (ac1_ovp & NU2115_AC_OVP_MASK) | (ac2_ovp & NU2115_AC_OVP_MASK)
            | (flt_stat & NU2115_BUS_OVP_MASK) | (flt_stat & NU2115_BAT_OVP_MASK);
	state.ocp = (flt_stat & NU2115_BUS_OCP_MASK) | (flt_stat & NU2115_BAT_OCP_MASK);
	state.tflt = flt_stat & NU2115_TFLT_MASK;

	if (state.tflt)
		val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (state.ovp)
		val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	else if (state.ocp)
		val->intval = POWER_SUPPLY_HEALTH_OVERCURRENT;
	
	return ret;
}

static int sc8541_get_health_property(struct sc8541_device *bq, union power_supply_propval *val)
{
        int ret = 0;
        //unsigned int chg_ctrl_2 = 0;
        unsigned int stat1 = 0;
        unsigned int stat2 = 0;
        unsigned int stat3 = 0;
        unsigned int stat4 = 0;
	struct sc8541_state state;
	
	val->intval = POWER_SUPPLY_HEALTH_GOOD;

	ret = regmap_read(bq->regmap, SC8541_STAT1, &stat1);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, SC8541_STAT2, &stat2);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, SC8541_STAT3, &stat3);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, SC8541_STAT4, &stat4);
	if (ret)
			return ret;

	state.ovp = (stat1 & SC8541_STAT1_OVP_MASK) |
		(stat3 & SC8541_STAT3_OVP_MASK);
	state.ocp = (stat1 & SC8541_STAT1_OCP_MASK) |
		(stat2 & SC8541_STAT2_OCP_MASK);
	state.tflt = stat4 & SC8541_STAT4_TFLT_MASK;

	if (state.tflt)
		val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (state.ovp)
		val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	else if (state.ocp)
		val->intval = POWER_SUPPLY_HEALTH_OVERCURRENT;
	
	return ret;
}

static int sc8541_get_charger_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct sc8541_device *bq = power_supply_get_drvdata(psy);
	int ret = 0;

	switch (psp) {
	//case POWER_SUPPLY_PROP_CHIP_VERSION:
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = SC8541_MANUFACTURER;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = bq->model_name;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		if (bq->part_no == NU2115_PART_NO) {
			ret = nu2115_get_health_property(bq, val);
		} else {
			ret = sc8541_get_health_property(bq, val);
		}
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = sc8541_get_ibat_adc(bq);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = sc8541_get_adc_vbat(bq);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = sc8541_get_const_charge_curr(bq);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = sc8541_get_const_charge_volt(bq);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;
	
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = sc8541_get_adc_vbus(bq);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;
	
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = sc8541_get_adc_ibus(bq);
		if (ret < 0)
			return ret;

		val->intval = ret;
		//dump_all_reg(bq);
		break;

	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = bq->state.online;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static bool sc8541_state_changed(struct sc8541_device *bq,
				  struct sc8541_state *new_state)
{
	struct sc8541_state old_state;

	mutex_lock(&bq->lock);
	old_state = bq->state;
	mutex_unlock(&bq->lock);

	return (old_state.dischg != new_state->dischg ||
		old_state.ovp != new_state->ovp ||
		old_state.ocp != new_state->ocp ||
		old_state.online != new_state->online ||
		old_state.wdt != new_state->wdt ||
		old_state.tflt != new_state->tflt ||
		old_state.ce != new_state->ce ||
		old_state.hiz != new_state->hiz ||
		old_state.bypass != new_state->bypass ||
		old_state.cp_switch != new_state->cp_switch);
}

static irqreturn_t sc8541_irq_handler_thread(int irq, void *private)
{
	struct sc8541_device *bq = private;
	struct sc8541_state state;
	int ret;

	dev_err(bq->dev,"[%s]%s enter\n",bq->model_name,__func__);
	mutex_lock(&bq->irq_complete);
	bq->irq_waiting = true;
	if (!bq->resume_completed) {
		dev_dbg(bq->dev, "IRQ triggered before device-resume\n");
		if (!bq->irq_disabled) {
			disable_irq_nosync(irq);
			bq->irq_disabled = true;
		}
		mutex_unlock(&bq->irq_complete);
		return IRQ_HANDLED;
	}
	bq->irq_waiting = false;

	if(bq->irq_counts > INT_MAX -1)
		bq->irq_counts = 0;
	else
		bq->irq_counts++;


	mutex_unlock(&bq->irq_complete);

	if (bq->part_no == NU2115_PART_NO) {
		ret = nu2115_get_state(bq, &state);
		if(ret < 0)
			goto irq_out;
	} else {
		ret = sc8541_get_state(bq, &state);
		if (ret < 0)
			goto irq_out;
	}
	if(bq->alarm_status.status > 0 ||
		bq->fault_status.status > 0)
		dump_all_reg(bq);
	if (!sc8541_state_changed(bq, &state))
		goto irq_out;

	mutex_lock(&bq->lock);
	bq->state = state;
	mutex_unlock(&bq->lock);

	power_supply_changed(bq->charger);

irq_out:
	return IRQ_HANDLED;
}

static enum power_supply_property sc8541_power_supply_props[] = {
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_MODEL_NAME,
	//POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
//	POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
//	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
//	POWER_SUPPLY_PROP_CHARGING_ENABLED,
/*
	POWER_SUPPLY_PROP_TI_ADC,
	POWER_SUPPLY_PROP_TI_BYPASS,
*/

	//POWER_SUPPLY_PROP_CP_STATUS1,//
	POWER_SUPPLY_PROP_PRESENT,
//	POWER_SUPPLY_PROP_CHARGING_ENABLED,//undeclared identifier
//	POWER_SUPPLY_PROP_VOLTAGE_NOW,
//	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	//POWER_SUPPLY_PROP_CP_IRQ_STATUS,//undeclared identifier
	//POWER_SUPPLY_PROP_CHIP_VERSION,
};

static char *sc8541_charger_supplied_to[] = {
	"main-battery",
};

static int sc8541_property_is_writeable(struct power_supply *psy,
					 enum power_supply_property prop)
{
	switch (prop) {
//	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
//	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
//	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
	case POWER_SUPPLY_PROP_PRESENT:
//	case POWER_SUPPLY_PROP_UPDATE_NOW:
//	case POWER_SUPPLY_PROP_TI_ADC:
//	case POWER_SUPPLY_PROP_TI_BYPASS:
		return true;
	default:
		return false;
	}
}

static bool sc8541_is_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SC8541_CHRGR_CTRL_2:
	case SC8541_STAT1...SC8541_FLAG5:
	case SC8541_ADC_CONTROL1...SC8541_TDIE_ADC_LSB:
		return true;
	default:
		return false;
	}
}

static bool nu2115_is_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case NU2115_AC1PROT:
	case NU2115_AC2PROT...NU2115_FLT_FLAG:
	case NU2115_ADC_CTRL...NU2115_TDIE_ADC_LSB:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config sc8541_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = SC8541_CTRL6_REG,
	.reg_defaults	= sc8541_reg_defs,
	.num_reg_defaults = ARRAY_SIZE(sc8541_reg_defs),
	.cache_type = REGCACHE_NONE,
	.volatile_reg = sc8541_is_volatile_reg,
};


static const struct sc8541_chip_info sc8541_chip_info_tbl[] = {
	[SC8541] = {
		.model_id = SC8541,
		.regmap_config = &sc8541_regmap_config,
		.reg_init_values = sc8541_reg_init_val,

		.busocp_sc_def = SC8541_BUSOCP_SC_DFLT_uA,
		.busocp_byp_def = SC8541_BUSOCP_BYP_DFLT_uA,
		.busocp_sc_min = SC8541_BUSOCP_MIN_uA,
		.busocp_sc_max = SC8541_BUSOCP_SC_MAX_uA,
		.busocp_byp_min = SC8541_BUSOCP_MIN_uA,
		.busocp_byp_max = SC8541_BUSOCP_BYP_MAX_uA,
		.busocp_step = SC8541_BUSOCP_STEP_uA,
		.busocp_offset = SC8541_BUSOCP_OFFSET_uA,

		.busovp_sc_def = SC8541_BUSOVP_DFLT_uV,
		.busovp_byp_def = SC8541_BUSOVP_BYPASS_DFLT_uV,
		.busovp_sc_step = SC8541_BUSOVP_SC_STEP_uV,
		.busovp_sc_offset = SC8541_BUSOVP_SC_OFFSET_uV,
		.busovp_byp_step = SC8541_BUSOVP_BYP_STEP_uV,
		.busovp_byp_offset = SC8541_BUSOVP_BYP_OFFSET_uV,
		.busovp_sc_min = SC8541_BUSOVP_SC_MIN_uV,
		.busovp_sc_max = SC8541_BUSOVP_SC_MAX_uV,
		.busovp_byp_min = SC8541_BUSOVP_BYP_MIN_uV,
		.busovp_byp_max = SC8541_BUSOVP_BYP_MAX_uV,

		.batovp_def = SC8541_BATOVP_DFLT_uV,
		.batovp_max = SC8541_BATOVP_MAX_uV,
		.batovp_min = SC8541_BATOVP_MIN_uV,
		.batovp_step = SC8541_BATOVP_STEP_uV,
		.batovp_offset = SC8541_BATOVP_OFFSET_uV,

		.batocp_def = SC8541_BATOCP_DFLT_uA,
		.batocp_max = SC8541_BATOCP_MAX_uA,

		.adc_curr_step = SC8541_ADC_CURR_STEP_IBUS_uA,
		.adc_vbat_volt_step = SC8541_ADC_VOLT_STEP_VBAT_deciuV,
		.adc_vbus_volt_step = SC8541_ADC_VOLT_STEP_VBUS_deciuV,
		.adc_vbus_volt_offset = 0,
		.adc_vout_volt_step = SC8541_ADC_VOLT_STEP_VOUT_deciuV,
		.adc_vout_volt_offset = 0,
	},
};

static const struct regmap_config nu2115_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = NU2115_CTRL6_REG,
	.reg_defaults	= nu2115_reg_defs,
	.num_reg_defaults = ARRAY_SIZE(nu2115_reg_defs),
	.cache_type = REGCACHE_NONE,
	.volatile_reg = nu2115_is_volatile_reg,
};

static const struct sc8541_chip_info nu2115_chip_info_tbl[] = {
	[SC8541] = {
		.model_id = SC8541,
		.regmap_config = &nu2115_regmap_config,
		.reg_init_values = nu2115_reg_init_val,

		.busocp_sc_def = NU2115_BUSOCP_SC_DFLT_uA,
		.busocp_byp_def = NU2115_BUSOCP_BYP_DFLT_uA,
		.busocp_sc_min = NU2115_BUSOCP_MIN_uA,
		.busocp_sc_max = NU2115_BUSOCP_SC_MAX_uA,
		.busocp_byp_min = NU2115_BUSOCP_MIN_uA,
		.busocp_byp_max = NU2115_BUSOCP_BYP_MAX_uA,
		.busocp_step = NU2115_BUSOCP_STEP_uA,
		.busocp_offset = NU2115_BUSOCP_OFFSET_uA,

		.busovp_sc_def = NU2115_BUSOVP_DFLT_uV,
		.busovp_byp_def = NU2115_BUSOVP_BYPASS_DFLT_uV,
		.busovp_sc_step = NU2115_BUSOVP_SC_STEP_uV,
		.busovp_sc_offset = NU2115_BUSOVP_SC_OFFSET_uV,
		.busovp_byp_step = NU2115_BUSOVP_BYP_STEP_uV,
		.busovp_byp_offset = NU2115_BUSOVP_BYP_OFFSET_uV,
		.busovp_sc_min = NU2115_BUSOVP_SC_MIN_uV,
		.busovp_sc_max = NU2115_BUSOVP_SC_MAX_uV,
		.busovp_byp_min = NU2115_BUSOVP_BYP_MIN_uV,
		.busovp_byp_max = NU2115_BUSOVP_BYP_MAX_uV,

		.batovp_def = NU2115_BATOVP_DFLT_uV,
		.batovp_max = NU2115_BATOVP_MAX_uV,
		.batovp_min = NU2115_BATOVP_MIN_uV,
		.batovp_step = NU2115_BATOVP_STEP_uV,
		.batovp_offset = NU2115_BATOVP_OFFSET_uV,

		.batocp_def = NU2115_BATOCP_DFLT_uA,
		.batocp_max = NU2115_BATOCP_MAX_uA,

		.adc_curr_step = NU2115_ADC_CURR_STEP_IBUS_uA,
		.adc_vbat_volt_step = NU2115_ADC_VOLT_STEP_VBAT_deciuV,
		.adc_vbus_volt_step = NU2115_ADC_VOLT_STEP_VBUS_deciuV,
		.adc_vbus_volt_offset = 0,
		.adc_vout_volt_step = NU2115_ADC_VOLT_STEP_VOUT_deciuV,
		.adc_vout_volt_offset = 0,
	},
};

static int sc8541_power_supply_init(struct sc8541_device *bq,
							struct device *dev,
							int driver_data)
{
	struct power_supply_config psy_cfg = { .drv_data = bq,
						.of_node = dev->of_node, };

	psy_cfg.supplied_to = sc8541_charger_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(sc8541_charger_supplied_to);

	switch (driver_data) {
	case SC8541_MASTER:
		bq->psy_desc.name = "bq25960-master";
		break;
	case SC8541_SLAVE:
		bq->psy_desc.name = "bq25960-slave";
		break;
	case SC8541_STANDALONE:
		bq->psy_desc.name = "bq25960-standalone";
		break;
	default:
		return -EINVAL;
	}

	bq->psy_desc.type = POWER_SUPPLY_TYPE_MAINS,
	bq->psy_desc.properties = sc8541_power_supply_props,
	bq->psy_desc.num_properties = ARRAY_SIZE(sc8541_power_supply_props),
	bq->psy_desc.get_property = sc8541_get_charger_property,
	bq->psy_desc.set_property = sc8541_set_charger_property,
	bq->psy_desc.property_is_writeable = sc8541_property_is_writeable,

	bq->charger = devm_power_supply_register(bq->dev,
						 &bq->psy_desc,
						 &psy_cfg);
	if (IS_ERR(bq->charger)) {
		dev_err(bq->dev, "bq register power supply fail");
		return -EINVAL;
	}

	return 0;
}

static int sc8541_reg_init(struct sc8541_device *bq)
{
	int i, ret;

	if (bq->part_no == NU2115_PART_NO) {
		for (i = 0; i < ARRAY_SIZE(nu2115_reg_init_val); i++) {
			ret = regmap_update_bits(bq->regmap, bq->chip_info->reg_init_values[i].reg,
				0xFF, bq->chip_info->reg_init_values[i].def);
			dev_notice(bq->dev, "init Reg[%02X] = 0x%02X\n",
				bq->chip_info->reg_init_values[i].reg,
				bq->chip_info->reg_init_values[i].def);
			if (ret) {
				dev_err(bq->dev, "Reg init fail ret=%d", ret);
				return ret;
			}
		}
	} else {
		for (i = 0; i < ARRAY_SIZE(sc8541_reg_init_val); i++) {
			ret = regmap_update_bits(bq->regmap, bq->chip_info->reg_init_values[i].reg,
				0xFF, bq->chip_info->reg_init_values[i].def);
			dev_notice(bq->dev, "init Reg[%02X] = 0x%02X\n",
				bq->chip_info->reg_init_values[i].reg,
				bq->chip_info->reg_init_values[i].def);
			if (ret) {
				dev_err(bq->dev, "Reg init fail ret=%d", ret);
				return ret;
			}
		}
	}
	return 0;
}

static int sc8541_parse_dt(struct sc8541_device *bq)
{
	int ret;

	ret = device_property_read_u32(bq->dev, "ti,watchdog-timeout-ms",
				       &bq->watchdog_timer);
	if (ret)
		bq->watchdog_timer = SC8541_WATCHDOG_MIN;

	if (bq->watchdog_timer > SC8541_WATCHDOG_MAX ||
		bq->watchdog_timer < SC8541_WATCHDOG_MIN)
		return -EINVAL;

	ret = device_property_read_u32(bq->dev,
				       "ti,sc-ovp-limit-microvolt",
				       &bq->init_data.sc_vlim);
	if (ret)
		bq->init_data.sc_vlim = bq->chip_info->busovp_sc_def;

	if (bq->init_data.sc_vlim > bq->chip_info->busovp_sc_max ||
	    bq->init_data.sc_vlim < bq->chip_info->busovp_sc_min) {
		dev_err(bq->dev, "SC ovp limit is out of range\n");
		return -EINVAL;
	}

	ret = device_property_read_u32(bq->dev,
				       "ti,sc-ocp-limit-microamp",
				       &bq->init_data.sc_ilim);
	if (ret)
		bq->init_data.sc_ilim = bq->chip_info->busocp_sc_def;

	if (bq->init_data.sc_ilim > bq->chip_info->busocp_sc_max ||
	    bq->init_data.sc_ilim < bq->chip_info->busocp_sc_min) {
		dev_err(bq->dev, "SC ocp limit is out of range\n");
		return -EINVAL;
	}

	ret = device_property_read_u32(bq->dev,
				       "ti,bypass-ovp-limit-microvolt",
				       &bq->init_data.bypass_vlim);
	if (ret)
		bq->init_data.bypass_vlim = bq->chip_info->busovp_byp_def;

	if (bq->init_data.bypass_vlim > bq->chip_info->busovp_byp_max ||
	    bq->init_data.bypass_vlim < bq->chip_info->busovp_byp_min) {
		dev_err(bq->dev, "Bypass ovp limit is out of range\n");
		return -EINVAL;
	}

	ret = device_property_read_u32(bq->dev,
				       "ti,bypass-ocp-limit-microamp",
				       &bq->init_data.bypass_ilim);
	if (ret)
		bq->init_data.bypass_ilim = bq->chip_info->busocp_byp_def;

	if (bq->init_data.bypass_ilim > bq->chip_info->busocp_byp_max ||
	    bq->init_data.bypass_ilim < bq->chip_info->busocp_byp_min) {
		dev_err(bq->dev, "Bypass ocp limit is out of range\n");
		return -EINVAL;
	}


	bq->state.bypass = device_property_read_bool(bq->dev,
						      "ti,bypass-enable");
	return 0;
}

static int sc8541_check_work_mode(struct sc8541_device *bq)
{
	int ret;
	int val;

	if (bq->part_no == NU2115_PART_NO) {
		ret = regmap_read(bq->regmap, NU2115_IBUS_UCP, &val);
		if (ret) {
			dev_err(bq->dev, "Failed to read operation mode register\n");
			return ret;
		}

		val = (val & NU2115_MS_MASK);
	} else {
		ret = regmap_read(bq->regmap, SC8541_CHRGR_CTRL_5, &val);
		if (ret) {
			dev_err(bq->dev, "Failed to read operation mode register\n");
			return ret;
		}

		val = (val & SC8541_MS_MASK);
	}

	if (bq->mode != val) {
		dev_err(bq->dev, "dts mode %d mismatch with hardware mode %d\n", bq->mode, val);
		return -EINVAL;
	}

	dev_info(bq->dev, "work mode:%s\n", bq->mode == SC_STANDALONE ? "Standalone" :
			(bq->mode == SC_SLAVE ? "Slave" : "Master"));
	return 0;
}
static int sc8541_get_part_no(struct sc8541_device *bq)
{
	struct i2c_client client;
	int ret;
	int len;
	const char *nu2115_name;

	bq->part_no = 0;

	ret = device_property_read_u32(bq->dev, "nu2115-addr",
				       &bq->nu2115_addr);
	if(ret)
		return SC8541_PART_NO;

	client = *(bq->client);
	ret = i2c_smbus_read_byte_data(&client, SC8541_DEVICE_INFO);
	if(ret == SC8541_PART_NO) {
		return SC8541_PART_NO; //read success
	}else {
		pr_info("nu2115_get_part_no: orig addr = %d, nu2115 addr =%d\n ",
			client.addr, bq->nu2115_addr);
		client.addr = bq->nu2115_addr;
		ret = i2c_smbus_read_byte_data(&client, NU2115_DEV_INFO);
		if(ret == NU2115_PART_NO) {
			memset((void*)bq->model_name, 0x00, sizeof(bq->model_name));
			device_property_read_string(bq->dev, "nu2115-name", &nu2115_name);
			len = strlen(nu2115_name);
			strncpy(bq->model_name, nu2115_name, min(I2C_NAME_SIZE,len) );
				pr_err("[%s] model_name=%s\n", __func__ , bq->model_name);
		}
	}

	return ret;
}

static ssize_t show_reg_addr(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sc8541_device *bq = dev_get_drvdata(dev);
	if (!bq) {
		pr_err("[%s] chip not valid\n", bq->model_name);
		return -ENODEV;
	}
	return sprintf(buf, "reg addr 0x%02x\n", bq->reg_addr);
}

static ssize_t store_reg_addr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int tmp;
	struct sc8541_device *bq = dev_get_drvdata(dev);
	if (!bq) {
		pr_err("[%s] chip not valid\n", bq->model_name);
		return -ENODEV;
	}

	tmp = simple_strtoul(buf, NULL, 0);
	bq->reg_addr = tmp;

	return count;
}
static DEVICE_ATTR(reg_addr, 0664, show_reg_addr, store_reg_addr);


static ssize_t show_reg_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	//int ret;
	ssize_t size = 0;
	struct sc8541_device *bq = dev_get_drvdata(dev);
	if (!bq) {
		pr_err("[%s] chip not valid\n", bq->model_name);
		return -ENODEV;
	}
	if( ( bq->reg_addr >= 0 ) && (bq->reg_addr <= 0x37) )
	{
		//ret = regmap_read(bq->regmap, bq->reg_addr, &bq->reg_data);
		regmap_read(bq->regmap, bq->reg_addr, &bq->reg_data);
		size = sprintf(buf, "reg[%02X]=0x%02X\n", bq->reg_addr, bq->reg_data);
	}else
	{
		size = sprintf(buf, "reg addr error\n");
	}
	return size;
}

static ssize_t store_reg_data(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int tmp;
	struct sc8541_device *bq = dev_get_drvdata(dev);
	if (!bq) {
		pr_err("[%s] chip not valid\n", bq->model_name);
		return -ENODEV;
	}

	tmp = simple_strtoul(buf, NULL, 0);
	bq->reg_data = tmp;

	if( (bq->reg_addr >= 0) && (bq->reg_addr <= 0x37) ) {
		if( (bq->reg_data >= 0) && (bq->reg_data <= 0xFF) ) {
			regmap_write(bq->regmap, bq->reg_addr, bq->reg_data);
		}else
			pr_err("reg data error : data=0x%X\n", bq->reg_data);
	}else
		pr_err("reg addr error : addr=0x%X\n", bq->reg_addr);

	return count;
}
static DEVICE_ATTR(reg_data, 0664, show_reg_data, store_reg_data);

static ssize_t show_force_chg_auto_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	int state = 0;
	bool enable;
	struct sc8541_device *bq = dev_get_drvdata(dev);
	if (!bq) {
		pr_err("[%s] chip not valid\n", bq->model_name);
		state = -ENODEV;
	}else {
		ret = sc8541_is_chg_en(bq, &enable);
		if (ret < 0) {
			pr_err("[%s] sc8541_is_chg_en not valid\n", bq->model_name);
			state = -ENODEV;
		}else
			state = enable;
	}
	return sprintf(buf, "%d\n", state);
}

static ssize_t store_force_chg_auto_enable(struct device *dev,
						 struct device_attribute *attr,
						 const char *buf, size_t count)
{
	int ret;
	bool enable;
	struct sc8541_device *bq = dev_get_drvdata(dev);
	if (!bq) {
		pr_err("[%s] chip not valid\n", bq->model_name);
		return -ENODEV;
	}

	enable = simple_strtoul(buf, NULL, 0);
	ret = sc8541_set_chg_en(bq, enable);
	if (ret) {
		pr_err("[%s] Couldn't %s charging rc=%d\n", bq->model_name,
			   enable ? "enable" : "disable", (int)ret);
		return ret;
	}

	pr_info("[%s] %s charging \n", bq->model_name,
			   enable ? "enable" : "disable");

	return count;
}
static DEVICE_ATTR(force_chg_auto_enable, 0664, show_force_chg_auto_enable, store_force_chg_auto_enable);

static ssize_t show_reg_dump(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	unsigned int val;
	int addr;
	ssize_t size = 0;
	struct sc8541_device *bq = dev_get_drvdata(dev);
	if (!bq) {
		pr_err("[%s] chip not valid\n", bq->model_name);
		return -ENODEV;
	}

	for (addr = 0; addr <= 0x3a; addr++) {
		ret = regmap_read(bq->regmap, addr, &val);
		if (!ret) {
			pr_err("%s_dump_register Reg[%02X]=0x%02X\n", bq->model_name, addr, val);
			size += snprintf(buf + size, PAGE_SIZE - size,
				"reg[%02X]=[0x%02X]\n", addr,val);
		}else {
			size += snprintf(buf + size, PAGE_SIZE - size,
				"reg[%02X]=[failed]\n", addr);
		}
	}

	return size;
}
static DEVICE_ATTR(reg_dump, 0444, show_reg_dump, NULL);

static ssize_t show_vbus(struct device *dev, struct device_attribute *attr, char *buf)
{
	int vbus;
	struct sc8541_device *bq = dev_get_drvdata(dev);
	if (!bq) {
		pr_err("[%s] chip not valid\n", bq->model_name);
		return -ENODEV;
	}
	vbus = sc8541_get_adc_vbus(bq);

	return sprintf(buf, "%d\n", vbus);
}
static DEVICE_ATTR(vbus, 0444, show_vbus, NULL);

static void sc8541_create_device_node(struct device *dev)
{
	device_create_file(dev, &dev_attr_force_chg_auto_enable);
	device_create_file(dev, &dev_attr_reg_addr);
	device_create_file(dev, &dev_attr_reg_data);
	device_create_file(dev, &dev_attr_reg_dump);
	device_create_file(dev, &dev_attr_vbus);
}

static int sc8541_iio_write_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int val1,
		int val2, long mask)
{
	struct sc8541_device *bq = iio_priv(indio_dev);
	int rc = 0;

	pr_err("[%s] iio_write_raw ch=%d val1=%d val2=%d\n",
			bq->model_name, chan->channel, val1, val2);
	switch (chan->channel) {
	case PSY_IIO_CP_ENABLE:
		sc8541_set_chg_en(bq, val1);
		pr_info("[%s] set_chg_en: %s\n", bq->model_name,
				val1 ? "enable" : "disable");
		break;
	case PSY_IIO_MMI_OTG_ENABLE:
		sc8541_set_otg_en(bq, !!val1);
		pr_info("[%s] set_otg_en: %s\n", bq->model_name,
				!!val1 ? "enable" : "disable");
		break;
	case PSY_IIO_ONLINE:
		sc8541_set_present(bq, !!val1);
		pr_info("[%s] set_present :%d\n", bq->model_name, val1);
		break;
	case PSY_IIO_CP_CLEAR_ERROR:
		bq->fault_status.status = 0;
		bq->alarm_status.status = 0;
		break;
	case PSY_IIO_CP_STATUS1:
		if (val1 == MMI_DISABLE_ADC)
			sc8541_set_adc_enable(bq, false);
		else if (val1 == MMI_ENABLE_ADC)
			sc8541_set_adc_enable(bq, true);
		break;
	default:
		pr_err("Unsupported [%s] IIO chan %d\n",
				bq->model_name, chan->channel);
		rc = -EINVAL;
		break;
	}

	if (rc < 0)
		pr_err("[%s] Couldn't write IIO channel %d, rc = %d\n",
			bq->model_name,
			chan->channel, rc);

	return rc;
}

static int sc8541_iio_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int *val1,
		int *val2, long mask)
{
	struct sc8541_device *bq = iio_priv(indio_dev);
	int rc = 0;
	int rc1 = 0;
	int rc2 = 0;
	int result;
	unsigned int ac_online;
	unsigned int bus_online;
	struct sc8541_state state;

	*val1 = 0;
	pr_err("[%s] iio_read_raw ch=%d \n",bq->model_name,chan->channel);
	switch (chan->channel) {
	case PSY_IIO_CP_ENABLE:
		if (bq->part_no == NU2115_PART_NO) {
			rc = regmap_read(bq->regmap, NU2115_CHGCTRL, &result);
			if (!rc) {
				bq->state.ce = result & NU2115_CHG_EN;
				*val1 = bq->state.ce;
				pr_info("[%s] read charge enable:%d\n", bq->model_name, *val1);
			} else
				pr_err("[%s] read charge enable err\n", bq->model_name);
		} else {
			rc = regmap_read(bq->regmap, SC8541_CHRGR_CTRL_2, &result);
			if (!rc) {
				bq->state.ce = result & SC8541_CHG_EN;
				*val1 = bq->state.ce;
				pr_info("[%s] read charge enable:%d\n", bq->model_name, *val1);
			} else
				pr_err("[%s] read charge enable err\n", bq->model_name);
		}
		break;
	case PSY_IIO_MMI_OTG_ENABLE:
		if (bq->part_no == NU2115_PART_NO) {
			rc = regmap_read(bq->regmap, NU2115_VAC12PRET, &result);
			if (!rc) {
				*val1 = !!(result & NU2115_EN_OTG);
				pr_info("[%s] read otg enable:%d\n", bq->model_name, *val1);
			}
		} else {
			rc = regmap_read(bq->regmap, SC8541_CHRGR_CTRL_2, &result);
			if (!rc) {
				*val1 = !!(result & SC8541_EN_OTG);
				pr_info("[%s] read otg enable:%d\n", bq->model_name, *val1);
			}
		}
		break;
	case PSY_IIO_MMI_CP_CHIP_ID:
		if (bq->part_no == NU2115_PART_NO)
			*val1 = NU2115_PART_NO;

		if (bq->part_no == SC8541_PART_NO)
			*val1 = SC8541_PART_NO;
		break;
	case PSY_IIO_ONLINE:
		if (bq->part_no == NU2115_PART_NO) {
			rc1 = regmap_read(bq->regmap, NU2115_VAC12PRET, &ac_online);
			rc2 = regmap_read(bq->regmap, NU2115_CP_OPTION2, &bus_online);
			if (!rc1 && !rc2) {
				bq->state.online = (ac_online & NU2115_AC_PRESENT_MASK) | (bus_online & NU2115_BUS_PRESENT_MASK);
				*val1 = bq->state.online;
				pr_info("[%s] read online:%d\n", bq->model_name, *val1);
			} else
				pr_err("[%s] read online err\n", bq->model_name);
		} else {
			rc = regmap_read(bq->regmap, SC8541_STAT3, &result);
			if (!rc) {
				bq->state.online = result & SC8541_PRESENT_MASK;
				*val1 = bq->state.online;
				pr_info("[%s] read online:%d\n", bq->model_name, *val1);
			}else
				pr_err("[%s] read online err\n", bq->model_name);
		}
		break;
	case PSY_IIO_MMI_CP_INPUT_VOLTAGE_NOW:
		rc = sc8541_get_adc_vbus(bq);
		if ( rc < 0 ) {
			pr_err("[%s] get_adc_vbus err %d\n", bq->model_name, rc);
		}else {
			*val1 = rc;
			pr_info("[%s] get_adc_vbus %duV\n", bq->model_name, *val1);
		}
		break;
	case PSY_IIO_MMI_CP_INPUT_CURRENT_NOW:
		rc = sc8541_get_adc_ibus(bq);
		if ( rc < 0 ) {
			pr_err("[%s] get_adc_ibus err %d\n", bq->model_name, rc);
		}else {
			*val1 = rc;
			pr_info("[%s] get_adc_ibus %duA\n", bq->model_name, *val1);
		}
		break;
	case PSY_IIO_CP_STATUS1:
		if (bq->part_no == NU2115_PART_NO) {
			rc = nu2115_get_state(bq,&state);
		} else {
			rc = sc8541_get_state(bq,&state);
		}

		if (rc) {
			pr_err("[%s] get_state err\n", bq->model_name);
			return rc;
		}else {
			if (!sc8541_state_changed(bq, &state)){
				mutex_lock(&bq->lock);
				bq->state = state;
				mutex_unlock(&bq->lock);
				power_supply_changed(bq->charger);
			}
			*val1 = bq->alarm_status.status | (bq->fault_status.status << 8) | (state.cp_switch << 18);
			pr_err("[%s] get_state fault:0x%02X , alarm:0x%02X , cp_switch:%d\n",
					bq->model_name,
					bq->fault_status.status,
					bq->alarm_status.status,
					state.cp_switch);
		}
		break;
	case PSY_IIO_CURRENT_NOW:
		rc = sc8541_get_ibat_adc(bq);
		if ( rc < 0) {
			pr_err("[%s] get_ibat_adc err %d\n", bq->model_name, rc);
		}else {
			*val1 = rc;
			pr_info("[%s] get_ibat_adc %duA\n", bq->model_name, *val1);
		}
		break;
	case PSY_IIO_VOLTAGE_NOW:
		rc = sc8541_get_adc_vbat(bq);
		if ( rc < 0 ) {
			pr_err("[%s] get_adc_vbat err %d\n", bq->model_name, rc);
		}else {
			*val1 = rc;
			pr_info("[%s] get_adc_vbat %duV\n", bq->model_name, *val1);
		}
		break;
	default:
		pr_err("[%s] Unsupported sc8541 IIO chan %d\n", bq->model_name, chan->channel);
		rc = -EINVAL;
		break;
	}

	if (rc < 0) {
		pr_err("[%s] Couldn't read IIO channel %d, rc = %d\n", bq->model_name,
			chan->channel, rc);
		return rc;
	}

	return IIO_VAL_INT;
}

static int sc8541_iio_of_xlate(struct iio_dev *indio_dev,
				const struct fwnode_reference_args *iiospec)
{
	struct sc8541_device *bq = iio_priv(indio_dev);
	struct iio_chan_spec *iio_chan = bq->iio_chan;
	int i;

	for (i = 0; i < ARRAY_SIZE(sc8541_iio_psy_channels);
					i++, iio_chan++)
		if (iio_chan->channel == iiospec->args[0])
			return i;

	return -EINVAL;
}

static const struct iio_info sc8541_iio_info = {
	.read_raw	= sc8541_iio_read_raw,
	.write_raw	= sc8541_iio_write_raw,
	.fwnode_xlate	= sc8541_iio_of_xlate,
};

static int sc8541_init_iio_psy(struct sc8541_device *bq)
{
	struct iio_dev *indio_dev = bq->indio_dev;
	struct iio_chan_spec *chan;
	int sc8541_num_iio_channels = ARRAY_SIZE(sc8541_iio_psy_channels);
	int rc, i;

	bq->iio_chan = devm_kcalloc(bq->dev, sc8541_num_iio_channels,
				sizeof(*bq->iio_chan), GFP_KERNEL);
	if (!bq->iio_chan)
		return -ENOMEM;

	bq->int_iio_chans = devm_kcalloc(bq->dev,
				sc8541_num_iio_channels,
				sizeof(*bq->int_iio_chans),
				GFP_KERNEL);
	if (!bq->int_iio_chans)
		return -ENOMEM;

	indio_dev->info = &sc8541_iio_info;
	indio_dev->dev.parent = bq->dev;
	indio_dev->dev.of_node = bq->dev->of_node;
	indio_dev->name = bq->model_name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = bq->iio_chan;
	indio_dev->num_channels = sc8541_num_iio_channels;

	for (i = 0; i < sc8541_num_iio_channels; i++) {
		bq->int_iio_chans[i].indio_dev = indio_dev;
		chan = &bq->iio_chan[i];
		bq->int_iio_chans[i].channel = chan;
		chan->address = i;
		chan->channel = sc8541_iio_psy_channels[i].channel_num;
		chan->type = sc8541_iio_psy_channels[i].type;
		chan->datasheet_name =
			sc8541_iio_psy_channels[i].datasheet_name;
		chan->extend_name =
			sc8541_iio_psy_channels[i].datasheet_name;
		chan->info_mask_separate =
			sc8541_iio_psy_channels[i].info_mask;
	}

	rc = devm_iio_device_register(bq->dev, indio_dev);
	if (rc)
		pr_err("Failed to register [%s] IIO device, rc=%d\n", indio_dev->name,rc);
	else
		pr_err("Success to register [%s] IIO device\n", indio_dev->name);

	return rc;
}

static int sc8541_parse_dt_id(struct sc8541_device *bq, int driver_data)
{
	switch (driver_data) {
	case SC8541_STANDALONE:
		bq->device_id = SC8541;
		bq->mode = SC_STANDALONE;
		break;
	case SC8541_SLAVE:
		bq->device_id = SC8541;
		bq->mode = SC_SLAVE;
		break;
	case SC8541_MASTER:
		bq->device_id = SC8541;
		bq->mode = SC_MASTER;
		break;
	default:
		dev_err(bq->dev, "dts compatible id %d is unknown", driver_data);
		return -EINVAL;
		break;
	}

	return 0;
}

static int sc8541_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct sc8541_device *bq;
	int ret, irq_gpio, irqn;
	struct iio_dev *indio_dev;

	printk("-------[%s] driver probe--------\n",id->name);

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*bq));

	if (!indio_dev)
		return -ENOMEM;

	bq = iio_priv(indio_dev);
	if (!bq) {
		dev_err(dev, "Out of memory\n");
		return -ENOMEM;
	}

	bq->indio_dev = indio_dev;
	bq->client = client;
	bq->dev = dev;

	mutex_init(&bq->lock);
	mutex_init(&bq->irq_complete);

	bq->resume_completed = true;
	bq->irq_waiting = false;

	strncpy(bq->model_name, id->name, I2C_NAME_SIZE);

	ret = sc8541_parse_dt_id(bq, id->driver_data);
	if (ret)
		goto free_mem;
	bq->part_no = sc8541_get_part_no(bq);

	if ( bq->part_no == SC8541_PART_NO ) {
		bq->chip_info = &sc8541_chip_info_tbl[bq->device_id];
	} else {
		bq->client->addr = bq->nu2115_addr;
		bq->chip_info = &nu2115_chip_info_tbl[bq->device_id];
	}

	bq->regmap = devm_regmap_init_i2c(client,
					  bq->chip_info->regmap_config);
	if (IS_ERR(bq->regmap)) {
		dev_err(dev, "Failed to allocate register map\n");
		devm_kfree(bq->dev, bq);
		return PTR_ERR(bq->regmap);
	}

	i2c_set_clientdata(client, bq);

	ret = sc8541_check_work_mode(bq);
	if (ret)
		goto free_mem;

	ret = sc8541_parse_dt(bq);
	if (ret) {
		dev_err(dev, "Failed to read device tree properties%d\n", ret);
		goto free_mem;
	}

#ifdef CONFIG_INTERRUPT_AS_GPIO
	irq_gpio = of_get_named_gpio(client->dev.of_node, "irq-gpio", 0);
	if (!gpio_is_valid(irq_gpio))
	{
		dev_err(bq->dev, "%s: %d gpio get failed\n", __func__, irq_gpio);
		goto free_mem;
	}
	ret = gpio_request(irq_gpio, "sc8541 irq pin");
	if (ret) {
		dev_err(bq->dev, "%s: %d gpio request failed\n", __func__, irq_gpio);
		goto free_mem;
	}
	gpio_direction_input(irq_gpio);
	irqn = gpio_to_irq(irq_gpio);
	if (irqn < 0) {
		dev_err(bq->dev, "%s:%d gpio_to_irq failed\n", __func__, irqn);
		goto free_mem;
	}
	client->irq = irqn;
#else

	bq->irq_pinctrl =
		pinctrl_get_select(bq->dev, "bq25960_int_default");
	if (!bq->irq_pinctrl) {
		dev_err(bq->dev,"Couldn't set pinctrl bq25960_int_default\n");
		goto free_mem;
	}
#endif

	if (client->irq) {
		ret = devm_request_threaded_irq(dev, client->irq, NULL,
						sc8541_irq_handler_thread,
						IRQF_TRIGGER_FALLING |
						IRQF_ONESHOT,
						dev_name(&client->dev), bq);
		if (ret < 0) {
			dev_err(bq->dev,"request irq for irq=%d failed, ret =%d\n",
				   client->irq, ret);
			goto free_mem;
		}
		//enable_irq_wake(client->irq);
	}

	ret = sc8541_power_supply_init(bq, dev, id->driver_data);
	if (ret)
		goto free_mem;

	ret = sc8541_reg_init(bq);
	if (ret) {
		dev_err(dev, "Cannot initialize the chip.\n");
		goto free_psy;
	}

	sc8541_init_iio_psy(bq);

	sc8541_create_device_node(bq->dev);

	//dump_reg(bq,0x00,0x37);

	printk("-------[%s] driver probe success--------\n",bq->model_name);
	return 0;
free_psy:
	power_supply_unregister(bq->charger);

free_mem:
	devm_kfree(bq->dev, bq);
	return ret;
}
static void sc8541_charger_remove(struct i2c_client *client)
{
	struct sc8541_device *bq = i2c_get_clientdata(client);

	sc8541_set_adc_enable(bq, false);

	power_supply_unregister(bq->charger);

	mutex_destroy(&bq->lock);
	mutex_destroy(&bq->irq_complete);

	dev_err(bq->dev,"remove Successfully\n");
	return;
}


static void sc8541_charger_shutdown(struct i2c_client *client)
{
	struct sc8541_device *bq = i2c_get_clientdata(client);

	if (bq->part_no == NU2115_PART_NO) {
		/*reg reset*/
		regmap_update_bits(bq->regmap, NU2115_CTRL_REG,
			NU2115_REG_RESET, NU2115_REG_RESET);
		sc8541_set_adc_enable(bq, false);

		regmap_write(bq->regmap, NU2115_CTRL_REG, 0);
	} else {
		/*reg reset*/
		regmap_update_bits(bq->regmap, SC8541_CHRGR_CTRL_2,
			SC8541_REG_RESET, SC8541_REG_RESET);
		sc8541_set_adc_enable(bq, false);

		regmap_write(bq->regmap, SC8541_CHRGR_CTRL_2, 0);
	}
	dev_err(bq->dev,"Shutdown Successfully\n");
}

static int sc8541_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sc8541_device *bq = i2c_get_clientdata(client);

	mutex_lock(&bq->irq_complete);
	bq->resume_completed = false;
	mutex_unlock(&bq->irq_complete);
	dev_err(bq->dev, "Suspend successfully!");

	return 0;
 }

static int sc8541_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sc8541_device *bq = i2c_get_clientdata(client);

	if (bq->irq_waiting) {
				pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
		return -EBUSY;
	}
	return 0;
}

static int sc8541_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sc8541_device *bq = i2c_get_clientdata(client);


	mutex_lock(&bq->irq_complete);
	bq->resume_completed = true;
	if (bq->irq_waiting) {
		bq->irq_disabled = false;
		enable_irq(client->irq);
		mutex_unlock(&bq->irq_complete);
		sc8541_irq_handler_thread(client->irq, bq);
	} else {
		mutex_unlock(&bq->irq_complete);
	}

	power_supply_changed(bq->charger);
	dev_err(bq->dev,"Resume successfully!");

	return 0;
}

static const struct dev_pm_ops sc8541_pm_ops = {
	.resume		= sc8541_resume,
	.suspend_noirq = sc8541_suspend_noirq,
	.suspend	= sc8541_suspend,
};

static const struct i2c_device_id sc8541_i2c_ids[] = {
	{ "bq25960-standalone", SC8541_STANDALONE },
	{ "bq25960-master", SC8541_MASTER },
	{ "bq25960-slave", SC8541_SLAVE },
	{},
};
MODULE_DEVICE_TABLE(i2c, sc8541_i2c_ids);

static const struct of_device_id sc8541_of_match[] = {
	{ .compatible = "ti,bq25960-standalone", .data = (void *)SC8541_STANDALONE},
	{ .compatible = "ti,bq25960-master", .data = (void *)SC8541_MASTER},
	{ .compatible = "ti,bq25960-slave", .data = (void *)SC8541_SLAVE},
	{ },
};
MODULE_DEVICE_TABLE(of, sc8541_of_match);

static struct i2c_driver sc8541_driver = {
	.driver = {
		.name = "sc8541-charger",
		.of_match_table = sc8541_of_match,
		.pm	= &sc8541_pm_ops,
	},
	.probe = sc8541_probe,
	.id_table = sc8541_i2c_ids,
	.remove		= sc8541_charger_remove,
	.shutdown	= sc8541_charger_shutdown,
};
module_i2c_driver(sc8541_driver);

MODULE_AUTHOR("Dan Murphy <dmurphy@ti.com>");
MODULE_AUTHOR("Ricardo Rivera-Matos <r-rivera-matos@ti.com>");
MODULE_DESCRIPTION("sc8541 charger driver");
MODULE_LICENSE("GPL v2");
