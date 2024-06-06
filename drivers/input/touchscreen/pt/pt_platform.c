/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FCNT LIMITED 2021
/*----------------------------------------------------------------------------*/
/*
 * pt_platform.c
 * Parade TrueTouch(TM) Standard Product Platform Module.
 * For use with Parade touchscreen controllers.
 * Supported parts include:
 * TMA5XX
 * TMA448
 * TMA445A
 * TT21XXX
 * TT31XXX
 * TT4XXXX
 * TT7XXX
 * TC3XXX
 *
 * Copyright (C) 2015-2020 Parade Technologies
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Contact Parade Technologies at www.paradetech.com <ttdrivers@paradetech.com>
 */

#include "pt_regs.h"
#include <linux/pt_platform.h>

#ifdef CONFIG_TOUCHSCREEN_PARADE_PLATFORM_FW_UPGRADE
/* FW for Panel ID = 0x00 */
#include "pt_fw_pid00.h"
static struct pt_touch_firmware pt_firmware_pid00 = {
	.img = pt_img_pid00,
	.size = ARRAY_SIZE(pt_img_pid00),
	.ver = pt_ver_pid00,
	.vsize = ARRAY_SIZE(pt_ver_pid00),
	.panel_id = 0x00,
};

/* FW for Panel ID = 0x01 */
#include "pt_fw_pid01.h"
static struct pt_touch_firmware pt_firmware_pid01 = {
	.img = pt_img_pid01,
	.size = ARRAY_SIZE(pt_img_pid01),
	.ver = pt_ver_pid01,
	.vsize = ARRAY_SIZE(pt_ver_pid01),
	.panel_id = 0x01,
};

/* FW for Panel ID not enabled (legacy) */
#include "pt_fw.h"
static struct pt_touch_firmware pt_firmware = {
	.img = pt_img,
	.size = ARRAY_SIZE(pt_img),
	.ver = pt_ver,
	.vsize = ARRAY_SIZE(pt_ver),
};
#else
/* FW for Panel ID not enabled (legacy) */
static struct pt_touch_firmware pt_firmware = {
	.img = NULL,
	.size = 0,
	.ver = NULL,
	.vsize = 0,
};
#endif

#ifdef CONFIG_TOUCHSCREEN_PARADE_PLATFORM_TTCONFIG_UPGRADE
/* TT Config for Panel ID = 0x00 */
#include "pt_params_pid00.h"
static struct touch_settings pt_sett_param_regs_pid00 = {
	.data = (uint8_t *)&pt_param_regs_pid00[0],
	.size = ARRAY_SIZE(pt_param_regs_pid00),
	.tag = 0,
};

static struct touch_settings pt_sett_param_size_pid00 = {
	.data = (uint8_t *)&pt_param_size_pid00[0],
	.size = ARRAY_SIZE(pt_param_size_pid00),
	.tag = 0,
};

static struct pt_touch_config pt_ttconfig_pid00 = {
	.param_regs = &pt_sett_param_regs_pid00,
	.param_size = &pt_sett_param_size_pid00,
	.fw_ver = ttconfig_fw_ver_pid00,
	.fw_vsize = ARRAY_SIZE(ttconfig_fw_ver_pid00),
	.panel_id = 0x00,
};

/* TT Config for Panel ID = 0x01 */
#include "pt_params_pid01.h"
static struct touch_settings pt_sett_param_regs_pid01 = {
	.data = (uint8_t *)&pt_param_regs_pid01[0],
	.size = ARRAY_SIZE(pt_param_regs_pid01),
	.tag = 0,
};

static struct touch_settings pt_sett_param_size_pid01 = {
	.data = (uint8_t *)&pt_param_size_pid01[0],
	.size = ARRAY_SIZE(pt_param_size_pid01),
	.tag = 0,
};

static struct pt_touch_config pt_ttconfig_pid01 = {
	.param_regs = &pt_sett_param_regs_pid01,
	.param_size = &pt_sett_param_size_pid01,
	.fw_ver = ttconfig_fw_ver_pid01,
	.fw_vsize = ARRAY_SIZE(ttconfig_fw_ver_pid01),
	.panel_id = 0x01,
};

/* TT Config for Panel ID not enabled (legacy)*/
#include "pt_params.h"
static struct touch_settings pt_sett_param_regs = {
	.data = (uint8_t *)&pt_param_regs[0],
	.size = ARRAY_SIZE(pt_param_regs),
	.tag = 0,
};

static struct touch_settings pt_sett_param_size = {
	.data = (uint8_t *)&pt_param_size[0],
	.size = ARRAY_SIZE(pt_param_size),
	.tag = 0,
};

static struct pt_touch_config pt_ttconfig = {
	.param_regs = &pt_sett_param_regs,
	.param_size = &pt_sett_param_size,
	.fw_ver = ttconfig_fw_ver,
	.fw_vsize = ARRAY_SIZE(ttconfig_fw_ver),
};
#else
/* TT Config for Panel ID not enabled (legacy)*/
static struct pt_touch_config pt_ttconfig = {
	.param_regs = NULL,
	.param_size = NULL,
	.fw_ver = NULL,
	.fw_vsize = 0,
};
#endif

static struct pt_touch_firmware *pt_firmwares[] = {
#ifdef CONFIG_TOUCHSCREEN_PARADE_PLATFORM_FW_UPGRADE
	&pt_firmware_pid00,
	&pt_firmware_pid01,
#endif
	NULL, /* Last item should always be NULL */
};

static struct pt_touch_config *pt_ttconfigs[] = {
#ifdef CONFIG_TOUCHSCREEN_PARADE_PLATFORM_TTCONFIG_UPGRADE
	&pt_ttconfig_pid00,
	&pt_ttconfig_pid01,
#endif
	NULL, /* Last item should always be NULL */
};

struct pt_loader_platform_data _pt_loader_platform_data = {
	.fw = &pt_firmware,
	.ttconfig = &pt_ttconfig,
	.fws = pt_firmwares,
	.ttconfigs = pt_ttconfigs,
	.flags = PT_LOADER_FLAG_CALIBRATE_AFTER_FW_UPGRADE,
};

#define PT_REG_DVDD_NAME "dvdd"
#define PT_REG_AVDD_NAME "avdd"

static int pt_gpio_request(unsigned gpio_num, const char *label)
{
	int rc;

	rc = gpio_request(gpio_num, label);
	if (unlikely(rc < 0))
	{
		gpio_free(gpio_num);
		rc = gpio_request(gpio_num, label);
	}

	return rc;
}

static int pt_pin_init(struct pt_core_platform_data *pdata, struct device *dev)
{
	int rc = 0;
	int rst_gpio = pdata->rst_gpio;
	int irq_gpio = pdata->irq_gpio;
	int ddi_rst_gpio = pdata->ddi_rst_gpio;

	if (rst_gpio)
	{
		rc = pt_gpio_request(rst_gpio, "touch_rst");
		if (rc < 0)
		{
			dev_err(dev, "%s: Error %d - request rst_gpio(%d)\n",
					__func__, rc, rst_gpio);
			return rc;
		}
	}

	if (irq_gpio)
	{
		rc = pt_gpio_request(irq_gpio, "touch_irq");
		if (rc < 0)
		{
			dev_err(dev, "%s: Error %d - request irq_gpio(%d)\n",
					__func__, rc, irq_gpio);
			return rc;
		}
	}

	if (ddi_rst_gpio)
	{
		rc = pt_gpio_request(ddi_rst_gpio, "touch_ddi_rst");
		if (rc < 0)
		{
			dev_err(dev, "%s: Error %d - request ddi_rst_gpio(%d)\n",
					__func__, rc, ddi_rst_gpio);
			return rc;
		}
	}

	return 0;
}

static int pt_pin_free(struct pt_core_platform_data *pdata)
{
	int rst_gpio = pdata->rst_gpio;
	int irq_gpio = pdata->irq_gpio;
	int ddi_rst_gpio = pdata->ddi_rst_gpio;

	if (ddi_rst_gpio)
		gpio_free(ddi_rst_gpio);
	if (irq_gpio)
		gpio_free(irq_gpio);
	if (rst_gpio)
		gpio_free(rst_gpio);

	return 0;
}

static int pt_regulator_get(struct pt_core_platform_data *pdata, struct device *dev)
{
	int rc;

	pdata->dvdd_reg = regulator_get(dev, PT_REG_DVDD_NAME);
	if (IS_ERR(pdata->dvdd_reg))
	{
		rc = PTR_ERR(pdata->dvdd_reg);
		pr_err("%s: DVDD Regurator Get Err\n", __func__);
		return rc;
	}
	if (pdata->dvdd_voltage)
	{
		rc = regulator_set_voltage(pdata->dvdd_reg, pdata->dvdd_voltage, pdata->dvdd_voltage);
		if (rc)
		{
			dev_err(dev, "%s: Error %d - Set DVDD Voltage\n", __func__, rc);
			regulator_put(pdata->dvdd_reg);
			pdata->dvdd_reg = NULL;
			return -EINVAL;
		}
	}
	if (pdata->dvdd_uA)
	{
		rc = regulator_set_load(pdata->dvdd_reg, pdata->dvdd_uA);
		if (rc)
		{
			dev_err(dev, "%s: Error %d - Set DVDD Load\n", __func__, rc);
			regulator_put(pdata->dvdd_reg);
			pdata->dvdd_reg = NULL;
			return -EINVAL;
		}
	}

	pdata->avdd_reg = regulator_get(dev, PT_REG_AVDD_NAME);
	if (IS_ERR(pdata->avdd_reg))
	{
		pr_err("%s: AVDD Regurator Get Err\n", __func__);
		regulator_put(pdata->dvdd_reg);
		pdata->dvdd_reg = NULL;
		return rc;
	}
	if (pdata->avdd_voltage)
	{
		rc = regulator_set_voltage(pdata->avdd_reg, pdata->avdd_voltage, pdata->avdd_voltage);
		if (rc)
		{
			pr_err("%s: Error %d - Set AVDD Voltage\n", __func__, rc);
			regulator_put(pdata->avdd_reg);
			regulator_put(pdata->dvdd_reg);
			pdata->avdd_reg = NULL;
			pdata->dvdd_reg = NULL;
			return -EINVAL;
		}
	}
	if (pdata->avdd_uA)
	{
		rc = regulator_set_load(pdata->avdd_reg, pdata->avdd_uA);
		if (rc)
		{
			pr_err("%s: Error %d - Set AVDD Voltage\n", __func__, rc);
			regulator_put(pdata->avdd_reg);
			regulator_put(pdata->dvdd_reg);
			pdata->avdd_reg = NULL;
			pdata->dvdd_reg = NULL;
			return -EINVAL;
		}
	}

	return 0;
}

static int pt_regulator_free(struct pt_core_platform_data *pdata)
{
	if (pdata->avdd_reg)
	{
		regulator_put(pdata->avdd_reg);
		pdata->avdd_reg = NULL;
	}

	if (pdata->dvdd_reg)
	{
		regulator_put(pdata->dvdd_reg);
		pdata->dvdd_reg = NULL;
	}

	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_xres
 *
 * SUMMARY: Toggles the reset gpio (TP_XRES) to perform a HW reset
 *
 * RETURN:
 *	 0 = success
 *	!0 = fail
 *
 * PARAMETERS:
 *  *pdata - pointer to the platform data structure
 *  *dev   - pointer to Device structure
 ******************************************************************************/
int pt_xres(struct pt_core_platform_data *pdata, struct device *dev)
{
	int rst_gpio = pdata->rst_gpio;
	int rc = 0;
	int ddi_rst_gpio = pdata->ddi_rst_gpio;

	pt_debug(dev, DL_INFO, "%s: 20ms HARD RESET on gpio=%d\n",
			 __func__, pdata->rst_gpio);

	/* Toggling only TP_XRES as DDI_XRES resets the entire part */
	gpio_set_value(rst_gpio, 1);
	if (ddi_rst_gpio)
		gpio_set_value(ddi_rst_gpio, 1);
	usleep_range(3000, 4000);
	gpio_set_value(rst_gpio, 0);
	usleep_range(6000, 7000);
	gpio_set_value(rst_gpio, 1);
	if (ddi_rst_gpio)
		gpio_set_value(ddi_rst_gpio, 1);

	/* Sleep to allow the DUT to boot */
	usleep_range(3000, 4000);
	return rc;
}

#ifdef PT_PINCTRL_EN
/*******************************************************************************
 * FUNCTION: pt_pinctrl_init
 *
 * SUMMARY: Pinctrl method to obtain pin state handler for TP_RST, IRQ
 *
 * RETURN:
 *	 0 = success
 *	!0 = fail
 *
 * PARAMETERS:
 *  *pdata - pointer to the platform data structure
 *  *dev   - pointer to Device structure
 ******************************************************************************/
static int pt_pinctrl_init(struct pt_core_platform_data *pdata,
						   struct device *dev)
{
	int ret = 0;

	pdata->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(pdata->pinctrl))
	{
		pt_debug(dev, DL_ERROR,
				 "Failed to get pinctrl, please check dts");
		ret = PTR_ERR(pdata->pinctrl);
		goto err_pinctrl_get;
	}

	pdata->pins_active =
		pinctrl_lookup_state(pdata->pinctrl, "pmx_ts_active");
	if (IS_ERR_OR_NULL(pdata->pins_active))
	{
		pt_debug(dev, DL_ERROR, "pmx_ts_active not found");
		ret = PTR_ERR(pdata->pins_active);
		goto err_pinctrl_lookup;
	}

	pdata->pins_suspend =
		pinctrl_lookup_state(pdata->pinctrl, "pmx_ts_suspend");
	if (IS_ERR_OR_NULL(pdata->pins_suspend))
	{
		pt_debug(dev, DL_ERROR, "pmx_ts_suspend not found");
		ret = PTR_ERR(pdata->pins_suspend);
		goto err_pinctrl_lookup;
	}

	pdata->pins_release =
		pinctrl_lookup_state(pdata->pinctrl, "pmx_ts_release");
	if (IS_ERR_OR_NULL(pdata->pins_release))
	{
		pt_debug(dev, DL_ERROR, "pmx_ts_release not found");
		ret = PTR_ERR(pdata->pins_release);
		goto err_pinctrl_lookup;
	}

	return 0;

err_pinctrl_lookup:
	devm_pinctrl_put(pdata->pinctrl);

err_pinctrl_get:
	pdata->pinctrl = NULL;
	pdata->pins_release = NULL;
	pdata->pins_suspend = NULL;
	pdata->pins_active = NULL;
	return ret;
}

/*******************************************************************************
 * FUNCTION: pt_pinctrl_select_normal
 *
 * SUMMARY: Pinctrl method to configure drive mode for TP_RST, IRQ - normal
 *
 * RETURN:
 *	 0 = success
 *	!0 = fail
 *
 * PARAMETERS:
 *  *pdata - pointer to the platform data structure
 *  *dev   - pointer to Device structure
 ******************************************************************************/
static int pt_pinctrl_select_normal(struct pt_core_platform_data *pdata,
									struct device *dev)
{
	int ret = 0;

	if (pdata->pinctrl && pdata->pins_active)
	{
		ret = pinctrl_select_state(pdata->pinctrl, pdata->pins_active);
		if (ret < 0)
		{
			pt_debug(dev, DL_ERROR, "Set normal pin state error=%d",
					 ret);
		}
	}

	return ret;
}

/*******************************************************************************
 * FUNCTION: pt_pinctrl_select_suspend
 *
 * SUMMARY:  Pinctrl method to configure drive mode for TP_RST, IRQ - suspend
 *
 * RETURN:
 *	 0 = success
 *	!0 = fail
 *
 * PARAMETERS:
 *  *pdata - pointer to the platform data structure
 *  *dev   - pointer to Device structure
 ******************************************************************************/
static int pt_pinctrl_select_suspend(struct pt_core_platform_data *pdata,
									 struct device *dev)
{
	int ret = 0;

	if (pdata->pinctrl && pdata->pins_suspend)
	{
		ret = pinctrl_select_state(pdata->pinctrl, pdata->pins_suspend);
		if (ret < 0)
		{
			pt_debug(dev, DL_ERROR,
					 "Set suspend pin state error=%d", ret);
		}
	}

	return ret;
}

/*******************************************************************************
 * FUNCTION: pt_pinctrl_select_release
 *
 * SUMMARY:  Pinctrl method to configure drive mode for TP_RST, IRQ - release
 *
 * RETURN:
 *	 0 = success
 *	!0 = fail
 *
 * PARAMETERS:
 *  *pdata - pointer to the platform data structure
 *  *dev   - pointer to Device structure
 ******************************************************************************/
static int pt_pinctrl_select_release(struct pt_core_platform_data *pdata,
									 struct device *dev)
{
	int ret = 0;

	if (pdata->pinctrl)
	{
		if (IS_ERR_OR_NULL(pdata->pins_release))
		{
			devm_pinctrl_put(pdata->pinctrl);
			pdata->pinctrl = NULL;
		}
		else
		{
			ret = pinctrl_select_state(pdata->pinctrl,
									   pdata->pins_release);
			if (ret < 0)
				pt_debug(dev, DL_ERROR,
						 "Set gesture pin state error=%d", ret);
		}
	}

	return ret;
}
#endif /* PT_PINCTRL_EN */

/*******************************************************************************
 * FUNCTION: pt_init
 *
 * SUMMARY: Set up/free gpios for TP_RST, IRQ, DDI_RST.
 *
 * RETURN:
 *	 0 = success
 *	!0 = fail
 *
 * PARAMETERS:
 *  *pdata - pointer to the platform data structure
 *   on    - flag to set up or free gpios(0:free; !0:set up)
 *  *dev   - pointer to Device structure
 ******************************************************************************/
int pt_init(struct pt_core_platform_data *pdata,int on, struct device *dev)
{
	int rc = 0;

#ifdef PT_PINCTRL_EN
	if (on)
	{
		rc = pt_pinctrl_init(pdata, dev);
		if (!rc)
		{
			pt_pinctrl_select_normal(pdata, dev);
		}
		else
		{
			pt_debug(dev, DL_ERROR,
					 "%s: Failed to request pinctrl\n", __func__);
		}
	}
#endif
	if (on)
	{
		rc = pt_pin_init(pdata, dev);
		if (rc)
		{
			return rc;
		}
		rc = pt_regulator_get(pdata, dev);
		if (rc)
		{
			pt_pin_free(pdata);
			return rc;
		}
	}
	else
	{
		pt_pin_free(pdata);
		pt_regulator_free(pdata);
	}

	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_wakeup
 *
 * SUMMARY: Resume power for "power on/off" sleep strategy which against to
 *  "deepsleep" strategy.
 *
 * RETURN:
 *	 0 = success
 *	!0 = fail
 *
 * PARAMETERS:
 *  *pdata       - pointer to the platform data structure
 *  *dev         - pointer to Device structure
 *  *ignore_irq  - pointer to atomic structure to allow the host ignoring false
 *                 IRQ during power up
 ******************************************************************************/
static int pt_wakeup(struct pt_core_platform_data *pdata,struct device *dev, atomic_t *ignore_irq)
{
	/* Example for TT7XXX */
	int rc = 0;

#ifdef PT_PINCTRL_EN
	pt_pinctrl_select_normal(pdata, dev);
#endif

#ifdef TT7XXX_EXAMPLE
	pt_debug(dev, DL_INFO,
			 "%s: Enable defined pwr: VDDI, VCC\n", __func__);
	/*
	 * Force part into RESET by holding XRES#(TP_XRES)
	 * while powering it up
	 */
	if (pdata->rst_gpio)
		gpio_set_value(pdata->rst_gpio, 0);

	/* Turn on VDDI [Digital Interface] (+1.8v) */
	if (pdata->vddi_gpio)
	{
		rc = gpio_request(pdata->vddi_gpio, NULL);
		if (rc < 0)
		{
			gpio_free(pdata->vddi_gpio);
			rc = gpio_request(pdata->vddi_gpio, NULL);
		}
		if (rc < 0)
		{
			pr_err("%s: Failed requesting VDDI GPIO %d\n",
				   __func__, pdata->vddi_gpio);
		}
		rc = gpio_direction_output(pdata->vddi_gpio, 1);
		if (rc)
			pr_err("%s: setcfg for VDDI GPIO %d failed\n",
				   __func__, pdata->vddi_gpio);
		gpio_free(pdata->vddi_gpio);
		usleep_range(3000, 4000);
	}

	/* Turn on VCC */
	if (pdata->vcc_gpio)
	{
		rc = gpio_request(pdata->vcc_gpio, NULL);
		if (rc < 0)
		{
			gpio_free(pdata->vcc_gpio);
			rc = gpio_request(pdata->vcc_gpio, NULL);
		}
		if (rc < 0)
		{
			pr_err("%s: Failed requesting VCC GPIO %d\n",
				   __func__, pdata->vcc_gpio);
		}
		rc = gpio_direction_output(pdata->vcc_gpio, 1);
		if (rc)
			pr_err("%s: setcfg for VCC GPIO %d failed\n",
				   __func__, pdata->vcc_gpio);
		gpio_free(pdata->vcc_gpio);
		usleep_range(3000, 4000);
	}

	usleep_range(12000, 15000);
	/* Force part out of RESET by releasing XRES#(TP_XRES) */
	if (pdata->rst_gpio)
		gpio_set_value(pdata->rst_gpio, 1);
#else
	pt_debug(dev, DL_INFO, "%s: Enable defined pwr\n", __func__);
#endif
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_sleep
 *
 * SUMMARY: Suspend power for "power on/off" sleep strategy which against to
 *  "deepsleep" strategy.
 *
 * RETURN:
 *	 0 = success
 *	!0 = fail
 *
 * PARAMETERS:
 *  *pdata       - pointer to the platform data structure
 *  *dev         - pointer to Device structure
 *  *ignore_irq  - pointer to atomic structure to allow the host ignoring false
 *                 IRQ during power down
 ******************************************************************************/
static int pt_sleep(struct pt_core_platform_data *pdata,struct device *dev, atomic_t *ignore_irq)
{
	/* Example for TT7XXX */
	int rc = 0;

#ifdef TT7XXX_EXAMPLE
	pt_debug(dev, DL_INFO,
			 "%s: Turn off defined pwr: VCC, VDDI\n", __func__);
	/*
	 * Force part into RESET by holding XRES#(TP_XRES)
	 * while powering it up
	 */
	if (pdata->rst_gpio)
		gpio_set_value(pdata->rst_gpio, 0);

	/* Turn off VCC */
	if (pdata->vcc_gpio)
	{
		rc = gpio_request(pdata->vcc_gpio, NULL);
		if (rc < 0)
		{
			gpio_free(pdata->vcc_gpio);
			rc = gpio_request(pdata->vcc_gpio, NULL);
		}
		if (rc < 0)
		{
			pr_err("%s: Failed requesting VCC GPIO %d\n",
				   __func__, pdata->vcc_gpio);
		}
		rc = gpio_direction_output(pdata->vcc_gpio, 0);
		if (rc)
			pr_err("%s: setcfg for VCC GPIO %d failed\n",
				   __func__, pdata->vcc_gpio);
		gpio_free(pdata->vcc_gpio);
	}

	/* Turn off VDDI [Digital Interface] (+1.8v) */
	if (pdata->vddi_gpio)
	{
		rc = gpio_request(pdata->vddi_gpio, NULL);
		if (rc < 0)
		{
			gpio_free(pdata->vddi_gpio);
			rc = gpio_request(pdata->vddi_gpio, NULL);
		}
		if (rc < 0)
		{
			pr_err("%s: Failed requesting VDDI GPIO %d\n",
				   __func__, pdata->vddi_gpio);
		}
		rc = gpio_direction_output(pdata->vddi_gpio, 0);
		if (rc)
			pr_err("%s: setcfg for VDDI GPIO %d failed\n",
				   __func__, pdata->vddi_gpio);
		gpio_free(pdata->vddi_gpio);
		usleep_range(10000, 12000);
	}
#else
	pt_debug(dev, DL_INFO, "%s: Turn off defined pwr\n", __func__);
#endif
#ifdef PT_PINCTRL_EN
	pt_pinctrl_select_suspend(pdata, dev);
#endif
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_power
 *
 * SUMMARY: Wrapper function to resume/suspend power with function
 *  pt_wakeup()/pt_sleep().
 *
 * RETURN:
 *	 0 = success
 *	!0 = fail
 *
 * PARAMETERS:
 *  *pdata       - pointer to the platform data structure
 *   on          - flag to remsume/suspend power(0:resume; 1:suspend)
 *  *dev         - pointer to Device structure
 *  *ignore_irq  - pointer to atomic structure to allow the host ignoring false
 *                 IRQ during power up/down
 ******************************************************************************/
int pt_power(struct pt_core_platform_data *pdata,int on, struct device *dev, atomic_t *ignore_irq)
{
	if (on)
		return pt_wakeup(pdata, dev, ignore_irq);

	return pt_sleep(pdata, dev, ignore_irq);
}

/*******************************************************************************
 * FUNCTION: pt_irq_stat
 *
 * SUMMARY: Obtain the level state of IRQ gpio.
 *
 * RETURN:
 *	 level state of IRQ gpio
 *
 * PARAMETERS:
 *  *pdata       - pointer to the platform data structure
 *  *dev         - pointer to Device structure
 ******************************************************************************/
int pt_irq_stat(struct pt_core_platform_data *pdata,struct device *dev)
{
	return gpio_get_value(pdata->irq_gpio);
}

#ifdef PT_DETECT_HW
/*******************************************************************************
 * FUNCTION: pt_detect
 *
 * SUMMARY: Detect the I2C device by reading one byte(FW sentiel) after the
 *  reset operation.
 *
 * RETURN:
 *	 0 - detected
 *  !0 - undetected
 *
 * PARAMETERS:
 *  *pdata - pointer to the platform data structure
 *  *dev   - pointer to Device structure
 *   read  - pointer to the function to perform a read operation
 ******************************************************************************/
int pt_detect(struct pt_core_platform_data *pdata,
			  struct device *dev, pt_platform_read read)
{
	int retry = 3;
	int rc;
	char buf[1];

	while (retry--)
	{
		/* Perform reset, wait for 100 ms and perform read */
		pt_debug(dev, DL_WARN, "%s: Performing a reset\n",
				 __func__);
		pdata->xres(pdata, dev);
		msleep(100);
		rc = read(dev, buf, 1);
		if (!rc)
			return 0;

		pt_debug(dev, DL_ERROR, "%s: Read unsuccessful, try=%d\n",
				 __func__, 3 - retry);
	}

	return rc;
}
#endif

/*******************************************************************************
 * FUNCTION: pt_setup_power
 *
 * SUMMARY: Turn on/turn off voltage regulator
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*pdata - pointer to  core platform data
 *	on     - flag to decide power state,PT_MT_POWER_ON/PT_MT_POWER_OFF
 *	*dev   - pointer to device
 ******************************************************************************/
int pt_setup_power(struct pt_core_platform_data *pdata, int on,struct device *dev)
{
	int rc = 0;

	if (on == PT_MT_POWER_ON)
	{
		/* Reset Assert */
		if (pdata->rst_gpio)
		{
			rc = gpio_direction_output(pdata->rst_gpio, 0);
			if (rc)
			{
				pr_err("%s: Error %d - Change rst_gpio Output Low\n", __func__, rc);
				return rc;
			}
		}
		/* DVDD ON */
		if (pdata->dvdd_reg)
		{
			rc = regulator_enable(pdata->dvdd_reg);
			if (rc)
			{
				pr_err("%s: Error %d - DVDD ON\n", __func__, rc);
				return rc;
			}
		}
		/* AVDD ON */
		if (pdata->avdd_reg)
		{
			rc = regulator_enable(pdata->avdd_reg);
			if (rc)
			{
				pr_err("%s: Error %d - AVDD ON\n", __func__, rc);
				return rc;
			}
		}
		/* 12ms Wait */
		usleep_range(12000, 13000);
		/* IRQ no-pull Setting */
		if (pdata->irq_gpio)
		{
			gpio_direction_input(pdata->irq_gpio);
			if (rc)
			{
				pr_err("%s: Error %d - Change irq_gpio Input\n", __func__, rc);
				return rc;
			}
		}
		/* Reset Deassert */
		if (pdata->rst_gpio)
		{
			gpio_direction_output(pdata->rst_gpio, 1);
			if (rc)
			{
				pr_err("%s: Error %d - Change rst_gpio Output High\n", __func__, rc);
				return rc;
			}
		}
		/* 1.1s Wait */
		usleep_range(1100000, 1100000);
	}
	else
	{
		/* Reset Assert */
		if (pdata->rst_gpio)
		{
			rc = gpio_direction_output(pdata->rst_gpio, 0);
			if (rc)
			{
				pr_err("%s: Error %d - Change rst_gpio Output Low\n", __func__, rc);
				return rc;
			}
		}
		/* IRQ Low Setting */
		if (pdata->irq_gpio)
		{
			gpio_direction_output(pdata->irq_gpio, 0);
			if (rc)
			{
				pr_err("%s: Error %d - Change irq_gpio Output Low\n", __func__, rc);
				return rc;
			}
		}
		/* AVDD OFF */
		if (pdata->avdd_reg)
		{
			rc = regulator_disable(pdata->avdd_reg);
			if (rc)
			{
				pr_err("%s: Error %d - AVDD ON\n", __func__, rc);
				return rc;
			}
		}
		/* DVDD OFF */
		if (pdata->dvdd_reg)
		{
			rc = regulator_disable(pdata->dvdd_reg);
			if (rc)
			{
				pr_err("%s: Error %d - DVDD ON\n", __func__, rc);
				return rc;
			}
		}
	}

	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_setup_irq
 *
 * SUMMARY: Configure the IRQ GPIO used by the TT DUT
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*pdata - pointer to core platform data
 *	 on    - flag to setup interrupt process work(PT_MT_IRQ_FREE/)
 *	*dev   - pointer to device
 ******************************************************************************/
int pt_setup_irq(struct pt_core_platform_data *pdata, int on,struct device *dev)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	unsigned long irq_flags;
	int rc = 0;

	if (on == PT_MT_IRQ_REG)
	{
		/*
		 * When TTDL has direct access to the GPIO the irq_stat function
		 * will be defined and the gpio_to_irq conversion must be
		 * performed. e.g. For CHROMEOS this is not the case, the irq is
		 * passed in directly.
		 */
		if (pdata->irq_stat)
		{
			/* Initialize IRQ */
			dev_vdbg(dev, "%s: Value Passed to gpio_to_irq =%d\n",
					 __func__, pdata->irq_gpio);
			cd->irq = gpio_to_irq(pdata->irq_gpio);
			dev_vdbg(dev,
					 "%s: Value Returned from gpio_to_irq =%d\n",
					 __func__, cd->irq);
		}
		if (cd->irq < 0)
			return -EINVAL;

		cd->irq_enabled = true;

		pt_debug(dev, DL_INFO, "%s: initialize threaded irq=%d\n",
				 __func__, cd->irq);

		if (pdata->level_irq_udelay > 0)
			/* use level triggered interrupts */
			irq_flags = IRQF_TRIGGER_LOW | IRQF_ONESHOT;
		else
			/* use edge triggered interrupts */
			irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;

		rc = request_threaded_irq(cd->irq, NULL, pt_irq,
								  irq_flags, dev_name(dev), cd);
		if (rc < 0)
			pt_debug(dev, DL_ERROR,
					 "%s: Error, could not request irq\n", __func__);
	}
	else
	{
		disable_irq_nosync(cd->irq);
		free_irq(cd->irq, cd);
	}
	return rc;
}
