/*
 * Copyright (C) 2024 Motorola Mobility LLC
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#define DRIVER_VERSION "0.0.1"
#define WLAN_ELNA_GPIO_NAME     "moto,wlan_elna-gpio"
#define WLAN_ELNA_GPIO_LABEL    "wlan_elna_gpio"

struct wlan_elna_drvdata {
	struct device	*dev;
	struct pinctrl *pinctrl;
	int wlan_elna_gpio;
	struct pinctrl_state *pstate_active;
	struct pinctrl_state *pstate_suspend;
    bool is_5g;
};

static ssize_t wlan_elna_en_read(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct wlan_elna_drvdata *data = dev_get_drvdata(dev);

	if (!data) {
		pr_err("wlan_elna drvdata is NULL\n");
		return -EINVAL;
	}

	if (gpio_get_value(data->wlan_elna_gpio))
		return snprintf(buf, PAGE_SIZE, "high\n");
	else
		return snprintf(buf, PAGE_SIZE, "low\n");
}

static ssize_t wlan_elna_en_write(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf,
				 size_t count)
{
	struct wlan_elna_drvdata *data = dev_get_drvdata(dev);
	int ret = 0;
	unsigned int res = 0;

	if (!data) {
		pr_err("wlan_elna drvdata is NULL\n");
		return -EINVAL;
	}

	ret = kstrtouint(buf, 0, &res);
	if(ret) {
		pr_err("wlan_elna failed to get data, set as default!\n");
	}

	if(1 == res) {
		ret = pinctrl_select_state(data->pinctrl, data->pstate_active);
        data->is_5g = true;
	} else {
		ret = pinctrl_select_state(data->pinctrl, data->pstate_suspend);
        data->is_5g = false;
	}

	if(ret) {
		pr_err("wlan_elna failed to set pinctrl!\n");
	}

	return count;
}

static DEVICE_ATTR(wlan_elna_en, 0664, wlan_elna_en_read, wlan_elna_en_write);

static struct attribute *wlan_elna_sysfs_attrs[] = {
    &dev_attr_wlan_elna_en.attr,
	NULL,
};

static struct attribute_group wlan_elna_sysfs_attr_grp = {
	.attrs = wlan_elna_sysfs_attrs,
};

static int wlan_elna_probe(struct platform_device *pdev)
{
	struct wlan_elna_drvdata *drvdata;
	int ret = 0;
	struct device *dev = &pdev->dev;

	drvdata = devm_kzalloc(dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	drvdata->dev = &pdev->dev;
	platform_set_drvdata(pdev, drvdata);

	drvdata->wlan_elna_gpio = of_get_named_gpio(dev->of_node, WLAN_ELNA_GPIO_NAME, 0);
    if (!gpio_is_valid(drvdata->wlan_elna_gpio))
    {
		pr_err("%s, Get gpio %s failed\n", __func__, WLAN_ELNA_GPIO_NAME);
        return -ENODEV;
    }

	ret = gpio_request(drvdata->wlan_elna_gpio, WLAN_ELNA_GPIO_LABEL);
	if (ret) {
		pr_err("%s, unable to request gpio %d (%d)\n", __func__, drvdata->wlan_elna_gpio, ret);
		return ret;
	}

	/* Get pinctrl if target uses pinctrl */
	drvdata->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(drvdata->pinctrl)) {
		ret = PTR_ERR(drvdata->pinctrl);
		pr_err("%s: Pincontrol DT property returned %X\n", __func__, ret);
		return ret;
	}

	drvdata->pstate_active = pinctrl_lookup_state(drvdata->pinctrl, "elna_active");
	if (IS_ERR_OR_NULL(drvdata->pstate_active)) {
		ret = PTR_ERR(drvdata->pstate_active);
		pr_err("Can not lookup elna_active pinstate %d\n", ret);
		return ret;
	}

	drvdata->pstate_suspend = pinctrl_lookup_state(drvdata->pinctrl, "elna_suspend");
	if (IS_ERR_OR_NULL(drvdata->pstate_suspend)) {
		ret = PTR_ERR(drvdata->pstate_suspend);
		pr_err("Can not lookup elna_suspend pinstate %d\n", ret);
		return ret;
	}

	ret = pinctrl_select_state(drvdata->pinctrl, drvdata->pstate_suspend);
	if(ret) {
		pr_err("wlan_elna failed to set pinctrl pstate_suspend!\n");
	} else {
		pr_info("wlan_elna enable pstate_suspend\n");
	}

	ret = sysfs_create_group(&dev->kobj, &wlan_elna_sysfs_attr_grp);
	if (ret) {
		pr_err("%s: sysfs group creation failed %d\n", __func__, ret);
		return ret;
	}

	dev_info(&pdev->dev, "probe: All success !\n");

	return ret;
}

static int wlan_elna_remove(struct platform_device *pdev)
{
    struct wlan_elna_drvdata* p_drvdata;

    p_drvdata = pdev->dev.driver_data;
    if (gpio_is_valid(drvdata->wlan_elna_gpio))
    {
        gpio_free(p_drvdata->wlan_elna_gpio);
    }

	return 0;
}

static int wlan_elna_pm_suspend(struct device *dev)
{
	int ret = 0;
	struct wlan_elna_drvdata *drvdata = dev_get_drvdata(dev);

	if (!drvdata) {
		pr_err("wlan_elna drvdata is NULL\n");
		return -EINVAL;
	}
	pr_info("wlan_elna suspend\n");

    if (drvdata->is_5g) {
	    ret = pinctrl_select_state(drvdata->pinctrl, drvdata->pstate_suspend);
	    if(ret) {
	    	pr_err("wlan_elna failed to set pinctrl pstate_suspend!\n");
	    } else {
	    	pr_info("wlan_elna enable pstate_suspend\n");
	    }
    }

	return ret;
}

static int wlan_elna_pm_resume(struct device *dev)
{
	int ret = 0;
	struct wlan_elna_drvdata *drvdata = dev_get_drvdata(dev);

	if (!drvdata) {
		pr_err("wlan_elna drvdata is NULL\n");
		return -EINVAL;
	}
	pr_info("wlan_elna resume\n");

    if (drvdata->is_5g) {
	    ret = pinctrl_select_state(drvdata->pinctrl, drvdata->pstate_active);
	    if(ret) {
	    	pr_err("wlan_elna failed to set pinctrl pstate_active\n");
	    } else {
	    	pr_info("wlan_elna enable pstate_active\n");
	    }
    }

	return ret;
}

static const struct of_device_id wlan_elna_match[] = {
	{ .compatible = "moto,wlan_elna" },
	{}
};

static const struct dev_pm_ops wlan_elna_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(wlan_elna_pm_suspend,
				wlan_elna_pm_resume)
};

static struct platform_driver wlan_elna_plat_driver = {
	.probe = wlan_elna_probe,
	.remove = wlan_elna_remove,
	.driver = {
		.name = "wlan_elna",
		.owner = THIS_MODULE,
		.of_match_table = wlan_elna_match,
        .pm = &wlan_elna_pm_ops,
	},
};

static int wlan_elna_init(void)
{
	return platform_driver_register(&wlan_elna_plat_driver);
}

static void wlan_elna_exit(void)
{
	platform_driver_unregister(&wlan_elna_plat_driver);
}

module_init(wlan_elna_init);
module_exit(wlan_elna_exit);

MODULE_AUTHOR("Motorola Mobiity");
MODULE_DESCRIPTION("wlan elna control interface driver");
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");
