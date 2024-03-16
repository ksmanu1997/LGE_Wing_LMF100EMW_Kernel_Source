/*
 * SLS USB1.1 Host Controller Core
 *
 * Copyright (C) 2019 LG Electronics, Inc.
 * Author: Hansun Lee <hansun.lee@lge.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/fpga/fpga-mgr.h>
#include <linux/spi/spi.h>
#include <linux/firmware.h>
#include <linux/clk.h>
#include <linux/extcon.h>

#define VDDA_3P3_VOL_MIN			3150000 /* uV */
#define VDDA_3P3_VOL_MAX			3300000 /* uV */
#define VDDA_3P3_HPM_LOAD			16000   /* uA */

#define VDDA_1P8_VOL_MIN			1704000 /* uV */
#define VDDA_1P8_VOL_MAX			1800000 /* uV */
#define VDDA_1P8_HPM_LOAD			19000   /* uA */

static unsigned int fpga_mgr_max_speed_hz;
module_param(fpga_mgr_max_speed_hz, uint, 0644);

static unsigned int max_speed_hz;
module_param(max_speed_hz, uint, 0644);

static unsigned int extcon_idx = 0;
module_param(extcon_idx, uint, 0644);

struct sls11hc;

struct extcon_nb {
	struct extcon_dev		*edev;
	struct sls11hc			*hc;
	int				idx;
	struct notifier_block		host_nb;
};

struct sls11hc {
	struct device			*dev;

	struct clk			*core_clk;

	struct regulator		*vdda18;
	struct regulator		*vdda33;

	struct gpio_desc		*ldo1_en;
	struct gpio_desc		*ldo2_en;
	struct gpio_desc		*reset;
	struct gpio_desc		*ds_sw_sel;

	bool				power_enabled;

	struct fpga_manager		*mgr;
	struct fpga_image_info		*info;

	bool				host_state;
	struct extcon_nb		*extcon;
	int				ext_idx;
	struct delayed_work		sm_work;

	struct platform_device		*hcd;
};

static int sls11hc_enable_power(struct sls11hc *hc, bool on)
{
	int ret = 0;

	if (hc->power_enabled == on) {
		dev_dbg(hc->dev, "already %s\n", on ? "ON" : "OFF");
		return 0;
	}

	if (!on)
		goto disable_vdda33;

	if (hc->core_clk) {
		ret = clk_prepare_enable(hc->core_clk);
		if (ret) {
			dev_err(hc->dev, "unable to enable core_clk:%d\n", ret);
			goto err_vdd;
		}
	}

	if (hc->vdda18) {
		ret = regulator_set_load(hc->vdda18, VDDA_1P8_HPM_LOAD);
		if (ret < 0) {
			dev_err(hc->dev,
				"unable to set HPM of vdda18:%d\n", ret);
			goto disable_clk;
		}

		ret = regulator_set_voltage(hc->vdda18, VDDA_1P8_VOL_MIN,
					    VDDA_1P8_VOL_MAX);
		if (ret) {
			dev_err(hc->dev,
				"unable to set voltage for vdda18:%d\n", ret);
			goto put_vdda18_lpm;
		}

		ret = regulator_enable(hc->vdda18);
		if (ret) {
			dev_err(hc->dev, "unable to enable vdda18:%d\n", ret);
			goto unset_vdda18;
		}
	}

	if (hc->ldo1_en)
		gpiod_set_value(hc->ldo1_en, 1);

	if (hc->vdda33) {
		ret = regulator_set_load(hc->vdda33, VDDA_3P3_HPM_LOAD);
		if (ret < 0) {
			dev_err(hc->dev,
				"unable to set HPM of vdda33:%d\n", ret);
			goto disable_vdda18;
		}

		ret = regulator_set_voltage(hc->vdda33, VDDA_3P3_VOL_MIN,
					    VDDA_3P3_VOL_MAX);
		if (ret) {
			dev_err(hc->dev,
				"unable to set voltage for vdda33:%d\n", ret);
			goto put_vdda33_lpm;
		}

		ret = regulator_enable(hc->vdda33);
		if (ret) {
			dev_err(hc->dev, "unable to enable vdda33:%d\n", ret);
			goto unset_vdda33;
		}
	}

	if (hc->ldo2_en)
		gpiod_set_value(hc->ldo2_en, 1);

	hc->power_enabled = true;
	dev_dbg(hc->dev, "regulators are turned ON\n");
	return ret;

disable_vdda33:
	if (hc->ldo2_en)
		gpiod_set_value(hc->ldo2_en, 0);

	if (hc->vdda33) {
		ret = regulator_disable(hc->vdda33);
		if (ret)
			dev_err(hc->dev, "unable to disable vdd33:%d\n", ret);
	}

unset_vdda33:
	if (hc->vdda33) {
		ret = regulator_set_voltage(hc->vdda33, 0, VDDA_3P3_VOL_MAX);
		if (ret)
			dev_err(hc->dev,
				"unable to set (0) voltage for vdda33:%d\n",
				ret);
	}

put_vdda33_lpm:
	if (hc->vdda33) {
		ret = regulator_set_load(hc->vdda33, 0);
		if (ret < 0)
			dev_err(hc->dev,
				"Unable to set (0) HPM of vdda33:%d\n", ret);
	}

disable_vdda18:
	if (hc->ldo1_en)
		gpiod_set_value(hc->ldo1_en, 0);

	if (hc->vdda18) {
		ret = regulator_disable(hc->vdda18);
		if (ret)
			dev_err(hc->dev, "unable to disable vdd18:%d\n", ret);
	}

unset_vdda18:
	if (hc->vdda18) {
		ret = regulator_set_voltage(hc->vdda18, 0, VDDA_1P8_VOL_MAX);
		if (ret)
			dev_err(hc->dev,
				"unable to set (0) voltage for vdda18:%d\n",
				ret);
	}

put_vdda18_lpm:
	if (hc->vdda18) {
		ret = regulator_set_load(hc->vdda18, 0);
		if (ret < 0)
			dev_err(hc->dev,
				"Unable to set (0) HPM of vdda18:%d\n", ret);
	}

disable_clk:
	if (hc->core_clk)
		clk_disable_unprepare(hc->core_clk);

err_vdd:
	hc->power_enabled = false;
	dev_dbg(hc->dev, "regulators are turned OFF\n");
	return ret;
}

static int sls11hc_fpga_mgr_load(struct sls11hc *hc)
{
	struct fpga_manager *mgr = hc->mgr;
	struct spi_device *spi = to_spi_device(mgr->dev.parent);
	int ret;

	spi->mode = SPI_MODE_0;
	spi->max_speed_hz = fpga_mgr_max_speed_hz;

	ret = spi_setup(spi);
	if (ret < 0) {
		dev_err(hc->dev, "unable to setup fpga_mgr SPI bus %d\n", ret);
		return ret;
	}

	ret = fpga_mgr_lock(mgr);
	if (ret) {
		dev_err(hc->dev, "FPGA manager is busy\n");
		return ret;
	}

	gpiod_set_value(hc->reset, 1);

	ret = fpga_mgr_load(mgr, hc->info);
	if (ret)
		dev_err(hc->dev, "failed to load fpga manager %d\n", ret);

	gpiod_set_value(hc->reset, 0);

	fpga_mgr_unlock(mgr);

	return ret;
}

static int sls11hc_host_init(struct sls11hc *hc)
{
	struct spi_device *spi = to_spi_device(hc->mgr->dev.parent);
	struct platform_device *hcd;
	int ret;

	spi->mode = SPI_MODE_0;
	spi->max_speed_hz = max_speed_hz;

	ret = spi_setup(spi);
	if (ret < 0) {
		dev_err(hc->dev, "unable to setup sls11hc SPI bus :%d\n", ret);
		return ret;
	}

	hcd = platform_device_alloc("sls-hcd", PLATFORM_DEVID_AUTO);
	if (!hcd) {
		dev_err(hc->dev, "couldn't allocate sls-hcd device\n");
		return -ENOMEM;
	}

	hcd->dev.parent = &spi->dev;
	hc->hcd = hcd;

	ret = platform_device_add(hcd);
	if (ret) {
		dev_err(hc->dev, "failed to register sls-hcd device\n");
		goto err;
	}

	return 0;

err:
	platform_device_put(hcd);
	hc->hcd = NULL;
	return ret;
}

static void sls11hc_host_exit(struct sls11hc *hc)
{
	if (hc->hcd) {
		platform_device_unregister(hc->hcd);
		hc->hcd = NULL;
	}
}

static int sls11hc_start_host(struct sls11hc *hc, int on)
{
	int ret;

	if (on) {
		dev_info(hc->dev, "%s: turn on host\n", __func__);

		ret = sls11hc_enable_power(hc, true);
		if (ret)
			return ret;

		ret = sls11hc_fpga_mgr_load(hc);
		if (ret)
			return ret;

		if (hc->ds_sw_sel)
			gpiod_set_value(hc->ds_sw_sel, 1);

		ret = sls11hc_host_init(hc);
		if (ret)
			return ret;
	} else {
		dev_info(hc->dev, "%s: turn off host\n", __func__);

		if (hc->ds_sw_sel)
			gpiod_set_value(hc->ds_sw_sel, 0);

		sls11hc_host_exit(hc);

		sls11hc_enable_power(hc, false);
	}

	return 0;
}

static int sls11hc_host_notifier(struct notifier_block *nb,
		unsigned long event, void *ptr)
{
	struct extcon_dev *edev = ptr;
	struct extcon_nb *enb = container_of(nb, struct extcon_nb, host_nb);
	struct sls11hc *hc = enb->hc;

	if (!edev || !hc)
		return NOTIFY_DONE;

	if (extcon_idx != enb->idx)
		return NOTIFY_DONE;

	if (hc->host_state == event)
		return NOTIFY_DONE;

	/* Flush processing any pending events before handling new ones */
	flush_delayed_work(&hc->sm_work);

	dev_dbg(hc->dev, "host:%ld event received\n", event);

	hc->ext_idx = enb->idx;
	hc->host_state = event;

	schedule_delayed_work(&hc->sm_work, 0);

	return NOTIFY_DONE;
}

static void sls11hc_sm_work(struct work_struct *w)
{
	struct sls11hc *hc = container_of(w, struct sls11hc, sm_work.work);
	sls11hc_start_host(hc, hc->host_state);
}

static int sls11hc_extcon_register(struct sls11hc *hc)
{
	struct device_node *node = hc->dev->of_node;
	int idx, extcon_cnt;
	struct extcon_dev *edev;
	bool phandle_found = false;
	int ret;

	extcon_cnt = of_count_phandle_with_args(node, "extcon", NULL);
	if (extcon_cnt < 0) {
		dev_err(hc->dev, "of_count_phandle_with_args failed\n");
		return -ENODEV;
	}

	hc->extcon = devm_kcalloc(hc->dev, extcon_cnt,
				  sizeof(*hc->extcon), GFP_KERNEL);
	if (!hc->extcon)
		return -ENOMEM;

	for (idx = 0; idx < extcon_cnt; idx++) {
		edev = extcon_get_edev_by_phandle(hc->dev, idx);
		if (IS_ERR(edev) && PTR_ERR(edev) != -ENODEV)
			return PTR_ERR(edev);

		if (IS_ERR_OR_NULL(edev))
			continue;

		phandle_found = true;

		hc->extcon[idx].hc = hc;
		hc->extcon[idx].edev = edev;
		hc->extcon[idx].idx = idx;

		hc->extcon[idx].host_nb.notifier_call = sls11hc_host_notifier;
		ret = extcon_register_notifier(edev, EXTCON_USB_HOST,
					       &hc->extcon[idx].host_nb);
		if (ret < 0)
			continue;

		if (extcon_get_state(edev, EXTCON_USB_HOST))
			sls11hc_host_notifier(&hc->extcon[idx].host_nb,
					      true, edev);
	}

	if (!phandle_found) {
		dev_err(hc->dev, "no extcon device found\n");
		return -ENODEV;
	}

	return 0;
}

static struct fpga_manager *of_sls11hc_get_mgr(struct device_node *np)
{
	struct device_node *mgr_node;
	struct fpga_manager *mgr;

	of_node_get(np);
	mgr_node = of_parse_phandle(np, "fpga-mgr", 0);
	if (mgr_node) {
		mgr = of_fpga_mgr_get(mgr_node);
		of_node_put(mgr_node);
		of_node_put(np);
		return mgr;
	}
	of_node_put(np);

	return ERR_PTR(-EINVAL);
}

static struct fpga_image_info *sls11hc_fpga_image_info_alloc(struct device *dev)
{
	struct fpga_image_info *info;
	const char *image_name;
	const struct firmware *fw;
	int ret;

	if (of_property_read_string(dev->of_node, "firmware-name", &image_name))
		return ERR_PTR(-ENOENT);

	info = fpga_image_info_alloc(dev);
	if (!info)
		return ERR_PTR(-ENOMEM);

	info->firmware_name = devm_kstrdup(dev, image_name, GFP_KERNEL);
	if (!info->firmware_name)
		return ERR_PTR(-ENOMEM);

	ret = request_firmware(&fw, image_name, dev);
	if (ret)
		return ERR_PTR(ret);

	info->buf = devm_kmemdup(dev, fw->data, fw->size, GFP_KERNEL);
	if (!info->buf) {
		release_firmware(fw);
		return ERR_PTR(-ENOMEM);
	}

	info->count = fw->size;

	release_firmware(fw);

	return info;
}

static ssize_t fw_name_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sls11hc *hc = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%s\n", hc->info->firmware_name);
}
static DEVICE_ATTR_RO(fw_name);

static ssize_t fw_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sls11hc *hc = dev_get_drvdata(dev);
	const char *fw_buf = hc->info->buf;
	size_t fw_count = hc->info->count;
	const char *p;

	if (fw_buf[0] != 0xff || fw_buf[1] != 0x00)
		return snprintf(buf, PAGE_SIZE, "invalid firmware\n");

	buf[0] = '\0';

	p = fw_buf + 2;
	while ((p - fw_buf) < fw_count && *p != 0xff) {
		snprintf(buf, PAGE_SIZE, "%s%s ", buf, p);
		p += strlen(p) + 1;
	}
	buf[strlen(buf) - 1] = '\n';

	return strlen(buf);
}
static DEVICE_ATTR_RO(fw_info);

static struct attribute *sls11hc_attrs[] = {
	&dev_attr_fw_name.attr,
	&dev_attr_fw_info.attr,
	NULL,
};

static const struct attribute_group sls11hc_attr_group = {
	.name = NULL,	/* we want them in the same directory */
	.attrs = sls11hc_attrs,
};

static int sls11hc_probe(struct platform_device *pdev)
{
	struct sls11hc *hc;
	struct device *dev = &pdev->dev;
	struct spi_device *spi;
	int ret;

	dev_info(dev, "%s\n", __func__);

	hc = devm_kzalloc(dev, sizeof(*hc), GFP_KERNEL);
	if (!hc) {
		dev_err(dev, "failed to alloc hc\n");
		return -ENOMEM;
	}

	hc->dev = dev;
	dev_set_drvdata(dev, hc);

	INIT_DELAYED_WORK(&hc->sm_work, sls11hc_sm_work);

	hc->core_clk = devm_clk_get(dev, "core_clk");
	if (IS_ERR(hc->core_clk)) {
		ret = PTR_ERR(hc->core_clk);
		dev_err(dev, "failed to get core_clk %d\n", ret);
		hc->core_clk = NULL;
	} else {
		clk_set_rate(hc->core_clk, 19200000);
	}

	hc->vdda33 = devm_regulator_get(dev, "vdda33");
	if (IS_ERR(hc->vdda33)) {
		ret = PTR_ERR(hc->vdda33);
		dev_err(dev, "unable to get vdda33 supply %d\n", ret);
		hc->vdda33 = NULL;
	}

	hc->vdda18 = devm_regulator_get(dev, "vdda18");
	if (IS_ERR(hc->vdda18)) {
		ret = PTR_ERR(hc->vdda18);
		dev_err(dev, "unable to get vdda18 supply %d\n", ret);
		hc->vdda18 = NULL;
	}

	hc->ldo1_en = devm_gpiod_get(dev, "ldo1_en", GPIOD_OUT_LOW);
	if (IS_ERR(hc->ldo1_en)) {
		ret = PTR_ERR(hc->ldo1_en);
		dev_err(dev, "failed to get ldo1_en gpio %d\n", ret);
		hc->ldo1_en = NULL;
	}

	hc->ldo2_en = devm_gpiod_get(dev, "ldo2_en", GPIOD_OUT_LOW);
	if (IS_ERR(hc->ldo2_en)) {
		ret = PTR_ERR(hc->ldo2_en);
		dev_err(dev, "failed to get ldo2_en gpio %d\n", ret);
		hc->ldo2_en = NULL;
	}

	hc->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(hc->reset)) {
		ret = PTR_ERR(hc->reset);
		dev_err(dev, "failed to get reset gpio %d\n", ret);
		return ret;
	}

	hc->ds_sw_sel = devm_gpiod_get(dev, "ds_sw_sel", GPIOD_OUT_LOW);
	if (IS_ERR(hc->ds_sw_sel)) {
		ret = PTR_ERR(hc->ds_sw_sel);
		dev_err(dev, "failed to get ds_sw_sel gpio %d\n", ret);
		hc->ds_sw_sel = NULL;
	}

	if (!max_speed_hz) {
		ret = of_property_read_u32(dev->of_node,
				"spi-max-frequency", &max_speed_hz);
		if (ret) {
			dev_err(hc->dev, "sls11hc: no valid 'spi-max-frequency' property %d\n",
				ret);
			return ret;
		}
	}

	hc->mgr = of_sls11hc_get_mgr(dev->of_node);
	if (IS_ERR(hc->mgr)) {
		dev_err(dev, "unable to get fpga manager\n");
		return -EPROBE_DEFER;
	}

	if (!fpga_mgr_max_speed_hz) {
		spi = to_spi_device(hc->mgr->dev.parent);
		ret = of_property_read_u32(spi->dev.of_node,
				"spi-max-frequency", &fpga_mgr_max_speed_hz);
		if (ret) {
			dev_err(hc->dev, "fpga_mgr: no valid 'spi-max-frequency' property %d\n",
				ret);
			return ret;
		}
	}

	hc->info = sls11hc_fpga_image_info_alloc(dev);
	if (IS_ERR(hc->info)) {
		dev_err(dev, "failed to alloc fpga_image_info\n");
		return PTR_ERR(hc->info);
	}

	ret = sls11hc_extcon_register(hc);
	if (ret) {
		dev_err(dev, "failed to register extcon %d\n", ret);
		return ret;
	}

	ret = sysfs_create_group(&dev->kobj, &sls11hc_attr_group);
	if (ret) {
		dev_err(dev, "failed to register sysfs attr %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct of_device_id of_sls11hc_match[] = {
	{ .compatible = "sls,sls11hc", },
	{ },
};
MODULE_DEVICE_TABLE(of, of_sls11hc_match);

static struct platform_driver sls11hc_driver = {
	.probe = sls11hc_probe,
	.driver = {
		.name = "sls11hc",
		.of_match_table = of_sls11hc_match,
	},
};
module_platform_driver(sls11hc_driver);

MODULE_AUTHOR("Hansun Lee <hansun.lee@lge.com>");
MODULE_DESCRIPTION("SLS USB1.1 Host Controller Core");
MODULE_LICENSE("GPL v2");
