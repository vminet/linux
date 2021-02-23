// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2019 Hewlett-Packard Development Company, L.P.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/io.h>
#include <linux/mutex.h>

#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include "gxp-soclib.h"

#define FN2_VPBTN	0x46
#define FN2_SEVSTAT	0x70
#define FN2_SEVMASK	0x74

struct gxp_fn2_drvdata {
	struct platform_device *pdev;
	struct device *dev;
	void __iomem *base;
	struct mutex mutex;
	int irq;
	struct work_struct irq_work;
};

static ssize_t vpbtn_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct gxp_fn2_drvdata *drvdata = dev_get_drvdata(dev);
	unsigned int value;
	ssize_t ret;

	mutex_lock(&drvdata->mutex);

	value = readb(drvdata->base + FN2_VPBTN);
	ret = sprintf(buf, "0x%02x", value);

	mutex_unlock(&drvdata->mutex);
	return ret;
}

static ssize_t vpbtn_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct gxp_fn2_drvdata *drvdata = dev_get_drvdata(dev);
	unsigned int value;
	int rc;

	rc = kstrtouint(buf, 0, &value);
	if (rc < 0)
		return -EINVAL;

	mutex_lock(&drvdata->mutex);

	writeb((value > 0)?0x01:0x00, drvdata->base + FN2_VPBTN);

	mutex_unlock(&drvdata->mutex);
	return count;
}
static DEVICE_ATTR_RW(vpbtn);

static ssize_t se_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct gxp_fn2_drvdata *drvdata = dev_get_drvdata(dev);
	unsigned int value;
	ssize_t ret;

	mutex_lock(&drvdata->mutex);

	value = readl(drvdata->base + FN2_SEVSTAT);
	ret = sprintf(buf, "0x%08x", value);

	mutex_unlock(&drvdata->mutex);
	return ret;
}
static DEVICE_ATTR_RO(se_status);

static ssize_t host_power_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct gxp_fn2_drvdata *drvdata = dev_get_drvdata(dev);
	unsigned int value;
	ssize_t ret;

	mutex_lock(&drvdata->mutex);

	value = readl(drvdata->base + FN2_SEVSTAT);
	//bit 24, PGOOD
	if (value & 0x01000000)
		ret = sprintf(buf, "on");
	else
		ret = sprintf(buf, "off");

	mutex_unlock(&drvdata->mutex);
	return ret;
}
static DEVICE_ATTR_RO(host_power);

static ssize_t se_mask_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct gxp_fn2_drvdata *drvdata = dev_get_drvdata(dev);
	unsigned int value;
	ssize_t ret;

	mutex_lock(&drvdata->mutex);

	value = readl(drvdata->base + FN2_SEVMASK);
	ret = sprintf(buf, "0x%08x", value);

	mutex_unlock(&drvdata->mutex);
	return ret;
}

static ssize_t se_mask_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct gxp_fn2_drvdata *drvdata = dev_get_drvdata(dev);
	unsigned int value;
	int rc;

	rc = kstrtouint(buf, 0, &value);
	if (rc < 0)
		return -EINVAL;

	mutex_lock(&drvdata->mutex);

	writel(value, drvdata->base + FN2_SEVMASK);

	mutex_unlock(&drvdata->mutex);
	return count;

}
static DEVICE_ATTR_RW(se_mask);

static struct attribute *fn2_attrs[] = {
	&dev_attr_vpbtn.attr,
	&dev_attr_se_status.attr,
	&dev_attr_se_mask.attr,
	&dev_attr_host_power.attr,
	NULL,
};
ATTRIBUTE_GROUPS(fn2);

static int sysfs_register(struct device *parent,
			struct gxp_fn2_drvdata *drvdata)
{
	struct device *dev;

	dev = device_create_with_groups(soc_class, parent, 0,
					drvdata, fn2_groups, "fn2");
	if (IS_ERR(dev))
		return PTR_ERR(dev);
	drvdata->dev = dev;
	return 0;
}

static void fn2_irq_work(struct work_struct *ws)
{
	struct gxp_fn2_drvdata *drvdata =
			container_of(ws, struct gxp_fn2_drvdata, irq_work);
	char *envp[2];
	char buf[128];
	uint32_t value;


	value = readl(drvdata->base + FN2_SEVSTAT);
	writel(0xffffffff, drvdata->base + FN2_SEVSTAT);

	if (value & 0x00000001) {
		//system power change
		if (value & 0x01000000) {
			//power on
			sprintf(buf, "HOST_POWER=on");
			envp[0] = buf;
			envp[1] = NULL;
			kobject_uevent_env(&drvdata->dev->kobj,
					KOBJ_CHANGE, envp);
		} else {
			//power off
			sprintf(buf, "HOST_POWER=off");
			envp[0] = buf;
			envp[1] = NULL;
			kobject_uevent_env(&drvdata->dev->kobj,
					KOBJ_CHANGE, envp);
		}
	} else {
		sprintf(buf, "SEVSTAT=0x%08x", value);
		envp[0] = buf;
		envp[1] = NULL;
		kobject_uevent_env(&drvdata->dev->kobj, KOBJ_CHANGE, envp);
	}
	enable_irq(drvdata->irq);
}

static irqreturn_t fn2_irq_handle(int irq, void *_drvdata)
{
	struct gxp_fn2_drvdata *drvdata = (struct gxp_fn2_drvdata *)_drvdata;

	disable_irq_nosync(irq);
	schedule_work(&drvdata->irq_work);

	return IRQ_HANDLED;
}

static int gxp_fn2_probe(struct platform_device *pdev)
{
	int ret;
	struct gxp_fn2_drvdata *drvdata;
	struct resource *res;

	drvdata = devm_kzalloc(&pdev->dev, sizeof(struct gxp_fn2_drvdata),
				GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	drvdata->pdev = pdev;
	platform_set_drvdata(pdev, drvdata);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	drvdata->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(drvdata->base))
		return PTR_ERR(drvdata->base);

	mutex_init(&drvdata->mutex);
	sysfs_register(&pdev->dev, drvdata);

	ret = platform_get_irq(pdev, 0);
	if (ret < 0) {
		dev_err(&pdev->dev, "irq request failed\n");
		return ret;
	}
	drvdata->irq = ret;

	INIT_WORK(&drvdata->irq_work, fn2_irq_work);
	ret = devm_request_irq(&pdev->dev, drvdata->irq, fn2_irq_handle,
				IRQF_SHARED, "fn2", drvdata);
	if (ret < 0) {
		dev_err(&pdev->dev, "irq request failed\n");
		return ret;
	}

	return ret;
}

static const struct of_device_id gxp_fn2_of_match[] = {
	{ .compatible = "hpe,gxp-fn2" },
	{},
};
MODULE_DEVICE_TABLE(of, gxp_fn2_of_match);

static struct platform_driver gxp_fn2_driver = {
	.probe = gxp_fn2_probe,
	.driver = {
		.name = "gxp-fn2",
		.of_match_table = of_match_ptr(gxp_fn2_of_match),
	},
};
module_platform_driver(gxp_fn2_driver);

MODULE_AUTHOR("Gilbert Chen <gilbert.chen@hpe.com>");
MODULE_DESCRIPTION("HPE GXP FN2 Driver");
