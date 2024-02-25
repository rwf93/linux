/*
 *  Xenon HW Monitor via SMC driver.
 *
 *  Copyright (C) 2010 Herbert Poetzl
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>

#define DRV_NAME	"xenon-hwmon"
#define DRV_VERSION	"0.1"

#if 0
struct hwmon {
	spinlock_t	lock;

	struct device	*xenon_hwmon_dev;
};
#endif
static unsigned int fan_speed[2];

int xenon_smc_message_wait(void *msg);

static unsigned long xenon_get_temp(unsigned nr)
{
	unsigned char msg[16] = { 0x07 };
	static unsigned int temp[4] = { 0 };

	/* FIXME: only every N jiffies */
	xenon_smc_message_wait(msg);
	temp[0] = (msg[1] | (msg[2] << 8)) * 1000 / 256;
	temp[1] = (msg[3] | (msg[4] << 8)) * 1000 / 256;
	temp[2] = (msg[5] | (msg[6] << 8)) * 1000 / 256;
	temp[3] = (msg[7] | (msg[8] << 8)) * 1000 / 256;

	return temp[nr & 3];
}

void xenon_smc_message(void *msg);

static int xenon_set_cpu_fan_speed(unsigned val)
{
	unsigned char msg[16] = { 0x94, (val & 0x7F) | 0x80 };

	xenon_smc_message(msg);
	return 0;
}

static int xenon_set_gpu_fan_speed(unsigned val)
{
	unsigned char msg[16] = { 0x89, (val & 0x7F) | 0x80 };

	xenon_smc_message(msg);
	return 0;
}


static ssize_t show_fan_speed(struct device *dev, struct device_attribute *attr, char *buf)
{
	int fan_nr = to_sensor_dev_attr(attr)->index;
	// void *p = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", fan_speed[fan_nr]);
}

static ssize_t set_fan_speed(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	int fan_nr = to_sensor_dev_attr(attr)->index;
	unsigned int val = simple_strtol(buf, NULL, 10);
	// void *p = dev_get_drvdata(dev);

	fan_speed[fan_nr] = val & 0xFF;

	if (fan_nr == 0)
		xenon_set_cpu_fan_speed(val);
	if (fan_nr == 1)
		xenon_set_gpu_fan_speed(val);

	return count;
}

static SENSOR_DEVICE_ATTR(cpu_fan_speed, S_IRUGO | S_IWUSR,
		show_fan_speed, set_fan_speed, 0);
static SENSOR_DEVICE_ATTR(gpu_fan_speed, S_IRUGO | S_IWUSR,
		show_fan_speed, set_fan_speed, 1);

static ssize_t show_temp(struct device *dev, struct device_attribute *attr, char *buf)
{
	int temp_nr = to_sensor_dev_attr(attr)->index;
	// struct dev *p = dev_get_drvdata(pdev);
	unsigned temp = xenon_get_temp(temp_nr);

	return sprintf(buf, "%d\n", temp);
}

static SENSOR_DEVICE_ATTR(cpu_temp, S_IRUGO, show_temp, NULL, 0);
static SENSOR_DEVICE_ATTR(gpu_temp, S_IRUGO, show_temp, NULL, 1);
static SENSOR_DEVICE_ATTR(edram_temp, S_IRUGO, show_temp, NULL, 2);
static SENSOR_DEVICE_ATTR(motherboard_temp, S_IRUGO, show_temp, NULL, 3);

static ssize_t show_name(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "xenon\n");
}

static SENSOR_DEVICE_ATTR(name, S_IRUGO, show_name, NULL, 0);

static struct attribute *xenon_hwmon_attributes[] = {
	&sensor_dev_attr_cpu_fan_speed.dev_attr.attr,
	&sensor_dev_attr_gpu_fan_speed.dev_attr.attr,
	&sensor_dev_attr_cpu_temp.dev_attr.attr,
	&sensor_dev_attr_gpu_temp.dev_attr.attr,
	&sensor_dev_attr_edram_temp.dev_attr.attr,
	&sensor_dev_attr_motherboard_temp.dev_attr.attr,
	&sensor_dev_attr_name.dev_attr.attr,
	NULL,
};

static const struct attribute_group xenon_hwmon_group = {
	.attrs = xenon_hwmon_attributes,
};

static int __init xenon_hwmon_probe(struct platform_device *pdev)
{
	struct device *dev;
	int err;

	err = sysfs_create_group(&pdev->dev.kobj, &xenon_hwmon_group);
	if (err)
		goto out;

	dev = hwmon_device_register(&pdev->dev);
	if (IS_ERR(dev)) {
		err = PTR_ERR(dev);
		goto out_sysfs_remove_group;
	}

	platform_set_drvdata(pdev, dev);
	return 0;

out_sysfs_remove_group:
	sysfs_remove_group(&pdev->dev.kobj, &xenon_hwmon_group);
out:
	return err;
}

static int __exit xenon_hwmon_remove(struct platform_device *pdev)
{
	hwmon_device_unregister(&pdev->dev);
	sysfs_remove_group(&pdev->dev.kobj, &xenon_hwmon_group);
	return 0;
}


static struct platform_driver xenon_hwmon_driver = {
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.remove		= __exit_p(xenon_hwmon_remove),
};

static int __init xenon_hwmon_init(void)
{
	int ret = platform_driver_probe(&xenon_hwmon_driver, xenon_hwmon_probe);

	printk("xenon_hwmon_init() = %d\n", ret);
	return ret;
}

static void __exit xenon_hwmon_exit(void)
{
	platform_driver_unregister(&xenon_hwmon_driver);
}

module_init(xenon_hwmon_init);
module_exit(xenon_hwmon_exit);

MODULE_AUTHOR("Herbert Poetzl <herbert@13thfloor.at>");
MODULE_DESCRIPTION("Character Interface for Xenon (H)ana");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);