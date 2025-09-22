// SPDX-License-Identifier: GPL-2.0
/*
 *  Xenon HW Monitor via SMC driver.
 *
 *  Copyright (C) 2025 Michael "Techflash" Garofalo
 *  Copyright (C) 2010 Herbert Poetzl
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>

#define DRV_NAME	"xenon-hwmon"
#define DRV_VERSION	"0.2"

#if 0
struct hwmon {
	spinlock_t	lock;

	struct device	*xenon_hwmon_dev;
};
#endif
static unsigned int fan_speed[2];

static const char *temp_labels[4] = {
	"CPU",
	"GPU",
	"EDRAM",
	"Motherboard"
};

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


static int xenon_read(struct device *dev, enum hwmon_sensor_types type,
                      u32 attr, int channel, long *val)
{
	switch (type) {
	case hwmon_temp:
		if (attr == hwmon_temp_input) {
			*val = xenon_get_temp(channel);
			return 0;
		}
		break;
	case hwmon_pwm:
		if (attr == hwmon_pwm_input) {
			*val = fan_speed[channel];
			return 0;
		}
		break;
	default:
		break;
	}
	return -EOPNOTSUPP;
}

static int xenon_read_string(struct device *dev, enum hwmon_sensor_types type,
                             u32 attr, int channel, const char **str)
{
	switch (type) {
	case hwmon_temp:
		if (attr == hwmon_temp_label) {
			if (channel > 3 || channel < 0)
				return -EINVAL;
			*str = temp_labels[channel];
			return 0;
		}
		break;
	/* labels not supported for PWM sadly :( */
	default:
		break;
	}
	return -EOPNOTSUPP;
}

static int xenon_write(struct device *dev, enum hwmon_sensor_types type,
                       u32 attr, int channel, long val)
{
	switch (type) {
	case hwmon_pwm:
		if (attr == hwmon_pwm_input) {
			fan_speed[channel] = val & 0xff;
			if (channel == 0)
				xenon_set_cpu_fan_speed(val);
			if (channel == 1)
				xenon_set_gpu_fan_speed(val);
			return 0;
		}
		break;
	default:
		break;
	}
	return -EOPNOTSUPP;
}

static umode_t xenon_is_visible(const void *data,
                               enum hwmon_sensor_types type,
			       u32 attr, int channel)
{
	switch (type) {
	case hwmon_pwm:
		switch (attr) {
		case hwmon_pwm_input:
			return 0644;
		default:
		}
		return 0;
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_label:
		case hwmon_temp_input:
			return 0444;
		default:
		}
		return 0;
	default:
		return 0;
	}

}

static const struct hwmon_channel_info *xenon_info[] = {
	HWMON_CHANNEL_INFO(temp,
		HWMON_T_INPUT | HWMON_T_LABEL,   /* cpu */
		HWMON_T_INPUT | HWMON_T_LABEL,   /* gpu */
		HWMON_T_INPUT | HWMON_T_LABEL,   /* edram */
		HWMON_T_INPUT | HWMON_T_LABEL    /* motherboard */
	),
	HWMON_CHANNEL_INFO(pwm,
		HWMON_PWM_INPUT, /* cpu fan */
		HWMON_PWM_INPUT  /* gpu fan */
	),
	NULL
};

static const struct hwmon_ops xenon_ops = {
	.is_visible = xenon_is_visible,
	.read = xenon_read,
	.read_string = xenon_read_string,
	.write = xenon_write,
};

static const struct hwmon_chip_info xenon_chip_info = {
	.ops = &xenon_ops,
	.info = xenon_info,
};


static int __init xenon_hwmon_probe(struct platform_device *pdev)
{
	struct device *dev;

	dev = hwmon_device_register_with_info(&pdev->dev,
					  "xenon",
					  NULL,
					  &xenon_chip_info,
					  NULL);
	return PTR_ERR_OR_ZERO(dev);
}

static int __exit xenon_hwmon_remove(struct platform_device *pdev)
{
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

MODULE_AUTHOR("Michael \"Techflash\" Garofalo <officialTechflashYT@gmail.com>");
MODULE_DESCRIPTION("Xenon HW Monitor via SMC driver - revised");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
