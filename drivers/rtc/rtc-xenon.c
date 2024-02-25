/*
 *  Xenon RTC via SMC driver.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>


#define DRV_NAME	"rtc-xenon"
#define DRV_VERSION	"0.1"

	/* for whatever reason, 15.Nov.2001 00:00 GMT */
#define	RTC_BASE	1005782400UL


int xenon_smc_message_wait(void *msg);

static unsigned long xenon_get_rtc(void)
{
	unsigned char msg[16] = { 0x04 };
	unsigned long msec;

	xenon_smc_message_wait(msg);
	msec = msg[1] | (msg[2] << 8) | (msg[3] << 16) |
		(msg[4] << 24) | ((unsigned long)msg[5] << 32);
	return RTC_BASE + msec/1000;
}

void xenon_smc_message(void *msg);

static int xenon_set_rtc(unsigned long secs)
{
	unsigned long msec = (secs - RTC_BASE) * 1000;
	unsigned char msg[16] = {
		0x85, msec & 0xFF, (msec >> 8) & 0xFF,
		(msec >> 16) & 0xFF, (msec >> 24) & 0xFF,
		(msec >> 32) & 0xFF };

	xenon_smc_message(msg);
	return 0;
}

static int xenon_read_time(struct device *dev, struct rtc_time *tm)
{
	rtc_time64_to_tm(xenon_get_rtc(), tm);
	return 0;
}


static int xenon_set_time(struct device *dev, struct rtc_time *tm)
{
	unsigned long time = 0;
	// int err;

	time = rtc_tm_to_time64(tm);
	// if (err)
	//	return err;

	return xenon_set_rtc(time);
}

static const struct rtc_class_ops xenon_rtc_ops = {
	.read_time	= xenon_read_time,
	.set_time	= xenon_set_time,
};

static int __init xenon_rtc_probe(struct platform_device *pdev)
{
	struct rtc_device *rtc = devm_rtc_device_register(&pdev->dev, DRV_NAME,
				     &xenon_rtc_ops, THIS_MODULE);

	printk("xenon_rtc_probe(%p) = %p\n", pdev, rtc);
	if (IS_ERR(rtc))
		return PTR_ERR(rtc);

	platform_set_drvdata(pdev, rtc);
	return 0;
}

static struct platform_driver xenon_rtc_driver = {
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.remove		= __exit_p(xenon_rtc_remove),
};

static int __init xenon_rtc_init(void)
{
	int ret = platform_driver_probe(&xenon_rtc_driver, xenon_rtc_probe);

	printk("xenon_rtc_init() = %d\n", ret);
	return ret;
	// return platform_driver_probe(&xenon_rtc_driver, xenon_rtc_probe);
}

static void __exit xenon_rtc_exit(void)
{
	platform_driver_unregister(&xenon_rtc_driver);
}

module_init(xenon_rtc_init);
module_exit(xenon_rtc_exit);

MODULE_AUTHOR("Herbert Poetzl <herbert@13thfloor.at>");
MODULE_DESCRIPTION("Xenon RTC driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
MODULE_ALIAS("platform:rtc-xenon");