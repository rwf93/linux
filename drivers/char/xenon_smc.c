/*
 *  Xenon SMC character driver.
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
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

#define DRV_NAME	"xenon_smc"
#define DRV_VERSION	"0.2"


/* single access for now */

static unsigned long is_active;
static unsigned char msg[16];

static ssize_t smc_read(struct file *file, char __user *buf,
	size_t count, loff_t *ppos)
{
	if ((count != 16) || *ppos)
		return -EINVAL;
	if (copy_to_user(buf, msg, 0x10))
		return -EFAULT;

	return 16;
}

int xenon_smc_message_wait(void *msg);

static ssize_t smc_write(struct file *file, const char __user *buf,
	size_t count, loff_t *ppos)
{
	if ((count != 16) || *ppos)
		return -EINVAL;

	if (copy_from_user(msg, buf, 16))
		return -EFAULT;

	xenon_smc_message_wait(msg);

	return 16;
}

static long smc_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg)
{
	return -ENODEV;
}

static int smc_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit(0, &is_active))
		return -EBUSY;

	return nonseekable_open(inode, file);
}

static int smc_release(struct inode *inode, struct file *file)
{
	clear_bit(0, &is_active);
	return 0;
}


const struct file_operations smc_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.read		= smc_read,
	.write		= smc_write,
	.unlocked_ioctl	= smc_ioctl,
	.open		= smc_open,
	.release	= smc_release,
};

static struct miscdevice smc_dev = {
	.minor =  MISC_DYNAMIC_MINOR,
	"smc",
	&smc_fops
};

int __init smc_init(void)
{
	int ret = 0;

	printk(KERN_INFO "Xenon SMC char driver version " DRV_VERSION "\n");

	ret = misc_register(&smc_dev);
	return ret;
}

void __exit smc_exit(void)
{
	misc_deregister(&smc_dev);
}

module_init(smc_init);
module_exit(smc_exit);

MODULE_AUTHOR("Herbert Poetzl <herbert@13thfloor.at>");
MODULE_DESCRIPTION("Character Interface for Xenon Southbridge SMC");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
