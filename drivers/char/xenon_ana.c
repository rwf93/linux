/*
 *  Xenon (H)ana via SMC character driver.
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

#define DRV_NAME	"xenon_ana"
#define DRV_VERSION	"0.2"


int xenon_smc_message_wait(void *msg);

static uint32_t ana_read_reg(uint8_t addr)
{
	unsigned char msg[16] = { 0x11,
		0x10, 0x05, 0x80 | 0x70, 0x00, 0xF0, addr };

	xenon_smc_message_wait(msg);
	return msg[4] | (msg[5] << 8) | (msg[6] << 16) | (msg[7] << 24);
}

static int ana_write_reg(uint8_t addr, uint32_t val)
{
	unsigned char msg[16] = { 0x11,
		0x60, 0x00, 0x80 | 0x70, 0x00, 0x00,
		addr, 0x00, val & 0xFF, (val >> 8) & 0xFF,
		(val >> 16) & 0xFF, (val >> 24) & 0xFF };

	xenon_smc_message_wait(msg);
	return msg[1];
}

static loff_t ana_llseek(struct file *file, loff_t offset, int origin)
{
	switch (origin) {
	case 1:
		offset += file->f_pos;
		break;
	case 2:
		offset += 0x400;
		break;
	}
	if (offset < 0)
		return -EINVAL;

	file->f_pos = offset;
	return file->f_pos;
}

typedef union {
	uint32_t val;
	uint8_t p[4];
} 	ana_reg_t;

static ssize_t ana_read(struct file *file, char __user *buf,
	size_t count, loff_t *ppos)
{
	uint32_t ppa = *ppos;

	if (*ppos >= 0x400UL)
		return -EINVAL;

	while (count) {
		/* optimize reads in same reg */
		int addr = ppa/4;
		int shift = ppa % 4;
		ana_reg_t r = { .val = ana_read_reg(addr) };

		int len = 4 - shift;

		if (len > count)
			len = count;
		if (copy_to_user(buf, &r.p[shift], len))
			return -EFAULT;

		count -= len;
		buf += len;
		ppa += len;

		/* end of register space? */
		if (ppa >= 0x400)
			break;
	}

	/* how much data was actually transferred? */
	count = ppa - *ppos;
	*ppos = ppa;
	return count;
}

static ssize_t ana_write(struct file *file, const char __user *buf,
	size_t count, loff_t *ppos)
{
	uint32_t ppa = *ppos;

	if (*ppos >= 0x400UL)
		return -EINVAL;

	while (count) {
		/* coalesce writes to same reg */
		int addr = ppa/4;
		int shift = ppa % 4;
		ana_reg_t r;

		int len = 4 - shift;

		if (len > count)
			len = count;

		/* handle partial write */
		if (len != 4)
			r.val = ana_read_reg(addr);

		if (copy_from_user(&r.p[shift], buf, len))
			return -EFAULT;

		/* FIXME: handle return code */
		ana_write_reg(addr, r.val);

		count -= len;
		buf += len;
		ppa += len;

		/* end of register space? */
		if (ppa >= 0x400)
			break;
	}

	/* how much data was actually transferred? */
	count = ppa - *ppos;
	*ppos = ppa;
	return count;
}

static long ana_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg)
{
	return -ENODEV;
}

static int ana_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int ana_release(struct inode *inode, struct file *file)
{
	return 0;
}


const struct file_operations ana_fops = {
	.owner		= THIS_MODULE,
	.llseek		= ana_llseek,
	.read		= ana_read,
	.write		= ana_write,
	.unlocked_ioctl	= ana_ioctl,
	.open		= ana_open,
	.release	= ana_release,
};

static struct miscdevice ana_dev = {
	.minor =  MISC_DYNAMIC_MINOR,
	"ana",
	&ana_fops
};

int __init ana_init(void)
{
	int ret = 0;

	printk(KERN_INFO "Xenon (H)ana char driver version " DRV_VERSION "\n");

	ret = misc_register(&ana_dev);
	return ret;
}

void __exit ana_exit(void)
{
	misc_deregister(&ana_dev);
}

module_init(ana_init);
module_exit(ana_exit);

MODULE_AUTHOR("Herbert Poetzl <herbert@13thfloor.at>");
MODULE_DESCRIPTION("Character Interface for Xenon (H)ana");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
