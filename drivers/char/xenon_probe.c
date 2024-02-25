/*
 *  Xenon Memory Probe character driver.
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
#include <asm/uaccess.h>
#include <asm/io.h>

#define DRV_NAME	"xenon_probe"
#define DRV_VERSION	"0.1"

static unsigned long base = 0xc8000000;
static unsigned long size = 0x10000;
static bool little_endian = 0;

module_param(base, ulong, 0);
MODULE_PARM_DESC(base, "Probe Memory Base");

module_param(size, ulong, 0);
MODULE_PARM_DESC(size, "Probe Memory Size");

module_param(little_endian, bool, 0);
MODULE_PARM_DESC(little_endian, "Probe Memory Endianess");

static void __iomem *mapped = NULL;


static uint32_t probe_map(uint32_t val)
{
	if (little_endian)
		return le32_to_cpu(val);
	else
		return be32_to_cpu(val);
}

static uint32_t probe_rmap(uint32_t val)
{
	if (little_endian)
		return cpu_to_le32(val);
	else
		return cpu_to_be32(val);
}


static loff_t probe_llseek(struct file *file, loff_t offset, int origin)
{
	switch (origin) {
	case 1:
		offset += file->f_pos;
		break;
	case 2:
		offset += size;
		break;
	}
	if ((offset < 0) || (offset >= size))
		return -EINVAL;

	file->f_pos = offset;
	return file->f_pos;
}

typedef union {
	uint32_t val;
	uint8_t p[4];
} 	probe_mem_t;

static ssize_t probe_read(struct file *file,
	char __user *buf, size_t count, loff_t *ppos)
{
	uint32_t ppa = *ppos;

	if (*ppos >= size)
		return -EINVAL;

	printk("probe_read(%x,%zx)\n", ppa, count);
	while (count) {
		/* optimize reads in same longword */
		unsigned long addr = ppa & ~3;
		int shift = ppa % 4;
		probe_mem_t r = { .val = probe_map(readl(mapped + addr)) };

		int len = 4 - shift;

		if (len > count)
			len = count;
		if (copy_to_user(buf, &r.p[shift], len))
			return -EFAULT;

		count -= len;
		buf += len;
		ppa += len;

		/* end of register space? */
		if (ppa >= size)
			break;
	}

	/* how much data was actually transferred? */
	count = ppa - *ppos;
	*ppos = ppa;
	return count;
}

static ssize_t probe_write(struct file *file,
	const char __user *buf, size_t count, loff_t *ppos)
{
	uint32_t ppa = *ppos;

	if (*ppos >= size)
		return -EINVAL;

	printk("probe_write(%x,%zx)\n", ppa, count);
	while (count) {
		/* coalesce writes to same reg */
		unsigned long addr = ppa & ~3;
		int shift = ppa % 4;
		probe_mem_t r;

		int len = 4 - shift;

		if (len > count)
			len = count;

		/* handle partial write */
		if (len != 4)
			r.val = probe_map(readl(mapped + addr));

		if (copy_from_user(&r.p[shift], buf, len))
			return -EFAULT;

		writel(probe_rmap(r.val), mapped + addr);

		count -= len;
		buf += len;
		ppa += len;

		/* end of register space? */
		if (ppa >= size)
			break;
	}

	/* how much data was actually transferred? */
	count = ppa - *ppos;
	*ppos = ppa;
	return count;
}

static long probe_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg)
{
	return -ENODEV;
}

static int probe_open(struct inode *inode, struct file *file)
{
	return generic_file_open(inode, file);
}

static int probe_release(struct inode *inode, struct file *file)
{
	return 0;
}


const struct file_operations probe_fops = {
	.owner		= THIS_MODULE,
	.llseek		= probe_llseek,
	.read		= probe_read,
	.write		= probe_write,
	.unlocked_ioctl	= probe_ioctl,
	.open		= probe_open,
	.release	= probe_release,
};

static struct miscdevice probe_dev = {
	.minor =  MISC_DYNAMIC_MINOR,
	"probe",
	&probe_fops
};

int __init probe_init(void)
{
	int ret = 0;

	printk(KERN_INFO "Xenon Memory Probe driver version " DRV_VERSION "\n");

	mapped = ioremap(base, size);
	if (!mapped)
		return -EINVAL;

	printk(KERN_INFO "XMP mapped 0x%04lx bytes @0x%08lx\n",
		size, base);

	ret = misc_register(&probe_dev);
	return ret;
}

void __exit probe_exit(void)
{
	misc_deregister(&probe_dev);
}

module_init(probe_init);
module_exit(probe_exit);

MODULE_AUTHOR("Herbert Poetzl <herbert@13thfloor.at>");
MODULE_DESCRIPTION("Xenon Memory Probe Interface");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
