/*
 *  Xenon SMC core.
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
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>

#define DRV_NAME	"xenon_smc_core"
#define DRV_VERSION	"0.1"

struct xenon_smc
{
	void __iomem *base;
	wait_queue_head_t wait_q;
	spinlock_t fifo_lock;

	void (*send)(void *msg);
	void (*wait)(void *msg);
	int (*reply)(void *msg);
	int (*cached)(void *msg);

	unsigned char cmd;
};

static struct xenon_smc smc;

static unsigned char smc_reply[][16] = {
	{ 0x01 },	/* power on type */
	{ 0x04 },	/* rtc */
	{ 0x07 },	/* temp */
	{ 0x0a },	/* tray state */
	{ 0x0f },	/* av pack */
	{ 0x11 },	/* (h)ana */
	{ 0x12 },	/* smc version */
	{ 0x13 },	/* echo back */
	{ 0x16 },	/* IR address */
	{ 0x17 },	/* tilt state */
	{ 0x1e },	/* 12b @83h */
	{ 0x20 },	/* 12b @8fh */
	{ 0x83 },	/* smc event */
};


static void _xenon_smc_send(void *msg)
{
	unsigned long flags;

	print_hex_dump(KERN_DEBUG, "_xenon_smc_send: ",
		DUMP_PREFIX_NONE, 16, 2, msg, 16, 0);

	spin_lock_irqsave(&smc.fifo_lock, flags);
	while (!(readl(smc.base + 0x84) & 4))
		cpu_relax();

	writel(4, smc.base + 0x84);
	writesl(smc.base + 0x80, msg, 4);
	writel(0, smc.base + 0x84);
	spin_unlock_irqrestore(&smc.fifo_lock, flags);
}

static void _xenon_smc_wait(void *msg)
{
	/* do we expect a reply? */
	if (*(unsigned char *)msg & 0x80)
		return;

#if 0
	/* wait for reply, maybe a timeout? */
	while (!(readl(smc.base + 0x94) & 4))
		cpu_relax();
#else
	wait_event_interruptible(smc.wait_q,
		smc.cmd == *(unsigned char *)msg);
#endif
}


static int _xenon_smc_reply(void *msg)
{
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&smc.fifo_lock, flags);
	if (readl(smc.base + 0x94) & 4) {
		writel(4, smc.base + 0x94);
		readsl(smc.base + 0x90, msg, 4);
		writel(0, smc.base + 0x94);
		ret = 1;
	} else {
		memset(msg, 0, 16);
	}
	spin_unlock_irqrestore(&smc.fifo_lock, flags);
	return ret;
}

static unsigned char * _xenon_smc_cache_lookup(unsigned char *msg) {
	unsigned char *ptr = smc_reply[0];

	while (ptr[0]) {
		if (msg[0] == ptr[0])
			return ptr;
		ptr += 16;
	}
	return NULL;
}

static int _xenon_smc_cached_reply(void *msg)
{
	unsigned char *ptr = _xenon_smc_cache_lookup(msg);

	if (ptr)
		memcpy(msg + 1, ptr + 1, 15);
	return (ptr != NULL);
}


static void _xenon_smc_cache(unsigned char *msg) {
	unsigned char *ptr = _xenon_smc_cache_lookup(msg);

	if (ptr)
		memcpy(ptr + 1, msg + 1, 15);
	else
		printk("unknown smc reply %02x", msg[0]);

	print_hex_dump(KERN_DEBUG, "_xenon_smc_cache: ",
		DUMP_PREFIX_NONE, 16, 2, msg, 16, 0);
	smc.cmd = msg[0];
}


int	xenon_smc_message(void *msg)
{
	int ret = 0;

	if (smc.send)
		smc.send(msg);

	return ret;
}

EXPORT_SYMBOL_GPL(xenon_smc_message);

int	xenon_smc_message_wait(void *msg)
{
	int ret = 0;

	smc.cmd = 0;
	if (smc.send)
		smc.send(msg);
	if (smc.wait)
		smc.wait(msg);
#if 0
	if (smc.reply)
		ret = smc.reply(msg);
#else
	if (smc.cached)
		ret = smc.cached(msg);
#endif
	return ret;
}

EXPORT_SYMBOL_GPL(xenon_smc_message_wait);


static void show_logo(void)
{
	// 0x99 = LED_CONTROL
	// 0x01 = SET
	// [green] [red]
	// bit position is light
	// 8 = br, 4 = bl, 2 = tr, 1 = tl
	unsigned char msg[16] = {0x99, 0x01, 0x78, 0};

	xenon_smc_message(msg);
}

void xenon_smc_restart(void)
{
	unsigned char msg[16] = {0x82, 0x04, 0x30, 0};

	xenon_smc_message(msg);
}

void xenon_smc_power_off(void)
{
	unsigned char msg[16] = {0x82, 0x01, 0x00, 0};

	xenon_smc_message(msg);
}

void xenon_smc_halt(void)
{
	return;
}

static const struct pci_device_id xenon_smc_pci_tbl[] = {
	{ PCI_VDEVICE(MICROSOFT, 0x580d), 0 },
	{ }	/* terminate list */
};

static irqreturn_t xenon_smc_irq(int irq, void *dev_id)
{
	static unsigned char msg[16];
	// struct pci_dev *pdev = dev_id;

	// on power press:
	// xenon_smc_irq() = 10000000,00000004
	// _xenon_smc_cache: 8311 0100 0000 0000 0000 0000 0000 0000

	unsigned int irqs = readl(smc.base + 0x50);

	printk(KERN_DEBUG "xenon_smc_irq() = %08x,%08x\n",
		irqs, readl(smc.base + 0x94));

	if (irqs & 0x10000000) {
		if (_xenon_smc_reply(msg)) {
			_xenon_smc_cache(msg);
			wake_up(&smc.wait_q);
		}
	}

	writel(irqs, smc.base + 0x58);	// ack irq
	return IRQ_NONE;
}

static int xenon_smc_init_one (struct pci_dev *pdev, const struct pci_device_id *ent)
{
	// static int printed_version;
	int rc;
	int pci_dev_busy = 0;
	unsigned long mmio_start;

	dev_printk(KERN_INFO, &pdev->dev, "version " DRV_VERSION "\n");

	rc = pci_enable_device(pdev);
	if (rc)
		return rc;

	rc = pci_request_regions(pdev, DRV_NAME);
	if (rc) {
		pci_dev_busy = 1;
		goto err_out;
	}

	pci_intx(pdev, 1);

	printk(KERN_INFO "attached to xenon SMC\n");

	mmio_start = pci_resource_start(pdev, 0);
	smc.base = ioremap(mmio_start, 0x100);
	if (!smc.base)
		goto err_out_regions;

	init_waitqueue_head(&smc.wait_q);
	spin_lock_init(&smc.fifo_lock);

	if (request_irq(pdev->irq, xenon_smc_irq, IRQF_SHARED,
		"xenon-smc", pdev)) {
		printk(KERN_ERR "xenon-smc: request_irq failed\n");
		goto err_out_ioremap;
	}

	smc.send = _xenon_smc_send;
	smc.wait = _xenon_smc_wait;
	smc.reply = _xenon_smc_reply;
	smc.cached = _xenon_smc_cached_reply;

	show_logo();
	return 0;

err_out_ioremap:
	iounmap(smc.base);

err_out_regions:
	pci_release_regions(pdev);

err_out:
	if (!pci_dev_busy)
		pci_disable_device(pdev);
	return rc;
}

static void xenon_smc_remove(struct pci_dev *pdev)
{
	smc.send = NULL;
	smc.reply = NULL;

	iounmap(smc.base);

	pci_release_regions(pdev);
	pci_disable_device(pdev);
}

static struct pci_driver xenon_smc_pci_driver = {
	.name			= DRV_NAME,
	.id_table		= xenon_smc_pci_tbl,
	.probe			= xenon_smc_init_one,
	.remove			= xenon_smc_remove
};


static int __init xenon_smc_init(void)
{
	return pci_register_driver(&xenon_smc_pci_driver);
}

static void __exit xenon_smc_exit(void)
{
	pci_unregister_driver(&xenon_smc_pci_driver);
}

module_init(xenon_smc_init);
module_exit(xenon_smc_exit);


MODULE_DESCRIPTION("Driver for Xenon Southbridge SMC");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
MODULE_DEVICE_TABLE(pci, xenon_smc_pci_tbl);
