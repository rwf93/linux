/*
 *  sata_xenon.c - SATA support for xenon southbridge
 *
 *  based on sata_sis.c, modifications by Felix Domke <tmbinc@elitedvb.net>
 *  minor modification by: wolie <wolie@telia.com>
 *
 *  		    Please ALWAYS copy linux-ide@vger.kernel.org
 *		    on emails.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2, or (at your option)
 *  any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; see the file COPYING.  If not, write to
 *  the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *
 *  libata documentation is available via 'make {ps|pdf}docs',
 *  as Documentation/DocBook/libata.*
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/blkdev.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <scsi/scsi_host.h>
#include <linux/libata.h>

#define DRV_NAME	"sata_xenon"
#define DRV_VERSION	"0.1.1"

	/* small note: it's completely unknown whether the xenon southbridge sata
	   is really based on SiS technology.
	   Most of SATA is standardized anyway.


	   So, we have these two pci devices, one for each port.

	   They have two BARs, one for the IDE registers (0..7,
	   altstatus/devctl is +0xA), and one for the BMDMA.

	   SCR seem to be sis-like in pci config space, but that should
	   be verified!

	   Note on the DVD-ROM part:

	   The drives usually require some tweaks to be usable under linux.

	   You either need to hack the scsi layer, or, in case of the GDR3120L,
	   set 'modeB' in the bootloader.
	*/

enum {
	/* PCI configuration registers */
	SIS_SCR_BASE		= 0xc0, /* sata0 phy SCR registers */
};

static int xenon_init_one (struct pci_dev *pdev, const struct pci_device_id *ent);
static int xenon_scr_read (struct ata_link *link, unsigned int sc_reg, u32 *val);
static int xenon_scr_write (struct ata_link *link, unsigned int sc_reg, u32 val);//void
static void xenon_bmdma_error_handler(struct ata_port *ap);

static const struct pci_device_id xenon_pci_tbl[] = {
	{ PCI_VDEVICE(MICROSOFT, 0x5803), 0 }, // Hdd
	{ PCI_VDEVICE(MICROSOFT, 0x5802), 0 }, // CdDvd

	{ }	/* terminate list */
};

static struct pci_driver xenon_pci_driver = {
	.name			= DRV_NAME,
	.id_table		= xenon_pci_tbl,
	.probe			= xenon_init_one,
	.remove			= ata_pci_remove_one,
};

static struct scsi_host_template xenon_sht = {
	ATA_BMDMA_SHT(DRV_NAME),
};

static struct ata_port_operations xenon_ops = {
	.inherits		= &ata_bmdma_port_ops,
// 	.lost_interrupt		= ATA_OP_NULL,
//	.error_handler		= xenon_bmdma_error_handler,
	.scr_read		= xenon_scr_read,
	.scr_write		= xenon_scr_write,
};

static const struct ata_port_info xenon_port_info = {
	.flags		= ATA_FLAG_SATA,
	.pio_mask	= ATA_PIO4,
	.mwdma_mask	= ATA_MWDMA2,
	.udma_mask	= ATA_UDMA6, //0x7F
	.port_ops	= &xenon_ops,
//	.irq_handler	= ata_interrupt,
//	.private_data   = NULL
};

static unsigned int get_scr_cfg_addr(unsigned int sc_reg)
{
	if ((sc_reg > SCR_CONTROL) || (sc_reg == SCR_ERROR)) /* doesn't exist in PCI cfg space */
		return -1;

	return SIS_SCR_BASE + (4 * sc_reg);

}

static int xenon_scr_read (struct ata_link *link, unsigned int sc_reg, u32 *val) //u32
{
	struct pci_dev *pdev = to_pci_dev(link->ap->host->dev);
	unsigned int cfg_addr;

	cfg_addr = get_scr_cfg_addr(sc_reg);

	if (cfg_addr == -1)
		return -EINVAL;

	pci_read_config_dword(pdev, cfg_addr, val);
	return 0;
}

static int xenon_scr_write (struct ata_link *link, unsigned int sc_reg, u32 val) //void
{
	struct pci_dev *pdev = to_pci_dev(link->ap->host->dev);
	unsigned int cfg_addr;

	cfg_addr = get_scr_cfg_addr(sc_reg);

	if (cfg_addr == -1)
		return -EINVAL;

	pci_write_config_dword(pdev, cfg_addr, val);
	return 0;
}

static int xenon_softreset(struct ata_link *link, unsigned int *classes, unsigned long deadline)
{
	struct ata_port *ap = link->ap;
	struct pci_dev *pdev = to_pci_dev(ap->host->dev);
		/* Host 0 (used for DVD-ROM) has a quirk when used with
		   an Toshiba/Samsung drive: It can hang after a device reset.

		   While the exact reason is unclear (anyone with a SATA port
		   analyzer?), this workaround will not let the reset happen, and
		   emulate the detection of an ATAPI device.

		   When the workaround is enabled, only ATAPI devices are supported
		   on host 0, but on this hardware, nothing else is possible anyway. */
	if (pdev->device == 0x5802)
	{
		classes[0] = ATA_DEV_ATAPI;
		classes[1] = ATA_DEV_NONE;
		printk(KERN_WARNING "Simulating reset of SATA device 0x5802\n");
		return -EIO;
	} else {
		return ata_sff_softreset(link, classes, deadline);
	}
}

static void xenon_bmdma_error_handler(struct ata_port *ap)
{
	ata_do_eh(ap, ata_std_prereset, xenon_softreset, sata_std_hardreset, ata_std_postreset);
}


static int xenon_init_one (struct pci_dev *pdev, const struct pci_device_id *ent)
{
	static int printed_version;
	struct ata_host *host;
	struct ata_ioports *ioaddr;
	struct ata_port_info pi = xenon_port_info;
	const struct ata_port_info *ppi[] = { &pi, NULL };
	int rc;

	if (!printed_version++)
		dev_printk(KERN_INFO, &pdev->dev, "version " DRV_VERSION "\n");

	rc = pci_enable_device(pdev);
	if (rc)
		return rc;

	rc = pci_request_regions(pdev, DRV_NAME);
	if (rc) {
		goto err_out;
	}

	rc = dma_set_mask(&pdev->dev, ATA_DMA_MASK);
	if (rc)
		goto err_out_regions;

	rc = dma_set_coherent_mask(&pdev->dev, ATA_DMA_MASK);
	if (rc)
		goto err_out_regions;

	host = ata_host_alloc_pinfo(&pdev->dev, ppi, 1);
	if (!host) {
		rc = -ENOMEM;
		goto err_out_regions;
	}

	ioaddr = &host->ports[0]->ioaddr;
	ioaddr->cmd_addr = ioremap(pci_resource_start(pdev, 0), PAGE_SIZE);
	ioaddr->altstatus_addr = ioaddr->cmd_addr + 0xa;
	ioaddr->ctl_addr = ioaddr->cmd_addr + 0xa;
	ioaddr->bmdma_addr = ioremap(pci_resource_start(pdev, 1), PAGE_SIZE);

	ata_sff_std_ports(ioaddr);

	pci_set_master(pdev);
	pci_intx(pdev, 1);

	return ata_host_activate(host, pdev->irq, ata_sff_interrupt,
				 IRQF_SHARED, &xenon_sht);

err_out_regions:
	pci_release_regions(pdev);

err_out:
	pci_disable_device(pdev);
	return rc;
}

static int __init xenon_init(void)
{
	return pci_register_driver(&xenon_pci_driver);
}

static void __exit xenon_exit(void)
{
	pci_unregister_driver(&xenon_pci_driver);
}

module_init(xenon_init);
module_exit(xenon_exit);

MODULE_DESCRIPTION("low-level driver for Xenon Southbridge SATA controller");
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(pci, xenon_pci_tbl);
MODULE_VERSION(DRV_VERSION);
