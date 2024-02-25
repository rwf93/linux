/*
 *  Xenon Nand.
 *
 *  Copyright (C) 2024 rwf93
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
#include <linux/pci.h>

#define DRV_NAME "xenon_nand"
#define DRV_VERSION "0.1"

#if 1
#define dprintk(f, x...) do { printk(KERN_DEBUG f "\n" , ##x); } while (0)
#else
#define dprintk(f, x...)
#endif

static int xenon_nand_init_probe(struct pci_dev *pdev, const struct pci_device_id *ent) {
	dprintk("Xenon Nand PCI Probe");
	return 0;	
}

static void xenon_nand_remove(struct pci_dev *pdev) {

}

static const struct pci_device_id xenon_nand_pci_tbl[] = {
	{ PCI_VDEVICE(MICROSOFT, 0x580b), 0 }, // Slim
	{ }
};

static struct pci_driver xenon_nand_pci_driver = {
	.name = DRV_NAME,
	.id_table = xenon_nand_pci_tbl,
	.probe = xenon_nand_init_probe,
	.remove = xenon_nand_remove
};

static int __init xenon_nand_init(void) {
	return pci_register_driver(&xenon_nand_pci_driver);
}

static void __exit xenon_nand_exit(void) {
	pci_unregister_driver(&xenon_nand_pci_driver);	
}

module_init(xenon_nand_init);
module_exit(xenon_nand_exit);

MODULE_DESCRIPTION("Driver for Xenon Nand");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
MODULE_DEVICE_TABLE(pci, xenon_nand_pci_tbl);
