/*
 * Xenon PCI support
 * Maintained by: Felix Domke <tmbinc@elitedvb.net>
 * Minor modification by: wolie <wolie@telia.com>
 * based on:
 * Copyright (C) 2004 Benjamin Herrenschmuidt (benh@kernel.crashing.org),
 *		      IBM Corp.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/init.h>

#include <asm/sections.h>
#include <asm/io.h>
#include <asm/prom.h>
#include <asm/pci-bridge.h>
#include <asm/machdep.h>
#include <asm/iommu.h>
#include <asm/ppc-pci.h>

#include <asm/dma-mapping.h>

#ifdef DEBUG
#define DBG(x...) printk(x)
#else
#define DBG(x...)
#endif

#define OFFSET(bus, slot, func)	\
	((((bus) << 8) + PCI_DEVFN(slot, func)) << 12)

static int xenon_pci_read_config(struct pci_bus *bus, unsigned int devfn,
			      int offset, int len, u32 *val)
{
	struct pci_controller *hose;
	unsigned int slot = PCI_SLOT(devfn);
	unsigned int func = PCI_FUNC(devfn);
	void* addr;

	hose = pci_bus_to_host(bus);
	if (hose == NULL)
		return PCIBIOS_DEVICE_NOT_FOUND;

	DBG("xenon_pci_read_config, slot %d, func %d\n", slot, func);

#if 0
	if (PCI_SLOT(devfn) >= 32)
		return PCIBIOS_DEVICE_NOT_FOUND;
	if (PCI_SLOT(devfn) == 3)
		return PCIBIOS_DEVICE_NOT_FOUND;
	if (PCI_SLOT(devfn) == 6)
		return PCIBIOS_DEVICE_NOT_FOUND;
	if (PCI_SLOT(devfn) == 0xB)
		return PCIBIOS_DEVICE_NOT_FOUND;
	if (PCI_FUNC(devfn) >= 2)
		return PCIBIOS_DEVICE_NOT_FOUND;
#endif
	DBG("xenon_pci_read_config, %p, devfn=%d, offset=%d, len=%d\n", bus, devfn, offset, len);

	addr = ((void*)hose->cfg_addr) + offset;

	/* map GPU to slot 0x0f */
	if (slot == 0x0f)
		addr += OFFSET(0, 0x02, func);
	else
		addr += OFFSET(1, slot, func);

	/*
	 * Note: the caller has already checked that offset is
	 * suitably aligned and that len is 1, 2 or 4.
	 */
	switch (len) {
	case 1:
		*val = in_8((u8 *)addr);
		break;
	case 2:
		*val = in_le16((u16 *)addr);
		break;
	default:
		*val = in_le32((u32 *)addr);
		break;
	}
	DBG("->%08x\n", (int)*val);
	return PCIBIOS_SUCCESSFUL;
}

static int xenon_pci_write_config(struct pci_bus *bus, unsigned int devfn,
			       int offset, int len, u32 val)
{
	struct pci_controller *hose;
	unsigned int slot = PCI_SLOT(devfn);
	unsigned int func = PCI_FUNC(devfn);
	void *addr;

	hose = pci_bus_to_host(bus);
	if (hose == NULL)
		return PCIBIOS_DEVICE_NOT_FOUND;

	DBG("xenon_pci_write_config, slot %d, func %d\n", slot, func);

	if (PCI_SLOT(devfn) >= 32)
		return PCIBIOS_DEVICE_NOT_FOUND;
#if 0
	if (PCI_SLOT(devfn) == 3)
		return PCIBIOS_DEVICE_NOT_FOUND;
#endif
	DBG("xenon_pci_write_config, %p, devfn=%d, offset=%x, len=%d, val=%08x\n", bus, devfn, offset, len, val);

	addr = ((void*)hose->cfg_addr) + offset;

	/* map GPU to slot 0x0f */
	if (slot == 0x0f)
		addr += OFFSET(0, 0x02, func);
	else
		addr += OFFSET(1, slot, func);

	if (len == 4)
		DBG("was: %08x\n", readl(addr));
	if (len == 2)
		DBG("was: %04x\n", readw(addr));
	if (len == 1)
		DBG("was: %02x\n", readb(addr));
	/*
	 * Note: the caller has already checked that offset is
	 * suitably aligned and that len is 1, 2 or 4.
	 */
	switch (len) {
	case 1:
		writeb(val, addr);
		break;
	case 2:
		writew(val, addr);
		break;
	default:
		writel(val, addr);
		break;
	}
	return PCIBIOS_SUCCESSFUL;
}

static struct pci_ops xenon_pci_ops =
{
	.read	= xenon_pci_read_config,
	.write	= xenon_pci_write_config,
};


#if 1
void __init xenon_pci_init(void)
{
	struct pci_controller *hose;
	struct device_node *np, *root;
	struct device_node *dev = NULL;

	root = of_find_node_by_path("/");
	if (root == NULL) {
		printk(KERN_CRIT "xenon_pci_init: can't find root of device tree\n");
		return;
	}
	for (np = NULL; (np = of_get_next_child(root, np)) != NULL;) {
		if (np->name == NULL)
			continue;
		// printk("found node %p %s\n", np, np->name);
		if (strcmp(np->name, "pci") == 0) {
			of_node_get(np);
			dev = np;
		}
	}
	of_node_put(root);

	if (!dev)
	{
		printk("couldn't find PCI node!\n");
		return;
	}

	hose = pcibios_alloc_controller(dev);
	if (hose == NULL)
	{
		printk("pcibios_alloc_controller failed!\n");
		return;
	}

	hose->first_busno = 0;
	hose->last_busno = 1;

	hose->ops = &xenon_pci_ops;
	hose->cfg_addr = ioremap(0xd0000000, 0x1000000);

	pci_process_bridge_OF_ranges(hose, dev, 1);

//	of_rescan_bus(root, ci_bus *bus)

	/* Tell pci.c to not change any resource allocations.  */
	pci_set_flags(PCI_PROBE_ONLY);

	of_node_put(dev);
	DBG("PCI initialized\n");

	pci_io_base = 0;

	// pcibios_scan_phb(hose, dev);
	// Xbox 360 doesn't have IOMMU support. Who made this code?
	// set_pci_dma_ops(&dma_direct_ops);
}

#else


static int __init xenon_add_bridge(struct device_node *dev)
{
	int len;
	struct pci_controller *hose;
	struct resource rsrc;
	char *disp_name;
	const int *bus_range;
	int primary = 1, has_address = 0;

	printk(KERN_DEBUG "Adding PCI host bridge %s\n", dev->full_name);

	/* Fetch host bridge registers address */
	has_address = (of_address_to_resource(dev, 0, &rsrc) == 0);

	/* Get bus range if any */
	bus_range = of_get_property(dev, "bus-range", &len);
	if (bus_range == NULL || len < 2 * sizeof(int)) {
		printk(KERN_WARNING "Can't get bus-range for %s, assume"
		       " bus 0\n", dev->full_name);
	}

	hose = pcibios_alloc_controller(dev);
	if (!hose)
		return -ENOMEM;
	hose->first_busno = bus_range ? bus_range[0] : 0;
	hose->last_busno = bus_range ? bus_range[1] : 0xff;

	hose->ops = &xenon_pci_ops;

	/* FIXME: should come from config */
	hose->cfg_addr = ioremap(0xd0000000, 0x1000000);

	disp_name = NULL;

	printk(KERN_INFO "Found %s PCI host bridge at 0x%016llx. "
	       "Firmware bus number: %d->%d\n",
		disp_name, (unsigned long long)rsrc.start, hose->first_busno,
		hose->last_busno);

	printk(KERN_DEBUG " ->Hose at 0x%p, cfg_addr=0x%p,cfg_data=0x%p\n",
		hose, hose->cfg_addr, hose->cfg_data);

	/* Interpret the "ranges" property */
	/* This also maps the I/O region and sets isa_io/mem_base */
	pci_process_bridge_OF_ranges(hose, dev, primary);

	/* Fixup "bus-range" OF property */
	// fixup_bus_range(dev);

	return 0;
}

void __init xenon_pci_init(void)
{
	struct device_node *np, *root;

	ppc_pci_set_flags(PPC_PCI_CAN_SKIP_ISA_ALIGN);

	root = of_find_node_by_path("/");
	if (root == NULL) {
		printk(KERN_CRIT "xenon_pci_init: can't find root "
		       "of device tree\n");
		return;
	}
	for (np = NULL; (np = of_get_next_child(root, np)) != NULL;) {
		if (np->name == NULL)
			continue;
		if (strcmp(np->name, "pci") == 0) {
			if (xenon_add_bridge(np) == 0)
				of_node_get(np);
		}
	}
	of_node_put(root);

	/* Setup the linkage between OF nodes and PHBs */
	pci_devs_phb_init();


	/* We can allocate missing resources if any */
	pci_probe_only = 0;
	pci_probe_only = 1;

	/* do we need that? */
	pci_io_base = 0;

	/* do we need that? */
	ppc_md.pci_dma_dev_setup = NULL;
	ppc_md.pci_dma_bus_setup = NULL;

	// Xbox 360 doesn't have IOMMU support. Who made this code?
	// set_pci_dma_ops(&dma_iommu_ops);
}

#endif
