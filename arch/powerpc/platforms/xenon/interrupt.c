/*
 * Xenon interrupt controller,
 *
 * Maintained by: Felix Domke <tmbinc@elitedvb.net>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License v2
 * as published by the Free Software Foundation.
 */

#include <linux/cpumask.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqnr.h>
#include <linux/module.h>
#include <linux/percpu.h>
#include <linux/types.h>
#include <linux/ioport.h>
#include <linux/irqdesc.h>

#ifdef CONFIG_SMP
#include <linux/smp.h>
#endif

#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/prom.h>
#include <asm/ptrace.h>
#include <asm/machdep.h>
#include <asm/udbg.h>

#include "interrupt.h"

#define DEBUG
#if defined(DEBUG)
#define DBG udbg_printf
#else
#define DBG pr_debug
#endif

static void *iic_base, // 00050000
	*bridge_base, // ea000000
	*biu,         // e1000000
	*graphics;    // ec800000
static struct irq_domain *host;

#define XENON_NR_IRQS 128

#define PRIO_IPI_4       0x08
#define PRIO_IPI_3       0x10
#define PRIO_SMM         0x14
#define PRIO_SFCX        0x18
#define PRIO_SATA_HDD    0x20
#define PRIO_SATA_CDROM  0x24
#define PRIO_OHCI_0      0x2C
#define PRIO_EHCI_0      0x30
#define PRIO_OHCI_1      0x34
#define PRIO_EHCI_1      0x38
#define PRIO_XMA         0x40
#define PRIO_AUDIO       0x44
#define PRIO_ENET        0x4C
#define PRIO_XPS         0x54
#define PRIO_GRAPHICS    0x58
#define PRIO_PROFILER    0x60
#define PRIO_BIU         0x64
#define PRIO_IOC         0x68
#define PRIO_FSB         0x6C
#define PRIO_IPI_2       0x70
#define PRIO_CLOCK       0x74
#define PRIO_IPI_1       0x78
#define PRIO_NONE		 0x7C

/*
 * Important interrupt registers (per CPU):
 *
 * 0x00: CPU_WHOAMI
 * 0x08: CPU_CURRENT_TASK_PRI: only receive interrupts higher than this
 * 0x10: CPU_IPI_DISPATCH_0
 * 0x18: unused(?)
 * 0x20: ? (read changes)
 * 0x28: ? (read changes)
 * 0x30: ? (read changes)
 * 0x38: same value as 0x20(?) (read changes)
 * 0x40: unused(?)
 * 0x48: unused(?)
 * 0x50: ACK
 * 0x58: ACK + set CPU_CURRENT_TASK_PRI
 * 0x60: EOI
 * 0x68: EOI + set CPU_CURRENT_TASK_PRI
 * 0x70: interrupt MCACK? mask? max interrupt? CPU dies when writing stuff that isn't 0x7C
 * 0xF0: ?
 */

/* CPU IRQ -> bridge (PCI) IRQ */
static const uint8_t xenon_pci_irq_map[] = {
	/* 0x00             */ -1,
	/* 0x04             */ -1,
	/* PRIO_IPI_4       */ -1,
	/* 0x0C             */ -1,
	/* PRIO_IPI_3       */ -1,
	/* PRIO_SMM         */ 3,
	/* PRIO_SFCX        */ 13,
	/* 0x1C             */ -1,
	/* PRIO_SATA_HDD    */ 2,
	/* PRIO_SATA_CDROM  */ 1,
	/* 0x28             */ -1,
	/* PRIO_OHCI_0      */ 4,
	/* PRIO_EHCI_0      */ 5,
	/* PRIO_OHCI_1      */ 6,
	/* PRIO_EHCI_1      */ 7,
	/* 0x3C             */ -1,
	/* PRIO_XMA         */ 11,
	/* PRIO_AUDIO       */ 12,
	/* 0x48             */ -1,
	/* PRIO_ENET        */ 10,
	/* 0x50             */ -1,
	/* PRIO_XPS         */ -1,
	/* PRIO_GRAPHICS    */ -1,
	/* 0x5C             */ -1,
	/* PRIO_PROFILER    */ -1,
	/* PRIO_BIU         */ -1,
	/* PRIO_IOC         */ -1,
	/* PRIO_FSB         */ -1,
	/* PRIO_IPI_2       */ -1,
	/* PRIO_CLOCK       */ 0,
	/* PRIO_IPI_1       */ -1,
	/* PRIO_NONE        */ -1,
};

static void disconnect_pci_irq(int prio)
{
	int i;

	printk(KERN_INFO "xenon IIC: disconnect irq 0x%.2X\n", prio);

	i = xenon_pci_irq_map[prio >> 2];
	if (i != -1) {
		writel(0, bridge_base + 0x10 + i * 4);
	}
}

/* connects a PCI IRQ to a CPU */
static void connect_pci_irq(int prio, int target_cpu)
{
	uint8_t i;

	printk(KERN_INFO "xenon IIC: connect irq 0x%.2X\n", prio);

	i = xenon_pci_irq_map[prio >> 2];
	if (i != -1) {
		/*
		 * Bits:
		 * 0x00800000 = enable(?)
		 * 0x00200000 = latched
		 * 0x00003F00 = cpu target
		 * 0x00000080 = level sensitive
		 * 0x0000007F = CPU IRQ
		 */
		uint32_t bits = 0x00800080;
		bits |= ((1 << target_cpu) & 0xFF) << 8;
		bits |= (prio >> 2) & 0x3F;

		writel(bits, bridge_base + 0x10 + i * 4);
	}
}

static void iic_mask(struct irq_data *data)
{
	disconnect_pci_irq(data->hwirq);
}

static void iic_unmask(struct irq_data *data)
{
	connect_pci_irq(data->hwirq, 0);
}

static void xenon_ipi_send_mask(struct irq_data *data,
				const struct cpumask *dest)
{
	int cpu = hard_smp_processor_id();
	out_be64(iic_base + cpu * 0x1000 + 0x10,
		 ((cpumask_bits(dest)[0] << 16) & 0x3F) | (data->hwirq & 0x7C));
}

static struct irq_chip xenon_pic = {
	.name = " XENON-PIC ",
	.irq_mask = iic_mask,
	.irq_unmask = iic_unmask,
	.ipi_send_mask = xenon_ipi_send_mask,
	.flags = 0,
};

void xenon_init_irq_on_cpu(int cpu)
{
	printk(KERN_INFO "xenon IIC: init on cpu %i\n", cpu);

	/* init that cpu's interrupt controller */
	out_be64(iic_base + cpu * 0x1000 + 0x70, 0x7C);
	out_be64(iic_base + cpu * 0x1000 + 0x08, 0); /* Set priority to 0 */
	out_be64(iic_base + cpu * 0x1000, 1 << cpu); /* "who am i" */

	/* read in and ack all outstanding interrupts */
	while (in_be64(iic_base + cpu * 0x1000 + 0x50) != PRIO_NONE);
	out_be64(iic_base + cpu * 0x1000 + 0x68, 0); /* EOI and set priority to 0 */
}

/* Get an IRQ number from the pending state register of the IIC */
static unsigned int iic_get_irq(void)
{
	int cpu = hard_smp_processor_id();
	void *my_iic_base;
	int index;

	my_iic_base = iic_base + cpu * 0x1000;

	/* read destructive pending interrupt */
	index = in_be64(my_iic_base + 0x50) & 0x7C;
	out_be64(my_iic_base + 0x60, 0x0); /* EOI this interrupt. */

	/* HACK: we will handle some (otherwise unhandled) interrupts here
	   to prevent them flooding. */
	switch (index) {
	case PRIO_GRAPHICS:
		writel(0, graphics + 0xed0);  // RBBM_INT_CNTL
		writel(0, graphics + 0x6540); // R500_DxMODE_INT_MASK
		break;
	case PRIO_IOC:
		writel(0, biu + 0x4002c);
		break;
	case PRIO_CLOCK:
		writel(0, bridge_base + 0x106C);
		break;
	default:
		break;
	}

	/* No interrupt. This really shouldn't happen. */
	if (index == PRIO_NONE) {
		return NO_IRQ;
	}

	return irq_linear_revmap(host, index);
}

static int xenon_irq_host_map(struct irq_domain *h, unsigned int virq,
				irq_hw_number_t hw)
{
	irq_set_chip_and_handler(virq, &xenon_pic, handle_percpu_irq);
	return 0;
}

static int xenon_irq_host_match(struct irq_domain *h, struct device_node *node,
				enum irq_domain_bus_token bus_token)
{
	return h->host_data != NULL && node == h->host_data;
}

static const struct irq_domain_ops xenon_irq_host_ops = {
	.map = xenon_irq_host_map,
	.match = xenon_irq_host_match,
};

void __init xenon_iic_init_IRQ(void)
{
	int i;
	struct device_node *dn;
	struct resource res;

	printk(KERN_DEBUG "xenon IIC: init\n");
			/* search for our interrupt controller inside the device tree */
	for (dn = NULL;
	     (dn = of_find_node_by_name(dn, "interrupt-controller")) != NULL;) {
		if (!of_device_is_compatible(dn, "xenon"))
			continue;

		if (of_address_to_resource(dn, 0, &res))
		{
			printk(KERN_WARNING "xenon IIC: Can't resolve addresses\n");
			of_node_put(dn);
			return;
		}

		iic_base = ioremap(res.start, 0x10000);

		host = irq_domain_add_linear(NULL, XENON_NR_IRQS, &xenon_irq_host_ops, NULL);
		host->host_data = of_node_get(dn);
		BUG_ON(host == NULL);
		irq_set_default_host(host);
	}

	ppc_md.get_irq = iic_get_irq;

	bridge_base = ioremap(0xea000000, 0x10000);
	biu = ioremap(0xe1000000, 0x2000000);
	graphics = ioremap(0xec800000, 0x10000);

		/* initialize interrupts */
	writel(0, bridge_base);
	writel(0x40000000, bridge_base + 4);

	writel(0x40000000, biu + 0x40074);
	writel(0xea000050, biu + 0x40078);

	writel(0, bridge_base + 0xc);  /* Interrupt mask register */
	writel(0x3, bridge_base);

		/* disconnect all PCI IRQs until they are requested */
	for (i=0; i<0x10; ++i)
		writel(0, bridge_base + 0x10 + i * 4);

	xenon_init_irq_on_cpu(0);
}

#ifdef CONFIG_SMP

static int ipi_to_prio(int ipi)
{
	switch (ipi) {
	case PPC_MSG_NMI_IPI:
		return PRIO_IPI_1;
		break;
	case PPC_MSG_CALL_FUNCTION:
		return PRIO_IPI_2;
		break;
	case PPC_MSG_RESCHEDULE:
		return PRIO_IPI_3;
		break;
	case PPC_MSG_TICK_BROADCAST:
		return PRIO_IPI_4;
		break;
	default:
		printk("unhandled ipi %d\n", ipi);
		BUG();
	}
	return 0;
}

void xenon_cause_IPI(int target_cpu, int msg)
{
	int ipi_prio;
	int cpu = hard_smp_processor_id();

	ipi_prio = ipi_to_prio(msg);
	out_be64(iic_base + cpu * 0x1000 + 0x10, (0x10000 << target_cpu) | ipi_prio);
}

static void xenon_request_ipi(int ipi)
{
	int virq;
	int prio = ipi_to_prio(ipi);

	virq = irq_create_mapping(host, prio);
	if (virq == NO_IRQ) {
		printk(KERN_ERR "xenon_request_ipi: failed to map IPI%d (%s)\n",
		       prio, smp_ipi_name[ipi]);
		return;
	}

	if (smp_request_message_ipi(virq, ipi))
		irq_dispose_mapping(virq);
}

void xenon_request_IPIs(void)
{
	xenon_request_ipi(PPC_MSG_TICK_BROADCAST);
	xenon_request_ipi(PPC_MSG_CALL_FUNCTION);
	xenon_request_ipi(PPC_MSG_RESCHEDULE);
	xenon_request_ipi(PPC_MSG_NMI_IPI);
}

#endif
