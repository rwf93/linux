/*
 *  linux/arch/powerpc/platforms/xenon/xenon_setup.c
 *
 *  Maintained by: Felix Domke <tmbinc@elitedvb.net>
 *  Minor modification by: wolie <wolie@telia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/root_dev.h>
#include <linux/console.h>
#include <linux/kexec.h>

#include <asm/mmu.h>
#include <asm/pci-bridge.h>
#include <asm/ppc-pci.h>
#include <asm/prom.h>
#include <asm/sections.h>

#include "interrupt.h"
#include "pci.h"
#include "smp.h"

#ifdef CONFIG_PPC_EARLY_DEBUG_XENON
void udbg_init_xenon_virtual(void);
#endif

static int xenon_early_init(void)
{
	// xenon_smc_early_led(0x01, 0x60);
	printk("%s\n", __func__);
	return 0;
}
machine_early_initcall(xenon, xenon_early_init);

static void xenon_show_cpuinfo(struct seq_file *m)
{
	struct device_node *root;
	const char *model = "";

	root = of_find_node_by_path("/");
	if (root)
		model = of_get_property(root, "model", NULL);
	seq_printf(m, "machine\t\t: %s\n", model);
	of_node_put(root);
}

static void __init xenon_init_irq(void)
{
	xenon_iic_init_IRQ();
}

static void __init xenon_setup_arch(void)
{
#ifdef CONFIG_SMP
	smp_init_xenon();
#endif
		/* init to some ~sane value until calibrate_delay() runs */
	loops_per_jiffy = 50000000;

	if (ROOT_DEV == 0)
		ROOT_DEV = Root_SDA1;

	xenon_pci_init();
#ifdef CONFIG_DUMMY_CONSOLE
	conswitchp = &dummy_con;
#endif
}

int xenon_smc_message(void *msg);
void xenon_smc_restart(void);

static void xenon_panic(char *str)
{
	// show a red ring
	unsigned char msg[16] = {0x99, 0x01, 0x0F, 0};
	xenon_smc_message(msg);

	smp_send_stop();
	printk("\n");
	printk("   System does not reboot automatically.\n");
	printk("   Please press POWER button.\n");
	printk("\n");

	local_irq_disable();
	while (1);
}

static void __noreturn xenon_restart(char *cmd)
{
	printk("   System restart ... \n");

	smp_send_stop();
	xenon_smc_restart();

	local_irq_disable();
	while (1);
}

void xenon_smc_power_off(void);

static void xenon_power_off(void)
{
	printk("   System power off ... \n");

	smp_send_stop();
	xenon_smc_power_off();

	local_irq_disable();
	while (1);
}

void xenon_smc_halt(void);

static void __noreturn xenon_halt(void)
{
	printk("   System halt ... \n");

	smp_send_stop();
	xenon_smc_halt();

	local_irq_disable();
	while (1);
}

static int __init xenon_probe(void)
{
	if (!of_machine_is_compatible("XENON")) {
		return 0;
	}

#ifdef CONFIG_PPC_EARLY_DEBUG_XENON
	udbg_init_xenon_virtual();
#endif

	hpte_init_native();
	pm_power_off = xenon_power_off;

	return 1;
}

#if 0
static int xenon_check_legacy_ioport(unsigned int baseport)
{
	return -ENODEV;
}
#endif

define_machine(xenon) {
	.name			= "Xenon",
	.probe			= xenon_probe,
	.setup_arch		= xenon_setup_arch,
	.show_cpuinfo	= xenon_show_cpuinfo,
	.calibrate_decr	= generic_calibrate_decr,
	.init_IRQ       = xenon_init_irq,
	.panic			= xenon_panic,
	.restart		= xenon_restart,
	.halt			= xenon_halt,
#if 0 && defined(CONFIG_KEXEC)
	.machine_kexec		= default_machine_kexec,
	.machine_kexec_prepare	= default_machine_kexec_prepare,
	.machine_crash_shutdown	= default_machine_crash_shutdown,
#endif
};
