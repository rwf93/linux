/*
 * SMP support for Xenon machines.
 *
 * Based on CBE's smp.c.
 *
 *      This program is free software; you can redistribute it and/or
 *      modify it under the terms of the GNU General Public License
 *      as published by the Free Software Foundation; either version
 *      2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/cpu.h>
#include <linux/module.h>

#include <asm/machdep.h>
#include <asm/smp.h>
#include <asm/udbg.h>

#include "interrupt.h"

#define DEBUG
#if defined(DEBUG)
#define DBG udbg_printf
#else
#define DBG pr_debug
#endif

extern int boot_cpuid;

static void __init smp_xenon_probe(void)
{
	xenon_request_IPIs();
}

static void smp_xenon_setup_cpu(int cpu)
{
	/* Boot CPU IRQs are already initialized at this point in
	 * xenon_iic_init_IRQ */
	if (cpu != boot_cpuid)
		xenon_init_irq_on_cpu(cpu);
}

static int smp_xenon_cpu_bootable(unsigned int nr)
{
	/* Special case - we inhibit secondary thread startup
	 * during boot if the user requests it.  Odd-numbered
	 * cpus are assumed to be secondary threads.
	 */
	if (system_state < SYSTEM_RUNNING && cpu_has_feature(CPU_FTR_SMT)
	    && !smt_enabled_at_boot && nr % 2 != 0)
		return 0;

	return 1;
}

static void smp_xenon_message_pass(int cpu, int msg)
{
	xenon_cause_IPI(cpu, msg);
}

static struct smp_ops_t xenon_smp_ops = {
	.probe = smp_xenon_probe,
	.message_pass = smp_xenon_message_pass,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_die = generic_cpu_die,
#endif
	.kick_cpu = smp_generic_kick_cpu,
	.setup_cpu = smp_xenon_setup_cpu,
	.cpu_bootable = smp_xenon_cpu_bootable,
};

/* This is called very early */
void __init smp_init_xenon(void)
{
	pr_debug(" -> smp_init_xenon()\n");

	smp_ops = &xenon_smp_ops;

	pr_debug(" <- smp_init_xenon()\n");
}
