#ifndef XENON_INTERRUPT_H
#define XENON_INTERRUPT_H

extern void xenon_init_irq_on_cpu(int cpu);
extern void __init xenon_iic_init_IRQ(void);
extern void xenon_cause_IPI(int target, int msg);
extern void xenon_request_IPIs(void);

#endif /* ASM_XENON_PIC_H */
