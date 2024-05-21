/* linux/drivers/serial/xenon.c
 *
 * Driver for Xenon XBOX 360 Serial
 *
 * Copyright (C) 2010 Herbert Poetzl
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/console.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/workqueue.h>
#include <linux/delay.h>

#define SERIAL_XENON_NAME	"xenon-uart"
#define SERIAL_XENON_DEV	"ttyS"
#define SERIAL_XENON_MAJOR	4
#define SERIAL_XENON_MINOR	64

#define SERIAL_XENON_BAUD	115200
#define SERIAL_XENON_BITS	8
#define SERIAL_XENON_PARITY	'n'
#define SERIAL_XENON_FLOW	'n'

#if 0
#define dprintk(f, x...) do { printk(KERN_DEBUG f "\n" , ##x); } while (0)
#else
#define dprintk(f, x...)
#endif

static int xenon_status(unsigned char __iomem *membase)
{
	return ioread32be(membase + 0x08);
}

static void xenon_putch(unsigned char __iomem *membase, unsigned char ch)
{
	/* wait for tx fifo ready */
	while (!(xenon_status(membase) & 0x02000000));

	/* put character into fifo */
	iowrite32be((ch << 24) & 0xFF000000, membase + 0x04);
}

static int xenon_getch(unsigned char __iomem *membase)
{
	uint32_t status;

	/* wait for data ready */
	while ((status = xenon_status(membase)) & ~0x03000000);

	if (status & 0x01000000)
		return ioread32be(membase) >> 24;

	return -1;
}

static void xenon_stop_rx(struct uart_port *port)
{
	dprintk("Xenon xenon_stop_rx()");
}

static void xenon_enable_ms(struct uart_port *port)
{
	dprintk("Xenon xenon_enable_ms()");
}

static void xenon_stop_tx(struct uart_port *port)
{
	dprintk("Xenon xenon_stop_tx()");
}

static void xenon_tx_chars(struct uart_port *port)
{
	struct circ_buf *xmit = &port->state->xmit;

	if (port->x_char) {
		xenon_putch(port->membase, port->x_char);
		port->icount.tx++;
		port->x_char = 0;
		return;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		xenon_stop_tx(port);
		return;
	}

#if 0
	count = port->fifosize >> 1;
	do {
		xenon_putch(port->membase, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);
#else
	while (!uart_circ_empty(xmit)) {
		xenon_putch(port->membase, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
	}
#endif

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	if (uart_circ_empty(xmit))
		xenon_stop_tx(port);
}

static void xenon_start_tx(struct uart_port *port)
{
	dprintk("Xenon xenon_start_tx()");
	xenon_tx_chars(port);
}

static void xenon_send_xchar(struct uart_port *port, char ch)
{
	dprintk("Xenon xenon_send_xchar(%d)", ch);
	xenon_putch(port->membase, ch);
}

static unsigned int xenon_tx_empty(struct uart_port *port)
{
	dprintk("Xenon xenon_tx_empty()");
	return 0;
}

static unsigned int xenon_get_mctrl(struct uart_port *port)
{
	dprintk("Xenon xenon_get_mctrl()");
	return 0;
}

static void xenon_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	dprintk("Xenon xenon_set_mctrl()");
}

static void xenon_break_ctl(struct uart_port *port, int break_state)
{
	dprintk("Xenon xenon_break_ctl()");
}

static void xenon_set_termios(struct uart_port *port, struct ktermios *new, const struct ktermios *old)
{
	int baud, quot, cflag = new->c_cflag;

	dprintk("Xenon xenon_set_termios()");
	/* get the byte size */
	switch (cflag & CSIZE) {
		case CS5:
			dprintk(" - data bits = 5");
		break;
		case CS6:
			dprintk(" - data bits = 6");
		break;
		case CS7:
			dprintk(" - data bits = 7");
		break;
		default: // CS8
			dprintk(" - data bits = 8");
		break;
	}

	/* determine the parity */
	if (cflag & PARENB)
		if (cflag & PARODD)
			pr_debug(" - parity = odd\n");
		else
			pr_debug(" - parity = even\n");
		else
			pr_debug(" - parity = none\n");

	/* figure out the stop bits requested */
	if (cflag & CSTOPB)
		pr_debug(" - stop bits = 2\n");
	else
		pr_debug(" - stop bits = 1\n");

	/* figure out the flow control settings */
	if (cflag & CRTSCTS)
		pr_debug(" - RTS/CTS is enabled\n");
	else
		pr_debug(" - RTS/CTS is disabled\n");

	/* Set baud rate */
	baud = uart_get_baud_rate(port, new, old, 0, port->uartclk/16);
	quot = uart_get_divisor(port, baud);
}

// Filthy hack to get around the fact that the xbox 360 SMC doesn't actually have proper interrupts for UART receiving.
#ifdef CONFIG_SERIAL_XENON_UART_POLL
static struct uart_port xenon_port;
static struct workqueue_struct *uart_workqueue;
static struct delayed_work uart_delayed_work;
static atomic_t keep_polling = ATOMIC_INIT(1);

static void xenon_rx_work_handler(struct work_struct *work)
{
	struct uart_port *port;
	int rx;

	if(atomic_read(&keep_polling) == 1) {
		dprintk("Exiting from polling.");
		return;
	}

	port = &xenon_port;
	rx = xenon_getch(port->membase);

	if(rx == -1 || rx == 255)
		goto skip;

	port->icount.rx++;

	if(uart_handle_sysrq_char(port, rx))
		goto skip;

	uart_insert_char(port, port->read_status_mask, 0, rx, TTY_NORMAL);
	tty_flip_buffer_push(&port->state->port);
skip:
	queue_delayed_work(uart_workqueue, &uart_delayed_work, msecs_to_jiffies(CONFIG_SERIAL_XENON_UART_POLL_RATE));
}

static int xenon_uart_setup_polling(void)
{
	uart_workqueue = create_singlethread_workqueue("xenon-uart-rx");
	if(uart_workqueue == NULL) {
		dprintk(KERN_ERR "Couldn't create xenon rx workqueue...");
		return -ENOMEM;
	}

	INIT_DELAYED_WORK(&uart_delayed_work, xenon_rx_work_handler);
	atomic_set(&keep_polling, 0);
	queue_delayed_work(uart_workqueue, &uart_delayed_work, msecs_to_jiffies(CONFIG_SERIAL_XENON_UART_POLL_RATE));
	return 0;
}

static void xenon_uart_shutdown_polling(void)
{
	atomic_set(&keep_polling, 1);
	flush_workqueue(uart_workqueue);
	msleep(500);
	destroy_workqueue(uart_workqueue);
}

#endif

static int xenon_startup(struct uart_port *port)
{
	/* this is the first time this port is opened */
	/* do any hardware initialization needed here */

	int ret = 0;
	dprintk("Xenon xenon_startup()");

#ifdef CONFIG_SERIAL_XENON_UART_POLL
	ret = xenon_uart_setup_polling();
	if(ret) return ret;
#endif

	return ret;
}

static void xenon_shutdown(struct uart_port *port)
{
	/* The port is being closed by the last user. */
	/* Do any hardware specific stuff here */

	dprintk("Xenon xenon_shutdown()");

#ifdef CONFIG_SERIAL_XENON_UART_POLL
	xenon_uart_shutdown_polling();
#endif
}

static const char *xenon_type(struct uart_port *port)
{
	return "Xenon SMC";
}

static void xenon_release_port(struct uart_port *port)
{
	dprintk("Xenon xenon_release_port()");
}

static int xenon_request_port(struct uart_port *port)
{
	dprintk("Xenon xenon_request_port()");
	return 0;
}

static void xenon_config_port(struct uart_port *port, int flags)
{
	dprintk("Xenon xenon_config_port()");

	if (flags & UART_CONFIG_TYPE) {
		port->type = PORT_XENON;
	}
}

static int xenon_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	dprintk("Xenon xenon_verify_port()");
	return 0;
}

#ifdef CONFIG_CONSOLE_POLL

static int xenon_poll_get_char(struct uart_port *port)
{
	return xenon_getch(port->membase);
}

static void xenon_poll_put_char(struct uart_port *port, unsigned char c)
{
	xenon_putch(port->membase, c);
}

#endif

static struct uart_ops xenon_ops = {
	.tx_empty     = xenon_tx_empty,
	.set_mctrl      = xenon_set_mctrl,
	.get_mctrl      = xenon_get_mctrl,
	.stop_tx        = xenon_stop_tx,
	.start_tx       = xenon_start_tx,
	.send_xchar     = xenon_send_xchar,
	.stop_rx        = xenon_stop_rx,
	.enable_ms      = xenon_enable_ms,
	.break_ctl      = xenon_break_ctl,
	.startup        = xenon_startup,
	.shutdown       = xenon_shutdown,
	.set_termios    = xenon_set_termios,
	.type           = xenon_type,
	.release_port   = xenon_release_port,
	.request_port   = xenon_request_port,
	.config_port    = xenon_config_port,
	.verify_port    = xenon_verify_port,
#ifdef CONFIG_CONSOLE_POLL
	.poll_put_char  = xenon_poll_put_char,
	.poll_get_char  = xenon_poll_get_char,
#endif
};

static struct device xenon_device = {
	.init_name = "Xenon UART",
};

static struct uart_port xenon_port = {
	.type           = PORT_XENON,
	.ops            = &xenon_ops,
	.flags          = UPF_FIXED_TYPE | UPF_IOREMAP,
	.mapbase        = 0x200ea001010ULL,
	.iotype         = UPIO_MEM,
	.uartclk        = 1843200,
	.dev		= &xenon_device,
};

static struct console xenon_console;

static struct uart_driver xenon_reg = {
	.owner          = THIS_MODULE,
	.driver_name    = SERIAL_XENON_NAME,
	.dev_name       = SERIAL_XENON_DEV,
	.major          = SERIAL_XENON_MAJOR,
	.minor          = SERIAL_XENON_MINOR,
	.nr             = 1,
#ifdef CONFIG_SERIAL_XENON_CONSOLE
	.cons           = &xenon_console,
	#endif
};

#ifdef CONFIG_SERIAL_XENON_CONSOLE

static void xenon_console_putchar(struct uart_port *port, unsigned char ch)
{
	xenon_putch(port->membase, ch);
}

/*
 * Print a string to the serial port trying not to disturb
 * any possible real use of the port...
 */
static void xenon_console_write(struct console *cons, const char *s, unsigned int count)
{
	uart_console_write(&xenon_port, s, count, xenon_console_putchar);
}

/*
 * Setup serial console baud/bits/parity.  We do two things here:
 * - construct a cflag setting for the first uart_open()
 * - initialise the serial port
 * Return non-zero if we didn't find a serial port.
 */
static int __init xenon_console_setup(struct console *cons, char *options)
{
	int baud	= SERIAL_XENON_BAUD;
	int bits	= SERIAL_XENON_BITS;
	int parity	= SERIAL_XENON_PARITY;
	int flow	= SERIAL_XENON_FLOW;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	
	return uart_set_options(&xenon_port, cons, baud, parity, bits, flow);
}

static struct console xenon_console = {
	.name   = SERIAL_XENON_DEV,
	.write  = xenon_console_write,
	.device = uart_console_device,
	.setup  = xenon_console_setup,
	.flags  = CON_PRINTBUFFER,
	.index  = -1,
	.data   = &xenon_reg,
};

static int __init xenon_serial_console_init(void)
{
	xenon_port.membase = ioremap(xenon_port.mapbase, 0x10);

	register_console(&xenon_console);

	return 0;
}

console_initcall(xenon_serial_console_init);

#endif /* CONFIG_SERIAL_XENON_CONSOLE */

static int __init xenon_init(void)
{
	int result;

	printk(KERN_INFO "Xenon XBOX 360 serial driver\n");

	result = device_register(&xenon_device);
	if(result)
		goto err_uart_device;

	result = uart_register_driver(&xenon_reg);
	if (result)
		goto err_uart_driver;

	xenon_port.membase = ioremap(xenon_port.mapbase, 0x10);

	result = uart_add_one_port(&xenon_reg, &xenon_port);
	if (result)
		goto err_uart_port;

	return result;

err_uart_port:
	dprintk(KERN_ERR "err_uart_port: %d", result);
	uart_remove_one_port(&xenon_reg, &xenon_port);
err_uart_driver:
	dprintk(KERN_ERR "err_uart: %d", result);
	uart_unregister_driver(&xenon_reg);
	device_unregister(&xenon_device); // Hopefully it's okay call before put_device...
err_uart_device:
	dprintk(KERN_ERR "err_device: %d", result);
	put_device(&xenon_device);

	return result;
}

static void __exit xenon_exit(void)
{
	printk(KERN_INFO "Xenon XBOX 360 serial driver exit\n");
	uart_remove_one_port(&xenon_reg, &xenon_port);
	uart_unregister_driver(&xenon_reg);
	put_device(&xenon_device);
	device_unregister(&xenon_device);
}

module_init(xenon_init);
module_exit(xenon_exit);

MODULE_AUTHOR("Herbert Poetzl <herbert@13thfloor.at>");
MODULE_DESCRIPTION("Xenon XBOX 360 Serial port driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS_CHARDEV(SERIAL_XENON_MAJOR, SERIAL_XENON_MINOR);
