/*
 * xenon_net.c: Driver for Xenon Southbridge Fast Ethernet
 *
 * Copyright 2007 Felix Domke <tmbinc@elitedvb.net>
 * Minor modification by: wolie <wolie@telia.com>
 * More modifications by: Justin Moore
 *
 * Licensed under the GPL v2.
 */

// Documentation for NAPI can be found at:
// https://wiki.linuxfoundation.org/networking/napi

#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/pci.h>

#include <asm/io.h>
#include <asm/udbg.h>

#define DEBUG
#if defined(DEBUG)
#define DBG udbg_printf
#else
#define DBG pr_debug
#endif

#define XENONNET_VERSION "1.0.2"
#define MODNAME "xenon_net"
#define XENONNET_DRIVER_LOAD_MSG                                               \
"Xenon Fast Ethernet driver " XENONNET_VERSION " loaded"
#define PFX MODNAME ": "

#define RX_RING_SIZE 16
#define TX_RING_SIZE 16

#define TX_TIMEOUT (6 * HZ)

static char version[] = KERN_INFO XENONNET_DRIVER_LOAD_MSG "\n";

static struct pci_device_id xenon_net_pci_tbl[] = {
	{ PCI_VENDOR_ID_MICROSOFT, 0x580a, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ 0, }
};

MODULE_DEVICE_TABLE(pci, xenon_net_pci_tbl);

/* Symbolic offsets to registers. */
enum XENONNET_registers {
	TX_CONFIG = 0x00,
	TX_DESCRIPTOR_BASE = 0x04,
	TX_DESCRIPTOR_STATUS = 0x0C,
	RX_CONFIG = 0x10,
	RX_DESCRIPTOR_BASE = 0x14,
	INTERRUPT_STATUS = 0x20,
	INTERRUPT_MASK = 0x24,
	CONFIG_0 = 0x28,
	POWER = 0x30,
	PHY_CONFIG = 0x40,
	PHY_CONTROL = 0x44,
	CONFIG_1 = 0x50,
	RETRY_COUNT = 0x54,
	MULTICAST_FILTER_CONTROL = 0x60,
	ADDRESS_0 = 0x62,
	MULTICAST_HASH = 0x68,
	MAX_PACKET_SIZE = 0x78,
	ADDRESS_1 = 0x7A
};

struct xenon_net_private {
	void *mmio_addr;

	struct net_device *dev2;
	struct napi_struct napi;

	struct pci_dev *pdev;
	struct net_device_stats stats;

	/* we maintain a list of rx and tx descriptors */
	void *tx_descriptor_base;
	void *rx_descriptor_base;
	dma_addr_t tx_descriptor_base_dma;
	dma_addr_t rx_descriptor_base_dma;

	struct sk_buff *rx_skbuff[RX_RING_SIZE];
	dma_addr_t rx_skbuff_dma[RX_RING_SIZE];

	struct sk_buff *tx_skbuff[TX_RING_SIZE];
	dma_addr_t tx_skbuff_dma[TX_RING_SIZE];

	atomic_t tx_next_free, tx_next_done;

	int rx_buf_sz, rx_next;

	spinlock_t lock;
};

static void xenon_set_tx_descriptor(struct xenon_net_private *tp, int index,
									u32 len, dma_addr_t addr, int valid)
{
	volatile u32 *descr = tp->tx_descriptor_base + index * 0x10;

	descr[0] = cpu_to_le32(len);
	descr[2] = cpu_to_le32(addr);
	descr[3] = cpu_to_le32(len | ((index == TX_RING_SIZE - 1) ? 0x80000000 : 0));

	wmb();
	if (valid)
		descr[1] = cpu_to_le32(0xc0230000);
	else
		descr[1] = 0;
}

static void xenon_set_rx_descriptor(struct xenon_net_private *tp, int index,
									u32 len, dma_addr_t addr, int valid)
{
	volatile u32 *descr = tp->rx_descriptor_base + index * 0x10;
	descr[0] = cpu_to_le32(0);
	descr[2] = cpu_to_le32(addr);
	descr[3] = cpu_to_le32(len |
						((index == RX_RING_SIZE - 1) ? 0x80000000 : 0));
	wmb();
	if (valid)
		descr[1] = cpu_to_le32(0xc0000000);
	else
		descr[1] = 0;
}

static int xenon_net_tx_interrupt(struct net_device *dev,
								  struct xenon_net_private *tp, void *ioaddr)
{
	int work = 0;

	BUG_ON(dev == NULL);
	BUG_ON(tp == NULL);
	BUG_ON(ioaddr == NULL);

	while (atomic_read(&tp->tx_next_free) !=
		atomic_read(&tp->tx_next_done)) {
		int e = atomic_read(&tp->tx_next_done) % TX_RING_SIZE;

		volatile u32 *descr = tp->tx_descriptor_base + e * 0x10;
		if (le32_to_cpu(descr[1]) & 0x80000000)
		break;

		if (!tp->tx_skbuff[e]) {
			printk(KERN_WARNING "spurious TX complete?!\n");
			break;
		}

		work++;
		pci_unmap_single(tp->pdev, tp->tx_skbuff_dma[e],
				   tp->tx_skbuff[e]->len, PCI_DMA_TODEVICE);
		dev_kfree_skb_irq(tp->tx_skbuff[e]);

		tp->tx_skbuff[e] = 0;
		tp->tx_skbuff_dma[e] = 0;

		atomic_inc(&tp->tx_next_done);
	}

	if ((atomic_read(&tp->tx_next_free) - atomic_read(&tp->tx_next_done)) <
		TX_RING_SIZE)
		netif_start_queue(dev);

	return work;
}

static int xenon_net_rx_interrupt(struct net_device *dev,
								  struct xenon_net_private *tp, void *ioaddr,
								  int budget)
{
	int received = 0; // count and send to work_done

	BUG_ON(dev == NULL);
	BUG_ON(tp == NULL);
	BUG_ON(ioaddr == NULL);
	BUG_ON(tp->rx_descriptor_base == NULL);

	while (1) {
		u32 size;
		dma_addr_t mapping;
		int index = tp->rx_next;
		volatile u32 *descr = tp->rx_descriptor_base + index * 0x10;
		struct sk_buff *skb = tp->rx_skbuff[index], *new_skb;

		// no data available
		if (le32_to_cpu(descr[1]) & 0x80000000)
		break;

		if (received >= budget)
		break; /* Over-budget. */

		new_skb = netdev_alloc_skb(dev, tp->rx_buf_sz);
		if (!new_skb)
		break; /* Failed to alloc memory, we'll break off and try again later. */

		size = le32_to_cpu(descr[0]) & 0xFFFF;
		mapping = tp->rx_skbuff_dma[index];
		pci_unmap_single(tp->pdev, mapping, tp->rx_buf_sz,
				   PCI_DMA_FROMDEVICE);

		skb->ip_summed = CHECKSUM_NONE;
		skb_put(skb, size);
		skb->protocol = eth_type_trans(skb, dev);

		netif_receive_skb(skb);
		received++;

		mapping = tp->rx_skbuff_dma[index] =
			pci_map_single(tp->pdev, new_skb->data, tp->rx_buf_sz,
				  PCI_DMA_FROMDEVICE);
		tp->rx_skbuff[index] = new_skb;

		xenon_set_rx_descriptor(tp, index, tp->rx_buf_sz,
						  tp->rx_skbuff_dma[index], 1);

		tp->rx_next = (tp->rx_next + 1) % RX_RING_SIZE;
	}

	if (received > 0) {
		/* enable RX */
		writel(0x00101c11, ioaddr + RX_CONFIG);
	}

	return received;
}

static int xenon_net_poll(struct napi_struct *napi, int budget)
{
	int work_done;
	struct xenon_net_private *tp;
	struct net_device *dev;

	tp = container_of(napi, struct xenon_net_private, napi);
	BUG_ON(tp == NULL);

	dev = tp->dev2;
	BUG_ON(dev == NULL);

	work_done = xenon_net_rx_interrupt(dev, tp, tp->mmio_addr, budget);

	/* Process the transmission ring */
	if (xenon_net_tx_interrupt(dev, tp, tp->mmio_addr) == TX_RING_SIZE) {
		/* We've processed the entire ring, so count it against the rest of the budget. */
		work_done = max(work_done, budget);
	}

	if (work_done < budget) {
		/* Re-enable network interrupts. */
		writel(0x00010044, tp->mmio_addr + INTERRUPT_MASK);
		napi_complete(napi);
	}

	return work_done;
}

static irqreturn_t xenon_net_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct xenon_net_private *tp = netdev_priv(dev);
	void *ioaddr = tp->mmio_addr;
	u32 status;

	spin_lock(&tp->lock);
	status = readl(ioaddr + INTERRUPT_STATUS);

	/* tx or rx interrupt */
	if (status & 0x0000004C) {
		/* Disable interrupts. */
		writel(0, ioaddr + INTERRUPT_MASK);

		if (napi_schedule_prep(&tp->napi)) {
			__napi_schedule(&tp->napi);
		}
	}

	spin_unlock(&tp->lock);
	return IRQ_HANDLED;
}

#if defined(CONFIG_NET_POLL_CONTROLLER)
/*
 * Polling receive - used by netconsole and other diagnostic tools
 * to allow network i/o with interrupts disabled.
 */
static void xenon_net_poll_controller(struct net_device *dev)
{
	struct xenon_net_private *tp = netdev_priv(dev);
	napi_schedule(&tp->napi);
}
#endif

/* Initialize the Rx and Tx rings, along with various 'dev' bits. */
static void xenon_net_init_ring(struct net_device *dev)
{
	struct xenon_net_private *tp = netdev_priv(dev);
	int i;

	tp->rx_next = 0;
	atomic_set(&tp->tx_next_done, 0);
	atomic_set(&tp->tx_next_free, 0);

	for (i = 0; i < TX_RING_SIZE; i++) {
		tp->tx_skbuff[i] = NULL;
		tp->tx_skbuff_dma[i] = 0;
	}

	/* allocate descriptor memory */
	tp->pdev->dev.dma_ops_bypass = true;
	tp->tx_descriptor_base =
		pci_alloc_consistent(tp->pdev,
					   TX_RING_SIZE * 0x10 + RX_RING_SIZE * 0x10,
					   &tp->tx_descriptor_base_dma);

	BUG_ON(tp->tx_descriptor_base <= 0);

	/* rx is right after tx */
	tp->rx_descriptor_base = tp->tx_descriptor_base + TX_RING_SIZE * 0x10;
	tp->rx_descriptor_base_dma = tp->tx_descriptor_base_dma + TX_RING_SIZE * 0x10;

	for (i = 0; i < TX_RING_SIZE; ++i)
		xenon_set_tx_descriptor(tp, i, 0, 0, 0);

	tp->rx_buf_sz = dev->mtu + 32;

	for (i = 0; i < RX_RING_SIZE; ++i) {
		struct sk_buff *skb = dev_alloc_skb(tp->rx_buf_sz);
		tp->rx_skbuff[i] = skb;
		if (skb == NULL)
		break;

		skb->dev = dev; /* Mark as being used by this device. */
		tp->rx_skbuff_dma[i] = pci_map_single(
			tp->pdev, skb->data, tp->rx_buf_sz, PCI_DMA_FROMDEVICE);

		xenon_set_rx_descriptor(tp, i, tp->rx_buf_sz,
						  tp->rx_skbuff_dma[i], 1);
	}
}

static int xenon_net_set_mac(struct net_device *dev, void *p)
{
	struct xenon_net_private *tp = netdev_priv(dev);
	void *ioaddr = tp->mmio_addr;
	struct sockaddr *addr = p;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	/* Write the MAC */
	writew(cpu_to_le16(*(u16 *)(addr->sa_data)), ioaddr + ADDRESS_0);
	writel(cpu_to_le32(*(u32 *)(addr->sa_data + 2)),
		ioaddr + ADDRESS_0 + 2);

	writew(cpu_to_le16(*(u16 *)(addr->sa_data)), ioaddr + ADDRESS_1);
	writel(cpu_to_le32(*(u32 *)(addr->sa_data + 2)),
		ioaddr + ADDRESS_1 + 2);

	memcpy((void *)dev->dev_addr, addr->sa_data, ETH_ALEN);
	memcpy((void *)dev->dev_addr_shadow, addr->sa_data, ETH_ALEN);

	return 0;
}

/* Start the hardware at open or resume. */
static void xenon_net_hw_start(struct net_device *dev)
{
	struct xenon_net_private *tp = netdev_priv(dev);
	void *ioaddr = tp->mmio_addr;

	/* Soft reset the chip. */
	writel(0, ioaddr + INTERRUPT_MASK);
	writel(0x08558001, ioaddr + CONFIG_0);
	udelay(100);
	writel(0x08550001, ioaddr + CONFIG_0);

	writel(4, ioaddr + PHY_CONTROL);
	udelay(100);
	writel(0, ioaddr + PHY_CONTROL);

	writew(1522, ioaddr + MAX_PACKET_SIZE);

	writel(0x2360, ioaddr + CONFIG_1);

	writew(0x0e38, ioaddr + MULTICAST_FILTER_CONTROL);

	/* Restore our idea of the MAC address. */
	writew(cpu_to_le16(*(u16 *)(dev->dev_addr + 0)), ioaddr + ADDRESS_0);
	writel(cpu_to_le32(*(u32 *)(dev->dev_addr + 2)),
		ioaddr + ADDRESS_0 + 2);

	writew(cpu_to_le16(*(u16 *)(dev->dev_addr + 0)), ioaddr + ADDRESS_1);
	writel(cpu_to_le32(*(u32 *)(dev->dev_addr + 2)),
		ioaddr + ADDRESS_1 + 2);

	writel(0, ioaddr + MULTICAST_HASH);
	writel(0, ioaddr + MULTICAST_HASH + 4);

	writel(0x00001c00, ioaddr + TX_CONFIG);
	writel(0x00101c00, ioaddr + RX_CONFIG);

	writel(0x04001901, ioaddr + PHY_CONFIG);

	tp->rx_next = 0;

	/* write base 0 */
	writel(0x00001c00, ioaddr + TX_CONFIG);
	writel(tp->tx_descriptor_base_dma, ioaddr + TX_DESCRIPTOR_BASE);

	/* write base 1 */
	writel(0x00011c00, ioaddr + TX_CONFIG);
	writel(tp->tx_descriptor_base_dma, ioaddr + TX_DESCRIPTOR_BASE);
	writel(0x00001c00, ioaddr + TX_CONFIG);

	writel(tp->rx_descriptor_base_dma, ioaddr + RX_DESCRIPTOR_BASE);
	writel(0x04001001, ioaddr + PHY_CONFIG);
	writel(0, ioaddr + CONFIG_1);

	writel(0x08550001, ioaddr + CONFIG_0);

	writel(0x00001c01, ioaddr + TX_CONFIG); /* enable tx */
	writel(0x00101c11, ioaddr + RX_CONFIG); /* enable rx */

	// Interrupts:
	// 0x20000000 = ??
	// 0x00010000 = multicast related
	// 0x00000040 = rx
	// 0x00000008 = tx (1)
	// 0x00000004 = tx (0)
	writel(0x00010044, ioaddr + INTERRUPT_MASK);
	writel(0x00010044, ioaddr + INTERRUPT_STATUS);

	netif_start_queue(dev);
}

static int xenon_net_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct xenon_net_private *tp = netdev_priv(dev);
	void *ioaddr = tp->mmio_addr;
	int entry;
	dma_addr_t mapping;
	u32 len;
	unsigned long flags;

	spin_lock_irqsave(&tp->lock, flags);

	/* Calculate the next Tx descriptor entry. */
	entry = atomic_read(&tp->tx_next_free) % TX_RING_SIZE;

	BUG_ON(tp->tx_skbuff[entry] != NULL);
	BUG_ON(tp->tx_skbuff_dma[entry] != 0);
	BUG_ON(skb_shinfo(skb)->nr_frags != 0);

	tp->tx_skbuff[entry] = skb;

	len = skb->len;

	mapping = pci_map_single(tp->pdev, skb->data, len, PCI_DMA_TODEVICE);
	tp->tx_skbuff_dma[entry] = mapping;

	xenon_set_tx_descriptor(tp, entry, skb->len, mapping, 1);

	atomic_inc(&tp->tx_next_free);
	if ((atomic_read(&tp->tx_next_free) - atomic_read(&tp->tx_next_done)) >=
		TX_RING_SIZE) {
		netif_stop_queue(dev);
	}

	writel(0x00101c11, ioaddr + TX_CONFIG); /* enable TX */

	spin_unlock_irqrestore(&tp->lock, flags);

	return NETDEV_TX_OK;
}

static void xenon_net_tx_clear(struct xenon_net_private *tp)
{
	int i;

	atomic_set(&tp->tx_next_free, 0);
	atomic_set(&tp->tx_next_done, 0);

	/* Dump the unsent Tx packets. */
	for (i = 0; i < TX_RING_SIZE; i++) {
		if (tp->tx_skbuff_dma[i] != 0) {
			pci_unmap_single(tp->pdev, tp->tx_skbuff_dma[i],
					tp->tx_skbuff[i]->len,
					PCI_DMA_TODEVICE);
		}
		if (tp->tx_skbuff[i]) {
			dev_kfree_skb(tp->tx_skbuff[i]);
			tp->tx_skbuff[i] = NULL;
			tp->stats.tx_dropped++;
		}
	}
}

static void xenon_net_tx_timeout(struct net_device *dev, unsigned int txqueue)
{
	/* Error handling was taken from eexpress.c */
	struct xenon_net_private *tp = netdev_priv(dev);
	void *ioaddr = tp->mmio_addr;
	unsigned long flags;

	printk(KERN_INFO "%s: transmit timed out, resetting.\n", dev->name);
	writel(0, ioaddr + INTERRUPT_MASK);
	disable_irq(dev->irq);

	/* Stop a shared interrupt from scavenging while we are. */
	spin_lock_irqsave(&tp->lock, flags);
	xenon_net_tx_clear(tp);
	xenon_net_hw_start(dev); // this will re-enable interrupts
	spin_unlock_irqrestore(&tp->lock, flags);
	enable_irq(dev->irq);

	tp->stats.tx_errors++;
	netif_wake_queue(dev);
}

static int xenon_net_open(struct net_device *dev)
{
	int retval;

	struct xenon_net_private *tp = netdev_priv(dev);

	retval = request_irq(dev->irq, xenon_net_interrupt, IRQF_SHARED,
					  dev->name, dev);

	if (retval) {
		printk(KERN_ERR "Unable to create IRQ for xenon_net_open.\n");
		return retval;
	}

	xenon_net_init_ring(dev); /* allocates ringbuffer, clears them */
	xenon_net_hw_start(dev); /* start HW */
	napi_enable(&tp->napi);
	netif_carrier_on(dev);

	return 0;
}

static int xenon_net_close(struct net_device *dev)
{
	struct xenon_net_private *tp = netdev_priv(dev);
	void *ioaddr = tp->mmio_addr;

	// Disable interrupts
	writel(0, ioaddr + INTERRUPT_MASK);
	disable_irq(dev->irq);

	netif_carrier_off(dev);
	netif_stop_queue(dev);
	napi_disable(&tp->napi);
	free_irq(dev->irq, dev);
	xenon_net_tx_clear(tp);


	pci_free_consistent(tp->pdev, TX_RING_SIZE * 0x10 + RX_RING_SIZE * 0x10,
					 tp->tx_descriptor_base, tp->tx_descriptor_base_dma);
	// dma_free_coherent(&tp->pdev->dev, TX_RING_SIZE * 0x10 + RX_RING_SIZE * 0x10,
	// 				 tp->tx_descriptor_base, tp->tx_descriptor_base_dma);


	// BUG: Race condition with xenon_net_poll (napi_disable should stop it?)
	tp->tx_descriptor_base = NULL;
	tp->rx_descriptor_base = NULL;

	return 0;
}

static struct net_device_ops xenon_netdev_ops = {
	.ndo_open = xenon_net_open,
	.ndo_stop = xenon_net_close,
	.ndo_start_xmit = xenon_net_start_xmit,
	.ndo_tx_timeout = xenon_net_tx_timeout,
	.ndo_set_mac_address = xenon_net_set_mac,
	.ndo_validate_addr = eth_validate_addr,
	//	.ndo_set_multicast_list = xenon_net_set_multicast_list,
	//  .ndo_set_mac_address 	= eth_mac_addr,
	//	.ndo_change_mtu 		= xenon_net_change_mtu,
#if defined(CONFIG_NET_POLL_CONTROLLER)
	.ndo_poll_controller = xenon_net_poll_controller,
	#endif
};

static int xenon_net_init_board(struct pci_dev *pdev,
								struct net_device **dev_out, void **ioaddr_out)
{
	void *ioaddr = NULL;
	struct net_device *dev;
	struct xenon_net_private *tp;
	int rc, i;
	unsigned long mmio_start, mmio_end, mmio_flags, mmio_len;

	BUG_ON(pdev == NULL);
	BUG_ON(ioaddr_out == NULL);

	*ioaddr_out = NULL;
	*dev_out = NULL;

	/* dev zeroed in alloc_etherdev */
	dev = alloc_etherdev(sizeof(*tp));
	if (dev == NULL) {
		dev_err(&pdev->dev, "unable to alloc new ethernet\n");
		return -ENOMEM;
	}
	SET_NETDEV_DEV(dev, &pdev->dev);
	tp = netdev_priv(dev);

	/* enable device (incl. PCI PM wakeup), and bus-mastering */
	rc = pci_enable_device(pdev);
	if (rc)
	goto err_out;

	mmio_start = pci_resource_start(pdev, 0);
	mmio_end = pci_resource_end(pdev, 0);
	mmio_flags = pci_resource_flags(pdev, 0);
	mmio_len = pci_resource_len(pdev, 0);

	/* make sure PCI base addr 0 is MMIO */
	if (!(mmio_flags & IORESOURCE_MEM)) {
		dev_err(&pdev->dev,
		  "region #0 not an MMIO resource, aborting\n");
		rc = -ENODEV;
	goto err_out;
	}

	rc = pci_request_regions(pdev, MODNAME);
	if (rc)
	goto err_out;

	pci_set_master(pdev);

	/* ioremap MMIO region */
	ioaddr = ioremap(mmio_start, mmio_len);
	if (ioaddr == NULL) {
		dev_err(&pdev->dev, "cannot remap MMIO, aborting\n");
		rc = -EIO;
	goto err_out_free_res;
	}

	dev->netdev_ops = &xenon_netdev_ops;
	i = register_netdev(dev);
	if (i)
	goto err_out_unmap;

	*ioaddr_out = ioaddr;
	*dev_out = dev;
	return 0;

err_out_unmap:
#ifndef USE_IO_OPS
	iounmap(ioaddr);
err_out_free_res:
#endif
	pci_release_regions(pdev);
err_out:
	free_netdev(dev);
	return rc;
}

static int xenon_net_init_one(struct pci_dev *pdev,
							  const struct pci_device_id *ent)
{
	struct net_device *dev = NULL;
	struct xenon_net_private *tp;
	void *ioaddr = NULL;
	struct sockaddr addr;
	int rc;

	/* when built into the kernel, we only print version if device is found */
#ifndef MODULE
	static int printed_version;
	if (!printed_version++)
		printk(version);
#endif

	BUG_ON(pdev == NULL);
	BUG_ON(ent == NULL);

	rc = xenon_net_init_board(pdev, &dev, &ioaddr);
	if (rc < 0)
		return rc;

	tp = netdev_priv(dev);

	BUG_ON(ioaddr == NULL);
	BUG_ON(dev == NULL);
	BUG_ON(tp == NULL);

	tp->dev2 = dev;
	//	dev->ethtool_ops = &xenon_net_ethtool_ops;
	dev->watchdog_timeo = TX_TIMEOUT;

	netif_napi_add(dev, &tp->napi, xenon_net_poll, 64);

	dev->irq = pdev->irq;
	dev->base_addr = (unsigned long)ioaddr;

	/* priv/tp zeroed and aligned in alloc_etherdev */
	tp = netdev_priv(dev);

	tp->pdev = pdev;
	tp->mmio_addr = ioaddr;
	spin_lock_init(&tp->lock);

	pci_set_drvdata(pdev, dev);

	// FIXME: Need to get the MAC from NAND.
	// \x7C\x1E\x52\x0D\xF2\xAF
	memcpy(addr.sa_data, "\x7C\x1E\x52\x0D\xF2\xAF", 6); /* same as xell */
	rc = xenon_net_set_mac(dev, &addr);
	if (rc)
		dev_err(&pdev->dev, "Failed to set MAC address: %i\n", rc);

	printk(KERN_INFO "%s: %pM, IRQ %d\n", dev->name, dev->dev_addr,
		dev->irq);

	return 0;
}

static void xenon_net_remove_one(struct pci_dev *pdev)
{
	struct net_device *dev = pci_get_drvdata(pdev);
	struct xenon_net_private *np;

	BUG_ON(dev == NULL);

	np = netdev_priv(dev);
	BUG_ON(np == NULL);

	unregister_netdev(dev);

#ifndef USE_IO_OPS
	iounmap(np->mmio_addr);
#endif /* !USE_IO_OPS */

	pci_release_regions(pdev);

	free_netdev(dev);

	pci_set_drvdata(pdev, NULL);

	pci_disable_device(pdev);
}

static struct pci_driver xenon_net_pci_driver = {
	.name = MODNAME,
	.id_table = xenon_net_pci_tbl,
	.probe = xenon_net_init_one,
	.remove = xenon_net_remove_one,
};

static int __init xenon_net_init_module(void)
{
	/* when a module, this is printed whether or not devices are found in probe */
#ifdef MODULE
	printk(version);
#endif
	return pci_register_driver(&xenon_net_pci_driver);
}

static void __exit xenon_net_cleanup_module(void)
{
	pci_unregister_driver(&xenon_net_pci_driver);
}

module_init(xenon_net_init_module);
module_exit(xenon_net_cleanup_module);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Felix Domke <tmbinc@elitedvb.net>");
MODULE_DESCRIPTION("Xenon Southbridge Fast Ethernet Driver");
