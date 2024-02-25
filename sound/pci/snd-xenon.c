/*
 *  xenon_snd.c - driver for XBOX 360 soundcard.
 *  Copyright (C) 2009 by jc4360@gmail.com
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

// #include <sound/driver.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/module.h>

#include <asm/io.h>

#include <sound/core.h>
#include <sound/control.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/pcm-indirect.h>
#include <sound/asoundef.h>

#define DESCRIPTOR_BUFFER_SIZE (32 * sizeof(u32) * 2)
#define CACHELINE_SIZE 128

#define IRQ_DISABLE             0
#define IRQ_ENABLE              1

static struct pci_device_id snd_xenon_ids[] = {
	{ 0x1414, 0x580c, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{ 0, }
};

MODULE_DEVICE_TABLE(pci, snd_xenon_ids);

static int index[SNDRV_CARDS] = SNDRV_DEFAULT_IDX;
static char *id[SNDRV_CARDS] = SNDRV_DEFAULT_STR;
static int enable[SNDRV_CARDS] = SNDRV_DEFAULT_ENABLE_PNP;


struct playback_device {
	struct snd_pcm_substream *playback_substream;

	int state;
	void *dma_base_virt;
	dma_addr_t descr_base_phys;
	u32 *descr_base_virt;

	int buffer_bytes;
	int period_bytes;
	int descr_bytes;
	int gap;
	int wptr;
};

struct snd_xenon {
	unsigned long iobase_phys;
	void *iobase_virt;

	spinlock_t lock;
	struct snd_card *card;
	struct pci_dev *pci;

	int irq;

	dma_addr_t descr_base_phys;
	void *descr_base_virt;

	struct snd_pcm *pcm[2];
	struct playback_device devices[2];

	unsigned int dig_status;
	unsigned int dig_pcm_status;

	struct timer_list timer;
	int timer_in_use;
};

static void cache_flush(void *addr, int size)
{
    void *p = addr;
    while (size) {
	__asm__ __volatile__ ("dcbst 0,%0" :: "r" (p));
	p += 128;
	size -= 128;
    }
    __asm__ __volatile__ ("sync" ::: "memory");
}

static inline u32 bswap32(u32 t)
{
	return ((t & 0xFF) << 24) | ((t & 0xFF00) << 8) | ((t & 0xFF0000) >> 8) | ((t & 0xFF000000) >> 24);
}

void xenon_smc_send_message(unsigned char *msg)
{

	void *base = ioremap(0x200ea001000, 0x1000);

	while (!(readl(base + 0x84) & 4));
	writel(4, base + 0x84);
	writel(bswap32(*(u32 *)(msg + 0)), base + 0x80);
	writel(bswap32(*(u32 *)(msg + 4)), base + 0x80);
	writel(bswap32(*(u32 *)(msg + 8)), base + 0x80);
	writel(bswap32(*(u32 *)(msg + 12)), base + 0x80);
	writel(0, base + 0x84);
	iounmap(base);
}

static inline void snd_xenon_set_irq_flag(struct snd_xenon *chip, int cmd)
{
	// enable/disable irq
	return;
}

static irqreturn_t snd_xenon_interrupt(int irq, void *dev_id)
{
	printk("xenon_snd: give me an interrupt, please!\n");
	return IRQ_HANDLED;
}

static void snd_xenon_timer_fn(struct timer_list * t)
{
	struct snd_xenon *chip = from_timer(chip, t, timer);

	u32 reg;
	int rptr_descr, wptr_descr, cur_len, size;

	struct playback_device *device = NULL;
	int dev_id = 0;

	for (dev_id = 0; dev_id < 2; dev_id++) {

		device = &chip->devices[dev_id];

		spin_lock(&chip->lock);

		if (device->state != 3) {
			spin_unlock(&chip->lock);
			continue;
		}

		reg = readl(chip->iobase_virt + 0x04 + dev_id * 0x10);
		rptr_descr = reg & 0x1f;
		wptr_descr = (reg & 0x1f00) >> 8;
		cur_len = (reg >> 16) & 0xFFFF;

		size = wptr_descr - rptr_descr;
		if (size < 0) size += 32;
		size *= device->descr_bytes;
		size += cur_len;
		if (wptr_descr < rptr_descr) size -= device->gap;
		size = device->buffer_bytes - size;

		if (size >= device->period_bytes) {
			spin_unlock(&chip->lock);
			snd_pcm_period_elapsed(device->playback_substream);
		} else
			spin_unlock(&chip->lock);
	}

	if (chip->timer_in_use) {
		mod_timer(&chip->timer, jiffies + usecs_to_jiffies(200));
	}

}

static struct snd_pcm_hardware snd_xenon_ana_playback_hw =
{
	.info =			(SNDRV_PCM_INFO_MMAP |
				SNDRV_PCM_INFO_INTERLEAVED |
				SNDRV_PCM_INFO_BLOCK_TRANSFER |
				SNDRV_PCM_INFO_MMAP_VALID),
	.formats =              SNDRV_PCM_FMTBIT_S16_LE,
	.rates =                SNDRV_PCM_RATE_48000,
	.rate_min =		48000,
	.rate_max =		48000,
	.channels_min =		2,
	.channels_max =		2,
	.buffer_bytes_max =	64 * 1024,
	.period_bytes_min =	64,
	.period_bytes_max =	64 * 1024,
	.periods_min =		1,
	.periods_max =		1024,
	.fifo_size =            0,
};

static struct snd_pcm_hardware snd_xenon_spdif_playback_hw =
{
	.info =                 (SNDRV_PCM_INFO_MMAP |
				SNDRV_PCM_INFO_INTERLEAVED |
				SNDRV_PCM_INFO_BLOCK_TRANSFER |
				SNDRV_PCM_INFO_MMAP_VALID),
	.formats =              SNDRV_PCM_FMTBIT_S16_LE,
	.rates =                SNDRV_PCM_RATE_48000,
	.rate_min =             48000,
	.rate_max =             48000,
	.channels_min =         2,
	.channels_max =         2,
	.buffer_bytes_max =     64 * 1024,
	.period_bytes_min =     64,
	.period_bytes_max =     64 * 1024,
	.periods_min =          1,
	.periods_max =          1024,
	.fifo_size =            0,
};

static int snd_xenon_playback_open(struct snd_pcm_substream *substream, int dev_id, struct snd_pcm_hardware *playback_hw)
{
	struct snd_xenon *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct playback_device *device = &chip->devices[dev_id];

	runtime->hw = *playback_hw;

	spin_lock_irq(&chip->lock);

	memset(device, 0, sizeof(struct playback_device));
	device->playback_substream = substream;

	writel(0x2000000, chip->iobase_virt + 0x08 + dev_id * 0x10);
	device->descr_base_phys =
		chip->descr_base_phys +  dev_id * 0x100;
	device->descr_base_virt =
		(u32 *)(chip->descr_base_virt + dev_id * 0x100);
	device->state = 1;

	spin_unlock_irq(&chip->lock);

	return 0;
}

static int snd_xenon_ana_playback_open(struct snd_pcm_substream *substream)
{
	return snd_xenon_playback_open(substream, 0, &snd_xenon_ana_playback_hw);
}

static int snd_xenon_spdif_playback_open(struct snd_pcm_substream *substream)
{
	return snd_xenon_playback_open(substream, 1, &snd_xenon_spdif_playback_hw);
}
static int snd_xenon_playback_close(struct snd_pcm_substream *substream)
{
	struct snd_xenon *chip = snd_pcm_substream_chip(substream);
	struct playback_device *device = &chip->devices[0];

	if (device->playback_substream != substream)
		device = &chip->devices[1];

	spin_lock_irq(&chip->lock);
	device->state = 0;
	spin_unlock_irq(&chip->lock);

	if (device->dma_base_virt != NULL)
	   iounmap(device->dma_base_virt);

	device->playback_substream = NULL;
	snd_pcm_lib_free_pages(substream);

	return 0;
}

static int snd_xenon_pcm_hw_params(struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *hw_params)
{
	int bytes = params_buffer_bytes(hw_params);
	return snd_pcm_lib_malloc_pages(substream, bytes);
}

static int snd_xenon_pcm_hw_free(struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_pages(substream);
}

static int snd_xenon_playback_prepare(struct snd_pcm_substream *substream)
{
	struct snd_xenon *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct playback_device *device = &chip->devices[0];
	int i , dev_id = 0;

	if (device->playback_substream != substream) {
		dev_id = 1;
		device = &chip->devices[1];
	}

	spin_lock_irq(&chip->lock);

	device->dma_base_virt =
		ioremap(runtime->dma_addr, runtime->dma_bytes);

	memset(device->dma_base_virt, 0, runtime->dma_bytes);
	cache_flush(device->dma_base_virt, runtime->dma_bytes);

	device->state = 2;
	device->period_bytes = snd_pcm_lib_period_bytes(substream);
	device->buffer_bytes = snd_pcm_lib_buffer_bytes(substream);
	device->descr_bytes = (device->buffer_bytes + 31 ) / 32;
	device->gap = (device->descr_bytes << 5) -
			    device->buffer_bytes;
	device->wptr = -1;


	for (i=0; i < 32; i++) {
		device->descr_base_virt[i*2] =
			bswap32(runtime->dma_addr + device->descr_bytes * i);
		device->descr_base_virt[i*2 + 1] =
			bswap32(0x80000000 | (device->descr_bytes - 1));
	}
	device->descr_base_virt[31*2 + 1] =
		bswap32(0x80000000 | (device->descr_bytes - 1 - device->gap));
	cache_flush(device->descr_base_virt, DESCRIPTOR_BUFFER_SIZE);

	writel(device->descr_base_phys, chip->iobase_virt + 0x00 + dev_id * 0x10);
	writel(0x1c08001c, chip->iobase_virt + 0x08 + dev_id * 0x10);
	writel((dev_id==0)?0x1c:0x02009902, chip->iobase_virt + 0x0c + dev_id * 0x10);

	spin_unlock_irq(&chip->lock);
	return 0;
}

static int snd_xenon_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_xenon *chip = snd_pcm_substream_chip(substream);
	struct playback_device *device = &chip->devices[0];
	int dev_id = 0, ret = 0;
	u32 reg;

	if (device->playback_substream != substream) {
		dev_id = 1;
		device = &chip->devices[1];
	}

	spin_lock(&chip->lock);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		device->state = 3;

		reg = readl(chip->iobase_virt + 0x08 + dev_id * 0x10);
		writel(reg | 0x1000000, chip->iobase_virt + 0x08 + dev_id * 0x10);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
		device->state = 4;
		reg = readl(chip->iobase_virt + 0x08 + dev_id * 0x10);
		writel(reg & ~0x1000000, chip->iobase_virt + 0x08 + dev_id * 0x10);
		break;

	default:
		ret = -EINVAL;
	}
	spin_unlock(&chip->lock);

	return ret;
}

static snd_pcm_uframes_t snd_xenon_pointer(struct snd_pcm_substream *substream)
{
	struct snd_xenon *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct playback_device *device = &chip->devices[0];

	int dev_id = 0;
	int app_ptr, app_descr, bytes;
	u32 reg;

	if (device->playback_substream != substream) {
		dev_id = 1;
		device = &chip->devices[1];
	}

	spin_lock(&chip->lock);

	app_ptr = frames_to_bytes(runtime, runtime->control->appl_ptr) %
		      device->buffer_bytes;

	if (app_ptr != device->wptr)
		cache_flush(device->dma_base_virt, runtime->dma_bytes);

	app_descr = app_ptr / device->descr_bytes;
	reg = readl(chip->iobase_virt + 0x04 + dev_id * 0x10);
	if (app_descr == (( reg & 0x1f00) >> 8))
	    app_descr -= 1;
	if (app_descr < 0) app_descr += 32;

	device->wptr = app_ptr;

	writel(app_descr << 8, chip->iobase_virt + 0x04 + dev_id * 0x10);

	spin_unlock(&chip->lock);

	bytes = (reg & 0x1f) * (device->descr_bytes);
	return bytes_to_frames(substream->runtime, bytes);
}

static struct snd_pcm_ops snd_xenon_ana_playback_ops = {
	.open =		snd_xenon_ana_playback_open,
	.close =	snd_xenon_playback_close,
	.ioctl =	snd_pcm_lib_ioctl,
	.hw_params =	snd_xenon_pcm_hw_params,
	.hw_free =	snd_xenon_pcm_hw_free,
	.prepare =	snd_xenon_playback_prepare,
	.trigger =	snd_xenon_trigger,
	.pointer =	snd_xenon_pointer,
};

static struct snd_pcm_ops snd_xenon_spdif_playback_ops = {
	.open =         snd_xenon_spdif_playback_open,
	.close =        snd_xenon_playback_close,
	.ioctl =        snd_pcm_lib_ioctl,
	.hw_params =    snd_xenon_pcm_hw_params,
	.hw_free =      snd_xenon_pcm_hw_free,
	.prepare =      snd_xenon_playback_prepare,
	.trigger =      snd_xenon_trigger,
	.pointer =      snd_xenon_pointer,
};

static int snd_xenon_new_pcm(struct snd_xenon *chip)
{
	struct snd_pcm *pcm;
	int err;

	err = snd_pcm_new(chip->card, "Xenon Audio", 0, 1, 0, &pcm);
	if (err < 0)
		return err;
	pcm->private_data = chip;
	strcpy(pcm->name, "Analog");
	chip->pcm[0] = pcm;

	/* set operators */
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK,
				&snd_xenon_ana_playback_ops);

	/* pre-allocation of buffers */
	snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV,
	&chip->pci->dev, 64*1024, 64*1024);

	err = snd_pcm_new(chip->card, "Xenon Audio", 1, 1, 0, &pcm);
	if (err < 0)
		return err;
	pcm->private_data = chip;
	strcpy(pcm->name, "Digital");
	chip->pcm[1] = pcm;
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK,
				&snd_xenon_spdif_playback_ops);
	snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV,
	&chip->pci->dev, 64*1024, 64*1024);

	return 0;
}

static void snd_xenon_init(struct snd_xenon *chip)
{
	unsigned long flags;
	static unsigned char smc_snd[32] = {0x8d, 1, 1};
	xenon_smc_send_message(smc_snd);

	spin_lock_irqsave(&chip->lock, flags);

	chip->descr_base_virt = pci_alloc_consistent(chip->pci,
			 DESCRIPTOR_BUFFER_SIZE * 2, &chip->descr_base_phys);
	chip->descr_base_phys &= 0x1fffffff;
	printk("snd_xenon: descr_base_virt=0x%llx, descr_base_phys=0x%llx\n",
		(unsigned long long)chip->descr_base_virt,
		(unsigned long long)chip->descr_base_phys);

	writel(0, chip->iobase_virt + 0x08);
	writel(0x2000000, chip->iobase_virt + 0x08);
	writel(chip->descr_base_phys, chip->iobase_virt + 0x00);

	writel(0, chip->iobase_virt + 0x18);
	writel(0x2000000, chip->iobase_virt + 0x18);
	writel(chip->descr_base_phys + DESCRIPTOR_BUFFER_SIZE,
		chip->iobase_virt + 0x10);

	/* Enable IRQ output */
	snd_xenon_set_irq_flag(chip, IRQ_ENABLE);

	spin_unlock_irqrestore(&chip->lock, flags);
}

static int snd_xenon_dev_free(struct snd_device *device);
static int snd_xenon_free(struct snd_xenon *chip);

static int snd_xenon_create(struct snd_card *card,
				       struct pci_dev *pci,
				       struct snd_xenon **rchip)
{
	struct snd_xenon *chip;
	int err;

	static struct snd_device_ops ops = {
		.dev_free = snd_xenon_dev_free,
	};
	*rchip = NULL;

	if ((err = pci_enable_device(pci)) < 0)
		return err;

	pci_set_master(pci);

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (chip == NULL) {
		pci_disable_device(pci);
		return -ENOMEM;
	}

	chip->card = card;
	chip->pci = pci;
	chip->irq = -1;
	chip->dig_status = SNDRV_PCM_DEFAULT_CON_SPDIF;
	chip->dig_pcm_status = SNDRV_PCM_DEFAULT_CON_SPDIF;
	spin_lock_init(&chip->lock);

	if (!( pci_resource_flags (pci, 0) & IORESOURCE_MEM)) {
		dev_err(&pci->dev,
			"region #0 not an MMIO resource, aborting\n");
		return -ENODEV;
	}

	if ((err = pci_request_regions(pci, "Xenon AudioPCI")) < 0) {
		kfree(chip);
		pci_disable_device(pci);
		return err;
	}

	chip->iobase_phys = pci_resource_start(pci, 0);
	chip->iobase_virt = ioremap(chip->iobase_phys,
				      pci_resource_len(pci, 0));

	printk("snd_xenon: iobase_phys=0x%lx iobase_virt=0x%llx\n", chip->iobase_phys, (unsigned long long)chip->iobase_virt);

	if (request_irq(pci->irq, snd_xenon_interrupt, IRQF_SHARED,
			card->shortname, chip)) {
		snd_printk(KERN_ERR "unable to grab IRQ %d\n", pci->irq);
		snd_xenon_free(chip);
		return -EBUSY;
	}
	chip->irq = pci->irq;
	pci_intx(pci, IRQ_ENABLE);
	printk("snd_xenon: irq=%x\n", chip->irq);

	snd_xenon_init(chip);

	if ((err = snd_xenon_new_pcm(chip)) < 0) {
		snd_printk(KERN_WARNING "Could not to create PCM\n");
		snd_xenon_free(chip);
		return err;
	}

	if ((err = snd_device_new(card, SNDRV_DEV_LOWLEVEL,
						chip, &ops)) < 0) {
		snd_xenon_free(chip);
		return err;
	}

	snd_card_set_dev(card, &pci->dev);

	*rchip = chip;

	timer_setup(&chip->timer, snd_xenon_timer_fn, 0);
	chip->timer_in_use = 1;

	// chip->timer.data = (unsigned long)chip;

	printk("snd_xenon: driver initialized\n");

	return 0;
}

static int snd_xenon_dev_free(struct snd_device *device)
{
	struct snd_xenon *chip = device->device_data;
	return snd_xenon_free(chip);
}

static int snd_xenon_free(struct snd_xenon *chip)
{
	snd_xenon_set_irq_flag(chip, IRQ_DISABLE);

	spin_lock_irq(&chip->lock);
	chip->timer_in_use = 0;
	spin_unlock_irq(&chip->lock);
	del_timer_sync(&chip->timer);

	if (chip->irq >= 0)
		free_irq(chip->irq, chip);
	if (chip->descr_base_virt)
		pci_free_consistent(chip->pci, DESCRIPTOR_BUFFER_SIZE * 2, chip->descr_base_virt, chip->descr_base_phys);
	if (chip->iobase_virt)
		  iounmap(chip->iobase_virt);
	pci_release_regions(chip->pci);
	pci_disable_device(chip->pci);
	kfree(chip);
	return 0;
}

static int snd_xenon_probe(struct pci_dev *pci,
			     const struct pci_device_id *pci_id)
{
	static int dev;
	struct snd_card *card;
	struct snd_xenon *chip;
	int err;

	if (dev >= SNDRV_CARDS)
		return -ENODEV;
	if (!enable[dev]) {
		dev++;
		return -ENOENT;
	}

	err = snd_card_new(&pci->dev, index[dev], id[dev], THIS_MODULE, 0, &card);
	if (err < 0)
		return -ENOMEM;

	if ((err = snd_xenon_create(card, pci, &chip)) < 0) {
		snd_card_free(card);
		snd_xenon_free(chip);
		return err;
	}
	card->private_data = chip;

	strcpy(card->driver, "snd-xenon");
	sprintf(card->shortname, "Xenon AudioPCI");
	sprintf(card->longname, "%s at 0x%lx irq %i",
				card->shortname, chip->iobase_phys, chip->irq);

	if ((err = snd_card_register(card)) < 0) {
		snd_card_free(card);
		snd_xenon_free(chip);
		return err;
	}

	pci_set_drvdata(pci, card);
	dev++;

	return 0;
}

static void snd_xenon_remove(struct pci_dev *pci)
{
	snd_card_free(pci_get_drvdata(pci));
	pci_set_drvdata(pci, NULL);
}

static struct pci_driver driver = {
	.name = "snd-xenon",
	.id_table = snd_xenon_ids,
	.probe = snd_xenon_probe,
	.remove = snd_xenon_remove,
};

static int __init alsa_card_xenon_init(void)
{
	return pci_register_driver(&driver);
}

static void __exit alsa_card_xenon_exit(void)
{
	pci_unregister_driver(&driver);
}

module_init(alsa_card_xenon_init)
module_exit(alsa_card_xenon_exit)

MODULE_AUTHOR("jc4360@gmail.com");
MODULE_DESCRIPTION("Xenon Audio Driver");
MODULE_LICENSE("GPL");
