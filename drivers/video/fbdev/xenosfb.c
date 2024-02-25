/*
 * framebuffer driver for Microsoft Xbox 360
 *
 * (c) 2006 ...
 * Original vesafb driver written by Gerd Knorr <kraxel@goldbach.in-berlin.de>
 *
 */

#include <linux/delay.h>
#include <linux/dmi.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/screen_info.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>

#include <asm/io.h>

#include <video/vga.h>

/* --------------------------------------------------------------------- */

static struct fb_var_screeninfo xenosfb_defined __initdata = {
    .activate = FB_ACTIVATE_NOW,
    .height = -1,
    .width = -1,
    .right_margin = 32,
    .upper_margin = 16,
    .lower_margin = 4,
    .vsync_len = 4,
    .vmode = FB_VMODE_NONINTERLACED,
};

static struct fb_fix_screeninfo xenosfb_fix __initdata = {
    .id = "XENON FB",
    .type = FB_TYPE_PACKED_PIXELS,
    .accel = FB_ACCEL_NONE,
    .visual = FB_VISUAL_TRUECOLOR,
};

// drivers/gpu/drm/radeon/r500_reg.h
typedef struct ati_info {
  uint32_t enable;          // 0x00 D1GRPH_ENABLE
  uint32_t control;         // 0x04 D1GRPH_CONTROL
  uint32_t lut_sel;         // 0x08 D1GRPH_LUT_SEL
  uint32_t unk;             // 0x0C D1GRPH_SWAP_CNTL
  uint32_t base;            // 0x10 D1GRPH_PRIMARY_SURFACE_ADDRESS
  uint32_t unk_14;          // 0x14 0x6114
  uint32_t secondary_base;  // 0x18 D1GRPH_SECONDARY_SURFACE_ADDRESS
  uint32_t unk_1C;          // 0x1C 0x611C
  uint32_t pitch;           // 0x20 D1GRPH_PITCH
  uint32_t offset_x;        // 0x24 D1GRPH_SURFACE_OFFSET_X
  uint32_t offset_y;        // 0x28 D1GRPH_SURFACE_OFFSET_Y
  uint32_t start_x;         // 0x2C D1GRPH_X_START
  uint32_t start_y;         // 0x30 D1GRPH_Y_START
  uint32_t width;           // 0x34 D1GRPH_X_END
  uint32_t height;          // 0x38 D1GRPH_Y_END
} ati_info_t;

typedef struct xenosfb_par {
  uint32_t pseudo_palette[16];
  uint32_t smem_len;

  void __iomem *graphics_base;
  ati_info_t __iomem *ati_regs;
} xenosfb_par_t;

#define DEFAULT_FB_MEM 1024 * 1024 * 16

/* --------------------------------------------------------------------- */

static int xenosfb_setcolreg(unsigned regno, unsigned red, unsigned green,
                             unsigned blue, unsigned transp,
                             struct fb_info *info) {
  /*
   *  Set a single color register. The values supplied are
   *  already rounded down to the hardware's capabilities
   *  (according to the entries in the `var' structure). Return
   *  != 0 for invalid regno.
   */

  if (regno >= info->cmap.len) return 1;

  if (regno < 16) {
    red >>= 8;
    green >>= 8;
    blue >>= 8;
    ((u32 *)(info->pseudo_palette))[regno] = (red << info->var.red.offset) |
                                             (green << info->var.green.offset) |
                                             (blue << info->var.blue.offset);
  }
  return 0;
}

#define XENON_XY_TO_STD_PTR(x, y)                              \
  ((int *)(((char *)p->screen_base) + y * p->fix.line_length + \
           x * (p->var.bits_per_pixel / 8)))
#define XENON_XY_TO_XENON_PTR(x, y) xenon_convert(p, XENON_XY_TO_STD_PTR(x, y))

inline void xenon_pset(struct fb_info *p, int x, int y, int color) {
  fb_writel(color, XENON_XY_TO_XENON_PTR(x, y));
}

inline int xenon_pget(struct fb_info *p, int x, int y) {
  return fb_readl(XENON_XY_TO_XENON_PTR(x, y));
}

void xenon_fillrect(struct fb_info *p, const struct fb_fillrect *rect) {
  __u32 x, y;
  for (y = 0; y < rect->height; y++) {
    for (x = 0; x < rect->width; x++) {
      xenon_pset(p, rect->dx + x, rect->dy + y, rect->color);
    }
  }
}

void xenon_copyarea(struct fb_info *p, const struct fb_copyarea *area) {
  /* if the beginning of the target area might overlap with the end of
  the source area, be have to copy the area reverse. */
  if ((area->dy == area->sy && area->dx > area->sx) || (area->dy > area->sy)) {
    __s32 x, y;
    for (y = area->height - 1; y > 0; y--) {
      for (x = area->width - 1; x > 0; x--) {
        xenon_pset(p, area->dx + x, area->dy + y,
                   xenon_pget(p, area->sx + x, area->sy + y));
      }
    }
  } else {
    __u32 x, y;
    for (y = 0; y < area->height; y++) {
      for (x = 0; x < area->width; x++) {
        xenon_pset(p, area->dx + x, area->dy + y,
                   xenon_pget(p, area->sx + x, area->sy + y));
      }
    }
  }
}

static struct fb_ops xenosfb_ops = {
    .owner = THIS_MODULE,
    .fb_setcolreg = xenosfb_setcolreg,
    .fb_fillrect = xenon_fillrect,
    .fb_copyarea = xenon_copyarea,
    .fb_imageblit = cfb_imageblit,
};

#ifdef CONFIG_PPC_EARLY_DEBUG_XENON
void udbg_shutdown_xenon(void);
#endif

static int __init xenosfb_probe(struct platform_device *dev) {
  struct screen_info screen_info;
  struct fb_info *info;
  int err;
  unsigned int size_vmode;
  unsigned int size_remap;
  unsigned int size_total;
  volatile int *gfx;
  xenosfb_par_t *par;

  info = framebuffer_alloc(sizeof(xenosfb_par_t), &dev->dev);
  if (!info) {
    err = -ENOMEM;
    return err;
  }
  par = info->par;
  info->pseudo_palette = par->pseudo_palette;
  par->graphics_base = ioremap(0x200ec800000ULL, 0x10000);
  par->ati_regs = par->graphics_base + 0x6100;
  if (!par->graphics_base) {
    printk(KERN_ERR
           "xenosfb: abort, cannot ioremap graphics registers "
           "0x%x @ 0x%llx\n",
           0x10000, 0x200ec800000ULL);
    err = -EIO;
    goto err_release_fb;
  }

  gfx = par->graphics_base + 0x6000;

  /* setup native resolution, i.e. disable scaling */
  int vxres = gfx[0x134 / 4];
  int vyres = gfx[0x138 / 4];

  int black_top = gfx[0x44 / 4];
  int offset = gfx[0x580 / 4];
  int offset_x = (offset >> 16) & 0xFFFF;
  int offset_y = offset & 0xFFFF;

  int nxres, nyres;
  int scl_h = gfx[0x5b4 / 4], scl_v = gfx[0x5c4 / 4];

  if (gfx[0x590 / 4] == 0) scl_h = scl_v = 0x01000000;

  nxres = (vxres - offset_x * 2) * 0x1000 / (scl_h / 0x1000);
  nyres = (vyres - offset_y * 2) * 0x1000 / (scl_v / 0x1000) + black_top * 2;

  printk("virtual resolution: %d x %d\n", vxres, vyres);
  printk("offset: x=%d, y=%d\n", offset_x, offset_y);
  printk("black: %d %d, %d %d\n", gfx[0x44 / 4], gfx[0x48 / 4], gfx[0x4c / 4],
         gfx[0x50 / 4]);

  printk("native resolution: %d x %d\n", nxres, nyres);

  screen_info.lfb_depth = 32;
  screen_info.lfb_size = DEFAULT_FB_MEM / 0x10000;
  screen_info.pages = 1;
  screen_info.blue_size = 8;
  screen_info.blue_pos = 24;
  screen_info.green_size = 8;
  screen_info.green_pos = 16;
  screen_info.red_size = 8;
  screen_info.red_pos = 8;
  screen_info.rsvd_size = 8;
  screen_info.rsvd_pos = 0;

  gfx[0x44 / 4] = 0;  // disable black bar
  gfx[0x48 / 4] = 0;
  gfx[0x4c / 4] = 0;
  gfx[0x50 / 4] = 0;

  gfx[0x590 / 4] = 0;  // disable scaling
  gfx[0x584 / 4] = (nxres << 16) | nyres;
  gfx[0x580 / 4] = 0;                       // disable offset
  gfx[0x5e8 / 4] = (nxres * 4) / 0x10 - 1;  // fix pitch
  gfx[0x134 / 4] = nxres;
  gfx[0x138 / 4] = nyres;

  par->ati_regs->base &= ~0xFFFF;  // page-align.

  screen_info.lfb_base = par->ati_regs->base;
  screen_info.lfb_width = par->ati_regs->width;
  screen_info.lfb_height = par->ati_regs->height;
  screen_info.lfb_linelength =
      screen_info.lfb_width * screen_info.lfb_depth / 4;

  /* fixup pitch, in case we switched resolution */
  gfx[0x120 / 4] = screen_info.lfb_linelength / 8;

  printk(KERN_INFO "xenosfb: detected %dx%d framebuffer @ 0x%08x\n",
         screen_info.lfb_width, screen_info.lfb_height, screen_info.lfb_base);

  xenosfb_fix.smem_start = screen_info.lfb_base;
  xenosfb_defined.bits_per_pixel = screen_info.lfb_depth;
  xenosfb_defined.xres = screen_info.lfb_width;
  xenosfb_defined.yres = screen_info.lfb_height;
  xenosfb_defined.xoffset = 0;
  xenosfb_defined.yoffset = 0;
  xenosfb_fix.line_length = screen_info.lfb_linelength;

  /*   size_vmode -- that is the amount of memory needed for the
   *                 used video mode, i.e. the minimum amount of
   *                 memory we need. */
  size_vmode = xenosfb_defined.yres * xenosfb_fix.line_length;

  /*   size_total -- all video memory we have. Used for
   *                 entries, ressource allocation and bounds
   *                 checking. */
  size_total = screen_info.lfb_size * 65536;
  if (size_total < size_vmode) size_total = size_vmode;

  /*   size_remap -- the amount of video memory we are going to
   *                 use for xenosfb.  With modern cards it is no
   *                 option to simply use size_total as that
   *                 wastes plenty of kernel address space. */
  size_remap = size_vmode * 2;
  if (size_remap < size_vmode) size_remap = size_vmode;
  if (size_remap > size_total) size_remap = size_total;
  xenosfb_fix.smem_len = size_remap;
  par->smem_len = size_total;

  if (!request_mem_region(xenosfb_fix.smem_start, size_total, "xenosfb")) {
    printk(KERN_WARNING "xenosfb: cannot reserve video memory at 0x%lx\n",
           xenosfb_fix.smem_start);
    /* We cannot make this fatal. Sometimes this comes from magic
       spaces our resource handlers simply don't know about */
  }

  info->screen_base = ioremap(xenosfb_fix.smem_start, xenosfb_fix.smem_len);
  if (!info->screen_base) {
    printk(KERN_ERR
           "xenosfb: abort, cannot ioremap video memory "
           "0x%x @ 0x%lx\n",
           xenosfb_fix.smem_len, xenosfb_fix.smem_start);
    err = -EIO;
    goto err_unmap_gfx;
  }

  printk(KERN_INFO
         "xenosfb: framebuffer at 0x%lx, mapped to 0x%p, "
         "using %dk, total %dk\n",
         xenosfb_fix.smem_start, info->screen_base, size_remap / 1024,
         size_total / 1024);
  printk(KERN_INFO "xenosfb: mode is %dx%dx%d, linelength=%d, pages=%d\n",
         xenosfb_defined.xres, xenosfb_defined.yres,
         xenosfb_defined.bits_per_pixel, xenosfb_fix.line_length,
         screen_info.pages);

  xenosfb_defined.xres_virtual = xenosfb_defined.xres;
  xenosfb_defined.yres_virtual = xenosfb_fix.smem_len / xenosfb_fix.line_length;
  printk(KERN_INFO "xenosfb: scrolling: redraw\n");
  xenosfb_defined.yres_virtual = xenosfb_defined.yres;

  /* some dummy values for timing to make fbset happy */
  xenosfb_defined.pixclock =
      10000000 / xenosfb_defined.xres * 1000 / xenosfb_defined.yres;
  xenosfb_defined.left_margin = (xenosfb_defined.xres / 8) & 0xf8;
  xenosfb_defined.hsync_len = (xenosfb_defined.xres / 8) & 0xf8;

  printk(KERN_INFO "xenosfb: pixclk=%ld left=%02x hsync=%02x\n",
         (unsigned long)xenosfb_defined.pixclock, xenosfb_defined.left_margin,
         xenosfb_defined.hsync_len);

  xenosfb_defined.red.offset = screen_info.red_pos;
  xenosfb_defined.red.length = screen_info.red_size;
  xenosfb_defined.green.offset = screen_info.green_pos;
  xenosfb_defined.green.length = screen_info.green_size;
  xenosfb_defined.blue.offset = screen_info.blue_pos;
  xenosfb_defined.blue.length = screen_info.blue_size;
  xenosfb_defined.transp.offset = screen_info.rsvd_pos;
  xenosfb_defined.transp.length = screen_info.rsvd_size;

  printk(KERN_INFO
         "xenosfb: %s: "
         "size=%d:%d:%d:%d, shift=%d:%d:%d:%d\n",
         "Truecolor", screen_info.rsvd_size, screen_info.red_size,
         screen_info.green_size, screen_info.blue_size, screen_info.rsvd_pos,
         screen_info.red_pos, screen_info.green_pos, screen_info.blue_pos);

  xenosfb_fix.ypanstep = 0;
  xenosfb_fix.ywrapstep = 0;

  /* request failure does not faze us, as vgacon probably has this
   * region already (FIXME) */
  request_region(0x3c0, 32, "xenosfb");

  info->fbops = &xenosfb_ops;
  info->var = xenosfb_defined;
  info->fix = xenosfb_fix;
  info->flags = FBINFO_FLAG_DEFAULT;

  if (fb_alloc_cmap(&info->cmap, 256, 0) < 0) {
    err = -ENOMEM;
    goto err_unmap;
  }
  if (register_framebuffer(info) < 0) {
    err = -EINVAL;
    goto err_fb_dealoc;
  }

  par->ati_regs->enable = 1;

// Shut down the early framebuffer driver (if enabled).
#ifdef CONFIG_PPC_EARLY_DEBUG_XENON
	udbg_shutdown_xenon();
#endif

	printk(KERN_INFO "fb%d: %s frame buffer device\n", info->node,
	       info->fix.id);
	return 0;

err_fb_dealoc:
  fb_dealloc_cmap(&info->cmap);
err_unmap:
  iounmap(info->screen_base);
  release_mem_region(xenosfb_fix.smem_start, size_total);
err_unmap_gfx:
  iounmap(par->graphics_base);
err_release_fb:
  framebuffer_release(info);
  return err;
}

static int __exit xenosfb_remove(struct platform_device *dev) {
  struct fb_info *info = platform_get_drvdata(dev);

  if (info) {
    xenosfb_par_t *par = info->par;
    par->ati_regs->enable = 0;

    unregister_framebuffer(info);
    fb_dealloc_cmap(&info->cmap);
    iounmap(info->screen_base);
    release_mem_region(xenosfb_fix.smem_start, par->smem_len);
    framebuffer_release(info);
  }

  return 0;
}

static struct platform_driver xenosfb_driver = {
    .probe = xenosfb_probe,
    .remove = xenosfb_remove,
    .driver =
        {
            .name = "xenosfb",
        },
};

static struct platform_device xenosfb_device = {
    .name = "xenosfb",
};

static int __init xenosfb_init(void) {
  int ret;

  ret = platform_driver_register(&xenosfb_driver);

  if (!ret) {
    ret = platform_device_register(&xenosfb_device);
    if (ret) platform_driver_unregister(&xenosfb_driver);
  }
  return ret;
}

static void __exit xenosfb_exit(void) {
  platform_device_unregister(&xenosfb_device);
  platform_driver_unregister(&xenosfb_driver);
}

module_init(xenosfb_init);
module_exit(xenosfb_exit);
MODULE_LICENSE("GPL");