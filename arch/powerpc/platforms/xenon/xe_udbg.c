/*
 * xe_udbg.c: Early debug console via framebuffer
 *
 * Copyright 2017 Justin Moore
 *
 * Licensed under the GPL v2.
 */

#include <linux/bug.h>
#include <linux/delay.h>
#include <linux/font.h>
#include <linux/slab.h>

#include <asm/cacheflush.h>
#include <asm/fixmap.h>
#include <asm/io.h>
#include <asm/page.h>
#include <asm/prom.h>
#include <asm/udbg.h>

// ========================================================================
// ========================================================================

typedef struct console_context {
  void __iomem *fb;
  uint32_t width;
  uint32_t height;

  uint32_t cursor_x;
  uint32_t cursor_y;
} console_context_t;
static console_context_t console_ctx;

static void console_clrscr(console_context_t *context) {
  unsigned int *fb = (unsigned int *)context->fb;
  int count = context->width * context->height;
  while (count--) out_be32(fb++, 0x00000000);

  context->cursor_x = 0;
  context->cursor_y = 0;
}

/* set a pixel to RGB values, must call console_init() first */
static inline void console_pset32(console_context_t *context, int x, int y,
                                  int color) {
  uint32_t *fb = ((uint32_t *)context->fb);
#define base                                                            \
  (((y >> 5) * 32 * context->width + ((x >> 5) << 10) + (x & 3) +       \
    ((y & 1) << 2) + (((x & 31) >> 2) << 3) + (((y & 31) >> 1) << 6)) ^ \
   ((y & 8) << 2))
  fb[base] = color;
#undef base
}

static inline void console_pset(console_context_t *context, int x, int y,
                                unsigned char r, unsigned char g,
                                unsigned char b) {
  console_pset32(context, x, y, (b << 24) + (g << 16) + (r << 8));
}

static void console_draw_char(console_context_t *context,
                              const uint8_t *fontdata_8x16, const int x,
                              const int y, const unsigned char c) {
#define font_pixel(ch, x, y) ((fontdata_8x16[ch * 16 + y] >> (7 - x)) & 1)
  int lx, ly;
  for (ly = 0; ly < 16; ly++) {
    for (lx = 0; lx < 8; lx++) {
      console_pset32(context, x + lx, y + ly,
                     font_pixel(c, lx, ly) == 1 ? 0xFFFFFF00 : 0x00000000);
    }
  }
#undef font_pixel
}

static void console_scroll32(console_context_t *context,
                             const unsigned int lines) {
  int l, bs;
  uint32_t *fb, *end;
  uint32_t console_size = context->width * context->height;

  bs = context->width * 32 * 4;
  // copy all tile blocks to a higher position
  for (l = lines; l * 32 < context->height; l++) {
    memcpy(context->fb + bs * (l - lines), context->fb + bs * l, bs);
  }

  // fill up last lines with background color
  fb = (uint32_t *)(context->fb + console_size * 4 - bs * lines);
  end = (uint32_t *)(context->fb + console_size * 4);
  memset(fb, 0x00000000, (end - fb) * 4);
}

static void console_newline(console_context_t *context) {
  /* reset to the left and flush line */
  context->cursor_x = 0;
  context->cursor_y++;

  if (context->cursor_y >= ((context->height - 32) / 16)) {
    console_scroll32(context, 1);
    context->cursor_y -= 2;
  }
}

static void console_putch(const char c) {
  if (!console_ctx.fb) return;

  if (c == '\r') {
    console_ctx.cursor_x = 0;
  } else if (c == '\n') {
    console_newline(&console_ctx);
  } else {
    console_draw_char(&console_ctx, font_vga_8x16.data,
                      console_ctx.cursor_x * 8 + 32,
                      console_ctx.cursor_y * 16 + 32, c);
    console_ctx.cursor_x++;
    if (console_ctx.cursor_x >= (console_ctx.width - 32) / 8)
      console_newline(&console_ctx);
  }
}

// ========================================================================
// ========================================================================

void __init udbg_init_xenon(void) {
  /*
   * 148x41
   * Since we're running in real mode now, we can't enable logging until
   * we reach virtual mode. Just set up our structure and clear the
   * screen. This depends on XeLL running before us, and it having already
   * set up the screen.
   */
  memset(&console_ctx, 0, sizeof(console_context_t));
  console_ctx.fb = (void *)0x1E000000ull;
  console_ctx.width = ((1280 + 31) >> 5) << 5;
  console_ctx.height = ((720 + 31) >> 5) << 5;
  console_clrscr(&console_ctx);
  // udbg_putc = console_putch;
}

void __init udbg_init_xenon_virtual(void) {
  void __iomem *framebuffer = ioremap(0x1E000000, 0x01FFFFFF);
  if (framebuffer) {
    console_ctx.fb = framebuffer;
    udbg_putc = console_putch;
  }
}
EXPORT_SYMBOL(udbg_init_xenon_virtual);

void udbg_shutdown_xenon(void) {
  if (console_ctx.fb) {
    // cannot iounmap early bolted memory
    // iounmap(console_ctx.fb);
    console_ctx.fb = NULL;
  }

  if (udbg_putc == NULL) {
    return;
  }

  udbg_putc = NULL;
}
EXPORT_SYMBOL(udbg_shutdown_xenon);