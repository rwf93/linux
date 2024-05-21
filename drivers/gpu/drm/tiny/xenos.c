/* SPDX-License-Identifier: GPL-2.0 */

#include <linux/iosys-map.h>
#include <linux/module.h>
#include <linux/pci.h>

#include <video/cirrus.h>
#include <video/vga.h>

#include <drm/drm_aperture.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_edid.h>
#include <drm/drm_fbdev_generic.h>
#include <drm/drm_file.h>
#include <drm/drm_format_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_ioctl.h>
#include <drm/drm_managed.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_module.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>

#define DRIVER_NAME "xenos"
#define DRIVER_DESC "DRM framebuffer driver for Xbox 360's Xenos"
#define DRIVER_DATE "20240519"
#define DRIVER_MAJOR 1
#define DRIVER_MINOR 0

struct xenos_device {
	struct drm_device dev;

	struct drm_simple_display_pipe pipe;
	struct drm_connector connector;

	struct drm_display_mode fixed_mode;
	struct drm_gem_dma_object *real_framebuffer;

	void __iomem *regs;
};

static struct xenos_device *xenos_of_pipe(struct drm_simple_display_pipe *pipe)
{
	return container_of(pipe, struct xenos_device, pipe);
}

static struct xenos_device *xenos_of_dev(struct drm_device *dev)
{
	return container_of(dev, struct xenos_device, dev);
}

/* --- Main display pipe --- */

static void xenos_enable(struct drm_simple_display_pipe *pipe,
			 struct drm_crtc_state *crtc_state,
			 struct drm_plane_state *plane_state)
{
	struct xenos_device *xenos = xenos_of_pipe(pipe);

	// since I haven't worked out how to describe nonlinear tilings yet,
	// let's just fake it with shadow buffers
	if (crtc_state->planes_changed ||
	    IS_ERR_OR_NULL(xenos->real_framebuffer)) {
		// we're guaranteed no scaling in simple pipe, so this is ok
		int width = plane_state->fb->width;
		int height = plane_state->fb->height;
		int bpp = drm_format_info_bpp(plane_state->fb->format, 0);
		int cpp = DIV_ROUND_UP(bpp, 8);

		if (!IS_ERR_OR_NULL(xenos->real_framebuffer))
			drm_gem_dma_free(xenos->real_framebuffer);

		xenos->real_framebuffer =
			drm_gem_dma_create(&xenos->dev, width * height * cpp);

		drm_info(&xenos->dev, "Using %dx%d (%04x) fb\n", width, height,
			 width * height * cpp);
		iowrite32be(xenos->real_framebuffer->dma_addr,
			    xenos->regs + 0x6110);
	}

	iowrite32be(1, xenos->regs + 0x6100);
}

static void xenos_disable(struct drm_simple_display_pipe *pipe)
{
	struct xenos_device *xenos = xenos_of_pipe(pipe);
	iowrite32be(0, xenos->regs + 0x6100);

	drm_gem_dma_free(xenos->real_framebuffer);
	xenos->real_framebuffer = NULL;
}

static void xenos_update(struct drm_simple_display_pipe *pipe,
			 struct drm_plane_state *old_state)
{
	struct xenos_device *xenos = xenos_of_pipe(pipe);
	struct drm_device *dev = &xenos->dev;

	struct drm_plane_state *state = pipe->plane.state;
	struct drm_shadow_plane_state *shadow_plane_state = to_drm_shadow_plane_state(state);
	struct drm_framebuffer *fb = state->fb;
	struct drm_atomic_helper_damage_iter iter;
	struct drm_rect damage;
	int ret, idx;

	struct iosys_map src = shadow_plane_state->data[0];
	struct iosys_map dst;

	// I don't understand the DRM lifecycle well enough, but this gets
	// called *before* enable in some cases.
	if (IS_ERR_OR_NULL(xenos->real_framebuffer))
		return;

	ret = drm_gem_dma_vmap(xenos->real_framebuffer, &dst);
	if (ret)
		return;

	if (!drm_dev_enter(&xenos->dev, &idx))
		return;

	drm_atomic_helper_damage_iter_init(&iter, old_state, state);
	drm_atomic_for_each_plane_damage(&iter, &damage) {
		struct drm_rect dst_clip = state->dst;

		if (!drm_rect_intersect(&dst_clip, &damage))
			continue;

		for (int y = damage.y1; y < damage.y2; y++) {
			for (int x = damage.x1; x < damage.x2; x++) {
				// horrible calculation from libxenon
				int ndx = ((((y & ~31)*fb->width) + (x & ~31)*32 ) +
					   (((x&3) + ((y&1)<<2) + ((x&28)<<1) + ((y&30)<<5)) ^ ((y&8)<<2)));

				u32 *dst32 = dst.vaddr + ndx * 4;
				const u32 *src32 = src.vaddr + x*4 + y * fb->pitches[0];
				*dst32 = swab32(*src32);
				//iosys_map_memcpy_to(&dst, ndx * 4, src.vaddr + x*4 + y * fb->pitches[0], 4);
			}
		}
	}

	dma_sync_single_for_device(dev->dev, xenos->real_framebuffer->dma_addr, 1280*720*4, DMA_TO_DEVICE);

	drm_dev_exit(idx);
}

static const struct drm_simple_display_pipe_funcs xenos_pipe_funcs = {
	DRM_GEM_SIMPLE_DISPLAY_PIPE_SHADOW_PLANE_FUNCS,
	.enable = xenos_enable,
	.disable = xenos_disable,
	.update = xenos_update,
};

/* --- Connector --- */

static int xenos_connector_helper_get_modes(struct drm_connector *connector)
{
	struct xenos_device *xenos = xenos_of_dev(connector->dev);

	return drm_connector_helper_get_modes_fixed(connector,
						    &xenos->fixed_mode);
}

static const struct drm_connector_helper_funcs xenos_connector_helper_funcs = {
	.get_modes = xenos_connector_helper_get_modes,
};

static const struct drm_connector_funcs xenos_connector_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

/* --- Main driver --- */

static const struct drm_mode_config_funcs xenos_mode_config_funcs = {
	.fb_create = drm_gem_fb_create_with_dirty,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static int xenos_load(struct xenos_device *xenos)
{
	struct drm_device *dev = &xenos->dev;
	struct drm_connector *connector = &xenos->connector;
	int ret;

	static const struct drm_display_mode mode = { DRM_MODE_INIT(
		60, 1280, 720, DRM_MODE_RES_MM(1280, 96ul),
		DRM_MODE_RES_MM(720, 96ul)) };
	static const uint32_t formats[] = { DRM_FORMAT_XRGB8888 };

	ret = drmm_mode_config_init(dev);
	if (ret)
		return ret;

	// todo these are guesses. check the radeon ref
	dev->mode_config.min_width = 32;
	dev->mode_config.max_width = 4096;
	dev->mode_config.min_height = 32;
	dev->mode_config.max_height = 4096;
	dev->mode_config.preferred_depth = 32;
	dev->mode_config.funcs = &xenos_mode_config_funcs;

	xenos->fixed_mode = mode;

	ret = drm_connector_init(dev, connector, &xenos_connector_funcs,
				 DRM_MODE_CONNECTOR_Unknown);
	if (ret)
		return ret;

	drm_connector_helper_add(connector, &xenos_connector_helper_funcs);
	drm_connector_set_panel_orientation_with_quirk(
		connector, DRM_MODE_PANEL_ORIENTATION_UNKNOWN, 1280, 720);

	ret = drm_simple_display_pipe_init(dev, &xenos->pipe, &xenos_pipe_funcs,
					   formats, ARRAY_SIZE(formats), NULL,
					   connector);
	if (ret)
		return ret;

	drm_mode_config_reset(dev);
	return 0;
}

/* ------------------------------------------------------------------ */

DEFINE_DRM_GEM_FOPS(xenos_fops);

static const struct drm_driver xenos_driver = {
	.driver_features = DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC,

	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.date = DRIVER_DATE,
	.major = DRIVER_MAJOR,
	.minor = DRIVER_MINOR,

	.fops = &xenos_fops,
	DRM_GEM_SHMEM_DRIVER_OPS,
};

static int xenos_pci_probe(struct pci_dev *pdev,
			   const struct pci_device_id *ent)
{
	struct drm_device *dev;
	struct xenos_device *xenos;
	int ret;

	ret = drm_aperture_remove_conflicting_pci_framebuffers(pdev,
							       &xenos_driver);
	if (ret)
		return ret;

	ret = pcim_enable_device(pdev);
	if (ret)
		return ret;

	ret = pci_request_regions(pdev, DRIVER_NAME);
	if (ret)
		return ret;

	ret = -ENOMEM;
	xenos = devm_drm_dev_alloc(&pdev->dev, &xenos_driver,
				   struct xenos_device, dev);
	if (IS_ERR(xenos))
		return PTR_ERR(xenos);

	dev = &xenos->dev;

	xenos->regs = devm_ioremap(&pdev->dev, pci_resource_start(pdev, 0),
				   pci_resource_len(pdev, 0));
	if (xenos->regs == NULL)
		return -ENOMEM;

	ret = xenos_load(xenos);
	if (ret)
		return ret;

	pci_set_drvdata(pdev, dev);
	ret = drm_dev_register(dev, 0);
	if (ret)
		return ret;

	drm_fbdev_generic_setup(dev, 32);
	return 0;
}

static void xenos_pci_remove(struct pci_dev *pdev)
{
	struct drm_device *dev = pci_get_drvdata(pdev);

	drm_dev_unplug(dev);
}

static const struct pci_device_id xenos_pci_tbl[] = {
	{ PCI_VDEVICE(MICROSOFT, 0x5811), 0 }, /* xenon */
	{ PCI_VDEVICE(MICROSOFT, 0x5821), 0 }, /* zephyr/falcon */
	{ PCI_VDEVICE(MICROSOFT, 0x5831), 0 }, /* jasper */
	{ PCI_VDEVICE(MICROSOFT, 0x5841), 0 }, /* slim */

	{} /* terminate list */
};
static struct pci_driver xenos_pci_driver = {
	.name = DRIVER_NAME,
	.id_table = xenos_pci_tbl,
	.probe = xenos_pci_probe,
	.remove = xenos_pci_remove,
};

drm_module_pci_driver(xenos_pci_driver);
MODULE_DEVICE_TABLE(pci, xenos_pci_tbl);
MODULE_LICENSE("GPL");
