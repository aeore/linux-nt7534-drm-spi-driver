// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DRM driver for Novatek NT7534 panels
 *
 * Copyright 2026 Yuriy Gurin <ygurin@outlook.com>
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/spi/spi.h>
#include <linux/backlight.h>
#include <video/mipi_display.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fbdev_generic.h>
#include <drm/drm_format_helper.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_mipi_dbi.h>
#include <drm/drm_rect.h>
#include <drm/drm_vblank.h>

/* controller-specific commands */
#define NT7534_POWERCONTROL_EXTERNAL  0x0
#define NT7534_POWERCONTROL_VF_ONLY   0x1
#define NT7534_POWERCONTROL_VFVR_ONLY 0x3
#define NT7534_POWERCONTROL_INTERNAL  0x7

#define NT7534_BIAS_TYPE1  0x0
#define NT7534_BIAS_TYPE2  0x1

static void nt7534_display_physical_reset(struct mipi_dbi *dbi) {
	gpiod_set_value(dbi->reset, 1);
	msleep(100);
	gpiod_set_value(dbi->reset, 0);
	msleep(100);
}

static void nt7534_spi_display_reset(struct mipi_dbi *dbi) {
	mipi_dbi_command(dbi, 0xE2);
}

static void nt7534_spi_display_onoff(struct mipi_dbi *dbi, bool on) {
	mipi_dbi_command(dbi, (on ? 0xAF : 0xAE));
}

static void nt7534_spi_display_setOutputStatus(struct mipi_dbi *dbi, bool normal) {
	mipi_dbi_command(dbi, (normal ? 0xC0 : 0xC8));
}

static void nt7534_spi_display_setPowerControl(struct mipi_dbi *dbi, u8 powerMask) {
	mipi_dbi_command(dbi, (0x28 | (powerMask & 0x07)));
}

static void nt7534_spi_display_setPage(struct mipi_dbi *dbi, u8 pageNo) {
	mipi_dbi_command(dbi, (0xB0 | (pageNo & 0x0F)));
}

static void nt7534_spi_display_setResistorRatio(struct mipi_dbi *dbi, u8 ratioSelector) {
	mipi_dbi_command(dbi, (0x20 | (ratioSelector & 0x07)));
}

static void nt7534_spi_display_setADC(struct mipi_dbi *dbi, bool normal) {
	mipi_dbi_command(dbi, (normal ? 0xA0 : 0xA1));
}

static void nt7534_spi_display_setBias(struct mipi_dbi *dbi, u8 bias) {
	mipi_dbi_command(dbi, (0xA2 | (bias & 0x01)));
}

static u32 nt7534_get_pixel_xrgb8888(void *vaddr, struct drm_framebuffer *fb, int x, int y) {
	u8 *row = (u8 *)vaddr + y * fb->pitches[0];
	u32 *pixel_addr = (u32 *)(row + x * 4);
	return *pixel_addr;
}

static void nt7534_fb_dirty(struct iosys_map *src, struct drm_framebuffer *fb,
							struct drm_rect *rect) {
	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(fb->dev);
	struct mipi_dbi *dbi = &dbidev->dbi;
	void *vaddr = src->vaddr;
	int ret;
	u8 *buffer = (u8*)dbidev->tx_buf;

	ret = drm_gem_fb_begin_cpu_access(fb, DMA_FROM_DEVICE);
	if (ret) {
		goto err_msg;
    }

	int num_pages = (rect->y2 - 1) / 8 - rect->y1 / 8 + 1;
	int width = rect->x2 - rect->x1;
	for (int p_idx = 0; p_idx < num_pages; p_idx++) {
		int p = rect->y1 / 8 + p_idx;
		for (int x = rect->x1; x < rect->x2; x++) {
			u8 byte = 0;
			for (int bit = 0; bit < 8; bit++) {
				int y = p * 8 + bit;
				if (y < fb->height && y < rect->y2) {
					if (nt7534_get_pixel_xrgb8888(vaddr, fb, x, y) & 0x00FFFFFF) {
						byte |= (1 << bit);
					}
				}
			}
			buffer[p_idx * width + (x - rect->x1)] = byte;
		}
	}
	drm_gem_fb_end_cpu_access(fb, DMA_FROM_DEVICE);

	for (int p_idx = 0; p_idx < num_pages; p_idx++) {
		int pageToUpdate = rect->y1 / 8 + p_idx;

		nt7534_spi_display_setPage(dbi, pageToUpdate);
		mipi_dbi_command(dbi, (0x00 | (rect->x1 & 0xf)));

		ret = mipi_dbi_command_buf(dbi, (0x10 | ((rect->x1 >> 4) & 0xf)), &buffer[p_idx * width],
									width);
		if (ret) {
			dev_err(fb->dev->dev, "SPI transfer failed: %d\n", ret);
		}
	}

err_msg:
	if (ret) {
		dev_err_once(fb->dev->dev, "Failed to update display %d\n", ret);
	}
}

static void nt7534_pipe_update(struct drm_simple_display_pipe *pipe,
								struct drm_plane_state *old_state) {
	struct drm_plane_state *state = pipe->plane.state;
	struct drm_shadow_plane_state *shadow_plane_state = to_drm_shadow_plane_state(state);
	struct drm_framebuffer *fb = state->fb;
	struct drm_rect rect;
	int idx;

	if (!pipe->crtc.state->active) {
		return;
	}

	if (!drm_dev_enter(fb->dev, &idx)) {
		return;
	}

	if (drm_atomic_helper_damage_merged(old_state, state, &rect)) {
		nt7534_fb_dirty(&shadow_plane_state->data[0], fb, &rect);
	}

	if (pipe->crtc.state->event) {
		spin_lock_irq(&pipe->crtc.dev->event_lock);
		drm_crtc_send_vblank_event(&pipe->crtc, pipe->crtc.state->event);
		pipe->crtc.state->event = NULL;
		spin_unlock_irq(&pipe->crtc.dev->event_lock);
	}

	drm_dev_exit(idx);
}

static void nt7534_pipe_enable(struct drm_simple_display_pipe *pipe, 
								struct drm_crtc_state *crtc_state,
								struct drm_plane_state *plane_state) {
	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(pipe->crtc.dev);
	struct drm_shadow_plane_state *shadow_plane_state = to_drm_shadow_plane_state(plane_state);
	struct drm_framebuffer *fb = plane_state->fb;
	struct mipi_dbi *dbi = &dbidev->dbi;
	struct drm_rect rect = {
		.x1 = 0,
		.x2 = fb->width,
		.y1 = 0,
		.y2 = fb->height,
	};
	int idx, ret;

	if (!drm_dev_enter(pipe->crtc.dev, &idx)) {
		return;
	}

	DRM_DEBUG_KMS("\n");

	ret = mipi_dbi_poweron_reset(dbidev);
	if (ret) {
		dev_err(fb->dev->dev, "Failed to power on and reset display: %d\n", ret);
		goto out_exit;
	}

	nt7534_display_physical_reset(dbi);
	nt7534_spi_display_reset(dbi);
	msleep(50);

	nt7534_spi_display_onoff(dbi, false);
	nt7534_spi_display_setBias(dbi, NT7534_BIAS_TYPE1);
	nt7534_spi_display_setADC(dbi, true);
	nt7534_spi_display_setOutputStatus(dbi, false);
	nt7534_spi_display_setResistorRatio(dbi, 0x04);
	nt7534_spi_display_setPowerControl(dbi, NT7534_POWERCONTROL_INTERNAL);
	msleep(50);

	nt7534_spi_display_onoff(dbi, true);

	if (dbidev->backlight) {
        dbidev->backlight->props.power = FB_BLANK_UNBLANK;
        dbidev->backlight->props.brightness = 1; // или твой уровень
        backlight_update_status(dbidev->backlight);
    }

	nt7534_fb_dirty(&shadow_plane_state->data[0], fb, &rect);

out_exit:
	drm_dev_exit(idx);
}

static void nt7534_pipe_disable(struct drm_simple_display_pipe *pipe) {
	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(pipe->crtc.dev);
	/*
	 * This callback is not protected by drm_dev_enter/exit since we want to
	 * turn off the display on regular driver unload. It's highly unlikely
	 * that the underlying SPI controller is gone should this be called after
	 * unplug.
	 */

	DRM_DEBUG_KMS("\n");

	nt7534_spi_display_onoff(&dbidev->dbi, false);
}

static const u32 nt7534_formats[] = {
	DRM_FORMAT_XRGB8888,
};

static const struct drm_simple_display_pipe_funcs nt7534_pipe_funcs = {
	.mode_valid            = mipi_dbi_pipe_mode_valid,
	.enable                = nt7534_pipe_enable,
	.disable               = nt7534_pipe_disable,
	.update                = nt7534_pipe_update,
	.begin_fb_access       = mipi_dbi_pipe_begin_fb_access,
	.end_fb_access         = mipi_dbi_pipe_end_fb_access,
	.reset_plane           = mipi_dbi_pipe_reset_plane,
	.duplicate_plane_state = mipi_dbi_pipe_duplicate_plane_state,
	.destroy_plane_state   = mipi_dbi_pipe_destroy_plane_state,
};

static const struct drm_display_mode nt7534_mode = {
	DRM_SIMPLE_MODE(128, 64, 32, 22),
};

DEFINE_DRM_GEM_DMA_FOPS(nt7534_fops);

static const struct drm_driver nt7534_driver = {
	.driver_features    = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops               = &nt7534_fops,
	DRM_GEM_DMA_DRIVER_OPS_VMAP,
	.debugfs_init       = mipi_dbi_debugfs_init,
	.name               = "nt7534",
	.desc               = "Novatek NT7534",
	.date               = "20260106",
	.major              = 1,
	.minor              = 0,
};

static const struct of_device_id nt7534_of_match[] = {
	{ .compatible = "novatek,nt7534" },
	{},
};
MODULE_DEVICE_TABLE(of, nt7534_of_match);

static const struct spi_device_id nt7534_id[] = {
	{ "nt7534", 0 },
	{ },
};
MODULE_DEVICE_TABLE(spi, nt7534_id);

static int nt7534_probe(struct spi_device *spi) {
	struct device *dev = &spi->dev;
	struct mipi_dbi_dev *dbidev;
	struct drm_device *drm;
	struct mipi_dbi *dbi;
	struct gpio_desc *a0;
	u32 rotation = 0;
	size_t bufsize;
	int ret;

	dbidev = devm_drm_dev_alloc(dev, &nt7534_driver, struct mipi_dbi_dev, drm);
	if (IS_ERR(dbidev)) {
		return PTR_ERR(dbidev);
	}

	dbi = &dbidev->dbi;
	drm = &dbidev->drm;

	bufsize = (nt7534_mode.vdisplay * nt7534_mode.hdisplay);

	dbi->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(dbi->reset)) {
		return dev_err_probe(dev, PTR_ERR(dbi->reset), "Failed to get GPIO 'reset'\n");
	}

	a0 = devm_gpiod_get(dev, "a0", GPIOD_OUT_LOW);
	if (IS_ERR(a0)) {
		return dev_err_probe(dev, PTR_ERR(a0), "Failed to get GPIO 'a0'\n");
	}

	device_property_read_u32(dev, "rotation", &rotation);

	dbidev->backlight = devm_of_find_backlight(dev);

	ret = mipi_dbi_spi_init(spi, dbi, a0);
	if (ret) {
		return ret;
	}

	/* Read from this controller is not supported */
	dbi->read_commands = NULL;

	ret = mipi_dbi_dev_init_with_formats(dbidev, &nt7534_pipe_funcs,
						nt7534_formats, ARRAY_SIZE(nt7534_formats),
						&nt7534_mode, rotation, bufsize);
	if (ret) {
		return ret;
	}

	drm_mode_config_reset(drm);

	ret = drm_dev_register(drm, 0);
	if (ret) {
		return ret;
	}

	spi_set_drvdata(spi, drm);
	drm_fbdev_generic_setup(drm, 0);

	return 0;
}

static void nt7534_remove(struct spi_device *spi) {
	struct drm_device *drm = spi_get_drvdata(spi);

	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);
}

static void nt7534_shutdown(struct spi_device *spi) {
	drm_atomic_helper_shutdown(spi_get_drvdata(spi));
}

static struct spi_driver nt7534_spi_driver = {
	.driver = {
		.name = "nt7534",
		.owner = THIS_MODULE,
		.of_match_table = nt7534_of_match,
	},
	.id_table = nt7534_id,
	.probe = nt7534_probe,
	.remove = nt7534_remove,
	.shutdown = nt7534_shutdown,
};
module_spi_driver(nt7534_spi_driver);

MODULE_DESCRIPTION("Novatek NT7534 DRM driver");
MODULE_AUTHOR("Yuriy Gurin <ygurin@outlook.com>");
MODULE_LICENSE("GPL");
