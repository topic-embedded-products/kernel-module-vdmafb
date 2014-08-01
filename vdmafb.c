/*
 * AXI VDMA frame buffer driver
 * Based on: ocfb.c, simplefb.c, axi_hdmi_crtc.c
 *
 * Copyright (C) 2014 Topic Embedded Products
 * Author: Mike Looijmans <mike.looijmans@topic.nl>
 *
 * Licensed under the GPL-2.
 *
 * Example devicetree contents:
 		axi_vdma_vga: axi_vdma_vga@7e000000 {
			compatible = "topic,vdma-fb";
			reg = <0x7e000000 0x10000>;
			dmas = <&axi_vdma_0 0>;
			dma-names = "video";
			width = <1024>;
			height = <600>;
			horizontal-front-porch = <160>;
			horizontal-back-porch = <160>;
			horizontal-sync = <136>;
			vertical-front-porch = <17>;
			vertical-back-porch = <18>;
		};
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/amba/xilinx_dma.h>

/* Register locations */
#define VDMAFB_CONTROL	0x00

#define VDMAFB_HORIZONTAL_TOTAL	0x04
#define VDMAFB_HORIZONTAL_WIDTH	0x08
#define VDMAFB_HORIZONTAL_SYNC	0x0C
#define VDMAFB_HORIZONTAL_FRONT_PORCH	0x10
#define VDMAFB_HORIZONTAL_BACK_PORCH	0x14
#define VDMAFB_HORIZONTAL_POLARITY	0x18

#define VDMAFB_VERTICAL_TOTAL	0x1C
#define VDMAFB_VERTICAL_HEIGHT	0x20
#define VDMAFB_VERTICAL_SYNC	0x24
#define VDMAFB_VERTICAL_FRONT_PORCH	0x28
#define VDMAFB_VERTICAL_BACK_PORCH	0x2C
#define VDMAFB_VERTICAL_POLARITY	0x30

#define VDMAFB_BACKLIGHT_CONTROL	0x50
#define VDMAFB_BACKLIGHT_LEVEL_1K	0x54

/* Register control flags */
#define VDMAFB_CONTROL_ENABLE 1

struct vdmafb_dev {
	struct backlight_device *backlight;
	struct fb_info info;
	void __iomem *regs;
	/* Physical and virtual addresses of framebuffer */
	phys_addr_t fb_phys;
	void __iomem *fb_virt;
	/* VDMA handle */
	struct dma_chan *dma;
	struct xilinx_dma_config dma_config;
	/* Palette data */
	u32 pseudo_palette[16];
};

static inline u32 vdmafb_readreg(struct vdmafb_dev *fbdev, loff_t offset)
{
	return ioread32(fbdev->regs + offset);
}

static inline void vdmafb_writereg(struct vdmafb_dev *fbdev, loff_t offset, u32 data)
{
	iowrite32(data, fbdev->regs + offset);
}

static int vdmafb_bl_update_status(struct backlight_device *bl)
{
	struct vdmafb_dev *fbdev = bl_get_data(bl);
	int	brightness = bl->props.brightness;
	u32 power = 1;

	pr_debug("%s: b=%d p=%d\n",
		__func__, brightness, bl->props.power);

	if (bl->props.power != 0)
		power = 0;
	vdmafb_writereg(fbdev, VDMAFB_BACKLIGHT_CONTROL, power);
	vdmafb_writereg(fbdev, VDMAFB_BACKLIGHT_LEVEL_1K, brightness);
	return 0;
}

static int vdmafb_bl_get_brightness(struct backlight_device *bl)
{
	struct vdmafb_dev *fbdev = bl_get_data(bl);

	return vdmafb_readreg(fbdev, VDMAFB_BACKLIGHT_LEVEL_1K);
}

static const struct backlight_ops vdmafb_bl_ops = {
	.update_status = vdmafb_bl_update_status,
	.get_brightness = vdmafb_bl_get_brightness,
};

static int vdmafb_setupfb(struct vdmafb_dev *fbdev)
{
	struct fb_var_screeninfo *var = &fbdev->info.var;
	struct dma_async_tx_descriptor *desc;

	/* Disable display */
	vdmafb_writereg(fbdev, VDMAFB_CONTROL, 0);

	/* Setup VDMA address etc */
	dmaengine_terminate_all(fbdev->dma);

	fbdev->dma_config.hsize = var->xres * 4;
	fbdev->dma_config.vsize = var->yres;
	fbdev->dma_config.stride = var->xres * 4;

	dmaengine_device_control(fbdev->dma, DMA_SLAVE_CONFIG,
		(unsigned long)&fbdev->dma_config);

	desc = dmaengine_prep_slave_single(fbdev->dma,
				fbdev->fb_phys,
				fbdev->dma_config.vsize * fbdev->dma_config.stride,
				DMA_MEM_TO_DEV, 0);
	if (!desc) {
		pr_err("Failed to prepare DMA descriptor\n");
		return -ENOMEM;
	} else {
		dmaengine_submit(desc);
		dma_async_issue_pending(fbdev->dma);
	}

	/* Configure IP via registers */
	vdmafb_writereg(fbdev, VDMAFB_HORIZONTAL_TOTAL,
		var->hsync_len + var->left_margin + var->xres + var->right_margin);
	vdmafb_writereg(fbdev, VDMAFB_HORIZONTAL_SYNC, var->hsync_len);
	vdmafb_writereg(fbdev, VDMAFB_HORIZONTAL_FRONT_PORCH, var->left_margin);
	vdmafb_writereg(fbdev, VDMAFB_HORIZONTAL_WIDTH, var->xres);
	vdmafb_writereg(fbdev, VDMAFB_HORIZONTAL_BACK_PORCH, var->right_margin);
	vdmafb_writereg(fbdev, VDMAFB_HORIZONTAL_POLARITY, 0); /* TODO */
	vdmafb_writereg(fbdev, VDMAFB_VERTICAL_TOTAL,
		var->vsync_len + var->upper_margin + var->yres + var->lower_margin);
	vdmafb_writereg(fbdev, VDMAFB_VERTICAL_SYNC, var->vsync_len);
	vdmafb_writereg(fbdev, VDMAFB_VERTICAL_FRONT_PORCH, var->upper_margin);
	vdmafb_writereg(fbdev, VDMAFB_VERTICAL_HEIGHT, var->yres);
	vdmafb_writereg(fbdev, VDMAFB_VERTICAL_BACK_PORCH, var->lower_margin);
	vdmafb_writereg(fbdev, VDMAFB_VERTICAL_POLARITY, 0);
	/* Enable output */
	vdmafb_writereg(fbdev, VDMAFB_CONTROL, VDMAFB_CONTROL_ENABLE);

	/* Set brightness */

	vdmafb_writereg(fbdev, VDMAFB_BACKLIGHT_CONTROL, 1);
	vdmafb_writereg(fbdev, VDMAFB_BACKLIGHT_LEVEL_1K, 800);

	return 0;
}

static void vdmafb_init_fix(struct vdmafb_dev *fbdev)
{
	struct fb_var_screeninfo *var = &fbdev->info.var;
	struct fb_fix_screeninfo *fix = &fbdev->info.fix;

	strcpy(fix->id, "vdma-fb");
	fix->line_length = var->xres * (var->bits_per_pixel/8);
	fix->smem_len = fix->line_length * var->yres;
	fix->type = FB_TYPE_PACKED_PIXELS;
	fix->visual = FB_VISUAL_TRUECOLOR;
}

static void vdmafb_init_var(struct vdmafb_dev *fbdev, struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct fb_var_screeninfo *var = &fbdev->info.var;
	int ret;

	ret = of_property_read_u32(np, "width", &var->xres);
	if (ret) {
		dev_err(&pdev->dev, "Can't parse width property, assume 1024\n");
		var->xres = 1024;
	}

	ret = of_property_read_u32(np, "height", &var->yres);
	if (ret) {
		dev_err(&pdev->dev, "Can't parse height property, assume 768\n");
		var->yres = 768;
	}

	var->accel_flags = FB_ACCEL_NONE;
	var->activate = FB_ACTIVATE_NOW;
	var->xres_virtual = var->xres;
	var->yres_virtual = var->yres;
	var->bits_per_pixel = 32;
	/* Clock settings */
	var->pixclock = KHZ2PICOS(51200);
	var->vmode = FB_VMODE_NONINTERLACED;
	of_property_read_u32(np, "horizontal-sync", &var->hsync_len);
	of_property_read_u32(np, "horizontal-front-porch", &var->left_margin);
	of_property_read_u32(np, "horizontal-back-porch", &var->right_margin);
	of_property_read_u32(np, "vertical-sync", &var->vsync_len);
	of_property_read_u32(np, "vertical-front-porch", &var->upper_margin);
	of_property_read_u32(np, "vertical-back-porch", &var->lower_margin);
	/* TODO: sync */
	/* 32 BPP */
	var->transp.offset = 24;
	var->transp.length = 8;
	var->red.offset = 16;
	var->red.length = 8;
	var->green.offset = 8;
	var->green.length = 8;
	var->blue.offset = 0;
	var->blue.length = 8;
}

static int vdmafb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			      u_int transp, struct fb_info *info)
{
	u32 *pal = info->pseudo_palette;
	u32 cr = red >> (16 - info->var.red.length);
	u32 cg = green >> (16 - info->var.green.length);
	u32 cb = blue >> (16 - info->var.blue.length);
	u32 value;

	if (regno >= 16)
		return -EINVAL;

	value = (cr << info->var.red.offset) |
		(cg << info->var.green.offset) |
		(cb << info->var.blue.offset);
	if (info->var.transp.length > 0) {
		u32 mask = (1 << info->var.transp.length) - 1;
		mask <<= info->var.transp.offset;
		value |= mask;
	}
	pal[regno] = value;

	return 0;
}

static struct fb_ops vdmafb_ops = {
	.owner		= THIS_MODULE,
	.fb_setcolreg	= vdmafb_setcolreg,
	.fb_fillrect	= sys_fillrect,
	.fb_copyarea	= sys_copyarea,
	.fb_imageblit	= sys_imageblit,
};

static int vdmafb_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct vdmafb_dev *fbdev;
	struct resource *res;
	int fbsize;
	struct backlight_properties props;
	struct backlight_device *bl;

	fbdev = devm_kzalloc(&pdev->dev, sizeof(*fbdev), GFP_KERNEL);
	if (!fbdev)
		return -ENOMEM;

	platform_set_drvdata(pdev, fbdev);

	fbdev->info.fbops = &vdmafb_ops;
	fbdev->info.device = &pdev->dev;
	fbdev->info.par = fbdev;

	vdmafb_init_var(fbdev, pdev);
	vdmafb_init_fix(fbdev);

	/* Request I/O resource */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "I/O resource request failed\n");
		return -ENXIO;
	}
	res->flags &= ~IORESOURCE_CACHEABLE;
	fbdev->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(fbdev->regs))
		return PTR_ERR(fbdev->regs);

	/* Allocate framebuffer memory */
	fbsize = fbdev->info.fix.smem_len;
	fbdev->fb_virt = dma_alloc_coherent(&pdev->dev, PAGE_ALIGN(fbsize),
					    &fbdev->fb_phys, GFP_KERNEL);
	if (!fbdev->fb_virt) {
		dev_err(&pdev->dev,
			"Frame buffer memory allocation failed\n");
		return -ENOMEM;
	}
	fbdev->info.fix.smem_start = fbdev->fb_phys;
	fbdev->info.screen_base = fbdev->fb_virt;
	fbdev->info.pseudo_palette = fbdev->pseudo_palette;

	pr_debug("%s virt=%p phys=%x size=%d\n", __func__,
		fbdev->fb_virt, fbdev->fb_phys, fbsize);

	/* Clear framebuffer */
	memset_io(fbdev->fb_virt, 0, fbsize);

	fbdev->dma = dma_request_slave_channel(&pdev->dev, "video");
	if (IS_ERR_OR_NULL(fbdev->dma)) {
		ret = PTR_ERR(fbdev->dma);
		dev_err(&pdev->dev, "Failed to allocate DMA channel (%d).\n", ret);
		goto err_dma_free;
	}

	/* Setup and enable the framebuffer */
	vdmafb_setupfb(fbdev);

	ret = fb_alloc_cmap(&fbdev->info.cmap, 256, 0);
	if (ret) {
		dev_err(&pdev->dev, "fb_alloc_cmap failed\n");
	}

	/* Register framebuffer */
	ret = register_framebuffer(&fbdev->info);
	if (ret) {
		dev_err(&pdev->dev, "Framebuffer registration failed\n");
		goto err_channel_free;
	}

	/* Register backlight */
	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = 1023;
	bl = backlight_device_register("backlight", &pdev->dev, fbdev,
				       &vdmafb_bl_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "error %ld on backlight register\n",
				PTR_ERR(bl));
	} else {
		fbdev->backlight = bl;
		bl->props.power = FB_BLANK_UNBLANK;
		bl->props.fb_blank = FB_BLANK_UNBLANK;
		bl->props.brightness = vdmafb_bl_get_brightness(bl);
	}

	return 0;

err_channel_free:
	dma_release_channel(fbdev->dma);
err_dma_free:
	dma_free_coherent(&pdev->dev, PAGE_ALIGN(fbsize), fbdev->fb_virt,
			  fbdev->fb_phys);

	return ret;
}

static int vdmafb_remove(struct platform_device *pdev)
{
	struct vdmafb_dev *fbdev = platform_get_drvdata(pdev);

	if (fbdev->backlight)
		backlight_device_unregister(fbdev->backlight);
	unregister_framebuffer(&fbdev->info);
	/* Disable display */
	vdmafb_writereg(fbdev, VDMAFB_BACKLIGHT_CONTROL, 0);
	vdmafb_writereg(fbdev, VDMAFB_CONTROL, 0);
	dma_release_channel(fbdev->dma);
	dma_free_coherent(&pdev->dev, PAGE_ALIGN(fbdev->info.fix.smem_len),
			  fbdev->fb_virt, fbdev->fb_phys);
	fb_dealloc_cmap(&fbdev->info.cmap);
	return 0;
}

static struct of_device_id vdmafb_match[] = {
	{ .compatible = "topic,vdma-fb", },
	{},
};
MODULE_DEVICE_TABLE(of, vdmafb_match);

static struct platform_driver vdmafb_driver = {
	.probe  = vdmafb_probe,
	.remove	= vdmafb_remove,
	.driver = {
		.name = "vdmafb_fb",
		.of_match_table = vdmafb_match,
	}
};
module_platform_driver(vdmafb_driver);

MODULE_AUTHOR("Mike Looijmans <mike.looijmans@topic.nl>");
MODULE_DESCRIPTION("Driver for VDMA controlled framebuffer");
MODULE_LICENSE("GPL v2");
