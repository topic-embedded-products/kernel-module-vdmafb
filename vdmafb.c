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
			height = <768>;
		};
 */

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
#define VDMAFB_CONTROL	0

/* Register control flags */
#define VDMAFB_CONTROL_ENABLE 1


struct vdmafb_dev {
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

static int vdmafb_setupfb(struct vdmafb_dev *fbdev)
{
	struct fb_var_screeninfo *var = &fbdev->info.var;
	struct device *dev = fbdev->info.device;
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

	/* Enable output */
	vdmafb_writereg(fbdev, VDMAFB_CONTROL, VDMAFB_CONTROL_ENABLE);

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
	var->pixclock = KHZ2PICOS(65000);
	var->vmode = FB_VMODE_NONINTERLACED;
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

	unregister_framebuffer(&fbdev->info);
	/* Disable display */
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
