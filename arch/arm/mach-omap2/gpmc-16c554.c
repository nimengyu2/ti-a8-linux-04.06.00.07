/*
 * linux/arch/arm/mach-omap2/gpmc-smc91x.c
 *
 * Copyright (C) 2009 Nokia Corporation
 * Contact:	Tony Lindgren
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/smc91x.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/serial_8250.h>

#include <plat/board.h>
#include <plat/gpmc.h>
#include <plat/gpmc-16c554.h>
#include <linux/lierda_debug.h>

static struct omap_16c554_platform_data *gpmc_cfg;
#define	M_16C554_UARTCLK 7372800
static struct plat_serial8250_port st16c554_platform_data[] = {
	[0] = {			
		.irqflags = IRQF_TRIGGER_RISING,	
		.iotype		= UPIO_PORT,		
		.flags		= UPF_BOOT_AUTOCONF,
		.uartclk = M_16C554_UARTCLK	
	},
	[1] = {			
		.irqflags = IRQF_TRIGGER_RISING,	
		.iotype		= UPIO_PORT,		
		.flags		= UPF_BOOT_AUTOCONF,
		.uartclk = M_16C554_UARTCLK	
	},
	[2] = {			
		.irqflags = IRQF_TRIGGER_RISING,	
		.iotype		= UPIO_PORT,		
		.flags		= UPF_BOOT_AUTOCONF,
		.uartclk = M_16C554_UARTCLK	
	},
	[3] = {			
		.irqflags = IRQF_TRIGGER_RISING,	
		.iotype		= UPIO_PORT,		
		.flags		= UPF_BOOT_AUTOCONF,
		.uartclk = M_16C554_UARTCLK	
	},
	[4] = {			
		/* end of array */
	},
};


static struct platform_device st16c554_device = {
	.name = "serial8250",
	.id = PLAT8250_DEV_PLATFORM,
	.dev = {
		.platform_data = st16c554_platform_data
	}
};


/*
 * Initialize smc91x device connected to the GPMC. Note that we
 * assume that pin multiplexing is done in the board-*.c file,
 * or in the bootloader.
 */
void __init gpmc_16c554_init(struct omap_16c554_platform_data *board_data)
{
	unsigned long cs_mem_base;
	int ret;

	gpmc_cfg = board_data;

	//if (gpmc_cfg->flags & GPMC_TIMINGS_SMC91C96)
	//	gpmc_cfg->retime = smc91c96_gpmc_retime;

	if (gpmc_cs_request(gpmc_cfg->cs, SZ_16M, &cs_mem_base) < 0) {
		//printk(KERN_ERR "Failed to request GPMC mem for smc91x\n");
		lsd_dbg(LSD_ERR,"gpmc_cs_request no ok\n");
		return;
	}
	else
	{
		lsd_dbg(LSD_OK,"gpmc_cs_request ok,gpmc_cfg->cs=%d,cs_mem_base=0x%08x\n",gpmc_cfg->cs,cs_mem_base);
	}

	st16c554_platform_data[0].iobase = ((volatile unsigned long)ioremap(cs_mem_base + 0x00,1));
	st16c554_platform_data[0].irq =  1*32+22;
	st16c554_platform_data[1].iobase = ((volatile unsigned long)ioremap(cs_mem_base + 0x20,1));
	st16c554_platform_data[1].irq =  1*32+23;
	st16c554_platform_data[2].iobase = ((volatile unsigned long)ioremap(cs_mem_base + 0x80,1));
	st16c554_platform_data[2].irq =  1*32+24;
	st16c554_platform_data[3].iobase = ((volatile unsigned long)ioremap(cs_mem_base + 0xA0,1));
	st16c554_platform_data[3].irq =  1*32+25;
	
	#if 0
	if (gpmc_cfg->retime) {
		ret = gpmc_cfg->retime();
		if (ret != 0)
			goto free1;
	}

	if (gpio_request_one(gpmc_cfg->gpio_irq, GPIOF_IN, "SMC91X irq") < 0)
		goto free1;

	gpmc_smc91x_resources[1].start = gpio_to_irq(gpmc_cfg->gpio_irq);

	if (gpmc_cfg->gpio_pwrdwn) {
		ret = gpio_request_one(gpmc_cfg->gpio_pwrdwn,
				       GPIOF_OUT_INIT_LOW, "SMC91X powerdown");
		if (ret)
			goto free2;
	}

	if (gpmc_cfg->gpio_reset) {
		ret = gpio_request_one(gpmc_cfg->gpio_reset,
				       GPIOF_OUT_INIT_LOW, "SMC91X reset");
		if (ret)
			goto free3;

		gpio_set_value(gpmc_cfg->gpio_reset, 1);
		msleep(100);
		gpio_set_value(gpmc_cfg->gpio_reset, 0);
	}
	#endif

	if (platform_device_register(&st16c554_device) < 0) {
		//printk(KERN_ERR "Unable to register smc91x device\n");
		lsd_dbg(LSD_ERR,"platform_device_register(&16c554_device) not ok\n");
	}
	else
	{
		lsd_dbg(LSD_OK,"platform_device_register(&16c554_device) ok\n");
	}

	return;

free3:
	//if (gpmc_cfg->gpio_pwrdwn)
	//	gpio_free(gpmc_cfg->gpio_pwrdwn);
free2:
	//gpio_free(gpmc_cfg->gpio_irq);
free1:
	gpmc_cs_free(gpmc_cfg->cs);

	printk(KERN_ERR "Could not initialize 16c554\n");
}
