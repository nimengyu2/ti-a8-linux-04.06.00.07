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
		.flags		= UPF_BOOT_AUTOCONF | UPF_IOREMAP,
		.uartclk = M_16C554_UARTCLK	
	},
	[1] = {			
		.irqflags = IRQF_TRIGGER_RISING,	
		.iotype		= UPIO_PORT,		
		.flags		= UPF_BOOT_AUTOCONF | UPF_IOREMAP,
		.uartclk = M_16C554_UARTCLK	
	},
	[2] = {			
		.irqflags = IRQF_TRIGGER_RISING,	
		.iotype		= UPIO_PORT,		
		.flags		= UPF_BOOT_AUTOCONF | UPF_IOREMAP,
		.uartclk = M_16C554_UARTCLK	
	},
	[3] = {			
		.irqflags = IRQF_TRIGGER_RISING,	
		.iotype		= UPIO_PORT,		
		.flags		= UPF_BOOT_AUTOCONF | UPF_IOREMAP,
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


static int st16c554_gpmc_retime(void)
{
	struct gpmc_timings t;
	const int t3 = 10;	/* Figure 12.2 read and 12.4 write */
	const int t4_r = 20;	/* Figure 12.2 read */
	const int t4_w = 5;	/* Figure 12.4 write */
	const int t5 = 25;	/* Figure 12.2 read */
	const int t6 = 15;	/* Figure 12.2 read */
	const int t7 = 5;	/* Figure 12.4 write */
	const int t8 = 5;	/* Figure 12.4 write */
	const int t20 = 185;	/* Figure 12.2 read and 12.4 write */
	u32 l;

	memset(&t, 0, sizeof(t));

	/* Read timings */
	t.cs_on = 0;
	t.adv_on = t.cs_on;
	t.oe_on = t.adv_on + t3;
	t.access = t.oe_on + t5;
	t.oe_off = t.access;
	t.adv_rd_off = t.oe_off + max(t4_r, t6);
	t.cs_rd_off = t.oe_off;
	t.rd_cycle = t20 - t.oe_on;

	/* Write timings */
	t.we_on = t.adv_on + t3;

	if (cpu_is_omap34xx() && (gpmc_cfg->flags & GPMC_MUX_ADD_DATA)) {
		t.wr_data_mux_bus = t.we_on;
		t.we_off = t.wr_data_mux_bus + t7;
	} else
		t.we_off = t.we_on + t7;
	if (cpu_is_omap34xx())
		t.wr_access = t.we_off;
	t.adv_wr_off = t.we_off + max(t4_w, t8);
	t.cs_wr_off = t.we_off + t4_w;
	t.wr_cycle = t20 - t.we_on;

	l = GPMC_CONFIG1_DEVICESIZE_16;
	if (gpmc_cfg->flags & GPMC_MUX_ADD_DATA)
		l |= GPMC_CONFIG1_MUXADDDATA;
	if (gpmc_cfg->flags & GPMC_READ_MON)
		l |= GPMC_CONFIG1_WAIT_READ_MON;
	if (gpmc_cfg->flags & GPMC_WRITE_MON)
		l |= GPMC_CONFIG1_WAIT_WRITE_MON;
	if (gpmc_cfg->wait_pin)
		l |= GPMC_CONFIG1_WAIT_PIN_SEL(gpmc_cfg->wait_pin);
	gpmc_cs_write_reg(gpmc_cfg->cs, GPMC_CS_CONFIG1, l);

	/*
	 * FIXME: Calculate the address and data bus muxed timings.
	 * Note that at least adv_rd_off needs to be changed according
	 * to omap3430 TRM Figure 11-11. Are the sdp boards using the
	 * FPGA in between smc91x and omap as the timings are different
	 * from above?
	 */
	if (gpmc_cfg->flags & GPMC_MUX_ADD_DATA)
		return 0;

	return gpmc_cs_set_timings(gpmc_cfg->cs, &t);
}

#define STNOR_GPMC_CONFIG1 0x03
#define STNOR_GPMC_CONFIG2 0x001E1E01
#define STNOR_GPMC_CONFIG3 0x000E0E02
#define STNOR_GPMC_CONFIG4 0x1D0C1D0C
#define STNOR_GPMC_CONFIG5 0x011C1F1F
#define STNOR_GPMC_CONFIG6 0x00000FCF

static const u32 gpmc_nor[7] = {
	STNOR_GPMC_CONFIG1,
	STNOR_GPMC_CONFIG2,
	STNOR_GPMC_CONFIG3,
	STNOR_GPMC_CONFIG4,
	STNOR_GPMC_CONFIG5,
	STNOR_GPMC_CONFIG6, 
	0};


#define GPMC_CS 3
/*
 * Initialize smc91x device connected to the GPMC. Note that we
 * assume that pin multiplexing is done in the board-*.c file,
 * or in the bootloader.
 */
void __init gpmc_16c554_init(struct omap_16c554_platform_data *board_data)
{
	unsigned long cs_mem_base;
	int ret;
	unsigned long val;

	gpmc_cfg = board_data;
	gpmc_cfg->retime = st16c554_gpmc_retime;


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
	st16c554_platform_data[1].iobase = ((volatile unsigned long)ioremap(cs_mem_base + 0x08,1));
	st16c554_platform_data[1].irq =  1*32+23;
	st16c554_platform_data[2].iobase = ((volatile unsigned long)ioremap(cs_mem_base + 0x20,1));
	st16c554_platform_data[2].irq =  1*32+24;
	st16c554_platform_data[3].iobase = ((volatile unsigned long)ioremap(cs_mem_base + 0x28,1));
	st16c554_platform_data[3].irq =  1*32+25;
	lsd_dbg(LSD_OK,"st16c554_platform_data[0].iobase=0x%08x\n",st16c554_platform_data[0].iobase);
	lsd_dbg(LSD_OK,"st16c554_platform_data[1].iobase=0x%08x\n",st16c554_platform_data[1].iobase);
	lsd_dbg(LSD_OK,"st16c554_platform_data[2].iobase=0x%08x\n",st16c554_platform_data[2].iobase);
	lsd_dbg(LSD_OK,"st16c554_platform_data[3].iobase=0x%08x\n",st16c554_platform_data[3].iobase);

	#if 0
	if (gpmc_cfg->retime) {
		ret = gpmc_cfg->retime();
		if (ret != 0)
		{
			lsd_dbg(LSD_ERR,"gpmc_cfg->retime not ok\n");
		}
		else
		{
			lsd_dbg(LSD_OK,"gpmc_cfg->retime ok\n");
		}
	}
	#endif
	gpmc_cs_write_reg(GPMC_CS, GPMC_CS_CONFIG1, gpmc_nor[0]);

	gpmc_cs_write_reg(GPMC_CS, GPMC_CS_CONFIG2, gpmc_nor[1]);

	gpmc_cs_write_reg(GPMC_CS, GPMC_CS_CONFIG3, gpmc_nor[2]);

	gpmc_cs_write_reg(GPMC_CS, GPMC_CS_CONFIG4, gpmc_nor[3]);

	gpmc_cs_write_reg(GPMC_CS, GPMC_CS_CONFIG5, gpmc_nor[4]);

	gpmc_cs_write_reg(GPMC_CS, GPMC_CS_CONFIG6, gpmc_nor[5]);
	
	val = gpmc_cs_read_reg(GPMC_CS, GPMC_CS_CONFIG7);
	val |= (1 << 6);
	gpmc_cs_write_reg(GPMC_CS, GPMC_CS_CONFIG7, val);

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
	#if 0
	while(1)
	{
		__raw_writeb(0xAA, st16c554_platform_data[3].iobase); 
		udelay(10);
	}
	#endif

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
