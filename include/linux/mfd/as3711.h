/*
 * as3711.h definitions
 *
 * Copyright (C) 2012 ams AG
 *
 * Author: Roman Schneider <roman.schneider@ams.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#ifndef __LINUX_MFD_AS3711_H
#define __LINUX_MFD_AS3711_H

#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/power_supply.h>
#include <linux/regulator/machine.h>

#define AS3711_REGISTER_COUNT 			0x92
#define AS3711_NUM_REGULATORS			17
#define AS3711_NUM_STEPDOWN_REGULATORS		3
#define AS3711_NUM_GPIO				4
#define AS3711_GPIO_IRQ_BASE			0

#define AS3711_REG_INIT(reg_offset, reg_value)  \
{						\
        .reg     = (reg_offset),     		\
        .val     = (reg_value),			\
}

#define AS3711_REG_INIT_TERMINATE		0xFF

#define AS3711_LDO1_ANA         		0
#define AS3711_LDO2_ANA         		1
#define AS3711_LDO3_DIG         		2
#define AS3711_LDO4_DIG         		3
#define AS3711_LDO5_DIG         		4
#define AS3711_LDO6_DIG         		5
#define AS3711_LDO7_DIG         		6
#define AS3711_LDO8_DIG         		7
#define AS3711_SD1              		8
#define AS3711_SD2              		9
#define AS3711_SD3              		10
#define AS3711_SD4              		11
#define AS3711_SU1              		12
#define AS3711_SU2              		13
#define AS3711_CURR1            		14
#define AS3711_CURR2            		15
#define AS3711_CURR3            		16

#define AS3711_ADDR_ASIC_ID1 			0x90
#define AS3711_ADDR_ASIC_ID2 			0x91

#define AS3711_LDO1_CONTROL_REG			0x04
#define AS3711_LDO2_CONTROL_REG			0x05
#define AS3711_LDO3_CONTROL_REG			0x06
#define AS3711_LDO4_CONTROL_REG			0x07
#define AS3711_LDO5_CONTROL_REG			0x08
#define AS3711_LDO6_CONTROL_REG			0x09
#define AS3711_LDO7_CONTROL_REG			0x0A
#define AS3711_LDO8_CONTROL_REG			0x0B
#define AS3711_GPIO1_CONTROL_REG		0x0C
#define AS3711_GPIO2_CONTROL_REG		0x0D
#define AS3711_GPIO3_CONTROL_REG		0x0E
#define AS3711_GPIO4_CONTROL_REG		0x0F
#define AS3711_GPIO_SIGNAL_OUT_REG		0x20
#define AS3711_GPIO_SIGNAL_IN_REG		0x21
#define AS3711_SD1_CONTROL_REG			0x00
#define AS3711_SD2_CONTROL_REG			0x01
#define AS3711_SD3_CONTROL_REG			0x02
#define AS3711_SD4_CONTROL_REG			0x03
#define AS3711_SU1_CONTROL_REG			0x50
#define AS3711_SU2_CONTROL_REG			0x51
#define AS3711_SU3_CONTROL_REG			0x52
#define AS3711_SU4_CONTROL_REG			0x53
#define AS3711_CURR1_CONTROL_REG		0x43
#define AS3711_CURR2_CONTROL_REG		0x44
#define AS3711_CURR3_CONTROL_REG		0x45
#define AS3711_RTC_CONTROL_REG			0x60
#define AS3711_RTC_SECOND_REG			0x61
#define AS3711_RTC_MINUTE1_REG			0x62
#define AS3711_RTC_MINUTE2_REG			0x63
#define AS3711_RTC_MINUTE3_REG			0x64
#define AS3711_RTC_ALARM_SECOND_REG		0x65
#define AS3711_RTC_ALARM_MINUTE1_REG		0x66
#define AS3711_RTC_ALARM_MINUTE2_REG		0x67
#define AS3711_RTC_ALARM_MINUTE3_REG		0x68

#define AS3711_LDO_ANA_ILIMIT_MASK		(1<<6)
#define AS3711_LDO_ANA_ILIMIT_BIT		(1<<6)
#define AS3711_LDO_ANA_ON_MASK		 	(1<<7)
#define AS3711_LDO_ANA_ON_BIT		 	(1<<7)
#define AS3711_LDO_ANA_VSEL_MASK 		0x1F

#define AS3711_LDO_DIG_ILIMIT_MASK		(1<<6)
#define AS3711_LDO_DIG_ILIMIT_BIT		(1<<6)
#define AS3711_LDO_DIG_ON_MASK		 	(1<<7)
#define AS3711_LDO_DIG_ON_BIT		 	(1<<7)
#define AS3711_LDO_DIG_VSEL_MASK 		0x3F

#define AS3711_SD_CONTROL_REG			0x10
#define AS3711_SD_CONTROL1_REG			0x30
#define AS3711_SD_CONTROL2_REG			0x31

#define AS3711_SD1_MODE_MASK			(1<<0)
#define AS3711_SD1_MODE_FAST			(1<<0)
#define AS3711_SD1_MODE_NORMAL			(0<<0)
#define AS3711_SD2_MODE_MASK			(1<<1)
#define AS3711_SD2_MODE_FAST			(1<<1)
#define AS3711_SD2_MODE_NORMAL			(0<<1)
#define AS3711_SD3_MODE_MASK			(1<<2)
#define AS3711_SD3_MODE_FAST			(1<<2)
#define AS3711_SD3_MODE_NORMAL			(0<<2)
#define AS3711_SD4_MODE_MASK			(1<<3)
#define AS3711_SD4_MODE_FAST			(1<<3)
#define AS3711_SD4_MODE_NORMAL			(0<<3)

#define AS3711_SD_VSEL_MASK 			0x7F
#define AS3711_SD1_ON				(1<<0)
#define AS3711_SD1_OFF				(0<<0)
#define AS3711_SD1_CTRL_MASK			(1<<0)
#define AS3711_SD2_ON				(1<<1)
#define AS3711_SD2_OFF				(0<<1)
#define AS3711_SD2_CTRL_MASK			(1<<1)
#define AS3711_SD3_ON				(1<<2)
#define AS3711_SD3_OFF				(0<<2)
#define AS3711_SD3_CTRL_MASK			(1<<2)
#define AS3711_SD4_ON				(1<<3)
#define AS3711_SD4_OFF				(0<<3)
#define AS3711_SD4_CTRL_MASK			(1<<3)

#define AS3711_SU_ON_BIT_MASK			(1<<0)
#define AS3711_SU_ON				(1<<0)
#define AS3711_SU_OFF				(0<<0)

#define AS3711_CURR_CONTROL_REG			0x40
#define AS3711_CURR_CSEL_MASK			0xFF
#define AS3711_CURR1_CTRL_MASK			0x03
#define AS3711_CURR1_SINK_ACTIVE		0x01
#define AS3711_CURR1_SINK_OFF			0x00
#define AS3711_CURR2_CTRL_MASK			0x0C
#define AS3711_CURR2_SINK_ACTIVE		0x04
#define AS3711_CURR2_SINK_OFF			0x00
#define AS3711_CURR3_CTRL_MASK			0xF0
#define AS3711_CURR3_SINK_ACTIVE		0x10
#define AS3711_CURR3_SINK_OFF			0x00

#define AS3711_CHARGERSTATUS1_REG		0x86
#define AS3711_CHST_CHDET			(1<<0)
#define AS3711_CHST_CCM				(1<<1)
#define AS3711_CHST_RESUME			(1<<2)
#define AS3711_CHST_TRICKLE			(1<<3)
#define AS3711_CHST_CVM				(1<<4)
#define AS3711_CHST_EOC				(1<<5)
#define AS3711_CHST_BATTEMP_HI			(1<<6)
#define AS3711_CHST_NOBAT			(1<<7)

#define AS3711_INTERRUPTMASK1_REG		0x74
#define AS3711_INTERRUPTMASK2_REG		0x75
#define AS3711_INTERRUPTMASK3_REG		0x76
#define AS3711_INTERRUPTSTATUS1_REG		0x77
#define AS3711_INTERRUPTSTATUS2_REG		0x78
#define AS3711_INTERRUPTSTATUS3_REG		0x79

#define AS3711_IRQ_HANDLER_MAX			5
#define AS3711_IRQ_HANDLER_CHARGER		0
#define AS3711_IRQ_HANDLER_ONKEY		1
#define AS3711_IRQ_HANDLER_WDT			2
#define AS3711_IRQ_HANDLER_RTC_REP		3
#define AS3711_IRQ_HANDLER_RTC_ALARM		4

#define AS3711_IRQ_MAX_HANDLER			10
#define AS3711_IRQ_TRICKLE			0
#define AS3711_IRQ_NOBAT			1
#define AS3711_IRQ_RESUME			2
#define AS3711_IRQ_EOC				3
#define AS3711_IRQ_CHDET			4
#define AS3711_IRQ_ONKEY			5
#define AS3711_IRQ_OVTMP			6
#define AS3711_IRQ_LOWBAT			7
#define AS3711_IRQ_RTC_REP			8
#define AS3711_IRQ_RTC_ALARM			9

#define AS3711_IRQ_MASK_TRICKLE			(1<<0)
#define AS3711_IRQ_MASK_NOBAT			(1<<1)
#define AS3711_IRQ_MASK_RESUME			(1<<2)
#define AS3711_IRQ_MASK_EOC			(1<<3)
#define AS3711_IRQ_MASK_CHDET			(1<<4)
#define AS3711_IRQ_MASK_ONKEY			(1<<5)
#define AS3711_IRQ_MASK_OVTMP			(1<<6)
#define AS3711_IRQ_MASK_LOWBAT			(1<<7)

#define AS3711_IRQ_MASK_RTC_REP			(1<<7)

#define AS3711_IRQ_MASK_RTC_ALARM		(1<<0)

#define AS3711_IRQ_BIT_TRICKLE			(1<<0)
#define AS3711_IRQ_BIT_NOBAT			(1<<1)
#define AS3711_IRQ_BIT_RESUME			(1<<2)
#define AS3711_IRQ_BIT_EOC			(1<<3)
#define AS3711_IRQ_BIT_CHDET			(1<<4)
#define AS3711_IRQ_BIT_ONKEY			(1<<5)
#define AS3711_IRQ_BIT_OVTMP			(1<<6)
#define AS3711_IRQ_BIT_LOWBAT			(1<<7)

#define AS3711_IRQ_BIT_RTC_REP			(1<<7)

#define AS3711_IRQ_BIT_RTC_ALARM		(1<<0)

#define AS3711_ADC_CONTROL_REG			0x70
#define AS3711_ADC_MSB_RESULT_REG		0x71
#define AS3711_ADC_LSB_RESULT_REG		0x72
#define AS3711_ADC_MASK_CONV_START		(1<<7)
#define AS3711_ADC_BIT_CONV_START		(1<<7)
#define AS3711_ADC_MASK_CONV_NOTREADY		(1<<7)
#define AS3711_ADC_BIT_CONV_NOTREADY		(1<<7)
#define AS3711_ADC_MASK_SOURCE_SELECT		0x0F
#define AS3711_ADC_SOURCE_BATTEMP_NTC		0
#define AS3711_ADC_SOURCE_VUSB			6
#define AS3711_ADC_SOURCE_CHIN			7
#define AS3711_ADC_SOURCE_VBAT			8
#define AS3711_ADC_SOURCE_VSUP			9

#define AS3711_GPIO_INV_MASK			0x80
#define AS3711_GPIO_INV				0x80
#define AS3711_GPIO_IOSF_MASK			0x78
#define AS3711_GPIO_IOSF_NORMAL			0
#define AS3711_GPIO_IOSF_INTERRUPT_OUT		(1<<3)
#define AS3711_GPIO_IOSF_VSUP_LOW_OUT		(2<<3)
#define AS3711_GPIO_IOSF_GPIO_INTERRUPT_IN	(3<<3)
#define AS3711_GPIO_IOSF_VSELECT_IN		(5<<3)
#define AS3711_GPIO_IOSF_STBY_VSELECT_RESTART_IN (6<<3)
#define AS3711_GPIO_IOSF_PWR_GOOD_OUT		(7<<3)
#define AS3711_GPIO_MODE_MASK			0x07
#define AS3711_GPIO_MODE_INPUT			0
#define AS3711_GPIO_MODE_OUTPUT			1
#define AS3711_GPIO_MODE_IO_OPEN_DRAIN		2
#define AS3711_GPIO_MODE_INPUT_W_PULLUP		4
#define AS3711_GPIO_MODE_INPUT_W_PULLDOWN	5
#define AS3711_GPIO_MODE_IO_OPEN_DRAIN_PULLUP	6
#define AS3711_GPIO1_SIGNAL_MASK		(1<<0)
#define AS3711_GPIO2_SIGNAL_MASK		(1<<1)
#define AS3711_GPIO3_SIGNAL_MASK		(1<<2)
#define AS3711_GPIO4_SIGNAL_MASK		(1<<3)

#define AS3711_RTC_REP_WAKEUP_EN_MASK		(1<<0)
#define AS3711_RTC_ALARM_WAKEUP_EN_MASK		(1<<1)
#define AS3711_RTC_ON_MASK			(1<<2)

struct as3711_reg_init {
	u8 reg;
	u8 val;
};

struct as3711_irq {
	irq_handler_t handler;
	void *data;
};

struct as3711_hwmon {
	struct platform_device *pdev;
	struct device *classdev;
};

struct as3711_power_stamp {
	unsigned long eoc;
};

struct as3711_power {
	struct platform_device *pdev;
	struct power_supply battery;
	struct power_supply usb;
	struct as3711_reg_init *pwr_init_data;
	struct as3711_power_stamp stamp;
};

struct as3711_gpio_dev {
	struct platform_device *pdev;
};

struct as3711_rtc {
	struct platform_device *pdev;
	struct rtc_device *rtc;
	int alarm_enabled;      /* used for suspend/resume */
};

struct as3711 {
	struct device *dev;

	int (*read_dev) (void *data, char reg, int count, u8 * dst);
	int (*write_dev) (void *data, char reg, int count, const u8 * src);

	struct mutex io_lock;
	void *io_data;

	struct platform_device *regulators[AS3711_NUM_REGULATORS];
	struct regulator_dev *rdevs[AS3711_NUM_REGULATORS];

	int chip_irq;
	struct mutex irq_lock;
	unsigned int irq_base;
	struct as3711_irq irq[AS3711_IRQ_MAX_HANDLER];
	u8 debug_reg;

	u8 chg_stat[2];
	u8 last_chg_stat[2];

	struct as3711_power power;
	struct as3711_gpio_dev gpio;
	struct as3711_rtc rtc;
};

struct as3711_platform_data {
	struct regulator_init_data *reg_init[AS3711_NUM_REGULATORS];

	/* register initialisation */
	struct as3711_reg_init *core_init_data;
	struct as3711_reg_init *pwr_init_data;
	int gpio_base;
	int irq_base;
	int rtc_start_year;
};

u8 as3711_reg_read(struct as3711 *as3711, u8 reg);
void as3711_reg_write(struct as3711 *as3711, u8 reg, u8 val);
int as3711_block_read(struct as3711 *as3711, u8 reg, int count, u8 * data);
int as3711_block_write(struct as3711 *as3711, u8 reg, int count, u8 * data);
int as3711_set_bits(struct as3711 *as3711, u8 reg, u8 mask, u8 val);
int as3711_register_irq(struct as3711 *as3711, int irq,
			irq_handler_t handler, unsigned long flags,
			const char *name, void *data);
int as3711_free_irq(struct as3711 *as3711, int irq);
int as3711_mask_irq(struct as3711 *as3711, int irq);
int as3711_unmask_irq(struct as3711 *as3711, int irq);
void as3711_reg_init(struct as3711 *as3711, struct as3711_reg_init *reg_data);
#endif
