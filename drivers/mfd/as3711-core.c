/*
 * as3711-core.c - core driver for AS3711 PMIC
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/bcd.h>
#include <linux/delay.h>
#include <linux/mfd/core.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/mfd/as3711.h>

#define AS3711_DRIVER_VERSION	"v1.0.4"

static int as3711_read(struct as3711 *as3711, u8 reg, int num_regs, u8 * dest)
{
	int ret = 0;

	/* we need to implement some caching here ...  */
	ret = as3711->read_dev(as3711->io_data, reg, num_regs, dest);

	WARN_ON(ret != 0);

	return ret;
}

static int as3711_write(struct as3711 *as3711, u8 reg, int num_regs, u8 * src)
{
	int ret;

	/* we need to implement some caching here ...  */
	ret = as3711->write_dev(as3711->io_data, reg, num_regs, src);

	WARN_ON(ret != 0);

	if (ret != 0)
		return -EIO;

	return 0;
}

u8 as3711_reg_read(struct as3711 * as3711, u8 reg)
{
	u8 val;

	mutex_lock(&as3711->io_lock);

	as3711_read(as3711, reg, 1, &val);

	mutex_unlock(&as3711->io_lock);

	return val;
}
EXPORT_SYMBOL_GPL(as3711_reg_read);

void as3711_reg_write(struct as3711 * as3711, u8 reg, u8 val)
{
	mutex_lock(&as3711->io_lock);

	as3711_write(as3711, reg, 1, &val);

	mutex_unlock(&as3711->io_lock);
}
EXPORT_SYMBOL_GPL(as3711_reg_write);

int as3711_block_read(struct as3711 *as3711, u8 reg, int count, u8 * data)
{
	int ret;

	mutex_lock(&as3711->io_lock);

	ret = as3711_read(as3711, reg, count, data);

	mutex_unlock(&as3711->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(as3711_block_read);

int as3711_block_write(struct as3711 *as3711, u8 reg, int count, u8 * data)
{
	int ret;

	mutex_lock(&as3711->io_lock);

	ret = as3711_write(as3711, reg, count, data);

	mutex_unlock(&as3711->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(as3711_block_write);

int as3711_set_bits(struct as3711 *as3711, u8 reg, u8 mask, u8 val)
{
	u8 tmp;
	int ret;

	mutex_lock(&as3711->io_lock);

	ret = as3711_read(as3711, reg, 1, &tmp);
	tmp = (tmp & ~mask) | val;
	if (ret == 0)
		ret = as3711_write(as3711, reg, 1, &tmp);

	mutex_unlock(&as3711->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(as3711_set_bits);

static int as3711_i2c_read(void *io_data, char reg, int count, u8 * dest)
{
	struct i2c_client *i2c = io_data;
	struct i2c_msg xfer[2];
	int ret;

	// printk(KERN_INFO " XXXX as3711_i2c_read: reg=%d, count=%d\n", reg, count);

	/* Write register */
	xfer[0].addr = i2c->addr;
	xfer[0].flags = 0;
	xfer[0].len = 1;
	xfer[0].buf = &reg;

	/* Read data */
	xfer[1].addr = i2c->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = count;
	xfer[1].buf = (u8 *) dest;

	ret = i2c_transfer(i2c->adapter, xfer, 2);
	if (ret == 2)
		ret = 0;
	else if (ret >= 0)
		ret = -EIO;

	return ret;
}

static int as3711_i2c_write(void *io_data, char reg, int count, const u8 * src)
{
	struct i2c_client *i2c = io_data;
	u8 *msg;
	int ret;

	/* We add 1 byte for device register - ideally I2C would gather. */
	msg = kmalloc(count + 1, GFP_KERNEL);
	if (msg == NULL)
		return -ENOMEM;

	msg[0] = reg;
	memcpy(&msg[1], src, count);

	ret = i2c_master_send(i2c, msg, count + 1);

	if (ret == count + 1)
		ret = 0;
	else if (ret >= 0)
		ret = -EIO;

	kfree(msg);

	return ret;
}

int as3711_mask_irq(struct as3711 *as3711, int irq)
{
	u8 reg, mask, bit;

	switch (irq) {
	case AS3711_IRQ_TRICKLE:
		reg = AS3711_INTERRUPTMASK1_REG;
		mask = AS3711_IRQ_MASK_TRICKLE;
		bit = AS3711_IRQ_BIT_TRICKLE;
		break;
	case AS3711_IRQ_NOBAT:
		reg = AS3711_INTERRUPTMASK1_REG;
		mask = AS3711_IRQ_MASK_NOBAT;
		bit = AS3711_IRQ_BIT_NOBAT;
		break;
	case AS3711_IRQ_RESUME:
		reg = AS3711_INTERRUPTMASK1_REG;
		mask = AS3711_IRQ_MASK_RESUME;
		bit = AS3711_IRQ_BIT_RESUME;
		break;
	case AS3711_IRQ_EOC:
		reg = AS3711_INTERRUPTMASK1_REG;
		mask = AS3711_IRQ_MASK_EOC;
		bit = AS3711_IRQ_BIT_EOC;
		break;
	case AS3711_IRQ_CHDET:
		reg = AS3711_INTERRUPTMASK1_REG;
		mask = AS3711_IRQ_MASK_CHDET;
		bit = AS3711_IRQ_BIT_CHDET;
		break;
	case AS3711_IRQ_ONKEY:
		reg = AS3711_INTERRUPTMASK1_REG;
		mask = AS3711_IRQ_MASK_ONKEY;
		bit = AS3711_IRQ_BIT_ONKEY;
		break;
	case AS3711_IRQ_OVTMP:
		reg = AS3711_INTERRUPTMASK1_REG;
		mask = AS3711_IRQ_MASK_OVTMP;
		bit = AS3711_IRQ_BIT_OVTMP;
		break;
	case AS3711_IRQ_LOWBAT:
		reg = AS3711_INTERRUPTMASK1_REG;
		mask = AS3711_IRQ_MASK_LOWBAT;
		bit = AS3711_IRQ_BIT_LOWBAT;
		break;
	case AS3711_IRQ_RTC_REP:
		reg = AS3711_INTERRUPTMASK2_REG;
		mask = AS3711_IRQ_MASK_RTC_REP;
		bit = AS3711_IRQ_BIT_RTC_REP;
		break;
	case AS3711_IRQ_RTC_ALARM:
		reg = AS3711_INTERRUPTMASK3_REG;
		mask = AS3711_IRQ_MASK_RTC_ALARM;
		bit = AS3711_IRQ_BIT_RTC_ALARM;
		break;
	default:
		dev_printk(KERN_ERR, as3711->dev, "unknown irq 0x%04X\n", irq);
		return -EINVAL;
	}

	return as3711_set_bits(as3711, reg, mask, bit);
}
EXPORT_SYMBOL_GPL(as3711_mask_irq);

int as3711_unmask_irq(struct as3711 *as3711, int irq)
{
	u8 reg, mask;

	switch (irq) {
	case AS3711_IRQ_TRICKLE:
		reg = AS3711_INTERRUPTMASK1_REG;
		mask = AS3711_IRQ_MASK_TRICKLE;
		break;
	case AS3711_IRQ_NOBAT:
		reg = AS3711_INTERRUPTMASK1_REG;
		mask = AS3711_IRQ_MASK_NOBAT;
		break;
	case AS3711_IRQ_RESUME:
		reg = AS3711_INTERRUPTMASK1_REG;
		mask = AS3711_IRQ_MASK_RESUME;
		break;
	case AS3711_IRQ_EOC:
		reg = AS3711_INTERRUPTMASK1_REG;
		mask = AS3711_IRQ_MASK_EOC;
		break;
	case AS3711_IRQ_CHDET:
		reg = AS3711_INTERRUPTMASK1_REG;
		mask = AS3711_IRQ_MASK_CHDET;
		break;
	case AS3711_IRQ_ONKEY:
		reg = AS3711_INTERRUPTMASK1_REG;
		mask = AS3711_IRQ_MASK_ONKEY;
		break;
	case AS3711_IRQ_OVTMP:
		reg = AS3711_INTERRUPTMASK1_REG;
		mask = AS3711_IRQ_MASK_OVTMP;
		break;
	case AS3711_IRQ_LOWBAT:
		reg = AS3711_INTERRUPTMASK1_REG;
		mask = AS3711_IRQ_MASK_LOWBAT;
		break;
	case AS3711_IRQ_RTC_REP:
		reg = AS3711_INTERRUPTMASK2_REG;
		mask = AS3711_IRQ_MASK_RTC_REP;
		break;
	case AS3711_IRQ_RTC_ALARM:
		reg = AS3711_INTERRUPTMASK3_REG;
		mask = AS3711_IRQ_MASK_RTC_ALARM;
		break;
	default:
		dev_printk(KERN_ERR, as3711->dev, "unknown irq 0x%04X\n", irq);
		return -EINVAL;
	}

	return as3711_set_bits(as3711, reg, mask, 0);
}
EXPORT_SYMBOL_GPL(as3711_unmask_irq);

int as3711_register_irq(struct as3711 *as3711, int irq,
			irq_handler_t handler, unsigned long flags,
			const char *name, void *data)
{
	if (irq < 0 || irq > AS3711_IRQ_HANDLER_MAX || !handler) {
		dev_printk(KERN_ERR, as3711->dev,
			   "cannot register invalid irq %d.\n", irq);
		return -EINVAL;
	}

	if (as3711->irq[irq].handler) {
		dev_printk(KERN_ERR, as3711->dev,
			   "irq %d registered already.\n", irq);
		return -EBUSY;
	}

	mutex_lock(&as3711->irq_lock);
	as3711->irq[irq].handler = handler;
	as3711->irq[irq].data = data;
	mutex_unlock(&as3711->irq_lock);

	switch (irq) {
	case AS3711_IRQ_HANDLER_CHARGER:
		as3711_unmask_irq(as3711, AS3711_IRQ_TRICKLE);
		as3711_unmask_irq(as3711, AS3711_IRQ_NOBAT);
		as3711_unmask_irq(as3711, AS3711_IRQ_RESUME);
		as3711_unmask_irq(as3711, AS3711_IRQ_EOC);
		as3711_unmask_irq(as3711, AS3711_IRQ_CHDET);
		as3711_unmask_irq(as3711, AS3711_IRQ_ONKEY);
		as3711_unmask_irq(as3711, AS3711_IRQ_OVTMP);
		as3711_unmask_irq(as3711, AS3711_IRQ_LOWBAT);
		break;

	case AS3711_IRQ_HANDLER_ONKEY:
		as3711_unmask_irq(as3711, AS3711_IRQ_ONKEY);
		break;
	case AS3711_IRQ_HANDLER_RTC_REP:
		as3711_unmask_irq(as3711, AS3711_IRQ_RTC_REP);
		break;
	case AS3711_IRQ_HANDLER_RTC_ALARM:
		as3711_unmask_irq(as3711, AS3711_IRQ_RTC_ALARM);
		break;
	default:
		dev_printk(KERN_ERR, as3711->dev, "unmask: unknow irq 0x%04X\n",
			   irq);
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(as3711_register_irq);

int as3711_free_irq(struct as3711 *as3711, int irq)
{
	if (irq < 0 || irq > AS3711_IRQ_HANDLER_MAX)
		return -EINVAL;

	switch (irq) {
	case AS3711_IRQ_HANDLER_CHARGER:
		as3711_mask_irq(as3711, AS3711_IRQ_TRICKLE);
		as3711_mask_irq(as3711, AS3711_IRQ_NOBAT);
		as3711_mask_irq(as3711, AS3711_IRQ_RESUME);
		as3711_mask_irq(as3711, AS3711_IRQ_EOC);
		as3711_mask_irq(as3711, AS3711_IRQ_CHDET);
		as3711_mask_irq(as3711, AS3711_IRQ_ONKEY);
		as3711_mask_irq(as3711, AS3711_IRQ_OVTMP);
		as3711_mask_irq(as3711, AS3711_IRQ_LOWBAT);
		break;

	case AS3711_IRQ_HANDLER_ONKEY:
		as3711_mask_irq(as3711, AS3711_IRQ_ONKEY);
		break;
	case AS3711_IRQ_HANDLER_RTC_REP:
		as3711_mask_irq(as3711, AS3711_IRQ_RTC_REP);
		break;
	case AS3711_IRQ_HANDLER_RTC_ALARM:
		as3711_mask_irq(as3711, AS3711_IRQ_RTC_ALARM);
		break;

	default:
		dev_printk(KERN_ERR, as3711->dev, "mask: unknow irq 0x%04X\n",
			   irq);
		return -EINVAL;
	}


	mutex_lock(&as3711->irq_lock);
	as3711->irq[irq].handler = NULL;
	mutex_unlock(&as3711->irq_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(as3711_free_irq);

static ssize_t debug_read_reg(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	u8 cont;
	struct as3711 *as3711 = dev_get_drvdata(dev);

	cont = as3711_reg_read(as3711, as3711->debug_reg);

	return sprintf(buf, "0x%02X\n", cont);
}

static ssize_t debug_write_reg(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct as3711 *as3711 = dev_get_drvdata(dev);
	long val = simple_strtol(buf, NULL, 16);

	as3711_set_bits(as3711, as3711->debug_reg, 0xFF, val);

	return count;
}

static ssize_t debug_show_reg_pointer(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct as3711 *as3711 = dev_get_drvdata(dev);

	return sprintf(buf, "0x%02X\n", as3711->debug_reg);
}

static ssize_t debug_set_reg_pointer(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct as3711 *as3711 = dev_get_drvdata(dev);
	long val = simple_strtol(buf, NULL, 16);

	as3711->debug_reg = (u8) val;

	return count;
}

static DEVICE_ATTR(reg_pointer, 0600, debug_show_reg_pointer,
		   debug_set_reg_pointer);
static DEVICE_ATTR(reg_content, 0600, debug_read_reg, debug_write_reg);

static void as3711_irq_call_handler(struct as3711 *as3711, int irq)
{
	mutex_lock(&as3711->irq_lock);

	if (as3711->irq[irq].handler) {
		as3711->irq[irq].handler(irq, as3711->irq[irq].data);
	} else {
		dev_printk(KERN_ERR, as3711->dev,
			   "irq %d nobody cared. now masked.\n", irq);
		as3711_mask_irq(as3711, irq);
	}

	mutex_unlock(&as3711->irq_lock);
}

static irqreturn_t as3711_irq_threaded(int irq, void *data)
{
	struct as3711 *as3711 = data;
	u8 irq_stat[3];

	as3711_block_read(as3711, 0x77, 3, &irq_stat[0]);

	dev_printk(KERN_DEBUG, as3711->dev, "irq: 0x%02X 0x%02X 0x%02X\n",
	       irq_stat[0], irq_stat[1], irq_stat[2]);

	if (irq_stat[0] & AS3711_IRQ_MASK_ONKEY) {
		as3711_irq_call_handler(as3711, AS3711_IRQ_HANDLER_ONKEY);
	}

	if ((irq_stat[0] & AS3711_IRQ_MASK_CHDET) ||
	    (irq_stat[0] & AS3711_IRQ_MASK_TRICKLE) ||
	    (irq_stat[0] & AS3711_IRQ_MASK_NOBAT) ||
	    (irq_stat[0] & AS3711_IRQ_MASK_RESUME) ||
	    (irq_stat[0] & AS3711_IRQ_MASK_EOC)) {
		as3711_irq_call_handler(as3711, AS3711_IRQ_HANDLER_CHARGER);
	}

	if (irq_stat[1] & AS3711_IRQ_MASK_RTC_REP) {
		as3711_irq_call_handler(as3711, AS3711_IRQ_HANDLER_RTC_REP);
	}
	if (irq_stat[2] & AS3711_IRQ_MASK_RTC_ALARM) {
		as3711_irq_call_handler(as3711, AS3711_IRQ_HANDLER_RTC_ALARM);
	}

	return IRQ_HANDLED;
}

static int as3711_irq_init(struct as3711 *as3711, int irq)
{
	int ret;

	mutex_init(&as3711->irq_lock);

	as3711->chip_irq = irq;

	ret = request_threaded_irq(irq, NULL, as3711_irq_threaded,
				   IRQF_ONESHOT | IRQF_TRIGGER_LOW,
				   "as3711", as3711);

	if (ret != 0) {
		dev_printk(KERN_ERR, as3711->dev,
			   "failed to request irq %d: %d\n", irq, ret);
		return ret;
	}

	return 0;
}

static int as3711_register_regulator(struct as3711 *as3711, int reg,
				     struct regulator_init_data *initdata)
{
	int ret;
	struct platform_device *pdev;
	struct regulator_init_data *platform_data;

	if (reg < 0 || reg >= AS3711_NUM_REGULATORS) {
		dev_printk(KERN_ERR, as3711->dev, "invalid reg-id %02d.\n", reg);
		return -EINVAL;
	}

	if (as3711->regulators[reg]) {
		dev_printk(KERN_ERR, as3711->dev, "regulator %02d is busy.\n", reg);
		return -EBUSY;
	}

	pdev = platform_device_alloc("as3711-regulator", reg);
	if (!pdev) {
		dev_printk(KERN_ERR, as3711->dev,
			   "can't alloc regulator device %02d.\n", reg);
		return -ENOMEM;
	}

	ret = platform_device_add_data(pdev, initdata,
				       sizeof(struct regulator_init_data));
	if (ret) {
		dev_printk(KERN_ERR, as3711->dev,
			   "failed to add platform data: %d\n", ret);
		goto device_add_err;
	}

	platform_data = (struct regulator_init_data *)pdev->dev.platform_data;
	platform_data->driver_data = as3711;

	pdev->dev.parent = as3711->dev;
	platform_set_drvdata(pdev, as3711);

	ret = platform_device_add(pdev);
	if (ret) {
		dev_printk(KERN_ERR, as3711->dev,
			"failed to add regulator device %02d: %d\n", reg, ret);
		as3711->regulators[reg] = NULL;
	}

	as3711->regulators[reg] = pdev;

	dev_printk(KERN_DEBUG, as3711->dev, "added regulator device: %02d\n", reg);

	return 0;

device_add_err:
	platform_device_put(pdev);
	return ret;
}

void as3711_reg_init(struct as3711 *as3711, struct as3711_reg_init *reg_data)
{
	int ret;

	while (reg_data->reg != AS3711_REG_INIT_TERMINATE) {
		ret = as3711_write(as3711, reg_data->reg, 1, &reg_data->val);
		if (ret) {
			dev_printk(KERN_ERR, as3711->dev,
				   "reg setup failed: %d\n", ret);
			return;
		}
		reg_data++;
	}

	dev_printk(KERN_INFO, as3711->dev, "register setup finished.\n");
}
EXPORT_SYMBOL_GPL(as3711_reg_init);

static void as3711_add_regulator_devices(struct as3711 *as3711,
					struct as3711_platform_data *pdata)
{
	int regulator;
	for (regulator = 0; regulator < AS3711_NUM_REGULATORS; regulator++) {
		if (pdata->reg_init[regulator])
			as3711_register_regulator(as3711, regulator,
						  pdata->reg_init[regulator]);
	}
}

static void as3711_client_dev_register(struct as3711 *as3711,
				       const char *name,
				       struct platform_device **pdev)
{
	int ret;

	*pdev = platform_device_alloc(name, -1);
	if (*pdev == NULL) {
		dev_printk(KERN_ERR, as3711->dev, "failed to allocate %s\n",
			   name);
		return;
	}

	(*pdev)->dev.parent = as3711->dev;
	platform_set_drvdata(*pdev, as3711);
	ret = platform_device_add(*pdev);
	if (ret != 0) {
		dev_printk(KERN_ERR, as3711->dev, "failed to register %s: %d\n",
			   name, ret);
		platform_device_put(*pdev);
		*pdev = NULL;
	}
}

static int as3711_init(struct as3711 *as3711,
		       struct as3711_platform_data *pdata, int irq)
{
	u8 reg;
	int ret;

	mutex_init(&as3711->io_lock);

	dev_set_drvdata(as3711->dev, as3711);

	/* Check that this is actually a AS3711 */
	ret = as3711->read_dev(as3711->io_data, AS3711_ADDR_ASIC_ID1, 1, &reg);
	if (ret != 0) {
		dev_printk(KERN_ERR, as3711->dev,
			   "Chip ID register read failed\n");
		return -EIO;
	}
	if ((reg != 0x0A) && (reg != 0x8B)) {
		dev_printk(KERN_ERR, as3711->dev,
			   "Device is not a AS3711-11, ID is %x\n", reg);
		return -ENODEV;
	}

	ret = as3711_read(as3711, AS3711_ADDR_ASIC_ID2, 1, &reg);
	if (ret != 0) {
		dev_printk(KERN_ERR, as3711->dev,
			   "ID register read failed: %d\n", ret);
		return ret;
	}
	dev_printk(KERN_INFO, as3711->dev, "found chip with revision %x\n",
                   reg);

	/* do some initial platform register setup */
	//if (pdata->core_init_data) {
	//	as3711_reg_init(as3711, pdata->core_init_data);
	//}

	/* add the regulator deviced */
	as3711_add_regulator_devices(as3711, pdata);

	/* add power device */
	if (pdata->pwr_init_data) {
		as3711->power.pwr_init_data = pdata->pwr_init_data;
	}

	as3711_client_dev_register(as3711, "as3711-power",
				   &(as3711->power.pdev));
	/* add gpio device */
	as3711_client_dev_register(as3711, "as3711-gpio",
				&(as3711->gpio.pdev));
	/* add rtc device */
	as3711_client_dev_register(as3711, "as3711-rtc",
				&(as3711->rtc.pdev));

	ret = as3711_irq_init(as3711, irq);
	if (ret != 0) {
		dev_printk(KERN_WARNING, as3711->dev,
			   "cannot init irq %d\n", irq);
	}

	return 0;
}

static int as3711_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct as3711 *as3711;
	int ret;

	as3711 = kzalloc(sizeof(struct as3711), GFP_KERNEL);
	if (as3711 == NULL) {
		ret = -ENOMEM;
		goto err;
	}

	as3711->io_data = i2c;
	as3711->read_dev = as3711_i2c_read;
	as3711->write_dev = as3711_i2c_write;
	as3711->dev = &i2c->dev;
	i2c_set_clientdata(i2c, as3711);

	ret = as3711_init(as3711, i2c->dev.platform_data, i2c->irq);
	if (ret != 0)
		goto struct_err;

	ret += device_create_file(as3711->dev, &dev_attr_reg_pointer);
	ret += device_create_file(as3711->dev, &dev_attr_reg_content);

	if (ret)
		dev_printk(KERN_WARNING, as3711->dev,
			   "failed to add reg_content sysfs: %d\n", ret);
	dev_info(as3711->dev, 
		"AS3711 core driver %s initialized successfully\n", 
		AS3711_DRIVER_VERSION);

	return 0;

struct_err:
	i2c_set_clientdata(i2c, NULL);
	kfree(as3711);
err:
	return ret;
}

static int as3711_i2c_remove(struct i2c_client *i2c)
{
	int i;
	struct as3711 *as3711 = i2c_get_clientdata(i2c);

	if (as3711->chip_irq)
		free_irq(as3711->chip_irq, as3711);

	for (i = 0; i < ARRAY_SIZE(as3711->regulators); i++) {
		if (as3711->regulators[i]) {
			platform_device_unregister(as3711->regulators[i]);
		}
	}
	device_remove_file(as3711->dev, &dev_attr_reg_pointer);
	device_remove_file(as3711->dev, &dev_attr_reg_content);
	platform_device_unregister(as3711->power.pdev);

	i2c_set_clientdata(i2c, NULL);
	kfree(as3711);

	return 0;
}

static const struct i2c_device_id as3711_i2c_id[] = {
	{"as3711", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, as3711_i2c_id);

static struct i2c_driver as3711_i2c_driver = {
	.driver = {
		   .name = "as3711",
		   .owner = THIS_MODULE,
		   },
	.probe = as3711_i2c_probe,
	.remove = as3711_i2c_remove,
	.id_table = as3711_i2c_id,
};

static int __init as3711_i2c_init(void)
{
	int ret;

	ret = i2c_add_driver(&as3711_i2c_driver);
	if (ret != 0)
		pr_err("Failed to register AS3711 I2C driver: %d\n", ret);

	return ret;
}

subsys_initcall(as3711_i2c_init);

static void __exit as3711_i2c_exit(void)
{
	i2c_del_driver(&as3711_i2c_driver);
}

module_exit(as3711_i2c_exit);

MODULE_DESCRIPTION("I2C support for the AS3711 PMIC");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Roman Schneider");
