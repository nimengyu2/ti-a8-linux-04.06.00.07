/*
 * as3711-power.c: power and charger driver
 *
 * copyright (c) 2012 ams AG
 *
 * author: roman schneider <roman.schneider@ams.com>
 *
 * this program is free software; you can redistribute it and/or modify
 * it under the terms of the gnu general public license as published by
 * the free software foundation; either version 2 of the license, or
 * (at your option) any later version.
 *
 * this program is distributed in the hope that it will be useful,
 * but without any warranty; without even the implied warranty of
 * merchantability or fitness for a particular purpose.  see the
 * gnu general public license for more details.
 *
 * you should have received a copy of the gnu general public license
 * along with this program; if not, write to the free software
 * foundation, inc., 59 temple place, suite 330, boston, ma  02111-1307 usa
 *
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/mfd/as3711.h>

static int as3711_get_voltage(struct as3711 *as3711, u8 adc_source)
{
	u8 msb, lsb;
	int factor;

	switch (adc_source) {

	case AS3711_ADC_SOURCE_VUSB:
		factor = 26400;
		break;
	case AS3711_ADC_SOURCE_CHIN:
		factor = 7030;
		break;
	case AS3711_ADC_SOURCE_VBAT:
		factor = 7030;
		break;
	default:
		printk(KERN_ERR "%s: adc source not valid\n", __func__);
		return -EINVAL;
	}

	/* set source and start conversion */
	as3711_set_bits(as3711, AS3711_ADC_CONTROL_REG,
			AS3711_ADC_MASK_CONV_START |
			AS3711_ADC_MASK_SOURCE_SELECT,
			AS3711_ADC_BIT_CONV_START | adc_source);

	/* takes approx. 60us with 275kHz */
	udelay(80);

	msb = as3711_reg_read(as3711, AS3711_ADC_MSB_RESULT_REG);

	/* check if conversion is ready */
	if (msb & AS3711_ADC_MASK_CONV_NOTREADY) {
		printk(KERN_ERR "%s: adc result not ready\n", __func__);
		return -EINVAL;
	}

	lsb = as3711_reg_read(as3711, AS3711_ADC_LSB_RESULT_REG);

	return ((((int)msb << 3) + lsb) * factor);
}

static ssize_t charger_state_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	u8 status;
	char *charge = "unknown";
	struct as3711 *as3711 = dev_get_drvdata(dev);

	status = as3711_reg_read(as3711, AS3711_CHARGERSTATUS1_REG);

	if (status & AS3711_CHST_EOC) {
		charge = "end of charge";
	} else if (status & AS3711_CHST_TRICKLE) {
		charge = "trickle";
	} else if (status & AS3711_CHST_CCM) {
		charge = "constant current";
	} else if (status & AS3711_CHST_CVM) {
		charge = "constant voltage";
	} else if (status & AS3711_CHST_RESUME) {
		charge = "resume";
	}

	return sprintf(buf, "%s\n", charge);
}

static DEVICE_ATTR(charger_state, 0444, charger_state_show, NULL);

static irqreturn_t as3711_onkey_handler(int irq, void *data)
{
	// struct as3711 *as3711 = data;
	printk(KERN_INFO "onkey_handler ....\n");
	return IRQ_HANDLED;
}

static int as3711_fast_call(unsigned long *last)
{
	int fast_call = 0;
	unsigned long now = jiffies;

	if (*last) {
		if (time_before(now, (*last) + HZ))
			fast_call = 1;
	}

	*last = jiffies;

	return fast_call;
}

static irqreturn_t as3711_charger_handler(int irq, void *data)
{
	int update_battery = 0;
	u8 curr_stat[2], chg[2];
	struct as3711 *as3711 = data;
	struct as3711_power *power = &as3711->power;
	static unsigned long time_stamp;

	as3711_block_read(as3711, AS3711_CHARGERSTATUS1_REG, 2, &curr_stat[0]);

	chg[0] = as3711->last_chg_stat[0] ^ curr_stat[0];
	chg[1] = as3711->last_chg_stat[1] ^ curr_stat[1];
	
	printk(KERN_DEBUG "chghandler: stat=%02X - chg1=%02X chg2=%02X\n",
	       curr_stat[0], chg[0], chg[1]);

	if (chg[0] & AS3711_CHST_CHDET) {
		printk(KERN_DEBUG "power_supply_changed(usb)\n");
		power_supply_changed(&power->usb);
	}	

	/*
	 * The "No bat detection" cycle generates lots of events. Thus we
	 * report the events below only one time a second.
	 */
	if ((chg[0] & AS3711_CHST_EOC) ||
	    (chg[0] & AS3711_CHST_TRICKLE) ||
	    (chg[0] & AS3711_CHST_CCM)) {
		if (!as3711_fast_call(&time_stamp))
			update_battery = 1;
	}

	if ((chg[0] & AS3711_CHST_NOBAT) ||
	    (chg[0] & AS3711_CHST_BATTEMP_HI) ||
	    (chg[0] & AS3711_CHST_CHDET))
		update_battery = 1;

	if (update_battery) {
		printk(KERN_DEBUG "power_supply_changed(battery)\n");
		power_supply_changed(&power->battery);
	}

	as3711->last_chg_stat[0] = curr_stat[0];
	as3711->last_chg_stat[1] = curr_stat[1];
	return IRQ_HANDLED;
}

static int as3711_usb_get_prop(struct power_supply *psy,
			       enum power_supply_property psp,
			       union power_supply_propval *val)
{
	struct as3711 *as3711 = dev_get_drvdata(psy->dev->parent);

	switch (psp) {

	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = ! !(as3711_reg_read(as3711,
						  AS3711_CHARGERSTATUS1_REG) &
				  AS3711_CHST_CHDET);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = as3711_get_voltage(as3711,
						 AS3711_ADC_SOURCE_CHIN);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int as3711_bat_get_prop_status(u8 status)
{
	if (!(status & AS3711_CHST_CHDET))
		return POWER_SUPPLY_STATUS_DISCHARGING;

	if (status & AS3711_CHST_NOBAT)
		return POWER_SUPPLY_STATUS_UNKNOWN;

	if (status & AS3711_CHST_EOC)
		return POWER_SUPPLY_STATUS_FULL;

	return POWER_SUPPLY_STATUS_CHARGING;
}

static int as3711_bat_get_prop_health(u8 status)
{
	if (!(status & AS3711_CHST_CHDET))
		return POWER_SUPPLY_HEALTH_UNKNOWN;

	if (status & AS3711_CHST_NOBAT)
		return POWER_SUPPLY_HEALTH_UNKNOWN;

	if (status & AS3711_CHST_BATTEMP_HI)
		return POWER_SUPPLY_HEALTH_OVERHEAT;

	return POWER_SUPPLY_HEALTH_GOOD;
}

static int as3711_bat_get_prop_charge_type(u8 status)
{
	if (!(status & AS3711_CHST_CHDET))
		return POWER_SUPPLY_CHARGE_TYPE_NONE;

	if (status & AS3711_CHST_NOBAT)
		return POWER_SUPPLY_CHARGE_TYPE_NONE;

	if (status & AS3711_CHST_TRICKLE)
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;

	if (status & AS3711_CHST_CCM)
		return POWER_SUPPLY_CHARGE_TYPE_FAST;

	return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
}

#if 0

/* Note by Roman Schneider: 
   Calculating the capacity via the charger states is not a good idea. You need a
   fuel gauge for doing that. I implemented this for demonstration only. */

static int as3711_bat_get_capacity_level(u8 status)
{
	if (status & AS3711_CHST_EOC)
		return POWER_SUPPLY_CAPACITY_LEVEL_FULL;

	if (status & AS3711_CHST_RESUME)
		return POWER_SUPPLY_CAPACITY_LEVEL_HIGH;

	if (status & AS3711_CHST_CVM)
		return POWER_SUPPLY_CAPACITY_LEVEL_HIGH;

	if (status & AS3711_CHST_CCM)
		return POWER_SUPPLY_CAPACITY_LEVEL_LOW;

	if (status & AS3711_CHST_TRICKLE)
		return POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;

	return POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
}
#endif

static int as3711_bat_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	u8 status;
	int ret = 0;
	struct as3711 *as3711 = dev_get_drvdata(psy->dev->parent);

	status = as3711_reg_read(as3711, AS3711_CHARGERSTATUS1_REG);

	switch (psp) {

	case POWER_SUPPLY_PROP_STATUS:
		val->intval = as3711_bat_get_prop_status(status);
		break;

	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = !(status & AS3711_CHST_NOBAT);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (!(status & AS3711_CHST_NOBAT)) {
			val->intval = as3711_get_voltage(as3711,
							 AS3711_ADC_SOURCE_VBAT);
		} else {
			val->intval = 0;
		}
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = as3711_bat_get_prop_health(status);
		break;

	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = as3711_bat_get_prop_charge_type(status);
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		if (!(status & AS3711_CHST_NOBAT)) {
			val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		} else {
			val->intval = POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
		}
		break;

	default:
		printk(KERN_ERR "%s: unhandled property %d\n", __func__, psp);
		ret = -EINVAL;
	}

	return ret;
}

static enum power_supply_property as3711_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static enum power_supply_property as3711_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_TECHNOLOGY,
};

static __devinit int as3711_power_probe(struct platform_device *pdev)
{
	struct as3711 *as3711 = platform_get_drvdata(pdev);
	struct as3711_power *power = &as3711->power;
	struct power_supply *usb = &power->usb;
	struct power_supply *battery = &power->battery;
	int ret;

	battery->name = "Battery";
	battery->properties = as3711_bat_props;
	battery->num_properties = ARRAY_SIZE(as3711_bat_props);
	battery->get_property = as3711_bat_get_property;
	battery->use_for_apm = 1;
	ret = power_supply_register(&pdev->dev, battery);
	if (ret)
		goto battery_failed;

	usb->name = "USB", usb->type = POWER_SUPPLY_TYPE_USB;
	usb->properties = as3711_usb_props;
	usb->num_properties = ARRAY_SIZE(as3711_usb_props);
	usb->get_property = as3711_usb_get_prop;
	ret = power_supply_register(&pdev->dev, usb);
	if (ret)
		goto usb_failed;

	ret = device_create_file(&pdev->dev, &dev_attr_charger_state);
	if (ret < 0)
		dev_printk(KERN_WARNING, as3711->dev,
			   "failed to add charge sysfs: %d\n", ret);
	ret = 0;

	/* register on key handler */
	ret = as3711_register_irq(as3711, AS3711_IRQ_HANDLER_ONKEY,
				  as3711_onkey_handler, 0, "onkey", as3711);
	if (ret < 0) {
		dev_printk(KERN_WARNING, as3711->dev,
			   "failed to install onkey irq handler: %d\n", ret);
	}

	/* register charger handler */
	ret = as3711_register_irq(as3711, AS3711_IRQ_HANDLER_CHARGER,
				  as3711_charger_handler, 0, "charger", as3711);
	if (ret < 0) {
		dev_printk(KERN_WARNING, as3711->dev,
			   "failed to install charger irq handler: %d\n", ret);
	}

	/* init charger with platform data */
	if (power->pwr_init_data)
		as3711_reg_init(as3711, power->pwr_init_data);

	return 0;

usb_failed:
	power_supply_unregister(battery);
battery_failed:
	return ret;
}

static __devexit int as3711_power_remove(struct platform_device *pdev)
{
	struct as3711 *as3711 = platform_get_drvdata(pdev);
	struct as3711_power *power = &as3711->power;

	as3711_free_irq(as3711, AS3711_IRQ_HANDLER_CHARGER);
	as3711_free_irq(as3711, AS3711_IRQ_HANDLER_ONKEY);
	power_supply_unregister(&power->battery);
	power_supply_unregister(&power->usb);
	device_remove_file(&pdev->dev, &dev_attr_charger_state);
	return 0;
}

static struct platform_driver as3711_power_driver = {
	.probe = as3711_power_probe,
	.remove = __devexit_p(as3711_power_remove),
	.driver = {
		   .name = "as3711-power",
		   },
};

static int __init as3711_power_init(void)
{
	return platform_driver_register(&as3711_power_driver);
}

module_init(as3711_power_init);

static void __exit as3711_power_exit(void)
{
	platform_driver_unregister(&as3711_power_driver);
}

module_exit(as3711_power_exit);

MODULE_DESCRIPTION("Power supply driver for the AS3711 pmic");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Roman Schneider");
