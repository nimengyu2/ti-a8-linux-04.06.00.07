/*
 * as3711-gpio.c  --  gpiolib support for ams AG AS3711 PMIC
 *
 * Copyright 2012 ams AG.
 *
 * Author: Florian Lobmaier <florian.lobmaier@ams.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/mfd/core.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>

#include <linux/mfd/as3711.h>

struct as3711_gpio {
	struct as3711 *as3711;
	struct gpio_chip gpio_chip;
};

static inline struct as3711_gpio *to_as3711_gpio(struct gpio_chip *chip)
{
	return container_of(chip, struct as3711_gpio, gpio_chip);
}

static int as3711_gpio_direction_in(struct gpio_chip *chip, unsigned offset)
{
	struct as3711_gpio *as3711_gpio = to_as3711_gpio(chip);
	struct as3711 *as3711 = as3711_gpio->as3711;

	return as3711_set_bits(as3711, AS3711_GPIO1_CONTROL_REG + offset,
			       AS3711_GPIO_MODE_MASK,
			       AS3711_GPIO_MODE_INPUT);
}

static int as3711_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct as3711_gpio *as3711_gpio = to_as3711_gpio(chip);
	struct as3711 *as3711 = as3711_gpio->as3711;
	int ret;

	ret = as3711_reg_read(as3711, AS3711_GPIO_SIGNAL_IN_REG);
	if (ret < 0)
		return ret;

	if (ret & (AS3711_GPIO1_SIGNAL_MASK << offset))
		return 1;
	else
		return 0;
}

static int as3711_gpio_direction_out(struct gpio_chip *chip,
				     unsigned offset, int value)
{
	struct as3711_gpio *as3711_gpio = to_as3711_gpio(chip);
	struct as3711 *as3711 = as3711_gpio->as3711;

	return as3711_set_bits(as3711, AS3711_GPIO1_CONTROL_REG + offset,
			       	AS3711_GPIO_MODE_MASK,
				AS3711_GPIO_MODE_OUTPUT);
}

static void as3711_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct as3711_gpio *as3711_gpio = to_as3711_gpio(chip);
	struct as3711 *as3711 = as3711_gpio->as3711;

	as3711_set_bits(as3711, AS3711_GPIO_SIGNAL_OUT_REG, 1 << offset,
			value << offset);
}

static int as3711_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	struct as3711_gpio *as3711_gpio = to_as3711_gpio(chip);
	struct as3711 *as3711 = as3711_gpio->as3711;

	if (!as3711->irq_base)
		return -EINVAL;

	return as3711->irq_base + AS3711_GPIO_IRQ_BASE + offset;
}

#ifdef CONFIG_DEBUG_FS
static void as3711_gpio_dbg_show(struct seq_file *s, struct gpio_chip *chip)
{
	struct as3711_gpio *as3711_gpio = to_as3711_gpio(chip);
	struct as3711 *as3711 = as3711_gpio->as3711;
	int i;

	for (i = 0; i < chip->ngpio; i++) {
		int gpio = i + chip->base;
		int reg;
		const char *label, *pull, *direction;

		/* We report the GPIO even if it's not requested since
		 * we're also reporting things like alternate
		 * functions which apply even when the GPIO is not in
		 * use as a GPIO.
		 */
		label = gpiochip_is_requested(chip, i);
		if (!label)
			label = "Unrequested";

		seq_printf(s, " gpio-%-3d (%-20.20s) ", gpio, label);

		reg = as3711_reg_read(as3711, AS3711_GPIO1_CONTROL_REG + i);
		if (reg < 0) {
			dev_err(as3711->dev,
				"GPIO control %d read failed: %d\n",
				gpio, reg);
			seq_printf(s, "\n");
			continue;
		}

		switch (reg & AS3711_GPIO_MODE_MASK) {
		case AS3711_GPIO_MODE_INPUT:
			direction = "in";
			pull = "nopull";
			break;
		case AS3711_GPIO_MODE_INPUT_W_PULLDOWN:
			direction = "in";
			pull = "pulldown";
			break;
		case AS3711_GPIO_MODE_INPUT_W_PULLUP:
			direction = "in";
			pull = "pullup";
			break;
		case AS3711_GPIO_MODE_OUTPUT:
			direction = "out";
			pull = "push and pull";
			break;
		case AS3711_GPIO_MODE_IO_OPEN_DRAIN:
			direction = "io";
			pull = "nopull";
			break;
		case AS3711_GPIO_MODE_IO_OPEN_DRAIN_PULLUP:
			direction = "io";
			pull = "pullup";
			break;
		default:
			direction = "INVALID DIRECTION/MODE";
			pull = "INVALID PULL";
			break;
		}

		seq_printf(s, " %s %s %s\n"
			   "                                  %s (0x%4x)\n",
			   direction,
			   as3711_gpio_get(chip, i) ? "high" : "low",
			   pull,
			   reg & AS3711_GPIO_INV_MASK ? " inverted" : "",
			   reg);
	}
}
#else
#define as3711_gpio_dbg_show NULL
#endif

static struct gpio_chip template_chip = {
	.label			= "as3711",
	.owner			= THIS_MODULE,
	.direction_input	= as3711_gpio_direction_in,
	.get			= as3711_gpio_get,
	.direction_output	= as3711_gpio_direction_out,
	.set			= as3711_gpio_set,
	.to_irq			= as3711_gpio_to_irq,
	.dbg_show		= as3711_gpio_dbg_show,
	.can_sleep		= 1,
};

static int __devinit as3711_gpio_probe(struct platform_device *pdev)
{
	struct as3711 *as3711 = platform_get_drvdata(pdev);
	struct as3711_platform_data *pdata = as3711->dev->platform_data;
	struct as3711_gpio *as3711_gpio;
	int ret;

	as3711_gpio = kzalloc(sizeof(*as3711_gpio), GFP_KERNEL);
	if (as3711_gpio == NULL)
		return -ENOMEM;

	as3711_gpio->as3711 = as3711;
	as3711_gpio->gpio_chip = template_chip;
	as3711_gpio->gpio_chip.ngpio = AS3711_NUM_GPIO;
	as3711_gpio->gpio_chip.dev = &pdev->dev;
	if (pdata && pdata->gpio_base) {
		as3711_gpio->gpio_chip.base = pdata->gpio_base;
	} else {
		as3711_gpio->gpio_chip.base = -1;
	}
	ret = gpiochip_add(&as3711_gpio->gpio_chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "Could not register gpiochip, %d\n",
			ret);
		goto err;
	}

	platform_set_drvdata(pdev, as3711_gpio);

	return ret;

err:
	kfree(as3711_gpio);
	return ret;
}

static int __devexit as3711_gpio_remove(struct platform_device *pdev)
{
	struct as3711_gpio *as3711_gpio = platform_get_drvdata(pdev);
	int ret;

	ret = gpiochip_remove(&as3711_gpio->gpio_chip);
	if (ret == 0)
		kfree(as3711_gpio);

	return ret;
}

static struct platform_driver as3711_gpio_driver = {
	.driver.name	= "as3711-gpio",
	.driver.owner	= THIS_MODULE,
	.probe		= as3711_gpio_probe,
	.remove		= __devexit_p(as3711_gpio_remove),
};

static int __init as3711_gpio_init(void)
{
	return platform_driver_register(&as3711_gpio_driver);
}
subsys_initcall(as3711_gpio_init);

static void __exit as3711_gpio_exit(void)
{
	platform_driver_unregister(&as3711_gpio_driver);
}
module_exit(as3711_gpio_exit);

MODULE_AUTHOR("Florian Lobmaier <florian.lobmaier@ams.com>");
MODULE_DESCRIPTION("GPIO interface for AS3711 PMIC");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:as3711-gpio");

