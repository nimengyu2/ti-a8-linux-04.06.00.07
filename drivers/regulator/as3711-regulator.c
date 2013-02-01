/*
 * as3711-regulator.c - voltage regulator support for AS3711
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

#include <linux/bug.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regulator/driver.h>
#include <linux/mfd/as3711.h>

struct as3711_register_mapping {
	u8 reg_id;
	u8 reg;
};

static struct as3711_register_mapping as3711_reg_lookup[] = {
	{AS3711_LDO1_ANA, AS3711_LDO1_CONTROL_REG},
	{AS3711_LDO2_ANA, AS3711_LDO2_CONTROL_REG},
	{AS3711_LDO3_DIG, AS3711_LDO3_CONTROL_REG},
	{AS3711_LDO4_DIG, AS3711_LDO4_CONTROL_REG},
	{AS3711_LDO5_DIG, AS3711_LDO5_CONTROL_REG},
	{AS3711_LDO6_DIG, AS3711_LDO6_CONTROL_REG},
	{AS3711_LDO7_DIG, AS3711_LDO7_CONTROL_REG},
	{AS3711_LDO8_DIG, AS3711_LDO8_CONTROL_REG},
	{AS3711_SD1, AS3711_SD1_CONTROL_REG},
	{AS3711_SD2, AS3711_SD2_CONTROL_REG},
	{AS3711_SD3, AS3711_SD3_CONTROL_REG},
	{AS3711_SD4, AS3711_SD4_CONTROL_REG},
	{AS3711_SU1, AS3711_SU1_CONTROL_REG},
	{AS3711_SU2, AS3711_SU2_CONTROL_REG},
	{AS3711_CURR1, AS3711_CURR1_CONTROL_REG},
	{AS3711_CURR2, AS3711_CURR2_CONTROL_REG},
	{AS3711_CURR3, AS3711_CURR3_CONTROL_REG},
};

/*
 * as3711 ldo analog
 */
static int as3711_ldo_ana_is_enabled(struct regulator_dev *dev)
{
	u8 val;
	struct as3711 *as3711 = rdev_get_drvdata(dev);

	val = as3711_reg_read(as3711, as3711_reg_lookup[rdev_get_id(dev)].reg);
	return (val & AS3711_LDO_ANA_ON_MASK) != 0;
}

static int as3711_ldo_ana_enable(struct regulator_dev *dev)
{
	struct as3711 *as3711 = rdev_get_drvdata(dev);

	return as3711_set_bits(as3711, as3711_reg_lookup[rdev_get_id(dev)].reg,
			       AS3711_LDO_ANA_ON_MASK, AS3711_LDO_ANA_ON_BIT);
}

static int as3711_ldo_ana_disable(struct regulator_dev *dev)
{
	struct as3711 *as3711 = rdev_get_drvdata(dev);

	return as3711_set_bits(as3711, as3711_reg_lookup[rdev_get_id(dev)].reg,
			       AS3711_LDO_ANA_ON_MASK, 0);
}

static int as3711_ldo_ana_list_voltage(struct regulator_dev *dev,
				       unsigned selector)
{
	if (selector > AS3711_LDO_ANA_VSEL_MASK)
		return -EINVAL;

	if (selector < 16)
		return 1200000 + (selector * 50000);

	return 1800000 + ((selector - 16) * 100000);
}

static int as3711_ldo_ana_get_voltage(struct regulator_dev *dev)
{
	u8 val;
	struct as3711 *as3711 = rdev_get_drvdata(dev);

	val = as3711_reg_read(as3711, as3711_reg_lookup[rdev_get_id(dev)].reg);
	val &= AS3711_LDO_ANA_VSEL_MASK;

	return as3711_ldo_ana_list_voltage(dev, val);
}

static int as3711_ldo_ana_set_voltage(struct regulator_dev *dev,
				      int min_uV, int max_uV)
{
	u8 val;
	struct as3711 *as3711 = rdev_get_drvdata(dev);

	if (min_uV < 1200000 || min_uV > 3300000)
		return -EINVAL;

	if (min_uV < 1950001) {
		/* Steps of 50mV from 1200mV;  */
		val = (min_uV - 1150001) / 50000;

		if ((val * 50000) + 1200000 > max_uV)
			return -EINVAL;
		BUG_ON((val * 50000) + 1200000 < min_uV);
	} else {
		/* Steps of 100mV from 1800mV */
		val = ((min_uV - 1700001) / 100000);

		if ((val * 100000) + 1800000 > max_uV)
			return -EINVAL;
		BUG_ON((val * 100000) + 1800000 < min_uV);

		val += 0x10;
	}

	return as3711_set_bits(as3711, as3711_reg_lookup[rdev_get_id(dev)].reg,
			       AS3711_LDO_ANA_VSEL_MASK, val);
}

static int as3711_ldo_ana_get_current_limit(struct regulator_dev *dev)
{
	u8 val;
	struct as3711 *as3711 = rdev_get_drvdata(dev);

	val = as3711_reg_read(as3711, as3711_reg_lookup[rdev_get_id(dev)].reg);
	val &= AS3711_LDO_ANA_ILIMIT_MASK;

	/* return ldo specific values */
	if (val)
		return 250000;

	return 150000;
}

static int as3711_ldo_ana_set_current_limit(struct regulator_dev *dev,
					    int min_uA, int max_uA)
{
	u8 val;
	struct as3711 *as3711 = rdev_get_drvdata(dev);

	/* we check the values in case the constraints are wrong */
	if (min_uA <= 150000 && max_uA >= 150000)
		val = 0;
	else if (min_uA > 150000 && max_uA >= 250000)
		val = AS3711_LDO_ANA_ILIMIT_BIT;
	else
		return -EINVAL;

	return as3711_set_bits(as3711, as3711_reg_lookup[rdev_get_id(dev)].reg,
			       AS3711_LDO_ANA_ILIMIT_MASK, val);
}

static struct regulator_ops as3711_ldo_ana_ops = {
	.is_enabled = as3711_ldo_ana_is_enabled,
	.enable = as3711_ldo_ana_enable,
	.disable = as3711_ldo_ana_disable,
	.list_voltage = as3711_ldo_ana_list_voltage,
	.get_voltage = as3711_ldo_ana_get_voltage,
	.set_voltage = as3711_ldo_ana_set_voltage,
	.get_current_limit = as3711_ldo_ana_get_current_limit,
	.set_current_limit = as3711_ldo_ana_set_current_limit,
};

/*
 * as3711 ldo digital
 */
static int as3711_ldo_dig_is_enabled(struct regulator_dev *dev)
{
	u8 val = 0;
	struct as3711 *as3711 = rdev_get_drvdata(dev);

	val = as3711_reg_read(as3711, as3711_reg_lookup[rdev_get_id(dev)].reg);
	return (val & AS3711_LDO_DIG_ON_MASK) != 0;
}

static int as3711_ldo_dig_enable(struct regulator_dev *dev)
{
	struct as3711 *as3711 = rdev_get_drvdata(dev);

	return as3711_set_bits(as3711, as3711_reg_lookup[rdev_get_id(dev)].reg,
			       AS3711_LDO_DIG_ON_MASK, AS3711_LDO_DIG_ON_BIT);
}

static int as3711_ldo_dig_disable(struct regulator_dev *dev)
{
	struct as3711 *as3711 = rdev_get_drvdata(dev);

	return as3711_set_bits(as3711, as3711_reg_lookup[rdev_get_id(dev)].reg,
			       AS3711_LDO_DIG_ON_MASK, 0);
}

static int as3711_ldo_dig_list_voltage(struct regulator_dev *dev,
				       unsigned selector)
{
	if (selector > AS3711_LDO_DIG_VSEL_MASK)
		return -EINVAL;

	if (selector <= 0x10)
		return 900000 + (selector * 50000);

	if (selector >= 0x20 && selector <= 0x3F)
		return 1750000 + ((selector - 0x20) * 50000);

	return -EINVAL;
}

static int as3711_ldo_dig_get_voltage(struct regulator_dev *dev)
{
	u8 val;
	struct as3711 *as3711 = rdev_get_drvdata(dev);

	val = as3711_reg_read(as3711, as3711_reg_lookup[rdev_get_id(dev)].reg);
	val &= AS3711_LDO_DIG_VSEL_MASK;

	return as3711_ldo_dig_list_voltage(dev, val);
}

static int as3711_ldo_dig_set_voltage(struct regulator_dev *dev,
				      int min_uV, int max_uV)
{
	u8 val;
	struct as3711 *as3711 = rdev_get_drvdata(dev);

	if (min_uV < 900000 || min_uV > 3300000)
		return -EINVAL;

	if (min_uV < 1700001) {
		/* Steps of 50mV from 900mV;  */
		val = (min_uV - 850001) / 50000;

		if ((val * 50000) + 900000 > max_uV)
			return -EINVAL;

		BUG_ON((val * 50000) + 900000 < min_uV);
	} else {
		/* Steps of 50mV from 1750mV */
		val = ((min_uV - 1700001) / 50000);

		if ((val * 50000) + 1750000 > max_uV)
			return -EINVAL;

		BUG_ON((val * 50000) + 1750000 < min_uV);

		val += 0x20;
	}

	return as3711_set_bits(as3711, as3711_reg_lookup[rdev_get_id(dev)].reg,
			       AS3711_LDO_DIG_VSEL_MASK, val);
}

static int as3711_ldo_dig_get_current_limit(struct regulator_dev *dev)
{
	struct as3711 *as3711 = rdev_get_drvdata(dev);
	u8 val;

	val = as3711_reg_read(as3711, as3711_reg_lookup[rdev_get_id(dev)].reg);
	val &= AS3711_LDO_DIG_ILIMIT_MASK;

	/* return ldo specific values */
	if (val)
		return 300000;

	return 150000;
}

static int as3711_ldo_dig_set_current_limit(struct regulator_dev *dev,
					    int min_uA, int max_uA)
{
	struct as3711 *as3711 = rdev_get_drvdata(dev);
	u8 val;

	/* we check the values in case the constraints are wrong */
	if (min_uA <= 150000 && max_uA >= 150000)
		val = 0;
	else if (min_uA > 150000 && max_uA >= 300000)
		val = AS3711_LDO_DIG_ILIMIT_BIT;
	else
		return -EINVAL;

	return as3711_set_bits(as3711, as3711_reg_lookup[rdev_get_id(dev)].reg,
			       AS3711_LDO_DIG_ILIMIT_MASK, val);
}

static struct regulator_ops as3711_ldo_dig_ops = {
	.is_enabled = as3711_ldo_dig_is_enabled,
	.enable = as3711_ldo_dig_enable,
	.disable = as3711_ldo_dig_disable,
	.list_voltage = as3711_ldo_dig_list_voltage,
	.get_voltage = as3711_ldo_dig_get_voltage,
	.set_voltage = as3711_ldo_dig_set_voltage,
	.get_current_limit = as3711_ldo_dig_get_current_limit,
	.set_current_limit = as3711_ldo_dig_set_current_limit,
};

/*
 * as3711 step down
 */
static int as3711_sd_is_enabled(struct regulator_dev *dev)
{
	u8 val;
	u8 reg_id = rdev_get_id(dev);
	struct as3711 *as3711 = rdev_get_drvdata(dev);

	val = as3711_reg_read(as3711, AS3711_SD_CONTROL_REG);

	switch (rdev_get_id(dev)) {
	case AS3711_SD1:
		return (val & AS3711_SD1_CTRL_MASK) != 0;
	case AS3711_SD2:
		return (val & AS3711_SD2_CTRL_MASK) != 0;
	case AS3711_SD3:
		return (val & AS3711_SD3_CTRL_MASK) != 0;
	case AS3711_SD4:
		return (val & AS3711_SD4_CTRL_MASK) != 0;
	default:
		printk(KERN_ERR "%s: regulator id %d invalid.\n", __func__,
		       reg_id);
	}

	return -ERANGE;
}

static int as3711_sd_enable(struct regulator_dev *dev)
{
	u8 val, mask;
	u8 reg_id = rdev_get_id(dev);
	struct as3711 *as3711 = rdev_get_drvdata(dev);

	switch (reg_id) {
	case AS3711_SD1:
		val = AS3711_SD1_ON;
		mask = AS3711_SD1_CTRL_MASK;
		break;
	case AS3711_SD2:
		val = AS3711_SD2_ON;
		mask = AS3711_SD2_CTRL_MASK;
		break;
	case AS3711_SD3:
		val = AS3711_SD3_ON;
		mask = AS3711_SD3_CTRL_MASK;
		break;
	case AS3711_SD4:
		val = AS3711_SD4_ON;
		mask = AS3711_SD4_CTRL_MASK;
		break;
	default:
		printk(KERN_ERR "%s: regulator id %d invalid.\n", __func__,
		       reg_id);
		return -ERANGE;
	}

	return as3711_set_bits(as3711, AS3711_SD_CONTROL_REG, mask, val);
}

static int as3711_sd_disable(struct regulator_dev *dev)
{
	u8 val, mask;
	u8 reg_id = rdev_get_id(dev);
	struct as3711 *as3711 = rdev_get_drvdata(dev);

	switch (reg_id) {
	case AS3711_SD1:
		val = AS3711_SD1_OFF;
		mask = AS3711_SD1_CTRL_MASK;
		break;
	case AS3711_SD2:
		val = AS3711_SD2_OFF;
		mask = AS3711_SD2_CTRL_MASK;
		break;
	case AS3711_SD3:
		val = AS3711_SD3_OFF;
		mask = AS3711_SD3_CTRL_MASK;
		break;
	case AS3711_SD4:
		val = AS3711_SD4_OFF;
		mask = AS3711_SD4_CTRL_MASK;
		break;
	default:
		printk(KERN_ERR "%s: regulator id %d invalid.\n", __func__,
		       reg_id);
		return -ERANGE;
	}

	return as3711_set_bits(as3711, AS3711_SD_CONTROL_REG, mask, val);
}

static unsigned int as3711_sd_get_mode(struct regulator_dev *dev)
{
	u8 val;
	u8 reg_id = rdev_get_id(dev);
	struct as3711 *as3711 = rdev_get_drvdata(dev);

	val = as3711_reg_read(as3711, AS3711_SD_CONTROL1_REG);

	switch (rdev_get_id(dev)) {
	case AS3711_SD1:
		if ((val & AS3711_SD1_MODE_MASK) == AS3711_SD1_MODE_FAST)
			return REGULATOR_MODE_FAST;
		else
			return REGULATOR_MODE_NORMAL;
	case AS3711_SD2:
		if ((val & AS3711_SD2_MODE_MASK) == AS3711_SD2_MODE_FAST)
			return REGULATOR_MODE_FAST;
		else
			return REGULATOR_MODE_NORMAL;
	case AS3711_SD3:
		if ((val & AS3711_SD3_MODE_MASK) == AS3711_SD3_MODE_FAST)
			return REGULATOR_MODE_FAST;
		else
			return REGULATOR_MODE_NORMAL;
	case AS3711_SD4:
		if ((val & AS3711_SD4_MODE_MASK) == AS3711_SD4_MODE_FAST)
			return REGULATOR_MODE_FAST;
		else
			return REGULATOR_MODE_NORMAL;
	default:
		printk(KERN_ERR "%s: regulator id %d invalid.\n", __func__,
		       reg_id);
	}

	return -ERANGE;
}

static int as3711_sd_set_mode(struct regulator_dev *dev, unsigned int mode)
{
	u8 val, mask;
	u8 reg_id = rdev_get_id(dev);
	struct as3711 *as3711 = rdev_get_drvdata(dev);

	if (mode != REGULATOR_MODE_FAST && mode != REGULATOR_MODE_NORMAL)
		return -EINVAL;

	switch (reg_id) {
	case AS3711_SD1:
		if (mode == REGULATOR_MODE_FAST)
			val = AS3711_SD1_MODE_FAST;
		else
			val = AS3711_SD1_MODE_NORMAL;

		mask = AS3711_SD1_MODE_MASK;
		break;
	case AS3711_SD2:
		if (mode == REGULATOR_MODE_FAST)
			val = AS3711_SD2_MODE_FAST;
		else
			val = AS3711_SD2_MODE_NORMAL;

		mask = AS3711_SD2_MODE_MASK;
		break;
	case AS3711_SD3:
		if (mode == REGULATOR_MODE_FAST)
			val = AS3711_SD3_MODE_FAST;
		else
			val = AS3711_SD3_MODE_NORMAL;

		mask = AS3711_SD3_MODE_MASK;
		break;
	case AS3711_SD4:
		if (mode == REGULATOR_MODE_FAST)
			val = AS3711_SD4_MODE_FAST;
		else
			val = AS3711_SD4_MODE_NORMAL;

		mask = AS3711_SD4_MODE_MASK;
		break;
	default:
		printk(KERN_ERR "%s: regulator id %d invalid.\n", __func__,
		       reg_id);
		return -EINVAL;
	}

	return as3711_set_bits(as3711, AS3711_SD_CONTROL1_REG, mask, val);
}

static int as3711_sd_list_voltage(struct regulator_dev *dev, unsigned selector)
{
	if (selector > AS3711_SD_VSEL_MASK)
		return -EINVAL;

	if (selector == 0)
		return 0;

	if (selector <= 0x40)
		return 600000 + selector * 12500;

	if (selector <= 0x70)
		return 1400000 + ((selector - 0x40) * 25000);

	if (selector <= 0x7F)
		return 2600000 + ((selector - 0x70) * 50000);

	return -ERANGE;
}

static int as3711_sd_get_voltage(struct regulator_dev *dev)
{
	struct as3711 *as3711 = rdev_get_drvdata(dev);
	u8 val;

	val = as3711_reg_read(as3711, as3711_reg_lookup[rdev_get_id(dev)].reg);
	val &= AS3711_SD_VSEL_MASK;

	return as3711_sd_list_voltage(dev, val);
}

static int as3711_sd_set_voltage(struct regulator_dev *dev,
				 int min_uV, int max_uV)
{
	struct as3711 *as3711 = rdev_get_drvdata(dev);
	u8 val;

	/*       0 ... 0x00 volts
	 *  612500 ... 1400000: 0x01 - 0x40, 12.5mV steps
	 * 1425000 ... 2600000: 0x41 - 0x70, 25mV steps
	 * 2650000 ... 3350000: 0x41 - 0x70, 50mV steps */

	if (min_uV < 0 || min_uV > 3350000)
		return -EINVAL;

	if (min_uV < 612500) {

		if (max_uV < 612500)
			return -EINVAL;
		val = 1;

	} else if (min_uV < 1400001) {

		val = (min_uV - 587501) / 12500;

		if ((val * 12500) + 600000 > max_uV)
			return -EINVAL;

		BUG_ON((val * 12500) + 600000 < min_uV);

	} else if (min_uV < 2600001) {

		val = ((min_uV - 1375001) / 25000);

		if ((val * 25000) + 1400000 > max_uV)
			return -EINVAL;

		BUG_ON((val * 25000) + 1400000 < min_uV);

		val += 0x40;

	} else {

		val = ((min_uV - 2550001) / 50000);

		if ((val * 50000) + 2600000 > max_uV)
			return -EINVAL;

		BUG_ON((val * 50000) + 2600000 < min_uV);

		val += 0x70;
	}

	return as3711_set_bits(as3711, as3711_reg_lookup[rdev_get_id(dev)].reg,
			       AS3711_SD_VSEL_MASK, val);
}

static struct regulator_ops as3711_sd_ops = {
	.is_enabled = as3711_sd_is_enabled,
	.enable = as3711_sd_enable,
	.disable = as3711_sd_disable,
	.list_voltage = as3711_sd_list_voltage,
	.get_voltage = as3711_sd_get_voltage,
	.set_voltage = as3711_sd_set_voltage,
	.get_mode = as3711_sd_get_mode,
	.set_mode = as3711_sd_set_mode,
};

/*
 * as3711 step up
 */
static int as3711_su_is_enabled(struct regulator_dev *dev)
{
	struct as3711 *as3711 = rdev_get_drvdata(dev);
	u8 val;

	val = as3711_reg_read(as3711, as3711_reg_lookup[rdev_get_id(dev)].reg);

	return (val & AS3711_SU_ON_BIT_MASK) != AS3711_SU_OFF;
}

static int as3711_su_enable(struct regulator_dev *dev)
{
	struct as3711 *as3711 = rdev_get_drvdata(dev);

	return as3711_set_bits(as3711, as3711_reg_lookup[rdev_get_id(dev)].reg,
			       AS3711_SU_ON_BIT_MASK, AS3711_SU_ON);
}

static int as3711_su_disable(struct regulator_dev *dev)
{
	struct as3711 *as3711 = rdev_get_drvdata(dev);

	return as3711_set_bits(as3711, as3711_reg_lookup[rdev_get_id(dev)].reg,
			       AS3711_SU_ON_BIT_MASK, AS3711_SU_OFF);
}

static struct regulator_ops as3711_su_ops = {
	.is_enabled = as3711_su_is_enabled,
	.enable = as3711_su_enable,
	.disable = as3711_su_disable,
};

/*
 * as3711 current sinks
 */
static int as3711_curr_is_enabled(struct regulator_dev *dev)
{
	struct as3711 *as3711 = rdev_get_drvdata(dev);
	u8 val;

	val = as3711_reg_read(as3711, AS3711_CURR_CONTROL_REG);

	switch (rdev_get_id(dev)) {
	case AS3711_CURR1:
		return (val & AS3711_CURR1_CTRL_MASK) != 0;
	case AS3711_CURR2:
		return (val & AS3711_CURR2_CTRL_MASK) != 0;
	case AS3711_CURR3:
		return (val & AS3711_CURR3_CTRL_MASK) != 0;
	}

	return -ERANGE;
}

static int as3711_curr_enable(struct regulator_dev *dev)
{
	u8 val, mask;
	u8 reg_id = rdev_get_id(dev);
	struct as3711 *as3711 = rdev_get_drvdata(dev);

	switch (reg_id) {
	case AS3711_CURR1:
		val = AS3711_CURR1_SINK_ACTIVE;
		mask = AS3711_CURR1_CTRL_MASK;
		break;
	case AS3711_CURR2:
		val = AS3711_CURR2_SINK_ACTIVE;
		mask = AS3711_CURR2_CTRL_MASK;
		break;
	case AS3711_CURR3:
		val = AS3711_CURR3_SINK_ACTIVE;
		mask = AS3711_CURR3_CTRL_MASK;
		break;
	default:
		printk(KERN_ERR "%s: regulator id %d invalid.\n", __func__,
		       reg_id);
		return -ERANGE;
	}

	return as3711_set_bits(as3711, AS3711_CURR_CONTROL_REG, mask, val);
}

static int as3711_curr_disable(struct regulator_dev *dev)
{
	u8 val, mask;
	u8 reg_id = rdev_get_id(dev);
	struct as3711 *as3711 = rdev_get_drvdata(dev);

	switch (reg_id) {
	case AS3711_CURR1:
		val = AS3711_CURR1_SINK_OFF;
		mask = AS3711_CURR1_CTRL_MASK;
		break;
	case AS3711_CURR2:
		val = AS3711_CURR2_SINK_OFF;
		mask = AS3711_CURR2_CTRL_MASK;
		break;
	case AS3711_CURR3:
		val = AS3711_CURR3_SINK_OFF;
		mask = AS3711_CURR3_CTRL_MASK;
		break;
	default:
		printk(KERN_ERR "%s: regulator id %d invalid.\n", __func__,
		       reg_id);
		return -ERANGE;
	}

	return as3711_set_bits(as3711, AS3711_CURR_CONTROL_REG, mask, val);
}

static int as3711_curr_get_current_limit(struct regulator_dev *dev)
{
	struct as3711 *as3711 = rdev_get_drvdata(dev);
	u8 val;

	val = as3711_reg_read(as3711, as3711_reg_lookup[rdev_get_id(dev)].reg);
	val &= AS3711_CURR_CSEL_MASK;

	return val * 150;
}

static int as3711_curr_set_current_limit(struct regulator_dev *dev,
					 int min_uA, int max_uA)
{
	struct as3711 *as3711 = rdev_get_drvdata(dev);
	u8 val;

	/* we check the values in case the constraints are wrong */
	if (min_uA < 0 || min_uA > 38250)
		return -EINVAL;

	val = (u8) ((min_uA + 149) / 150);

	if ((val * 150) < min_uA || (val * 150) > max_uA) {
		printk(KERN_ERR
		       "%s: cannot set current limit min=%duA max=%duA\n",
		       __func__, min_uA, max_uA);
		return -EINVAL;
	}

	return as3711_set_bits(as3711, as3711_reg_lookup[rdev_get_id(dev)].reg,
			       AS3711_CURR_CSEL_MASK, val);
}

static struct regulator_ops as3711_curr_ops = {
	.is_enabled = as3711_curr_is_enabled,
	.enable = as3711_curr_enable,
	.disable = as3711_curr_disable,
	.get_current_limit = as3711_curr_get_current_limit,
	.set_current_limit = as3711_curr_set_current_limit,
};

static struct regulator_desc regulators[] = {
	{
	 .name = "as3711-ldo1-analog",
	 .id = AS3711_LDO1_ANA,
	 .ops = &as3711_ldo_ana_ops,
	 .n_voltages = AS3711_LDO_ANA_VSEL_MASK + 1,
	 .type = REGULATOR_VOLTAGE,
	 .owner = THIS_MODULE,
	 },
	{
	 .name = "as3711-ldo2-analog",
	 .id = AS3711_LDO2_ANA,
	 .ops = &as3711_ldo_ana_ops,
	 .n_voltages = AS3711_LDO_ANA_VSEL_MASK + 1,
	 .type = REGULATOR_VOLTAGE,
	 .owner = THIS_MODULE,
	 },
	{
	 .name = "as3711-ldo3-digital",
	 .id = AS3711_LDO3_DIG,
	 .ops = &as3711_ldo_dig_ops,
	 .n_voltages = AS3711_LDO_DIG_VSEL_MASK + 1,
	 .type = REGULATOR_VOLTAGE,
	 .owner = THIS_MODULE,
	 },
	{
	 .name = "as3711-ldo4-digital",
	 .id = AS3711_LDO4_DIG,
	 .ops = &as3711_ldo_dig_ops,
	 .n_voltages = AS3711_LDO_DIG_VSEL_MASK + 1,
	 .type = REGULATOR_VOLTAGE,
	 .owner = THIS_MODULE,
	 },
	{
	 .name = "as3711-ldo5-digital",
	 .id = AS3711_LDO5_DIG,
	 .ops = &as3711_ldo_dig_ops,
	 .n_voltages = AS3711_LDO_DIG_VSEL_MASK + 1,
	 .type = REGULATOR_VOLTAGE,
	 .owner = THIS_MODULE,
	 },
	{
	 .name = "as3711-ldo6-digital",
	 .id = AS3711_LDO6_DIG,
	 .ops = &as3711_ldo_dig_ops,
	 .n_voltages = AS3711_LDO_DIG_VSEL_MASK + 1,
	 .type = REGULATOR_VOLTAGE,
	 .owner = THIS_MODULE,
	 },
	{
	 .name = "as3711-ldo7-digital",
	 .id = AS3711_LDO7_DIG,
	 .ops = &as3711_ldo_dig_ops,
	 .n_voltages = AS3711_LDO_DIG_VSEL_MASK + 1,
	 .type = REGULATOR_VOLTAGE,
	 .owner = THIS_MODULE,
	 },
	{
	 .name = "as3711-ldo8-digital",
	 .id = AS3711_LDO8_DIG,
	 .ops = &as3711_ldo_dig_ops,
	 .n_voltages = AS3711_LDO_DIG_VSEL_MASK + 1,
	 .type = REGULATOR_VOLTAGE,
	 .owner = THIS_MODULE,
	 },
	{
	 .name = "as3711-sd1",
	 .id = AS3711_SD1,
	 .ops = &as3711_sd_ops,
	 .n_voltages = AS3711_SD_VSEL_MASK + 1,
	 .type = REGULATOR_VOLTAGE,
	 .owner = THIS_MODULE,
	 },
	{
	 .name = "as3711-sd2",
	 .id = AS3711_SD2,
	 .ops = &as3711_sd_ops,
	 .n_voltages = AS3711_SD_VSEL_MASK + 1,
	 .type = REGULATOR_VOLTAGE,
	 .owner = THIS_MODULE,
	 },
	{
	 .name = "as3711-sd3",
	 .id = AS3711_SD3,
	 .ops = &as3711_sd_ops,
	 .n_voltages = AS3711_SD_VSEL_MASK + 1,
	 .type = REGULATOR_VOLTAGE,
	 .owner = THIS_MODULE,
	 },
	{
	 .name = "as3711-sd4",
	 .id = AS3711_SD4,
	 .ops = &as3711_sd_ops,
	 .n_voltages = AS3711_SD_VSEL_MASK + 1,
	 .type = REGULATOR_VOLTAGE,
	 .owner = THIS_MODULE,
	 },
	{
	 .name = "as3711-su1",
	 .id = AS3711_SU1,
	 .ops = &as3711_su_ops,
	 .n_voltages = 1,
	 .type = REGULATOR_VOLTAGE,
	 .owner = THIS_MODULE,
	 },
	{
	 .name = "as3711-su2",
	 .id = AS3711_SU2,
	 .ops = &as3711_su_ops,
	 .n_voltages = 1,
	 .type = REGULATOR_VOLTAGE,
	 .owner = THIS_MODULE,
	 },
	{
	 .name = "as3711-curr1",
	 .id = AS3711_CURR1,
	 .ops = &as3711_curr_ops,
	 .n_voltages = 1,
	 .type = REGULATOR_CURRENT,
	 .owner = THIS_MODULE,
	 },
	{
	 .name = "as3711-curr2",
	 .id = AS3711_CURR2,
	 .ops = &as3711_curr_ops,
	 .n_voltages = 1,
	 .type = REGULATOR_CURRENT,
	 .owner = THIS_MODULE,
	 },
	{
	 .name = "as3711-curr3",
	 .id = AS3711_CURR3,
	 .ops = &as3711_curr_ops,
	 .n_voltages = 1,
	 .type = REGULATOR_CURRENT,
	 .owner = THIS_MODULE,
	 },
};

static int __devinit as3711_regulator_probe(struct platform_device *pdev)
{
	struct regulator_dev *rdev;
	struct as3711 *as3711 = dev_get_drvdata(&pdev->dev);

	if (WARN_ON(pdev->id < 0 || pdev->id >= AS3711_NUM_REGULATORS))
		return -EINVAL;

	rdev = regulator_register(&regulators[pdev->id], &pdev->dev,
				  pdev->dev.platform_data, as3711);

	if (IS_ERR(rdev)) {
		printk(KERN_ERR "as3711 register regulator rdev error\n");
		return PTR_ERR(rdev);
	}

	as3711->rdevs[pdev->id] = rdev;

	return 0;
}

static int __devexit as3711_regulator_remove(struct platform_device *pdev)
{
	struct as3711 *as3711 = dev_get_drvdata(&pdev->dev);

	if (WARN_ON(pdev->id < 0 || pdev->id >= AS3711_NUM_REGULATORS))
		return -EINVAL;

	if (as3711->rdevs[pdev->id]) {
		regulator_unregister(as3711->rdevs[pdev->id]);
		as3711->rdevs[pdev->id] = NULL;
	}

	return 0;
}

static struct platform_driver as3711_regulator_driver = {
	.driver = {
		   .name = "as3711-regulator",
		   .owner = THIS_MODULE,
		   },
	.probe = as3711_regulator_probe,
	.remove = __devexit_p(as3711_regulator_remove),
};

static int __init as3711_regulator_init(void)
{
	return platform_driver_register(&as3711_regulator_driver);
}

subsys_initcall(as3711_regulator_init);

static void __exit as3711_regulator_exit(void)
{
	platform_driver_unregister(&as3711_regulator_driver);
}

module_exit(as3711_regulator_exit);

MODULE_AUTHOR("Roman Schneider <roman.schneider@ams.com>");
MODULE_DESCRIPTION("AS3711 regulator driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:as3711-regulator");
