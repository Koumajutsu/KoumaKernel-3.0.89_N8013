/*
 * max8997.c - Voltage regulator driver for the Maxim 8997
 *
 *  Copyright (C) 2009-2010 Samsung Electronics
 *
 *  based on max8998.c
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/mfd/max8997.h>
#include <linux/mfd/max8997-private.h>

#include <mach/sec_debug.h>

struct max8997_data {
	struct device		*dev;
	struct max8997_dev	*iodev;
	int			num_regulators;
	struct regulator_dev	**rdev;
	bool			buck1_gpiodvs;
	int			buck_set1;
	int			buck_set2;
	int			buck_set3;
	u8                      buck1_vol[8]; /* voltages for selection */
	unsigned int		buck1_idx; /* index to last changed voltage */
					   /* value in a set */
	bool			buck_ramp_en;
	int			buck_ramp_delay;
	struct max8997_buck1_dvs_funcs funcs;
	struct mutex		dvs_lock;
};

struct vol_cur_map_desc {
	int min;
	int max;
	int step;
};

/* Voltage maps */
static const struct vol_cur_map_desc ldos_vol_cur_map_desc = {
	.min = 800,	.step = 50,	.max = 3950,
};
static const struct vol_cur_map_desc buck1245_vol_cur_map_desc = {
	.min = 650,	.step = 25,	.max = 2225,
};
static const struct vol_cur_map_desc buck37_vol_cur_map_desc = {
	.min = 750,	.step = 50,	.max = 3900,
};

/* flash currents just aren't matching up right! */
static const struct vol_cur_map_desc flash_vol_cur_map_desc = {
	.min = 23440,	.step = 23440,	.max = 750080,
};
static const struct vol_cur_map_desc movie_vol_cur_map_desc = {
	.min = 15625,	.step = 15625,	.max = 250000,
};
#ifdef MAX8997_SUPPORT_TORCH
static const struct vol_cur_map_desc torch_vol_cur_map_desc = {
	.min = 15625,	.step = 15625,	.max = 250000,
};
#endif /* MAX8997_SUPPORT_TORCH */

static const struct vol_cur_map_desc *ldo_vol_cur_map[] = {
	NULL,
	&ldos_vol_cur_map_desc,		/* LDO1 */
	&ldos_vol_cur_map_desc,		/* LDO2 */
	&ldos_vol_cur_map_desc,		/* LDO3 */
	&ldos_vol_cur_map_desc,		/* LDO4 */
	&ldos_vol_cur_map_desc,		/* LDO5 */
	&ldos_vol_cur_map_desc,		/* LDO6 */
	&ldos_vol_cur_map_desc,		/* LDO7 */
	&ldos_vol_cur_map_desc,		/* LDO8 */
	&ldos_vol_cur_map_desc,		/* LDO9 */
	&ldos_vol_cur_map_desc,		/* LDO10 */
	&ldos_vol_cur_map_desc,		/* LDO11 */
	&ldos_vol_cur_map_desc,		/* LDO12 */
	&ldos_vol_cur_map_desc,		/* LDO13 */
	&ldos_vol_cur_map_desc,		/* LDO14 */
	&ldos_vol_cur_map_desc,		/* LDO15 */
	&ldos_vol_cur_map_desc,		/* LDO16 */
	&ldos_vol_cur_map_desc,		/* LDO17 */
	&ldos_vol_cur_map_desc,		/* LDO18 */
	&ldos_vol_cur_map_desc,		/* LDO21 */
	&buck1245_vol_cur_map_desc,	/* BUCK1 */
	&buck1245_vol_cur_map_desc,	/* BUCK2 */
	&buck37_vol_cur_map_desc,	/* BUCK3 */
	&buck1245_vol_cur_map_desc,	/* BUCK4 */
	&buck1245_vol_cur_map_desc,	/* BUCK5 */
	NULL,				/* BUCK6 */
	&buck37_vol_cur_map_desc,	/* BUCK7 */
	NULL,				/* EN32KH_AP */
	NULL,				/* EN32KH_CP */
	NULL,				/* ENVICHG */
	NULL,				/* ESAFEOUT1 */
	NULL,				/* ESAFEOUT2 */
	&flash_vol_cur_map_desc,	/* FLASH_EN */
	&movie_vol_cur_map_desc,	/* MOVIE_EN */
#ifdef MAX8997_SUPPORT_TORCH
	&torch_vol_cur_map_desc,	/* TORCH */
#endif /* MAX8997_SUPPORT_TORCH */
};

static inline int max8997_get_ldo(struct regulator_dev *rdev)
{
	return rdev_get_id(rdev);
}

static int max8997_list_voltage(struct regulator_dev *rdev,
				unsigned int selector)
{
	const struct vol_cur_map_desc *desc;
	int ldo = max8997_get_ldo(rdev);
	int val;

	if (ldo >= ARRAY_SIZE(ldo_vol_cur_map))
		return -EINVAL;

	desc = ldo_vol_cur_map[ldo];
	if (desc == NULL)
		return -EINVAL;

	val = desc->min + desc->step * selector;
	if (val > desc->max)
		return -EINVAL;

	return val * 1000;
}

static int max8997_list_current(struct regulator_dev *rdev,
				unsigned int selector)
{
	const struct vol_cur_map_desc *desc;
	int co = max8997_get_ldo(rdev);
	int val;

	if (co >= ARRAY_SIZE(ldo_vol_cur_map))
		return -EINVAL;

	desc = ldo_vol_cur_map[co];
	if (desc == NULL)
		return -EINVAL;

	val = desc->min + desc->step * selector;
	if (val > desc->max)
		return -EINVAL;

	return val;
}

static int max8997_get_enable_register(struct regulator_dev *rdev,
					int *reg, int *shift)
{
	int ldo = max8997_get_ldo(rdev);

	switch (ldo) {
	case MAX8997_LDO1 ... MAX8997_LDO21:
		*reg = MAX8997_REG_LDO1CTRL + (ldo - MAX8997_LDO1);
		*shift = 6;
		break;
	case MAX8997_BUCK1:
		*reg = MAX8997_REG_BUCK1CTRL;
		*shift = 0;
		break;
	case MAX8997_BUCK2:
		*reg = MAX8997_REG_BUCK2CTRL;
		*shift = 0;
		break;
	case MAX8997_BUCK3:
		*reg = MAX8997_REG_BUCK3CTRL;
		*shift = 0;
		break;
	case MAX8997_BUCK4:
		*reg = MAX8997_REG_BUCK4CTRL;
		*shift = 0;
		break;
	case MAX8997_BUCK5:
		*reg = MAX8997_REG_BUCK5CTRL;
		*shift = 0;
		break;
	case MAX8997_BUCK6:
		*reg = MAX8997_REG_BUCK6CTRL1;
		*shift = 0;
		break;
	case MAX8997_BUCK7:
		*reg = MAX8997_REG_BUCK7CTRL;
		*shift = 0;
		break;
	case MAX8997_EN32KHZ_AP ... MAX8997_EN32KHZ_CP:
		*reg = MAX8997_REG_CONTROL1;
		*shift = 0 + (ldo - MAX8997_EN32KHZ_AP);
		break;
	case MAX8997_ENVICHG:
		*reg = MAX8997_REG_MBCCTRL1;
		*shift = 7;
		break;
	case MAX8997_ESAFEOUT1 ... MAX8997_ESAFEOUT2:
		*reg = MAX8997_REG_SAFEOUTCTRL;
		*shift = 6 + (ldo - MAX8997_ESAFEOUT1);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int max8997_get_enable_mask(struct regulator_dev *rdev)
{
	int ret = 0;
	int ldo = max8997_get_ldo(rdev);

	switch (ldo) {
	case MAX8997_LDO1 ... MAX8997_LDO21:
		ret = 3;
		break;
	case MAX8997_BUCK1 ... MAX8997_ESAFEOUT2:
		ret = 1;
		break;
	default:
		break;
	}

	return ret;
}

static int max8997_get_disable_val(struct regulator_dev *rdev)
{
	int ret = 0;
	int ldo = max8997_get_ldo(rdev);

	switch (ldo) {
	case MAX8997_LDO1:
	case MAX8997_LDO10:
	case MAX8997_LDO21:
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}

	return ret;
}

static int max8997_ldo_is_enabled(struct regulator_dev *rdev)
{
	struct max8997_data *max8997 = rdev_get_drvdata(rdev);
	struct i2c_client *i2c = max8997->iodev->i2c;
	int ret, reg, shift = 8;
	u8 val, mask;

	ret = max8997_get_enable_register(rdev, &reg, &shift);
	if (ret < 0)
		return ret;

	ret = max8997_read_reg(i2c, reg, &val);
	if (ret < 0)
		return ret;

	mask = max8997_get_enable_mask(rdev);

	return val & (mask << shift);
}

static int max8997_ldo_enable(struct regulator_dev *rdev)
{
	struct max8997_data *max8997 = rdev_get_drvdata(rdev);
	struct i2c_client *i2c = max8997->iodev->i2c;
	int reg, shift = 8, ret;
	u8 mask;

	ret = max8997_get_enable_register(rdev, &reg, &shift);
	if (ret < 0)
		return ret;

	mask = max8997_get_enable_mask(rdev);

	return max8997_update_reg(i2c, reg, mask<<shift, mask<<shift);
}

static int max8997_ldo_disable(struct regulator_dev *rdev)
{
	struct max8997_data *max8997 = rdev_get_drvdata(rdev);
	struct i2c_client *i2c = max8997->iodev->i2c;
	int reg, shift = 8, ret, val;
	u8 mask;

	ret = max8997_get_enable_register(rdev, &reg, &shift);
	if (ret < 0)
		return ret;

	mask = max8997_get_enable_mask(rdev);
	val = max8997_get_disable_val(rdev);

	return max8997_update_reg(i2c, reg, val<<shift, mask<<shift);
}

static int max8997_ldo_suspend_enable(struct regulator_dev *rdev)
{
	if (rdev->use_count > 0)
		return max8997_ldo_enable(rdev);
	else
		return max8997_ldo_disable(rdev);
}

static int max8997_get_voltage_register(struct regulator_dev *rdev,
				int *_reg, int *_shift, int *_mask)
{
	int ldo = max8997_get_ldo(rdev);
	struct max8997_data *max8997 = rdev_get_drvdata(rdev);
	int reg, shift = 0, mask = 0xff;

	switch (ldo) {
	case MAX8997_LDO1 ... MAX8997_LDO21:
		reg = MAX8997_REG_LDO1CTRL + (ldo - MAX8997_LDO1);
		mask = 0x3f;
		break;
	case MAX8997_BUCK1:
		reg = MAX8997_REG_BUCK1DVSTV1 + max8997->buck1_idx;
		break;
	case MAX8997_BUCK2:
		reg = MAX8997_REG_BUCK2DVSTV1 + 1;
		break;
	case MAX8997_BUCK3:
		reg = MAX8997_REG_BUCK3DVSTV;
		break;
	case MAX8997_BUCK4:
		reg = MAX8997_REG_BUCK4DVSTV;
		break;
	case MAX8997_BUCK5:
		reg = MAX8997_REG_BUCK5DVSTV1 + 1;
		break;
	case MAX8997_BUCK7:
		reg = MAX8997_REG_BUCK7DVSTV;
		break;
	default:
		return -EINVAL;
	}

	*_reg = reg;
	*_shift = shift;
	*_mask = mask;

	return 0;
}

static int max8997_get_voltage(struct regulator_dev *rdev)
{
	struct max8997_data *max8997 = rdev_get_drvdata(rdev);
	struct i2c_client *i2c = max8997->iodev->i2c;
	int reg, shift, mask, ret;
	int rid = max8997_get_rid(rdev);
	u8 val;

	ret = max8997_get_voltage_register(rdev, &reg, &shift, &mask);
	if (ret)
		return ret;

	if ((rid == MAX8997_BUCK1 && max8997->buck1_gpiodvs) ||
			(rid == MAX8997_BUCK2 && max8997->buck2_gpiodvs) ||
			(rid == MAX8997_BUCK5 && max8997->buck5_gpiodvs))
		reg += max8997->buck125_gpioindex;

	ret = max8997_read_reg(i2c, reg, &val);
	if (ret)
		return ret;

	val >>= shift;
	val &= mask;

	if (rdev->desc && rdev->desc->ops && rdev->desc->ops->list_voltage)
		return rdev->desc->ops->list_voltage(rdev, val);

	/*
	 * max8997_list_voltage returns value for any rdev with voltage_map,
	 * which works for "CHARGER" and "CHARGER TOPOFF" that do not have
	 * list_voltage ops (they are current regulators).
	 */
	return max8997_list_voltage(rdev, val);
}

static inline int max8997_get_voltage_proper_val(
		const struct voltage_map_desc *desc,
		int min_vol, int max_vol)
{
	int i = 0;

	if (desc == NULL)
		return -EINVAL;

	if (max_vol < desc->min || min_vol > desc->max)
		return -EINVAL;

	while (desc->min + desc->step * i < min_vol &&
			desc->min + desc->step * i < desc->max)
		i++;

	if (desc->min + desc->step * i > max_vol)
		return -EINVAL;

	if (i >= (1 << desc->n_bits))
		return -EINVAL;

	return i;
}

static int max8997_set_voltage_charger_cv(struct regulator_dev *rdev,
		int min_uV, int max_uV, unsigned *selector)
{
	struct max8997_data *max8997 = rdev_get_drvdata(rdev);
	struct i2c_client *i2c = max8997->iodev->i2c;
	int rid = max8997_get_rid(rdev);
	int lb, ub;
	int reg, shift = 0, mask, ret = 0;
	u8 val = 0x0;

	if (rid != MAX8997_CHARGER_CV)
		return -EINVAL;

	ret = max8997_get_voltage_register(rdev, &reg, &shift, &mask);
	if (ret)
		return ret;

	if (max_uV < 4000000 || min_uV > 4350000)
		return -EINVAL;

	if (min_uV <= 4000000) {
		if (max_uV >= 4000000)
			return -EINVAL;
		else
			val = 0x1;
	} else if (min_uV <= 4200000 && max_uV >= 4200000)
		val = 0x0;
	else {
		lb = (min_uV - 4000001) / 20000 + 2;
		ub = (max_uV - 4000000) / 20000 + 1;

		if (lb > ub)
			return -EINVAL;

		if (lb < 0xf)
			val = lb;
		else {
			if (ub >= 0xf)
				val = 0xf;
			else
				return -EINVAL;
		}
	}

	*selector = val;

	ret = max8997_update_reg(i2c, reg, val << shift, mask);

	return ret;
}

/*
 * For LDO1 ~ LDO21, BUCK1~5, BUCK7, CHARGER, CHARGER_TOPOFF
 * BUCK1, 2, and 5 are available if they are not controlled by gpio
 */
static int max8997_set_voltage_ldobuck(struct regulator_dev *rdev,
		int min_uV, int max_uV, unsigned *selector)
{
	struct max8997_data *max8997 = rdev_get_drvdata(rdev);
	struct i2c_client *i2c = max8997->iodev->i2c;
	int min_vol = min_uV / 1000, max_vol = max_uV / 1000;
	const struct voltage_map_desc *desc;
	int rid = max8997_get_rid(rdev);
	int reg, shift = 0, mask, ret;
	int i;
	u8 org;

	switch (rid) {
	case MAX8997_LDO1 ... MAX8997_LDO21:
		break;
	case MAX8997_BUCK1 ... MAX8997_BUCK5:
		break;
	case MAX8997_BUCK6:
		return -EINVAL;
	case MAX8997_BUCK7:
		break;
	case MAX8997_CHARGER:
		break;
	case MAX8997_CHARGER_TOPOFF:
		break;
	default:
		return -EINVAL;
	}

	desc = reg_voltage_map[rid];

	i = max8997_get_voltage_proper_val(desc, min_vol, max_vol);
	if (i < 0)
		return i;

	ret = max8997_get_voltage_register(rdev, &reg, &shift, &mask);
	if (ret)
		return ret;

	max8997_read_reg(i2c, reg, &org);
	org = (org & mask) >> shift;

	ret = max8997_update_reg(i2c, reg, i << shift, mask << shift);
	*selector = i;

	if (rid == MAX8997_BUCK1 || rid == MAX8997_BUCK2 ||
			rid == MAX8997_BUCK4 || rid == MAX8997_BUCK5) {
		/* If the voltage is increasing */
		if (org < i)
			udelay(DIV_ROUND_UP(desc->step * (i - org),
						max8997->ramp_delay));
	}

	return ret;
}

/*
 * Assess the damage on the voltage setting of BUCK1,2,5 by the change.
 *
 * When GPIO-DVS mode is used for multiple bucks, changing the voltage value
 * of one of the bucks may affect that of another buck, which is the side
 * effect of the change (set_voltage). This function examines the GPIO-DVS
 * configurations and checks whether such side-effect exists.
 */
static int max8997_assess_side_effect(struct regulator_dev *rdev,
		u8 new_val, int *best)
{
	struct max8997_data *max8997 = rdev_get_drvdata(rdev);
	int rid = max8997_get_rid(rdev);
	u8 *buckx_val[3];
	bool buckx_gpiodvs[3];
	int side_effect[8];
	int min_side_effect = INT_MAX;
	int i;

	*best = -1;

	switch (rid) {
	case MAX8997_BUCK1:
		rid = 0;
		break;
	case MAX8997_BUCK2:
		rid = 1;
		break;
	case MAX8997_BUCK5:
		rid = 2;
		break;
	default:
		return -EINVAL;
	}

	buckx_val[0] = max8997->buck1_vol;
	buckx_val[1] = max8997->buck2_vol;
	buckx_val[2] = max8997->buck5_vol;
	buckx_gpiodvs[0] = max8997->buck1_gpiodvs;
	buckx_gpiodvs[1] = max8997->buck2_gpiodvs;
	buckx_gpiodvs[2] = max8997->buck5_gpiodvs;

	for (i = 0; i < 8; i++) {
		int others;

		if (new_val != (buckx_val[rid])[i]) {
			side_effect[i] = -1;
			continue;
		}

		side_effect[i] = 0;
		for (others = 0; others < 3; others++) {
			int diff;

			if (others == rid)
				continue;
			if (buckx_gpiodvs[others] == false)
				continue; /* Not affected */
			diff = (buckx_val[others])[i] -
				(buckx_val[others])[max8997->buck125_gpioindex];
			if (diff > 0)
				side_effect[i] += diff;
			else if (diff < 0)
				side_effect[i] -= diff;
		}
		if (side_effect[i] == 0) {
			*best = i;
			return 0; /* NO SIDE EFFECT! Use This! */
		}
		if (side_effect[i] < min_side_effect) {
			min_side_effect = side_effect[i];
			*best = i;
		}
	}

	if (*best == -1)
		return -EINVAL;

	return side_effect[*best];
}

/*
 * For Buck 1 ~ 5 and 7. If it is not controlled by GPIO, this calls
 * max8997_set_voltage_ldobuck to do the job.
 */
static int max8997_set_voltage_buck(struct regulator_dev *rdev,
		int min_uV, int max_uV, unsigned *selector)
{
	struct max8997_data *max8997 = rdev_get_drvdata(rdev);
	int rid = max8997_get_rid(rdev);
	const struct voltage_map_desc *desc;
	int new_val, new_idx, damage, tmp_val, tmp_idx, tmp_dmg;
	bool gpio_dvs_mode = false;
	int min_vol = min_uV / 1000, max_vol = max_uV / 1000;

	if (rid < MAX8997_BUCK1 || rid > MAX8997_BUCK7)
		return -EINVAL;

	switch (rid) {
	case MAX8997_BUCK1:
		if (max8997->buck1_gpiodvs)
			gpio_dvs_mode = true;
		break;
	case MAX8997_BUCK2:
		if (max8997->buck2_gpiodvs)
			gpio_dvs_mode = true;
		break;
	case MAX8997_BUCK5:
		if (max8997->buck5_gpiodvs)
			gpio_dvs_mode = true;
		break;
	}

	if (!gpio_dvs_mode)
		return max8997_set_voltage_ldobuck(rdev, min_uV, max_uV,
						selector);

	desc = reg_voltage_map[rid];
	new_val = max8997_get_voltage_proper_val(desc, min_vol, max_vol);
	if (new_val < 0)
		return new_val;

	tmp_dmg = INT_MAX;
	tmp_idx = -1;
	tmp_val = -1;
	do {
		damage = max8997_assess_side_effect(rdev, new_val, &new_idx);
		if (damage == 0)
			goto out;

		if (tmp_dmg > damage) {
			tmp_idx = new_idx;
			tmp_val = new_val;
			tmp_dmg = damage;
		}

		new_val++;
	} while (desc->min + desc->step * new_val <= desc->max);

	new_idx = tmp_idx;
	new_val = tmp_val;

	if (max8997->ignore_gpiodvs_side_effect == false)
		return -EINVAL;

	dev_warn(&rdev->dev, "MAX8997 GPIO-DVS Side Effect Warning: GPIO SET:"
			" %d -> %d\n", max8997->buck125_gpioindex, tmp_idx);

out:
	if (new_idx < 0 || new_val < 0)
		return -EINVAL;

	max8997->buck125_gpioindex = new_idx;
	max8997_set_gpio(max8997);
	*selector = new_val;

	return 0;
}

static const int safeoutvolt[] = {
	3300000,
	4850000,
	4900000,
	4950000,
};

/* For SAFEOUT1 and SAFEOUT2 */
static int max8997_set_voltage_safeout(struct regulator_dev *rdev,
		int min_uV, int max_uV, unsigned *selector)
{
	struct max8997_data *max8997 = rdev_get_drvdata(rdev);
	struct i2c_client *i2c = max8997->iodev->i2c;
	int rid = max8997_get_rid(rdev);
	int reg, shift = 0, mask, ret;
	int i = 0;
	u8 val;

	if (rid != MAX8997_ESAFEOUT1 && rid != MAX8997_ESAFEOUT2)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(safeoutvolt); i++) {
		if (min_uV <= safeoutvolt[i] &&
				max_uV >= safeoutvolt[i])
			break;
	}

	if (i >= ARRAY_SIZE(safeoutvolt))
		return -EINVAL;

	if (i == 0)
		val = 0x3;
	else
		val = i - 1;

	ret = max8997_get_voltage_register(rdev, &reg, &shift, &mask);
	if (ret)
		return ret;

	ret = max8997_update_reg(i2c, reg, val << shift, mask << shift);
	*selector = val;

	return ret;
}

static int max8997_reg_enable_suspend(struct regulator_dev *rdev)
{
	return 0;
}

static int max8997_reg_disable_suspend(struct regulator_dev *rdev)
{
	struct max8997_data *max8997 = rdev_get_drvdata(rdev);
	struct i2c_client *i2c = max8997->iodev->i2c;
	int ret, reg, mask, pattern;
	int rid = max8997_get_rid(rdev);

	ret = max8997_get_enable_register(rdev, &reg, &mask, &pattern);
	if (ret)
		return ret;

	max8997_read_reg(i2c, reg, &max8997->saved_states[rid]);

	if (rid == MAX8997_LDO1 ||
			rid == MAX8997_LDO10 ||
			rid == MAX8997_LDO21) {
		dev_dbg(&rdev->dev, "Conditional Power-Off for %s\n",
				rdev->desc->name);
		return max8997_update_reg(i2c, reg, 0x40, mask);
	}

	dev_dbg(&rdev->dev, "Full Power-Off for %s (%xh -> %xh)\n",
			rdev->desc->name, max8997->saved_states[rid] & mask,
			(~pattern) & mask);
	return max8997_update_reg(i2c, reg, ~pattern, mask);
}

static struct regulator_ops max8997_ldo_ops = {
	.list_voltage		= max8997_list_voltage,
	.is_enabled		= max8997_reg_is_enabled,
	.enable			= max8997_reg_enable,
	.disable		= max8997_reg_disable,
	.get_voltage		= max8997_get_voltage,
	.set_voltage		= max8997_set_voltage_ldobuck,
	.set_suspend_enable	= max8997_reg_enable_suspend,
	.set_suspend_disable	= max8997_reg_disable_suspend,
};

static struct regulator_ops max8997_buck_ops = {
	.list_voltage		= max8997_list_voltage,
	.is_enabled		= max8997_reg_is_enabled,
	.enable			= max8997_reg_enable,
	.disable		= max8997_reg_disable,
	.get_voltage		= max8997_get_voltage,
	.set_voltage		= max8997_set_voltage_buck,
	.set_suspend_enable	= max8997_reg_enable_suspend,
	.set_suspend_disable	= max8997_reg_disable_suspend,
};

static struct regulator_ops max8997_fixedvolt_ops = {
	.list_voltage		= max8997_list_voltage,
	.is_enabled		= max8997_reg_is_enabled,
	.enable			= max8997_reg_enable,
	.disable		= max8997_reg_disable,
	.set_suspend_enable	= max8997_reg_enable_suspend,
	.set_suspend_disable	= max8997_reg_disable_suspend,
};

static struct regulator_ops max8997_safeout_ops = {
	.list_voltage		= max8997_list_voltage_safeout,
	.is_enabled		= max8997_reg_is_enabled,
	.enable			= max8997_reg_enable,
	.disable		= max8997_reg_disable,
	.get_voltage		= max8997_get_voltage,
	.set_voltage		= max8997_set_voltage_safeout,
	.set_suspend_enable	= max8997_reg_enable_suspend,
	.set_suspend_disable	= max8997_reg_disable_suspend,
};

static struct regulator_ops max8997_fixedstate_ops = {
	.list_voltage		= max8997_list_voltage_charger_cv,
	.get_voltage		= max8997_get_voltage,
	.set_voltage		= max8997_set_voltage_charger_cv,
};

static int max8997_set_voltage_ldobuck_wrap(struct regulator_dev *rdev,
		int min_uV, int max_uV)
{
	unsigned dummy;

	return max8997_set_voltage_ldobuck(rdev, min_uV, max_uV, &dummy);
}


static struct regulator_ops max8997_charger_ops = {
	.is_enabled		= max8997_reg_is_enabled,
	.enable			= max8997_reg_enable,
	.disable		= max8997_reg_disable,
	.get_current_limit	= max8997_get_voltage,
	.set_current_limit	= max8997_set_voltage_ldobuck_wrap,
};

static struct regulator_ops max8997_charger_fixedstate_ops = {
	.is_enabled		= max8997_reg_is_enabled,
	.get_current_limit	= max8997_get_voltage,
	.set_current_limit	= max8997_set_voltage_ldobuck_wrap,
};

#define regulator_desc_ldo(num)		{	\
	.name		= "LDO"#num,		\
	.id		= MAX8997_LDO##num,	\
	.ops		= &max8997_ldo_ops,	\
	.type		= REGULATOR_VOLTAGE,	\
	.owner		= THIS_MODULE,		\
}
#define regulator_desc_buck(num)		{	\
	.name		= "BUCK"#num,		\
	.id		= MAX8997_BUCK##num,	\
	.ops		= &max8997_buck_ops,	\
	.type		= REGULATOR_VOLTAGE,	\
	.owner		= THIS_MODULE,		\
}

static struct regulator_desc regulators[] = {
	regulator_desc_ldo(1),
	regulator_desc_ldo(2),
	regulator_desc_ldo(3),
	regulator_desc_ldo(4),
	regulator_desc_ldo(5),
	regulator_desc_ldo(6),
	regulator_desc_ldo(7),
	regulator_desc_ldo(8),
	regulator_desc_ldo(9),
	regulator_desc_ldo(10),
	regulator_desc_ldo(11),
	regulator_desc_ldo(12),
	regulator_desc_ldo(13),
	regulator_desc_ldo(14),
	regulator_desc_ldo(15),
	regulator_desc_ldo(16),
	regulator_desc_ldo(17),
	regulator_desc_ldo(18),
	regulator_desc_ldo(21),
	regulator_desc_buck(1),
	regulator_desc_buck(2),
	regulator_desc_buck(3),
	regulator_desc_buck(4),
	regulator_desc_buck(5),
	{
		.name	= "BUCK6",
		.id	= MAX8997_BUCK6,
		.ops	= &max8997_fixedvolt_ops,
		.type	= REGULATOR_VOLTAGE,
		.owner	= THIS_MODULE,
	},
	regulator_desc_buck(7),
	{
		.name	= "EN32KHz AP",
		.id	= MAX8997_EN32KHZ_AP,
		.ops	= &max8997_fixedvolt_ops,
		.type	= REGULATOR_VOLTAGE,
		.owner	= THIS_MODULE,
	}, {
		.name	= "EN32KHz CP",
		.id	= MAX8997_EN32KHZ_CP,
		.ops	= &max8997_fixedvolt_ops,
		.type	= REGULATOR_VOLTAGE,
		.owner	= THIS_MODULE,
	}, {
		.name	= "ENVICHG",
		.id	= MAX8997_ENVICHG,
		.ops	= &max8997_fixedvolt_ops,
		.type	= REGULATOR_VOLTAGE,
		.owner	= THIS_MODULE,
	}, {
		.name	= "ESAFEOUT1",
		.id	= MAX8997_ESAFEOUT1,
		.ops	= &max8997_safeout_ops,
		.type	= REGULATOR_VOLTAGE,
		.owner	 = THIS_MODULE,
	}, {
		.name	= "ESAFEOUT2",
		.id	= MAX8997_ESAFEOUT2,
		.ops	= &max8997_safeout_ops,
		.type	= REGULATOR_VOLTAGE,
		.owner	 = THIS_MODULE,
	}, {
		.name	= "CHARGER CV",
		.id	= MAX8997_CHARGER_CV,
		.ops	= &max8997_fixedstate_ops,
		.type	= REGULATOR_VOLTAGE,
		.owner	 = THIS_MODULE,
	}, {
		.name	= "CHARGER",
		.id	= MAX8997_CHARGER,
		.ops	= &max8997_charger_ops,
		.type	= REGULATOR_CURRENT,
		.owner	 = THIS_MODULE,
	}, {
		.name	= "CHARGER TOPOFF",
		.id	= MAX8997_CHARGER_TOPOFF,
		.ops	= &max8997_charger_fixedstate_ops,
		.type	= REGULATOR_CURRENT,
		.owner	 = THIS_MODULE,
	},
};

static __devinit int max8997_pmic_probe(struct platform_device *pdev)
{
	struct max8997_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct max8997_platform_data *pdata = dev_get_platdata(iodev->dev);
	struct regulator_dev **rdev;
	struct max8997_data *max8997;
	struct i2c_client *i2c;
	int i, size, ret = 0;

	if (!pdata) {
		dev_err(pdev->dev.parent, "No platform init data supplied\n");
		return -ENODEV;
	}

	max8997 = kzalloc(sizeof(struct max8997_data), GFP_KERNEL);
	if (!max8997)
		return -ENOMEM;

	size = sizeof(struct regulator_dev *) * pdata->num_regulators;
	max8997->rdev = kzalloc(size, GFP_KERNEL);
	if (!max8997->rdev) {
		ret = -ENOMEM;
		goto err3;
	}

	mutex_init(&max8997->dvs_lock);

	rdev = max8997->rdev;
	max8997->dev = &pdev->dev;
	max8997->iodev = iodev;
	max8997->num_regulators = pdata->num_regulators;
	platform_set_drvdata(pdev, max8997);
	i2c = max8997->iodev->i2c;
	max8997->buck1_gpiodvs = pdata->buck1_gpiodvs;
	max8997->buck_set1 = pdata->buck_set1;
	max8997->buck_set2 = pdata->buck_set2;
	max8997->buck_set3 = pdata->buck_set3;

	/* NOTE:
	 * This MAX8997 PMIC driver support only BUCK1 GPIO DVS
	 * because BUCK1(ARM clock voltage) is most frequently changed
	 */
	/* For the safety, set max voltage before DVS configuration */
	if (!pdata->buck1_max_vol || !pdata->buck2_max_vol
			|| !pdata->buck5_max_vol) {
		pr_err("MAX8997: must set buck max voltage!\n");
		goto err2;
	}

	ret = max8997_set_buck_max_voltage(max8997, 1, pdata->buck1_max_vol);
	if (ret < 0) {
		pr_err("MAX8997: fail to set buck1 max voltage!\n");
		goto err2;
	}

	ret = max8997_set_buck_max_voltage(max8997, 2, pdata->buck2_max_vol);
	if (ret < 0) {
		pr_err("MAX8997: fail to set buck2 max voltage!\n");
		goto err2;
	}

	ret = max8997_set_buck_max_voltage(max8997, 5, pdata->buck5_max_vol);
	if (ret < 0) {
		pr_err("MAX8997: fail to set buck5 max voltage!\n");
		goto err2;
	}

	/* NOTE: */
	/* For unused GPIO NOT marked as -1 (thereof equal to 0)  WARN_ON */
	/* will be displayed */
	/* Check if MAX8997 voltage selection GPIOs are defined */
	if (gpio_is_valid(max8997->buck_set1) &&
	    gpio_is_valid(max8997->buck_set2) &&
	    gpio_is_valid(max8997->buck_set3)) {
		/* Check if SET1 is not equal to 0 */
		if (!max8997->buck_set1) {
			pr_err("MAX8997 SET1 GPIO defined as 0 !\n");
			WARN_ON(!pdata->buck_set1);
			ret = -EIO;
			goto err2;
		}
		/* Check if SET2 is not equal to 0 */
		if (!max8997->buck_set2) {
			pr_err("MAX8998 SET2 GPIO defined as 0 !\n");
			WARN_ON(!pdata->buck_set2);
			ret = -EIO;
			goto err2;
		}
		/* Check if SET3 is not equal to 0 */
		if (!max8997->buck_set3) {
			pr_err("MAX8997 SET3 GPIO defined as 0 !\n");
			WARN_ON(!max8997->buck_set3);
			ret = -EIO;
			goto err2;
		}

		/* To prepare watchdog reset, index 0 of voltage table is
		 * always highest voltage.
		 * Default voltage of BUCK1,2,5 was configured by bootloader.
		 */
		max8997->buck1_idx = 1;
		gpio_request(max8997->buck_set1, "MAX8997 BUCK_SET1");
		gpio_direction_output(max8997->buck_set1, 1);
		gpio_request(max8997->buck_set2, "MAX8997 BUCK_SET2");
		gpio_direction_output(max8997->buck_set2, 0);
		gpio_request(max8997->buck_set3, "MAX8997 BUCK_SET3");
		gpio_direction_output(max8997->buck_set3, 0);

		if (max8997->buck1_gpiodvs) {
			/* Set predefined value for BUCK1 register 2 ~ 8 */
			ret = max8997_set_buck1_voltages(max8997,
				pdata->buck1_voltages, BUCK1_TABLE_SIZE);
			if (ret < 0)
				goto err2;
		}
	} else {
		pr_err("MAX8997 SETx GPIO is invalid!\n");
		goto err2;
	}

	max8997->funcs.set_buck1_dvs_table = max8997_set_buck1_dvs_table;
	if (pdata->register_buck1_dvs_funcs)
		pdata->register_buck1_dvs_funcs(&max8997->funcs);

	max8997_set_buckramp(max8997, pdata);

	if (pdata->flash_cntl_val) {
		ret =  max8997_write_reg(i2c, MAX8997_REG_FLASH_CNTL,
				pdata->flash_cntl_val);
		if (ret < 0) {
			dev_err(max8997->dev, "flash init failed: %d\n", ret);
			goto err2;
		}
	}

	if (pdata->mr_debounce_time)
		max8997_set_mr_debouce_time(max8997, pdata);

	for (i = 0; i < pdata->num_regulators; i++) {
		const struct vol_cur_map_desc *desc;
		int id = pdata->regulators[i].id;
		int index = id - MAX8997_LDO1;

		if (pdata->regulators[i].is_valid_regulator) {
			if (!pdata->regulators[i].is_valid_regulator(id,
						pdata->regulators[i].initdata))
				continue;
		}

		desc = ldo_vol_cur_map[id];
		if (desc && regulators[index].ops != &max8997_others_ops) {
			int count = (desc->max - desc->min) / desc->step + 1;
			regulators[index].n_voltages = count;
		}
		rdev[i] = regulator_register(&regulators[index], max8997->dev,
				pdata->regulators[i].initdata, max8997);
		if (IS_ERR(rdev[i])) {
			ret = PTR_ERR(rdev[i]);
			dev_err(max8997->dev, "regulator init failed\n");
			rdev[i] = NULL;
			goto err1;
		}
	}

	return 0;
err1:
	for (i = 0; i < max8997->num_regulators; i++)
		if (rdev[i])
			regulator_unregister(rdev[i]);
err2:
	kfree(max8997->rdev);
err3:
	kfree(max8997);

	return ret;
}

static int __devexit max8997_pmic_remove(struct platform_device *pdev)
{
	struct max8997_data *max8997 = platform_get_drvdata(pdev);
	struct regulator_dev **rdev = max8997->rdev;
	int i;

	for (i = 0; i < max8997->num_regulators; i++)
		if (rdev[i])
			regulator_unregister(rdev[i]);

	kfree(max8997->rdev);
	kfree(max8997);

	return 0;
}

static struct platform_driver max8997_pmic_driver = {
	.driver = {
		.name = "max8997-pmic",
		.owner = THIS_MODULE,
	},
	.probe = max8997_pmic_probe,
	.remove = __devexit_p(max8997_pmic_remove),
};

static int __init max8997_pmic_init(void)
{
	return platform_driver_register(&max8997_pmic_driver);
}
subsys_initcall(max8997_pmic_init);

static void __exit max8997_pmic_cleanup(void)
{
	platform_driver_unregister(&max8997_pmic_driver);
}
module_exit(max8997_pmic_cleanup);

MODULE_DESCRIPTION("MAXIM 8997 voltage regulator driver");
MODULE_AUTHOR("<ms925.kim@samsung.com>");
MODULE_LICENSE("GPL");
