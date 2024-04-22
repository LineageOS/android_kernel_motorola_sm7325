/*
 * et5904-regulator.h - Regulator definitions for ET5904
 * Copyright (C) 2021  ETEK Semiconductor Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __ET5904_H__
#define __ET5904_H__

/* System Control and Event Registers */


#include <linux/regulator/machine.h>


#define ET5904_REG_CONTROL		0x10


/* ET5904 REGULATOR IDs */
// #define ET5904_ID_REGULATOR1	0
// #define ET5904_ID_REGULATOR2	1
// #define ET5904_ID_REGULATOR3	2
// #define ET5904_ID_REGULATOR4	3


// #define ET5904_MAX_REGULATORS	7

/* Regulators */
enum {
	ET5904_ID_REGULATOR1,
	ET5904_ID_REGULATOR2,
	ET5904_ID_REGULATOR3,
	ET5904_ID_REGULATOR4,
	ET5904_MAX_REGULATORS,
};
enum et5904_chip_id {
	ET5904,
};

struct et5904_pdata {
	/*
	 * Number of regulator
	 */
	struct device_node *reg_node[ET5904_MAX_REGULATORS];
	struct regulator_init_data *init_data[ET5904_MAX_REGULATORS];
};

/**
 * et5904_regulator_data - regulator data
 * @id: regulator id
 * @name: regulator name
 * @init_data: regulator init data
 * @of_node: device tree node (optional)
 */
struct et5904_regulator_data {
	int id;
	const char *name;
	struct regulator_init_data *init_data;
	struct device_node *of_node;
};

/**
 * et5904_platform_data - platform data for et5904
 * @num_regulators: number of regulators used
 * @regulators: pointer to regulators used
 */
struct et5904_platform_data {
	int num_regulators;
	struct et5904_regulator_data *regulators;
};


struct et5904_regulator {
	struct device *dev;
	struct regmap *regmap;
	struct et5904_pdata *pdata;
	struct regulator_dev *rdev[ET5904_MAX_REGULATORS];
	int num_regulator;
	int chip_irq;
	int chip_id;
	// int reset_gpio;
};

#define	ET5904_REG_CHIPID			0x00
#define	ET5904_REG_ILIMIT			0x01
#define	ET5904_REG_RDIS			0x02
#define	ET5904_REG_DVO1			0x03
#define	ET5904_REG_DVO2			0x04
#define	ET5904_REG_AVO1			0x05
#define	ET5904_REG_AVO2			0x06
#define	ET5904_REG_SEQ1			0x0A
#define	ET5904_REG_SEQ2			0x0B
#define	ET5904_REG_LDO_EN			0x0E
#define	ET5904_REG_SEQ_C			0x0F
#define	ET5904_REG_ILIMT_COAR			0x10
#define ET5904_LDO_REG_INDEX(n)		(ET5904_REG_DVO1 + (n))

/* DEVICE IDs */
#define ET5904_DEVICE_ID	0x00

#define ET5904_BIT_0		(1 << 0)
#define ET5904_BIT_1		(1 << 1)
#define ET5904_BIT_2		(1 << 2)
#define ET5904_BIT_3		(1 << 3)
#define ET5904_BIT_4		(1 << 4)
#define ET5904_BIT_5		(1 << 5)
#define ET5904_BIT_6		(1 << 6)
#define ET5904_BIT_7		(1 << 7)

#endif	/* __ET5904_REGISTERS_H__ */
