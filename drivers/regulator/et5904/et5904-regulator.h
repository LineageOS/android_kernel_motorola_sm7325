// SPDX-License-Identifier: GPL-2.0+
/*
 * ET5904, Multi-Output Regulators
 * Copyright (C) 2019  Motorola Mobility LLC,
 *
 * Author: Huqian, Motorola Mobility LLC,
 */

#ifndef __ET5904_REGISTERS_H__
#define __ET5904_REGISTERS_H__

/* Registers */
#define ET5904_REG_NUM (ET5904_SEQ_STATUS-ET5904_CHIP_REV+1)

#define ET5904_CHIP_REV 0x00
#define ET5904_CURRENT_LIMITSEL 0x01
#define ET5904_DISCHARGE_RESISTORS 0x02
#define ET5904_LDO1_VOUT 0x03
#define ET5904_LDO2_VOUT 0x04
#define ET5904_LDO3_VOUT 0x05
#define ET5904_LDO4_VOUT 0x06
#define ET5904_LDO1_LDO2_SEQ 0x0a
#define ET5904_LDO3_LDO4_SEQ 0x0b
#define ET5904_LDO_EN 0x0e
#define ET5904_SEQ_STATUS 0x0f


/* ET5904_LDO1_VSEL ~ ET5904_LDO4_VSEL =
 * 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09
 */
#define  ET5904_LDO1_VSEL                      ET5904_LDO1_VOUT
#define  ET5904_LDO2_VSEL                      ET5904_LDO2_VOUT
#define  ET5904_LDO3_VSEL                      ET5904_LDO3_VOUT
#define  ET5904_LDO4_VSEL                      ET5904_LDO4_VOUT


#define  ET5904_VSEL_SHIFT                     0
#define  ET5904_VSEL_MASK                      (0xff << 0)

#endif /* __ET5904_REGISTERS_H__ */
