// SPDX-License-Identifier: GPL-2.0+
/*
 * dio8018, Multi-Output Regulators
 * Copyright (C) 2023  Motorola Mobility LLC,
 *
 * Author: ChengLong, Motorola Mobility LLC,
 */

#ifndef __DIO8018_REGISTERS_H__
#define __DIO8018_REGISTERS_H__

/* Registers */
#define DIO8018_REG_NUM (DIO8018_MINT3 - DIO8018_PRODUCT_ID + 1)

#define DIO8018_PRODUCT_ID 0x00
#define DIO8018_CHIP_REV   0x01
#define DIO8018_IOUT       0x02
#define DIO8018_LDO_EN     0x03
#define DIO8018_LDO1_VOUT  0x04
#define DIO8018_LDO2_VOUT  0x05
#define DIO8018_LDO3_VOUT  0x06
#define DIO8018_LDO4_VOUT  0x07
#define DIO8018_LDO5_VOUT  0x08
#define DIO8018_LDO6_VOUT  0x09
#define DIO8018_LDO7_VOUT  0x0A
#define DIO8018_LDO12_SEQ  0x0B
#define DIO8018_LDO34_SEQ  0x0C
#define DIO8018_LDO56_SEQ  0x0D
#define DIO8018_LDO7_SEQ   0x0E
#define DIO8018_SEQUENCING 0x0F
#define DIO8018_DISCHARGE  0x10
#define DIO8018_RESET      0x11
#define DIO8018_I2C_ADDR   0x12
#define DIO8018_RESERVED1  0x13
#define DIO8018_RESERVED2  0x14
#define DIO8018_INTERRUPT1 0x15
#define DIO8018_INTERRUPT2 0x16
#define DIO8018_INTERRUPT3 0x17
#define DIO8018_STATUS1    0x18
#define DIO8018_STATUS2    0x19
#define DIO8018_STATUS3    0x1A
#define DIO8018_STATUS4    0x1B
#define DIO8018_MINT1      0x1C
#define DIO8018_MINT2      0x1D
#define DIO8018_MINT3      0x1E

#define DIO8018_CHIP_ID    0x04
#define WL28681C_CHIP_ID   0x0D

/* DIO8018_LDO1_VSEL ~ DIO8018_LDO7_VSEL =
 * 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09
 */
#define  DIO8018_LDO1_VSEL                      DIO8018_LDO1_VOUT
#define  DIO8018_LDO2_VSEL                      DIO8018_LDO2_VOUT
#define  DIO8018_LDO3_VSEL                      DIO8018_LDO3_VOUT
#define  DIO8018_LDO4_VSEL                      DIO8018_LDO4_VOUT
#define  DIO8018_LDO5_VSEL                      DIO8018_LDO5_VOUT
#define  DIO8018_LDO6_VSEL                      DIO8018_LDO6_VOUT
#define  DIO8018_LDO7_VSEL                      DIO8018_LDO7_VOUT

#define  DIO8018_VSEL_SHIFT                     0
#define  DIO8018_VSEL_MASK                      (0xff << 0)

#define LDO1_2_MIN_UV                            (504000)  //0.504V
#define LDO1_2_MAX_UV                            (1504000) //1.504V
#define LDO1_2_STEP_UV                           (8000)    //0.008V
#define LDO3_7_MIN_UV                           (1500000) //1.5V
#define LDO3_7_MAX_UV                           (3412000) //3.412V
#define LDO3_7_STEP_UV                          (8000)    //0.008V

#endif /* __DIO8018_REGISTERS_H__ */
