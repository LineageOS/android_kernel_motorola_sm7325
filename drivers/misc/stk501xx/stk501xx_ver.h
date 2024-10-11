/**
 *  * @file stk501xx_ver.h
 *  *
 *  * Copyright (c) 2022, Sensortek.
 *  * All rights reserved.
 *  *
 *******************************************************************************/

/*==============================================================================
 *
 *     Change Log:
 *
 *         EDIT HISTORY FOR FILE
 *
 *         MAY 19 2022 STK - 1.0.0
 *         - First draft version
 *         FEB 23 2023 STK - 1.1.0
 *         - Add set dist(threahold using) function.
 *         MAR 03 2023 STK - 1.2.0
 *         - Merge stk501xx_drv_i2c.c to stk501xx_qualcomm.c
 *         - Edit some STK_SPREADTRUM define region
 *         Mar 16 2023 STK - 1.3.0
 *         - Edit symbol int, u16, u32...etc to intxx_t
 *         Mar 21 2023 STK - 1.4.0
 *         - Add TWS using initail setting
 *         Mar 23 2023 STK - 1.5.0
 *         - Modify soft reset flow
 *         - Smothing cadc setting will be check chip index
 *         Apr 21 2023 STK - 1.6.0
 *         - Initial add AFE power timing control
 *         - Add force read STK_ADDR_TRIGGER_CMD
 *         May 10 2023 STK - 1.7.0
 *         - Delta threshold can be set by decimal value,
 *           driver will auto devide gain and sqrt.
 *         JUN  2 2023 STK - 1.8.0
 *         - change strcpy and print args, remove STK_ABS define in driver
 *         JUN 12 2023 STK - 1.9.0
 *         - Modify MTK /Qualcomm send command function
 *         - Enhance dual channel of temperature compensation
 *         JUN 14 2023 STK - 1.10.0
 *         - edit print function and remove print define
 *         JUN 29 2023 STK - 1.11.0
 *         - Add Phase 6 7 threshold at default.
 *         Jul 3 2023 STK - 1.12.0
 *         - Correct read data /delta format
 *         Jul 6 2023 STK - 1.13.0
 *         - Support multiple slider key using.
 *         - Fix compile warning.
 *         Jul 12 2023 STK - 1.14.0
 *         - New IC hardward support i2c collision,
 *           remove i2c collision software solution.
 *         Jul 27 2023 STK - 1.15.0
 *         - Update stk_sqrt function.
 *         - Add start up thd mechanism.
 *         Aug 2 2023 STK - 1.16.0
 *         - Fix start up mechanism operation sign.
 *         - Fix sw reset flow.
 *         Aug  8 2023 STK - 1.17.0
 *         - Move Common define to common_define.h
 *         Sep  6 2023 STK - 1.18.0
 *         - Fix softreset delay time(5~6ms)
 *         Sep  11 2023 STK - 1.19.0
 *         - Change temperature calibration method.
 *         - Remove no use pasue mode.
 *         - Parameterize stk_alg_work_queue waiting time.
 *         Nov  24 2023 STK - 2.0.0
 *         - Fix startup bug
 *         Dec  21 2023 STK - 2.1.0
 *         - Enhance temperature compensation flow.
 *         Dec  25 2023 STK - 2.2.0
 *         - Remove repeat flow and unused parameter.
 *         Jan  9 2024 STK - 2.3.0
 *         - change callback argument
 *         Jan 22 2024 STK - 2.4.0
 *         - Update fix cadc in startup stage.
 *============================================================================*/

#ifndef _STK501XX_VER_H
#define _STK501XX_VER_H

// 32-bit version number represented as major[31:16].minor[15:8].rev[7:0]
#define STK501XX_MAJOR        2
#define STK501XX_MINOR        4
#define STK501XX_REV          0
#define VERSION_STK501XX  ((STK501XX_MAJOR<<16) | (STK501XX_MINOR<<8) | STK501XX_REV)

#endif //_STK501XX_VER_H
