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
 *         Mar 11 2024 STK - 2.5.0
 *         - Update fix cadc mechanism.
 *         Mar 14 2024 STK - 2.6.0
 *         - Move stk_set_thd function to register init.
 *============================================================================*/

#ifndef _STK501XX_VER_H
#define _STK501XX_VER_H

// 32-bit version number represented as major[31:16].minor[15:8].rev[7:0]
#define STK501XX_MAJOR        2
#define STK501XX_MINOR        6
#define STK501XX_REV          0
#define VERSION_STK501XX  ((STK501XX_MAJOR<<16) | (STK501XX_MINOR<<8) | STK501XX_REV)

#endif //_STK501XX_VER_H
