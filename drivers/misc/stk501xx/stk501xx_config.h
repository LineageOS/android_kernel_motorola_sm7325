#ifndef __STK501XX_CONFIG_H__
#define __STK501XX_CONFIG_H__

/*****************************************************************************
 * Global variable
 *****************************************************************************/
#define STK_INTERRUPT_MODE
//#define STK_POLLING_MODE
#define STK_SENSORS_DEV  //if can't find sensors.h or not android system, don't use this define.
//#define STK_MOVEMENT_DET       // alg for any
#ifdef STK_INTERRUPT_MODE
    //#define MCU_GESTURE            //alg for MCU gesture
#endif
#define TEMP_COMPENSATION

#ifdef MCU_GESTURE
    #define STK_STARTUP_CALI
#endif

#define STK_QUALCOMM
#ifdef STK_QUALCOMM
    #ifdef STK_SENSORS_DEV
        #include <linux/sensors.h>
    #endif // STK_SENSORS_DEV
    #undef STK_SPREADTRUM
#elif defined STK_MTK
    #undef STK_INTERRUPT_MODE
    //#undef STK_POLLING_MODE
#elif defined STK_SPREADTRUM
    #include <linux/limits.h>
    #include <linux/version.h>
    #undef STK_INTERRUPT_MODE
    #define STK_POLLING_MODE
#elif defined STK_ALLWINNER
    #undef STK_INTERRUPT_MODE
    #define STK_POLLING_MODE
#endif /* STK_QUALCOMM, STK_MTK, STK_SPREADTRUM, or STK_ALLWINNER */

#endif /* __STK501XX_CONFIG_H__ */
