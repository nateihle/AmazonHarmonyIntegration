/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app_config.h

  Summary:
    This header file provides prototypes and definitions derived from the
    system_config.h and should be included in the app.h in place of 
    system_config.h

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
//DOM-IGNORE-END

#ifndef _APP_CONFIG_H
#define _APP_CONFIG_H

#include "system_config.h"

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
extern "C" {
#endif
//DOM-IGNORE-END

// App common interface abstraction based on resouces defined in hardware specific
// system_config.h
#define APP_QUEUING_DEPTH            USB_DEVICE_AUDIO_QUEUE_DEPTH_COMBINED

//Align to Cache-Line (16byte) - should pad the last buffer cache line also
#define APP_MAKE_BUFFER_DMA_READY    \
             __attribute__((coherent)) __attribute__((aligned(16)))

#define APP_LED1_ON()                BSP_LEDOn(BSP_LED_9)
#define APP_LED1_OFF()               BSP_LEDOff(BSP_LED_9)
#define APP_LED1_TOGGLE()            BSP_LEDToggle(BSP_LED_9)

#define APP_LED2_ON()                BSP_LEDOn(BSP_LED_6)
#define APP_LED2_OFF()               BSP_LEDOff(BSP_LED_6)
#define APP_LED2_TOGGLE()            BSP_LEDToggle(BSP_LED_6)

#define APP_LED3_ON()                BSP_LEDOn(BSP_LED_7)
#define APP_LED3_OFF()               BSP_LEDOff(BSP_LED_7)
#define APP_LED3_TOGGLE()            BSP_LEDToggle(BSP_LED_7)

#define APP_LED4_ON()                BSP_LEDOn(BSP_LED_8)
#define APP_LED4_OFF()               BSP_LEDOff(BSP_LED_8)
#define APP_LED4_TOGGLE()            BSP_LEDToggle(BSP_LED_8)

#define APP_LED5_ON()                BSP_LEDOn(BSP_LED_5)
#define APP_LED5_OFF()               BSP_LEDOff(BSP_LED_5)
#define APP_LED5_TOGGLE()            BSP_LEDToggle(BSP_LED_5)


//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif //_APP_CONFIG_H

/*******************************************************************************
 End of File
 */
