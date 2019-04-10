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
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

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

#define APP_QUEUING_DEPTH            USB_DEVICE_AUDIO_QUEUE_DEPTH_COMBINED
//NOTE: Cache coherency and 16 byte alignment required for MZ processor,
//      -->as the cache page line size on PIC32MZ is 16 bytes.
//      You don’t want to run into an issue where linker allocates the data 
//      structure in the same page as another data structure and then a line 
//      flush causes coherency issues.
#define APP_MAKE_BUFFER_DMA_READY  __attribute__((coherent)) __attribute__((aligned(16))) 

#define APP_LED1_ON()          BSP_LED_1On() 
#define APP_LED1_OFF()         BSP_LED_1Off()
#define APP_LED1_TOGGLE()      BSP_LED_1Toggle() 
#define APP_LED1_STATE()       BSP_LED_1StateGet()

#define APP_LED2_ON()          BSP_LED_2On() 
#define APP_LED2_OFF()         BSP_LED_2Off()
#define APP_LED2_TOGGLE()      BSP_LED_2Toggle() 
#define APP_LED2_STATE()       BSP_LED_2StateGet()

#define APP_LED3_ON()          BSP_LED_3On() 
#define APP_LED3_OFF()         BSP_LED_3Off()
#define APP_LED3_TOGGLE()      BSP_LED_3Toggle() 
#define APP_LED3_STATE()       BSP_LED_3StateGet()

#define APP_LED4_ON()          BSP_LED_D6On() 
#define APP_LED4_OFF()         BSP_LED_D6Off()
#define APP_LED4_TOGGLE()      BSP_LED_D6Toggle() 
#define APP_LED4_STATE()       BSP_LED_D6StateGet()

#define APP_LED5_ON()          BSP_LED_D7On() 
#define APP_LED5_OFF()         BSP_LED_D7Off()
#define APP_LED5_TOGGLE()      BSP_LED_D7Toggle() 
#define APP_LED5_STATE()       BSP_LED_D7StateGet()

#define APP_USB_SWITCH_1       BSP_SWITCH_1
   
//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif //_APP_CONFIG_H

/*******************************************************************************
 End of File
 */
