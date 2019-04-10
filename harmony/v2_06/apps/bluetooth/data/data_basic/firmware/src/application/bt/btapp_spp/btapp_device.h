/*******************************************************************************
    BT APP Device

  Company:
    Microchip Technology Inc.

  File Name:
    btapp_device.h

  Summary:
    Contains device specific declarations of application.

  Description:
    This file contains device specific declarations of application.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
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
// DOM-IGNORE-END

#ifndef __BTAPP_DEVICE_H_INCLUDED__
#define __BTAPP_DEVICE_H_INCLUDED__
// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************
#define TEMP_CALLBACK_RATE     1000  /* The default time interval to send spp
                                         data to terminal emulator continuously */
#define RGB_LED_COLOR           255   /* Command to send RGB data */
#define ONE_TIME_TEMP_CALL      254   /* Command to send temprature to the
                                         terminal emulator once */
#define CONTINUOUS_TEMP         253   /* Command to send the temperature to the
                                         terminal emulator continuously */
#define TEMP_CALLBACK_RATE_ADDR 0xFC  /* Command to change the default time
                                         interval to send spp data to terminal
                                         emulator */
#define TEMP_TYPE_REG           251
#define SINGLE_CHAR_RED50       'r'
#define SINGLE_CHAR_GREEN50     'g'
#define SINGLE_CHAR_BLUE50      'b'
#define SINGLE_CHAR_RED100      'R'
#define SINGLE_CHAR_GREEN100    'G'
#define SINGLE_CHAR_BLUE100     'B'
#define BUFFER_SIZE             20

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
typedef enum _BTAPP_STATUS
{
    BTAPP_STATUS_IDLE,
    BTAPP_STATUS_INITIALIZED,
    BTAPP_STATUS_STARTED

} BTAPP_STATUS;

typedef enum _BTAPP_BUTTONS
{
    BTAPP_BUTTON_S1     =  1,
    BTAPP_BUTTON_S2     =  2,
    BTAPP_BUTTON_UP     =  3,
    BTAPP_BUTTON_DOWN   =  4,
    BTAPP_BUTTON_LEFT   =  5,
    BTAPP_BUTTON_RIGHT  =  6,
    BTAPP_BUTTON_SELECT =  7,
    BTAPP_BUTTON_S3     =  8,
    BTAPP_BUTTON_S4     =  9,
    BTAPP_BUTTON_S5     = 10,
    BTAPP_BUTTON_S6     = 11

} BTAPP_BUTTONS;

typedef enum _BTAPP_TIMER_ID
{
    BTAPP_TIMER_0,
    BTAPP_TIMER_1,
    BTAPP_TIMER_2,
    BTAPP_TIMER_3,
    BTAPP_TIMER_4,
    BTAPP_TIMER_5,
    BTAPP_TIMER_6,
    BTAPP_TEMPTIMER,
    BTAPP_MAX_TIMERS
} BTAPP_TIMER_ID;

typedef void (*BTAPP_TIMER_CALLBACK)(void);
// *****************************************************************************
// *****************************************************************************
// Section: Function Prototypes
// *****************************************************************************
// *****************************************************************************
void btapp_init(void);
void btapp_start(void);
BTAPP_STATUS btapp_getStatus(void);
void btapp_onButtonDown(bt_uint button, bt_uint repeatCount);
void btapp_onButtonUp(bt_uint button, bt_uint repeatCount);

// *****************************************************************************
// *****************************************************************************
// Section: extern declarations
// *****************************************************************************
// *****************************************************************************
#endif // __BTAPP_DEVICE_H_INCLUDED__
/*******************************************************************************
 End of File
*/