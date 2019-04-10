/*******************************************************************************
  BT Timer Interface

  Company:
    Microchip Technology Inc.

  File Name:
    bttimer.h

  Summary:
    Contains the BT Timer Interface specific defintions and function prototypes.

  Description:
    This file contains the BT Timer Interface specific defintions and function
    prototypes.
*******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef __BTTIMER_H_INCLUDED__
#define __BTTIMER_H_INCLUDED__
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
// Configuration file that defines timer IDs
#define TOTAL_TIMERS  (BT_TIMER_MAX + BTTIMER_MAX_TIMERS + BTAPP_MAX_TIMERS)

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
typedef struct _TimerData
{
    bt_ulong              duration;
    bt_ulong              startTime;
    bt_timer_callback_fp  callback;
} TimerData;
typedef void (*bttimer_TimerCallback)(void);

// *****************************************************************************
// *****************************************************************************
// Section: Function Prototypes
// *****************************************************************************
// *****************************************************************************
void bttimer_init(void);
void bttimer_onSystemTick(void);
void bttimer_setTimer(bt_uint timerId, bt_ulong milliseconds, bttimer_TimerCallback callback);
void bttimer_clearTimer(bt_uint timerId);
void bttask_pal_initTimer(void);
void bttask_pal_handleTimerSignal(void);
void btapp_setTimer(BTAPP_TIMER_ID timerId, bt_ulong milliseconds, BTAPP_TIMER_CALLBACK callback);
void btapp_clearTimer(BTAPP_TIMER_ID timerId);
void bt_oem_timer_set(bt_uint timerId, bt_ulong milliseconds, bt_timer_callback_fp callback);
void bt_oem_timer_clear(bt_uint timerId);

// *****************************************************************************
// *****************************************************************************
// Section: extern declarations
// *****************************************************************************
// *****************************************************************************
#endif // __BTTIMER_H_INCLUDED__
/*******************************************************************************
 End of File
*/
