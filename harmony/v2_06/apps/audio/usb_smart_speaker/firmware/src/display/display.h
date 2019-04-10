
/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    display.h

  Summary:
    This header file provides prototypes and definitions for the display
    task function for the data_basic application.

*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _DISPLAY_H
#define _DISPLAY_H

#include "bluetooth/cdbt/bt/bt_std.h"
#include "btapp_device.h"

//// DOM-IGNORE-BEGIN
#ifdef __cplusplus
extern "C" {
#endif
// DOM-IGNORE-END

//******************************************************************************
// for BT display
//******************************************************************************
typedef enum
{
    NotVisable, //gray icon
    NotPaired_NotConnected,// white icon
    Paired_NotConnected, //white icon with triangle
    Paired_Connected //blue icon with triangle

} BT_Status;

typedef struct 
{
    int DisplayUpdate;
    BT_Status BlueTooth_Status;
    int VLED_Update;
    int VLED1;
    int VLED2;
    int VLED3;
    int VLED4;
    int VLED5;
    int VLED_R;
    int VLED_G;
    int VLED_B;
    int BTMACFLG;
    char *BTMACADD;
    int RXBUFFER_POINTER;
    int RXBUFFER_LENGTH;
    char *BTDEMONAME;
    char RXBUFFER_DATA[BUFFER_SIZE];
    int KNOWN_COMMAND;
    int PROCESS_TEXT;
    int DISPLAY_ALL;
    int BTPORT;
    int BTPORTFLAG;
    char dTestLine1;
    char dTestLine2;
    char dTestLine3;
} _BT_DISPLAY_STATS;

//******************************************************************************
// display API()
//
// display_init():
//
// display_task():
//   Updates the display when BT_DISPLAY_STATS.DisplayUpdate is True.
//
//   1. BlueTooth_Status:  
//         NotVisable, NotPaired_NotConnected, Paired_NotConnected
//         Pared_Connected.
//   2. VLED_Update:
//         VLED_<R,G,B>
//   3. BTMACFLG:
//         BTDEMONAME, BTMACADD
//   4  VLED<1,2,3,4,5)
//
//   Update the RX Text Display when BT_DISPLAY_STATS.DISPLAY_ALL is true.
//   --> BT_DISPLAY_STATS.PROCESS_TEXT true.
//
//   Update the connected icon BT_DISPLAY_STATS.BTPORTFLAG is true.
//
//******************************************************************************
void display_init(_BT_DISPLAY_STATS * BT_DISPLAY_STATS);
void display_tasks(_BT_DISPLAY_STATS * BT_DISPLAY_STATS);

extern _BT_DISPLAY_STATS BT_DISPLAY_STATS;

#endif /* _APP_H */
#ifdef __cplusplus
}
#endif

/*******************************************************************************
 End of File
 */

