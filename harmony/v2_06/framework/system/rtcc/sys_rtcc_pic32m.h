/*******************************************************************************
  Timer Device Driver Interface Definition

  Company:
    Microchip Technology Inc.

  File Name:
    sys_rtcc_pic32m.h.h
  Summary:
    Timer device driver definitions header file.

  Description:
    This header file contains the definitions of the
    data types and constants that make up the interface to the Timer device
    driver.
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

#ifndef _RTCC_DEFINITIONS_PIC32M_H
#define _RTCC_DEFINITIONS_PIC32M_H

#include "peripheral/rtcc/plib_rtcc.h"
#include "peripheral/int/plib_int.h"
#include "system/rtcc/sys_rtcc.h"
#define RTCC_PLIB_ID    RTCC_ID_0
// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
typedef struct _SYS_RTCC_OBJ_STRUCT
{
    /* - Object status */
    SYS_RTCC_STATUS status;
	/* - Call back function for RTCC. Happens at ALARMED */
    SYS_RTCC_ALARM_CALLBACK  callback;
	/* - Client data (Event Context) that will be passed to callback */
    uintptr_t context;
	/* - RTCC Alarm handle */
    SYS_RTCC_ALARM_HANDLE handle;
    /* - Interrupt source */
    INT_SOURCE interruptSource;
} SYS_RTCC_OBJECT;
//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END


#endif // #ifndef _DRV_TMR_H

/*******************************************************************************
 End of File
*/
