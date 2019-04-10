/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    calibrate.h

  Summary:
    This header file provides prototypes and definitions for resistive calibration.

  Description:
    This header file provides function prototypes and data type definitions for
    the calibration activities.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2018 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _CALIBRATE_H
#define _CALIBRATE_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

typedef enum
{
	/* Application's state machine's initial state. */
	APP_CAL_STATE_INIT=0,
    APP_CAL_STATE_IDLE,
    APP_CAL_WAIT_FOR_PRESS,
    APP_CAL_WAIT_FOR_RELEASE,
    APP_CAL_LOAD,
    APP_CAL_SAVE,
    APP_CAL_DONE,

	/* TODO: Define states used by the application state machine. */

} APP_CALIBRATION_STATES;

typedef struct
{
    /* The application's current state */
    APP_CALIBRATION_STATES state;

    /* TODO: Define any additional data used by the application. */

} APP_CALIBRATION_DATA;

void APP_CalibrationInitialize( void );
bool APP_Calibration_Tasks( void );
bool APP_CalibrationLoadFromUser(void );
bool APP_CalibrationLoadFromFlash(void);
bool APP_CalibrationLoadFromSram(void);


#endif /* _CALIBRATE_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

