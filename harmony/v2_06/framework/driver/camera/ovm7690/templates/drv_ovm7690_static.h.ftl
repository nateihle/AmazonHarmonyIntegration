/*******************************************************************************
 OVM7690 Camera Driver Interface File

  File Name:
    drv_ovm7690_static.h

  Summary:
    OVM7690 camera driver interface declarations for the
    static single instance driver.

  Description:
    The OVM7690 camera device driver provides a simple interface to manage
    the OVM7690 camera cube interfacing to Microchip microcontrollers. This
    file defines the interface declarations for the OVM7690 driver.
 ******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2015 released Microchip Technology Inc.  All rights reserved.

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
 ******************************************************************************/
// DOM-IGNORE-END

#ifndef _DRV_OVM7690_STATIC_H
#define _DRV_OVM7690_STATIC_H

//DOM-IGNORE-END

#include "driver/camera/drv_camera.h"

// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for the static driver
// *****************************************************************************
// *****************************************************************************
void DRV_OVM7690_Initialize(void * frame);

void DRV_OVM7690_Tasks(void);

void DRV_OVM7690_Open(uint16_t hSize, uint16_t vSize, uint16_t vStart,
                     uint16_t hStart, bool subSample, bool greyScale);

bool _DRV_CAMERA_OVM7690_RegisterSet ( uint8_t regIndex, uint8_t regValue );
#endif // #ifndef _DRV_OVM7690_STATIC_H

/*******************************************************************************
 End of File
*/
