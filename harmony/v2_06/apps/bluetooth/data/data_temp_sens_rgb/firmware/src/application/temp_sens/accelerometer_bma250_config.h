/*******************************************************************************
  Accelerometer configuration Interface

  Company:
    Microchip Technology Inc.

  File Name:
    accelerometer_bma250_config.h

  Summary:
    Contains the Accelerometer configuration Interface specific defintions and
    function prototypes.

  Description:
    This file contains the Accelerometer configuration Interface specific
    defintions and function prototypes.
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

#ifndef _ACCEL_BMA250_CONFIGURATION_H
#define _ACCEL_BMA250_CONFIGURATION_H

//// DOM-IGNORE-BEGIN
#ifdef __cplusplus
extern "C" {
#endif
// DOM-IGNORE-END

// *****************************************************************************
/* Accelerometer I2C Module Configuration.

  Summary:
    Accelerometer I2C Module configuration.

  Description:
    Communication to the Accelerometer device is through the I2C module.
    The application must select
    an I2C module.  The I2C module must be consistant for all macros.

  Remarks:
	None.
*/
#define ACCEL_BMA250_USE_I2C1
/*************************************************************************
  Summary:
    Accelerometer I2C Module Data Rate.
  Description:
    Accelerometer I2C Module Data Rate.

    The data rate that is used for communicating to the Accelerometer device.

  Remarks:
	None.
  *************************************************************************/
#define ACCEL_BMA250_BAUD          (400000)
#define ACCEL_USE_POLLING
//#define ACCEL_USE_EXTERNAL_INTERUPT_HANDLER

#ifdef __cplusplus
}
#endif

#endif /* _ACCEL_BMA250_CONFIGURATION_H */
/*******************************************************************************
 End of File
*/
