/*******************************************************************************
  SQI Driver Local Data Structures

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sqi_init.h

  Summary:
    SQI driver initialization data structures

  Description:
    This file contains the SQI driver's initialization data structures.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2016 - 2017 released Microchip Technology Inc. All rights reserved.

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

#ifndef _DRV_SQI_INIT_H
#define _DRV_SQI_INIT_H

// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************

#include "driver/sqi/drv_sqi.h"
#include "peripheral/sqi/plib_sqi.h"
#include "system/int/sys_int.h"

// *****************************************************************************
/* SQI Clock divider values

  Summary:
    Enumeration identifying the clock divider values supported by the SQI
    Controller.

  Description:
    This enumeration lists the various clock divider values that the SQI
    controller supports.

  Remarks:  
    None
*/

typedef enum
{
    /* This value specifies the highest frequency of the SQI Clock. */
    DRV_SQI_CLK_DIV_1 = 0x0,

    /* Base clock divided by 2 */
    DRV_SQI_CLK_DIV_2 = 0x1,

    /* Base clock divided by 4 */
    DRV_SQI_CLK_DIV_4 = 0x2,

    /* Base clock divided by 8 */
    DRV_SQI_CLK_DIV_8 = 0x4,

    /* Base clock divided by 16 */
    DRV_SQI_CLK_DIV_16 = 0x8,

    /* Base clock divided by 32 */
    DRV_SQI_CLK_DIV_32 = 0x10,

    /* Base clock divided by 64 */
    DRV_SQI_CLK_DIV_64 = 0x20,

    /* Base clock divided by 128 */
    DRV_SQI_CLK_DIV_128 = 0x40,

    /* Base clock divided by 256 */
    DRV_SQI_CLK_DIV_256 = 0x80,

    /* Base clock divided by 512 */
    DRV_SQI_CLK_DIV_512 = 0x100,

    /* Base clock divided by 1024 */
    DRV_SQI_CLK_DIV_1024 = 0x200,

    /* Base clock divided by 2048 */
    DRV_SQI_CLK_DIV_2048 = 0x400

} DRV_SQI_CLK_DIV;


// *****************************************************************************
/* Enable SQI Device

  Summary:
    Enumeration identifying the SQI devices to be managed by the SQI
    controller.

  Description:
    The SQI Controller has two Chip Select lines, CS0 and CS1. This allows the
    controller to control upto two SQI devices. This enumeration is used to
    indicate the SQI devices to be managed by the controller.

  Remarks:  
    None
*/

typedef enum
{
    /* Enable device 0 */
    DRV_SQI_ENABLE_DEVICE_0 = 0,

    /* Enable device 1 */
    DRV_SQI_ENABLE_DEVICE_1,

    /* Enable both devices */
    DRV_SQI_ENABLE_BOTH_DEVICES

} DRV_SQI_ENABLE_DEVICE;

// *****************************************************************************
/* SQI Driver Device specific configuration information.

  Summary:
    Defines the configuration data specific to the SQI devices.

  Description:
    This data type defines the SQI device specific configuration information
    required to initialize or reinitialize the SQI driver.

  Remarks:
    None.
*/

typedef struct
{
    /* SPI Mode of operation. */
    DRV_SQI_SPI_OPERATION_MODE spiMode;

    /* Send or receive least significant bit of a byte first. */
    bool lsbFirst;

} DRV_SQI_DEVICE_CFG;


// *****************************************************************************
/* SQI Driver Initialization Data

  Summary:
    Defines the data required to initialize or reinitialize the SQI driver

  Description:
    This data type defines the data required to initialize or reinitialize the
    SQI driver.

  Remarks:
    Not all initialization features are available for all devices. Please
	refer to the specific device data sheet to determine availability.
*/

typedef struct
{
    /* Identifies SQI hardware module (PLIB-level) ID */
    SQI_MODULE_ID sqiId;

    /* Interrupt Source */
    INT_SOURCE interruptSource;

    /* Identifies the enabled devices. */
    DRV_SQI_ENABLE_DEVICE enabledDevices;

    /* Clock divider value to use. */
    DRV_SQI_CLK_DIV clockDivider;

    /* SQI Controller can handle a maximum of two SQI devices. This member
     * contains the device specific configuration information. */
    DRV_SQI_DEVICE_CFG devCfg[2];

} DRV_SQI_INIT;

#endif //#ifndef _DRV_SQI_LOCAL_H

/*******************************************************************************
 End of File
*/

