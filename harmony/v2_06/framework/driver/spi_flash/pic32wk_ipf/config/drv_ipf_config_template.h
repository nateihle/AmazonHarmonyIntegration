/*******************************************************************************
 IPF Driver Configuration Template

  Company:
    Microchip Technology Inc.

  File Name:
    drv_ipf_config_template.h

  Summary:
    IPF Driver Configuration Template.

  Description:
    These file provides the list of all the configurations that can be used with
    the driver. This file should not be included in the driver.

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

#ifndef _DRV_IPF_CONFIG_TEMPLATE_H
#define _DRV_IPF_CONFIG_TEMPLATE_H


//DOM-IGNORE-BEGIN
#error "This is a configuration template file.  Do not include it directly."
//DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Core Functionality Configuration Options
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* IPF mode

  Summary:
    Determines whether the driver is implemented as static or dynamic

  Description:
    Determines whether the driver is implemented as static or dynamic. Static 
    drivers control the peripheral directly with peripheral library routines.

  Remarks:
    None.
*/

#define DRV_IPF_MODE                              DYNAMIC

// *****************************************************************************
/* IPF driver objects configuration

  Summary:
    Sets up the maximum number of hardware instances that can be supported

  Description:
    Sets up the maximum number of hardware instances that can be supported.
    It is recommended that this number be set exactly equal to the number of
    IPF modules that are needed by the application. Hardware Instance
    support consumes RAM memory space. 

  Remarks:
    As PIC32WK has only 1 instance of IPF, this macro is always set to 1.
*/

#define DRV_IPF_INSTANCES_NUMBER                      1


// *****************************************************************************
/* IPF Client Count Configuration

  Summary:
    Sets up the maximum number of clients that can be connected to the hardware
    instance.

  Description:
    Sets up the maximum number of clients that can be connected to the hardware
    instance. So if IPF will be accessed by 2 clients then this number should be 2. 
    It is recommended that this be set exactly equal to the number of expected
    clients. Client support consumes RAM memory space.

  Remarks:
    None.
*/

#define DRV_IPF_CLIENTS_NUMBER                      4

#endif // #ifndef _DRV_IPF_CONFIG_TEMPLATE_H

/*******************************************************************************
 End of File
*/
