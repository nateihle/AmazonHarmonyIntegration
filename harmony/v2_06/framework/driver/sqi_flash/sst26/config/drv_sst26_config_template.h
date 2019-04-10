/******************************************************************************
  SST26 Driver Configuration Template Header file.

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sst26_config_template.h

  Summary:
    SST26 driver configuration definitions.

  Description:
    This template file describes all the mandatory and optional configuration
    macros that are needed for building the SST26 driver. Do not include this file
    in source code.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _DRV_SST26_CONFIG_TEMPLATE_H
#define _DRV_SST26_CONFIG_TEMPLATE_H

#error "This is a configuration template file.  Do not include it directly."

// *****************************************************************************
/* SST26 Driver instance configuration

  Summary:
    Selects the maximum number of Driver instances that can be supported by
    the dynamic driver.

  Description:
    This definition selects the maximum number of Driver instances that can be
    supported by the dynamic driver. In case of this driver, multiple instances
    of the driver could use the same hardware instance.

  Remarks:
    This macro is mandatory when building the driver for dynamic operation.
*/

#define DRV_SST26_INSTANCES_NUMBER                        			1

// *****************************************************************************
/* SST26 maximum number of clients

  Summary:
    Selects the maximum number of clients

  Description:
    This definition selects the maximum number of clients that the SST26 driver
    can supported at run time. This constant defines the total number of SST26
    driver clients that will be available to all instances of the SST26 driver.

  Remarks:
    This macro is mandatory when building the driver for dynamic operation.
*/

#define DRV_SST26_CLIENTS_NUMBER                          			1

// *****************************************************************************
/* SST26 Driver maximum number of buffer objects

  Summary:
    Selects the maximum number of buffer objects

  Description:
    This definition selects the maximum number of buffer objects. This
    indirectly also specifies the queue depth. The SST26 Driver can queue up
    DRV_SST26_BUFFER_OBJECT_NUMBER of read/write/erase requests before return a
    DRV_SST26_BUFFER_HANDLE_INVALID due to the queue being full. Buffer objects
    are shared by all instances of the driver. Increasing this number increases
    the RAM requirement of the driver.

  Remarks:
    This macro is mandatory when building the driver for dynamic operation.
*/

#define DRV_SST26_BUFFER_OBJECT_NUMBER                          		5

// *****************************************************************************
/* SST26 Driver Register with File System

  Summary:
    Register to use with the File system

  Description:
    Specifying this macro enables the SST26 driver to register its services with
    the SYS FS.

  Remarks:
    This macro is optional and should be specified only if the SST26 driver is
    to be used with the File System.
*/

#define DRV_SST26_SYS_FS_REGISTER

#endif // #ifndef _DRV_SST26_CONFIG_TEMPLATE_H

/*******************************************************************************
 End of File
*/

