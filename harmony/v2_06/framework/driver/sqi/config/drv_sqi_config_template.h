/******************************************************************************
  SQI Driver Configuration Template Header file.

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sqi_config_template.h

  Summary:
    SQI driver configuration definitions.

  Description:
    This template file describes all the mandatory and optional configuration
    macros that are needed for building the SQI driver. Do not include this
    file in source code.
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

#ifndef _DRV_SQI_CONFIG_TEMPLATE_H
#define _DRV_SQI_CONFIG_TEMPLATE_H

#error "This is a configuration template file.  Do not include it directly."

// *****************************************************************************
/* SQI Driver instance configuration

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

#define DRV_SQI_INSTANCES_NUMBER                        			1

// *****************************************************************************
/* SQI maximum number of clients

  Summary:
    Selects the maximum number of clients

  Description:
    This definition selects the maximum number of clients that the SQI driver
    can supported at run time. This constant defines the total number of SQI
    driver clients that will be available to all instances of the SQI driver.

  Remarks:
    This macro is mandatory when building the driver for dynamic operation.
*/

#define DRV_SQI_CLIENTS_NUMBER                          			1

// *****************************************************************************
/* SQI Driver maximum number of buffer objects

  Summary:
    Selects the maximum number of buffer objects

  Description:
    This definition selects the maximum number of buffer objects. This
    indirectly also specifies the queue depth. The SQI Driver can queue up
    DRV_SQI_BUFFER_OBJECT_NUMBER of read/write/erase requests before return a
    DRV_SQI_BUFFER_HANDLE_INVALID due to the queue being full. Buffer objects
    are shared by all instances of the driver. Increasing this number increases
    the RAM requirement of the driver.

  Remarks:
    This macro is mandatory when building the driver for dynamic operation.
*/

#define DRV_SQI_BUFFER_OBJECT_NUMBER                          		        5

// *****************************************************************************
/* SQI Driver maximum number DMA Buffer Descriptors

  Summary:
    Selects the maximum number of DMA Buffer descriptors to be used by the
    driver.

  Description:
    This definition selects the maximum number of DMA buffer descriptor
    objects.  The SQI Driver can queue up to
    DRV_SQI_DMA_BUFFER_DESCRIPTORS_NUMBER of transactions to be processed by
    the hardware. DMA buffer desired are shared by all instances of the driver.
    Increasing this number increases the RAM requirement of the driver.

  Remarks:
    This macro is mandatory when building the driver for dynamic operation.
*/

#define DRV_SQI_DMA_BUFFER_DESCRIPTORS_NUMBER                                   4

// *****************************************************************************
/* SQI interrupt and polled mode operation control

  Summary:
    Macro specifies operation of the driver to be in the interrupt mode
    or polled mode

  Description:
    This macro specifies operation of the driver to be in the interrupt mode
    or polled mode

    - true  - Select if interrupt mode of SQI operation is desired
    - false - Select if polling mode of SQI operation is desired

    Not defining this option to true or false will result in build error.

  Remarks:
    This macro is mandatory when building the driver for dynamic operation.
*/

#define DRV_SQI_INTERRUPT_MODE                          			true

#endif // #ifndef _DRV_SQI_CONFIG_TEMPLATE_H

/*******************************************************************************
 End of File
*/

