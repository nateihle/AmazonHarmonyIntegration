/*******************************************************************************
  PMP Driver Configuration Definitions for the Template Version

  Company:
    Microchip Technology Inc.

  File Name:
    drv_pmp_config_template.h

  Summary:
    PMP driver configuration definitions template

  Description:
    These definitions statically define the driver's mode of operation.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _DRV_PMP_CONFIG_TEMPLATE_H
#define _DRV_PMP_CONFIG_TEMPLATE_H


// #error "This is a configuration template file.  Do not include it directly."

// *****************************************************************************
/* PMP hardware instance configuration

  Summary:
    Selects the maximum number of hardware instances that can be supported by
    the dynamic driver.

  Description:
    This definition selects the maximum number of hardware instances that can be 
    supported by the dynamic driver.

  Remarks:
    None.
*/

#define DRV_PMP_INSTANCES_NUMBER                        1


// *****************************************************************************
/* PMP maximum number of clients

  Summary:
    Selects the maximum number of clients.

  Description:
    This definition select the maximum number of clients that the PMP driver can
    support at run time.

  Remarks:
    None.

*/

#define DRV_PMP_CLIENTS_NUMBER                          2


// *****************************************************************************
/* PMP static index selection

  Summary:
    PMP static index selection.

  Description:
    This definition selects the PMP static index for the driver object reference.

  Remarks:
    This index is required to make a reference to the driver object
*/

// #define DRV_PMP_INDEX                                   DRV_PMP_INDEX_0


// *****************************************************************************
/* PMP queue size

  Summary:
    PMP queue size for different instances.

  Description:
    The PMP queue size for a driver instances should be placed here. If more than 
    one driver instance of PMP is present, then all takes the same queue size.

  Remarks:
    All the transfers (Read/Write) first gets queued and gets completed sequentially 
    when Task API is called in a loop. Therefore, the minimum value of this index 
    should be 1.
*/
#define DRV_PMP_QUEUE_SIZE				8

/* for EPMP */
//#define DRV_PMP_NO_OF_CS                                2

#endif // #ifndef _DRV_PMP_CONFIG_TEMPLATE_H

/*******************************************************************************
 End of File
*/

