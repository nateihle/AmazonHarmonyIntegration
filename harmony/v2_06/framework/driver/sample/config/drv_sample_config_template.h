/*******************************************************************************
  Sample Driver Configuration Definitions for the template version

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sample_config_template.h

  Summary:
    Sample driver configuration definitions template.

  Description:
    These definitions statically define the driver's mode of operation.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012-2015 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _DRV_SAMPLE_CONFIG_TEMPLATE_H
#define _DRV_SAMPLE_CONFIG_TEMPLATE_H


//#error "This is a configuration template file.  Do not include it directly."


// *****************************************************************************
/* Sample hardware instance configuration

  Summary:
    Selects the maximum number of hardware instances that can be supported by
    the dynamic driver.

  Description:
    This definition selects the maximum number of hardware instances that can be
    supported by the dynamic driver. Not defining it means using a static driver.

  Remarks:
    None.
*/

#define DRV_SAMPLE_INSTANCES_NUMBER                1


// *****************************************************************************
/* Sample Maximum Number of Clients

  Summary:
    Selects the maximum number of clients.

  Description:
    This definition select the maximum number of clients that the Sample driver can
    support at run time. Not defining it means using a single client.

  Remarks:
    None.

*/

#define DRV_SAMPLE_CLIENTS_NUMBER                1


// *****************************************************************************
/* Sample Static Index Selection

  Summary:
    Sample static index selection.

  Description:
    This macro provides the sample static index selection for the driver object 
	reference.

  Remarks:
    This index is required to make a reference to the driver object.
*/

#define DRV_SAMPLE_INDEX                                DRV_SAMPLE_INDEX_2


// *****************************************************************************
/* Sample Interrupt And Polled Mode Operation Control

  Summary:
    Controls operation of the driver in Interrupt mode or Polled mode.

  Description:
    This macro controls the operation of the driver in the Interrupt
    mode of operation. The possible values of this macro are:
    - true  - Select if Interrupt mode of timer operation is desired
    - false - Select if Polling mode of timer operation is desired

    Not defining this option to true or false will result in a build error.

  Remarks:
    None.
*/

#define DRV_SAMPLE_INTERRUPT_MODE          true


// *****************************************************************************
// *****************************************************************************
// Section: Initialization Overrides
// *****************************************************************************
// *****************************************************************************
/* This section defines the initialization overrides */

// *****************************************************************************
/* Sample Peripheral ID Selection

  Summary:
    Defines an override of the peripheral ID.

  Description:
    This macro defines an override of the peripheral ID, using macros.

  Remarks:
    Some devices also support SAMPLE_ID_0
*/

#define DRV_SAMPLE_PERIPHERAL_ID                         SAMPLE_ID_1


// *****************************************************************************
/* Sample Interrupt Source

  Summary:
    Defines an override of the interrupt source in case of a static driver.

  Description:
    This macro defines an override of the interrupt source in case of a static driver.

  Remarks:
    Refer to the Interrupt Peripheral Library help document for more information 
	on the INT_SOURCE enumeration.

*/

#define DRV_SAMPLE_INTERRUPT_SOURCE                INT_SOURCE_TIMER_1


// *****************************************************************************
/* Sample power state configuration

  Summary:
    Defines an override of the power state of the Sample Driver.

  Description:
    This macro defines an override of the power state of the Sample Driver.

  Remarks:
    This feature may not be available in the device or the Sample module
    selected. 
*/

#define DRV_SAMPLE_POWER_STATE                 SYS_MODULE_POWER_IDLE_STOP


#endif // #ifndef _DRV_SAMPLE_CONFIG_TEMPLATE_H

/*******************************************************************************
 End of File
*/