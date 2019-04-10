/*******************************************************************************
  MXT336T Touch Driver Configuration Template

  Company:    
    Microchip Technology Inc.

  File Name:   
    drv_MXT336T_config_template.h
  
  Summary:    
    MXT336T Touch Driver configuration template.

  Description:
    This header file contains the build-time configuration selections for the
    MXT336T Touch Driver.  This is the template file which give all possible
    configurations that can be made. This file should not be included in 
    any project.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014-2015 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _DRV_MXT336T_CONFIG_TEMPLATE_H
#define _DRV_MXT336T_CONFIG_TEMPLATE_H

//#error "This is a configuration template file.  Do not include it directly."

// *****************************************************************************
// *****************************************************************************
// Section: MXT336T Core Functionality Configuration Macros
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
/* MXT336T hardware instance configuration

  Summary:
    Sets up the maximum number of hardware instances that can be supported.

  Description:
    This macro sets up the maximum number of hardware instances that can be 
    supported.

  Remarks:
    None.
*/

#define DRV_MXT336T_INSTANCES_NUMBER                 1


//**********************************************************************
/* MXT336T maximum number of clients

  Summary:
    Selects the maximum number of clients.

  Description:
    This macro selects the maximum number of clients.
    
    This definition selected the maximum number of clients that the MXT336T
    driver can support at run time.

  Remarks:
    None.                                                              
 */

#define DRV_MXT336T_CLIENTS_NUMBER                   5


// *****************************************************************************
/* MXT336T Static Index Selection

  Summary:
    MXT336T static index selection.

  Description:
    This macro specifies the static index selection for the driver object reference.

  Remarks:
    This index is required to make a reference to the driver object.
*/

#define DRV_MXT336T_INDEX                            DRV_MXT336T_INDEX_0


// *****************************************************************************
/* MXT336T Interrupt And Polled Mode Operation Control

  Summary:
    Controls operation of the driver in the interrupt or polled mode.

  Description:
    This macro controls the operation of the driver in the interrupt
    mode of operation. The possible values of this macro are:
    - true  - Select if interrupt mode of MXT336T operation is desired
    - false - Select if polling mode of MXT336T operation is desired
    Not defining this option to true or false will result in a build error.

  Remarks:
    None.
*/

#define DRV_MXT336T_INTERRUPT_MODE                   false


// *****************************************************************************
// *****************************************************************************
// Section: Initialization Overrides
// *****************************************************************************
// *****************************************************************************
/* This section defines the initialization overrides */

// *****************************************************************************
/* MXT336T Calibration Inset

  Summary:
    Defines the calibration inset.

  Description:
    This macro defines the calibration inset.

  Remarks:
    None.
*/

#define DRV_MXT336T_CALIBRATION_INSET                25


//**************************************************************************
/* MXT336T Touch Diameter

  Summary:
    Defines the touch diameter.

  Description:
    This macro defines the touch diameter

  Remarks:
    None.
*/

#define DRV_MXT336T_TOUCH_DIAMETER                   10


//**************************************************************************
/* MXT336T Sample Points

  Summary:
    Define the sample points.

  Description:
    MXT336T sample points

  Remarks:
    None.
*/

#define DRV_MXT336T_SAMPLE_POINTS                    4


// *****************************************************************************
/* MXT336T Calibration Delay

  Summary:
    Defines the calibration delay.

  Description:
    This macro enables the delay between calibration touch points.

  Remarks:
    None.
*/

#define DRV_MXT336T_CALIBRATION_DELAY                300

#endif // _DRV_MXT336T_CONFIG_TEMPLATE_H

/*******************************************************************************
 End of File
*/