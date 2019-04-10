/*******************************************************************************
  Motor Control PWM (MCPWM) Driver Interface Declarations for Static Single Instance Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_mcpwm.h

  Summary:
    MCPWM driver interface declarations for the static single instance driver.

  Description:
    The MCPWM device driver provides a simple interface to manage the MCPWM module on 
    Microchip microcontrollers. This file defines the interface Declarations for the 
    MCPWM driver.
    
  Remarks:
    Static interfaces incorporate the driver instance number within the names
    of the routines, eliminating the need for an object ID or object handle.
    
    Static single-open interfaces also eliminate the need for the open handle.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

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
#ifndef _DRV_MCPWM_H
#define _DRV_MCPWM_H

#include "peripheral/mcpwm/plib_mcpwm.h"
#include "peripheral/int/plib_int.h"
// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for the Static Driver
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    void DRV_MCPWM_Initialize(void)

  Summary:
    Initializes the MCPWM instance for the specified driver index.
    <p><b>Implementation:</b> Static</p>	

  Description:
    This routine initializes the MCPWM Driver instance for the specified driver
    instance, making it ready for clients to use it. The initialization
    routine is specified by the MHC parameters.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    This routine must be called before any other MCPWM routine is called.
    This routine should only be called once during system initialization. 
*/
void DRV_MCPWM_Initialize(void);

// *****************************************************************************
/* Function:
    void DRV_MCPWM_Enable(void)

  Summary:
    Enables the MCPWM instance for the specified driver index. 
    <p><b>Implementation:</b> Static</p>	

  Description:
    This routine enables the MCPWM Driver instance for the specified driver
    instance, making it ready for clients to use it. The enable
    routine is specified by the MHC parameters.

  Precondition:
    DRV_MCPWM_Initialize has been called.

  Parameters:
    None.

  Returns:
    None.

  Remarks:

*/
void DRV_MCPWM_Enable(void);

// *****************************************************************************
/* Function:
     void DRV_MCPWM_Disable(void)

  Summary:
    Disables the MCPWM instance for the specified driver index.
    <p><b>Implementation:</b> Static</p>	

  Description:
    This routine disables the MCPWM Driver instance for the specified driver
    instance.

  Precondition:
    DRV_MCPWM_Initialize has been called.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
 
*/
void DRV_MCPWM_Disable(void);
#endif
/*******************************************************************************
 End of File
*/
