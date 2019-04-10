/*******************************************************************************
 System Interrupts File

  File Name:
    system_interrupt.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system/common/sys_common.h"
#include "pfc_app.h"
#include "mc_app.h"
#include "x2c_scope.h"
#include "system_definitions.h"

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************

void __ISR(_ADC_DATA3_VECTOR, ipl3AUTO) _IntHandlerDrvAdc_DATA3(void)
{
    
    if(PFC_APP_pfcParam.pfc_good==1)
    {
        MC_APP_MC_ADCISRTasks();
    }  
      X2CScope_Update();
    /* Clear ADC Interrupt Flag of INT_SOURCE_ADC_1_DATA3*/
   PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1_DATA3);
}
void __ISR(_ADC_DATA4_VECTOR, ipl4AUTO) _IntHandlerDrvAdc_DATA4(void)
{
    PFC_APP_pfc_control();
    
    
    
     
    /* Clear ADC Interrupt Flag of INT_SOURCE_ADC_1_DATA4*/
   PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1_DATA4);
}
     







void __ISR(_PWM1_VECTOR, ipl4AUTO) _IntHandlerDrvMCPWMChannelInstanceMCPWM_CHANNEL1(void)
{
	MC_APP_MC_FaultISRTasks();
    /*Clear PWM Channel Interrupt Flag*/
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_PWM1);
}














/*******************************************************************************
 End of File
*/
