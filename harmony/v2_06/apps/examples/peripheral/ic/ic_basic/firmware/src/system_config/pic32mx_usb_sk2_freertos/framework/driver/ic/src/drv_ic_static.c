/*******************************************************************************
  IC Driver Functions for Static Single Instance Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_ic_static.c

  Summary:
    IC driver implementation for the static single instance driver.

  Description:
    The IC device driver provides a simple interface to manage the IC
    modules on Microchip microcontrollers.
    
  Remarks:
    Static interfaces incorporate the driver instance number within the names
    of the routines, eliminating the need for an object ID or object handle.
    Static single-open interfaces also eliminate the need for the open handle.
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

// *****************************************************************************
// *****************************************************************************
// Header Includes
// *****************************************************************************
// *****************************************************************************
#include "peripheral/ic/plib_ic.h"
#include "peripheral/int/plib_int.h"

// *****************************************************************************
// *****************************************************************************
// Section: Instance 0 static driver functions
// *****************************************************************************
// *****************************************************************************
void DRV_IC0_Initialize(void)
{	
    PLIB_IC_ModeSelect(IC_ID_1, IC_INPUT_CAPTURE_RISING_EDGE_MODE);
    PLIB_IC_FirstCaptureEdgeSelect(IC_ID_1, IC_EDGE_RISING);
    PLIB_IC_TimerSelect(IC_ID_1, IC_TIMER_TMR2);
    PLIB_IC_BufferSizeSelect(IC_ID_1, IC_BUFFER_SIZE_32BIT);
    PLIB_IC_EventsPerInterruptSelect(IC_ID_1, IC_INTERRUPT_ON_EVERY_CAPTURE_EVENT);   

    /* Setup Interrupt */   
    PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_INPUT_CAPTURE_1);
    PLIB_INT_VectorPrioritySet(INT_ID_0, INT_VECTOR_IC1, INT_PRIORITY_LEVEL2);
    PLIB_INT_VectorSubPrioritySet(INT_ID_0, INT_VECTOR_IC1, INT_SUBPRIORITY_LEVEL0);          
}

void DRV_IC0_Start(void)
{
   PLIB_IC_Enable(IC_ID_1);
}

void DRV_IC0_Stop(void)
{
   PLIB_IC_Disable(IC_ID_1);
}

void DRV_IC0_Open(void)
{
}

void DRV_IC0_Close(void)
{
}
uint32_t DRV_IC0_Capture32BitDataRead(void)
{
   return PLIB_IC_Buffer32BitGet(IC_ID_1);
}

uint16_t DRV_IC0_Capture16BitDataRead(void)
{
   return PLIB_IC_Buffer16BitGet(IC_ID_1);
}

bool DRV_IC0_BufferIsEmpty(void)
{
   return PLIB_IC_BufferIsEmpty(IC_ID_1);
}

/*******************************************************************************
 End of File
*/
