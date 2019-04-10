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
#include "app.h"
#include "system_definitions.h"

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************




void __ISR(_DMA0_VECTOR, ipl1AUTO) _IntHandlerSysDmaCh0(void)
{          
    extern APP_DATA appData;
    extern uint16_t adc_buffer[];
    uint16_t i;
        
    /* Clear the interrupt flag */
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_DMA_0);
    
    /* TODO: Add code as needed */ 
    if (PLIB_DMA_ChannelXINTSourceFlagGet(DMA_ID_0, DMA_CHANNEL_0, DMA_INT_DESTINATION_HALF_FULL))
    {
        PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, DMA_CHANNEL_0, DMA_INT_DESTINATION_HALF_FULL);
        appData.buffer_flag = 1;
        
        /* perform average of the data */
        appData.avgValue = 0;
        
        for(i = 0; i < 127; i++)
        {
            appData.avgValue += adc_buffer[i];
        }
    }
    if (PLIB_DMA_ChannelXINTSourceFlagGet(DMA_ID_0, DMA_CHANNEL_0, DMA_INT_DESTINATION_DONE))
    {
        PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, DMA_CHANNEL_0, DMA_INT_DESTINATION_DONE);
        appData.buffer_flag = 2;

        appData.avgValue = 0;
        
        for(i = 0; i < 127; i++)
        {
            appData.avgValue += adc_buffer[i + 127];
        }        
    }
    
    appData.avgValue = appData.avgValue/128;

    appData.avgValue >>= 7; /* 10-bit value to 3-bit value */

    appData.ledMask = 0;

    /* Creates a mask for the LEDs, corresponding to the value read from
       the potentiometer */
    for (i = 0; i <= appData.avgValue; i++)
    {
        appData.ledMask |=  1<<(i);
    }

    /* Write the mask to the LEDs */
    PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_A, (PORTS_DATA_MASK)appData.ledMask);    
}

 
/*******************************************************************************
 End of File
*/

