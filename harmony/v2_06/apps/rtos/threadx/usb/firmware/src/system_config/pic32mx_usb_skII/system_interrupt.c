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
#include "app1.h"
#include "system_definitions.h"

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************

void ReloadTimer(void);
// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
Core timer is used as the RTOS tick source, user is free to change this to any
timer source wanted.
*******************************************************************************/
#if ( __mips_micromips == 1 ) 
void __attribute__((micromips)) __attribute__((interrupt(IPL2AUTO))) __attribute__((vector(_CORE_TIMER_VECTOR))) CoreTimerHandler(void)
#else 
void __ISR(_CORE_TIMER_VECTOR, IPL2AUTO) CoreTimerHandler(void)
#endif
{

   /* Call ThreadX context save.  */
   _tx_thread_context_save();

   /*reload timer, 1ms period*/
   ReloadTimer();

   /* Call ThreadX timer interrupt processing.  */
   _tx_timer_interrupt();

   /* Call ThreadX context restore.  */
   _tx_thread_context_restore();
}


/*******************************************************************************
 * This function is written to reload the core timer, which sources the RTOS
 * tick in this application.  The user can substitue any timer source wanted by
 * altering the code to reload any of PIC32 timers.
 *
 ******************************************************************************/
void ReloadTimer(void)
{
   unsigned int interrupt_save;
   /*do this operation atomically*/
   interrupt_save = __builtin_disable_interrupts();

   /*make sure the core timer starts counting next period*/
   __builtin_mtc0(11,0,(__builtin_mfc0(9,0) + (SYS_CLK_SystemFrequencyGet())/(2*1000)));
   /*clear core timer interrupt flag*/
   PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_CORE);
   //IFS0CLR = 0x00000001;

   __builtin_mtc0(12,0,interrupt_save);
}
 
void __ISR(_USB_1_VECTOR, ipl4AUTO) _IntHandlerUSBInstance0(void)

{
   /* Call ThreadX context save.  */
   _tx_thread_context_save();
    DRV_USBFS_Tasks_ISR(sysObj.drvUSBObject);
   /* Call ThreadX context restore.  */
   _tx_thread_context_restore();
}

/*******************************************************************************
 End of File
*/
