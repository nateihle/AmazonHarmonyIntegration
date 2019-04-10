/*******************************************************************************
 System Tasks File

  File Name:
    system_tasks.c

  Summary:
    This file contains source code necessary to maintain system's polled state
    machines.

  Description:
    This file contains source code necessary to maintain system's polled state
    machines.  It implements the "SYS_Tasks" function that calls the individual
    "Tasks" functions for all the MPLAB Harmony modules in the system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    polled in the system.  These handles are passed into the individual module
    "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

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

#include "system_config.h"
#include "system_definitions.h"
#include "app.h"


// *****************************************************************************
// *****************************************************************************
// Section: Local Prototypes
// *****************************************************************************
// *****************************************************************************



uint8_t*   _sys_tx_thread_stk_ptr;

/*create any necessary threadx rtos resources*/
TX_BYTE_POOL _sys_byte_pool_0;
TX_THREAD  _SYS_Tasks_TCB;
 
 

TX_THREAD  _APP_Tasks_TCB;
 

static void _SYS_Tasks( ULONG thread_input );
 
 
static void _APP_Tasks(ULONG thread_input);


// *****************************************************************************
// *****************************************************************************
// Section: System "Tasks" Routine
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SYS_Tasks ( void )

  Remarks:
    See prototype in system/common/sys_module.h.
*/

#define SYS_BYTE_POOL_SIZE   2560

void tx_application_define(void* first_unused_memory)
{
    /* Create a byte memory pool from which to allocate the thread stacks.  */
    tx_byte_pool_create(&_sys_byte_pool_0, "sys byte pool 0", first_unused_memory,SYS_BYTE_POOL_SIZE);

    /* Allocate the stack for system and application threads */
    tx_byte_allocate(&_sys_byte_pool_0, (VOID **) &_sys_tx_thread_stk_ptr, 
        1024,TX_NO_WAIT);

    /* create the RTOS thread*/
    tx_thread_create(&_SYS_Tasks_TCB,"Sys Tasks",_SYS_Tasks,0,
        _sys_tx_thread_stk_ptr,1024,6, 6,
        TX_NO_TIME_SLICE,TX_AUTO_START);

 
 

    /* Maintain the application's state machine. */
    /* Allocate the stack for system and application threads */
    tx_byte_allocate(&_sys_byte_pool_0, (VOID **) &_sys_tx_thread_stk_ptr, 
        1024,TX_NO_WAIT);

    /* create the RTOS thread*/
    tx_thread_create(&_APP_Tasks_TCB,"APP Tasks",_APP_Tasks,0,
        _sys_tx_thread_stk_ptr,1024,5, 5,
        TX_NO_TIME_SLICE,TX_AUTO_START);
}
// *****************************************************************************
// *****************************************************************************
// Section: System "Tasks" Routine
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SYS_Tasks ( void )

  Remarks:
    See prototype in system/common/sys_module.h.
*/
void SYS_Tasks ( void )
{
    /*Enter the ThreadX kernel.*/
    tx_kernel_enter();
}


/*******************************************************************************
  Function:
    void _SYS_Tasks ( ULONG thread_input )

  Summary:
    Maintains state machines of system modules.
*/
static void _SYS_Tasks ( ULONG thread_input)
{
    while(1)
    {
        /* Maintain system services */

        /* Maintain Device Drivers */
 
 

        /* Maintain Middleware */

        /* Task Delay */
        tx_thread_sleep(1000);
    }
}

 
 

/*******************************************************************************
  Function:
    void _APP_Tasks ( ULONG thread_input )

  Summary:
    Maintains state machine of APP.
*/

static void _APP_Tasks(ULONG thread_input)
{
    while(1)
    {
        APP_Tasks();
        tx_thread_sleep(500);
    }
}


/*******************************************************************************
 End of File
 */
