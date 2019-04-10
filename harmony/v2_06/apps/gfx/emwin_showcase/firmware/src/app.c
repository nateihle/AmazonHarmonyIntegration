/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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


#include <xc.h>
#include <sys/attribs.h>
#include "app.h"
#include "GUI.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

extern volatile GUI_TIMER_TIME OS_TimeMS;

void __ISR(_TIMER_2_VECTOR,ipl3AUTO) _Timer2Handler(void) {
  static unsigned char Cnt;

  SYS_INT_SourceStatusClear(INT_SOURCE_TIMER_2);
  OS_TimeMS += (Cnt ^= 1);
}

/* TODO:  Add any necessary local functions.
*/
//static void _StartTimer(void) {
//  T2CON    = 0x0;        // Stop the timer and clear the control register,
//                         // prescaler at 1:1,internal clock source
//  TMR2     = 0x0;        // Clear the timer register
//  PR2      = 0xBD55;     // Load the period register
//  IPC2SET  = 0x0000000C; // Set priority level = 3
//  IPC2SET  = 0x00000001; // Set subpriority level = 1
//  IFS0CLR  = 0x00000100; // Clear the timer interrupt status flag
//  IEC0SET  = 0x00000100; // Enable timer interrupts
//  T2CONSET = 0x8000;     // Start the timer
//  SYS_INT_SourceEnable(INT_SOURCE_TIMER_2);
//}


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    
    if (GFX_Open(0, 0, 0, GFX_NULL) == NULL)
    {
        return;
    }
    
    SYS_INT_SourceDisable(INT_SOURCE_TIMER_CORE);

    T2CON    = 0x0;        // Stop the timer and clear the control register,
                           // prescaler at 1:1,internal clock source
    TMR2     = 0x0;        // Clear the timer register
    PR2      = 0xBD55;     // Load the period register
    IPC2SET  = 0x0000000C; // Set priority level = 3
    IPC2SET  = 0x00000001; // Set subpriority level = 1
    IFS0CLR  = 0x00000100; // Clear the timer interrupt status flag
    IEC0SET  = 0x00000100; // Enable timer interrupts
    T2CONSET = 0x8000;     // Start the timer
    SYS_INT_SourceEnable(INT_SOURCE_TIMER_2);

    /* set priority for timer interrupt source */
    SYS_INT_VectorPrioritySet(INT_VECTOR_T2, INT_PRIORITY_LEVEL3);

    /* set sub-priority for timer interrupt source */
    SYS_INT_VectorSubprioritySet(INT_VECTOR_T2, INT_SUBPRIORITY_LEVEL3);
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            MainTask();
            break;
        }

        /* TODO: implement your application state machine.*/

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}
 

/*******************************************************************************
 End of File
 */
