/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    heartbeat.c

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

#include "heartbeat.h"

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

HEARTBEAT_DATA heartbeatData;

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


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void HEARTBEAT_Initialize ( void )

  Remarks:
    See prototype in heartbeat.h.
 */

void HEARTBEAT_Initialize ( void )
{
    SYS_MESSAGE("\r\nApplication created " __DATE__ " " __TIME__ " initialized!\r\n");
    //Test out error handling
//    assert(0);
//    {
//        uint8_t x, y, z;
//        x = 1;
//        y = 0;
//        z = x/y;
//        SYS_DEBUG_PRINT(SYS_ERROR_DEBUG,"x: %d, y: %d, z: %d\r\n",x,y,z);
//    }
    
    /* Place the App state machine in its initial state. */
    heartbeatData.state = HEARTBEAT_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void HEARTBEAT_Tasks ( void )

  Remarks:
    See prototype in heartbeat.h.
 */

void HEARTBEAT_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( heartbeatData.state )
    {
        /* Application's initial state. */
        case HEARTBEAT_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
                heartbeatData.hDelayTimer = SYS_TMR_DelayMS(HEARTBEAT_DELAY);
                if (heartbeatData.hDelayTimer != SYS_TMR_HANDLE_INVALID)
                { // Valid handle returned
                    BSP_LEDOn(HEARTBEAT_LED);
                    heartbeatData.state = HEARTBEAT_STATE_SERVICE_TASKS;
                }            
                heartbeatData.state = HEARTBEAT_STATE_SERVICE_TASKS;
            }
            break;
        }

        case HEARTBEAT_STATE_SERVICE_TASKS:
        {
            if (SYS_TMR_DelayStatusGet(heartbeatData.hDelayTimer))
            { // Single shot timer has now timed out.
                BSP_LEDToggle(HEARTBEAT_LED);
                heartbeatData.state = HEARTBEAT_RESTART_TIMER;
            }        
            break;
        }

        /* TODO: implement your application state machine.*/
        case HEARTBEAT_RESTART_TIMER:
        { // Create a new timer
            heartbeatData.hDelayTimer = SYS_TMR_DelayMS(HEARTBEAT_DELAY);
            if (heartbeatData.hDelayTimer != SYS_TMR_HANDLE_INVALID)
            { // Valid handle returned
                heartbeatData.state = HEARTBEAT_STATE_SERVICE_TASKS;
            }
            break;
        }        

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
