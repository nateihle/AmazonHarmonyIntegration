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

#include "app.h"

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

void APP_ReadComplete (void *handle)
{
    appData.rdComplete = true;
}

void APP_WriteComplete (void *handle)
{
    appData.wrComplete = true;
}

void APP_Reset ()
{
    appData.rdComplete = true;
    appData.wrComplete = true;
}

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
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    appData.wrComplete = true;
    appData.rdComplete = true;
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    SYS_STATUS consoleStatus;

    consoleStatus = SYS_CONSOLE_Status(sysObj.sysConsole0);

    //Do not proceed in the current app state unless the console is ready
    if (consoleStatus != SYS_STATUS_READY)
    {
        if (consoleStatus == SYS_STATUS_ERROR)
        {
            APP_Reset();
            SYS_CONSOLE_Flush(SYS_CONSOLE_INDEX_0);
        }

        return;
    }

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
            if (SYS_CONSOLE_Status(sysObj.sysConsole0) == SYS_STATUS_READY)
            {
                SYS_CONSOLE_RegisterCallback(SYS_CONSOLE_INDEX_0, APP_ReadComplete, SYS_CONSOLE_EVENT_READ_COMPLETE);
                SYS_CONSOLE_RegisterCallback(SYS_CONSOLE_INDEX_0, APP_WriteComplete, SYS_CONSOLE_EVENT_WRITE_COMPLETE);
                appData.state = APP_STATE_START_ADC_CAPTURE;
                SYS_PRINT("\r\nStarting the test.\r\n");
                
                DRV_ADC0_Open();

                DRV_ADC_DigitalFilter0_Open();
                
                DRV_TMR0_Start();
            }
            break;
            
        case APP_STATE_START_ADC_CAPTURE:
        {
            DRV_ADC_Start();
            appData.state = APP_STATE_WAIT;
        }
        break;
        
        case APP_STATE_WAIT:
        {
            if (true == appData.overSampledDataReady)
            {
                appData.state = APP_STATE_SEND_RESULTS;
            }
            BSP_LEDOff(BSP_LED_3);
            BSP_LEDOn(BSP_LED_1);            
        }
        break;
        
        case APP_STATE_SEND_RESULTS:
        {
            if(true == appData.overSampledDataReady)
            {
                BSP_LEDOn(BSP_LED_3);
                BSP_LEDOff(BSP_LED_1);
                SYS_PRINT("Result: %d\r\n", appData.overSampledResult);
            }
			appData.overSampledDataReady = false;
            appData.state = APP_STATE_SPIN;
        }
        break;
        
        case APP_STATE_SPIN:
        {
            if (appData.tick)
            {
                appData.tick = false;
                appData.state = APP_STATE_START_ADC_CAPTURE;
            }
        }
        break;
        
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
