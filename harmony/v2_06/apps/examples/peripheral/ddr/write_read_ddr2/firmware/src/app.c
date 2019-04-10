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
}


#define LED_TOGGLE_COUNT    0x10000
#define MEM_SIZE            DDR_SIZE
#define DDR_START           (0xA8000000)
#define READ_WRITE_BLOCK    0x100
/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    volatile unsigned int strt_addr;

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            appData.addr = DDR_START;
            appData.count = 0;
            appData.state = APP_STATE_WRITE;
            BSP_LEDOff(BSP_LED_1);
            BSP_LEDOff(BSP_LED_2);
            BSP_LEDOff(BSP_LED_3);
            break;
        }
        case APP_STATE_WRITE:
        {
            if(BSP_SWITCH_STATE_PRESSED == BSP_SwitchStateGet(BSP_SWITCH_1))
            {
                appData.state = APP_STATE_INIT;
            }

            if(appData.count == 0)
            {
                appData.count = LED_TOGGLE_COUNT;
                BSP_LEDToggle(BSP_LED_1);
                BSP_LEDToggle(BSP_LED_2);
            }
            appData.count--;

            if ( appData.addr < (DDR_START + MEM_SIZE)) 
            {
                strt_addr = appData.addr;
                while (appData.addr < strt_addr + READ_WRITE_BLOCK)
                {
                    (*(volatile unsigned int *)(appData.addr)) = appData.addr;
                    appData.addr += 4;
                }
            }
            else
            {
                appData.addr = DDR_START;
                appData.state = APP_STATE_READ;
                appData.count = 0;
                BSP_LEDOff(BSP_LED_1);
                BSP_LEDOff(BSP_LED_2);
                BSP_LEDOff(BSP_LED_3);
            }
            break;
        }
        case APP_STATE_READ:
        {
            if(BSP_SWITCH_STATE_PRESSED == BSP_SwitchStateGet(BSP_SWITCH_1))
            {
                appData.state = APP_STATE_INIT;
            }

            if(appData.count == 0)
            {
                appData.count = LED_TOGGLE_COUNT;
                BSP_LEDToggle(BSP_LED_3);
                BSP_LEDToggle(BSP_LED_2);
            }
            appData.count--;

            if ( appData.addr < (DDR_START + MEM_SIZE)) 
            {
                strt_addr = appData.addr;
                while (appData.addr < strt_addr + READ_WRITE_BLOCK)
                {
                    if( (*(volatile unsigned int *)(appData.addr)) == appData.addr)
                    {
                        appData.addr += 4;
                    }
                    else
                    {
                        BSP_LEDOff(BSP_LED_1);
                        BSP_LEDOff(BSP_LED_2);
                        BSP_LEDOff(BSP_LED_3);
                        appData.count = 0;
                        appData.state = APP_STATE_FAIL;
                        break;
                    }
                }
            }
            else
            {
                BSP_LEDOff(BSP_LED_1);
                BSP_LEDOff(BSP_LED_2);
                BSP_LEDOff(BSP_LED_3);
                appData.count = 0;
                appData.state = APP_STATE_PASS;
            }
            break;
        }
        case APP_STATE_FAIL:
        {
            if(BSP_SWITCH_STATE_PRESSED == BSP_SwitchStateGet(BSP_SWITCH_1))
            {
                appData.state = APP_STATE_INIT;
            }

            if(appData.count == 0)
            {
                appData.count = LED_TOGGLE_COUNT;
                BSP_LEDToggle(BSP_LED_1);
            }
            appData.count--;
            break;
        }
        case APP_STATE_PASS:
        {
            if(BSP_SWITCH_STATE_PRESSED == BSP_SwitchStateGet(BSP_SWITCH_1))
            {
                appData.state = APP_STATE_INIT;
            }
            if(appData.count == 0)
            {
                appData.count = LED_TOGGLE_COUNT;
                BSP_LEDToggle(BSP_LED_3);
            }
            appData.count--;
            break;
        }

        /* TODO: implement your application state machine.*/
    //res = ddr_test();
    
    //if (res) {
    //    /* FAIL: LED 1 (red) */
    //    BSP_LEDOn(BSP_LED_1);
    //} else {
    //  /  /* PASS: LED 3 (green) */
    //    BSP_LEDOn(BSP_LED_3);
    //}
    //    /* The default state should never be executed. */
    //    default:
    //    {
    //        /* TODO: Handle error in application's state machine. */
    //        break;
    //    }
    }
}
 

/*******************************************************************************
 End of File
 */
