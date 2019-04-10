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
#include "rn4871.h"

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
uint32_t ButtonADC = 0;

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */
float Voltage = 0;

void APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {  
//                ADC_init();
                SYS_MESSAGE(" \n\r");
                SYS_MESSAGE("**********************************\n\r");
                SYS_MESSAGE("*      Initialized               *\n\r");
                SYS_MESSAGE("**********************************\n\r");
                DRV_ADC_Open();
                DRV_ADC_Start();
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }
        
        
        

        case APP_STATE_SERVICE_TASKS:
        {
            
            if (DRV_ADC_SamplesAvailable() == true)
            {
                //idle ~579 codes ~1.69v
                ButtonADC = DRV_ADC_SamplesRead(7);
                Voltage = 3.0/1024*ButtonADC;
//                SYS_PRINT("\r\n ADC value Raw: %d  || Voltage: %f",ButtonADC,Voltage);
                if(ButtonADC == 822  || ButtonADC == 823  ||ButtonADC == 824 )
                {
                    SYS_MESSAGE("\r\n SW5 pressed \r\n");
                }
                else if(ButtonADC == 699  || ButtonADC == 700  ||ButtonADC == 701 )
                {
                    SYS_MESSAGE("\r\n SW4 pressed \r\n");
                }
                else if(ButtonADC == 638  || ButtonADC == 639  ||ButtonADC == 640 )
                {
                    SYS_MESSAGE("\r\n SW3 pressed \r\n");
                }
//                else if( ButtonADC == 581  ||ButtonADC == 582 )
//                {
//                    SYS_MESSAGE("\r\n SW2 pressed \r\n");
//                }
                else if(ButtonADC == 592  || ButtonADC == 593  ||ButtonADC == 5594 )
                {
                    SYS_MESSAGE("\r\n SW1 pressed \r\n");
                }
            }
            RN4871_Tasks(); //will be moved to system task over time
            Nop();
            break;
        }

        
        

        /* The default state should never be executed. */
        default:
        {
            BSP_LEDOn(BSP_LED_RED);
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
