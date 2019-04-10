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
#define ADC_REF_VOLT            (float)3.33
#define ADC_MAX_READING         (float)4096        // bits
#define ADC_CONV_FACT           (float)(ADC_MAX_READING/ADC_REF_VOLT)   // bits/V

/* definitions for temperature sensor */
#define MCP9700_COEFF               (float)0.01         // V/degC
#define MCP9700_COEFF_SCALED        (int)(MCP9700_COEFF * ADC_CONV_FACT)   // bits/degC
#define MCP9700_0DEG_OUT            (float)0.5          // V @ 0degC
#define MCP9700_0DEG_OUT_SCALED     (int)(MCP9700_0DEG_OUT * ADC_CONV_FACT)   // bits/degC

/* definitions for accelerometer */
#define ADXL325_SENSITIVITY         (float)0.174        // V/g
#define ADXL325_COEFF               (int)(ADXL325_SENSITIVITY *ADC_CONV_FACT)   // bits/g
 
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
    appData.dataReady = true;
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
                appData.state = APP_STATE_WAIT;
                SYS_PRINT("\r\nStarting the test.\r\n");
                
                DRV_ADC0_Open();

                
                DRV_TMR0_Start();
            }
            break;
            
        case APP_STATE_WAIT:
        {
            if (true == appData.dataReady)
            {
                appData.state = APP_STATE_SEND_RESULTS;
            } else {
                // Count ticks and dump blank data after some time
                if (appData.tick)
                {
                    appData.tick = false;
                    appData.tickcount++;
                    if(appData.tickcount > 100)
                    {
                        appData.tickcount = 0;
            			appData.dataReady = true;
                        SYS_PRINT("Timeout!\r\n");
                    }
                }
                
            }
            BSP_LEDOff(BSP_LED_3);
            BSP_LEDOn(BSP_LED_1);            
        }
        break;
        
        case APP_STATE_SEND_RESULTS:
        {
            if(true == appData.dataReady)
            {
                BSP_LEDOn(BSP_LED_3);
                BSP_LEDOff(BSP_LED_1);
                appData.tempDegC = (appData.tempSensor - MCP9700_0DEG_OUT_SCALED)/MCP9700_COEFF_SCALED;
                SYS_PRINT("Temp: %d deg C\r\n", appData.tempDegC);
                
                appData.fltxAxis = (float)appData.xAxis/ADXL325_COEFF;
                appData.fltyAxis = (float)appData.yAxis/ADXL325_COEFF;
                appData.fltzAxis = (float)appData.zAxis/ADXL325_COEFF;
                
                if(appData.fltxAxis > 1.0)
                    appData.fltxAxis = 1.0;
                else if(appData.fltxAxis < -1.0)
                    appData.fltxAxis = -1.0;
                
                if(appData.fltyAxis > 1.0)
                    appData.fltyAxis = 1.0;
                else if(appData.fltyAxis < -1.0)
                    appData.fltyAxis = -1.0;
                
                if(appData.fltzAxis > 1.0)
                    appData.fltzAxis = 1.0;
                else if(appData.fltzAxis < -1.0)
                    appData.fltzAxis = -1.0;
                
                appData.fltxAxisSqr = appData.fltxAxis * appData.fltxAxis;
                appData.fltyAxisSqr = appData.fltyAxis * appData.fltyAxis;
                appData.fltzAxisSqr = appData.fltzAxis * appData.fltzAxis;
                
                appData.xySqr = sqrtf(appData.fltxAxisSqr + appData.fltyAxisSqr);
                appData.yzSqr = sqrtf(appData.fltyAxisSqr + appData.fltzAxisSqr);
                appData.zxSqr = sqrtf(appData.fltzAxisSqr + appData.fltxAxisSqr);
                
                appData.xRatio = (appData.fltxAxis/appData.yzSqr);
                appData.yRatio = (appData.fltyAxis/appData.zxSqr);
                appData.zRatio = (appData.fltzAxis/appData.xySqr);
                
                appData.xAngle = atanf(appData.xRatio);
                appData.yAngle = atanf(appData.yRatio);
                appData.zAngle = atanf(appData.zRatio); 
                
                SYS_PRINT("x_axis: %f radians\r\n", appData.xAngle);
                SYS_PRINT("y_axis: %f radians\r\n", appData.yAngle);
                SYS_PRINT("z_axis: %f radians\r\n", appData.zAngle);     
                SYS_PRINT("\r\n");  
            }
			appData.dataReady = false;
            appData.state = APP_STATE_SPIN;
        }
        break;
        
        case APP_STATE_SPIN:
        {
            if (appData.tick)
            {
                appData.tick = false;
                appData.state = APP_STATE_WAIT;
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
