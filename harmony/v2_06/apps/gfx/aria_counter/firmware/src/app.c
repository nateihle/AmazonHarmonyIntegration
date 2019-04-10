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
#include "app_splash.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

#define GET_TICKS() __builtin_mfc0(9, 0)

#define MAX_COUNTDOWN 9999
#define MIN_COUNTDOWN 0
#define MAX_COUNTUP 99999
#define MIN_COUNTUP 0

uint32_t tickDivisor;
uint32_t lastTickUp;
uint32_t lastTickDown;

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

void APP_DecrementCount(laLabelWidget* labelWidget)
{
    static uint32_t tickCount = MAX_COUNTDOWN;
    static char charBuff[10];
    laString str;
    
    sprintf(charBuff, "%u", tickCount--);
    
    str = laString_CreateFromCharBuffer(charBuff, 
            GFXU_StringFontIndexLookup(&stringTable, string_NewTxt, 0));
    
    laLabelWidget_SetText(labelWidget, str);
    
    laString_Destroy(&str);
    
    if (tickCount == MIN_COUNTDOWN)
        tickCount = MAX_COUNTDOWN;

}

void APP_IncrementCount(laLabelWidget* digit0,
                        laLabelWidget* digit1,
                        laLabelWidget* digit2,
                        laLabelWidget* digit3,
                        laLabelWidget* digit4)
{
    static uint32_t tickCountUp = MIN_COUNTUP;    
    
    tickCountUp++;
    uint32_t tickMod = tickCountUp % 10;
    uint32_t tickMod10 = (tickCountUp / 10) % 10;
    uint32_t tickMod100 = (tickCountUp / 100) % 10;
    uint32_t tickMod1000 = (tickCountUp / 1000) % 10;
    uint32_t tickMod10000 = (tickCountUp / 10000) % 10;
    
    switch (tickMod)
    {
        case 0:
            laLabelWidget_SetText(digit0, laString_CreateFromID(string_Zero));            
            break;
        case 1:
            laLabelWidget_SetText(digit0, laString_CreateFromID(string_One));            
            break;
        case 2:
            laLabelWidget_SetText(digit0, laString_CreateFromID(string_Two));            
            break;
        case 3:
            laLabelWidget_SetText(digit0, laString_CreateFromID(string_Three));            
            break;
        case 4:
            laLabelWidget_SetText(digit0, laString_CreateFromID(string_Four));            
            break;
        case 5:
            laLabelWidget_SetText(digit0, laString_CreateFromID(string_Five));            
            break;
        case 6:
            laLabelWidget_SetText(digit0, laString_CreateFromID(string_Six));            
            break;
        case 7:
            laLabelWidget_SetText(digit0, laString_CreateFromID(string_Seven));            
            break;
        case 8:
            laLabelWidget_SetText(digit0, laString_CreateFromID(string_Eight));            
            break;
        case 9:
            laLabelWidget_SetText(digit0, laString_CreateFromID(string_Nine));            
            break;
        default:
            break;
    }

    switch (tickMod10)
    {
        case 0:
            laLabelWidget_SetText(digit1, laString_CreateFromID(string_Zero));            
            break;
        case 1:
            laLabelWidget_SetText(digit1, laString_CreateFromID(string_One));            
            break;
        case 2:
            laLabelWidget_SetText(digit1, laString_CreateFromID(string_Two));            
            break;
        case 3:
            laLabelWidget_SetText(digit1, laString_CreateFromID(string_Three));            
            break;
        case 4:
            laLabelWidget_SetText(digit1, laString_CreateFromID(string_Four));            
            break;
        case 5:
            laLabelWidget_SetText(digit1, laString_CreateFromID(string_Five));            
            break;
        case 6:
            laLabelWidget_SetText(digit1, laString_CreateFromID(string_Six));            
            break;
        case 7:
            laLabelWidget_SetText(digit1, laString_CreateFromID(string_Seven));            
            break;
        case 8:
            laLabelWidget_SetText(digit1, laString_CreateFromID(string_Eight));            
            break;
        case 9:
            laLabelWidget_SetText(digit1, laString_CreateFromID(string_Nine));            
            break;
        default:
            break;
    }

    switch (tickMod100)
    {
        case 0:
            laLabelWidget_SetText(digit2, laString_CreateFromID(string_Zero));            
            break;
        case 1:
            laLabelWidget_SetText(digit2, laString_CreateFromID(string_One));            
            break;
        case 2:
            laLabelWidget_SetText(digit2, laString_CreateFromID(string_Two));            
            break;
        case 3:
            laLabelWidget_SetText(digit2, laString_CreateFromID(string_Three));            
            break;
        case 4:
            laLabelWidget_SetText(digit2, laString_CreateFromID(string_Four));            
            break;
        case 5:
            laLabelWidget_SetText(digit2, laString_CreateFromID(string_Five));            
            break;
        case 6:
            laLabelWidget_SetText(digit2, laString_CreateFromID(string_Six));            
            break;
        case 7:
            laLabelWidget_SetText(digit2, laString_CreateFromID(string_Seven));            
            break;
        case 8:
            laLabelWidget_SetText(digit2, laString_CreateFromID(string_Eight));            
            break;
        case 9:
            laLabelWidget_SetText(digit2, laString_CreateFromID(string_Nine));            
            break;
        default:
            break;
    }

    switch (tickMod1000)
    {
        case 0:
            laLabelWidget_SetText(digit3, laString_CreateFromID(string_Zero));            
            break;
        case 1:
            laLabelWidget_SetText(digit3, laString_CreateFromID(string_One));            
            break;
        case 2:
            laLabelWidget_SetText(digit3, laString_CreateFromID(string_Two));            
            break;
        case 3:
            laLabelWidget_SetText(digit3, laString_CreateFromID(string_Three));            
            break;
        case 4:
            laLabelWidget_SetText(digit3, laString_CreateFromID(string_Four));            
            break;
        case 5:
            laLabelWidget_SetText(digit3, laString_CreateFromID(string_Five));            
            break;
        case 6:
            laLabelWidget_SetText(digit3, laString_CreateFromID(string_Six));            
            break;
        case 7:
            laLabelWidget_SetText(digit3, laString_CreateFromID(string_Seven));            
            break;
        case 8:
            laLabelWidget_SetText(digit3, laString_CreateFromID(string_Eight));            
            break;
        case 9:
            laLabelWidget_SetText(digit3, laString_CreateFromID(string_Nine));            
            break;
        default:
            break;
    }

    switch (tickMod10000)
    {
        case 0:
            laLabelWidget_SetText(digit4, laString_CreateFromID(string_Zero));            
            break;
        case 1:
            laLabelWidget_SetText(digit4, laString_CreateFromID(string_One));            
            break;
        case 2:
            laLabelWidget_SetText(digit4, laString_CreateFromID(string_Two));            
            break;
        case 3:
            laLabelWidget_SetText(digit4, laString_CreateFromID(string_Three));            
            break;
        case 4:
            laLabelWidget_SetText(digit4, laString_CreateFromID(string_Four));            
            break;
        case 5:
            laLabelWidget_SetText(digit4, laString_CreateFromID(string_Five));            
            break;
        case 6:
            laLabelWidget_SetText(digit4, laString_CreateFromID(string_Six));            
            break;
        case 7:
            laLabelWidget_SetText(digit4, laString_CreateFromID(string_Seven));            
            break;
        case 8:
            laLabelWidget_SetText(digit4, laString_CreateFromID(string_Eight));            
            break;
        case 9:
            laLabelWidget_SetText(digit4, laString_CreateFromID(string_Nine));            
            break;
        default:
            break;
    }

    if (tickCountUp == MAX_COUNTUP)
        tickCountUp = MIN_COUNTUP;
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************



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

extern APP_SPLASH_DATA appSplashData;
void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    appSplashData.state = 0;

    tickDivisor = SYS_CLK_FREQ / 100000;    
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    uint32_t currentTick;
    
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true; 
            
            if (appInitialized)
            {
                appData.state = APP_STATE_SPLASH;
            }
            break;
        }

        case APP_STATE_SPLASH:
        {            
            if (APP_IsSplashScreenComplete())
            {
                appData.state = APP_STATE_SERVICE_TASKS;
                
                laContext_SetActiveScreen(1);
            }
        
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
            currentTick = GET_TICKS();
        
            if (1)
            {
                if ((currentTick - lastTickUp) > 25000000)
                {
                    
                    if (laContext_GetActiveScreenIndex() == MainScreen_ID)
                    {
                        APP_DecrementCount(LabelWidget2);
                    }
                    else
                    {
                        APP_DecrementCount(LabelWidget_Single);
                    }
                    lastTickUp = currentTick;
                }    
                if((currentTick - lastTickDown) > 5000000)
                {                
                    
                    if (laContext_GetActiveScreenIndex() == MainScreen_ID)
                    {
                        APP_IncrementCount(Digit0, Digit1, Digit2, Digit3, Digit4);
                    }
                    else
                    {
                        APP_IncrementCount(Digit0_1, Digit1_1, Digit2_1, Digit3_1, Digit4_1);
                    }
                    lastTickDown = currentTick;
                }                
            }            

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
