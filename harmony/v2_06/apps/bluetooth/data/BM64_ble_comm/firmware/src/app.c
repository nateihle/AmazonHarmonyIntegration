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
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

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

#include <string.h>
#include "app.h"
#include "driver/bluetooth/bm64/drv_bm64.h"     // only reference to BM64 (others use generic BT))
#if defined( ENABLE_SYS_LOG )
  #include "sys_log/sys_log.h"
  #include "sys_log/sys_log_define.h"
#endif

#include "ble.h"
extern BLE_DATA bleData;

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
    void APP_Initialize()

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize()
{
    APP_LED1_ON();      // all LED's on during initialization    
    APP_LED2_ON();
    APP_LED3_ON();
    APP_LED4_ON();
    APP_LED5_ON();
        
    //Place the App state machine in its initial state.
    appData.state = APP_STATE_INIT;
    
    appData.buttonState = BUTTON_STATE_IDLE;
    appData.buttonDelay = 0;        // used for button debounce
    appData.queryDelay = 0;         // used for status queries
    appData.linkStatus = -1;        // force mismatch 
    
    bleInitialize();
    
    APP_LED1_OFF();      // all LED's off
    APP_LED2_OFF();
    APP_LED3_OFF();
    APP_LED4_OFF();
    APP_LED5_OFF();    

 #if defined( ENABLE_SYS_LOG )    
    SYS_LOG_Init();
    SYS_LOG("----------------------------------------");
    SYS_LOG("- Starting:");
    SYS_LOG("----------------------------------------");
  #endif

}  //End APP_Initialize()

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */
SYS_TMR_HANDLE tmrHandle;
uint16_t lastappData_state = 0xffff;

void APP_Tasks()
{    
#if defined( ENABLE_SYS_LOG )
    if (appData.state != lastappData_state)    
    {
        SYS_LOG1("APP_Tasks: appData.state=%d",appData.state);
        lastappData_state = appData.state;
    }
#endif
  
    bleTasks(); 
   
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            laWidget_SetVisible((laWidget*)GFX_CONNECTED, LA_FALSE);
            laWidget_SetVisible((laWidget*)GFX_PAIRED, LA_FALSE);
            laWidget_SetVisible((laWidget*)GFX_NOPAIR_NOCONNECTION, LA_TRUE);
            
            tmrHandle = SYS_TMR_CallbackPeriodic (1, 0, &Timer_1ms);
            if (tmrHandle == DRV_HANDLE_INVALID)
            {
#if defined( ENABLE_SYS_LOG )    
                SYS_LOG("----------------------------------------");
                SYS_LOG("- Invalid tmrHandle in app.c");
                SYS_LOG("----------------------------------------");
#endif 
                // sticks in this state if handle invalid
            }
            else
            {           
                appData.state = APP_STATE_WAIT_INIT;
            }       
            break;
        }            

        case APP_STATE_WAIT_INIT:
        {
            if (DRV_BT_STATUS_READY == DRV_BT_GetPowerStatus())
            {           
                appData.state=APP_STATE_IDLE;
            }
            break;
        }
        
        // Initialized 
        case APP_STATE_IDLE:
        {
            buttonTasks();
            
            if (appData.queryDelay == 0)
            {
                DRV_BT_BLE_QueryStatus(bleData.bt.handle);
                appData.queryDelay = QUERY_DELAY;
            }
            
            break;
        }        
    }
 
}

void buttonTasks( )
{ 
    char* test_string;
    
    //BUTTON PROCESSING
    /* Check the buttons' current state. */      
    switch ( appData.buttonState )
    {
        case BUTTON_STATE_IDLE:
        {
            if ( (appData.buttonDelay==0)&&
                 ((BSP_SwitchStateGet(BSP_SWITCH_1)==BSP_SWITCH_STATE_PRESSED)||
                  (BSP_SwitchStateGet(BSP_SWITCH_2)==BSP_SWITCH_STATE_PRESSED)||
                  (BSP_SwitchStateGet(BSP_SWITCH_3)==BSP_SWITCH_STATE_PRESSED)||
                  (BSP_SwitchStateGet(BSP_SWITCH_4)==BSP_SWITCH_STATE_PRESSED)||
                  (BSP_SwitchStateGet(BSP_SWITCH_5)==BSP_SWITCH_STATE_PRESSED)) )
            {
                appData.buttonDelay=BUTTON_DEBOUNCE;       
                appData.buttonState=BUTTON_STATE_PRESSED;               
            }
            
            break;
        }
    
        case BUTTON_STATE_PRESSED:
        { 
            if (appData.buttonDelay>0)
            {
                break;      // still debouncing
            }
            
            if(BSP_SwitchStateGet(BSP_SWITCH_1)==BSP_SWITCH_STATE_PRESSED)              // SW1 pressed
            {                
                appData.buttonDelay=LONG_BUTTON_PRESS;          // 1 sec is long press
                appData.buttonState=BUTTON_STATE_SWITCH_1_PRESSED;                  
            }
            else if(BSP_SwitchStateGet(BSP_SWITCH_2)==BSP_SWITCH_STATE_PRESSED)         // SW2 pressed
            {
                appData.buttonDelay=LONG_BUTTON_PRESS;          // 1 sec is long press
                appData.buttonState=BUTTON_STATE_SWITCH_2_PRESSED;                 
            }                        
            
            else if (BSP_SwitchStateGet(BSP_SWITCH_3)==BSP_SWITCH_STATE_PRESSED)        // SW3 pressed
            {
                appData.buttonDelay=LONG_BUTTON_PRESS;          // 1 sec is long press
                appData.buttonState=BUTTON_STATE_SWITCH_3_PRESSED;                
            }
            
            else if (BSP_SwitchStateGet(BSP_SWITCH_4)==BSP_SWITCH_STATE_PRESSED)        // SW4 pressed
            {
                appData.buttonDelay=LONG_BUTTON_PRESS;          // 1 sec is long press
                appData.buttonState=BUTTON_STATE_SWITCH_4_PRESSED;                   
            }            
            else if (BSP_SwitchStateGet(BSP_SWITCH_5)==BSP_SWITCH_STATE_PRESSED)        // SW5 pressed
            {               
                appData.buttonDelay=LONG_BUTTON_PRESS;          // 1 sec is long press
                appData.buttonState=BUTTON_STATE_SWITCH_5_PRESSED;
            }             
            else
            {
                appData.buttonState=BUTTON_STATE_IDLE;                 // button not pressed anymore, assume noise                
            }

            break;
        }
        
        case BUTTON_STATE_SWITCH_1_PRESSED:
        {
            if ((appData.buttonDelay>0)&&
                (BSP_SwitchStateGet(BSP_SWITCH_1)!=BSP_SWITCH_STATE_PRESSED))           // SW1 pressed and released < 1 sec
            {
                test_string = "Button SW1 pressed";
                DRV_BT_SendDataOverBLE(bleData.bt.handle, (uint8_t*)test_string, strlen(test_string));
                
                appData.buttonDelay=BUTTON_DEBOUNCE;                
                appData.buttonState=BUTTON_STATE_IDLE;              
            }
            else if ((appData.buttonDelay==0)&&
                     (BSP_SwitchStateGet(BSP_SWITCH_1)==BSP_SWITCH_STATE_PRESSED))       // SW1 still pressed after 1 sec
            {
                DRV_BT_BLE_EnableAdvertising(bleData.bt.handle, true);
                appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE;                
            }
            
            break;
        }
        
        case BUTTON_STATE_SWITCH_2_PRESSED:
        {
            if ((appData.buttonDelay>0)&&
                (BSP_SwitchStateGet(BSP_SWITCH_2)!=BSP_SWITCH_STATE_PRESSED))           // SW2 pressed and released < 1 sec
            {
                test_string = "Button SW2 pressed";
                DRV_BT_SendDataOverBLE(bleData.bt.handle, (uint8_t*)test_string, strlen(test_string));
                
                appData.buttonDelay=BUTTON_DEBOUNCE;                
                appData.buttonState=BUTTON_STATE_IDLE;              
            }
            else if ((appData.buttonDelay==0)&&
                     (BSP_SwitchStateGet(BSP_SWITCH_2)==BSP_SWITCH_STATE_PRESSED))       // SW2 still pressed after 1 sec
            {
                DRV_BT_BLE_EnableAdvertising(bleData.bt.handle, false);                
                appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE;                
            }
            
            break;
            
            break;
        } 
        
        case BUTTON_STATE_SWITCH_3_PRESSED:
        {
            if ((appData.buttonDelay>0)&&
                (BSP_SwitchStateGet(BSP_SWITCH_3)!=BSP_SWITCH_STATE_PRESSED))           // SW3 pressed and released < 1 sec
            {
                test_string = "Button SW3 pressed";
                DRV_BT_SendDataOverBLE(bleData.bt.handle, (uint8_t*)test_string, strlen(test_string));
                
                appData.buttonDelay=BUTTON_DEBOUNCE;                
                appData.buttonState=BUTTON_STATE_IDLE;              
            }
            else if ((appData.buttonDelay==0)&&
                     (BSP_SwitchStateGet(BSP_SWITCH_3)==BSP_SWITCH_STATE_PRESSED))       // SW3 still pressed after 1 sec
            {               
                appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE;                
            }
            
            break;                     
        }
        
        case BUTTON_STATE_SWITCH_4_PRESSED:
        {
            if ((appData.buttonDelay>0)&&
                (BSP_SwitchStateGet(BSP_SWITCH_4)!=BSP_SWITCH_STATE_PRESSED))           // SW4 pressed and released < 1 sec
            {
                test_string = "Button SW4 pressed";
                DRV_BT_SendDataOverBLE(bleData.bt.handle, (uint8_t*)test_string, strlen(test_string));
                
                appData.buttonDelay=BUTTON_DEBOUNCE;                
                appData.buttonState=BUTTON_STATE_IDLE;              
            }
            else if ((appData.buttonDelay==0)&&
                     (BSP_SwitchStateGet(BSP_SWITCH_4)==BSP_SWITCH_STATE_PRESSED))       // SW4 still pressed after 1 sec
            {                
                appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE;                
            }
            
            break;
        }
        
        case BUTTON_STATE_SWITCH_5_PRESSED:
        {
            if ((appData.buttonDelay>0)&&
                (BSP_SwitchStateGet(BSP_SWITCH_5)!=BSP_SWITCH_STATE_PRESSED))           // SW5 pressed and released < 1 sec
            {
                test_string = "Button SW5 pressed";
                DRV_BT_SendDataOverBLE(bleData.bt.handle, (uint8_t*)test_string, strlen(test_string));
                
                appData.buttonDelay=BUTTON_DEBOUNCE;                
                appData.buttonState=BUTTON_STATE_IDLE;              
            }
            else if ((appData.buttonDelay==0)&&
                     (BSP_SwitchStateGet(BSP_SWITCH_5)==BSP_SWITCH_STATE_PRESSED))       // SW5 still pressed after 1 sec
            {               
                appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE;                
            }
            
            break;
        }  

        case BUTTON_STATE_WAIT_FOR_RELEASE:
        {
            if ((BSP_SwitchStateGet(BSP_SWITCH_1)!=BSP_SWITCH_STATE_PRESSED)&&
                (BSP_SwitchStateGet(BSP_SWITCH_2)!=BSP_SWITCH_STATE_PRESSED)&&
                (BSP_SwitchStateGet(BSP_SWITCH_3)!=BSP_SWITCH_STATE_PRESSED)&&
                (BSP_SwitchStateGet(BSP_SWITCH_4)!=BSP_SWITCH_STATE_PRESSED)&&
                (BSP_SwitchStateGet(BSP_SWITCH_5)!=BSP_SWITCH_STATE_PRESSED))
            {
                appData.buttonDelay=BUTTON_DEBOUNCE;
                appData.buttonState=BUTTON_STATE_IDLE;
            }
        }

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
    
}

void Timer_1ms(uintptr_t context, uint32_t currTick)
{
    //Button Processing
    if(appData.buttonDelay)
    {
        --appData.buttonDelay;
    }

    //BLE status
    if (appData.queryDelay)
    {
        --appData.queryDelay;
    }
}

uint32_t coretime_i = 0;
uint32_t coretime_f = 0;
uint32_t coretime [MAX_CORETIME_I];
uint32_t __attribute__((nomips16)) APP_ReadCoreTimer(void)
{
    uint32_t timer;
    // get the count reg
    asm volatile("mfc0   %0, $9" : "=r"(timer));
    return (timer);
}

void APP_SaveCoreTime(void)
{
    // to take measurement
    if (coretime_f && (coretime_i < (MAX_CORETIME_I-1)))
    {
        coretime [coretime_i++] = APP_ReadCoreTimer();
    }
}

void APP_StartCoreTime()
{
    coretime_f = 1;
}
/*******************************************************************************
 End of File
 */
