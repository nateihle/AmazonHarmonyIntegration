/*******************************************************************************
  SPP data Demo

  Company:
    Microchip Technology Inc.

  File Name:
    app.c

  Summary:
    Contains the functional implementation of this demo application.

  Description:
    This file contains the functional implementation of this demo application.
    The BT_MEB-II/BTAD exchanges commands with the BT module over SPP link.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
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
//#include <proc/p32mz2048ech144.h>

#include "app.h"

//#include "gfx_resources.h"
#include "msg_handler.h"
#include "cdbt/bt/bt_version.h"

#if defined( ENABLE_SYS_LOG )
#include "application/sys_log/sys_log.h"
#endif
#include "application/sys_log/sys_log_define.h"

//static GFX_INDEX            gfxIndex=0;
// *****************************************************************************
// *****************************************************************************
// Section: Global Variable Definitions
// *****************************************************************************
// *****************************************************************************
/*****************************************************
 * Initialize the application data structure. All
 * application related variables are stored in this
 * data structure.
 *****************************************************/
APP_DATA appData =
{
    /* App state */
    .state = APP_STATE_BT_REGISTER_TICK_HANDLER,
};

/******************************************************
 * Application Initialize. It is
 * called from the SYS_Initialized() function.
 ******************************************************/
void APP_Initialize (void)
{
    //Bluetooth Version 1.8.xx is Dostack 30
    const bt_byte * __attribute__((unused)) version =  bt_sys_get_version();

    #if defined( ENABLE_SYS_LOG )    
    SYS_LOG_Init();
    SYS_LOG("----------------------------------------");
    SYS_LOG1("- Starting: BT Version %s",version);
    SYS_LOG("----------------------------------------");

    #endif

    /* Bluetooth related Initializations */
    bluetoothInit();

    /* Buttons initialization */
    buttonsInit();

    display_init(&BT_DISPLAY_STATS);
}

/**********************************************************
 * Application tasks routine. This function implements the
 * application state machine.
 ***********************************************************/
void APP_Tasks (void )
{

    switch(appData.state)
    {
        /* Register the BT Tick Handler with Timer system service */
        case APP_STATE_BT_REGISTER_TICK_HANDLER:
        {
            appData.tickSysTmrHandle = SYS_TMR_CallbackPeriodic (APP_BT_TICK_TIMER_MS,
                            (uintptr_t)0, &APP_BTTimerTickCallbackHandler);
            if(SYS_TMR_HANDLE_INVALID == appData.tickSysTmrHandle)
            {
                /* Do Nothing. We iterate till we get a valid Timer System
                 * service handle */
//                BSP_LEDToggle(BSP_LED_D6);
            }
            else
            {
                appData.state = APP_STATE_BT_RESET_MODULE_START;
            }
        }
        break;

        /* Initiate the BT Radio Reset process */
        case APP_STATE_BT_RESET_MODULE_START:
        {
            bttask_pal_startBluetoothPort_1(&onBluetoothPortStarted);
            appData.tickSysTmrHandle = SYS_TMR_DelayMS(APP_BT_MODULE_RESET_DURATION_MS);
            appData.state = APP_STATE_BT_RESET_MODULE_PROCESSING;
        }
        break;

        /* Waiting till the BT Radio Reset time elapses */
        case APP_STATE_BT_RESET_MODULE_PROCESSING:
        {
            if (true == SYS_TMR_DelayStatusGet(appData.tickSysTmrHandle))
            {
                SYS_PORTS_PinSet(PORTS_ID_0, APP_BT_RESET_PORT, APP_BT_RESET_BIT);

                appData.state = APP_STATE_BT_SETUP_USART_DRIVER;
            }
        }
        break;

        /* Setting up the BT USART port for HCI communications */
        case APP_STATE_BT_SETUP_USART_DRIVER:
        {
            /* Open a USART client and setup the buffer event handler */
            appData.usartClient.handle = DRV_USART_Open(DRV_USART_INDEX_0,
                        DRV_IO_INTENT_READWRITE|DRV_IO_INTENT_NONBLOCKING);
            if(appData.usartClient.handle != DRV_HANDLE_INVALID)
            {
                if(DRV_USART_CLIENT_STATUS_READY == DRV_USART_ClientStatus(appData.usartClient.handle))
                {
                    DRV_USART_BufferEventHandlerSet(appData.usartClient.handle,
                    (const DRV_USART_BUFFER_EVENT_HANDLER)APP_USARTBufferEventHandler,
                        (const uintptr_t)&appData.usartClient.context);
                }
                else
                {
                    SYS_DEBUG(0, "USART Driver Not Ready");
                }
                appData.tickSysTmrHandle = SYS_TMR_DelayMS(500);
                appData.state = APP_STATE_BT_SETUP_BLUETOOTHPORT;
//                bttask_pal_startBluetoothPort_2();
//                appData.state = APP_STATE_SETUP_BUTTON_TIMER;
            }
            else
            {
                ;
            }
        }
        break;

        case APP_STATE_BT_SETUP_BLUETOOTHPORT:
        {
            appData.tickSysTmrHandle = SYS_TMR_DelayMS(500);
            while (!SYS_TMR_DelayStatusGet(appData.tickSysTmrHandle))
            { // Wait for USART to boot
                asm("NOP");
            }            
            bttask_pal_startBluetoothPort_2();
            appData.state = APP_STATE_SETUP_BUTTON_TIMER;
            
//            if (true == SYS_TMR_DelayStatusGet(appData.tickSysTmrHandle))
//            {
//                bttask_pal_startBluetoothPort_2();
//                appData.state = APP_STATE_SETUP_BUTTON_TIMER;
//            }
        }
        break; 
        // Setting up the repeat timer 
        // --For servicing the bluetooth task
        case APP_STATE_SETUP_BUTTON_TIMER:
        {
            /* Open the repeat timer driver */
            if (SYS_STATUS_READY == DRV_TMR_Status(sysObj.drvTmr1))
            {
                appData.repeatTmrHandle = DRV_TMR_Open (DRV_TMR_INDEX_1,
                            DRV_IO_INTENT_EXCLUSIVE);
                if(DRV_HANDLE_INVALID == appData.repeatTmrHandle ||
                        (DRV_HANDLE) NULL == appData.repeatTmrHandle)
                {
                    SYS_DEBUG(0, "Timer DRV_TMR_Open Error");
                }
                else
                {
                    DRV_TMR_Start (appData.repeatTmrHandle);
                    DRV_TMR_AlarmRegister (appData.repeatTmrHandle,
                            APP_BT_BUTTON_REPEAT_TIMER_INIT_PERIOD, true,
                            (uintptr_t)0, &APP_BTRepeatTimerCallbackHandler);

    	    #ifdef ENABLE_SYS_LOG
    	    appData.state = APP_STATE_BT_SYS_LOG;
    	    #else
    	    appData.state = APP_STATE_BT_TASK_RUN;
    	    #endif
                }
            }
            else
            {
                SYS_DEBUG(0, "Timer Driver Not Ready");
            }
        }
        break;

        #ifdef ENABLE_SYS_LOG
        case APP_STATE_BT_SYS_LOG:
        {
            SYS_LOG_Task();
            appData.state = APP_STATE_BT_TASK_RUN;
        }
        break;
        #endif

        /* Run the Bluetooth Task */
        case APP_STATE_BT_TASK_RUN:
        {
            bluetoothTask();
            appData.state = APP_STATE_BUTTON_TASK_RUN;
        }
        break;

        /* Run the Button Task */
        case APP_STATE_BUTTON_TASK_RUN:
        {
            buttonsTask();
            appData.state = APP_STATE_SPP_TASK;
        }
        break;
        
        /* Run the SPP Task */
        case APP_STATE_SPP_TASK :
        {
            sppMsgHandlerTask();
            appData.state = APP_STATE_DISPLAY;
        }
        break;
        
        /* Run the Display Task */
        case APP_STATE_DISPLAY:
        {
            display_tasks(&BT_DISPLAY_STATS);

            #ifdef ENABLE_SYS_LOG
            appData.state = APP_STATE_BT_SYS_LOG;
            #else
            appData.state = APP_STATE_BT_TASK_RUN;
            #endif
        }
        break;

        default:
        {
        }
        break;
    }

} //End APP_Tasks())


/**********************************************************
 * Application USART buffer Event handler.
 * This function is called back by the USART driver when
 * a USART data buffer TX/RX completes.
 ***********************************************************/
void APP_USARTBufferEventHandler(DRV_USART_BUFFER_EVENT event,
        DRV_USART_BUFFER_HANDLE handle, uintptr_t context )
{
    switch(event)
    {
        case DRV_USART_BUFFER_EVENT_COMPLETE:
        {
            if(handle == appData.usartClient.writeBufHandle)
            {
                bttask_setSignal(BTTASK_SIG_TX);
            }
            if(handle == appData.usartClient.readBufHandle)
            {
                bttask_setSignal(BTTASK_SIG_RX);
            }
        }
        break;
        case DRV_USART_BUFFER_EVENT_ERROR:
        {
        } break;

        case DRV_USART_BUFFER_EVENT_ABORT:
        {
        } break;

    }
}


/**********************************************************
 * Application System BT Timer Tick Event handler.
 * This is a callback for the 10ms BT interrupt tick.
 ***********************************************************/
void APP_BTTimerTickCallbackHandler(uintptr_t context, uint32_t currTick)
{    APP_LED2_TOGGLE();
    bttimer_onSystemTick();
}


/**********************************************************
 * Application Repeat Timer Tick Event handler.
 * This is a callback for the Repeat TImer interrupt.
 ***********************************************************/
void APP_BTRepeatTimerCallbackHandler(uintptr_t context, uint32_t alarmCount )
{
    bttask_setSignal(BTTASK_SIG_BUTTON_REPEAT);
}

/**********************************************************
 * Application Read Core Timer
 * This function reads the core timer current value
 ***********************************************************/
uint32_t __attribute__((nomips16)) APP_ReadCoreTimer(void)
{
    uint32_t timer;
    // get the count reg
    asm volatile("mfc0   %0, $9" : "=r"(timer));
    return((uint32_t)timer);
}

/*******************************************************************************
 End of File
 */

