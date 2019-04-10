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
#include "bsp.h"


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

APP_DATA appData =
{
    /* App's state */
    .state = APP_STATE_BT_REGISTER_TICK_HANDLER,
};



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
    /* Bluetooth related Initializations */
    bluetoothInit();
    /* RGB LED Initializations */
    LEDinit();
    /* Temperature sensor Initializations */
    AccelInit();
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
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
                bttask_pal_startBluetoothPort_2();
                appData.state = APP_STATE_BT_TASK_RUN;
            }
            else
            {
                ;
            }
        }
        break;

        /* Run the Bluetooth Task Always */
        case APP_STATE_BT_TASK_RUN:
        {
            bluetoothTask();
        }
        break;

        default:
        {
        }
        break;
    }
}

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
        /* The Submitted BT USART data completed.
         * Indicate BT stack about completion */
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
        }
        break;

        case DRV_USART_BUFFER_EVENT_ABORT:
        {
        }
        break;
    }
}


/**********************************************************
 * Application System BT Timer Tick Event handler.
 * This is a callback for the 10ms BT interrupt tick.
 ***********************************************************/
void APP_BTTimerTickCallbackHandler(uintptr_t context, uint32_t currTick)
{
    bttimer_onSystemTick();
}


/**********************************************************
 * Application Repeat Timer Tick Event handler.
 * This is a callback for the Repeat TImer interrupt.
 ***********************************************************/
void APP_BTRepeatTimerCallbackHandler(void)
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

