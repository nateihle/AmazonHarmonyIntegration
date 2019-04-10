/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef _APP_H
#define _APP_H

//// DOM-IGNORE-BEGIN
#ifdef __cplusplus
extern "C" {
#endif
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>

/* PIC32 related includes */
#include <p32xxxx.h>
#include <xc.h>

/* Harmony related Includes */
#include "system/system.h"
#include "system/debug/sys_debug.h"
#include "system_config.h"
#include "system_definitions.h"
#include "peripheral/nvm/plib_nvm.h"
#include "bsp.h"

/* BT stack related Includes */
#include "bluetooth/cdbt/bt/bt_std.h"
#include "bluetooth/cdbt/bt/bt_hcitr.h"
#include "bluetooth/cdbt/bt/bt_timer.h"
#include "bluetooth/cdbt/hci/hci.h"
#include "bluetooth/cdbt/hci/hcitr_uart.h"
#include "bluetooth/cdbt/extra/csr/csr.h"
#include "bluetooth/cdbt/bt/bt_system.h"
#include "bluetooth/cdbt/spp/spp.h"
#include "bluetooth/cdbt/hci/baseband.h"
#include "bluetooth/cdbt/bt/bt_signal.h"
#include "bluetooth/cdbt/bt/bt_oem.h"

/* BT App and profile related Includes */
#include "btapp_spp_sdpdb.h"
#include "btapp_device.h"

/* BT Configuration related Includes */
#include "bttimer_config.h"
#include "btcontroller.h"

/* BT Task related Includes */
#include "btport.h"
#include "bttask.h"
#include "btstorage.h"
#include "bttimer.h"

#include "display.h"

/* User configuration of BT Task */
#include "user_config.h"

/* Button Application */
#include "buttons.h"

/* Application specific NVM functions */
#include "app_nvm.h"

/* Error and assert support */
#include "app_error.h"

#ifdef ENABLE_SYS_LOG
#include "app_queue.h"
#endif
    
// Link state
#define BTLINK_STATE_IDLE        0
#define BTLINK_STATE_CONNECTING  1
#define BTLINK_STATE_CONNECTED   2

#define APP_BT_USART_BAUD_CLOCK                         (uint32_t) SYS_PBCLK_CLOCK_HZ
#define APP_BT_USART_WORKING_BAUD_RATE                  4000000
/* Peripheral Bus Clock frequency */
#define SYS_PBCLK_CLOCK_HZ                              SYS_CLK_BUS_PERIPHERAL_1

/* BT Tick Timer1 configuration settings */
#define SYS_BT_TICK_TIMER_PRESCALE                      TMR_PRESCALE_VALUE_8
#define SYS_BT_TICK_TIMER_PRESCALE_VAL                  8
#define APP_BT_TICK_TIMER_MS                            10
#define SYS_BT_TICK_TIMER_RATE_HZ                       (uint32_t)100
#define SYS_BT_TICK_TIMER_PERIOD                        (uint32_t)(((SYS_PBCLK_CLOCK_HZ/SYS_BT_TICK_TIMER_PRESCALE_VAL) / SYS_BT_TICK_TIMER_RATE_HZ) - 1)


/* BT Repeat Timer configuration settings */
#define SYS_BT_BUTTON_REPEAT_TIMER_PRESCALE          TMR_PRESCALE_VALUE_256
#define SYS_BT_BUTTON_REPEAT_TIMER_INIT_PERIOD       (SYS_PBCLK_CLOCK_HZ/SYS_BT_BUTTON_REPEAT_TIMER_PRESCALE/2)
#define APP_BT_BUTTON_REPEAT_TIMER_INIT_PERIOD       SYS_BT_BUTTON_REPEAT_TIMER_INIT_PERIOD
#define APP_BT_BUTTON_REPEAT_TIMER_REPEAT_PERIOD     (SYS_PBCLK_CLOCK_HZ/SYS_BT_BUTTON_REPEAT_TIMER_PRESCALE/128)


/* BT USART Baud Rate Settings */    
typedef struct _bt_data_port
{
    bt_spp_port_t* port;
    char mConnected;
    char mSending;
    char mSendToThisDevice;
    char mReceiving;
    char mState;
    char active;
    bt_bdaddr_t* mBtAddress;
    void* mRxBuffer;
    char bootloaderEnable;
}bt_data_port;
    
/* Prototypes from parsing_engine.c */
void TakeAction(char rcvdStr[]);

/* Prototypes from btapp_spp.c */
void btapp_spp_reconnect(bt_uint newConnectTick, bt_uint reConnectTick);

/* Prototypes from btport.c */
void bttask_pal_startBluetoothPort_1(BTTASK_START_CALLBACK callback);
void bttask_pal_startBluetoothPort_2(void);

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************
#define APP_BT_PAIRING_STORAGE_SUPPORTED                false

/* BT module reset duration */
#define APP_BT_MODULE_RESET_DURATION_MS                  50

#define BTAPP_MAX_SESSIONS              7

#define BTAPP_DID_VENDOR_ID_SOURCE      0x0001
#define BTAPP_DID_VENDOR_ID             0x0111
#define BTAPP_DID_PRODUCT_ID            0x0001
#define BTAPP_DID_VERSION               0x0001

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application States

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
    APP_STATE_BT_REGISTER_TICK_HANDLER,
    APP_STATE_BT_RESET_MODULE_START,
    APP_STATE_BT_RESET_MODULE_PROCESSING,
    APP_STATE_BT_SETUP_USART_DRIVER,
    APP_STATE_SETUP_BUTTON_TIMER,
#ifdef ENABLE_SYS_LOG
    APP_STATE_BT_SYS_LOG,
#endif
            APP_STATE_AUDIO_CODEC_OPEN,
            APP_SUBMIT_INITIAL_CODEC_READ_REQUEST,
            APP_SENDING_VOICE_DATA,
    APP_STATE_BT_TASK_RUN,
    APP_STATE_BUTTON_TASK_RUN,
    APP_STATE_ERROR,
    APP_STATE_BT_SETUP_BLUETOOTHPORT,
    APP_STATE_DISPLAY,
            APP_STATE_SPP_TASK
} APP_STATES;


// *****************************************************************************
/* Application USART client for BT

  Summary:
    Application USART client for BT.

  Description:
    This object holds the BT USART's client handle, read and write buffer handle
    created and the context
*/
typedef struct
{
    DRV_HANDLE handle;
    DRV_USART_BUFFER_HANDLE readBufHandle;
    DRV_USART_BUFFER_HANDLE writeBufHandle;
    uintptr_t context;
} APP_USART_CLIENT;

typedef struct
{
    DRV_HANDLE handle;
    DRV_CODEC_BUFFER_HANDLE bufHandle1;
    DRV_CODEC_BUFFER_HANDLE bufHandle2;    
    DRV_CODEC_BUFFER_EVENT_HANDLER bufferHandler;
    uintptr_t context;
    uint8_t *bufferObject1;
    uint8_t *bufferObject2;
    uint32_t bufferSize;
    bool isCodecReadComplete1;
    bool isCodecReadComplete2;
} APP_CODEC_CLIENT;
// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* Application's current state*/
    APP_STATES state;
    APP_CODEC_CLIENT codecClientRead;

    /* USART client handle */
    APP_USART_CLIENT usartClient;

    /* delay ms handle */
    SYS_TMR_HANDLE delayMSHandle;

    /* BT tick timer handle */
    SYS_TMR_HANDLE tickSysTmrHandle;

    /* BT repeat timer handle */
    DRV_HANDLE repeatTmrHandle;
    bool sppConnected;
} APP_DATA;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the
    application in its initial state and prepares it to run so that its
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void APP_Initialize ( void );


/*******************************************************************************
  Function:
    void APP_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void APP_Tasks ( void );

/*******************************************************************************
  Function:
    void APP_USARTBufferEventHandler(DRV_USART_BUFFER_EVENT event,
        DRV_USART_BUFFER_HANDLE handle, uintptr_t context )

  Summary:
    Event Handler for USART Task.

  Description:
    This is the Event Handler for USART Tx and Rx Complete Events.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
*/
void APP_USARTBufferEventHandler(DRV_USART_BUFFER_EVENT event,
        DRV_USART_BUFFER_HANDLE handle, uintptr_t context );

/*******************************************************************************
  Function:
    void APP_BTTimerTickCallbackHandler(uintptr_t context, uint32_t currTick)

  Summary:
    Event Handler for BT Tick Task.

  Description:
    This is the Event Handler for BT Tick 10ms handler.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    context         - A client parameter that's passed in the callback function.
                        This will help to identify the callback.
    currTick        - The current system tick when the notification is called.

  Returns:
    None.

  Example:
    <code>
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
*/
void APP_BTTimerTickCallbackHandler(uintptr_t context, uint32_t currTick);

/*******************************************************************************
  Function:
    void APP_BTRepeatTimerCallbackHandler(uintptr_t context, uint32_t alarmCount)

  Summary:
    Event Handler for BT Repeat timer .

  Description:
    This is the Event Handler for BT Repeat timer .

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    context - User specific context
    alarmCount - The current alarmCount when the notification is called.
 *
  Returns:
    None.

  Example:
    <code>
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
*/
void APP_BTRepeatTimerCallbackHandler(uintptr_t context, uint32_t alarmCount);

/*******************************************************************************
  Function:
    uint32_t __attribute__((nomips16)) APP_ReadCoreTimer(void);

  Summary:
    Read System Core Timer Value.

  Description:
    Read System Core Timer Value.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None

  Returns:
    uint32_t - Current core timer tick value

  Example:
    <code>
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
*/
uint32_t __attribute__((nomips16)) APP_ReadCoreTimer(void);

// *****************************************************************************
// *****************************************************************************
// Section: extern declarations
// *****************************************************************************
// *****************************************************************************
extern APP_DATA appData;
extern void onBluetoothPortStarted(void);



#endif /* _APP_H */
#ifdef __cplusplus
}
#endif

/*******************************************************************************
 End of File
 */

