/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
     RN4871.h

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
Copyright (c) 2013-2017 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _EXAMPLE_FILE_NAME_H    /* Guard against multiple inclusion */
#define _EXAMPLE_FILE_NAME_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
/* This section lists the other files that are included in this file.
 */

void RN4871_Tasks(void);
void RN4871_Initialize(void); 
void TIMER_OBJECT_CALLBACK(uintptr_t context, uint32_t currTick);
void SEND_SENSOR_DATA(int value);
void TEST_COUNTER_OBJECT_CALLBACK(uintptr_t context, uint32_t currTick);

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

    
    
typedef enum
{
	/* Application's state machine's initial state. */
	DRV_STATE_INIT=0,
            DRV_STATE_OPEN_USART,
            DRV_STATE_RESET_MODULE_WAIT,
            DRV_STATE_REBOOT_DELAY,
            DRV_STATE_REBOOT_DELAY_WAIT,
            DRV_STATE_FACTORY_RESET,
            DRV_STATE_FACTORY_RESET_WAIT,
            DRV_STATE_SF_RB,
            DRV_STATE_SF_RB_WAIT,
            DRV_STATE_SET_NAME,
            DRV_STATE_NAME_WAIT,
            DRV_STATE_CLEAR_ALL_SERVICES,
            DRV_STATE_CLEAR_ALL_SERVICES_WAIT,
            DRV_STATE_SET_UUID,
            DRV_STATE_SET_UUID_WAIT,
            DRV_STATE_SET_CHARACTERISTIC,
            DRV_STATE_SET_CHARACTERISTIC_WAIT,
            DRV_STATE_SOFT_REBOOT,
            DRV_STATE_SOFT_REBOOT_WAIT,
            DRV_STATE_REENTER_COMMAND,
            DRV_STATE_REENTER_COMMAND_WAIT,
            DRV_STATE_NEW_READ,
            DRV_STATE_ADVERTISEMENT_CLEAR,
            DRV_STATE_ADVERTISEMENT_CLEAR_WAIT,
            DRV_STATE_ADVERTISEMENT_SET,
            DRV_STATE_ADVERTISEMENT_SET_WAIT,
            DRV_STATE_ADVERTISEMENT,
            DRV_STATE_ADVERTISEMENT_WAIT,
            DRV_STATE_LIST,
            DRV_STATE_LIST_WAIT,
            DRV_STATE_LIST_READ,
            DRV_STATE_LIST_READ_PROCESS,
            DRV_STATE_TOKEN,
            DRV_STATE_WAIT_FOR_CONNECTION,
            DRV_STATE_DEVICE,
            DRV_STATE_DEVICE_WAIT,
            DRV_STATE_REBOOT_DONE,
            DRV_STATE_COMMAND_WAIT,
    DRV_STATE_COMMAND_MODE,
            DRV_STATE_COMMAND_MODE_WAIT,
            DRV_STATE_SEND_DATA,
            DRV_STATE_SET_NAME_WAIT,
    DRV_STATE_SETUP_START,
    DRV_STATE_SETUP_RESTART,
    DRV_STATE_SETUP_WAIT,
    DRV_STATE_SETUP,
	DRV_STATE_SERVICE_TASKS,
    DRV_STATE_TEST1,
    DRV_STATE_IDLE,
    DRV_STATE_TEST,
            DRV_STATE_SETUP_RX,
            DRV_STATE_WAIT,
DRV_STATE_ERROR_REBOOT,
            
} DRV_RN4871_STATES;

typedef enum
{
    DRV_COMMAND_MODE_INIT=0,
    DRV_ENTER_COMMAND_MODE,
    DRV_WAIT_4_COMMAND_MODE,
            
}DRV_RN4871_COMMAND;

typedef struct
{
    /* The drivers current state */
    DRV_RN4871_STATES state;
    DRV_HANDLE drvUSARTHandle_Master;
    SYS_TMR_HANDLE sysTimerHandleMaster;
    SYS_TMR_HANDLE sysTimerHandleMaster2;
    DRV_USART_BUFFER_HANDLE  TXbufferHandle;
    DRV_USART_BUFFER_HANDLE  TXbufferHandle2;
    DRV_USART_BUFFER_HANDLE  RXbufferHandle;
    DRV_USART_BUFFER_HANDLE  RXbufferHandle2;
    DRV_RN4871_COMMAND commandState;
    volatile bool TXDone;
    volatile bool RXDone;
    bool CommsERROR;
    volatile bool CommsTimeOut;
    volatile bool SensorFlag;
} DRV_RN4871_DATA;

//extern DRV_RN4871_DATA drvData;

typedef enum
{
    DRV_RN4871_ERROR =0,
    DRV_RN4871_BUSY,
    DRV_RN4871_SUCESS,
} DRV_TX_STATUS;

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
