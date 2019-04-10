/*******************************************************************************
  BT task Interface

  Company:
    Microchip Technology Inc.

  File Name:
    bttask.h

  Summary:
    Contains the BT task Interface specific defintions and function prototypes.

  Description:
    This file contains the BT task Interface specific defintions and function
    prototypes.
*******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef __BTTASK_H_INCLUDED__
#define __BTTASK_H_INCLUDED__

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

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************
#define BTMGR_MAX_LINK_KEYS     10
#define SIGNATURE     			0x4826
#define FLAG_CHANGED  			0x01
#define FLAG_WRITING  			0x02

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
typedef struct _btmgr_PersistentData
{
    bt_uint      signature;
    bt_bdaddr_t  lastConnectedDevice;
    bt_uint      nextSlot;
    struct
    {
        bt_bdaddr_t  addr;
        bt_linkkey_t linkKey;
    } linkKeys[BTMGR_MAX_LINK_KEYS];
} BTMGR_PERSISTENT_DATA;

typedef enum _BTTASK_SIGNALS
{
    BTTASK_SIG_RX            = 0x0001,
    BTTASK_SIG_TX            = 0x0002,
    BTTASK_SIG_BTSIGNAL      = 0x0004,
    BTTASK_SIG_TIMER         = 0x0008,
    BTTASK_SIG_WAKEUP        = 0x0010,
    BTTASK_SIG_BUTTONS       = 0x0020,
    BTTASK_SIG_BUTTON_REPEAT = 0x0040,
    BTTASK_SIG_KEYBOARD      = 0x0080,
    BTTASK_SIG_STORAGE       = 0x0100,
    BTTASK_SIG_AUDIO         = 0x0200,
    BTTASK_SIG_ACP           = 0x0400,
    BTTASK_SIG_DISPLAY       = 0x0800

} BTTASK_SIGNALS;

typedef enum _BTTASK_STATE_MACHINE
{
    BTTASK_STATE_DEFAULT = 0x0,
    BTTASK_STATE_INTERNAL_NVM_WRITE_START = 0x1,
    BTTASK_STATE_INTERNAL_NVM_WRITE_COMPLETE = 0x2
} BTTASK_STATE_MACHINE;

typedef void (*BTTASK_START_CALLBACK)(void);
typedef void (*BTTASK_SIGNAL_HANDLER)(void);

// *****************************************************************************
// *****************************************************************************
// Section: Function Prototypes
// *****************************************************************************
// *****************************************************************************
void bluetoothInit(void);
void bttask_setSignal(bt_uint signal);
void bluetoothTask(void);
void onBluetoothPortStarted(void);
void btSignalInit(void);
void btmgr_init(void);
void btmgr_start(void);
bt_bool btmgr_getLastConnectedDevice(bt_bdaddr_t* bdaddr);
void btmgr_setLastConnectedDevice(const bt_bdaddr_t* bdaddr);
void btmgr_clearDeviceInfo(const bt_bdaddr_t* bdaddr);
void btmgr_clearAllDeviceInfo();
void bttask_pal_setAddrsAssigned(void);
void bt_oem_linkkey_notification(bt_linkkey_notification_t* lkn);
void bt_oem_linkkey_request(bt_linkkey_request_t* lkr);
void bt_oem_schedule_signals(void);
void bttask_setState(BTTASK_STATE_MACHINE);
BTTASK_STATE_MACHINE bttask_getState(void);

#ifdef __cplusplus
}
#endif

#endif // __BTTASK_H_INCLUDED__
/*******************************************************************************
 End of File
*/
