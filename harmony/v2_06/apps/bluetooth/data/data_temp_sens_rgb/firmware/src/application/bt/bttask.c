/*******************************************************************************
    BT Task

  Company:
    Microchip Technology Inc.

  File Name:
    bttask.c

  Summary:
    Contains the functional implementation of BT Task.

  Description:
    This file contains the functional implementation of BT Task.
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

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Static/Local Variables
// *****************************************************************************
// *****************************************************************************
static unsigned char            mFlags;
//static bt_ulong                 mSamplingRate;
static bt_byte                  mSignalsFlag;
//static BTTASK_SIGNAL_HANDLER    mDisplaySignalHandler;
static bt_uint                  mAddrsAssigned;
static volatile bt_uint         mSignals;
static volatile bool            intStatus;
static volatile bt_uint         mBTTaskState;
static volatile bt_uint         mBTTaskCurrentState;

// *****************************************************************************
// *****************************************************************************
// Section: Static/Local Functions
// *****************************************************************************
// *****************************************************************************
static void writePersistentData(void);
static void writePersistentDataCallback(void);

// *****************************************************************************
// *****************************************************************************
// Section: Bluetooth Functions
// *****************************************************************************
// *****************************************************************************
void bluetoothInit(void)
{
    btSignalInit();
    btmgr_init();
    btapp_init();
    bttask_pal_initTimer();
    bttask_pal_initBluetoothPort();
    btmgr_start();
    bttask_setState(BTTASK_STATE_DEFAULT);
}

void bttask_setSignal(bt_uint signal)
{
    intStatus = SYS_INT_Disable();
    mSignals |= signal;
    mSignalsFlag = 1;
    if(intStatus)
    {
        SYS_INT_Enable();
    }
}

void bluetoothTask(void)
{
    bt_uint signals;
    
    mBTTaskCurrentState = bttask_getState();
    switch(mBTTaskCurrentState)
    {
        case BTTASK_STATE_INTERNAL_NVM_WRITE_START:
            if(APP_NVMIsWriteCompleted())
            {
                APP_NVMDisableOperation();
                bttask_setState(BTTASK_STATE_INTERNAL_NVM_WRITE_COMPLETE);
            }
            else
            {
            }
        break;

        case BTTASK_STATE_INTERNAL_NVM_WRITE_COMPLETE:
            if(APP_NVMIsVoltageError() || APP_NVMIsWriteOPerationTerminated())
            {
                /* Write Failed */
                while(1);
            }
            else
            {
                APP_btx_csr_set_ps_vars();
                bttask_setState(BTTASK_STATE_DEFAULT);
            }
        break;

        case BTTASK_STATE_DEFAULT:
            if(mSignalsFlag == 1)
            {
                intStatus = SYS_INT_Disable();
                mSignalsFlag = 0;
                signals = mSignals;
                mSignals = 0;
                if(intStatus)
                {
                    SYS_INT_Enable();
                }

                if (signals & BTTASK_SIG_BTSIGNAL)
                {
                    bt_signal_process_pending();
                }

                if (signals & BTTASK_SIG_RX)
                {
                    bttask_pal_handleRxSignal();
                }
                mBTTaskCurrentState = bttask_getState();
                if(mBTTaskCurrentState != BTTASK_STATE_DEFAULT)
                {
                    break;
                }

                if (signals & BTTASK_SIG_TX)
                {
                    bttask_pal_handleTxSignal();
                }

                if (signals & BTTASK_SIG_TIMER)
                {
                    bttask_pal_handleTimerSignal();
                    //btapp_spp_reconnect(NEWCONNECT_TICK, RECONNECT_TICK);
                }
            }
            break;
    }
}

void onBluetoothPortStarted(void)
{
    btapp_start();
}

void btSignalInit(void)
{
    mSignals = 0;
    mSignalsFlag = 0;
}

void btmgr_init(void)
{
    mFlags = 0;
}
// *****************************************************************************
// *****************************************************************************
// Section: Bluetooth Manager Functions
// *****************************************************************************
// *****************************************************************************
void btmgr_start(void)
{
    BTMGR_PERSISTENT_DATA* data;
    data = btmgr_pal_getPersistentData();
    // Check it is valid
    if (data->signature != SIGNATURE || data->nextSlot > BTMGR_MAX_LINK_KEYS - 1)
    {
        // Initialzie RAM copy if data in info memory is corrupt or not initialized.
        memset(data, 0, sizeof(BTMGR_PERSISTENT_DATA));
        data->signature = SIGNATURE;
    }
}

bt_bool btmgr_getLastConnectedDevice(bt_bdaddr_t* bdaddr)
{
    BTMGR_PERSISTENT_DATA* data = btmgr_pal_getPersistentData();

    memcpy(bdaddr, &data->lastConnectedDevice, sizeof(bt_bdaddr_t));
    return !bt_bdaddr_is_null(bdaddr);
}

void btmgr_setLastConnectedDevice(const bt_bdaddr_t* bdaddr)
{
    BTMGR_PERSISTENT_DATA* data = btmgr_pal_getPersistentData();

    memcpy(&data->lastConnectedDevice, bdaddr, sizeof(bt_bdaddr_t));

    mFlags |= FLAG_CHANGED;
    writePersistentData();
}

void btmgr_clearDeviceInfo(const bt_bdaddr_t* bdaddr)
{
    int i;
    BTMGR_PERSISTENT_DATA* data = btmgr_pal_getPersistentData();

    for (i = 0; i < BTMGR_MAX_LINK_KEYS; i++)
    {
        if (bt_bdaddrs_are_equal(&data->linkKeys[i].addr, (bt_bdaddr_t*)bdaddr))
        {
            memset(&data->linkKeys[i].addr, 0, sizeof(bt_bdaddr_t));
            memset(&data->linkKeys[i].linkKey, 0, sizeof(bt_linkkey_t));
            mFlags |= FLAG_CHANGED;
            writePersistentData();
            break;
        }
    }
}

void btmgr_clearAllDeviceInfo()
{
    BTMGR_PERSISTENT_DATA* data = btmgr_pal_getPersistentData();

    memset(data, 0, sizeof(BTMGR_PERSISTENT_DATA));
    data->signature = SIGNATURE;
    mFlags |= FLAG_CHANGED;
    writePersistentData();
}

void bttask_pal_setAddrsAssigned(void)
{
    mAddrsAssigned = 1;
}

static void writePersistentData(void)
{
    if ((mFlags & FLAG_CHANGED) && !(mFlags & FLAG_WRITING))
    {
        mFlags &= ~FLAG_CHANGED;
        mFlags |= FLAG_WRITING;
        btmgr_pal_writePersistentData(&writePersistentDataCallback);
    }
}

static void writePersistentDataCallback(void)
{
    mFlags &= ~FLAG_WRITING;

    // It is possible that data was changed while it was being written.
    // In this case we write it again.
    if (mFlags & FLAG_CHANGED)
    {
        writePersistentData();
    }
}


// *****************************************************************************
// *****************************************************************************
// Section: Bluetooth Signals
// *****************************************************************************
// *****************************************************************************
void bt_oem_schedule_signals(void)
{
    bttask_setSignal(BTTASK_SIG_BTSIGNAL);
}
// *****************************************************************************
// *****************************************************************************
// Section: Bluetooth Link Keys
// *****************************************************************************
// *****************************************************************************

void bt_oem_linkkey_notification(bt_linkkey_notification_t* lkn)
{
    int i;
    BTMGR_PERSISTENT_DATA* data = btmgr_pal_getPersistentData();

    // Look if we already have a key with the remote device.
    // If so, use that slot.
    for (i = 0; i < BTMGR_MAX_LINK_KEYS; i++)
    {
        if (bt_bdaddrs_are_equal(&data->linkKeys[i].addr, &lkn->bdaddr_remote))
        {
                break;
        }
    }

    if (i == BTMGR_MAX_LINK_KEYS)
    {
        // Take the next slot. This may override previously stored link key
        // for another device.
        i = data->nextSlot;
        data->nextSlot = (data->nextSlot + 1) % BTMGR_MAX_LINK_KEYS;
    }

    // Save the link key.
    data->linkKeys[i].addr = lkn->bdaddr_remote;
    memcpy(&data->linkKeys[i].linkKey, &lkn->key, sizeof(bt_linkkey_t));

    // Also save the BD address of the device we just paired with as the last
    // connected device address.
    btmgr_setLastConnectedDevice(&lkn->bdaddr_remote); // this also will call writePersistentData()

}


void bt_oem_linkkey_request(bt_linkkey_request_t* lkr)
{
    int i;
    BTMGR_PERSISTENT_DATA* data = btmgr_pal_getPersistentData();

    for (i = 0; i < BTMGR_MAX_LINK_KEYS; i++)
    {
        if (bt_bdaddrs_are_equal(&data->linkKeys[i].addr, &lkr->bdaddr_remote))
        {
            break;
        }
    }

    if (i < BTMGR_MAX_LINK_KEYS)
    {
        bt_hci_send_linkkey(&lkr->bdaddr_remote, &data->linkKeys[i].linkKey, NULL);
    }
    else
    {
        bt_hci_send_linkkey(&lkr->bdaddr_remote, NULL, NULL);
    }
}

void bttask_setState(BTTASK_STATE_MACHINE state)
{
    mBTTaskState = state;
}

BTTASK_STATE_MACHINE bttask_getState(void)
{
    return (BTTASK_STATE_MACHINE) mBTTaskState;
}
/*******************************************************************************
 End of File
 */

