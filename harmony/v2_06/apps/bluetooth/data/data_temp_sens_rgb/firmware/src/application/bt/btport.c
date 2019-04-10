/*******************************************************************************
    BT Serial Port

  Company:
    Microchip Technology Inc.

  File Name:
    btport.c

  Summary:
    Contains the functional implementation of BT Serial Port.

  Description:
    This file contains the functional implementation of BT Serial Port.
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

#include "app.h"

#if BT_CONTROLLER == BT_CONTROLLER_FLC_BTM805
// Receive FIFO buffer size
#define FIFO_SIZE                           32
// Interval between auto-baud packets (ms)
#define AUTO_BAUD_INTERVAL                  70
//
// Definitions for RTS/CTS flow control:
//
// When the number of free bytes in the receive buffer becomes
// less than FLOW_CTL_STOP_THRESHOLD the RTS line is set high
// to suspend transmission from the BT controller.
// When the number of free bytes in the receive buffer rises to
// FLOW_CTL_START_THRESHOLD the RTS line is set low to
// resume transmission from the controller.
#define FLOW_CTL_STOP_THRESHOLD             4
#define FLOW_CTL_START_THRESHOLD            8

/* Global variables */
static volatile bool            intStatus;
static bt_oem_recv_callback_fp  mRxCallback;
//static bt_byte*                 mRxBuffer;
//static bt_uint                  mRxIndex;
//static bt_byte                  mRxBufferBusy;
//static bt_byte                  mRxFifo[FIFO_SIZE];
//static bt_uint                  mRxFifoHead;
//static bt_uint                  mRxFifoTail;
//static bt_byte                  mStalled;
//static const bt_byte*           mTxBuffer;
//static bt_uint                  mTxLen;
//static bt_uint                  mTxIndex;
static volatile bool            intStatus;
static bt_uint                  mRxLen;
static bt_byte                  mRxSuspended;
static BTTASK_START_CALLBACK    mStartCallback;
static bt_oem_send_callback_fp  mTxCallback;
//static btport_StartCallback     driverStartCB;
static bt_oem_recv_callback_fp  mRxCallback;
static bt_byte                  mSleep;
static bt_uint                  mAddrsAssigned = 0;

// Static function prototypes.
static void selHostInterfaceCallback(bt_bool success, btx_csr_autobaud_buffer_t* buffer);
static void setPsVarsCallback(bt_bool success, btx_csr_set_ps_vars_buffer_t* buffer);
static void setBitRateCallback(bt_bool success, btx_csr_set_ps_vars_buffer_t* buffer);
static void hciStopCallback(void* param);
static void timerCallback(void);
static void warmResetCallback(bt_int status, bt_hci_command_t* cmd, bt_hci_event_t* evt);
static void btx_csr_set_bdaddr(const bt_uint* pbc7_ps_values_bdaddr);

// Persistent Store (PS) values for BC7 initialization.
static const bt_uint BC7_PS_VALUES[] __attribute__((address(PIC32_BC7_PS_VALUES_LOCATION))) =
{
    // Set reference clock frequency to 26 MHz.
    SET_PS_VALUE_UINT16(PSKEY_ANA_FREQ, 26000),
    // Turn VM off as it has no use fo us.
    SET_PS_VALUE_UINT16(PSKEY_VM_DISABLE, 1),
    // Do not consider levels on any PIO pins when deciding whether deep sleep is possible.
    SET_PS_VALUE_UINT32(PSKEY_PIO_DEEP_SLEEP_EITHER_LEVEL, 0xFFFFFFFF),
    // Use extrenal clock for deep sleep.
    // SET_PS_VALUE_UINT16(PSKEY_DEEP_SLEEP_USE_EXTERNAL_CLOCK, 1),
    // Set deep sleep state to:
    //    1 - The controller will go to deep sleep whenever possible.
    //    3 - The controller will go to deep sleep whenever possible
    //        and also it will assume that the external slow clock is
    //        at least 20 ppm accurate. This will result in faster wakeup.
    // For now it is set to 1.
    //SET_PS_VALUE_UINT16(PSKEY_DEEP_SLEEP_STATE, 1),
    // Clear RTS (i.e. set it high) when the controller goes to deep sleep.
    //SET_PS_KEY_UINT16(PSKEY_DEEP_SLEEP_CLEAR_RTS, 1),
    // Disable waking up on active level on the CTS line.
    //SET_PS_KEY_UINT16(PSKEY_DEEP_SLEEP_WAKE_CTS, 0),
    // Configure controller to indicate on PIO2 when the exrnal fast clock is
    // required. Note, this cannot be used to monitor when the controller goes
    // to deep sleep. Sometimes the clock is not required but the controller
    // is awake. More accurate signal to monitor deep sleep is the RTS pin
    // which was configured earlier.
    SET_PS_VALUE_UINT16(PSKEY_CLOCK_REQUEST_ENABLE, 2),
    // UART baud rate.
    // SET_PS_VALUE_UINT32(PSKEY_UART_BITRATE, BT_UART_WORKING_BAUD_RATE),
    // Set host interface to H4.
    SET_PS_VALUE_UINT16(PSKEY_HOST_INTERFACE, 3),  // H4
    // Configure UART parameters for H4 interface.
    SET_PS_VALUE_UINT16(PSKEY_UART_CONFIG_H4, 0x08A8),
    // Disable initial HCI_Command_Status event.
    SET_PS_VALUE_UINT16(PSKEY_HCI_NOP_DISABLE, 1),
    // Reset timeout on break level on controller's UART Rx line.
    SET_PS_VALUE_UINT32(PSKEY_HOSTIO_UART_RESET_TIMEOUT, 5000),
    // Set board address
    //SET_PS_VALUE_BDADDR(PSKEY_BDADDR, 0x1234, 0x56789077),
    SET_PS_VALUE_BDADDR(PSKEY_BDADDR, BTX_BT_ADDRESS_NAP, BTX_BT_ADDRESS_UAP_LAP),
    0x0000 // End of list
};

static bt_uint BC7_PS_VALUES_BITRATE[] =
{
    // UART baud rate.
    SET_PS_VALUE_UINT32(PSKEY_UART_BITRATE, 0),
    0x0000 // End of list
};

// Work buffers for btx_csr functions.
static union
{
    btx_csr_autobaud_buffer_t btx_csr_autobaud;
    btx_csr_exec_script_buffer_t btx_csr_exec_script;
    btx_csr_set_ps_vars_buffer_t btx_csr_set_ps_vars;
} mWorkBuffers;

// *****************************************************************************
// *****************************************************************************
// Section: Fuction Implementation
// *****************************************************************************
bt_byte bttask_pal_getAddrsAssigned(void)
{
    return mAddrsAssigned;
}

void bttask_pal_setetAddrsAssigned(void)
{
    mAddrsAssigned = 1;
}

void bttask_pal_initBluetoothPort(void)
{
    bt_ulong tmp, tmp2;
    unsigned long clock;
    mSleep = 0;
    mRxSuspended = 0;

    // Calculate actual working baud rate
    if (APP_BT_USART_WORKING_BAUD_RATE > (115200 * 2))
    {
        clock = APP_BT_USART_BAUD_CLOCK / 4;
        tmp2 = clock % APP_BT_USART_WORKING_BAUD_RATE;
        tmp = clock / APP_BT_USART_WORKING_BAUD_RATE - (tmp2 >= APP_BT_USART_WORKING_BAUD_RATE / 2 ? 0 : 1);
        tmp = clock / (tmp + 1);
    }
    else
    {
        clock = APP_BT_USART_BAUD_CLOCK / 16;
        tmp2 = clock % APP_BT_USART_WORKING_BAUD_RATE;
        tmp = clock / APP_BT_USART_WORKING_BAUD_RATE - (tmp2 >= APP_BT_USART_WORKING_BAUD_RATE / 2 ? 0 : 1);
        tmp = clock / (tmp + 1);
    }
    BC7_PS_VALUES_BITRATE[2] = (bt_uint)(tmp >> 16);
    BC7_PS_VALUES_BITRATE[3] = (bt_uint)(tmp & 0xFFFF);
}

void bttask_pal_startBluetoothPort_1(BTTASK_START_CALLBACK callback)
{
    assert(callback != NULL);
    mStartCallback = callback;
}

void bttask_pal_startBluetoothPort_2(void)
{
    // Initialize and start transport layer that will be used
    // for controller initialization.
    bt_hcitr_uart_init();
    bt_hcitr_uart_start();
    // Initialize the HCI layer but do not start it.
    // This is needed because the subsequent call to btx_csr_autobaud
    // is using HCI internal buffers.
    bt_hci_init();
    // Start BlueCore controller initialization.
    btx_csr_init();
    btx_csr_bc7_sel_host_interface_h4(&mWorkBuffers.btx_csr_autobaud,
                        AUTO_BAUD_INTERVAL, selHostInterfaceCallback, NULL);
}

void bttask_pal_handleRxSignal(void)
{
    mRxCallback(mRxLen);
}

void bttask_pal_handleTxSignal(void)
{
    mTxCallback();
}

void bt_oem_send(const bt_byte* buffer, bt_uint len, bt_oem_send_callback_fp callback)
{
    assert(len != 0 && buffer != 0 && callback != 0);
    intStatus = SYS_INT_Disable();
    {
        mTxCallback = callback;
        DRV_USART_BufferAddWrite(appData.usartClient.handle,
                &appData.usartClient.writeBufHandle, (void *)buffer, len);
    }
    if(intStatus)
    {
        SYS_INT_Enable();
    }
}

void bt_oem_recv(bt_byte* buffer, bt_uint len, bt_oem_recv_callback_fp callback)
{
    assert(len != 0 && buffer != 0 && callback != 0);
    intStatus = SYS_INT_Disable();
    {
        mRxLen = len;
        mRxCallback = callback;
        DRV_USART_BufferAddRead(appData.usartClient.handle,
            &appData.usartClient.readBufHandle, buffer, len);
    }
    if(intStatus)
    {
        SYS_INT_Enable();
    }
}

static void warmResetCallback(bt_int status, bt_hci_command_t* cmd, bt_hci_event_t* evt)
{
    while(DRV_USART_TRANSFER_STATUS_TRANSMIT_EMPTY !=
            (DRV_USART_TransferStatus(appData.usartClient.handle)
            & DRV_USART_TRANSFER_STATUS_TRANSMIT_EMPTY))
    {
        ;
    }
    // At this point the controller is reseting itself. While it is busy doing that
    // we reconfigure our UART to working speed.
    {
        DRV_USART_CLIENT_STATUS clientStatus;
        DRV_USART_Close(appData.usartClient.handle);
        if(DRV_USART_CLIENT_STATUS_CLOSED == DRV_USART_ClientStatus(
                                        appData.usartClient.handle))
        {
            appData.usartClient.handle = DRV_USART_Open(DRV_USART_INDEX_0,
                        DRV_IO_INTENT_READWRITE|DRV_IO_INTENT_NONBLOCKING);
            clientStatus = DRV_USART_ClientStatus(appData.usartClient.handle);
            if(DRV_USART_CLIENT_STATUS_READY == clientStatus)
            {
                DRV_USART_BufferEventHandlerSet(appData.usartClient.handle,
                (const DRV_USART_BUFFER_EVENT_HANDLER)APP_USARTBufferEventHandler,
                        (const uintptr_t)&appData.usartClient.context);
                DRV_USART_BaudSet(appData.usartClient.handle,APP_BT_USART_WORKING_BAUD_RATE);
            }
            else
            {
                SYS_DEBUG(0, "USART Driver Not Ready");
            }
        }
    }

    // We are going to wait for a short period of time that should be enough
    // for the controller to receive the warm reset command and reset.
    // Since the L2CAP layer is not yet functioning we can reuse one of its timers.
    bt_oem_timer_set(BT_TIMER_L2CAP, 200, &timerCallback);
}

static void timerCallback(void)
{
    // Stop the HCI layer as we do not need it any more.
    bt_hci_stop(&hciStopCallback, NULL);
}

static void hciStopCallback(void* param)
{
    // HCI layer is now stopped. Do some final cleanup and
    // and call the start callback. The application layer will
    // start the HCI and other stack layers.
    intStatus = SYS_INT_Disable();
    {
            mRxCallback = NULL;
            mTxCallback = NULL;
    }
    if(intStatus)
    {
        SYS_INT_Enable();
    }
    // Reinitialize transport.
    bt_hcitr_uart_reset();
    bt_hcitr_uart_start();
    mStartCallback();
}
#endif

static void btx_csr_set_bdaddr(const bt_uint* pbc7_ps_values_bdaddr)
{
    const bt_uint* btx_bt_lap_address = pbc7_ps_values_bdaddr;
    bt_ulong btx_bt_rand;
    uint32_t h8;
    uint16_t l4;

    h8 = (BC7_PS_VALUES[31] << 16) | (BC7_PS_VALUES[30] << 8) | BC7_PS_VALUES[28];
    l4 = BC7_PS_VALUES[29];
    if(h8 == BT_DEVICE_DESIGN_ID && l4 == BT_DEVICE_ID_2LSB)
    {
        if(BT_DEVICE_ID_2LSB_RANDOMIZE == 1)
        {
            srand(APP_ReadCoreTimer());
            btx_bt_rand = ((bt_ulong)(rand()>>8))<<16;
            APP_NVMwriteWord((uint32_t)btx_bt_lap_address, ((btx_bt_rand & 0xFFFF0000) | BTX_BT_ADDRESS_4B));
        }
        else
        {
            APP_btx_csr_set_ps_vars();
        }
    }
    else
    {
        bttask_pal_setAddrsAssigned();
        APP_btx_csr_set_ps_vars();
    }

}

static void selHostInterfaceCallback(bt_bool success, btx_csr_autobaud_buffer_t* buffer)
{
    if (success)
    {
        // At this point the controller has configured its UART and
        // we successfully sent a reset command and received a response.
        // Start the HCI layer but without executing its standard initialization
        // sequence as for now we are going to use it to send patches and
        // set configuration parameters. After that we will perform a warm reset and
        // start everything again.
        bt_hci_start_no_init();
        // CSR 8811 does not have any patches yet.
        // Set configuration parameters.
        btx_csr_set_bdaddr(&BC7_PS_VALUES[28]);
    }
    else
    {
        /* There is an Error. Execution stops here */
        SYS_INT_Disable();
        for (;;)
        {
            LEDColorSet(1,1,1);
        }
    }
}

void APP_btx_csr_set_ps_vars(void)
{
    btx_csr_set_ps_vars(BC7_PS_VALUES, &mWorkBuffers.btx_csr_set_ps_vars, setPsVarsCallback, NULL);
}

static void setPsVarsCallback(bt_bool success, btx_csr_set_ps_vars_buffer_t* buffer)
{
    assert(success);
    // Set module's baud rate.
    btx_csr_set_ps_vars(BC7_PS_VALUES_BITRATE, &mWorkBuffers.btx_csr_set_ps_vars, setBitRateCallback, NULL);

}

static void setBitRateCallback(bt_bool success, btx_csr_set_ps_vars_buffer_t* buffer)
{
    // Perform warm reset. The controller will reset but it will keep the
    // patches and configuration parameters we sent before. The btx_csr_warm_reset()
    // returns immediately after submitting the HCI command carrying the warm reset
    // command. The controller does not respond to this command as it starts
    // resetting immediately upon receiving the command. warmResetCallback is called
    // right after the command packet has been transmitted to the controller.
    btx_csr_warm_reset_ex(&warmResetCallback, NULL);
}

/*******************************************************************************
 End of File
 */