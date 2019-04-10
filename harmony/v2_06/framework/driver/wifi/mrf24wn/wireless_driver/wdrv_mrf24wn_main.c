/*******************************************************************************
  MRF24WN Wireless Driver

  File Name:
    wdrv_mrf24wn_main.c

  Summary:
    Wireless driver PIC32 Implementation for MRF24WN support.

  Description:
    Wireless driver PIC32 Implementation for MRF24WN support.
 *******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc. All rights reserved.

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

#include "osal/osal.h"
#include "system/tmr/sys_tmr.h"
#include "tcpip/tcpip.h"
#include "tcpip/src/link_list.h"
#include "tcpip/src/tcpip_manager_control.h"
#include "tcpip/src/tcpip_packet.h"

#include "driver/wifi/mrf24wn/include/wdrv_mrf24wn_api.h"
#include "driver/wifi/mrf24wn/wireless_driver/include/wdrv_mrf24wn_main.h"

#define TCPIP_THIS_MODULE_ID TCPIP_MODULE_MAC_MRF24WN

#define ETH_HEADER_SIZE 14
#define NUM_PREALLOCATED_RX_PACKETS 4
#define MAX_IP_PACKET_SIZE 1564 // including header
#define MAX_RX_PACKET_SIZE 1518
#define MAX_TX_PACKET_SIZE 1518
#define MAX_MULTICAST_FILTER_SIZE 16

#define WAIT_FOR_DISCONNECT_COMPLETE() WDRV_SemTake(&g_wdrv_priv.disconnectDoneSync, OSAL_WAIT_FOREVER)

// used to keep track of Rx packets queued for stack
typedef struct
{
    int front;
    int rear;
    TCPIP_MAC_PACKET *items[NUM_PREALLOCATED_RX_PACKETS + 1];
} WDRV_MRF24WN_RX_FIFO;

// MRF24WN wireless driver descriptor
typedef struct
{
    const TCPIP_MAC_OBJECT *pObj; // safe cast to TCPIP_MAC_DCPT
    TCPIP_NET_IF *pNetIf;         // interface we belong to
    bool isInit;                  // simple init status flag
    bool isOpen;                  // simple open status flag
    SYS_STATUS sysStat;           // driver status
} WDRV_MRF24WN_DCPT;

OSAL_MUTEX_HANDLE_TYPE *g_debugConsoleLock;
WDRV_MRF24WN_PRIV g_wdrv_priv =
{
	/* explicity initialize .initConn and .isDisconnectRequested to true */
	.initConn = true,
	.isDisconnectRequested = true
};
WDRV_SCAN_STATUS g_wdrv_scanStatus;

// Only one hardware instance for now!
static WDRV_MRF24WN_DCPT wdrv_mrf24wn_dcpt =
{
	&WDRV_MRF24WN_MACObject,	// specific PIC32 MAC data
	NULL,						// pNetIf
	false,						// isInit
	false,						// isOpen
	SYS_STATUS_UNINITIALIZED,	// sysStat
};
static uint32_t s_wdrvext_priv_default[] = {MODULE_EVENT_PRINT, WDRV_EXT_RTOS_INIT_TASK_PRIORITY, WDRV_EXT_RTOS_MAIN_TASK_PRIORITY,
    WDRV_EXT_RTOS_INIT_TASK_SIZE, WDRV_EXT_RTOS_MAIN_TASK_SIZE, WDRV_BOARD_TYPE};
static bool s_isInitComplete = false;
static TCPIP_MAC_MODULE_CTRL s_macCtrl;
static TCPIP_MAC_PKT_AllocF s_pktAllocF = 0;
static TCPIP_MAC_PKT_FreeF s_pktFreeF = 0;
static TCPIP_MAC_PKT_AckF s_pktAckF = 0;
static WDRV_MRF24WN_RX_FIFO s_rxFifo;
static SINGLE_LIST s_rxQueue; // queue of Rx packets waiting to be processed by stack from host
static TCPIP_MAC_PACKET *s_rxPacketBuffer[NUM_PREALLOCATED_RX_PACKETS];
static TCPIP_MAC_ADDR s_MulticastFilter[MAX_MULTICAST_FILTER_SIZE];
static OSAL_MUTEX_HANDLE_TYPE *s_rxFifoLock;
static OSAL_MUTEX_HANDLE_TYPE *s_rxQueueLock;
static OSAL_MUTEX_HANDLE_TYPE *s_multicastFilterLock;

TCPIP_MAC_RES WDRV_MRF24WN_MulticastFilterSet(TCPIP_MAC_HANDLE hMac, const TCPIP_MAC_ADDR *DestMACAddr);

static SYS_MODULE_OBJ WDRV_MRF24WN_Initialize(const SYS_MODULE_INDEX index, const SYS_MODULE_INIT *const init);
static void WDRV_MRF24WN_Reinitialize(SYS_MODULE_OBJ object, const SYS_MODULE_INIT *const init);
static void WDRV_MRF24WN_Deinitialize(SYS_MODULE_OBJ object);
static void WDRV_MRF24WN_Tasks(SYS_MODULE_OBJ object);
static SYS_STATUS WDRV_MRF24WN_Status(SYS_MODULE_OBJ object);
static TCPIP_MAC_HANDLE WDRV_MRF24WN_Open(const SYS_MODULE_INDEX drvIndex, const DRV_IO_INTENT intent);
static void WDRV_MRF24WN_Close(TCPIP_MAC_HANDLE hMac);
static TCPIP_MAC_RES WDRV_MRF24WN_RegisterStatisticsGet(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_STATISTICS_REG_ENTRY *pRegEntries,
        int nEntries, int *pHwEntries);
static bool WDRV_MRF24WN_CheckLink(TCPIP_MAC_HANDLE hMac);
static bool WDRV_MRF24WN_PowerMode(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_POWER_MODE pwrMode);
static TCPIP_MAC_RES WDRV_MRF24WN_TxPacket(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_PACKET *ptrPacket);
static TCPIP_MAC_PACKET *WDRV_MRF24WN_RxPacket (TCPIP_MAC_HANDLE hMac, TCPIP_MAC_RES *pRes,
        const TCPIP_MAC_PACKET_RX_STAT **ppPktStat);
static TCPIP_MAC_RES WDRV_MRF24WN_Process(TCPIP_MAC_HANDLE hMac);
static TCPIP_MAC_RES WDRV_MRF24WN_StatisticsGet(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_RX_STATISTICS *pRxStatistics,
        TCPIP_MAC_TX_STATISTICS *pTxStatistics);
static TCPIP_MAC_RES WDRV_MRF24WN_ParametersGet(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_PARAMETERS *pMacParams);
static bool WDRV_MRF24WN_EventSetMask(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT macEvents, bool enable);
static bool WDRV_MRF24WN_EventAcknowledge(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT macEvents);
static TCPIP_MAC_EVENT WDRV_MRF24WN_EventGet(TCPIP_MAC_HANDLE hMac);
static size_t WDRV_MRF24WN_GetConfig(TCPIP_MODULE_MAC_ID modId, void *configBuff, size_t buffSize,
        size_t *pConfigSize);
static void FifoInit(WDRV_MRF24WN_RX_FIFO *const p_fifo);
static bool isFifoEmpty(WDRV_MRF24WN_RX_FIFO *const p_fifo);
static void FifoInsert(WDRV_MRF24WN_RX_FIFO *const p_fifo, TCPIP_MAC_PACKET *p_packet);
static TCPIP_MAC_PACKET *FifoRemove(WDRV_MRF24WN_RX_FIFO *const p_fifo);
static TCPIP_MAC_PACKET *GetRxPacket(void);
static bool RxPacketAckFunc(TCPIP_MAC_PACKET *pktHandle, const void *ackParam);
static void RxPacketBufferInit(void);
static void RxPacketBufferDeinit(void);

const TCPIP_MAC_OBJECT WDRV_MRF24WN_MACObject =
{
    TCPIP_THIS_MODULE_ID,
    "MRF24WN",
    WDRV_MRF24WN_Initialize,
    WDRV_MRF24WN_Deinitialize,
    WDRV_MRF24WN_Reinitialize,
    WDRV_MRF24WN_Status,
    WDRV_MRF24WN_Tasks,
    WDRV_MRF24WN_Open,
    WDRV_MRF24WN_Close,
    WDRV_MRF24WN_CheckLink,
    WDRV_MRF24WN_MulticastFilterSet,
    WDRV_MRF24WN_PowerMode,
    WDRV_MRF24WN_TxPacket,
    WDRV_MRF24WN_RxPacket,
    WDRV_MRF24WN_Process,
    WDRV_MRF24WN_StatisticsGet,
    WDRV_MRF24WN_ParametersGet,
    WDRV_MRF24WN_RegisterStatisticsGet,
    WDRV_MRF24WN_GetConfig,
    WDRV_MRF24WN_EventSetMask,
    WDRV_MRF24WN_EventAcknowledge,
    WDRV_MRF24WN_EventGet,
};

static void RFReadyCB(const uint8_t *const addr)
{
    g_wdrv_priv.updateMacAddressRequired = 1;
    memcpy(g_wdrv_priv.macAddr, addr, 6);
}

static void ScanDoneCB(uint32_t status)
{
    if (status == 0) { // 0 means success
        WDRV_EXT_ScanDoneSet();
        WDRV_EXT_CmdScanGet(&g_wdrv_scanStatus.numberOfResults);
        g_wdrv_priv.isScanDone = true;
        g_wdrv_scanStatus.scanInProgress = false;
        WDRV_DBG_INFORM_PRINT("Scan is completed successfully\r\n");
    } else {
        WDRV_DBG_INFORM_PRINT("Scan failed, status = %d\r\n", status);
    }
}

static void InitDoneCB(void)
{
	s_isInitComplete = true;
}

static TCPIP_MAC_RES WDRV_MRF24WN_RegisterStatisticsGet(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_STATISTICS_REG_ENTRY *pRegEntries,
        int nEntries, int *pHwEntries)
{
    /* unsupported */
    return TCPIP_MAC_RES_OP_ERR;
}

static void WDRV_MRF24WN_Close(TCPIP_MAC_HANDLE hMac)
{
	WDRV_MRF24WN_DCPT *pDcpt = (WDRV_MRF24WN_DCPT *)hMac;
	if (pDcpt->isOpen) {
		if (isLinkUp()) {
			WDRV_Disconnect(true);
			WAIT_FOR_DISCONNECT_COMPLETE();
		}
		g_wdrv_priv.isDriverOpen = false;
		pDcpt->isOpen = false;
	}
}

static bool WDRV_MRF24WN_CheckLink(TCPIP_MAC_HANDLE hMac)
{
    return isLinkUp();
}

static void MulticastFilter_Initialize(void)
{
    int i;
    static bool firstTime = true;

    if (firstTime) {
        firstTime = false;
        for (i = 0; i < MAX_MULTICAST_FILTER_SIZE; ++i) {
            memset(&s_MulticastFilter[i], 0, sizeof(TCPIP_MAC_ADDR));
        }
    }
}

static bool isMulticastAddrSet(const TCPIP_MAC_ADDR *addr)
{
    int i;

    for (i = 0; i < MAX_MULTICAST_FILTER_SIZE; i++) {
        if (memcmp(addr, &s_MulticastFilter[i], 6) == 0)
            return true;
    }

    return false;
}

static void MulticastAddr_Set(const TCPIP_MAC_ADDR *addr, int index)
{
     memcpy(&s_MulticastFilter[index], addr, 6);
}

TCPIP_MAC_RES WDRV_MRF24WN_MulticastFilterSet(TCPIP_MAC_HANDLE hMac, const TCPIP_MAC_ADDR *DestMACAddr)
{
    int i;
    TCPIP_MAC_RES res;
    uint8_t all_zeros[6] = {0, 0, 0, 0, 0, 0};

    WDRV_MUTEX_LOCK(s_multicastFilterLock, OSAL_WAIT_FOREVER);

    do {
        if (isMulticastAddrSet(DestMACAddr)) {
            res = TCPIP_MAC_RES_OK;
            break;
        }

        for (i = 0; i < MAX_MULTICAST_FILTER_SIZE; i++) {
            if (memcmp(all_zeros, &s_MulticastFilter[i], 6) == 0)
                break;
        }

        if (i == MAX_MULTICAST_FILTER_SIZE) {
            WDRV_ASSERT(false, "Multicast filter is full");
            res = TCPIP_MAC_RES_OP_ERR;
            break;
        }

        MulticastAddr_Set(DestMACAddr, i);
        res = TCPIP_MAC_RES_OK;
    } while(0);

    WDRV_MUTEX_UNLOCK(s_multicastFilterLock);

    return res;
}

static bool WDRV_MRF24WN_PowerMode(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_POWER_MODE pwrMode)
{
    if (pwrMode == TCPIP_MAC_POWER_FULL)
        WDRV_GPIO_PowerOn();
    else if (pwrMode == TCPIP_MAC_POWER_DOWN)
        WDRV_GPIO_PowerOff();
    else
        WDRV_DBG_INFORM_PRINT("This power Mode is not supported by the current MRF24WN driver\r\n");

    return true;
}

static TCPIP_MAC_RES WDRV_MRF24WN_TxPacket(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_PACKET *ptrPacket)
{
    TCPIP_MAC_DATA_SEGMENT *p_seg;
    TCPIP_MAC_RES res = TCPIP_MAC_RES_OK;
    uint8_t txPacket[MAX_IP_PACKET_SIZE];
    uint8_t *p_segData;
    uint16_t curIndex = 0;
    uint32_t sendResult;

    if (WDRV_IsPowerOff())
    {
        WDRV_DBG_INFORM_PRINT("MRF24WN is turned off, dropped the Tx packet\r\n");
        return TCPIP_MAC_RES_QUEUE_TX_FULL;
    }

    if (isLinkUp() == false) {
        WDRV_DBG_INFORM_PRINT("MRF24WN is in unconnected state, dropped the Tx packet\r\n");
        res = TCPIP_MAC_RES_PACKET_ERR;
        // call stack Ack function to let it know packet was transmitted
        if (s_pktAckF)
            _TCPIP_PKT_ACK_FNC(ptrPacket, TCPIP_MAC_PKT_ACK_TX_OK, TCPIP_THIS_MODULE_ID);
        else
            WDRV_ASSERT(false, "Should never happen");
        return res;
    }

    // write out the first segment to MRF24WN, including prepended internal header
    p_seg = ptrPacket->pDSeg; // point to first segment
    p_segData = p_seg->segLoad;

    if (p_seg->segLen > MAX_TX_PACKET_SIZE) {
        WDRV_DBG_ERROR_PRINT("Invalid packet length %d, dropped the Tx packet\r\n", p_seg->segLen);
        res = TCPIP_MAC_RES_PACKET_ERR;
        // call stack Ack function to let it know packet was transmitted
        if (s_pktAckF)
            _TCPIP_PKT_ACK_FNC(ptrPacket, TCPIP_MAC_PKT_ACK_TX_OK, TCPIP_THIS_MODULE_ID);
        else
            WDRV_ASSERT(false, "Should never happen");
        return res;
    }

    if (p_seg->next == NULL) {
        if (p_seg->segLen > MAX_TX_PACKET_SIZE) {
            WDRV_DBG_ERROR_PRINT("Invalid packet length %d, dropped the TX packet\r\n", p_seg->segLen);
            res = TCPIP_MAC_RES_PACKET_ERR;
            // call stack Ack function to let it know packet was transmitted
            if (s_pktAckF)
                _TCPIP_PKT_ACK_FNC(ptrPacket, TCPIP_MAC_PKT_ACK_TX_OK, TCPIP_THIS_MODULE_ID);
            else
                WDRV_ASSERT(false, "Should never happen");
            return res;
        }
        sendResult = WDRV_EXT_DataSend(p_seg->segLen, p_segData);
    } else {
        memcpy(txPacket, p_segData, p_seg->segLen);
        curIndex += p_seg->segLen;
        while (p_seg->next != NULL) {
            if (curIndex > MAX_TX_PACKET_SIZE) {
                WDRV_DBG_ERROR_PRINT("Invalid packet length %d, dropped the TX packet\r\n", curIndex);
                res = TCPIP_MAC_RES_PACKET_ERR;
                // call stack Ack function to let it know packet was transmitted
                if (s_pktAckF)
                    _TCPIP_PKT_ACK_FNC(ptrPacket, TCPIP_MAC_PKT_ACK_TX_OK, TCPIP_THIS_MODULE_ID);
                else
                    WDRV_ASSERT(false, "Should never happen");
                return res;
            }
            p_seg = p_seg->next;
            p_segData = p_seg->segLoad;
            memcpy(txPacket + curIndex, p_segData, p_seg->segLen);
            curIndex += p_seg->segLen;
        }
        sendResult = WDRV_EXT_DataSend(curIndex, txPacket);
    }

    if (sendResult != 0) {
        res = TCPIP_MAC_RES_PACKET_ERR;
        WDRV_DBG_TRACE_PRINT("No Tx buffer is available, dropped the packet\r\n");
    } else {
        WDRV_DBG_TRACE_PRINT("Sent packet to module\r\n");
    }

    // call stack Ack function to let it know packet was transmitted
    if (s_pktAckF)
        _TCPIP_PKT_ACK_FNC(ptrPacket, TCPIP_MAC_PKT_ACK_TX_OK, TCPIP_THIS_MODULE_ID);
    else
        WDRV_ASSERT(false, "Should never happen");
    return res;
}

static TCPIP_MAC_PACKET *WDRV_MRF24WN_RxPacket (TCPIP_MAC_HANDLE hMac, TCPIP_MAC_RES *pRes,
    const TCPIP_MAC_PACKET_RX_STAT **ppPktStat)
{
    WDRV_DBG_TRACE_PRINT("Received packet from module\r\n");
    return GetRxPacket();
}

static TCPIP_MAC_RES WDRV_MRF24WN_Process(TCPIP_MAC_HANDLE hMac)
{
    /* unsupported */
    return TCPIP_MAC_RES_OK;
}

static TCPIP_MAC_RES WDRV_MRF24WN_StatisticsGet(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_RX_STATISTICS *pRxStatistics,
    TCPIP_MAC_TX_STATISTICS *pTxStatistics)
{
    /* unsupported */
    return TCPIP_MAC_RES_OP_ERR;
}

static TCPIP_MAC_RES WDRV_MRF24WN_ParametersGet(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_PARAMETERS *pMacParams)
{
    WDRV_MRF24WN_DCPT *pDcpt = (WDRV_MRF24WN_DCPT *)hMac;

    if (pDcpt->sysStat == SYS_STATUS_READY) {
        if (pMacParams) {
            memcpy(pMacParams->ifPhyAddress.v, s_macCtrl.ifPhyAddress.v, sizeof(pMacParams->ifPhyAddress));
            pMacParams->processFlags = (TCPIP_MAC_PROCESS_FLAG_RX | TCPIP_MAC_PROCESS_FLAG_TX);
            pMacParams->macType = TCPIP_MAC_TYPE_WLAN;
        }
        return TCPIP_MAC_RES_OK;
    }
    return TCPIP_MAC_RES_IS_BUSY;
}

static bool WDRV_MRF24WN_EventSetMask(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT macEvents, bool enable)
{
    return WDRV_TrafficEventMask(hMac, macEvents, enable);
}

static bool WDRV_MRF24WN_EventAcknowledge(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT macEvents)
{
    return WDRV_TrafficEventAck(hMac, macEvents);
}

static TCPIP_MAC_EVENT WDRV_MRF24WN_EventGet(TCPIP_MAC_HANDLE hMac)
{
    return  WDRV_TrafficEventGet(hMac);
}

static void FifoInit(WDRV_MRF24WN_RX_FIFO *const p_fifo)
{
    memset(p_fifo, 0x00, sizeof(WDRV_MRF24WN_RX_FIFO));
    p_fifo->front = p_fifo->rear = NUM_PREALLOCATED_RX_PACKETS - 1;
}

static bool isFifoEmpty(WDRV_MRF24WN_RX_FIFO *const p_fifo)
{
    return p_fifo->front == p_fifo->rear;
}

static bool isFifoFull(WDRV_MRF24WN_RX_FIFO *const p_fifo)
{
    if ((p_fifo->front == 0) && (p_fifo->rear == NUM_PREALLOCATED_RX_PACKETS - 1))
        return true;
    else if (p_fifo->front == p_fifo->rear + 1)
        return true;
    else
        return false;
}

static void FifoInsert(WDRV_MRF24WN_RX_FIFO *const p_fifo, TCPIP_MAC_PACKET *p_packet)
{
    WDRV_MUTEX_LOCK(s_rxFifoLock, OSAL_WAIT_FOREVER);

    if (isFifoFull(p_fifo))
    {
        if ((*p_packet->ackFunc)(p_packet, p_packet->ackParam))
        {
            p_packet->pktFlags &= ~TCPIP_MAC_PKT_FLAG_QUEUED;
        }
    }

    if (p_fifo->rear == NUM_PREALLOCATED_RX_PACKETS - 1)
        p_fifo->rear = 0;
    else
        ++p_fifo->rear;

    p_fifo->items[p_fifo->rear] = p_packet;

    WDRV_MUTEX_UNLOCK(s_rxFifoLock);
}

static TCPIP_MAC_PACKET *FifoRemove(WDRV_MRF24WN_RX_FIFO *const p_fifo)
{
    if (p_fifo->front == NUM_PREALLOCATED_RX_PACKETS - 1)
        p_fifo->front = 0;
    else
        ++p_fifo->front;

    return p_fifo->items[p_fifo->front];
}

// retrieve the oldest of the queued Rx packets to deliver to the stack
static TCPIP_MAC_PACKET *GetRxPacket(void)
{
    TCPIP_MAC_PACKET *ret;

    WDRV_MUTEX_LOCK(s_rxFifoLock, OSAL_WAIT_FOREVER);

    if (!isFifoEmpty(&s_rxFifo))
        ret = FifoRemove(&s_rxFifo);
    else
        ret = NULL; // signals no Rx packet to process

    WDRV_MUTEX_UNLOCK(s_rxFifoLock);

    return ret;
}

static bool RxPacketAckFunc(TCPIP_MAC_PACKET *pktHandle, const void *ackParam)
{
    if (pktHandle){
        // if this is packet allocated at init and is going to be reused
        if ((pktHandle->pDSeg->segFlags & TCPIP_MAC_SEG_FLAG_RX_STICKY) == TCPIP_MAC_SEG_FLAG_RX_STICKY) {
            pktHandle->pktFlags &= ~TCPIP_MAC_PKT_FLAG_QUEUED;
            WDRV_MUTEX_LOCK(s_rxQueueLock, OSAL_WAIT_FOREVER);
            TCPIP_Helper_SingleListTailAdd(&s_rxQueue, (SGL_LIST_NODE *)pktHandle); // add packet back to free list
            WDRV_MUTEX_UNLOCK(s_rxQueueLock);
        }
    } else {
        WDRV_ASSERT(false, "pktHandle cannot be null");
    }

    return false;
}

static void RxPacketBufferInit(void)
{
    int32_t i;
    // create a queue to hold pointers to preallocated Rx packets
    TCPIP_Helper_SingleListInitialize(&s_rxQueue);

    for (i = 0; i < NUM_PREALLOCATED_RX_PACKETS; ++i) {
        // preallocate Rx buffers to store Rx packets as they come in (1500 bytes data plus header and checksum)
        s_rxPacketBuffer[i] = s_pktAllocF ? _TCPIP_PKT_ALLOC_FNC(sizeof(TCPIP_MAC_PACKET), MAX_RX_PACKET_SIZE, 0) : 0;

        if (s_rxPacketBuffer[i] != NULL) {
            s_rxPacketBuffer[i]->next = NULL;
            s_rxPacketBuffer[i]->ackFunc = RxPacketAckFunc;
            s_rxPacketBuffer[i]->ackParam = NULL;
            s_rxPacketBuffer[i]->pktFlags = 0;
            s_rxPacketBuffer[i]->pDSeg->segFlags |= TCPIP_MAC_SEG_FLAG_RX_STICKY;
            TCPIP_Helper_SingleListTailAdd(&s_rxQueue, (SGL_LIST_NODE *)s_rxPacketBuffer[i]);
        } else {
            WDRV_ASSERT(false, "");
        }
    }

    FifoInit(&s_rxFifo);
}

static void RxPacketBufferDeinit(void)
{
    int i;

    if (s_pktFreeF != NULL) {
        for (i = 0; i < NUM_PREALLOCATED_RX_PACKETS; ++i) {
            if (s_rxPacketBuffer[i] != NULL) {
                _TCPIP_PKT_FREE_FNC(s_rxPacketBuffer[i]);
                s_rxPacketBuffer[i] = NULL;
            }
        }
    }
}

// Finds an available Rx packet structure from the list that was allocated and
// queued up at initialization process.
static TCPIP_MAC_PACKET *GetAvailRxPakcetBuffer(void)
{
    TCPIP_MAC_PACKET *p_packet = NULL;

    WDRV_MUTEX_LOCK(s_rxQueueLock, OSAL_WAIT_FOREVER);
    if (s_rxQueue.nNodes > 0) {
        p_packet = (TCPIP_MAC_PACKET *)TCPIP_Helper_SingleListHeadRemove(&s_rxQueue);
        WDRV_ASSERT(p_packet != NULL, "Should never happen");
    }
    WDRV_MUTEX_UNLOCK(s_rxQueueLock);

    return p_packet;
}

static bool isBroadcastPacket(const uint8_t *addr)
{
    int i;

    for (i = 0; i < 6; i++) {
        if (addr[i] != 0xff)
            return false;
    }

    return true;
}

static bool isMulticastPacket(uint8_t const *const frame)
{
    if (((frame[0] & 0x01) == 0x01) && (!isBroadcastPacket(&frame[0])))
        return true;
    else
        return false;
}

static bool isSolicitedMulticastPacket(uint8_t const *const frame)
{
    return isMulticastAddrSet((TCPIP_MAC_ADDR *)frame);
}

static bool isPacketValid(uint8_t const *const frame)
{
    if (isMulticastPacket(frame)) {
        if (isSolicitedMulticastPacket(frame))
            return true;
        else
            return false;
    } else {
        return true;
    }

    return false;
}

static void CopyFrameToStackPacketBufferCB(uint32_t len, const uint8_t *const frame)
{
    TCPIP_MAC_PACKET *p_packet;

    WDRV_DBG_TRACE_PRINT("Received packet\r\n");

    if (!isPacketValid(frame))
        return;

    if (!isLinkUp()) {
        WDRV_DBG_INFORM_PRINT("MRF24WN is in unconnected state, the received packet will be discarded\r\n");
        return;
    }

    p_packet = GetAvailRxPakcetBuffer();

    if (!p_packet) {
        WDRV_DBG_INFORM_PRINT("No Rx buffer is available, the received packet will be dropped\r\n");
        return;
    }

    // mark packet as queued and stuff in timestamp
    p_packet->pDSeg->segLen = len - ETH_HEADER_SIZE;
    p_packet->pktFlags |= TCPIP_MAC_PKT_FLAG_QUEUED;
    p_packet->tStamp = SYS_TMR_TickCountGet();

    // Note: re-set pMacLayer and pNetLayer; IPv6 changes these pointers inside the packet, so
    //       when Rx packets are reused this is needed.
    memcpy(p_packet->pDSeg->segLoad, frame, len);
    p_packet->pMacLayer = p_packet->pDSeg->segLoad;
    p_packet->pNetLayer = p_packet->pMacLayer + sizeof(TCPIP_MAC_ETHERNET_HEADER);

    // store packet pointer in FIFO and signal stack that rx packet ready to process
    FifoInsert(&s_rxFifo, p_packet);

    // notify stack that Rx packet has arrived
    WDRV_TrafficEventReq(TCPIP_EV_RX_DONE, 0);
}

static SYS_MODULE_OBJ WDRV_MRF24WN_Initialize(const SYS_MODULE_INDEX index, const SYS_MODULE_INIT *const init)
{
	const TCPIP_MAC_MODULE_CTRL *const macCtrl = ((TCPIP_MAC_INIT *)init)->macControl;
	WDRV_CALLBACKS CBs;

	WDRV_ASSERT((macCtrl != NULL), "macCtrl is null");

	// already initialized, have a client connected
	if (wdrv_mrf24wn_dcpt.isOpen)
		return (SYS_MODULE_OBJ)&wdrv_mrf24wn_dcpt;

	s_isInitComplete = false;

    /*
     * Initialize g_wdrv_priv, intentionally exclude initConn.
     * For isDisconnectRequested, it allows module to stay idle when
     * WDRV_Connect() is not called, and it should remain true until
     * WDRV_Connect() is called.
     */
	memset(g_wdrv_priv.macAddr, 0, sizeof(uint8_t) * 6);
	g_wdrv_priv.isDriverOpen = false;
	g_wdrv_priv.updateMacAddressRequired = false;
	g_wdrv_priv.isScanDone = false;
	g_wdrv_priv.isConnReestablished = false;
	g_wdrv_priv.isDisconnectRequested = true;
	WDRV_SemInit(&g_wdrv_priv.disconnectDoneSync);

    g_wdrv_scanStatus.scanInProgress = false;
    g_wdrv_scanStatus.numberOfResults = 0;

	g_debugConsoleLock = NULL;
	s_multicastFilterLock = NULL;
	s_rxFifoLock = NULL;
	s_rxQueueLock = NULL;
	WDRV_MUTEX_CREATE(&g_debugConsoleLock);
	WDRV_MUTEX_CREATE(&s_multicastFilterLock);
	WDRV_MUTEX_CREATE(&s_rxFifoLock);
	WDRV_MUTEX_CREATE(&s_rxQueueLock);

	s_macCtrl = *macCtrl;
	s_pktAllocF = (TCPIP_MAC_PKT_AllocF)macCtrl->pktAllocF;
	s_pktFreeF = (TCPIP_MAC_PKT_FreeF)macCtrl->pktFreeF;
	s_pktAckF = (TCPIP_MAC_PKT_AckF)macCtrl->pktAckF;

	if (macCtrl->moduleId != TCPIP_THIS_MODULE_ID)
		return SYS_MODULE_OBJ_INVALID; // only MRF24WN is supported

	WDRV_TrafficEventInit(macCtrl->eventF, macCtrl->eventParam);

	RxPacketBufferInit();
	WDRV_AllEventClear();

#if defined(TCPIP_STACK_COMMANDS_WIFI_ENABLE)
	WDRV_CLI_Init();
#endif

	WDRV_GPIO_Init();
	WDRV_GPIO_PowerOn();
	WDRV_SPI_Init();

	WDRV_EXT_PrivConfig(s_wdrvext_priv_default);

	CBs.CopyFrameToStackPacketBuffer_CB = CopyFrameToStackPacketBufferCB;
	CBs.ProceedConnectEvent_CB = ProceedConnectEventCB;
	CBs.RFReady_CB = RFReadyCB;
	CBs.ScanDone_CB = ScanDoneCB;
	CBs.InitDone_CB = InitDoneCB;
	CBs.WPSDone_CB = WPSDoneCB;
	WDRV_EXT_Initialize(&CBs);

	if (!WDRV_CONFIG_Load())
		return SYS_MODULE_OBJ_INVALID;

	wdrv_mrf24wn_dcpt.isInit = true;
	wdrv_mrf24wn_dcpt.sysStat = SYS_STATUS_BUSY;

	return (SYS_MODULE_OBJ)&wdrv_mrf24wn_dcpt;
}

static void WDRV_MRF24WN_Tasks(SYS_MODULE_OBJ object)
{
    TCPIP_MAC_RES res = TCPIP_MAC_RES_PENDING;
    WDRV_MRF24WN_DCPT *pDcpt = (WDRV_MRF24WN_DCPT *)object;

    switch (pDcpt->sysStat) {
    case SYS_STATUS_UNINITIALIZED:
        break;
    case SYS_STATUS_BUSY:
        if (g_wdrv_priv.updateMacAddressRequired) {
            memcpy((uint8_t *)(s_macCtrl.ifPhyAddress.v), g_wdrv_priv.macAddr, 6);
            WDRV_DBG_INFORM_PRINT("## MAC address: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                    s_macCtrl.ifPhyAddress.v[0], s_macCtrl.ifPhyAddress.v[1],
                    s_macCtrl.ifPhyAddress.v[2], s_macCtrl.ifPhyAddress.v[3],
                    s_macCtrl.ifPhyAddress.v[4], s_macCtrl.ifPhyAddress.v[5]);
            g_wdrv_priv.updateMacAddressRequired = false;
        }

		if (s_isInitComplete) {
			if (g_wdrv_priv.initConn)
				WDRV_Connect();
			res = TCPIP_MAC_RES_OK;
		}

        if (res == TCPIP_MAC_RES_OK) {
            g_wdrv_priv.isDriverOpen = true;
            pDcpt->sysStat = SYS_STATUS_READY;
            WDRV_DBG_TRACE_PRINT("MRF24WN: Initialization is complete\r\n");
        } else if (res != TCPIP_MAC_RES_PENDING) {
            WDRV_CONFIG_Save();
            WDRV_AllEventClear();
            WDRV_TrafficEventDeinit();
            pDcpt->sysStat = SYS_STATUS_ERROR;
        }
        break;
    case SYS_STATUS_READY:
    case SYS_STATUS_READY_EXTENDED:
        if (isEventPending())
            WDRV_PendingEventProcess();
        break;
    default: // SYS_STATUS_ERROR
        WDRV_ASSERT(false, "Should never happen");
        break;
    }
}

static void WDRV_MRF24WN_Reinitialize(SYS_MODULE_OBJ object, const SYS_MODULE_INIT *const init)
{
    /* unsupported */
}

static void WDRV_MRF24WN_Deinitialize(SYS_MODULE_OBJ object)
{
    WDRV_MRF24WN_DCPT *pDcpt = (WDRV_MRF24WN_DCPT *)object;

    WDRV_DBG_INFORM_PRINT("MRF24WN: De-initializing . . .\r\n");
    s_isInitComplete = false;
    pDcpt->isInit = false;
    pDcpt->sysStat = SYS_STATUS_UNINITIALIZED;

    WDRV_CONFIG_Save();
    WDRV_AllEventClear();
    WDRV_TrafficEventDeinit();
    RxPacketBufferDeinit();
    WDRV_EXT_Deinitialize();
    WDRV_GPIO_PowerOff();
    WDRV_GPIO_DeInit();
    WDRV_SPI_Deinit();

    WDRV_MUTEX_DELETE(&g_debugConsoleLock);
    WDRV_MUTEX_DELETE(&s_multicastFilterLock);
    WDRV_MUTEX_DELETE(&s_rxFifoLock);
    WDRV_MUTEX_DELETE(&s_rxQueueLock);
    g_debugConsoleLock = NULL;
    s_multicastFilterLock = NULL;
    s_rxFifoLock = NULL;
    s_rxQueueLock = NULL;

    WDRV_SemDeInit(&g_wdrv_priv.disconnectDoneSync);
}

static SYS_STATUS WDRV_MRF24WN_Status(SYS_MODULE_OBJ object)
{
    return ((WDRV_MRF24WN_DCPT *)object)->sysStat;
}

static size_t WDRV_MRF24WN_GetConfig(TCPIP_MODULE_MAC_ID modId, void *configBuff, size_t buffSize, size_t *pConfigSize)
{
    /* unsupported */
    return 0;
}

static TCPIP_MAC_HANDLE WDRV_MRF24WN_Open(const SYS_MODULE_INDEX drvIndex, const DRV_IO_INTENT intent)
{
    if (!wdrv_mrf24wn_dcpt.isOpen) {
        MulticastFilter_Initialize();
        wdrv_mrf24wn_dcpt.isOpen = true;
    }
    return (TCPIP_MAC_HANDLE)&wdrv_mrf24wn_dcpt;
}

bool isMacInitialized(void)
{
    return wdrv_mrf24wn_dcpt.isInit;
}

uint8_t WDRV_ScanStart(void)
{
    if (g_wdrv_scanStatus.scanInProgress)
        return 0;

    g_wdrv_priv.isScanDone = false;
    g_wdrv_scanStatus.scanInProgress = true;

    WDRV_EXT_CmdScanStart();

    return WDRV_SUCCESS;
}

//DOM-IGNORE-END
