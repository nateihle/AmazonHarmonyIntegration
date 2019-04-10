/*******************************************************************************
  MRF24WN Wireless Driver SPI Communications Support

  File Name: 
    wdrv_mrf24wn_spi.c  

  Summary:
    MRF24WN Wireless Driver SPI Communications Support

  Description:
    Supports SPI communications to the MRF24WN module.
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

#include "driver/spi/drv_spi.h"
#include "system/devcon/sys_devcon.h"

#include "driver/wifi/mrf24wn/include/wdrv_mrf24wn_api.h"

#if defined(__PIC32MZ__)
#define WDRV_DCACHE_CLEAN(addr, size) _DataCacheClean(addr, size)
#else /* !defined(__PIC32MZ__) */
#define WDRV_DCACHE_CLEAN(addr, size) do { } while (0)
#endif /* defined(__PIC32MZ__) */

#define SPI_TRANSACTION_COMPLETE_NOTIFY(sem) WDRV_SemGiveFromISR(sem)
#define SPI_WAIT_FOR_TX_COMPLETION() WDRV_SemTake(&s_txSync, OSAL_WAIT_FOREVER)
#define SPI_WAIT_FOR_RX_COMPLETION() WDRV_SemTake(&s_rxSync, OSAL_WAIT_FOREVER)

#if (DRV_SPI_DMA != 0)
#define SPI_DMA_DCACHE_CLEAN(addr, size) WDRV_DCACHE_CLEAN(addr, size)
#define SPI_DMA_MAX_TX_SIZE DRV_SPI_DMA_TXFER_SIZE
#define SPI_DMA_MAX_RX_SIZE DRV_SPI_DMA_DUMMY_BUFFER_SIZE
#else /* (DRV_SPI_DMA == 0) */
#define SPI_DMA_DCACHE_CLEAN(addr, size) do { } while (0)
#define SPI_DMA_MAX_TX_SIZE -1 /* Unused. Just to avoid compilation error. */
#define SPI_DMA_MAX_RX_SIZE -1 /* Unused. Just to avoid compilation error. */
#endif /* (DRV_SPI_DMA != 0) */

static DRV_HANDLE s_spiHandle = -1;
static DRV_SPI_BUFFER_HANDLE s_spiBufferHandleTx;
static DRV_SPI_BUFFER_HANDLE s_spiBufferHandleRx;
OSAL_SEM_HANDLE_TYPE s_txSync;
OSAL_SEM_HANDLE_TYPE s_rxSync;

#if defined(__PIC32MZ__)
static void _DataCacheClean(unsigned char *address, uint32_t size)
{
    if (IS_KVA0(address)) {
        uint32_t a = (uint32_t)address & 0xfffffff0;
        uint32_t r = (uint32_t)address & 0x0000000f;
        uint32_t s = ((size + r + 15) >> 4) << 4;

        SYS_DEVCON_DataCacheClean(a, s);
    }
}
#endif /* defined(__PIC32MZ__) */

static void _CS_Init(void)
{
    SYS_PORTS_PinDirectionSelect(PORTS_ID_0, SYS_PORTS_DIRECTION_OUTPUT, WF_CS_PORT_CHANNEL, WF_CS_BIT_POS);
}

static void _CS_Assert()
{
    SYS_PORTS_PinClear(PORTS_ID_0, WF_CS_PORT_CHANNEL, WF_CS_BIT_POS);
}

static void _CS_Deassert()
{
    SYS_PORTS_PinSet(PORTS_ID_0, WF_CS_PORT_CHANNEL, WF_CS_BIT_POS);
}

static bool _SPI_Init(void)
{
    if (s_spiHandle == ((DRV_HANDLE)(-1))) {
        s_spiHandle = DRV_SPI_Open(WDRV_SPI_INDEX, DRV_IO_INTENT_READWRITE|DRV_IO_INTENT_BLOCKING);
        if (s_spiHandle == ((DRV_HANDLE)(-1)))
        {
            WDRV_ASSERT(false, "SPI init failed");
            return false;
        }
    }

    WDRV_SemInit(&s_txSync);
    WDRV_SemInit(&s_rxSync);

    return true;
}

static void _SPI_Deinit(void)
{
    WDRV_SemDeInit(&s_txSync);
    WDRV_SemDeInit(&s_rxSync);
    //DRV_SPI_Close(s_spiHandle);
}

static void _SPI_TxComplete(DRV_SPI_BUFFER_EVENT event, DRV_SPI_BUFFER_HANDLE bufferHandle, void *context)
{
    SPI_TRANSACTION_COMPLETE_NOTIFY(&s_txSync);
}

static void _SPI_RxComplete(DRV_SPI_BUFFER_EVENT event, DRV_SPI_BUFFER_HANDLE bufferHandle, void *context)
{
    SPI_TRANSACTION_COMPLETE_NOTIFY(&s_rxSync);
}

static bool _SPI_Tx(unsigned char *buf, uint32_t size)
{
    bool ret = true;

    SPI_DMA_DCACHE_CLEAN(buf, size);
    s_spiBufferHandleTx = DRV_SPI_BufferAddWrite(s_spiHandle, buf, size, _SPI_TxComplete, 0);
    if (s_spiBufferHandleTx == (DRV_SPI_BUFFER_HANDLE)(-1))
        ret = false;

    SPI_WAIT_FOR_TX_COMPLETION();
    return ret;
}

static bool _SPI_Rx(unsigned char *const buf, uint32_t size)
{
    bool ret = true;

    SPI_DMA_DCACHE_CLEAN(buf, size);
    s_spiBufferHandleRx = DRV_SPI_BufferAddRead(s_spiHandle, buf, size, _SPI_RxComplete, 0);
    if (s_spiBufferHandleRx == (DRV_SPI_BUFFER_HANDLE)(-1))
        ret = false;

    SPI_WAIT_FOR_RX_COMPLETION();
    return ret;
}

static bool _SPI_Write(unsigned char *p_txBuf, uint32_t txLen)
{
    int c = 0;
    bool ret = true;
    
    _CS_Assert();

    while (txLen > SPI_DMA_MAX_TX_SIZE) {
        ret = _SPI_Tx(p_txBuf + c * SPI_DMA_MAX_TX_SIZE, SPI_DMA_MAX_TX_SIZE);
        txLen -= SPI_DMA_MAX_TX_SIZE;
        ++c;
    }
    
    if (txLen > 0)
        ret = _SPI_Tx(p_txBuf + c * SPI_DMA_MAX_TX_SIZE, txLen);

    _CS_Deassert();
    WDRV_ASSERT(ret, "SPI write failed");        
    return ret;
}

static bool _SPI_Read(unsigned char *p_cmd, uint32_t cmdLen, unsigned char *p_rxBuf, uint32_t rxLen)
{
    int c = 0;
    bool ret = true;

    _CS_Assert();
    if (cmdLen > 0)
        ret = _SPI_Tx(p_cmd, cmdLen);

    if (ret && rxLen > 0) {
        while (rxLen > SPI_DMA_MAX_RX_SIZE) {
            ret = _SPI_Rx(p_rxBuf + c * SPI_DMA_MAX_RX_SIZE, SPI_DMA_MAX_RX_SIZE);
            rxLen -= SPI_DMA_MAX_RX_SIZE;
            ++c;
        }
        
        if (rxLen > 0)
            ret = _SPI_Rx(p_rxBuf + c * SPI_DMA_MAX_RX_SIZE, rxLen);
    }

    _CS_Deassert();
    WDRV_ASSERT(ret, "SPI read failed");        
    return ret;
}

void WDRV_SPI_Out(uint8_t *const bufOut, uint16_t OutSize)
{
    bool intEnabled = SYS_INT_SourceDisable(MRF_INT_SOURCE);

    _SPI_Write(bufOut, OutSize);
    if (intEnabled)
        WDRV_INTR_SourceEnable();
}

void WDRV_SPI_In(uint8_t *const OutBuf, uint16_t OutSize, uint8_t *const InBuf, uint16_t InSize)
{
    bool intEnabled = SYS_INT_SourceDisable(MRF_INT_SOURCE);

    _SPI_Read(OutBuf, OutSize, InBuf, InSize);
    if (intEnabled)
        WDRV_INTR_SourceEnable();
}

void WDRV_SPI_Init(void)
{     
    _CS_Init();     
    _CS_Deassert();
    _SPI_Init(); 
}

void WDRV_SPI_Deinit(void)
{
    _SPI_Deinit();
}

//DOM-IGNORE-END
