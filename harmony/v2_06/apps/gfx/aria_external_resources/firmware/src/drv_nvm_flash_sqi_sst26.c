/*
SOFTWARE AND DOCUMENTATION ARE PROVIDED ?AS IS? WITHOUT WARRANTY OF ANY

KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY

OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR

PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR

OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,

BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT

DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL,

INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA,

COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY

CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),

OR OTHER SIMILAR COSTS.


	Filename:	drv_nvm_flash_sqi_sst26.c
 	Author:		Fergus O'Kane/Mihai Paiu
	Reviewer:
	Date:		21Jan15
	Processor:	PIC32MZ2048ECH144
	Compiler:	XC32 v1.34
 	Notes:

*/

// include files
#include <string.h>
#include "drv_nvm_flash_sqi_sst26.h"

// function definitions

// *****************************************************************************

/* buffers in non cached memory */
uint8_t __attribute__((coherent)) sqiCmdBuffer[32];
sqiDMADesc __attribute__((coherent)) sqiDescCommand1;
sqiDMADesc __attribute__((coherent)) sqiDescCommand2;
sqiDMADesc __attribute__((coherent)) sqiDescCommand3;
sqiDMADesc __attribute__((coherent)) sqiDescBuffer[SQI_NUM_BUFFER_DESC + 1]; /* allow for end descriptor */
sqiDMADesc __attribute__((coherent)) sqiTxDescBuffer[2]; /* allow for end descriptor */
static uint8_t __attribute__((coherent, aligned(4))) sqiReadBuffer[SQI_BUFFER_SIZE];
static uint8_t __attribute__((coherent, aligned(4))) sqiWriteBuffer[SST26_PAGE_SIZE];
static uint8_t __attribute__((coherent, aligned(4))) sqiTemp[4];
static uint32_t __attribute__((coherent, aligned(4))) sqiReadUint32;

//static bool sqiBusy = false;

// *****************************************************************************
// *****************************************************************************
// Function: Core Timer Read
// *****************************************************************************
// *****************************************************************************

static uint32_t APP_ReadCoreTimer()
{
    volatile uint32_t timer;

    // get the count reg
    asm volatile("mfc0   %0, $9" : "=r"(timer));

    return (timer);
}

// *****************************************************************************
// *****************************************************************************
// Function: Core Timer Sart
// *****************************************************************************
// *****************************************************************************

static void APP_StartCoreTimer(uint32_t period)
{
    /* Reset the coutner */
    volatile uint32_t loadZero = 0;

    asm volatile("mtc0   %0, $9" : "+r"(loadZero));
    asm volatile("mtc0   %0, $11" : "+r" (period));
}
// *****************************************************************************
// *****************************************************************************
// Function: Core Timer Delay
// *****************************************************************************
// *****************************************************************************

static void APP_CoreTimer_Delay(uint32_t delayValue)
{
    while ((APP_ReadCoreTimer() <= delayValue))
        asm("nop");
}

/*
 Function: SST26Init
 Description: Initialize the SQI peripheral with the settings from pInitData
 structure.
 The DRV_SQI_INIT_DATA will contain settings for:
 	Chip select output enable bits <CSEN>
 	Clock phase/polarity bits <CPOL/CPHA>
 	SQI clock divider <CLKDIV>
 The macros for these settings can be found in the Harmony SQI help.
 Reset the SQI module and set it for quad mode.
 */
void SST26Init(DRV_SQI_INIT_DATA *pInitData)
{

    // configure the SQI
    // Set up SQI Configuration (SQI1CFG) Register
    PLIB_SQI_ConfigWordSet(SQI_ID_0,
                           1,
                           pInitData->csPins,
                           SQI_DATA_OEN_QUAD,
                           1, // Resets control, transmit, receive buffers and state machines
                           1, // Burst Enable (always set to '1')
                           0, // SQID2 doesn?t act as HOLD# signal in single and dual lane modes
                           0, // SQID3 doesn?t act as WP# signal in single and dual lane modes
                           0, // Receive latch is not active in transmit mode
                           SQI_DATA_FORMAT_MSBF,
                           pInitData->dataMode,
                           SQI_XFER_MODE_PIO
                           );

    PLIB_SQI_TapDelaySet(SQI_ID_0, 9, 0, 2);

    // configure SQI1clkcon
    PLIB_SQI_ClockDividerSet(SQI_ID_0, pInitData->clkDivider);
    PLIB_SQI_ClockEnable(SQI_ID_0);
    while (!PLIB_SQI_ClockIsStable(SQI_ID_0));

    // Setup SQI FIFO Thresholds
    PLIB_SQI_ControlBufferThresholdSet(SQI_ID_0, 1);
    PLIB_SQI_TxBufferThresholdSet(SQI_ID_0, 4);
    PLIB_SQI_RxBufferThresholdSet(SQI_ID_0, 4);
    PLIB_SQI_TxBufferThresholdIntSet(SQI_ID_0, 4);
    PLIB_SQI_RxBufferThresholdIntSet(SQI_ID_0, 4);

    // Reset the Flash
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_SINGLE, SQI_CMD_TRANSMIT, 1); // NOP
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_SINGLE, SQI_CMD_TRANSMIT, 1); // NOP
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_SINGLE, SQI_CMD_TRANSMIT, 1); // NOP
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_SINGLE, SQI_CMD_TRANSMIT, 1); // RSTEN
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_SINGLE, SQI_CMD_TRANSMIT, 1); // RESET
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_SINGLE, SQI_CMD_TRANSMIT, 1); // NOP
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_SINGLE, SQI_CMD_TRANSMIT, 1); // NOP
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_SINGLE, SQI_CMD_TRANSMIT, 1); // NOP
    PLIB_SQI_TransmitData(SQI_ID_0, (SST26VF_RSTEN << 24) | (SST26VF_NOP << 16) | (SST26VF_NOP << 8) | SST26VF_NOP);
    // wait for Tcph = min 25 ms
    APP_StartCoreTimer(0);
    APP_CoreTimer_Delay(5000000); // delay 50ms
    Nop();
    // transmit RESET
    PLIB_SQI_TransmitData(SQI_ID_0, (SST26VF_NOP << 24) | (SST26VF_NOP << 16) | (SST26VF_NOP << 8) | SST26VF_RESET);
    // Wait for reset to happen, max 1 ms
    APP_StartCoreTimer(0);
    APP_CoreTimer_Delay(100000); // delay 1ms
    Nop();

    // configure the SQI for quad mode
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_SINGLE, SQI_CMD_TRANSMIT, 1);
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_SINGLE, SQI_CMD_TRANSMIT, 1);
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_SINGLE, SQI_CMD_TRANSMIT, 1);
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_SINGLE, SQI_CMD_TRANSMIT, 1);
    PLIB_SQI_TransmitData(SQI_ID_0, (SST26VF_EQIO << 24) | 0x00000000); // 0x38

    // TODO check busy statuses instead of blocking delays - make a state machine
    APP_StartCoreTimer(0);
    APP_CoreTimer_Delay(100000); // delay 1ms
}

/*
 Function: SST26WriteByte
 Description: Call the SST26WriteEnable function to enable writes.
 Configure for PIO mode.
 Send the write command (SST26VF_PAGE_WRITE) followed by the address
 and the data byte. Note we have to send 4 bytes, so when sending one byte we
 convert this to a 32-bit word and set the other bits to ones (0xFFFFFFxx).
 Wait the write cycle time for write complete.
 */
void SST26WriteByte(uint32_t address, uint8_t data)
{
    uint8_t tempAddress1, tempAddress2, tempAddress3;
    uint32_t txWord;

    // init variables
    txWord = 0xFFFFFF00;

    // Address manipulation (LaZ logic)
    tempAddress1 = (uint8_t) (address >> 16);
    tempAddress2 = (uint8_t) (address >> 8);
    tempAddress3 = (uint8_t) address;
    address = tempAddress1 | tempAddress2 << 8 | tempAddress3 << 16;

    /* configure for PIO mode */
    PLIB_SQI_TransferModeSet(SQI_ID_0, SQI_XFER_MODE_PIO);
    PLIB_SQI_TxBufferThresholdSet(SQI_ID_0, 4);
    PLIB_SQI_TxBufferThresholdIntSet(SQI_ID_0, 4);

    // enable writes
    SST26WriteEnable();

    /* write the command and address */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 0, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);  /* Page write */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 0, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 3);  /* address */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 4);  /* data */

    // Write the command to the transfer buffer
    PLIB_SQI_TransmitData(SQI_ID_0, address << 8 | SST26VF_PAGE_WRITE);   // page_write = 0x02

    // convert the data byte to 32-bits and set other bits to 1's
    txWord |= (uint32_t)data;
    PLIB_SQI_TransmitData(SQI_ID_0, txWord);

    APP_StartCoreTimer(0);
    APP_CoreTimer_Delay(180000);   // was 10000000
}

/*
 Function:
    uint8_t SST26ReadByte(uint32_t address)

 Parameters:
    address - The address inside the flash device where to read from

 Return:
    the read Byte (unsigned char)

 Description:


 Remarks:
 */
uint8_t SST26ReadByte(uint32_t address)
{

    uint8_t tempAddress1, tempAddress2, tempAddress3;

    // configure for PIO mode
    PLIB_SQI_TransferModeSet(SQI_ID_0, SQI_XFER_MODE_PIO);

    // Setup SQI FIFO Thresholds
    PLIB_SQI_ControlBufferThresholdSet(SQI_ID_0, 4);
    PLIB_SQI_TxBufferThresholdSet(SQI_ID_0, 4);
    PLIB_SQI_RxBufferThresholdSet(SQI_ID_0, 4);
    PLIB_SQI_TxBufferThresholdIntSet(SQI_ID_0, 4);
    PLIB_SQI_RxBufferThresholdIntSet(SQI_ID_0, 4);

    PLIB_SQI_InterruptSignalEnable(SQI_ID_0, SQI_RXTHR);

    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1); // NOP
    PLIB_SQI_ControlWordSet(SQI_ID_0, 0, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1); // FAST_READ
    PLIB_SQI_ControlWordSet(SQI_ID_0, 0, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 3); // address
    PLIB_SQI_ControlWordSet(SQI_ID_0, 0, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1); // MODE
    PLIB_SQI_ControlWordSet(SQI_ID_0, 0, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1); // DUMMY
    PLIB_SQI_ControlWordSet(SQI_ID_0, 0, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1); // DUMMY

    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_RECEIVE, 4);  // READ BYTE - errata - 1 byte is on 4 bytes

    // Address manipulation (LaZ logic)
    tempAddress1 = (uint8_t) (address >> 16); // MSB
    tempAddress2 = (uint8_t) (address >> 8); // Middle
    tempAddress3 = (uint8_t) address; // LSB
    address = tempAddress1 | tempAddress2 << 8 | tempAddress3 << 16;
    
    PLIB_SQI_TransmitData(SQI_ID_0, (address << 16) | (SST26VF_FAST_READ << 8) | SST26VF_NOP); // Nop, Nop, Nop, FAST READ
    PLIB_SQI_TransmitData(SQI_ID_0, (DUMMY_2_BYTES << 16) | (HIGH_SPEED_NOT_IN_MODE_READ << 8) | address >> 16); // Address + Dummy Byte

    // wait for the interrupt flag to set
    while (PLIB_SQI_InterruptFlagGet(SQI_ID_0, SQI_RXTHR) == false);
    sqiReadUint32 = PLIB_SQI_ReceiveData(SQI_ID_0);

    PLIB_SQI_InterruptSignalDisable(SQI_ID_0, SQI_RXTHR);

    return (uint8_t)sqiReadUint32;
}

/*
 Function: SST26WriteEnable
 Description: Send the write enable command, SST26_WEN.
 */
void SST26WriteEnable(void)
{
    // write the control words and WEN
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1); /* NOP */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1); /* NOP */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1); /* NOP */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1); /* WEN */

    PLIB_SQI_TransmitData(SQI_ID_0, (SST26VF_WEN << 24) | DUMMY_3_BYTES);      // wen = 0x06
}

/*
 Function: SST26IsWriteBusy
 Description: Send a read status register command and check the BUSY bit of the
 response. The BUSY bit is bit 0 or bit 7(two options). The status register result
 is 8-bits. Return 1 if BUSY, otherwise 0.
 */
uint8_t SST26IsWriteBusy(void)
{
    uint8_t readStatus = 0;

    readStatus = SST26ReadStatus();

    if ( (readStatus & SST26_BUSY_MASK) | (readStatus & SST26_BUSY_MASK_01) ) {
        return SST26_IS_BUSY;
    }

    return SST26_NOT_BUSY;
}

/*
 Function:
    uint8_t SST26IsWriteEnable(void)

 Parameters
    void

 Return
    boolean: SST26_IS_WRITE_ENABLE (1) or SST26_NOT_WRITE_ENABLE (0)

 Remarks
    none
 */
uint8_t SST26IsWriteEnable(void)
{
    uint8_t readStatus = 0;

    readStatus = SST26ReadStatus();

    if (readStatus & SST26_WEL_MASK) {
        return SST26_IS_WRITE_ENABLE;
    }

    return SST26_NOT_WRITE_ENABLE;
}

/*
 Function:
    uint8_t SST26ReadStatus(void)

 Parameters
    void

 Return
    unsigned char with SST26 STATUS Register

 Remarks
    The SST 26 STATUS Register is documented in DS20005218B-page 10
 */
uint8_t SST26ReadStatus(void)
{
    // configure for PIO mode
    PLIB_SQI_TransferModeSet(SQI_ID_0, SQI_XFER_MODE_PIO);

    PLIB_SQI_InterruptSignalEnable(SQI_ID_0, SQI_RXTHR);

    // Setup SQI FIFO Thresholds
    PLIB_SQI_ControlBufferThresholdSet(SQI_ID_0, 1);
    PLIB_SQI_TxBufferThresholdSet(SQI_ID_0, 4);
    PLIB_SQI_RxBufferThresholdSet(SQI_ID_0, 4);
    PLIB_SQI_TxBufferThresholdIntSet(SQI_ID_0, 4);
    PLIB_SQI_RxBufferThresholdIntSet(SQI_ID_0, 4);

    // Configure to send the Read Quad ID

    // B parts need a dummy after RDSR
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);  // NOP
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);  // NOP
    PLIB_SQI_ControlWordSet(SQI_ID_0, 0, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);  // RDSR
    PLIB_SQI_ControlWordSet(SQI_ID_0, 0, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);  // NOP

    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_RECEIVE, 4); // RECEIVED data - 4 bytes

    PLIB_SQI_TransmitData(SQI_ID_0, (DUMMY_BYTE << 24) | (SST26VF_RDSR << 16) | (SST26VF_NOP << 8) | SST26VF_NOP);

    /* wait until transmission finished */
    while (PLIB_SQI_InterruptFlagGet(SQI_ID_0, SQI_RXTHR) == false);
    sqiReadUint32 = PLIB_SQI_ReceiveData(SQI_ID_0);
    memcpy(&sqiTemp[0], &sqiReadUint32, 4);

    PLIB_SQI_InterruptSignalDisable(SQI_ID_0, SQI_RXTHR);

    return (sqiTemp[3] | sqiTemp[2] | sqiTemp[1] | sqiTemp[0]);
}

/*
 Function: SST26WriteArray
 Description:
 Data pointed to by 'pData' will be written to 'address'. The number of bytes
 'nCount' will be written.
 */
uint8_t SST26WriteArray(uint32_t address, uint8_t *pData, uint32_t nCount)
{
    uint32_t bytesWritten;

    while (nCount > 0) {
        // write on this page
        //bytesWritten = SST26WritePagePIO(address, pData, nCount);
        bytesWritten = SST26WritePageDMA(address, pData, nCount);
        // increment address
        address += bytesWritten;

        // decrement nCount
        nCount -= bytesWritten;

#ifdef __DEBUG
        if (nCount > 0) {
            Nop(); // for breakpoint
        }
#endif
        // increment data address
        pData += bytesWritten;
    }

    return 0;
}

/*
 Function: SST26WritePage
 Description: Call the SST26WriteEnable function to enable writes. Configure for PIO mode.
 Send the write command (SST26VF_PAGE_WRITE) followed by the address of the page
 and up to 256 data bytes for the page address.
 Delay for the write cycle time until the write is complete.
 Returns the number of bytes written
 */
uint32_t SST26WritePagePIO(uint32_t address, uint8_t *pData, uint32_t nCount)
{
    uint8_t tempAddress1, tempAddress2, tempAddress3;
    uint32_t* wordBuffer;
    uint32_t txWord;
    uint32_t endPageAddress, maxBytes;
    uint32_t returnBytes = 0, transmitBytes = 0;

    // upper address
    endPageAddress = ( address & ADDRESS_PAGE_MASK ) + SST26_PAGE_SIZE;

    // compute the maximum number of bytes to write
    maxBytes = endPageAddress - address;

    // chunk the nCount to maxBytes
    if (nCount > maxBytes) {
        nCount = maxBytes;
        Nop();
    }
    returnBytes = nCount;

    memcpy(&sqiWriteBuffer[0], pData, returnBytes);

    wordBuffer = (uint32_t*) &sqiWriteBuffer[0];

    // errata for PIC32MZ
    // transmit multiple of 4
    transmitBytes = nCount;
    if (transmitBytes & 3) {
        transmitBytes += 4 - (transmitBytes & 3);
    }

    // Address manipulation (LaZ logic)
    tempAddress1 = (uint8_t) (address >> 16);
    tempAddress2 = (uint8_t) (address >> 8);
    tempAddress3 = (uint8_t) address;
    address = tempAddress1 | tempAddress2 << 8 | tempAddress3 << 16;

    // enable writes
    SST26WriteEnable();

    /* configure for PIO mode */
    PLIB_SQI_TransferModeSet(SQI_ID_0, SQI_XFER_MODE_PIO);
    PLIB_SQI_TxBufferThresholdSet(SQI_ID_0, 4);
    PLIB_SQI_TxBufferThresholdIntSet(SQI_ID_0, 4);

    PLIB_SQI_InterruptSignalEnable(SQI_ID_0, SQI_TXTHR);

    /* write the command and address */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 0, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);  /* Page write */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 0, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 3);  /* address */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, transmitBytes);  /* data */

    // Write the command to the transfer buffer
    PLIB_SQI_TransmitData(SQI_ID_0, address << 8 | SST26VF_PAGE_WRITE);   // page_write = 0x02
    while (PLIB_SQI_InterruptFlagGet(SQI_ID_0, SQI_TXTHR) == false);

    while (nCount >= 4) {
        txWord = *wordBuffer++;
        PLIB_SQI_TransmitData(SQI_ID_0, txWord);
        while (PLIB_SQI_InterruptFlagGet(SQI_ID_0, SQI_TXTHR) == false);
        nCount -= 4;
    }
    if (nCount > 0) {
        txWord = 0xFFFFFFFF;
        memcpy(&txWord, wordBuffer, nCount);
        PLIB_SQI_TransmitData(SQI_ID_0, txWord);
        while (PLIB_SQI_InterruptFlagGet(SQI_ID_0, SQI_TXTHR) == false);
        Nop();
    }

    PLIB_SQI_InterruptSignalDisable(SQI_ID_0, SQI_TXTHR);

    while (SST26IsWriteBusy() == SST26_IS_BUSY);

    return returnBytes;
}

/*
 Function: SST26WritePage
 Description: Call the SST26WriteEnable function to enable writes. Configure for PIO mode.
 Send the write command (SST26VF_PAGE_WRITE) followed by the address of the page
 and up to 256 data bytes for the page address.
 Delay for the write cycle time until the write is complete.
 Returns the number of bytes written
 */
uint32_t SST26WritePageDMA(uint32_t address, uint8_t *pData, uint32_t nCount)
{
    uint8_t tempAddress1, tempAddress2, tempAddress3;
    uint32_t* wordBuffer;
    uint32_t endPageAddress, maxBytes;
    uint32_t txWord;
    uint32_t returnBytes = 0, transmitBytes = 0;

    // upper address
    endPageAddress = ( address & ADDRESS_PAGE_MASK ) + SST26_PAGE_SIZE;

    // compute the maximum number of bytes to write
    maxBytes = endPageAddress - address;

    // chunk the nCount to maxBytes
    if (nCount > maxBytes) {
        nCount = maxBytes;
        Nop();
    }
    returnBytes = nCount;

    // set FFs in sqiWriteBuffer
    memset(&sqiWriteBuffer[0], 0xFF, sizeof(sqiWriteBuffer));

    // copy the bytes to be writen to flash in the buffer
    memcpy(&sqiWriteBuffer[0], pData, nCount);

    // errata for PIC32MZ
    // transmit multiple of 4
    transmitBytes = nCount;
    if (transmitBytes & 3) {
        transmitBytes += 4 - (transmitBytes & 3);
    }

    if (transmitBytes <= 32)
    {
        wordBuffer = (uint32_t*) &sqiWriteBuffer[0];
    }
    else
    {
        memset(sqiTxDescBuffer, 0, sizeof(sqiTxDescBuffer));

        sqiTxDescBuffer[0].BDCon = (0xD08D0000 | transmitBytes);
        sqiTxDescBuffer[0].BDStat = 0;
        sqiTxDescBuffer[0].BDAddr = (unsigned int*) KVA_TO_PA(&sqiWriteBuffer[0]);
        sqiTxDescBuffer[0].nextBDAddr = (struct sqiDMADesc*) KVA_TO_PA(&sqiTxDescBuffer[1]);;

        /* Dummy descriptor */
        sqiTxDescBuffer[1].BDCon = 0x50000000;
        sqiTxDescBuffer[1].nextBDAddr = 0x00000000;
    }

    // Address manipulation (LaZ logic)
    tempAddress1 = (uint8_t) (address >> 16);
    tempAddress2 = (uint8_t) (address >> 8);
    tempAddress3 = (uint8_t) address;
    address = tempAddress1 | tempAddress2 << 8 | tempAddress3 << 16;
    
    /* configure for PIO mode */
    PLIB_SQI_TransferModeSet(SQI_ID_0, SQI_XFER_MODE_PIO);
    PLIB_SQI_TxBufferThresholdSet(SQI_ID_0, 4);
    PLIB_SQI_TxBufferThresholdIntSet(SQI_ID_0, 4);
    
    PLIB_SQI_InterruptSignalEnable(SQI_ID_0, SQI_TXTHR);

    /* write the control words and WEN */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1); /* NOP */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1); /* NOP */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1); /* NOP */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1); /* WEN */

    PLIB_SQI_TransmitData(SQI_ID_0, (SST26VF_WEN << 24) | 0x00000000);
    
    while (!PLIB_SQI_InterruptFlagGet(SQI_ID_0, SQI_TXTHR));

    /* write the command and address */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 0, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 4);  /* page_write + device address */
    
    // Write the command to the transfer buffer
    PLIB_SQI_TransmitData(SQI_ID_0, (address << 8) | SST26VF_PAGE_WRITE);
    while (!PLIB_SQI_InterruptFlagGet(SQI_ID_0, SQI_TXTHR));

    if (transmitBytes <= 32)
    {
        PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, transmitBytes);  /* data */

        while (nCount >= 4) {
            txWord = *wordBuffer++;
            PLIB_SQI_TransmitData(SQI_ID_0, txWord);
            while (!PLIB_SQI_InterruptFlagGet(SQI_ID_0, SQI_TXTHR));
            nCount -= 4;
        }
        if (nCount > 0) {
            txWord = 0xFFFFFFFF;
            memcpy(&txWord, wordBuffer, nCount);
            PLIB_SQI_TransmitData(SQI_ID_0, txWord);
            while (!PLIB_SQI_InterruptFlagGet(SQI_ID_0, SQI_TXTHR));
            Nop();
        }
        PLIB_SQI_InterruptSignalDisable(SQI_ID_0, SQI_TXTHR);
    }
    else
    {
        // Enable BDDONE flag
        PLIB_SQI_InterruptEnable(SQI_ID_0, SQI_BDDONE); // writes to SQI1INTEN

        // Setup SQI transfer thresholds
        PLIB_SQI_ControlBufferThresholdSet(SQI_ID_0, 1);
        PLIB_SQI_RxBufferThresholdSet(SQI_ID_0, 4);
        PLIB_SQI_RxBufferThresholdIntSet(SQI_ID_0, 4);

        //BD_BUFFER1_ADDR address pointer (Base buffer descriptor)
        PLIB_SQI_DMABDBaseAddressSet(SQI_ID_0, (void *) (KVA_TO_PA(&sqiTxDescBuffer[0])));

        //Configure DMA mode
        PLIB_SQI_TransferModeSet(SQI_ID_0, SQI_XFER_MODE_DMA);

        // Enable and start DMA
        PLIB_SQI_DMAEnable(SQI_ID_0);
        PLIB_SQI_DMABDFetchStart(SQI_ID_0);

        //Check to see if BD is finished
        while(!PLIB_SQI_InterruptFlagGet(SQI_ID_0, SQI_BDDONE));

        PLIB_SQI_DMADisable(SQI_ID_0);

        // Disable BDDONE flag
        PLIB_SQI_InterruptDisable(SQI_ID_0,SQI_BDDONE);  
    }
    
    /* delay 1.6 ms to allow write to complete */
    APP_StartCoreTimer(0);
    APP_CoreTimer_Delay(160000);

    return returnBytes;
}




/*******************************************************************************
  Function:
    void SST26ReadArray(uint32_t flashAddress, uint8_t *destBuffer, uint32_t nCount)

  Description:
    Reads nCount bytes of data
    from the desired address of the SQI Flash
    to the desired address in RAM (e.g. pointer inside the frame buffer address)

 Parameters:
    flashAddress - the address in flash where the write should start from
    destBuffer - pointer to the address where the array should be read at
    nCount - number of bytes to count

 Return:
    void

  Remarks:
    Uses DMA mode
    The result of the read will be filled from the address pointed by destBuffer, nCount positions
    It does not add '\0' at the end
 */
void SST26ReadArray(uint32_t flashAddress, uint8_t* destAddress, uint32_t nCount)
{
    uint32_t pageCount, pageIndex, lastPageNbBytes;
    uint32_t readCount;
    //uint8_t* baseAddress;
    uint8_t* readAddress;
    uint32_t waitLoop;

    // get the addres of the buffer
    //baseAddress = (uint8_t*)&sqiReadBuffer[0];

    while (nCount > 0) {
        // set the number of bytes to read
        readCount = (nCount <= sizeof(sqiReadBuffer)) ? nCount : sizeof(sqiReadBuffer);

        // clean the cmd buffer
        memset(sqiReadBuffer, 0xAA, sizeof(sqiReadBuffer));

        // point to the beginning of the sqi read buffer
        readAddress = (uint8_t*)&sqiReadBuffer[0];

        // clean the cmd buffer
        memset(sqiCmdBuffer, 0, sizeof(sqiCmdBuffer));

        // clean the dma descriptor buffers buffer
        memset(sqiDescBuffer, 0, sizeof(sqiDescBuffer));

        sqiCmdBuffer[0] = (uint8_t) SST26VF_FAST_READ;
        sqiCmdBuffer[1] = (uint8_t) (flashAddress >> 16);
        sqiCmdBuffer[2] = (uint8_t) (flashAddress >> 8);
        sqiCmdBuffer[3] = (uint8_t) (flashAddress);
        sqiCmdBuffer[4] = (uint8_t) HIGH_SPEED_NOT_IN_MODE_READ;
        sqiCmdBuffer[5] = (uint8_t) DUMMY_BYTE;
        sqiCmdBuffer[6] = (uint8_t) DUMMY_BYTE;

        // each buffer in DMA will have a page
        // count the number of pages to read
        pageCount = readCount / SST26_PAGE_SIZE;
        // last N bytes need to be multiple of 4
        // PIC32MZ errata
        lastPageNbBytes = readCount % SST26_PAGE_SIZE;
        if (lastPageNbBytes > 0) {
            // if more than 0 bytes remained
            // increment the page counter
            pageCount++;
            // make the last number of bytes multiple of 4
            // if it is not
            if (lastPageNbBytes & 3) {
                lastPageNbBytes += ( 4 - ( lastPageNbBytes & 3) );
            }
        } else {
            // 0 bytes remained to read
            // actually means the last page will have to read SST26_PAGE_SIZE bytes
            lastPageNbBytes = SST26_PAGE_SIZE;
        }

        // sanity check
        // the MZ part has a max number of buffers available - do not pass
        if (pageCount > SQI_NUM_BUFFER_DESC)
        {
            // number of buffers is too high
            // round to maximum
            pageCount = SQI_NUM_BUFFER_DESC;
            // new value for readCount
            readCount = SQI_NUM_BUFFER_DESC * SST26_PAGE_SIZE;
            // multiple of SST26_PAGE_SIZE means last page is SST26_PAGE_SIZE long
            lastPageNbBytes = SST26_PAGE_SIZE;
        }

        /************************************************************************************/
        /* Prepare the buffer descriptors */
        // Command Descriptor 1 - BD1 (to send fast read command and address)
        sqiDescCommand1.BDCon = 0x90800002;
        sqiDescCommand1.BDStat = 0;
        sqiDescCommand1.BDAddr = (unsigned int*) KVA_TO_PA(&sqiCmdBuffer[0]);
        sqiDescCommand1.nextBDAddr = (struct sqiDMADesc *) KVA_TO_PA(&sqiDescCommand2);

        // Command Descriptor 2 - BD2 (Send MODEs)
        sqiDescCommand2.BDCon = 0x90800004;
        sqiDescCommand2.BDStat = 0;
        sqiDescCommand2.BDAddr = (unsigned int*) KVA_TO_PA(&sqiCmdBuffer[2]);
        sqiDescCommand2.nextBDAddr = (struct sqiDMADesc *) KVA_TO_PA(&sqiDescCommand3);

        // Command Descriptor 3 - BD3 (to send dummy cycles)
        sqiDescCommand3.BDCon = 0x90900001;
        sqiDescCommand3.BDStat = 0;
        sqiDescCommand3.BDAddr = (unsigned int*) KVA_TO_PA(&sqiCmdBuffer[6]);
        sqiDescCommand3.nextBDAddr = (struct sqiDMADesc *) KVA_TO_PA(&sqiDescBuffer[0]);


        // set addresses to read from to, paged
        // clean the descriptor buffers
        memset(sqiDescBuffer, 0, sizeof(sqiDescBuffer));
        // read first n - 1 pages, last page needs special BDCon
        for(pageIndex = 0; pageIndex < pageCount - 1; pageIndex++)
        {
            sqiDescBuffer[pageIndex].BDCon = (0x90900000 | SST26_PAGE_SIZE);
            sqiDescBuffer[pageIndex].BDStat = 0;
            sqiDescBuffer[pageIndex].BDAddr = (unsigned int*) KVA_TO_PA(readAddress);
            sqiDescBuffer[pageIndex].nextBDAddr = (struct sqiDMADesc*) KVA_TO_PA(&sqiDescBuffer[pageIndex + 1]);
            // go one page up for the remaining bytes of a page
            readAddress += SST26_PAGE_SIZE;
        }

        // Last data descriptor
        sqiDescBuffer[pageIndex].BDCon = (0xD09D0000 | lastPageNbBytes);
        sqiDescBuffer[pageIndex].BDStat = 0;
        sqiDescBuffer[pageIndex].BDAddr = (unsigned int*) KVA_TO_PA(readAddress);
        // sqiDescBuffer[pageIndex].nextBDAddr = (struct sqiDMADesc*) KVA_TO_PA(&sqiDescBuffer[pageIndex + 1]);
        sqiDescBuffer[pageIndex].nextBDAddr = 0;

        /* Dummy descriptor */
        sqiDescBuffer[pageIndex+1].BDCon = 0x50000000;
        sqiDescBuffer[pageIndex+1].nextBDAddr = 0x00000000;

        // Enable BDDONE flag
        PLIB_SQI_InterruptEnable(SQI_ID_0,SQI_BDDONE);

        // Setup SQI transfer thresholds
        PLIB_SQI_ControlBufferThresholdSet(SQI_ID_0, 4);
        PLIB_SQI_RxBufferThresholdSet(SQI_ID_0, 24);
        PLIB_SQI_RxBufferThresholdIntSet(SQI_ID_0, 24);

        //BD_BUFFER1_ADDR address pointer (Base buffer descriptor)
        PLIB_SQI_DMABDBaseAddressSet(SQI_ID_0, (void *) (KVA_TO_PA(&sqiDescCommand1)));

        //Configure DMA mode
        PLIB_SQI_TransferModeSet(SQI_ID_0, SQI_XFER_MODE_DMA);

        // Enable and start DMA
        PLIB_SQI_DMAEnable(SQI_ID_0);
        PLIB_SQI_DMABDFetchStart(SQI_ID_0);

              
        //Check to see if BD is finished
        while(!PLIB_SQI_InterruptFlagGet(SQI_ID_0, SQI_BDDONE));        

        // Disable BDDONE flag
        PLIB_SQI_InterruptDisable(SQI_ID_0,SQI_BDDONE);
        
        PLIB_SQI_DMADisable(SQI_ID_0);        

        // invalidate cache
        // SYS_DEVCON_DataCacheInvalidate((uint32_t) baseAddress, readCount);
        
        /* wait for the process to complete...*/
        for(waitLoop = 0; waitLoop < pageCount; waitLoop++)
        {
            //APP_StartCoreTimer(0);
            APP_CoreTimer_Delay(2600);
        }        

        // copy the read information to the desired buffer
        memcpy(destAddress, sqiReadBuffer, readCount);

        // decrement nCount
        nCount -= readCount;

        // increment the buffer address
        destAddress += readCount;

        // increment the flash address
        flashAddress += readCount;
    }
}



/*
 Function:
   void SST26ChipErase(void)

 Parameters:
    void

 Return:
    void

 Description:
    Erases the whole flash memory
    Read Byte at any valid address should return FF

 Remarks:
    none
 */
void SST26ChipErase(void)
{
    /* configure for PIO mode */
    PLIB_SQI_TransferModeSet(SQI_ID_0, SQI_XFER_MODE_PIO);

    /* set thresholds */
    PLIB_SQI_ControlBufferThresholdSet(SQI_ID_0, 1);
    PLIB_SQI_TxBufferThresholdSet(SQI_ID_0, 4);
    PLIB_SQI_RxBufferThresholdSet(SQI_ID_0, 4);
    PLIB_SQI_TxBufferThresholdIntSet(SQI_ID_0, 4);
    PLIB_SQI_RxBufferThresholdIntSet(SQI_ID_0, 4);

    PLIB_SQI_InterruptSignalEnable(SQI_ID_0, SQI_TXTHR);

    // send block unprotect command
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);     //
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);     //
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);     //
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);     //

    PLIB_SQI_TransmitData(SQI_ID_0, (SST26VF_ULBPR << 24) | (SST26VF_WEN << 16) | (SST26VF_NOP << 8) | SST26VF_NOP);

    while (PLIB_SQI_InterruptFlagGet(SQI_ID_0, SQI_TXTHR) == false);
    PLIB_SQI_InterruptSignalDisable(SQI_ID_0, SQI_TXTHR);

    // wait 50 ms
    // TODO check busy statuses instead of blocking delays - make a state machine
    APP_StartCoreTimer(0);
    APP_CoreTimer_Delay(5000000); // delay 50ms
    Nop();
    
    /* send commands */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);     /* NOP */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);     /* NOP */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);     /* WREN */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);     /* ERASE */

    PLIB_SQI_TransmitData(SQI_ID_0, (SST26VF_ERASE << 24) | (SST26VF_WEN << 16) | (SST26VF_NOP << 8) | SST26VF_NOP );
    while (PLIB_SQI_InterruptFlagGet(SQI_ID_0, SQI_TXTHR) == false);

    PLIB_SQI_InterruptSignalDisable(SQI_ID_0, SQI_TXTHR);

    while(SST26IsWriteBusy() == SST26_IS_BUSY) {
        Nop();
    }

    // wait 50 ms
    // TODO check busy statuses instead of blocking delays - make a state machine
    APP_StartCoreTimer(0);
    APP_CoreTimer_Delay(5000000); // delay 50ms
}

/*
 Function:
    void SST26ResetWriteProtection(void)
 Parameters:
    void

 Return:
    void

 Description:
    Call this method to set the chip Write Disabled
 */
void SST26ResetWriteProtection(void)
{
    /* send commands */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);     /* NOP */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);     /* NOP */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);     /* NOP */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);     /* WRDI */

    PLIB_SQI_TransmitData(SQI_ID_0, (SST26VF_WRDI << 24) | DUMMY_3_BYTES); // NOP, NOP, NOP, WRDI
}

/*
 Function:
    void SST26SectorErase(uint32_t address)

 Parameters:
    address - the address where the Sector is located - can be any address inside the sector

 Return:
    void

 Description:
    The sector that contains the given address is erased.
    Any address inside the sector should read FF after calling this method

 Remarks:
    A Write Enable should be performed by the application before calling this method
    The method takes care to send the right address to the flash chip
 */
void SST26SectorErase(uint32_t address)
{
    uint8_t tempAddress1, tempAddress2, tempAddress3;

    /* configure for PIO mode */
    PLIB_SQI_TransferModeSet(SQI_ID_0, SQI_XFER_MODE_PIO);

    /* set thresholds */
    PLIB_SQI_ControlBufferThresholdSet(SQI_ID_0, 1);
    PLIB_SQI_TxBufferThresholdSet(SQI_ID_0, 4);
    //PLIB_SQI_RxBufferThresholdSet(SQI_ID_0, 4);
    PLIB_SQI_TxBufferThresholdIntSet(SQI_ID_0, 4);
    //PLIB_SQI_RxBufferThresholdIntSet(SQI_ID_0, 4);

    /* send commands */
    // Address manipulation (LaZ logic)
    tempAddress1 = (uint8_t) (address >> 16);
    tempAddress2 = (uint8_t) (address >> 8);
    tempAddress3 = (uint8_t) address;
    address = tempAddress1 | tempAddress2 << 8 | tempAddress3 << 16;
    //
    // Sector Addresses: Use AMS - A12, remaining address are don't care, but must be set to VIL or VIH.
    address = address & ADDRESS_SECTOR_MASK;
    // send the entire address, the 4K bytes sector corresponding to the sent address should be erased
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);     /* SE */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 3);     /* ADDR */
    PLIB_SQI_TransmitData(SQI_ID_0, (address << 8) | SST26VF_SE ); // ADDR, SE
}


// *****************************************************************************
// *****************************************************************************
// Section: SQI Flash Setup
// *****************************************************************************
// *****************************************************************************
SQI_STATUS SST26_SQI_FlashID_Read(void)
{
    // Setup SQI FIFO Thresholds
    PLIB_SQI_ControlBufferThresholdSet(SQI_ID_0, 1);
    PLIB_SQI_TxBufferThresholdSet(SQI_ID_0, 4);
    PLIB_SQI_RxBufferThresholdSet(SQI_ID_0, 4);
    PLIB_SQI_TxBufferThresholdIntSet(SQI_ID_0, 4);
    PLIB_SQI_RxBufferThresholdIntSet(SQI_ID_0, 4);

    PLIB_SQI_InterruptSignalEnable(SQI_ID_0, SQI_RXTHR);

    // Configure to send the Read Quad ID
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);
    PLIB_SQI_ControlWordSet(SQI_ID_0, 0, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);
    PLIB_SQI_ControlWordSet(SQI_ID_0, 0, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_RECEIVE, 4);

    PLIB_SQI_TransmitData(SQI_ID_0, (DUMMY_BYTE << 24) | (SST26VF_QJID << 16) | (SST26VF_NOP << 8) | SST26VF_NOP);
    APP_StartCoreTimer(0);
    APP_CoreTimer_Delay(40000);

    /* wait until transmission finished */
    while (PLIB_SQI_InterruptFlagGet(SQI_ID_0, SQI_RXTHR) == false);
    sqiReadUint32 = PLIB_SQI_ReceiveData(SQI_ID_0);
    /* align data */
    sqiReadUint32 = sqiReadUint32 & SST26VF_JEDECID_MASK;

    PLIB_SQI_InterruptSignalDisable(SQI_ID_0, SQI_RXTHR);

    if (sqiReadUint32 == SST26VF_JEDECID)
        return SQI_STATUS_SUCCESS;
    else
        return SQI_STATUS_FAILURE;
//    return((int)JedecID);
}

/* EOF */
