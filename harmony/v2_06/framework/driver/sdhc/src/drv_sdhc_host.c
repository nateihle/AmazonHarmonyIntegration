/*******************************************************************************
  SD Host Controller Device Driver Definition

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sdhc_host.c

  Summary:
    SD Host Controller Device Driver Dynamic Implementation

  Description:
    The SD Host Controller device driver provides a simple interface to manage the 
    SD Host Controller modules on Microchip microcontrollers.  This file Implements 
    the core interface routines for the SD Host Controller driver.

*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

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


// *****************************************************************************
// *****************************************************************************
// Section: Include Files
// *****************************************************************************
// *****************************************************************************

#include <sys/kmem.h>

#include "system_config.h"
#include "system_definitions.h"
#include "system/debug/sys_debug.h"
#include "driver/sdhc/src/drv_sdhc_host.h"
#include "drv_sdhc_host_local.h"

// *****************************************************************************
// *****************************************************************************
// Section: File Scope Variables
// *****************************************************************************
// *****************************************************************************

uint8_t gSDHOSTScrReg[SDHOST_SCR_REG_LEN] __attribute__((coherent, aligned(32)));
uint8_t gSDHOSTCidReg[SDHOST_CID_REG_LEN] __attribute__((coherent, aligned(32)));
uint8_t gSDHOSTCsdReg[SDHOST_CSD_REG_LEN] __attribute__((coherent, aligned(32)));
uint8_t gSDHOSTOcrReg[SDHOST_OCR_REG_LEN] __attribute__((coherent, aligned(32)));
uint8_t gSDHOSTSwitchStatusReg[SDHOST_SWITCH_STATUS_REG_LEN] __attribute__((coherent, aligned(32)));

SDHOST_ADMA_DESCR gSDHOSTDmaDescrTable[SDHOST_DMA_NUM_DESCR_LINES]  __attribute__((coherent, aligned(32)));

SDHOST_CARD_CTXT gSDHOSTCardCtxt;
SDHOST_CONTROLLER gSDHOSTCtl;

void sdhostSetThreshold(void)
{
    CFGCON2bits.SDRDFTHR = 0x200;
    CFGCON2bits.SDWRFTHR = 0x200;
}
void sdhostCardDetectEnable(void)
{
	CFGCON2bits.SDCDEN = 0x1;
}

void sdhostCardDetectDisable(void)
{
	CFGCON2bits.SDCDEN = 0x0;
}

void sdhostWriteProtectEnable(void)
{
    CFGCON2bits.SDWPEN = 0x1;
}

void sdhostWriteProtectDisable(void)
{
    CFGCON2bits.SDWPEN = 0x0;
}

static void sdhostDelay(uint16_t timeout)
{
    while (timeout > 0)
    {
        timeout --;
        Nop ();
    }
}

/* This function clears the error type(CMD, DATA or ALL) and waits until the
 * host controller resets the error status. */
void sdhostResetError(SDHOST_RESET_TYPE resetType)
{
    SDHOST->swReset = resetType;
    
    /* Wait until host resets the error status */
    while (SDHOST->swReset & resetType);
}

/* This function sets the bus width to be used with the card. */
void sdhostSetBusWidth(SDHOST_BUS_WIDTH busWidth)
{
    if (busWidth == SDHOST_BUS_WIDTH_4_BIT)
    {
        SDHOST->hostCtl |= (SDHOST_BUS_WIDTH_4_BIT << 1);
    }
    else
    {
        SDHOST->hostCtl &= ~(SDHOST_BUS_WIDTH_4_BIT << 1);
    }
}

/* This function sets the speed mode to be used with the card. */
void sdhostSetSpeedMode(SDHOST_SPEED_MODE speedMode)
{
    if (speedMode == SDHOST_SPEED_MODE_HIGH)
    {
        SDHOST->hostCtl |= (SDHOST_SPEED_MODE_HIGH << 2);
    }
    else
    {
        SDHOST->hostCtl &= ~(SDHOST_SPEED_MODE_HIGH << 2);
    }
}

/* Each descriptor line can transfer 64k of data.
 * */
void sdhostSetupDma(uint8_t *buffer, uint16_t numBytes, DRV_SDHC_OPERATION_TYPE operation)
{
    gSDHOSTDmaDescrTable[0].address = (uint32_t)KVA_TO_PA(buffer);
    gSDHOSTDmaDescrTable[0].length = numBytes;
    gSDHOSTDmaDescrTable[0].attribute = (SDHOST_DESC_TABLE_ATTR_XFER_DATA | SDHOST_DESC_TABLE_ATTR_VALID | SDHOST_DESC_TABLE_ATTR_INTR | SDHOST_DESC_TABLE_ATTR_END);

    SDHOST->admaSysAddress1 = (uint32_t)KVA_TO_PA(&gSDHOSTDmaDescrTable);
}

void sdhostSetBlockSize(uint16_t blockSize)
{
    SDHOST->blockSize = blockSize;
}

void sdhostSetBlockCount(uint16_t numBlocks)
{
    SDHOST->blockCount = numBlocks;
}

void sdhostSetClock(uint32_t clock)
{
    uint32_t div = 0;
    
    /* Disable the clock */
    SDHOST->clockCtl &= ~(SDHOST_CLOCK_ENABLE | SDHOST_CLOCK_INTERNAL_CLK_ENABLE);

    if (clock < gSDHOSTCtl.maxClk)
    {
        div = gSDHOSTCtl.maxClk / clock;

        div >>= 1;
    }

    /* Bits 15-8 sdclock frequency select */
    SDHOST->clockCtl = (div & 0xFF) << 8;
    /* Bits 7-6 Upper bits of sdclock frequency select */
    SDHOST->clockCtl |= ((div & 0x3FF) >> 8) << 6;;

    /* Set the internal clock enable bit */
    SDHOST->clockCtl |= SDHOST_CLOCK_INTERNAL_CLK_ENABLE;
}

bool sdhostIsCardAttached(void)
{
    if (SDHOST->presentState & SDHOST_PSTATE_CARD_INSERTED)
    {
        gSDHOSTCardCtxt.isAttached = true;
    }
    else
    {
        gSDHOSTCardCtxt.isAttached = false;
    }

    return gSDHOSTCardCtxt.isAttached;
}

void sdhostClockEnable(void)
{
    /* Enable the clock */
    SDHOST->clockCtl |= SDHOST_CLOCK_ENABLE;
}

void sdhostClockDisable(void)
{
    /* Disable the clock */
    SDHOST->clockCtl &= ~(SDHOST_CLOCK_ENABLE | SDHOST_CLOCK_INTERNAL_CLK_ENABLE);
}

void sdhostInterruptHandler(SDHOST_CARD_CTXT *cardCtxt)
{
    uint16_t intMask = 0;
    uint16_t intFlags = 0;
    uint16_t errIntStatus = 0;
    
    intMask = SDHOST->intEnable;
    intFlags = SDHOST->intStatus;
    
    if ((intMask & intFlags) == 0)
    {
        return;
    }
    
    if ((intFlags & (SDHOST_COMMAND_COMPLETE_INTERRUPT | SDHOST_ERROR_INTERRUPT)) && (cardCtxt->waitForCmdResp))
    {
        if (intFlags & SDHOST_ERROR_INTERRUPT)
        {
            errIntStatus = SDHOST->errIntStatus;
            if (errIntStatus & 0x0F)
            {
                sdhostResetError (SDHOST_RESET_CMD);
                cardCtxt->commandError = true;
                cardCtxt->errorFlag = errIntStatus & 0x0F;
            }
            else
            {
                cardCtxt->commandCompleted = true;
            }
        }
        else
        {
            cardCtxt->commandCompleted = true;
        }

        if (cardCtxt->isDataPresent)
        {
            cardCtxt->waitForData = true;
        }
    }

    if (cardCtxt->waitForData == true)
    {
        if (intFlags & (SDHOST_TRANSFER_COMPLETE_INTERRUPT | SDHOST_DMA_INTERRUPT | SDHOST_ERROR_INTERRUPT))
        {
            cardCtxt->errorFlag = 0x00;
            if (intFlags & SDHOST_ERROR_INTERRUPT)
            {
                errIntStatus = SDHOST->errIntStatus;
                sdhostResetError (SDHOST_RESET_DAT);
                if (errIntStatus & 0x70)
                {
                    cardCtxt->dataCompleted = true;
                    cardCtxt->errorFlag = errIntStatus & 0x70;
             //       sdhostResetError (SDHOST_RESET_DAT);
                }
                else
                {
                    if (intFlags & (SDHOST_TRANSFER_COMPLETE_INTERRUPT | SDHOST_DMA_INTERRUPT))
                    {
                        cardCtxt->dataCompleted = true;
                    }
                }
            }
            else
            {
                if (intFlags & (SDHOST_TRANSFER_COMPLETE_INTERRUPT | SDHOST_DMA_INTERRUPT))
                {
                    cardCtxt->dataCompleted = true;
                }
            }
        }
    }

    SDHOST->intStatus = intFlags;
    SDHOST->errIntStatus = 0xFFFF;
}

static void sdhostSetTransferMode(uint8_t opcode)
{
    uint16_t transferMode = 0;
    
    switch(opcode)
    {
        case 51:
        case 6:
        case 17:
            {
                /* Read single block of data from the device. */
                transferMode = (SDHOST_XFER_MODE_DMA_ENABLE | SDHOST_XFER_MODE_DATA_XFER_DIR);
                break;
            }

        case 18:
            {
                /* Read mulitple blocks of data from the device. */
                transferMode = (SDHOST_XFER_MODE_DMA_ENABLE | SDHOST_XFER_MODE_DATA_XFER_DIR | SDHOST_XFER_MODE_BLK_CNT_ENABLE | SDHOST_XFER_MODE_MULTI_BLOCK_SEL);
                break;
            }
        case 24:
            {
                /* Write single block of data to the device. */
                transferMode = SDHOST_XFER_MODE_DMA_ENABLE;
                break;
            }
        case 25:
            {
                /* Write mulitple blocks of data to the device. */
                transferMode = (SDHOST_XFER_MODE_DMA_ENABLE | SDHOST_XFER_MODE_BLK_CNT_ENABLE | SDHOST_XFER_MODE_MULTI_BLOCK_SEL);
                break;
            }

        default:
            {
                break;
            }
    }
    
    SDHOST->transferMode = transferMode;
}

bool sdhostIsCmdLineBusy(void)
{
    /* Check if the command inhibit(CMD) bit is clear */
    return (SDHOST->presentState & 0x01);
}

bool sdhostIsDat0LineBusy(void)
{
    /* Check if the command inhibit(DAT) bit is clear */
    return (((SDHOST->presentState & 0x02) == 0x02)? 1: 0);
}

void sdhostReadResponse(SDHOST_READ_RESPONSE_REG respReg, uint32_t *response)
{
    switch (respReg)
    {
        case SDHOST_READ_RESP_REG_0:
        default:
            {
                *response = SDHOST->response0;
                break;
            }
        case SDHOST_READ_RESP_REG_1:
            {
                *response = SDHOST->response1;
                break;
            }
        case SDHOST_READ_RESP_REG_2:
            {
                *response = SDHOST->response2;
                break;
            }
        case SDHOST_READ_RESP_REG_3:
            {
                *response = SDHOST->response3;
                break;
            }
        case SDHOST_READ_RESP_REG_ALL:
            {
                uint32_t *ptr = response;
                *ptr++ = SDHOST->response0;
                *ptr++ = SDHOST->response1;
                *ptr++ = SDHOST->response2;
                *ptr++ = SDHOST->response3;
                break;
            }
    }
}

void sdhostSendCommand(uint8_t opCode, uint8_t respType, uint8_t dataPresent, uint32_t argument)
{
    uint16_t cmd = 0;
    uint16_t interrupt = 0;
    uint8_t flags = 0;
    SDHOST_CARD_CTXT *cardCtxt = &gSDHOSTCardCtxt;
    
    /* Setup the command argument */
    SDHOST->argument = argument;
    
    switch (respType)
    {
        case SDHOST_CMD_RESP_R1:
        case SDHOST_CMD_RESP_R5:
        case SDHOST_CMD_RESP_R6:
        case SDHOST_CMD_RESP_R7:
            flags = (SDHOST_CMD_RESP_LEN_48 | SDHOST_CMD_CRC_CHK_ENABLE | SDHOST_CMD_IDX_CHK_ENABLE);
            break;

        case SDHOST_CMD_RESP_R3:
        case SDHOST_CMD_RESP_R4:
            flags = SDHOST_CMD_RESP_LEN_48;
            break;

        case SDHOST_CMD_RESP_R1B:
            flags = (SDHOST_CMD_RESP_LEN_48BUSY | SDHOST_CMD_CRC_CHK_ENABLE | SDHOST_CMD_IDX_CHK_ENABLE);
            break;

        case SDHOST_CMD_RESP_R2:
            flags = (SDHOST_CMD_RESP_LEN_136 | SDHOST_CMD_CRC_CHK_ENABLE);
            break;

        default:
            flags = SDHOST_CMD_RESP_LEN_ZERO;
            break;
    }

    interrupt = (SDHOST_COMMAND_COMPLETE_INTERRUPT | SDHOST_ERROR_INTERRUPT);
    if (dataPresent)
    {
        /* Configure the transfer mode register. */
        cardCtxt->isDataPresent = true;
        sdhostSetTransferMode(opCode);
        interrupt |= (SDHOST_TRANSFER_COMPLETE_INTERRUPT | SDHOST_DMA_INTERRUPT);
    }
    else
    {
        SDHOST->transferMode = 0;
    }
    
    /* Clear the interrupt status */
    SDHOST->intStatus = 0;
    SDHOST->errIntStatus = 0xFFFF;
    /* Enable the required interrupts */
    SDHOST->intEnable = interrupt;
    SDHOST->errIntEnable = 0;
    SDHOST->intSigEnable = interrupt;
    SDHOST->errIntSigEnable = 0;

    cardCtxt->waitForCmdResp = true;
    cmd = ((opCode << 8) | (dataPresent << 5) | (flags));
    SDHOST->command = cmd;

    return;
}

bool sdhostIsWriteProtected(void)
{
    return (SDHOST->presentState & SDHOST_PSTATE_WRITE_PROTECT) ? 0 : 1;
}

static void sdhostInitVariables(SDHOST_CARD_CTXT *cardCtxt)
{
    cardCtxt->isAttached = false;
    cardCtxt->rca = 0;
    cardCtxt->busWidth = 0;
    cardCtxt->scr = &gSDHOSTScrReg[0];
    cardCtxt->cid = &gSDHOSTCidReg[0];
    cardCtxt->csd = &gSDHOSTCsdReg[0];
    cardCtxt->ocr = &gSDHOSTOcrReg[0];
    cardCtxt->switchStatus = &gSDHOSTSwitchStatusReg[0];

    cardCtxt->cmd6Mode = false;
    cardCtxt->voltWindow = 0;
    /* HC or Normal card. */
    cardCtxt->cardType = 0;
    /* Capacity of the card in number of blocks. */
    cardCtxt->discCapacity = 0;

    cardCtxt->cardVer = 0;
    cardCtxt->writeProtected = 0;
    cardCtxt->locked = 0;

    /* Variables to track the command/data status. */
    cardCtxt->dataCompleted = false;
    cardCtxt->commandCompleted = false;
    cardCtxt->waitForCmdResp = false;
    cardCtxt->waitForData = false;
    cardCtxt->isDataPresent = false;
    cardCtxt->commandError = 0;
    cardCtxt->errorFlag = 0;
}

bool sdhostInit(SDHOST_CARD_CTXT **cardCtxt)
{
	gSDHOSTCtl.hostVer = SDHOST->hostVer & 0xFF;
	if (gSDHOSTCtl.hostVer > SDHOST_HOST_SPEC_VER_3)
	{
        return false;
	}

    /* Issue software reset All */
    SDHOST->swReset = SDHOST_RESET_ALL;

    /* Wait until the host controller reset is completed. */
    while (SDHOST->swReset & SDHOST_RESET_ALL);

	/* Find the maximum clock frequency supported by the Host Controller. */
	gSDHOSTCtl.maxClk = (SDHOST->caps1 & SDHOST_CLOCK_BASE_MASK) >> SDHOST_CLOCK_BASE_SHIFT;
	if (gSDHOSTCtl.maxClk == 0)
    {	
        return false;
	}

    /* Convert to Hertz */
	gSDHOSTCtl.maxClk *= 1000000;
	
    /* Timeout clock to be used for configuring the timeout counter value. */
	gSDHOSTCtl.timeoutClk = (SDHOST->caps1 & SDHOST_TIMEOUT_CLOCK_MASK) >> SDHOST_TIMEOUT_CLOCK_SHIFT;
	if(gSDHOSTCtl.timeoutClk == 0)
    {
        return false;
	}
    
	if (SDHOST->caps1 & SDHOST_TIMEOUT_CLOCK_UNIT)
    {
	    gSDHOSTCtl.timeoutClk *= 1000;
    }
	
    *cardCtxt = (SDHOST_CARD_CTXT *)&gSDHOSTCardCtxt;
    sdhostInitVariables (*cardCtxt);

	/* Set bus voltage to defaut value of 3.3 Volts */
	SDHOST->powerCtl = ((SDHOST_PWR_CTL_3_3V_SEL << 1) | SDHOST_PWR_CTL_BUS_PWR_ON);

    sdhostSetClock (SDHOST_CLOCK_FREQ_400_KHZ);
    sdhostDelay (1000);
    sdhostClockEnable ();
	
	/* Configure the data timeout counter value. */
	SDHOST->timeoutCtl = 0x0E;
	
    /* Host Control register:
     * LED = OFF
     * Bus Width = 1 Bit mode to start with
     * High Speed Enable = false to start with
     * DMA Select = 32-bit address ADMA2
     * Ext Bus Width = 0
     * Card det test level = 0
     * Card Det sig selection = 0 */
    SDHOST->hostCtl = SDHOST_HCTL_DMA_EN_CTRL;
    if (!CFGCON2bits.SDCDEN)
    {
        /* If card detect has not been enabled then enable the card detect
         * signal select. Refer to the SDHC errata. */
        SDHOST->hostCtl |= SDHOST_HCTL_CARD_DET_SIG_SEL;
    }
	
    /* Disable all interrupts. */
    SDHOST->intEnable = 0x0;
    SDHOST->intSigEnable = 0x0;
    return true;
}

void sdhostParseCsd(SDHOST_CARD_CTXT *cardCtxt)
{
	uint8_t  cSizeMultiplier = 0;
    uint16_t blockLength = 0;
    uint32_t cSize = 0;
    uint8_t *csdPtr = NULL;
    uint32_t mult = 0;

    /* Note: The structure format depends on if it is a CSD V1 or V2 device.
       Therefore, need to first determine version of the specs that the card
       is designed for, before interpreting the individual fields.
       */
    csdPtr = cardCtxt->csd;

    /* Bits 127:126 */
    if (((csdPtr[14] >> 6) & 0x03) == 0x01)
    {
        /* CSD Version 2.0 */

        /* TODO: Check if DSR is implemented. Bit 76 */
        //dsr = (csdPtr[8] >> 4) & 0x01;
        /* Extract the C_SIZE field from the response. It is a 22-bit
           number in bit position 69:48. 
           */
        cSize = (csdPtr[7] & 0x3F) << 16;
        cSize |= csdPtr[6] << 8;
        cSize |= csdPtr[5];

        cardCtxt->discCapacity = ((uint32_t)(cSize + 1) * (uint32_t)(1024));
    }
    else
    {
        /* CSD Version 1.0 */

        /* Memory capacity = BLOCKNR * BLOCK_LEN
         * BLOCKNR = (C_SIZE + 1) * MULT
         * MULT = 2 POW(C_SIZE_MULT + 2)
         * BLOCK_LEN = 2 POW(READ_BL_LEN)
         */

        /* READ_BL_LEN Bits 83:80 */
        blockLength = csdPtr[9] & 0x0F;
        blockLength = 1 << (blockLength - 9);

        /* CSIZE Bits 73:62 */
        cSize = (csdPtr[8] & 0x03) << 10;
        cSize |= csdPtr[7] << 2;
        cSize |= csdPtr[6] >> 6;
        
        /* C_SIZE_MULT Bits 49:47 */
        cSizeMultiplier = (csdPtr[4] & 0x03) << 1;  
        cSizeMultiplier |= csdPtr[3] >> 7;

        mult = 1 << (cSizeMultiplier + 2);
        cardCtxt->discCapacity = (((uint32_t)(cSize + 1) * mult) * blockLength);
    }
}


