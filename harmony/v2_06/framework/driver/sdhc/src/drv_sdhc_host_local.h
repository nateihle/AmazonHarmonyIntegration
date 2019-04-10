/*******************************************************************************
  SD Host Controller Driver Interface

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sdhc_host_local.h

  Summary:
    SD Host Controller Driver System Host Definitions

  Description:
    The SD Host Controller driver provides a simple interface to manage the SD 
    Host Controller peripheral.  This file defines the interface definitions 
    and prototypes for the SD Host Controller driver.
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

#ifndef _DRV_SDHC_HOST_LOCAL_H
#define _DRV_SDHC_HOST_LOCAL_H

#include "system_definitions.h"

/* SD Host Controller clock related defines. */
#define SDHOST_CLOCK_BASE_MASK		        0x0000FF00
#define SDHOST_TIMEOUT_CLOCK_MASK  	        0x0000003F
#define SDHOST_TIMEOUT_CLOCK_UNIT 	        0x00000080

#define SDHOST_CLOCK_BASE_SHIFT		        8
#define SDHOST_TIMEOUT_CLOCK_SHIFT 		    0

#define SDHOST_CLOCK_CLK_GEN_PGM_MODE_SEL   (0x10)
#define SDHOST_CLOCK_ENABLE                 (0x04)
#define SDHOST_CLOCK_INTERNAL_CLK_STABLE    (0x02)
#define SDHOST_CLOCK_INTERNAL_CLK_ENABLE    (0x01)

#define SDHOST_CLOCK_FREQ_400_KHZ           (400000)
#define SDHOST_CLOCK_FREQ_DS_25_MHZ         (25000000)
#define SDHOST_CLOCK_FREQ_HS_50_MHZ         (50000000)

/* SD Host Controller transfer mode fields. */
#define SDHOST_XFER_MODE_DMA_ENABLE         (1 << 0)
#define SDHOST_XFER_MODE_BLK_CNT_ENABLE     (1 << 1)
#define SDHOST_XFER_MODE_AUTOCMD_12_ENABLE  (1 << 2)
#define SDHOST_XFER_MODE_AUTOCMD_23_ENABLE  (1 << 3)
#define SDHOST_XFER_MODE_DATA_XFER_DIR      (1 << 4)
#define SDHOST_XFER_MODE_MULTI_BLOCK_SEL    (1 << 5)

#define SDHOST_CMD_IDX_CHK_ENABLE           (1 << 4)
#define SDHOST_CMD_CRC_CHK_ENABLE           (1 << 3)

#define SDHOST_HCTL_DMA_EN_CTRL             0x10
#define SDHOST_HCTL_CARD_DET_TEST_LVL       0x40
#define SDHOST_HCTL_CARD_DET_SIG_SEL        0x80

#define SDHOST_DMA_NUM_DESCR_LINES           (2)

/* Response Types defined at Command register offset(0x0E)*/
#define SDHOST_CMD_RESP_LEN_ZERO            0x0
#define SDHOST_CMD_RESP_LEN_136             0x1
#define SDHOST_CMD_RESP_LEN_48              0x2
#define SDHOST_CMD_RESP_LEN_48BUSY          0x3

/* Power Control Register Fields - Offset(0x29) */
#define SDHOST_PWR_CTL_BUS_PWR_ON          (0x01)
#define SDHOST_PWR_CTL_1_8V_SEL            (0x05)
#define SDHOST_PWR_CTL_3_0V_SEL            (0x06)
#define SDHOST_PWR_CTL_3_3V_SEL            (0x07)

#define SDHOST_PSTATE_CARD_INSERTED        (1 << 16)
#define SDHOST_PSTATE_WRITE_PROTECT        (1 << 19)

#define SDHOST_SCR_REG_LEN (8)
#define SDHOST_CID_REG_LEN (16)
#define SDHOST_OCR_REG_LEN (4)
#define SDHOST_CSD_REG_LEN (16)
#define SDHOST_SWITCH_STATUS_REG_LEN (64)

#define SDHOST_COMMAND_COMPLETE_INTERRUPT  (1 << 0)
#define SDHOST_TRANSFER_COMPLETE_INTERRUPT (1 << 1)
#define SDHOST_DMA_INTERRUPT               (1 << 3)
#define SDHOST_CARD_INS_REM_INTERRUPT      (1 << 6)
#define SDHOST_ERROR_INTERRUPT             (1 << 15)
#define SDHOST_CARD_REMOVAL_INTERRUPT      (1 << 7)
#define SDHOST_CARD_INSERTION_INTERRUPT    (1 << 8)

/* ADMA Descriptor Table Attribute Mask */
#define SDHOST_DESC_TABLE_ATTR_NO_OP          (0x00 << 4)
#define SDHOST_DESC_TABLE_ATTR_RSVD           (0x01 << 4)
#define SDHOST_DESC_TABLE_ATTR_XFER_DATA      (0x02 << 4)
#define SDHOST_DESC_TABLE_ATTR_LINK_DESC      (0x03 << 4)

#define SDHOST_DESC_TABLE_ATTR_VALID          (1 << 0)
#define SDHOST_DESC_TABLE_ATTR_END            (1 << 1)
#define SDHOST_DESC_TABLE_ATTR_INTR           (1 << 2)

typedef struct SDHOST_CONTROLLER
{
    uint8_t hostVer;
    uint32_t timeoutClk;
    uint32_t maxClk;
    uint32_t caps1;
    uint32_t caps2;
} SDHOST_CONTROLLER;

typedef struct SDHOST_ADMA_DESCR
{
    uint16_t attribute;    
    uint16_t length;
    uint32_t address;
} SDHOST_ADMA_DESCR;

typedef struct __attribute__((packed))
{
    /* Offset 0x00 */
    volatile uint32_t sdmaSysAddr;
    volatile uint16_t blockSize;
    volatile uint16_t blockCount;
    volatile uint32_t argument;
    volatile uint16_t transferMode;
    volatile uint16_t command;
    /* Offset 0x10 */
    volatile uint32_t response0;
    volatile uint32_t response1;
    volatile uint32_t response2;
    volatile uint32_t response3;
    /* Offset 0x20 */
    volatile uint32_t bufferDataPort;
    volatile uint32_t presentState;
    volatile uint8_t  hostCtl;
    volatile uint8_t  powerCtl;
    volatile uint8_t  blockGapCtl;
    volatile uint8_t  wakeupCtl;
    volatile uint16_t clockCtl;
    volatile uint8_t  timeoutCtl;
    volatile uint8_t  swReset;
    /* Offset 0x30 */
    volatile uint16_t intStatus;
    volatile uint16_t errIntStatus;
    volatile uint16_t intEnable;
    volatile uint16_t errIntEnable;
    volatile uint16_t intSigEnable;
    volatile uint16_t errIntSigEnable;
    volatile uint16_t autoCmdErrStatus;
    volatile uint16_t hostCtl2;
    /* Offset 0x40 */
    volatile uint32_t caps1;
    volatile uint32_t caps2;
    volatile uint32_t currentCaps1;
    volatile uint32_t currentCaps2;
    /* Offset 0x50 */
    volatile uint16_t forceEvtAutoCmdErrStatus;
    volatile uint16_t forceEvtErrIntStatus;
    volatile uint8_t  admaErrStatus;
    volatile uint8_t  rsvd1;
    volatile uint16_t rsvd2;
    volatile uint32_t admaSysAddress1;
    volatile uint32_t admaSysAddress2;
    /* Offset 0x60 */
    volatile uint16_t presetValInit;
    volatile uint16_t presetValDefSpeed;
    volatile uint16_t presetValHighSpeed;
    volatile uint16_t presetValSDR12;
    volatile uint16_t presetValSDR25;
    volatile uint16_t presetValSDR50;
    volatile uint16_t presetValSDR104;
    volatile uint16_t presetValDDR50;

    /* Reserved from 0x70 till 0xDF */
    volatile uint16_t rsvd3[56];

    /* 0xE0 */
    volatile uint32_t sharedBusCtl;

    /* Reserved from 0xE4 till 0xFB */
    volatile uint16_t rsvd4[12];

    /* Offset 0xFC */
    volatile uint16_t slotIntStatus;

    /* Offset 0xFE */
    volatile uint8_t hostVer;

} SDHOSTRegs_t;

#define SDHOST ((SDHOSTRegs_t *)(((uint32_t)&SDHCBLKCON) - 4))

#endif

