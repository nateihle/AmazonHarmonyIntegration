/*
SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS” WITHOUT WARRANTY OF ANY

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


	Filename:	drv_nvm_flash_sqi_sst26.h
 	Author:		Fergus O'Kane/Mihai Paiu
	Reviewer:
	Date:		21Jan15
	Processor:	PIC32MZ2048ECH144
	Compiler:	XC32 v1.34
 	Notes:

*/
#ifndef _DRV_NVM_FLASH_SQI_SST26_H
#define _DRV_NVM_FLASH_SQI_SST26_H

// include files
#include <stdint.h>
#include "sys/kmem.h"
#include "peripheral/sqi/plib_sqi.h"

// defines
// number of descriptor buffers for reading data
#define SQI_NUM_BUFFER_DESC         256
// buffer needed for SQI reads to avoid exceptions if the target buffer is not aligned and coherent
// keep it at least 512 bytes
#define SQI_BUFFER_SIZE             512

// SST26 commands
#define SST26VF_ERASE               0xC7
#define SST26VF_EQIO                0x38
#define SST26VF_WEN                 0x06
#define SST26VF_WRDI                0x04
#define SST26VF_NOP                 0x00
#define SST26VF_WBPR                0x42
#define SST26VF_ULBPR               0x98
#define SST26VF_PAGE_WRITE          0x02
#define SST26VF_FAST_READ           0x0B
#define SST26VF_RDSR                0x05
#define SST26VF_QJID                0xAF
#define SST26VF_SE                  0x20
#define SST26VF_RQIO                0xFF

#define SST26VF_RSTEN               0x66
#define SST26VF_RESET               0x99

#define SST26VF_JEDECID            0x000026BF
#define SST26VF_JEDECID_MASK       0x0000FFFF

#define DUMMY_BYTE                  0x00
#define DUMMY_2_BYTES               0x0000
#define DUMMY_3_BYTES               0x000000
#define DUMMY_4_BYTES               0x00000000

#define SST26_PAGE_SIZE             0x100
#define ADDRESS_PAGE_MASK           0xFFFFFF00
#define ADDRESS_SECTOR_MASK         0xFFFFF000
#define HIGH_SPEED_IN_MODE_READ     0xAF
#define HIGH_SPEED_NOT_IN_MODE_READ 0xCF

// bit masks
#define SST26_BUSY_MASK_01          0x01 // Write Operation Status
#define SST26_WEL_MASK              0x02 // Write-Enable Latch Status
#define SST26_WSE_MASK              0x04 // Write Suspend-Erase Status
#define SST26_WSP_MASK              0x08 // Write Suspend-Program Status
#define SST26_WPLD_MASK             0x10 // Wirte Protection Lock Down Status
#define SST26_SEC_MASK              0x20 // Security ID Status
#define SST26_RES_MASK              0x40 // Reserved
#define SST26_BUSY_MASK             0x80 // Write Operation Status

#define SST26_BIT_IS_SET            0x01
#define SST26_BIT_IS_NOT_SET        0x00

#define SST26_IS_BUSY               0x01
#define SST26_NOT_BUSY              0x00

#define SST26_IS_WRITE_ENABLE       0x01
#define SST26_NOT_WRITE_ENABLE      0x00

#define PIC32_KVA0_TO_KVA1_VAR(v)   (*(typeof(v)*)((unsigned long)&(v) | 0x20000000u))
#define PIC32_KVA0_TO_KVA1_PTR(v)   ((typeof(v)*)((unsigned long)(v) | 0x20000000u))
#define PIC32_UNCACHED_VAR(v)       PIC32_KVA0_TO_KVA1_VAR(v)
#define PIC32_UNCACHED_PTR(v)       PIC32_KVA0_TO_KVA1_PTR(v)

// SQI settings structure for initialization
typedef struct
{
	SQI_CS_OEN csPins;
	SQI_DATA_MODE dataMode;
	SQI_CLK_DIV clkDivider;
} DRV_SQI_INIT_DATA;

/* SQI DMA descriptor

  Summary:
    Holds DMA descriptor data

  Description:
    This structure holds the DMA descriptor data.

  Remarks:
    None.
 */
typedef struct
{
    // Buffer Descriptor Control Word
    unsigned int BDCon;

    // Buffer Descriptor Status Word - reserved.
    unsigned int BDStat;

    // Buffer Address.
    unsigned int *BDAddr;

    // Next Buffer Descriptor Address Pointer
    struct sqiDMADesc *nextBDAddr;

} sqiDMADesc;

/*
 * Status of the SQI Flash connected with the PIC
 */
typedef enum
{
    /* An error has occurred. */
    SQI_STATUS_FAILURE = 0,

    /* No errors occurred */
    SQI_STATUS_SUCCESS = 1,

} SQI_STATUS;

// function prototypes
void SST26Init(DRV_SQI_INIT_DATA *pInitData);
void SST26WriteByte(uint32_t address, uint8_t data);
uint8_t SST26ReadByte(uint32_t address);
void SST26WriteEnable(void);
uint8_t SST26IsWriteBusy(void);
uint8_t SST26ReadStatus(void);

uint8_t SST26WriteArray(uint32_t address, uint8_t *pData, uint32_t nCount);
void SST26ReadArray(uint32_t address, uint8_t *pData, uint32_t nCount);
void SST26ChipErase(void);
void SST26ResetWriteProtection(void);
void SST26SectorErase(uint32_t address);
uint32_t SST26WritePagePIO(uint32_t address, uint8_t *pData, uint32_t nCount);
uint32_t SST26WritePageDMA(uint32_t address, uint8_t *pData, uint32_t nCount);
SQI_STATUS SST26_SQI_FlashID_Read(void);

#endif // _DRV_NVM_FLASH_SQI_SST26_H
