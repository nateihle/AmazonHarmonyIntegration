/*******************************************************************************
  SQI Driver Interface Declarations for Static Single Instance Driver

  Company:
    Microchip Technology Inc.

  File Name:
    app_sqi_static.h

  Summary:
    SQI driver interface declarations for the static single instance driver.

  Description:
    The SQI device driver provides a simple interface to manage the SQI
    modules on Microchip microcontrollers. This file defines the interface
    Declarations for the SQI driver.

  Remarks:
    Static interfaces incorporate the driver instance number within the names
    of the routines, eliminating the need for an object ID or object handle.

    Static single-open interfaces also eliminate the need for the open handle.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2015 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _APP_SQI_STATIC_H
#define _APP_SQI_STATIC_H

// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************
/* Note:  A file that maps the interface definitions above to appropriate static
          implementations (depending on build mode) is included at the bottom of
          this file.
*/
#include <stdbool.h>
#include "sys/kmem.h"
#include "system_config.h"
#include "peripheral/sqi/plib_sqi.h"

// *****************************************************************************
// *****************************************************************************
// Section: Enumerations
// *****************************************************************************
// *****************************************************************************
typedef enum {

    APP_SQI_ID_0 = SQI_ID_0,
    APP_SQI_NUMBER_OF_MODULES

} APP_SQI_MODULE_ID;

typedef enum {

    APP_SQI_XFER_MODE_XIP = SQI_XFER_MODE_XIP,
    APP_SQI_XFER_MODE_DMA = SQI_XFER_MODE_DMA,
    APP_SQI_XFER_MODE_PIO = SQI_XFER_MODE_PIO

} APP_SQI_XFER_MODE;

typedef enum {

    APP_SQI_DATA_MODE_3 = SQI_DATA_MODE_3,
    APP_SQI_DATA_MODE_0 = SQI_DATA_MODE_0

} APP_SQI_DATA_MODE;

typedef enum {

    APP_SQI_DATA_FORMAT_LSBF = SQI_DATA_FORMAT_LSBF,
    APP_SQI_DATA_FORMAT_MSBF = SQI_DATA_FORMAT_MSBF

} APP_SQI_DATA_FORMAT;

typedef enum {

    APP_SQI_DATA_OEN_QUAD = SQI_DATA_OEN_QUAD,
    APP_SQI_DATA_OEN_DUAL = SQI_DATA_OEN_DUAL,
    APP_SQI_DATA_OEN_SINGLE = SQI_DATA_OEN_SINGLE

} APP_SQI_DATA_OEN;

typedef enum {

    APP_SQI_CS_OEN_BOTH = SQI_CS_OEN_BOTH,
    APP_SQI_CS_OEN_1 = SQI_CS_OEN_1,
    APP_SQI_CS_OEN_0 = SQI_CS_OEN_0,
    APP_SQI_CS_OEN_NONE = SQI_CS_OEN_NONE

} APP_SQI_CS_OEN;

typedef enum {

    APP_SQI_CMD_RECEIVE = SQI_CMD_RECEIVE,
    APP_SQI_CMD_TRANSMIT = SQI_CMD_TRANSMIT,
    APP_SQI_CMD_IDLE = SQI_CMD_IDLE

} APP_SQI_XFER_CMD;

typedef enum {

    APP_SQI_LANE_QUAD = SQI_LANE_QUAD,
    APP_SQI_LANE_DUAL = SQI_LANE_DUAL,
    APP_SQI_LANE_SINGLE = SQI_LANE_SINGLE

} APP_SQI_LANE_MODE;

typedef enum {

    APP_SQI_CS_1 = SQI_CS_1,
    APP_SQI_CS_0 = SQI_CS_0

} APP_SQI_CS_NUM;

typedef enum {

    APP_ADDR_BYTES_4 = ADDR_BYTES_4,
    APP_ADDR_BYTES_3 = ADDR_BYTES_3,
    APP_ADDR_BYTES_2 = ADDR_BYTES_2,
    APP_ADDR_BYTES_1 = ADDR_BYTES_1,
    APP_ADDR_BYTES_0 = ADDR_BYTES_0

} APP_SQI_ADDR_BYTES;

typedef enum {

    APP_DUMMY_BYTES_7 = DUMMY_BYTES_7,
    APP_DUMMY_BYTES_6 = DUMMY_BYTES_6,
    APP_DUMMY_BYTES_5 = DUMMY_BYTES_5,
    APP_DUMMY_BYTES_4 = DUMMY_BYTES_4,
    APP_DUMMY_BYTES_3 = DUMMY_BYTES_3,
    APP_DUMMY_BYTES_2 = DUMMY_BYTES_2,
    APP_DUMMY_BYTES_1 = DUMMY_BYTES_1,
    APP_DUMMY_BYTES_0 = DUMMY_BYTES_0

} APP_SQI_DUMMY_BYTES;

typedef enum {

    APP_MODE_BYTES_3 = MODE_BYTES_3,
    APP_MODE_BYTES_2 = MODE_BYTES_2,
    APP_MODE_BYTES_1 = MODE_BYTES_1,
    APP_MODE_BYTES_0 = MODE_BYTES_0

} APP_SQI_MODE_BYTES;

typedef enum {

    APP_SQI_DMAERROR = 11,
    APP_SQI_PKTCOMP = SQI_PKTCOMP,
    APP_SQI_BDDONE = SQI_BDDONE,
    APP_SQI_CONTHR = SQI_CONTHR,
    APP_SQI_CONEMPTY = SQI_CONEMPTY,
    APP_SQI_CONFULL = SQI_CONFULL,
    APP_SQI_RXTHR = SQI_RXTHR,
    APP_SQI_RXEMPTY = SQI_RXEMPTY,
    APP_SQI_RXFULL = SQI_RXFULL,
    APP_SQI_TXTHR = SQI_TXTHR,
    APP_SQI_TXEMPTY = SQI_TXEMPTY,
    APP_SQI_TXFULL = SQI_TXFULL

} APP_SQI_INTERRUPTS;

typedef enum {

    APP_BD_STATE_DISABLED = BD_STATE_DISABLED,
    APP_BD_STATE_DONE = BD_STATE_DONE,
    APP_BD_STATE_PROCESSING_DATA = BD_STATE_PROCESSING_DATA,
    APP_BD_STATE_BEING_FETCHED = BD_STATE_BEING_FETCHED,
    APP_BD_STATE_FETCH_REQ_PENDING = BD_STATE_FETCH_REQ_PENDING,
    APP_BD_STATE_IDLE = BD_STATE_IDLE

} APP_SQI_BD_STATE;

// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
#define MAX_WRITE_BUF_DEPTH                 16
#define MAX_READ_BUF_DEPTH                  8
#define XIP_UNCACHED_ADDR_MASK              0xF0000000
#define XIP_CACHED_ADDR_MASK                0xD0000000
#define SQI_NUM_BUFFER_DESC                 256

//Flash Commands
#define FLASH_NOP                           0x00
#define FLASH_RSTEN							0x66
#define FLASH_RST							0x99
#define FLASH_EQIO                          0x38
#define FLASH_WEN                           0x06
#define FLASH_UNPROTECT_GLOBAL              0x98
#define FLASH_UNPROTECT						0x42
#define FLASH_QJID                          0xAF
#define FLASH_ERASE                         0xC7
#define FLASH_PAGE_WRITE                    0x02
#define FLASH_FAST_READ                     0x0B
#define FLASH_RDSR							0x05
#define FLASH_BUSY							0x01
#define FLASH_READY							0x00
#define FLASH_BUSY_MASK                     0x08
#define FLASH_BUSY_MASK_DUPLICATE			0x01

#define FLASH_PAGE_SIZE						256

//Flash device ID
#define FLASH_JEDECID					    0x0226BF
#define FLASH_JEDECID_B					    0x4226BF

// *****************************************************************************
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


typedef enum
{
    /* An error has occurred.*/
    SQI_STATUS_FAILURE = 0,

    /* No errors occurred*/
    SQI_STATUS_SUCCESS = 1,

} SQI_STATUS;

// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for SQI Static Driver
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    void APP_SQI_Initialize(void);

  Summary:
    Initializes Serial Quad Interface (SQI) module.
    <p><b>Implementation:</b> Static</p>

  Description:
	This routine is used to initialize SQI module by setting configuration
	words and clock services.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_SQI_Initialize();
    </code>

  Remarks:
    None.
*/
void APP_SQI_Initialize(void);

// *****************************************************************************
/* Function:
    void APP_SQI_Flash_Initialize(void);

  Summary:
    Initializes external quad flash.
    <p><b>Implementation:</b> Static</p>

  Description:
	This routine is used to initialize external quad flash by unprotecting it for
	ERASE/READ/WRITE operations

  Precondition:
    APP_SQI_Initialize should be called before using this function.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_SQI_Flash_Initialize();
    </code>

  Remarks:
    APP_SQI_Initialize function calls this before exiting.
*/
void APP_SQI_Flash_Initialize(void);

// *****************************************************************************
/* Function:
    void APP_SQI_Flash_Erase(void);

  Summary:
    Erase external quad flash.
    <p><b>Implementation:</b> Static</p>

  Description:
	This routine when called will issue commands to erase the external flash.

  Precondition:
    APP_SQI_Initialize should be called before using this function.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_SQI_Flash_Erase();
    </code>

  Remarks:
    None.
*/
void APP_SQI_Flash_Erase (void);

// *****************************************************************************
/* Function:
    int APP_SQI_Flash_ID_Check(void);

  Summary:
    Read external flash ID.
    <p><b>Implementation:</b> Static</p>

  Description:
	This routine when called will issue commands to read the external flash ID,
	checks against expected value and returns true/false.

  Precondition:
    APP_SQI_Initialize should be called before using this function.

  Parameters:
    None.

  Returns:
    SQI_STATUS (bool).

  Example:
    <code>
    bool IDReadStatus = APP_SQI_Flash_ID_Check();

	if ( sqiStatus !=  true)
        appData.state = APP_STATE_CURRENT;
    else
        appData.state = APP_STATE_NEXT;

    </code>

  Remarks:
    None.
*/
SQI_STATUS APP_SQI_Flash_ID_Check (void);

// *****************************************************************************
/* Function:
    void APP_SQI_PIO_PageWrite (uint32_t flashAddress, uint8_t* writeBuffer);

  Summary:
    Write external flash page.
    <p><b>Implementation:</b> Static</p>

  Description:
	This routine when called will issue commands to write external flash page.

  Precondition:
    APP_SQI_Initialize should be called before using this function.

  Parameters:
    flashAddress - Page address in flash.
	writeBuffer  - Pointer to data to be written

  Returns:
    None.

  Example:
    <code>
    APP_SQI_PIO_PageWrite (FLASH_PAGE_ADDRESS, writeBuffer);
    </code>

  Remarks:
    None.
*/
void APP_SQI_PIO_PageWrite (uint32_t flashAddress, uint8_t* writeBuffer);

// *****************************************************************************
/* Function:
   void APP_SQI_PIO_PageRead(uint32_t flashAddress, uint8_t* readBuffer);

  Summary:
    Read external flash page using PIO mode.
    <p><b>Implementation:</b> Static</p>

  Description:
	This routine when called will issue commands to read external flash page using
	PIO mode.

  Precondition:
    APP_SQI_Initialize should be called before using this function.

  Parameters:
    flashAddress - Page address in flash
	readBuffer   - Pointer to data to be read

  Returns:
    None.

  Example:
    <code>
    APP_SQI_PIO_PageRead(FLASH_ADDRESS, FLASH_READ_COUNT, readBuffer);
    </code>

  Remarks:
    This routine read one page at a time.
*/
void APP_SQI_PIO_PageRead(uint32_t flashAddress, uint32_t* readBuffer);

// *****************************************************************************
/* Function:
   uint8_t APP_SQI_FlashIsBusy(void);

  Summary:
    Check external flash status.
    <p><b>Implementation:</b> Static</p>

  Description:
	This routine when called will check the external flash status through
	APP_SQI_FlashReadStatus() function and checks the availability.

  Precondition:
    APP_SQI_Initialize should be called before using this function.

  Parameters:
    None.

  Returns:
    READY/BUSY status.

  Example:
    <code>
    readStatus = APP_SQI_FlashIsBusy();
    </code>

  Remarks:
    None.
*/
uint8_t APP_SQI_FlashIsBusy(void);

// *****************************************************************************
/* Function:
   uint8_t APP_SQI_FlashReadStatus(void);

  Summary:
    Read external flash status.
    <p><b>Implementation:</b> Static</p>

  Description:
	This routine when called will issue commands to read external flash status.

  Precondition:
    APP_SQI_Initialize should be called before using this function.

  Parameters:
    None.

  Returns:
    STATUS register value.

  Example:
    <code>
    statusRegisterValue = APP_SQI_FlashReadStatus();
    </code>

  Remarks:
    None.
*/
uint8_t APP_SQI_FlashReadStatus(void);

#endif // #ifndef _APP_SQI_STATIC_H

/*******************************************************************************
 End of File
*/
