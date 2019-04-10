/*******************************************************************************
  SST25 SPI Flash Driver Local Data Structures

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sst25_local.h

  Summary:
    SST25 SPI Flash Driver Local Data Structures

  Description:
    Driver Local Data Structures
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute Software
only when embedded on a Microchip microcontroller or digital  signal  controller
that is integrated into your product or third party  product  (pursuant  to  the
sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS  WITHOUT  WARRANTY  OF  ANY  KIND,
EITHER EXPRESS  OR  IMPLIED,  INCLUDING  WITHOUT  LIMITATION,  ANY  WARRANTY  OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A  PARTICULAR  PURPOSE.
IN NO EVENT SHALL MICROCHIP OR  ITS  LICENSORS  BE  LIABLE  OR  OBLIGATED  UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,  BREACH  OF  WARRANTY,  OR
OTHER LEGAL  EQUITABLE  THEORY  ANY  DIRECT  OR  INDIRECT  DAMAGES  OR  EXPENSES
INCLUDING BUT NOT LIMITED TO ANY  INCIDENTAL,  SPECIAL,  INDIRECT,  PUNITIVE  OR
CONSEQUENTIAL DAMAGES, LOST  PROFITS  OR  LOST  DATA,  COST  OF  PROCUREMENT  OF
SUBSTITUTE  GOODS,  TECHNOLOGY,  SERVICES,  OR  ANY  CLAIMS  BY  THIRD   PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE  THEREOF),  OR  OTHER  SIMILAR  COSTS.
*******************************************************************************/
//DOM-IGNORE-END

#ifndef _DRV_SST25_LOCAL_H
#define _DRV_SST25_LOCAL_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "driver/spi_flash/sst25/drv_sst25.h"
#include "driver/spi_flash/sst25/src/drv_sst25_variant_mapping.h"
#include "osal/osal.h"
#include "system/debug/sys_debug.h"

// *****************************************************************************
// *****************************************************************************
// Section: Constant Definitions
// *****************************************************************************
// *****************************************************************************

/* op-code definitions for different operations */

/* Read Memory at Normal Speed */
#define DRV_SST25_CMD_READ                  (0x03)

/* Read Memory at High Speed */
#define DRV_SST25_CMD_HIGH_SPEED_READ       (0x0B)

/* Erase 4 KByte of Memory Area */
#define DRV_SST25_CMD_BLOCK_ERASE           (0x20)

/* Chip Erase */
#define DRV_SST25_CMD_CHIP_ERASE            (0x60)

/* Page program */
#define DRV_SST25_CMD_PAGE_PROGRAM          (0x02)

/* AAI Program */
#define DRV_SST25_CMD_AAI_PROGRAM           (0xAD)

/* Opcode for AAI is 0xAF for SST25VF010A */
#define DRV_SST25_CMD_AAI_PROGRAM1          (0xAF)

/* Read Status Register */
#define DRV_SST25_CMD_READ_STATUS_REG       (0x05)

/* Write Status Register */
#define DRV_SST25_CMD_WRITE_STATUS_REG      (0x01)

/* Write Enable */
#define DRV_SST25_CMD_WRITE_ENABLE          (0x06)

/* Write Disable */
#define DRV_SST25_CMD_WRITE_DISABLE         (0x04)

/* Read the device id. */
#define DRV_SST25_CMD_READ_ID               (0x90)


#define DRV_SST25_CMD_ENABLE_WRITE_STATUS_REG (0x50)

#define _DRV_SST25_CHIP_SELECT(port,pin)    SYS_PORTS_PinClear(PORTS_ID_0,port,pin)
#define _DRV_SST25_CHIP_DESELECT(port,pin) 	SYS_PORTS_PinSet(PORTS_ID_0,port,pin)

#define DRV_SST25_PAGE_SIZE         (256)
#define DRV_SST25_ERASE_SECTOR_SIZE (4096)

#define DRV_SST25_NUM_DEVICE_SUPPORTED  (6)

typedef struct
{
    DRV_SPI_BUFFER_EVENT (*unlock) (void *dObj);
    DRV_SPI_BUFFER_EVENT (*read) (void *dObj, uint8_t *data, uint32_t blockStart, uint32_t nBlocks);
    DRV_SPI_BUFFER_EVENT (*write) (void *dObj, uint8_t *data, uint32_t blockStart, uint32_t nBlocks);
    DRV_SPI_BUFFER_EVENT (*erase) (void *dObj, uint32_t blockStart, uint32_t nBlocks);
    DRV_SPI_BUFFER_EVENT (*chipErase) (void *dObj);
    DRV_SPI_BUFFER_EVENT (*eraseWrite) (void *dObj, uint8_t *data, uint32_t blockStart, uint32_t nBlocks);

} DRV_SST25_FLASH_FUNCTIONS;

typedef struct DRV_SST25_FLASH_OPCODES
{
    uint8_t read;
    uint8_t write;
    uint8_t erase;
    uint8_t chipErase;

    uint8_t writeEnable;
    uint8_t writeDisable;

    uint8_t readStatus;
    uint8_t writeStatus;
    uint8_t enableWriteStatus;

} DRV_SST25_FLASH_OPCODES;

// *****************************************************************************
/* SST25 Read/Write/Erase Region Index Numbers

  Summary:
    SST25 Geometry Table Index definitions.

  Description:
    These constants provide SST25 Geometry Table index definitions.

  Remarks:
    None
*/
#define DRV_SST25_GEOMETRY_TABLE_READ_ENTRY   (0)
#define DRV_SST25_GEOMETRY_TABLE_WRITE_ENTRY  (1)
#define DRV_SST25_GEOMETRY_TABLE_ERASE_ENTRY  (2)
// *****************************************************************************
/* SST Flash Driver Buffer Handle Macros

  Summary:
    SST Flash driver Buffer Handle Macros

  Description:
    Buffer handle related utility macros. SST Flash driver buffer handle is a
    combination of buffer token and the buffer object index. The buffer token
    is a 16 bit number that is incremented for every new write, read or erase
    request and is used along with the buffer object index to generate a new
    buffer handle for every request.

  Remarks:
    None
*/

#define DRV_SST25_BUF_TOKEN_MAX         (0xFFFF)
#define DRV_SST25_MAKE_HANDLE(token, index) ((token) << 16 | (index))
#define DRV_SST25_UPDATE_BUF_TOKEN(token) \
{ \
    (token)++; \
    (token) = ((token) == DRV_SST25_BUF_TOKEN_MAX) ? 0: (token); \
}

// *****************************************************************************
// *****************************************************************************
// Section: Data Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* SST25 Driver operations.

  Summary:
    Enumeration listing the SST25 driver operations.

  Description:
    This enumeration defines the possible SST25 driver operations.

  Remarks:
    None
*/

typedef enum 
{
    /* Request is read operation. */
    DRV_SST25_OPERATION_TYPE_READ = 0,

    /* Request is write operation. */
    DRV_SST25_OPERATION_TYPE_WRITE,

    /* Request is erase operation. */
    DRV_SST25_OPERATION_TYPE_ERASE,

    /* Request is chip erase operation. */
    DRV_SST25_OPERATION_TYPE_CHIP_ERASE,

    /* Request is erase write operation. */
    DRV_SST25_OPERATION_TYPE_ERASE_WRITE

} DRV_SST25_OPERATION_TYPE;

typedef enum
{
    /* Init state. */
    DRV_SST25_WRITE_INIT = 0,

    /* Send write enable command. */
    DRV_SST25_WRITE_ENABLE_WRITE,

    /* Send write command and the data to be written. */
    DRV_SST25_WRITE_PROGRAM_DATA,

    /* Check for the completion of the write command. */
    DRV_SST25_WRITE_PROGRAM_DATA_STATUS,

    /* Poll the status register. */
    DRV_SST25_WRITE_READ_STATUS_REGISTER,

    /* Send write disable command. */
    DRV_SST25_WRITE_DISABLE_WRITE,

    /* Check the status of the write disable command. */
    DRV_SST25_WRITE_DISABLE_WRITE_STATUS

} DRV_SST25_WRITE_STATES;

typedef enum
{
    /* Init state.*/
    DRV_SST25_ERASE_INIT = 0,

    /* Write Enable for erase. */
    DRV_SST25_ERASE_ENABLE_WRITE,

    /* Send Erase command. */
    DRV_SST25_ERASE_ERASE_CMD,

    /* Check for the completion of the erase command. */
    DRV_SST25_ERASE_ERASE_CMD_STATUS,

    /* Poll the status register. */
    DRV_SST25_ERASE_READ_STATUS_REGISTER

} DRV_SST25_ERASE_STATES;

typedef enum
{
    /* Init state. */
    DRV_SST25_EW_INIT = 0,

    /* Read the sector. */
    DRV_SST25_EW_READ_SECTOR,

    /* Erase the sector. */
    DRV_SST25_EW_ERASE_SECTOR,

    /* Overlay the new data. */
    DRV_SST25_EW_WRITE_SECTOR

} DRV_SST25_ERASE_WRITE_STATES;

typedef enum
{
    /* Send WREN command to the SPI Flash. */
    DRV_SST25_WRITE_ENABLE = 0,

    /* Check the status of the WREN command. */
    DRV_SST25_WRITE_ENABLE_STATUS,

    /* Send command to read the status register. */
    DRV_SST25_STATUS_REG_WRITE_CMD,

    /* Check the status of the command and queue command to read the register.
     * */
    DRV_SST25_STATUS_REG_CHECK_WRITE_STATUS,

    /* Wait for the Read to complete. */
    DRV_SST25_STATUS_REG_CHECK_READ_STATUS,

    /* Send down the read command and address. */
    DRV_SST25_READ_INIT,

    /* Queue command to read the data. */
    DRV_SST25_READ_TRANSFER_DATA,

    /* Wait for the command to complete. */
    DRV_SST25_READ_TRANSFER_DATA_STATUS,

    /* Send down the command to read the id of the device. */
    DRV_SST25_READ_ID,

    /* Wait for the command to complete. Queue another read to read the device
     * id. */
    DRV_SST25_READ_ID_STATUS,

    /* Wait for the device id read to complete. */
    DRV_SST25_READ_ID_DATA_STATUS,

    /* Unlock writes to the WRSR by writing to EWSR register. */
    DRV_SST25_UNLOCK_ENABLE_WRSR_REG,

    /* Write the unlock command with required parameters. */
    DRV_SST25_UNLOCK_WRSR_CMD,

    /* Check the status of the unlock command */
    DRV_SST25_UNLOCK_WRSR_CMD_STATUS

} DRV_SST25_SUBTASK_STATES;

typedef enum
{
    /* Open the SPI driver */
    DRV_SST25_TASK_OPEN_SPI_DRIVER = 0,

    /* Unlock the SPI Flash. */
    DRV_SST25_TASK_UNLOCK_FLASH,

    /* Read the flash Id. */
    DRV_SST25_TASK_READ_FLASH_ID,

    /* Process the queued requests. */
    DRV_SST25_TASK_PROCESS_QUEUE,

    /* Perform the required operation. */
    DRV_SST25_TASK_TRANSFER,

    /* Idle state of the task. */
    DRV_SST25_TASK_IDLE,

    /* Error state */
    DRV_SST25_TASK_ERROR

} DRV_SST25_TASK_STATES;

// *****************************************************************************
/* SST25 SPI Flash Driver Client Object

  Summary:
    Object used to track a single client.

  Description:
    This object is used to keep the data necesssary to keep track of a single 
    client.

  Remarks:
    None.
*/

typedef struct
{
    /* The hardware instance object associated with the client */
    void *driverObj;

    /* The IO intent with which the client was opened */
    DRV_IO_INTENT intent;

    /* This flags indicates if the object is in use or is available */
    bool inUse;

    /* Event handler for this function */
    DRV_SST25_EVENT_HANDLER eventHandler;

    /* Application Context associated with this client */
    uintptr_t context;

} DRV_SST25_CLIENT_OBJ;

// *****************************************************************************
/* SST25 SPI Flash Driver Buffer Object

  Summary:
    Object used to keep track of a client's buffer.

  Description:
    This object is used to keep track of a client's buffer in the driver's 
    queue.

  Remarks:
    None.
*/

typedef struct DRV_SST25_BUFFER_OBJ
{
    /* True if object is allocated */
    bool inUse;

    /* Client that owns this buffer */
    DRV_SST25_CLIENT_OBJ *hClient;

    /* Present status of this command */
    DRV_SST25_COMMAND_STATUS status;

    /* Pointer to the next buffer in the queue */
    struct DRV_SST25_BUFFER_OBJ * next;

    /* Pointer to the previous buffer in the queue */
    struct DRV_SST25_BUFFER_OBJ * previous;

    /* Current command handle of this buffer object */
    DRV_SST25_BLOCK_COMMAND_HANDLE commandHandle;

    /* Pointer to the source/destination buffer */
    uint8_t *buffer;

    /* Start address of the operation. */
    uint32_t blockStart;

    /* Number of blocks */
    uint32_t nBlocks;

    /* Operation type - read/write/erase/chip erase/erasewrite */
    DRV_SST25_OPERATION_TYPE opType;

} DRV_SST25_BUFFER_OBJ;

// *****************************************************************************
/* SST25 SPI Flash Driver Instance Object

  Summary:
    Object used to keep any data required for an instance of the SST25 SPI
    Flash driver.

  Description:
    This object is used to keep track of any data that must be maintained to 
    manage a single instance of the SPI Flash driver.

  Remarks:
    None.
*/

typedef struct
{
    /* The status of the driver */
    SYS_STATUS status;

    bool inUse;
    bool isExclusive;
    bool disableCs;
    uint8_t numClients;

    DRV_SST25_TASK_STATES state;
    DRV_SST25_SUBTASK_STATES subState;
    DRV_SST25_ERASE_WRITE_STATES ewState;
    DRV_SST25_WRITE_STATES writeState;
    DRV_SST25_ERASE_STATES eraseState;

    uint8_t *cmdParams;
    uint8_t cmdParamsLen;
    uint8_t cmdOpCode;
    uint8_t *data;
    uint8_t *ewBuffer;
    uint32_t nBlocks;
    uint32_t blockAddress;
    uint32_t sectorAddress;
    uint32_t blockOffsetInSector;
    uint32_t nBlocksToWrite;

    uint8_t *flashId;
    uint8_t *statusReg;

    DRV_SST25_BUFFER_OBJ *queue;

    /* Chip select pin port channel */
    PORTS_CHANNEL csPort;

    /* Chip Select Bit pin position */
    PORTS_BIT_POS csPin;

    /* The module index of associated SPI driver */
    SYS_MODULE_INDEX spiDriverIndex;
    
    /* SPI Driver Handle */
    DRV_HANDLE spiDriverHandle;

    DRV_SST25_BUFFER_OBJ *currentBufObj;

    /* SPI Buffer Handle */
    DRV_SPI_BUFFER_HANDLE spiBufferHandle;

    /* SPI Driver client specific data. */
    DRV_SPI_CLIENT_DATA spiClientData;

    /* Flash functions */
    DRV_SST25_FLASH_FUNCTIONS flashFunctions;

    /* Flash specific opcodes */
    DRV_SST25_FLASH_OPCODES opCodes;

    /* SST25 driver geometry object */
    SYS_FS_MEDIA_GEOMETRY mediaGeometryObj;

    /* SST25 driver media geometry table. */
    SYS_FS_MEDIA_REGION_GEOMETRY mediaGeometryTable[3];

    /* Hardware instance mutex */
    OSAL_MUTEX_DECLARE(mutex);

} DRV_SST25_OBJ;

#endif //#ifndef _DRV_SST25_LOCAL_H

/*******************************************************************************
 End of File
*/

