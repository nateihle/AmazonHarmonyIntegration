/*******************************************************************************
  S25FL Driver Local Data Structures

  Company:
    Microchip Technology Inc.

  File Name:
    drv_s25fl_local.h

  Summary:
    S25FL driver local declarations and definitions

  Description:
    This file contains the S25FL driver's local declarations and definitions.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2016 - 2017 released Microchip Technology Inc. All rights reserved.

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

#ifndef _DRV_S25FL_LOCAL_H
#define _DRV_S25FL_LOCAL_H

// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "driver/sqi_flash/s25fl/drv_s25fl.h"
#include "driver/sqi_flash/s25fl/src/drv_s25fl_variant_mapping.h"
#include "system/debug/sys_debug.h"

// *****************************************************************************
// *****************************************************************************
// Section: Version Numbers
// *****************************************************************************
// *****************************************************************************
/* Versioning of the driver */

// *****************************************************************************
/* S25FL Driver Version Macros

  Summary:
    S25FL driver version

  Description:
    These constants provide S25FL driver version information. The driver
    version is
    DRV_S25FL_VERSION_MAJOR.DRV_S25FL_VERSION_MINOR.DRV_S25FL_VERSION_PATCH.
    It is represented in DRV_S25FL_VERSION as
    MAJOR *10000 + MINOR * 100 + PATCH, so as to allow comparisons.
    It is also represented in string format in DRV_S25FL_VERSION_STR.
    DRV_S25FL_TYPE provides the type of the release when the release is alpha
    or beta. The interfaces DRV_S25FL_VersionGet() and
    DRV_S25FL_VersionStrGet() provide interfaces to the access the version
    and the version string.

  Remarks:
    Modify the return value of DRV_S25FL_VersionStrGet and the
    DRV_S25FL_VERSION_MAJOR, DRV_S25FL_VERSION_MINOR,
    DRV_S25FL_VERSION_PATCH and DRV_S25FL_VERSION_TYPE
*/

#define _DRV_S25FL_VERSION_MAJOR         0
#define _DRV_S25FL_VERSION_MINOR         2
#define _DRV_S25FL_VERSION_PATCH         0
#define _DRV_S25FL_VERSION_TYPE          "Alpha"
#define _DRV_S25FL_VERSION_STR           "0.2.0 Alpha"

// *****************************************************************************
// *****************************************************************************
// Section: Local Data Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* S25FL Driver Buffer Handle Macros

  Summary:
    S25FL driver Buffer Handle Macros

  Description:
    Buffer handle related utility macros. S25FL driver buffer handle is a 
    combination of buffer token and the buffer object index. The buffertoken
    is a 16 bit number that is incremented for every new write or erase request
    and is used along with the buffer object index to generate a new buffer 
    handle for every request.

  Remarks:
    None
*/

#define DRV_S25FL_BUF_TOKEN_MAX         (0xFFFF)
#define DRV_S25FL_MAKE_HANDLE(token, index) ((token) << 16 | (index))
#define DRV_S25FL_UPDATE_BUF_TOKEN(token) \
{ \
    (token)++; \
    (token) = ((token) == DRV_S25FL_BUF_TOKEN_MAX) ? 0: (token); \
}

// *****************************************************************************
/* S25FL Command set

  Summary:
    Enumeration listing the S25FLVF commands.

  Description:
    This enumeration defines the commands used to interact with the S25FLVF
    series of devices.

  Remarks:
    None
*/

typedef enum
{
    /* Reset enable command. */
    DRV_S25FL_CMD_FLASH_RESET_ENABLE = 0x66,

    /* Command to reset the flash. */
    DRV_S25FL_CMD_FLASH_RESET        = 0x99,

    /* Command to read JEDEC-ID of the flash device. */
    DRV_S25FL_CMD_JEDEC_ID_READ      = 0x9F,

    /* Command to perform High Speed Read */
    DRV_S25FL_CMD_HIGH_SPEED_READ    = 0x0B,

    /* Write enable command. */
    DRV_S25FL_CMD_WRITE_ENABLE       = 0x06,

    /* Write disable command. */
    DRV_S25FL_CMD_WRITE_DISABLE      = 0x04,
            
    /* Write disable command. */
    DRV_S25FL_CMD_WRITE_STATUS_REG   = 0x01,
            
    /* Page Program command. */
    DRV_S25FL_CMD_PAGE_PROGRAM       = 0x02,

    /* Command to read the Flash status register. */
    DRV_S25FL_CMD_READ_STATUS_REG    = 0x05,
    DRV_S25FL_CMD_READ_STATUS_REG_2  = 0x35,
    DRV_S25FL_CMD_READ_STATUS_REG_3  = 0x33,

    /* Command to perform sector erase */
    DRV_S25FL_CMD_SECTOR_ERASE       = 0x20,

    /* Command to perform chip erase */
    DRV_S25FL_CMD_CHIP_ERASE         = 0x60,

} DRV_S25FL_CMD;

// *****************************************************************************
/* S25FL Read/Write/Erase Region Index Numbers

  Summary:
    S25FL Geometry Table Index definitions.

  Description:
    These constants provide S25FL Geometry Table index definitions.

  Remarks:
    None
*/
#define DRV_S25FL_GEOMETRY_TABLE_READ_ENTRY   (0)
#define DRV_S25FL_GEOMETRY_TABLE_WRITE_ENTRY  (1)
#define DRV_S25FL_GEOMETRY_TABLE_ERASE_ENTRY  (2)

// *****************************************************************************
/* S25FL Driver operations.

  Summary:
    Enumeration listing the S25FL driver operations.

  Description:
    This enumeration defines the possible S25FL driver operations.

  Remarks:
    None
*/

typedef enum 
{
    /* Request is read operation. */
    DRV_S25FL_OPERATION_TYPE_READ = 0,

    /* Request is write operation. */
    DRV_S25FL_OPERATION_TYPE_WRITE,

    /* Request is erase operation. */
    DRV_S25FL_OPERATION_TYPE_ERASE,

    /* Request is erase write operation. */
    DRV_S25FL_OPERATION_TYPE_ERASE_WRITE

} DRV_S25FL_OPERATION_TYPE;

// *****************************************************************************
/* S25FL Driver subtask states.

  Summary:
    Enumeration listing the S25FL driver's subtask states.

  Description:
    This enumeration defines the possible S25FL driver's subtask states.

  Remarks:
    None
*/

typedef enum
{
    /* Subtask command state */
    DRV_S25FL_SUBTASK_CMD = 0,

    /* Subtask command status state */
    DRV_S25FL_SUBTASK_CMD_STATUS,

    /* Subtask reset enable command state */
    DRV_S25FL_SUBTASK_RESET_ENABLE_CMD,

    /* Subtask reset enable command status state */
    DRV_S25FL_SUBTASK_RESET_ENABLE_STATUS,

    /* Subtask reset enable delay state */
    DRV_S25FL_SUBTASK_RESET_ENABLE_DELAY,

    /* Subtask reset flash state */
    DRV_S25FL_SUBTASK_RESET_FLASH_STATUS,

    /* Subtask enable quad io command state */
    DRV_S25FL_SUBTASK_ENABLE_QUAD_IO,

    /* Subtask enable quad io command status state */
    DRV_S25FL_SUBTASK_ENABLE_QUAD_IO_STATUS,

    /* Subtask JEDEC-ID read command state */
    DRV_S25FL_SUBTASK_JEDEC_ID_READ_CMD,

    /* Subtask JEDEC-ID read command status state */
    DRV_S25FL_SUBTASK_JEDEC_ID_READ_STATUS,

    /* Subtask Unlock flash command state */
    DRV_S25FL_SUBTASK_UNLOCK_FLASH_CMD,

    /* Subtask Unlock flash command status state */
    DRV_S25FL_SUBTASK_UNLOCK_FLASH_CMD_STATUS,

} DRV_S25FL_SUBTASK_STATE;

typedef enum
{
    DRV_S25FL_CFG_FLASH_READ_STATUS_REG_1 = 0,
    DRV_S25FL_CFG_FLASH_READ_STATUS_REG_2,
    DRV_S25FL_CFG_FLASH_READ_STATUS_REG_3,
    DRV_S25FL_CFG_FLASH_WRITE_ENABLE,
    DRV_S25FL_CFG_FLASH_WRITE_STATUS_REG,
    DRV_S25FL_CFG_FLASH_WRITE_STATUS_REG_STATUS,
    DRV_S25FL_CFG_FLASH_STATUS,
} DRV_S25FL_CFG_FLASH_STATE;
// *****************************************************************************
/* S25FL Driver write states.

  Summary:
    Enumeration listing the S25FL driver's write states.

  Description:
    This enumeration defines the possible S25FL driver's write states.

  Remarks:
    None
*/

typedef enum
{
    /* Write init state */
    DRV_S25FL_WRITE_INIT = 0,
            
    /* Enable write on the flash. */
    DRV_S25FL_WRITE_ENABLE,

    /* Write command state */
    DRV_S25FL_WRITE_CMD,

    /* Write command status state */
    DRV_S25FL_WRITE_CMD_STATUS

} DRV_S25FL_WRITE_STATE;

// *****************************************************************************
/* S25FL Driver erase states.

  Summary:
    Enumeration listing the S25FL driver's erase states.

  Description:
    This enumeration defines the possible S25FL driver's erase states.

  Remarks:
    None
*/
typedef enum
{
    /* Erase init state */
    DRV_S25FL_ERASE_INIT = 0,

    /* Enable write on the flash. */
    DRV_S25FL_ERASE_WRITE_ENABLE,
            
    /* Erase command state */
    DRV_S25FL_ERASE_CMD,

    /* Erase command status state */
    DRV_S25FL_ERASE_CMD_STATUS

} DRV_S25FL_ERASE_STATE;

// *****************************************************************************
/* S25FL Driver erasewrite states.

  Summary:
    Enumeration listing the S25FL driver's erasewrite states.

  Description:
    This enumeration defines the possible S25FL driver's erasewrite states.

  Remarks:
    None
*/

typedef enum
{
    /* Erase write init state. */
    DRV_S25FL_EW_INIT = 0,
    
    /* Erase write read state */
    DRV_S25FL_EW_READ_SECTOR,

    /* Erase write erase state */
    DRV_S25FL_EW_ERASE_SECTOR,

    /* Erase write write state */
    DRV_S25FL_EW_WRITE_SECTOR

} DRV_S25FL_EW_STATE;

typedef enum
{
    /* Check if the SQI driver is ready. If ready open the SQI driver instance.
     * */
    DRV_S25FL_OPEN_SQI = 0,

    /* Reset the SST flash. */
    DRV_S25FL_RESET_FLASH,

    /* Read the SST flash ID */
    DRV_S25FL_READ_FLASH_ID,

    /* Configure the flash parameters. */
    DRV_S25FL_CFG_FLASH,

    /* Process the operations queued at the SST driver. */
    DRV_S25FL_PROCESS_QUEUE,

    /* Perform the required transfer */
    DRV_S25FL_TRANSFER,

    /* Idle state of the driver. */
    DRV_S25FL_IDLE,

    /* Error state. */
    DRV_S25FL_ERROR

} DRV_S25FL_STATE;

/**************************************
 * S25FL Driver Client 
 **************************************/
typedef struct DRV_S25FL_CLIENT_OBJ_STRUCT
{
    /* The hardware instance object associate with the client */
    void * driverObj;

    /* Status of the client object */
    SYS_STATUS sysStatus;

    /* The intent with which the client was opened */
    DRV_IO_INTENT intent;

    /* Flag to indicate in use */
    bool inUse;

    /* Client specific event handler */
    DRV_S25FL_EVENT_HANDLER eventHandler;

    /* Client specific context */
    uintptr_t context;

} DRV_S25FL_CLIENT_OBJECT;

/*******************************************
 * S25FL Driver Buffer Object that services
 * a driver request.
 ******************************************/

typedef struct DRV_S25FL_BUFFER_OBJECT
{
    /* True if object is allocated */
    bool inUse;

    /* Client that owns this buffer */
    DRV_S25FL_CLIENT_OBJECT *hClient;

    /* Present status of this command */
    DRV_S25FL_COMMAND_STATUS status;

    /* Pointer to the next buffer in the queue */
    struct DRV_S25FL_BUFFER_OBJECT * next;

    /* Pointer to the previous buffer in the queue */
    struct DRV_S25FL_BUFFER_OBJECT * previous;

    /* Current command handle of this buffer object */
    DRV_S25FL_COMMAND_HANDLE commandHandle;

    /* Pointer to the source/destination buffer */
    uint8_t *buffer;

    /* Start address of the operation. */
    uint32_t blockStart;

    /* Number of blocks */
    uint32_t nBlocks;

    /* Operation type - read/write/erase/erasewrite */
    DRV_S25FL_OPERATION_TYPE opType;

} DRV_S25FL_BUFFER_OBJECT;

/**************************************
 * S25FL Driver Hardware Instance Object
 **************************************/
typedef struct
{
    /* Object Index */
    SYS_MODULE_INDEX objIndex;

    /* Buffer object for S25FL Operations. */
    DRV_S25FL_BUFFER_OBJECT *queue;

    /* Pointer to the current buffer object */
    DRV_S25FL_BUFFER_OBJECT *currentBufObj;

    /* The status of the driver */
    SYS_STATUS status;

    /* SQI Controller driver handle */
    DRV_HANDLE sqiHandle;

    /* SQI Command handle */
    DRV_SQI_COMMAND_HANDLE cmdHandle;

    /* Flag to indicate in use  */
    bool inUse;

    /* Flag to indicate that the driver is used in exclusive access mode */
    bool isExclusive;

    /* Number of clients connected to the hardware instance */
    uint8_t numClients;

    /* SQI Device Id */
    uint8_t sqiDevice;

    /* Pointer to the Erase Write buffer */
    uint8_t *ewBuffer;

    /* Erase state */
    DRV_S25FL_ERASE_STATE eraseState;
    
    DRV_S25FL_CFG_FLASH_STATE cfgFlashState;

    /* Write state */
    DRV_S25FL_WRITE_STATE writeState;

    /* Erase write state */
    DRV_S25FL_EW_STATE ewState;

    /* Subtask state */
    DRV_S25FL_SUBTASK_STATE subState;

    /* S25FL main task routine's states */
    DRV_S25FL_STATE state;

    /* Tracks the current buffer offset for the write operation. */
    uint32_t bufferOffset;

    /* Tracks the current block address for the write operation. */
    uint32_t blockAddress;

    /* Tracks the current number of blocks of the write operation. */
    uint32_t nBlocks;

    /* */
    uint32_t sectorAddress;

    /* Block offset within the current sector. */
    uint32_t blockOffsetInSector;

    /* Number of blocks to write. */
    uint32_t nBlocksToWrite;

    /* This instance's flash start address */
    uint32_t blockStartAddress;

    /* */
    uint8_t *writePtr;

    /* Pointer to the transfer frames. */
    DRV_SQI_TransferFrame *xferFrame;

    /* Pointer to the flash id register */
    uint8_t *flashId;

    /* Pointer to the flash status register. */
    uint8_t *statusReg;
    
    /* Pointer to the timer handle. */
    SYS_TMR_HANDLE tmrHandle;

    /* S25FL driver geometry object */
    SYS_FS_MEDIA_GEOMETRY mediaGeometryObj;

    /* S25FL driver media geometry table. */
    SYS_FS_MEDIA_REGION_GEOMETRY mediaGeometryTable[3];

    /* Driver object Mutex */
    OSAL_MUTEX_DECLARE(mutex);

} DRV_S25FL_OBJECT;

typedef DRV_SQI_COMMAND_STATUS (*DRV_S25FL_TransferOperation)(DRV_S25FL_OBJECT *dObj, uint8_t *data, uint32_t blockStart, uint32_t nBlocks);

#endif //#ifndef _DRV_S25FL_LOCAL_H

/*******************************************************************************
 End of File
*/

