/*******************************************************************************
  SST26 Driver Local Data Structures

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sst26_local.h

  Summary:
    SST26 driver local declarations and definitions

  Description:
    This file contains the SST26 driver's local declarations and definitions.
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

#ifndef _DRV_SST26_LOCAL_H
#define _DRV_SST26_LOCAL_H

// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "driver/sqi_flash/sst26/drv_sst26.h"
#include "driver/sqi_flash/sst26/src/drv_sst26_variant_mapping.h"
#include "system/debug/sys_debug.h"

// *****************************************************************************
// *****************************************************************************
// Section: Version Numbers
// *****************************************************************************
// *****************************************************************************
/* Versioning of the driver */

// *****************************************************************************
/* SST26 Driver Version Macros

  Summary:
    SST26 driver version

  Description:
    These constants provide SST26 driver version information. The driver
    version is
    DRV_SST26_VERSION_MAJOR.DRV_SST26_VERSION_MINOR.DRV_SST26_VERSION_PATCH.
    It is represented in DRV_SST26_VERSION as
    MAJOR *10000 + MINOR * 100 + PATCH, so as to allow comparisons.
    It is also represented in string format in DRV_SST26_VERSION_STR.
    DRV_SST26_TYPE provides the type of the release when the release is alpha
    or beta. The interfaces DRV_SST26_VersionGet() and
    DRV_SST26_VersionStrGet() provide interfaces to the access the version
    and the version string.

  Remarks:
    Modify the return value of DRV_SST26_VersionStrGet and the
    DRV_SST26_VERSION_MAJOR, DRV_SST26_VERSION_MINOR,
    DRV_SST26_VERSION_PATCH and DRV_SST26_VERSION_TYPE
*/

#define _DRV_SST26_VERSION_MAJOR         0
#define _DRV_SST26_VERSION_MINOR         2
#define _DRV_SST26_VERSION_PATCH         0
#define _DRV_SST26_VERSION_TYPE          "Alpha"
#define _DRV_SST26_VERSION_STR           "0.2.0 Alpha"

// *****************************************************************************
// *****************************************************************************
// Section: Local Data Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* SST26 Driver Buffer Handle Macros

  Summary:
    SST26 driver Buffer Handle Macros

  Description:
    Buffer handle related utility macros. SST26 driver buffer handle is a 
    combination of buffer token and the buffer object index. The buffertoken
    is a 16 bit number that is incremented for every new write or erase request
    and is used along with the buffer object index to generate a new buffer 
    handle for every request.

  Remarks:
    None
*/

#define DRV_SST26_BUF_TOKEN_MAX         (0xFFFF)
#define DRV_SST26_MAKE_HANDLE(token, index) ((token) << 16 | (index))
#define DRV_SST26_UPDATE_BUF_TOKEN(token) \
{ \
    (token)++; \
    (token) = ((token) == DRV_SST26_BUF_TOKEN_MAX) ? 0: (token); \
}

// *****************************************************************************
/* SST26 Command set

  Summary:
    Enumeration listing the SST26VF commands.

  Description:
    This enumeration defines the commands used to interact with the SST26VF
    series of devices.

  Remarks:
    None
*/

typedef enum
{
    /* Reset enable command. */
    DRV_SST26_CMD_FLASH_RESET_ENABLE = 0x66,

    /* Command to reset the flash. */
    DRV_SST26_CMD_FLASH_RESET        = 0x99,

    /* Command to Enable QUAD IO */
    DRV_SST26_CMD_ENABLE_QUAD_IO     = 0x38,

    /* Command to Reset QUAD IO */
    DRV_SST26_CMD_RESET_QUAD_IO      = 0xFF,

    /* Command to read JEDEC-ID of the flash device. */
    DRV_SST26_CMD_JEDEC_ID_READ      = 0x9F,

    /* QUAD Command to read JEDEC-ID of the flash device. */
    DRV_SST26_CMD_QUAD_JEDEC_ID_READ = 0xAF,
    
    /* Command to perfrom High Speed Read */
    DRV_SST26_CMD_HIGH_SPEED_READ    = 0x0B,

    /* Write enable command. */
    DRV_SST26_CMD_WRITE_ENABLE       = 0x06,

    /* Page Program command. */
    DRV_SST26_CMD_PAGE_PROGRAM       = 0x02,

    /* Command to read the Flash status register. */
    DRV_SST26_CMD_READ_STATUS_REG    = 0x05,

    /* Command to perfrom sector erase */
    DRV_SST26_CMD_SECTOR_ERASE       = 0x20,

    /* Command to unlock the flash device. */
    DRV_SST26_CMD_UNPROTECT_GLOBAL   = 0x98

} DRV_SST26_CMD;

// *****************************************************************************
/* SST26 Read/Write/Erase Region Index Numbers

  Summary:
    SST26 Geometry Table Index definitions.

  Description:
    These constants provide SST26 Geometry Table index definitions.

  Remarks:
    None
*/
#define DRV_SST26_GEOMETRY_TABLE_READ_ENTRY   (0)
#define DRV_SST26_GEOMETRY_TABLE_WRITE_ENTRY  (1)
#define DRV_SST26_GEOMETRY_TABLE_ERASE_ENTRY  (2)

// *****************************************************************************
/* SST26 Driver operations.

  Summary:
    Enumeration listing the SST26 driver operations.

  Description:
    This enumeration defines the possible SST26 driver operations.

  Remarks:
    None
*/

typedef enum 
{
    /* Request is read operation. */
    DRV_SST26_OPERATION_TYPE_READ = 0,

    /* Request is write operation. */
    DRV_SST26_OPERATION_TYPE_WRITE,

    /* Request is erase operation. */
    DRV_SST26_OPERATION_TYPE_ERASE,

    /* Request is erase write operation. */
    DRV_SST26_OPERATION_TYPE_ERASE_WRITE

} DRV_SST26_OPERATION_TYPE;

// *****************************************************************************
/* SST26 Driver subtask states.

  Summary:
    Enumeration listing the SST26 driver's subtask states.

  Description:
    This enumeration defines the possible SST26 driver's subtask states.

  Remarks:
    None
*/

typedef enum
{
    /* Subtask command state */
    DRV_SST26_SUBTASK_CMD = 0,

    /* Subtask command status state */
    DRV_SST26_SUBTASK_CMD_STATUS,

    /* Subtask reset enable command state */
    DRV_SST26_SUBTASK_RESET_ENABLE_CMD,

    /* Subtask reset enable command status state */
    DRV_SST26_SUBTASK_RESET_ENABLE_STATUS,

    /* Subtask reset enable delay state */
    DRV_SST26_SUBTASK_RESET_ENABLE_DELAY,

    /* Subtask reset flash state */
    DRV_SST26_SUBTASK_RESET_FLASH_STATUS,

    /* Subtask enable quad io command state */
    DRV_SST26_SUBTASK_ENABLE_QUAD_IO,

    /* Subtask enable quad io command status state */
    DRV_SST26_SUBTASK_ENABLE_QUAD_IO_STATUS,

    /* Subtask JEDEC-ID read command state */
    DRV_SST26_SUBTASK_JEDEC_ID_READ_CMD,

    /* Subtask JEDEC-ID read command status state */
    DRV_SST26_SUBTASK_JEDEC_ID_READ_STATUS,

    /* Subtask Unlock flash command write enable state */
    DRV_SST26_SUBTASK_UNLOCK_FLASH_WRITE_ENABLE,
            
    /* Subtask Unlock flash command state */
    DRV_SST26_SUBTASK_UNLOCK_FLASH_CMD,

    /* Subtask Unlock flash command status state */
    DRV_SST26_SUBTASK_UNLOCK_FLASH_CMD_STATUS,
    
} DRV_SST26_SUBTASK_STATE;

typedef enum
{
    DRV_SST26_WRITE_ENABLE_CMD = 0,
            
    DRV_SST26_WRITE_ENABLE_CMD_STATUS
} DRV_SST26_WRITE_ENABLE_STATE;

// *****************************************************************************
/* SST26 Driver write states.

  Summary:
    Enumeration listing the SST26 driver's write states.

  Description:
    This enumeration defines the possible SST26 driver's write states.

  Remarks:
    None
*/

typedef enum
{
    /* Write init state */
    DRV_SST26_WRITE_INIT = 0,

    /* Write command state */
    DRV_SST26_WRITE_CMD,

    /* Write Enable command state */
    DRV_SST26_WRITE_WRITE_ENABLE,

    /* Write command status state */
    DRV_SST26_WRITE_CMD_STATUS

} DRV_SST26_WRITE_STATE;

// *****************************************************************************
/* SST26 Driver erase states.

  Summary:
    Enumeration listing the SST26 driver's erase states.

  Description:
    This enumeration defines the possible SST26 driver's erase states.

  Remarks:
    None
*/
typedef enum
{
    /* Erase init state */
    DRV_SST26_ERASE_INIT = 0,

    /* Write Enable state. */
    DRV_SST26_ERASE_WRITE_ENABLE,
    
    /* Erase command state */
    DRV_SST26_ERASE_CMD,

    /* Erase command status state */
    DRV_SST26_ERASE_CMD_STATUS

} DRV_SST26_ERASE_STATE;

// *****************************************************************************
/* SST26 Driver erasewrite states.

  Summary:
    Enumeration listing the SST26 driver's erasewrite states.

  Description:
    This enumeration defines the possible SST26 driver's erasewrite states.

  Remarks:
    None
*/

typedef enum
{
    /* Erase write init state. */
    DRV_SST26_EW_INIT = 0,
    
    /* Erase write read state */
    DRV_SST26_EW_READ_SECTOR,

    /* Erase write erase state */
    DRV_SST26_EW_ERASE_SECTOR,

    /* Erase write write state */
    DRV_SST26_EW_WRITE_SECTOR

} DRV_SST26_EW_STATE;

typedef enum
{
    /* Check if the SQI driver is ready. If ready open the SQI driver instance.
     * */
    DRV_SST26_OPEN_SQI = 0,

    /* Reset the SST flash. */
    DRV_SST26_RESET_FLASH,

    /* Read the SST flash ID */
    DRV_SST26_READ_FLASH_ID,

    /* Enable the QUAD IO on the flash. */
    DRV_SST26_ENABLE_QUAD_IO,

    /* Perform a global unlock command. */
    DRV_SST26_UNLOCK_FLASH,

    /* Process the operations queued at the SST driver. */
    DRV_SST26_PROCESS_QUEUE,

    /* Perform the required transfer */
    DRV_SST26_TRANSFER,

    /* Idle state of the driver. */
    DRV_SST26_IDLE,

    /* Error state. */
    DRV_SST26_ERROR

} DRV_SST26_STATE;

/**************************************
 * SST26 Driver Client 
 **************************************/
typedef struct DRV_SST26_CLIENT_OBJ_STRUCT
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
    DRV_SST26_EVENT_HANDLER eventHandler;

    /* Client specific context */
    uintptr_t context;

} DRV_SST26_CLIENT_OBJECT;

/*******************************************
 * SST26 Driver Buffer Object that services
 * a driver request.
 ******************************************/

typedef struct DRV_SST26_BUFFER_OBJECT
{
    /* True if object is allocated */
    bool inUse;

    /* Client that owns this buffer */
    DRV_SST26_CLIENT_OBJECT *hClient;

    /* Present status of this command */
    DRV_SST26_COMMAND_STATUS status;

    /* Pointer to the next buffer in the queue */
    struct DRV_SST26_BUFFER_OBJECT * next;

    /* Pointer to the previous buffer in the queue */
    struct DRV_SST26_BUFFER_OBJECT * previous;

    /* Current command handle of this buffer object */
    DRV_SST26_COMMAND_HANDLE commandHandle;

    /* Pointer to the source/destination buffer */
    uint8_t *buffer;

    /* Start address of the operation. */
    uint32_t blockStart;

    /* Number of blocks */
    uint32_t nBlocks;

    /* Operation type - read/write/erase/erasewrite */
    DRV_SST26_OPERATION_TYPE opType;

} DRV_SST26_BUFFER_OBJECT;

/**************************************
 * SST26 Driver Hardware Instance Object
 **************************************/
typedef struct
{
    /* Object Index */
    SYS_MODULE_INDEX objIndex;

    /* Buffer object for SST26 Operations. */
    DRV_SST26_BUFFER_OBJECT *queue;

    /* Pointer to the current buffer object */
    DRV_SST26_BUFFER_OBJECT *currentBufObj;

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
    DRV_SST26_ERASE_STATE eraseState;

    /* Write state */
    DRV_SST26_WRITE_STATE writeState;

    /* Write Enable state */
    DRV_SST26_WRITE_ENABLE_STATE writeEnableState;
    
    /* Erase write state */
    DRV_SST26_EW_STATE ewState;

    /* Subtask state */
    DRV_SST26_SUBTASK_STATE subState;

    /* SST26 main task routine's states */
    DRV_SST26_STATE state;

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

    /* Pointer to the transfer elements. */
    DRV_SQI_TransferElement *transferElement;

    /* Pointer to the transfer frame. */
    DRV_SQI_TransferFrame *xferFrame;
    /* Pointer to the flash id register */
    uint8_t *flashId;

    /* Pointerr to the flash status register. */
    uint8_t *statusReg;

    /* Pointer to the command parameter array */
    uint8_t *cmdParams;

    /* Pointer to the timer handle. */
    SYS_TMR_HANDLE tmrHandle;

    /* SST26 driver geometry object */
    SYS_FS_MEDIA_GEOMETRY mediaGeometryObj;

    /* SST26 driver media geomtery table. */
    SYS_FS_MEDIA_REGION_GEOMETRY mediaGeometryTable[3];

    /* Driver object Mutex */
    OSAL_MUTEX_DECLARE(mutex);

} DRV_SST26_OBJECT;

typedef DRV_SQI_COMMAND_STATUS (*DRV_SST26_TransferOperation)(DRV_SST26_OBJECT *dObj, uint8_t *data, uint32_t blockStart, uint32_t nBlocks);

#endif //#ifndef _DRV_SST26_LOCAL_H

/*******************************************************************************
 End of File
*/

