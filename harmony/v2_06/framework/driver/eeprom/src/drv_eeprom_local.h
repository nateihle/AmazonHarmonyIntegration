/*******************************************************************************
  EEPROM Driver Local Data Structures

  Company:
    Microchip Technology Inc.

  File Name:
    drv_eeprom_local.h

  Summary:
    EEPROM driver local declarations and definitions

  Description:
    This file contains the timer driver's local declarations and definitions.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _DRV_EEPROM_LOCAL_H
#define _DRV_EEPROM_LOCAL_H

// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "driver/eeprom/drv_eeprom.h"
#include "driver/eeprom/src/drv_eeprom_variant_mapping.h"

// *****************************************************************************
// *****************************************************************************
// Section: Version Numbers
// *****************************************************************************
// *****************************************************************************
/* Versioning of the driver */

// *****************************************************************************
/* EEPROM Driver Version Macros

  Summary:
    EEPROM driver version

  Description:
    These constants provide EEPROM driver version information. The driver
    version is
    DRV_EEPROM_VERSION_MAJOR.DRV_EEPROM_VERSION_MINOR.DRV_EEPROM_VERSION_PATCH.
    It is represented in DRV_EEPROM_VERSION as
    MAJOR *10000 + MINOR * 100 + PATCH, so as to allow comparisons.
    It is also represented in string format in DRV_EEPROM_VERSION_STR.
    DRV_EEPROM_TYPE provides the type of the release when the release is alpha
    or beta. The interfaces DRV_EEPROM_VersionGet() and
    DRV_EEPROM_VersionStrGet() provide interfaces to the access the version
    and the version string.

  Remarks:
    Modify the return value of DRV_EEPROM_VersionStrGet and the
    DRV_EEPROM_VERSION_MAJOR, DRV_EEPROM_VERSION_MINOR,
    DRV_EEPROM_VERSION_PATCH and DRV_EEPROM_VERSION_TYPE
*/

#define _DRV_EEPROM_VERSION_MAJOR         0
#define _DRV_EEPROM_VERSION_MINOR         2
#define _DRV_EEPROM_VERSION_PATCH         0
#define _DRV_EEPROM_VERSION_TYPE          "Alpha"
#define _DRV_EEPROM_VERSION_STR           "0.2.0 Alpha"

// *****************************************************************************
/* EEPROM Flash Read/Write/Erase Region Index Numbers

  Summary:
    EEPROM Geometry Table Index definitions.

  Description:
    These constants provide EEPROM Geometry Table index definitions.

  Remarks:
    None
*/
#define GEOMETRY_TABLE_READ_ENTRY   (0)
#define GEOMETRY_TABLE_WRITE_ENTRY  (1)
#define GEOMETRY_TABLE_ERASE_ENTRY  (2)

/* EEPROM Unlock keys. */
#define DRV_EEPROM_UNLOCK_KEY_1 0xEDB7
#define DRV_EEPROM_UNLOCK_KEY_2 0x1248

/* Macro to create a word */
#define DRV_EEPROM_MAKE_WORD(a, b, c, d) ((a) | ((b) << 8) | ((c) << 16) | ((d) << 24))

// *****************************************************************************
/* EEPROM Driver Buffer Handle Macros

  Summary:
    EEPROM driver Buffer Handle Macros

  Description:
    Buffer handle related utility macros. EEPROM driver buffer handle is a 
    combination of buffer token and the buffer object index. The buffertoken
    is a 16 bit number that is incremented for every new write or erase request
    and is used along with the buffer object index to generate a new buffer 
    handle for every request.

  Remarks:
    None
*/

#define DRV_EEPROM_BUF_TOKEN_MAX         (0xFFFF)
#define DRV_EEPROM_MAKE_HANDLE(token, index) ((token) << 16 | (index))
#define DRV_EEPROM_UPDATE_BUF_TOKEN(token) \
{ \
    (token)++; \
    (token) = ((token) == DRV_EEPROM_BUF_TOKEN_MAX) ? 0: (token); \
}

/* Enumeration listing the type of buffer operations supported by the EEPROM
 * driver. */
typedef enum
{
    /* Read operation. */
    DRV_EEPROM_BUFFER_OPERATION_READ = GEOMETRY_TABLE_READ_ENTRY,

    /* Write operation. */
    DRV_EEPROM_BUFFER_OPERATION_WRITE = GEOMETRY_TABLE_WRITE_ENTRY,

    /* Page Erase operation. */
    DRV_EEPROM_BUFFER_OPERATION_ERASE = GEOMETRY_TABLE_ERASE_ENTRY,

    /* Bulk Erase operation. */
    DRV_EEPROM_BUFFER_OPERATION_BULK_ERASE

} DRV_EEPROM_BUFFER_OPERATION;

/* EEPROM Task routine states. */
typedef enum
{
    /* Check if the EEPROM ready bit is set. */
    DRV_EEPROM_TASK_EEPROM_READY = 0,

    /* Perform the EEPROM initialization(configure the calibration data). */
    DRV_EEPROM_TASK_INIT,

    /* Process queued read/write operation.s */
    DRV_EEPROM_TASK_PROCESS_QUEUE,

    /* EEPROM Idle state. */
    DRV_EEPROM_TASK_IDLE

} DRV_EEPROM_TASK_STATES;

typedef enum
{
    /* Init state. */
    DRV_EEPROM_PROCESS_REQ_START = 0,

    /* Process the queued read operation request. */
    DRV_EEPROM_PROCESS_READ,

    /* Check the status of the read operation. */
    DRV_EEPROM_PROCESS_READ_STATUS,

    /* Process the queued write operation request. */
    DRV_EEPROM_PROCESS_WRITE,

    /* Check the status of the write operation. */
    DRV_EEPROM_PROCESS_WRITE_STATUS,

    /* Process the queued erase operation request. */
    DRV_EEPROM_PROCESS_ERASE,

    /* Check the status of the erase operation. */
    DRV_EEPROM_PROCESS_ERASE_STATUS,

    /* Process the queued bulk erase operation request. */
    DRV_EEPROM_PROCESS_BULK_ERASE,

    /* Check the status of the bulk erase operation. */
    DRV_EEPROM_PROCESS_BULK_ERASE_STATUS,

    /* The transfer request processing is complete. Check if there are any
     * errors and handle it. */
    DRV_EEPROM_PROCESS_XFER_COMPLETION,

    /* State to handle the Brown out reset condition. */
    DRV_EEPROM_PROCESS_BOR_RESET,

} DRV_EEPROM_REQUEST_TASK_STATES;

/* EEPROM Init subtask states. */
typedef enum
{
    /* Start the EEPROM initialization. */
    DRV_EEPROM_INIT_START = 0,

    /* Configure the EEPROM calibration data. */
    DRV_EEPROM_INIT_CALIBRATION_DATA,

    /* State to check the calibration status. */
    DRV_EEPROM_INIT_CALIBRATION_STATUS_CHECK,

    /* State to configure the EEPROM access time. */
    DRV_EEPROM_INIT_DONE

} DRV_EEPROM_INIT_STATES;

// *****************************************************************************
// *****************************************************************************
// Section: Local Data Type Definitions
// *****************************************************************************
// *****************************************************************************

/*******************************************
 * EEPROM Driver Buffer Object that services
 * a driver request.
 ******************************************/

typedef struct _DRV_EEPROM_BUFFER_OBJ_STRUCT
{
    /* True if object is allocated */
    bool inUse;

    /* Type of EEPROM driver operation */
    DRV_EEPROM_BUFFER_OPERATION operation;

    /* Client source or destination pointer for read and write operations. */
    uint8_t *buffer;

    /* Client source or destination pointer */
    uint32_t address;

    /* Number of blocks to read/write */
    uint32_t nBlocks;

    /* Client that owns this buffer */
    DRV_HANDLE hClient;

    /* Present status of this command */
    DRV_EEPROM_COMMAND_STATUS status;

    /* Current command handle of this buffer object */
    DRV_EEPROM_COMMAND_HANDLE commandHandle;

    /* Pointer to the next buffer in the queue */
    struct _DRV_EEPROM_BUFFER_OBJ_STRUCT *next;

    /* Pointer to the previous buffer in the queue */
    struct _DRV_EEPROM_BUFFER_OBJ_STRUCT *previous;

} DRV_EEPROM_BUFFER_OBJECT;

/**************************************
 * EEPROM Driver Instance Object
 **************************************/
typedef struct _DRV_EEPROM_OBJ_STRUCT
{
    /* Flag to indicate in use */
    bool inUse;

    /* Flag to indicate that SAMPLE is used in exclusive access mode */
    bool isExclusive;

    /* Number of clients connected to the hardware instance */
    uint8_t numClients;

    /* The module index associated with the object*/
    NVM_MODULE_ID moduleId;

    /* Object Index */
    SYS_MODULE_INDEX drvIndex;

    /* Variable to track the error with this instance of the EEPROM. */
    EEPROM_ERROR error;

    /* Used for calibrating the EEPROM. */
    uint32_t address;

    /* The buffer queue for the read/write/erase operations */
    DRV_EEPROM_BUFFER_OBJECT *queue;

    /* Block start address */
    uint32_t blockStartAddress;

    /* Variable to track the current interrupt status. */
    uint32_t intStatus;

    /* The status of the driver */
    SYS_STATUS status;

    /* EEPROM Initialize sub task states. */
    DRV_EEPROM_INIT_STATES initState;
    
    /* EEPROM task states. */
    DRV_EEPROM_TASK_STATES taskState;

    /* EEPROM request task states. */
    DRV_EEPROM_REQUEST_TASK_STATES reqState;

    /* Driver object Mutex */
    OSAL_MUTEX_DECLARE(mutex);

    /* Geometry object */
    SYS_FS_MEDIA_GEOMETRY *mediaGeometryObj;

} DRV_EEPROM_OBJECT;

/**************************************
 * EEPROM Driver Client 
 **************************************/
typedef struct _DRV_EEPROM_CLIENT_OBJ_STRUCT
{
    /* Flag to indicate that client object is in use */
    bool inUse;

    /* The hardware instance object associate with the client */
    void *driverObj;

    /* Status of the client object */
    SYS_STATUS status;

    /* The intent with which the client was opened */
    DRV_IO_INTENT intent;

    /* Client specific event handler */
    DRV_EEPROM_EVENT_HANDLER  eventHandler;

    /* Client specific context */
    uintptr_t context;

} DRV_EEPROM_CLIENT_OBJECT;

#endif //#ifndef _DRV_EEPROM_LOCAL_H

/*******************************************************************************
 End of File
*/

