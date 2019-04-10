/*******************************************************************************
  SRAM Driver Local Data Structures

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sram_local.h

  Summary:
    SRAM driver local declarations and definitions

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

#ifndef _DRV_SRAM_LOCAL_H
#define _DRV_SRAM_LOCAL_H

// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "driver/sram/drv_sram.h"
#include "driver/sram/src/drv_sram_variant_mapping.h"

// *****************************************************************************
// *****************************************************************************
// Section: Version Numbers
// *****************************************************************************
// *****************************************************************************
/* Versioning of the driver */

// *****************************************************************************
/* SRAM Driver Version Macros

  Summary:
    SRAM driver version

  Description:
    These constants provide SRAM driver version information. The driver
    version is
    DRV_SRAM_VERSION_MAJOR.DRV_SRAM_VERSION_MINOR.DRV_SRAM_VERSION_PATCH.
    It is represented in DRV_SRAM_VERSION as
    MAJOR *10000 + MINOR * 100 + PATCH, so as to allow comparisons.
    It is also represented in string format in DRV_SRAM_VERSION_STR.
    DRV_SRAM_TYPE provides the type of the release when the release is alpha
    or beta. The interfaces DRV_SRAM_VersionGet() and
    DRV_SRAM_VersionStrGet() provide interfaces to the access the version
    and the version string.

  Remarks:
    Modify the return value of DRV_SRAM_VersionStrGet and the
    DRV_SRAM_VERSION_MAJOR, DRV_SRAM_VERSION_MINOR,
    DRV_SRAM_VERSION_PATCH and DRV_SRAM_VERSION_TYPE
*/

#define _DRV_SRAM_VERSION_MAJOR         0
#define _DRV_SRAM_VERSION_MINOR         2
#define _DRV_SRAM_VERSION_PATCH         0
#define _DRV_SRAM_VERSION_TYPE          "Alpha"
#define _DRV_SRAM_VERSION_STR           "0.2.0 Alpha"

// *****************************************************************************
/* SRAM Flash Read/Write/Erase Region Index Numbers

  Summary:
    SRAM Geometry Table Index definitions.

  Description:
    These constants provide SRAM Geometry Table index definitions.

  Remarks:
    None
*/
#define GEOMETRY_TABLE_READ_ENTRY   (0)
#define GEOMETRY_TABLE_WRITE_ENTRY  (1)
#define GEOMETRY_TABLE_ERASE_ENTRY  (2)

/*****************************************************************************
 * If the SRAM needs to be controlled by media manager, then declare the
 * following as 1. Otherwise, declare as 0.
 *
 *****************************************************************************/

// *****************************************************************************
/* SRAM Driver Buffer Handle Macros

  Summary:
    SRAM driver Buffer Handle Macros

  Description:
    Buffer handle related utility macros. SRAM driver buffer handle is a 
    combination of buffer token and the buffer object index. The buffertoken
    is a 16 bit number that is incremented for every new write or erase request
    and is used along with the buffer object index to generate a new buffer 
    handle for every request.

  Remarks:
    None
*/

#define _DRV_SRAM_BUF_TOKEN_MAX         (0xFFFF)
#define _DRV_SRAM_MAKE_HANDLE(token, index) ((token) << 16 | (index))
#define _DRV_SRAM_UPDATE_BUF_TOKEN(token) \
{ \
    (token)++; \
    (token) = ((token) == _DRV_SRAM_BUF_TOKEN_MAX) ? 0: (token); \
}


typedef enum {
    DRV_SRAM_BUFFER_OPERATION_READ = 0,
    DRV_SRAM_BUFFER_OPERATION_WRITE
} DRV_SRAM_BUFFER_OPERATION;

// *****************************************************************************
// *****************************************************************************
// Section: Local Data Type Definitions
// *****************************************************************************
// *****************************************************************************

/*******************************************
 * SRAM Driver Buffer Object that services
 * a driver request.
 ******************************************/

typedef struct _DRV_SRAM_BUFFER_OBJ_STRUCT
{
    /* True if object is allocated */
    bool inUse;

    /* Type of SRAM driver operation */
    DRV_SRAM_BUFFER_OPERATION operation;

    /* Client source or destination pointer */
    uint8_t *buffer;

    /* Client source or destination pointer */
    uint32_t address;

    /* Number of blocks to read/write */
    uint32_t nBlocks;

    /* Client that owns this buffer */
    DRV_HANDLE hClient;

    /* Present status of this command */
    DRV_SRAM_COMMAND_STATUS status;

    /* Current command handle of this buffer object */
    DRV_SRAM_COMMAND_HANDLE commandHandle;

} DRV_SRAM_BUFFER_OBJECT;

/**************************************
 * SRAM Driver Instance Object
 **************************************/
typedef struct _DRV_SRAM_OBJ_STRUCT
{
    /* Flag to indicate in use */
    bool inUse;

    /* Flag to indicate that SAMPLE is used in exclusive access mode */
    bool isExclusive;

    /* Number of clients connected to the hardware instance */
    uint8_t numClients;

    /* The status of the driver */
    SYS_STATUS status;

    /* Block start address */
    uintptr_t blockStartAddress;

    /* Geometry Object */
    SYS_FS_MEDIA_GEOMETRY *sramMediaGeometry;

} DRV_SRAM_OBJECT;

/**************************************
 * SRAM Driver Client 
 **************************************/
typedef struct _DRV_SRAM_CLIENT_OBJ_STRUCT
{
    /* The hardware instance object associate with the client */
    void *driverObj;

    /* The intent with which the client was opened */
    DRV_IO_INTENT intent;

    /* Client specific event handler */
    DRV_SRAM_EVENT_HANDLER eventHandler;

    /* Client specific context */
    uintptr_t context;

    /* Flag to indicate that client object is in use */
    bool inUse;

} DRV_SRAM_CLIENT_OBJECT;

#endif //#ifndef _DRV_SRAM_LOCAL_H

/*******************************************************************************
 End of File
*/

