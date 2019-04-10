/*******************************************************************************
  IPF SPI Flash Driver Local Data Structures

  Company:
    Microchip Technology Inc.

  File Name:
    drv_ipf_local.h

  Summary:
    IPF SPI Flash Driver Local Data Structures

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

#ifndef _DRV_IPF_LOCAL_H
#define _DRV_IPF_LOCAL_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "driver/spi_flash/pic32wk_ipf/drv_ipf.h"
#include "driver/spi_flash/pic32wk_ipf/src/drv_ipf_variant_mapping.h"
#include "osal/osal.h"
#include "system/debug/sys_debug.h"



// *****************************************************************************
// *****************************************************************************
// Section: Constant Definitions
// *****************************************************************************
// *****************************************************************************

/* op-code definitions for different operations */
#define IPF_WREN_OP_CODE    0x06
#define IPF_WRSR_OP_CODE    0x01
#define IPF_ERASE_OP_CODE    0x20
#define IPF_WRITE_BYTE_OP_CODE    0x02
#define IPF_NORMAL_READ_OP_CODE    0x03
#define IPF_HIGH_SPEED_READ_OP_CODE    0x0B
#define IPF_RDSR_OP_CODE    0x05
#define IPF_ULBPR_OP_CODE 0x98
#define IPF_READ_BLOCK_PROT_STATUS 0x72
#define IPF_WRITE_BLOCK_PROT_REG 0x42
							
#define DRV_IPF_BLOCK_PROT_BYTES 0x06							

// *****************************************************************************
// *****************************************************************************
// Section: Data Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* DRV_IPF_BUFFER_PROCESS_STATE

  Summary
    Lists the different states that internal buffer processing task routine
    can have.

  Description
    This enumeration lists the different states that internal buffer processing
    task routine can have.

  Remarks:
    None.
*/

typedef enum{

    /* Beginning of Buffer Process */
    DRV_IPF_BUFFER_PROCESS_INIT,

    /* Send write enable command */
    DRV_IPF_SEND_WREN_CMD,

    /* Check if write has been enabled */
    DRV_IPF_WREN_EXECUTION_STATUS_CHECK,

    /* Send Write command along with write address and data pointer */
    DRV_IPF_SEND_WRITE_CMD_ADDRESS_AND_DATA,

    /* Send erase command and address */
    DRV_IPF_SEND_ERASE_CMD_AND_ADDRESS,

    /* Send read command and address */
    DRV_IPF_SEND_READ_CMD_AND_ADDRESS,

    /* Start the read operation */
    DRV_IPF_START_READING,

    /* Wait for Write/Erase buffer completion */
    DRV_IPF_WAIT_FOR_WRITE_OR_ERASE_BUFFER_COMPLETE,

    /* Send command to read the status register */
    DRV_IPF_SEND_COMMAND_FOR_BUSY_STATUS,

    /* Read the status of Busy Bit */
    DRV_IPF_READ_BUSY_STATUS,

    /* Wait until BUSY bit is clear */
    DRV_IPF_WAIT_FOR_BUSY_CLEAR,
            
    /* Send Memory Protection word to Flash */        
    DRV_IPF_SEND_MEM_PROT_WORD,

    /* Read operation is completed */
    DRV_IPF_READ_COMPLETED,
       
    /* Send command for BPR status read */
    DRV_IPF_SEND_READ_HW_BLOCK_PROT_STATUS,
    
    /* Read BPR status */
    DRV_IPF_HW_PROT_STATUS_READING,
    
    /* check if Read BPR status complete */
    DRV_IPF_HW_PROT_READ_COMPLETED

}DRV_IPF_BUFFER_PROCESS_STATE;

// *****************************************************************************
/* IPF Driver task states

  Summary
    Lists the different states that IPF task routine can have.

  Description
    This enumeration lists the different states that IPF task routine
    can have.

  Remarks:
    None.
*/

typedef enum
{
    /* Open SPI driver */
    DRV_IPF_TASK_OPEN_SPI_DRIVER,

    /* This state is to send the WREN command to be able to modify
       SST Status register */
    DRV_IPF_SEND_WREN_CMD_WRSR,

    /* Check if write has been enabled */
    DRV_IPF_WREN_EXECUTION_STATUS_CHECK_WRSR,

    /* Send write status register command and status register value */
    DRV_IPF_SEND_WRSR_CMD_AND_VALUE,

    /* Check if Status register is written */
    DRV_IPF_WRSR_EXECUTION_STATUS_CHECK,

    /* Process Queue */
    DRV_IPF_TASK_PROCESS_QUEUE,
    
    /* Send Write enable before sending the unlock global block protection command*/
    DRV_IPF_SEND_WREN_CMD_ULBPR,
            
    /* Check if write has been enabled */        
    DRV_IPF_WREN_EXECUTION_STATUS_CHECK_ULBPR,
    
    /* Send Unlock global block protection command */        
    DRV_IPF_SEND_ULBPR_CMD,
    
    /* Check if the execution of ULBPR instruction is complete */        
    DRV_IPF_ULBPR_EXECUTION_STATUS_CHECK,
	
	/* Send the Block Protection Read command */ 
	DRV_IPF_READ_BPR_STATUS_CMD_SEND,
	
	/* Read the BPR status to local buffer */
	DRV_IPF_READ_BPR_STATUS,
	
	/* Check if the BPR read status is complete */
	DRV_IPF_READ_BPR_COMPLETE_CHECK
	
} DRV_IPF_DATA_OBJECT_STATE;


// *****************************************************************************
/* IPF SPI Flash Driver Buffer Object

  Summary:
    Object used to keep track of a client's buffer.

  Description:
    This object is used to keep track of a client's buffer in the driver's 
    queue.

  Remarks:
    None.
*/

typedef struct _DRV_IPF_BUFFER_OBJ
{
    /* This flag tracks whether this object is in use */
    bool inUse;

    /* Pointer to the application read or write buffer */
    uint8_t * buffer;

    /* address of flash from where to read or to write or erase from */
    uint32_t address;
    
    /* Tracks how much data has been transferred */
    uint32_t nCurrentBlocks;

    /* Number of blocks to be transferred */
    uint32_t size;

    /* block operation which has been requested */
    DRV_IPF_BLOCK_OPERATION operation;
            
    /* Client that owns this buffer */
    void * hClient;

    /* Next buffer pointer */
    struct _DRV_IPF_BUFFER_OBJ * next;

    /* Previous buffer pointer */
    struct _DRV_IPF_BUFFER_OBJ * previous;

    /* Below members are applicable only in case of memory protect or unprotect Operation */
    /* Memory Protection Mode */
    DRV_IPF_PROT_MODE protMode;
    
    /* Memory Protection BPR Bit Position */
	uint8_t protBitPos;
    
} DRV_IPF_BUFFER_OBJ;

// *****************************************************************************
/* IPF Driver Global Instances Object

  Summary:
    Object used to keep track of data that is common to all instances of the
    IPF driver.

  Description:
    This object is used to keep track of any data that is common to all
    instances of the IPF driver.

  Remarks:
    None.
*/

typedef struct
{
    /* Set to true if all members of this structure
       have been initialized once */
    bool membersAreInitialized;

    /* Mutex to protect client object pool */
    OSAL_MUTEX_DECLARE(mutexClientObjects);

    /* Mutex to protect buffer queue object pool */
    OSAL_MUTEX_DECLARE(mutexBufferQueueObjects);

} DRV_IPF_COMMON_DATA_OBJ;

// *****************************************************************************
/* IPF SPI Flash Driver Instance Object

  Summary:
    Object used to keep any data required for an instance of the IPF SPI
    Flash driver.

  Description:
    This object is used to keep track of any data that must be maintained to 
    manage a single instance of the SPI Flash driver.

  Remarks:
    None.
*/

typedef struct
{
    /*  The module index of associated SPI driver */
    SYS_MODULE_INDEX    spiDriverModuleIndex;
    
    /* SPI Driver Handle */
    DRV_HANDLE spiDriverOpenHandle;

    /* The status of the driver */
    SYS_STATUS status;

    /* Flag to indicate this object is in use  */
    bool inUse;

    /* Flag to indicate that driver has been opened exclusively. */
    bool isExclusive;

    /* Keeps track of the number of clients that have opened this driver */
    uint32_t nClients;

    /* The buffer Queue Tail pointer*/
    /* elements are removed from the queue from the position pointed by
     * queueTail */
    DRV_IPF_BUFFER_OBJ  *queueTail;

    /* It is maximum number of requests that driver will queue */
    uint32_t queueSize;
    
    /* Current queue occupancy, it should never be more than queueSize */
    uint32_t queueOccupancy;

    /* Chip select pin port channel */
    PORTS_CHANNEL                       chipSelectPortChannel;

    /* Chip Select Bit pin position */
    PORTS_BIT_POS                       chipSelectBitPosition;

    /* State of the task */
    DRV_IPF_DATA_OBJECT_STATE    	state;

    /* State of the buffer object transfer */
    DRV_IPF_BUFFER_PROCESS_STATE    		bufferProcessState;

    /* temprary SPI Buffer Handle */
    DRV_HANDLE        spiBufferHandle;

    /* array to keep commands, addresses and data */
    uint8_t         commandAddressData[262];

    /* device geometry */
    SYS_FS_MEDIA_GEOMETRY		ipfDeviceGeometry;

    /* Read, Write and Erase region details */
    SYS_FS_MEDIA_REGION_GEOMETRY        memoryRegions[3];

    /* Hardware instance mutex */
    OSAL_MUTEX_DECLARE(mutexDriverInstance);

} DRV_IPF_OBJ;

// *****************************************************************************
/* IPF SPI Flash Driver Client Object

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
    DRV_IPF_OBJ * hDriver;

    /* The IO intent with which the client was opened */
    DRV_IO_INTENT   ioIntent;

    /* This flags indicates if the object is in use or is available */
    bool inUse;

    /* Event handler for this function */
    DRV_IPF_EVENT_HANDLER eventHandler;

    /* Client Status */
    DRV_IPF_CLIENT_STATUS clientStatus;

    /* Application Context associated with this client */
    uintptr_t context;
	
	/* Block Protection request history */
	uint8_t blockProtStatus[DRV_IPF_BLOCK_PROT_BYTES];

} DRV_IPF_CLIENT_OBJ;

/* Memory Prot helper functions */
uint8_t DRV_IPF_GetBlockProtectBitPosition(uintptr_t memAddress, DRV_IPF_PROT_MODE protMode);
void DRV_IPF_SetBitInArray(uint8_t arr[], uint8_t bitPosition);
bool DRV_IPF_CheckBitInArray(uint8_t arr[], uint8_t bitPosition);
void DRV_IPF_ClearBitInArray(uint8_t arr[], uint8_t bitPosition);
void DRV_IPF_BPRBitSet(uint8_t bitPosition, DRV_IPF_PROT_MODE protMode);
void DRV_IPF_BPRBitClear(uint8_t bitPosition, DRV_IPF_PROT_MODE protMode);

/* Hold and WP Pin Control functions */
void _DRV_IPF_WPAssert();
void _DRV_IPF_WPDeAssert();
void _DRV_IPF_HoldAssert();
void _DRV_IPF_HoldAssert();
void _DRV_IPF_HoldDeAssert();

/* FS Related functions */
void DRV_IPF_FS_BlockRead
(
    const DRV_HANDLE hClient,
    DRV_IPF_BLOCK_COMMAND_HANDLE * commandHandle,
    uint8_t *targetBuffer,
    uint32_t blockStart,
    uint32_t nBlock
);

uintptr_t DRV_IPF_AddressGet
(
    const DRV_HANDLE handle
);

DRV_IPF_COMMAND_STATUS DRV_IPF_CommandStatus
(
    const DRV_HANDLE handle,
    const DRV_IPF_BLOCK_COMMAND_HANDLE commandHandle
);


#endif //#ifndef _DRV_IPF_LOCAL_H

/*******************************************************************************
 End of File
*/

