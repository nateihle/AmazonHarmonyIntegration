/*******************************************************************************
  IPF SPI Flash Driver memory protection Dynamic implemention.

  Company:
    Microchip Technology Inc.

  File Name:
    drv_ipf_fs.c

  Summary:
    Source code for the SPI flash driver File System supporting function
	implementation.

  Description:
    This file contains the Source code for the SPI flash driver memory 
	protection supporting function implementation.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2015 released Microchip Technology Inc.  All rights reserved.

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


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "driver/spi_flash/pic32wk_ipf/src/drv_ipf_local.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data
// *****************************************************************************
// *****************************************************************************

#if defined(DRV_IPF_REGISTER_MEDIA)
void DRV_IPF_FS_BlockRead
(
    const DRV_HANDLE hClient,
    DRV_IPF_BLOCK_COMMAND_HANDLE * commandHandle,
    uint8_t *targetBuffer,
    uint32_t blockStart,
    uint32_t nBlock
)
{
    uint32_t address = blockStart + DRV_IPF_FS_BASE_ADDRESS;
    
    DRV_IPF_BlockRead(hClient,commandHandle, targetBuffer, address, nBlock);
}


uintptr_t DRV_IPF_AddressGet
(
    const DRV_HANDLE clientHandle
)
{
    
	DRV_IPF_CLIENT_OBJ * clientObj = NULL;

	/* Validate the driver handle */
	clientObj = (DRV_IPF_CLIENT_OBJ *) _DRV_IPF_DriverHandleValidate(clientHandle);
	if(clientObj == NULL)
	{
		/* We got an invalid client handle */
		SYS_DEBUG(0, "DRV_IPF: Invalid Driver Handle \n");
		return;
	}    

    return DRV_IPF_FS_BASE_ADDRESS;
}

// *****************************************************************************
/* Function:
    DRV_IPF_COMMAND_STATUS DRV_IPF_CommandStatus
    (
        const DRV_HANDLE handle, 
        const DRV_IPF_BLOCK_COMMAND_HANDLE commandHandle
    );

  Summary:
    Gets the current status of the command.

  Description:
    This routine gets the current status of the buffer. The application must use
    this routine where the status of a scheduled buffer needs to polled on. The
    function may return DRV_NVM_COMMAND_HANDLE_INVALID in a case where the buffer
    handle has expired. A buffer handle expires when the internal buffer object
    is re-assigned to another erase or write request. It is recommended that this
    function be called regularly in order to track the buffer status correctly.

    The application can alternatively register an event handler to receive write
    or erase operation completion events.

  Remarks:
    Refer to drv_nvm.h for usage information.
*/

DRV_IPF_COMMAND_STATUS DRV_IPF_CommandStatus
(
    const DRV_HANDLE handle,
    const DRV_IPF_BLOCK_COMMAND_HANDLE commandHandle
)
{
    DRV_IPF_CLIENT_OBJ * client;
    DRV_IPF_BUFFER_OBJ * bufferObj;
    /* Validate the driver handle */
    client = (DRV_IPF_CLIENT_OBJ *)_DRV_IPF_DriverHandleValidate(handle);

    bufferObj = (DRV_IPF_BUFFER_OBJ *)commandHandle;
    
    if(bufferObj->inUse == false || bufferObj->hClient != client)
        return DRV_IPF_COMMAND_COMPLETED;
    else if (bufferObj->nCurrentBlocks == 0)
        return DRV_IPF_COMMAND_QUEUED;
    else if(bufferObj->nCurrentBlocks < bufferObj->size)
        return DRV_IPF_COMMAND_IN_PROGRESS;
    else if (bufferObj->nCurrentBlocks == bufferObj->size)
        return DRV_IPF_COMMAND_COMPLETED;
}

#endif
