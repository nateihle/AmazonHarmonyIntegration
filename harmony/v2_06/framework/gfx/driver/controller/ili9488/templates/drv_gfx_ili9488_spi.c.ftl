/*******************************************************************************
  MPLAB Harmony Generated Driver Implementation File

  File Name:
    drv_gfx_ili9488_intf.c

  Summary:
    Implements 4-line SPI interface for ILI9488

  Description:
    Implements 4-line SPI interface for the ILI9488.

    Created with MPLAB Harmony Version ${CONFIG_MPLAB_HARMONY_VERSION_STRING}
 *******************************************************************************/
// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

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
// DOM-IGNORE-END

#include "system_config.h"
#include "system_definitions.h"

#include "gfx/hal/inc/gfx_common.h"
#include "gfx/hal/inc/gfx_context.h"

#include "drv_gfx_ili9488_cmd_defs.h"
#include "drv_gfx_ili9488_common.h"

/** SPI_TRANS_STATUS

  Summary:
    Enum type of SPI transaction status

 */
typedef enum 
{
    SPI_TRANS_IDLE,
    SPI_TRANS_CMD_WR_PENDING,
    SPI_TRANS_CMD_RD_PENDING,
    SPI_TRANS_DONE,
    SPI_TRANS_FAIL,
} SPI_TRANS_STATUS;

/** ILI9488_SPI_PRIV

  Summary:
    Structure contains status and handles for SPI interface.
    
 */
typedef struct 
{
    /* SPI Device handle */
    DRV_HANDLE drvSPIHandle;

    /* Write buffer handle */
    DRV_SPI_BUFFER_HANDLE drvSPIWRBUFHandle;

    /* Read buffer handle */
    DRV_SPI_BUFFER_HANDLE drvSPIRDBUFHandle;

    /* SPI transaction status */
    volatile SPI_TRANS_STATUS drvSPITransStatus;
} ILI9488_SPI_PRIV;

/* ************************************************************************** */

/** 
  Function:
    static void ILI9488_SPI_CallBack(DRV_SPI_BUFFER_EVENT event, 
                                     DRV_SPI_BUFFER_HANDLE bufferHandle, 
                                     void * context )

  Summary:
    Callback function called by SPI driver to deliver events.

  Description:

    This callback will set the ILI9488 SPI driver's SPI transaction status 
    based on the event.


  Parameters:
    event           - SPI buffer event
    bufferHandle    - SPI buffer handle
    context         - SPI transaction context

  Returns:
    None

 */
static void ILI9488_SPI_CallBack(DRV_SPI_BUFFER_EVENT event,
                                 DRV_SPI_BUFFER_HANDLE bufferHandle,
                                 void * context) 
{
    SPI_TRANS_STATUS *status = (SPI_TRANS_STATUS *) context;

    if (!status)
        return;

    switch (event) 
    {
        case DRV_SPI_BUFFER_EVENT_COMPLETE:
            *status = SPI_TRANS_DONE;
            break;
        case DRV_SPI_BUFFER_EVENT_ERROR:
            *status = SPI_TRANS_FAIL;
        default:
            break;
    }
}

/** 
  Function:
    static GFX_Result ILI9488_Intf_Read(struct ILI9488_DRV *drv, 
                                        uint8_t cmd, 
                                        uint8_t *data,
                                        int bytes)

  Summary:
    Sends read command and returns response from the ILI9488 device.

  Description:
    This function will do a SPI write operation to send the read command 
    to the ILI9488, and then do a SPI read operation to read the response.

  Parameters:
    drv         - ILI9488 driver handle
    cmd         - Read command
    data        - Buffer to store read data
    bytes       - Number of bytes to read
 
  Returns:
    * GFX_SUCCESS       - Operation successful
    * GFX_FAILURE       - Operation failed
 
  Remarks:
    This is a full-blocking read call, and waits for the SPI transaction to 
    complete. 


 */
static GFX_Result ILI9488_Intf_Read(struct ILI9488_DRV *drv,
                                    uint8_t cmd,
                                    uint8_t *data,
                                    int bytes) 
{
    GFX_Result returnValue = GFX_FAILURE;
    ILI9488_SPI_PRIV *spiPriv = NULL;

    if ((!drv) || (!data) || (bytes <= 0))
        return GFX_FAILURE;

    spiPriv = (ILI9488_SPI_PRIV *) drv->port_priv;

    //Assert SS = LOW and D/CX = LOW (command)
    ILI9488_SPI_DCX_Command();
    ILI9488_SPI_SS_Assert();

    spiPriv->drvSPITransStatus = SPI_TRANS_CMD_WR_PENDING;
    spiPriv->drvSPIWRBUFHandle = DRV_SPI_BufferAddWrite2(spiPriv->drvSPIHandle,
            (void*) &cmd,
            1,
            ILI9488_SPI_CallBack,
            (void*) &spiPriv->drvSPITransStatus,
            NULL);
    if (DRV_SPI_BUFFER_HANDLE_INVALID == spiPriv->drvSPIWRBUFHandle)
    {
        ILI9488_SPI_SS_Deassert();
        return GFX_FAILURE;
    }

    //Wait for the callback (full block/no timeout)
    while (SPI_TRANS_CMD_WR_PENDING == spiPriv->drvSPITransStatus);

    // Read data
    if ((SPI_TRANS_DONE == spiPriv->drvSPITransStatus))
    {
        ILI9488_SPI_DCX_Data();

        spiPriv->drvSPITransStatus = SPI_TRANS_CMD_RD_PENDING;
        spiPriv->drvSPIRDBUFHandle = DRV_SPI_BufferAddRead2(spiPriv->drvSPIHandle,
                (void *) data,
                bytes,
                ILI9488_SPI_CallBack,
                (void *) &spiPriv->drvSPITransStatus,
                0);
        if (DRV_SPI_BUFFER_HANDLE_INVALID == spiPriv->drvSPIWRBUFHandle)
        {
            ILI9488_SPI_DCX_Command();
            ILI9488_SPI_SS_Deassert();
            return GFX_FAILURE;
        }


        //Wait for the callback (full block/no timeout)
        while (SPI_TRANS_CMD_RD_PENDING == spiPriv->drvSPITransStatus);
    }

    ILI9488_SPI_DCX_Command();
    ILI9488_SPI_SS_Deassert();

    if (SPI_TRANS_DONE == spiPriv->drvSPITransStatus)
        returnValue = GFX_SUCCESS;

    spiPriv->drvSPITransStatus = SPI_TRANS_IDLE;

    return returnValue;
}

/** 
  Function:
    GFX_Result ILI9488_Intf_WriteCmd(struct ILI9488_DRV *drv,
                                     uint8_t cmd,
                                     uint8_t *parms, 
                                     int num_parms)

  Summary:
    Sends write command and parameters to the ILI9488 device.

  Description:
    This function will do a write operation to send the write command 
    and its parameters to the ILI9488.

  Parameters:
    drv         - ILI9488 driver handle
    cmd         - Write command
    parms       - Pointer to array of 8-bit parameters
    bytes       - number of command parameters
 
  Returns:
    * GFX_SUCCESS       - Operation successful
    * GFX_FAILURE       - Operation failed
 
  Remarks:
    In SPI mode, this is a full-blocking call and waits for the SPI transaction
    to complete. 

 */
GFX_Result ILI9488_Intf_WriteCmd(struct ILI9488_DRV *drv,
                                 uint8_t cmd,
                                 uint8_t *parms,
                                 int num_parms) 
{
    GFX_Result returnValue = GFX_FAILURE;
    ILI9488_SPI_PRIV *spiPriv = NULL;

    if (!drv)
        return GFX_FAILURE;

    spiPriv = (ILI9488_SPI_PRIV *) drv->port_priv;

    //Assert SS = LOW and D/CX = LOW (command)
    ILI9488_SPI_DCX_Command();
    ILI9488_SPI_SS_Assert();

    spiPriv->drvSPITransStatus = SPI_TRANS_CMD_WR_PENDING;
    spiPriv->drvSPIWRBUFHandle = DRV_SPI_BufferAddWrite2(spiPriv->drvSPIHandle,
                                        (void *) &cmd,
                                        1,
                                        ILI9488_SPI_CallBack,
                                        (void *) &spiPriv->drvSPITransStatus,
                                        NULL);
    if (DRV_SPI_BUFFER_HANDLE_INVALID == spiPriv->drvSPIWRBUFHandle)
    {
        ILI9488_SPI_SS_Deassert();
        return GFX_FAILURE;
    }

    //Wait for the callback (full block/no timeout)
    while (SPI_TRANS_CMD_WR_PENDING == spiPriv->drvSPITransStatus);

    // Send command parameters if any
    if ((SPI_TRANS_DONE == spiPriv->drvSPITransStatus) && (num_parms > 0))
    {
        ILI9488_SPI_DCX_Data();

        spiPriv->drvSPITransStatus = SPI_TRANS_CMD_WR_PENDING;
        spiPriv->drvSPIWRBUFHandle = DRV_SPI_BufferAddWrite2(spiPriv->drvSPIHandle,
                                            (void *) parms,
                                            (size_t) num_parms,
                                            ILI9488_SPI_CallBack,
                                            (void *) &spiPriv->drvSPITransStatus,
                                            NULL);
        if (DRV_SPI_BUFFER_HANDLE_INVALID == spiPriv->drvSPIWRBUFHandle)
        {
            ILI9488_SPI_DCX_Command();
            ILI9488_SPI_SS_Deassert();
            return GFX_FAILURE;
        }

        //Wait for the callback (full block/no timeout)
        while (SPI_TRANS_CMD_WR_PENDING == spiPriv->drvSPITransStatus);
    }

    ILI9488_SPI_DCX_Command();
    ILI9488_SPI_SS_Deassert();

    if (SPI_TRANS_DONE == spiPriv->drvSPITransStatus)
        returnValue = GFX_SUCCESS;

    spiPriv->drvSPITransStatus = SPI_TRANS_IDLE;

    return returnValue;
}

/** 
  Function:
    GFX_Result ILI9488_Intf_WritePixels(struct ILI9488_DRV *drv,
                                              uint32_t start_x,
                                              uint32_t start_y,
                                              uint8_t *data,
                                              unsigned int num_pixels)

  Summary:
    Writes pixel data to ILI9488 GRAM from specified position.

  Description:
    This function will fist write the start column, page information, then 
    write the pixel data to the ILI9488 GRAM.

  Parameters:
    drv             - ILI9488 driver handle
    start_x         - Start column position
    start_y         - Start page position
    data            - Array of 8-bit pixel data (8-bit/pixel RGB)
    num_pixels      - Number of pixels
 
  Returns:
    * GFX_SUCCESS       - Operation successful
    * GFX_FAILURE       - Operation failed

  Remarks:
    In SPI mode, this function performs multiple full-blocking write calls to
    the SPI port and won't return until the SPI transaction completes.

 */
GFX_Result ILI9488_Intf_WritePixels(struct ILI9488_DRV *drv,
                                   uint32_t start_x,
                                   uint32_t start_y,
                                   uint8_t *data,
                                   unsigned int num_pixels)
{
    GFX_Result returnValue = GFX_FAILURE;
    uint8_t buf[4];

    //Set column
    buf[0] = (start_x >> 8);
    buf[1] = (start_x & 0xff);
    buf[2] = (((drv->gfx->display_info->rect.width - 1) & 0xff00) >> 8);
    buf[3] = ((drv->gfx->display_info->rect.width - 1) & 0xff);
    returnValue = ILI9488_Intf_WriteCmd(drv,
                                       ILI9488_CMD_COLUMN_ADDRESS_SET,
                                       buf,
                                       4);
    if (GFX_SUCCESS != returnValue)
        return GFX_FAILURE;

    //Set page
    buf[0] = (start_y >> 8);
    buf[1] = (start_y & 0xff);
    buf[2] = (((drv->gfx->display_info->rect.height - 1) & 0xff00) >> 8);
    buf[3] = ((drv->gfx->display_info->rect.height - 1) & 0xff);
    returnValue = ILI9488_Intf_WriteCmd(drv,
                                       ILI9488_CMD_PAGE_ADDRESS_SET,
                                       buf,
                                       4);
    if (GFX_SUCCESS != returnValue)
        return GFX_FAILURE;

    returnValue = ILI9488_Intf_WriteCmd(drv,
                                       ILI9488_CMD_MEMORY_WRITE,
                                       data,
                                       num_pixels * 3);

    return returnValue;
}

/** 
  Function:
    GFX_Result ILI9488_Intf_ReadPixels(struct ILI9488_DRV *drv,
                                      uint32_t x,
                                      uint32_t y,
                                      uint16_t *value,
                                      unsigned int num_pixels)

  Summary:
    Read pixel data from specified position in ILI9488 GRAM.

  Description:
    This function will first write the start column, page information, then
    read the pixel data from the ILI9488 GRAM.

  Parameters:
    drv             - ILI9488 driver handle
    x               - Column position
    y               - Page position
    value           - Value to store the read pixel color (8-bit/pixel RGB)
    num_pixels      - Number of pixels to read
 
  Returns:
    * GFX_SUCCESS       - Operation successful
    * GFX_FAILURE       - Operation failed

  Remarks
    For SPI mode, this function performs multiple full-blocking write/read calls
    to the SPI port and won't return until the SPI transaction completes.

 */
GFX_Result ILI9488_Intf_ReadPixels(struct ILI9488_DRV *drv,
                                  uint32_t x,
                                  uint32_t y,
                                  uint8_t *value,
                                  unsigned int num_pixels)
{
    GFX_Result returnValue = GFX_FAILURE;
    ILI9488_SPI_PRIV *spiPriv = (ILI9488_SPI_PRIV *) drv->port_priv;
    uint8_t cmd = ILI9488_CMD_MEMORY_READ;
    uint8_t dummy;
    uint8_t buf[4];

    //Set column
    buf[0] = ((x & 0xff00) >> 8);
    buf[1] = (x & 0xff);
    buf[2] = (((drv->gfx->display_info->rect.width - 1) & 0xff00) >> 8);
    buf[3] = ((drv->gfx->display_info->rect.width - 1) & 0xff);
    returnValue = ILI9488_Intf_WriteCmd(drv,
                                       ILI9488_CMD_COLUMN_ADDRESS_SET,
                                       buf,
                                       4);
    if (GFX_SUCCESS != returnValue)
        return GFX_FAILURE;

    //Set page
    buf[0] = ((y & 0xff00) >> 8);
    buf[1] = (y & 0xff);
    buf[2] = (((drv->gfx->display_info->rect.height - 1) & 0xff00) >> 8);
    buf[3] = ((drv->gfx->display_info->rect.height - 1) & 0xff);
    returnValue = ILI9488_Intf_WriteCmd(drv,
                                       ILI9488_CMD_PAGE_ADDRESS_SET,
                                       buf,
                                       4);
    if (GFX_SUCCESS != returnValue)
        return GFX_FAILURE;

    //Assert SS = LOW and D/CX = LOW (command)
    ILI9488_SPI_DCX_Command();
    ILI9488_SPI_SS_Assert();

    spiPriv->drvSPITransStatus = SPI_TRANS_CMD_WR_PENDING;
    spiPriv->drvSPIWRBUFHandle = DRV_SPI_BufferAddWrite2(spiPriv->drvSPIHandle,
                                            (void*) &cmd,
                                            1,
                                            ILI9488_SPI_CallBack,
                                            (void*) &spiPriv->drvSPITransStatus,
                                            NULL);
    if (DRV_SPI_BUFFER_HANDLE_INVALID == spiPriv->drvSPIWRBUFHandle)
    {
        ILI9488_SPI_SS_Deassert();
        return GFX_FAILURE;
    }

    //Wait for the callback (full block/no timeout)
    while (SPI_TRANS_CMD_WR_PENDING == spiPriv->drvSPITransStatus);

    // Read the pixel data from display GRAM
    if ((SPI_TRANS_DONE == spiPriv->drvSPITransStatus))
    {
        ILI9488_SPI_DCX_Data();

        // Read the dummy byte
        spiPriv->drvSPITransStatus = SPI_TRANS_CMD_RD_PENDING;
        spiPriv->drvSPIRDBUFHandle = DRV_SPI_BufferAddRead2(spiPriv->drvSPIHandle,
                                        (void *) &dummy,
                                        1,
                                        ILI9488_SPI_CallBack,
                                        (void *) &spiPriv->drvSPITransStatus,
                                        0);
        if (DRV_SPI_BUFFER_HANDLE_INVALID == spiPriv->drvSPIWRBUFHandle)
        {
            ILI9488_SPI_DCX_Command();
            ILI9488_SPI_SS_Deassert();
            return GFX_FAILURE;
        }

        //Wait for the callback (full block/no timeout)
        while (SPI_TRANS_CMD_RD_PENDING == spiPriv->drvSPITransStatus);

        // Read the valid pixels
        spiPriv->drvSPITransStatus = SPI_TRANS_CMD_RD_PENDING;
        spiPriv->drvSPIRDBUFHandle = DRV_SPI_BufferAddRead2(spiPriv->drvSPIHandle,
                                            (void *) value,
                                            num_pixels * 3,
                                            ILI9488_SPI_CallBack,
                                            (void *) &spiPriv->drvSPITransStatus,
                                            0);
        if (DRV_SPI_BUFFER_HANDLE_INVALID == spiPriv->drvSPIWRBUFHandle)
        {
            ILI9488_SPI_DCX_Command();
            ILI9488_SPI_SS_Deassert();
            return GFX_FAILURE;
        }

        //Wait for the callback (full block/no timeout)
        while (SPI_TRANS_CMD_RD_PENDING == spiPriv->drvSPITransStatus);
    }

    ILI9488_SPI_DCX_Command();
    ILI9488_SPI_SS_Deassert();

    if (SPI_TRANS_DONE == spiPriv->drvSPITransStatus)
        returnValue = GFX_SUCCESS;

    spiPriv->drvSPITransStatus = SPI_TRANS_IDLE;

    return returnValue;
}

/** 
   Function:
    GFX_Result ILI9488_Intf_ReadCmd(struct ILI9488_DRV *drv, 
                                          uint8_t cmd, 
                                          uint8_t *data,
                                          int bytes);

  Summary:
    Sends read command and reads response from ILI9488.

  Description:
    This function will fist write the the read command and then read back the 
    response from the ILI9488 GRAM.

  Parameters:
    drv             - ILI9488 driver handle
    cmd             - Read command
    data            - Buffer to store the read data to
    bytes           - Number of bytes to read
 
  Returns:
    * GFX_SUCCESS       Operation successful
    * GFX_FAILURE       Operation failed
 
  Remarks:
    This function only supports 8-, 24- or 32-bit reads.
    In SPI mode, this function performs multiple full-blocking write/read calls 
    to the SPI port and won't return until the SPI transaction completes.

 */
GFX_Result ILI9488_Intf_ReadCmd(struct ILI9488_DRV *drv,
                               uint8_t cmd,
                               uint8_t *data,
                               int bytes)
{
    GFX_Result returnValue = GFX_FAILURE;
    uint8_t buff[5];

    //API supports only 8-, 24-, or 32-bit reads
    if ((!drv) || (!data) ||
        ((bytes != 1) && (bytes != 3) && (bytes != 4)))
        return GFX_FAILURE;

    returnValue = ILI9488_Intf_Read(drv, cmd, buff, bytes + 1);
    if (returnValue == GFX_SUCCESS)
    {
        switch (bytes)
        {
            case 1:
                data[0] = buff[0];
                break;
                // For 3 or 4-byte reads, ili9488 requires an extra dummy clock 
                // before read data becomes available so shift left 1 bit, and 
                // use MSB of next bytes as LSB
            case 4:
                data[3] = (buff[3] << 1) | (buff[4] >> 7);
                // no break, fall through
            case 3:
                data[2] = (buff[2] << 1) | (buff[3] >> 7);
                data[1] = (buff[1] << 1) | (buff[2] >> 7);
                data[0] = (buff[0] << 1) | (buff[1] >> 7);
                break;
            default:
                break;
        }
    }

    return returnValue;
}

// *****************************************************************************

/** 
  Function:
    GFX_Result ILI9488_Intf_Open(ILI9488_DRV *drv, unsigned int index)

  Summary:
    Opens the specified port to the ILI9488 device.

  Description:
    In SPI mode, this function will open the SPI port, allocate the port-specific
    data structures and set the port operation handler functions. When done 
    using the port, ILI9488_Intf_Close must be called to free up the data 
    structures and close the port.

  Parameters:
    drv         - ILI9488 driver handle
    index       - Port index
 
  Returns:
    * GFX_SUCCESS       - Operation successful
    * GFX_FAILURE       - Operation failed

 */
GFX_Result ILI9488_Intf_Open(ILI9488_DRV *drv, unsigned int index)
{
    ILI9488_SPI_PRIV *spiPriv = NULL;

    if (!drv)
        return GFX_FAILURE;

    spiPriv = (ILI9488_SPI_PRIV *) 
                drv->gfx->memory.calloc(1, sizeof (ILI9488_SPI_PRIV));

    spiPriv->drvSPIHandle = DRV_SPI_Open(index, DRV_IO_INTENT_READWRITE);
    if (DRV_HANDLE_INVALID == spiPriv->drvSPIHandle)
    {
        drv->gfx->memory.free(spiPriv);

        return GFX_FAILURE;
    }

    drv->port_priv = (void *) spiPriv;

    return GFX_SUCCESS;
}

/** 
  Function:
    void ILI9488_Intf_Close(ILI9488_DRV *drv)

  Summary:
    Closes the HW interface to the ILI9488 device.

  Description:
    This function will close the specified interface, free the port-specific
    data structures and unset the port operation handler functions.

  Parameters:
    drv         - ILI9488 driver handle
 
  Returns:
    None.

 */
void ILI9488_Intf_Close(ILI9488_DRV *drv) 
{
    ILI9488_SPI_PRIV *spiPriv = NULL;

    if (!drv)
        return;

    spiPriv = (ILI9488_SPI_PRIV *) drv->port_priv;

    DRV_SPI_Close(spiPriv->drvSPIHandle);

    drv->gfx->memory.free(spiPriv);

    drv->port_priv = NULL;

}
/* *****************************************************************************
 End of File
 */
