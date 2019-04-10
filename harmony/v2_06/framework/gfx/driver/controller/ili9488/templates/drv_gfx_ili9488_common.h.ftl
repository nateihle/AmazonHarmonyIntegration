/*******************************************************************************
 Module for Microchip Graphics Library - Hardware Abstraction Layer

  Company:
    Microchip Technology Inc.

  File Name:
    drv_gfx_ili9488_common.h

  Summary:
    Contains ILI9488 driver type definitions and interface prototypes.

  Description:
    Contains ILI9488 driver type definitions and interface prototypes.
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

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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

#ifndef DRV_GFX_ILI9488_COMMON_H    /* Guard against multiple inclusion */
#define DRV_GFX_ILI9488_COMMON_H

<#if CONFIG_DRV_GFX_CONTROLLER_TYPE == "ILI9488 (SPI 4-LINE)">
#define ILI9488_SPI_SS_Assert()     BSP_ILI9488_SPI_CSXStateSet(0)
#define ILI9488_SPI_SS_Deassert()   BSP_ILI9488_SPI_CSXStateSet(1)
#define ILI9488_SPI_DCX_Command()   BSP_ILI9488_SPI_DCXStateSet(0)
#define ILI9488_SPI_DCX_Data()      BSP_ILI9488_SPI_DCXStateSet(1)
</#if>

#define ILI9488_Reset_Assert()      BSP_DisplayResetStateSet(0)
#define ILI9488_Reset_Deassert()    BSP_DisplayResetStateSet(1)

#define ILI9488_Backlight_On()      BSP_DisplayBacklightStateSet(1)
#define ILI9488_Backlight_Off()     BSP_DisplayBacklightStateSet(0)

/** ILI9488_CMD_PARAM

  Summary:
    Structure contains command and parameter information.

*/
typedef struct 
{
    /* Command */
    uint8_t cmd;
    
    /* Number of command parameters */
    uint8_t parmCount;
    
    /* Command parameters, max of 4 */
    uint8_t parms[4];
} ILI9488_CMD_PARAM;

/** ILI9488_DRV_STATE

  Summary:
    Enum of ILI9488 driver states.
    
*/
typedef enum
{
    INIT = 0,
    RUN
} ILI9488_DRV_STATE;

/** ILI9488_DRV

  Summary:
    Structure contains driver-specific data and ops pointers.
    
*/
typedef struct ILI9488_DRV 
{
    /* GFX context pointer */
    GFX_Context* gfx;
    
    /* Driver state */
    ILI9488_DRV_STATE state;
        
    /* Line buffer information */
    uint8_t *pixelBuffer;
    int currentLine;
    int lineX_Start;
    int lineX_End;
    GFX_Bool linePending;

    /* bytes per pixel buffer */
    unsigned int bytesPerPixelBuffer;
    
    /* Port-specific private data */
    void *port_priv;
} ILI9488_DRV;

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
GFX_Result ILI9488_Intf_Open(ILI9488_DRV *drv, unsigned int index);

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
void ILI9488_Intf_Close(ILI9488_DRV *drv);

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
                                       int num_parms);

/** 
  Function:
    GFX_Result ILI9488_Intf_ReadCmd(struct ILI9488_DRV *drv, 
                                          uint8_t cmd, 
                                          uint8_t *data,
                                          int bytes);

  Summary:
    Sends read command and reads response from ILI9488.

  Description:
    This function will first write the the read command and then read back the 
    response from the ILI9488 GRAM.

  Parameters:
    drv             - ILI9488 driver handle
    cmd             - Read command
    data            - Buffer to store the read data to
    bytes           - Number of bytes to read
 
  Returns:
    * GFX_SUCCESS       - Operation successful
    * GFX_FAILURE       - Operation failed
 
  Remarks:
    This function only supports 8-, 24- or 32-bit reads.
    For SPI mode, this function performs multiple full-blocking write/read calls
    to the SPI port and won't return until the SPI transaction completes.

 */
GFX_Result ILI9488_Intf_ReadCmd(struct ILI9488_DRV *drv,
                                      uint8_t cmd,
                                      uint8_t *data,
                                      int bytes);

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
                                          unsigned int num_pixels);

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
                                  unsigned int num_pixels);

/* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif // DRV_GFX_ILI9488_COMMON_H

/* *****************************************************************************
 End of File
 */
