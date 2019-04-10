/*******************************************************************************
  ILI9488 Display Top-Level Driver Source File

  File Name:
    drv_gfx_ili9488.c

  Summary:
    Top level driver for ILI9488.

  Description:
    Build-time generated implementation for the ILI9488 Driver.

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

#include "drv_gfx_ili9488_cmd_defs.h"
#include "drv_gfx_ili9488_common.h"
#include "drv_gfx_ili9488.h"

// Number of layers
#define LAYER_COUNT     1

// Default max width/height of ILI9488 frame
#define DISPLAY_DEFAULT_WIDTH   320
#define DISPLAY_DEFAULT_HEIGHT  480

#define DISPLAY_WIDTH   ${CONFIG_DRV_GFX_DISPLAY_WIDTH}
#define DISPLAY_HEIGHT  ${CONFIG_DRV_GFX_DISPLAY_HEIGHT}

<#if CONFIG_DRV_GFX_CONTROLLER_TYPE == "ILI9488 (SPI 4-LINE)">
#define BYTES_PER_PIXEL_BUFFER 3
</#if>

#define PIXEL_BUFFER_WIDTH DISPLAY_WIDTH
<#if CONFIG_DRV_GFX_ILI9488_PIXEL_BUFFER == "Frame">
#define PIXEL_BUFFER_HEIGHT DISPLAY_HEIGHT
<#else>
#define PIXEL_BUFFER_HEIGHT 1
</#if>

#define PIXEL_BUFFER_COLOR_MODE GFX_COLOR_MODE_RGB_565

// Driver name
const char* DRIVER_NAME = "ILI9488";

/** initCmdParm

  Summary:
    Table of command/parameter(s) used to initialize the ILI9488.

  Description:
    This table contains command/parameter(s) values that are written to the
    ILI9488 during initialization.

  Remarks:
    Add project-specific initialization values for the ILI9488 here.
 */
ILI9488_CMD_PARAM initCmdParm[] =
{
<#if CONFIG_DRV_GFX_CONTROLLER_TYPE == "ILI9488 (SPI 4-LINE)">
    {ILI9488_CMD_INTERFACE_PIXEL_FORMAT_SET, 1, {ILI9488_COLOR_PIX_FMT_18BPP}},
    {ILI9488_CMD_SET_IMAGE_FUNCTION, 1, {0x00}},
</#if>
    {ILI9488_CMD_INTERFACE_MODE_CONTROL, 1, {0x00}},
    {ILI9488_CMD_MEMORY_ACCESS_CONTROL, 1, {(
<#if CONFIG_DRV_GFX_ILI9488_MADCTL_RGB_BGR == "BGR">
                                             ILI9488_MADCTL_RGB_BGR_ORDER_CTRL |
</#if>
<#if CONFIG_DRV_GFX_ILI9488_MADCTL_ROW_COLUMN_EXCHANGE == "Reverse Mode">
                                             ILI9488_MADCTL_ROW_COLUMN_EXCHANGE |
</#if>
<#if CONFIG_DRV_GFX_ILI9488_MADCTL_COL_ADDRESS_ORDER == "Right to Left">
                                             ILI9488_MADCTL_COL_ADDR_ORDER |
</#if>
<#if CONFIG_DRV_GFX_ILI9488_MADCTL_ROW_ADDRESS_ORDER == "Bottom to Top">
                                             ILI9488_MADCTL_ROW_ADDR_ORDER |
</#if>
                                            0)}},

    {ILI9488_CMD_SLEEP_OUT, 0, {0x00}},
    {ILI9488_CMD_DISPLAY_ON, 0, {0x00}},
};

/* ************************************************************************** */

/**
  Function:
    static void ILI9488_DelayMS(int ms)

  Summary:
    Delay helper function.

  Description:
    This is a helper function for delay using the system tick timer.

  Parameters:
    ms      - Delay in milliseconds

  Returns:
    None.

*/

static inline void ILI9488_DelayMS(int ms)
{
    #define GET_TICKS() __builtin_mfc0(9, 0)

    uint32_t prev_tick = GET_TICKS();
    while ((GET_TICKS() - prev_tick) < (SYS_CLK_FREQ/1000 * ms));
}
/**
  Function:
    static GFX_Result ILI9488_Reset(void)

  Summary:
    Toggles the hardware reset to the ILI9488.

  Description:
    This function toggles the GPIO pin for asserting reset to the ILI9488.

  Parameters:
    None

  Returns:
    None

*/
static GFX_Result ILI9488_Reset(void)
{
    ILI9488_Reset_Deassert();
    ILI9488_DelayMS(10);
    ILI9488_Reset_Assert();
    ILI9488_DelayMS(10);
    ILI9488_Reset_Deassert();
    ILI9488_DelayMS(10);

    return GFX_SUCCESS;
}

<#if CONFIG_DRV_GFX_ILI9488_PIXEL_BUFFER != "Frame">
/**
  Function:
    static GFX_Color ILI9488_PixelGet(const GFX_PixelBuffer *buf,
                                      const GFX_Point *pnt)

  Summary:
    HAL interface function for reading pixel value from the ILI9488 GRAM.

  Description:
    This function uses the interface-specific call to read pixel value from
    the ILI9488 GRAM.

  Parameters:
    buf     - GFX_PixelBuffer pointer where pixel value will be stored
    pnt     - GFX_Point pointer describing pixel position

  Returns:

    GFX_UNSUPPORTED   Operation is not supported
    GFX_FAILURE       Operation failed
    GFX_SUCCESS       Operation successful


*/
static GFX_Color ILI9488_PixelGet(const GFX_PixelBuffer *buf,
                                  const GFX_Point *pnt)
{
    GFX_Context *context = GFX_ActiveContext();
    GFX_Result returnValue;
    ILI9488_DRV *drv;
    uint8_t data[BYTES_PER_PIXEL_BUFFER];
    GFX_Color pixel;

    if (!context)
        return GFX_FAILURE;

    drv = (ILI9488_DRV *) context->driver_data;

    returnValue = ILI9488_Intf_ReadPixels(drv,
                                         pnt->x,
                                         pnt->y,
                                         data,
                                         1);

    if (returnValue == GFX_SUCCESS &&
        context->colorMode == GFX_COLOR_MODE_RGB_565)
    {
<#if CONFIG_DRV_GFX_CONTROLLER_TYPE == "ILI9488 (SPI 4-LINE)">
        pixel = ((data[0] & 0xf8) << 8);
        pixel |= ((data[1] & 0xfc) << 3);
        pixel |= ((data[2] & 0xf8) >> 3);
</#if>
    }
    else
        return GFX_FAILURE;

    return pixel;
}
</#if>

<#if CONFIG_DRV_GFX_ILI9488_PIXEL_BUFFER != "Frame">
/**
  Function:
    static GFX_Result ILI9488_SetPixel(const GFX_PixelBuffer *buf,
                                       const GFX_Point *pnt,
                                       GFX_Color color)

  Summary:
    HAL interface function for writing pixel value to the ILI9488 GRAM.

  Description:
    This function uses the interface-specific call to write pixel value to the
    ILI9488 GRAM.


  Parameters:
    buf     - GFX_PixelBuffer pointer
    pnt     - GFX_Point pointer describing pixel position
    color   - pixel color value (RGB565)

  Returns:
    * GFX_SUCCESS       - Operation successful
    * GFX_FAILURE       - Operation failed

*/
static GFX_Result ILI9488_SetPixel(const GFX_PixelBuffer *buf,
                            const GFX_Point *pnt,
                            GFX_Color color)
{
    GFX_Context *context = GFX_ActiveContext();
    GFX_Result returnValue = GFX_SUCCESS;
    ILI9488_DRV *drv;
<#if CONFIG_DRV_GFX_ILI9488_PIXEL_BUFFER == "Line">
    uint8_t *data;
    uint8_t *pixelBuffer;
<#else>
    uint8_t pixelBuffer[BYTES_PER_PIXEL_BUFFER];
</#if>

    if (!context)
        return GFX_FAILURE;

    drv = (ILI9488_DRV *) context->driver_data;

<#if CONFIG_DRV_GFX_ILI9488_PIXEL_BUFFER == "Line">
    //Assumes all pixel writes are linear
    data = drv->pixelBuffer;

    //Writing to new line, write pending line
    if (drv->currentLine != pnt->y)
    {
        if (drv->linePending)
        {
            returnValue = ILI9488_Intf_WritePixels(drv,
                                          drv->lineX_Start,
                                          drv->currentLine,
                                          &data[drv->lineX_Start * BYTES_PER_PIXEL_BUFFER],
                                          (drv->lineX_End - drv->lineX_Start + 1));

            drv->linePending = GFX_FALSE;
        }
    }

    if (drv->linePending == GFX_FALSE)
    {
        drv->linePending = GFX_TRUE;
        drv->lineX_Start = pnt->x;
        drv->currentLine = pnt->y;

        // Populate line buffer with pixels from display
        returnValue = ILI9488_Intf_ReadPixels(drv,
                            pnt->x,
                            pnt->y,
                            &data[pnt->x * BYTES_PER_PIXEL_BUFFER],
                            (context->display_info->rect.width - pnt->x + 1));
    }

    drv->lineX_End = pnt->x;
    pixelBuffer = (uint8_t *) &data[pnt->x * BYTES_PER_PIXEL_BUFFER];

</#if>

    if (context->colorMode == GFX_COLOR_MODE_RGB_565)
    {
<#if CONFIG_DRV_GFX_CONTROLLER_TYPE == "ILI9488 (SPI 4-LINE)">
        pixelBuffer[0] = ((color & 0xf800) >> 8);
        pixelBuffer[1] = ((color & 0x07e0) >> 3 );
        pixelBuffer[2] = ((color & 0x001f) << 3);
</#if>
    }
    else
    {
        return GFX_FAILURE;
    }

<#if CONFIG_DRV_GFX_ILI9488_PIXEL_BUFFER == "Pixel">
    returnValue = ILI9488_Intf_WritePixels(drv,
                                  pnt->x,
                                  pnt->y,
                                  pixelBuffer,
                                  1);
</#if>

    return returnValue;
}
</#if>

/**
  Function:
    GFX_Result ILI9488_InfoGet(GFX_DriverInfo *info)

  Summary:
    HAL interface function for providing driver information.

  Description:
    This function provides driver information to the HAL.

  Parameters:
    info    - Pointer to driver information

  Returns:
    * GFX_SUCCESS       - Operation successful
    * GFX_FAILURE       - Operation failed

*/
GFX_Result ILI9488_InfoGet(GFX_DriverInfo *info)
{
    if(info == NULL)
        return GFX_FAILURE;

    // populate info struct
    strcpy(info->name, DRIVER_NAME);
    info->color_formats = GFX_COLOR_MASK_RGB_565;
    info->layer_count = LAYER_COUNT;

    return GFX_SUCCESS;
}

/**
  Function:
    static GFX_Result ILI9488_Init(ILI9488_DRV *drv,
                                   ILI9488_CMD_PARAM *initVals,
                                   int numVals)

  Summary:
    Initializes the ILI9488 registers.

  Description:
    This function uses the register/parameter table initVals to program the
    ILI9488 registers to their initial values. It also uses the display size
    information from gfx context to set the area of the frame memory that the
    MCU can access.


  Parameters:
    drv         - ILI9488 driver handle
    initVals    - Table of command/parameters for initialization
    numVals     - Number of entries in initVals

  Returns:
    * GFX_SUCCESS       - Operation successful
    * GFX_FAILURE       - Operation failed

*/
static GFX_Result ILI9488_Init(ILI9488_DRV *drv,
                               ILI9488_CMD_PARAM *initVals,
                               int numVals)
{
    GFX_Result returnValue;
    uint8_t buf[5];
    unsigned int i;

    for (i = 0; i < numVals; i++, initVals++)
    {
        returnValue = ILI9488_Intf_WriteCmd(drv,
                                         initVals->cmd,
                                         initVals->parms,
                                         initVals->parmCount);
        if (GFX_SUCCESS != returnValue)
            return GFX_FAILURE;
    }

    buf[0] = 0;
    buf[1] = 0;
    buf[2] = (((drv->gfx->display_info->rect.width - 1)  & 0xFF00) >> 8);
    buf[3] = ((drv->gfx->display_info->rect.width - 1)  & 0xFF);
    returnValue = ILI9488_Intf_WriteCmd(drv,
                                     ILI9488_CMD_COLUMN_ADDRESS_SET,
                                     buf,
                                     4);
    if (GFX_SUCCESS != returnValue)
        return GFX_FAILURE;

    buf[0] = 0;
    buf[1] = 0;
    buf[2] = (((drv->gfx->display_info->rect.height - 1)  & 0xFF00) >> 8);
    buf[3] = ((drv->gfx->display_info->rect.height - 1)  & 0xFF);
    returnValue = ILI9488_Intf_WriteCmd(drv,
                                  ILI9488_CMD_PAGE_ADDRESS_SET,
                                  buf,
                                  4);
    if (GFX_SUCCESS != returnValue)
        return GFX_FAILURE;

    return returnValue;
}

<#if CONFIG_DRV_GFX_ILI9488_PIXEL_BUFFER == "Frame">
/**
  Function:
    static void layerSwapped(GFX_Layer* layer)

  Summary:
    Driver-specific implementation of GFX HAL Swapped function.

  Description:
    This function gets called by the graphics library and pushes the contents
    of the frame buffer to the ILI9488 GRAM.

  Parameters:
    layer   - GFX layer

  Returns:
    * GFX_SUCCESS       - Operation successful
    * GFX_FAILURE       - Operation failed

*/
static void layerSwapped(GFX_Layer* layer)
{
    GFX_Context *context = GFX_ActiveContext();
    unsigned int line = 0;
    GFX_PixelBuffer* buffer;
    uint8_t * buffer_to_tx;
    GFX_Point drawPoint;
<#if CONFIG_DRV_GFX_ILI9488_PALETTE_MODE = true>
    uint16_t * palette;
    unsigned int i;
</#if>
    ILI9488_DRV *drv;

    if(context == NULL)
        return;

    drv = (ILI9488_DRV *) context->driver_data;

    buffer = &context->layer.active->buffers[0].pb;

<#if CONFIG_DRV_GFX_ILI9488_PALETTE_MODE = true>
        palette = (uint16_t*) GFX_ActiveContext()->globalPalette;
</#if>

    //write out per line
    for (line = 0; line < PIXEL_BUFFER_HEIGHT; line += 1)
    {
        drawPoint.x = 0;
        drawPoint.y = line;

        buffer_to_tx = GFX_PixelBufferOffsetGet_Unsafe(buffer, &drawPoint);

<#if CONFIG_DRV_GFX_ILI9488_PALETTE_MODE = true>
        for (i = 0; i < PIXEL_BUFFER_WIDTH; i++)
            frameLine[i] = palette[buffer_to_tx[i]];

        ILI9488_Intf_WritePixels(drv,
                                 0,
                                 line,
                                 (uint8_t *) frameLine,
                                 PIXEL_BUFFER_WIDTH);
<#else>
        ILI9488_Intf_WritePixels(drv,
                                 0,
                                 line,
                                 buffer_to_tx,
                                 PIXEL_BUFFER_WIDTH);
</#if>
    }

}

GFX_Result layerSwapSet(GFX_Bool value)
{
    GFX_Context* context = GFX_ActiveContext();

    context->layer.active->swap = value;

    return GFX_SUCCESS;
}
</#if>

/**
  Function:
    static GFX_Result ILI9488_Update(void)

  Summary:
    Driver-specific implementation of GFX HAL update function.

  Description:
    On GFX update, this function flushes any pending pixels to the ILI9488.

  Parameters:
    None.

  Returns:
    * GFX_SUCCESS       - Operation successful
    * GFX_FAILURE       - Operation failed

*/
static GFX_Result ILI9488_Update(void)
{
    GFX_Context *context = GFX_ActiveContext();
    GFX_Result returnValue = GFX_SUCCESS;
    ILI9488_DRV *drv;

    if(context == NULL)
        return GFX_FAILURE;

    drv = (ILI9488_DRV *) context->driver_data;

    if(drv->state == INIT)
    {
        // perform driver initialization here
        ILI9488_Reset();

        returnValue = ILI9488_Init(drv,
                                   initCmdParm,
                                   sizeof(initCmdParm)/sizeof(ILI9488_CMD_PARAM));

        ILI9488_Backlight_On();

        drv->state = RUN;
    }

<#if CONFIG_DRV_GFX_ILI9488_PIXEL_BUFFER == "Line">
    if ((drv->state == RUN) && (drv->linePending == GFX_TRUE))
    {
        returnValue = ILI9488_Intf_WritePixels(drv,
                                      drv->lineX_Start,
                                      drv->currentLine,
                                      &drv->pixelBuffer[drv->lineX_Start * BYTES_PER_PIXEL_BUFFER],
                                      drv->lineX_End - drv->lineX_Start + 1);

        drv->linePending = GFX_FALSE;
    }
</#if>

<#if CONFIG_DRV_GFX_ILI9488_PIXEL_BUFFER == "Frame">
    if (context->layer.active->swap)
    {
        layerSwapped(context->layer.active);
        context->layer.active->swap = GFX_FALSE;
    }
</#if>

    return returnValue;
}

/**
  Function:
    static void ILI9488_Destroy(GFX_Context *context)

  Summary:
    Driver-specific implementation of GFX HAL destroy function.

  Description:
    This function closes the ILI9488 interface and frees up the data structures
    allocated by the driver.

  Parameters:
    context     - GFX context

  Returns:
    * GFX_SUCCESS       - Operation successful
    * GFX_FAILURE       - Operation failed

*/
static void ILI9488_Destroy(GFX_Context *context)
{
    // driver specific shutdown tasks
    ILI9488_DRV *drv = (ILI9488_DRV *) context->driver_data;

    //Close ILI9488 controller
    ILI9488_Intf_Close(drv);

    ILI9488_Backlight_Off();

<#if CONFIG_DRV_GFX_ILI9488_PIXEL_BUFFER == "Line">
    if (drv->pixelBuffer)
    {
        GFX_PixelBufferDestroy(&context->layer.layers[0].buffers[0].pb,
                               &context->memory);

        drv->pixelBuffer = GFX_NULL;
    }
</#if>

    if(context->driver_data != GFX_NULL)
    {
        context->memory.free(context->driver_data);
        context->driver_data = GFX_NULL;
    }

    // general default shutdown
    defDestroy(context);
}

/**
  Function:
    static void ILI9488_Initialize(GFX_Context *context)

  Summary:
    Driver-specific implementation of GFX HAL initialize function.

  Description:
    This function creates driver-specific data structures, initializes data
    needed by HAL, and opens the port-specific interface to the ILI9488 device.

  Parameters:
    context     - GFX context

  Returns:
    * GFX_SUCCESS       - Operation successful
    * GFX_FAILURE       - Operation failed

*/
static GFX_Result ILI9488_Initialize(GFX_Context *context)
{
    ILI9488_DRV *drv;
    GFX_Result returnValue = GFX_FAILURE;

    drv = (void *) context->memory.calloc(1, sizeof(ILI9488_DRV));
    if (!drv)
        return GFX_FAILURE;

    drv->gfx = context;
    drv->pixelBuffer = NULL;
    drv->state = INIT;

<#if CONFIG_DRV_GFX_ILI9488_PIXEL_BUFFER == "Line">
    drv->pixelBuffer = (void*) context->memory.calloc(
                               context->display_info->rect.width,
                               BYTES_PER_PIXEL_BUFFER);
    if (!drv->pixelBuffer)
    {
        context->memory.free(drv);
        return GFX_FAILURE;
    }
</#if>

    drv->bytesPerPixelBuffer = BYTES_PER_PIXEL_BUFFER;

    context->layer.layers[0].enabled = GFX_TRUE;
    context->layer.layers[0].visible = GFX_TRUE;

    context->layer.layers[0].vsync = GFX_FALSE;
    context->layer.layers[0].swap = GFX_FALSE;

    context->layer.layers[0].rect.local.x = 0;
    context->layer.layers[0].rect.local.y = 0;
    context->layer.layers[0].rect.local.width = context->display_info->rect.width;
    context->layer.layers[0].rect.local.height = context->display_info->rect.height;

    context->layer.layers[0].rect.display = context->layer.layers[0].rect.local;

    context->layer.layers[0].alphaEnable = GFX_FALSE;
    context->layer.layers[0].alphaAmount = 255;

    context->layer.layers[0].maskEnable = GFX_FALSE;
    context->layer.layers[0].maskColor = 0;

    context->layer.layers[0].buffer_count = 1;
    context->layer.layers[0].buffer_read_idx = 0;
    context->layer.layers[0].buffer_write_idx = 0;

    GFX_PixelBufferCreate(PIXEL_BUFFER_WIDTH,
                          PIXEL_BUFFER_HEIGHT,
                          PIXEL_BUFFER_COLOR_MODE,
                          (const void *) drv->pixelBuffer,
                          &context->layer.layers[0].buffers[0].pb);

    context->layer.layers[0].buffers[0].state = GFX_BS_MANAGED;

    //Open interface to ILI9488 controller
<#if CONFIG_DRV_GFX_CONTROLLER_TYPE == "ILI9488 (SPI 4-LINE)">
    returnValue = ILI9488_Intf_Open(drv, DRV_SPI_INDEX_${CONFIG_DRV_GFX_ILI9488_SPI_PORT});
</#if>
    if (GFX_FAILURE == returnValue)
    {
        if (drv)
            context->memory.free(drv);
        if (drv->pixelBuffer)
            context->memory.free(drv->pixelBuffer);

        return GFX_FAILURE;
    }

    context->driver_data = (void *) drv;

    // general default initialization
    if(defInitialize(context) == GFX_FAILURE)
    {
        if (drv)
            context->memory.free(drv);
        if (drv->pixelBuffer)
            context->memory.free(drv->pixelBuffer);

        return GFX_FAILURE;
    }

    return GFX_SUCCESS;
}

/**
  Function:
    static GFX_Result ILI9488_brightnessRangeGet(uint32_t *low, uint32_t *high)

  Summary:
    Driver-specific implementation of GFX HAL brightnessRangeGet function.

  Description:
    Stub function, operation not supported in driver.

  Returns:
     * GFX_UNSUPPORTED      - Operation not supported

*/
static GFX_Result ILI9488_brightnessRangeGet(uint32_t *low, uint32_t *high)
{
    return GFX_UNSUPPORTED;
}

/**
  Function:
    static GFX_Result ILI9488_brightnessSet(uint32_t val)

  Summary:
    Driver-specific implementation of GFX HAL brightnessSet function

  Description:
    Stub function, operation not supported in driver.

  Returns:
    * GFX_UNSUPPORTED       - Operation not supported

*/
static GFX_Result ILI9488_brightnessSet(uint32_t val)
{
    return GFX_UNSUPPORTED;
}

/**
  Function:
    static GFX_Result ILI9488_vsyncSet(GFX_Bool enable)

  Summary:
    Driver-specific implementation of GFX HAL layerVsyncSet function.

  Description:
     Stub function, operation not supported in driver.

  Returns:
    * GFX_UNSUPPORTED       - Operation not supported

*/
static GFX_Result ILI9488_vsyncSet(GFX_Bool enable)
{
    return GFX_UNSUPPORTED;
}

/**
  Function:
    static GFX_Result ILI9488_vsyncCallbackSet(GFX_SyncCallback_FnPtr cb)

  Summary:
    Driver-specific implementation of GFX HAL vsyncCallbackSet function.

  Description:
    Stub function, operation not supported in driver.


  Returns:
    * GFX_UNSUPPORTED       - Operation not supported

*/
static GFX_Result ILI9488_vsyncCallbackSet(GFX_SyncCallback_FnPtr cb)
{
    return GFX_UNSUPPORTED;
}

/**
  Function:
    static GFX_Result ILI9488_hsyncCallbackSet(GFX_SyncCallback_FnPtr cb)

  Summary:
    Driver-specific implementation of GFX HAL hsyncCallbackSet function.

  Description:
    Stub function, operation not supported in driver.


  Returns:
    * GFX_UNSUPPORTED       - Operation not supported

*/
static GFX_Result ILI9488_hsyncCallbackSet(GFX_SyncCallback_FnPtr cb)
{
    return GFX_UNSUPPORTED;
}

/**
  Function:
    sstatic GFX_Result ILI9488_layerActiveSet(uint32_t idx)

  Summary:
    Driver-specific implementation of GFX HAL layerActiveSet function.

  Description:
    Stub function, operation not supported in driver.

  Returns:
    * GFX_UNSUPPORTED       - Operation not supported

*/
static GFX_Result ILI9488_layerActiveSet(uint32_t idx)
{
    return GFX_UNSUPPORTED;
}

/**
  Function:
    static GFX_Result ILI9488_layerEnabledSet(GFX_Bool val)

  Summary:
    Driver-specific implementation of GFX HAL layerEnabledSet function.

  Description:
    Stub function, operation not supported in driver.

  Returns:
    * GFX_UNSUPPORTED       - Operation not supported

*/
static GFX_Result ILI9488_layerEnabledSet(GFX_Bool val)
{
    return GFX_UNSUPPORTED;
}

/**
  Function:
    static GFX_Result ILI9488_layerPositionSet(int32_t x, int32_t y)

  Summary:
    Driver-specific implementation of GFX HAL layerPositionSet function.

  Description:
    Stub function, operation not supported in driver.

  Returns:
    * GFX_UNSUPPORTED       - Operation not supported

*/
static GFX_Result ILI9488_layerPositionSet(int32_t x, int32_t y)
{
    return GFX_UNSUPPORTED;
}

/**
  Function:
    static GFX_Result ILI9488_layerSizeSet(int32_t width, int32_t height)

  Summary:
    Driver-specific implementation of GFX HAL layerSizeSet function.

  Description:
     Stub function, operation not supported in driver.

  Returns:
    * GFX_UNSUPPORTED     Operation not supported

*/
static GFX_Result ILI9488_layerSizeSet(int32_t width, int32_t height)
{
    return GFX_UNSUPPORTED;
}

/**
  Function:
    static GFX_Result ILI9488_layerBufferCountSet(uint32_t count)

  Summary:
    Driver-specific implementation of GFX HAL layerBufferCountSet function.

  Description:
    Stub function, operation not supported in driver.

  Returns:
    * GFX_UNSUPPORTED       - Operation not supported

*/
static GFX_Result ILI9488_layerBufferCountSet(uint32_t count)
{
    return GFX_UNSUPPORTED;
}

/**
  Function:
    static GFX_Result ILI9488_layerBufferAddressSet(uint32_t idx,
                                                    GFX_Buffer address)

  Summary:
    Driver-specific implementation of GFX HAL layerBufferAddressSet function.

  Description:
    Stub function, operation not supported in driver.

  Returns:
    * GFX_UNSUPPORTED       - Operation not supported

*/
static GFX_Result ILI9488_layerBufferAddressSet(uint32_t idx, GFX_Buffer address)
{
    return GFX_UNSUPPORTED;
}

/**
  Function:
    static GFX_Result ILI9488_layerBufferCoherentSet(uint32_t idx,
                                                     GFX_Bool coherent)

  Summary:
    Driver-specific implementation of GFX HAL layerBufferCoherentSet function

  Description:
    Stub function, operation not supported in driver.

  Returns:
    * GFX_UNSUPPORTED       - Operation not supported

*/
static GFX_Result ILI9488_layerBufferCoherentSet(uint32_t idx, GFX_Bool coherent)
{
    return GFX_UNSUPPORTED;
}

/**
  Function:
    static GFX_Result ILI9488_layerBufferAllocate(uint32_t idx)

  Summary:
    Driver-specific implementation of GFX HAL layerBufferAllocate function.

  Description:
    Stub function, operation not supported in driver.


  Returns:
    * GFX_UNSUPPORTED       - Operation not supported

*/
static GFX_Result ILI9488_layerBufferAllocate(uint32_t idx)
{
    return GFX_UNSUPPORTED;
}

/**
  Function:
    static GFX_Result ILI9488_layerBufferFree(uint32_t idx)

  Summary:
    Driver-specific implementation of GFX HAL layerBufferFree function.

  Description:
    Stub function, operation not supported in driver.

  Returns:
    * GFX_UNSUPPORTED       - Operation not supported

*/
static GFX_Result ILI9488_layerBufferFree(uint32_t idx)
{
    return GFX_UNSUPPORTED;
}

/**
  Function:
    static GFX_Result ILI9488_layerVisibleSet(GFX_Bool val)

  Summary:
    Driver-specific implementation of GFX HAL layerVisibleSet function.

  Description:
    Stub function, operation not supported in driver.


  Returns:
    * GFX_UNSUPPORTED       - Operation not supported

*/
static GFX_Result ILI9488_layerVisibleSet(GFX_Bool val)
{
    return GFX_UNSUPPORTED;
}

/**
  Function:
    static GFX_Result ILI9488_layerAlphaEnableSet(GFX_Bool enable)

  Summary:
    Driver-specific implementation of GFX HAL layerAlphaEnableSet function.

  Description:
     Stub function, operation not supported in driver.


  Returns:

      GFX_UNSUPPORTED     Operation not supported

*/
static GFX_Result ILI9488_layerAlphaEnableSet(GFX_Bool enable, GFX_Bool wait)
{
    return GFX_UNSUPPORTED;
}

/**
  Function:
    GFX_Result ILI9488_ContextInitialize(GFX_Context *context)

  Summary:
    Initializes the driver context.

  Description:
    Initializes the driver context.

  Returns:
    * GFX_SUCCESS       - Operation successful

*/
GFX_Result ILI9488_ContextInitialize(GFX_Context *context)
{
    // set driver-specific data initialization function address
    context->hal.initialize = &ILI9488_Initialize;

    // override essential hal functions
    context->hal.destroy = &ILI9488_Destroy;
    context->hal.update = &ILI9488_Update;

    // set driver-specific function implementations
    context->hal.brightnessRangeGet = &ILI9488_brightnessRangeGet;
    context->hal.brightnessSet = &ILI9488_brightnessSet;
    context->hal.layerVsyncSet = &ILI9488_vsyncSet;
    context->hal.vsyncCallbackSet = &ILI9488_vsyncCallbackSet;
    context->hal.hsyncCallbackSet = &ILI9488_hsyncCallbackSet;
    context->hal.layerActiveSet = &ILI9488_layerActiveSet;
    context->hal.layerEnabledSet = &ILI9488_layerEnabledSet;
    context->hal.layerPositionSet = &ILI9488_layerPositionSet;
    context->hal.layerSizeSet = &ILI9488_layerSizeSet;
    context->hal.layerBufferCountSet = &ILI9488_layerBufferCountSet;
    context->hal.layerBufferAddressSet = &ILI9488_layerBufferAddressSet;
    context->hal.layerBufferCoherentSet = &ILI9488_layerBufferCoherentSet;
    context->hal.layerBufferAllocate = &ILI9488_layerBufferAllocate;
    context->hal.layerBufferFree = &ILI9488_layerBufferFree;
    context->hal.layerVisibleSet = &ILI9488_layerVisibleSet;
    context->hal.layerAlphaEnableSet = &ILI9488_layerAlphaEnableSet;

<#if CONFIG_DRV_GFX_ILI9488_PIXEL_BUFFER == "Frame">
    context->hal.layerSwapSet = &layerSwapSet;
<#else>
    context->hal.drawPipeline[GFX_PIPELINE_GCU].pixelSet = &ILI9488_SetPixel;
    context->hal.drawPipeline[GFX_PIPELINE_GCU].pixelGet = &ILI9488_PixelGet;

    context->hal.drawPipeline[GFX_PIPELINE_GCUGPU].pixelSet = &ILI9488_SetPixel;
    context->hal.drawPipeline[GFX_PIPELINE_GCUGPU].pixelGet = &ILI9488_PixelGet;
</#if>


    return GFX_SUCCESS;
}

/**** End Hardware Abstraction Interfaces ****/
