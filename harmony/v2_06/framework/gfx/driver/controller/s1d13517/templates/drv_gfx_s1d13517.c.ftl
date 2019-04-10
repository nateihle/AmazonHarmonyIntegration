/*******************************************************************************
  Company:
    Microchip Technology Incorporated

  File Name:
    drv_gfx_s1d13517.c

  Summary:
    Main source file for Epson S1D13517 display driver

  Description:
    None
*******************************************************************************/
//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

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
#include "framework/gfx/driver/controller/s1d13517/drv_gfx_s1d13517.h"

#include "gfx/hal/inc/gfx_driver_interface.h"
#include "gfx/hal/inc/gfx_default_impl.h"

#include "framework/driver/pmp/drv_pmp_static.h"

#define MAX_LAYER_COUNT  1
#define MAX_BUFFER_COUNT 2

<#if CONFIG_DRV_GFX_S1D13517_COLOR_MODE == "RGB_565">
static uint32_t supportedColorModes = GFX_COLOR_MASK_RGB_565;
<#elseif CONFIG_DRV_GFX_S1D13517_COLOR_MODE == "RGB_888">
static uint32_t supportedColorModes = GFX_COLOR_MASK_RGB_888;
</#if>

const char* DRIVER_NAME = "S1D13517";


#define GetSystemClock()    (40000000ULL) // todo read from system_config.h

static void TMR_DelayMS( uint32_t delay_in_ms )
{
    uint32_t tWait = ( GetSystemClock() / 2000 ) * delay_in_ms;
    uint32_t tStart = _CP0_GET_COUNT();
    while( ( _CP0_GET_COUNT() - tStart ) < tWait );
}

#define BUFFER_SIZE 0x140000l  /*WVGA used for ALL LCDs*/

uint32_t drvS1D13517_SetRegister(uint8_t index, uint8_t value)
{
    BSP_DisplaySetOff(); // set RS line to low for command

    DRV_PMP0_Write(index);

    BSP_DisplaySetOn();    // set RS line to high for data

    DRV_PMP0_Write(value);

    return 0;
}

uint8_t drvS1D13517_GetRegister(uint8_t  index)
{
     uint16_t myReadBuffer = 0xff;

     BSP_DisplaySetOff(); // set RS line to low for command

     DRV_PMP0_Write(index);

     BSP_DisplaySetOn(); // set RS line to high for data

     DRV_PMP0_Read(); //Perform the fake read

     PMCONbits.PMPEN = 0; // disable PMP
     myReadBuffer = DRV_PMP0_Read();
     PMCONbits.PMPEN = 1; // disable PMP

    return (uint8_t)myReadBuffer;
}

static void setBrightness(uint16_t  level)
{
   if(level == 100)
   {
       while(drvS1D13517_SetRegister(S1D13517_REG70_PWM_CONTROL,0x85));          //Turn on Backlight
   }
   else if (level == 0)
   {
       drvS1D13517_SetRegister(S1D13517_REG70_PWM_CONTROL,0x84);       //Turn off Backlight
   }
   else if (level <= 50)
   {
       level >>= 1;
       level *=  5;  /*Sets the value from (0-250)*/

       drvS1D13517_SetRegister(S1D13517_REG72_PWM_HIGH_DC_0,0xff);
       drvS1D13517_SetRegister(S1D13517_REG74_PWM_HIGH_DC_1,level);
       drvS1D13517_SetRegister(S1D13517_REG7A_PWM_LOW_DC_0,0xff);
       drvS1D13517_SetRegister(S1D13517_REG7C_PWM_LOW_DC_1,0xff);
       drvS1D13517_SetRegister(S1D13517_REG70_PWM_CONTROL,0x86);   //Turn off Backlight PWM
   }
   else
   {
       level >>= 1;
       level *=  5;  /*Sets the value from (0-250)*/

       drvS1D13517_SetRegister(S1D13517_REG72_PWM_HIGH_DC_0,level);
       drvS1D13517_SetRegister(S1D13517_REG74_PWM_HIGH_DC_1,0xff);
       drvS1D13517_SetRegister(S1D13517_REG7A_PWM_LOW_DC_0,0xff);
       drvS1D13517_SetRegister(S1D13517_REG7C_PWM_LOW_DC_1,0xff);
       drvS1D13517_SetRegister(S1D13517_REG70_PWM_CONTROL,0x86);   //Turn off Backlight PWM

    }
}

static GFX_Result initialize(GFX_Context* context)
{
    uint8_t state = 0;

    uint16_t horizontalSize = context->display_info->rect.width;
    uint16_t verticalSize = context->display_info->rect.height;
    uint16_t horizontalPulse = context->display_info->attributes.horz.pulse_width;
    uint16_t horizontalFrontPorch = context->display_info->attributes.horz.front_porch + 1;
    uint16_t verticalPulse = context->display_info->attributes.vert.pulse_width;
    uint16_t verticalFrontPorch = context->display_info->attributes.vert.front_porch;
    int16_t invLShift = context->display_info->attributes.inv_left_shift;

    uint32_t i;

    // initialize all layers
    for(i = 0; i < context->layer.count; i++)
    {
        context->layer.layers[i].enabled = GFX_TRUE;
        context->layer.layers[i].visible = GFX_TRUE;

        context->layer.layers[i].vsync = GFX_FALSE;
        context->layer.layers[i].swap = GFX_FALSE;

        context->layer.layers[i].rect.local.x = 0;
        context->layer.layers[i].rect.local.y = 0;
        context->layer.layers[i].rect.local.width = context->display_info->rect.width;
        context->layer.layers[i].rect.local.height = context->display_info->rect.height;

        context->layer.layers[i].rect.display = context->layer.layers[i].rect.local;

        context->layer.layers[i].alphaEnable = GFX_FALSE;
        context->layer.layers[i].alphaAmount = 255;

        context->layer.layers[i].maskEnable = GFX_FALSE;
        context->layer.layers[i].maskColor = 0;

        context->layer.layers[i].buffer_count = 1;
        context->layer.layers[i].buffer_read_idx = 0;
        context->layer.layers[i].buffer_write_idx = 0;

        GFX_PixelBufferCreate(context->display_info->rect.width,
                              context->display_info->rect.height,
                              GFX_COLOR_MODE_RGB_565,
                              NULL,
                              &context->layer.layers[i].buffers[0].pb);

        context->layer.layers[i].buffers[0].state = GFX_BS_MANAGED;
    }

    uint16_t horizontalTiming = context->display_info->attributes.horz.pulse_width +
                                context->display_info->attributes.horz.front_porch +
                                context->display_info->attributes.horz.back_porch;

    uint16_t verticalTiming = context->display_info->attributes.vert.pulse_width +
                              context->display_info->attributes.vert.front_porch +
                              context->display_info->attributes.vert.back_porch;

    BSP_DisplayResetOff();

    while(state <= 40)
    {
        switch(state)
        {
            case 0:
                BSP_DisplayResetOn();
                BSP_DisplayOff();

                state++;
                break;

            case 11:
                TMR_DelayMS(100);
                break;

        }

        /*S1D13517 Registers to be initialized*/
        uint8_t registers[40][2] =
        {
            { S1D13517_REG2A_DSP_MODE,         0x00 },
            { S1D13517_REG68_POWER_SAVE,       0x00 },
            { S1D13517_REG04_PLL_DDIVIDER,     0x17 },
            { S1D13517_REG06_PLL_0,            0x29 },
            { S1D13517_REG08_PLL_1,            0x01 },
            { S1D13517_REG0A_PLL_2,            0x08 },
            { S1D13517_REG0C_PLL_NDIVIDER,     0x59 },
#if (DISP_HOR_RESOLUTION < 481)
            { S1D13517_REG12_CLK_SRC_SELECT,   0x02 },
#else
            { S1D13517_REG12_CLK_SRC_SELECT,   0x92 },
#endif
            { S1D13517_REG04_PLL_DDIVIDER,     0x97 },
            { S1D13517_REG0E_SS_CONTROL_0,     0x8F },
<#if     CONFIG_DRV_GFX_S1D13517_COLOR_MODE == "RGB_888">
            { S1D13517_REG14_LCD_PANEL_TYPE,   0x02 },
<#elseif CONFIG_DRV_GFX_S1D13517_COLOR_MODE == "RGB_565">
            { S1D13517_REG14_LCD_PANEL_TYPE,   0x05 },
</#if>
            { S1D13517_REG16_HDISP_WIDTH,      (horizontalSize >> 3) - 1 },
            { S1D13517_REG18_HNDP_PERIOD,      ((horizontalTiming)>>1)-1 },
            { S1D13517_REG1A_VDISP_HEIGHT_0,   verticalSize-1 },
            { S1D13517_REG1C_VDISP_HEIGHT_1,   (verticalSize-1)>>8 },
            { S1D13517_REG1E_VNDP_PERIOD,      ((verticalTiming) >> 1) - 1 },
            { S1D13517_REG20_PHS_PULSE_WIDTH,  horizontalPulse - 1 },
            { S1D13517_REG22_PHS_PULSE_START,  horizontalFrontPorch },
            { S1D13517_REG24_PVS_PULSE_WIDTH,  verticalPulse - 1 },
            { S1D13517_REG26_PVS_PULSE_START,  verticalFrontPorch },
            { S1D13517_REG28_PCLK_POLARITY,    (invLShift << 7) },
            { S1D13517_REG82_SDRAM_CONTROL_0,  0x03 },
            { S1D13517_REG8C_SDRAM_RFS_CNT_0,  0xFF },
            { S1D13517_REG8E_SDRAM_RFS_CNT_1,  0x03 },
            { S1D13517_REG90_SDRAM_BUF_SIZE,   0x50 },
            { S1D13517_REG68_POWER_SAVE,       0xE8 },
            { S1D13517_REG68_POWER_SAVE,       0x00 },
            { S1D13517_REG68_POWER_SAVE,       0x01 },
            { S1D13517_REG84_SDRAM_STATUS_0,   0x86 },
            { S1D13517_REG52_INPUT_MODE,       0x08 },
<#if CONFIG_GFX_HAL_DRAW_PIPELINE_ENABLED>
<#if     CONFIG_DRV_GFX_S1D13517_COLOR_MODE == "RGB_888">
            { S1D13517_REG54_TRANSP_KEY_RED,   GFX_ColorChannelRed(S1D13517_MASK_COLOR_1, GFX_COLOR_MODE_RGB_888)} ,
            { S1D13517_REG56_TRANSP_KEY_GREEN, GFX_ColorChannelGreen(S1D13517_MASK_COLOR_1, GFX_COLOR_MODE_RGB_888) },
            { S1D13517_REG58_TRANSP_KEY_BLUE,  GFX_ColorChannelBlue(S1D13517_MASK_COLOR_1, GFX_COLOR_MODE_RGB_888)},
<#elseif CONFIG_DRV_GFX_S1D13517_COLOR_MODE == "RGB_565">
            { S1D13517_REG54_TRANSP_KEY_RED,   GFX_ColorChannelRed(S1D13517_MASK_COLOR_1, GFX_COLOR_MODE_RGB_565) << 3 },
            { S1D13517_REG56_TRANSP_KEY_GREEN, GFX_ColorChannelGreen(S1D13517_MASK_COLOR_1, GFX_COLOR_MODE_RGB_565) << 2 },
            { S1D13517_REG58_TRANSP_KEY_BLUE,  GFX_ColorChannelBlue(S1D13517_MASK_COLOR_1, GFX_COLOR_MODE_RGB_565) << 3 },
</#if>
</#if>
            { S1D13517_REG6E_GPO_1,            0x07 },
            { S1D13517_REG2A_DSP_MODE,         0x01 },
            { S1D13517_REG50_DISPLAY_CONTROL,  0x80 },
            { S1D13517_REGB2_INTERRUPT_CTRL,   0x01 },
            { S1D13517_REG9E_ALP_VALUE,        0x80 },
            { S1D13517_REG94_ALP_CONTROL,      0x01 },
            { S1D13517_REG94_ALP_CONTROL,      0x00 }
        };

        while(drvS1D13517_SetRegister(*registers[state-1], registers[state-1][1]));

        state++;
    }

    TMR_DelayMS(500);

    setBrightness(100);

    return GFX_SUCCESS;
}

static void destroy(GFX_Context* context)
{
    // driver specific shutdown tasks
    if(context->driver_data != GFX_NULL)
    {
        context->memory.free(context->driver_data);
        context->driver_data = GFX_NULL;
    }

    // general default shutdown
    defDestroy(context);
}


static GFX_Result brightnessRangeGet(uint32_t* low, uint32_t* high)
{
    return GFX_UNSUPPORTED;
}

static GFX_Result brightnessSet(uint32_t val)
{
    return GFX_UNSUPPORTED;
}

static GFX_Result vsyncSet(GFX_Bool enable)
{
    return GFX_UNSUPPORTED;
}

static GFX_Result vsyncCallbackSet(GFX_SyncCallback_FnPtr cb)
{
    return GFX_UNSUPPORTED;
}

static GFX_Result hsyncCallbackSet(GFX_SyncCallback_FnPtr cb)
{
    return GFX_UNSUPPORTED;
}

/*static GFX_Result orientationSet(GFX_Orientation ori)
{
    return GFX_UNSUPPORTED;
}

static GFX_Result mirroringSet(GFX_Bool mirr)
{
    return GFX_UNSUPPORTED;
}*/

static GFX_Result layerActiveSet(uint32_t idx)
{
    return GFX_UNSUPPORTED;
}

static GFX_Result layerEnabledSet(GFX_Bool val)
{
    // layers are always enabled
    /*if(val == GFX_FALSE)
        return GFX_FAILURE;

    return GFX_SUCCESS;*/
    return GFX_UNSUPPORTED;
}

static GFX_Result layerPositionSet(int32_t x, int32_t y)
{
    return GFX_UNSUPPORTED;
}

static GFX_Result layerSizeSet(int32_t width, int32_t height)
{
    return GFX_UNSUPPORTED;
}

static GFX_Result layerBufferCountSet(uint32_t count)
{
    GFX_Layer* layer;
    uint32_t i;

    layer = GFX_ActiveContext()->layer.active;

    // driver max is 2 for now
    if(count > MAX_BUFFER_COUNT)
        return GFX_FAILURE;

    // use default implementation to initialize buffer struct
    defLayerBufferCountSet(count);

    // ensure all buffers are marked as managed by the driver so application
    // can't delete or modify them
    for(i = 0; i < layer->buffer_count; i++)
        layer->buffers[i].state = GFX_BS_MANAGED;

    return GFX_SUCCESS;
}

static GFX_Result layerBufferAddressSet(uint32_t idx, GFX_Buffer address)
{
    return GFX_UNSUPPORTED;
}

static GFX_Result layerBufferCoherentSet(uint32_t idx, GFX_Bool coherent)
{
    return GFX_UNSUPPORTED;
}

static GFX_Result layerBufferAllocate(uint32_t idx)
{
    return GFX_UNSUPPORTED;
}

static GFX_Result layerBufferFree(uint32_t idx)
{
    return GFX_UNSUPPORTED;
}

static GFX_Result layerVisibleSet(GFX_Bool val)
{
    // layers are always visible
    if(val == GFX_FALSE)
        return GFX_FAILURE;

    return GFX_SUCCESS;
}

void layerSwapped(GFX_Layer* layer)
{
    // can only swap layer 0 with this hardware
    if(layer->id == GFX_ActiveContext()->layer.layers[0].id)
    {
        // set new write buffer
        while(drvS1D13517_SetRegister(S1D13517_REG52_INPUT_MODE, 0x08 | (layer->buffer_write_idx << 4))); // 0x8 preserves transparency flag

        // set new read buffer
        while(drvS1D13517_SetRegister(S1D13517_REG2A_DSP_MODE, ((0x1) | (layer->buffer_read_idx << 4)))); // 0x1 is display enable bit (should always stay on))
        while(drvS1D13517_SetRegister(S1D13517_REG50_DISPLAY_CONTROL, 0x80));
    }
}

static GFX_Result layerAlphaEnableSet(GFX_Bool enable, GFX_Bool wait)
{
    return GFX_UNSUPPORTED;
}

<#if CONFIG_GFX_HAL_DRAW_PIPELINE_ENABLED>
static GFX_Color pixelGet(const GFX_PixelBuffer* buf,
                          const GFX_Point* pnt)
{
    return 0;
}
</#if>

// function that returns the information for this driver
GFX_Result driverS1D13517InfoGet(GFX_DriverInfo* info)
{
    if(info == GFX_NULL)
        return GFX_FAILURE;

    // populate info struct
    strcpy(info->name, DRIVER_NAME);
    info->color_formats = supportedColorModes;
    info->layer_count = MAX_LAYER_COUNT;

    return GFX_SUCCESS;
}

// function that initialized the driver context
GFX_Result driverS1D13517ContextInitialize(GFX_Context* context)
{
    struct S1D13517DriverData* data;

    // set driver-specific function implementations
    context->hal.initialize = &initialize;
    context->hal.destroy = &destroy;
    context->hal.brightnessRangeGet = &brightnessRangeGet;
    context->hal.brightnessSet = &brightnessSet;
    context->hal.layerVsyncSet = &vsyncSet;
    context->hal.vsyncCallbackSet = &vsyncCallbackSet;
    context->hal.hsyncCallbackSet = &hsyncCallbackSet;
    //context->hal.orientationSet = &orientationSet;
    //context->hal.mirroringSet = &mirroringSet;
    context->hal.layerActiveSet = &layerActiveSet;
    context->hal.layerEnabledSet = &layerEnabledSet;
    context->hal.layerPositionSet = &layerPositionSet;
    context->hal.layerSizeSet = &layerSizeSet;
    context->hal.layerBufferCountSet = &layerBufferCountSet;
    context->hal.layerBufferAddressSet = &layerBufferAddressSet;
    context->hal.layerBufferCoherentSet = &layerBufferCoherentSet;
    context->hal.layerBufferAllocate = &layerBufferAllocate;
    context->hal.layerBufferFree = &layerBufferFree;
    context->hal.layerVisibleSet = &layerVisibleSet;
    context->hal.layerSwapped = &layerSwapped;
    context->hal.layerAlphaEnableSet = &layerAlphaEnableSet;

    <#if CONFIG_GFX_HAL_DRAW_PIPELINE_ENABLED>
    context->hal.drawPipeline[GFX_PIPELINE_GCU].pixelSet = &drvS1D13517_SetPixel;
    context->hal.drawPipeline[GFX_PIPELINE_GCU].pixelGet = &pixelGet;
    context->hal.drawPipeline[GFX_PIPELINE_GCU].drawLine[GFX_DRAW_LINE][GFX_ANTIALIAS_OFF] = &drvS1D13517_DrawLine;
    context->hal.drawPipeline[GFX_PIPELINE_GCU].drawRect[GFX_DRAW_FILL][GFX_ANTIALIAS_OFF] = &drvS1D13517_FillRect;

    context->hal.drawPipeline[GFX_PIPELINE_GCUGPU].pixelSet = &drvS1D13517_SetPixel;
    context->hal.drawPipeline[GFX_PIPELINE_GCUGPU].pixelGet = &pixelGet;
    context->hal.drawPipeline[GFX_PIPELINE_GCUGPU].drawLine[GFX_DRAW_LINE][GFX_ANTIALIAS_OFF] = &drvS1D13517_DrawLine;
    context->hal.drawPipeline[GFX_PIPELINE_GCUGPU].drawRect[GFX_DRAW_FILL][GFX_ANTIALIAS_OFF] = &drvS1D13517_FillRect;
    </#if>
    // allocate space for driver specific data
    data = context->memory.calloc(1, sizeof(struct S1D13517DriverData));

    if(data == GFX_NULL)
        return GFX_FAILURE;

    context->driver_data = data;

    return GFX_SUCCESS;
}


<#if CONFIG_USE_SEGGER_EMWIN_LIBRARY>

    /*********************************************************************
    *
    *       _Write16_A0
    */
     void drvS1D13517emWin_Write16A0(uint16_t Data) {
      BSP_DisplaySetOff();
      DRV_PMP0_Write(Data);
      BSP_DisplaySetOn();
    }

    /*********************************************************************
    *
    *       _Write16_A1
    */
     void drvS1D13517emWin_Write16A1(uint16_t Data) {
      DRV_PMP0_Write(Data);
    }

    /*********************************************************************
    *
    *       _WriteM16_A1
    */
     void drvS1D13517emWin_WriteM16_A1(uint16_t * pData, int NumItems) {
      do {
        DRV_PMP0_Write(*pData++);
      } while (--NumItems);
    }

    void drvS1D13517emWin_SetCS(uint8_t State) {
      if (State == 0) {
        BSP_DisplayOff();
      } else {
        BSP_DisplayOn();
      }
    }

</#if>