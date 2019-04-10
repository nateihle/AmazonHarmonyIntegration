/*******************************************************************************
  Company:
    Microchip Technology Incorporated

  File Name:
    drv_gfx_ssd1926.c

  Summary:
    Main source file for SSD1926 display driver

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
#include "framework/gfx/driver/controller/ssd1926/drv_gfx_ssd1926.h"

#include "gfx/hal/inc/gfx_driver_interface.h"
#include "gfx/hal/inc/gfx_default_impl.h"

#include "framework/driver/pmp/drv_pmp_static.h"

<#if CONFIG_DRV_GFX_TCON_ENABLE == true>
<#if CONFIG_DRV_GFV_TCON == "SSD1289">
void GFX_TCON_SSD1289Init(void);
</#if>
</#if>

#define MAX_LAYER_COUNT  1
#define MAX_BUFFER_COUNT 1

const char* DRIVER_NAME = "SSD1926";

static uint32_t supportedColorModes = GFX_COLOR_MASK_RGB_565;

#define GetSystemClock()    (40000000ULL) // todo read from system_config.h

static void TMR_DelayMS( uint32_t delay_in_ms )
{
    uint32_t tWait = ( GetSystemClock() / 2000 ) * delay_in_ms;
    uint32_t tStart = _CP0_GET_COUNT();
    while( ( _CP0_GET_COUNT() - tStart ) < tWait );
}

typedef struct
{
   uint32_t  address;
   uint16_t* array;
   uint16_t  data;
} command;

<#if CONFIG_DRV_PMP_DATA_SIZE == "PMP_DATA_SIZE_16_BITS">
// 16 bit PMP
#define dataWidth PMP_DATA_SIZE_16_BITS

static void PMP_WriteShort(uint16_t val)
{
    DRV_PMP0_Write(val);
}

static void PMP_WriteWord(uint32_t val)
{
    DRV_PMP0_Write(val >> 16);
    DRV_PMP0_Write(val);
}
<#elseif CONFIG_DRV_PMP_DATA_SIZE == "PMP_DATA_SIZE_8_BITS">
// 8 bit PMP
#define dataWidth PMP_DATA_SIZE_8_BITS

static void PMP_WriteShort(uint16_t val)
{
    DRV_PMP0_Write((uint8_t)(val >> 8));
    DRV_PMP0_Write((uint8_t)val);
}

static void PMP_WriteWord(uint32_t val)
{
    DRV_PMP0_Write(val >> 24);
    DRV_PMP0_Write(val >> 16);
    DRV_PMP0_Write(val >> 8);
    DRV_PMP0_Write(val);
}
</#if>

uint8_t drvSSD1926_GetRegister(uint16_t index, uint8_t* data)
{
    uint16_t myReadBuffer = 0xff;

    BSP_DisplaySetOff(); // Set Address Line Low

<#if CONFIG_DRV_PMP_DATA_SIZE == "PMP_DATA_SIZE_8_BITS">
    DRV_PMP0_Write(0x00);
    DRV_PMP0_Write(index >> 8);
    DRV_PMP0_Write(index);
<#elseif CONFIG_DRV_PMP_DATA_SIZE == "PMP_DATA_SIZE_16_BITS">
    DRV_PMP0_Write(index >> 8);
    DRV_PMP0_Write(index << 8);
</#if>

    BSP_DisplaySetOn(); //Set Address Line Low

    myReadBuffer = DRV_PMP0_Read();
    DRV_PMP0_Read();

    PMCONbits.PMPEN = 0; // disable PMP
    myReadBuffer = DRV_PMP0_Read();
    PMCONbits.PMPEN = 1; // disable PMP

    myReadBuffer >>= (dataWidth * 8);
    *data = (uint8_t)myReadBuffer;

    return(0);
}

uint32_t drvSSD1926_SetRegister(uint16_t index, uint8_t value)
{
    uint32_t myWriteBuffer;

    BSP_DisplaySetOff(); //Set Address Line Low

<#if CONFIG_DRV_PMP_DATA_SIZE == "PMP_DATA_SIZE_8_BITS">
    DRV_PMP0_Write(0x00);
    DRV_PMP0_Write(index >> 8);
    DRV_PMP0_Write(index);
<#elseif CONFIG_DRV_PMP_DATA_SIZE == "PMP_DATA_SIZE_16_BITS">
    DRV_PMP0_Write(index >> 8);
    DRV_PMP0_Write(index << 8);
</#if>

    BSP_DisplaySetOn(); //Set Address Line Low

    myWriteBuffer = (value << (dataWidth*8*(!(index & 0x0001))));

    DRV_PMP0_Write(myWriteBuffer);

    return(0);
}

static uint16_t setWriteAddress(uint32_t address)
{
    uint32_t temp;

    temp = (address << 8) + 1;
    temp |= (0x80000000);

    BSP_DisplaySetOff(); //Set Address Line Low

    PMP_WriteWord(temp);

    BSP_DisplaySetOn(); //Set Address Line Low

    return(0);
}

static GFX_Result initialize(GFX_Context* context)
{
    uint8_t state = 0;
    uint8_t dummy = 1;
    uint16_t rotation, horizontalTotal, verticalTotal;
    uint16_t horzFrontPorch, horzBackPorch, horzPulseWidth, horzWidth;
    uint16_t vertFrontPorch, vertBackPorch, vertPulseWidth, vertHeight;
    uint8_t panelType = 0;
    uint8_t panelWidth = 0;
    uint32_t windowStartAddress = 0, windowOffset = 0;
    uint32_t i;

    //BSP_DisplayBacklightOff();

    rotation = 0;

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

    horzFrontPorch = context->display_info->attributes.horz.front_porch;
    horzBackPorch = context->display_info->attributes.horz.back_porch;
    horzPulseWidth = context->display_info->attributes.horz.pulse_width;
    horzWidth = context->display_info->rect.width;

    vertFrontPorch = context->display_info->attributes.vert.front_porch;
    vertBackPorch = context->display_info->attributes.vert.back_porch;
    vertPulseWidth = context->display_info->attributes.vert.pulse_width;
    vertHeight = context->display_info->rect.height;

    horizontalTotal = horzPulseWidth + horzBackPorch +
                      horzFrontPorch + horzWidth;

    verticalTotal = vertPulseWidth + vertBackPorch +
                    vertFrontPorch + vertHeight;


    while(state <= 45)
    {
        switch(state)
        {

            case 0:
            {
                BSP_DisplayResetOff(); // enable chip select line
                BSP_DisplayResetOn();
                BSP_DisplayOff();

                /*time for the controller to power up*/
                TMR_DelayMS(500);

                state++;

                panelWidth |= ((context->display_info->attributes.data_width >> 3)<<4);

                //if (LCD_TYPE == LCD_TFT)
                //{
                    panelType |= 0x41;
                /*}
                else if (LCD_TYPE == LCD_CSTN2)
                {
                    panelType |= 0x40;
                }*/

                // old orientation code
                /*switch(orientation)
                {
                    case 90:
                        windowStartAddress = ((((uint32_t) horizontalSize + 1) >> 1) - 1);
                        rotation = 1;
                        break;

                    case 180:
                        windowStartAddress = (((((uint32_t) horizontalSize + 1) * (verticalSize + 1)) >> 1) - 1);
                        rotation = 2;
                        break;

                    case 270:
                        windowStartAddress = ((((uint32_t) horizontalSize + 1) * verticalSize) >> 1);
                        rotation = 3;
                        break;

                    default:
                        windowStartAddress = 0ul;
                        rotation = 0;
                        break;

                }*/

                windowOffset = (horzWidth + 1) >> 1;

                break;
            }
            case 33:
            {
                while(dummy)
                    while(drvSSD1926_GetRegister(SSD1926_REG_POWER_SAVE_CONFIG, &dummy));

                state++;
                state++;
            }
            default:
            {
                break;
            }
        }

        /*SSD1926 Registers to be initialized*/
        uint16_t registers[44][2] =
        {
            { SSD1926_REG_PLL_CONFIG_0,              0x0a },
            { SSD1926_REG_PLL_CONFIG_1,              0xc8 },
            { SSD1926_REG_PLL_CONFIG_2,              0xae },
            { SSD1926_REG_PLL_CONFIG_0,              0x8a },
            { SSD1926_REG_MEMCLK_CONFIG,             0 },
            { SSD1926_REG_PCLK_FREQ_RATIO_0,         0 },
            { SSD1926_REG_PCLK_FREQ_RATIO_1,         0 },
            { SSD1926_REG_PCLK_FREQ_RATIO_2,         0x02 },
            { SSD1926_REG_PANEL_TYPE,                panelType|panelWidth },
            { SSD1926_REG_HORIZ_TOTAL_0,             horizontalTotal >> 3 },
            { SSD1926_REG_HORIZ_TOTAL_1,             horizontalTotal % 8 },
            { SSD1926_REG_HDP,                       (horzWidth >> 3) - 1 },
            { SSD1926_REG_HDP_START_POS0,            horzPulseWidth + horzBackPorch },
            { SSD1926_REG_HDP_START_POS1,            (horzPulseWidth + horzBackPorch) >> 8 },
            { SSD1926_REG_HSYNC_PULSE_WIDTH,         horzPulseWidth - 1 },
            { SSD1926_REG_VERT_TOTAL0,               verticalTotal },
            { SSD1926_REG_VERT_TOTAL1,               verticalTotal >> 8 },
            { SSD1926_REG_VDP0,                      vertHeight - 1 },
            { SSD1926_REG_VDP1,                      (vertHeight - 1) >> 8 },
            { SSD1926_REG_VDP_START_POS0,            (vertPulseWidth + vertBackPorch) },
            { SSD1926_REG_VDP_START_POS1,            (vertPulseWidth + vertBackPorch) >> 8 },
            { SSD1926_REG_VSYNC_PULSE_WIDTH,         vertPulseWidth - 1 },
            { SSD1926_REG_SPECIAL_EFFECTS,           0x40 | rotation },
            { SSD1926_REG_MAIN_WIN_DISP_START_ADDR0, windowStartAddress  },
            { SSD1926_REG_MAIN_WIN_DISP_START_ADDR1, windowStartAddress >> 8  },
            { SSD1926_REG_MAIN_WIN_DISP_START_ADDR2, windowStartAddress >> 16 },
            { SSD1926_REG_MAIN_WIN_ADDR_OFFSET0,     windowOffset },
            { SSD1926_REG_MAIN_WIN_ADDR_OFFSET1,     windowOffset >> 8 },
            { SSD1926_REG_DISPLAY_MODE,              0x04  },
            { SSD1926_REG_RGB_SETTING,               0xc0  },
            { SSD1926_REG_LSHIFT_POLARITY,           context->display_info->attributes.inv_left_shift },
            { SSD1926_REG_POWER_SAVE_CONFIG,         0  },
            { 0,                                     0 },
            { 0,                                     0  },
            { SSD1926_REG_GPIO_STATUS_CONTROL1,      0x80  },
            { SSD1926_REG_2D_1f8,                    horzWidth },
            { SSD1926_REG_2D_1f9,                    horzWidth >> 8 },
            { SSD1926_REG_2D_1d8,                    horzWidth },
            { SSD1926_REG_2D_1d9,                    horzWidth >> 8 },
            { SSD1926_REG_2D_214,                    1 },
            { SSD1926_REG_2D_215,                    0 },
            { SSD1926_REG_2D_218,                    1 },
            { SSD1926_REG_2D_219,                    0 },
            { SSD1926_REG_2D_1dd,                    0 }
        };

        while(drvSSD1926_SetRegister(*registers[state-1], registers[state-1][1]));

        state++;
    }

<#if CONFIG_DRV_GFX_TCON_ENABLE == true>
<#if CONFIG_DRV_GFV_TCON == "SSD1289">
    // initialize SSD1289 timing controller
    GFX_TCON_SSD1289Init();
</#if>
</#if>

    //BSP_DisplayBacklightOn();

    return GFX_SUCCESS;
}

static GFX_Result pixelSet(const GFX_PixelBuffer* buf,
                           const GFX_Point* pnt,
                           GFX_Color color)
{
    uint32_t address = ((pnt->y * buf->size.width) + pnt->x) *
                       GFX_ColorInfo[buf->mode].size;

    while(setWriteAddress(address));

    PMP_WriteShort(color);

    return GFX_SUCCESS;
}

static uint16_t deviceBusy()
{
    uint8_t status = 0x00;

    if(drvSSD1926_GetRegister(SSD1926_REG_2D_220,(uint8_t*)&status))
        return(1);

    return((bool)(!status));
}

GFX_Result drawLine(const GFX_Point* p1,
                    const GFX_Point* p2,
                    const GFX_DrawState* state)
{
    uint8_t st = 0;
    GFX_Point tfpt1, tfpt2, tfptt;
    GFX_Context* context = GFX_ActiveContext();

    uint16_t lineParams[16][2] =
    {
        {SSD1926_REG_2D_1e4, 0}, // source width low
        {SSD1926_REG_2D_1e5, 0}, // source width high
        {SSD1926_REG_2D_1e8, 0}, // source height low
        {SSD1926_REG_2D_1e9, 0}, // source height high
        {SSD1926_REG_2D_1ec, 0}, // dest width low
        {SSD1926_REG_2D_1ed, 0}, // dest width high
        {SSD1926_REG_2D_1f0, 0}, // dest height low
        {SSD1926_REG_2D_1f1, 0}, // dest height high
        {SSD1926_REG_2D_1f4, 0}, // brush address
        {SSD1926_REG_2D_1f5, 0},
        {SSD1926_REG_2D_1f6, 0},
        {SSD1926_REG_2D_1fe, 0x09}, // red channel
        {SSD1926_REG_2D_1fd, 0x01}, // green channel
        {SSD1926_REG_2D_1fc, 0xf0}, // blue channel
        {SSD1926_REG_2D_1d1, 0x01}, // line draw
        {SSD1926_REG_2D_1d2, 0x01} // auto mode on
    }; // line 2d Commands

    tfpt1 = GFX_LayerPointFromOrientedSpace(context->layer.active,
                                            p1,
                                            context->orientation,
                                            context->mirrored);

    tfpt2 = GFX_LayerPointFromOrientedSpace(context->layer.active,
                                            p2,
                                            context->orientation,
                                            context->mirrored);

    // hardware seems to prefer the topmost point be sent first
    if(tfpt1.y > tfpt2.y)
    {
        tfptt = tfpt1;
        tfpt1 = tfpt2;
        tfpt2 = tfptt;
    }

    /* Line Boundaries */
    lineParams[0][1] =  (uint8_t)tfpt1.x;
    lineParams[1][1] =  (uint8_t)(tfpt1.x >> 8);
    lineParams[2][1] =  (uint8_t)tfpt1.y;
    lineParams[3][1] =  (uint8_t)(tfpt1.y >> 8);
    lineParams[4][1] =  (uint8_t)tfpt2.x;
    lineParams[5][1] =  (uint8_t)(tfpt2.x >> 8);
    lineParams[6][1] =  (uint8_t)tfpt2.y;
    lineParams[7][1] =  (uint8_t)(tfpt2.y >> 8);

    /* Set Color */
    lineParams[11][1] = GFX_ColorChannelRed(state->color, GFX_COLOR_MODE_RGB_565) << 3;
    lineParams[12][1] = GFX_ColorChannelGreen(state->color, GFX_COLOR_MODE_RGB_565) << 2;
    lineParams[13][1] = GFX_ColorChannelBlue(state->color, GFX_COLOR_MODE_RGB_565) << 3;

    while(1)
    {
        while(drvSSD1926_SetRegister(lineParams[st-1][0],
                                     (uint8_t)lineParams[st-1][1]));

        st++;

        if(st == 17)
        {
            st = 0;

            while(deviceBusy() == 1);

            return(GFX_SUCCESS);
        }
    }

    return(GFX_SUCCESS);
}

GFX_Result fillRect(const GFX_Rect* rect,
                    const GFX_DrawState* state)
{
   GFX_Context* context = GFX_ActiveContext();

   GFX_Rect tf_Rect;

   uint32_t address;
   uint8_t st = 0;

   tf_Rect = GFX_LayerRectFromOrientedSpace(context->layer.active,
                                            rect,
                                            context->orientation,
                                            context->mirrored);

   uint16_t barParams[13][2] = {{SSD1926_REG_2D_1f4,0},{SSD1926_REG_2D_1f5, 0},{SSD1926_REG_2D_1f6,0}, //Destination Address
                                {SSD1926_REG_2D_204,0},{SSD1926_REG_2D_205,0},{SSD1926_REG_2D_206,0},  //Brush Address
                                {SSD1926_REG_2D_1e4,0},{SSD1926_REG_2D_1e5,0},{SSD1926_REG_2D_1e8,0},{SSD1926_REG_2D_1e9,0}, //Width and Height
                                {SSD1926_REG_2D_1fc,0xf0},{SSD1926_REG_2D_1d1,0x09},{SSD1926_REG_2D_1d2,0x01}}; //Bar 2d Commands

    switch(st)
    {
        case 0:
            //width = right - left + 1;
            barParams[6][1] = tf_Rect.width;
            barParams[7][1] = tf_Rect.width >> 8;

            //height = bottom - top + 1;
            barParams[8][1] = tf_Rect.height;
            barParams[9][1] = (tf_Rect.height >> 8);


            address = (uint32_t)(((context->display_info->rect.width) * tf_Rect.y) + tf_Rect.x);

            barParams[0][1] = address;
            barParams[1][1] = (address >> 8);
            barParams[2][1] = (address >> 16);

            barParams[3][1] = barParams[0][1];
            barParams[4][1] = barParams[1][1];
            barParams[5][1] = barParams[2][1];

            st++;

        case 1:
            while(setWriteAddress(address << 1));
            st++;

        case 2:

            PMP_WriteShort(state->color);
            st++;

        default:
            break;
    }

    while(1)
    {
        while(drvSSD1926_SetRegister(barParams[st-3][0], (uint8_t)barParams[st-3][1]));

        if(st++ == 15)
        {
            st = 0;
            while(deviceBusy(0) == 1);
            return(GFX_SUCCESS);
        }
    }

    return(GFX_SUCCESS);
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

static GFX_Result layerActiveSet(uint32_t idx)
{
    return GFX_UNSUPPORTED;
}

static GFX_Result layerEnabledSet(GFX_Bool val)
{
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
    return GFX_UNSUPPORTED;
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
    return GFX_UNSUPPORTED;
}

static GFX_Result layerAlphaEnableSet(GFX_Bool enable, GFX_Bool wait)
{
    return GFX_UNSUPPORTED;
}

static GFX_Color pixelGet(const GFX_PixelBuffer* buf,
                          const GFX_Point* pnt)
{
    return 0;
}

// function that returns the information for this driver
GFX_Result driverSSD1926InfoGet(GFX_DriverInfo* info)
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
GFX_Result driverSSD1926ContextInitialize(GFX_Context* context)
{
    // set driver-specific function implementations
    context->hal.initialize = &initialize;
    context->hal.destroy = &destroy;
    context->hal.brightnessRangeGet = &brightnessRangeGet;
    context->hal.brightnessSet = &brightnessSet;
    context->hal.layerVsyncSet = &vsyncSet;
    context->hal.vsyncCallbackSet = &vsyncCallbackSet;
    context->hal.hsyncCallbackSet = &hsyncCallbackSet;
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
    context->hal.layerAlphaEnableSet = &layerAlphaEnableSet;

    context->hal.drawPipeline[GFX_PIPELINE_GCU].pixelSet = &pixelSet;
    context->hal.drawPipeline[GFX_PIPELINE_GCU].pixelGet = &pixelGet;

    context->hal.drawPipeline[GFX_PIPELINE_GCUGPU].pixelSet = &pixelSet;
    context->hal.drawPipeline[GFX_PIPELINE_GCUGPU].pixelGet = &pixelGet;

    // accelerated draw line disabled for now, something keeps cutting off pixels
    // towards the bottom of the screen in some cases
    //context->hal.drawPipeline[GFX_PIPELINE_GCU].drawLine[GFX_DRAW_LINE][GFX_ANTIALIAS_OFF] = &drawLine;

    context->hal.drawPipeline[GFX_PIPELINE_GCU].drawRect[GFX_DRAW_FILL][GFX_ANTIALIAS_OFF] = &fillRect;
    context->hal.drawPipeline[GFX_PIPELINE_GCUGPU].drawRect[GFX_DRAW_FILL][GFX_ANTIALIAS_OFF] = &fillRect;

    return GFX_SUCCESS;
}