/*******************************************************************************
  Company:
    Microchip Technology Incorporated

  File Name:
    drv_gfx_otm2201a.c

  Summary:
    Main source file for OTM2201A display driver

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

#include "gfx/hal/inc/gfx_driver_interface.h"
#include "gfx/hal/inc/gfx_default_impl.h"

#include "framework/driver/pmp/drv_pmp_static.h"

#define MAX_LAYER_COUNT  1
#define MAX_BUFFER_COUNT 1

const char* DRIVER_NAME = "OTM2201A";

static uint32_t supportedColorModes = GFX_COLOR_MASK_RGB_565;

#define RAM_LINE_WIDTH                      256

#define NUM_BYTES_2                         2

#define REPEAT_CNT_0                        0

#define REG_ADDR_LOW_INDEX_SET              0
#define REG_ADDR_LOW_DATA_SET               1
#define REG_ADDR_HIGH_INDEX_SET             2
#define REG_ADDR_HIGH_DATA_SET              3

#define REG_WRITE_INDEX_SET                 0
#define REG_WRITE_DATA_SET                  1
#define REG_READ_DUMMY_DATA                 1
#define REG_READ_DATA                       2
#define REG_TRANSFER_STATUS                 3

#define DRV_OTM2201A_ERROR_NO_ERROR          0
#define DRV_OTM2201A_ERROR_PMP_WRITE         1
#define DRV_OTM2201A_ERROR_PMP_READ          1
#define DRV_OTM2201A_ERROR_QUEUE_FULL        1
#define DRV_OTM2201A_ERROR_REG_GET           1
#define DRV_OTM2201A_ERROR_REG_SET           1
#define DRV_OTM2201A_ERROR_DEVICE_BUSY       1

#define DRV_OTM2201A_ADDR_LOW_MASK           0x000000FF
#define DRV_OTM2201A_ADDR_LOW_SHIFT          0

#define DRV_OTM2201A_ADDR_HIGH_MASK          0x0000FF00
#define DRV_OTM2201A_ADDR_HIGH_SHIFT         8

#define DRV_OTM2201A_FEATURE_NOT_SUPPORTED   1

#define DATA_WIDTH_18                       18
#define DATA_WIDTH_24                       24

#define REG_DEV_CODE_READ                       0x00
#define REG_DRV_OUT_CTRL                        0x01
#define REG_LCD_AC_DRV_CTRL                     0x02
#define REG_ENTRY_MODE                          0x03
#define REG_DISP_CTRL                           0x07
#define REG_BLANKING_CTRL                       0x08
#define REG_FRAME_CYCLE_CTRL                    0x0B
#define REG_EXTERNAL_INTERFACE_CTRL             0x0C
#define REG_OSC_CTRL                            0x0F
#define REG_POWER_CTRL_1                        0x10
#define REG_POWER_CTRL_2                        0x11
#define REG_POWER_CTRL_3                        0x12
#define REG_POWER_CTRL_4                        0x13
#define REG_POWER_CTRL_5                        0x14
#define REG_VCI_PERIOD                          0x15
#define REG_RAM_ADDR_LOW                        0x20
#define REG_RAM_ADDR_HIGH                       0x21
#define REG_GRAM_DATA                           0x22
#define REG_SW_RESET                            0x28
#define REG_GATE_SCAN_START_POS                 0x30
#define REG_VERT_SCROLL_CTRL_1                  0x31
#define REG_VERT_SCROLL_CTRL_2                  0x32
#define REG_VERT_SCROLL_CTRL_3                  0x33
#define REG_PART_SCREEN_AREA_1                  0x34
#define REG_PART_SCREEN_AREA_2                  0x35
#define REG_HORZ_WND_ADDR_1                     0x36
#define REG_HORZ_WND_ADDR_2                     0x37
#define REG_VERT_WND_ADDR_1                     0x38
#define REG_VERT_WND_ADDR_2                     0x39
#define REG_GAMMA_CTRL_1                        0x50
#define REG_GAMMA_CTRL_2                        0x51
#define REG_GAMMA_CTRL_3                        0x52
#define REG_GAMMA_CTRL_4                        0x53
#define REG_GAMMA_CTRL_5                        0x54
#define REG_GAMMA_CTRL_6                        0x55
#define REG_GAMMA_CTRL_7                        0x56
#define REG_GAMMA_CTRL_8                        0x57
#define REG_GAMMA_CTRL_9                        0x58
#define REG_GAMMA_CTRL_10                       0x59

#define ENTRY_MODE_0                            0x1000
#define ENTRY_MODE_1                            0x1008
#define ENTRY_MODE_2                            0x1030

#define SCAN_MODE_0                             0x001C
#define SCAN_MODE_1                             0x011C
#define SCAN_MODE_2                             0x031C

// 0 degrees entry 2, scan 0
// 90 degress entry 1, scan 1
// 180 degrees entry 0, scan 1
// 270 degrees entry 1, scan 2

static void TMR_DelayMS( uint32_t delay_in_ms )
{
    uint32_t tWait = ( SYS_CLK_FREQ / 2000 ) * delay_in_ms;
    uint32_t tStart = _CP0_GET_COUNT();
    while( ( _CP0_GET_COUNT() - tStart ) < tWait );
}

<#if CONFIG_DRV_PMP_DATA_SIZE == "PMP_DATA_SIZE_16_BITS">
// 16 bit PMP
static void PMP_Write(uint16_t val)
{
    DRV_PMP0_Write(val);
}
<#elseif CONFIG_DRV_PMP_DATA_SIZE == "PMP_DATA_SIZE_8_BITS">
// 8 bit PMP
static void PMP_Write(uint16_t val)
{
    DRV_PMP0_Write((uint8_t)(val >> 8));
    DRV_PMP0_Write((uint8_t)val);
}
</#if>

static uint16_t setRegister(uint16_t index, uint16_t value)
{

    BSP_DisplaySetOff(); // set RS line to low for command
    PMP_Write(index);

    BSP_DisplaySetOn(); // set RS line to low for command
    PMP_Write(value);

    return(DRV_OTM2201A_ERROR_NO_ERROR);
}

static uint16_t setWriteAddress(uint32_t address)
{
    static uint32_t data = 0x00000000;

    BSP_DisplaySetOff(); // set RS line to low for command
    PMP_Write(REG_RAM_ADDR_LOW);
    BSP_DisplaySetOn(); // set RS line to low for command

    data  = address & DRV_OTM2201A_ADDR_LOW_MASK;
    PMP_Write(data);

    BSP_DisplaySetOff(); // set RS line to low for command
    PMP_Write(REG_RAM_ADDR_HIGH);
    BSP_DisplaySetOn(); // set RS line to low for command

    data = (address & DRV_OTM2201A_ADDR_HIGH_MASK)
            >> DRV_OTM2201A_ADDR_HIGH_SHIFT;

    PMP_Write(data);

    return(DRV_OTM2201A_ERROR_NO_ERROR);
}

static uint8_t getRegister(uint16_t  index, uint16_t *data)
{

    static uint16_t myReadBuffer = 0x0000;

    BSP_DisplaySetOff(); // set RS line to low for command
    PMP_Write(index);
    BSP_DisplaySetOn(); // set RS line to low for command

    myReadBuffer = PMDIN;
    while (PMMODEbits.BUSY);
    PMCONbits.PMPEN = 0; // disable PMP
    myReadBuffer = PMDIN;
    while (PMMODEbits.BUSY);
    PMCONbits.PMPEN = 1; // enable  PMP

    *data  = myReadBuffer;
    return(DRV_OTM2201A_ERROR_NO_ERROR);

}

GFX_Result setPixel(const GFX_PixelBuffer* buf,
                    const GFX_Point* pnt,
                    GFX_Color color)
{
     uint32_t address = pnt->x | (pnt->y << 8);

     while(setWriteAddress(address));

     BSP_DisplaySetOff(); // set RS line to low for command
     PMP_Write(REG_GRAM_DATA);
     BSP_DisplaySetOn(); // set RS line to low for command

     PMP_Write(color);

     return GFX_SUCCESS;
}

static GFX_Result initialize(GFX_Context* context)
{
    uint8_t state = 0;
    uint32_t i;
    uint16_t dummy = 1;
    uint16_t devId;
    uint8_t regOffset = 0;

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

    while (state <= 32)
    {
        switch (state)
        {
            case 0:
            {
                /* hold in reset */
                BSP_DisplayResetOff();

                TMR_DelayMS(20);

                /* release from reset */
                BSP_DisplayResetOn();


                TMR_DelayMS(20);

                BSP_DisplayOff();

                TMR_DelayMS(20);

                break;
            }
            case 1:
            {
                while (!dummy)
                    while (getRegister(REG_POWER_CTRL_2, &dummy));

                getRegister(0x00, &devId);

                switch (devId)
                {
                    case 0x77: // newer bt audio dev boards (correct value)
                    case 0x3: // newer bt audio dev boards (leave for aria quickstart)
                    {
                       regOffset = 0;
                       break;
                    }
                    default:
                    {
                        regOffset = 33;
                        break;
                    }
                }

                state = 2;
                break;
            }
            case 3:
            {
                break;
            }
            default:
            {
                break;
            }
        }

        //OTM2201A Registers to be initialized
        uint16_t registers[66][2] =
        {
            //  0x5075 REG_DEV_CODE_READ
            { REG_DRV_OUT_CTRL, SCAN_MODE_1 },
            { REG_LCD_AC_DRV_CTRL, 0x0100 },
            { REG_ENTRY_MODE, ENTRY_MODE_1 },
            { REG_BLANKING_CTRL, 0x0808 },
            { REG_EXTERNAL_INTERFACE_CTRL, 0x0000 },
            { REG_OSC_CTRL, 0x0001 },
            { REG_RAM_ADDR_LOW, 0x0000 },
            { REG_RAM_ADDR_HIGH, 0x0000 },
            { REG_POWER_CTRL_1, 0x0000 },
            { REG_POWER_CTRL_2, 0x1000 },
            { REG_GATE_SCAN_START_POS, 0x0000 },
            { REG_VERT_SCROLL_CTRL_1, 0x00db },
            { REG_VERT_SCROLL_CTRL_2, 0x0000 },
            { REG_VERT_SCROLL_CTRL_3, 0x0000 },
            { REG_PART_SCREEN_AREA_1, 0x00db },
            { REG_PART_SCREEN_AREA_2, 0x0000 },
            { REG_HORZ_WND_ADDR_1, 0x00AF },
            { REG_HORZ_WND_ADDR_2, 0x0000 },
            { REG_VERT_WND_ADDR_1, 0x00DB },
            { REG_VERT_WND_ADDR_2, 0x0000 },
            { REG_GAMMA_CTRL_1, 0x0203 },
            { REG_GAMMA_CTRL_2, 0x0a09 },
            { REG_GAMMA_CTRL_3, 0x0005 },
            { REG_GAMMA_CTRL_4, 0x1021 },
            { REG_GAMMA_CTRL_5, 0x0602 },
            { REG_GAMMA_CTRL_6, 0x0003 },
            { REG_GAMMA_CTRL_7, 0x0703 },
            { REG_GAMMA_CTRL_8, 0x0507 },
            { REG_GAMMA_CTRL_9, 0x1021 },
            { REG_GAMMA_CTRL_10, 0x0703 },
            { REG_OSC_CTRL, 0x2501 },
            { REG_DISP_CTRL, 0x0000 },
            { REG_DISP_CTRL, 0x0017 },

            /*  0x5064 REG_DEV_CODE_READ */
            { REG_POWER_CTRL_2,    0x0018 },
            { REG_POWER_CTRL_3,    0x0000 },
            { REG_POWER_CTRL_4,    0x0063 },
            { REG_POWER_CTRL_5,    0x556A },
            { REG_POWER_CTRL_1,    0x0800 },
            { REG_POWER_CTRL_2,    0x0118 },
            { REG_POWER_CTRL_2,    0x0318 },
            { REG_POWER_CTRL_2,    0x0718 },
            { REG_POWER_CTRL_2,    0x0F18 },
            { REG_POWER_CTRL_2,    0x0F38 },
            { REG_DISP_CTRL,       0x001A },
            { REG_DRV_OUT_CTRL,    SCAN_MODE_1 },
            { REG_ENTRY_MODE,      ENTRY_MODE_1 },
            { REG_DISP_CTRL,       0x0000 },
            { REG_BLANKING_CTRL,   0x0808 },
            { REG_VCI_PERIOD,      0x0020 },
            { REG_HORZ_WND_ADDR_1, 0x00AF },
            { REG_HORZ_WND_ADDR_2, 0x0000 },
            { REG_VERT_WND_ADDR_1, 0x00DB },
            { REG_VERT_WND_ADDR_2, 0x0000 },
            { REG_GAMMA_CTRL_1,    0x0001 },
            { REG_GAMMA_CTRL_2,    0x0208 },
            { REG_GAMMA_CTRL_3,    0x0805 },
            { REG_GAMMA_CTRL_4,    0x0404 },
            { REG_GAMMA_CTRL_5,    0x0C0C },
            { REG_GAMMA_CTRL_6,    0x000C },
            { REG_GAMMA_CTRL_7,    0x0100 },
            { REG_GAMMA_CTRL_8,    0x0400 },
            { REG_GAMMA_CTRL_9,    0x1108 },
            { REG_GAMMA_CTRL_10,   0x050C },
            { REG_OSC_CTRL,        0x1F01 },
            { REG_DISP_CTRL,       0x0012 },
            { REG_DISP_CTRL,       0x0017 }
        };

        while (setRegister(*registers[state+regOffset],
                           registers[state+regOffset][1]));

        state++;
    }

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
GFX_Result driverOTM2201AInfoGet(GFX_DriverInfo* info)
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
GFX_Result driverOTM2201AContextInitialize(GFX_Context* context)
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

    context->hal.drawPipeline[GFX_PIPELINE_GCU].pixelSet = &setPixel;
    context->hal.drawPipeline[GFX_PIPELINE_GCU].pixelGet = &pixelGet;

    context->hal.drawPipeline[GFX_PIPELINE_GCUGPU].pixelSet = &setPixel;
    context->hal.drawPipeline[GFX_PIPELINE_GCUGPU].pixelGet = &pixelGet;

    return GFX_SUCCESS;
}