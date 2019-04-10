/*******************************************************************************
  Company:
    Microchip Technology Incorporated

  File Name:
    drv_gfx_ssd1963.c

  Summary:
    Main source file for SSD1963 display driver

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

#include <stdint.h>
#include <GenericTypeDefs.h>
#include "system/clk/sys_clk.h"
#include "system/debug/sys_debug.h"
#include "framework/gfx/driver/controller/ssd1963/drv_gfx_ssd1963.h"
#include "gfx/hal/inc/gfx_driver_interface.h"
#include "gfx/hal/inc/gfx_default_impl.h"

// Active Page
BYTE  _activePage = 0;

// Statics to allow call of ResetDevice
static uint16_t horzFrontPorch, horzBackPorch, horzPulseWidth, horzWidth;
static uint16_t vertFrontPorch, vertBackPorch, vertPulseWidth, vertHeight;

// Equivalences for ResetDevice
#define DISP_HOR_PULSE_WIDTH  horzPulseWidth
#define DISP_HOR_BACK_PORCH   horzBackPorch
#define DISP_HOR_RESOLUTION   horzWidth
#define DISP_HOR_FRONT_PORCH  horzFrontPorch

#define DISP_VER_PULSE_WIDTH  vertPulseWidth
#define DISP_VER_BACK_PORCH   vertBackPorch
#define DISP_VER_RESOLUTION   vertHeight
#define DISP_VER_FRONT_PORCH  vertFrontPorch

#define GetMaxX() (horzWidth-1)
#define GetMaxY() (vertHeight-1)

#define MAX_LAYER_COUNT  1
#define MAX_BUFFER_COUNT 1

const char* DRIVER_NAME = "SSD1963";

static uint32_t supportedColorModes = GFX_COLOR_MASK_RGB_565;

void SetArea(int16_t start_x, int16_t start_y, int16_t end_x, int16_t end_y);

/*********************************************************************
* Macros:  PMPWaitBusy()
*
* Overview: waits for PMP cycle end.
*
* PreCondition: none
*
* Input: none
*
* Output: none
*
* Side Effects: none
*
* Note:
********************************************************************/
#define PMPWaitBusy()  while(PMMODEbits.BUSY);


/*********************************************************************
* Macros:  WriteCommand(cmd)
*
* PreCondition:
*
* Input: cmd - controller command
*
* Output: none
*
* Side Effects: none
*
* Overview: writes command
*
* Note: none
********************************************************************/
  #define WriteCommand(cmd) {DATA_OR_COMMAND_BAROff(); \
                             CHIP_SELECT_BAROff(); \
                             WR_STROBE_BAROff(); \
                             PLIB_PMP_MasterSend(PMP_ID_0,cmd); \
                             EXTRA_NOPS \
                             WR_STROBE_BAROn(); \
                             CHIP_SELECT_BAROn(); EXTRA_NOPS \
                            }

/*********************************************************************
* Function:  void  WriteData(uint16_t data)
*
* PreCondition:
*
* Input:  value - value to be written in uint16_t format
*
* Output: none
*
* Side Effects: none
*
* Overview:
********************************************************************/
#define WriteData(cmd) {DATA_OR_COMMAND_BAROn(); \
                        CHIP_SELECT_BAROff(); \
                        WR_STROBE_BAROff(); \
                        PLIB_PMP_MasterSend(PMP_ID_0,cmd); \
                        EXTRA_NOPS \
                        WR_STROBE_BAROn(); \
                        CHIP_SELECT_BAROn(); EXTRA_NOPS \
                       }

/*********************************************************************
* Function:  void DelayMs(int ms)
********************************************************************/
static void DelayMs(int ms)
{
    uint32_t start = _CP0_GET_COUNT();
    uint32_t end = start + SYS_CLK_SystemFrequencyGet() / 1000 / 2 * ms;
    if (end > start)
    {
        while (_CP0_GET_COUNT() < end);
    }
    else
    {
        while (_CP0_GET_COUNT() > start || _CP0_GET_COUNT() < end);
    }
}

/*********************************************************************
* Function:  void ResetDevice()
*
* PreCondition: none
*
* Input: none
*
* Output: none
*
* Side Effects: none
*
* Overview: Resets LCD, initializes PMP
*           Initialize low level IO port for mcu,
*           initialize SSD1963 for PCLK,
*           HSYNC, VSYNC etc
*
* Note: Need to set the backlight intensity by SetBacklight(BYTE intensity)
*       in main()
*
********************************************************************/
void ResetDevice(void)
{
    uint8_t  iByte;
    uint32_t lcdc_fpr;  // Mimics LCDC_FPR in the data sheet

    RESET_BAROn();
    CHIP_SELECT_BAROn();
    RD_STROBE_BAROn();
    WR_STROBE_BAROn();

    // PMP setup
    PLIB_PMP_Disable( PMP_ID_0 );

    PLIB_PMP_OperationModeSelect( PMP_ID_0,
                                  PMP_MASTER_READ_WRITE_STROBES_INDEPENDENT  );

    // Before PLL is set and locked, the reference clock = crystal freq.
    // Set the largest data setup time for 10MHz clock
    PLIB_PMP_DataSizeSelect(PMP_ID_0,PMP_DATA_SIZE_16_BITS);

    PLIB_PMP_ReadWriteStrobePortDisable(PMP_ID_0);
    PLIB_PMP_WriteEnableStrobePortDisable(PMP_ID_0);

    PLIB_PMP_Enable( PMP_ID_0 );

    RESET_BAROff();
    DelayMs(1);
    RESET_BAROn();
    //Set MN(multipliers) of PLL, VCO = crystal freq * (N+1)
    //PLL freq = VCO/M with 250MHz < VCO < 800MHz
    //The max PLL freq is 110MHz. To obtain 100MHz as the PLL freq do the following:

    WriteCommand(CMD_SET_PLL_MN);  // Set PLL with OSC = 10MHz (hardware)
                                   // Multiplier M = 29, VCO (>250MHz)= OSC*(M+1), VCO = 300MHz
    WriteData(0x1D);               // Muliplier M = 29, VCO = 12*(N+1)  = 300 MHz
    WriteData(0x02);               // Divider N = 2,   PLL = 360/(N+1) = 100MHz
    WriteData(0x54);               // Validate M and N values ("Effectuate" values)

    WriteCommand(CMD_PLL_START);    // Start PLL command
    WriteData(0x01);                // enable PLL

    DelayMs(10);                    // wait to stabilize

    WriteCommand(CMD_PLL_START);    // Start PLL command again
    WriteData(0x03);                // now, use PLL output as system clock

    //once PLL locked (at 100MHz), the data hold time set shortest
    WriteCommand(CMD_SOFT_RESET);
    DelayMs(10);

  //WriteCommand(CMD_SET_PCLK);   // Set Pixel clock to 15 MHz
  //Set LSHIFT freq, i.e. the DCLK with PLL freq 120MHz set previously
  //Typical DCLK for PDA TMA4301B is 15MHz
  //15MHz = 100MHz*(LCDC_FPR+1)/2^20
  //LCDC_FPR = (15*2^20)/100 - 1 = 131,071 (0x02665)
  //WriteData(0x02);
  //WriteData(0x66);
  //WriteData(0x65);

    WriteCommand(CMD_SET_PCLK);   // Set Pixel clock to 15 MHz
    lcdc_fpr = ( (uint32_t)((1UL<<20)/SSD1963_PIXEL_CLOCK_DIVIDER) ) - 1 ;
    SYS_DEBUG_PRINT(SYS_ERROR_INFO,"SSD1963 driver LCDC_FPR: %d, (0x%06X)\r\n",lcdc_fpr,lcdc_fpr);
    iByte = 0xFF & (lcdc_fpr>>16);
    WriteData( iByte );
    iByte = 0xFF & (lcdc_fpr>>8);
    WriteData( iByte );
    iByte = 0xFF &  lcdc_fpr;
    WriteData( iByte );

    WriteCommand(CMD_SET_PANEL_MODE);//Set panel mode, varies from individual manufacturer
    WriteData(0x20); // set 24-bit for TY430TF480272 4.3" panel data latch in rising edge for LSHIFT
    WriteData(0x00); // set Hsync+Vsync mode
    WriteData((DISP_HOR_RESOLUTION-1)>>8); //Set panel size
    WriteData(DISP_HOR_RESOLUTION-1);
    WriteData((DISP_VER_RESOLUTION-1)>>8);
    WriteData(DISP_VER_RESOLUTION-1);
    WriteData(0x00);                //RGB sequence

    WriteCommand(CMD_SET_HOR_PERIOD);  //Set horizontal period
    #define HT (DISP_HOR_PULSE_WIDTH+DISP_HOR_BACK_PORCH+DISP_HOR_RESOLUTION+DISP_HOR_FRONT_PORCH)
    WriteData((HT-1)>>8);
    WriteData(HT-1);
    #define HPS (DISP_HOR_PULSE_WIDTH+DISP_HOR_BACK_PORCH)
    WriteData((HPS-1)>>8);
    WriteData(HPS-1);
    WriteData(DISP_HOR_PULSE_WIDTH-1);
    WriteData(0x00);
    WriteData(0x00);
    WriteData(0x00);

    WriteCommand(CMD_SET_VER_PERIOD);  //Set vertical period
    #define VT (DISP_VER_PULSE_WIDTH+DISP_VER_BACK_PORCH+DISP_VER_RESOLUTION+DISP_VER_FRONT_PORCH)
    WriteData((VT-1)>>8);
    WriteData(VT-1);
    #define VSP (DISP_VER_PULSE_WIDTH+DISP_VER_BACK_PORCH)
    WriteData((VSP-1)>>8);
    WriteData(VSP-1);
    WriteData(DISP_VER_PULSE_WIDTH-1);
    WriteData(0x00);
    WriteData(0x00);

    WriteCommand(CMD_SET_PIXEL_FORMAT);    //Set pixel format, i.e. the bpp
    WriteData(0x55);                // set 16bpp

    WriteCommand(CMD_SET_DATA_INTERFACE);    //Set pixel data interface
    WriteData(0x03); //16-bit(565 format) data for 16bpp PIC32MX only

    WriteCommand(CMD_ON_DISPLAY); // Turn on display; show the image on display

  #if defined( SSD1963_DRIVES_PWM )
    SetBacklight(0xFF);
  #else
    BACKLIGHT_PWMOn(); // Backlight on
  #endif

}

static GFX_Result initialize(GFX_Context* context)
{
    uint32_t i;
    GFX_Result resultBufferCreate;

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

        resultBufferCreate =
          GFX_PixelBufferCreate(context->display_info->rect.width,
                                context->display_info->rect.height,
                                GFX_COLOR_MODE_RGB_565,
                                NULL,
                                &context->layer.layers[i].buffers[0].pb);
        assert(resultBufferCreate == GFX_SUCCESS);

        context->layer.layers[i].buffers[0].state = GFX_BS_MANAGED;
    }

    horzFrontPorch = context->display_info->attributes.horz.front_porch;
    horzBackPorch  = context->display_info->attributes.horz.back_porch;
    horzPulseWidth = context->display_info->attributes.horz.pulse_width;
    horzWidth      = context->display_info->rect.width;

    vertFrontPorch = context->display_info->attributes.vert.front_porch;
    vertBackPorch  = context->display_info->attributes.vert.back_porch;
    vertPulseWidth = context->display_info->attributes.vert.pulse_width;
    vertHeight     = context->display_info->rect.height;

    ResetDevice();

    return GFX_SUCCESS;
}


/*********************************************************************
* Function:  SetArea(start_x,start_y,end_x,end_y)
*
* PreCondition: SetActivePage(page)
*
* Input: start_x, end_x - start column and end column
*        start_y,end_y  - start row and end row position (i.e. page address)
*
* Output: none
*
* Side Effects: none
*
* Overview: defines start/end columns and start/end rows for memory access
*           from host to SSD1963
* Note: none
********************************************************************/
void SetArea(int16_t start_x, int16_t start_y, int16_t end_x, int16_t end_y)
{
    uint32_t offset;

    offset = (int16_t)_activePage*(GetMaxY()+1);

    start_y = offset + start_y;
    end_y   = offset + end_y;

    WriteCommand(CMD_SET_COLUMN);
    WriteData(start_x>>8);
    WriteData(start_x);
    WriteData(end_x>>8);
    WriteData(end_x);

    WriteCommand(CMD_SET_PAGE);
    WriteData(start_y>>8);
    WriteData(start_y);
    WriteData(end_y>>8);
    WriteData(end_y);

}

/*********************************************************************
* Function:  SetScrollArea(int16_t top, int16_t scroll, int16_t bottom)
*
* PreCondition: none
*
* Input: top - Top Fixed Area in number of lines from the top
*               of the frame buffer
*        scroll - Vertical scrolling area in number of lines
*        bottom - Bottom Fixed Area in number of lines
*
* Output: none
*
* Side Effects: none
*
* Overview:
*
* Note: Reference: section 9.22 Set Scroll Area, SSD1963 datasheet Rev0.20
********************************************************************/
void SetScrollArea(int16_t top, int16_t scroll, int16_t bottom)
{
    WriteCommand(CMD_SET_SCROLL_AREA);
    WriteData(top>>8);
    WriteData(top);
    WriteData(scroll>>8);
    WriteData(scroll);
    WriteData(bottom>>8);
    WriteData(bottom);
}

/*********************************************************************
* Function:  void  SetScrollStart(int16_t line)
*
* Overview: First, we need to define the scrolling area by SetScrollArea()
*           before using this function.
*
* PreCondition: SetScrollArea(int16_t top, int16_t scroll, int16_t bottom)
*
* Input: line - Vertical scrolling pointer (in number of lines) as
*        the first display line from the Top Fixed Area defined in SetScrollArea()
*
* Output: none
*
* Note: Example -
*
*       int16_t line=0;
*       SetScrollArea(0,272,0);
*       for(line=0;line<272;line++) {SetScrollStart(line);DelayMs(100);}
*
*       Code above scrolls the whole page upwards in 100ms interval
*       with page 2 replacing the first page in scrolling
********************************************************************/
void SetScrollStart(int16_t line)
{
    WriteCommand(CMD_SET_SCROLL_START);
    WriteData(line>>8);
    WriteData(line);
}

/*********************************************************************
* Function:  void EnterSleepMode (void)
* PreCondition: none
* Input:  none
* Output: none
* Side Effects: none
* Overview: SSD1963 enters sleep mode
* Note: Host must wait 5mS after sending before sending next command
********************************************************************/
void EnterSleepMode (void)
{
    WriteCommand(CMD_ENT_SLEEP);
}

/*********************************************************************
* Function:  void ExitSleepMode (void)
* PreCondition: none
* Input:  none
* Output: none
* Side Effects: none
* Overview: SSD1963 enters sleep mode
* Note:   none
********************************************************************/
void ExitSleepMode (void)
{
    WriteCommand(CMD_EXIT_SLEEP);
}

/*********************************************************************
* Function      : void DisplayOff(void)
* PreCondition  : none
* Input         : none
* Output        : none
* Side Effects  : none
* Overview      : SSD1963 changes the display state to OFF state
* Note          : none
********************************************************************/
void DisplayOff(void)
{
    WriteCommand(CMD_BLANK_DISPLAY);
}

/*********************************************************************
* Function      : void DisplayOn(void)
* PreCondition  : none
* Input         : none
* Output        : none
* Side Effects  : none
* Overview      : SSD1963 changes the display state to ON state
* Note          : none
********************************************************************/
void DisplayOn(void)
{
    WriteCommand(CMD_ON_DISPLAY);
}

/*********************************************************************
* Function      : void EnterDeepSleep(void)
* PreCondition  : none
* Input         : none
* Output        : none
* Side Effects  : none
* Overview      : SSD1963 enters deep sleep state with PLL stopped
* Note          : none
********************************************************************/
void EnterDeepSleep(void)
{
    WriteCommand(CMD_ENT_DEEP_SLEEP);
}

/*********************************************************************
* Function:  void  SetBacklight(BYTE intensity)
*
* Overview: This function makes use of PWM feature of ssd1963 to adjust
*           the backlight intensity.
*
* PreCondition: Backlight circuit with shutdown pin connected to PWM output of ssd1963.
*
* Input:    (BYTE) intensity from
*           0x00 (total backlight shutdown, PWM pin pull-down to VSS)
*           0xff (99% pull-up, 255/256 pull-up to VDD)
*
* Output: none
*
* Note: The base frequency of PWM set to around 300Hz with PLL set to 120MHz.
*       This parameter is hardware dependent
********************************************************************/
void SetBacklight(BYTE intensity) // NOT YET SUPPORTED ON HARDWARE!!!
{
    WriteCommand(CMD_SET_PWM_CONF);

    WriteData(0x0E);            // PWMF[7:0] = 2, PWM base freq = PLL/(256*(1+5))/256 =
                                // 300Hz for a PLL freq = 120MHz
    WriteData(intensity);       // Set duty cycle, from 0x00 (total pull-down) to 0xFF
                                // (99% pull-up , 255/256)
    WriteData(0x01);            // PWM enabled and controlled by host (mcu)
    WriteData(0x00);
    WriteData(0x00);
}

/*********************************************************************
* Function:  void  SetTearingCfg(BOOL state, BOOL mode)
*
* Overview: This function enable/disable tearing effect
*
* PreCondition: none
*
* Input:    BOOL state -    1 to enable
*                           0 to disable
*           BOOL mode -     0:  the tearing effect output line consists
*                               of V-blanking information only
*                           1:  the tearing effect output line consists
*                               of both V-blanking and H-blanking info.
* Output: none
*
* Note:
********************************************************************/
void SetTearingCfg(BOOL state, BOOL mode)
{
    if(state == 1)
    {
        WriteCommand(CMD_SET_TEAR_ON);
        WriteData(mode&0x01);
    }
    else
    {
        WriteCommand(0x34);
    }


}

static GFX_Result pixelSet(const GFX_PixelBuffer* buf,
                           const GFX_Point* pnt,
                           GFX_Color color)
{
    SetArea(pnt->x,pnt->y,GetMaxX(),GetMaxY());
    WriteCommand(CMD_WR_MEMSTART);
    WriteData(color);

    return GFX_SUCCESS;
}

GFX_Result fillRect(const GFX_Rect* pRect,
                    const GFX_DrawState* pState)
{
    int16_t      x, y, left, right, top, bottom;
    GFX_Context* pContext = GFX_ActiveContext();
    GFX_Layer*   pLayer  = pContext->layer.active;
    GFX_Rect     clipRect;
    GFX_Color    color;

    // a basic fill is an optimal case for this driver
    // everything else should go through software pixel pipeline
    if(pContext->orientation != GFX_ORIENTATION_0 ||
       pContext->mirrored != GFX_FALSE)
        return cpuDrawRect_Fill(pRect, pState);

    // clip against the physical pLayer bounds
    if(GFX_PixelBufferClipRect(&pLayer->buffers[pLayer->buffer_write_idx].pb,
                               pRect,
                               &clipRect) == GFX_FAILURE)
    { // Input rectangle doesn't intersect with clip rectangle, we're done!
        return GFX_FAILURE;
    }

  #if GFX_BOUNDS_CLIPPING_ENABLED
    // clip against the global clipping rectangle
    if(pState->clipEnable == GFX_TRUE)
    {
        if(GFX_RectIntersects(pRect, &pState->clipRect) == GFX_FALSE)
            return GFX_SUCCESS;

        GFX_RectClip(pRect, &pState->clipRect, &clipRect);
    }
    else
    { // Clipping not on
        clipRect = *pRect;
    }
  #else
    clipRect = *pRect; // Clipping disabled.
  #endif

    left   = clipRect.x;
    right  = left + clipRect.width;
    top    = clipRect.y;
    bottom = top + clipRect.height;
    color  = pState->color;

    SetArea(left,top,right,bottom);

    WriteCommand(CMD_WR_MEMSTART);
    for(y=top; y<bottom+1; y++){
        for(x=left; x<right+1; x++){
            WriteData(color);
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

static GFX_Result layerAlphaEnableSet(GFX_Bool enable,GFX_Bool wait)
{
    return GFX_UNSUPPORTED;
}

static GFX_Color pixelGet(const GFX_PixelBuffer* buf,
                          const GFX_Point* pnt)
{
    return 0;
}

// function that returns the information for this driver
GFX_Result driverSSD1963InfoGet(GFX_DriverInfo* info)
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
GFX_Result driverSSD1963ContextInitialize(GFX_Context* context)
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

    context->hal.drawPipeline[GFX_PIPELINE_GCU].drawRect[GFX_DRAW_FILL][GFX_ANTIALIAS_OFF] = &fillRect;
    context->hal.drawPipeline[GFX_PIPELINE_GCUGPU].drawRect[GFX_DRAW_FILL][GFX_ANTIALIAS_OFF] = &fillRect;

    return GFX_SUCCESS;
}