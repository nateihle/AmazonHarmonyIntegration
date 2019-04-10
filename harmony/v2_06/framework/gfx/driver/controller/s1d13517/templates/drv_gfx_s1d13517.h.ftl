/*******************************************************************************
  Company:
    Microchip Technology Incorporated

  File Name:
    drv_gfx_s1d13517_h

  Summary:
    Common header file for generated Epson S1D13517 display driver

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

#ifndef DRV_GFX_S1D13517_H
#define DRV_GFX_S1D13517_H

#include "gfx/hal/gfx.h"

#ifdef __cplusplus
    extern "C" {
#endif

<#if CONFIG_DRV_GFX_S1D13517_COLOR_MODE == "RGB_888">
#define S1D13517_MASK_COLOR_1 0xF0F0F0
#define S1D13517_MASK_COLOR_2 0xE0E0E0
<#elseif CONFIG_DRV_GFX_S1D13517_COLOR_MODE == "RGB_565">
#define S1D13517_MASK_COLOR_1 0xF79E
#define S1D13517_MASK_COLOR_2 0xE71C
</#if>

/*********************************************************************
 * Overview: S1D13517S1D13517_REGisters definitions.
 *********************************************************************/
#define S1D13517_REG00_PROD_CODE        0x00     // Product CodeS1D13517_REGister [READONLY]
#define S1D13517_REG02_READBACK         0x02     // Configuration ReadbackS1D13517_REGister [READONLY]
#define S1D13517_REG04_PLL_DDIVIDER     0x04     // PLL D-DividerS1D13517_REGister
#define S1D13517_REG06_PLL_0            0x06     // PLL SettingS1D13517_REGister 0
#define S1D13517_REG08_PLL_1            0x08     // PLL SettingS1D13517_REGister 1
#define S1D13517_REG0A_PLL_2            0x0A     // PLL SettingS1D13517_REGister 2
#define S1D13517_REG0C_PLL_NDIVIDER     0x0C     // PLL N-DividerS1D13517_REGister
#define S1D13517_REG0E_SS_CONTROL_0     0x0E     // SS ControlS1D13517_REGister 0
#define S1D13517_REG10_SS_CONTROL_1     0x10     // SS ControlS1D13517_REGister 1
#define S1D13517_REG12_CLK_SRC_SELECT   0x12     // Clock Source SelectS1D13517_REGister
#define S1D13517_REG14_LCD_PANEL_TYPE   0x14     // LCD Panel TypeS1D13517_REGister
#define S1D13517_REG16_HDISP_WIDTH      0x16     // Horizontal Display WidthS1D13517_REGister
#define S1D13517_REG18_HNDP_PERIOD      0x18     // Horizontal Non-Display PeriodS1D13517_REGister
#define S1D13517_REG1A_VDISP_HEIGHT_0   0x1A     // Vertical Display HeightS1D13517_REGister 0
#define S1D13517_REG1C_VDISP_HEIGHT_1   0x1C     // Vertical Display HeightS1D13517_REGister 1
#define S1D13517_REG1E_VNDP_PERIOD      0x1E     // Vertical Non-Display PeriodS1D13517_REGister
#define S1D13517_REG20_PHS_PULSE_WIDTH  0x20     // PHS Pulse Width (HSW)S1D13517_REGister
#define S1D13517_REG22_PHS_PULSE_START  0x22     // PHS Pulse Start Position (HPS)S1D13517_REGister
#define S1D13517_REG24_PVS_PULSE_WIDTH  0x24     // PVS Pulse Width (VSW)S1D13517_REGister
#define S1D13517_REG26_PVS_PULSE_START  0x26     // PVS Pulse Start Position (VPS)S1D13517_REGister
#define S1D13517_REG28_PCLK_POLARITY    0x28     // PCLK PolarityS1D13517_REGister
#define S1D13517_REG2A_DSP_MODE         0x2A     // Display ModeS1D13517_REGister
#define S1D13517_REG2C_PIP1_DSP_SA_0    0x2C     // PIP1 Display Start AddressS1D13517_REGister 0
#define S1D13517_REG2E_PIP1_DSP_SA_1    0x2E     // PIP1 Display Start AddressS1D13517_REGister 1
#define S1D13517_REG30_PIP1_DSP_SA_2    0x30     // PIP1 Display Start AddressS1D13517_REGister 2
#define S1D13517_REG32_PIP1_WIN_X_SP    0x32     // PIP1 Window X Start PositionS1D13517_REGister
#define S1D13517_REG34_PIP1_WIN_Y_SP_0  0x34     // PIP1 Window Y Start PositionS1D13517_REGister 0
#define S1D13517_REG36_PIP1_WIN_Y_SP_1  0x36     // PIP1 Window Y Start PositionS1D13517_REGister 1
#define S1D13517_REG38_PIP1_WIN_X_EP    0x38     // PIP1 Window X End PositionS1D13517_REGister
#define S1D13517_REG3A_PIP1_WIN_Y_EP_0  0x3A     // PIP1 Window Y End PositionS1D13517_REGister 0
#define S1D13517_REG3C_PIP1_WIN_Y_EP_1  0x3C     // PIP1 Window Y End PositionS1D13517_REGister 1
#define S1D13517_REG3E_PIP2_DSP_SA_0    0x3E     // PIP2 Display Start AddressS1D13517_REGister 0
#define S1D13517_REG40_PIP2_DSP_SA_1    0x40     // PIP2 Display Start AddressS1D13517_REGister 1
#define S1D13517_REG42_PIP2_DSP_SA_2    0x42     // PIP2 Display Start AddressS1D13517_REGister 2
#define S1D13517_REG44_PIP2_WIN_X_SP    0x44     // PIP2 Window X Start PositionS1D13517_REGister
#define S1D13517_REG46_PIP2_WIN_Y_SP_0  0x46     // PIP2 Window Y Start PositionS1D13517_REGister 0
#define S1D13517_REG48_PIP2_WIN_Y_SP_1  0x48     // PIP2 Window Y Start PositionS1D13517_REGister 1
#define S1D13517_REG4A_PIP2_WIN_X_EP    0x4A     // PIP2 Window X End PositionS1D13517_REGister
#define S1D13517_REG4C_PIP2_WIN_Y_EP_0  0x4C     // PIP2 Window Y End PositionS1D13517_REGister 0
#define S1D13517_REG4E_PIP2_WIN_Y_EP_1  0x4E     // PIP2 Window Y End PositionS1D13517_REGister 1
#define S1D13517_REG50_DISPLAY_CONTROL  0x50     // Display ControlS1D13517_REGister [WRITEONLY]
#define S1D13517_REG52_INPUT_MODE       0x52     // Input ModeS1D13517_REGister
#define S1D13517_REG54_TRANSP_KEY_RED   0x54     // Transparency Key Color RedS1D13517_REGister
#define S1D13517_REG56_TRANSP_KEY_GREEN 0x56     // Transparency Key Color GreenS1D13517_REGister
#define S1D13517_REG58_TRANSP_KEY_BLUE  0x58     // Transparency Key Color BlueS1D13517_REGister
#define S1D13517_REG5A_WRITE_WIN_X_SP   0x5A     // Write Window X Start PositionS1D13517_REGister
#define S1D13517_REG5C_WRITE_WIN_Y_SP_0 0x5C     // Write Window Start PositionS1D13517_REGister 0
#define S1D13517_REG5E_WRITE_WIN_Y_SP_1 0x5E     // Write Window Start PositionS1D13517_REGister 1
#define S1D13517_REG60_WRITE_WIN_X_EP   0x60     // Write Window X End PositionS1D13517_REGister
#define S1D13517_REG62_WRITE_WIN_Y_EP_0 0x62     // Write Window Y End PositionS1D13517_REGister 0
#define S1D13517_REG64_WRITE_WIN_Y_EP_1 0x64     // Write Window Y End PositionS1D13517_REGister 1
#define S1D13517_REG66_MEM_DATA_PORT_0  0x66     // Memory Data PortS1D13517_REGister 0 [WRITEONLY]
#define S1D13517_REG67_MEM_DATA_PORT_1  0x67     // Memory Data PortS1D13517_REGister 1 [WRITEONLY]
#define S1D13517_REG68_POWER_SAVE       0x68     // Power SaveS1D13517_REGister
#define S1D13517_REG6A_N_DISP_PER_CTRS  0x6A     // Non-Display Period Control/StatusS1D13517_REGister
#define S1D13517_REG6C_GPO_0            0x6C     // General Purpose OutputS1D13517_REGister 0
#define S1D13517_REG6E_GPO_1            0x6E     // General Purpose OutputS1D13517_REGister 1
#define S1D13517_REG70_PWM_CONTROL      0x70     // PWM ControlS1D13517_REGister
#define S1D13517_REG72_PWM_HIGH_DC_0    0x72     // PWM High Duty CycleS1D13517_REGister 0
#define S1D13517_REG74_PWM_HIGH_DC_1    0x74     // PWM High Duty CycleS1D13517_REGister 1
#define S1D13517_REG76_PWM_HIGH_DC_2    0x76     // PWM High Duty CycleS1D13517_REGister 2
#define S1D13517_REG78_PWM_HIGH_DC_3    0x78     // PWM High Duty CycleS1D13517_REGister 3
#define S1D13517_REG7A_PWM_LOW_DC_0     0x7A     // PWM Low Duty CycleS1D13517_REGister 0
#define S1D13517_REG7C_PWM_LOW_DC_1     0x7C     // PWM Low Duty CycleS1D13517_REGister 1
#define S1D13517_REG7E_PWM_LOW_DC_2     0x7E     // PWM Low Duty CycleS1D13517_REGister 2
#define S1D13517_REG80_PWM_LOW_DC_3     0x80     // PWM Low Duty CycleS1D13517_REGister 3
#define S1D13517_REG82_SDRAM_CONTROL_0  0x82     // SDRAM ControlS1D13517_REGister 0
#define S1D13517_REG84_SDRAM_STATUS_0   0x84     // SDRAM StatusS1D13517_REGister 0 [WRITEONLY]
#define S1D13517_REG86_SDRAM_STATUS_1   0x86     // SDRAM StatusS1D13517_REGister 1 [READONLY]
#define S1D13517_REG88_SDRAM_MRS_VAL_0  0x88     // SDRAM MRS ValueS1D13517_REGister 0
#define S1D13517_REG8A_SDRAM_MRS_VAL_1  0x8A     // SDRAM MRS ValueS1D13517_REGister 1
#define S1D13517_REG8C_SDRAM_RFS_CNT_0  0x8C     // SDRAM Refresh CounterS1D13517_REGister 0
#define S1D13517_REG8E_SDRAM_RFS_CNT_1  0x8E     // SDRAM Refresh CounterS1D13517_REGister 1
#define S1D13517_REG90_SDRAM_BUF_SIZE   0x90     // SDRAM Write Buffer Memory SizeS1D13517_REGister
#define S1D13517_REG92_SDRAM_DEBUG      0x92     // SDRAM DebugS1D13517_REGister
#define S1D13517_REG94_ALP_CONTROL      0x94     // Alpha-Blend ControlS1D13517_REGister [WRITEONLY]
#define S1D13517_REG96_ALP_STATUS       0x96     // Alpha-Blend StatusS1D13517_REGister [READONLY]
#define S1D13517_REG98_ALP_HR_SIZE      0x98     // Alpha-Blend Horizontal SizeS1D13517_REGister
#define S1D13517_REG9A_ALP_VR_SIZE_0    0x9A     // Alpha-Blend Vertical SizeS1D13517_REGister 0
#define S1D13517_REG9C_ALP_VR_SIZE_1    0x9C     // Alpha-Blend Vertical SizeS1D13517_REGister 1
#define S1D13517_REG9E_ALP_VALUE        0x9E     // Alpha-Blend ValueS1D13517_REGister
#define S1D13517_REGA0_ALP_IN_IMG1_SA_0 0xA0     // Alpha-Blend Input Image 1 Start AddressS1D13517_REGister 0
#define S1D13517_REGA2_ALP_IN_IMG1_SA_1 0xA2     // Alpha-Blend Input Image 1 Start AddressS1D13517_REGister 1
#define S1D13517_REGA4_ALP_IN_IMG1_SA_2 0xA4     // Alpha-Blend Input Image 1 Start AddressS1D13517_REGister 2
#define S1D13517_REGA6_ALP_IN_IMG2_SA_0 0xA6     // Alpha-Blend Input Image 2 Start AddressS1D13517_REGister 0
#define S1D13517_REGA8_ALP_IN_IMG2_SA_1 0xA8     // Alpha-Blend Input Image 2 Start AddressS1D13517_REGister 1
#define S1D13517_REGAA_ALP_IN_IMG2_SA_2 0xAA     // Alpha-Blend Input Image 2 Start AddressS1D13517_REGister 2
#define S1D13517_REGAC_ALP_OUT_IMG_SA_0 0xAC     // Alpha-Blend Output Image Start AddressS1D13517_REGister 0
#define S1D13517_REGAE_ALP_OUT_IMG_SA_1 0xAE     // Alpha-Blend Output Image Start AddressS1D13517_REGister 1
#define S1D13517_REGB0_ALP_OUT_IMG_SA_2 0xB0     // Alpha-Blend Output Image Start AddressS1D13517_REGister 2
#define S1D13517_REGB2_INTERRUPT_CTRL   0xB2     // Interrupt ControlS1D13517_REGister
#define S1D13517_REGB4_INTERRUPT_STAT   0xB4     // Interrupt StatusS1D13517_REGister [READONLY]
#define S1D13517_REGB6_INTERRUPT_CLEAR  0xB6     // Interrupt ClearS1D13517_REGister [WRITEONLY]
#define S1D13517_REGFLAG_BASE           0xF0     // Special reserved flags beyond this point
#define S1D13517_REGFLAG_DELAY          0xFC     // PLLS1D13517_REGister Programming Delay (in us)
#define S1D13517_REGFLAG_OFF_DELAY      0xFD     // LCD Panel Power Off Delay (in ms)
#define S1D13517_REGFLAG_ON_DELAY       0xFE     // LCD Panel Power On Delay (in ms)
#define S1D13517_REGFLAG_END_OF_TABLE   0xFF     // End ofS1D13517_REGisters Marker
//DOM-IGNORE-END


struct S1D13517DriverData
{
    uint32_t preDrawCount;
    uint32_t postDrawCount;
};

<#if CONFIG_GFX_HAL_DRAW_PIPELINE_ENABLED>
uint32_t drvS1D13517_SetRegister(uint8_t index, uint8_t value);
uint8_t  drvS1D13517_GetRegister(uint8_t index);

GFX_Result drvS1D13517_SetPixel(const GFX_PixelBuffer* buf,
                                const GFX_Point* pnt,
                                GFX_Color color);

GFX_Result drvS1D13517_DrawLine(const GFX_Point* p1,
                                const GFX_Point* p2,
                                const GFX_DrawState* state);
								
GFX_Result drvS1D13517_FillRect(const GFX_Rect* rect,
                                const GFX_DrawState* state);
</#if>

<#if CONFIG_USE_SEGGER_EMWIN_LIBRARY>
void drvS1D13517emWin_Write16A0(uint16_t Data);
void drvS1D13517emWin_Write16A1(uint16_t Data);
void drvS1D13517emWin_WriteM16_A1(uint16_t * pData, int NumItems);
void drvS1D13517emWin_SetCS(uint8_t State);
</#if>
					
#ifdef __cplusplus
    }
#endif

#endif /* DRV_GFX_S1D13517_H */