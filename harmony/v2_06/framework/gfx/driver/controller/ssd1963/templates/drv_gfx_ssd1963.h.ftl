/*******************************************************************************
  Company:
    Microchip Technology Incorporated

  File Name:
    drv_gfx_ssd1963_h

  Summary:
    Common header file for generated SSD1963 display driver

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

#ifndef DRV_GFX_SSD1963_H
#define DRV_GFX_SSD1963_H

#include "gfx/hal/gfx.h"

#ifdef __cplusplus
    extern "C" {
#endif

//DOM-IGNORE-BEGIN

#define SSD1963_PIXEL_CLOCK_DIVIDER  ${CONFIG_DRV_GFX_SSD1963_CLK_DIVIDER}

<#if CONFIG_DRV_GFX_SSD1963_DRIVES_BACKLIGHT_PWM == true>
#define SSD1963_DRIVES_PWM
</#if>

<#if CONFIG_DRV_GFX_SSD1963_EXTRA_NOPS_SELECTED == "0">
#define EXTRA_NOPS
</#if>
<#if CONFIG_DRV_GFX_SSD1963_EXTRA_NOPS_SELECTED == "1">
#define EXTRA_NOPS asm("NOP");
</#if>
<#if CONFIG_DRV_GFX_SSD1963_EXTRA_NOPS_SELECTED == "2">
#define EXTRA_NOPS asm("NOP");asm("NOP");
</#if>

#include "./peripheral/pmp/plib_pmp.h"

#define CMD_NOP                 0x00    //No operation
#define CMD_SOFT_RESET          0x01    //Software reset
#define CMD_GET_PWR_MODE        0x0A    //Get the current power mode
#define CMD_GET_ADDR_MODE       0x0B    //Get the frame memory to the display panel read order
#define CMD_GET_PIXEL_FORMAT    0x0C    //Get the current pixel format
#define CMD_GET_DISPLAY_MODE    0x0D    //Returns the display mode
#define CMD_GET_SIGNAL_MODE     0x0E    //
#define CMD_GET_DIAGNOSTIC      0x0F
#define CMD_ENT_SLEEP           0x10
#define CMD_EXIT_SLEEP          0x11
#define CMD_ENT_PARTIAL_MODE    0x12
#define CMD_ENT_NORMAL_MODE     0x13
#define CMD_EXIT_INVERT_MODE    0x20
#define CMD_ENT_INVERT_MODE     0x21
#define CMD_SET_GAMMA           0x26
#define CMD_BLANK_DISPLAY       0x28
#define CMD_ON_DISPLAY          0x29
#define CMD_SET_COLUMN          0x2A
#define CMD_SET_PAGE            0x2B
#define CMD_WR_MEMSTART         0x2C
#define CMD_RD_MEMSTART         0x2E
#define CMD_SET_PARTIAL_AREA    0x30
#define CMD_SET_SCROLL_AREA     0x33
#define CMD_SET_TEAR_OFF        0x34    //synchronization information is not sent from the display
#define CMD_SET_TEAR_ON         0x35    //sync. information is sent from the display
#define CMD_SET_ADDR_MODE       0x36    //set fram buffer read order to the display panel
#define CMD_SET_SCROLL_START    0x37
#define CMD_EXIT_IDLE_MODE      0x38
#define CMD_ENT_IDLE_MODE       0x39
#define CMD_SET_PIXEL_FORMAT    0x3A    //defines how many bits per pixel is used
#define CMD_WR_MEM_AUTO         0x3C
#define CMD_RD_MEM_AUTO         0x3E
#define CMD_SET_TEAR_SCANLINE   0x44
#define CMD_GET_SCANLINE        0x45
#define CMD_RD_DDB_START        0xA1
#define CMD_RD_DDB_AUTO         0xA8
#define CMD_SET_PANEL_MODE      0xB0
#define CMD_GET_PANEL_MODE      0xB1
#define CMD_SET_HOR_PERIOD      0xB4
#define CMD_GET_HOR_PERIOD      0xB5
#define CMD_SET_VER_PERIOD      0xB6
#define CMD_GET_VER_PERIOD      0xB7
#define CMD_SET_GPIO_CONF       0xB8
#define CMD_GET_GPIO_CONF       0xB9
#define CMD_SET_GPIO_VAL        0xBA
#define CMD_GET_GPIO_STATUS     0xBB
#define CMD_SET_POST_PROC       0xBC
#define CMD_GET_POST_PROC       0xBD
#define CMD_SET_PWM_CONF        0xBE
#define CMD_GET_PWM_CONF        0xBF
#define CMD_SET_LCD_GEN0        0xC0
#define CMD_GET_LCD_GEN0        0xC1
#define CMD_SET_LCD_GEN1        0xC2
#define CMD_GET_LCD_GEN1        0xC3
#define CMD_SET_LCD_GEN2        0xC4
#define CMD_GET_LCD_GEN2        0xC5
#define CMD_SET_LCD_GEN3        0xC6
#define CMD_GET_LCD_GEN3        0xC7
#define CMD_SET_GPIO0_ROP       0xC8
#define CMD_GET_GPIO0_ROP       0xC9
#define CMD_SET_GPIO1_ROP       0xCA
#define CMD_GET_GPIO1_ROP       0xCB
#define CMD_SET_GPIO2_ROP       0xCC
#define CMD_GET_GPIO2_ROP       0xCD
#define CMD_SET_GPIO3_ROP       0xCE
#define CMD_GET_GPIO3_ROP       0xCF
#define CMD_SET_ABC_DBC_CONF    0xD0
#define CMD_GET_ABC_DBC_CONF    0xD1
#define CMD_SET_DBC_HISTO_PTR   0xD2
#define CMD_GET_DBC_HISTO_PTR   0xD3
#define CMD_SET_DBC_THRES       0xD4
#define CMD_GET_DBC_THRES       0xD5
#define CMD_SET_ABM_TMR         0xD6
#define CMD_GET_ABM_TMR         0xD7
#define CMD_SET_AMB_LVL0        0xD8
#define CMD_GET_AMB_LVL0        0xD9
#define CMD_SET_AMB_LVL1        0xDA
#define CMD_GET_AMB_LVL1        0xDB
#define CMD_SET_AMB_LVL2        0xDC
#define CMD_GET_AMB_LVL2        0xDD
#define CMD_SET_AMB_LVL3        0xDE
#define CMD_GET_AMB_LVL3        0xDF
#define CMD_PLL_START           0xE0    //start the PLL
#define CMD_PLL_STOP            0xE1    //disable the PLL
#define CMD_SET_PLL_MN          0xE2
#define CMD_GET_PLL_MN          0xE3
#define CMD_GET_PLL_STATUS      0xE4    //get the current PLL status
#define CMD_ENT_DEEP_SLEEP      0xE5
#define CMD_SET_PCLK            0xE6    //set pixel clock (LSHIFT signal) frequency
#define CMD_GET_PCLK            0xE7    //get pixel clock (LSHIFT signal) freq. settings
#define CMD_SET_DATA_INTERFACE  0xF0
#define CMD_GET_DATA_INTERFACE  0xF1

//DOM-IGNORE-END

/*********************************************************************
* Function:  void ResetDevice()
*
* Overview: Initializes LCD module.
*
* PreCondition: none
*
* Input: none
*
* Output: none
*
* Side Effects: none
*
********************************************************************/
void ResetDevice(void);


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
*
********************************************************************/
void SetScrollArea(int16_t top, int16_t scroll, int16_t bottom);

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
*       SetScrollArea(0,272,272);
*       for(line=0;line<272;line++) {SetScrollStart(line);DelayMs(100);}
*
*       Code above scrolls the whole page upwards in 100ms interval
*       with page 2 replacing the first page in scrolling
********************************************************************/
void SetScrollStart(int16_t line);

/*********************************************************************
* Function:  void EnterSleepMode (void)
* PreCondition: none
* Input:  none
* Output: none
* Side Effects: none
* Overview: SSD1963 enters sleep mode
* Note: Host must wait 5mS after sending before sending next command
********************************************************************/
void EnterSleepMode (void);

/*********************************************************************
* Function:  void ExitSleepMode (void)
* PreCondition: none
* Input:  none
* Output: none
* Side Effects: none
* Overview: SSD1963 exits sleep mode
* Note:   cannot be called sooner than 15ms
********************************************************************/
void ExitSleepMode (void);

/*********************************************************************
* Function      : void DisplayOff(void)
* PreCondition  : none
* Input         : none
* Output        : none
* Side Effects  : none
* Overview      : SSD1963 changes the display state to OFF state
* Note          : none
********************************************************************/
void DisplayOff(void);

/*********************************************************************
* Function      : void DisplayOn(void)
* PreCondition  : none
* Input         : none
* Output        : none
* Side Effects  : none
* Overview      : SSD1963 changes the display state to ON state
* Note          : none
********************************************************************/
void DisplayOn(void);

/*********************************************************************
* Function      : void EnterDeepSleep(void)
* PreCondition  : none
* Input         : none
* Output        : none
* Side Effects  : none
* Overview      : SSD1963 enters deep sleep state with PLL stopped
* Note          : none
********************************************************************/
void EnterDeepSleep(void);

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
void SetBacklight(BYTE intensity);

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
void SetTearingCfg(BOOL state, BOOL mode);

#ifdef __cplusplus
    }
#endif

#endif /* DRV_GFX_SSD1963_H */