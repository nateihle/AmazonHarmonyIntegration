/*******************************************************************************
 Module for Microchip Graphics Library

  Company:
    Microchip Technology Inc.

  File Name:
    drv_gfx_ssd1289.c

  Summary:
    This contains the source for the SSD1289 timing controller (TCON) driver.
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

#include "system_definitions.h"
#include "system_config.h"

static void wait(uint32_t ms)
{
	uint32_t sysClk = SYS_CLK_FREQ;
    uint32_t t0;
		
	//Primitive Blocking Mode
    if(ms)
    {		
#if defined (__C32__)
        t0 = _CP0_GET_COUNT();
        while (_CP0_GET_COUNT() - t0 < (sysClk / 2000) * ms);
#elif defined (__C30__)
        t0 = ReadTimer23();
        while (ReadTimer23() - t0 < (sysClk / 2000) * ms);
#else
        #error "Neither __C32__ nor __C30__ is defined!"
#endif
    }
}

#define BB_CS                   0x01
#define BB_SCL                  0x02
#define BB_SDO                  0x04
#define BB_DC                   0x08
#define BB_BL                   0x10

<#if CONFIG_DRV_GFX_TCON_SSD_1289_MODE = "LCC">
//#if defined (GFX_USE_DISPLAY_CONTROLLER_LCC)

#define TCON_CSLow()            (BSP_TCON_CSOn())
#define TCON_CSHigh()           (BSP_TCON_CSOff())
#define TCON_CLKLow()           (BSP_TCON_SCLOn())
#define TCON_CLKHigh()          (BSP_TCON_SCLOff())
#define TCON_DataLow()          (BSP_TCON_SDOOn())
#define TCON_DataHigh()         (BSP_TCON_SDOOff())

#define TCON_SetCommand()       (BSP_TCON_DCOn())
#define TCON_SetData()          (BSP_TCON_DCOff())    

<#elseif CONFIG_DRV_GFX_TCON_SSD_1289_MODE = "SSD1926"> 
    
#include "framework/gfx/driver/controller/ssd1926/drv_gfx_ssd1926.h"
	
// use the bitbang version using SSD1926 GPIO pins 
#define TCON_CSLow()            (setIO(BB_CS, 0))
#define TCON_CSHigh()           (setIO(BB_CS, 1))
#define TCON_CLKLow()           (setIO(BB_SCL, 0))
#define TCON_CLKHigh()          (setIO(BB_SCL, 1))
#define TCON_DataLow()          (setIO(BB_SDO, 0))
#define TCON_DataHigh()         (setIO(BB_SDO, 1))

#define TCON_SetCommand()       (setIO(BB_DC, 0))
#define TCON_SetData()          (setIO(BB_DC, 1))

// this is only needed here since the controller IO's are used
// instead of the IO's from the PIC device.
#define SetRegister(addr, data)    while(drvSSD1926_SetRegister(addr, data) == 1)

<#elseif CONFIG_DRV_GFX_TCON_SSD_1289_MODE = "S1D13517"> 

#include "framework/gfx/driver/controller/sd113517/drv_gfx_sd113517.h"

// use the bitbang version using S1D13517 GPIO pins

#define TCON_CSLow()            (setIO(BB_CS, 0))
#define TCON_CSHigh()           (setIO(BB_CS, 1))
#define TCON_CLKLow()           (setIO(BB_SCL, 0))
#define TCON_CLKHigh()          (setIO(BB_SCL, 1))
#define TCON_DataLow()          (setIO(BB_SDO, 0))
#define TCON_DataHigh()         (setIO(BB_SDO, 1))

#define TCON_SetCommand()       (setIO(BB_DC, 0))
#define TCON_SetData()          (setIO(BB_DC, 1))

// this is only needed here since the controller IO's are used
// instead of the IO's from the PIC device.
#define SetRegister(addr, data)    (drvS1D13517_SetRegister(addr, data))

</#if>

static void setIO(uint8_t mask, uint8_t level)
{
<#if CONFIG_DRV_GFX_TCON_SSD_1289_MODE = "LCC">
    switch(mask)
    {
        case BB_CS:
		{
			level == 1 ? BSP_TCON_CSOn() : BSP_TCON_CSOff();
				break;
		}
        case BB_SCL:
		{
			level == 1 ? BSP_TCON_SCLOn() : BSP_TCON_SCLOff();
				break;
		}
        case BB_SDO:
		{
			level == 1 ? BSP_TCON_SDOOn() : BSP_TCON_SDOOff();
                break;
		}
        case BB_DC:
		{
			level == 1 ? BSP_TCON_DCOn() : BSP_TCON_DCOff();
                break;
		}
        default:
		{
            break;            
		}
    }
    
    Nop();
<#elseif CONFIG_DRV_GFX_TCON_SSD_1289_MODE = "SSD1926">
    static uint8_t value = 0xFF;

    if(level == 0)
    {
        value &= ~mask;
    }
    else
    {
        value |= mask;
    }

    SetRegister(SSD1926_REG_GPIO_STATUS_CONTROL0, value);
<#elseif CONFIG_DRV_GFX_TCON_SSD_1289_MODE = "S1D13517"> 
    static uint8_t temp = 0;

    switch(mask)
    {
        case BB_CS:
		{		
            while(temp = drvS1D13517_GetRegister(S1D13517_REG6E_GPO_1) == 0);
			
            if(level == 1)
				temp |= 0x02;
            else
				temp &= 0xFD;       
                    
			while(SetRegister(S1D13517_REG6E_GPO_1, temp));
            
			break;
		}
		case BB_SCL:
		{
            level == 1 ? TCON_CLKHigh() : TCON_CLKLow();
            
			break;
		}
		case BB_SDO:
		{
            level == 1 ? TCON_DataHigh() : TCON_DataLow();
            
			break;
		}
        case BB_DC:    
        {
			while(temp = drvS1D13517_GetRegister(S1D13517_REG6E_GPO_1) == 0);
            
			if(level == 1)
				temp |= 0x04;
            else
				temp &= 0xFB;       
            
			while(SetRegister(S1D13517_REG6E_GPO_1,temp));
            
			break;
		}
    }
    
    Nop();
</#if>
}

// write byte to SPI
static void writeByte(uint8_t value)
{
    uint8_t mask = 0x80;

    while(mask)
    {
        setIO(BB_SCL, 0);
		
        wait(1);
		
        if(mask & value)
            setIO(BB_SDO, 1);
        else
            setIO(BB_SDO, 0);

        setIO(BB_SCL, 1);
		
        mask >>= 1;
    }
}

// write command to SPI
static void writeCommand(uint16_t index, uint16_t value)
{
    setIO(BB_CS, 0);

    // Index
    setIO(BB_DC, 0);
    writeByte(index>>8);
    writeByte(index);

    setIO(BB_CS, 1);
    wait(1);
    setIO(BB_CS, 0);

    // Data
    setIO(BB_DC, 1);
    writeByte(value>>8);
    writeByte(value);
    setIO(BB_CS, 1);
	
    wait(1);
}

// *****************************************************************************
/*
  Function:
     void GFX_TCON_SSD1289Init(void)

  Summary:
     Initialize Solomon Systech ssd1289 Timing Controller

  Description:
     Initialize the IOs to implement Bitbanged SPI interface
     to connect to the Timing Controller through SPI.

  Returns:
     none

*/
void GFX_TCON_SSD1289Init(void)
{
<#if CONFIG_DRV_GFX_TCON_SSD_1289_MODE = "SSD1926">
	// set the GPIO of SSD1926 to as outputs. (used for SSD1289 TCON signals)
    // and initialize them all to "1"
	SetRegister(SSD1926_REG_GPIO_CONFIG0,         0x1F);  
	SetRegister(SSD1926_REG_GPIO_STATUS_CONTROL0, 0x1F);
</#if>
  
 	writeCommand(0x0028, 0x0006);
	writeCommand(0x0000, 0x0001);

    wait(15);

	writeCommand(0x002B, 0x9532);
	writeCommand(0x0003, 0xAAAC);
	writeCommand(0x000C, 0x0002);
	writeCommand(0x000D, 0x000A);
	writeCommand(0x000E, 0x2C00);
	writeCommand(0x001E, 0x00AA);
	writeCommand(0x0025, 0x8000);

    wait(15);

	writeCommand(0x0001, 0x2B3F);
	writeCommand(0x0002, 0x0600);
	writeCommand(0x0010, 0x0000);
	
	wait(20);

	writeCommand(0x0005, 0x0000);
	writeCommand(0x0006, 0x0000);

	writeCommand(0x0016, 0xEF1C);
	writeCommand(0x0017, 0x0003);
	writeCommand(0x0007, 0x0233);
	writeCommand(0x000B, 0x5312);
	writeCommand(0x000F, 0x0000);
	
	wait(20);

	writeCommand(0x0041, 0x0000);
	writeCommand(0x0042, 0x0000);
	writeCommand(0x0048, 0x0000);
	writeCommand(0x0049, 0x013F);
	writeCommand(0x0044, 0xEF00);
	writeCommand(0x0045, 0x0000);
	writeCommand(0x0046, 0x013F);
	writeCommand(0x004A, 0x0000);
	writeCommand(0x004B, 0x0000);
	
	wait(20);

	writeCommand(0x0030, 0x0707);
	writeCommand(0x0031, 0x0704);
	writeCommand(0x0032, 0x0204);
	writeCommand(0x0033, 0x0201);
	writeCommand(0x0034, 0x0203);
	writeCommand(0x0035, 0x0204);
	writeCommand(0x0036, 0x0204);
	writeCommand(0x0037, 0x0502);
	writeCommand(0x003A, 0x0302);
	writeCommand(0x003B, 0x0500);
	
	wait(20);

    TCON_CLKLow();        
}

