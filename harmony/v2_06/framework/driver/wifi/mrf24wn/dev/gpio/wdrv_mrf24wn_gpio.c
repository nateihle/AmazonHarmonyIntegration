/*******************************************************************************
  MRF24WN Wireless Driver GPIO Interface

  File Name:
    wdrv_mrf24wn_gpio.c

  Summary:
    GPIO interface for MRF24WN wireless driver.

  Description:
    GPIO interface for MRF24WN wireless driver.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc. All rights reserved.

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

#include "system_config.h"

#include "system/ports/sys_ports.h"

#include "driver/wifi/mrf24wn/include/wdrv_mrf24wn_common.h"

typedef void (*GPIO_OUTLOW_T)(uint32_t channel, uint32_t bit_pos);
typedef void (*GPIO_OUTHIGH_T)(uint32_t channel, uint32_t bit_pos);

static bool s_isPowerOff = false;
static GPIO_OUTLOW_T s_GPIOOutLow = NULL;
static GPIO_OUTHIGH_T s_GPIOOutHigh = NULL;

#if WDRV_BOARD_TYPE == WDRV_BD_TYPE_EXP16

static void __GPIO_OutLow_PIC32MX_EXP16(uint32_t channel, uint32_t bit_pos)
{
    SYS_PORTS_PinWrite(PORTS_ID_0, channel, bit_pos, 0);
    SYS_PORTS_PinDirectionSelect(PORTS_ID_0, SYS_PORTS_DIRECTION_OUTPUT, channel, bit_pos);
}

static void __GPIO_OutHigh_PIC32MX_EXP16(uint32_t channel, uint32_t bit_pos)
{
    SYS_PORTS_PinWrite(PORTS_ID_0, channel, bit_pos, 1);
    SYS_PORTS_PinDirectionSelect(PORTS_ID_0, SYS_PORTS_DIRECTION_OUTPUT, channel, bit_pos);
}

#elif WDRV_BOARD_TYPE == WDRV_BD_TYPE_MX_ESK

static void __GPIO_OutLow_PIC32MX_ESK(uint32_t channel, uint32_t bit_pos)
{
    SYS_PORTS_PinWrite(PORTS_ID_0, channel, bit_pos, 0);
    SYS_PORTS_PinDirectionSelect(PORTS_ID_0, SYS_PORTS_DIRECTION_OUTPUT, channel, bit_pos);
}

static void __GPIO_OutHigh_PIC32MX_ESK(uint32_t channel, uint32_t bit_pos)
{
    SYS_PORTS_PinWrite(PORTS_ID_0, channel, bit_pos, 1);
    SYS_PORTS_PinDirectionSelect(PORTS_ID_0, SYS_PORTS_DIRECTION_OUTPUT, channel, bit_pos);
}

#elif WDRV_BOARD_TYPE == WDRV_BD_TYPE_MZ_ESK

static void __GPIO_OutLow_PIC32MZ_ESK(uint32_t channel, uint32_t bit_pos)
{
    PLIB_PORTS_PinModePerPortSelect(PORTS_ID_0, channel, bit_pos, PORTS_PIN_MODE_DIGITAL);
    SYS_PORTS_PinDirectionSelect(PORTS_ID_0, SYS_PORTS_DIRECTION_OUTPUT, channel, bit_pos);
    SYS_PORTS_PinClear(PORTS_ID_0, channel, bit_pos);
}

static void __GPIO_OutHigh_PIC32MZ_ESK(uint32_t channel, uint32_t bit_pos)
{
    PLIB_PORTS_PinModePerPortSelect(PORTS_ID_0, channel, bit_pos, PORTS_PIN_MODE_DIGITAL);
    SYS_PORTS_PinDirectionSelect(PORTS_ID_0, SYS_PORTS_DIRECTION_OUTPUT, channel, bit_pos);
    SYS_PORTS_PinSet(PORTS_ID_0, channel, bit_pos);
}

#elif WDRV_BOARD_TYPE ==  WDRV_BD_TYPE_CUSTOM

static void __GPIO_OutLow_Custom_Board(uint32_t channel, uint32_t bit_pos)
{
#if defined(PLIB_PORTS_ExistsPinModePerPort)
    if (PLIB_PORTS_ExistsPinModePerPort(PORTS_ID_0)) {
        PLIB_PORTS_PinModePerPortSelect(PORTS_ID_0, channel, bit_pos, PORTS_PIN_MODE_DIGITAL);
    }
#endif /* PLIB_PORTS_ExistsPinModePerPort */
    SYS_PORTS_PinDirectionSelect(PORTS_ID_0, SYS_PORTS_DIRECTION_OUTPUT, channel, bit_pos);
    SYS_PORTS_PinWrite(PORTS_ID_0, channel, bit_pos,0);
}

static void __GPIO_OutHigh_Custom_Board(uint32_t channel, uint32_t bit_pos)
{
#if defined(PLIB_PORTS_ExistsPinModePerPort)
    if (PLIB_PORTS_ExistsPinModePerPort(PORTS_ID_0)) {
        PLIB_PORTS_PinModePerPortSelect(PORTS_ID_0, channel, bit_pos, PORTS_PIN_MODE_DIGITAL);
    }
#endif /* PLIB_PORTS_ExistsPinModePerPort */
    SYS_PORTS_PinDirectionSelect(PORTS_ID_0, SYS_PORTS_DIRECTION_OUTPUT, channel, bit_pos);
    SYS_PORTS_PinWrite(PORTS_ID_0, channel, bit_pos,1);
}

#endif /* WDRV_BOARD_TYPE */

static void __GPIO_OutLow(uint32_t channel, uint32_t bit_pos)
{
    WDRV_ASSERT((int)s_GPIOOutLow, "s_GPIOOutLow is not valid");
    s_GPIOOutLow(channel, bit_pos);
}

static void __GPIO_OutHigh(uint32_t channel, uint32_t bit_pos)
{
    WDRV_ASSERT((int)s_GPIOOutHigh, "s_GPIOOutHigh is not valid");
    s_GPIOOutHigh(channel, bit_pos);
}

void WDRV_GPIO_PowerOn(void)
{
    __GPIO_OutHigh(WF_POWER_PORT_CHANNEL, WF_POWER_BIT_POS);
    s_isPowerOff = false;
}

void WDRV_GPIO_PowerOff(void)
{
    __GPIO_OutLow(WF_POWER_PORT_CHANNEL, WF_POWER_BIT_POS);
    s_isPowerOff = true;
}

bool WDRV_IsPowerOff(void)
{
    return s_isPowerOff;
}

void WDRV_GPIO_Init(void)
{
#define GPIO_OUTLOW_FUNC(HW) __GPIO_OutLow_##HW
#define GPIO_OUTHIGH_FUNC(HW) __GPIO_OutHigh_##HW

#if WDRV_BOARD_TYPE == WDRV_BD_TYPE_EXP16
    s_GPIOOutLow = GPIO_OUTLOW_FUNC(PIC32MX_EXP16);
    s_GPIOOutHigh = GPIO_OUTHIGH_FUNC(PIC32MX_EXP16);
#elif WDRV_BOARD_TYPE == WDRV_BD_TYPE_MX_ESK
    s_GPIOOutLow = GPIO_OUTLOW_FUNC(PIC32MX_ESK);
    s_GPIOOutHigh = GPIO_OUTHIGH_FUNC(PIC32MX_ESK);
#elif WDRV_BOARD_TYPE == WDRV_BD_TYPE_MZ_ESK
    s_GPIOOutLow = GPIO_OUTLOW_FUNC(PIC32MZ_ESK);
    s_GPIOOutHigh = GPIO_OUTHIGH_FUNC(PIC32MZ_ESK);
#elif WDRV_BOARD_TYPE ==  WDRV_BD_TYPE_CUSTOM
    s_GPIOOutLow = GPIO_OUTLOW_FUNC(Custom_Board);
    s_GPIOOutHigh = GPIO_OUTHIGH_FUNC(Custom_Board);
#endif
}

void WDRV_GPIO_DeInit(void)
{
    s_GPIOOutLow = NULL;
    s_GPIOOutHigh = NULL;
}

//DOM-IGNORE-END
