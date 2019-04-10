<#--
/*******************************************************************************
  Wi-Fi Driver Interrupt Handler Template File

  File Name:
    drv_wifi_int.c

  Summary:
    This file contains Wi-Fi driver interrupt handler source code.

  Description:
    It generates code that is added to system_interrupt.c in order to handle
    Wi-Fi driver interrupts.
 *******************************************************************************/

/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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
 -->
<#if CONFIG_USE_DRV_WIFI_WK!false == true>
extern void mac_isr(unsigned int vector);
extern void timer_tick_isr(unsigned int param);
volatile unsigned int g_isTimerInterrupt_contionous = 0;
void __ISR(_RFMAC_VECTOR, ipl1AUTO0) _IntHandlerRfMacInstance0(void)
{
    mac_isr(1);
    IFS2bits.RFMACIF = 0;
}

void __ISR(_RFTM0_VECTOR, ipl1AUTO0) _IntHandlerRfTimer0Instance0(void)
{
    timer_tick_isr(0);
    g_isTimerInterrupt_contionous++;
    IFS2bits.RFTM0IF = 0;
}
</#if>
<#--
/*******************************************************************************
 End of File
*/
-->
