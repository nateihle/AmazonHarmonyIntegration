/*******************************************************************************
  WINC1500 Wireless Driver External Interrupt Handler

  File Name:
    winc1500_eint.c

  Summary:
    External interrupt handler for WINC1500 wireless driver.

  Description:
    External interrupt handler for WINC1500 wireless driver.
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

#include "system/int/sys_int.h"
#include "system/ports/sys_ports.h"

#include "driver/wifi/winc1500/include/wdrv_winc1500_api.h"

static void (*s_isr)(void) = NULL;

void WDRV_STUB_INTR_SourceEnable(void)
{
    bool pinStatus = SYS_PORTS_PinRead(PORTS_ID_0, WDRV_IRQN_PORT_CHANNEL, WDRV_IRQN_BIT_POS);

    // If INT line is low, we might have missed an interrupt, hence force an interrupt to
    // clear the status
    if (pinStatus == 0) {
#if defined(PLIB_INT_ExistsSourceFlag)
        if (PLIB_INT_ExistsSourceFlag(INT_ID_0)) {
            PLIB_INT_SourceFlagSet(INT_ID_0, WINC1500_INT_SOURCE);
        }
#endif
    }

    /* enable the external interrupt */
    SYS_INT_SourceEnable(WINC1500_INT_SOURCE);
}

void WDRV_STUB_INTR_SourceDisable(void)
{
    SYS_INT_SourceDisable(WINC1500_INT_SOURCE);
}

void WDRV_STUB_INTR_Init(void (*isr)(void))
{
    /* disable the external interrupt */
    SYS_INT_SourceDisable(WINC1500_INT_SOURCE);

    /* clear and enable the interrupt */
    SYS_INT_SourceStatusClear(WINC1500_INT_SOURCE); // clear status

    s_isr = isr;
}

void WDRV_STUB_INTR_Deinit(void)
{
    SYS_INT_SourceDisable(WINC1500_INT_SOURCE);
    s_isr = NULL;
}

void WDRV_WINC1500_ISR(void)
{
    WDRV_STUB_INTR_SourceDisable(); // disable further interrupts
    if (s_isr)
        s_isr();
}

//DOM-IGNORE-END
