<#--
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
/******************************************************************************
  Function:
    static void ${APP_NAME?upper_case}_Bootloader_ForceEvent (void)
    
   Remarks:
    Sets a trigger to be passed to force bootloader callback.
    Run bootloader if switch_1 is pressed OR
    if memory location == '0xFFFFFFFF' otherwise jump to user 
    application.
*/ 
int ${APP_NAME?upper_case}_Bootloader_ForceEvent(void)
{
    /* Check the switch press to trigger bootloader */
    if (BSP_SWITCH_STATE_PRESSED == BSP_SwitchStateGet(BTL_TRIGGER_SWITCH))
    {
        return (1);
    }

    /* Check the trigger memory location and return true/false. */
    if (*(uint32_t *)APP_RESET_ADDRESS == 0xFFFFFFFF)
        return (1);
    
    return (0);
}
<#--
/*******************************************************************************
 End of File
*/
-->
