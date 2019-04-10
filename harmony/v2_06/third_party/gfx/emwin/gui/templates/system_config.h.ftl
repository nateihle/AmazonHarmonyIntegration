<#--
/*******************************************************************************
  SEGGER emWin Freemarker Template File for MPLAB Harmony

  Company:
    Microchip Technology Inc.

  File Name:
    system_config.h.ftl

  Summary:
    SEGGER emWin Freemarker Template File for MPLAB Harmony

  Description:

*******************************************************************************/

/*******************************************************************************
Copyright (c) 2016 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

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
-->
<#if CONFIG_USE_SEGGER_EMWIN_LIBRARY == true>
<#if CONFIG_SEGGER_EMWIN_MEM_BLOCK_SIZE?has_content>
#define EMWIN_GUI_MEM_BLOCK_SIZE           ${CONFIG_SEGGER_EMWIN_MEM_BLOCK_SIZE}
</#if>
<#if CONFIG_SEGGER_EMWIN_GUI_NUM_SCREENS?has_content>
#define EMWIN_GUI_NUM_SCREENS              ${CONFIG_SEGGER_EMWIN_GUI_NUM_SCREENS}
</#if>
</#if>

