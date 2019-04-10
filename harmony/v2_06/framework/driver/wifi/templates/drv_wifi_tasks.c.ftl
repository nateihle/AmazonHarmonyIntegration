<#--
/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

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
<#if !(DRV_WIFI_TASKS?has_content)>
<#assign DRV_WIFI_TASKS = "TASK_CALL_NO_RTOS">
</#if>
<#if DRV_WIFI_TASKS == "CREATE_TASK">
 <#if WILC1000_RTOS_TASK == "Standalone">
    /* Create task for WILC1000 Wi-Fi driver */
<@RTOS_TASK_CREATE RTOS_NAME=CONFIG_3RDPARTY_RTOS_USED TASK_FUNC_NAME="_DRV_WIFI_Tasks" TASK_NAME="DRV_WIFI_Tasks" TASK_PRI=CONFIG_WILC1000_RTOS_TASK_PRIORITY TASK_STK_SZ=CONFIG_WILC1000_RTOS_TASK_SIZE/>
 </#if>
</#if>
<#if (DRV_WIFI_TASKS == "TASK_CALL_NO_RTOS") || (DRV_WIFI_TASKS == "TASK_CALL" && WILC1000_RTOS_TASK != "Standalone")>
    /* WILC1000 Device layer tasks routine */
    WDRV_WILC1000_Tasks2();
</#if>
<#if DRV_WIFI_TASKS == "LOCAL_FUNCTION">
<#if WILC1000_RTOS_TASK == "Standalone">
void _DRV_WIFI_Tasks(void)
</#if>
</#if>
<#--
/*******************************************************************************
 End of File
 */
-->
