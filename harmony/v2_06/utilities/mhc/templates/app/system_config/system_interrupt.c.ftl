/*******************************************************************************
 System Interrupts File

  File Name:
    system_interrupt.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

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

<#include "/utilities/mhc/templates/freemarker_functions.ftl">
// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system/common/sys_common.h"
<#if LIST_SYSTEM_INTERRUPT_C_INCLUDES?has_content>
<@mhc_expand_list list=LIST_SYSTEM_INTERRUPT_C_INCLUDES/>
</#if>
<#if CONFIG_APP_IDX_0?has_content>
#include "${CONFIG_APP_NAME_0?lower_case}.h"
<#else>
#include "app.h"
</#if>
<#if CONFIG_APP_IDX_1?has_content>
<#if CONFIG_APP_IDX_1 == true>
#include "${CONFIG_APP_NAME_1?lower_case}.h"
</#if>
<#if CONFIG_APP_IDX_2 == true>
#include "${CONFIG_APP_NAME_2?lower_case}.h"
</#if>
<#if CONFIG_APP_IDX_3 == true>
#include "${CONFIG_APP_NAME_3?lower_case}.h"
</#if>
<#if CONFIG_APP_IDX_4 == true>
#include "${CONFIG_APP_NAME_4?lower_case}.h"
</#if>
<#if CONFIG_APP_IDX_5 == true>
#include "${CONFIG_APP_NAME_5?lower_case}.h"
</#if>
<#if CONFIG_APP_IDX_6 == true>
#include "${CONFIG_APP_NAME_6?lower_case}.h"
</#if>
<#if CONFIG_APP_IDX_7 == true>
#include "${CONFIG_APP_NAME_7?lower_case}.h"
</#if>
<#if CONFIG_APP_IDX_8 == true>
#include "${CONFIG_APP_NAME_8?lower_case}.h"
</#if>
<#if CONFIG_APP_IDX_9 == true>
#include "${CONFIG_APP_NAME_9?lower_case}.h"
</#if>
</#if>
#include "system_definitions.h"

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************
<#if LIST_SYSTEM_INTERRUPT_C_VECTORS?has_content>
<@mhc_expand_list list=LIST_SYSTEM_INTERRUPT_C_VECTORS/>
</#if>
<#if (CONFIG_3RDPARTY_RTOS_USED == "ThreadX")>
<#include "/third_party/rtos/ThreadX/templates/threadX_int.c.ftl">
</#if>
<#if CONFIG_USE_DRV_ADC == true>
<#include "/framework/driver/adc/templates/drv_adc_int.c.ftl">
</#if>
<#if CONFIG_USE_DRV_CTR == true>
<#include "/framework/driver/ctr/templates/drv_ctr_int.c.ftl">
</#if>
<#if CONFIG_USE_DRV_PTG == true>
<#include "/framework/driver/ptg/templates/drv_ptg_int.c.ftl">
</#if>
<#if CONFIG_USE_EXT_INT?has_content><#if CONFIG_USE_EXT_INT == true>
<#include "/framework/system/int/templates/ext_int_static_int.c.ftl">
</#if> </#if>
<#if CONFIG_USE_DRV_TMR == true>
<#include "/framework/driver/tmr/templates/drv_tmr_int.c.ftl">
</#if>
<#if CONFIG_USE_TEST_HARNESS == true>
<#include "/framework/test/templates/system_interrupt.c.ftl">
</#if>
<#if CONFIG_SAMPLE_MODULE_INTERRUPT_MODE == true>
<#include "/framework/sample/templates/system_interrupt.c.ftl">
</#if>
<#if CONFIG_DRV_SPI_USE_DRIVER == true>
<#include "/framework/driver/spi/config/drv_spi_int.c.ftl">
</#if>
<#if CONFIG_USE_DRV_NVM == true>
<#include "/framework/driver/nvm/config/drv_nvm_int.c.ftl">
</#if>
<#if CONFIG_USE_DRV_SDHC == true>
<#include "/framework/driver/sdhc/templates/drv_sdhc_int.c.ftl">
</#if>
<#if CONFIG_USE_DRV_IC == true>
<#include "/framework/driver/ic/templates/drv_ic_int.c.ftl">
</#if>
<#if CONFIG_USE_DRV_CMP == true>
<#include "/framework/driver/cmp/templates/drv_cmp_int.c.ftl">
</#if>
<#if CONFIG_USE_DRV_MCPWM == true>
<#include "/framework/driver/mcpwm/templates/drv_mcpwm_int.c.ftl">
</#if>
<#if CONFIG_USE_DRV_OC == true>
<#include "/framework/driver/oc/templates/drv_oc_int.c.ftl">
</#if>
<#if CONFIG_USE_DRV_CAN == true>
<#include "/framework/driver/can/templates/drv_can_int.c.ftl">
</#if>
<#if CONFIG_USE_DRV_I2S == true>
<#include "/framework/driver/i2s/templates/drv_i2s_int.c.ftl">
</#if>
<#if CONFIG_USE_DRV_RTCC == true>
<#include "/framework/driver/rtcc/templates/drv_rtcc_int.c.ftl">
</#if>
<#if CONFIG_USE_USB_STACK == true>
<#include "/framework/usb/templates/usb_interrupt.c.ftl">
</#if>
<#if CONFIG_USE_TCPIP_STACK == true>
<#include "/framework/tcpip/config/tcpip_mac_int.c.ftl">
</#if>
<#include "/framework/net/templates/system_interrupt.c.ftl">
/*******************************************************************************
 End of File
*/
