<#--
/*******************************************************************************
  CTR Driver Freemarker Template File

  Company:
    Microchip Technology Inc.

  File Name:
    drv_ctr.h.ftl

  Summary:
    CTR Driver Freemarker Template File

  Description:

*******************************************************************************/

/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

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
<#if CONFIG_USE_DRV_CTR == true>
<#if CONFIG_DRV_CTR_DRIVER_MODE == "DYNAMIC">
// *****************************************************************************
/* CTR Driver Configuration Options
*/

#define DRV_CTR_POWER_STATE              ${CONFIG_DRV_CTR_POWER_STATE}
#define DRV_CTR_MODULE_ID                CTR_ID_0 /* To be added to hconfig if more CTR macros are added */
#define DRV_CTR_CLIENTS_NUMBER			 ${CONFIG_DRV_CTR_CLIENTS_NUMBER}
#define DRV_CTR_INSTANCES_NUMBER		 ${CONFIG_DRV_CTR_INSTANCES_NUMBER}
#define DRV_CTR_EVENT_INTERRUPT_SOURCE   ${CONFIG_DRV_CTR_EVENT_INTERRUPT_SOURCE}
<#if CONFIG_DRV_CTR_EVENT_INT_MODE?has_content && CONFIG_DRV_CTR_EVENT_INT_MODE = "Trigger">
#define DRV_CTR_EVENT_INTERRUPT_MODE     CTR_LATCH_TRIG
</#if>
<#if CONFIG_DRV_CTR_EVENT_INT_MODE?has_content && CONFIG_DRV_CTR_EVENT_INT_MODE = "Latch Half-Full">
#define DRV_CTR_EVENT_INTERRUPT_MODE     CTR_BUFFER_HALF
</#if>
<#if CONFIG_DRV_CTR_EVENT_INT_MODE?has_content && CONFIG_DRV_CTR_EVENT_INT_MODE = "Latch Full">
#define DRV_CTR_EVENT_INTERRUPT_MODE     CTR_BUFFER_FULL
</#if>
#define DRV_CTR_TRIGGER_INTERRUPT_SOURCE ${CONFIG_DRV_CTR_TRIGGER_INTERRUPT_SOURCE}
#define DRV_CTR_M_0						 ${CONFIG_DRV_CTR_M_0}
#define DRV_CTR_N_0                      ${CONFIG_DRV_CTR_N_0}
#define DRV_CTR_LSB_0					 ${CONFIG_DRV_CTR_LSB_0}
<#if CONFIG_DRV_CTR_US_MODE_0?has_content && CONFIG_DRV_CTR_US_MODE_0 = "MicroSec Mode">
#define DRV_CTR_MODE_0					 CTR_US
</#if>
<#if CONFIG_DRV_CTR_US_MODE_0?has_content && CONFIG_DRV_CTR_US_MODE_0 = "1394 Mode">
#define DRV_CTR_MODE_0					 CTR_1394
</#if>
#define DRV_CTR_M_1						 ${CONFIG_DRV_CTR_M_1}
#define DRV_CTR_N_1                      ${CONFIG_DRV_CTR_N_1}
#define DRV_CTR_LSB_1					 ${CONFIG_DRV_CTR_LSB_1}
<#if CONFIG_DRV_CTR_US_MODE_1?has_content && CONFIG_DRV_CTR_US_MODE_1 = "MicroSec Mode">
#define DRV_CTR_MODE_1					 CTR_US
</#if>
<#if CONFIG_DRV_CTR_US_MODE_1?has_content && CONFIG_DRV_CTR_US_MODE_1 = "1394 Mode">
#define DRV_CTR_MODE_1					 CTR_1394
</#if>
<#if CONFIG_DRV_CTR_COUNTER_SEL?has_content && CONFIG_DRV_CTR_COUNTER_SEL = "Counter0_us">
#define DRV_CTR_COUNTER_SEL				 CTR_CTR0_US
</#if>
<#if CONFIG_DRV_CTR_COUNTER_SEL?has_content && CONFIG_DRV_CTR_COUNTER_SEL = "Counter0_Lin">
#define DRV_CTR_COUNTER_SEL				 CTR_CTR0_LIN
</#if>
<#if CONFIG_DRV_CTR_COUNTER_SEL?has_content && CONFIG_DRV_CTR_COUNTER_SEL = "Counter1_us">
#define DRV_CTR_COUNTER_SEL				 CTR_CTR1_US
</#if>
<#if CONFIG_DRV_CTR_COUNTER_SEL?has_content && CONFIG_DRV_CTR_COUNTER_SEL = "Counter1_Lin">
#define DRV_CTR_COUNTER_SEL				 CTR_CTR1_LIN
</#if>
#define DRV_CTR_DIVIDER					 ${CONFIG_DRV_CTR_DIVIDER}
<#if CONFIG_DRV_CTR_USE_CASE?has_content && CONFIG_DRV_CTR_USE_CASE == "Wifi">
#define DRIVER_MODE						 WIFI_MODE
#define DRV_CTR_LATCH0_TRIG              CTR_WIFI_TM_1
#define DRV_CTR_LATCH1_TRIG              CTR_WIFI_TM_2
#define DRV_CTR_LATCH2_TRIG              CTR_WIFI_TM_3
#define DRV_CTR_LATCH3_TRIG              CTR_WIFI_TM_4
</#if>
<#if CONFIG_DRV_CTR_USE_CASE?has_content && CONFIG_DRV_CTR_USE_CASE == "USB">
#define DRIVER_MODE						 USB_MODE
#define DRV_CTR_LATCH0_TRIG              CTR_USBSOF
</#if>
<#if CONFIG_DRV_CTR_USE_CASE?has_content && CONFIG_DRV_CTR_USE_CASE == "GPIO">
#define DRIVER_MODE						 GPIO_MODE
<#if CONFIG_DRV_CTR_GPIO_SEL?has_content && CONFIG_DRV_CTR_GPIO_SEL == "CTRTM0">
#define DRV_CTR_LATCH0_TRIG              CTR_GPIO0
</#if>
<#if CONFIG_DRV_CTR_GPIO_SEL?has_content && CONFIG_DRV_CTR_GPIO_SEL == "CTRTM1">
#define DRV_CTR_LATCH0_TRIG              CTR_GPIO1
</#if>
</#if>
<#if CONFIG_DRV_CTR_TRIGGER_SOURCE?has_content && CONFIG_DRV_CTR_TRIGGER_SOURCE = "Counter0_us">
#define DRV_CTR_TRIGGER_SOURCE				 CTR_CTR0_US
</#if>
<#if CONFIG_DRV_CTR_TRIGGER_SOURCE?has_content && CONFIG_DRV_CTR_TRIGGER_SOURCE = "Counter0_Lin">
#define DRV_CTR_TRIGGER_SOURCE				 CTR_CTR0_LIN
</#if>
<#if CONFIG_DRV_CTR_TRIGGER_SOURCE?has_content && CONFIG_DRV_CTR_TRIGGER_SOURCE = "Counter1_us">
#define DRV_CTR_TRIGGER_SOURCE				 CTR_CTR1_US
</#if>
<#if CONFIG_DRV_CTR_TRIGGER_SOURCE?has_content && CONFIG_DRV_CTR_TRIGGER_SOURCE = "Counter1_Lin">
#define DRV_CTR_TRIGGER_SOURCE				 CTR_CTR1_LIN
</#if>
#define DRV_CTR_TRIGGER_PHASE				 ${CONFIG_DRV_CTR_TRIGGER_PHASE}	
</#if>
</#if>

<#--
/*******************************************************************************
 End of File
*/
-->
