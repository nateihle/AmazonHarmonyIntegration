<#--
/*******************************************************************************
  SST26 Driver Freemarker Template File

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sst26.h.ftl

  Summary:
    SST26 Driver Freemarker Template File

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

/*** SST26 Driver Configuration ***/
<#if CONFIG_USE_DRV_SST26 == true>
<#-- SST26 Driver Defines -->
<#assign totalClients = 0>
<#assign totalBufferObjects = 0>
<#if CONFIG_DRV_SST26_CLIENTS_NUMBER_IDX0?has_content>
<#assign totalClients = totalClients + CONFIG_DRV_SST26_CLIENTS_NUMBER_IDX0?number>
</#if>
<#if CONFIG_DRV_SST26_CLIENTS_NUMBER_IDX1?has_content>
<#assign totalClients = totalClients + CONFIG_DRV_SST26_CLIENTS_NUMBER_IDX1?number>
</#if>
<#if CONFIG_DRV_SST26_BUFFER_OBJECT_NUMBER_IDX0?has_content>
<#assign totalBufferObjects = totalBufferObjects + CONFIG_DRV_SST26_BUFFER_OBJECT_NUMBER_IDX0?number>
</#if>
<#if CONFIG_DRV_SST26_BUFFER_OBJECT_NUMBER_IDX1?has_content>
<#assign totalBufferObjects = totalBufferObjects + CONFIG_DRV_SST26_BUFFER_OBJECT_NUMBER_IDX1?number>
</#if>
#define DRV_SST26_INSTANCES_NUMBER     	${CONFIG_DRV_SST26_INSTANCES_NUMBER}
#define DRV_SST26_CLIENTS_NUMBER        ${totalClients}
#define DRV_SST26_BUFFER_OBJECT_NUMBER  ${totalBufferObjects}
<#if CONFIG_USE_DRV_SST26_SYS_FS_REGISTER == true>
#define DRV_SST26_SYS_FS_REGISTER
</#if>
</#if>

<#--
/*******************************************************************************
 End of File
*/
-->

