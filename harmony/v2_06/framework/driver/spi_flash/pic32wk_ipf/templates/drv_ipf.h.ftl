<#--
/*******************************************************************************
  IPF Driver Freemarker Template File

  Company:
    Microchip Technology Inc.

  File Name:
    drv_ipf.h.ftl

  Summary:
    IPF Driver Freemarker Template File

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
<#if CONFIG_USE_DRV_IPF == true>
<#if CONFIG_DRV_IPF_DRIVER_MODE == "DYNAMIC">
// *****************************************************************************
/* In-Package Flash Driver Configuration Options
*/


#define DRV_IPF_CLIENTS_NUMBER                          ${CONFIG_DRV_IPF_CLIENTS_NUMBER}
#define DRV_IPF_INSTANCES_NUMBER                        ${CONFIG_DRV_IPF_INSTANCES_NUMBER}

#define DRV_IPF_POWER_STATE                     	    ${CONFIG_DRV_IPF_POWER_STATE}
#define DRV_IPF_SPI_DRIVER_INSTANCE           		    ${CONFIG_DRV_IPF_SPI_DRIVER_INSTANCE}
#define DRV_IPF_QUEUE_SIZE                         		${CONFIG_DRV_IPF_QUEUE_SIZE}
#define DRV_IPF_HOLD_PIN_PORT_CHANNEL                   ${CONFIG_DRV_IPF_HOLD_PIN_PORT_CHANNEL}
#define DRV_IPF_HOLD_PIN_PORT_BIT_POS                   ${CONFIG_DRV_IPF_HOLD_PIN_PORT_BIT_POS}
#define DRV_IPF_WRITE_PROTECT_PIN_PORT_CHANNEL          ${CONFIG_DRV_IPF_WRITE_PROTECT_PIN_PORT_CHANNEL}
#define DRV_IPF_WRITE_PROTECT_PIN_BIT_POS               ${CONFIG_DRV_IPF_WRITE_PROTECT_PIN_BIT_POS}
#define DRV_IPF_CHIP_SELECT_PORT_CHANNEL                ${CONFIG_DRV_IPF_CHIP_SELECT_PORT_CHANNEL}
#define DRV_IPF_CHIP_SELECT_PORT_BIT_POS                ${CONFIG_DRV_IPF_CHIP_SELECT_PORT_BIT_POS}
#define DRV_IPF_FS_BASE_ADDRESS							${CONFIG_DRV_IPF_FS_BASE_ADDRESS}

<#if CONFIG_DRV_IPF_USE_FS == true>
#define DRV_IPF_REGISTER_MEDIA
</#if>

<#if CONFIG_DEVICE == "PIC32WK1057GPA132" || CONFIG_DEVICE == "PIC32WK1057GPB132" || CONFIG_DEVICE == "PIC32WK1057GPC132" || CONFIG_DEVICE == "PIC32WK1057GPD132">
#define IPF_RW_NUMBLOCKS (1*1024*1024)
#define IPF_ERASE_NUMBLOCKS (256)
</#if>
<#if CONFIG_DEVICE == "PIC32WK2057GPA132" || CONFIG_DEVICE == "PIC32WK2057GPB132" || CONFIG_DEVICE == "PIC32WK2057GPC132" || CONFIG_DEVICE == "PIC32WK2057GPD132">
#define IPF_RW_NUMBLOCKS (2*1024*1024)
#define IPF_ERASE_NUMBLOCKS (512)
</#if>

</#if>
</#if>

<#--
/*******************************************************************************
 End of File
*/
-->
