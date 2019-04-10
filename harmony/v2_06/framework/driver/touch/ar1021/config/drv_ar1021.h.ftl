<#--
/*******************************************************************************
  AR1021 Driver Freemarker Template File

  Company:
    Microchip Technology Inc.

  File Name:
    drv_ar1021.ftl

  Summary:
    AR1021 Driver Freemarker Template File

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

/*** AR1021 Driver Configuration ***/
<#if CONFIG_USE_DRV_TOUCH_AR1021 == true>
#define DRV_AR1021_INSTANCES_NUMBER                     1
#define DRV_AR1021_CLIENTS_NUMBER                       ${CONFIG_DRV_TOUCH_AR1021_CLIENTS_NUMBER}
#define DRV_AR1021_SPI_CHANNEL_INDEX                    ${("CONFIG_DRV_SPI_SPI_ID_IDX" + "${CONFIG_DRV_AR1021_SPI_INSTANCE}")?eval}
#define DRV_TOUCH_AR1021_REG_TOUCH_THRESHOLD            ${CONFIG_DRV_TOUCH_AR1021_REG_TOUCH_THRESHOLD}
#define DRV_TOUCH_AR1021_REG_SENSITIVITY_FILTER         ${CONFIG_DRV_TOUCH_AR1021_REG_SENSITIVITY_FILTER}
#define DRV_TOUCH_AR1021_REG_SAMPLING_FAST              ${CONFIG_DRV_TOUCH_AR1021_REG_SAMPLING_FAST}
#define DRV_TOUCH_AR1021_REG_SAMPLING_SLOW              ${CONFIG_DRV_TOUCH_AR1021_REG_SAMPLING_SLOW}
#define DRV_TOUCH_AR1021_REG_ACCURACY_FILTER_FAST       ${CONFIG_DRV_TOUCH_AR1021_REG_ACCURACY_FILTER_FAST}
#define DRV_TOUCH_AR1021_REG_ACCURACY_FILTER_SLOW       ${CONFIG_DRV_TOUCH_AR1021_REG_ACCURACY_FILTER_SLOW}
#define DRV_TOUCH_AR1021_REG_SPEED_THRESHOLD            ${CONFIG_DRV_TOUCH_AR1021_REG_SPEED_THRESHOLD}
#define DRV_TOUCH_AR1021_REG_SLEEP_DELAY                ${CONFIG_DRV_TOUCH_AR1021_REG_SLEEP_DELAY}
#define DRV_TOUCH_AR1021_REG_PEN_UP_DELAY               ${CONFIG_DRV_TOUCH_AR1021_REG_PEN_UP_DELAY}
#define DRV_TOUCH_AR1021_REG_TOUCH_MODE                 ${CONFIG_DRV_TOUCH_AR1021_REG_TOUCH_MODE}
#define DRV_TOUCH_AR1021_REG_PEN_STATE_REPORT_DELAY     ${CONFIG_DRV_TOUCH_AR1021_REG_PEN_STATE_REPORT_DELAY}
</#if>

<#--
/*******************************************************************************
 End of File
*/
-->

