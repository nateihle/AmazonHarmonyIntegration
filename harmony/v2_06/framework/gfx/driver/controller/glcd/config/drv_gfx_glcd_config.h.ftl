<#--
/*******************************************************************************
  Driver Freemarker Template File

  Company:
    Microchip Technology Inc.

  File Name:

  Summary:
    Driver Freemarker Template File

  Description:

*******************************************************************************/

/*******************************************************************************
Copyright (c) 2015 released Microchip Technology Inc.  All rights reserved.

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
/*** GLCD Driver Configuration ***/
<#if CONFIG_USE_DRV_GFX_GLCD == true>
<#if CONFIG_DRV_GFX_GLCD_ENABLE_ALL_RGB_PINS>
#define  GFX_GLCD_CONFIG_CONTROL                       0x80000000
<#else>
#define  GFX_GLCD_CONFIG_CONTROL                       0xC0000000
</#if>
#define  GFX_GLCD_BACKGROUND_COLOR                     ${CONFIG_DRV_GFX_GLCD_BACKGROUND_COLOR}
#define  GFX_GLCD_LAYERS                               ${CONFIG_DRV_GFX_GLCD_LAYERS_NUMBER}
<#if CONFIG_DRV_GFX_GLCD_LAYER_0 == true>
/*** GLCD Layer 0 Configuration ***/
#define  GFX_GLCD_LAYER0_BASEADDR                      0xA8000000
#define  GFX_GLCD_LAYER0_DBL_BASEADDR                  0xA8465000
</#if>
<#if CONFIG_DRV_GFX_GLCD_LAYER_1 == true>
/*** GLCD Layer 1 Configuration ***/
#define  GFX_GLCD_LAYER1_BASEADDR                      0xA8177000
#define  GFX_GLCD_LAYER1_DBL_BASEADDR                  0xA85DC000
</#if>
<#if CONFIG_DRV_GFX_GLCD_LAYER_2 == true>
/*** GLCD Layer 2 Configuration ***/
#define  GFX_GLCD_LAYER2_BASEADDR                      0xA82EE000
#define  GFX_GLCD_LAYER2_DBL_BASEADDR                  0xA8753000
</#if>
</#if>

<#--
/*******************************************************************************
 End of File
*/
-->

