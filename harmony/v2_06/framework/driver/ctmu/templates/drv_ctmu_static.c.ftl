/*******************************************************************************
  CTMU Driver Functions for Static Single Instance Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_ctmu_static.c

  Summary:
    CTMU driver implementation for the static single instance driver.

  Description:
    The CTMU device driver provides a simple interface to manage the CTMU
    modules on Microchip microcontrollers.
    
  Remarks:
    Static interfaces incorporate the driver instance number within the names
    of the routines, eliminating the need for an object ID or object handle.
    Static single-open interfaces also eliminate the need for the open handle.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DCTMUUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTCTMUULAR PURPOSE.
IN NO EVENT SHALL MCTMURCTMUHIP OR ITS LCTMUENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRCTMUT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PRCTMUUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVCTMUES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
//DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Header Includes
// *****************************************************************************
// *****************************************************************************
#include "driver/ctmu/drv_ctmu.h"

<#macro DRV_CTMU_STATIC_FUNCTIONS DRV_INSTANCE CTMU_INSTANCE CTMU_EDGE1_SOURCE CTMU_EDGE2_SOURCE>
// *****************************************************************************
// *****************************************************************************
// Section: Instance ${DRV_INSTANCE} static driver functions
// *****************************************************************************
// *****************************************************************************
void DRV_CTMU${DRV_INSTANCE}_Initialize(void)
{	
    /* Setup CTMU${DRV_INSTANCE} Instance */
    //Disable CTMU
    PLIB_CTMU_Disable(${CTMU_INSTANCE});
    
    //Select the current source range.
    PLIB_CTMU_CurrentRangeSet(${CTMU_INSTANCE},${CONFIG_DRV_CTMU_CURRENT_RANGE});

    //Adjust the current source trim.
    PLIB_CTMU_CurrentTrimSet(${CTMU_INSTANCE}, ${CONFIG_DRV_CTMU_CURRENT_TRIM}); // increase by 20% = +10 * 2%
    <#if CONFIG_DRV_CTMU_TIME_PULSE == true>

    //Select the operating mode (Pulse Generation)
    PLIB_CTMU_TimePulseGenerationEnable(${CTMU_INSTANCE}); // Use Comparator Input/ANx pin
    </#if>

    //Disable Edging
    PLIB_CTMU_EdgeDisable(${CTMU_INSTANCE});
    <#if CONFIG_DRV_CTMU_EDGE_SAMPLING == true>

    //Use the same input source for both edges.
    PLIB_CTMU_EdgeTriggerSourceSelect(${CTMU_INSTANCE},CTMU_EDGE1,${CONFIG_DRV_CTMU_EDGE1_SOURCE});
    PLIB_CTMU_EdgeTriggerSourceSelect(${CTMU_INSTANCE},CTMU_EDGE2,${CONFIG_DRV_CTMU_EDGE2_SOURCE});

    //Configure the input sensitivities:
    PLIB_CTMU_EdgeSensitivitySet(${CTMU_INSTANCE},CTMU_EDGE1,${CONFIG_DRV_CTMU_EDGE1_SENSITIVITY});
    PLIB_CTMU_EdgeSensitivitySet(${CTMU_INSTANCE},CTMU_EDGE2,${CONFIG_DRV_CTMU_EDGE2_SENSITIVITY});

    //Configure the input polarities:
    PLIB_CTMU_EdgePolaritySet(${CTMU_INSTANCE},CTMU_EDGE1, ${CONFIG_DRV_CTMU_EDGE1_POLARITY});
    PLIB_CTMU_EdgePolaritySet(${CTMU_INSTANCE},CTMU_EDGE2, ${CONFIG_DRV_CTMU_EDGE2_POLARITY});
    <#if CONFIG_DRV_CTMU_EDGE_SEQUENCING == true>

    //Enable edge sequencing so Edge 1 (rising) occurs before Edge 2 (falling)
    PLIB_CTMU_EdgeSequenceEnable(${CTMU_INSTANCE});
    </#if>

    //Enable edges to control charge pump
    PLIB_CTMU_EdgeEnable(${CTMU_INSTANCE});
    </#if>

    //Turn on the CTMU module, wait 50 us for charge pump to stabilize
    PLIB_CTMU_Enable(${CTMU_INSTANCE});
    
    // TO DO: Wait 50 microseconds

    //Discharge the connected circuit by grounding the charge pump.
    DRV_CTMU0_CurrentSourceGround(true);
}

void DRV_CTMU${DRV_INSTANCE}_Deinitialize(void)
{	
     PLIB_CTMU_Disable(${CTMU_INSTANCE});
}

void DRV_CTMU${DRV_INSTANCE}_Enable(void)
{
     PLIB_CTMU_Enable(${CTMU_INSTANCE});
}

void DRV_CTMU${DRV_INSTANCE}_Disable(void)
{
     PLIB_CTMU_Disable(${CTMU_INSTANCE});
}

void DRV_CTMU${DRV_INSTANCE}_CurrentRangeSet(CTMU_CURRENT_RANGE range)
{
     PLIB_CTMU_CurrentRangeSet(${CTMU_INSTANCE}, range);
}

void DRV_CTMU${DRV_INSTANCE}_CurrentTrimSet(uint16_t trim)
{
     PLIB_CTMU_CurrentTrimSet(${CTMU_INSTANCE}, trim);
}

void DRV_CTMU${DRV_INSTANCE}_CurrentSourceGround(bool onOff)
{

     if(onOff == true)
     {
          PLIB_CTMU_CurrentDischargeDisable(CTMU_ID_0);
     }
     else
     {
          PLIB_CTMU_CurrentDischargeEnable(CTMU_ID_0);
     }
   
}

</#macro>

<#if CONFIG_DRV_CTMU_DRIVER_MODE == "STATIC">
<@DRV_CTMU_STATIC_FUNCTIONS DRV_INSTANCE="0" CTMU_INSTANCE="CTMU_ID_0" 
CTMU_EDGE1_SOURCE=CONFIG_DRV_CTMU_EDGE1_SOURCE CTMU_EDGE2_SOURCE=CONFIG_DRV_CTMU_EDGE2_SOURCE/>
</#if>

/*******************************************************************************
 End of File
*/
