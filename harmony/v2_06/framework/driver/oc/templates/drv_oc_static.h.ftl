/*******************************************************************************
  OC Driver Interface Declarations for Static Single Instance Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_oc_static.h

  Summary:
    OC driver interface declarations for the static single instance driver.

  Description:
    The OC device driver provides a simple interface to manage the OC
    modules on Microchip microcontrollers. This file defines the interface
    Declarations for the OC driver.
    
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

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTOCULAR PURPOSE.
IN NO EVENT SHALL MOCROCHIP OR ITS LOCENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STROCT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVOCES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
//DOM-IGNORE-END

#ifndef _DRV_OC_STATIC_H
#define _DRV_OC_STATIC_H

<#macro DRV_OC_STATIC_API DRV_INSTANCE OC_MODES>
// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for Instance ${DRV_INSTANCE} for the static driver
// *****************************************************************************
// *****************************************************************************
void DRV_OC${DRV_INSTANCE}_Initialize(void);
void DRV_OC${DRV_INSTANCE}_Enable(void);
void DRV_OC${DRV_INSTANCE}_Disable(void);
void DRV_OC${DRV_INSTANCE}_Start(void);
void DRV_OC${DRV_INSTANCE}_Stop(void);
bool DRV_OC${DRV_INSTANCE}_FaultHasOccurred(void);
<#if OC_MODES == "OC_SET_HIGH_SINGLE_PULSE_MODE" || OC_MODES == "OC_SET_LOW_SINGLE_PULSE_MODE" || OC_MODES == "OC_TOGGLE_CONTINUOUS_PULSE_MODE">
void DRV_OC${DRV_INSTANCE}_CompareValuesSingleSet(uint32_t compareValue);
</#if>
<#if OC_MODES == "OC_DUAL_COMPARE_SINGLE_PULSE_MODE" || OC_MODES == "OC_DUAL_COMPARE_CONTINUOUS_PULSE_MODE">
void DRV_OC${DRV_INSTANCE}_CompareValuesDualSet(uint32_t priVal, uint32_t secVal);
</#if>
<#if OC_MODES == "OC_COMPARE_PWM_MODE_WITHOUT_FAULT_PROTECTION" || OC_MODES == "OC_COMPARE_PWM_MODE_WITH_FAULT_PROTECTION" || OC_MODES == "OC_COMPARE_PWM_EDGE_ALIGNED_MODE">
void DRV_OC${DRV_INSTANCE}_PulseWidthSet(uint32_t pulseWidth);
</#if>
</#macro>

#define DRV_OC_Open(drvIndex, intent) (drvIndex)
#define DRV_OC_Close(handle)


<#if CONFIG_DRV_OC_INST_IDX0 == true>
<@DRV_OC_STATIC_API DRV_INSTANCE="0" 
OC_MODES=CONFIG_DRV_OC_COMPARE_MODES_IDX0/>
</#if>
<#if CONFIG_DRV_OC_INST_IDX1 == true>
<@DRV_OC_STATIC_API DRV_INSTANCE="1" 
OC_MODES=CONFIG_DRV_OC_COMPARE_MODES_IDX1/>
</#if>
<#if CONFIG_DRV_OC_INST_IDX2 == true>
<@DRV_OC_STATIC_API DRV_INSTANCE="2" 
OC_MODES=CONFIG_DRV_OC_COMPARE_MODES_IDX2/>
</#if>
<#if CONFIG_DRV_OC_INST_IDX3 == true>
<@DRV_OC_STATIC_API DRV_INSTANCE="3" 
OC_MODES=CONFIG_DRV_OC_COMPARE_MODES_IDX3/>
</#if>
<#if CONFIG_DRV_OC_INST_IDX4 == true>
<@DRV_OC_STATIC_API DRV_INSTANCE="4" 
OC_MODES=CONFIG_DRV_OC_COMPARE_MODES_IDX4/>
</#if>
<#if CONFIG_DRV_OC_INST_IDX5 == true>
<@DRV_OC_STATIC_API DRV_INSTANCE="5" 
OC_MODES=CONFIG_DRV_OC_COMPARE_MODES_IDX5/>
</#if>
<#if CONFIG_DRV_OC_INST_IDX6 == true>
<@DRV_OC_STATIC_API DRV_INSTANCE="6" 
OC_MODES=CONFIG_DRV_OC_COMPARE_MODES_IDX6/>
</#if>
<#if CONFIG_DRV_OC_INST_IDX7 == true>
<@DRV_OC_STATIC_API DRV_INSTANCE="7" 
OC_MODES=CONFIG_DRV_OC_COMPARE_MODES_IDX7/>
</#if>
<#if CONFIG_DRV_OC_INST_IDX8 == true>
<@DRV_OC_STATIC_API DRV_INSTANCE="8" 
OC_MODES=CONFIG_DRV_OC_COMPARE_MODES_IDX8/>
</#if>
<#if CONFIG_DRV_OC_INST_IDX9 == true>
<@DRV_OC_STATIC_API DRV_INSTANCE="9" 
OC_MODES=CONFIG_DRV_OC_COMPARE_MODES_IDX9/>
</#if>
<#if CONFIG_DRV_OC_INST_IDX10 == true>
<@DRV_OC_STATIC_API DRV_INSTANCE="10" 
OC_MODES=CONFIG_DRV_OC_COMPARE_MODES_IDX10/>
</#if>
<#if CONFIG_DRV_OC_INST_IDX11 == true>
<@DRV_OC_STATIC_API DRV_INSTANCE="11" 
OC_MODES=CONFIG_DRV_OC_COMPARE_MODES_IDX11/>
</#if>
<#if CONFIG_DRV_OC_INST_IDX12 == true>
<@DRV_OC_STATIC_API DRV_INSTANCE="12" 
OC_MODES=CONFIG_DRV_OC_COMPARE_MODES_IDX12/>
</#if>
<#if CONFIG_DRV_OC_INST_IDX13 == true>
<@DRV_OC_STATIC_API DRV_INSTANCE="13" 
OC_MODES=CONFIG_DRV_OC_COMPARE_MODES_IDX13/>
</#if>
<#if CONFIG_DRV_OC_INST_IDX14 == true>
<@DRV_OC_STATIC_API DRV_INSTANCE="14" 
OC_MODES=CONFIG_DRV_OC_COMPARE_MODES_IDX14/>
</#if>
<#if CONFIG_DRV_OC_INST_IDX15 == true>
<@DRV_OC_STATIC_API DRV_INSTANCE="15" 
OC_MODES=CONFIG_DRV_OC_COMPARE_MODES_IDX15/>
</#if>
#endif // #ifndef _DRV_OC_STATIC_H

/*******************************************************************************
 End of File
*/
