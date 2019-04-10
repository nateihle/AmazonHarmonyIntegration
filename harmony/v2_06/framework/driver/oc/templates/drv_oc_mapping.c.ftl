/*******************************************************************************
  OC Driver Dynamic to Static mapping

  Company:
    Microchip Technology Inc.

  File Name:
    drv_oc_mapping.c

  Summary:
    Source code for the OC driver dynamic APIs to static API mapping.

  Description:
    This file contains code that maps dynamic APIs to static whenever
    the static mode of the driver is selected..

  Remarks:
    Static interfaces incorporate the driver instance number within the names
    of the routines, eliminating the need for an object ID or object handle.

    Static single-open interfaces also eliminate the need for the open handle.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2015 released Microchip Technology Inc.  All rights reserved.

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

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "system_config.h"
#include "system_definitions.h"


SYS_MODULE_OBJ DRV_OC_Initialize(const SYS_MODULE_INDEX index,const SYS_MODULE_INIT * const init)
{
    SYS_MODULE_OBJ returnValue = index;

    switch(index)
    {
<#if CONFIG_DRV_OC_INST_IDX0 == true>
        case DRV_OC_INDEX_0:
        {
            DRV_OC0_Initialize();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX1 == true>
        case DRV_OC_INDEX_1:
        {
            DRV_OC1_Initialize();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX2 == true>
        case DRV_OC_INDEX_2:
        {
            DRV_OC2_Initialize();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX3 == true>
        case DRV_OC_INDEX_3:
        {
            DRV_OC3_Initialize();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX4 == true>
        case DRV_OC_INDEX_4:
        {
            DRV_OC4_Initialize();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX5 == true>
        case DRV_OC_INDEX_5:
        {
            DRV_OC5_Initialize();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX6 == true>
        case DRV_OC_INDEX_6:
        {
            DRV_OC6_Initialize();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX7 == true>
        case DRV_OC_INDEX_7:
        {
            DRV_OC7_Initialize();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX8 == true>
        case DRV_OC_INDEX_8:
        {
            DRV_OC8_Initialize();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX9 == true>
        case DRV_OC_INDEX_9:
        {
            DRV_OC9_Initialize();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX10 == true>
        case DRV_OC_INDEX_10:
        {
            DRV_OC10_Initialize();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX11 == true>
        case DRV_OC_INDEX_11:
        {
            DRV_OC11_Initialize();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX12 == true>
        case DRV_OC_INDEX_12:
        {
            DRV_OC12_Initialize();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX13 == true>
        case DRV_OC_INDEX_13:
        {
            DRV_OC13_Initialize();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX14 == true>
        case DRV_OC_INDEX_14:
        {
            DRV_OC14_Initialize();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX15 == true>
        case DRV_OC_INDEX_15:
        {
            DRV_OC15_Initialize();
            break;
        }
</#if>
        default:
        {
            returnValue = SYS_MODULE_OBJ_INVALID;
            break;
        }
    }
    return returnValue;
}

DRV_HANDLE DRV_OC_Start(const SYS_MODULE_INDEX drvIndex, const DRV_IO_INTENT intent)
{
    SYS_MODULE_OBJ returnValue = drvIndex;

    switch(drvIndex)
    {
<#if CONFIG_DRV_OC_INST_IDX0 == true>
        case DRV_OC_INDEX_0:
        {
            DRV_OC0_Start();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX1 == true>
        case DRV_OC_INDEX_1:
        {
            DRV_OC1_Start();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX2 == true>
        case DRV_OC_INDEX_2:
        {
            DRV_OC2_Start();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX3 == true>
        case DRV_OC_INDEX_3:
        {
            DRV_OC3_Start();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX4 == true>
        case DRV_OC_INDEX_4:
        {
            DRV_OC4_Start();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX5 == true>
        case DRV_OC_INDEX_5:
        {
            DRV_OC5_Start();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX6 == true>
        case DRV_OC_INDEX_6:
        {
            DRV_OC6_Start();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX7 == true>
        case DRV_OC_INDEX_7:
        {
            DRV_OC7_Start();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX8 == true>
        case DRV_OC_INDEX_8:
        {
            DRV_OC8_Start();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX9 == true>
        case DRV_OC_INDEX_9:
        {
            DRV_OC9_Start();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX10 == true>
        case DRV_OC_INDEX_10:
        {
            DRV_OC10_Start();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX11 == true>
        case DRV_OC_INDEX_11:
        {
            DRV_OC11_Start();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX12 == true>
        case DRV_OC_INDEX_12:
        {
            DRV_OC12_Start();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX13 == true>
        case DRV_OC_INDEX_13:
        {
            DRV_OC13_Start();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX14 == true>
        case DRV_OC_INDEX_14:
        {
            DRV_OC14_Start();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX15 == true>
        case DRV_OC_INDEX_15:
        {
            DRV_OC15_Start();
            break;
        }
</#if>
        default:
        {
            returnValue = SYS_MODULE_OBJ_INVALID;
            break;
        }
    }
    return returnValue;
}


void DRV_OC_Stop(DRV_HANDLE handle)
{
    switch(handle)
    {
<#if CONFIG_DRV_OC_INST_IDX0 == true>
        case DRV_OC_INDEX_0:
        {
            DRV_OC0_Stop();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX1 == true>
        case DRV_OC_INDEX_1:
        {
            DRV_OC1_Stop();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX2 == true>
        case DRV_OC_INDEX_2:
        {
            DRV_OC2_Stop();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX3 == true>
        case DRV_OC_INDEX_3:
        {
            DRV_OC3_Stop();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX4 == true>
        case DRV_OC_INDEX_4:
        {
            DRV_OC4_Stop();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX5 == true>
        case DRV_OC_INDEX_5:
        {
            DRV_OC5_Stop();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX6 == true>
        case DRV_OC_INDEX_6:
        {
            DRV_OC6_Stop();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX7 == true>
        case DRV_OC_INDEX_7:
        {
            DRV_OC7_Stop();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX8 == true>
        case DRV_OC_INDEX_8:
        {
            DRV_OC8_Stop();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX9 == true>
        case DRV_OC_INDEX_9:
        {
            DRV_OC9_Stop();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX10 == true>
        case DRV_OC_INDEX_10:
        {
            DRV_OC10_Stop();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX11 == true>
        case DRV_OC_INDEX_11:
        {
            DRV_OC11_Stop();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX12 == true>
        case DRV_OC_INDEX_12:
        {
            DRV_OC12_Stop();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX13 == true>
        case DRV_OC_INDEX_13:
        {
            DRV_OC13_Stop();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX14 == true>
        case DRV_OC_INDEX_14:
        {
            DRV_OC14_Stop();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX15 == true>
        case DRV_OC_INDEX_15:
        {
            DRV_OC15_Stop();
            break;
        }
</#if>
        default:
        {
            break;
        }
    }
}

void DRV_OC_CompareValuesSingleSet(DRV_HANDLE handle, uint32_t compareValue)
{
    /* This API is supported only when selected instance of the OC driver is 
     * configured for the Single Compare match modes. */

    switch(handle)
    {
<#if CONFIG_DRV_OC_INST_IDX0 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX0 == "OC_SET_HIGH_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX0 == "OC_SET_LOW_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX0 == "OC_TOGGLE_CONTINUOUS_PULSE_MODE">
        case DRV_OC_INDEX_0:
        {
            DRV_OC0_CompareValuesSingleSet(compareValue);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX1 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX1 == "OC_SET_HIGH_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX1 == "OC_SET_LOW_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX1 == "OC_TOGGLE_CONTINUOUS_PULSE_MODE">
        case DRV_OC_INDEX_1:
        {
            DRV_OC1_CompareValuesSingleSet(compareValue);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX2 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX2 == "OC_SET_HIGH_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX2 == "OC_SET_LOW_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX2 == "OC_TOGGLE_CONTINUOUS_PULSE_MODE">
        case DRV_OC_INDEX_2:
        {
            DRV_OC2_CompareValuesSingleSet(compareValue);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX3 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX3 == "OC_SET_HIGH_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX3 == "OC_SET_LOW_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX3 == "OC_TOGGLE_CONTINUOUS_PULSE_MODE">
        case DRV_OC_INDEX_3:
        {
            DRV_OC3_CompareValuesSingleSet(compareValue);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX4 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX4 == "OC_SET_HIGH_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX4 == "OC_SET_LOW_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX4 == "OC_TOGGLE_CONTINUOUS_PULSE_MODE">
        case DRV_OC_INDEX_4:
        {
            DRV_OC4_CompareValuesSingleSet(compareValue);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX5 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX5 == "OC_SET_HIGH_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX5 == "OC_SET_LOW_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX5 == "OC_TOGGLE_CONTINUOUS_PULSE_MODE">
        case DRV_OC_INDEX_5:
        {
            DRV_OC5_CompareValuesSingleSet(compareValue);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX6 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX6 == "OC_SET_HIGH_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX6 == "OC_SET_LOW_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX6 == "OC_TOGGLE_CONTINUOUS_PULSE_MODE">
        case DRV_OC_INDEX_6:
        {
            DRV_OC6_CompareValuesSingleSet(compareValue);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX7 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX7 == "OC_SET_HIGH_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX7 == "OC_SET_LOW_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX7 == "OC_TOGGLE_CONTINUOUS_PULSE_MODE">
        case DRV_OC_INDEX_7:
        {
            DRV_OC7_CompareValuesSingleSet(compareValue);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX8 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX8 == "OC_SET_HIGH_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX8 == "OC_SET_LOW_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX8 == "OC_TOGGLE_CONTINUOUS_PULSE_MODE">
        case DRV_OC_INDEX_8:
        {
            DRV_OC8_CompareValuesSingleSet(compareValue);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX9 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX9 == "OC_SET_HIGH_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX9 == "OC_SET_LOW_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX9 == "OC_TOGGLE_CONTINUOUS_PULSE_MODE">
        case DRV_OC_INDEX_9:
        {
            DRV_OC9_CompareValuesSingleSet(compareValue);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX10 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX10 == "OC_SET_HIGH_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX10 == "OC_SET_LOW_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX10 == "OC_TOGGLE_CONTINUOUS_PULSE_MODE">
        case DRV_OC_INDEX_10:
        {
            DRV_OC10_CompareValuesSingleSet(compareValue);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX11 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX11 == "OC_SET_HIGH_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX11 == "OC_SET_LOW_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX11 == "OC_TOGGLE_CONTINUOUS_PULSE_MODE">
        case DRV_OC_INDEX_11:
        {
            DRV_OC11_CompareValuesSingleSet(compareValue);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX12 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX12 == "OC_SET_HIGH_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX12 == "OC_SET_LOW_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX12 == "OC_TOGGLE_CONTINUOUS_PULSE_MODE">
        case DRV_OC_INDEX_12:
        {
            DRV_OC12_CompareValuesSingleSet(compareValue);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX13 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX13 == "OC_SET_HIGH_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX13 == "OC_SET_LOW_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX13 == "OC_TOGGLE_CONTINUOUS_PULSE_MODE">
        case DRV_OC_INDEX_13:
        {
            DRV_OC13_CompareValuesSingleSet(compareValue);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX14 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX14 == "OC_SET_HIGH_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX14 == "OC_SET_LOW_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX14 == "OC_TOGGLE_CONTINUOUS_PULSE_MODE">
        case DRV_OC_INDEX_14:
        {
            DRV_OC14_CompareValuesSingleSet(compareValue);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX15 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX15 == "OC_SET_HIGH_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX15 == "OC_SET_LOW_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX15 == "OC_TOGGLE_CONTINUOUS_PULSE_MODE">
        case DRV_OC_INDEX_15:
        {
            DRV_OC15_CompareValuesSingleSet(compareValue);
            break;
        }
</#if>
        default:
        {
            SYS_ASSERT(false, "The selected instance of the OC driver is not configured for the Single Compare match mode");
            break;
        }
    }
}

void DRV_OC_CompareValuesDualSet(DRV_HANDLE handle, uint32_t priVal, uint32_t secVal)
{
    /* This API is supported only when selected instance of the OC driver is 
     * configured for the Dual Compare match modes. */

    switch(handle)
    {
<#if CONFIG_DRV_OC_INST_IDX0 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX0 == "OC_DUAL_COMPARE_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX0 == "OC_DUAL_COMPARE_CONTINUOUS_PULSE_MODE">
        case DRV_OC_INDEX_0:
        {
            DRV_OC0_CompareValuesDualSet(priVal, secVal);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX1 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX1 == "OC_DUAL_COMPARE_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX1 == "OC_DUAL_COMPARE_CONTINUOUS_PULSE_MODE">
        case DRV_OC_INDEX_1:
        {
            DRV_OC1_CompareValuesDualSet(priVal, secVal);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX2 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX2 == "OC_DUAL_COMPARE_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX2 == "OC_DUAL_COMPARE_CONTINUOUS_PULSE_MODE">
        case DRV_OC_INDEX_2:
        {
            DRV_OC2_CompareValuesDualSet(priVal, secVal);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX3 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX3 == "OC_DUAL_COMPARE_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX3 == "OC_DUAL_COMPARE_CONTINUOUS_PULSE_MODE">
        case DRV_OC_INDEX_3:
        {
            DRV_OC3_CompareValuesDualSet(priVal, secVal);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX4 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX4 == "OC_DUAL_COMPARE_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX4 == "OC_DUAL_COMPARE_CONTINUOUS_PULSE_MODE">
        case DRV_OC_INDEX_4:
        {
            DRV_OC4_CompareValuesDualSet(priVal, secVal);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX5 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX5 == "OC_DUAL_COMPARE_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX5 == "OC_DUAL_COMPARE_CONTINUOUS_PULSE_MODE">
        case DRV_OC_INDEX_5:
        {
            DRV_OC5_CompareValuesDualSet(priVal, secVal);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX6 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX6 == "OC_DUAL_COMPARE_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX6 == "OC_DUAL_COMPARE_CONTINUOUS_PULSE_MODE">
        case DRV_OC_INDEX_6:
        {
            DRV_OC6_CompareValuesDualSet(priVal, secVal);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX7 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX7 == "OC_DUAL_COMPARE_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX7 == "OC_DUAL_COMPARE_CONTINUOUS_PULSE_MODE">
        case DRV_OC_INDEX_7:
        {
            DRV_OC7_CompareValuesDualSet(priVal, secVal);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX8 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX8 == "OC_DUAL_COMPARE_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX8 == "OC_DUAL_COMPARE_CONTINUOUS_PULSE_MODE">
        case DRV_OC_INDEX_8:
        {
            DRV_OC8_CompareValuesDualSet(priVal, secVal);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX9 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX9 == "OC_DUAL_COMPARE_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX9 == "OC_DUAL_COMPARE_CONTINUOUS_PULSE_MODE">
        case DRV_OC_INDEX_9:
        {
            DRV_OC9_CompareValuesDualSet(priVal, secVal);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX10 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX10 == "OC_DUAL_COMPARE_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX10 == "OC_DUAL_COMPARE_CONTINUOUS_PULSE_MODE">
        case DRV_OC_INDEX_10:
        {
            DRV_OC10_CompareValuesDualSet(priVal, secVal);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX11 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX11 == "OC_DUAL_COMPARE_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX11 == "OC_DUAL_COMPARE_CONTINUOUS_PULSE_MODE">
        case DRV_OC_INDEX_11:
        {
            DRV_OC11_CompareValuesDualSet(priVal, secVal);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX12 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX12 == "OC_DUAL_COMPARE_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX12 == "OC_DUAL_COMPARE_CONTINUOUS_PULSE_MODE">
        case DRV_OC_INDEX_12:
        {
            DRV_OC12_CompareValuesDualSet(priVal, secVal);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX13 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX13 == "OC_DUAL_COMPARE_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX13 == "OC_DUAL_COMPARE_CONTINUOUS_PULSE_MODE">
        case DRV_OC_INDEX_13:
        {
            DRV_OC13_CompareValuesDualSet(priVal, secVal);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX14 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX14 == "OC_DUAL_COMPARE_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX14 == "OC_DUAL_COMPARE_CONTINUOUS_PULSE_MODE">
        case DRV_OC_INDEX_14:
        {
            DRV_OC14_CompareValuesDualSet(priVal, secVal);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX15 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX15 == "OC_DUAL_COMPARE_SINGLE_PULSE_MODE" || CONFIG_DRV_OC_COMPARE_MODES_IDX15 == "OC_DUAL_COMPARE_CONTINUOUS_PULSE_MODE">
        case DRV_OC_INDEX_15:
        {
            DRV_OC15_CompareValuesDualSet(priVal, secVal);
            break;
        }
</#if>
        default:
        {
            SYS_ASSERT(false, "The selected instance of the OC driver is not configured for the Dual Compare match mode");
            break;
        }
    }
}

void DRV_OC_PulseWidthSet(DRV_HANDLE handle, uint32_t pulseWidth)
{
    /* This API is supported only when selected instance of the OC driver is 
     * configured for the Dual Compare match modes. */

    switch(handle)
    {
<#if CONFIG_DRV_OC_INST_IDX0 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX0 == "OC_COMPARE_PWM_MODE_WITHOUT_FAULT_PROTECTION" || CONFIG_DRV_OC_COMPARE_MODES_IDX0 == "OC_COMPARE_PWM_MODE_WITH_FAULT_PROTECTION" || CONFIG_DRV_OC_COMPARE_MODES_IDX0 == "OC_COMPARE_PWM_EDGE_ALIGNED_MODE">
        case DRV_OC_INDEX_0:
        {
            DRV_OC0_PulseWidthSet(pulseWidth);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX1 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX1 == "OC_COMPARE_PWM_MODE_WITHOUT_FAULT_PROTECTION" || CONFIG_DRV_OC_COMPARE_MODES_IDX1 == "OC_COMPARE_PWM_MODE_WITH_FAULT_PROTECTION" || CONFIG_DRV_OC_COMPARE_MODES_IDX1 == "OC_COMPARE_PWM_EDGE_ALIGNED_MODE">
        case DRV_OC_INDEX_1:
        {
            DRV_OC1_PulseWidthSet(pulseWidth);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX2 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX2 == "OC_COMPARE_PWM_MODE_WITHOUT_FAULT_PROTECTION" || CONFIG_DRV_OC_COMPARE_MODES_IDX2 == "OC_COMPARE_PWM_MODE_WITH_FAULT_PROTECTION" || CONFIG_DRV_OC_COMPARE_MODES_IDX2 == "OC_COMPARE_PWM_EDGE_ALIGNED_MODE">
        case DRV_OC_INDEX_2:
        {
            DRV_OC2_PulseWidthSet(pulseWidth);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX3 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX3 == "OC_COMPARE_PWM_MODE_WITHOUT_FAULT_PROTECTION" || CONFIG_DRV_OC_COMPARE_MODES_IDX3 == "OC_COMPARE_PWM_MODE_WITH_FAULT_PROTECTION" || CONFIG_DRV_OC_COMPARE_MODES_IDX3 == "OC_COMPARE_PWM_EDGE_ALIGNED_MODE">
        case DRV_OC_INDEX_3:
        {
            DRV_OC3_PulseWidthSet(pulseWidth);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX4 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX4 == "OC_COMPARE_PWM_MODE_WITHOUT_FAULT_PROTECTION" || CONFIG_DRV_OC_COMPARE_MODES_IDX4 == "OC_COMPARE_PWM_MODE_WITH_FAULT_PROTECTION" || CONFIG_DRV_OC_COMPARE_MODES_IDX4 == "OC_COMPARE_PWM_EDGE_ALIGNED_MODE">
        case DRV_OC_INDEX_4:
        {
            DRV_OC4_PulseWidthSet(pulseWidth);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX5 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX5 == "OC_COMPARE_PWM_MODE_WITHOUT_FAULT_PROTECTION" || CONFIG_DRV_OC_COMPARE_MODES_IDX5 == "OC_COMPARE_PWM_MODE_WITH_FAULT_PROTECTION" || CONFIG_DRV_OC_COMPARE_MODES_IDX5 == "OC_COMPARE_PWM_EDGE_ALIGNED_MODE">
        case DRV_OC_INDEX_5:
        {
            DRV_OC5_PulseWidthSet(pulseWidth);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX6 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX6 == "OC_COMPARE_PWM_MODE_WITHOUT_FAULT_PROTECTION" || CONFIG_DRV_OC_COMPARE_MODES_IDX6 == "OC_COMPARE_PWM_MODE_WITH_FAULT_PROTECTION" || CONFIG_DRV_OC_COMPARE_MODES_IDX6 == "OC_COMPARE_PWM_EDGE_ALIGNED_MODE">
        case DRV_OC_INDEX_6:
        {
            DRV_OC6_PulseWidthSet(pulseWidth);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX7 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX7 == "OC_COMPARE_PWM_MODE_WITHOUT_FAULT_PROTECTION" || CONFIG_DRV_OC_COMPARE_MODES_IDX7 == "OC_COMPARE_PWM_MODE_WITH_FAULT_PROTECTION" || CONFIG_DRV_OC_COMPARE_MODES_IDX7 == "OC_COMPARE_PWM_EDGE_ALIGNED_MODE">
        case DRV_OC_INDEX_7:
        {
            DRV_OC7_PulseWidthSet(pulseWidth);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX8 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX8 == "OC_COMPARE_PWM_MODE_WITHOUT_FAULT_PROTECTION" || CONFIG_DRV_OC_COMPARE_MODES_IDX8 == "OC_COMPARE_PWM_MODE_WITH_FAULT_PROTECTION" || CONFIG_DRV_OC_COMPARE_MODES_IDX8 == "OC_COMPARE_PWM_EDGE_ALIGNED_MODE">
        case DRV_OC_INDEX_8:
        {
            DRV_OC8_PulseWidthSet(pulseWidth);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX9 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX9 == "OC_COMPARE_PWM_MODE_WITHOUT_FAULT_PROTECTION" || CONFIG_DRV_OC_COMPARE_MODES_IDX9 == "OC_COMPARE_PWM_MODE_WITH_FAULT_PROTECTION" || CONFIG_DRV_OC_COMPARE_MODES_IDX9 == "OC_COMPARE_PWM_EDGE_ALIGNED_MODE">
        case DRV_OC_INDEX_9:
        {
            DRV_OC9_PulseWidthSet(pulseWidth);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX10 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX10 == "OC_COMPARE_PWM_MODE_WITHOUT_FAULT_PROTECTION" || CONFIG_DRV_OC_COMPARE_MODES_IDX10 == "OC_COMPARE_PWM_MODE_WITH_FAULT_PROTECTION" || CONFIG_DRV_OC_COMPARE_MODES_IDX10 == "OC_COMPARE_PWM_EDGE_ALIGNED_MODE">
        case DRV_OC_INDEX_10:
        {
            DRV_OC10_PulseWidthSet(pulseWidth);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX11 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX11 == "OC_COMPARE_PWM_MODE_WITHOUT_FAULT_PROTECTION" || CONFIG_DRV_OC_COMPARE_MODES_IDX11 == "OC_COMPARE_PWM_MODE_WITH_FAULT_PROTECTION" || CONFIG_DRV_OC_COMPARE_MODES_IDX11 == "OC_COMPARE_PWM_EDGE_ALIGNED_MODE">
        case DRV_OC_INDEX_11:
        {
            DRV_OC11_PulseWidthSet(pulseWidth);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX12 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX12 == "OC_COMPARE_PWM_MODE_WITHOUT_FAULT_PROTECTION" || CONFIG_DRV_OC_COMPARE_MODES_IDX12 == "OC_COMPARE_PWM_MODE_WITH_FAULT_PROTECTION" || CONFIG_DRV_OC_COMPARE_MODES_IDX12 == "OC_COMPARE_PWM_EDGE_ALIGNED_MODE">
        case DRV_OC_INDEX_12:
        {
            DRV_OC12_PulseWidthSet(pulseWidth);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX13 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX13 == "OC_COMPARE_PWM_MODE_WITHOUT_FAULT_PROTECTION" || CONFIG_DRV_OC_COMPARE_MODES_IDX13 == "OC_COMPARE_PWM_MODE_WITH_FAULT_PROTECTION" || CONFIG_DRV_OC_COMPARE_MODES_IDX13 == "OC_COMPARE_PWM_EDGE_ALIGNED_MODE">
        case DRV_OC_INDEX_13:
        {
            DRV_OC13_PulseWidthSet(pulseWidth);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX14 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX14 == "OC_COMPARE_PWM_MODE_WITHOUT_FAULT_PROTECTION" || CONFIG_DRV_OC_COMPARE_MODES_IDX14 == "OC_COMPARE_PWM_MODE_WITH_FAULT_PROTECTION" || CONFIG_DRV_OC_COMPARE_MODES_IDX14 == "OC_COMPARE_PWM_EDGE_ALIGNED_MODE">
        case DRV_OC_INDEX_14:
        {
            DRV_OC14_PulseWidthSet(pulseWidth);
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX15 == true && CONFIG_DRV_OC_COMPARE_MODES_IDX15 == "OC_COMPARE_PWM_MODE_WITHOUT_FAULT_PROTECTION" || CONFIG_DRV_OC_COMPARE_MODES_IDX15 == "OC_COMPARE_PWM_MODE_WITH_FAULT_PROTECTION" || CONFIG_DRV_OC_COMPARE_MODES_IDX15 == "OC_COMPARE_PWM_EDGE_ALIGNED_MODE">
        case DRV_OC_INDEX_15:
        {
            DRV_OC15_PulseWidthSet(pulseWidth);
            break;
        }
</#if>
        default:
        {
            SYS_ASSERT(false, "The selected instance of the OC driver is not configured for the PWM mode");
            break;
        }
    }
}

bool DRV_OC_FaultHasOccurred(DRV_HANDLE handle)
{
    bool returnValue = true;  // Default state of buffer is empty.

    switch(handle)
    {
<#if CONFIG_DRV_OC_INST_IDX0 == true>
        case DRV_OC_INDEX_0:
        {
            returnValue = DRV_OC0_FaultHasOccurred();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX1 == true>
        case DRV_OC_INDEX_1:
        {
            returnValue = DRV_OC1_FaultHasOccurred();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX2 == true>
        case DRV_OC_INDEX_2:
        {
            returnValue = DRV_OC2_FaultHasOccurred();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX3 == true>
        case DRV_OC_INDEX_3:
        {
            returnValue = DRV_OC3_FaultHasOccurred();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX4 == true>
        case DRV_OC_INDEX_4:
        {
            returnValue = DRV_OC4_FaultHasOccurred();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX5 == true>
        case DRV_OC_INDEX_5:
        {
            returnValue = DRV_OC5_FaultHasOccurred();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX6 == true>
        case DRV_OC_INDEX_6:
        {
            returnValue = DRV_OC6_FaultHasOccurred();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX7 == true>
        case DRV_OC_INDEX_7:
        {
            returnValue = DRV_OC7_FaultHasOccurred();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX8 == true>
        case DRV_OC_INDEX_8:
        {
            returnValue = DRV_OC8_FaultHasOccurred();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX9 == true>
        case DRV_OC_INDEX_9:
        {
            returnValue = DRV_OC9_FaultHasOccurred();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX10 == true>
        case DRV_OC_INDEX_10:
        {
            returnValue = DRV_OC10_FaultHasOccurred();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX11 == true>
        case DRV_OC_INDEX_11:
        {
            returnValue = DRV_OC11_FaultHasOccurred();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX12 == true>
        case DRV_OC_INDEX_12:
        {
            returnValue = DRV_OC12_FaultHasOccurred();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX13 == true>
        case DRV_OC_INDEX_13:
        {
            returnValue = DRV_OC13_FaultHasOccurred();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX14 == true>
        case DRV_OC_INDEX_14:
        {
            returnValue = DRV_OC14_FaultHasOccurred();
            break;
        }
</#if>
<#if CONFIG_DRV_OC_INST_IDX15 == true>
        case DRV_OC_INDEX_15:
        {
            returnValue = DRV_OC15_FaultHasOccurred();
            break;
        }
</#if>
        default:
        {
            break;
        }
    }
    return returnValue;
}

/*******************************************************************************
 End of File
*/
