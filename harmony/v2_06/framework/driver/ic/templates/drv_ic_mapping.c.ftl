/*******************************************************************************
  IC Driver Dynamic to Static mapping

  Company:
    Microchip Technology Inc.

  File Name:
    drv_ic_mapping.c

  Summary:
    Source code for the IC driver dynamic APIs to static API mapping.

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
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
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


SYS_MODULE_OBJ DRV_IC_Initialize(const SYS_MODULE_INDEX index,const SYS_MODULE_INIT * const init)
{
    SYS_MODULE_OBJ returnValue = index;

    switch(index)
    {
        case DRV_IC_INDEX_0:
        {
            DRV_IC0_Initialize();
            break;
        }
<#if CONFIG_DRV_IC_INST_IDX1 == true>
        case DRV_IC_INDEX_1:
        {
            DRV_IC1_Initialize();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX2 == true>
        case DRV_IC_INDEX_2:
        {
            DRV_IC2_Initialize();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX3 == true>
        case DRV_IC_INDEX_3:
        {
            DRV_IC3_Initialize();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX4 == true>
        case DRV_IC_INDEX_4:
        {
            DRV_IC4_Initialize();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX5 == true>
        case DRV_IC_INDEX_5:
        {
            DRV_IC5_Initialize();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX6 == true>
        case DRV_IC_INDEX_6:
        {
            DRV_IC6_Initialize();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX7 == true>
        case DRV_IC_INDEX_7:
        {
            DRV_IC7_Initialize();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX8 == true>
        case DRV_IC_INDEX_8:
        {
            DRV_IC8_Initialize();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX9 == true>
        case DRV_IC_INDEX_9:
        {
            DRV_IC9_Initialize();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX10 == true>
        case DRV_IC_INDEX_10:
        {
            DRV_IC10_Initialize();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX11 == true>
        case DRV_IC_INDEX_11:
        {
            DRV_IC11_Initialize();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX12 == true>
        case DRV_IC_INDEX_12:
        {
            DRV_IC12_Initialize();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX13 == true>
        case DRV_IC_INDEX_13:
        {
            DRV_IC13_Initialize();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX14 == true>
        case DRV_IC_INDEX_14:
        {
            DRV_IC14_Initialize();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX15 == true>
        case DRV_IC_INDEX_15:
        {
            DRV_IC15_Initialize();
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

DRV_HANDLE DRV_IC_Start(const SYS_MODULE_INDEX drvIndex, const DRV_IO_INTENT intent)
{
    SYS_MODULE_OBJ returnValue = drvIndex;

    switch(drvIndex)
    {
        case DRV_IC_INDEX_0:
        {
            DRV_IC0_Start();
            break;
        }
<#if CONFIG_DRV_IC_INST_IDX1 == true>
        case DRV_IC_INDEX_1:
        {
            DRV_IC1_Start();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX2 == true>
        case DRV_IC_INDEX_2:
        {
            DRV_IC2_Start();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX3 == true>
        case DRV_IC_INDEX_3:
        {
            DRV_IC3_Start();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX4 == true>
        case DRV_IC_INDEX_4:
        {
            DRV_IC4_Start();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX5 == true>
        case DRV_IC_INDEX_5:
        {
            DRV_IC5_Start();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX6 == true>
        case DRV_IC_INDEX_6:
        {
            DRV_IC6_Start();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX7 == true>
        case DRV_IC_INDEX_7:
        {
            DRV_IC7_Start();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX8 == true>
        case DRV_IC_INDEX_8:
        {
            DRV_IC8_Start();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX9 == true>
        case DRV_IC_INDEX_9:
        {
            DRV_IC9_Start();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX10 == true>
        case DRV_IC_INDEX_10:
        {
            DRV_IC10_Start();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX11 == true>
        case DRV_IC_INDEX_11:
        {
            DRV_IC11_Start();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX12 == true>
        case DRV_IC_INDEX_12:
        {
            DRV_IC12_Start();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX13 == true>
        case DRV_IC_INDEX_13:
        {
            DRV_IC13_Start();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX14 == true>
        case DRV_IC_INDEX_14:
        {
            DRV_IC14_Start();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX15 == true>
        case DRV_IC_INDEX_15:
        {
            DRV_IC15_Start();
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


void DRV_IC_Stop(DRV_HANDLE handle)
{
    switch(handle)
    {
        case DRV_IC_INDEX_0:
        {
            DRV_IC0_Stop();
            break;
        }
<#if CONFIG_DRV_IC_INST_IDX1 == true>
        case DRV_IC_INDEX_1:
        {
            DRV_IC1_Stop();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX2 == true>
        case DRV_IC_INDEX_2:
        {
            DRV_IC2_Stop();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX3 == true>
        case DRV_IC_INDEX_3:
        {
            DRV_IC3_Stop();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX4 == true>
        case DRV_IC_INDEX_4:
        {
            DRV_IC4_Stop();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX5 == true>
        case DRV_IC_INDEX_5:
        {
            DRV_IC5_Stop();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX6 == true>
        case DRV_IC_INDEX_6:
        {
            DRV_IC6_Stop();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX7 == true>
        case DRV_IC_INDEX_7:
        {
            DRV_IC7_Stop();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX8 == true>
        case DRV_IC_INDEX_8:
        {
            DRV_IC8_Stop();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX9 == true>
        case DRV_IC_INDEX_9:
        {
            DRV_IC9_Stop();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX10 == true>
        case DRV_IC_INDEX_10:
        {
            DRV_IC10_Stop();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX11 == true>
        case DRV_IC_INDEX_11:
        {
            DRV_IC11_Stop();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX12 == true>
        case DRV_IC_INDEX_12:
        {
            DRV_IC12_Stop();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX13 == true>
        case DRV_IC_INDEX_13:
        {
            DRV_IC13_Stop();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX14 == true>
        case DRV_IC_INDEX_14:
        {
            DRV_IC14_Stop();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX15 == true>
        case DRV_IC_INDEX_15:
        {
            DRV_IC15_Stop();
            break;
        }
</#if>
        default:
        {
            break;
        }
    }
}

uint32_t DRV_IC_Capture32BitDataRead(DRV_HANDLE handle)
{
    uint32_t returnValue;

    switch(handle)
    {
        case DRV_IC_INDEX_0:
        {
            returnValue = DRV_IC0_Capture32BitDataRead();
            break;
        }
<#if CONFIG_DRV_IC_INST_IDX1 == true>
        case DRV_IC_INDEX_1:
        {
            returnValue = DRV_IC1_Capture32BitDataRead();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX2 == true>
        case DRV_IC_INDEX_2:
        {
            returnValue = DRV_IC2_Capture32BitDataRead();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX3 == true>
        case DRV_IC_INDEX_3:
        {
            returnValue = DRV_IC3_Capture32BitDataRead();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX4 == true>
        case DRV_IC_INDEX_4:
        {
            returnValue = DRV_IC4_Capture32BitDataRead();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX5 == true>
        case DRV_IC_INDEX_5:
        {
            returnValue = DRV_IC5_Capture32BitDataRead();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX6 == true>
        case DRV_IC_INDEX_6:
        {
            returnValue = DRV_IC6_Capture32BitDataRead();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX7 == true>
        case DRV_IC_INDEX_7:
        {
            returnValue = DRV_IC7_Capture32BitDataRead();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX8 == true>
        case DRV_IC_INDEX_8:
        {
            returnValue = DRV_IC8_Capture32BitDataRead();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX9 == true>
        case DRV_IC_INDEX_9:
        {
            returnValue = DRV_IC9_Capture32BitDataRead();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX10 == true>
        case DRV_IC_INDEX_10:
        {
            returnValue = DRV_IC10_Capture32BitDataRead();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX11 == true>
        case DRV_IC_INDEX_11:
        {
            returnValue = DRV_IC11_Capture32BitDataRead();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX12 == true>
        case DRV_IC_INDEX_12:
        {
            returnValue = DRV_IC12_Capture32BitDataRead();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX13 == true>
        case DRV_IC_INDEX_13:
        {
            returnValue = DRV_IC13_Capture32BitDataRead();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX14 == true>
        case DRV_IC_INDEX_14:
        {
            returnValue = DRV_IC14_Capture32BitDataRead();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX15 == true>
        case DRV_IC_INDEX_15:
        {
            returnValue = DRV_IC15_Capture32BitDataRead();
            break;
        }
</#if>
        default:
        {
            returnValue = 0;
            break;
        }
    }
    return returnValue;
}


uint16_t DRV_IC_Capture16BitDataRead(DRV_HANDLE handle)
{
    uint16_t returnValue;

    switch(handle)
    {
        case DRV_IC_INDEX_0:
        {
            returnValue = DRV_IC0_Capture16BitDataRead();
            break;
        }
<#if CONFIG_DRV_IC_INST_IDX1 == true>
        case DRV_IC_INDEX_1:
        {
            returnValue = DRV_IC1_Capture16BitDataRead();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX2 == true>
        case DRV_IC_INDEX_2:
        {
            returnValue = DRV_IC2_Capture16BitDataRead();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX3 == true>
        case DRV_IC_INDEX_3:
        {
            returnValue = DRV_IC3_Capture16BitDataRead();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX4 == true>
        case DRV_IC_INDEX_4:
        {
            returnValue = DRV_IC4_Capture16BitDataRead();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX5 == true>
        case DRV_IC_INDEX_5:
        {
            returnValue = DRV_IC5_Capture16BitDataRead();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX6 == true>
        case DRV_IC_INDEX_6:
        {
            returnValue = DRV_IC6_Capture16BitDataRead();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX7 == true>
        case DRV_IC_INDEX_7:
        {
            returnValue = DRV_IC7_Capture16BitDataRead();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX8 == true>
        case DRV_IC_INDEX_8:
        {
            returnValue = DRV_IC8_Capture16BitDataRead();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX9 == true>
        case DRV_IC_INDEX_9:
        {
            returnValue = DRV_IC9_Capture16BitDataRead();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX10 == true>
        case DRV_IC_INDEX_10:
        {
            returnValue = DRV_IC10_Capture16BitDataRead();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX11 == true>
        case DRV_IC_INDEX_11:
        {
            returnValue = DRV_IC11_Capture16BitDataRead();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX12 == true>
        case DRV_IC_INDEX_12:
        {
            returnValue = DRV_IC12_Capture16BitDataRead();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX13 == true>
        case DRV_IC_INDEX_13:
        {
            returnValue = DRV_IC13_Capture16BitDataRead();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX14 == true>
        case DRV_IC_INDEX_14:
        {
            returnValue = DRV_IC14_Capture16BitDataRead();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX15 == true>
        case DRV_IC_INDEX_15:
        {
            returnValue = DRV_IC15_Capture16BitDataRead();
            break;
        }
</#if>
        default:
        {
            returnValue = 0;
            break;
        }
    }
    return returnValue;
}


bool DRV_IC_BufferIsEmpty(DRV_HANDLE handle)
{
    bool returnValue = true;  // Default state of buffer is empty.

    switch(handle)
    {
        case DRV_IC_INDEX_0:
        {
            returnValue = DRV_IC0_BufferIsEmpty();
            break;
        }
<#if CONFIG_DRV_IC_INST_IDX1 == true>
        case DRV_IC_INDEX_1:
        {
            returnValue = DRV_IC1_BufferIsEmpty();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX2 == true>
        case DRV_IC_INDEX_2:
        {
            returnValue = DRV_IC2_BufferIsEmpty();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX3 == true>
        case DRV_IC_INDEX_3:
        {
            returnValue = DRV_IC3_BufferIsEmpty();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX4 == true>
        case DRV_IC_INDEX_4:
        {
            returnValue = DRV_IC4_BufferIsEmpty();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX5 == true>
        case DRV_IC_INDEX_5:
        {
            returnValue = DRV_IC5_BufferIsEmpty();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX6 == true>
        case DRV_IC_INDEX_6:
        {
            returnValue = DRV_IC6_BufferIsEmpty();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX7 == true>
        case DRV_IC_INDEX_7:
        {
            returnValue = DRV_IC7_BufferIsEmpty();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX8 == true>
        case DRV_IC_INDEX_8:
        {
            returnValue = DRV_IC8_BufferIsEmpty();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX9 == true>
        case DRV_IC_INDEX_9:
        {
            returnValue = DRV_IC9_BufferIsEmpty();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX10 == true>
        case DRV_IC_INDEX_10:
        {
            returnValue = DRV_IC10_BufferIsEmpty();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX11 == true>
        case DRV_IC_INDEX_11:
        {
            returnValue = DRV_IC11_BufferIsEmpty();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX12 == true>
        case DRV_IC_INDEX_12:
        {
            returnValue = DRV_IC12_BufferIsEmpty();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX13 == true>
        case DRV_IC_INDEX_13:
        {
            returnValue = DRV_IC13_BufferIsEmpty();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX14 == true>
        case DRV_IC_INDEX_14:
        {
            returnValue = DRV_IC14_BufferIsEmpty();
            break;
        }
</#if>
<#if CONFIG_DRV_IC_INST_IDX15 == true>
        case DRV_IC_INDEX_15:
        {
            returnValue = DRV_IC15_BufferIsEmpty();
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
