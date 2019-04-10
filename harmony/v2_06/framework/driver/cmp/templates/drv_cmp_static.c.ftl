/*******************************************************************************
  CMP Driver Functions for Static Single Instance Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_cmp_static.c

  Summary:
    CMP driver implementation for the static single instance driver.

  Description:
    The CMP device driver provides a simple interface to manage the CMP
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
// Header Includes
// *****************************************************************************
// *****************************************************************************
#include "framework/driver/cmp/drv_cmp_static.h"

<#macro DRV_CMP_STATIC_FUNCTIONS DRV_INSTANCE CMP_INSTANCE NON_INVERTING_INPUT 
INVERTING_INPUT OUTPUT_ENABLE OUTPUT_FILTER_ENABLE FILTER_CLK_SRC FILTER_CLK_DIV OPAMP_ENABLE OPAMP_OUTPUT_ENABLE CMP_EVE_INT CMP_INT_SRC CMP_INT_VEC CMP_INT_PRI CMP_INT_SPRI OUTPUT_INVERT>
// *****************************************************************************
/* Comparator driver instance ${DRV_INSTANCE} */
    PLIB_CMP_NonInvertingInputChannelSelect(${CMP_INSTANCE}, ${NON_INVERTING_INPUT});
    PLIB_CMP_InvertingInputChannelSelect(${CMP_INSTANCE}, ${INVERTING_INPUT});

    <#if OUTPUT_INVERT == false>
    PLIB_CMP_OutputInvertDisable(${CMP_INSTANCE});
    <#else>
    PLIB_CMP_OutputInvertEnable(${CMP_INSTANCE});
    </#if>    
    <#if OUTPUT_ENABLE == false>
    PLIB_CMP_OutputDisable(${CMP_INSTANCE});
    <#else>
    PLIB_CMP_OutputEnable(${CMP_INSTANCE});
    </#if>
    <#if CONFIG_PIC32MK == true>
 
    /* Setup comparator Output digital filter */
    <#if OUTPUT_FILTER_ENABLE == true>
    PLIB_CMP_ComparatorOutputDigitalFilterEnable(${CMP_INSTANCE});
    PLIB_CMP_ComparatorOutputDigitalFilterClkSetup (${CMP_INSTANCE}, ${FILTER_CLK_SRC}, ${FILTER_CLK_DIV} );
    <#else>
    PLIB_CMP_ComparatorOutputDigitalFilterDisable(${CMP_INSTANCE});
    </#if>
<#if CMP_INSTANCE != "CMP_ID_4">
 
    /* Setup Opamp mode */
    <#if OPAMP_ENABLE == true>
    PLIB_CMP_OpAmpEnable(${CMP_INSTANCE});
    <#if OPAMP_OUTPUT_ENABLE == true>
    PLIB_CMP_OpAmpOutputEnable(${CMP_INSTANCE});
    <#else>
    PLIB_CMP_OpAmpOutputDisable(${CMP_INSTANCE});
    </#if>
    <#else>
    PLIB_CMP_OpAmpDisable(${CMP_INSTANCE});
    </#if>
</#if>
    </#if>
    <#if CONFIG_DRV_CMP_INTERRUPT_MODE == true>

    /* Setup Interrupt */
    PLIB_CMP_InterruptEventSelect(${CMP_INSTANCE}, ${CMP_EVE_INT});   
    PLIB_INT_SourceEnable(INT_ID_0, ${CMP_INT_SRC});
    PLIB_INT_VectorPrioritySet(INT_ID_0, ${CMP_INT_VEC}, ${CMP_INT_PRI});
    PLIB_INT_VectorSubPrioritySet(INT_ID_0, ${CMP_INT_VEC}, ${CMP_INT_SPRI});          
    </#if>

    /* Enable Comparator */
    PLIB_CMP_Enable(${CMP_INSTANCE});


</#macro>

// *****************************************************************************
// *****************************************************************************
// Section: CMP static driver functions
// *****************************************************************************
// *****************************************************************************
void DRV_CMP_Initialize(void)
{

    <#if CONFIG_DRV_CVREF_ENABLE>	
    /* Configure CVREF for comparator use. */
    PLIB_CMP_CVREF_SourceVoltageSelect(CMP_ID_1, ${CONFIG_DRV_CMP_CVREF_VOLTAGE_SOURCE});
    <#if CONFIG_DRV_CMP_CVREF_WIDE_RANGE == true>
    PLIB_CMP_CVREF_WideRangeEnable(CMP_ID_1);
    </#if>
    #if defined(PLIB_CMP_ExistsCVREFRefVoltageRangeSelect )
        if ( PLIB_CMP_ExistsCVREFRefVoltageRangeSelect  ( CMP_ID_1 ) )
        {
		    PLIB_CMP_CVREF_ReferenceVoltageSelect ( CMP_ID_1,  ${CONFIG_DRV_CMP_CVREF} );
        }
        else
        {
            /* If Voltage reference selection for CVref feature doesn't exist 
			on CVREF module instance,
            then by default Resister Latter Network is selected as reference, so do nothing */
        }
    #endif	
    PLIB_CMP_CVREF_ValueSelect(CMP_ID_1, ${CONFIG_DRV_CMP_CVREF_VALUE});
    #if defined(PLIB_CMP_ExistsCVREFBGRefVoltageRangeSelect )
        if ( PLIB_CMP_ExistsCVREFBGRefVoltageRangeSelect  ( CMP_ID_1 ) )
        {
		    PLIB_CMP_CVREF_BandGapReferenceSourceSelect ( CMP_ID_1,  ${CONFIG_DRV_CMP_IVREF} );
        }
        else
        {
            /* If Voltage reference selection for IVref feature doesn't exist 
			on CVREF module instance,
            then by default internal 1.2V is selected as reference, so do nothing */
        }
    #endif	
    <#if CONFIG_DRV_CMP_CVREF_OE == true>
    PLIB_CMP_CVREF_OutputEnable(CMP_ID_1);
    </#if> 
    PLIB_CMP_CVREF_Enable(CMP_ID_1);
    </#if> 

<#if CONFIG_DRV_CMP_INST_IDX0 == true>
<@DRV_CMP_STATIC_FUNCTIONS DRV_INSTANCE="0" CMP_INSTANCE=CONFIG_DRV_CMP_PERIPHERAL_ID_IDX0 
NON_INVERTING_INPUT=CONFIG_DRV_CMP_NON_INVERTING_INPUT_IDX0 
INVERTING_INPUT=CONFIG_DRV_CMP_INVERTING_INPUT_IDX0 
OUTPUT_ENABLE=CONFIG_DRV_CMP_OUTPUT_ENABLE_IDX0
OUTPUT_FILTER_ENABLE=CONFIG_DRV_CMP_OUTPUT_DIGITAL_FILTER_IDX0
FILTER_CLK_SRC=CONFIG_DRV_CMP_OUTPUT_DIGITAL_FILTER_CLOCK_IDX0
FILTER_CLK_DIV=CONFIG_DRV_CMP_OUTPUT_DIGITAL_FILTER_CLOCK_DIV_IDX0
OPAMP_ENABLE=CONFIG_DRV_CMP_OPAMP_ENABLE_IDX0
OPAMP_OUTPUT_ENABLE=CONFIG_DRV_CMP_OPAMP_OUTPUT_ENABLE_IDX0
CMP_EVE_INT=CONFIG_DRV_CMP_INTERRUPT_EVENT_IDX0 CMP_INT_SRC=CONFIG_DRV_CMP_INTERRUPT_SOURCE_IDX0
CMP_INT_VEC=CONFIG_DRV_CMP_INTERRUPT_VECTOR_IDX0 CMP_INT_PRI=CONFIG_DRV_CMP_INT_PRIORITY_IDX0
CMP_INT_SPRI=CONFIG_DRV_CMP_INT_SUB_PRIORITY_IDX0 OUTPUT_INVERT=CONFIG_DRV_CMP_OUTPUT_INVERT_IDX0/>
</#if>
<#if CONFIG_DRV_CMP_INST_IDX1 == true>
<@DRV_CMP_STATIC_FUNCTIONS DRV_INSTANCE="1" CMP_INSTANCE=CONFIG_DRV_CMP_PERIPHERAL_ID_IDX1 
NON_INVERTING_INPUT=CONFIG_DRV_CMP_NON_INVERTING_INPUT_IDX1 
INVERTING_INPUT=CONFIG_DRV_CMP_INVERTING_INPUT_IDX1 
OUTPUT_ENABLE=CONFIG_DRV_CMP_OUTPUT_ENABLE_IDX1
OUTPUT_FILTER_ENABLE=CONFIG_DRV_CMP_OUTPUT_DIGITAL_FILTER_IDX1
FILTER_CLK_SRC=CONFIG_DRV_CMP_OUTPUT_DIGITAL_FILTER_CLOCK_IDX1
FILTER_CLK_DIV=CONFIG_DRV_CMP_OUTPUT_DIGITAL_FILTER_CLOCK_DIV_IDX1
OPAMP_ENABLE=CONFIG_DRV_CMP_OPAMP_ENABLE_IDX1
OPAMP_OUTPUT_ENABLE=CONFIG_DRV_CMP_OPAMP_OUTPUT_ENABLE_IDX1
CMP_EVE_INT=CONFIG_DRV_CMP_INTERRUPT_EVENT_IDX1 CMP_INT_SRC=CONFIG_DRV_CMP_INTERRUPT_SOURCE_IDX1
CMP_INT_VEC=CONFIG_DRV_CMP_INTERRUPT_VECTOR_IDX1 CMP_INT_PRI=CONFIG_DRV_CMP_INT_PRIORITY_IDX1
CMP_INT_SPRI=CONFIG_DRV_CMP_INT_SUB_PRIORITY_IDX1 OUTPUT_INVERT=CONFIG_DRV_CMP_OUTPUT_INVERT_IDX1/>
</#if>
<#if CONFIG_DRV_CMP_INST_IDX2 == true>
<@DRV_CMP_STATIC_FUNCTIONS DRV_INSTANCE="2" CMP_INSTANCE=CONFIG_DRV_CMP_PERIPHERAL_ID_IDX2 
NON_INVERTING_INPUT=CONFIG_DRV_CMP_NON_INVERTING_INPUT_IDX2 
INVERTING_INPUT=CONFIG_DRV_CMP_INVERTING_INPUT_IDX2 
OUTPUT_ENABLE=CONFIG_DRV_CMP_OUTPUT_ENABLE_IDX2
OUTPUT_FILTER_ENABLE=CONFIG_DRV_CMP_OUTPUT_DIGITAL_FILTER_IDX2
FILTER_CLK_SRC=CONFIG_DRV_CMP_OUTPUT_DIGITAL_FILTER_CLOCK_IDX2
FILTER_CLK_DIV=CONFIG_DRV_CMP_OUTPUT_DIGITAL_FILTER_CLOCK_DIV_IDX2
OPAMP_ENABLE=CONFIG_DRV_CMP_OPAMP_ENABLE_IDX2
OPAMP_OUTPUT_ENABLE=CONFIG_DRV_CMP_OPAMP_OUTPUT_ENABLE_IDX2
CMP_EVE_INT=CONFIG_DRV_CMP_INTERRUPT_EVENT_IDX2 CMP_INT_SRC=CONFIG_DRV_CMP_INTERRUPT_SOURCE_IDX2
CMP_INT_VEC=CONFIG_DRV_CMP_INTERRUPT_VECTOR_IDX2 CMP_INT_PRI=CONFIG_DRV_CMP_INT_PRIORITY_IDX2
CMP_INT_SPRI=CONFIG_DRV_CMP_INT_SUB_PRIORITY_IDX2 OUTPUT_INVERT=CONFIG_DRV_CMP_OUTPUT_INVERT_IDX2/>
</#if>
<#if CONFIG_DRV_CMP_INST_IDX3 == true>
<@DRV_CMP_STATIC_FUNCTIONS DRV_INSTANCE="3" CMP_INSTANCE=CONFIG_DRV_CMP_PERIPHERAL_ID_IDX3 
NON_INVERTING_INPUT=CONFIG_DRV_CMP_NON_INVERTING_INPUT_IDX3 
INVERTING_INPUT=CONFIG_DRV_CMP_INVERTING_INPUT_IDX3 
OUTPUT_ENABLE=CONFIG_DRV_CMP_OUTPUT_ENABLE_IDX3
OUTPUT_FILTER_ENABLE=CONFIG_DRV_CMP_OUTPUT_DIGITAL_FILTER_IDX3
FILTER_CLK_SRC=CONFIG_DRV_CMP_OUTPUT_DIGITAL_FILTER_CLOCK_IDX3
FILTER_CLK_DIV=CONFIG_DRV_CMP_OUTPUT_DIGITAL_FILTER_CLOCK_DIV_IDX3
OPAMP_ENABLE=0
OPAMP_OUTPUT_ENABLE=0
CMP_EVE_INT=CONFIG_DRV_CMP_INTERRUPT_EVENT_IDX3 CMP_INT_SRC=CONFIG_DRV_CMP_INTERRUPT_SOURCE_IDX3
CMP_INT_VEC=CONFIG_DRV_CMP_INTERRUPT_VECTOR_IDX3 CMP_INT_PRI=CONFIG_DRV_CMP_INT_PRIORITY_IDX3
CMP_INT_SPRI=CONFIG_DRV_CMP_INT_SUB_PRIORITY_IDX3 OUTPUT_INVERT=CONFIG_DRV_CMP_OUTPUT_INVERT_IDX3/>
</#if>
<#if CONFIG_DRV_CMP_INST_IDX4 == true>
<@DRV_CMP_STATIC_FUNCTIONS DRV_INSTANCE="4" CMP_INSTANCE=CONFIG_DRV_CMP_PERIPHERAL_ID_IDX4 
NON_INVERTING_INPUT=CONFIG_DRV_CMP_NON_INVERTING_INPUT_IDX4 
INVERTING_INPUT=CONFIG_DRV_CMP_INVERTING_INPUT_IDX4 
OUTPUT_ENABLE=CONFIG_DRV_CMP_OUTPUT_ENABLE_IDX4
OUTPUT_FILTER_ENABLE=CONFIG_DRV_CMP_OUTPUT_DIGITAL_FILTER_IDX4
FILTER_CLK_SRC=CONFIG_DRV_CMP_OUTPUT_DIGITAL_FILTER_CLOCK_IDX4
FILTER_CLK_DIV=CONFIG_DRV_CMP_OUTPUT_DIGITAL_FILTER_CLOCK_DIV_IDX4
OPAMP_ENABLE=CONFIG_DRV_CMP_OPAMP_ENABLE_IDX4
OPAMP_OUTPUT_ENABLE=CONFIG_DRV_CMP_OPAMP_OUTPUT_ENABLE_IDX4
CMP_EVE_INT=CONFIG_DRV_CMP_INTERRUPT_EVENT_IDX4 CMP_INT_SRC=CONFIG_DRV_CMP_INTERRUPT_SOURCE_IDX4
CMP_INT_VEC=CONFIG_DRV_CMP_INTERRUPT_VECTOR_IDX4 CMP_INT_PRI=CONFIG_DRV_CMP_INT_PRIORITY_IDX4
CMP_INT_SPRI=CONFIG_DRV_CMP_INT_SUB_PRIORITY_IDX4 OUTPUT_INVERT=CONFIG_DRV_CMP_OUTPUT_INVERT_IDX4/>
</#if>

}

/*******************************************************************************
 End of File
*/
