<#--
/*******************************************************************************
  Memory System Service EBI Initialization File

  File Name:
    sys_memory_ebi_static.c

  Summary:
    This file contains source code necessary to initialize the EBI controller.

  Description:
    This file contains source code necessary to initialize the EBI controller.
 *******************************************************************************/

/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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
 -->
// *****************************************************************************
// *****************************************************************************
// Header Includes
// *****************************************************************************
// *****************************************************************************
#include "peripheral/ebi/plib_ebi.h"
#include "system/memory/ebi/sys_memory_ebi_static.h"

<#if CONFIG_USE_SYS_MEMORY_EBI == true>
<#if CONFIG_SYS_MEMORY_EBI_SERVICE_MODE == "STATIC">
/*******************************************************************************
  Function:
    void SYS_MEMORY_EBI_Initialize(void)

  Summary:
    Initializes EBI Controller

  Remarks:
 */
void SYS_MEMORY_EBI_Initialize(void)
{
    /* Configure EBI Pins */
    /* Global Pin Control by EBI */
    PLIB_EBI_ControlEnableSet(EBI_ID_0, true);
    /* Enable Address Pins */
    PLIB_EBI_AddressPinEnableBitsSet(EBI_ID_0, ${CONFIG_SYS_MEMORY_EBI_ADDRESS_PINS_CONFIG});
    /* Data Byte Enables */ 
<#if CONFIG_SYS_MEMORY_EBI_DATA_PINS_CONFIG == "MEMORY_WIDTH_16BIT">	
    PLIB_EBI_DataEnableSet(EBI_ID_0, true, true);
<#else>
    PLIB_EBI_DataEnableSet(EBI_ID_0, false, true);
</#if>
<#if CONFIG_SYS_MEMORY_EBI_CSX0 == true && CONFIG_SYS_MEMORY_EBI_CSX1 == false && CONFIG_SYS_MEMORY_EBI_CSX2 == false && CONFIG_SYS_MEMORY_EBI_CSX3 == false>
    /* /EBICSx Pin Configuration */
    PLIB_EBI_ChipSelectEnableSet (EBI_ID_0, true, false, false, false);
</#if>   
<#if CONFIG_SYS_MEMORY_EBI_CSX0 == true && CONFIG_SYS_MEMORY_EBI_CSX1 == true && CONFIG_SYS_MEMORY_EBI_CSX2 == false && CONFIG_SYS_MEMORY_EBI_CSX3 == false>
    /* /EBICSx Pin Configuration */
    PLIB_EBI_ChipSelectEnableSet (EBI_ID_0, true, true, false, false); 
</#if>   
<#if CONFIG_SYS_MEMORY_EBI_CSX0 == true && CONFIG_SYS_MEMORY_EBI_CSX1 == true && CONFIG_SYS_MEMORY_EBI_CSX2 == true && CONFIG_SYS_MEMORY_EBI_CSX3 == false>
    /* /EBICSx Pin Configuration */
    PLIB_EBI_ChipSelectEnableSet (EBI_ID_0, true, true, true, false);
</#if>
<#if CONFIG_SYS_MEMORY_EBI_CSX0 == true && CONFIG_SYS_MEMORY_EBI_CSX1 == true && CONFIG_SYS_MEMORY_EBI_CSX2 == true && CONFIG_SYS_MEMORY_EBI_CSX3 == true>
    /* /EBICSx Pin Configuration */
    PLIB_EBI_ChipSelectEnableSet (EBI_ID_0, true, true, true, true);   
</#if>
<#if (CONFIG_SYS_MEMORY_EBI_BYTE_SELECT_PIN0 && CONFIG_SYS_MEMORY_EBI_BYTE_SELECT_PIN1) == true>
    /* /EBIBS0 and /EBIBS1 enabled */
    PLIB_EBI_ByteSelectPinSet(EBI_ID_0, true, true);
</#if> 
<#if CONFIG_SYS_MEMORY_EBI_BYTE_SELECT_PIN0 == true && CONFIG_SYS_MEMORY_EBI_BYTE_SELECT_PIN1 == false>
    /* /EBIBS0 enabled and /EBIBS1 disabled */
    PLIB_EBI_ByteSelectPinSet(EBI_ID_0, true, false);
</#if> 
<#if CONFIG_SYS_MEMORY_EBI_BYTE_SELECT_PIN0 == false && CONFIG_SYS_MEMORY_EBI_BYTE_SELECT_PIN1 == true>
    /* /EBIBS0 disabled and /EBIBS1 enabled */
    PLIB_EBI_ByteSelectPinSet(EBI_ID_0, false, true);
</#if>   
<#if CONFIG_SYS_MEMORY_EBI_BYTE_SELECT_PIN0 == false && CONFIG_SYS_MEMORY_EBI_BYTE_SELECT_PIN1 == false>
    /* /EBIBS0 and /EBIBS1 disabled */
    PLIB_EBI_ByteSelectPinSet(EBI_ID_0, false, false);
</#if> 
<#if CONFIG_SYS_MEMORY_EBI_ENABLE_WE == true && CONFIG_SYS_MEMORY_EBI_ENABLE_OE == true>
    /* /EBIWE and /EBIOE are enabled */
    PLIB_EBI_WriteOutputControlSet (EBI_ID_0, true, true); 
</#if>
<#if CONFIG_SYS_MEMORY_EBI_ENABLE_WE == true && CONFIG_SYS_MEMORY_EBI_ENABLE_OE == false>
    /* /EBIWE is enabled and /EBIOE is disabled */
    PLIB_EBI_WriteOutputControlSet (EBI_ID_0, true, false); 
</#if>
<#if CONFIG_SYS_MEMORY_EBI_ENABLE_WE == false && CONFIG_SYS_MEMORY_EBI_ENABLE_OE == true>
    /* /EBIWE is disabled and /EBIOE is enabled */
    PLIB_EBI_WriteOutputControlSet (EBI_ID_0, false, true); 
</#if>
<#if CONFIG_SYS_MEMORY_EBI_ENABLE_WE == false && CONFIG_SYS_MEMORY_EBI_ENABLE_OE == false>
    /* /EBIWE and /EBIOE are disabled */
    PLIB_EBI_WriteOutputControlSet (EBI_ID_0, false, false); 
</#if>
<#if CONFIG_SYS_MEMORY_EBI_MEMORY_NOR_FLASH == true>
<#if CONFIG_SYS_MEMORY_EBI_MEMORY_NOR_FLASH_RP_CONFIG == true>
    /* /EBIRP is enabled */
    PLIB_EBI_FlashResetPinSet(EBI_ID_0, true);
<#else>
    /* /EBIRP is disabled */
	PLIB_EBI_FlashResetPinSet(EBI_ID_0, false);	
</#if>
</#if>	
</#if>
<#if CONFIG_SYS_MEMORY_EBI_USE_RDY_PINS == true>
<#if CONFIG_SYS_MEMORY_EBI_USE_RDY1_PIN == true>
<#if CONFIG_SYS_MEMORY_EBI_USE_RDY1_PIN_INV == true>
    /* EBIRDY1 Enabled and Inverted */
    PLIB_EBI_ReadyPin1ConfigSet (EBI_ID_0, true, true);
<#else>
    /* EBIRDY1 Enabled but not Inverted */
    PLIB_EBI_ReadyPin1ConfigSet (EBI_ID_0, true, false);
</#if>
</#if>
<#if CONFIG_SYS_MEMORY_EBI_USE_RDY2_PIN == true>
<#if CONFIG_SYS_MEMORY_EBI_USE_RDY2_PIN_INV == true>
    /* EBIRDY2 Enabled and Inverted */
    PLIB_EBI_ReadyPin2ConfigSet (EBI_ID_0, true, true);
<#else>
    /* EBIRDY2 Enabled but not Inverted */
    PLIB_EBI_ReadyPin2ConfigSet (EBI_ID_0, true, false);
</#if>
</#if>
<#if CONFIG_SYS_MEMORY_EBI_USE_RDY3_PIN == true>
<#if CONFIG_SYS_MEMORY_EBI_USE_RDY3_PIN_INV == true>
    /* EBIRDY3 Enabled and Inverted */
    PLIB_EBI_ReadyPin3ConfigSet (EBI_ID_0, true, true); 
<#else>
    /* EBIRDY3 Enabled but not Inverted */
    PLIB_EBI_ReadyPin3ConfigSet (EBI_ID_0, true, false);
</#if>
</#if>
<#if CONFIG_SYS_MEMORY_EBI_RDY_PIN_SENSITIVITY == "EDGE">
    /* EBIRDYx is edge sensitive */ 
    PLIB_EBI_ReadyPinSensSet (EBI_ID_0, false); 
</#if>
<#if CONFIG_SYS_MEMORY_EBI_RDY_PIN_SENSITIVITY == "LEVEL">
    /* EBIRDYx is level sensitive */
    PLIB_EBI_ReadyPinSensSet (EBI_ID_0, true);
</#if>
</#if>
<#macro SYS_MEMORY_EBI_CSX_INST CS_NUM MEM_BASE_ADDR MEM_SIZE MEM_TYPE TMG_REG>

    /* Initiialize EBI for Memory on EBICS${CS_NUM} */
    /* Setup EBICS${CS_NUM} */
    PLIB_EBI_BaseAddressSet(EBI_ID_0, ${CS_NUM}, ${MEM_BASE_ADDR});
    /* Setup EBIMSK${CS_NUM} */
    PLIB_EBI_MemoryCharacteristicsSet(EBI_ID_0, ${CS_NUM}, ${MEM_TYPE}, ${MEM_SIZE}, ${TMG_REG});
<#if TMG_REG == "CS_TIMING_0">
    /* Setup EBISMT0 */
<#if CONFIG_SYS_MEMORY_EBI_USE_RDY_PINS == true>
    /* Setup EBISMT0->RDYMODE */
    PLIB_EBI_ReadyModeSet(EBI_ID_0, true, false, false);
<#else>
    /* Setup EBISMT0->RDYMODE */
    PLIB_EBI_ReadyModeSet(EBI_ID_0, false, false, false);    	
</#if>
<#if CONFIG_SYS_MEMORY_EBI_MEM_PAGE_MODE_TMG0 == true>
    PLIB_EBI_MemoryPagingSet(EBI_ID_0, ${CS_NUM}, true, ${CONFIG_SYS_MEMORY_EBI_MEM_PAGE_SIZE_TMG0});
    PLIB_EBI_MemoryTimingConfigSet(EBI_ID_0, 0, ${CONFIG_SYS_MEMORY_EBI_MEM_PAGE_MODE_TRC_TMG0}, ${CONFIG_SYS_MEMORY_EBI_MEM_DATABUS_TURNAROUND_TIME_TMG0}, ${CONFIG_SYS_MEMORY_EBI_MEM_WRITE_PULSE_WIDTH_TMG0}, ${CONFIG_SYS_MEMORY_EBI_MEM_ADDRESS_DATA_HOLD_TIME_TMG0}, ${CONFIG_SYS_MEMORY_EBI_MEM_ADDRESS_SETUP_TIME_TMG0}, ${CONFIG_SYS_MEMORY_EBI_MEM_READ_CYCLE_TIME_TMG0});
<#else>
    PLIB_EBI_MemoryPagingSet(EBI_ID_0, ${CS_NUM}, false, PAGE_WORD32);
    PLIB_EBI_MemoryTimingConfigSet(EBI_ID_0, 0, 0, ${CONFIG_SYS_MEMORY_EBI_MEM_DATABUS_TURNAROUND_TIME_TMG0}, ${CONFIG_SYS_MEMORY_EBI_MEM_WRITE_PULSE_WIDTH_TMG0}, ${CONFIG_SYS_MEMORY_EBI_MEM_ADDRESS_DATA_HOLD_TIME_TMG0}, ${CONFIG_SYS_MEMORY_EBI_MEM_ADDRESS_SETUP_TIME_TMG0}, ${CONFIG_SYS_MEMORY_EBI_MEM_READ_CYCLE_TIME_TMG0});
</#if>
    /* Setup EBISMCON->SMWIDTH0 */
    PLIB_EBI_StaticMemoryWidthRegisterSet(EBI_ID_0, 0, ${CONFIG_SYS_MEMORY_EBI_MEM_WIDTH_SET_TMG0});
</#if>
<#if TMG_REG == "CS_TIMING_1">
    /* Setup EBISMT1 */
<#if CONFIG_SYS_MEMORY_EBI_USE_RDY_PINS == true>
    /* Setup EBISMT1->RDYMODE */
    PLIB_EBI_ReadyModeSet(EBI_ID_0, false, true, false);
<#else>
    /* Setup EBISMT1->RDYMODE */
    PLIB_EBI_ReadyModeSet(EBI_ID_0, false, false, false); 	
</#if>
<#if CONFIG_SYS_MEMORY_EBI_MEM_PAGE_MODE_TMG1 == true>
    PLIB_EBI_MemoryPagingSet(EBI_ID_0, ${CS_NUM}, true, ${CONFIG_SYS_MEMORY_EBI_MEM_PAGE_SIZE_TMG1});
    PLIB_EBI_MemoryTimingConfigSet(EBI_ID_0, 1, ${CONFIG_SYS_MEMORY_EBI_MEM_PAGE_MODE_TRC_TMG1}, ${CONFIG_SYS_MEMORY_EBI_MEM_DATABUS_TURNAROUND_TIME_TMG1}, ${CONFIG_SYS_MEMORY_EBI_MEM_WRITE_PULSE_WIDTH_TMG1}, ${CONFIG_SYS_MEMORY_EBI_MEM_ADDRESS_DATA_HOLD_TIME_TMG1}, ${CONFIG_SYS_MEMORY_EBI_MEM_ADDRESS_SETUP_TIME_TMG1}, ${CONFIG_SYS_MEMORY_EBI_MEM_READ_CYCLE_TIME_TMG1});
<#else>
    PLIB_EBI_MemoryPagingSet(EBI_ID_0, ${CS_NUM}, false, PAGE_WORD32);
    PLIB_EBI_MemoryTimingConfigSet(EBI_ID_0, 1, 0, ${CONFIG_SYS_MEMORY_EBI_MEM_DATABUS_TURNAROUND_TIME_TMG1}, ${CONFIG_SYS_MEMORY_EBI_MEM_WRITE_PULSE_WIDTH_TMG1}, ${CONFIG_SYS_MEMORY_EBI_MEM_ADDRESS_DATA_HOLD_TIME_TMG1}, ${CONFIG_SYS_MEMORY_EBI_MEM_ADDRESS_SETUP_TIME_TMG1}, ${CONFIG_SYS_MEMORY_EBI_MEM_READ_CYCLE_TIME_TMG1});
</#if>
    /* Setup EBISMCON->SMWIDTH1 */
    PLIB_EBI_StaticMemoryWidthRegisterSet(EBI_ID_0, 1, ${CONFIG_SYS_MEMORY_EBI_MEM_WIDTH_SET_TMG1});
</#if>	
<#if TMG_REG == "CS_TIMING_2">
    /* Setup EBISMT2 */
<#if CONFIG_SYS_MEMORY_EBI_USE_RDY_PINS == true>
    /* Setup EBISMT2->RDYMODE */
    PLIB_EBI_ReadyModeSet(EBI_ID_0, false, false, true);
<#else>
    /* Setup EBISMT3->RDYMODE */
    PLIB_EBI_ReadyModeSet(EBI_ID_0, false, false, false); 	
</#if>
<#if CONFIG_SYS_MEMORY_EBI_MEM_PAGE_MODE_TMG2 == true>
    PLIB_EBI_MemoryPagingSet(EBI_ID_0, ${CS_NUM}, true, ${CONFIG_SYS_MEMORY_EBI_MEM_PAGE_SIZE_TMG2});
    PLIB_EBI_MemoryTimingConfigSet(EBI_ID_0, 2, ${CONFIG_SYS_MEMORY_EBI_MEM_PAGE_MODE_TRC_TMG2}, ${CONFIG_SYS_MEMORY_EBI_MEM_DATABUS_TURNAROUND_TIME_TMG2}, ${CONFIG_SYS_MEMORY_EBI_MEM_WRITE_PULSE_WIDTH_TMG2}, ${CONFIG_SYS_MEMORY_EBI_MEM_ADDRESS_DATA_HOLD_TIME_TMG2}, ${CONFIG_SYS_MEMORY_EBI_MEM_ADDRESS_SETUP_TIME_TMG2}, ${CONFIG_SYS_MEMORY_EBI_MEM_READ_CYCLE_TIME_TMG2});
<#else>
    PLIB_EBI_MemoryPagingSet(EBI_ID_0, ${CS_NUM}, false, PAGE_WORD32);
    PLIB_EBI_MemoryTimingConfigSet(EBI_ID_0, 2, 0, ${CONFIG_SYS_MEMORY_EBI_MEM_DATABUS_TURNAROUND_TIME_TMG2}, ${CONFIG_SYS_MEMORY_EBI_MEM_WRITE_PULSE_WIDTH_TMG2}, ${CONFIG_SYS_MEMORY_EBI_MEM_ADDRESS_DATA_HOLD_TIME_TMG2}, ${CONFIG_SYS_MEMORY_EBI_MEM_ADDRESS_SETUP_TIME_TMG2}, ${CONFIG_SYS_MEMORY_EBI_MEM_READ_CYCLE_TIME_TMG2});
</#if>
    /* Setup EBISMCON->SMWIDTH2 */
    PLIB_EBI_StaticMemoryWidthRegisterSet(EBI_ID_0, 2, ${CONFIG_SYS_MEMORY_EBI_MEM_WIDTH_SET_TMG2});   
</#if>	
</#macro>
<#if CONFIG_SYS_MEMORY_EBI_CSX0 == true>
<@SYS_MEMORY_EBI_CSX_INST CS_NUM=CONFIG_SYS_MEMORY_EBI_CS_NUMBER_CSX0 
MEM_BASE_ADDR=CONFIG_SYS_MEMORY_EBI_BASE_ADDR_CSX0 MEM_SIZE=CONFIG_SYS_MEMORY_EBI_MEMORY_SIZE_CSX0
MEM_TYPE=CONFIG_SYS_MEMORY_EBI_MEMORY_TYPE_CSX0 TMG_REG=CONFIG_SYS_MEMORY_EBI_CS_TIMING_CSX0/>
</#if>
<#if CONFIG_SYS_MEMORY_EBI_CSX1 == true>
<@SYS_MEMORY_EBI_CSX_INST CS_NUM=CONFIG_SYS_MEMORY_EBI_CS_NUMBER_CSX1 
MEM_BASE_ADDR=CONFIG_SYS_MEMORY_EBI_BASE_ADDR_CSX1 MEM_SIZE=CONFIG_SYS_MEMORY_EBI_MEMORY_SIZE_CSX1
MEM_TYPE=CONFIG_SYS_MEMORY_EBI_MEMORY_TYPE_CSX1 TMG_REG=CONFIG_SYS_MEMORY_EBI_CS_TIMING_CSX1/>
<#if CONFIG_SYS_MEMORY_EBI_CSX2 == true>
<@SYS_MEMORY_EBI_CSX_INST CS_NUM=CONFIG_SYS_MEMORY_EBI_CS_NUMBER_CSX2 
MEM_BASE_ADDR=CONFIG_SYS_MEMORY_EBI_BASE_ADDR_CSX2 MEM_SIZE=CONFIG_SYS_MEMORY_EBI_MEMORY_SIZE_CSX2
MEM_TYPE=CONFIG_SYS_MEMORY_EBI_MEMORY_TYPE_CSX2 TMG_REG=CONFIG_SYS_MEMORY_EBI_CS_TIMING_CSX2/>
</#if>
<#if CONFIG_SYS_MEMORY_EBI_CSX3 == true>
<@SYS_MEMORY_EBI_CSX_INST CS_NUM=CONFIG_SYS_MEMORY_EBI_CS_NUMBER_CSX3
MEM_BASE_ADDR=CONFIG_SYS_MEMORY_EBI_BASE_ADDR_CSX3 MEM_SIZE=CONFIG_SYS_MEMORY_EBI_MEMORY_SIZE_CSX3
MEM_TYPE=CONFIG_SYS_MEMORY_EBI_MEMORY_TYPE_CSX3 TMG_REG=CONFIG_SYS_MEMORY_EBI_CS_TIMING_CSX3/>
</#if>
</#if>
}
</#if>
<#--
/*******************************************************************************
 End of File
*/
-->
