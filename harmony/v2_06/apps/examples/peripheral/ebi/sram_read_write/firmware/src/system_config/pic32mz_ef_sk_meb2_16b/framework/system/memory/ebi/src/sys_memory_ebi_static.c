// *****************************************************************************
// *****************************************************************************
// Header Includes
// *****************************************************************************
// *****************************************************************************
#include "peripheral/ebi/plib_ebi.h"
#include "system/memory/ebi/sys_memory_ebi_static.h"

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
    PLIB_EBI_AddressPinEnableBitsSet(EBI_ID_0, EBI_EBIADDR_PIN19);
    /* Data Byte Enables */ 
    PLIB_EBI_DataEnableSet(EBI_ID_0, true, true);
    /* /EBICSx Pin Configuration */
    PLIB_EBI_ChipSelectEnableSet (EBI_ID_0, true, false, false, false);
    /* /EBIBS0 and /EBIBS1 enabled */
    PLIB_EBI_ByteSelectPinSet(EBI_ID_0, true, true);
    /* /EBIWE and /EBIOE are enabled */
    PLIB_EBI_WriteOutputControlSet (EBI_ID_0, true, true); 

    /* Initiialize EBI for Memory on EBICS0 */
    /* Setup EBICS0 */
    PLIB_EBI_BaseAddressSet(EBI_ID_0, 0, 0x20000000);
    /* Setup EBIMSK0 */
    PLIB_EBI_MemoryCharacteristicsSet(EBI_ID_0, 0, SRAM, MEMORY_SIZE_2MB, CS_TIMING_0);
    /* Setup EBISMT0 */
    /* Setup EBISMT0->RDYMODE */
    PLIB_EBI_ReadyModeSet(EBI_ID_0, false, false, false);    	
    PLIB_EBI_MemoryPagingSet(EBI_ID_0, 0, false, PAGE_WORD32);
    PLIB_EBI_MemoryTimingConfigSet(EBI_ID_0, 0, 0, 0, 10, 1, 2, 2);
    /* Setup EBISMCON->SMWIDTH0 */
    PLIB_EBI_StaticMemoryWidthRegisterSet(EBI_ID_0, 0, MEMORY_WIDTH_16BIT);
}
