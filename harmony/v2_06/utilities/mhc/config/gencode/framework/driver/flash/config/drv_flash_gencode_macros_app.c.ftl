<#-- drv_flash_gencode_macros_app.c.ftl -->
<#--
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
#include "${APP_NAME?lower_case}.h"
-->
<#macro macro_drv_flash_app_c_includes>
</#macro>
<#--
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
/* Application Data
*/
-->
<#macro macro_drv_flash_app_c_global_data>
// String to write into the flash.  Be aware that the double quotes adds a null byte at the end of the string.
// So, writing "Hello World!" actually stores 13 bytes.
const uint8_t ${APP_NAME?lower_case}FlashWriteString[] = "${("CONFIG_APP_DRV_FLASH_WRITE_STRING" + "${HCONFIG_APP_INSTANCE}")?eval}";
/* Buffer in the KSEG1 RAM to store the data */
uint32_t ${APP_NAME?lower_case}FlashRowBuff[DRV_FLASH_ROW_SIZE/sizeof(uint32_t)] __attribute__((coherent, aligned(16)));
<#if ("CONFIG_DRV_FLASH_APP_WRITE_STRING_VERIFY" + "${HCONFIG_APP_INSTANCE}")?eval>
<#--  These lines can be inserted to read the flash at various points for debugging.
/* Array in the KSEG1 RAM to read the data for verification purposes.*/
uint32_t ${APP_NAME?lower_case}FlashBuffVerify[${APP_NAME?upper_case}_FLASH_DRV_DWORD_COUNT] __attribute__((coherent, aligned(16)));
-->
</#if>
/* FlashReserveArea refers to memory starting from address 
   FLASH_DRV_START_ADDRESS. The attribute keep instructs the linker not to remove the section
   even if it is not refered anywhere in the code.
 */
const uint8_t ${APP_NAME?lower_case}FlashReserveArea[${APP_NAME?upper_case}_FLASH_DRV_BYTE_COUNT] __attribute__ ((keep)) __attribute__((address(${APP_NAME?upper_case}_FLASH_DRV_START_ADDRESS))) = {0xFF, };
</#macro>
<#--
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
-->
<#macro macro_drv_flash_app_c_callback_functions>
</#macro>
<#--
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
-->
<#macro macro_drv_flash_app_c_local_functions>
/* Application's Flash Setup Function */
static void FlashSetup( void )
{
    ${APP_NAME?lower_case}Data.flashStates = ${APP_NAME?upper_case}_FLASH_START;
}
/******************************************************************************
  Function:
    static void ${APP_NAME?upper_case}_FLASH_Task (void)
    
   Remarks:
    Allows a polled state machine to manage flash testing.

*/
static void ${APP_NAME?upper_case}_FLASH_Task (void)
{		
	switch(${APP_NAME?lower_case}Data.flashStates)
	{
		default:
		
		case ${APP_NAME?upper_case}_FLASH_START:
        {
            ${APP_NAME?lower_case}Data.flashAddress = (uint32_t)${APP_NAME?lower_case}FlashReserveArea;
            ${APP_NAME?lower_case}Data.flashEndAddress = ${APP_NAME?lower_case}Data.flashAddress + ${APP_NAME?upper_case}_FLASH_DRV_BYTE_COUNT;
            ${APP_NAME?lower_case}Data.bufferAddress = ((uint32_t *)${APP_NAME?lower_case}FlashWriteString);
            ${APP_NAME?lower_case}Data.flashStates = ${APP_NAME?upper_case}_FLASH_ERASE;
			break;
		}	
		case ${APP_NAME?upper_case}_FLASH_ERASE:
        {
            /*
                This state will Erase any and all pages which contain data between the Start Address
                and the End address.  Each erase operation erases one Page of flash.  So, this state 
                will perform more than one operation if required.
                Be aware that even the amount of flash to be used is small, the minimum erasure is still
                one full Page.
            */
            if(!DRV_FLASH_IsBusy(${APP_NAME?lower_case}Data.${("CONFIG_APP_FLASH_DRV_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval}))
            {
                if(${APP_NAME?lower_case}Data.flashAddress < ${APP_NAME?lower_case}Data.flashEndAddress)
                {
                    DRV_FLASH_ErasePage(${APP_NAME?lower_case}Data.${("CONFIG_APP_FLASH_DRV_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval}
                                        , ${APP_NAME?lower_case}Data.flashAddress);
                    ${APP_NAME?lower_case}Data.flashAddress += DRV_FLASH_PAGE_SIZE;
                } else {
                    <#if ("CONFIG_DRV_FLASH_APP_WRITE_STRING_VERIFY" + "${HCONFIG_APP_INSTANCE}")?eval>
                    <#--  These lines can be inserted to read the flash at various points for debugging.
                    // Verify the erase operation by reading the flash into the verify buffer.
                    // This is primarily for debug purposes.
                    memcpy((void *)(${APP_NAME?lower_case}FlashBuffVerify), (const void *)(KVA0_TO_KVA1(${APP_NAME?lower_case}FlashReserveArea)), ${APP_NAME?upper_case}_FLASH_DRV_BYTE_COUNT);
                    -->
                    </#if>
                    ${APP_NAME?lower_case}Data.flashAddress = (uint32_t)${APP_NAME?lower_case}FlashReserveArea;
                    ${APP_NAME?lower_case}Data.flashStates = ${APP_NAME?upper_case}_FLASH_WRITE;
                }
            }
			break;
		}	
		case ${APP_NAME?upper_case}_FLASH_WRITE:
        {
            /*
                This state will Write the given string one row at a time.  Since the string may take 
                multiple rows to write, it will continue performing write operations until the whole 
                string has been written.  Since the start address may not coinside with the Row start
                address, it pre loads the Row ram buffer with data from the flash.  This also allows
                the last write operation to stop before the end of a Row.
            */
            if(!DRV_FLASH_IsBusy(${APP_NAME?lower_case}Data.${("CONFIG_APP_FLASH_DRV_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval}))
            {
                if(${APP_NAME?lower_case}Data.flashAddress < ${APP_NAME?lower_case}Data.flashEndAddress)
                {
                    uint32_t NextRowEndFlash = ${APP_NAME?upper_case}_FLASH_DRV_ROW_END_ADDRESS(${APP_NAME?lower_case}Data.flashAddress);
                    uint32_t * NextRowStartFlash = (uint32_t *)${APP_NAME?upper_case}_FLASH_DRV_ROW_START_ADDRESS(${APP_NAME?lower_case}Data.flashAddress);
                    uint32_t LastBufferIndex = DRV_FLASH_ROW_SIZE/sizeof(uint32_t);
                    uint32_t FirstBufferIndex = 0;
                    uint32_t BytesToProgram = DRV_FLASH_ROW_SIZE;
                    // If next flash address is the starting address, then compensate for any row start offset.
                    // The starting address might not be aligned with the start of a row.  This math aligns
                    // the data into the row buffer correctly.
                    if(${APP_NAME?lower_case}Data.flashAddress == (uint32_t)${APP_NAME?lower_case}FlashReserveArea)
                    {
                        FirstBufferIndex = ${APP_NAME?upper_case}_ROW_START_OFFSET/sizeof(uint32_t);
                    } 
                    // If the end of the next row is beyond the end of the flash, then
                    // reduce the amount of data copied into the row buffer by the amount
                    // that a full row write would have gone past the end of flash.
                    if(NextRowEndFlash > ${APP_NAME?lower_case}Data.flashEndAddress)
                    {
                        LastBufferIndex -= (NextRowEndFlash - ${APP_NAME?lower_case}Data.flashEndAddress)/sizeof(uint32_t);
                    }
                    /*  Initialize row buffer to Flash Contents */
                    memcpy((void *)(${APP_NAME?lower_case}FlashRowBuff), (const void *)(KVA0_TO_KVA1(NextRowStartFlash)), DRV_FLASH_ROW_SIZE);
                    //Copy next rows worth of the string into the row buffer.
                    BytesToProgram = (LastBufferIndex*sizeof(uint32_t) - FirstBufferIndex*sizeof(uint32_t));
                    memcpy((void *)(${APP_NAME?lower_case}FlashRowBuff+FirstBufferIndex), (const void *)(KVA0_TO_KVA1(${APP_NAME?lower_case}Data.bufferAddress)), BytesToProgram);

                    DRV_FLASH_WriteRow(${APP_NAME?lower_case}Data.${("CONFIG_APP_FLASH_DRV_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval}
                                        , (uint32_t)(KVA0_TO_KVA1(${APP_NAME?lower_case}Data.flashAddress)), ${APP_NAME?lower_case}FlashRowBuff);
                    ${APP_NAME?lower_case}Data.flashAddress += BytesToProgram;
                    ${APP_NAME?lower_case}Data.bufferAddress += BytesToProgram/4;
                } else {
                    <#if ("CONFIG_DRV_FLASH_APP_WRITE_STRING_VERIFY" + "${HCONFIG_APP_INSTANCE}")?eval>
                    <#--  These lines can be inserted to read the flash at various points for debugging.
                    // Verify the erase operation by reading the flash into the verify buffer.
                    // This is primarily for debug purposes.
                    memcpy((void *)(${APP_NAME?lower_case}FlashBuffVerify), (const void *)(KVA0_TO_KVA1(${APP_NAME?lower_case}FlashReserveArea)), ${APP_NAME?upper_case}_FLASH_DRV_BYTE_COUNT);
                    -->
                    ${APP_NAME?lower_case}Data.flashStates = ${APP_NAME?upper_case}_FLASH_VERIFY;
                    <#else>
                    ${APP_NAME?lower_case}Data.flashStates = ${APP_NAME?upper_case}_FLASH_DONE;
                    </#if>
                }
            }
			break;
        }
        <#if ("CONFIG_DRV_FLASH_APP_WRITE_STRING_VERIFY" + "${HCONFIG_APP_INSTANCE}")?eval>
		case ${APP_NAME?upper_case}_FLASH_VERIFY:
        {
            if(!DRV_FLASH_IsBusy(${APP_NAME?lower_case}Data.${("CONFIG_APP_FLASH_DRV_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval}))
            {
                <#--  These lines can be inserted to read the flash at various points for debugging.
                // Verify the write operation(s) by reading the flash into the verify buffer.
                // This is primarily for debug purposes.
                memcpy((void *)(${APP_NAME?lower_case}FlashBuffVerify), (const void *)(KVA0_TO_KVA1(${APP_NAME?lower_case}FlashReserveArea)), ${APP_NAME?upper_case}_FLASH_DRV_BYTE_COUNT);
                -->
                /* Verify that data written to flash memory is valid (FlashReserveArea array read from kseg1) */
                if (!memcmp((const void *)${APP_NAME?lower_case}FlashWriteString, (const void *)(KVA0_TO_KVA1(${APP_NAME?lower_case}FlashReserveArea)), ${APP_NAME?upper_case}_FLASH_DRV_BYTE_COUNT))
                {
                    ${APP_NAME?lower_case}Data.flashStates = ${APP_NAME?upper_case}_FLASH_DONE;
                }
                else
                {
                    ${APP_NAME?lower_case}Data.flashStates = ${APP_NAME?upper_case}_FLASH_ERROR;
                }
            }
			break;
        }
		</#if>	
		case ${APP_NAME?upper_case}_FLASH_DONE:
		{
			break;
        }
        <#if ("CONFIG_DRV_FLASH_APP_WRITE_STRING_VERIFY" + "${HCONFIG_APP_INSTANCE}")?eval>
        case ${APP_NAME?upper_case}_FLASH_ERROR:
		{
            break;
        }
        </#if>
    }
}
</#macro>

<#--
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void ${APP_NAME?upper_case}_Initialize ( void )

  Remarks:
    See prototype in ${APP_NAME?lower_case}.h.
 */

void ${APP_NAME?upper_case}_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    ${APP_NAME?lower_case}Data.state = ${APP_NAME?upper_case}_STATE_INIT;
-->
<#macro macro_drv_flash_app_c_initialize>
    ${APP_NAME?lower_case}Data.${("CONFIG_APP_FLASH_DRV_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval} = DRV_HANDLE_INVALID;
</#macro>

<#--
}


/******************************************************************************
  Function:
    void ${APP_NAME?upper_case}_Tasks ( void )

  Remarks:
    See prototype in ${APP_NAME?lower_case}.h.
 */

void ${APP_NAME?upper_case}_Tasks ( void )
{
-->
<#macro macro_drv_flash_app_c_tasks_data>
</#macro>

<#--
    /* Check the application's current state. */
    switch ( ${APP_NAME?lower_case}Data.state )
    {
        /* Application's initial state. */
        case ${APP_NAME?upper_case}_STATE_INIT:
        {
            bool appInitialized = true;
-->   
<#macro macro_drv_flash_app_c_tasks_state_init>
            if (${APP_NAME?lower_case}Data.${("CONFIG_APP_FLASH_DRV_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval} == DRV_HANDLE_INVALID)
            {
                ${APP_NAME?lower_case}Data.${("CONFIG_APP_FLASH_DRV_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval} = DRV_FLASH_Open(${APP_NAME?upper_case}_FLASH_INDEX, DRV_IO_INTENT_EXCLUSIVE);
                appInitialized &= ( DRV_HANDLE_INVALID != ${APP_NAME?lower_case}Data.${("CONFIG_APP_FLASH_DRV_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval} );
            }
</#macro>    

<#--        
            if (appInitialized)
            {
-->
<#macro macro_drv_flash_app_c_tasks_calls_after_init>
                FlashSetup();
</#macro>

<#--            
                ${APP_NAME?lower_case}Data.state = ${APP_NAME?upper_case}_STATE_SERVICE_TASKS;
            }
            break;
        }

        case ${APP_NAME?upper_case}_STATE_SERVICE_TASKS:
        {
-->
<#macro macro_drv_flash_app_c_tasks_state_service_tasks>
		    /* Run the state machine for servicing Flash */
            ${APP_NAME?upper_case}_FLASH_Task();
</#macro>

<#--        
            break;
        }

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}
-->

