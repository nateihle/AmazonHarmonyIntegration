<#-- drv_flash_gencode_macros_app.h.ftl -->

<#--
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
-->
<#macro macro_drv_flash_app_h_includes>
#include "system_config.h"
#include "driver/driver_common.h"
#include "driver/flash/drv_flash.h"
#include "peripheral/ports/plib_ports.h"
</#macro>

<#--
// *****************************************************************************
/* Application Data
// *****************************************************************************

// *****************************************************************************
/* Application constants

  Summary:
    Constants defined for the application

  Description:
    Constants defined for the application
*/
-->
<#macro macro_drv_flash_app_h_constants>

/*
Flash driver model constants used by the application:

Flash driver model constants used by the application:
#define ${APP_NAME?upper_case}_FLASH_DRV_START_ADDRESS  : First Address in flash to write to.  From user input.
#define ${APP_NAME?upper_case}_FLASH_DRV_BYTE_COUNT     : Number of bytes to write to flash.
#define ${APP_NAME?upper_case}_FLASH_DRV_DWORD_COUNT    : Size of the string to write in dwords (uint32_t)
#define ${APP_NAME?upper_case}_FLASH_DRV_HANDLE         : Handle name for use with the driver's Open function.
#define ${APP_NAME?upper_case}_FLASH_INDEX              : Index of flash instance.
#define ${APP_NAME?upper_case}_ROW_START_OFFSET         : The flash start address may not begin at the begining of a flash row.
                                                            This macro provides the offset into the row of the starting address.
#define ${APP_NAME?upper_case}_PAGE_START_OFFSET        : The flash start address may not begin at the begining of a flash page.
                                                            This macro provides the offset into the page of the starting address.
*/

#define ${APP_NAME?upper_case}_FLASH_DRV_START_ADDRESS          ${("CONFIG_APP_FLASH_DRV_START_ADDRESS" + "${HCONFIG_APP_INSTANCE}")?eval}
#define ${APP_NAME?upper_case}_FLASH_DRV_ROW_START_ADDRESS(x)   ((x) & (~DRV_FLASH_ROW_SIZE+1))
#define ${APP_NAME?upper_case}_FLASH_DRV_ROW_END_ADDRESS(x)     (${APP_NAME?upper_case}_FLASH_DRV_ROW_START_ADDRESS(x+DRV_FLASH_ROW_SIZE))
#define ${APP_NAME?upper_case}_FLASH_DRV_BYTE_COUNT             (sizeof(${APP_NAME?lower_case}FlashWriteString))
#define ${APP_NAME?upper_case}_FLASH_DRV_DWORD_COUNT            (((${APP_NAME?upper_case}_FLASH_DRV_BYTE_COUNT)/(sizeof(uint32_t))))
#define ${APP_NAME?upper_case}_FLASH_DRV_HANDLE                 ${("CONFIG_APP_FLASH_DRV_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval}
#define ${APP_NAME?upper_case}_FLASH_INDEX                      DRV_FLASH_INDEX_0
#define ${APP_NAME?upper_case}_ROW_START_OFFSET                 ((${APP_NAME?upper_case}_FLASH_DRV_START_ADDRESS) - ${APP_NAME?upper_case}_FLASH_DRV_ROW_START_ADDRESS(${APP_NAME?upper_case}_FLASH_DRV_START_ADDRESS))

</#macro>



<#--
// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_drv_flash_app_h_type_definitions>
// *****************************************************************************
/* Flash States
*/
typedef enum
{
    ${APP_NAME?upper_case}_FLASH_START,	
    ${APP_NAME?upper_case}_FLASH_ERASE,		
    ${APP_NAME?upper_case}_FLASH_WRITE,
    <#if ("CONFIG_DRV_FLASH_APP_WRITE_STRING_VERIFY" + "${HCONFIG_APP_INSTANCE}")?eval>
    ${APP_NAME?upper_case}_FLASH_VERIFY,
    </#if>
    ${APP_NAME?upper_case}_FLASH_DONE,
    ${APP_NAME?upper_case}_FLASH_ERROR,
} ${APP_NAME?upper_case}_FLASH_STATES;
</#macro>

<#--
// *****************************************************************************
/* Application Data

typedef struct
{
    /* The application's current state */
    ${APP_NAME?upper_case}_STATES state;

    /* TODO: Define any additional data used by the application. */
-->
<#macro macro_drv_flash_app_h_data>
    DRV_HANDLE ${("CONFIG_APP_FLASH_DRV_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval};
	${APP_NAME?upper_case}_FLASH_STATES		flashStates;
	uint32_t    flashAddress;
    uint32_t    flashEndAddress;
	uint32_t    * bufferAddress;
</#macro>
<#--
} ${APP_NAME?upper_case}_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/
-->
<#macro macro_drv_flash_app_h_callback_function_declarations>
</#macro>

<#--	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_drv_flash_app_h_function_declarations>
</#macro>




