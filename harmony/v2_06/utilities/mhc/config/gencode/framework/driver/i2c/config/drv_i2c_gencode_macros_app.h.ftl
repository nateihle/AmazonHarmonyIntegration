<#-- drv_i2c_gencode_macros_app.h.ftl -->

<#--
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
-->
<#macro macro_drv_i2c_app_h_includes>
#include "driver/i2c/drv_i2c.h"
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
<#macro macro_drv_i2c_app_h_constants>
/*
I2C driver model constants used by the application:
*/
#define ${APP_NAME?upper_case}_DRV_I2C_INDEX   			    ${("CONFIG_APP_DRV_I2C_INSTANCE_INDEX" + "${HCONFIG_APP_INSTANCE}")?eval}
#define ${APP_NAME?upper_case}_PERPH_I2C_INDEX 			    ${("CONFIG_DRV_I2C_PERIPHERAL_ID_IDX" + "${(\"CONFIG_APP_DRV_I2C_INSTANCE_INDEX\" + \"${HCONFIG_APP_INSTANCE}\")?eval}")?eval}

#define ${APP_NAME?upper_case}_DATABUFF_SIZE_BYTES          (${APP_NAME?upper_case}_I2C_DRV_BYTE_COUNT)
#define ${APP_NAME?upper_case}_DATABUFF_SIZE_DWORDS         ((${APP_NAME?upper_case}_I2C_DRV_BYTE_COUNT)/4)
</#macro>



<#--
// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_drv_i2c_app_h_type_definitions>
// *****************************************************************************
/* I2C States
*/
typedef enum
{
    ${APP_NAME?upper_case}_I2C_START,	
    <#if ("CONFIG_APP_DRV_I2C_MASTER_TRANSMIT" + "${HCONFIG_APP_INSTANCE}")?eval>
    ${APP_NAME?upper_case}_I2C_TRANSMIT,		
    </#if>
    <#if ("CONFIG_APP_DRV_I2C_SLAVE_RECEIVE" + "${HCONFIG_APP_INSTANCE}")?eval>
    ${APP_NAME?upper_case}_I2C_RECEIVE,
    </#if>
    ${APP_NAME?upper_case}_I2C_DONE,
    ${APP_NAME?upper_case}_I2C_ERROR,
} ${APP_NAME?upper_case}_I2C_STATES;
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
<#macro macro_drv_i2c_app_h_data>
    /* I2C Driver variables  */
    DRV_HANDLE                              ${("CONFIG_APP_DRV_I2C_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval};
	${APP_NAME?upper_case}_I2C_STATES		i2cStates;
    DRV_I2C_BUFFER_HANDLE                   I2CBufferHandle;
    DRV_I2C_BUFFER_EVENT                    I2CBufferEvent;
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
<#macro macro_drv_i2c_app_h_callback_function_declarations>
<#if ("CONFIG_APP_DRV_I2C_SLAVE_RECEIVE" + "${HCONFIG_APP_INSTANCE}")?eval>
void ${("CONFIG_DRV_I2C_SLAVE_CALLBACK_FUNCTION_IDX" + "${(\"CONFIG_APP_DRV_I2C_INSTANCE_INDEX\" + \"${HCONFIG_APP_INSTANCE}\")?eval}")?eval}(DRV_I2C_BUFFER_EVENT event, void * context);
</#if>
</#macro>

<#--	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_drv_i2c_app_h_function_declarations>
</#macro>




