<#-- drv_tmr_gencode_macros_system_config.h.ftl -->

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
<#macro macro_drv_tmr_system_config_h_app_constants>
#define ${APP_NAME?upper_case}_TMR_DRV                       ${("CONFIG_APP_TMR_DRV_INSTANCE_INDEX" + "${HCONFIG_APP_INSTANCE}")?eval}
#define ${APP_NAME?upper_case}_TMR_DRV_IS_PERIODIC           <#if ("CONFIG_APP_TMR_DRV_PERIODIC" + "${HCONFIG_APP_INSTANCE}")?eval>true<#else>false</#if>
#define ${APP_NAME?upper_case}_TMR_DRV_PERIOD                ${("CONFIG_APP_TMR_DRV_PERIOD" + "${HCONFIG_APP_INSTANCE}")?eval}
<#if ("CONFIG_APP_TMR_DRV_USE_GLOBAL_EVENT" + "${HCONFIG_APP_INSTANCE}")?eval>
<#if ("CONFIG_APP_TMR_DRV_GLOBAL_EVENT_COUNT" + "${HCONFIG_APP_INSTANCE}")?eval?number != 1>
#define ${APP_NAME?upper_case}_TIMER_CALLBACKS_PER_EVENT     ${("CONFIG_APP_TMR_DRV_GLOBAL_EVENT_COUNT" + "${HCONFIG_APP_INSTANCE}")?eval}
</#if>
</#if>
</#macro>
