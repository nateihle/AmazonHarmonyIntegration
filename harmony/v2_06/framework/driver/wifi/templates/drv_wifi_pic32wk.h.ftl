/*** Wi-Fi Driver Configuration ***/
#define DRV_WIFI_MODE   "${CONFIG_DRV_WIFI_WK_MODE}"
<#if CONFIG_DRV_WIFI_OTA_ENABLE == true>
#define DRV_WIFI_OTA_ENABLE
</#if>
<#if CONFIG_DRV_WIFI_DEBUG_ENABLE == true>
#define DRV_WIFI_DEBUG_ENABLE
<#if CONFIG_DRV_WIFI_DEBUG_LEVEL == "ERROR">
#define DRV_DEBUG_PRINT_LEVEL		1
</#if>
<#if CONFIG_DRV_WIFI_DEBUG_LEVEL == "DEBUG">
#define DRV_DEBUG_PRINT_LEVEL		2
</#if>
<#if CONFIG_DRV_WIFI_DEBUG_LEVEL == "INFO">
#define DRV_DEBUG_PRINT_LEVEL		3
</#if>
<#if CONFIG_DRV_WIFI_DEBUG_LEVEL == "FUNCTION">
#define DRV_DEBUG_PRINT_LEVEL		4
</#if>
</#if>
<#--
/*******************************************************************************
 End of File
 */
-->