#include "gfx/hal/inc/gfx_driver_interface.h"

<#assign GFX_DRIVER_COUNT = 0>
<#if CONFIG_USE_GFX_STACK = true>
<#if CONFIG_DRV_GFX_CONTROLLER_TYPE = "Low Cost Controllerless">
<#include "/framework/gfx/driver/controller/lcc/templates/drv_gfx_lcc_definition.ftl">
<@DRV_GFX_LCC_DRIVER_DECLARATION IDX = 0/>
</#if>
<#if CONFIG_DRV_GFX_CONTROLLER_TYPE = "GLCD">
<#include "/framework/gfx/driver/controller/glcd/templates/drv_gfx_glcd_definition.ftl">
<@DRV_GFX_GLCD_DRIVER_DECLARATION IDX = 0/>
</#if>
<#if CONFIG_DRV_GFX_CONTROLLER_TYPE = "Epson S1D13517">
<#include "/framework/gfx/driver/controller/s1d13517/templates/drv_gfx_s1d13517_definition.ftl">
<@DRV_GFX_S1D13517_DRIVER_DECLARATION IDX = 0/>
</#if>
<#if CONFIG_DRV_GFX_CONTROLLER_TYPE = "SSD1926">
<#include "/framework/gfx/driver/controller/ssd1926/templates/drv_gfx_ssd1926_definition.ftl">
<@DRV_GFX_SSD1926_DRIVER_DECLARATION IDX = 0/>
</#if>
<#if CONFIG_DRV_GFX_CONTROLLER_TYPE = "SSD1963">
<#include "/framework/gfx/driver/controller/ssd1963/templates/drv_gfx_ssd1963_definition.ftl">
<@DRV_GFX_SSD1963_DRIVER_DECLARATION IDX = 0/>
</#if>
<#if CONFIG_DRV_GFX_CONTROLLER_TYPE = "OTM2201A">
<#include "/framework/gfx/driver/controller/otm2201a/templates/drv_gfx_otm2201a_definition.ftl">
<@DRV_GFX_OTM2201A_DRIVER_DECLARATION IDX = 0/>
</#if>
<#if CONFIG_DRV_GFX_CONTROLLER_TYPE = "ILI9488 (SPI 4-LINE)">
<#include "/framework/gfx/driver/controller/ili9488/templates/drv_gfx_ili9488_definition.ftl">
<@DRV_GFX_ILI9488_DRIVER_DECLARATION IDX = 0/>
</#if>
<#if CONFIG_DRV_GFX_CONTROLLER_TYPE = "ILI9488 (16-bit Parallel/SMC)">
<#include "/framework/gfx/driver/controller/ili9488/templates/drv_gfx_ili9488_definition.ftl">
<@DRV_GFX_ILI9488_DRIVER_DECLARATION IDX = 0/>
</#if>
<#if CONFIG_DRV_GFX_CONTROLLER_TYPE = "ILI9488 (8-bit Parallel/SMC)">
<#include "/framework/gfx/driver/controller/ili9488/templates/drv_gfx_ili9488_definition.ftl">
<@DRV_GFX_ILI9488_DRIVER_DECLARATION IDX = 0/>
</#if>
</#if>

GFX_Result GFX_InitializeDriverList()
{
<#if CONFIG_USE_GFX_STACK = true>
<#if CONFIG_DRV_GFX_CONTROLLER_TYPE = "Low Cost Controllerless">
<#include "/framework/gfx/driver/controller/lcc/templates/drv_gfx_lcc_definition.ftl">
<@DRV_GFX_LCC_DRIVER_DEFINITION
	ID = GFX_DRIVER_COUNT
	IDX = 0/>
<#assign GFX_DRIVER_COUNT = GFX_DRIVER_COUNT + 1>
</#if>
<#if CONFIG_DRV_GFX_CONTROLLER_TYPE = "GLCD">
<#include "/framework/gfx/driver/controller/glcd/templates/drv_gfx_glcd_definition.ftl">
<@DRV_GFX_GLCD_DRIVER_DEFINITION
        ID = GFX_DRIVER_COUNT
        IDX = 0/>
<#assign GFX_DRIVER_COUNT = GFX_DRIVER_COUNT + 1>
</#if>
<#if CONFIG_DRV_GFX_CONTROLLER_TYPE = "Epson S1D13517">
<#include "/framework/gfx/driver/controller/s1d13517/templates/drv_gfx_s1d13517_definition.ftl">
<@DRV_GFX_S1D13517_DRIVER_DEFINITION
	ID = GFX_DRIVER_COUNT
	IDX = 0/>
<#assign GFX_DRIVER_COUNT = GFX_DRIVER_COUNT + 1>
</#if>
<#if CONFIG_DRV_GFX_CONTROLLER_TYPE = "SSD1926">
<#include "/framework/gfx/driver/controller/ssd1926/templates/drv_gfx_ssd1926_definition.ftl">
<@DRV_GFX_SSD1926_DRIVER_DEFINITION
	ID = GFX_DRIVER_COUNT
	IDX = 0/>
<#assign GFX_DRIVER_COUNT = GFX_DRIVER_COUNT + 1>
</#if>
<#if CONFIG_DRV_GFX_CONTROLLER_TYPE = "SSD1963">
<#include "/framework/gfx/driver/controller/ssd1963/templates/drv_gfx_ssd1963_definition.ftl">
<@DRV_GFX_SSD1963_DRIVER_DEFINITION
    ID = GFX_DRIVER_COUNT
    IDX = 0/>
<#assign GFX_DRIVER_COUNT = GFX_DRIVER_COUNT + 1>
</#if>
<#if CONFIG_DRV_GFX_CONTROLLER_TYPE = "OTM2201A">
<#include "/framework/gfx/driver/controller/otm2201a/templates/drv_gfx_otm2201a_definition.ftl">
<@DRV_GFX_OTM2201A_DRIVER_DEFINITION
	ID = GFX_DRIVER_COUNT
	IDX = 0/>
<#assign GFX_DRIVER_COUNT = GFX_DRIVER_COUNT + 1>
</#if>
<#if CONFIG_DRV_GFX_CONTROLLER_TYPE = "ILI9488 (SPI 4-LINE)">
<#include "/framework/gfx/driver/controller/ili9488/templates/drv_gfx_ili9488_definition.ftl">
<@DRV_GFX_ILI9488_DRIVER_DEFINITION
	ID = GFX_DRIVER_COUNT
	IDX = 0/>
<#assign GFX_DRIVER_COUNT = GFX_DRIVER_COUNT + 1>
</#if>
<#if CONFIG_DRV_GFX_CONTROLLER_TYPE = "ILI9488 (16-bit Parallel/SMC)">
<#include "/framework/gfx/driver/controller/ili9488/templates/drv_gfx_ili9488_definition.ftl">
<@DRV_GFX_ILI9488_DRIVER_DEFINITION
	ID = GFX_DRIVER_COUNT
	IDX = 0/>
<#assign GFX_DRIVER_COUNT = GFX_DRIVER_COUNT + 1>
</#if>
<#if CONFIG_DRV_GFX_CONTROLLER_TYPE = "ILI9488 (8-bit Parallel/SMC)">
<#include "/framework/gfx/driver/controller/ili9488/templates/drv_gfx_ili9488_definition.ftl">
<@DRV_GFX_ILI9488_DRIVER_DEFINITION
	ID = GFX_DRIVER_COUNT
	IDX = 0/>
<#assign GFX_DRIVER_COUNT = GFX_DRIVER_COUNT + 1>
</#if>
</#if>

    return GFX_SUCCESS;
}
