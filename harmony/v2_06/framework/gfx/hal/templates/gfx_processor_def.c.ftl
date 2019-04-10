#include "gfx/hal/inc/gfx_processor_interface.h"

<#global GFX_PROCESSOR_COUNT = 0>
<#if CONFIG_USE_GFX_STACK = true>
<#if CONFIG_DRV_GFX_PROCESSOR_TYPE = "NANO 2D">
<#include "/framework/gfx/driver/processor/nano2d/templates/drv_gfx_nano2d_definition.ftl">
<@DRV_GFX_NANO2D_PROCESSOR_DECLARATION IDX = 0/>
</#if>
</#if>

GFX_Result GFX_InitializeProcessorList()
{
<#if CONFIG_USE_GFX_STACK = true>
<#if CONFIG_DRV_GFX_PROCESSOR_TYPE = "NANO 2D">
<#include "/framework/gfx/driver/processor/nano2d/templates/drv_gfx_nano2d_definition.ftl">
<@DRV_GFX_NANO2D_PROCESSOR_DEFINITION
	ID = GFX_PROCESSOR_COUNT
	IDX = 0/>
<#global GFX_PROCESSOR_COUNT = GFX_PROCESSOR_COUNT + 1>
</#if>
</#if>

    return GFX_SUCCESS;
}