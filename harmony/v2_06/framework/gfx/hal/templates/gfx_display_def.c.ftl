#include "gfx/hal/inc/gfx_display.h"
#include "gfx/hal/inc/gfx_common.h"

<#macro BOOL_TO_INT
	VAL
	COMMENT>
<#if VAL == true>
            1, //  ${COMMENT}
<#else>
            0, //  ${COMMENT}
</#if>
</#macro>

<#macro DRV_GFX_DISPLAY_DEF 
	IDX>
    <#noparse>{</#noparse>
	    "${CONFIG_DRV_GFX_DISPLAY_DESC}",  // description
<#if CONFIG_LIBARIA_CONTEXT_COLOR_MODE??>
		${CONFIG_LIBARIA_CONTEXT_COLOR_MODE},  // color mode
<#elseif CONFIG_SEGGER_EMWIN_LCC_COLOR_DEPTH?? && CONFIG_SEGGER_EMWIN_LCC_COLOR_DEPTH?has_content>
        ${CONFIG_SEGGER_EMWIN_LCC_COLOR_DEPTH},  // color mode
<#else>
		GFX_COLOR_MODE_RGB_565, // default color mode
</#if>
		<#noparse>{</#noparse>
			0,  // x position (always 0)
			0,  // y position (always 0)
			${CONFIG_DRV_GFX_DISPLAY_WIDTH},  // display width
			${CONFIG_DRV_GFX_DISPLAY_HEIGHT}, // display height
		<#noparse>},</#noparse>
		<#noparse>{</#noparse>
		    ${CONFIG_DRV_GFX_DISPLAY_DATA_WIDTH},  // data bus width
		    <#noparse>{</#noparse>
				${CONFIG_DRV_GFX_DISPLAY_HOR_PULSE_WIDTH},  // horizontal pulse width
				${CONFIG_DRV_GFX_DISPLAY_HOR_BACK_PORCH},  // horizontal back porch
				${CONFIG_DRV_GFX_DISPLAY_HOR_FRONT_PORCH},  // horizontal front porch
		    <#noparse>},</#noparse>
		    <#noparse>{</#noparse>
				${CONFIG_DRV_GFX_DISPLAY_VER_PULSE_WIDTH},  // vertical pulse width
				${CONFIG_DRV_GFX_DISPLAY_VER_BACK_PORCH},  // vertical back porch
				${CONFIG_DRV_GFX_DISPLAY_VER_FRONT_PORCH},  // vertical front porch
		    <#noparse>},</#noparse>
			${CONFIG_DRV_GFX_DISPLAY_INV_LSHIFT},  // inverted left shift
		<#noparse>},</#noparse>
	<#noparse>},</#noparse>
</#macro>

GFX_DisplayInfo GFX_DisplayInfoList[] =
{
<#if CONFIG_USE_GFX_STACK = true>
	<@DRV_GFX_DISPLAY_DEF IDX = 0/>
<#else>
	{ },
</#if>
};