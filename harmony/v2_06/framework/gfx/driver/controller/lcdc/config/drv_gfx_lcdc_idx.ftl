config DRV_GFX_LCDC_LAYERS_NUMBER_GT_${INSTANCE+1}
    bool
    depends on USE_DRV_GFX_LCDC
<#if INSTANCE != 0>
    default n if DRV_GFX_LCDC_LAYERS_NUMBER_GT_${INSTANCE} = n
</#if>	
    default n if DRV_GFX_LCDC_LAYERS_NUMBER = ${INSTANCE+1}
	default y
	
config DRV_GFX_LCDC_LAYER_${INSTANCE}
    depends on USE_DRV_GFX_LCDC
<#if INSTANCE != 0>
                 && DRV_GFX_LCDC_LAYERS_NUMBER_GT_${INSTANCE}
</#if>
    bool 
	default y
    ---help---
    IDH_HTML_Graphics_Driver_Library
    ---endhelp---

