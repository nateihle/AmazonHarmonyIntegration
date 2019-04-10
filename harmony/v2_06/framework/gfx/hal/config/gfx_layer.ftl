config GFX_RENDER_LAYERS_NUMBER_GT_${INSTANCE+1}
    bool
    depends on GFX_CONTEXT_ENABLED
<#if INSTANCE != 0>
    default n if GFX_RENDER_LAYERS_NUMBER_GT_${INSTANCE} = n
</#if>	
    default n if GFX_CONTEXT_MAX_LAYERS = ${INSTANCE+1}
	default y
	
config GFX_RENDER_LAYER_${INSTANCE}
    depends on GFX_CONTEXT_ENABLED
<#if INSTANCE != 0>
                 && GFX_RENDER_LAYERS_NUMBER_GT_${INSTANCE}
</#if>
    bool
    default y
    ---help---
    IDH_HTML_Graphics_Driver_Library
    ---endhelp---

ifblock GFX_RENDER_LAYER_${INSTANCE}

menu "Layer ${INSTANCE}"
depends on GFX_CONTEXT_ENABLED

config GFX_RENDER_LAYER_${INSTANCE}_VISIBLE
    bool "Visible"
    default y
    ---help---
    IDH_HTML_Graphics_Driver_Library
    ---endhelp---

config GFX_RENDER_LAYER_${INSTANCE}_POSITION_X
    int "Position X"
    default 0
    ---help---
    IDH_HTML_Graphics_Driver_Library
    ---endhelp---

config GFX_RENDER_LAYER_${INSTANCE}_POSITION_Y
    int "Position Y"
    default 0
    ---help---
    IDH_HTML_Graphics_Driver_Library
    ---endhelp---

config GFX_RENDER_LAYER_${INSTANCE}_WIDTH
    int "Width"
	default 320
    default DRV_GFX_DISPLAY_WIDTH
	range 1 9999
    ---help---
    IDH_HTML_Graphics_Driver_Library
    ---endhelp---
 
config GFX_RENDER_LAYER_${INSTANCE}_HEIGHT
    int "Height"
	default 200
    default DRV_GFX_DISPLAY_HEIGHT
	range 1 9999
    ---help---
    IDH_HTML_Graphics_Driver_Library
    ---endhelp---

config GFX_RENDER_LAYER_${INSTANCE}_COLOR_MODE
    string "Color Mode"
    range GFX_LAYER_COLOR_MODE
    default "GFX_COLOR_MODE_RGBA_8888"
    ---help---
    IDH_HTML_SYSTEM_Library_Interface
    ---endhelp---

ifblock DRV_GFX_CONTROLLER_HAS_MANAGED_BUFFERS = n
	
config GFX_RENDER_LAYER_${INSTANCE}_BUFFER_COUNT
    int "Buffer Count"
	default 1
	range 1 GFX_CONTEXT_MAX_LAYERS
    ---help---
    IDH_HTML_Graphics_Driver_Library
    ---endhelp---
	
endif
	
endmenu

endif