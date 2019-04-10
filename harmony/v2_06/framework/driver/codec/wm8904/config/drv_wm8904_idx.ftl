config DRV_WM8904_INSTANCES_NUMBER_GT_${INSTANCE+1}
    depends on SELECT_DRV_WM8904
    depends on USE_DRV_CODEC_WM8904           
    bool
<#if INSTANCE != 0>
    default n if DRV_WM8904_INSTANCES_NUMBER_GT_${INSTANCE} = n     
</#if>	
    default n if DRV_WM8904_INSTANCES_NUMBER = ${INSTANCE+1}
    default y
	
config DRV_CODEC_WM8904_INST_IDX${INSTANCE}
    depends on SELECT_DRV_WM8904
    depends on USE_DRV_CODEC_WM8904            
<#if INSTANCE != 0>
	             && DRV_WM8904_INSTANCES_NUMBER_GT_${INSTANCE}
</#if>
    bool
    default y

ifblock DRV_WM8904_I2S_DRIVER_MODULE_INDEX_IDX${INSTANCE} = "DRV_I2S_INDEX_0"
config DRV_WM8904_I2S_NEEDED0
    depends on SELECT_DRV_WM8904
    depends on USE_DRV_CODEC_WM8904
    bool
endif

ifblock DRV_WM8904_I2S_DRIVER_MODULE_INDEX_IDX${INSTANCE} = "DRV_I2S_INDEX_1"
config DRV_WM8904_I2S_NEEDED1
    depends on SELECT_DRV_WM8904
    depends on USE_DRV_CODEC_WM8904
    bool
endif

config DRV_WM8904_I2S_DRIVER_MODULE_INDEX_IDX${INSTANCE}
    depends on SELECT_DRV_WM8904
    depends on USE_DRV_CODEC_WM8904	
    select USE_DRV_I2S_NEEDED
    select DRV_WM8904_I2S_NEEDED0
    select DRV_WM8904_I2S_NEEDED1
    string "I2S driver(used for data interface) instance"
    range DRV_I2S_INDEX	
    default "DRV_I2S_INDEX_0"
    ---help---
    IDH_HTML_DRV_WM8904_INIT
    ---endhelp---

config DRV_WM8904_I2C_DRIVER_MODULE_INDEX_IDX${INSTANCE}
    depends on SELECT_DRV_WM8904
    depends on USE_DRV_CODEC_WM8904	
    select USE_DRV_I2C_NEEDED
    string "Dynamic I2C driver(used for control interface) instance"
    range DRV_I2C_INDEX
    default "DRV_I2C_INDEX_0"
    ---help---
    IDH_HTML_DRV_WM8904_INIT
    ---endhelp---
	
ifblock DRV_WM8904_TMR_DRIVER_MODULE_INDEX_IDX${INSTANCE} = "DRV_TMR_INDEX_0"
config DRV_WM8904_TMR_NEEDED0
    depends on SELECT_DRV_WM8904
    depends on USE_DRV_CODEC_WM8904
    bool
endif

ifblock DRV_WM8904_TMR_DRIVER_MODULE_INDEX_IDX${INSTANCE} = "DRV_TMR_INDEX_1"
config DRV_WM8904_TMR_NEEDED1
    depends on SELECT_DRV_WM8904
    depends on USE_DRV_CODEC_WM8904
    bool
endif
	
ifblock DRV_WM8904_TMR_DRIVER_MODULE_INDEX_IDX${INSTANCE} = "DRV_TMR_INDEX_2"
config DRV_WM8904_TMR_NEEDED2
    depends on SELECT_DRV_WM8904
    depends on USE_DRV_CODEC_WM8904
    bool
endif

ifblock DRV_WM8904_TMR_DRIVER_MODULE_INDEX_IDX${INSTANCE} = "DRV_TMR_INDEX_3"
config DRV_WM8904_TMR_NEEDED3
    depends on SELECT_DRV_WM8904
    depends on USE_DRV_CODEC_WM8904
    bool
endif

config DRV_WM8904_TMR_DRIVER_MODULE_INDEX_IDX${INSTANCE}
    depends on SELECT_DRV_WM8904
    depends on USE_DRV_CODEC_WM8904	
    select USE_DRV_TMR_NEEDED
    select DRV_WM8904_TMR_NEEDED0
    select DRV_WM8904_TMR_NEEDED1
    select DRV_WM8904_TMR_NEEDED2
    select DRV_WM8904_TMR_NEEDED3
    string "Timer driver(used for driver timing) instance"
    range DRV_TMR_INDEX
    default "DRV_TMR_INDEX_0"
    ---help---
    IDH_HTML_DRV_WM8904_INIT
    ---endhelp---
	
ifblock USE_DRV_CODEC_WM8904=y && SELECT_DRV_WM8904=y
file DRV_CODEC_WM8904_C "$HARMONY_VERSION_PATH/framework/driver/codec/wm8904/src/dynamic/drv_wm8904.c" to "$PROJECT_SOURCE_FILES/framework/driver/codec/wm8904/src/dynamic/drv_wm8904.c"
endif

ifblock ( DRV_CODEC_WM8904_INST_IDX0 = y )
add "SYS_MODULE_OBJ drvwm8904Codec0;" to list SYSTEM_DEFINITIONS_H_OBJECTS
endif

