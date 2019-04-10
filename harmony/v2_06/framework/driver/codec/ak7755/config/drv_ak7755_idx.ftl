config DRV_AK7755_INSTANCES_NUMBER_GT_${INSTANCE+1}
    depends on SELECT_DRV_AK7755
	depends on USE_DRV_CODEC_AK7755           
    bool
<#if INSTANCE != 0>
	default n if DRV_AK7755_INSTANCES_NUMBER_GT_${INSTANCE} = n     
</#if>	
	default n if DRV_AK7755_INSTANCES_NUMBER = ${INSTANCE+1}
	default y
	
config DRV_CODEC_AK7755_INST_IDX${INSTANCE}
	depends on SELECT_DRV_AK7755
    depends on USE_DRV_CODEC_AK7755            
<#if INSTANCE != 0>
	             && DRV_AK7755_INSTANCES_NUMBER_GT_${INSTANCE}
</#if>
    bool "Codec AK7755 Driver Instance ${INSTANCE}"
    default y
    ---help---
    IDH_HTML_DRV_AK7755_INSTANCES_NUMBER
    ---endhelp---


config DRV_AK7755_I2S_DRIVER_MODULE_INDEX_IDX${INSTANCE}
	depends on SELECT_DRV_AK7755
	depends on USE_DRV_CODEC_AK7755	
	select USE_DRV_I2S_NEEDED
	string "I2S driver(used for data interface) instance"
	range DRV_I2S_INDEX	
	default "DRV_I2S_INDEX_0"
    	---help---
    	IDH_HTML_DRV_AK7755_INIT
    	---endhelp---



config DRV_AK7755_I2C_DRIVER_MODULE_INDEX_IDX${INSTANCE}
	depends on SELECT_DRV_AK7755
	depends on USE_DRV_CODEC_AK7755	
	select USE_DRV_I2C_NEEDED
	string "Dynamic I2C driver(used for control interface) instance"
	range DRV_I2C_INDEX
	default "DRV_I2C_INDEX_0"
	---help---
    	IDH_HTML_DRV_AK7755_INIT
    	---endhelp---
	
	
	
ifblock USE_DRV_CODEC_AK7755=y && SELECT_DRV_AK7755=y
file DRV_CODEC_AK7755_C "$HARMONY_VERSION_PATH/framework/driver/codec/ak7755/src/dynamic/drv_ak7755.c" to "$PROJECT_SOURCE_FILES/framework/driver/codec/ak7755/src/dynamic/drv_ak7755.c"
endif

