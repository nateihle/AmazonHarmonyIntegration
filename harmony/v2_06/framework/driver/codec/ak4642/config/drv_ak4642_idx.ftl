config DRV_AK4642_INSTANCES_NUMBER_GT_${INSTANCE+1}
    depends on SELECT_DRV_AK4642
    depends on USE_DRV_CODEC_AK4642           
    bool
<#if INSTANCE != 0>
	default n if DRV_AK4642_INSTANCES_NUMBER_GT_${INSTANCE} = n     
</#if>	
	default n if DRV_AK4642_INSTANCES_NUMBER = ${INSTANCE+1}
	default y
	
config DRV_CODEC_AK4642_INST_IDX${INSTANCE}
    depends on SELECT_DRV_AK4642
    depends on USE_DRV_CODEC_AK4642            
<#if INSTANCE != 0>
	             && DRV_AK4642_INSTANCES_NUMBER_GT_${INSTANCE}
</#if>
    bool "Codec AK4642 Driver Instance ${INSTANCE}"
    default y
    ---help---
    IDH_HTML_DRV_AK4642_INSTANCES_NUMBER
    ---endhelp---


config DRV_AK4642_I2S_DRIVER_MODULE_INDEX_IDX${INSTANCE}
	depends on SELECT_DRV_AK4642
	depends on USE_DRV_CODEC_AK4642	
	select USE_DRV_I2S_NEEDED
	string "I2S driver(used for data interface) instance"
	range DRV_I2S_INDEX	
	default "DRV_I2S_INDEX_0"
    	---help---
    	IDH_HTML_DRV_AK4642_INIT
    	---endhelp---



config DRV_AK4642_I2C_DRIVER_MODULE_INDEX_IDX${INSTANCE}
	depends on SELECT_DRV_AK4642
	depends on USE_DRV_CODEC_AK4642	
	select USE_DRV_I2C_NEEDED
	string "Dynamic I2C driver(used for control interface) instance"
	range DRV_I2C_INDEX
	default "DRV_I2C_INDEX_0"
	---help---
    	IDH_HTML_DRV_AK4642_INIT
    	---endhelp---
	
	
	
ifblock USE_DRV_CODEC_AK4642=y && SELECT_DRV_AK4642=y
file DRV_CODEC_AK4642_C "$HARMONY_VERSION_PATH/framework/driver/codec/ak4642/src/dynamic/drv_ak4642.c" to "$PROJECT_SOURCE_FILES/framework/driver/codec/ak4642/src/dynamic/drv_ak4642.c"
endif

