config DRV_AK4384_INSTANCES_NUMBER_GT_${INSTANCE+1}
    depends on SELECT_DRV_AK4384    
    depends on USE_DRV_CODEC_AK4384
    depends on USE_DRV_AK4384_BIT_BANGED_SPI_CONTROL_INTERFACE       
    bool
<#if INSTANCE != 0>
	default n if DRV_AK4384_INSTANCES_NUMBER_GT_${INSTANCE} = n     
</#if>	
	default n if DRV_AK4384_INSTANCES_NUMBER = ${INSTANCE+1}
	default y
	
config DRV_CODEC_AK4384_INST_IDX${INSTANCE}
    depends on SELECT_DRV_AK4384    
    depends on USE_DRV_CODEC_AK4384 
    depends on USE_DRV_AK4384_BIT_BANGED_SPI_CONTROL_INTERFACE       
<#if INSTANCE != 0>
	             && DRV_AK4384_INSTANCES_NUMBER_GT_${INSTANCE}
</#if>
    bool "Codec AK4384 Driver Instance ${INSTANCE}"
    default y
    ---help---
    IDH_HTML_DRV_AK4384_INSTANCES_NUMBER
    ---endhelp---


ifblock DRV_CODEC_AK4384_INST_IDX${INSTANCE}   

config DRV_AK4384_CONTROL_CLOCK_IDX${INSTANCE}
    	depends on SELECT_DRV_AK4384    
	depends on USE_DRV_CODEC_AK4384
	depends on USE_DRV_AK4384_BIT_BANGED_SPI_CONTROL_INTERFACE
    	range 1 5000000
	default 1000


config DRV_AK4384_TIMER_DRIVER_MODULE_INDEX_IDX${INSTANCE}
    	depends on SELECT_DRV_AK4384    
	depends on USE_DRV_CODEC_AK4384
	depends on USE_DRV_AK4384_BIT_BANGED_SPI_CONTROL_INTERFACE		
	select USE_DRV_TMR_NEEDED
	int "Timer driver(used for bit banging)  instance"
	range 0	0 if TMR_NUMBER_OF_MODULES = "1"
	range 0 1 if TMR_NUMBER_OF_MODULES = "2"
	range 0 2 if TMR_NUMBER_OF_MODULES = "3"
	range 0 3 if TMR_NUMBER_OF_MODULES = "4"
	range 0 4 if TMR_NUMBER_OF_MODULES = "5"
	range 0 5 if TMR_NUMBER_OF_MODULES = "6"
	range 0 6 if TMR_NUMBER_OF_MODULES = "7"
	range 0 7 if TMR_NUMBER_OF_MODULES = "8"	
	range 0 8 if TMR_NUMBER_OF_MODULES = "9"	
	range 0 9 if TMR_NUMBER_OF_MODULES = "10"		
	default 0
    	---help---
    	IDH_HTML_DRV_AK4384_TIMER_DRIVER_MODULE_INDEX
    	---endhelp---

config DRV_AK4384_TIMER_PERIOD_IDX${INSTANCE}
    	depends on SELECT_DRV_AK4384    
	depends on USE_DRV_CODEC_AK4384
	depends on USE_DRV_AK4384_BIT_BANGED_SPI_CONTROL_INTERFACE		

endif
config DRV_AK4384_I2S_DRIVER_MODULE_INDEX_IDX${INSTANCE}
    	depends on SELECT_DRV_AK4384    
	depends on USE_DRV_CODEC_AK4384	
	select USE_DRV_I2S_NEEDED
	string "I2S driver(used for data interface) instance"
	range DRV_I2S_INDEX	
	default "DRV_I2S_INDEX_0"
    	---help---
    	IDH_HTML_DRV_AK4384_INIT
    	---endhelp---

ifblock USE_DRV_AK4384_BIT_BANGED_SPI_CONTROL_INTERFACE=y && SELECT_DRV_AK4384=y
file DRV_CODEC_AK4384_C "$HARMONY_VERSION_PATH/framework/driver/codec/ak4384/src/dynamic/drv_ak4384_bit_banged_control_interface.c" to "$PROJECT_SOURCE_FILES/framework/driver/codec/ak4384/src/dynamic/drv_ak4384_bit_banged_control_interface.c"
endif

