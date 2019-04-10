config DRV_CTR_COUNTER_INSTANCES_NUMBER_GT_${INSTANCE+1}
    bool
    depends on USE_DRV_CTR
<#if INSTANCE != 0>
	default n if DRV_CTR_COUNTER_INSTANCES_NUMBER_GT_${INSTANCE} = n     
</#if>	
	default n if DRV_CTR_COUNTER_MAX = ${INSTANCE+1}
	default y
	
menu "Counter ${INSTANCE} Configuration"
    depends on USE_DRV_CTR

config DRV_CTR_M_${INSTANCE}
	hex "M"
	depends on USE_DRV_CTR
	default 0x000000
	range 0x000000 0xFFFFFF

config DRV_CTR_N_${INSTANCE}
	hex "N"
	depends on USE_DRV_CTR
	default 0x000000
	range 0x000000 0xFFFFFF

config DRV_CTR_LSB_${INSTANCE}
	hex "LSB"
	depends on USE_DRV_CTR
	default 0x00
	range 0x00 0x03

config DRV_CTR_US_MODE_${INSTANCE}
	string "Mode"
	depends on USE_DRV_CTR
	default "MicroSec Mode"
	range CTR_MODE
	
endmenu

