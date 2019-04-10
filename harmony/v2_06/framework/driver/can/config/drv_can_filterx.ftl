config DRV_CAN_FILTER_NUMBER_GT_${INSTANCE+1}
    bool
    depends on USE_DRV_CAN
<#if INSTANCE != 0>
	default n if DRV_CAN_FILTER_NUMBER_GT_${INSTANCE} = n     
</#if>	
	default n if DRV_CAN_FILTERS_NUMBER0 = ${INSTANCE+1}
	default y
	
config DRV_CAN_FILT_IDX${INSTANCE}
    depends on USE_DRV_CAN 
<#if INSTANCE != 0>
	             && DRV_CAN_FILTER_NUMBER_GT_${INSTANCE}
</#if>
    bool "CAN Driver 0 Filter ${INSTANCE}"
    default n

ifblock DRV_CAN_FILT_IDX${INSTANCE}


config DRV_CAN_FILTERIDENTIFIER_IDX${INSTANCE}
	depends on USE_DRV_CAN
        string "CAN FILTER Identifier Mode"
        range CAN_ID_TYPE
	default "CAN_SID"
        ---help---
        IDH_HTML_CAN_ID_TYPE
        ---endhelp---

config DRV_CAN_FILTERACCEPTANCE_IDX${INSTANCE}
	depends on USE_DRV_CAN
        string "CAN Filter Acceptance Value"
	default "0x7fff"
	
endif
