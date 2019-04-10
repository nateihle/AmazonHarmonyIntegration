config DRV_CAN_FILTER3_NUMBER_GT_${INSTANCE+1}
    bool
    depends on USE_DRV_CAN
<#if INSTANCE != 0>
	default n if DRV_CAN_FILTER3_NUMBER_GT_${INSTANCE} = n     
</#if>	
	default n if DRV_CAN_FILTERS_NUMBER3 = ${INSTANCE+1}
	default y
	
config DRV_CAN_FILT3_IDX${INSTANCE}
    depends on USE_DRV_CAN 
<#if INSTANCE != 0>
	             && DRV_CAN_FILTER3_NUMBER_GT_${INSTANCE}
</#if>
    bool "CAN Driver 3 Filter ${INSTANCE}"
    default y

ifblock DRV_CAN_FILT3_IDX${INSTANCE}


config DRV_CAN_FILTERIDENTIFIER3_IDX${INSTANCE}
	depends on USE_DRV_CAN
        string "CAN FILTER Identifier Mode"
        range CAN_ID_TYPE
	default "CAN_SID"
        ---help---
        IDH_HTML_CAN_ID_TYPE
        ---endhelp---

config DRV_CAN_FILTERACCEPTANCE3_IDX${INSTANCE}
	depends on USE_DRV_CAN
        string "CAN Filter Acceptance Value"
	default "0x7fff"
	
endif
