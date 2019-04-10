config DRV_CAN_FILTER2_NUMBER_GT_${INSTANCE+1}
    bool
    depends on USE_DRV_CAN
<#if INSTANCE != 0>
	default n if DRV_CAN_FILTER2_NUMBER_GT_${INSTANCE} = n     
</#if>	
	default n if DRV_CAN_FILTERS_NUMBER2 = ${INSTANCE+1}
	default y
	
config DRV_CAN_FILT2_IDX${INSTANCE}
    depends on USE_DRV_CAN 
<#if INSTANCE != 0>
	             && DRV_CAN_FILTER2_NUMBER_GT_${INSTANCE}
</#if>
    bool "CAN Driver 2 Filter ${INSTANCE}"
    default y

ifblock DRV_CAN_FILT2_IDX${INSTANCE}


config DRV_CAN_FILTERIDENTIFIER2_IDX${INSTANCE}
	depends on USE_DRV_CAN
        string "CAN FILTER Identifier Mode"
        range CAN_ID_TYPE
	default "CAN_SID"
        ---help---
        IDH_HTML_CAN_ID_TYPE
        ---endhelp---

config DRV_CAN_FILTERACCEPTANCE2_IDX${INSTANCE}
	depends on USE_DRV_CAN
        string "CAN Filter Acceptance Value"
	default "0x7fff"
	
endif
