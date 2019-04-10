config DRV_CAN_CHANNEL_NUMBER_GT_${INSTANCE+1}
    bool
    depends on USE_DRV_CAN
<#if INSTANCE != 0>
	default n if DRV_CAN_CHANNEL_NUMBER_GT_${INSTANCE} = n     
</#if>	
	default n if DRV_CAN_CHANNELS_NUMBER0 = ${INSTANCE+1}
	default y
	
config DRV_CAN_CHANNEL_IDX${INSTANCE}
    depends on USE_DRV_CAN 
<#if INSTANCE != 0>
	             && DRV_CAN_CHANNEL_NUMBER_GT_${INSTANCE}
</#if>
    bool "CAN Driver 0 Channel ${INSTANCE}"
    default y

ifblock DRV_CAN_CHANNEL_IDX${INSTANCE}

config DRV_CAN_CHANNELTYPE_IDX${INSTANCE}
	depends on USE_DRV_CAN
        string "Channel Transfer Type"
        range DRV_CAN_CHANNEL_TYPE
	default "CAN_TX_RTR_DISABLED"

config DRV_CAN_CHANNEL_USEEVENTS_IDX${INSTANCE}
        depends on USE_DRV_CAN
        bool "Use Tx/Rx Events for this channel?"
        default n
        ---help---
        Checking this enables transfer interrupts for the module
        ---endhelp---

ifblock DRV_CAN_CHANNEL_USEEVENTS_IDX${INSTANCE}

config DRV_CAN_CHANNELEVENT_IDX${INSTANCE}
	depends on USE_DRV_CAN
        string "Channel Event Type"
        range CAN_CHANNEL_EVENT
	default "CAN_RX_CHANNEL_NOT_EMPTY"
        ---help---
        IDH_HTML_CAN_CHANNEL_EVENT
        ---endhelp---
endif

config DRV_CAN_CHANNELSize_IDX${INSTANCE}
	depends on USE_DRV_CAN
        int "Channel Buffer Size"
        range 1 32
	default 1

config DRV_CAN_CHANNEL_FILTERMASK_IDX${INSTANCE}
        depends on USE_DRV_CAN
        default y if DRV_CAN_CHANNELTYPE_IDX${INSTANCE} = "CAN_RX_DATA_ONLY" || DRV_CAN_CHANNELTYPE_IDX${INSTANCE} = "CAN_RX_FULL_RECEIVE"
        bool "Use Filter/Mask for this channel?"
        comment "**** Warning: A Filter/Mask Must be Enabled if in CAN Receive Mode. ****"

ifblock DRV_CAN_CHANNEL_FILTERMASK_IDX${INSTANCE}

config DRV_CAN_CHANNEL_FILTER_IDX${INSTANCE}
        depends on USE_DRV_CAN
	string "Channel Filter"
        range CAN_FILTER
        default "CAN_FILTER0"
        ---help---
        IDH_HTML_CAN_FILTER
        ---endhelp---

config DRV_CAN_CHANNEL_MASK_IDX${INSTANCE}
        depends on USE_DRV_CAN
	string "Channel Mask Filter"
        range CAN_FILTER_MASK
        default "CAN_FILTER_MASK0"
        ---help---
        IDH_HTML_CAN_FILTER_MASK
        ---endhelp---
endif  
endif
