config DRV_CAN_CHANNEL2_NUMBER_GT_${INSTANCE+1}
    bool
    depends on USE_DRV_CAN
<#if INSTANCE != 0>
	default n if DRV_CAN_CHANNEL2_NUMBER_GT_${INSTANCE} = n     
</#if>	
	default n if DRV_CAN_CHANNELS_NUMBER2 = ${INSTANCE+1}
	default y
	
config DRV_CAN_CHANNEL2_IDX${INSTANCE}
    depends on USE_DRV_CAN 
<#if INSTANCE != 0>
	             && DRV_CAN_CHANNEL2_NUMBER_GT_${INSTANCE}
</#if>
    bool "CAN Driver 2 Channel ${INSTANCE}"
    default y

ifblock DRV_CAN_CHANNEL2_IDX${INSTANCE}

config DRV_CAN_CHANNELTYPE2_IDX${INSTANCE}
	depends on USE_DRV_CAN
        string "Channel Transfer Type"
        range DRV_CAN_CHANNEL_TYPE
	default "CAN_TX_RTR_DISABLED"

config DRV_CAN_CHANNEL_USEEVENTS2_IDX${INSTANCE}
        depends on USE_DRV_CAN
        bool "Use Tx/Rx Events for this channel?"
        default n
        ---help---
        Checking this enables transfer interrupts for the module
        ---endhelp---

ifblock DRV_CAN_CHANNEL_USEEVENTS2_IDX${INSTANCE}

config DRV_CAN_CHANNELEVENT2_IDX${INSTANCE}
	depends on USE_DRV_CAN
        string "Channel Event Type"
        range CAN_CHANNEL_EVENT
	default "CAN_RX_CHANNEL_NOT_EMPTY"
        ---help---
        IDH_HTML_CAN_CHANNEL_EVENT
        ---endhelp---
endif

config DRV_CAN_CHANNELSize2_IDX${INSTANCE}
	depends on USE_DRV_CAN
        int "Channel Buffer Size"
        range 1 32
	default 1

config DRV_CAN_CHANNEL_FILTERMASK2_IDX${INSTANCE}
        depends on USE_DRV_CAN
        default y if DRV_CAN_CHANNELTYPE2_IDX${INSTANCE} = "CAN_RX_DATA_ONLY" || DRV_CAN_CHANNELTYPE2_IDX${INSTANCE} = "CAN_RX_FULL_RECEIVE"
        bool "Use Filter/Mask for this channel?"
        ---help---
        Must be Enabled if in CAN Receive Mode.
        ---endhelp---

ifblock DRV_CAN_CHANNEL_FILTERMASK2_IDX${INSTANCE}

config DRV_CAN_CHANNEL_FILTER2_IDX${INSTANCE}
        depends on USE_DRV_CAN
	string "Channel Filter"
        range CAN_FILTER
        default "CAN_FILTER0"
        ---help---
        IDH_HTML_CAN_FILTER
        ---endhelp---

config DRV_CAN_CHANNEL_MASK2_IDX${INSTANCE}
        depends on USE_DRV_CAN
	string "Channel Mask Filter"
        range CAN_FILTER_MASK
        default "CAN_FILTER_MASK0"
        ---help---
        IDH_HTML_CAN_FILTER_MASK
        ---endhelp---
endif  
endif
