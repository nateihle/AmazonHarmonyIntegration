<#macro global_event_trigger_hconfig module_name>
config APP_${module_name}_USE_GLOBAL_EVENT${INSTANCE}
    bool "Trigger global event upon callback?"
    select USE_APP_GLOBAL_EVENT
    default n

ifblock APP_${module_name}_USE_GLOBAL_EVENT${INSTANCE}

config APP_${module_name}_GLOBAL_EVENT_NAME${INSTANCE}
    string "Specify a unique event name"
    default "global_event_${module_name?lower_case}"

config APP_${module_name}_GLOBAL_EVENT_COUNT${INSTANCE}
    int "# Callback calls per event"
    default 1
    
add "APP_${module_name}_GLOBAL_EVENT_NAME${INSTANCE}" to list GLOBAL_EVENT_NAMES
    
endif
</#macro>

<#macro global_event_trigger_no_count_hconfig module_name>
config APP_${module_name}_USE_GLOBAL_EVENT${INSTANCE}
    bool "Trigger global event?"
    select USE_APP_GLOBAL_EVENT
    default n

ifblock APP_${module_name}_USE_GLOBAL_EVENT${INSTANCE}

config APP_${module_name}_GLOBAL_EVENT_NAME${INSTANCE}
    string "Specify a unique event name"

comment "**** NOTE: Must specify a global event name! ****"
    
add "APP_${module_name}_GLOBAL_EVENT_NAME${INSTANCE}" to list GLOBAL_EVENT_NAMES
    
endif
</#macro>

<#macro global_event_triggered_hconfig module_name action>
config APP_${module_name}_UPON_EVENT${INSTANCE}
    bool "${action} ${module_name} upon global event?"
    select USE_APP_GLOBAL_EVENT
    default n

ifblock APP_${module_name}_UPON_EVENT${INSTANCE}
    
config APP_${module_name}_UPON_EVENT_NAME${INSTANCE}
    string "Specify the global event name"

comment "**** NOTE: Must specify a global event name! ****"

config APP_${module_name}_UPON_EVENT_COUNT${INSTANCE}
    int "Event count per ${action}"
    default 1
    
endif
</#macro>

<#macro global_event_trigger eventName><#nt>global_events.${eventName} = true;</#macro>

<#macro global_event_triggered eventName>
global_event_triggered(&global_events.${eventName})</#macro>
