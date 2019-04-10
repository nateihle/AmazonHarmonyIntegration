<#macro global_pipe_create_hconfig module_name>

config APP_${module_name}_CREATE_GLOBAL_PIPE${INSTANCE}
    bool "Send data to pipe"
    default n
    
config APP_${module_name}_CREATE_GLOBAL_PIPE_NAME${INSTANCE}
    string "Specify a unique name for the data pipe"
    default "pipe_${module_name?lower_case}"
    depends on APP_${module_name}_CREATE_GLOBAL_PIPE${INSTANCE}
    
</#macro>

<#macro global_pipe_use_hconfig module_name>
    
config APP_${module_name}_USE_GLOBAL_PIPE${INSTANCE}
    bool "Get data from pipe"
    default n
    
config APP_${module_name}_USE_GLOBAL_PIPE_NAME${INSTANCE}
    string "Specify the pipe name"
    default "pipe_${module_name?lower_case}"
    depends on APP_${module_name}_USE_GLOBAL_PIPE${INSTANCE}
    
</#macro>
