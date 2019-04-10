config APP_INSTANCES_NUMBER_GT_${INSTANCE+1}
    bool
<#if INSTANCE != 0>
	default n if APP_INSTANCES_NUMBER_GT_${INSTANCE} = n     
</#if>	
    default n if APP_INSTANCES = ${INSTANCE+1}
    default y

config APP_IDX_${INSTANCE}
<#if INSTANCE != 0>
    depends on APP_INSTANCES_NUMBER_GT_${INSTANCE}
</#if>
    bool
    default y

ifblock APP_IDX_${INSTANCE}

menu "Application ${INSTANCE} Configuration"

config APP_MENU_IDX${INSTANCE}
    bool "Use Application Configuration?"
    default y
    persistent

config APP_NAME_${INSTANCE}
    string "Application Name"
    <#if INSTANCE == 0>
    default "app"
    <#else>
    default "app${INSTANCE}"
    </#if>

comment "Application name must be valid C-language identifiers and should be short and lowercase."

menu "RTOS Configuration"
    depends on USE_3RDPARTY_RTOS

config APP_TASK_SIZE_IDX${INSTANCE}
    int "Task Size"
    default 1024

config APP_TASK_PRIORITY_IDX${INSTANCE}
    int "Task Priority"
    default 1

config APP_TASK_USE_DELAY_IDX${INSTANCE}
    bool "Use Task Delay?"
    default y

config APP_TASK_DELAY_IDX${INSTANCE}
    int "Task Delay"
    depends on APP_TASK_USE_DELAY_IDX${INSTANCE}
    default 1000

endmenu
endmenu

endif

template APP_H_${INSTANCE} "$HARMONY_VERSION_PATH/utilities/mhc/templates/app/app.h.ftl" to "$PROJECT_HEADER_FILES/app/$SYMBOL(APP_NAME_${INSTANCE}, ${INSTANCE}).h" if APP_IDX_${INSTANCE}
template APP_C_${INSTANCE} "$HARMONY_VERSION_PATH/utilities/mhc/templates/app/app.c.ftl" to "$PROJECT_SOURCE_FILES/app/$SYMBOL(APP_NAME_${INSTANCE}, ${INSTANCE}).c" if APP_IDX_${INSTANCE}
