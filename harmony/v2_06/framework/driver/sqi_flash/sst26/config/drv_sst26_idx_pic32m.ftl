config DRV_SST26_INSTANCES_NUMBER_GT_${INSTANCE+1}
    depends on USE_DRV_SST26
    bool
<#if INSTANCE != 0>
	default n if DRV_SST26_INSTANCES_NUMBER_GT_${INSTANCE} = n     
</#if>	
	default n if DRV_SST26_INSTANCES_NUMBER = ${INSTANCE+1}
	default y
	
config DRV_SST26_INST_IDX${INSTANCE}
    depends on USE_DRV_SST26
<#if INSTANCE != 0>
	             && DRV_SST26_INSTANCES_NUMBER_GT_${INSTANCE}
</#if>
    bool "SST26 Driver Instance ${INSTANCE}"
    default y
    ---help---
    IDH_HTML_DRV_SST26_INSTANCES_NUMBER
    ---endhelp---

ifblock DRV_SST26_INST_IDX${INSTANCE}

config DRV_SST26_CLIENTS_NUMBER_IDX${INSTANCE}
    int "Number of Driver Clients"
    depends on USE_DRV_SST26
    default 1
    ---help---
      IDH_HTML_DRV_SST26_CLIENTS_NUMBER
    ---endhelp---
    
config DRV_SST26_BUFFER_OBJECT_NUMBER_IDX${INSTANCE}
    int "Number of Buffer Objects"
    depends on USE_DRV_SST26
    range 1 10
    default 5
    ---help---
      IDH_HTML_DRV_SST26_BUFFER_OBJECT_NUMBER
    ---endhelp---

config DRV_SST26_SQI_DEVICE_IDX${INSTANCE}
    int "Chip Select Line to use"
    depends on USE_DRV_SST26
    range 0 1
    default 1
    set USE_DRV_SQI_DEVICE_0 optionally to y if DRV_SST26_SQI_DEVICE_IDX${INSTANCE} = 0
    set USE_DRV_SQI_DEVICE_1 optionally to y if DRV_SST26_SQI_DEVICE_IDX${INSTANCE} = 1
    ---help---
    IDH_HTML_SST26_DEVICE_IDX
    ---endhelp---

menu "RTOS Configuration (Instance ${INSTANCE})"
    depends on USE_DRV_SST26
    depends on USE_3RDPARTY_RTOS

config DRV_SST26_RTOS_IDX${INSTANCE}
    string "Run This Driver Instance As"
    range 3RDPARTY_RTOS_SYS_TASKS_OPTIONS
    default "Combined with System Tasks"

config DRV_SST26_IDX${INSTANCE}_RTOS_TASK_SIZE
    int "Task Size"
    depends on DRV_SST26_RTOS_IDX${INSTANCE} = "Standalone"
    default 1024

config DRV_SST26_IDX${INSTANCE}_RTOS_TASK_PRIORITY
    int "Task Priority"
    depends on DRV_SST26_RTOS_IDX${INSTANCE} = "Standalone"
    default 1

config DRV_SST26_IDX${INSTANCE}_RTOS_USE_DELAY
    bool "Use Task Delay?"
    depends on DRV_SST26_RTOS_IDX${INSTANCE} = "Standalone"
    default y

config DRV_SST26_IDX${INSTANCE}_RTOS_DELAY
    int "Task Delay"
    depends on DRV_SST26_RTOS_IDX${INSTANCE} = "Standalone"
    depends on DRV_SST26_IDX${INSTANCE}_RTOS_USE_DELAY
    default 1000
endmenu
endif

