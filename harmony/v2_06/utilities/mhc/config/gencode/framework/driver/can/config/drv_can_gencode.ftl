menu "CAN"
    depends on HAVE_CAN

config GENERATE_CODE_DRV_CAN${INSTANCE}
    bool "CAN?"
    default n
    select USE_DRV_CAN if GENERATE_CODE_DRV_CAN${INSTANCE}
    ---help---
    IDH_HTML_CAN_Driver_Library
    ---endhelp---

ifblock GENERATE_CODE_DRV_CAN${INSTANCE}

config APP_DRV_CAN_TX${INSTANCE}
    bool "Transmit Buffer"
    default n
    ---help---
    <!DOCTYPE HTML>
    <html>
    <h2>MPLAB Harmony CAN TX Application Template</h2>
    <p>
    This template generates a simple code example which transmits
    given data at the given address on Channel 0. The CAN driver configuration
    must be modified by the user using MHC, under
    Harmony Framework Configuration -> Drivers -> CAN
    </p>
    <p> This template can be combined with the CAN RX and other templates.</p>
    </html>
    ---endhelp---

config APP_DRV_CAN_TX_STRING${INSTANCE}
    string "Transmit Data"
    default "Hello World"
    depends on APP_DRV_CAN_TX${INSTANCE}

config APP_DRV_CAN_TX_ADDRESS${INSTANCE}
    int "Transmit Address"
    default 0
    depends on APP_DRV_CAN_TX${INSTANCE}

config APP_DRV_CAN_RX${INSTANCE}
    bool "Receive Buffer"
    default n
    ---help---
    <!DOCTYPE HTML>
    <html>
    <h2>MPLAB Harmony CAN RX Application Template</h2>
    <p>
    This template generates a simple code example which receives
    data at the given address on Channel 0 into a buffer of the given size.
    The CAN driver configuration  must be modified by the user using MHC, under
    Harmony Framework Configuration -> Drivers -> CAN
    </p>
    <p> This template can be combined with the CAN TX and other templates.</p>
    </html>
    ---endhelp---

config APP_DRV_CAN_RX_BUFFER_SIZE${INSTANCE}
    int "Receive Buffer Size"
    default 10
    depends on APP_DRV_CAN_RX${INSTANCE}

config APP_DRV_CAN_RX_ADDRESS${INSTANCE}
    int "Receive Address"
    default 0
    depends on APP_DRV_CAN_RX${INSTANCE}


menu "Options"
comment "**** Be sure the timing and features for the CAN instance are set in the Framework Configuration "

config APP_DRV_CAN_INSTANCE_INDEX${INSTANCE}
    int "CAN Driver Instance Index"
    range 0 0 if DRV_CAN_INSTANCES_NUMBER = 1
    range 0 1 if DRV_CAN_INSTANCES_NUMBER = 2
    range 0 2 if DRV_CAN_INSTANCES_NUMBER = 3
    range 0 3 if DRV_CAN_INSTANCES_NUMBER = 4
    default 0
    ---help---
    <!DOCTYPE HTML>
    <html>
    <p>
    This should match the Harmony Configuration of the CAN instance to be used.
    </p>
    </html>
    ---endhelp---

config APP_DRV_CAN_HANDLE${INSTANCE}
    string "Name of the CAN driver handle?"
    default "handleCAN0" if APP_DRV_CAN_INSTANCE_INDEX${INSTANCE} = 0
    default "handleCAN1" if APP_DRV_CAN_INSTANCE_INDEX${INSTANCE} = 1
    default "handleCAN2" if APP_DRV_CAN_INSTANCE_INDEX${INSTANCE} = 2
    default "handleCAN3" if APP_DRV_CAN_INSTANCE_INDEX${INSTANCE} = 3
    default "handleCAN"
    ---help---
    <!DOCTYPE HTML>
    <html>
    <p>
    Enter a custom name if desired, otherwise a default will be assigned.
    </p>
    </html>
    ---endhelp---

endmenu




add "^#include \"/utilities/mhc/config/gencode/framework/driver/can/templates/drv_can_gencode_macros_app.h.ftl\">" to list APP_FREEMARKER_MACROS
add "^@macro_drv_can_app_h_includes/>" to list APP${INSTANCE}_H_INCLUDES
add "^@macro_drv_can_app_h_states/>" to list APP${INSTANCE}_H_APP_STATES
add "^@macro_drv_can_app_h_data/>" to list APP${INSTANCE}_H_APP_DATA
add "^@macro_drv_can_app_h_callback_function_declarations/>" to list APP${INSTANCE}_H_APP_CALLBACK_FUNCTION_DECLARATIONS
add "^@macro_drv_can_app_h_function_declarations/>" to list APP${INSTANCE}_H_APP_FUNCTION_DECLARATIONS

add "^#include \"/utilities/mhc/config/gencode/framework/driver/can/templates/drv_can_gencode_macros_app.c.ftl\">" to list APP_FREEMARKER_MACROS
add "^@macro_drv_can_app_c_includes/>" to list APP${INSTANCE}_C_INCLUDES
add "^@macro_drv_can_app_c_global_data/>" to list APP${INSTANCE}_C_GLOBAL_DATA
add "^@macro_drv_can_app_c_callback_functions/>" to list APP${INSTANCE}_C_CALLBACK_FUNCTIONS
add "^@macro_drv_can_app_c_local_functions/>" to list APP${INSTANCE}_C_LOCAL_FUNCTIONS
add "^@macro_drv_can_app_c_initialize/>" to list APP${INSTANCE}_C_INITIALIZE
add "^@macro_drv_can_app_c_tasks_data/>" to list APP${INSTANCE}_C_TASKS_DATA
add "^@macro_drv_can_app_c_tasks_state_init/>" to list APP${INSTANCE}_C_TASKS_STATE_INIT
add "^@macro_drv_can_app_c_tasks_calls_after_init/>" to list APP${INSTANCE}_C_TASKS_CALLS_AFTER_INIT
add "^@macro_drv_can_app_c_tasks_state_service_tasks/>" to list APP${INSTANCE}_C_TASKS_STATE_SERVICE_TASKS
add "^@macro_drv_can_app_c_tasks_states/>" to list APP${INSTANCE}_C_TASKS_STATES

endif

endmenu
