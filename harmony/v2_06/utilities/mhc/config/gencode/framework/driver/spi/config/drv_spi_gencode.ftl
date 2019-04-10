menu "SPI"
    depends on HAVE_SPI

config GENERATE_CODE_DRV_SPI${INSTANCE}
    bool "SPI?"
    default n
    select DRV_SPI_USE_DRIVER_NEEDED if GENERATE_CODE_DRV_SPI${INSTANCE}
    ---help---
    IDH_HTML_SPI_Driver_Library
    ---endhelp---

ifblock GENERATE_CODE_DRV_SPI${INSTANCE}

config APP_DRV_SPI_BLOCKING${INSTANCE}
    bool "Transmit & Receive Buffers (Blocking)"
    default n
    set DRV_SPI_DRIVER_MODE to "STATIC" if APP_DRV_SPI_BLOCKING${INSTANCE}
    set DRV_SPI_USE_BLOCKING to y if APP_DRV_SPI_BLOCKING${INSTANCE}
    persistent if APP_DRV_SPI_NON_BLOCKING_RX${INSTANCE} || APP_DRV_SPI_NON_BLOCKING_TX${INSTANCE}
    ---help---
    <!DOCTYPE HTML>
    <html>
    <h2>MPLAB Harmony SPI TX/RX Blocking Application Template</h2>
    <p>	This template generates a simple code example which transmits
    and receives bytes via SPI. It will automatically configure
    the SPI driver with the following settings:</p>
    <br>- Static Driver Mode
    <br>- Blocking Mode
    <p>All other SPI driver configuration options are set to their
    default values. The driver configuration may be modified by
    the user using MHC, under Harmony Framework Configuration ->
    Drivers -> SPI</p></html>
    ---endhelp---

config APP_DRV_SPI_BLOCKING_TX_STRING${INSTANCE}
    string "Transmit Data"
    default "Hello World"
    depends on APP_DRV_SPI_BLOCKING${INSTANCE}


config APP_DRV_SPI_NON_BLOCKING_TX${INSTANCE}
    bool "Transmit Buffer (Non-Blocking)"
    default n
    set DRV_SPI_DRIVER_MODE to "DYNAMIC" if APP_DRV_SPI_NON_BLOCKING_TX${INSTANCE}
    set DRV_SPI_USE_ISR_MODE optionally to y if APP_DRV_SPI_NON_BLOCKING_TX${INSTANCE}
    persistent if APP_DRV_SPI_BLOCKING${INSTANCE}
    ---help---
    <!DOCTYPE HTML>
    <html>
    <h2>MPLAB Harmony SPI TX Non-Blocking Application Template</h2>
    <p>	This template generates a simple code example which transmits
    bytes via SPI. It will automatically configure
    the SPI driver with the following settings:</p>
    <br>- Dynamic Driver Mode
    <br>- Non-Blocking Mode
    <p>All other SPI driver configuration options are set to their
    default values. The driver configuration may be modified by
    the user using MHC, under Harmony Framework Configuration ->
    Drivers -> SPI</p></html>
    ---endhelp---

config APP_DRV_SPI_NON_BLOCKING_TX_STRING${INSTANCE}
    string "Transmit Data"
    default "Hello World"
    depends on APP_DRV_SPI_NON_BLOCKING_TX${INSTANCE}


config APP_DRV_SPI_NON_BLOCKING_RX${INSTANCE}
    bool "Receive Buffer (Non-Blocking)"
    default n
    set DRV_SPI_DRIVER_MODE to "DYNAMIC" if APP_DRV_SPI_NON_BLOCKING_RX${INSTANCE}
    persistent if APP_DRV_SPI_BLOCKING${INSTANCE}
    ---help---
    <!DOCTYPE HTML>
    <html>
    <h2>MPLAB Harmony SPI RX Non-Blocking Application Template</h2>
    <p>	This template generates a simple code example which receives
    bytes via SPI. It will automatically configure
    the SPI driver with the following settings:</p>
    <br>- Dynamic Driver Mode
    <br>- Non-Blocking Mode
    <p>All other SPI driver configuration options are set to their
    default values. The driver configuration may be modified by
    the user using MHC, under Harmony Framework Configuration ->
    Drivers -> SPI</p></html>
    ---endhelp---

config APP_DRV_SPI_RX_BUFFER_SIZE${INSTANCE}
    int "Receive Buffer Size"
    default 10
    depends on APP_DRV_SPI_NON_BLOCKING_RX${INSTANCE}



menu "Options"
config APP_DRV_SPI_INSTANCE_INDEX${INSTANCE}
    int "SPI Driver Instance Index"
    range 0 0 if DRV_SPI_INSTANCES_NUMBER = 1
    range 0 1 if DRV_SPI_INSTANCES_NUMBER = 2
    range 0 2 if DRV_SPI_INSTANCES_NUMBER = 3
    range 0 3 if DRV_SPI_INSTANCES_NUMBER = 4
    range 0 4 if DRV_SPI_INSTANCES_NUMBER = 5
    range 0 5 if DRV_SPI_INSTANCES_NUMBER = 6
    range 0 6 if DRV_SPI_INSTANCES_NUMBER = 7
    range 0 7 if DRV_SPI_INSTANCES_NUMBER = 8
    range 0 8 if DRV_SPI_INSTANCES_NUMBER = 9
    default 0

config APP_DRV_SPI_HANDLE${INSTANCE}
    string "Name of the SPI driver handle?"
    default "handleSPI0" if APP_DRV_SPI_INSTANCE_INDEX${INSTANCE} = 0
    default "handleSPI1" if APP_DRV_SPI_INSTANCE_INDEX${INSTANCE} = 1
    default "handleSPI2" if APP_DRV_SPI_INSTANCE_INDEX${INSTANCE} = 2
    default "handleSPI3" if APP_DRV_SPI_INSTANCE_INDEX${INSTANCE} = 3
    default "handleSPI4" if APP_DRV_SPI_INSTANCE_INDEX${INSTANCE} = 4
    default "handleSPI5" if APP_DRV_SPI_INSTANCE_INDEX${INSTANCE} = 5
    default "handleSPI6" if APP_DRV_SPI_INSTANCE_INDEX${INSTANCE} = 6
    default "handleSPI7" if APP_DRV_SPI_INSTANCE_INDEX${INSTANCE} = 7
    default "handleSPI8" if APP_DRV_SPI_INSTANCE_INDEX${INSTANCE} = 8
    default "handleSPI9" if APP_DRV_SPI_INSTANCE_INDEX${INSTANCE} = 9
    default "handleSPI"

endmenu



add "^#include \"/utilities/mhc/config/gencode/framework/driver/spi/config/drv_spi_gencode_macros_app.h.ftl\">" to list APP_FREEMARKER_MACROS
add "^@macro_drv_spi_app_h_includes/>" to list APP${INSTANCE}_H_INCLUDES
add "^@macro_drv_spi_app_h_states/>" to list APP${INSTANCE}_H_APP_STATES
add "^@macro_drv_spi_app_h_data/>" to list APP${INSTANCE}_H_APP_DATA
add "^@macro_drv_spi_app_h_callback_function_declarations/>" to list APP${INSTANCE}_H_APP_CALLBACK_FUNCTION_DECLARATIONS
add "^@macro_drv_spi_app_h_function_declarations/>" to list APP${INSTANCE}_H_APP_FUNCTION_DECLARATIONS

add "^#include \"/utilities/mhc/config/gencode/framework/driver/spi/config/drv_spi_gencode_macros_app.c.ftl\">" to list APP_FREEMARKER_MACROS
add "^@macro_drv_spi_app_c_includes/>" to list APP${INSTANCE}_C_INCLUDES
add "^@macro_drv_spi_app_c_global_data/>" to list APP${INSTANCE}_C_GLOBAL_DATA
add "^@macro_drv_spi_app_c_callback_functions/>" to list APP${INSTANCE}_C_CALLBACK_FUNCTIONS
add "^@macro_drv_spi_app_c_local_functions/>" to list APP${INSTANCE}_C_LOCAL_FUNCTIONS
add "^@macro_drv_spi_app_c_initialize/>" to list APP${INSTANCE}_C_INITIALIZE
add "^@macro_drv_spi_app_c_tasks_data/>" to list APP${INSTANCE}_C_TASKS_DATA
add "^@macro_drv_spi_app_c_tasks_state_init/>" to list APP${INSTANCE}_C_TASKS_STATE_INIT
add "^@macro_drv_spi_app_c_tasks_calls_after_init/>" to list APP${INSTANCE}_C_TASKS_CALLS_AFTER_INIT
add "^@macro_drv_spi_app_c_tasks_state_service_tasks/>" to list APP${INSTANCE}_C_TASKS_STATE_SERVICE_TASKS
add "^@macro_drv_spi_app_c_tasks_states/>" to list APP${INSTANCE}_C_TASKS_STATES

endif

endmenu
