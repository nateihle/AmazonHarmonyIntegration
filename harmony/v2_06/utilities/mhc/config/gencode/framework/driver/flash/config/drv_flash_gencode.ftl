menu "Flash"
    depends on HAVE_NVM
    
config GENERATE_CODE_DRV_FLASH${INSTANCE}
    bool "Flash?"
    default n
    set DRV_FLASH_NEEDED to y if GENERATE_CODE_DRV_FLASH${INSTANCE}
    ---help---
    IDH_HTML_DRV_FLASH_Flash_Driver_Library
    ---endhelp---

ifblock GENERATE_CODE_DRV_FLASH${INSTANCE}

# comment "**** Note: Please configure Flash Driver settings under Framework Configuration menu ****"

config DRV_FLASH_APP_WRITE_STRING${INSTANCE}
    bool "Write String to Flash"
    default n
    ---help---
        <!DOCTYPE HTML>
        <html>
        <h2>MPLAB Harmony Write String to Flash Application Template</h2>
        <p>	This template generates a simple code example which writes
        a string of bytes to the flash.</p>
        </html>
    ---endhelp---
    

config DRV_FLASH_APP_WRITE_STRING_VERIFY${INSTANCE}
    bool "Write String to Flash then Verify"
    default n
    ---help---
        <!DOCTYPE HTML>
        <html>
        <h2>MPLAB Harmony Write String to Flash then Verify Application Template</h2>
        <p>	This template generates a simple code example which writes
        a string of bytes to the flash and then verifies the flash contents.</p>
        </html>
    ---endhelp---
    
menu "Options"

config APP_FLASH_DRV_START_ADDRESS${INSTANCE}
    hex "Start Address of the Flash storage?"
    default 0x9D008000

config APP_DRV_FLASH_WRITE_STRING${INSTANCE}
	string "Transmit String"
	default "Hello World"
	depends on DRV_FLASH_APP_WRITE_STRING${INSTANCE} || DRV_FLASH_APP_WRITE_STRING_VERIFY${INSTANCE}

config APP_FLASH_DRV_HANDLE${INSTANCE}
    string "Name of the flash driver handle?"
    default "handleFlashDrv"

    
#config APP_FLASH_DRV_PAGE_COUNT${INSTANCE}
#    int "Number of flash pages to use for storage?"
#    default 4
#comment "**** Please note the PAGE and ROW sizes of the configured device ****"
#config APP_FLASH_DRV_BYTE_COUNT${INSTANCE}
#    int "Number of bytes to use for storage?"
#    default APP_FLASH_DRV_PAGE_SIZE${INSTANCE}
#
#    # These two parameters are something of a hack.  They are meant to remind the user
#    # how large the PAGE and ROW are for his configured device.  They are meant to make
#    # assiging a size to the flash storage a little easier.
#    # Ideally, the ROW and PAGE size values should be defined elsewhere.  Perhaps even in the 
#    # <HARMONY>\framework\peripheral\nvm\processor\nvm_<processor>.hconfig file.
#    # That file, however, is generated, so the change would have to be made in the generator.
#    # This limitation is also faced by the NVM driver.
#config APP_FLASH_DRV_ROW_SIZE${INSTANCE}
#    int "ROW - Minimum number of bytes to program per operation:"
#    default 2048 if PIC32MZ = y
#    default 512 if PIC32MX = y
#    persistent    
#
#config APP_FLASH_DRV_PAGE_SIZE${INSTANCE}
#    int "PAGE - Minimum number of bytes to erase per operation:"
#    default 16384  if PIC32MZ = y
#    default 4096  if PIC32MX = y    
#    persistent    
#
#config APP_FLASH_DRV_BYTE_COUNT${INSTANCE}
#    int "Number of bytes to use for storage?"
#    default APP_FLASH_DRV_PAGE_COUNT${INSTANCE} * APP_FLASH_DRV_PAGE_SIZE${INSTANCE}
#    persistent    

endmenu




add "^#include \"/utilities/mhc/config/gencode/framework/driver/flash/config/drv_flash_gencode_macros_app.h.ftl\">" to list APP_FREEMARKER_MACROS
add "^@macro_drv_flash_app_h_includes/>" to list APP${INSTANCE}_H_INCLUDES
add "^@macro_drv_flash_app_h_constants/>" to list APP${INSTANCE}_H_CONSTANTS
add "^@macro_drv_flash_app_h_type_definitions/>" to list APP${INSTANCE}_H_TYPE_DEFINITIONS
add "^@macro_drv_flash_app_h_data/>" to list APP${INSTANCE}_H_APP_DATA
add "^@macro_drv_flash_app_h_callback_function_declarations/>" to list APP${INSTANCE}_H_APP_CALLBACK_FUNCTION_DECLARATIONS
add "^@macro_drv_flash_app_h_function_declarations/>" to list APP${INSTANCE}_H_APP_FUNCTION_DECLARATIONS

add "^#include \"/utilities/mhc/config/gencode/framework/driver/flash/config/drv_flash_gencode_macros_app.c.ftl\">" to list APP_FREEMARKER_MACROS
add "^@macro_drv_flash_app_c_includes/>" to list APP${INSTANCE}_C_INCLUDES
add "^@macro_drv_flash_app_c_global_data/>" to list APP${INSTANCE}_C_GLOBAL_DATA
add "^@macro_drv_flash_app_c_callback_functions/>" to list APP${INSTANCE}_C_CALLBACK_FUNCTIONS
add "^@macro_drv_flash_app_c_local_functions/>" to list APP${INSTANCE}_C_LOCAL_FUNCTIONS
add "^@macro_drv_flash_app_c_initialize/>" to list APP${INSTANCE}_C_INITIALIZE
add "^@macro_drv_flash_app_c_tasks_data/>" to list APP${INSTANCE}_C_TASKS_DATA
add "^@macro_drv_flash_app_c_tasks_state_init/>" to list APP${INSTANCE}_C_TASKS_STATE_INIT
add "^@macro_drv_flash_app_c_tasks_calls_after_init/>" to list APP${INSTANCE}_C_TASKS_CALLS_AFTER_INIT
add "^@macro_drv_flash_app_c_tasks_state_service_tasks/>" to list APP${INSTANCE}_C_TASKS_STATE_SERVICE_TASKS
 
#add "^#include \"/utilities/mhc/config/gencode/framework/driver/flash/config/drv_flash_gencode_macros_system_config.h.ftl\">" to list APP_FREEMARKER_MACROS 
#add "^@macro_drv_flash_system_config_h_app_constants/>" to list APP${INSTANCE}_H_CONSTANTS


endif

endmenu
