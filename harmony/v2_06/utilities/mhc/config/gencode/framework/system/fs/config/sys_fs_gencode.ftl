menu "File System"

config GENERATE_CODE_SYS_FS${INSTANCE}
    bool "Use File System?"
    default n
    select USE_SYS_FS_NEEDED if GENERATE_CODE_SYS_FS${INSTANCE}
    set SYS_FS_AUTO_MOUNT to y if GENERATE_CODE_SYS_FS${INSTANCE}
    set SYS_FS_MAX_FILES to 4 if GENERATE_CODE_SYS_FS${INSTANCE}
    ---help---
    IDH_HTML_SYS_FS_Library
    ---endhelp---


ifblock GENERATE_CODE_SYS_FS${INSTANCE}


config APP_SYS_FS_WRITE${INSTANCE}
    bool "SDCARD: Write Data to File"
    default n
    ---help---
    <!DOCTYPE HTML>
    <html>
    <h2>MPLAB Harmony Filesystem Write Template</h2>
    <p>	This template generates a simple code example which writes
    data to the filesystem. It will automatically configure
    the filesystem driver with the following settings: SDCARD</p>
    <p>All other filesystem configuration options are set to their
    default values. The driver configuration must be modified by
    the user using MHC, under Harmony Framework Configuration ->
    System Services-> File System and setup the Media to use.</p></html>
    ---endhelp---

config APP_SYS_FS_WRITE_FILENAME${INSTANCE}
    string "Filename"
    depends on APP_SYS_FS_WRITE${INSTANCE}
    default "filename.txt"

config APP_SYS_FS_DATA${INSTANCE}
    string "Data to Write"
    depends on APP_SYS_FS_WRITE${INSTANCE}
    default "Hello World"

config APP_SYS_FS_WRITE_MEDIA${INSTANCE}
    int "Media #"
    depends on APP_SYS_FS_WRITE${INSTANCE}
    range 0 0 if SYS_FS_INSTANCES_NUMBER = 1
    range 0 1 if SYS_FS_INSTANCES_NUMBER = 2
    range 0 2 if SYS_FS_INSTANCES_NUMBER = 3
    range 0 3 if SYS_FS_INSTANCES_NUMBER = 4
    set SYS_FS_MEDIA_TYPE_DEFINE_IDX0 to "SYS_FS_MEDIA_TYPE_SD_CARD" if APP_SYS_FS_WRITE_MEDIA${INSTANCE} = 0
    set SYS_FS_MEDIA_TYPE_DEFINE_IDX1 to "SYS_FS_MEDIA_TYPE_SD_CARD" if APP_SYS_FS_WRITE_MEDIA${INSTANCE} = 1
    set SYS_FS_MEDIA_TYPE_DEFINE_IDX2 to "SYS_FS_MEDIA_TYPE_SD_CARD" if APP_SYS_FS_WRITE_MEDIA${INSTANCE} = 2
    set SYS_FS_MEDIA_TYPE_DEFINE_IDX3 to "SYS_FS_MEDIA_TYPE_SD_CARD" if APP_SYS_FS_WRITE_MEDIA${INSTANCE} = 3
    default 0
    ---help---
    <!DOCTYPE HTML>
    <html>
    <h2>Enter Media #</h2>
    <p>	Use the Media number from the File System configuration
    that corrosponds to the selected media (NVM or SDCARD)</p>
    </html>
    ---endhelp---


config APP_SYS_FS_READ${INSTANCE}
    bool "SDCARD: Read Data from File"
    default n
    ---help---
    <!DOCTYPE HTML>
    <html>
    <h2>MPLAB Harmony Filesystem Write Template</h2>
    <p>	This template generates a simple code example which reads
    data from the filesystem. It will automatically configure
    the filesystem driver with the following settings: SDCARD</p>
    <p>All other filesystem configuration options are set to their
    default values. The driver configuration must be modified by
    the user using MHC, under Harmony Framework Configuration ->
    System Services-> File System and setup the Media to use.</p></html>
    ---endhelp---

config APP_SYS_FS_READ_FILENAME${INSTANCE}
    string "Filename"
    depends on APP_SYS_FS_READ${INSTANCE}
    default "filename.txt"

config APP_SYS_FS_READ_SIZE${INSTANCE}
    int "Number of Characters to Read"
    depends on APP_SYS_FS_READ${INSTANCE}
    default 20

config APP_SYS_FS_READ_MEDIA${INSTANCE}
    int "Media #"
    depends on APP_SYS_FS_READ${INSTANCE}
    range 0 0 if SYS_FS_INSTANCES_NUMBER = 1
    range 0 1 if SYS_FS_INSTANCES_NUMBER = 2
    range 0 2 if SYS_FS_INSTANCES_NUMBER = 3
    range 0 3 if SYS_FS_INSTANCES_NUMBER = 4
    default 0
    ---help---
    <!DOCTYPE HTML>
    <html>
    <h2>Enter Media #</h2>
    <p>	Use the Media number from the File System configuration
    that corrosponds to the selected media (NVM, SDCARD or SRAM)</p>
    </html>
    ---endhelp---


config APP_SYS_SRAM_WRITE${INSTANCE}
    bool "SRAM: Write Data to File in SRAM"
    select DRV_SRAM_NEEDED if APP_SYS_SRAM_WRITE${INSTANCE}
    select USE_DRV_SRAM_SYS_FS_REGISTER_IDX0 if APP_SYS_SRAM_WRITE${INSTANCE}
    set DRV_SRAM_MEDIA_START_ADDRESS_IDX0 to "sramDiskImage" if APP_SYS_SRAM_WRITE${INSTANCE}
    default n
    ---help---
    <!DOCTYPE HTML>
    <html>
    <h2>MPLAB Harmony Filesystem Write Template</h2>
    <p>	This template generates a simple code example which writes
    data to the filesystem. It will automatically configure
    the filesystem driver with the following settings: SRAM</p>
    <p>All other filesystem configuration options are set to their
    default values. The driver configuration must be modified by
    the user using MHC, under Harmony Framework Configuration ->
    System Services-> File System and setup the Media to use.</p></html>
    ---endhelp---

config APP_SYS_SRAM_WRITE_FILENAME${INSTANCE}
    string "Filename"
    depends on APP_SYS_SRAM_WRITE${INSTANCE}
    default "file.txt"

config APP_SYS_SRAM_DATA${INSTANCE}
    string "Data to Write"
    depends on APP_SYS_SRAM_WRITE${INSTANCE}
    default "Hello World"

config APP_SYS_SRAM_WRITE_MEDIA${INSTANCE}
    int "Media #"
    depends on APP_SYS_SRAM_WRITE${INSTANCE}
    range 0 0 if SYS_FS_INSTANCES_NUMBER = 1
    range 0 1 if SYS_FS_INSTANCES_NUMBER = 2
    range 0 2 if SYS_FS_INSTANCES_NUMBER = 3
    range 0 3 if SYS_FS_INSTANCES_NUMBER = 4

    set SYS_FS_TYPE_DEFINE_IDX0 to "FAT" if APP_SYS_SRAM_WRITE_MEDIA${INSTANCE} = 0
    set SYS_FS_TYPE_DEFINE_IDX1 to "FAT" if APP_SYS_SRAM_WRITE_MEDIA${INSTANCE} = 1
    set SYS_FS_TYPE_DEFINE_IDX2 to "FAT" if APP_SYS_SRAM_WRITE_MEDIA${INSTANCE} = 2
    set SYS_FS_TYPE_DEFINE_IDX3 to "FAT" if APP_SYS_SRAM_WRITE_MEDIA${INSTANCE} = 3

    set SYS_FS_MEDIA_TYPE_DEFINE_IDX0 to "SYS_FS_MEDIA_TYPE_RAM" if APP_SYS_SRAM_WRITE_MEDIA${INSTANCE} = 0
    set SYS_FS_MEDIA_TYPE_DEFINE_IDX1 to "SYS_FS_MEDIA_TYPE_RAM" if APP_SYS_SRAM_WRITE_MEDIA${INSTANCE} = 1
    set SYS_FS_MEDIA_TYPE_DEFINE_IDX2 to "SYS_FS_MEDIA_TYPE_RAM" if APP_SYS_SRAM_WRITE_MEDIA${INSTANCE} = 2
    set SYS_FS_MEDIA_TYPE_DEFINE_IDX3 to "SYS_FS_MEDIA_TYPE_RAM" if APP_SYS_SRAM_WRITE_MEDIA${INSTANCE} = 3

    set SYS_FS_USE_SRAM_MBR0 to y if APP_SYS_SRAM_WRITE_MEDIA${INSTANCE} = 0
    set SYS_FS_USE_SRAM_MBR1 to y if APP_SYS_SRAM_WRITE_MEDIA${INSTANCE} = 1
    set SYS_FS_USE_SRAM_MBR2 to y if APP_SYS_SRAM_WRITE_MEDIA${INSTANCE} = 2
    set SYS_FS_USE_SRAM_MBR3 to y if APP_SYS_SRAM_WRITE_MEDIA${INSTANCE} = 3

    default 0
    ---help---
    <!DOCTYPE HTML>
    <html>
    <h2>Enter Media #</h2>
    <p>	Use the Media number from the File System configuration
    that corrosponds to the selected media (NVM, SDCARD or SRAM)</p>
    </html>
    ---endhelp---


config APP_SYS_SRAM_READ${INSTANCE}
    bool "SRAM: Read Data from File"
    select DRV_SRAM_NEEDED if APP_SYS_SRAM_READ${INSTANCE}
    select USE_DRV_SRAM_SYS_FS_REGISTER_IDX0 if APP_SYS_SRAM_READ${INSTANCE}
    set DRV_SRAM_MEDIA_START_ADDRESS_IDX0 to "sramDiskImage" if APP_SYS_SRAM_READ${INSTANCE}
    default n
    ---help---
    <!DOCTYPE HTML>
    <html>
    <h2>MPLAB Harmony Filesystem Write Template</h2>
    <p>	This template generates a simple code example which reads
    data from the filesystem. It will automatically configure
    the filesystem driver with the following settings: SRAM</p>
    <p>All other filesystem configuration options are set to their
    default values. The driver configuration must be modified by
    the user using MHC, under Harmony Framework Configuration ->
    System Services-> File System and setup the Media to use.</p></html>
    ---endhelp---

config APP_SYS_SRAM_READ_FILENAME${INSTANCE}
    string "Filename"
    depends on APP_SYS_SRAM_READ${INSTANCE}
    default "file.txt"

config APP_SYS_SRAM_READ_SIZE${INSTANCE}
    int "Number of Characters to Read"
    depends on APP_SYS_SRAM_READ${INSTANCE}
    default 20

config APP_SYS_SRAM_READ_MEDIA${INSTANCE}
    int "Media #"
    depends on APP_SYS_SRAM_READ${INSTANCE}
    range 0 0 if SYS_FS_INSTANCES_NUMBER = 1
    range 0 1 if SYS_FS_INSTANCES_NUMBER = 2
    range 0 2 if SYS_FS_INSTANCES_NUMBER = 3
    range 0 3 if SYS_FS_INSTANCES_NUMBER = 4

    set SYS_FS_TYPE_DEFINE_IDX0 to "FAT" if APP_SYS_SRAM_READ_MEDIA${INSTANCE} = 0
    set SYS_FS_TYPE_DEFINE_IDX1 to "FAT" if APP_SYS_SRAM_READ_MEDIA${INSTANCE} = 1
    set SYS_FS_TYPE_DEFINE_IDX2 to "FAT" if APP_SYS_SRAM_READ_MEDIA${INSTANCE} = 2
    set SYS_FS_TYPE_DEFINE_IDX3 to "FAT" if APP_SYS_SRAM_READ_MEDIA${INSTANCE} = 3

    set SYS_FS_MEDIA_TYPE_DEFINE_IDX0 to "SYS_FS_MEDIA_TYPE_RAM" if APP_SYS_SRAM_READ_MEDIA${INSTANCE} = 0
    set SYS_FS_MEDIA_TYPE_DEFINE_IDX1 to "SYS_FS_MEDIA_TYPE_RAM" if APP_SYS_SRAM_READ_MEDIA${INSTANCE} = 1
    set SYS_FS_MEDIA_TYPE_DEFINE_IDX2 to "SYS_FS_MEDIA_TYPE_RAM" if APP_SYS_SRAM_READ_MEDIA${INSTANCE} = 2
    set SYS_FS_MEDIA_TYPE_DEFINE_IDX3 to "SYS_FS_MEDIA_TYPE_RAM" if APP_SYS_SRAM_READ_MEDIA${INSTANCE} = 3

    set SYS_FS_USE_SRAM_MBR0 to y if APP_SYS_SRAM_READ_MEDIA${INSTANCE} = 0
    set SYS_FS_USE_SRAM_MBR1 to y if APP_SYS_SRAM_READ_MEDIA${INSTANCE} = 1
    set SYS_FS_USE_SRAM_MBR2 to y if APP_SYS_SRAM_READ_MEDIA${INSTANCE} = 2
    set SYS_FS_USE_SRAM_MBR3 to y if APP_SYS_SRAM_READ_MEDIA${INSTANCE} = 3

    default 0
    ---help---
    <!DOCTYPE HTML>
    <html>
    <h2>Enter Media #</h2>
    <p>	Use the Media number from the File System configuration
    that corrosponds to the selected media (NVM, SDCARD or SRAM)</p>
    </html>
    ---endhelp---




config APP_SYS_NVM_WRITE${INSTANCE}
    bool "NVM: Write Data to File"
    default n
    select DRV_NVM_NEEDED if APP_SYS_NVM_WRITE${INSTANCE}
    set USE_DRV_NVM_SYS_FS_REGISTER to y if APP_SYS_NVM_WRITE${INSTANCE}
    ---help---
    <!DOCTYPE HTML>
    <html>
    <h2>MPLAB Harmony Filesystem Write Template</h2>
    <p>	This template generates a simple code example which writes
    data to the filesystem on NVM. It will automatically configure
    the filesystem driver with the following settings:</p>
    <p>All other filesystem configuration options are set to their
    default values. The driver configuration may be modified by
    the user using MHC, under Harmony Framework Configuration ->
    System Services-> File System and setup the Media to use.</p></html>
    ---endhelp---

config APP_SYS_NVM_WRITE_FILENAME${INSTANCE}
    string "Filename"
    depends on APP_SYS_NVM_WRITE${INSTANCE}
    default "filename.txt"

config APP_SYS_NVM_DATA${INSTANCE}
    string "Data to Write"
    depends on APP_SYS_NVM_WRITE${INSTANCE}
    default "Hello World"

config APP_SYS_NVM_WRITE_MEDIA${INSTANCE}
    int "Media #"
    depends on APP_SYS_NVM_WRITE${INSTANCE}
    range 0 0 if SYS_FS_INSTANCES_NUMBER = 1
    range 0 1 if SYS_FS_INSTANCES_NUMBER = 2
    range 0 2 if SYS_FS_INSTANCES_NUMBER = 3
    range 0 3 if SYS_FS_INSTANCES_NUMBER = 4
    set SYS_FS_TYPE_DEFINE_IDX0 to "FAT" if APP_SYS_NVM_WRITE_MEDIA${INSTANCE} = 0
    set SYS_FS_TYPE_DEFINE_IDX1 to "FAT" if APP_SYS_NVM_WRITE_MEDIA${INSTANCE} = 1
    set SYS_FS_TYPE_DEFINE_IDX2 to "FAT" if APP_SYS_NVM_WRITE_MEDIA${INSTANCE} = 2
    set SYS_FS_TYPE_DEFINE_IDX3 to "FAT" if APP_SYS_NVM_WRITE_MEDIA${INSTANCE} = 3
    set SYS_FS_MEDIA_TYPE_DEFINE_IDX0 to "SYS_FS_MEDIA_TYPE_NVM" if APP_SYS_NVM_WRITE_MEDIA${INSTANCE} = 0
    set SYS_FS_MEDIA_TYPE_DEFINE_IDX1 to "SYS_FS_MEDIA_TYPE_NVM" if APP_SYS_NVM_WRITE_MEDIA${INSTANCE} = 1
    set SYS_FS_MEDIA_TYPE_DEFINE_IDX2 to "SYS_FS_MEDIA_TYPE_NVM" if APP_SYS_NVM_WRITE_MEDIA${INSTANCE} = 2
    set SYS_FS_MEDIA_TYPE_DEFINE_IDX3 to "SYS_FS_MEDIA_TYPE_NVM" if APP_SYS_NVM_WRITE_MEDIA${INSTANCE} = 3
    set SYS_FS_USE_NVM_MBR0 to y if APP_SYS_NVM_WRITE_MEDIA${INSTANCE} = 0
    set SYS_FS_USE_NVM_MBR1 to y if APP_SYS_NVM_WRITE_MEDIA${INSTANCE} = 1
    set SYS_FS_USE_NVM_MBR2 to y if APP_SYS_NVM_WRITE_MEDIA${INSTANCE} = 2
    set SYS_FS_USE_NVM_MBR3 to y if APP_SYS_NVM_WRITE_MEDIA${INSTANCE} = 3
    default 0
    ---help---
    <!DOCTYPE HTML>
    <html>
    <h2>Enter Media #</h2>
    <p>	Use the Media number from the File System configuration
    that corrosponds to the selected media (NVM or SDCARD)</p>
    </html>
    ---endhelp---


config APP_SYS_NVM_READ${INSTANCE}
    bool "NVM: Read Data from File"
    default n
    select DRV_NVM_NEEDED if APP_SYS_NVM_READ${INSTANCE}
    ---help---
    <!DOCTYPE HTML>
    <html>
    <h2>MPLAB Harmony Filesystem Write Template</h2>
    <p>	This template generates a simple code example which reads
    data from the filesystem on NVM. The driver configuration must be modified by
    the user using MHC, under Harmony Framework Configuration ->
    System Services-> File System and setup the Media to use.</p></html>
    ---endhelp---

config APP_SYS_NVM_READ_FILENAME${INSTANCE}
    string "Filename"
    depends on APP_SYS_NVM_READ${INSTANCE}
    default "filename.txt"

config APP_SYS_NVM_READ_SIZE${INSTANCE}
    int "Number of Characters to Read"
    depends on APP_SYS_NVM_READ${INSTANCE}
    default 20

config APP_SYS_NVM_READ_MEDIA${INSTANCE}
    int "Media #"
    depends on APP_SYS_NVM_READ${INSTANCE}
    range 0 0 if SYS_FS_INSTANCES_NUMBER = 1
    range 0 1 if SYS_FS_INSTANCES_NUMBER = 2
    range 0 2 if SYS_FS_INSTANCES_NUMBER = 3
    range 0 3 if SYS_FS_INSTANCES_NUMBER = 4
    set SYS_FS_TYPE_DEFINE_IDX0 to "FAT" if APP_SYS_NVM_READ_MEDIA${INSTANCE} = 0
    set SYS_FS_TYPE_DEFINE_IDX1 to "FAT" if APP_SYS_NVM_READ_MEDIA${INSTANCE} = 1
    set SYS_FS_TYPE_DEFINE_IDX2 to "FAT" if APP_SYS_NVM_READ_MEDIA${INSTANCE} = 2
    set SYS_FS_TYPE_DEFINE_IDX3 to "FAT" if APP_SYS_NVM_READ_MEDIA${INSTANCE} = 3
    set SYS_FS_MEDIA_TYPE_DEFINE_IDX0 to "SYS_FS_MEDIA_TYPE_NVM" if APP_SYS_NVM_READ_MEDIA${INSTANCE} = 0
    set SYS_FS_MEDIA_TYPE_DEFINE_IDX1 to "SYS_FS_MEDIA_TYPE_NVM" if APP_SYS_NVM_READ_MEDIA${INSTANCE} = 1
    set SYS_FS_MEDIA_TYPE_DEFINE_IDX2 to "SYS_FS_MEDIA_TYPE_NVM" if APP_SYS_NVM_READ_MEDIA${INSTANCE} = 2
    set SYS_FS_MEDIA_TYPE_DEFINE_IDX3 to "SYS_FS_MEDIA_TYPE_NVM" if APP_SYS_NVM_READ_MEDIA${INSTANCE} = 3
    set SYS_FS_USE_NVM_MBR0 to y if APP_SYS_NVM_READ_MEDIA${INSTANCE} = 0
    set SYS_FS_USE_NVM_MBR1 to y if APP_SYS_NVM_READ_MEDIA${INSTANCE} = 1
    set SYS_FS_USE_NVM_MBR2 to y if APP_SYS_NVM_READ_MEDIA${INSTANCE} = 2
    set SYS_FS_USE_NVM_MBR3 to y if APP_SYS_NVM_READ_MEDIA${INSTANCE} = 3
    default 0
    ---help---
    <!DOCTYPE HTML>
    <html>
    <h2>Enter Media #</h2>
    <p>	Use the Media number from the File System configuration
    that corrosponds to the selected media (NVM or SDCARD)</p>
    </html>
    ---endhelp---



ifblock APP_SYS_FS_WRITE${INSTANCE}
add "^#include \"/utilities/mhc/config/gencode/framework/system/fs/templates/sys_fs_gencode_macros_write_app.h.ftl\">" to list APP_FREEMARKER_MACROS
add "^@macro_sys_fs_write_app_h_includes/>" to list APP${INSTANCE}_H_INCLUDES
add "^@macro_sys_fs_write_app_h_states/>" to list APP${INSTANCE}_H_APP_STATES
add "^@macro_sys_fs_write_app_h_data/>" to list APP${INSTANCE}_H_APP_DATA
add "^@macro_sys_fs_write_app_h_callback_function_declarations/>" to list APP${INSTANCE}_H_APP_CALLBACK_FUNCTION_DECLARATIONS
add "^@macro_sys_fs_write_app_h_function_declarations/>" to list APP${INSTANCE}_H_APP_FUNCTION_DECLARATIONS

add "^#include \"/utilities/mhc/config/gencode/framework/system/fs/templates/sys_fs_gencode_macros_write_app.c.ftl\">" to list APP_FREEMARKER_MACROS
add "^@macro_sys_fs_write_app_c_includes/>" to list APP${INSTANCE}_C_INCLUDES
add "^@macro_sys_fs_write_app_c_global_data/>" to list APP${INSTANCE}_C_GLOBAL_DATA
add "^@macro_sys_fs_write_app_c_callback_functions/>" to list APP${INSTANCE}_C_CALLBACK_FUNCTIONS
add "^@macro_sys_fs_write_app_c_local_functions/>" to list APP${INSTANCE}_C_LOCAL_FUNCTIONS
add "^@macro_sys_fs_write_app_c_initialize/>" to list APP${INSTANCE}_C_INITIALIZE
add "^@macro_sys_fs_write_app_c_tasks_data/>" to list APP${INSTANCE}_C_TASKS_DATA
add "^@macro_sys_fs_write_app_c_tasks_state_init/>" to list APP${INSTANCE}_C_TASKS_STATE_INIT
add "^@macro_sys_fs_write_app_c_tasks_calls_after_init/>" to list APP${INSTANCE}_C_TASKS_CALLS_AFTER_INIT
add "^@macro_sys_fs_write_app_c_tasks_state_service_tasks/>" to list APP${INSTANCE}_C_TASKS_STATE_SERVICE_TASKS
add "^@macro_sys_fs_write_app_c_tasks_states/>" to list APP${INSTANCE}_C_TASKS_STATES
endif

ifblock APP_SYS_FS_READ${INSTANCE}
add "^#include \"/utilities/mhc/config/gencode/framework/system/fs/templates/sys_fs_gencode_macros_read_app.h.ftl\">" to list APP_FREEMARKER_MACROS
add "^@macro_sys_fs_read_app_h_includes/>" to list APP${INSTANCE}_H_INCLUDES
add "^@macro_sys_fs_read_app_h_states/>" to list APP${INSTANCE}_H_APP_STATES
add "^@macro_sys_fs_read_app_h_data/>" to list APP${INSTANCE}_H_APP_DATA
add "^@macro_sys_fs_read_app_h_callback_function_declarations/>" to list APP${INSTANCE}_H_APP_CALLBACK_FUNCTION_DECLARATIONS
add "^@macro_sys_fs_read_app_h_function_declarations/>" to list APP${INSTANCE}_H_APP_FUNCTION_DECLARATIONS

add "^#include \"/utilities/mhc/config/gencode/framework/system/fs/templates/sys_fs_gencode_macros_read_app.c.ftl\">" to list APP_FREEMARKER_MACROS
add "^@macro_sys_fs_read_app_c_includes/>" to list APP${INSTANCE}_C_INCLUDES
add "^@macro_sys_fs_read_app_c_global_data/>" to list APP${INSTANCE}_C_GLOBAL_DATA
add "^@macro_sys_fs_read_app_c_callback_functions/>" to list APP${INSTANCE}_C_CALLBACK_FUNCTIONS
add "^@macro_sys_fs_read_app_c_local_functions/>" to list APP${INSTANCE}_C_LOCAL_FUNCTIONS
add "^@macro_sys_fs_read_app_c_initialize/>" to list APP${INSTANCE}_C_INITIALIZE
add "^@macro_sys_fs_read_app_c_tasks_data/>" to list APP${INSTANCE}_C_TASKS_DATA
add "^@macro_sys_fs_read_app_c_tasks_state_init/>" to list APP${INSTANCE}_C_TASKS_STATE_INIT
add "^@macro_sys_fs_read_app_c_tasks_calls_after_init/>" to list APP${INSTANCE}_C_TASKS_CALLS_AFTER_INIT
add "^@macro_sys_fs_read_app_c_tasks_state_service_tasks/>" to list APP${INSTANCE}_C_TASKS_STATE_SERVICE_TASKS
add "^@macro_sys_fs_read_app_c_tasks_states/>" to list APP${INSTANCE}_C_TASKS_STATES
endif


ifblock APP_SYS_NVM_WRITE${INSTANCE}
add "^#include \"/utilities/mhc/config/gencode/framework/system/fs/templates/sys_nvm_gencode_macros_write_app.h.ftl\">" to list APP_FREEMARKER_MACROS
add "^@macro_sys_nvm_write_app_h_includes/>" to list APP${INSTANCE}_H_INCLUDES
add "^@macro_sys_nvm_write_app_h_states/>" to list APP${INSTANCE}_H_APP_STATES
add "^@macro_sys_nvm_write_app_h_data/>" to list APP${INSTANCE}_H_APP_DATA
add "^@macro_sys_nvm_write_app_h_callback_function_declarations/>" to list APP${INSTANCE}_H_APP_CALLBACK_FUNCTION_DECLARATIONS
add "^@macro_sys_nvm_write_app_h_function_declarations/>" to list APP${INSTANCE}_H_APP_FUNCTION_DECLARATIONS

add "^#include \"/utilities/mhc/config/gencode/framework/system/fs/templates/sys_nvm_gencode_macros_write_app.c.ftl\">" to list APP_FREEMARKER_MACROS
add "^@macro_sys_nvm_write_app_c_includes/>" to list APP${INSTANCE}_C_INCLUDES
add "^@macro_sys_nvm_write_app_c_global_data/>" to list APP${INSTANCE}_C_GLOBAL_DATA
add "^@macro_sys_nvm_write_app_c_callback_functions/>" to list APP${INSTANCE}_C_CALLBACK_FUNCTIONS
add "^@macro_sys_nvm_write_app_c_local_functions/>" to list APP${INSTANCE}_C_LOCAL_FUNCTIONS
add "^@macro_sys_nvm_write_app_c_initialize/>" to list APP${INSTANCE}_C_INITIALIZE
add "^@macro_sys_nvm_write_app_c_tasks_data/>" to list APP${INSTANCE}_C_TASKS_DATA
add "^@macro_sys_nvm_write_app_c_tasks_state_init/>" to list APP${INSTANCE}_C_TASKS_STATE_INIT
add "^@macro_sys_nvm_write_app_c_tasks_calls_after_init/>" to list APP${INSTANCE}_C_TASKS_CALLS_AFTER_INIT
add "^@macro_sys_nvm_write_app_c_tasks_state_service_tasks/>" to list APP${INSTANCE}_C_TASKS_STATE_SERVICE_TASKS
add "^@macro_sys_nvm_write_app_c_tasks_states/>" to list APP${INSTANCE}_C_TASKS_STATES
endif

ifblock APP_SYS_NVM_READ${INSTANCE}
add "^#include \"/utilities/mhc/config/gencode/framework/system/fs/templates/sys_nvm_gencode_macros_read_app.h.ftl\">" to list APP_FREEMARKER_MACROS
add "^@macro_sys_nvm_read_app_h_includes/>" to list APP${INSTANCE}_H_INCLUDES
add "^@macro_sys_nvm_read_app_h_states/>" to list APP${INSTANCE}_H_APP_STATES
add "^@macro_sys_nvm_read_app_h_data/>" to list APP${INSTANCE}_H_APP_DATA
add "^@macro_sys_nvm_read_app_h_callback_function_declarations/>" to list APP${INSTANCE}_H_APP_CALLBACK_FUNCTION_DECLARATIONS
add "^@macro_sys_nvm_read_app_h_function_declarations/>" to list APP${INSTANCE}_H_APP_FUNCTION_DECLARATIONS

add "^#include \"/utilities/mhc/config/gencode/framework/system/fs/templates/sys_nvm_gencode_macros_read_app.c.ftl\">" to list APP_FREEMARKER_MACROS
add "^@macro_sys_nvm_read_app_c_includes/>" to list APP${INSTANCE}_C_INCLUDES
add "^@macro_sys_nvm_read_app_c_global_data/>" to list APP${INSTANCE}_C_GLOBAL_DATA
add "^@macro_sys_nvm_read_app_c_callback_functions/>" to list APP${INSTANCE}_C_CALLBACK_FUNCTIONS
add "^@macro_sys_nvm_read_app_c_local_functions/>" to list APP${INSTANCE}_C_LOCAL_FUNCTIONS
add "^@macro_sys_nvm_read_app_c_initialize/>" to list APP${INSTANCE}_C_INITIALIZE
add "^@macro_sys_nvm_read_app_c_tasks_data/>" to list APP${INSTANCE}_C_TASKS_DATA
add "^@macro_sys_nvm_read_app_c_tasks_state_init/>" to list APP${INSTANCE}_C_TASKS_STATE_INIT
add "^@macro_sys_nvm_read_app_c_tasks_calls_after_init/>" to list APP${INSTANCE}_C_TASKS_CALLS_AFTER_INIT
add "^@macro_sys_nvm_read_app_c_tasks_state_service_tasks/>" to list APP${INSTANCE}_C_TASKS_STATE_SERVICE_TASKS
add "^@macro_sys_nvm_read_app_c_tasks_states/>" to list APP${INSTANCE}_C_TASKS_STATES
endif

ifblock APP_SYS_SRAM_WRITE${INSTANCE}
add "^#include \"/utilities/mhc/config/gencode/framework/system/fs/templates/sys_sram_gencode_macros_write_app.h.ftl\">" to list APP_FREEMARKER_MACROS
add "^@macro_sys_sram_write_app_h_includes/>" to list APP${INSTANCE}_H_INCLUDES
add "^@macro_sys_sram_write_app_h_states/>" to list APP${INSTANCE}_H_APP_STATES
add "^@macro_sys_sram_write_app_h_data/>" to list APP${INSTANCE}_H_APP_DATA
add "^@macro_sys_sram_write_app_h_callback_function_declarations/>" to list APP${INSTANCE}_H_APP_CALLBACK_FUNCTION_DECLARATIONS
add "^@macro_sys_sram_write_app_h_function_declarations/>" to list APP${INSTANCE}_H_APP_FUNCTION_DECLARATIONS

add "^#include \"/utilities/mhc/config/gencode/framework/system/fs/templates/sys_sram_gencode_macros_write_app.c.ftl\">" to list APP_FREEMARKER_MACROS
add "^@macro_sys_sram_write_app_c_includes/>" to list APP${INSTANCE}_C_INCLUDES
add "^@macro_sys_sram_write_app_c_global_data/>" to list APP${INSTANCE}_C_GLOBAL_DATA
add "^@macro_sys_sram_write_app_c_callback_functions/>" to list APP${INSTANCE}_C_CALLBACK_FUNCTIONS
add "^@macro_sys_sram_write_app_c_local_functions/>" to list APP${INSTANCE}_C_LOCAL_FUNCTIONS
add "^@macro_sys_sram_write_app_c_initialize/>" to list APP${INSTANCE}_C_INITIALIZE
add "^@macro_sys_sram_write_app_c_tasks_data/>" to list APP${INSTANCE}_C_TASKS_DATA
add "^@macro_sys_sram_write_app_c_tasks_state_init/>" to list APP${INSTANCE}_C_TASKS_STATE_INIT
add "^@macro_sys_sram_write_app_c_tasks_calls_after_init/>" to list APP${INSTANCE}_C_TASKS_CALLS_AFTER_INIT
add "^@macro_sys_sram_write_app_c_tasks_state_service_tasks/>" to list APP${INSTANCE}_C_TASKS_STATE_SERVICE_TASKS
add "^@macro_sys_sram_write_app_c_tasks_states/>" to list APP${INSTANCE}_C_TASKS_STATES
endif

ifblock APP_SYS_SRAM_READ${INSTANCE}
add "^#include \"/utilities/mhc/config/gencode/framework/system/fs/templates/sys_sram_gencode_macros_read_app.h.ftl\">" to list APP_FREEMARKER_MACROS
add "^@macro_sys_sram_read_app_h_includes/>" to list APP${INSTANCE}_H_INCLUDES
add "^@macro_sys_sram_read_app_h_states/>" to list APP${INSTANCE}_H_APP_STATES
add "^@macro_sys_sram_read_app_h_data/>" to list APP${INSTANCE}_H_APP_DATA
add "^@macro_sys_sram_read_app_h_callback_function_declarations/>" to list APP${INSTANCE}_H_APP_CALLBACK_FUNCTION_DECLARATIONS
add "^@macro_sys_sram_read_app_h_function_declarations/>" to list APP${INSTANCE}_H_APP_FUNCTION_DECLARATIONS

add "^#include \"/utilities/mhc/config/gencode/framework/system/fs/templates/sys_sram_gencode_macros_read_app.c.ftl\">" to list APP_FREEMARKER_MACROS
add "^@macro_sys_sram_read_app_c_includes/>" to list APP${INSTANCE}_C_INCLUDES
add "^@macro_sys_sram_read_app_c_global_data/>" to list APP${INSTANCE}_C_GLOBAL_DATA
add "^@macro_sys_sram_read_app_c_callback_functions/>" to list APP${INSTANCE}_C_CALLBACK_FUNCTIONS
add "^@macro_sys_sram_read_app_c_local_functions/>" to list APP${INSTANCE}_C_LOCAL_FUNCTIONS
add "^@macro_sys_sram_read_app_c_initialize/>" to list APP${INSTANCE}_C_INITIALIZE
add "^@macro_sys_sram_read_app_c_tasks_data/>" to list APP${INSTANCE}_C_TASKS_DATA
add "^@macro_sys_sram_read_app_c_tasks_state_init/>" to list APP${INSTANCE}_C_TASKS_STATE_INIT
add "^@macro_sys_sram_read_app_c_tasks_calls_after_init/>" to list APP${INSTANCE}_C_TASKS_CALLS_AFTER_INIT
add "^@macro_sys_sram_read_app_c_tasks_state_service_tasks/>" to list APP${INSTANCE}_C_TASKS_STATE_SERVICE_TASKS
add "^@macro_sys_sram_read_app_c_tasks_states/>" to list APP${INSTANCE}_C_TASKS_STATES
endif

endif
endmenu
