menu "Global Event"
    
config GENERATE_CODE_GLOBAL_EVENT_FANOUT${INSTANCE}
    bool "Fanout global event?"
    default n
    select USE_APP_GLOBAL_EVENT

ifblock GENERATE_CODE_GLOBAL_EVENT_FANOUT${INSTANCE}

config GENERATE_CODE_GLOBAL_EVENT_FANOUT_SOURCE${INSTANCE}
    string "From global event name"
    default "global_event_source"

config GENERATE_CODE_GLOBAL_EVENT_FANOUT_NUMBER${INSTANCE}
    int "Number of fanouts"
    default 2
    range 1 10

config GENERATE_CODE_GLOBAL_EVENT_FANOUT_DEST_1${INSTANCE}
    string "To global event name 1"
    default "global_event_dest_1"
    
add "GENERATE_CODE_GLOBAL_EVENT_FANOUT_DEST_1${INSTANCE}" to list GLOBAL_EVENT_NAMES
    
ifblock GENERATE_CODE_GLOBAL_EVENT_FANOUT_NUMBER${INSTANCE} != 1

config GENERATE_CODE_GLOBAL_EVENT_FANOUT_DEST_2${INSTANCE}
    string "To global event name 2"
    default "global_event_dest_2"
    
add "GENERATE_CODE_GLOBAL_EVENT_FANOUT_DEST_2${INSTANCE}" to list GLOBAL_EVENT_NAMES
   
ifblock GENERATE_CODE_GLOBAL_EVENT_FANOUT_NUMBER${INSTANCE} != 2

config GENERATE_CODE_GLOBAL_EVENT_FANOUT_DEST_3${INSTANCE}
    string "To global event name 3"
    default "global_event_dest_3"
    
add "GENERATE_CODE_GLOBAL_EVENT_FANOUT_DEST_3${INSTANCE}" to list GLOBAL_EVENT_NAMES
    
ifblock GENERATE_CODE_GLOBAL_EVENT_FANOUT_NUMBER${INSTANCE} != 3

config GENERATE_CODE_GLOBAL_EVENT_FANOUT_DEST_4${INSTANCE}
    string "To global event name 4"
    default "global_event_dest_4"

add "GENERATE_CODE_GLOBAL_EVENT_FANOUT_DEST_4${INSTANCE}" to list GLOBAL_EVENT_NAMES
    
ifblock GENERATE_CODE_GLOBAL_EVENT_FANOUT_NUMBER${INSTANCE} != 4

config GENERATE_CODE_GLOBAL_EVENT_FANOUT_DEST_5${INSTANCE}
    string "To global event name 5"
    default "global_event_dest_5"
    
add "GENERATE_CODE_GLOBAL_EVENT_FANOUT_DEST_5${INSTANCE}" to list GLOBAL_EVENT_NAMES
    
ifblock GENERATE_CODE_GLOBAL_EVENT_FANOUT_NUMBER${INSTANCE} != 5

config GENERATE_CODE_GLOBAL_EVENT_FANOUT_DEST_6${INSTANCE}
    string "To global event name 6"
    default "global_event_dest_6"
    
add "GENERATE_CODE_GLOBAL_EVENT_FANOUT_DEST_6${INSTANCE}" to list GLOBAL_EVENT_NAMES
    
ifblock GENERATE_CODE_GLOBAL_EVENT_FANOUT_NUMBER${INSTANCE} != 6

config GENERATE_CODE_GLOBAL_EVENT_FANOUT_DEST_7${INSTANCE}
    string "To global event name 7"
    default "global_event_dest_7"

add "GENERATE_CODE_GLOBAL_EVENT_FANOUT_DEST_7${INSTANCE}" to list GLOBAL_EVENT_NAMES
    
ifblock GENERATE_CODE_GLOBAL_EVENT_FANOUT_NUMBER${INSTANCE} != 7

config GENERATE_CODE_GLOBAL_EVENT_FANOUT_DEST_8${INSTANCE}
    string "To global event name 8"
    default "global_event_dest_8"

add "GENERATE_CODE_GLOBAL_EVENT_FANOUT_DEST_8${INSTANCE}" to list GLOBAL_EVENT_NAMES
    
ifblock GENERATE_CODE_GLOBAL_EVENT_FANOUT_NUMBER${INSTANCE} != 8

config GENERATE_CODE_GLOBAL_EVENT_FANOUT_DEST_9${INSTANCE}
    string "To global event name 9"
    default "global_event_dest_9"
    
add "GENERATE_CODE_GLOBAL_EVENT_FANOUT_DEST_9${INSTANCE}" to list GLOBAL_EVENT_NAMES
    
ifblock GENERATE_CODE_GLOBAL_EVENT_FANOUT_NUMBER${INSTANCE} != 9

config GENERATE_CODE_GLOBAL_EVENT_FANOUT_DEST_10${INSTANCE}
    string "To global event name 10"
    default "global_event_dest_10"
    
add "GENERATE_CODE_GLOBAL_EVENT_FANOUT_DEST_10${INSTANCE}" to list GLOBAL_EVENT_NAMES
    
endif
endif
endif
endif
endif
endif
endif
endif
endif

add "^#include \"/utilities/mhc/config/gencode/framework/global_event/fanout/config/global_event_fanout_gencode_macros_app.h.ftl\">" to list APP_FREEMARKER_MACROS
add "^@macro_global_event_fanout_app_h_includes/>" to list APP${INSTANCE}_H_INCLUDES
add "^@macro_global_event_fanout_app_h_type_definitions/>" to list APP${INSTANCE}_H_TYPE_DEFINITIONS
add "^@macro_global_event_fanout_app_h_data/>" to list APP${INSTANCE}_H_APP_DATA
add "^@macro_global_event_fanout_app_h_callback_function_declarations/>" to list APP${INSTANCE}_H_APP_CALLBACK_FUNCTION_DECLARATIONS
add "^@macro_global_event_fanout_app_h_function_declarations/>" to list APP${INSTANCE}_H_APP_FUNCTION_DECLARATIONS

add "^#include \"/utilities/mhc/config/gencode/framework/global_event/fanout/config/global_event_fanout_gencode_macros_app.c.ftl\">" to list APP_FREEMARKER_MACROS
add "^@macro_global_event_fanout_app_c_includes/>" to list APP${INSTANCE}_C_INCLUDES
add "^@macro_global_event_fanout_app_c_callback_functions/>" to list APP${INSTANCE}_C_CALLBACK_FUNCTIONS
add "^@macro_global_event_fanout_app_c_local_functions/>" to list APP${INSTANCE}_C_LOCAL_FUNCTIONS
add "^@macro_global_event_fanout_app_c_initialize/>" to list APP${INSTANCE}_C_INITIALIZE
add "^@macro_global_event_fanout_app_c_tasks_data/>" to list APP${INSTANCE}_C_TASKS_DATA
add "^@macro_global_event_fanout_app_c_tasks_state_init/>" to list APP${INSTANCE}_C_TASKS_STATE_INIT
add "^@macro_global_event_fanout_app_c_tasks_calls_after_init/>" to list APP${INSTANCE}_C_TASKS_CALLS_AFTER_INIT
add "^@macro_global_event_fanout_app_c_tasks_state_service_tasks/>" to list APP${INSTANCE}_C_TASKS_STATE_SERVICE_TASKS
 
add "^#include \"/utilities/mhc/config/gencode/framework/global_event/fanout/config/global_event_fanout_gencode_macros_system_config.h.ftl\">" to list APP_FREEMARKER_MACROS 
add "^@macro_global_event_fanout_system_config_h_app_constants/>" to list APP${INSTANCE}_H_CONSTANTS

endif

endmenu
