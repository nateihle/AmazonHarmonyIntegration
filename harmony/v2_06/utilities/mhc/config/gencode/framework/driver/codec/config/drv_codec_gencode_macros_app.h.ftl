<#-- drv_codec_gencode_macros_app.h.ftl -->

<#--
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
-->
<#macro macro_drv_codec_app_h_includes>
</#macro>

<#macro macro_drv_codec_app_h_type_definitions>
typedef struct
{
    DRV_HANDLE handle;
    DRV_I2S_BUFFER_HANDLE writeBufHandle;
    DRV_I2S_BUFFER_EVENT_HANDLER bufferHandler;
    uintptr_t context;
    uint8_t *txbufferObject;
    size_t bufferSize;

} ${APP_NAME?upper_case}_I2S_CLIENT;

typedef enum
{
    CODEC_COMMAND_NONE,
    CODEC_COMMAND_SAMPLING_RATE_SET

} CODEC_COMMAND;

typedef struct
{
    DRV_HANDLE handle;
    DRV_CODEC_BUFFER_HANDLE writeBufHandle;
    DRV_CODEC_BUFFER_EVENT_HANDLER bufferHandler;
    DRV_CODEC_COMMAND_EVENT_HANDLER commandHandler;
    CODEC_COMMAND currentCommand;
    uintptr_t context;
    uint8_t *txbufferObject;
    size_t bufferSize;

} ${APP_NAME?upper_case}_AUDIO_CODEC_CLIENT; 

typedef enum
{
    ${APP_NAME?upper_case}_AUDIO_TONE_TASK_STATE_CODEC_ADD_BUFFER,
    ${APP_NAME?upper_case}_AUDIO_TONE_TASK_STATE_CODEC_WAIT_FOR_BUFFER_COMPLETE,
    ${APP_NAME?upper_case}_AUDIO_TONE_TASK_STATE_CODEC_BUFFER_COMPLETE
    
} ${APP_NAME?upper_case}_AUDIO_TONE_TASK_STATES;
</#macro>

<#--
// *****************************************************************************
/* Application Data

typedef struct
{
    /* The application's current state */
    ${APP_NAME?upper_case}_STATES state;

    /* TODO: Define any additional data used by the application. */
-->
<#macro macro_drv_codec_app_h_data>
    ${APP_NAME?upper_case}_AUDIO_CODEC_CLIENT codecClient;
    
    ${APP_NAME?upper_case}_AUDIO_TONE_TASK_STATES AudioToneTaskState;
</#macro>
<#--
} ${APP_NAME?upper_case}_DATA;





// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/
-->
<#macro macro_drv_codec_app_h_callback_function_declarations>
</#macro>

<#--	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_drv_codec_app_h_function_declarations>
void ${APP_NAME?upper_case}_CodecCommandClear(void);
</#macro>


<#macro macro_drv_codec_app_h_states>
</#macro>




