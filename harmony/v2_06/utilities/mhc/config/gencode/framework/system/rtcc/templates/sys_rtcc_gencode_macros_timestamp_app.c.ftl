
<#macro macro_sys_rtcc_timestamp_app_c_includes>

#define ${APP_NAME?upper_case}_TIMESTAMP_NUM ${("CONFIG_APP_SYS_RTCC_TIMESTAMP" + "${HCONFIG_APP_INSTANCE}")?eval}
</#macro>

<#macro macro_sys_rtcc_timestamp_app_c_global_data>
typedef struct 
{
    uint32_t    stamp[${APP_NAME?upper_case}_TIMESTAMP_NUM];
    uint8_t     index;
    uint8_t     limit;
} timestampsT;


static SYS_RTCC_ALARM_HANDLE ${APP_NAME?lower_case}Handle;
</#macro>


<#macro macro_sys_rtcc_timestamp_app_c_callback_functions>
/* to be registered with the system RTCC service */
void ${APP_NAME?upper_case}_RTCC_TIMESTAMP_Callback(SYS_RTCC_ALARM_HANDLE handle, uintptr_t context)
{
    /* save typing and help readability */
    timestampsT *times = (timestampsT *)context;
    
    /* the handle will tell us it is our handle */
    if ((handle == ${APP_NAME?lower_case}Handle) && times)
    {
        /* check the limit - room for one more? */
        if (times->index < times->limit)
        {
            SYS_RTCC_BCD_TIME timestamp;
            SYS_RTCC_STATUS status = SYS_RTCC_TimeGet(&timestamp);
            if (SYS_RTCC_STATUS_OK == status)
            {
                times->stamp[times->index++] = timestamp;
            }
        }
    }
}
</#macro>

<#macro macro_sys_rtcc_timestamp_app_c_local_functions>
/* state machine for the Real Time Clock & Calendar */
static void ${APP_NAME?upper_case}_RTCC_TIMESTAMP_Task(void)
{
    static timestampsT timestamps;

    switch (${APP_NAME?lower_case}Data.rtccTimeStateMachine)
    {
        default:
        case ${APP_NAME?upper_case}_RTCC_TIMESTAMP_START:
            /* register the callback with system RTCC */
            timestamps.limit = ${APP_NAME?upper_case}_TIMESTAMP_NUM;
            timestamps.index = 0;
            ${APP_NAME?lower_case}Handle = SYS_RTCC_AlarmRegister(${APP_NAME?upper_case}_RTCC_TIMESTAMP_Callback, 
                (uintptr_t)&timestamps);

            /* continue only if the handle is valid */
            if (${APP_NAME?lower_case}Handle != SYS_RTCC_ALARM_HANDLE_INVALID)
            {
                SYS_RTCC_AlarmEnable();
                ${APP_NAME?lower_case}Data.rtccTimeStateMachine =  ${APP_NAME?upper_case}_RTCC_TIMESTAMP_CHECK;
            }
            break;

        case ${APP_NAME?upper_case}_RTCC_TIMESTAMP_CHECK:
            /* real work is being done in the call back. Are we done? */
            if (timestamps.index == timestamps.limit)
            {
                ${APP_NAME?lower_case}Data.rtccTimeStateMachine =  ${APP_NAME?upper_case}_RTCC_TIMESTAMP_DONE;
            }
            break;

        case ${APP_NAME?upper_case}_RTCC_TIMESTAMP_DONE:
            break;
    }
}

</#macro>

<#macro macro_sys_rtcc_timestamp_app_c_initialize>

    /* Place the state machine in its initial state */
    ${APP_NAME?lower_case}Data.rtccTimeStateMachine = ${APP_NAME?upper_case}_RTCC_TIMESTAMP_START;
</#macro>

<#macro macro_sys_rtcc_timestamp_app_c_tasks_data>
</#macro>

<#macro macro_sys_rtcc_timestamp_app_c_tasks_state_init>
</#macro>

<#macro macro_sys_rtcc_timestamp_app_c_tasks_calls_after_init>
</#macro>

<#macro macro_sys_rtcc_timestamp_app_c_tasks_state_service_tasks>

            /* run the state machine */
            ${APP_NAME?upper_case}_RTCC_TIMESTAMP_Task();
</#macro>

<#macro macro_sys_rtcc_timestamp_app_c_tasks_states>
</#macro>
