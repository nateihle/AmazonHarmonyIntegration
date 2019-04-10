
<#macro macro_sys_nvm_read_app_c_includes>
</#macro>

<#macro macro_sys_nvm_read_app_c_global_data>

static char ${APP_NAME?lower_case}NVMReadPath[] = "${("CONFIG_SYS_FS_MEDIA_MOUNT_1_NAME_IDX" + ("CONFIG_APP_SYS_NVM_READ_MEDIA" + "${HCONFIG_APP_INSTANCE}")?eval)?eval}/${("CONFIG_APP_SYS_NVM_READ_FILENAME" + "${HCONFIG_APP_INSTANCE}")?eval}";
static uint8_t __attribute__ ((aligned (16))) ${APP_NAME?lower_case}NVMReadData[${("CONFIG_APP_SYS_NVM_READ_SIZE" + "${HCONFIG_APP_INSTANCE}")?eval}];
</#macro>


<#macro macro_sys_nvm_read_app_c_callback_functions>
</#macro>

<#macro macro_sys_nvm_read_app_c_local_functions>
/* state machine for the File System */
static void ${APP_NAME?upper_case}_NVM_READ_Task(void)
{
    static SYS_FS_ERROR FsError;
    static uint8_t HandleAttempts;
    static uint32_t TotalBytesRead;
    static SYS_FS_HANDLE FileHandle;

    switch (${APP_NAME?lower_case}Data.nvmReadStateMachine)
    {
        default:
        case ${APP_NAME?upper_case}_NVM_READ_STATE_START:
            HandleAttempts = 0;
            ${APP_NAME?lower_case}Data.nvmReadStateMachine = ${APP_NAME?upper_case}_NVM_READ_STATE_OPEN;
            break;

        case ${APP_NAME?upper_case}_NVM_READ_STATE_OPEN:
            /* open the file to read */
            FileHandle = SYS_FS_FileOpen(${APP_NAME?lower_case}NVMReadPath, SYS_FS_FILE_OPEN_READ);
            if (FileHandle == SYS_FS_HANDLE_INVALID)
            {
                /* try 10 times to get the handle while auto-mount happens */
                if (++HandleAttempts > 10)
                {
                    FsError = SYS_FS_Error();
                    ${APP_NAME?lower_case}Data.nvmReadStateMachine = ${APP_NAME?upper_case}_NVM_READ_STATE_ERROR;
                }
            }
            else
            {
                TotalBytesRead = 0;
                ${APP_NAME?lower_case}Data.nvmReadStateMachine = ${APP_NAME?upper_case}_NVM_READ_STATE_FILEOPS;
            }
            break;

        case ${APP_NAME?upper_case}_NVM_READ_STATE_FILEOPS:
            {
                size_t BytesRead = SYS_FS_FileRead(FileHandle, ${APP_NAME?lower_case}NVMReadData,
                    sizeof(${APP_NAME?lower_case}NVMReadData) - TotalBytesRead);
                if (BytesRead == (size_t)-1)
                {
                    /* error */
                    FsError = SYS_FS_Error();
                    ${APP_NAME?lower_case}Data.nvmReadStateMachine = ${APP_NAME?upper_case}_NVM_READ_STATE_ERROR;
                    return;
                }
                else
                {
                    TotalBytesRead += BytesRead;
                    /* have we gathered all the bytes now? */
                    if (TotalBytesRead >= sizeof(${APP_NAME?lower_case}NVMReadData))
                    {
                        SYS_FS_FileClose(FileHandle);
                        ${APP_NAME?lower_case}Data.nvmReadStateMachine = ${APP_NAME?upper_case}_NVM_READ_STATE_DONE;
                    }
                }
            }
            break;

        case ${APP_NAME?upper_case}_NVM_READ_STATE_DONE:
            break;

        /* if we get to this state - something went wrong - check FsError for cause */
        case ${APP_NAME?upper_case}_NVM_READ_STATE_ERROR:
            (void)FsError;
            break;
    }
}

</#macro>

<#macro macro_sys_nvm_read_app_c_initialize>

    /* Place the File System NVM state machine in its initial state */
    ${APP_NAME?lower_case}Data.nvmReadStateMachine = ${APP_NAME?upper_case}_NVM_READ_STATE_START;
</#macro>

<#macro macro_sys_nvm_read_app_c_tasks_data>
</#macro>

<#macro macro_sys_nvm_read_app_c_tasks_state_init>
</#macro>

<#macro macro_sys_nvm_read_app_c_tasks_calls_after_init>
</#macro>

<#macro macro_sys_nvm_read_app_c_tasks_state_service_tasks>

            /* run the state machine for servicing the NVM file system */
            ${APP_NAME?upper_case}_NVM_READ_Task();
</#macro>

<#macro macro_sys_nvm_read_app_c_tasks_states>
</#macro>
