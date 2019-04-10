
<#macro macro_sys_nvm_app_c_includes>
</#macro>

<#macro macro_sys_nvm_app_c_global_data>
<#if ("CONFIG_APP_SYS_NVM_WRITE" + "${HCONFIG_APP_INSTANCE}")?eval>

static uint32_t nvm_${APP_NAME?lower_case}FsBytesWritten;
static SYS_FS_HANDLE ${APP_NAME?lower_case}NVMWriteHandle = SYS_FS_HANDLE_INVALID;
static char ${APP_NAME?lower_case}NVMWritePath[] = "/mnt/myDrive2/${("CONFIG_APP_SYS_NVM_WRITE_FILENAME" + "${HCONFIG_APP_INSTANCE}")?eval}";
static uint8_t __attribute__ ((aligned (16))) ${APP_NAME?lower_case}_nvm_data[] = "${("CONFIG_APP_SYS_NVM_DATA" + "${HCONFIG_APP_INSTANCE}")?eval}";
</#if>
<#if ("CONFIG_APP_SYS_NVM_READ" + "${HCONFIG_APP_INSTANCE}")?eval>

static uint32_t nvm_${APP_NAME?lower_case}FsBytesRead;
static SYS_FS_HANDLE nvmReadHandle = SYS_FS_HANDLE_INVALID;
static char nvmReadPath[] = "/mnt/myDrive2/${("CONFIG_APP_SYS_NVM_READ_FILENAME" + "${HCONFIG_APP_INSTANCE}")?eval}";
static uint8_t __attribute__ ((aligned (16))) ${APP_NAME?lower_case}_nvm_read[${("CONFIG_APP_SYS_NVM_READ_SIZE" + "${HCONFIG_APP_INSTANCE}")?eval}];
</#if>
</#macro>


<#macro macro_sys_nvm_app_c_callback_functions>
</#macro>

<#macro macro_sys_nvm_app_c_local_functions>
/* state machine for the File System */
static void SYS_NVM_Task(void)
{
    static SYS_FS_ERROR FsError;

    switch (${APP_NAME?lower_case}Data.nvmStateMachine)
    {
        default:
        case ${APP_NAME?upper_case}_NVM_STATE_START:
            /* Automount should have mounted the disk.
                Add code here if you want to do a manual mount */
<#if ("CONFIG_APP_SYS_NVM_WRITE" + "${HCONFIG_APP_INSTANCE}")?eval || ("CONFIG_APP_SYS_NVM_READ" + "${HCONFIG_APP_INSTANCE}")?eval>
                ${APP_NAME?lower_case}Data.nvmStateMachine = ${APP_NAME?upper_case}_NVM_STATE_OPEN;
<#else>
                ${APP_NAME?lower_case}Data.nvmStateMachine = ${APP_NAME?upper_case}_NVM_STATE_DONE;
</#if>
            break;

<#if ("CONFIG_APP_SYS_NVM_WRITE" + "${HCONFIG_APP_INSTANCE}")?eval || ("CONFIG_APP_SYS_NVM_READ" + "${HCONFIG_APP_INSTANCE}")?eval>
        case ${APP_NAME?upper_case}_NVM_STATE_OPEN:
<#if ("CONFIG_APP_SYS_NVM_WRITE" + "${HCONFIG_APP_INSTANCE}")?eval>
            /* open the file to write - create file if it does not exist */
            ${APP_NAME?lower_case}NVMWriteHandle = SYS_FS_FileOpen(${APP_NAME?lower_case}NVMWritePath, SYS_FS_FILE_OPEN_WRITE_PLUS);
            if (${APP_NAME?lower_case}NVMWriteHandle == SYS_FS_HANDLE_INVALID)
            {
                FsError = SYS_FS_Error();
                ${APP_NAME?lower_case}Data.nvmStateMachine =  ${APP_NAME?upper_case}_NVM_STATE_ERROR;
                return;
            }
            nvm_${APP_NAME?lower_case}FsBytesWritten = 0;

</#if>
<#if ("CONFIG_APP_SYS_NVM_READ" + "${HCONFIG_APP_INSTANCE}")?eval>
            /* open the file to read */
            nvmReadHandle = SYS_FS_FileOpen(nvmReadPath, SYS_FS_FILE_OPEN_READ);
            if (nvmReadHandle == SYS_FS_HANDLE_INVALID)
            {
                FsError = SYS_FS_Error();
                ${APP_NAME?lower_case}Data.nvmStateMachine =  ${APP_NAME?upper_case}_NVM_STATE_ERROR;
                return;
            }
            nvm_${APP_NAME?lower_case}FsBytesRead = 0;

</#if>
            ${APP_NAME?lower_case}Data.nvmStateMachine =  ${APP_NAME?upper_case}_NVM_STATE_FILEOPS;
            break;

        case ${APP_NAME?upper_case}_NVM_STATE_FILEOPS:
<#if ("CONFIG_APP_SYS_NVM_WRITE" + "${HCONFIG_APP_INSTANCE}")?eval>
            if (${APP_NAME?lower_case}NVMWriteHandle != SYS_FS_HANDLE_INVALID)
            {
                size_t BytesWritten = SYS_FS_FileWrite(${APP_NAME?lower_case}NVMWriteHandle,
                    (const void *)${APP_NAME?lower_case}_nvm_data, sizeof(${APP_NAME?lower_case}_nvm_data) - nvm_${APP_NAME?lower_case}FsBytesWritten);
                if (BytesWritten == (size_t)-1)
                {
                    /* error */
                    FsError = SYS_FS_Error();
                    ${APP_NAME?lower_case}Data.nvmStateMachine =  ${APP_NAME?upper_case}_NVM_STATE_ERROR;
                    return;
                }
                else
                {
                    nvm_${APP_NAME?lower_case}FsBytesWritten += BytesWritten;
                    /* have we written all the bytes now? */
                    if (nvm_${APP_NAME?lower_case}FsBytesWritten >= sizeof(${APP_NAME?lower_case}_nvm_data))
                    {
                        SYS_FS_FileClose(${APP_NAME?lower_case}NVMWriteHandle);
                        ${APP_NAME?lower_case}NVMWriteHandle = SYS_FS_HANDLE_INVALID;
                    }
                }
            }

</#if>
<#if ("CONFIG_APP_SYS_NVM_READ" + "${HCONFIG_APP_INSTANCE}")?eval>
            if (nvmReadHandle != SYS_FS_HANDLE_INVALID)
            {
                size_t BytesRead = SYS_FS_FileRead(nvmReadHandle,
                    ${APP_NAME?lower_case}_nvm_read, sizeof(${APP_NAME?lower_case}_nvm_read) - nvm_${APP_NAME?lower_case}FsBytesRead);
                if (BytesRead == (size_t)-1)
                {
                    /* error */
                    FsError = SYS_FS_Error();
                    ${APP_NAME?lower_case}Data.nvmStateMachine =  ${APP_NAME?upper_case}_NVM_STATE_ERROR;
                    return;
                }
                else
                {
                    nvm_${APP_NAME?lower_case}FsBytesRead += BytesRead;
                    /* have we gathered all the bytes now? */
                    if (nvm_${APP_NAME?lower_case}FsBytesRead >= sizeof(${APP_NAME?lower_case}_nvm_read))
                    {
                        SYS_FS_FileClose(nvmReadHandle);
                        nvmReadHandle = SYS_FS_HANDLE_INVALID;
                    }
                }
            }

</#if>
            /* The file operations are finished when handle has been set back to SYS_FS_HANDLE_INVALID */
<#if ("CONFIG_APP_SYS_NVM_WRITE" + "${HCONFIG_APP_INSTANCE}")?eval && ("CONFIG_APP_SYS_NVM_READ" + "${HCONFIG_APP_INSTANCE}")?eval>
            if ((nvmReadHandle == SYS_FS_HANDLE_INVALID) && (${APP_NAME?lower_case}NVMWriteHandle == SYS_FS_HANDLE_INVALID))
<#elseif ("CONFIG_APP_SYS_NVM_READ" + "${HCONFIG_APP_INSTANCE}")?eval>
            if (nvmReadHandle == SYS_FS_HANDLE_INVALID)
<#else>
            if (${APP_NAME?lower_case}NVMWriteHandle == SYS_FS_HANDLE_INVALID)
</#if>
            {
                ${APP_NAME?lower_case}Data.nvmStateMachine =  ${APP_NAME?upper_case}_NVM_STATE_DONE;
            }
            break;
</#if>

        case ${APP_NAME?upper_case}_NVM_STATE_DONE:
            break;

        /* if we get to this state - something went wrong - check FsError for cause */
        case ${APP_NAME?upper_case}_NVM_STATE_ERROR:
<#if ("CONFIG_APP_SYS_NVM_READ" + "${HCONFIG_APP_INSTANCE}")?eval>
            if (nvmReadHandle != SYS_FS_HANDLE_INVALID)
            {
                /* clean-up open file */
                SYS_FS_FileClose(nvmReadHandle);
                nvmReadHandle = SYS_FS_HANDLE_INVALID;
            }

</#if>
<#if ("CONFIG_APP_SYS_NVM_WRITE" + "${HCONFIG_APP_INSTANCE}")?eval>
            if (${APP_NAME?lower_case}NVMWriteHandle != SYS_FS_HANDLE_INVALID)
            {
                /* clean-up open file */
                SYS_FS_FileClose(${APP_NAME?lower_case}NVMWriteHandle);
                ${APP_NAME?lower_case}NVMWriteHandle = SYS_FS_HANDLE_INVALID;
            }

</#if>
            break;
    }
}

</#macro>

<#macro macro_sys_nvm_app_c_initialize>

    /* Place the File System NVM state machine in its initial state */
    ${APP_NAME?lower_case}Data.nvmStateMachine = ${APP_NAME?upper_case}_NVM_STATE_START;
</#macro>

<#macro macro_sys_nvm_app_c_tasks_data>
</#macro>

<#macro macro_sys_nvm_app_c_tasks_state_init>
</#macro>

<#macro macro_sys_nvm_app_c_tasks_calls_after_init>
</#macro>

<#macro macro_sys_nvm_app_c_tasks_state_service_tasks>

            /* run the state machine for servicing the NVM file system */
            SYS_NVM_Task();
</#macro>

<#macro macro_sys_nvm_app_c_tasks_states>
</#macro>
