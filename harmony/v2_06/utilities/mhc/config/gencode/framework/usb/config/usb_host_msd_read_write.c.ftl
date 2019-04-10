
<#macro macro_usb_host_msd_read_write_c_includes>
</#macro>

<#macro macro_usb_host_msd_read_write_c_global_data>
<#if ("CONFIG_APP_USB_HOST_MSD_READ" + "${HCONFIG_APP_INSTANCE}")?eval>
static char ${APP_NAME?lower_case}MsdReadPath[] = "${("CONFIG_SYS_FS_MEDIA_MOUNT_1_NAME_IDX" + ("CONFIG_APP_USB_HOST_MSD_READ_MEDIA" + "${HCONFIG_APP_INSTANCE}")?eval)?eval}/${("CONFIG_APP_USB_HOST_MSD_READ_FILENAME" + "${HCONFIG_APP_INSTANCE}")?eval}";
static uint8_t __attribute__ ((aligned (16))) ${APP_NAME?lower_case}MsdReadData[${("CONFIG_APP_USB_HOST_MSD_READ_SIZE" + "${HCONFIG_APP_INSTANCE}")?eval}];

</#if>
<#if ("CONFIG_APP_USB_HOST_MSD_WRITE" + "${HCONFIG_APP_INSTANCE}")?eval>
static char ${APP_NAME?lower_case}MsdWritePath[] = "${("CONFIG_SYS_FS_MEDIA_MOUNT_1_NAME_IDX" + ("CONFIG_APP_USB_HOST_MSD_WRITE_MEDIA" + "${HCONFIG_APP_INSTANCE}")?eval)?eval}/${("CONFIG_APP_USB_HOST_MSD_WRITE_FILENAME" + "${HCONFIG_APP_INSTANCE}")?eval}";
static uint8_t __attribute__ ((aligned (16))) ${APP_NAME?lower_case}MsdWriteData[] = "${("CONFIG_APP_USB_HOST_MSD_WRITE_STRING" + "${HCONFIG_APP_INSTANCE}")?eval}";

</#if>
</#macro>


<#macro macro_usb_host_msd_read_write_c_callback_functions>
</#macro>

<#macro macro_usb_host_msd_read_write_c_local_functions>
void ${APP_NAME?upper_case}_SYSFSEventHandler(SYS_FS_EVENT event, void * eventData, uintptr_t context)
{
    switch (event)
    {
        case SYS_FS_EVENT_MOUNT:
            ${APP_NAME?lower_case}Data.msdDeviceIsConnected = true;
            break;
            
        case SYS_FS_EVENT_UNMOUNT:
            ${APP_NAME?lower_case}Data.msdDeviceIsConnected = false;
            break;
            
        default:
            break;
    }
}


<#if ("CONFIG_APP_USB_HOST_MSD_READ" + "${HCONFIG_APP_INSTANCE}")?eval>
/* state machine for the File System Read */
static void ${APP_NAME?upper_case}_MSD_READ_Task(void)
{
    static SYS_FS_ERROR FsError;
    static uint8_t HandleAttempts;
    static uint32_t TotalBytesRead;
    static SYS_FS_HANDLE FileHandle;

    switch (${APP_NAME?lower_case}Data.msdReadStateMachine)
    {
        default:
        case ${APP_NAME?upper_case}_MSD_READ_STATE_START:
            HandleAttempts = 0;
            ${APP_NAME?lower_case}Data.msdReadStateMachine =  ${APP_NAME?upper_case}_MSD_READ_STATE_OPEN;
            break;

        case ${APP_NAME?upper_case}_MSD_READ_STATE_OPEN:
            /* open the file to read */
            FileHandle = SYS_FS_FileOpen(${APP_NAME?lower_case}MsdReadPath, SYS_FS_FILE_OPEN_READ);
            if (FileHandle == SYS_FS_HANDLE_INVALID)
            {
                /* try 10 times to get the handle while auto-mount happens */
                if (++HandleAttempts > 10)
                {
                    FsError = SYS_FS_Error();
                    ${APP_NAME?lower_case}Data.msdReadStateMachine =  ${APP_NAME?upper_case}_MSD_READ_STATE_ERROR;
                }
            }
            else
            {
                TotalBytesRead = 0;
                ${APP_NAME?lower_case}Data.msdReadStateMachine =  ${APP_NAME?upper_case}_MSD_READ_STATE_FILEOPS;
            }
            break;

        case ${APP_NAME?upper_case}_MSD_READ_STATE_FILEOPS:
            {
                size_t BytesRead = SYS_FS_FileRead(FileHandle, ${APP_NAME?lower_case}MsdReadData,
                    sizeof(${APP_NAME?lower_case}MsdReadData) - TotalBytesRead);
                if (BytesRead == (size_t)-1)
                {
                    /* error */
                    FsError = SYS_FS_Error();
                    ${APP_NAME?lower_case}Data.msdReadStateMachine =  ${APP_NAME?upper_case}_MSD_READ_STATE_ERROR;
                    return;
                }
                else
                {
                    TotalBytesRead += BytesRead;
                    if (TotalBytesRead >= sizeof(${APP_NAME?lower_case}MsdReadData) ||
                        SYS_FS_FileEOF(FileHandle))
                    {
                        SYS_FS_FileClose(FileHandle);
                        ${APP_NAME?lower_case}Data.msdReadStateMachine =  ${APP_NAME?upper_case}_MSD_READ_STATE_DONE;
                    }
                }
            }
            break;

        case ${APP_NAME?upper_case}_MSD_READ_STATE_DONE:
            break;

        /* if we get to this state - something went wrong - check FsError for status */
        case ${APP_NAME?upper_case}_MSD_READ_STATE_ERROR:
            break;
    }
}



</#if>
<#if ("CONFIG_APP_USB_HOST_MSD_WRITE" + "${HCONFIG_APP_INSTANCE}")?eval>
/* state machine for the File System Write */
static void ${APP_NAME?upper_case}_MSD_WRITE_Task(void)
{
    static SYS_FS_ERROR FsError;
    static uint8_t HandleAttempts;
    static uint32_t TotalBytesWritten;
    static SYS_FS_HANDLE FileHandle;

    switch (${APP_NAME?lower_case}Data.msdWriteStateMachine)
    {
        default:
        case ${APP_NAME?upper_case}_MSD_WRITE_STATE_START:
            HandleAttempts = 0;
            ${APP_NAME?lower_case}Data.msdWriteStateMachine = ${APP_NAME?upper_case}_MSD_WRITE_STATE_OPEN;
            break;

        case ${APP_NAME?upper_case}_MSD_WRITE_STATE_OPEN:
            /* open the file to write - create file if it does not exist */
            FileHandle = SYS_FS_FileOpen(${APP_NAME?lower_case}MsdWritePath, SYS_FS_FILE_OPEN_WRITE_PLUS);
            if (FileHandle == SYS_FS_HANDLE_INVALID)
            {
                /* try 10 times to get the handle - allow auto-mount to finish */
                if (++HandleAttempts > 10)
                {
                    FsError = SYS_FS_Error();
                    ${APP_NAME?lower_case}Data.msdWriteStateMachine = ${APP_NAME?upper_case}_MSD_WRITE_STATE_ERROR;
                }
            }
            else
            {
                TotalBytesWritten = 0;
                ${APP_NAME?lower_case}Data.msdWriteStateMachine = ${APP_NAME?upper_case}_MSD_WRITE_STATE_FILEOPS;
            }
            break;

        case ${APP_NAME?upper_case}_MSD_WRITE_STATE_FILEOPS:
            {
                size_t BytesWritten = SYS_FS_FileWrite(FileHandle, (const void *)${APP_NAME?lower_case}MsdWriteData,
                    sizeof(${APP_NAME?lower_case}MsdWriteData) - TotalBytesWritten);
                if (BytesWritten == (size_t)-1)
                {
                    /* error */
                    FsError = SYS_FS_Error();
                    ${APP_NAME?lower_case}Data.msdWriteStateMachine = ${APP_NAME?upper_case}_MSD_WRITE_STATE_ERROR;
                    return;
                }
                else
                {
                    TotalBytesWritten += BytesWritten;
                    /* have we written all the bytes now? */
                    if (TotalBytesWritten >= sizeof(${APP_NAME?lower_case}MsdWriteData))
                    {
                        SYS_FS_FileClose(FileHandle);
                        ${APP_NAME?lower_case}Data.msdWriteStateMachine = ${APP_NAME?upper_case}_MSD_WRITE_STATE_DONE;
                    }
                }
            }
            break;

        case ${APP_NAME?upper_case}_MSD_WRITE_STATE_DONE:
            break;

        /* if we get to this state - something went wrong - check FsError for status */
        case ${APP_NAME?upper_case}_MSD_WRITE_STATE_ERROR:
            break;
    }
}



</#if>
</#macro>

<#macro macro_usb_host_msd_read_write_c_initialize>
<#if ("CONFIG_APP_USB_HOST_MSD_READ" + "${HCONFIG_APP_INSTANCE}")?eval>
    /* Place the File System state machine in its initial state */
    ${APP_NAME?lower_case}Data.msdReadStateMachine = ${APP_NAME?upper_case}_MSD_READ_STATE_START;

</#if>
<#if ("CONFIG_APP_USB_HOST_MSD_WRITE" + "${HCONFIG_APP_INSTANCE}")?eval>
    /* Place the File System state machine in its initial state */
    ${APP_NAME?lower_case}Data.msdWriteStateMachine = ${APP_NAME?upper_case}_MSD_WRITE_STATE_START;

</#if>
   /* Set the event handler and enable the bus */
    SYS_FS_EventHandlerSet(${APP_NAME?upper_case}_SYSFSEventHandler, (uintptr_t)NULL);
    USB_HOST_BusEnable(0);

</#macro>

<#macro macro_usb_host_msd_read_write_c_tasks_data>
</#macro>

<#macro macro_usb_host_msd_read_write_c_tasks_state_init>
            appInitialized = appInitialized && USB_HOST_BusIsEnabled(0);
            appInitialized = appInitialized && ${APP_NAME?lower_case}Data.msdDeviceIsConnected;

</#macro>

<#macro macro_usb_host_msd_read_write_c_tasks_calls_after_init>
</#macro>

<#macro macro_usb_host_msd_read_write_c_tasks_state_service_tasks>
<#if ("CONFIG_APP_USB_HOST_MSD_READ" + "${HCONFIG_APP_INSTANCE}")?eval>
            /* run the state machine for servicing the file system READ */
            ${APP_NAME?upper_case}_MSD_READ_Task();

</#if>
<#if ("CONFIG_APP_USB_HOST_MSD_WRITE" + "${HCONFIG_APP_INSTANCE}")?eval>
            /* run the state machine for servicing the file system WRITE */
            ${APP_NAME?upper_case}_MSD_WRITE_Task();

</#if>
</#macro>

<#macro macro_usb_host_msd_read_write_c_tasks_states>
</#macro>
