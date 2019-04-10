
<#macro macro_drv_spi_app_c_includes>
</#macro>

<#macro macro_drv_spi_app_c_global_data>
<#if ("CONFIG_APP_DRV_SPI_BLOCKING" + "${HCONFIG_APP_INSTANCE}")?eval>
static uint8_t __attribute__ ((aligned (16))) ${APP_NAME?lower_case}_spi_tx_buffer[] = "${("CONFIG_APP_DRV_SPI_BLOCKING_TX_STRING" + "${HCONFIG_APP_INSTANCE}")?eval}";
static uint8_t __attribute__ ((aligned (16))) ${APP_NAME?lower_case}_spi_rx_buffer[sizeof(${APP_NAME?lower_case}_spi_tx_buffer)];
</#if>
<#if ("CONFIG_APP_DRV_SPI_NON_BLOCKING_TX" + "${HCONFIG_APP_INSTANCE}")?eval>
static uint8_t __attribute__ ((aligned (16))) ${APP_NAME?lower_case}_spi_tx_buffer[] = "${("CONFIG_APP_DRV_SPI_NON_BLOCKING_TX_STRING" + "${HCONFIG_APP_INSTANCE}")?eval}";
</#if>
<#if ("CONFIG_APP_DRV_SPI_NON_BLOCKING_RX" + "${HCONFIG_APP_INSTANCE}")?eval>
static uint8_t __attribute__ ((aligned (16))) ${APP_NAME?lower_case}_spi_rx_buffer[${("CONFIG_APP_DRV_SPI_RX_BUFFER_SIZE" + "${HCONFIG_APP_INSTANCE}")?eval}];
</#if>
</#macro>

<#macro macro_drv_spi_app_c_callback_functions>
</#macro>

<#macro macro_drv_spi_app_c_local_functions>
/* state machine for the SPI */
static void SPI_Task(void)
{
    /* run the state machine here for SPI */
    switch (${APP_NAME?lower_case}Data.spiStateMachine)
    {
        default:
        case ${APP_NAME?upper_case}_SPI_STATE_START:
<#if ("CONFIG_APP_DRV_SPI_NON_BLOCKING_RX" + "${HCONFIG_APP_INSTANCE}")?eval || ("CONFIG_APP_DRV_SPI_NON_BLOCKING_TX" + "${HCONFIG_APP_INSTANCE}")?eval>
            /* set the state to 'wait' early so that the interrupt doesn't
                finish fast and write the state and then is overwritten */
            ${APP_NAME?lower_case}Data.spiStateMachine =  ${APP_NAME?upper_case}_SPI_STATE_WAIT;
</#if>
<#if ("CONFIG_APP_DRV_SPI_BLOCKING" + "${HCONFIG_APP_INSTANCE}")?eval>
            /* Blocking: When the call exits, all data has been exchanged (if we get a valid handle) */
            ${APP_NAME?lower_case}Data.drvSPIBufferHandle = DRV_SPI_BufferAddWriteRead(${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_SPI_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval},
                ${APP_NAME?lower_case}_spi_tx_buffer, sizeof(${APP_NAME?lower_case}_spi_tx_buffer),
                ${APP_NAME?lower_case}_spi_rx_buffer, sizeof(${APP_NAME?lower_case}_spi_rx_buffer),
                0, 0);

            if (DRV_SPI_BUFFER_HANDLE_INVALID != ${APP_NAME?lower_case}Data.drvSPIBufferHandle)
            {
                /* Blocking Write/Read has finished */
                ${APP_NAME?lower_case}Data.spiStateMachine =  ${APP_NAME?upper_case}_SPI_STATE_DONE;
            }
</#if>
<#if ("CONFIG_APP_DRV_SPI_NON_BLOCKING_RX" + "${HCONFIG_APP_INSTANCE}")?eval && ("CONFIG_APP_DRV_SPI_NON_BLOCKING_TX" + "${HCONFIG_APP_INSTANCE}")?eval>
            ${APP_NAME?lower_case}Data.drvSPIBufferHandle = DRV_SPI_BufferAddWriteRead(${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_SPI_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval},
                ${APP_NAME?lower_case}_spi_tx_buffer, sizeof(${APP_NAME?lower_case}_spi_tx_buffer),
                ${APP_NAME?lower_case}_spi_rx_buffer, sizeof(${APP_NAME?lower_case}_spi_rx_buffer),
                0, 0);
<#elseif ("CONFIG_APP_DRV_SPI_NON_BLOCKING_RX" + "${HCONFIG_APP_INSTANCE}")?eval>
            ${APP_NAME?lower_case}Data.drvSPIBufferHandle = DRV_SPI_BufferAddRead(${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_SPI_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval},
                ${APP_NAME?lower_case}_spi_rx_buffer, sizeof(${APP_NAME?lower_case}_spi_rx_buffer),
                0, 0);
<#elseif ("CONFIG_APP_DRV_SPI_NON_BLOCKING_TX" + "${HCONFIG_APP_INSTANCE}")?eval>
            ${APP_NAME?lower_case}Data.drvSPIBufferHandle = DRV_SPI_BufferAddWrite(${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_SPI_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval},
                ${APP_NAME?lower_case}_spi_tx_buffer, sizeof(${APP_NAME?lower_case}_spi_tx_buffer),
                0, 0);
</#if>
<#if ("CONFIG_APP_DRV_SPI_NON_BLOCKING_RX" + "${HCONFIG_APP_INSTANCE}")?eval ||
    ("CONFIG_APP_DRV_SPI_NON_BLOCKING_TX" + "${HCONFIG_APP_INSTANCE}")?eval ||
    ("CONFIG_APP_DRV_SPI_BLOCKING" + "${HCONFIG_APP_INSTANCE}")?eval>

            if (DRV_SPI_BUFFER_HANDLE_INVALID == ${APP_NAME?lower_case}Data.drvSPIBufferHandle)
            {
                /* try again if we get a bad handle */
                ${APP_NAME?lower_case}Data.spiStateMachine =  ${APP_NAME?upper_case}_SPI_STATE_START;
            }
</#if>
        break;
<#if ("CONFIG_APP_DRV_SPI_NON_BLOCKING_RX" + "${HCONFIG_APP_INSTANCE}")?eval || ("CONFIG_APP_DRV_SPI_NON_BLOCKING_TX" + "${HCONFIG_APP_INSTANCE}")?eval>

        case ${APP_NAME?upper_case}_SPI_STATE_WAIT:
        {
            if ( DRV_SPI_BufferStatus(${APP_NAME?lower_case}Data.drvSPIBufferHandle) & DRV_SPI_BUFFER_EVENT_COMPLETE)
            {
                ${APP_NAME?lower_case}Data.spiStateMachine = ${APP_NAME?upper_case}_SPI_STATE_DONE;
            }
        }
        break;
</#if>

        case ${APP_NAME?upper_case}_SPI_STATE_DONE:
        break;
    }
}

</#macro>

<#macro macro_drv_spi_app_c_initialize>
    ${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_SPI_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval} = DRV_HANDLE_INVALID;
</#macro>

<#macro macro_drv_spi_app_c_tasks_data>
</#macro>

<#macro macro_drv_spi_app_c_tasks_state_init>

            if (DRV_HANDLE_INVALID == ${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_SPI_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval})
            {
<#if  (("CONFIG_APP_DRV_SPI_NON_BLOCKING_RX" + "${HCONFIG_APP_INSTANCE}")?eval && ("CONFIG_APP_DRV_SPI_NON_BLOCKING_TX" + "${HCONFIG_APP_INSTANCE}")?eval)>
                ${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_SPI_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval} = DRV_SPI_Open(${("CONFIG_APP_DRV_SPI_INSTANCE_INDEX" + "${HCONFIG_APP_INSTANCE}")?eval}, DRV_IO_INTENT_READWRITE);
<#elseif ("CONFIG_APP_DRV_SPI_NON_BLOCKING_RX" + "${HCONFIG_APP_INSTANCE}")?eval>
                ${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_SPI_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval} = DRV_SPI_Open(${("CONFIG_APP_DRV_SPI_INSTANCE_INDEX" + "${HCONFIG_APP_INSTANCE}")?eval}, DRV_IO_INTENT_READ);

<#elseif ("CONFIG_APP_DRV_SPI_NON_BLOCKING_TX" + "${HCONFIG_APP_INSTANCE}")?eval>
                ${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_SPI_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval} = DRV_SPI_Open(${("CONFIG_APP_DRV_SPI_INSTANCE_INDEX" + "${HCONFIG_APP_INSTANCE}")?eval}, DRV_IO_INTENT_WRITE);
<#else>
                ${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_SPI_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval} = DRV_SPI_Open(${("CONFIG_APP_DRV_SPI_INSTANCE_INDEX" + "${HCONFIG_APP_INSTANCE}")?eval}, DRV_IO_INTENT_READWRITE);
</#if>
                appInitialized &= (DRV_HANDLE_INVALID != ${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_SPI_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval});
            }
</#macro>

<#macro macro_drv_spi_app_c_tasks_calls_after_init>
                /* initialize the SPI state machine */
                ${APP_NAME?lower_case}Data.spiStateMachine = ${APP_NAME?upper_case}_SPI_STATE_START;
</#macro>

<#macro macro_drv_spi_app_c_tasks_state_service_tasks>
            /* run the state machine for servicing the SPI */
            SPI_Task();
</#macro>

<#macro macro_drv_spi_app_c_tasks_states>
</#macro>
