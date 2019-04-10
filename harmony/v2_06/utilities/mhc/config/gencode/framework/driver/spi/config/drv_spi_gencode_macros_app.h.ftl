
<#macro macro_drv_spi_app_h_includes>

typedef enum
{
    ${APP_NAME?upper_case}_SPI_STATE_START,
    ${APP_NAME?upper_case}_SPI_STATE_WAIT,
    ${APP_NAME?upper_case}_SPI_STATE_DONE
} ${APP_NAME?upper_case}_SPI_STATES;
</#macro>


<#macro macro_drv_spi_app_h_states>
</#macro>

<#macro macro_drv_spi_app_h_data>

    ${APP_NAME?upper_case}_SPI_STATES spiStateMachine;

    /* SPI Driver Handle  */
    DRV_HANDLE ${("CONFIG_APP_DRV_SPI_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval};

    /* SPI Buffer Handle */
    DRV_SPI_BUFFER_HANDLE drvSPIBufferHandle;
</#macro>

<#macro macro_drv_spi_app_h_callback_function_declarations>
</#macro>

<#macro macro_drv_spi_app_h_function_declarations>
</#macro>
