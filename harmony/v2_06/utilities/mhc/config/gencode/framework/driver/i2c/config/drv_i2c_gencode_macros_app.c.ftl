<#-- drv_i2c_gencode_macros_app.c.ftl -->

<#--
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************

#include "${APP_NAME?lower_case}.h"
-->
<#macro macro_drv_i2c_app_c_includes>
</#macro>
<#--
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data
*/
-->
<#macro macro_drv_i2c_app_c_global_data>
<#if ("CONFIG_APP_DRV_I2C_MASTER_TRANSMIT" + "${HCONFIG_APP_INSTANCE}")?eval>
// This is the string to write to the slave device.  Be aware that the double quotes adds a null byte at the end of the string.
// So, writing "Hello World!" actually transmits 13 bytes.
uint8_t ${APP_NAME?lower_case}WriteString[] = "${("CONFIG_APP_DRV_I2C_TX_STRING" + "${HCONFIG_APP_INSTANCE}")?eval}";
const I2C_SLAVE_ADDRESS_VALUE ${APP_NAME?lower_case}SlaveAddress = ${("CONFIG_APP_DRV_I2C_SLAVE_ADDRESS" + "${HCONFIG_APP_INSTANCE}")?eval};
</#if>

<#if ("CONFIG_APP_DRV_I2C_SLAVE_RECEIVE" + "${HCONFIG_APP_INSTANCE}")?eval>
// This is the buffer to fill while receiving characters.  Be aware that if the master device sends more than expected, this
// buffer will simply wrap around and data will be lost.
uint8_t ${APP_NAME?lower_case}ReadBuffer[${("CONFIG_APP_DRV_I2C_RX_SIZE" + "${HCONFIG_APP_INSTANCE}")?eval}] = { 0 };
const I2C_SLAVE_ADDRESS_VALUE ${APP_NAME?lower_case}SlaveAddress = ${("CONFIG_DRV_I2C_SLAVE_ADDRESS_VALUE_IDX" + "${('CONFIG_APP_DRV_I2C_INSTANCE_INDEX' + '${HCONFIG_APP_INSTANCE}')?eval}")?eval};
</#if>

</#macro>

<#--
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
-->
<#macro macro_drv_i2c_app_c_callback_functions>
<#if ("CONFIG_APP_DRV_I2C_SLAVE_RECEIVE" + "${HCONFIG_APP_INSTANCE}")?eval>
void ${("CONFIG_DRV_I2C_SLAVE_CALLBACK_FUNCTION_IDX" + "${(\"CONFIG_APP_DRV_I2C_INSTANCE_INDEX\" + \"${HCONFIG_APP_INSTANCE}\")?eval}")?eval}(DRV_I2C_BUFFER_EVENT event, void * context)
{
        switch (event)
    {
        case DRV_I2C_BUFFER_SLAVE_READ_REQUESTED:
            ${APP_NAME?lower_case}Data.I2CBufferHandle = DRV_I2C_Receive (   ${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_I2C_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval}, 
                                                                ${APP_NAME?lower_case}SlaveAddress,
                                                                ${APP_NAME?lower_case}ReadBuffer,
                                                                sizeof(${APP_NAME?lower_case}ReadBuffer),
                                                                NULL);
            break;
        case DRV_I2C_BUFFER_SLAVE_WRITE_REQUESTED:                        
            ${APP_NAME?lower_case}Data.I2CBufferHandle = DRV_I2C_Transmit (   ${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_I2C_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval}, 
                                                                ${APP_NAME?lower_case}SlaveAddress,
                                                                ${APP_NAME?lower_case}ReadBuffer,
                                                                sizeof(${APP_NAME?lower_case}ReadBuffer),
                                                                NULL);
            Nop();
            break;
        default:
            break;
    }
}
</#if>
</#macro>

<#--
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
-->
<#macro macro_drv_i2c_app_c_local_functions>

/* Application's i2c Setup Function */
static void I2C_Setup( void )
{
    ${APP_NAME?lower_case}Data.i2cStates = ${APP_NAME?upper_case}_I2C_START;
}
/******************************************************************************
  Function:
    static void ${APP_NAME?upper_case}_I2C_Task (void)
    
   Remarks:
    Allows a polled state machine to manage i2c testing.

*/
static void ${APP_NAME?upper_case}_I2C_Task (void)
{		
	switch(${APP_NAME?lower_case}Data.i2cStates)
	{
		default:
		
		case ${APP_NAME?upper_case}_I2C_START:
        {
            <#if ("CONFIG_APP_DRV_I2C_MASTER_TRANSMIT" + "${HCONFIG_APP_INSTANCE}")?eval>
            // Switch to the Transmit State.
            ${APP_NAME?lower_case}Data.i2cStates = ${APP_NAME?upper_case}_I2C_TRANSMIT;
            </#if>
            <#if ("CONFIG_APP_DRV_I2C_SLAVE_RECEIVE" + "${HCONFIG_APP_INSTANCE}")?eval>
            // Switch to the Receive State.
            ${APP_NAME?lower_case}Data.i2cStates = ${APP_NAME?upper_case}_I2C_RECEIVE;
            </#if>
        
			break;
		}	
        <#if ("CONFIG_APP_DRV_I2C_MASTER_TRANSMIT" + "${HCONFIG_APP_INSTANCE}")?eval>
		case ${APP_NAME?upper_case}_I2C_TRANSMIT:
        {
            // If no transmission has occured yet...
            if(${APP_NAME?lower_case}Data.I2CBufferHandle == NULL)
            {
                ${APP_NAME?lower_case}Data.I2CBufferHandle = DRV_I2C_Transmit ( ${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_I2C_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval},
                                                        ${APP_NAME?lower_case}SlaveAddress,
                                                        ${APP_NAME?lower_case}WriteString,
                                                        sizeof(${APP_NAME?lower_case}WriteString), 
                                                        NULL);
            } else {
                ${APP_NAME?lower_case}Data.I2CBufferEvent = DRV_I2C_TransferStatusGet ( ${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_I2C_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval},
                         ${APP_NAME?lower_case}Data.I2CBufferHandle );
                if(${APP_NAME?lower_case}Data.I2CBufferEvent == DRV_I2C_BUFFER_EVENT_COMPLETE) 
                    {
                        ${APP_NAME?lower_case}Data.i2cStates = ${APP_NAME?upper_case}_I2C_DONE;
                    }
                if(${APP_NAME?lower_case}Data.I2CBufferEvent == DRV_I2C_BUFFER_EVENT_ERROR)
                    {
                        ${APP_NAME?lower_case}Data.i2cStates = ${APP_NAME?upper_case}_I2C_ERROR;
                    }
            }

            break;
		}
        </#if>
        <#if ("CONFIG_APP_DRV_I2C_SLAVE_RECEIVE" + "${HCONFIG_APP_INSTANCE}")?eval>
		case ${APP_NAME?upper_case}_I2C_RECEIVE:
        {
            // If Driver has not started looking for data ...
            if(${APP_NAME?lower_case}Data.I2CBufferHandle != NULL)
            {

                if ( PLIB_I2C_StopWasDetected ( ${APP_NAME?upper_case}_PERPH_I2C_INDEX ) )
                {
                        ${APP_NAME?lower_case}Data.i2cStates = ${APP_NAME?upper_case}_I2C_DONE;
                } else {
                    ${APP_NAME?lower_case}Data.I2CBufferEvent = DRV_I2C_TransferStatusGet ( ${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_I2C_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval},
                             ${APP_NAME?lower_case}Data.I2CBufferHandle );
                    if(${APP_NAME?lower_case}Data.I2CBufferEvent == DRV_I2C_BUFFER_EVENT_COMPLETE) 
                        {
                            ${APP_NAME?lower_case}Data.i2cStates = ${APP_NAME?upper_case}_I2C_DONE;
                        }
                    if(${APP_NAME?lower_case}Data.I2CBufferEvent == DRV_I2C_BUFFER_EVENT_ERROR)
                        {
                            ${APP_NAME?lower_case}Data.i2cStates = ${APP_NAME?upper_case}_I2C_ERROR;
                        }
                }
            }
			break;
		}	
        </#if>
		case ${APP_NAME?upper_case}_I2C_DONE:
		{
			break;
        }
        case ${APP_NAME?upper_case}_I2C_ERROR:
		{
            break;
        }

    }
}
</#macro>

<#--
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void ${APP_NAME?upper_case}_Initialize ( void )

  Remarks:
    See prototype in ${APP_NAME?lower_case}.h.
 */

void ${APP_NAME?upper_case}_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    ${APP_NAME?lower_case}Data.state = ${APP_NAME?upper_case}_STATE_INIT;
-->
<#macro macro_drv_i2c_app_c_initialize>
    ${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_I2C_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval} = DRV_HANDLE_INVALID;
</#macro>

<#--
}


/******************************************************************************
  Function:
    void ${APP_NAME?upper_case}_Tasks ( void )

  Remarks:
    See prototype in ${APP_NAME?lower_case}.h.
 */

void ${APP_NAME?upper_case}_Tasks ( void )
{
-->
<#macro macro_drv_i2c_app_c_tasks_data>
</#macro>

<#--
    /* Check the application's current state. */
    switch ( ${APP_NAME?lower_case}Data.state )
    {
        /* Application's initial state. */
        case ${APP_NAME?upper_case}_STATE_INIT:
        {
            bool appInitialized = true;
-->   
<#macro macro_drv_i2c_app_c_tasks_state_init>
            if (${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_I2C_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval} == DRV_HANDLE_INVALID)
            {
                ${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_I2C_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval} = DRV_I2C_Open(${APP_NAME?upper_case}_DRV_I2C_INDEX, DRV_IO_INTENT_EXCLUSIVE);
                appInitialized &= ( DRV_HANDLE_INVALID != ${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_I2C_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval} );
            }
</#macro>    

<#--        
            if (appInitialized)
            {
-->
<#macro macro_drv_i2c_app_c_tasks_calls_after_init>
                I2C_Setup();
</#macro>

<#--            
                ${APP_NAME?lower_case}Data.state = ${APP_NAME?upper_case}_STATE_SERVICE_TASKS;
            }
            break;
        }

        case ${APP_NAME?upper_case}_STATE_SERVICE_TASKS:
        {
-->
<#macro macro_drv_i2c_app_c_tasks_state_service_tasks>
		    /* Run the state machine for servicing I2C */
            ${APP_NAME?upper_case}_I2C_Task();
</#macro>

<#--        
            break;
        }

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}
-->

