<#macro macro_drv_sqi_app_c_includes>
</#macro>

<#macro macro_drv_sqi_app_c_global_data>
<#if ("CONFIG_APP_DRV_SST26_SQI_DMA" + "${HCONFIG_APP_INSTANCE}")?eval>
static uint32_t __attribute__ ((coherent, aligned (16))) ${APP_NAME?lower_case}_sst26_write_buffer[XFER_SIZE];
static uint32_t __attribute__ ((coherent, aligned (16))) ${APP_NAME?lower_case}_sst26_read_buffer[XFER_SIZE];
</#if>
</#macro>

<#macro macro_drv_sqi_app_c_callback_functions>
void ${APP_NAME?upper_case}_SST26EventHandler
(
	DRV_SST26_EVENT event, 
    DRV_SST26_COMMAND_HANDLE commandHandle,
    uintptr_t contextHandle
)
{
	switch(event)
    {
		case DRV_SST26_EVENT_COMMAND_COMPLETE:
		{
			${APP_NAME?lower_case}Data.operationComplete = true;

			break;
        }
		
        case DRV_SST26_EVENT_COMMAND_ERROR:
		{
			${APP_NAME?lower_case}Data.operationComplete = false;
			
			break;
		}

        default:
			
			break;
    }
}
</#macro>

<#macro macro_drv_sqi_app_c_local_functions>
/* State machine for the SST26 */
static void SST26_Task(void)
{
	SYS_FS_MEDIA_GEOMETRY * sst26Geometry;
	uint32_t eraseBlockSize, writeBlockSize, readBlockSize;

    /* State machine for SST26 */
    switch (${APP_NAME?lower_case}Data.sst26StateMachine)
    {
        default:

		case ${APP_NAME?upper_case}_SQI_STATE_SST26_GET_GEOMETRY:
		{
			/* Read SST26 memory organization */
			sst26Geometry = DRV_SST26_GeometryGet(${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_SST26_SQI_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval});
			
			/* Read erase, write and read block sizes */
			eraseBlockSize = (sst26Geometry->geometryTable +2)->blockSize;
			writeBlockSize = (sst26Geometry->geometryTable +1)->blockSize;
			readBlockSize = sst26Geometry->geometryTable->blockSize;
			
			/* Calculate starting block for erase, write and read operations */
			${APP_NAME?lower_case}Data.eraseStartBlock = SST26_BLOCK_START_ADDRESS/eraseBlockSize;
			${APP_NAME?lower_case}Data.writeStartBlock = SST26_BLOCK_START_ADDRESS/writeBlockSize;
			${APP_NAME?lower_case}Data.readStartBlock  = SST26_BLOCK_START_ADDRESS/readBlockSize;
			
			/* Calculate number of blocks based on the size of transfer and block sizes */
			${APP_NAME?lower_case}Data.numEraseBlocks = (XFER_SIZE > eraseBlockSize)? XFER_SIZE/eraseBlockSize : 1;
			${APP_NAME?lower_case}Data.numWriteBlocks = XFER_SIZE/writeBlockSize;
			${APP_NAME?lower_case}Data.numReadBlocks  = XFER_SIZE/readBlockSize;
			
			${APP_NAME?lower_case}Data.sst26StateMachine = ${APP_NAME?upper_case}_SQI_STATE_SST26_ERASE;
			
			break;
		}
        
		case ${APP_NAME?upper_case}_SQI_STATE_SST26_ERASE:
		{
			${APP_NAME?lower_case}Data.operationComplete = false;
		
			/* Erase flash block(s) */
			DRV_SST26_Erase(${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_SST26_SQI_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval}, &${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_SST26_SQI_CMD_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval}, ${APP_NAME?lower_case}Data.eraseStartBlock, ${APP_NAME?lower_case}Data.numEraseBlocks);
			
			/* Check for valid handle */
			if(DRV_SST26_COMMAND_HANDLE_INVALID != ${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_SST26_SQI_CMD_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval})
			{
				${APP_NAME?lower_case}Data.sst26StateMachine = ${APP_NAME?upper_case}_SQI_STATE_SST26_ERASE_DONE;
			}
			else
			{
				/* Stay in the same state until a valid handle is returned */
			}		

			break;
		}
		
		case ${APP_NAME?upper_case}_SQI_STATE_SST26_ERASE_DONE:
		{
			/* Wait until erase operation is complete */
			if (${APP_NAME?lower_case}Data.operationComplete)
			{
				${APP_NAME?lower_case}Data.sst26StateMachine = ${APP_NAME?upper_case}_SQI_STATE_SST26_WRITE;
			}		

			break;
		}

		case ${APP_NAME?upper_case}_SQI_STATE_SST26_WRITE:
		{
			${APP_NAME?lower_case}Data.operationComplete = false;
			
			/* Populate write buffer */
			memset (&${APP_NAME?lower_case}_sst26_write_buffer, 0xA5A5A5A5, sizeof(${APP_NAME?lower_case}_sst26_write_buffer));

			/* Write flash block(s) */
			DRV_SST26_Write(${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_SST26_SQI_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval}, &${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_SST26_SQI_CMD_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval}, ${APP_NAME?lower_case}_sst26_write_buffer, ${APP_NAME?lower_case}Data.writeStartBlock, ${APP_NAME?lower_case}Data.numWriteBlocks);
			
			/* Check for valid handle */
			if(DRV_SST26_COMMAND_HANDLE_INVALID != ${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_SST26_SQI_CMD_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval})
			{
				${APP_NAME?lower_case}Data.sst26StateMachine = ${APP_NAME?upper_case}_SQI_STATE_SST26_WRITE_DONE;
			}
			else
			{
				/* Stay in the same state until a valid handle is returned */
			}

			break;
		}
		
		case ${APP_NAME?upper_case}_SQI_STATE_SST26_WRITE_DONE:
		{
			/* Wait until write operation is complete */
			if (${APP_NAME?lower_case}Data.operationComplete)
			{
				${APP_NAME?lower_case}Data.sst26StateMachine = ${APP_NAME?upper_case}_SQI_STATE_SST26_READ;
			}		

			break;
		}

		case ${APP_NAME?upper_case}_SQI_STATE_SST26_READ:
		{
			${APP_NAME?lower_case}Data.operationComplete = false;
			
			/* Read flash block(s) */
			DRV_SST26_Read(${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_SST26_SQI_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval}, &${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_SST26_SQI_CMD_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval}, ${APP_NAME?lower_case}_sst26_read_buffer, ${APP_NAME?lower_case}Data.readStartBlock, ${APP_NAME?lower_case}Data.numReadBlocks);
			
			/* Check for valid handle */
			if(DRV_SST26_COMMAND_HANDLE_INVALID != ${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_SST26_SQI_CMD_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval})
			{
				${APP_NAME?lower_case}Data.sst26StateMachine = ${APP_NAME?upper_case}_SQI_STATE_SST26_READ_DONE;
			}
			else
			{
				/* Stay in the same state until a valid handle is returned */
			}

			break;
		}

		case ${APP_NAME?upper_case}_SQI_STATE_SST26_READ_DONE:
		{
			/* Wait until read operation is complete */
			if (${APP_NAME?lower_case}Data.operationComplete)
			{
				/* Compare read buffer to write buffer */
				${APP_NAME?lower_case}Data.compareStatus = memcmp(${APP_NAME?lower_case}_sst26_read_buffer, ${APP_NAME?lower_case}_sst26_write_buffer, XFER_SIZE);
				
				/* Check the memcmp return value and jump to appropriate state (${APP_NAME?upper_case}_SQI_STATE_DONE - SUCCESS, ${APP_NAME?upper_case}_SQI_STATE_ERROR - FAILURE) */
				if (${APP_NAME?lower_case}Data.compareStatus == 0)
				{
					${APP_NAME?lower_case}Data.sst26StateMachine = ${APP_NAME?upper_case}_SQI_STATE_DONE;
				}
				else
				{
					${APP_NAME?lower_case}Data.sst26StateMachine = ${APP_NAME?upper_case}_SQI_STATE_ERROR;
				}
			}		

			break;
		}
		
		case ${APP_NAME?upper_case}_SQI_STATE_ERROR:
        
			break;
		
		case ${APP_NAME?upper_case}_SQI_STATE_DONE:
        
			break;
		
    }
}
</#macro>

<#macro macro_drv_sqi_app_c_initialize>
    ${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_SST26_SQI_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval} = DRV_HANDLE_INVALID;
</#macro>

<#macro macro_drv_sqi_app_c_tasks_data>
</#macro>

<#macro macro_drv_sqi_app_c_tasks_state_init>
            if (DRV_HANDLE_INVALID == ${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_SST26_SQI_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval})
            {
                ${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_SST26_SQI_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval} = DRV_SST26_Open(DRV_SST26_INDEX_0, DRV_IO_INTENT_READWRITE);
                appInitialized &= (DRV_HANDLE_INVALID != ${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_SST26_SQI_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval});
            }
</#macro>

<#macro macro_drv_sqi_app_c_tasks_calls_after_init>
				/* Set SST26 event handler */
				DRV_SST26_EventHandlerSet(${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_SST26_SQI_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval}, ${APP_NAME?upper_case}_SST26EventHandler, 0);

                /* Initialize the SST26 state machine */
                ${APP_NAME?lower_case}Data.sst26StateMachine = ${APP_NAME?upper_case}_SQI_STATE_SST26_GET_GEOMETRY;
</#macro>

<#macro macro_drv_sqi_app_c_tasks_state_service_tasks>
            /* Run state machine servicing SST26 */
            SST26_Task();
</#macro>

<#macro macro_drv_sqi_app_c_tasks_states>
</#macro>
