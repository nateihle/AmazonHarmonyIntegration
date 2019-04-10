<#macro macro_drv_sqi_app_h_includes>

#define SST26_BLOCK_START_ADDRESS		${("CONFIG_APP_DRV_SST26_SQI_START_ADDRESS" + "${HCONFIG_APP_INSTANCE}")?eval}
#define XFER_SIZE						${("CONFIG_APP_DRV_SST26_SQI_XFER_SIZE" + "${HCONFIG_APP_INSTANCE}")?eval}

/* SQI Application States */
typedef enum
{
	${APP_NAME?upper_case}_SQI_STATE_SST26_GET_GEOMETRY,
    ${APP_NAME?upper_case}_SQI_STATE_SST26_ERASE,
	${APP_NAME?upper_case}_SQI_STATE_SST26_ERASE_DONE,
	${APP_NAME?upper_case}_SQI_STATE_SST26_WRITE,
	${APP_NAME?upper_case}_SQI_STATE_SST26_WRITE_DONE,	
	${APP_NAME?upper_case}_SQI_STATE_SST26_READ,
	${APP_NAME?upper_case}_SQI_STATE_SST26_READ_DONE,	
	${APP_NAME?upper_case}_SQI_STATE_ERROR,	
	${APP_NAME?upper_case}_SQI_STATE_DONE	
} ${APP_NAME?upper_case}_SQI_STATES;
</#macro>

<#macro macro_drv_sqi_app_h_states>
</#macro>

<#macro macro_drv_sqi_app_h_data>

    ${APP_NAME?upper_case}_SQI_STATES sst26StateMachine;

    /* SQI Driver Handle  */
    DRV_HANDLE ${("CONFIG_APP_DRV_SST26_SQI_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval};
	
	/* SQI Command Handle */
	DRV_SST26_COMMAND_HANDLE ${("CONFIG_APP_DRV_SST26_SQI_CMD_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval};

	/* Erase Block Size */
	uint32_t eraseStartBlock;

	/* Write Block Size */
	uint32_t writeStartBlock;

	/* Read Block Size */
	uint32_t readStartBlock;
	
	/* Erase Block Size */
	uint32_t numEraseBlocks;

	/* Write Block Size */
	uint32_t numWriteBlocks;

	/* Read Block Size */
	uint32_t numReadBlocks;
	
	/* Verification Status Flag */
	uint32_t compareStatus;
	
	/* Erase Command Status */
	bool operationComplete;
</#macro>

<#macro macro_drv_sqi_app_h_callback_function_declarations>
</#macro>

<#macro macro_drv_sqi_app_h_function_declarations>
</#macro>
