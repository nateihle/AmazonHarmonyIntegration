/*******************************************************************************
  CTR Driver Dynamic implemention.

  Company:
    Microchip Technology Inc.

  File Name:
    drv_ctr.c

  Summary:
    Source code for the CTR driver dynamic implementation.

  Description:
    This file contains the source code for the dynamic implementation of the 
    CTR driver.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute Software
only when embedded on a Microchip microcontroller or digital  signal  controller
that is integrated into your product or third party  product  (pursuant  to  the
sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS  WITHOUT  WARRANTY  OF  ANY  KIND,
EITHER EXPRESS  OR  IMPLIED,  INCLUDING  WITHOUT  LIMITATION,  ANY  WARRANTY  OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A  PARTICULAR  PURPOSE.
IN NO EVENT SHALL MICROCHIP OR  ITS  LICENSORS  BE  LIABLE  OR  OBLIGATED  UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,  BREACH  OF  WARRANTY,  OR
OTHER LEGAL  EQUITABLE  THEORY  ANY  DIRECT  OR  INDIRECT  DAMAGES  OR  EXPENSES
INCLUDING BUT NOT LIMITED TO ANY  INCIDENTAL,  SPECIAL,  INDIRECT,  PUNITIVE  OR
CONSEQUENTIAL DAMAGES, LOST  PROFITS  OR  LOST  DATA,  COST  OF  PROCUREMENT  OF
SUBSTITUTE  GOODS,  TECHNOLOGY,  SERVICES,  OR  ANY  CLAIMS  BY  THIRD   PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE  THEREOF),  OR  OTHER  SIMILAR  COSTS.
*******************************************************************************/
//DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "peripheral/ctr/plib_ctr.h"
#include "driver/ctr/src/drv_ctr_local.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data
// *****************************************************************************
// *****************************************************************************

/* This is the driver instance object array. */
DRV_CTR_OBJ gDrvCTRObj[DRV_CTR_INSTANCES_NUMBER] ;

/* This is the client object array. */
DRV_CTR_CLIENT_OBJ gDrvCTRClientObj[DRV_CTR_CLIENTS_NUMBER];

/* This object maintains data that is required by all CTR
   driver instances. */
DRV_CTR_COMMON_DATA_OBJ gDrvCTRCommonDataObj;

/* TimeStamp Buffer */
uint32_t timeStampBuffer[DRV_CTR_MAX_LATCH_USED * DRV_CTR_LATCH_FIFO_CNT];

// *****************************************************************************
// *****************************************************************************
// Section: CTR SPI Flash Driver Interface Implementations
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    DRV_CTR_CLIENT_OBJ * _DRV_CTR_DriverHandleValidate(DRV_HANDLE handle)

  Summary:
    Dynamic implementation of the _DRV_CTR_DriverHandleValidate() function.

  Description:
    Dynamic implementation of the _DRV_CTR_DriverHandleValidate() function.
    This function returns NULL if the handle is invalid else it return a pointer
    to the CTR Driver Client Object associated with this handle.

  Remarks:
    This is a private function and should not be called directly by an
    application.
*/

DRV_CTR_CLIENT_OBJ * _DRV_CTR_DriverHandleValidate(DRV_HANDLE handle)
{
    /* This function returns the pointer to the client object that is
       associated with this handle if the handle is valid. Returns NULL
       otherwise. */

    DRV_CTR_CLIENT_OBJ * client;

    if((DRV_HANDLE_INVALID == handle) || (NULL == handle))
    {
        return(NULL);
    }

    client = (DRV_CTR_CLIENT_OBJ *)handle;

    if(!client->inUse)
    {
        return(NULL);
    }

    return(client);
}

// *****************************************************************************
/* Function:
    void DRV_CTR_HardwareSetup(DRV_CTR_INIT *ctrInit);

  Summary:
    Function to Configure the CTR hardware before use by clients.

  Description:
    This function configures the CTR counter, latches and trigger as per the
	user configuration data provided in DRV_CTR_INIT.
  
  Remarks:
    See drv_ctr.h for usage information.
*/

bool DRV_CTR_HardwareSetup(DRV_CTR_INIT *ctrInit)
{
	/* Counter Initialization */
	/* The clock divider parameters N and M, Counter increment step LSB and the 
	   Mode for us counter is configured in Counter initialization */
	/* Counter CTR0 Initialization */
	PLIB_CTR_NValueSet(ctrInit->ctrId, CTR0, ctrInit->ctrCounter[0].N);
	PLIB_CTR_NValueSet(ctrInit->ctrId, CTR0, ctrInit->ctrCounter[0].M);
	PLIB_CTR_LSBValueSet(ctrInit->ctrId, CTR0, ctrInit->ctrCounter[0].LSB);
	PLIB_CTR_CTRModeSelect(ctrInit->ctrId, CTR0, ctrInit->ctrCounter[0].Mode);
	
	/* Counter CTR1 Initialization */
	PLIB_CTR_NValueSet(ctrInit->ctrId, CTR1, ctrInit->ctrCounter[1].N);
	PLIB_CTR_NValueSet(ctrInit->ctrId, CTR1, ctrInit->ctrCounter[1].M);
	PLIB_CTR_LSBValueSet(ctrInit->ctrId, CTR1, ctrInit->ctrCounter[1].LSB);
	PLIB_CTR_CTRModeSelect(ctrInit->ctrId, CTR1, ctrInit->ctrCounter[1].Mode);	
	
	/* Latch Initialization */
	/* Configures Latch trigger source, Counter and interrupts*/
	/* For Wifi, Only Latch3 interrupt is enabled as Latch 3 contains the 
	   timestamp for last Wifi event (IEEE 802l.11v). For USB and GPIO, only
	   one latch is used to store the timestamp */
	if(ctrInit->drvMode == WIFI_MODE)
	{
		/* WIFITM1 */
		PLIB_CTR_LatchTriggerSelect(ctrInit->ctrId, CTR_LATCH0, ctrInit->ctrLatch[0].trigSel);
		PLIB_CTR_LatchCTRSelect(ctrInit->ctrId, CTR_LATCH0, ctrInit->ctrLatch[0].ctrSel);
		PLIB_CTR_LatchDivSet(ctrInit->ctrId, CTR_LATCH0, ctrInit->ctrLatch[0].divider);
		/* WIFITM2 */
		PLIB_CTR_LatchTriggerSelect(ctrInit->ctrId, CTR_LATCH1, ctrInit->ctrLatch[1].trigSel);
		PLIB_CTR_LatchCTRSelect(ctrInit->ctrId, CTR_LATCH1, ctrInit->ctrLatch[1].ctrSel);
		PLIB_CTR_LatchDivSet(ctrInit->ctrId, CTR_LATCH1, ctrInit->ctrLatch[1].divider);
		/* WIFITM3 */
		PLIB_CTR_LatchTriggerSelect(ctrInit->ctrId, CTR_LATCH2, ctrInit->ctrLatch[2].trigSel);
		PLIB_CTR_LatchCTRSelect(ctrInit->ctrId, CTR_LATCH2, ctrInit->ctrLatch[2].ctrSel);
		PLIB_CTR_LatchDivSet(ctrInit->ctrId, CTR_LATCH2, ctrInit->ctrLatch[2].divider);
		/* WIFITM4 */
		PLIB_CTR_LatchTriggerSelect(ctrInit->ctrId, CTR_LATCH3, ctrInit->ctrLatch[3].trigSel);
		PLIB_CTR_LatchCTRSelect(ctrInit->ctrId, CTR_LATCH3, ctrInit->ctrLatch[3].ctrSel);
		PLIB_CTR_LatchDivSet(ctrInit->ctrId, CTR_LATCH3, ctrInit->ctrLatch[3].divider);		
		PLIB_CTR_IntModeLatchSelect(ctrInit->ctrId, ctrInit->ctrLatchEventMode, CTR_LATCH3);		
		PLIB_CTR_IntLatchSelect(ctrInit->ctrId, CTR_LATCH3);
	}
	else if(ctrInit->drvMode == USB_MODE)
	{
		/* USB SOF */
		PLIB_CTR_LatchTriggerSelect(ctrInit->ctrId, CTR_LATCH0, ctrInit->ctrLatch[0].trigSel);
		PLIB_CTR_LatchCTRSelect(ctrInit->ctrId, CTR_LATCH0, ctrInit->ctrLatch[0].ctrSel);
		PLIB_CTR_LatchDivSet(ctrInit->ctrId, CTR_LATCH0, ctrInit->ctrLatch[0].divider);		
		PLIB_CTR_IntModeLatchSelect(ctrInit->ctrId, ctrInit->ctrLatchEventMode, CTR_LATCH0);
		PLIB_CTR_IntLatchSelect(ctrInit->ctrId, CTR_LATCH0);
	}
	else if(ctrInit->drvMode == GPIO_MODE)
	{
		/* GPIO */
		PLIB_CTR_LatchTriggerSelect(ctrInit->ctrId, CTR_LATCH0, ctrInit->ctrLatch[0].trigSel);
		PLIB_CTR_LatchCTRSelect(ctrInit->ctrId, CTR_LATCH0, ctrInit->ctrLatch[0].ctrSel);
		PLIB_CTR_LatchDivSet(ctrInit->ctrId, CTR_LATCH0, ctrInit->ctrLatch[0].divider);	
		PLIB_CTR_IntModeLatchSelect(ctrInit->ctrId, ctrInit->ctrLatchEventMode, CTR_LATCH0);		
		PLIB_CTR_IntLatchSelect(ctrInit->ctrId, CTR_LATCH0);
	}
	else
	{
        SYS_DEBUG(0, "DRV_CTR: Invalid Mode \n");
        return FALSE;  		
	}
	
	/* Trigger Initialization */
	/* Trigger counter and the phase are configured in trigger initialization */
	PLIB_CTR_TriggerSelect(ctrInit->ctrId, ctrInit->ctrTrigger.trigSource);
	PLIB_CTR_CycleOffsetValueSet(ctrInit->ctrId, ctrInit->ctrTrigger.phase);
	return TRUE;
}

// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_CTR_Initialize
    (
        const SYS_MODULE_INDEX index,
        const SYS_MODULE_INIT * const init
    );

  Summary:
    Dynamic impementation of DRV_CTR_Initialize system interface function.

  Description:
    This is the dynamic impementation of DRV_CTR_Initialize system
    interface function.
  
  Remarks:
    See drv_ctr.h for usage information.
*/

SYS_MODULE_OBJ DRV_CTR_Initialize
(
    const SYS_MODULE_INDEX drvIndex,
    const SYS_MODULE_INIT * const init
)
{
    DRV_CTR_OBJ *dObj = (DRV_CTR_OBJ*)NULL;
    DRV_CTR_INIT *ctrInit = NULL ;

    /* Check if the specified driver index is in valid range */
    if(drvIndex >= DRV_CTR_INSTANCES_NUMBER)
    {
        SYS_DEBUG(0, "DRV_CTR: Invalid driver index \n");
        return SYS_MODULE_OBJ_INVALID;
    }

    /* Check if this hardware instance was already initialized */
    if(gDrvCTRObj[drvIndex].inUse == true)
    {
        SYS_DEBUG(0, "DRV_CTR: Instance already in use \n");
        return SYS_MODULE_OBJ_INVALID;
    }
    
    /* Assign to the local pointer the init data passed */
    ctrInit = ( DRV_CTR_INIT * ) init ;

    /* Allocate the driver object and set the operation flag to be in use */
    dObj = &gDrvCTRObj[drvIndex];
    dObj->inUse = true;
    
    /* Set the driver status as busy */
    dObj->status =  SYS_STATUS_BUSY;

	dobj->ctrModuleIndex = drvIndex;
	dobj->ctrId = ctrInit->ctrId;
    dObj->nClients = 0;
    dObj->isExclusive = false;
	dobj->drvMode = ctrInit->drvMode;
	
	/* Initialize the interrupt Sources */
	dobj->ctrEventInterruptSource = ctrInit->ctrEventInterruptSource;
	dbj->ctrTriggerInterruptSource = ctrInit->ctrTriggerInterruptSource;
    dobj->ctrLatchEventMode = ctrInit->ctrLatchEventMode;
	
	dobj->clientObj = &gDrvCTRClientObj[0];
	
    dObj->latchCallBackOccupancy = 0;
   
	if(DRV_CTR_HardwareSetup(ctrInit) == false)
	{
        SYS_DEBUG(0, "DRV_CTR: Hardware Init Failed \n");
		dObj->inUse = false;
        return SYS_MODULE_OBJ_INVALID;		
	}
	
	if(ctrDrvInstance->drvMode == WIFI_MODE)
		dObj->latMax = 4;
	else
		dObj->latMax = 1;

	if(ctrDrvInstance->ctrLatchEventMode == CTR_BUFFER_FULL)
		bufMax = DRV_CTR_LATCH_FIFO_CNT;
	else if(ctrDrvInstance->ctrLatchEventMode == CTR_BUFFER_HALF)
		bufMax = DRV_CTR_LATCH_FIFO_CNT / 2;
	else
		bufMax = 1;
	
    /* Create the hardware instance mutex. */
     OSAL_ASSERT((OSAL_MUTEX_Create(&(dObj->mutexDriverInstance)) == OSAL_RESULT_TRUE),
                 "Unable to create hardware instance mutex");


    /* Check if the global mutexes have been created. If not
       then create these. */

     if(!gDrvCTRCommonDataObj.membersAreInitialized)
     {
         /* This means that mutexes where not created. Create them. */
         OSAL_ASSERT((OSAL_MUTEX_Create(&(gDrvCTRCommonDataObj.mutexClientObjects)) == OSAL_RESULT_TRUE),
                     "Unable to create client instance mutex");

   	     /* Set this flag so that global mutexes get allocated only once */
         gDrvCTRCommonDataObj.membersAreInitialized = true;
     }

    /* Set the driver status as busy */
    dObj->status =  SYS_STATUS_READY;

    /* Driver status will be made ready once SPI Driver is openend
       successfully and Write Protection is disabled in Task API */

    return drvIndex;
}


//******************************************************************************
/* Function:
    void DRV_CTR_Deinitialize( SYS_MODULE_OBJ object )

  Summary:
    Deinitializes the specified instance of the CTR driver

  Description:
    Deinitializes the specified instance of the CTR driver,
    disabling its operation (and any hardware). Invalidates all the
    internal data.

  Remarks:
    See drv_ctr.h for usage information.
*/

void DRV_CTR_Deinitialize( SYS_MODULE_OBJ object)
{
    DRV_CTR_OBJ * dObj;

    /* Check that the object is valid */
    if(object == SYS_MODULE_OBJ_INVALID)
    {
        SYS_DEBUG(0, "DRV_CTR: Invalid system module object \n" );
        return;
    }
    
    if(object >= DRV_CTR_INSTANCES_NUMBER)
    {
        SYS_DEBUG(0, "DRV_CTR: Invalid system module object \n" );
        return;
    }

    dObj = (DRV_CTR_OBJ*) &gDrvCTRObj[object];

    if(dobj->nClients > 0)
    {
        SYS_DEBUG(0, "DRV_CTR: Still the driver has clients \n");
        return;        
    }
    if(!dObj->inUse)
    {
        SYS_DEBUG(0, "DRV_CTR: Invalid system module object \n");
        return;
    }

    /* The driver will not have clients when it is
       being de-initialized. So the order in which
       we do the following steps is not that important */

    /* Indicate that this object is not is use */
    dObj->inUse = false;

    /* Deinitialize the CTR status */
    dObj->status =  SYS_STATUS_UNINITIALIZED ;


    /* Deallocate all mutexes */
    OSAL_ASSERT( (OSAL_MUTEX_Delete(&(dObj->mutexDriverInstance)) == OSAL_RESULT_TRUE),
             "Unable to delete client handle mutex" );
}

//*************************************************************************
/* Function:
    SYS_STATUS DRV_CTR_Status( SYS_MODULE_OBJ object )

  Summary:
    Gets the current status of the CTR driver.

  Description:
    This routine provides the current status of the CTR driver.
  
  Remarks:
    See drv_ctr.h for usage information.
*/

SYS_STATUS DRV_CTR_Status( SYS_MODULE_OBJ object)
{
    /* Check if we have a valid object */
    if(object == SYS_MODULE_OBJ_INVALID)
    {
        SYS_DEBUG(0, "DRV_CTR: Invalid system object handle \n");
        return(SYS_STATUS_UNINITIALIZED);
    }
    
    if(object > DRV_CTR_INSTANCES_NUMBER)
    {
        SYS_DEBUG(0, "DRV_CTR: Invalid system object handle \n");
        return(SYS_STATUS_UNINITIALIZED);
    }

    /* Return the system status of the hardware instance object */
    return (gDrvCTRObj[object].status);
}

// *****************************************************************************
/* Function:
    DRV_HANDLE DRV_CTR_Open
    (
        const SYS_MODULE_INDEX drvIndex,
        const DRV_IO_INTENT ioIntent
    )

  Summary:
    Opens the specified CTR driver instance and returns a handle to it

  Description:
    This routine opens the specified CTR driver instance and provides a
    handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver.

  Remarks:
    See drv_ctr.h for usage information.
*/

DRV_HANDLE DRV_CTR_Open
( 
    const SYS_MODULE_INDEX drvIndex,
    const DRV_IO_INTENT ioIntent
)
{
    DRV_CTR_CLIENT_OBJ *clientObj;
    DRV_CTR_OBJ *dObj;
    unsigned int iClient;

    if (drvIndex >= DRV_CTR_INSTANCES_NUMBER)
    {
        /* Invalid driver index */
        SYS_DEBUG(0, "DRV_CTR: Invalid Driver Instance \n");
        return (DRV_HANDLE_INVALID);
    }

    dObj = &gDrvCTRObj[drvIndex];
    
    if((dObj->status != SYS_STATUS_READY) || (dObj->inUse == false)) 
    {
        /* The CTR module should be ready */

        SYS_DEBUG(0, "DRV_CTR: Was the driver initialized? \n");
        return DRV_HANDLE_INVALID;
    }

    if(dObj->isExclusive)
    {
        /* This means the another client has opened the driver in exclusive
           mode. The driver cannot be opened again */

        SYS_DEBUG(0, "DRV_CTR: Driver already opened exclusively \n"); 
        return ( DRV_HANDLE_INVALID ) ;
    }

    if((dObj->nClients > 0) && (ioIntent & DRV_IO_INTENT_EXCLUSIVE))
    {
        /* This means the driver was already opened and another driver was 
           trying to open it exclusively.  We cannot give exclusive access in 
           this case */

        SYS_DEBUG(0, "DRV_CTR: Driver already opened. Cannot be opened exclusively \n");
        return(DRV_HANDLE_INVALID);
    }

    /* Grab client object mutex here */

    if(OSAL_MUTEX_Lock(&(gDrvCTRCommonDataObj.mutexClientObjects), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
    {
        /* Enter here only if the lock was obtained (appplicable in 
           RTOS only). If the mutex lock fails due to time out then
           this code does not get executed */

        for(iClient = 0; iClient <= DRV_CTR_CLIENTS_NUMBER; iClient ++)
        {
            if(!gDrvCTRClientObj[iClient].inUse)
            {
                /* This means we have a free client object to use */
                clientObj = &gDrvCTRClientObj[iClient];
                clientObj->inUse        = true;
                
                /* We have found a client object. Release the mutex */

                OSAL_ASSERT(OSAL_MUTEX_Unlock(&(gDrvCTRCommonDataObj.mutexClientObjects)),
                        "Unable to unlock clients objects routine mutex");
                
                clientObj->hDriver      = dObj;                
                clientObj->ioIntent     = ioIntent;
                clientObj->timeStampCallback = (DRV_CTR_CALLBACK) NULL;
                clientObj->context      = (uintptr_t) NULL;
				clientObj->oneTimeCallback = false;
				
                if(ioIntent & DRV_IO_INTENT_EXCLUSIVE)
                {
                    /* Set the driver exclusive flag */
                    dObj->isExclusive = true;
                }

                dObj->nClients ++;

                // /* Create the semaphores */
                // OSAL_ASSERT(((OSAL_SEM_Create(&(clientObj->semReadDone), OSAL_SEM_TYPE_COUNTING, 1, 0)) == OSAL_RESULT_TRUE),
                        // "Unable to create client read done semaphore");
                // OSAL_ASSERT(((OSAL_SEM_Create(&(clientObj->semWriteDone), OSAL_SEM_TYPE_COUNTING, 1, 0)) == OSAL_RESULT_TRUE),
                        // "Unable to create client write done semaphore");

                /* Update the client status */
                clientObj->clientStatus = DRV_CTR_CLIENT_STATUS_READY;
                return ((DRV_HANDLE) clientObj );
            }
        }

        /* Could not find a client object. Release the mutex and 
           return with an invalid handle. */
        OSAL_ASSERT((OSAL_MUTEX_Unlock(&(gDrvCTRCommonDataObj.mutexClientObjects))),
                    "Unable to unlock clients objects routine mutex");
    }

    /* If we have reached here, it means either we could not find a spare
       client object or the mutex timed out in a RTOS environment. */
    
    return DRV_HANDLE_INVALID;
}

// *****************************************************************************
/* Function:
    void DRV_CTR_Close( DRV_Handle handle );

  Summary:
    Closes an opened-instance of the SPI Flash driver

  Description:
    This routine closes an opened-instance of the SPI Flash driver, invalidating
    the handle.

  Remarks:
    See drv_ctr.h for usage information.
*/

void DRV_CTR_Close( const DRV_HANDLE handle)
{
    /* This function closes the client, The client
       object is de-allocated and returned to the
       pool. */

    DRV_CTR_CLIENT_OBJ  * clientObj;
    DRV_CTR_OBJ * dObj;

    /* Validate the handle */
    clientObj = _DRV_CTR_DriverHandleValidate(handle);

    if(NULL == clientObj)
    {
        /* Driver handle is not valid */
        SYS_DEBUG(0, "DRV_CTR: Invalid Driver Handle \n");
        return;
    }

    dObj = (DRV_CTR_OBJ *)clientObj->hDriver;

    if(dObj->isExclusive)
    {
        /* clear the driver exclusive flag if it was set */
        dObj->isExclusive = false;
    }

    /* Reduce the number of clients */
    dObj->nClients --;

    /* De-allocate the object */
    clientObj->clientStatus = DRV_CTR_CLIENT_STATUS_CLOSED;
    clientObj->inUse = false;

    return;
}
 
// *****************************************************************************
/* Function:
    DRV_CTR_CLIENT_STATUS DRV_CTR_ClientStatus (DRV_HANDLE handle )

  Summary:
    Dynamic impementation of DRV_CTR_ClientStatus client interface function.

  Description:
    This is the dynamic impementation of DRV_CTR_ClientStatus client interface 
    function.
  
  Remarks:
    See drv_CTR.h for usage information.
*/

DRV_CTR_CLIENT_STATUS DRV_CTR_ClientStatus(DRV_HANDLE handle)
{
    DRV_CTR_CLIENT_OBJ * client;

    /* Validate the driver handle */
    client = _DRV_CTR_DriverHandleValidate(handle);

    if(NULL == client)
    {
        /* Driver handle is not valid */
        
        SYS_DEBUG(0, "DRV_CTR: Invalid driver handle \n");
        return DRV_CTR_CLIENT_STATUS_CLOSED;
    }

    /* Return the client status */
    return(client->clientStatus);
}

// *****************************************************************************
/* Function:
    void DRV_CTR_RegisterCallBack
	(
		const DRV_HANDLE handle,
		const DRV_CTR_CALLBACK callback,
		const bool oneTimeCallback,
		const uintptr_t context
	)

  Summary:
    Allows a client to register a callback function once the CTR event is 
	triggered.

  Description:
    This function allows a client to register a callback which is called when
	the CTR event is triggered for a given latch as per the interrupt mode 
	configured. 

  Remarks:
    See drv_ctr.h for usage information.
*/

void DRV_CTR_RegisterCallBack
(
    const DRV_HANDLE handle,
    const DRV_CTR_CALLBACK callback,
	const bool oneTimeCallback,
    const uintptr_t context
)
{
    DRV_CTR_CLIENT_OBJ *client;

    /* Validate the driver handle */
    client = _DRV_CTR_DriverHandleValidate(handle);

    if(NULL == client)
    {
        /* Driver handle is not valid */

        SYS_DEBUG(0, "DRV_CTR: Invalid driver handle \n");
        return;
    }

    /* save the required data in the Client Object */
    client->context = context;
    client->oneTimeCallback = oneTimeCallback;
	client->timeStampCallback = callback;
}

// *****************************************************************************
/* Function:
    void _DRV_CTR_ModuleObj(SYS_MODULE_OBJ object)

  Summary:
    This function is used to validate the driver instance index and get the 
	pointer for the same.

  Description:
    This function is used to validate the driver instance index and get the 
	pointer for the same. 

  Remarks:
    
*/

static DRV_CTR_OBJ * _DRV_CTR_ModuleObj(SYS_MODULE_OBJ object)
{
	DRV_CTR_OBJ * pCtrInstance;
	
	if(object >= DRV_CTR_INSTANCES_NUMBER)
		return NULL;
	
	pCtrInstance = &gDrvCTRObj[object];
	
	if(!pCtrInstance->inUse)
		return NULL;
	
	return pCtrInstance;
	
}

// *****************************************************************************
/* Function:
    void DRV_CTR_EventISR(SYS_MODULE_OBJ object)

  Summary:
    Interrupt task routine for CTR Event interrupt.
	
  Description:
    This function is an interrupt service routince for CTR event interrupt. This
	function is called from the interrupt vector location.

  Remarks:
    See drv_ctr.h for usage information.
*/

void DRV_CTR_EventISR(SYS_MODULE_OBJ object)
{
	DRV_CTR_OBJ * ctrDrvInstance = _DRV_CTR_ModuleObj(object);
	DRV_CTR_CLIENT_OBJ * ctrClient = ctrDrvInstance->clientObj;
	CTR_LATCH_UNIT_SELECT latNum[] = {CTR_LATCH0, CTR_LATCH1, CTR_LATCH2, CTR_LATCH3};
	uint8_t buffSize = 0;
	uint8_t latIdx, bufIdx, clientIdx;
	
	for(latIdx = 0; latIdx < ctrDrvInstance->latMax; latIdx++)
	{
		for(bufIdx = 0; bufIdx < ctrDrvInstance->bufMax; bufIdx++)
		{
			timeStampBuffer[(latIdx * bufMax) + bufIdx] = PLIB_CTR_LatchGetValue(ctrDrvInstance->ctrId, latNum[latIdx]);
			buffSize++;
		}
	}
	
    PLIB_INT_SourceFlagClear(INT_ID_0, ctrDrvInstance->ctrEventInterruptSource);

	for(clientIdx = 0; clientIdx < DRV_CTR_CLIENTS_NUMBER; clientIdx)
	{
		if(ctrClient[clientIdx].inUse)
		{
			if(ctrClient[clientIdx].timeStampCallback)
			{
				ctrClient[clientIdx].timeStampCallback(ctrClient[clientIdx].context, timeStampBuffer, buffSize); 
				if(ctrClient[clientIdx].oneTimeCallback)
					ctrClient[clientIdx].timeStampCallback = NULL;
			}
		}
	}
	
}

// *****************************************************************************
/* Function:
    void DRV_CTR_TriggerISR(SYS_MODULE_OBJ object)

  Summary:
    Interrupt task routine for CTR Trigger interrupt.
	
  Description:
    This function is an interrupt service routince for CTR trigger interrupt. This
	function is called from the interrupt vector location.

  Remarks:
    See drv_ctr.h for usage information.
*/

void DRV_CTR_TriggerISR(SYS_MODULE_OBJ object)
{
	DRV_CTR_OBJ * ctrDrvInstance = _DRV_CTR_ModuleObj(object);
	
	/* TO DO : Add trigger interrupt handling code here */
	
	PLIB_INT_SourceFlagClear(INT_ID_0, ctrDrvInstance->ctrTriggerInterruptSource);
}

// *****************************************************************************
/* Function:
	void DRV_CTR_Adjust(DRV_HANDLE handle, CTR_LATCH_CTR_SELECT ctrSel, uint16_t adjustVal)

  Summary:
    Sets the adjust value for a counter selected.
	
  Description:
    This function sets the adjust value for a counter selected by ctrSel. The 
	enumeration CTR_LATCH_CTR_SELECT itself specifies  which counter in a CTR to 
	be used for adjustment.

  Remarks:
    See drv_ctr.h for usage information.
*/

void DRV_CTR_Adjust(DRV_HANDLE handle, CTR_LATCH_CTR_SELECT ctrSel, uint16_t adjustVal)
{
    DRV_CTR_CLIENT_OBJ *client = NULL;
	DRV_CTR_OBJ * ctrDrvInstance = NULL;
	
    /* Validate the driver handle */
    client = _DRV_CTR_DriverHandleValidate(handle);
	if(NULL == client)
	{
        /* Driver handle is not valid */
        SYS_DEBUG(0, "DRV_CTR: Invalid driver handle \n");
        return;		
	}
	ctrDrvInstance = client->hDriver;
	
	if(NULL == ctrDrvInstance)
	{
        /* Driver handle is not valid */
        SYS_DEBUG(0, "DRV_CTR: Invalid driver Instance \n");
        return;				
	}
	
	if(ctrSel == CTR_CTR0_US)
	{
		PLIB_CTR_CTRAdjustValueInitialize(ctrDrvInstance->ctrID, CTR0, adjustVal);
	}
	else if(ctrSel == CTR_CTR1_US)
	{
		PLIB_CTR_CTRAdjustValueInitialize(ctrDrvInstance->ctrID, CTR1, adjustVal);		
	}
	else if(CTR_CTR0_LIN)
	{
		PLIB_CTR_CTRLinearAdjustInitialize(ctrDrvInstance->ctrID, CTR0, adjustVal);
	}
	else if(CTR_CTR1_LIN)
	{
		PLIB_CTR_CTRLinearAdjustInitialize(ctrDrvInstance->ctrID, CTR1, adjustVal);
	}
	else
	{
		SYS_DEBUG(0, "DRV_CTR: Invalid Counter selection \n");
        return;				
	}
}


// *****************************************************************************
/* Function:
	void DRV_CTR_Drift(DRV_HANDLE handle, CTR_LATCH_CTR_SELECT ctrSel, uint32_t driftVal)

  Summary:
    Sets the drift value for a counter selected.
	
  Description:
    This function sets the drift value for a counter selected by ctrSel. The 
	enumeration CTR_LATCH_CTR_SELECT itself specifies  which counter in a CTR to 
	be used for adjustment.

  Remarks:
    See drv_ctr.h for usage information.
*/

void DRV_CTR_Drift(DRV_HANDLE handle, CTR_LATCH_CTR_SELECT ctrSel, uint32_t driftVal)
{
    DRV_CTR_CLIENT_OBJ *client = NULL;
	DRV_CTR_OBJ * ctrDrvInstance = NULL;
	
    /* Validate the driver handle */
    client = _DRV_CTR_DriverHandleValidate(handle);
	if(NULL == client)
	{
        /* Driver handle is not valid */
        SYS_DEBUG(0, "DRV_CTR: Invalid driver handle \n");
        return;		
	}
	ctrDrvInstance = client->hDriver;
	
	if(NULL == ctrDrvInstance)
	{
        /* Driver handle is not valid */
        SYS_DEBUG(0, "DRV_CTR: Invalid driver Instance \n");
        return;				
	}
	
	if(ctrSel == CTR_CTR0_US)
	{
		PLIB_CTR_CTRDriftValueSet(ctrDrvInstance->ctrID, CTR0, driftVal);
	}
	else if(ctrSel == CTR_CTR1_US)
	{
		PLIB_CTR_CTRDriftValueSet(ctrDrvInstance->ctrID, CTR1, driftVal);		
	}
	else if(CTR_CTR0_LIN)
	{
		PLIB_CTR_CTRLinearDriftSet(ctrDrvInstance->ctrID, CTR0, driftVal);
	}
	else if(CTR_CTR1_LIN)
	{
		PLIB_CTR_CTRLinearDriftSet(ctrDrvInstance->ctrID, CTR1, driftVal);
	}
	else
	{
		SYS_DEBUG(0, "DRV_CTR: Invalid Counter selection \n");
        return;				
	}
}

/*******************************************************************************
 End of File
*/
