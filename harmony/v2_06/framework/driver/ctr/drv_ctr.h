/*******************************************************************************
  CTR Driver Interface Definition

  Company:
    Microchip Technology Inc.

  File Name:
    drv_CTR.h

  Summary:
    CTR Driver Interface Definition

  Description:
    The CTR device driver provides a simple interface to manage the CTR Module
    This file defines the interface definition for the CTR Driver.
******************************************************************************/

//DOM-IGNORE-BEGIN
/******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
******************************************************************************/
//DOM-IGNORE-END

#ifndef _DRV_CTR_H
#define _DRV_CTR_H


// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************
/* Note:  A file that maps the interface definitions above to appropriate static
          implementations (depending on build mode) is included at the end of
          this file.
*/

#include "driver/driver_common.h"
#include "system/system.h"
#include "system/int/sys_int.h"
#include "osal/osal.h"



// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Driver CTR Module Index reference

  Summary:
    CTR driver index definitions

  Description:
    These constants provide CTR driver index definitions.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.

    These values should be passed into the DRV_CTR_Initialize and
    DRV_CTR_Open routines to identify the driver instance in use.
*/

#define      DRV_CTR_INDEX_0      0

// *****************************************************************************
/* Counters present in the CTR module

  Summary:
    Number of counters in CTR module

  Description:
    These constants provide Number of counters in CTR module.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.

*/

#define      DRV_CTR_COUNTER_NUM  2

// *****************************************************************************
/* Latches present in the CTR module

  Summary:
    Number of latches in CTR module

  Description:
    These constants provide Number of latches in CTR module.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.

*/

#define      DRV_CTR_LATCH_NUM    6

// *****************************************************************************
/* FIFO size for each latch in the CTR module

  Summary:
    FIFO size for each latch in CTR module

  Description:
    These constants provide Number of FIFO location available in each latch 
	in CTR module.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.
*/

#define      DRV_CTR_LATCH_FIFO_CNT  4

// *****************************************************************************
/* CTR Client Status

  Summary:
    Defines the client status.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    Defines the various client status codes.

  Remarks:
    None.
*/

typedef enum
{
    /* Up and running, ready to start new operations */
    DRV_CTR_CLIENT_STATUS_READY = DRV_CLIENT_STATUS_READY + 0,

    /* Operation in progress, unable to start a new one */
    DRV_CTR_CLIENT_STATUS_BUSY = DRV_CLIENT_STATUS_BUSY,

    /* Client is closed */
    DRV_CTR_CLIENT_STATUS_CLOSED = DRV_CLIENT_STATUS_CLOSED,

    /* Client Error */
    DRV_CTR_CLIENT_STATUS_ERROR = DRV_CLIENT_STATUS_ERROR

} DRV_CTR_CLIENT_STATUS;

// *****************************************************************************
/* CTR Driver mode

  Summary:
    Defines the driver mode.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    Driver can be configured to use for either of Wifi, USB or GPIO.

  Remarks:
    None.
*/

typedef enum
{
	WIFI_MODE = 0,
	USB_MODE,
	GPIO_MODE
}DRV_MODE;

// *****************************************************************************
/* CTR Counter init structure

  Summary:
    Contains all the data necessary to initialize the CTR counter.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This structure contains all of the data necessary to initialize the CTR 
	counter increment steps and the resolution.

  Remarks:
    This structure is a part of initialization structure, which is used to 
	initialize the CTR module.
*/

typedef struct
{
	uint32_t M;
	uint32_t N;
	uint8_t LSB;
	CTR_MODE_SELECT Mode;
}DRV_CTR_COUNTER;

// *****************************************************************************
/* CTR Latch init structure

  Summary:
    Contains all the data necessary to initialize the CTR Latches.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This structure contains all of the data necessary to initialize the CTR 
	Latches for mapping the trigger source and counter for a given latch.
  Remarks:
    This structure is a part of initialization structure, which is used to 
	initialize the CTR module.
*/

typedef struct
{
	CTR_LATCH_TRIGGER_SELECT trigSel;
	CTR_LATCH_CTR_SELECT ctrSel;
	uint8_t divider;
}DRV_CTR_LATCH;

// *****************************************************************************
/* CTR Trigger init structure

  Summary:
    Contains all the data necessary to initialize the CTR Triggers.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This structure contains all of the data necessary to initialize the CTR 
	Triggers for generating triggers from CTR.
  Remarks:
    This structure is a part of initialization structure, which is used to 
	initialize the CTR module.
*/

typedef struct 
{
	CTR_LATCH_CTR_SELECT trigSource;
	uint16_t phase;
}DRV_CTR_TRIGGER;

// *****************************************************************************
/* CTR Event interrupt callback function

  Summary:
    Callback function definition for CTR event interrupt.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
	The clients must define their callback functions in the same prototype as 
	DRV_CTR_CALLBACK. All the registered callbacks will be called from drive ISR
	for CTR event.
	
  Remarks:
    This structure is a part of initialization structure, which is used to 
	initialize the CTR module.
*/

typedef void ( *DRV_CTR_CALLBACK ) ( uintptr_t context, uint32_t * timestampbuffer, uint8_t BufferSize );

// *****************************************************************************
/* CTR Driver Initialization Data

  Summary:
    Contains all the data necessary to initialize the CTR.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This structure contains all of the data necessary to initialize the CTR.

  Remarks:
    A pointer to a structure of this format containing the desired
    initialization data must be passed into the DRV_CTR_Initialize
    function.
*/

typedef struct
{
    /* System module initialization */
    SYS_MODULE_INIT                     moduleInit;
	
	/* Identifies the CTR peripheral instance */
	CTR_MODULE_ID                        ctrId;
	
	/* CTR Event Interrupt Source */ 
	INT_SOURCE                           ctrEventInterruptSource;
	
	/* CTR Event Interrupt Mode */
	CTR_LATCH_INT_MODE ctrLatchEventMode;
	
	/* CTR Triggetr Interrupt Source */
	INT_SOURCE                            ctrTriggerInterruptSource;
	
	/* Counter Init Data */
	DRV_CTR_COUNTER ctrCounter[DRV_CTR_COUNTER_NUM];

	/* Latch Init Data */
	DRV_CTR_LATCH ctrLatch[DRV_CTR_LATCH_NUM];
	
	DRV_CTR_TRIGGER ctrTrigger;
	
	/* Driver Mode */
	DRV_MODE drvMode;
} DRV_CTR_INIT;

// *****************************************************************************
// *****************************************************************************
// Section: CTR Driver Module Interface Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_CTR_Initialize
    (
        const SYS_MODULE_INDEX index,
        const SYS_MODULE_INIT * const init
    );

  Summary:
    Initializes the CTR Driver instance for the specified driver index.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function initializes the CTR driver instance for the specified
    driver index, making it ready for clients to open and use it.

  Precondition:
    None.

  Parameters :
    index -  Identifier for the instance to be initialized

    init -   Pointer to a data structure containing data necessary to
             initialize the driver.

  Returns:
    If successful, returns a valid handle to a driver instance object.
    Otherwise, it returns SYS_MODULE_OBJ_INVALID.

  Example:
    <code>
    // This code snippet shows an example of initializing the CTR Driver. All 
	the CTR initialization is done in #defines mentioned, and the init structure 
	is initialized with corresponding #defines and then passed to initialize 
	function.

// *****************************************************************************
// CTR Driver Configuration Options

	#define DRV_CTR_POWER_STATE              SYS_MODULE_POWER_RUN_FULL
	#define DRV_CTR_MODULE_ID                CTR_ID_0 
	#define DRV_CTR_CLIENTS_NUMBER			 1
	#define DRV_CTR_INSTANCES_NUMBER		 1
	#define DRV_CTR_EVENT_INTERRUPT_SOURCE   INT_SOURCE_CTR1_EVENT
	#define DRV_CTR_EVENT_INTERRUPT_MODE     CTR_LATCH_TRIG
	#define DRV_CTR_TRIGGER_INTERRUPT_SOURCE INT_SOURCE_CTR1_TRG
	#define DRV_CTR_M_0						 0x000000
	#define DRV_CTR_N_0                      0x000000
	#define DRV_CTR_LSB_0					 0x00
	#define DRV_CTR_MODE_0					 CTR_US
	#define DRV_CTR_M_1						 0x000000
	#define DRV_CTR_N_1                      0x000000
	#define DRV_CTR_LSB_1					 0x00
	#define DRV_CTR_MODE_1					 CTR_US
	#define DRV_CTR_COUNTER_SEL				 CTR_CTR0_LIN
	#define DRV_CTR_DIVIDER					 0
	#define DRIVER_MODE						 WIFI_MODE
	#define DRV_CTR_LATCH0_TRIG              CTR_WIFI_TM_1
	#define DRV_CTR_LATCH1_TRIG              CTR_WIFI_TM_2
	#define DRV_CTR_LATCH2_TRIG              CTR_WIFI_TM_3
	#define DRV_CTR_LATCH3_TRIG              CTR_WIFI_TM_4
	#define DRV_CTR_TRIGGER_SOURCE				 CTR_CTR0_LIN
	#define DRV_CTR_TRIGGER_PHASE				 0x000		
	
    DRV_CTR_INIT   CTRInitData;
    SYS_MODULE_OBJ      objectHandle;

    CTRInitData.moduleInit = DRV_CTR_POWER_STATE,
    CTRInitData.ctrEventInterruptSource = DRV_CTR_EVENT_INTERRUPT_SOURCE, 
    CTRInitData.ctrLatchEventMode = DRV_CTR_EVENT_INTERRUPT_MODE, 
    CTRInitData.ctrTriggerInterruptSource = DRV_CTR_TRIGGER_INTERRUPT_SOURCE, 
    CTRInitData.ctrCounter[0].M = DRV_CTR_M_0, 
    CTRInitData.ctrCounter[0].N = DRV_CTR_N_0, 
    CTRInitData.ctrCounter[0].LSB = DRV_CTR_LSB_0, 
    CTRInitData.ctrCounter[1].M = DRV_CTR_M_1, 
    CTRInitData.ctrCounter[1].N = DRV_CTR_N_1, 
    CTRInitData.ctrCounter[1].LSB = DRV_CTR_LSB_1, 
	CTRInitData.ctrLatch[0].ctrSel = DRV_CTR_COUNTER_SEL,
	CTRInitData.ctrLatch[1].ctrSel = DRV_CTR_COUNTER_SEL,
	CTRInitData.ctrLatch[2].ctrSel = DRV_CTR_COUNTER_SEL,
	CTRInitData.ctrLatch[3].ctrSel = DRV_CTR_COUNTER_SEL,
	CTRInitData.ctrLatch[0].trigSel = DRV_CTR_LATCH0_TRIG,
	CTRInitData.ctrLatch[1].trigSel = DRV_CTR_LATCH1_TRIG,
	CTRInitData.ctrLatch[2].trigSel = DRV_CTR_LATCH2_TRIG,
	CTRInitData.ctrLatch[3].trigSel = DRV_CTR_LATCH3_TRIG,
	CTRInitData.ctrLatch[0].divider = DRV_CTR_DIVIDER,
	CTRInitData.ctrLatch[1].divider = DRV_CTR_DIVIDER,
	CTRInitData.ctrLatch[2].divider = DRV_CTR_DIVIDER,
	CTRInitData.ctrLatch[3].divider = DRV_CTR_DIVIDER,
	CTRInitData.ctrTrigger.trigSource = DRV_CTR_TRIGGER_SOURCE,
	CTRInitData.ctrTrigger.phase = DRV_CTR_TRIGGER_PHASE,
	CTRInitData.drvMode = DRIVER_MODE
	
    objectHandle = DRV_CTR_Initialize(DRV_CTR_INDEX_0,
                                    (SYS_MODULE_INIT*)CTRInitData);
    if (SYS_MODULE_OBJ_INVALID == objectHandle)
    {
        // Handle error
    }
    </code>

  Remarks:
    This function must be called before any other CTR function is called.

    This function should only be called once during system initialization
    unless DRV_CTR_Deinitialize is called to deinitialize the driver
    instance.
*/

SYS_MODULE_OBJ DRV_CTR_Initialize
(
    const SYS_MODULE_INDEX index,
    const SYS_MODULE_INIT * const init
);

//******************************************************************************
/* Function:
    void DRV_CTR_Deinitialize( SYS_MODULE_OBJ object )

  Summary:
    Deinitializes the specified instance of the CTR driver module.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    Deinitializes the specified instance of the CTR Driver module,
    disabling its operation (and any hardware) and invalidates all of the
    internal data.

  Precondition:
    Function DRV_CTR_Initialize should have been called before calling
    this function.

  Parameters:
    object -  Driver object handle, returned from the DRV_CTR_Initialize
              function
  Returns:
    None.

  Example:
    <code>
    // This code snippet shows an example of deinitializing the driver.

    SYS_MODULE_OBJ      object;     //  Returned from DRV_CTR_Initialize
    SYS_STATUS          status;


    DRV_CTR_Deinitialize(object);

    status = DRV_CTR_Status(object);
    if (SYS_STATUS_UNINITIALIZED != status)
    {
        // Check again later if you need to know
        // when the driver is deinitialized.
    }
    </code>

  Remarks:
    Once the Initialize operation has been called, the Deinitialize
    operation must be called before the Initialize operation can be called
    again. This function will NEVER block waiting for hardware.
*/

void DRV_CTR_Deinitialize( SYS_MODULE_OBJ object);

//*************************************************************************
/* Function:
    SYS_STATUS DRV_CTR_Status( SYS_MODULE_OBJ object )

  Summary:
    Gets the current status of the CTR Driver module.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function provides the current status of the CTR Driver module.

  Precondition:
    Function DRV_CTR_Initialize should have been called before calling
    this function.

  Parameters:
    object -  Driver object handle, returned from the DRV_CTR_Initialize
              function
  Returns:
    SYS_STATUS_READY - Indicates that the driver is ready and accept requests
                       for new operations

    SYS_STATUS_UNINITIALIZED - Indicates that the driver is not initialized

  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_CTR_Initialize
    SYS_STATUS          CTRStatus;

    CTRStatus = DRV_CTR_Status(object);
    if (SYS_STATUS_ERROR == CTRStatus)
    {
        // Handle error
    }
    </code>

  Remarks:
    A driver can only be opened when its status is SYS_STATUS_READY.

*/

SYS_STATUS DRV_CTR_Status( SYS_MODULE_OBJ object);


// *****************************************************************************
// *****************************************************************************
// Section: CTR Driver Client Routines
// *****************************************************************************
// *****************************************************************************

/* Function:
    DRV_HANDLE DRV_CTR_Open
    (
        const SYS_MODULE_INDEX drvIndex,
        const DRV_IO_INTENT ioIntent
    );

  Summary:
    Opens the specified CTR driver instance and returns a handle to it.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function opens the specified CTR driver instance and provides a
    handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver.

  Precondition:
    Function DRV_CTR_Initialize must have been called before calling
    this function.

  Parameters:
    drvIndex -  Identifier for the object instance to be opened
    ioIntent -  Zero or more of the values from the enumeration
                DRV_IO_INTENT "ORed" together to indicate the intended use
                of the driver

  Returns:
    If successful, the function returns a valid open-instance handle (a
    number identifying both the caller and the module instance).

    If an error occurs, the return value is DRV_HANDLE_INVALID. Errors can occur
	under the following circumstances:
    - if the number of client objects allocated via
         DRV_CTR_CLIENTS_NUMBER is insufficient
    - if the client is trying to open the driver but driver has been opened
         exclusively by another client
    - if the driver hardware instance being opened is not initialized or is
         invalid
    - if the client is trying to open the driver exclusively, but has already
         been opened in a non exclusive mode by another client.
    - if the driver status is not ready.

  Example:
    <code>
    DRV_HANDLE handle;

    handle = DRV_CTR_Open(DRV_CTR_INDEX_0, DRV_IO_INTENT_EXCLUSIVE);
    if (DRV_HANDLE_INVALID == handle)
    {
        // Unable to open the driver
    }
    </code>

  Remarks:
    The driver will always work in Non-Blocking mode even if IO-intent is
    selected as blocking.

    The handle returned is valid until the DRV_CTR_Close function is
    called.

    This function will NEVER block waiting for hardware.
*/

DRV_HANDLE DRV_CTR_Open
(
    const SYS_MODULE_INDEX drvIndex,
    const DRV_IO_INTENT ioIntent
);

// *****************************************************************************
/* Function:
    void DRV_CTR_Close( DRV_Handle handle );

  Summary:
    Closes an opened-instance of the CTR driver.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function closes an opened-instance of the CTR driver, invalidating
    the handle.

  Precondition:
    The DRV_CTR_Initialize function must have been called for the
    specified CTR driver instance.

    DRV_CTR_Open must have been called to obtain a valid opened device
    handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

  Returns:
    None.

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_CTR_Open

    DRV_CTR_Close(handle);
    </code>

  Remarks:
    After calling this function, the handle passed in "handle" must not be used
    with any of the remaining driver routines.  A new handle must be obtained by
    calling DRV_CTR_Open before the caller may use the driver again.

    Note: Usually, there is no need for the driver client to verify that the
    Close operation has completed.
*/

void DRV_CTR_Close( const DRV_HANDLE handle);

// ****************************************************************************
/* Function:
    DRV_CTR_CLIENT_STATUS DRV_CTR_ClientStatus(DRV_HANDLE handle);

  Summary:
    Gets current client-specific status of the CTR driver.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function gets the client-specific status of the CTR driver
    associated with the given handle.

  Precondition:
    The DRV_CTR_Initialize function must have been called.

    DRV_CTR_Open must have been called to obtain a valid opened device
    handle.

  Parameters:
    handle -  A valid open instance handle, returned from the driver's open
              function

  Returns:
    A DRV_CTR_CLIENT_STATUS value describing the current status of the
    driver.

  Example:
    <code>
    DRV_HANDLE      handle;         // Returned from DRV_CTR_Open
    DRV_CTR_CLIENT_STATUS     clientStatus;

    clientStatus = DRV_CTR_ClientStatus(handle);
    if(DRV_CTR_CLIENT_STATUS_READY == clientStatus)
    {
        // do the tasks
    }
    </code>

  Remarks:
    This function will not block for hardware access and will immediately
    return the current status.
*/

DRV_CTR_CLIENT_STATUS   DRV_CTR_ClientStatus( const DRV_HANDLE handle );

// ****************************************************************************
/* Function:
    void DRV_CTR_RegisterCallBack(
		const DRV_HANDLE handle,
		const DRV_CTR_CALLBACK callback,
		const bool oneTimeCallback,
		const uintptr_t context
	);


  Summary:
    Registers a callback function for the event interrupt of CTR.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function registers a client callback function for the event interrupt 
	associated with the use-case. 
	For Wifi usecase, Only Latch 3 interrupt will be enabled, as the last event
	timestamp will be filled in latch 3 for IEEE 802.11v. 
	For USBSoF and GPIO use-cases, only one latch is needed and the interrupt
	will be enabled for the same latch.
	As per user's configuration of interrupt mode for full, half-full or every 
	trigger, the interrupt will be generated and the client callback functions 
	will be called from the ISR.
	The flag oneTimeCallback is passed as an argument for this function. If the
	value of this flag is TRUE, then the callback will be called only once. If 
	client needs one more callback, he needs to register the callback once more.
	If this value is false, then whenever interrupt is generated, the callback
	function will be called until the client call the close function.

  Precondition:
    The DRV_CTR_Initialize function must have been called.

    DRV_CTR_Open must have been called to obtain a valid opened device
    handle.

  Parameters:
    handle  		- A valid open instance handle, returned from the driver's open
					  function
	callback        - A function pointer for client callback function
	oneTimeCallback - If client needs callback to be called only once, then 
					  this flag must be true.
	context			- The value of parameter will be passed back to the client
				      unchanged, when the callback function is called. It can
				      be used to identify any client specific data object that
				      identifies the instance of the client module (for example,
				      it may be a pointer to the client module's state structure).

  Returns:
	None.
	
  Example:
    <code>
	#define CLIENT_ID 0x01
    DRV_HANDLE      handle;         // Returned from DRV_CTR_Open
    void ClientCallack( uintptr_t context, uint32_t * timestampbuffer, 
	                    uint8_t BufferSize);
	
	DRV_CTR_RegisterCallBack(handle, ClientCallack, FALSE, CLIENT_ID);
    </code>

  Remarks:
	The registered callback function will be called from ISR. So, it is 
	recommended to keep the callback functions light and not process intensive.
 */

void DRV_CTR_RegisterCallBack
(
    const DRV_HANDLE handle,
    const DRV_CTR_CALLBACK callback,
	const bool oneTimeCallback,
    const uintptr_t context
);

// ****************************************************************************
/* Function:
    void DRV_CTR_EventISR(SYS_MODULE_OBJ object);

  Summary:
    Interrupt Service Routine called for the CTR event interrupt.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function is called when the interrupt is generated for CTR event 
	interrupt. The latch buffers are read and stored in a local buffer, and all 
	the registered client callback functions will be called from this function.
	The number of latches to be read depends upon the use-case configured. For
	wifi, 4 latches are read, and for USBSoF and GPIO, only 1 latch is read.
	Number of buffers to read in each latch depends on the interrupt mode configuration.
	For Full, all 4 buffers needs to be read, whereas for half-full, only 2 buffers
	needs to be read and for every trigger, only 1 buffer is read.

  Precondition:
	None.
  
  Parameters:
	object - The driver instance handle returned after the initialization.
	
  Returns:
	None.
	
  Example:
	This function is not called from clients/system. This function will be called
	when the interrupt for event is generated.
	
  Remarks:
	All the handling specific for a client should be done in the respective callback
	functions. This function should not be modified.
 */
 
void DRV_CTR_EventISR(SYS_MODULE_OBJ object);

// ****************************************************************************
/* Function:
    void DRV_CTR_TriggerISR(SYS_MODULE_OBJ object);

  Summary:
    Interrupt Service Routine called for the CTR Trigger interrupt.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function is called when the interrupt is generated for CTR trigger 
	interrupt. The interrupt handling for this interrupt is application specific.
	So, this function is kept open for the clients to modify.

  Precondition:
	None.
  
  Parameters:
	object - The driver instance handle returned after the initialization.
	
  Returns:
	None.
	
  Example:
	This function is not called from clients/system. This function will be called
	when the interrupt for event is generated.
	
  Remarks:
	Specific interrupt handling can be taken care of by application developer, as
	the need for this interrupt is application specific.
  */
 
void DRV_CTR_TriggerISR(SYS_MODULE_OBJ object);

// ****************************************************************************
/* Function:
    void DRV_CTR_Adjust(DRV_HANDLE handle, CTR_LATCH_CTR_SELECT ctrSel, 
					    uint16_t adjustVal);

  Summary:
    Sets the adjust value for a given CTR counter.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function sets the adjust value for a given CTR counter.

  Precondition:
    The DRV_CTR_Initialize function must have been called.

    DRV_CTR_Open must have been called to obtain a valid opened device
    handle.
  
  Parameters:
	handle    - A valid open instance handle, returned from the driver's open
			    function
	ctrSel    - CTR counter to be selected out of the 4 counters available.
	adjustVal - Adjust value to be set
	
  Returns:
	None.
	
  Example:
	<code>
	DRV_HANDLE handle; // handle returned by open function
	uint16_t adjustVal = 0xFFF;
	
	DRV_CTR_Adjust(handle, CTR_CTR0_LIN, adjustVal);
	</code>
	
  Remarks:

  */
 
void DRV_CTR_Adjust(DRV_HANDLE handle, CTR_LATCH_CTR_SELECT ctrSel, uint16_t adjustVal);

// ****************************************************************************
/* Function:
    void DRV_CTR_Drift(DRV_HANDLE handle, CTR_LATCH_CTR_SELECT ctrSel, 
					    uint16_t driftVal);

  Summary:
    Sets the drift value for a given CTR counter.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function sets the drift value for a given CTR counter.

  Precondition:
    The DRV_CTR_Initialize function must have been called.

    DRV_CTR_Open must have been called to obtain a valid opened device
    handle.
  
  Parameters:
	handle    - A valid open instance handle, returned from the driver's open
			    function
	ctrSel    - CTR counter to be selected out of the 4 counters available.
	adjustVal - Drift value to be set
	
  Returns:
	None.
	
  Example:
	<code>
	DRV_HANDLE handle; // handle returned by open function
	uint16_t driftVal = 0xFFF;
	
	DRV_CTR_Drift(handle, CTR_CTR0_LIN, driftVal);
	</code>
	
  Remarks:

  */
 
void DRV_CTR_Drift(DRV_HANDLE handle, CTR_LATCH_CTR_SELECT ctrSel, uint32_t driftVal);

#endif // #ifndef _DRV_CTR_H
/*******************************************************************************
 End of File
*/

