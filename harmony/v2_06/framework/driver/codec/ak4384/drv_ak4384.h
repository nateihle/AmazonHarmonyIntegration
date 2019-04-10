/*******************************************************************************
  AK4384 Codec Driver Interface

  Company:
    Microchip Technology Inc.

  File Name:
    drv_ak4384.h

  Summary:
    AK4384 Codec Driver Interface header file

  Description:
    The AK4384 Codec device driver interface provides a simple interface to
    manage the AK4384 106 dB 192 kHz 24-Bit DAC that can be interfaced
    Microchip Microcontroller. This file provides the
    interface definition for the AK4384 Codec device driver.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

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
// DOM-IGNORE-END

/*************************************************************
 * Include files.
 ************************************************************/

#ifndef _DRV_AK4384_H
#define _DRV_AK4384_H

// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************
#include "system/int/sys_int.h"
#include "system/ports/sys_ports.h"
#include "driver/driver_common.h"
#include "driver/i2s/drv_i2s.h"

// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Driver AK4384 Module Index

  Summary:
    AK4384 driver index definitions.

  Description:
    These constants provide AK4384 driver index definition.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.
    These values should be passed into the DRV_AK4384_Initialize and
    DRV_AK4384_Open routines to identify the driver instance in use.
 */
#define      DRV_AK4384_INDEX_0      0
#define      DRV_AK4384_INDEX_1      1
#define      DRV_AK4384_INDEX_2      2
#define      DRV_AK4384_INDEX_3      3
#define      DRV_AK4384_INDEX_4      4
#define      DRV_AK4384_INDEX_5      5


// *****************************************************************************
/* AK4384 Driver Module Count

  Summary:
    Number of valid AK4384 driver indices.

  Description:
    This constant identifies the maximum number of AK4384 Driver instances that
    should be defined by the application. Defining more instances than this
    constant will waste RAM memory space.

    This constant can also be used by the application to identify the number of
    AK4384 instances on this microcontroller.

  Remarks:
    This value is device-specific.

 */
#define DRV_AK4384_COUNT /*DOM-IGNORE-BEGIN*/ DRV_AK4384_INSTANCES_NUMBER /*DOM-IGNORE-END*/


// *****************************************************************************
/* AK4384 Driver Buffer Handle

  Summary:
    Handle identifying a write buffer passed to the driver.

  Description:
    A buffer handle value is returned by a call to the DRV_AK4384_BufferAddWrite
    function. This handle is associated with the buffer passed into the function
    and it allows the application to track the completion of the data from (or into)
    that buffer.  The buffer handle value returned from the "buffer add" function
    is returned back to the client by the "event handler callback" function
    registered with the driver.

    The buffer handle assigned to a client request expires when the client has
    been notified of the completion of the buffer transfer (after event handler
    function that notifies the client returns) or after the buffer has been
    retired by the driver if no event handler callback was set.

  Remarks:
    None.
 */
typedef uintptr_t DRV_AK4384_BUFFER_HANDLE;


// *****************************************************************************
/* AK4384 Driver Invalid Buffer Handle

  Summary:
    Definition of an invalid buffer handle.

  Description:
    This is the definition of an invalid buffer handle. An invalid buffer handle
    is returned by DRV_AK4384_BufferAddWrite function if the buffer add request
    was not successful.

  Remarks:
    None.
 */
#define DRV_AK4384_BUFFER_HANDLE_INVALID ((DRV_AK4384_BUFFER_HANDLE)(-1))

// *****************************************************************************

/* AK4384 Driver Events

   Summary:
    Identifies the possible events that can result from a buffer add request.

   Description:
    This enumeration identifies the possible events that can result from a
    buffer add request caused by the client calling either the
    DRV_AK4384_BufferAddWrite function.

   Remarks:
    One of these values is passed in the "event" parameter of the event
    handling callback function that the client registered with the driver by
    calling the DRV_AK4384_BufferEventHandlerSet function when a buffer
    transfer request is completed.

 */
typedef enum {
    /* Data was transferred successfully. */
    DRV_AK4384_BUFFER_EVENT_COMPLETE,

    /* Error while processing the request */
    DRV_AK4384_BUFFER_EVENT_ERROR,

    /* Data transfer aborted (Applicable in DMA mode) */
    DRV_AK4384_BUFFER_EVENT_ABORT

} DRV_AK4384_BUFFER_EVENT;


typedef enum
{
    DATA_LENGTH_16,
    DATA_LENGTH_24,
    DATA_LENGTH_32,
}DATA_LENGTH; // in bits


//left/right channel sample length
typedef enum
{
    SAMPLE_LENGTH_16,
    SAMPLE_LENGTH_32
}SAMPLE_LENGTH; //in bits

// *****************************************************************************
/* AK4384 Driver Buffer Event Handler Function

   Summary:
    Pointer to a AK4384 Driver Buffer Event handler function.

   Description:
    This data type defines the required function signature for the AK4384 driver
    buffer event handling callback function. A client must register a pointer
    to a buffer event handling function whose function signature (parameter
    and return value types) match the types specified by this function pointer
    in order to receive buffer related event calls back from the driver.

    The parameters and return values are described here and a partial example
    implementation is provided.

  Parameters:
    event           - Identifies the type of event

    bufferHandle    - Handle identifying the buffer to which the event relates

    context         - Value identifying the context of the application that registered
                      the event handling function.

  Returns:
    None.

  Example:
    <code>
    void APP_MyBufferEventHandler( DRV_AK4384_BUFFER_EVENT event,
                                   DRV_AK4384_BUFFER_HANDLE bufferHandle,
                                   uintptr_t context )
    {
        MY_APP_DATA_STRUCT pAppData = (MY_APP_DATA_STRUCT) context;

        switch(event)
        {
            case DRV_AK4384_BUFFER_EVENT_COMPLETE:
                // Handle the completed buffer.
            break;

            case DRV_AK4384_BUFFER_EVENT_ERROR:
            default:
                // Handle error.
            break;
        }
    }
    </code>

   Remarks:

    If the event is DRV_AK4384_BUFFER_EVENT_COMPLETE, this means that the data
    was transferred successfully.

    If the event is DRV_AK4384_BUFFER_EVENT_ERROR, this means that the data was
    not transferred successfully. The bufferHandle parameter contains the buffer
    handle of the buffer that failed. The DRV_AK4384_BufferProcessedSizeGet function
    can be called to find out how many bytes were processed.

    The bufferHandle parameter contains the buffer handle of the buffer that
    associated with the event.

    The context parameter contains a handle to the client context,
    provided at the time the event handling function was  registered using the
    DRV_AK4384_BufferEventHandlerSet function.  This context handle value is
    passed back to the client as the "context" parameter.  It can be any value
    necessary to identify the client context or instance (such as a pointer to
    the client's data) instance of the client that made the buffer add request.

    The buffer handle in bufferHandle expires after this event handler exits. In
    that the buffer object that was allocated is deallocated by the driver
    after the event handler exits.

    The event handler function executes in the data driver (I2S) peripheral's interrupt
    context when the driver is configured for interrupt mode operation. It is
    recommended of the application to not perform process intensive or blocking
    operations with in this function.

    DRV_AK4384_BufferAddWrite function can be called in the event handler
    to add a buffer to the driver queue.

 */
typedef void (*DRV_AK4384_BUFFER_EVENT_HANDLER) (DRV_AK4384_BUFFER_EVENT event,
        DRV_AK4384_BUFFER_HANDLE bufferHandle, uintptr_t contextHandle);


// *****************************************************************************
/* AK4384 Driver Command Event Handler Function

   Summary:
    Pointer to a AK4384 Driver Command Event Handler Function

   Description:
    This data type defines the required function signature for the AK4384 driver
    command event handling callback function.

    A command is a control instruction to the AK4384 Codec. For
    example, Mute ON/OFF, Zero Detect Enable/Disable, etc.

    A client must register a pointer to a command event handling function whose
    function signature (parameter and return value types) match the types
    specified by this function pointer in order to receive command related event
    calls back from the driver.

    The parameters and return values are described here and a partial example
    implementation is provided.

  Parameters:
    context         - Value identifying the context of the application that registered
                      the event handling function.

  Returns:
    None.

  Example:
    <code>
    void APP_AK4384CommandEventHandler( uintptr_t context )
    {
        MY_APP_DATA_STRUCT pAppData = (MY_APP_DATA_STRUCT) context;

        // Last Submitted command is completed.
        // Perform further processing here
    }
    </code>

   Remarks:

    The occurrence of this call back means that the last control command was
    transferred successfully.

    The context parameter contains a handle to the client context,
    provided at the time the event handling function was  registered using the
    DRV_AK4384_CommandEventHandlerSet function.  This context handle value is
    passed back to the client as the "context" parameter.  It can be any value
    necessary to identify the client context or instance (such as a pointer to
    the client's data) of the client that made the buffer add request.

    The event handler function executes in the control data driver
    interrupt context. It is recommended of the application to not perform
    process intensive or blocking operations with in this function.

 */
typedef void (*DRV_AK4384_COMMAND_EVENT_HANDLER) (uintptr_t contextHandle);


// *****************************************************************************

/* AK4384 Audio data format

  Summary:
    Identifies the Serial Audio data interface format.

  Description:
    This enumeration identifies Serial Audio data interface format.

  Remarks:
    None.
 */
typedef enum {
    /* 16 bit Right Justified Audio data format */
    DRV_AK4384_AUDIO_DATA_FORMAT_16BIT_RIGHT_JUSTIFIED = 0,

    /* 20 bit Right Justified Audio data format */
    DRV_AK4384_AUDIO_DATA_FORMAT_20BIT_RIGHT_JUSTIFIED,

    /* 24 bit Left Justified Audio data format */
    DRV_AK4384_AUDIO_DATA_FORMAT_24BIT_LEFT_JUSTIFIED,

    /* 24 bit I2S Audio data format */
    DRV_AK4384_AUDIO_DATA_FORMAT_24BIT_I2S,

    /* 24 bit Right Justified Audio data format */
    DRV_AK4384_AUDIO_DATA_FORMAT_24BIT_RIGHT_JUSTIFIED

} DRV_AK4384_AUDIO_DATA_FORMAT;


// *****************************************************************************

/* AK4384 Master clock frequency mode

  Summary:
    Identifies the mode of master clock to AK4384 DAC.

  Description:
    This enumeration identifies mode of master clock to AK4384 DAC.
    In Manual Setting Mode, the sampling speed is set by setting  DFS0/1 bits in
    Control Register 2. The frequency of MCLK at each sampling speed is set
    automatically.
    In Auto Setting Mode, the MCLK frequency is detected automatically

  Remarks:
    None.
 */
typedef enum {
    /* Master clock frequency mode Manual */
    DRV_AK4384_MCLK_MODE_MANUAL,

    /* Master clock frequency mode Auto
           This is the default mode. */
    DRV_AK4384_MCLK_MODE_AUTO

} DRV_AK4384_MCLK_MODE;


// *****************************************************************************

/* AK4384 Zero Detect mode

  Summary:
    Identifies Zero Detect Function mode

  Description:
    This enumeration identifies the mode of zero detect function

  Remarks:
    None.
 */
typedef enum {
    /* Zero Detect channel separated.
       When the input data at each channel is continuously zeros
       for 8192 LRCK cycles, DZF pin of each channel goes to “H”
       This is the default mode. */
    DRV_AK4384_ZERO_DETECT_MODE_CHANNEL_SEPARATED,

    /* Zero Detect Anded
      DZF pins of both channels go to “H” only when
      the input data at both channels are continuously
      zeros for 8192 LRCK cycles */
    DRV_AK4384_ZERO_DETECT_MODE_ANDED

} DRV_AK4384_ZERO_DETECT_MODE;


// *****************************************************************************

/* AK4384 De-Emphasis Filter

  Summary:
    Identifies de-emphasis filter function.

  Description:
    This enumeration identifies the settings for de-emphasis filter function.

  Remarks:
    None.
 */
typedef enum {
    /* De-Emphasis filter for 44.1kHz. */
    DRV_AK4384_DEEMPHASIS_FILTER_44_1KHZ,

    /* De-Emphasis filter Off
       This is the default setting.*/
    DRV_AK4384_DEEMPHASIS_FILTER_OFF,

    /* De-Emphasis filter for 48kHz. */
    DRV_AK4384_DEEMPHASIS_FILTER_48KHZ,

    /* De-Emphasis filter for 32kHz. */
    DRV_AK4384_DEEMPHASIS_FILTER_32KHZ,

} DRV_AK4384_DEEMPHASIS_FILTER;


// *****************************************************************************

/* AK4384 Audio Channel

  Summary:
    Identifies Left/Right Audio channel

  Description:
    This enumeration identifies Left/Right Audio channel

  Remarks:
    None.
 */
typedef enum {
    DRV_AK4384_CHANNEL_LEFT,

    DRV_AK4384_CHANNEL_RIGHT,

    DRV_AK4384_CHANNEL_LEFT_RIGHT,

    DRV_AK4384_NUMBER_OF_CHANNELS

} DRV_AK4384_CHANNEL;


// *****************************************************************************

/* AK4384 Driver Initialization Data

  Summary:
    Defines the data required to initialize or reinitialize the AK4384 driver.

  Description:
    This data type defines the data required to initialize or reinitialize the
    AK4384 Codec driver.

  Remarks:
    None.
 */
typedef struct {
    /* System module initialization */
    SYS_MODULE_INIT moduleInit;

    /* Identifies control module(SPI) driver ID for
       control interface of Codec */
    SYS_MODULE_INDEX spiDriverModuleIndex;

    /* Identifies data module(I2S) driver ID for
       data interface of Codec */
    SYS_MODULE_INDEX i2sDriverModuleIndex;

    /* Volume */
    uint8_t volume;

    /* Set MCLK mode. */
    DRV_AK4384_MCLK_MODE mclkMode;


    /* true if driver initialization should be delayed due to shared RESET pin */
    bool delayDriverInitialization;

} DRV_AK4384_INIT;


// *****************************************************************************
// *****************************************************************************
// Section: AK4384 Driver Module Interface Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/*   Function:
        SYS_MODULE_OBJ  DRV_AK4384_Initialize
        (
                const SYS_MODULE_INDEX drvIndex,
                const SYS_MODULE_INIT *const init
        );

  Summary:
    Initializes hardware and data for the instance of the AK4384 DAC module.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This routine initializes the AK4384 driver instance for the specified driver
    index, making it ready for clients to open and use it. The initialization
    data is specified by the 'init' parameter. The initialization may fail if the
    number of driver objects allocated are insufficient or if the specified
    driver instance is already initialized.

  Precondition:
    DRV_I2S_Initialize must be called before calling this function to initialize
    the data interface of this CODEC driver.
    DRV_SPI_Initialize must be called if SPI driver is used for handling
    the control interface of this CODEC driver.

  Parameters:
    drvIndex        - Identifier for the driver instance to be initialized
    init            - Pointer to the data structure containing any data
                      necessary to initialize the hardware. This pointer may
                      be null if no data is required and default
                      initialization is to be used.

  Returns:
    If successful, returns a valid handle to a driver instance object.
    Otherwise, it returns SYS_MODULE_OBJ_INVALID.

Example:
    <code>
    DRV_AK4384_INIT              	init;
    SYS_MODULE_OBJ              	objectHandle;

    init.moduleInit.value           = SYS_MODULE_POWER_RUN_FULL;
    init.spiDriverModuleIndex       = DRV_SPI_INDEX_0;	// This will be ignored for a custom
														// control interface driver implementation
    init.i2sDriverModuleIndex       = DRV_I2S_INDEX_0;
    init.mclkMode	            	= DRV_AK4384_MCLK_MODE_MANUAL;
    init.audioDataFormat            = DRV_AK4384_AUDIO_DATA_FORMAT_24BIT_I2S;
    init.powerDownPortChannel	    = PORT_CHANNEL_G;
    init.powerDownBitPosition       = PORTS_BIT_POS_15;

  objectHandle = DRV_AK4384_Initialize(DRV_AK4384_0, (SYS_MODULE_INIT*)init);
    if (SYS_MODULE_OBJ_INVALID == objectHandle)
    {
        // Handle error
    }
    </code>

  Remarks:
    This routine must be called before any other AK4384 routine is called.

    This routine should only be called once during system initialization
    unless DRV_AK4384_Deinitialize is called to deinitialize the driver
    instance. This routine will NEVER block for hardware access.

 */
SYS_MODULE_OBJ DRV_AK4384_Initialize
(
        const SYS_MODULE_INDEX drvIndex,
        const SYS_MODULE_INIT * const init
);


// *****************************************************************************
/* Function:
    void DRV_AK4384_Deinitialize( SYS_MODULE_OBJ object)

  Summary:
    Deinitializes the specified instance of the AK4384 driver module.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    Deinitializes the specified instance of the AK4384 driver module, disabling
    its operation (and any hardware).  Invalidates all the internal data.

  Precondition:
    Function DRV_AK4384_Initialize should have been called before calling this
    function.

  Parameters:
    object          - Driver object handle, returned from the
                      DRV_AK4384_Initialize routine

  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object;     //  Returned from DRV_AK4384_Initialize
    SYS_STATUS          status;


    DRV_AK4384_Deinitialize(object);

    status = DRV_AK4384_Status(object);
    if (SYS_MODULE_DEINITIALIZED != status)
    {
        // Check again later if you need to know
        // when the driver is deinitialized.
    }
    </code>

  Remarks:
    Once the Initialize operation has been called, the Deinitialize operation
    must be called before the Initialize operation can be called again. This
    routine will NEVER block waiting for hardware.
 */
void DRV_AK4384_Deinitialize(SYS_MODULE_OBJ object);


// *****************************************************************************
/* Function:
    SYS_STATUS DRV_AK4384_Status( SYS_MODULE_OBJ object)

  Summary:
    Gets the current status of the AK4384 driver module.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This routine provides the current status of the AK4384 driver module.

  Precondition:
    Function DRV_AK4384_Initialize should have been called before calling this
    function.

  Parameters:
    object          - Driver object handle, returned from the
                      DRV_AK4384_Initialize routine

  Returns:
    SYS_STATUS_DEINITIALIZED  - Indicates that the driver has been
                                deinitialized

    SYS_STATUS_READY          - Indicates that any previous module operation
                                for the specified module has completed

    SYS_STATUS_BUSY           - Indicates that a previous module operation for
                                the specified module has not yet completed

    SYS_STATUS_ERROR          - Indicates that the specified module is in an
                                error state

  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_AK4384_Initialize
    SYS_STATUS          ak4384Status;

    ak4384Status = DRV_AK4384_Status(object);
    if (SYS_STATUS_READY == ak4384Status)
    {
        // This means the driver can be opened using the
        // DRV_AK4384_Open function.
    }
    </code>

  Remarks:
    A driver can opened only when its status is SYS_STATUS_READY.
 */
SYS_STATUS DRV_AK4384_Status(SYS_MODULE_OBJ object);

//****************************************************************************
/* Function:
    void DRV_AK4384_EnableInitialization(SYS_MODULE_OBJ object);

  Summary:
   Enable delayed initialization of the driver.

  Description:
   If the AK4384 codec is sharing a RESET line with another peripheral, such as
   a Bluetooth module with its own driver, then the codec driver initialization
   has to be delayed until after the Bluetooth module has toggled its RESET pin.
   Once this has been accomplished, this function should be called to kick-start
   the codec driver initialization.

  Precondition:
    The DRV_AK4384_Initialize routine must have been called for the specified
    AK4384 driver instance.

  Parameters:
    object      - Object handle for the specified driver instance (returned from
                  DRV_AK4384_Initialize)
  Returns:
    None.

  Remarks:
    This is not needed for audio-only applications without a Bluetooth module.
*/
void DRV_AK4384_EnableInitialization(SYS_MODULE_OBJ object);

//*****************************************************************************
/* Function:
    bool DRV_AK4384_IsInitializationDelayed(SYS_MODULE_OBJ object);

  Summary:
   Checks if delayed initialization of the driver has been requested.

  Description:
   If the AK4384 codec is sharing a RESET line with another peripheral, such as
   a Bluetooth module with its own driver, then the codec driver initialization
   has to be delayed until after the Bluetooth module has toggled its RESET pin.
   This function returns true if that option has been selected in MHC in the
   checkbox: "Delay driver initialization (due to shared RESET pin)"

  Precondition:
    The DRV_AK4384_Initialize routine must have been called for the specified
    AK4384 driver instance.

  Parameters:
    object      - Object handle for the specified driver instance (returned from
                  DRV_AK4384_Initialize)
  Returns:
    true if the delayed initialization option has been enabled

  Remarks:
    This is not needed for audio-only applications without a Bluetooth module.
*/

bool DRV_AK4384_IsInitializationDelayed(SYS_MODULE_OBJ object);


// *****************************************************************************
/* Function:
    void  DRV_AK4384_Tasks(SYS_MODULE_OBJ object);

  Summary:
    Maintains the driver's control and data interface state machine.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This routine is used to maintain the driver's internal control and data
    interface state machine and implement its control and data interface
    implementations.
    This function should be called from the SYS_Tasks function.

  Precondition:
    The DRV_AK4384_Initialize routine must have been called for the specified
    AK4384 driver instance.

  Parameters:
    object      - Object handle for the specified driver instance (returned from
                  DRV_AK4384_Initialize)

  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_AK4384_Initialize

    while (true)
    {
        DRV_AK4384_Tasks (object);

        // Do other tasks
    }
    </code>

  Remarks:
    This routine is normally not called directly by an application.  It is
    called by the system's Tasks routine (SYS_Tasks).

 */
void DRV_AK4384_Tasks(SYS_MODULE_OBJ object);


// *****************************************************************************
// *****************************************************************************
// Section: AK4384 CODEC Driver Client Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    DRV_HANDLE DRV_AK4384_Open
    (
        const SYS_MODULE_INDEX drvIndex,
        const DRV_IO_INTENT ioIntent
    )

  Summary:
    Opens the specified AK4384 driver instance and returns a handle to it.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This routine opens the specified AK4384 driver instance and provides a
    handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver. The ioIntent
    parameter defines how the client interacts with this driver instance.

    The DRV_IO_INTENT_BLOCKING and DRV_IO_INTENT_NONBLOCKING ioIntent
    options are not relevant to this driver. All the data transfer functions
    of this driver are non blocking.

    Only DRV_IO_INTENT_WRITE is a valid ioIntent option as AK4384 is DAC only.

    Specifying a DRV_IO_INTENT_EXCLUSIVE will cause the driver to provide
    exclusive access to this client. The driver cannot be opened by any
    other client.

  Precondition:
    Function DRV_AK4384_Initialize must have been called before calling this
    function.

  Parameters:
    drvIndex    - Identifier for the object instance to be opened

    ioIntent    - Zero or more of the values from the enumeration
                  DRV_IO_INTENT "ORed" together to indicate the intended use
                  of the driver. See function description for details.

  Returns:
    If successful, the routine returns a valid open-instance handle (a number
    identifying both the caller and the module instance).

    If an error occurs, the return value is DRV_HANDLE_INVALID. Errors can occur under
	following conditions:
    - if the number of client objects allocated via DRV_AK4384_CLIENTS_NUMBER is insufficient
    - if the client is trying to open the driver but driver has been opened exclusively by another client
    - if the driver hardware instance being opened is not initialized or is invalid
    - if the ioIntent options passed are not relevant to this driver

  Example:
    <code>
    DRV_HANDLE handle;

    handle = DRV_AK4384_Open(DRV_AK4384_INDEX_0, DRV_IO_INTENT_EXCLUSIVE);
    if (DRV_HANDLE_INVALID == handle)
    {
        // Unable to open the driver
        // May be the driver is not initialized or the initialization
        // is not complete.
    }
    </code>

  Remarks:
    The handle returned is valid until the DRV_AK4384_Close routine is called.
    This routine will NEVER block waiting for hardware.If the requested intent
    flags are not supported, the routine will return DRV_HANDLE_INVALID.  This
    function is thread safe in a RTOS application. It should not be called in an
    ISR.
 */
DRV_HANDLE DRV_AK4384_Open
(
    const SYS_MODULE_INDEX iDriver,
    const DRV_IO_INTENT ioIntent
);

// *****************************************************************************
/* Function:
    void DRV_AK4384_Close( DRV_Handle handle )

  Summary:
    Closes an opened-instance of the AK4384 driver.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This routine closes an opened-instance of the AK4384 driver, invalidating the
    handle. Any buffers in the driver queue that were submitted by this client
    will be removed.  After calling this routine, the handle passed in "handle"
    must not be used with any of the remaining driver routines.  A new handle must
    be obtained by calling DRV_AK4384_Open before the caller may use the driver
    again

  Precondition:
    The DRV_AK4384_Initialize routine must have been called for the specified
    AK4384 driver instance.

    DRV_AK4384_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None.

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_AK4384_Open

    DRV_AK4384_Close(handle);

    </code>

  Remarks:
    Usually there is no need for the driver client to verify that the Close
    operation has completed.  The driver will abort any ongoing operations
    when this routine is called.
 */
void DRV_AK4384_Close(const DRV_HANDLE handle);


// *****************************************************************************
/*
Function:
        void DRV_AK4384_BufferAddWrite
        (
            const DRV_HANDLE handle,
            DRV_AK4384_BUFFER_HANDLE *bufferHandle,
            void *buffer, size_t size
        )

  Summary:
    Schedule a non-blocking driver write operation.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function schedules a non-blocking write operation. The function returns
    with a valid buffer handle in the bufferHandle argument if the write request
    was scheduled successfully. The function adds the request to the hardware
    instance transmit queue and returns immediately. While the request is in the
    queue, the application buffer is owned by the driver and should not be
    modified.  The function returns DRV_AK4384_BUFFER_HANDLE_INVALID if:
    - a buffer could not be allocated to the request
    - the input buffer pointer is NULL
    - the buffer size is '0'
    - the queue is full or the queue depth is insufficient
    If the requesting client registered an event callback with the driver,
    the driver will issue a DRV_AK4384_BUFFER_EVENT_COMPLETE event if the buffer
    was processed successfully of DRV_AK4384_BUFFER_EVENT_ERROR event if the
    buffer was not processed successfully.

  Precondition:
    The DRV_AK4384_Initialize routine must have been called for the specified
    AK4384 device instance and the DRV_AK4384_Status must have returned
    SYS_STATUS_READY.

    DRV_AK4384_Open must have been called to obtain a valid opened device handle.

    DRV_IO_INTENT_WRITE must have been specified in the DRV_AK4384_Open call.

  Parameters:
    handle       - Handle of the AK4384 instance as return by the
                   DRV_AK4384_Open function.
    buffer       - Data to be transmitted.
    size         - Buffer size in bytes.
    bufferHandle - Pointer to an argument that will contain the
                   return buffer handle.

  Returns:
    The bufferHandle parameter will contain the return buffer handle. This will be
    DRV_AK4384_BUFFER_HANDLE_INVALID if the function was not successful.

  Example:
    <code>

    MY_APP_OBJ myAppObj;
    uint8_t mybuffer[MY_BUFFER_SIZE];
    DRV_AK4384_BUFFER_HANDLE bufferHandle;

    // myAK4384Handle is the handle returned
    // by the DRV_AK4384_Open function.

    // Client registers an event handler with driver

    DRV_AK4384_BufferEventHandlerSet(myAK4384Handle,
                    APP_AK4384BufferEventHandler, (uintptr_t)&myAppObj);

    DRV_AK4384_BufferAddWrite(myAK4384handle, &bufferHandle
                                        myBuffer, MY_BUFFER_SIZE);

    if(DRV_AK4384_BUFFER_HANDLE_INVALID == bufferHandle)
    {
        // Error handling here
    }

    // Event is received when
    // the buffer is processed.

    void APP_AK4384BufferEventHandler(DRV_AK4384_BUFFER_EVENT event,
            DRV_AK4384_BUFFER_HANDLE bufferHandle, uintptr_t contextHandle)
    {
        // contextHandle points to myAppObj.

        switch(event)
        {
            case DRV_AK4384_BUFFER_EVENT_COMPLETE:

                // This means the data was transferred.
                break;

            case DRV_AK4384_BUFFER_EVENT_ERROR:

                // Error handling here.
                break;

            default:
                break;
        }
    }

    </code>

  Remarks:
    This function is thread safe in a RTOS application. It can be called from
    within the AK4384 Driver Buffer Event Handler that is registered by this
    client. It should not be called in the event handler associated with another
    AK4384 driver instance. It should not otherwise be called directly in an ISR.

 */
void DRV_AK4384_BufferAddWrite
(
    const DRV_HANDLE handle,
    DRV_AK4384_BUFFER_HANDLE *bufferHandle,
    void *buffer, size_t size
);


// *****************************************************************************
/*
  Function:
        void DRV_AK4384_BufferEventHandlerSet
        (
            DRV_HANDLE handle,
            const DRV_AK4384_BUFFER_EVENT_HANDLER eventHandler,
            const uintptr_t contextHandle
        )

  Summary:
    This function allows a client to identify a buffer event handling function
    for the driver to call back when queued buffer transfers have finished.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function allows a client to identify a buffer event handling function
    for the driver to call back when queued buffer transfers have finished.
    When a client calls DRV_AK4384_BufferAddWrite function, it is provided with
    a handle identifying  the buffer that was added to the driver's buffer queue.
    The driver will pass this handle back to the client by calling "eventHandler"
    function when the buffer transfer has completed.

    The event handler should be set before the client performs any "buffer add"
    operations that could generate events. The event handler once set, persists
    until the client closes the driver or sets another event handler (which
    could be a "NULL" pointer to indicate no callback).

  Precondition:
    The DRV_AK4384_Initialize routine must have been called for the specified
    AK4384 driver instance.

    DRV_AK4384_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine
    eventHandler - Pointer to the event handler function.
    context      - The value of parameter will be passed back to the client
                   unchanged, when the eventHandler function is called.  It can
                   be used to identify any client specific data object that
                   identifies the instance of the client module (for example,
                   it may be a pointer to the client module's state structure).

  Returns:
    None.

  Example:
    <code>
    MY_APP_OBJ myAppObj;
    uint8_t mybuffer[MY_BUFFER_SIZE];
    DRV_AK4384_BUFFER_HANDLE bufferHandle;

    // myAK4384Handle is the handle returned
    // by the DRV_AK4384_Open function.

    // Client registers an event handler with driver

    DRV_AK4384_BufferEventHandlerSet(myAK4384Handle,
                    APP_AK4384BufferEventHandler, (uintptr_t)&myAppObj);

    DRV_AK4384_BufferAddWrite(myAK4384handle, &bufferHandle
                                        myBuffer, MY_BUFFER_SIZE);

    if(DRV_AK4384_BUFFER_HANDLE_INVALID == bufferHandle)
    {
        // Error handling here
    }

    // Event is received when
    // the buffer is processed.

    void APP_AK4384BufferEventHandler(DRV_AK4384_BUFFER_EVENT event,
            DRV_AK4384_BUFFER_HANDLE bufferHandle, uintptr_t contextHandle)
    {
        // contextHandle points to myAppObj.

        switch(event)
        {
            case DRV_AK4384_BUFFER_EVENT_COMPLETE:

                // This means the data was transferred.
                break;

            case DRV_AK4384_BUFFER_EVENT_ERROR:

                // Error handling here.
                break;

            default:
                break;
        }
    }
    </code>

  Remarks:
    If the client does not want to be notified when the queued buffer transfer
    has completed, it does not need to register a callback.
 */
void DRV_AK4384_BufferEventHandlerSet
(
    DRV_HANDLE handle,
    const DRV_AK4384_BUFFER_EVENT_HANDLER eventHandler,
    const uintptr_t contextHandle
);


// *****************************************************************************
/*
  Function:
    size_t DRV_AK4384_BufferProcessedSizeGet(DRV_HANDLE handle)

  Summary:
    This function returns number of bytes that have been processed for the
    specified buffer.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function returns number of bytes that have been processed for the
    specified buffer. The client can use this function, in a case where the
    buffer has terminated due to an error, to obtain the number of bytes that
    have been processed.
    If this function is called on a invalid buffer handle, or if the buffer
    handle has expired, the function returns 0.

  Precondition:
    The DRV_AK4384_Initialize routine must have been called for the specified
    I2S driver instance.

    DRV_AK4384_Open must have been called to obtain a valid opened device handle.

     One of DRV_AK4384_BufferAddRead, DRV_AK4384_BufferAddWrite
     function must have been called and a valid buffer handle returned.

  Parameters:
    bufferhandle    - Handle of the buffer of which the processed number of bytes
                      to be obtained.

  Returns:
    Returns the number of the bytes that have been processed for this buffer.
    Returns 0 for an invalid or an expired buffer handle.

  Example:
    <code>
    // myAppObj is an application specific object.
    MY_APP_OBJ myAppObj;

    uint8_t mybuffer[MY_BUFFER_SIZE];
    DRV_AK4384_BUFFER_HANDLE bufferHandle;

    // myAK4384Handle is the handle returned
    // by the DRV_AK4384_Open function.

    // Client registers an event handler with driver. This is done once

    DRV_AK4384_BufferEventHandlerSet(myAK4384Handle, APP_AK4384BufferEventHandle,
                                                            (uintptr_t)&myAppObj);

    DRV_AK4384_BufferAddRead(myAK4384handle,&bufferHandle,
                                        myBuffer, MY_BUFFER_SIZE);

    if(DRV_AK4384_BUFFER_HANDLE_INVALID == bufferHandle)
    {
        // Error handling here
    }

    // Event Processing Technique. Event is received when
    // the buffer is processed.

    void APP_AK4384BufferEventHandler(DRV_AK4384_BUFFER_EVENT event,
            DRV_AK4384_BUFFER_HANDLE bufferHandle, uintptr_t contextHandle)
    {
        // The context handle was set to an application specific
        // object. It is now retrievable easily in the event handler.
        MY_APP_OBJ myAppObj = (MY_APP_OBJ *) contextHandle;
        size_t processedBytes;

        switch(event)
        {
            case DRV_AK4384_BUFFER_EVENT_COMPLETE:

                // This means the data was transferred.
                break;

            case DRV_AK4384_BUFFER_EVENT_ERROR:

                // Error handling here.
                // We can find out how many bytes were processed in this
                // buffer before the error occurred.

                processedBytes = DRV_AK4384_BufferProcessedSizeGet(myAK4384Handle);

                break;

            default:
                break;
        }
    }
    </code>

  Remarks:
    None.
*/
size_t DRV_AK4384_BufferProcessedSizeGet(DRV_HANDLE handle);

// *****************************************************************************
/*
  Function:
    size_t DRV_AK4384_BufferCombinedQueueSizeGet(DRV_HANDLE handle)

  Summary:
    This function returns the number of bytes queued (to be processed) in the
    buffer queue.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function returns the number of bytes queued (to be processed) in the
    buffer queue associated with the driver instance to which the calling
    client belongs. The client can use this function to know number of bytes
    that is in the queue to be transmitted.

  Precondition:
    The DRV_AK4384_Initialize routine must have been called for the specified
    AK4384 driver instance.

    DRV_AK4384_Open must have been called to obtain a valid opened device handle.

    One of DRV_AK4384_BufferAddRead/DRV_AK4384_BufferAddWrite
    function must have been called and buffers should have been queued for transmission.

  Parameters:
    handle    - Opened client handle associated with a driver object.

  Returns:
    Returns the number of the bytes that have been processed for this buffer.
    Returns 0 for an invalid or an expired client handle.

  Example:
    <code>
    // myAppObj is an application specific object.
    MY_APP_OBJ myAppObj;
	size_t bufferQueuedSize;
    uint8_t mybuffer[MY_BUFFER_SIZE];
    DRV_AK4384_BUFFER_HANDLE bufferHandle;

    // myI2SHandle is the handle returned
    // by the DRV_AK4384_Open function.

    // Client registers an event handler with driver. This is done once

    DRV_AK4384_BufferEventHandlerSet(myAK4384Handle, APP_AK4384BufferEventHandle,
                                                            (uintptr_t)&myAppObj);

    DRV_AK4384_BufferAddRead(myAK4384handle,&bufferHandle,
                                        myBuffer, MY_BUFFER_SIZE);

    if(DRV_AK4384_BUFFER_HANDLE_INVALID == bufferHandle)
    {
        // Error handling here
    }

    // The data is being processed after adding the buffer to the queue.
    // The user can get to know dynamically available data in the queue to be
    // transmitted by calling DRV_AK4384_BufferCombinedQueueSizeGet
    bufferQueuedSize = DRV_AK4384_BufferCombinedQueueSizeGet(myAK4384Handle);

    </code>

  Remarks:
    None.
*/
size_t DRV_AK4384_BufferCombinedQueueSizeGet(DRV_HANDLE handle);


// *****************************************************************************
/*
  Function:
    void DRV_AK4384_BufferQueueFlush(DRV_HANDLE handle)

  Summary:
    This function flushes off the buffers associated with the client object.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function flushes off the buffers associated with the client object and
    disables the DMA channel used for transmission.

  Precondition:
    The DRV_AK4384_Initialize routine must have been called for the specified
    AK4384 driver instance.

    DRV_AK4384_Open must have been called to obtain a valid opened device handle.

    One of DRV_AK4384_BufferAddRead/DRV_AK4384_BufferAddWrite
    function must have been called and buffers should have been queued for transmission.

  Parameters:
    handle    - Opened client handle associated with a driver object.

  Returns:
    None.

  Example:
    <code>
    // myAppObj is an application specific object.
    MY_APP_OBJ myAppObj;
	size_t bufferQueuedSize;
    uint8_t mybuffer[MY_BUFFER_SIZE];
    DRV_AK4384_BUFFER_HANDLE bufferHandle;

    // myAK4384Handle is the handle returned
    // by the DRV_AK4384_Open function.

    // Client registers an event handler with driver. This is done once

    DRV_AK4384_BufferEventHandlerSet(myAK4384Handle, APP_AK4384BufferEventHandle,
                                                            (uintptr_t)&myAppObj);

    DRV_AK4384_BufferAddRead(myAK4384handle,&bufferHandle,
                                        myBuffer, MY_BUFFER_SIZE);

    if(DRV_AK4384_BUFFER_HANDLE_INVALID == bufferHandle)
    {
        // Error handling here
    }

    // The data is being processed after adding the buffer to the queue.
    // The user can stop the data processing and flushoff the data
    // in the queue by calling DRV_AK4384_BufferQueueFlush
    DRV_AK4384_BufferQueueFlush(myAK4384Handle);

    </code>

  Remarks:
    None.
*/
void DRV_AK4384_BufferQueueFlush( const DRV_HANDLE handle);


// *****************************************************************************
/*
  Function:
    void DRV_AK4384_SetAudioCommunicationMode
(
    DRV_HANDLE handle, 
    const DATA_LENGTH dl, 
    const SAMPLE_LENGTH sl
)

  Summary:
    This function provides a run time audio format configuration

  Description:
    This function sets up audio mode in I2S protocol

  Precondition:
    The DRV_AK4384_Initialize routine must have been called for the specified
    AK4384 driver instance.

    DRV_AK4384_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine
    dl           - Data length for I2S audio interface
    sl           - Left/Right Sample Length for I2S audio interface
  Returns:
    None

  Remarks:
    None.
*/
void DRV_AK4384_SetAudioCommunicationMode
(
    DRV_HANDLE handle, 
    const DATA_LENGTH dl, 
    const SAMPLE_LENGTH sl);

// *****************************************************************************
// *****************************************************************************
// Section: AK4384 CODEC Specific Client Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/*
  Function:
    void DRV_AK4384_SamplingRateSet(DRV_HANDLE handle, uint32_t samplingRate)

  Summary:
    This function sets the sampling rate of the media stream.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function sets the media sampling rate for the client handle.

  Precondition:
    The DRV_AK4384_Initialize routine must have been called for the specified
    AK4384 driver instance.

    DRV_AK4384_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine
    baudRate     - Baud Rate to be set

  Returns:
    None.

  Example:
    <code>

    // myAK4384Handle is the handle returned
    // by the DRV_AK4384_Open function.

    DRV_AK4384_SamplingRateSet(myAK4384Handle, 48000);	//Sets 48000 media sampling rate

    </code>

  Remarks:
    None.
 */
void DRV_AK4384_SamplingRateSet(DRV_HANDLE handle, uint32_t samplingRate);


// *****************************************************************************
/*
  Function:
    uint32_t DRV_AK4384_SamplingRateGet(DRV_HANDLE handle)

  Summary:
    This function gets the sampling rate set on the DAC AK4384.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function gets the sampling rate set on the DAC AK4384.

  Precondition:
    The DRV_AK4384_Initialize routine must have been called for the specified
    AK4384 driver instance.

    DRV_AK4384_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None.

  Example:
    <code>
   uint32_t baudRate;

    // myAK4384Handle is the handle returned
    // by the DRV_AK4384_Open function.

    baudRate = DRV_AK4384_SamplingRateGet(myAK4384Handle);

    </code>

  Remarks:
    None.
 */
uint32_t DRV_AK4384_SamplingRateGet(DRV_HANDLE handle);


// *****************************************************************************
/*
  Function:
    void DRV_AK4384_VolumeSet(DRV_HANDLE handle, DRV_AK4384_CHANNEL chan, uint8_t volume)

  Summary:
    This function sets the volume for AK4384 Codec.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This functions sets the volume value from 0-255, which can attenuate
    from 0 dB to –48 dB and mute.

  Precondition:
    The DRV_AK4384_Initialize routine must have been called for the specified
    AK4384 driver instance.

    DRV_AK4384_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine
    chan         - Audio channel volume to be set
    volume       - volume value from 0-255, which can attenuate
                   from 0 dB to –48 dB and mute
  Returns:
    None.

  Example:
    <code>
    // myAppObj is an application specific object.
    MY_APP_OBJ myAppObj;

    // myAK4384Handle is the handle returned
    // by the DRV_AK4384_Open function.

        DRV_AK4384_VolumeSet(myAK4384Handle, DRV_AK4384_CHANNEL_LEFT_RIGHT, 120); //Step 120 volume

    </code>

  Remarks:
    None.
 */
void DRV_AK4384_VolumeSet(DRV_HANDLE handle, DRV_AK4384_CHANNEL chan, uint8_t volume);


// *****************************************************************************
/*
  Function:
    uint8_t DRV_AK4384_VolumeGet(DRV_HANDLE handle, DRV_AK4384_CHANNEL chan)

  Summary:
    This function gets the volume for AK4384 Codec.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This functions gets the current volume programmed to the DAC AK4384.

  Precondition:
    The DRV_AK4384_Initialize routine must have been called for the specified
    AK4384 driver instance.

    DRV_AK4384_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine
    chan         - Audio channel volume to get.

  Returns:
    None.

  Example:
    <code>
    // myAppObj is an application specific object.
    MY_APP_OBJ myAppObj;
    uint8_t volume;

    // myAK4384Handle is the handle returned
    // by the DRV_AK4384_Open function.

      volume = DRV_AK4384_VolumeGet(myAK4384Handle, DRV_AK4384_CHANNEL_LEFT_RIGHT);
    </code>

  Remarks:
    None.
 */
uint8_t DRV_AK4384_VolumeGet(DRV_HANDLE handle, DRV_AK4384_CHANNEL chan);

// *****************************************************************************
/*
  Function:
    void DRV_AK4384_MuteOn(DRV_HANDLE handle);

  Summary:
    Allows AK4384 output for soft mute on.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function Enables AK4384 output for soft mute.

  Precondition:
    The DRV_AK4384_Initialize routine must have been called for the specified
    AK4384 driver instance.

    DRV_AK4384_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None.

  Example:
    <code>
    // myAppObj is an application specific object.
    MY_APP_OBJ myAppObj;

    // myAK4384Handle is the handle returned
    // by the DRV_AK4384_Open function.

    DRV_AK4384_MuteOn(myAK4384Handle);	//AK4384 output soft muted

    </code>

  Remarks:
    None.
 */
void DRV_AK4384_MuteOn(DRV_HANDLE handle);


// *****************************************************************************
/*
  Function:
        void DRV_AK4384_MuteOff(DRV_HANDLE handle)

  Summary:
    Disables AK4384 output for soft mute.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function disables AK4384 output for soft mute.

  Precondition:
    The DRV_AK4384_Initialize routine must have been called for the specified
    AK4384 driver instance.

    DRV_AK4384_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None.

  Example:
    <code>
    // myAppObj is an application specific object.
    MY_APP_OBJ myAppObj;

    // myAK4384Handle is the handle returned
    // by the DRV_AK4384_Open function.

        DRV_AK4384_MuteOff(myAK4384Handle);	//AK4384 output soft mute disabled

    </code>

  Remarks:
    None.
 */
void DRV_AK4384_MuteOff(DRV_HANDLE handle);


// *****************************************************************************
/*
  Function:
    void DRV_AK4384_ZeroDetectEnable(DRV_HANDLE handle)

  Summary:
    Enables AK4384 channel-independent zeros detect function.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function enables AK4384 channel-independent zeros detect function.

  Precondition:
    The DRV_AK4384_Initialize routine must have been called for the specified
    AK4384 driver instance.

    DRV_AK4384_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine
  Returns:
    None.

  Example:
    <code>
    // myAppObj is an application specific object.
    MY_APP_OBJ myAppObj;

    // myAK4384Handle is the handle returned
    // by the DRV_AK4384_Open function.

    DRV_AK4384_ZeroDetectEnable(myAK4384Handle);

    </code>

  Remarks:
    None.
 */
void DRV_AK4384_ZeroDetectEnable(DRV_HANDLE handle);


// *****************************************************************************
/*
  Function:
    void DRV_AK4384_ZeroDetectDisable(DRV_HANDLE handle)

  Summary:
    Disables AK4384 channel-independent zeros detect function.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function disables AK4384 channel-independent zeros detect function.

  Precondition:
    The DRV_AK4384_Initialize routine must have been called for the specified
    AK4384 driver instance.

    DRV_AK4384_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine
  Returns:
    None.

  Example:
    <code>
    // myAppObj is an application specific object.
    MY_APP_OBJ myAppObj;

    // myAK4384Handle is the handle returned
    // by the DRV_AK4384_Open function.

    DRV_AK4384_ZeroDetectDisable(myAK4384Handle);

    </code>

  Remarks:
    None.
 */
void DRV_AK4384_ZeroDetectDisable(DRV_HANDLE handle);


// *****************************************************************************
/*
  Function:
        void DRV_AK4384_ZeroDetectModeSet
        (
            DRV_HANDLE handle,
            DRV_AK4384_ZERO_DETECT_MODE zdMode
        )

  Summary:
    Sets mode of AK4384 channel-independent zeros detect function.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function sets mode of AK4384 channel-independent zeros detect function

  Precondition:
    The DRV_AK4384_Initialize routine must have been called for the specified
    AK4384 driver instance.

    DRV_AK4384_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine
    zdMode	 - Specifies zero detect function mode.

  Returns:
    None.

  Example:
    <code>
    // myAppObj is an application specific object.
    MY_APP_OBJ myAppObj;

    // myAK4384Handle is the handle returned
    // by the DRV_AK4384_Open function.

    DRV_AK4384_ZeroDetectModeSet(myAK4384Handle, DRV_AK4384_ZERO_DETECT_MODE_ANDED);

    </code>

  Remarks:
    None.
 */
void DRV_AK4384_ZeroDetectModeSet
(
    DRV_HANDLE handle,
    DRV_AK4384_ZERO_DETECT_MODE zdMode
);


// *****************************************************************************
/*
  Function:
    void DRV_AK4384_ZeroDetectInvertEnable(DRV_HANDLE handle)

  Summary:
    Enables inversion of polarity for zero detect function.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function enables inversion of polarity for zero detect function.
    DZF goes “L” at Zero Detection

  Precondition:
    The DRV_AK4384_Initialize routine must have been called for the specified
    AK4384 driver instance.

    DRV_AK4384_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None.

  Example:
    <code>
    // myAppObj is an application specific object.
    MY_APP_OBJ myAppObj;

    // myAK4384Handle is the handle returned
    // by the DRV_AK4384_Open function.

    DRV_AK4384_ZeroDetectInvertEnable(myAK4384Handle);

    </code>

  Remarks:
    None.
 */
void DRV_AK4384_ZeroDetectInvertEnable(DRV_HANDLE handle);


// *****************************************************************************
/*
  Function:
    void DRV_AK4384_ZeroDetectInvertDisable(DRV_HANDLE handle)

  Summary:
    Disables inversion of polarity for zero detect function.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function disables inversion of polarity for zero detect function.
    DZF goes “H” at Zero Detection.

  Precondition:
    The DRV_AK4384_Initialize routine must have been called for the specified
    AK4384 driver instance.

    DRV_AK4384_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None.

  Example:
    <code>
    // myAppObj is an application specific object.
    MY_APP_OBJ myAppObj;

    // myAK4384Handle is the handle returned
    // by the DRV_AK4384_Open function.

    DRV_AK4384_ZeroDetectInvertDisable(myAK4384Handle);

    </code>

  Remarks:
    None.
 */
void DRV_AK4384_ZeroDetectInvertDisable(DRV_HANDLE handle);


// *****************************************************************************
/*
  Function:
    void DRV_AK4384_ChannelOutputInvertEnable(DRV_HANDLE handle, DRV_AK4384_CHANNEL chan)

  Summary:
    Enables output polarity of the selected channel.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function enables output polarity of the selected channel.

  Precondition:
    The DRV_AK4384_Initialize routine must have been called for the specified
    AK4384 driver instance.

    DRV_AK4384_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine
    chan         - Left or Right channel

  Returns:
    None.

  Example:
    <code>
    // myAppObj is an application specific object.
    MY_APP_OBJ myAppObj;

    // myAK4384Handle is the handle returned
    // by the DRV_AK4384_Open function.

    DRV_AK4384_ChannelOutputInvertEnable(myAK4384Handle, DRV_AK4384_CHANNEL_LEFT);

    </code>

  Remarks:
    None.
 */
void DRV_AK4384_ChannelOutputInvertEnable(DRV_HANDLE handle, DRV_AK4384_CHANNEL chan);


// *****************************************************************************
/*
  Function:
    void DRV_AK4384_ChannelOutputInvertDisable(DRV_HANDLE handle, DRV_AK4384_CHANNEL chan)

  Summary:
    Disables output polarity of the selected Channel.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function disables output polarity of the selected Channel.

  Precondition:
    The DRV_AK4384_Initialize routine must have been called for the specified
    AK4384 driver instance.

    DRV_AK4384_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine
    chan         - Left or Right channel

  Returns:
    None.

  Example:
    <code>
    // myAppObj is an application specific object.
    MY_APP_OBJ myAppObj;

    // myAK4384Handle is the handle returned
    // by the DRV_AK4384_Open function.

    DRV_AK4384_ChannelOutputInvertDisable(myAK4384Handle, DRV_AK4384_CHANNEL_LEFT);

    </code>

  Remarks:
    None.
 */
void DRV_AK4384_ChannelOutputInvertDisable(DRV_HANDLE handle, DRV_AK4384_CHANNEL chan);


// *****************************************************************************
/*
  Function:
    void DRV_AK4384_SlowRollOffFilterEnable(DRV_HANDLE handle);

  Summary:
    Enables Slow Roll-off filter function.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function enables Slow Roll-off filter function.

  Precondition:
    The DRV_AK4384_Initialize routine must have been called for the specified
    AK4384 driver instance.

    DRV_AK4384_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None.

  Example:
    <code>
    // myAppObj is an application specific object.
    MY_APP_OBJ myAppObj;

    // myAK4384Handle is the handle returned
    // by the DRV_AK4384_Open function.

    DRV_AK4384_SlowRollOffFilterEnable(myAK4384Handle);

    </code>

  Remarks:
    None.
 */
void DRV_AK4384_SlowRollOffFilterEnable(DRV_HANDLE handle);


// *****************************************************************************
/*
  Function:
    void DRV_AK4384_SlowRollOffFilterDisable(DRV_HANDLE handle);

  Summary:
    Disables Slow Roll-off filter function.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function disables Slow Roll-off filter function. Sharp Roll-off filter
    function gets enabled.

  Precondition:
    The DRV_AK4384_Initialize routine must have been called for the specified
    AK4384 driver instance.

    DRV_AK4384_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None.

  Example:
    <code>
    // myAppObj is an application specific object.
    MY_APP_OBJ myAppObj;

    // myAK4384Handle is the handle returned
    // by the DRV_AK4384_Open function.

    DRV_AK4384_SlowRollOffFilterDisable(myAK4384Handle);

    </code>

  Remarks:
    None.
 */
void DRV_AK4384_SlowRollOffFilterDisable(DRV_HANDLE handle);



// *****************************************************************************
/*
  Function:
        void DRV_AK4384_DeEmphasisFilterSet
        (
            DRV_HANDLE handle,
            DRV_AK4384_DEEMPHASIS_FILTER filter
        )

  Summary:
    Allows specifies enabling of digital de-emphasis filter.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
        This function allows specifies enabling of digital de-emphasis for 32, 44.1 or
        48 kHz sampling rates (tc = 50/15 µs)

  Precondition:
    The DRV_AK4384_Initialize routine must have been called for the specified
    AK4384 driver instance.

    DRV_AK4384_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine
        filter	 	- Specifies Enable of de-emphasis filter

  Returns:
    None.

  Example:
    <code>
    // myAppObj is an application specific object.
    MY_APP_OBJ myAppObj;

    // myAK4384Handle is the handle returned
    // by the DRV_AK4384_Open function.

    DRV_AK4384_DeEmphasisFilterSet(myAK4384Handle, DRV_AK4384_DEEMPHASIS_FILTER_44_1KHZ)

    </code>

  Remarks:
    None.
 */
void DRV_AK4384_DeEmphasisFilterSet
(
    DRV_HANDLE handle,
    DRV_AK4384_DEEMPHASIS_FILTER filter
);


// *****************************************************************************
/*
  Function:
        void DRV_AK4384_CommandEventHandlerSet
        (
            DRV_HANDLE handle,
            const DRV_AK4384_COMMAND_EVENT_HANDLER eventHandler,
            const uintptr_t contextHandle
        )

  Summary:
    This function allows a client to identify a command event handling function
    for the driver to call back when the last submitted command have finished.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function allows a client to identify a command event handling function
    for the driver to call back when the last submitted command have finished.

    When a client calls DRV_AK4384_BufferAddWrite function, it is provided with
    a handle identifying  the buffer that was added to the driver's buffer queue.
    The driver will pass this handle back to the client by calling "eventHandler"
    function when the buffer transfer has completed.

    The event handler should be set before the client performs any "AK4384 CODEC
    Specific Client Routines" operations that could generate events.
    The event handler once set, persists until the client closes the driver or
    sets another event handler (which could be a "NULL" pointer to indicate no callback).

  Precondition:
    The DRV_AK4384_Initialize routine must have been called for the specified
    AK4384 driver instance.

    DRV_AK4384_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine
    eventHandler - Pointer to the event handler function.
    context      - The value of parameter will be passed back to the client
                   unchanged, when the eventHandler function is called.  It can
                   be used to identify any client specific data object that
                   identifies the instance of the client module (for example,
                   it may be a pointer to the client module's state structure).

  Returns:
    None.

  Example:
    <code>
    MY_APP_OBJ myAppObj;

    // myAK4384Handle is the handle returned
    // by the DRV_AK4384_Open function.

    // Client registers an event handler with driver

    DRV_AK4384_CommandEventHandlerSet(myAK4384Handle,
                    APP_AK4384CommandEventHandler, (uintptr_t)&myAppObj);

    DRV_AK4384_DeEmphasisFilterSet(myAK4384Handle, DRV_AK4384_DEEMPHASIS_FILTER_44_1KHZ)

    // Event is received when
    // the buffer is processed.

    void APP_AK4384CommandEventHandler(uintptr_t contextHandle)
    {
        // contextHandle points to myAppObj.

        switch(event)
        {
                // Last Submitted command is completed.
                // Perform further processing here
        }
    }
    </code>

  Remarks:
    If the client does not want to be notified when the command
    has completed, it does not need to register a callback.
 */
void DRV_AK4384_CommandEventHandlerSet
(
    DRV_HANDLE handle,
    const DRV_AK4384_COMMAND_EVENT_HANDLER eventHandler,
    const uintptr_t contextHandle
);

// *****************************************************************************
// *****************************************************************************
// Section: AK4384 CODEC Version Information Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/*
  Function:
    int8_t* DRV_AK4384_VersionStrGet(void)

  Summary:
    Returns the version of AK4384 driver in string format.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    The DRV_AK4384_VersionStrGet function returns a string in the format:
    "<major>.<minor>[.<patch>][<type>]"
    Where:
        <major> is the AK4384 driver's version number.
        <minor> is the AK4384 driver's version number.
        <patch> is an optional "patch" or "dot" release number (which is not
        included in the string if it equals '00').
        <type> is an optional release type ('a' for alpha, 'b' for beta
        not the entire word spelled out) that is not included if the release
        is a production version (i.e., not an alpha or beta).

        The String does not contain any spaces.

        Example:
        "0.03a"
        "1.00"

  Precondition:
    None.

  Parameters:
    None.

  Returns: returns a string containing the version of AK4384 driver.

  Example:
    <code>
        int8_t *ak4384string;
        ak4384string = DRV_AK4384_VersionStrGet();
    </code>

  Remarks:
    None.
 */
int8_t* DRV_AK4384_VersionStrGet(void);


// *****************************************************************************
/*
  Function:
    uint32_t DRV_AK4384_VersionGet( void )

  Summary:
    Returns the version of the AK4384 driver.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    The version number returned from the DRV_AK4384_VersionGet function is an
    unsigned integer in the following decimal format.
    <major> * 10000 + <minor> * 100 + <patch>

    Where the numbers are represented in decimal and the meaning is the same as
    above.  Note that there is no numerical representation of release type.

    Example:
    For version "0.03a", return:  0 * 10000 + 3 * 100 + 0
    For version "1.00", return:  1 * 100000 + 0 * 100 + 0

  Parameters:
    None.

  Returns:
    Returns the version of AK4384 driver.

  Example:
    <code>
        uint32_t ak4384version;
        ak4384version = DRV_AK4384_VersionGet();
    </code>

  Remarks:
    None.
 */
uint32_t DRV_AK4384_VersionGet(void);


#endif // #ifndef _DRV_AK4384_H
/*******************************************************************************
 End of File
 */

