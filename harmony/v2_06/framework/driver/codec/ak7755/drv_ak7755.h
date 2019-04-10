/*******************************************************************************
  AK7755 CODEC Driver Interface

  Company:
    Microchip Technology Inc.

  File Name:
    drv_ak7755.h

  Summary:
    AK7755 CODEC Driver Interface header file

  Description:
    The AK7755 CODEC device driver interface provides a simple interface to
    manage the AK7755 16/24-Bit Codec that can be interfaced
    Microchip Microcontroller. This file provides the
    interface definition for the AK7755 Codec device driver.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2016 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _DRV_AK7755_H
#define _DRV_AK7755_H

// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************
#include "system/int/sys_int.h"
#include "system/ports/sys_ports.h"
#include "driver/driver_common.h"
#include "driver/codec/ak7755/drv_codec_i2c_mapping.h"
// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************


#define DRV_I2C_INDEX DRV_AK7755_I2C_INSTANCES_NUMBER

/* Driver AK7755 Module Index

  Summary:
    AK7755 driver index definitions

  Description:
    These constants provide AK7755 Codec Driver index definition.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.
    These values should be passed into the DRV_AK7755_Initialize and
    DRV_AK7755_Open functions to identify the driver instance in use.
 */
 
#define      DRV_AK7755_INDEX_0      0
#define      DRV_AK7755_INDEX_1      1
#define      DRV_AK7755_INDEX_2      2
#define      DRV_AK7755_INDEX_3      3
#define      DRV_AK7755_INDEX_4      4
#define      DRV_AK7755_INDEX_5      5


// *****************************************************************************
/* AK7755 Driver Module Count

  Summary:
    Number of valid AK7755 Codec Driver indices

  Description:
    This constant identifies the maximum number of AK7755 Codec Driver instances that
    should be defined by the application. Defining more instances than this
    constant will waste RAM memory space.

    This constant can also be used by the application to identify the number of
    AK7755 instances on this microcontroller.

  Remarks:
    This value is device-specific.

 */
#define DRV_AK7755_COUNT /*DOM-IGNORE-BEGIN*/ DRV_AK7755_INSTANCES_NUMBER /*DOM-IGNORE-END*/


// *****************************************************************************
/* AK7755 Driver Buffer Handle

  Summary:
    Handle identifying a write buffer passed to the driver.

  Description:
    A buffer handle value is returned by a call to the DRV_AK7755_BufferAddWrite or DRV_AK7755_BufferAddRead function. This handle is associated with the buffer passed 
    into the function and it allows the application to track the completion of the data 
    from (or into) that buffer.  The buffer handle value returned from the "buffer add" 
    function is returned back to the client by the "event handler callback" function
    registered with the driver.

    The buffer handle assigned to a client request expires when the client has
    been notified of the completion of the buffer transfer (after event handler
    function that notifies the client returns) or after the buffer has been
    retired by the driver if no event handler callback was set.

  Remarks:
    None.
 */
typedef uintptr_t DRV_AK7755_BUFFER_HANDLE;


// *****************************************************************************
/* AK7755 Driver Invalid Buffer Handle

  Summary:
    Definition of an invalid buffer handle.

  Description:
    This is the definition of an invalid buffer handle. An invalid buffer handle
    is returned by DRV_AK7755_BufferAddWrite and the DRV_AK7755_BufferAddRead 
    function if the buffer add request was not successful.

  Remarks:
    None.
 */
#define DRV_AK7755_BUFFER_HANDLE_INVALID ((DRV_AK7755_BUFFER_HANDLE)(-1))

// *****************************************************************************
/* AK7755 Driver Events

   Summary:
    Identifies the possible events that can result from a buffer add request.

   Description:
    This enumeration identifies the possible events that can result from a
    buffer add request caused by the client calling either the
    DRV_AK7755_BufferAddWrite or the DRV_AK7755_BufferAddRead function.

   Remarks:
    One of these values is passed in the "event" parameter of the event
    handling callback function that the client registered with the driver by
    calling the DRV_AK7755_BufferEventHandlerSet function when a buffer
    transfer request is completed.

 */
typedef enum {
    /* Data was transferred successfully. */
    DRV_AK7755_BUFFER_EVENT_COMPLETE,

    /* Error while processing the request */
    DRV_AK7755_BUFFER_EVENT_ERROR,

    /* Data transfer aborted (Applicable in DMA mode) */
    DRV_AK7755_BUFFER_EVENT_ABORT

} DRV_AK7755_BUFFER_EVENT;

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
/* AK7755 Driver Buffer Event Handler Function
  Summary:
    Pointer to a AK7755 Driver Buffer Event handler function.

  Description:
    This data type defines the required function signature for the AK7755 Codec Driver
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
    void APP_MyBufferEventHandler( DRV_AK7755_BUFFER_EVENT event,
                                   DRV_AK7755_BUFFER_HANDLE bufferHandle,
                                   uintptr_t context )
    {
        MY_APP_DATA_STRUCT pAppData = (MY_APP_DATA_STRUCT) context;

        switch(event)
        {
            case DRV_AK7755_BUFFER_EVENT_COMPLETE:
                // Handle the completed buffer.
            break;

            case DRV_AK7755_BUFFER_EVENT_ERROR:
            default:
                // Handle error.
            break;
        }
    }
    </code>

   Remarks:
    If the event is DRV_AK7755_BUFFER_EVENT_COMPLETE, this means that the data
    was transferred successfully.

    If the event is DRV_AK7755_BUFFER_EVENT_ERROR, this means that the data was
    not transferred successfully. The bufferHandle parameter contains the buffer
    handle of the buffer that failed. The DRV_AK7755_BufferProcessedSizeGet function
    can be called to find out how many bytes were processed.

    The bufferHandle parameter contains the buffer handle of the buffer that
    associated with the event.

    The context parameter contains a handle to the client context,
    provided at the time the event handling function was  registered using the
    DRV_AK7755_BufferEventHandlerSet function.  This context handle value is
    passed back to the client as the "context" parameter.  It can be any value
    necessary to identify the client context or instance (such as a pointer to
    the client's data) instance of the client that made the buffer add request.

    The buffer handle in bufferHandle expires after this event handler exits. In
    that the buffer object that was allocated is deallocated by the driver
    after the event handler exits.

    The event handler function executes in the data driver (i.e., I2S) peripheral's interrupt
    context when the driver is configured for interrupt mode operation. It is
    recommended of the application to not perform process intensive or blocking
    operations with in this function.

    DRV_AK7755_BufferAddWrite function can be called in the event handler
    to add a buffer to the driver queue.

 */
typedef void (*DRV_AK7755_BUFFER_EVENT_HANDLER) 
                  (DRV_AK7755_BUFFER_EVENT event,
                  DRV_AK7755_BUFFER_HANDLE bufferHandle, 
                  uintptr_t contextHandle);


// *****************************************************************************
/* AK7755 Driver Command Event Handler Function

  Summary:
    Pointer to a AK7755 Codec Driver command event handler function.

  Description:
    This data type defines the required function signature for the AK7755 Codec Driver
    command event handling callback function.

    A command is a control instruction to the AK7755 Codec. For example, Mute ON/OFF, 
    Zero Detect Enable/Disable, etc.

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
    void APP_AK7755CommandEventHandler( uintptr_t context )
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
    DRV_AK7755_CommandEventHandlerSet function.  This context handle value is
    passed back to the client as the "context" parameter.  It can be any value
    necessary to identify the client context or instance (such as a pointer to
    the client's data) instance of the client that made the buffer add request.

    The event handler function executes in the control data driver
    interrupt context. It is recommended of the application to not perform
    process intensive or blocking operations with in this function.

 */
typedef void (*DRV_AK7755_COMMAND_EVENT_HANDLER) (uintptr_t contextHandle);


// *****************************************************************************
/* AK7755 Audio Data Format

  Summary:
    Identifies the Serial Audio data interface format.

  Description:
    This enumeration identifies the Serial Audio data interface format.

  Remarks:
 
 */

typedef enum{
    DRV_AK7755_DAC_INPUT_24BITMSB,
    DRV_AK7755_DAC_INPUT_24BITLSB,
    DRV_AK7755_DAC_INPUT_20BITLSB, // not supported
    DRV_AK7755_DAC_INPUT_16BITLSB,
    
}DRV_AK7755_DAC_INPUT_FORMAT;

typedef enum{
    DRV_AK7755_DSP_DIN1_INPUT_24BITMSB,
    DRV_AK7755_DSP_DIN1_INPUT_24BITLSB,
    DRV_AK7755_DSP_DIN1_INPUT_20BITLSB,
    DRV_AK7755_DSP_DIN1_INPUT_16BITLSB,
//  DRV_AK7755_DSP_DIN1_INPUT_8BITuLAWMSB,
//  DRV_AK7755_DSP_DIN1_INPUT_8BITALAWMSB,            
}DRV_AK7755_DSP_DIN1_INPUT_FORMAT;


typedef enum{
    DRV_AK7755_DSP_DOUT1_OUTPUT_24BITMSB,
    DRV_AK7755_DSP_DOUT1_OUTPUT_24BITLSB,
    DRV_AK7755_DSP_DOUT1_OUTPUT_20BITLSB,
    DRV_AK7755_DSP_DOUT1_OUTPUT_16BITLSB,
//  DRV_AK7755_DSP_DOUT1_OUTPUT_8BITuLAWMSB,
//  DRV_AK7755_DSP_DOUT1_OUTPUT_8BITALAWMSB,     
}DRV_AK7755_DSP_DOUT1_OUTPUT_FORMAT;

typedef enum{
    DRV_AK7755_DSP_DOUT4_OUTPUT_24BITMSB,
    DRV_AK7755_DSP_DOUT4_OUTPUT_24BITLSB,
    DRV_AK7755_DSP_DOUT4_OUTPUT_20BITLSB,
    DRV_AK7755_DSP_DOUT4_OUTPUT_16BITLSB, 
}DRV_AK7755_DSP_DOUT4_OUTPUT_FORMAT;
typedef enum{
    DRV_AK7755_BICK_64FS,
    DRV_AK7755_BICK_48FS,
    DRV_AK7755_BICK_32FS,
    DRV_AK7755_BICK_256FS,
}DRV_AK7755_BICK_FS_FORMAT;
typedef enum{
    DRV_AK7755_LRCK_IF_STANDARD,
    DRV_AK7755_LRCK_IF_I2S_COMPATIBLE,
    DRV_AK7755_LRCK_IF_PCM_SHORT_FRAME,
    DRV_AK7755_LRCK_IF_PCM_LONG_FRAME,
}DRV_AK7755_LRCK_IF_FORMAT;

typedef enum{
    DRV_AK7755_DSP_ECHO_CANCELLATION,
    DRV_AK7755_DSP_REGULAR,
}DRV_AK7755_DSP_PROGRAM;


// *****************************************************************************
/* AK7755 Mic Internal / External Input

  Summary:
    Identifies the Mic input source.

  Description:
    This enumeration identifies the Mic input source.

  Remarks:
    None.

 */
typedef enum {

    INT_MIC,

    EXT_MIC,

} DRV_AK7755_INT_EXT_MIC;

// *****************************************************************************
/* AK7755 Mic Mono/Stereo Input

  Summary:
    Identifies the Mic input as Mono/Stereo.

  Description:
    This enumeration identifies the Mic input as Mono/Stereo.

  Remarks:
    None.

 */
typedef enum {
    ALL_ZEROS,
    MONO_RIGHT_CHANNEL,
    MONO_LEFT_CHANNEL,
    STEREO,
} DRV_AK7755_MONO_STEREO_MIC;

// *****************************************************************************
/* AK7755 Audio Channel

  Summary:
    Identifies left/right audio channel.

  Description:
    This enumeration identifies the left/right audio channel.

  Remarks:
    None.
 */
typedef enum {
    DRV_AK7755_CHANNEL_LEFT,

    DRV_AK7755_CHANNEL_RIGHT,

    DRV_AK7755_CHANNEL_LEFT_RIGHT,

    DRV_AK7755_NUMBER_OF_CHANNELS

} DRV_AK7755_CHANNEL;

// *****************************************************************************
/* AK7755 Driver Initialization Data

  Summary:
    Defines the data required to initialize or reinitialize the AK7755 Codec Driver.

  Description:
    This data type defines the data required to initialize or reinitialize the
    AK7755 Codec Driver.

  Remarks:
    None.
 */
typedef struct {
    /* System module initialization */
    SYS_MODULE_INIT moduleInit;

    /* Identifies data module (I2S) driver ID for
       data interface of CODEC */
    SYS_MODULE_INDEX i2sDriverModuleIndex;

    /* Identifies data module (I2C) driver ID for
    control interface of CODEC */
    SYS_MODULE_INDEX i2cDriverModuleIndex;

    /* Sampling rate */
    uint32_t samplingRate;

    /* Volume */
    uint8_t volume;
    /* Identifies the Audio data format */
//    DRV_AK7755_AUDIO_DATA_FORMAT audioDataFormat;


} DRV_AK7755_INIT;


// *****************************************************************************
// *****************************************************************************
// Section: AK7755 Driver Module Interface Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/*   Function:
        SYS_MODULE_OBJ  DRV_AK7755_Initialize
        (
                const SYS_MODULE_INDEX drvIndex,
                const SYS_MODULE_INIT *const init
        );

  Summary:
    Initializes hardware and data for the instance of the AK7755 DAC module

  Description:
    This function initializes the AK7755 Codec Driver instance for the specified driver
    index, making it ready for clients to open and use it. The initialization
    data is specified by the init parameter. The initialization may fail if the
    number of driver objects allocated are insufficient or if the specified
    driver instance is already initialized.

  Precondition:
    DRV_I2S_Initialize must be called before calling this function to initialize
    the data interface of this codec driver.
    DRV_SPI_Initialize must be called if SPI driver is used for handling
    the control interface of this codec driver.

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
    DRV_AK7755_INIT                  init;
    SYS_MODULE_OBJ                  objectHandle;

    init->inUse                           = true;
    init->status                          = SYS_STATUS_BUSY;
    init->numClients                      = 0;
    init->i2sDriverModuleIndex            = ak7755Init->i2sDriverModuleIndex;
    init->i2cDriverModuleIndex            = ak7755Init->i2cDriverModuleIndex;
    init->samplingRate                    = DRV_AK7755_AUDIO_SAMPLING_RATE;
    init->audioDataFormat                 = DRV_AK7755_AUDIO_DATA_FORMAT_MACRO;
    
    init->isInInterruptContext            = false;

    init->commandCompleteCallback = (DRV_AK7755_COMMAND_EVENT_HANDLER)0;
    init->commandContextData = 0;
    init->mclk_multiplier = DRV_AK7755_MCLK_SAMPLE_FREQ_MULTPLIER;


    objectHandle = DRV_AK7755_Initialize(DRV_AK7755_0, (SYS_MODULE_INIT*)init);
    if (SYS_MODULE_OBJ_INVALID == objectHandle)
    {
        // Handle error
    }
    </code>

  Remarks:
    This function must be called before any other AK7755 function is called.

    This function should only be called once during system initialization
    unless DRV_AK7755_Deinitialize is called to deinitialize the driver
    instance. This function will NEVER block for hardware access.

 */
SYS_MODULE_OBJ DRV_AK7755_Initialize(const SYS_MODULE_INDEX drvIndex,
                                     const SYS_MODULE_INIT * const init);

// *****************************************************************************
/* Function:
    void DRV_AK7755_Deinitialize( SYS_MODULE_OBJ object)

  Summary:
    Deinitializes the specified instance of the AK7755 Codec Driver module.

  Description:
    This function deinitializes the specified instance of the AK7755 Codec Driver module, 
    disabling its operation (and any hardware).  Invalidates all the internal data.

  Precondition:
    The DRV_AK7755_Initialize function should have been called before calling this
    function.

  Parameters:
    object          - Driver object handle, returned from the
                      DRV_AK4642_Initialize routine

  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object;     //  Returned from DRV_AK7755_Initialize
    SYS_STATUS          status;

    DRV_AK7755_Deinitialize(object-->);

    status = DRV_AK7755_Status(object);
    if (SYS_MODULE_DEINITIALIZED != status)
    {
        // Check again later if you need to know
        // when the driver is deinitialized.
    }
    </code>

  Remarks:
    Once the Initialize operation has been called, the Deinitialize operation
    must be called before the Initialize operation can be called again. This
    function will NEVER block waiting for hardware.
 */
void DRV_AK7755_Deinitialize(SYS_MODULE_OBJ object);


// *****************************************************************************
/* Function:
    SYS_STATUS DRV_AK7755_Status( SYS_MODULE_OBJ object)

  Summary:
    Gets the current status of the AK7755 Codec Driver module.

  Description:
    This function provides the current status of the AK7755 Codec Driver module.

  Precondition:
    The DRV_AK7755_Initialize function should have been called before calling this
    function.

  Parameters:
    object          - Driver object handle, returned from the
                      DRV_AK4642_Initialize routine

  Returns:
    - SYS_STATUS_DEINITIALIZED  - Indicates that the driver has been
                                  deinitialized

    - SYS_STATUS_READY          - Indicates that any previous module operation
                                  for the specified module has completed

    - SYS_STATUS_BUSY           - Indicates that a previous module operation for
                                  the specified module has not yet completed

    - SYS_STATUS_ERROR          - Indicates that the specified module is in an
                                  error state

  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_AK7755_Initialize
    SYS_STATUS          AK7755Status;

    AK7755Status = DRV_AK7755_Status(object);
    if (SYS_STATUS_READY == AK7755Status)
    {
        // This means the driver can be opened using the
        // DRV_AK7755_Open function.
    }
    </code>

  Remarks:
    A driver can be opened only when its status is SYS_STATUS_READY.
 */
SYS_STATUS DRV_AK7755_Status(SYS_MODULE_OBJ object);


// *****************************************************************************
/* Function:
    void  DRV_AK7755_Tasks(SYS_MODULE_OBJ object);

  Summary:
    Maintains the driver's control and data interface state machine.

  Description:
    This function is used to maintain the driver's internal control and data
    interface state machine and implement its control and data interface
    implementations.
    This function should be called from the SYS_Tasks function.

  Precondition:
    The DRV_AK7755_Initialize function must have been called for the specified
    AK7755 Codec Driver instance.

  Parameters:
    object      - Object handle for the specified driver instance (returned from
                  DRV_AK7755_Initialize)

  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_AK7755_Initialize

    while (true)
    {
        DRV_AK7755_Tasks (object);

        // Do other tasks
    }
    </code>

  Remarks:
    This function is normally not called directly by an application.  It is
    called by the system's Tasks function (SYS_Tasks).

 */
void DRV_AK7755_Tasks(SYS_MODULE_OBJ object);


// *****************************************************************************
// *****************************************************************************
// Section: AK7755 CODEC Driver Client Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    DRV_HANDLE DRV_AK7755_Open
    (
        const SYS_MODULE_INDEX drvIndex,
        const DRV_IO_INTENT ioIntent
    )

  Summary:
    Opens the specified AK7755 Codec Driver instance and returns a handle to it

  Description:
    This function opens the specified AK7755 Codec Driver instance and provides a
    handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver. The ioIntent
    parameter defines how the client interacts with this driver instance.

    The DRV_IO_INTENT_BLOCKING and DRV_IO_INTENT_NONBLOCKING ioIntent
    options are not relevant to this driver. All the data transfer functions
    of this driver are non blocking.

    Only DRV_IO_INTENT_WRITE is a valid ioIntent option as AK7755 is DAC only.

    Specifying a DRV_IO_INTENT_EXCLUSIVE will cause the driver to provide
    exclusive access to this client. The driver cannot be opened by any
    other client.

  Precondition:
    The DRV_AK7755_Initialize function must have been called before calling this
    function.

  Parameters:
    drvIndex    - Identifier for the object instance to be opened

    ioIntent    - Zero or more of the values from the enumeration
                  DRV_IO_INTENT "ORed" together to indicate the intended use
                  of the driver. See function description for details.

  Returns:
    If successful, the function returns a valid open-instance handle (a number
    identifying both the caller and the module instance).

    If an error occurs, the return value is DRV_HANDLE_INVALID. Error can occur
    - if the number of client objects allocated via DRV_AK7755_CLIENTS_NUMBER is insufficient.
    - if the client is trying to open the driver but driver has been opened exclusively by another client.
    - if the driver hardware instance being opened is not initialized or is invalid.
    - if the ioIntent options passed are not relevant to this driver.

  Example:
    <code>
    DRV_HANDLE handle;

    handle = DRV_AK7755_Open(DRV_AK7755_INDEX_0, DRV_IO_INTENT_EXCLUSIVE);
    if (DRV_HANDLE_INVALID == handle)
    {
        // Unable to open the driver
        // May be the driver is not initialized or the initialization
        // is not complete.
    }
    </code>

  Remarks:
    The handle returned is valid until the DRV_AK7755_Close function is called.
    This function will NEVER block waiting for hardware.If the requested intent
    flags are not supported, the function will return DRV_HANDLE_INVALID.  This
    function is thread safe in a RTOS application. It should not be called in an
    ISR.
 */
DRV_HANDLE DRV_AK7755_Open(const SYS_MODULE_INDEX iDriver,
                           const DRV_IO_INTENT ioIntent);

// *****************************************************************************
/* Function:
    void DRV_AK7755_Close( DRV_Handle handle )

  Summary:
    Closes an opened-instance of the AK7755 Codec Driver.

  Description:
    This function closes an opened-instance of the AK7755 Codec Driver, invalidating the
    handle. Any buffers in the driver queue that were submitted by this client
    will be removed.  After calling this function, the handle passed in "handle"
    must not be used with any of the remaining driver functions.  A new handle must
    be obtained by calling DRV_AK7755_Open before the caller may use the driver
    again.

  Precondition:
    The DRV_AK7755_Initialize function must have been called for the specified
    AK7755 Codec Driver instance.

    DRV_AK7755_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

  Returns:
    None.

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_AK7755_Open

    DRV_AK7755_Close(handle);

    </code>

  Remarks:

    Usually there is no need for the driver client to verify that the Close
    operation has completed.  The driver will abort any ongoing operations
    when this function is called.
 */
void DRV_AK7755_Close(const DRV_HANDLE handle);


// *****************************************************************************
/*
Function:
        void DRV_AK7755_BufferAddWrite
        (
            const DRV_HANDLE handle,
            DRV_AK7755_BUFFER_HANDLE *bufferHandle,
            void *buffer, size_t size
        )

  Summary:
    Schedule a non-blocking driver write operation.

  Description:
    This function schedules a non-blocking write operation. The function returns
    with a valid buffer handle in the bufferHandle argument if the write request
    was scheduled successfully. The function adds the request to the hardware
    instance transmit queue and returns immediately. While the request is in the
    queue, the application buffer is owned by the driver and should not be
    modified.  The function returns DRV_AK7755_BUFFER_HANDLE_INVALID:
    - if a buffer could not be allocated to the request
    - if the input buffer pointer is NULL
    - if the buffer size is 0
    - if the queue is full or the queue depth is insufficient
    If the requesting client registered an event callback with the driver,
    the driver will issue a DRV_AK7755_BUFFER_EVENT_COMPLETE event if the buffer
    was processed successfully of DRV_AK7755_BUFFER_EVENT_ERROR event if the
    buffer was not processed successfully.

  Precondition:
    The DRV_AK7755_Initialize function must have been called for the specified
    AK7755 device instance and the DRV_AK7755_Status must have returned
    SYS_STATUS_READY.

    DRV_AK7755_Open must have been called to obtain a valid opened device handle.

    DRV_IO_INTENT_WRITE must have been specified in the DRV_AK7755_Open call.

  Parameters:
    handle       - Handle of the AK7755 instance as return by the
                   DRV_AK7755_Open function.
    buffer       - Data to be transmitted.
    size         - Buffer size in bytes.
    bufferHandle - Pointer to an argument that will contain the
                   return buffer handle.

  Returns:
    The bufferHandle parameter will contain the return buffer handle. This will be
    DRV_AK7755_BUFFER_HANDLE_INVALID if the function was not successful.

  Example:
    <code>

    MY_APP_OBJ myAppObj;
    uint8_t mybuffer[MY_BUFFER_SIZE];
    DRV_AK7755_BUFFER_HANDLE bufferHandle;

    // myAK7755Handle is the handle returned
    // by the DRV_AK7755_Open function.

    // Client registers an event handler with driver

    DRV_AK7755_BufferEventHandlerSet(myAK7755Handle,
                    APP_AK7755BufferEventHandler, (uintptr_t)&myAppObj);

    DRV_AK7755_BufferAddWrite(myAK7755handle, &bufferHandle
                                        myBuffer, MY_BUFFER_SIZE);

    if(DRV_AK7755_BUFFER_HANDLE_INVALID == bufferHandle)
    {
        // Error handling here
    }

    // Event is received when the buffer is processed.

    void APP_AK7755BufferEventHandler(DRV_AK7755_BUFFER_EVENT event,
            DRV_AK7755_BUFFER_HANDLE bufferHandle, uintptr_t contextHandle)
    {
        // contextHandle points to myAppObj.

        switch(event)
        {
            case DRV_AK7755_BUFFER_EVENT_COMPLETE:

                // This means the data was transferred.
                break;

            case DRV_AK7755_BUFFER_EVENT_ERROR:

                // Error handling here.
                break;

            default:
                break;
        }
    }

    </code>

  Remarks:
    This function is thread safe in a RTOS application. It can be called from
    within the AK7755 Driver Buffer Event Handler that is registered by this
    client. It should not be called in the event handler associated with another
    AK7755 Codec Driver instance. It should not otherwise be called directly in an ISR.

 */
void DRV_AK7755_BufferAddWrite(const DRV_HANDLE handle,
                               DRV_AK7755_BUFFER_HANDLE *bufferHandle,
                               void *buffer, size_t size);

// *****************************************************************************
/*
Function:
    void DRV_AK7755_BufferAddRead
    (
        const DRV_HANDLE handle,
        DRV_AK7755_BUFFER_HANDLE *bufferHandle,
        void *buffer, size_t size
    )

  Summary:
    Schedule a non-blocking driver read operation.

  Description:
    This function schedules a non-blocking read operation. The function returns
    with a valid buffer handle in the bufferHandle argument if the read request
    was scheduled successfully. The function adds the request to the hardware
    instance receive queue and returns immediately. While the request is in the
    queue, the application buffer is owned by the driver and should not be
    modified.  The function returns DRV_AK7755_BUFFER_HANDLE_INVALID
    - if a buffer could not be allocated to the request
    - if the input buffer pointer is NULL
    - if the buffer size is 0.
    - if the queue is full or the queue depth is insufficient
    If the requesting client registered an event callback with the driver,
    the driver will issue a DRV_AK7755_BUFFER_EVENT_COMPLETE event if the buffer
    was processed successfully of DRV_AK7755_BUFFER_EVENT_ERROR event if the
    buffer was not processed successfully.

  Precondition:
    The DRV_AK7755_Initialize function must have been called for the specified
    AK7755 device instance and the DRV_AK7755_Status must have returned
    SYS_STATUS_READY.

    DRV_AK7755_Open must have been called to obtain a valid opened device handle.

    DRV_IO_INTENT_READ must have been specified in the DRV_AK7755_Open call.

  Parameters:
    handle       - Handle of the AK7755 instance as return by the
                   DRV_AK7755_Open function.
    buffer       - Data to be transmitted.
    size         - Buffer size in bytes.
    bufferHandle - Pointer to an argument that will contain the
                   return buffer handle.

  Returns:
    The bufferHandle parameter will contain the return buffer handle. This will be
    DRV_AK7755_BUFFER_HANDLE_INVALID if the function was not successful.

   Remarks:
    This function is thread safe in a RTOS application. It can be called from
    within the AK7755 Driver Buffer Event Handler that is registered by this
    client. It should not be called in the event handler associated with another
    AK7755 Codec Driver instance. It should not otherwise be called directly in an ISR.

*/
void DRV_AK7755_BufferAddRead(const DRV_HANDLE handle,
                               DRV_AK7755_BUFFER_HANDLE *bufferHandle,
                               void *buffer, size_t size);

void DRV_AK7755_BufferAddWriteRead(const DRV_HANDLE handle,
                                   DRV_AK7755_BUFFER_HANDLE    *bufferHandle,
                                   void *transmitBuffer, void *receiveBuffer,
                                   size_t size);

// *****************************************************************************
/*
  Function:
        void DRV_AK7755_BufferEventHandlerSet
        (
            DRV_HANDLE handle,
            const DRV_AK7755_BUFFER_EVENT_HANDLER eventHandler,
            const uintptr_t contextHandle
        )

  Summary:
    This function allows a client to identify a buffer event handling function
    for the driver to call back when queued buffer transfers have finished.

  Description:
    This function allows a client to identify a buffer event handling function
    for the driver to call back when queued buffer transfers have finished.
    When a client calls DRV_AK7755_BufferAddWrite function, it is provided with
    a handle identifying  the buffer that was added to the driver's buffer queue.
    The driver will pass this handle back to the client by calling "eventHandler"
    function when the buffer transfer has completed.

    The event handler should be set before the client performs any "buffer add"
    operations that could generate events. The event handler once set, persists
    until the client closes the driver or sets another event handler (which
    could be a "NULL" pointer to indicate no callback).

  Precondition:
    The DRV_AK7755_Initialize function must have been called for the specified
    AK7755 Codec Driver instance.

    DRV_AK7755_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function
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
    DRV_AK7755_BUFFER_HANDLE bufferHandle;

    // myAK7755Handle is the handle returned
    // by the DRV_AK7755_Open function.

    // Client registers an event handler with driver

    DRV_AK7755_BufferEventHandlerSet(myAK7755Handle,
                    APP_AK7755BufferEventHandler, (uintptr_t)&myAppObj);

    DRV_AK7755_BufferAddWrite(myAK7755handle, &bufferHandle
                                        myBuffer, MY_BUFFER_SIZE);

    if(DRV_AK7755_BUFFER_HANDLE_INVALID == bufferHandle)
    {
        // Error handling here
    }

    // Event is received when
    // the buffer is processed.

    void APP_AK7755BufferEventHandler(DRV_AK7755_BUFFER_EVENT event,
            DRV_AK7755_BUFFER_HANDLE bufferHandle, uintptr_t contextHandle)
    {
        // contextHandle points to myAppObj.

        switch(event)
        {
            case DRV_AK7755_BUFFER_EVENT_COMPLETE:

                // This means the data was transferred.
                break;

            case DRV_AK7755_BUFFER_EVENT_ERROR:

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
void DRV_AK7755_BufferEventHandlerSet(DRV_HANDLE handle,
                                      const DRV_AK7755_BUFFER_EVENT_HANDLER 
                                                eventHandler,
                                      const uintptr_t contextHandle);

// *****************************************************************************
// *****************************************************************************
// Section: AK7755 CODEC Specific Client Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/*
  Function:
    void DRV_AK7755_SamplingRateSet(DRV_HANDLE handle, uint32_t samplingRate)

  Summary:
    This function sets the sampling rate of the media stream.

  Description:
    This function sets the media sampling rate for the client handle.

  Precondition:
    The DRV_AK7755_Initialize function must have been called for the specified
    AK7755 Codec Driver instance.

    DRV_AK7755_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function
    samplingRate - Sampling frequency in Hz
    
  Returns:
    None.

  Example:
    <code>
    // myAK7755Handle is the handle returned
    // by the DRV_AK7755_Open function.

    DRV_AK7755_SamplingRateSet(myAK7755Handle, 48000);    //Sets 48000 media sampling rate

    </code>

  Remarks:
    None.
 */
void DRV_AK7755_SamplingRateSet(DRV_HANDLE handle, uint32_t samplingRate);

// *****************************************************************************
/*
  Function:
    uint32_t DRV_AK7755_SamplingRateGet(DRV_HANDLE handle)

  Summary:
    This function gets the sampling rate set on the AK7755.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function gets the sampling rate set on the DAC AK7755.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function
                   
  Example:
    <code>
   uint32_t baudRate;

    // myAK7755Handle is the handle returned
    // by the DRV_AK7755_Open function.

    baudRate = DRV_AK7755_SamplingRateGet(myAK7755Handle);

    </code>

  Remarks:
    None.
 */
uint32_t DRV_AK7755_SamplingRateGet(DRV_HANDLE handle);

// *****************************************************************************
/*
  Function:
    void DRV_AK7755_VolumeSet(DRV_HANDLE handle, DRV_AK7755_CHANNEL channel, uint8_t volume);

  Summary:
    This function sets the volume for AK7755 CODEC.

  Description:
    This functions sets the volume value from 0-255, which can attenuate
    from -115 dB to +12 dB. All decibels below approximately -50 dB are inaudible.
    
  Precondition:
    The DRV_AK7755_Initialize function must have been called for the specified
    AK7755 Codec Driver instance.

    DRV_AK7755_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   Open function
    channel      - argument indicating Left or Right or Both channel volume to be modified
    volume       - Updated volume specified in the range 0-255

  Returns:
    None

  Example:
    <code>
    // myAppObj is an application specific object.
    MY_APP_OBJ myAppObj;

    uint8_t mybuffer[MY_BUFFER_SIZE];
    DRV_BUFFER_HANDLE bufferHandle;

    // myAK7755Handle is the handle returned
    // by the DRV_AK7755_Open function.

        DRV_AK7755_VolumeSet(myAK7755Handle,DRV_AK7755_CHANNEL_LEFT, 120);    //Step 120 volume

    </code>

  Remarks:
    None.
 */
void DRV_AK7755_VolumeSet(DRV_HANDLE handle, DRV_AK7755_CHANNEL channel, uint8_t volume);

// *****************************************************************************
/*
  Function:
    uint8_t DRV_AK7755_VolumeGet(DRV_HANDLE handle, DRV_AK7755_CHANNEL channel)

  Summary:
    Gets the volume for the AK7755 Codec Driver.

  Description:
    This functions gets the current volume programmed to the AK7755 Codec Driver.

  Precondition:
    The DRV_AK7755_Initialize function must have been called for the specified
    AK7755 Codec Driver instance.

    DRV_AK7755_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function
    channel      - argument indicating Left or Right or Both channel volume to be modified

  Returns:
    None.

  Example:
    <code>
    // myAppObj is an application specific object.
    MY_APP_OBJ myAppObj;
    uint8_t volume;

    // myAK7755Handle is the handle returned
    // by the DRV_AK7755_Open function.

      volume = DRV_AK7755_VolumeGet(myAK7755Handle, DRV_AK7755_CHANNEL_LEFT);
    </code>

  Remarks:
    None.
 */
uint8_t DRV_AK7755_VolumeGet(DRV_HANDLE handle, DRV_AK7755_CHANNEL channel);


// *****************************************************************************
/*
  Function:
    void DRV_AK7755_MuteOn(DRV_HANDLE handle);

  Summary:
    Allows AK7755 output for soft mute on.

  Description:
    This function enables AK7755 output for soft mute.

  Precondition:
    The DRV_AK7755_Initialize function must have been called for the specified
    AK7755 Codec Driver instance.

    DRV_AK7755_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

  Returns:
    None.

  Example:
    <code>
    // myAppObj is an application specific object.
    MY_APP_OBJ myAppObj;

    uint8_t mybuffer[MY_BUFFER_SIZE];
    DRV_BUFFER_HANDLE bufferHandle;

    // myAK7755Handle is the handle returned
    // by the DRV_AK7755_Open function.

    DRV_AK7755_MuteOn(myAK7755Handle);    //AK7755 output soft muted

    </code>

  Remarks:
    None.
 */
void DRV_AK7755_MuteOn(DRV_HANDLE handle);


// *****************************************************************************
/*
  Function:
        void DRV_AK7755_MuteOff(DRV_HANDLE handle)

  Summary:
    Disables AK7755 output for soft mute.

  Description:
    This function disables AK7755 output for soft mute.

  Precondition:
    The DRV_AK7755_Initialize function must have been called for the specified
    AK7755 Codec Driver instance.

    DRV_AK7755_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

  Returns:
    None.

  Example:
    <code>
    // myAppObj is an application specific object.
    MY_APP_OBJ myAppObj;

    uint8_t mybuffer[MY_BUFFER_SIZE];
    DRV_BUFFER_HANDLE bufferHandle;

    // myAK7755Handle is the handle returned
    // by the DRV_AK7755_Open function.

        DRV_AK7755_MuteOff(myAK7755Handle);    //AK7755 output soft mute disabled

    </code>

  Remarks:
    None.
 */
void DRV_AK7755_MuteOff(DRV_HANDLE handle);

// *****************************************************************************
/*
  Function:
    void DRV_AK7755_IntExtMicSet(DRV_HANDLE handle);

  Summary:
    Sets up the codec for the internal or the external microphone use.

  Description:
    This function sets up the codec for the internal or the external microphone use.

  Precondition:
    The DRV_AK7755_Initialize function must have been called for the specified
    AK7755 Codec Driver instance.

    DRV_AK7755_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function
    micInput     - Internal vs. External microphone input
  Returns:
    None.

  Remarks:
    None.
*/
void DRV_AK7755_IntExtMicSet(DRV_HANDLE handle, DRV_AK7755_INT_EXT_MIC micInput);

// *****************************************************************************
/*
  Function:
    void DRV_AK7755_MonoStereoMicSet(DRV_HANDLE handle);

  Summary:
    Sets up the codec for the Mono or Stereo microphone mode.

  Description:
    This function sets up the codec for the Mono or Stereo microphone mode.

  Precondition:
    The DRV_AK7755_Initialize function must have been called for the specified
    AK7755 Codec Driver instance.

    DRV_AK7755_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle          - A valid open-instance handle, returned from the driver's
                   open function
    mono_stereo_mic - Mono/Stereo microphone setup
    
  Returns:
    None.

  Remarks:
    None.
*/
void DRV_AK7755_MonoStereoMicSet(DRV_HANDLE handle, DRV_AK7755_MONO_STEREO_MIC mono_stereo_mic);


// *****************************************************************************
/*
  Function:
    void DRV_AK7755_SetAudioCommunicationMode
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
    The DRV_AK7755_Initialize routine must have been called for the specified
    AK7755 driver instance.

    DRV_AK7755_Open must have been called to obtain a valid opened device handle.

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
void DRV_AK7755_SetAudioCommunicationMode
(
    DRV_HANDLE handle, 
    const DATA_LENGTH dl, 
    const SAMPLE_LENGTH sl);

// *****************************************************************************
/*
  Function:
        void DRV_AK7755_CommandEventHandlerSet
        (
            DRV_HANDLE handle,
            const DRV_AK7755_COMMAND_EVENT_HANDLER eventHandler,
            const uintptr_t contextHandle
        )

  Summary:
    Allows a client to identify a command event handling function
    for the driver to call back when the last submitted command have finished.

  Description:
    This function allows a client to identify a command event handling function
    for the driver to call back when the last submitted command have finished.

    When a client calls DRV_AK7755_BufferAddWrite function, it is provided with
    a handle identifying  the buffer that was added to the driver's buffer queue.
    The driver will pass this handle back to the client by calling "eventHandler"
    function when the buffer transfer has completed.

    The event handler should be set before the client performs any "AK7755 CODEC
    Specific Client Routines" operations that could generate events.
    The event handler once set, persists until the client closes the driver or
    sets another event handler (which could be a "NULL" pointer to indicate no callback).

  Precondition:
    The DRV_AK7755_Initialize function must have been called for the specified
    AK7755 Codec Driver instance.

    DRV_AK7755_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function
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
    DRV_AK7755_BUFFER_HANDLE bufferHandle;

    // myAK7755Handle is the handle returned
    // by the DRV_AK7755_Open function.

    // Client registers an event handler with driver

    DRV_AK7755_CommandEventHandlerSet(myAK7755Handle,
                    APP_AK7755CommandEventHandler, (uintptr_t)&myAppObj);

    DRV_AK7755_DeEmphasisFilterSet(myAK7755Handle, DRV_AK7755_DEEMPHASIS_FILTER_44_1KHZ)

    // Event is received when
    // the buffer is processed.

    void APP_AK7755CommandEventHandler(uintptr_t contextHandle)
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
void DRV_AK7755_CommandEventHandlerSet(DRV_HANDLE handle,
                                       const DRV_AK7755_COMMAND_EVENT_HANDLER 
                                                eventHandler,
                                       const uintptr_t contextHandle);

// *****************************************************************************
// *****************************************************************************
// Section: AK7755 CODEC Version Information Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/*
  Function:
    int8_t* DRV_AK7755_VersionStrGet(void)

  Summary:
    This function returns the version of AK7755 Codec Driver in string format.

  Description:
    The DRV_AK7755_VersionStrGet function returns a string in the format:
    "<major>.<minor>[.<patch>][<type>]"
    Where:
        * <major> is the AK7755 Codec Driver's version number.
        * <minor> is the AK7755 Codec Driver's version number.
        * <patch> is an optional "patch" or "dot" release number (which is not
        included in the string if it equals "00").
        * <type> is an optional release type ("a" for alpha, "b" for beta ?
        not the entire word spelled out) that is not included if the release
        is a production version (I.e. Not an alpha or beta).

        The String does not contain any spaces. For example, 
        "0.03a"
        "1.00"
    
  Precondition:
    None.

  Parameters:
    None.

  Returns: returns a string containing the version of the AK7755 Codec Driver.

  Example:
    <code>
        int8_t *AK7755string;
        AK7755string = DRV_AK7755_VersionStrGet();
    </code>

  Remarks:
    None
 */
int8_t* DRV_AK7755_VersionStrGet(void);


// *****************************************************************************
/*
  Function:
    uint32_t DRV_AK7755_VersionGet( void )

  Summary:
    Returns the version of the AK7755 Codec Driver.

  Description:
    The version number returned from the DRV_AK7755_VersionGet function is an
    unsigned integer in the following decimal format:
    * <major> * 10000 + <minor> * 100 + <patch>

    Where the numbers are represented in decimal and the meaning is the same as
    above.  Note that there is no numerical representation of release type.

    Example:
    * For version "0.03a", return:  0 * 10000 + 3 * 100 + 0
    * For version "1.00", return:  1 * 100000 + 0 * 100 + 0


  Precondition:
    None.

  Parameters:
    None.

  Returns: 
    Returns the version of the AK7755 Codec Driver.

  Example:
    <code>
        uint32_t AK7755version;
        AK7755version = DRV_AK7755_VersionGet();
    </code>

  Remarks:
    None.
 */
uint32_t DRV_AK7755_VersionGet(void);


#endif // #ifndef _DRV_AK7755_H
/*******************************************************************************
 End of File
 */
