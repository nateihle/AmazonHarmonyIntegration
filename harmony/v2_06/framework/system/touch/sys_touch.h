/*******************************************************************************
  Touch System Service Implementation

  Company:
    Microchip Technology Inc.

  File Name:
    sys_touch.c

  Summary:
    Touch System Service Implementation.

  Description:
    The Touch System Service provides a simple interface to manage the touch
    screen drivers. This file implements the core interface routines
    for the Touch System Service.
	This is a resistive touch screen driver that is using the 
	Microchip Graphics Library. The calibration values are 
	automatically checked (by reading a specific memory location
	on the non-volatile memory) when initializing the module if the 
	function pointers to the read and write callback functions 
	are initialized. If the read value is invalid calibration 
	will automatically be executed. Otherwise, the calibration
	values will be loaded and used.
	The driver assumes that the application side provides the 
	read and write routines to a non-volatile memory. 
	If the callback functions are not initialized, the calibration
	routine will always be called at startup to initialize the
	global calibration values.
	This driver assumes that the Graphics Library is initialized 
	and will be using the default font of the library.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014-2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute Software
only when embedded on a Microchip microcontroller or digital  signal  controller
that is integrated into your product or third party  product  (pursuant  to  the
sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED 'AS IS'  WITHOUT  WARRANTY  OF  ANY  KIND,
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


#ifndef _SYS_TOUCH_H
#define _SYS_TOUCH_H

#include "system_definitions.h"
#include "driver/touch/drv_touch.h"

#ifdef __cplusplus
    extern "C" {
#endif
        
typedef uint16_t (*NVM_READ_FUNC)(uint32_t);           // typedef for read function pointer
typedef void (*NVM_WRITE_FUNC)(uint16_t, uint32_t);    // typedef for write function pointer
typedef void (*NVM_SECTORERASE_FUNC)(uint32_t);    // typedef for sector erase function pointer


// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
/*
  SYS TOUCH Module Index Numbers

  Summary:
    Identifies the Touch System Service module index definitions.

  Description:
    These constants provide the Touch System Service index definitions.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.

    These values should be passed into the SYS_TOUCH_Initialize and
    function to identify the driver instance in use.
*/
typedef enum
{
    /* First Touch service instance */
    SYS_TOUCH_INDEX_0,
    /* Second Touch service */
    SYS_TOUCH_INDEX_1

} SYS_TOUCH_INDEX;

// *****************************************************************************
/* Touch System Service handle

  Summary:
    Handle for requested Touch interface.

  Description:
    A Touch handle is returned by a call to the SYS_TOUCH_Open
    function. This handle is an identification for the
    associated touch interface.

    The handle once assigned to a client expires when the
    client calls SYS_TOUCH_Close.

  Remarks:
    None.
*/
typedef uintptr_t SYS_TOUCH_HANDLE;

// *****************************************************************************
/* Invalid Touch Handle

 Summary:
    Invalid touch handle.

 Description:
    If the touch system service is unable to allow an additional clients to use it,
    it must then return the special value SYS_TOUCH_HANDLE_INVALID.  Callers
    should check the handle returned for this value to ensure this value was not
    returned before attempting to call any other driver routines using the handle.

 Remarks:
    None.
*/

#define SYS_TOUCH_HANDLE_INVALID  (((SYS_TOUCH_HANDLE) -1))

// *****************************************************************************
/*  Input Device Type: SYS_INPUT_DEVICE_TYPE

    Summary:
      Specifies the different user input devices supported in the library.

    Description:
      This enumeration specifies the different user input devices
      supported in the library.

    Remarks:
      Beta. The enum will be relocated to SYS_INPUT when available.

*/
// *****************************************************************************
typedef enum
{
    TYPE_UNKNOWN    = 0,            // Unknown device.
    TYPE_KEYBOARD,                  // Keyboard.
    TYPE_TOUCHSCREEN,               // Touchscreen.
// DOM-IGNORE-BEGIN
    TYPE_MOUSE,                     // Mouse.
    TYPE_TIMER,                     // Timer.
    TYPE_SYSTEM                     // System Messages.
// DOM-IGNORE-END
} SYS_INPUT_DEVICE_TYPE;

// *****************************************************************************
/*  Input device event: INPUT_DEVICE_EVENT

    Summary:
        Specifies the different user input device events supported
        in the library.

    Description:
        This enumeration specifies the different user input device events
        supported in the graphics library.

    Remarks:
        None.

*/
// *****************************************************************************
typedef enum
{
    EVENT_INVALID   = 0,            // Invalid event.
    EVENT_MOVE,                     // Move event.
    EVENT_PRESS,                    // Press event.
    EVENT_STILLPRESS,               // Continuous press event.
    EVENT_RELEASE,                  // Release event.
    EVENT_KEYSCAN,                  // Key scan event, parameters for the object
                                    // ID and keyboard scan code will be sent
                                    // with this event in the GFX_GOL_MESSAGE
                                    // as parameter.
    EVENT_CHARCODE,                 // Character code event. The actual
                                    // character code will be sent with this
                                    // event in the GFX_GOL_MESSAGE as
                                    // parameter.
    EVENT_SET,                      // Generic set event.
    EVENT_SET_STATE,                // Generic set state event.
    EVENT_CLR_STATE                 // Generic clear state event.
} SYS_INPUT_DEVICE_EVENT;


// *****************************************************************************
/* Touch System Service status

  Summary:
    Identifies the current status/state of touch.

  Description:
    Identifies the current status/state of touch.

  Remarks:
    This enumeration is the return type for the status routine.
*/
typedef enum
{
    /*An unspecified error has occurred.*/
    SYS_TOUCH_ERROR  = -1,

    // The module has not yet been initialized
    SYS_TOUCH_UNINITIALIZED = 0,

    // An operation is currently in progress
    SYS_TOUCH_BUSY = 1,

    // Any previous operations have succeeded and the service is ready for
    // additional operations
    SYS_TOUCH_READY = 2

} SYS_TOUCH_STATUS;

// *****************************************************************************
/* Touch System Service Initialization

  Summary:
    Identifies the touch attributes supported

  Description:
    This data type defines the touch id and relate with
    the driver that should be used to initialize touch system.

    These functions help to align touch with hardware touch driver specific
    driver function calls.

  Remarks:
    None.
*/
typedef struct
{
    /* The Module index of the driver to which touch is attached. */
    SYS_MODULE_INDEX driverModuleIndex;
    
} SYS_TOUCH_INIT;


typedef struct
{
    union
    {
        struct
        {
            uint8_t nMessageTypeID; // Message type identifier
            uint8_t        nSource; // Message source identifier
            uint16_t        param0; // Message parameter zero
            uint16_t        param1; // Message parameter one
            uint16_t        param2; // Message parameter two
        };
        struct
        {
            uint16_t   dummy;
            uint16_t   nSizeData; // Size of data that pData identifies
            uintptr_t   *  pData; // Pointer to additional message data
        };
    };



    
} TOUCH_MSG_OBJ;
// *****************************************************************************
// *****************************************************************************
// Section: Touch System Service Interface Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: Initialization
// *****************************************************************************
// *****************************************************************************

//******************************************************************************
/* Function:
    SYS_MODULE_OBJ SYS_TOUCH_Initialize ( const SYS_MODULE_INIT * const init)

  Summary:
    Initializes and Enables the Touch Controller.

  Description:
    This function Enables the Touch module. Enable/Disable stop in idle mode
    feature based on the passed parameter value.

    This routine initializes the Touch module making it ready for clients to
    open and use it. The initialization data is specified by the init parameter.


  Precondition:
    None.

  Parameters:
    init            - Pointer to the data structure containing any data
                      necessary to initialize the hardware. This pointer may
                      be null if no data is required and default
                      initialization is to be used.

  Returns:
    If successful, returns a valid handle to the DMA module object.
    Otherwise, it returns SYS_MODULE_OBJ_INVALID.

  Example:
  <code>
    SYS_MODULE_OBJ objectHandle;
    SYS_TOUCH_INIT touchInit;

    objectHandle = SYS_TOUCH_Initialize(SYS_TOUCH_INDEX_1,
                                      (SYS_MODULE_INIT*)touchInit);
    if (SYS_MODULE_OBJ_INVALID == objectHandle)
    {
        // Handle error
    }
  </code>

  Remarks:
    This routine must be called before any other TOUCH systems service routines
    are called.

    Not all features are available on all devices. Refer to the specific
	device data sheet to determine availability.
*/
SYS_MODULE_OBJ SYS_TOUCH_Initialize (const SYS_MODULE_INDEX   moduleIndex,
                                     const SYS_MODULE_INIT * const init);

//*********************************************************************
/* Function:
    void SYS_TOUCH_TouchMsgGet(SYS_MODULE_INDEX moduleIndex)
	
   Summary:
     Populates the Graphics Object Library (GOL) message structure.
	 
   Description:
     This function populates the GOL message structure.
	 
   Preconditions:
     None.
	 
   Parameters:
     Pointer to the message structure to be populated.
	 
   Returns:
     None.
	 
   Remarks:
     None.

*/

//*********************************************************************
/* Function:
 (TOUCH_MSG_OBJ*) SYS_TOUCH_DrvObjGet(SYS_MODULE_INDEX moduleIndex)
	
   Summary:
     Populates the Graphics Object Library (GOL) message structure.
	 
   Description:
     This function populates the GOL message structure.
	 
   Preconditions:
     None.
	 
   Parameters:
     Pointer to the message structure to be populated.
	 
   Returns:
     None.
	 
   Remarks:
     None.

*/
TOUCH_MSG_OBJ* SYS_TOUCH_DrvObjGet(SYS_MODULE_INDEX moduleIndex);


// macro to draw repeating text
#define TouchShowMessage( pStr, color, x, y, width, height)             \
                {                                       \
                    GFX_ColorSet(gfxIndex, color);                    \
                    while(GFX_TextStringBoxDraw(gfxIndex, x,y,width,height,pStr, 0, GFX_ALIGN_LEFT) == GFX_STATUS_FAILURE);        \
                }	

//*******************************************************************************
/* Function:
    void SYS_Touch_Tasks(SYS_MODULE_OBJ object );

  Summary:
    Maintains the system service's state machine.

  Description:
    This function is used to maintain the Touch system service's internal state machine.
    This function is specifically designed for non interrupt trigger
    implementations(polling mode), and should be used only in polling mode.
    this function should be called from the SYS_Tasks() function.

  Precondition:
    Touch should have been initialized by calling SYS_Touch_Initialize.

  Parameters:
    object      - Object handle for the Touch module (returned from
                  SYS_Touch_Initialize)

  Returns:
    None.

  Example:
    <code>
    // 'object' Returned from SYS_Touch_Initialize

    while (true)
    {
        SYS_Touch_Tasks ((object) );

        // Do other tasks
    }
    </code>

  Remarks:
    This function is normally not called directly by an application.  It is
    called by the system's Tasks routine (SYS_Tasks).

 */
 
void SYS_TOUCH_Tasks(SYS_MODULE_OBJ object);


// *****************************************************************************
// *****************************************************************************
// Section: Audio System Service Client Routines
// *****************************************************************************
// *****************************************************************************

/* Function:
    SYS_TOUCH_HANDLE SYS_TOUCH_Open( SYS_MODULE_INDEX  moduleIndex)

  Summary:
    Opens the touch service specified by the moduleIndex and returns a handle to it.

  Description:
    This function opens the touch service specified by the index and provides a
    handle that must be provided to all other client-level operations to
    identify the caller.

  Precondition:
    The SYS_TOUCH_Initialize function must have been called before calling this
    function.

  Parameters:
    moduleIndex    - Identifier for the touch to be opened

  Returns:
    If successful, the routine returns a valid open-instance handle (a number
    identifying both the caller and the module instance).

    If an error occurs, the return value is SYS_TOUCH_HANDLE_INVALID.

   Example:
    <code>
    SYS_TOUCH_HANDLE handle;

    handle = SYS_TOUCH_Open(SYS_TOUCH_INDEX_0);
    if (SYS_TOUCH_HANDLE_INVALID == handle)
    {
        // Unable to open the service
        // May be the service is not initialized or the initialization
        // is not complete.
    }
    </code>

  Remarks:
    The handle returned is valid until the SYS_TOUCH_lose routine is called.
*/
SYS_TOUCH_HANDLE SYS_TOUCH_Open( SYS_MODULE_INDEX  moduleIndex);


// *****************************************************************************
/* Function:
    void SYS_TOUCH_CalibrationSet( SYS_MODULE_INDEX  moduleIndex, 
	                               DRV_TOUCH_SAMPLE_POINTS * samplePoints)

  Summary:
    Sets the calibration values for Touch client specified by moduleIndex.

  Description:
    This function sets the calibration values provided by samplePoints for the
    specified client.

  Precondition:
    Function SYS_TOUCH_Open must have been called before calling this
    function.

  Parameters:
    moduleIndex    - Identifier for the touch to be opened
    samplePoints   - sample points

  Returns:
    none.

   Example:
    <code>
    SYS_MODULE_INDEX moduleIndex;
    SYS_TOUCH_SAMPLE_POINTS samplePoints;
    SYS_TOUCH_CalibrationSet( moduleIndex,  samplePoints);
    </code>

  Remarks:
     None.
*/
void SYS_TOUCH_CalibrationSet( SYS_MODULE_INDEX moduleIndex, 
                               DRV_TOUCH_SAMPLE_POINTS *samplePoints);


// *****************************************************************************
/* Function:
    int SYS_TOUCH_RegisterObserver(SYS_MODULE_INDEX moduleIndex, 
                                    void (*callback)(TOUCH_MSG_OBJ *pMsg));

  Summary:
    Registers an observer to touch events. 

  Description:
    This function registers an observer to touch events. The callback function 
    gets called when a valid touch event is detected by the touch system service.

  Precondition:
    Function SYS_TOUCH_Open must have been called before calling this
    function.

  Parameters:
    moduleIndex    - Identifier for the touch to be opened
    callback       - callback function

  Returns:
    none.

   Example:
    <code>
    SYS_MODULE_INDEX moduleIndex;
    void callbackFunction(TOUCH_MSG_OBJ *pMsg);
 
    SYS_TOUCH_RegisterObserver( moduleIndex,  callbackFunction);
    </code>

  Remarks:
     None.
*/
int SYS_TOUCH_RegisterObserver(SYS_MODULE_INDEX moduleIndex, 
                                void (*callback)(TOUCH_MSG_OBJ *pMsg));

#ifdef __cplusplus
    }
#endif
    
#endif //_SYS_TOUCH_H