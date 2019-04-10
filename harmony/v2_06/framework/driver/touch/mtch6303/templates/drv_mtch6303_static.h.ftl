/*******************************************************************************
  MTCH6303 Driver Interface Declarations for Static Single Instance Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_mtch6303_static.h

  Summary:
    MTCH6303 driver interface declarations for the static single instance driver.

  Description:
    The MTCH6303 device driver provides a simple interface to manage the MTCH6303
    modules. This file defines the interface Declarations for the MTCH6303 driver.
    
  Remarks:
    Static interfaces incorporate the driver instance number within the names
    of the routines, eliminating the need for an object ID or object handle.
    
    Static single-open interfaces also eliminate the need for the open handle.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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
*******************************************************************************/
//DOM-IGNORE-END

#ifndef _DRV_TOUCH_MTCH6303_STATIC_H
#define _DRV_TOUCH_MTCH6303_STATIC_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "driver/touch/drv_touch.h"
#include "driver/driver_common.h"
#include "system/system.h"

// *****************************************************************************
/* 

  Summary:
    

  Description:
    

  Remarks:
    

*/

#define DRV_TOUCH_MTCH6303_TOUCH_NUM_INPUTS          0xA

// *****************************************************************************
/* MTCH6303 Driver Module Index Count

  Summary:
    Number of valid MTCH6303 driver indices.

  Description:
    This constant identifies the number of valid MTCH6303 driver indices.

  Remarks:
    This constant should be used in place of hard-coded numeric literals.

    This value is derived from device-specific header files defined as part of 
    the peripheral libraries.
*/

typedef enum {

    MTCH6303_ID_1 = 0,
    MTCH6303_NUMBER_OF_MODULES
            
} DRV_TOUCH_MTCH6303_MODULE_ID;

// *****************************************************************************
/* MTCH6303 Driver Buffer Handle

  Summary:
    Handle identifying a read or write buffer passed to the driver.

  Description:
    A buffer handle value is returned by a call to the DRV_TOUCH_MTCH6303_BufferAddRead 
    or DRV_TOUCH_MTCH6303_BufferAddWrite functions. This handle is associated with the
    buffer passed into the function and it allows the application to track the
    completion of the data from (or into) that buffer.  The buffer handle value
    returned from the "buffer add" function is returned back to the client 
    by the "event handler callback" function registered with the driver.

    The buffer handle assigned to a client request expires when the client has 
    been notified of the completion of the buffer transfer (after event handler 
    function that notifies the client returns) or after the buffer has been 
    retired by the driver if no event handler callback was set.

  Remarks:
    None
*/

typedef uintptr_t DRV_TOUCH_MTCH6303_BUFFER_HANDLE;

// *****************************************************************************
/* 

  Summary:
    

  Description:
    

  Remarks:
    

*/

typedef uintptr_t DRV_TOUCH_MTCH6303_TOUCH_BUFFER_HANDLE;

// *****************************************************************************
/* MTCH6303 Driver Invalid Buffer Handle

  Summary:
    Definition of an invalid buffer handle.

  Description:
    This is the definition of an invalid buffer handle. An invalid buffer handle
    is returned by DRV_TOUCH_MTCH6303_BufferAddRead and DRV_TOUCH_MTCH6303_BufferAddWrite
    functions if the buffer add request was not successful. 

  Remarks:
    None
*/

#define DRV_TOUCH_MTCH6303_BUFFER_HANDLE_INVALID /*DOM-IGNORE-BEGIN*/((DRV_TOUCH_MTCH6303_BUFFER_HANDLE)(-1))/*DOM-IGNORE-END*/

// *****************************************************************************
/* 

  Summary:
    

  Description:
    

  Remarks:
    

*/

#define DRV_TOUCH_MTCH6303_TOUCH_BUFFER_HANDLE_INVALID /*DOM-IGNORE-BEGIN*/((DRV_TOUCH_MTCH6303_TOUCH_BUFFER_HANDLE)(-1))/*DOM-IGNORE-END*/

// *****************************************************************************
/* MTCH6303 Buffer Events

  Summary:
    Lists the different conditions that happens during a buffer transfer.

  Description:
    This enumeration identifies the different conditions that can happen during
    a buffer transaction. Callbacks can be made with the appropriate buffer
    condition passed as a parameter to execute the desired action.
    The application can also poll the BufferStatus flag to check the status of
    transfer.

    The values act like flags and multiple flags can be set.

  Remarks:
    None.
*/

typedef enum
{
    /* */
    DRV_TOUCH_MTCH6303_BUFFER_EVENT_COMPLETE = 0,

    /* */
    DRV_TOUCH_MTCH6303_BUFFER_EVENT_ERROR,

    /* */
    DRV_TOUCH_MTCH6303_BUFFER_EVENT_ABORT

} DRV_TOUCH_MTCH6303_BUFFER_EVENT;


// *****************************************************************************
/* 

  Summary:
    

  Description:
    

  Remarks:
    

*/

typedef enum
{
    /* */
    DRV_TOUCH_MTCH6303_TOUCH_BUFFER_EVENT_COMPLETE = 0,

    /* */
    DRV_TOUCH_MTCH6303_TOUCH_BUFFER_EVENT_ERROR,

    /* */
    DRV_TOUCH_MTCH6303_TOUCH_BUFFER_EVENT_ABORT

} DRV_TOUCH_MTCH6303_TOUCH_BUFFER_EVENT;

// *****************************************************************************
/* 

  Summary:

  Description:

  Remarks:

*/
typedef enum
{
    /* */
    DRV_TOUCH_MTCH6303_MSG_REPORT_ECHO /* */ = 0x04, /* */
    
    /* */
    DRV_TOUCH_MTCH6303_MSG_REPORT_FLASH_CONTENTS /* */ = 0x17, /* */
            
    /* */
    DRV_TOUCH_MTCH6303_MSG_REPORT_ADC_DBG        /* */ = 0x60, /* */
            
    /* */
    DRV_TOUCH_MTCH6303_MSG_REPORT_TRACE          /* */ = 0x90, /* */
            
    /* */
    DRV_TOUCH_MTCH6303_MSG_REPORT_SWIPE          /* */ = 0xA0, /* */
            
    /* */
    DRV_TOUCH_MTCH6303_MSG_REPORT_SCROLL         /* */ = 0xA1, /* */
            
    /* */
    DRV_TOUCH_MTCH6303_MSG_REPORT_TAP            /* */ = 0xA2, /* */
            
    /* */
    DRV_TOUCH_MTCH6303_MSG_REPORT_NOISE          /* */ = 0xB0, /* */
            
    /* */
    DRV_TOUCH_MTCH6303_MSG_REPORT_MUT_NORM_SEC   /* */ = 0xC3, /* */
            
    /* */
    DRV_TOUCH_MTCH6303_MSG_REPORT_PARAM_READ     /* */ = 0x04, /* */
            
    /* */
    DRV_TOUCH_MTCH6303_MSG_REPORT_ACK            /* */ = 0xF0, /* */
            
    /* */
    DRV_TOUCH_MTCH6303_MSG_REPORT_TOUCH_FILTERED /* */ = 0xF2, /* */
            
    /* */
    DRV_TOUCH_MTCH6303_MSG_REPORT_TOUCH_PREDICT  /* */ = 0xF3, /* */
            
    /* */
    DRV_TOUCH_MTCH6303_MSG_REPORT_TOUCH_RAW      /* */ = 0xF4, /* */
    
    /* */
    DRV_TOUCH_MTCH6303_MSG_REPORT_TOUCH_POS16    /* */ = 0xF5, /* */
            
    /* */
    DRV_TOUCH_MTCH6303_MSG_REPORT_SELF_RAW       /* */ = 0xFA, /* */
            
    /* */
    DRV_TOUCH_MTCH6303_MSG_REPORT_SELF_NORM      /* */ = 0xFD, /* */
            
    /* */
    DRV_TOUCH_MTCH6303_MSG_REPORT_FWD_GESTIC     /* */ = 0xFE, /* */
            
    /* */
    DRV_TOUCH_MTCH6303_MSG_REPORT_FW_VERSION     /* */ = 0xFF, /* */
            
    /* */
    DRV_TOUCH_MTCH6303_MSG_CMD_ECHO              /* */ = 0x04, /* */
            
    /* */
    DRV_TOUCH_MTCH6303_MSG_CMD_READ_FLASH        /* */ = 0x17, /* */
            
    /* */
    DRV_TOUCH_MTCH6303_MSG_CMD_ENTER_BOOTLDR     /* */ = 0x55, /* */
            
    /* */
    DRV_TOUCH_MTCH6303_MSG_CMD_SET_PARAM         /* */ = 0xE0, /* */
     
    /* */
    DRV_TOUCH_MTCH6303_MSG_CMD_GET_PARAM         /* */ = 0xE1, /* */
            
    /* */
    DRV_TOUCH_MTCH6303_MSG_CMD_FORCE_BASE_lINE   /* */ = 0xFB, /* */
            
    /* */
    DRV_TOUCH_MTCH6303_MSG_CMD_RESET_GESTIC      /* */ = 0xFC, /* */
            
    /* */
    DRV_TOUCH_MTCH6303_MSG_CMD_QUERY_VERSION     /* */ = 0xE0, /* */
            
} DRV_TOUCH_MTCH6303_MSG_ID;

// *****************************************************************************
/* MTCH6303 Buffer Event Callback

  Function:
    void ( *DRV_TOUCH_MTCH6303_BUFFER_EVENT_HANDLER ) ( DRV_TOUCH_MTCH6303_BUFFER_EVENT event, 
                                                  DRV_TOUCH_MTCH6303_BUFFER_HANDLE bufferHandle, 
                                                  uintptr_t context )

  Summary:
    Points to a callback after completion of an register read -write or message
  stream read - write.

  Description:
    This type identifies the MTCH6303 Buffer Event. It allows the client driver
    to register a callback using DRV_TOUCH_MTCH6303_BUFFER_EVENT_HANDLER. By using this
    mechanism, the driver client will be notified at the completion of the
    corresponding transfer.

  Parameters:
    DRV_TOUCH_MTCH6303_BUFFER_EVENT - Status of I2C transfer

    bufferHandle - Handle that identifies that identifies the particular Buffer
                   Object

    context     -  pointer to the object to be processed.

  Remarks:
    A transfer can be composed of various transfer segments.  Once a transfer
    is completed the driver will call the client registered transfer
    callback.

    The callback could be called from ISR context and should be kept as short
    as possible.  It is meant for signaling and it should not be blocking.

*/

typedef void ( *DRV_TOUCH_MTCH6303_BUFFER_EVENT_HANDLER ) (DRV_TOUCH_MTCH6303_BUFFER_EVENT event,
				DRV_TOUCH_MTCH6303_BUFFER_HANDLE bufferHandle, uintptr_t context );

// *****************************************************************************
/* MTCH6303 Touch Event Callback Handler

    void ( *DRV_TOUCH_MTCH6303_TOUCH_EVENT_HANDLER ) ( uint8_t event, 
                                                  int16_t x, 
                                                  int16_t y)
  Summary:
    

  Description:
    

  Remarks:
    

*/

typedef void ( *DRV_TOUCH_MTCH6303_TOUCH_EVENT_HANDLER ) (uint8_t event,
                int16_t x, int16_t y);

// *****************************************************************************
/* MTCH6303 Client-Specific Driver Status

  Summary:
    Defines the client-specific status of the MTCH6303 driver.

  Description:
    This enumeration defines the client-specific status codes of the MTCH6303
    driver.

  Remarks:
    Returned by the DRV_TOUCH_MTCH6303_ClientStatus function.
*/

typedef enum
{
    /* An error has occurred.*/
    DRV_TOUCH_MTCH6303_CLIENT_STATUS_ERROR    = DRV_CLIENT_STATUS_ERROR,

    /* The driver is closed, no operations for this client are ongoing,
    and/or the given handle is invalid. */
    DRV_TOUCH_MTCH6303_CLIENT_STATUS_CLOSED   = DRV_CLIENT_STATUS_CLOSED,

    /* The driver is currently busy and cannot start additional operations. */
    DRV_TOUCH_MTCH6303_CLIENT_STATUS_BUSY     = DRV_CLIENT_STATUS_BUSY,

    /* The module is running and ready for additional operations */
    DRV_TOUCH_MTCH6303_CLIENT_STATUS_READY    = DRV_CLIENT_STATUS_READY

} DRV_TOUCH_MTCH6303_CLIENT_STATUS;


// *****************************************************************************
/* MTCH6303 Driver Errors.

  Summary:
    Defines the possible errors that can occur during driver operation.

  Description:
    This data type defines the possible errors that can occur when occur during
    MTCH6303 driver operation. These values are returned by DRV_TOUCH_MTCH6303_ErrorGet
    function.

  Remarks:
    None
*/

typedef enum
{
    /* There was no error */
    DRV_TOUCH_MTCH6303_ERROR_NONE = 
            /*DOM-IGNORE-BEGIN*/ 0 /*DOM-IGNORE-END*/,

    /* Invalid address */
    DRV_TOUCH_MTCH6303_ERROR_INVALID_ADDRESS = 
            /*DOM-IGNORE-BEGIN*/ (1 << 0) /*DOM-IGNORE-END*/,            

} DRV_TOUCH_MTCH6303_ERROR;

// *****************************************************************************
/* MTCH6303 Driver Transfer Flags

  Summary
    Specifies the status of the receive or transmit

  Description
    This type specifies the status of the receive or transmit operation.

  Remarks:
    More than one of these values may be OR'd together to create a complete
    status value.  To test a value of this type, the bit of interest must be
    ANDed with the value and checked to see if the result is non-zero.
*/

typedef enum
{

    /* Indicates that at least one byte of Data has been received */
    DRV_TOUCH_MTCH6303_TRANSFER_STATUS_READ_BUFFER_FULL
        /*DOM-IGNORE-BEGIN*/  = (1 << 0) /*DOM-IGNORE-END*/,

    /* Indicates that the core driver receiver buffer is empty */
    DRV_TOUCH_MTCH6303_TRANSFER_STATUS_READ_BUFFER_EMPTY
        /*DOM-IGNORE-BEGIN*/  = (1 << 1) /*DOM-IGNORE-END*/,

    /* Indicates that the core driver transmitter buffer is full */
    DRV_TOUCH_MTCH6303_TRANSFER_STATUS_WRITE_BUFFER_FULL
        /*DOM-IGNORE-BEGIN*/  = (1 << 2) /*DOM-IGNORE-END*/,

    /* Indicates that the core driver transmitter buffer is empty */
    DRV_TOUCH_MTCH6303_TRANSFER_STATUS_WRITE_BUFFER_EMPTY
        /*DOM-IGNORE-BEGIN*/  = (1 << 3) /*DOM-IGNORE-END*/

} DRV_TOUCH_MTCH6303_TRANSFER_STATUS;

// *****************************************************************************
/* 

  Summary:
    

  Description:
    

  Remarks:
    

*/

typedef struct __attribute__ ( ( __packed__ ) )
{
    /* */
    uint32_t nTouch : 4;

    /* */
    uint32_t streamReady : 1;

    /* */
    uint32_t gestureReady : 1;

    /* */
    uint32_t gestICData : 1;

    /* */
    uint32_t reserved : 1;

} DRV_TOUCH_MTCH6303_TOUCH_STATUS;

// *****************************************************************************
/* 

  Summary:
    

  Description:
    

  Remarks:
    

*/

typedef struct __attribute__ ( ( __packed__ ) )
{
    /* */
    uint32_t touchState : 1;

    /* */
    uint32_t inRange : 1;

    /* */
    uint32_t reserved : 6;

} DRV_TOUCH_MTCH6303_TOUCH_NIBBLE_0;

// *****************************************************************************
/* 

  Summary:
    

  Description:
    

  Remarks:
    

*/

typedef struct __attribute__ ( ( __packed__ ) )
{
    /* */
    DRV_TOUCH_MTCH6303_TOUCH_NIBBLE_0 nibble_0;

    /* */
    uint8_t touchId;

    /* */
    uint16_t x;

    /* */
    uint16_t y;

} DRV_TOUCH_MTCH6303_TOUCH_INPUT;

// *****************************************************************************
/* 

  Summary:
    

  Description:
    

  Remarks:
    

*/

typedef struct __attribute__ ( ( __packed__ ) ) 
{
    /* */
    //Fixing the I2C alignment issue - remove the first byte being taken in as device address
	//uint8_t i2cReadAddr;

    /* */
    DRV_TOUCH_MTCH6303_TOUCH_STATUS status;

    /* */
    DRV_TOUCH_MTCH6303_TOUCH_INPUT  touch [ DRV_TOUCH_MTCH6303_TOUCH_NUM_INPUTS ];

} DRV_TOUCH_MTCH6303_TOUCH_DATA;

// *****************************************************************************
/* 

  Summary:
    

  Description:
    

  Remarks:
    

*/

typedef struct __attribute__ ( ( __packed__ ) )
{
    /* */
    uint32_t msgFragSize:  6;

    /* */
    uint32_t continued:    1;

    /* */
    uint32_t moreMessages: 1;

} DRV_TOUCH_MTCH6303_TOUCH_MESSAGE_HEADER;

// *****************************************************************************
/* 

  Summary:
    

  Description:
    

  Remarks:
    

*/

typedef struct __attribute__ ( ( __packed__ ) )
{
    /* */
    DRV_TOUCH_MTCH6303_TOUCH_MESSAGE_HEADER  header;

    /* */
    uint8_t                            payload[0x3E];

} DRV_TOUCH_MTCH6303_TOUCH_MESSAGE;

// *********************************************************************************************
// *********************************************************************************************
// Section: System Interface Headers for the Instance 0 of MTCH6303 static driver
// *********************************************************************************************
// *********************************************************************************************

SYS_MODULE_OBJ DRV_TOUCH_MTCH6303_Initialize   ( const SYS_MODULE_INDEX   index, const SYS_MODULE_INIT    * const init );
void           DRV_TOUCH_MTCH6303_Deinitialize ( SYS_MODULE_OBJ object );
SYS_STATUS     DRV_TOUCH_MTCH6303_Status       ( SYS_MODULE_OBJ object );
void           DRV_TOUCH_MTCH6303_Tasks        ( SYS_MODULE_OBJ object );
void           DRV_TOUCH_MTCH6303_TouchTasks   ( void );

/*********************************************************************
  Function:
    DRV_TOUCH_POSITION_SINGLE DRV_TOUCH_MTCH6303_TouchStatus( const SYS_MODULE_INDEX index )

  Summary:
    Returns the status of the current touch input.
	<p><b>Implementation:</b> Static</p>

  Description:
    It returns the status of the current touch input.

  Parameters
    None.

  Returns
    It returns the status of the current touch input.

*/
DRV_TOUCH_POSITION_STATUS DRV_TOUCH_MTCH6303_TouchStatus( const SYS_MODULE_INDEX index );


/*********************************************************************
  Function:
    void DRV_TOUCH_MTCH6303_TouchDataRead( const SYS_MODULE_INDEX index )

  Summary:
    Notifies the driver that the current touch data has been read
	<p><b>Implementation:</b> Static</p>

  Description:
    Notifies the driver that the current touch data has been read

  Parameters
    None.

  Returns
    None.

*/
void DRV_TOUCH_MTCH6303_TouchDataRead( const SYS_MODULE_INDEX index );


/*********************************************************************
  Function:
    short DRV_TOUCH_MTCH6303_TouchGetX( uint8 touchNumber )

  Summary:
    Returns the x coordinate of touch input.
	<p><b>Implementation:</b> Static</p>

  Description:
    It returns the x coordinate in form of number of pixes for a touch input
  denoted by touchNumber.

  Parameters
   touchNumber - index to the touch input.

  Returns
    It returns the x coordinate of the touch input in terms of number of pixels.

*/
short DRV_TOUCH_MTCH6303_TouchGetX( uint8_t touchNumber );


/*********************************************************************
  Function:
    short DRV_TOUCH_MTCH6303_TouchGetY( uint8 touchNumber )

  Summary:
    Returns the y coordinate of touch input.
	<p><b>Implementation:</b> Static</p>

  Description:
    It returns the y coordinate in form of number of pixes for a touch input
  denoted by touchNumber.

  Parameters
   touchNumber - index to the touch input.

  Returns
    It returns the y coordinate of the touch input in terms of number of pixels.

*/

short DRV_TOUCH_MTCH6303_TouchGetY( uint8_t touchNumber );

// *********************************************************************************************
// *********************************************************************************************
// Section: General Client Interface Headers for the Instance 0 of MTCH6303 static driver
// *********************************************************************************************
// *********************************************************************************************

DRV_HANDLE    DRV_TOUCH_MTCH6303_Open (  const SYS_MODULE_INDEX drvIndex, const DRV_IO_INTENT intent );
DRV_TOUCH_MTCH6303_CLIENT_STATUS   DRV_TOUCH_MTCH6303_Close          ( void );
DRV_TOUCH_MTCH6303_CLIENT_STATUS   DRV_TOUCH_MTCH6303_ClientStatus   ( void );
DRV_TOUCH_MTCH6303_ERROR           DRV_TOUCH_MTCH6303_ErrorGet       ( void );
DRV_TOUCH_MTCH6303_TRANSFER_STATUS DRV_TOUCH_MTCH6303_TransferStatus ( void );

void DRV_TOUCH_MTCH6303_TouchInputRead( SYS_MODULE_OBJ object );

inline uint16_t DRV_TOUCH_MTCH6303_TouchInputMap( uint16_t touchValue, uint16_t dispResolution );

void DRV_TOUCH_MTCH6303_AddRegisterRead( DRV_TOUCH_MTCH6303_BUFFER_HANDLE * bufferHandle,
                                   uint8_t source,
                                   size_t  nBytes,
                                   uint8_t * destination );

void DRV_TOUCH_MTCH6303_AddRegisterWrite( DRV_TOUCH_MTCH6303_BUFFER_HANDLE * bufferHandle,
                                    uint8_t destination,
                                    size_t  nBytes,
                                    uint8_t * source );

void DRV_TOUCH_MTCH6303_BufferEventHandlerSet
(
    const DRV_TOUCH_MTCH6303_BUFFER_EVENT_HANDLER eventHandler,
    const uintptr_t context
);

void DRV_TOUCH_MTCH6303_TOUCH_AddTouchInputRead
( 
    DRV_TOUCH_MTCH6303_TOUCH_BUFFER_HANDLE * bufferHandle,
    DRV_TOUCH_MTCH6303_TOUCH_DATA          * touchData 
);

void DRV_TOUCH_MTCH6303_TOUCH_AddMessageReportRead
( 
    DRV_TOUCH_MTCH6303_TOUCH_BUFFER_HANDLE * bufferHandle,
    DRV_TOUCH_MTCH6303_TOUCH_MESSAGE       * messageRep,
    size_t                             messageSize
);

void DRV_TOUCH_MTCH6303_TOUCH_AddMessageCommandWrite
( 
    DRV_TOUCH_MTCH6303_TOUCH_BUFFER_HANDLE   * bufferHandle,
    DRV_TOUCH_MTCH6303_TOUCH_MESSAGE         * messageCmd,
    size_t                               messageSize
);

void DRV_TOUCH_MTCH6303_TOUCH_TouchEventHandlerSet
(
    const DRV_TOUCH_MTCH6303_TOUCH_EVENT_HANDLER eventHandler,
    const uintptr_t context
);

void DRV_TOUCH_MTCH6303_TOUCH_Tasks( void );

#endif

/*******************************************************************************
 End of File
*/