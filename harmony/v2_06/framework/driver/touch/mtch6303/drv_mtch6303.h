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
    module. This file defines the interface Declarations for the MTCH6303 driver.
    
  Remarks:
    Static single instance driver interface eliminates the need for an object ID
    or object handle.
    Static single-open interfaces also eliminate the need for the open handle.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2015 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _DRV_MTCH6303_STATIC_H
#define _DRV_MTCH6303_STATIC_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "driver/driver_common.h"
#include "system/system.h"

// *****************************************************************************
/* MTCH6303 Number of touch input packets

  Summary:
   Definition of number of touch input packets can be identified by MTCH6303.

  Description:
   MTCH6303 supports multi-touch and can identify upto 10 different touch input 
   packets.

  Remarks:
   None.

*/

#define DRV_MTCH6303_TOUCH_NUM_INPUTS          0xA

// *****************************************************************************
/* MTCH6303 Driver Buffer Handle

  Summary:
    Handle identifying a read or write buffer passed to the driver.

  Description:
    A buffer handle value is returned by a call to the 
    DRV_MTCH6303_AddRegisterRead, DRV_MTCH6303_AddRegisterWrite or
    DRV_MTCH6303_TouchInputRead functions. This handle is associated with the
    buffer passed into the function and it allows the application to track the
    completion of the data from (or into) that buffer.  The buffer handle value
    returned from these functions is returned back to the client 
    by the "event handler callback" function registered with the driver.

    The buffer handle assigned to a client request expires when the client has 
    been notified of the completion of the buffer transfer (after event handler 
    function that notifies the client returns) or after the buffer has been 
    retired by the driver if no event handler callback was set.

  Remarks:
    None
*/

typedef uintptr_t DRV_MTCH6303_BUFFER_HANDLE;

// *****************************************************************************
/* MTCH6303 Driver Touch Message Queue Buffer Handle

  Summary:
    Handle identifying a read or write touch message buffer passed to the driver.

  Description:
    A touch message buffer handle value is returned by a call to the 
    DRV_MTCH6303_TOUCH_AddMessageReportRead, 
    DRV_MTCH6303_TOUCH_AddMessageCommandWrite or 
    DRV_MTCH6303_TOUCH_AddTouchInputRead. This handle is associated with the
    buffer passed into the function and it allows the application to track the
    completion of the data from (or into) that buffer.  The buffer handle value
    returned from these functions is returned back to the client 
    by the "event handler callback" function registered with the driver.

    The buffer handle assigned to a client request expires when the client has 
    been notified of the completion of the buffer transfer (after event handler 
    function that notifies the client returns) or after the buffer has been 
    retired by the driver if no event handler callback was set.

  Remarks:
    None.

*/

typedef uintptr_t DRV_MTCH6303_TOUCH_BUFFER_HANDLE;

// *****************************************************************************
/* MTCH6303 Driver Invalid Buffer Handle

  Summary:
    Definition of an invalid buffer handle.

  Description:
    This is the definition of an invalid buffer handle. An invalid buffer handle
    is returned by DRV_MTCH6303_AddRegisterRead, DRV_MTCH6303_AddRegisterWrite or
    DRV_MTCH6303_TouchInputRead functions if the request was not successful. 

  Remarks:
    None
*/

#define DRV_MTCH6303_BUFFER_HANDLE_INVALID /*DOM-IGNORE-BEGIN*/((DRV_MTCH6303_BUFFER_HANDLE)(-1))/*DOM-IGNORE-END*/

// *****************************************************************************
/* MTCH6303 Driver Invalid Buffer Handle

  Summary:
    Definition of an invalid buffer handle.

  Description:
    This is the definition of an invalid buffer handle. An invalid buffer handle
    is returned by DRV_MTCH6303_TOUCH_AddMessageReportRead, 
    DRV_MTCH6303_TOUCH_AddMessageCommandWrite or 
    DRV_MTCH6303_TOUCH_AddTouchInputRead functions if the request was not successful. 

  Remarks:
    None
*/

#define DRV_MTCH6303_TOUCH_BUFFER_HANDLE_INVALID /*DOM-IGNORE-BEGIN*/((DRV_MTCH6303_TOUCH_BUFFER_HANDLE)(-1))/*DOM-IGNORE-END*/

// *****************************************************************************
/* MTCH6303 Buffer Events

  Summary:
    Lists the different conditions that happens during a buffer transfer.

  Description:
    This enumeration identifies the different conditions that can happen during
    a buffer transaction. Callbacks can be made with the appropriate buffer
    condition passed as a parameter to execute the desired action.

    The values act like flags and multiple flags can be set.

  Remarks:
    None.
*/

typedef enum
{
    /* Event buffer transfer complete */
    DRV_MTCH6303_BUFFER_EVENT_COMPLETE /*DOM-IGNORE-BEGIN*/ = 0 /*DOM-IGNORE-END*/, 

    /* Event buffer transfer error */
    DRV_MTCH6303_BUFFER_EVENT_ERROR,

    /* Event buffer transfer abort */
    DRV_MTCH6303_BUFFER_EVENT_ABORT

} DRV_MTCH6303_BUFFER_EVENT;

// *****************************************************************************
/* MTCH6303 Touch Message Buffer Events

  Summary:
    Lists the different conditions that happens during a touch message buffer 
    transfer.

  Description:
    This enumeration identifies the different conditions that can happen during
    a touch message buffer transaction. Callbacks can be made with the 
    appropriate touch message buffer condition passed as a parameter to execute 
    the desired action.

    The values act like flags and multiple flags can be set.

  Remarks:
    None.
*/

typedef enum
{
    /* Event touch message buffer transfer complete */
    DRV_MTCH6303_TOUCH_BUFFER_EVENT_COMPLETE /*DOM-IGNORE-BEGIN*/ = 0 /*DOM-IGNORE-END*/,

    /* Event touch message buffer transfer error */
    DRV_MTCH6303_TOUCH_BUFFER_EVENT_ERROR,

    /* Event touch message buffer transfer abort */
    DRV_MTCH6303_TOUCH_BUFFER_EVENT_ABORT

} DRV_MTCH6303_TOUCH_BUFFER_EVENT;

// *****************************************************************************
/* MTCH6303 I2C Accessible Register Identification.

  Summary:
    List of MTCH6303 I2C Accessible Register Identification.

  Description:
    This enumeration identifies the different I2C accessible MTCH6303 Registers. 
    The identifier is passed as source to the register read routine or as 
    destination to the register write routine. The MTCH6303 driver routine to 
    read the I2C accessible MTCH6303 registers is DRV_MTCH6303_AddRegisterRead. 
    The MTCH6303 driver routine to write the I2C accessible MTCH6303 registers 
    is DRV_MTCH6303_AddRegisterWrite.

  Remarks:
    To read or write multiple registers, identifier of only first register is 
    sufficient as source or destination respectively.
*/
        
typedef enum
{
    /* Touch Status register */
    DRV_MTCH6303_REG_TOUCH_STATUS /* DOM-IGNORE-BEGIN */ = 0x00, /* DOM-IGNORE-END */
            
    /* Touch 0th Input first nibble register */
    DRV_MTCH6303_REG_TOUCH_0_NIBBLE_0 /* DOM-IGNORE-BEGIN */ = 0x01, /* DOM-IGNORE-END */
            
    /* Touch 0th Input identifier */
    DRV_MTCH6303_REG_TOUCH_0_ID /* DOM-IGNORE-BEGIN */ = 0x02, /* DOM-IGNORE-END */

    /* Touch 0th Input LSB of x coordinate */            
    DRV_MTCH6303_REG_TOUCH_0_X1_LSB /* DOM-IGNORE-BEGIN */ = 0x03, /* DOM-IGNORE-END */
            
    /* Touch 0th Input MSB of x coordinate */
    DRV_MTCH6303_REG_TOUCH_0_X1_MSB /* DOM-IGNORE-BEGIN */ = 0x04, /* DOM-IGNORE-END */
    
    /* Touch 0th Input LSB of Y coordinate */
    DRV_MTCH6303_REG_TOUCH_0_Y1_LSB /* DOM-IGNORE-BEGIN */ = 0x05, /* DOM-IGNORE-END */

    /* Touch 0th Input LSB of y coordinate */            
    DRV_MTCH6303_REG_TOUCH_0_Y1_MSB /* DOM-IGNORE-BEGIN */ = 0x06, /* DOM-IGNORE-END */

    /* Touch 9th Input first nibble register */
    DRV_MTCH6303_REG_TOUCH_9_NIBBLE_0 /* DOM-IGNORE-BEGIN */ = 0x37, /* DOM-IGNORE-END */
            
    /* Touch 9th Input identifier */
    DRV_MTCH6303_REG_TOUCH_9_ID /* DOM-IGNORE-BEGIN */ = 0x38, /* DOM-IGNORE-END */

    /* Touch 9th Input LSB of x coordinate */            
    DRV_MTCH6303_REG_TOUCH_9_X1_LSB /* DOM-IGNORE-BEGIN */ = 0x39, /* DOM-IGNORE-END */
            
    /* Touch 9th Input MSB of x coordinate */
    DRV_MTCH6303_REG_TOUCH_9_X1_MSB /* DOM-IGNORE-BEGIN */ = 0x3A, /* DOM-IGNORE-END */
    
    /* Touch 9th Input LSB of y coordinate */
    DRV_MTCH6303_REG_TOUCH_9_Y1_LSB /* DOM-IGNORE-BEGIN */ = 0x3B, /* DOM-IGNORE-END */

    /* Touch 9th Input MSB of y coordinate */            
    DRV_MTCH6303_REG_TOUCH_9_Y1_MSB /* DOM-IGNORE-BEGIN */ = 0x3C, /* DOM-IGNORE-END */

    /* Maximum touch input registers */
    DRV_MTCH6303_REG_TOUCH_MAX      /* DOM-IGNORE-BEGIN */ = 0x3D, /* DOM-IGNORE-END */
            
    /* Register specifying space available for writing into Rx buffer */
    DRV_MTCH6303_REG_RX_BYTES_READY /* DOM-IGNORE-BEGIN */ = 0xFB, /* DOM-IGNORE-END */
            
    /* Register Pointing to Rx buffer */
    DRV_MTCH6303_REG_RX_BUFFER_POINTER /* DOM-IGNORE-BEGIN */ = 0xFC, /* DOM-IGNORE-END */
            
    /* Register specifying Bytes ready to be read from TX buffer */
    DRV_MTCH6303_REG_TX_BYTES_READY /* DOM-IGNORE-BEGIN */ = 0xFD, /* DOM-IGNORE-END */
            
    /* Register Pointing to Tx buffer */
    DRV_MTCH6303_REG_TX_BUFFER_POINTER /* DOM-IGNORE-BEGIN */ = 0xFE, /* DOM-IGNORE-END */
            
} DRV_TOUCH_MTCH6303_I2C_REGISTER_MAP;

// *****************************************************************************
/* MTCH6303 Touch message Identification.

  Summary:
    List of report or command message identification.

  Description:
    This enumeration identifies the different report or command messages 
    supported by MTCH6303. This identifier identifies the type of the message. 
    The identifier is passed in the message DRV_MTCH6303_TOUCH_MESSAGE as first 
    byte of the payload. It is applicable only for first fragment of message. If 
    message involves multiple fragments, the payload of message fragments other 
    than first fragment should start with normal payload byte. The touch message
    is read or send to MTCH6303 by using DRV_MTCH6303_TOUCH_AddMessageReportRead
    or DRV_MTCH6303_TOUCH_AddMessageCommandWrite.
  
  Remarks:
    To be passed as first byte of message payload. Applicable only for first 
    fragment of message.
*/
typedef enum
{
    /* Message will echo the exact payload of a received "echo" command */
    DRV_TOUCH_MTCH6303_MSG_REPORT_ECHO /*DOM-IGNORE-BEGIN*/ = 0x04, /*DOM-IGNORE-END*/
    
    /* Message will read back flash contents */
    DRV_TOUCH_MTCH6303_MSG_REPORT_FLASH_CONTENTS /*DOM-IGNORE-BEGIN*/ = 0x17, /*DOM-IGNORE-END*/
            
    /* Message will output raw samples from ADC */
    DRV_TOUCH_MTCH6303_MSG_REPORT_ADC_DBG        /*DOM-IGNORE-BEGIN*/ = 0x60, /*DOM-IGNORE-END*/
            
    /* Message will output trace packets for particular event */
    DRV_TOUCH_MTCH6303_MSG_REPORT_TRACE          /*DOM-IGNORE-BEGIN*/ = 0x90, /*DOM-IGNORE-END*/
            
    /* Message will report swipe gesture */
    DRV_TOUCH_MTCH6303_MSG_REPORT_SWIPE          /*DOM-IGNORE-BEGIN*/ = 0xA0, /*DOM-IGNORE-END*/
            
    /* Message will report scroll gesture */
    DRV_TOUCH_MTCH6303_MSG_REPORT_SCROLL         /*DOM-IGNORE-BEGIN*/ = 0xA1, /*DOM-IGNORE-END*/
            
    /* Message will report tap gesture */
    DRV_TOUCH_MTCH6303_MSG_REPORT_TAP            /*DOM-IGNORE-BEGIN*/ = 0xA2, /*DOM-IGNORE-END*/
            
    /* Message will report noise data */
    DRV_TOUCH_MTCH6303_MSG_REPORT_NOISE          /*DOM-IGNORE-BEGIN*/ = 0xB0, /*DOM-IGNORE-END*/
            
    /* Message will report dynamic amount of nodes (from 1 to full RX electrode )*/
    DRV_TOUCH_MTCH6303_MSG_REPORT_MUT_NORM_SEC   /*DOM-IGNORE-BEGIN*/ = 0xC3, /*DOM-IGNORE-END*/
            
    /* Message will report parameter read response */
    DRV_TOUCH_MTCH6303_MSG_REPORT_PARAM_READ     /*DOM-IGNORE-BEGIN*/ = 0x04, /*DOM-IGNORE-END*/
            
    /* Message will acknowledge receipt of command */
    DRV_TOUCH_MTCH6303_MSG_REPORT_ACK            /*DOM-IGNORE-BEGIN*/ = 0xF0, /*DOM-IGNORE-END*/
            
    /* Message will report filtered(not scaled) touch coordinates */
    DRV_TOUCH_MTCH6303_MSG_REPORT_TOUCH_FILTERED /*DOM-IGNORE-BEGIN*/ = 0xF2, /*DOM-IGNORE-END*/
            
    /* Message will report prediction value for a touch */
    DRV_TOUCH_MTCH6303_MSG_REPORT_TOUCH_PREDICT  /*DOM-IGNORE-BEGIN*/ = 0xF3, /*DOM-IGNORE-END*/
            
    /* Message will report raw touch (pre-filter)*/
    DRV_TOUCH_MTCH6303_MSG_REPORT_TOUCH_RAW      /*DOM-IGNORE-BEGIN*/ = 0xF4, /*DOM-IGNORE-END*/
    
    /* Message will report final scaled touch. First byte, bit 7 = touch status */
    DRV_TOUCH_MTCH6303_MSG_REPORT_TOUCH_POS16    /*DOM-IGNORE-BEGIN*/ = 0xF5, /*DOM-IGNORE-END*/
            
    /* Message will report raw self measurements */
    DRV_TOUCH_MTCH6303_MSG_REPORT_SELF_RAW       /*DOM-IGNORE-BEGIN*/ = 0xFA, /*DOM-IGNORE-END*/
            
    /* Message will report normalized self measurements */
    DRV_TOUCH_MTCH6303_MSG_REPORT_SELF_NORM      /*DOM-IGNORE-BEGIN*/ = 0xFD, /*DOM-IGNORE-END*/
            
    /* Message will report packet from GestIC */
    DRV_TOUCH_MTCH6303_MSG_REPORT_FWD_GESTIC     /*DOM-IGNORE-BEGIN*/ = 0xFE, /*DOM-IGNORE-END*/
            
    /* Message will report firmware information */
    DRV_TOUCH_MTCH6303_MSG_REPORT_FW_VERSION     /*DOM-IGNORE-BEGIN*/ = 0xFF, /*DOM-IGNORE-END*/
            
    /* Message sends command to echo the payload sent */
    DRV_TOUCH_MTCH6303_MSG_CMD_ECHO              /*DOM-IGNORE-BEGIN*/ = 0x04, /*DOM-IGNORE-END*/
            
    /* Message sends command to read flash contents of device */
    DRV_TOUCH_MTCH6303_MSG_CMD_READ_FLASH        /*DOM-IGNORE-BEGIN*/ = 0x17, /*DOM-IGNORE-END*/
            
    /* Message sends command to firmware to enter into bootloader mode, ACK will be sent before entering */
    DRV_TOUCH_MTCH6303_MSG_CMD_ENTER_BOOTLDR     /*DOM-IGNORE-BEGIN*/ = 0x55, /*DOM-IGNORE-END*/
            
    /* Message sends command to write the parameter */
    DRV_TOUCH_MTCH6303_MSG_CMD_SET_PARAM         /*DOM-IGNORE-BEGIN*/ = 0xE0, /*DOM-IGNORE-END*/
     
    /* Message sends command to read the parameters */
    DRV_TOUCH_MTCH6303_MSG_CMD_GET_PARAM         /*DOM-IGNORE-BEGIN*/ = 0xE1, /*DOM-IGNORE-END*/
            
    /* Message sends command to force a baseline */
    DRV_TOUCH_MTCH6303_MSG_CMD_FORCE_BASE_lINE   /*DOM-IGNORE-BEGIN*/ = 0xFB, /*DOM-IGNORE-END*/
            
    /* Message sends command to reset GestIC*/
    DRV_TOUCH_MTCH6303_MSG_CMD_RESET_GESTIC      /*DOM-IGNORE-BEGIN*/ = 0xFC, /*DOM-IGNORE-END*/
    
    /* Message sends packet to GestIC */
    DRV_TOUCH_MTCH6303_MSG_CMD_SEND_GESTIC_PACKET /*DOM-IGNORE-BEGIN*/ = 0xFD, /*DOM-IGNORE-END*/
            
    /* Message sends firmware version query command. Bytes 124:127 = Rev[2].Minor.Major*/
    DRV_TOUCH_MTCH6303_MSG_CMD_QUERY_VERSION     /*DOM-IGNORE-BEGIN*/ = 0xE0, /*DOM-IGNORE-END*/
            
} DRV_TOUCH_MTCH6303_MSG_ID;

// *****************************************************************************
/* MTCH6303 Buffer Event Callback

  Function:
    void ( *DRV_MTCH6303_BUFFER_EVENT_HANDLER ) ( DRV_MTCH6303_BUFFER_EVENT event, 
                                                  DRV_MTCH6303_BUFFER_HANDLE bufferHandle, 
                                                  uintptr_t context )

  Summary:
    Points to a callback after completion of an register read -write or message
  stream read - write.

  Description:
    This type identifies the MTCH6303 Buffer Event. It allows the client driver
    to register a callback using DRV_MTCH6303_BUFFER_EVENT_HANDLER. By using this
    mechanism, the driver client will be notified at the completion of the
    corresponding transfer.

  Parameters:
    DRV_MTCH6303_BUFFER_EVENT - Status of MTCH6303 transfer

    bufferHandle - Handle that identifies the particular Buffer Object

    context     -  pointer to the object to be processed.

  Remarks:
    A transfer can be composed of various transfer segments.  Once a transfer
    is completed the driver will call the client registered transfer
    callback.

    The callback could be called from ISR context and should be kept as short
    as possible.  It is meant for signaling and it should not be blocking.

*/

typedef void ( *DRV_MTCH6303_BUFFER_EVENT_HANDLER ) (DRV_MTCH6303_BUFFER_EVENT event,
				DRV_MTCH6303_BUFFER_HANDLE bufferHandle, uintptr_t context );

// *****************************************************************************
/* MTCH6303 Touch Buffer Event Callback

  Function:
    void ( *DRV_MTCH6303_TOUCH_BUFFER_EVENT_HANDLER ) ( DRV_MTCH6303_TOUCH_BUFFER_EVENT event, 
                                                  DRV_MTCH6303_TOUCH_BUFFER_HANDLE bufferHandle, 
                                                  uintptr_t context )

  Summary:
    Points to a callback after completion of an message report read or message
    command write.

  Description:
    This type identifies the MTCH6303 Touch Buffer Event. It allows the client 
    driver to register a callback using DRV_MTCH6303_TOUCH_BUFFER_EVENT_HANDLER. 
    By using this mechanism, the driver client will be notified at the completion 
    of the corresponding transfer.

  Parameters:
    DRV_MTCH6303_TOUCH_BUFFER_EVENT - Status of MTCH6303 touch message transfer

    bufferHandle - Handle that identifies the particular Buffer Object

    context     -  pointer to the object to be processed.

  Remarks:
    A transfer can be composed of various transfer segments.  Once a transfer
    is completed the driver will call the client registered transfer
    callback.

    The callback could be called from ISR context and should be kept as short
    as possible.  It is meant for signaling and it should not be blocking.

*/

typedef void ( *DRV_MTCH6303_TOUCH_BUFFER_EVENT_HANDLER ) (DRV_MTCH6303_TOUCH_BUFFER_EVENT event,
                DRV_MTCH6303_TOUCH_BUFFER_HANDLE bufferHandle, uintptr_t context );

// *****************************************************************************
/* MTCH6303 Client-Specific Driver Status

  Summary:
    Defines the client-specific status of the MTCH6303 driver.

  Description:
    This enumeration defines the client-specific status codes of the MTCH6303
    driver.

  Remarks:
    Returned by the DRV_MTCH6303_ClientStatus function.
*/

typedef enum
{
    /* An error has occurred.*/
    DRV_MTCH6303_CLIENT_STATUS_ERROR    = DRV_CLIENT_STATUS_ERROR,

    /* The driver is closed, no operations for this client are ongoing,
    and/or the given handle is invalid. */
    DRV_MTCH6303_CLIENT_STATUS_CLOSED   = DRV_CLIENT_STATUS_CLOSED,

    /* The driver is currently busy and cannot start additional operations. */
    DRV_MTCH6303_CLIENT_STATUS_BUSY     = DRV_CLIENT_STATUS_BUSY,

    /* The module is running and ready for additional operations */
    DRV_MTCH6303_CLIENT_STATUS_READY    = DRV_CLIENT_STATUS_READY

} DRV_MTCH6303_CLIENT_STATUS;


// *****************************************************************************
/* MTCH6303 Driver Errors.

  Summary:
    Defines the possible errors that can occur during driver operation.

  Description:
    This data type defines the possible errors that can occur when occur during
    MTCH6303 driver operation. These values are returned by DRV_MTCH6303_ErrorGet
    function.

  Remarks:
    None
*/

typedef enum
{
    /* There was no error */
    DRV_MTCH6303_ERROR_NONE = 
            /*DOM-IGNORE-BEGIN*/ 0 /*DOM-IGNORE-END*/,

    /* Invalid address */
    DRV_MTCH6303_ERROR_INVALID_ADDRESS = 
            /*DOM-IGNORE-BEGIN*/ (1 << 0) /*DOM-IGNORE-END*/,            

} DRV_MTCH6303_ERROR;

// *****************************************************************************
/* MTCH6303 I2C touch status

  Summary:
    Defines the I2C touch status register bits 

  Description:
    This structure defines the I2C touch status register bits. 

  Remarks:
    It is part of DRV_MTCH6303_TOUCH_DATA structure. It is packed to form
    structure of size 1 byte.

*/

typedef struct __attribute__ ( ( __packed__ ) )
{
    /* Number of available touch packets */
    uint32_t nTouch : 4;

    /* stream data ready */
    uint32_t streamReady : 1;

    /* gesture data ready */
    uint32_t gestureReady : 1;

    /* GestIC data ready */
    uint32_t gestICData : 1;

    /* reserved bit */
    uint32_t reserved : 1;

} DRV_MTCH6303_TOUCH_STATUS;

// *****************************************************************************
/* MTCH6303 I2C Touch Input Packet Nibble 0 

  Summary:
   Defines the I2C Nibble 0 of MTCH6303 Touch input packet.

  Description:
   This structure defines the I2C Nibble 0 of MTCH6303 Touch input packet.

  Remarks:
    It is part of DRV_MTCH6303_TOUCH_INPUT structure. It is packed to form
    structure of size 1 byte.

*/

typedef struct __attribute__ ( ( __packed__ ) )
{
    /* Touch packet available */
    uint32_t touchState : 1;

    /* Touch packet in range */
    uint32_t inRange : 1;

    /* Reserved bits */
    uint32_t reserved : 6;

} DRV_MTCH6303_TOUCH_NIBBLE_0;

// *****************************************************************************
/* MTCH6303 Touch Input Packet.

  Summary:
   Defines MTCH6303 Touch Input Packet

  Description:
   This structure defines the MTCH6303 Touch Input Packet.

  Remarks:
   It is part of DRV_MTCH6303_TOUCH_DATA structure. It is packed to form
    structure of size 6 bytes.

*/

typedef struct __attribute__ ( ( __packed__ ) )
{
    /* MTCH6303 I2C Touch Input Packet Nibble 0 */
    DRV_MTCH6303_TOUCH_NIBBLE_0 nibble_0;

    /* MTCH6303 I2C Touch Input Packet ID (0 - 16) */
    uint8_t touchId;

    /* MTCH6303 I2C Touch Input Packet position x (0 - 0x7FFF) */
    uint16_t x;

    /* MTCH6303 I2C Touch Input Packet position y (0 - 0x7FFF) */
    uint16_t y;

} DRV_MTCH6303_TOUCH_INPUT;

// *****************************************************************************
/* MTCH6303 I2C Touch Data

  Summary:
   Defines MTCH6303 I2C Touch Data

  Description:
   This structure defines MTCH6303 I2C Touch Data. The structure 
   DRV_MTCH6303_TOUCH_DATA is passed to API's DRV_MTCH6303_AddRegisterRead or
   DRV_MTCH6303_TOUCH_AddTouchInputRead. The API's will update the structure
   with touch input. 

  Remarks:
    It is packed to form structure of size 62 bytes. The structure member 
  i2cReadAddr is only applicable if the I2C driver is of type bitbang. Otherwise
  the variable required to be commented out. 

*/

typedef struct __attribute__ ( ( __packed__ ) ) 
{
    /* Dummy I2C Read Address required for bitbang driver */
    uint8_t i2cReadAddr;

    /* MTCH6303 Touch Status */
    DRV_MTCH6303_TOUCH_STATUS status;

    /* MTCH6303 Touch Input array of size DRV_MTCH6303_TOUCH_NUM_INPUTS */
    DRV_MTCH6303_TOUCH_INPUT  touch [ DRV_MTCH6303_TOUCH_NUM_INPUTS ];

} DRV_MTCH6303_TOUCH_DATA;

// *****************************************************************************
/* MTCH6303 Touch Message Header

  Summary:
   Defines Touch Message Header.

  Description:
   This structure defines Touch Message Header.

  Remarks:
   It is part of structure DRV_MTCH6303_TOUCH_MESSAGE. It is packed to form 
   structure of size 1 byte.

*/

typedef struct __attribute__ ( ( __packed__ ) )
{
    /* MTCH6303 Message Fragment Size. If Message Fragment size is 0x3F the 
       Fragment is incomplete and uses up ALL of the parent transport layer
       packet. */
    uint32_t msgFragSize:  6;

    /* MTCH6303 Message continued from last fragment if set to 1. */
    uint32_t continued:    1;

    /* MTCH6303 more messages to follow in this block if set to 1.*/
    uint32_t moreMessages: 1;

} DRV_MTCH6303_TOUCH_MESSAGE_HEADER;

// *****************************************************************************
/* MTCH6303 Touch Message

  Summary:
   Defines MTCH6303 Touch Message.

  Description:
   This structure defines MTCH6303 Touch Message. The variable pointer of type
   DRV_MTCH6303_TOUCH_MESSAGE is passed to the API's 
   DRV_MTCH6303_TOUCH_AddMessageReportRead or 
   DRV_MTCH6303_TOUCH_AddMessageCommandWrite.

  Remarks:
    It is packed to form structure of size 63 bytes.

*/

typedef struct __attribute__ ( ( __packed__ ) )
{
    /* MTCH6303 Touch Message Header */
    DRV_MTCH6303_TOUCH_MESSAGE_HEADER  header;

    /* MTCH6303 Touch Message payload. First byte of payload is of type 
       DRV_TOUCH_MTCH6303_MSG_ID in case of first fragment of message. Otherwise
       the first byte acts as a normal payload. */
    uint8_t                            payload[0x3E];

} DRV_MTCH6303_TOUCH_MESSAGE;

// *********************************************************************************************
// *********************************************************************************************
// Section: System Interface Headers for the Instance 0 of MTCH6303 static driver
// *********************************************************************************************
// *********************************************************************************************

// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_MTCH6303_Initialize ( void ) 

  Summary:
    Initializes the MTCH6303 static single instance. 

  Description:
    This routine initializes the MTCH6303 static driver instance. It makes the
    instance ready for a client to open and use it. The instance parameters are
    initialized by values set by MPLAB Harmony Configurator. 
 
  Precondition:
    None.

  Parameters:
    None.

  Returns:
    If successful, returns a valid handle to a driver instance object. Otherwise,
    returns SYS_MODULE_OBJ_INVALID.

  Example:
    <code>
    // The following code snippet shows an example MTCH6303 driver initialization.
    
    SYS_MODULE_OBJ    objectHandle;

    objectHandle = DRV_MTCH6303_Initialize();
    if( SYS_MODULE_OBJ_INVALID == objectHandle )
    {
       // Handle error
    }
    <code>

  Remarks:
    This routine must be called before any other MTCH6303 routine is called.
    
    This routine should only be called once during system initialization unless
    DRV_MTCH6303_Deinitialize is called to deinitialize the driver instance.
    This routine will NEVER block for hardware access.
*/

SYS_MODULE_OBJ DRV_MTCH6303_Initialize   ( void );

// *****************************************************************************
/* Function:
    void DRV_MTCH6303_Deinitialize( void ) 

  Summary:
    Deinitializes the instance of the MTCH6303 driver module.

  Description:
    Deinitializes the instance of the MTCH6303 driver module, disabling its 
    operation. Invalidates all the internal data.

  Precondition:
    Function DRV_MTCH6303_Initialize should have been called before calling this
    function.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    SYS_STATUS    status;
    
    DRV_MTCH6303_Deinitialize();
    
    status = DRV_MTCH6303_Status();
    if(SYS_MODULE_DEINITIALIZED != status)
    {
        //check again later if you need to know
        //when the driver is deinitialized
    }
    </code>

  Remarks:
    once the initialize operation has been called, the Deinitialize operation
    must be called before the Initialize operation can be called again. this 
    routine will NEVER block waiting for hardware.
    
*/

void DRV_MTCH6303_Deinitialize ( void );

// *****************************************************************************
/* Function:
    SYS_STATUS DRV_MTCH6303_Status( void )

  Summary:
    Gets the current status of the MTCH6303 driver module.

  Description:
    This routine provides the current status of the MTCH6303 driver module.

  Precondition:
    Function DRV_MTCH6303_Initialize should have been called before calling this
    function.

  Parameters:
    None.

  Returns:
    SYS_STATUS_READY - Indicates that the driver is busy with a previous system
                       level operation and cannot start another.
    
    SYS_STATUS_DEINITIALIZED - Indicates that the driver has been deinitialized.
 
  Example:
    <code>
    
    SYS_STATUS mtch6303Status;
    
    mtch6303Status = DRV_MTCH6303_Status();
    if(SYS_STATUS_READY == mtch6303Status)
    {
        // This means the driver can be opened using the
        // DRV_MTCH6303_Open() function.
    }
    </code>

  Remarks:
    A driver can opened only when its status is SYS_STATUS_READY.    

*/

SYS_STATUS     DRV_MTCH6303_Status       ( void );

// *****************************************************************************
/* Function:
    void DRV_MTCH6303_Tasks( void )

  Summary:
    Maintains the driver's register read/write state machine and implements its 
    ISR.

  Description:
    This routine is used to maintain the driver's register read/write state 
    machine and implement its ISR for interrupt-driven implementations.
    In interrupt mode, this function is called in I2C Driver event Handler
    routine. The I2C Driver event Handler routine is registered by MTCH6303 
    event Handler register routine.

  Precondition:
    Function DRV_MTCH6303_Initialize should have been called before calling this
    function. It also needs registration of the MTCH6303 Driver event handler 
    routine.
    
  Parameters:
    None.

  Returns:
    None.

  Remarks:    
    This routine may execute in an ISR context and will never block or access any
    resources that may cause it to block.
*/

void DRV_MTCH6303_Tasks( void );

// *****************************************************************************
/* Function:
    void DRV_MTCH6303_TOUCH_Tasks( void )

  Summary:
    Maintains the driver's message state machine and implements its ISR.

  Description:
    This routine is used to maintain the driver's message state machine and 
    implement its ISR for interrupt-driven implementations.
    In interrupt mode, this function is called in I2C Driver event Handler
    routine. The I2C Driver event Handler routine is registered by MTCH6303 Touch
    event Handler register routine.

  Precondition:
    Function DRV_MTCH6303_Initialize should have been called before calling this
    function. It also needs registration of the MTCH6303 Driver Touch event handler 
    routine.
    
  Parameters:
    None.

  Returns:
    None.

  Remarks:    
    This routine may execute in an ISR context and will never block or access any
    resources that may cause it to block.
*/

void DRV_MTCH6303_TOUCH_Tasks( void );

// *********************************************************************************************
// *********************************************************************************************
// Section: General Client Interface Headers for the Instance 0 of MTCH6303 static driver
// *********************************************************************************************
// *********************************************************************************************

// *****************************************************************************
/* Function:
   DRV_HANDLE DRV_MTCH6303_Open { void }

  Summary:
   Opens the MTCH6303 driver instance and returns a handle to it.

  Description:
   This routine opens the specified MTCH6303 driver instance and provides a 
   handle. 

  Precondition:
   Function DRV_MTCH6303_Initialize must have been called before calling this
   function.

  Parameters:
   None.

  Returns:
   If successful, the routine returns a valid open-instance handle.
   If an error occurs, the return value is DRV_HANDLE_INVALID. Error can occur
   - if the driver is not ready to be opened, typically when the initialize
      routine has not completed execution.
   - if the bus driver fails to open
   - if the client is trying to open the driver but driver has been opened
      exclusively by another client.

Example:
    <code>
    DRV_HANDLE handle;

    handle = DRV_MTCH6303_Open( );
    if (DRV_HANDLE_INVALID == handle)
    {
        // Unable to open the driver
        // May be the driver is not initialized or the initialization
        // is not complete.
    }
    </code>

  Remarks:
    The handle returned is valid until the DRV_MTCH6303_Close routine is called.
    This routine will NEVER block waiting for hardware.If the requested intent
    flags are not supported, the routine will return DRV_HANDLE_INVALID.  This
    function is thread safe in a RTOS application. 

*/

DRV_HANDLE DRV_MTCH6303_Open ( void );

// *****************************************************************************
/* Function:
    DRV_MTCH6303_CLIENT_STATUS DRV_MTCH6303_Close ( void )

  Summary:
    Closes an opened-instance of the MTCH6303 driver.

  Description:
    This routine closes an opened-instance of the MTCH6303 driver. Any buffers 
    in the driver queue that were submitted by this client will be removed. 
    DRV_MTCH6303_Open must be called to before using the driver again.

  Precondition:
    The DRV_MTCH6303_Initialize routine must have been called. 
    DRV_MTCH6303_Open must have been called.

  Parameters:
    None. 

  Returns:
    DRV_MTCH6303_CLIENT_STATUS_ERROR - if driver fails to remove buffer objects 
                                       from queue.

    DRV_MTCHC6303_CLIENT_STATUS_CLOSED - client is successfully closed
    
  Example:
    <code>
    
    DRV_MTH6303_CLIENT_STATUS mtch6303Status;

    mtch6303Status = DRV_MTCH6303_Close()
    if( DRV_MTCH6303_CLIENT_STATUS_ERROR == mtch6303Status )
    {
        //retry closing the driver client
    }
    </code>

  Remarks:
    The driver will abort any ongoing operations when this routine is called.    

*/

DRV_MTCH6303_CLIENT_STATUS DRV_MTCH6303_Close ( void );

// *****************************************************************************
/* Function:
    DRV_MTCH6303_ERROR DRV_MTCH6303_ErrorGet ( void )

  Summary:
    This function returns the error associated with the last client request.

  Description:
    This function returns the error associated with the last client request.

  Precondition:
    The DRV_MTCH6303_Initialize routine must have been called. DRV_MTCH6303_Open
    must have been called to open a device client.

  Parameters:
    None.

  Returns:
    DRV_MTCH6303_ERROR_NONE - no error
 
  Remarks:
    This routine always return DRV_MTCH6303_ERROR_NONE the client error is
    currently not updated by any of the MTCH6303 operations API's.

*/

DRV_MTCH6303_ERROR DRV_MTCH6303_ErrorGet ( void );

// *****************************************************************************
/* Function:
    void DRV_MTCH6303_BufferEventHandlerSet
    (
        const DRV_MTCH6303_BUFFER_EVENT_HANDLER eventHandler,
        const uintptr_t context
    )

  Summary:
    Allows a client to identify a buffer event handling function for the driver
    to call back when queued buffer transfers have finished.

  Description:
    This function allows a client to identify a buffer event handling function 
    for the driver to call back when queued buffer transfers have finished.  
    When a client calls either the DRV_MTCH6303_TouchInputRead, 
    DRV_MTCH6303_AddRegisterRead or DRV_MTCH6303_AddRegisterWrite function, it 
    is provided with a handle identifying the buffer that was added to the 
    driver's buffer queue. The driver will pass this handle back to the client 
    by calling "eventHandler" function when the buffer transfer has completed.
    
    The event handler should be set before the client performs any "buffer add"
    operations that could generate events. The event handler once set, persists 
    until the client closes the driver or sets another event handler (which 
    could be a "NULL" pointer to indicate no callback).

  Precondition:
    The DRV_MTCH6303_Initialize routine must have been called and the 
    DRV_MTCH6303_Status must have returned SYS_STATUS_READY.

  Parameters:
    
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
    // myAppObj is an application specific state data object.
    MY_APP_OBJ myAppObj;

    uint8_t mybuffer[MY_BUFFER_SIZE];

    // myMTCH6303Handle is the handle returned 
    // by the DRV_MTCH6303_Open function.
    
    // Client registers an event handler with driver. This is done once

    DRV_MTCH6303_BufferEventHandlerSet( APP_MTCH6303BufferEventHandle, 
                                     (uintptr_t)&myAppObj );

    DRV_MTCH6303_AddRegisterRead( &bufferHandle  
                                   DRV_MTCH6303_REG_TOUCH_STATUS,
                                   MY_BUFFER_SIZE,
                                   &mybuffer);

    if(DRV_MTCH6303_BUFFER_HANDLE_INVALID == bufferHandle)
    {
        // Error handling here
    }

    // Event Processing Technique. Event is received when
    // the buffer is processed.

    void APP_MTCH6303BufferEventHandle( DRV_MTCH6303_BUFFER_EVENT event, 
                                        DRV_MTCH6303_BUFFER_HANDLE handle, 
                                        uintptr_t context)
    {
        // contextHandle points to myAppObj.

        switch(event)
        {
            case DRV_MTCH6303_BUFFER_EVENT_COMPLETE:

                // This means the data was transferred. 
                break;
            
            case DRV_MTCH6303_BUFFER_EVENT_ERROR:

                // Error handling here.

                break;

            default:
                break;
        }
    }
    </code>

  Remarks:
    None.
*/

void DRV_MTCH6303_BufferEventHandlerSet
(
    const DRV_MTCH6303_BUFFER_EVENT_HANDLER eventHandler,
    const uintptr_t context
);

// *****************************************************************************
/* Function:
    void DRV_MTCH6303_TOUCH_BufferEventHandlerSet
    (
        const DRV_MTCH6303_TOUCH_BUFFER_EVENT_HANDLER eventHandler,
        const uintptr_t context
    )

  Summary:
    Allows a client to identify a buffer event handling function for the driver
    to call back when queued message transfers have finished.

  Description:
    This function allows a client to identify a message event handling function 
    for the driver to call back when queued message transfers have finished.  
    When a client calls either the DRV_MTCH6303_TOUCH_AddTouchInputRead, 
    DRV_MTCH6303_TOUCH_AddMessageReportRead or 
    DRV_MTCH6303_TOUCH_AddMessageCommandWrite function, it is provided with a 
    handle identifying the message that was added to the driver's message queue. 
    The driver will pass this handle back to the client by calling "eventHandler" 
    function when the message transfer has completed.
    
    The event handler should be set before the client performs any "message add"
    operations that could generate events. The event handler once set, persists 
    until the client closes the driver or sets another event handler (which 
    could be a "NULL" pointer to indicate no callback).

  Precondition:
    The DRV_MTCH6303_Initialize routine must have been called and the 
    DRV_MTCH6303_Status must have returned SYS_STATUS_READY.

  Parameters:
    
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
    // myAppObj is an application specific state data object.
    MY_APP_OBJ myAppObj;
    DRV_MTCH6303_TOUCH_MESSAGE messageReport;
    DRV_MTCH6303_TOUCH_BUFFER_HANDLE bufferHandle;

    // myMTCH6303Handle is the handle returned 
    // by the DRV_MTCH6303_Open function.
    
    // Client registers an event handler with driver. This is done once

    DRV_MTCH6303_TOUCH_BufferEventHandlerSet( APP_MTCH6303BufferEventHandler, 
                                     (uintptr_t)&myAppObj );

    DRV_MTCH6303_TOUCH_AddMessageReportRead( &bufferHandle, &messageReport );

    if(DRV_MTCH6303_TOUCH_BUFFER_HANDLE_INVALID == bufferHandle)
    {
        // Error handling here
    }

    // Event is received when
    // the buffer is processed.

    void APP_MTCH6303BufferEventHandler( DRV_MTCH6303_TOUCH_BUFFER_EVENT event, 
                                         DRV_MTCH6303_TOUCH_BUFFER_HANDLE bufferHandle,
                                         uintptr_t contextHandle )
    {
        // contextHandle points to myAppObj.

        switch(event)
        {
            case DRV_MTCH6303_TOUCH_BUFFER_EVENT_COMPLETE:

                // This means the data was transferred. 
                break;
            
            case DRV_MTCH6303_TOUCH_BUFFER_EVENT_ERROR:

                // Error handling here.

                break;

            default:
                break;
        }
    }
    </code>

  Remarks:
    None.
*/

void DRV_MTCH6303_TOUCH_BufferEventHandlerSet
(
    const DRV_MTCH6303_TOUCH_BUFFER_EVENT_HANDLER eventHandler,
    const uintptr_t context
);

// *****************************************************************************
/* Function:
    void DRV_MTCH6303_TouchInputRead( DRV_MTCH6303_BUFFER_HANDLE * bufferHandle,
                                      DRV_MTCH6303_TOUCH_DATA    * touchData )

  Summary:
    Schedules a non-blocking read buffer request to read touch input from MTCH6303.

  Description:
    This function schedules a non-blocking read buffer request to read touch 
    input from MTCH6303.  The function returns with a valid buffer handle in the
    bufferHandle argument if the read request was scheduled successfully. The 
    function adds the request to the hardware instance queue and returns
    immediately. The function returns DRV_MTCH6303_BUFFER_HANDLE_INVALID in the 
    bufferHandle argument:
    - if a buffer could not be allocated to the request
    - if the input buffer pointer is NULL
    - if the buffer size is 0 
    - if the read queue size is full or queue depth is insufficient.
    - if the driver handle is invalid 

    If the requesting client registered an event callback with the driver, the
    driver will issue a DRV_MTCH6303_BUFFER_EVENT_COMPLETE event if the buffer 
    was processed successfully or DRV_MTCH6303_BUFFER_EVENT_ERROR event if the 
    buffer was not processed successfully. The touch data is collected into 
    touchData and can be read once a buffer event complete is reported. A event 
    handler is called on buffer event complete where the touch data must be read
    from touchData.

  Precondition:
    The DRV_MTCH6303_Initialize routine must have been called and the 
    DRV_MTCH6303_Status must have returned SYS_STATUS_READY.

    DRV_MTCH6303_Open must have been called to obtain a valid opened device 
    handle.

  Parameters:
    bufferHandle - Handle to the buffer scheduled.
    
    touchData - Buffer collecting touch data.

  Returns:
    None.

  Example
    <code>
    MY_APP_OBJ myAppObj;    
    DRV_MTCH6303_TOUCH_DATA touchData;
    DRV_MTCH6303_BUFFER_HANDLE bufferHandle;

    // Client registers an event handler with driver

    DRV_MTCH6303_BufferEventHandlerSet( APP_MTCH6303BufferEventHandler, 
                                       (uintptr_t)&myAppObj);

    DRV_MTCH6303_TouchInputRead( &bufferHandle, &touchData );

    if(DRV_MTCH6303_BUFFER_HANDLE_INVALID == bufferHandle)
    {
        // Error handling here
    }

    // Event is received when
    // the buffer is processed.

    void APP_MTCH6303BufferEventHandler( DRV_MTCH6303_BUFFER_EVENT event, 
                                         DRV_MTCH6303_BUFFER_HANDLE bufferHandle,
                                         uintptr_t contextHandle )
    {
        // contextHandle points to myAppObj.

        switch(event)
        {
            case DRV_MTCH6303_BUFFER_EVENT_COMPLETE:

                // This means the data was transferred. 
                break;
            
            case DRV_MTCH6303_BUFFER_EVENT_ERROR:

                // Error handling here.

                break;

            default:
                break;
        }
    }
    </code>

  Remarks:
    None.
    
*/

void DRV_MTCH6303_TouchInputRead( DRV_MTCH6303_BUFFER_HANDLE * bufferHandle,
                                  DRV_MTCH6303_TOUCH_DATA    * touchData );

// *****************************************************************************
/* Function:
    void DRV_MTCH6303_AddRegisterRead( DRV_MTCH6303_BUFFER_HANDLE * bufferHandle,
                                       uint8_t source,
                                       size_t  nBytes,
                                       uint8_t * destination )

  Summary:
    Schedules a non-blocking register read request to read I2C accessible MTCH6303
    registers.

  Description:
    This function schedules a non-blocking register read request to read I2C
    accessible MTCH6303 registers. The function returns with a valid buffer handle 
    in the bufferHandle argument if the register read request was scheduled 
    successfully. The function adds the request to the hardware instance queue and 
    returns immediately. The function returns DRV_MTCH6303_BUFFER_HANDLE_INVALID 
    in the bufferHandle argument:
    - if a buffer could not be allocated to the request
    - if the input buffer pointer is NULL
    - if the buffer size is 0 
    - if the read queue size is full or queue depth is insufficient.
    - if the driver handle is invalid 

    If the requesting client registered an event callback with the driver, the
    driver will issue a DRV_MTCH6303_BUFFER_EVENT_COMPLETE event if the buffer 
    was processed successfully or DRV_MTCH6303_BUFFER_EVENT_ERROR event if the 
    buffer was not processed successfully. The register data is collected into 
    destination and can be read once a buffer event complete is reported. A event 
    handler is called on buffer event complete where the register data must be read
    from destination.

   Precondition:
    The DRV_MTCH6303_Initialize routine must have been called and the 
    DRV_MTCH6303_Status must have returned SYS_STATUS_READY.

    DRV_MTCH6303_Open must have been called to obtain a valid opened device 
    handle.

  Parameters:
    bufferHandle - Handle to the buffer scheduled.
    
    source - Register index.

    nBytes - Number of registers to be read, starting from source.

    destination - buffer collecting register data.

  Returns:
    None.

  Example
    <code>
    MY_APP_OBJ myAppObj;    
    uint8_t registerData[NUM_REGISTERS];
    DRV_MTCH6303_BUFFER_HANDLE bufferHandle;

    // Client registers an event handler with driver

    DRV_MTCH6303_BufferEventHandlerSet( APP_MTCH6303BufferEventHandler, 
                                       (uintptr_t)&myAppObj);

    DRV_MTCH6303_AddRegisterRead( &bufferHandle,
                                  DRV_MTCH6303_REG_TOUCH_STATUS,
                                  NUM_REGISTERS,
                                  &registerData );

    if(DRV_MTCH6303_BUFFER_HANDLE_INVALID == bufferHandle)
    {
        // Error handling here
    }

    // Event is received when
    // the buffer is processed.

    void APP_MTCH6303BufferEventHandler( DRV_MTCH6303_BUFFER_EVENT event, 
                                         DRV_MTCH6303_BUFFER_HANDLE bufferHandle,
                                         uintptr_t contextHandle )
    {
        // contextHandle points to myAppObj.

        switch(event)
        {
            case DRV_MTCH6303_BUFFER_EVENT_COMPLETE:

                // This means the data was transferred. 
                break;
            
            case DRV_MTCH6303_BUFFER_EVENT_ERROR:

                // Error handling here.

                break;

            default:
                break;
        }
    }
    <code>

  Remarks:
    None.

*/

void DRV_MTCH6303_AddRegisterRead( DRV_MTCH6303_BUFFER_HANDLE * bufferHandle,
                                   uint8_t source,
                                   size_t  nBytes,
                                   uint8_t * destination );

// *****************************************************************************
/* Function:
    void DRV_MTCH6303_AddRegisterWrite( DRV_MTCH6303_BUFFER_HANDLE * bufferHandle,
                                        uint8_t destination,
                                        size_t  nBytes,
                                        uint8_t * source )

  Summary:
    Schedule a non-blocking driver register write operation to write I2C 
    accessible MTCH6303 registers.

  Description:
    This function schedules a non-blocking register write request to write I2C
    accessible MTCH6303 registers. The function returns with a valid buffer handle 
    in the bufferHandle argument if the register write request was scheduled 
    successfully. The function adds the request to the hardware instance queue and 
    returns immediately. While the request is in the queue, the application buffer 
    is owned by the driver and should not be modified. The function returns 
    DRV_MTCH6303_BUFFER_HANDLE_INVALID in the bufferHandle argument:
    - if a buffer could not be allocated to the request
    - if the input buffer pointer is NULL
    - if the buffer size is 0 
    - if the write queue size is full or queue depth is insufficient.
    - if the driver handle is invalid 

    If the requesting client registered an event callback with the driver, the
    driver will issue a DRV_MTCH6303_BUFFER_EVENT_COMPLETE event if the buffer 
    was processed successfully or DRV_MTCH6303_BUFFER_EVENT_ERROR event if the 
    buffer was not processed successfully. A event handler is called on buffer 
    event complete where the application data is written to the I2C accessible
    MTCH6303 Register. 

  Precondition:
    The DRV_MTCH6303_Initialize routine must have been called and the 
    DRV_MTCH6303_Status must have returned SYS_STATUS_READY.

    DRV_MTCH6303_Open must have been called to obtain a valid opened device 
    handle.

  Parameters:
    bufferHandle - Pointer to an argument that will contain the return buffer handle.

    destination  - Index to the start of destination register list.

    nBytes       - number of registers.

    source       - pointer to the data to be written to the register.

  Returns:
    None.

  Example:
    <code>

    MY_APP_OBJ myAppObj;    
    uint8_t registerData[NUM_REGISTERS];
    DRV_MTCH6303_BUFFER_HANDLE bufferHandle;

    // Client registers an event handler with driver

    DRV_MTCH6303_BufferEventHandlerSet( APP_MTCH6303BufferEventHandler, 
                                       (uintptr_t)&myAppObj);

    DRV_MTCH6303_AddRegisterWrite( &bufferHandle,
                                   DRV_MTCH6303_REG_TOUCH_STATUS,
                                   NUM_REGISTERS,
                                   &registerData );

    if(DRV_MTCH6303_BUFFER_HANDLE_INVALID == bufferHandle)
    {
        // Error handling here
    }

    // Event is received when
    // the buffer is processed.

    void APP_MTCH6303BufferEventHandler( DRV_MTCH6303_BUFFER_EVENT event, 
                                         DRV_MTCH6303_BUFFER_HANDLE bufferHandle,
                                         uintptr_t contextHandle )
    {
        // contextHandle points to myAppObj.

        switch(event)
        {
            case DRV_MTCH6303_BUFFER_EVENT_COMPLETE:

                // This means the data was transferred. 
                break;
            
            case DRV_MTCH6303_BUFFER_EVENT_ERROR:

                // Error handling here.

                break;

            default:
                break;
        }
    }

    </code>

  Remarks:
    None.
*/

void DRV_MTCH6303_AddRegisterWrite( DRV_MTCH6303_BUFFER_HANDLE * bufferHandle,
                                    uint8_t destination,
                                    size_t  nBytes,
                                    uint8_t * source );

// *****************************************************************************
/* Function: 
    void DRV_MTCH6303_TOUCH_AddTouchInputRead
    (    DRV_MTCH6303_TOUCH_BUFFER_HANDLE * bufferHandle,
         DRV_MTCH6303_TOUCH_DATA          * touchData )

  Summary:
    Schedules a non-blocking read buffer request to read touch input from MTCH6303.

  Description:
    This function schedules a non-blocking read buffer request to read touch 
    input from MTCH6303.  The function returns with a valid buffer handle in the
    bufferHandle argument if the read request was scheduled successfully. The 
    function adds the request to the hardware instance queue and returns
    immediately. The function returns DRV_MTCH6303_TOUCH_BUFFER_HANDLE_INVALID 
    in the bufferHandle argument:
    - if a buffer could not be allocated to the request
    - if the input buffer pointer is NULL
    - if the buffer size is 0 
    - if the read queue size is full or queue depth is insufficient.
    - if the driver handle is invalid 

    If the requesting client registered an event callback with the driver, the
    driver will issue a DRV_MTCH6303_TOUCH_BUFFER_EVENT_COMPLETE event if the 
    buffer was processed successfully or DRV_MTCH6303_TOUCH_BUFFER_EVENT_ERROR 
    event if the buffer was not processed successfully. The touch data is 
    collected into touchData and can be read once a buffer event complete is 
    reported. A event handler is called on buffer event complete where the touch
    data must be read from touchData.

  Precondition:
    The DRV_MTCH6303_Initialize routine must have been called and the 
    DRV_MTCH6303_Status must have returned SYS_STATUS_READY.

    DRV_MTCH6303_Open must have been called to obtain a valid opened device 
    handle.

  Parameters:
    bufferHandle - Handle to the buffer scheduled.
    
    touchData - Buffer collecting touch data.

  Returns:
    None.

  Example
    <code>
    MY_APP_OBJ myAppObj;    
    DRV_MTCH6303_TOUCH_DATA touchData;
    DRV_MTCH6303_BUFFER_HANDLE bufferHandle;

    // Client registers an event handler with driver

    DRV_MTCH6303_TOUCH_BufferEventHandlerSet( APP_MTCH6303BufferEventHandler, 
                                              (uintptr_t)&myAppObj);

    DRV_MTCH6303_TOUCH_AddTouchInputRead( &bufferHandle, &touchData );

    if(DRV_MTCH6303_TOUCH_BUFFER_HANDLE_INVALID == bufferHandle)
    {
        // Error handling here
    }

    // Event is received when
    // the buffer is processed.

    void APP_MTCH6303BufferEventHandler( DRV_MTCH6303_TOUCH_BUFFER_EVENT event, 
                                         DRV_MTCH6303_TOUCH_BUFFER_HANDLE bufferHandle,
                                         uintptr_t contextHandle )
    {
        // contextHandle points to myAppObj.

        switch(event)
        {
            case DRV_MTCH6303_TOUCH_BUFFER_EVENT_COMPLETE:

                // This means the data was transferred. 
                break;
            
            case DRV_MTCH6303_TOUCH_BUFFER_EVENT_ERROR:

                // Error handling here.

                break;

            default:
                break;
        }
    }
    </code>

  Remarks:
    None.
    
*/

void DRV_MTCH6303_TOUCH_AddTouchInputRead
( 
    DRV_MTCH6303_TOUCH_BUFFER_HANDLE * bufferHandle,
    DRV_MTCH6303_TOUCH_DATA          * touchData 
);

// *****************************************************************************
/* Function: 
    void DRV_MTCH6303_TOUCH_AddMessageReportRead
    (   DRV_MTCH6303_TOUCH_BUFFER_HANDLE * bufferHandle,
        DRV_MTCH6303_TOUCH_MESSAGE       * messageRep,
        size_t                             messageSize )

  Summary:
    Schedules a non-blocking report message read request to read the report
    message from MTCH6303 device.

  Description:
    This function schedules a non-blocking report message read request to read
    the report message from MTCH6303 device. The function returns with a valid 
    buffer handle in the bufferHandle argument if the register read request was 
    scheduled successfully. The function adds the request to the hardware instance 
    queue and returns immediately. The function returns 
    DRV_MTCH6303_TOUCH_BUFFER_HANDLE_INVALID in the bufferHandle argument:
    - if a buffer could not be allocated to the request
    - if the input buffer pointer is NULL
    - if the buffer size is 0 
    - if the read queue size is full or queue depth is insufficient.
    - if the driver handle is invalid 

    If the requesting client registered an event callback with the driver, the
    driver will issue a DRV_MTCH6303_TOUCH_BUFFER_EVENT_COMPLETE event if the 
    buffer was processed successfully or DRV_MTCH6303_TOUCH_BUFFER_EVENT_ERROR 
    event if the buffer was not processed successfully. The register data is 
    collected into destination and can be read once a buffer event complete is 
    reported. A event handler is called on buffer event complete where the 
    register data must be read from destination.

   Precondition:
    The DRV_MTCH6303_Initialize routine must have been called and the 
    DRV_MTCH6303_Status must have returned SYS_STATUS_READY.

    DRV_MTCH6303_Open must have been called to obtain a valid opened device 
    handle.

  Parameters:
    bufferHandle - Handle to the buffer scheduled.
    
    messageRep - report message buffer.

    messageSize - report message size. It includes message header and payload size.

  Returns:
    None.

  Example
    <code>
    MY_APP_OBJ myAppObj;    
    DRV_MTCH6303_TOUCH_MESSAGE messageReport;
    DRV_MTCH6303_TOUCH_BUFFER_HANDLE bufferHandle;

    // Client registers an event handler with driver

    DRV_MTCH6303_TOUCH_BufferEventHandlerSet( APP_MTCH6303BufferEventHandler, 
                                              (uintptr_t)&myAppObj);

    DRV_MTCH6303_TOUCH_AddMessageReportRead( &bufferHandle, 
                                             &messageReport,
                                             MY_MESSAGE_SIZE );

    if(DRV_MTCH6303_TOUCH_BUFFER_HANDLE_INVALID == bufferHandle)
    {
        // Error handling here
    }

    // Event is received when
    // the buffer is processed.

    void APP_MTCH6303BufferEventHandler( DRV_MTCH6303_TOUCH_BUFFER_EVENT event, 
                                         DRV_MTCH6303_TOUCH_BUFFER_HANDLE bufferHandle,
                                         uintptr_t contextHandle )
    {
        // contextHandle points to myAppObj.

        switch(event)
        {
            case DRV_MTCH6303_TOUCH_BUFFER_EVENT_COMPLETE:

                // This means the data was transferred. 
                break;
            
            case DRV_MTCH6303_TOUCH_BUFFER_EVENT_ERROR:

                // Error handling here.

                break;

            default:
                break;
        }
    }
    </code>

  Remarks:
    None.

*/

void DRV_MTCH6303_TOUCH_AddMessageReportRead
( 
    DRV_MTCH6303_TOUCH_BUFFER_HANDLE * bufferHandle,
    DRV_MTCH6303_TOUCH_MESSAGE       * messageRep,
    size_t                             messageSize
);

// *****************************************************************************
/* Function: 
    void DRV_MTCH6303_TOUCH_AddMessageCommandWrite
    (   DRV_MTCH6303_TOUCH_BUFFER_HANDLE   * bufferHandle,
        DRV_MTCH6303_TOUCH_MESSAGE         * messageCmd,
        size_t                               messageSize )

  Summary:
    Schedule a non-blocking driver command message write operation to write 
    command message to MTCH6303 registers.

  Description:
    This function schedules a non-blocking command message write request to write 
    command message to MTCH6303. The function returns with a valid buffer handle 
    in the bufferHandle argument if the register command message write request 
    was scheduled successfully. The function adds the request to the hardware 
    instance queue and returns immediately. While the request is in the queue, 
    the application message buffer is owned by the driver and should not be 
    modified. The function returns DRV_MTCH6303_TOUCH_BUFFER_HANDLE_INVALID in 
    the bufferHandle argument:
    - if a buffer could not be allocated to the request
    - if the input buffer pointer is NULL
    - if the buffer size is 0 
    - if the message write queue size is full or queue depth is insufficient.
    - if the driver handle is invalid 

    If the requesting client registered an event callback with the driver, the
    driver will issue a DRV_MTCH6303_TOUCH_BUFFER_EVENT_COMPLETE event if the 
    buffer was processed successfully or DRV_MTCH6303_TOUCH_BUFFER_EVENT_ERROR 
    event if the buffer was not processed successfully. A event handler is called 
    on buffer event complete where the application command message is written to
    MTCH6303. 

  Precondition:
    The DRV_MTCH6303_Initialize routine must have been called and the 
    DRV_MTCH6303_Status must have returned SYS_STATUS_READY.

    DRV_MTCH6303_Open must have been called to obtain a valid opened device 
    handle.

  Parameters:
    bufferHandle - Pointer to an argument that will contain the return buffer handle.

    messageCmd - command message to write to MTCH6303.

    messageSize - command message size. It includes message header and payload size.

  Returns:
    None.

  Example:
    <code>

    MY_APP_OBJ myAppObj;    
    DRV_MTCH6303_TOUCH_MESSAGE messageCommand;
    DRV_MTCH6303_TOUCH_BUFFER_HANDLE bufferHandle;

    // Client registers an event handler with driver

    DRV_MTCH6303_TOUCH_BufferEventHandlerSet( APP_MTCH6303BufferEventHandler, 
                                              (uintptr_t)&myAppObj);

    DRV_MTCH6303_TOUCH_AddMessageCommandWrite( &bufferHandle, 
                                               &messageCommand,
                                               MY_MESSAGE_SIZE );

    if(DRV_MTCH6303_TOUCH_BUFFER_HANDLE_INVALID == bufferHandle)
    {
        // Error handling here
    }

    // Event is received when
    // the buffer is processed.

    void APP_MTCH6303BufferEventHandler( DRV_MTCH6303_TOUCH_BUFFER_EVENT event, 
                                         DRV_MTCH6303_TOUCH_BUFFER_HANDLE bufferHandle,
                                         uintptr_t contextHandle )
    {
        // contextHandle points to myAppObj.

        switch(event)
        {
            case DRV_MTCH6303_TOUCH_BUFFER_EVENT_COMPLETE:

                // This means the data was transferred. 
                break;
            
            case DRV_MTCH6303_TOUCH_BUFFER_EVENT_ERROR:

                // Error handling here.

                break;

            default:
                break;
        }
    }

    </code>

  Remarks:
    None.
*/

void DRV_MTCH6303_TOUCH_AddMessageCommandWrite
( 
    DRV_MTCH6303_TOUCH_BUFFER_HANDLE   * bufferHandle,
    DRV_MTCH6303_TOUCH_MESSAGE         * messageCmd,
    size_t                               messageSize
);

// *****************************************************************************
/* Function:
    uint16_t DRV_MTCH6303_TouchInputMap( uint16_t touchValue, uint16_t dispResolution )

  Summary:
    Maps the raw touch input to display resolution.

  Description:
    This function maps the raw touch input to display resolution. Raw touch input
    touchValue is obtained from the individual x or y value of 
    DRV_MTCH6303_TOUCH_DATA. Raw touch value varies from 0 to 0x7FFF. The 
    displayResolution is either horizontal or vertical resolution of the display
    in pixels. The function returns the raw touch input mapped to display 
    resolution in form of number of pixels.

  Precondition:
    None.

  Parameters:
    touchValue - raw touch input either in x or y direction (0 - 0x7FFF).

    dispResolution  - display resolution specifying either width or height of 
                      the display in pixels.

  Return:
    This function returns the raw touch input mapped to display resolution in 
    form of number of pixels.
 
  Example:
    <code>

      // Display with resolution 800 x 480
      #define DISP_HOR_RESOUTION  800
      #define DISP_VER_RESOLUTION 480

      DRV_MTCH6303_TOUCH_DATA touchData;
      uint16_t rawTouchX;
      uint16_t rawTouchY;
      uint16_t touchX;
      uint16_t touchY; 
      
      // map 0th touch packet to display resolution
      rawTouchX = touchData.touch[0].x;
      rawTouchY = touchData.touch[0].y;

      // map raw touch input in x direction to display horizontal resolution
      touchX = DRV_MTCH6303_TouchInputMap( rawTouchX, DISP_HOR_RESOLUTION );

      // map raw touch input in y direction to display vertical resolution
      touchY = DRV_MTCH6303_TouchInputMap( rawTouchY, DISP_VER_RESOLUTION );
 
      // use touchX and touchY as input to graphics objects.

    </code>

  Remarks:
    None.    

*/

inline uint16_t DRV_MTCH6303_TouchInputMap( uint16_t touchValue, uint16_t dispResolution );

#endif

/*******************************************************************************
 End of File
*/