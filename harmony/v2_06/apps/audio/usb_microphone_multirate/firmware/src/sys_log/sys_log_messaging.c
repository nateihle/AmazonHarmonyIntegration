/*******************************************************************************
  Application System Log Messaging Services

  Company:
    Microchip Technology Inc.

  File Name:
    btlog_messaging.c

  Summary:
    Provides messaging services for the application event logging.

  Description:
    Provides messaging services for the application event logging.
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


// *****************************************************************************
// *****************************************************************************
// Section: Include Files
// *****************************************************************************
// *****************************************************************************

#if defined( ENABLE_SYS_LOG )

#include <cp0defs.h>
#include "system/int/sys_int.h"
#include "sys_log_messaging.h"
#include "system/debug/sys_debug.h"


// *****************************************************************************
// *****************************************************************************
// Section: File Scope Variables
// *****************************************************************************
// *****************************************************************************
static SYS_LOG_MESSAGE _NullMessage;
static SYS_LOG_MESSAGE _MessageList[SYS_LOG_MESSAGE_QUEUE_SIZE];
static QUEUE_POINTERS _MessageQueue;

// *****************************************************************************
// *****************************************************************************
// Section: File Scope Functions
// *****************************************************************************
// *****************************************************************************
// Inserts a message at location defined by index
static void   _QueueInsertFunc(int32_t index, void *msg)
{
    assert( 0 <= index  );
    assert(      index < SYS_LOG_MESSAGE_QUEUE_SIZE );
    _MessageList[index] = *((SYS_LOG_MESSAGE *)msg);
}

// Returns pointer to message list entry defined by index
static void * _QueueDeleteFunc(int32_t index)
{
    if ( 0 <= index && index < SYS_LOG_MESSAGE_QUEUE_SIZE )
    { // If legal index into message queue, return the message
        return (void *) &_MessageList[index];
    }
    else
    { // Illegal address, return null message
      // Message Type = 0, won't be part of any case statement
        return (void *) &_NullMessage;
    }
}


// *****************************************************************************
// *****************************************************************************
// Section: Messaging Functions
// *****************************************************************************
// *****************************************************************************

int16_t SYS_LOG_MESSAGE_Initialize(void)
{
    if ( 0 == _MessageQueue.q_size )
    {
        /* System Log Queue Initialization */
        _MessageQueue.front=-1;
        _MessageQueue.rear=-1;
        _MessageQueue.q_size = SYS_LOG_MESSAGE_QUEUE_SIZE;
        _MessageQueue.q_content = 0;
        _MessageQueue.insertBackFunc  = _QueueInsertFunc;
        _MessageQueue.deleteFrontFunc = _QueueDeleteFunc;
        return SYS_LOG_MESSAGE_QUEUE_SIZE;
    }
    else
    { // Queue already intialized
        return -1;
    }

}//SYS_LOG_MESSAGE_Initialize


void SYS_LOG_MESSAGE_Deinitialize(void)
{
    _MessageQueue.q_size = 0;
}


uint8_t SYS_LOG_MESSAGE_MessageLength( const char *msg )
{
    uint16_t msgLength;

    msgLength = strlen(msg);
    msgLength = msgLength <= (SYS_LOG_MESSAGE_CHAR_LENGTH-1) ? msgLength : (SYS_LOG_MESSAGE_CHAR_LENGTH-1);
    return (uint8_t)msgLength;
}


int16_t SYS_LOG_MESSAGE_Send( SYS_LOG_MESSAGE * pMessage )
{
    int32_t iQueueResult,iQueueContent;
    bool    bInterruptsEnabled;

    assert( 0 < _MessageQueue.q_size );  // Queue intialized

    bInterruptsEnabled = SYS_INT_Disable(); // Protect queue pointers from ISRs.
    if( _MessageQueue.q_content < _MessageQueue.q_size )
    { // Queue has room for message
        _MessageQueue.msg = (void *) pMessage;
        iQueueResult  = nQueueInsert(&_MessageQueue);
        assert( 1 == iQueueResult );
        iQueueContent = _MessageQueue.q_content;
    }
    else
    { // Queue already full, no room for new message
        iQueueContent = SYS_LOG_QUEUE_ALREADY_FULL;
    }
    if ( bInterruptsEnabled ) SYS_INT_Enable(); // Re-enable interrupts
    return iQueueContent;

}//SYS_LOG_MESSAGE_Send


int16_t SYS_LOG_MESSAGE_Receive( SYS_LOG_MESSAGE * pMessage )
{
    int32_t iQueueResult,iQueueContent;
    bool    bInterruptsEnabled;

    assert( 0 < _MessageQueue.q_size );  // Queue initialized

    bInterruptsEnabled = SYS_INT_Disable(); // Protect queue pointers from ISRs.
    if ( 0 < _MessageQueue.q_content )
    { // Queue not empty
        iQueueResult  = nQueueRemove(&_MessageQueue);
        assert( 1 == iQueueResult );
        iQueueContent = _MessageQueue.q_content;
        *pMessage = *((SYS_LOG_MESSAGE *)_MessageQueue.msg);
    }
    else
    { // Queue already empty, there is not message to return
        *pMessage = _NullMessage;
        iQueueContent = SYS_LOG_QUEUE_ALREADY_EMPTY;
    }
    if ( bInterruptsEnabled ) SYS_INT_Enable(); // Re-enable interrupts
    return iQueueContent;

}//SYS_LOG_MESSAGE_Receive


int16_t SYS_LOG_MESSAGE_QueueSizeGet(void)
{
    assert( 0 < _MessageQueue.q_size );  // Queue intialized
    return _MessageQueue.q_content;

}//SYS_LOG_MESSAGE_QueueSizeGet


SYS_LOG_QUEUE_STATUS SYS_LOG_MESSAGE_QueueStatusGet(void)
{
    int32_t iQueueContent;
    bool    bInterruptsEnabled;

    assert( 0 < _MessageQueue.q_size );  // Queue intialized

    bInterruptsEnabled = SYS_INT_Disable(); // Protect queue from ISRs.
    if      ( _MessageQueue.q_content == 0 )
    {
        iQueueContent = SYS_LOG_QUEUE_EMPTY;
    }
    else if ( _MessageQueue.q_content == _MessageQueue.q_size )
    {
        iQueueContent = SYS_LOG_QUEUE_FULL;
    }
    else
    {
        iQueueContent = SYS_LOG_QUEUE_NOT_EMPTY;
    }
    if ( bInterruptsEnabled ) SYS_INT_Enable(); // Re-enable interrupts
    return iQueueContent;

}//SYS_LOG_MESSAGE_QueueStatusGet

#endif//defined( ENABLE_SYS_LOG )

/*******************************************************************************
End of File
*/
