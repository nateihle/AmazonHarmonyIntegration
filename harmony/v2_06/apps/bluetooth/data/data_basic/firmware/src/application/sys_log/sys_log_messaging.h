/*******************************************************************************
  Application System Log Messaging Services

  Company:
    Microchip Technology Inc.

  File Name:
    sys_log_messaging.h

  Summary:
    Provides messaging services for the application event logging.

  Description:
    Provides messaging services for the application event logging.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
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
*******************************************************************************/
//DOM-IGNORE-END

#ifndef _SYS_LOG_MESSAGING_H
#define _SYS_LOG_MESSAGING_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "user_config.h"
#include "error.h"


// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************
/* Size of System Log tasking message queue */
#define SYS_LOG_MESSAGE_QUEUE_SIZE  200 //700

#define SYS_LOG_MESSAGE_CHAR_LENGTH (89)

typedef struct
{
    int32_t    rear;
    int32_t    front;
    void       *msg;
    int32_t    q_size;
    int32_t    q_content;
    void (*insertBackFunc)(int32_t, void *);
    void* (*deleteFrontFunc)(int32_t);
} QUEUE_POINTERS;

int32_t nQueueInsert(QUEUE_POINTERS *);
int32_t nQueueRemove(QUEUE_POINTERS *);
int32_t nQueueCheck(QUEUE_POINTERS *);
int32_t nQueueContent(QUEUE_POINTERS *);

// *****************************************************************************
/*  System Log Queue Status Enumeration

  Summary:
   System Log queue status enumeration

  Description:
    Provides enumeration of System Log queue status.
    Negative values indicate an error condition.

  Remarks:
    None.
*/
typedef enum
{
    /* Error retrieving message off of queue, queue is already empty */
    SYS_LOG_QUEUE_ALREADY_EMPTY /*DOM-IGNORE-BEGIN*/ = -2 /*DOM-IGNORE-END*/,

    /* Error queuing message onto the queue, queue is already full */
    SYS_LOG_QUEUE_ALREADY_FULL  /*DOM-IGNORE-BEGIN*/ = -1 /*DOM-IGNORE-END*/,

    /* Queue has no messages */
    SYS_LOG_QUEUE_EMPTY         /*DOM-IGNORE-BEGIN*/ =  0 /*DOM-IGNORE-END*/,

    /* Queue has messages but is not full */
    SYS_LOG_QUEUE_NOT_EMPTY,

    /* Queue is full, no additional messages can be placed on the queue */
    SYS_LOG_QUEUE_FULL

} SYS_LOG_QUEUE_STATUS;


// *****************************************************************************
/* System Log Message Tasking Message Types Enumeration

  Summary:
   Provides an enumeration of all possible System Log tasking messages.

  Description:
   Provides an enumeration of all possible System Log tasking messages.

  Remarks:
 These enumerations are used in the .msgType element of a message structure:
  <code>
    int16_t iSendResult;
    SYS_LOG_MESSAGE msg;
    msg.msgType = MSG_STRING;
    msg.msgStringLength = SYS_LOG_MESSAGE_MessageLength(myMessage);
    strncpy(msg.msgString,myMessage,msg.msgStringLength);
    iSendResult = SYS_LOG_MESSAGE_Send(&msg);
    assert( 0 < iSendResult );
  </code>
*/
typedef enum {
    MSG_STRING /*DOM-IGNORE-BEGIN*/  =  1 /*DOM-IGNORE-END*/,
    MSG_OTHER
} SYS_LOG_MESSAGE_TYPE;

// Message structure between System Log code and UART output task
typedef struct
{
    uint8_t            msgType;
    uint8_t            numMessagesSkipped;
    uint32_t           msgCPUcount;
    char               msgString[SYS_LOG_MESSAGE_CHAR_LENGTH];
    uint8_t            msgStringLength;
} SYS_LOG_MESSAGE;


// *****************************************************************************
// *****************************************************************************
// Section: System Log messaging primitives
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    int16_t SYS_LOG_MESSAGE_Initialize(void)

  Summary:
    Intializes System Log messaging queue.

  Description:
    Initializes System Log messaging queue, returning true if successful, false otherwise

  Precondition:
    System Log messaging queue has not been intialized with prior call to this routine.

  Parameters:
    None.

  Returns:
    Queue size if successful, -1 otherwise

  Example:
  <code>
    int16_t iInitResult;
    iInitResult = SYS_LOG_MESSAGE_Initialize();
    assert( 0 < iInitResult );
  </code>

  Remarks:
    See also SYS_LOG_MESSAGE_Deinitialize.
    Message queue size defined by SYS_LOG_MESSAGE_QUEUE_SIZE.
*/
int16_t SYS_LOG_MESSAGE_Initialize(void);


// *****************************************************************************
/* Function:
    void SYS_LOG_MESSAGE_Deinitialize(void)

  Summary:
    Deinitializes System Log message queue.

  Description:
    Deinitializes System Log message queue.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Example:
  <code>
    int16_t iInitResult;
    // Flush System Log message queue of all messages
    SYS_LOG_MESSAGE_Deinitialize();
    iInitResult = SYS_LOG_MESSAGE_Initialize();
    assert( 0 < iInitResult );
  </code>

  Remarks:
    None.
*/
void SYS_LOG_MESSAGE_Deinitialize(void);

uint8_t SYS_LOG_MESSAGE_MessageLength( const char *msg );

// *****************************************************************************
/* Function:
    int16_t SYS_LOG_MESSAGE_Send( SYS_LOG_MESSAGE * pMessage )

  Summary:
    Sends System Log tasking message for later execution by displayTask.

  Description:
    Sends System Log tasking message for later execution by displayTask.

  Precondition:
    SYS_LOG_MESSAGE_Initialize has been called.

  Parameters:
    pMessage - Pointer to System Log Task message to be sent.

  Returns:
    Number of System Log task messages currently in the queue
    OR SYS_LOG_QUEUE_ALREADY_FULL if there is no room in the queue for the new
    message.

  Example:
  <code>
    int16_t iSendResult;
    SYS_LOG_MESSAGE msg;
    msg.msgType = MSG_STRING;
    msg.msgStringLength = SYS_LOG_MESSAGE_MessageLength(myMessage);
    strncpy(msg.msgString,myMessage,msg.msgStringLength);
    iSendResult = SYS_LOG_MESSAGE_Send(&msg);
    assert( 0 < iSendResult );
  </code>

  Remarks:
    None.
*/
int16_t SYS_LOG_MESSAGE_Send( SYS_LOG_MESSAGE * pMessage );


// *****************************************************************************
/* Function:
    int16_t SYS_LOG_MESSAGE_Receive( SYS_LOG_MESSAGE * pMessage )

  Summary:
    Receive System Log Message task message for processing.

  Description:
    Receive System Log Message task message for processing.  Oldest message is
    dequeued and returned in *pMessage

  Precondition:
    SYS_LOG_MESSAGE_Initialize has been called and SYS_LOG_MESSAGE_Send has queued
    a message for proecssing.

  Parameters:
    pMessage - Pointer to System Log Task message to be received.

  Returns:
    Number of audio task messages left in the queue after returning next message
    OR SYS_LOG_QUEUE_ALREADY_EMPTY if there are no messages left on the queue
    for processing.

  Example:
  <code>
    SYS_LOG_MESSAGE msg;
    while( SYS_LOG_MESSAGE_Receive(&msg) >= 0 )
    {
        switch( msg.msgType )
        {
            case MSG_STRING:
              .
              .
              .
        }//end switch( msg.msgType )
    }
  </code>

  Remarks:
    None.
*/
int16_t SYS_LOG_MESSAGE_Receive( SYS_LOG_MESSAGE * pMessage );


// *****************************************************************************
/* Function:
    int16_t SYS_LOG_MESSAGE_QueueSizeGet(void)

  Summary:
    Returns number of queue entries currently in the System Log Task Message queue.

  Description:
    Returns number of queue entries currently in the System Log Task Message queue.

  Precondition:
    SYS_LOG_MESSAGE_Initialize has been called.

  Parameters:
    None.

  Returns:
    Returns the number of System Log Task Messages currently on the queue.

  Example:
  <code>
  </code>

  Remarks:
    None.
*/
int16_t SYS_LOG_MESSAGE_QueueSizeGet(void);


// *****************************************************************************
/* Function:
    SYS_LOG_QUEUE_STATUS SYS_LOG_MESSAGE_QueueStatusGet(void)

  Summary:
    Returns System Log Task Mesage queue status.

  Description:
    Returns System Log Task Mesage queue status.

  Precondition:
    SYS_LOG_MESSAGE_Initialize has been called.

  Parameters:
    None.

  Returns:
    SYS_LOG_QUEUE_EMPTY, SYS_LOG_QUEUE_NOT_EMPTY, or SYS_LOG_QUEUE_FULL


  Example:
  <code>
    if ( SYS_LOG_QUEUE_FULL != SYS_LOG_MESSAGE_QueueStatusGet )
    {   // Send another System Log Task Message
    }
  </code>

  Remarks:
    None.
*/
SYS_LOG_QUEUE_STATUS SYS_LOG_MESSAGE_QueueStatusGet(void);

#endif // #ifndef _SYS_LOG_MESSAGING_H

/*******************************************************************************
 End of File
*/

