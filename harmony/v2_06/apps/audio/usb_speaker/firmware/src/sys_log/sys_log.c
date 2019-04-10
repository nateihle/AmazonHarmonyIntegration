/*******************************************************************************
  Application System Log Messaging Services

  Company:
    Microchip Technology Inc.

  File Name:
    sys_log.c

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

#if defined( ENABLE_SYS_LOG )

#include <xc.h>
#include <cp0defs.h>

#include "sys_log.h"
#include "sys_log_local.h"
#include "sys_log_define.h"
#include "sys_log_messaging.h"

#include "system/clk/sys_clk.h"
#include "system/debug/sys_debug.h"

#include <stdarg.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

// Max string length
#define MAX_STRING_LENGTH 128

// Buffer for messaging
static char     _Buffer[MAX_STRING_LENGTH];
static uint16_t _NumMessagesSkipped = 0;
#define         MAX_NUMMESSAGESSKIPPED UINT16_MAX

// Buffer for composing log messages coming from the pre-compiled libraries
// through SYS_LOG_LibEntryWrite().  Need second buffer, since function that
// uses this buffer calls _WriteToLogBuffer, which uses _Buffer defined above.
static char   _LibBuffer[MAX_STRING_LENGTH];

// Local functions
static void   _WriteToLogBuffer(const char* msg);


void SYS_LOG_Init(void)
{
    int16_t iResult;
    iResult = SYS_LOG_MESSAGE_Initialize();  // Initialize message queue
    assert(iResult >= 0);

    SYS_LOG_PAL_Init();  // Initialize (setup) the UART
    SYS_LOG_PAL_Start(); // Start (enable) the UART
    _CP0_SET_COUNT(0);   // Set CPU count to zero

}//SYS_LOG_Init


void SYS_LOG_Deinit(void)
{
    SYS_LOG_MESSAGE_Deinitialize();  // Deinit queue
    SYS_LOG_PAL_Stop(); // Disable (stop) UART.

}//SYS_LOG_Init


void SYS_LOG_Start(void)
{
    SYS_LOG_PAL_Start();
}//SYS_LOG_Start


void SYS_LOG_Stop(void)
{
    SYS_LOG_PAL_Stop();
}//SYS_LOG_Start


void SYS_LOG_Task()
{
    // If there are log messages pending and we are not transmitting...
    if ( SYS_LOG_MESSAGE_QueueStatusGet() != SYS_LOG_QUEUE_EMPTY && !SYS_LOG_PAL_Transmitting() )
    {
        SYS_LOG_PAL_StartTransmitting();
    }

}//SYS_LOG_Task


void SYS_LOG_EntryWrite(const char* tag, const char* message)
{
    int tagLength;

    memset(_Buffer,0,sizeof(_Buffer));
    if (tag != NULL)
    {
        tagLength = strlen(tag);
        tagLength = tagLength <= 10 ? tagLength : 10;
        strncpy(_Buffer,tag,tagLength);
        strcat(_Buffer,":");
    }

    strcat(_Buffer,message);

    _WriteToLogBuffer(_Buffer);

}//SYS_LOG_EntryWrite


void SYS_LOG_FormatEntryWrite(const char* tag, const char* format, ...)
{
    int iString;
    va_list ap;

    va_start(ap, format);

    memset(_Buffer,0,sizeof(_Buffer));

    if (tag != NULL)
    {
        iString = strlen(tag);
        iString = iString <= 10 ? iString : 10;
        strncpy(_Buffer,tag,iString);
        strcat(_Buffer,":");
    }

    vsprintf(_Buffer+strlen(_Buffer),format,ap);

    _WriteToLogBuffer(_Buffer);

    va_end(ap);

}//SYS_LOG_FormatEntryWrite


void SYS_LOG_LongEntryWrite(const char* tag, const char* msg)
{
    char c;
    int tagLength;
    int _LibBufferIndex = 0;

    memset(_LibBuffer,0,sizeof(_LibBuffer));

    if (tag != NULL)
    {
        tagLength = strlen(tag);
        strncpy(_LibBuffer,tag,tagLength);
        strcat(_LibBuffer,":");
        _LibBufferIndex = strlen(_LibBuffer);
    }

    while ((c = *msg++) != 0)
    {
        if (_LibBufferIndex == (SYS_LOG_MESSAGE_CHAR_LENGTH - 2))
        {// Reached useful end of buffer
            if ( *(msg+1) !=0 )
            { // More text to output
                _LibBuffer[_LibBufferIndex++] = '\\'; // Add line continuation
            }
            _LibBuffer[_LibBufferIndex] = 0;  // Terminate the string.

            _WriteToLogBuffer(_LibBuffer);
            _LibBufferIndex = 0;
        }

        if (c == '\n')
        {
            _LibBuffer[_LibBufferIndex] = 0;
            _WriteToLogBuffer(_LibBuffer);
            _LibBufferIndex = 0;
        }
        else
        {
            _LibBuffer[_LibBufferIndex++] = c;
        }

    }//end while ((c = *msg++) != 0)

    if ( _LibBufferIndex > 0 )
    {
        _LibBuffer[_LibBufferIndex] = 0;
        _WriteToLogBuffer(_LibBuffer);
    }

}//SYS_LOG_LongEntryWrite


void SYS_LOG_LibEntryWrite(const char* msg)
{
    SYS_LOG_LongEntryWrite(NULL,msg);

}//SYS_LOG_LibEntryWrite


static char _NextMessage[MAX_STRING_LENGTH+2]; // Message + n> prefix
static char *pNextChar = NULL;
static char EOLString[] = { '\r', '\n', 0 };
static bool bEOLOutput = false;

char SYS_LOG_GetNextChar(void)
{
    int iString;
    char nextChar = 0;
    uint8_t numMsgsSkipped;

    if ( pNextChar != NULL ) // If pointer is initialized
    {
        if ( 0 != *pNextChar ) // Not at end of string
        { // Return next character in the message
            nextChar = *pNextChar++;
        }
        else // At end of a string
        { // End of string is from what??
            if ( bEOLOutput ) // Outputting EOL string.
            { // Reached end of EOL string
                bEOLOutput = false;
                pNextChar = NULL; // Fetch next message if available
            }
            else // Reached end of message string
            { // Start EOL output
                pNextChar = EOLString;
                nextChar = *pNextChar++;
                bEOLOutput = true;
            }
        }
    }//if ( pNextChar != NULL )

    // If pNextChar uninitialized or have run out of message text
    if ( NULL == pNextChar )
    {// Get next message
        SYS_LOG_MESSAGE msg;
        if ( SYS_LOG_MESSAGE_Receive(&msg) >= 0 )
        { // Got new log message

            // Clear message string.
            memset(_NextMessage,0,sizeof(_NextMessage));

            // Add time of message in milliseconds.
            sprintf(_NextMessage,"%13.5f ",2*1000.0*msg.msgCPUcount/SYS_CLK_SystemFrequencyGet());
            
            // Add number of messages
            numMsgsSkipped = msg.numMessagesSkipped;
            if ( numMsgsSkipped == 0 )
            {
                nextChar = '-';
            }
            else
            {
                nextChar =  numMsgsSkipped  < 10 ? ('0' + (char)numMsgsSkipped) :
                           (numMsgsSkipped <= 15 ? ('A' + (char)(numMsgsSkipped-10) ) : '*' );
            }
            iString = strlen(_NextMessage);
            _NextMessage[iString]   = nextChar;
            _NextMessage[iString+1] = '>';

            // Add queued message to message string.
            strncat(_NextMessage,msg.msgString,msg.msgStringLength);

            pNextChar = _NextMessage;
            nextChar  = *pNextChar++;

        }
        else
        { // No messages available
            pNextChar = NULL;
            nextChar = 0;
        }
    }//end if ( NULL == pNextChar )

    return nextChar;

}//SYS_LOG_GetNextChar


static uint32_t _OldCPUCount = 0;

static uint32_t _GetDeltaCount(void)
{
    uint32_t newCPUCount,deltaCPUCount;
    newCPUCount = _CP0_GET_COUNT(); // Current CPU Clock/2 count
    if ( _OldCPUCount < newCPUCount )
    { // Counter hasn't wrapped
        deltaCPUCount = newCPUCount - _OldCPUCount;
    }
    else if ( newCPUCount < _OldCPUCount )
    { // Counter has wrapped
        deltaCPUCount = _OldCPUCount - newCPUCount;
        deltaCPUCount = UINT32_MAX - ( newCPUCount - 1);
    }
    else
    {
        deltaCPUCount = 0;
    }
    _OldCPUCount = newCPUCount;
    return deltaCPUCount;
}


static void _WriteToLogBuffer(const char* logMessage)
{
    int16_t iSendResult;
    SYS_LOG_MESSAGE msg;

    memset(msg.msgString,0,sizeof(msg.msgString));
    msg.msgType             = MSG_STRING;
    msg.msgStringLength     = SYS_LOG_MESSAGE_MessageLength(logMessage);
    msg.numMessagesSkipped  = _NumMessagesSkipped;
    strncpy(msg.msgString,logMessage,msg.msgStringLength);
    msg.msgCPUcount         = _GetDeltaCount(); // Record delta CPU Clock/2 count
    iSendResult = SYS_LOG_MESSAGE_Send(&msg);
    if ( 0 < iSendResult )
    { // Success
        _NumMessagesSkipped = 0;
    }
    else
    {
        if ( _NumMessagesSkipped < MAX_NUMMESSAGESSKIPPED )
        {
            _NumMessagesSkipped++;
        }
        else
        {
            _NumMessagesSkipped = MAX_NUMMESSAGESSKIPPED;
        }
    }
}//_WriteToLogBuffer

#else

void SYS_LOG_LibEntryWrite(const char* msg)
{
    // Do Nothing: System Logging disabled.
}

#endif//defined( ENABLE_SYS_LOG )
