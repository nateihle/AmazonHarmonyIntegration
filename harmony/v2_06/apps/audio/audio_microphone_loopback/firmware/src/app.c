/*******************************************************************************
  Audio Microphone Loopback Demo

  Company:
    Microchip Technology Inc.

  File Name:
    app.c

  Summary:
    Contains the functional implementation of this demo application.

  Description:
    This file contains the functional implementation of this demo application.
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
// DOM-IGNORE-END

//// DOM-IGNORE-BEGIN
#ifdef __cplusplus
extern "C" {
#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Variable Definitions
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
// Section: Global Variable Definitions
// *****************************************************************************
// *****************************************************************************

uint16_t micbuf1[400];
uint16_t micbuf2[400];
APP_DATA appData;
uint32_t someVar;

/******************************************************
 * Application Initialize. It is
 * called from the SYS_Initialized() function.
 ******************************************************/
void APP_Initialize (void)
{
    memset(micbuf1,0,sizeof(micbuf1));
    memset(micbuf2,0,sizeof(micbuf2));
    appData.state = APP_STATE_CODEC_OPEN;
    appData.codecClient.context = (uintptr_t)&someVar;
    appData.codecClient.bufferEventHandler = (DRV_CODEC_BUFFER_EVENT_HANDLER) APP_CodecBufferEventHandler;
    appData.codecClient.txbufferObject = (uint8_t *) micbuf1;
    appData.codecClient.rxbufferObject = (uint8_t *) micbuf2;
    appData.codecClient.bufferSize = sizeof(micbuf1);
    appData.pingPongBuffer = false;

}

/**********************************************************
 * Application tasks routine. This function implements the
 * application state machine.
 ***********************************************************/

void APP_Tasks (void )
{
    switch(appData.state)
    {
        case APP_STATE_CODEC_OPEN:
        {
            /* A client opens the driver object to get an Handle */
            appData.codecClient.handle = DRV_CODEC_Open(DRV_CODEC_INDEX_0,
                        DRV_IO_INTENT_READWRITE | DRV_IO_INTENT_EXCLUSIVE);
            if(appData.codecClient.handle != DRV_HANDLE_INVALID)
            {
                appData.state = APP_STATE_CODEC_SET_BUFFER_HANDLER;
            }
            else
            {
                /* Got an Invalid Handle.  Wait for CODEC to Initialize */
                ;
            }
        }
        break;

        /* Set a handler for the audio buffer completion event */
        case APP_STATE_CODEC_SET_BUFFER_HANDLER:
        {

            DRV_CODEC_BufferEventHandlerSet(appData.codecClient.handle,
                    appData.codecClient.bufferEventHandler,
                    appData.codecClient.context);

            appData.state = APP_STATE_CODEC_ADD_FIRST_BUFFER_READ;
        }
        break;

        case APP_STATE_CODEC_ADD_FIRST_BUFFER_READ:
        {
            /* Initiate the first Read Request by writing 'Zeroes' */
            DRV_CODEC_BufferAddWriteRead(appData.codecClient.handle,
                    &appData.codecClient.writeReadBufHandle,
                    appData.codecClient.txbufferObject,
                    appData.codecClient.rxbufferObject,
                    appData.codecClient.bufferSize);
            if(appData.codecClient.writeReadBufHandle != DRV_CODEC_BUFFER_HANDLE_INVALID)
            {
                appData.state = APP_STATE_CODEC_WAIT_FOR_BUFFER_COMPLETE;
            }
            else
            {
                SYS_DEBUG(0, "Find out whats wrong \r\n");
            }
        }
        break;
       
        /* Audio data request under process */
        case APP_STATE_CODEC_WAIT_FOR_BUFFER_COMPLETE:
        {
            /*Do nothing*/
	    asm("nop");
        }
        break;

        default:
        {
        }
        break;
    }

}
/**********************************************************
 * Application CODEC buffer Event handler.
 * This function is called back by the CODEC driver when
 * a CODEC data buffer RX or TX completes.
 ***********************************************************/
void APP_CodecBufferEventHandler(DRV_CODEC_BUFFER_EVENT event,
        DRV_CODEC_BUFFER_HANDLE handle, uintptr_t context )
{
    switch(event)
    {
        case DRV_CODEC_BUFFER_EVENT_COMPLETE:
        {
            if(appData.pingPongBuffer == false)
            {
                appData.codecClient.rxbufferObject = (uint8_t *) micbuf1;
                appData.codecClient.txbufferObject = (uint8_t *) micbuf2;
                appData.pingPongBuffer = true;
            }
            else
            {
                appData.codecClient.rxbufferObject = (uint8_t *) micbuf2;
                appData.codecClient.txbufferObject = (uint8_t *) micbuf1;
                appData.pingPongBuffer = false;                
            }
            DRV_CODEC_BufferAddWriteRead(appData.codecClient.handle,
                    &appData.codecClient.writeReadBufHandle,
                    appData.codecClient.txbufferObject,
                    appData.codecClient.rxbufferObject,
                    appData.codecClient.bufferSize);
            appData.state = APP_STATE_CODEC_WAIT_FOR_BUFFER_COMPLETE;
        }
        break;
        case DRV_CODEC_BUFFER_EVENT_ERROR:
        {
        } break;

        case DRV_CODEC_BUFFER_EVENT_ABORT:
        {
        } break;

    }
}

#ifdef __cplusplus
}
#endif

/*******************************************************************************
 End of File
 */

