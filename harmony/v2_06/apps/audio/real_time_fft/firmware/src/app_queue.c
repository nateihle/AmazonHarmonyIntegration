/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app_queue.c

  Summary:
    Contains the functional implementation of the application queue.

  Description:
    This file contains the source code for the application queue, which is used
    by multiple applications in this demo. It provides helper functions for easy
    access to the queue.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app_queue.h"

#define MAX_MSG_QUEUES                          3
#define ENTER_CRITICAL(interruptState)          interruptState = SYS_INT_StatusGetAndDisable()
#define EXIT_CRITICAL(interruptState)           SYS_INT_StatusRestore(interruptState)

static MSG_QUEUE msgQueue[MAX_MSG_QUEUES] = {{0},{0},{0}};

static MSG_QUEUE* APP_QUEUE_GetInstance(void)
{
	uint8_t i;
	MSG_QUEUE* pMsgQueue = NULL;

	for (i = 0; i < MAX_MSG_QUEUES; i++)
	{
		if (msgQueue[i].inUse == false)
		{
			pMsgQueue = &msgQueue[i];
			break;
		}
	}
	return pMsgQueue;
}

MSG_QUEUE_HANDLE APP_QUEUE_Open(
    uint8_t* pBuffer, 
    uint32_t elementSize, 
    uint32_t nElements
)
{        	
	MSG_QUEUE* pMsgQueue = APP_QUEUE_GetInstance();
	
	if (pMsgQueue)
	{		
		pMsgQueue->pBuffer = pBuffer;
		pMsgQueue->pInptr = pMsgQueue->pOutptr = pMsgQueue->pBuffer;
		pMsgQueue->elementSize = elementSize;
		pMsgQueue->nElements = nElements;
		pMsgQueue->totalQueueSizeInBytes = pMsgQueue->elementSize * pMsgQueue->nElements;
		pMsgQueue->inUse = true;
	}

	return (MSG_QUEUE_HANDLE)pMsgQueue;    
}

void APP_QUEUE_Flush(MSG_QUEUE_HANDLE qHandle)
{    
	MSG_QUEUE* pMsgQueue = (MSG_QUEUE*)qHandle;
    uint32_t interruptState; 
    
    ENTER_CRITICAL(interruptState);
    
    pMsgQueue->pInptr = pMsgQueue->pOutptr = pMsgQueue->pBuffer;        
    
    EXIT_CRITICAL(interruptState);
}

bool APP_QUEUE_IsQueued(MSG_QUEUE_HANDLE qHandle)
{    
    bool isQueued = false;    
	MSG_QUEUE* pMsgQueue = (MSG_QUEUE*)qHandle;
    uint32_t interruptState; 
    
    ENTER_CRITICAL(interruptState);
    
    if (pMsgQueue->pInptr != pMsgQueue->pOutptr)
    {
        isQueued = true;
    }        
    
    EXIT_CRITICAL(interruptState);
    return isQueued;
}

bool APP_QUEUE_Pull(MSG_QUEUE_HANDLE qHandle, void* pQElement)
{
    bool isSuccess = false;
	MSG_QUEUE* pMsgQueue = (MSG_QUEUE*)qHandle;
    uint32_t interruptState; 
    
    ENTER_CRITICAL(interruptState);
                
    if (pMsgQueue->pInptr != pMsgQueue->pOutptr)
    {        
		memcpy(pQElement, pMsgQueue->pOutptr, pMsgQueue->elementSize);
        pMsgQueue->pOutptr += pMsgQueue->elementSize;
		if (pMsgQueue->pOutptr >= pMsgQueue->pBuffer + pMsgQueue->totalQueueSizeInBytes)
        {
            pMsgQueue->pOutptr = pMsgQueue->pBuffer;
        }
        isSuccess = true;
    }    
    
    EXIT_CRITICAL(interruptState);
    return isSuccess;
}

bool APP_QUEUE_Push(MSG_QUEUE_HANDLE qHandle, void* pQElement)
{
    bool isSuccess = false;
	MSG_QUEUE* pMsgQueue = (MSG_QUEUE*)qHandle;
    uint8_t* pTemp;
    uint32_t interruptState; 
    
    ENTER_CRITICAL(interruptState);

	pTemp = pMsgQueue->pInptr + pMsgQueue->elementSize;
    
	if (pTemp >= pMsgQueue->pBuffer + pMsgQueue->totalQueueSizeInBytes)
    {
        pTemp = pMsgQueue->pBuffer;
    }
	if (pTemp != pMsgQueue->pOutptr)
    {
		memcpy(pMsgQueue->pInptr, pQElement, pMsgQueue->elementSize);
		pMsgQueue->pInptr = pTemp;        
        isSuccess = true;
    }        
    
    EXIT_CRITICAL(interruptState);
    
    return isSuccess;
}

bool APP_QUEUE_IsFull(MSG_QUEUE_HANDLE qHandle)
{
    bool isQueueFull = false;
	MSG_QUEUE* pMsgQueue = (MSG_QUEUE*)qHandle;
    uint8_t* pTemp;
    uint32_t interruptState; 
    
    ENTER_CRITICAL(interruptState);
        
    pTemp = pMsgQueue->pInptr + pMsgQueue->elementSize;
    
	if (pTemp >= pMsgQueue->pBuffer + pMsgQueue->totalQueueSizeInBytes)
    {
		pTemp = pMsgQueue->pBuffer;
    }
	if (pTemp == pMsgQueue->pOutptr)
    {
        isQueueFull = true;
    }       
    
    EXIT_CRITICAL(interruptState);
    
    return isQueueFull;
}