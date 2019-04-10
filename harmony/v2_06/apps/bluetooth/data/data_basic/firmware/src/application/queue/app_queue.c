/********************************************************************
 Software License Agreement:

 The software supplied herewith by Microchip Technology Inc.
 (the "Company") for its PIC Microcontroller is intended and
 supplied to you, the Company's customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
********************************************************************/

#include <stdint.h>
#include "app_queue.h"
#include "app.h"

int32_t nQueueInsert(QUEUE_POINTERS *qp)
{
    int32_t q_ret;
    if(qp == (QUEUE_POINTERS *)0)
    {// Uninitialized queue
        return (int32_t)-1;
    }
    /* Queue Full condition */
    if ( qp->q_content == qp->q_size)
    {
        assert( (qp->front==0 && qp->rear==qp->q_size-1)
               || (qp->front==qp->rear+1 )              ); // Condition from V2.0
        q_ret = 0;
    }
    else
    {
        if(qp->rear == -1)
        {//Empty queue, initialize pointers for first entry
            qp->rear = 0;
            qp->front = 0;
        }
        else
        { // Queue not empty, advance rear pointer
            if(qp->rear == qp->q_size-1)
            {
                qp->rear = 0;
            }
            else
            {
                qp->rear++;
            }
        }
        qp->insertBackFunc(qp->rear, qp->msg);
        qp->q_content++;
        q_ret = 1;
    }
    return (int32_t)q_ret;
}

int32_t nQueueRemove(QUEUE_POINTERS *qp)
{
    int32_t q_ret;

    if(qp == (QUEUE_POINTERS *)0)
    {
        return (int32_t)-1;
    }
    /* Queue empty condition */
    if(0 == qp->q_content)
    {
        assert( qp->front==-1 ); // Condition from V2.0
        q_ret = 0;
    }
    else
    { // Access next message, update pointers to remove it from the queue
        qp->msg = qp->deleteFrontFunc(qp->front);

        // If front and rear point to same entry...
        if(qp->front == qp->rear)
        {// Queue now empty
            qp->rear  = -1;
            qp->front = -1;
            qp->q_content = 0;
        }
        else
        {// Queue not empty, advance front pointer
            if(qp->front == qp->q_size-1)
            {
                qp->front = 0;
            }
            else
            {
                qp->front++;
            }
            qp->q_content--;
        }
        q_ret = 1;
    }
    return (int32_t)q_ret;
}

int32_t nQueueCheck(QUEUE_POINTERS *qp)
{
    int32_t q_ret;

    if(qp == (QUEUE_POINTERS *)0)
    {
        return (int32_t)-1;
    }
    /* Queue empty condition */
    if(0 == qp->q_content)
    {
        q_ret = 0;
    }
    else
    { // Access next message without removing it
        qp->msg = qp->deleteFrontFunc(qp->front);
        q_ret = 1;
    }
    return (int32_t)q_ret;
}


int32_t nQueueContent(QUEUE_POINTERS *qp)
{
    if(qp == (QUEUE_POINTERS *)0)
    {
        return -1;
    }
    return (int32_t)(qp->q_content);
}