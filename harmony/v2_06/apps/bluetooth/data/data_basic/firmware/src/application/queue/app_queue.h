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


#ifndef __APP_QUEUE_H_INCLUDED__
#define __APP_QUEUE_H_INCLUDED__

//// DOM-IGNORE-BEGIN
#ifdef __cplusplus
extern "C" {
#endif
// DOM-IGNORE-END

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

#ifdef __cplusplus
}
#endif

#endif
/* __APP_QUEUE_H_INCLUDED__ */
/*******************************************************************************
 End of File
*/
