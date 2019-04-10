/*******************************************************************************
  SQI Driver Interface Implementation

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sqi.c

  Summary:
    SQI Driver Interface Definition

  Description:
    The SQI driver provides data structues and interfaces to manage the SQI
    controller. This file contains the data structures and interface
    definitions of the SQI driver.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2016 - 2017 released Microchip Technology Inc.  All rights reserved.

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

#include "driver/sqi/src/drv_sqi_local.h"
#include "system/debug/sys_debug.h"

/************************************************
 * This token is incremented for every request
 * added to the queue and is used to generate
 * a different buffer handle for every request.
 ***********************************************/

uint16_t gDrvSQIBufferToken = 0;

/*************************************************
 * OSAL Declarations
 *************************************************/
/* SQI Client Object Mutex */
OSAL_MUTEX_DECLARE(sqiClientObjMutex);

/* SQI Buffer Object Mutex*/
OSAL_MUTEX_DECLARE(sqiBufObjMutex);

_DRV_SQI_DMA_Desc gSQIDmaBDDesc[DRV_SQI_DMA_BUFFER_DESC_NUMBER] __attribute__((coherent));

DRV_SQI_CLIENT_OBJECT gDrvSQIClientObj[DRV_SQI_CLIENTS_NUMBER];

DRV_SQI_OBJECT gDrvSQIObj[DRV_SQI_INSTANCES_NUMBER];
_DRV_SQI_BUFFER_OBJECT gDrvSQIBufferObject[DRV_SQI_BUFFER_OBJECT_NUMBER];
DRV_SQI_TransferElement gDrvSQIXferElement[DRV_SQI_BUFFER_OBJECT_NUMBER][3];
uint8_t __attribute__((coherent)) elementData0[DRV_SQI_BUFFER_OBJECT_NUMBER][7];
uint8_t __attribute__((coherent)) elementData1[DRV_SQI_BUFFER_OBJECT_NUMBER][7];

/* This function checks if the sqi device to be used is enabled. */
static bool _DRV_SQI_ValidateSqiDevice
(
    DRV_SQI_OBJECT *dObj,
    uint8_t sqiDevice
)
{
    if (sqiDevice > 2)
    {
        return false;
    }
    if ((sqiDevice == 0) && (dObj->enabledDevices == DRV_SQI_ENABLE_DEVICE_1))
    {
        return false;
    }
    else if ((sqiDevice == 1) && (dObj->enabledDevices == DRV_SQI_ENABLE_DEVICE_0))
    {
        return false;
    }

    return true;
}

/* This function validates the client handle. */
static DRV_SQI_CLIENT_OBJECT* _DRV_SQI_ValidateClientHandle
(
    DRV_HANDLE handle
)
{
    DRV_SQI_CLIENT_OBJECT *clientObj = NULL;
    DRV_SQI_OBJECT *dObj = NULL;

    /* Validate the handle */
    if (handle == DRV_HANDLE_INVALID)
    {
        return NULL;
    }

    /* See if the client has been opened */
    clientObj = (DRV_SQI_CLIENT_OBJECT *)handle;
    if (!clientObj->inUse)
    {
        return NULL;
    }

    /* Check if the driver is ready for operation */
    dObj = (DRV_SQI_OBJECT *)clientObj->driverObj;
    if (dObj->status != SYS_STATUS_READY)
    {
        return NULL;
    }

    return clientObj;
}

/* This function adds the buffer object to the tail of the queue. */
static void _DRV_SQI_AddToQueue
(
    DRV_SQI_OBJECT *dObj,
    _DRV_SQI_BUFFER_OBJECT *bufferObject
)
{
    if (dObj->rwQueue == NULL)
    {
        dObj->rwQueue = bufferObject;    
    }
    else
    {
        if (dObj->rwQueue->previous != NULL)
        {
            dObj->rwQueue->previous->next = bufferObject;
            bufferObject->previous = dObj->rwQueue->previous;
            dObj->rwQueue->previous = bufferObject;
        }
        else
        {
            dObj->rwQueue->previous = bufferObject;
            dObj->rwQueue->next = bufferObject;
            bufferObject->previous = dObj->rwQueue;
        }
    }
}

/* This function updates the head of the queue. */
static void _DRV_SQI_UpdateQueue
(
    DRV_SQI_OBJECT *dObj
)
{
    _DRV_SQI_BUFFER_OBJECT * bufferObject = dObj->rwQueue;

    if (dObj->rwQueue != NULL)
    {
        bufferObject->inUse = false;
        if (dObj->rwQueue->next != NULL)
        {
            dObj->rwQueue = dObj->rwQueue->next;
            if (dObj->rwQueue == bufferObject->previous)
            {
                dObj->rwQueue->previous = NULL;
            }
            else
            {
                dObj->rwQueue->previous = bufferObject->previous;
            }
            bufferObject->previous = NULL;
        }
        else
        {
            dObj->rwQueue->previous = NULL;
            dObj->rwQueue = NULL;
        }
    }

    return;
}

/* This function removes the buffer objects that were queued by a client. */
static void _DRV_SQI_RemoveClientBufferObjects
(
    const DRV_HANDLE handle,
    DRV_SQI_OBJECT *dObj
)
{
    _DRV_SQI_BUFFER_OBJECT *bufferObject = NULL;
    _DRV_SQI_BUFFER_OBJECT *lastObject = NULL;
    _DRV_SQI_BUFFER_OBJECT *head = NULL;
    _DRV_SQI_BUFFER_OBJECT *temp = NULL;

    bufferObject = dObj->rwQueue;

    if (dObj->rwQueue != NULL)
    {
        dObj->rwQueue->previous = NULL;
    }

    while (bufferObject != NULL)
    {
        temp = bufferObject->next;
        if (bufferObject->hClient == handle)
        {
            bufferObject->inUse = false;

            if(bufferObject->previous != NULL)
            {
                bufferObject->previous->next = bufferObject->next;
            }

            if(bufferObject->next != NULL)
            {
                bufferObject->next->previous = bufferObject->previous;
            }

            bufferObject->previous = NULL;
            bufferObject->next = NULL;
        }
        else
        {
            if (head == NULL)
            {
                head = bufferObject;
            }

            lastObject = bufferObject;
        }

        bufferObject = temp;
    }

    dObj->rwQueue = head;

    if ((head != NULL) && (head != lastObject))
    {
        dObj->rwQueue->previous = lastObject;
    }
}
static void DRV_SQI_FindLaneCfg
(
    uint8_t laneCfg,
    uint8_t *instrLane,
    uint8_t *addrLane,
    uint8_t *dataLane
) 
{
    switch (laneCfg) {
            /* Instruction opcode, Address and Data are all sent in single lane */
        case DRV_SQI_LANE_SINGLE:
        default:
            *instrLane = DRV_SQI_FLAG_MODE_SINGLE_LANE;
            *addrLane = DRV_SQI_FLAG_MODE_SINGLE_LANE;
            *dataLane = DRV_SQI_FLAG_MODE_SINGLE_LANE;
            break;

            /* Instruction opcode and Address are sent in single lane, while data is
             * sent using dual lane. */
        case DRV_SQI_LANE_DUAL_DATA:
            *instrLane = *addrLane = DRV_SQI_FLAG_MODE_SINGLE_LANE;
            *dataLane = DRV_SQI_FLAG_MODE_DUAL_LANE;
            break;

            /* Instruction opcode and Address are sent in single lane, while data is
             * sent using quad lane. */
        case DRV_SQI_LANE_QUAD_DATA:
            *instrLane = *addrLane = DRV_SQI_FLAG_MODE_SINGLE_LANE;
            *dataLane = DRV_SQI_FLAG_MODE_QUAD_LANE;
            break;

            /* Instruction opcode is sent in single lane, Address and Data are sent
             * using dual lane. */
        case DRV_SQI_LANE_DUAL_ADDR_DATA:
            *instrLane = DRV_SQI_FLAG_MODE_SINGLE_LANE;
            *addrLane = DRV_SQI_FLAG_MODE_DUAL_LANE;
            *dataLane = DRV_SQI_FLAG_MODE_DUAL_LANE;
            break;

            /* Instruction opcode is sent in single lane, Address and Data are sent
             * using quad lane. */
        case DRV_SQI_LANE_QUAD_ADDR_DATA:
            *instrLane = DRV_SQI_FLAG_MODE_SINGLE_LANE;
            *addrLane = DRV_SQI_FLAG_MODE_QUAD_LANE;
            *dataLane = DRV_SQI_FLAG_MODE_QUAD_LANE;
            break;

            /* Instruction opcode, Address and Data are sent using dual lanes. */
        case DRV_SQI_LANE_DUAL_ALL:
            *instrLane = *addrLane = DRV_SQI_FLAG_MODE_DUAL_LANE;
            *dataLane = DRV_SQI_FLAG_MODE_DUAL_LANE;
            break;

            /* Instruction opcode, Address and Data are sent using quad lanes. */
        case DRV_SQI_LANE_QUAD_ALL:
            *instrLane = *addrLane = DRV_SQI_FLAG_MODE_QUAD_LANE;
            *dataLane = DRV_SQI_FLAG_MODE_QUAD_LANE;
            break;
    }
}

static _DRV_SQI_BUFFER_OBJECT* _DRV_SQI_AllocateBufferObject2
(
    DRV_HANDLE handle,
    uint8_t sqiDevice,
    DRV_SQI_TransferFrame *frame,
    uint8_t numFrames
)
{
    uint8_t iEntry = 0;
    uint8_t numElements = 0;
    uint8_t elementLength = 0;
    
    uint8_t instrLane = DRV_SQI_FLAG_MODE_SINGLE_LANE;
    uint8_t addrLane = DRV_SQI_FLAG_MODE_SINGLE_LANE;
    uint8_t dataLane = DRV_SQI_FLAG_MODE_SINGLE_LANE;
    
    uint8_t *ptr = NULL;
    
    _DRV_SQI_BUFFER_OBJECT *bufferObj = NULL;

    for (iEntry = 0; iEntry < DRV_SQI_BUFFER_OBJECT_NUMBER; iEntry++)
    {
        /* Search for a free buffer object to use */
        if (gDrvSQIBufferObject[iEntry].inUse == false)
        {
            /* Found a free buffer object. */
            bufferObj = &gDrvSQIBufferObject[iEntry];

            bufferObj->inUse         = true;
            bufferObj->commandHandle = _DRV_SQI_MAKE_HANDLE(gDrvSQIBufferToken, iEntry);
            bufferObj->hClient       = handle;
            bufferObj->sqiDevice     = sqiDevice;
            
            /* Typical sequence flow of data on the flash:
             * Instruction
             * Address
             * Option
             * Dummy Bytes
             * Data
             */
            
            DRV_SQI_FindLaneCfg (frame->laneCfg, &instrLane, &addrLane, &dataLane);
            ptr = elementData0[iEntry];
            memset (&elementData0[iEntry][0], 0, 7);
            memset (&elementData1[iEntry][0], 0, 7);
            
            if (frame->flags & DRV_SQI_FLAG_INSTR_ENABLE_MASK)
            {
                ptr[elementLength++] = frame->instruction;
            }
            
            /* If there is no address or data then the CS needs to be
             * de-asserted after the instruction is sent out. */
            if ((frame->flags & (DRV_SQI_FLAG_ADDR_ENABLE_MASK | DRV_SQI_FLAG_DATA_ENABLE_MASK)) == 0)
            {
                gDrvSQIXferElement[iEntry][numElements].data = &elementData0[iEntry][0];
                gDrvSQIXferElement[iEntry][numElements].length = elementLength;
                gDrvSQIXferElement[iEntry][numElements].flag = (instrLane | DRV_SQI_FLAG_DEASSERT_CS);
                numElements++;
            }
            else
            {
                if (instrLane != addrLane)
                {
                    ptr = elementData1[iEntry];
                    elementLength = 0;
                    numElements++;
                }
                
                if (frame->flags & DRV_SQI_FLAG_ADDR_ENABLE_MASK) 
                {
                    if (frame->flags & DRV_SQI_FLAG_32_BIT_ADDR_ENABLE_MASK)
                    {
                        ptr[elementLength++] = frame->address >> 24;
                    }

                    ptr[elementLength++] = frame->address >> 16;
                    ptr[elementLength++] = frame->address >> 8;
                    ptr[elementLength++] = frame->address & 0xFF;
                }

                /* Check if option is to be sent out. */
                if (frame->flags & DRV_SQI_FLAG_OPT_ENABLE_MASK)
                {
                    ptr[elementLength++] = frame->option;
                }

                /* Check if dummy bytes need to be sent out and the number of
                 * dummy bytes that need to be sent out. */
                elementLength += frame->numDummyBytes;
                
                gDrvSQIXferElement[iEntry][numElements].data = ptr;
                gDrvSQIXferElement[iEntry][numElements].length = elementLength;
                gDrvSQIXferElement[iEntry][numElements].flag = (addrLane);
                
                if ((frame->flags & DRV_SQI_FLAG_DATA_ENABLE_MASK) == 0)
                {
                    /* De-assert the CS as this is the last transfer element. */
                    gDrvSQIXferElement[iEntry][numElements].flag |= DRV_SQI_FLAG_DEASSERT_CS;
                    numElements ++;
                }
                else
                {
                    numElements ++;
                    gDrvSQIXferElement[iEntry][numElements].data = frame->data;
                    gDrvSQIXferElement[iEntry][numElements].length = frame->length;
                    gDrvSQIXferElement[iEntry][numElements].flag = (dataLane | DRV_SQI_FLAG_DEASSERT_CS);
                    if (frame->flags & DRV_SQI_FLAG_DATA_DIRECTION_READ)
                    {
                        gDrvSQIXferElement[iEntry][numElements].flag |= DRV_SQI_FLAG_DIR_READ;
                    }
                    
                    numElements ++;
                }
            }
            
            bufferObj->xferData      = &gDrvSQIXferElement[iEntry][0];
            bufferObj->numElements   = numElements;
            bufferObj->opPending     = true;
            bufferObj->status        = DRV_SQI_COMMAND_QUEUED;
            bufferObj->next          = NULL;
            bufferObj->previous      = NULL;

            /* Update the token number. */
            _DRV_SQI_UPDATE_BUF_TOKEN(gDrvSQIBufferToken);
            break;
        }
    }

    return bufferObj;
}

/* This function finds and allocates a free buffer object which is then
 * populated with the transfer request. */
static _DRV_SQI_BUFFER_OBJECT* _DRV_SQI_AllocateBufferObject
(
    DRV_HANDLE handle,
    uint8_t sqiDevice,
    DRV_SQI_TransferElement *xferData,
    uint8_t numElements
)
{
    uint8_t iEntry = 0;
    _DRV_SQI_BUFFER_OBJECT *bufferObj = NULL;

    for (iEntry = 0; iEntry < DRV_SQI_BUFFER_OBJECT_NUMBER; iEntry++)
    {
        /* Search for a free buffer object to use */
        if (gDrvSQIBufferObject[iEntry].inUse == false)
        {
            /* Found a free buffer object. */
            bufferObj = &gDrvSQIBufferObject[iEntry];

            bufferObj->inUse         = true;
            bufferObj->commandHandle = _DRV_SQI_MAKE_HANDLE(gDrvSQIBufferToken, iEntry);
            bufferObj->hClient       = handle;
            bufferObj->sqiDevice     = sqiDevice;
            bufferObj->xferData      = xferData;
            bufferObj->numElements   = numElements;
            bufferObj->opPending     = true;
            bufferObj->status        = DRV_SQI_COMMAND_QUEUED;
            bufferObj->next          = NULL;
            bufferObj->previous      = NULL;

            /* Update the token number. */
            _DRV_SQI_UPDATE_BUF_TOKEN(gDrvSQIBufferToken);
            break;
        }
    }

    return bufferObj;
}

/* This function finds and allocates a free client object. */
static DRV_SQI_CLIENT_OBJECT* _DRV_SQI_AllocateClientObject
(
    void
)
{
    uint8_t iClient = 0;
    DRV_SQI_CLIENT_OBJECT *object = &gDrvSQIClientObj[0];

    /* Find available slot in array of client objects */
    for (iClient = 0; iClient < DRV_SQI_CLIENTS_NUMBER ; iClient++)
    {
        if (!object->inUse)
        {
            return object;
        }

        object ++;
    }

    return NULL;
}

/* This function updates the current sqi device to be operated upon.
 * */
static void DRV_SQI_UpdateCurrentDevice
(
    DRV_SQI_OBJECT *dObj
)
{
    if (dObj->enabledDevices == DRV_SQI_ENABLE_BOTH_DEVICES)
    {
        if (dObj->currentSqiDevice != dObj->currentBufObj->sqiDevice)
        {
            dObj->currentSqiDevice = dObj->currentBufObj->sqiDevice;
        }
    }
}

/* This function creates the BD chain and updates the required DMA registers.
 * */
static void _DRV_SQI_CreateBDList
(
    DRV_SQI_OBJECT *dObj
)
{
    _DRV_SQI_DMA_Desc *bdDesc = NULL;
    uint8_t i = 0;
    uint8_t flag = 0;
    _DRV_SQI_BUFFER_OBJECT *bufferObj = dObj->currentBufObj;
    bool last = false;
    bool lastBd = false;
    uint8_t threshold = DRV_SQI_MAX_THRESHOLD_VALUE;
    uint32_t length = 0;

    bdDesc = &gSQIDmaBDDesc[0];

    for (i = 0; i < DRV_SQI_DMA_BUFFER_DESC_NUMBER; i++)
    {
        flag = bufferObj->xferData[dObj->currentElement].flag;
        /* configure the BD Control register */
        bdDesc->bdCtrl = 0;
        bdDesc->nextAddr = 0;

        if ((dObj->currentSqiDevice == 1) || ((dObj->enabledDevices == DRV_SQI_ENABLE_DEVICE_1) && (dObj->currentSqiDevice == 0)))
        {
            bdDesc->bdCtrl |= _DRV_SQI_BDCTRL_SQICS1;
        }

        if (dObj->devCfg[dObj->currentSqiDevice].lsbFirst)
        {
            bdDesc->bdCtrl |= _DRV_SQI_BDCTRL_LSBF;
        }

        if (flag & DRV_SQI_FLAG_MODE_QUAD_LANE)
        {
            bdDesc->bdCtrl |= _DRV_SQI_BDCTRL_MODE_QUAD_LANE;
        }
        else if (flag & DRV_SQI_FLAG_MODE_DUAL_LANE)
        {
            bdDesc->bdCtrl |= _DRV_SQI_BDCTRL_MODE_DUAL_LANE;
        }

        if (flag & DRV_SQI_FLAG_DIR_READ)
        {
            bdDesc->bdCtrl |= _DRV_SQI_BDCTRL_DIR_READ;
        }

        bdDesc->bufAddr = (uint32_t *)_DRV_SQI_KVA_TO_PA((uint32_t)&bufferObj->xferData[dObj->currentElement].data[dObj->bufferOffset]);

        /* Find the number of bytes to be transferred as part of this BD. */
        length = bufferObj->xferData[dObj->currentElement].length - dObj->bufferOffset;
        if (length > 256)
        {
            uint16_t temp = 0;
            length = 256;

            temp = bufferObj->xferData[dObj->currentElement].length - dObj->bufferOffset;
            if (temp < 256)
            {
                if (!(temp / DRV_SQI_MAX_THRESHOLD_VALUE))
                {
                    lastBd = true;
                }
            }
        }
        else
        {
            uint16_t quotient = 0;
            uint16_t remainder = 0;
            quotient = length / DRV_SQI_MAX_THRESHOLD_VALUE;
            remainder = length % DRV_SQI_MAX_THRESHOLD_VALUE;

            if (i != 0)
            {
                length = quotient * DRV_SQI_MAX_THRESHOLD_VALUE;

                /* We have already chosen a threshold value of
                 * DRV_SQI_MAX_THRESHOLD_VALUE. Check if we can transfer data
                 * of length matching this threshold.
                 * */
                if (quotient)
                {
                    if (remainder == 0)
                    {
                        last = true;
                    }
                }
            }
            else
            {
                /* Length is less than 256. */
                if (quotient)
                {
                    /* Max threshold value can be used. */
                    length = quotient * DRV_SQI_MAX_THRESHOLD_VALUE;
                    threshold = DRV_SQI_MAX_THRESHOLD_VALUE;
                    if (remainder == 0)
                    {
                        last = true;
                    }
                    else
                    {
                        lastBd = true;
                    }
                }
                else
                {
                    threshold = 1;
                    length = remainder;
                    last = true;
                }
            }
        }

        if (last)
        {
            /* At the end of the current xfer element. Check if the CS needs to
             * be de-asserted. */
            if (flag & DRV_SQI_FLAG_DEASSERT_CS)
            {
                bdDesc->bdCtrl |= _DRV_SQI_BDCTRL_DEASSERT;
            }

            /* Go to the next xfer element. */
            dObj->currentElement ++;

            /* Reset the buffer offset. */
            dObj->bufferOffset = 0;
        }
        else
        {
            /* There is still data left to be processed after this. */
            dObj->bufferOffset += length;
        }

        /* Configure the Tx and Rx threshold values. */
        PLIB_SQI_TxBufferThresholdSet (dObj->sqiId, threshold);
        PLIB_SQI_RxBufferThresholdSet (dObj->sqiId, threshold);

        /* Check if DDR mode needs to be set. */
        if (flag & DRV_SQI_FLAG_DDR_MODE)
        {
            PLIB_SQI_DDRModeSet(dObj->sqiId);
        }
        else
        {
            if (PLIB_SQI_DDRModeGet (dObj->sqiId))
            {
                PLIB_SQI_DDRModeClear(dObj->sqiId);
            }
        }

        bdDesc->bdCtrl |= (_DRV_SQI_BDCTRL_PKT_INT_ENABLE | _DRV_SQI_BDCTRL_DESCEN);

        /* Set the transfer length. */
        bdDesc->bdCtrl |= (length);

        /* Reserved. */
        bdDesc->bdStat = 0x0;

        if (((i + 1) == DRV_SQI_DMA_BUFFER_DESC_NUMBER) || (dObj->currentElement == bufferObj->numElements))
        {
            lastBd = true;
        }

        if (last || lastBd)
        {
            /* This is 
             * 1. Either the end of the xfer request or
             * 2. There are no more DMA BDs available or
             * 3. Transfer involves size of less than max threshold value.
             * */

            /* Set the lastpkt and the lastbd flag. */
            bdDesc->bdCtrl |= (_DRV_SQI_BDCTRL_LASTPKT | _DRV_SQI_BDCTRL_LASTBD);
            bdDesc->nextAddr = 0x00000000;

            break;
        }
        else
        {
            bdDesc->nextAddr = (struct _DRV_SQI_DMA_Desc *)_DRV_SQI_KVA_TO_PA((uint32_t)&gSQIDmaBDDesc[i + 1]);
        }

        /* Go to the next DMA BD. */
        bdDesc++;
    }
}

/* This function enables the required interrupt mask bits, enables the
 * interrupt and kickstarts the DMA operation. */
static void _DRV_SQI_StartDMAOperation
(
    DRV_SQI_OBJECT *dObj
)
{
    PLIB_SQI_DMABDBaseAddressSet(dObj->sqiId, (void *)_DRV_SQI_KVA_TO_PA((uint32_t)&gSQIDmaBDDesc[0]));

    /* Enable the DMA Error and the Packet Complete Interrupts. */
    //PLIB_SQI_InterruptSignalEnable(dObj->sqiId, SQI_DMAERROR);
    PLIB_SQI_InterruptSignalEnable(dObj->sqiId, SQI_PKTCOMP);
    //PLIB_SQI_InterruptEnable(dObj->sqiId, SQI_DMAERROR);
    PLIB_SQI_InterruptEnable(dObj->sqiId, SQI_PKTCOMP);

    /* Clear the interrupt flag */
    SYS_INT_SourceStatusClear(dObj->interruptSource);
    /* Enable the SQI Interrupt. */
    SYS_INT_SourceEnable(dObj->interruptSource);

    /* Kick start the DMA operation. */
    PLIB_SQI_DMAEnable(dObj->sqiId);
    PLIB_SQI_DMABDFetchStart(dObj->sqiId);
}

// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_SQI_Initialize
    ( 
        const SYS_MODULE_INDEX index,
        const SYS_MODULE_INIT * const init 
    );
    
  Summary:
    Initializes the SQI instance for the specified driver index

  Description:
    This routine initializes the SQI driver instance for the specified
    driver index, making it ready for clients to open and use it.

  Remarks:
    This routine must be called before any other SQI routine is called.
    
    This routine should only be called once during system initialization unless
    DRV_SQI_Deinitialize is called to deinitialize the driver instance.
    
    This routine will NEVER block for hardware access. If the operation
    requires time to allow the hardware to initialize, it will be reported by
    the DRV_SQI_Status operation. The system must use DRV_SQI_Status to find
    out when the driver is in the ready state.
*/

SYS_MODULE_OBJ DRV_SQI_Initialize
(
    const SYS_MODULE_INDEX drvIndex,
    const SYS_MODULE_INIT *const init
)
{
    OSAL_RESULT retVal = OSAL_RESULT_FALSE;
    DRV_SQI_OBJECT *dObj = NULL;
    DRV_SQI_INIT *sqiInit = NULL;
    SQI_CS_OEN csPins = SQI_CS_OEN_NONE;

    /* Validate the driver index */
    if (drvIndex > DRV_SQI_INSTANCES_NUMBER)
    {
        return SYS_MODULE_OBJ_INVALID;
    }

    /* Check if the instance has already been initialized */
    if (gDrvSQIObj[drvIndex].inUse)
    {
        return SYS_MODULE_OBJ_INVALID;
    }

    sqiInit = (DRV_SQI_INIT *)init;

    retVal = OSAL_MUTEX_Create(&sqiClientObjMutex);
    if (retVal != OSAL_RESULT_TRUE)
    {
        return SYS_MODULE_OBJ_INVALID;
    }

    retVal = OSAL_MUTEX_Create(&sqiBufObjMutex);
    if (retVal != OSAL_RESULT_TRUE)
    {
        return SYS_MODULE_OBJ_INVALID;
    }

    dObj = &gDrvSQIObj[drvIndex];

    /* Indicate tha this object is in use */
    dObj->inUse = true;

    /* Update the SQI PLIB Id */
    dObj->sqiId = sqiInit->sqiId;

    /* Initialize the Interrupt Source */
    dObj->interruptSource = sqiInit->interruptSource;

    /* Clear the interrupt flag if any */
    SYS_INT_SourceStatusClear(dObj->interruptSource);

    dObj->enabledDevices = sqiInit->enabledDevices;
    dObj->devCfg = &sqiInit->devCfg[0];
    dObj->clockDivider = sqiInit->clockDivider;

    /* Initialize number of clients */
    dObj->numClients = 0;
    dObj->currentElement = 0;

    dObj->currentSqiDevice = 0;
    /* Find which cs pins have to be enabled. */
    if (dObj->enabledDevices == DRV_SQI_ENABLE_BOTH_DEVICES)
    {
        csPins = SQI_CS_OEN_BOTH;
    }
    else if (dObj->enabledDevices == DRV_SQI_ENABLE_DEVICE_1)
    {
        csPins = SQI_CS_OEN_1;
    }
    else
    {
        csPins = SQI_CS_OEN_0;
    }

    /* Configure the SQI Configuration register and enable the module. */
    PLIB_SQI_ConfigWordSet(dObj->sqiId,
                           true, /* SQI Enable */ 
                           csPins, /* CS OEN */
                           SQI_DATA_OEN_QUAD, /* SQID3-SQID0 are enabled. */
                           false, /* Sofware reset select bit. */
                           true,  /* Burst Enable. */
                           false, /* Used as SQID3 instead of HOLD Pin */
                           false, /* Used as SQID2 instead of WP Pin */
                           false, /* rxLatch */
                           dObj->devCfg[0].lsbFirst, /* lsb first */
                           dObj->devCfg[0].spiMode, /* SPI MODE0/MODE3 operation */
                           SQI_XFER_MODE_DMA /* DMA Mode of transfer */
                           );


    /* Configure the SQI Clock. */
    PLIB_SQI_ClockDividerSet (dObj->sqiId, dObj->clockDivider);
    PLIB_SQI_ClockEnable (dObj->sqiId);

    /* Clear the Interrupt Enable and Interrupt Signal Enable registers. */
    PLIB_SQI_InterruptDisableAll(dObj->sqiId);
    PLIB_SQI_InterruptSignalDisableAll (dObj->sqiId);

    /* Set the Control buffer, RX and TX command thresholds. */
	PLIB_SQI_ControlBufferThresholdSet(dObj->sqiId, 1);
	PLIB_SQI_TxBufferThresholdSet(dObj->sqiId, 1);
    PLIB_SQI_RxBufferThresholdSet(dObj->sqiId, 1);

    /* Wait for the clock to stabilize. */
    while (PLIB_SQI_ClockIsStable (dObj->sqiId) == 0);

    dObj->status = SYS_STATUS_READY;

    /* Return the driver index and the System Module Object */
    return drvIndex;
}

// ****************************************************************************
/* Function:
    void DRV_SQI_Deinitialize
    (
        SYS_MODULE_OBJ object 
    );
    
  Summary:
    Deinitializes the specified instance of the SQI driver module

  Description:
    Deinitializes the specified instance of the SQI driver module, disabling its
    operation (and any hardware). Invalidates all the internal data.
  
  Remarks:
    Once the Initialize operation has been called, the Deinitialize operation
    must be called before the Initialize operation can be called again.
*/

void DRV_SQI_Deinitialize
(
    SYS_MODULE_OBJ object
)
{
    DRV_SQI_OBJECT * dObj = (DRV_SQI_OBJECT*)NULL;

    /* Validate the object */
    if ((object == SYS_MODULE_OBJ_INVALID) || (object >= DRV_SQI_INSTANCES_NUMBER))
    {
        return;
    }

    dObj = (DRV_SQI_OBJECT*)&gDrvSQIObj[object];

    /* Disable the Interrupt */
    SYS_INT_SourceDisable(dObj->interruptSource);

    /* Reset the client count and the exclusive flag */
    dObj->numClients = 0;
    dObj->isExclusive = false;

    /* Reset the queue */
    dObj->rwQueue= NULL;

    /* Set the Hardware instance object status an un-initialized */
    dObj->status = SYS_STATUS_UNINITIALIZED;

    /* Hardware instance object is no more in use */
    dObj->inUse = false;

    OSAL_MUTEX_Delete(&sqiClientObjMutex);
    OSAL_MUTEX_Delete(&sqiBufObjMutex);
}

// ****************************************************************************
/* Function:
    SYS_STATUS DRV_SQI_Status( SYS_MODULE_OBJ object )

  Summary:
    Gets the current status of the SQI driver module.

  Description:
    This routine provides the current status of the SQI driver module.

  Remarks:
    Refer to drv_sqi.h for usage information.
*/

SYS_STATUS DRV_SQI_Status
(
    SYS_MODULE_OBJ object
)
{
    /* Validate the object */
    if ((object == SYS_MODULE_OBJ_INVALID) || (object >= DRV_SQI_INSTANCES_NUMBER))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_Status(): Invalid parameter.\n");
        return SYS_STATUS_UNINITIALIZED;
    }

    /* Return the driver status */
    return (gDrvSQIObj[object].status);
}

// ****************************************************************************
/* Function:
    DRV_HANDLE DRV_SQI_Open
    ( 
        const SYS_MODULE_INDEX index,
        const DRV_IO_INTENT ioIntent
    );
    
  Summary:
    Opens the specified SQI driver instance and returns a handle to it
  
  Description:
    This routine opens the specified SQI driver instance and provides a handle.
    This handle must be provided to all other client-level operations to
    identify the caller and the instance of the driver.
  
  Remarks:
    The handle returned is valid until the DRV_SQI_Close routine is called.
    This routine will NEVER block waiting for hardware. If the driver has 
    has already been opened, it cannot be opened exclusively.
*/

DRV_HANDLE DRV_SQI_Open
(
    const SYS_MODULE_INDEX drvIndex,
    const DRV_IO_INTENT ioIntent
)
{
    DRV_SQI_CLIENT_OBJECT *clientObj = NULL;
    DRV_SQI_OBJECT *dObj = NULL;
    OSAL_RESULT retVal = OSAL_RESULT_FALSE;

    if (drvIndex >= DRV_SQI_INSTANCES_NUMBER)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_Open(): Invalid driver index.\n");
        return DRV_HANDLE_INVALID;
    }

    dObj = &gDrvSQIObj[drvIndex];
    if (dObj->status != SYS_STATUS_READY)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_Open(): Driver not ready.\n");
        return DRV_HANDLE_INVALID;
    }

    /* Check if the driver has already been opened in exclusive mode */
    if ((dObj->isExclusive) || ((dObj->numClients > 0) && (ioIntent & DRV_IO_INTENT_EXCLUSIVE)))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_Open(): Driver cannot be opened exclusively.\n");
        return DRV_HANDLE_INVALID;
    }

    /* Obtain the Client object mutex */
    retVal = OSAL_MUTEX_Lock(&sqiClientObjMutex, OSAL_WAIT_FOREVER);
    if (retVal != OSAL_RESULT_TRUE)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_Open(): Failed to acquire the SQI Client Object Mutex.\n");
        return DRV_HANDLE_INVALID;
    }

    clientObj = _DRV_SQI_AllocateClientObject ();

    if (clientObj == NULL)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_Open(): Failed to allocate a Client Object.\n");
    }
    else
    {
        /* Found a client object that can be used */
        clientObj->inUse = true;
        clientObj->driverObj =  dObj;
        clientObj->intent = ioIntent;
        clientObj->eventHandler = NULL;

        if (ioIntent & DRV_IO_INTENT_EXCLUSIVE)
        {
            /* Driver was opened in exclusive mode */
            dObj->isExclusive = true;
        }

        dObj->numClients ++;
        
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_Open(): Open successful.\n");
    }

    OSAL_MUTEX_Unlock(&sqiClientObjMutex);

    return clientObj ? ((DRV_HANDLE)clientObj) : DRV_HANDLE_INVALID;
}

// *****************************************************************************
/* Function:
    void DRV_SQI_Close
    (
        const DRV_HANDLE handle
    );

  Summary:
    Closes an opened-instance of the SQI driver

  Description:
    This routine closes an opened-instance of the SQI driver, invalidating the
    handle.

  Remarks:
    After calling this routine, the handle passed in "handle" must not be used
    with any of the remaining driver routines. A new handle must be obtained by
    calling DRV_SQI_Open before the caller may use the driver again. Usually
    there is no need for the driver client to verify that the Close operation
    has completed.
*/

void DRV_SQI_Close
(
    const DRV_HANDLE handle
)
{
    DRV_SQI_CLIENT_OBJECT *clientObj = NULL;
    DRV_SQI_OBJECT *dObj = NULL;

    /* Get the Client object from the handle passed */
    clientObj = _DRV_SQI_ValidateClientHandle(handle);
    /* Check if the driver handle is valid */
    if (clientObj == NULL)
    {
        SYS_DEBUG_PRINT (SYS_ERROR_INFO, "DRV_SQI_Close(): Invalid handle.\n");
        return;
    }

    dObj = clientObj->driverObj;
    _DRV_SQI_RemoveClientBufferObjects (handle, dObj);

    /* Update the client count */
    dObj->numClients --;
    dObj->isExclusive = false;

    /* Free the Client Instance */
    clientObj->inUse = false;

    SYS_DEBUG_PRINT (SYS_ERROR_INFO, "DRV_SQI_Close(): Close successful.\n");
    return;
}

// *****************************************************************************
/* Function:
    void DRV_SQI_TransferFrames
    (
        DRV_HANDLE handle,
        DRV_SQI_COMMAND_HANDLE *commandHandle,
        DRV_SQI_TransferFrame *frame,
        uint8_t numFrames
    );

  Summary:
    Queue a transfer operation on the SQI device.

  Description:
    This routine queues a transfer operation on the SQI device. In order to
    perform any operation on the sqi flash device, a one byte instruction
    specifying the operation to be performed needs to be sent out. This is
    followed by optional address from/to which data is to be read/written,
    option, dummy and data bytes.

    If an event handler is registered with the driver the event handler would
    be invoked with the status of the operation once the operation has been
    completed. The function returns DRV_SQI_COMMAND_HANDLE_INVALID in the
    commandHandle argument under the following circumstances:
    - if the driver handle is invalid
    - if the transfer element is NULL or number of transfer elements is zero
    - if a buffer object could not be allocated to the request

  Remarks:
    See the drv_sqi.h for usage information.
*/
void DRV_SQI_TransferFrames
(
    DRV_HANDLE handle,
    DRV_SQI_COMMAND_HANDLE *commandHandle,
    DRV_SQI_TransferFrame *frame,
    uint8_t numFrames
)
{
    bool start = false;
    bool isEnabled = false;
    DRV_SQI_CLIENT_OBJECT *clientObj = NULL;
    DRV_SQI_OBJECT *dObj = NULL;
    _DRV_SQI_BUFFER_OBJECT *bufferObj = NULL;
    OSAL_RESULT retVal = OSAL_RESULT_FALSE;
    DRV_SQI_COMMAND_HANDLE *tempHandle1 = NULL, tempHandle2;
    uint8_t sqiDevice = 0;
    
    clientObj = _DRV_SQI_ValidateClientHandle(handle);

    tempHandle1 = commandHandle ? commandHandle : &tempHandle2;
    *tempHandle1 = DRV_SQI_COMMAND_HANDLE_INVALID;

    /* Validate the client handle */
    if (clientObj == NULL)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_TransferFrames(): Invalid handle.\n");
        return;
    }

    /* Validate the parameters */
    if ((frame == NULL) || (numFrames == 0))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_TransferFrames(): Invalid parameters.\n");
        return;
    }

    dObj = clientObj->driverObj;

    /* Validate the sqiDevice */
    sqiDevice = ((frame->flags >> DRV_SQI_FLAG_SQI_CS_NUMBER_POS) & 0x03);
    if (_DRV_SQI_ValidateSqiDevice (dObj, sqiDevice) == false)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_TransferFrames(): Invalid Sqi Device Id.\n");
        return;
    }

    /* Obtain the buffer object mutex */
    retVal = OSAL_MUTEX_Lock(&sqiBufObjMutex, OSAL_WAIT_FOREVER);
    if (retVal != OSAL_RESULT_TRUE)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_TransferFrames(): Failed to acquire the SQI Buffer Object Mutex.\n");
        return;
    }
    
    /* Disable the Interrupt */
    isEnabled = SYS_INT_SourceDisable(dObj->interruptSource);

    bufferObj = _DRV_SQI_AllocateBufferObject2 (handle, sqiDevice, frame, numFrames);
    if (bufferObj == NULL)
    {
        if (isEnabled)
        {
            SYS_INT_SourceEnable (dObj->interruptSource);
        }

        OSAL_MUTEX_Unlock(&sqiBufObjMutex);

        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_TransferFrames(): Failed to queue the request.");
        return;
    }

    *tempHandle1 = (bufferObj->commandHandle);
    if (dObj->rwQueue == NULL)
    {
        start = true;
    }

    _DRV_SQI_AddToQueue (dObj, bufferObj);

    if (isEnabled)
    {
        SYS_INT_SourceEnable (dObj->interruptSource);
    }

    OSAL_MUTEX_Unlock(&sqiBufObjMutex);

    if (start)
    {
        dObj->currentBufObj = dObj->rwQueue;
        dObj->currentBufObj->status = DRV_SQI_COMMAND_IN_PROGRESS;

        DRV_SQI_UpdateCurrentDevice (dObj);

        _DRV_SQI_CreateBDList(dObj);
        _DRV_SQI_StartDMAOperation(dObj);
    }

    SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_TransferFrames(): Request queued successfully.");
}

// *****************************************************************************
/* Function:
    void DRV_SQI_TransferData
    (
        DRV_HANDLE handle,
        DRV_SQI_COMMAND_HANDLE *commandHandle,
        uint8_t sqiDevice,
        DRV_SQI_TransferElement *xferData,
        uint8_t numElements
    );

  Summary:
    Queue a data transfer operation on the specified SQI device.

  Description:
    This routine queues a data transfer operation on the specified SQI device.
    The reads or writes of blocks of data generally involves sending down the
    read or a write command, the address on the device from/to which data is to
    be read/written. The client also has to specify the source or destination
    buffer and the number of bytes to be read or written. The client builds an
    array of transfer elements containing these information and passes the
    array and the number of elements of the array as part of this transfer
    operation. If an event handler is registered with the driver the event
    handler would be invoked with the status of the operation once the
    operation has been completed. The function returns
    DRV_SQI_COMMAND_HANDLE_INVALID in the commandHandle argument under the
    following circumstances:
    - if the driver handle is invalid
    - if the transfer element is NULL or number of transfer elements is zero
    - if a buffer object could not be allocated to the request

  Remarks:
    None.
*/

void DRV_SQI_TransferData
(
    DRV_HANDLE handle,
    DRV_SQI_COMMAND_HANDLE *commandHandle,
    uint8_t sqiDevice,
    DRV_SQI_TransferElement *xferData,
    uint8_t numElements
)
{
    bool start = false;
    bool isEnabled = false;
    DRV_SQI_CLIENT_OBJECT *clientObj = NULL;
    DRV_SQI_OBJECT *dObj = NULL;
    _DRV_SQI_BUFFER_OBJECT *bufferObj = NULL;
    OSAL_RESULT retVal = OSAL_RESULT_FALSE;
    DRV_SQI_COMMAND_HANDLE *tempHandle1 = NULL, tempHandle2;
    
    clientObj = _DRV_SQI_ValidateClientHandle(handle);

    tempHandle1 = commandHandle ? commandHandle : &tempHandle2;
    *tempHandle1 = DRV_SQI_COMMAND_HANDLE_INVALID;

    /* Validate the client handle */
    if (clientObj == NULL)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_TransferData(): Invalid handle.\n");
        return;
    }

    /* Validate the parameters */
    if ((xferData == NULL) || (numElements == 0))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_TransferData(): Invalid parameters.\n");
        return;
    }

    dObj = clientObj->driverObj;

    /* Validate the sqiDevice */
    if (_DRV_SQI_ValidateSqiDevice (dObj, sqiDevice) == false)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_TransferData(): Invalid Sqi Device Id.\n");
        return;
    }

    /* Obtain the buffer object mutex */
    retVal = OSAL_MUTEX_Lock(&sqiBufObjMutex, OSAL_WAIT_FOREVER);
    if (retVal != OSAL_RESULT_TRUE)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_TransferData(): Failed to acquire the SQI Buffer Object Mutex.\n");
        return;
    }
    
    /* Disable the Interrupt */
    isEnabled = SYS_INT_SourceDisable(dObj->interruptSource);

    bufferObj = _DRV_SQI_AllocateBufferObject (handle, sqiDevice, xferData, numElements);
    if (bufferObj == NULL)
    {
        if (isEnabled)
        {
            SYS_INT_SourceEnable (dObj->interruptSource);
        }

        OSAL_MUTEX_Unlock(&sqiBufObjMutex);

        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_TransferData(): Failed to queue the request.");
        return;
    }

    *tempHandle1 = (bufferObj->commandHandle);
    if (dObj->rwQueue == NULL)
    {
        start = true;
    }

    _DRV_SQI_AddToQueue (dObj, bufferObj);

    if (isEnabled)
    {
        SYS_INT_SourceEnable (dObj->interruptSource);
    }

    OSAL_MUTEX_Unlock(&sqiBufObjMutex);

    if (start)
    {
        dObj->currentBufObj = dObj->rwQueue;
        dObj->currentBufObj->status = DRV_SQI_COMMAND_IN_PROGRESS;

        DRV_SQI_UpdateCurrentDevice (dObj);

        _DRV_SQI_CreateBDList(dObj);
        _DRV_SQI_StartDMAOperation(dObj);
    }

    SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_TransferData(): Request queued successfully.");
}

// *****************************************************************************
/* Function:
    void DRV_SQI_EventHandlerSet
    (
        const DRV_HANDLE handle,
        const void *eventHandler,
        const uintptr_t context
    );

  Summary:
    Allows a client to identify an event handling function for the driver to
    call back when queued operation has completed.

  Description:
    This function allows a client to identify an event handling function for
    the driver to call back when queued operation has completed. When a client
    calls read or a write function, it is provided with a handle identifying
    the command that was added to the driver's buffer queue.  The driver will
    pass this handle back to the client by calling "eventHandler" function when
    the queued operation has completed.
    
    The event handler should be set before the client performs any read or
    write operations that could generate events. The event handler once set,
    persists until the client closes the driver or sets another event handler
    (which could be a "NULL" pointer to indicate no callback).

  Remarks:
    If the client does not want to be notified when the queued operation has
    completed, it does not need to register a callback.
*/

void DRV_SQI_EventHandlerSet
(
    const DRV_HANDLE handle,
    const void * eventHandler,
    const uintptr_t context
)
{
    DRV_SQI_CLIENT_OBJECT *clientObj;

    clientObj = _DRV_SQI_ValidateClientHandle(handle);
    /* Check if the client handle is valid */
    if (clientObj == NULL)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_EventHandlerSet(): Invalid driver handle.\n");
        return;
    }

    /* Set the event handler */
    clientObj->eventHandler = eventHandler;
    clientObj->context = context;
}
// *****************************************************************************
/* Function:
    DRV_SQI_COMMAND_STATUS DRV_SQI_CommandStatus
    (
        const DRV_HANDLE handle, 
        const DRV_SQI_COMMAND_HANDLE commandHandle
    );

  Summary:
    Gets the current status of the command.

  Description:
    This routine gets the current status of the command. The application must
    use this routine where the status of a scheduled command needs to polled
    on. The function may return DRV_SQI_COMMAND_COMPLETED in a case where the
    command handle has expired. A command handle expires when the internal
    buffer object is re-assigned to another request. It is recommended that
    this function be called regularly in order to track the command status
    correctly.

    The application can alternatively register an event handler to receive the
    operation completion events.

  Remarks:
    Refer to drv_sqi.h for usage information.
*/

DRV_SQI_COMMAND_STATUS DRV_SQI_CommandStatus
(
    const DRV_HANDLE handle,
    const DRV_SQI_COMMAND_HANDLE commandHandle
)
{
    uint16_t iEntry = 0;

    /* Validate the client handle */
    if (_DRV_SQI_ValidateClientHandle(handle) == NULL)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_CommandStatus(): Invalid driver handle.\n");
        return DRV_SQI_COMMAND_ERROR_UNKNOWN;
    }

    if (commandHandle == DRV_SQI_COMMAND_HANDLE_INVALID)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_CommandStatus(): Invalid command handle.\n");
        return DRV_SQI_COMMAND_ERROR_UNKNOWN;
    }

    /* The upper 16 bits of the command handle are the token and the lower 16
     * bits are index into the gDrvSQIBufferObject array */
    iEntry = commandHandle & 0xFFFF;

    if (gDrvSQIBufferObject[iEntry].commandHandle != commandHandle)
    {
        /* This means that object has been re-used by another request. Indicate
         * that the operation is completed.  */
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_CommandStatus(): Command handle is expired.\n");
        return (DRV_SQI_COMMAND_COMPLETED);
    }

    /* Return the last known buffer object status */
    return (gDrvSQIBufferObject[iEntry].status);
}

// ****************************************************************************
/* Function:
    void DRV_SQI_Tasks 
    (
        SYS_MODULE_OBJ object
    );
    
  Summary:
    Maintains the driver's task state machine.
  
  Description:
    This routine is used to maintain the driver's internal task state machine.
  
  Remarks:
    This routine is to be called by the system's task routine(SYS_Tasks) or
    from the ISR.
*/

void DRV_SQI_Tasks
(
    SYS_MODULE_OBJ object
)
{
    uint8_t error = 0;
    uint8_t done = 0;
    uint32_t flags = 0;
    bool start = false;

    DRV_SQI_OBJECT *dObj = NULL;
    DRV_SQI_CLIENT_OBJECT *clientObj = NULL;
    DRV_SQI_EVENT event = DRV_SQI_EVENT_COMMAND_ERROR;

    if (object == SYS_MODULE_OBJ_INVALID)
    {
        /* Invalid system object */
        return;
    }

    dObj = &gDrvSQIObj[object];

    flags = PLIB_SQI_InterruptWordGet (dObj->sqiId);

    error = (flags & (1 << SQI_DMAERROR)) ? 1 : 0;
    done  = (flags & (1 << SQI_PKTCOMP)) ? 1: 0;

    /* Clear the Interrupt Status bits */
    PLIB_SQI_InterruptWordClear(dObj->sqiId, flags);

    PLIB_SQI_InterruptDisableAll(dObj->sqiId);
    PLIB_SQI_InterruptSignalDisableAll(dObj->sqiId);

    /* Clear the interrupt flag */
    SYS_INT_SourceStatusClear(dObj->interruptSource);
    SYS_INT_SourceDisable(dObj->interruptSource);

    /* Disable the BD processor and stop the DMA. */
    PLIB_SQI_DMABDFetchStop(dObj->sqiId);
    PLIB_SQI_DMADisable(dObj->sqiId);

    if (((dObj->currentElement == dObj->currentBufObj->numElements) && done) || error)
    {
        if (error)
        {
            event = DRV_SQI_EVENT_COMMAND_ERROR;
            dObj->currentBufObj->status = DRV_SQI_COMMAND_ERROR_UNKNOWN;
        }
        else
        {
            event = DRV_SQI_EVENT_COMMAND_COMPLETE;
            dObj->currentBufObj->status = DRV_SQI_COMMAND_COMPLETED;
        }

        dObj->currentElement = 0;

        /* Request is complete. Update the queue. */
        _DRV_SQI_UpdateQueue (dObj);

        /* Invoke the callback with the result of the transfer. */
        clientObj = (DRV_SQI_CLIENT_OBJECT *)dObj->currentBufObj->hClient;
        if(clientObj->eventHandler != NULL)
        {
            /* Call the event handler */
            clientObj->eventHandler(event, dObj->currentBufObj->commandHandle, (void*)clientObj->context);
        }

        dObj->currentBufObj = dObj->rwQueue;
        start = true;
    }

    if (dObj->currentBufObj != NULL)
    {
        if (start)
        {
            DRV_SQI_UpdateCurrentDevice (dObj);
        }

        /* Either this is a continuation of the previous request or a new request. */
        _DRV_SQI_CreateBDList(dObj);
        _DRV_SQI_StartDMAOperation(dObj);
    }
}

