/*******************************************************************************
  SQI Driver Local Data Structures

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sqi_local.h

  Summary:
    SQI driver local declarations and definitions

  Description:
    This file contains the SQI driver's local declarations and definitions.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2016 - 2017 released Microchip Technology Inc. All rights reserved.

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

#ifndef _DRV_SQI_LOCAL_H
#define _DRV_SQI_LOCAL_H

// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "driver/sqi/drv_sqi.h"
#include "driver/sqi/drv_sqi_init.h"
#include "peripheral/sqi/plib_sqi.h"
#include "system/int/sys_int.h"
#include "osal/osal.h"

// *****************************************************************************
/* SQI DMA Descriptor related bit field macros.

  Summary:
    SQI DMA Descriptor related bit field macros.

  Description:
    These macros define the bit fields of the SQI DMA Buffer Descriptor
    registers.

  Remarks:
    None
*/
#define _DRV_SQI_KVA_TO_PA(address)    ((address) & 0x1FFFFFFFu)

#define BIT(x)  (1ul << (x))

#define _DRV_SQI_BDCTRL_DESCEN              BIT(31)
/* Bit 30 */
#define _DRV_SQI_BDCTRL_DEASSERT            BIT(30)
/* Bits 29-28 */
#define _DRV_SQI_BDCTRL_SQICS1              BIT(28)
/* Bit 26 */
#define _DRV_SQI_BDCTRL_SCHECK              BIT(26)
/* Bit 25 */
#define _DRV_SQI_BDCTRL_LSBF                BIT(25)
/* Bits 23-22 */
#define _DRV_SQI_BDCTRL_MODE_QUAD_LANE      BIT(23)
#define _DRV_SQI_BDCTRL_MODE_DUAL_LANE      BIT(22)
#define _DRV_SQI_BDCTRL_DIR_READ            BIT(20)
#define _DRV_SQI_BDCTRL_LASTBD              BIT(19)
#define _DRV_SQI_BDCTRL_LASTPKT             BIT(18)
#define _DRV_SQI_BDCTRL_PKT_INT_ENABLE      BIT(17)
#define _DRV_SQI_BDCTRL_BDDONE_INT_ENABLE   BIT(16)
#define _DRV_SQI_BDCTRL_BUF_LEN_MASK        (0x1FF)

/* Max threshold value for TX and RX paths. */
#define DRV_SQI_MAX_THRESHOLD_VALUE (0x20)

// *****************************************************************************
/* SQI Driver Buffer Handle Macros

  Summary:
    SQI driver Buffer Handle Macros

  Description:
    Buffer handle related utility macros. SQI driver buffer handle is a 
    combination of buffer token and the buffer object index. The buffertoken
    is a 16 bit number that is incremented for every new write or erase request
    and is used along with the buffer object index to generate a new buffer 
    handle for every request.

  Remarks:
    None
*/

#define _DRV_SQI_BUF_TOKEN_MAX         (0xFFFF)
#define _DRV_SQI_MAKE_HANDLE(token, index) ((token) << 16 | (index))
#define _DRV_SQI_UPDATE_BUF_TOKEN(token) \
{ \
    (token)++; \
    (token) = ((token) == _DRV_SQI_BUF_TOKEN_MAX) ? 0: (token); \
}

/* SQI DMA Buffer descriptor registers */
typedef struct _DRV_SQI_DMA_DESC
{
    uint32_t bdCtrl;
    uint32_t bdStat;
    uint32_t *bufAddr;
    struct _DRV_SQI_DMA_Desc *nextAddr;
} _DRV_SQI_DMA_Desc;

/* SQI Driver Buffer Object */
typedef struct _DRV_SQI_BUFFER_OBJECT
{
    /* True if object is allocated */
    bool inUse;

    /* Flag indicating if the operation is still pending. */
    bool opPending;

    /* SQI device for which this buffer object is queued. */
    uint8_t sqiDevice;

    /* Client that owns this buffer */
    DRV_HANDLE hClient;

    /* Current command handle of this buffer object */
    DRV_SQI_COMMAND_HANDLE commandHandle;

    /* Present status of this command */
    DRV_SQI_COMMAND_STATUS status;

    /* Pointer to the transfer request */
    DRV_SQI_TransferElement *xferData;

    /* Number of elements in xferData */
    uint8_t numElements;

    uint32_t length;

    /* Pointer to the next buffer in the queue */
    struct _DRV_SQI_BUFFER_OBJECT *next;

    /* Pointer to the previous buffer in the queue */
    struct _DRV_SQI_BUFFER_OBJECT *previous;

} _DRV_SQI_BUFFER_OBJECT;

/* SQI Client Object */
typedef struct
{
    /* The hardware instance object associate with the client */
    void * driverObj;

    /* Status of the client object */
    SYS_STATUS status;

    /* The intent with which the client was opened */
    DRV_IO_INTENT intent;

    /* Flag to indicate in use */
    bool inUse;

    /* Client specific event handler */
    DRV_SQI_EVENT_HANDLER eventHandler;

    /* Client specific context */
    uintptr_t context;

} DRV_SQI_CLIENT_OBJECT;

/* SQI Driver Hardware Instance Object */
typedef struct
{
    /* The module index associated with the object*/
    SQI_MODULE_ID sqiId;

    /* Object Index */
    SYS_MODULE_INDEX objIndex;

    /* The buffer object queue */
    _DRV_SQI_BUFFER_OBJECT *rwQueue;

    /* The status of the driver */
    SYS_STATUS status;

    /* Flag to indicate in use  */
    bool inUse;

    /* Flag to indicate that the driver is used in exclusive access mode */
    bool isExclusive;

    /* Number of clients connected to the hardware instance */
    uint8_t numClients;

    /* SQI Interrupt Source */
    INT_SOURCE interruptSource;

    /* Current buffer object that is being processed. */
    _DRV_SQI_BUFFER_OBJECT *currentBufObj;

    /* Offset within the current buffer. */
    uint32_t bufferOffset;

    /* Current element of xferData that is being processed. */
    uint8_t currentElement;

    /* Current sqi device that controller is talking to. */
    uint8_t currentSqiDevice;
    
    /* Identifies the enabled devices. */
    DRV_SQI_ENABLE_DEVICE enabledDevices;

    DRV_SQI_CLK_DIV clockDivider;

    /* Device specific configuration information. */
    DRV_SQI_DEVICE_CFG *devCfg;

} DRV_SQI_OBJECT;

#endif //#ifndef _DRV_SQI_LOCAL_H

/*******************************************************************************
 End of File
*/

