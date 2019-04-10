/*******************************************************************************
  MXT336T Driver Local Data Structures

  Company:
    Microchip Technology Inc.

  File Name:
    drv_MXT336T_local.h

  Summary:
    MXT336T driver local declarations and definitions.

  Description:
    This file contains the MXT336T driver's local declarations and definitions.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _DRV_MXT336T_LOCAL_H
#define _DRV_MXT336T_LOCAL_H


// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "system_config.h"
#include "system_definitions.h"
#include "driver/touch/mxt336t_legacy/drv_mxt336t.h"
#include "osal/osal.h"


// *****************************************************************************
// *****************************************************************************
// Section: Data Type Definitions
// *****************************************************************************
// *****************************************************************************

// maximum divider value for 32 bit operation mode
//#define     DRV_TIMER_DIVIDER_MAX_32BIT     0xffffffff

typedef struct
{
    uint8_t         family_id;              /* address 0 */
    uint8_t         variant_id;             /* address 1 */
    uint8_t         version;                /* address 2 */
    uint8_t         build;                  /* address 3 */
    uint8_t         matrix_x_size;          /* address 4 */
    uint8_t         matrix_y_size;          /* address 5 */
    uint8_t         num_declared_objects;   /* address 6 */
} __attribute__((packed)) DRV_MXT336T_INFO_ID_T;

typedef struct
{
    uint8_t         object_type;            /* Object type ID */
    uint16_t        i2c_address;            /* Start address of the obj config structure */
    uint8_t         size;                   /* Byte length of the obj config structure -1 */
    uint8_t         instances;              /* Number of objects of this obj. type -1 */
    uint8_t         num_report_ids;         /* e.g. max number of touches in a screen etc */
    
} __attribute__((packed)) DRV_MXT336T_OBJECT_T;


typedef struct
{
    uint16_t        CRC_lo;                 /* low 16 bits of 24 bit value */
    uint8_t         CRC_hi;                 /* high 8 bits of 24 bit value */
} __attribute__((packed)) DRV_MXT336T_CRC_T;


typedef struct
{
    /* Pointer to the struct containing ID information */
    DRV_MXT336T_INFO_ID_T        *id;
    /* Pointer to an array of objects */
    DRV_MXT336T_OBJECT_T         *objects;
    /* Pointer to information block checksum */
    DRV_MXT336T_CRC_T            *crc;
    
}  __attribute__((packed)) DRV_MXT336T_INFO_BLOCK_T;


// *****************************************************************************
/* MXT336T Driver Module Index Count

  Summary:
    Number of valid MXT336T driver indices.

  Description:
    This constant identifies the number of valid MXT336T driver indices.

  Remarks:
    This constant should be used in place of hard-coded numeric literals.

    This value is derived from device-specific header files defined as part of 
    the peripheral libraries.
*/

typedef enum {

    MXT336T_ID_1 = 0,
    MXT336T_NUMBER_OF_MODULES
            
} DRV_MXT336T_MODULE_ID;


typedef enum
{
    /* Driver Initialize state */
    DRV_MXT336T_DEVICE_STATE_INIT = 0,
            
    DRV_MXT336T_DEVICE_STATE_DELAYED_OPEN,
    /* reset the address */
    DRV_MXT336T_DEVICE_STATE_INIT_RESET,        
            
    /* Read information block */
    DRV_MXT336T_DEVICE_STATE_READ_IB,
       
    /* Create object table */
    DRV_MXT336T_DEVICE_STATE_READ_OBJECT_TABLE,
    
    /* Read XRANGE YRANGE Resolution configs*/        
    DRV_MXT336T_DEVICE_STATE_READ_T100_XRANGE,
    
    DRV_MXT336T_DEVICE_STATE_READ_T100_YRANGE,
    /* Driver ready state */
    DRV_MXT336T_DEVICE_STATE_READY,   
            
    /* read a specified object from the device */        
    DRV_MXT336T_DEVICE_STATE_READ_MESSAGE_OBJECT,
            
    /* In error state */        
    DRV_MXT336T_DEVICE_STATE_ERROR, 
            
} DRV_MXT336T_DEVICE_STATE;

// *****************************************************************************
/* MXT336T Touch Controller Driver Task State

  Summary:
    Enumeration defining MXT336T touch controller driver task state.

  Description:
    This enumeration defines the MXT336T touch controller driver task state.
    The task state helps to synchronize the operations of initialization the
    the task, adding the read input task to the task queue once the touch
    controller notifies the available touch input and a decoding the touch input
    received.

  Remarks:
    None.
*/

typedef enum
{
    /* Task initialize state */
    DRV_MXT336T_TASK_STATE_INIT = 0,
            
    /* Establish contact with device */
    DRV_MXT336T_TASK_STATE_CONTACT,
    /* Read information block */
    DRV_MXT336T_TASK_STATE_READ_IB,
    /* Create object table */
    DRV_MXT336T_TASK_STATE_READ_OBJECT,
    /* Configure objects */
    DRV_MXT336T_TASK_STATE_CONFIG_OBJECT,
    /* Read input */
    DRV_MXT336T_TASK_STATE_READ_INPUT,
    /* Decode input */
    DRV_MXT336T_TASK_STATE_DECODE_INPUT,
    /* Task complete state */
    DRV_MXT336T_TASK_STATE_DONE,

} DRV_MXT336T_TASK_STATE;

// *****************************************************************************
/* MXT336T Touch Controller driver task data structure.

  Summary:
    Defines the MXT336T Touch Controller driver task data structure.

  Description:
    This data type defines the data structure maintaining task context in the task
    queue. The inUse flag denotes the task context allocation for a task.
    The enum variable taskState maintains the current task state. The I2C
    buffer handle drvI2CReadBufferHandle maintains the I2C driver buffer handle
    returned by the I2C driver read request. The byte array variable
    drvI2CReadFrameData maintains the I2C frame data sent by MXT336T after a
    successful read request.

  Remarks:
    None.
*/
typedef struct
{
    /* Flag denoting the allocation of task */
    bool                            inUse;

    /* Enum maintaining the task state */
    DRV_MXT336T_TASK_STATE   taskState;

    /* I2C Buffer handle */
    DRV_I2C_BUFFER_HANDLE           drvI2CBufferHandle;

    /* Response to Read Object Command */
    uint8_t                         drvI2CFrameData[DRV_MXT336T_I2C_FRAME_SIZE];

} DRV_MXT336T_TASK_QUEUE;

// *****************************************************************************
/* MXT336T Driver client object

  Summary:
    MXT336T Driver client object maintaining client data.

  Description:
    This defines the object required for the maintenance of the software
    clients instance. This object exists once per client instance.

  Remarks:
    None.
*/

/* resolve forward reference */
struct DRV_MXT336T_DEVICE_OBJECT;

struct DRV_MXT336T_DEVICE_CLIENT_OBJECT
{
    /* Driver Object associated with the touch object */
    struct DRV_MXT336T_DEVICE_OBJECT               *deviceObject;

    /* base report ID for this object */
    uint8_t                                         report_id_base;
    
    /* Callback function for the client */
    DRV_MXT336T_CLIENT_CALLBACK                    clientCallback;
    
    /* context for the callback */
    uintptr_t                                       context;

    /* Next client object on the device that we are monitoring */
    struct DRV_MXT336T_DEVICE_CLIENT_OBJECT        *pNext;
    
    /* Pointer to the MXT336T object data stored in the corresponding device object */
    DRV_MXT336T_OBJECT_T                           *pMXT336TObject;    
};

// *****************************************************************************
/* MXT336T Driver Instance Object.

  Summary:
    Defines the data structure maintaining MXT336T driver instance object.

  Description:
    This data structure maintains the MXT336T driver instance object. The
    object exists once per hardware instance. It is the base object for 
    a MXT336T device.

  Remarks:
    None.
*/

struct DRV_MXT336T_DEVICE_OBJECT
{
    /* The status of the driver */
    SYS_STATUS                      status;
    
    /* Driver state */
    DRV_MXT336T_DEVICE_STATE       deviceState;

    /* The peripheral Id associated with the object */
    int                             touchId;

    /* Save the index of the driver. Important to know this
    as we are using reference based accessing */
    SYS_MODULE_INDEX                drvIndex;

    /* Flag to indicate instance in use  */
    bool                            inUse;

    /* Flag to indicate module used in exclusive access mode */
    bool                            isExclusive;


    /* Callback for I2C Driver Open call */
    DRV_HANDLE                      (*drvOpen) ( const SYS_MODULE_INDEX index,
                                                const DRV_IO_INTENT intent );

    /* Touch input interrupt source */
    INT_SOURCE                      interruptSource;
    /* interrupt pin for driver instance */
    PORTS_BIT_POS                   interruptPin;
    
    /* port channel for interrupt instance */
    PORTS_CHANNEL                   interruptChannel;
       
    /* reset pin for driver instance */
    PORTS_BIT_POS                   resetPin;
    
    /* port channel for reset pin */
    PORTS_CHANNEL                   resetChannel;

    /* Head of the task queue */
    DRV_MXT336T_TASK_QUEUE*        taskQueue;

    /* I2C bus driver handle */
    DRV_HANDLE                      drvI2CHandle;
    
    /* To save memory the MXT336T driver only keeps a copy of the device ID BLOB and object table*/
    uint8_t*                        pObjectTable;
    
    /* Internal instance of the general message processor client object */
    struct DRV_MXT336T_DEVICE_CLIENT_OBJECT      messageClientObject;
    
        /* Internal instance of the general message processor client object */
    struct DRV_MXT336T_DEVICE_CLIENT_OBJECT      clientObject;
    
    SYS_TMR_HANDLE                               sysTmrMXT336T;
};


// *****************************************************************************
/* Find a specific MXT336T client object in the device object table 
 * Internally used by the driver to find the MXT336T object data  */
 DRV_MXT336T_OBJECT_T* DRV_MXT336T_DEVICE_ClientObjectFind(const struct DRV_MXT336T_DEVICE_OBJECT *pDrvObject,
            const uint8_t objType, const uint8_t objInstance);

// *****************************************************************************
/* Find the report ID base number for a given MXT336T object, this is calculated
 * by summing all of the device instances and number of reports up to the specified
 * object in the table */
 uint8_t DRV_MXT336T_DEVICE_ClientObjectFindReportIDBase(const struct DRV_MXT336T_DEVICE_OBJECT *pDrvObject,
            const uint8_t objType, const uint8_t objInstance);


// *****************************************************************************
/* Send request to read message processor object */
static void _DRV_MXT336T_DEVICE_MessageObjectRead(struct DRV_MXT336T_DEVICE_OBJECT *pDrvObject);

/* Send request to read message processor object */
static void _DRV_MXT336T_DEVICE_RegRead(struct DRV_MXT336T_DEVICE_OBJECT *pDrvObject, DRV_MXT336T_OBJECT_T  *pMXT336TObject, uint8_t reg);

#endif //#ifndef _DRV_MXT336T_LOCAL_H

/*******************************************************************************
 End of File
*/

