/*******************************************************************************
  MTCH6303 Driver Interface Declarations for Static Single Instance Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_mtch6303_static_local.h

  Summary:
    MTCH6303 Driver Local Data Structures for static implementation.

  Description:
    Driver Local Data Structures for static implementation
    
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

#ifndef _DRV_TOUCH_MTCH6303_STATIC_LOCAL_H
#define _DRV_TOUCH_MTCH6303_STATIC_LOCAL_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "driver/touch/mtch6303/drv_mtch6303_static.h"
#include "osal/osal.h"
#include "system/int/sys_int.h"
#include "system/debug/sys_debug.h"

// *****************************************************************************
// *****************************************************************************
// Section: Data Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Driver Touch Controller MTCH6303 I2C Device Address

  Summary:


  Description:


  Remarks:

*/
        
#define DRV_TOUCH_MTCH6303_I2C_DEVICE_ADDRESS        0x4A

// *****************************************************************************
/* 

  Summary:
    

  Description:
    

  Remarks:
    

*/

#define DRV_TOUCH_MTCH6303_MAX_POSITION_OUTPUT       0x7FFF

// *****************************************************************************
/* 

  Summary:
    

  Description:
    

  Remarks:
    

*/
typedef enum
{
    /* */
    DRV_TOUCH_MTCH6303_BUFFER_OBJ_FLAG_REG_READ = (1 << 0),

    /* */
    DRV_TOUCH_MTCH6303_BUFFER_OBJ_FLAG_REG_WRITE = (1 << 1),

} DRV_TOUCH_MTCH6303_BUFFER_OBJ_FLAG;

// *****************************************************************************
/* 

  Summary:
    

  Description:
    

  Remarks:
    

*/
typedef enum
{
    /* */
    DRV_TOUCH_MTCH6303_TOUCH_BUFFER_FLAG_STATUS_READ    = (1 << 0),

    /* */
    DRV_TOUCH_MTCH6303_TOUCH_BUFFER_FLAG_INPUT_READ     = (1 << 1),

    /* */
    DRV_TOUCH_MTCH6303_TOUCH_BUFFER_FLAG_MSG_TXRDY_READ = (1 << 2),

    /* */
    DRV_TOUCH_MTCH6303_TOUCH_BUFFER_FLAG_MSG_READ       = (1 << 3),

    /* */
    DRV_TOUCH_MTCH6303_TOUCH_BUFFER_FLAG_MSG_RXRDY_READ = (1 << 4),

    /* */
    DRV_TOUCH_MTCH6303_TOUCH_BUFFER_FLAG_MSG_WRITE      = (1 << 5),

} DRV_TOUCH_MTCH6303_TOUCH_BUFFER_OBJ_FLAG;

// *****************************************************************************
/* Driver Touch Controller MTCH6303 number of touch inputs supported

  Summary:


  Description:


  Remarks:

*/
        
typedef enum
{
    /* */
    DRV_TOUCH_MTCH6303_REG_TOUCH_STATUS /* DOM-IGNORE-BEGIN */ = 0x00, /* DOM-IGNORE-END */
            
    /* */
    DRV_TOUCH_MTCH6303_REG_TOUCH_0_NIBBLE_0 /* DOM-IGNORE-BEGIN */ = 0x01, /* DOM-IGNORE-END */
            
    /* */
    DRV_TOUCH_MTCH6303_REG_TOUCH_0_ID /* DOM-IGNORE-BEGIN */ = 0x02, /* DOM-IGNORE-END */

    /*  */            
    DRV_TOUCH_MTCH6303_REG_TOUCH_0_X1_LSB /* DOM-IGNORE-BEGIN */ = 0x03, /* DOM-IGNORE-END */
            
    /* */
    DRV_TOUCH_MTCH6303_REG_TOUCH_0_X1_MSB /* DOM-IGNORE-BEGIN */ = 0x04, /* DOM-IGNORE-END */
    
    /* */
    DRV_TOUCH_MTCH6303_REG_TOUCH_0_Y1_LSB /* DOM-IGNORE-BEGIN */ = 0x05, /* DOM-IGNORE-END */

    /*  */            
    DRV_TOUCH_MTCH6303_REG_TOUCH_0_Y1_MSB /* DOM-IGNORE-BEGIN */ = 0x06, /* DOM-IGNORE-END */

    /* */
    DRV_TOUCH_MTCH6303_REG_TOUCH_9_NIBBLE_0 /* DOM-IGNORE-BEGIN */ = 0x37, /* DOM-IGNORE-END */
            
    /* */
    DRV_TOUCH_MTCH6303_REG_TOUCH_9_ID /* DOM-IGNORE-BEGIN */ = 0x38, /* DOM-IGNORE-END */

    /*  */            
    DRV_TOUCH_MTCH6303_REG_TOUCH_9_X1_LSB /* DOM-IGNORE-BEGIN */ = 0x39, /* DOM-IGNORE-END */
            
    /* */
    DRV_TOUCH_MTCH6303_REG_TOUCH_9_X1_MSB /* DOM-IGNORE-BEGIN */ = 0x3A, /* DOM-IGNORE-END */
    
    /* */
    DRV_TOUCH_MTCH6303_REG_TOUCH_9_Y1_LSB /* DOM-IGNORE-BEGIN */ = 0x3B, /* DOM-IGNORE-END */

    /*  */            
    DRV_TOUCH_MTCH6303_REG_TOUCH_9_Y1_MSB /* DOM-IGNORE-BEGIN */ = 0x3C, /* DOM-IGNORE-END */

    /* */
    DRV_TOUCH_MTCH6303_REG_TOUCH_MAX      /* DOM-IGNORE-BEGIN */ = 0x3D, /* DOM-IGNORE-END */
            
    /*  */
    DRV_TOUCH_MTCH6303_REG_RX_BYTES_READY /* DOM-IGNORE-BEGIN */ = 0xFB, /* DOM-IGNORE-END */
            
    /* */
    DRV_TOUCH_MTCH6303_REG_RX_BUFFER_POINTER /* DOM-IGNORE-BEGIN */ = 0xFC, /* DOM-IGNORE-END */
            
    /*  */
    DRV_TOUCH_MTCH6303_REG_TX_BYTES_READY /* DOM-IGNORE-BEGIN */ = 0xFD, /* DOM-IGNORE-END */
            
    /* */
    DRV_TOUCH_MTCH6303_REG_TX_BUFFER_POINTER /* DOM-IGNORE-BEGIN */ = 0xFE, /* DOM-IGNORE-END */
            
} DRV_TOUCH_MTCH6303_I2C_REGISTER_MAP;

// *****************************************************************************
/* 

  Summary:
    

  Description:
    

  Remarks:
    

*/

typedef struct _DRV_TOUCH_MTCH6303_BUFFER_OBJ
{
    /* */
    bool                             inUse;

    /* */
    size_t                           size;

    /* */
    size_t                           nCurrentBytes;

    /* */
    uint8_t                          writeBuffer[68];

    /* */
    uint8_t                        * readBuffer;

    /* */
    uint8_t                          regAddress;

    /* */
    uint32_t                         flags;

<#if CONFIG_DRV_TOUCH_MTCH6303_BUS_SELECT?has_content>
<#if CONFIG_DRV_TOUCH_MTCH6303_BUS_SELECT = "DRV_TOUCH_MTCH6303_BUS_I2C">
    /* */   
    DRV_I2C_BUFFER_HANDLE            hBusBuffer;
</#if>
</#if>

    /* */
    struct _DRV_TOUCH_MTCH6303_BUFFER_OBJ *next;

    /* */
    struct _DRV_TOUCH_MTCH6303_BUFFER_OBJ *previous;

} DRV_TOUCH_MTCH6303_BUFFER_OBJ;

// *****************************************************************************
/* 

  Summary:
    

  Description:
    

  Remarks:
    

*/

typedef struct _DRV_TOUCH_MTCH6303_TOUCH_BUFFER_OBJ
{
    /* */
    bool                                   inUse;

    /* */
    size_t                                 size;

    /* */
    size_t                                 nCurrentBytes;

    /* */
    uint32_t                               flags;

    /* */   
    DRV_TOUCH_MTCH6303_TOUCH_BUFFER_HANDLE       hRegBuffer;

    /* */
    DRV_TOUCH_MTCH6303_TOUCH_DATA            *   touchData;
    
    /* */
    DRV_TOUCH_MTCH6303_TOUCH_MESSAGE         *   message;

    /* */
    uint8_t                                numHWBytes;

    /* */
    struct _DRV_TOUCH_MTCH6303_TOUCH_BUFFER_OBJ *next;

    /* */
    struct _DRV_TOUCH_MTCH6303_TOUCH_BUFFER_OBJ *previous;

} DRV_TOUCH_MTCH6303_TOUCH_BUFFER_OBJ;

// *****************************************************************************
/* 

  Summary:
    

  Description:
    

  Remarks:
    

*/

typedef struct
{
    /* */
    bool                              inUse;

    /* */
    SYS_STATUS                        status;

    /* */
    uint32_t                          queueSizeCurrent;

    /* */
    uint32_t                          touchQueueSizeCurrent;

    /* */
    uint32_t                          queueSize;

    /* */
    uint32_t                          touchQueueSize;

    /* */
    DRV_TOUCH_MTCH6303_BUFFER_OBJ           *queue;

    /* */
    DRV_TOUCH_MTCH6303_TOUCH_BUFFER_OBJ     *touchQueue;

    /* */
    bool                              clientInUse;

    /* */
    DRV_TOUCH_MTCH6303_TOUCH_EVENT_HANDLER touchEventHandler;

    /* */
    uintptr_t                         context;

    /* */
    uintptr_t                         touchContext;

    /* */
    DRV_TOUCH_MTCH6303_ERROR                error;

    /* */
    DRV_TOUCH_MTCH6303_CLIENT_STATUS        clientStatus;

    /* */
    DRV_TOUCH_MTCH6303_BUFFER_EVENT         event;

    /* */
    DRV_HANDLE                        drvBusHandle;

    /* */
    uint8_t                           deviceAddress;

    DRV_TOUCH_MTCH6303_BUFFER_HANDLE        bufferHandle;

    DRV_TOUCH_MTCH6303_TOUCH_DATA           touchData;

    /* Most recent valid touch position x-Coordinate */
    int16_t                           mostRecentTouchX;
    
    /* Most recent valid touch position y-Coordinate */
    int16_t                           mostRecentTouchY;

    DRV_TOUCH_POSITION_STATUS         touchStatus;
    
} DRV_TOUCH_MTCH6303_STATIC_OBJ;


bool _DRV_TOUCH_MTCH6303_ClientBufferQueueObjectsRemove( void );
void _DRV_TOUCH_MTCH6303_BufferQueueTasks( DRV_TOUCH_MTCH6303_STATIC_OBJ * hDriver );
void _DRV_TOUCH_MTCH6303_TouchBufferQueueTasks(DRV_TOUCH_MTCH6303_STATIC_OBJ * hDriver);
void _DRV_TOUCH_MTCH6303_I2C_EventHandler( DRV_I2C_BUFFER_EVENT event,
                                     DRV_I2C_BUFFER_HANDLE  bufferHandle, 
                                     uintptr_t contextHandle );
void _DRV_TOUCH_MTCH6303_Buffer_EventHandler( DRV_TOUCH_MTCH6303_BUFFER_EVENT  event,
                                        DRV_TOUCH_MTCH6303_BUFFER_HANDLE bufferHandle, 
                                        uintptr_t contextHandle );

#endif

/*******************************************************************************
 End of File
*/