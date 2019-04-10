/*******************************************************************************
  I2C Device Driver interface names mapping

  Company:
    Microchip Technology Inc.

  File Name:
    drv_I2C_mapping.h

  Summary:
    I2C Device Driver Interface names mapping.

  Description:
    This header file contains the data type definitions.
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

#ifndef _DRV_CODEC_I2C_MAPPING_H
#define _DRV_CODEC_I2C_MAPPING_H


// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************
/* Note:  See the bottom of file for implementation header include files.
*/

#include <stdint.h>
#include <stdbool.h>

#include "system_config.h"


// *****************************************************************************
// *****************************************************************************
// Section: Build Parameter Checking
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Macro: DRV_I2C_INSTANCES_NUMBER Check

  Summary:
    Checks the DRV_I2C_INSTANCES_NUMBER definition.

  Description:
    If DRV_I2C_INSTANCES_NUMBER is greater than the number of
    I2C instances available on the part, an error is generated.

  Remarks:
    The _I2C_EXISTS is a processor-specific value defined by the processor
    headers in the peripheral library.
    
    If the configuration does not specify the number of driver instances to 
    allocate it defaults to then number of I2C instances on the device.
*/

#if defined(DRV_I2C_INSTANCES_NUMBER)

    #if (DRV_I2C_DRIVER_OBJECTS_NUMBER > _I2C_EXISTS )

        #error "The number of I2C instances configured is more than the available I2Cs on the part"

    #endif

#endif


// *****************************************************************************
// *****************************************************************************
// Section: I2C Driver Static API Name Generation
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Macro: _DRV_I2C_MAKE_NAME(name)

  Summary:
    Creates an instance-specific static interface name.

  Description:
     This macro creates the instance-specific name of the given static interface
     routine by inserting the index number into the name.
     
     Example: DRV_I2C_Initialize becomes DRV_I2C1_Initialize for an index of 1.
     
  Remarks:
    Multi-client configurations add the word "multi" to the API name, single-
    client configurations do not.
    
    Example: DRV_I2C_Initialize becomes DRV_I2C1multi_Initialize for an index
    of 1.
*/


#if !defined(DRV_I2C_INSTANCES_NUMBER)

    // Static builds use static naming to reduce calling overhead. 
    #if !defined(DRV_I2C_CLIENTS_NUMBER)
    
        // Static Single-Client Interface Name Generation
        #define _DRV_I2C_STATIC_API_SINGLE(index, name)     DRV_I2C ## index ## _ ## name
        #define _DRV_I2C_STATIC_API(index, name)            _DRV_I2C_STATIC_API_SINGLE(index, name)

    #else // ( DRV_I2C_CLIENTS_NUMBER >= 1 )

        // Static Multi-Client Interface Name Generation
        #define _DRV_I2C_STATIC_API_MULTI(index, name)      DRV_I2C ## index ## multi_ ## name
        #define _DRV_I2C_STATIC_API(index, name)            _DRV_I2C_STATIC_API_MULTI(index, name)

    #endif

    // Static naming Macros
    #define _DRV_I2C_MAKE_NAME(sysID, name)                        _DRV_I2C_STATIC_API(sysID, name)
        
#else // (DRV_I2C_CONFIG_BUILD_TYPE == DRV_BUILD_TYPE_DYNAMIC)

        // Dynamic Interface Name Generation
        #define _DRV_I2C_MAKE_NAME(name)  DRV_I2C_ ## name

#endif


// *****************************************************************************
// *****************************************************************************
// Section: I2C Driver Static API Mapping
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Static Interface Mapping

  Summary:
    Maps the dynamic interface calls to appropriate static interface.

  Description:
    These macros map calls to the dynamic interface routines to calls to the 
    appropriate instance-specific static interface routine when a static build
    (DRV_I2C_INSTANCES_NUMBER is not defined) is configured.
    
    Example:
    
        DRV_I2C_Status(DRV_I2C_INDEX_1);
        
    Becomes:
    
        DRV_I2C_Status();
        
  Remarks:
    Static configuration eliminate the need to pass the object parameter.  
    However, the index is "returned" as the object handle (from the 
    "Initialize" routine) to allow code written for the dynamic API to continue
    to build when using a static configuration.
    
    Example:
    
        object = DRV_I2C_Initialize(DRV_I2C_INDEX_1, &initData);
        
    Becomes:
    
        object = ( DRV_I2C1_Initialize(&initData), DRV_I2C_INDEX );
        
    Static single-client configurations also eliminate the client handle 
    parameter.  However, the index (the driver-object index) is "returned" from
    the "Open" routine to allow code written for multi-client drivers to build
    when using a static single-open configuration.
    
    Example:
    
        handle = DRV_I2C_Open(DRV_I2C_INDEX_1, intent);
        
    Becomes:
    
        handle = ( DRV_I2C_Open(intent), DRV_I2C_INDEX );
*/

#if !defined( DRV_I2C_INSTANCES_NUMBER )   // Static configuration
    
    /*#if !defined( DRV_I2C_CLIENTS_NUMBER )    // single-Client */
	
	#define DRV_I2C_Initialize(sysID )                  (_DRV_I2C_MAKE_NAME(sysID, Initialize)())
    
        #define DRV_I2C_ByteRead(sysID)                 	       (_DRV_I2C_MAKE_NAME(sysID, ByteRead)())

        #define DRV_I2C_ByteWrite(sysID, inData)                   (_DRV_I2C_MAKE_NAME(sysID, ByteWrite)(inData))

        #define DRV_I2C_TransmitAcknowledged(sysID)                (_DRV_I2C_MAKE_NAME(sysID, TransmitAcknowledged)())

        #define DRV_I2C_ReceiverBufferIsEmpty(sysID)               (_DRV_I2C_MAKE_NAME(sysID, ReceiverBufferIsEmpty)())

        #define DRV_I2C_BaudRateSet(sysID, inData)                 (_DRV_I2C_MAKE_NAME(sysID, BaudRateSet)(inData))

        #define DRV_I2C_MasterStart(sysID)                         (_DRV_I2C_MAKE_NAME(sysID, MasterStart)())

        #define DRV_I2C_MasterReStart(sysID)                       (_DRV_I2C_MAKE_NAME(sysID, MasterReStart)())

        #define DRV_I2C_MasterStop(sysID)                          (_DRV_I2C_MAKE_NAME(sysID, MasterStop)())

        #define DRV_I2C_MasterAcknowledge(sysID)                   (_DRV_I2C_MAKE_NAME(sysID, MasterAcknowledge)())

        #define DRV_I2C_MasterNotAcknowledge(sysID)                (_DRV_I2C_MAKE_NAME(sysID, MasterNotAcknowledge)())

        #define DRV_I2C_DeInitialize(sysID)                        (_DRV_I2C_MAKE_NAME(sysID, DeInitialize)())

        #define DRV_I2C_WaitForStartComplete(sysID)                (_DRV_I2C_MAKE_NAME(sysID, WaitForStartComplete)())

        #define DRV_I2C_WaitForByteWriteToComplete(sysID)          (_DRV_I2C_MAKE_NAME(sysID, WaitForByteWriteToComplete)())

        #define DRV_I2C_WriteByteAcknowledged(sysID)          (_DRV_I2C_MAKE_NAME(sysID, WriteByteAcknowledged)())

        #define DRV_I2C_WaitForStopComplete(sysID)          (_DRV_I2C_MAKE_NAME(sysID, WaitForStopComplete)())

        #define DRV_I2C_WaitForACKOrNACKComplete(sysID)          (_DRV_I2C_MAKE_NAME(sysID, WaitForACKOrNACKComplete)())

       
		#define DRV_I2C_MasterBusIdle(sysID)    (_DRV_I2C_MAKE_NAME(sysID, MasterBusIdle)())

    //#endif

#endif


// *****************************************************************************
// *****************************************************************************
// Section: I2C Driver API Mapping
// *****************************************************************************
// *****************************************************************************
/* This section maps the max and min values of the I2C available on the part
   to the appropriate interface of the PLIB */



#endif // _DRV_I2C_MAPPING_H

/*******************************************************************************
 End of File
*/
