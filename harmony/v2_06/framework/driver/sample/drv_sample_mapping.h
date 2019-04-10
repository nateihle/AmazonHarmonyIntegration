/*******************************************************************************
  Sample Device Driver Interface Names Mapping

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sample_mapping.h

  Summary:
    Sample Device Driver Interface names mapping.

  Description:
    This header file maps the interface prototypes in "drv_sample.h" to
    static variants of these routines appropriate for the selected configuration.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012-2015 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _DRV_SAMPLE_MAPPING_H
#define _DRV_SAMPLE_MAPPING_H


// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************
/* See the end of file for implementation header include files.
*/

#include <stdint.h>
#include <stdbool.h>

#include "system_config.h"
#include "sample/drv_sample_static_single.h"
#include "sample/drv_sample_static_multi.h"


// *****************************************************************************
// *****************************************************************************
// Section: Build Parameter Checking
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Macro: DRV_SAMPLE_INSTANCES_NUMBER Check

  Summary:
    Checks the DRV_SAMPLE_INSTANCES_NUMBER definition.

  Description:
    If DRV_SAMPLE_INSTANCES_NUMBER is greater than the number of
    SAMPLE instances available on the part, an error is generated.

  Remarks:
    The _SAMPLE_EXISTS is a processor-specific value defined by the processor
    headers in the PLIB.

    If the configuration does not specify the number of driver instances to
    allocate it defaults to then number of Sample instances on the device.
*/

#if defined(DRV_SAMPLE_INSTANCES_NUMBER)

    #if (DRV_SAMPLE_INSTANCES_NUMBER > _SAMPLE_EXISTS )

        #error "The number of Sample instances configured is more than the available Samples on the device"

    #endif

#endif


// *****************************************************************************
// *****************************************************************************
// Section: SAMPLE Driver Static API Name Generation
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Macro: _DRV_SAMPLE_MAKE_NAME(name)

  Summary:
    Creates an instance-specific static interface name

  Description:
    This macro creates the instance-specific name of the given static interface
    function by inserting the index number into the name.

    For example, DRV_SAMPLE_Initialize becomes DRV_SAMPLE1_Initialize for an index 
	of 1.

  Remarks:
    Multi-client configurations add the word "multi" to the API name, single-client 
	configurations do not.

    For example, DRV_SAMPLE_Initialize becomes DRV_SAMPLE1multi_Initialize for an index
    of 1.
*/


#if !defined(DRV_SAMPLE_INSTANCES_NUMBER)

    // Static builds use static naming to reduce calling overhead.
    #if !defined(DRV_SAMPLE_CLIENTS_NUMBER)

        // Static Single-Client Interface Name Generation
        #define _DRV_SAMPLE_STATIC_API_SINGLE(index, name)     DRV_SAMPLE ## index ## _ ## name
        #define _DRV_SAMPLE_STATIC_API(index, name)            _DRV_SAMPLE_STATIC_API_SINGLE(index, name)

    #else // ( DRV_SAMPLE_CLIENTS_NUMBER >= 1 )

        // Static Multi-Client Interface Name Generation
        #define _DRV_SAMPLE_STATIC_API_MULTI(index, name)      DRV_SAMPLE ## index ## multi_ ## name
        #define _DRV_SAMPLE_STATIC_API(index, name)            _DRV_SAMPLE_STATIC_API_MULTI(index, name)

    #endif

    // Static naming Macros
    #define _DRV_SAMPLE_MAKE_NAME(name)                        _DRV_SAMPLE_STATIC_API(DRV_SAMPLE_INDEX, name)

#else // (DRV_SAMPLE_CONFIG_BUILD_TYPE == DRV_BUILD_TYPE_DYNAMIC)

    // Dynamic Interface Name Generation
    #define _DRV_SAMPLE_MAKE_NAME(name)                         DRV_SAMPLE_ ## name

#endif


// *****************************************************************************
// *****************************************************************************
// Section: Sample Driver Static API Mapping
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Static Interface Mapping

  Summary:
    Maps the dynamic interface calls to appropriate static interface.

  Description:
    These macros map calls to the dynamic interface routines to calls to the 
    appropriate instance-specific static interface routine when a static build
    (DRV_SAMPLE_INSTANCES_NUMBER is not defined) is configured.
    
    For example, DRV_SAMPLE_Status(DRV_SAMPLE_INDEX_1);, becomes DRV_SAMPLE_Status();.
        
  Remarks:
    Static configuration eliminate the need to pass the object parameter.  
    However, the index is "returned" as the object handle (from the 
    "Initialize" routine) to allow code written for the dynamic API to continue
    to build when using a static configuration.
    
    For example, object = DRV_SAMPLE_Initialize(DRV_SAMPLE_INDEX_1, &initData);
	becomes object = ( DRV_SAMPLE1_Initialize(&initData), DRV_SAMPLE_INDEX );
        
    Static single-client configurations also eliminate the client handle 
    parameter.  However, the index (the driver-object index) is "returned" from
    the "Open" routine to allow code written for multi-client drivers to build
    when using a static single-open configuration.
    
    For example, handle = DRV_SAMPLE_Open(DRV_SAMPLE_INDEX_1, intent); becomes
	handle = ( DRV_SAMPLE_Open(intent), DRV_SAMPLE_INDEX );.
*/

#if !defined(DRV_SAMPLE_INSTANCES_NUMBER) // static

    #if !defined(DRV_SAMPLE_CLIENTS_NUMBER) // single client

        #define DRV_SAMPLE_Initialize(sysID, inData)          (_DRV_SAMPLE_MAKE_NAME(Initialize)(inData), DRV_SAMPLE_INDEX)

        #define DRV_SAMPLE_Reinitialize(sysObj, inData)        _DRV_SAMPLE_MAKE_NAME(Reinitialize)(inData)

        #define DRV_SAMPLE_Deinitialize(sysObj)                _DRV_SAMPLE_MAKE_NAME(Deinitialize)()

        #define DRV_SAMPLE_Status(sysObj)                      _DRV_SAMPLE_MAKE_NAME(Status)()

        #define DRV_SAMPLE_Tasks(sysObj)                       _DRV_SAMPLE_MAKE_NAME(Tasks)()

        #define DRV_SAMPLE_Open(sysID, intent)                (_DRV_SAMPLE_MAKE_NAME(Open)(intent), DRV_SAMPLE_INDEX)

        #define DRV_SAMPLE_Close(handle)                       _DRV_SAMPLE_MAKE_NAME(Close)()

        #define DRV_SAMPLE_ClientStatus(handle)                _DRV_SAMPLE_MAKE_NAME(ClientStatus)()

    #else // multi-client

        #define DRV_SAMPLE_Initialize(sysID, inData)          (_DRV_SAMPLE_MAKE_NAME(Initialize)(inData), DRV_SAMPLE_INDEX)

        #define DRV_SAMPLE_Reinitialize(sysObj, inData)        _DRV_SAMPLE_MAKE_NAME(Reinitialize)(inData)

        #define DRV_SAMPLE_Deinitialize(sysObj)                _DRV_SAMPLE_MAKE_NAME(Deinitialize)()

        #define DRV_SAMPLE_Status(sysObj)                      _DRV_SAMPLE_MAKE_NAME(Status)()

        #define DRV_SAMPLE_Tasks(sysObj)                       _DRV_SAMPLE_MAKE_NAME(Tasks)()

        #define DRV_SAMPLE_Open(sysID, intent)                (_DRV_SAMPLE_MAKE_NAME(Open)(intent))

        #define DRV_SAMPLE_Close(handle)                       _DRV_SAMPLE_MAKE_NAME(Close)(handle)

        #define DRV_SAMPLE_ClientStatus(handle)                _DRV_SAMPLE_MAKE_NAME(ClientStatus)(handle)


	#endif

#else // Dynamic Build

    // No Change in the Naming convention

#endif


// *****************************************************************************
// *****************************************************************************
// Section: Sample Driver API Mapping
// *****************************************************************************
// *****************************************************************************

/* This section maps the specific Sample Driver APIs to the appropriate Peripheral
   Library APIs */


#endif // #ifndef _DRV_SAMPLE_MAPPING_H

/*******************************************************************************
 End of File
*/