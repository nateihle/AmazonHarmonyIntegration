/*******************************************************************************
  SAMPLE Driver Feature Variant Implementations

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sample_variant_mapping.h

  Summary:
    SAMPLE Driver Feature Variant Implementations

  Description:
    This file implements the functions which differ based on different parts
    and various implementations of the same feature.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

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


#ifndef _DRV_SAMPLE_VARIANT_MAPPING_H
#define _DRV_SAMPLE_VARIANT_MAPPING_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#if !defined(DRV_SAMPLE_INSTANCES_NUMBER)

    #if defined(DRV_SAMPLE_CLIENTS_NUMBER)
    
        /* Map internal macros and functions to the static multi open variant */
        #include "sample/src/static/drv_sample_hw_static.h"
        #include "sample/src/client_single/drv_sample_client_multi.h"
    
    #else
    
        /* Map internal macros and functions to the static single open variant */
        #include "sample/src/static/drv_sample_hw_static.h"
        #include "sample/src/client_single/drv_sample_client_single.h"

    #endif
    
#else // (DRV_SAMPLE_INSTANCES_NUMBER > 1)

    /* Map internal macros and functions to the dynamic variant */
    #include "sample/src/dynamic/drv_sample_hw_dynamic.h"
    #include "sample/src/client_multi/drv_sample_client_multi.h"

#endif


// *****************************************************************************
// *****************************************************************************
// Section: Feature Variant Mapping
// *****************************************************************************
// *****************************************************************************
/* Some variants are determined by hardware feature existence, some features
   are determined user configuration of the driver, and some variants are
   combination of the two.
*/

// *****************************************************************************
/* PLIB ID Static Configuration Override

  Summary:
    Allows static override of the peripehral library ID

  Description:
    These macros allow the peripheral library ID to be statically overriden by 
    the DRV_SAMPLE_PERIPHERAL_ID configuration macro, if it is defined.
    
    _DRV_SAMPLE_PERIPHERAL_ID_GET replaces the value passed in with the value 
    defined by the DRV_SAMPLE_PERIPHERAL_ID configuration option.
    
    _DRV_SAMPLE_STATIC_PLIB_ID removes any statement passed into it from the 
    build if the DRV_SAMPLE_PERIPHERAL_ID configuration option is defined.
*/

#if defined(DRV_SAMPLE_PERIPHERAL_ID)

    #define _DRV_SAMPLE_PERIPHERAL_ID_GET(plibId)      DRV_SAMPLE_PERIPHERAL_ID
    #define _DRV_SAMPLE_STATIC_PLIB_ID(any)

#else

    #define _DRV_SAMPLE_PERIPHERAL_ID_GET(plibId)      plibId
    #define _DRV_SAMPLE_STATIC_PLIB_ID(any)            any

#endif


// *****************************************************************************
/* Interrupt Source Static Configuration Override

  Summary:
    Allows static override of the interrupt source

  Description:
    These macros allow the interrupt source to be statically overriden by the 
    DRV_SAMPLE_INTERRUPT_SOURCE configuration macro, if it is defined.
    
    _DRV_SAMPLE_INT_SRC_GET replaces the value passed in with the value defined 
    by the DRV_SAMPLE_INTERRUPT_SOURCE configuration option.
    
    _DRV_SAMPLE_STATIC_INT_SRC removes any statement passed into it from the 
    build if the DRV_SAMPLE_INTERRUPT_SOURCE configuration option is defined.
*/

#if defined(DRV_SAMPLE_INTERRUPT_SOURCE)

    #define _DRV_SAMPLE_INT_SRC_GET(source)    DRV_SAMPLE_INTERRUPT_SOURCE
    #define _DRV_SAMPLE_STATIC_INT_SRC(any)

#else

    #define _DRV_SAMPLE_INT_SRC_GET(source)    source
    #define _DRV_SAMPLE_STATIC_INT_SRC(any)    any

#endif


// *****************************************************************************
/* Interrupt Source Control

  Summary:
    Macros to enable, disable or clear the interrupt source

  Description:
    This macro enables, disables or clears the interrupt source

    The macros get mapped to the respective SYS module APIs if the configuration
    option DRV_SAMPLE_INTERRUPT_MODE is set to true
 
  Remarks:
    This macro is mandatory
*/

#if defined (DRV_SAMPLE_INTERRUPT_MODE) && (DRV_SAMPLE_INTERRUPT_MODE == true)

    #define _DRV_SAMPLE_InterruptSourceEnable(source)          SYS_INT_SourceEnable( source )
    #define _DRV_SAMPLE_InterruptSourceDisable(source)         SYS_INT_SourceDisable( source )
    #define _DRV_SAMPLE_InterruptSourceClear(source)           SYS_INT_SourceStatusClear( source )

    #define _DRV_SAMPLE_InterruptSourceStatusGet(source)       SYS_INT_SourceStatusGet( source )

#elif defined (DRV_SAMPLE_INTERRUPT_MODE) && (DRV_SAMPLE_INTERRUPT_MODE == false)

    #define _DRV_SAMPLE_InterruptSourceEnable(source)
    #define _DRV_SAMPLE_InterruptSourceDisable(source)
    #define _DRV_SAMPLE_InterruptSourceClear(source)           SYS_INT_SourceStatusClear( source )

    #define _DRV_SAMPLE_InterruptSourceStatusGet(source)       SYS_INT_SourceStatusGet( source )

#else

    #error "No Task mode chosen at build, interrupt or polling needs to be selected. "

#endif


#endif //_DRV_SAMPLE_VARIANT_MAPPING_H

/*******************************************************************************
 End of File
*/

