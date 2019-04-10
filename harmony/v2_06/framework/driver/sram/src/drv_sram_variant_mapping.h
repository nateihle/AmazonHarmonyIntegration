/*******************************************************************************
  SRAM Driver Feature Variant Implementations

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sram_variant_mapping.h

  Summary:
    SRAM Driver Feature Variant Implementations

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


#ifndef _DRV_SRAM_VARIANT_MAPPING_H
#define _DRV_SRAM_VARIANT_MAPPING_H

// *****************************************************************************
// *****************************************************************************
// Section: Feature Variant Mapping
// *****************************************************************************
// *****************************************************************************
/* Some variants are determined by hardware feature existence, some features
   are determined user configuration of the driver, and some variants are
   combination of the two.
*/

#if defined __PIC32MX__
    #define DRV_SRAM_INVALIDATE_CACHE(address, bytes)
#elif defined __PIC32MZ__
    #define DRV_SRAM_INVALIDATE_CACHE(address, bytes) SYS_DEVCON_DataCacheInvalidate((address), (bytes))
#elif defined __PIC32WK__
    #define DRV_SRAM_INVALIDATE_CACHE(address, bytes)
#else
    #define DRV_SRAM_INVALIDATE_CACHE(address, bytes)
#endif

#if defined (DRV_SRAM_DISABLE_ERROR_CHECK)
#define _DRV_SRAM_VALIDATE_EXPR(y, z)
#define _DRV_SRAM_VALIDATE_EXPR_VOID(y)
#else
#define _DRV_SRAM_VALIDATE_EXPR(y, z) if ((y)) return (z);
#define _DRV_SRAM_VALIDATE_EXPR_VOID(y) if ((y)) return;
#endif

#endif //_DRV_SRAM_VARIANT_MAPPING_H

/*******************************************************************************
 End of File
*/


