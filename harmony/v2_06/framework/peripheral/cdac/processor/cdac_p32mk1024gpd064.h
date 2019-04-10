/* Created by plibgen $Revision: 1.31 $ */

#ifndef _CDAC_P32MK1024GPD064_H
#define _CDAC_P32MK1024GPD064_H

/* Section 1 - Enumerate instances, define constants, VREGs */

#include <xc.h>
#include <stdbool.h>

#include "peripheral/peripheral_common_32bit.h"

/* Default definition used for all API dispatch functions */
#ifndef PLIB_INLINE_API
    #define PLIB_INLINE_API extern inline
#endif

/* Default definition used for all other functions */
#ifndef PLIB_INLINE
    #define PLIB_INLINE extern inline
#endif

typedef enum {

    CDAC_ID_1 = _CDAC1_BASE_ADDRESS,
    CDAC_ID_2 = _CDAC2_BASE_ADDRESS,
    CDAC_ID_3 = _CDAC3_BASE_ADDRESS,
    CDAC_NUMBER_OF_MODULES = 3

} CDAC_MODULE_ID;

typedef enum {

    CDAC_OUTPUT1 = 1

} CDAC_OUTPUT_SELECTION;

typedef enum {

    CDAC_DISABLE = 0,
    CDAC_ENABLE = 1

} CDAC_MODULE_ENABLE;

typedef enum {

    CDAC_OUT_DISABLE = 0,
    CDAC_OUT_ENABLE = 1

} CDAC_OUTPUT_ENABLE;

typedef enum {

    CDAC_VREF_AVDD = 3

} CDAC_REF_SEL;

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE
#include "../templates/cdac_EnableControl_Default.h"
#include "../templates/cdac_OutputControl_Default.h"
#include "../templates/cdac_ReferenceSelect_Default.h"
#include "../templates/cdac_DataControl_Default.h"

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API void PLIB_CDAC_Enable(CDAC_MODULE_ID index)
{
     CDAC_Enable_Default(index);
}

PLIB_INLINE_API void PLIB_CDAC_Disable(CDAC_MODULE_ID index)
{
     CDAC_Disable_Default(index);
}

PLIB_INLINE_API bool PLIB_CDAC_ExistsEnableControl(CDAC_MODULE_ID index)
{
     return CDAC_ExistsEnableControl_Default(index);
}

PLIB_INLINE_API void PLIB_CDAC_OutputEnable(CDAC_MODULE_ID index, CDAC_OUTPUT_SELECTION output)
{
     CDAC_OutputEnable_Default(index, output);
}

PLIB_INLINE_API void PLIB_CDAC_OutputDisable(CDAC_MODULE_ID index, CDAC_OUTPUT_SELECTION output)
{
     CDAC_OutputDisable_Default(index, output);
}

PLIB_INLINE_API bool PLIB_CDAC_ExistsOutputControl(CDAC_MODULE_ID index)
{
     return CDAC_ExistsOutputControl_Default(index);
}

PLIB_INLINE_API void PLIB_CDAC_ReferenceVoltageSelect(CDAC_MODULE_ID index, CDAC_REF_SEL refSel)
{
     CDAC_ReferenceVoltageSelect_Default(index, refSel);
}

PLIB_INLINE_API bool PLIB_CDAC_ExistsReferenceVoltageSelect(CDAC_MODULE_ID index)
{
     return CDAC_ExistsReferenceVoltageSelect_Default(index);
}

PLIB_INLINE_API uint16_t PLIB_CDAC_DataRead(CDAC_MODULE_ID index)
{
     return CDAC_DataRead_Default(index);
}

PLIB_INLINE_API void PLIB_CDAC_DataWrite(CDAC_MODULE_ID index, uint16_t cdacData)
{
     CDAC_DataWrite_Default(index, cdacData);
}

PLIB_INLINE_API bool PLIB_CDAC_ExistsDataControl(CDAC_MODULE_ID index)
{
     return CDAC_ExistsDataControl_Default(index);
}

#endif
