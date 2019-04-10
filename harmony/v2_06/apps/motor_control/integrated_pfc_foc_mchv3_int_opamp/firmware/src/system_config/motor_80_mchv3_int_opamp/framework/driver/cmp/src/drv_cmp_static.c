/*******************************************************************************
  CMP Driver Functions for Static Single Instance Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_cmp_static.c

  Summary:
    CMP driver implementation for the static single instance driver.

  Description:
    The CMP device driver provides a simple interface to manage the CMP
    modules on Microchip microcontrollers.
    
  Remarks:
    Static interfaces incorporate the driver instance number within the names
    of the routines, eliminating the need for an object ID or object handle.
    Static single-open interfaces also eliminate the need for the open handle.
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

// *****************************************************************************
// *****************************************************************************
// Header Includes
// *****************************************************************************
// *****************************************************************************
#include "framework/driver/cmp/drv_cmp_static.h"


// *****************************************************************************
// *****************************************************************************
// Section: CMP static driver functions
// *****************************************************************************
// *****************************************************************************
void DRV_CMP_Initialize(void)
{


// *****************************************************************************
/* Comparator driver instance 0 */
    PLIB_CMP_NonInvertingInputChannelSelect(CMP_ID_3, CMP_NON_INVERTING_INPUT_CDAC);
    PLIB_CMP_InvertingInputChannelSelect(CMP_ID_3, CMP_INVERTING_INPUT_4);

    PLIB_CMP_OutputInvertDisable(CMP_ID_3);
    PLIB_CMP_OutputEnable(CMP_ID_3);
 
    /* Setup comparator Output digital filter */
    PLIB_CMP_ComparatorOutputDigitalFilterDisable(CMP_ID_3);
 
    /* Setup Opamp mode */
    PLIB_CMP_OpAmpEnable(CMP_ID_3);
    PLIB_CMP_OpAmpOutputDisable(CMP_ID_3);

    /* Enable Comparator */
    PLIB_CMP_Enable(CMP_ID_3);


// *****************************************************************************
/* Comparator driver instance 1 */
    PLIB_CMP_NonInvertingInputChannelSelect(CMP_ID_2, CMP_NON_INVERTING_INPUT_1);
    PLIB_CMP_InvertingInputChannelSelect(CMP_ID_2, CMP_INVERTING_INPUT_1);

    PLIB_CMP_OutputInvertDisable(CMP_ID_2);
    PLIB_CMP_OutputDisable(CMP_ID_2);
 
    /* Setup comparator Output digital filter */
    PLIB_CMP_ComparatorOutputDigitalFilterDisable(CMP_ID_2);
 
    /* Setup Opamp mode */
    PLIB_CMP_OpAmpEnable(CMP_ID_2);
    PLIB_CMP_OpAmpOutputDisable(CMP_ID_2);

    /* Enable Comparator */
    PLIB_CMP_Enable(CMP_ID_2);


// *****************************************************************************
/* Comparator driver instance 2 */
    PLIB_CMP_NonInvertingInputChannelSelect(CMP_ID_1, CMP_NON_INVERTING_INPUT_1);
    PLIB_CMP_InvertingInputChannelSelect(CMP_ID_1, CMP_INVERTING_INPUT_1);

    PLIB_CMP_OutputInvertDisable(CMP_ID_1);
    PLIB_CMP_OutputDisable(CMP_ID_1);
 
    /* Setup comparator Output digital filter */
    PLIB_CMP_ComparatorOutputDigitalFilterDisable(CMP_ID_1);
 
    /* Setup Opamp mode */
    PLIB_CMP_OpAmpEnable(CMP_ID_1);
    PLIB_CMP_OpAmpOutputDisable(CMP_ID_1);

    /* Enable Comparator */
    PLIB_CMP_Enable(CMP_ID_1);



}

/*******************************************************************************
 End of File
*/
