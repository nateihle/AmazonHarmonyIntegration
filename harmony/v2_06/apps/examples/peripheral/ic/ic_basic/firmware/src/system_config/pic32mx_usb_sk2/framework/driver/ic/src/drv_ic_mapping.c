/*******************************************************************************
  IC Driver Dynamic to Static mapping

  Company:
    Microchip Technology Inc.

  File Name:
    drv_ic_mapping.c

  Summary:
    Source code for the IC driver dynamic APIs to static API mapping.

  Description:
    This file contains code that maps dynamic APIs to static whenever
    the static mode of the driver is selected..

  Remarks:
    Static interfaces incorporate the driver instance number within the names
    of the routines, eliminating the need for an object ID or object handle.

    Static single-open interfaces also eliminate the need for the open handle.
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

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "system_config.h"
#include "system_definitions.h"


SYS_MODULE_OBJ DRV_IC_Initialize(const SYS_MODULE_INDEX index,const SYS_MODULE_INIT * const init)
{
    SYS_MODULE_OBJ returnValue = index;

    switch(index)
    {
        case DRV_IC_INDEX_0:
        {
            DRV_IC0_Initialize();
            break;
        }
        default:
        {
            returnValue = SYS_MODULE_OBJ_INVALID;
            break;
        }
    }
    return returnValue;
}

DRV_HANDLE DRV_IC_Start(const SYS_MODULE_INDEX drvIndex, const DRV_IO_INTENT intent)
{
    SYS_MODULE_OBJ returnValue = drvIndex;

    switch(drvIndex)
    {
        case DRV_IC_INDEX_0:
        {
            DRV_IC0_Start();
            break;
        }
        default:
        {
            returnValue = SYS_MODULE_OBJ_INVALID;
            break;
        }
    }
    return returnValue;
}


void DRV_IC_Stop(DRV_HANDLE handle)
{
    switch(handle)
    {
        case DRV_IC_INDEX_0:
        {
            DRV_IC0_Stop();
            break;
        }
        default:
        {
            break;
        }
    }
}

uint32_t DRV_IC_Capture32BitDataRead(DRV_HANDLE handle)
{
    uint32_t returnValue;

    switch(handle)
    {
        case DRV_IC_INDEX_0:
        {
            returnValue = DRV_IC0_Capture32BitDataRead();
            break;
        }
        default:
        {
            returnValue = 0;
            break;
        }
    }
    return returnValue;
}


uint16_t DRV_IC_Capture16BitDataRead(DRV_HANDLE handle)
{
    uint16_t returnValue;

    switch(handle)
    {
        case DRV_IC_INDEX_0:
        {
            returnValue = DRV_IC0_Capture16BitDataRead();
            break;
        }
        default:
        {
            returnValue = 0;
            break;
        }
    }
    return returnValue;
}


bool DRV_IC_BufferIsEmpty(DRV_HANDLE handle)
{
    bool returnValue = true;  // Default state of buffer is empty.

    switch(handle)
    {
        case DRV_IC_INDEX_0:
        {
            returnValue = DRV_IC0_BufferIsEmpty();
            break;
        }
        default:
        {
            break;
        }
    }
    return returnValue;
}

/*******************************************************************************
 End of File
*/
