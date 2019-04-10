/*******************************************************************************
    BT storage

  Company:
    Microchip Technology Inc.

  File Name:
    btstorage.c

  Summary:
    Contains the functional implementation of BT storage.

  Description:
    This file contains the functional implementation of BT storage.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

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
// DOM-IGNORE-END
#include "app.h"

//
// Bluetooth storage
// -------------------------------------------------------------------
//
// DotStack requires a non-volatile storage for keeping link key (pairing)
// information. This storage is implemnted using the FLASH or EEPROM part on the
// board. The part is connected to SPI channel.

static unsigned char mRamCopy[BT_STORAGE_SIZE];

BTMGR_PERSISTENT_DATA* btmgr_pal_getPersistentData(void)
{
    return (BTMGR_PERSISTENT_DATA*) mRamCopy;
}


void btmgr_pal_writePersistentData(BTMGR_PAL_WRITE_PERSISTENT_DATA_CALLBACK callback)
{
    if(false == APP_BT_PAIRING_STORAGE_SUPPORTED)
    {
    }
    else
    {
    }
}
/*******************************************************************************
 End of File
 */