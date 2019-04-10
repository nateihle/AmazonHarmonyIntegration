/*******************************************************************************
  Application configuration

  Company:
    Microchip Technology Inc.

  File Name:
    app_nvm.c

  Summary:
    Contains the functional implementation of application specific
    NVM functions.

  Description:
    This file contains the functional implementation of application specific
    NVM functions.
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

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Local Function
// *****************************************************************************
// *****************************************************************************
static uint32_t virtualToPhysical(uint32_t address);

// *****************************************************************************
// *****************************************************************************
// Section: Global Variable Definitions
// *****************************************************************************
// *****************************************************************************


/*******************************************************************************

  Function:
    static uint32_t virtualToPhysical (uint32_t address)

  Summary:
    Converts a virtual memory address to a physical one
*/
static uint32_t virtualToPhysical(uint32_t address)
{
    return (address & 0x1FFFFFFF);
}


/*******************************************************************************

  Function:
    void APP_NVMwriteWord (uint32_t address, uint32_t data)

  Summary:
    Writes a word in flash memory (4 bytes)
*/
void APP_NVMwriteWord(uint32_t address, uint32_t data)
{
   /* Base address of where word is to be written */
   PLIB_NVM_FlashAddressToModify(NVM_ID_0, virtualToPhysical(address));

   /* Word of data to be written */
   PLIB_NVM_FlashProvideData(NVM_ID_0 , data);

   /* Disable flash write/erase operations */
   PLIB_NVM_MemoryModifyInhibit(NVM_ID_0);

   /* Select Word Program operation */
   PLIB_NVM_MemoryOperationSelect(NVM_ID_0, WORD_PROGRAM_OPERATION);

   /* Enable flash write/erase operations */
   PLIB_NVM_MemoryModifyEnable(NVM_ID_0);

   /* Write the unlock key sequence */
   PLIB_NVM_FlashWriteKeySequence(NVM_ID_0, 0x0);
   PLIB_NVM_FlashWriteKeySequence(NVM_ID_0, 0xAA996655);
   PLIB_NVM_FlashWriteKeySequence(NVM_ID_0, 0x556699AA);

   /* Start Writing */
   PLIB_NVM_FlashWriteStart(NVM_ID_0);

   bttask_setState(BTTASK_STATE_INTERNAL_NVM_WRITE_START);
}

/*******************************************************************************

  Function:
    bool APP_NVMIsWriteCompleted (void)

  Summary:
     Returns the write status.
 */
bool APP_NVMIsWriteCompleted(void)
{
    return PLIB_NVM_FlashWriteCycleHasCompleted(NVM_ID_0);

}

/*******************************************************************************

  Function:
    void APP_NVMIsVoltageError (void)

  Summary:
     Checks whether there was a low voltage error
 */
bool APP_NVMIsVoltageError(void)
{
    return PLIB_NVM_LowVoltageIsDetected(NVM_ID_0);
}


/*******************************************************************************

  Function:
    void APP_NVMIsWriteOPerationTerminated (void)

  Summary:
     Checks whether there was a write termination error
 */
bool APP_NVMIsWriteOPerationTerminated(void)
{
    return PLIB_NVM_WriteOperationHasTerminated(NVM_ID_0);
}

/*******************************************************************************

  Function:
    void APP_NVMDisableOperation (void)

  Summary:
    Disables write operations to nvm.
 */
void APP_NVMDisableOperation(void)
{
    /* Disable future Flash Write/Erase operations */
   PLIB_NVM_MemoryModifyInhibit(NVM_ID_0);
}


/*******************************************************************************
 End of File
 */

