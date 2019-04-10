/*******************************************************************************
  Application configuration

  Company:
    Microchip Technology Inc.

  File Name:
    app_nvm.h

  Summary:
    Contains the application specific defintions and function prototypes
    for NVM.

  Description:
    This file contains the application specific defintions and function prototypes
    for NVM.
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

#ifndef _APP_CONFIG_HEADER_H
#define _APP_CONFIG_HEADER_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: Application Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_NVMwriteWord (uint32_t address, uint32_t data)

  Summary:
     Writes a word in NVM.

  Description:
    Writes a word (4 bytes) in NVM flash.
*/

void APP_NVMwriteWord (uint32_t address, uint32_t data);

/*******************************************************************************
  Function:
    bool APP_NVMIsWriteCompleted (void)

  Summary:
     Returns the write status.

  Description:
    Returns the write status.

  Returns: 0 if the last write operation was completed
           >0 if the last write operation is underprocessing.

*/
bool APP_NVMIsWriteCompleted(void);

/*******************************************************************************
  Function:
    void APP_NVMIsVoltageError (void)

  Summary:
     Checks whether there was a low voltage error

  Description:
    Returns the write status.

  Returns: 1 if the there was a low voltage error, 0 otherwise

*/
bool APP_NVMIsVoltageError(void);

/*******************************************************************************
  Function:
    void APP_NVMIsWriteOPerationTerminated (void)

  Summary:
     Checks whether there was a write termination error

  Description:
    Returns the write status.

  Returns: 1 if the there was write termination error, 0 otherwise

*/
bool APP_NVMIsWriteOPerationTerminated(void);

/*******************************************************************************
  Function:
    void APP_NVMDisableOperation (void)

  Summary:
    Disables write operations to nvm.

  Description:
    Disables write operations to nvm.

  Returns: None

*/
void APP_NVMDisableOperation(void);

#endif /* _APP_CONFIG_HEADER_H */
/*******************************************************************************
 End of File
*/

