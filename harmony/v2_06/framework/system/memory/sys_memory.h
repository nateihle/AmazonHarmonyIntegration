/*******************************************************************************
  Memory System Service Implementation

  Company:
    Microchip Technology Inc.

  File Name:
    sys_memory.h

  Summary:
    Memory System Service Implementation.

  Description:
    The Memory System Service provides a simple interface to manage the
    memory controllers. This file implements the core interface routines
    for the Memory System Service.

*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014-2017 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS  WITHOUT  WARRANTY  OF  ANY  KIND,
EITHER EXPRESS  OR  IMPLIED,  INCLUDING  WITHOUT  LIMITATION,  ANY  WARRANTY  OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A  PARTICULAR  PURPOSE.
IN NO EVENT SHALL MICROCHIP OR  ITS  LICENSORS  BE  LIABLE  OR  OBLIGATED  UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,  BREACH  OF  WARRANTY,  OR
OTHER LEGAL  EQUITABLE  THEORY  ANY  DIRECT  OR  INDIRECT  DAMAGES  OR  EXPENSES
INCLUDING BUT NOT LIMITED TO ANY  INCIDENTAL,  SPECIAL,  INDIRECT,  PUNITIVE  OR
CONSEQUENTIAL DAMAGES, LOST  PROFITS  OR  LOST  DATA,  COST  OF  PROCUREMENT  OF
SUBSTITUTE  GOODS,  TECHNOLOGY,  SERVICES,  OR  ANY  CLAIMS  BY  THIRD   PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE  THEREOF),  OR  OTHER  SIMILAR  COSTS.
*******************************************************************************/
//DOM-IGNORE-END
// *****************************************************************************
// *****************************************************************************
// Header Includes
// *****************************************************************************
// *****************************************************************************
#ifndef _SYS_MEMORY_H
#define _SYS_MEMORY_H

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility
  extern "C" {
#endif
// DOM-IGNORE-END

//******************************************************************************
/* Function:
    void SYS_MEMORY_Initialize ( void * data)

  Summary:
    Initializes and Enables the External Memory Controller(s).

  Description:
    This function Enables the external memory controller module(s).

  Precondition:
    None.

  Parameters:
    data            - Pointer to the data structure containing any data
                      necessary to initialize the hardware. This pointer may
                      be null if no data is required and default
                      initialization is to be used.

  Returns:
    None.

  Example:
  <code>
    SYS_MEMORY_Initialize(NULL);
  </code>

  Remarks:
    This routine must be called before any attempt to access external
    memory.

    Not all features are available on all devices. Refer to the specific
	device data sheet to determine availability.
*/
void SYS_MEMORY_Initialize(void);

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif

#endif //_SYS_MEMORY_H
