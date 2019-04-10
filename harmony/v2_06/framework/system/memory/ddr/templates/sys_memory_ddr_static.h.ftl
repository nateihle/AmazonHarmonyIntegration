/*******************************************************************************
  Memory System Service Settings for DDR Initialization

  Company:
    Microchip Technology Inc.

  File Name:
    sys_memory_ddr_static.h

  Summary:
    Memory System Service implementation for the DDR controller.

  Description:
    The Memory System Service initializes the DDR Controller and PHY to
    provide access to external DDR2 SDRAM.

  Remarks:
    Static interfaces incorporate the driver instance number within the names
    of the routines, eliminating the need for an object ID or object handle.
    Static single-open interfaces also eliminate the need for the open handle.
*******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef _SYS_MEMORY_DDR_STATIC_H
#define _SYS_MEMORY_DDR_STATIC_H

// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

/* Host Commands */
#define DRV_DDR_IDLE_NOP                0x00FFFFFF
#define DRV_DDR_PRECH_ALL_CMD           0x00FFF401
#define DRV_DDR_REF_CMD                 0x00FFF801
#define DRV_DDR_LOAD_MODE_CMD           0x00FFF001
#define DRV_DDR_CKE_LOW                 0x00FFEFFE

/* DDR address decoding */
#define COL_HI_RSHFT            0
#define COL_HI_MASK             0
#define COL_LO_MASK             ((1 << ${CONFIG_SYS_MEMORY_DDR_COL_BITS}) - 1)

#define BA_RSHFT                ${CONFIG_SYS_MEMORY_DDR_COL_BITS}
#define BANK_ADDR_MASK          ((1 << ${CONFIG_SYS_MEMORY_DDR_BANK_BITS}) - 1)

#define ROW_ADDR_RSHIFT         (BA_RSHFT + ${CONFIG_SYS_MEMORY_DDR_BANK_BITS})
#define ROW_ADDR_MASK           ((1 << ${CONFIG_SYS_MEMORY_DDR_ROW_BITS}) - 1)

//#define CS_ADDR_RSHIFT        (ROW_ADDR_RSHIFT + ${CONFIG_SYS_MEMORY_DDR_ROW_BITS})
#define CS_ADDR_RSHIFT          0
#define CS_ADDR_MASK            0

#define CTRL_CLK_PERIOD         (${CONFIG_SYS_MEMORY_DDR_CLK_PER} * 2)

// *****************************************************************************
// *****************************************************************************
// Section: Function Prototypes
// *****************************************************************************
// *****************************************************************************

//******************************************************************************
/* Function:
    void SYS_MEMORY_DDR_Initialize ( void * data)

  Summary:
    Initializes and Enables the DDR External Memory Controller.

  Description:
    This function Enables the external DDR memory controller module.

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
    SYS_MEMORY_DDR_Initialize(NULL);
  </code>

  Remarks:
    This routine must be called before any attempt to access external
    DDR memory.

    Not all features are available on all devices. Refer to the specific
	device data sheet to determine availability.
*/
void SYS_MEMORY_DDR_Initialize(void);

#endif // #ifndef _SYS_MEMORY_DDR_STATIC_H

/*******************************************************************************
 End of File
*/
