/*******************************************************************************
  PMP Driver Functions for Static Single Instance Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_pmp_static.c

  Summary:
    PMP driver implementation for the static single instance driver.

  Description:
    The PMP device driver provides a simple interface to manage the PMP
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
#include "framework/driver/pmp/drv_pmp_static.h"


// *****************************************************************************
// *****************************************************************************
// Section: Instance 0 static driver functions
// *****************************************************************************
// *****************************************************************************
void DRV_PMP0_Initialize(void)
{	

    PLIB_PMP_Disable(PMP_ID_0);
    PLIB_PMP_Enable(PMP_ID_0);   

}

void DRV_PMP0_ModeConfig(void)
{

    /*Configure Main Communication Mode */    
    PLIB_PMP_OperationModeSelect(PMP_ID_0, PMP_MASTER_READ_WRITE_STROBES_INDEPENDENT);

    /*Set the data width size*/
    PLIB_PMP_DataSizeSelect(PMP_ID_0, PMP_DATA_SIZE_8_BITS);
    /*Setup Read Strobe Registers */    
    PLIB_PMP_ReadWriteStrobePortEnable(PMP_ID_0);
    PLIB_PMP_ReadWriteStrobePolaritySelect(PMP_ID_0, PMP_POLARITY_ACTIVE_LOW);
    /*Setup Write Strobe Registers */
    PLIB_PMP_WriteEnableStrobePortEnable(PMP_ID_0);
    PLIB_PMP_WriteEnableStrobePolaritySelect(PMP_ID_0, PMP_POLARITY_ACTIVE_LOW);


    /* Configure the wait states */
    PLIB_PMP_WaitStatesDataSetUpSelect(PMP_ID_0, PMP_DATA_WAIT_ONE);
    PLIB_PMP_WaitStatesStrobeSelect(PMP_ID_0, PMP_STROBE_WAIT_4);
    PLIB_PMP_WaitStatesDataHoldSelect(PMP_ID_0, PMP_DATA_HOLD_1);

}

void DRV_PMP0_TimingSet(PMP_DATA_WAIT_STATES dataWait,
                   PMP_STROBE_WAIT_STATES strobeWait,
                   PMP_DATA_HOLD_STATES dataHold)
{
   /* Configure the wait states */
    PLIB_PMP_WaitStatesDataSetUpSelect(PMP_ID_0, dataWait);
    PLIB_PMP_WaitStatesStrobeSelect(PMP_ID_0, strobeWait);
    PLIB_PMP_WaitStatesDataHoldSelect(PMP_ID_0, dataHold);
}

void DRV_PMP0_Write(uint8_t data)
{
    PLIB_PMP_MasterSend(PMP_ID_0, data);
    while(PLIB_PMP_PortIsBusy(PMP_ID_0) == true);
}

uint8_t DRV_PMP0_Read(void)
{
    uint8_t value;
    value = PLIB_PMP_MasterReceive(PMP_ID_0);
    while(PLIB_PMP_PortIsBusy(PMP_ID_0) == true);
	
    return (value);
}


/*******************************************************************************
 End of File
*/
