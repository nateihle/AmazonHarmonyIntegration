/*******************************************************************************
  I2C Driver Functions for Static Single Instance Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_i2c_static.c
	
  Summary:
    I2C driver impementation for the static single instance driver.

  Description:
    The I2C device driver provides a simple interface to manage the I2C
    modules on Microchip microcontrollers. This file contains implemenation
    for the I2C driver.
    
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
#include "framework/driver/i2c/drv_i2c_static.h"



// *****************************************************************************
// *****************************************************************************
// Section: Instance 0 static driver functions
// *****************************************************************************
// *****************************************************************************
void DRV_I2C0_Initialize(void)
{
    /* Initialize I2C0 */
    PLIB_I2C_BaudRateSet(I2C_ID_1, SYS_CLK_PeripheralFrequencyGet(CLK_BUS_PERIPHERAL_1), 50000);
    PLIB_I2C_StopInIdleDisable(I2C_ID_1);

    /* Low frequency is enabled (**NOTE** PLIB function logic inverted) */
    PLIB_I2C_HighFrequencyEnable(I2C_ID_1);

    /* Initialize interrupts */
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_I2C_1_MASTER);
    PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_I2C_1_MASTER);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_I2C_1_ERROR);
    PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_I2C_1_ERROR);
    PLIB_INT_VectorPrioritySet(INT_ID_0, INT_VECTOR_I2C1, INT_PRIORITY_LEVEL1);
    PLIB_INT_VectorSubPrioritySet(INT_ID_0, INT_VECTOR_I2C1, INT_SUBPRIORITY_LEVEL0);

    /* Enable I2C0 */
    PLIB_I2C_Enable(I2C_ID_1);
}

void DRV_I2C0_DeInitialize(void)
{
    /* Disable I2C0 */
    PLIB_I2C_Disable(I2C_ID_1);
}

// *****************************************************************************
// Section: Instance 0 Byte Transfer Functions (Master/Slave)
// *****************************************************************************
bool DRV_I2C0_SetUpByteRead(void)
{
    /* Check for receive overflow */
    if ( PLIB_I2C_ReceiverOverflowHasOccurred(I2C_ID_1))  
    {
        PLIB_I2C_ReceiverOverflowClear(I2C_ID_1); 
        return false;
    }
	
    /* Initiate clock to receive */
    PLIB_I2C_MasterReceiverClock1Byte(I2C_ID_1);
    return true;
}

bool DRV_I2C0_WaitForReadByteAvailable(void)
{
	/* Wait for Receive Buffer Full */
    if(PLIB_I2C_ReceivedByteIsAvailable(I2C_ID_1))
       return true;

	return false;
}

uint8_t DRV_I2C0_ByteRead(void)
{	
    /* Return received value */
    return (PLIB_I2C_ReceivedByteGet(I2C_ID_1));
}

bool DRV_I2C0_ByteWrite(const uint8_t byte)
{
        
    /* if no IWCOL errors exist, then transmit byte */
	if ( (!(PLIB_I2C_TransmitterIsBusy(I2C_ID_1))) &&
		(PLIB_I2C_TransmitterByteHasCompleted(I2C_ID_1)) )
    {
		PLIB_I2C_TransmitterByteSend(I2C_ID_1, byte);
	}
	
	/* check if writing to I2CxTRN caused a transmitter overflow */
	if (PLIB_I2C_TransmitterOverflowHasOccurred(I2C_ID_1))
	    return false;
    

    return true;
}

bool DRV_I2C0_WaitForByteWriteToComplete(void)
{
    /* if TBF == 0 and TRSTAT == 0 then write complete */ 
	
	if ( (!(PLIB_I2C_TransmitterIsBusy(I2C_ID_1))) &&
		(PLIB_I2C_TransmitterByteHasCompleted(I2C_ID_1)) )
		return true;
	
    return false;
}

bool DRV_I2C0_WriteByteAcknowledged(void)
{
    /* Check to see if transmit ACKed = true or NACKed = false */
    if (PLIB_I2C_TransmitterByteWasAcknowledged(I2C_ID_1))
       return true;

    return false;
}

// *****************************************************************************
// Section: Instance 0 I2C Master Mode Functions
// *****************************************************************************
void DRV_I2C0_BaudRateSet(I2C_BAUD_RATE baudRate)
{
    /* Disable I2C0 */
    PLIB_I2C_Disable(I2C_ID_1);
    
    /* Change baud rate */
    PLIB_I2C_BaudRateSet(I2C_ID_1, SYS_CLK_PeripheralFrequencyGet(CLK_BUS_PERIPHERAL_1), baudRate);

    /* Low frequency is enabled (**NOTE** PLIB function inverted) */
    PLIB_I2C_HighFrequencyEnable(I2C_ID_1);
 
    /* Enable I2C0 */
    PLIB_I2C_Enable(I2C_ID_1);       
}

bool DRV_I2C0_MasterBusIdle(void)
{
    if (PLIB_I2C_BusIsIdle(I2C_ID_1))
       return true;
    else
       return false;
}

bool DRV_I2C0_MasterStart(void)
{
    /* if bus is not idle return with false */
	if (!(PLIB_I2C_BusIsIdle(I2C_ID_1)))
	   return false;

    /* return false is Bus Collision exisits */
    if (PLIB_I2C_ArbitrationLossHasOccurred(I2C_ID_1))
    {
        return false;
    }
    
    /* Issue start */
    PLIB_I2C_MasterStart(I2C_ID_1);
       
    return true;
}

bool DRV_I2C0_WaitForStartComplete(void)
{
    /* Wait for start/restart sequence to finish  (hardware clear) */
	
	if ( (PLIB_I2C_BusIsIdle(I2C_ID_1)) &&
         (PLIB_I2C_StartWasDetected(I2C_ID_1)) )
	    return true;
	
	return false;
}

bool DRV_I2C0_MasterRestart(void)
{
    /* if bus is not idle return with false */
	if (!(PLIB_I2C_BusIsIdle(I2C_ID_1)))
	   return false;

    /* return false is Bus Collision exisits */
    if (PLIB_I2C_ArbitrationLossHasOccurred(I2C_ID_1))
    {
        return false;
    }
    
    /* Issue restart */
    PLIB_I2C_MasterStartRepeat(I2C_ID_1); 
       
    return true;    
}

bool DRV_I2C0_MasterStop(void)
{   
    
    /* if bus is not idle return with false */
	if (!(PLIB_I2C_BusIsIdle(I2C_ID_1)))
	   return false;

    /* Issue stop */
    PLIB_I2C_MasterStop(I2C_ID_1);

    
     return true;
}

bool DRV_I2C0_WaitForStopComplete(void)
{
    if ( (PLIB_I2C_BusIsIdle(I2C_ID_1)) && 
	    (PLIB_I2C_StopWasDetected(I2C_ID_1))	)	
        return true;
		  
		return false;  
}

void DRV_I2C0_MasterACKSend(void)
{
    /* Check if receive is ready to ack */
    if ( PLIB_I2C_MasterReceiverReadyToAcknowledge(I2C_ID_1) )
    {
        PLIB_I2C_ReceivedByteAcknowledge (I2C_ID_1, true);
    }    

}

void DRV_I2C0_MasterNACKSend(void)
{
    /* Check if receive is ready to nack */
    if ( PLIB_I2C_MasterReceiverReadyToAcknowledge(I2C_ID_1) )
    {
       PLIB_I2C_ReceivedByteAcknowledge (I2C_ID_1, false);
    }    
}

bool DRV_I2C0_WaitForACKOrNACKComplete(void)
{
    /* Check for ACK/NACK to complete */
    if(PLIB_I2C_ReceiverByteAcknowledgeHasCompleted(I2C_ID_1))
		return true;
	
	return false;
}



/*******************************************************************************
 End of File
*/