/*******************************************************************************
 Data Stream I2C Source File

  File Name:
    i2c.c

  Summary:
 Data Stream I2C source

  Description:
    This file contains source code necessary for the data stream interface.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
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
// DOM-IGNORE-END
#include "system/common/sys_module.h"
#include "../datastream.h"
#include "peripheral/ports/plib_ports.h"
#include "peripheral/usart/plib_usart.h"
#include "peripheral/dma/plib_dma.h"
#include "system_config.h"
#include "system/clk/sys_clk.h"
#include "framework/driver/i2c/drv_i2c_static.h"
#include "../datastream.h"

// *****************************************************************************
/* Application Status Information

  Summary:
    I2C registers definitions

  Description:
    The status of the bootloader operation are stored in different internal
    registers. The I2C master queries these registers and the slave bootloader
    application responds with the appropriate status
*/
void DATASTREAM_Tasks( void )
{

    if (handler == (DATASTREAM_HandlerType*)NULL)
    {
        return;
    }
    
    if (RX == currDir)
    {
        
        if(DRV_I2C0_WaitForReadByteAvailable())
        {
            
            _rxBuffer[_rxCurSize] = DRV_I2C0_ByteRead();
            
            if(I2C2STATbits.D_A == 1)
            { 
                _rxCurSize++;
            }
            while(DRV_I2C0_SetUpByteRead() == false);
            BSP_LEDStateSet(BSP_LED_2,1);
              
            if (_rxCurSize == _rxMaxSize)
            {
             currDir = IDLE;
             BSP_LEDStateSet(BSP_LED_2,0);
             handler(DATASTREAM_BUFFER_EVENT_COMPLETE, (DATASTREAM_BUFFER_HANDLE)_bufferHandle, _context);         
            }
            
        }
    }
    else if(TX == currDir)
    {
        if (DRV_I2C0_WaitForReadByteAvailable() && (_txCurPos < _txMaxSize))
        {
            DRV_I2C0_ByteWrite(_txBuffer[_txCurPos++]);
            DRV_I2C0_WaitForByteWriteToComplete();
                  
            if (_txCurPos == _txMaxSize) // All data has been sent or is in the buffer
            {
                currDir = IDLE;
                handler(DATASTREAM_BUFFER_EVENT_COMPLETE, (DATASTREAM_BUFFER_HANDLE)_bufferHandle, _context);
            }
        }
    }
}
