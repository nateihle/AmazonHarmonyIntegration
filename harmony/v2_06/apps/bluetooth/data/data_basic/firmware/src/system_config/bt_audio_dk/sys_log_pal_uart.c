/*******************************************************************************
  Application System Log Messaging Services

  Company:
    Microchip Technology Inc.

  File Name:
    sys_log_pal_uart.c

  Summary:
    Provides UART support for messaging services for the application event logging.

  Description:
    Provides UART support for messaging services for the application event logging.

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

// Contents of this file are only compiled if logging is enabled.
#if defined( ENABLE_SYS_LOG )

// *****************************************************************************
// *****************************************************************************
// Section: Include Files
// *****************************************************************************
// *****************************************************************************
#include "user_config.h"
#include "application/sys_log/sys_log_local.h"

#include <xc.h>

#include "system/clk/sys_clk.h"
#include "peripheral/usart/plib_usart.h"
#include "peripheral/int/plib_int.h"
#include "peripheral/ports/plib_ports.h"


// ****************************************************************************
// ****************************************************************************
// Configuration of System Log UART Channel
// ****************************************************************************
// ****************************************************************************
#define LOG_UART_USART_ID        USART_ID_4

#define LOG_UART_INT_SOURCE      INT_SOURCE_USART_4_TRANSMIT
#define LOG_UART_INT_VECTOR      INT_VECTOR_UART4
#define LOG_UART_ISR_VECTOR      _UART_4_VECTOR // From <xc.h>

#define LOG_UART_OUTPUT_FUNCTION OUTPUT_FUNC_U4TX
#define LOG_UART_OUTPUT_PIN      OUTPUT_PIN_RPF12

#define LOG_UART_PERIPHERAL_BUS  CLK_BUS_PERIPHERAL_1  // UART on Peripheral Bus 1

#define LOG_UART_BAUD_RATE       115200


// *****************************************************************************
// *****************************************************************************
// Section: File Scope Variables
// *****************************************************************************
// *****************************************************************************
static bool _bEnabledUART = false;
static bool _bTransmitting;


// *****************************************************************************
// *****************************************************************************
// Section: File Scope Functions
// *****************************************************************************
// *****************************************************************************
static void _InitUart(void)
{
    // Connect LOG TX pin to LOG UART
    PLIB_PORTS_RemapOutput(PORTS_ID_0,LOG_UART_OUTPUT_FUNCTION,LOG_UART_OUTPUT_PIN);
    
    // Configure Log UART
  #if LOG_UART_BAUD_RATE > (115200*2)
    PLIB_USART_BaudRateHighEnable(LOG_UART_USART_ID);
  #else
    PLIB_USART_BaudRateHighDisable(LOG_UART_USART_ID);
  #endif

    PLIB_USART_InitializeOperation(LOG_UART_USART_ID,
                                   USART_RECEIVE_FIFO_ONE_CHAR, // Not used
                                   USART_TRANSMIT_FIFO_IDLE,    // Same as TX_DONE
                                   USART_ENABLE_TX_RX_USED);
    PLIB_USART_LineControlModeSelect(LOG_UART_USART_ID,USART_8N1);
    PLIB_USART_BaudRateSet(LOG_UART_USART_ID,
                           SYS_CLK_PeripheralFrequencyGet(LOG_UART_PERIPHERAL_BUS),
                           LOG_UART_BAUD_RATE);
    
    // Configure Log UART interrupt
    PLIB_INT_VectorPrioritySet(INT_ID_0,LOG_UART_INT_VECTOR,1);
    PLIB_INT_VectorSubPrioritySet(INT_ID_0,LOG_UART_INT_VECTOR,INT_SUBPRIORITY_LEVEL0);
}


// *****************************************************************************
// *****************************************************************************
// Section: System Log Functions
// *****************************************************************************
// *****************************************************************************
void SYS_LOG_PAL_Init(void)
{
    _bTransmitting = false;
    _InitUart();
}


bool SYS_LOG_PAL_Transmitting(void)
{
    return _bTransmitting;
}


void SYS_LOG_PAL_StartTransmitting(void)
{
    char nextChar;
    
    _bTransmitting = false;
    
    if ( _bEnabledUART )
    { // If UART is enabled...
        while ( !PLIB_USART_TransmitterBufferIsFull(LOG_UART_USART_ID) )
        {// While transmit buffer is NOT full.
            nextChar = SYS_LOG_GetNextChar();
            if ( nextChar != 0 )
            {// Have character to transmit
                _bTransmitting = true;
                PLIB_USART_TransmitterByteSend(LOG_UART_USART_ID,nextChar);
            }
            else
            {// Nothing left to transmit.
                break;
            }
        }//while ( !PLIB_USART_TransmitterBufferIsFull(LOG_UART_USART_ID) )
    }

    if ( _bTransmitting )
    {
        PLIB_INT_SourceEnable(INT_ID_0,LOG_UART_INT_SOURCE);
    }
    else
    {
        PLIB_INT_SourceDisable(INT_ID_0,LOG_UART_INT_SOURCE);
    }
}


void SYS_LOG_PAL_Start(void)
{
    // Enable UART
    PLIB_USART_TransmitterEnable(LOG_UART_USART_ID); 
    PLIB_USART_Enable(LOG_UART_USART_ID);
    _bEnabledUART  = true;
}


void SYS_LOG_PAL_Stop(void)
{
    // Disable UART
    PLIB_USART_Disable(LOG_UART_USART_ID);
    PLIB_USART_TransmitterDisable(LOG_UART_USART_ID); 
    _bEnabledUART  = false;
}


void __attribute__( (interrupt(IPL1SOFT), vector(LOG_UART_ISR_VECTOR))) SYS_LOG_InterruptHandler(void);
void SYS_LOG_InterruptHandler(void)
{
    char nextChar;

    // Clear the TX interrupt Flag
    PLIB_INT_SourceFlagClear(INT_ID_0,LOG_UART_INT_SOURCE);
            
    while ( !PLIB_USART_TransmitterBufferIsFull(LOG_UART_USART_ID) )
    {// While transmit buffer is NOT full
        nextChar = SYS_LOG_GetNextChar();
        if ( nextChar != 0 )
        {// Have character to transmit
            _bTransmitting = true;
            PLIB_USART_TransmitterByteSend(LOG_UART_USART_ID,nextChar);
        }
        else
        {// Nothing left to transmit.
            PLIB_INT_SourceDisable(INT_ID_0,LOG_UART_INT_SOURCE);
            _bTransmitting = false;
            break;
        }
    }//end while ( !PLIB_USART_TransmitterBufferIsFull(LOG_UART_USART_ID) )

}

#endif//defined( ENABLE_SYS_LOG )
