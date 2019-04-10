/*******************************************************************************
  Application System Log Messaging Services

  Company:
    Microchip Technology Inc.

  File Name:
    sys_log_local.h

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

#ifndef __SYS_LOG_LOCAL_H_INCLUDED__
#define __SYS_LOG_LOCAL_H_INCLUDED__

#include <stdarg.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>


// *****************************************************************************
/* Function:
    char SYS_LOG_GetNextChar(void)

  Summary:
    Get next character from System Log queuing.

  Description:
    Get next character from System Log queuing.  Dequeue next message when the
    end of the current message is reached.  Return NULL if no messages are left
    in the queue.

  Precondition:
    ENABLE_SYS_LOG must be defined by the active MPLAB.X configuration, otherwise
    this function is replaced by a #define ((void)0).

  Parameters:
    None.

  Returns:
    Next System Log message character, or NULL if no messages are left.
*/
#if defined( ENABLE_SYS_LOG )
  char    SYS_LOG_GetNextChar(void);
#else
  #define SYS_LOG_GetNextChar()      ((void)0)
#endif//defined( ENABLE_SYS_LOG )


// *****************************************************************************
/* Function:
    void SYS_LOG_PAL_Init(void)

  Summary:
    Initializes UART for System Log messages.

  Description:
    Initializes UART for System Log messages.

  Precondition:
    ENABLE_SYS_LOG must be defined by the active MPLAB.X configuration, otherwise
    this function is replaced by a #define ((void)0).

  Parameters:
    None.

  Returns:
    None.

*/
#if defined( ENABLE_SYS_LOG )
  void    SYS_LOG_PAL_Init(void);
#else
  #define SYS_LOG_PAL_Init()         ((void)0)
#endif//defined( ENABLE_SYS_LOG )


// *****************************************************************************
/* Function:
    void SYS_LOG_PAL_Start(void)

  Summary:
    Starts (enables) UART.

  Description:
    Starts (enables) UART.

  Precondition:
    ENABLE_SYS_LOG must be defined by the active MPLAB.X configuration, otherwise
    this function is replaced by a #define ((void)0).

  Parameters:
    None.

  Returns:
    None.

*/

#if defined( ENABLE_SYS_LOG )
  void    SYS_LOG_PAL_Start(void);
#else
  #define SYS_LOG_PAL_Start()        ((void)0)
#endif//defined( ENABLE_SYS_LOG )


// *****************************************************************************
/* Function:
    void SYS_LOG_PAL_Stop(void)

  Summary:
    Stops (disables) UART.

  Description:
    Stops (disables) UART.

  Precondition:
    ENABLE_SYS_LOG must be defined by the active MPLAB.X configuration, otherwise
    this function is replaced by a #define ((void)0).

  Parameters:
    None.

  Returns:
    None.

*/

#if defined( ENABLE_SYS_LOG )
  void    SYS_LOG_PAL_Stop(void);
#else
  #define SYS_LOG_PAL_Stop()        ((void)0)
#endif//defined( ENABLE_SYS_LOG )


// *****************************************************************************
/* Function:
    bool  SYS_LOG_PAL_Transmitting(void)

  Summary:
    Returns true if UART is transmitting messages, false otherwise.

  Description:
    Returns true if UART is transmitting messages, false otherwise.

  Precondition:
    ENABLE_SYS_LOG must be defined by the active MPLAB.X configuration, otherwise
    this function is replaced by a #define ((void)0).

  Parameters:
    None.

  Returns:
    Returns true if UART is transmitting messages, false otherwise.

*/
#if defined( ENABLE_SYS_LOG )
  bool    SYS_LOG_PAL_Transmitting(void);
#else
  #define SYS_LOG_PAL_Transmitting() ((void)0)
#endif//defined( ENABLE_SYS_LOG )


// *****************************************************************************
/* Function:
    void SYS_LOG_PAL_StartTransmitting(void)

  Summary:
    Start transmitting system log messages over the UART.

  Description:
    Start transmitting system log messages over the UART.

  Precondition:
    ENABLE_SYS_LOG must be defined by the active MPLAB.X configuration, otherwise
    this function is replaced by a #define ((void)0).

  Parameters:
    None.

  Returns:
    None.

*/
#if defined( ENABLE_SYS_LOG )
  void    SYS_LOG_PAL_StartTransmitting(void);
#else
  #define SYS_LOG_PAL_StartTransmitting() ((void)0)
#endif//defined( ENABLE_SYS_LOG )

#endif // __SYS_LOG_LOCAL_H_INCLUDED__
