/*******************************************************************************
 System Interrupts File

  File Name:
    system_interrupt.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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

#include "system/common/sys_common.h"
#include "app.h"
#include "system_definitions.h"

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************
//CUSTOM CODE - DO NOT DELETE
void DisplayISR (void);
//END OF CUSTOM CODE
 
 
 
void __ISR(_TIMER_9_VECTOR, ipl3AUTO) _IntHandlerDrvI2CMasterInstance0(void)
{
    DRV_I2C_BB_Tasks(sysObj.drvI2C0);
    //CUSTOM CODE - DO NOT DELETE
    BSP_LED_2Off();
    //END OF CUSTOM CODE
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_TIMER_9);
}


 

 




 




  
void __ISR(_UART6_TX_VECTOR, ipl1AUTO) _IntHandlerDrvUsartTransmitInstance0(void)
{
    DRV_USART_TasksTransmit(sysObj.drvUsart0);
}
void __ISR(_UART6_RX_VECTOR, ipl1AUTO) _IntHandlerDrvUsartReceiveInstance0(void)
{
    DRV_USART_TasksReceive(sysObj.drvUsart0);
}
void __ISR(_UART6_FAULT_VECTOR, ipl1AUTO) _IntHandlerDrvUsartErrorInstance0(void)
{
    DRV_USART_TasksError(sysObj.drvUsart0);
}
 
 

 

 

 

 

 
void __ISR(_CHANGE_NOTICE_F_VECTOR, ipl5AUTO) _IntHandlerChangeNotification_PortF(void)
{
    
       PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_CHANGE_NOTICE_F);
      //CUSTOM CODE - DO NOT CHANGE
    if (!PLIB_PORTS_PinGet(PORTS_ID_0, PORT_CHANNEL_F, PORTS_BIT_POS_6))
    {// MaxTouch Change Bar pin is low => event
        BSP_LED_2On();
        Nop();
//        SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, ".", 1);
        SYS_MESSAGE (".");
        DRV_MXT336T_ReadRequest(sysObj.drvMXT336T);
    }
  //END OF CUSTOM CODE
}



void __ISR(_CAN1_VECTOR, IPL1AUTO) _IntHandlerDrvCANInstance0(void)
{
    //uint8_t TestMessage[4]; //Test message to transmit on CAN

    if((PLIB_CAN_ModuleEventGet(CAN_ID_1) & (CAN_RX_EVENT)) != 0)
    {
        DisplayISR ();
        
        PLIB_CAN_ModuleEventClear(CAN_ID_1,  CAN_RX_EVENT); //Clear CAN Interrupt flag
    }

    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_CAN_1);
}

 

/*******************************************************************************
 End of File
*/
