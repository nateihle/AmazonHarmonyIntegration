/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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

#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;
uint32_t AppSysContext;
/* The buffer to be CRC computed and transferred is "123456789" */
const uint8_t flashBuff[]= {0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0,0,0,0 };
uint8_t __attribute__((coherent)) ramBuff[250];
SYS_DMA_CHANNEL_HANDLE channelHandle;
SYS_DMA_CHANNEL_OPERATION_MODE_CRC crc;  
uint32_t blockCrc;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary local functions.
*/

static void App_Mem2Mem_Event_Handler(SYS_DMA_TRANSFER_EVENT event,
        SYS_DMA_CHANNEL_HANDLE handle, uintptr_t contextHandle);
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */  
void APP_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            /* OFF LEDs on board */
            BSP_LEDOff(BSP_LED_1);
            BSP_LEDOff(BSP_LED_2);
            BSP_LEDOff(BSP_LED_3);                
                  
            /* Allocate a DMA channel */
            channelHandle = SYS_DMA_ChannelAllocate(DMA_CHANNEL_0);
            if(SYS_DMA_CHANNEL_HANDLE_INVALID != channelHandle)
            {
                /* Register an event handler for the channel */
                SYS_DMA_ChannelTransferEventHandlerSet(channelHandle,
                    App_Mem2Mem_Event_Handler, (uintptr_t)&AppSysContext);

                /* Setup the channel */
                SYS_DMA_ChannelSetup(channelHandle,
                                 (SYS_DMA_CHANNEL_OP_MODE_BASIC | SYS_DMA_CHANNEL_OP_MODE_CRC ),
                                 DMA_TRIGGER_SOURCE_NONE);
                
                /* Initialize the CRC computation parameters */                
                crc.type            = DMA_CRC_LFSR;                      
                crc.bitOrder        = DMA_CRC_BIT_ORDER_MSB;
                crc.byteOrder       = DMA_CRC_BYTEORDER_NO_SWAPPING;
                crc.writeOrder      = SYS_DMA_CRC_WRITE_ORDER_MAINTAIN;
                crc.mode            = SYS_DMA_CHANNEL_CRC_MODE_BACKGROUND;
                
                crc.xorBitMask      = APP_DMA_CRC_BIT_MASK;
                crc.polyLength      = APP_DMA_CRC_POLY_16;                
                crc.data            = APP_DMA_CRC_INIT_VAL;    
                
                SYS_DMA_ChannelCRCSet(channelHandle, crc);

                /* Add the memory block transfer request.
                 * Note size is added with 'polylength' bytes to get
                 * comparable CRC results */                                                           
                SYS_DMA_ChannelTransferAdd(channelHandle, flashBuff,
                        (APP_DMA_CRC_BUFFER_SIZE + (crc.polyLength/8)) ,
                        ramBuff,(APP_DMA_CRC_BUFFER_SIZE + (crc.polyLength/8)),
                        (APP_DMA_CRC_BUFFER_SIZE + (crc.polyLength/8)));                         
                               
                /* Start the DMA transfer */
                SYS_DMA_ChannelForceStart(channelHandle);
        
                appData.state++;
            }
            else
            {
                /* Channel Handle not available */
                ;
            }                        
        }
        break;
         
        default:
        {
            /* Do Nothing */
            break;
        }
    }
}
 

static void App_Mem2Mem_Event_Handler(SYS_DMA_TRANSFER_EVENT event,
        SYS_DMA_CHANNEL_HANDLE handle, uintptr_t contextHandle)
{
    /* Success event */
    if(SYS_DMA_TRANSFER_EVENT_COMPLETE == event)
    {
        blockCrc = SYS_DMA_ChannelCRCGet();
        BSP_LEDOn(BSP_LED_2);     
    }
    /* Failure Event */
    else if(SYS_DMA_TRANSFER_EVENT_ABORT == event)
    {
        /* It should never come here */
    }
}
/*******************************************************************************
 End of File
 */
