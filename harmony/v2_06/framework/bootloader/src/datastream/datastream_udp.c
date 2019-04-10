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

#include "tcpip/tcpip.h"
#include "system_definitions.h"

char _Port_Buffer[6];
extern BOOTLOADER_DATA bootloaderData __attribute__((coherent, aligned(16)));
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
UDP_SOCKET              udpsocket;
bool                    connected;



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

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback funtions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

DRV_HANDLE DATASTREAM_Open(const DRV_IO_INTENT ioIntent)
{
    SYS_STATUS          tcpipStat;
    static IPV4_ADDR    dwLastIP[2] = { {-1}, {-1} };
    IPV4_ADDR           ipAddr;
    TCPIP_NET_HANDLE    netH;
    int                 i, nNets;
    
            tcpipStat = TCPIP_STACK_Status(sysObj.tcpip);
            
            if(tcpipStat < 0)
            {   // some error occurred
                return(DRV_HANDLE_INVALID);
            }
            else if(tcpipStat == SYS_STATUS_READY)
            {   
                // now that the stack is ready we can check the 
                // available interfaces 
                nNets = TCPIP_STACK_NumberOfNetworksGet();
                
                for (i = 0; i < nNets; i++)
                {
                    netH = TCPIP_STACK_IndexToNet(i);
                    ipAddr.Val = TCPIP_STACK_NetAddress(netH);
                    if(dwLastIP[i].Val != ipAddr.Val)
                    {
                        dwLastIP[i].Val = ipAddr.Val;

                        if (ipAddr.v[0] != 0 && ipAddr.v[0] != 169) // Wait for a Valid IP
                        {
                            strcpy(_Port_Buffer, BOOTLOADER_UDP_PORT_NUMBER);
                            uint16_t port = atoi(_Port_Buffer);
                        
                            udpsocket = TCPIP_UDP_ServerOpen(IP_ADDRESS_TYPE_IPV4,
                                 port, 0);
                
                            if (udpsocket == INVALID_SOCKET)
                            {
                                return(DRV_HANDLE_INVALID);
                            }
                            connected = true;
                            //BSP_LEDStateSet(BSP_LED_2, BSP_LED_STATE_ON);
                            return(0); //Success
                        }
                    }
                }
            }
            return(DRV_HANDLE_INVALID);
            
}

void DATASTREAM_Tasks(void)
{
    
    uint16_t avlBytes = 0;
    
    if (handler == (DATASTREAM_HandlerType*)NULL)
    {
        return;
    }
    
    if (TX == currDir)
    {
    
    if(TCPIP_UDP_PutIsReady(udpsocket) >= _txMaxSize)
    {   
        TCPIP_UDP_ArrayPut(udpsocket, &bootloaderData.data->buffers.buff2[0], _txMaxSize);
        TCPIP_UDP_Flush(udpsocket);
        currDir = IDLE;
        handler(DATASTREAM_BUFFER_EVENT_COMPLETE, (DATASTREAM_BUFFER_HANDLE)_bufferHandle, _txMaxSize);
        
    }

    }
    
    if (RX == currDir)
    {

    if (TCPIP_UDP_GetIsReady(udpsocket))
    {
        avlBytes = TCPIP_UDP_GetIsReady(udpsocket);
        
        avlBytes = TCPIP_UDP_ArrayGet(udpsocket, &bootloaderData.data->buffers.buff1[0], avlBytes);
        //TCPIP_UDP_Discard(appData.socket);
        currDir = IDLE;
        handler(DATASTREAM_BUFFER_EVENT_COMPLETE, (DATASTREAM_BUFFER_HANDLE)_bufferHandle, avlBytes);

    }
    
    }
    
}

DRV_CLIENT_STATUS DATASTREAM_ClientStatus(DRV_HANDLE handle)
{
    if(TCPIP_UDP_IsConnected(udpsocket) == true)
    {
        return DRV_CLIENT_STATUS_READY;
    }
    else
    {    
        return DRV_CLIENT_STATUS_BUSY;
    }
}

void DATASTREAM_Close(void)
{
    if (connected)
    {
        TCPIP_UDP_Close(udpsocket);
        TCPIP_STACK_Deinitialize(sysObj.tcpip);
        PLIB_ETH_Disable(sysObj.tcpip);
        
    }
    //Disable Interrupt sources so bootloader application runs without issues
    SYS_INT_SourceDisable(DRV_TMR_INTERRUPT_SOURCE_IDX0);
    SYS_INT_VectorPrioritySet(DRV_TMR_INTERRUPT_VECTOR_IDX0, INT_DISABLE_INTERRUPT);
    SYS_INT_VectorSubprioritySet(DRV_TMR_INTERRUPT_VECTOR_IDX0, INT_SUBPRIORITY_LEVEL0);
    SYS_INT_VectorPrioritySet(INT_VECTOR_ETH, INT_DISABLE_INTERRUPT);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_ETH, INT_SUBPRIORITY_LEVEL0);
    SYS_INT_SourceDisable(INT_SOURCE_ETH_1);
    PLIB_TMR_Stop(DRV_TMR_PERIPHERAL_ID_IDX0);
}

/*******************************************************************************
 End of File
 */

