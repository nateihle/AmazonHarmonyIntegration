/*******************************************************************************
  MPLAB Harmony Application File

  Company:
    Microchip Technology Inc.

  File Name:
     RN4871.c

  Summary:
                            ***THIS IS ALPHA ONLY!***
    This file implements basic Communication with the RN4871 that works with the
    Microchip BLE App (BLEsensorAPP) This app can be found on the Google play
    store or the iTunes App Store. Both Apps are Free. Currently this only works
    with 1 sensor field in the App. The light meter. 
    APP UUID: AD11CF40063F11E5BE3E0002A5D5C51B

    Private Characteristics
        Light Sensor:   BF3FBD80063F11E59E690002A5D5C501 
        Potentiometer:  BF3FBD80063F11E59E690002A5D5C502 
        Switch/LED:     BF3FBD80063F11E59E690002A5D5C503
        Temperature:    BF3FBD80063F11E59E690002A5D5C504
        Battery:        BF3FBD80063F11E59E690002A5D5C505

  Description:
    This file contains the source code for Communication implementation to the
    RN4871 BLE module. 
*******************************************************************************/

#include "rn4871.h"
#include <string.h>
#include <stdio.h>

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2017 released Microchip Technology Inc.  All rights reserved.

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
//DOM-IGNORE-END


/* This section lists the other files that are included in this file.
 */




/* ************************************************************************** */
/* ************************************************************************** */
/* Section: File Scope or Global Data                                         */
/* ************************************************************************** */
/* ************************************************************************** */


DRV_RN4871_DATA drvData;
SYS_MODULE_OBJ  RN4871TimerHandle;
SYS_TMR_INIT    initConfig;


#define MY_BUFFER_SIZE 150
char RadioReturn1[MY_BUFFER_SIZE];
char RadioReturn2[MY_BUFFER_SIZE];        
char ServerHandle[20];
char TempBuff[20];
char tempBuff2[20];

#define RESET_DELAY_LOW 100
#define RESET_DELAY_HIGH 200
#define COMMAND_TIMEOUT_DELAY 5000
#define TEST_COUNTER_OBJECT_DELAY 500

#define CONNECT_LED_ON
#define CONNECT_LED_OFF
#define ERROR_LED 

#define BUFFER_LEN_MAX 25

unsigned int    count;
unsigned int    total =0;

/* ************************************************************************** */
/* ************************************************************************** */

void RN4871_Initialize(void) 
{
    drvData.RXDone = false;
    drvData.TXDone = false;
    drvData.state = DRV_STATE_INIT;
    drvData.commandState = DRV_ENTER_COMMAND_MODE;
    drvData.TXbufferHandle = DRV_USART_BUFFER_HANDLE_INVALID;
    drvData.CommsTimeOut = false;
    drvData.SensorFlag = false;
}

void RN4871_Tasks(void)
{
    switch(drvData.state)
    {
        case(DRV_STATE_INIT): 
        {
//            BSP_RADIO_RESETOn();
            Nop();
            drvData.CommsTimeOut = false;
            drvData.sysTimerHandleMaster = SYS_TMR_ObjectCreate ( RESET_DELAY_LOW,NULL,TIMER_OBJECT_CALLBACK, SYS_TMR_FLAG_SINGLE);
            if (drvData.sysTimerHandleMaster  != SYS_TMR_HANDLE_INVALID)
            {
                BSP_RADIO_RESETOff(); // reseting bluetooth to know state. 
                drvData.state = DRV_STATE_RESET_MODULE_WAIT;
            }
            
            break;
        }
        
        case(DRV_STATE_RESET_MODULE_WAIT):
        {
            if ( drvData.CommsTimeOut == true )
            {
                //timeout here wanted. only time it should
                drvData.CommsTimeOut = false;
                BSP_RADIO_RESETOn();
                SYS_TMR_ObjectDelete(drvData.sysTimerHandleMaster);
                drvData.sysTimerHandleMaster = SYS_TMR_ObjectCreate ( RESET_DELAY_HIGH,NULL,TIMER_OBJECT_CALLBACK, SYS_TMR_FLAG_SINGLE);
                drvData.state = DRV_STATE_REBOOT_DELAY;
            }           
            break;
        }
        case (DRV_STATE_REBOOT_DELAY):
        {
            
            if (drvData.sysTimerHandleMaster  ==  SYS_TMR_HANDLE_INVALID)
            {
               drvData.sysTimerHandleMaster = SYS_TMR_ObjectCreate ( RESET_DELAY_HIGH,NULL,TIMER_OBJECT_CALLBACK, SYS_TMR_FLAG_SINGLE);
            }
            else
            {
                drvData.drvUSARTHandle_Master = DRV_USART_Open(DRV_USART_INDEX_0, DRV_IO_INTENT_NONBLOCKING|DRV_IO_INTENT_EXCLUSIVE | DRV_IO_INTENT_READWRITE);
                if ((drvData.drvUSARTHandle_Master != DRV_HANDLE_INVALID) )
                {
                    DRV_USART_BufferAddRead(drvData.drvUSARTHandle_Master, &drvData.RXbufferHandle, RadioReturn1, MY_BUFFER_SIZE); //%REBOOT%
                    
                }
                drvData.state = DRV_STATE_REBOOT_DELAY_WAIT;
            }
            
            break;
        }
        case (DRV_STATE_REBOOT_DELAY_WAIT):
        {
            if ( drvData.CommsTimeOut == false )
            {
                if ((drvData.RXbufferHandle == DRV_USART_BUFFER_HANDLE_INVALID ) )
                {
                    DRV_USART_BufferAddRead(drvData.drvUSARTHandle_Master, &drvData.RXbufferHandle, RadioReturn1, MY_BUFFER_SIZE); //%REBOOT%
                    break;
                }
                
                RadioReturn1[8] = 0;
                if (strcmp("%REBOOT%",RadioReturn1) ==0)
                {
                    DRV_USART_BufferRemove(drvData.RXbufferHandle);
                    SYS_TMR_ObjectDelete(drvData.sysTimerHandleMaster);
                    drvData.sysTimerHandleMaster = SYS_TMR_ObjectCreate ( COMMAND_TIMEOUT_DELAY,NULL,TIMER_OBJECT_CALLBACK, SYS_TMR_FLAG_SINGLE);
                    DRV_USART_BufferAddWrite(drvData.drvUSARTHandle_Master,&drvData.TXbufferHandle,"$$$",3);
                    SYS_MESSAGE("Enter Command mode: ");
                    memset(RadioReturn1, 0, 10);
                    drvData.state = DRV_STATE_REBOOT_DONE;
                }
                
            }  
            else 
            {
                drvData.state = DRV_STATE_ERROR_REBOOT;

            }
            break;
            
        }
        
        case(DRV_STATE_REBOOT_DONE):
        {
            if ( drvData.CommsTimeOut == false )
            {
                if ((drvData.TXbufferHandle == DRV_USART_BUFFER_HANDLE_INVALID ))
                {
                    DRV_USART_BufferAddWrite(drvData.drvUSARTHandle_Master,&drvData.TXbufferHandle,"$$$",3);
                    break;
                }
                DRV_USART_BufferAddRead(drvData.drvUSARTHandle_Master, &drvData.RXbufferHandle, RadioReturn1, MY_BUFFER_SIZE); //'CMD> '
                drvData.state = DRV_STATE_COMMAND_WAIT; 
            }  
            else 
            {
                drvData.state = DRV_STATE_ERROR_REBOOT;
            }
            break;
        }
        case(DRV_STATE_COMMAND_WAIT):
        {
            if (drvData.CommsTimeOut == false)
            {
                if ((drvData.RXbufferHandle == DRV_USART_BUFFER_HANDLE_INVALID ))
                {
                    DRV_USART_BufferAddRead(drvData.drvUSARTHandle_Master, &drvData.RXbufferHandle, RadioReturn1, MY_BUFFER_SIZE); //'CMD> '
                    break;
                }
                
                RadioReturn1[5] = 0;
                if (strcmp("CMD> ",RadioReturn1) ==0)
                {
                    SYS_MESSAGE("RN4871: PASS\r\n");
                    DRV_USART_BufferRemove(drvData.RXbufferHandle);
                    SYS_TMR_ObjectDelete(drvData.sysTimerHandleMaster);
                    drvData.sysTimerHandleMaster = SYS_TMR_ObjectCreate ( COMMAND_TIMEOUT_DELAY,NULL,TIMER_OBJECT_CALLBACK, SYS_TMR_FLAG_SINGLE);
                    DRV_USART_BufferAddWrite(drvData.drvUSARTHandle_Master,&drvData.TXbufferHandle,"SN,Harmony_BLE\r", 15);
                    SYS_MESSAGE("PIC32: Setting Name: ");
                    memset(RadioReturn1, 0, 10);
                    drvData.state = DRV_STATE_COMMAND_MODE;
                }
            }
            else
            {
                drvData.state = DRV_STATE_ERROR_REBOOT;
            }
                break;
        }
       
        case(DRV_STATE_COMMAND_MODE):
        {
            if ( drvData.CommsTimeOut == false )
            {
                if ((drvData.TXbufferHandle == DRV_USART_BUFFER_HANDLE_INVALID ))
                {
                    DRV_USART_BufferAddWrite(drvData.drvUSARTHandle_Master,&drvData.TXbufferHandle,"SN,Harmony_BLE\r", 15);                    
                    break;
                }

                DRV_USART_BufferAddRead(drvData.drvUSARTHandle_Master, &drvData.RXbufferHandle, RadioReturn1, MY_BUFFER_SIZE); //'CMD> '
                drvData.state = DRV_STATE_SET_NAME_WAIT;  
//                drvData.state = DRV_STATE_SET_UUID;
            }  
            else 
            {
                drvData.state = DRV_STATE_ERROR_REBOOT;
            }
            break;
        }


        case(DRV_STATE_SET_NAME_WAIT):
        {
            if (drvData.CommsTimeOut == false)
            {
                if ((drvData.RXbufferHandle == DRV_USART_BUFFER_HANDLE_INVALID ) )
                {
                    DRV_USART_BufferAddRead(drvData.drvUSARTHandle_Master, &drvData.RXbufferHandle, RadioReturn1, MY_BUFFER_SIZE); //''
                    break;
                }
                
                RadioReturn1[10] = 0;
                if (strcmp("AOK\r\nCMD> ",RadioReturn1) ==0)
                {
//                    DRV_USART_BufferRemove(drvData.RXbufferHandle);
                    if (DRV_USART_BufferRemove(drvData.RXbufferHandle) ==DRV_USART_BUFFER_RESULT_REMOVED_SUCCESFULLY)
                    {
                        SYS_MESSAGE("RN4871: PASS\r\n");
                        SYS_TMR_ObjectDelete(drvData.sysTimerHandleMaster);
                        drvData.sysTimerHandleMaster = SYS_TMR_ObjectCreate ( COMMAND_TIMEOUT_DELAY,NULL,TIMER_OBJECT_CALLBACK, SYS_TMR_FLAG_SINGLE);
                        DRV_USART_BufferAddWrite(drvData.drvUSARTHandle_Master,&drvData.TXbufferHandle,"PZ\r", 3);
                        SYS_MESSAGE("PIC32: Clearing BLE Services: ");
                        memset(RadioReturn1, 0, 20);
                        drvData.state = DRV_STATE_CLEAR_ALL_SERVICES_WAIT;
                    }

                }
            }
            else
            {
                drvData.state = DRV_STATE_ERROR_REBOOT;
            }
            break;
        }
        case(DRV_STATE_CLEAR_ALL_SERVICES_WAIT):
        {
            if (drvData.CommsTimeOut == false)
            {
                if ((drvData.TXbufferHandle == DRV_USART_BUFFER_HANDLE_INVALID ))
                {
                    DRV_USART_BufferAddWrite(drvData.drvUSARTHandle_Master,&drvData.TXbufferHandle,"PZ\r", 3);                    
                    break;
                }
                Nop();
                DRV_USART_BufferAddRead(drvData.drvUSARTHandle_Master, &drvData.RXbufferHandle, RadioReturn1, MY_BUFFER_SIZE);
                drvData.state = DRV_STATE_SET_UUID;
            }
            else
            {
                drvData.state = DRV_STATE_ERROR_REBOOT;
            }
            break;
        }
        

        
        
        
        case(DRV_STATE_SET_UUID):
        {
            if (drvData.CommsTimeOut == false)
            {
                if ((drvData.RXbufferHandle == DRV_USART_BUFFER_HANDLE_INVALID ) )
                {
                    DRV_USART_BufferAddRead(drvData.drvUSARTHandle_Master, &drvData.RXbufferHandle, RadioReturn1, MY_BUFFER_SIZE); //''
                    break;
                }
                
                RadioReturn1[10] = 0;
                if (strcmp("AOK\r\nCMD> ",RadioReturn1) ==0)
                {
                    SYS_MESSAGE("RN4871: PASS\r\n");
                    DRV_USART_BufferRemove(drvData.RXbufferHandle);
                    SYS_TMR_ObjectDelete(drvData.sysTimerHandleMaster);
                    drvData.sysTimerHandleMaster = SYS_TMR_ObjectCreate ( COMMAND_TIMEOUT_DELAY,NULL,TIMER_OBJECT_CALLBACK, SYS_TMR_FLAG_SINGLE);
                    DRV_USART_BufferAddWrite(drvData.drvUSARTHandle_Master,&drvData.TXbufferHandle,"PS,AD11CF40063F11E5BE3E0002A5D5C51B\r", 36);
                    SYS_MESSAGE("PIC32: Setting UUID: ");
                    memset(RadioReturn1, 0, 20); 
                    drvData.state = DRV_STATE_SET_UUID_WAIT;
                }
            }
            else
            {
                drvData.state = DRV_STATE_ERROR_REBOOT;
            }
            break;
        }
        
        case(DRV_STATE_SET_UUID_WAIT):
        {
            if (drvData.CommsTimeOut == false)
            {
                if ((drvData.TXbufferHandle == DRV_USART_BUFFER_HANDLE_INVALID ))
                {
                    DRV_USART_BufferAddWrite(drvData.drvUSARTHandle_Master,&drvData.TXbufferHandle,"PS,AD11CF40063F11E5BE3E0002A5D5C51B\r", 36);                   
                    break;
                }
//               
                DRV_USART_BufferAddRead(drvData.drvUSARTHandle_Master, &drvData.RXbufferHandle, RadioReturn1, MY_BUFFER_SIZE);
                drvData.state = DRV_STATE_SET_CHARACTERISTIC;
            }
            else
            {
                drvData.state = DRV_STATE_ERROR_REBOOT;
            }
            break;
        }
        
        case(DRV_STATE_SET_CHARACTERISTIC):
        {
            if (drvData.CommsTimeOut == false)
            {
                if ((drvData.RXbufferHandle == DRV_USART_BUFFER_HANDLE_INVALID ) )
                {
                    DRV_USART_BufferAddRead(drvData.drvUSARTHandle_Master, &drvData.RXbufferHandle, RadioReturn1, MY_BUFFER_SIZE); //''
                    break;
                }

                if (strcmp("AOK\r\nCMD> ",RadioReturn1) ==0)
                {
                    SYS_MESSAGE("RN4871: PASS\r\n");
                    DRV_USART_BufferRemove(drvData.RXbufferHandle);
                    SYS_TMR_ObjectDelete(drvData.sysTimerHandleMaster);
                    drvData.sysTimerHandleMaster = SYS_TMR_ObjectCreate ( COMMAND_TIMEOUT_DELAY,NULL,TIMER_OBJECT_CALLBACK, SYS_TMR_FLAG_SINGLE);
                    DRV_USART_BufferAddWrite(drvData.drvUSARTHandle_Master,&drvData.TXbufferHandle,"PC,BF3FBD80063F11E59E690002A5D5C501,10,02\r", 42);
                    SYS_MESSAGE("PIC32: Setting Characteristic");
                    memset(RadioReturn1, 0, 20);
                    drvData.state = DRV_STATE_SET_CHARACTERISTIC_WAIT;
                }
            }
            else
            {
                drvData.state = DRV_STATE_ERROR_REBOOT;
            }
            break;
        }
        
        case(DRV_STATE_SET_CHARACTERISTIC_WAIT):
        {
            if (drvData.CommsTimeOut == false)
            {
                if ((drvData.TXbufferHandle == DRV_USART_BUFFER_HANDLE_INVALID ))
                {
                    DRV_USART_BufferAddWrite(drvData.drvUSARTHandle_Master,&drvData.TXbufferHandle,"PC,BF3FBD80063F11E59E690002A5D5C501,10,02\r", 42);                   
                    break;
                }
                DRV_USART_BufferAddRead(drvData.drvUSARTHandle_Master, &drvData.RXbufferHandle, RadioReturn1, MY_BUFFER_SIZE);
                drvData.state = DRV_STATE_SOFT_REBOOT;
            }
            else
            {
                drvData.state = DRV_STATE_ERROR_REBOOT;
            }
            break;
        }
        
        case(DRV_STATE_SOFT_REBOOT):
        {
            if (drvData.CommsTimeOut == false)
            {
                if ((drvData.RXbufferHandle == DRV_USART_BUFFER_HANDLE_INVALID ))
                {
                    DRV_USART_BufferAddRead(drvData.drvUSARTHandle_Master, &drvData.RXbufferHandle, RadioReturn1, MY_BUFFER_SIZE);
                    break;
                }
                
                if (strcmp("AOK\r\nCMD> ",RadioReturn1) ==0)
                {
                    SYS_MESSAGE("RN4871: PASS\r\n");
                    DRV_USART_BufferRemove(drvData.RXbufferHandle);
                    SYS_TMR_ObjectDelete(drvData.sysTimerHandleMaster);
                    drvData.sysTimerHandleMaster = SYS_TMR_ObjectCreate ( COMMAND_TIMEOUT_DELAY,NULL,TIMER_OBJECT_CALLBACK, SYS_TMR_FLAG_SINGLE);
                    DRV_USART_BufferAddWrite(drvData.drvUSARTHandle_Master,&drvData.TXbufferHandle,"R,1\r", 4);
                    SYS_MESSAGE("PIC32: Setting Soft Reboot: ");
                    memset(RadioReturn1, 0, 20);
                    drvData.state = DRV_STATE_SOFT_REBOOT_WAIT;
                }
                
            }
            else
            {
                drvData.state = DRV_STATE_ERROR_REBOOT;
            }
            break;
        }
        /*****/

        case (DRV_STATE_SOFT_REBOOT_WAIT):
        {
            if (drvData.CommsTimeOut == false)
            { 
                if ((drvData.TXbufferHandle == DRV_USART_BUFFER_HANDLE_INVALID ))
                {
                    DRV_USART_BufferAddWrite(drvData.drvUSARTHandle_Master,&drvData.TXbufferHandle,"R,1\r", 4);                   
                    break;
                }
                DRV_USART_BufferAddRead(drvData.drvUSARTHandle_Master, &drvData.RXbufferHandle, RadioReturn1, MY_BUFFER_SIZE);
                drvData.state = DRV_STATE_REENTER_COMMAND;
                
            }
            else
            {
                drvData.state = DRV_STATE_ERROR_REBOOT;
            }
            break;
            
        }
        
        case(DRV_STATE_REENTER_COMMAND):
        {
            if (drvData.CommsTimeOut == false)
            {
                if ((drvData.RXbufferHandle == DRV_USART_BUFFER_HANDLE_INVALID ))
                {
                    DRV_USART_BufferAddRead(drvData.drvUSARTHandle_Master, &drvData.RXbufferHandle, RadioReturn1, MY_BUFFER_SIZE);                   
                    break;
                }
                
                if (strcmp("Rebooting\r\n",RadioReturn1) ==0)
                {
                    SYS_MESSAGE("RN4871:....");
                    DRV_USART_BufferRemove(drvData.RXbufferHandle);
                    SYS_TMR_ObjectDelete(drvData.sysTimerHandleMaster);
                    drvData.sysTimerHandleMaster = SYS_TMR_ObjectCreate ( COMMAND_TIMEOUT_DELAY,NULL,TIMER_OBJECT_CALLBACK, SYS_TMR_FLAG_SINGLE);
                    memset(RadioReturn1, 0, 20);
                    // need to schedule a new read to handle "%REBOOTING%
                    DRV_USART_BufferAddRead(drvData.drvUSARTHandle_Master, &drvData.RXbufferHandle, RadioReturn1, MY_BUFFER_SIZE);
                    drvData.state = DRV_STATE_NEW_READ;
                }
                
            }
            else
            {
                drvData.state = DRV_STATE_ERROR_REBOOT;
            }
            break;
        }
        
        case (DRV_STATE_NEW_READ):
        {
            if (drvData.CommsTimeOut == false)
            {
                if ((drvData.RXbufferHandle == DRV_USART_BUFFER_HANDLE_INVALID ))
                {
                    DRV_USART_BufferAddRead(drvData.drvUSARTHandle_Master, &drvData.RXbufferHandle, RadioReturn1, MY_BUFFER_SIZE);                   
                    break;
                }
                
                if (strcmp("%REBOOT%",RadioReturn1) ==0)
                {
                    SYS_MESSAGE("RN4871: PASS\r\n");
                    DRV_USART_BufferRemove(drvData.RXbufferHandle);
                    SYS_TMR_ObjectDelete(drvData.sysTimerHandleMaster);
                    drvData.sysTimerHandleMaster = SYS_TMR_ObjectCreate ( COMMAND_TIMEOUT_DELAY,NULL,TIMER_OBJECT_CALLBACK, SYS_TMR_FLAG_SINGLE);
                    DRV_USART_BufferAddWrite(drvData.drvUSARTHandle_Master,&drvData.TXbufferHandle,"$$$", 3);
                    SYS_MESSAGE("PIC32: Enter Command Mode: ");
                    memset(RadioReturn1, 0, 20);
                    drvData.state = DRV_STATE_REENTER_COMMAND_WAIT;
                }
                
            }
            else
            {
                drvData.state = DRV_STATE_ERROR_REBOOT;
            }
            break;
        }
        
        case(DRV_STATE_REENTER_COMMAND_WAIT):
        {
             if (drvData.CommsTimeOut == false)
            { 
                if ((drvData.TXbufferHandle == DRV_USART_BUFFER_HANDLE_INVALID ))
                {
                    DRV_USART_BufferAddWrite(drvData.drvUSARTHandle_Master,&drvData.TXbufferHandle,"$$$", 3);                   
                    break;
                }
                DRV_USART_BufferAddRead(drvData.drvUSARTHandle_Master, &drvData.RXbufferHandle, RadioReturn1, MY_BUFFER_SIZE);
                drvData.state = DRV_STATE_ADVERTISEMENT_CLEAR;
                
            }
            else
            {
                drvData.state = DRV_STATE_ERROR_REBOOT;
            }
            break;
        }
        
        case(DRV_STATE_ADVERTISEMENT_CLEAR):
        {
            if (drvData.CommsTimeOut == false)
            {
                if ((drvData.RXbufferHandle == DRV_USART_BUFFER_HANDLE_INVALID ))
                {
                    DRV_USART_BufferAddRead(drvData.drvUSARTHandle_Master, &drvData.RXbufferHandle, RadioReturn1, MY_BUFFER_SIZE);                   
                    break;
                }
                
                if (strcmp("CMD> ",RadioReturn1) ==0)
                {
                    SYS_MESSAGE("RN4871: PASS\r\n");
                    DRV_USART_BufferRemove(drvData.RXbufferHandle);
                    SYS_TMR_ObjectDelete(drvData.sysTimerHandleMaster);
                    drvData.sysTimerHandleMaster = SYS_TMR_ObjectCreate ( COMMAND_TIMEOUT_DELAY,NULL,TIMER_OBJECT_CALLBACK, SYS_TMR_FLAG_SINGLE);
                    DRV_USART_BufferAddWrite(drvData.drvUSARTHandle_Master,&drvData.TXbufferHandle,"IA,Z\r", 5);
                    SYS_MESSAGE("PIC32: Clearing Advertisements: ");
                    memset(RadioReturn1, 0, 20);
                    drvData.state = DRV_STATE_ADVERTISEMENT_CLEAR_WAIT;
                }
                
            }
            else
            {
                drvData.state = DRV_STATE_ERROR_REBOOT;
            }
            break;
        }
        
        
        case(DRV_STATE_ADVERTISEMENT_CLEAR_WAIT):
        {
            if (drvData.CommsTimeOut == false)
            { 
                if ((drvData.TXbufferHandle == DRV_USART_BUFFER_HANDLE_INVALID ))
                {
                    DRV_USART_BufferAddWrite(drvData.drvUSARTHandle_Master,&drvData.TXbufferHandle,"IA,Z\r", 5);                   
                    break;
                }
                DRV_USART_BufferAddRead(drvData.drvUSARTHandle_Master, &drvData.RXbufferHandle, RadioReturn1, MY_BUFFER_SIZE);
                drvData.state = DRV_STATE_ADVERTISEMENT_SET;
                
            }
            else
            {
                drvData.state = DRV_STATE_ERROR_REBOOT;
            }
            break;
        }
        
        case(DRV_STATE_ADVERTISEMENT_SET):
        {
            if (drvData.CommsTimeOut == false)
            {
                if ((drvData.RXbufferHandle == DRV_USART_BUFFER_HANDLE_INVALID ))
                {
                    DRV_USART_BufferAddRead(drvData.drvUSARTHandle_Master, &drvData.RXbufferHandle, RadioReturn1, MY_BUFFER_SIZE);                   
                    break;
                }
                
                if (strcmp("AOK\r\nCMD> ",RadioReturn1) ==0)
                {
                    SYS_MESSAGE("RN4871: PASS\r\n");
                    DRV_USART_BufferRemove(drvData.RXbufferHandle);
                    SYS_TMR_ObjectDelete(drvData.sysTimerHandleMaster);
                    drvData.sysTimerHandleMaster = SYS_TMR_ObjectCreate ( COMMAND_TIMEOUT_DELAY,NULL,TIMER_OBJECT_CALLBACK, SYS_TMR_FLAG_SINGLE);
                    DRV_USART_BufferAddWrite(drvData.drvUSARTHandle_Master,&drvData.TXbufferHandle,"IA,FF,CD00FE14AD11CF40063F11E5BE3E0002A5D5C51B\r", 47);
                    SYS_MESSAGE("PIC32: Setting new advertisements: ");
                    memset(RadioReturn1, 0, 20);
                    drvData.state = DRV_STATE_ADVERTISEMENT;
                }
                
            }
            else
            {
                drvData.state = DRV_STATE_ERROR_REBOOT;
            }
            break;
        }
        
        case(DRV_STATE_ADVERTISEMENT):
        {
            if (drvData.CommsTimeOut == false)
            { 
                if ((drvData.TXbufferHandle == DRV_USART_BUFFER_HANDLE_INVALID ))
                {
                    DRV_USART_BufferAddWrite(drvData.drvUSARTHandle_Master,&drvData.TXbufferHandle,"IA,FF,CD00FE14AD11CF40063F11E5BE3E0002A5D5C51B\r", 47);                   
                    break;
                }
                DRV_USART_BufferAddRead(drvData.drvUSARTHandle_Master, &drvData.RXbufferHandle, RadioReturn1, MY_BUFFER_SIZE);
                drvData.state = DRV_STATE_ADVERTISEMENT_WAIT;
                
            }
            else
            {
                drvData.state = DRV_STATE_ERROR_REBOOT;
            }
            break;
        }
        
        
        case(DRV_STATE_ADVERTISEMENT_WAIT):
        {
            if (drvData.CommsTimeOut == false)
            {
                if ((drvData.RXbufferHandle == DRV_USART_BUFFER_HANDLE_INVALID ))
                {
                    DRV_USART_BufferAddRead(drvData.drvUSARTHandle_Master, &drvData.RXbufferHandle, RadioReturn1, MY_BUFFER_SIZE);                   
                    break;
                }
                
                if (strcmp("AOK\r\nCMD> ",RadioReturn1) ==0)
                {
                    SYS_MESSAGE("RN4871: PASS\r\n");
                    DRV_USART_BufferRemove(drvData.RXbufferHandle);
                    SYS_TMR_ObjectDelete(drvData.sysTimerHandleMaster);
                    drvData.sysTimerHandleMaster = SYS_TMR_ObjectCreate ( COMMAND_TIMEOUT_DELAY,NULL,TIMER_OBJECT_CALLBACK, SYS_TMR_FLAG_SINGLE);
                    DRV_USART_BufferAddWrite(drvData.drvUSARTHandle_Master,&drvData.TXbufferHandle,"A\r", 2);
                    SYS_MESSAGE("PCI32: Advertise Now: ");
                    memset(RadioReturn1, 0, 20);
                    drvData.state = DRV_STATE_LIST;
                }
                
            }
            else
            {
                drvData.state = DRV_STATE_ERROR_REBOOT;
            }
            break;
        }
        
        case(DRV_STATE_LIST):
        {
            if (drvData.CommsTimeOut == false)
            { 
                if ((drvData.TXbufferHandle == DRV_USART_BUFFER_HANDLE_INVALID ))
                {
                    DRV_USART_BufferAddWrite(drvData.drvUSARTHandle_Master,&drvData.TXbufferHandle,"A\r", 2);                   
                    break;
                }
                DRV_USART_BufferAddRead(drvData.drvUSARTHandle_Master, &drvData.RXbufferHandle, RadioReturn1, MY_BUFFER_SIZE);
                drvData.state = DRV_STATE_LIST_WAIT;
                
            }
            else
            {
                drvData.state = DRV_STATE_ERROR_REBOOT;
            }
            break;
        }
        
        
        case(DRV_STATE_LIST_WAIT):
        {
            if (drvData.CommsTimeOut == false)
            {
                if ((drvData.RXbufferHandle == DRV_USART_BUFFER_HANDLE_INVALID ))
                {
                    DRV_USART_BufferAddRead(drvData.drvUSARTHandle_Master, &drvData.RXbufferHandle, RadioReturn1, MY_BUFFER_SIZE);                   
                    break;
                }
                
                if (strcmp("AOK\r\nCMD> ",RadioReturn1) ==0)
                {
                    SYS_MESSAGE("RN4871: PASS\r\n");
                    DRV_USART_BufferRemove(drvData.RXbufferHandle);
                    SYS_TMR_ObjectDelete(drvData.sysTimerHandleMaster);
                    drvData.sysTimerHandleMaster = SYS_TMR_ObjectCreate ( COMMAND_TIMEOUT_DELAY,NULL,TIMER_OBJECT_CALLBACK, SYS_TMR_FLAG_SINGLE);
                    DRV_USART_BufferAddWrite(drvData.drvUSARTHandle_Master,&drvData.TXbufferHandle,"LS\r", 3);
                    SYS_MESSAGE("PIC32: Get Server List: ");
                    memset(RadioReturn1, 0, MY_BUFFER_SIZE);
                    drvData.state = DRV_STATE_LIST_READ;
                }
                
            }
            else
            {
                drvData.state = DRV_STATE_ERROR_REBOOT;
            }
            break;
        }
        
        case(DRV_STATE_LIST_READ):
        {
            if (drvData.CommsTimeOut == false)
            { 
                if ((drvData.TXbufferHandle == DRV_USART_BUFFER_HANDLE_INVALID ))
                {
                    DRV_USART_BufferAddWrite(drvData.drvUSARTHandle_Master,&drvData.TXbufferHandle,"LS\r", 3);                   
                    break;
                }
                DRV_USART_BufferAddRead(drvData.drvUSARTHandle_Master, &drvData.RXbufferHandle, RadioReturn1, MY_BUFFER_SIZE);
                drvData.state = DRV_STATE_LIST_READ_PROCESS;
                
            }
            else
            {
                drvData.state = DRV_STATE_ERROR_REBOOT;
            }
            break;
        }
        
        case(DRV_STATE_LIST_READ_PROCESS):
        {
            size_t CurrentCount;
            if (drvData.CommsTimeOut == false)
            {
                if ((drvData.RXbufferHandle == DRV_USART_BUFFER_HANDLE_INVALID ))
                {
                    DRV_USART_BufferAddRead(drvData.drvUSARTHandle_Master, &drvData.RXbufferHandle, RadioReturn1, MY_BUFFER_SIZE);                   
                    break;
                }
                
                CurrentCount = DRV_USART_BufferProcessedSizeGet( drvData.RXbufferHandle);
                if (strncmp("\r\nEND\r\nCMD> ",&RadioReturn1[CurrentCount -12],12) ==0)
                {
                    RadioReturn1[CurrentCount +1] = 0;

                    DRV_USART_BufferRemove(drvData.RXbufferHandle);
                    SYS_TMR_ObjectDelete(drvData.sysTimerHandleMaster); // stop command time out counter
                    drvData.state = DRV_STATE_TOKEN;
                }
                    Nop();
            }
            else
            {
                drvData.state = DRV_STATE_ERROR_REBOOT;
            }
            break;
        }
        
        case(DRV_STATE_TOKEN):
        {
            
            SYS_PRINT("\r\nRN4871: %s",RadioReturn1);
            strcpy(TempBuff, "SHW,");
            memcpy(ServerHandle,&RadioReturn1[69],4);
            strcat(TempBuff,ServerHandle);
            strcat(TempBuff,",");//SHW,xxxx,
            memcpy(ServerHandle,TempBuff,9); //save server handle
            SYS_PRINT("\r\nRN4871: %s",ServerHandle);
            memset(RadioReturn1, 0, MY_BUFFER_SIZE);//clear out buffer
            DRV_USART_BufferAddRead(drvData.drvUSARTHandle_Master, &drvData.RXbufferHandle, RadioReturn1, MY_BUFFER_SIZE);                   

            drvData.state = DRV_STATE_WAIT_FOR_CONNECTION;

        }
        

        
        case (DRV_STATE_WAIT_FOR_CONNECTION):
        {
            if ((drvData.RXbufferHandle == DRV_USART_BUFFER_HANDLE_INVALID ))
            {
                DRV_USART_BufferAddRead(drvData.drvUSARTHandle_Master, &drvData.RXbufferHandle, RadioReturn1, MY_BUFFER_SIZE);                   
                break;
            }
            BSP_LEDOn(BSP_LED_BLUE);
            if (strcmp("%CONNECT,",RadioReturn1) ==0)
            {
                SYS_PRINT("\r\nRN4871: %s",RadioReturn1);
                //here we are connected. now we can send data
                //any data sent before %CONNECT, is ignored
                BSP_LEDOff(BSP_LED_BLUE);
                BSP_LEDOn(BSP_LED_GREEN);
                DRV_USART_BufferRemove(drvData.RXbufferHandle);
                memset(RadioReturn1, 0, MY_BUFFER_SIZE);
                DRV_USART_BufferAddRead(drvData.drvUSARTHandle_Master, &drvData.RXbufferHandle, RadioReturn1, MY_BUFFER_SIZE);
                drvData.state = DRV_STATE_IDLE;
            }
            
            break;
        }
        
        case (DRV_STATE_IDLE):
        {
            //line blow is a test vector only. data will ramp up then down, then repeat
            drvData.sysTimerHandleMaster = SYS_TMR_ObjectCreate ( TEST_COUNTER_OBJECT_DELAY ,NULL,TEST_COUNTER_OBJECT_CALLBACK, SYS_TMR_FLAG_PERIODIC);

            if ((drvData.RXbufferHandle == DRV_USART_BUFFER_HANDLE_INVALID ))
            {
                DRV_USART_BufferAddRead(drvData.drvUSARTHandle_Master, &drvData.RXbufferHandle, RadioReturn1, MY_BUFFER_SIZE);                   
                break;
            }
            if (strcmp("%DISCONNECT%",RadioReturn1) ==0)
            {
                //here we are connected. now we can send data
                //any data sent before %CONNECT, is ignored
                BSP_LEDOff(BSP_LED_GREEN);
                BSP_LEDOn(BSP_LED_BLUE);
                DRV_USART_BufferRemove(drvData.RXbufferHandle);
                memset(RadioReturn1, 0, MY_BUFFER_SIZE);
                DRV_USART_BufferAddRead(drvData.drvUSARTHandle_Master, &drvData.RXbufferHandle, RadioReturn1, MY_BUFFER_SIZE);
                drvData.state = DRV_STATE_WAIT_FOR_CONNECTION;
            }
            else if(strcmp("AOK\r\nCMD> ",RadioReturn1) ==0)
            {
                //message was sent and accepted 
                
            }
            else if(strcmp("ERR",RadioReturn1) ==0)
            {
                //message sent and failed to process
                
            }
            Nop();
            break;
        }
        
        case (DRV_STATE_ERROR_REBOOT):
        {
            BSP_LEDToggle(BSP_LED_RED);
            DRV_USART_Close(drvData.drvUSARTHandle_Master);
            SYS_TMR_ObjectDelete(drvData.sysTimerHandleMaster);
            SYS_MESSAGE("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\r\n");
            SYS_MESSAGE("~ AN ERROR HAS OCCURED WITH THE RN4871  ~\r\n");
            SYS_MESSAGE("~ DRIVER HAS TIMED OUT OR RESPONSE FROM ~\r\n");
            SYS_MESSAGE("~ RADIO IS NOT CORRECT & CAN'T RECOVER! ~\r\n");
            SYS_MESSAGE("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\r\n");
            SYS_MESSAGE("~            REBOOTING RADIO            ~\r\n");
            SYS_MESSAGE("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\r\n");
            drvData.state = DRV_STATE_INIT;


            break;
        }
        
        
        default:
        {
            break;
        }
    }//end of switch
            
}
/**********************************************************************************************************/

void TIMER_OBJECT_CALLBACK(uintptr_t context, uint32_t currTick)
{
    drvData.CommsTimeOut = true;
}

int TestValue = 0;
int UpDownCount =0;
//char TempBuff[20];
void TEST_COUNTER_OBJECT_CALLBACK(uintptr_t context, uint32_t currTick)
{
    // this function is a test vector only
    char tempBuff2[20];
    memset(TempBuff, 0, 20);
    memset(tempBuff2, 0, 20);
    switch (UpDownCount)
    {
        case (0):
        {
            TestValue = TestValue+10;
            break;
        }
        case (1):
        {
            
            TestValue=TestValue-10;
            break;
        }
        
    }
    if (TestValue >= 3300)
    {
        UpDownCount=1;
    }
    else if(TestValue <= 0)
    {
        UpDownCount=0;
    }

    sprintf(tempBuff2,"%04X",TestValue);
    memcpy(TempBuff,ServerHandle,9); // "SHW,xxxx," xxxx= server handle from above

    memcpy(&TempBuff[9],tempBuff2,4);

    strcat(TempBuff,"\r"); //"SHW,xxxx,xxxx\r"
    DRV_USART_BufferAddWrite(drvData.drvUSARTHandle_Master,&drvData.TXbufferHandle,TempBuff, 14);

    
}

void SEND_SENSOR_DATA(int value)
{

    Nop();
    if (drvData.state == DRV_STATE_IDLE)
    {
        Nop();

        memset(TempBuff, 0, 20);
        memset(tempBuff2, 0, 20);
        Nop();
        sprintf(tempBuff2,"%04X",value);
        memcpy(TempBuff,ServerHandle,9); //SHW, + server handle "SHW,xxxx," xxxx = Server handle

        memcpy(&TempBuff[9],tempBuff2,4); //"SHW,xxxx,yyyy" yyyy =  passed in data
        strcat(TempBuff,"\r"); //"SHW,xxxx,data\r"
        Nop();
        DRV_USART_BufferAddWrite(drvData.drvUSARTHandle_Master,&drvData.TXbufferHandle,TempBuff, 14);
        drvData.SensorFlag = true;

    }
        
}

/* *****************************************************************************
 End of File
 */
