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
#define HALF_BUFF_SIZE  128
#define ADC_NUM_BUF_IN_1_SECOND 6000000/128//3ch * 2000000 Msps/128 
 
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
BSP_LED LED_Object_ID[3] = {BSP_LED_1, BSP_LED_2, BSP_LED_3};
uint8_t ADC_State = 0;
uint32_t One_Second_Cnt = 0;
uint32_t ADC_Average = 0;
uint32_t __attribute__((coherent)) adc_buf[ADC_MAX_CHANNEL][256];
SYS_DMA_CHANNEL_HANDLE channelHandle0, channelHandle1, channelHandle2;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

void APP_ReadComplete (void *handle)
{
    appData.rdComplete = true;
}

void APP_WriteComplete (void *handle)
{
    appData.wrComplete = true;
}

void APP_Reset ()
{
    appData.rdComplete = true;
    appData.wrComplete = true;
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************
/* TODO:  Add any necessary local functions.
*/

void ADC_Task(void)
{
    int i;
    uint32_t *ADC_Buff_ptr;

    ADC_Average = 0;
    if(appData.ADC_Buf_Status[ADC_State] ==  ADC_DMA_1ST_BUF_FULL)
    {
        One_Second_Cnt++;
        BSP_LEDStateSet(LED_Object_ID[ADC_State], BSP_LED_STATE_ON);
        appData.ADC_Buf_Status[ADC_State] = ADC_DMA_BUFF_EMPTY;
        //process 1st buffer here
        ADC_Buff_ptr = (uint32_t*)&adc_buf[ADC_State][0];
        ADC_Average = 0;
        for (i = 0; i < HALF_BUFF_SIZE; i++)
        {
            ADC_Average += *ADC_Buff_ptr;
            ADC_Buff_ptr++;
        }
        appData.ADC_Average[ADC_State] = ADC_Average / HALF_BUFF_SIZE;      
    }
    else if(appData.ADC_Buf_Status[ADC_State] ==  ADC_DMA_2ND_BUF_FULL)
    {
        One_Second_Cnt++;
        BSP_LEDStateSet(LED_Object_ID[ADC_State], BSP_LED_STATE_OFF);
        appData.ADC_Buf_Status[ADC_State] = ADC_DMA_BUFF_EMPTY;
        
        //process 2nd half buffer
        ADC_Buff_ptr = (uint32_t*)&adc_buf[ADC_State][HALF_BUFF_SIZE];
        ADC_Average = 0;        
        for (i = 0; i < HALF_BUFF_SIZE; i++)
        {
            ADC_Average += *ADC_Buff_ptr;
            ADC_Buff_ptr++;
        }
        appData.ADC_Average[ADC_State] = ADC_Average / HALF_BUFF_SIZE;      
    }

    if ( One_Second_Cnt > ADC_NUM_BUF_IN_1_SECOND)
    {
        SYS_PRINT("CH_A = %d, CH_B = %d, CH_C = %d \r\n\r\n", 
                appData.ADC_Average[0], 
                appData.ADC_Average[1],
                appData.ADC_Average[2]);   
        One_Second_Cnt = 0;
    }

    //increment to next ADC state
    ADC_State++;
    if (ADC_State >= ADC_MAX_CHANNEL)
    {
        ADC_State = 0;
    }
}  



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
    uint8_t i;
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    appData.wrComplete = true;
    appData.rdComplete = true;
    
    for(i = 0; i < ADC_MAX_CHANNEL; i++)
    {
        appData.ADC_Buf_Status[ADC_MAX_CHANNEL] = ADC_DMA_BUFF_EMPTY;
    }
    
     /* Allocate the DMA channels */
     channelHandle0 = SYS_DMA_ChannelAllocate(DMA_CHANNEL_0);
     channelHandle1 = SYS_DMA_ChannelAllocate(DMA_CHANNEL_1);
     channelHandle2 = SYS_DMA_ChannelAllocate(DMA_CHANNEL_2);
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    SYS_STATUS consoleStatus;
 
    consoleStatus = SYS_CONSOLE_Status(sysObj.sysConsole0);

    //Do not proceed in the current app state unless the console is ready
    if (consoleStatus != SYS_STATUS_READY)
    {
        if (consoleStatus == SYS_STATUS_ERROR)
        {
            APP_Reset();
            SYS_CONSOLE_Flush(SYS_CONSOLE_INDEX_0);
        }

        return;
    }

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
            
            if (SYS_CONSOLE_Status(sysObj.sysConsole0) == SYS_STATUS_READY)
            {
                SYS_CONSOLE_RegisterCallback(SYS_CONSOLE_INDEX_0, APP_ReadComplete, SYS_CONSOLE_EVENT_READ_COMPLETE);
                SYS_CONSOLE_RegisterCallback(SYS_CONSOLE_INDEX_0, APP_WriteComplete, SYS_CONSOLE_EVENT_WRITE_COMPLETE);
                appData.state = APP_STATE_SPIN;
                SYS_PRINT("\r\nStarting the test.\r\n");
                
                //initial enable DMA
                // setup DMA transfer for half and full buffer transfer to provide ping pong buffer transfer
                SYS_DMA_ChannelTransferAdd(channelHandle0, (const void *)&ADCDATA0, 4, &adc_buf[0][0], 1024, 4 );
                SYS_DMA_ChannelSetup(channelHandle0,SYS_DMA_CHANNEL_OP_MODE_AUTO, DMA_TRIGGER_0);
                PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, DMA_CHANNEL_0, DMA_INT_DESTINATION_DONE);
                PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, DMA_CHANNEL_0, DMA_INT_DESTINATION_HALF_FULL);
                PLIB_DMA_ChannelXINTSourceEnable(DMA_ID_0, DMA_CHANNEL_0, DMA_INT_DESTINATION_HALF_FULL);
                PLIB_DMA_ChannelXINTSourceEnable(DMA_ID_0, DMA_CHANNEL_0, DMA_INT_DESTINATION_DONE);
                SYS_DMA_ChannelEnable(channelHandle0);

                SYS_DMA_ChannelTransferAdd(channelHandle1, (const void *)&ADCDATA1, 4, &adc_buf[1][0], 1024, 4 );
                SYS_DMA_ChannelSetup(channelHandle1,SYS_DMA_CHANNEL_OP_MODE_AUTO, DMA_TRIGGER_1);
                PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, DMA_CHANNEL_1, DMA_INT_DESTINATION_DONE);
                PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, DMA_CHANNEL_1, DMA_INT_DESTINATION_HALF_FULL);
                PLIB_DMA_ChannelXINTSourceEnable(DMA_ID_0, DMA_CHANNEL_1, DMA_INT_DESTINATION_HALF_FULL);
                PLIB_DMA_ChannelXINTSourceEnable(DMA_ID_0, DMA_CHANNEL_1, DMA_INT_DESTINATION_DONE);
                SYS_DMA_ChannelEnable(channelHandle1);

                SYS_DMA_ChannelTransferAdd(channelHandle2, (const void *)&ADCDATA2, 4, &adc_buf[2][0], 1024, 4 );
                SYS_DMA_ChannelSetup(channelHandle2,SYS_DMA_CHANNEL_OP_MODE_AUTO, DMA_TRIGGER_2);
                PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, DMA_CHANNEL_2, DMA_INT_DESTINATION_DONE);
                PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, DMA_CHANNEL_2, DMA_INT_DESTINATION_HALF_FULL);
                PLIB_DMA_ChannelXINTSourceEnable(DMA_ID_0, DMA_CHANNEL_2, DMA_INT_DESTINATION_HALF_FULL);
                PLIB_DMA_ChannelXINTSourceEnable(DMA_ID_0, DMA_CHANNEL_2, DMA_INT_DESTINATION_DONE);
                SYS_DMA_ChannelEnable(channelHandle2);
                
                //enable ADC channels
                DRV_ADC0_Open();
                DRV_ADC1_Open();
                DRV_ADC2_Open();
 
                 //Set data ready interrupt for triggering DMA data transfer.  
                PLIB_ADCHS_AnalogInputDataReadyInterruptEnable(DRV_ADC_ID_1, ADCHS_AN0);
                PLIB_ADCHS_AnalogInputDataReadyInterruptEnable(DRV_ADC_ID_1, ADCHS_AN1);
                PLIB_ADCHS_AnalogInputDataReadyInterruptEnable(DRV_ADC_ID_1, ADCHS_AN2);
                PLIB_ADCHS_AnalogInputDataReadyInterruptEnable(DRV_ADC_ID_1, ADCHS_AN3);
                PLIB_ADCHS_AnalogInputDataReadyInterruptEnable(DRV_ADC_ID_1, ADCHS_AN4);
                //enable timer to trigger ADC sample
                DRV_TMR0_Start();
                
            }
            break;
        case APP_STATE_SPIN:
        {
            ADC_Task();
        }
        break;
        
        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}
 

/*******************************************************************************
 End of File
 */
