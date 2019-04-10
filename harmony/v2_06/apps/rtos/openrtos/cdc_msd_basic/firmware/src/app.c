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

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************
/* This is the string that will written to the file */
 uint8_t writeData[20]  __attribute__((aligned(4))) = "Hello World ";;

/* This is the string that will written to the CDC device */
 
 /* This is the string that will written to the CDC device */
uint8_t prompt[10] __attribute__((coherent)) __attribute__((aligned(16)))  = "\r\nData : ";


/* Application MSD Task Object */
APP_MSD_DATA appMSDData __attribute__((coherent)) __attribute__((aligned(16))) ;;

/* Application CDC Task Object */
APP_CDC_DATA  appCDCData __attribute__((coherent)) __attribute__((aligned(16))) ;

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

static uint8_t  count = 0 ;

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************
USB_HOST_EVENT_RESPONSE APP_USBHostEventHandler 
(
    USB_HOST_EVENT event, 
    void * eventData,
    uintptr_t context
)
{
    /* This function is called by the USB Host whenever a USB Host Layer event
     * has occurred. In this example we only handle the device unsupported event
     * */

    switch (event)
    {
        case USB_HOST_EVENT_DEVICE_UNSUPPORTED:
            
            /* The attached device is not supported for some reason */
            break;
            
        default:
            break;
                    
    }
    
    return(USB_HOST_EVENT_RESPONSE_NONE);
}

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    APP_CDC_Initialize( &appCDCData );
    APP_MSD_Initialize ( &appMSDData);
     
}

void APP_SYSFSEventHandler(SYS_FS_EVENT event, void * eventData, uintptr_t context)
{
    switch(event)
    {
        case SYS_FS_EVENT_MOUNT:
            appMSDData.deviceIsConnected = true;
            vTaskResume(appData.taskHandleMSD);
            xSemaphoreGive(appMSDData.xSemaphoreBlockUsbMSDAttach );
            break;
            
        case SYS_FS_EVENT_UNMOUNT:
            appMSDData.deviceIsConnected = false;
            vTaskSuspend (appData.taskHandleMSD);
            appMSDData.state = APP_MSD_STATE_WAIT_FOR_DEVICE_ATTACH;
            break;
            
        default:
            break;
    }
}

/*************************************************
 * Application MSD Task Initialize Function.
 *************************************************/

void APP_MSD_Initialize (APP_MSD_DATA * appMSDData )
{
     appMSDData->state = APP_MSD_STATE_WAIT_FOR_DEVICE_ATTACH;
     appMSDData->deviceIsConnected = false;
}

/*************************************************
 * Application CDC Task Initialize Function.
 *************************************************/

void APP_CDC_Initialize ( APP_CDC_DATA *appCDCData )
{
      /* Initialize the application state machine */
    
    appCDCData->state =  APP_CDC_STATE_WAIT_FOR_DEVICE_ATTACH;
    appCDCData->cdcHostLineCoding.dwDTERate     = APP_HOST_CDC_BAUDRATE_SUPPORTED;
    appCDCData->cdcHostLineCoding.bDataBits     = (uint8_t)APP_HOST_CDC_NO_OF_DATA_BITS;
    appCDCData->cdcHostLineCoding.bParityType   = (uint8_t)APP_HOST_CDC_PARITY_TYPE;
    appCDCData->cdcHostLineCoding.bCharFormat   = (uint8_t)APP_HOST_CDC_STOP_BITS;
    appCDCData->controlLineState.dtr = 0;
    appCDCData->controlLineState.carrier = 0;
    appCDCData->deviceIsAttached = false;
    appCDCData->deviceWasDetached = false;
    appCDCData->readTransferDone = false;
    appCDCData->writeTransferDone = false;
    appCDCData->controlRequestDone = false;
}

void APP_USBHostCDCAttachEventListener(USB_HOST_CDC_OBJ cdcObj, uintptr_t context)
{
    /* This function gets called when the CDC device is attached. Update the
     * application data structure to let the application know that this device
     * is attached */
    
    appCDCData.deviceIsAttached = true;
    appCDCData.cdcObj = cdcObj;
    vTaskResume(appData.taskHandleCDC);
    xSemaphoreGive(appCDCData.xSemaphoreBlockUsbCDCAttach );
}
/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
     static bool blockAppTask = false;
    BaseType_t errStatus;
    
    if (blockAppTask == false)
    {
        errStatus = xTaskCreate((TaskFunction_t) APP_USB_Host_Open,
                "USB_AttachTask",
                USB_ATTACH_TASK_SIZE,
                NULL,
                APP_USB_HOST_OPEN_PRIO,
                NULL);
        /*dont proceed if Task was not created...*/
        if(errStatus != pdTRUE)
        {
            BSP_LEDOn ( APP_USB_LED_1 );
            BSP_LEDOn ( APP_USB_LED_2 );
            BSP_LEDOn ( APP_USB_LED_3 );
            while(1);
        }
        blockAppTask = true; 
    }
    
}
 
void APP_USB_Host_Open( )
{
    BaseType_t errStatus;
    static bool busEnable = false;

     errStatus = xTaskCreate((TaskFunction_t)  APP_MSD_Tasks,
        "MSD",
        USB_MSD_TASK_SIZE,
        NULL,
         APP_USB_MSD_TASK_PRIO ,
        &appData.taskHandleMSD);
     
     /*Don't proceed if Task was not created...*/
        if(errStatus != pdTRUE)
        {
            BSP_LEDOn ( APP_USB_LED_1 );
            BSP_LEDOn ( APP_USB_LED_2 );
            BSP_LEDOn ( APP_USB_LED_3 );
            while(1);
        }


    errStatus = xTaskCreate((TaskFunction_t) APP_CDC_Tasks,
        "CDC",
        USB_CDC_TASK_SIZE ,
        NULL,
         APP_USB_CDC_TASK_PRIO ,
        &appData.taskHandleCDC );
    
    /*Don't proceed if Task was not created...*/
        if(errStatus != pdTRUE)
        {
            BSP_LEDOn ( APP_USB_LED_1 );
            BSP_LEDOn ( APP_USB_LED_2 );
            BSP_LEDOn ( APP_USB_LED_3 );
            while(1);
        }


     /* Create the semaphore used to signal that USB Device is configured. */
    appCDCData.xSemaphoreBlockUsbCDCAttach = xSemaphoreCreateBinary();
    appMSDData.xSemaphoreBlockUsbMSDAttach = xSemaphoreCreateBinary();
    appCDCData.xSemaphoreCDCReadComplete = xSemaphoreCreateBinary();
     vTaskSuspend (appData.taskHandleMSD);
     vTaskSuspend (appData.taskHandleCDC);
    for(;;)
    {
        
        if ( busEnable == false )
        {
             /* Set the event handler and enable the bus */
            SYS_FS_EventHandlerSet(APP_SYSFSEventHandler, (uintptr_t)1);
            USB_HOST_EventHandlerSet(APP_USBHostEventHandler, 0 );
            USB_HOST_EventHandlerSet(APP_USBHostEventHandler, (uintptr_t)NULL);
            USB_HOST_CDC_AttachEventHandlerSet(APP_USBHostCDCAttachEventListener, (uintptr_t) 0);
            USB_HOST_BusEnable(0);
            
            busEnable = true ;
            
        }
        
        else if(USB_HOST_BusIsEnabled(0)) 
        {
              vTaskSuspend (NULL );
            
        }
         else
        {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }


    }
}


void APP_MSD_Tasks ( )
{
    /* The application task state machine */
    while( 1)
    {

        /* Block on thappMSDData.e binary semaphore given by USB Configure event */

    switch(appMSDData.state)
    {
        case APP_MSD_STATE_WAIT_FOR_DEVICE_ATTACH:

            /* Wait for device attach. The state machine will move
             * to the next state when the attach event
             * is received.  */
            if(appMSDData.deviceIsConnected)
            {
                BSP_LEDOn( APP_USB_LED_3 );
                BSP_LEDOff( APP_USB_LED_2 );
                appMSDData.state = APP_MSD_STATE_DEVICE_CONNECTED;
            }

            break;

        case APP_MSD_STATE_DEVICE_CONNECTED:

            /* Command request was successful. Wait for the
             * command request to complete */
            xSemaphoreTake(appMSDData.xSemaphoreBlockUsbMSDAttach , portMAX_DELAY );
            /* Device was connected. We can try mounting the disk */
            appMSDData.state = APP_MSD_STATE_OPEN_FILE;

            break;

        case APP_MSD_STATE_OPEN_FILE:

            xSemaphoreTake(appCDCData.xSemaphoreCDCReadComplete , portMAX_DELAY );
            /* Try opening the file for append */
            appMSDData.fileHandle = SYS_FS_FileOpen("/mnt/myDrive1/file.txt", (SYS_FS_FILE_OPEN_APPEND_PLUS));
            if(appMSDData.fileHandle == SYS_FS_HANDLE_INVALID)
            {
                /* Could not open the file. Error out*/
                appMSDData.state = APP_MSD_STATE_ERROR;

            }
            else
            {
                /* File opened successfully. Write to file */
                appMSDData.state = APP_MSD_STATE_WRITE_TO_FILE;

            }
            break;

        case APP_MSD_STATE_WRITE_TO_FILE:

          
           /* Try writing to the file */
            if (SYS_FS_FileWrite( appMSDData.fileHandle, (const void *) writeData, count ) == -1)
            {
                /* Write was not successful. Close the file
                 * and error out.*/
                SYS_FS_FileClose(appMSDData.fileHandle);
                appMSDData.state = APP_MSD_STATE_ERROR;

            }
            else
            {
                 SYS_FS_FileClose(appMSDData.fileHandle);
                /* We are done writing. Close the file */
                appMSDData.state = APP_MSD_STATE_IDLE;
            }

            break;

        
        case APP_MSD_STATE_IDLE:

            /* The appliction comes here when the demo
             * has completed successfully. Switch on
             * green LED. */

            BSP_LEDOn(APP_USB_LED_2);
            appMSDData.state = APP_MSD_STATE_OPEN_FILE;
            appCDCData.state =  APP_CDC_STATE_SEND_PROMPT_TO_DEVICE;
            
            break;

        case APP_MSD_STATE_ERROR:

            /* The appliction comes here when the demo
             * has failed. Switch on the red LED.*/

            BSP_LEDOn(APP_USB_LED_1);
          
            break;
        default:
            break;
    }
     vTaskDelay( 10 / portTICK_PERIOD_MS);
    }
}


/**********************************************
 * Application CDC Task Routine.
 ***********************************************/

void APP_CDC_Tasks()
{
    USB_HOST_CDC_RESULT result;
    uint8_t  temp;
    uint8_t  i;
    

    while(1)
    {
    switch (appCDCData.state)
    {
        case APP_CDC_STATE_WAIT_FOR_DEVICE_ATTACH:

            /* Wait for device attach. The state machine will move
             * to the next state when the USB_HOST_CDC_EVENT_ATTACH
             * is received. The application state is update in the
             * CDC Host event handler */
             /* In this state the application is waiting for the device to be
             * attached */
            if(appCDCData.deviceIsAttached)
            {
                /* A device is attached. We can open this device */
                appCDCData.state = APP_CDC_STATE_OPEN_DEVICE;
                appCDCData.deviceIsAttached = false;
            }

            break;
         case APP_CDC_STATE_OPEN_DEVICE:
            
            /* In this state the application opens the attached device */
            appCDCData.cdcHostHandle = USB_HOST_CDC_Open(appCDCData.cdcObj);
            if(appCDCData.cdcHostHandle != USB_HOST_CDC_HANDLE_INVALID)
            {
                /* The driver was opened successfully. Set the event handler
                 * and then go to the next state. */
                USB_HOST_CDC_EventHandlerSet(appCDCData.cdcHostHandle, APP_USBHostCDCEventHandler, (uintptr_t)0);
                appCDCData.state = APP_CDC_STATE_SET_LINE_CODING;
                xSemaphoreTake(appCDCData.xSemaphoreBlockUsbCDCAttach , portMAX_DELAY );
            }
            break;

            case APP_CDC_STATE_SET_LINE_CODING:
            
            /* Here we set the Line coding. The control request done flag will
             * be set to true when the control request has completed. */
            
            appCDCData.controlRequestDone = false;
            result = USB_HOST_CDC_ACM_LineCodingSet(appCDCData.cdcHostHandle, NULL, &appCDCData.cdcHostLineCoding);
            
            if(result == USB_HOST_CDC_RESULT_SUCCESS)
            {
                /* We wait for the set line coding to complete */
                appCDCData.state = APP_CDC_STATE_WAIT_FOR_SET_LINE_CODING;
            }
                            
            break;
            
            
       case APP_CDC_STATE_WAIT_FOR_SET_LINE_CODING:
            
            if(appCDCData.controlRequestDone)
            {
                if(appCDCData.controlRequestResult != USB_HOST_CDC_RESULT_SUCCESS)
                {
                    /* The control request was not successful. */
                    appCDCData.state = APP_CDC_STATE_ERROR;
                }
                else
                {
                    /* Next we set the Control Line State */
                    appCDCData.state = APP_CDC_STATE_SEND_SET_CONTROL_LINE_STATE;
                }
            }
            break;
            
        case APP_CDC_STATE_SEND_SET_CONTROL_LINE_STATE:
            
            /* Here we set the control line state */
            appCDCData.controlRequestDone = false;
            result = USB_HOST_CDC_ACM_ControlLineStateSet(appCDCData.cdcHostHandle, NULL, 
                    &appCDCData.controlLineState);
            
            if(result == USB_HOST_CDC_RESULT_SUCCESS)
            {
                /* We wait for the set line coding to complete */
                appCDCData.state = APP_CDC_STATE_WAIT_FOR_SET_CONTROL_LINE_STATE;
            }
            
            break;
       
        case APP_CDC_STATE_WAIT_FOR_SET_CONTROL_LINE_STATE:
            
            /* Here we wait for the control line state set request to complete */
            if(appCDCData.controlRequestDone)
            {
                if(appCDCData.controlRequestResult != USB_HOST_CDC_RESULT_SUCCESS)
                {
                    /* The control request was not successful. */
                    appCDCData.state = APP_CDC_STATE_ERROR;
                }
                else
                {
                    /* Next we set the Control Line State */
                    appCDCData.state = APP_CDC_STATE_SEND_PROMPT_TO_DEVICE;
                }
            }
            
            break;
            
           
        case APP_CDC_STATE_SEND_PROMPT_TO_DEVICE:

            /* In this state, the application sends a prompt
             * message to the attached CDC device. */
             appCDCData.writeTransferDone = false;
             BSP_LEDOff(APP_USB_LED_2);
             count = 0;

             for( i = 0; i < 15 ; i++ )
             {
                 writeData[i] = 0 ;
             }

             
            result = USB_HOST_CDC_Write(appCDCData.cdcHostHandle, NULL, prompt, 8);

            if(USB_HOST_CDC_RESULT_SUCCESS == result)
            {
                /* Command request was successful. Wait for the
                 * command request to complete */

               
                appCDCData.state = APP_CDC_STATE_WAIT_FOR_PROMPT_SEND_COMPLETE;
            }
            else
            {
                /* There was an error in the command request.
                 * Goto error state */

                appCDCData.state = APP_CDC_STATE_ERROR;
            }

            break;
            
            case APP_CDC_STATE_WAIT_FOR_PROMPT_SEND_COMPLETE:
            
            /* Here we check if the write transfer is done */
            if(appCDCData.writeTransferDone)
            {
                if(appCDCData.writeTransferResult == USB_HOST_CDC_RESULT_SUCCESS)
                {
                    /* Now to get data from the device */
                    appCDCData.state = APP_CDC_STATE_GET_DATA_FROM_DEVICE;
                }
                else
                {
                    /* Try sending the prompt again. */
                    appCDCData.state = APP_CDC_STATE_SEND_PROMPT_TO_DEVICE;
                }
            }
            
            break;
            
         case APP_CDC_STATE_GET_DATA_FROM_DEVICE:
            
            /* Here we request data from the device */
            appCDCData.readTransferDone = false;
            result = USB_HOST_CDC_Read(appCDCData.cdcHostHandle, NULL, appCDCData.inDataArray, 1);
            if(result == USB_HOST_CDC_RESULT_SUCCESS)
            {
                appCDCData.state = APP_CDC_STATE_DATA_RECEIVED_FROM_DEVICE;
            }
            break;   
            
        case APP_CDC_STATE_DATA_RECEIVED_FROM_DEVICE:

            /* We got data from the attached device.
             * This should be a number; 1, 2 or 3 which
             * which decides which LED on the PIC32 starter
             * kit should be switched on */

            if ( appCDCData.readTransferDone == true )
            {
                temp = appCDCData.inDataArray[0];

                writeData[count] =  appCDCData.inDataArray[0] ;

                count++;
                if ( temp == 13 || count == 13 )
                {

                    writeData[count] = ' ';
                    count++ ;

                    xSemaphoreGive(appCDCData.xSemaphoreCDCReadComplete );
                    appCDCData.state = APP_CDC_STATE_IDLE;
                
                }
           

                else
                {
                /* Send the prompt to the device and wait
                 * for data again */
                    appCDCData.state =  APP_CDC_STATE_GET_DATA_FROM_DEVICE ;
                }
           
            }
            break;
            
        case APP_CDC_STATE_IDLE:
                
                break;

        case APP_CDC_STATE_ERROR:

            /* Some error has occurred */

            break;
        default:
            break;
    }

    vTaskDelay( 10 / portTICK_PERIOD_MS);
    }

}


USB_HOST_CDC_EVENT_RESPONSE APP_USBHostCDCEventHandler
(
    USB_HOST_CDC_HANDLE cdcHandle,
    USB_HOST_CDC_EVENT event,
    void * eventData,
    uintptr_t context
)
{
    /* This function is called when a CDC Host event has occurred. A pointer to
     * this function is registered after opening the device. See the call to
     * USB_HOST_CDC_EventHandlerSet() function. */

    USB_HOST_CDC_EVENT_ACM_SET_LINE_CODING_COMPLETE_DATA * setLineCodingEventData;
    USB_HOST_CDC_EVENT_ACM_SET_CONTROL_LINE_STATE_COMPLETE_DATA * setControlLineStateEventData;
    USB_HOST_CDC_EVENT_WRITE_COMPLETE_DATA * writeCompleteEventData;
    USB_HOST_CDC_EVENT_READ_COMPLETE_DATA * readCompleteEventData;
    
    switch(event)
    {
        case USB_HOST_CDC_EVENT_ACM_SET_LINE_CODING_COMPLETE:
            
            /* This means the application requested Set Line Coding request is
             * complete. */
            setLineCodingEventData = (USB_HOST_CDC_EVENT_ACM_SET_LINE_CODING_COMPLETE_DATA *)(eventData);
            appCDCData.controlRequestDone = true;
            appCDCData.controlRequestResult = setLineCodingEventData->result;
            break;
            
        case USB_HOST_CDC_EVENT_ACM_SET_CONTROL_LINE_STATE_COMPLETE:
            
            /* This means the application requested Set Control Line State 
             * request has completed. */
            setControlLineStateEventData = (USB_HOST_CDC_EVENT_ACM_SET_CONTROL_LINE_STATE_COMPLETE_DATA *)(eventData);
            appCDCData.controlRequestDone = true;
            appCDCData.controlRequestResult = setControlLineStateEventData->result;
            break;
            
        case USB_HOST_CDC_EVENT_WRITE_COMPLETE:
            
            /* This means an application requested write has completed */
            appCDCData.writeTransferDone = true;
            writeCompleteEventData = (USB_HOST_CDC_EVENT_WRITE_COMPLETE_DATA *)(eventData);
            appCDCData.writeTransferResult = writeCompleteEventData->result;
            break;
            
        case USB_HOST_CDC_EVENT_READ_COMPLETE:
            
            /* This means an application requested write has completed */
            appCDCData.readTransferDone = true;
            readCompleteEventData = (USB_HOST_CDC_EVENT_READ_COMPLETE_DATA *)(eventData);
            appCDCData.readTransferResult = readCompleteEventData->result;
            break;
            
        case USB_HOST_CDC_EVENT_DEVICE_DETACHED:
            
            /* The device was detached */
            appCDCData.deviceWasDetached = true;
            vTaskSuspend (appData.taskHandleCDC);
            appCDCData.state = APP_CDC_STATE_WAIT_FOR_DEVICE_ATTACH;
            break;
            
        default:
            break;
    }
    
    return(USB_HOST_CDC_EVENT_RESPONE_NONE);
}

/*******************************************************************************
 End of File
 */
