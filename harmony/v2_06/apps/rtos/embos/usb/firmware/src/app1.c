/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app1.c

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

#include "app1.h"

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

APP1_DATA appData;
/*Keyboard Report to be transmitted*/
extern KEYBOARD_INPUT_REPORT __attribute__((coherent, aligned(4))) keyboardInputReport ;
/* Keyboard output report */
extern KEYBOARD_OUTPUT_REPORT __attribute__((coherent, aligned(4))) keyboardOutputReport;
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


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP1_Initialize ( void )

  Remarks:
    See prototype in app1.h.
 */

void APP1_Initialize ( void )
{
   appData.deviceHandle = USB_DEVICE_HANDLE_INVALID;
   appData.isConfigured = false;

   /* Initialize the keycode array */
   appData.key = USB_HID_KEYBOARD_KEYPAD_KEYBOARD_A;
   appData.keyCodeArray.keyCode[0] = USB_HID_KEYBOARD_KEYPAD_RESERVED_NO_EVENT_INDICATED;
   appData.keyCodeArray.keyCode[1] = USB_HID_KEYBOARD_KEYPAD_RESERVED_NO_EVENT_INDICATED;
   appData.keyCodeArray.keyCode[2] = USB_HID_KEYBOARD_KEYPAD_RESERVED_NO_EVENT_INDICATED;
   appData.keyCodeArray.keyCode[3] = USB_HID_KEYBOARD_KEYPAD_RESERVED_NO_EVENT_INDICATED;
   appData.keyCodeArray.keyCode[4] = USB_HID_KEYBOARD_KEYPAD_RESERVED_NO_EVENT_INDICATED;
   appData.keyCodeArray.keyCode[5] = USB_HID_KEYBOARD_KEYPAD_RESERVED_NO_EVENT_INDICATED;

   /* Initialize the modifier keys */
   appData.keyboardModifierKeys.modifierkeys = 0;

   /* Initialise the led state */
   memset(&keyboardOutputReport.data, 0, 64);

   /* Intialize the switch state */
   appData.isSwitchPressed = false;
   appData.ignoreSwitchPress = false;

   /* Initialize the HID instance index.  */
   appData.hidInstance = 0;

   /* Initialize tracking variables */
   appData.isReportReceived = false;
   appData.isReportSentComplete = true;

   /* Initialize the application state*/
   appData.state = APP_STATE_INIT;

}


/******************************************************************************
  Function:
    void APP1_Tasks ( void )

  Remarks:
    See prototype in app1.h.
 */

void APP1_Tasks ( void )
{

    /* Call the application's tasks routine */
    switch(appData.state)
    {
        case APP_STATE_INIT:

            /* Open an instance of the device layer */
            appData.deviceHandle = USB_DEVICE_Open( USB_DEVICE_INDEX_0,
                    DRV_IO_INTENT_READWRITE );

            if(appData.deviceHandle == USB_DEVICE_HANDLE_INVALID)
            {
                break;
            }

            /* Register a callback with device layer to get
             * event notification (for end point 0) */
            USB_DEVICE_EventHandlerSet(appData.deviceHandle, ApplicationUSBDeviceEventHandler, 0);

            appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            break;
        case APP_STATE_WAIT_FOR_CONFIGURATION:

            /* Check if the device is configured. The
             * isConfigured flag is updated in the
             * Device Event Handler */
            if(appData.isConfigured)
            {
               /* Initialize the flag and place a request for a
                * output report */
               appData.isReportReceived = false;
               USB_DEVICE_HID_ReportReceive(appData.hidInstance,
                       &appData.receiveTransferHandle,
                       (uint8_t *)&keyboardOutputReport,64);
               appData.state = APP_STATE_CHECK_IF_CONFIGURED;
            }
            break;
        case APP_STATE_CHECK_IF_CONFIGURED:

            /* This state is needed because the device can get
             * unconfigured asynchronously. Any application state
             * machine reset should happen within the state machine
             * context only. */

            if(appData.isConfigured)
            {
               appData.state = APP_STATE_SWITCH_PROCESS;
            }
            else
            {
               /* This means the device got de-configured.
                * We reset the state and the wait for configuration */
               appData.isReportReceived = false;
               appData.isReportSentComplete = true;
               appData.key = USB_HID_KEYBOARD_KEYPAD_KEYBOARD_A;
               appData.keyboardModifierKeys.modifierkeys = 0;
               memset(&keyboardOutputReport.data, 0, 64);
               appData.isSwitchPressed = false;
               appData.ignoreSwitchPress = false;

               appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            }
            break;
        case APP_STATE_SWITCH_PROCESS:

            appData.isSwitchPressed = false;

            /* Check if the switch was pressed, this state machine must be
            called quickly enough to catch the user pressing the switch. This
            could lead to problems if not implemented correctly.*/
            if(BSP_SwitchStateGet( BSP_SWITCH_3) == BSP_SWITCH_STATE_PRESSED)
            {
               /*pause the task for 200ms, to ensure user has let go of switch,
               if switch is still pressed, stay here until switch is let go.  If
               user presses switch and holds on, worst case the usb task will not
               progess thorough the state machine, but rest of system will be
               functional.*/
               do
               {
                  OS_Delay(100);
               }while( BSP_SwitchStateGet( BSP_SWITCH_3) == BSP_SWITCH_STATE_PRESSED);

               /* get here, this means this is a valid switch press */
               appData.isSwitchPressed = true;
            }
            /*go to the next state. */
            appData.state = APP_STATE_CHECK_FOR_OUTPUT_REPORT;
            break;
        case APP_STATE_CHECK_FOR_OUTPUT_REPORT:
            appData.state = APP_STATE_EMULATE_KEYBOARD;

            /*was a report receieved*/
            if(!appData.isReportReceived)
               break;

            /* Get here report received, update the LED, schedule and request*/
            if(keyboardOutputReport.ledState.numLock == KEYBOARD_LED_STATE_ON)
            {
               BSP_LEDOff (  BSP_LED_2);
            }
            else
            {
               BSP_LEDOn (  BSP_LED_2);
            }

            if(keyboardOutputReport.ledState.capsLock == KEYBOARD_LED_STATE_ON)
            {
               BSP_LEDOff(  BSP_LED_3);
            }
            else
            {
               BSP_LEDOn (  BSP_LED_3);
            }
            appData.isReportReceived = false;
            USB_DEVICE_HID_ReportReceive(appData.hidInstance,
                    &appData.receiveTransferHandle,
                    (uint8_t *)&keyboardOutputReport,64);
            break;
        case APP_STATE_EMULATE_KEYBOARD:
            /*go to next state no matter what..*/
            appData.state = APP_STATE_CHECK_IF_CONFIGURED;

            /*only process if report is complete*/
            if(!appData.isReportSentComplete)
               break;

            /*Get here, this means report can be sent, Emulate keyboard*/
            if(appData.isSwitchPressed)
            {
               /* Clear the switch pressed flag */
               appData.isSwitchPressed = false;

               if(appData.key == USB_HID_KEYBOARD_KEYPAD_KEYBOARD_RETURN_ENTER)
               {
                  appData.key = USB_HID_KEYBOARD_KEYPAD_KEYBOARD_A;
               }
               appData.keyCodeArray.keyCode[0] = appData.key;
               /* If the switch was pressed, update the key counter and then
                * add the key to the keycode array. */
               appData.key ++;
            }
            else
            {
                /* Indicate no event */
               appData.keyCodeArray.keyCode[0] =
                  USB_HID_KEYBOARD_KEYPAD_RESERVED_NO_EVENT_INDICATED;
            }

            KEYBOARD_InputReportCreate(&appData.keyCodeArray,
               &appData.keyboardModifierKeys, &keyboardInputReport);

            /*clear report complete for next time through*/
            appData.isReportSentComplete = false;

            USB_DEVICE_HID_ReportSend(appData.hidInstance,
                &appData.sendTransferHandle,
                (uint8_t *)&keyboardInputReport,
                sizeof(KEYBOARD_INPUT_REPORT));
            break;
        case APP_STATE_ERROR:
             break;
        default:
             break;
    }
}
 

/*******************************************************************************
 End of File
 */
