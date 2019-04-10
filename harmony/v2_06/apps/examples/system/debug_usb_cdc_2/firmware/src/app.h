/*******************************************************************************
  MPLAB Harmony Application

  Application Header
  
  Company:
    Microchip Technology Inc.

  File Name:
    app.h

  Summary:
	Application definitions. 

  Description:
	 This file contains the  application definitions.
*******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef _APP_HEADER_H
#define _APP_HEADER_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system/system.h"
#include "system/int/sys_int.h"
#include "driver/driver_common.h"
#include "usb/usb_cdc.h"
#include "usb/usb_device.h"
#include "usb/usb_device_cdc.h"
#include "system/devcon/sys_devcon.h"
#include "system/console/sys_console.h"
#include "system/console/src/sys_console_usb_cdc_local.h"
#include "system/debug/sys_debug.h"
#include "system/tmr/sys_tmr.h"

#include "usb/usb_chapter_9.h"
#include "usb/usb_device.h"


// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

/* Fulfill USB DMA transfer criteria */
#define APP_READ_BUFFER_SIZE                    64
#define APP_WRITE_BUFFER_SIZE                   64

// *****************************************************************************
/* Application States

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
    APP_STATE_INIT,

    APP_STATE_READY,

    APP_STATE_WAIT_FOR_SWITCH,

    APP_STATE_WRITE_TEST_1,

    APP_STATE_WRITE_TEST_1_WFC,

    APP_STATE_WRITE_TEST_2,

    APP_STATE_WRITE_TEST_2_WFC,

    APP_STATE_WRITE_TEST_3,

    APP_STATE_WRITE_TEST_3_WFC,

    APP_STATE_READ_TEST_1,

    APP_STATE_READ_TEST_1_WFC,

    APP_STATE_READ_TEST_2,

    APP_STATE_READ_TEST_2_WFC,

    APP_STATE_ECHO_TEST,

    APP_STATE_ECHO_TEST_RD,

    APP_STATE_ECHO_TEST_WR,

    APP_STATE_DONE,

    APP_STATE_ERROR

} APP_STATES;





// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* Application's current state*/
    APP_STATES state;

    bool switchPressed;

    size_t bytesRead;

    bool wrComplete;

    bool rdComplete;

} APP_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/

/*******************************************************************************
  Function:
    void APP_UsbDeviceEventCallBack(USB_DEVICE_EVENTS events)

  Summary:
    Device layer event notification callback.

  Description:
    This routine defines the device layer event notification callback.

  Precondition:
    The device layer should be opened by the application and the callback should
    be registered with the device layer.

  Parameters:
    events  - specific device event

  Returns:
    None.

  Remarks:
    None.
*/

void APP_USBDeviceEventCallBack(USB_DEVICE_EVENT events,
        void * eventData, uintptr_t context);

//void USBDeviceCDCEventHandler
//(
//    USB_DEVICE_CDC_INDEX index ,
//    USB_DEVICE_CDC_EVENT event ,
//    void * pData,
//    uintptr_t userData
//);

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Summary:
     MPLAB Harmony Demo application initialization routine

  Description:
    This routine initializes Harmony Demo application.  This function opens
    the necessary drivers, initializes the timer and registers the application
    callback with the USART driver.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    APP_Initialize();


  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void APP_Initialize ( void );


/*******************************************************************************
  Function:
    void APP_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void APP_Tasks ( void );

void SYS_Initialize ( void* data );
void SYS_Tasks ( void );
// *****************************************************************************
// *****************************************************************************
// Section: extern declarations
// *****************************************************************************
// *****************************************************************************

extern APP_DATA appData;

//extern CONS_USB_CDC_DATA consUsbData;
//extern USB_DEVICE_FUNCTION_DRIVER cdcFuncDriver;

#endif /* _APP_HEADER_H */

/*******************************************************************************
 End of File
 */



