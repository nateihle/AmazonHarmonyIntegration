<#-- usb_mouse_device_macros_app.h.ftl -->

<#--
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
-->
<#macro macro_lib_usb_app_h_includes>
</#macro>

<#--
// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_lib_usb_app_h_type_definitions>

// *****************************************************************************
/* Number of Mouse Buttons.

  Summary:
    Number of Mouse Buttons.

  Description:
    Number of Mouse Buttons.

  Remarks:
    None.
*/
#define MOUSE_BUTTON_NUMBERS 2

// *****************************************************************************
/* Mouse Coordinate.

  Summary:
    Mouse Coordinate type

  Description:
    This type defines the  Mouse Coordinate data type.
    
  Remarks:
    None.
*/

typedef int8_t MOUSE_COORDINATE; 

// *****************************************************************************
/*  Mouse Button State.

  Summary:
   Mouse Button State.

  Description:
    This enumeration defines the possible state of the mouse button.
    
  Remarks:
    None.
*/

typedef enum
{
    /* Button is not pressed */
    MOUSE_BUTTON_STATE_RELEASED,
	
	/* Button is pressed */
    MOUSE_BUTTON_STATE_PRESSED

}
MOUSE_BUTTON_STATE;

// *****************************************************************************
/* Mouse Report

  Summary:
   Mouse Report.

  Description:
    This is the Mouse Report. The application can use the 
    MOUSE_ReportCreate() function to populate this report.
    
  Remarks:
    None.
*/

typedef struct
{
    uint8_t data[3];
}
MOUSE_REPORT;

/* Macro defines USB internal DMA Buffer criteria*/
#define APP_MAKE_BUFFER_DMA_READY __attribute__((coherent, aligned(16)))

/* Macro defines the conversion factor to be
 * multiplied to convert to millisecs*/
#define APP_USB_CONVERT_TO_MILLISECOND (1/8)
</#macro>


<#--
// *****************************************************************************
/* Application Data

typedef struct
{
    /* The application's current state */
    ${APP_NAME?upper_case}_STATES state;

    /* TODO: Define any additional data used by the application. */
-->
<#macro macro_lib_usb_app_h_data>
    /*
     * USB variables used by the mouse device application:
     * 
     *     handleUsbDevice          : USB Device driver handle
     *     usbDeviceIsConfigured    : If true, USB Device is configured
     *     emulateMouse             : If true, then mouse is emulated
     *     xCoordinate              : Mouse x coordinate
     *     yCoordinate              : Mouse y coordinate
     *     mouseButton[MOUSE_BUTTON_NUMBERS] : Mouse buttons
     *     reportTransferHandle     : Transfer handle 
     *     activeProtocol           : USB HID active Protocol
     *     idleRate                 : USB HID current Idle
     *     isMouseReportSendBusy    : Tracks the progress of the report send 
     *     setIdleTimer             : SET IDLE timer
     */
    USB_DEVICE_HANDLE                handleUsbDevice;
    bool                             usbDeviceIsConfigured;
    bool                             emulateMouse;
    MOUSE_COORDINATE                 xCoordinate;
    MOUSE_COORDINATE                 yCoordinate;
    MOUSE_BUTTON_STATE               mouseButton[MOUSE_BUTTON_NUMBERS];
    USB_DEVICE_HID_TRANSFER_HANDLE   reportTransferHandle;
    uint8_t                          activeProtocol;
    uint8_t                          idleRate;
    bool                             isMouseReportSendBusy;
    uint16_t                         setIdleTimer;

</#macro>
<#--
} ${APP_NAME?upper_case}_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/
-->
<#macro macro_lib_usb_app_h_callback_function_declarations>
</#macro>

<#--	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_lib_usb_app_h_function_declarations>
</#macro>

<#macro macro_lib_usb_app_h_states>
</#macro>

