#ifndef APP_KEYBOARD_H
#define	APP_KEYBOARD_H

// *****************************************************************************
// *****************************************************************************
// Section: USB Device HID Keyboard types and definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Keyboard Modifier Key State.

  Summary:
    Keyboard Modifier Key State.

  Description:
    This enumeration defines the possible state of the modifier keys.

  Remarks:
    None.
*/

typedef enum
{
   /* ModKey is not pressed */
   KEYBOARD_MODIFIER_KEY_STATE_RELEASED,
   
   /* Button is pressed */
   KEYBOARD_MODIFIER_KEY_STATE_PRESSED
   
}KEYBOARD_MODIFIER_KEY_STATE;

// *****************************************************************************
/* Keyboard Key Code Array.

  Summary:
    Keyboard Key Code Array.

  Description:
    This is the definition of the key code array.

  Remarks:
    None.
*/

typedef struct
{
   /* Key codes */
   USB_HID_KEYBOARD_KEYPAD keyCode[6];

}KEYBOARD_KEYCODE_ARRAY;


// *****************************************************************************
/* Keyboard Modifier Keys

  Summary:
    Keyboard Modifier Keys.

  Description:
    This is the HID Keyboard Modifier keys data type.

  Remarks:
    None.
*/
typedef struct
{
   union
   {
      struct
      {
         unsigned int leftCtrl   : 1;
         unsigned int leftShift  : 1;
         unsigned int leftAlt    : 1;
         unsigned int leftGui    : 1;
         unsigned int rightCtrl  : 1;
         unsigned int rightShift : 1;
         unsigned int rightAlt   : 1;
         unsigned int rightGui   : 1;
      };

      int8_t modifierkeys;
   };

} KEYBOARD_MODIFIER_KEYS;

// *****************************************************************************
/* Keyboard Input Report

  Summary:
    Keyboard Input Report.

  Description:
    This is the HID Keyboard Input Report.

  Remarks:
    None.
*/

typedef struct
{
   uint8_t data[8];

}KEYBOARD_INPUT_REPORT;

// *****************************************************************************
/* Keyboard Output Report

  Summary:
    Keyboard Output Report.

  Description:
    This is the HID Keyboard Output Report. This reports is received by the
    keyboard from the host. The application can use this data type while
    placing a request for the keyboard output report by using the
    USB_DEVICE_HID_ReportReceive() function.

  Remarks:
    None.
*/

typedef union
{
   struct
   {
      unsigned int numLock      :1;
      unsigned int capsLock     :1;
      unsigned int scrollLock   :1;
      unsigned int compose      :1;
      unsigned int kana         :1;
      unsigned int constant     :3;
   }ledState;

   uint8_t data[64];

}KEYBOARD_OUTPUT_REPORT;

// *****************************************************************************
/* Keyboard LED State

  Summary:
    Keyboard LED State.

  Description:
    This is the HID Keyboard LED state.

  Remarks:
    None.
*/

typedef enum
{
   /* This is the LED OFF state */
   KEYBOARD_LED_STATE_OFF = 0,

   /* This is the LED ON state */
   KEYBOARD_LED_STATE_ON = 1

}KEYBOARD_LED_STATE;



// *****************************************************************************
// *****************************************************************************
// Section: USB Device HID Keyboard functions
// *****************************************************************************
// *****************************************************************************

void KEYBOARD_InputReportCreate
(
   KEYBOARD_KEYCODE_ARRAY * keyboardKeycodeArray,
   KEYBOARD_MODIFIER_KEYS * keyboardModifierKeys,
   KEYBOARD_INPUT_REPORT * keyboardInputReport
);

#endif	/* APP_KEYBOARD_H */

