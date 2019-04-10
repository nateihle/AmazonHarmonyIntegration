#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "usb/usb_common.h"
#include "usb/usb_chapter_9.h"
#include "usb/usb_device_hid.h"
#include "usb/usb_device.h"
#include "app_keyboard.h"

/*Keyboard Report to be transmitted*/
KEYBOARD_INPUT_REPORT __attribute__((coherent, aligned(4))) keyboardInputReport ;
/* Keyboard output report */
KEYBOARD_OUTPUT_REPORT __attribute__((coherent, aligned(4))) keyboardOutputReport;

void KEYBOARD_InputReportCreate
(
    KEYBOARD_KEYCODE_ARRAY * keyboardKeycodeArray,
    KEYBOARD_MODIFIER_KEYS * keyboardModifierKeys,
    KEYBOARD_INPUT_REPORT * keyboardInputReport
)
{
   int index;

   for (index = 0; index < 6 ; index ++)
   {
      /* Create the keyboard button bit map */
      keyboardInputReport->data[index + 2] = keyboardKeycodeArray->keyCode[index];
   }

   /* Update the modifier key */
   keyboardInputReport->data[0] = keyboardModifierKeys->modifierkeys;

   return;
}
