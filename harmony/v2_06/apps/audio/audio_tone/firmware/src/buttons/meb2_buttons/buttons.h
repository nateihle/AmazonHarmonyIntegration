/* File:   pic32mx_btad_buttons/btad_buttons.h
 * Author: C16825 (CAL)
 * Created: 07/24/2015 */

#ifndef BTAD_BUTTONS_H
#define	BTAD_BUTTONS_H

//#include "app.h"

//// DOM-IGNORE-BEGIN
#ifdef __cplusplus
extern "C" {
#endif
// DOM-IGNORE-END

typedef enum
{
    TOUCH_F1_PLUS,
    TOUCH_F1_MINUS,
    TOUCH_F2_PLUS,
    TOUCH_F2_MINUS,
    TOUCH_T_PLUS,
    TOUCH_T_MINUS,
    TOUCH_VOLUME_PLUS,
    TOUCH_VOLUME_MINUS,
    TOUCH_PLAYPAUSE,
    TOUCH_SINECHIRP,            
    TOUCH_NUM_BUTTONS       // for array 
} TOUCH_BUTTONS;

extern bool buttons[TOUCH_NUM_BUTTONS];
    
void read_buttons( void );    
void buttons_handleInterrupt( void );

//// DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
// DOM-IGNORE-END

#endif	/* BTAD_BUTTONS_H */

