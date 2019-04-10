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

//void APP_ButtonInit(void);
//void APP_ButtonTask(void);
//void APP_OnButtonEvent(APP_DATA * appData,
//                       uint8_t button, 
//                       bool bButtonClosed, 
//                       int32_t repeatCount);
    
void read_buttons( void );    
void buttons_handleInterrupt( void );

//// DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
// DOM-IGNORE-END

#endif	/* BTAD_BUTTONS_H */

