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

void buttons_handleInterrupt( void );
void read_buttons( void );

//// DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
// DOM-IGNORE-END

#endif	/* BTAD_BUTTONS_H */

