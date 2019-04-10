/* 
 * File:   display.h
 * Author: C16266
 *
 * Created on November 16, 2016, 2:19 PM
 */

#ifndef DISPLAY_H
#define	DISPLAY_H

#ifdef	__cplusplus
extern "C" {
#endif
    

typedef enum{
    DISPLAY_SWITCH_SCREEN = 0,
    DISPLAY_USB_INSERT,
    DISPLAY_USB_CONNECTED,
    DISPLAY_START_RECORD,
    DISPLAY_STOP_RECORD,
    DISPLAY_SAVE_FILE,
}DISPLAY_EVENT;

typedef struct{
    char saveFileName[64];
    
}APP_DISPLAY_DATA;

void APP_DisplayInit();
void APP_DisplayTask();
//void APP_UpdateDisplay(DISPLAY_EVENT de);
// prompt string
void APP_DisplayInsertUSB();
void APP_DisplayStartRecord();
void APP_DisplayStopRecord();
void APP_DisplaySavedFile();
//void APP_UpdatePromptLabel(const char *data);

//void APP_UpdateMessageLabel(const char *data);
//void APP_UpdateTrackName(const char *track);
//void APP_UpdateArtistName(const char *data);
//void APP_UpdateAlbumName(const char *data);
//void APP_UpdateTrackLength(uint32_t data);
//void APP_UpdatePlaytime(uint32_t data);
//void APP_UpdateScreen(uint32_t data);
//void APP_UpdateFileList(uint32_t data);
#ifdef	__cplusplus
}
#endif

#endif	/* DISPLAY_H */

