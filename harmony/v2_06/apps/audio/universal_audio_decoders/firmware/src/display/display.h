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
    
//#include "../data_structures.h"

typedef enum{
    DISPLAY_ENABLE_WAV = 0,
    DISPLAY_ENABLE_MP3,
    DISPLAY_ENABLE_AAC,
    DISPLAY_ENABLE_WMA,
    DISPLAY_ENABLE_OPUS,
    DISPLAY_ENABLE_ADPCM,
    DISPLAY_ENABLE_SPEEX,
    DISPLAY_ENABLE_FLAC,
    DISPLAY_HIGHLIGHT_WAV,
    DISPLAY_HIGHLIGHT_MP3,
    DISPLAY_HIGHLIGHT_AAC,
    DISPLAY_HIGHLIGHT_WMA,
    DISPLAY_HIGHLIGHT_OPUS,
    DISPLAY_HIGHLIGHT_ADPCM,
    DISPLAY_HIGHLIGHT_SPEEX,
    DISPLAY_HIGHLIGHT_FLAC,
    DISPLAY_TRACK_NAME,
    DISPLAY_ARTIST_NAME,
    DISPLAY_ALBUM_NAME,
    DISPLAY_TRACK_LENGTH,
    DISPLAY_PLAY_TIME,
    DISPLAY_SWITCH_SCREEN,
    DISPLAY_WELCOME_MESSAGE,
    DISPLAY_FILE_LIST,
}DISPLAY_EVENT;

typedef struct{
    GFXU_CHAR   album[64];
    uint8_t     albumLen;
    GFXU_CHAR   track[64];
    uint8_t     trackLen;
    GFXU_CHAR   artist[64];
    uint8_t     artistLen;
    
    GFXU_CHAR   prompt_msg[64];
    uint8_t     prompt_msg_len;
    
    uint8_t     screenID;
    uint32_t    track_length;
    GFXU_CHAR   track_duration[16];
    uint32_t    track_dur_len;
    
    uint32_t    playtime;
    GFXU_CHAR   playtime_str[16];
    uint32_t    playtime_str_len;
   
}APP_DISPLAY_DATA;

void APP_DisplayInit();
void APP_DisplayTasks();
void APP_UpdateDisplay(DISPLAY_EVENT de);
void APP_UpdateMessageLabel(const char *data);
void APP_UpdateTrackName(const char *track);
void APP_UpdateArtistName(const char *data);
void APP_UpdateAlbumName(const char *data);
void APP_UpdateTrackLength(uint32_t data);
void APP_UpdatePlaytime(uint32_t data);
void APP_UpdateScreen(uint32_t data);
void APP_CleanMetaData();
void APP_UpdateFileList(uint32_t data);
#ifdef	__cplusplus
}
#endif

#endif	/* DISPLAY_H */

