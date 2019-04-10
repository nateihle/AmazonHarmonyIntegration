/*******************************************************************************
 Display Tasks

  Company:
    Microchip Technology Inc.

  File Name:
   Display_tasks.c

  Summary:
    Contains the functional implementation of display task for

  Description:
    This file contains the functional implementation of data parsing functions
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

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
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "display.h"

/**STATIC VARIABLES*/
static int32_t _displayEvent;
static APP_DISPLAY_DATA _displayData[1];

static void _display_switch_screen();
static void _display_enable_decoder(APP_DECODER_TYPE decoder);
static void _display_highlight_decoder(APP_DECODER_TYPE decoder);
static void _display_dehighlight_decoder(laLabelWidget* _prevDecoder);
static void _display_track_name();
static void _display_artist_name();
static void _display_album_name();
static void _display_track_length();
static void _display_play_time();
static void _display_prompt_message();
//static void _display_file_list();

static void _updateLabelWidgetText(laLabelWidget* lbl, GFXU_CHAR *la_data, uint32_t data_len, uint8_t fidx);
//static bool _list_add_item(laListWidget *listWidget, laString *item_str, GFXU_ImageAsset* icon);
extern GFXU_FontAsset* fontList[5];
static int32_t pb_circle_initial_x = 0;
static laLabelWidget* _prevDecoder=NULL;


void APP_DisplayInit(){
    _displayEvent = 0;
    if(CircleIndicator != NULL)
    {
        pb_circle_initial_x = CircleIndicator->widget.rect.x;
    }
    memset((void*)_displayData, 0, sizeof(APP_DISPLAY_DATA));
}

void APP_DisplayTasks()
{
    if(_displayEvent == 0)
    {
        return;
    }
    laContext* context = laContext_GetActive();
    
    if (laContext_GetActiveScreenIndex() == MainScreen_ID && context->activeScreen->layers[0]->frameState == LA_LAYER_FRAME_IN_PROGRESS)
    {
        return;
    }

    if((_displayEvent >> DISPLAY_SWITCH_SCREEN) & 1)
    {
        _display_switch_screen();
        _displayEvent &= ~(1 << DISPLAY_SWITCH_SCREEN);
    }
   // check event bit
    if((_displayEvent >> DISPLAY_ENABLE_WAV) & 1){
        _display_enable_decoder(APP_DECODER_WAV);
        // clear this event
        _displayEvent &= ~(1 << DISPLAY_ENABLE_WAV);
    }
    if((_displayEvent >> DISPLAY_ENABLE_MP3) & 1){
        _display_enable_decoder(APP_DECODER_MP3);
        _displayEvent &= ~(1 << DISPLAY_ENABLE_MP3);
    }
    if((_displayEvent >> DISPLAY_ENABLE_AAC) & 1){
        _display_enable_decoder(APP_DECODER_AAC);
        _displayEvent &= ~(1 << DISPLAY_ENABLE_AAC);
    }
    
    if((_displayEvent >> DISPLAY_ENABLE_WMA) & 1){
        _display_enable_decoder(APP_DECODER_WMA);
        _displayEvent &= ~(1 << DISPLAY_ENABLE_WMA);
    }
    
    if((_displayEvent >> DISPLAY_ENABLE_OPUS) & 1){
        _display_enable_decoder(APP_DECODER_OPUS);
        _displayEvent &= ~(1 << DISPLAY_ENABLE_OPUS);
    }
    
    if((_displayEvent >> DISPLAY_ENABLE_SPEEX) & 1){
        _display_enable_decoder(APP_DECODER_SPEEX);
        _displayEvent &= ~(1 << DISPLAY_ENABLE_SPEEX);
    }
    
    if((_displayEvent >> DISPLAY_ENABLE_ADPCM) & 1){
        _display_enable_decoder(APP_DECODER_ADPCM);
        _displayEvent &= ~(1 << DISPLAY_ENABLE_ADPCM);
    }
    if((_displayEvent >> DISPLAY_ENABLE_FLAC) & 1){
        _display_enable_decoder(APP_DECODER_FLAC);
        _displayEvent &= ~(1 << DISPLAY_ENABLE_FLAC);
    }
    if((_displayEvent >> DISPLAY_TRACK_NAME) & 1){
        _display_track_name();
        _displayEvent &= ~(1 << DISPLAY_TRACK_NAME);
    }
    
    if((_displayEvent >> DISPLAY_ARTIST_NAME) & 1){
        _display_artist_name();
        _displayEvent &= ~(1 << DISPLAY_ARTIST_NAME);
    }
    if((_displayEvent >> DISPLAY_HIGHLIGHT_WAV) & 1){
        _display_dehighlight_decoder(_prevDecoder);
        _display_highlight_decoder(APP_DECODER_WAV);
        _displayEvent &= ~(1 << DISPLAY_HIGHLIGHT_WAV);
    }
    if((_displayEvent >> DISPLAY_HIGHLIGHT_MP3) & 1){
        _display_dehighlight_decoder(_prevDecoder);
        _display_highlight_decoder(APP_DECODER_MP3);
        _displayEvent &= ~(1 << DISPLAY_HIGHLIGHT_MP3);
    }
    if((_displayEvent >> DISPLAY_HIGHLIGHT_AAC) & 1){
        _display_dehighlight_decoder(_prevDecoder);
        _display_highlight_decoder(APP_DECODER_AAC);
        _displayEvent &= ~(1 << DISPLAY_HIGHLIGHT_AAC);
    }
    
    if((_displayEvent >> DISPLAY_HIGHLIGHT_WMA) & 1){
        _display_dehighlight_decoder(_prevDecoder);
        _display_highlight_decoder(APP_DECODER_WMA);
        _displayEvent &= ~(1 << DISPLAY_HIGHLIGHT_WMA);
    }
    
    if((_displayEvent >> DISPLAY_HIGHLIGHT_OPUS) & 1){
        _display_dehighlight_decoder(_prevDecoder);
        _display_highlight_decoder(APP_DECODER_OPUS);
        _displayEvent &= ~(1 << DISPLAY_HIGHLIGHT_OPUS);
    }
    
    if((_displayEvent >> DISPLAY_HIGHLIGHT_SPEEX) & 1){
        _display_dehighlight_decoder(_prevDecoder);
        _display_highlight_decoder(APP_DECODER_SPEEX);
        _displayEvent &= ~(1 << DISPLAY_HIGHLIGHT_SPEEX);
    }
    
    if((_displayEvent >> DISPLAY_HIGHLIGHT_ADPCM) & 1){
        _display_dehighlight_decoder(_prevDecoder);
        _display_highlight_decoder(APP_DECODER_ADPCM);
        _displayEvent &= ~(1 << DISPLAY_HIGHLIGHT_ADPCM);
    }
    if((_displayEvent >> DISPLAY_HIGHLIGHT_FLAC) & 1){
        _display_dehighlight_decoder(_prevDecoder);
        _display_highlight_decoder(APP_DECODER_FLAC);
        _displayEvent &= ~(1 << DISPLAY_HIGHLIGHT_FLAC);
    }
    if((_displayEvent >> DISPLAY_ALBUM_NAME) & 1){
        _display_album_name();
        _displayEvent &= ~(1 << DISPLAY_ALBUM_NAME);
    }
    
    if((_displayEvent >> DISPLAY_TRACK_LENGTH) & 1){
        _display_track_length();
        _displayEvent &= ~(1 << DISPLAY_TRACK_LENGTH);
    }
    
    if((_displayEvent >> DISPLAY_PLAY_TIME) & 1){
        _display_play_time();
        _displayEvent &= ~(1 << DISPLAY_PLAY_TIME);
    }
    if((_displayEvent >> DISPLAY_WELCOME_MESSAGE) & 1){
        _display_prompt_message();
        _displayEvent &= ~(1 << DISPLAY_WELCOME_MESSAGE);
    }
    if((_displayEvent >> DISPLAY_FILE_LIST) & 1)
    {
        _displayEvent &= ~(1 << DISPLAY_FILE_LIST);
    }

}

/**
 * provide the application a display API to update screen
 * @param de
 */
void APP_UpdateDisplay(DISPLAY_EVENT de){
    _displayEvent |= (1 << de);
    
}
void APP_UpdateMessageLabel(const char *data)
{
    if(data != NULL)
    {
        uint8_t cur = 0;
        while(data[cur]!='\0' && cur<63){
            _displayData->prompt_msg[cur] = data[cur];
            cur++;
        }
        _displayData->prompt_msg[cur]='\0';
        _displayData->prompt_msg_len = cur;
    }
}
void APP_UpdateTrackName(const char *data)
{
    if(data != NULL)
    {
        uint8_t cur = 0;
        while(data[cur]!=0x00 && cur < 63){
            _displayData->track[cur] = data[cur];
            cur++;
        }
        _displayData->track[cur]=0x00;
        _displayData->trackLen = cur;
    }
}

void APP_UpdateArtistName(const char *data)
{
    if(data != NULL)
    {
        uint8_t cur = 0;
        while(data[cur]!='\0' && cur < 63){
            _displayData->artist[cur] = data[cur];
            cur++;
        }
        _displayData->artist[cur]=0x00;
        _displayData->artistLen = cur;
    }
}

void APP_UpdateAlbumName(const char *data)
{
    if(data != NULL)
    {
        uint8_t cur = 0;
        while(data[cur]!='\0' && cur < 63){
            _displayData->album[cur] = data[cur];
            cur++;
        }
        _displayData->album[cur]=0x00;
        _displayData->albumLen = cur;
    }
}

void APP_UpdateTrackLength(uint32_t data)
{
    _displayData->track_length = data;
}

void APP_UpdatePlaytime(uint32_t data)
{
    _displayData->playtime = data;
}

void APP_UpdateScreen(uint32_t data)
{
    _displayData->screenID = (uint8_t)data;
}

void APP_CleanMetaData()
{
//    _updateLabelWidgetText(TrackNameLabel, " ", 3);
//    _updateLabelWidgetText(TrackArtistLabel, " ", 3);
//    _updateLabelWidgetText(TrackAlbumLabel, " ", 3);
}

void APP_UpdateFileList(uint32_t data)
{
    //_displayData->fileList = (ListNode*)data;
}

void _display_enable_decoder(APP_DECODER_TYPE decoder)
{
    GFX_Color brightGreen = 0x7C0;
    if(laContext_GetActiveScreenIndex() == MainScreen_ID)
    {
        switch(decoder){
            case APP_DECODER_WAV:
                WAVIndicator->widget.scheme->text = brightGreen;
                laWidget_Invalidate((laWidget*)WAVIndicator);
    //            laLabelWidget_SetTextColor(WAVIndicator, brightGreen);
                break;
            case APP_DECODER_MP3:
                MP3Indicator->widget.scheme->text = brightGreen;
                laWidget_Invalidate((laWidget*)MP3Indicator);
                //laLabelWidget_SetTextColor(MP3Indicator, brightGreen);
                break;
            case APP_DECODER_AAC:
                AACIndicator->widget.scheme->text = brightGreen;
                laWidget_Invalidate((laWidget*)AACIndicator);
                break;
            case APP_DECODER_WMA:
                WMAIndicator->widget.scheme->text = brightGreen;
                laWidget_Invalidate((laWidget*)WMAIndicator);
                break;
            case APP_DECODER_OPUS:
                OPUSIndicator->widget.scheme->text = brightGreen;
                laWidget_Invalidate((laWidget*)OPUSIndicator);
                break;
            case APP_DECODER_SPEEX:
                SPXIndicator->widget.scheme->text = brightGreen;
                laWidget_Invalidate((laWidget*)SPXIndicator);
                break;
            case APP_DECODER_ADPCM:
                PCMIndicator->widget.scheme->text = brightGreen;
                laWidget_Invalidate((laWidget*)PCMIndicator);
                break;
            case APP_DECODER_FLAC:
                FLACIndicator->widget.scheme->text = brightGreen;
                laWidget_Invalidate((laWidget*)FLACIndicator);
                break;
            default:break;
        }
    }    
}

void _display_dehighlight_decoder(laLabelWidget* _prevDecoder)
{
    if(laContext_GetActiveScreenIndex() == MainScreen_ID)
    {
        if(_prevDecoder!=NULL)
            laWidget_SetBackgroundType((laWidget*)_prevDecoder, LA_WIDGET_BACKGROUND_NONE);
    }
}
void _display_highlight_decoder(APP_DECODER_TYPE decoder)
{
    if(laContext_GetActiveScreenIndex() == MainScreen_ID)
    {
        switch(decoder){
            case APP_DECODER_WAV:
                laWidget_SetBackgroundType((laWidget*)WAVIndicator, LA_WIDGET_BACKGROUND_FILL);
                _prevDecoder = WAVIndicator;
                break;
            case APP_DECODER_MP3:
                laWidget_SetBackgroundType((laWidget*)MP3Indicator, LA_WIDGET_BACKGROUND_FILL);
                _prevDecoder = MP3Indicator;
                break;
            case APP_DECODER_AAC:
                laWidget_SetBackgroundType((laWidget*)AACIndicator, LA_WIDGET_BACKGROUND_FILL);
                 _prevDecoder = AACIndicator;
                break;
            case APP_DECODER_WMA:
                laWidget_SetBackgroundType((laWidget*)WMAIndicator, LA_WIDGET_BACKGROUND_FILL);
                _prevDecoder = WMAIndicator;
                break;
            case APP_DECODER_OPUS:
                laWidget_SetBackgroundType((laWidget*)OPUSIndicator, LA_WIDGET_BACKGROUND_FILL);
                _prevDecoder = OPUSIndicator;
                break;
            case APP_DECODER_SPEEX:
                laWidget_SetBackgroundType((laWidget*)SPXIndicator, LA_WIDGET_BACKGROUND_FILL);
                _prevDecoder = SPXIndicator;
                break;
            case APP_DECODER_ADPCM:
                laWidget_SetBackgroundType((laWidget*)PCMIndicator, LA_WIDGET_BACKGROUND_FILL);
                _prevDecoder = PCMIndicator;
                break;
            case APP_DECODER_FLAC:
                laWidget_SetBackgroundType((laWidget*)FLACIndicator, LA_WIDGET_BACKGROUND_FILL);
                _prevDecoder = FLACIndicator;
                break;
            default:break;
        }
    }    
    
}
void _display_track_name()
{
    if(laContext_GetActiveScreenIndex() == MainScreen_ID)
    {
        if(_displayData->track[0] != 0x00)
        {
            _updateLabelWidgetText(TrackNameLabel, (GFXU_CHAR*)(_displayData->track), _displayData->trackLen, 3);
        }
    }
    
}
void _display_artist_name()
{
    if(laContext_GetActiveScreenIndex() == MainScreen_ID)
    {
        if(_displayData->artist[0] != 0x00)
        {
            _updateLabelWidgetText(TrackArtistLabel, (GFXU_CHAR*)(_displayData->artist), _displayData->artistLen, 3);
        }
    }
}

void _display_album_name()
{
    if(laContext_GetActiveScreenIndex() == MainScreen_ID)
    {
        if(_displayData->album[0] != 0x00)
        {
            _updateLabelWidgetText(TrackAlbumLabel, (GFXU_CHAR*)(_displayData->album), _displayData->albumLen, 3);
        }
    }
}

void _display_prompt_message()
{
    if(laContext_GetActiveScreenIndex() == Welcome_ID)
    {
        if(_displayData->prompt_msg[0] != 0x00)
        {
             _updateLabelWidgetText(MessageLabel, _displayData->prompt_msg, _displayData->prompt_msg_len, 3);
        }
    }
}
/*Partial Done, Finish later*/
//void _display_file_list()
//{
//    if(laContext_GetActiveScreenIndex() == FileExplorer_ID)
//    {
//        if(_displayData->fileList != NULL)
//        {
//            ListNode *cur = _displayData->fileList;
//            while(cur != NULL)
//            {
//                laString tmp = laString_CreateFromCharBuffer(cur->val, fontList[3]);
//                _list_add_item(MusicList, &tmp, &MUSIC_ICON);
//                cur = (ListNode*)cur->next;
//            }
//        }
//        
//        // clear flag
//        _displayEvent &= ~(1 << DISPLAY_FILE_LIST);
//    }
//}
void _convertSeconds2TimeString(int32_t data, char* ret)
{
    ret[0] = '\0';
    uint16_t hour  = data/3600;
    uint8_t minute = (data-hour*3600)/60;
    uint8_t second = data - hour*3600 - minute*60;    
    uint8_t cur = 0;
    if(hour != 0)
    {
        sprintf(ret,"%d", hour);
        while(ret[cur] != '\0')
        {
            cur++;
        }
        ret[cur] = ':';
        ret[cur+1] = '\0';
        cur++;
    }

    if(minute != 0)
    {
        sprintf(&(ret[cur]), "%d", minute);
        while(ret[cur] != '\0')
        {
            cur++;
        }
        ret[cur] = ':';
        ret[cur+1] = '\0';
        cur++;
    }else
    {
        ret[cur] = '0';
        ret[cur+1] = '0';
        ret[cur+2] = ':';
        ret[cur+3] = '\0';
        cur += 3;
        
    }

    if(second != 0)
    {
        sprintf(&(ret[cur]), "%d", second);
        if(second/10 == 0)
        {
            char tmp = ret[cur];
            ret[cur] = '0'; 
            ret[cur+1] = tmp;
            ret[cur+2]='\0';
            
        }
        cur += 2;
    }else
    {
        ret[cur] = '0';
        ret[cur+1]='0';
        ret[cur+2] = '\0';
        cur += 2;
    }
}

void _display_track_length()
{
    if(laContext_GetActiveScreenIndex() == MainScreen_ID)
    {
        uint8_t cur=0;
        if(_displayData->track_length != 0)
        {
            char tracklength[16];
            _convertSeconds2TimeString(_displayData->track_length, tracklength);
            while(tracklength[cur]!='\0'){
                _displayData->track_duration[cur] = tracklength[cur];
                cur++;
            }
            _displayData->track_duration[cur]=0x00;
            _displayData->track_dur_len = cur;
           
        }
        else
        {
            char *tracklength = "00:00";
            while(tracklength[cur]!='\0'){
                _displayData->track_duration[cur] = tracklength[cur];
                cur++;
            }
            _displayData->track_duration[cur]=0x00;
            _displayData->track_dur_len = cur;
        }
        _updateLabelWidgetText(TrackLengthLabel, _displayData->track_duration, _displayData->track_dur_len, 1);
        
    }
}

void _display_play_time()
{
    if(laContext_GetActiveScreenIndex() == MainScreen_ID)
    {
        int32_t pb_width = ProgressBarRect->widget.rect.width;
        int32_t circle_indicator_offset_x = 0;
        if(_displayData->track_length != 0)
        {
            circle_indicator_offset_x = (pb_width*_displayData->playtime)/_displayData->track_length;
        }
        char playtime[16];uint8_t cur=0;
        _convertSeconds2TimeString(_displayData->playtime, playtime);
        while(playtime[cur]!='\0'){
               _displayData->playtime_str[cur] = playtime[cur];
               cur++;
        }
        _displayData->playtime_str[cur]=0x00;
        _displayData->playtime_str_len = cur;
        _updateLabelWidgetText(PlaytimeLabel, _displayData->playtime_str, _displayData->playtime_str_len, 1);


        
        int32_t newpos = circle_indicator_offset_x + ProgressBarRect->widget.rect.x;
        
        if(newpos > pb_width+ProgressBarRect->widget.rect.x)
            newpos = pb_width+ProgressBarRect->widget.rect.x;
        
        laWidget_SetX((laWidget*)CircleIndicator, newpos);
    } 
}



void _updateLabelWidgetText(laLabelWidget* lbl, GFXU_CHAR *la_data, uint32_t data_len, uint8_t fidx)
{
//    GFXU_CHAR gchar_data[64];
//    laString str;
//    int len = 0;
//    while(data[len]!='\0'){
//        gchar_data[len] = data[len];
//        len ++;
//    }
//    gchar_data[len] = 0x00;
//
//    str = laString_CreateFromBuffer(gchar_data, fontList[fidx]);
//
//    laLabelWidget_SetText(lbl, str);
//    laString_Destroy(&str);
    
    lbl->text.data = la_data;
    lbl->text.length = data_len;
    lbl->text.table_index = -1;
    lbl->text.capacity = 64;
    lbl->text.font = fontList[fidx];

    laWidget_Invalidate((laWidget*)lbl);
}


void _display_switch_screen()
{
    laContext_SetActiveScreen(_displayData->screenID);
}
/*** A wraper to append an item to a ListWidget ***/
//bool _list_add_item(laListWidget *listWidget, laString *item_str, GFXU_ImageAsset* icon)
//{
//    if(listWidget == NULL)
//    {
//        return false;
//    }
//    /*allocate an item in ListWidget*/
//    int idx = laListWidget_AppendItem(listWidget);
//    if(idx == -1)
//    {
//        return false;
//    }
//    
//    laListWidget_SetItemText(listWidget, idx, *item_str);
//    laListWidget_SetItemIcon(listWidget, idx, icon);
//    
//    return true;
//}