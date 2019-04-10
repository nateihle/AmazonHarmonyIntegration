#include "mp3.h"

bool MP3_Initialize ( void *heap, uint16_t size, SYS_FS_HANDLE mp3Filehandle)
{
    return false;
}

bool MP3_Decode ( uint8_t *input, uint16_t inSize, uint16_t *read, uint8_t *output, uint16_t *written )
{
    return false;
}

void MP3_FileSearch()
{
    return;
}

bool isMP3decoder_enabled()
{
    return false;
}

uint32_t MP3_GetAudioSize(){
    return 0;
}
uint8_t MP3_GetChannels(){
    return 0;
}
uint32_t MP3_UpdatePlaytime(){
    return 0;
}
void MP3_RegisterDecoderEventHandlerCallback(DecoderEventHandlerCB fptr)
{
}