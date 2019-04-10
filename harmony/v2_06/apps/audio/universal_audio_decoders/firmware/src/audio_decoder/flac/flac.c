#include "flac.h"

bool FLAC_Initialize (SYS_FS_HANDLE flacFilehandle, void *input_buffer)
{
    return false;
}

bool FLAC_Decoder( uint8_t *input, uint16_t inSize, uint16_t *read, uint8_t *output, uint16_t *written  )
{
    return false;
}

bool isFLACdecoder_enabled()
{
    return false;
}

uint8_t FLAC_GetChannels(){
    return 0;
}

void FLAC_RegisterDecoderEventHandlerCallback(DecoderEventHandlerCB fptr)
{
    return;
}

int32_t FLAC_GetBitRate()
{
    return 0;
}
int32_t FLAC_GetSamplingRate() 
{
    return 0;
}

int32_t FLAC_GetBlockSize()
{
    return 0;
} 

void FLAC_Cleanup()
{
    return;
}