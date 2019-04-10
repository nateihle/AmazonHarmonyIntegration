#include "aac.h"


bool AAC_Initialize(void *heap,uint16_t size,uint8_t *ptr, SYS_FS_HANDLE aacFilehandle){return false;}

int16_t AAC_Decoder( uint8_t *input, uint16_t inSize, uint16_t *read, int16_t *output, uint16_t *written ){return (int16_t)-1;}

bool isAACdecoder_enabled()
{
	return false;
}

uint8_t AAC_GetChannels(){
    return 0;
}

int32_t AAC_GetSamplingFrequency(uint8_t *ptr){return (int32_t) 0;}

void AAC_RegisterDecoderEventHandlerCallback(SetReadBytesInAppData fptr){}