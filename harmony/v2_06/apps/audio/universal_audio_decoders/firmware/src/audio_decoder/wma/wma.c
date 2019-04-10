#include "wma.h"

int32_t wma_packet_offset = 0;

void WMA_Initialize(SYS_FS_HANDLE wmaFilehandle, uint32_t inputBufferSize)
{
}

int16_t WMA_Decoder( uint8_t *input, uint16_t inSize, uint16_t *read, int16_t *output, uint16_t *written )
{
    return (int16_t) -1;
}

int32_t WMA_SamplingFrequency_Get(void)
{
    return (int16_t) -1;
}

bool isWMAdecoder_enabled()
{
    return (bool) false;
}
uint8_t WMA_GetChannels(){
    return 0;
}

void WMA_FreeMemory(){
    
}

int32_t WMA_GetHeaderPacketOffset(){
    return (int8_t)0;
}
int32_t WMA_BitRate_Get(void)
{
    return (int32_t) 0;
}
void WMA_RegisterAppCallback(SetReadBytesReadFlagInAppData fptr0, GetReadBytesInAppData fptr1)
{
}