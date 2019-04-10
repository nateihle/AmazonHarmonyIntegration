#include "opus_support.h"


/********************************* Local variables*****************************/

// OPUS file always play at 48Khz, see OPUS spec
static const uint16_t   OPUS_FS = 48000;



/********************OPUS DECODER FUNCTIONS IMPLEMENTATION*********************/
   
OPUS_ERROR_MSG OPUS_Initialize(const SYS_FS_HANDLE opus_file_handle){
    return OPUS_GENERAL_ERROR;
}

OPUS_ERROR_MSG OPUS_Decoder(const uint8_t *input, uint16_t inSize, uint16_t *read, 
                              int16_t *output, uint16_t *written, uint16_t outSize){
    return OPUS_GENERAL_ERROR;
}


int32_t        OPUS_DiskRead(uint8_t *inBuff){
   
    return -1;
}


/*********************************************************************/
int32_t OPUS_GetSamplingRate(){return OPUS_FS;}
bool    isOPUSdecoder_enabled(){return false;}
uint8_t OPUS_GetChannels(){return 1;}
void    OPUS_Cleanup()
{
}
