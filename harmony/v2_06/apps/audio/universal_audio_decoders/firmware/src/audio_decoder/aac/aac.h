
//DOM-IGNORE-BEGIN
#ifdef __cplusplus
extern "C" {
#endif
//DOM-IGNORE-END
#include "system/fs/sys_fs.h"
#include <stdbool.h>
#include <stdint.h>



#define AACDECODER_STATE_SIZE 11032                // in Bytes
#define AAC_FRAME_HEADER_SIZE     7                   // in Bytes apple header frame


#define MAX_STACK           0x5000                 // in Bytes
#define	AAC_PROFILE         1
#define STACK               1

#define INPUT_BUF_SIZE              (6144*2/8)           // in Bytes
#define FRAME_SIZE                  1024                 // in Samples
#define AAC_FRAME_HEADER_SIZE       7                    // in Bytes
#define AACDECODER_STATE_SIZE       11032                // in Bytes

    
typedef void (*SetReadBytesInAppData)(int32_t val);
    
bool    isAACdecoder_enabled();
bool    AAC_Initialize(void *heap,uint16_t size,uint8_t *ptr, SYS_FS_HANDLE aacFilehandle);
int32_t AAC_GetSamplingFrequency(uint8_t *ptr);
uint8_t AAC_GetChannels();
int16_t AAC_Decoder( uint8_t *input, uint16_t inSize, uint16_t *read, int16_t *output, uint16_t *written );
void    AAC_RegisterDecoderEventHandlerCallback(SetReadBytesInAppData fptr);
#ifdef __cplusplus
}
#endif