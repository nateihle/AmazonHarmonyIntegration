

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
extern "C" {
#endif
#include "system/fs/sys_fs.h"
//DOM-IGNORE-END
    
typedef void(*SetReadBytesReadFlagInAppData)(int32_t val, bool b);
typedef int32_t(*GetReadBytesInAppData)();

bool isWMAdecoder_enabled();
void WMA_Initialize(SYS_FS_HANDLE wmaFilehandle, uint32_t inputBufferSize);
int32_t WMA_SamplingFrequency_Get(void);
int32_t WMA_BitRate_Get(void);
int32_t WMA_GetHeaderPacketOffset();
int16_t WMA_Decoder( uint8_t *input, uint16_t inSize, uint16_t *read, int16_t *output, uint16_t *written );
void WMA_FreeMemory();
uint8_t WMA_GetChannels();
void WMA_RegisterAppCallback(SetReadBytesReadFlagInAppData fptr0, GetReadBytesInAppData fptr1);
#ifdef __cplusplus
}
#endif

