#ifndef DECODER_OPUS_SUPPORT_H
#define DECODER_OPUS_SUPPORT_H


#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "system/fs/sys_fs.h"
/******************Ogg Container Structures ********/


typedef enum{
    OPUS_SUCCESS = 1,
    OPUS_READ_ERROR,
    OPUS_STREAM_ERROR,
    OPUS_BUFF_ERROR,
    OPUS_STREAM_END,
    OPUS_PLAYBACK_ERROR,
    OPUS_OUT_OF_MEM_ERROR,
    OPUS_DISK_ERROR,
    OPUS_GENERAL_ERROR
}OPUS_ERROR_MSG;


typedef struct {
    char signature[8]; // "OpusHead"
    uint8_t version;
    uint8_t channels; /* Number of channels: 1..255 */
    uint16_t preskip;
    unsigned int input_sample_rate;
    uint16_t gain; /* in dB S7.8 should be zero whenever possible */
    uint8_t channel_mapping;
    /* The rest is only used if channel_mapping != 0 */
    int8_t nb_streams;
    int8_t nb_coupled;
    unsigned char stream_map[255];
} sOpusHeader;



typedef struct  __attribute__((packed))
{
	int32_t	pageCapture;		// should be OGG_ID_MAGIC
	int8_t	struct_ver;         // version of the Ogg file format. Should be 0 (RFC3533)
	int8_t	headerFlags;		// an eOggHeaderFlags value
	int64_t	granulePos;         // stream dependent position info
	int32_t	streamNo;           // logical bit stream identifier
	int32_t	pageNo;             // page sequence number
	int32_t	pageCrc;            // CRC32 checksum of the page
    uint8_t	pageSegments;		// number of page segments to follow
	uint8_t	segmentTbl[255];	// actually segmentTbl[pageSegments]; contains the lace 
	                            // values for all segments in the page
}sOggPageHdr;	// header of an Ogg page, full segment info included

typedef struct
{
	int		pktBytes;		// how many bytes in this packet
	int		pktSeqNo;		// packet sequence number
}sOpusPktDcpt;	            // decoder data packet descriptor	

typedef struct
{
//	sSpxRunDcpt	runDcpt;		// global info
	sOggPageHdr	pageHdr;        // current page header
	int		segIx;			    // current packet segment index in the current page
	int		pktIx;			    // current packet index, 0 -> ...
	int		prevBytes;		    // previous value of the bytes in the encoded output buffer 
}sOpusStreamDcpt;		        // info needed by the stream at run-time

typedef struct
{
//	int		framesPerPacket;	// frames per Ogg packet
//	int		frameSize;		    // size of the frames
	int		processedPktNo;		// counter of processed packets
	int		currPktNo;		    // number of the currently received packet from the stream
	int		nInBytes;		    // bytes available in the input buffer	
//	int		outFrameSize;
}opusDecDcpt;


/*****************************MACROS*******************************************/
#define OPUS_INPUT_BUFFER_SIZE (1024*2)
#define OPUS_OUTPUT_BUFFER_SIZE (1024*7)
#define OPUS_MAX_FRAME_SIZE (960*6) // 120ms @ 48Khz


bool isOPUSdecoder_enabled();


OPUS_ERROR_MSG OPUS_Initialize(const SYS_FS_HANDLE opus_file_handler);
OPUS_ERROR_MSG OPUS_Decoder(const uint8_t *input, uint16_t inSize, uint16_t *read, 
                              int16_t *output, uint16_t *written, uint16_t outSize);
int32_t        OPUS_DiskRead(uint8_t *inBuff);
int32_t        OPUS_GetSamplingRate();
uint8_t        OPUS_GetChannels();

void OPUS_Cleanup();
#endif