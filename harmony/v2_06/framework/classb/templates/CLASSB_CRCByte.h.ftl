// *****************************************************************************
// Flash CRC16 Test Generator Polynomials
/*
  Description:
    The value of the generator polynomial is used as an input parameter for the
    CLASSB_CRCFlashTest() function.
    It specifies what polynomial to be used for the CRC calculation.

    Following is a list of some of the most commonly used 16 bit Generator Polynomials
    that can be used.
    Any other polynomial that has the required fault detection capabilities can be used.
*/    
#define CRC_08_GEN_POLY             0x07     /* x^8 + x^2 + x + 1*/    
#define CRC_16_GEN_POLY             0x8005   /* x^16 + x^15 + x^2 + 1*/    
#define CRC_CCITT_GEN_POLY          0x1021   /* x^16 + x^12 + x^5 + 1*/    
#define CRC_32_GEN_POLY             0x04C11DB7  /* x^32 + x^26 + x^23 + x^22 + x^16 + x^12 + x^11 + x^10 + x^8 + x^7 + x^5 + x^4 + x^2 + x + 1*/    
// definitions
#define     FLASH_CRC8_MASK     ((1L<<8)-1)    		    // mask to retain the useful CRC result for 8 bit polynomial
#define     FLASH_CRC8_MSB      (1L<<(8-1))			    // mask to obtain the MSb-1, transport to the MSb.
#define     FLASH_CRC16_MASK    ((1L<<16)-1)    		// mask to retain the useful CRC result for 16 bit polynomial
#define     FLASH_CRC16_MSB     (1L<<(16-1))			// mask to obtain the MSb-1, transport to the MSb.
#define     FLASH_CRC32_MASK    ((1L<<32)-1)    		// mask to retain the useful CRC result for 32 bit polynomial
#define     FLASH_CRC32_MSB     (1L<<(32-1))			// mask to obtain the MSb-1, transport to the MSb.

/*********************************************************************
 * Function:        unsigned int Flash_CRC16(unsigned char* pBuff, 
                                             unsigned int crcPoly, 
                                             unsigned int crcReg, 
                                             unsigned int bSize)
 *
 * PreCondition:    pBuff valid pointer
 * 
 * Input:           - pBuff:    buffer to calculate CRC over
 *                  - crcPoly:  the generator polynomial to be used
 *                  - crcReg:   initial value of the CRC LFSR (seed)
 * 					- bSize:    buffer size, bytes
 * 
 * Output:          value of the CRC
 * 
 * Side Effects:    None
 * 
 * Overview:        Shifts bytes through the CRC shift register.
 * 
 * Note:            Simple (and slow) CRC16 calculation directly based
 *                  on the hardware LFSR implementation.
 *                  
 ********************************************************************/
unsigned int Flash_CRC16(unsigned char* pBuff, unsigned int crcPoly, unsigned int crcReg, unsigned int bSize);


