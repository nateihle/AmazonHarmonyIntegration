#ifndef __TEMPLATE_ADCHS_REGISTERS_PIC32MK_H_
#define __TEMPLATE_ADCHS_REGISTERS_PIC32MK_H_
  
typedef struct {
    uint32_t bits;
    volatile unsigned int clr;
    volatile unsigned int set;
    volatile unsigned int inv;
} four_words_register_t;

typedef struct {
    __ADCFLTR1bits_t bits;
    volatile unsigned int clr;
    volatile unsigned int set;
    volatile unsigned int inv;
} four_words_reg_ADCFLTR1_t;

typedef struct {
    __ADCCMPCON1bits_t bits;
    volatile unsigned int clr;
    volatile unsigned int set;
    volatile unsigned int inv;
} four_words_reg_ADCCMPCON1_t;

   
typedef struct adchs_cmp_reg {
    four_words_register_t ADCCMPEN;
    four_words_register_t ADCCMP;
} adchs_cmp_register_t;

typedef struct {
    __ADCDSTATbits_t bits;
    volatile unsigned int clr;
    volatile unsigned int set;
    volatile unsigned int inv;
} four_words_reg_ADCDSTAT_t;

/* Indy*/
typedef struct adchs_regs {
    __ADCCON1bits_t ADCCON1;              /* 0x000 */
    volatile unsigned int ADCCON1CLR;
    volatile unsigned int ADCCON1SET;
    volatile unsigned int ADCCON1INV;
    __ADCCON2bits_t ADCCON2;              /* 0x010 */
    volatile unsigned int ADCCON2CLR;
    volatile unsigned int ADCCON2SET;
    volatile unsigned int ADCCON2INV;
    __ADCCON3bits_t ADCCON3;              /* 0x020 */
    volatile unsigned int ADCCON3CLR;
    volatile unsigned int ADCCON3SET;
    volatile unsigned int ADCCON3INV;
    four_words_register_t ADCTRGMODE;           /* 0x030 */
    four_words_register_t ADCIMCONx[4];         /* 0x040 - 0x070 */
    four_words_register_t ADCGIRQENx[2];        /* 0x080 - 0x090 */
    four_words_register_t ADCCSSx[2];           /* 0x0A0 - 0x0B0 */
    four_words_register_t ADCSTATx[2];          /* 0x0C0 - 0x0D0 */
    adchs_cmp_register_t adccmpx[4];            /* 0x0E0 - 0x150 */
    four_words_register_t OFFSET1[4];           /* 0x160 - 0x190 */
    four_words_reg_ADCFLTR1_t ADCFLTRx[4];      /* 0x1A0 - 0x1D0 */
    four_words_register_t OFFSET2[2];           /* 0x1E0 - 0x1F0 */
    four_words_register_t ADCTRGx[7];           /* 0x200 - 0x260 */
    four_words_register_t OFFSET3;              /* 0x270 */
    four_words_reg_ADCCMPCON1_t ADCCMPCONx[4];  /* 0x280 - 0x2B0 */   
    four_words_register_t OFFSET4[4];           /* 0x2C0 - 0x2F0*/
    four_words_register_t ADCBASE;              /* 0x300 */
    four_words_reg_ADCDSTAT_t ADCDSTAT;             /* 0x310 */
    four_words_register_t ADCCNTB;              /* 0x320 */
    four_words_register_t ADCDMAB;              /* 0x330 */
    four_words_register_t ADCTRGSNS;            /* 0x340 */
    four_words_register_t ADCxTIME[6];          /* 0x350 - 0x3A0 */
    four_words_register_t OFFSET5;              /* 0x3B0 */
    four_words_register_t ADCEIENx[2];          /* 0x3C0 - 0x3D0 */
    four_words_register_t ADCEISTATx[2];        /* 0x3E0 - 0x3F0 */
    four_words_register_t ADCANCON;             /* 0x400 */
    four_words_register_t OFFSET6[31];          /* 0x410 - 0x5F0 */
    four_words_register_t ADCDATAx[54];         /* 0x600 - 0x950 */
    four_words_register_t OFFSET7[42];          /* 0x960 - 0xBF0 */
    four_words_register_t ADCxCFG[8];           /* 0xC00 - 0xC70 */
    four_words_register_t OFFSET8[8];          /* 0xC80 - 0xCF0 */
    four_words_register_t ADCSYSCFGx[2];        /* 0xD00 - 0xD10*/
} adchs_register_t;

#ifndef BIT
#define BIT(x)   (1 << (x))
#endif

//******************************************************************************
/* Function :  ADCHS_ExistsRegisters_pic32mk

  Summary:
    Implements default variant of ADCHS_ExistsRegisters_pic32mk

  Description:
    This template implements the default variant of the ADCHS_ExistsRegisters_pic32mk function.
*/

PLIB_TEMPLATE bool ADCHS_ExistsRegisters_pic32mk( ADCHS_MODULE_ID index )
{
    return true;
}

#endif /* __TEMPLATE_ADCHS_REGISTERS_PIC32MK_H_ */
