//*****************************************************************************
// File: rlms.h
//
// Description: This is header file for Adaptive Echo Canceller 
//*****************************************************************************

#ifndef _RLMS_H_
#define _RLMS_H_

#include <stdbool.h>
#include <stdint.h>
#include "libq_c.h"
#include "dtd.h"

//Conversion factors
const long double TWOEXP9    = 512.0;
const long double TWOEXP15   = 32768.0;
const long double TWOEXP21   = 2095152.0; 
const long double TWOEXP25   = 33554432.0;
const long double TWOEXP31   = 2147483648.0;

/* Value used in reciprocal power calculation: Maximum fractional number in
   Q2.14 format */
#define  MAXFRACTQ2D14  0x3fff

/* Number of shifts to convert a fractional number from Q1.15 to Q7.9 format */
#define  Q7D9FMT        6

// Binary exponent of the number (use minimal echo tail length).  This is used
// in the power regularization factor computation (1/P(k+1)).  This fudge
// factor estimates the rectangular window power from the exponential 
// sample averaging window power 
#define  EXP16          4
#define  EXP32          5
#define  EXP64          6
#define  EXP128         7
#define  EXP256         8
#define  EXP512         9

// Number of shifts to scale reference power reciprocal
// for to estimate a 128 sample rectangular window  
#define  POWERRCPFMT    (EXP128+Q7D9FMT-1)

// Regularization parameter is used to limit value of
// normalization coefficient(1/power) when power level is low
// NOTE: lower half 32 bit value required to be 0
// for efficient assembly.  The value is a 16 bit  approximation
// of 1e-3
#define AECPOWERREGQ1D31  Fl2Fract32(0.001007080078125) // 0x00210000L

#define AECREFMINPOWERQ1D31   Fl2Fract32(.01)

// Initialization constant for power estimate
// Can be tuned to improve robustness after reset
#define AECPOWERINITQ1D31 Fl2Fract32(0)    // 0x000000000L

// Exponential averaging is used for power estimation:
// POWEREST[i]=POWEREST[i-1]*(1-alfa) + POWER[i]*alfa
// alfa = 2^-AECPOWEREXP
// tau = 1/(ln2*AECPOWEREXP), i.e. exp time constant
#define AECPOWEREXP      8

/* Number of bits in a 16-bit word */
#define  WORD16BITS     16

/* Number of guard-bits to protect from intermediate FIR overflow */
#define  AECFIRGUARDBITS 1

/* LSB used for rounding */
#define  LSB  1

// Leakage implemented as a multiplication by 2^-15 and truncation
// toward zero once per (AECLEAKFRAMES+1) frames
// This is equivalent to using leakage coefficient of
// 1 - 2^-(16+LOG2(FLENGTH)+LOG2(AECLEAKAGE+1)) = 1 - 2^-26
//#define AECLEAKPERIOD 32
#define  AECLEAKPERIOD  64

/* Leakage Value (1 - 2^(-15)) */
#define  AECLEAKVALUE   0x7fff

//********************************************************************
// Constants for robust scaling                                       
//********************************************************************
// Exponent of forgetting factor (Lambda) for scaled error            
// EXP - Adapt Falling
// HOLD - Adapt Rising
// EXPNAD - Not Adapting
#define AECSCALEEXP        7       //Falling error (converging)
#define AECSCALEHOLD	   12      //Not Adapting (NOT USED)
#define AECSCALEEXPNAD     19      //Rising error (near speech - diverging)

//Reciprocal of distribution coefficient (Beta = 0.60665) in Q2.14
#define AECSCALERBETAQ2D14 Fl2FxPnt16(1.0/0.60665, 14) // 0x697f

//Format factor of reciprocal of distribution coefficient
#define AECSCALERBETAF     1

//Allowance coefficient in Q2.14 format
#define AECSCALEK0         Fl2FxPnt16(1.1, 14) // 0x4666
#define AECSCALEK0Q2D14    AECSCALEK0

//Format factor of allowance coefficient
#define AECSCALEK0F        1

//Minimum value for scaled error
//#define AECSCALESMINQ1D31  Fl2FxPnt16(0.00048828125, 15)  //  0x00100000L
#define AECSCALESMINQ1D31  Fl2FxPnt32(0.0010070801, 31)  //  0x00210000L

// Initialization value for scaled error set to slightly above 
// echo level at max gain.
#define AECSCALESINITQ1D31 Fl2Fract32(0.1000061035)  // 0x0ccd0000L

// Reference Multiplier Value.  This will be used as the initial value
// when the refMultQ8d8 variable is allowed to adapt to the dynamic
// range of the coefficients.  Now it is fixed to the Q8d8 value 
// given here
// ***NOTE:  NOT USED***
#define AECREFMULT Fl2FxPnt16(1.5,8)

//Enumerated Table of AecMemRec - Names
typedef enum 
{
    AECMEM_X        = 0,  //xQ1d15
    AECMEM_H        = 1,  //hQXdX16
    AECMEM_NORMERR  = 2,  //normErrQXdX16
} AECMEMTABIDX;

//Memory Record
typedef struct _AecMemRec
{
  int16_t    size;              // size of allocation
  int16_t    alignment;         // alignment requirement (MAU)
  void       *base;             // base address of the allocated buffer
  char       padding[8];
} AecMemRec;

#define KEEPHIST
#ifdef KEEPHIST
#define HISTMAXSAMPLES 4096 
typedef struct _AecHist
{
    int16_t numSamples;
    int16_t nextIdx; //Circular buffering
    int16_t cancelEn[HISTMAXSAMPLES];
    int16_t adaptEn[HISTMAXSAMPLES];
    q15     errQ1d15[HISTMAXSAMPLES];       //Error from previous FIR computation 
    q15     errScaledQ1d15[HISTMAXSAMPLES]; //Scale limited error
    q15     sampleCount[HISTMAXSAMPLES];
    q15     xQ1d15[HISTMAXSAMPLES];
    q15     yQ1d15[HISTMAXSAMPLES];
    q15     yestQ1d15[HISTMAXSAMPLES];
    q31     xn2Q11d21[HISTMAXSAMPLES];
    q31     xn2InvQ1d15[HISTMAXSAMPLES];
    q31     signal1Q31[HISTMAXSAMPLES];
    char pad[12];
} AecHist;
#endif


//*****************************************************************************
// Aec
//
// Summary:
//   Structure to keep track of current state of echo cancellor function
//
// Description:
//   Structure to keep track of current state of echo cancellor function
//   Used for all calls to member functions of Aec Object
//*****************************************************************************
typedef struct _Aec 
{
    int32_t    sampleCount; //Sample count (for initial FIR fill and debug 
    int32_t    adaptCount;  //Number of adapt samples
    int16_t    initCount;   //Initialization count for unscaled adaptation

    int16_t    frameLength; //Length of audio frame

    int16_t    powerExp;    //Ref Power exponential average window exponent
    q31        powerQ1d31;  //Current ref. power estimate - exp. av. window

    int16_t    scaleUp;     //Increasing error scale exponent
    int16_t    scaleDn;     //Decreasing error scale exponent
    int16_t    converged;   //AEC has converged to echo path solution
    q31        scaleQ1d31;  //Scaled error level for robust scaling (Phi)

    int16_t    cancelEn;    //Echo cancelation enable flag
    int16_t    adaptEn;     //Coefficient update enable flag

    int16_t    xBufLen;     //Length of the reference delay buffer
    q15        *xQ1d15;     //Pointer to the delay buffer
    int16_t    xIndx;       //Current index of the delay buffer
  
    int16_t    firLen;      //Current FIR length for this instance
    int16_t    hExp;        //H coef. scaling
    q15        *hQXdX16;    //Pointer to FIR coefficients vector
  
    int16_t    flatSamp;    //Flat delay time in samples
    int16_t    stepLen;     //Number of samples in each exp-step
    q15        reverbQ1d15; //Reverberation decay for 1/Fs period
    q15        stepDecayQ1d15; //Reverberation decay for one step
    q15        mu0Q1d15;    //Initial value for mu in range (0;1)
    int16_t    numEsSteps;  //Number of Exponential Steps (mu)
  
    int16_t    leakCntr;    //Coef. Leakage frame counter
  
    q15       *echoQ1d15;   //Pointer to echo estimate buffer
    q15       errQ1d15;       //Error from previous FIR computation 
    q15       errScaledQ1d15; //Scale limited error
    q15       refMultQ8d8;  //Reference signal multiplier to scale the h value
                            //(can be adapted to the dynamic range of input)
                            // NOTE:  Not Used.

    q15 *      normErrQXdX16; //Normalized error step function array.

#ifdef KEEPHIST
    AecHist    *hist;
#endif
} Aec;

typedef struct _AecParams {
    int16_t    size;         //Size of this structure
    int16_t    maxAecTail;   //Acoustic echo tail length in samples
    int16_t    maxTxBlkDly;  //TX (acoustic) bulk delay in number of samples
                             //NOTE:  Not used.
} AecParams;

typedef struct _AecParam
{
    int16_t    maxFirLen;   //Maximum FIR length for this instance
    int16_t    numEsSteps;  //Size of normalization error value array
                            // (Number of Exp Steps)
    q15       *echoQ1d15;   //Pointer to echo estimate buffer
} AecParam;

typedef struct _AecConfig
{
    int16_t    initAdaptCnt; //Initial frame count to adapt without error scale
    int16_t    frameLength;  //Number
    int16_t    firLen;       //Initial length of FIR filter
    int16_t    flatSamp;     //Flat delay time in samples
    q15        mu0Q1d15;     //Step size value (mu0) in range [0;1)
    int16_t    numEsSteps;   //Number of Exp. decaying mu steps.
    q15        reverbQ1d15;  //Reverberation decay for 1/Fs period
    q15 *      echoQ1d15;    //Echo Estimate frame array
} AecConfig;

// Macros to directly access AEC object data members
#define aecNumAlloc(a)  2
  
//*****************************************************************************
/* Function aecSetCancelEn()
 *
 * Summary:
 *   Dynamically enable/disable AEC echo cancellation
 *
 * Description:
 *   Dynamically enable/disable AEC echo cancellation
 *
 * Arguments:
 *   Aec   *aec - [out] Pointer to AEC object to update
 *   int16_t cancelEn - [in]  1 - enable echo cancellation
 *                            0 - disable echo cancellation
 *
 * Return Value:
 *   None
 */
void aecSetCancelEn(Aec *aec, int16_t cancelEn);
  
//******************************************************************************
/* Function aecGetCancelEn()
 *
 * Summary:
 *   Get echo cancellation enable/disable flag of the AEC instance
 *
 * Description:
 *   Get echo cancellation enable/disable flag of the AEC instance
 *
 * Parameters:
 *   Aec   *a - [in] Pointer to AEC object
 *
 * Returns:
 *   1 - cancellation is enabled
 *   0 - cancellation is disabled
 *
 */
#define aecGetCancelEn(a)        ((a)->cancelEn)

//******************************************************************************
/* Function aecSetAdaptEn()
 *
 * Summary:
 *   Set adaptation enable/disable flag for the AEC instance
 *
 * Description:
 *   Set adaptation enable/disable flag for the AEC instance
 *
 * Parameters:
 *   Aec   *a - [out] Pointer to AEC object
 *   int16_t  b  - [in]  New value for adaptation flag
 *
 * Returns:
 *   none
 *
 */ 
void aecSetAdaptEn(Aec *aec, int16_t adaptEn); 

//******************************************************************************
/* Function aecGetAdaptEn()
 *
 * Summary:
 *   Get adaptation enable/disable flag of the AEC instance
 *
 * Description:
 *   Get adaptation enable/disable flag of the AEC instance
 *
 * Parameters:
 *   Aec   *a - [in] Pointer to AEC object
 *
 * Returns:
 *   1 - adaptation is enabled
 *   0 - adaptation is disabled
 *
 */
#define aecGetAdaptEn(a) ((a)->adaptEn)

//*****************************************************************************
/* Function aecSetEcho()
 *
 * Summary:
 *   Set pointer for echo estimate buffer for the AEC instance
 *
 * Description:
 *   Set pointer for echo estimate buffer for the AEC instance
 *
 * Parameters:
 *   Aec  *a - [out] Pointer to AEC object
 *   q15     *b - [in]  New pointer to echo estimate buffer
 *
 * Returns:
 *   none
 *
 */
#define aecSetEcho(a, b) ((a)->echoQ1d15=(b))


//******************************************************************************
/* Function aecGetEcho()
 *
 * Summary:
 *   Get pointer to echo estimate buffer of the AEC instance
 *
 * Description:
 *   Get pointer to echo estimate buffer of the AEC instance
 *
 * Parameters:
 *   Aec   *a - [in] Pointer to AEC object
 *
 * Returns:
 *   Pointer to echo estimate buffer
 *
 */
#define aecGetEcho(a)    ((a)->echoQ1d15)

//******************************************************************************
/* Function aecGetXBufLen()
 *
 * Summary:
 *   Get length of the delay buffer
 *
 * Description:
 *   Get length of the delay buffer
 *
 * Parameters:
 *   Aec   *a - [in] Pointer to AEC object
 *
 * Returns:
 *   Length of delay buffer
 *
 ******************************************************************************/
#define aecGetXBufLen(a)         ((a)->xBufLen)

//******************************************************************************
/* Function aecGetXBufPtr()
 *
 * Summary:
 *   Get pointer to the delay buffer
 *
 * Description:
 *   Get pointer to the delay buffer
 *
 * Parameters:
 *   Aec   *a - [in] Pointer to AEC object
 *
 * Returns:
 *   Pointer to the delay buffer
 *
 ******************************************************************************/
#define aecGetXBufPtr(a)         ((a)->xQ1d15)

//*****************************************************************************
/* Function aecGetHBufLen()
 *
 * Summary:
 *   Get length of the vector of FIR filter coefficients
 *
 * Description:
 *   Get length of the vector of FIR filter coefficients
 *
 * Parameters:
 *   Aec   *a - [in] Pointer to AEC object
 *
 * Returns:
 *   Length of the h vector of FIR filter coefficients
 *
 ******************************************************************************/
#define aecGetHBufLen(a)         ((a)->xBufLen)

//******************************************************************************
/* Function aecGetHBufPtr()
 *
 * Summary:
 *   Get pointer to the vector of FIR filter coefficients
 *
 * Description:
 *   Get pointer to the vector of FIR filter coefficients
 *
 * Parameters:
 *   Aec   *a
 *     [in] Pointer to AEC object
 *
 * Returns:
 *   Pointer to the vector of FIR filter coefficients
 *
 */
#define aecGetHBufPtr(a)         ((a)->hQ1d15)

//******************************************************************************
/* Function aecGetInitCount()
 *
 * Summary:
 *   Get pointer to the  initial scale adapt delay count
 *
 * Description:
 *   Get pointer to the  initial scale adapt delay count
 *
 * Parameters:
 *   Aec   *a
 *     [in] Pointer to AEC object
 *
 * Returns:
 *    int16_t initCount value
 *
 */
#define aecGetInitCount(a)         ((a)->initCount)

//******************************************************************************
/* Function aecGetMu0()
 *
 * Summary:
 *   Get initial step-size of AEC instance
 *
 * Description:
 *   Get initial step-size of AEC instance
 *
 * Parameters:
 *   Aec   *a - [in] Pointer to AEC object
 *
 * Returns:
 *   Initial step-size value
 *
 */
#define aecGetMu0(a)             ((a)->mu0Q1d15)

//******************************************************************************
/* Function aecGetFlatSamples
 *
 * Summary:
 *   Get flat delay in samples for the AEC instance
 *
 * Description:
 *   Get flat delay in samples for the AEC instance
 *
 * Parameters:
 *   Aec   *a - [in] Pointer to AEC object
 *
 * Returns:
 *   Flat delay in samples
 *
 */
#define aecGetFlatSamples(a)     ((a)->flatSamp)

//******************************************************************************
/* Funciton aecGetFrameLength
 *
 * Summary:
 *   Get length of frame in samples
 *
 * Description:
 *   Get length of frame in samples
 *
 * Parameters:
 *   Aec   *a - [in] Pointer to AEC object
 *
 * Returns:
 *   int16_t Length of sample frame.
 *
 */
#define aecGetFrameLength(a)       ((a)->frameLength)

//******************************************************************************
/* Funciton aecGetFirLength
 *
 * Summary:
 *   Get length of FIR filter in samples for the AEC instance
 *
 * Description:
 *   Get length of FIR filter in samples for the AEC instance
 *
 * Parameters:
 *   Aec   *a - [in] Pointer to AEC object
 *
 * Returns:
 *   Length of FIR filter in samples
 *
 */
#define aecGetFirLength(a)       ((a)->firLen)

//******************************************************************************
/* Function aecGetReverbDecay
 *
 * Summary:
 *   Get reverberation decay on the period of one sample
 *
 * Description:
 *   Get reverberation decay on the period of one sample
 *
 * Parameters:
 *   Aec   *a - [in] Pointer to AEC object
 *
 * Returns:
 *   Reverberation decay on the period of one sample (in Q1.15 format)
 *
 */
#define aecGetReverbDecay(a) ((a)->reverbQ1d15)

//******************************************************************************
/* Function aecGetStepDecay
 *
 * Summary:
 *   Get reverberation decay on the period of one step
 *
 * Description:
 *   Get reverberation decay on the period of one step
 *
 * Parameters:
 *   Aec   *a - [in] Pointer to AEC object
 *
 * Returns:
 *   Reverberation decay on the period of one step (in Q1.15 format)
 *
 */
#define aecGetStepDecay(a) ((a)->stepDecayQ1d15)

//******************************************************************************
/* Function aecGetPower
 *
 * Description:
 *   Get current power estimate value
 *
 * Parameters:
 *   Aec   *a - [in] Pointer to AEC object
 *
 * Returns:
 *   Current power estimate value (in Q1.31 format)
 *
 */
#define aecGetPower(a) ((a)->powerQ1d31)

//******************************************************************************
/* Function aecGetScale
 *
 * Summary:
 *   Get current scaled error value
 *
 * Description:
 *   Get current scaled error value
 *
 * Parameters:
 *   Aec   *a
 *     [in] Pointer to AEC object
 *
 * Returns:
 *   Current scaled error value (in Q1.31 format)
 *
 */
#define aecGetScale(a) ((a)->scaleQ1d31)

/* Function prototypes                                                */
#ifdef  __cplusplus
extern "C" {
#endif


//*****************************************************************************
/* Function aecInit()
 *
 * Summary:
 *   Initializes AEC object with data from memTab and aecParam structures
 *
 * Description:
 *   Initializes AEC object with data from memTab and aecParam structures
 *
 * Arguments:
 *   Aec  *aec
 *     [out] Pointer to AEC object to initialize
 *   AecMemRec *memTab
 *     [in]  Pointer to memory table
 *   AecParam  *aecParam
 *     [in]  Pointer to parameter structure
 *
 * Return Value:
 *   None
 */
void aecInit(Aec *aec, AecMemRec *memTab, AecParam *aecParam);


//*****************************************************************************
/* Function aecProc()
 *
 * Summary:
 *   Performs adaptive echo cancellation using flesrnLMS algorithm
 *
 * Description:
 *   Performs adaptive echo cancellation using flesrnLMS algorithm
 *
 * Arguments:
 *   Aec  *aec - [in/out] Pointer to AEC object
 *   q15 *refInQ1d15 - [in] Pointer to reference input buffer
 *   q15 *micInQ1d15 - [in] Pointer to speech input buffer
 *   q15 *micOutQ1d15 - [out] Pointer to speech output buffer
 *
 * Return Value:
 *   None
 */
void aecProc(Aec *aec, 
             q15 *refInQ1d15,
             q15 *micInQ1d15,
             q15 *micOutQ1d15);

//*****************************************************************************
/* Function aecSetFirLength()
 *
 * Summary:
 *   Dynamically validates and changes AEC echo tail length
 *
 * Description:
 *   Dynamically validates and changes AEC echo tail length
 *
 * Arguments:
 *   Aec   *aec   - [out] Pointer to AEC object to update
 *   AtiInt16 firLength - [in]  New length of AEC echo tail in samples
 *
 * Return Value:
 *   1 - if firLength is valid
 *   0 - if firLength is invalid
 */
int16_t  aecSetFirLength(Aec *aec, int16_t firLength);


//*****************************************************************************
/* Function aecSetConfig()
 *
 * Summary:
 *   Dynamically sets new AEC "environment" parameters
 *
 * Description:
 *   Dynamically sets new AEC "environment" parameters
 *
 * Arguments:
 *   Aec   *aec - [out] Pointer to AEC object to update
 *   AecParam *aecParam - [in]  Pointer to AEC Param structure with new AEC parameters
 *
 *
 * Return Value:
 *   1 - if input parameters are valid
 *   0 - otherwise
 *
 * Notes:
 *     Following data members of Param structure are ignored
 *     - cancelEn
 *     - adaptEn
 *     - echoEstQ1d15 (if it exists)
 *     Data member maxFirLength of Param structure is used as a new
 *     tail length of AEC (not maximum tail length!)
 */
int16_t  aecSetConfig(Aec *aec, const AecConfig *aecConfig);

//*****************************************************************************
/* Function aecGetConfig()
 *
 * Summary:
 *   Returns current setting of AEC "environment" parameters
 *
 * Description:
 *   Returns current setting of AEC "environment" parameters
 *
 * Arguments:
 *   Aec   *aec   - [in] Pointer to AEC object
 *   AecParam *aecParam - [out] Pointer to AEC Param structure to fill 
 *                              with current AEC parameters
 * Notes:
 *     Data member echoEstQ1d15 of Param structure is ignored (if it exists)
 *     Data member maxFirLength of Param structure on exit contains
 *     current tail length of AEC (not maximum tail length!)
 *
 * Return Value:
 *   None
 *
 */
void aecGetConfig(Aec *aec, AecConfig *aecConfig);

//*****************************************************************************
/* Function aecReset()
 *
 * Summary:
 *   Resets state of AEC instance without changing configuration parameters
 *
 * Description:
 *   Resets state of AEC instance without changing configuration parameters
 *
 * Arguments:
 *   Aec   *aec - [in]   Pointer to AEC instance object
 * Notes:
 *
 * Return Value:
 *   None
 *
 */
void     aecReset(Aec *aec);


//******************************************************************************
//DSP Functions - acceleration of rlms_t.c functions
//******************************************************************************
void libq_Macq31q15q15(q31 *sum, q15 *indata1, q15 *indata2);

//int16_t DSP_FilterLMS16(
//                     q15 refInQ1d15,     //x(n)
//                     int16_t micInQ1d15, //y(n)
//                     int16_t * hQ1XdX16, //H(n-1) 
//                     int16_t * xQ1d15,   //X(n-1),
//                     int16_t * errQ1d15, //e(n-1) 
//                     int firLen);         //FIR Length 

//NOTE:  This routine is Delayed Error LMS (DELMS) and does not converge
int16_t mips_nlms16(
                     int16_t refInQ1d15, //x(n)
                     int16_t micInQ1d15, //y(n)
                     int16_t * hQ1XdX16, //H(n-1) 
                     int16_t * xQ1d15,   //X(n-1),
                     int16_t * errQ1d15, //e(n-1) 
                     int firLen);         //FIR Length 

//WORKING VERSION of standard LMS compatible with nLMS with Robust error scaling 
//and leaky H coefficients 
//int16_t mips_norm_lms16(
//                     int16_t in, 
//                     int16_t ref,
//                     int16_t *coeffs,
//                     int16_t *delayline,
//                     int16_t *error, 
//                     int16_t K, 
//                     int16_t mu);
int16_t mips_errn_lms16(int16_t in, int16_t ref, int16_t *coeffs, int16_t *delayline,
                      int16_t *error, int16_t K, int mu);
void mips_hadj_lms16(int16_t in, int16_t ref, int16_t *coeffs, int16_t *delayline,
                      int16_t *error, int16_t K, int mu);

#ifdef  __cplusplus
}
#endif

#endif /* _RLMS_H_ */
