//*****************************************************************************
// File: rlms_t.c  
//
// Description: This file contains the time-critical portion of the AEC code.
//              The AEC is implemented with following features:
//
//              1) nLMS algorithm with exponential decaying relaxation step 
//                 function on fixed number of coef. blocks
//              2) H coef. leakage to prevent coef. overflow
//                 and h coef. scaling to maintain dynamic range.
//              3) Robust error scaling to prevent divergence
//
//******************************************************************************

#include "rlms.h"
#include "system_config.h"
#include "system_definitions.h"

#undef DEBUGLMS
#if defined(DEBUGLMS) 
#if defined(CPUCORETIMER)
extern int reportCnt;
extern int start;
#endif
#endif

//Library inline functions
inline extern q15 libq_q15_Abs_q15(q15);
inline extern q15 libq_q15_Add_q15_q15(q15, q15);
inline extern q15 libq_q15_DivisionWithSaturation_q15_q15(q15, q15);
inline extern q15 libq_q15_ExtractH_q31(q31);
inline extern q15 libq_q15_ExtractL_q31(q31);
inline extern q15 libq_q15_MultiplyR2_q15_q15(q15, q15);
inline extern q15 libq_q15_Negate_q15(q15);
inline extern q15 libq_q15_RoundL_q31(q31);
inline extern q15 libq_q15_Sub_q15_q15(q15, q15);
inline extern q31 libq_q31_DepositH_q15(q15);
inline extern q31 libq_q31_DepositL_q15(q15);
inline extern q15 libq_q15_ShiftRightRound_q15_i16(q15, i16);
inline extern q15 libq_q15_ShiftRight_q15_i16(q15, i16);
inline extern q31 libq_q31_Mult2_q15_q15(q15, q15);
inline extern q15 libq_q15_ShiftLeft_q15_i16(q15, i16);
inline extern q31 libq_q31_Multi_q15_q31(q15 argAQ1d15, q31 argBQ1d31);
inline extern q31 libq_q31_Mac_q31_q15_q15(q31, q15, q15);

#if 0
//Local function prototypes
inline static void _aecLeak(Aec *aec, 
                  q15 *refInQ1d15,
                  q15 *micInQ1d15,
                  q15 *micOutQ1d15);

#endif
static inline void _aecFrameLoop(Aec *aec, 
                  q15 *refInQ1d15,
                  q15 *micInQ1d15,
                  q15 *micOutQ1d15);

static inline void _aecRefPower(Aec *aec, 
                  q15 *refInQ1d15, 
                  int16_t i);
static inline void _aecScaleErr(Aec *aec, q15 *errQ1d15); 

#if 0
static inline void _aecErrStepAdjust(Aec *aec, 
                  q15  errQ1d15);

static inline void _aecHexp(Aec *aec);
#endif


static q15     _muQ1d15 = 0;
#ifndef DEBUGLMS
//static q15 _muNormErrQ1d15 = 0;
//static q15 _normErrQ1d15 = 0;
//static q31 _xPowerQ1d31 = 0;
//Compute the normalized error

#endif //DEBUGLMS

static q15     _errScaledQ1d15 = 0;
static q15     _yEstQ1d15= 0;
int cycles, timerStart, timerEnd;
static int32_t _xn2Q11d21 = 0;
static int32_t _prodQ2d30  = 0;
static int16_t _xInQ15;
static int16_t _yInQ15;
static double  _xn2;
static double  _xn2Inv;
static int16_t _adjQ1d15;
static int16_t _xn2InvQ1d15;
static int16_t * _hQXdX16;   //H(n-1) 
static int16_t * _xQ1d15;    //X(n-1)
static int16_t _errQ1d15;  //norm scaled e(n-1),out:e(n) 
static int16_t firNumTaps;
//static int16_t * normErrQXdX16;


//*****************************************************************************
//                                                                            
// aecProc()                                                                  
//                                                                            
// Description:                                                               
//   Performs adaptive echo cancellation using exponential step nLMS
//   with flat delay and robustness error normalized step size. 
//                                                                            
//   The processing steps are:                                                
//                                                                            
//     A) For Each Frame:                                                     
//        1. If cancelEn, leak the coefficients                               
//                                                                            
//     B) For every 1 sample(s) 
//                                                                            
//        1. X^2 power estimate and delay vector X update 
//           a) Update x vector power estimate.                               
//           b) Update delay vector x by 1 sample(s).                         
//                                                                            
//        2. Yhat, Echo estimate and output subtraction
//           a) If cancelEn, compute echo estimates for the 1 sample time(s)    
//           b) Output the echo-cancelled speech for 1 sample time(s)           
//                                                                            
//        3. Robust scale normalized error,en
//           a) If adaptEn, compute both scaled normalized errors             
//           b) If adaptEn, compute the new error scaling (Phi)               
//                                                                            
//        4. H Coefficient update
//           a) If adaptEn, update the coefficient vector h                   
//                                                                            
// Arguments:                                                                 
//   Aec *aec            - [in/out] Pointer to AEC object                                         
//   q15 *refInQ1d15     - [in] Pointer to reference input buffer                                 
//   q15 *micInQ1d15  - [in] Pointer to speech input buffer                                    
//   q15 *micOutQ1d15 - [out] Pointer to speech output buffer                                  
//                                                                            
// Return Value:  None                                                                     
//                                                                            
//*****************************************************************************
//void __attribute__((optimize("O0"))) 
void aecProc( Aec *aec, q15 *refInQ1d15,     /* Reference signal */
                        q15 *micInQ1d15,  /* Speech input     */
                        q15 *micOutQ1d15 ) /* Speech output    */
{
    //Leakage of Coefficients 
    //_aecLeak(aec,refInQ1d15,micInQ1d15,micOutQ1d15);
  
    //Frame Loop
    _aecFrameLoop(aec,refInQ1d15,micInQ1d15,micOutQ1d15);

    //SYS_PRINT("End of sample %d\r\n",aec->sampleCount);
  
    //Coefficient magnitude scaling
    //_aecHexp(aec);

} //End aecProc()




//*****************************************************************************
// _aecLeak()
//
// Arguments:
//   Aec *aec            - [in/out] Pointer to AEC object
//   q15 *refInQ1d15     - [in] Pointer to reference input buffer
//   q15 *micInQ1d15  - [in] Pointer to speech input buffer
//   q15 *micOutQ1d15 - [out] Pointer to speech output buffer
//
// Return Value: None
//*****************************************************************************
#if 0
void _aecLeak(Aec *aec, 
              q15 *refInQ1d15,    
              q15 *micInQ1d15,
              q15 *micOutQ1d15)
{
    int16_t j;
    q15 *hQXdX16;
  
    // 16-bit fixed-point temporary variables
    int16_t   frTmp1QXdX16;
    const int16_t  firLen   = aec->firLen;    /* FIR filter length          */
  
    // H coefficient need to be leaked toward 0 every frame to prevent overflow
    // during narrowband signals (The H would tend to a small number of
    // large coefficients, which could have values > 1). 
    // AECLEAKPERIOD set to 64 --> 256ms leak @ 8Khz
    // NOTE: Narrowband signals (music) underdetermine the filter.
    if (aec->cancelEn)
    {
        // Multiply by AECLEAKVALUE with rounding toward zero at the leakage
        // schedule every AECLEAKPERIOD frames
        hQXdX16 = aec->hQXdX16;
        aec->leakCntr--;
        if (aec->leakCntr == 0)
        {
            aec->leakCntr = AECLEAKPERIOD;
            for(j = 0; j < firLen; j++)
            {
                /* Apply Leak with rounding toward 0 */
                frTmp1QXdX16 = libq_q15_MultiplyR2_q15_q15(
                                    libq_q15_Abs_q15(*hQXdX16), 
                                    AECLEAKVALUE);
        
                if (*hQXdX16 < 0) frTmp1QXdX16 = -frTmp1QXdX16;
        
                *hQXdX16++  = frTmp1QXdX16;
            }
        }
    }
}
#endif

//*****************************************************************************
//
// _aecFrameLoop()
//
// Summary:
//   Process 1 frame 1 sample(s) at a time
//
// Arguments:
//   Aec *aec            - [in/out] Pointer to AEC object
//   q15 *refInQ1d15     - [in] Pointer to reference input buffer
//   q15 *micInQ1d15  - [in] Pointer to speech input buffer
//   q15 *micOutQ1d15 - [out] Pointer to speech output buffer
//
// Return Value: None
//
//*****************************************************************************
//__attribute__((always_inline)) 
void _aecFrameLoop(Aec *aec, q15 *refInQ1d15,
                   q15 *micInQ1d15,
                   q15 *micOutQ1d15)
{
    int16_t   i;

    for(i = 0; i < aec->frameLength; i++)
    {
        if (aec->adaptEn == 1)
        {
            //Compute Exponentially decaying the H adjustment value,
            //   aec->normErrQ1d15[numEsSteps], across the taps:
            //_aecErrStepAdjust(aec, aec->errQ1d15);  //scaled error
            //_muNormErrQ1d15 = aec->normErrQXdX16[0]; //mu*e(n-1)/P(n)
                _muQ1d15 = aec->mu0Q1d15;
        } //aec-adaptEn block
        else
        {
            _muQ1d15 = 0; //No Adapt
        }

        //CYCLE COUNT
        asm volatile("mtc0   $0,$9");
        asm volatile("mfc0   %0, $9" : "=r"(timerStart));

        //CANCEL ECHO
        //      x(n) - speaker out reference (refInQ1d15)
        //      y(n) - Microphone input (micInQ1d15)
        //      e(n-1) - Echo estimate error from previous iteration
        //      H(n-1) - previous FIR Coef. vector 
        //      X(n-1) - previous reference delay vector.

        //  1) Update the input delay buffer (X(n) vector))
        //       X(n-1) = x(0..K-1)  --> X(n) = x(IN,0..K-2)
        //  2) Compute the echo estimate and error:
        //       y_est(n) = H(n)*X(n)  -- estimated echo
        //       e(n) = y(n)-y_est(n)  -- Input without echo reduction
        //
        _xInQ15 = refInQ1d15[i];
        _yInQ15 = micInQ1d15[i];
        _hQXdX16 = aec->hQXdX16;
        _xQ1d15 = aec->xQ1d15;     //X(n-1)
        firNumTaps = aec->firLen;     //Number of taps
        _errQ1d15 = _yInQ15;
        aec->errQ1d15 = _yInQ15;

        // ECHOCANCELLER
        // At time current sample time n: 
        //            Updates X(n-1)->X(n)
        //            returns e(n) = H(n)X(n)
        _yEstQ1d15 = (int16_t) mips_errn_lms16(_xInQ15,          //x(n) or in
                                               _yInQ15,          //y(n), echo    
                                               _hQXdX16,         //H(n-1) 
                                               _xQ1d15,          //X(n-1)
                                               &_errQ1d15,       //Return e(n)
                                               firNumTaps,       //K
                                               _muQ1d15);        //<NOT USED>
        if (aec->cancelEn)
        {
            
            // H ERROR ADAPT
            //  3) TODO: Robust error scale limiting 
            //           e(n) -> es(n), scale limited error
            //  4) Adapt the H value using scaled error :  
            //       H(n) = H(n-1) + mu*es(n-1)*X(n-1)/P(n-1)
            aec->errQ1d15 = _errQ1d15;
            //Reference Vector Power
            //--Est. by Exponential power window scaled
            //  alpha = 1^(-128)  --> scale by 128 by get ~rect. window
            _aecRefPower(aec, refInQ1d15, i);   

            //Echo Activity Detector
            if ( (aec->powerQ1d31)  < AECREFMINPOWERQ1D31 )
            {
                //Cancel/No Adapt
                _muQ1d15 = 0; 
            }
            else
            { 
                //_xPowerQ1d31 = DSP_VectorSumSquares16_32(aec->xQ1d15, aec->firLen, 0);
                //aec->powerQ1d31 = _xPowerQ1d31;
                
                //TODO: Scale limit the error from changing too rapidlly
                //--after the aec->initCount
                //_aecScaleErr(aec, &_errScaledQ1d15);
                _errScaledQ1d15 = aec->errQ1d15; //NO SCALING
                aec->errScaledQ1d15 = _errScaledQ1d15;

                //_xn2Q7d9 = DSP_VectorSumSquares16(_xQ1d15, firNumTaps, 6);
                //_xn2Q7d9 += ((int16_t)(AECREFMINPOWERQ1D31 >> (16+6));
                //_xn2 = _xn2Q7d9/(double)TWOEXP9 + .01;

                //Error Normalization
                //xn2Q7d9 = DSP_VectorSumSquares16(xDelayQ1d15, LMSTAPS, 6);
                int jj;
                _xn2Q11d21 = 0;
                for (jj=0; jj<firNumTaps; jj++)
                {
                    _prodQ2d30  = _xQ1d15[jj]*_xQ1d15[jj]; //product Q2.30
                    _xn2Q11d21 += (_prodQ2d30 + 0x0100) >> 9;       //scale and round Q11.21
                }
                _xn2 = (double)_xn2Q11d21/(double)TWOEXP21 + .01;
                _xn2Inv = 1./_xn2;
                if (_xn2Inv >= 1.000) 
                {
                    //Too small
                    _xn2InvQ1d15 = 0x7FFF;
                    _adjQ1d15 = 0; 
                }
                else
                {
                    _xn2InvQ1d15 = (int16_t)(_xn2Inv * 32768.0);
                    _adjQ1d15   = (( ((int32_t)_errScaledQ1d15 * 
                                      (int32_t)_xn2InvQ1d15) 
                                    + 0x4000) >> 15);
                    aec->adaptCount++;
                }
                
                // H(n+1) Update, t time n:
                //    computes Hadj(n) = mu*adj(n)*X, adj(n) = es(n)/|X(n)|^2  
                //    Updates H(n+1) = H(n) + Hadj(n)
                mips_hadj_lms16(_xInQ15,    //x(n) or in
                                _yInQ15,    //y(n), echo    
                                _hQXdX16,   //H(n) -> H(n+1) 
                                _xQ1d15,    //X(n-1)
                                &_adjQ1d15, //H Adjust value e(n)/|X|^2
                                firNumTaps, //K
                                _muQ1d15);  //mu0

            } //End Echo Cancel/H Adapt
            micOutQ1d15[i] = aec->errQ1d15;  //Unscaled e(n) = y(n)-yest(n))

            if (aec->adaptCount == 1024)
            {
                aec->converged = 1;
            }
        } //cancelEn Block
        else
        {
            micOutQ1d15[i] = micInQ1d15[i];  //y(n)
        }
        aec->sampleCount++;

        asm volatile("mfc0   %0, $9" : "=r"(timerEnd));
        cycles = 2*(timerEnd - timerStart);  // eval cycles for compare real function

#ifdef KEEPHIST
        if (aec->adaptCount < HISTMAXSAMPLES)
        {
            int idx; 
            idx = aec->hist->nextIdx;

            aec->hist->cancelEn[idx]       = aec->cancelEn;
            aec->hist->adaptEn[idx]        = aec->mu0Q1d15==0.?0:aec->adaptEn;
            aec->hist->errQ1d15[idx]       = aec->errQ1d15;
            aec->hist->errScaledQ1d15[idx] = aec->errScaledQ1d15;
            aec->hist->sampleCount[idx]    = aec->sampleCount-1; 
            aec->hist->xQ1d15[idx] = _xInQ15; 
            aec->hist->yQ1d15[idx] = _yInQ15; 
            aec->hist->yestQ1d15[idx]   = _yEstQ1d15;
            aec->hist->xn2Q11d21[idx]   = _xn2Q11d21;
            aec->hist->xn2InvQ1d15[idx] = _xn2InvQ1d15;

            //q31 powerQ16d16 = 0;
            //powerQ16d16 = (aec->powerQ1d31 + 0x8000) >> 9;  //16 - 7, 128 window 
            aec->hist->signal1Q31[idx] = _adjQ1d15<<16;

            aec->hist->nextIdx++;
            if (aec->hist->nextIdx >= HISTMAXSAMPLES)
            {
                aec->hist->nextIdx = 0;
            }
        }
        else
        {
            Nop();
        }
#endif
    } //End Frame Loop 
} /* end aecFrameLoop */


//*****************************************************************************
// _aecRefPower()
//
// Summary:
//   Update the power estimate using a exponential averaging filter on the current
//   sample i with alpha = 2^(-128)
//
// Arguments:
//   Aec *aec         - [in/out] Pointer to AEC object
//   q15 *refInQ1d15  - [in] Pointer to reference input buffer
//   int16_t i        - index to current reference frame sample
//
// Return Value: //   None
//*****************************************************************************
void _aecRefPower(Aec *aec, q15 *refInQ1d15, int16_t i)
{
    q31 framePowerQ1d31;
    q31 frTmp1Q1d31, frTmp2Q1d31;
    q31 fr32tmp1;
  
    //  Short Term Power (exponential window) estimate computed every 1
    //  sample(s)  The exponential window is used for efficiency and stability.
    //  This power estimate is used to scale the error updates in the filter
    //  update equation.
    //
    //  forgetting factor Lambda = (1 - 2^-powerExp)
    //  Let N=powerExp, and k=sample index.
    //
    //  P(k) = (1-2^-N)P(k-1) + (2^-N)*X(k)*x(k)
    //       = P(k-1) - 2^-N*P(k-1) + 2^-N*x(k)*x(k) 
    //  
    //  NOTE:  This is an approximation of the X^2 value. The exponential
    //         filter will weight the more current values of x more than the 
    //         older values.
    
    //P(k-1)
    framePowerQ1d31 = aec->powerQ1d31; // Reference X vector power 
    // P(k-1)*(2^-N) 
    fr32tmp1 = libq_q31_ShiftRight_q31_i16(framePowerQ1d31, aec->powerExp);
    // P(k-1)-P(k-1)**(2^-N)
    frTmp1Q1d31 = libq_q31_Sub_q31_q31(framePowerQ1d31, fr32tmp1);
    // x(k)^2
    fr32tmp1 =libq_q31_Mult2_q15_q15(refInQ1d15[i],refInQ1d15[i]);
    // x(k)^2*(2^-N)
    frTmp2Q1d31 = libq_q31_ShiftRight_q31_i16(fr32tmp1, aec->powerExp);
    // P(k-1)-P(k-1)(2^-N) + x(k)^2*(2^-N)
    framePowerQ1d31 = libq_q31_Add_q31_q31(frTmp1Q1d31, frTmp2Q1d31);

    aec->powerQ1d31 = framePowerQ1d31;

} //End _aecRefPower()

#if 1
//****************************************************************************
// _aecScaleErr()
//
// Summary:
//   The error value is exponentially filtered and limited to remain within
//   a range (scaled error) based on prior values.
//
// Arguments:
//   Aec *aec         - [in/out] Pointer to AEC object
//   q15 *errQ1d15)   - [in/out] absolute error/scaled error 
//
// Return Value: None
//
//****************************************************************************/
void _aecScaleErr(Aec *aec, q15 *errQ1d15) 
{

    /* Number of shifts to be done for normalization */
    int16_t   inorm;
  
    /* 16-bit fixed-point temporary variables */
    int16_t   frTmp1Q1d15;
    int16_t   frTmp1QXdX16;
  
    //Current upper limit for error magnitude
    q31 frScaleQXdX32;

    //Temporary variable for normalized error scale
    q31 scaleNormQXdX32;
    q15 scaleNormQXdX16;

    //32-bit temporary variables
    q31 frTmp1Q1d31;
    q31 frTmp1QXdX32, frTmp2QXdX32;

    //Scale forgetting factor exponent */
    int16_t   scaleExp;
  
    //The following locals are used (instead of the direct object variables)
    //to minimize the overhead incurred when compiling "for loop" limits 
    //const int16_t  firLen   = aec->firLen;    /* FIR filter length          */
  
    //==========================================================================
    // Robustness error step limiting by error scaling
    // 
    // NOTE: Scale is designated as "Phi" in the following equations.
    // 
    // If adaptEn
    // 
    //   1.  Compute limited error for the current sample 
    // 
    //       C = min(K0*Phi, |err1|)
    // 
    //       if C = K0*Phi
    //            err1 = K0*Phi*sign(err1)
    //       else
    //            err1 unchanged
    // 
    //   2.  Compute new error scaling (Phi) using an exponential window:
    // 
    //       New Phi = (1-2^-K)*Phi + (2^-K)*C/beta
    //
    //       where,
    //             2^-K = forgetting factor,
    //             K = scaleExp,
    //
    //   3.  Make sure that the new error scale (New Phi) does not drop below
    //       the minimum scale value:
    //       New Phi = max(New Phi, AECSCALESMINQ1D31)
    //
    // NOTES:  
    //   1)  The time constant exponent for 16ks/s must be twice that 
    //       for 8ks/s to have the same rate.
    //==========================================================================

    //Delay the initial error scaling
    if((aec->adaptEn) && (aec->initCount!=0)) 
    {
        aec->initCount--;
    }
  
    //Error Scale Filtering
    if ( aec->adaptEn && (aec->initCount)==0 )
    {
        // Normalize current scale (Phi) in 1.31 to range [0.25:0.5)
        //Exponent of scale factor (Phi)
        inorm = Fx32Norm(aec->scaleQ1d31)-1;
        //Mantissa of scale factor (Phi) in Q1-inorm.31+inorm format
        scaleNormQXdX32   = libq_q31_ShiftLeft_q31_i16 (aec->scaleQ1d31, inorm);
        //Round scale factor (Phi) to 16 bits : Q1-inorm.15+inorm format
        scaleNormQXdX16   = libq_q15_RoundL_q31(scaleNormQXdX32);
    
        //Compute error limit, K0*Phi : Since K0 is in Q2.14 format and Phi
        //is in Q1-inorm.15+inorm format, the result of the multiply will be in
        //Q2-inorm.30+inorm format.  It is then shifted by 1 (AECSCALEK0F) to
        //convert it to Q1-inorm.31+inorm format. */
        frScaleQXdX32 =
          libq_q31_ShiftLeft_q31_i16(libq_q31_Mult2_q15_q15(scaleNormQXdX16, 
                                                           AECSCALEK0Q2D14), 
                                     AECSCALEK0F);
    
        // |err|) in Phi format 
        frTmp1Q1d15   = libq_q15_Abs_q15(*errQ1d15); // |err|
        // Load |err| to high word of double
        frTmp1Q1d31 = libq_q31_DepositH_q15(frTmp1Q1d15);
        // Normalize |err| to same format as scale (Phi):
        //   Q1-inorm.31+inorm format */
        frTmp1QXdX32 = libq_q31_ShiftLeft_q31_i16(frTmp1Q1d31, inorm);
    
        // C = min(K0*Phi, |err|) */
        // If K0*Phi < |err|, then set C = K0*Phi
        if (frScaleQXdX32 < frTmp1QXdX32)
        {
            //C = K0*Phi
            frTmp1QXdX32 = frScaleQXdX32;
      
            //err = C*sign(err)
            if (*errQ1d15 < 0)
            {
                /* negative value */
                *errQ1d15 = libq_q15_RoundL_q31(libq_q31_Negate_q31(
                               libq_q31_ShiftRight_q31_i16(frTmp1QXdX32, inorm)));
            }
            else
            {
                /* positive or 0 */
                *errQ1d15 =  libq_q15_RoundL_q31(
                               libq_q31_ShiftRight_q31_i16(frTmp1QXdX32, inorm));
            }
        }
    
        //========================================================================
        //Compute new error scaling (Phi) using an exponential window */
        //New Phi = (1-2^-K)*Phi + (2^-K)*C/beta */
        
        // Round C to 16 bits : Q1-inorm.15+inorm format 
        frTmp1QXdX16   = libq_q15_RoundL_q31(frTmp1QXdX32);
    
        // C/beta 
        // 1/beta is in Q2.14 format, and C is in Q1-inorm.15+inorm
        // format, so C/beta in is in Q2-inorm.30+inorm format 
        frTmp1QXdX32 = libq_q31_Mult2_q15_q15(frTmp1QXdX16, AECSCALERBETAQ2D14);
    
        //Determine forgetting factor based on rising or falling values.
        //If C > Phi, then scale up, else scale down.
        //-- C and Phi are both in Q1-inorm.31+inorm format
        scaleExp=
            (libq_q31_ShiftLeft_q31_i16(frTmp1QXdX32,AECSCALERBETAF)>scaleNormQXdX32)
            ?aec->scaleUp:aec->scaleDn;
    
        //(2^-(K-AECSCALERBETAF)) * C/beta: Q1-inorm.31+inorm format 
        frTmp1QXdX32 =
          libq_q31_ShiftRight_q31_i16(frTmp1QXdX32, 
                                      (int16_t)(scaleExp-AECSCALERBETAF));
    
        //(2^-K) * Phi : Q1-inorm.31+inorm format 
        frTmp2QXdX32 = libq_q31_ShiftRight_q31_i16(scaleNormQXdX32, 
                                                   scaleExp);
    
        //Phi-((2^-K)*Phi) = (1-2^-K)*Phi 
        frTmp2QXdX32 = libq_q31_Sub_q31_q31(scaleNormQXdX32, 
                                            frTmp2QXdX32);
    
        //Phi(k+1) = (1-2^-K)*Phi(k) + (2^-K)*C/beta */
        frTmp1QXdX32 = libq_q31_Add_q31_q31(frTmp1QXdX32, frTmp2QXdX32);
    
        //Restore the normalized new Phi to Q1.31 format 
        frTmp1Q1d31 = libq_q31_ShiftRight_q31_i16(frTmp1QXdX32, inorm);
    
        // Make sure that the new error scale (New Phi) does not drop below    */
        // the minimum scale value                                             */
        if (frTmp1Q1d31>AECSCALESMINQ1D31) 
        {
           aec->scaleQ1d31 = frTmp1Q1d31;
        }
        else 
        {
           aec->scaleQ1d31 = AECSCALESMINQ1D31; 
           aec->converged  = true;  //Converged when scale reaches minimum
        }
    } //End Adapt Block

} //End _aecScaleErr() 
#endif

#if 0
//******************************************************************************
// _aecErrStepAdjust()
//
// Summary:
//   Compute the adjustment value:  mu*_errScaled(n)/P(n)
//
// Arguments:
//   Aec *aec            [in/out] Pointer to AEC object
//
//                        Inputs:   
//                            aec->initCount
//                            aec->adaptEn
//                            aec->powerQ1d31
//                        Calculates: aec->normErrQXdX16[aec->numEsSteps]
//   q15 errQ1d15        [in] e(n-1) 
//
// Return Value:    None
//****************************************************************************/
void _aecErrStepAdjust(Aec *aec, q15  errQ1d15)
{
    int16_t   j;

    //Number of shifts to be done for normalization
    static int16_t   inorm;

    //Power normalization factor
    static q15 pwrNormFactQ1d15;
    static q15 pwrNormFactQ7d9;
  
    //16-bit fixed-point temporary variables
    static int16_t   frTmp1QXdX16;
  
    //32-bit power estimate */
    static q31 framePowerQ1d31;
    static q31 framePowerQXdX32;
  
    //32-bit temporary variables */
    static q31 frTmp1Q7d25;
    static q31 frTmp1QXdX32; 
    //static q31 frTmp2QXdX32;
  
    // Updated H(n) adjustment value
    // 
    //  step*mu0*scaleErr[j]/P(k+1))
    //       normalized error = scaleErr[j]/P(k+1)
    //
    aec->normErrQXdX16[0] = errQ1d15;
    framePowerQ1d31 = aec->powerQ1d31;
  
    if (aec->adaptEn && aec->initCount==0)
    {
        //Adapt and wait for initCount to count down to 0
    
        //P(k+1) in Q1.31 with calculated normalization left shift
        //--no significant bits lost
        inorm = Fx32Norm(framePowerQ1d31);
  
        //P(k+1) in Q1-inorm.31+inorm format 
        framePowerQXdX32 = libq_q31_ShiftLeft_q31_i16(framePowerQ1d31, inorm);
    
        //Reciprocal of the power, P(k+1)
        //Since the power estimate is normalized in the range [0.5:1), its
        //inverse can be computed by dividing it into a Q2.14 formatted 1
        //(MAXFRACTQ2D14).  This gives you the value 0.5/norm(P(k+1)).  In order
        //to recover the true value of 1/P(k+1), this result must be shifted
        //left 1 bit and then converted back to Q1.15 
        pwrNormFactQ1d15 = libq_q15_DivisionWithSaturation_q15_q15(
                               MAXFRACTQ2D14, 
                               libq_q15_RoundL_q31(framePowerQXdX32));
    
        //Number of shifts to convert the Q1.31 exponential window reciprocal power
        //factor for an approximation of a Q7.9 rectangular window
        //power normalization factor.  
        //NOTE:  POWERRCPFMT = EXP128+Q7D9FMT-1.
        //  1) The EXP128 term converts the exponential window inverse
        //     power factor to a rectangular window power factor, i.e. x(k)**2.  
        //  2) Q7D9FMT shifts the inverse power factor to a Q7.9 format.  
        //  3) -1 accounts for the shift necessary to get the true inverse 
        //      1/P(k+1) (since the computed inverse was actually 
        //      0.5/norm(P(k+1)) */
        inorm = POWERRCPFMT - inorm;
        pwrNormFactQ7d9 = libq_q15_ShiftRight_q15_i16(pwrNormFactQ1d15, inorm);
    
        //Normalize the error for the current input sample:
        //  1)Multiply by the power normalization factor: err1/P(k+1)
        //  2)Multiply by the initial step coefficient:   mu0*err1/P(k+1) */
    
        //Compute normalized err1/P(k+1) in Q7.25  
        //--Q1.15 * Q7.9 -- Q8.24, Mult2 performs << 1 -->Q7.9
        frTmp1Q7d25 = libq_q31_Mult2_q15_q15(aec->normErrQXdX16[0], 
                                             pwrNormFactQ7d9);
        //Convert result to Q7-eExp.25+eExp 
        inorm = Fx32Norm(frTmp1Q7d25); //eExp = inorm;
        frTmp1QXdX32 = libq_q31_ShiftLeft_q31_i16(frTmp1Q7d25, inorm);
        //Update inorm to reflect how many bits to shift to go from Q7.9 to
        //Q1.15 format 
        inorm = inorm - Q7D9FMT;  //inorm = eExp - 6
        if (inorm > WORD16BITS)
        {
            //inorm is out of range (err1/P(k+1) too small)
            //Set err1*mu0/P(k+1) to 0 (No Regularization)
            aec->normErrQXdX16[0] = 0;
        }
        else
        {
            //err1/P(k+1): Q7-eExp.9+eExp format
            frTmp1QXdX16 = libq_q15_RoundL_q31(frTmp1QXdX32);
            //mu0*err1/P(k+1): Q7-inorm.9+inorm format 
            frTmp1QXdX32 = libq_q31_Mult2_q15_q15(frTmp1QXdX16, aec->mu0Q1d15);
            //Include hExp scaling in normalized updates:
            //  Q7-inorm.9+inorm format
            inorm = inorm - aec->hExp;
            //Scale mu0*err1/P(k+1) to Q1-hExp.31+hExp format
            frTmp1QXdX32 = libq_q31_ShiftRight_q31_i16 (frTmp1QXdX32, inorm);
            //mu0*err1/P(k+1) in Q1-hExp.15+hExp format
            aec->normErrQXdX16[0] = libq_q15_RoundL_q31(frTmp1QXdX32);
        }
    
        // Exponential decay function on adjustment value
        // --normalized errors decay over filter length to maintain stability  
        for (j = 1; j < aec->numEsSteps; j++)
        {
            // err1(j)=step*mu0*err1(j-1)/P(k+1): Q1-hExp.15+hExp format 
            aec->normErrQXdX16[j] = libq_q15_MultiplyR2_q15_q15(
                                            aec->normErrQXdX16[j-1], 
                                            aec->stepDecayQ1d15);
        }
    } //End AdaptEn
} //End _aecErrStepAdjust()
#endif


#if 0
//******************************************************************************
// aecHexp()
//
// Summary:
//
// Arguments:
//   Aec *aec            - [in/out] Pointer to AEC object
//   q15 *refInQ1d15     - [in] Pointer to reference input buffer
//   q15 *micInQ1d15  - [in] Pointer to speech input buffer
//   q15 *micOutQ1d15 - [out] Pointer to speech output buffer
//
// Return Value: //   None
//****************************************************************************/
void _aecHexp(Aec *aec)
{
    int16_t   j;
    int16_t   inorm;
  
    int16_t   frTmp1QXdX16, frTmp2QXdX16;

    // Pointer to FIR coefficient vector
    q15 *hQXdX16;
  
    // 32-bit temporary variables
    q31 frTmp1QXdX32;
  
    const int16_t  firLen   = aec->firLen;    //FIR filter length
  
    //==========================================================================
    // Compute the h coefficient exponential scale factor (hExp).          
    //                                                                    
    // This was added to give more dynamic range to the AEC filter   
    // coefficients.  hExp is computed by finding the maximum coefficient
    // magnitude and normalizing it in the range [0.25:0.5].  The       
    // remaining coefficients are scaled the same amount.  Care must be
    // taken in the filter update equation to properly scale the error
    // updates with hExp                                             
    //==========================================================================
    if (aec->adaptEn==1  && aec->initCount==0)
    {

        hQXdX16  = aec->hQXdX16;

        //Initialize the max to 0 
        frTmp1QXdX16 = 0;
  
        //Search for maximum absolute value in the h coefficient vector
        for(j = 0; j < firLen; j++)
        {
          //Load the next coefficient magnitude 
          frTmp2QXdX16 = libq_q15_Abs_q15(hQXdX16[j]);
  
          //Check to see if the this is the largest coefficient so far */
          if (frTmp2QXdX16 > frTmp1QXdX16) frTmp1QXdX16 = frTmp2QXdX16;
        }
    
        //Load into 32 bit high word to add LSB
        frTmp1QXdX32=libq_q31_DepositH_q15(frTmp1QXdX16);
  
        //Add LSB to avoid problem with zero
        frTmp1QXdX32 = libq_q31_Add_q31_q31(frTmp1QXdX32, LSB);
    
        //Choose a normalization factor that will scale the coefficients to the
        //range [0.25:0.5]
        inorm = Fx32Norm(frTmp1QXdX32) - 1;
    
        // If the maximum coefficient is < 0.25, limit the exponential scale for
        // this iteration (this frame) to 1 bit.  This essentially limits the rate
        // at which the coefficients can be scaled.  hExp cannot exceed 4 bits.  If
        // the maximum coefficient is > 0.5, the coefficients are scaled down one
        // bit to force the maximum coefficient to the range [0.25:0.5).  Thus,
        // hExp is limited to the range [-1:4] */
        if (inorm > 0)
        {
          //Limit hExp to 4 and the allow only 1 bit scaling (up) per frame */
          if ((inorm > 1) && (aec->hExp < 4))
            inorm = 1;
          else
            inorm = 0;
        }
    
        //Scale the coefficients */
        for(j = 0; j < firLen; j++)
        {
          hQXdX16[j] = libq_q15_ShiftLeft_q15_i16(hQXdX16[j], inorm);
        }
    
        //Update hExp */
        aec->hExp = aec->hExp + inorm;

    } //End Adapt Block
} //End _aecHexp()
#endif