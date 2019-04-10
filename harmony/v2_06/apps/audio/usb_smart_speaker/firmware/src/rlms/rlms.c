//******************************************************************************
// File: rlms.c
//
// Description: This file contains non-time critical control code for rLMS 
//              module.
//
//              Features:  
//                1) nLMS algorithm 
//                2) Coefficient leakage to 0
//                2) Flat delay samples
//                3) Single error update
//                4) Robust error scaling
//
//******************************************************************************

#include "rlms.h"

/* Default configuration structure */
AecConfig defConfig = 
{
    .initAdaptCnt = 64,    //Samples
    .frameLength  = 8,
    .firLen       = 64,
    .mu0Q1d15     = 6554, // Step-Size m0 = 0.2          
    .flatSamp     = 0,
    .reverbQ1d15  = 32329 // 0.9866: Echo Decay time constant for 64ms, Q1d15
};

//*****************************************************************************
// _recalcMuStepDecay()
//
// Description:
//   Private function to calculate reverberation decay on the period of step
//   of step function: mu0 = sampleReverberationDecay^numStepSamples
//
// Arguments:
//   Aec  *aec - [in/out] Pointer to AEC object to update
//
// Return Value: None
//******************************************************************************
static void _recalcMuStepDecay(Aec *aec)
{
    int16_t i;
    q15 frtmp;
  
    frtmp = MAXFRACT16;   //Represent ~1.0 in Q1.15
    for(i = 0; i < aec->stepLen; i++)
    {
        // Calculate exponential: sampleReverberationDecay^numStepSamples 
        // --Use reverb decay frac. each sample
        frtmp = libq_q15_RoundL_q31(libq_q31_Mult2_q15_q15(frtmp, 
                                                          aec->reverbQ1d15)); 
    }
    aec->stepDecayQ1d15 = frtmp;  //Step decay (frac. change in step size)
} //End _recalcMuStepDecay()


//*****************************************************************************
// aecInit()
//
// Description:
//   Initializes AEC object with data from memTab and aecParam structures
//   --configuration set from defConfig.  The defConfig value change by
//     aecSetConfig()
//   
// Arguments:
//   Aec       *aec      - [out] Pointer to AEC object to initialize
//   AecMemRec *memTab - [in]  Pointer to memory table for x and h values
//   AecParam  *aecParam - [in]  Pointer to parameter structure
//
// Return Value: None
//*****************************************************************************/
#define APP_MAKE_BUFFER_DMA_READY  __attribute__((coherent)) __attribute__((aligned(16))) 
#ifdef KEEPHIST
AecHist APP_MAKE_BUFFER_DMA_READY aecHist;
#endif

void aecInit(Aec *aec, AecMemRec *memTab, AecParam *aecParam)
{
    int16_t i;
  
    // Set initial state of echo canceller
    aec->cancelEn = 0;
    aec->adaptEn  = 0;
  
    // Initialize maximum and current FIR filter length
    aec->xBufLen  = aecParam->maxFirLen;
    aec->numEsSteps = aecParam->numEsSteps;

    //Reference Power calculation
    aec->powerExp   = AECPOWEREXP;
    aec->powerQ1d31 = AECPOWERINITQ1D31;

    //Scaled Error - error
    //--Set to slightly above expected near speech level
    aec->scaleQ1d31 = AECSCALESINITQ1D31; 

    //Leaky LMS - to mitigate coef. est. bias
    aec->leakCntr   = AECLEAKPERIOD;

    aec->refMultQ8d8 = 1<<9;  //NOT USED
    aec->scaleDn =  AECSCALEEXP;      //Scale Exp - Converging
    aec->scaleUp =  AECSCALEEXPNAD;   //Scale Exp - Slow the Adapt

    // Initialize pointers to x delay circular buffer,
    //                     to h vector linear buffer, 
    //                     to norm error adapt array.
    // Initializes buffers to zero.            
    aec->xQ1d15   = (q15 *) memTab[AECMEM_X].base;
    aec->hQXdX16  = (q15 *) memTab[AECMEM_H].base;
    for(i = 0; i < aec->xBufLen; i++)
    {
        //Clear all
        aec->xQ1d15[i]  = 0; 
        aec->hQXdX16[i] = 0;
    }
    aec->normErrQXdX16 = (q15 *) memTab[AECMEM_NORMERR].base;
  
    //Default configuration (see above)
    if (aecSetConfig(aec, &defConfig) == 1)
    {
        //SYS_DEBUG(0,"ERROR - AEC Set Default Config\r\n");
    }
  
    aec->echoQ1d15     = aecParam->echoQ1d15; //Buffer Address - Echo Est.
    aec->errQ1d15      = 0;   //Error value after cancellation

    aec->converged = 0;     //from Scaled Error calculation 
    aec->hExp      = 0;     //Coefficient scaling

#ifdef KEEPHIST
    aec->hist      = &aecHist;
    aec->hist->numSamples = HISTMAXSAMPLES;
#endif
} //End aecInit()

//*****************************************************************************
// aecSetConfig()
//
// Description:
//   Dynamically sets new AEC "environment" parameters
//
// Arguments:
//   Aec   *aec - [out] Pointer to AEC object to update
//   AecParam *aecParam - [in]  Pointer to AEC Param structure with new AEC parameters
//
// Return Value:
//   0 - Valid parameters 
//   1 - Error in parameters
//*****************************************************************************/
int16_t aecSetConfig(Aec *aec, const AecConfig *aecConfig)
{
  int i;

  if (aecConfig->frameLength < 0) return 1;
  aec->frameLength = aecConfig->frameLength;

  // Validate input parameters
  if ((aecConfig->firLen > (aec->xBufLen)) ||
      ((aecConfig->firLen !=  32) &&
       (aecConfig->firLen !=  64) &&
       (aecConfig->firLen != 128) &&
       (aecConfig->firLen != 256) &&
       (aecConfig->firLen != 512)))
  {
      return 1;
  }

  if ((aecConfig->flatSamp > aecConfig->firLen) ||
      (aecConfig->flatSamp < 0))
  {
      return 1;
  }

  if (aecConfig->mu0Q1d15 < 0) return 1;

  if (aecConfig->reverbQ1d15 < 0) return 1;

  //Set new aec parameters
  aec->flatSamp      = aecConfig->flatSamp;
  aec->mu0Q1d15      = aecConfig->mu0Q1d15;
  aec->reverbQ1d15   = aecConfig->reverbQ1d15;
  if (aecSetFirLength(aec, aecConfig->firLen) == 1)
  {
      return 1;
  }

  //Clear the reference delay delay buffer
  for (i = 0; i < aec->xBufLen; i++) aec->xQ1d15[i] = 0;
  aec->xIndx       = 0;

  aec->sampleCount = 0;
  aec->adaptCount = 0;
  aec->initCount = aecConfig->initAdaptCnt;
  aec->echoQ1d15 = aecConfig->echoQ1d15;   

  return 0;

} //End aecSetConfig()


//*****************************************************************************
// aecSetFirLength()
//
// Description:
//   Dynamically validates and changes AEC echo tail length
//
// Arguments:
//   Aec   *aec - [out] Pointer to AEC object to update
//   int16_t firLength - [in]  New length of AEC echo tail in samples
//
// Return Value:
//   0 - if firLength is valid
//   1 - if firLength is invalid
//*****************************************************************************/
int16_t aecSetFirLength(Aec *aec, int16_t firLength)
{
  int i;

  if ((firLength  > (aec->xBufLen)) ||
      (firLength == 32)   ||                
      (firLength == 64)   ||                
      (firLength == 128)  || 
      (firLength == 256)  ||
      (firLength == 512))
  {
      //Valid FIR Length
      aec->firLen    = firLength;
  
      // Calculate step length 
      aec->stepLen   = firLength / aec->numEsSteps;
      _recalcMuStepDecay(aec);
      for (i = 0; i < aec->firLen; i++)
        aec->hQXdX16[i] = 0;
      aec->leakCntr = AECLEAKPERIOD;
      return 0;
  }
  else
  {
      //SYS_DEBUG(0,"ERROR - AEC Set FIR Length %d\r\n",firLength);
      return 1;
  }

} //End aecSetFirLength()


//*****************************************************************************
// aecSetCancelEn()
//
// Description:
//   Dynamically enable/disable AEC echo cancellation
//
// Arguments:
//   Aec   *aec - [out] Pointer to AEC object to update
//   int16_t cancelEn - [in]  1 - enable echo cancellation
//                            0 - disable echo cancellation
//
// Return Value:
//   1 - if firLength is valid
//   0 - if firLength is invalid
//*****************************************************************************/
void aecSetCancelEn (Aec *aec, int16_t cancelEn)
{
    //On the falling edge of cancelEn flag reset AEC and start over
    // H(n) is cleared, so reconvergence is necessary
    if ((cancelEn == 0) && (aec->cancelEn != 0))
    {
      aecReset(aec);
    }
  
    aec->cancelEn = cancelEn;
} //End aecSetCancelEn()


//*****************************************************************************
// aecSetAdaptEn()
//
// Description:
//   Dynamically enable/disable AEC echo cancellation
//
// Arguments:
//   Aec   *aec - [out] Pointer to AEC object to update
//   int16_t cancelEn - [in]  1 - enable echo cancellation
//                            0 - disable echo cancellation
//
// Return Value:
//   1 - if firLength is valid
//   0 - if firLength is invalid
//*****************************************************************************/
void aecSetAdaptEn (Aec *aec, int16_t adaptEn)
{
    aec->adaptEn = adaptEn;
} //End aecSetAdaptEn()


//*****************************************************************************
// aecGetConfig()
//
// Description:
//   Returns current setting of AEC "environment" parameters
//
// Arguments:
//   Aec   *aec - [in]   Pointer to AEC object
//   AecParam *aecParam - [out]  Pointer to AEC Param structure to 
//                               fill with current AEC parameters
// Notes:
//     Data member echoEstQ1d15 of Param structure is ignored (if it exists)
//     Data member maxFirLength of Param structure on exit contains
//     current tail length of AEC (not maximum tail length!)
//
// Return: None
//*****************************************************************************/
void aecGetConfig(Aec *aec, AecConfig *aecConfig)
{
    aecConfig->firLen       = aec->firLen;
    aecConfig->flatSamp     = aec->flatSamp;
    aecConfig->mu0Q1d15     = aec->mu0Q1d15;
    aecConfig->reverbQ1d15  = aec->reverbQ1d15;
} //End aecGetConfig() 


//*****************************************************************************
// aecReset()
//
// Description:
//   Resets state of AEC instance without changing configuration parameters
//
// Arguments:
//   Aec   *aec - [in] Pointer to AEC instance object
//
// Notes:
//
// Return: None
//*****************************************************************************/
void aecReset(Aec *aec)
{
    int i;
  
    //Clear vector of FIR coefficients
    for (i = 0; i < aec->xBufLen; i++)
    {
        aec->hQXdX16[i] = 0;
    }
  
    //Clear delay buffer
    for (i = 0; i < aec->xBufLen; i++)
    {
        aec->xQ1d15[i] = 0;
    }
  
    aec->xIndx       = 0;
  
    // Reset power estimate, level of scaled error, leakage counter
    aec->powerQ1d31 = AECPOWERINITQ1D31;
    aec->scaleQ1d31 = AECSCALESINITQ1D31; //Set to slightly above near speech level
    aec->leakCntr   = AECLEAKPERIOD;
  
    aec->errQ1d15 = 0;
} //End aecReset()