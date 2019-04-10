/*****************************************************************************
*
* Name: upsample.c
*
* Synopsis: Upsamples a signal.
*
*****************************************************************************/

#include "upsample.h"

__inline__ void Upsample_0_OrderFilter(DRV_I2S_DATA32 audioSample,
                                       //DRV_I2S_DATA32 * outBuffer, 
                                       q31 * outDataQ1d31,
                                       int upsampleRatio)
{
    int j;

    q31 audioDataQ1d31;

    //16bit data in 1 channel MSB justified to 32 bit word
    audioDataQ1d31 = (audioSample.rightDataPad + audioSample.leftData) 
                        & 0xFFFF00000;
    audioSample.rightDataPad = audioDataQ1d31;
    audioSample.leftData     = audioDataQ1d31;

    for (j=0; j < upsampleRatio; j++)
    {
        //DRV_I2S_DATA32 * outDataPtr = outBuffer + j;

        //NOTE:  MONO Mode Stereo Data.
        //outBuffer[j].leftData = 
        //        audioSample.leftData;
        //outBuffer[j].rightDataPad = 
        //        audioSample.rightDataPad;
        outDataQ1d31[j] = audioDataQ1d31; 
    }  
} //End _Upsample_0_Order_Filter

//******************************************************************************
// Upsample_NX_Polyphase())
//
//    upsampleRatio:
//        the interpolation factor (must be >= 1)
//
//    numTapsPerPhase:
//        the number of taps per polyphase filter, which is the number of taps
//        in the filter divided by factor_L.
//
//    hQ1d31:
//        pointer to the array of coefficients for the resampling filter.  (The
//        number of total taps in hPhase is upsampleRatio*numTapsPerPhase
//        Number of filter coef. must is divisible by upsampleRatio.)
//
// Input/Outputs:
//
//    audioSampleQ1d31:
//        The input stereo audio sample in DRV_I2S_DATA32 format. 
//        Range:  -1 to 1 
//        NOTE:  16 bit Q1d15 data should be deposited in the MSB of the 
//
//    xQ1d31: pointer to the input delay line (Length is numTapsPerPhase)
//
// Outputs:
//
//    outDataBuffer:
//        pointer to the upsample output buffer
//
// NOTE:
//  1) The order of each polyphase filter is #coef/upsampleRatio
//  2) The inputs and output are float
//
//******************************************************************************
__inline__ void Upsample_NX_PolyPhase(
                   int         upsampleRatio,    //L upsample factor
                   int         numTapsPerPhase,  //#taps/phase = #elements/phase 
                   q31 * const hQ1d31,           //coefficients
                   q31         audioQ1d31,       //Input sample to filter
                   q31 * xQ1d31,       //Delay line array (#elements/phase) 
                   q31 * outDataQ1d31) //Upsample output buffer
{
    int32_t tap; 
    int phaseNum;

    //q31 audioDataQ1d31;

    //16bit data in 1 channel MSB justified to 32 bit word
    //audioDataQ1d31 = (audioSample.rightDataPad + audioSample.leftData) 
    //                    & 0xFFFF00000;
    //audioSample.rightDataPad = audioDataQ1d31;
    //audioSample.leftData     = audioDataQ1d31;

    q31 *hPhaseQ1d31;    //Phase coefficients
    q31  sumQ1d31=0;
    q31 *outQ1d31 = outDataQ1d31;

    // Shift x delay line 
    // --lose the last sample, add new sample at 0
    //for (tap = numTapsPerPhase- 1; tap > 0; tap--)  
    for (tap = 0; tap < numTapsPerPhase-1; tap++)  //newer moved to older
    {
        *(xQ1d31+1) = *(xQ1d31); //index is delay value
    }
    xQ1d31[0]= audioQ1d31; //Input

    /* Phase Outputs Filters */
    for (phaseNum = 0; phaseNum < upsampleRatio; phaseNum++) 
    {
        /* Current polyphase filter */
        //NOTE:  Phase coeffients orderd by phase group sequence
        hPhaseQ1d31 = hQ1d31 + phaseNum;   //Pointer to coefficient

        /* Phase FIR sum */
        sumQ1d31 = 0;
        for (tap = 0; tap < numTapsPerPhase; tap++) 
        {
            q15 xQ1d15;
            q15 hPhaseQ1d15;

            //sumQ1d31 += ((*hPhaseQ1d31) * xQ1d31[tap]); //ASR
            xQ1d15 = libq_q15_RoundL_q31(xQ1d31[tap]);
            hPhaseQ1d15 = libq_q15_RoundL_q31(*hPhaseQ1d31);

            sumQ1d31 = libq_q31_Mac_q31_q15_q15(
                    sumQ1d31, xQ1d15, hPhaseQ1d15); 
             
            //Point to next coefficient
            hPhaseQ1d31 += upsampleRatio;   
        }
        
        //sumQ1d31 = audioQ1d31;   //DEBUG:  0-order filter 
        *outQ1d31++ = sumQ1d31;     /* store sum and point to next output */
    }
} //End Upsample_NX_PolyPhase()
