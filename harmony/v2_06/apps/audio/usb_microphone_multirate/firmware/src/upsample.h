/****************************************************************************
*
* Name: interp.h
*
* Synopsis:
*
*   Interpolates a real or complex signal.  For more information, about
*   interpolation, see dspGuru's Multirate FAQ at:
*
*       http://www.dspguru.com/dsp/faqs/multirate
*
* Description: See function descriptons below.
*
* by Grant R. Griffin
* Copyright 2001-2015, Iowegian International Corporation
* (http://www.iowegian.com)
*
*                          The Wide Open License (WOL)
*
* Permission to use, copy, modify, distribute and sell this software and its
* documentation for any purpose is hereby granted without fee, provided that
* the above copyright notice and this license appear in all source copies. 
* THIS SOFTWARE IS PROVIDED "AS IS" WITHOUT EXPRESS OR IMPLIED WARRANTY OF
* ANY KIND. See http://www.dspguru.com/wide-open-license for more information.
*
*
* NOTE:  The original code with the WOL has been modified extensively to work 
*        in Microchip Harmony for specific upsamples and downsamples
*****************************************************************************/

#include "driver/i2s/drv_i2s.h"
#include "math/libq/libq_c.h"
#include "filterCoeffs.h"

#if 0
//1st order linear interpolator coefficients for 3X polyphase interpolation
//filter
//-- x(k)   = 1*x(k) +   0*x(k+3)
//-- x(k+1) = 1/3*x(k) + 2/3*x(k+3)
//-- x(k+2) = 2/3*x(k) + 1/3*x(k+3)
#define  H_INTERP_1ORDER_3X   \
{ \
    1.00000000, \
    0.33333334, \
    0.66666667, \
    0.00000000, \
    0.66666667, \
    0.33333334  \
}

//1st order linear interpolator coefficients for 2X polyphase interpolation
//filter
//-- x(k)   = 1*x(k) +   0*x(k+3)
//-- x(k+1) = 1/3*x(k) + 2/3*x(k+3)
//-- x(k+2) = 2/3*x(k) + 1/3*x(k+3)
#define  H_INTERP_1ORDER_2X   \
{ \
    1.00000000, \
    0.50000000, \
    0.00000000, \
    0.50000000  \
}

//1st order linear interpolation coefficients for 6X polyphase interpolation
//--Polyphase filter coefficients
// {1, 1/6, 2/6, ... 5/6, 0, 5/6, 4/6 ... 1/6}
#define H_INTERP_1ORDER_6X  \
{ \
    1.00000000, \
    0.16666667, \
    0.33333333, \
    0.50000000, \
    0.66666667, \
    0.83333333, \
    0.00000000, \
    0.83333333, \
    0.66666667, \
    0.50000000, \
    0.33333333, \
    0.16666667, \
}

// {1, 1/3, 2/3, 0, 2/3, 1/3 }
#define H_INTERP_1ORDER_3X_Q1D31  \
{ \
    0x8FFFFFFF, \
    0x2AAAAAAA, \
    0x55555555, \
    0x00000000, \
    0x2AAAAAAA, \
    0x55555555, \
}

// {1, 1/6, 2/6, ... 5/6, 0, 5/6, 4/6 ... 1/6}
#define H_INTERP_1ORDER_6X_Q1D31  \
{ \
    0x8FFFFFFF, \
    0x15555555, \
    0x2AAAAAAA, \
    0x40000000, \
    0x55555555, \
    0x6AAAAAAA, \
    0x00000000, \
    0x6AAAAAAA, \
    0x55555555, \
    0x40000000, \
    0x2AAAAAAA, \
    0x15555555 \
}

// Python firls Design with cutoff around 5000Hz and 
// >20dB stop-band after 8Khz
// ['0.028467651', '0.066874938', '0.105958578', '0.139625284', 
//  '0.162382326', '0.170421608', '0.162382326', '0.139625284', 
//  '0.105958578', '0.066874938', '0.028467651']
#define H_INTERP_1ORDER_LS_6X_Q1D31  \
{ \
    0x03a4d3f8, \
    0x088f5ba3, \
    0x0d900cf9, \
    0x11df3dc6, \
    0x14c8f1ae, \
    0x15d06011, \
    0x14c8f1ae, \
    0x11df3dc6, \
    0x0d900cf9, \
    0x088f5ba3, \
    0x03a4d3f8, \
    0x00000000  \
}

// {1, 1, 1, 1, 1, 1}
#define  H_INTERP_0ORDER_6X_Q1D31  \
{ \
    0x8FFFFFFF, \
    0x8FFFFFFF, \
    0x8FFFFFFF, \
    0x8FFFFFFF, \
    0x8FFFFFFF, \
    0x8FFFFFFF \
}
#endif //0

//__inline__ void Upsample_0_OrderFilter(DRV_I2S_DATA32 audioSample,
//                                       DRV_I2S_DATA32 * outBuffer, 
//                                       int upsampleRatio);
__inline__ void Upsample_0_OrderFilter(DRV_I2S_DATA32 audioSample,
                                       q31 * outDataQ1d31,
                                       int upsampleRatio);

//******************************************************************************
// Upsample_NX_Polyphase())
//
// Polyphase FIR implementation of upsampler.
//
//    upsampleRatio:
//        The interpolation factor (must be > 1)
//
//    numTapsPerPhase:
//        The number of taps per polyphase filter, which is the number of taps
//        in the filter divided by factor_L.
//
//    h:
//        Pointer to the array of coefficients for the resampling filter.  (The
//        number of total taps in hPhase is upsampleRatio*numTapsPerPhase
//        Number of filter coef. must is divisible by upsampleRatio.)
//
// Input/Outputs:
//
//    audioSample:
//        The input stereo audio sample in DRV_I2S_DATA32 format. 
//        Range:  -1 to 1 
//
//    x: 
//        pointer to the input delay line (Length is numTapsPerPhase)
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
                   int  upsampleRatio,    //L upsample factor
                   int  numTapsPerPhase,  //#taps/phase = #elements/phase 
                   q31 * const hQ1d31,    //coefficients
                   q31   audioQ1d31,      //Input sample to filter
                   q31 * xQ1d31,  //Delay line array (#elements/phase) 
                   q31 * outDataQ1d31); //Upsample output buffer
