
#include "libq_c.h"


//*****************************************************************************
// ExpAveragePower()
//
// Summary:
//   Short Term Power (exponential window) estimate computed every 1
//   sample(s)  The exponential window is used for efficiency and stability.
//   This power estimate is used to scale the error updates in the filter
//   update equation.
//
//   forgetting factor Lambda = (1 - 2^-lambExp)
//   Let N=lambExp, and k=sample index.
//
//   P(k) = (1-2^-N)P(k-1) + (2^-N)*X(k)*x(k)
//        = P(k-1) - 2^-N*P(k-1) + 2^-N*x(k)*x(k) 
//  
//   NOTE:  This is an approximation of the Xt*X value. The exponential
//          filter will weight the more current values of x more than the 
//          older values.
//
// Arguments:
//   q31 powerQ1d31 - [in/out] p(k-1) in Q1.31 format
//   q15 refInQ1d15 - [in] x(k)) in Q1.16format
//   int16_t powerExp - N
//
// Return Value:
//   p(k) in Q1d31 format
//
// Return Value: //   None
//*****************************************************************************
q31 ExpAvgPower(q31 powerQ1d31, q15 refInQ1d15, int16_t lambExp)
{
    q31 frTmp1Q1d31, frTmp2Q1d31;
    q31 fr32Tmp1;
    q31 powkQ1d31;
  
    // P(k-1)*2^(-N) 
    fr32Tmp1    = libq_q31_ShiftRight_q31_i16(powerQ1d31, lambExp);
    
    // P(k-1)-P(k-1)*2^-(N)
    frTmp1Q1d31 = libq_q31_Sub_q31_q31(powerQ1d31,fr32Tmp1);

    // x(k)^2
    frTmp2Q1d31 = libq_q31_Mult2_q15_q15(refInQ1d15,refInQ1d15);

    //(2^-N)*x(k)^2
    frTmp2Q1d31 = libq_q31_ShiftRight_q31_i16(frTmp2Q1d31,lambExp);

    // P(k-1)-P(k-1)(2^-N)  +   x(k)^2*(2^-N)
    powkQ1d31 = libq_q31_Add_q31_q31(frTmp1Q1d31, frTmp2Q1d31);

    return powkQ1d31;
} //End ExpAvgPower()
