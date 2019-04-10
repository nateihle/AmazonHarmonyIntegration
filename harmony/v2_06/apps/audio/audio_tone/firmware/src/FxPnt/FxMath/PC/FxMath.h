//=============================================================================
//
// File: FrMath.h
//
// Header file for FrMath.c. Fractional Math.
//
// NOTES: 1) Named functions all start with Fr (Fract) or Fl (Float)
//           followed by give <s,d> for the sizes of arguments and then
//           the return values, using:
//             i - integer scale factor (2^m) argument 
//             s - 16 bit short Qmdn
//             w - 32 bit word (MCU32) Qmdn 
//             d - 64 bit double word Qmdn
//
//           and then followed by function name (capitalized)
//
//        2) Conversion functions in FrConvert library.
//
//        3) Q format is designated by Qmdn, where m+n includes the sign bit of the
//           2's Complement integer and n is the number of 2'sC 
//           fractional bits.
//           --> Short form Qn designates a Qmdn number with an known 
//               numerical bit length
//           
//=============================================================================

#ifndef _FRMATH_H_
#define _FRMATH_H_

#include "FrConvert.h"

/* log10(2) scaled to Q5.11 format */
#define LOG102Q5D11      Fl2FxPnt16(0.301029996,11)
#define NINETYQ10D6      FlFxPnt16(90,6)   /* Ninety degrees scaled to Q10d6 */
#define ONEEIGHTYQ10D6   FlFxPnt16(180,6)  /* 180 degrees scaled to Q10d6    */
#define TWOSEVENTYQ10D6  Fl2FxPnt16(270,6)  /* 270 degrees scaled to Q10d6    */
#define THREESIXTYQ10D6  Fl2FxPnt16(360,6)  /* 360 degrees scaled to Q10d6    */
#define NINETYQ10D22     Fl2FxPnt32(90,22)  /* Ninety degrees scaled to Q10d22*/
#define ONEEIGHTYQ10D22  Fl2FxPnt32(180,22) /* 180 degrees scaled to Q10d22   */
#define TWOSEVENTYQ10D22 Fl2FxPnt32(270,22) /* 270 degrees scaled to Q10d22   */
#define THREESIXTYQ10D22 Fl2FxPnt32(360,22) /* 360 degrees scaled to Q10d22   */

/* find the maximum of two numbers */
#define FrMax(a,b) ((a)>(b)?(a):(b))

/* find the minimum of two numbers */
#define FrMin(a,b) ((a)<(b)?(a):(b))

#ifdef __cplusplus
extern "C" {
#endif

//==============================================================================
//
// FrwsSqrt()
//
// Description:
// This routine takes a 32-bit number and returns a 16-bit result.  The
// core algorithm is a 16-bit algorithm.
//
// The algorithm used in this function was created to take the square
// root of numbers 0.5 and 1.0.  For numbers outside this range, they are
// scaled to that range and then square root rescaled at the end.
//
// The algorithm used is: sqrt(x) = 0.2075806 + 1.454895x - 1.34491x^2
// + 1.106812x^3 - 0.536499x^4 + 0.1121216x^5.  The coefficients have
// been scaled by 1/2 to couch them in Fract16 (Q15) format.  The result
// is then scaled up by 2.
//
// Magnitude scaling for the square root takes the following route:
// x is the number to find the square root of and y is the
// scaled value.  Their relationship is: y = (2^n)*x, so
// sqrt(y) = sqrt(2^n)*sqrt(x)
//         = 2^(n/2)*sqrt(x) for n even
//         = sqrt(2)*(2^(n-1)/2)*sqrt(x) for n odd
//
// Representation scaling is also necessary & follows the pattern
// shown:
//   IN                        OUT
//  xQm.n             [sqrt(x)/sqrt(2)][Q(m/2)+1].[16-(m/2)-1], for m even
//  xQm.n              sqrt(x)[Q(m+1)/2].[16 - (m+1)/2], for m odd
//
//  xQ1d15             sqrt(x) in Q1d15, m is odd
//  xQ2d14             [sqrt(x)/sqrt(2)] in Q2d14, m is even
//  xQ3d13             sqrt(x) in Q2d14, m is odd
//  xQ4d12             [sqrt(x)/sqrt(2)] in Q3d13, m even
//
//  The user must fix the value of sqrt(x) for even values of m.
//  For example for a Q2d14, 
//     use frsssMult(f2FxPnt16(1.414213562,14),sqrtQ1d15)
//  to get the appropriately scaled representation described.
//
// Arguments:
//   Fract32 xQ1d31
//     -- The input value for which the square root is desired:
//          Range: [0, 1).
//
// Return Value:
//   Fract16 -- The square root of xQ1d31 in Q1d15 (possibly scaled)  
//
//==============================================================================
Fract16 FrwsSqrt(Fract32);


//==============================================================================
//
// FrssLog10()
//
// Description:
//   The following code calculates the log base 10.  The log base 10 of
//   any number x between 1 and 2 can be approximated using the following
//   polynomial: 2*log10(x) = 0.8678284(x-1) - 0.42555677(x-1)^2 +
//   0.2481384(x-1)^3 - 0.1155701(x-1)^4 + 0.272522(x-1)^5,
//
//   therefore:
//   log10(x) = 0.4339142(x-1) - 0.21278385(x-1)^2 + 0.1240692(x-1)^3 -
//   0.05778505(x-1)^4 + 0.0136261(x-1)^5.
//
//   To compute the log for numbers less than 1 or greater than 2, the
//   input can be scaled to fit the range 1 to 2 and then rescaled to give
//   the proper result.
//
// Arguments:
//   FxPnt16 xQ1d15 -- The input value to find the log base 10 of.
//   int16_t m -- The scale of x, i.e., if x is scaled Q10d6, m = 10
//          suggest 2.14(m=2) format for x<1 and 11.5(m=11) for 2<=x<1023
//
// Return Value:
//   FxPnt16 log10(xQmd<16-m>) in Q5d11 
//
//==============================================================================
Fract16 FrsisLog10(FxPnt16, int16_t);


//==============================================================================
//
// FrwsLog10()
//
// Description:
//   This routine takes a 32-bit number and returns a 16-bit result.  The
//   core algorithm is a 16-bit algorithm.
//
//   The following code calculates the log base 10.  The log base 10 of
//   any number x between 1 and 2 can be approximated using the following
//   polynomial: 2*log10(x) = 0.8678284(x-1) - 0.42555677(x-1)^2 +
//   0.2481384(x-1)^3 - 0.1155701(x-1)^4 + 0.272522(x-1)^5,
//
//   therefore:
//   log10(x) = 0.4339142(x-1) - 0.21278385(x-1)^2 + 0.1240692(x-1)^3 -
//   0.05778505(x-1)^4 + 0.0136261(x-1)^5.
//
//   To compute the log for numbers less than 1 or greater than 2, the
//   input can be scaled to fit the range 1 to 2 and then rescaled to give
//   the proper result.
//
// Arguments:
//   FxPnt32 xQ1d31 -- The input value to find the log base 10 of.
//   int16_t m      -- The scale of x, i.e., if x is scaled Q10d6, m = 10
//          suggest 1.31(m=1) or 2:30 (m = 2) format for x<1 and
//          11.21(m=11) for 2<=x<1023.
//
// Return Value:
//   FxPnt16 Log10(Qmd<32-m>) in Q5d11
//
//==============================================================================
Fract16 FrwsLog10(FxPnt32, int16_t);


//==============================================================================
//
// FrssCos()
//
// Description:
//   This function returns the cosine of an angle.
//   It uses the trig identity cos(theta) = sin(theta+90).  So after
//   adding 90 degrees to the value of the angle, it calls the sin
//   function.  Any angle between -360 and 360 can be entered.  This
//   value is expected to be scaled to a 10.6 format; the value of the
//   cos is returned as a 1.15.
//
// Arguments:
//   FxPnt16 angleQ10d6 -- The angle in degrees for which the cosine is to be 
//        computed.
//
// Return Value:
//   Fract16.  cos(Q10d6 degrees) in Q1d15
//
//==============================================================================
Fract16 FrssCos(FxPnt16);


//==============================================================================
//
// FrssSin()
//
// Description:
//   This function approximates the sine of an angle using the
//   following algorithm: sin(x) = 3.140625x + 0.02026367x^2 -
//   5.325196x^3 + 0.5446778x^4 + 1.800293x^5.  The approximation
//   is accurate for any value of x from 0 degrees to 90 degrees.
//   Because sin(-x) = - sin(x) and sin(x) = sin(180 - x), the sine
//   of any angle can be inferred from an angle in the first quadrant.
//   Therefore, any angle > 90 is converted to an angle between 0 & 90.
//   The coefficients of the algorithm have been scaled by 1/8 to
//   fit a Q1d15 format.  So the result is scaled up by 8 to obtain
//   the proper magnitudes.  The algorithm expects the angle to be in
//   degrees and represented in 10.6 format.  The computed sine value
//   is returned in 1.15 format.
//
// Arguments:
//   FxPnt16 angleQ10d6 -- The angle in degrees for which the sine is computed
//
// Return Value:
//   Fract16.  sin(Q10d6 degrees) in Q1d15
//
//==============================================================================
Fract16 FrssSin(FxPnt16);


//==============================================================================
//
// FrwsCos()
//
// Description:
//   This function returns the cosine of an angle.
//   It uses the trig identity cos(theta) = sin(theta+90).  So after
//   adding 90 degrees to the value of the angle, it calls the sin
//   function.  Any angle between -360 and 360 can be entered.  This
//   value is expected to be scaled to a 10.22 format; the value of the
//   cos is returned as a 1.15.
//
// Arguments:
//   FxPnt16 angleQ10d22
//     -- The angle in degrees for which the cosine is to be computed.
//
// Return Value:
//   Fract16.  cos(10d22) in Q1d15
//
//==============================================================================
Fract16 FrwsCos(FxPnt32);


//==============================================================================
//
// FrwsSin()
//
// Description:
//   This function takes a 32 bit angle and returns a 16 bit sine.
//   It is derived from frssSin which uses a 16 bit core algorithm.
//
//   This function approximates the sine of an angle using the
//   following algorithm: sin(x) = 3.140625x + 0.02026367x^2 -
//   5.325196x^3 + 0.5446778x^4 + 1.800293x^5.  The approximation
//   is accurate for any value of x from 0 degrees to 90 degrees.
//   Because sin(-x) = - sin(x) and sin(x) = sin(180 - x), the sine
//   of any angle can be inferred from an angle in the first quadrant.
//   Therefore, any angle > 90 is converted to an angle between 0 & 90.
//   The coefficients of the algorithm have been scaled by 1/8 to
//   fit a Q1d15 format.  So the result is scaled up by 8 to obtain
//   the proper magnitudes.  The algorithm expects the angle to be in
//   degrees and represented in 10.22 format.  The computed sine value
//   is returned in 1.15 format.
//
// Arguments:
//   FxPnt32 angleQ10d22 -- The angle in degrees for which the sine is computed
//
// Return Value:
//
//   Fract16.  sin(Q10d22) in Q1d15
//
//==============================================================================
Fract16 FrwsSin(FxPnt32);


//==============================================================================
//
// FrssSqrt()
//
// Description:
// The algorithm used in this function was created to take the square
// root of numbers between 0.5 and 1.0.  For numbers outside this range,
// they may be scaled to that range and then the square root rescaled at the
// end.  The program will take any size number, in proper representation,
// and find the square root.  The final scaling, if any is needed, is left
// to the user.  This scaling is explained below.
//
// The algorithm used is: sqrt(x) = 0.2075806 + 1.454895x - 1.34491x^2
// + 1.106812x^3 - 0.536499x^4 + 0.1121216x^5.  It was designed to find the
// square root of numbers between 0.5 and 1.  However, the square root of
// any number can be found with proper scaling.  In this program, the
// coefficients are scaled by 1/2 to represent them in Fract16 format.
// The final result, i.e. the square root, is then scaled up by 2.
//
// Magnitude scaling for the square root takes the following route:
// let x be the number to find the square root of and y be the
// scaled value.  Their relationship is: y = (2^n)*x, so
// sqrt(y) = sqrt(2^n)*sqrt(x)
//         = 2^(n/2)*sqrt(x)              for n even
//         = sqrt(2)*(2^(n-1)/2)*sqrt(x)  for n odd
//
// However, the sqrt(x) is the value sought and may be found by proper
// algebraic manipulation.
//
// Representation scaling is also necessary & follows the pattern
// shown next:
//  IN Representation    OUT Representation (sqrt represented in this format)
//  xQm.n                [sqrt(x)/sqrt(2)][Q(m/2)+1].[16-(m/2)-1], for m even
//  xQm.n                sqrt(x)[Q(m+1)/2].[16 - (m+1)/2],         for m odd
//          Examples:
//  xQ1d15             sqrt(x)Q1d15,           m is odd
//  xQ2d14             [sqrt(x)/sqrt(2)]Q2d14, m is even
//  xQ3d13             sqrt(x)Q2d14,           m is odd
//  xQ4d12             [sqrt(x)/sqrt(2)]Q3d13, m even
//
//  The user must fix the value of sqrt(x) for even values of m.
//  For even values, use frsssMult(f2FxPnt16(1.414213562,14),sqrtQ1d15)
//  to get the appropriately scaled representation described.
//
// Arguments:
//   Fract16 xQ1d15
//     -- The input value for which the square root is desired:
//          Range: (0, 1).
//
// Return Value:
//   Fract16 -- sqrtQ1d15: The square root of xQ1d15 (when input not scaled) 
//
//==============================================================================
Fract16 FrssSqrt(Fract16);


//==============================================================================
//
// FrwssRand()
//
// Description:
//   Generates a frame of 16-bit random numbers using the linear congruential
//   method.  The linear congruential method has the form:
//      x(n) = M*x(n-1) + K
//   where
//      x(n) = Random number at sample n,
//      M = Constant multiplier,
//      K = Constant increment.
//   The intent is that this routine is called using the last random value
//   generated in the previous frame as the seed for the current frame.  In
//   this way, successive calls of the routine will produce 65,536 random
//   numbers.
//
//   Note that it is required that modulo-2^32 arithmetic be performed:
//   operations should overflow and wrap.  Future implementers should not "fix"
//   the math by allowing guard bits, larger intermediate values, etc.
//
//   Reference: "Random Number Generation on a TMS320C5x"  Texas Instruments
//   Application Brief: SPRA239, July, 1994.
//
// Arguments:
//   int32_t *lastRand
//     [in/out] This must be a persistent memory location.  Initially it is
//     the last random value generated during the previous call.  After this
//     routine completes, it is the last random generated in the current
//     frame.
//
//   int16_t *rand -- Pointer to the random output data.
//
//   int16_t length -- size of output data frame
//
// Return Value -- None.
//
//==============================================================================
void FrwssRand(int32_t * prevValue, int16_t * newValue, int16_t frameLength);


//==============================================================================
//
// FrswwMult()
//
// Description:
//   Implement 16 bit by 32 bit multiply. A textual pictoral representation of
//   the extended precision math is shown below. The 's' and 'u' notation shows
//   the processing of signed and unsigned numbers. Since the numbers are signed
//   fractional numbers, the sign bit is replicated in the upper two bits of
//   the product. Overflow detection is done on the two sign bits before they
//   are combined into a single bit.
//
//        -B1- -B0-   s u     2nd argument is 32 bits
//             -A0-     s     1st argument is 16 bits
//   --------------
//        A0B0 A0B0   s=s*u   1st 32-bit product is A0*B0
//   A0B1 A0B1        s=s*s   2nd 32-bit product is A0*B1
//   --------------
//   -S2- -S1- -S0-   s=s+s   48-bit result is the sum of products
//   -P1- -P0-                32-bit return is the most significant bits of the sum
//
// Arguments:
//   Fract16 AQ1d15
//     -- 16 bit argument A.
//   Fract32 BQ1d31
//     -- 2 bit argument B.
//
// Return Value:
//   Fract32 - A x B in Q1d31
//
//==============================================================================
Fract32 FrswwMult(Fract16, Fract32);


/******************************************************************************
 *
 * FrwwwMult()
 *
 * Description:
 *   Implements a 32 bit by 32 bit multiply. This is a 32 bit by 32 bit
 *   multiply with 32 bit result. A textual pictoral representation of
 *   the extended precision math is shown below. The 's' and 'u' notation shows
 *   the processing of signed and unsigned numbers. Since the numbers are signed
 *   fractional numbers, the sign bit is replicated in the upper two bits of
 *   the product. Overflow detection is done on the two sign bits before they
 *   are combined into a single bit.
 *
 *   In this implementation, the first product is ignored in the interests of
 *   speed and at the expense of a slight bit of accuracy. If the result of
 *   accumulating the first product would have increased bit 32 of the 64-bit
 *   result, the final product will be in error by 1 lsb.
 *
 *             -B1- -B0-   s u     2nd argument is 32 bits
 *             -A1- -A0-   s u     1st argument is 32 bits
 *   -------------------
 *             A0B0 A0B0   u=u*u   1st 32-bit product is A0*B0 (ignored)
 *        A0B1 A0B1        s=u*s   2nd 32-bit product is A0*B1
 *        A1B0 A1B0        s=s*u   3rd 32-bit product is A1*B0
 *   A1B1 A1B1             s=s*s   4th 32-bit product is A1*B1
 *   -------------------
 *   -S2- -S1- -S0-        s=s+s   48-bit result is the sum of products
 *   -P1- -P0-                     32-bit return is the MSbits of the sum
 *
 * Arguments:
 *   Fract32 AQ1d31
 *     -- First multiplicand A.
 *   Fract32 BQ1d31
 *     -- Second multiplicand B.
 *
 * Return Value:
 *   Fract32 - A x B in Q1d31.
 *
 ******************************************************************************/
Fract32 FrwwwMult(Fract32, Fract32);


//==============================================================================
//
// FrssssExpAvg()
//
// Description:
//   Exponential averaging implements a smoothing function based on the 
//   recursive averaging form:
//       avg[i+1] = avg[i] * lamda + new * (1-lamda)
//
//   In this implementation, is has been optimized as follows.
//       avg[i+1] = (avg[i] - new) * lamda + new
//
//   The optimization precludes accurate processing of new numbers that differ
//   from the current average by more than unity. If the difference is greater
//   than unity or less than negative unity, the difference is saturated.
//
//   The effect is akin to a smaller lambda, e.g., the new value will have a
//   greater weight than expected. If the smoothing is of data that is entirely
//   positive or entirely negative, then the saturation will not be an issue.
//
// Arguments:
//   FxPnt16 prevAvg16 (Q15) -- Previous exponential average
//   FxPnt16 newMeas16 (Q15) -- New value to be averaged in
//   Fract16 lamdaQ1d15 -- exponential averaging constant
//
// Return Value:
//   FxPnt16 - newAvg16 (Q15)
//
//==============================================================================
FxPnt16 frssssExpAvg(FxPnt16, FxPnt16, Fract16);


//==============================================================================
//******************************************************************************
// frsisLog()
//
// Description:
//   The following code calculates the natural log.  The natural log of a number
//   can be approximated using the following Taylor's series.
//
// log(x) = (x-1) - 0.5(x-1)^2 + 0.3333333(x-1)^3 - 0.25(x-1)^4 + ........
//
// Arguments:
//   Fract16 xQ1d15 -- The input value to find the natural log.
//   int16_t polyOrder -- Order of taylor series apprx.
//
// Return Value:
//   Fract16 -- log10 value in Q1d15
//
// !!!!! WARNING : This function expects the input in the range [0.5,1) !!!!!!!
//                 Maximum allowable polynomial value is 10
//==============================================================================
Fract16 FrsisLog(Fract16 xQ1d15, int16_t polyOrder);


//==============================================================================
// FrsisExp()
//
// Description:
//   The following code calculates the exponential.  The exponential of a number
//   can be approximated using the following Taylor's series.
//
// exp(x) = 1 + x + x^2(2!) + x^3/(3!) + x^4/(4!) + ........
//
// Arguments:
//   Fract16 xQ1d15 -- The input value to find the exponential.
//
//   int16_t polyOrder -- order of taylor series
//
// Return Value:
//   Fract16 -- exponential value in Q1d15
//
// !!!!! WARNING : This function expects the input in the range [-1,0) !!!!!!!
//                 Maximum allowable polynomial value is 10
//==============================================================================
Fract16 FrsisExp(Fract16 xQ1d15, int16_t polyOrder);


//==============================================================================
//
// FrssiCmplxMag()
//
// Description:
//   This function calculates the magnitude of the complex vector.
//   It accepts the complex data of size N and returns the real
//   magnitude vector of size N.
//
// Arguments:
//   Fract16 *cmplxData --  Pointer to complex data.
//   Fract16 *realMag   --  Pointer to real magnitude (result)
//   int16_t   len      -- Array size.
//
// Return Value:
//   none
//
// !!!!! WARNING : The real and imaginary part of the complex data should
//                 be in the same scale.
//==============================================================================
void frssiCmplxMag(Fract16 *cmplxData,
                   Fract16 *realMag,
                   int16_t len);


//==============================================================================
//
// frssiDotProd()
//
// Description:
//   Performs element-by-element multiplication (dot product) of
//   two vectors. Result is stored in the third vector.
//
// Arguments:
//   Fract16 *arr1Q1d15   -- Pointer to the first input array
//   Fract16 *arr2Q1d15   -- Pointer to the second input array
//   Fract16 *resultQ1d15 -- Pointer to the  output array (result)
//   int16_t  len         -- Length of arrays
//
// Return Value:
//   none
//
//==============================================================================
void frssiDotProd(Fract16 *arr1Q1d15,
                  Fract16 *arr2Q1d15,
                  Fract16 *resultQ1d15,
                  int16_t    len);


//==============================================================================
//
// frssiScaleMult()
//
// Description:
//   Multiplies a vector by a real value. Result is stored in
//   the second vector.
//
// Arguments:
//   Fract16 *vector -- Pointer to the first input array
//   Fract16  scale  -- scaler to multiply by
//   Fract16 *result -- Pointer to the  output array (result)
//   int16_t   len   -- Length of arrays
//
// Return Value:
//   none
//
//==============================================================================
void frssiScaleMult(Fract16 *vector,
                    Fract16  scale,
                    Fract16 *result,
                    int16_t  len);


#ifdef __cplusplus
}
#endif

#endif /* _FRMATH_H_ */
