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

#include "FxConvert.h"

//// DOM-IGNORE-BEGIN
#ifdef __cplusplus
extern "C" {
#endif
// DOM-IGNORE-END

/* log10(2) scaled to Q5.11 format */
#define LOG102Q5D11      Fl2FxPnt16(0.301029996,11)
#define NINETYQ10D6      Fl2FxPnt16(90,6)   /* Ninety degrees scaled to Q10d6 */
#define ONEEIGHTYQ10D6   Fl2FxPnt16(180,6)  /* 180 degrees scaled to Q10d6    */
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
Fract32 FrwwSin_O3(FxPnt32 angQ20d12); //3rd order Polynomial apprx. Q20d12 input/return
Fract16 FrssSin_AngT5(FxPnt16 angleQ10d6);


//// DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
// DOM-IGNORE-END

#endif /* _FRMATH_H_ */
