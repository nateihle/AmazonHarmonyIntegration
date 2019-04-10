//==============================================================================
//
// File: FxConvertVar.h
//
// Description: Fixed and Q Floating point conversion macros for variables.
//
// NOTE:  This file contains the conversion macros intended to operate on
//        variables, whereas FxConvert.h operates on constants.
//
//==============================================================================


#ifndef _FXCONVERTVAR_H_
#define _FXCONVERTVAR_H_

#include "float.h"
#include "Fxtypes.h"
#include "FxMath.h"

//// DOM-IGNORE-BEGIN
#ifdef __cplusplus
extern "C" {
#endif
// DOM-IGNORE-END

FlsssMult(FxFloat32 a, FxFloat32 b)
{
    FxFloat32 product;
    product = a * b;
}

//==============================================================================
//
// FxPntToFloat32()
//
// Converts a fractional value to a 32 bit floating point value.
//
// value -- Fixed point value to convert (expected to be a variable).
//
// bits -- Number of fractional bits (expected to be constant).
//
// Return Value
//   Equivalent floating point value as FxFloat32.
//
//==============================================================================
#define FxPntToFloat32(value, bits) \
( \
  FlsssMult((FxFloat32)value,(FxFloat32)((double)1.0/((double)(1UL<<bits)))) \
)

//==============================================================================
//
// Fract16ToFloat32()
//
// Converts 16 bit (Q1.15) fractional value to 32 bit floating point value.
//
// value -- Fixed point value to convert (expected to be a variable).
//
// Return Value
//   Equivalent floating point value as FxFloat32.
//
//==============================================================================
#define Fract16ToFloat32(value) \
( \
  FxPntToFloat32(value,15) \
)

//==============================================================================
//
// Float32ToFract()
//
// Converts a 32 bit floating point value to a fractional value.
//
// value -- Floating point value to convert (expected to be a variable).
//
// bits -- Number of fractional bits (expected to be constant).
//
// Return Value
//   Equivalent fixed point value.
//
//==============================================================================
#define Float32ToFract(value, bits) \
( \
  (FlsssMult(value,(double)(1UL<<(bits)))) + \
    (value >= 0.0 ? 0.5 : -0.5) \
)

//==============================================================================
//
// Float32ToFract16()
//
// Converts a 32 bit floating point value to a 16 bit fractional value.
//
// value -- Floating point value to convert (expected to be a variable).
//
// Return Value
//   Equivalent fixed point value.
//
//==============================================================================
#define Float32ToFract16(value) \
( \
  (FxFract16)FxMin(FxMax \
  (MININT16, (Float32ToFract(value,15))), MAXINT16) \
)

//==============================================================================
//
// Exponent16ToFloat32()
//
// Converts a power of 2 16-bit integer to 32-bit floating point value.
//
// value -- 16-bit integer number for power (expected to be a variable).
//
// Return Value
//   32-bit floating point value of 2^n.
//
//==============================================================================
#define Exponent16ToFloat32(value) \
( \
  (((value) < 0) ? ((float)1.0/(float)(1UL<<(-(value)))): \
   ((float)(1UL<<(value)))) \
)

//==============================================================================
//
// Float32ToFractSat()
//
// Description:
// Converts a 32 bit floating point value to a 32 bit fractional value with 
// overflow check. If it overflows, then the fractional value will be saturated.
//
// Arguments:
//   FxFloat32 val
//     --  32-bit floating point value
//   FxInt16 bits
//     --  16-bit signed integer shift index
//   FxFxPnt32 result
//     -- 32-bit fixed point value
//
// Return Value:
//   None.
//
//==============================================================================
#define Float32ToFractSat(val, bits, result)                                  \
{                                                                             \
  FxFract32 temp;    /* Temp var for Shifted Sign compare results */         \
  /* Shift Input value left by 'b' which is the default case. */              \
  (result) = (FxFract32)((val) * (1UL << (bits)));                           \
  /* Also overflow will occur 'if all the bits are not all the same' between  \
   * the original sign bit (bit32) and the bit that will become the sign      \
   * bit after 'b' left shifts. */                                            \
  if ((val) > 0)                                                              \
  {                                                                           \
    temp = (((FxFract32)MSBBITFRACT32 >> (bits)) & (FxFract32)(val));    \
    if (temp != 0)                                                            \
    {                                                                         \
      (result) = (FxFract32)MAXFRACT32;                                   \
    }                                                                         \
  }                                                                           \
  else                                                                        \
  {                                                                           \
    temp = (((FxFract32)MSBBITFRACT32 >> (bits)) & (FxFract32)-(val));   \
    if (temp != 0)                                                            \
    {                                                                         \
      (result) = (FxFract32)MINFRACT32;                                   \
    }                                                                         \
  }                                                                           \
}

#endif  /* _FXCONVERTVAR_H_ */

//// DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
// DOM-IGNORE-END

//==============================================================================
/* End of file */
