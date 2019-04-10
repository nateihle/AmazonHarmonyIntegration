//==============================================================================
//
// File: FxConvert.h
//
// Description: Fixed-point and floating-point conversion macros.
//
// WARNINGS:
//   (1) None of these macros saturate (use values within the Qformat).
//   (2) These macros are intended only to be used with constants resolved by
//       the preprocessor 
//
//==============================================================================

#ifndef _FXCONVERT_H_
#define _FXCONVERT_H_

#include "FxTypes.h"

//// DOM-IGNORE-BEGIN
#ifdef __cplusplus
extern "C" {
#endif
// DOM-IGNORE-END

/******************************************************************************
 *
 * Fl2Fract16()
 *
 * Converts floating point constant value to fractional 16-bit value with
 * rounding.
 *
 * value -- Floating point value constant expression to convert.
 *
 * Return Value
 *   Equivalent fractional value as Fract16.
 *
 ******************************************************************************/
#define Fl2Fract16(value) \
( \
  (Fract16)Fl2FxPnt((value), 15) \
)

/******************************************************************************
 *
 * Fl2Fract32()
 *
 * Converts floating point constant value to fractional 32-bit value with
 * rounding.
 *
 * value -- Floating point value constant expression to convert.
 *
 * Return Value
 *   Equivalent fractional value as AtiFract32.
 *
 ******************************************************************************/
#define Fl2Fract32(value) \
( \
  (Fract32)Fl2FxPnt((value), 31) \
)

/******************************************************************************
 *
 * Fl2FxPnt16()
 *
 * Converts floating point constant value to fixed point 16-bit value with
 * rounding.
 *
 * value -- Floating point value constant expression to convert.
 * bits  -- Number of fractional bits.
 *
 * Return Value
 *   Equivalent fixed point value as AtiFxPnt16.
 *
 ******************************************************************************/
#define Fl2FxPnt16(value, bits) \
( \
  (FxPnt16)Fl2FxPnt((value), (bits)) \
)

/******************************************************************************
 *
 * f2FxPnt32()
 *
 * Converts floating point constant value to fixed point 32-bit value with
 * rounding.
 *
 * value
 *   [in] Floating point value constant expression to convert.
 * bits
 *   [in] Number of fractional bits.
 *
 * Return Value
 *   Equivalent fixed point value as AtiFxPnt32.
 *
 ******************************************************************************/
#define Fl2FxPnt32(value, bits) \
( \
  (FxPnt32)Fl2FxPnt((value), (bits)) \
)

/******************************************************************************
 *
 * Fl2FxPnt()
 *
 * Converts floating point constant value to fixed/fractional value of
 * specified precision with rounding.
 *
 * value -- Floating point value constant expression to convert.
 * bits  -- Number of fractional bits.
 *
 * Return Value
 *   Equivalent fixed point value as double.
 *
 ******************************************************************************/
#define Fl2FxPnt(value, bits) \
( \
  ((double)(value)*(double)(1UL<<(bits))) + \
    (((double)((double)(value) >= 0.0)) - ((double)0.5)) \
)

/******************************************************************************
 *
 * Fl2Int16()
 *
 * Converts floating point constant expression to 16-bit integer value
 * Conversion is safe for compilers which assign fractional data type to
 * constants with decimal point
 *
 * value --   Floating point value constant expression to convert.
 *
 * Return Value
 *   Calculated integer value.
 *
 ******************************************************************************/
#define Fl2Int16(value) ((int16_t)((double)(value)))

/******************************************************************************
 *
 * Fl2Int32()
 *
 * Converts floating point constant expression to 32-bit integer value
 * Conversion is safe for compilers which assign fractional data type to
 * constants with decimal point
 *
 * value -- Floating point value constant expression to convert.
 *
 * Return Value
 *   Calculated integer value.
 *
 ******************************************************************************/
#define Fl2Int32(value) ((int32_t)((double)(value)))

/******************************************************************************
 *
 * Fl2QFloat32()
 *
 * Converts a decimal floating-point constant expression to pseudo float
 *
 * mantissa
 *   [in] Floating-point value constant expression to convert to the mantissa
 *        portion of the ATI floating-point number.
 * exponent
 *   [in] Integer value constant expression to convert to the exponent portion
 *        of the ATI floating-point number.
 *
 * Return Value
 *   Calculated integer value.
 *
 ******************************************************************************/
#define Fl2Qfloat32(mantissa, exponent) \
  { \
    Fl2Fract16(mantissa), \
    Fl2Int16(exponent) \
  }

//// DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
// DOM-IGNORE-END

#endif  /* _FXCONVERT_H_ */
