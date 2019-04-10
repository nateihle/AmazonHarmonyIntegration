//*****************************************************************************
//
// File: FrsisShl.c
//
// Description: Performs an 'Arithmetic' Left Shift of the 16-bit input
//   argument 'a' by the input argument 'b' bit positions.
//
//*****************************************************************************

#include "FxConvert.h"
#include "FxFract.h"


/******************************************************************************
 *
 * FrsisShl()
 *
 * Description:
 *   Performs an 'Arithmetic' Shift of the 16-bit input argument 'a' left
 *   by the input argument 'b' bit positions.  If 'b' is a positive number,
 *   a 16-bit left shift is performed with 'zeros' inserted to the right
 *   of the shifted bits.  If 'b' is a negative number, a right shift by abs(b)
 *   positions with 'sign extention' is done by calling the frsisShr() function.
 *   Saturation is applied if shifting causes an overflow or an underflow.
 *   This function relates to the ETSI shl function.
 *
 * Arguments:
 *   Fract16 a
 *     [in]  16-bit signed integer value to be shifted
 *   Int16 b
 *     [in]  16-bit signed integer shift index
 *          positive value: # of bits to left shift (zeros inserted at LSB's)
 *                          {To not always saturate:
 *                              if 'a=0', then max b=15, else max b=14}
 *          negative value: # of bits to right shift (sign extend)
 *
 * Return Value:
 *   Fract16 result
 *     [return]  Arithmetically shifted 16-bit signed integer output
 *
 ******************************************************************************/
Fract16 FrsisShl(Fract16 a, int16_t b)
{
  Fract16 result;                   /* Value returned */
  Fract32 temp;    /* Temp var for shifted value into a 32-bit word */

  /* Check the input argument 'b', the shift index, for the direction to shift.
   * If 'b' is a positive value then, perform the shift to the "LEFT". */
  if (b >= 0)
  {
    /* Create a temporary 32-bit value which contains 'a' shifted left from
     * the lower 16-bits into the upper 16-bits by 'b' bit positions.
     * This is done by a 32-bit multiply on sign extended 'a' times the
     * 'desired shifted LSB bit position'. */
    temp  = (Fract32)a * (Fract32)(1 << b);
    /* Check for overflow/underflow conditions.  If 'a' was not zero and the
     * LSB 'bit1' is shifted to or past the output sign bit, (i.e. an overflow
     * will occur if bit1 is shifted to or past bit16).  Also overflows can
     * occur, when the sign of the original input 'a' and the sign of the output
     * shifted value are different.  If an Overflow/Underflow occurred, then
     * saturate the output value to the Max 16-bit positive value if input 'a'
     * was positive, and set the output value to the Min 16-bit negative value
     * if 'a' was negative. */
    if ( ((b >= (NUMBITSFRACT16 - 1)) && (a != 0)) || \
        (temp != (Fract32)((Fract16)temp)))
    {
      result = (a > 0) ? (Fract16)MAXFRACT16: (Fract16)MINFRACT16;
    }
    /* No Overflow/Underflow will occur so just take the bottom 16-bits
     * from the temporary 32-bit value of 'a' shifted left by 'b' bits. */
    else
    {
      result = (Fract16)temp;
    }
  }
  /* Else, the shift index 'b' was negative so perform a RIGHT shift
   * on 'a' by 'b' bits with sign extention and saturation applied. */
  else
  {
    if (b != MININT16)
    {
      result = FrsisShr(a,(int16_t)(-b));
    }
    else
    {
      result = FrsisShr(a,(int16_t)(MAXINT16));
    }
  }
  return (result);
}
