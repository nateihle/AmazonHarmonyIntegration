//*****************************************************************************
//
// File: FrwiwShr.c
//
// Description: Performs an 'Arithmetic' Right Shift of the 32-bit input 
//   argument 'a' by the input argument 'b' bit positions.
//
//*****************************************************************************

#include "FxConvert.h"
#include "FxFract.h"


/******************************************************************************
 *
 * FrwiwShr()
 *
 * Description:
 *   Performs an 'Arithmetic' RIGHT Shift on a 32-bit input by 'b' bit
 *   positions.  For positive shift directions (b>0), 'b' Lsb-bits are shifted
 *   out to the right and 'b' sign-extended Msb-bits fill in from the left.  
 *   For negative shift directions (b<0), 'b' Lsb's are shifted to the
 *   LEFT with 0's filling in the empty lsb position. The left shifting
 *   causes 'b' Msb-bits to fall off to the left, saturation is applied to
 *   any shift left value that overflows. This function calls the
 *   frdidShl() function to perform any 32-bit left shifts.  This function 
 *   does not provide any status-type information to indicate when overflows 
 *   occur.
 *   This function relates to the ETSI L_shr function.
 *
 * Arguments:
 *   Fract32 a
 *     [in]  32-bit signed integer value to be shifted
 *   Int16 b
 *     [in]  16-bit signed integer shift index 
 *          positive value: # of bits to right shift (sign extend)
 *          negative value: # of bits to left shift (zeros inserted at LSB's)
 * 
 * Return Value:
 *   Fract32 result
 *     [return]  Arithmetically shifted 32-bit signed integer output
 *
 ******************************************************************************/
Fract32 FrwiwShr(Fract32 a, int16_t b)
{

  Fract32 result;                   /* Value returned */

  /* "Right shift" or "Left shift"*/
  if (b >= 0)
  {   
    /* Right shift, first check whether 'Msb' will shift past 'Lsb'.
     * If yes, every bit in result is a copy of original sign bit. */
    if (b >= (NUMBITSFRACT32-1)) 
    {
      result = (a < 0) ? (Fract32)BITMASKFRACT32: (Fract32)0;
    }
    /* Else, Right shift input by 'b' bits with sign extention. If 'a'
     * is positive, the sign-extenting is automatic. If 'a' is negative,
     * the sign bit must be held while shifting.  Hold the sign bit 
     * by complementing the value 'before' and 'after' the shift, ex:
     * (a=0x8000000,b=16): before shift: ~(0x80000000)=>0x7fffffff and
     * after shift: ~(0x007fffff)=>0xff800000 */ 
    else
    {
      result = (a < 0) ? (Fract32)(~ ((~a) >> b)): (Fract32)(a >> b);
    }
  }
  /* Else, Left shift the 32-bit input by 'b' bits with zero-fill. Any 
   * overflow caused by the shift is saturated. */
  else
  {
    if (b != MININT16)
    {
      result = FrwiwShl(a,(int16_t)(-b));
    }
    else
    {
      result = FrwiwShl(a,(int16_t)(MAXINT16));
    }
  }
  return (result);
}
