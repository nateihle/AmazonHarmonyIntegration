//*****************************************************************************
//
// File: FrsswMult.c
//
// Description: Performs fractional multiplication of two 16-bit fractional 
//   values and returns a 32-bit fractional scaled result.
//
//*****************************************************************************

#include "FxConvert.h"
#include "FxFract.h"


//*****************************************************************************
//
// FrsswMult()
//
// Description:
//   Performs fractional multiplication of two 16-bit fractional values
//   and returns the 32-bit fractional scaled result. 
//
// Arguments:
//   Fract16 a
//     [in]  16-bit signed integer operand 1
//   Fract16 b
//     [in]  16-bit signed integer operand 2
//
// Return Value:
//   Fract32 result
//     [return] 32-bit signed integer output value 
//
//******************************************************************************
Fract32 FrsswMult(Fract16 a, Fract16 b)
{
  Fract32 result;                   /* Value returned */

  /* Perform a 32-bit multiply on the fractional Qd15 input values */
  result = (Fract32)a * (Fract32)b;
  /* Check if the multiply resulted in a overflow or underflow condition. 
   * If it did the result will have the 'two' fractional sign bits set 
   * differently, i.e. bit31 & bit30. */
  if ((result & MSBBITFRACT32) ^ ((result << 1) & MSBBITFRACT32))
  {
    /* If Bit31=0 and Bit30=1, overflow occured saturate on positive end */
    if (result > 0)
    {
      result = MAXFRACT32;
    }
    /* Else Bit31=1 and Bit30=0, underflow occured saturate on negative end */
    else
    {
      result = MINFRACT32;
    }
  }
  /* Else the multiply produced good results, i.e. the two msb sign bits are 
   * the same.  So now get rid of the extra fractional sign bit (bit31) */
  else
  {
    result = result << 1;
  }
  return (result);
}
