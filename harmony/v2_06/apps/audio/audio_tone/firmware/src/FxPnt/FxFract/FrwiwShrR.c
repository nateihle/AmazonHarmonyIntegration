//*****************************************************************************
//
// File: FrwiwShrR.c
//
// Description: Performs an 'Arithmetic' Right Shift of the 32-bit input 
//   argument 'a' by the input argument 'b' bit positions with Rounding applied.
//
//*****************************************************************************

#include "FxConvert.h"
#include "FxFract.h"


/******************************************************************************
 *
 * FrwiwShrR()
 *
 * Description:
 *   Performs an 'Arithmetic' RIGHT Shift on a 32-bit input by 'b' bits with 
 *   Rounding applied.  The rounding occurs before any shift by adding a
 *   bit weight of "1/2 Lsb", where the "Lsb" is the Ending (shifted) Lsb.
 *   For example: The initial Bit#(i+b) is after the right shift Bit#(i),
 *   so the rounding bit weight is Bit#(i+b-1).  Rounding does not occur
 *   on left shifts, when b is negative.  After rounding, this function
 *   calls the frdidShr() function to perform the actual 32-bit right shift.
 *   For positive shift directions (b>0), 'b' Lsb-bits are shifted out 
 *   to the right and 'b' sign-extended Msb-bits fill in from the left.  
 *   For negative shift directions (b<0), 'b' Lsb's are shifted to the LEFT 
 *   with 0's filling in the empty lsb position. The left shifting causes 
 *   'b' Msb-bits to fall off to the left, saturation is applied to any shift 
 *   left value that overflows. This function calls the frdidShl() function 
 *   to perform the actual 32-bit left shift.  This function does not provide 
 *   any status-type flag to indicate occurence of overflow.
 *   This function relates to the ETSI L_shr_r function.
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
Fract32 FrwiwShrR(Fract32 a, int16_t b)
{  
  Fract32 result;                   /* Value returned */
  Fract32 rndNum;  /* Var for calculating rounded input */
       
  /* Check if 'Msb' shifts past the 'Lsb'.*/
  if (b > (NUMBITSFRACT32 - 1))
  {
    /* After shifting & ROUNDING the result is always '0'.*/
    result = 0;
  }
  else
  {
    /* Else the Msb isn't shifted out. The standard 32-bit right shift is done, 
     * including Left & No shift cases */ 
    result = FrwiwShr(a, b);
    /* If Right shift then check to see if result should be modified. */
    if (b > 0)
    {
      /* The Round weight is '1/2 of an ENDING Lsb', i.e. Bit'b' shifts 
       * to Bit'0' so the preshifted rounding weigth is Bit'b-1'. Adding in 
       * the rounding weight will only change the results if the original 
       * '1/2 Lsb' Bit'b-1' is set. */
      if ((a & ((Fract32)1 << (b - 1))) != 0)
      {
        /* The 1/2 Lsb bit is set to round up by a full '1' Lsb.  Saturate
         * any overflow cases.  */
        rndNum = (Fract32)1;
        result = FrwwwAdd(result, rndNum);  
      }
    }
  }
  return (result);
}
