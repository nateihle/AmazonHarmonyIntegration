//*****************************************************************************
//
// File: FrsisShrR.c
//
// Description: Performs an 'Arithmetic' Right Shift of the 16-bit input 
//   argument 'a' by the input argument 'b' bit positions with Rounding applied.
//
//*****************************************************************************/

#include "FxConvert.h"
#include "FxFract.h"


/******************************************************************************
 *
 * FrsisShrR()
 *
 * Description:
 *
 * Description:
 *   Performs an 'Arithmetic' RIGHT Shift on a 16-bit input by 'b' bits with 
 *   Rounding applied.  The rounding occurs by adding a bit weight of "1/2 Lsb",
 *   where the "Lsb" is the Ending (shifted) Lsb. For example: The initial 
 *   Bit#(b) is after the right shift Bit#(0), so the rounding bit weight is
 *   Bit#(b-1).  
 *
 *   Rounding does not occur on either left shifts or on no shift needed cases. 
 *   For positive shift directions (b>0), 'b' Lsb-bits are shifted out 
 *   to the right and 'b' sign-extended Msb-bits fill in from the left.  
 *   For negative shift directions (b<0), 'b' Lsb's are shifted to the LEFT 
 *   with 0's filling in the empty lsb position. The left shifting causes 
 *   'b' Msb-bits to fall off to the left, saturation is applied to any shift 
 *   left value that overflows. This function calls the frsisShl() function 
 *   to perform the actual 16-bit left shift.  This function does not provide 
 *   any status-type information to indicate when overflows occur.
 *
 *   This function relates to the ETSI shr_r function.
 *
 * Arguments:
 *   Fract16 a
 *     [in]  16-bit signed integer value to be shifted
 *   int16_t  b
 *     [in]  16-bit signed integer shift index 
 *          positive value: # of bits to right shift (sign extend)
 *                          {b > 15, results in all sign bits}  
 *          negative value: # of bits to left shift (zeros inserted at LSB's)
 *
 * Return Value:
 *   Fract16 result
 *     [return]  Arithmetically shifted 16-bit signed integer output 
 *
 ******************************************************************************/
Fract16 FrsisShrR(Fract16 a, int16_t  b)
{  
  Fract16 result;                   /* Value returned */
  Fract16 rndNum;  /* Var for calculating rounded input */
    
  /* Check if 'Msb' shifts past the 'Lsb'.*/
  if (b > (NUMBITSFRACT16 - 1))
  {
    /* After shifting & ROUNDING the result is always '0'.*/
    result = 0;
  }
  else
  {
    /* Else the Msb isn't shifted out. The standard 16-bit right shift is done, 
     * including Left & No shift cases */ 
    result = FrsisShr(a, b);
    /* If Right shift then check to see if result should be modified. */
    if (b > 0)
    {
      /* The Round weight is '1/2 of an ENDING Lsb', i.e. Bit'b' shifts 
       * to Bit'0' so the preshifted rounding weigth is Bit'b-1'. Adding in 
       * the rounding weight will only change the results if the original 
       * '1/2 Lsb' Bit'b-1' is set. */
      if ((a & ((Fract16)1 << (b - 1))) != 0)
      {
        /* The 1/2 Lsb bit is set to round up by a full '1' Lsb.  Saturate
         * any overflow cases.  */
        rndNum = (Fract16)1;
        result = FrsssAdd(result, rndNum);  
      }
    }
  }
  return (result);
}
