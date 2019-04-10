//*****************************************************************************
//
// File: FrwsssMacR.c
//
// Description: This function is like frdssdMac() but WITH Rounding applied to 
//   the accumulator result before it is saturated and the top 16-bits taken.
//
//*****************************************************************************/

#include "FxConvert.h"
#include "FxFract.h"


/******************************************************************************
 *
 * FrwsssMacR()
 *
 * Description:
 *   This function is like frdssdMac() but WITH Rounding applied to the 
 *   accumulator result before it is saturated and the top 16-bits taken.
 *   This function first multiplies the two 16-bit input values 'b x c' which 
 *   results in a 32-bit value.  This result is left shifted by one to account 
 *   for the extra sign bit inherent in the fractional-type multiply. So, the
 *   shifted number now has a '0' in the Lsb. The shifted multiplier output is
 *   then added to the 32-bit fractional input 'a'. Then the 32-bits of the
 *   accumulator output are rounded by adding '2^15'.  This value is then
 *   saturated to be within the Fract16 range.  
 *   It is assumed that the binary point of the 32-bit input value a is in
 *   the same bit position as the shifted multiplier output.
 *   The frdsssMacR() function is for fractional Qtype format data only and
 *   it therefore will not give the correct results for true integers. 
 *   This function relates to the ETSI L_mac_r function.
 *
 * Arguments:
 *   Fract32 a
 *     [in]  32-bit accumulator operand 1
 *   Fract16 b
 *     [in]  16-bit multiplication operand 1
 *   Fract16 c
 *     [in]  16-bit multiplication operand 2
 *
 * Return Value:
 *   Fract16 result
 *     [return] 16-bit  
 *
 ******************************************************************************/
Fract16 FrwsssMacR(Fract32 a, Fract16 b, Fract16 c)
{  
  Fract16 result;                   /* Value returned */
  Fract32 macOut;    /* Temp var for 32-bit Mac operations */

  /* Call frdssdMac() which multiplies of the 16-bit inputs, producing a 32-bit 
   * result.  It then shifts the result left by 1 bit (to get rid of the extra 
   * sign bit), and adds it to the 32-bit input 'a' */
  macOut = FrwsswMac(a, b, c);
  /* Add Fractional Rounding to 32-bit Mac Results, by rounding by 2^15.
   * Then saturate the rounded result.  The frddAdd() function performs both. */
  macOut = FrwwwAdd(macOut,(Fract32) ROUNDFRACT32);
  /* Extract the upper 16-bits from the rounded 32-bit sum */
  result = FrwsExtractH(macOut);
  return (result);
}
