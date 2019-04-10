//*****************************************************************************
//
// File: FrwsRound.c
//
// Description: Rounds the lower 16-bits of the 32-bit fractional input into a 
//   16-bit fractional value with saturation.
//
//*****************************************************************************/

#include "FxConvert.h"
#include "FxFract.h"


/******************************************************************************
 *
 * FrwsRound()
 *
 * Description:
 *   Rounds the lower 16-bits of the 32-bit fractional input into a 16-bit
 *   fractional value with saturation. This converts the 32-bit fractional
 *   value to 16-bit fractional value with rounding.  This function calls the
 *   'FrwwwAdd' function to perform the 32-bit rounding of the input value and
 *   'FrwsExtractH' function to extract to top 16-bits.  This has the effect of
 *   rounding positive fractional values up and more positive, and has the 
 *   effect of rounding negative fractional values up and more positive.
 *   This function relates to the ETSI round function.
 *
 * Arguments:
 *   Fract32 a
 *     [in] input 32-bit fractional argument
 *
 * Return Value:
 *   Fract16 result
 *     [return]  rounded 16-bit fractional value
 *
 ******************************************************************************/
Fract16 FrwsRound(Fract32 a)
{
  Fract16 result;                   /* Value returned */
  Fract32 temp;     /* Temp var to hold frdddAdd result  */

  /* Add Fractional Rounding bit to 32-bit input so that the upper 16-bits
   * of the input round up.  Saturation is part of the addition. */
  temp = FrwwwAdd(a,(Fract32)ROUNDFRACT32);
  /* Extract the upper 16-bits from the rounded 32-bit sum. */
  result = FrwsExtractH(temp);
  return (result);
}
