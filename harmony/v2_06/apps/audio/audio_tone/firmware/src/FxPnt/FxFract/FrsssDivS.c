//*****************************************************************************
//
// File: FrsssDivS.c
//
// Description: Performs fractional division with saturation.
//
//*****************************************************************************/

#include "FxConvert.h"
#include "FxFract.h"


//*****************************************************************************
//
// Subc()
//
// Description:
//   Private function used only by this fractional library to perform
//   one iteration (step) of fractional division routine.
//
// Arguments:
//   Fract32 num;
//     [in] Iteration 1: contains the numerator in the upper 16-bits.
//          Iteration x: for the subsequent iterations it contains the
//                       partial result in the lower 16-bits and partial
//                       remainder in the upper 16-bits.
//   Fract16 b;
//     [in] 16-bit fractional denumerator
//
// Return Value:
//   Frac32 iteration result  <partial_remainder><parital_result>
//     [return]  32-bit value same format as num argument. It is composed of
//                       the partial result in the lower 16-bits and partial
//                       remainder in the upper 16-bits.
//
//*****************************************************************************
static Fract32 Subc(Fract32 num, Fract16 den)
{
  Fract32 result;                   /* Value returned */
  Fract32 temp;                     /* Difference     */

  /* Align the numerator and denominator and subtract the two. Shift the
   * denomintor so the its First bit aligns to the right on the last
   * numerator bit. */
  temp = num - ((Fract32)den << (NUMBITSFRACT16-1));

  /* If the difference from these is not zero, then have next bit of quotient,
   * so place the bit into the bottom of the shifted partial result. Else
   * denomintor is to big so shift the numertor 1 bit and store result for
   * next time. */
  if (temp >= (Fract32)0)
  {
    result = (Fract32)((temp << 1) + 1);
  }
  else
  {
    result = (Fract32)(num << 1);
  }
  return (result);
}


/******************************************************************************
 *
 * FrsssDivS()
 *
 * Description:
 *   Performs fractional division with saturation. There are three restrictions
 *   that the calling code must satisfy.
 *   1. Both the numerator and denominator must be positive.
 *   2. In order to obtain a non-saturated result, the numerator must be LESS
 *      than or equal to the denominator.
 *   3. The denominator must not equal zero.
 *   If 'num' equals 'den', then the result equals MAXINT16.
 *   This function relates to the ETSI div_s function.
 *
 * Arguments:
 *   Fract16 num;
 *     [in] 16-bit fractional numerator
 *   Fract16 den;
 *     [in] 16-bit fractional denumerator
 *
 * Return Value:
 *   Ration a/b in 16-bit fractional format
 *
 ******************************************************************************/
Fract16 FrsssDivS (Fract16 num, Fract16 den)
{
  Fract16 result;      /* Value returned */
  Fract32 partRes;     /* Holds parital result which is subc input/output */
  int16_t signFlag;    /* Holds Final Sign of result */
  int16_t i;           /* Shifting Index */


  /* Keep track of what the final sign is */
  signFlag = (((Fract32)num * den) > 0) ? 0: 1;

  /* Take absolute values so division can be done as unsigned.
   * Convert Numertor value into 32-bits for subc routine. */
  den  = FrssAbs(den);
  partRes = FrwwAbs((Fract32)num);

  /* Make sure that tbe denominator is bigger than numerator, if not
   * result is set to the maximum limits. */
  if(partRes >= den)
  {
    result = (Fract16)((signFlag != 0) ? MININT16: MAXINT16);
  }
  else
  {
    /* Shift the starting numerator up into the top 16 bits. */
    partRes = (partRes << NUMBITSFRACT16);
    /* Perform a 'One-Bit' division for each bit in the denominator.
     * The 'subc' routine is a 'conditional subtraction' local function.  It
     * performs a 'one'bit' division on the input partial result and returns
     * the new partial result.  The partRes input value for the 'First' interation
     * holds the numerator in the top 16-bits and zeros in the lower 16-bits.
     * Every iteration after that it holds the current partial remainder in
     * the top 16-bits and the partial result in the lower 16-bits.  */
    for (i = 0; i < NUMBITSFRACT16; i++)
    {
      partRes = Subc(partRes, den);
    }
    /* The 32-bit partial result holds the remainder in the top 16-bits and the
     * quotient in the lower 16-bits.  Take just the quotient bits.  Shift
     * left by one to account for the fact that division was done as unsigned.
     * This puts the sign bit position back in. */
    partRes = partRes & (uint16_t)(BITMASKFRACT16);
    partRes = (partRes >> 1);
    /* Based on original signs, if output is suppose to be negative, take negative
     * of the parital result. */
    if (signFlag)
    {
      partRes = (-partRes);
    }
    /* Truncate the 32-bit parital result to the 16-bit output result */
    result = (Fract16)partRes;
  }
  return (result);
}
