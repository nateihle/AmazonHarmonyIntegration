//*****************************************************************************
//
// File: FrssNegate.c
//
// Description: Negate 16-bit 2s-complement fractional value with saturation.
//
//*****************************************************************************/

#include "FxConvert.h"
#include "FxFract.h"


/******************************************************************************
 *
 * FrssNegate()
 *
 * Description:
 *   Negate 16-bit 2s-complement fractional value with saturation. The
 *   saturation is for handling the case where negating a MINFRACT16 is
 *   greater than MAXFRACT16, or the allowable range of values.
 *   This function relates to the ETSI negate function.
 *
 * Arguments:
 *   Fract16 a
 *     [in] input argument
 *
 * Return Value:
 *   Fract16 result
 *     [return]  Range: MINFRACT16 <= result <= MAXFRACT16
 *
 ******************************************************************************/
Fract16 FrssNegate(Fract16 a)
{
  Fract16 result;                   /* Value returned */

  /* If anything but the most negative 2's complement input value*/
  if (a != (Fract16)MINFRACT16)
  {
    /* Simply take the negative 2's complement value */
    result = (Fract16)(-a);
  }
  else
  {
    /* Else for the max negative value, saturate to the most positive value */
    result = (Fract16)MAXFRACT16;
  }
  return (result);
}
