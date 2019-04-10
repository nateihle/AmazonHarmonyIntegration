//*****************************************************************************
//
// File: FrwwNegate.c
//
// Description: Negate 32-bit 2s-complement fractional value with saturation.
//
//*****************************************************************************/

#include "FxConvert.h"
#include "FxFract.h"


/******************************************************************************
 *
 * FrwwNegate()
 *
 * Description:
 *   Negate 32-bit 2s-complement fractional value with saturation. The
 *   saturation is for handling the case where negating a ATIMINFRACT32 is
 *   greater than ATIMAXFRACT32, or the allowable range of values.
 *   This function relates to the ETSI L_negate function.
 *
 * Arguments:
 *   AtiFract32 a
 *     [in] input argument
 *
 * Return Value:
 *   AtiFract32 result
 *     [return]  Range: ATIMINFRACT32 <= result <= ATIMAXFRACT32
 *
 ******************************************************************************/
Fract32 FrwwNegate(Fract32 a)
{
  Fract32 result;                   /* Value returned */

  /* If anything but the most negative 2's complement input value*/
  if (a != (Fract32)MINFRACT32)
  {
    /* Simply take the negative 2's complement value */
    result = (Fract32)(-a);
  }
  else
  {
    /* else for the max negative value, saturate to the most positive value */
    result = (Fract32)MAXFRACT32;
  }
  return (result);
}
