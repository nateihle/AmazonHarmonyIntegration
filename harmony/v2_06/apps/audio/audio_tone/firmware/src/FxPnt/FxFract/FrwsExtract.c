//*****************************************************************************
//
// File: FrwsExtract.c
//
//*****************************************************************************/

#include "FxConvert.h"
#include "FxFract.h"

#define BITSFRACT16 (16)

/******************************************************************************
 *
 * FrwsExtractH()
 *
 * Description:
 *   Extracts upper 16 bits of input 32-bit fractional value and returns them
 *   as 16-bit fractional value.  This is a bit-for-bit extraction of the top
 *   16-bits of the 32-bit input.
 *   This function relates to the ETSI extract_h function.
 *
 * Arguments:
 *   Fract32 a
 *     [in] input argument
 *
 * Return Value:
 *   Fract16 result
 *     [return] Upper 16 bits of 32-bit argument a
 *
 ******************************************************************************/
Fract16 FrwsExtractH(Fract32 a)
{
  Fract16 result;                   /* Value returned */

  /* Bit-for-Bit extract just the top 16-bits */
  result = (Fract16)(a >> BITSFRACT16);
  return (result);
}


/******************************************************************************
 *
 * FrwsExtractL()
 *
 * Description:
 *   Extracts lower 16-bits of input 32-bit fractional value and returns them
 *   as 16-bit fractional value.  This is a bit-for-bit extraction of the
 *   bottom 16-bits of the 32-bit input.
 *   This function relates to the ETSI extract_l function.
 *
 * Arguments:
 *   Fract32 a
 *     [in] input argument
 *
 * Return Value:
 *   Fract16 result
 *     [return] Lower 16 bits of 32-bit argument a
 *
 ******************************************************************************/
Fract16 FrwsExtractL(Fract32 a)
{
  Fract16 result;                   /* Value returned */

  /* Bit-for-Bit extract just the bottom 16-bits */
  result = (Fract16)a;
  return (result);
}
