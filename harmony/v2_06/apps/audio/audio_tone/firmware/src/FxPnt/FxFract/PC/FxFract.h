//*****************************************************************************
// File: FxFract.h
//
// Description: This is header file for Fractional arithmetic library for base
//              C code. The library function naming convention is shown below.
//
//              Target-specific header files may be constructed that replace
//              any number of selected prototypes with macro definitions that
//              implement the same functionality in a different fashion. Some
//              examples would be to use intrinsic CPU primitives in place of
//              the C code or to alter the C code to improve optimization by a
//              specific compiler.
//
//     Fractional Library Naming Convention:
//     ----------------------------------------
//     FrswilOpr() (example only)
//     Fr_______ - prefix for Fractional library functions
//     __s______ - type of 1st argument   ('s' for example)
//     ___w_____ - type of 2nd argument   ('2' for example)
//     ____i____ - type of 3rd argument   ('i' for example)
//     _____l___ - type of return value   ('l' for example)
//     ______Op_ - name of operation
//         Where:  Types of arguments and return value:
//                 s - single precision Fractional (16-bit Q1.15)
//                 d - double precision Fractional (32-bit Q1.31)
//                 i - 16-bit integer (16-bit Q16.0)
//                 l - 32-bit integer (32-bit Q32.0)
//
//     The  Fractional Library implements the following functions.
//     The group and order that they are listed below, is the same
//     order that the function's defintion appears in this file.
//
//     General Purpose Functions:
//     -------------------------
//       1) FrwsSat -
//       2) FrssAbs -
//       3) FrwwAbs -
//       4) FrssNegate -
//       5) FrwwNegate -
//       6) FrwsRound  -
//       7) FrswDepositH -
//       8) FrswDepositL -
//       9) FrwsExtractH -
//      10) FrwsExtractL -
//
//     Addition and Subtraction Functions:
//     ----------------------------------
//       1) FrsssAdd  -
//       2) FrwwwAdd  -
//       3) FrsssSub  -
//       4) FrwwwSub  -
//
//     Normalize and Shift Functions:
//     -----------------------------
//       1) FrsiNorm  -
//       2) FrwiNorm  -
//       3) FrsisShl  -
//       4) FrwiwShl  -
//       5) FrsisShr  -
//       6) FrwiwShr  -
//       7) FrsisShrR -
//       8) FrwiwShrR -
//
//     Multiply and Divide Functions:
//     -----------------------------
//       1) FrsssMult  -
//       2) FrsswMult  -
//       3) FrsssMultR -
//       4) FrsssDivS  -
//
//     Multiply/Accummulate and Multiply/Subtract Functions:
//     ----------------------------------------------------
//       1) FrwsswMac   -
//       2) FrwsssMacR  -
//       3) FrwsswMsu   -
//       4) FrwsssMsuR  -
//
//*****************************************************************************

#ifndef _FXFRACT_H_
#define _FXFRACT_H_

#include "FxTypes.h"


// Defines
#define MSBBITFRACT32   MININT32                    /* 32-bit Sign Bit */
#define MSBBITFRACT16   MININT16                    /* 16-bit Sign Bit */

#define BITMASKFRACT32  (~((uint32_t)0x0L))           /* Bit Mask for 32 */
#define BITMASKFRACT16  (~((uint16_t)0x0))            /* Bit Mask for 16 */

#define NUMBITSFRACT32  ((int16_t)0x020)              /* Num of bits 32  */
#define NUMBITSFRACT16  ((int16_t)0x010)              /* Num of bits 16  */

#define NORMPOSFRACT32  ((Fract32)0x40000000L)      /* Min +val for 32 */
#define NORMPOSFRACT16  ((Fract16)0x4000)           /* Min +val for 16 */

#define NORMNEGFRACT32  ((Fract32)0xc0000000L)      /* Max -val for 32 */
#define NORMNEGFRACT16  ((Fract16)0xc000)           /* Max -val for 16 */

#define ROUNDFRACT32    ((Fract32)0x00008000L)      /* Rounding value  */


// Function Prototypes 
#ifdef  __cplusplus
extern "C" {
#endif

//===== General Purpose Functions ==============================================

//==============================================================================
//
// FrssAbs()
//
// Description:
//   Creates a saturated Absolute value.  It takes the absolute value of the
//   16-bit 2s-complement Fractional input with saturation. The saturation is
//   for handling the case where taking the absolute value of MINFRACT16 is
//   greater than MAXFRACT16, or the allowable range of 16-bit values.
//   This function relates to the ETSI abs function.
//
// Arguments:
//   Fract16 a
//     [in] input argument
//
// Return Value:
//   Fract16 result
//     [return]  abs(input) <= MAXFRACT16
//
//==============================================================================
Fract16 FrssAbs(Fract16);                              /* abs_s(a)      */                                /* abs_s(a) */

//==============================================================================
//
// FrwwAbs()
//
// Description:
//   Creates a saturated Absolute value.  It takes the absolute value of the
//   32-bit 2s-complement Fractional input with saturation. The saturation is
//   for handling the case where taking the absolute value of MINFRACT32 is
//   greater than MAXFRACT32, or the allowable range of 32-bit values.
//   This function relates to the ETSI L_abs function.
//
// Arguments:
//   Fract32 a
//     [in] input argument
//
// Return Value:
//   Fract32 result
//     [return]  abs(input) <= MAXFRACT32
//
//==============================================================================
Fract32 FrwwAbs(Fract32);                              /* L_abs(a)      */


//==============================================================================
//
// FrssNegate()
//
// Description:
//   Negate 16-bit 2s-complement Fractional value with saturation. The
//   saturation is for handling the case where negating a MINFRACT16 is
//   greater than MAXFRACT16, or the allowable range of values.
//   This function relates to the ETSI negate function.
//
// Arguments:
//   Fract16 a
//     [in] input argument
//
// Return Value:
//   Fract16 result
//     [return]  Range: MINFRACT16 <= result <= MAXFRACT16
//
//==============================================================================
Fract16 FrssNegate(Fract16);                           /* negate(a)     */


//==============================================================================
//
// FrwwNegate()
//
// Description:
//   Negate 32-bit 2s-complement Fractional value with saturation. The
//   saturation is for handling the case where negating a MINFRACT32 is
//   greater than MAXFRACT32, or the allowable range of values.
//   This function relates to the ETSI L_negate function.
//
// Arguments:
//   Fract32 a
//     [in] input argument
//
// Return Value:
//   Fract32 result
//     [return]  Range: MINFRACT32 <= result <= MAXFRACT32
//
//==============================================================================
Fract32 FrwwNegate(Fract32);                           /* L_negate(a)   */


//==============================================================================
//
// FrwsRound()
//
// Description:
//   Rounds the lower 16-bits of the 32-bit Fractional input into a 16-bit
//   Fractional value with saturation. This converts the 32-bit Fractional
//   value to 16-bit Fractional value with rounding.  This function calls the
//   'frwwdAdd' function to perform the 32-bit rounding of the input value and
//   'frwsExtractH' function to extract to top 16-bits.  This has the effect of
//   rounding positive Fractional values up and more positive, and has the
//   effect of rounding negative Fractional values up and more positive.
//   This function relates to the ETSI round function.
//
// Arguments:
//   Fract32 a
//     [in] input 32-bit Fractional argument
//
// Return Value:
//   Fract16 result
//     [return]  rounded 16-bit Fractional value
//
//==============================================================================
Fract16 FrwsRound(Fract32);                            /* round(a)      */


//==============================================================================
//
// FrswDepositH()
//
// Description:
//   Composes a 32-bit Fractional value by placing the input 16-bit Fractional
//   value in the composite MSB's and zeros the composite 16-bit LSB's
//   This is a bit-for-bit placement of input 16-bits into the top 32-bit
//   result.
//   This function relates to the ETSI L_deposit_H function.
//
// Arguments:
//   Fract16 a
//     [in] input argument
//
// Return Value:
//   Fract32 result
//     [return] Input 16-bits in upper MSB's and zeros in the lower LSB's
//
//==============================================================================
Fract32 FrswDepositH(Fract16);                         /* L_deposit_h(a)*/


//==============================================================================
//
// FrswDepositL()
//
// Description:
//   Composes a 32-bit Fractional value by placing the 16-bit Fraction input
//   value into the lower 16-bits of the 32-bit composite value. The 16-bit
//   MSB's of the composite output are sign extended. This is a bit-for-bit
//   placement of input 16-bits into the bottom portion of the composite
//   32-bit result with sign extention.
//   This function relates to the ETSI L_deposit_l function.
//
// Arguments:
//   Fract16 a
//     [in] input argument
//
// Return Value:
//   Fract32 result
//     [return] SignExtend 16-bit MSB's and Input Value in lower 16-bit LSB's
//
//==============================================================================
Fract32 FrswDepositL(Fract16);                         /* L_deposit_l(a)*/


//==============================================================================
//
// FrwsExtractH()
//
// Description:
//   Extracts upper 16 bits of input 32-bit Fractional value and returns them
//   as 16-bit Fractional value.  This is a bit-for-bit extraction of the top
//   16-bits of the 32-bit input.
//   This function relates to the ETSI extract_h function.
//
// Arguments:
//   Fract32 a
//     [in] input argument
//
// Return Value:
//   Fract16 result
//     [return] Upper 16 bits of 32-bit argument a
//
//==============================================================================
Fract16 FrwsExtractH(Fract32);                         /* extract_h(a)  */


//==============================================================================
//
// FrwsExtractL()
//
// Description:
//   Extracts lower 16-bits of input 32-bit Fractional value and returns them
//   as 16-bit Fractional value.  This is a bit-for-bit extraction of the
//   bottom 16-bits of the 32-bit input.
//   This function relates to the ETSI extract_l function.
//
// Arguments:
//   Fract32 a
//     [in] input argument
//
// Return Value:
//   Fract16 result
//     [return] Lower 16 bits of 32-bit argument a
//
//==============================================================================
Fract16 FrwsExtractL(Fract32);                         /* extract_l(a)  */

/*----  Addition and Subtraction Functions:  -------------------------------- */

//==============================================================================
//
// FrsssAdd()
//
// Description:
//   Add two 16-bit 2s-complement Fractional (op1 + op2) to produce a 16-bit
//   2s-complement Fractional result with saturation. The saturation is for
//   handling the overflow/underflow cases, where the result is set to
//   MAX16 when an overflow occurs and the result is set to
//   MIN16 when an underflow occurs.  This function does not produce
//   any status flag to indicate when an overflow or underflow has occured.
//   It is assumed that the binary point is in exactly the same bit position
//   for both 16-bit inputs and the resulting 16-bit output.
//
//   This function calls the FrwsSat()
//
// Arguments:
//   Fract16 a
//     [in] Addition operand op1.
//   Fract16 b;
//     [in] Addition operand op2.
//
// Return Value:
//   Fract16 result
//     [return]  Range: MINFRACT16 <= result <= MAXFRACT16
//
//==============================================================================
Fract16 FrsssAdd(Fract16, Fract16);                 /* add(a,b)      */


//==============================================================================
//
// FrwwwAdd()
//
// Description:
//   Add two 32-bit 2s-complement Fractional (op1 + op2) to produce a 32-bit
//   2s-complement Fractional result with saturation. The saturation is for
//   handling the overflow/underflow cases, where the result is set to
//   MAX32 when an overflow occurs and the result is set to
//   MIN32 when an underflow occurs.  This function does not produce
//   any status flag to indicate when an overflow or underflow has occured.
//   It is assumed that the binary point is in exactly the same bit position
//   for both 32-bit inputs and the resulting 32-bit output.
//   This function relates to the ETSI L_add function.
//
// Arguments:
//   Fract32 a
//     [in] Addition operand op1.
//   Fract32 b;
//     [in] Addition operand op2.
//
// Return Value:
//   Fract32 result
//     [return]  Range: MINFRACT32 <= result <= MAXFRACT32
//
//==============================================================================
Fract32 FrwwwAdd(Fract32, Fract32);                 /* L_add(a,b)    */


//==============================================================================
//
// FrsssSub()
//
// Description:
//   Subtract two 16-bit 2s-complement Fractional (op1 - op2) to produce a
//   16-bit 2s-complement Fractional difference result with saturation. The
//   saturation is for handling the overflow/underflow cases, where the result
//   is set to MAX16 when an overflow occurs and the result is set to
//   MIN16 when an underflow occurs.  This function does not produce
//   any status flag to indicate when an overflow or underflow has occured.
//   It is assumed that the binary point is in exactly the same bit position
//   for both 16-bit inputs and the resulting 16-bit output.
//   This function relates to the ETSI sub function.
//
//   This function calls the FrwsSat()
//
// Arguments:
//   Fract16 a
//     [in] Subtraction operand op1.
//   Fract16 b;
//     [in] Subtraction operand op2.
//
// Return Value:
//   Fract16 result
//     [return]  Range: MINFRACT16 <= result <= MAXFRACT16
//
//==============================================================================
Fract16 FrsssSub(Fract16, Fract16);                 /* sub(a,b)      */


//==============================================================================
//
// FrwwwSub()
//
// Description:
//   Subtract two 32-bit 2s-complement Fractional (op1 - op2) to produce a
//   32-bit 2s-complement Fractional result with saturation. The saturation
//   is for handling the overflow/underflow cases, where the result is set to
//   MAX32 when an overflow occurs and the result is set to
//   MIN32 when an underflow occurs.  This function does not produce
//   any status flag to indicate when an overflow or underflow has occured.
//   It is assumed that the binary point is in exactly the same bit position
//   for both 32-bit inputs and the resulting 32-bit output.
//   This function relates to the ETSI L_sub function.
//
// Arguments:
//   Fract32 a
//     [in] Subtraction operand op1.
//   Fract32 b;
//     [in] Subtraction operand op2.
//
// Return Value:
//   Fract32 result
//     [return]  Range: MINFRACT32 <= result <= MAXFRACT32
//
//==============================================================================
Fract32 FrwwwSub(Fract32, Fract32);                 /* L_sub(a,b)    */


/*----  Normalize and Shift Functions:  ------------------------------------- */

//==============================================================================
//
// FrsiNorm()
//
// Description:
//   Produces then number of left shifts needed to Normalize the 16-bit
//   Fractional input.  If the input 'a' is a positive number, it will produce
//   the number of left shifts required to normalized it to the range of a
//   minimum of [(MAXFRACT16+1)/2] to a maximum of [MAXFRACT16].
//   If the input 'a' is a negative number, it will produce
//   the number of left shifts required to normalized it to the range of a
//   minimum of [MINFRACT16] to a maximum of [MINFRACT16/2].
//   This function does not actually normalize the input, it just produces
//   the number of left shifts required.  To actually normalize the value
//   the FrsisShl() function should be used with the value returned From
//   this function.
//   This function relates to the ETSI norm_s function.
//
// Arguments:
//   Fract16 a
//     [in]  16-bit signed integer value to be normalized
//
// Return Value:
//   In16 result
//     [return] The number of left shifts required to normalize the 16-bit input.
//              Range: 0 => result < 16  (i.e. NUMBITSFRACT16)
//                If a>0: 0x4000 > Normalilzed Value <= 0x7fff
//                   i.e.  (MAXFRACT16+1)/2 >  aNorm <= MAXFRACT16
//                If a<0: 0x8000 >= Normalilzed Value < 0xC000
//                   i.e.  MINFRACT16>=  aNorm < MINFRACT16/2
//
//*****************************************************************************/
int16_t   FrsiNorm(Fract16);                             /* norm_s(a)     */


//==============================================================================
//
// FrdiNorm()
//
// Description:
//   Produces then number of left shifts needed to Normalize the 32-bit
//   Fractional input.  If the input 'a' is a positive number, it will produce
//   the number of left shifts required to normalized it to the range of a
//   minimum of [(MAXFRACT32+1)/2] to a maximum of [MAXFRACT32].
//   If the input 'a' is a negative number, it will produce
//   the number of left shifts required to normalized it to the range of a
//   minimum of [MINFRACT32] to a maximum of [MINFRACT32/2].
//   This function does not actually normalize the input, it just produces
//   the number of left shifts required.  To actually normalize the value
//   the FrdidShl() function should be used with the value returned From
//   this function.
//   This function relates to the ETSI norm_l function.
//
// Arguments:
//   Fract32 a
//     [in]  32-bit signed integer value to be normalized
//
// Return Value:
//   In16 result
//     [return] The number of left shifts required to normalize the 32-bit input.
//              Range: 0 => result < 32  (i.e. NUMBITSFRACT32)
//                If a>0: 0x40000000 > Normalilzed Value <= 0x7fffffff
//                   i.e.  (MAXFRACT32+1)/2 >  aNorm <= MAXFRACT32
//                If a<0: 0x80000000 >= Normalilzed Value < 0xC0000000
//                   i.e.  MINFRACT32>=  aNorm < MINFRACT32/2
//
//==============================================================================
int16_t   FrdiNorm(Fract32);                             /* norm_l(a)     */


//==============================================================================
//
// FrsisShl()
//
// Description:
//   Performs an 'Arithmetic' Shift of the 16-bit input argument 'a' left
//   by the input argument 'b' bit positions.  If 'b' is a positive number,
//   a 16-bit left shift is performed with 'zeros' inserted to the right
//   of the shifted bits.  If 'b' is a negative number, a right shift by abs(b)
//   positions with 'sign extention' is done by calling the FrsisShr() function.
//   Saturation is applied if shifting causes an overflow or an underflow.
//   This function relates to the ETSI shl function.
//
// Arguments:
//   Fract16 a
//     [in]  16-bit signed integer value to be shifted
//   int16_t b
//     [in]  16-bit signed integer shift index
//          positive value: # of bits to left shift (zeros inserted at LSB's)
//                          {To not always saturate:
//                              if 'a=0', then max b=15, else max b=14}
//          negative value: # of bits to right shift (sign extend)
//
// Return Value:
//   Fract16 result
//     [return]  Arithmetically shifted 16-bit signed integer output
//
Fract16 FrsisShl(Fract16, int16_t);                   /* shl(a,b)      */


//==============================================================================
//
// FrdidShl()
//
// Description:
//   Performs an 'Arithmetic' Shift of the 32-bit input argument 'a' left
//   by the input argument 'b' bit positions.  If 'b' is a positive number,
//   a 32-bit left shift is performed with 'zeros' inserted to the right
//   of the shifted bits.  If 'b' is a negative number, a 32-bit right shift
//   by b bit positions with 'sign extention' is done by calling the FrdidShr()
//   function.  Saturation is applied if shifting causes an overflow or an
//   underflow.
//   This function relates to the ETSI L_shl function.
//
// Arguments:
//   Fract32 a
//     [in]  32-bit signed integer value to be shifted
//   int16_t b
//     [in]  16-bit signed integer shift index
//          positive value: # of bits to left shift (zeros inserted at LSB's)
//          negative value: # of bits to right shift (sign extend)
//
// Return Value:
//   Fract32 result
//     [return]  Arithmetically shifted 32-bit signed integer output
//
//==============================================================================
Fract32 FrwiwShl(Fract32, int16_t);                   /* L_shl(a,b)    */


//==============================================================================
//
// FrsisShr()
//
// Description:
//   Performs an 'Arithmetic' RIGHT Shift on a 16-bit input by 'b' bit
//   positions.  For positive shift directions (b>0), 'b' Lsb-bits are shifted
//   out to the right and 'b' sign-extended Msb-bits fill in From the left.
//   For negative shift directions (b<0), 'b' Lsb's are shifted to the
//   LEFT with 0's filling in the empty lsb position. The left shifting
//   causes 'b' Msb-bits to fall off to the left, saturation is applied to
//   any shift left value that overflows. This function calls the
//   FrsisShl() function to perform any 16-bit left shifts.  This function
//   does not provide any status-type information to indicate when overflows
//   occur.
//   This function relates to the ETSI shr function.
//
// Arguments:
//   Fract16 a
//     [in]  16-bit signed input value to shift
//   int16_t b
//     [in]  16-bit signed integer shift index
//          positive value: # of bits to right shift (sign extend)
//                          { To get all sign bits, b>=15 }
//          negative value: # of bits to left shift (zeros inserted at LSB's)
//
// Return Value:
//   Fract16 result
//     [return]  16-bit signed shifted output
//
//==============================================================================
Fract16 FrsisShr(Fract16, int16_t);                   /* shr(a,b)      */


//==============================================================================
//
// FrdidShr()
//
// Description:
//   Performs an 'Arithmetic' RIGHT Shift on a 32-bit input by 'b' bit
//   positions.  For positive shift directions (b>0), 'b' Lsb-bits are shifted
//   out to the right and 'b' sign-extended Msb-bits fill in From the left.
//   For negative shift directions (b<0), 'b' Lsb's are shifted to the
//   LEFT with 0's filling in the empty lsb position. The left shifting
//   causes 'b' Msb-bits to fall off to the left, saturation is applied to
//   any shift left value that overflows. This function calls the
//   FrdidShl() function to perform any 32-bit left shifts.  This function
//   does not provide any status-type information to indicate when overflows
//   occur.
//   This function relates to the ETSI L_shr function.
//
// Arguments:
//   Fract32 a
//     [in]  32-bit signed integer value to be shifted
//   int16_t b
//     [in]  16-bit signed integer shift index
//          positive value: # of bits to right shift (sign extend)
//          negative value: # of bits to left shift (zeros inserted at LSB's)
//
// Return Value:
//   Fract32 result
//     [return]  Arithmetically shifted 32-bit signed integer output
//
//==============================================================================
Fract32 FrwiwShr(Fract32, int16_t);                   /* L_shr(a,b)    */


//==============================================================================
//
// FrsisShrR()
//
// Description:
//
// Description:
//   Performs an 'Arithmetic' RIGHT Shift on a 16-bit input by 'b' bits with
//   Rounding applied.  The rounding occurs by adding a bit weight of "1/2 Lsb",
//   where the "Lsb" is the Ending (shifted) Lsb. For example: The initial Bit#(b)
//   is after the right shift Bit#(0), so the rounding bit weight is Bit#(b-1).
//   Rounding does not occur on either left shifts or on no shift needed cases.
//   For positive shift directions (b>0), 'b' Lsb-bits are shifted out
//   to the right and 'b' sign-extended Msb-bits fill in From the left.
//   For negative shift directions (b<0), 'b' Lsb's are shifted to the LEFT
//   with 0's filling in the empty lsb position. The left shifting causes
//   'b' Msb-bits to fall off to the left, saturation is applied to any shift
//   left value that overflows. This function calls the FrsisShl() function
//   to perform the actual 16-bit left shift.  This function does not provide
//   any status-type information to indicate when overflows occur.
//
//   This function relates to the ETSI shr_r function.
//
// Arguments:
//   Fract16 a
//     [in]  16-bit signed integer value to be shifted
//   int16_t b
//     [in]  16-bit signed integer shift index
//          positive value: # of bits to right shift (sign extend)
//                          {b > 15, results in all sign bits}
//          negative value: # of bits to left shift (zeros inserted at LSB's)
//
// Return Value:
//   Fract16 result
//     [return]  Arithmetically shifted 16-bit signed integer output
//
//==============================================================================
Fract16 FrsisShrR(Fract16, int16_t);                  /* shr_r(a,b)    */


//*****************************************************************************
//
// FrdidShrR()
//
// Description:
//   Performs an 'Arithmetic' RIGHT Shift on a 32-bit input by 'b' bits with
//   Rounding applied.  The rounding occurs before any shift by adding a
//   bit weight of "1/2 Lsb", where the "Lsb" is the Ending (shifted) Lsb.
//   For example: The initial Bit#(i+b) is after the right shift Bit#(i),
//   so the rounding bit weight is Bit#(i+b-1).  Rounding does not occur
//   on left shifts, when b is negative.  After rounding, this function
//   calls the FrdidShr() function to perform the actual 32-bit right shift.
//   For positive shift directions (b>0), 'b' Lsb-bits are shifted out
//   to the right and 'b' sign-extended Msb-bits fill in From the left.
//   For negative shift directions (b<0), 'b' Lsb's are shifted to the LEFT
//   with 0's filling in the empty lsb position. The left shifting causes
//   'b' Msb-bits to fall off to the left, saturation is applied to any shift
//   left value that overflows. This function calls the FrdidShl() function
//   to perform the actual 32-bit left shift.  This function does not provide
//   any status-type flag to indicate occurence of overflow.
//   This function relates to the ETSI L_shr_r function.
//
// Arguments:
//   Fract32 a
//     [in]  32-bit signed integer value to be shifted
//   int16_t b
//     [in]  16-bit signed integer shift index
//          positive value: # of bits to right shift (sign extend)
//          negative value: # of bits to left shift (zeros inserted at LSB's)
//
// Return Value:
//   Fract32 result
//     [return]  Arithmetically shifted 32-bit signed integer output
//
//==============================================================================
Fract32 FrwiwShrR(Fract32, int16_t);                  /* L_shr_r(a,b)  */


/*----  Multiply and Divide  Functions:  ------------------------------------ */

//==============================================================================
//
// FrsssMult()
//
// Description:
//   Performs Fractional multiplication of two 16-bit Fractional values
//   and returns a 16-bit Fractional scaled result. The function performs
//   a Q15xQ15->Q30 bit multiply with a left shift by '1' to give a Q31
//   result.  This automatic shift left is done to get rid of the extra sign
//   bit that occurs in the interpretation of the Fractional multiply result.
//   Saturation is applied to any 32-bit result that overflows. Only the TOP
//   16-bits are the extracted and returned. This function is for Fractional
//   'Qtype' data only and it therefore will not give the correct results for
//   true integers (because left shift by '1'). For the special case where
//   both inputs equal the MINFACT16, the function returns a value equal
//   to MAXFACT16, i.e. 0x7fff = FrsssMult(0x8000,0x8000).
//   This function internally calls the FrsswMult() routine to perform
//   the acutal multiplication.
//   This function relates to the ETSI mult function.
//
// Arguments:
//   Fract16 a
//     [in]  16-bit signed integer (fractQtype) operand 1
//   Fract16 b
//     [in]  16-bit signed integer (fractQtype) operand 2
//
// Return Value:
//   Fract16 result
//     [return] 16-bit signed integer (fractQtype) output value
//
//==============================================================================
Fract16 FrsssMult(Fract16, Fract16);                /* mult(a,b)     */

//==============================================================================
//
// FrsswMult()
//
// Description:
//   Performs Fractional multiplication of two 16-bit Fractional values
//   and returns the 32-bit Fractional scaled result. The function
//   performs the Q15xQ15->Q30 Fractional bit multiply.  It then shifts the
//   result left by '1', to give a Q31 type result, (the lsb is zero-filled).
//   This automatic shift left is done to get rid of the extra sign bit
//   that occurs in the interpretation of the Fractional multiply result.
//   Saturation is applied to any results that overflow, and then the
//   function returns the 32-bit Fractional Qd31 result.  This function is
//   for Fractional 'Qtype' data only and it therefore will not give
//   correct results for true integers (because left shift by '1').
//   For the special case where both inputs equal the MINFACT16, the
//   function returns a value equal to MAXFACT32,
//   i.e. 0x7fffffff = FrsswMult(0x8000,0x8000).
//   This function relates to the ETSI L_mult function.
//
// Arguments:
//   Fract16 a
//     [in]  16-bit signed integer (fractQtype) operand 1
//   Fract16 b
//     [in]  16-bit signed integer (fractQtype) operand 2
//
// Return Value:
//   Fract32 result
//     [return] 32-bit signed integer (fractQtype) output value
//
//==============================================================================
Fract32 FrsswMult(Fract16, Fract16);                /* L_mult(a,b)   */

//==============================================================================
//
// FrsssMultR()
//
// Description:
//   Performs Fractional multiplication of two 16-bit Fractional values
//   and returns a ROUNDED 16-bit Fractional result. The function performs
//   a Q15xQ15->Q30 bit multiply with a left shift by '1' to give a Q31
//   result.  This automatic shift left is done to get rid of the extra sign
//   bit that occurs in the interpretation of the Fractional multiply result.
//   Saturation is applied to any 32-bit result that overflows. Rounding
//   is applied to the 32-bit SHIFTED result by adding in a weight factor
//   of 2^15, again any overflows are saturated.  The TOP 16-bits are
//   extracted and returned.  This function is for Fractional 'Qtype' data
//   only and it therefore will not give the correct results for
//   true integers (because left shift by '1').  This function assumes that
//   the binary point in the 32-bit shifted multiplier ouput is between
//   bit_16 and bit_15 when the rounding factor is added. For the special
//   case where both inputs equal the MINFACT16, the function returns a
//   value equal to MAXFACT16, i.e. 0x7fff = FrsssMult(0x8000,0x8000).
//   This function internally calls the FrsswMult() routine to perform
//   the acutal multiplication and the FrwsRound() routine to perform the
//   actual rounding.
//   This function relates to the ETSI mult_r function.
//
// Arguments:
//   Fract16 a
//     [in]  16-bit signed integer (fract_Q1d15) operand 1
//   Fract16 b
//     [in]  16-bit signed integer (fract_Q1d15) operand 2
//
// Return Value:
//   Fract16 result
//     [return]  rounded 16-bit signed integer (fract_Q1d15) output value
//
//==============================================================================
Fract16 FrsssMultR(Fract16, Fract16);               /* mult_r(a,b)   */


//==============================================================================
//
// FrsssDivS()
//
// Description:
//   Performs Fractional division with saturation. There are three restrictions
//   that the calling code must satisfy.
//   1. Both the numerator and denominator must be positive.
//   2. In order to obtain a non-saturated result, the numerator must be LESS
//      than or equal to the denominator.
//   3. The denominator must not equal zero.
//   If 'num' equals 'den', then the result equals MAXINT16.
//   This function relates to the ETSI div_s function.
//
// Arguments:
//   Fract16 num;
//     [in] 16-bit Fractional numerator
//   Fract16 den;
//     [in] 16-bit Fractional denumerator
//
// Return Value:
//   Ration a/b in 16-bit Fractional format
//
//==============================================================================
Fract16 FrsssDivS(Fract16, Fract16);                /* div_s(a,b)    */

/*----  Multiply/Add and Multiply/Sub Functions:  --------------------------- */

//==============================================================================
//
// FrwsswMac()
//
// Description:
//   Performs a Multiply-Accumulate function WITH saturation.  This routine
//   returns the Fractional 32-bit result From the accumulator output
//   'SAT(addOut_Q1d31)=outQ1d15' where 'multOut_Q1d31 + a_Q1d31 = addOut_Q1d31',
//   and 'b_Q1d15 x c_Q1d15 = multOut_Q1d31'. The multiply is performed on the
//   two 16-bit Fractional input values 'b x c' which results in a 32-bit value.
//   This result is left shifted by one to account for the extra sign bit
//   inherent in the Fractional-type multiply.  The shifted number represents
//   a Q1d31 number with the lsb set to '0'.  This Q1d31 number is added with
//   the 32-bit Fractional input argument 'a'.  Saturation is applied on the
//   output of the accumulator to keep the value within the 32-bit Fractional
//   range and then this value is returned.
//   This instruction is equalivalent to performing the following functions:
//         FrwsswMac(a,b,c) ====> FrwwwAdd(a, FrsswMult(b,c)));
//   The FrwsswMac() function is for Fractional Qtype format data only and it
//   therefore will not give the correct results for true integers.
//   This function relates to the ETSI L_mac function.
//
// Arguments:
//   Fract32 a
//     [in]  32-bit accumulator operand 1
//   Fract16 b
//     [in]  16-bit multiplication operand 1
//   Fract16 c
//     [in]  16-bit multiplication operand 2
//
// Return Value:
//   Fract32 result
//     [return] 32-bit
//
//==============================================================================
Fract32 FrwsswMac(Fract32, Fract16, Fract16);    /* L_mac(a,b,c)  */

//==============================================================================
//
// FrwsssMacR()
//
// Description:
//   This function is like FrwsswMac() but WITH Rounding applied to the
//   accumulator result before it is saturated and the top 16-bits taken.
//   This function first multiplies the two 16-bit input values 'b x c' which
//   results in a 32-bit value.  This result is left shifted by one to account
//   for the extra sign bit inherent in the Fractional-type multiply. So, the
//   shifted number now has a '0' in the Lsb. The shifted multiplier output is
//   then added to the 32-bit Fractional input 'a'. Then the 32-bits of the
//   accumulator output are rounded by adding '2^15'.  This value is then
//   saturated to be within the Fract16 range.
//   It is assumed that the binary point of the 32-bit input value a is in
//   the same bit position as the shifted multiplier output.
//   The FrwsssMacR() function is for Fractional Qtype format data only and
//   it therefore will not give the correct results for true integers.
//   This function relates to the ETSI L_mac_r function.
//
// Arguments:
//   Fract32 a
//     [in]  32-bit accumulator operand 1
//   Fract16 b
//     [in]  16-bit multiplication operand 1
//   Fract16 c
//     [in]  16-bit multiplication operand 2
//
// Return Value:
//   Fract16 result
//     [return] 16-bit
//
//==============================================================================
Fract16 FrwsssMacR(Fract32, Fract16, Fract16);   /* mac_r(a,b,c)  */


//==============================================================================
//
// FrwsswMsu()
//
// Description:
//   Performs a Multiply-Subtraction function WITH saturation.  This routine
//   returns the Fractional 32-bit result From the subtractor output
//   'SAT(addOut_Q1d31)=outQ1d15' where 'a_Q1d31 - multOut_Q1d31 = addOut_Q1d31',
//   and 'b_Q1d15 x c_Q1d15 = multOut_Q1d31'. The multiply is performed on the
//   two 16-bit Fractional input values 'b x c' which results in a 32-bit value.
//   This result is left shifted by one to account for the extra sign bit
//   inherent in the Fractional-type multiply.  The shifted number represents a
//   Q1d31 number with the lsb set to '0'.  This Q1d31 number then is subtracted
//   From the 32-bit Fractional input argument 'a'.  Saturation is
//   applied on the output of the accumulator(Subtractor) to keep the value
//   within the 32-bit Fractional range and then this value is returned.
//   This instruction is equalivalent to performing the following functions:
//         FrwsswMsu(a,b,c) ====> FrwwwSub(a, FrsswMult(b,c)));
//   The FrwsswMsu() function is for Fractional Qtype format data only and it
//   therefore will not give the correct results for true integers.
//   This function relates to the ETSI L_msu function.
//
// Arguments:
//   Fract32 a
//     [in]  32-bit operand that is subtracted From
//   Fract16 b
//     [in]  16-bit multiplication operand 1
//   Fract16 c
//     [in]  16-bit multiplication operand 2
//
// Return Value:
//   Fract32 result
//     [return] 32-bit
//
//==============================================================================
Fract32 FrwsswMsu(Fract32, Fract16, Fract16);    /* L_msu(a,b,c)  */

/******************************************************************************
//==============================================================================
 *
 * FrwsssMsuR()
 *
 * Description:
 *   This function is like FrwsswMsu() but WITH Rounding applied to the
 *   subtractor result before it is saturated and the top 16-bits taken.
 *   This function first multiplies the two 16-bit input values 'b x c' which
 *   results in a 32-bit value.  This result is left shifted by one to account
 *   for the extra sign bit inherent in the Fractional-type multiply. So, the
 *   shifted number now has a '0' in the Lsb. The shifted multiplier output
 *   is then SUBTRACTED From the 32-bit Fractional input 'a'. Then the 32-bits
 *   output From this subtraction are rounded by adding '2^15'.  This value
 *   is then saturated to be within the Fract16 range.
 *   It is assumed that the binary point of the 32-bit input value a is in
 *   the same bit position as the shifted multiplier output.
 *   The FrwsssMsuR() function is for Fractional Qtype format data only and
 *   it therefore will not give the correct results for true integers.
 *   This function relates to the ETSI msu_r function.
 *
 * Arguments:
 *   Fract32 a
 *     [in]  32-bit 32-bit Value which is subtracted From
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
//==============================================================================
Fract16 FrwsssMsuR(Fract32, Fract16, Fract16);   /* msu_r(a,b,c)  */

#ifdef  __cplusplus
}
#endif

#endif /* _FXFRACT_H_ */

