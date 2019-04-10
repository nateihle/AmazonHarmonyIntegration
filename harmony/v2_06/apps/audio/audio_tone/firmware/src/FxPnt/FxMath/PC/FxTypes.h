//=============================================================================
//
// File: FxTypes.h
//
// Description: Fixed point data types.
//
//=============================================================================

#ifndef _FXTYPES_H_
#define _FXTYPES_H_

// Fractional data types. 
typedef int16_t Fract16; //Qn
typedef int16_t FxPnt16; //Q?

typedef int32_t Fract32; //Qn
typedef int32_t FxPnt32; //Q?

typedef int64_t Fract64; //Qn
typedef int32_t FxPnt64; //Q?

// FX floating point type
// (extended FxQflExt32 type useful for assigning constants with f2Qfloat32) 
struct {
  Fract16   man;
  FxInt16   exp;
} FxQFloat32;


// Maximum and minimum values for 16-bit data types. 
#define MAXINT16   ((int16_t) 0x7fff)
#define MININT16   ((int16_t) (-FXMAXINT16 - 1))
#define MAXFRACT16 FXMAXINT16       /* +0.999969 */
#define MINFRACT16 FXMININT16       /* -1.000000 */

#define MAXINT32   ((int32_t)0x7fffffffL)
#define MININT32   ((int32_t)(-FXMAXINT32 - 1))
#define FXMAXFRACT32 FXMAXINT32       /* +0.9999999995 */
#define FXMINFRACT32 FXMININT32       /* -1.0000000000 */

// minimum and maximum definitions for FX floating point data types 
#define FXMAXPFLOAT32 {FXMAXFRACT16, FXMAXINT16}
#define FXMINPFLOAT32 {FXMINFRACT16, FXMAXINT16}

#define FXUNITYFLOAT 1.0

// Null pointer value (normally, NULL is defined in stddef.h) 
#define FXNULL ((void *)0)

#endif  /* _FXTYPES_H_ */
