/****************************************************************************
*
*    Copyright (c) 2005 - 2014 by Vivante Corp.  All rights reserved.
*
*    The material in this file is confidential and contains trade secrets
*    of Vivante Corporation. This is proprietary information owned by
*    Vivante Corporation. No part of this work may be disclosed,
*    reproduced, copied, transmitted, or used in any way for any purpose,
*    without the express written permission of Vivante Corporation.
*
*****************************************************************************/


#ifndef _nano2D_types_h__
#define _nano2D_types_h__

#ifdef __cplusplus
extern "C" {
#endif

#define IN
#define OUT

typedef int                 n2d_bool_t;
typedef unsigned char       n2d_uint8_t;
typedef short               n2d_int16_t;
typedef unsigned short      n2d_uint16_t;
typedef int                 n2d_int32_t;
typedef unsigned int        n2d_uint32_t;
typedef unsigned long long  n2d_uint64_t;
typedef unsigned int        n2d_size_t;
typedef float               n2d_float_t;

#ifdef __cplusplus
#define N2D_NULL 0
#else
#define N2D_NULL ((void *) 0)
#endif

#define N2D_TRUE  1
#define N2D_FALSE 0

#define N2D_INFINITE ((n2d_uint32_t) ~0U)

#define __gcmSTART(reg_field) \
    (0 ? reg_field)

#define __gcmEND(reg_field) \
    (1 ? reg_field)

#define __gcmGETSIZE(reg_field) \
    (__gcmEND(reg_field) - __gcmSTART(reg_field) + 1)

#define __gcmALIGN(data, reg_field) \
    (((n2d_uint32_t) (data)) << __gcmSTART(reg_field))

#define __gcmMASK(reg_field) \
    ((n2d_uint32_t) ((__gcmGETSIZE(reg_field) == 32) \
        ?  ~0 \
        : (~(~0 << __gcmGETSIZE(reg_field)))))

#define gcmVERIFYFIELDVALUE(data, reg, field, value) \
( \
    (((n2d_uint32_t) (data)) >> __gcmSTART(reg##_##field) & \
                             __gcmMASK(reg##_##field)) \
        == \
    (reg##_##field##_##value & __gcmMASK(reg##_##field)) \
)

#define gcmSETFIELD(data, reg, field, value) \
( \
    (((n2d_uint32_t) (data)) \
        & ~__gcmALIGN(__gcmMASK(reg##_##field), reg##_##field)) \
        |  __gcmALIGN((n2d_uint32_t) (value) \
            & __gcmMASK(reg##_##field), reg##_##field) \
)

#define gcmGETFIELD(data, reg, field) \
( \
    ((((n2d_uint32_t) (data)) >> __gcmSTART(reg##_##field)) \
        & __gcmMASK(reg##_##field)) \
)

#define gcmSETFIELDVALUE(data, reg, field, value) \
( \
    (((n2d_uint32_t) (data)) \
        & ~__gcmALIGN(__gcmMASK(reg##_##field), reg##_##field)) \
        |  __gcmALIGN(reg##_##field##_##value \
            & __gcmMASK(reg##_##field), reg##_##field) \
)

#define gcmSETMASKEDFIELDVALUE(reg, field, value) \
( \
    gcmSETFIELDVALUE(~0, reg,          field, value) & \
    gcmSETFIELDVALUE(~0, reg, MASK_ ## field, ENABLED) \
)

#define gcmSETMASKEDFIELD(reg, field, value) \
( \
    gcmSETFIELD     (~0, reg,          field, value) & \
    gcmSETFIELDVALUE(~0, reg, MASK_ ## field, ENABLED) \
)

#define gcmALIGN(n, align) \
( \
    ((n) + ((align) - 1)) & ~((align) - 1) \
)

#define gcmMIN(x, y) \
( \
    ((x) <= (y)) \
        ? (x) \
        : (y) \
)

#define gcmMAX(x, y) \
( \
    ((x) >= (y)) \
        ? (x) \
        : (y) \
)

#define gcmINT2PTR(i) \
( \
    (void *)(n2d_uint32_t)(i) \
)

#define gcmPTR2INT(p) \
( \
    (n2d_uint32_t)(p) \
)

#define N2D_IS_SUCCESS(error) (error == N2D_SUCCESS)
#define N2D_IS_ERROR(error)   (error != N2D_SUCCESS)

#define N2D_ON_ERROR(func) \
    do \
    { \
        error = func; \
        if (N2D_IS_ERROR(error)) \
        { \
            goto on_error; \
        } \
    } \
    while (0)

#define gcmASSERT(exp)
#define gcmkASSERT(exp)
#define gcmkTRACE(...)

#ifndef gcmCOUNTOF
#   define gcmCOUNTOF(array) \
        (sizeof(array) / sizeof(array[0]))
#endif

#ifdef __cplusplus
}
#endif

#endif /* _nano2D_types_h__ */
