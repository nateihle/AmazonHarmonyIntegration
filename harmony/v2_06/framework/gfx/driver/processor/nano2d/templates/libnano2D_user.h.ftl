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


#ifndef _nano2D_user_h__
#define _nano2D_user_h__

#ifdef __cplusplus
extern "C" {
#endif

n2d_error_t
n2d_user_os_allocate(
    n2d_uint32_t size,
    void **memory);

n2d_error_t
n2d_user_os_free(
    void *memory);

void
n2d_user_os_memory_fill(
    void *memory,
    n2d_uint8_t filler,
    n2d_uint32_t bytes);

n2d_float_t
n2d_user_os_math_sine(
    n2d_float_t x);

n2d_error_t
n2d_user_os_ioctl(
    n2d_kernel_command_t command,
    void *data);

#ifdef __cplusplus
}
#endif

#endif /* _nano2D_user_h__ */
