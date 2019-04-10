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


#ifndef _nano2D_dispatch_h_
#define _nano2D_dispatch_h_

#include "nano2D.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum n2d_kernel_command
{
    N2D_KERNEL_COMMAND_OPEN,
    N2D_KERNEL_COMMAND_CLOSE,
    N2D_KERNEL_COMMAND_ALLOCATE,
    N2D_KERNEL_COMMAND_FREE,
    N2D_KERNEL_COMMAND_MAP,
    N2D_KERNEL_COMMAND_UNMAP,
    N2D_KERNEL_COMMAND_COMMIT,
}
n2d_kernel_command_t;

typedef struct n2d_kernel_command_open
{
    /* [out] */ n2d_uint32_t    chipModel;
    /* [out] */ n2d_uint32_t    chipRevision;
    /* [out] */ n2d_uint32_t    chipFeatures;
    /* [out] */ n2d_uint32_t    chipMinorFeatures;
    /* [out] */ n2d_uint32_t    chipMinorFeatures1;
    /* [out] */ n2d_uint32_t    chipMinorFeatures2;
    /* [out] */ n2d_uint32_t    chipMinorFeatures3;
    /* [out] */ n2d_uint32_t    chipMinorFeatures4;
}
n2d_kernel_command_open_t;

typedef struct n2d_kernel_command_allocate
{
    /* [in] */  n2d_uint32_t    size;
    /* [out] */ void *          memory;
    /* [out] */ n2d_uint32_t    physical;
    /* [out] */ n2d_uint32_t    gpu;
}
n2d_kernel_command_allocate_t;

typedef struct n2d_kernel_command_free
{
    /* [in] */  void *          memory;
    /* [in] */  n2d_uint32_t    physical;
    /* [in] */  n2d_uint32_t    gpu;
}
n2d_kernel_command_free_t;

typedef struct n2d_kernel_command_map
{
    /* [in] */  void *          memory;
    /* [in] */  n2d_uint32_t    physical;
    /* [in] */  n2d_uint32_t    size;
    /* [out] */ void *          handle;
    /* [out] */ n2d_uint32_t    gpu;
}
n2d_kernel_command_map_t;

typedef struct n2d_kernel_command_unmap
{
    /* [in] */  n2d_uint32_t    size;
    /* [in] */  void *          handle;
    /* [in] */  n2d_uint32_t    gpu;
}
n2d_kernel_command_unmap_t;

typedef struct n2d_kernel_command_commit
{
    /* [in] */  void *          memory;
    /* [in] */  n2d_uint32_t    count;
}
n2d_kernel_command_commit_t;

/* This is the function to call from the driver to interface with the gpu. */
n2d_error_t
n2d_kernel_dispatch(
    n2d_kernel_command_t command,
    void *data);

#ifdef __cplusplus
}
#endif

#endif /* _nano2D_dispatch_h_ */
