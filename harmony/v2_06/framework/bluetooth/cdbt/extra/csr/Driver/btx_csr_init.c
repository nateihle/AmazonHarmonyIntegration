/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2011-2016 SEARAN LLC. All Rights Reserved.
*******************************************************************************/


#include "cdbt/bt/bt_std.h"
#include "cdbt/bt/bt_private.h"
#include "cdbt/bt/bt_hcitr.h"
#include "cdbt/bt/bt_timer.h"
#include "cdbt/hci/hci.h"
#include "cdbt/extra/csr/csr.h"

void _btx_csr_bccmd_init(void);

void btx_csr_init(void)
{
	_btx_csr_bccmd_init();
}
