#include "cdbt/bt/bt_std.h"
#include "cdbt/bt/bt_private.h"
#include "cdbt/extra/ti/ti.h"

static const btx_ti_script_t script =
{
	NULL,
	0,
	6, 15         // compatible FW version 6015
};

const btx_ti_script_t* btx_ti_get_script__BL6450L_BT_Service_Pack_2_12_BLE_AddOn(void)
{
	return &script;
}
