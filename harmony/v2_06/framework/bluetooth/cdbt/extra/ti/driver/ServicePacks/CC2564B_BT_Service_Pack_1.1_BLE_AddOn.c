#include "cdbt/bt/bt_std.h"
#include "cdbt/bt/bt_private.h"
#include "cdbt/extra/ti/ti.h"

static const btx_ti_script_t script =
{
	NULL,
	0,
	7, 16         // compatible FW version 7016
};

const btx_ti_script_t* btx_ti_get_script__CC2564B_BT_Service_Pack_1_1_BLE_AddOn(void)
{
	return &script;
}
