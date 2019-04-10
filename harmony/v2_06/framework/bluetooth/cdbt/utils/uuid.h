/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2011-2016 SEARAN LLC. All Rights Reserved.
*******************************************************************************/

#ifndef __UUID_H
#define __UUID_H

#ifdef __cplusplus
extern "C" {
#endif

#define BT_DECLARE_UUID(var_name, uuid3, uuid2, uuid1, uuid0)                 bt_uuid_t var_name = { uuid0, uuid1, uuid2, uuid3 }
#define BT_DECLARE_UUID_WITH_UUID32(var_name, uuid)                           bt_uuid_t var_name = { 0x5F9B34FB, 0x80000080, 0x00001000, uuid }
#define BT_DECLARE_UUID_WITH_UUID16(var_name, uuid)                           bt_uuid_t var_name = { 0x5F9B34FB, 0x80000080, 0x00001000, (bt_uuid32)uuid }

#define BT_DECLARE_CONST_UUID(var_name, uuid3, uuid2, uuid1, uuid0)           const bt_uuid_t var_name = { uuid0, uuid1, uuid2, uuid3 }
#define BT_DECLARE_CONST_UUID_WITH_UUID32(var_name, uuid)                     const bt_uuid_t var_name = { 0x5F9B34FB, 0x80000080, 0x00001000, uuid }
#define BT_DECLARE_CONST_UUID_WITH_UUID16(var_name, uuid)                     const bt_uuid_t var_name = { 0x5F9B34FB, 0x80000080, 0x00001000, (bt_uuid32)uuid }

#define BT_DECLARE_CONST_STATIC_UUID(var_name, uuid3, uuid2, uuid1, uuid0)    const static bt_uuid_t var_name = { uuid0, uuid1, uuid2, uuid3 }
#define BT_DECLARE_CONST_STATIC_UUID_WITH_UUID32(var_name, uuid)              const static bt_uuid_t var_name = { 0x5F9B34FB, 0x80000080, 0x00001000, uuid }
#define BT_DECLARE_CONST_STATIC_UUID_WITH_UUID16(var_name, uuid)              const static bt_uuid_t var_name = { 0x5F9B34FB, 0x80000080, 0x00001000, (bt_uuid32)uuid }

extern const bt_uuid_t bt_base_uuid;

void bt_uuid16_to_uuid128(bt_uuid16 uuid16, bt_uuid_t* puuid);
void bt_uuid32_to_uuid128(bt_uuid32 uuid32, bt_uuid_t* puuid);
bt_bool bt_is_uuid_equal(const bt_uuid_t* puuid1, const bt_uuid_t* puuid2);

#ifdef __cplusplus
}
#endif

#endif // __UUID_H
