/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2011-2016 SEARAN LLC. All Rights Reserved.
*
* SEARAN LLC is the exclusive licensee and developer of dotstack with
* all its modifications and enhancements.
*
* Contains proprietary and confidential information of CandleDragon and
* may not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2009, 2010, 2011 CandleDragon. All Rights Reserved.
*******************************************************************************/

#ifndef __SDP_CLIENT_H
#define __SDP_CLIENT_H

#ifdef __cplusplus
extern "C" {
#endif

#define SDP_CLIENT_EVT_NULL                 0
#define SDP_CLIENT_EVT_CONNECTED            1
#define SDP_CLIENT_EVT_DISCONNECTED         2
#define SDP_CLIENT_EVT_CONNECTION_FAILED    3

typedef void (*bt_sdp_service_search_callback_fp)(bt_sr_handle_p shlist, bt_int count, void* param);
typedef bt_bool (*bt_sdp_service_attribute_callback_fp)(bt_uint attrid, bt_sdp_data_element_p pde, void* param);
typedef bt_bool (*bt_sdp_read_de_callback_ftp)(bt_sdp_data_element_p pde, bt_bool seq_start);

typedef void (*bt_sdp_client_callback_fp)(bt_byte evt, void* evt_param, void* param);

typedef struct bt_sdp_client_evt_connected_t
{
	bt_l2cap_channel_t* channel;
} bt_sdp_client_evt_connected_t;

typedef struct bt_sdp_client_evt_disconnected_t
{
	bt_l2cap_channel_t* channel;
} bt_sdp_client_evt_disconnected_t;

/**
 * \brief Search service records
 * \ingroup sdp
 *
 * \details This function locates service records on a remote SDP server that match the given service search pattern.
 *
 * \param channel The L2CAP channel used to communicate to the remote SDP server.
 * \param pattern The service search pattern is a data element sequence where each element
 * in the sequence is a UUID. The sequence must contain at least
 * one UUID. The maximum number of UUIDs in the sequence is 12.
 * The pattern buffer must be valid for the duration of the search operation, i.e. until \c callback is called.
 * To define a data element sequence use the BEGIN_DE_SEQUENCE and END_DE_SEQUENCE macros. 
 * These macros will define a variable whose name is the id of the data element sequence passed to the macros prefixed with "seq_". 
 * A pointer to this variable can be used as the value for the pattern parameter.
 * \param callback The callback function that will be called when search has completed.
 * \param callback_param A pointer to arbitrary data to be passed to the \c callback callback.
 * 
 * \return
 *        \li \c TRUE if the function succeeds.
 *        \li \c FALSE otherwise. The callback function is not called in this case.
 */
bt_bool bt_sdp_request_service_search(bt_l2cap_channel_t* channel, bt_sdp_data_element_p pattern, bt_sdp_service_search_callback_fp callback, void* callback_param);

/**
 * \brief Search attributes
 * \ingroup sdp
 *
 * \details This function retrieves attribute values from a service record.
 *
 * \param channel The L2CAP channel used to communicate to the remote SDP server.
 * \param sr The service record handle specifies the service record from which attribute values are to be retrieved.
 * \param pattern The attribute search pattern is a data element sequence where each element in the list is either an attribute ID or a range of attribute IDs.
 * The pattern buffer must be valid for the duration of the search operation, i.e. until \c callback is called for the first time.
 * To define a data element sequence use the BEGIN_DE_SEQUENCE and END_DE_SEQUENCE macros. 
 * These macros will define a variable whose name is the id of the data element sequence passed to the macros prefixed with "seq_". 
 * A pointer to this variable can be used as the value for the pattern parameter.
 *
 * \param callback The callback function that will be called when search has completed.
 *
 * \param callback_param A pointer to arbitrary data to be passed to the \c callback callback.
 * 
 * \return
 *        \li \c TRUE if the function succeeds.
 *        \li \c FALSE otherwise. The callback function is not called in this case.
 *
 */
bt_bool bt_sdp_request_service_attribute(bt_l2cap_channel_t* channel, bt_sr_handle_t sr, bt_sdp_data_element_p pattern, bt_sdp_service_attribute_callback_fp callback, void* callback_param);

#ifdef __cplusplus
}
#endif

#endif // __SDP_CLIENT_H
