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

#ifndef __RFCOMM_H
#define __RFCOMM_H

#include "cdbt/l2cap/l2cap.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \defgroup rfcomm RFCOMM
 */


#include "cdbt/rfcomm/rfcomm_cmd_queue.h"
#include "cdbt/rfcomm/rfcomm_packet.h"

#define RFCOMM_FRAME_TYPE_SABM	0x2f
#define RFCOMM_FRAME_TYPE_UA	0x63
#define RFCOMM_FRAME_TYPE_DM	0x0f
#define RFCOMM_FRAME_TYPE_DISC	0x43
#define RFCOMM_FRAME_TYPE_UIH	0xef
#define RFCOMM_FRAME_TYPE_UI	0x03

#define RFCOMM_FLAG_EA	0x01
#define RFCOMM_FLAG_PF	0x10
#define RFCOMM_FLAG_CR	0x02

#define RFCOMM_ROLE_RESPONDER	0x00
#define RFCOMM_ROLE_INITIATOR	0x02

#define RFCOMM_COMMAND	0x02
#define RFCOMM_RESPONSE	0x00

#define RFCOMM_CTL_MSG_PN		0x80
#define RFCOMM_CTL_MSG_PSC		0x40
#define RFCOMM_CTL_MSG_CLD		0xc0
#define RFCOMM_CTL_MSG_TEST		0x20
#define RFCOMM_CTL_MSG_FCON		0x10
#define RFCOMM_CTL_MSG_FCOFF	0x60
#define RFCOMM_CTL_MSG_MSC		0xe0
#define RFCOMM_CTL_MSG_NSC		0x10
#define RFCOMM_CTL_MSG_RPN		0x90
#define RFCOMM_CTL_MSG_RLS		0x50
#define RFCOMM_CTL_MSG_SNC		0xd0

#define RFCOMM_DLCI_CONTROL		0
#define RFCOMM_DLCI_FREE		0xff

#define RFCOMM_DLC_STATE_CLOSED	0
#define RFCOMM_DLC_STATE_OPEN	1

#define RFCOMM_DLC_CHANGED_CONN_STATE	0
#define RFCOMM_DLC_CHANGED_REMOTE_MSC	1
#define RFCOMM_DLC_CONNECTION_FAILED	2

#define RFCOMM_SESSION_STATE_FREE			0
#define RFCOMM_SESSION_STATE_CONNECTED 		1
#define RFCOMM_SESSION_STATE_DISCONNECTED 	2

#define RFCOMM_SESSION_CHANGED_CONN_STATE	0
#define RFCOMM_SESSION_CHANGED_AFC			1


#define RFCOMM_CMD_STATUS_PENDING			0
#define RFCOMM_CMD_STATUS_WAITING_RESPONSE	1
#define RFCOMM_CMD_STATUS_FC_PENDING		2

#define RFCOMM_FRAME_HEADER_LEN		5
#define RFCOMM_MIX_INFO_LEN			31
#define RFCOMM_MAX_INFO_LEN			_rfcomm_info_len

#define RFCOMM_SERIAL_PORT_CH_1    1
#define RFCOMM_SERIAL_PORT_CH_2    2
#define RFCOMM_SERIAL_PORT_CH_3    3
#define RFCOMM_SERIAL_PORT_CH_4    4
#define RFCOMM_SERIAL_PORT_CH_5    5
#define RFCOMM_SERIAL_PORT_CH_6    6
#define RFCOMM_SERIAL_PORT_CH_7    7
#define RFCOMM_SERIAL_PORT_CH_8    8
#define RFCOMM_SERIAL_PORT_CH_9    9
#define RFCOMM_SERIAL_PORT_CH_10   10
#define RFCOMM_SERIAL_PORT_CH_11   11
#define RFCOMM_SERIAL_PORT_CH_12   12
#define RFCOMM_SERIAL_PORT_CH_13   13
#define RFCOMM_SERIAL_PORT_CH_14   14
#define RFCOMM_SERIAL_PORT_CH_15   15
#define RFCOMM_SERIAL_PORT_CH_16   16
#define RFCOMM_SERIAL_PORT_CH_17   17
#define RFCOMM_SERIAL_PORT_CH_18   18
#define RFCOMM_SERIAL_PORT_CH_19   19
#define RFCOMM_SERIAL_PORT_CH_20   20
#define RFCOMM_SERIAL_PORT_CH_21   21
#define RFCOMM_SERIAL_PORT_CH_22   22
#define RFCOMM_SERIAL_PORT_CH_23   23
#define RFCOMM_SERIAL_PORT_CH_24   24
#define RFCOMM_SERIAL_PORT_CH_25   25
#define RFCOMM_SERIAL_PORT_CH_26   26
#define RFCOMM_SERIAL_PORT_CH_27   27
#define RFCOMM_SERIAL_PORT_CH_28   28
#define RFCOMM_SERIAL_PORT_CH_29   29
#define RFCOMM_SERIAL_PORT_CH_30   30

#define RFCOMM_FC_TYPE_AGREGATE		0
#define RFCOMM_FC_TYPE_CREDIT		1

#define RFCOMM_CFC_MAX_INITIAL_CREDIT   7
#define RFCOMM_CFC_LOCAL_CREDIT	        _rfcomm_local_credit
#define RFCOMM_CFC_ENABLED              (pdlc->psess->fc_type == RFCOMM_FC_TYPE_CREDIT)

#define RFCOMM_TIMEOUT	60

#define RFCOMM_ERR_SUCCESS		0x00
#define RFCOMM_ERR_TIMEOUT		0x01
#define RFCOMM_ERR_DM			0x02
#define RFCOMM_ERR_INTERRUPTED	0x03

#define RFCOMM_RPN_BAUD_RATE_24		0x00
#define RFCOMM_RPN_BAUD_RATE_48		0x01
#define RFCOMM_RPN_BAUD_RATE_72		0x02
#define RFCOMM_RPN_BAUD_RATE_96		0x03 // default
#define RFCOMM_RPN_BAUD_RATE_192	0x04
#define RFCOMM_RPN_BAUD_RATE_384	0x05
#define RFCOMM_RPN_BAUD_RATE_576	0x06
#define RFCOMM_RPN_BAUD_RATE_1152	0x07
#define RFCOMM_RPN_BAUD_RATE_2304	0x08

#define RFCOMM_RPN_DATA_BIT_5		0x00
#define RFCOMM_RPN_DATA_BIT_6		0x01
#define RFCOMM_RPN_DATA_BIT_7		0x02
#define RFCOMM_RPN_DATA_BIT_8		0x03 // default

#define RFCOMM_RPN_STOP_BIT_1		0x00 // default
#define RFCOMM_RPN_STOP_BIT_1_5		0x04

#define RFCOMM_RPN_PARITY_N			0x00 // default
#define RFCOMM_RPN_PARITY_Y			0x08

#define RFCOMM_RPN_PARITY_ODD		0x00
#define RFCOMM_RPN_PARITY_EVEN		0x10
#define RFCOMM_RPN_PARITY_MARK		0x20
#define RFCOMM_RPN_PARITY_SPACE		0x30

#define RFCOMM_RPN_FLC_N				0x00
#define RFCOMM_RPN_FLC_XONOFF_INPUT		0x01
#define RFCOMM_RPN_FLC_XONOFF_OUTPUT	0x02
#define RFCOMM_RPN_FLC_RTR_INPUT		0x04
#define RFCOMM_RPN_FLC_RTR_OUTPUT		0x08
#define RFCOMM_RPN_FLC_RTC_INPUT		0x10
#define RFCOMM_RPN_FLC_RTC_OUTPUT		0x20

#define RFCOMM_RPN_XON_DEFAULT		0x11
#define RFCOMM_RPN_XOFF_DEFAULT		0x13

#define RFCOMM_MX_MSG_MAX_DATA_LEN  10

//#define RFCOMM_MS_RTC

#define MK_CMD_ADDRESS(addr, pdlc)	((addr & 0xfc) | pdlc->psess->role | RFCOMM_FLAG_EA)
#define MK_DLCI(bt_server_channel, rfcomm_session)			(bt_server_channel << 3 | ((~rfcomm_session->role & 2) << 1))


struct _bt_rfcomm_command_t;
struct _bt_rfcomm_dlc_t;

typedef void (*bt_rfcomm_cmd_callback_fp)(struct _bt_rfcomm_dlc_t* dlc, struct _bt_rfcomm_command_t *cmd, bt_int status);
typedef void (*bt_rfcomm_send_data_callback_fp)(struct _bt_rfcomm_dlc_t* dlc, bt_byte_p data, bt_int len, bt_int status);

typedef struct _bt_rfcomm_command_t
{
	struct _bt_rfcomm_command_t* next_cmd;
	bt_byte address;
	bt_byte control;
	bt_byte credit;
	bt_byte_p pdata;
	bt_int len;
	bt_byte status;
	bt_rfcomm_cmd_callback_fp cb;
	bt_rfcomm_send_data_callback_fp send_data_cb;
	bt_long send_time;
	bt_byte mx_params[RFCOMM_MX_MSG_MAX_DATA_LEN];

} bt_rfcomm_command_t, *bt_rfcomm_command_p;

struct _bt_rfcomm_session_t;

struct _bt_rfcomm_dlc_t;
typedef void (*bt_rfcomm_read_data_callback_fp)(struct _bt_rfcomm_dlc_t* dlc, bt_byte_p data, bt_int len);
typedef void (*bt_rfcomm_dlc_state_callback_fp)(struct _bt_rfcomm_dlc_t* dlc, bt_int what, void* param);

typedef struct _bt_rfcomm_dlc_t
{
	bt_byte id;
	bt_byte priority;
	bt_uint max_frame_size;
	bt_byte cfc_remote_credit;
	bt_byte cfc_freed_local_buffers;
	bt_byte state;
	bt_byte local_ms;
	bt_byte remote_ms;
	bt_byte local_ls;
	bt_byte remote_ls;
	bt_bool buffered;

	bt_rfcomm_read_data_callback_fp read_data_cb;
	bt_rfcomm_dlc_state_callback_fp state_cb;
	void* cb_param;

	bt_rfcomm_packet_t tx_packet;

	struct _bt_rfcomm_session_t *psess;
	bt_queue_element_t* cmd_queue;

} bt_rfcomm_dlc_t, *bt_rfcomm_dlc_p;

typedef struct _bt_rfcomm_ctl_msg_t {
	bt_byte type;
	bt_int len;
	bt_byte_p pdata;
} bt_rfcomm_ctl_msg_t, *bt_rfcomm_ctl_msg_p;

struct _bt_rfcomm_session_t;
typedef void (*bt_rfcomm_state_callback_fp)(struct _bt_rfcomm_session_t* session, bt_int what, void* param);

typedef struct _bt_rfcomm_session_t
{
	bt_byte state;
	bt_l2cap_channel_t* pch;
	bt_byte role;
	bt_byte fc_type;
	bt_bool afc_off;
	bt_rfcomm_dlc_t* dlcs;
	bt_rfcomm_dlc_state_callback_fp connect_cb;
	void* connect_param;
	bt_byte connect_server_channel;
	bt_rfcomm_command_p pcmd_cur;
	bt_l2cap_mgr_p l2cap_mgr;

} bt_rfcomm_session_t, *bt_rfcomm_session_p;



/**
 * \brief Initialize the RFCOMM layer.
 * \ingroup rfcomm
 *
 * \details This function initializes the RFCOMM layer of the stack. It must be called prior to any other
 * RFCOMM function can be called.
 */
bt_bool bt_rfcomm_init(void);


/**
 * \brief Listen for incoming connections.
 * \ingroup rfcomm
 *
 * \details This function enables incoming connections on the specified RFCOMM session. Changes in
 * the session state are reported through a callback function. 
 *
 * \param server_channel A server channel on which the RFCOMM session will listen and accept incoming connections.
 * \param callback The callback function that is called when session state changes.
 * \param param An arbitrary data pointer that will be passed to the callback function specified by
 *              the \c callback parameter.
 * 
 * \return
 *        \li \c TRUE if the function succeeds.
 *        \li \c FALSE otherwise. The callback function is not called in this case. 
 */
bt_bool bt_rfcomm_listen(bt_byte server_channel, bt_uint acl_config, bt_rfcomm_dlc_state_callback_fp callback, void* param);

bt_bool bt_rfcomm_cancel_listen(bt_byte server_channel);

/**
 * \brief Connect to a remote device.
 * \ingroup rfcomm
 *
 * \details This function establishes an RFCOMM connection with a remote device and opens a data DLC.
 * Changes in the session state are reported through a callback function specified when the session has been allocated via call to bt_rfcomm_allocate_session.
 * Changes in the data DLC are reported through a callback function specified in this call.
 *
 * \param remote_addr Address of the remote device.
 * \param server_channel A server channel of the remote RFCOMM server.
 * \param callback The callback function for reporting changes in DLC state opened by this call.
 * \param param An arbitrary data pointer that will be passed to the callback function specified by
 *              the \c callback parameter.
 * 
 * \return
 *        \li \c TRUE if the function succeeds.
 *        \li \c FALSE otherwise. The callback function is not called in this case. 
 */
bt_bool bt_rfcomm_connect(
	bt_bdaddr_p remote_addr, bt_byte server_channel, 
	bt_uint acl_config, bt_rfcomm_dlc_state_callback_fp callback, void* param);

/**
 * \brief Open DLC.
 * \ingroup rfcomm
 *
 * \details This function opens the specified DLC. Before calling this function the RFCOMM session
 * must be already open. This function is not to be used with DLCI = 0. Changes in DLC state are
 * reported through a callback function.
 *
 * \param dlc The DLC to open.
 * \param callback The callback function for reporting changes in DLC state.
 * \param param An arbitrary data pointer that will be passed to the callback function specified by
 *              the \c callback parameter.
 *
 * \return
 *        \li \c TRUE if the function succeeds.
 *        \li \c FALSE otherwise. The callback function is not called in this case. 
 */
bt_bool bt_rfcomm_open_dlc(bt_rfcomm_dlc_t* dlc, bt_rfcomm_dlc_state_callback_fp callback, void* param);


/**
 * \brief Close DLC.
 * \ingroup rfcomm
 *
 * \details This function closes a DLC. If DLCI = 0, the parent RFCOMM session is also closed.
 *
 */
void bt_rfcomm_close_dlc(bt_rfcomm_dlc_t* dlc);


/**
 * \brief Allocate RFCOMM session.
 * \ingroup rfcomm
 *
 * \details This function allocates a new RFCOMM session.
 *
 * \param l2cap_mgr The L2CAP manager on which the RFCOMM session is to be created.
 * \param callback The callback function that is called when session state changes.
 * \param param An arbitrary data pointer that will be passed to the callback function specified by
 *              the \c callback parameter.
 *
 * \return
 *         \li A pointer to the new RFCOMM session structure if the function succeeds.
 *         \li \c NULL otherwise.
 *
 */
bt_rfcomm_session_t* bt_rfcomm_allocate_session(bt_l2cap_mgr_t* l2cap_mgr);


/**
 * \brief Release RFCOMM session.
 * \ingroup rfcomm
 *
 * \details This function deallocates the specified RFCOMM session.
 * This function does not disconnect the session. It just frees the memory used by the bt_rfcomm_session structure.
 * The session has to be disconnected by calling bt_rfcomm_close_dlc with DLCI = 0 first.
 *
 * \param session The RFCOMM session to be deallocated.
 *
 */
void bt_rfcomm_free_session(bt_rfcomm_session_t* session);


/**
 * \brief Allocate DLC.
 * \ingroup rfcomm
 *
 * \details This function allocates a new DLC on the specified RFCOMM session.
 *
 * \param session The RFCOMM session.
 * \param dlci DLCI of the new DLC.
 *
 * \return
 *         \li A pointer to the new DLC if the function succeeds.
 *         \li \c NULL otherwise.
 */
bt_rfcomm_dlc_t* bt_rfcomm_allocate_dlc(bt_rfcomm_session_t* session, bt_byte dlci);


/**
 * \brief Release DLC.
 * \ingroup rfcomm
 *
 * \details This function releases the specified DLC.
 *
 * \param dlc The DLC to be released.
 *
 */
void bt_rfcomm_free_dlc(bt_rfcomm_dlc_t* dlc);


/**
* \brief Find DLC
*
*/
bt_rfcomm_dlc_t* bt_rfcomm_find_dlc(bt_rfcomm_session_t* session, bt_byte address);


/**
 * \brief Send data over a DLC.
 * \ingroup rfcomm
 *
 * \details This function sends data over the specified DLC. Operation completion is reported
 * through callback function.
 *
 * \param dlc The DLC.
 * \param data A pointer to the data to be sent.
 * \param len Data length.
 * \param callback The callback function that is called when operation completes.
 *
 * \return
 *        \li \c TRUE if the function succeeds.
 *        \li \c FALSE otherwise. The callback function is not called in this case. 
 */
bt_bool bt_rfcomm_send_data(bt_rfcomm_dlc_t* dlc, void *data, bt_int len, bt_rfcomm_send_data_callback_fp callback);


bt_bool bt_rfcomm_send_credit(bt_rfcomm_dlc_t* dlc, bt_byte credit);


bt_int bt_rfcomm_get_frame_length(bt_rfcomm_dlc_t* dlc);

#include "cdbt/rfcomm/rfcomm_mgr.h"
#include "cdbt/rfcomm/rfcomm_mx.h"
#include "cdbt/rfcomm/rfcomm_private.h"

#ifdef __cplusplus
}
#endif

#endif // __RFCOMM_H
