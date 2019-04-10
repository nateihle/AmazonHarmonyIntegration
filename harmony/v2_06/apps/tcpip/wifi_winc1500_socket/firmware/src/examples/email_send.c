/*******************************************************************************
  File Name:
    email_send.c

  Summary:
    WINC1500 Email Sending Example

  Description:
    This example demonstrates the use of the WINC1500 to send email using Gmail
    SMTP server.

    For using Gmail, the root certificate must be installed.
    Download the root certificate using the root_certificate_downloader from the
    firmware updater.

    The configuration options for this example are:
        WLAN_SSID           -- AP to connect to
        WLAN_AUTH           -- Security for the AP
        WLAN_PSK            -- Passphrase for WPA security
        WLAN_WEP_KEY        -- Key for WEP security
        WLAN_WEP_KEY_INDEX  -- Key index for WEP security
        FROM_ADDRESS        -- Sender's Gmail account
        FROM_PASSWORD       -- Sender's Gmail account's password
        TO_ADDRESS          -- Receiver's email address
 *******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/

#include "app.h"
#include "m2m_wifi.h"
#include "socket.h"

#if EMAIL_SEND_EXAMPLE

#define WLAN_SSID           "DEMO_AP" /* Target AP */
#define WLAN_AUTH           M2M_WIFI_SEC_WPA_PSK /* AP Security, M2M_WIFI_SEC_WPA_PSK, M2M_WIFI_SEC_WEP or M2M_WIFI_SEC_OPEN */
#define WLAN_PSK            "12345678" /* Passphrase for WPA Security */
#define WLAN_WEP_KEY        "1234567890" /* Key for WEP Security */
#define WLAN_WEP_KEY_INDEX  1 /* Key Index for WEP Security */

/** SMTP Configuration Definitions */
#define FROM_ADDRESS        "sender@gmail.com" /* Set FROM Email Address */
#define FROM_PASSWORD       "12345678" /* Set Sender Email Password */
#define TO_ADDRESS          "recipient@gmail.com" /* Set TO Email Address */

#define SMTP_BUF_LEN        1024
#define GMAIL_HOST_NAME     "smtp.gmail.com"
#define GMAIL_HOST_PORT     465
#define SENDER_RFC          "<sender@gmail.com>" /* Set Sender Email Address */
#define RECIPIENT_RFC       "<recipient@gmail.com>" /* Set Recipient Email Address */
#define EMAIL_SUBJECT       "Hello from WINC1500!"
#define EMAIL_MSG           "This mail is sent from Email Send Example."
#define WAITING_TIME        30000
#define RETRY_COUNT         3

#define EXAMPLE_HEADER \
"\r\n===========================\r\n"\
    "WINC1500 Email Send Example\r\n"\
    "===========================\r\n"

#define IPV4_BYTE(val, index)       ((val >> (index * 8)) & 0xFF)

#define app_state_get() s_app_state
#define app_state_set(x) do {s_app_state = x;} while (0)
#define is_link_up() s_connected

typedef enum {
	APP_RADIO_INIT,
	APP_WIFI_OPEN,
	APP_WIFI_CONNECT_WAIT,
	APP_NET_UP,
	APP_END,
	APP_PARK
} APP_STATE;

typedef enum {
	SOCKET_INIT = 0,
	SOCKET_CONNECT,
	SOCKET_WAITING,
	SOCKET_COMPLETE,
	SOCKET_ERROR
} SOCKET_STATUS;

typedef enum {
	SMTP_INACTIVE = 0,
	SMTP_INIT,
	SMTP_HELO,
	SMTP_AUTH,
	SMTP_AUTH_USERNAME,
	SMTP_AUTH_PASSWORD,
	SMTP_FROM,
	SMTP_RCPT,
	SMTP_DATA,
	SMTP_MESSAGE_SUBJECT,
	SMTP_MESSAGE_TO,
	SMTP_MESSAGE_FROM,
	SMTP_MESSAGE_CRLF,
	SMTP_MESSAGE_BODY,
	SMTP_MESSAGE_DATAEND,
	SMTP_QUIT,
	SMTP_ERROR
} SMTP_STATUS;

typedef enum {
	EMAIL_ERROR_FAILED = -1,
	EMAIL_ERROR_NONE = 0,
	EMAIL_ERROR_INIT,
	EMAIL_ERROR_HELO,
	EMAIL_ERROR_AUTH,
	EMAIL_ERROR_AUTH_USERNAME,
	EMAIL_ERROR_AUTH_PASSWORD,
	EMAIL_ERROR_FROM,
	EMAIL_ERROR_RCPT,
	EMAIL_ERROR_DATA,
	EMAIL_ERROR_MESSAGE,
	EMAIL_ERROR_QUIT
} EMAIL_ERROR;

/** Return Codes */
static const char sc_smtp_code_ready[] = {'2', '2', '0', '\0'};
static const char sc_smtp_code_ok_reply[] = {'2', '5', '0', '\0'};
static const char sc_smtp_code_intermediate_reply[] = {'3', '5', '4', '\0'};
static const char sc_smtp_code_auth_reply[] = {'3', '3', '4', '\0'};
static const char sc_smtp_code_auth_success[] = {'2', '3', '5', '\0'};

/** Send Codes */
static const char sc_smtp_helo[] = {'H', 'E', 'L', 'O', '\0'};
static const char sc_smtp_mail_from[] = {'M', 'A', 'I', 'L', ' ', 'F', 'R', 'O', 'M', ':', ' ', '\0'};
static const char sc_smtp_rcpt[] = {'R', 'C', 'P', 'T', ' ', 'T', 'O', ':', ' ', '\0'};
static const char sc_smtp_data[] = "DATA";
static const char sc_smtp_cr_lf[] = "\r\n";
static const char sc_smtp_subject[] = "Subject: ";
static const char sc_smtp_to[] = "To: ";
static const char sc_smtp_from[] = "From: ";
static const char sc_smtp_data_end[] = {'\r', '\n', '.', '\r', '\n', '\0'};
static const char sc_smtp_quit[] = {'Q', 'U', 'I', 'T', '\r', '\n', '\0'};

static APP_STATE s_app_state;
static bool s_connected;
static bool s_host_ip_by_name; /* Get host IP status variable. */
static SOCKET s_tcp_client_sock; /* TCP client socket handler. */
static uint8_t s_socket_status;
static uint32_t s_host_ip; /* IP address of host. */
static uint8_t s_smtp_status; /* SMTP information. */
static int8_t s_email_error; /* SMTP email error information. */
static char s_send_recv_buf[SMTP_BUF_LEN]; /** Send and receive buffer definition. */
static char s_handler_buf[SMTP_BUF_LEN]; /* Handler buffer definition. */
static char s_username_basekey[128]; /* Username basekey definition. */
static char s_password_basekey[128]; /* Password basekey definition. */
static uint8_t s_retry_cnt; /* Retry count. */

extern void ConvertToBase64(char *pcOutStr, const char *pccInStr, int iLen);

/*
 * Generates Base64 Key needed for authentication.
 *
 * input: string to be converted to base64.
 * basekey: the base64 converted output.
 */
static void base64key_generate(char *input, char *basekey)
{
	/*
	 * In case the input string needs to be modified before conversion, define
	 * new string to pass-through.
	 */
	int16_t inputLen = strlen(input);
	char inputStr[128];

	memcpy(inputStr, input, inputLen);
	ConvertToBase64(basekey, (void *)inputStr, inputLen);
}

/*
 * Callback to get the Wi-Fi status update.
 *
 * u8MsgType: type of Wi-Fi notification.
 * pvMsg: pointer to a buffer containing the notification parameters.
 */
static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
	switch (u8MsgType) {
	case M2M_WIFI_RESP_CON_STATE_CHANGED:
	{
		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
		if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
			SYS_CONSOLE_PRINT("wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: CONNECTED\r\n");
		} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
			int8_t connect_ret;
			s_connected = false;
			s_host_ip_by_name = false;
			SYS_CONSOLE_PRINT("wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: DISCONNECTED\r\n");
			if (WLAN_AUTH == M2M_WIFI_SEC_WEP) {
				tstrM2mWifiWepParams wep_params;
				wep_params.u8KeyIndx = WLAN_WEP_KEY_INDEX;
				wep_params.u8KeySz = sizeof(WLAN_WEP_KEY);
				memcpy(wep_params.au8WepKey, WLAN_WEP_KEY, sizeof(WLAN_WEP_KEY));
				connect_ret = m2m_wifi_connect((char *)WLAN_SSID, sizeof(WLAN_SSID),
					WLAN_AUTH, (void *)&wep_params, M2M_WIFI_CH_ALL);
			} else {
				connect_ret = m2m_wifi_connect((char *)WLAN_SSID, sizeof(WLAN_SSID),
					WLAN_AUTH, (void *)WLAN_PSK, M2M_WIFI_CH_ALL);
			}

			if (connect_ret != M2M_SUCCESS)
				SYS_CONSOLE_PRINT("wifi_cb: m2m_wifi_connect call error!(%d)\r\n", connect_ret);
		}
	}
		break;
	case M2M_WIFI_REQ_DHCP_CONF:
	{
		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
		int8_t gethostbyname_ret;
		s_connected = true;
		SYS_CONSOLE_PRINT("wifi_cb: M2M_WIFI_REQ_DHCP_CONF: IP is %u.%u.%u.%u\r\n",
			pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);

		/* Obtain the IP Address by network name. */
		gethostbyname_ret = gethostbyname((uint8_t *)GMAIL_HOST_NAME);
		if (gethostbyname_ret != M2M_SUCCESS)
			SYS_CONSOLE_PRINT("wifi_cb: gethostbyname call error!(%d)\r\n", gethostbyname_ret);
	}
		break;
	default:
		break;
	}
}

/*
 * Callback function of IP address.
 *
 * hostName: domain name.
 * hostIp: server IP.
 */
static void resolve_cb(uint8_t *hostName, uint32_t hostIp)
{
	s_host_ip = hostIp;
	s_host_ip_by_name = true;
	SYS_CONSOLE_PRINT("resolve_cb: Host IP is %d.%d.%d.%d\r\n", (int)IPV4_BYTE(hostIp, 0), (int)IPV4_BYTE(hostIp, 1),
			(int)IPV4_BYTE(hostIp, 2), (int)IPV4_BYTE(hostIp, 3));
	SYS_CONSOLE_PRINT("resolve_cb: Host Name is %s\r\n", hostName);
}

/*
 * Callback function of TCP client socket.
 *
 * sock: socket handler.
 * u8Msg: type of socket notification.
 * pvMsg: structure contains notification informations.
 */
static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
	/* Check for socket event on TCP socket. */
	if (sock == s_tcp_client_sock) {
		switch (u8Msg) {
		case SOCKET_MSG_CONNECT:
		{
			tstrSocketConnectMsg *pstrConnect = (tstrSocketConnectMsg *)pvMsg;
			if (pstrConnect && pstrConnect->s8Error >= SOCK_ERR_NO_ERROR) {
				memset(s_handler_buf, 0, SMTP_BUF_LEN);
				recv(s_tcp_client_sock, s_handler_buf, sizeof(s_handler_buf), 0);
			} else {
				s_socket_status = SOCKET_ERROR;
				SYS_CONSOLE_PRINT("socket_cb: SOCKET_MSG_CONNECT: CONNECTION ERROR!(%d)\r\n", pstrConnect->s8Error);
			}
		}
			break;
		case SOCKET_MSG_SEND:
			switch (s_smtp_status) {
			case SMTP_MESSAGE_SUBJECT:
				s_socket_status = SOCKET_CONNECT;
				s_smtp_status = SMTP_MESSAGE_TO;
				break;
			case SMTP_MESSAGE_TO:
				s_socket_status = SOCKET_CONNECT;
				s_smtp_status = SMTP_MESSAGE_FROM;
				break;
			case SMTP_MESSAGE_FROM:
				s_socket_status = SOCKET_CONNECT;
				s_smtp_status = SMTP_MESSAGE_CRLF;
				break;
			case SMTP_MESSAGE_CRLF:
				s_socket_status = SOCKET_CONNECT;
				s_smtp_status = SMTP_MESSAGE_BODY;
				break;
			case SMTP_MESSAGE_BODY:
				s_socket_status = SOCKET_CONNECT;
				s_smtp_status = SMTP_MESSAGE_DATAEND;
				break;
			case SMTP_QUIT:
				s_socket_status = SOCKET_COMPLETE;
				s_smtp_status = SMTP_INIT;
				break;
			default:
				break;
			}
			break;
		case SOCKET_MSG_RECV:
		{
			tstrSocketRecvMsg *pstrRecv = (tstrSocketRecvMsg *)pvMsg;

			if (s_socket_status == SOCKET_WAITING) {
				s_socket_status = SOCKET_CONNECT;
				switch (s_smtp_status) {
				case SMTP_INIT:
					if (pstrRecv && pstrRecv->s16BufferSize > 0) {
						/* If buffer has 220 'OK' from server, set state to HELO. */
						if (pstrRecv->pu8Buffer[0] == sc_smtp_code_ready[0] &&
								pstrRecv->pu8Buffer[1] == sc_smtp_code_ready[1] &&
								pstrRecv->pu8Buffer[2] == sc_smtp_code_ready[2]) {
							s_smtp_status = SMTP_HELO;
						} else {
							s_smtp_status = SMTP_ERROR;
							s_email_error = EMAIL_ERROR_INIT;
							SYS_CONSOLE_PRINT("socket_cb: SMTP_INIT: No response\r\n");
						}
					} else {
						s_smtp_status = SMTP_ERROR;
						s_email_error = EMAIL_ERROR_INIT;
						SYS_CONSOLE_PRINT("socket_cb: SMTP_INIT: recv error!\r\n");
					}
					break;
				case SMTP_HELO:
					if (pstrRecv && pstrRecv->s16BufferSize > 0) {
						/* If buffer has 250, set state to AUTH. */
						if (pstrRecv->pu8Buffer[0] == sc_smtp_code_ok_reply[0] &&
								pstrRecv->pu8Buffer[1] == sc_smtp_code_ok_reply[1] &&
								pstrRecv->pu8Buffer[2] == sc_smtp_code_ok_reply[2]) {
							s_smtp_status = SMTP_AUTH;
						} else {
							s_smtp_status = SMTP_ERROR;
							s_email_error = EMAIL_ERROR_HELO;
							SYS_CONSOLE_PRINT("socket_cb: SMTP_HELO: No response\r\n");
						}
					} else {
						s_smtp_status = SMTP_ERROR;
						s_email_error = EMAIL_ERROR_HELO;
						SYS_CONSOLE_PRINT("socket_cb: SMTP_HELO: recv error!\r\n");
					}
					break;
				case SMTP_AUTH:
					if (pstrRecv && pstrRecv->s16BufferSize > 0) {
						/* Function handles authentication for all services. */
						base64key_generate((char *)FROM_ADDRESS, s_username_basekey);

						/* If buffer is 334, give username in base64. */
						if (pstrRecv->pu8Buffer[0] == sc_smtp_code_auth_reply[0] &&
								pstrRecv->pu8Buffer[1] == sc_smtp_code_auth_reply[1] &&
								pstrRecv->pu8Buffer[2] == sc_smtp_code_auth_reply[2]) {
							s_smtp_status = SMTP_AUTH_USERNAME;
						} else {
							s_smtp_status = SMTP_ERROR;
							s_email_error = EMAIL_ERROR_AUTH;
							SYS_CONSOLE_PRINT("socket_cb: SMTP_AUTH: No response\r\n");
						}
					} else {
						s_smtp_status = SMTP_ERROR;
						s_email_error = EMAIL_ERROR_AUTH;
						SYS_CONSOLE_PRINT("socket_cb: SMTP_AUTH: recv error!\r\n");
					}
					break;
				case SMTP_AUTH_USERNAME:
					if (pstrRecv && pstrRecv->s16BufferSize > 0) {
						base64key_generate((char *)FROM_PASSWORD, s_password_basekey);

						/* If buffer is 334, give password in base64. */
						if (pstrRecv->pu8Buffer[0] == sc_smtp_code_auth_reply[0] &&
								pstrRecv->pu8Buffer[1] == sc_smtp_code_auth_reply[1] &&
								pstrRecv->pu8Buffer[2] == sc_smtp_code_auth_reply[2]) {
							s_smtp_status = SMTP_AUTH_PASSWORD;
						} else {
							s_smtp_status = SMTP_ERROR;
							s_email_error = EMAIL_ERROR_AUTH_USERNAME;
							SYS_CONSOLE_PRINT("socket_cb: SMTP_AUTH_USERNAME: No response\r\n");
						}
					} else {
						s_smtp_status = SMTP_ERROR;
						s_email_error = EMAIL_ERROR_AUTH_USERNAME;
						SYS_CONSOLE_PRINT("socket_cb: SMTP_AUTH_USERNAME: recv error!\r\n");
					}
					break;
				case SMTP_AUTH_PASSWORD:
					if (pstrRecv && pstrRecv->s16BufferSize > 0) {
						if (pstrRecv->pu8Buffer[0] == sc_smtp_code_auth_success[0] &&
								pstrRecv->pu8Buffer[1] == sc_smtp_code_auth_success[1] &&
								pstrRecv->pu8Buffer[2] == sc_smtp_code_auth_success[2]) {
							/* Authentication was successful, set state to FROM. */
							s_smtp_status = SMTP_FROM;
						} else {
							s_smtp_status = SMTP_ERROR;
							s_email_error = EMAIL_ERROR_AUTH_PASSWORD;
							SYS_CONSOLE_PRINT("socket_cb: SMTP_AUTH_PASSWORD: No response\r\n");
						}
					} else {
						s_smtp_status = SMTP_ERROR;
						s_email_error = EMAIL_ERROR_AUTH_PASSWORD;
						SYS_CONSOLE_PRINT("socket_cb: SMTP_AUTH_PASSWORD: recv error!\r\n");
					}
					break;
				case SMTP_FROM:
					if (pstrRecv && pstrRecv->s16BufferSize > 0) {
						/* If buffer has 250, set state to RCPT. */
						if (pstrRecv->pu8Buffer[0] == sc_smtp_code_ok_reply[0] &&
								pstrRecv->pu8Buffer[1] == sc_smtp_code_ok_reply[1] &&
								pstrRecv->pu8Buffer[2] == sc_smtp_code_ok_reply[2]) {
							s_smtp_status = SMTP_RCPT;
						} else {
							s_smtp_status = SMTP_ERROR;
							s_email_error = EMAIL_ERROR_FROM;
							SYS_CONSOLE_PRINT("socket_cb: SMTP_FROM: No response\r\n");
						}
					} else {
						s_smtp_status = SMTP_ERROR;
						s_email_error = EMAIL_ERROR_FROM;
						SYS_CONSOLE_PRINT("socket_cb: SMTP_FROM: recv error!\r\n");
					}
					break;
				case SMTP_RCPT:
					if (pstrRecv && pstrRecv->s16BufferSize > 0) {
						/* If buffer has 250, set state to DATA. */
						if (pstrRecv->pu8Buffer[0] == sc_smtp_code_ok_reply[0] &&
								pstrRecv->pu8Buffer[1] == sc_smtp_code_ok_reply[1] &&
								pstrRecv->pu8Buffer[2] == sc_smtp_code_ok_reply[2]) {
							s_smtp_status = SMTP_DATA;
						} else {
							s_smtp_status = SMTP_ERROR;
							s_email_error = EMAIL_ERROR_RCPT;
							SYS_CONSOLE_PRINT("socket_cb: SMTP_RCPT: No response\r\n");
						}
					} else {
						s_smtp_status = SMTP_ERROR;
						s_email_error = EMAIL_ERROR_RCPT;
						SYS_CONSOLE_PRINT("socket_cb: SMTP_RCPT: recv error!\r\n");
					}
					break;
				case SMTP_DATA:
					if (pstrRecv && pstrRecv->s16BufferSize > 0) {
						/* If buffer has 354, set state to MESSAGE_SUBJECT. */
						if (pstrRecv->pu8Buffer[0] == sc_smtp_code_intermediate_reply[0] &&
								pstrRecv->pu8Buffer[1] == sc_smtp_code_intermediate_reply[1] &&
								pstrRecv->pu8Buffer[2] == sc_smtp_code_intermediate_reply[2]) {
							s_smtp_status = SMTP_MESSAGE_SUBJECT;
						} else {
							s_smtp_status = SMTP_ERROR;
							s_email_error = EMAIL_ERROR_DATA;
							SYS_CONSOLE_PRINT("No response for data transmission.\r\n");
						}
					} else {
						s_smtp_status = SMTP_ERROR;
						s_email_error = EMAIL_ERROR_DATA;
						SYS_CONSOLE_PRINT("SMTP_DATA: recv error!\r\n");
					}

					break;

				case SMTP_MESSAGE_DATAEND:
					if (pstrRecv && pstrRecv->s16BufferSize > 0) {
						/* If buffer has 250, set state to QUIT. */
						if (pstrRecv->pu8Buffer[0] == sc_smtp_code_ok_reply[0] &&
								pstrRecv->pu8Buffer[1] == sc_smtp_code_ok_reply[1] &&
								pstrRecv->pu8Buffer[2] == sc_smtp_code_ok_reply[2]) {
							s_smtp_status = SMTP_QUIT;
						} else {
							s_smtp_status = SMTP_ERROR;
							s_email_error = EMAIL_ERROR_MESSAGE;
							SYS_CONSOLE_PRINT("socket_cb: SMTP_MESSAGE_DATAEND: No response\r\n");
						}
					} else {
						s_smtp_status = SMTP_ERROR;
						s_email_error = EMAIL_ERROR_MESSAGE;
						SYS_CONSOLE_PRINT("socket_cb: SMTP_MESSAGE_DATAEND: recv error!\r\n");
					}
					break;
				default:
					break;
				}
			}
		}
			break;
		default:
			break;
		}
	}
}

static int8_t wifi_open(void)
{
	tstrWifiInitParam param;
	int8_t ret;

	/* Initialize Wi-Fi parameters structure. */
	memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

	/* Initialize Wi-Fi driver with data and status callbacks. */
	param.pfAppWifiCb = wifi_cb;
	ret = m2m_wifi_init(&param);
	if (ret != M2M_SUCCESS) {
		SYS_CONSOLE_PRINT("m2m_wifi_init call error!(%d)\r\n", ret);
		return ret;
	}

	/* Initialize socket module. */
	socketInit();
	registerSocketCallback(socket_cb, resolve_cb);

	/* Connect to AP. */
	if (WLAN_AUTH == M2M_WIFI_SEC_WEP) {
		tstrM2mWifiWepParams wep_params;
		wep_params.u8KeyIndx = WLAN_WEP_KEY_INDEX;
		wep_params.u8KeySz = sizeof(WLAN_WEP_KEY);
		memcpy(wep_params.au8WepKey, WLAN_WEP_KEY, sizeof(WLAN_WEP_KEY));
		ret = m2m_wifi_connect((char *)WLAN_SSID, sizeof(WLAN_SSID),
			WLAN_AUTH, (void *)&wep_params, M2M_WIFI_CH_ALL);
	} else {
		ret = m2m_wifi_connect((char *)WLAN_SSID, sizeof(WLAN_SSID),
			WLAN_AUTH, (char *)WLAN_PSK, M2M_WIFI_CH_ALL);
	}

	if (ret != M2M_SUCCESS) {
		SYS_CONSOLE_PRINT("m2m_wifi_connect call error!(%d)\r\n", ret);
	}

	return ret;
}

/*
 * Creates and connects to a secure socket to be used for SMTP client.
 *
 * Return:
 *   SOCK_ERR_NO_ERROR if success,
 *   -1 if socket create error,
 *   SOCK_ERR_INVALID if socket connect error.
 */
static int8_t smtp_connect(void)
{
	struct sockaddr_in addr_in;

	addr_in.sin_family = AF_INET;
	addr_in.sin_port = _htons(GMAIL_HOST_PORT);
	addr_in.sin_addr.s_addr = s_host_ip;

	/* Create secure socket. */
	if (s_tcp_client_sock < 0) {
		s_tcp_client_sock = socket(AF_INET, SOCK_STREAM, SOCKET_FLAGS_SSL);
	}

	/* Check if socket was created successfully. */
	if (s_tcp_client_sock == -1) {
		SYS_CONSOLE_PRINT("socket error.\r\n");
		close(s_tcp_client_sock);
		return -1;
	}

	/* If success, connect to socket. */
	if (connect(s_tcp_client_sock, (struct sockaddr *)&addr_in, sizeof(struct sockaddr_in)) != SOCK_ERR_NO_ERROR) {
		SYS_CONSOLE_PRINT("connect error.\r\n");
		return SOCK_ERR_INVALID;
	}

	/* Success. */
	return SOCK_ERR_NO_ERROR;
}

/*
 * Sends an SMTP command and provides the server response.
 *
 * socket: the socket descriptor to be used for sending.
 * cmd: string of the command.
 * cmdpara: command parameter.
 * respBuf: pointer to the SMTP response from the server.
 */
static void smtp_recv_send(long socket, char *cmd, char *cmdparam, char *respBuf)
{
	uint16_t sendLen = 0;
	memset(s_send_recv_buf, 0, sizeof(s_send_recv_buf));

	if (cmd != NULL) {
		sendLen = strlen(cmd);
		memcpy(s_send_recv_buf, cmd, strlen(cmd));
	}

	if (cmdparam != NULL) {
		memcpy(&s_send_recv_buf[sendLen], cmdparam, strlen(cmdparam));
		sendLen += strlen(cmdparam);
	}

	memcpy(&s_send_recv_buf[sendLen], sc_smtp_cr_lf, strlen(sc_smtp_cr_lf));
	sendLen += strlen(sc_smtp_cr_lf);
	send(socket, s_send_recv_buf, sendLen, 0);

	if (respBuf != NULL) {
		memset(respBuf, 0, SMTP_BUF_LEN);
		recv(socket, respBuf, SMTP_BUF_LEN, 0);
	}
}

/*
 * Handles SMTP states.
 *
 * Return:
 *   EMAIL_ERROR_NONE if success,
 *   EMAIL_ERROR_FAILED if handler error.
 */
static int8_t smtp_state_handler(void)
{
	/* Check for acknowledge from SMTP server. */
	switch (s_smtp_status) {
	/* Send Introductory "HELO" to SMTP server. */
	case SMTP_HELO:
		smtp_recv_send(s_tcp_client_sock, (char *)"HELO localhost", NULL, s_handler_buf);
		break;
	/* Send request to server for authentication. */
	case SMTP_AUTH:
		smtp_recv_send(s_tcp_client_sock, (char *)"AUTH LOGIN", NULL, s_handler_buf);
		break;
	/* Handle authentication with server username. */
	case SMTP_AUTH_USERNAME:
		smtp_recv_send(s_tcp_client_sock, s_username_basekey, NULL, s_handler_buf);
		break;
	/* Handle authentication with server password. */
	case SMTP_AUTH_PASSWORD:
		smtp_recv_send(s_tcp_client_sock, s_password_basekey, NULL, s_handler_buf);
		break;
	/* Send source email to SMTP server. */
	case SMTP_FROM:
		smtp_recv_send(s_tcp_client_sock, (char *)sc_smtp_mail_from, (char *)SENDER_RFC, s_handler_buf);
		break;
	/* Send the destination email to SMTP server. */
	case SMTP_RCPT:
		smtp_recv_send(s_tcp_client_sock, (char *)sc_smtp_rcpt, (char *)RECIPIENT_RFC, s_handler_buf);
		break;
	/* Send the "DATA" message to server. */
	case SMTP_DATA:
		smtp_recv_send(s_tcp_client_sock, (char *)sc_smtp_data, NULL, s_handler_buf);
		break;
	/* Send actual message, preceded by From, To and Subject fields. */
	case SMTP_MESSAGE_SUBJECT:
		/* Start with email's "Subject:" field. */
		smtp_recv_send(s_tcp_client_sock, (char *)sc_smtp_subject, (char *)EMAIL_SUBJECT, NULL);
		break;
	case SMTP_MESSAGE_TO:
		/* Add email's "To:" field. */
		SYS_CONSOLE_PRINT("Recipient email address is %s\r\n", (char *)TO_ADDRESS);
		smtp_recv_send(s_tcp_client_sock, (char *)sc_smtp_to, (char *)TO_ADDRESS, NULL);
		break;
	case SMTP_MESSAGE_FROM:
		/* Add email's "From:" field. */
		smtp_recv_send(s_tcp_client_sock, (char *)sc_smtp_from, (char *)FROM_ADDRESS, NULL);
		break;
	case SMTP_MESSAGE_CRLF:
		/* Send CRLF. */
		send(s_tcp_client_sock, (char *)sc_smtp_cr_lf, strlen(sc_smtp_cr_lf), 0);
		break;
	case SMTP_MESSAGE_BODY:
		/* Send body of message. */
		smtp_recv_send(s_tcp_client_sock, (char *)EMAIL_MSG, NULL, NULL);
		break;
	case SMTP_MESSAGE_DATAEND:
		/* End of message. */
		smtp_recv_send(s_tcp_client_sock, (char *)sc_smtp_data_end, NULL, s_handler_buf);
		break;
	case SMTP_QUIT:
		send(s_tcp_client_sock, (char *)sc_smtp_quit, strlen(sc_smtp_quit), 0);
		break;
	/* Error Handling for SMTP. */
	case SMTP_ERROR:
		return EMAIL_ERROR_FAILED;
	default:
		break;
	}
	return EMAIL_ERROR_NONE;
}

static void socket_close(void)
{
	close(s_tcp_client_sock);
	s_tcp_client_sock = -1;
}

static void smtp_server_retry(void)
{
	socket_close();
	s_socket_status = SOCKET_INIT;
	s_smtp_status = SMTP_INIT;
	s_host_ip_by_name = false;
	WDRV_TIME_DELAY(WAITING_TIME);
	m2m_wifi_disconnect();
}

static uint8_t email_send(void)
{
	if (s_host_ip_by_name) {
		if (s_socket_status == SOCKET_INIT) {
			if (s_tcp_client_sock < 0) {
				s_socket_status = SOCKET_WAITING;
				if (smtp_connect() != SOCK_ERR_NO_ERROR) {
					s_socket_status = SOCKET_INIT;
				}
			}
		} else if (s_socket_status == SOCKET_CONNECT) {
			s_socket_status = SOCKET_WAITING;
			if (smtp_state_handler() != EMAIL_ERROR_NONE) {
				if (s_email_error == EMAIL_ERROR_INIT) {
					s_socket_status = SOCKET_ERROR;
				} else {
					socket_close();
				}
			}
		} else if (s_socket_status == SOCKET_COMPLETE) {
			socket_close();
			SYS_CONSOLE_PRINT("Email was successfully sent.\r\n");
		} else if (s_socket_status == SOCKET_ERROR) {
			if (s_retry_cnt < RETRY_COUNT) {
				s_retry_cnt++;
				smtp_server_retry();
				SYS_CONSOLE_PRINT("Waiting to connect to server. (30 seconds)\r\n\r\n");
			} else {
				s_retry_cnt = 0;
				socket_close();
				SYS_CONSOLE_PRINT("Failed retrying to connect to server. Press reset your board.\r\n");
			}
		}
	}
	return s_socket_status;
}

void app_init(void)
{
	SYS_CONSOLE_MESSAGE(EXAMPLE_HEADER);

	app_state_set(APP_RADIO_INIT);
	s_connected = false;
	s_host_ip_by_name = false;
	s_tcp_client_sock = -1;
	s_socket_status = SOCKET_INIT;
	s_host_ip = 0;
	s_smtp_status = SMTP_INIT;
	s_email_error = EMAIL_ERROR_NONE;
	memset(s_send_recv_buf, 0 , sizeof(s_send_recv_buf));
	memset(s_handler_buf, 0 , sizeof(s_handler_buf));
	memset(s_username_basekey, 0 , sizeof(s_username_basekey));
	memset(s_password_basekey, 0 , sizeof(s_password_basekey));
	s_retry_cnt = 0;
}

void app_task(void)
{
	int8_t ret;

	switch (app_state_get()) {
	case APP_RADIO_INIT:
		radio_init(NULL);
		app_state_set(APP_WIFI_OPEN);
		break;
	case APP_WIFI_OPEN:
		ret = wifi_open();
		if (ret)
			app_state_set(APP_END);
		else
			app_state_set(APP_WIFI_CONNECT_WAIT);
		break;
	case APP_WIFI_CONNECT_WAIT:
		if (is_link_up())
			app_state_set(APP_NET_UP);
		break;
	case APP_NET_UP:
		ret = email_send();
		if ((ret == SOCKET_COMPLETE) || (ret == SOCKET_ERROR))
			app_state_set(APP_END);
		break;
	case APP_END:
		radio_deinit();
		app_state_set(APP_PARK);
		break;
	case APP_PARK:
		/* Example has finished. Spinning wheels... */
		break;
	default:
		break;
	}
}

#endif /* EMAIL_SEND_EXAMPLE */

//DOM-IGNORE-END
