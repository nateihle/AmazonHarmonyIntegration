<#-- tcpip_tcp_server_ssl_macros_app.c.ftl -->

<#--
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************

#include "${APP_NAME?lower_case}.h"
-->
<#macro macro_lib_tcpip_app_c_includes>
</#macro>

<#--
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data
*/
-->
<#macro macro_lib_tcpip_app_c_global_data>
<#if ("CONFIG_TCPIP_TCP_SERVER_SSL_TXRX" + "${HCONFIG_APP_INSTANCE}")?eval>
static char ${APP_NAME?lower_case}MsgToClient[] = "${("CONFIG_TCPIP_TCP_SERVER_SSL_TXRX_TRANSMIT_STRING" + "${HCONFIG_APP_INSTANCE}")?eval}\n\r";
static uint8_t ${APP_NAME?lower_case}MsgFromClient[${("CONFIG_TCPIP_TCP_SERVER_SSL_TXRX_RCV_BUFFER_SIZE" + "${HCONFIG_APP_INSTANCE}")?eval}];

static TCPIP_NET_HANDLE    	${APP_NAME?lower_case}_netH;
static SYS_STATUS          	${APP_NAME?lower_case}_tcpipStat;
static int                 	${APP_NAME?lower_case}_nNets;
</#if>
</#macro>

<#--
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
-->
<#macro macro_lib_tcpip_app_c_callback_functions>
</#macro>

<#--
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
-->
<#macro macro_lib_tcpip_app_c_local_functions>
<#if ("CONFIG_TCPIP_TCP_SERVER_SSL_TXRX" + "${HCONFIG_APP_INSTANCE}")?eval>
/******************************************************************************
  Function:
    static void TCP_Server_TXRX_Task (void)
    
   Remarks:
    Feeds the USB write function. 
*/
static void TCP_Server_TXRX_Task (void)
{
	static IPV4_ADDR    		dwLastIP[2] = { {-1}, {-1} };
	static IPV4_ADDR           	ipAddr;
	int                 		i;
	NET_PRES_SKT_ERROR_T 		error;

	switch (${APP_NAME?lower_case}Data.txrxTaskState)
	{
        case ${APP_NAME?upper_case}_TCPIP_WAIT_FOR_IP:
        {
            ${APP_NAME?lower_case}_nNets = TCPIP_STACK_NumberOfNetworksGet();

            for (i = 0; i < ${APP_NAME?lower_case}_nNets; i++)
            {
                ${APP_NAME?lower_case}_netH = TCPIP_STACK_IndexToNet(i);
                ipAddr.Val = TCPIP_STACK_NetAddress(${APP_NAME?lower_case}_netH);
                if (TCPIP_STACK_NetIsReady(${APP_NAME?lower_case}_netH))
                {
                    ${APP_NAME?lower_case}Data.txrxTaskState = ${APP_NAME?upper_case}_TCPIP_OPENING_SERVER_SOCKET;
                }
            }
            break;
        }
        case ${APP_NAME?upper_case}_TCPIP_OPENING_SERVER_SOCKET:
        {
            ${APP_NAME?lower_case}Data.socket = NET_PRES_SocketOpen(0, NET_PRES_SKT_ENCRYPTED_STREAM_SERVER, IP_ADDRESS_TYPE_IPV4, ${APP_NAME?lower_case}Data.port, 0, &error);
            if (${APP_NAME?lower_case}Data.socket == NET_PRES_INVALID_SOCKET)
            {
                break;
            }
            ${APP_NAME?lower_case}Data.txrxTaskState = ${APP_NAME?upper_case}_TCPIP_WAIT_FOR_CONNECTION;;
        }
        break;

        case ${APP_NAME?upper_case}_TCPIP_WAIT_FOR_CONNECTION:
        {
            if (!NET_PRES_SocketIsConnected(${APP_NAME?lower_case}Data.socket))
            {
                break;
            }
			${APP_NAME?lower_case}Data.txrxTaskState = ${APP_NAME?upper_case}_TCPIP_NEGOTIATING_SSL;
        }
        break;

		case ${APP_NAME?upper_case}_TCPIP_NEGOTIATING_SSL:
		{
            if (NET_PRES_SocketIsNegotiatingEncryption(${APP_NAME?lower_case}Data.socket))
            {
                break;
            }
            if (!NET_PRES_SocketIsSecure(${APP_NAME?lower_case}Data.socket))
            {
                ${APP_NAME?lower_case}Data.txrxTaskState = ${APP_NAME?upper_case}_TCPIP_CLOSING_CONNECTION;
                break;
            }
            ${APP_NAME?lower_case}Data.txrxTaskState = ${APP_NAME?upper_case}_TCPIP_SENDING_MSG;
		}
        break;

		case ${APP_NAME?upper_case}_TCPIP_SENDING_MSG:
		{
            if (!NET_PRES_SocketIsConnected(${APP_NAME?lower_case}Data.socket))
            {
                ${APP_NAME?lower_case}Data.txrxTaskState = ${APP_NAME?upper_case}_TCPIP_CLOSING_CONNECTION;
                break;
            }
            if (NET_PRES_SocketWriteIsReady(${APP_NAME?lower_case}Data.socket, strlen(${APP_NAME?lower_case}MsgToClient), strlen(${APP_NAME?lower_case}MsgToClient)) != 0)
            {
                NET_PRES_SocketWrite(${APP_NAME?lower_case}Data.socket, (uint8_t*)${APP_NAME?lower_case}MsgToClient, strlen(${APP_NAME?lower_case}MsgToClient));
                ${APP_NAME?lower_case}Data.txrxTaskState = ${APP_NAME?upper_case}_TCPIP_WAIT_FOR_RESPONSE;
            }
		}

        case ${APP_NAME?upper_case}_TCPIP_WAIT_FOR_RESPONSE:
        {
            if (!NET_PRES_SocketIsConnected(${APP_NAME?lower_case}Data.socket))
            {
                ${APP_NAME?lower_case}Data.txrxTaskState = ${APP_NAME?upper_case}_TCPIP_WAIT_FOR_IP;
                break;
            }
            if (NET_PRES_SocketReadIsReady(${APP_NAME?lower_case}Data.socket))
            {
                NET_PRES_SocketRead(${APP_NAME?lower_case}Data.socket, ${APP_NAME?lower_case}MsgFromClient, sizeof(${APP_NAME?lower_case}MsgFromClient) - 1);
            }
        }
        break;

        case ${APP_NAME?upper_case}_TCPIP_CLOSING_CONNECTION:
        {
            NET_PRES_SocketClose(${APP_NAME?lower_case}Data.socket);
            ${APP_NAME?lower_case}Data.txrxTaskState = ${APP_NAME?upper_case}_TCPIP_OPENING_SERVER_SOCKET;
        }

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}
</#if>
</#macro>

<#--
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void ${APP_NAME?upper_case}_Initialize ( void )

  Remarks:
    See prototype in ${APP_NAME?lower_case}.h.
 */

void ${APP_NAME?upper_case}_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    ${APP_NAME?lower_case}Data.state = ${APP_NAME?upper_case}_STATE_INIT;
-->
<#macro macro_lib_tcpip_app_c_initialize>
<#if ("CONFIG_TCPIP_TCP_SERVER_SSL_TXRX" + "${HCONFIG_APP_INSTANCE}")?eval>
	${APP_NAME?lower_case}Data.port = ${("CONFIG_TCPIP_TCP_SERVER_SSL_TXRX_PORT" + "${HCONFIG_APP_INSTANCE}")?eval};
</#if>
</#macro>

<#--
}


/******************************************************************************
  Function:
    void ${APP_NAME?upper_case}_Tasks ( void )

  Remarks:
    See prototype in ${APP_NAME?lower_case}.h.
 */

void ${APP_NAME?upper_case}_Tasks ( void )
{
-->
<#macro macro_lib_tcpip_app_c_tasks_data>
<#if ("CONFIG_TCPIP_TCP_SERVER_SSL_TXRX" + "${HCONFIG_APP_INSTANCE}")?eval>
    int i;
</#if>
</#macro>

<#--
    /* Check the application's current state. */
    switch ( ${APP_NAME?lower_case}Data.state )
    {
        /* Application's initial state. */
        case ${APP_NAME?upper_case}_STATE_INIT:
        {
            bool appInitialized = true;
-->   
<#macro macro_lib_tcpip_app_c_tasks_state_init>
<#if ("CONFIG_TCPIP_TCP_SERVER_SSL_TXRX" + "${HCONFIG_APP_INSTANCE}")?eval>
            ${APP_NAME?lower_case}_tcpipStat = TCPIP_STACK_Status(sysObj.tcpip);
            if(${APP_NAME?lower_case}_tcpipStat < 0)
            {   // some error occurred
                ${APP_NAME?lower_case}Data.state = ${APP_NAME?upper_case}_STATE_ERROR;
				appInitialized = false;
            }
            else if(${APP_NAME?lower_case}_tcpipStat == SYS_STATUS_READY)
            {
                // now that the stack is ready we can check the
                // available interfaces
                ${APP_NAME?lower_case}_nNets = TCPIP_STACK_NumberOfNetworksGet();
                for(i = 0; i < ${APP_NAME?lower_case}_nNets; i++)
                {
                    ${APP_NAME?lower_case}_netH = TCPIP_STACK_IndexToNet(i);
                }
                ${APP_NAME?lower_case}Data.txrxTaskState = ${APP_NAME?upper_case}_TCPIP_WAIT_FOR_IP;
            }
</#if>
</#macro>    

<#--        
            if (appInitialized)
            {
-->
<#macro macro_lib_tcpip_app_c_tasks_calls_after_init>
</#macro>

<#--            /* Advance to the next state */
                ${APP_NAME?lower_case}Data.state = ${APP_NAME?upper_case}_STATE_SERVICE_TASKS;
            }
            break;
        }

        case ${APP_NAME?upper_case}_STATE_SERVICE_TASKS:
        {
-->
<#macro macro_lib_tcpip_app_c_tasks_state_service_tasks>
<#if ("CONFIG_TCPIP_TCP_SERVER_SSL_TXRX" + "${HCONFIG_APP_INSTANCE}")?eval>
            TCP_Server_TXRX_Task();
</#if>
</#macro>

<#--        
            break;
        }

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}
-->

<#macro macro_lib_tcpip_app_c_tasks_states>
<#if ("CONFIG_TCPIP_TCP_SERVER_SSL_TXRX" + "${HCONFIG_APP_INSTANCE}")?eval>
        case ${APP_NAME?upper_case}_STATE_ERROR:
        {
			break;
		}
</#if>
</#macro>
