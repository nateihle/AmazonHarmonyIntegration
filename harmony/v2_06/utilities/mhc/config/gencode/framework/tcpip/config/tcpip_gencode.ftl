menu "TCPIP"
    depends on HAVE_ETH
    
config TCPIP_TCP_CLIENT_TXRX${INSTANCE}
    bool "TCP Client: Transmit and Receive (same socket)"
    set USE_TCPIP_STACK_NEEDED to y if TCPIP_TCP_CLIENT_TXRX${INSTANCE}
    set TCPIP_USE_DNS_CLIENT to n if TCPIP_TCP_CLIENT_TXRX${INSTANCE}
    set TCPIP_NETWORK_DEFAULT_INTERFACE_NAME_IDX0 to "PIC32INT" if TCPIP_TCP_CLIENT_TXRX${INSTANCE}
    set TCPIP_NETWORK_DEFAULT_MAC_ADDR_IDX0 to "00:04:a3:11:22:33" if TCPIP_TCP_CLIENT_TXRX${INSTANCE}
    set TCPIP_USE_LINK_ZERO_CONFIG to y if TCPIP_TCP_CLIENT_TXRX${INSTANCE}
	set TCPIP_USE_ETH_MAC to y if TCPIP_TCP_CLIENT_TXRX${INSTANCE}
	set USE_CRYPTO_LIB_NEEDED to n if TCPIP_TCP_CLIENT_TXRX${INSTANCE}
    default n
	---help---
	<!DOCTYPE HTML>
	<html>
	<h2>MPLAB Harmony TCP Client Application Template</h2>
	<h3>Transmit and Receive to/from TCP/IP Server</h2>
	<p>	This template generates a simple code example which transmits
	a string of bytes to and receives a string of bytes from a TCPIP 
	Server. It will automatically configure the TCPIP stack with the 
	following settings:</p>
	<br>- Internal Ethernet MAC
	<br>- External SMSC LAN8740 PHY
	<br>- PIC32INT Interface
	<br>- MAC Address: 00:04:a3:11:22:3
	<br>- DNS Client Disabled
	<br>- Crypto Library Disabled
	<p>All other TCPIP configuration options are set to their 
	default values. The driver configuration may be modified by 
	the user using MHC, under Harmony Framework Configuration -> 
	TCP\IP Stack</p></html>
	---endhelp---

config TCPIP_TCP_CLIENT_TXRX_SERVER_IP${INSTANCE}
    string "Server IP"
	depends on TCPIP_TCP_CLIENT_TXRX${INSTANCE}
	default "192.168.100.101"

config TCPIP_TCP_CLIENT_TXRX_PORT${INSTANCE}
    int "Port"
	depends on TCPIP_TCP_CLIENT_TXRX${INSTANCE}
    default 49152

config TCPIP_TCP_CLIENT_TXRX_TRANSMIT_STRING${INSTANCE}
    string "Transmit String"
	depends on TCPIP_TCP_CLIENT_TXRX${INSTANCE}
    default "Hello Server!"

config TCPIP_TCP_CLIENT_TXRX_RCV_BUFFER_SIZE${INSTANCE}
	int "Number of Characters to Receive"
	depends on TCPIP_TCP_CLIENT_TXRX${INSTANCE}
	default 80

config TCPIP_TCP_CLIENT_TX${INSTANCE}
    bool "TCP Client: Transmit"
    set USE_TCPIP_STACK_NEEDED to y if TCPIP_TCP_CLIENT_TX${INSTANCE}
    set TCPIP_USE_DNS_CLIENT to n if TCPIP_TCP_CLIENT_TX${INSTANCE}
    set TCPIP_NETWORK_DEFAULT_INTERFACE_NAME_IDX0 to "PIC32INT" if TCPIP_TCP_CLIENT_TX${INSTANCE}
    set TCPIP_NETWORK_DEFAULT_MAC_ADDR_IDX0 to "00:04:a3:11:22:33" if TCPIP_TCP_CLIENT_TX${INSTANCE}
    set TCPIP_USE_LINK_ZERO_CONFIG to y if TCPIP_TCP_CLIENT_TX${INSTANCE}
	set TCPIP_USE_ETH_MAC to y if TCPIP_TCP_CLIENT_TX${INSTANCE}
	set USE_CRYPTO_LIB_NEEDED to n if TCPIP_TCP_CLIENT_TX${INSTANCE}
    default n
    ---help---
	<!DOCTYPE HTML>
	<html>
	<h2>MPLAB Harmony TCP Client Application Template</h2>
	<h3>Transmit to TCP/IP Server</h2>
	<p>	This template generates a simple code example which transmits
	a string of bytes to a TCPIP Server. It will automatically configure
	the TCPIP stack with the following settings:</p>
	<br>- Internal Ethernet MAC
	<br>- External SMSC LAN8740 PHY
	<br>- PIC32INT Interface
	<br>- MAC Address: 00:04:a3:11:22:3
	<br>- DNS Client Disabled
	<br>- Crypto Library Disabled
	<p>All other TCPIP configuration options are set to their 
	default values. The driver configuration may be modified by 
	the user using MHC, under Harmony Framework Configuration -> 
	TCP\IP Stack</p></html>
	---endhelp---


config TCPIP_TCP_CLIENT_TX_SERVER_IP${INSTANCE}
    string "Server IP"
	depends on TCPIP_TCP_CLIENT_TX${INSTANCE}
	default "192.168.100.101"

config TCPIP_TCP_CLIENT_TX_PORT${INSTANCE}
    int "Port"
	depends on TCPIP_TCP_CLIENT_TX${INSTANCE}
    default 49152

config TCPIP_TCP_CLIENT_TX_TRANSMIT_STRING${INSTANCE}
    string "Transmit String"
	depends on TCPIP_TCP_CLIENT_TX${INSTANCE}
    default "Hello Server!"

config TCPIP_TCP_CLIENT_RX${INSTANCE}
    bool "TCP Client: Receive"
    set USE_TCPIP_STACK_NEEDED to y if TCPIP_TCP_CLIENT_RX${INSTANCE}
    set TCPIP_USE_DNS_CLIENT to n if TCPIP_TCP_CLIENT_RX${INSTANCE}
    set TCPIP_NETWORK_DEFAULT_INTERFACE_NAME_IDX0 to "PIC32INT" if TCPIP_TCP_CLIENT_RX${INSTANCE}
    set TCPIP_NETWORK_DEFAULT_MAC_ADDR_IDX0 to "00:04:a3:11:22:33" if TCPIP_TCP_CLIENT_RX${INSTANCE}
    set TCPIP_USE_LINK_ZERO_CONFIG to y if TCPIP_TCP_CLIENT_RX${INSTANCE}
	set TCPIP_USE_ETH_MAC to y if TCPIP_TCP_CLIENT_RX${INSTANCE}
	set USE_CRYPTO_LIB_NEEDED to n if TCPIP_TCP_CLIENT_RX${INSTANCE}
    default n
    ---help---
	<!DOCTYPE HTML>
	<html>
	<h2>MPLAB Harmony TCP Client Application Template</h2>
	<h3>Receive from TCP/IP Server</h2>
	<p>	This template generates a simple code example which receives 
	a string of bytes from a TCPIP Server. It will automatically 
	configure the TCPIP stack with the following settings:</p>
	<br>- Internal Ethernet MAC
	<br>- External SMSC LAN8740 PHY
	<br>- PIC32INT Interface
	<br>- MAC Address: 00:04:a3:11:22:3
	<br>- DNS Client Disabled
	<br>- Crypto Library Disabled
	<p>All other TCPIP configuration options are set to their 
	default values. The driver configuration may be modified by 
	the user using MHC, under Harmony Framework Configuration -> 
	TCP\IP Stack</p></html>
	---endhelp---

config TCPIP_TCP_CLIENT_RX_SERVER_IP${INSTANCE}
    string "Server IP"
	depends on TCPIP_TCP_CLIENT_RX${INSTANCE}
	default "192.168.100.101"

config TCPIP_TCP_CLIENT_RX_PORT${INSTANCE}
    int "Port"
	depends on TCPIP_TCP_CLIENT_RX${INSTANCE}
    default 49152

config TCPIP_TCP_CLIENT_RX_RCV_BUFFER_SIZE${INSTANCE}
	int "Number of Characters to Receive"
	depends on TCPIP_TCP_CLIENT_RX${INSTANCE}
	default 80

config TCPIP_TCP_SERVER_TXRX${INSTANCE}
    bool "TCP Server: Transmit and Receive (same socket)"
    set USE_TCPIP_STACK_NEEDED to y if TCPIP_TCP_SERVER_TXRX${INSTANCE}
    set TCPIP_USE_DNS_CLIENT to n if TCPIP_TCP_SERVER_TXRX${INSTANCE}
    set TCPIP_NETWORK_DEFAULT_INTERFACE_NAME_IDX0 to "PIC32INT" if TCPIP_TCP_SERVER_TXRX${INSTANCE}
    set TCPIP_NETWORK_DEFAULT_MAC_ADDR_IDX0 to "00:04:a3:11:22:33" if TCPIP_TCP_SERVER_TXRX${INSTANCE}
    set TCPIP_USE_LINK_ZERO_CONFIG to y if TCPIP_TCP_SERVER_TXRX${INSTANCE}
	set TCPIP_USE_ETH_MAC to y if TCPIP_TCP_SERVER_TXRX${INSTANCE}
	set USE_CRYPTO_LIB_NEEDED to n if TCPIP_TCP_SERVER_TXRX${INSTANCE}
	set TCPIP_STACK_USE_ICMPV4 to y if TCPIP_TCP_SERVER_TXRX${INSTANCE}
	set TCPIP_STACK_USE_ICMP_SERVER to y if TCPIP_TCP_SERVER_TXRX${INSTANCE}
	set TCPIP_STACK_USE_ICMP_CLIENT to n if TCPIP_TCP_SERVER_TXRX${INSTANCE}
    default n
	---help---
	<!DOCTYPE HTML>
	<html>
	<h2>MPLAB Harmony TCP Server Application Template</h2>
	<h3>Transmit and Receive to/from TCP/IP Client</h2>
	<p>	This template generates a simple code example which transmits
	a string of bytes to and receives a string of bytes from a TCPIP 
	client. It will automatically configure the TCPIP stack with the 
	following settings:</p>
	<br>- Internal Ethernet MAC
	<br>- External SMSC LAN8740 PHY
	<br>- PIC32INT Interface
	<br>- MAC Address: 00:04:a3:11:22:3
	<br>- DNS Client Disabled
	<br>- Crypto Library Disabled
	<br>- ICMP Enabled
	<p>All other TCPIP configuration options are set to their 
	default values. The driver configuration may be modified by 
	the user using MHC, under Harmony Framework Configuration -> 
	TCP\IP Stack.</p>
	
	<p>To get server ip address, ping mchpboard_e.</p></html>
	---endhelp---

config TCPIP_TCP_SERVER_TXRX_PORT${INSTANCE}
    int "Port"
	depends on TCPIP_TCP_SERVER_TXRX${INSTANCE}
    default 49152

config TCPIP_TCP_SERVER_TXRX_TRANSMIT_STRING${INSTANCE}
    string "Transmit String"
	depends on TCPIP_TCP_SERVER_TXRX${INSTANCE}
    default "Hello Client!"

config TCPIP_TCP_SERVER_TXRX_RCV_BUFFER_SIZE${INSTANCE}
	int "Number of Characters to Receive"
	depends on TCPIP_TCP_SERVER_TXRX${INSTANCE}
	default 80

config TCPIP_TCP_SERVER_SSL_TXRX${INSTANCE}
    bool "TCP Server (Secure): Transmit and Receive (same socket)"
	set USE_CRYPTO_LIBRARY to y if TCPIP_TCP_SERVER_SSL_TXRX${INSTANCE}
	set USE_CRYPTO_RANDOM to y if TCPIP_TCP_SERVER_SSL_TXRX${INSTANCE}
	set NET_PRES_USE to y if TCPIP_TCP_SERVER_SSL_TXRX${INSTANCE}
	set NET_PRES_SUPPORT_ENCRYPTION0 to y if TCPIP_TCP_SERVER_SSL_TXRX${INSTANCE}
	set NET_PRES_SUPPORT_SERVER_ENC_IDX0 to y if TCPIP_TCP_SERVER_SSL_TXRX${INSTANCE}
	set NET_PRES_SUPPORT_CLIENT_ENC_IDX0 to n if TCPIP_TCP_SERVER_SSL_TXRX${INSTANCE}
	set NET_PRES_BLOB_CERT_REPO to y if TCPIP_TCP_SERVER_SSL_TXRX${INSTANCE}
	set NET_PRES_BLOB_CLIENT_SUPPORT to n if TCPIP_TCP_SERVER_SSL_TXRX${INSTANCE}
	set NET_PRES_BLOB_SERVER_SUPPORT to y if TCPIP_TCP_SERVER_SSL_TXRX${INSTANCE}
	set NET_PRES_BLOB_SERVER_CERT_FILENAME to "wolfssl/certs_test.h" if TCPIP_TCP_SERVER_SSL_TXRX${INSTANCE}
	set NET_PRES_BLOB_SERVER_CERT_VARIABLE to "server_cert_der_2048" if TCPIP_TCP_SERVER_SSL_TXRX${INSTANCE}
	set NET_PRES_BLOB_SERVER_CERT_LEN_VARIABLE to "sizeof_server_cert_der_2048" if TCPIP_TCP_SERVER_SSL_TXRX${INSTANCE}
	set NET_PRES_BLOB_SERVER_KEY_FILENAME to "wolfssl/certs_test.h" if TCPIP_TCP_SERVER_SSL_TXRX${INSTANCE}
	set NET_PRES_BLOB_SERVER_KEY_VARIABLE to "server_key_der_2048" if TCPIP_TCP_SERVER_SSL_TXRX${INSTANCE}
	set NET_PRES_BLOB_SERVER_KEY_LEN_VARIABLE to "sizeof_server_key_der_2048" if TCPIP_TCP_SERVER_SSL_TXRX${INSTANCE}
    set USE_TCPIP_STACK_NEEDED to y if TCPIP_TCP_SERVER_SSL_TXRX${INSTANCE}
    set TCPIP_USE_DNS_CLIENT to y if TCPIP_TCP_SERVER_SSL_TXRX${INSTANCE}
    set TCPIP_NETWORK_DEFAULT_INTERFACE_NAME_IDX0 to "PIC32INT" if TCPIP_TCP_SERVER_SSL_TXRX${INSTANCE}
    set TCPIP_NETWORK_DEFAULT_MAC_ADDR_IDX0 to "00:04:a3:11:22:33" if TCPIP_TCP_SERVER_SSL_TXRX${INSTANCE}
    set TCPIP_USE_LINK_ZERO_CONFIG to y if TCPIP_TCP_SERVER_SSL_TXRX${INSTANCE}
	set TCPIP_USE_ETH_MAC to y if TCPIP_TCP_SERVER_SSL_TXRX${INSTANCE}
	set USE_CRYPTO_LIB_NEEDED to n if TCPIP_TCP_SERVER_SSL_TXRX${INSTANCE}
	set TCPIP_STACK_USE_ICMPV4 to n if TCPIP_TCP_SERVER_SSL_TXRX${INSTANCE}
	set USE_3RDPARTY_WOLFSSL to y if TCPIP_TCP_SERVER_SSL_TXRX${INSTANCE}
	set WOLFSSL_WOLFSSL_CLIENT to n if TCPIP_TCP_SERVER_SSL_TXRX${INSTANCE}
	set XC32_HEAP to "54960" if TCPIP_TCP_SERVER_SSL_TXRX${INSTANCE}
    default n
	---help---
	<!DOCTYPE HTML>
	<html>
	<h2>MPLAB Harmony TCP Server (Secure) Application Template</h2>
	<h3>Transmit and Receive to/from TCP/IP Client</h2>
	<p>	This template generates a simple code example which transmits
	a string of bytes to and receives a string of bytes from a TCPIP 
	client. It uses the WolfSSL crypto stack in concert with the Harmony
	Net Presentation Layer to provide a secure (SSL) connection to the client.
	It will automatically configure the TCPIP stack, presentation layer, and
	WolfSSL as needed:</p>
	
	<p>To get server ip address, ping mchpboard_e.</p></html>
	---endhelp---

config TCPIP_TCP_SERVER_SSL_TXRX_PORT${INSTANCE}
    int "Port"
	depends on TCPIP_TCP_SERVER_SSL_TXRX${INSTANCE}
    default 49152

config TCPIP_TCP_SERVER_SSL_TXRX_TRANSMIT_STRING${INSTANCE}
    string "Transmit String"
	depends on TCPIP_TCP_SERVER_SSL_TXRX${INSTANCE}
    default "Hello Client!"

config TCPIP_TCP_SERVER_SSL_TXRX_RCV_BUFFER_SIZE${INSTANCE}
	int "Number of Characters to Receive"
	depends on TCPIP_TCP_SERVER_SSL_TXRX${INSTANCE}
	default 80


ifblock TCPIP_TCP_CLIENT_TXRX${INSTANCE}

add "^#include \"/utilities/mhc/config/gencode/framework/tcpip/config/tcpip_tcp_client_txrx_macros_app.h.ftl\">" to list APP_FREEMARKER_MACROS
add "^#include \"/utilities/mhc/config/gencode/framework/tcpip/config/tcpip_tcp_client_txrx_macros_app.c.ftl\">" to list APP_FREEMARKER_MACROS
add "^#include \"/utilities/mhc/config/gencode/framework/tcpip/config/tcpip_tcp_client_txrx_macros_system_config.h.ftl\">" to list APP_FREEMARKER_MACROS

endif

ifblock TCPIP_TCP_CLIENT_TX${INSTANCE}

add "^#include \"/utilities/mhc/config/gencode/framework/tcpip/config/tcpip_tcp_client_tx_macros_app.h.ftl\">" to list APP_FREEMARKER_MACROS
add "^#include \"/utilities/mhc/config/gencode/framework/tcpip/config/tcpip_tcp_client_tx_macros_app.c.ftl\">" to list APP_FREEMARKER_MACROS
add "^#include \"/utilities/mhc/config/gencode/framework/tcpip/config/tcpip_tcp_client_tx_macros_system_config.h.ftl\">" to list APP_FREEMARKER_MACROS

endif

ifblock TCPIP_TCP_CLIENT_RX${INSTANCE}

add "^#include \"/utilities/mhc/config/gencode/framework/tcpip/config/tcpip_tcp_client_rx_macros_app.h.ftl\">" to list APP_FREEMARKER_MACROS
add "^#include \"/utilities/mhc/config/gencode/framework/tcpip/config/tcpip_tcp_client_rx_macros_app.c.ftl\">" to list APP_FREEMARKER_MACROS
add "^#include \"/utilities/mhc/config/gencode/framework/tcpip/config/tcpip_tcp_client_rx_macros_system_config.h.ftl\">" to list APP_FREEMARKER_MACROS

endif

ifblock TCPIP_TCP_SERVER_TXRX${INSTANCE}

add "^#include \"/utilities/mhc/config/gencode/framework/tcpip/config/tcpip_tcp_server_txrx_macros_app.h.ftl\">" to list APP_FREEMARKER_MACROS
add "^#include \"/utilities/mhc/config/gencode/framework/tcpip/config/tcpip_tcp_server_txrx_macros_app.c.ftl\">" to list APP_FREEMARKER_MACROS
add "^#include \"/utilities/mhc/config/gencode/framework/tcpip/config/tcpip_tcp_server_txrx_macros_system_config.h.ftl\">" to list APP_FREEMARKER_MACROS

endif

ifblock TCPIP_TCP_SERVER_SSL_TXRX${INSTANCE}

add "^#include \"/utilities/mhc/config/gencode/framework/tcpip/config/tcpip_tcp_server_ssl_txrx_macros_app.h.ftl\">" to list APP_FREEMARKER_MACROS
add "^#include \"/utilities/mhc/config/gencode/framework/tcpip/config/tcpip_tcp_server_ssl_txrx_macros_app.c.ftl\">" to list APP_FREEMARKER_MACROS
add "^#include \"/utilities/mhc/config/gencode/framework/tcpip/config/tcpip_tcp_server_ssl_txrx_macros_system_config.h.ftl\">" to list APP_FREEMARKER_MACROS

endif

ifblock TCPIP_TCP_CLIENT_TXRX${INSTANCE} || TCPIP_TCP_CLIENT_TX${INSTANCE} || TCPIP_TCP_CLIENT_RX${INSTANCE} || TCPIP_TCP_SERVER_TXRX${INSTANCE} || TCPIP_TCP_SERVER_SSL_TXRX${INSTANCE}

add "^@macro_lib_tcpip_app_h_includes/>" to list APP${INSTANCE}_H_INCLUDES
add "^@macro_lib_tcpip_system_config_h_app_constants/>" to list APP${INSTANCE}_H_CONSTANTS
add "^@macro_lib_tcpip_app_h_type_definitions/>" to list APP${INSTANCE}_H_TYPE_DEFINITIONS
add "^@macro_lib_tcpip_app_h_data/>" to list APP${INSTANCE}_H_APP_DATA
add "^@macro_lib_tcpip_app_h_callback_function_declarations/>" to list APP${INSTANCE}_H_APP_CALLBACK_FUNCTION_DECLARATIONS
add "^@macro_lib_tcpip_app_h_function_declarations/>" to list APP${INSTANCE}_H_APP_FUNCTION_DECLARATIONS
add "^@macro_lib_tcpip_app_h_states/>" to list APP${INSTANCE}_H_APP_STATES

add "^@macro_lib_tcpip_app_c_includes/>" to list APP${INSTANCE}_C_INCLUDES
add "^@macro_lib_tcpip_app_c_global_data/>" to list APP${INSTANCE}_C_GLOBAL_DATA
add "^@macro_lib_tcpip_app_c_callback_functions/>" to list APP${INSTANCE}_C_CALLBACK_FUNCTIONS
add "^@macro_lib_tcpip_app_c_local_functions/>" to list APP${INSTANCE}_C_LOCAL_FUNCTIONS
add "^@macro_lib_tcpip_app_c_initialize/>" to list APP${INSTANCE}_C_INITIALIZE
add "^@macro_lib_tcpip_app_c_tasks_data/>" to list APP${INSTANCE}_C_TASKS_DATA
add "^@macro_lib_tcpip_app_c_tasks_state_init/>" to list APP${INSTANCE}_C_TASKS_STATE_INIT
add "^@macro_lib_tcpip_app_c_tasks_calls_after_init/>" to list APP${INSTANCE}_C_TASKS_CALLS_AFTER_INIT
add "^@macro_lib_tcpip_app_c_tasks_state_service_tasks/>" to list APP${INSTANCE}_C_TASKS_STATE_SERVICE_TASKS
add "^@macro_lib_tcpip_app_c_tasks_states/>" to list APP${INSTANCE}_C_TASKS_STATES

endif

endmenu
