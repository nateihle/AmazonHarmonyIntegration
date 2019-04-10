// DOM-IGNORE-BEGIN
/*******************************************************************************
* Source contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC or Microchip Technology Inc.
* Copyright (c) 2011, 2012 SEARAN LLC. All Rights Reserved.
*******************************************************************************/
// DOM-IGNORE-END

#include "cdbt/bt/bt_std.h"
#include "cdbt/bt/bt_private.h"
#include "cdbt/bt/bt_hcitr.h"
#include "cdbt/hci/hci.h"
#include "cdbt/hci/hci_transport.h"
#include "cdbt/hci/hcitr_tih4.h"
#include <string.h>

// HCILL packets
#define HCILL_GO_TO_SLEEP_IND      0x30
#define HCILL_GO_TO_SLEEP_ACK      0x31
#define HCILL_WAKE_UP_IND          0x32
#define HCILL_WAKE_UP_ACK          0x33


typedef enum _power_state_t
{
    POWER_STATE_AWAKE,
    POWER_STATE_WAIT_WAKE_UP_ACK,
    POWER_STATE_WAIT_WAKE_UP_IND,
    POWER_STATE_SLEEP,
    POWER_STATE_GOING_TO_SLEEP
} power_state_t;


typedef enum _rx_state_t
{
    RX_STATE_IDLE,
    RX_STATE_PACKET_TYPE,
    RX_STATE_PACKET_HEADER,
    RX_STATE_PACKET_PAYLOAD,
    RX_STATE_DISCARD_PACKET_PAYLOAD
} rx_state_t;


#define FLAG_SENDING              0x01
#define FLAG_SEND_PENDING         0x02
#define FLAG_RECEIVING            0x04
#define FLAG_SENDING_HCILL        0x08
#define FLAG_HCILL_PENDING        0x10

// Array of zeroes for sending packet padding
#define PADDING_LEN 16
static const bt_byte PADDING[PADDING_LEN] = {0};

static power_state_t                          _power_state;
static bt_hcitr_tih4_power_callback_fp        _power_callback;
static bt_byte                                _flags;

static rx_state_t                             _rx_state;
static bt_byte*                               _rx_packet;
static bt_int                                 _rx_packet_max_len;
static bt_int                                 _rx_packet_len;
static bt_int                                 _rx_packet_discard_len;
static bt_int                                 _rx_len;
static bt_hci_transport_recv_packet_callback_fp  _recv_packet_callback;

static const bt_byte*                         _tx_packet;
static bt_uint                                _tx_packet_len;
static bt_int                                 _tx_padding_len;
static bt_hci_transport_send_packet_callback_fp  _send_packet_callback;

static bt_byte                                _hcill_packet;
static bt_byte                                _pending_hcill_packet;
static bt_byte                                _hcill_packet;

// Buffer for receiving remainder of a long packet that
// does not fit to the buffer provided by the upper level.
// This data is discarded.
static bt_byte                                _rx_discard_buffer[8];

static void send_packet(const bt_byte* buffer, bt_uint len, bt_hci_transport_send_packet_callback_fp callback);
static void do_send_packet(void);
static void recv_packet(bt_byte* buffer, bt_uint len, bt_hci_transport_recv_packet_callback_fp callback);
static void rx_callback(bt_uint len);
static void tx_callback(void);
static void begin_host_initiated_wake_up(void);
static void begin_controller_initiated_wake_up(void);
static void begin_controller_initiated_sleep(void);
static void handle_hcill_packet(bt_byte packet);
static void send_hcill_packet(bt_byte packet);
static void send_hcill_packet_callback(void);
static void handle_pending_requests(void);


void bt_hcitr_tih4_init(bt_hcitr_tih4_power_callback_fp callback)
{
    BT_ASSERT(callback != NULL);
    if(callback!=NULL)
    {
        _power_callback = callback;
        bt_hcitr_tih4_reset();
    }
}

void bt_hcitr_tih4_reset(void)
{
    hci_transport_t transport;

    _flags = 0;
    _rx_state = RX_STATE_IDLE;
    _tx_padding_len = 0;

    transport.send_packet = &send_packet;
    transport.recv_packet = &recv_packet;

    bt_hci_transport_set_transport(&transport);

    _power_state = POWER_STATE_AWAKE;
}


void bt_hcitr_tih4_start(void)
{
    // nothing to do
}


void bt_hcitr_tih4_wake_up(void)
{
    begin_controller_initiated_wake_up();
}


static void send_packet(const bt_byte* buffer, bt_uint len, bt_hci_transport_send_packet_callback_fp callback)
{
    BT_ASSERT((_flags & (FLAG_SENDING|FLAG_SEND_PENDING)) == 0);

    if (_power_state == POWER_STATE_AWAKE)
    {
        _flags |= FLAG_SENDING;
        _tx_packet = buffer;
        _tx_packet_len = len;
        _send_packet_callback = callback;
        do_send_packet();
    }
    else
    {
        _flags |= FLAG_SEND_PENDING;
        _tx_packet = buffer;
        _tx_packet_len = len;
        _send_packet_callback = callback;

        if (_power_state == POWER_STATE_SLEEP)
        {
            begin_host_initiated_wake_up();
        }
    }
}


static void do_send_packet(void)
{
    _tx_padding_len = 0;

    if (_tx_packet[0] == HCI_PACKET_TYPE_COMMAND)
    {
        _tx_padding_len = _tx_packet[3] - _tx_packet_len + 4;
    }

    bt_oem_send(_tx_packet, _tx_packet_len, &tx_callback);
}


static void recv_packet(bt_byte* buffer, bt_uint len, bt_hci_transport_recv_packet_callback_fp callback)
{
    BT_ASSERT((_flags & FLAG_RECEIVING) == 0);
    BT_ASSERT(len > 7);

    _rx_packet = buffer;
    _rx_packet_max_len = len;
    _rx_packet_len = 0;
    _recv_packet_callback = callback;

    _rx_state = RX_STATE_PACKET_TYPE;
    _flags |= FLAG_RECEIVING;
    _rx_len = 1;
    bt_oem_recv(buffer, 1, &rx_callback);
}


static void rx_callback(bt_uint len)
{
    bt_uint next_len = 0;
    bt_byte hcill_packet = 0;

    if (len != _rx_len)
    {
//        BT_ASSERT(0);
#if DEBUGLEVEL >= DEBUGTERSE
        //printf("ERROR: rx_callback len %d != _rx_len %d\n", len, _rx_len);
#endif
        return;
    }

    if (_rx_state != RX_STATE_DISCARD_PACKET_PAYLOAD)
        _rx_packet_len += len;

    switch (_rx_state)
    {
    case RX_STATE_PACKET_TYPE:
        switch (_rx_packet[0])
        {
        case HCI_PACKET_TYPE_COMMAND:
        case HCI_PACKET_TYPE_SCO_DATA:
            next_len = 3;
            break;

        case HCI_PACKET_TYPE_ACL_DATA:
            next_len = 4;
            break;

        case HCI_PACKET_TYPE_EVENT:
            next_len = 2;
            break;

        case HCILL_GO_TO_SLEEP_IND:
        case HCILL_GO_TO_SLEEP_ACK:
        case HCILL_WAKE_UP_IND:
        case HCILL_WAKE_UP_ACK:
            next_len = 0;
            hcill_packet = 1;
            break;

        default:
//            BT_ASSERT(FALSE);
#if DEBUGLEVEL >= DEBUGTERSE
            //printf("ERROR: rx_callback _rx_packet[0] = %d\n", _rx_packet[0]);
#endif
            return;
        }

        if (next_len != 0)
            _rx_state = RX_STATE_PACKET_HEADER;

        break;

    case RX_STATE_PACKET_HEADER:
        switch (_rx_packet[0])
        {
        case HCI_PACKET_TYPE_COMMAND:
        case HCI_PACKET_TYPE_SCO_DATA:
            next_len = _rx_packet[3];
            break;
        case HCI_PACKET_TYPE_ACL_DATA:
            next_len = _rx_packet[3] + (_rx_packet[4] << 8);
            break;
        case HCI_PACKET_TYPE_EVENT:
            next_len = _rx_packet[2];
            break;
        default:
            BT_ASSERT(FALSE);
        }

        if (next_len != 0)
        {
            _rx_state = RX_STATE_PACKET_PAYLOAD;
            _rx_packet_discard_len = (_rx_packet_len + next_len) - _rx_packet_max_len;
            if (_rx_packet_discard_len > 0)
            {
                next_len -= _rx_packet_discard_len;
            }
        }

        break;

    case RX_STATE_PACKET_PAYLOAD:
    case RX_STATE_DISCARD_PACKET_PAYLOAD:
        if (_rx_packet_discard_len > 0)
        {
            next_len = sizeof(_rx_discard_buffer);
            if (next_len > _rx_packet_discard_len)
                next_len = _rx_packet_discard_len;
            _rx_packet_discard_len -= next_len;
            _rx_state = RX_STATE_DISCARD_PACKET_PAYLOAD;
        }
        else
        {
            next_len = 0;
        }
        break;

    default:
        BT_ASSERT(0);
    }

    if (next_len != 0)
    {
        _rx_len = next_len;
        if (_rx_state == RX_STATE_DISCARD_PACKET_PAYLOAD)
        {
            bt_oem_recv(_rx_discard_buffer, next_len, &rx_callback);
        }
        else
        {
            bt_oem_recv(_rx_packet + _rx_packet_len, next_len, &rx_callback);
        }
    }
    else
    {
        if (hcill_packet)
        {
            handle_hcill_packet(_rx_packet[0]);

            // Restart receiving packet.
            BT_ASSERT(_rx_state == RX_STATE_PACKET_TYPE);
            _rx_packet_len = 0;
            bt_oem_recv(_rx_packet, 1, &rx_callback);
        }
        else
        {
            _rx_state = RX_STATE_IDLE;
            _flags &= ~FLAG_RECEIVING;
            _recv_packet_callback(_rx_packet_len);
        }
    }
}


static void tx_callback(void)
{
    if (_tx_padding_len)
    {
        bt_int send_len = bt_min(PADDING_LEN, _tx_padding_len);
        _tx_padding_len -= send_len;
        bt_oem_send(PADDING, send_len, &tx_callback);
    }
    else if (_send_packet_callback)
    {
        bt_hci_transport_send_packet_callback_fp cb = _send_packet_callback;
        _send_packet_callback = NULL;
        _flags &= ~FLAG_SENDING;
        cb();
        handle_pending_requests();
    }

}


static void begin_host_initiated_wake_up(void)
{
    BT_ASSERT(_power_state == POWER_STATE_SLEEP);
    BT_ASSERT((_flags & (FLAG_SENDING|FLAG_SENDING_HCILL)) == 0);

    // Tell the platform to wake up.  The platform must do the following:
    // - Disable the wake up CTS interrupt.
    // - Release RTS (set to 0)
    _power_callback(HCITR_TIH4_POWER_EVENT_WAKE_UP);

    // Send HCILL_WAKE_UP_IND to the controller to wake it up.
    _flags |= FLAG_SENDING_HCILL;
    _hcill_packet = HCILL_WAKE_UP_IND;
    bt_oem_send(&_hcill_packet, 1, &send_hcill_packet_callback);

    // Now wait for HCILL_WAKE_UP_ACK from the controller.
    _power_state = POWER_STATE_WAIT_WAKE_UP_ACK;
}


static void begin_controller_initiated_wake_up(void)
{
    if (_power_state == POWER_STATE_SLEEP)
    {
        // Tell the platform to wake up.  The platform must do the following:
        // - Disable the wake up CTS interrupt.
        // - Release RTS (set to 0)
        _power_callback(HCITR_TIH4_POWER_EVENT_WAKE_UP);

        // Now wait for HCILL_WAKE_UP_IND from the controller.
        _power_state = POWER_STATE_WAIT_WAKE_UP_IND;
    }
}


static void begin_controller_initiated_sleep(void)
{
    // Enter this intermediate state until the acknowledgement is sent.
    _power_state = POWER_STATE_GOING_TO_SLEEP;

    // Tell the platform to prepare to sleep.  The platform must do the following:
    // - Enable the wake up CTS interrupt
    // - Pull RTS high (set to 1)
    _power_callback(HCITR_TIH4_POWER_EVENT_PREPARE_TO_SLEEP);

    // Send aknowledgment.  When this packet is sent transition
    // to the sleep mode is complete.
    send_hcill_packet(HCILL_GO_TO_SLEEP_ACK);
}


static void handle_hcill_packet(bt_byte packet)
{
    BT_LOGINT("H4TI: Received HCILL packet: ", packet);

    switch (packet)
    {
        case HCILL_GO_TO_SLEEP_IND:
            switch (_power_state)
            {
                case POWER_STATE_AWAKE:
                    begin_controller_initiated_sleep();
                    break;
                default:
                    // Ignore in all other states.
                    break;
            }
            break;

        case HCILL_GO_TO_SLEEP_ACK:
            // We never send HCILL_GO_TO_SLEEP_IND so we should not receive
            // HCILL_GO_TO_SLEEP_ACK.
            BT_ASSERT(0);
            break;

        case HCILL_WAKE_UP_IND:
            switch (_power_state)
            {
                case POWER_STATE_AWAKE:
                    // We are already awake, just reply with HCILL_WAKE_UP_ACK.
                    send_hcill_packet(HCILL_WAKE_UP_ACK);
                    break;
                case POWER_STATE_WAIT_WAKE_UP_ACK:
                    // If HCILL_WAKE_UP_IND is received while waiting for
                    // wake up aknowledgement the host must enter awake state
                    // and DO NOT reply with HCILL_WAKE_UP_ACK.
                    _power_state = POWER_STATE_AWAKE;
                    handle_pending_requests();
                    break;
                case POWER_STATE_WAIT_WAKE_UP_IND:
                    // We received what we were waiting for.
                    _power_state = POWER_STATE_AWAKE;
                    // Reply with HCILL_WAKE_UP_ACK.
                    send_hcill_packet(HCILL_WAKE_UP_ACK);
                    break;
                case POWER_STATE_GOING_TO_SLEEP:
                case POWER_STATE_SLEEP:
                    // It is a possible situation that a wakeup indicator is
                    // received before we went to sleep but only now we got to
                    // processing it. In this case the hardware is already
                    // configured for sleep so first we tell the platform to
                    // wake it up.
                    _power_callback(HCITR_TIH4_POWER_EVENT_WAKE_UP);
                    // Since WAKEUP_IND received we are now awake.
                    _power_state = POWER_STATE_AWAKE;
                    // Reply with HCILL_WAKE_UP_ACK.
                    send_hcill_packet(HCILL_WAKE_UP_ACK);
                    break;
            }
            break;

        case HCILL_WAKE_UP_ACK:
            switch (_power_state)
            {
                case POWER_STATE_AWAKE:
                    // Just ignore it (should not really happen).
                    break;
                case POWER_STATE_WAIT_WAKE_UP_ACK:
                    // We received what we were waiting for.
                    _power_state = POWER_STATE_AWAKE;
                    handle_pending_requests();
                    break;
                case POWER_STATE_WAIT_WAKE_UP_IND:
                case POWER_STATE_SLEEP:
                case POWER_STATE_GOING_TO_SLEEP:
                    // Should not happen.
                    BT_ASSERT(0);
                    break;
            }
            break;
    }
}


static void send_hcill_packet(bt_byte packet)
{
    // Send HCILL packet if the channel is free.
    if ((_flags & (FLAG_SENDING | FLAG_SENDING_HCILL)) == 0)
    {
        BT_LOGINT("Sending HCILL packet: ", (int)packet);
        _hcill_packet = packet;
        _flags |= FLAG_SENDING_HCILL;
        bt_oem_send(&_hcill_packet, 1, send_hcill_packet_callback);
    }
    else
    {
        // Othrewise, save it to send later.
        BT_ASSERT((_flags & FLAG_HCILL_PENDING) == 0);
        BT_LOGINT("Pending HCILL packet: ", (int)packet);
        _flags |= FLAG_HCILL_PENDING;
        _pending_hcill_packet = packet;
    }
}


static void send_hcill_packet_callback(void)
{
    _flags &= ~FLAG_SENDING_HCILL;

    BT_LOG("HCILL packet sent");

    switch (_hcill_packet)
    {
        case HCILL_GO_TO_SLEEP_ACK:
            if (_power_state == POWER_STATE_GOING_TO_SLEEP)
            {
                // Transition to sleep is complete.
                _power_state = POWER_STATE_SLEEP;

                // If there is a pending send request, initiate the wake up procedure.
                if (_flags & FLAG_SEND_PENDING)
                {
                    begin_host_initiated_wake_up();
                }
                else
                {
                    // Otherwise, tell the platform it can enter a low power mode.
                    _power_callback(HCITR_TIH4_POWER_EVENT_SLEEP);
                }
            }
            else
            {
                handle_pending_requests();
            }
            break;

        case HCILL_WAKE_UP_IND:
            // fall through
        case HCILL_WAKE_UP_ACK:
            handle_pending_requests();
            break;

        default:
            BT_ASSERT(0);
    }
}


static void handle_pending_requests(void)
{
    // Check if the channel is free.
    if ((_flags & (FLAG_SENDING | FLAG_SENDING_HCILL)) != 0)
        return;

    // Send pending HCILL packet if there is one.
    if (_flags & FLAG_HCILL_PENDING)
    {
        _flags &= ~FLAG_HCILL_PENDING;
        send_hcill_packet(_pending_hcill_packet);
        return;
    }

    if (_power_state == POWER_STATE_AWAKE)
    {
        // Send pending HCI packet if there is one (only if awake)
        if (_flags & FLAG_SEND_PENDING)
        {
            _flags &= ~FLAG_SEND_PENDING;
            _flags |= FLAG_SENDING;
            do_send_packet();
        }
    }

}
