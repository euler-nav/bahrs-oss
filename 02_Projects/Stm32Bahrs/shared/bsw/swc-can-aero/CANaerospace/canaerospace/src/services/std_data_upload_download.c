/*
 * Standard services: Data Upload Service and Data Download Service
 * Pavel Kirienko, 2013 (pavel.kirienko@gmail.com)
 */

#include <stdbool.h>
#include <string.h>
#include <canaerospace/services/std_data_upload_download.h>
#include "../debug.h"

static const int PAYLOAD_BYTES_PER_MESSAGE = 4;

static const int SERVICE_CODE_DDS = 2;
static const int SERVICE_CODE_DUS = 3;

static const uint64_t SDRM_SURM_TIMEOUT_USEC       = 100 * 1000;    // This is declared explicitly by specification
static const uint64_t DUS_SLAVE_INITIAL_DELAY_USEC = 10 * 1000;     // And this too

static const int DEFAULT_TX_INTERVAL_USEC     = 10 * 1000;
static const int DEFAULT_SESSION_TIMEOUT_USEC = 10 * 1000000;

/// Order must comply with the handler table below.
enum
{
    SESSION_TYPE_NONE = 0,
    SESSION_TYPE_DDS_MASTER, // Transmit
    SESSION_TYPE_DDS_SLAVE,  // Receive
    SESSION_TYPE_DUS_MASTER, // Receive
    SESSION_TYPE_DUS_SLAVE,  // Transmit
    SESSION_TYPE_BOUND_
};

typedef struct
{
    union
    {
        CanasSrvDdsMasterDoneCallback dds_master_done;
        CanasSrvDusMasterDoneCallback dus_master_done;
    } callback;
    void* callback_arg;
    uint64_t update_timestamp;
    uint32_t memid;
    uint8_t rx_message_count;     // Only for RX sessions
    uint8_t type;
    union
    {
        uint8_t node_id;          // For master sessions. For master, service_channel is always its own.
        uint8_t service_channel;  // For slave sessions. For slave, Node ID is always its own.
    } designation;
    uint8_t state;
    uint8_t next_message_code;
    uint16_t datalen;
    uint8_t buffer[CANAS_SRV_DATA_MAX_PAYLOAD_LEN];
} SessionEntry;

typedef struct
{
    CanasSrvDdsSlaveRequestCallback rx_request_callback;
    CanasSrvDdsSlaveDoneCallback rx_done_callback;
    CanasSrvDusSlaveRequestCallback tx_request_callback;
    uint32_t tx_interval_usec;
    uint32_t session_timeout_usec;
    uint8_t entry_count;
    SessionEntry entries[];
} ServiceState;

static uint8_t _msgcountByDatalen(uint16_t datalen)
{
    return ((datalen - 1) / PAYLOAD_BYTES_PER_MESSAGE) + 1;
}

static uint16_t _datalenByMsgcount(uint8_t msgcount)
{
    return ((msgcount + 1) * PAYLOAD_BYTES_PER_MESSAGE) - 1;
}

static uint32_t _computeChecksum(const uint8_t* pbuf, uint_fast16_t len)
{
    uint32_t chksum = 0;
    for (uint_fast16_t i = 0; i < len; i++)
        chksum += pbuf[i];
    return chksum;
}

static CanasMessageData _makeDataChunk(SessionEntry* pses, uint_fast8_t current_message_code, bool* plast_chunk)
{
    const uint_fast16_t data_offset = current_message_code * PAYLOAD_BYTES_PER_MESSAGE;
    uint_fast16_t transmit_size = pses->datalen - data_offset;

    *plast_chunk = transmit_size <= 4;

    CanasMessageData msgdata;
    memset(&msgdata, 0, sizeof(msgdata));

    if (transmit_size == 1)
        msgdata.type = CANAS_DATATYPE_UCHAR;
    else if (transmit_size == 2)
        msgdata.type = CANAS_DATATYPE_UCHAR2;
    else if (transmit_size == 3)
        msgdata.type = CANAS_DATATYPE_UCHAR3;
    else
    {
        transmit_size = 4;
        msgdata.type = CANAS_DATATYPE_UCHAR4;
    }
    memcpy(msgdata.container.UCHAR4, pses->buffer + data_offset, transmit_size);

    return msgdata;
}

// ------ DDS MASTER ------

/*
 * MASTER  --- SDRM request  -->  SLAVE
 *              100 us max
 * MASTER  <-- SDRM response ---  SLAVE
 * MASTER  ---  Data chunks  -->  SLAVE
 *         <-- opt. XON/XOFF ---
 *                 ***
 *          SESSION TIMOUT max
 * MASTER  <--   Checksum    ---  SLAVE
 */

enum
{
    DDS_MASTER_STATE_SDRM_PENDING, // Start Download Request Message
    DDS_MASTER_STATE_TRANSMISSION,
    DDS_MASTER_STATE_CHECKSUM,     // Checksum is transmitted from SLAVE to MASTER
    DDS_MASTER_STATE_XOFF
};

static void _ddsMasterDone(CanasInstance* pi, SessionEntry* pses, CanasSrvDataSessionStatus status, int32_t* premoteerr)
{
    CanasSrvDdsMasterDoneCallbackArgs cbargs;
    memset(&cbargs, 0, sizeof(cbargs));
    cbargs.status  = status;
    cbargs.node_id = pses->designation.node_id;
    cbargs.memid   = pses->memid;
    cbargs.parg    = pses->callback_arg;
    if (premoteerr != NULL)
        cbargs.remote_error_code = *premoteerr;
    if (pses->callback.dds_master_done != NULL)           // Callback is required
        pses->callback.dds_master_done(pi, &cbargs);
    memset(pses, 0, sizeof(*pses));                       // Erase this entry
}

static int _ddsMasterTransmitNextChunk(CanasInstance* pi, SessionEntry* pses, bool* plast_chunk)
{
    CanasMessage msg;
    memset(&msg, 0, sizeof(msg));
    msg.node_id      = pses->designation.node_id;
    msg.service_code = SERVICE_CODE_DDS;
    msg.message_code = pses->next_message_code++;
    msg.data         = _makeDataChunk(pses, msg.message_code, plast_chunk);
    return canasServiceSendRequest(pi, &msg);
}

static void _ddsMasterPoll(CanasInstance* pi, CanasServicePollCallbackArgs* pargs, SessionEntry* pses)
{
    ServiceState* pstate = (ServiceState*)pargs->pstate;
    const uint64_t sinceupdate = pargs->timestamp_usec - pses->update_timestamp;

    switch (pses->state)
    {
    case DDS_MASTER_STATE_SDRM_PENDING:
        if (sinceupdate > SDRM_SURM_TIMEOUT_USEC)
        {
            CANAS_TRACE("srv dds master poll: SDRM timeout\n");
            _ddsMasterDone(pi, pses, CANAS_SRV_DATA_SESSION_TIMEOUT, NULL);
        }
        break;
    case DDS_MASTER_STATE_TRANSMISSION:
        if (sinceupdate >= pstate->tx_interval_usec)
        {
            pses->update_timestamp = pargs->timestamp_usec;
            bool last_chunk = false;
            int res = _ddsMasterTransmitNextChunk(pi, pses, &last_chunk);
            if (res != 0)
            {
                CANAS_TRACE("srv dds master poll: transmission failure: %i\n", res);
                _ddsMasterDone(pi, pses, CANAS_SRV_DATA_SESSION_LOCAL_ERROR, NULL);
            }
            else if (last_chunk)
            {
                CANAS_TRACE("srv dds master poll: last chunk has been sent\n");
                pses->state = DDS_MASTER_STATE_CHECKSUM;
            }
        }
        break;
    case DDS_MASTER_STATE_CHECKSUM:
    case DDS_MASTER_STATE_XOFF:
        if (sinceupdate > pstate->session_timeout_usec)
        {
            CANAS_TRACE("srv dds master poll: checksum or XOFF timeout\n");
            _ddsMasterDone(pi, pses, CANAS_SRV_DATA_SESSION_TIMEOUT, NULL);
        }
        break;
    default:
        CANAS_TRACE("srv dds master poll: invalid state %i\n", (int)pses->state);
        _ddsMasterDone(pi, pses, CANAS_SRV_DATA_SESSION_LOCAL_ERROR, NULL);
        break;
    }
}

static void _ddsMasterResponse(CanasInstance* pi, CanasServiceResponseCallbackArgs* pargs, SessionEntry* pses)
{
    pses->update_timestamp = pargs->timestamp_usec;

    switch (pses->state)
    {
    case DDS_MASTER_STATE_SDRM_PENDING:
    case DDS_MASTER_STATE_XOFF:
    case DDS_MASTER_STATE_TRANSMISSION:
        if (pargs->message.data.type == CANAS_DATATYPE_LONG)
        {
            if (pargs->message.data.container.LONG == CANAS_SRV_DDS_RESPONSE_XON)
            {
                pses->state = DDS_MASTER_STATE_TRANSMISSION;
                break;
            }
            if (pargs->message.data.container.LONG == CANAS_SRV_DDS_RESPONSE_XOFF)
            {
                pses->state = DDS_MASTER_STATE_XOFF;
                break;
            }
            // Neither XON nor XOFF are fatal errors
            CANAS_TRACE("srv dds master resp: bad status code: %li\n", (long)pargs->message.data.container.LONG);
            _ddsMasterDone(pi, pses, CANAS_SRV_DATA_SESSION_REMOTE_ERROR, &pargs->message.data.container.LONG);
        }
        else
        {
            CANAS_TRACE("srv dds master resp: bad data type (not long): %i\n", (int)pargs->message.data.type);
            _ddsMasterDone(pi, pses, CANAS_SRV_DATA_SESSION_UNEXPECTED_RESPONSE, NULL);
        }
        break;
    case DDS_MASTER_STATE_CHECKSUM:
        if (pargs->message.data.type == CANAS_DATATYPE_CHKSUM)
        {
            const uint32_t my_checksum = _computeChecksum(pses->buffer, pses->datalen);
            // Message Code should be as in the last request message, but who cares?
            if (pargs->message.data.container.CHKSUM != my_checksum)
            {
                CANAS_TRACE("srv dds master resp: checksum mismatch\n");
                _ddsMasterDone(pi, pses, CANAS_SRV_DATA_SESSION_CHECKSUM_ERROR, NULL);
            }
            else
                _ddsMasterDone(pi, pses, CANAS_SRV_DATA_SESSION_OK, NULL);
        }
        else
        {
            CANAS_TRACE("srv dds master resp: bad data type (not checksum): %i\n", (int)pargs->message.data.type);
            _ddsMasterDone(pi, pses, CANAS_SRV_DATA_SESSION_UNEXPECTED_RESPONSE, NULL);
        }
        break;
    default:
        CANAS_TRACE("srv dds master poll: invalid state %i\n", (int)pses->state);
        _ddsMasterDone(pi, pses, CANAS_SRV_DATA_SESSION_LOCAL_ERROR, NULL);
        break;
    }
}

// ------ DDS SLAVE ------

static void _ddsSlaveDone(CanasInstance* pi, SessionEntry* pses)
{
    (void)pi;
    memset(pses, 0, sizeof(*pses));                       // Erase this entry
}

static int _ddsSlaveInit(CanasInstance* pi, CanasServiceRequestCallbackArgs* pargs, SessionEntry* pses)
{
    ServiceState* pstate = (ServiceState*)pargs->pstate;
    const uint16_t expected_datalen = _datalenByMsgcount(pargs->message.message_code);

    pses->designation.service_channel = pargs->service_channel;
    pses->update_timestamp = pargs->timestamp_usec;
    pses->rx_message_count = pargs->message.message_code;
    pses->datalen          = 0;
    pses->type             = SESSION_TYPE_DDS_SLAVE;
    pses->state            = 0;                           // No state for this type of session
    pses->memid            = pargs->message.data.container.MEMID;

    int32_t response = CANAS_SRV_DDS_RESPONSE_XON;         // Will accept all transfers by default
    if (pstate->rx_request_callback != NULL)
        response = pstate->rx_request_callback(pi, pargs->message.data.container.MEMID, expected_datalen);

    if (response == CANAS_SRV_DDS_RESPONSE_XOFF)           // Application must not care about flow control.
        response = CANAS_SRV_DDS_RESPONSE_XON;

    CanasMessage msg = pargs->message;
    msg.data.type = CANAS_DATATYPE_LONG;
    msg.data.container.LONG = response;
    int ret = canasServiceSendResponse(pi, &msg, pargs->service_channel);
    if (ret != 0)
        return ret;
    if (response != CANAS_SRV_DDS_RESPONSE_XON)
        return -CANAS_ERR_LOGIC;
    return 0;
}

static void _ddsSlavePoll(CanasInstance* pi, CanasServicePollCallbackArgs* pargs, SessionEntry* pses)
{
    ServiceState* pstate = (ServiceState*)pargs->pstate;
    const uint64_t sinceupdate = pargs->timestamp_usec - pses->update_timestamp;

    if (pses->state != 0)     // Paranoid check
    {
        CANAS_TRACE("srv dds slave poll: invalid state %i\n", (int)pses->state);
        _ddsSlaveDone(pi, pses);
        return;
    }
    if (sinceupdate > pstate->session_timeout_usec)
    {
        CANAS_TRACE("srv dds slave poll: session timeout\n");
        _ddsSlaveDone(pi, pses);
    }
}

static void _ddsSlaveRequest(CanasInstance* pi, CanasServiceRequestCallbackArgs* pargs, SessionEntry* pses)
{
    ServiceState* pstate = (ServiceState*)pargs->pstate;
    pses->update_timestamp = pargs->timestamp_usec;

    if (pargs->message.message_code != pses->next_message_code)
    {
        CANAS_TRACE("srv dds slave req: bad message code: %i expected, %i got\n",
            (int)pses->next_message_code, (int)pargs->message.message_code);
        _ddsSlaveDone(pi, pses);
        return;
    }
    pses->next_message_code++;
    if (pses->rx_message_count != 0)
        pses->rx_message_count--;            // Number of messages left to receive

    if (pargs->message.data.length < 1 ||                         // Paranoid check
        pargs->message.data.length > PAYLOAD_BYTES_PER_MESSAGE)
    {
        CANAS_TRACE("srv dds slave req: bad data len: %i\n", (int)pargs->message.data.length);
        _ddsSlaveDone(pi, pses);
        return;
    }
    memcpy(pses->buffer + pses->datalen, pargs->message.data.container.UCHAR4, pargs->message.data.length);
    pses->datalen += pargs->message.data.length;

    if (pses->rx_message_count == 0)        // Reception done
    {
        CANAS_TRACE("srv dds slave req: last chunk received, %i bytes total\n", (int)pses->datalen);
        // Send the checksum back to master, and that's it:
        CanasMessage msg = pargs->message;
        msg.data.type = CANAS_DATATYPE_CHKSUM;
        msg.data.container.CHKSUM = _computeChecksum(pses->buffer, pses->datalen);
        const int res = canasServiceSendResponse(pi, &msg, pargs->service_channel);

        if (pstate->rx_done_callback != NULL)
            pstate->rx_done_callback(pi, pses->memid, pses->buffer, pses->datalen);

        _ddsSlaveDone(pi, pses);
        if (res != 0)
            CANAS_TRACE("srv dds slave req: failed to send checksum, error %i\n", res);
    }
}

// ------ DUS MASTER ------

/*
 * MASTER  --- SURM request  -->  SLAVE
 *              100 us max
 * MASTER  <-- SURM response ---  SLAVE
 *              10 us fixed
 * MASTER  <--  Data chunks  ---  SLAVE
 *                 ***
 * MASTER  <--   Checksum    ---  SLAVE
 */

enum
{
    DUS_MASTER_STATE_SURM_PENDING, // Start Upload Request Message
    DUS_MASTER_STATE_RECEPTION     // Checksum is transmitted from SLAVE to MASTER
};

static void _dusMasterDone(CanasInstance* pi, SessionEntry* pses, CanasSrvDataSessionStatus status, int32_t* premoteerr)
{
    CanasSrvDusMasterDoneCallbackArgs cbargs;
    memset(&cbargs, 0, sizeof(cbargs));
    cbargs.status  = status;
    cbargs.node_id = pses->designation.node_id;
    cbargs.memid   = pses->memid;
    cbargs.parg    = pses->callback_arg;
    if (status == CANAS_SRV_DATA_SESSION_OK)
    {
        cbargs.datalen = pses->datalen;
        cbargs.pdata   = pses->buffer;
    }
    if (premoteerr != NULL)
        cbargs.remote_error_code = *premoteerr;
    if (pses->callback.dus_master_done != NULL)           // This callback must never be NULL, anyway
        pses->callback.dus_master_done(pi, &cbargs);
    memset(pses, 0, sizeof(*pses));                       // Erase this entry
}

static void _dusMasterPoll(CanasInstance* pi, CanasServicePollCallbackArgs* pargs, SessionEntry* pses)
{
    ServiceState* pstate = (ServiceState*)pargs->pstate;
    const uint64_t sinceupdate = pargs->timestamp_usec - pses->update_timestamp;

    switch (pses->state)
    {
    case DUS_MASTER_STATE_SURM_PENDING:
        if (sinceupdate > SDRM_SURM_TIMEOUT_USEC)
        {
            CANAS_TRACE("srv dus master poll: SURM timeout\n");
            _dusMasterDone(pi, pses, CANAS_SRV_DATA_SESSION_TIMEOUT, NULL);
        }
        break;
    case DUS_MASTER_STATE_RECEPTION:
        if (sinceupdate > pstate->session_timeout_usec)
        {
            CANAS_TRACE("srv dus master poll: checksum or reception timeout\n");
            _dusMasterDone(pi, pses, CANAS_SRV_DATA_SESSION_TIMEOUT, NULL);
        }
        break;
    default:
        CANAS_TRACE("srv dus master poll: invalid state %i\n", (int)pses->state);
        _dusMasterDone(pi, pses, CANAS_SRV_DATA_SESSION_LOCAL_ERROR, NULL);
        break;
    }
}

static void _dusMasterResponse(CanasInstance* pi, CanasServiceResponseCallbackArgs* pargs, SessionEntry* pses)
{
    pses->update_timestamp = pargs->timestamp_usec;

    switch (pses->state)
    {
    case DUS_MASTER_STATE_SURM_PENDING:
        if (pargs->message.data.type == CANAS_DATATYPE_LONG)
        {
            if (pargs->message.data.container.LONG == CANAS_SRV_DUS_RESPONSE_OK)
            {
                pses->state = DUS_MASTER_STATE_RECEPTION;
            }
            else
            {
                CANAS_TRACE("srv dus master resp: remote error: %li\n", (long)pargs->message.data.container.LONG);
                _dusMasterDone(pi, pses, CANAS_SRV_DATA_SESSION_REMOTE_ERROR, &pargs->message.data.container.LONG);
            }
        }
        else
        {
            CANAS_TRACE("srv dus master resp: bad data type (not long): %i\n", (int)pargs->message.data.type);
            _dusMasterDone(pi, pses, CANAS_SRV_DATA_SESSION_UNEXPECTED_RESPONSE, NULL);
        }
        break;
        // We don't care how many messages were expected and how many of them were actually received.
        // Instead, we terminate transfer when checksum received.
    case DUS_MASTER_STATE_RECEPTION:
        if (pargs->message.data.type == CANAS_DATATYPE_CHKSUM)
        {
            if (pargs->message.message_code == pses->next_message_code - 1)
            {
                const uint32_t my_checksum = _computeChecksum(pses->buffer, pses->datalen);
                if (pargs->message.data.container.CHKSUM != my_checksum)
                {
                    CANAS_TRACE("srv dus master resp: checksum mismatch\n");
                    _dusMasterDone(pi, pses, CANAS_SRV_DATA_SESSION_CHECKSUM_ERROR, NULL);
                }
                else
                    _dusMasterDone(pi, pses, CANAS_SRV_DATA_SESSION_OK, NULL);
            }
            else
            {
                CANAS_TRACE("srv dus master resp: checksum message has wrong msgcode\n");
                _dusMasterDone(pi, pses, CANAS_SRV_DATA_SESSION_UNEXPECTED_RESPONSE, NULL);
            }
        }
        else
        {
            if (pargs->message.message_code == pses->next_message_code)
            {
                pses->next_message_code++;
                memcpy(pses->buffer + pses->datalen, pargs->message.data.container.UCHAR4,
                       pargs->message.data.length);
                pses->datalen += pargs->message.data.length;
                // Since the message sequence length is not checked, we must check the data length instead:
                if (pses->datalen > CANAS_SRV_DATA_MAX_PAYLOAD_LEN)
                {
                    CANAS_TRACE("srv dus master resp: too many bytes received; msgcode: %i\n",
                        (int)pargs->message.message_code);
                    _dusMasterDone(pi, pses, CANAS_SRV_DATA_SESSION_UNEXPECTED_RESPONSE, NULL);
                }
                /*
                 * Well, the remote node can transmit more than 255 messages, and we will not
                 * detect that until the overall data length has not exceeded 1020 bytes.
                 */
            }
            else
            {
                CANAS_TRACE("srv dus master resp: bad message code: %i expected, %i got\n",
                    (int)pses->next_message_code, (int)pargs->message.message_code);
                _dusMasterDone(pi, pses, CANAS_SRV_DATA_SESSION_UNEXPECTED_RESPONSE, NULL);
            }
        }
        break;
    default:
        CANAS_TRACE("srv dus master resp: invalid state %i\n", (int)pses->state);
        _dusMasterDone(pi, pses, CANAS_SRV_DATA_SESSION_LOCAL_ERROR, NULL);
        break;
    }
}

// ------ DUS SLAVE ------

enum
{
    DUS_SLAVE_STATE_INITIAL_DELAY,
    DUS_SLAVE_STATE_TRANSMISSION,
    DUS_SLAVE_STATE_CHECKSUM
};

static void _dusSlaveDone(CanasInstance* pi, SessionEntry* pses)
{
    (void)pi;
    memset(pses, 0, sizeof(*pses));                       // Erase this entry
}

static int _dusSlaveTransmitNextChunk(CanasInstance* pi, SessionEntry* pses, bool* plast_chunk)
{
    CanasMessage msg;
    memset(&msg, 0, sizeof(msg));
    msg.node_id      = pi->config.node_id;             // Response must contain Node ID of the transmitting node
    msg.service_code = SERVICE_CODE_DUS;
    msg.message_code = pses->next_message_code++;
    msg.data         = _makeDataChunk(pses, msg.message_code, plast_chunk);
    return canasServiceSendResponse(pi, &msg, pses->designation.service_channel);
}

static int _dusSlaveInit(CanasInstance* pi, CanasServiceRequestCallbackArgs* pargs, SessionEntry* pses)
{
    ServiceState* pstate = (ServiceState*)pargs->pstate;
    const uint16_t expected_datalen = _datalenByMsgcount(pargs->message.message_code);

    pses->designation.service_channel = pargs->service_channel;
    pses->update_timestamp = pargs->timestamp_usec;
    pses->datalen          = expected_datalen;
    pses->type             = SESSION_TYPE_DUS_SLAVE;
    pses->state            = DUS_SLAVE_STATE_INITIAL_DELAY;
    pses->memid            = pargs->message.data.container.MEMID;

    int32_t response = CANAS_SRV_DUS_RESPONSE_ABORT;
    if (pstate->tx_request_callback == NULL)
    {
        CANAS_TRACE("srv dus init slave: no callback, abort\n");
    }
    else
    {
        response = pstate->tx_request_callback(pi, pargs->message.data.container.MEMID, expected_datalen, pses->buffer,
            &pses->datalen);
        if (pses->datalen < 1 || pses->datalen > CANAS_SRV_DATA_MAX_PAYLOAD_LEN)
        {
            CANAS_TRACE("srv dus init slave: invalid data len, abort\n");
            response = CANAS_SRV_DUS_RESPONSE_ABORT;
        }
    }

    CanasMessage msg = pargs->message;
    msg.data.type = CANAS_DATATYPE_LONG;
    msg.data.container.LONG = response;
    int ret = canasServiceSendResponse(pi, &msg, pargs->service_channel);
    if (ret != 0)
        return ret;
    if (response != CANAS_SRV_DUS_RESPONSE_OK)
        return -CANAS_ERR_LOGIC;
    return 0;
}

static void _dusSlavePoll(CanasInstance* pi, CanasServicePollCallbackArgs* pargs, SessionEntry* pses)
{
    ServiceState* pstate = (ServiceState*)pargs->pstate;
    const uint64_t sinceupdate = pargs->timestamp_usec - pses->update_timestamp;

    switch (pses->state)
    {
    case DUS_SLAVE_STATE_INITIAL_DELAY:
        if (sinceupdate < DUS_SLAVE_INITIAL_DELAY_USEC)
            break;
        /* no break */
    case DUS_SLAVE_STATE_TRANSMISSION:
    {
        if (pses->state != DUS_SLAVE_STATE_TRANSMISSION)
            pses->state = DUS_SLAVE_STATE_TRANSMISSION;
        else if (sinceupdate < pstate->tx_interval_usec)
            break;
        pses->update_timestamp = pargs->timestamp_usec;
        bool last_chunk = false;
        int res = _dusSlaveTransmitNextChunk(pi, pses, &last_chunk);
        if (res != 0)
        {
            CANAS_TRACE("srv dus slave poll: transmission failure: %i\n", res);
            _dusSlaveDone(pi, pses);
        }
        else if (last_chunk)
        {
            CANAS_TRACE("srv dus slave poll: last chunk has been sent\n");
            pses->state = DUS_SLAVE_STATE_CHECKSUM;
        }
        break;
    }
    case DUS_SLAVE_STATE_CHECKSUM:
    {
        if (sinceupdate < pstate->tx_interval_usec)
            break;
        CanasMessage msg;
        memset(&msg, 0, sizeof(msg));
        msg.node_id      = pi->config.node_id;   // Self Node ID
        msg.service_code = SERVICE_CODE_DUS;
        msg.message_code = pses->next_message_code - 1;    // Use the last transmitted Message Code
        msg.data.type    = CANAS_DATATYPE_CHKSUM;
        msg.data.container.CHKSUM = _computeChecksum(pses->buffer, pses->datalen);
        int res = canasServiceSendResponse(pi, &msg, pses->designation.service_channel);
        _dusSlaveDone(pi, pses);
        if (res != 0)
            CANAS_TRACE("srv dus slave poll: failed to send checksum: %i\n", res);
        break;
    }
    default:
        CANAS_TRACE("srv dus slave poll: invalid state %i\n", (int)pses->state);
        _dusSlaveDone(pi, pses);
        break;
    }
}

static void _dusSlaveRequest(CanasInstance* pi, CanasServiceRequestCallbackArgs* pargs, SessionEntry* pses)
{
    (void)pi;
    (void)pargs;
    // DUS Slave should never receive messages, except SURM request
    CANAS_TRACE("srv dus slave req: no requests allowed; datatype=%i, msgcode=%i\n",
        (int)pargs->message.data.type, (int)pargs->message.message_code);
    _dusSlaveDone(pi, pses);
}

// ---------------

static void (*_poll_handlers[])(CanasInstance*, CanasServicePollCallbackArgs*, SessionEntry*) =
{
    NULL,
    _ddsMasterPoll,
    _ddsSlavePoll,
    _dusMasterPoll,
    _dusSlavePoll
};

static SessionEntry* _allocateSession(ServiceState* pstate)
{
    for (int i = 0; i < pstate->entry_count; i++)
    {
        if (pstate->entries[i].type == SESSION_TYPE_NONE)
            return pstate->entries + i;
    }
    return NULL;
}

static void _initSlaveSession(CanasInstance* pi, CanasServiceRequestCallbackArgs* pargs)
{
    if (pargs->message.data.type != CANAS_DATATYPE_MEMID)
    {
        CANAS_TRACE("srv data init slave: wrong data type in request: %i\n", (int)pargs->message.data.type);
        return;
    }

    ServiceState* pstate = (ServiceState*)pargs->pstate;

    CanasMessage msg = pargs->message;
    SessionEntry* pnewses = _allocateSession(pstate);
    if (pnewses == NULL)
    {
        CANAS_TRACE("srv data init slave: no free entries, abort\n");
        if (pargs->message.service_code == SERVICE_CODE_DDS ||
            pargs->message.service_code == SERVICE_CODE_DUS)
        {
            msg.data.type = CANAS_DATATYPE_LONG;
            if (pargs->message.service_code == SERVICE_CODE_DDS)
                msg.data.container.LONG = CANAS_SRV_DDS_RESPONSE_ABORT;
            else
                msg.data.container.LONG = CANAS_SRV_DUS_RESPONSE_ABORT;
            const int response_result = canasServiceSendResponse(pi, &msg, pargs->service_channel);
            if (response_result != 0)
                CANAS_TRACE("srv data init slave: failed to send the abort response: %i\n", response_result);
        }
        return;
    }
    memset(pnewses, 0, sizeof(*pnewses));

    int result = -CANAS_ERR_LOGIC;
    if (pargs->message.service_code == SERVICE_CODE_DDS)
        result = _ddsSlaveInit(pi, pargs, pnewses);
    else if (pargs->message.service_code == SERVICE_CODE_DUS)
        result = _dusSlaveInit(pi, pargs, pnewses);
    else
        CANAS_TRACE("srv data init slave: wtf service code %i\n", (int)pargs->message.service_code);

    if (result != 0)
    {
        CANAS_TRACE("srv data init slave: srv %i failed with error %i\n", (int)pargs->message.service_code, result);
        memset(pnewses, 0, sizeof(*pnewses));
    }
}

static void _poll(CanasInstance* pi, CanasServicePollCallbackArgs* pargs)
{
    ServiceState* pstate = (ServiceState*)pargs->pstate;
    if (pstate == NULL)
    {
        CANAS_TRACE("srv data poll: invalid state pointer\n");
        return;
    }
    for (int i = 0; i < pstate->entry_count; i++)
    {
        uint8_t type = pstate->entries[i].type;
        if (type == SESSION_TYPE_NONE)
            continue;
        if (type >= SESSION_TYPE_BOUND_)
        {
            CANAS_TRACE("srv data poll: invalid entry type: %i\n", (int)type);
            continue;
        }
        _poll_handlers[type](pi, pargs, pstate->entries + i);
    }
}

static void _response(CanasInstance* pi, CanasServiceResponseCallbackArgs* pargs)
{
    ServiceState* pstate = (ServiceState*)pargs->pstate;
    if (pstate == NULL)
    {
        CANAS_TRACE("srv data resp: invalid state pointer\n");
        return;
    }
    for (int i = 0; i < pstate->entry_count; i++)
    {
        SessionEntry* pses = pstate->entries + i;
        if (pses->designation.node_id != pargs->message.node_id)   // Looking for a corresponding Node ID
            continue;
        if (pargs->message.service_code == SERVICE_CODE_DDS && pses->type == SESSION_TYPE_DDS_MASTER)
        {
            _ddsMasterResponse(pi, pargs, pses);
            return;
        }
        if (pargs->message.service_code == SERVICE_CODE_DUS && pses->type == SESSION_TYPE_DUS_MASTER)
        {
            _dusMasterResponse(pi, pargs, pses);
            return;
        }
    }
    /*
     * It is not possible to initiate a new session by response message.
     * All we can do is complain about that:
     */
    CANAS_TRACE("srv data resp: unmatched message from %i\n", (int)pargs->message.node_id);
}

static void _request(CanasInstance* pi, CanasServiceRequestCallbackArgs* pargs)
{
    ServiceState* pstate = (ServiceState*)pargs->pstate;
    if (pstate == NULL)
    {
        CANAS_TRACE("srv data req: invalid state pointer\n");
        return;
    }
    for (int i = 0; i < pstate->entry_count; i++)
    {
        SessionEntry* pses = pstate->entries + i;
        if (pses->designation.service_channel != pargs->service_channel) // Looking for a corresponding Service Channel
            continue;
        if (pargs->message.service_code == SERVICE_CODE_DDS && pses->type == SESSION_TYPE_DDS_SLAVE)
        {
            _ddsSlaveRequest(pi, pargs, pses);
            return;
        }
        if (pargs->message.service_code == SERVICE_CODE_DUS && pses->type == SESSION_TYPE_DUS_SLAVE)
        {
            _dusSlaveRequest(pi, pargs, pses);
            return;
        }
    }
    CANAS_TRACE("srv data req: new slave session from %i of type %i\n",
                (int)pargs->message.node_id, (int)pargs->message.service_code);
    _initSlaveSession(pi, pargs);
}

int canasSrvDataInit(CanasInstance* pi, uint8_t max_active_sessions,
                     CanasSrvDdsSlaveRequestCallback rx_request_callback,
                     CanasSrvDdsSlaveDoneCallback rx_done_callback,
                     CanasSrvDusSlaveRequestCallback tx_request_callback)
{
    if (pi == NULL || max_active_sessions < 1)
        return -CANAS_ERR_ARGUMENT;
    if ((rx_done_callback == NULL) != (rx_request_callback == NULL))
        return -CANAS_ERR_ARGUMENT;

    const bool need_dds = rx_done_callback != NULL;
    const bool need_dus = tx_request_callback != NULL;
    if (!(need_dus || need_dds))
        return -CANAS_ERR_ARGUMENT;
    /*
     * Both DDS and DUS share the same state
     */
    int size = sizeof(ServiceState) + sizeof(SessionEntry) * max_active_sessions;
    ServiceState* ps = canasMalloc(pi, size);
    if (ps == NULL)
        return -CANAS_ERR_NOT_ENOUGH_MEMORY;
    memset(ps, 0, size);

    ps->entry_count          = max_active_sessions;
    ps->tx_interval_usec     = DEFAULT_TX_INTERVAL_USEC;
    ps->session_timeout_usec = DEFAULT_SESSION_TIMEOUT_USEC;

    ps->rx_request_callback = rx_request_callback;
    ps->rx_done_callback    = rx_done_callback;
    ps->tx_request_callback = tx_request_callback;

    int ret = 0;
    if (need_dds)
    {
        ret = canasServiceRegister(pi, SERVICE_CODE_DDS, _poll, _request, _response, ps);
        if (ret != 0)
            goto error_cleanup;
    }
    if (need_dus)
    {
        ret = canasServiceRegister(pi, SERVICE_CODE_DUS, _poll, _request, _response, ps);
        if (ret != 0)
            goto error_cleanup;
    }
    return 0;

    error_cleanup:
    canasFree(pi, ps);
    canasServiceUnregister(pi, SERVICE_CODE_DDS);
    canasServiceUnregister(pi, SERVICE_CODE_DUS);
    return ret;
}

int canasSrvDataOverrideDefaults(CanasInstance* pi, uint32_t* ptx_interval_usec, uint32_t* psession_timeout_usec)
{
    if (pi == NULL)
        return -CANAS_ERR_ARGUMENT;

    ServiceState* pstate = NULL;
    int ret = canasServiceGetState(pi, SERVICE_CODE_DDS, (void**)&pstate);
    if (ret != 0)
        return ret;
    if (pstate == NULL)
        return -CANAS_ERR_LOGIC;

    if (ptx_interval_usec != NULL)
        pstate->tx_interval_usec = *ptx_interval_usec;

    if (psession_timeout_usec != NULL)
        pstate->session_timeout_usec = *psession_timeout_usec;

    return 0;
}

int canasSrvDdsDownloadTo(CanasInstance* pi, uint8_t node_id, uint32_t memid, const void* pdata, uint16_t datalen,
                          CanasSrvDdsMasterDoneCallback callback, void* callback_arg)
{
    if (pi == NULL || callback == NULL || pdata == NULL || datalen < 1 || datalen > CANAS_SRV_DATA_MAX_PAYLOAD_LEN)
        return -CANAS_ERR_ARGUMENT;
    if (node_id == CANAS_BROADCAST_NODE_ID)
        return -CANAS_ERR_BAD_NODE_ID;

    ServiceState* pstate = NULL;
    int ret = canasServiceGetState(pi, SERVICE_CODE_DDS, (void**)&pstate);
    if (ret != 0)
        return ret;
    if (pstate == NULL)
        return -CANAS_ERR_LOGIC;

    // Make sure that there is no other sessions active with the same node:
    for (int i = 0; i < pstate->entry_count; i++)
    {
        SessionEntry* pses = pstate->entries + i;
        if (pses->type == SESSION_TYPE_DDS_MASTER && pses->designation.node_id == node_id)
            return -CANAS_ERR_ENTRY_EXISTS;
    }

    // Find block to store the session context:
    SessionEntry* pses = _allocateSession(pstate);
    if (pses == NULL)
        return -CANAS_ERR_QUOTA_EXCEEDED;

    // Send SDRM request:
    CanasMessage msg;
    msg.service_code = SERVICE_CODE_DDS;
    msg.message_code = _msgcountByDatalen(datalen);
    msg.node_id      = node_id;
    msg.data.type    = CANAS_DATATYPE_MEMID;
    msg.data.container.MEMID = memid;
    ret = canasServiceSendRequest(pi, &msg);
    if (ret != 0)
        return ret;

    // Init the session context:
    memset(pses, 0, sizeof(*pses));
    memcpy(pses->buffer, pdata, datalen);
    pses->type                     = SESSION_TYPE_DDS_MASTER;
    pses->designation.node_id      = node_id;
    pses->datalen                  = datalen;
    pses->state                    = DDS_MASTER_STATE_SDRM_PENDING;
    pses->update_timestamp         = canasTimestamp(pi);
    pses->callback.dds_master_done = callback;
    pses->callback_arg             = callback_arg;
    pses->memid                    = memid;
    return 0;
}

int canasSrvDusUploadFrom(CanasInstance* pi, uint8_t node_id, uint32_t memid, uint16_t expected_datalen,
                          CanasSrvDusMasterDoneCallback callback, void* callback_arg)
{
    // Note that expected_datalen may be zero.
    if (pi == NULL || callback == NULL || expected_datalen > CANAS_SRV_DATA_MAX_PAYLOAD_LEN)
        return -CANAS_ERR_ARGUMENT;
    if (node_id == CANAS_BROADCAST_NODE_ID)
        return -CANAS_ERR_BAD_NODE_ID;

    ServiceState* pstate = NULL;
    int ret = canasServiceGetState(pi, SERVICE_CODE_DUS, (void**)&pstate);
    if (ret != 0)
        return ret;
    if (pstate == NULL)
        return -CANAS_ERR_LOGIC;

    // Make sure that there is no other sessions active with the same node:
    for (int i = 0; i < pstate->entry_count; i++)
    {
        SessionEntry* pses = pstate->entries + i;
        if (pses->type == SESSION_TYPE_DUS_MASTER && pses->designation.node_id == node_id)
            return -CANAS_ERR_ENTRY_EXISTS;
    }

    // Find block to store the session context:
    SessionEntry* pses = _allocateSession(pstate);
    if (pses == NULL)
        return -CANAS_ERR_QUOTA_EXCEEDED;

    const uint8_t message_count = _msgcountByDatalen(expected_datalen);

    // Send SURM request:
    CanasMessage msg;
    msg.service_code = SERVICE_CODE_DUS;
    msg.message_code = message_count;
    msg.node_id      = node_id;
    msg.data.type    = CANAS_DATATYPE_MEMID;
    msg.data.container.MEMID = memid;
    ret = canasServiceSendRequest(pi, &msg);
    if (ret != 0)
        return ret;

    // Init the session context:
    memset(pses, 0, sizeof(*pses));
    pses->type                     = SESSION_TYPE_DUS_MASTER;
    pses->designation.node_id      = node_id;
    pses->state                    = DUS_MASTER_STATE_SURM_PENDING;
    pses->update_timestamp         = canasTimestamp(pi);
    pses->callback.dus_master_done = callback;
    pses->callback_arg             = callback_arg;
    pses->memid                    = memid;
    /*
     * rx_message_count is not used here because incoming data transfer will be terminated by the
     * CHKSUM message from the transmitting node.
     */
    return 0;
}
