/*
 * CANaerospace Node Service Protocol support
 * Pavel Kirienko, 2013 (pavel.kirienko@gmail.com)
 */

#include <stdlib.h>
#include <string.h>
#include "debug.h"
#include "service.h"

#ifdef __GNUC__
// RANGEINCLUSIVE() may produce a lot of these warnings
#  pragma GCC diagnostic ignored "-Wtype-limits"
#endif

#define RANGEINCLUSIVE(x, min, max) ((x) >= (min) && (x) <= (max))

static int _serviceChannelFromMessageID(uint16_t msg_id, bool* pisrequest)
{
    static const uint16_t REQ_MASK = ~(uint16_t)1;
    if (RANGEINCLUSIVE(msg_id, CANAS_MSGTYPE_NODE_SERVICE_HIGH_MIN, CANAS_MSGTYPE_NODE_SERVICE_HIGH_MAX))
    {
        int srvchan = ((msg_id & REQ_MASK) - 128) / 2;
        *pisrequest = !(msg_id & 1);
        return srvchan + CANAS_SERVICE_CHANNEL_HIGH_MIN;
    }
    if (RANGEINCLUSIVE(msg_id, CANAS_MSGTYPE_NODE_SERVICE_LOW_MIN, CANAS_MSGTYPE_NODE_SERVICE_LOW_MAX))
    {
        int srvchan = ((msg_id & REQ_MASK) - 2000) / 2;
        *pisrequest = !(msg_id & 1);
        return srvchan + CANAS_SERVICE_CHANNEL_LOW_MIN;
    }
    return -CANAS_ERR_BAD_MESSAGE_ID;
}

int canasServiceChannelToMessageID(uint8_t service_channel, bool isrequest)
{
    if (RANGEINCLUSIVE(service_channel, CANAS_SERVICE_CHANNEL_HIGH_MIN, CANAS_SERVICE_CHANNEL_HIGH_MAX))
    {
        int ret = 128 + service_channel * 2;
        if (!isrequest)
            ret++;
        return ret;
    }
    if (RANGEINCLUSIVE(service_channel, CANAS_SERVICE_CHANNEL_LOW_MIN,  CANAS_SERVICE_CHANNEL_LOW_MAX))
    {
        service_channel -= CANAS_SERVICE_CHANNEL_LOW_MIN;
        int ret = 2000 + service_channel * 2;
        if (!isrequest)
            ret++;
        return ret;
    }
    return -CANAS_ERR_BAD_SERVICE_CHAN;
}

static void _issueRequestCallback(CanasInstance* pi, CanasServiceSubscription* psrv, CanasMessage* pmsg,
                                  uint8_t service_channel, uint64_t timestamp_usec)
{
    if (psrv->callback_request == NULL)
        return;

    CanasServiceRequestCallbackArgs args;
    memset(&args, 0, sizeof(args));
    args.message = *pmsg;
    args.pstate = psrv->pstate;
    args.service_channel = (uint8_t)service_channel;
    args.timestamp_usec = timestamp_usec;

    psrv->callback_request(pi, &args);
}

static void _issueResponseCallback(CanasInstance* pi, CanasServiceSubscription* psrv, CanasMessage* pmsg,
                                   uint64_t timestamp_usec)
{
    if (psrv->callback_response == NULL)
        return;

    CanasServiceResponseCallbackArgs args;
    memset(&args, 0, sizeof(args));
    args.message = *pmsg;
    args.pstate = psrv->pstate;
    args.timestamp_usec = timestamp_usec;

    psrv->callback_response(pi, &args);
}

static bool _applyFilters(CanasInstance* pi, uint8_t service_channel, bool is_service_request, CanasMessage* pmsg)
{
    if (is_service_request)
    {
        // We should accept incoming requests only if Node ID matches
        // Service Channel collision is allowed by the standard
        if (pmsg->node_id != pi->config.node_id &&
            pmsg->node_id != 0)                       // 0 is Broadcast Node ID, must accept it too
        {
            CANAS_TRACE("serv req: foreign request: srvch=%i srvcode=%i nodeid=%i\n", (int)service_channel,
                (int)pmsg->service_code, (int)pmsg->node_id);
            return false;
        }
    }
    else
    {
        // We can receive Service Responses only with our own Service Channel ID, and any Node ID except our own
        if (service_channel != pi->config.service_channel)
        {
            CANAS_TRACE("serv resp: foreign response: srvch=%i srvcode=%i\n", (int)service_channel, (int)pmsg->service_code);
            return false;
        }
        if (pmsg->node_id == pi->config.node_id)
        {
            CANAS_TRACE("serv resp: node id collision: srvch=%i srvcode=%i\n", (int)service_channel,
                (int)pmsg->service_code);
            return false;
        }
    }
    return true;
}

bool canasIsInterestingServiceMessage(CanasInstance* pi, uint16_t msg_id, CanasMessage *pmsg)
{
    bool is_service_request = false;
    int service_channel = _serviceChannelFromMessageID(msg_id, &is_service_request);
    if (service_channel < 0 || service_channel > 0xFF)
    {
        CANAS_TRACE("serv bad channel: msgid=%03x srvch=%i isreq=%i\n", (unsigned int)msg_id, (int)service_channel,
            (int)is_service_request);
        return false;
    }
    return _applyFilters(pi, service_channel, is_service_request, pmsg);
}

void canasHandleReceivedService(CanasInstance* pi, CanasServiceSubscription* psrv, uint8_t iface, uint16_t msg_id,
                                CanasMessage* pmsg, const CanasCanFrame* pframe, uint64_t timestamp_usec)
{
    bool is_service_request = false;
    int service_channel = _serviceChannelFromMessageID(msg_id, &is_service_request);

    uint32_t header = 0;
    memcpy(&header, pframe->data, 4);  // Byte order doesn't matter.

    int oldest = 0;
    // Ugly but works.
    for (int i = 0; i < psrv->history_len; i++)
    {
        CanasServiceFrameHistoryEntry* ph = psrv->history + i;
        if (ph->timestamp_usec < psrv->history[oldest].timestamp_usec)
            oldest = i;
        if (ph->timestamp_usec == 0)                                          // Empty entry, no match
            continue;
        if (ph->header != header)
            continue;
        if ((timestamp_usec - ph->timestamp_usec) > pi->config.repeat_timeout_usec)
            continue;
        if (ph->ifaces_mask & (1 << iface))                                   // Interface matched - no repetition
            continue;
        // well, it is repetition
        ph->ifaces_mask |= 1 << iface;                                        // Mark bit of this iface and that's it.
        CANAS_TRACE("serv rep msgid=%03x ifmask=%02x srvcode=%i\n", (unsigned int)msg_id,
                    (unsigned int)ph->ifaces_mask, (int)psrv->service_code);
        return;
    }
    if (psrv->history_len > 0)
    {
        CanasServiceFrameHistoryEntry* ph = psrv->history + oldest;
        ph->header = header;
        ph->ifaces_mask = 1 << iface;
        ph->timestamp_usec = timestamp_usec;
    }

    if (is_service_request)
        _issueRequestCallback(pi, psrv, pmsg, (uint8_t)service_channel, timestamp_usec);
    else
        _issueResponseCallback(pi, psrv, pmsg, timestamp_usec);
}

void canasPollServices(CanasInstance* pi, uint64_t timestamp_usec)
{
    if ((timestamp_usec - pi->last_service_ts) < pi->config.service_poll_interval_usec)
        return;
    pi->last_service_ts = timestamp_usec;                          // It is time to poll services
    CanasServiceSubscription* psrv = pi->pservice_subs;
    CanasServicePollCallbackArgs args;
    memset(&args, 0, sizeof(args));
    while (psrv)
    {
        if (psrv->callback_poll != NULL)
        {
            args.pstate = psrv->pstate;
            args.timestamp_usec = timestamp_usec;
            psrv->callback_poll(pi, &args);
        }
        psrv = psrv->pnext;
    }
}

bool canasIsValidServiceChannel(uint8_t service_channel)
{
    return
        RANGEINCLUSIVE(service_channel, CANAS_SERVICE_CHANNEL_HIGH_MIN, CANAS_SERVICE_CHANNEL_HIGH_MAX) ||
        RANGEINCLUSIVE(service_channel, CANAS_SERVICE_CHANNEL_LOW_MIN,  CANAS_SERVICE_CHANNEL_LOW_MAX);
}
