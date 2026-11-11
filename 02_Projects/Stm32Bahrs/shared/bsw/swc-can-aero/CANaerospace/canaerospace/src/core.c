/*
 * CANaerospace main logic
 * Pavel Kirienko, 2013 (pavel.kirienko@gmail.com)
 */

#include <string.h>
#include <stdbool.h>
#include <canaerospace/canaerospace.h>
#include "service.h"
#include "marshal.h"
#include "debug.h"
#include "list.h"

static const int CANAS_DEFAULT_REPEAT_TIMEOUT_USEC = 30 * 1000 * 1000;

#ifdef __GNUC__
// RANGEINCLUSIVE() may produce a lot of these warnings
#  pragma GCC diagnostic ignored "-Wtype-limits"
#endif

#define ALL_IFACES  -1

#define REDUND_CHAN_MULT 65536ul

#define RANGEINCLUSIVE(x, min, max) ((x) >= (min) && (x) <= (max))

typedef enum
{
    MSGGROUP_WTF,
    MSGGROUP_PARAMETER,
    MSGGROUP_SERVICE
} MessageGroup;

static MessageGroup _detectMessageGroup(uint16_t id)
{
    if (RANGEINCLUSIVE(id, CANAS_MSGTYPE_EMERGENCY_EVENT_MIN, CANAS_MSGTYPE_EMERGENCY_EVENT_MAX))
        return MSGGROUP_PARAMETER;

    if (RANGEINCLUSIVE(id, CANAS_MSGTYPE_NODE_SERVICE_HIGH_MIN, CANAS_MSGTYPE_NODE_SERVICE_HIGH_MAX))
        return MSGGROUP_SERVICE;

    if (RANGEINCLUSIVE(id, CANAS_MSGTYPE_USER_DEFINED_HIGH_MIN, CANAS_MSGTYPE_USER_DEFINED_HIGH_MAX))
        return MSGGROUP_PARAMETER;

    if (RANGEINCLUSIVE(id, CANAS_MSGTYPE_NORMAL_OPERATION_MIN, CANAS_MSGTYPE_NORMAL_OPERATION_MAX))
        return MSGGROUP_PARAMETER;

    if (RANGEINCLUSIVE(id, CANAS_MSGTYPE_USER_DEFINED_LOW_MIN, CANAS_MSGTYPE_USER_DEFINED_LOW_MAX))
        return MSGGROUP_PARAMETER;

    if (RANGEINCLUSIVE(id, CANAS_MSGTYPE_DEBUG_SERVICE_MIN, CANAS_MSGTYPE_DEBUG_SERVICE_MAX))
        return MSGGROUP_PARAMETER;

    if (RANGEINCLUSIVE(id, CANAS_MSGTYPE_NODE_SERVICE_LOW_MIN, CANAS_MSGTYPE_NODE_SERVICE_LOW_MAX))
        return MSGGROUP_SERVICE;

    CANAS_TRACE("msggroup: failed to detect, msgid=%03x\n", (unsigned int)id);
    return MSGGROUP_WTF;
}

static bool _isConfigOk(const CanasConfig* pcfg)
{
    if (pcfg->fn_send == NULL)
        return false;

    if (pcfg->fn_malloc == NULL)
        return false;

    if (pcfg->fn_timestamp == NULL)
        return false;

    if (!canasIsValidServiceChannel(pcfg->service_channel))
        return false;

    if (pcfg->iface_count < 1 || pcfg->iface_count > CANAS_IFACE_COUNT_MAX)
        return false;

    if (pcfg->service_poll_interval_usec < 1)
        return false;

    if ((pcfg->fn_filter == NULL) ^ (pcfg->filters_per_iface == 0))
        return false;

    if (pcfg->node_id == CANAS_BROADCAST_NODE_ID)
        return false;

    if (pcfg->service_request_timeout_usec < 1)
        return false;

    return true;
}

/// positive if a > b
static int _diffU8(uint8_t a, uint8_t b)
{
    const int d = a - b;
    if (d <= -128)
        return 256 + d;
    else if (d >= 127)
        return d - 256;
    return d;
}

static void _issueMessageHookCallback(CanasInstance* pi, int iface, uint16_t msg_id, CanasMessage* pmsg,
                                      uint8_t redund_ch, uint64_t timestamp_usec)
{
    if (pi->config.fn_hook == NULL)
        return;

    CanasHookCallbackArgs args;
    memset(&args, 0, sizeof(args));
    args.iface = (uint8_t)iface;
    args.message = *pmsg;
    args.message_id = msg_id;
    args.redund_channel_id = redund_ch;
    args.timestamp_usec = timestamp_usec;

    pi->config.fn_hook(pi, &args);
}

static void _handleReceivedParam(CanasInstance* pi, CanasParamSubscription* ppar, uint16_t msg_id, CanasMessage* pmsg,
                                 uint8_t redund_ch, uint64_t timestamp_usec)
{
    if (redund_ch >= ppar->redund_count)      // We have no buffer for this redundancy channel.
        return;                               // Sadface.

    // Timestamp in redund cache is initialized to zero:
    if (ppar->redund_cache[redund_ch].timestamp_usec > 0 &&
        (timestamp_usec - ppar->redund_cache[redund_ch].timestamp_usec) < pi->config.repeat_timeout_usec)
    {
        int msgcode_diff = _diffU8(pmsg->message_code, ppar->redund_cache[redund_ch].message.message_code);
        // msgcode_diff == 0 means that we've got exactly the same message as before, so it needs to be skipped too.
        if (msgcode_diff <= 0)
        {
            CANAS_TRACE("param rep msgid=%03x redund=%i msgcode=%i usecago=%u\n",(unsigned int)msg_id, (int)redund_ch,
                (int)pmsg->message_code, (unsigned int)(timestamp_usec - ppar->redund_cache[redund_ch].timestamp_usec));
            return;                           // It's repeated message
        }
    }
    ppar->redund_cache[redund_ch].message = *pmsg;       // Save the whole message. Redundantly, but simple.
    ppar->redund_cache[redund_ch].timestamp_usec = timestamp_usec;
    if (ppar->callback != NULL)
    {
        CanasParamCallbackArgs args;
        memset(&args, 0, sizeof(args));
        args.message = *pmsg;
        args.message_id = msg_id;
        args.parg = ppar->callback_arg;
        args.redund_channel_id = redund_ch;
        args.timestamp_usec = timestamp_usec;

        ppar->callback(pi, &args);
    }
}

static int _parseFrame(const CanasCanFrame* pframe, uint16_t* pmsg_id, CanasMessage* pmsg, uint8_t* predund_chan)
{
    if (pframe->dlc < 4 || pframe->dlc > 8)
    {
        CANAS_TRACE("frameparser: bad dlc=%i\n", (int)pframe->dlc);
        return -CANAS_ERR_BAD_CAN_FRAME;
    }
    if (pframe->id & CANAS_CAN_FLAG_RTR)
    {
        CANAS_TRACE("frameparser: RTR flag is not allowed\n");
        return -CANAS_ERR_BAD_CAN_FRAME;
    }

    *pmsg_id = pframe->id & CANAS_CAN_MASK_STDID;

    uint16_t redundancy_ch_id_raw = 0;
    if (pframe->id & CANAS_CAN_FLAG_EFF)      // Redundancy channel ID is supplied only on extended frames
    {
        redundancy_ch_id_raw = (pframe->id & CANAS_CAN_MASK_EXTID) / REDUND_CHAN_MULT;
        if (redundancy_ch_id_raw > 0xFF)
        {
            CANAS_TRACE("frameparser: bad redund=%i\n", (int)redundancy_ch_id_raw);
            return -CANAS_ERR_BAD_REDUND_CHAN;
        }
    }
    *predund_chan = (uint8_t)redundancy_ch_id_raw;

    memset(pmsg, 0, sizeof(*pmsg));
    pmsg->node_id      = pframe->data[0];
    pmsg->service_code = pframe->data[2];
    pmsg->message_code = pframe->data[3];

    int ret = canasNetworkToHost(&pmsg->data, pframe->data + 4, pframe->dlc - 4, pframe->data[1]);
    if (ret < 0)
    {
        CANAS_TRACE("frameparser: bad data type=%i error=%i\n", (int)pframe->data[1], ret);
        return ret;
    }
    return 0;
}

static int _makeFrame(CanasCanFrame* pframe, uint16_t msg_id, const CanasMessage* pmsg, uint8_t redund_chan)
{
    memset(pframe, 0, sizeof(*pframe));
    pframe->id = msg_id & CANAS_CAN_MASK_STDID;
    if (redund_chan)
    {
        pframe->id |= ((uint32_t)redund_chan) * REDUND_CHAN_MULT;
        pframe->id |= CANAS_CAN_FLAG_EFF;                             // On redundancy channel 0 STD IDs are used.
    }
    pframe->data[0] = pmsg->node_id;
    pframe->data[1] = pmsg->data.type;
    pframe->data[2] = pmsg->service_code;
    pframe->data[3] = pmsg->message_code;

    int datalen = canasHostToNetwork(pframe->data + 4, &pmsg->data);
    if (datalen < 0)
    {
        CANAS_TRACE("framemaker: bad data type=%i error=%i\n", (int)pmsg->data.type, datalen);
        return datalen;
    }
    pframe->dlc = datalen + 4;
    return 0;
}

static CanasParamSubscription* _findParamSubscription(CanasInstance* pi, uint16_t id)
{
    CanasParamSubscription* ppar = pi->pparam_subs;
    while (ppar)
    {
        if (ppar->message_id == id)
            return ppar;
        ppar = ppar->pnext;
    }
    return NULL;
}

static CanasParamAdvertisement* _findParamAdvertisement(CanasInstance* pi, uint16_t id)
{
    CanasParamAdvertisement* padv = pi->pparam_advs;
    while (padv)
    {
        if (padv->message_id == id)
            return padv;
        padv = padv->pnext;
    }
    return NULL;
}

static CanasServiceSubscription* _findServiceSubscription(CanasInstance* pi, uint8_t service_code)
{
    CanasServiceSubscription* psrv = pi->pservice_subs;
    while (psrv)
    {
        if (psrv->service_code == service_code)
            return psrv;
        psrv = psrv->pnext;
    }
    return NULL;
}

static int _genericSend(CanasInstance* pi, int iface, uint16_t msg_id, uint8_t msggroup, const CanasMessage* pmsg)
{
    CanasCanFrame frame;
    int mkframe_result = -1;
    if (msggroup == MSGGROUP_PARAMETER)
        mkframe_result = _makeFrame(&frame, msg_id, pmsg, pi->config.redund_channel_id);
    else
        mkframe_result = _makeFrame(&frame, msg_id, pmsg, 0);              // redundancy channel 0 is for services

    if (mkframe_result != 0)
        return mkframe_result;

    CANAS_TRACE("sending %s\n", CANAS_DUMPFRAME(&frame));

    bool sent_successfully = false;
    if (iface < 0)
    {
        for (int i = 0; i < pi->config.iface_count; i++)
        {
            const int send_result = pi->config.fn_send(pi, i, &frame);
            if (send_result == 1)
                sent_successfully = true;            // At least one successful sending is enough to return success.
            else
                CANAS_TRACE("send failed: iface=%i result=%i\n", i, send_result);
        }
    }
    else
    {
        const int send_result = pi->config.fn_send(pi, iface, &frame);
        sent_successfully = send_result == true;
        if (!sent_successfully)
            CANAS_TRACE("send failed: iface=%i result=%i\n", iface, send_result);
    }
    return sent_successfully ? 0 : -CANAS_ERR_DRIVER;
}

CanasConfig canasMakeConfig(void)
{
    CanasConfig cfg;
    memset(&cfg, 0, sizeof(cfg));

    cfg.service_request_timeout_usec = CANAS_DEFAULT_SERVICE_REQUEST_TIMEOUT_USEC;
    cfg.service_poll_interval_usec   = CANAS_DEFAULT_SERVICE_POLL_INTERVAL_USEC;
    cfg.service_frame_hist_len       = CANAS_DEFAULT_SERVICE_HIST_LEN;
    cfg.repeat_timeout_usec          = CANAS_DEFAULT_REPEAT_TIMEOUT_USEC;

    return cfg;
}

int canasInit(CanasInstance* pi, const CanasConfig* pcfg, void* pthis)
{
    if (pi == NULL || pcfg == NULL)
        return -CANAS_ERR_ARGUMENT;
    if (!_isConfigOk(pcfg))
        return -CANAS_ERR_ARGUMENT;

    memset(pi, 0, sizeof(*pi));
    pi->config = *pcfg;
    pi->pthis = pthis;

    if (pcfg->fn_filter != NULL && pcfg->filters_per_iface) // Ask filter to accept all messages
    {
        CanasCanFilterConfig filt;
        filt.id = 0;                                        // Dear filter,
        filt.mask = CANAS_CAN_FLAG_RTR;                     // Would you be so kind to...
        for (int i = 0; i < pcfg->iface_count; i++)
            if ((pcfg->fn_filter(pi, i, &filt, 1)) < 0)
                return -CANAS_ERR_DRIVER;
    }
    return 0;
}

int canasUpdate(CanasInstance* pi, int iface, const CanasCanFrame* pframe)
{
    const uint64_t timestamp = canasTimestamp(pi);

    if (pi == NULL)
        return -CANAS_ERR_ARGUMENT;
    if (pframe != NULL && (iface >= pi->config.iface_count || iface < 0))
        return -CANAS_ERR_ARGUMENT;

    uint16_t msg_id = 0xFFFF;
    CanasMessage msg;
    MessageGroup msggroup = MSGGROUP_WTF;
    uint8_t redund_ch = 0;
    int ret = 0;

    if (pframe != NULL)
    {
        //CANAS_TRACE("recv %s\n", CANAS_DUMPFRAME(pframe));
        ret = _parseFrame(pframe, &msg_id, &msg, &redund_ch);
        if (ret == 0)
        {
            msggroup = _detectMessageGroup(msg_id);
            if (msggroup == MSGGROUP_WTF)
            {
                CANAS_TRACE("update: failed to detect the message group\n");
                ret = -CANAS_ERR_BAD_MESSAGE_ID;
            }
        }
    }

    if (msggroup != MSGGROUP_WTF && pi->config.fn_hook != NULL)
        _issueMessageHookCallback(pi, iface, msg_id, &msg, redund_ch, timestamp);

    if (msggroup == MSGGROUP_PARAMETER)
    {
        CanasParamSubscription* ppar = _findParamSubscription(pi, msg_id);
        if (ppar != NULL)
            _handleReceivedParam(pi, ppar, msg_id, &msg, redund_ch, timestamp);
        else
            CANAS_TRACE("foreign param msgid=%03x datatype=%i\n", (unsigned  int)msg_id, (int)msg.data.type);
    }
    else if (msggroup == MSGGROUP_SERVICE)
    {
        if (canasIsInterestingServiceMessage(pi, msg_id, &msg))
        {
            CanasServiceSubscription* psrv = _findServiceSubscription(pi, msg.service_code);
            // Redundancy channel ID is not used with services
            if (psrv != NULL)
                canasHandleReceivedService(pi, psrv, iface, msg_id, &msg, pframe, timestamp);
            else
                CANAS_TRACE("foreign serv msgid=%03x srvcode=%i\n", (unsigned  int)msg_id, (int)msg.service_code);
        }
    }
    canasPollServices(pi, timestamp);
    return ret;
}

int canasParamSubscribe(CanasInstance* pi, uint16_t msg_id, uint8_t redund_chan_count,
                        CanasParamCallbackFn callback, void* callback_arg)
{
    if (pi == NULL)
        return -CANAS_ERR_ARGUMENT;

    if (_detectMessageGroup(msg_id) != MSGGROUP_PARAMETER)
        return -CANAS_ERR_BAD_MESSAGE_ID;
    if (redund_chan_count < 1)
        return -CANAS_ERR_BAD_REDUND_CHAN;
    if (_findParamSubscription(pi, msg_id) != NULL)
        return -CANAS_ERR_ENTRY_EXISTS;

    // this size magic is necessary because C++ does not allow flexible and zero-length arrays
    int size = sizeof(CanasParamSubscription) +
        sizeof(CanasParamCacheEntry) * redund_chan_count - sizeof(CanasParamCacheEntry);
    CanasParamSubscription* psub = canasMalloc(pi, size);
    if (psub == NULL)
        return -CANAS_ERR_NOT_ENOUGH_MEMORY;

    memset(psub, 0, size);
    psub->callback = callback;
    psub->callback_arg = callback_arg;
    psub->message_id = msg_id;
    psub->redund_count = redund_chan_count;

    canasListInsert((CanasListEntry**)&pi->pparam_subs, psub);
    return 0;
}

int canasParamUnsubscribe(CanasInstance* pi, uint16_t msg_id)
{
    if (pi == NULL)
        return -CANAS_ERR_ARGUMENT;

    CanasParamSubscription* psub = _findParamSubscription(pi, msg_id);
    if (psub != NULL)
    {
        canasListRemove((CanasListEntry**)&pi->pparam_subs, psub);
        canasFree(pi, psub);
        return 0;
    }
    return -CANAS_ERR_NO_SUCH_ENTRY;
}

int canasParamRead(CanasInstance* pi, uint16_t msg_id, uint8_t redund_chan, CanasParamCallbackArgs* pargs)
{
    if (pi == NULL || pargs == NULL)
        return -CANAS_ERR_ARGUMENT;

    CanasParamSubscription* psub = _findParamSubscription(pi, msg_id);
    if (psub != NULL)
    {
        if (redund_chan >= psub->redund_count)
            return -CANAS_ERR_BAD_REDUND_CHAN;
        pargs->message = psub->redund_cache[redund_chan].message;
        pargs->timestamp_usec = psub->redund_cache[redund_chan].timestamp_usec;
        pargs->message_id = msg_id;
        pargs->parg = psub->callback_arg;
        pargs->redund_channel_id = redund_chan;
        return 0;
    }
    return -CANAS_ERR_NO_SUCH_ENTRY;
}

int canasParamAdvertise(CanasInstance* pi, uint16_t msg_id, bool interlaced)
{
    if (pi == NULL)
        return -CANAS_ERR_ARGUMENT;
    if (_detectMessageGroup(msg_id) != MSGGROUP_PARAMETER)
        return -CANAS_ERR_BAD_MESSAGE_ID;
    if (_findParamAdvertisement(pi, msg_id) != NULL)
        return -CANAS_ERR_ENTRY_EXISTS;

    CanasParamAdvertisement* padv = canasMalloc(pi, sizeof(CanasParamAdvertisement));
    if (padv == NULL)
        return -CANAS_ERR_NOT_ENOUGH_MEMORY;
    memset(padv, 0, sizeof(*padv));
    padv->message_id = msg_id;

    // Interlacing is only enabled if we have more than one interface (obviously)
    if (pi->config.iface_count < 2)
        interlaced = false;
    padv->interlacing_next_iface = interlaced ? 0 : ALL_IFACES;

    canasListInsert((CanasListEntry**)&pi->pparam_advs, padv);
    return 0;
}

int canasParamUnadvertise(CanasInstance* pi, uint16_t msg_id)
{
    if (pi == NULL)
        return -CANAS_ERR_ARGUMENT;

    CanasParamAdvertisement* padv = _findParamAdvertisement(pi, msg_id);
    if (padv != NULL)
    {
        canasListRemove((CanasListEntry**)&pi->pparam_advs, padv);
        canasFree(pi, padv);
        return 0;
    }
    return -CANAS_ERR_NO_SUCH_ENTRY;
}

int canasParamPublish(CanasInstance* pi, uint16_t msg_id, const CanasMessageData* pdata, uint8_t service_code)
{
    if (pi == NULL || pdata == NULL)
        return -CANAS_ERR_ARGUMENT;

    const uint8_t msggroup = _detectMessageGroup(msg_id);
    if (msggroup != MSGGROUP_PARAMETER)
        return -CANAS_ERR_BAD_MESSAGE_ID;

    CanasParamAdvertisement* padv = _findParamAdvertisement(pi, msg_id);
    if (padv == NULL)
        return -CANAS_ERR_NO_SUCH_ENTRY;

    int iface = ALL_IFACES;
    if (padv->interlacing_next_iface >= 0)
    {
        iface = padv->interlacing_next_iface++;
        if (padv->interlacing_next_iface >= pi->config.iface_count)
            padv->interlacing_next_iface = 0;
    }

    CanasMessage msg;
    memset(&msg, 0, sizeof(msg));
    msg.node_id = pi->config.node_id;
    msg.service_code = service_code;
    msg.message_code = padv->message_code++;    // Keeping the correct value of message code
    msg.data = *pdata;
    return _genericSend(pi, iface, msg_id, msggroup, &msg);
}

int canasServiceSendRequest(CanasInstance* pi, const CanasMessage* pmsg)
{
    if (pi == NULL || pmsg == NULL)
        return -CANAS_ERR_ARGUMENT;

    if (pmsg->node_id == pi->config.node_id)  // Self-addressed requests are ridiculous
        return -CANAS_ERR_BAD_NODE_ID;

    int msg_id = canasServiceChannelToMessageID(pi->config.service_channel, true);
    if (msg_id < 0)
        return msg_id;
    return _genericSend(pi, ALL_IFACES, (uint16_t)msg_id, MSGGROUP_SERVICE, pmsg);
}

int canasServiceSendResponse(CanasInstance* pi, const CanasMessage* pmsg, uint8_t service_channel)
{
    if (pi == NULL || pmsg == NULL)
        return -CANAS_ERR_ARGUMENT;

    CanasMessage msg = *pmsg;
    if (msg.node_id == CANAS_BROADCAST_NODE_ID)
    {
        CANAS_TRACE("srv response to broadcast request\n");
        msg.node_id = pi->config.node_id;   // Silently correct the Node ID for responses to global requests
    }

    if (msg.node_id != pi->config.node_id)  // Usage of foreign Node ID in response is against specification.
        return -CANAS_ERR_BAD_NODE_ID;

    int msg_id = canasServiceChannelToMessageID(service_channel, false); // Also will check validity of service_channel
    if (msg_id < 0)
        return msg_id;
    return _genericSend(pi, ALL_IFACES, (uint16_t)msg_id, MSGGROUP_SERVICE, &msg);
}

int canasServiceRegister(CanasInstance* pi, uint8_t service_code, CanasServicePollCallbackFn callback_poll,
                         CanasServiceRequestCallbackFn callback_request,
                         CanasServiceResponseCallbackFn callback_response, void* pstate)
{
    if (pi == NULL)
        return -CANAS_ERR_ARGUMENT;
    if (_findServiceSubscription(pi, service_code) != NULL)
        return -CANAS_ERR_ENTRY_EXISTS;

    // this size magic is necessary because C++ does not allow flexible and zero-length arrays
    int size = sizeof(CanasServiceSubscription) +
        sizeof(CanasServiceFrameHistoryEntry) * pi->config.service_frame_hist_len - sizeof(CanasServiceFrameHistoryEntry);
    CanasServiceSubscription* psrv = canasMalloc(pi, size);
    if (psrv == NULL)
        return -CANAS_ERR_NOT_ENOUGH_MEMORY;

    memset(psrv, 0, size);
    psrv->callback_poll = callback_poll;
    psrv->callback_request = callback_request;
    psrv->callback_response = callback_response;
    psrv->pstate = pstate;
    psrv->history_len = pi->config.service_frame_hist_len;
    psrv->service_code = service_code;
    for (int i = 0; i < psrv->history_len; i++)
        psrv->history[i].ifaces_mask = 0xFF;        // Default value, to avoid false-positives on repetition detection

    canasListInsert((CanasListEntry**)&pi->pservice_subs, psrv);
    return 0;
}

int canasServiceUnregister(CanasInstance* pi, uint8_t service_code)
{
    if (pi == NULL)
        return -CANAS_ERR_ARGUMENT;

    CanasServiceSubscription* psrv = _findServiceSubscription(pi, service_code);
    if (psrv != NULL)
    {
        canasListRemove((CanasListEntry**)&pi->pservice_subs, psrv);
        canasFree(pi, psrv);
        return 0;
    }
    return -CANAS_ERR_NO_SUCH_ENTRY;
}

int canasServiceSetState(CanasInstance* pi, uint8_t service_code, void* pstate)
{
    if (pi == NULL)
        return -CANAS_ERR_ARGUMENT;

    CanasServiceSubscription* psrv = _findServiceSubscription(pi, service_code);
    if (psrv != NULL)
    {
        psrv->pstate = pstate;
        return 0;
    }
    return -CANAS_ERR_NO_SUCH_ENTRY;
}

int canasServiceGetState(CanasInstance* pi, uint8_t service_code, void** ppstate)
{
    if (pi == NULL || ppstate == NULL)
        return -CANAS_ERR_ARGUMENT;

    CanasServiceSubscription* psrv = _findServiceSubscription(pi, service_code);
    if (psrv != NULL)
    {
        *ppstate = psrv->pstate;
        return 0;
    }
    return -CANAS_ERR_NO_SUCH_ENTRY;
}
