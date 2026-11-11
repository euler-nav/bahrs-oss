/*
 * The standard Flash Programming Service.
 * Pavel Kirienko, 2013 (pavel.kirienko@gmail.com)
 */

#include <stdlib.h>
#include <string.h>
#include <canaerospace/services/std_flashprog.h>
#include "../debug.h"

static const uint8_t THIS_SERVICE_CODE = 6;

typedef struct
{
    uint64_t deadline;
    CanasSrvFpsResponseCallback callback;
    void* callback_arg;
    uint8_t node_id;
} CanasSrvFpsPendingRequest;

typedef struct
{
    CanasSrvFpsRequestCallback incoming_request_callback;
    CanasSrvFpsPendingRequest pending_request;
} CanasSrvFpsState;

static void _poll(CanasInstance* pi, CanasServicePollCallbackArgs* pargs)
{
    CanasSrvFpsState* ps = (CanasSrvFpsState*)pargs->pstate;
    if (ps == NULL)
    {
        CANAS_TRACE("srv fps poll: invalid state pointer\n");
        return;
    }
    if (ps->pending_request.node_id == 0 || ps->pending_request.callback == NULL)
        return;
    if (ps->pending_request.deadline >= pargs->timestamp_usec)
        return;

    CanasSrvFpsResponseCallback cb = ps->pending_request.callback;
    void* cb_arg = ps->pending_request.callback_arg;
    uint8_t node_id = ps->pending_request.node_id;

    memset(&ps->pending_request, 0, sizeof(ps->pending_request));  // It is necessary to clear the slot first

    cb(pi, node_id, true, 0, cb_arg);
}

static void _response(CanasInstance* pi, CanasServiceResponseCallbackArgs* pargs)
{
    CanasSrvFpsState* ps = (CanasSrvFpsState*)pargs->pstate;
    if (ps == NULL)
    {
        CANAS_TRACE("srv fps resp: invalid state pointer\n");
        return;
    }
    if (pargs->message.data.type != CANAS_DATATYPE_NODATA)
    {
        CANAS_TRACE("srv fps resp: wrong data type %i\n", (int)pargs->message.data.type);
        return;
    }
    if (pargs->message.node_id != ps->pending_request.node_id)
    {
        CANAS_TRACE("srv fps resp: unexpected response from %i\n", (int)pargs->message.node_id);
        return;
    }
    if (ps->pending_request.callback == NULL)
    {
        CANAS_TRACE("srv fps resp: no callback\n");
        return;
    }
    if (ps->pending_request.node_id != 0)
    {
        CanasSrvFpsResponseCallback cb = ps->pending_request.callback;
        void* cb_arg = ps->pending_request.callback_arg;
        int8_t srespc = (int8_t)pargs->message.message_code;

        memset(&ps->pending_request, 0, sizeof(ps->pending_request));  // It is necessary to clear the slot first

        cb(pi, pargs->message.node_id, false, srespc, cb_arg);
    }
}

static void _request(CanasInstance* pi, CanasServiceRequestCallbackArgs* pargs)
{
    const CanasSrvFpsState* ps = (CanasSrvFpsState*)pargs->pstate;
    if (ps == NULL)
    {
        CANAS_TRACE("srv fps req: invalid state pointer\n");
        return;
    }
    CanasMessage msg = pargs->message;

    if (pargs->message.data.type != CANAS_DATATYPE_NODATA)
    {
        CANAS_TRACE("srv fps req: wrong data type %i, abort\n", (int)pargs->message.data.type);
        msg.message_code = (uint8_t)CANAS_SRV_FPS_RESULT_ABORT;
    }
    else if (ps->incoming_request_callback == NULL)
    {
        CANAS_TRACE("srv fps req: no request handler, abort\n");
        msg.message_code = (uint8_t)CANAS_SRV_FPS_RESULT_ABORT;
    }
    else
        msg.message_code = (uint8_t)ps->incoming_request_callback(pi, msg.message_code);

    int ret = canasServiceSendResponse(pi, &msg, pargs->service_channel);
    if (ret != 0)
        CANAS_TRACE("srv fps: failed to respond: %i\n", ret);
}

int canasSrvFpsInit(CanasInstance* pi, CanasSrvFpsRequestCallback callback)
{
    if (pi == NULL)
        return -CANAS_ERR_ARGUMENT;

    CanasSrvFpsState* ps = canasMalloc(pi, sizeof(CanasSrvFpsState));
    if (ps == NULL)
        return -CANAS_ERR_NOT_ENOUGH_MEMORY;

    memset(ps, 0, sizeof(*ps));
    ps->incoming_request_callback = callback;

    int ret = canasServiceRegister(pi, THIS_SERVICE_CODE, _poll, _request, _response, ps);
    if (ret != 0)
        canasFree(pi, ps);
    return ret;
}

int canasSrvFpsRequest(CanasInstance* pi, uint8_t node_id, uint8_t security_code, CanasSrvFpsResponseCallback callback,
                       void* callback_arg)
{
    if (pi == NULL || callback == NULL)
        return -CANAS_ERR_ARGUMENT;

    if (node_id == CANAS_BROADCAST_NODE_ID)  // Broadcast requests are not allowed
        return -CANAS_ERR_BAD_NODE_ID;

    CanasSrvFpsState* ps = NULL;
    int ret = canasServiceGetState(pi, THIS_SERVICE_CODE, (void**)&ps);
    if (ret != 0)
        return ret;
    if (ps == NULL)
        return -CANAS_ERR_LOGIC;

    // Check is there another request pending or not:
    if (ps->pending_request.node_id != 0)
        return -CANAS_ERR_QUOTA_EXCEEDED;

    // Send request:
    CanasMessage msg;
    memset(&msg, 0, sizeof(msg));
    msg.node_id      = node_id;
    msg.message_code = security_code;
    msg.service_code = THIS_SERVICE_CODE;
    msg.data.type    = CANAS_DATATYPE_NODATA;
    ret = canasServiceSendRequest(pi, &msg);
    if (ret != 0)
        return ret;

    // Init the pending request struct:
    const uint64_t deadline = canasTimestamp(pi) + pi->config.service_request_timeout_usec;
    ps->pending_request.node_id = node_id;
    ps->pending_request.callback = callback;
    ps->pending_request.callback_arg = callback_arg;
    ps->pending_request.deadline = deadline;
    return 0;
}
