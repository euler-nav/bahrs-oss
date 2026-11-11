/*
 * Standard Node Synchronization Service
 * Pavel Kirienko, 2013 (pavel.kirienko@gmail.com)
 */

#include <stdlib.h>
#include <string.h>
#include <canaerospace/services/std_nodesync.h>
#include "../debug.h"

static const int THIS_SERVICE_CODE = 1;

typedef struct  // ISO C forbids conversion of object pointer to function pointer
{
    CanasSrvNssCallback callback;
} State;

static void _request(CanasInstance* pi, CanasServiceRequestCallbackArgs* pargs)
{
    State* ps = (State*)pargs->pstate;
    if (ps == NULL)
    {
        CANAS_TRACE("srv nss req: invalid state pointer\n");
        return;
    }
    if (pargs->message.data.type != CANAS_DATATYPE_ULONG)
    {
        CANAS_TRACE("srv nss req: wrong data type %i\n", (int)pargs->message.data.type);
        return;
    }
    if (pargs->message.message_code != 0)
    {
        CANAS_TRACE("srv nss req: wrong message code %i\n", (int)pargs->message.message_code);
        return;
    }
    if (ps->callback != NULL)
        ps->callback(pi, pargs->message.data.container.ULONG);
}

int canasSrvNssInit(CanasInstance* pi, CanasSrvNssCallback callback)
{
    if (pi == NULL)
        return -CANAS_ERR_ARGUMENT;

    State* ps = canasMalloc(pi, sizeof(State));
    if (ps == NULL)
        return -CANAS_ERR_NOT_ENOUGH_MEMORY;
    ps->callback = callback;

    int ret = canasServiceRegister(pi, THIS_SERVICE_CODE, NULL, _request, NULL, ps);
    if (ret != 0)
        canasFree(pi, ps);
    return ret;
}

int canasSrvNssPublish(CanasInstance* pi, uint32_t timestamp)
{
    CanasMessage msg;
    memset(&msg, 0, sizeof(msg));
    msg.service_code = THIS_SERVICE_CODE;
    msg.node_id      = CANAS_BROADCAST_NODE_ID;
    msg.data.type    = CANAS_DATATYPE_ULONG;
    msg.data.container.ULONG = timestamp;
    return canasServiceSendRequest(pi, &msg);
}
