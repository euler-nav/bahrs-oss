/*
 * std_identification.c
 * Pavel Kirienko, 2013 (pavel.kirienko@gmail.com)
 */

#include <stdlib.h>
#include <string.h>
#include <canaerospace/services/std_identification.h>
#include "../debug.h"

#define MAX_FOREIGN_NODES (CANAS_MAX_NODES - 1)

static const uint8_t THIS_SERVICE_CODE = 0;

typedef struct
{
    uint64_t deadline;
    CanasSrvIdsResponseCallback callback;
    uint8_t node_id;
} CanasSrvIdsRequestHandle;

typedef struct
{
    CanasSrvIdsPayload self_definition;
    uint8_t pending_requests_len;
    CanasSrvIdsRequestHandle pending_requests[];
} CanasSrvIdsData;

static void _poll(CanasInstance* pi, CanasServicePollCallbackArgs* pargs)
{
    CanasSrvIdsData* pd = (CanasSrvIdsData*)pargs->pstate;
    if (pd == NULL)
    {
        CANAS_TRACE("srv ids poll: invalid state pointer\n");
        return;
    }
    // Check timeouts for the all entries
    for (int i = 0; i < pd->pending_requests_len; i++)
    {
        CanasSrvIdsRequestHandle* prh = pd->pending_requests + i;
        if (prh->node_id != 0)
        {
            if (prh->deadline < pargs->timestamp_usec)
            {
                if (prh->callback != NULL)
                    prh->callback(pi, prh->node_id, NULL); // Callback with NULL payload means that request was timed out
                memset(prh, 0, sizeof(*prh));              // Clear entry
            }
        }
    }
}

static void _response(CanasInstance* pi, CanasServiceResponseCallbackArgs* pargs)
{
    CanasSrvIdsData* pd = (CanasSrvIdsData*)pargs->pstate;
    if (pd == NULL)
    {
        CANAS_TRACE("srv ids resp: invalid state pointer\n");
        return;
    }
    if (pargs->message.data.type != CANAS_DATATYPE_UCHAR4)
    {
        CANAS_TRACE("srv ids resp: wrong data type %i\n", (int)pargs->message.data.type);
        return;
    }
    // Search for the appropriate entry and call its callback:
    for (int i = 0; i < pd->pending_requests_len; i++)
    {
        CanasSrvIdsRequestHandle* prh = pd->pending_requests + i;
        if (prh->node_id == pargs->message.node_id)
        {
            if (prh->callback != NULL)
            {
                CanasSrvIdsPayload payload;
                payload.hardware_revision = pargs->message.data.container.UCHAR4[0];
                payload.software_revision = pargs->message.data.container.UCHAR4[1];
                payload.id_distribution   = pargs->message.data.container.UCHAR4[2];
                payload.header_type       = pargs->message.data.container.UCHAR4[3];
                prh->callback(pi, prh->node_id, &payload);
            }
            memset(prh, 0, sizeof(*prh)); // Clear entry
            break; // No need to go further because the next entries may be intended for other responses
            /*
             * Consider the components A and B of an application logic. "A" sent IDS request no the node N,
             * then "B" did the same.
             * Now, if the response slot for B was allocated with lower index than "A", "B" WILL get the response which
             * was originally addressed for A, and then A will get the response which was intended for B.
             */
        }
    }
}

static void _request(CanasInstance* pi, CanasServiceRequestCallbackArgs* pargs)
{
    const CanasSrvIdsData* pd = (CanasSrvIdsData*)pargs->pstate;
    if (pd == NULL)
    {
        CANAS_TRACE("srv ids req: invalid state pointer\n");
        return;
    }

    CanasMessage msg = pargs->message;
    msg.data.type = CANAS_DATATYPE_UCHAR4;
    msg.data.container.UCHAR4[0] = pd->self_definition.hardware_revision;
    msg.data.container.UCHAR4[1] = pd->self_definition.software_revision;
    msg.data.container.UCHAR4[2] = pd->self_definition.id_distribution;
    msg.data.container.UCHAR4[3] = pd->self_definition.header_type;

    int ret = canasServiceSendResponse(pi, &msg, pargs->service_channel);
    if (ret != 0)
        CANAS_TRACE("srv ids: failed to respond: %i\n", ret);
}

int canasSrvIdsInit(CanasInstance* pi, const CanasSrvIdsPayload* pself_definition, uint8_t max_pending_requests)
{
    if (pi == NULL)
        return -CANAS_ERR_ARGUMENT;

    const int size = sizeof(CanasSrvIdsData) + sizeof(CanasSrvIdsRequestHandle) * max_pending_requests;
    CanasSrvIdsData* pd = canasMalloc(pi, size);
    if (pd == NULL)
        return -CANAS_ERR_NOT_ENOUGH_MEMORY;
    memset(pd, 0, size);
    pd->pending_requests_len = max_pending_requests;
    pd->self_definition = *pself_definition;

    int ret = canasServiceRegister(pi, THIS_SERVICE_CODE, _poll, _request, _response, pd);
    if (ret != 0)
        canasFree(pi, pd);
    return ret;
}

int canasSrvIdsRequest(CanasInstance* pi, uint8_t node_id, CanasSrvIdsResponseCallback callback)
{
    if (pi == NULL || callback == NULL)
        return -CANAS_ERR_ARGUMENT;

    CanasSrvIdsData* pd = NULL;
    int ret = canasServiceGetState(pi, THIS_SERVICE_CODE, (void**)&pd);
    if (ret != 0)
        return ret;
    if (pd == NULL)
        return -CANAS_ERR_LOGIC;

    if (node_id == CANAS_BROADCAST_NODE_ID && pd->pending_requests_len < MAX_FOREIGN_NODES) // Quick check
        return -CANAS_ERR_QUOTA_EXCEEDED;
    /*
     * First we send the request,
     * then create response entries for it.
     */
    CanasMessage msg;
    memset(&msg, 0, sizeof(msg));
    msg.node_id = node_id;
    msg.message_code = 0;
    msg.service_code = THIS_SERVICE_CODE;
    msg.data.type = CANAS_DATATYPE_NODATA;
    ret = canasServiceSendRequest(pi, &msg);
    if (ret != 0)
        return ret;
    /*
     * Note that the following part of this function may fail.
     * In this case we will be unable to match the received responses with the corresponding callbacks,
     * but this is not a big problem.
     */
    const uint64_t deadline = canasTimestamp(pi) + pi->config.service_request_timeout_usec;

    if (node_id == CANAS_BROADCAST_NODE_ID)                // Broadcast request
    {
        int free_entries = 0;
        for (int i = 0; i < pd->pending_requests_len; i++)
        {
            if (pd->pending_requests[i].node_id == 0)      // This is free
                free_entries++;
        }
        if (free_entries < MAX_FOREIGN_NODES)
            return -CANAS_ERR_QUOTA_EXCEEDED;

        int ids_allocated = 0;
        uint8_t next_node_id = 1;                          // Remember, lowest Node ID is 1
        for (int i = 0; ids_allocated < MAX_FOREIGN_NODES; i++)
        {
            if (pd->pending_requests[i].node_id != 0)
                continue;

            if (next_node_id == pi->config.node_id)        // Self-addressed queries are ridiculous
                next_node_id++;

            pd->pending_requests[i].node_id  = next_node_id;
            pd->pending_requests[i].callback = callback;
            pd->pending_requests[i].deadline = deadline;

            ids_allocated++;
            next_node_id++;
        }
    }
    else                                                   // Non-broadcast request
    {
        /*
         * Note that it is allowed to send more than one request to the same node at the same time.
         * I don't know who cares, but it is possible.
         */
        bool okay = false;
        for (int i = 0; i < pd->pending_requests_len; i++)
        {
            if (pd->pending_requests[i].node_id != 0)
                continue;
            pd->pending_requests[i].node_id  = node_id;
            pd->pending_requests[i].callback = callback;
            pd->pending_requests[i].deadline = deadline;
            okay = true;
            break;
        }
        if (!okay)
            return -CANAS_ERR_QUOTA_EXCEEDED;
    }
    return 0;
}
