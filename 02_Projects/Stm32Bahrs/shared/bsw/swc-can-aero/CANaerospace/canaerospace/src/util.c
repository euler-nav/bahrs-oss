/*
 * Misc stuff
 * Pavel Kirienko, 2013 (pavel.kirienko@gmail.com)
 */

#include <string.h>
#include <stdio.h>
#include <canaerospace/canaerospace.h>

uint64_t canasTimestamp(CanasInstance* pi)
{
    return pi->config.fn_timestamp(pi);
}

void* canasMalloc(CanasInstance* pi, int size)
{
    if (pi != NULL && pi->config.fn_malloc != NULL)
        return pi->config.fn_malloc(pi, size);
    return NULL;
}

void canasFree(CanasInstance* pi, void* ptr)
{
    if (pi != NULL && pi->config.fn_free != NULL)
        pi->config.fn_free(pi, ptr);
}

char* canasDumpCanFrame(const CanasCanFrame* pframe, char* pbuf)
{
    static const int ASCII_COLUMN_OFFSET = 34;
    char* wpos = pbuf, *epos = pbuf + CANAS_DUMP_BUF_LEN;
    memset(pbuf, 0, CANAS_DUMP_BUF_LEN);

    if (pframe->id & CANAS_CAN_FLAG_EFF)
        wpos += snprintf(wpos, epos - wpos, "%08x  ", (unsigned int)(pframe->id & CANAS_CAN_MASK_EXTID));
    else
        wpos += snprintf(wpos, epos - wpos, "     %03x  ", (unsigned int)(pframe->id & CANAS_CAN_MASK_STDID));

    if (pframe->id & CANAS_CAN_FLAG_RTR)
    {
        wpos += snprintf(wpos, epos - wpos, " RTR");
    }
    else
    {
        for (int dlen = 0; dlen < pframe->dlc; dlen++)                         // hex bytes
            wpos += snprintf(wpos, epos - wpos, " %02x", pframe->data[dlen]);

        while (wpos < pbuf + ASCII_COLUMN_OFFSET)                              // alignment
            *wpos++ = ' ';

        wpos += snprintf(wpos, epos - wpos, "  \'");                           // ascii
        for (int dlen = 0; dlen < pframe->dlc; dlen++)
        {
            uint8_t ch = pframe->data[dlen];
            if (ch < 0x20 || ch > 0x7E)
                ch = '.';
            wpos += snprintf(wpos, epos - wpos, "%c", ch);
        }
        wpos += snprintf(wpos, epos - wpos, "\'");
    }
    return pbuf;
}

char* canasDumpMessage(const CanasMessage* pmsg, char* pbuf)
{
    snprintf(pbuf, CANAS_DUMP_BUF_LEN, "N%02x D%02x S%02x M%02x   [%02x %02x %02x %02x]",
        // header
        (unsigned int)pmsg->node_id,
        (unsigned int)pmsg->data.type,
        (unsigned int)pmsg->service_code,
        (unsigned int)pmsg->message_code,
        // payload
        (unsigned int)pmsg->data.container.UCHAR4[0],
        (unsigned int)pmsg->data.container.UCHAR4[1],
        (unsigned int)pmsg->data.container.UCHAR4[2],
        (unsigned int)pmsg->data.container.UCHAR4[3]);
    return pbuf;
}
