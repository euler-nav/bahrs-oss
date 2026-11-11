/*
 * Message payload conversions
 * Pavel Kirienko, 2013 (pavel.kirienko@gmail.com)
 */

#include <stdlib.h>
#include <string.h>
#include "marshal.h"
#include "debug.h"

#ifdef __GNUC__
// error: comparison is always true due to limited range of data type [-Werror=type-limits]
#  pragma GCC diagnostic ignored "-Wtype-limits"
#endif

#define IS_UDEF(type) ((type) >= CANAS_DATATYPE_UDEF_BEGIN_ && (type) <= CANAS_DATATYPE_UDEF_END_)

#if __BYTE_ORDER__ == __BIG_ENDIAN
#  define SWITCH(x)
#else

static inline void _switchByteOrder(void* ptr, int len)
{
    uint8_t tmp[4], *pbyte = ptr;
    if (len == 2)
    {
        memcpy(tmp, ptr, 2);
        pbyte[0] = tmp[1];
        pbyte[1] = tmp[0];
    }
    else if (len == 4)
    {
        memcpy(tmp, ptr, 4);
        pbyte[0] = tmp[3];
        pbyte[1] = tmp[2];
        pbyte[2] = tmp[1];
        pbyte[3] = tmp[0];
    }
}

#  define SWITCH(x) _switchByteOrder(&(x), sizeof(x))
#endif

#define MARSHAL_RESULT_ERROR -1
#define MARSHAL_RESULT_UDEF  -2

static int _marshal(CanasMessageData* pmsg)
{
    switch (pmsg->type)
    {
    case CANAS_DATATYPE_NODATA:
        return 0;

    case CANAS_DATATYPE_FLOAT:
        /*
         * Target platform must support IEEE754 floats. The good news that almost every platform does that.
         * But if some platform doesn't, proper converting from native float
         * representation to IEEE754 (and vice versa) must be implemented somewhere here.
         */
        // FALLTHROUGH
    case CANAS_DATATYPE_ERROR:
    case CANAS_DATATYPE_LONG:
    case CANAS_DATATYPE_ULONG:
    case CANAS_DATATYPE_BLONG:
        SWITCH(pmsg->container.ULONG);
        return 4;

    case CANAS_DATATYPE_SHORT:
    case CANAS_DATATYPE_USHORT:
    case CANAS_DATATYPE_BSHORT:
        SWITCH(pmsg->container.USHORT);
        return 2;

    case CANAS_DATATYPE_CHAR:
    case CANAS_DATATYPE_UCHAR:
    case CANAS_DATATYPE_BCHAR:
        return 1;

    case CANAS_DATATYPE_SHORT2:
    case CANAS_DATATYPE_USHORT2:
    case CANAS_DATATYPE_BSHORT2:
        SWITCH(pmsg->container.USHORT2[0]);
        SWITCH(pmsg->container.USHORT2[1]);
        return 4;

    case CANAS_DATATYPE_CHAR4:
    case CANAS_DATATYPE_UCHAR4:
    case CANAS_DATATYPE_BCHAR4:
        return 4;

    case CANAS_DATATYPE_CHAR2:
    case CANAS_DATATYPE_UCHAR2:
    case CANAS_DATATYPE_BCHAR2:
        return 2;

    case CANAS_DATATYPE_MEMID:
    case CANAS_DATATYPE_CHKSUM:
        SWITCH(pmsg->container.MEMID);
        return 4;

    case CANAS_DATATYPE_ACHAR:
        return 1;

    case CANAS_DATATYPE_ACHAR2:
        return 2;

    case CANAS_DATATYPE_ACHAR4:
        return 4;

    case CANAS_DATATYPE_CHAR3:
    case CANAS_DATATYPE_UCHAR3:
    case CANAS_DATATYPE_BCHAR3:
    case CANAS_DATATYPE_ACHAR3:
        return 3;

    case CANAS_DATATYPE_DOUBLEH:
    case CANAS_DATATYPE_DOUBLEL:
        // See note about IEEE754 compatibility above.
        SWITCH(pmsg->container.DOUBLEL);
        return 4;

    default:
        if (IS_UDEF(pmsg->type) && pmsg->length <= 4)
            return MARSHAL_RESULT_UDEF;
        CANAS_TRACE("marshal: unknown data type %02x, udf_len=%i\n", (int)pmsg->type, (int)pmsg->length);
        return MARSHAL_RESULT_ERROR;
    }
}

int canasHostToNetwork(uint8_t* pdata, const CanasMessageData* phost)
{
    if (pdata == NULL || phost == NULL)
        return -CANAS_ERR_ARGUMENT;

    CanasMessageData nwk = *phost;
    const int ret = _marshal(&nwk);
    if (ret >= 0)
    {
        memcpy(pdata, &nwk.container, ret);
        return ret;
    }
    if (ret == MARSHAL_RESULT_UDEF)
    {
        memcpy(pdata, &nwk.container, nwk.length);
        return nwk.length;
    }
    return -CANAS_ERR_BAD_DATA_TYPE;
}

int canasNetworkToHost(CanasMessageData* phost, const uint8_t* pdata, uint8_t datalen, uint8_t datatype)
{
    if (pdata == NULL || phost == NULL)
        return -CANAS_ERR_ARGUMENT;
    if (datalen > 4)
        return -CANAS_ERR_BAD_DATA_TYPE;

    memset(phost, 0, sizeof(*phost));
    phost->type = datatype;
    phost->length = datalen;             // For standard types this value should be ignored.
    memcpy(&phost->container, pdata, datalen);

    const int ret = _marshal(phost);
    if (ret >= 0)
    {
        if (ret == datalen)
            return ret;
        CANAS_TRACE("marshal n2h: datalen mismatch: got %i, declared %i\n", (int)datalen, (int)ret);
        return -CANAS_ERR_BAD_DATA_TYPE;
    }
    if (ret == MARSHAL_RESULT_UDEF)
        return datalen;
    return -CANAS_ERR_BAD_DATA_TYPE;
}
