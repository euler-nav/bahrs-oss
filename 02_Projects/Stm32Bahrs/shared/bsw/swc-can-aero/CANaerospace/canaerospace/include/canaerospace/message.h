/*
 * Message definitions
 * Pavel Kirienko, 2013 (pavel.kirienko@gmail.com)
 */

#ifndef CANAEROSPACE_MESSAGE_H_
#define CANAEROSPACE_MESSAGE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    CANAS_PARAM_DISTRIBUTION_DEFAULT = 0
} CanasParamIdDistributionMode;

typedef enum
{
    CANAS_MSGTYPE_EMERGENCY_EVENT_MIN = 0,
    CANAS_MSGTYPE_EMERGENCY_EVENT_MAX = 127,

    CANAS_MSGTYPE_NODE_SERVICE_HIGH_MIN = 128,
    CANAS_MSGTYPE_NODE_SERVICE_HIGH_MAX = 199,

    CANAS_MSGTYPE_USER_DEFINED_HIGH_MIN = 200,
    CANAS_MSGTYPE_USER_DEFINED_HIGH_MAX = 299,

    CANAS_MSGTYPE_NORMAL_OPERATION_MIN = 300,
    CANAS_MSGTYPE_NORMAL_OPERATION_MAX = 1799,

    CANAS_MSGTYPE_USER_DEFINED_LOW_MIN = 1800,
    CANAS_MSGTYPE_USER_DEFINED_LOW_MAX = 1899,

    CANAS_MSGTYPE_DEBUG_SERVICE_MIN = 1900,
    CANAS_MSGTYPE_DEBUG_SERVICE_MAX = 1999,

    CANAS_MSGTYPE_NODE_SERVICE_LOW_MIN = 2000,
    CANAS_MSGTYPE_NODE_SERVICE_LOW_MAX = 2031
} CanasMessageTypeID;

typedef enum
{
    CANAS_SERVICE_CHANNEL_HIGH_MIN = 0,
    CANAS_SERVICE_CHANNEL_HIGH_MAX = 35,

    CANAS_SERVICE_CHANNEL_LOW_MIN = 100,
    CANAS_SERVICE_CHANNEL_LOW_MAX = 115
} CanasServiceChannelID;

typedef enum
{
    CANAS_DATATYPE_NODATA,
    CANAS_DATATYPE_ERROR,

    CANAS_DATATYPE_FLOAT,

    CANAS_DATATYPE_LONG,
    CANAS_DATATYPE_ULONG,
    CANAS_DATATYPE_BLONG,

    CANAS_DATATYPE_SHORT,
    CANAS_DATATYPE_USHORT,
    CANAS_DATATYPE_BSHORT,

    CANAS_DATATYPE_CHAR,
    CANAS_DATATYPE_UCHAR,
    CANAS_DATATYPE_BCHAR,

    CANAS_DATATYPE_SHORT2,
    CANAS_DATATYPE_USHORT2,
    CANAS_DATATYPE_BSHORT2,

    CANAS_DATATYPE_CHAR4,
    CANAS_DATATYPE_UCHAR4,
    CANAS_DATATYPE_BCHAR4,

    CANAS_DATATYPE_CHAR2,
    CANAS_DATATYPE_UCHAR2,
    CANAS_DATATYPE_BCHAR2,

    CANAS_DATATYPE_MEMID,
    CANAS_DATATYPE_CHKSUM,

    CANAS_DATATYPE_ACHAR,
    CANAS_DATATYPE_ACHAR2,
    CANAS_DATATYPE_ACHAR4,

    CANAS_DATATYPE_CHAR3,
    CANAS_DATATYPE_UCHAR3,
    CANAS_DATATYPE_BCHAR3,
    CANAS_DATATYPE_ACHAR3,

    CANAS_DATATYPE_DOUBLEH,
    CANAS_DATATYPE_DOUBLEL,

    CANAS_DATATYPE_RESVD_BEGIN_,
    CANAS_DATATYPE_RESVD_END_  = 99,

    CANAS_DATATYPE_UDEF_BEGIN_ = 100,
    CANAS_DATATYPE_UDEF_END_   = 255,

    CANAS_DATATYPE_ALL_END_    = 255
} CanasStandardDataTypeID;

typedef union
{
    uint32_t ERROR;

    float FLOAT;

    int32_t  LONG;
    uint32_t ULONG;
    uint32_t BLONG;

    int16_t  SHORT;
    uint16_t USHORT;
    uint16_t BSHORT;

    int8_t  CHAR;
    uint8_t UCHAR;
    uint8_t BCHAR;

    int16_t  SHORT2[2];
    uint16_t USHORT2[2];
    uint16_t BSHORT2[2];

    int8_t  CHAR4[4];
    uint8_t UCHAR4[4];
    uint8_t BCHAR4[4];

    int8_t  CHAR2[2];
    uint8_t UCHAR2[2];
    uint8_t BCHAR2[2];

    uint32_t MEMID;
    uint32_t CHKSUM;

    uint8_t ACHAR;
    uint8_t ACHAR2[2];
    uint8_t ACHAR4[4];

    int8_t  CHAR3[3];
    uint8_t UCHAR3[3];
    uint8_t BCHAR3[3];
    uint8_t ACHAR3[3];

    uint32_t DOUBLEH;
    uint32_t DOUBLEL;
} CanasDataContainer;

typedef struct
{
    uint8_t type;                 ///< @ref CanasStandardDataTypeID or custom
    uint8_t length;               ///< Ignored with standard datatypes; required otherwise. Leave zero if unused.
    CanasDataContainer container;
} CanasMessageData;

typedef struct
{
    CanasMessageData data;
    uint8_t node_id;
    uint8_t service_code;
    uint8_t message_code;
} CanasMessage;

#ifdef __cplusplus
}
#endif
#endif
