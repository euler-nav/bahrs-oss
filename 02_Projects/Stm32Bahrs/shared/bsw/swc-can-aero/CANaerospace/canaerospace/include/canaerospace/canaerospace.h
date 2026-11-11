/*
 * CANaerospace main header
 * Pavel Kirienko, 2013 (pavel.kirienko@gmail.com)
 */

#ifndef CANAEROSPACE_H_
#define CANAEROSPACE_H_

#include <stdint.h>
#include "driver.h"
#include "message.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Broadcast Node ID can be used to perform network-wide service requests
 */
static const int CANAS_BROADCAST_NODE_ID = 0;

/**
 * Maximum number of nodes per one network
 */
static const int CANAS_MAX_NODES = 255;

/**
 * This library can work with the number of redundant interfaces not higher than this.
 */
static const int CANAS_IFACE_COUNT_MAX = 8;

/**
 * Buffer size required for functions like canasDump*()
 */
#define CANAS_DUMP_BUF_LEN 50

/**
 * Nearly all API calls return an error code.
 * @note API calls return the negative error codes. You have to invert the sign to get the actual error code.
 */
typedef enum
{
    CANAS_ERR_OK = 0,

    CANAS_ERR_ARGUMENT,
    CANAS_ERR_NOT_ENOUGH_MEMORY,

    CANAS_ERR_DRIVER,

    CANAS_ERR_NO_SUCH_ENTRY,
    CANAS_ERR_ENTRY_EXISTS,

    CANAS_ERR_BAD_DATA_TYPE,
    CANAS_ERR_BAD_MESSAGE_ID,
    CANAS_ERR_BAD_NODE_ID,
    CANAS_ERR_BAD_REDUND_CHAN,
    CANAS_ERR_BAD_SERVICE_CHAN,
    CANAS_ERR_BAD_CAN_FRAME,

    CANAS_ERR_QUOTA_EXCEEDED,
    CANAS_ERR_LOGIC                 ///< May be returned by a service if it goes wrong
} CanasErrorCode;

typedef struct CanasInstanceStruct CanasInstance;

/**
 * Send a message to the bus.
 * @param [in] pi    Instance pointer
 * @param [in] iface Interface index
 * @param [in] pmsg  Pointer to the message to be sent
 * @return           Number of messages sent (0 or 1), negative on failure
 */
typedef int (*CanasCanSendFn)(CanasInstance*, int, const CanasCanFrame*);

/**
 * Configure acceptance filters.
 * @param [in] pi       Instance pointer
 * @param [in] iface    Interface index
 * @param [in] pfilters Pointer to an array of filter configs
 * @param [in] len      Length of the config array
 * @return              0 if ok, negative on failure
 */
typedef int (*CanasCanFilterFn)(CanasInstance*, int, const CanasCanFilterConfig*, int);

/**
 * Allocates a chunk of memory.
 * If the application does not require de-initialization features like unsubscription of unadvertisement, then dynamic
 * memory is not needed at all. If this is the case, you can use a static pool allocator.
 * This feature is useful for embedded systems where dynamic memory is not always available.
 * @note Allocated memory must be aligned properly.
 * @param [in] pi   Instance pointer
 * @param [in] size Size of the memory block required
 * @return          Pointer or NULL if no memory available
 */
typedef void* (*CanasMallocFn)(CanasInstance*, int);

/**
 * Deallocates memory. This function is not required in most cases, see @ref CanasMallocFn for details.
 * If the deallocation is not needed (e.g. in case of static allocator), then pointer to this function should be NULL.
 * @param [in] pi  Instance pointer
 * @param [in] ptr Pointer to the memory to be deallocated
 */
typedef void (*CanasFreeFn)(CanasInstance*, void*);

/**
 * Returns current timestamp in microseconds.
 * Any base value (uptime or UNIX time) will do.
 * @param [in] pi Instance pointer
 * @return        Current timestamp in microseconds
 */
typedef uint64_t (*CanasTimestampFn)(CanasInstance*);

typedef struct
{
    uint64_t timestamp_usec;
    CanasMessage message;
    uint16_t message_id;
    uint8_t redund_channel_id;
    uint8_t iface;
} CanasHookCallbackArgs;
typedef void (*CanasHookCallbackFn)(CanasInstance*, CanasHookCallbackArgs*);

typedef struct
{
    uint64_t timestamp_usec;
    void* parg;
    CanasMessage message;
    uint16_t message_id;
    uint8_t redund_channel_id;
} CanasParamCallbackArgs;
typedef void (*CanasParamCallbackFn)(CanasInstance*, CanasParamCallbackArgs*);

typedef struct
{
    uint64_t timestamp_usec;
    void* pstate;
} CanasServicePollCallbackArgs;
typedef void (*CanasServicePollCallbackFn)(CanasInstance*, CanasServicePollCallbackArgs*);

typedef struct
{
    uint64_t timestamp_usec;
    void* pstate;
    CanasMessage message;
    uint8_t service_channel;
} CanasServiceRequestCallbackArgs;
typedef void (*CanasServiceRequestCallbackFn)(CanasInstance*, CanasServiceRequestCallbackArgs*);

typedef struct
{
    uint64_t timestamp_usec;
    void* pstate;
    CanasMessage message;
} CanasServiceResponseCallbackArgs;
typedef void (*CanasServiceResponseCallbackFn)(CanasInstance*, CanasServiceResponseCallbackArgs*);

typedef struct
{
    uint64_t timestamp_usec;        ///< Empty entry contains zero timestamp
    uint32_t header;                ///< First 4 bytes of frame
    uint8_t ifaces_mask;
} CanasServiceFrameHistoryEntry;

typedef struct
{
    void* pnext;                    ///< Must be the first entry
    CanasServicePollCallbackFn callback_poll;
    CanasServiceRequestCallbackFn callback_request;
    CanasServiceResponseCallbackFn callback_response;
    void* pstate;
    uint8_t service_code;
    uint8_t history_len;
    CanasServiceFrameHistoryEntry history[1]; // flexible
} CanasServiceSubscription;

typedef struct
{
    uint64_t timestamp_usec;        ///< Empty entry contains zero timestamp
    CanasMessage message;
} CanasParamCacheEntry;

typedef struct
{
    void* pnext;                    ///< Must be the first entry
    CanasParamCallbackFn callback;
    void* callback_arg;
    uint16_t message_id;
    uint8_t redund_count;
    CanasParamCacheEntry redund_cache[1]; // flexible
} CanasParamSubscription;

typedef struct
{
    void* pnext;                    ///< Must be the first entry
    uint16_t message_id;
    uint8_t message_code;
    int8_t interlacing_next_iface;
} CanasParamAdvertisement;

typedef struct
{
    CanasCanSendFn fn_send;         ///< Required
    CanasCanFilterFn fn_filter;     ///< May be null if filters are not available

    CanasTimestampFn fn_timestamp;  ///< Required

    CanasMallocFn fn_malloc;        ///< Required; read the notes @ref CanasMallocFn
    CanasFreeFn fn_free;            ///< Optional, may be NULL. Read the notes @ref CanasFreeFn

    CanasHookCallbackFn fn_hook;    ///< Should be null if not used

    uint8_t iface_count;            ///< Number of interfaces available
    uint8_t filters_per_iface;      ///< Number of filters per interface. May be 0 if no filters available.

    uint32_t service_request_timeout_usec;///< Time to wait for response from remote node. Default is okay.
    uint16_t service_poll_interval_usec;  ///< Do not change
    uint8_t service_frame_hist_len;       ///< Do not change
    uint8_t service_channel;              ///< Service channel of the Local Node. May be of high or low priority.
    uint32_t repeat_timeout_usec;         ///< Largest interval of repeated messages (default should be good enough)

    uint8_t node_id;                ///< Local Node ID
    uint8_t redund_channel_id;      ///< Local Node Redundancy Channel ID
} CanasConfig;

struct CanasInstanceStruct
{
    CanasConfig config;
    void* pthis;                    ///< To be used by application

    uint64_t last_service_ts;

    CanasServiceSubscription* pservice_subs;
    CanasParamSubscription* pparam_subs;
    CanasParamAdvertisement* pparam_advs;
};

/**
 * Make a pre-initialized config struct, with default values provided where possible.
 * @return Pre-initialized @ref CanasConfig.
 */
CanasConfig canasMakeConfig(void);

/**
 * Initialize instance.
 * @param [out] pi    Pointer to instance to be initialized
 * @param [in]  pcfg  Pointer to the instance configuration
 * @param [in]  pthis Application-specific pointer, goes to the corresponding instance field.
 * @return            @ref CanasErrorCode
 */
int canasInit(CanasInstance* pi, const CanasConfig* pcfg, void* pthis);

/**
 * Update instance state.
 * Must be called for every new incoming frame or by timeout.
 * @note It is recommended to call this function at least every 10 ms
 * @param [in] pi     Instance pointer
 * @param [in] iface  Interface index from which the frame was received; ignored when no frame provided
 * @param [in] pframe Pointer to the received frame, NULL when called by timeout
 * @return            @ref CanasErrorCode
 */
int canasUpdate(CanasInstance* pi, int iface, const CanasCanFrame* pframe);

/**
 * Parameter subscriptions.
 * Each parameter must be subscribed before you can read it from the bus.
 * Each subscription can track an arbitrary number of redundant units transmitting this message, from 1 to 255.
 * Functions of this group return @ref CanasErrorCode.
 * @{
 */
int canasParamSubscribe(CanasInstance* pi, uint16_t msg_id, uint8_t redund_chan_count, CanasParamCallbackFn callback,
                        void* callback_arg);
int canasParamUnsubscribe(CanasInstance* pi, uint16_t msg_id);
int canasParamRead(CanasInstance* pi, uint16_t msg_id, uint8_t redund_chan, CanasParamCallbackArgs* pargs);
/**
 * @}
 */

/**
 * Parameter publications.
 * Each parameter must be advertised before you can publish it to the bus.
 * 'interlaced' stands for traffic sharing between all available interfaces.
 * Functions of this group return @ref CanasErrorCode.
 * @{
 */
int canasParamAdvertise(CanasInstance* pi, uint16_t msg_id, bool interlaced);
int canasParamUnadvertise(CanasInstance* pi, uint16_t msg_id);
int canasParamPublish(CanasInstance* pi, uint16_t msg_id, const CanasMessageData* pdata, uint8_t service_code);
/**
 * @}
 */

/**
 * Node Service Protocol API.
 * Normally you will not use it if you are not going to implement your own service.
 * Functions of this group return @ref CanasErrorCode.
 * @{
 */
int canasServiceSendRequest(CanasInstance* pi, const CanasMessage* pmsg);
int canasServiceSendResponse(CanasInstance* pi, const CanasMessage* pmsg, uint8_t service_channel);
int canasServiceRegister(CanasInstance* pi, uint8_t service_code, CanasServicePollCallbackFn callback_poll,
                         CanasServiceRequestCallbackFn callback_request,
                         CanasServiceResponseCallbackFn callback_response, void* pstate);
int canasServiceUnregister(CanasInstance* pi, uint8_t service_code);
int canasServiceSetState(CanasInstance* pi, uint8_t service_code, void* pstate);
int canasServiceGetState(CanasInstance* pi, uint8_t service_code, void** ppstate);
/**
 * @}
 */

/**
 * Dump a CAN frame for humans.
 * @param [in]  pframe Pointer to frame to be dumped
 * @param [out] pbuf   Pointer to output string buffer of size @ref CANAS_DUMP_BUF_LEN
 * @return             pbuf
 */
char* canasDumpCanFrame(const CanasCanFrame* pframe, char* pbuf);

/**
 * Dump a CANaerospace message for humans.
 * @param [in]  pframe Pointer to message to be dumped
 * @param [out] pbuf   Pointer to output string buffer of size @ref CANAS_DUMP_BUF_LEN
 * @return             pbuf
 */
char* canasDumpMessage(const CanasMessage* pmsg, char* pbuf);

/**
 * Convenience functions for Node Service Protocol services.
 * They are just wrappers over corresponding function pointers in the configuration structure.
 * @{
 */
uint64_t canasTimestamp(CanasInstance* pi);
void* canasMalloc(CanasInstance* pi, int size);
void canasFree(CanasInstance* pi, void* ptr);
/**
 * @}
 */

#ifdef __cplusplus
}
#endif
#endif
