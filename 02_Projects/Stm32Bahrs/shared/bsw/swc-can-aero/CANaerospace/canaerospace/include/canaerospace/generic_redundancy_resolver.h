/*
 * Generic Redundancy Resolver
 * Pavel Kirienko, 2013 (pavel.kirienko@gmail.com)
 */

#ifndef GENERIC_REDUNDANCY_RESOLVER_H_
#define GENERIC_REDUNDANCY_RESOLVER_H_

#include "canaerospace.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    CANAS_GRR_REASON_NONE,   ///< Active channel was not changed
    CANAS_GRR_REASON_INIT,   ///< First update after initialization returns this
    CANAS_GRR_REASON_FOM,    ///< Switched to a channel with better FOM (figure of merit)
    CANAS_GRR_REASON_TIMEOUT ///< Old channel was timed out; the new channel may have worse FOM
} CanasGrrSwitchReason;

typedef struct
{
    uint8_t num_channels;                  ///< Maximum number of redundant channels to track
    float fom_hysteresis;                  ///< FOM (figure of merit) switch hysteresis
    uint64_t min_fom_switch_interval_usec; ///< Min interval between selecting a better FOM (prevents jitter)
    uint64_t channel_timeout_usec;         ///< Channel timeout (ignores @ref min_fom_switch_interval_usec)
} CanasGrrConfig;

typedef struct
{
    float fom;
    uint64_t last_update_timestamp_usec;
} CanasGrrChannelState;

typedef struct
{
    CanasGrrConfig config;
    CanasInstance* pcanas;

    uint64_t last_switch_timestamp_usec;
    uint8_t active_channel;
    CanasGrrChannelState* channels;
} CanasGrrInstance;

/**
 * Make a pre-initialized config struct, with default values provided where possible.
 * @return Pre-initialized @ref CanasGrrConfig.
 */
CanasGrrConfig canasGrrMakeConfig(void);

/**
 * Initialize a GRR instance.
 * @param [out] pgrr   Pointer to GRR instance to be initialized
 * @param [in]  pcfg   Pointer to the instance configuration
 * @param [in]  pcanas Libcanaerospace instance
 * @return             @ref CanasErrorCode
 */
int canasGrrInit(CanasGrrInstance* pgrr, const CanasGrrConfig* pcfg, CanasInstance* pcanas);

/**
 * Safely deinitialize a GRR instance.
 * @param [in] pgrr GRR pointer
 * @return          @ref CanasErrorCode
 */
int canasGrrDispose(CanasGrrInstance* pgrr);

/**
 * Returns currently active channel; 0 by default.
 * @param [in] pgrr GRR pointer
 * @return          Channel index or @ref CanasErrorCode
 */
int canasGrrGetActiveChannel(CanasGrrInstance* pgrr);

/**
 * Forcely switch the active channel.
 * Note that if this function was executed prior first update, the latter will not return @ref CANAS_GRR_REASON_INIT.
 * @param [in] pgrr        GRR pointer
 * @param [in] redund_chan Redundant channel index
 * @return                 @ref CanasErrorCode
 */
int canasGrrOverrideActiveChannel(CanasGrrInstance* pgrr, uint8_t redund_chan);

/**
 * Get the timestamp at which the last channel switch was performed,
 * or 0 if the update call was not issued yet for this GRR instance.
 * @param [in]  pgrr       GRR pointer
 * @param [out] ptimestamp Timestamp or 0
 * @return                 @ref CanasErrorCode
 */
int canasGrrGetLastSwitchTimestamp(CanasGrrInstance* pgrr, uint64_t* ptimestamp);

/**
 * Get the channel state. Any out argument may be NULL.
 * @param [in]  pgrr        GRR pointer
 * @param [in]  redund_chan Redundant channel index
 * @param [out] pfom        Pointer to last figure of merit value
 * @param [out] ptimestamp  Pointer to last update timestamp (will be 0 if there was no update yet)
 * @return                  @ref CanasErrorCode
 */
int canasGrrGetChannelState(CanasGrrInstance* pgrr, uint8_t redund_chan, float* pfom, uint64_t* ptimestamp);

/**
 * Update the GRR state.
 * This function should be called each time when a new message for this redundant function arrives;
 * then the function @ref canasGrrGetActiveChannel can be used to decide whether to use this message or to discard it.
 * @param [in] pgrr        GRR pointer
 * @param [in] redund_chan Redundancy channel index of this message
 * @param [in] fom         Figure of merit of this message
 * @param [in] timestamp   Timestamp of this message, as provided by libcanaerospace
 * @return
 */
int canasGrrUpdate(CanasGrrInstance* pgrr, uint8_t redund_chan, float fom, uint64_t timestamp);

#ifdef __cplusplus
}
#endif
#endif /* GENERIC_REDUNDANCY_RESOLVER_H_ */
