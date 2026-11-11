/*
 * Pavel Kirienko, 2013 (pavel.kirienko@gmail.com)
 */

#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <float.h>
#include <canaerospace/generic_redundancy_resolver.h>
#include "debug.h"

static bool _isConfigOk(const CanasGrrConfig* pcfg)
{
    if (pcfg == NULL)
        return false;

    if (pcfg->channel_timeout_usec < 1)
        return false;

    if (!isfinite(pcfg->fom_hysteresis) || pcfg->fom_hysteresis < 0.0f)
        return false;

    if (pcfg->num_channels < 1)
        return false;

    // Both hysteresis and time constraint are disabled, this is likely to be wrong
    if (pcfg->fom_hysteresis == 0.0f && pcfg->min_fom_switch_interval_usec == 0)
        return false;

    return true;
}

CanasGrrConfig canasGrrMakeConfig(void)
{
    CanasGrrConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.fom_hysteresis = nan("");
    return cfg;
}

int canasGrrInit(CanasGrrInstance* pgrr, const CanasGrrConfig* pcfg, CanasInstance* pcanas)
{
    if (pgrr == NULL || pcfg == NULL || pcanas == NULL)
        return -CANAS_ERR_ARGUMENT;
    if (!_isConfigOk(pcfg))
        return -CANAS_ERR_ARGUMENT;

    memset(pgrr, 0, sizeof(*pgrr));
    pgrr->config = *pcfg;
    pgrr->pcanas = pcanas;

    const size_t channels_size = sizeof(CanasGrrChannelState) * pgrr->config.num_channels;
    pgrr->channels = canasMalloc(pcanas, channels_size);
    memset(pgrr->channels, 0, channels_size);

    return 0;
}

int canasGrrDispose(CanasGrrInstance* pgrr)
{
    if (pgrr == NULL)
        return -CANAS_ERR_ARGUMENT;
    canasFree(pgrr->pcanas, pgrr->channels);
    memset(pgrr, 0, sizeof(*pgrr));   // Erasing pointers and config
    return 0;
}

int canasGrrGetActiveChannel(CanasGrrInstance* pgrr)
{
    if (pgrr == NULL)
        return -CANAS_ERR_ARGUMENT;
    return pgrr->active_channel;
}

int canasGrrOverrideActiveChannel(CanasGrrInstance* pgrr, uint8_t redund_chan)
{
    if (pgrr == NULL || redund_chan >= pgrr->config.num_channels)
        return -CANAS_ERR_ARGUMENT;
    pgrr->active_channel = redund_chan;
    pgrr->last_switch_timestamp_usec = canasTimestamp(pgrr->pcanas);
    return 0;
}

int canasGrrGetLastSwitchTimestamp(CanasGrrInstance* pgrr, uint64_t* ptimestamp)
{
    if (pgrr == NULL || ptimestamp == NULL)
        return -CANAS_ERR_ARGUMENT;
    *ptimestamp = pgrr->last_switch_timestamp_usec;
    return 0;
}

int canasGrrGetChannelState(CanasGrrInstance* pgrr, uint8_t redund_chan, float* pfom, uint64_t* ptimestamp)
{
    if (pgrr == NULL || redund_chan >= pgrr->config.num_channels)
        return -CANAS_ERR_ARGUMENT;

    if (pfom)
        *pfom = pgrr->channels[redund_chan].fom;

    if (ptimestamp)
        *ptimestamp = pgrr->channels[redund_chan].last_update_timestamp_usec;

    return 0;
}

int canasGrrUpdate(CanasGrrInstance* pgrr, uint8_t redund_chan, float fom, uint64_t timestamp)
{
    if (pgrr == NULL || redund_chan >= pgrr->config.num_channels || timestamp == 0)
        return -CANAS_ERR_ARGUMENT;

    if (isnan(fom))
        fom = -FLT_MAX; // NAN will be interpreted as minimum possible finite FOM value (-inf is still less)

    pgrr->channels[redund_chan].fom = fom;
    pgrr->channels[redund_chan].last_update_timestamp_usec = timestamp;

    const CanasGrrChannelState* updating_chan = pgrr->channels + redund_chan;
    const CanasGrrChannelState* active_chan = pgrr->channels + pgrr->active_channel;

    CanasGrrSwitchReason reason = CANAS_GRR_REASON_NONE;

    /*
     * Initial switch
     */
    if (pgrr->last_switch_timestamp_usec == 0)
        reason = CANAS_GRR_REASON_INIT;

    /*
     * By timeout
     */
    if (reason == CANAS_GRR_REASON_NONE && updating_chan != active_chan)
    {
        const uint64_t time_threshold = active_chan->last_update_timestamp_usec + pgrr->config.channel_timeout_usec;
        if (updating_chan->last_update_timestamp_usec > time_threshold)
            reason = CANAS_GRR_REASON_TIMEOUT;
    }

    /*
     * By figure of merit
     */
    if (reason == CANAS_GRR_REASON_NONE && updating_chan != active_chan)
    {
        const float fom_threshold = active_chan->fom + pgrr->config.fom_hysteresis;
        const uint64_t fom_switch_dead_time =
            pgrr->last_switch_timestamp_usec + pgrr->config.min_fom_switch_interval_usec;

        if ((updating_chan->fom > fom_threshold) && (timestamp >= fom_switch_dead_time))
            reason = CANAS_GRR_REASON_FOM;
    }

    if (reason != CANAS_GRR_REASON_NONE)
    {
        CANAS_TRACE("grr: selecting better alternative: %i[%f] --> %i[%f], reason: %i\n",
            (int)pgrr->active_channel, active_chan->fom, (int)redund_chan, updating_chan->fom, reason);
        pgrr->active_channel = redund_chan;
        pgrr->last_switch_timestamp_usec = timestamp;
    }
    return reason;
}
