/*
 * Standard Node Synchronization Service
 * Pavel Kirienko, 2013 (pavel.kirienko@gmail.com)
 */

#ifndef CANAEROSPACE_SERVICES_STD_NODESYNC_H_
#define CANAEROSPACE_SERVICES_STD_NODESYNC_H_

#include "../canaerospace.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Application can receive incoming NSS data via a callback of this type.
 * @param [in] pi        Instance pointer
 * @param [in] timestamp Received timestamp
 */
typedef void (*CanasSrvNssCallback)(CanasInstance* pi, uint32_t timestamp);

/**
 * Init NSS service.
 * @param [in] pi       Instance pointer
 * @param [in] callback Callback to be called on incoming NSS request
 * @return              @ref CanasErrorCode
 */
int canasSrvNssInit(CanasInstance* pi, CanasSrvNssCallback callback);

/**
 * Send broadcast NSS request.
 * @param [in] pi        Instance pointer
 * @param [in] timestamp Timestamp to transmit
 * @return               @ref CanasErrorCode
 */
int canasSrvNssPublish(CanasInstance* pi, uint32_t timestamp);

#ifdef __cplusplus
}
#endif
#endif
