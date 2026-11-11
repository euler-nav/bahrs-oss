/*
 * The standard Flash Programming Service.
 * Pavel Kirienko, 2013 (pavel.kirienko@gmail.com)
 */

#ifndef CANAEROSPACE_SERVICES_STD_FLASHPROG_H_
#define CANAEROSPACE_SERVICES_STD_FLASHPROG_H_

#include <stdbool.h>
#include "../canaerospace.h"

#ifdef __cplusplus
extern "C" {
#endif

static const int8_t CANAS_SRV_FPS_RESULT_OK = 0;
static const int8_t CANAS_SRV_FPS_RESULT_ABORT = -1;
static const int8_t CANAS_SRV_FPS_RESULT_INVALID_SECURITY_CODE = -3;

/**
 * This callback is called when remote node requests FPS on local node.
 * @param pi            Instance pointer
 * @param security_code Security code provided by remote node. Must be matched against local SC by application.
 * @return              Response Code to be returned to Remote Node. @ref CANAS_SRV_FPS_RESULT_OK and others.
 */
typedef int8_t (*CanasSrvFpsRequestCallback)(CanasInstance* pi, uint8_t security_code);

/**
 * This callback is called when remote node responds to FPS request, or when request is timed out.
 * @param pi                    Instance pointer
 * @param node_id               Responding Node ID
 * @param timed_out             Is request timed out or not
 * @param signed_response_code  Contains response code if response received; non-defined in case of timeout
 * @param parg                  Callback argument
 */
typedef void (*CanasSrvFpsResponseCallback)(CanasInstance* pi, uint8_t node_id, bool timed_out,
    int8_t signed_response_code, void* parg);

/**
 * Initializes the Flash Programming Service. This service is optional.
 * @param pi       Instance pointer
 * @param callback Callback to be invoked when incoming FPS request received. Optional, may be NULL.
 * @return         @ref CanasErrorCode
 */
int canasSrvFpsInit(CanasInstance* pi, CanasSrvFpsRequestCallback callback);

/**
 * Send FPS request to a remote node.
 * Neither concurrent nor broadcast requests are allowed for this service; only one node can be requested at once.
 * @param pi            Instance pointer
 * @param node_id       Target Node ID
 * @param security_code Security code for the target node
 * @param callback      Callback to be invoked when either response received or timeout expired
 * @param callback_arg  Argument to be passed to the callback
 * @return              @ref CanasErrorCode
 */
int canasSrvFpsRequest(CanasInstance* pi, uint8_t node_id, uint8_t security_code, CanasSrvFpsResponseCallback callback,
                       void* callback_arg);

#ifdef __cplusplus
}
#endif
#endif
