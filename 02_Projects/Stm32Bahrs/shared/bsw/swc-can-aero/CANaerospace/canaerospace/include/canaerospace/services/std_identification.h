/*
 * The standard Identification Service.
 * Pavel Kirienko, 2013 (pavel.kirienko@gmail.com)
 */

#ifndef CANAEROSPACE_SERVICES_STD_IDENTIFICATION_H_
#define CANAEROSPACE_SERVICES_STD_IDENTIFICATION_H_

#include "../canaerospace.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    CANAS_SRV_IDS_ID_DISTRIBUTION_STD = 0
} CanasSrvIdsIdDistribution;

typedef enum
{
    CANAS_SRV_IDS_HEADER_TYPE_STD = 0
} CanasSrvIdsHeaderType;

static const int CANAS_SRV_IDS_MAX_PENDING_REQUESTS = 255;

/**
 * Data to be transferred by this service
 */
typedef struct
{
    uint8_t hardware_revision;
    uint8_t software_revision;
    uint8_t id_distribution;
    uint8_t header_type;
} CanasSrvIdsPayload;

/**
 * This callback is called when remote node responds to IDS request
 * @param pi       Instance pointer
 * @param node_id  Node ID of responding node
 * @param ppayload Will be NULL in case of timeout. That is important.
 */
typedef void (*CanasSrvIdsResponseCallback)(CanasInstance* pi, uint8_t node_id, CanasSrvIdsPayload* ppayload);

/**
 * Initializes standard Identification Service.
 * Note that this service must be available on every CANaerospace node (as required by specification),
 * so you should not left it uninitialized.
 * Also keep in mind that outgoing broadcast request creates CANAS_MAX_NODES pending requests, so you must
 * provide adequate number of pending request slots; see the corresponding parameter @ref max_pending_requests.
 * @param pi                   Instance pointer
 * @param pself_definition     Description of the local node
 * @param max_pending_requests Max number of simultaneous outgoing IDS requests. May be zero.
 * @return                     @ref CanasErrorCode
 */
int canasSrvIdsInit(CanasInstance* pi, const CanasSrvIdsPayload* pself_definition, uint8_t max_pending_requests);

/**
 * Sends IDS request to a remote node; response will be returned through callback.
 * @param pi       Instance pointer
 * @param node_id  Remote Node ID (node to be requested)
 * @param callback Function to be called when request completed or timed out. Required.
 * @return         @ref CanasErrorCode
 */
int canasSrvIdsRequest(CanasInstance* pi, uint8_t node_id, CanasSrvIdsResponseCallback callback);

#ifdef __cplusplus
}
#endif
#endif
