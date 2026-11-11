/*
 * CANaerospace Node Service Protocol support
 * Pavel Kirienko, 2013 (pavel.kirienko@gmail.com)
 */

#ifndef CANAEROSPACE_SERVICE_H_
#define CANAEROSPACE_SERVICE_H_

#include <stdbool.h>
#include <canaerospace/canaerospace.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * At least 10ms is required by some services.
 * Thus, increasing is NOT recommended.
 */
static const int CANAS_DEFAULT_SERVICE_POLL_INTERVAL_USEC = 10 * 1000;

/**
 * History length for repetition detection.
 * Expressed in number of frames to track.
 */
static const int CANAS_DEFAULT_SERVICE_HIST_LEN = 32;

/**
 * Time to wait for response from remote node
 */
static const int CANAS_DEFAULT_SERVICE_REQUEST_TIMEOUT_USEC = 100 * 1000;

bool canasIsInterestingServiceMessage(CanasInstance* pi, uint16_t msg_id, CanasMessage *pmsg);

void canasHandleReceivedService(CanasInstance* pi, CanasServiceSubscription* psrv, uint8_t iface, uint16_t msg_id,
                                CanasMessage *pmsg, const CanasCanFrame* pframe, uint64_t timestamp_usec);

void canasPollServices(CanasInstance* pi, uint64_t timestamp_usec);

int canasServiceChannelToMessageID(uint8_t service_channel, bool isrequest);

bool canasIsValidServiceChannel(uint8_t service_channel);

#ifdef __cplusplus
}
#endif
#endif
