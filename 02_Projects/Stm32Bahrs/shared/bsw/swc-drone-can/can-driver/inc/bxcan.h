/// @file bxcan.h
/// @brief Declaration of minimal STM32 bxCAN driver for the DroneCAN stack.
///
/// Provides only what swc-drone-can needs: bring up the CAN peripheral with an
/// accept-all receive filter, transmit an extended-identifier data frame, and poll
/// a single received frame. Both transmission and reception are polled -- no
/// interrupts, queues or timers are used; the DroneCAN receive task calls
/// BxCan_Receive() in a loop.
///
/// The CAN instance (CAN1 or CAN2) is selected at compile time by
/// CAN_INTERFACE_NUMBER (defaults to CAN2). The CubeMX-generated handle
/// (hcan1/hcan2) is used so that its MSP init (GPIO, clocks, NVIC) is applied.
///
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef DRONE_CAN_BXCAN_H
#define DRONE_CAN_BXCAN_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/// @brief Maximum classic-CAN payload length in bytes.
#define BXCAN_MAX_DLC 8U

/// @brief Initialise the bxCAN peripheral used for DroneCAN.
///
/// Configures 500 kbit/s, installs an accept-all filter routed to RX FIFO0, and
/// starts the peripheral. No interrupts are enabled (TX and RX are polled).
///
/// @return true on success, false on any HAL error.
bool BxCan_Init(void);

/// @brief Queue one extended-identifier data frame for transmission.
///
/// @param [in] uExtId  29-bit extended identifier.
/// @param [in] kpData  Payload bytes (uDlc of them).
/// @param [in] uDlc    Payload length, 0..BXCAN_MAX_DLC.
/// @return true if the frame was placed in a free TX mailbox; false if all
///         mailboxes are busy (the caller should retry later).
bool BxCan_Send(uint32_t uExtId, const uint8_t* kpData, uint8_t uDlc);

/// @brief Poll RX FIFO0 for one received frame.
///
/// @param [out] opuId       Received identifier (extended or standard).
/// @param [out] opbExtended true if the frame used a 29-bit extended identifier.
/// @param [out] opuData     Payload buffer, at least BXCAN_MAX_DLC bytes.
/// @param [out] opuDlc      Payload length in bytes.
/// @return true if a frame was read, false if RX FIFO0 was empty.
bool BxCan_Receive(uint32_t* opuId, bool* opbExtended, uint8_t* opuData, uint8_t* opuDlc);

#ifdef __cplusplus
}
#endif

#endif // DRONE_CAN_BXCAN_H

