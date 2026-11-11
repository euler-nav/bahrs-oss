/// @file CDroneCanHandler.h
/// @brief Declaration of the CDroneCanHandler class (DroneCAN handler software component).
///
/// CDroneCanHandler owns the libcanard instance and its static memory pool.
/// It provides:
///  - Broadcast()       - enqueue a DroneCAN transfer into the libcanard TX queue.
///  - TransmitFrames()  - dequeue pending frames and send them via the bxCAN driver.
///  - ReceiveFrames()   - (static) poll received CAN frames, feed them into libcanard.
///
/// The low-level CAN access goes through the minimal polled bxCAN driver (bxcan.h):
/// BxCan_Init() / BxCan_Send() / BxCan_Receive(). DroneCAN uses 29-bit extended
/// frame IDs (CANARD_CAN_FRAME_EFF).
///
/// Thread safety: a single FreeRTOS mutex serialises all libcanard API calls.
/// Broadcast() and TransmitFrames() must be called from the CAN sender task.
/// ReceiveFrames() must be called from the CAN receiver task.
///
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef C_DRONE_CAN_HANDLER_H
#define C_DRONE_CAN_HANDLER_H

#include "canardWrapper.h"
#include "GetMicroseconds.h"
#include "cmsis_os.h"
#include "General/CSoftwareComponentBase.h"

typedef StaticSemaphore_t osStaticMutexDef_t;

/// @brief DroneCAN General Handler software component.
class CDroneCanHandler : public CSoftwareComponent<CDroneCanHandler, 1U>
{
  friend class CSoftwareComponent<CDroneCanHandler, 1U>;
  FORBID_CLASS_COPY_AND_MOVE(CDroneCanHandler)
  DECLARE_MANDATORY_APIS(CDroneCanHandler)

public:
  static constexpr uint32_t skuMemoryPoolSize { 1024U }; ///< libcanard memory pool in bytes
  static constexpr uint32_t skuMutexTout      { 1U };    ///< Mutex acquire timeout in OS ticks

  /// @brief Enqueue a DroneCAN broadcast transfer into the libcanard TX queue.
  ///
  /// Thread-safe wrapper around canardBroadcast().  Call this from the CAN sender
  /// task, then call TransmitFrames() to push the queued frames onto the bus.
  ///
  /// @param [in]     uDataTypeSignature  64-bit DSDL data type signature.
  /// @param [in]     uDataTypeId         DSDL data type ID.
  /// @param [in,out] upTransferId        Persistent rolling transfer ID counter (per message type).
  /// @param [in]     uPriority           Transfer priority (CANARD_TRANSFER_PRIORITY_*).
  /// @param [in]     kpPayload           Serialized payload buffer.
  /// @param [in]     uPayloadLen         Payload length in bytes.
  /// @return                             Number of frames enqueued (>=1), or negative error code.
  int16_t Broadcast(uint64_t uDataTypeSignature,
                    uint16_t uDataTypeId,
                    uint8_t* upTransferId,
                    uint8_t  uPriority,
                    const void* kpPayload,
                    uint16_t uPayloadLen);

  /// @brief Drain the libcanard TX queue and send each frame via the bxCAN driver.
  ///
  /// Must be called from the CAN sender task after one or more Broadcast() calls.
  /// Stops early if the bxCAN TX FIFO is full; remaining frames stay in the
  /// libcanard queue and will be sent on the next invocation.
  void TransmitFrames();

  /// @brief Poll the CAN RX FIFO and feed received frames into libcanard.
  ///
  /// Drains RX FIFO0 by polling (the legacy interrupt/queue path is bypassed) and
  /// yields briefly. Call repeatedly from a dedicated CAN receiver task. libcanard
  /// reassembles multi-frame transfers internally and invokes onTransferReception_()
  /// when complete.
  static void ReceiveCanFrames();

private:
  CDroneCanHandler();
  ~CDroneCanHandler() = default;

  CanardInstance oCanardInstance_;                                            ///< libcanard instance
  uint8_t auMemoryPool_[skuMemoryPoolSize] __attribute__((aligned(__BIGGEST_ALIGNMENT__))); ///< Static memory arena
  osMutexId_t    pMutex_;                                                     ///< FreeRTOS mutex
  osStaticMutexDef_t oMutexCb_;                                               ///< Static mutex control block
  osMutexAttr_t  oMutexAttr_;                                                 ///< Mutex init attributes

  bool bIsInitialized_ { false };

  /// @brief libcanard callback - invoked when a complete transfer has been received.
  ///
  /// Handles uavcan.protocol.GetNodeInfo service requests by replying with the
  /// node's identity (name, software/hardware version, unique ID). All other
  /// accepted transfers are ignored.
  ///
  /// @param [in] opIns      libcanard instance.
  /// @param [in] opTransfer Completed incoming transfer.
  static void onTransferReception_(CanardInstance* opIns, CanardRxTransfer* opTransfer);

  /// @brief libcanard callback - decide whether to accept an incoming transfer.
  ///
  /// Accepts uavcan.protocol.GetNodeInfo service requests (setting the matching
  /// data type signature) and rejects everything else.
  ///
  /// @param [in]  kopIns                 libcanard instance (const).
  /// @param [out] oupDataTypeSignature   Must be set to the DSDL signature when returning true.
  /// @param [in]  uDataTypeId            Incoming data type ID.
  /// @param [in]  eTransferType          Transfer type.
  /// @param [in]  uSourceNodeId          Sender node ID.
  /// @return                             true to accept, false to discard.
  static bool shouldAcceptTransfer_(const CanardInstance* kopIns,
                                    uint64_t* oupDataTypeSignature,
                                    uint16_t  uDataTypeId,
                                    CanardTransferType eTransferType,
                                    uint8_t   uSourceNodeId);
};

#endif // C_DRONE_CAN_HANDLER_H
