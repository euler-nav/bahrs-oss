/// @file CRs232OutputHandler.h
/// @brief Declaration of the CRs232OutputHandler class (RS232 output handler software component).
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef C_OUTPUT_HANDLER_H
#define C_OUTPUT_HANDLER_H

#include <limits>

#include "CSerialMessageBuilder.h"
#include "General/CSoftwareComponentBase.h"
#include "cmsis_os.h"

/// @brief RS232 Output Handler software component.
class CRs232OutputHandler : public CSoftwareComponent<CRs232OutputHandler, 1U>
{
  friend class CSoftwareComponent<CRs232OutputHandler, 1U>;
  FORBID_CLASS_COPY_AND_MOVE(CRs232OutputHandler)
  DECLARE_MANDATORY_APIS(CRs232OutputHandler)

public:
  /// @brief Compose and send an inertial data message.
  /// @note Does nothing if SEND_DEBUG_OUTPUT is defined.
  void SendInertialDataMessage();

  /// @brief Compose and send a "time of inertial data" message.
  /// @note Does nothing if SEND_DEBUG_OUTPUT is defined.
  void SendTimeOfInertialDataMessage();

  /// @brief Compose and send a navigation data message.
  /// @note Does nothing if SEND_DEBUG_OUTPUT is defined.
  void SendNavigationDataMessage();

  /// @brief Compose and send a "time of navigation data" message.
  /// @note Does nothing if SEND_DEBUG_OUTPUT is defined.
  void SendTimeOfNavigationDataMessage();

  /// @brief Compose and send navigation data accuracy.
  /// @note Does nothing if SEND_DEBUG_OUTPUT is defined.
  void SendAccuracyDataMessage();

  /// @brief Compose and send the time of the latest pulse.
  /// @note Does nothing if SEND_DEBUG_OUTPUT is defined.
  void SendTimeOfLatestSyncPulseMessage();

  /// @brief Compose and send the software version message.
  void SendSoftwareVersionMessage();

  /// @brief Compose and send the hardware version message.
  void SendHardwareVersionMessage();

  /// @brief Transmit data using DMA.
  /// @param upData Pointer to data.
  /// @param uSize Number of bytes to be sent.
  /// @return True if the packet was allocated successfully. False otherwise.
  bool TransmitMessage(uint8_t* upData, size_t uSize);

  /// @brief DMA Transmission completed callback.
  /// @param uSize Number of bytes sent.
  void UARTTxInterruptHandler(uint16_t uSize);

#ifdef SEND_DEBUG_OUTPUT
  /// @brief Send debug message.
  /// @param korDebugData Reference to debug data.
  void SendDebugData(const CSerialProtocol::SDebugData& korDebugData);
#endif // SEND_DEBUG_OUTPUT

protected:

private:
  CRs232OutputHandler () = default;
  ~CRs232OutputHandler () = default;

  CSerialMessageBuilder oMessageBuilder_; ///< An object that builds messages from generic inputs.

#ifndef SEND_DEBUG_OUTPUT
  static constexpr uint32_t skuBufferLength_ { 512U }; ///< Maximum allowed message length in bytes
#else
  static constexpr uint32_t skuBufferLength_ { 4096U }; ///< Maximum allowed message length in bytes
#endif // SEND_DEBUG_OUTPUT

  static_assert(skuBufferLength_ <= std::numeric_limits<uint16_t>::max(), "UART DMA buffer size must fit uint16_t");

  uint8_t auTxBuffer_[skuBufferLength_]; ///< A buffer for DMA transfer.
  uint32_t uTxBufferRdIdx_ { 0U }; ///< Output buffer read index.
  uint32_t uTxBufferWrIdx_ { 0U }; ///< Output buffer write index.

  /// @brief Checks whether the DMA buffer is able to allocate the amount of bytes that
  ///        we wish to enqueue.
  /// @param uSize Number of bytes sent.
  bool isBufferFree(size_t uSize);

  /// The typedef copied from a CMSIS_OS header.
  typedef StaticSemaphore_t osStaticMutexDef_t;

  static constexpr uint32_t skuMutexTimeoutTicks_ { 2U }; ///< Mutex acquisition timeout in RTOS ticks.

  osMutexId_t pMutexHandle_ { nullptr }; ///< Mutex handle.
  osStaticMutexDef_t sMutexControlBlock_; ///< Memory reserved for the mutex control block.
  const osMutexAttr_t sMutexAttributes_ { NULL, 0, &sMutexControlBlock_, sizeof(sMutexControlBlock_) }; ///< A structure with mutex attributes required for creation.
};

#endif // C_OUTPUT_HANDLER_H
