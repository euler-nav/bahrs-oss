/// @file CRs232OutputHandler.h
/// @brief Declaration of the CRs232OutputHandler class (RS232 output handler software component).
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef C_OUTPUT_HANDLER_H
#define C_OUTPUT_HANDLER_H

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
  /// @brief Put message transmission request into the queue of the sender task.
  /// The function does not make a snapshot of the data to be sent.
  /// @param eMessageId Id of the message to be sent.
  void QueueTransmissionRequest(CSerialProtocol::EMessageIds eMessageId);

  /// @brief Compose and send an inertial data message.
  void SendInertialDataMessage();

  /// @brief Compose and send a "time of inertial data" message.
  void SendTimeOfInertialDataMessage();

  /// @brief Compose and send a navigation data message.
  void SendNavigationDataMessage();

  /// @brief Compose and send a "time of navigation data" message.
  void SendTimeOfNavigationDataMessage();

  /// @brief Compose and send navigation data accuracy.
  void SendAccuracyDataMessage();

  /// @brief Compose and send the time of the latest pulse.
  void SendTimeOfLatestSyncPulseMessage();

  /// @brief Compose and send the software version message.
  void SendSoftwareVersionMessage();

  /// @brief Compose and send the hardware version message.
  void SendHardwareVersionMessage();

  /// @brief Transmit data using DMA.
  /// @param upData Pointer to data.
  /// @param uSize Number of bytes to be sent.
  /// @return True if the packet was allocated successfully. False otherwise.
  bool TransmitMessage(uint8_t* upData, uint16_t uSize);

  /// @brief DMA Transmission completed callback.
  /// @param uSize Number of bytes sent.
  void UARTTxInterruptHandler(uint16_t uSize);

protected:

private:
  CRs232OutputHandler () = default;
  ~CRs232OutputHandler () = default;

  CSerialMessageBuilder oMessageBuilder_; ///< An object that builds messages from generic inputs.
  static constexpr uint32_t skuBufferLength_ { 512U }; ///< Maximum allowed message length in bytes

  uint8_t auTxBuffer_[skuBufferLength_]; ///< A buffer for DMA transfer.
  uint32_t uTxBufferRdIdx_ { 0U }; ///< Output buffer read index.
  uint32_t uTxBufferWrIdx_ { 0U }; ///< Output buffer write index.

  /// @brief Checks whether the DMA buffer is able to allocate the amount of bytes that
  ///        we wish to enqueue.
  /// @param uSize Number of bytes sent.
  bool isBufferFree(uint16_t uSize);

  /// The typedef copied from a CMSIS_OS header.
  typedef StaticSemaphore_t osStaticMutexDef_t;

  osMutexId_t pMutexHandle_ { nullptr }; ///< Mutex handle.
  osStaticMutexDef_t sMutexControlBlock_; ///< Memory reserved for the mutex control block.
  const osMutexAttr_t sMutexAttributes_ { NULL, 0, &sMutexControlBlock_, sizeof(sMutexControlBlock_) }; ///< A structure with mutex attributes required for creation.
};

#endif /* C_OUTPUT_HANDLER_H */
