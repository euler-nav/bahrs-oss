/**
 * @file CRs323OutputHandler.h
 * @brief Declaration of the CRs232OutputHandler class (RS232 output handler software component).
 * @author Fedor Baklanov
 * @date 07 June 2022
 */

#ifndef C_OUTPUT_HANDLER_H
#define C_OUTPUT_HANDLER_H

#include "CSerialProtocol.h"
#include "General/CSoftwareComponentBase.h"

/**
 * @brief RS232 Output Handler software component.
*/
class CRs232OutputHandler : public CSoftwareComponent<CRs232OutputHandler, 1U>
{
  friend class CSoftwareComponent<CRs232OutputHandler, 1U>;
  FORBID_CLASS_COPY_AND_MOVE(CRs232OutputHandler)
  DECLARE_MANDATORY_APIS(CRs232OutputHandler)

public:
  /**
   * \brief Put message transmission request into the queue of the sender task.
   * The function does not make a snapshot of the data to be sent.
   * \param eMessageId
   */
  void QueueTransmissionRequest(CSerialProtocol::EMessageIds eMessageId);

  /**
   * @brief Compose and send an inertial data message.
  */
  void SendInertialDataMessage();

  /**
   * @brief Compose and send a "time of inertial data" message.
  */
  void SendTimeOfInertialDataMessage();

  /**
   * @brief Compose and send a navigation data message.
  */
  void SendNavigationDataMessage();

  /**
   * @brief Compose and send a "time of navigation data" message.
  */
  void SendTimeOfNavigationDataMessage();

  /**
   * @brief Compose and send navigation data accuracy.
  */
  void SendAccuracyDataMessage();

  /**
   * @brief Compose and send the time of the latest pulse.
  */
  void SendTimeOfLatestSyncPulseMessage();

  /**
   * @brief Compose and send the software version message.
  */
  void SendSoftwareVersionMessage();

protected:

private:
  CRs232OutputHandler () = default;
  ~CRs232OutputHandler () = default;

  CSerialProtocol oProtocol_; ///< Serial protocol object that builds messages from generic inputs.
  static constexpr uint32_t skuBufferLength_ { 256U }; ///< Maximum allowed message length in bytes
  static constexpr uint32_t skuMessageBufferLength_ { 2U }; ///< Max number of messages in DMA memory buffer.
  uint8_t auTxBuffer_[skuMessageBufferLength_][skuBufferLength_]; ///< A buffer for DMA transfer.
  uint8_t uCurrentBufferSlot_ { 0U }; ///< Output buffer slot is use.
  uint16_t uNumberOfBytesToSend_ { 0U }; ///< Number of bytes in the current output buffer slot.

  /**
   *  @brief Transmit data using DMA:
   * \param upData Pointer to data.
   * \param uSize Number of bytes to be sent.
   */
  void transmitMessage(uint8_t* upData, uint16_t uSize);

  /**
   * Increment index of the current data slot in the output buffer. Resets the index to zero if it exceeds
   * the reserved number of slots. Resets number of bytes to send to zero on every call.
   */
  void incrementBufferSlot();
};

#endif /* C_OUTPUT_HANDLER_H */
