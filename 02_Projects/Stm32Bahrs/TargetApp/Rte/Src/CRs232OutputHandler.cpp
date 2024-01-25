/**
 * @file CRs323OutputHandler.cpp
 * @brief Implementation of the RS232 Output Handler software component.
 * @author Fedor Baklanov
 * @date 07 June 2022
 */

#include "CRs232OutputHandler.h"
#include "CRte.h"
#include "usart.h"
#include "GetMicroseconds.h"

extern osMessageQueueId_t QueueTaskRs232SenderHandle;

CRs232OutputHandler& CRs232OutputHandler::getInstanceImpl(unsigned uInstanceIndex)
{
  static CRs232OutputHandler soInstance;
  assert(uInstanceIndex == 0U);
  return soInstance;
}

void CRs232OutputHandler::Init()
{
  // Do nothing
}

bool CRs232OutputHandler::IsInitialized()
{
  return true;
}

void CRs232OutputHandler::QueueTransmissionRequest(CSerialProtocol::EMessageIds eMessageId)
{
  uint8_t uMessageToSend = static_cast<uint8_t>(eMessageId);
  osMessageQueuePut(QueueTaskRs232SenderHandle, &uMessageToSend, 0U, 0U);
}

void CRs232OutputHandler::SendInertialDataMessage()
{
  CRte& orRte = CRte::GetInstance();
  SOutputImuData oImuData;
  CSerialProtocol::SInertialDataMessage oMessage;

  bool bPortReadStatus = orRte.oPortImuOutput_.Read(oImuData);

  if (true == bPortReadStatus)
  {
    oMessage = oProtocol_.BuildInertialDataMessage(oImuData);
    transmitMessage(reinterpret_cast<uint8_t*>(&oMessage), sizeof(oMessage));
  }
}

void CRs232OutputHandler::SendTimeOfInertialDataMessage()
{
  CRte& orRte = CRte::GetInstance();
  SOutputImuData oImuData;
  CSerialProtocol::STimeOfInertialDataMessage oMessage;

  bool bPortReadStatus = orRte.oPortImuOutput_.Read(oImuData);

  if (true == bPortReadStatus)
  {
    oMessage = oProtocol_.BuildTimeOfInertialDataMessage(oImuData);
    transmitMessage(reinterpret_cast<uint8_t*>(&oMessage), sizeof(oMessage));
  }
}

void CRs232OutputHandler::SendNavigationDataMessage()
{
  CSerialProtocol::SNavigationDataMessage oMessage;
  NBahrsFilterApi::SOutputData oBahrsData;

  bool bReadStatus = CRte::GetInstance().oPortBahrsFilterOutput_.Read(oBahrsData);

  if (true == bReadStatus)
  {
    oMessage = oProtocol_.BuildNavigationDataMessage(oBahrsData);
    transmitMessage(reinterpret_cast<uint8_t*>(&oMessage), sizeof(oMessage));
  }
}

void CRs232OutputHandler::SendTimeOfNavigationDataMessage()
{
  CSerialProtocol::STimeOfNavigationDataMessage oMessage;
  NBahrsFilterApi::SOutputData oBahrsData;

  bool bReadStatus = CRte::GetInstance().oPortBahrsFilterOutput_.Read(oBahrsData);

  if (true == bReadStatus)
  {
    oMessage = oProtocol_.BuildTimeOfNavigationDataMessage(oBahrsData);
    transmitMessage(reinterpret_cast<uint8_t*>(&oMessage), sizeof(oMessage));
  }
}

void CRs232OutputHandler::SendAccuracyDataMessage()
{
  CSerialProtocol::SAccuracyDataMessage oMessage;
  NBahrsFilterApi::SOutputData oBahrsData;

  bool bReadStatus = CRte::GetInstance().oPortBahrsFilterOutput_.Read(oBahrsData);

  if (true == bReadStatus)
  {
    oMessage = oProtocol_.BuildAccuracyDataMessage(oBahrsData);
    transmitMessage(reinterpret_cast<uint8_t*>(&oMessage), sizeof(oMessage));
  }
}

void CRs232OutputHandler::SendTimeOfLatestSyncPulseMessage()
{
  uint64_t uTimestampUs;
  bool bReadStatus = CRte::GetInstance().oPortSyncPulseTime_.Read(uTimestampUs);

  if (true == bReadStatus)
  {
    CSerialProtocol::STimeOfLatestSyncPulseMessage oMessage = oProtocol_.BuildTimeOfLatestSyncPulseMessage(uTimestampUs);
    transmitMessage(reinterpret_cast<uint8_t*>(&oMessage), sizeof(oMessage));
  }
}

void CRs232OutputHandler::SendSoftwareVersionMessage()
{
  CSerialProtocol::SSoftwareVersionMessage oMessage = oProtocol_.BuildSoftwareVersionMessage();
  transmitMessage(reinterpret_cast<uint8_t*>(&oMessage), sizeof(oMessage));
}

void CRs232OutputHandler::transmitMessage(uint8_t* upData, uint16_t uSize)
{
  assert((uSize > 0) && (uSize <= skuBufferLength_));

  if (HAL_UART_STATE_READY == huart1.gState)
  {
    // UART is ready, we can send data immediately
    if ((uNumberOfBytesToSend_ + uSize) <= skuBufferLength_)
    {
      // Append the message to the current data slot and send
      memcpy(reinterpret_cast<void*>(&(auTxBuffer_[uCurrentBufferSlot_][uNumberOfBytesToSend_])), reinterpret_cast<void*>(upData), uSize);
      HAL_UART_Transmit_DMA(&huart1, reinterpret_cast<uint8_t*>(auTxBuffer_[uCurrentBufferSlot_]), uNumberOfBytesToSend_ + uSize);
      incrementBufferSlot();
    }
    else
    {
      // Send accumulated data and put the new message to the next buffer slot
      HAL_UART_Transmit_DMA(&huart1, reinterpret_cast<uint8_t*>(auTxBuffer_[uCurrentBufferSlot_]), uNumberOfBytesToSend_);
      incrementBufferSlot();
      memcpy(reinterpret_cast<void*>(auTxBuffer_[uCurrentBufferSlot_]), reinterpret_cast<void*>(upData), uSize);
      uNumberOfBytesToSend_ = uSize;
    }
  }
  else
  {
    // Transmission is already ongoing, so we need to put the message into the buffer
    if ((uNumberOfBytesToSend_ + uSize) <= skuBufferLength_)
    {
      memcpy(reinterpret_cast<void*>(&(auTxBuffer_[uCurrentBufferSlot_][uNumberOfBytesToSend_])), reinterpret_cast<void*>(upData), uSize);
      uNumberOfBytesToSend_ += uSize;
    }
    else
    {
      // It seems that we try to transmit too much data
      assert(false);
    }
  }
}

void CRs232OutputHandler::incrementBufferSlot()
{
  ++uCurrentBufferSlot_;

  if (uCurrentBufferSlot_ >= skuMessageBufferLength_)
  {
    uCurrentBufferSlot_ = 0U;
  }

  uNumberOfBytesToSend_ = 0U;
}

