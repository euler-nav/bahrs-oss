/// @file CRs232OutputHandler.cpp
/// @brief Implementation of the RS232 Output Handler software component.
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "CRs232OutputHandler.h"
#include "CRte.h"
#include "usart.h"
#include <string.h>

extern osMessageQueueId_t QueueTaskRs232SenderHandle;

static UART_HandleTypeDef& getUartHandle()
{
#ifdef BAHRS_HW_V3
  return huart2;
#else
  return huart1;
#endif
}

CRs232OutputHandler& CRs232OutputHandler::getInstanceImpl(unsigned uInstanceIndex)
{
  static CRs232OutputHandler soInstance;
  assert(uInstanceIndex == 0U);
  return soInstance;
}

void CRs232OutputHandler::Init()
{
  if (nullptr == pMutexHandle_)
  {
    pMutexHandle_ = osMutexNew(&sMutexAttributes_);
  }
}

bool CRs232OutputHandler::IsInitialized()
{
  return (nullptr != pMutexHandle_);
}

void CRs232OutputHandler::QueueTransmissionRequest(CSerialProtocol::EMessageIds eMessageId)
{
  uint8_t uMessageToSend = static_cast<uint8_t>(eMessageId);
  osMessageQueuePut(QueueTaskRs232SenderHandle, &uMessageToSend, 0U, 0U);
}

void CRs232OutputHandler::SendSoftwareVersionMessage()
{
  CSerialProtocol::SSoftwareVersionMessage oMessage = oMessageBuilder_.BuildSoftwareVersionMessage();
  TransmitMessage(reinterpret_cast<uint8_t*>(&oMessage), sizeof(oMessage));
}

void CRs232OutputHandler::SendHardwareVersionMessage()
{
  CSerialProtocol::SHardwareVersionMessage oMessage = oMessageBuilder_.BuildHardwareVersionMessage();
  TransmitMessage(reinterpret_cast<uint8_t*>(&oMessage), sizeof(oMessage));
}

void CRs232OutputHandler::SendInertialDataMessage()
{
  CRte& orRte = CRte::GetInstance();
  SOutputImuData oImuData;
  CSerialProtocol::SInertialDataMessage oMessage;

  bool bPortReadStatus = orRte.oPortImuOutput_.Read(oImuData);

  if (true == bPortReadStatus)
  {
    osStatus_t eStatus = osMutexAcquire(pMutexHandle_, 1);

    if (osOK == eStatus)
    {
      oMessage = oMessageBuilder_.BuildInertialDataMessage(oImuData);
      TransmitMessage(reinterpret_cast<uint8_t*>(&oMessage), sizeof(oMessage));
      osMutexRelease(pMutexHandle_);
    }
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
    osStatus_t eStatus = osMutexAcquire(pMutexHandle_, 1);

    if (osOK == eStatus)
    {
      oMessage = oMessageBuilder_.BuildTimeOfInertialDataMessage(oImuData);
      TransmitMessage(reinterpret_cast<uint8_t*>(&oMessage), sizeof(oMessage));
      osMutexRelease(pMutexHandle_);
    }
  }
}

void CRs232OutputHandler::SendNavigationDataMessage()
{
  CSerialProtocol::SNavigationDataMessage oMessage;
  SSafeVerticalChannelData oVerticalChannelData;
  SSafeAttitudeData oVehicleAttitude;
  SMagneticHeading oMagneticHeading;

  if (false == CRte::GetInstance().oPortSafeVerticalChannelData_.Read(oVerticalChannelData))
  {
    oVerticalChannelData = SSafeVerticalChannelData();
  }

  if (false == CRte::GetInstance().oPortSafeVehicleAttitude_.Read(oVehicleAttitude))
  {
    oVehicleAttitude = SSafeAttitudeData();
  }

  if (false == CRte::GetInstance().oPortMagneticHeading_.Read(oMagneticHeading))
  {
    oMagneticHeading = SMagneticHeading();
  }

  osStatus_t eStatus = osMutexAcquire(pMutexHandle_, 1);

  if (osOK == eStatus)
  {
    oMessage = oMessageBuilder_.BuildNavigationDataMessage(oVerticalChannelData, oVehicleAttitude, oMagneticHeading);
    TransmitMessage(reinterpret_cast<uint8_t*>(&oMessage), sizeof(oMessage));
    osMutexRelease(pMutexHandle_);
  }
}

void CRs232OutputHandler::SendTimeOfNavigationDataMessage()
{
  CSerialProtocol::STimeOfNavigationDataMessage oMessage;
  SSafeAttitudeData oAttitudeData;

  bool bReadStatus = CRte::GetInstance().oPortSafeVehicleAttitude_.Read(oAttitudeData);

  if (true == bReadStatus)
  {
    osStatus_t eStatus = osMutexAcquire(pMutexHandle_, 1);

    if (osOK == eStatus)
    {
      oMessage = oMessageBuilder_.BuildTimeOfNavigationDataMessage(oAttitudeData);
      TransmitMessage(reinterpret_cast<uint8_t*>(&oMessage), sizeof(oMessage));
      osMutexRelease(pMutexHandle_);
    }
  }
}

void CRs232OutputHandler::SendAccuracyDataMessage()
{
  CSerialProtocol::SAccuracyDataMessage oMessage;
  SSafeAttitudeData oVehicleAttitude;

  bool bReadStatus = CRte::GetInstance().oPortSafeVehicleAttitude_.Read(oVehicleAttitude);

  if (true == bReadStatus)
  {
    osStatus_t eStatus = osMutexAcquire(pMutexHandle_, 1);

    if (osOK == eStatus)
    {
      oMessage = oMessageBuilder_.BuildAccuracyDataMessage(oVehicleAttitude);
      TransmitMessage(reinterpret_cast<uint8_t*>(&oMessage), sizeof(oMessage));
      osMutexRelease(pMutexHandle_);
    }
  }
}

void CRs232OutputHandler::SendTimeOfLatestSyncPulseMessage()
{
  STimeOfSyncPulse oTimeOfSyncPulse;
  bool bReadStatus = CRte::GetInstance().oPortSyncPulseTime_.Read(oTimeOfSyncPulse);

  if (true == bReadStatus)
  {
    osStatus_t eStatus = osMutexAcquire(pMutexHandle_, 1);

    if (osOK == eStatus)
    {
      CSerialProtocol::STimeOfLatestSyncPulseMessage oMessage = oMessageBuilder_.BuildTimeOfLatestSyncPulseMessage(oTimeOfSyncPulse.uPulseTimeUs_);
      TransmitMessage(reinterpret_cast<uint8_t*>(&oMessage), sizeof(oMessage));
      osMutexRelease(pMutexHandle_);
    }
  }
}

bool CRs232OutputHandler::TransmitMessage(uint8_t* upData, uint16_t uSize)
{
  bool bReturn = true;
  uint32_t uRemain;
  assert((uSize > 0) && (uSize <= skuBufferLength_));

  taskENTER_CRITICAL();

  /* Check if we have place to allocate the message */
  bReturn = isBufferFree(uSize);
  if (bReturn)
  {
    /* Enqueue the message to the buffer */
    if ((uTxBufferWrIdx_ + uSize) <= skuBufferLength_)
    {
      uRemain = uSize;
      memcpy(static_cast<void*>(&(auTxBuffer_[uTxBufferWrIdx_])), static_cast<void*>(upData), uSize);
    }
    else
    {
      /* Handle buffer roll over */
      uRemain = skuBufferLength_ - uTxBufferWrIdx_;
      memcpy(static_cast<void*>(&(auTxBuffer_[uTxBufferWrIdx_])), static_cast<void*>(upData), uRemain);
      memcpy(static_cast<void*>(auTxBuffer_), static_cast<void*>(upData + uRemain), uSize - uRemain);
    }
    uTxBufferWrIdx_ = (uTxBufferWrIdx_ + uSize) % skuBufferLength_;

    /* If UART DMA is ready means that the buffer was empty before the last insertion */
    /* We directly trigger the DMA transmission and start the flow from here */
    if (HAL_UART_STATE_READY == getUartHandle().gState)
    {
      /* UART is ready, we can send data immediately */
      if (HAL_OK != HAL_UART_Transmit_DMA(&getUartHandle(), static_cast<uint8_t*>(&auTxBuffer_[uTxBufferRdIdx_]), uRemain))
      {
        bReturn = false;
      }
    }
  }

  taskEXIT_CRITICAL();

  return bReturn;
}

void CRs232OutputHandler::UARTTxInterruptHandler(uint16_t uSize)
{
  /* Function shall be called within ISR context */
  assert(SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk);

  /* Update the transmitted bytes index */
  uTxBufferRdIdx_ = (uTxBufferRdIdx_ + uSize) % skuBufferLength_;

  /* Check if there is remaining data to be transmitted */
  int32_t iRemain = static_cast<int32_t>(uTxBufferWrIdx_ - uTxBufferRdIdx_);

  if (iRemain > 0)
  {
    /*******************************
     * [0][1][2][3][4][5][6][7][8] *
     *     ^        ^
     *     Rd       Wr
     *******************************/
    HAL_UART_Transmit_DMA(&getUartHandle(), static_cast<uint8_t*>(&auTxBuffer_[uTxBufferRdIdx_]), iRemain);
  }
  else if (iRemain < 0)
  {
    /*******************************
     * [0][1][2][3][4][5][6][7][8] *
     *     ^        ^
     *     Wr       Rd
     *******************************/
    iRemain = skuBufferLength_ - uTxBufferRdIdx_;
    HAL_UART_Transmit_DMA(&getUartHandle(), static_cast<uint8_t*>(&auTxBuffer_[uTxBufferRdIdx_]), iRemain);
  }
  else
  {
    // No mode data. Exit
  }
}

bool CRs232OutputHandler::isBufferFree(uint16_t uSize)
{
  uint32_t uTotalBytes = 0;
  int32_t iDiff = static_cast<int32_t>(uTxBufferWrIdx_ - uTxBufferRdIdx_);

  if ((iDiff) >= 0)
  {
    uTotalBytes = (iDiff) + uSize;
  }
  else
  {
    uTotalBytes = (skuBufferLength_ - uTxBufferRdIdx_) + uTxBufferWrIdx_ + uSize;
  }

  return uTotalBytes < skuBufferLength_;
}
