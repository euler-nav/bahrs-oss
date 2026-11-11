/// @file CRs232OutputHandler.cpp
/// @brief Implementation of the RS232 Output Handler software component.
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "CRs232OutputHandler.h"
#include "CRte.h"
#include "AmsAssert.h"
#include "usart.h"
#include <string.h>

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

#ifdef SEND_DEBUG_OUTPUT

void CRs232OutputHandler::SendDebugData(const CSerialProtocol::SDebugData& korDebugData)
{
  size_t uMessageLen = 0U;
  uint8_t* upMessage = oMessageBuilder_.BuildDebugMessage(korDebugData, uMessageLen);

  if ((uMessageLen > 0U) && (nullptr != upMessage))
  {
    TransmitMessage(upMessage, uMessageLen);
  }

  if (nullptr != upMessage)
  {
    vPortFree(upMessage);
  }
}

void CRs232OutputHandler::SendInertialDataMessage()
{
  // Regular data messages are not sent in debug-output builds.
}

void CRs232OutputHandler::SendTimeOfInertialDataMessage()
{
  // Regular data messages are not sent in debug-output builds.
}

void CRs232OutputHandler::SendNavigationDataMessage()
{
  // Regular data messages are not sent in debug-output builds.
}

void CRs232OutputHandler::SendTimeOfNavigationDataMessage()
{
  // Regular data messages are not sent in debug-output builds.
}

void CRs232OutputHandler::SendAccuracyDataMessage()
{
  // Regular data messages are not sent in debug-output builds.
}

void CRs232OutputHandler::SendTimeOfLatestSyncPulseMessage()
{
  // Regular data messages are not sent in debug-output builds.
}

#else

void CRs232OutputHandler::SendInertialDataMessage()
{
  CRte& orRte = CRte::GetInstance();
  SOutputImuData oImuData;
  CSerialProtocol::SInertialDataMessage oMessage;

  bool bPortReadStatus = orRte.oPortImuOutput_.Read(oImuData);

  if (true == bPortReadStatus)
  {
    osStatus_t eStatus = osMutexAcquire(pMutexHandle_, skuMutexTimeoutTicks_);

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
    osStatus_t eStatus = osMutexAcquire(pMutexHandle_, skuMutexTimeoutTicks_);

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
  SSafeMagneticHeadingData oMagneticHeading;

  if (false == CRte::GetInstance().oPortSafeVerticalChannelData_.Read(oVerticalChannelData))
  {
    oVerticalChannelData = SSafeVerticalChannelData();
  }

  if (false == CRte::GetInstance().oPortSafeVehicleAttitude_.Read(oVehicleAttitude))
  {
    oVehicleAttitude = SSafeAttitudeData();
  }

  if (false == CRte::GetInstance().oPortSafeMagneticHeading_.Read(oMagneticHeading))
  {
    oMagneticHeading = SSafeMagneticHeadingData();
  }

  osStatus_t eStatus = osMutexAcquire(pMutexHandle_, skuMutexTimeoutTicks_);

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
    osStatus_t eStatus = osMutexAcquire(pMutexHandle_, skuMutexTimeoutTicks_);

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
    osStatus_t eStatus = osMutexAcquire(pMutexHandle_, skuMutexTimeoutTicks_);

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
    osStatus_t eStatus = osMutexAcquire(pMutexHandle_, skuMutexTimeoutTicks_);

    if (osOK == eStatus)
    {
      CSerialProtocol::STimeOfLatestSyncPulseMessage oMessage = oMessageBuilder_.BuildTimeOfLatestSyncPulseMessage(oTimeOfSyncPulse.uPulseTimeUs_);
      TransmitMessage(reinterpret_cast<uint8_t*>(&oMessage), sizeof(oMessage));
      osMutexRelease(pMutexHandle_);
    }
  }
}

#endif // SEND_DEBUG_OUTPUT

bool CRs232OutputHandler::TransmitMessage(uint8_t* upData, size_t uSize)
{
  bool bReturn = true;
  uint32_t uRemain;
  AMS_HARD_ASSERT((uSize > 0) && (uSize <= skuBufferLength_));

  taskENTER_CRITICAL();

  // Check if we have place to allocate the message
  bReturn = isBufferFree(uSize);
  if (bReturn)
  {
    // Enqueue the message to the buffer
    if ((uTxBufferWrIdx_ + uSize) <= skuBufferLength_)
    {
      uRemain = uSize;
      memcpy(static_cast<void*>(&(auTxBuffer_[uTxBufferWrIdx_])), static_cast<void*>(upData), uSize);
    }
    else
    {
      // Handle buffer roll over
      uRemain = skuBufferLength_ - uTxBufferWrIdx_;
      memcpy(static_cast<void*>(&(auTxBuffer_[uTxBufferWrIdx_])), static_cast<void*>(upData), uRemain);
      memcpy(static_cast<void*>(auTxBuffer_), static_cast<void*>(upData + uRemain), uSize - uRemain);
    }
    uTxBufferWrIdx_ = (uTxBufferWrIdx_ + uSize) % skuBufferLength_;

    // If UART DMA is ready means that the buffer was empty before the last insertion
    // We directly trigger the DMA transmission and start the flow from here
    if (HAL_UART_STATE_READY == getUartHandle().gState)
    {
      // UART is ready, we can send data immediately
      if (HAL_OK != HAL_UART_Transmit_DMA(&getUartHandle(), static_cast<uint8_t*>(&auTxBuffer_[uTxBufferRdIdx_]), static_cast<uint16_t>(uRemain)))
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
  // Function shall be called within ISR context
  assert(SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk);

  // Update the transmitted bytes index
  uTxBufferRdIdx_ = (uTxBufferRdIdx_ + uSize) % skuBufferLength_;

  // Check if there is remaining data to be transmitted
  if (uTxBufferWrIdx_ > uTxBufferRdIdx_)
  {
    //*******************************
    // [0][1][2][3][4][5][6][7][8] *
    //     ^        ^
    //     Rd       Wr
    //*******************************
    const uint32_t kuRemain = uTxBufferWrIdx_ - uTxBufferRdIdx_;
    HAL_UART_Transmit_DMA(&getUartHandle(), static_cast<uint8_t*>(&auTxBuffer_[uTxBufferRdIdx_]), static_cast<uint16_t>(kuRemain));
  }
  else if (uTxBufferWrIdx_ < uTxBufferRdIdx_)
  {
    //*******************************
    // [0][1][2][3][4][5][6][7][8] *
    //     ^        ^
    //     Wr       Rd
    //*******************************
    const uint32_t kuRemain = skuBufferLength_ - uTxBufferRdIdx_;
    HAL_UART_Transmit_DMA(&getUartHandle(), static_cast<uint8_t*>(&auTxBuffer_[uTxBufferRdIdx_]), static_cast<uint16_t>(kuRemain));
  }
  else
  {
    // No mode data. Exit
  }
}

bool CRs232OutputHandler::isBufferFree(size_t uSize)
{
  uint32_t uTotalBytes = 0;

  if (uTxBufferWrIdx_ >= uTxBufferRdIdx_)
  {
    uTotalBytes = (uTxBufferWrIdx_ - uTxBufferRdIdx_) + uSize;
  }
  else
  {
    uTotalBytes = (skuBufferLength_ - uTxBufferRdIdx_) + uTxBufferWrIdx_ + uSize;
  }

  return uTotalBytes < skuBufferLength_;
}
