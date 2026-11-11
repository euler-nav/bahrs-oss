/// @file CDiagnosticMode.cpp
/// @brief Implementation of the diagnostic mode.
/// @copyright Copyright 2025. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "CDiagnosticMode.h"
#include "GetMicroseconds.h"

bool CDiagnosticMode::isDiagnosticModeEnabled()
{
  return (eCurrState_ > EDiagnosticModeStatus::eDiagnosticModeStatusNormal ) ? true : false;
}

void CDiagnosticMode::SetUARTHandle(UART_HandleTypeDef* opUartHandle, DMA_HandleTypeDef* opDMAHandle)
{
  opUartHandle_ = opUartHandle;
  opDmaHandle_ = opDMAHandle;
}

void CDiagnosticMode::UARTRxInterruptHandler(uint16_t uSize)
{
  uartDataParser(uSize);

  HAL_UARTEx_ReceiveToIdle_DMA(opUartHandle_, auUartRxBuffer_, skuRXBufferSize_);
  __HAL_DMA_DISABLE_IT(opDmaHandle_, DMA_IT_HT);
}

bool CDiagnosticMode::handlePacket(CSerialProtocol::EMessageIds ePacketType)
{
  switch(ePacketType)
  {
    case CSerialProtocol::EMessageIds::eTypeReadNVMPage:
      {
        CSerialProtocol::SPacketReadNVMPage* opRequest = reinterpret_cast<CSerialProtocol::SPacketReadNVMPage*>(auUartRxBuffer_);

        CSerialProtocol::SPacketNVMPage oPacket;
        oPacket.uPageNumber = opRequest->uPageNumber;

        if (CNvmDriver::ReadPage(opRequest->uPageNumber, oPacket.auPageData))
        {
          oPacket.uCrc32 = crc32(&oPacket, sizeof(oPacket) - sizeof(oPacket.uCrc32));
          fillSendBuffer(&oPacket, sizeof(oPacket));
        }
        else  // Read failed, send confirmation packet instead of page packet with error.
        {
          CSerialProtocol::SPacketReceiveConfirmation oPacket;
          oPacket.eMessageType = ePacketType;
          oPacket.uStatus = 0xFFU;

          oPacket.uCrc32 = crc32(&oPacket, sizeof(oPacket) - sizeof(oPacket.uCrc32));
          fillSendBuffer(&oPacket, sizeof(oPacket));
        }
        break;
      }

    case CSerialProtocol::EMessageIds::eTypeNVMPageData:
      {
        CSerialProtocol::SPacketNVMPage* opRequest = reinterpret_cast<CSerialProtocol::SPacketNVMPage*>(auUartRxBuffer_);

        CSerialProtocol::SPacketReceiveConfirmation oPacket;
        oPacket.eMessageType = ePacketType;

        if (!CNvmDriver::WritePage(opRequest->uPageNumber, opRequest->auPageData))
        {
          oPacket.uStatus = 0xFFU;
        }

        oPacket.uCrc32 = crc32(&oPacket, sizeof(oPacket) - sizeof(oPacket.uCrc32));
        fillSendBuffer(&oPacket, sizeof(oPacket));

        break;
      }

    default:
      {
        CSerialProtocol::SPacketReceiveConfirmation oPacket;
        oPacket.eMessageType = ePacketType;
        oPacket.uCrc32 = crc32(&oPacket, sizeof(oPacket) - sizeof(oPacket.uCrc32));
        fillSendBuffer(&oPacket, sizeof(oPacket));
      }
  }

  return diagnosticSend();
}

void CDiagnosticMode::fillSendBuffer(const void* pBuffer, uint16_t uSize)
{
  const uint8_t* upDataBuffer = reinterpret_cast<const uint8_t*>(pBuffer);
  uint8_t* upTxBuffer = auUartTxBuffer_;
  uint8_t* upEndBuffer = auUartTxBuffer_ + uSize;

  while(upTxBuffer != upEndBuffer)
  {
    *(upTxBuffer++) = *(upDataBuffer++);
  }

  uUartTxSendSize_ = uSize;
}

bool CDiagnosticMode::diagnosticSend()
{
  bool bPacketSent = false;

  if (opUartHandle_ != nullptr)
  {
    uint64_t uTimeout = GetMicroseconds() + skuUARTSendTimeoutUs_;
    uint64_t uCurrTime;

    // At this stage of integrating the CDiagnosticMode module into the existing code,
    // we send a confirmation using UART/DMA functions directly from HAL.
    // In the future, it is planned to use Crt / CRs232OutputHandler classes to do
    // this, and possibly send this package from the FREERTOS level, and not from
    // the interrupt (using native Queue).

    // So, let's stop the any DMA UART transmission before sending receive
    // confirmation packet.

    HAL_UART_DMAStop(opUartHandle_);

    do
    {
      if (HAL_UART_STATE_READY == opUartHandle_->gState)
      {
        HAL_UART_Transmit_DMA(opUartHandle_, auUartTxBuffer_, uUartTxSendSize_);
        bPacketSent = true;
        break;
      }

      uCurrTime = GetMicroseconds();
    } while (uCurrTime < uTimeout);
  }

  return bPacketSent;
}

//
// In the current implementation, I use the ASK-Handshake exchange principle.
// In other words, we are working in "packet" mode and the reception of each
// packet must be confirmed by the device.
//
// A new data packet will not be sent until the current one is confirmed.
// Therefore, as soon as we have found a valid package, we should send
// the packet receive confirmation and exit our parser.
//

void CDiagnosticMode::uartDataParser(uint16_t uSize)
{
  if (uSize > skuRXBufferSize_)
  {
    eCurrState_ = EDiagnosticModeStatus::eDiagnosticModeStatusErrInvalidSize;
  }
  else
  {
    CSerialProtocol::SMessageHeader* opHeader = reinterpret_cast<CSerialProtocol::SMessageHeader*>(auUartRxBuffer_);
    CSerialProtocol::CrcType_t* upCrc32 = reinterpret_cast<CSerialProtocol::CrcType_t*>(auUartRxBuffer_ + uSize - sizeof(CSerialProtocol::CrcType_t));
    CSerialProtocol::CrcType_t uDataCrc32 = crc32(auUartRxBuffer_, uSize - sizeof(CSerialProtocol::CrcType_t));

    if (uDataCrc32 != *upCrc32)
    {
      eCurrState_ = EDiagnosticModeStatus::eDiagnosticModeStatusErrDataCorrupted;
    }
    else
    {

      if ( (opHeader->uMarker1_ == CSerialProtocol::uMarker1_) &&
           (opHeader->uMarker2_ == CSerialProtocol::uMarker2_) &&
           (opHeader->uVersion_ == CSerialProtocol::uVersion_))
      {
        bool bValidPacketFound = true;

        switch(static_cast<CSerialProtocol::EMessageIds>(opHeader->uMsgType_))
        {
        case CSerialProtocol::EMessageIds::eTypeOpenDiagnostic:
          eCurrState_ = EDiagnosticModeStatus::eDiagnosticModeStatusInDebug;
          break;

        case CSerialProtocol::EMessageIds::eTypeCloseDiagnostic:
          eCurrState_ = EDiagnosticModeStatus::eDiagnosticModeStatusNormal;
          break;

        case CSerialProtocol::EMessageIds::eTypeReadNVMPage:
          break;

        case CSerialProtocol::EMessageIds::eTypeNVMPageData:
          break;

        default:
          bValidPacketFound = false;
          break;
        }

        if (bValidPacketFound)
        {
          if(!handlePacket(static_cast<CSerialProtocol::EMessageIds>(opHeader->uMsgType_)))
          {
            eCurrState_ = EDiagnosticModeStatus::eDiagnosticModeStatusErrUART;
          }
        }
      }
    }
  }
}


void CDiagnosticMode::DiagnosticModeProcess()
{
  HAL_UARTEx_ReceiveToIdle_DMA(opUartHandle_, auUartRxBuffer_, skuRXBufferSize_);
  __HAL_DMA_DISABLE_IT(opDmaHandle_, DMA_IT_HT);

  uint64_t uEndTime = GetMicroseconds() + skuDiagnosticModeTimeoutUs_;
  for(;;)
  {
    if ( (GetMicroseconds() >= uEndTime) &&             // Timeout end
         !isDiagnosticModeEnabled() &&                  // Not in Diagnostic mode
         HAL_UART_STATE_READY == opUartHandle_->gState) // Not sending packet
    {
      break;
    }
  }

  HAL_UART_AbortReceive_IT(opUartHandle_);
}
