/// @file CDiagnosticMode.h
/// @brief Declaration of the CDiagnosticMode class.
/// @copyright Copyright 2025. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef C_DIAGNOSTIC_MODE_H
#define C_DIAGNOSTIC_MODE_H

#include <stdint.h>
#include "usart.h"
#include "crc32.h"

//******************************************************************************
// CDiagnosticMode class is designed to control the device in debug mode.
//
// USAGE EXAMPLE:
//
// .ioc:
// ____________________________________________________________________________
//
// 1. Set up the UART RX with DMA
// 2. Enable UART Global Interrupt
//
// main.cpp:
// ____________________________________________________________________________
//
// 1. Set UART handle using class method:
// CDiagnosticMode::getInstance().setUARTHandle(&huart1)
//
// 2. Start UART in "Receive till IDLE DMA mode"
//
// HAL_UARTEx_ReceiveToIdle_DMA(
//                       &huart1,
//                       CDiagnosticMode::getInstance().getRXBufferPtr(),
//                       CDiagnosticMode::getInstance().getRXBufferSize() );
//
// 3. Turn off the interrupt for a half-filled buffer
//  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
//
// 4. Start the Diagnostic process before FREERTOS scheduler:
//
// CDiagnosticMode::GetInstance().DiagnosticModeProcess();
//
// 5. Implement the UART receive interrupt function and call class handler
//    method. After it, restart DMA receiving:
//
// void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
// {
//   if (huart->Instance == USART1) {
//     DebugMode_UARTRxInterruptHandler(Size);
//     HAL_UARTEx_ReceiveToIdle_DMA(
//                        &huart1,
//                        CDiagnosticMode::getInstance().getRXBufferPtr(),
//                        CDiagnosticMode::getInstance().getRXBufferSize() );
//     __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
//   }
// }
//******************************************************************************

#include "CNvmDriver.h"
#include "CSerialProtocol.h"

class CDiagnosticMode
{
  static constexpr uint64_t skuUARTSendTimeoutUs_ { 1000000 };
  static constexpr uint64_t skuDiagnosticModeTimeoutUs_ { 5000000 };

#pragma pack(push, 1)

  typedef union
  {
    CSerialProtocol::SPacketReadNVMPage oStructA;
    CSerialProtocol::SPacketNVMPage oStructB;
  } URXPacketMaxSize;

  typedef union
  {
    CSerialProtocol::SPacketReceiveConfirmation oStructA;
    CSerialProtocol::SPacketNVMPage oStructB;
  } UTXPacketMaxSize;

#pragma pack(pop)

  static constexpr uint16_t skuRXBufferSize_ { sizeof(URXPacketMaxSize) };
  static constexpr uint16_t skuTXBufferSize_ { sizeof(UTXPacketMaxSize) };

public:
  enum class EDiagnosticModeStatus : int8_t
  {
    // Errors (should be negative)
    eDiagnosticModeStatusErrUART = -3,
    eDiagnosticModeStatusErrInvalidSize = -2,
    eDiagnosticModeStatusErrDataCorrupted = -1,

    // Non-NVM mode (normal opertation mode) == 0
    eDiagnosticModeStatusNormal = 0,

    // NVM Mode States (should be positive)
    eDiagnosticModeStatusInDebug = 1
  };

  static CDiagnosticMode& GetInstance()
  {
    static CDiagnosticMode soInstance;
    return soInstance;
  };

  /// Method: setUARTHandle()
  /// Sets the UART_HandleTypeDef handler to send messages via UART.
  void SetUARTHandle(UART_HandleTypeDef* opUartHandle, DMA_HandleTypeDef* opDMAHandle);

  /// Method: UARTRxInterruptHandler()
  /// The handler of the received data in the UART RX interrupt.
  void UARTRxInterruptHandler(uint16_t uSize);

  /// Method: DiagnosticModeProcess()
  /// Stops the main program flow until the end of the diagnostic mode.
  void DiagnosticModeProcess();

  /// Method: IsDiagnosticModeEnabled()
  /// Returns true if device is working in Debug mode.
  bool isDiagnosticModeEnabled();

private:
  CDiagnosticMode(){};
  CDiagnosticMode(const CDiagnosticMode&);
  CDiagnosticMode& operator=(CDiagnosticMode&);

  uint8_t auUartRxBuffer_[skuRXBufferSize_];
  uint8_t auUartTxBuffer_[skuTXBufferSize_];
  uint16_t uUartTxSendSize_ { 0u };

  EDiagnosticModeStatus eCurrState_ { EDiagnosticModeStatus::eDiagnosticModeStatusNormal };

  UART_HandleTypeDef* opUartHandle_ { nullptr };
  DMA_HandleTypeDef* opDmaHandle_ { nullptr };

  /// Method: UARTDataParser()
  /// Packet data parser. Changes the mode of operation in case of successful 
  /// package parsing.
  inline void uartDataParser(uint16_t uSize);

  /// Method: DiagnosticSend()
  /// Sends a message via UART using a pre-setted up handle.
  bool diagnosticSend();

  // Method fillSendBuffer()
  // The function fills the buffer for transmitting data via UART

  void fillSendBuffer(const void* pBuffer, uint16_t uSize);

  /// Method: PacketAccepted()
  /// Sends a message confirming the successful packet processing by the device.
  bool handlePacket(CSerialProtocol::EMessageIds ePacketType);
};


#endif // C_DIAGNOSTIC_MODE_H
