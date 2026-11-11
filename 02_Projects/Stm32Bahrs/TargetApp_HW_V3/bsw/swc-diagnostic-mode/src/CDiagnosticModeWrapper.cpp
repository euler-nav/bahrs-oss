/// @file CDiagnosticModeWrapper.cpp
/// @brief Implementation of C-APIs of the diagnostic mode.
/// @copyright Copyright 2025. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "CDiagnosticModeWrapper.h"
#include "CDiagnosticMode.h"

extern "C" void DiagnosticModeUARTRxInterruptHandler(uint16_t uSize)
{
  // Function shall be called within ISR context
  assert(SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk);

  CDiagnosticMode::GetInstance().UARTRxInterruptHandler(uSize);
}


extern "C" void DiagnosticModeSetUARTHandle(UART_HandleTypeDef* opUartHandle, DMA_HandleTypeDef* opDMAHandle)
{
  CDiagnosticMode::GetInstance().SetUARTHandle(opUartHandle, opDMAHandle);
}

extern "C" void DiagnosticModeProcess()
{
  CDiagnosticMode::GetInstance().DiagnosticModeProcess();
}

extern "C" bool DiagnosticModeIsEnabled()
{
  return CDiagnosticMode::GetInstance().isDiagnosticModeEnabled();
}
