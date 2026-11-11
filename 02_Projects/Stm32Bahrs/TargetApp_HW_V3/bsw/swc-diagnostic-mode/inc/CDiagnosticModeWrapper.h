/// @file CDiagnosticModeWrapper.h
/// @brief Declaration of C-APIs of the diagnostic mode.
/// @copyright Copyright 2025. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef C_DIAGNOSTIC_MODE_WRAPPER_H
#define C_DIAGNOSTIC_MODE_WRAPPER_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "usart.h"

#ifdef __cplusplus
extern "C" {
#endif

// INTERRUPT WRAPPERS
////////////////////////////////////////////////////////////////////////////////

void DiagnosticModeUARTRxInterruptHandler(uint16_t uSize);
void DiagnosticModeSetUARTHandle(UART_HandleTypeDef* opUartHandle, DMA_HandleTypeDef* opDMAHandle);
void DiagnosticModeProcess();
bool DiagnosticModeIsEnabled();

#ifdef __cplusplus
}
#endif

#endif // C_DIAGNOSTIC_MODE_WRAPPER_H
