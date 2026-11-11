#ifndef INC_TEST_H_
#define INC_TEST_H_

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

#endif /* INC_TEST_H_ */
