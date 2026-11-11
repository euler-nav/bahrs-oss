/// @file CRs232OutputHandlerWrapper.h
/// @brief Declaration of the C wrapper functions for the RS232 output handler interrupts.
/// @copyright Copyright 2025. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef C_OUTPUT_HANDLER_WRAPPER_H
#define C_OUTPUT_HANDLER_WRAPPER_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// INTERRUPT WRAPPERS
////////////////////////////////////////////////////////////////////////////////

void Rs232TxInterruptHandler(uint16_t uSize);

#ifdef __cplusplus
}
#endif

#endif // C_OUTPUT_HANDLER_WRAPPER_H
