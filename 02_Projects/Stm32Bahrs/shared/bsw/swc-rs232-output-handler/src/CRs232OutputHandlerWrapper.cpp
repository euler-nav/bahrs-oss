/// @file CRs232OutputHandlerWrapper.cpp
/// @brief Implementation of the C wrapper functions for the RS232 output handler interrupts.
/// @copyright Copyright 2025. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "CRs232OutputHandlerWrapper.h"
#include "CRs232OutputHandler.h"

extern "C" void Rs232TxInterruptHandler(uint16_t uSize)
{
  CRs232OutputHandler::GetInstance().UARTTxInterruptHandler(uSize);
}
