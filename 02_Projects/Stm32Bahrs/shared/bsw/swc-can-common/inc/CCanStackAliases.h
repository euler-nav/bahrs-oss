/// @file CCanStackAliases.h
/// @brief Compile-time aliases that map canonical CAN stack component names to the active stack implementation.
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef C_CAN_STACK_ALIASES_H
#define C_CAN_STACK_ALIASES_H

#ifdef USE_DRONECAN

#include "CDroneCanHandler.h"
#include "CDroneCanOutputHandler.h"

using CCanHandler = CDroneCanHandler;
using CCanOutputHandler = CDroneCanOutputHandler;

#else

#include "CCanHandler.h"
#include "CCanOutputHandler.h"

#endif // USE_DRONECAN

#endif // C_CAN_STACK_ALIASES_H
