/// @file SyncPulseHandlerCApi.h
/// @brief Declaration of a C wrapper for CSyncPulseHandler SWC APIs.
/// @copyright Copyright 2023. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef SYNC_PULSE_HANDLER_C_API_H
#define SYNC_PULSE_HANDLER_C_API_H

#ifdef __cplusplus
  #error This header must not be included in .cpp files.
#endif

/// @brief A callback that is executed by the dedicated interrupt handler.
void SyncPulseCallback();

#endif // SYNC_PULSE_HANDLER_C_API_H
