/// @file canardWrapper.h
/// @brief Declaration of wrapper around canard.h that suppresses the -Wpedantic warning caused
///        by the C99 flexible array member (uint8_t data[]) in CanardBufferBlock.
///
/// ISO C++ forbids flexible array members, but canard.h is a third-party header
/// that must not be modified.  This wrapper uses GCC diagnostic pragmas to
/// silence the warning for that header only and immediately re-enables -Wpedantic
/// afterwards.
///
/// All application code that needs the libcanard API should include this file
/// instead of including canard.h directly.
///
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef CANARD_WRAPPER_H
#define CANARD_WRAPPER_H

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "canard.h"
#pragma GCC diagnostic pop

#endif // CANARD_WRAPPER_H
