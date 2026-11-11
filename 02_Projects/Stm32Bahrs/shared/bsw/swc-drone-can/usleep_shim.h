/// @file usleep_shim.h
/// @brief Maps POSIX usleep() to HAL_Delay() for bare-metal STM32 builds.
///
/// canard_stm32.c includes <unistd.h> and calls usleep(1000) (= 1 ms) to wait
/// for bxCAN initialisation acknowledgement.  <unistd.h> is not available in
/// the arm-none-eabi nano.specs toolchain, so the declaration is missing and
/// the compiler raises -Wimplicit-function-declaration.
///
/// This file is force-included into lib-canard translation units only (via
/// -include in the CMakeLists.txt for swc-drone-can).  It must not be included
/// by application code.
///
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef USLEEP_SHIM_H
#define USLEEP_SHIM_H

#include "stm32f4xx_hal.h"

// usleep() argument is in microseconds; HAL_Delay() takes milliseconds.
// usleep(1000) in canard_stm32.c equals exactly 1 ms, so the conversion
// (x) / 1000U is exact for all call sites in that file.
#define usleep(x) HAL_Delay((x) / 1000U)

#endif // USLEEP_SHIM_H
