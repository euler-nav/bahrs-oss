/// @file AmsAssert.h
/// @brief Declaration of the AMS assertion macro.
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef AMS_ASSERT_H
#define AMS_ASSERT_H

#include <assert.h>
#define AMS_HARD_ASSERT(__e) ((__e) ? ((void)0) : assert(__e))

#endif // AMS_ASSERT_H