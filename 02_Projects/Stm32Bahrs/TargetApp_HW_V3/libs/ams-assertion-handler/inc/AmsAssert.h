/**
 * @file AmsAssert.h
 * @brief Declaration of project-specific assert macro.
 * @author Fedor Baklanov
 * @date 2 August 2024
 * @copyright Copyright 2024. AMS Advanced Air Mobility Sensors UG. All rights reserved.
 */

#ifndef AMS_ASSERT_H
#define AMS_ASSERT_H

#ifndef _MSC_VER
#define AMS_HARD_ASSERT(__e) ((__e) ? ((void)0) : AmsHardAssertFunc(__func__, __LINE__))
#else
#include <assert.h>
#define AMS_HARD_ASSERT(__e) ((__e) ? ((void)0) : assert(__e))
#endif // _MSC_VER

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef _MSC_VER
void AmsHardAssertFunc(const char* kcpFunction, int iLine) __attribute__ ((__noreturn__));
#endif

#ifdef __cplusplus
}
#endif

#endif // AMS_ASSERT_H
