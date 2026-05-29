/// @file ams_std_memory_functions.cpp
/// @brief Implementation of dynamic memory functions to replace the functions from the stdlib.
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "AmsAssert.h"
#include "FreeRTOS.h"
#include "task.h"
#include <new>
#include <stddef.h>
#include <string.h>
#include <limits.h>

static_assert(__cplusplus == 201703L, "Please review the list of required replacebale memory operators.");

#ifndef ENABLE_PRINTF

extern "C"
{

/// @brief Disabled _sbrk: prevents usage and initialization of newlib's heap.
/// @note This is a strong symbol that replaces the auto-generated sysmem.c version.
/// @return Always returns (void*)-1 to indicate failure.
void* _sbrk(ptrdiff_t)
{
  AMS_HARD_ASSERT(0);
  return (void*)-1;
}


/// @brief Disabled malloc() override. Usage of C-style dynamic memory is prohibited.
void* malloc(size_t)
{
  AMS_HARD_ASSERT(0);
  return NULL;
}


/// @brief Disabled free() override. Usage of C-style dynamic memory is prohibited.
void free(void*)
{
  AMS_HARD_ASSERT(0);
}


/// @brief Disabled calloc() override. Usage of C-style dynamic memory is prohibited.
void* calloc(size_t, size_t)
{
  AMS_HARD_ASSERT(0);
  return NULL;
}

/// @brief Disabled realloc() override. Usage of C-style dynamic memory is prohibited.
void* realloc(void*, size_t)
{
  AMS_HARD_ASSERT(0);
  return NULL;
}

} // extern "C"

#else

// Allow newlib's heap functions to enable printf() and similar for temporary local debug.

extern "C"
{

/// @brief _sbrk remains disabled. All dynamic memory is routed through C++ new/delete.
/// @return Always returns (void*)-1 to indicate failure.
void* _sbrk(ptrdiff_t)
{
  AMS_HARD_ASSERT(0);
  return (void*)-1;
}

/// @brief malloc() override for local debug. Uses C++ new.
/// @return Pointer to allocated memory or NULL.
void* __wrap__malloc_r(struct _reent*, size_t size)
{
  return ::operator new(size);
}

/// @brief free() override for local debug. Uses C++ delete.
void __wrap__free_r(struct _reent*, void* pPtr)
{
  ::operator delete(pPtr);
}

} // extern "C"

#endif // ENABLE_PRINTF

//
// Internal helpers
//

namespace
{

void* allocateOrAssert(std::size_t size)
{
  AMS_HARD_ASSERT(pdFALSE == xPortIsInsideInterrupt());

  // pvPortMalloc() calls xTaskResumeAll() which uses taskENTER/EXIT_CRITICAL.
  // Before the scheduler starts uxCriticalNesting == 0xaaaaaaaa (FreeRTOS sentinel),
  // so taskEXIT_CRITICAL never reaches 0 and BASEPRI is left permanently raised,
  // masking all peripheral interrupts for the rest of boot. Thus, we forbid allocations.
  AMS_HARD_ASSERT(taskSCHEDULER_NOT_STARTED != xTaskGetSchedulerState());

  void* pPtr{pvPortMalloc(size)};

  if (nullptr == pPtr)
  {
    // Note: Should throw std::bad_alloc here, but exceptions are disabled so we assert instead.
    AMS_HARD_ASSERT(false);
  }

  return pPtr;
}

void* allocateNoThrow(std::size_t size) noexcept
{
  AMS_HARD_ASSERT(pdFALSE == xPortIsInsideInterrupt());

  // pvPortMalloc() calls xTaskResumeAll() which uses taskENTER/EXIT_CRITICAL.
  // Before the scheduler starts uxCriticalNesting == 0xaaaaaaaa (FreeRTOS sentinel),
  // so taskEXIT_CRITICAL never reaches 0 and BASEPRI is left permanently raised,
  // masking all peripheral interrupts for the rest of boot. Thus, we forbid allocations.
  AMS_HARD_ASSERT(taskSCHEDULER_NOT_STARTED != xTaskGetSchedulerState());

  return pvPortMalloc(size);
}

void freeIfNotNull(void* pPtr) noexcept
{
  AMS_HARD_ASSERT(pdFALSE == xPortIsInsideInterrupt());
  AMS_HARD_ASSERT(taskSCHEDULER_NOT_STARTED != xTaskGetSchedulerState());

  if (nullptr != pPtr)
  {
    vPortFree(pPtr);
  }
}

} // namespace

//
// C++ operator new/delete overrides - throwing versions
//

void* operator new(std::size_t size)
{
  return allocateOrAssert(size);
}

void* operator new[](std::size_t size)
{
  return allocateOrAssert(size);
}

void* operator new(std::size_t size, std::align_val_t)
{
  // FreeRTOS's pvPortMalloc does not support alignment.
  AMS_HARD_ASSERT(false);
  __builtin_unreachable();
}

void* operator new[](std::size_t size, std::align_val_t)
{
  // FreeRTOS's pvPortMalloc does not support alignment.
  AMS_HARD_ASSERT(false);
  __builtin_unreachable();
}

void operator delete(void* pPtr) noexcept
{
  freeIfNotNull(pPtr);
}

void operator delete[](void* pPtr) noexcept
{
  freeIfNotNull(pPtr);
}

//
// C++ operator new/delete - nothrow versions
//

void* operator new(std::size_t size, const std::nothrow_t&) noexcept
{
  return allocateNoThrow(size);
}

void* operator new[](std::size_t size, const std::nothrow_t&) noexcept
{
  return allocateNoThrow(size);
}

void* operator new(std::size_t size, std::align_val_t, const std::nothrow_t&) noexcept
{
  // FreeRTOS's pvPortMalloc does not support alignment.
  AMS_HARD_ASSERT(false);
  return nullptr;
}

void* operator new[](std::size_t size, std::align_val_t, const std::nothrow_t&) noexcept
{
  // FreeRTOS's pvPortMalloc does not support alignment.
  AMS_HARD_ASSERT(false);
  return nullptr;
}

void operator delete(void* pPtr, const std::nothrow_t&) noexcept
{
  freeIfNotNull(pPtr);
}

void operator delete[](void* pPtr, const std::nothrow_t&) noexcept
{
  freeIfNotNull(pPtr);
}

//
// C++ operator delete - sized delete versions (C++14)
//

void operator delete(void* pPtr, std::size_t) noexcept
{
  freeIfNotNull(pPtr);
}

void operator delete[](void* pPtr, std::size_t) noexcept
{
  freeIfNotNull(pPtr);
}

//
// C++ operator delete - aligned delete versions (C++17)
//

void operator delete(void* pPtr, std::align_val_t) noexcept
{
  // FreeRTOS's pvPortMalloc does not support alignment.
  AMS_HARD_ASSERT(false);
}

void operator delete[](void* pPtr, std::align_val_t) noexcept
{
  // FreeRTOS's pvPortMalloc does not support alignment.
  AMS_HARD_ASSERT(false);
}

void operator delete(void* pPtr, std::size_t, std::align_val_t) noexcept
{
  // FreeRTOS's pvPortMalloc does not support alignment.
  AMS_HARD_ASSERT(false);
}

void operator delete[](void* pPtr, std::size_t, std::align_val_t) noexcept
{
  // FreeRTOS's pvPortMalloc does not support alignment.
  AMS_HARD_ASSERT(false);
}

//
// C++ operator delete - aligned nothrow delete versions (C++17)
//

void operator delete(void* pPtr, std::align_val_t, const std::nothrow_t&) noexcept
{
  // FreeRTOS's pvPortMalloc does not support alignment.
  AMS_HARD_ASSERT(false);
}

void operator delete[](void* pPtr, std::align_val_t, const std::nothrow_t&) noexcept
{
  // FreeRTOS's pvPortMalloc does not support alignment.
  AMS_HARD_ASSERT(false);
}
