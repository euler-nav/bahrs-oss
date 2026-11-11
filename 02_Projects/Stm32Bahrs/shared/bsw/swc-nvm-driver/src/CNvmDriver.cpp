/// @file CNvmDriver.cpp
/// @brief Implementation of mandatory APIs of the NVM driver SWC.
/// @copyright Copyright 2025. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "CNvmDriver.h"

void CNvmDriver::Init()
{
  // Do nothing
}

bool CNvmDriver::IsInitialized()
{
  return true;
}

CNvmDriver& CNvmDriver::getInstanceImpl(unsigned uInstanceIndex)
{
  // The method shall not be used, because multiplicity of the NVM drive instances
  // is zero. Nevertheless we must comply with architecture patterns.
  static CNvmDriver soInstance;
  assert(false);
  return soInstance;
}

