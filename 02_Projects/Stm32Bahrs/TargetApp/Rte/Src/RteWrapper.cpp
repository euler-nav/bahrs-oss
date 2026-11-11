/// @file RteWrapper.cpp
/// @brief Implementation of C-wrappers of the CRte class methods.
/// @copyright Copyright 2022. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "CRte.h"
#include <assert.h>

extern "C" void RteInit()
{
  CRte::GetInstance().Init();
}

