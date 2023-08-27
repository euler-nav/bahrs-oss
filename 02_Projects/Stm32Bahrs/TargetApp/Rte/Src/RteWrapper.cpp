/**
 * @file RteWrapper.cpp
 * @brief Implementation of C-wrappers of the CRte class methods.
 * @author Fedor Baklanov
 * @date 04 February 2022
 */

#include "CRte.h"
#include <assert.h>

extern "C" void RteInit()
{
  CRte::GetInstance().Init();
}

