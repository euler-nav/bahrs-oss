/**
 * @file CRte.cpp
 * @brief Implementation of the runtime environment class.
 * @author Fedor Baklanov
 * @date 07 June 2022
 */

#include "CRte.h"

CRte& CRte::GetInstance()
{
  static CRte soRteInstance;
  return soRteInstance;
}

CRte::CRte()
{
}

void CRte::Init()
{
  oScha63TDriverPort_.Init();
  oPortBmp384Input_.Init();
  oPortMmc5983Input_.Init();
  oPortBmm150Input1_.Init();
  oPortBmm150Input2_.Init();
  oPortBahrsFilterOutput_.Init();
  oPortIcm20789BaroInput1_.Init();
  oPortIcm20789ImuInput1_.Init();
  oPortIcm20789BaroInput2_.Init();
  oPortIcm20789ImuInput2_.Init();
  oPortSyncPulseTime_.Init();
}

