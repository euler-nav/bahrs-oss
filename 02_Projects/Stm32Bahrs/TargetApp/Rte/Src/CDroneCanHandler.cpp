/**
 * @file CDroneCanHandler.cpp
 * @brief Implementation of the CDroneCanHandler class (DroneCAN handler basic software component).
 * @author Denis Ledkov
 * @date 16 February 2022
 */

#include "CDroneCanHandler.h"
#include "CRte.h"

CDroneCanHandler& CDroneCanHandler::getInstanceImpl(unsigned uInstanceIndex)
{
  static CDroneCanHandler soInstance;
  assert(uInstanceIndex == 0U);
  return soInstance;
}

void CDroneCanHandler::Init()
{
  // Do Nothing
}

bool CDroneCanHandler::IsInitialized()
{
  return true;
}

void CDroneCanHandler::SendNavigationDataMessage()
{
  CRte& orRte = CRte::GetInstance();
  NBahrsFilterApi::SOutputData oBahrsData;

  bool bReadStatus = orRte.oPortBahrsFilterOutput_.Read(oBahrsData);

  if (true == bReadStatus)
  {
    /* TO DO. Compose the message according to the DroneCAN and then send it. */
  }
}
