/// @file WriteToUserDefinedRtePort.cpp
/// @brief Implementation of RTE port filler function for ports with user-defined data type.
/// @copyright Copyright 2024. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "WriteToRtePort.h"

void WriteToUserDefinedRtePort(CRte::EPortIds ePortId, const std::vector<uint8_t>& korBytes)
{
  switch (ePortId)
  {
  case CRte::EPortIds::eImuOutput:
    {
      SOutputImuData oPortData;

      if (oPortData.FromByteVector(korBytes))
      {
        CRte::GetInstance().oPortImuOutput_.Write(oPortData);
      }
    }
    break;
  case CRte::EPortIds::eSafeVehicleAttitude:
    {
      SSafeAttitudeData oPortData;

      if (oPortData.FromByteVector(korBytes))
      {
        CRte::GetInstance().oPortSafeVehicleAttitude_.Write(oPortData);
      }
    }
    break;
  case CRte::EPortIds::eSafeVerticalChannelData:
    {
      SSafeVerticalChannelData oPortData;

      if (oPortData.FromByteVector(korBytes))
      {
        CRte::GetInstance().oPortSafeVerticalChannelData_.Write(oPortData);
      }
    }
    break;
  case CRte::EPortIds::eSafeMagneticHeading:
    {
      SSafeMagneticHeadingData oPortData;

      if (oPortData.FromByteVector(korBytes))
      {
        CRte::GetInstance().oPortSafeMagneticHeading_.Write(oPortData);
      }
    }
    break;
  default:
    break;
  }
}
