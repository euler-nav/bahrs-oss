#include "CRte.h"
#include "crc32.h"
#include "UintToBool.h"
#include "NavigationUtilities.h"

COutputTransformer& COutputTransformer::getInstanceImpl(unsigned uInstanceIndex)
{
  static COutputTransformer soOutputTransformer;
  return soOutputTransformer;
}

void COutputTransformer::Init()
{
  // Placeholder for the open-source variant: no NVM driver is included, so vehicle-to-IMU installation parameters are not loaded here.
  // A user may hard-code alignment angles, implement their own loading, or use the Basic EULER-NAV BAHRS software license.
  oDcmDeviceToVehicle_ = Eigen::Matrix3f::Identity();
  oQuatVehicleToDevice_ = CQuaternion(1.0F, 0.0F, 0.0F, 0.0F);
  bIsInitialized_ = true;
}

bool COutputTransformer::IsInitialized()
{
  return bIsInitialized_;
}

void COutputTransformer::TransformImuSignals()
{
  SOutputImuData oImuOutputData;

  if (true == bIsInitialized_)
  {
    CImuDataAfterMonitor oImuData;
    bool bStatus = CRte::GetInstance().oPortImuDataAfterMonitor_.Read(oImuData);

    if (true == bStatus)
    {
      using EImuSignals = CImuDataAfterMonitor::EScalarSignals;
      using EHealthInfo = CSerialProtocol::ESignalHealthInfo;

      const EHealthInfo keSpecificForceHealth = buildHealthInfoForVectorSignal<EImuSignals::eSpecificForceX, EImuSignals::eSpecificForceY, EImuSignals::eSpecificForceZ>(oImuData);
      const EHealthInfo keAngularRateHealth = buildHealthInfoForVectorSignal<EImuSignals::eAngularRateX, EImuSignals::eAngularRateY, EImuSignals::eAngularRateZ>(oImuData);
      uint64_t uMeanSpecificForceTimestamp{ 0U };
      uint64_t uMeanAngularRateTimestamp{ 0U };
      const bool kbIsSpecificForceAvailable = (EHealthInfo::eUnavailable != keSpecificForceHealth);
      const bool kbIsAngularRateAvailable = (EHealthInfo::eUnavailable != keAngularRateHealth);

      if (kbIsSpecificForceAvailable)
      {
        Eigen::Vector3f oSpecificForce = oDcmDeviceToVehicle_ * Eigen::Vector3f(oImuData.GetSignal(EImuSignals::eSpecificForceX).fSignal_,
                                                                                oImuData.GetSignal(EImuSignals::eSpecificForceY).fSignal_,
                                                                                oImuData.GetSignal(EImuSignals::eSpecificForceZ).fSignal_);

        oImuOutputData.fSpecificForceX_ = oSpecificForce(0);
        oImuOutputData.fSpecificForceY_ = oSpecificForce(1);
        oImuOutputData.fSpecificForceZ_ = oSpecificForce(2);

        oImuOutputData.eSpecificForceHealth_ = keSpecificForceHealth;

        uMeanSpecificForceTimestamp = oImuData.GetSignal(EImuSignals::eSpecificForceX).uTimestampUs_ / 3 + \
                                      oImuData.GetSignal(EImuSignals::eSpecificForceY).uTimestampUs_ / 3 + \
                                      oImuData.GetSignal(EImuSignals::eSpecificForceZ).uTimestampUs_ / 3;
      }

      if (kbIsAngularRateAvailable)
      {
        Eigen::Vector3f oAngularRate = oDcmDeviceToVehicle_ * Eigen::Vector3f(oImuData.GetSignal(EImuSignals::eAngularRateX).fSignal_,
                                                                              oImuData.GetSignal(EImuSignals::eAngularRateY).fSignal_,
                                                                              oImuData.GetSignal(EImuSignals::eAngularRateZ).fSignal_);

        oImuOutputData.fAngularRateX_ = oAngularRate(0);
        oImuOutputData.fAngularRateY_ = oAngularRate(1);
        oImuOutputData.fAngularRateZ_ = oAngularRate(2);

        oImuOutputData.eAngularRateHealth_ = keAngularRateHealth;

        uMeanAngularRateTimestamp = oImuData.GetSignal(EImuSignals::eAngularRateX).uTimestampUs_ / 3 + \
                                    oImuData.GetSignal(EImuSignals::eAngularRateY).uTimestampUs_ / 3 + \
                                    oImuData.GetSignal(EImuSignals::eAngularRateZ).uTimestampUs_ / 3;
      }

      if (kbIsSpecificForceAvailable && kbIsAngularRateAvailable)
      {
        oImuOutputData.uTimestampUs_ = uMeanSpecificForceTimestamp / 2U + uMeanAngularRateTimestamp / 2U;
      }
      else if (kbIsSpecificForceAvailable)
      {
        oImuOutputData.uTimestampUs_ = uMeanSpecificForceTimestamp;
      }
      else if (kbIsAngularRateAvailable)
      {
        oImuOutputData.uTimestampUs_ = uMeanAngularRateTimestamp;
      }
      else
      {
        oImuOutputData.uTimestampUs_ = 0U;
      }
    }
  }

  CRte::GetInstance().oPortImuOutput_.Write(oImuOutputData);
}

void COutputTransformer::TransformOrientation()
{
  if (true == bIsInitialized_)
  {
    CBahrsFilterOutput oFilterOutput1, oFilterOutput2, oFilterOutput3;
    bool bStatus1 = CRte::GetInstance().oPortBahrsFilterOutput1_.Read(oFilterOutput1);
    bool bStatus2 = CRte::GetInstance().oPortBahrsFilterOutput2_.Read(oFilterOutput2);
    bool bStatus3 = CRte::GetInstance().oPortBahrsFilterOutput3_.Read(oFilterOutput3);

    if ((true == bStatus1) &&
        (CClosedLoopErrorStateKfApi::EFilterModes::RUNNING == oFilterOutput1.eFilterMode_))
    {
      CQuaternion oQuat = oFilterOutput1.oState_.oQuaternionBodyToNed_ * oQuatVehicleToDevice_;
      SAttitudeData oAttitudeData;
      float fYaw{ 0.0F };

      NNavigationUtilities::EulerFromQuaternion(oQuat, oAttitudeData.fRoll_, oAttitudeData.fPitch_, fYaw);
      oAttitudeData.fAttitudeStd1_ = oFilterOutput1.oStateStd_.oAttitude_(0);
      oAttitudeData.fAttitudeStd2_ = oFilterOutput1.oStateStd_.oAttitude_(1);
      oAttitudeData.uTimestampUs_ = oFilterOutput1.uTimestampUs_;
      oAttitudeData.uValid_ = BoolToUint(true);

      CRte::GetInstance().oPortVehicleAttitude1_.Write(oAttitudeData);
    }
    else
    {
      CRte::GetInstance().oPortVehicleAttitude1_.Write(SAttitudeData());
    }

    if ((true == bStatus2) &&
        (CClosedLoopErrorStateKfApi::EFilterModes::RUNNING == oFilterOutput2.eFilterMode_))
    {
      CQuaternion oQuat = oFilterOutput2.oState_.oQuaternionBodyToNed_ * oQuatVehicleToDevice_;
      SAttitudeData oAttitudeData;
      float fYaw{ 0.0F };

      NNavigationUtilities::EulerFromQuaternion(oQuat, oAttitudeData.fRoll_, oAttitudeData.fPitch_, fYaw);
      oAttitudeData.fAttitudeStd1_ = oFilterOutput2.oStateStd_.oAttitude_(0);
      oAttitudeData.fAttitudeStd2_ = oFilterOutput2.oStateStd_.oAttitude_(1);
      oAttitudeData.uTimestampUs_ = oFilterOutput2.uTimestampUs_;
      oAttitudeData.uValid_ = BoolToUint(true);

      CRte::GetInstance().oPortVehicleAttitude2_.Write(oAttitudeData);
    }
    else
    {
      CRte::GetInstance().oPortVehicleAttitude2_.Write(SAttitudeData());
    }

    if ((true == bStatus3) &&
        (CClosedLoopErrorStateKfApi::EFilterModes::RUNNING == oFilterOutput3.eFilterMode_))
    {
      CQuaternion oQuat = oFilterOutput3.oState_.oQuaternionBodyToNed_ * oQuatVehicleToDevice_;
      SAttitudeData oAttitudeData;
      float fYaw{ 0.0F };

      NNavigationUtilities::EulerFromQuaternion(oQuat, oAttitudeData.fRoll_, oAttitudeData.fPitch_, fYaw);
      oAttitudeData.fAttitudeStd1_ = oFilterOutput3.oStateStd_.oAttitude_(0);
      oAttitudeData.fAttitudeStd2_ = oFilterOutput3.oStateStd_.oAttitude_(1);
      oAttitudeData.uTimestampUs_ = oFilterOutput3.uTimestampUs_;
      oAttitudeData.uValid_ = BoolToUint(true);

      CRte::GetInstance().oPortVehicleAttitude3_.Write(oAttitudeData);
    }
    else
    {
      CRte::GetInstance().oPortVehicleAttitude3_.Write(SAttitudeData());
    }
  }
  else
  {
    CRte::GetInstance().oPortVehicleAttitude1_.Write(SAttitudeData());
    CRte::GetInstance().oPortVehicleAttitude2_.Write(SAttitudeData());
    CRte::GetInstance().oPortVehicleAttitude3_.Write(SAttitudeData());
  }
}

void COutputTransformer::TransformMagnetometerInput()
{
  SMagneticMeasurement oTransformedMagData;

  if (true == bIsInitialized_)
  {
    SMagneticMeasurement oCompensatedMagData;
    bool bReadStatus = CRte::GetInstance().oPortCompensatedMagnetometerData_.Read(oCompensatedMagData);

    if ((true == bReadStatus) && (true == UintToBool(oCompensatedMagData.uValid_)))
    {
      Eigen::Vector3f oVectorInVehicleFrame = oDcmDeviceToVehicle_ * Eigen::Vector3f(oCompensatedMagData.fVectorX_,
                                                                                     oCompensatedMagData.fVectorY_,
                                                                                     oCompensatedMagData.fVectorZ_);

      oTransformedMagData.uValid_ = BoolToUint(true);
      oTransformedMagData.uTimestampUs_ = oCompensatedMagData.uTimestampUs_;
      oTransformedMagData.fVectorX_ = oVectorInVehicleFrame(0);
      oTransformedMagData.fVectorY_ = oVectorInVehicleFrame(1);
      oTransformedMagData.fVectorZ_ = oVectorInVehicleFrame(2);
    }
  }

  CRte::GetInstance().oPortCompensatedMagnetometerDataInVehicleFrame_.Write(oTransformedMagData);
}


template<CImuDataAfterMonitor::EScalarSignals eSignal1,
         CImuDataAfterMonitor::EScalarSignals eSignal2,
         CImuDataAfterMonitor::EScalarSignals eSignal3>
CSerialProtocol::ESignalHealthInfo COutputTransformer::buildHealthInfoForVectorSignal(const CImuDataAfterMonitor& korImuData)
{
  static_assert(eSignal1 != eSignal2);
  static_assert(eSignal1 != eSignal3);

  using EHealthInfo = CSerialProtocol::ESignalHealthInfo;
  using EImuSignals = CImuDataAfterMonitor::EScalarSignals;
  using EDetectionResult = CImuDataAfterMonitor::EDetectionResult;
  using EIsolationResult = CImuDataAfterMonitor::EIsolationResult;

  EHealthInfo eHealthInfo{ EHealthInfo::eUnavailable };
  static constexpr std::array<EImuSignals, 3> skoSignalLabels{ eSignal1, eSignal2, eSignal3 };
  bool bIsVectorSignalAvailable{ true };

  // Check if all projections of a vector signal are available.
  for (auto eSignal : skoSignalLabels)
  {
    if (false == korImuData.GetSignal(eSignal).bValid_)
    {
      bIsVectorSignalAvailable = false;
      break;
    }
  }

  if (bIsVectorSignalAvailable)
  {
    bool bIsVectorSignalSafe{ true };

    for (auto eSignal : skoSignalLabels)
    {
      const auto& korSignal = korImuData.GetSignal(eSignal);

      if (!((korSignal.eDetectionResults_ == EDetectionResult::eGood) ||
            ((korSignal.eDetectionResults_ == EDetectionResult::eFailure) && (korSignal.eIsolationResults_ == EIsolationResult::eGood))))
      {
        bIsVectorSignalSafe = false;
        break;
      }
    }

    if (bIsVectorSignalSafe)
    {
      eHealthInfo = EHealthInfo::eSafe;
    }
    else
    {
      eHealthInfo = EHealthInfo::eIntegrityRisk;
    }
  }

  return eHealthInfo;
}

