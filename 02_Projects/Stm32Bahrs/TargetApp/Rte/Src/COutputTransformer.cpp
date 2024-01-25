#include "CRte.h"
#include "UintToBool.h"
#include "NavigationUtilities.h"

COutputTransformer& COutputTransformer::getInstanceImpl(unsigned uInstanceIndex)
{
  static COutputTransformer soOutputTransformer;
  return soOutputTransformer;
}

void COutputTransformer::Init()
{
  if (false == bIsInitialized_)
  {
    // This function is a placeholder. In proprietary SW it reads parameters from NVM
    // and returns false on failure.
    bIsInitialized_ = true;
  }
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
      using EImuSignals = CImuDataAfterMonitor::EImuSignals;

      const bool kbIsSpecificForceValid = isSpecificForceValidAndSafe(oImuData);
      const bool kbIsAngularRateValid = isAngularRateValidAndSafe(oImuData);
      uint64_t uMeanSpecificForceTimestamp{ 0U };
      uint64_t uMeanAngularRateTimestamp{ 0U };

      if (kbIsSpecificForceValid)
      {
        oImuOutputData.fSpecificForceX_ = oImuData.GetSignal(EImuSignals::eSpecificForceX).fSignal_;
        oImuOutputData.fSpecificForceY_ = oImuData.GetSignal(EImuSignals::eSpecificForceY).fSignal_;
        oImuOutputData.fSpecificForceZ_ = oImuData.GetSignal(EImuSignals::eSpecificForceZ).fSignal_;

        oImuOutputData.uSpecificForceValid_ = BoolToUint(true);

        uMeanSpecificForceTimestamp = oImuData.GetSignal(EImuSignals::eSpecificForceX).uTimestampUs_ / 3 + \
                                      oImuData.GetSignal(EImuSignals::eSpecificForceY).uTimestampUs_ / 3 + \
                                      oImuData.GetSignal(EImuSignals::eSpecificForceZ).uTimestampUs_ / 3;
      }

      if (kbIsAngularRateValid)
      {
        oImuOutputData.fAngularRateX_ = oImuData.GetSignal(EImuSignals::eAngularRateX).fSignal_;
        oImuOutputData.fAngularRateY_ = oImuData.GetSignal(EImuSignals::eAngularRateY).fSignal_;
        oImuOutputData.fAngularRateZ_ = oImuData.GetSignal(EImuSignals::eAngularRateZ).fSignal_;

        oImuOutputData.uAngularRateValid_ = BoolToUint(true);

        uMeanAngularRateTimestamp = oImuData.GetSignal(EImuSignals::eAngularRateX).uTimestampUs_ / 3 + \
                                    oImuData.GetSignal(EImuSignals::eAngularRateY).uTimestampUs_ / 3 + \
                                    oImuData.GetSignal(EImuSignals::eAngularRateZ).uTimestampUs_ / 3;
      }

      if (kbIsSpecificForceValid && kbIsAngularRateValid)
      {
        oImuOutputData.uTimestampUs_ = uMeanSpecificForceTimestamp / 2U + uMeanAngularRateTimestamp / 2U;
      }
      else if (kbIsSpecificForceValid)
      {
        oImuOutputData.uTimestampUs_ = uMeanSpecificForceTimestamp;
      }
      else if (kbIsAngularRateValid)
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

bool COutputTransformer::isSpecificForceValidAndSafe(const CImuDataAfterMonitor& korImuData)
{
  bool bResult{ true };

  using EImuSignals = CImuDataAfterMonitor::EImuSignals;
  using EDetectionResult = CImuDataAfterMonitor::EDetectionResult;
  using EIsolationResult = CImuDataAfterMonitor::EIsolationResult;

  std::array<CImuDataAfterMonitor::EImuSignals, 3> oSignalLabels{ EImuSignals::eSpecificForceX, EImuSignals::eSpecificForceY, EImuSignals::eSpecificForceZ};

  for (EImuSignals eLabel : oSignalLabels)
  {
    if (false == korImuData.GetSignal(eLabel).bValid_)
    {
      bResult = false;
      break;
    }
    else
    {
      if (!((korImuData.GetSignal(eLabel).eDetectionResults_ == EDetectionResult::eGood) ||
            ((korImuData.GetSignal(eLabel).eDetectionResults_ == EDetectionResult::eFailure) && (korImuData.GetSignal(eLabel).eIsolationResults_ == EIsolationResult::eGood))))
      {
        bResult = false;
        break;
      }
    }
  }

  return bResult;
}

bool COutputTransformer::isAngularRateValidAndSafe(const CImuDataAfterMonitor& korImuData)
{
  bool bResult{ true };

  using EImuSignals = CImuDataAfterMonitor::EImuSignals;
  using EDetectionResult = CImuDataAfterMonitor::EDetectionResult;
  using EIsolationResult = CImuDataAfterMonitor::EIsolationResult;

  std::array<CImuDataAfterMonitor::EImuSignals, 3> oSignalLabels{ EImuSignals::eAngularRateX, EImuSignals::eAngularRateY, EImuSignals::eAngularRateZ};

  for (EImuSignals eLabel : oSignalLabels)
  {
    if (false == korImuData.GetSignal(eLabel).bValid_)
    {
      bResult = false;
      break;
    }
    else
    {
      if (!((korImuData.GetSignal(eLabel).eDetectionResults_ == EDetectionResult::eGood) ||
            ((korImuData.GetSignal(eLabel).eDetectionResults_ == EDetectionResult::eFailure) && (korImuData.GetSignal(eLabel).eIsolationResults_ == EIsolationResult::eGood))))
      {
        bResult = false;
        break;
      }
    }
  }

  return bResult;
}

