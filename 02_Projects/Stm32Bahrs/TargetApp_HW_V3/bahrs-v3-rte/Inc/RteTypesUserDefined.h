/**
 * @file RteTypesUserDefined.h
 * @brief Declaration of software component user-defined port types.
 * @date 15 April 2024
 */

#ifndef RTE_TYPES_USER_DEFINED_H
#define RTE_TYPES_USER_DEFINED_H

#ifndef __cplusplus
  #error This header must not be included in .c files.
#endif /* __cplusplus */

#include "ImuMonitorApi.h"
#include "BahrsFilterApi.h"
#include "DataSerializer.h"
#include "CSerializable.h"
#include "CSerialProtocol.h"
#include "UintToBool.h"

class CImuDataAfterMonitor final : public NImuMonitorApi::COutputData, public NLibCommon::CSerializable
{
public:
#if defined(SEND_DEBUG_OUTPUT) || defined(_MSC_VER)

  std::vector<uint8_t> ToByteVector() final
  {
    using EScalarSignals = NImuMonitorApi::COutputData::EScalarSignals;

    const auto& oSpecificForceX = GetSignal(EScalarSignals::eSpecificForceX);
    const auto& oSpecificForceY = GetSignal(EScalarSignals::eSpecificForceY);
    const auto& oSpecificForceZ = GetSignal(EScalarSignals::eSpecificForceZ);
    const auto& oAngularRateX = GetSignal(EScalarSignals::eAngularRateX);
    const auto& oAngularRateY = GetSignal(EScalarSignals::eAngularRateY);
    const auto& oAngularRateZ = GetSignal(EScalarSignals::eAngularRateZ);

    const uint8_t uSpecificForceXValid = BoolToUint(oSpecificForceX.bValid_);
    const uint8_t uSpecificForceYValid = BoolToUint(oSpecificForceY.bValid_);
    const uint8_t uSpecificForceZValid = BoolToUint(oSpecificForceZ.bValid_);
    const uint8_t uAngularRateXValid = BoolToUint(oAngularRateX.bValid_);
    const uint8_t uAngularRateYValid = BoolToUint(oAngularRateY.bValid_);
    const uint8_t uAngularRateZValid = BoolToUint(oAngularRateZ.bValid_);

    std::vector<uint8_t> oBytes;
    bool bStatus = NLibCommon::Serialize(oBytes,
      oSpecificForceX.fSignal_,
      oSpecificForceX.uTimestampUs_,
      oSpecificForceX.eSensorId_,
      oSpecificForceX.eDetectionResults_,
      oSpecificForceX.eIsolationResults_,
      oSpecificForceX.eIsolatedSensor_,
      uSpecificForceXValid,
      oSpecificForceY.fSignal_,
      oSpecificForceY.uTimestampUs_,
      oSpecificForceY.eSensorId_,
      oSpecificForceY.eDetectionResults_,
      oSpecificForceY.eIsolationResults_,
      oSpecificForceY.eIsolatedSensor_,
      uSpecificForceYValid,
      oSpecificForceZ.fSignal_,
      oSpecificForceZ.uTimestampUs_,
      oSpecificForceZ.eSensorId_,
      oSpecificForceZ.eDetectionResults_,
      oSpecificForceZ.eIsolationResults_,
      oSpecificForceZ.eIsolatedSensor_,
      uSpecificForceZValid,
      oAngularRateX.fSignal_,
      oAngularRateX.uTimestampUs_,
      oAngularRateX.eSensorId_,
      oAngularRateX.eDetectionResults_,
      oAngularRateX.eIsolationResults_,
      oAngularRateX.eIsolatedSensor_,
      uAngularRateXValid,
      oAngularRateY.fSignal_,
      oAngularRateY.uTimestampUs_,
      oAngularRateY.eSensorId_,
      oAngularRateY.eDetectionResults_,
      oAngularRateY.eIsolationResults_,
      oAngularRateY.eIsolatedSensor_,
      uAngularRateYValid,
      oAngularRateZ.fSignal_,
      oAngularRateZ.uTimestampUs_,
      oAngularRateZ.eSensorId_,
      oAngularRateZ.eDetectionResults_,
      oAngularRateZ.eIsolationResults_,
      oAngularRateZ.eIsolatedSensor_,
      uAngularRateZValid);

    if (false == bStatus)
    {
      oBytes.clear();
    }

    return oBytes;
  }

  bool FromByteVector(const std::vector<uint8_t>& korBytes) final
  {
    using EScalarSignals = NImuMonitorApi::COutputData::EScalarSignals;

    auto& oSpecificForceX = GetSignal(EScalarSignals::eSpecificForceX);
    auto& oSpecificForceY = GetSignal(EScalarSignals::eSpecificForceY);
    auto& oSpecificForceZ = GetSignal(EScalarSignals::eSpecificForceZ);
    auto& oAngularRateX = GetSignal(EScalarSignals::eAngularRateX);
    auto& oAngularRateY = GetSignal(EScalarSignals::eAngularRateY);
    auto& oAngularRateZ = GetSignal(EScalarSignals::eAngularRateZ);

    uint8_t uSpecificForceXValid = 0U;
    uint8_t uSpecificForceYValid = 0U;
    uint8_t uSpecificForceZValid = 0U;
    uint8_t uAngularRateXValid = 0U;
    uint8_t uAngularRateYValid = 0U;
    uint8_t uAngularRateZValid = 0U;

    bool bStatus = NLibCommon::Deserialize(korBytes,
      oSpecificForceX.fSignal_,
      oSpecificForceX.uTimestampUs_,
      oSpecificForceX.eSensorId_,
      oSpecificForceX.eDetectionResults_,
      oSpecificForceX.eIsolationResults_,
      oSpecificForceX.eIsolatedSensor_,
      uSpecificForceXValid,
      oSpecificForceY.fSignal_,
      oSpecificForceY.uTimestampUs_,
      oSpecificForceY.eSensorId_,
      oSpecificForceY.eDetectionResults_,
      oSpecificForceY.eIsolationResults_,
      oSpecificForceY.eIsolatedSensor_,
      uSpecificForceYValid,
      oSpecificForceZ.fSignal_,
      oSpecificForceZ.uTimestampUs_,
      oSpecificForceZ.eSensorId_,
      oSpecificForceZ.eDetectionResults_,
      oSpecificForceZ.eIsolationResults_,
      oSpecificForceZ.eIsolatedSensor_,
      uSpecificForceZValid,
      oAngularRateX.fSignal_,
      oAngularRateX.uTimestampUs_,
      oAngularRateX.eSensorId_,
      oAngularRateX.eDetectionResults_,
      oAngularRateX.eIsolationResults_,
      oAngularRateX.eIsolatedSensor_,
      uAngularRateXValid,
      oAngularRateY.fSignal_,
      oAngularRateY.uTimestampUs_,
      oAngularRateY.eSensorId_,
      oAngularRateY.eDetectionResults_,
      oAngularRateY.eIsolationResults_,
      oAngularRateY.eIsolatedSensor_,
      uAngularRateYValid,
      oAngularRateZ.fSignal_,
      oAngularRateZ.uTimestampUs_,
      oAngularRateZ.eSensorId_,
      oAngularRateZ.eDetectionResults_,
      oAngularRateZ.eIsolationResults_,
      oAngularRateZ.eIsolatedSensor_,
      uAngularRateZValid);

    if (false == bStatus)
    {
      *this = CImuDataAfterMonitor();
    }
    else
    {
      oSpecificForceX.bValid_ = (uSpecificForceXValid != 0U);
      oSpecificForceY.bValid_ = (uSpecificForceYValid != 0U);
      oSpecificForceZ.bValid_ = (uSpecificForceZValid != 0U);
      oAngularRateX.bValid_ = (uAngularRateXValid != 0U);
      oAngularRateY.bValid_ = (uAngularRateYValid != 0U);
      oAngularRateZ.bValid_ = (uAngularRateZValid != 0U);
    }

    return bStatus;
  }
#endif /* SEND_DEBUG_OUTPUT || _MSC_VER */
};

class CBahrsFilterOutput final : public NBahrsFilterApi::SOutputData, public NLibCommon::CSerializable
{
public:
#if defined(SEND_DEBUG_OUTPUT) || defined(_MSC_VER)

  std::vector<uint8_t> ToByteVector() final
  {
    const uint8_t uVerticalChannelDiverged = BoolToUint(bVerticalChannelDiverged_);

    std::vector<uint8_t> oBytes;
    bool bStatus = NLibCommon::Serialize(oBytes,
      oState_.fHeight_,
      oState_.fVelocityDown_,
      oState_.oQuaternionBodyToNed_.W(),
      oState_.oQuaternionBodyToNed_.X(),
      oState_.oQuaternionBodyToNed_.Y(),
      oState_.oQuaternionBodyToNed_.Z(),
      oState_.oAccelerometerBias_.x(),
      oState_.oAccelerometerBias_.y(),
      oState_.oAccelerometerBias_.z(),
      oState_.oGyroscopeBias_.x(),
      oState_.oGyroscopeBias_.y(),
      oState_.oGyroscopeBias_.z(),
      oState_.oAccelerometerScaleFactor_.x(),
      oState_.oAccelerometerScaleFactor_.y(),
      oState_.oAccelerometerScaleFactor_.z(),
      oState_.oGyroscopeScaleFactor_.x(),
      oState_.oGyroscopeScaleFactor_.y(),
      oState_.oGyroscopeScaleFactor_.z(),
      oState_.oAccelerationNed_.x(),
      oState_.oAccelerationNed_.y(),
      oState_.oAccelerationNed_.z(),
      oStateStd_.fHeight_,
      oStateStd_.fVelocityDown_,
      oStateStd_.oAttitude_.x(),
      oStateStd_.oAttitude_.y(),
      oStateStd_.oAttitude_.z(),
      oStateStd_.oAccelerometerBias_.x(),
      oStateStd_.oAccelerometerBias_.y(),
      oStateStd_.oAccelerometerBias_.z(),
      oStateStd_.oGyroscopeBias_.x(),
      oStateStd_.oGyroscopeBias_.y(),
      oStateStd_.oGyroscopeBias_.z(),
      oStateStd_.oAccelerometerScaleFactor_.x(),
      oStateStd_.oAccelerometerScaleFactor_.y(),
      oStateStd_.oAccelerometerScaleFactor_.z(),
      oStateStd_.oGyroscopeScaleFactor_.x(),
      oStateStd_.oGyroscopeScaleFactor_.y(),
      oStateStd_.oGyroscopeScaleFactor_.z(),
      oStateStd_.oAccelerationNed_.x(),
      oStateStd_.oAccelerationNed_.y(),
      oStateStd_.oAccelerationNed_.z(),
      uTimestampUs_,
      eFilterMode_,
      uVerticalChannelDiverged);

    if (false == bStatus)
    {
      oBytes.clear();
    }

    return oBytes;
  }

  bool FromByteVector(const std::vector<uint8_t>& korBytes) final
  {
    uint8_t uVerticalChannelDiverged = 0U;

    bool bStatus = NLibCommon::Deserialize(korBytes,
      oState_.fHeight_,
      oState_.fVelocityDown_,
      oState_.oQuaternionBodyToNed_.W(),
      oState_.oQuaternionBodyToNed_.X(),
      oState_.oQuaternionBodyToNed_.Y(),
      oState_.oQuaternionBodyToNed_.Z(),
      oState_.oAccelerometerBias_.x(),
      oState_.oAccelerometerBias_.y(),
      oState_.oAccelerometerBias_.z(),
      oState_.oGyroscopeBias_.x(),
      oState_.oGyroscopeBias_.y(),
      oState_.oGyroscopeBias_.z(),
      oState_.oAccelerometerScaleFactor_.x(),
      oState_.oAccelerometerScaleFactor_.y(),
      oState_.oAccelerometerScaleFactor_.z(),
      oState_.oGyroscopeScaleFactor_.x(),
      oState_.oGyroscopeScaleFactor_.y(),
      oState_.oGyroscopeScaleFactor_.z(),
      oState_.oAccelerationNed_.x(),
      oState_.oAccelerationNed_.y(),
      oState_.oAccelerationNed_.z(),
      oStateStd_.fHeight_,
      oStateStd_.fVelocityDown_,
      oStateStd_.oAttitude_.x(),
      oStateStd_.oAttitude_.y(),
      oStateStd_.oAttitude_.z(),
      oStateStd_.oAccelerometerBias_.x(),
      oStateStd_.oAccelerometerBias_.y(),
      oStateStd_.oAccelerometerBias_.z(),
      oStateStd_.oGyroscopeBias_.x(),
      oStateStd_.oGyroscopeBias_.y(),
      oStateStd_.oGyroscopeBias_.z(),
      oStateStd_.oAccelerometerScaleFactor_.x(),
      oStateStd_.oAccelerometerScaleFactor_.y(),
      oStateStd_.oAccelerometerScaleFactor_.z(),
      oStateStd_.oGyroscopeScaleFactor_.x(),
      oStateStd_.oGyroscopeScaleFactor_.y(),
      oStateStd_.oGyroscopeScaleFactor_.z(),
      oStateStd_.oAccelerationNed_.x(),
      oStateStd_.oAccelerationNed_.y(),
      oStateStd_.oAccelerationNed_.z(),
      uTimestampUs_,
      eFilterMode_,
      uVerticalChannelDiverged);

    if (false == bStatus)
    {
      *this = CBahrsFilterOutput();
    }
    else
    {
      bVerticalChannelDiverged_ = (uVerticalChannelDiverged != 0U);
    }

    return bStatus;
  }
#endif /* SEND_DEBUG_OUTPUT || _MSC_VER */
};

struct SOutputImuData final : public NLibCommon::CSerializable
{
  float fSpecificForceX_{ 0.0F }; ///< Specific force X, [m/s^2]
  float fSpecificForceY_{ 0.0F }; ///< Specific force Y, [m/s^2]
  float fSpecificForceZ_{ 0.0F }; ///< Specific force Z, [m/s^2]
  float fAngularRateX_{ 0.0F }; ///< Angular rate X, [rad/s]
  float fAngularRateY_{ 0.0F }; ///< Angular rate Y, [rad/s]
  float fAngularRateZ_{ 0.0F }; ///< Angular rate Z, [rad/s]
  uint64_t uTimestampUs_{ 0U }; ///< Timestamp in microseconds
  CSerialProtocol::ESignalHealthInfo eSpecificForceHealth_{ CSerialProtocol::ESignalHealthInfo::eUnavailable }; ///< Health status of specific force
  CSerialProtocol::ESignalHealthInfo eAngularRateHealth_{ CSerialProtocol::ESignalHealthInfo::eUnavailable }; ///< Health status of angular rate

#if defined(SEND_DEBUG_OUTPUT) || defined(_MSC_VER)

  std::vector<uint8_t> ToByteVector() final
  {
    std::vector<uint8_t> oBytes;
    bool bStatus =  NLibCommon::Serialize(oBytes,
      fSpecificForceX_,
      fSpecificForceY_,
      fSpecificForceZ_,
      fAngularRateX_,
      fAngularRateY_,
      fAngularRateZ_,
      uTimestampUs_,
      eSpecificForceHealth_,
      eAngularRateHealth_);

    if (false == bStatus)
    {
      oBytes.clear();
    }

    return oBytes;
  }

  bool FromByteVector(const std::vector<uint8_t>& korBytes) final
  {
    bool bStatus = NLibCommon::Deserialize(korBytes,
      fSpecificForceX_,
      fSpecificForceY_,
      fSpecificForceZ_,
      fAngularRateX_,
      fAngularRateY_,
      fAngularRateZ_,
      uTimestampUs_,
      eSpecificForceHealth_,
      eAngularRateHealth_);

    if (false == bStatus)
    {
      *this = SOutputImuData();
    }

    return bStatus;
  }
#endif /* SEND_DEBUG_OUTPUT || _MSC_VER */
};

struct SSafeAttitudeData final : public NLibCommon::CSerializable
{
  float fRoll_{ 0.0F }; ///< Roll angle, [rad]
  float fPitch_{ 0.0F }; ///< Pitch angle, [rad]
  float fAttitudeStd1_{ 0.0F }; ///< Standard deviation of attitude estimate, [rad]
  float fAttitudeStd2_{ 0.0F }; ///< Standard deviation of attitude estimate, [rad]
  uint64_t uTimestampUs_{ 0U }; ///< Timestamp in microseconds
  CSerialProtocol::ESignalHealthInfo eHealth_{ CSerialProtocol::ESignalHealthInfo::eUnavailable }; ///< Attitude health information

#if defined(SEND_DEBUG_OUTPUT) || defined(_MSC_VER)

  std::vector<uint8_t> ToByteVector() final
  {
    std::vector<uint8_t> oBytes;
    bool bStatus =  NLibCommon::Serialize(oBytes,
      fRoll_,
      fPitch_,
      fAttitudeStd1_,
      fAttitudeStd2_,
      uTimestampUs_,
      eHealth_);

    if (false == bStatus)
    {
      oBytes.clear();
    }

    return oBytes;
  }

  bool FromByteVector(const std::vector<uint8_t>& korBytes) final
  {
    bool bStatus = NLibCommon::Deserialize(korBytes,
      fRoll_,
      fPitch_,
      fAttitudeStd1_,
      fAttitudeStd2_,
      uTimestampUs_,
      eHealth_);

    if (false == bStatus)
    {
      *this = SSafeAttitudeData();
    }

    return bStatus;
  }
#endif /* SEND_DEBUG_OUTPUT || _MSC_VER */
};

struct SSafeVerticalChannelData final : public NLibCommon::CSerializable
{
  float fHeight_{ 0.0F }; ///< Height, [m]
  float fVelocityDown_{ 0.0F }; ///< Vertical velocity (positive downwards), [m/s]
  uint64_t uTimestampUs_{ 0U }; ///< Timestamp in microseconds
  CSerialProtocol::ESignalHealthInfo eHealth_{ CSerialProtocol::ESignalHealthInfo::eUnavailable }; ///< Health information

#if defined(SEND_DEBUG_OUTPUT) || defined(_MSC_VER)

  std::vector<uint8_t> ToByteVector() final
  {
    std::vector<uint8_t> oBytes;
    bool bStatus =  NLibCommon::Serialize(oBytes,
      fHeight_,
      fVelocityDown_,
      uTimestampUs_,
      eHealth_);

    if (false == bStatus)
    {
      oBytes.clear();
    }

    return oBytes;
  }

  bool FromByteVector(const std::vector<uint8_t>& korBytes) final
  {
    bool bStatus = NLibCommon::Deserialize(korBytes,
      fHeight_,
      fVelocityDown_,
      uTimestampUs_,
      eHealth_);

    if (false == bStatus)
    {
      *this = SSafeVerticalChannelData();
    }

    return bStatus;
  }
#endif /* SEND_DEBUG_OUTPUT || _MSC_VER */
};

#endif /* RTE_TYPES_USER_DEFINED_H */
