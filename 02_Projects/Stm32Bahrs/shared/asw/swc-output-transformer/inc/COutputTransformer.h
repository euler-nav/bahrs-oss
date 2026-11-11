/// @file COutputTransformer.h
/// @brief Declaration of the Output Transformer SWC class.
/// @copyright Copyright 2023. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef C_OUTPUT_TRANSFORMER_H
#define C_OUTPUT_TRANSFORMER_H

#include "Eigen/Dense"
#include "General/CSoftwareComponentBase.h"
#include "CSerialProtocol.h"

/// @brief The class implements a SW component that converts internal signals to customer-specific signals.
class COutputTransformer : public CSoftwareComponent<COutputTransformer, 1U>
{
  friend class CSoftwareComponent<COutputTransformer, 1U>;
  FORBID_CLASS_COPY_AND_MOVE(COutputTransformer)
  DECLARE_MANDATORY_APIS(COutputTransformer)

public:
  /// @brief A runnable that transforms IMU signals to customer-defined vehicle frame.
  void TransformImuSignals();

  /// @brief A runnable that computes orientation of the customer-defined vehicle frame
  /// The function derives the orientation from estimated orientation of the BAHRS body frame
  /// and customer-configured rotation from the BAHRS body frame to the vehicle frame.
  void TransformOrientation();

  /// @brief A runnable that transforms magnetometer measurements to vehicle frame.
  /// @param uSensorIndex Zero-based magnetometer index (0 to 2).
  void TransformMagnetometerInput(uint32_t uSensorIndex);

protected:

private:
  COutputTransformer() = default;
  ~COutputTransformer() = default;

  /// @brief Read compensated magnetometer data of the selected sensor from its RTE port.
  /// @param uSensorIndex Zero-based magnetometer index (0 to 2).
  /// @param orData Magnetic measurement read from the port.
  /// @return True if the read succeeded.
  static bool readCompensatedMagnetometerPort(uint32_t uSensorIndex, SMagneticMeasurement& orData);

  /// @brief Write magnetometer data in vehicle frame to the RTE port of the selected sensor.
  /// @param uSensorIndex Zero-based magnetometer index (0 to 2).
  /// @param korData Magnetic measurement to write.
  static void writeMagnetometerInVehicleFramePort(uint32_t uSensorIndex, const SMagneticMeasurement& korData);

  /// @brief Checks if a 3D signal composed of scalar values that are provided by the monitor, is valid and safe.
  /// @tparam eSignal1 Label of the 1st scalar signal.
  /// @tparam eSignal2 Label of the 2nd scalar signal.
  /// @tparam eSignal3 Label of the 3rd scalar signal.
  /// @param korImuData Output of the IMU monitor.
  /// @return Health information.
  template<CImuDataAfterMonitor::EScalarSignals eSignal1,
           CImuDataAfterMonitor::EScalarSignals eSignal2,
           CImuDataAfterMonitor::EScalarSignals eSignal3>
  static CSerialProtocol::ESignalHealthInfo buildHealthInfoForVectorSignal(const CImuDataAfterMonitor& korImuData);

  CQuaternion oQuatVehicleToDevice_;
  Eigen::Matrix3f oDcmDeviceToVehicle_ { Eigen::Matrix3f::Identity() };
  bool bIsInitialized_ { false };
};

#endif // C_OUTPUT_TRANSFORMER_H
