/// @file CBmm350Driver.h
/// @brief Implementation of the BMM350 driver software component class.
/// @copyright Copyright 2025. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef C_BMM350_DRIVER_H
#define C_BMM350_DRIVER_H

#include "General/CSoftwareComponentBase.h"


class CBmm350Driver : public CSoftwareComponent<CBmm350Driver, 1U>
{
  friend class CSoftwareComponent<CBmm350Driver, 1U>;
  FORBID_CLASS_COPY_AND_MOVE(CBmm350Driver)
  DECLARE_MANDATORY_APIS(CBmm350Driver)

public:
  void PollSensor(uint64_t uTimestampUs);

private:
  CBmm350Driver() = default;
  ~CBmm350Driver() = default;

  struct SCompensationParameters
  {
    float fOffsetX_{ 0.0F };
    float fOffsetY_{ 0.0F };
    float fOffsetZ_{ 0.0F };
    float fScaleX_{ 0.0F };
    float fScaleY_{ 0.0F };
    float fScaleZ_{ 0.0F };
    float fTemperature0_{ 0.0F };
    float fTemperatureOffset_{ 0.0F };
    float fTemperatureScale_{ 0.0F };
    float fOffsetTemperatureCoefX_{ 0.0F };
    float fOffsetTemperatureCoefY_{ 0.0F };
    float fOffsetTemperatureCoefZ_{ 0.0F };
    float fScaleTemperatureCoefX_{ 0.0F };
    float fScaleTemperatureCoefY_{ 0.0F };
    float fScaleTemperatureCoefZ_{ 0.0F };
    float fCrossAxisSensitivityXY_{ 0.0F };
    float fCrossAxisSensitivityYX_{ 0.0F };
    float fCrossAxisSensitivityZY_{ 0.0F };
    float fCrossAxisSensitivityZX_{ 0.0F };
  };

  using COtpDataVector = std::array<uint16_t, 32U>;

  struct CVector3dAndTemperature
  {
//    std::array<float, 3U> oVector_{0.0F, 0.0F, 0.0F};
    float fMagneticVectorX_{0.0F}; // X-achsis in sensor coordinate system
    float fMagneticVectorY_{0.0F}; // Y-achsis in sensor coordinate system
    float fMagneticVectorZ_{0.0F}; // Z-achsis in sensor coordinate system
    float fTemperatureInDegreesCelsius_{0.0F};
  };

  SCompensationParameters oCompensationParameters_{};

  /// @brief Compensate raw magnetic measurement using temperature and compensation parameters.
  ///
  /// ## Compensation formulae
  /// The compensation process comprises 3 steps:
  /// 1. Compensation of the temperature readings
  /// 2. Offset and scale compensation (temperature dependent)
  /// 3. Cross-axis compensation (temperature independent)
  ///
  /// ### Step 1: Temperature compensation
  /// The raw temperature reading is first compensated as \f$ T = (1 + s_T) T^{raw} + b_T \f$, where
  /// - \f$ T^{raw} \f$ is the raw temperature measurement,
  /// - \f$ s_T \f$ is the temperature scale factor,
  /// - \f$ b_T \f$ is the temperature offset,
  /// - \f$ T \f$ is the compensated temperature.
  ///
  /// ### Step 2: Offset and scale compensation
  /// For each axis X, Y, and Z, the compensated value is calculated as:
  /// \f[
  ///
  /// B^{tmp}_{(\cdot)} = \frac{1}{1 + c^s_{(\cdot)} (T - T_0)} \cdot \left( (1 + s_{(\cdot)}) \cdot B^{raw}_{(\cdot)} + b_{(\cdot)} + c^{o}_{(\cdot)} (T - T_0) \right)
  ///
  /// \f]
  /// where:
  /// - \f$ B^{tmp}_{(\cdot)} \f$ is the intermediate compensated magnetic field.
  /// - \f$ B^{raw}_{(\cdot)} \f$ is the raw magnetic field measurement.
  /// - \f$ T \f$ is the current temperature measurement.
  /// - \f$ T_0 \f$ is the reference temperature.
  /// - \f$ s_{(\cdot)} \f$ is the scale factor.
  /// - \f$ b_{(\cdot)} \f$ is the offset.
  /// - \f$ c^{o}_{(\cdot)} \f$ is the offset temperature coefficient.
  /// - \f$ c^{s}_{(\cdot)} \f$ is the scale temperature coefficient.
  ///
  /// The symbol \f$ (\cdot) \f$ represents the respective axis: X, Y, or Z.
  ///
  /// ### Step 3: Cross-axis compensation
  /// The final compensated magnetic field values are computed as:
  /// \f{eqnarray*}{
  /// B_{x} &=& \frac{B^{tmp}_x - c_{xy} B^{tmp}_{y}}{1 - c_{yx} c_{xy}} \newline
  /// B_{y} &=& \frac{B^{tmp}_y - c_{yx} B^{tmp}_{x}}{1 - c_{yx} c_{xy}} \newline
  /// B_{z} &=& B^{tmp}_z + \frac{B^{tmp}_{x} (c_{yx} c_{zy} - c_{zx}) - B^{tmp}_{y} (c_{zy} - c_{xy} c_{zx})}{1 - c_{yx} c_{xy}}
  /// \f}
  /// where:
  /// - \f$ B_{(\cdot)} \f$ is the final compensated magnetic field.
  /// - \f$ c_{(\cdot)} \f$ are the cross-axis sensitivity coefficients.
  ///
  /// @param korRawData Raw magnetometer readings as a 3D vector (X, Y, Z) and raw temperature.
  /// @param oCompensationParameters Reference to the structure containing the compensation parameters.
  /// @return Compensated magnetometer measurement as a 3D vector (X, Y, Z).
  static CVector3dAndTemperature compensateMeasurement(const CVector3dAndTemperature& korRawData, const SCompensationParameters& oCompensationParameters);

  /// @brief Read-out the sensor's OTP words and decode compensation parameters.
  ///
  /// OTP overview:
  /// - OTP (one-time programmable) is a non-volatile trim memory used for calibration and variant data.
  /// - Size is 32 words (indices 0x00..0x1F).
  /// - Word length is 16 bits, read as MSB and LSB bytes and combined into a single uint16_t.
  ///
  /// OTP word map and field-to-parameter mapping (shared words explicitly listed):
  /// - Word 0x0D
  ///   - [7:0]: Temperature sensor offset, 8-bit, signed, scaled by 1/5, [degC]
  ///   - [15:8]: Temperature sensor scale correction, 8-bit, signed, scaled by 1/512, [-]
  static constexpr uint8_t skuBmm350OtpWordTempOffsetScale_{ 0x0D };
  /// - Word 0x0E
  ///   - [11:0]: Magnetometer offset for X axis in [uT], signed 12-bit
  ///   - [15:12]: High 4 bits of magnetometer offset for Y axis in [uT]
  static constexpr uint8_t skuBmm350OtpWordMagOffsetXY_{ 0x0E };
  /// - Word 0x0F
  ///   - [7:0]: Low 8 bits of magnetometer offset for Y axis in [uT]
  ///   - [11:8]: High 4 bits of magnetometer offset for Z axis in [uT]
  static constexpr uint8_t skuBmm350OtpWordMagOffsetYZ_{ 0x0F };
  /// - Word 0x10
  ///   - [7:0]: Low 8 bits of magnetometer offset for Z axis in [uT]
  ///   - [15:8]: Magnetometer scale for X axis, signed 8-bit, scaled by 1/256
  static constexpr uint8_t skuBmm350OtpWordMagOffsetZScaleX_{ 0x10 };
  /// - Word 0x11
  ///   - [7:0]: Magnetometer scale for Y axis, signed 8-bit, scaled by 1/256
  ///   - [15:8]: Magnetometer scale for Z axis, signed 8-bit, scaled by 1/256
  static constexpr uint8_t skuBmm350OtpWordMagScaleYZ_{ 0x11 };
  /// - Word 0x12:
  ///   - [7:0]: Temperature coefficient for the X axis offset, signed 8-bit, scaled by 1/32, [uT/degC]
  ///   - [15:8]: Temperature coefficient for the X axis scale, signed 8-bit, scaled by 1/16384, [1/degC]
  static constexpr uint8_t skuBmm350OtpWordTempCoefOffsetXScaleX_{ 0x12 };
  /// - Word 0x13:
  ///   - [7:0]: Temperature coefficient for the Y axis offset, signed 8-bit, scaled by 1/32, [uT/degC]
  ///   - [15:8]: Temperature coefficient for the Y axis scale, signed 8-bit, scaled by 1/16384, [1/degC]
  static constexpr uint8_t skuBmm350OtpWordTempCoefOffsetYScaleY_{ 0x13 };
  /// - Word 0x14:
  ///   - [7:0]: Temperature coefficient for the Z axis offset, signed 8-bit, scaled by 1/32, [uT/degC]
  ///   - [15:8]: Temperature coefficient for the Z axis scale, signed 8-bit, scaled by 1/16384, [1/degC]
  static constexpr uint8_t skuBmm350OtpWordTempCoefOffsetZScaleZ_{ 0x14 };
  /// - Word 0x15:
  ///   - [7:0]: Cross-axis sensitivity XY, signed 8-bit, scaled by 1/800, [-]
  ///   - [15:8]: Cross-axis sensitivity YX, signed 8-bit, scaled by 1/800, [-]
  static constexpr uint8_t skuBmm350OtpWordCrossAxisSensXYYX_{ 0x15 };
  /// - Word 0x16:
  ///   - [7:0]: Cross-axis sensitivity ZX, signed 8-bit, scaled by 1/800, [-]
  ///   - [15:8]: Cross-axis sensitivity ZY, signed 8-bit, scaled by 1/800, [-]
  static constexpr uint8_t skuBmm350OtpWordCrossAxisSensZXZY_{ 0x16 };
  /// - Word 0x18: Reference temperature, signed 16-bit, scaled by 1/512, then shifted by +23, [degC]
  static constexpr uint8_t skuBmm350OtpWordReferenceTemp_{ 0x18 };
  /// - Word 0x1E: variant ID derived from bits [14:9]
  ///
  /// Principle for reading a single OTP word:
  /// 1. Issue a direct-read command with the word address masked to 5 bits.
  /// 2. Poll the OTP status register until the command-done bit is set.
  /// 3. If any error bits are set, translate them to an error status.
  /// 4. On success, read OTP data MSB and LSB registers and combine into a 16-bit word.
  ///
  /// Required behavior:
  /// 1. Read the full OTP word range in ascending index order. Store each word into the
  ///    device's OTP array at the same index. Do not skip indices. The read-out loop aborts
  ///    on a single read error.
  /// 3. Decode the compensation parameters only if all OTP words were read successfully;
  ///    otherwise, leave the compensation structure unchanged and return a failure status.

  static constexpr float skfBmm350BxySens_{ 14.55F };
  static constexpr float skfBmm350BzSens_{ 9.0F };
  static constexpr float skfBmm350TempSens_{ 0.00204F };
  static constexpr float skfBmm350InaXyGainTarget_{ 19.46F };
  static constexpr float skfBmm350InaZGainTarget_{ 31.0F };
  static constexpr float skfBmm350AdcGain_{ 1.0F / 1.5F };
  static constexpr float skfBmm350LutGain_{ 0.714607238769531F };
  static constexpr float skfBmm350AdcCounts_{ 1048576.0F };
  static constexpr float skfBmm350MicroScale_{ 1000000.0F };
  static constexpr float skfBmm350PowerScale_{ skfBmm350MicroScale_ / skfBmm350AdcCounts_ };

  static constexpr float skfBmm350LsbToMicroTeslaXy_{ skfBmm350PowerScale_ /
                                             (skfBmm350BxySens_ * skfBmm350InaXyGainTarget_ *
                                              skfBmm350AdcGain_ * skfBmm350LutGain_) };

  static constexpr float skfBmm350LsbToMicroTeslaZ_{ skfBmm350PowerScale_ /
                                            (skfBmm350BzSens_ * skfBmm350InaZGainTarget_ *
                                             skfBmm350AdcGain_ * skfBmm350LutGain_) };

  static constexpr float skfBmm350LsbToDegC_{ 1.0F /
                                             (skfBmm350TempSens_ * skfBmm350AdcGain_ *
                                              skfBmm350LutGain_ * skfBmm350AdcCounts_) };

  static constexpr float skfBmm350TemperatureShiftDegC_{ 25.49F };

  static constexpr float skfBmm350MicroTeslaToGauss_{ 0.01F };

  static constexpr uint16_t skuBmm350I2CAddress_{ 0x0015 << 1U }; // SAD+R/W

  // Registers
  static constexpr uint8_t skuBmm350RegDeviceId_{ 0x00 };
  static constexpr uint8_t skuBmm350RegPmuCmdAggrSet_{ 0x04 };
  static constexpr uint8_t skuBmm350RegPmuCmdAxisEn_{ 0x05 };
  static constexpr uint8_t skuBmm350RegPmuCmd_{ 0x06 };
  static constexpr uint8_t skuBmm350RegIntCtrl_{ 0x2E };
  static constexpr uint8_t skuBmm350RegIntStatus_{ 0x30 };
  static constexpr uint8_t skuBmm350RegMagXXlsb_{ 0x31 };

  static constexpr uint8_t skuBmm350ChipId_{ 0x33 };

  // OTP registers
  static constexpr uint8_t skuBmm350RegOtpCmd_{ 0x50 };
  static constexpr uint8_t skuBmm350RegOtpMsb_{ 0x52 };
  static constexpr uint8_t skuBmm350RegOtpStatus_{ 0x55 };

  // Register OTP_CMD_REG (0x50)
  static constexpr uint8_t skuBmm350OtpCmdDirRead_{ 0x20 };
  static constexpr uint8_t skuBmm350OtpCmdPwrOffOtp_{ 0x80 };
  static constexpr uint8_t skuBmm350OtpWordAddrMsk_{ 0x1F };

  static constexpr uint8_t skuBmm350OtpStatusCmdDone_{ 0x01 };
  static constexpr uint8_t skuBmm350OtpStatusMsk_{ 0xE0 };
  static constexpr uint8_t skuBmm350OtpStatusNoError_{ 0x00 };

  // Register INT_CRTL (0x2E)
  static constexpr uint8_t skuBmm350IntOutputEn_{ 0x01 << 3U };
  static constexpr uint8_t skuBmm350RdryDataRegEn_{ 0x01 << 7U }; // : Enable Mag Data Ready interrupt onto INT pin and INT_STATUS
  static constexpr uint8_t skuBmm350RegIntCtrlValue_{ skuBmm350IntOutputEn_ | skuBmm350RdryDataRegEn_ };

  // Register PMU_CMD_AGGR_SET (0x04)
  static constexpr uint8_t skuBmm350Odr12Hz5_{ 0x07 }; // 12.5 Hz ODR
  static constexpr uint8_t skuBmm350Average8_{ 0x03 << 4U }; // Ultra Low Noise - Average between 8 samples
  static constexpr uint8_t skuBmm350Odr12Hz5Average8_ = skuBmm350Odr12Hz5_ | skuBmm350Average8_;

  // Register PMU_CMD_AXIS_EN (0x05)
  static constexpr uint8_t skuBmm350EnableXYZAxis_{ 0x07 }; // Enable X, Y and Z axis

  // Register PMU_CMD_AXIS_EN (0x06)
  static constexpr uint8_t skuBmm350PmuCmdSus_{ 0x00 }; // Suspend mode
  static constexpr uint8_t skuBmm350PmuCmdNm_{ 0x01 }; // Normal mode
  static constexpr uint8_t skuBmm350PmuCmdOae_{ 0x02 };
  static constexpr uint8_t skuBmm350PmuCmdBrFast_{ 0x08 };

  // Register INT_STATUS (0x30)
  static constexpr uint8_t skuBmm350DrDyDataReg_{ 0x01 << 2U };

  // I2C timeout
  static constexpr uint32_t skuBmm350I2cTimeout_{ 2U };
  static constexpr uint32_t skuBmm350DummyBytes_{ 2U };
  static constexpr uint32_t skuBmm350I2cBufferSize_{ 127U };

  bool getDeviceId(uint8_t& urChipId);
  bool readOtp();
  void updateCompensationParameters(COtpDataVector& auOtpData);
  bool readUncompensatedMagAndTempData(CVector3dAndTemperature& orRawSensorData);
  bool readCompensatedMagAndTempData(CVector3dAndTemperature& orSensorData);
  bool configureInterrupt();
  bool configureOdrAndAveraging();
  bool enableXYZAxis();
  bool enableNormalMode();

  bool readReg(uint8_t uRegAddr, uint8_t* upBuffer, uint16_t uLen);
  bool writeReg(uint8_t uRegAddr, const uint8_t* upBuffer, uint16_t uLen);

  bool bIsInitialized_{ false };  ///< Sensor status after initialization
};

#endif /* C_BMM350_DRIVER_H */
