/**
 * @file CBmi270Driver.h
 * @brief Declaration of the BMI270 driver software component class.
 * @copyright Copyright 2025. AMS Advanced Air Mobility Sensors UG. All rights reserved.
 */

#ifndef C_BMI270_DRIVER_H
#define C_BMI270_DRIVER_H

#include "General/CSoftwareComponentBase.h"
#include <array>

class CBmi270Driver : public CSoftwareComponent<CBmi270Driver, 1U>
{
  friend class CSoftwareComponent<CBmi270Driver, 1U>;
  FORBID_CLASS_COPY_AND_MOVE(CBmi270Driver)
  DECLARE_MANDATORY_APIS(CBmi270Driver)

public:
  void PollSensor(uint64_t uTimestamp);

private:
  CBmi270Driver() = default;
  ~CBmi270Driver() = default;

  static const uint16_t skuBmi270ConfigFileSize_{ 8192U };
  static const uint8_t skauBmi270ConfigFile[skuBmi270ConfigFileSize_];

  static constexpr uint8_t skuBmi270RegChipId_{ 0x00 };
  static constexpr uint8_t skuBmi270RegStatus_{ 0x03 };
  static constexpr uint8_t skuBmi270RegPwrCtrl_{ 0x7D };
  static constexpr uint8_t skuBmi270RegPwrConf_{ 0x7C };
  static constexpr uint8_t skuBmi270RegInitCtrl_{ 0x59 };
  static constexpr uint8_t skuBmi270RegInitAddr1_{ 0x5C };
  static constexpr uint8_t skuBmi270RegInitData_{ 0x5E };
  static constexpr uint8_t skuBmi270RegIntStat_{ 0x21 };

  static constexpr uint8_t skuBmi270RegTemp0_{ 0x22 };
  static constexpr uint8_t skuBmi270RegTemp1_{ 0x23 };

  static constexpr uint8_t skuBmi270RegAccelConf_{ 0x40 };
  static constexpr uint8_t skuBmi270RegAccelRange_{ 0x41 };
  static constexpr uint8_t skuBmi270RegGyroConf_{ 0x42 };
  static constexpr uint8_t skuBmi270RegGyroRange_{ 0x43 };

  static constexpr uint8_t skuBmi270RegIntStatus1_{ 0x1D }; // p. 83
  static constexpr uint8_t skuBmi270RegIntMapData_{ 0x58 }; // p. 112
  static constexpr uint8_t skuBmi270RegInt1IoCtrl_{ 0x53 }; // p. 109

  static constexpr uint8_t skuBmi270RegAccXLsb_{ 0x0C };

  static constexpr uint8_t skuBmi270ChipId_{ 0x24 };

  static constexpr uint8_t skuBmi270AdvPwrSaveMsk_{ 0x01 }; // 5.2.84 reg. 0x7c
  static constexpr uint8_t skuBmi270EnableGyroAccelTemp_{ 0x0E };

  static constexpr uint8_t skuBmi270PrepareConfLoad_{ 0x00 };
  static constexpr uint8_t skuBmi270CompleteConfLoad_{ 0x01 };
  static constexpr uint8_t skuBmi270AsicInitDoneMsk_{ 0x0F };
  static constexpr uint8_t skuBmi270AsicInitDone_{ 0x01 };
  static constexpr uint8_t skuBmi270InitAddr1Pos_{ 4U };
  static constexpr uint16_t skuBmi270InitDataSize_{ 8192U };
  static constexpr uint16_t skuBmi270ChunkSize_{ 512U }; // 512 skuBytes per SPI transfer
  static constexpr uint16_t skuBmi270PacketNum_{ skuBmi270InitDataSize_ / skuBmi270ChunkSize_ };

  static constexpr uint8_t skuBmi270AccelOdr_200_{ 0x09 }; // p. 100
  static constexpr uint8_t skuBmi270AccelOsr_4_{ 0x00 };
  static constexpr uint8_t skuBmi270AccelFiltPerfOpt_{ 0x01 << 7U };
  static constexpr uint8_t skuBmi270AccelRange8G_{ 0x02 };

  static constexpr uint8_t skuBmi270AccelConfVal_{ skuBmi270AccelOdr_200_ | skuBmi270AccelOsr_4_ | skuBmi270AccelFiltPerfOpt_ };
  static constexpr uint8_t skuBmi270AccelRangeVal_{ skuBmi270AccelRange8G_ }; // p. 102

  static constexpr float skfBmi270AccelSensitivity_{ 1.0F / 4096.0F }; // Table 2: Sensitivity
  static constexpr float skfStandardGravity_{ 9.80665F };

  static constexpr uint8_t skuBmi270GyroOdr200_{ 0x09 };
  static constexpr uint8_t skuBmi270GyroOsr4_{ 0x00 };
  static constexpr uint8_t skuBmi270GyroNoisePerfOpt_{ 0x01 << 6U };
  static constexpr uint8_t skuBmi270GyroFiltPerfOpt_{ 0x01 << 7U };
  static constexpr uint8_t skuBmi270GyroRange500_{ 0x02 };

  static constexpr uint8_t skuBmi270GyroConfVal_{ skuBmi270GyroOdr200_ | skuBmi270GyroOsr4_ | skuBmi270GyroNoisePerfOpt_ | skuBmi270GyroFiltPerfOpt_ };
  static constexpr uint8_t skuBmi270GyroRangeVal_{ skuBmi270GyroRange500_ };

  static constexpr float skfBmi270GyroSensitivity_{ 1.0F / 65.536F }; // Table 3: Sensitivity
  static constexpr float skfDegToRad_{ 3.141592F / 180.0F };

  static constexpr float skfBmi270TempSensitivity_{ 1.0F / 512.0F }; // Table 4: Sensitivity

  static constexpr uint8_t skuBmi270GyroDataReadyInt_{ 1 << 6U };
  static constexpr uint8_t skuBmi270AccelDataReadyInt_{ 1 << 7U };
  static constexpr uint8_t skuBmi270GyroAccelDatareadyInt_{ skuBmi270GyroDataReadyInt_ | skuBmi270AccelDataReadyInt_ };

  static constexpr uint8_t skuBmi270DrdyInt1_{ 1 << 2U };

  static constexpr uint8_t skuBmi270Int1IoOpenDrain_{ 1 << 2U };
  static constexpr uint8_t skuBmi270Int1IoOutputEnable_{ 1 << 3U };
  static constexpr uint8_t skuBmi270Int1IoCtrlValue_{ skuBmi270Int1IoOpenDrain_ | skuBmi270Int1IoOutputEnable_ };

  static constexpr uint8_t skuBmi270GyroDrFlag_{ 1 << 6U }; // Gets reset, when one Gyroscope DATA register is read out (p. 75)
  static constexpr uint8_t skuBmi270AccelDrFlag_{ 1 << 7U }; // Gets reset, when one Accelerometer DATA register is read out (p. 75)

  bool getChipID(uint8_t& urChipId);
  bool getIntStatusReg(uint8_t& urBmiInitStatus);
  bool enableGyroAccelTemp();
  bool configAccel();
  bool configGyro();
  bool configIntPin();
  bool isAccelGyroDataReady();

  using CSpecificForce = std::array<float, 3>;
  using CAngularRate = std::array<float, 3>;

  bool readAccelAndGyro(CSpecificForce& orSpecificForce, CAngularRate& orAngularRate);

  bool readTemp(float& fTemperature);
  bool readReg(uint8_t uRegAddr, uint8_t* upBuffer, uint16_t uLen);
  bool writeReg(uint8_t uRegAddr, const uint8_t* upBuffer, uint16_t uLen);

  bool bIsInitialized_{ false };  ///< Sensor status after initialization
};

#endif /* C_BMI270_DRIVER_H */
