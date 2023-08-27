/**
 * @file CBmm150Driver.h
 * @brief Declaration of the BMM150 driver software component
 * @author Fedor Baklanov
 * @date 08 August 2022
 */

#ifndef C_BMM_150_DRIVER_H
#define C_BMM_150_DRIVER_H

#include "stdint.h"
#include "General/CSoftwareComponentBase.h"

#define BMM150_DEF_I2C_ADDR             0x10
#define BMM150_TIMEOUT_1MS              1

/* Register definitions */
#define BMM150_REG_CHIPID               0x40
#define BMM150_DEF_CHIP_ID_VAL          0x32

#define BMM150_REG_MAG_DATA_LOW_X       0x42

#define BMM150_REG_MAG_DATA_LOW_Y       0x44

#define BMM150_REG_MAG_DATA_LOW_Z       0x46

#define BMM150_REG_PWR_CTRL             0x4B
#define BMM150_REG_PWR_CTRL_BIT         0x0

#define BMM150_REG_OP_MODE              0x4C
#define BMM150_OP_MODE_ACTIVE_NRML      0x0
#define BMM150_OP_MODE_ACTIVE_FRCD      0x1
#define BMM150_OP_MODE_SLEEP            0x3
#define BMM150_SELF_TEST_BIT            0x0

/**
 * @brief The class implements the BMM150 Driver.
 * The driver adjusts the sensor BMM150
 */
class CBmm150Driver : public CSoftwareComponent<CBmm150Driver, 2U>
{
  friend class CSoftwareComponent<CBmm150Driver, 2U>;
  FORBID_CLASS_COPY_AND_MOVE(CBmm150Driver)
  DECLARE_MANDATORY_APIS(CBmm150Driver)

public:

  /**
   * @brief BMM150 chips supported by the driver.
   */
  enum class EBmmIds : uint8_t
  {
    eInvalid = 0, ///< Invalid BMM chip
    eBmm1,        ///< The first BMM150
    eBmm2         ///< The second BMM150
  };

  static constexpr int16_t skiOvrflwAdcValXYaxisFlip_ { -4096 };
  static constexpr int16_t skiOvrflwAdcValZaxisHall_ { -16384 };
  static constexpr int16_t skiNegSaturationZaxis_ { -32767 };
  static constexpr int16_t skiPosSaturationZaxis_ { 32767 };
  static constexpr int32_t skiOvrflwOutput_ { -32768 };

  /**
   * @brief Structure of BMM150 compensation trim parameters
   */
  struct STrimRegData
  {
    /*! trim x1 data */
    int8_t iDigX1_;

    /*! trim y1 data */
    int8_t iDigY1_;

    /*! trim x2 data */
    int8_t iDigX2_;

    /*! trim y2 data */
    int8_t iDigY2_;

    /*! trim z1 data */
    uint16_t uDigZ1_;

    /*! trim z2 data */
    int16_t iDigZ2_;

    /*! trim z3 data */
    int16_t iDigZ3_;

    /*! trim z4 data */
    int16_t iDigZ4_;

    /*! trim xy1 data */
    uint8_t uDigXY1_;

    /*! trim xy2 data */
    int8_t iDigXY2_;

    /*! trim xyz1 data */
    uint16_t uDigXYZ1_;
  };

  /**
   * @brief Read Magnetic data from BMM150 Sensor
   */
  void Bmm150ReadMagData(void);

private:
  CBmm150Driver() = delete;

  /**
   * @brief Creates a driver instance for the specified BMM150 chip.
   * \param eBmmId
   */
  CBmm150Driver(EBmmIds eBmmId);

  ~CBmm150Driver() = default;

  STrimRegData oTrimRegData_;  ///< compensation trim parameters
  uint8_t uSensorStatus_ = 0;  ///< Sensor status after initialization

  /**
   * @brief Read BMM150 sensor registers
   * @param uRegAddr - Address of Sensor register
   * @param upRegData - Array of data from sensor register
   * @param uSize - Number of register data bytes to be read
   * @return true on success else false
   */
  bool readFromAddr(uint8_t uRegAddr, uint8_t* upRegData, uint8_t uSize);

  /**
   * @brief write to BMM150 sensor registers
   * @param uRegAddr - Address of Sensor register
   * @param upRegData - Array of data to be written to sensor registers
   * @return true on success else false
   */
  bool writeRegister(uint8_t uRegAddr, uint8_t *upRegData);

  /**
   * @brief Set BMM150 sensor Normal operating mode
   * @return true on success else false
   */
  bool setNormalOperationMode(void);

  /**
   * @brief Set BMM150 sensor preset configuration
   * @param uXYRep - set xy repetitions
   * @param uZRep - set z repetitions
   * @Param uODR - set operating data rate
   * @return true on success else false number
   */
  bool setPresetConfig(uint8_t uXYRep, uint8_t uZRep, uint8_t uODR);

  /**
   * @brief Read the trim registers of the sensor
   */
  bool readTrimRegisters(void);

  /**
   * @brief compensate magnetic data of x axis
   */
  int16_t compensateX(int16_t iMagDataX, uint16_t uDataRHall);

  /**
   * @brief compensate magnetic data of y axis
   */
  int16_t compensateY(int16_t iMagDataY, uint16_t uDataRHall);

  /**
   * @brief compensate magnetic data of z axis
   */
  int16_t compensateZ(int16_t iMagDataZ, uint16_t uDataRHall);

  const EBmmIds keSensorId_; ///< ID of the sensor corresponding to the driver instance.

};
#endif
