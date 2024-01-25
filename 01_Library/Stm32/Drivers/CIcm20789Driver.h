/**
 * @file CIcm20789Driver.h
 * @brief Declaration of the ICM20789 Driver software component
 * @author Evgeniy Zamyatin
 * @date 22 June 2022
 */

#ifndef C_ICM_20789_DRIVER_H
#define C_ICM_20789_DRIVER_H

#include <stdint.h>
#include "General/CSoftwareComponentBase.h"
#include "i2c.h"

// Sensor constants
#define ICM20789_ADDRESS_IMU                ((uint8_t)0x68) ///< Address of the sensor
#define ICM20789_ADDRESS_PRESS              ((uint8_t)0x63) ///< Address of the pressure sensor
#define ICM20789_TIMEOUT                    (2U) ///< A timeout for read/write operations
#define ICM20789_TIMEOUT_OTP                (5U) ///< A timeout for pressure calibration parameters read operations
#define ICM20789_IMU_ID                     ((uint8_t)0x03) ///< IMU ID stored in the WHO_AM_I register
#define ICM20789_PRESSURE_ID                ((uint8_t)0x08) ///< ID of the pressure sensor stored in the sensor register
#define ICM20789_GYRO_ENGINE_UP_TIME        (50) ///< Gyroscope start-up time, [ms]
#define ICM20789_IMU_BYTES_TO_POLL          (14) ///< Number of bytes to poll from the IMU at every cycle

// Register addresses
#define ICM20789_IMU_REG_SMPLRT_DIV         ((uint8_t)0x19) ///< IMU's sample rate divider register
#define ICM20789_IMU_REG_CONFIG             ((uint8_t)0x1A) ///< IMU's configuration register
#define ICM20789_IMU_REG_ACCEL_XOUT_H       ((uint8_t)0x3B) ///< Accelerometer measurement register (X axis, upper byte)
#define ICM20789_IMU_REG_WHO_AM_I           ((uint8_t)0x75) ///< IMU's who am I register
#define ICM20789_IMU_REG_GYRO_CONFIG        ((uint8_t)0x1B) ///< IMU's gyro configuration register
#define ICM20789_IMU_REG_ACCEL_CONFIG       ((uint8_t)0x1C) ///< IMU's accelerometer configuration register
#define ICM20789_IMU_REG_ACCEL_CONFIG_2     ((uint8_t)0x1D) ///< IMU's accelerometer configuration register 2
#define ICM20789_IMU_REG_LP_CONFIG          ((uint8_t)0x1E) ///< IMU's low power mode configuration register
#define ICM20789_IMU_REG_FIFO_EN            ((uint8_t)0x23) ///< IMU's register for enabling/disabling FIFO
#define ICM20789_IMU_REG_INT_ENABLE         ((uint8_t)0x38) ///< IMU's interrupt enable register
#define ICM20789_IMU_REG_USER_CTRL          ((uint8_t)0x6A) ///< IMU's user control register
#define ICM20789_IMU_REG_PWR_MGMT1          ((uint8_t)0x6B) ///< IMU's power management register 1
#define ICM20789_IMU_REG_PWR_MGMT2          ((uint8_t)0x6C) ///< IMU's power management register 2
#define ICM20789_IMU_REG_FIFO_COUNTH        ((uint8_t)0x72) ///< IMU's upper byte of FIFO count register
#define ICM20789_IMU_REG_FIFO_R_W           ((uint8_t)0x74) ///< IMU's register for FIFO read/write command
#define ICM20789_IMU_REG_INT_PIN_CFG        ((uint8_t)0x37) ///< IMU's interrupt/bypass configuration register
#define ICM20789_IMU_REG_GYRO_X_ST          ((uint8_t)0x00) ///< IMU's gyro X self-test register
#define ICM20789_IMU_REG_ACCEL_X_ST         ((uint8_t)0x0D) ///< IMU's accel X self-test register

// Register configuration values and commands
#define ICM20789_PWR_MGMT1_VALUE            ((uint8_t)0x01) ///< A configuration value for the power management 1 register of the IMU
#define ICM20789_PWR_MGMT2_VALUE            ((uint8_t)0xC0) ///< A configuration value for the power management 2 register of the IMU
#define ICM20789_SMPLRT_DIV_VALUE           ((uint8_t)0x04) ///< Sample rate divider of the IMU
#define ICM20789_BIT_FIFO_RST               ((uint8_t)0x04) ///< A command to reset FIFO
#define ICM20789_BIT_FIFO_EN                ((uint8_t)0x40) ///< A command to enable FIFO

// Auxiliary constants
#define ICM20789_BYTES_PER_IMU_SENSOR       (6) ///< Number of bytes per measurement of a gyroscope and an accelerometer
#define ICM20789_BYTES_PER_TEMP_SENSOR      (2) ///< Number of bytes per measurement of a temperature sensor

// Constants related to the self test procedure suggested by the manufacturer
#define ICM20789_SELF_TEST_PRECISION        (1000) ///< A precision parameter for the IMU self-test

#define ICM20789_GYRO_SELF_TEST_SHIFT_DELTA  (500)
#define ICM20789_ACCEL_SELF_TEST_SHIFT_DELTA (500)

#define ICM20789_SELF_TEST_ACCEL_FS_MG      (2000) ///< Default accelerometer full scale for a self test, [mg]
#define ICM20789_SELF_TEST_GYRO_FS_DPS      (250) ///< Default gyroscope full scale for a self test, [deg/s]
#define ICM20789_SELF_TEST_SCALE            (32768) ///< Self test scale
#define ICM20789_SELF_TEST_GYRO_SENS        (ICM20789_SELF_TEST_SCALE / ICM20789_SELF_TEST_GYRO_FS_DPS)
#define ICM20789_GYRO_SELF_TEST_AL          (60) ///< Gyroscope self-test absolute limit, [deg/s]
#define ICM20789_GYRO_OFFSET_MAX            (20) ///< Maximum gyro offset value, [deg/s]

#define ICM20789_ACCEL_SELF_TEST_AL_MIN     (225) ///< Minimum absolute limit for accelerometer self-test, [mg]
#define ICM20789_ACCEL_SELF_TEST_AL_MAX     (675) ///< Maximum absolute limit for accelerometer self-test, [mg]

#define ACCEL_SELF_TEST_AL_MIN ((ICM20789_ACCEL_SELF_TEST_AL_MIN * ICM20789_SELF_TEST_SCALE \
                / ICM20789_SELF_TEST_ACCEL_FS_MG) * ICM20789_SELF_TEST_PRECISION)
#define ACCEL_SELF_TEST_AL_MAX ((ICM20789_ACCEL_SELF_TEST_AL_MAX * ICM20789_SELF_TEST_SCALE \
                / ICM20789_SELF_TEST_ACCEL_FS_MG) * ICM20789_SELF_TEST_PRECISION)

#define ICM20789_BIT_XG_ST                  ((uint8_t)0x80) ///< Self-test enabler bits in the dedicated register
#define ICM20789_BIT_YG_ST                  ((uint8_t)0x40) ///< Self-test enabler bits in the dedicated register
#define ICM20789_BIT_ZG_ST                  ((uint8_t)0x20) ///< Self-test enabler bits in the dedicated register

class CIcm20789Driver : public CSoftwareComponent<CIcm20789Driver, 2U>
{
  friend class CSoftwareComponent<CIcm20789Driver, 2U>;
  FORBID_CLASS_COPY_AND_MOVE(CIcm20789Driver)
  DECLARE_MANDATORY_APIS(CIcm20789Driver)

public:
  /**
   * @brief ICM20789 chip instances supported by the driver.
   */
  enum class EIcmIds : uint8_t
  {
    eInvalid = 0, ///< Invalid ICM chip
    eIcm1,        ///< The first ICM20789
    eIcm2         ///< The second ICM20789
  };

  /**
   * @brief Implements an iteration of cyclic data exchange with the pressure sensor
   * */
  void PollPressureSensor();

  /**
   * @brief Implements an iteration of cyclic data exchange with the inertial sensor
   * */
  void PollInertialSensor();

  /**
   * @brief Request IMU data in DMA mode.
   * When reception is completed, received bytes can be processed in the interrupt handler.
   * \return Status of the DMA request, true -- OK, false otherwise.
   */
  bool RequestInertialSensorDataDma();

  /**
   * @brief Parses IMU data received in DMA mode.
   */
  void ParseReceivedImuDataDma();

private:
  CIcm20789Driver() = delete;

  /**
   * \brief The only valid constructor.
   * \param eIcmChipId An ID of the ICM20789 chip to instantiate a driver for.
   */
  CIcm20789Driver(EIcmIds eIcmChipId);

  /**
   * \brief Compensation parameters of the pressure sensors.
   * The parameters are retrieved from the sensor's internal memory.
   */
  struct SCalibParam
  {
    float afSensorConstants_[4];
    float afPaCalib_[3];
    float fLutLower_;
    float fLutUpper_;
    float fQuadrFactor_;
    float fOffstFactor_;
  };

  enum class EGyroDynamicRange
  {
    e250dps    = 0x00,
    e500dps    = 0x01,
    e1000dps   = 0x02,
    e2000dps   = 0x03,
  };

  enum class EAccelDynamicRange
  {
    e2g    = 0x00,
    e4g    = 0x01,
    e8g    = 0x02,
    e16g   = 0x03,
  };

  const uint16_t kauSelfTestEquation[256] =
  {
    2620, 2646, 2672, 2699, 2726, 2753, 2781, 2808,
    2837, 2865, 2894, 2923, 2952, 2981, 3011, 3041,
    3072, 3102, 3133, 3165, 3196, 3228, 3261, 3293,
    3326, 3359, 3393, 3427, 3461, 3496, 3531, 3566,
    3602, 3638, 3674, 3711, 3748, 3786, 3823, 3862,
    3900, 3939, 3979, 4019, 4059, 4099, 4140, 4182,
    4224, 4266, 4308, 4352, 4395, 4439, 4483, 4528,
    4574, 4619, 4665, 4712, 4759, 4807, 4855, 4903,
    4953, 5002, 5052, 5103, 5154, 5205, 5257, 5310,
    5363, 5417, 5471, 5525, 5581, 5636, 5693, 5750,
    5807, 5865, 5924, 5983, 6043, 6104, 6165, 6226,
    6289, 6351, 6415, 6479, 6544, 6609, 6675, 6742,
    6810, 6878, 6946, 7016, 7086, 7157, 7229, 7301,
    7374, 7448, 7522, 7597, 7673, 7750, 7828, 7906,
    7985, 8065, 8145, 8227, 8309, 8392, 8476, 8561,
    8647, 8733, 8820, 8909, 8998, 9088, 9178, 9270,
    9363, 9457, 9551, 9647, 9743, 9841, 9939, 10038,
    10139, 10240, 10343, 10446, 10550, 10656, 10763, 10870,
    10979, 11089, 11200, 11312, 11425, 11539, 11654, 11771,
    11889, 12008, 12128, 12249, 12371, 12495, 12620, 12746,
    12874, 13002, 13132, 13264, 13396, 13530, 13666, 13802,
    13940, 14080, 14221, 14363, 14506, 14652, 14798, 14946,
    15096, 15247, 15399, 15553, 15709, 15866, 16024, 16184,
    16346, 16510, 16675, 16842, 17010, 17180, 17352, 17526,
    17701, 17878, 18057, 18237, 18420, 18604, 18790, 18978,
    19167, 19359, 19553, 19748, 19946, 20145, 20347, 20550,
    20756, 20963, 21173, 21385, 21598, 21814, 22033, 22253,
    22475, 22700, 22927, 23156, 23388, 23622, 23858, 24097,
    24338, 24581, 24827, 25075, 25326, 25579, 25835, 26093,
    26354, 26618, 26884, 27153, 27424, 27699, 27976, 28255,
    28538, 28823, 29112, 29403, 29697, 29994, 30294, 30597,
    30903, 31212, 31524, 31839, 32157, 32479, 32804
  };

  static constexpr float skfTempScale_ {333.87F}; ///< A scale factor for converting raw temperature readings to degrees Celsius
  static constexpr float skfTempOffset_ {21.0F}; ///< An offset for converting raw temperature readings to degrees Celsius
  static constexpr EAccelDynamicRange skeAccelDynamicRange_ {EAccelDynamicRange::e8g}; ///< Configured accelerometer dynamic range
  static constexpr EGyroDynamicRange skeGyroDynamicRange_ {EGyroDynamicRange::e500dps}; ///< Configured gyroscope dynamic range
  static constexpr float skfAccelRawToMetersPerSecondSquared_ {(1.0F / static_cast<float>(32768)) * \
                                                               (9.8F * static_cast<float>(2 << static_cast<int>(skeAccelDynamicRange_)))};
  static constexpr float skfGyroRawToRadiansPerSecond_ {(1.0F / static_cast<float>(32768)) * \
                                                        ((3.141592F / 180.0F) * static_cast<float>(250 * (1 << static_cast<int>(skeGyroDynamicRange_))))};

  SCalibParam oCalibParam_;
  uint8_t auImuDataBuffer_[14];

  bool bIsInitialized_ { false };  ///< Sensor status after initialization
  const EIcmIds keSensorId_; ///< ID of the sensor corresponding to the driver instance.
  I2C_HandleTypeDef* const opkI2CHandle_; ///< Pointer to the I2C handle.

  /**
   * @brief Get the Chip ID value barometr part
   * @param uId reference to the ID variable
   * @return True -- success, false -- failure.
   */
  bool getChipIDBaro(uint8_t& urId);

  /**
   * @brief Request soft reset
   * @return True -- success, false -- failure.
   */
  bool softResetBaro();

  /**
   * @brief Request measurement temperature and pressure
   * @return True -- success, false -- failure.
   */
  bool setLowNoiseModeBaro();

  /**
  * @brief Get the Trim Parameters structure
  * @return True -- success, false -- failure.
  */
  bool getOTPBaro();

  /**
   * @brief Get the Temperature And Pressure values
   * @param fTemperature reference to the temperature variable
   * @param fPressure reference to the pressure variable
   * @return True -- success, false -- failure.
   */
  bool getTemperatureAndPressure(float& frTemperature, float& frPressure);

  /**
   * @brief Processing the raw data
   * @param iPressData - Raw pressure data from sensor
   * @param iTempData - Raw temperature data from sensor
   * @param fTemperature reference to the temperature variable
   * @param fPressure reference to the pressure variable
   */
  void processDataBaro(uint32_t iPressData, uint32_t iTempData, float& frPressure, float& frTemperature);

  /**
   * @brief Get the Chip ID value Gyro sensor part
   * @return True -- success, false -- failure.
   */
  bool getImuChipId(uint8_t& urId);

  /**
   * @brief Request soft reset
   * @return True -- success, false -- failure.
   */
  bool softResetImu();

  /**
   * @brief Set dynamic range for accelerometer
   * @return True -- success, false -- failure.
   */
  bool setAccelerometerDynamicRange(EAccelDynamicRange eAccelDynamicRange);

  /**
   * @brief Set dynamic range for gyroscope
   * @return True -- success, false -- failure.
   */
  bool setGyroscopeDynamicRange(EGyroDynamicRange eGyroDynamicRange);

  /**
   * @brief Enable axes
   * @return True -- success, false -- failure.
   */
  bool enableAllImuAxes();

  /**
   * @brief Set Imu sample rate
   * @return True -- success, false -- failure.
   * */
  bool setImuSampleRate();

  /**
   * @brief Set Imu Filter config
   * @return True -- success, false -- failure.
   * */
  bool setImuFilter();

  /**
   * @brief Perform a self test procedure for an IMU.
   * @return True -- self test successful and passed, false -- otherwise.
   * */
  bool imuSelfTest();

  /**
   *@brief Enable bypass
   *@return True -- success, false -- failure.
   */
  bool enableBypass();

  /**
   * @brief Calculate CRC-8 for Barometer
   * @param upData - pointer on data
   * @param uLen - length of data
   * @return CRC
   * */
  uint8_t crc8(uint8_t* upData, uint8_t uLen);

  /**
   * \brief Collect self-test statistics.
   * \param bWithSelfTestFlag Run test with self-test enabled flag
   * \param ipGyroResult Test statistics of a gyroscope
   * \param ipAccelResult Test statistics of an accelerometer
   * \return True -- success. false -- failure
   */
  bool collectTestStatistics(bool bWithSelfTestFlag, int* ipGyroResult, int* ipAccelResult);

  /**
  *  @brief check accel self test
  *  @param[in] ipMeanNormalTestValues average value of normal test.
  *  @param[in] ipMeanSelfTestValues   average value of self test
  *  @return True -- success, false -- failure.
  */
  bool checkAccelSelfTest(int* ipMeanNormalTestValues, int* ipMeanSelfTestValues);

  /**
  *  @brief check gyro self test
  *  @param[in] ipMeanNormalTestValues average value of normal test.
  *  @param[in] ipMeanSelfTestValues   average value of self test
  *  @return True -- success, false -- failure.
  */
  bool checkGyroSelfTest(int* ipMeanNormalTestValues, int* ipMeanSelfTestValues);

  /**
   * Write via I2C.
   * \param uIicAddress Device address
   * \param upData Pointer do data buffer to be written
   * \param uLen Number of bytes to write
   * \return True -- success, false -- failure.
   */
  bool i2cWrite(uint8_t uIicAddress, uint8_t* upData, uint8_t uLen);

  /**
   * Read via I2C.
   * \param uIicAddress Device address to read from.
   * \param upData A pointer to output data buffer.
   * \param uLen Number of bytes to read.
   * \return True -- success, false -- failure.
   */
  bool i2cRead(uint8_t uIicAddress, uint8_t* upData, uint8_t uLen);

  /**
   * Read via I2C. The function uses increased timeout value.
   * \param uIicAddress Device address to read from.
   * \param upData A pointer to output data buffer.
   * \param uLen Number of bytes to read.
   * \return True -- success, false -- failure.
   */
  bool i2cReadOtp(uint8_t uIicAddress, uint8_t* upData, uint8_t uLen);

 /**
  * Read data from registers of the IMU starting from the specified address.
  * \param uReg Register address to start the read from
  * \param upData Pointer to the output data buffer
  * \param uLen Number of bytes to read
  * \return True -- success, false -- failure.
  */
  bool readFromImuRegisters(uint8_t uReg, uint8_t* upData, uint8_t uLen);

  /**
   * Write a byte to IMU's register
   * \param uReg Register address
   * \param uData A value to write
   * \return True -- success, false -- failure.
   */
  bool writeImuRegister(uint8_t uReg, uint8_t uData);

  /**
   * \brief Get an I2C handle corresponding to the chip.
   * \param eIcmChipId An ID of the ICM20789 chip.
   * \return nullptr if an ID is invalid, a pointer to a valid bus handle otherwise.
   */
  I2C_HandleTypeDef* getI2CHandle(EIcmIds eIcmChipId);
};

#endif /* C_ICM_20789_DRIVER_H */
