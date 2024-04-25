/**
 * @file CBmp384Driver.h
 * @brief Declaration of the BMP384 Driver software component
 * @author Evgeniy Zamyatin
 * @date 26 April 2022
 */

#ifndef C_BMP_384_DRIVER_H
#define C_BMP_384_DRIVER_H

#include "stdint.h"
#include "General/CSoftwareComponentBase.h"

#define BMP384_REG_CHIP_ID                ((uint8_t)0x00)
#define BMP384_REG_ERR                    ((uint8_t)0x02)
#define BMP384_REG_STATUS                 ((uint8_t)0x03)
#define BMP384_REG_PREES0                 ((uint8_t)0x04)
#define BMP384_REG_PREES1                 ((uint8_t)0x05)
#define BMP384_REG_PREES2                 ((uint8_t)0x06)
#define BMP384_REG_TEMP0                  ((uint8_t)0x07)
#define BMP384_REG_TEMP1                  ((uint8_t)0x08)
#define BMP384_REG_TEMP2                  ((uint8_t)0x09)

#define BMP384_REG_SENSOR_TIME0           ((uint8_t)0x0C)
#define BMP384_REG_SENSOR_TIME1           ((uint8_t)0x0D)
#define BMP384_REG_SENSOR_TIME2           ((uint8_t)0x0E)
#define BMP384_REG_SENSOR_TIME3           ((uint8_t)0x0F)

#define BMP384_REG_EVENT                  ((uint8_t)0x10)
#define BMP384_REG_INT_STATUS             ((uint8_t)0x10)
#define BMP384_REG_PWR_CTRL               ((uint8_t)0x1B)

#define BMP384_REG_OSR                    ((uint8_t)0x1C)
#define BMP384_REG_ODR                    ((uint8_t)0x1D)

#define BMP384_REG_CONFIG                 ((uint8_t)0x1F)

#define BMP384_REG_TRIM                   ((uint8_t)0x31)
#define BMP384_REG_CMD                    ((uint8_t)0x7E)

#define BMP384_SOFTRESET                  ((uint8_t)0xB6)
#define BMP384_DATA_RD                    ((uint8_t)0x04)

#define BMP384_SLEEP_MODE                 ((uint8_t)0x00)
#define BMP384_FORCED_MODE                ((uint8_t)0x10)
#define BMP384_NORMAL_MODE                ((uint8_t)0x30)

#define BMP384_PRESS_EN                   ((uint8_t)0x02)
#define BMP384_TEMP_EN                    ((uint8_t)0x01)

#define READWRITE_CMD                     ((uint8_t)0x80)
#define MULTIPLEBYTE_CMD                  ((uint8_t)0x40)
#define DUMMY_BYTE                        ((uint8_t)0x00)

#define BMP384_OK                         ((uint8_t)0x00)
#define BMP384_ERR_CHIPID                 ((uint8_t)0x01)
#define BMP384_ERR_SOFTRESET              ((uint8_t)0x02)

#define BMP384_DRDY_TEMP                  ((uint8_t)0x40)
#define BMP384_DRDY_PRESS                 ((uint8_t)0x20)
#define BMP384_DRDY_CMD                   ((uint8_t)0x10)

/**
 * @brief The class implements the BMP384 Driver.
 * The driver adjusts the sensor BMP384
 */
class CBmp384Driver : public CSoftwareComponent<CBmp384Driver, 1U>
{
  friend class CSoftwareComponent<CBmp384Driver, 1U>;
  FORBID_CLASS_COPY_AND_MOVE(CBmp384Driver)
  DECLARE_MANDATORY_APIS(CBmp384Driver)

public:
  /**
   * @brief Implements an iteration of cyclic data exchange with the sensor.
   */
  void PollSensor();

private:
  CBmp384Driver() = default;
  ~CBmp384Driver() = default;

  /**
  * @brief An enum of bit fields with redundant
  * sampling in the control and measurement register
  */
  enum class EOversampling
  {
    eSkip      = 0x00,  ///< Oversampling x1
    eX2        = 0x01,  ///< Oversampling x2
    eX4        = 0x02,  ///< Oversampling x3
    eX8        = 0x03,  ///< Oversampling x8
    eX16       = 0x04,  ///< Oversampling x16
    eX32       = 0x05   ///< Oversampling x32
  };

  /**
  * @brief An enum of the bit fields
  * of the infinite Impulse response (IIR) filter in the configuration register
  */
  enum class EIirFilter
  {
    eOff       = 0x00,  ///< Coefficient 0
    e1         = 0x01,  ///< Coefficient 1
    e3         = 0x02,  ///< Coefficient 3
    e7         = 0x03,  ///< Coefficient 7
    e15        = 0x04,  ///< Coefficient 15
    e31        = 0x05,  ///< Coefficient 31
    e63        = 0x06,  ///< Coefficient 63
    e127       = 0x07   ///< Coefficient 127
  };

  /**
   * @brief An enum of the waiting time bit fields in the output data rate register (ODR)
   */
  enum class ETimeStandby
  {
    e5MS       = 0x00,  ///< 5 millisecond - 200 Hz
    e10MS      = 0x01,  ///< 10 millisecond - 100 Hz
    e20MS      = 0x02,  ///< 20 millisecond - 50 Hz
    e40MS      = 0x03,  ///< 40 millisecond - 25 Hz
    e80MS      = 0x04,  ///< 80 millisecond - 12.5 Hz
    e160MS     = 0x05,  ///< 160 millisecond - 6.25 Hz
    e320MS     = 0x06,  ///< 320 millisecond - 3.1 Hz
    e640MS     = 0x07,  ///< 640 millisecond - 1.5 Hz
    e1280MS    = 0x08,  ///< 1280 millisecond - 0.78 Hz
    e2560MS    = 0x09,  ///< 2560 millisecond - 0.39 Hz
    e5120MS    = 0x0A,  ///< 5120 millisecond - 0.2 Hz
    e10240MS   = 0x0B,  ///< 10240 millisecond - 0.1 Hz
    e20480MS   = 0x0C,  ///< 20480 millisecond - 0.05 Hz
    e40960MS   = 0x0D,  ///< 40960 millisecond - 0.02 Hz
    e81920MS   = 0x0E,  ///< 81920 millisecond - 0.01 Hz
    e163840MS  = 0x0F,  ///< 163840 millisecond - 0.006 Hz
    e327680MS  = 0x10,  ///< 327680 millisecond - 0.003 Hz
    e655360MS  = 0x11   ///< 655360 millisecond - 0.0015 Hz
  };

  /**
   * @brief An enum of operated power modes
   *
   */
  enum class EMode
  {
    eSleep,  ///< Sleep mode
    eForce,  ///< Forced mode
    eNormal  ///< Normal mode
  };

  /**
   * @brief Integer pressure compensation parameters read from the NVM of BMP384
   */
#pragma pack(push,1)
  struct SParams
  {
    uint16_t  uT1_;  ///< T1 register
    uint16_t  uT2_;  ///< T2 register
    int8_t    uT3_;  ///< T3 register
    int16_t   uP1_;  ///< P1 register
    int16_t   uP2_;  ///< P2 register
    int8_t    uP3_;  ///< P3 register
    int8_t    uP4_;  ///< P4 register
    uint16_t  uP5_;  ///< P5 register
    uint16_t  uP6_;  ///< P6 register
    int8_t    uP7_;  ///< P7 register
    int8_t    uP8_;  ///< P8 register
    int16_t   uP9_;  ///< P9 register
    int8_t    uP10_; ///< P10 register
    int8_t    uP11_; ///< P11 register
  };
#pragma pack(pop)

  /**
   * @brief Structure of BMP384 compensation trim parameters
   */
  struct SFloatParams
  {
    float  fT1_;  ///< Calibration coefficient T1
    float  fT2_;  ///< Calibration coefficient T2
    float  fT3_;  ///< Calibration coefficient T3
    float  fP1_;  ///< Calibration coefficient P1
    float  fP2_;  ///< Calibration coefficient P2
    float  fP3_;  ///< Calibration coefficient P3
    float  fP4_;  ///< Calibration coefficient P4
    float  fP5_;  ///< Calibration coefficient P5
    float  fP6_;  ///< Calibration coefficient P6
    float  fP7_;  ///< Calibration coefficient P7
    float  fP8_;  ///< Calibration coefficient P8
    float  fP9_;  ///< Calibration coefficient P9
    float  fP10_;  ///< Calibration coefficient P10
    float  fP11_;  ///< Calibration coefficient P11
  };

  /**
   * @brief Get the Chip ID value
   * @param urChipId Reference to the chip ID to be filled
   * @return True -- success, false -- failure.
   */
  bool getChipID(uint8_t& urChipId);

  /**
   * @brief Get the Trim Parameters structure
   * @return True -- success, false -- failure.
   */
  bool getTrimParam();

  /**
   * @brief Get the Temperature And Pressure values
   * @param fTemperature reference to the temperature variable
   * @param fPressure reference to the pressure variable
   * @return True -- success, false -- failure.
   */
  bool getTemperatureAndPressure(float& frTemperature, float& frPressure);

  /**
   * @brief Request soft reset
   * @return True -- success, false -- failure.
   */
  bool setSoftReset();

  /**
   * @brief Set the Power Mode
   * @param eMode power mode
   * @return True -- success, false -- failure.
   */
  bool setPowerMode(EMode eMode);

  /**
   * @brief Set the Filter value
   * @param eFilterType Filter value
   */
  bool setFilter(EIirFilter eFilterType);

  /**
   * @brief Set the Time Standby
   * @param eTimeStandby  Time Standby
   */
  bool setTimeStandby(ETimeStandby eTimeStandby);

  /**
   * @brief Set the Pressure Oversampling value
   * @param ePresOversampling  Pressure Oversampling value
   */
  bool setPresOversampling(EOversampling ePresOversampling);

  /**
   * @brief Set the Temperature oversampling value
   * @param eTempOversampling Temperature oversampling value
   */
  bool setTempOversampling(EOversampling eTempOversampling);

  /**
   * @brief Set the Oversampling Register value
   * @param ePresOversampling Pressure over sampling value
   * @param eTempOversamping Temperature over sampling value
   */
  bool setOversamplingRegister(EOversampling ePresOversampling, EOversampling eTempOversamping);

  /**
   * @brief Is the data ready to be read
   * @return True -- ready, false -- not ready.
   */
  bool isReadyRead();

  /**
   * @brief Read from address via SPI
   * @param upBuffer Buffer for record
   * @param uReadAddr Register address
   * @param uNumByteToRead Number of bytes to read
   * @return True -- success, false -- failure.
   */
  bool readFromAddress(uint8_t *upBuffer, uint8_t uReadAddr, uint16_t uNumByteToRead);

  /**
   * @brief Write to address via SPI
   * @param uWriteAddr Register address
   * @param uVal The resulting value
   * @return True -- success, false -- failure.
   */
  bool writeToAddress(uint8_t uWriteAddr, uint8_t uVal);

  /**
   * @brief Compensate temperature value
   * @param fUncompensatedTemperature Temperature value
   * @return Compensated temperature
   */
  float compensateTemperature(float fUncompensatedTemperature);

  /**
   * @brief Compensate pressure value
   * @param fUncompensatedPressure Pressure value
   * @param fCompensatedTemperature Temperature value (after compensation)
   * @return Compensated pressure
   */
  float compensatePressure(float fUncompensatedPressure, float fCompensatedTemperature);

  /**
   * @brief Set the chip select pin is On
   */
  void setCsOn();

  /**
   * @brief Set the chip select pin is Off
   */
  void setCsOff();

  SFloatParams oFloatParams_;  ///< Float point compensation trim parameters
  SParams oParams_;  ///< Compensation trim parameters (coefficients)
  bool bIsInitialized_{ false };  ///< Sensor status after initialization
};

#endif /* C_BMP_384_DRIVER_H */
