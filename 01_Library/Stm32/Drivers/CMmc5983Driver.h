/**
 * @file CMmc5983Driver.h
 * @brief Declaration of the MMC5983 Driver software component
 * @author Evgeniy Zamyatin
 * @date 31 May 2022
 */

#ifndef C_MMC_5983_DRIVER_H
#define C_MMC_5983_DRIVER_H

#include "stdint.h"
#include "General/CSoftwareComponentBase.h"

// Register MAP
#define MMC5983_REG_X_OUT0                  ((uint8_t)0x00)
#define MMC5983_REG_X_OUT1                  ((uint8_t)0x01)
#define MMC5983_REG_Y_OUT0                  ((uint8_t)0x02)
#define MMC5983_REG_Y_OUT1                  ((uint8_t)0x03)
#define MMC5983_REG_Z_OUT0                  ((uint8_t)0x04)
#define MMC5983_REG_Z_OUT1                  ((uint8_t)0x05)
#define MMC5983_REG_XYZ_OUT2                ((uint8_t)0x06)

#define MMC5983_REG_T_OUT0                  ((uint8_t)0x07)

#define MMC5983_REG_STATUS                  ((uint8_t)0x08)
#define MMC5983_REG_CONTROL0                ((uint8_t)0x09)
#define MMC5983_REG_CONTROL1                ((uint8_t)0x0A)
#define MMC5983_REG_CONTROL2                ((uint8_t)0x0B)
#define MMC5983_REG_CONTROL3                ((uint8_t)0x0C)
#define MMC5983_REG_PRODUCT_ID              ((uint8_t)0x2F)

//Status macros
#define MMC5983_OK                          ((uint8_t)0x00)
#define MMC5983_ERR_CHIPID                  ((uint8_t)0x01)
#define MMC5983_ERR_SELFTEST                ((uint8_t)0x02)

#define MMC5983_READWRITE                   ((uint8_t)0x80)
#define DUMMY_BYTE                          ((uint8_t)0x00)

#define MMC5983_SW_RST                      ((uint8_t)0x80)
#define MMC5983_AUTO_SR_EN                  ((uint8_t)0x20)
#define MMC5983_CM_EN                       ((uint8_t)0x08)

#define MMC5983_18BIT_OFFSET                     (131072.0F)
#define MMC5983_18BIT_SENSITIVITY                 (16384.0F)

#define MMC5983_M_DONE                      ((uint8_t)0x01)
#define MMC5983_T_DONE                      ((uint8_t)0x02)

#define MMC5983_CMD_TMT                     ((uint8_t)0x02)
#define MMC5983_T_SENSITIVITY                        (0.8F)
#define MMC5983_T_ZERO                             (-75.0F)

#define MMC5983_BIT_RESET                   ((uint8_t)0x10)
#define MMC5983_BIT_SET                     ((uint8_t)0x08)

#define MMC5983_XYZ_0_SHIFT                             10
#define MMC5983_XYZ_1_SHIFT                              2

#define MMC5983_X2_MASK                               0xC0
#define MMC5983_Y2_MASK                               0x30
#define MMC5983_Z2_MASK                               0x0C

class CMmc5983Driver : public CSoftwareComponent<CMmc5983Driver, 1U>
{
  friend class CSoftwareComponent<CMmc5983Driver, 1U>;
  FORBID_CLASS_COPY_AND_MOVE(CMmc5983Driver)
  DECLARE_MANDATORY_APIS(CMmc5983Driver)

public:
  /**
  * @brief Implements an iteration of cyclic data exchange with the sensor.
  */
  void PollSensor();

private:
  CMmc5983Driver() = default;
  ~CMmc5983Driver() = default;

  /**
   * @brief An enum of measurement mode
   */
  enum class EMeasurementMode
  {
    eSkip   = 0x00, ///< No continuous measurement mode
    e1Hz    = 0x01, ///< Frequency 1 Hz
    e10Hz   = 0x02, ///< Frequency 10 Hz
    e20Hz   = 0x03, ///< Frequency 20 Hz
    e50Hz   = 0x04, ///< Frequency 50 Hz
    e100Hz  = 0x05, ///< Frequency 100 Hz
    e200Hz  = 0x06, ///< Frequency 200 Hz
    e1000Hz = 0x07, ///< Frequency 1000 Hz
  };

  /**
   * @brief An enum of bandwidth filter
   */
  enum class EBandwidth
  {
    e100Hz  = 0x00, ///< Filter bandwidth 100 Hz
    e200Hz  = 0x01, ///< Filter bandwidth 200 Hz
    e400Hz  = 0x02, ///< Filter bandwidth 400 Hz
    e800Hz  = 0x03, ///< Filter bandwidth 800 Hz
  };

  /**
   * @brief Full sensor reset
   */
  void setSoftReset();

  /**
   * @brief Request SET function inductor coil integrated into chip
   * See sensor datasheet for more information
   */
  void setBitSet();

  /**
   * @brief Request RESET function inductor coil integrated into chip
   * See sensor datasheet for more information
   */
  void setBitReset();

  /**
   * @brief Set the Measurement Mode value and enable continuous mode
   * @param eMeasurementMode Mode value
   */
  void setContinuousModeFrequency(EMeasurementMode eMeasurementMode);

  /**
   * @brief Set the Filter Bandwidth value
   * @param eBandwidth filter value
   */
  void setFilterBandwidth(EBandwidth eBandwidth);

  /**
   * @brief Enable auto Set/Reset mode
   */
  void setAutoSetReset();

  /**
   * @brief Get measured magnetic field vector
   * @param frValueX reference to the value X variable
   * @param frValueY reference to the value Y variable
   * @param frValueZ reference to the value Z variable
   */
  void getMeasurementXYZ(float& frValueX, float& frValueY, float& frValueZ);

  /**
   * @brief Get the Temperature value
   * @return temperature value
   * @Comment: Temperature and magnetic cannot be measured simultaneously
   */
  float getTemperature();

  /**
   * @brief Get the Product ID value
   * @return ProductID value
   */
  uint8_t getProductID();

  /**
   * @brief Is the data ready to be read
   * @return uint8_t Bit mask
   */
  uint8_t isReadyRead();

  /**
   * @brief Read from address via SPI
   * @param upBuffer Buffer for record
   * @param uReadAddr Register address
   * @param uNumByteToRead Number of bytes to read
   */
  void readFromAddress(uint8_t *upBuffer, uint8_t uReadAddr, uint16_t uNumByteToRead);

  /**
   * @brief Write to address via SPI
   * @param uWriteAddr Register address
   * @param uVal The resulting value
   */
  void writeToAddress(uint8_t uWriteAddr, uint8_t uVal);

  /**
   * @brief Set the chip select pin is On
   */
  void setCsOn();

  /**
   * @brief Set the chip select pin is Off
   */
  void setCsOff();
  /**
   * @brief Read/Write via SPI
   * @param uByte Register add or dummy byte
   * @return The read value via SPI
   */
  uint8_t spiWriteRead(uint8_t uByte);

  uint8_t uSensorStatus_ = 0;  ///< Sensor status after initialization
};

#endif /* C_MMC_5983_DRIVER_H */
