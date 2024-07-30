/**
 * @file CScha63TDriver.h
 * @brief Declaration of the SCHA63T driver software component.
 * @author Fedor Baklanov
 * @date 07 Mai 2022
 */

#ifndef C_SCHA63T_DRIVER_H
#define C_SCHA63T_DRIVER_H

#include "stm32f4xx.h"
#include "General/CSoftwareComponentBase.h"
#include "Scha63TDriverTypes.h"

//
// Pre-calculated SPI frames for various operations
//

#define SPI_FRAME_READ_GYRO_X          0x040000F7
#define SPI_FRAME_READ_GYRO_Y          0x0C0000FB
#define SPI_FRAME_READ_GYRO_Z          0x040000F7
#define SPI_FRAME_READ_ACC_X           0x100000E9
#define SPI_FRAME_READ_ACC_Y           0x140000EF
#define SPI_FRAME_READ_ACC_Z           0x180000E5
#define SPI_FRAME_READ_TEMP            0x1C0000E3

#define SPI_FRAME_READ_SUMMARY_STATUS  0x380000D5
#define SPI_FRAME_READ_RATE_STATUS_1   0x40000091
#define SPI_FRAME_READ_RATE_STATUS_2   0x44000097
#define SPI_FRAME_READ_COMMON_STATUS_1 0x50000089
#define SPI_FRAME_READ_COMMON_STATUS_2 0x5400008F
#define SPI_FRAME_READ_ACC_STATUS_1    0x4800009D

#define SPI_FRAME_WRITE_RESET          0xE000017C
#define SPI_FRAME_WRITE_REG_BANK_0     0xFC000073 ///< Select register bank 0
#define SPI_FRAME_READ_TRC_0           0x740000BF ///< Traceability 0 register 1Dh
#define SPI_FRAME_READ_TRC_1           0x780000B5 ///< Traceability 1 register 1Eh
#define SPI_FRAME_READ_TRC_2           0x700000B9 ///< Traceability 2 Register 1Ch

#define SPI_FRAME_WRITE_OP_MODE_NORMAL    0xE4000067 ///< Set normal operating mode (register 19h)

#define SPI_FRAME_WRITE_FILTER_13HZ_RATE  0xD8000045 ///< Set 13 Hz filter for rate (register 16h)
#define SPI_FRAME_WRITE_FILTER_20HZ_RATE  0xD80909A6 ///< Set 20 Hz filter for rate (register 16h)
#define SPI_FRAME_WRITE_FILTER_46HZ_RATE  0xD812129E ///< Set 46 Hz filter for rate (register 16h)
#define SPI_FRAME_WRITE_FILTER_200HZ_RATE 0xD81B1B7D ///< Set 200 Hz filter for rate (register 16h)
#define SPI_FRAME_WRITE_FILTER_300HZ_RATE 0xD82424EE ///< Set 300 Hz filter for rate (register 16h)

#define SPI_FRAME_WRITE_FILTER_13HZ_ACC   0xE800006D ///< Set 13 Hz filter for acceleration (register 1Ah)
#define SPI_FRAME_WRITE_FILTER_20HZ_ACC   0xE800006D ///< Set 20 Hz filter for acceleration (register 1Ah)
#define SPI_FRAME_WRITE_FILTER_46HZ_ACC   0xE8022248 ///< Set 46 Hz filter for acceleration (register 1Ah)
#define SPI_FRAME_WRITE_FILTER_200HZ_ACC  0xE80333D4 ///< Set 200 Hz filter for acceleration (register 1Ah)
#define SPI_FRAME_WRITE_FILTER_300HZ_ACC  0xE8044427 ///< Set 300 Hz filter for acceleration (register 1Ah)
#define SPI_FRAME_WRITE_EOI_BIT           0xE000025B

//
// Frames needed for test mode activation. Mode register address: 19h
//

#define SPI_FRAME_READ_MODE               0x640000A7
#define SPI_FRAME_WRITE_MODE_ASM_010      0xE40010AA ///< Unlock_ASM[2:0] = 010
#define SPI_FRAME_WRITE_MODE_ASM_001      0xE400088F ///< Unlock_ASM[2:0] = 001
#define SPI_FRAME_WRITE_MODE_ASM_100      0xE40020E0 ///< Unlock_ASM[2:0] = 100

#define SPI_FRAME_WRITE_USER_DATA_17H     0xDCAAAA4E
#define SPI_FRAME_READ_USER_DATA_17H      0x5CAAAA8E

/**
 * @brief This is class implements a driver for the Murata SCHA63T inertial sensor.
*/
class CScha63TDriver : public CSoftwareComponent<CScha63TDriver, 1U>
{
  friend class CSoftwareComponent<CScha63TDriver, 1U>;
  FORBID_CLASS_COPY_AND_MOVE(CScha63TDriver)
  DECLARE_MANDATORY_APIS(CScha63TDriver)

public:
  /**
   * @brief Poll the sensor. Puts batches of 30 measurements periodically into the queue of the FreeRTOS task.
   * The function is called from the 6kHz interrupt triggered by the TIM7.
  */
  void PollSensor();

  /**
   * @brief Converts batches of 30 raw measurements to IMU measurement, applies factory calibration and writes the result to the RTE port.
   * The routine shall be called from FreeRTOS tasks.
  */
  void ConvertRawDataset();

protected:

private:
  CScha63TDriver() = default;
  ~CScha63TDriver() = default;

  /**
   * @brief A struct to store compensation parameters stored in the sensor's NVM.
  */
  struct SCompensationParameters
  {
    float fCxx_ { 1.0F };
    float fCxy_ { 0.0F };
    float fCxz_ { 0.0F };
    float fCyx_ { 0.0F };
    float fCyy_ { 1.0F };
    float fCyz_ { 0.0F };
    float fCzx_ { 0.0F };
    float fCzy_ { 0.0F };
    float fCzz_ { 1.0F };
    float fBxx_ { 1.0F };
    float fBxy_ { 0.0F };
    float fBxz_ { 0.0F };
    float fByx_ { 0.0F };
    float fByy_ { 1.0F };
    float fByz_ { 0.0F };
    float fBzx_ { 0.0F };
    float fBzy_ { 0.0F };
    float fBzz_ { 1.0F };
  };

  static constexpr float skfGravity_ { 9.8F }; ///< Gravity constant, [m/s^2]
  static constexpr float skfDegreesToRadians_ { 0.017453292F }; ///< Scale factor to convert degrees to radians, [rad/deg]
  static constexpr float skfAccelerometerSensitivity_ { 4905.0F }; ///< Accelerometer sensitivity, [LSB/g]
  static constexpr float skfGyroscopeSensitivity_ { 80.0F }; ///< Gyroscope sensitivity, [LSB/deg/s]
  static constexpr int skiMaxAttemptsToConfigure_ { 2 }; ///< Number of attempts to configure the sensor

  /**
   * @brief Enumeration of measurement frames for SPI communication.
  */
  enum EMeasurementFrames
  {
    eSpecificForceX = 0,
    eSpecificForceY,
    eSpecificForceZ,
    eAngularRateX,
    eAngularRateY,
    eAngularRateZ,
    eTemperatureUno,
    eTemperatureDue,
    eFrameCount
  };

  /**
   * @brief Poll the status of the UNO ASIC.
   * @return True if the UNO ASIC operates normally, false otherwise.
  */
  bool pollUnoStatus();

  /**
   * @brief Poll the status of the DUE ASIC.
   * @return True if the DUE ASIC operates normally, false otherwise.
  */
  bool pollDueStatus();

  /**
   * @brief Check for RS error bits in the frame. 
   * @param uFrame 32-bit frame received from the ASIC.
   * @return True -- there are errors, false -- no errors.
  */
  inline static bool checkRsError(uint32_t uFrame);

  /**
   * @brief Check for RS errors in an array of frames received from the ASIC:
   * @param upFrames A pointer to the first array element.
   * @param uFrameCount Number of frames ín the array.
   * @return True if there are errors, false -- no errors.
  */
  static bool checkRsErrorInFrames(uint32_t* upFrames, unsigned uFrameCount);

  /**
   * @brief Read/write a 32-bit frame to/from the UNO ASIC.
   * @param uDataOut Data to be send on SPI (for example read address).
   * @return Response of the sensor to the previous operation.
  */
  uint32_t spiWriteReadUno(uint32_t uDataOut);

  /**
   * @brief Read/write a 32-bit frame to/from the DUE ASIC.
   * @param uDataOut Data to be send on SPI (for example read address).
   * @return Response of the sensor to the previous operation.
  */
  uint32_t spiWriteReadDue(uint32_t uDataOut);

  /**
   * @brief SPI communication with the sensor.
   * @param uDataOut Data to be sent.
   * @param opGpioX CSB GPIO group.
   * @param uGpioPin  CSB GPIO pin.
   * @return Response of the sensor.
  */
  uint32_t spiWriteRead(uint32_t uDataOut, GPIO_TypeDef* opGpioX, uint16_t uGpioPin);

  /**
   * @brief Get a 16-bit integer from the frame received from the ASIC.
   * @param uFrame A 32-bit frame received from the ASIC:
   * @return 16-bit integer.
  */
  inline static int16_t frameToInt16(uint32_t uFrame);

  /**
   * @brief Get a 16-bit unsigned integer from the frame received from the ASIC.
   * @param uFrame A 32-bit frame received from the ASIC:
   * @return A 16-bit unsigned integer.
  */
  inline static uint16_t frameToUint16(uint32_t uFrame);

  /**
   * @brief Get a 8-bit integer from the frame received from the ASIC using the lower bits.
   * @param uFrame A 32-bit frame received from the ASIC.
   * @return Desired 8-bit integer.
  */
  inline static int8_t lowerInt8FromFrame(uint32_t uFrame);

  /**
   * @brief Get a 8-bit integer from the frame received from the ASIC using the upper bits.
   * @param uFrame A 32-bit frame received from the ASIC.
   * @return Desired 8-bit integer.
  */
  inline static int8_t upperInt8FromFrame(uint32_t uFrame);

  bool bIsInitialized_ { false }; ///< Sensor initialization status: true -- initialized, false otherwise.
  char acSerialNumber_[14] { "" }; ///< ID of the sensor read from the sensor's NVM.
  SCompensationParameters oCompensationParameters_; ///< Compensation parameters read from the sensor's NVM.
  SScha63TDataset oLatestDataset_; ///< The latest 30 measurements that are transfered periodically to the queue of the dedicated FreeRTOS task.
  SScha63TDataset oOutputDataset_;
  bool bErrorFlags_ { false }; ///< The flag indicating errors during ASIC operation.
  SScha63TStatusUno oStatusUno_; ///< The latest content of the UNO status registers. Valid only when bErrorFlags_ is true.
  SScha63TStatusDue oStatusDue_; ///< The latest content of the DUE status registers. Valid only when bErrorFlags_ is true.
  bool bDatasetAvailable_ { false };

};

#endif /* C_SCHA63T_DRIVER_H */
