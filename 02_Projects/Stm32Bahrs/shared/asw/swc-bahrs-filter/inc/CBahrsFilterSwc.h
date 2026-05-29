/**
 * @file CBahrsFilterSwc.h
 * @brief Implementation of the BAHRS filter software component
 * @author Fedor Baklanov
 * @date 08 June 2022
 */

#ifndef C_BAHRS_FILTER_SWC_H
#define C_BAHRS_FILTER_SWC_H

#include "BahrsFilterApi.h"
#include "CRte.h"

#ifndef _MSC_VER
#include "cmsis_os.h"
#endif /* _MSC_VER */

class CBahrsFilterSwc : public CSoftwareComponent<CBahrsFilterSwc, 3U>
{
  friend class CSoftwareComponent<CBahrsFilterSwc, 3U>;
  FORBID_CLASS_COPY_AND_MOVE(CBahrsFilterSwc)
  DECLARE_MANDATORY_APIS(CBahrsFilterSwc)

public:
  /**
   * @brief A wrapper around CBahrs::SetInput(). Has mutex protection.
  */
  void SetImuInput();

  /**
   * @brief A wrapper around CBahrs::SetInput(). Has mutex protection.
  */
  void SetPressureInput();

  /**
   * @brief A wrapper arounf CBahrs::Step().
   * @param uTimestampUs Time of the API call in microseconds.
  */
  void Step(uint64_t uTimestampUs);

protected:

private:
  /**
   * @brief The constructor creates a mutex for shared BAHRS filter data.
   * @param uInstanceIndex Instance index of the software component.
  */
  CBahrsFilterSwc(uint32_t uInstanceIndex);
  ~CBahrsFilterSwc() = default;

  /**
   * @brief Read IMU data from input port.
   * Selects input port based on SWC instance index.
   * @param orImuData Reference to IMU data object to be populated.
   * @param erSensorId ID of the sensor used as input source.
   */
  bool readImuDataPort(SImuMeasurement& orImuData, NFusionLibCommon::ESensorId& erSensorId) const;

  /**
   * @brief Read pressure data from input port.
   * Selects input port based on SWC instance index.
   * @param orPressureData Reference to pressure data object to be populated.
   * @param erSensorId ID of the sensor used as input source.
   */
  bool readPressureDataPort(SBarometerMeasurement& orPressureData, NFusionLibCommon::ESensorId& erSensorId) const;

  /**
   * @brief Write BAHRS filter output to RTE port.
   * Selects output port based on SWC instance index.
   * @param korFilterOutput Filter output data to write.
   */
  void writeOutputToPort(const CBahrsFilterOutput& korFilterOutput) const;

  const uint32_t kuInstanceIndex_; ///< Index of the component instance

#ifdef BAHRS_HW_V3
  static constexpr NFusionLibCommon::ESensorId skeImuId1_{NFusionLibCommon::ESensorId::eScha63T};
  static constexpr NFusionLibCommon::ESensorId skeImuId2_{NFusionLibCommon::ESensorId::eBmi270};
  static constexpr NFusionLibCommon::ESensorId skeImuId3_{NFusionLibCommon::ESensorId::eAsm330};
  static constexpr NFusionLibCommon::ESensorId skeBaroId1_{NFusionLibCommon::ESensorId::eIcp20100};
  static constexpr NFusionLibCommon::ESensorId skeBaroId2_{NFusionLibCommon::ESensorId::eBmp384};
  static constexpr NFusionLibCommon::ESensorId skeBaroId3_{NFusionLibCommon::ESensorId::eLps22};
#elif defined(BAHRS_HW_V2)
  static constexpr NFusionLibCommon::ESensorId skeImuId1_{NFusionLibCommon::ESensorId::eScha63T};
  static constexpr NFusionLibCommon::ESensorId skeImuId2_{NFusionLibCommon::ESensorId::eIcm20789Imu1};
  static constexpr NFusionLibCommon::ESensorId skeImuId3_{NFusionLibCommon::ESensorId::eIcm20789Imu2};
  static constexpr NFusionLibCommon::ESensorId skeBaroId1_{NFusionLibCommon::ESensorId::eBmp384};
  static constexpr NFusionLibCommon::ESensorId skeBaroId2_{NFusionLibCommon::ESensorId::eIcm20789Baro1};
  static constexpr NFusionLibCommon::ESensorId skeBaroId3_{NFusionLibCommon::ESensorId::eIcm20789Baro2};
#else
#error "BAHRS hardware version is not defined"
#endif

  /**
   * The typedef copied from a CMSIS_OS header.
   */
  typedef StaticSemaphore_t osStaticMutexDef_t;

  osMutexId_t pMutexHandle_; ///< Mutex handle.
  osStaticMutexDef_t sMutexControlBlock_; ///< Memory reserved for the mutex control block.
  const osMutexAttr_t sMutexAttributes_ { NULL, 0, &sMutexControlBlock_, sizeof(sMutexControlBlock_) }; ///< A structure with mutex attributes required for creation.

};

#endif /* C_BAHRS_FILTER_SWC_H */
