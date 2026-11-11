/// @file CMagneticHeadingFilterSwc.h
/// @brief Declaration of the magnetic heading filter software component.
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef C_MAGNETIC_HEADING_FILTER_SWC_H
#define C_MAGNETIC_HEADING_FILTER_SWC_H

#include "General/CSoftwareComponentBase.h"
#include "RteTypesGenerated.h"
#include "MagneticHeadingFilterApi.h"

#ifndef _MSC_VER
#include "cmsis_os.h"
#endif // _MSC_VER

namespace NMagneticHeadingFilterApi
{
#ifdef BAHRS_HW_V3
  using namespace ::NMagneticHeadingFilterApi::NBahrsV3;
#elif defined(BAHRS_HW_V2)
  using namespace ::NMagneticHeadingFilterApi::NBahrsV2;
#else
#error "BAHRS hardware version is not defined"
#endif
} // namespace NMagneticHeadingFilterApi

/// @brief The class implements a magnetic heading filter SWC. There are three independent instances,
/// each fed by its own magnetometer and by the common safe vehicle attitude.
class CMagneticHeadingFilterSwc : public CSoftwareComponent<CMagneticHeadingFilterSwc, 3U>
{
  friend class CSoftwareComponent<CMagneticHeadingFilterSwc, 3U>;
  FORBID_CLASS_COPY_AND_MOVE(CMagneticHeadingFilterSwc)
  DECLARE_MANDATORY_APIS(CMagneticHeadingFilterSwc)

public:
  /// @brief Magnetometer sensor identifier for magnetic heading filter instances.
  enum class ESensorId : uint8_t
  {
    eMagnetometer1 = 0U,
    eMagnetometer2 = 1U,
    eMagnetometer3 = 2U
  };

  /// @brief A wrapper around MagneticHeadingFilterApi::MagneticHeadingFilterSetInput(). Has mutex protection.
  void SetAttitudeInput();

  /// @brief A wrapper around MagneticHeadingFilterApi::MagneticHeadingFilterStep().
  /// @param uTimestampUs Time of the API call in microseconds.
  void Step(uint64_t uTimestampUs);

protected:

private:
  /// @brief Static configuration of a single filter instance.
  struct SFilterConfiguration
  {
    ESensorId eSensorId{ ESensorId::eMagnetometer1 }; ///< Magnetometer assigned to the instance
    NFusionLibCommon::ESensorId eFusionLibSensorId{ NFusionLibCommon::ESensorId::eUnknown }; ///< ID of the magnetometer model, as seen by the fusion library
  };

  /// @brief Create a filter for a specific magnetometer. The constructor creates a mutex for shared filter data.
  /// @param oFilterConfig Configuration of the filter instance.
  explicit CMagneticHeadingFilterSwc(SFilterConfiguration oFilterConfig);

  ~CMagneticHeadingFilterSwc() = default;

  /// @brief Convert sensor ID to a zero-based index. The index is also the instance index of the filter in the fusion library.
  /// @param eSensorId Magnetometer sensor ID.
  /// @return Zero-based index.
  static constexpr uint32_t toIndex(ESensorId eSensorId)
  {
    return static_cast<uint32_t>(eSensorId);
  }

  /// @brief Read vehicle-frame magnetometer data from the input port.
  /// Selects the input port based on the magnetometer assigned to the instance.
  /// @param orMagData Reference to the magnetometer data object to be populated.
  /// @return True if the read succeeded.
  bool readMagnetometerDataPort(SMagneticMeasurement& orMagData) const;

  /// @brief Write filter output to the RTE port selected by the magnetometer assigned to the instance.
  /// @param korOutput Magnetic heading data to write.
  void writeOutputToPort(const SMagneticHeading& korOutput) const;

  /// @brief Get the static configuration of the filter instance for a magnetometer. Specialized per hardware variant.
  /// @tparam eSensorId Magnetometer sensor ID.
  /// @return Filter configuration.
  template<ESensorId eSensorId> static constexpr SFilterConfiguration getFilterConfiguration();

  const ESensorId keSensorId_; ///< Magnetometer assigned to this instance
  const NFusionLibCommon::ESensorId keFusionLibSensorId_; ///< ID of the magnetometer model, as seen by the fusion library

#ifndef _MSC_VER
  /// The typedef copied from a CMSIS_OS header.
  typedef StaticSemaphore_t osStaticMutexDef_t;

  osMutexId_t pMutexHandle_; ///< Mutex handle.
  osStaticMutexDef_t sMutexControlBlock_; ///< Memory reserved for the mutex control block.
  const osMutexAttr_t sMutexAttributes_ { NULL, 0, &sMutexControlBlock_, sizeof(sMutexControlBlock_) }; ///< A structure with mutex attributes required for creation.
#endif // _MSC_VER
};

#ifdef BAHRS_HW_V3

template<>
constexpr CMagneticHeadingFilterSwc::SFilterConfiguration CMagneticHeadingFilterSwc::getFilterConfiguration<CMagneticHeadingFilterSwc::ESensorId::eMagnetometer1>()
{
  return { ESensorId::eMagnetometer1, NFusionLibCommon::ESensorId::eMmc5983 };
}

template<>
constexpr CMagneticHeadingFilterSwc::SFilterConfiguration CMagneticHeadingFilterSwc::getFilterConfiguration<CMagneticHeadingFilterSwc::ESensorId::eMagnetometer2>()
{
  return { ESensorId::eMagnetometer2, NFusionLibCommon::ESensorId::eBmm350 };
}

template<>
constexpr CMagneticHeadingFilterSwc::SFilterConfiguration CMagneticHeadingFilterSwc::getFilterConfiguration<CMagneticHeadingFilterSwc::ESensorId::eMagnetometer3>()
{
  return { ESensorId::eMagnetometer3, NFusionLibCommon::ESensorId::eLis3Mdl };
}

#elif defined(BAHRS_HW_V2)

template<>
constexpr CMagneticHeadingFilterSwc::SFilterConfiguration CMagneticHeadingFilterSwc::getFilterConfiguration<CMagneticHeadingFilterSwc::ESensorId::eMagnetometer1>()
{
  return { ESensorId::eMagnetometer1, NFusionLibCommon::ESensorId::eMmc5983 };
}

template<>
constexpr CMagneticHeadingFilterSwc::SFilterConfiguration CMagneticHeadingFilterSwc::getFilterConfiguration<CMagneticHeadingFilterSwc::ESensorId::eMagnetometer2>()
{
  return { ESensorId::eMagnetometer2, NFusionLibCommon::ESensorId::eBmm150X1 };
}

template<>
constexpr CMagneticHeadingFilterSwc::SFilterConfiguration CMagneticHeadingFilterSwc::getFilterConfiguration<CMagneticHeadingFilterSwc::ESensorId::eMagnetometer3>()
{
  return { ESensorId::eMagnetometer3, NFusionLibCommon::ESensorId::eBmm150X2 };
}

#else
#error "BAHRS hardware version is not defined"
#endif // BAHRS_HW_V3

#endif // C_MAGNETIC_HEADING_FILTER_SWC_H
