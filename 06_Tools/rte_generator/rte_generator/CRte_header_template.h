/**
 * @file {CLASS_NAME}.h
 * @brief Declaration of the runtime environment class.
 *
 * THIS FILE WAS AUTOMATICALLY GENERATED FROM
 * A TEMPLATE AND A JSON CONFIGURATION FILE
 *
 * @date {DATE_GENERATED}
 */

#ifndef {INCLUSION_GUARD}
#define {INCLUSION_GUARD}

#include "CSoftwareComponentPort.h"
#include "RteTypesGenerated.h"
#include "RteTypesUserDefined.h"
#include "General/HelperSoftwareComponentMacros.h"

{INCLUSION_OF_SOFTWARE_COMPONENT_HEADERS}

/**
 * Runtime environment (RTE) class. The class is a singleton. It collects all the application application
 * software components and ports. RTE shall be created during initialization.
 */
class CRte
{
public:
  FORBID_CLASS_COPY_AND_MOVE(CRte)

{DECLARATION_OF_SOFTWARE_COMPONENTS}

{DECLARATION_OF_PORT_IDS}

  enum class ERunnableIds : uint8_t
  {
    eInvalid = 0U,
    eRunnableBahrsFilterStep1,
    eRunnableBahrsFilterStep2,
    eRunnableBahrsFilterStep3,
    eRunnableBahrsFilterSetImuInput1,
    eRunnableBahrsFilterSetImuInput2,
    eRunnableBahrsFilterSetImuInput3,
    eRunnableBahrsFilterSetPressureInput1,
    eRunnableBahrsFilterSetPressureInput2,
    eRunnableBahrsFilterSetPressureInput3,
    eRunnableMagneticHeadingFilterStep1,
    eRunnableMagneticHeadingFilterStep2,
    eRunnableMagneticHeadingFilterStep3,
    eRunnableMagneticHeadingFilterSetAttitude1,
    eRunnableMagneticHeadingFilterSetAttitude2,
    eRunnableMagneticHeadingFilterSetAttitude3,
    eRunnableImuMonitorRun,
    eRunnableBarometerMonitorRun,
    eRunnableAttitudeMonitorRun,
    eRunnableVerticalChannelMonitorRun
  };
  
#ifdef _MSC_VER
  friend void WriteToRtePort(CRte::EPortIds ePortId, const std::vector<uint8_t>& korBytes);
  friend void handleRunnableCallEvent(const uint8_t* upMessageStartAddress, size_t uFileLength, size_t& urPositionInFile);
  friend void WriteToUserDefinedRtePort(CRte::EPortIds ePortId, const std::vector<uint8_t>& korBytes);
  friend void SetPortsToRecordInReplayTool();
#endif // _MSC_VER

  /**
   * Get a pointer to the RTE instance.
   */
  static CRte& GetInstance();

  /**
   * Initialize the RTE. Creates and initializes all the member objects.
   */
  void Init();

protected:

private:
  /**
   * The default constructor. Called only by the CRte::GetInstance().
   */
  CRte();
  ~CRte() = default;

  //
  // Declaration of ports
  //

{DECLARATION_OF_PORTS}

};

#endif /* {INCLUSION_GUARD} */
