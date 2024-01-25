/**
 * @file COutputTransformer.h
 * @brief Declaration of the Output Transformer SWC class.
 * @author Fedor Baklanov
 * @date 18 February 2023
*/

#ifndef C_OUTPUT_TRANSFORMER_H
#define C_OUTPUT_TRANSFORMER_H

#include "Eigen/Dense"
#include "General/CSoftwareComponentBase.h"

/**
 * @brief The class implements a SW component that converts internal signals to customer-specific signals.
*/
class COutputTransformer : public CSoftwareComponent<COutputTransformer, 1U>
{
  friend class CSoftwareComponent<COutputTransformer, 1U>;
  FORBID_CLASS_COPY_AND_MOVE(COutputTransformer)
  DECLARE_MANDATORY_APIS(COutputTransformer)

public:
  /**
   * @brief A runnable that transforms IMU signals to customer-defined vehicle frame.
  */
  void TransformImuSignals();

protected:

private:
  COutputTransformer() = default;
  ~COutputTransformer() = default;

  /**
   * \brief Checks if 3D specific force provided by the monitor is valid and safe.
   * \param korImuData Output of the IMU monitor.
   * \return True -- valid and safe, false -- otherwise.
   */
  static bool isSpecificForceValidAndSafe(const CImuDataAfterMonitor& korImuData);

  /**
   * \brief Checks if 3D angular provided by the monitor is valid and safe.
   * \param korImuData Output of the IMU monitor.
   * \return True -- valid and safe, false -- otherwise.
   */
  static bool isAngularRateValidAndSafe(const CImuDataAfterMonitor& korImuData);

  bool bIsInitialized_ { false };
};

#endif /* C_OUTPUT_TRANSFORMER_H */
