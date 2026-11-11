/// @file CButterworthLowPassFilter2.h
/// @brief Declaration of a 2nd order Butterworth low-pass filter.
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef C_BUTTERWORTH_LOW_PASS_FILTER_2_H
#define C_BUTTERWORTH_LOW_PASS_FILTER_2_H

/// @brief 2nd order Butterworth low-pass filter implemented in direct form I.
class CButterworthLowPassFilter2
{
public:
  /// @brief Construct a pass-through placeholder filter.
  ///
  /// The default-constructed filter uses unity gain with no filtering until Configure() is
  /// called or the configured constructor is used.
  CButterworthLowPassFilter2() = default;

  /// @brief Construct and configure the filter.
  ///
  /// ## Function design
  /// ### Normal case
  /// The function SHALL compute the normalized IIR coefficients for a 2nd order Butterworth low-pass filter.
  /// For the formulae see the helper function documentation.
  ///
  /// ### Argument checks
  /// -# The function SHALL assert that the provided cutoff frequency is finite and positive.
  /// -# The function SHALL assert that the provided sampling frequency is finite and positive.
  /// -# The function SHALL assert that the provided cutoff frequency is less than half the sampling frequency (Nyquist frequency).
  ///
  /// @param fCutoffFrequencyHz Cutoff frequency in [Hz]
  /// @param fSamplingFrequencyHz Sampling frequency in [Hz]
  CButterworthLowPassFilter2(float fCutoffFrequencyHz, float fSamplingFrequencyHz);

  /// @brief Get the latest filtered value.
  /// @return Latest filter output. Returns 0.0F while the filter is uninitialized, for example right after default construction or Reset().
  float Get() const;

  /// @brief Feed one new sample into the filter.
  /// @param fNewValue New input sample.
  ///
  /// ## Function design
  /// ### Case 1: Filter is not initialized
  /// -# IF the filter is not initialized, THEN the function SHALL initialize the filter.
  /// -# After initialization, the function SHALL update the pre-computed filtered output.
  /// -# After updating the pre-computed filtered output, the function SHALL update the filter's state.
  ///
  /// For the filter initialization procedure see the helper function documentation.
  ///
  /// ### Case 2: Filter is initialized
  /// -# WHEN the filter is initialized, THEN the function SHALL update the pre-computed filtered output.
  /// -# After updating the pre-computed filtered output, the function SHALL update the filter's state.
  ///
  /// ### Computation of the filtered output
  /// The latest filtered output is given by the direct-form-I recurrence
  ///
  /// @f[
  ///
  /// y[k] = b_0 x[k] + b_1 x[k - 1] + b_2 x[k - 2] - a_1 y[k - 1] - a_2 y[k - 2]
  ///
  /// @f]
  ///
  /// where @f$ x[k] = @f$ `fNewValue`, and the coefficients are given by the current filter configuration.
  void Add(float fNewValue);

  /// @brief Reset the filter state.
  ///
  /// ## Function design
  /// -# The function SHALL transition the filter to an uninitialized state.
  /// -# The function SHALL NOT modify the filter coefficients.
  /// -# The function SHALL set the pre-computed filter output to zero.
  void Reset();

  /// @brief Reconfigure the filter coefficients.
  ///
  /// ## Function design
  /// ### Normal case
  /// The function SHALL compute the normalized IIR coefficients for a 2nd order Butterworth low-pass filter.
  /// For the formulae see the helper function documentation.
  ///
  /// ### Argument checks
  /// -# The function SHALL assert that the provided cutoff frequency is finite and positive.
  /// -# The function SHALL assert that the provided sampling frequency is finite and positive.
  /// -# The function SHALL assert that the provided cutoff frequency is less than half the sampling frequency (Nyquist frequency).
  ///
  /// @param fCutoffFrequencyHz Cutoff frequency in [Hz]
  /// @param fSamplingFrequencyHz Sampling frequency in [Hz]
  void Configure(float fCutoffFrequencyHz, float fSamplingFrequencyHz);

private:

  struct SCoefficients
  {
    float fB0_{ 1.0F }; ///< Coefficient @f$ b_0 @f$
    float fB1_{ 0.0F }; ///< Coefficient @f$ b_1 @f$
    float fB2_{ 0.0F }; ///< Coefficient @f$ b_2 @f$
    float fA1_{ 0.0F }; ///< Coefficient @f$ a_1 @f$
    float fA2_{ 0.0F }; ///< Coefficient @f$ a_2 @f$
  };

  struct SFilterState
  {
    float fInput1_{ 0.0F }; ///< Previous input sample.
    float fInput2_{ 0.0F }; ///< Input sample delayed by two epochs.
    float fOutput1_{ 0.0F }; ///< Previous output sample.
    float fOutput2_{ 0.0F }; ///< Output sample delayed by two epochs.
  };

  /// @brief Compute normalized IIR coefficients for the configured Butterworth filter.
  ///
  /// ## Function design
  /// ### Normal case
  /// The function SHALL compute coefficients for the direct-form-I recurrence
  ///
  /// @f[
  ///
  /// y[k] = b_0 x[k] + b_1 x[k - 1] + b_2 x[k - 2] - a_1 y[k - 1] - a_2 y[k - 2]
  ///
  /// @f]
  ///
  /// where @f$ f_c = @f$ `fCutoffFrequencyHz`, @f$ f_s = @f$ `fSamplingFrequencyHz`.
  ///
  /// The function SHALL return the normalized Butterworth coefficients
  ///
  /// @f[
  ///
  /// b_0 = \Omega^2 / c, \; b_1 = 2 b_0, \; b_2 = b_0, @f]
  ///
  /// @f[
  ///
  /// a_1 = 2 (\Omega^2 - 1) / c, \; a_2 = (1 - \sqrt{2} \Omega + \Omega^2) / c.
  ///
  /// @f]
  ///
  /// Herein @f( \Omega = \tan\left(\frac{\pi f_c}{f_s}\right) @f), and @f( c = 1 + \sqrt{2} \Omega + \Omega^2 @f).
  ///
  /// ### Argument checks
  /// -# The function SHALL NOT perform argument checks.
  ///
  /// @param fCutoffFrequencyHz Cutoff frequency in [Hz]
  /// @param fSamplingFrequencyHz Sampling frequency in [Hz]
  /// @return Coefficients of the direct-form-I recurrence.
  static SCoefficients computeCoefficients(float fCutoffFrequencyHz, float fSamplingFrequencyHz);

  /// @brief Initialize the direct-form-I state to the steady state for a constant input.
  /// 
  /// ## Function design
  /// -# The function SHALL set the states @ref SFilterState::fInput1_ and @ref SFilterState::fInput2_ to the provided input value.
  /// -# The function SHALL set the states @ref SFilterState::fOutput1_ and @ref SFilterState::fOutput2_ to the corresponding steady-state output value.
  ///
  /// The steady-state output value is given by
  ///
  /// @f[
  ///
  /// y_\infty = \frac{b_0 + b_1 + b_2}{1 + a_1 + a_2} x_\infty
  ///
  /// @f]
  ///
  /// where @f$ x_\infty = @f$ `fInputValue`.
  ///
  /// @param fInputValue Constant input value to initialize the filter from.
  /// @param korCoefficients Filter coefficients.
  static SFilterState initializeStateWithoutTransient(float fInputValue, const SCoefficients& korCoefficients);

  SCoefficients oCoefficients_{}; ///< Filter coefficients.
  SFilterState oState_{}; ///< State of the direct-form-I recurrence.
  float fFilteredSignal_{ 0.0F }; ///< Latest filter output.
  bool bIsInitialized_{ false }; ///< True once the internal state has been initialized from an input sample.

#ifdef UNIT_TESTS
  friend class CButterworthLowPassFilter2Test;
#endif
};

#endif // C_BUTTERWORTH_LOW_PASS_FILTER_2_H
