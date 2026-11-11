/// @file CLowPassFilter.h
/// @brief Declaration of the simple low-pass filter.
/// @copyright Copyright 2023. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef C_LOWPASS_FILTER_H
#define C_LOWPASS_FILTER_H

class CLowpassFilter
{
public:
  CLowpassFilter() = default;
  CLowpassFilter(float fCutoffFrequency, float fSamplingFrequency);

  float Get();
  void Add(float fNewValue);
  void Reset();
  void Configure(float fCutoffFrequency, float fSamplingFrequency);

protected:

private:

  static float computeLowpassFilterAplha(float fCutoffFrequency, float fSamplingFrequency);

  float fAlpha_{ 0.0F };
  float fFilteredSignal_ { 0.0F };
  bool bIsInitialized_ { false };
};

#endif // C_LOWPASS_FILTER_H
