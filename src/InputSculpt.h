#pragma once
#include "daisysp.h"

using namespace daisysp;

// Simple input sculpting bandpass filter with controllable center frequency and bandwidth (Q).
// Values are provided in normalized 0..1 range; mapping handled internally.
class InputSculpt
{
  public:
    InputSculpt ()  = default;
    ~InputSculpt () = default;

    void init (float sampleRate);

    // Normalized 0..1 -> 50Hz..18000Hz (logarithmic mapping for perceptual uniformity)
    void setFreq (float f);

    // Normalized 0..1 -> Q 0.5 .. 12 (inverse exponential so higher knob widens bandwidth)
    void setWidth (float w);

    // Process one sample
    float processSample (float in);

    float getCenterFreq () const { return centerFreq_; }

    float getQ () const { return q_; }

  private:
    daisysp::Svf svf_;
    float        centerFreq_ = 1000.0f;
    float        q_          = 0.0f;

    void updateFilter (float freq, float q);
};
