#pragma once
#include "common.h"
#include "daisysp.h"
#include "Effects/overdrive.h"

using namespace daisysp;

// Simple input sculpting bandpass filter with controllable center frequency and bandwidth (Q).
// Values are provided in normalized 0..1 range; mapping handled internally.
class InputSculpt
{
  public:
    InputSculpt ()  = default;
    ~InputSculpt () = default;

    void init (float sampleRate);

    // Process one sample
    float processSample (float in);

    // Normalized 0..1 -> 50Hz.. 18000Hz (logarithmic mapping for perceptual uniformity)
    void setFreq (float f);

    // Normalized 0..1 -> Q 0.5.. 12 (inverse exponential so higher knob widens bandwidth)
    void setWidth (float w);

    // Getters for filter parameters
    float getCenterFreq () const { return centerFreq_; }
    float getQ () const { return q_; }

    // Set overdrive amount (mapped to kMinDriveAmt.. kMaxDriveAmt range)
    void setOverdrive (float od);
    // Get overdrive amount
    float getOverdrive () const { return overdriveAmt_; }

  private:
    // Filter parameters
    float centerFreq_ = 1000.0f;
    float q_          = 0.0f;
    // Overdrive parameters
    float kMinDriveAmt      = 0.5f;
    float kMaxDriveAmt      = 0.7f;
    float kMinDriveGainComp = 0.5f;
    float kMaxDriveGainComp = 0.1f;
    float overdriveAmt_     = kMinDriveAmt;

    daisysp::Svf       svf_;
    daisysp::Overdrive overdrive_;

    void updateFilter (float freq, float q);
};
