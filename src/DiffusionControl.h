#pragma once

#include "daisysp.h"
#include "hardware.h"
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

// Lightweight diffusion control that splits the incoming stereo signal into
// multiple logarithmically spaced bands using low-pass, band-pass and high-pass filters.
class DiffusionControl
{
  public:
    struct Parameters
    {
        int   numActiveBands = 4;          // (1..maxBands)
        float centerFreq     = 1000.0f;    // (Hz)
    };

    // Maximum number of supported bands (made public for other modules)
    static constexpr int     kMaxBands = 8;     // Max number of bands
    static constexpr uint8_t kMaxOrder = 16;    // Max filter order

    static constexpr float kMinFreq    = 250.0f;     // Minimum frequency (Hz)
    static constexpr float kMaxFreq    = 8000.0f;    // Maximum frequency (Hz)

    void init (float sampleRate, int numBands = 4);
    void setParameters (const Parameters &p);

    int getNumBands () const { return numBands_; }

    void getBandFrequencies (std::vector<float> &frequencies) const;

    // Process one audio block. outBand must point to an array
    // of numBands() float* buffers, each buffer having blockSize samples.
    void processBlockMono (const float *in, const uint8_t ch, float **outBand, size_t blockSize);

  private:
    float sampleRate_ = 48000.0f;
    int   numBands_   = 4;

    float minFreq_    = kMinFreq;
    float maxFreq_    = kMaxFreq;

    std::array<std::array<daisysp::Svf, kMaxBands>, spotykach::kNumberChannelsStereo> filters_{};
    std::array<float, kMaxBands>        centerFreqs_{};

    void updateBandLayout ();
};
