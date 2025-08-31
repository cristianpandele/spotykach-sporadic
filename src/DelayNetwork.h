#pragma once

#include "DiffusionControl.h"
#include "DelayNodes.h"
#include <array>

// DelayNetwork wiring DiffusionControl -> DelayNodes -> sum of bands.
class DelayNetwork
{
  public:
    struct Parameters
    {
        int   numBands   = 4;          // (1..DiffusionControl::kMaxBands)
        float centerFreq = 1000.0f;    // Default center frequency
    };

    // Provide expected processing block size so internal buffers can be
    // allocated once and re-used every audio callback (no per-block allocs).
    void init (float sampleRate, size_t blockSize, int numBands);
    void setParameters (const Parameters &p);

#if DEBUG
    void getBandFrequencies (std::vector<float> &frequencies) const;
#endif

    // Process a block: input stereo arrays (size N) -> output stereo (size N)
    void processBlockMono (const float *in, const uint8_t ch, float *out, size_t blockSize);

  private:
    float  sampleRate_ = 48000.0f;
    int    numBands_   = 4;
    float  centerFreq_ = 1000.0f;
    size_t blockSize_  = 16;    // default, must match init provided size

    DiffusionControl diffusion_;
    DelayNodes       delayNodes_;

    // Flat contiguous storage: bands * blockSize samples per channel stage
    std::array<float *, DiffusionControl::kMaxBands> bandInPtrs_{};
    std::array<float *, DiffusionControl::kMaxBands> bandOutPtrs_{};

    // Owning buffer backing the above pointers
    std::vector<float> storageIn_;
    std::vector<float> storageOut_;

    void allocateStorage ();
};
