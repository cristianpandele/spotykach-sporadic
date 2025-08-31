#pragma once

#include "DelayNodes.h"
#include "DiffusionControl.h"
#include "constants.h"
#include <array>

using namespace spotykach;
// DelayNetwork wiring DiffusionControl -> DelayNodes -> sum of bands.
class DelayNetwork
{
  public:
    struct Parameters
    {
        int   numBands;
        int   numProcs;
        float centerFreq;
    };

    // Provide expected processing block size so internal buffers can be
    // allocated once and re-used every audio callback (no per-block allocs).
    void init (float sampleRate, size_t blockSize, int numBands, int numProcs);
    void setParameters (const Parameters &p);

    void getBandFrequencies (std::vector<float> &frequencies) const;

    // Process a block: input stereo arrays (size N) -> output stereo (size N)
    void processBlockMono (const float *in, const uint8_t ch, float *out, size_t blockSize);

  private:
    float  sampleRate_ = 48000.0f;
    size_t blockSize_  = 16;
    int    numBands_   = kMaxNutrientBands;
    int    numProcs_   = kMaxNumDelayProcsPerBand;
    float  centerFreq_ = 1000.0f;

    DiffusionControl diffusion_;
    DelayNodes       delayNodes_;

    // Pointers to storage: channel * bands * blockSize samples per channel stage
    std::array<std::array<float *, kMaxNutrientBands>, kNumberChannelsStereo> bandInPtrs_{};
    std::array<std::array<float *, kMaxNutrientBands>, kNumberChannelsStereo> bandOutPtrs_{};

    // Owning buffer backing the above pointers
    std::array<std::array<std::array<float, kBlockSize>, kMaxNutrientBands>, kNumberChannelsStereo> storageIn_;
    std::array<std::array<std::array<float, kBlockSize>, kMaxNutrientBands>, kNumberChannelsStereo> storageOut_;

    void allocateStorage ();
};
