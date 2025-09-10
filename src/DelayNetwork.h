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
        float stretch;
    };

    // Provide expected processing block size so internal buffers can be
    // allocated once and re-used every audio callback (no per-block allocs).
    void init (float sampleRate, size_t blockSize, int numBands, int numProcs);
    void setParameters (const Parameters &p);

    void getBandFrequencies (std::vector<float> &frequencies) const;

    // Process a block: input mono arrays (size N) -> output mono (size N)
    // Per-processor gains applied during mixing of per-processor outputs.
    void processBlockMono (const float *in, const uint8_t ch, float *out, size_t blockSize);

  private:
    float  sampleRate_ = 48000.0f;
    size_t blockSize_  = 16;
    int    numBands_   = kMaxNutrientBands;
    int    numProcs_   = kMaxNumDelayProcs;
    float  centerFreq_ = 1000.0f;
    float  stretch_    = 1.0f;

    DiffusionControl diffusion_;
    DelayNodes       delayNodes_;

    // Backing storage for per-band diffusion outputs: channel x band x block
    std::array<std::array<std::array<float, kBlockSize>, kMaxNutrientBands>, kNumberChannelsStereo> storageBandsIn_;

    // Backing storage for per-processor outputs (tree outputs): channel x proc x block
    std::array<std::array<std::array<float, kBlockSize>, kMaxNumDelayProcs>, kNumberChannelsStereo> storageProcOut_{};

    void allocateStorage ();
};
