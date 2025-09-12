#pragma once

#include "DelayNodes.h"
#include "DiffusionControl.h"
#include "app.h"
#include "constants.h"
#include <array>

using namespace spotykach;
using namespace spotykach_hwtest;
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

    // Provide the current envelope ring used for per-processor gain sampling.
    // ring: array of length len with values typically in [0,1].
    void setFoldWindow(const float* ring, uint8_t len);

    // Tree density (0..1) maps linearly to number of active trees [1..numProcs_].
    void setTreeDensity(float density);

    // Get normalized tree positions [0,1], size == numActiveTrees
    void getTreePositions(std::vector<float>& positions) const;
    int  getNumActiveTrees() const { return numActiveTrees_; }

  private:
    float  sampleRate_     = 48000.0f;
    size_t blockSize_      = 16;
    size_t numBands_       = kMaxNutrientBands;
    size_t numProcs_       = kMaxNumDelayProcs;
    float  centerFreq_     = 1000.0f;
    float  stretch_        = 1.0f;
    float  treeDensity_    = 1.0f;    // [0,1]
    size_t numActiveTrees_ = kMaxNumDelayProcs;

    DiffusionControl diffusion_;
    DelayNodes       delayNodes_;

    // Backing storage for per-band diffusion outputs: channel x band x block
    std::array<std::array<std::array<float, kBlockSize>, kMaxNutrientBands>, kNumberChannelsStereo> storageBandsIn_;

    // Backing storage for per-processor outputs (tree outputs): channel x proc x block
    std::array<std::array<std::array<float, kBlockSize>, kMaxNumDelayProcs>, kNumberChannelsStereo> storageProcOut_{};

    // Fold window storage for per-delay processor gains
    std::array<float, Hardware::kNumLedsPerRing> foldWindow_{};
    // Current length of the fold window
    size_t foldWindowLen_ = 0;
    // Current per-processor gains
    std::array<float, kMaxNumDelayProcs> perProcGains_{};

    // Allocate storage for internal buffers and delay processors
    void allocateStorage ();
    // Update per-processor gains based on current fold window and active trees
    void updatePerProcGains ();
};
