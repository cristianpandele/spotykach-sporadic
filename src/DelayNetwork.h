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
      bool  play;
      bool  reverse;
      int   numBands;
      int   numProcs;
      float centerFreq;
      float stretch;
      float treeDensity;
      float treeOffset;
      float myceliaMix;
    };

    // Provide expected processing block size so internal buffers can be
    // allocated once and re-used every audio callback (no per-block allocs).
    void init (float sampleRate, size_t blockSize, int numBands, int numProcs);
    void setParameters (const Parameters &p);

    // Getter for current band cutoff frequencies
    void getBandFrequencies (std::vector<float> &frequencies) const;

#ifdef DEBUG
    // Getter for current inter-node connection matrix (numProcs x numProcs)
    void getNodeInterconnectionMatrix (std::vector<std::vector<float>> &matrix) const;

    // Get current sidechain levels for all processors
    void getSidechainLevels (size_t ch, std::vector<float> &scLevels) const;
#endif

    // Process a block: input mono arrays (size N) -> output mono (size N)
    // Per-processor gains applied during mixing of per-processor outputs.
    void processBlockMono (const float *in, const uint8_t ch, float *out, size_t blockSize);

    // Provide the current envelope ring used for per-processor gain sampling.
    // ring: array of length len with values typically in [0,1].
    void setFoldWindow(const float* ring, uint8_t len);

    // Tree density (0..1) maps linearly to number of active trees [1..numProcs_].
    void setTreeDensity(float density);

    // Getter for number of active trees
    int getNumActiveTrees () const { return numActiveTrees_; }

    // Get normalized tree positions [0,1], size == numActiveTrees
    void getTreePositions (std::vector<float> &positions) const;

  private:
    float  sampleRate_     = 48000.0f;
    size_t blockSize_      = 16;
    size_t numBands_       = kMaxNutrientBands;
    size_t numProcs_       = kMaxNumDelayProcs;
    bool   play_           = false;
    bool   reverse_        = false;
    float  centerFreq_     = 1000.0f;
    float  stretch_        = 1.0f;
    float  treeOffset_     = 0.0f;    // Normalized offset applied to tree positions
    float  myceliaMix_     = 1.0f;    // Blend factor for inter-node routing
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
    std::array<Utils::SmoothValue, kMaxNumDelayProcs> perProcGains_;

    // Allocate storage for internal buffers and delay processors
    void allocateStorage ();
    // Update per-processor gains based on current fold window and active trees
    void updatePerProcGains ();
};
