#pragma once

#include <vector>
#include <cstddef>

// Placeholder DelayNodes: currently a talk-through stage that simply copies
// diffusion band buffers to its outputs. This mirrors the structural role of
// the future complex network.
class DelayNodes
{
  public:
    void init (float sampleRate, int numBands);
    void processBlockMono (float **inBand, float **outBand, size_t blockSize);

    int getNumBands () const { return numBands_; }

  private:
    float sampleRate_ = 48000.0f;
    int   numBands_   = 0;
};
