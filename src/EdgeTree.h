#pragma once
#include "Modulation.h"
#include "common.h"
#include <cmath>
#include <daisysp.h>

// EdgeTree class for envelope following and volume modulation
class EdgeTree
{
  public:
    EdgeTree (float sampleRate) : envFollower_(sampleRate) { }

    EdgeTree () = delete;

    void init (float sampleRate);

    // Set tree size
    void setTreeSize (float size);

    // Process a single input sample: apply envelope following and modulate volume
    float process (float input);

    void processBlockMono (float *input, float *output, size_t blockSize);

    // Getter for current envelope value
    float getEnvelope() const { return envelope_; }

  private:
    EnvelopeFollower envFollower_;

    float envelope_ = 0.0f;    // Current envelope value

    ///////////
    NOCOPY (EdgeTree);
};
