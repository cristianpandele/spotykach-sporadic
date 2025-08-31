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

    // Process a single input sample: apply envelope following and modulate volume
    float process (float input);

    void processBlockMono (float *input, float *output, size_t blockSize);

    // Set attack/release times
    void setAttackMs (float ms) { envFollower_.setAttackMs(ms); }

    void setReleaseMs (float ms) { envFollower_.setReleaseMs(ms); }

  private:
    EnvelopeFollower envFollower_;
};
