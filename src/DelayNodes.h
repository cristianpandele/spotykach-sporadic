#pragma once

#include "constants.h"
#include "daisy.h"
#include <array>
#include <cstddef>
#include <vector>

using namespace spotykach;

// DelayNodes: per channel we maintain a set of delay processors (proc[0..numProcs_-1]).
// A routing matrix interNodeConnections_[src][dst] (size kMaxNumDelayProcs^2) defines how
// the output of processor 'src' feeds the input of processor 'dst'. Matrix defaulting to
// a simple forward chain: p -> p+1 with weight 1.0f).
// processorBuffers_[p] holds the most recent output sample for processor p while
// processing a block.
class DelayNodes
{
  public:
    void init (float sampleRate, size_t blockSize, int numBands, int numProcs);
    void processBlockMono (float **inBand, float **outBand, size_t ch, size_t blockSize);

    int getNumBands () const { return numBands_; }

    void setStretch (float stretch);

  private:
    float  sampleRate_ = 48000.0f;
    size_t blockSize_  = 16;
    int    numBands_   = kMaxNutrientBands;
    int    numProcs_   = kMaxNumDelayProcs;
    float  stretch_    = 1.0f;

    void allocateResources ();
    void setInitialConnections ();
    void setDelayProcsParameters ();

    // Routing matrix: weight from src (row) to dst (col).
    float interNodeConnections_[kMaxNumDelayProcs][kMaxNumDelayProcs] = {};
    // Per-processor scratch buffer for current sample processing.
    float processorBuffers_[kMaxNumDelayProcs] = {};
};
