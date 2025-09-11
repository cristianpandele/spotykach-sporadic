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
  // Process a block. Inputs are per-band buffers (numBands_ x blockSize).
  // Outputs are per-processor buffers (kMaxNumDelayProcs x blockSize).
  // Only the first numProcs_ rows are written; the rest should be treated as zero.
  void processBlockMono (float **inBand, float **treeOutputs, size_t ch, size_t blockSize);

    int getNumBands () const { return numBands_; }

    void setStretch (float stretch);

    // Density controls how many trees (processors) are considered active.
    // 0 -> 1 active tree, 1 -> numProcs_ active trees (linear mapping).
    void setTreeDensity(float density);

    // Number of active trees computed from density (in [1, numProcs_]).
    int getNumActiveTrees() const { return numActiveTrees_; }

    // Get normalized tree positions in [0,1], size == numActiveTrees_.
    void getTreePositions(std::vector<float>& positions) const;

  private:
    float  sampleRate_ = 48000.0f;
    size_t blockSize_  = 16;
    int    numBands_   = kMaxNutrientBands;
    int    numProcs_   = kMaxNumDelayProcs;
    float  stretch_    = 1.0f;

    // Tree density (0..1) maps linearly to number of active trees [1..numProcs_].
    float treeDensity_ = 1.0f;    // [0,1]
    // The current number of active trees
    size_t numActiveTrees_ = 1;    // [1,numProcs_]
    // Normalized positions within [0,1]; last entry is 1.0 to indicate end of chain.
    std::array<float, kMaxNumDelayProcs> treePositions_{};

    void allocateResources ();
    void setInitialConnections ();
    void setDelayProcsParameters ();
    void updateTreePositions ();

    // Routing matrix: weight from src (row) to dst (col).
    float interNodeConnections_[kMaxNumDelayProcs][kMaxNumDelayProcs] = {};
    // Per-processor scratch buffer for current sample processing.
    float processorBuffers_[kMaxNumDelayProcs] = {};
};
