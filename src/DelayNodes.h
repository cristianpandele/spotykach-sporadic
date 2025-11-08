#pragma once

#include "constants.h"
#include "daisy.h"
#include <array>
#include <cstddef>
#include <vector>

using namespace spotykach;

struct DelayProc;

// DelayNodes: per channel we maintain a set of delay processors (proc[0..numProcs_-1]).
// A routing matrix interNodeConnections_[src][dst] (size kMaxNumDelayProcs^2) defines how
// the output of processor 'src' feeds the input of processor 'dst'. Matrix defaulting to
// a simple forward chain: p -> p+1 with weight 1.0f).
// processorBuffers_[p] holds the most recent output sample for processor p while
// processing a block.
class DelayNodes
{
  public:
    DelayNodes();
    using DelayProcMatrix = DelayProc (*)[kMaxNumDelayProcs];

    void init (float sampleRate, size_t blockSize, size_t numBands, size_t numProcs);
    // Process a block. Inputs are per-band buffers (numBands_ x blockSize).
    // Outputs are per-processor buffers (kMaxNumDelayProcs x blockSize).
    // Only the first numProcs_ rows are written; the rest should be treated as zero.
    void processBlockMono (float **inBand, float **treeOutputs, size_t ch, size_t blockSize);

    // Getter for number of bands for diffusion
    int getNumBands () const { return numBands_; }

    void setReverse(bool reverse) { reverse_ = reverse; }

    // Set the stretch factor controlling delay times (overall scale).
    void setStretch (float stretch);

    // Set the tree offset (normalized position in [0,1]).
    void setTreeOffset (float offset);

    // Density controls how many trees (processors) are considered active.
    // 0 -> 1 active tree, 1 -> numProcs_ active trees (linear mapping).
    void setTreeDensity (float density);

    // Blend between simple chain routing (0) and full mycelial network (1).
    void setMyceliaMix (float mix);

    // Update the inter-node routing matrix (2D: numProcs_ x numProcs_)
    void updateNodeInterconnections ();

#ifdef DEBUG
    // Getter for current inter-node connection matrix (numProcs x numProcs)
    void getNodeInterconnectionMatrix (std::vector<std::vector<float>> &matrix) const;

    // Get current sidechain levels for all processors
    void getSidechainLevels (size_t ch, std::vector<float> &scLevels) const;
#endif

    // Get normalized tree positions in [0,1], size == numActiveTrees_.
    void getTreePositions (std::vector<float> &positions) const;

    // Number of active trees computed from density (in [1, numProcs_]).
    int getNumActiveTrees() const { return numActiveTrees_; }

  private:
    float  sampleRate_    = 48000.0f;
    size_t blockSize_     = 16;
    size_t numBands_      = kMaxNutrientBands;
    size_t numProcs_      = kMaxNumDelayProcs;
    bool   reverse_       = false;   // Whether the delay network is in reverse mode
    float  stretch_       = 1.0f;    // Overall stretch factor for delay times
    float  entanglement_  = 0.0f;    // [0,1] strength of interconnection dynamics
    float  feedback_      = 0.0f;    // Feedback level for all delay processors
    float  myceliaMix_    = 1.0f;    // Blend factor for inter-node routing
    float  treeOffset_    = 0.0f;    // Normalized offset applied to tree positions
    float  treeDensity_   = 1.0f;    // Normalized tree density (0..1)

    static constexpr float  kNodeInterconnectionUpdateIntervalMs = 2000.0f;    // Update routing every 2 seconds

    DelayProcMatrix delayProcs_ = nullptr;

    // The current number of active trees
    size_t numActiveTrees_ = 1;    // [1,numProcs_]
    // Normalized positions within [0,1]; last entry is 1.0 to indicate end of chain.
    std::array<float, kMaxNumDelayProcs> treePositions_{};

    void allocateResources ();
    void setInitialConnections ();
    void setDelayProcsParameters ();
    void updateTreePositions (bool uniform = true);
    void updateSidechainLevels (size_t ch);

    // Routing matrix: weight from src (row) to dst (col).
    float interNodeConnections_[kMaxNumDelayProcs][kMaxNumDelayProcs] = {};
    // Per-processor scratch buffer for current sample processing.
    float processorBuffers_[kMaxNumDelayProcs][kBurstSizeSamples] = {};
    // Per-processor sidechain level (computed from routing + env levels)
    float sidechainLevels_[kMaxNumDelayProcs] = {};

    daisy::StopwatchTimer interconnectionUpdateTimer;
};
