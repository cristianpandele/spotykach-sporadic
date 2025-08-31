#pragma once

#include <vector>
#include <cstddef>
#include "DaisySP/Source/daisysp.h"
#include "DelayProc.h"

// DelayNodes implemented as a matrix of DelayProc entries (bands x procs)
class DelayNodes
{
  public:
    void init (float sampleRate, int numBands);
    void processBlockMono (float **inBand, float **outBand, size_t blockSize);

    int getNumBands () const { return numBands_; }

  private:
    float sampleRate_ = 48000.0f;
    int   numBands_   = 0;
    size_t blockSize_ = 16;
    static constexpr int numProcs_ = 4; // Number of processors per band (columns)
    static constexpr int numTrees_ = 4; // Number of trees (output taps)

    // Matrix of delay processors: bands x procs
    std::vector<std::vector<DelayProc>> delayProcs_;

    // Processor buffers: band x proc x samples
    std::vector<std::vector<std::vector<float>>> processorBuffers_;

    // Tree output buffers: band x tree x samples
    std::vector<std::vector<std::vector<float>>> treeOutputBuffers_;

    // Tree connections: band x tree (true if connected)
    std::vector<std::vector<bool>> treeConnections_;

    // Inter-node connections: band x proc x targetBand x targetProc
    std::vector<std::vector<std::vector<std::vector<float>>>> interNodeConnections_;

    void allocateResources();
    void setInitialConnections();
};
