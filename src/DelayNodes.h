#pragma once

#include "constants.h"
#include "daisy.h"
#include <array>
#include <cstddef>
#include <vector>

using namespace spotykach;

// DelayNodes implemented as a matrix of DelayProc entries (bands x procs)
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
    int    numProcs_   = kMaxNumDelayProcsPerBand;
    float  stretch_    = 1.0f;

    void allocateResources ();
    void setInitialConnections ();
    void setDelayProcsParameters ();
};
