#include "DelayNodes.h"
#include <algorithm>

void DelayNodes::init (float sampleRate, int numBands)
{
  sampleRate_ = sampleRate;
  numBands_   = std::max(1, numBands);
}

void DelayNodes::processBlockMono (float **inBand, float **outBand, size_t blockSize)
{
  for (int b = 0; b < numBands_; ++b)
  {
    std::copy(inBand[b], inBand[b] + blockSize, outBand[b]);
  }
}
