#include "DelayNetwork.h"
#include <algorithm>
#include <cassert>

void DelayNetwork::init (float sampleRate, size_t blockSize, int numBands, int numProcs)
{
  sampleRate_ = sampleRate;
  numBands_   = std::max(1, numBands);
  numBands_   = std::min(numBands_, kMaxNutrientBands);
  numProcs_   = std::max(1, numProcs);
  numProcs_   = std::min(numProcs_, kMaxNumDelayProcsPerBand);
  blockSize_  = blockSize;
  diffusion_.init(sampleRate_, numBands_);
  delayNodes_.init(sampleRate_, blockSize_, numBands_, numProcs_);
}

void DelayNetwork::setParameters (const Parameters &p)
{
  int newBands = std::max(1, p.numBands);
  int newProcs = std::max(1, p.numProcs);
  float newCenterFreq = daisysp::fclamp(p.centerFreq, DiffusionControl::kMinFreq, DiffusionControl::kMaxFreq);
  if ((newBands != numBands_) || (newCenterFreq != centerFreq_))
  {
    diffusion_.setParameters({.numActiveBands = newBands, .centerFreq = newCenterFreq});
  }
  if ((newBands != numBands_) || (newProcs != numProcs_))
  {
    delayNodes_.init(sampleRate_, blockSize_, newBands, newProcs);
  }
  centerFreq_ = newCenterFreq;
  numBands_ = newBands;
  numProcs_ = newProcs;
}

void DelayNetwork::getBandFrequencies (std::vector<float> &frequencies) const
{
  diffusion_.getBandFrequencies(frequencies);
}

void DelayNetwork::processBlockMono (const float *in, const uint8_t ch, float *out, size_t blockSize)
{
  // Block size must match initialized block size (simplest design); assert in debug.
  assert(blockSize == blockSize_);

  float *outBand[numBands_];
  // Stage 1: Diffusion writes into the "input" staging buffers for the delay network.
  // This keeps a clear pipeline: diffusion -> storageIn_ -> DelayNodes -> storageOut_.
  for (int b = 0; b < numBands_; ++b)
  {
    outBand[b] = &storageIn_[ch][b][0];
  }

  diffusion_.processBlockMono(in, ch, outBand, blockSize);

  float *inDelay[numBands_];
  float *outDelay[numBands_];
  for (int b = 0; b < numBands_; ++b)
  {
    inDelay[b]  = &storageIn_[ch][b][0];
    outDelay[b] = &storageOut_[ch][b][0];
  }

  delayNodes_.processBlockMono(inDelay, outDelay, ch, blockSize);

  // For now, sum all bands to the channel output
  std::fill(out, out + blockSize, 0.0f);
  for (int band = 0; band < numBands_; ++band)
  {
    const float *b = outDelay[band];
    for (size_t i = 0; i < blockSize; ++i)
    {
      out[i] += b[i];
    }
  }
  // Normalize the output to avoid excessive gain
  // float norm = 1.0f / std::max(1, numBands_);
  // for (size_t i = 0; i < blockSize; ++i)
  // {
  //   out[i] *= norm;
  // }
}
