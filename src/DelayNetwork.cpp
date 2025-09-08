#include "DelayNetwork.h"
#include "app.h"
#include <algorithm>
#include <cassert>

void DelayNetwork::init (float sampleRate, size_t blockSize, int numBands, int numProcs)
{
  sampleRate_ = sampleRate;
  numBands_   = std::max(1, numBands);
  numBands_   = std::min(numBands_, kMaxNutrientBands);
  numProcs_   = std::max(1, numProcs);
  numProcs_   = std::min(numProcs_, kMaxNumDelayProcs);
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
  float newStretch = daisysp::fmap(p.stretch, kMinStretch, kMaxStretch, Mapping::LOG);
  if (newStretch != stretch_)
  {
    delayNodes_.setStretch(newStretch);
  }
  centerFreq_ = newCenterFreq;
  numBands_   = newBands;
  numProcs_   = newProcs;
  stretch_    = newStretch;
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
  // This keeps a clear pipeline: diffusion -> storageBandsIn_ -> DelayNodes -> storageProcOut_.
  for (int b = 0; b < numBands_; ++b)
  {
    outBand[b] = &storageBandsIn_[ch][b][0];
  }

  diffusion_.processBlockMono(in, ch, outBand, blockSize);

  float *inDelay[numBands_];
  for (int b = 0; b < numBands_; ++b)
  {
    inDelay[b] = &storageBandsIn_[ch][b][0];
  }

  // Prepare per-processor output pointer table
  float *treeOutputs[kMaxNumDelayProcs];
  for (int p = 0; p < kMaxNumDelayProcs; ++p)
  {
    treeOutputs[p] = &storageProcOut_[ch][p][0];
  }

  delayNodes_.processBlockMono(inDelay, treeOutputs, ch, blockSize);

  // Mix only the active processors' outputs into the channel buffer
  std::fill(out, out + blockSize, 0.0f);
  for (int p = 0; p < numProcs_; ++p)
  {
    const float *procBuf = treeOutputs[p];
    for (size_t i = 0; i < blockSize; ++i)
    {
      out[i] += procBuf[i];
    }
  }
  // Optional normalization by number of active processors
  float norm = 1.0f / static_cast<float>(std::max(1, numProcs_));
  for (size_t i = 0; i < blockSize; ++i)
  {
    out[i] *= norm;
  }
}
