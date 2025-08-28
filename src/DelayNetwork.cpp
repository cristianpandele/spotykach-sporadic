#include "DelayNetwork.h"
#include <algorithm>
#include <cassert>

void DelayNetwork::init (float sampleRate, size_t blockSize, int numBands)
{
  sampleRate_ = sampleRate;
  numBands_   = std::max(1, numBands);
  numBands_   = std::min(numBands_, DiffusionControl::kMaxBands);
  blockSize_  = blockSize;
  diffusion_.init(sampleRate_, numBands_);
  delayNodes_.init(sampleRate_, numBands_);
  allocateStorage();
}

void DelayNetwork::setParameters (const Parameters &p)
{
  int newBands = std::max(1, p.numBands);
  if (newBands != numBands_)
  {
    numBands_ = newBands;
    diffusion_.setParameters(
      {
        .numActiveBands = numBands_,
        .minFreq = DiffusionControl::kMinFreq,
        .maxFreq = DiffusionControl::kMaxFreq
      });
    delayNodes_.init(sampleRate_, numBands_);
    allocateStorage();
  }
}

void DelayNetwork::getBandFrequencies (std::vector<float> &frequencies) const
{
  diffusion_.getBandFrequencies(frequencies);
}

void DelayNetwork::allocateStorage ()
{
  // Allocate contiguous storage for each channel stage: size = numBands_ * blockSize_
  auto ensure = [&] (std::vector<float> &buf) { buf.assign(numBands_ * blockSize_, 0.0f); };
  ensure(storageIn_);
  ensure(storageOut_);

  for (int b = 0; b < DiffusionControl::kMaxBands; ++b)
  {
    bandInPtrs_[b]  = (b < numBands_) ? &storageIn_[b * blockSize_] : nullptr;
    bandOutPtrs_[b] = (b < numBands_) ? &storageOut_[b * blockSize_] : nullptr;
  }
}

void DelayNetwork::processBlockMono (const float *in, float *out, size_t blockSize)
{
  // Block size must match initialized block size (simplest design); assert in debug.
  assert(blockSize == blockSize_);

  float *outBand[numBands_];
  // Stage 1: Diffusion writes into the "input" staging buffers for the delay network.
  // This keeps a clear pipeline: diffusion -> storageIn_ -> DelayNodes -> storageOut_.
  for (int b = 0; b < numBands_; ++b)
  {
    outBand[b] = bandInPtrs_[b];
  }

  diffusion_.processBlockMono(in, outBand, blockSize);

  float *inDelay[numBands_];
  float *outDelay[numBands_];
  for (int b = 0; b < numBands_; ++b)
  {
    inDelay[b]  = bandInPtrs_[b];
    outDelay[b] = bandOutPtrs_[b];
  }

  delayNodes_.processBlockMono(inDelay, outDelay, blockSize);

  // For now, sum all bands to the channel output
  std::fill(out, out + blockSize, 0.0f);
  for (int band = 0; band < numBands_; ++band)
  {
    const float *b = bandOutPtrs_[band];
    for (size_t i = 0; i < blockSize; ++i)
    {
      out[i] += b[i];
    }
  }
  // Normalize the
  float norm = 1.0f / std::max(1, numBands_);
  for (size_t i = 0; i < blockSize; ++i)
  {
    out[i] *= norm;
  }
}
