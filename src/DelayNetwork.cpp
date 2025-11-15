#include "DelayNetwork.h"
#include "app.h"
#include <algorithm>
#include <cassert>

void DelayNetwork::init (float sampleRate, size_t blockSize, int numBands, int numProcs)
{
  sampleRate_ = sampleRate;
  blockSize_  = blockSize;
  numBands_   = std::max(1, numBands);
  numBands_   = std::min(numBands_, kMaxNutrientBands);
  numProcs_   = std::max(1, numProcs);
  numProcs_   = std::min(numProcs_, kMaxNumDelayProcs);
  stretch_    = 1.0f;
  perProcGains_.fill(Utils::SmoothValue(1.0f, 150.0f, kSamplePeriodMs * kBlockSize));
  diffusion_.init(sampleRate_, numBands_);
  delayNodes_.init(sampleRate_, blockSize_, numBands_, numProcs_);
}

void DelayNetwork::setParameters (const Parameters &p)
{
  bool   newPlay       = p.play;
  bool   newReverse    = p.reverse;
  size_t newBands      = std::max(1, p.numBands);
  size_t newProcs      = std::max(1, p.numProcs);
  float  newCenterFreq = daisysp::fclamp(p.centerFreq, DiffusionControl::kMinFreq, DiffusionControl::kMaxFreq);
  if (newReverse != reverse_)
  {
    delayNodes_.setReverse(newReverse);
  }
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

  play_       = newPlay;
  reverse_    = newReverse;
  centerFreq_ = newCenterFreq;
  numBands_   = newBands;
  numProcs_   = newProcs;
  stretch_    = newStretch;
}

// Getter for current band cutoff frequencies
void DelayNetwork::getBandFrequencies (std::vector<float> &frequencies) const
{
  diffusion_.getBandFrequencies(frequencies);
}

#ifdef DEBUG
// Getter for current inter-node connection matrix (numProcs x numProcs)
void DelayNetwork::getNodeInterconnectionMatrix (std::vector<std::vector<float>> &matrix) const
{
  delayNodes_.getNodeInterconnectionMatrix(matrix);
}

void DelayNetwork::getSidechainLevels (size_t ch, std::vector<float> &scLevels) const
{
  const_cast<DelayNodes &>(delayNodes_).getSidechainLevels(ch, scLevels);
}
#endif

void DelayNetwork::getTreePositions (std::vector<float> &positions) const
{
  // Gather from nodes; DelayNodes holds the authoritative positions
  // Note: const_cast used to call non-const getter if needed; prefer const in DelayNodes if making changes later
  const_cast<DelayNodes &>(delayNodes_).getTreePositions(positions);
}

void DelayNetwork::processBlockMono (const float *in, const uint8_t ch, float *out, size_t blockSize)
{
  // Block size must match initialized block size (simplest design); assert in debug.
  assert(blockSize == blockSize_);

  if (!play_)
  {
    // If not playing, silence the output
    for (size_t b = 0; b < numBands_; ++b)
    {
      std::fill(out, out + blockSize, 0.0f);
    }
    return;
  }

  float *outBand[numBands_];
  // Stage 1: Diffusion writes into the "input" staging buffers for the delay network.
  // This keeps a clear pipeline: diffusion -> storageBandsIn_ -> DelayNodes -> storageProcOut_.
  for (size_t b = 0; b < numBands_; ++b)
  {
    outBand[b] = &storageBandsIn_[ch][b][0];
  }

  diffusion_.processBlockMono(in, ch, outBand, blockSize);

  float *inDelay[numBands_];
  for (size_t b = 0; b < numBands_; ++b)
  {
    inDelay[b] = &storageBandsIn_[ch][b][0];
  }

  // Prepare per-processor output pointer table
  float *treeOutputs[kMaxNumDelayProcs];
  for (size_t p = 0; p < kMaxNumDelayProcs; ++p)
  {
    treeOutputs[p] = &storageProcOut_[ch][p][0];
  }

  delayNodes_.processBlockMono(inDelay, treeOutputs, ch, blockSize);

  // Mix only the active processors' outputs into the channel buffer
  std::fill(out, out + blockSize, 0.0f);
  for (size_t p = 0; p < numProcs_; ++p)
  {
    const float *procBuf = treeOutputs[p];
    const float  gain    = perProcGains_[p].getSmoothVal();
    for (size_t i = 0; i < blockSize; ++i)
    {
      out[i] += procBuf[i] * gain;
    }
  }
}

void DelayNetwork::setTreeDensity (float density)
{
  delayNodes_.setTreeDensity(density);
  // Cache active tree count for external queries
  numActiveTrees_ = delayNodes_.getNumActiveTrees();
  updatePerProcGains();    // Tree topology changed -> resample gains
}

void DelayNetwork::setFoldWindow (const float *ring, uint8_t len)
{
  // Store a bounded copy and mark gains dirty
  foldWindowLen_ = len;
  if (ring && foldWindowLen_ > 0)
  {
    std::copy(ring, ring + foldWindowLen_, foldWindow_.begin());
  }
  else
  {
    foldWindowLen_ = 0;
  }
  updatePerProcGains();    // Fold window changed -> resample gains
}

void DelayNetwork::updatePerProcGains ()
{
  // Default: unity gains
  for (size_t p = 0; p < kMaxNumDelayProcs; ++p)
  {
    perProcGains_[p] = 1.0f;
  }
  // If we have a fold window and active trees, sample gains at tree positions
  if (foldWindowLen_ && numActiveTrees_)
  {
    // Get tree positions
    std::vector<float> positions;
    getTreePositions(positions);

    if (numActiveTrees_ > positions.size())
    {
      // If fewer positions than active trees, return (should not happen)
      return;
    }

    for (size_t p = 0; p < numActiveTrees_; ++p)
    {
      size_t idx = static_cast<size_t>(p) % positions.size();
      float  t   = positions[idx];

      // Map t in [0,1] to ring index
      float ri         = daisysp::fmap(t, 0.0f, static_cast<float>(foldWindowLen_ - 1));
      perProcGains_[p] = foldWindow_[std::round(ri)];
    }

    // Clear remaining gains to 0
    for (size_t p = numActiveTrees_; p < kMaxNumDelayProcs; ++p)
    {
      perProcGains_[p] = 0.0f;
    }
  }
}