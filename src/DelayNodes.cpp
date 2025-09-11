#include "DelayNodes.h"
#include "DelayProc.h"
#include <algorithm>

// Delay processors laid out as: channel x proc
static DSY_SDRAM_BSS std::array<std::array<DelayProc, kMaxNumDelayProcs>, kNumberChannelsStereo> delayProcs_;

void DelayNodes::init (float sampleRate, size_t blockSize, int numBands, int numProcs)
{
  sampleRate_ = sampleRate;
  numBands_   = std::clamp(numBands, 1, kMaxNutrientBands);
  numProcs_   = std::clamp(numProcs, 1, kMaxNumDelayProcs);
  blockSize_  = blockSize;
  allocateResources();
}

void DelayNodes::setStretch (float stretch)
{
  stretch_ = stretch;
  setDelayProcsParameters();
}

void DelayNodes::allocateResources ()
{
  for (int ch = 0; ch < kNumberChannelsStereo; ++ch)
  {
    for (int p = 0; p < numProcs_; ++p)
    {
      delayProcs_[ch][p].init(sampleRate_, DelayProc::kMaxDelaySamples);
    }
  }
  // Set default parameters
  setDelayProcsParameters();
  setInitialConnections();
}

void DelayNodes::setDelayProcsParameters ()
{
  float perProcStretch = stretch_ / static_cast<float>(std::max(1, numProcs_));
  for (int ch = 0; ch < kNumberChannelsStereo; ++ch)
  {
    for (int p = 0; p < numProcs_; ++p)
    {
      delayProcs_[ch][p].setParameters(perProcStretch, 0.0f);
    }
  }
}

void DelayNodes::setInitialConnections ()
{
  // Initialize all to 0.
  for (int r = 0; r < kMaxNumDelayProcs; ++r)
  {
    std::fill(std::begin(interNodeConnections_[r]), std::end(interNodeConnections_[r]), 0.0f);
  }
  // Simple forward chain: p -> p+1 gets weight 1.0f
  for (int p = 0; p < kMaxNumDelayProcs - 1; ++p)
  {
    interNodeConnections_[p][p + 1] = 1.0f;
  }
}

void DelayNodes::setTreeDensity(float density)
{
  // Clamp to [0,1]
  float d = std::max(0.0f, std::min(1.0f, density));
  if (d == treeDensity_ && numActiveTrees_)
  {
    return;
  }
  treeDensity_ = d;
  // Map to [1, numProcs_]
  size_t newActiveTrees = std::round(daisysp::fmap(d, 0.0f, numProcs_));
  numActiveTrees_ = std::max(static_cast<size_t>(1), newActiveTrees);
  updateTreePositions();
}

void DelayNodes::getTreePositions(std::vector<float>& positions) const
{
  positions.clear();
  positions.reserve(static_cast<size_t>(numActiveTrees_));
  for (size_t i = 0; i < numActiveTrees_; ++i)
  {
    positions.push_back(treePositions_[i]);
  }
}

void DelayNodes::updateTreePositions()
{
  // Ensure last tree is at 1.0
  treePositions_[numActiveTrees_ - 1] = 1.0f;    // single tree at end of chain

  // Zero out the rest for cleanliness
  for (size_t i = numActiveTrees_; i < kMaxNumDelayProcs; ++i)
  {
    treePositions_[i] = 0.0f;
  }

  // Uniform distribution of any remaining trees in [0,1]
  const float step = 1.0f / static_cast<float>(numActiveTrees_);
  for (size_t i = 0; i < numActiveTrees_ - 1; ++i)
  {
    treePositions_[i] = step * (static_cast<float>(i) + 0.5f);
  }
}

void DelayNodes::processBlockMono (float **inBand, float **treeOutputs, size_t ch, size_t blockSize)
{
  // Routing-based processing:
  // For each sample:
  // 1. Sum all band inputs -> externalInput.
  // 2. For each processor p from 0..numProcs_-1 compute its input as:
  //      (p==0 ? externalInput : 0) + sum_{src < numProcs_} processorBuffers_[src] * interNodeConnections_[src][p]
  // 3. Process to produce processorBuffers_[p].
  // 4. After each processor runs for sample s, write processorBuffers_[p] to treeOutputs[p][s].

  for (size_t s = 0; s < blockSize; ++s)
  {
    // External mixed input across bands
    float externalInput = 0.0f;
    for (int b = 0; b < numBands_; ++b)
    {
      externalInput += inBand[b][s];
    }

    // Compute each processor output
    for (int p = 0; p < numProcs_; ++p)
    {
      float inVal = (p == 0 ? externalInput : 0.0f);
      // Sum contributions from previous processors per routing matrix
      for (int src = 0; src < numProcs_; ++src)
      {
        float w = interNodeConnections_[src][p];
        if (w != 0.0f)
        {
          inVal += processorBuffers_[src] * w;
        }
      }
      processorBuffers_[p] = delayProcs_[ch][p].process(inVal);
    }

    // Write per-processor outputs for this sample
    for (int p = 0; p < numProcs_; ++p)
    {
      treeOutputs[p][s] = processorBuffers_[p];
    }
  }
}
