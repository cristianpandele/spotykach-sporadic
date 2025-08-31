#include "DelayNodes.h"
#include "Sporadic.h"
#include "DelayProc.h"
#include "app.h"
#include <algorithm>

  // Matrix of delay processors: ch x bands x procs
  static DSY_SDRAM_BSS std::array<std::array<std::array<DelayProc, kMaxNumDelayProcsPerBand>, kMaxNutrientBands>, kNumberChannelsStereo> delayProcs_;

  // Processor buffers: ch x band x proc x samples
  static DSY_SDRAM_BSS std::array<std::array<std::array<std::array<float, kBlockSize>, kMaxNumDelayProcsPerBand>, kMaxNutrientBands>, kNumberChannelsStereo>
                        processorBuffers_;

  // Tree output buffers: ch x band x tree x samples
  static DSY_SDRAM_BSS std::array<
    std::array<std::array<std::array<float, kBlockSize>, kMaxNumDelayProcsPerBand>, kMaxNutrientBands>,
    kNumberChannelsStereo> treeOutputBuffers_;

// Tree connections: band x tree (true if connected)
static DSY_SDRAM_BSS std::array<std::array<bool, kMaxNumDelayProcsPerBand>, kMaxNutrientBands> treeConnections_;

// Inter-node connections: band x proc x targetBand x targetProc
static DSY_SDRAM_BSS std::array<
                      std::array<
                        std::array<
                          std::array<float,
                      kMaxNumDelayProcsPerBand>,
                        kMaxNutrientBands>,
                          kMaxNumDelayProcsPerBand>,
                            kMaxNutrientBands>
                      interNodeConnections_;

void DelayNodes::init (float sampleRate, size_t blockSize, int numBands, int numProcs)
{
  sampleRate_ = sampleRate;
  numBands_   = std::max(1, numBands);
  numProcs_   = std::max(1, numProcs);
  blockSize_  = blockSize;
  allocateResources();
  setInitialConnections();
}

void DelayNodes::setStretch (float stretch)
{
  stretch_ = stretch;
  setDelayProcsParameters();
}

void DelayNodes::allocateResources ()
{
  // Allocate delay processors matrix
  for (int ch = 0; ch < kNumberChannelsStereo; ++ch)
  {
    for (int b = 0; b < numBands_; ++b)
    {
      for (int p = 0; p < numProcs_; ++p)
      {
        delayProcs_[ch][b][p].init(sampleRate_, DelayProc::MAX_DELAY);
      }
      // Set default parameters
      setDelayProcsParameters();
    }
  }
}

void DelayNodes::setDelayProcsParameters ()
{
  float perProcStretch = stretch_ / static_cast<float>(numProcs_);
  for (int ch = 0; ch < kNumberChannelsStereo; ++ch)
  {
    for (int b = 0; b < numBands_; ++b)
    {
      for (int p = 0; p < numProcs_; ++p)
      {
        delayProcs_[ch][b][p].setParameters(perProcStretch, 0.0f);
      }
    }
  }
}

void DelayNodes::setInitialConnections ()
{
  // Set tree connections to all true
  for (int b = 0; b < numBands_; ++b)
  {
    for (int t = 0; t < numProcs_; ++t)
    {
      treeConnections_[b][t] = true;
    }
  }

  // Set inter-node connections: same row, sequential
  for (int b = 0; b < numBands_; ++b)
  {
    for (int p = 0; p < numProcs_; ++p)
    {
      for (int tb = 0; tb < numBands_; ++tb)
      {
        for (int tp = 0; tp < numProcs_; ++tp)
        {
          if (b == tb && tp == p + 1)
          {
            interNodeConnections_[b][p][tb][tp] = 1.0f;
          }
          else
          {
            interNodeConnections_[b][p][tb][tp] = 0.0f;
          }
        }
      }
    }
  }
}

void DelayNodes::processBlockMono (float **inBand, float **outBand, size_t ch, size_t blockSize)
{
  // Clear tree outputs
  std::fill(treeOutputBuffers_.begin(),
            treeOutputBuffers_.end(),
            std::array<std::array<std::array<float, kBlockSize>, kMaxNumDelayProcsPerBand>, kMaxNutrientBands>{0.0f});

  // Process each band
  for (int b = 0; b < numBands_; ++b)
  {
    // For each processor in the band
    for (int p = 0; p < numProcs_; ++p)
    {
      // Assume blockSize matches blockSize_
      for (size_t s = 0; s < blockSize; ++s)
      {
        // // Update the target delay
        // delayProcs_[ch][b][p].updateCurrentDelay();

        // Prepare the inputs to the delay processor
        float input = 0.0f;
        if (p == 0)
        {
          // First processor gets input from inBand
          input = inBand[b][s];
        }
        else
        {
          // Subsequent get from previous processor
          input += processorBuffers_[ch][b][p - 1][s];
        }

        // Add inter-node inputs
        for (int sb = 0; sb < numBands_; ++sb)
        {
          for (int sp = 0; sp < numProcs_; ++sp)
          {
            if (interNodeConnections_[sb][sp][b][p] > 0.0f)
            {
              input += processorBuffers_[ch][sb][sp][s] * interNodeConnections_[sb][sp][b][p];
            }
          }
        }

        // Process through delay proc
        processorBuffers_[ch][b][p][s] = delayProcs_[ch][b][p].process(input);

        // Add to tree outputs if connected
        for (int t = 0; t < numProcs_; ++t)
        {
          if (treeConnections_[b][t])
          {
            treeOutputBuffers_[ch][b][t][s] += processorBuffers_[ch][b][p][s];
          }
        }
      }
    }

    // Sum tree outputs to outBand
    for (size_t s = 0; s < blockSize; ++s)
    {
      outBand[b][s] = 0.0f;
      for (int t = 0; t < numProcs_; ++t)
      {
        outBand[b][s] += treeOutputBuffers_[ch][b][t][s];
      }
      // Normalize
      outBand[b][s] /= numProcs_;
    }
  }
}
