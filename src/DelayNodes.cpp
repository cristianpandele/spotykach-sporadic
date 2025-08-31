#include "DelayNodes.h"
#include <algorithm>

void DelayNodes::init (float sampleRate, int numBands)
{
  sampleRate_ = sampleRate;
  numBands_   = std::max(1, numBands);
  blockSize_  = 16; // Assume default, can be made parameter later
  allocateResources();
  setInitialConnections();
}

void DelayNodes::allocateResources()
{
  // Allocate delay processors matrix
  delayProcs_.resize(numBands_);
  for (int b = 0; b < numBands_; ++b)
  {
    delayProcs_[b].resize(numProcs_);
    for (int p = 0; p < numProcs_; ++p)
    {
      // Max delay: 1 second
      size_t maxDelaySamples = static_cast<size_t>(sampleRate_ * 1.0f);
      delayProcs_[b][p].init(sampleRate_, maxDelaySamples);
      // Set default parameters
      delayProcs_[b][p].setParameters(100.0f + p * 50.0f, 0.5f, 1000.0f + b * 500.0f);
    }
  }

  // Allocate processor buffers: band x proc x blockSize
  processorBuffers_.resize(numBands_);
  for (int b = 0; b < numBands_; ++b)
  {
    processorBuffers_[b].resize(numProcs_);
    for (int p = 0; p < numProcs_; ++p)
    {
      processorBuffers_[b][p].resize(blockSize_);
    }
  }

  // Allocate tree output buffers: band x tree x blockSize
  treeOutputBuffers_.resize(numBands_);
  for (int b = 0; b < numBands_; ++b)
  {
    treeOutputBuffers_[b].resize(numTrees_);
    for (int t = 0; t < numTrees_; ++t)
    {
      treeOutputBuffers_[b][t].resize(blockSize_);
    }
  }

  // Allocate tree connections: band x tree
  treeConnections_.resize(numBands_);
  for (int b = 0; b < numBands_; ++b)
  {
    treeConnections_[b].resize(numTrees_);
  }

  // Allocate inter-node connections: band x proc x band x proc
  interNodeConnections_.resize(numBands_);
  for (int b = 0; b < numBands_; ++b)
  {
    interNodeConnections_[b].resize(numProcs_);
    for (int p = 0; p < numProcs_; ++p)
    {
      interNodeConnections_[b][p].resize(numBands_);
      for (int tb = 0; tb < numBands_; ++tb)
      {
        interNodeConnections_[b][p][tb].resize(numProcs_);
      }
    }
  }
}

void DelayNodes::setInitialConnections()
{
  // Set tree connections to all true
  for (int b = 0; b < numBands_; ++b)
  {
    for (int t = 0; t < numTrees_; ++t)
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

void DelayNodes::processBlockMono (float **inBand, float **outBand, size_t blockSize)
{
  // Assume blockSize matches blockSize_
  for (size_t s = 0; s < blockSize; ++s)
  {
    // Clear tree outputs
    for (int b = 0; b < numBands_; ++b)
    {
      for (int t = 0; t < numTrees_; ++t)
      {
        treeOutputBuffers_[b][t][s] = 0.0f;
      }
    }

    // Process each band
    for (int b = 0; b < numBands_; ++b)
    {
      // For each processor in the band
      for (int p = 0; p < numProcs_; ++p)
      {
        float input = 0.0f;
        if (p == 0)
        {
          // First processor gets input from inBand
          input = inBand[b][s];
        }
        else
        {
          // Subsequent get from previous processor
          input = processorBuffers_[b][p - 1][s];
        }

        // Add inter-node inputs
        for (int tb = 0; tb < numBands_; ++tb)
        {
          for (int tp = 0; tp < numProcs_; ++tp)
          {
            if (interNodeConnections_[b][p][tb][tp] > 0.0f)
            {
              input += processorBuffers_[tb][tp][s] * interNodeConnections_[b][p][tb][tp];
            }
          }
        }

        // Process through delay proc
        processorBuffers_[b][p][s] = delayProcs_[b][p].process(input);

        // Add to tree outputs if connected
        for (int t = 0; t < numTrees_; ++t)
        {
          if (treeConnections_[b][t])
          {
            treeOutputBuffers_[b][t][s] += processorBuffers_[b][p][s];
          }
        }
      }
    }

    // Sum tree outputs to outBand
    for (int b = 0; b < numBands_; ++b)
    {
      outBand[b][s] = 0.0f;
      for (int t = 0; t < numTrees_; ++t)
      {
        outBand[b][s] += treeOutputBuffers_[b][t][s];
      }
      // Normalize
      outBand[b][s] /= numTrees_;
    }
  }
}
