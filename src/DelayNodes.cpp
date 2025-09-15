#include "DelayNodes.h"
#include "DelayProc.h"
#include <algorithm>

// Delay processors laid out as: channel x proc
static DSY_SDRAM_BSS std::array<std::array<DelayProc, kMaxNumDelayProcs>, kNumberChannelsStereo> delayProcs_;

void DelayNodes::init (float sampleRate, size_t blockSize, size_t numBands, size_t numProcs)
{
  sampleRate_ = sampleRate;
  numBands_   = std::clamp(numBands, static_cast<size_t>(1), kMaxNutrientBands);
  numProcs_   = std::clamp(numProcs, static_cast<size_t>(1), kMaxNumDelayProcs);
  blockSize_  = blockSize;
  interconnectionUpdateTimer.Init();
  allocateResources();
}

void DelayNodes::setStretch (float stretch)
{
  stretch_ = stretch;
  setDelayProcsParameters();
}

void DelayNodes::allocateResources ()
{
  for (uint8_t ch = 0; ch < kNumberChannelsStereo; ++ch)
  {
    for (size_t p = 0; p < numProcs_; ++p)
    {
      delayProcs_[ch][p].init(sampleRate_, DelayProc::kMaxDelaySamples);
    }
  }
  // Set default parameters
  setInitialConnections();
  // Initialize tree positions to full density by default
  numActiveTrees_ = std::max(static_cast<size_t>(1), numProcs_);
  updateTreePositions();
}

void DelayNodes::setDelayProcsParameters ()
{
  for (size_t p = 0; p < numProcs_; ++p)
  {
    float prevTreePos = (p == 0) ? 0.0f : treePositions_[p - 1];
    // Space out stretch according to tree position in [0,1]
    float perProcStretch = stretch_ * (treePositions_[p] - prevTreePos);
    for (uint8_t ch = 0; ch < kNumberChannelsStereo; ++ch)
    {
      delayProcs_[ch][p].setParameters(perProcStretch, 1.0f);
    }
  }
}

void DelayNodes::setInitialConnections ()
{
  // Initialize all to 0.
  for (size_t r = 0; r < kMaxNumDelayProcs; ++r)
  {
    std::fill(std::begin(interNodeConnections_[r]), std::end(interNodeConnections_[r]), 0.0f);
  }
  // Simple forward chain: p -> p+1 gets weight 1.0f
  for (size_t p = 0; p < kMaxNumDelayProcs - 1; ++p)
  {
    interNodeConnections_[p][p + 1] = 1.0f;
  }
}

void DelayNodes::setTreeDensity(float density)
{
  // Clamp to [0,1]
  float treeDensity_ = infrasonic::unitclamp(density);

  float newActiveTrees;
  // Map to [numProcs_, 1, numProcs_] following a U-shaped curve
  if (treeDensity_ < 0.5f)
  {
    newActiveTrees = infrasonic::map(treeDensity_, 0.0f, 0.5f, static_cast<float>(numProcs_ + 1), 0.0f);
  }
  else
  {
    newActiveTrees = infrasonic::map(treeDensity_, 0.5f, 1.0f, 0.0f, static_cast<float>(numProcs_ + 1));
  }
  newActiveTrees  = std::round<size_t>(newActiveTrees);
  newActiveTrees  = std::max(static_cast<size_t>(1), static_cast<size_t>(newActiveTrees));

  if (newActiveTrees == numActiveTrees_)
  {
    return;
  }

  numActiveTrees_ = newActiveTrees;

  // Update tree positions (this also updates delay times)
  updateTreePositions((treeDensity_ < 0.5f));    // Uniform if density < 0.5
  // Update the entanglement factor based on density
  if (treeDensity_ > 0.5f)
  {
    entanglement_ = infrasonic::map(treeDensity_, 0.5f, 1.0f, 0.0f, 1.0f);
  }
  else
  {
    entanglement_ = 0.0f;
  }

  // If density < 0.5, ignore inter-node connections (linear chain only)
  if (treeDensity_ < 0.5f)
  {
    ignoreMycelia_ = true;
  }
  else
  {
    ignoreMycelia_ = false;
  }
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

void DelayNodes::updateTreePositions(bool uniform)
{
  // Ensure last tree is at 1.0
  treePositions_[numActiveTrees_ - 1] = 1.0f;    // single tree at end of chain

  // Zero out the rest for cleanliness
  for (size_t i = numActiveTrees_; i < kMaxNumDelayProcs; ++i)
  {
    treePositions_[i] = 0.0f;
  }

  if (uniform)
  {
    for (size_t i = 0; i < numActiveTrees_ - 1; ++i)
    {

    }
  }

  const float step = 1.0f / static_cast<float>(numActiveTrees_);
  for (size_t i = 0; i < numActiveTrees_ - 1; ++i)
  {
    float variationFactor = 1.0f;
    if (!uniform)
    {
      // Random distribution of any remaining trees in [0,1]
      // 75% chance to have a variation of ±2.5% of the base delay time
      // 15% chance to have a variation of ±5% of the base delay time
      // 7.5% chance to have a variation of ±7.5% of the base delay time
      // 2.5% chance to have a variation of ±25% of the base delay time
      // Generate a random number between 0 and 1
      float randomValue = daisy::Random::GetFloat(0.0f, 1.0f);
      if (randomValue < 0.75f)
      {
        // 75% chance for ±2.5%
        variationFactor = 0.975f + daisy::Random::GetFloat(0.0f, 0.05f);
      }
      else if (randomValue < 0.90f)
      {
        // 15% chance for ±5%
        variationFactor = 0.95f + daisy::Random::GetFloat(0.0f, 0.1f);
      }
      else if (randomValue < 0.975f)
      {
        // 7.5% chance for ±7.5%
        variationFactor = 0.925f + daisy::Random::GetFloat(0.0f, 0.15f);
      }
      else
      {
        // 2.5% chance for ±25%
        variationFactor = 0.75f + daisy::Random::GetFloat(0.0f, 0.5f);
      }
    }

    // Uniform distribution of any remaining trees in [0,1]
    treePositions_[i] = step * (static_cast<float>(i) + 0.5f);
    // Apply variation factor
    treePositions_[i] *= variationFactor;
  }

  // Sort to ensure increasing order
  std::sort(treePositions_.begin(), treePositions_.begin() + numActiveTrees_ - 1);

  setDelayProcsParameters();    // Update delay times according to new tree positions
}

void DelayNodes::updateNodeInterconnections ()
{
  // Update existing connections by adding a small random delta scaled by
  // entanglement and the minimum age of the pair; occasionally create new
  // connections with probability proportional to entanglement and distance.

  // For now, use channel 0 for age queries
  for (size_t src = 0; src < kMaxNumDelayProcs; ++src)
  {
    for (size_t dst = 0; dst < kMaxNumDelayProcs; ++dst)
    {
      float &connectionStrength = interNodeConnections_[src][dst];
      float  ageSrc             = delayProcs_[0][src].getAge();
      float  ageDst             = delayProcs_[0][dst].getAge();
      float  pairMinAge         = std::min(ageSrc, ageDst);

      if (src == dst)
      {
        connectionStrength = 0.0f;
        continue;
      }

      // normalized entanglement in [0,1]
      float normEntanglement = entanglement_;

      if (connectionStrength > 0.0f)
      {
        // Existing connection: add a small delta that decays (and finally recedes) with age
        float pairEntanglementDelta =
          daisy::Random::GetFloat(0.0f, 0.1f) * normEntanglement * (0.5f - pairMinAge);
        pairEntanglementDelta *= connectionStrength;
        connectionStrength += pairEntanglementDelta;
      }
      else
      {
        if (interNodeConnections_[dst][src] > 0.0f)
        {
          // Avoid mutual connections
          connectionStrength = 0.0f;
          continue;
        }
        // Potential creation of new connection
        // Probability diminishes with distance between nodes
        size_t distance   = std::abs(static_cast<int>(dst) - static_cast<int>(src));
        float maxDist    = static_cast<float>(std::max<size_t>(1, numProcs_ - 1));
        float proximity  = 1.0f - (static_cast<float>(distance) / maxDist);    // 1 near neighbors, 0 far

        // Random chance to create a new connection
        float rnd        = daisy::Random::GetFloat(0.0f, 1.0f);
        float createProb = normEntanglement * (1.0f - pairMinAge);
        if (rnd < createProb)
        {
          // Create a new connection with strength scaled by entanglement, proximity, and age
          connectionStrength = daisy::Random::GetFloat(0.0f, 0.5f) / (1.0f + pairMinAge);
          connectionStrength *= normEntanglement;
          connectionStrength *= proximity;
        }
      }
    }
  }

  // Normalize per destination so that sum of incoming weights is just a bit under 1.0f
  // - this prevents runaway feedback in the common case of a loop
  // Note: this does not prevent feedback loops if the matrix contains cycles; that is left to the user to explore.
  for (size_t dst = 0; dst < kMaxNumDelayProcs; ++dst)
  {
    float sum = 0.0f;
    for (size_t src = 0; src < kMaxNumDelayProcs; ++src)
    {
      sum += interNodeConnections_[src][dst];
    }

    float age = delayProcs_[0][dst].getAge();
    if ((sum > 1.0f) ||     // Normalize only if sum > 1.0f (to prevent excessive feedback)
        (age > 0.5f))       // or if the pair is old enough and nodes start decaying
    {
      float inv = 1.0f / (sum + 0.05f); // +0.05f to avoid being at the edge of feedback instability
      for (size_t src = 0; src < kMaxNumDelayProcs; ++src)
      {
        interNodeConnections_[src][dst] *= inv;
      }
    }
  }
}

#ifdef DEBUG
// Getter for current inter-node connection matrix (numProcs x numProcs)
void DelayNodes::getNodeInterconnectionMatrix (std::vector<std::vector<float>> &matrix) const
{
  matrix.resize(numProcs_);
  for (auto &row : matrix)
  {
    row.resize(numProcs_);
  }
  for (size_t r = 0; r < numProcs_; ++r)
  {
    for (size_t c = 0; c < numProcs_; ++c)
    {
      matrix[r][c] = interNodeConnections_[r][c];
    }
  }
}
#endif

void DelayNodes::updateSidechainLevels (size_t ch)
{
  // For node p, compute sidechain as:
  //   sc[p] = sum_{src != p} interNodeConnections_[src][p+1] * outputLevel(src) - outputLevel(p)
  // For p == last, there is no p+1; set sc to 0.
  for (size_t p = 0; p < numProcs_; ++p)
  {
    float sc = 0.0f;
    if (p + 1 < numProcs_)
    {
      const size_t dst    = p + 1;
      float        inflow = 0.0f;
      for (size_t src = 0; src < numProcs_; ++src)
      {
        if (src == p)
          continue;
        float w = interNodeConnections_[src][dst];
        if (w > 0.0f)
        {
          inflow += w * delayProcs_[ch][src].outputLevel;
        }
      }
      sc = inflow - delayProcs_[ch][p].outputLevel;
    }
    sidechainLevels_[p] = daisysp::fclamp(sc, 0.0f, 1.0f);
  }

  // Push to processors
  for (size_t p = 0; p < numProcs_; ++p)
  {
    delayProcs_[ch][p].setSidechainLevel(sidechainLevels_[p]);
  }
}

void DelayNodes::processBlockMono (float **inBand, float **treeOutputs, size_t ch, size_t blockSize)
{
  // Routing-based processing:
  // For each block:
  // 1. Update the routing matrix.
  // 2. Update sidechain levels for each processor based on routing + activity.

  // For each sample:
  // 1. Sum all band inputs -> externalInput.
  // 2. For each processor p from 0..numProcs_-1 compute its input as:
  //      (p==0 ? externalInput : 0) + sum_{src < numProcs_} processorBuffers_[src] * interNodeConnections_[src][p]
  // 3. Process to produce processorBuffers_[p].
  // 4. After each processor runs for sample s, write processorBuffers_[p] to treeOutputs[p][s].

  // Update dynamic routing once per timer expiry interval
  if (interconnectionUpdateTimer.HasPassedMs(kNodeInterconnectionUpdateIntervalMs))
  {
    updateNodeInterconnections();
    interconnectionUpdateTimer.Restart();
  }

  // Update sidechain levels once per block
  updateSidechainLevels(ch);

  for (size_t s = 0; s < blockSize; ++s)
  {
    // External mixed input across bands
    float externalInput = 0.0f;
    for (size_t b = 0; b < numBands_; ++b)
    {
      externalInput += inBand[b][s];
    }

    // Compute each processor output
    for (size_t p = 0; p < numProcs_; ++p)
    {
      float inVal = (p == 0 ? externalInput : 0.0f);
      if (ignoreMycelia_)
      {
        // Sum contribution only from previous processor in chain
        if (p > 0)
        {
          inVal += processorBuffers_[p - 1];
        }
      }
      else
      {
        // Sum contributions all other processors per routing matrix
        for (size_t src = 0; src < numProcs_; ++src)
        {
          float w = interNodeConnections_[src][p];
          if (w != 0.0f)
          {
            inVal += processorBuffers_[src] * w;
          }
        }
      }
      processorBuffers_[p] = delayProcs_[ch][p].process(inVal);
    }

    // Write per-processor outputs for this sample
    for (size_t p = 0; p < numProcs_; ++p)
    {
      treeOutputs[p][s] = processorBuffers_[p];
    }
  }
}
