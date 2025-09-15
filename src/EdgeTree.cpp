#include "EdgeTree.h"

void EdgeTree::init (float sampleRate)
{
  envFollower_ = EnvelopeFollower(sampleRate);
  envFollower_.setAmplitude(1.0f);    // Full modulation range
}

// Set tree size
void EdgeTree::setTreeSize (float size)
{
  // Map size from (0,1) to (20ms, 1200ms)
  float attackTime = infrasonic::map(size, 0.0f, 1.0f, kMinAttackTime, kMaxAttackTime);
  envFollower_.setAttackMs(attackTime);
  float releaseTime = infrasonic::map(1.0f - size, 0.0f, 1.0f, kMinReleaseTime, kMaxReleaseTime);
  envFollower_.setReleaseMs(releaseTime);
}

// Process a single input sample: apply envelope following and modulate volume
float EdgeTree::process (float input)
{
  envelope_ = envFollower_.process(input);
  return input * envelope_;    // Modulate volume by envelope
}

void EdgeTree::processBlockMono (float *input, float *output, size_t blockSize)
{
  for (size_t n = 0; n < blockSize; ++n)
  {
    output[n] = process(input[n]);
  }
}
