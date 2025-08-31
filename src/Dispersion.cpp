#include "Dispersion.h"

void Dispersion::init (float sampleRate)
{
  // Initialize defaults that were previously provided via in-class init
  fs                 = sampleRate;
  inDispersionAmount = 0.0f;
  inAllpassFreq      = 1000.0f;
  a[0]               = 0.0f;
  a[1]               = 0.0f;
  y1                 = 0.0f;
  reset();
  setParameters({.dispersionAmount = inDispersionAmount, .allpassFreq = inAllpassFreq});
}

void Dispersion::reset()
{
  std::fill(stageFb.begin(), stageFb.end(), 0.0f);
}

float Dispersion::processSample(float x)
{
  auto numStages = inDispersionAmount * maxNumStages;
  const auto numStagesInt = static_cast<size_t>(numStages);
  float y = x;

  // process integer stages
  for (size_t stage = 0; stage < numStagesInt; ++stage)
      y = processStage(y, stage);

  // process fractional stage
  float stageFrac = numStages - numStagesInt;
  y = stageFrac * processStage(y, numStagesInt) + (1.0f - stageFrac) * y;

  // Save the allpass path output for the next call
  y1 = y;

  // Add the direct path + allpass path
  y = 0.5f * (x + y);
  // Apply the feedback path
  y = y - 0.4f * y1;

  return y;
}

float Dispersion::processStage(float x, size_t stage)
{
  float y = a[1] * x + stageFb[stage];
  stageFb[stage] = x * a[0] - y * a[1];
  return y;
}

void Dispersion::setParameters(const Parameters &params)
{
  auto dispAmtVal = params.dispersionAmount * maxNumStages;
  auto dispAmtChanged = (std::abs(inDispersionAmount - dispAmtVal) / (dispAmtVal + 1e-6f) > 0.01f);
  auto allpassFreqChanged = (std::abs(inAllpassFreq - params.allpassFreq) / (params.allpassFreq + 1e-6f) > 0.01f);

  if (dispAmtChanged)
  {
      inDispersionAmount = dispAmtVal;
  }

  if (allpassFreqChanged)
  {
      inAllpassFreq = params.allpassFreq;
      updateAllpassCoefficients();
  }
}

void Dispersion::updateAllpassCoefficients()
{
  a[0] = 1.0f;

  float wT = 2.0f * M_PI * inAllpassFreq / fs;
  a[1] = -1.0f * wT;
}
