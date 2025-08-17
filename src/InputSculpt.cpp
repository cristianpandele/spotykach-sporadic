#include "InputSculpt.h"

void InputSculpt::init (float sampleRate)
{
  svf_.Init(sampleRate);
  updateFilter(centerFreq_, q_);
}

void InputSculpt::setFreq (float f)
{
  f                    = daisysp::fclamp(f, 0.0f, 1.0f);
  constexpr float fMin = 50.0f;
  constexpr float fMax = 18000.0f;
  centerFreq_ = daisysp::fmap(f, fMin, fMax, Mapping::LOG);
  updateFilter(centerFreq_, q_);
}

void InputSculpt::setWidth (float w)
{
  w = daisysp::fclamp(w, 0.0f, 1.0f);
  // Map w=0 narrow (high Q), w=1 wide (low Q)
  q_ = daisysp::fmap(1.0f - w, 0.0f, 1.0f, Mapping::EXP);

  updateFilter(centerFreq_, q_);
}

void InputSculpt::updateFilter (float freq, float q)
{
  svf_.SetFreq(freq);
  svf_.SetRes(q);
}

float InputSculpt::processSample (float in)
{
  svf_.Process(in);
  return svf_.Band();
}