#include "Modulation.h"

ModulationEngine::ModulationEngine (float sampleRate) : sampleRate_(sampleRate)
{
  for (int i = 0; i < 3; ++i)
  {
    osc_[i].Init(sampleRate);
    osc_[i].SetAmp(1.0f);
    osc_[i].SetFreq(1.0f);
  }
  osc_[0].SetWaveform(daisysp::Oscillator::WAVE_POLYBLEP_SQUARE);
  osc_[1].SetWaveform(daisysp::Oscillator::WAVE_SIN);
  osc_[2].SetWaveform(daisysp::Oscillator::WAVE_POLYBLEP_SAW);
  s_h_.Init(sampleRate);
  s_h_.SetAmp(1.0f);
  s_h_.SetFreq(1.0f);
}

Modulator::Modulator(const ModType *modTypes, size_t numModTypes, float sampleRate) : ModulationEngine(sampleRate)
{
  for (size_t i = 0; i < numModTypes && i < kNumModTypes; ++i)
  {
    supportedModTypes[i] = modTypes[i];
  }
}

void Modulator::setModType (ModType t)
{
  for (size_t i = 0; i < kNumModTypes; ++i)
  {
    if (supportedModTypes[i] == t)
    {
      // If in the list of modulation types, set it
      currentModType = t;
      break;
    }
  }
}

float Modulator::process ()
{
  // Process all oscillators
  for (size_t i = 0; i < kNumModTypes; ++i)
  {
    osc_[i].Process();
  }
  s_h_.Process();

  // Return the output based on the current modulation type
  switch (currentModType)
  {
    case S_H:
      return s_h_.Process();
    case SQUARE:
      return osc_[0].Process();
    case SINE:
      return osc_[1].Process();
    case SAW:
      return osc_[2].Process();
    default:
      return 0.0f;
  }
}

float SampleAndHold::Process()
{
  float noise = ((rand() / (float)RAND_MAX) * 2.0f - 1.0f);
  noise       = infrasonic::map(noise, 0.0f, 1.0f, -1.0f, 1.0f);
  noise      *= amp_;
  return s_h_.Process(true, noise, daisysp::SampleHold::MODE_SAMPLE_HOLD);
}