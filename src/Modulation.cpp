#include "Modulation.h"

ModulationEngine::ModulationEngine (float sampleRate) : sampleRate_(sampleRate), s_h_(sampleRate)
{
  for (size_t i = 0; i < kNumModWaves - 1; ++i)
  {
    osc_[i].Init(sampleRate);
    osc_[i].SetAmp(1.0f);
    osc_[i].SetFreq(1.0f);
  }
  osc_[0].SetWaveform(daisysp::Oscillator::WAVE_SQUARE);
  osc_[1].SetWaveform(daisysp::Oscillator::WAVE_SIN);
  osc_[2].SetWaveform(daisysp::Oscillator::WAVE_RAMP);

  // final oscillator is of type SampleAndHold
  s_h_ = SampleAndHold(sampleRate);
}

void ModulationEngine::setAmplitude (float amp)
{
  for (size_t i = 0; i < kNumModWaves - 1; ++i)
  {
    osc_[i].SetAmp(amp);
  }
  s_h_.setAmplitude(amp);
}

void ModulationEngine::setFrequency (float freq, bool useAltFactor)
{
  // Map the frequency from (0,1) to (0.05, 15.0)
  float freqMap = infrasonic::map(freq, 0.0f, 1.0f, kMinFreq, kMaxFreq);

  // If using the alt factor, multiply the frequency by 10
  if (useAltFactor)
  {
    freqMap *= 10.0f;
  }

  // Set the frequency for all oscillators
  for (size_t i = 0; i < kNumModWaves - 1; ++i)
  {
    osc_[i].SetFreq(freqMap);
  }

  // Set the frequency for Sample and Hold oscillator
  s_h_.setFrequency(freqMap);
}

// Map the current modulation type to the corresponding oscillator output
size_t ModulationEngine::mapWaveformsToOscIndex (ModulationEngine::ModType waveform)
{
  switch(waveform)
  {
    case SQUARE:
    {
      return 0;
    }
    case SINE:
    {
      return 1;
    }
    case RAMP:
    {
      return 2;
    }
    case S_H:
    {
      return 3;
    }
    default:
    {
      return -1; // Invalid type
    }
  }
}

Modulator::Modulator (const ModType *modTypes, size_t numModTypes, float sampleRate)
  : ModulationEngine(sampleRate)
{
  for (size_t i = 0; i < numModTypes && i < kNumModsPerSide; ++i)
  {
    supportedModTypes[i] = modTypes[i];
  }
}

void Modulator::setModType (ModType t)
{
  for (size_t i = 0; i < kNumModsPerSide; ++i)
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
  float oscOut[kNumModWaves] = {0.0f};

  // Process all oscillators
  for (size_t i = 0; i < kNumModWaves - 1; ++i)
  {
    oscOut[i] = osc_[i].Process();
  }
  oscOut[3] = s_h_.process();
;

  // Return the output based on the current modulation type
  // -- Map the current modulation type to the corresponding oscillator output
  size_t oscIx = mapWaveformsToOscIndex(currentModType);

  // -- Check if the moodulation type is part of the supported types
  bool isValidType = false;
  for (size_t i = 0; i < kNumModsPerSide; ++i)
  {
    if (supportedModTypes[i] == currentModType)
    {
      // If in the list of modulation types, set the flag
      isValidType = true;
      break;
    }
  }

  if (isValidType)
  {
    // -- Return the output based on the current modulation type
    if (oscIx >= 0 && oscIx < kNumModWaves)
    {
      return oscOut[oscIx];
    }
  }
  return 0.0f;
}

SampleAndHold::SampleAndHold (float sampleRate)
{
  amp_        = 1.0f;
  freq_       = 1.0f;    // Frequency of the sample and hold
  phase_      = 0.0f;    // Phase of the sample and hold
  cvOutput_   = 0.0f;    // Initialize CV output
  sampleRate_ = sampleRate;
}

float SampleAndHold::process ()
{
  // Update the phase
  phase_ += 1.0f / sampleRate_;
  if (phase_ >= 1.0f / freq_)
  {
    // Generate a random value between 0 and 1
    cvOutput_ = ((float)rand() / (float)RAND_MAX);
    // Reset the phase and toggle the trigger
    phase_ -= 1.0f / freq_;
    // Apply the amplitude to the noise
    cvOutput_ *= amp_;
  }
  return cvOutput_;
}
