#include "Modulation.h"

ModulationEngine::ModulationEngine (float sampleRate) : sampleRate_(sampleRate), s_h_(sampleRate), env_(sampleRate)
{
  for (size_t i = 0; i < kNumModWaves - 2; ++i)
  {
    osc_[i].Init(sampleRate);
    osc_[i].SetAmp(1.0f);
    osc_[i].SetFreq(1.0f);
    osc_[i].SetWaveform(waveformList[i]);
  }
}

void ModulationEngine::setAmplitude (float amp)
{
  for (size_t i = 0; i < kNumModWaves - 2; ++i)
  {
    osc_[i].SetAmp(amp);
  }
  s_h_.setAmplitude(amp);
  env_.setAmplitude(amp);
}

void ModulationEngine::setFrequency (float freq, bool useAltFactor)
{
  // Map the frequency from (0,1) to (0.05, 15.0)
  float freqMap = infrasonic::map(freq, 0.0f, 1.0f, kMinFreq, kMaxFreq);

  // The envelope follower does not respond to the high range
  env_.setFrequency(freqMap);

  // If using the alt factor, multiply the frequency by 10
  if (useAltFactor)
  {
    freqMap *= 10.0f;
  }

  // Set the frequency for all oscillators
  for (size_t i = 0; i < kNumModWaves - 2; ++i)
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
    case ENV_FOLLOWER:
    {
      return 4;
    }
    default:
    {
      return -1; // Invalid type
    }
  }
}

void ModulationEngine::setEnvelopeInput (const float *x, size_t blockSize)
{
  for (size_t n = 0; n < blockSize; ++n)
  {
    env_.process(x[n]);
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
  for (size_t i = 0; i < kNumModWaves - 2; ++i)
  {
    oscOut[i] = osc_[i].Process();
  }
  oscOut[3] = s_h_.process();
  oscOut[4] = env_.output(true);

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

EnvelopeFollower::EnvelopeFollower (float sampleRate)
{
  sampleRate_ = sampleRate;
  setAttackMs(20.0f);
  setReleaseMs(120.0f);
}

void EnvelopeFollower::setAttackMs (float ms)
{
  attackMs_ = std::max(0.1f, ms);
  atkCoef_  = 1000.0f / (attackMs_ * sampleRate_);
  atkCoef_  = daisysp::fclamp(atkCoef_, 1e-6f, 1e-2f);
}

void EnvelopeFollower::setReleaseMs (float ms)
{
  releaseMs_ = std::max(0.1f, ms);
  relCoef_   = 1000.0f / (releaseMs_ * sampleRate_);
  relCoef_   = daisysp::fclamp(relCoef_, 1e-6f, 1e-2f);
}

void EnvelopeFollower::setFrequency (float f)
{
  // Map modulation frequency to env follower time constants.
  // Fastest at towards kMinFreq -> short attack/ long release, slowest at kMaxFreq -> long attack / short release times.
  // Use a musically useful range, e.g., attack 2-100ms, release 10-500ms.
  using namespace daisysp;
  float norm = infrasonic::map(f, ModulationEngine::kMinFreq, ModulationEngine::kMaxFreq, 0.0f, 1.0f);
  float atkMs = fmap(norm, kMinAttackTime, kMaxAttackTime, Mapping::LINEAR);
  float relMs = fmap(1.0f - norm, kMinReleaseTime, kMaxReleaseTime, Mapping::LINEAR);
  setAttackMs(atkMs);
  setReleaseMs(relMs);
}

float EnvelopeFollower::gainCompensation () const
{
  float gainCompNorm = infrasonic::map(attackMs_, kMinAttackTime, kMaxAttackTime, 0.0f, 1.0f);
  return daisysp::fmap(gainCompNorm, 4.0f, 75.0f, daisysp::Mapping::LINEAR);
}

// Process a single sample
float EnvelopeFollower::process (float x)
{
  x = std::fabs(x);

  // Attack when rising, release when falling
  if (x > env_)
  {
    env_ += atkCoef_ * (x - env_);
  }
  else
  {
    env_ -= relCoef_ * (env_);
  }

  // Apply gain compensation
  float out = env_ * gainCompensation();

  // Scale and clamp to [0,1]
  out = infrasonic::unitclamp(out * amp_);
  return out;
}


// Read current output without advancing
float EnvelopeFollower::output (bool attenuate) const
{
  // Gain compensation and clamp to [0,1]
  float out = infrasonic::unitclamp(env_ * gainCompensation());

  if (attenuate)
  {
    out *= amp_;   // Apply attenuation
  }
  return out;
}