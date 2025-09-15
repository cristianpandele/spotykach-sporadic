#pragma once
#include "common.h"
#include "constants.h"
#include <daisysp.h>

using namespace ::spotykach;
// Sample and Hold oscillator wrapper
class SampleAndHold
{
  public:
    SampleAndHold (float sampleRate);

    float process ();

    void  setAmplitude (float amp) { amp_ = amp; }

    void  setFrequency (float freq) { freq_ = freq; }

  private:
    float               sampleRate_ = 0.0f;
    float               amp_    = 1.0f;
    // Sample and Hold specific parameters
    float               freq_  = 0.0f;     // Frequency of the sample and hold
    float               phase_  = 0.0f;    // Phase of the sample and hold
    float               cvOutput_;
};

// Envelope follower (attack/release one-pole)
class EnvelopeFollower
{
  public:
    EnvelopeFollower (float sampleRate);
    EnvelopeFollower () = default;

    void setAttackMs (float ms);
    float getAttackMs () const { return attackMs_; }
    float getAttackCoefficient () const { return atkCoef_; }

    void setReleaseMs (float ms);
    float getReleaseMs () const { return releaseMs_; }
    float getReleaseCoefficient () const { return relCoef_; }

    void init (float sampleRate);

    void setAmplitude (float a) { amp_ = a; }

    void setFrequency (float f);

    // Compensate the gain of the envelope follower
    float gainCompensation () const;

    // Process a single sample
    float process (float x);

    // Read current output without advancing
    float output (bool attenuate = false) const;

  private:
    float sampleRate_;
    float attackMs_;
    float releaseMs_;
    float atkCoef_;
    float relCoef_;
    float env_;
    float amp_;
};

// Base class for modulation engines
class ModulationEngine
{
  public:
    static constexpr size_t kNumModsPerSide = 3;    // Number of modulation types per modulator
    static constexpr size_t kNumModWaves    = 5;    // Number of modulation waveforms (square, sine, saw, sample and hold, envelope follower)

    enum ModType
    {
      ENV_FOLLOWER,
      S_H,
      SQUARE,
      SINE,
      RAMP,
      MOD_TYPE_LAST
    };

    static float constexpr kMinFreq = 0.05f; // Minimum modulation frequency
    static float constexpr kMaxFreq = 15.0f; // Maximum modulation frequency

    ModulationEngine (float sampleRate);
    ModulationEngine ()          = delete;
    virtual ~ModulationEngine () = default;

    virtual void setModType (ModType t) { currentModType = t; }

    void setAmplitude (float amp);

    void setFrequency (float freq, bool altFactor);

    size_t mapWaveformsToOscIndex (ModulationEngine::ModType waveform);

    // Feed a block of input samples into the envelope follower
    void setEnvelopeInput (const float *x, size_t blockSize);

  protected:
    float               sampleRate_;
    ModType             supportedModTypes[kNumModsPerSide] = {MOD_TYPE_LAST};
    ModType             currentModType                     = supportedModTypes[0];
    daisysp::Oscillator osc_[kNumModWaves - 2];    // 0: Square, 1: Sine, 2: Ramp. Sample and Hold and Env Follower are separate classes
    SampleAndHold       s_h_;                      // Sample and Hold oscillator
    EnvelopeFollower    env_;                      // Envelope follower

  private:
    size_t waveformList[kNumModWaves - 2] = {
      daisysp::Oscillator::WAVE_SQUARE,
      daisysp::Oscillator::WAVE_SIN,
      daisysp::Oscillator::WAVE_RAMP
    };

    ///////////
    NOCOPY (ModulationEngine);
};

// Modulator for each side, interprets ModType differently
class Modulator : public ModulationEngine
{
  public:
    Modulator (const ModType *modTypes, size_t numModTypes, float sampleRate);
    void setModType (ModType t) override;
    float process ();

    float getAttackMs () const { return env_.getAttackMs(); }
    float getReleaseMs () const { return env_.getReleaseMs(); }

    float getAttackCoefficient () const { return env_.getAttackCoefficient(); }

    float getReleaseCoefficient () const { return env_.getReleaseCoefficient(); }

  private:
    float sampleRate_;

    ///////////
    NOCOPY (Modulator);
};
