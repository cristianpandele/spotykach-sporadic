#pragma once
#include "common.h"
#include <daisysp.h>

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

// Base class for modulation engines
class ModulationEngine
{
  public:
    static constexpr size_t kNumModsPerSide = 3;    // Number of modulation types per modulator
    static constexpr size_t kNumModWaves    = 4;    // Number of modulation waveforms (square, sine, saw, sample and hold)

    enum ModType
    {
      ENV_FOLLOWER,
      S_H,
      SQUARE,
      SINE,
      RAMP,
      MOD_TYPE_LAST
    };

    ModulationEngine (float sampleRate);
    ModulationEngine ()          = delete;
    virtual ~ModulationEngine () = default;

    virtual void setModType (ModType t) { currentModType = t; }

    void setAmplitude (float amp);

    void setFrequency (float freq, bool altFactor);

    size_t mapWaveformsToOscIndex (ModulationEngine::ModType waveform);

  protected:
    float               sampleRate_;
    ModType             supportedModTypes[kNumModsPerSide] = {MOD_TYPE_LAST};
    ModType             currentModType                     = supportedModTypes[0];
    daisysp::Oscillator osc_[kNumModWaves - 1];    // 0: Square, 1: Sine, 2: Ramp. Sample and Hold is separate class
    SampleAndHold       s_h_;                      // Sample and Hold oscillator
};

// Modulator for each side, interprets ModType differently
class Modulator : public ModulationEngine
{
  public:
    Modulator (const ModType *modTypes, size_t numModTypes, float sampleRate);
    void setModType (ModType t) override;
    float process ();
  private:
    float sampleRate_;
};
