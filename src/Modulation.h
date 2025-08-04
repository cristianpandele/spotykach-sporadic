#pragma once
#include "common.h"
#include <daisysp.h>

// Sample and Hold oscillator wrapper
class SampleAndHold
{
  public:
    float Process ();
    void Init (float sampleRate);
    void SetAmp(float amp) { amp_ = amp; }
    void SetFreq(float freq) { freq_ = freq; }

  private:
    daisysp::SampleHold s_h_;
    float sampleRate_;
    float amp_ = 1.0f;
    float freq_ = 1.0f;
};

// Base class for modulation engines
class ModulationEngine
{
  public:
    static constexpr size_t kNumModTypes = 3;    // Number of modulation types per modulator

    enum ModType
    {
      ENV_FOLLOWER,
      S_H,
      SQUARE,
      SINE,
      SAW,
      MOD_TYPE_LAST
    };

    ModulationEngine (float sampleRate);
    virtual ~ModulationEngine () = default;

    virtual void setModType (ModType t) { currentModType = t; }

    void setAmplitude (float amp)
    {
      float amp_map = infrasonic::map(amp, 0.0f, 1.0f, -1.0f, 1.0f);
      for (int i = 0; i < 3; ++i)
      {
        osc_[i].SetAmp(amp_map);
      }
      s_h_.SetAmp(amp_map);
    }

    void setFrequency (float freq)
    {
      float freq_map = infrasonic::map(freq, 0.0f, 1.0f, 0.05f, 15.0f);
      for (int i = 0; i < 3; ++i)
      {
        osc_[i].SetFreq(freq_map);
      }
      s_h_.SetFreq(freq_map);
    }

  protected:
    float               sampleRate_;
    ModType             supportedModTypes[kNumModTypes] = {MOD_TYPE_LAST, MOD_TYPE_LAST, MOD_TYPE_LAST};
    ModType             currentModType        = MOD_TYPE_LAST;
    daisysp::Oscillator osc_[3];    // 0: Square, 1: Sine, 2: Saw
    SampleAndHold       s_h_;
};

// Modulator for each side, interprets ModType differently
class Modulator : public ModulationEngine
{
  public:
    Modulator (const ModType *modTypes, size_t numModTypes, float sampleRate);
    void setModType (ModType t) override;
    float process ();
};
