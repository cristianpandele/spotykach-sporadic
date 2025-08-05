#pragma once

#include "daisy_seed.h"
using namespace daisy;

// Base class for Effect implementation
class Effect
{
  public:
    enum ChannelConfig
    {
      OFF,
      MONO_LEFT,
      MONO_RIGHT,
      STEREO,
      CH_CONFIG_LAST
    };

    enum EffectMode
    {
      MODE_1,
      MODE_2,
      MODE_3,
      MODE_LAST
    };

    Effect ()          = default;
    virtual ~Effect () = default;

    void setChannelConfig (ChannelConfig mode);

    virtual void setMode (EffectMode m);

    virtual void setMix (float m) { mix = m; }

    virtual void setPitch (float p) { pitch = p; }

    virtual void setPosition (float p) { position = p; }

    virtual void setSize (float s) { size = s; }

    virtual void setShape (float s) { shape = s; }

    virtual void processAudio (AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size);

  protected:
    // Channel configuration for the effect
    ChannelConfig channelConfig = ChannelConfig::OFF;

    // Mix control
    float mix = 0.5f;

    // V.Oct pitch control
    float pitch = 0.0f;

    // Position control
    float position = 0.0f;

    // Size control
    float size = 0.0f;

    // Shape control
    float shape = 0.0f;

    // Current effect mode
    EffectMode mode = MODE_1;

  private:
    Effect (const Effect &)           = delete;
    Effect &operator=(const Effect &) = delete;
};
