#pragma once

// #include "common.h"
#include "daisy_seed.h"
// #include "daisysp.h"
// #include "app.h"

// using namespace spotykach;
// using namespace spotykach_hwtest;
using namespace daisy;

// Base class for Effect implementation
class Effect
{
  public:
    enum EffectMode
    {
      OFF,
      MONO_LEFT,
      MONO_RIGHT,
      STEREO,
      MODE_LAST
    };

    Effect ()          = default;
    virtual ~Effect () = default;

    void         setMode (EffectMode mode);
    virtual void processAudio (AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size);

  private:
    EffectMode currentMode            = EffectMode::OFF;

    Effect (const Effect &)           = delete;
    Effect &operator=(const Effect &) = delete;
};
