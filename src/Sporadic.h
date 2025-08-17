#pragma once

#include "common.h"
#include "daisy_seed.h"
#include "daisysp.h"
#include "Effect.h"
#include "hardware.h"

using namespace daisy;
using namespace spotykach_hwtest;

// Class for Sporadic effect implementation
class Sporadic : public Effect
{
  public:
    Sporadic (size_t sampleRate) : Effect(sampleRate) {}
    ~Sporadic () = default;

    void init () override;
    void updateAnalogControls (const AnalogControlFrame &c) override;
    void updateDigitalControls (const DigitalControlFrame &c) override;
    void getDigitalControls (DigitalControlFrame &c) override;
    void updateDisplayState () override;
    void processAudio(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t blockSize) override;

  private:
    Sporadic (const Sporadic &)           = delete;
    Sporadic &operator=(const Sporadic &) = delete;
};
