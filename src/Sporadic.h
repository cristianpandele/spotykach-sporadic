#pragma once

#include "common.h"
#include "daisy_seed.h"
#include "daisysp.h"
#include "Effect.h"
#include "hardware.h"
#include "InputSculpt.h"

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
    // Input sculpting bandpass filter
    InputSculpt inputSculpt_;

    // Setters for mix, position and size (overloads point to Sporadic versions)
    void setMix (float m, bool fluxLatch = false);
    void setMix (float m) override { setMix(m, false); }
    void setPosition (float p, bool fluxLatch = false);
    void setPosition (float p) override { setPosition(p, false); }
    void setSize (float s, bool fluxLatch = false);
    void setSize (float s) override { setSize(s, false); }

    uint8_t freqToLed (float f, uint8_t numLeds, float fMin, float fMax);
    void ledBrightnessTriangleGradient (uint8_t spanSize, float minBrightness, float maxBrightness, float *gradValues);
    void populateLedRing (Effect::RingSpan &ringSpan,
                          uint8_t           ringSize,
                          LedRgbBrightness  colorBright,
                          uint8_t           start,
                          uint8_t           spanSize,
                          bool              gradient = false);
    void updateFluxDisplayState (DisplayState& view);

    Sporadic (const Sporadic &)           = delete;
    Sporadic &operator=(const Sporadic &) = delete;
};
