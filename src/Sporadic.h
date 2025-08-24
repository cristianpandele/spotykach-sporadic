#pragma once

#include "common.h"
#include "daisy_seed.h"
#include "daisysp.h"
#include "Deck.h"
#include "hardware.h"
#include "InputSculpt.h"
#include "DelayNetwork.h"

using namespace daisy;
using namespace spotykach_hwtest;

// Class for Sporadic deck implementation
class Sporadic : public Deck
{
  public:
    Sporadic (size_t sampleRate) : Deck(sampleRate) {}
    ~Sporadic () = default;

    void init () override;
    void updateAnalogControls (const AnalogControlFrame &c) override;
    void updateDigitalControls (const DigitalControlFrame &c) override;
    void getDigitalControls (DigitalControlFrame &c) override;
    void updateDisplayState () override;
    void processAudio(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t blockSize) override;

 #if DEBUG
    void getBandFrequencies (std::vector<float> &frequencies) const;
#endif

  private:
    // Input sculpting bandpass filter
    InputSculpt  inputSculpt_;
    // Delay network for feedback and modulation
    DelayNetwork delayNetwork_;

    // Constants
    static constexpr uint8_t kNumBands = 4; // Number of bands for diffusion

    // Internal working buffers (single block) to avoid per-callback allocations.
    float inputSculptBuf_[kNumberChannelsStereo][kBlockSize] {};
    float delayNetworkBuf_[kNumberChannelsStereo][kBlockSize] {};

    // Setters for mix, position and size (overloads point to Sporadic versions)
    void setMix (float m, bool fluxLatch = false);
    void setMix (float m) override { setMix(m, false); }
    void setPosition (float p, bool fluxLatch = false);
    void setPosition (float p) override { setPosition(p, false); }
    void setSize (float s, bool fluxLatch = false);
    void setSize (float s) override { setSize(s, false); }

    uint8_t freqToLed (float f, uint8_t numLeds, float fMin, float fMax);
    void ledBrightnessTriangleGradient (uint8_t spanSize, float minBrightness, float maxBrightness, float *gradValues);
    void    populateLedRing (Deck::RingSpan  &ringSpan,
                             uint8_t          ringSize,
                             LedRgbBrightness colorBright,
                             uint8_t          start,
                             uint8_t          spanSize,
                             bool             gradient = false);

    void updateBandpassDisplay (const uint8_t numLeds, uint8_t &start, uint8_t &end);
    void updateFluxDisplayState (DisplayState& view);

    Sporadic (const Sporadic &)           = delete;
    Sporadic &operator=(const Sporadic &) = delete;
};
