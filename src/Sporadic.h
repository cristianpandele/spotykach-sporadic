#pragma once

#include "common.h"
#include "daisy_seed.h"
#include "daisysp.h"
#include "Deck.h"
#include "hardware.h"
#include "DelayNetwork.h"
#include "EdgeTree.h"

using namespace daisy;
using namespace spotykach_hwtest;

// Class for Sporadic deck implementation
class Sporadic : public Deck
{
  public:
    Sporadic (size_t sampleRate, size_t blockSize) : Deck(sampleRate, blockSize), edgeTree_(sampleRate) { init(); }
    Sporadic ()  = delete;
    ~Sporadic () = default;

    void init () override;
    void updateAnalogControls (const AnalogControlFrame &c) override;
    void updateDigitalControls (const DigitalControlFrame &c) override;
    void getDigitalControls (DigitalControlFrame &c) override;
    void updateDisplayState () override;
    void processAudio(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t blockSize) override;
#if DEBUG
    // Getter for band frequencies
    void getBandFrequencies (std::vector<float> &frequencies) const;
#endif

  private:
    // Delay network for feedback and modulation
    DelayNetwork delayNetwork_;

    // EdgeTree for envelope following and volume modulation
    EdgeTree edgeTree_;

    // Constants
    static constexpr uint8_t kNumBands     = 4;    // Number of bands for diffusion

    // Internal working buffers (single block) to avoid per-callback allocations.
    float delayNetworkBuf_[kNumberChannelsStereo][kBlockSize]{};

    // Buffer for modulated input before input sculpt
    float modulatedInputBuf_[kNumberChannelsStereo][kBlockSize]{};

    // Setters for mix, position and size (overloads point to Sporadic versions)
    void setMix (float m, bool gritLatch = false);
    void setMix (float m) override { setMix(m, false); }
    void setPosition (float p, bool gritLatch = false);
    void setPosition (float p) override { setPosition(p, false); }
    void setSize (float s, bool gritLatch = false);
    void setSize (float s) override { setSize(s, false); }
    void setShape (float s, bool gritLatch = false);
    void setShape (float s) override { setShape(s, false); }

    Sporadic (const Sporadic &)           = delete;
    Sporadic &operator=(const Sporadic &) = delete;
};
