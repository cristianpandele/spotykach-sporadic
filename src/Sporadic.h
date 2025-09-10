#pragma once

#include "common.h"
#include "daisy_seed.h"
#include "daisysp.h"
#include "Deck.h"
#include "hardware.h"
#include "DelayNetwork.h"
#include "EdgeTree.h"

using namespace daisy;

// Class for Sporadic deck implementation
class Sporadic : public Deck
{
  public:
    Sporadic (size_t sampleRate, size_t blockSize) : Deck(sampleRate, blockSize), edgeTree_{sampleRate, sampleRate} {}
    Sporadic ()  = delete;
    ~Sporadic () = default;

    void init () override;
    void updateAnalogControls (const AnalogControlFrame &c) override;
    void updateDigitalControls (const DigitalControlFrame &c) override;
    void getDigitalControls (DigitalControlFrame &c) override;
    void updateDisplayState () override;
    void processAudio(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t blockSize) override;
    // Getter for band frequencies
    void getBandFrequencies (std::vector<float> &frequencies) const;

  private:
    // Delay network for feedback and modulation
    DelayNetwork delayNetwork_;

    // EdgeTree for envelope following and volume modulation
    EdgeTree edgeTree_[kNumberChannelsStereo];

    // Internal working buffers (single block) to avoid per-callback allocations.
    float delayNetworkBuf_[kNumberChannelsStereo][kBlockSize]{};

    // Buffer for modulated input before input sculpt
    float modulatedInputBuf_[kNumberChannelsStereo][kBlockSize]{};

    // Envelope ring (visual + mixing gains source). 32 slots per ring.
    static constexpr uint8_t kNumLeds = spotykach::Hardware::kNumLedsPerRing;
    float envelopeRing_[kNumLeds]{};

    // Setters for mix, position and size (overloads point to Sporadic versions)
    void setPosition (float p, bool gritLatch = false);
    void setPosition (float p) override { setPosition(p, false); }
    void setSize (float s, bool gritLatch = false);
    void setSize (float s) override { setSize(s, false); }
    void setShape (float s, bool gritLatch = false);
    void setShape (float s) override { setShape(s, false); }
    void setPitch (float p, bool gritLatch = false);
    void setPitch (float p) override { setPitch(p, false); }
    void setSpoty (float s) override;

    // Prepare and set delay network parameters
    void setDelayNetworkParameters (float centerFreq, float stretch);

    void updateDiffusionRingState(DisplayState &view);

    // Draw the envelope ring to the current DisplayState using position_/size_/shape_.
    void updateFoldWindowState(DisplayState &view);

    ///////////
    NOCOPY (Sporadic);
};
