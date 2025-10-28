#pragma once

#include "Deck.h"
#include "common.h"
#include "daisy_seed.h"
#include "daisysp.h"
#include "hardware.h"

using namespace daisy;
using namespace daisysp;
using namespace spotykach_hwtest;

// Class for Spotykach looper implementation
class Spotykach : public Deck
{
  public:
    Spotykach (size_t sampleRate, size_t blockSize) : Deck(sampleRate, blockSize) { }
    ~Spotykach () = default;

    void init ();

    void updateAnalogControls (const AnalogControlFrame &c) override;
    void updateDigitalControls (const DigitalControlFrame &c) override;
    void getDigitalControls(DigitalControlFrame &c) override;

    void updateDisplayState () override;

    void processAudio(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t blockSize) override;

#if DEBUG
    float getPitch () const
    {
      return speed_;
    }

    float getSize () const
    {
      return size_;
    }

    float getPosition () const
    {
      return position_;
    }

    float getReadIx () const
    {
      return readIx_;
    }

    float getWriteIx () const
    {
      return writeIx_;
    }
#endif // DEBUG

  private:
    enum State
    {
      OFF,
      ECHO,
      RECORDING,
      LOOP_PLAYBACK
    };

    // Span structure for representing a range
    template<typename T>
    struct Span
    {
      T start;
      T end;
    };

    // Override the DeckMode enum to define specific modes for Spotykach
    enum DeckMode
    {
      REEL  = 0,
      SLICE = 1,
      DRIFT = 2,
      MODE_LAST
    };

    // Read and write pointers for the looper buffer
    float readIx_  = 0;
    float writeIx_ = 0;
    // Speed of looper heads
    float speed_ = 1.0f;
    // Feedback amount for overdub
    float feedback_ = 0.0f;
    // Record flag
    bool record_ = false;
    // Safe to play flag for ECHO mode
    bool safeToPlay = false;

    // Spotykach deck state
    State state_ = OFF;

    // Envelope generators for window shapes
    daisysp::Adsr envSquare_;
    daisysp::Adsr envFall_;
    daisysp::Adsr envTri_;
    daisysp::Adsr envRise_;
    // Phase tracking for wrap/retrigger detection
    float prevReadIx_ = 0.0f;
    float windowSamples_ = 0.0f;
    float prevWindowSamples_ = 0.0f;
    uint32_t envSampleCounter_ = 0;

    // Setters for mix, position and size (overloads point to Spotykach versions)
    void setMix (float m, bool altLatch = false);
    void setMix (float m) override { setMix(m, false); }
    void setFeedback (float fb) { feedback_ = std::clamp(fb, 0.0f, 0.99f); }
    void setPosition (float p, bool gritLatch = false);
    void setPosition (float p) override { setPosition(p, false); }
    void setSize (float s, bool gritLatch = false);
    void setSize (float s) override { setSize(s, false); }
    void setShape (float s, bool gritLatch = false);
    void setShape (float s) override { setShape(s, false); }
    void setPitch (float p, bool gritLatch = false);
    void setPitch (float p) override { setPitch(p, false); }

    void setPlay (bool p) override;
    void setReverse (bool r) override;
    void setAltPlay (bool r);
    void setSpotyPlay (bool s);

    void updateLooperDisplayState (DisplayState &view);

    void updateIndex (float &index, float increment, Span<float> window);
    // Retrigger envelopes when the read index wraps around
    void retriggerEnvelopesOnSpanWrap (float prevReadIx, Span<float> readSpan);

    // Helper to process audio samples
    void processAudioSample (AudioHandle::InputBuffer  in,
                             AudioHandle::OutputBuffer out,
                             size_t                    sample,
                             bool                      applyEnvelope = false,
                             bool                      record        = false);

    void initEnvelopes(float sampleRate);
    void retriggerEnvelopes (bool hard);
    // Helper to configure ADSR lengths per window
    void configureEnvelopeLength (float windowLenSamples);
    // Compute blended envelope value from shape_ in [0,1]
    float processEnvelope (bool gate);

    void  setState (State s) { state_ = s; }
    State getState () const { return state_; }

    NOCOPY (Spotykach);
};
