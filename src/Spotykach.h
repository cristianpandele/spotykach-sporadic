#pragma once

#include "Effect.h"
#include "common.h"
#include "daisy_seed.h"
#include "daisysp.h"
#include "hardware.h"

using namespace daisy;
using namespace daisysp;
using namespace spotykach_hwtest;

// Class for Spotykach looper implementation
class Spotykach : public Effect
{
  public:
    enum State
    {
      OFF,
      ECHO,
      RECORDING,
      LOOP_PLAYBACK
    };
    Spotykach (uint8_t side) : effectSide_(side) {}
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
    // Span structure for representing a range
    template<typename T>
    struct Span
    {
      T start;
      T end;
    };

    float readWindowStart;
    float readWindowEnd;

    // Read and write pointers for the looper buffer
    float readIx_  = 0;
    float writeIx_ = 0;
    // Speed of looper heads
    float speed_ = 1.0f;
    // Feedback amount for overdub
    float feedback_ = 0.0f;
    // Record flag
    bool record_ = false;

    // Spotykach effect state
    State state_ = OFF;
    // Current side being processed (0 or 1)
    uint8_t effectSide_ = 0;

    bool isChannelActive (size_t ch) const;

    void setMix (float m, bool altLatch);
    void setPitch (float s) override;
    void setPlay (bool p) override;
    void setReverse (bool r) override;
    void setSize (float p) override;
    void setFeedback (float fb) { feedback_ = std::clamp(fb, 0.0f, 0.99f); }
    void setAltPlay (bool r);
    void setSpotyPlay (bool s);

    void updateEchoReadIndexPosition (float p);
    void updateIndex (float &index, float increment, Span<float> window);

    void  setState (State s) { state_ = s; }
    State getState () const { return state_; }

    // Override the EffectMode enum to define specific modes for Spotykach
    enum EffectMode
    {
      REEL  = 0,
      SLICE = 1,
      DRIFT = 2,
      MODE_LAST
    };

    Spotykach (const Spotykach &)           = delete;
    Spotykach &operator=(const Spotykach &) = delete;
};
