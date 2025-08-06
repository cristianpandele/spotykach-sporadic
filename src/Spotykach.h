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
    Spotykach (uint8_t side) : effectSide_(side) {}
    ~Spotykach () = default;

    void init ();

    void processAudio (AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t blockSize) override;

    void setPitch (float s) override;

    void setPlay (bool p) override;

    void setReverse (bool r) override;

#if DEBUG
    float getPitch () const
    {
      return speed_;
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

    void setPosition (float p) override;

    // Override the EffectMode enum to define specific modes for Spotykach
    enum EffectMode
    {
      REEL = 0,
      SLICE = 1,
      DRIFT = 2,
      MODE_LAST
    };

  private:
    // Read and write pointers for the looper buffer
    float readIx_  = 0;
    float writeIx_ = 0;
    // Speed of looper heads
    float speed_ = 1.0f;
    // Position knob value
    float   position_   = 0.0f;
    // Current side being processed (0 or 1)
    uint8_t effectSide_ = 0;

    bool    isChannelActive (size_t ch) const;

    Spotykach (const Spotykach &)           = delete;
    Spotykach &operator=(const Spotykach &) = delete;
};
