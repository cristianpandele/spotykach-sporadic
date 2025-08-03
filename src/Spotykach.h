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
    Spotykach (uint8_t side) : effectSide(side) {}
    ~Spotykach () = default;

    void init ();

    void processAudio (AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size) override;

    void setSpeed (float s)
    {
      // Map the pitch to -4..4
      speed = infrasonic::map(s, 0.0f, 1.0f, -4.0f, 4.0f);
    }

  private:
    // Read and write pointers for the looper buffer
    float readIx  = 0;
    float writeIx = 0;
    // Speed of looper heads
    float speed   = 1.0f;

    bool    isChannelActive (size_t ch) const;
    uint8_t effectSide                      = 0;    // Current side being processed (0 or 1)

    Spotykach (const Spotykach &)           = delete;
    Spotykach &operator=(const Spotykach &) = delete;
};
