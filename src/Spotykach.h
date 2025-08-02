#pragma once

#include "common.h"
#include "daisy_seed.h"
#include "daisysp.h"
#include "hardware.h"

using namespace daisy;
using namespace spotykach_hwtest;

// Class for Spotykach looper implementation
class Spotykach
{
  public:
    Spotykach ()  = default;
    ~Spotykach () = default;

    void init ();

    void processAudio (AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size);

  private:
    // Read and write pointers for the looper buffers
    float readIx[kNumberSpotykachSides]     = {0};
    float writeIx[kNumberSpotykachSides]    = {0};

    Spotykach (const Spotykach &)           = delete;
    Spotykach &operator=(const Spotykach &) = delete;
};
