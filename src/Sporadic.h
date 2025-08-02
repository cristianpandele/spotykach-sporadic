#pragma once

#include "common.h"
#include "daisy_seed.h"
#include "daisysp.h"
#include "hardware.h"

using namespace daisy;

// Class for Sporadic effect implementation
class Sporadic
{
  public:
    Sporadic ()  = default;
    ~Sporadic () = default;

    // void init ();

    void processAudio (AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size);

  private:
    // Read and write pointers for the looper buffers
    // uint16_t readIx  = 0;
    // uint16_t writeIx = 0;

    Sporadic (const Sporadic &)           = delete;
    Sporadic &operator=(const Sporadic &) = delete;
};
