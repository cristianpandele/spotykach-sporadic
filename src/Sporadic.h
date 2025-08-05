#pragma once

#include "common.h"
#include "daisy_seed.h"
#include "daisysp.h"
#include "Effect.h"
#include "hardware.h"

using namespace daisy;

// Class for Sporadic effect implementation
class Sporadic : public Effect
{
  public:
    Sporadic ()  = default;
    ~Sporadic () = default;

    void processAudio (AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size) override;

  private:
    Sporadic (const Sporadic &)           = delete;
    Sporadic &operator=(const Sporadic &) = delete;
};
