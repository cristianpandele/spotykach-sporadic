#pragma once

#include "DaisySP/Source/daisysp.h"
#include <array>
#include <cstddef>

/**
 * Class for delay line dispersion processing
 *
 * The DSP is made up of many stages of first-order allpass
 * filters, with linear fading between stages.
 */
class Dispersion
{
  public:
    struct Parameters
    {
        float dispersionAmount;
        float allpassFreq;
    };

    // Trivial default constructor; call init() before use
    Dispersion () = default;

    void init (float sampleRate);
    void reset ();

    float processSample (float x);
    void  setParameters (const Parameters &params);

  private:
    void  updateAllpassCoefficients ();
    float processStage (float x, size_t stage);

    // Parameters
    float inDispersionAmount;
    float inAllpassFreq;

    static constexpr size_t maxNumStages = 2;

    float                               fs;
    float                               a[2];
    float                               y1;
    std::array<float, maxNumStages + 1> stageFb;    // cleared in reset()
};
