#pragma once

#include <cstddef>
#include <array>
#include "DaisySP/Source/daisysp.h"

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

    Dispersion();

    void prepare(float sampleRate);
    void reset();

    float processSample(float x);
    void setParameters(const Parameters &params);

private:
    void updateAllpassCoefficients();
    float processStage(float x, size_t stage);

    // Parameters
    float inDispersionAmount = 0.0f;
    float inAllpassFreq = 800.0f;

    static constexpr size_t maxNumStages = 10;

    float fs = 44100.0f;
    float a[2] = {0.0f};
    float y1 = 0.0f;
    std::array<float, maxNumStages + 1> stageFb = {0.0f};
};
