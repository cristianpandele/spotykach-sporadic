#pragma once

#include <cstddef>
#include <cmath>
#include "DaisySP/Source/daisysp.h"
#include "Dispersion.h"

// Simple DelayProc inspired by MultiDelay example, with basic filter
struct DelayProc
{
    static constexpr size_t MAX_DELAY = 48000; // 1 second at 48kHz
    daisysp::DelayLine<float, MAX_DELAY> delay;
    Dispersion dispersion;
    float feedback = 0.5f;
    float delayMs = 100.0f;
    float sampleRate = 48000.0f;

    // Provisions for future concepts from Mycelia DelayProc
    // Age: represents the "aging" of the delay processor, affecting modulation and dispersion
    float currentAge = 0.0f; // Current age value (0.0 to 1.0 normalized)

    // Dispersion: amount of frequency dispersion applied to the delay
    float dispersionAmount = 0.0f; // Dispersion amount based on age

    // Envelope followers: track input and output levels for dynamic processing
    float inputLevel = 0.0f; // Current input envelope level
    float outputLevel = 0.0f; // Current output envelope level
    // Placeholder parameters for envelope followers (attack/release in ms, level type)
    float envAttackMs = 150.0f;
    float envReleaseMs = 25.0f;
    // Level type could be RMS or Peak, but using float for now

    // Additional parameters for growth and modulation
    float growthRate = 0.5f; // Rate at which age increases
    float baseDelayMs = 500.0f; // Base delay for age calculations

    void init(float sr, size_t maxDelaySamples);
    void setDelay(float dMs);
    void setParameters(float dMs, float fb, float fFreq);
    float process(float in);
};
