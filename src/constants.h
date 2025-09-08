#pragma once

#include "common.h"
#include "daisy_seed.h"
#include "sr_165.h"
#include "ws2812.h"

namespace spotykach
{
    // Constants for the application
    constexpr size_t kSampleRate     = 48000;
    constexpr float  kSamplePeriodMs = 1000.0f / kSampleRate;
    constexpr size_t kBlockSize      = 48;

  // Constants for Sporadic
  constexpr int kMaxNumDelayProcs = 6;    // Maximum number of trees (output taps)
  constexpr int kMaxNutrientBands = 4;    // Maximum number of nutrient bands

    // Constants for the delay nodes
    constexpr float kMinStretch = 1.0f / 16.0f;   // Minimum stretch factor for delay nodes
    constexpr float kMaxStretch = 16.0f;          // Maximum stretch factor for delay nodes
};    // namespace spotykach
