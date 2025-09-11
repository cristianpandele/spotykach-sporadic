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
  constexpr float kMinStretch = 1.0f / 16.0f;    // Minimum stretch factor for delay nodes
  constexpr float kMaxStretch = 16.0f;           // Maximum stretch factor for delay nodes

  // Constants for the LEDs
  constexpr float kOffLedBrightness = 0.0f;
  constexpr float kMinLedBrightness = 0.1f;
  constexpr float kLowLedBrightness = 0.35f;
  constexpr float kMaxLedBrightness = 0.8f;

  // Threshold for parameter change detection
  static constexpr float kParamChThreshold = 0.015f;
};    // namespace spotykach
