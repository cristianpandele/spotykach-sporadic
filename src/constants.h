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
  constexpr size_t kMaxNumDelayProcs = 4;    // Maximum number of trees (output taps)
  constexpr size_t kMaxNutrientBands = 4;    // Maximum number of nutrient bands

  // Constants for the delay nodes
  constexpr float kMinStretch = 1.0f / 32.0f;    // Minimum stretch factor for delay nodes
  constexpr float kMaxStretch = 32.0f;           // Maximum stretch factor for delay nodes

  // Constants for the LEDs
  constexpr float kOffLedBrightness = 0.0f;
  constexpr float kMinLedBrightness = 0.1f;
  constexpr float kLowLedBrightness = 0.35f;
  constexpr float kMidLedBrightness = 0.45f;
  constexpr float kMaxLedBrightness = 0.75f;

  // Constants for the envelope follower
  static constexpr float kMinAttackTime  = 20.0f;
  static constexpr float kMaxAttackTime  = 2400.0f;
  static constexpr float kMinReleaseTime = 10.0f;
  static constexpr float kMaxReleaseTime = 800.0f;

  // Constants for the delay line burst processing
  static constexpr size_t kBurstSizeSamples = 4 * 4 / sizeof(float);    // Burst size is 4 words, divided by the sample size processed every step

  // Threshold for parameter change detection
  static constexpr float kParamChThreshold = 0.015f;
};    // namespace spotykach
