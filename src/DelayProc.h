#pragma once

#include "Modulation.h"
#include "app.h"
#include "constants.h"
#include "daisysp.h"
#include <cmath>
#include <cstddef>

using namespace spotykach;
using namespace spotykach_hwtest;
using SmoothValue = Utils::SmoothValue;

struct DelayProc
{
  static constexpr size_t kSecInMin   = 60;
  static constexpr size_t kMinBpm     = 30;
  static constexpr size_t kDefaultBpm = 120;
  static constexpr size_t kMaxBpm     = 300;
  static constexpr size_t kMaxDelaySamples =
    kSampleRate * (static_cast<float>(kSecInMin) / static_cast<float>(kDefaultBpm)) * kMaxStretch;    // 8 seconds at 120 BPM (at 48kHz)

  daisysp::DelayLine<float, kMaxDelaySamples> delay;
  EnvelopeFollower                            inputEnvFollower;
  EnvelopeFollower                            outputEnvFollower;

  float sampleRate_;
  float feedback_;
  float baseDelayMs_;
  float stretch_;
  float targetDelay_;
  float currentDelay_;

  // Envelope followers: track input and output levels for dynamic processing
  float inputLevel;
  float outputLevel;
  // Placeholder parameters for envelope followers (attack/release in ms, level type)
  float envAttackMs_;
  float envReleaseMs_;

  // Trivial default constructor; call init() before use
  DelayProc () = default;

  void  init (float sr, size_t maxDelaySamples);
  void  setDelay (float dMs);
  void  updateCurrentDelay ();
  void  setParameters (float dMs, float fb);
  float process (float in);

  ///////////
  NOCOPY (DelayProc);
};
