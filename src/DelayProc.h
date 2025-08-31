#pragma once

#include <cstddef>
#include <cmath>
#include "daisysp.h"
#include "Modulation.h"
#include "constants.h"
#include "app.h"

using namespace spotykach;
using namespace spotykach_hwtest;
using SmoothValue = Utils::SmoothValue;

struct DelayProc
{
  static constexpr size_t MAX_DELAY = (kSampleRate / 2.0f) * kMaxStretch; // 8 seconds at 48kHz
  daisysp::DelayLine<float, MAX_DELAY> delay;
  EnvelopeFollower inputEnvFollower;
  EnvelopeFollower outputEnvFollower;

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
  DelayProc() = default;

  void  init(float sr, size_t maxDelaySamples);
  void  setDelay(float dMs);
  void  updateCurrentDelay();
  void  setParameters(float dMs, float fb);
  float process(float in);

  ///////////
  NOCOPY (DelayProc);
};
