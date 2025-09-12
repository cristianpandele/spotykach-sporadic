#pragma once

#include "Modulation.h"
#include "app.h"
#include "constants.h"
#include "daisysp.h"
#include <cmath>
#include <cstddef>

using namespace spotykach;
struct DelayProc
{
  static constexpr size_t kSecInMin   = 60;
  static constexpr size_t kMinBpm     = 30;
  static constexpr size_t kDefaultBpm = 120;
  static constexpr size_t kMaxBpm     = 300;
  static constexpr size_t kMaxDelaySamples =
    kSampleRate * (static_cast<float>(kSecInMin) / static_cast<float>(kDefaultBpm)) * kMaxStretch;    // 8 seconds at 120 BPM (at 48kHz)

  static constexpr float kGrowthRate         = 5e-8f;  // This gives us approximately 10 minutes to age out completely
  static constexpr float kMetabolicThreshold = 1e-3f;

  daisysp::DelayLine<float, kMaxDelaySamples> delay;
  EnvelopeFollower                            inputEnvFollower;     // Used for input level tracking
  EnvelopeFollower                            outputEnvFollower;    // Used for output level tracking
  daisysp::Compressor                         compressor;           // Used for feedback-path sidechain compression

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

  // Sidechain and age tracking
  float sidechainLevel_;
  float currentAge_;

  // Trivial default constructor; call init() before use
  DelayProc () = default;

  void  init (float sr, size_t maxDelaySamples);
  void  setParameters (float dMs, float fb);
  void  updateCurrentDelay ();
  void  setSidechainLevel (float sc);
  float process (float in);

  // Trivial accessors
  float getAge () const { return currentAge_; }

  ///////////
  NOCOPY (DelayProc);
};
