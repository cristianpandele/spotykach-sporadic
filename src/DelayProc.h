#pragma once

#include "Modulation.h"
#include "app.h"
#include "constants.h"
#include "daisysp.h"
#include "daisy.h"
#include <cmath>
#include <cstddef>

using namespace spotykach;

// Custom delay line that reads/writes in bursts of 4 samples to leverage SDRAM hardware burst mode
template <typename T, size_t max_size, size_t burst_size> class BurstDelayLine
{
  public:
    BurstDelayLine () { }

    ~BurstDelayLine () { }

    /** initializes the delay line by clearing the values within, and setting delay to 1 sample.
     */
    void Init () { Reset(); }

    /** clears buffer, sets write ptr to 0, and delay to 1 sample.
     */
    void Reset ()
    {
      for (size_t i = 0; i < max_size; i++)
      {
        line_[i] = T(0);
      }
      write_ptr_ = 0;
      delay_     = 1;
    }

    /** sets the delay time in samples
        If a float is passed in, a fractional component will be calculated for interpolating the delay line.
    */
    inline void SetDelay (size_t delay)
    {
      frac_  = 0.0f;
      delay_ = delay < max_size ? delay : max_size - 1;
    }

    /** sets the delay time in samples
        If a float is passed in, a fractional component will be calculated for interpolating the delay line.
    */
    inline void SetDelay (float delay)
    {
      int32_t int_delay = static_cast<int32_t>(delay);
      frac_             = delay - static_cast<float>(int_delay);
      delay_            = static_cast<size_t>(int_delay) < max_size ? int_delay : max_size - 1;
    }

    /** writes the sample of type T to the delay line, and advances the write ptr
     */
    inline void WriteBurst (const T samples[burst_size])
    {
      // Handle circular buffer wrapping: if write would cross boundary, split into two writes
      size_t remaining = max_size - write_ptr_;
      if (remaining >= burst_size)
      {
        // Single contiguous write
        memcpy(&line_[write_ptr_], samples, burst_size * sizeof(T));
      }
      else
      {
        // Split write: first part to end of buffer, second part from start
        memcpy(&line_[write_ptr_], samples, remaining * sizeof(T));
        memcpy(&line_[0], &samples[remaining], (burst_size - remaining) * sizeof(T));
      }
      write_ptr_ = (write_ptr_ - burst_size + max_size) % max_size;
    }

    /** returns the next sample of type T in the delay line, interpolated if necessary.
     */
    inline const T ReadHermite (float delay, T samples[burst_size]) const
    {
      int32_t delay_integral   = static_cast<int32_t>(delay);
      float   delay_fractional = delay - static_cast<float>(delay_integral);

      int32_t     t            = (write_ptr_ + delay_integral + max_size);
      const T     xm1          = line_[(t - 1) % max_size];
      const T     x0           = line_[(t) % max_size];
      const T     x1           = line_[(t + 1) % max_size];
      const T     x2           = line_[(t + 2) % max_size];
      const float c            = (x1 - xm1) * 0.5f;
      const float v            = x0 - x1;
      const float w            = c + v;
      const float a            = w + v + (x2 - x0) * 0.5f;
      const float b_neg        = w + a;
      const float f            = delay_fractional;
      return (((a * f) - b_neg) * f + c) * f + x0;
    }

    // Read 4 samples with Hermite interpolation (optimized for burst reads)
    inline void ReadHermiteBurst (float delay, float output[burst_size]) const
    {
      int32_t delay_integral   = static_cast<int32_t>(delay);
      float   delay_fractional = delay - static_cast<float>(delay_integral);

      // Read 4 consecutive samples from SDRAM in a burst
      int32_t base_t = (write_ptr_ + delay_integral + max_size);

      // We need 7 consecutive samples for Hermite interpolation of 4 outputs
      // (4 samples for each output, overlapping: xm1, x0, x1, x2 for each)
      constexpr size_t num_samples_needed = burst_size + 4 - 1;  // 7 samples
      float samples[num_samples_needed];

      // Calculate starting index with proper wrapping
      size_t start_idx = (base_t - 1 + max_size) % max_size;
      size_t remaining = max_size - start_idx;

      if (remaining >= num_samples_needed)
      {
        // Single contiguous read
        memcpy(samples, &line_[start_idx], num_samples_needed * sizeof(T));
      }
      else
      {
        // Split read: first part to end of buffer, second part from start
        memcpy(samples, &line_[start_idx], remaining * sizeof(T));
        memcpy(&samples[remaining], &line_[0], (num_samples_needed - remaining) * sizeof(T));
      }

      // Compute 4 interpolated outputs
      for (size_t out_idx = 0; out_idx < burst_size; out_idx++)
      {
        size_t      sample_idx = out_idx;
        const float xm1        = samples[sample_idx];
        const float x0         = samples[sample_idx + 1];
        const float x1         = samples[sample_idx + 2];
        const float x2         = samples[sample_idx + 3];
        const float c          = (x1 - xm1) * 0.5f;
        const float v          = x0 - x1;
        const float w          = c + v;
        const float a          = w + v + (x2 - x0) * 0.5f;
        const float b_neg      = w + a;
        const float f          = delay_fractional;
        output[out_idx]        = (((a * f) - b_neg) * f + c) * f + x0;
      }
    }

  private:
    float  frac_;
    size_t write_ptr_;
    size_t delay_;
    T      line_[max_size];
};

struct DelayProc
{
  static constexpr size_t kSecInMin   = 60;
  static constexpr size_t kMinBpm     = 30;
  static constexpr size_t kDefaultBpm = 120;
  static constexpr size_t kMaxBpm     = 300;
  static constexpr size_t kMaxDelaySamples = kSampleRate *
                                             (static_cast<float>(kSecInMin) / static_cast<float>(kDefaultBpm)) *
                                             kMaxStretch;    // 16 seconds at 120 BPM (at 48kHz)

  static constexpr float kGrowthRate         = 5e-8f;  // This gives us approximately 10 minutes to age out completely
  static constexpr float kMetabolicThreshold = 1e-3f;

  BurstDelayLine<float, kMaxDelaySamples, kBurstSizeSamples> delay;
  EnvelopeFollower                                           inputEnvFollower;     // Used for input level tracking
  EnvelopeFollower                                           outputEnvFollower;    // Used for output level tracking
  daisysp::Compressor compressor;    // Used for feedback-path sidechain compression

  float sampleRate_;
  float feedback_;
  float baseDelayMs_;
  float stretch_;
  float targetDelay_;
  float currentDelay_;

  // Whether the delay processors aging is in reverse mode
  bool reverse_;

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

  void init (float sr, size_t maxDelaySamples);
  void setParameters (bool reverse, float dMs, float fb);
  void updateCurrentDelay ();
  void setSidechainLevel (float sc);
  float process (float in);  // Process single sample
  void processBurst (const float in[kBurstSizeSamples], float out[kBurstSizeSamples]);  // Process 4 samples in burst mode

  // Trivial accessors
  float getAge () const { return currentAge_; }

  ///////////
  NOCOPY (DelayProc);
};
