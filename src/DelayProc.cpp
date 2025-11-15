#include "DelayProc.h"

void DelayProc::init (float sr, size_t maxDelaySamples)
{
  // Set defaults previously done via in-class initializers
  sampleRate_     = sr;
  feedback_       = 0.0f;
  envAttackMs_    = 150.0f;
  envReleaseMs_   = 25.0f;
  baseDelayMs_    = 500.0f;
  stretch_        = 1.0f;
  currentDelay_   = baseDelayMs_ * stretch_ * sampleRate_ / 1000.0f;
  targetDelay_    = baseDelayMs_ * stretch_ * sampleRate_ / 1000.0f;
  currentAge_     = 0;
  sidechainLevel_ = 0.0f;

  // Initialize sub-components
  delay.Init();
  // Initialize envelope followers
  inputEnvFollower.init(sampleRate_);
  inputEnvFollower.setAttackMs(20.0f);
  inputEnvFollower.setReleaseMs(200.0f);
  outputEnvFollower.init(sampleRate_);
  outputEnvFollower.setAttackMs(20.0f);
  outputEnvFollower.setReleaseMs(200.0f);
  // Initialize compressor
  compressor.Init(sampleRate_);
  compressor.SetAttack(0.02f);
  compressor.SetRelease(0.25f);
  compressor.SetThreshold(-24.0f);
  compressor.SetRatio(6.0f);
  compressor.AutoMakeup(false);
}

void DelayProc::setParameters (bool reverse, float stretch, float fb)
{
  reverse_ = reverse;
  if ((stretch != stretch_) || (fb != feedback_))
  {
    stretch_     = stretch;
    targetDelay_ = baseDelayMs_ * stretch_ * sampleRate_ / 1000.0f;
    feedback_    = fb;
  }
}

void DelayProc::updateCurrentDelay ()
{
  float maxStep = 2.5f;
  float delta   = targetDelay_ - currentDelay_;

  if (std::abs(delta) > maxStep)
  {
    delta = std::copysign(maxStep, delta);
  }
  currentDelay_ += delta;
  // Clamp to valid range for Hermite interpolation
  currentDelay_ = daisysp::fclamp(currentDelay_, 2.0f, kMaxDelaySamples - 3.0f);
}

void DelayProc::setSidechainLevel (float sc)
{
  sidechainLevel_ = sc;
  // For stability, clamp sidechain level and convert to a key signal.
  float key = daisysp::fclamp(sidechainLevel_, 0.0f, 1.0f);
  // Apply to compressor
  compressor.Process(key);
}

void DelayProc::processBurst (const float in[kBurstSizeSamples], float out[kBurstSizeSamples])
{
  // Update current delay value (once per kBurstSizeSamples samples)
  updateCurrentDelay();

  // Read samples from delay line using burst mode
  float read_samples[kBurstSizeSamples];
  float write_samples[kBurstSizeSamples];

  delay.ReadHermiteBurst(currentDelay_, read_samples);

  inputLevel = inputEnvFollower.process(in[0]);
  // Age update based on growth rate (and input level)
  if (inputLevel > kMetabolicThreshold)
  {
    float sign  = reverse_ ? -1.0f : 1.0f;
    currentAge_ = infrasonic::unitclamp(currentAge_ + kBurstSizeSamples * sign * kGrowthRate);
  }

  // Process each of the kBurstSizeSamples samples
  for (size_t i = 0; i < kBurstSizeSamples; i++)
  {
    // Apply sidechain compression to the feedback path
    out[i] = compressor.Apply(read_samples[i]);

    // Prepare write sample with feedback
    write_samples[i] = (read_samples[i] - out[i]) * feedback_ + in[i];
  }

  // Update output envelope follower
  outputLevel = outputEnvFollower.process(out[0]);

  // Write kBurstSizeSamples samples back to delay line using burst mode
  delay.WriteBurst(write_samples);
}
