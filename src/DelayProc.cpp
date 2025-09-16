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
  outputEnvFollower.init(sampleRate_);
  // Initialize compressor
  compressor.Init(sampleRate_);
  compressor.SetAttack(0.02f);
  compressor.SetRelease(0.25f);
  compressor.SetThreshold(-24.0f);
  compressor.SetRatio(6.0f);
  compressor.AutoMakeup(false);
}

void DelayProc::setParameters (float stretch, float fb)
{
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

float DelayProc::process (float in)
{
  // Update input envelope
  inputLevel = std::abs(in);
  inputLevel = inputEnvFollower.process(in);

  // Update current delay value
  updateCurrentDelay();

  // Process delay
  float read = delay.ReadHermite(currentDelay_);
  // Update output envelope follower
  outputLevel = outputEnvFollower.process(read);

  // Apply sidechain compression to the feedback path.
  float y = compressor.Apply(read);

  // Write back with feedback
  delay.Write((y - read) * feedback_ + in);

  // Age update based on growth rate (and input level)
  if (inputLevel > kMetabolicThreshold)
  {
    currentAge_ = infrasonic::unitclamp(currentAge_ + kGrowthRate);
  }
  return y;
}
