#include "DelayProc.h"

void DelayProc::init (float sr, size_t maxDelaySamples)
{
  // Set defaults previously done via in-class initializers
  sampleRate_       = sr;
  feedback_         = 0.0f;
  envAttackMs_      = 150.0f;
  envReleaseMs_     = 25.0f;
  baseDelayMs_      = 500.0f;
  stretch_          = 1.0f;
  currentDelay_     = baseDelayMs_ * stretch_ * sampleRate_ / 1000.0f;
  targetDelay_      = baseDelayMs_ * stretch_ * sampleRate_ / 1000.0f;

  // Initialize sub-components
  delay.Init();
  // Set target delay
  setDelay(targetDelay_);
  // Initialize envelope followers
  inputEnvFollower.init(sampleRate_);
  outputEnvFollower.init(sampleRate_);
}

void DelayProc::setDelay (float dSamples)
{
  delay.SetDelay(dSamples);
}

void DelayProc::updateCurrentDelay()
{
  float maxStep = 2.5f;
  float delta = targetDelay_ - currentDelay_;

  if (std::abs(delta) > maxStep)
  {
    delta = std::copysign(maxStep, delta);
  }
  currentDelay_ += delta;
  currentDelay_ = daisysp::fclamp(currentDelay_, 2.0f, kMaxDelaySamples - 3.0f);
}

void DelayProc::setParameters (float stretch, float fb)
{
  if ((stretch != stretch_) || (fb != feedback_))
  {
    stretch_     = stretch;
    targetDelay_ = baseDelayMs_ * stretch_ * sampleRate_ / 1000.0f;
    feedback_ = fb;
  }
}

float DelayProc::process (float in)
{
  // Update input envelope
  inputLevel = std::abs(in);
  inputLevel = inputEnvFollower.process(in);

  // Update current delay value
  updateCurrentDelay();

  // Process delay
  // float read = delay.Read(currentDelay_);
  float read = delay.ReadHermite(currentDelay_);
  outputLevel = inputEnvFollower.process(read);
  // Write back with feedback
  delay.Write(read * feedback_ + in);

  return read;
}
