#include "InputSculpt.h"

void InputSculpt::init (float sampleRate)
{
  sampleRate_ = sampleRate;
  svf_.Init(sampleRate_);
  overdrive_.SetDrive(kMinDriveAmt);
  updateFilter(centerFreq_, q_);
}

void InputSculpt::setFreq (float f)
{
  centerFreq_ = f;
  updateFilter(centerFreq_, q_);
}

void InputSculpt::setWidth (float w)
{
  w = infrasonic::unitclamp(w);
  // Map w=0 narrow (high Q), w=1 wide (low Q)
  q_ = daisysp::fmap(1.0f - w, 0.0f, 1.0f, Mapping::EXP);

  updateFilter(centerFreq_, q_);
}

void InputSculpt::setShape (float s)
{
  shape_ = infrasonic::unitclamp(s);
}

void InputSculpt::setOverdrive (float od)
{
  overdriveAmt_ = daisysp::fclamp(od, 0.0f, 1.0f);
  overdriveAmt_ = daisysp::fmap(overdriveAmt_, kMinDriveAmt, kMaxDriveAmt);
  overdrive_.SetDrive(overdriveAmt_);
}

void InputSculpt::updateFilter (float freq, float q)
{
  svf_.SetFreq(freq);
  svf_.SetRes(q);
}

float InputSculpt::processSample (float in)
{
  // Apply overdrive
  float od = overdrive_.Process(in);
  // Compensate the overdrive gain (in reverse, higher drive -> lower gain)
  float invOd = kMinDriveAmt + (kMaxDriveAmt - overdriveAmt_);
  float gain  = infrasonic::map(invOd, kMinDriveAmt, kMaxDriveAmt, kMinDriveGainComp, kMaxDriveGainComp);
  od *= gain;

  // Apply the filtering
  svf_.Process(od);

  float t = shape_;
  float out;
  if (t < 0.33333334f)
  {
    float u = t / 0.33333334f;
    out     = infrasonic::lerp(od, svf_.Low(), u);
  }
  else if (t < 0.6666667f)
  {
    float u = (t - 0.33333334f) / 0.33333334f;
    out     = infrasonic::lerp(svf_.Low(), svf_.Band(), u);
  }
  else
  {
    float u = (t - 0.6666667f) / 0.33333334f;
    out     = infrasonic::lerp(svf_.Band(), svf_.High(), u);
  }
  return out;
}

void InputSculpt::processBlockMono (const float *in, float *out, size_t blockSize)
{
  for (size_t i = 0; i < blockSize; ++i)
  {
    out[i] = processSample(in[i]);
  }
}