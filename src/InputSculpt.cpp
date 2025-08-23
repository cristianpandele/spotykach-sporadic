#include "InputSculpt.h"

void InputSculpt::init (float sampleRate)
{
  svf_.Init(sampleRate);
  overdrive_.SetDrive(kMinDriveAmt);
  updateFilter(centerFreq_, q_);
}

void InputSculpt::setFreq (float f)
{
  f                    = daisysp::fclamp(f, 0.0f, 1.0f);
  constexpr float fMin = 50.0f;
  constexpr float fMax = 18000.0f;
  centerFreq_ = daisysp::fmap(f, fMin, fMax, Mapping::LOG);
  updateFilter(centerFreq_, q_);
}

void InputSculpt::setWidth (float w)
{
  w = daisysp::fclamp(w, 0.0f, 1.0f);
  // Map w=0 narrow (high Q), w=1 wide (low Q)
  q_ = daisysp::fmap(1.0f - w, 0.0f, 1.0f, Mapping::EXP);

  updateFilter(centerFreq_, q_);
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
  float bp = svf_.Band();
  return bp;
}

void InputSculpt::processBlock (const float *inL, const float *inR, float *outL, float *outR, size_t blockSize)
{
  for (size_t i = 0; i < blockSize; ++i)
  {
    outL[i] = processSample(inL[i]);
    outR[i] = processSample(inR[i]);
  }
}