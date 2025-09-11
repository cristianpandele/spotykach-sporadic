#include "DiffusionControl.h"

#ifndef M_PI
constexpr float kPi = 3.14159265358979323846f;
#endif

void DiffusionControl::init (float sampleRate, size_t numBands)
{
  sampleRate_ = sampleRate;
  numBands_   = std::max(static_cast<size_t>(1), std::min(numBands, kMaxNutrientBands));
  for (uint8_t ch = 0; ch < spotykach::kNumberChannelsStereo; ++ch)
  {
    for (size_t band = 0; band < numBands_; ++band)
    {
      filters_[ch][band].Init(sampleRate_);
    }
  }
  updateBandLayout();
}

void DiffusionControl::setParameters (const Parameters &p)
{
  size_t newBands = std::max(static_cast<size_t>(1), std::min(p.numActiveBands, kMaxNutrientBands));
  float  minFreq  = std::max(kMinFreq, p.centerFreq / 8);    // Three octaves below center frequency
  float  maxFreq  = std::min(kMaxFreq, p.centerFreq * 8);    // Three octaves above center frequency
  if ((newBands != numBands_) || (minFreq != kMinFreq) || (maxFreq != kMaxFreq))
  {
    numBands_ = newBands;
    minFreq_  = minFreq;
    maxFreq_  = maxFreq;
    updateBandLayout();
  }
}

void DiffusionControl::getBandFrequencies (std::vector<float> &frequencies) const
{
  frequencies.clear();
  for (size_t band = 0; band < numBands_; ++band)
  {
    frequencies.push_back(centerFreqs_[band]);
  }
}

void DiffusionControl::updateBandLayout ()
{
  for (size_t band = 0; band < numBands_; ++band)
  {
    float t            = (numBands_ == 1) ? 0.0f : static_cast<float>(band) / static_cast<float>(numBands_ - 1);
    centerFreqs_[band] = minFreq_ * powf(maxFreq_ / minFreq_, t);
  }

  for (uint8_t ch = 0; ch < spotykach::kNumberChannelsStereo; ++ch)
  {
    for (size_t band = 0; band < numBands_; ++band)
    {
      filters_[ch][band].SetFreq(centerFreqs_[band]);
      filters_[ch][band].SetRes(0.707f);
    }
  }
}

void DiffusionControl::processBlockMono (const float *in, const uint8_t ch, float **outBand, size_t blockSize)
{
  for (size_t band = 0; band < numBands_; ++band)
  {
    float *o = outBand[band];
    auto  &f = filters_[ch][band];
    for (size_t i = 0; i < blockSize; ++i)
    {
      f.Process(in[i]);
      if (band == 0)
      {
        o[i] = f.Low();
      }
      else if (band == numBands_ - 1)
      {
        o[i] = f.High();
      }
      else
      {
        o[i] = f.Band();
      }
    }
  }
}
