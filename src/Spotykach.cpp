#include "app.h"
#include "Spotykach.h"

static constexpr size_t kLooperAudioDataSamples = 15.0f * kSampleRate * kNumberChannelsMono;

// Reserve two buffers for 15-second Spotykach loopers for each side - 16bit mono audio file at 48khz (about 0.172 MB each)
static DSY_SDRAM_BSS float   looperAudioData[kNumberSpotykachSides][kNumberChannelsStereo][kLooperAudioDataSamples];

void Spotykach::init ()
{
  // Initialize the Spotykach looper pointers
  readIx_  = 0;
  writeIx_ = 0;

  // Initialize the looper audio data buffers
  for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
  {
    std::fill(looperAudioData[effectSide_][ch], looperAudioData[effectSide_][ch] + kLooperAudioDataSamples, 0.0f);
  }
}

void Spotykach::processAudio (AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t blockSize)
{
  // Pass-through if mode is not MODE_1 or channelConfig is not MONO_LEFT, MONO_RIGHT, or STEREO
  if ((mode_ != MODE_1) ||
      (channelConfig_ == ChannelConfig::OFF || channelConfig_ >= ChannelConfig::CH_CONFIG_LAST))
  {
    std::copy(IN_L, IN_L + blockSize, OUT_L);
    std::copy(IN_R, IN_R + blockSize, OUT_R);
    return;
  }

  // Only process channels that are active for the current mode
  for (size_t i = 0; i < size; ++i)
  {
      for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
      {
        // Only process if this channel is active in the current mode
        if (isChannelActive(ch))
        {
          // Fractional read index
          float rIdx = readIx;
          size_t idx0 = static_cast<size_t>(rIdx);
          size_t idx1 = (idx0 + 1) % kLooperAudioDataSamples;
          float frac = rIdx - idx0;
          float s0 = looperAudioData[effectSide][ch][idx0];
          float s1 = looperAudioData[effectSide][ch][idx1];
          // Linear interpolation when between two samples
          out[ch][i] = infrasonic::lerp(s0, s1, frac);
          // Linear interpolation between dry input and looper output (when not in recording playback mode)
          out[ch][i] = infrasonic::lerp(in[ch][i], out[ch][i], mix);

          // Write input sample at fractional index
          size_t wIdx0                             = static_cast<size_t>(writeIx);
          looperAudioData[effectSide][ch][wIdx0]   = (writeIx - wIdx0) * in[ch][i];
          looperAudioData[effectSide][ch][wIdx0+1] = (1.0f - (writeIx - wIdx0)) * in[ch][i];
        }
        else
        {
          out[ch][i] = 0.0f;
        }
      }
      // Advance and wrap read/write indexes, preserving fractional part
      readIx += speed;
      if (readIx >= kLooperAudioDataSamples)
      {
        readIx -= kLooperAudioDataSamples;
      }
      else if (readIx < 0)
      {
        readIx += kLooperAudioDataSamples;
      }
      //
      writeIx += speed;
      if (writeIx >= kLooperAudioDataSamples)
      {
        writeIx -= kLooperAudioDataSamples;
      }
      else if (writeIx < 0)
      {
        writeIx += kLooperAudioDataSamples;
      }
  }
}

void Spotykach::setPosition(float p)
{
  // Map p in (0,1) to (0, 2*kSampleRate)
  position_ = infrasonic::map(p, 0.0f, 1.0f, 0.0f, 2.0f * kSampleRate);
  // Set readIx_ to be writeIx_ - position_, wrapping if needed
  if (speed_ < 0)
  {
    readIx_ = writeIx_ + std::abs(position_) + kBlockSize;
  }
  else
  {
    readIx_ = writeIx_ - (position_ + kBlockSize);
  }
  //
  if(readIx_ < 0)
  {
    readIx_ += kLooperAudioDataSamples;
  }
  else if(readIx_ >= kLooperAudioDataSamples)
  {
    readIx_ -= kLooperAudioDataSamples;
  }
}

// Helper to determine if a channel should be processed in the current mode
bool Spotykach::isChannelActive(size_t ch) const
{
  // Example logic: only process left channel in MONO_LEFT, right in MONO_RIGHT, both in STEREO
  switch (channelConfig_)
  {
    case ChannelConfig::MONO_LEFT:
    {
      return (ch == 0);
    }
    case ChannelConfig::MONO_RIGHT:
    {
      return (ch == 1);
    }
    case ChannelConfig::STEREO:
    {
      return ((ch == 0 || ch == 1));
    }
    default:
    {
      return false;
    }
  }
}
