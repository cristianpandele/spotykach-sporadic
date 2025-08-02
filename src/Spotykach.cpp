#include "app.h"
#include "Spotykach.h"

static constexpr size_t kLooperAudioDataSamples = 15.0f * kSampleRate * kNumberChannelsMono;

// Reserve two buffers for 15-second Spotykach loopers for each side - 16bit mono audio file at 48khz (about 0.172 MB each)
static DSY_SDRAM_BSS float   looperAudioData[kNumberSpotykachSides][kNumberChannelsStereo][kLooperAudioDataSamples];

void Spotykach::init ()
{
  for (size_t side = 0; side < kNumberSpotykachSides; ++side)
  {
    // Initialize the Spotykach looper pointers
    readIx  = 0;
    writeIx = (float)kSampleRate;

    // Initialize the looper audio data buffers
    for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
    {
      std::fill(looperAudioData[side][ch], looperAudioData[side][ch] + kLooperAudioDataSamples, 0.0f);
    }
  }
}

void Spotykach::processAudio (AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
  // Only process channels that are active for the current mode
  for (size_t i = 0; i < size; ++i)
  {
    // for (size_t side = 0; side < kNumberSpotykachSides; ++side)
    // {
      for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
      {
        // Only process if this channel is active in the current mode
        if (isChannelActive(ch))
        {
          out[ch][i] = looperAudioData[effectSide][ch][static_cast<size_t>(readIx)];
          looperAudioData[effectSide][ch][static_cast<size_t>(writeIx)] = in[ch][i];
        }
        else
        {
          out[ch][i] = 0.0f;
        }
      }
      readIx  = (static_cast<size_t>(readIx) + 1) % kLooperAudioDataSamples;
      writeIx = (static_cast<size_t>(writeIx) + 1) % kLooperAudioDataSamples;
    // }
  }
}

// Helper to determine if a channel should be processed in the current mode
bool Spotykach::isChannelActive(size_t ch) const
{
  // Example logic: only process left channel in MONO_LEFT, right in MONO_RIGHT, both in STEREO
  switch (currentMode)
  {
    case EffectMode::MONO_LEFT:
    {
      return (ch == 0);
    }
    case EffectMode::MONO_RIGHT:
    {
      return (ch == 1);
    }
    case EffectMode::STEREO:
    {
      return ((ch == 0 || ch == 1));
    }
    default:
    {
      return false;
    }
  }
}
