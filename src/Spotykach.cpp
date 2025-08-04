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
  position = infrasonic::map(p, 0.0f, 1.0f, 0.0f, 2.0f * kSampleRate);
  // Set readIx to be writeIx - position, wrapping if needed
  readIx = writeIx - position - kBlockSize;
  if(readIx < 0)
  {
    readIx += kLooperAudioDataSamples;
  }
  else if(readIx >= kLooperAudioDataSamples)
  {
    readIx -= kLooperAudioDataSamples;
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
