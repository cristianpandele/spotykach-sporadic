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
    std::fill(readIx, readIx + kNumberSpotykachSides, 0.0f);
    std::fill(writeIx, writeIx + kNumberSpotykachSides, (float)kSampleRate);

    // Initialize the looper audio data buffers
    for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
    {
      std::fill(looperAudioData[side][ch], looperAudioData[side][ch] + kLooperAudioDataSamples, 0.0f);
    }
  }
}

void Spotykach::processAudio (AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
  // Process the Spotykach looper audio
  // Always record on side A for now, irrespective of the current mode
  for (size_t i = 0; i < size; ++i)
  {
    for (size_t side = 0; side < 1/* kNumberSpotykachSides */; ++side)
    {
      for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
      {
        out[ch][i] = looperAudioData[0 /*side*/][ch][static_cast<size_t>(readIx[side])];
        //
        looperAudioData[0/* side */][ch][static_cast<size_t>(writeIx[side])] = in[ch][i];
        //
      }
      readIx[side]  = (static_cast<size_t>(readIx[side]) + 1) % kLooperAudioDataSamples;
      writeIx[side] = (static_cast<size_t>(writeIx[side]) + 1) % kLooperAudioDataSamples;
    }
  }
}
