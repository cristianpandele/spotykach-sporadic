#include "app.h"
#include "Spotykach.h"

static constexpr size_t kLooperAudioDataSamples = 15.0f * kSampleRate * kNumberChannelsMono;

// Reserve two buffers for 15-second Spotykach loopers for each side - 16bit mono audio file at 48khz (about 0.172 MB each)
static DSY_SDRAM_BSS float   looperAudioData[kNumberEffectSlots][kNumberChannelsStereo][kLooperAudioDataSamples] = {{{0.0f}}};

void Spotykach::init ()
{
  // Initialize the Spotykach looper pointers
  readIx_  = 0;
  writeIx_ = 0;

  // Initialize the looper audio data buffers
  for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
  {
    if (isChannelActive(ch))
    {
      std::fill(looperAudioData[effectSide_][ch], looperAudioData[effectSide_][ch] + kLooperAudioDataSamples, 0.0f);
    }
  }
}

void Spotykach::setPitch (float s)
{
  // Map the pitch to 0..4
  speed_ = infrasonic::map(s, 0.0f, 1.0f, 0.0f, 4.0f);
  if (reverse_)
  {
    // Reverse playback, set speed to negative
    speed_ = -std::abs(speed_);
  }
  else
  {
    // Normal playback, set speed to positive
    speed_ = std::abs(speed_);
  }
}

void Spotykach::updateReadIndexPosition (float p)
{
  // Set readIx_ to be writeIx_ - position_, wrapping if needed
  if (speed_ < 0)
  {
    readIx_ = writeIx_ + (position_ + std::abs(speed_ * kBlockSize));
  }
  else
  {
    readIx_ = writeIx_ - (position_ + std::abs(speed_ * kBlockSize));
  }
  //
  if (readIx_ < 0)
  {
    readIx_ += kLooperAudioDataSamples;
  }
  else if (readIx_ >= kLooperAudioDataSamples)
  {
    readIx_ -= kLooperAudioDataSamples;
  }
}

void Spotykach::setMix (float m, bool altLatch)
{
  // If alt latch is pressed, set feedback instead of mix
  if (altLatch)
  {
    setFeedback(m);
  }
  else
  {
    mix_ = m;
  }
}

void Spotykach::setSize (float s)
{
  // Ensure size is just a hair above 0 at all times
  size_ = infrasonic::map(s, 0.0f, 1.0f, 0.05f, 1.0f);
  size_ = std::clamp(size_, 0.05f, 1.0f);
}

void Spotykach::setPlay (bool p)
{
  play_ = p;

  switch (state_)
  {
    case OFF:
    {
      if (play_)
      {
        // Log::PrintLine("Switching to ECHO state");
        state_ = ECHO;
        // Clear the audio buffers
        for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
        {
          if (isChannelActive(ch))
          {
            std::fill(looperAudioData[effectSide_][ch],
                      looperAudioData[effectSide_][ch] + kLooperAudioDataSamples,
                      0.0f);
          }
        }
      }
      break;
    }
    case ECHO:
    {
      if (!play_)
      {
        // If stopped playing, return to OFF state
        // Log::PrintLine("Switching to OFF state");
        state_ = OFF;
      }
      break;
    }
    default:
    {
      // Other states do not change on play toggle
      break;
    }
  }
}

void Spotykach::setAltPlay (bool r)
{
  record_ = r;

  switch (state_)
  {
    case RECORDING:
    {
      if (!record_)
      {
        // If recording, switch to LOOP_PLAYBACK state
        // Log::PrintLine("Switching to LOOP_PLAYBACK state");
        state_ = LOOP_PLAYBACK;
      }
      break;
    }
    default:
    {
      if (record_)
      {
        // Reset read and write indices
        readIx_  = 0;
        writeIx_ = 0;
        // Switch to RECORDING state
        // Log::PrintLine("Switching to RECORDING state");
        state_ = RECORDING;
        break;
      }
    }
  }
}

void Spotykach::setSpotyPlay (bool s)
{
  bool reset = s;

  if (reset)
  {
    // Switch to OFF state
    // Log::PrintLine("Switching to OFF state");
    state_ = OFF;
  }
}

void Spotykach::setReverse (bool r)
{
  reverse_ = r;
  if (reverse_)
  {
    // Reverse playback, set speed to negative
    speed_ = -std::abs(speed_);
  }
  else
  {
    // Normal playback, set speed to positive
    speed_ = std::abs(speed_);
  }
}

void Spotykach::processAudio (AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t blockSize)
{
  // Pass-through if mode is not MODE_1 or channelConfig is not MONO_LEFT, MONO_RIGHT, or STEREO
  if ((mode_ != MODE_1) ||
      (channelConfig_ == ChannelConfig::OFF || channelConfig_ >= ChannelConfig::CH_CONFIG_LAST))
  {
    for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
    {
      if (isChannelActive(ch))
      {
        std::copy(in[ch], in[ch] + blockSize, out[ch]);
      }
    }
    return;
  }
  switch (state_)
  {
    case OFF:
    {
      // Reset heads to 0
      readIx_ = writeIx_ = 0.0f;
      for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
      {
        // Only process if this channel is active in the current mode
        if (isChannelActive(ch))
        {
          std::copy(in[ch], in[ch] + blockSize, out[ch]);
        }
      }
      return;
    }
    ///////////////
    case ECHO:
    {
      for (size_t i = 0; i < blockSize; ++i)
      {
        for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
        {
          // Only process if this channel is active in the current mode
          if (isChannelActive(ch))
          {
            // Neighbouring sample indices
            size_t rIdx0 = static_cast<size_t>(readIx_);
            size_t rIdx1 = (rIdx0 + 1) % kLooperAudioDataSamples;
            // Fractional part for interpolation
            float frac = readIx_ - rIdx0;
            // Neighbouring samples
            float s0 = looperAudioData[effectSide_][ch][rIdx0];
            float s1 = looperAudioData[effectSide_][ch][rIdx1];
            // Linear interpolation when between the two samples
            float loopOut = infrasonic::lerp(s0, s1, frac);
            // Mix the input with the loop output
            out[ch][i] = infrasonic::lerp(in[ch][i], loopOut, mix_);

            // Feedback/overdub
            size_t wIdx0                            = static_cast<size_t>(writeIx_);
            size_t wIdx1                            = static_cast<size_t>((wIdx0 + 1) % kLooperAudioDataSamples);
            float  old0                             = looperAudioData[effectSide_][ch][wIdx0];
            float  old1                             = looperAudioData[effectSide_][ch][wIdx1];
            looperAudioData[effectSide_][ch][wIdx0] = in[ch][i] + feedback_ * old0;
            looperAudioData[effectSide_][ch][wIdx1] = in[ch][i] + feedback_ * old1;
          }
          else
          {
            out[ch][i] = 0.0f;
          }
        }
        writeIx_ += 1.0f; // reverse_ ? -1.0f : 1.0f;
        if (writeIx_ >= kLooperAudioDataSamples)
        {
          writeIx_ -= kLooperAudioDataSamples;
        }
        else if (writeIx_ < 0)
        {
          writeIx_ += kLooperAudioDataSamples;
        }
        readIx_ += speed_;
        if (readIx_ >= kLooperAudioDataSamples)
        {
          readIx_ -= kLooperAudioDataSamples;
        }
        else if (readIx_ < 0)
        {
          readIx_ += kLooperAudioDataSamples;
        }
      }
      // Ensure readIx within a span: (writeIx_ - position_, writeIx_ - position_ * (1.0f - size_))
      float spanOffset = position_ * (1.0f - size_);
      float spanTarget = writeIx_ - spanOffset;
      if (spanTarget < 0)
      {
        spanTarget += kLooperAudioDataSamples;
      }
      else if (spanTarget >= kLooperAudioDataSamples)
      {
        spanTarget -= kLooperAudioDataSamples;
      }
      if (speed_ < 0)
      {
        if (readIx_ < spanTarget + std::abs(speed_ * blockSize))
        {
          updateReadIndexPosition(position_);
        }
      }
      else
      {
        if (readIx_ > spanTarget - std::abs(speed_ * blockSize))
        {
          updateReadIndexPosition(position_);
        }
      }
      return;
    }
    ///////////////
    case RECORDING:
    {
      for (size_t i = 0; i < blockSize; ++i)
      {
        for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
        {
          // Only process if this channel is active in the current mode
          if (isChannelActive(ch))
          {
            // Neighbouring sample indices
            size_t rIdx0 = static_cast<size_t>(readIx_);
            size_t rIdx1 = (rIdx0 + 1) % kLooperAudioDataSamples;
            // Fractional part for interpolation
            float frac = readIx_ - rIdx0;
            // Neighbouring samples
            float s0 = looperAudioData[effectSide_][ch][rIdx0];
            float s1 = looperAudioData[effectSide_][ch][rIdx1];
            if (play_)
            {
              // Linear interpolation when between the two samples
              float loopOut = infrasonic::lerp(s0, s1, frac);
              // Mix the input with the loop output
              out[ch][i] = infrasonic::lerp(in[ch][i], loopOut, mix_);
            }
            else
            {
              out[ch][i] = in[ch][i];    // If not playing, audition the input
            }
            size_t wIdx0                            = static_cast<size_t>(writeIx_);
            size_t wIdx1                            = static_cast<size_t>((wIdx0 + 1) % kLooperAudioDataSamples);
            float  old0                             = looperAudioData[effectSide_][ch][wIdx0];
            float  old1                             = looperAudioData[effectSide_][ch][wIdx1];
            looperAudioData[effectSide_][ch][wIdx0] = in[ch][i] + feedback_ * old0;
            looperAudioData[effectSide_][ch][wIdx1] = in[ch][i] + feedback_ * old1;
          }
        }
        writeIx_ += speed_;    // reverse_ ? -1.0f : 1.0f;
        if (writeIx_ >= kLooperAudioDataSamples)
        {
          writeIx_ -= kLooperAudioDataSamples;
        }
        else if (writeIx_ < 0)
        {
          writeIx_ += kLooperAudioDataSamples;
        }
        readIx_ += speed_;
        if (readIx_ >= kLooperAudioDataSamples)
        {
          readIx_ -= kLooperAudioDataSamples;
        }
        else if (readIx_ < 0)
        {
          readIx_ += kLooperAudioDataSamples;
        }
      }
      return;
    }
    ///////////////
    case LOOP_PLAYBACK:
    {
      for (size_t i = 0; i < blockSize; ++i)
      {
        for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
        {
          // Only process if this channel is active in the current mode
          if (isChannelActive(ch))
          {
            // Neighbouring sample indices
            size_t rIdx0 = static_cast<size_t>(readIx_);
            size_t rIdx1 = (rIdx0 + 1) % kLooperAudioDataSamples;
            // Fractional part for interpolation
            float frac = readIx_ - rIdx0;
            // Neighbouring samples
            float s0 = looperAudioData[effectSide_][ch][rIdx0];
            float s1 = looperAudioData[effectSide_][ch][rIdx1];
            if (play_)
            {
              // Linear interpolation when between the two samples
              float loopOut = infrasonic::lerp(s0, s1, frac);
              // Mix the input with the loop output
              out[ch][i] = infrasonic::lerp(in[ch][i], loopOut, mix_);
            }
            else
            {
              out[ch][i] = in[ch][i];    // If not playing, audition the input
            }
          }
          else
          {
            out[ch][i] = 0.0f;
          }
        }
        // Advance read, write follows read
        readIx_ += speed_;
        if (readIx_ >= kLooperAudioDataSamples)
          readIx_ -= kLooperAudioDataSamples;
        else if (readIx_ < 0)
          readIx_ += kLooperAudioDataSamples;
        writeIx_ = readIx_;
        // Constrain readIx_ to span [spanStart, spanEnd)
        float spanStart = position_;
        float spanEnd   = position_ + size_ * (kLooperAudioDataSamples - position_);
        if (spanEnd > kLooperAudioDataSamples)
          spanEnd = kLooperAudioDataSamples;
        if (speed_ >= 0)
        {
          if (readIx_ >= spanEnd)
            readIx_ = spanStart;
        }
        else
        {
          if (readIx_ < spanStart)
            readIx_ = spanEnd - 1;
        }
      }
      return;
    }
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
