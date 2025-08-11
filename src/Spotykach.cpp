#include "app.h"
#include "Spotykach.h"

static constexpr size_t kLooperAudioDataSamples = 15.0f * kSampleRate * kNumberChannelsMono;
static constexpr size_t kEchoAudioDataSamples = 2.0f * kSampleRate;

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

void Spotykach::updateAnalogControls (const AnalogControlFrame &c)
{
  // Update the analog effect parameters based on the control frame
  setMix(c.mix, c.mixAlt);
  setPitch(c.pitch);
  setPosition(c.position);
  setSize(c.size);
  setShape(c.shape);
}

void Spotykach::updateDigitalControls (const DigitalControlFrame &c)
{
  // Update the digital effect parameters based on the control frame
  setSpotyPlay(c.spotyPlay);
  if (c.spotyPlay)
  {
    setReverse(false);
    setPlay(false);
    setAltPlay(false);
  }
  else
  {
    setReverse(c.reverse);
    setPlay(c.play);
    setAltPlay(c.altPlay);
  }

  updateDisplayState();
}

void Spotykach::getDigitalControls(DigitalControlFrame &c)
{
  c.reverse = reverse_;
  c.play = play_;
  c.altPlay = record_;
  c.spotyPlay = false;  // Reset Spotykach+Play state
}

void Spotykach::updateDisplayState()
{
  // Prepare a minimal display view without exposing internals
  DisplayState view{};
  view.playActive     = play_;
  view.reverseActive  = reverse_;
  view.altPlayActive  = record_;

  if (reverse_)
  {
    view.reverseLedColors[0] = {0x0000ff, 1.0f};  // Blue
    view.reverseLedColors[1] = {0x0000ff, 0.5f};  // Blue
  }

  constexpr uint8_t N = spotykach::Hardware::kNumLedsPerRing;

  switch (state_)
  {
    case ECHO:
    {
      // Factor the length of the echo into the LED ring (and zoom in twice for a better view)
      float positionFactor = 2.0f * position_ * (float)kEchoAudioDataSamples / (float)kLooperAudioDataSamples;

      // Compute spans in LED index space
      uint8_t start    = static_cast<uint8_t>((1.0f - positionFactor) * N);
      uint8_t spanSize = static_cast<uint8_t>(size_ * positionFactor * N);

      // Yellow area indicating the position
      LedRgbBrightness ledColor     = {0xffff00, 0.5f};
      view.rings[view.layerCount++] = Effect::RingSpan{start, N, ledColor};
      // Orange span indicating the size
      ledColor                      = {0xff8000, 0.5f};
      view.rings[view.layerCount++] = Effect::RingSpan{start, std::min<uint8_t>(start + spanSize, N), ledColor};

      // Compute the span between the read and write heads
      float relativePos = writeIx_ - readIx_;
      if (relativePos < 0)
      {
        relativePos += kLooperAudioDataSamples;
      }

      // Convert span between the read and write heads to proportion of the echo buffer
      positionFactor = relativePos / (float)kEchoAudioDataSamples;
      // Invert position factor for LED mapping
      positionFactor = 1.0f - positionFactor;

      // Read head position in LED index space
      uint8_t readIxLed  = std::min(start + static_cast<uint8_t>(positionFactor * spanSize), N - 1);
      uint8_t readEndLed = std::min<uint8_t>(readIxLed + 1, N);

      // Display read head position
      ledColor                      = {0xff00ff, 1.0f};
      view.rings[view.layerCount++] = Effect::RingSpan{readIxLed, readEndLed, ledColor};

      // Play LED colors
      view.playLedColors[0] = {0x00ff00, 1.0f};    // Green
      view.playLedColors[1] = {0x000000, 0.0f};    // Off

      break;
    }

    case RECORDING:
    {
      // Read head position
      uint8_t readIxLed = static_cast<uint8_t>(readIx_ * N / kLooperAudioDataSamples);
      uint8_t readEndLed   = std::min<uint8_t>(readIxLed + 1, N);

      // Write head position
      uint8_t writeIxLed = static_cast<uint8_t>(writeIx_ * N / kLooperAudioDataSamples);
      uint8_t writeEnd   = std::min<uint8_t>(writeIxLed + 1, N);

      // Yellow area indicating the writable area
      LedRgbBrightness ledColor     = {0xffff00, 0.5f};
      view.rings[view.layerCount++] = Effect::RingSpan{0, N, ledColor};

      // Orange area indicating the actively written area
      ledColor = {0xff8000, 0.5f};
      if (speed_ > 0)
      {
        view.rings[view.layerCount++] = Effect::RingSpan{writeIxLed, N, ledColor};
      }
      else
      {
        view.rings[view.layerCount++] = Effect::RingSpan{0, writeIxLed, ledColor};
      }

      // Display read head position
      ledColor                      = {0xff00ff, 1.0f};
      view.rings[view.layerCount++] = Effect::RingSpan{readIxLed, readEndLed, ledColor};

      // Display write head position
      ledColor                      = {0xff0000, 1.0f};
      view.rings[view.layerCount++] = Effect::RingSpan{writeIxLed, writeEnd, ledColor};

      // Play LED colors
      if (play_)
      {
        view.playLedColors[0] = {0x00ff00, 1.0f};  // Green
      }
      else
      {
        view.playLedColors[0] = {0x000000, 0.0f};  // Off
      }
      view.playLedColors[1] = {0xff0000, 1.0f};    // Red

      break;
    }

    case LOOP_PLAYBACK:
    {
      // Compute spans in LED index space
      uint8_t start = static_cast<uint8_t>(position_ * N);
      uint8_t spanSize  = static_cast<uint8_t>(size_ * N);

      // Read head position
      uint8_t readIxLed  = static_cast<uint8_t>(readIx_ * N / kLooperAudioDataSamples);
      uint8_t readEndLed = std::min<uint8_t>(readIxLed + 1, N);

      // Yellow area indicating the position and size
      LedRgbBrightness ledColor     = {0xffff00, 0.5f};
      view.rings[view.layerCount++] = Effect::RingSpan{start, std::min<uint8_t>(start + spanSize, N), ledColor};

      // Orange area indicating the actively read area
      ledColor = {0xff8000, 0.5f};
      if (speed_ > 0)
      {
        view.rings[view.layerCount++] =
          Effect::RingSpan{readIxLed, std::min<uint8_t>(start + spanSize, N), ledColor};
      }
      else
      {
        view.rings[view.layerCount++] = Effect::RingSpan{start, readIxLed, ledColor};
      }

      // Display read head position
      ledColor                      = {0xff00ff, 1.0f};
      view.rings[view.layerCount++] = Effect::RingSpan{readIxLed, readEndLed, ledColor};

      // Play LED colors
      if (play_)
      {
        view.playLedColors[0] = {0x00ff00, 1.0f};    // Green
        view.playLedColors[1] = {0x00ff00, 1.0f};    // Green
      }
      else
      {
        view.playLedColors[0] = {0x000000, 0.0f};    // Off
        view.playLedColors[1] = {0x000000, 0.0f};    // Off
      }

      break;
    }

    default:
    {
      // OFF state - no active rings
      view.layerCount = 0;
      // Play LED colors
      view.playLedColors[0] = {0x000000, 0.0f};    // Off
      view.playLedColors[1] = {0x000000, 0.0f};    // Off
      break;
    }
  }
  publishDisplay(view);
}

void Spotykach::updateIndex(float &index, float increment, Span<float> window)
{
  // Update the write index by incrementing it and wrapping around if needed
  index += increment;

  float windowSize = window.end - window.start;
  float windowEnd  = window.end;
  // Handle wraparound when the window is inverted
  if (window.end < window.start)
  {
    windowEnd += kLooperAudioDataSamples;
    windowSize = windowEnd - window.start;
  }

  if (window.start == window.end)
  {
    // If the window is a point, just clamp the index to that point
    index = window.start;
  }
  else
  {
    while (index > window.end)
    {
      index -= windowSize;
    }
    while (index < window.start)
    {
      index += windowSize;
    }
  }
}

void Spotykach::processAudio(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t blockSize)
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

        // Update the write index
        Span<float> writeSpan = {0.0f, kLooperAudioDataSamples};
        updateIndex(writeIx_, 1.0f, writeSpan);

        // Map position_ in (0,1) to (0, kEchoAudioDataSamples)
        float readWindowOffset = position_ * (float)kEchoAudioDataSamples;
        // Leave a margin to prevent the read index overtaking the write index
        float readWindowMargin = std::abs(speed_ * kBlockSize);
        // Set the start of the read window to trail the write index (with wraparound)
        float readWindowStart = writeIx_;
        updateIndex(readWindowStart, -(readWindowOffset + readWindowMargin), writeSpan);
        // Set the end of the read window based on the size (with wraparound)
        float readWindowEnd = readWindowStart;
        updateIndex(readWindowEnd, readWindowOffset * size_, writeSpan);

        // Update the read index
        Span<float> readSpan = {readWindowStart, readWindowEnd};
        updateIndex(readIx_, speed_, readSpan);
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
        // Update the write index
        Span<float> writeSpan = {0.0f, kLooperAudioDataSamples};
        updateIndex(writeIx_, speed_, writeSpan);

        // Update the read index
        updateIndex(readIx_, speed_, writeSpan);
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
        Span<float> loopSpan   = {spanStart, spanEnd};
        float prevReadIx = readIx_;
        updateIndex(readIx_, speed_, loopSpan);
        writeIx_ = readIx_;
        // Constrain readIx_ to span [spanStart, spanEnd)
        // Map position_ relative to kLooperAudioDataSamples
        float pos       = position_ * (float) kLooperAudioDataSamples;
        float spanStart = pos;
        float spanEnd   = pos + size_ * (kLooperAudioDataSamples - pos);
        if (spanEnd > kLooperAudioDataSamples)
        {
          spanEnd = kLooperAudioDataSamples;
        }

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
