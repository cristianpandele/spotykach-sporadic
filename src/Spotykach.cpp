#include "app.h"
#include "Spotykach.h"

static constexpr size_t kLooperAudioDataSamples = 15.0f * kSampleRate * kNumberChannelsMono;
static constexpr size_t kEchoAudioDataSamples = 2.0f * kSampleRate;

// Reserve two buffers for 15-second Spotykach loopers for each side - 16bit mono audio file at 48khz (about 0.172 MB each)
static DSY_SDRAM_BSS float   looperAudioData[kNumberEffectSlots][kNumberChannelsStereo][kLooperAudioDataSamples] {{{0.0f}}};

using namespace infrasonic;

void Spotykach::init ()
{
  // Initialize the Spotykach looper pointers
  readIx_  = 0;
  writeIx_ = 0;

  // Initialize ADSR envelopes
  initEnvelopes(kSampleRate);
  // Default lengths; will be reconfigured each block based on window size
  configureEnvelopeLength(kBlockSize);
  prevReadIx_ = 0.0f;

  // Initialize the looper audio data buffers
  for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
  {
    if (isChannelActive(ch))
    {
      std::fill(std::begin(looperAudioData[effectSide_][ch]), std::end(looperAudioData[effectSide_][ch]), 0.0f);
    }
  }
}

// Initialize the envelopes
void Spotykach::initEnvelopes (float sampleRate)
{
  envSquare_.Init(sampleRate);
  envFall_.Init(sampleRate);
  envTri_.Init(sampleRate);
  envRise_.Init(sampleRate);
}

// Retrigger the envelopes
void Spotykach::retriggerEnvelopes (bool hard)
{
  envSquare_.Retrigger(hard);
  envFall_.Retrigger(hard);
  envTri_.Retrigger(hard);
  envRise_.Retrigger(hard);
}

// Configure ADSR segment times so that the total duration matches windowLenSamples
// Shapes:
//  - Square: fast attack, long sustain, fast release
//  - Falling Ramp: fast attack, long decay to 0
//  - Triangle: linear up then down
//  - Rising Ramp: long attack up to 1, fast release
void Spotykach::configureEnvelopeLength (float windowLenSamples)
{
  windowSamples_ = std::max(1.0f, windowLenSamples);
  const float windowLenSec = windowSamples_ / float(kSampleRate);

  // Square-ish
  envSquare_.SetAttackTime(std::min(0.001f, 0.1f * windowLenSec));
  envSquare_.SetDecayTime(0.001f);
  envSquare_.SetSustainLevel(1.0f);
  envSquare_.SetReleaseTime(std::min(0.001f, 0.1f * windowLenSec));

  // Falling ramp: Attack fast to 1, then decay across window
  envFall_.SetAttackTime(std::min(0.001f, 0.05f * windowLenSec));
  envFall_.SetDecayTime(std::max(0.001f, 0.95f * windowLenSec));
  envFall_.SetSustainLevel(0.0f);
  envFall_.SetReleaseTime(0.001f);

  // Triangle: half up, half down
  envTri_.SetAttackTime(std::max(0.001f, 0.5f * windowLenSec));
  envTri_.SetDecayTime(std::max(0.001f, 0.5f * windowLenSec));
  envTri_.SetSustainLevel(0.0f);
  envTri_.SetReleaseTime(0.001f);

  // Rising ramp: Attack over window, quick release
  envRise_.SetAttackTime(std::max(0.001f, 0.95f * windowLenSec));
  envRise_.SetDecayTime(0.001f);
  envRise_.SetSustainLevel(1.0f);
  envRise_.SetReleaseTime(std::min(0.001f, 0.05f * windowLenSec));
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
            std::fill(std::begin(looperAudioData[effectSide_][ch]), std::end(looperAudioData[effectSide_][ch]), 0.0f);
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
        // Switch to RECORDING state and start playing
        play_    = true;
        // Initialize the looper audio data buffers
        for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
        {
          if (isChannelActive(ch))
          {
            std::fill(std::begin(looperAudioData[effectSide_][ch]), std::end(looperAudioData[effectSide_][ch]), 0.0f);
          }
        }

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
}

void Spotykach::getDigitalControls(DigitalControlFrame &c)
{
  c.reverse = reverse_;
  c.play = play_;
  c.altPlay = record_;
  c.spotyPlay = false;  // Reset Spotykach+Play state
}

void Spotykach::populateLedRing (Effect::RingSpan &ringSpan,
                                 uint8_t ringSize,
                                 LedRgbBrightness colorBright,
                                 uint8_t start,
                                 uint8_t spanSize,
                                 bool gradient)
{
  LedRgbBrightness ledColor[Hardware::kNumLedsPerRing];
  // Clear ledColor
  std::fill(std::begin(ledColor), std::end(ledColor), LedRgbBrightness{0x000000, 0.0f});
  // Populate the current span with the specified color (at indicated brightness)
  std::fill(ledColor + start, std::min(ledColor + start + spanSize, std::end(ledColor)), colorBright);

  // Compute per-LED gradient values, using the brightness indicated in colorBright as maximum value
  if (gradient)
  {
    float ledGradient[Hardware::kNumLedsPerRing] = {0.0f};
    ledBrightnessGradient(spanSize, (colorBright.brightness/3.0f), colorBright.brightness, ledGradient);

    for (uint8_t i = 0; i < spanSize; ++i)
    {
      if (speed_ >= 0)
      {
        // Copy resulting gradients to the ledColor array
        ledColor[start + i] = {colorBright.rgb, ledGradient[i]};
      }
      else
      {
        // Copy resulting gradients to the ledColor array in reverse order
        ledColor[start + i] = {colorBright.rgb, ledGradient[spanSize - 1 - i]};
      }
    }
  }

  ringSpan.start = start;
  ringSpan.end   = std::min(static_cast<uint8_t>(start + spanSize), ringSize);
  std::copy(std::begin(ledColor), std::end(ledColor), std::begin(ringSpan.led));
}

void Spotykach::ledBrightnessGradient (uint8_t spanSize, float minBrightness, float maxBrightness, float* gradValues)
{
  // Per-LED gradient using analytic envelope within [0, spanSize)
  const uint8_t gradLen = std::min<uint8_t>(spanSize, Hardware::kNumLedsPerRing);
  std::memset(gradValues, 0, sizeof(float) * Hardware::kNumLedsPerRing);

  if (gradLen > 0)
  {
    for (uint8_t i = 0; i < gradLen; ++i)
    {
      float x    = (gradLen > 1) ? ((maxBrightness - minBrightness) * (float)i / (float)(gradLen - 1)) : 0.0f;
      float sq   = maxBrightness;
      float fall = (maxBrightness - x);
      float tri  = (x <= 0.5f) ? minBrightness + (x / 0.5f) : maxBrightness - (x / 0.5f);
      float rise = minBrightness + x;
      float t    = shape_;
      float v;
      if (t < 0.33333334f)
      {
        float u = t / 0.33333334f;
        v       = infrasonic::lerp(sq, fall, u);
      }
      else if (t < 0.6666667f)
      {
        float u = (t - 0.33333334f) / 0.33333334f;
        v       = infrasonic::lerp(fall, tri, u);
      }
      else
      {
        float u = (t - 0.6666667f) / 0.33333334f;
        v       = infrasonic::lerp(tri, rise, u);
      }
      gradValues[i] = daisysp::fclamp(v, minBrightness, maxBrightness);
    }
  }
}

void Spotykach::updateDisplayState()
{
  // Prepare a minimal display view without exposing internals
  DisplayState view{};
  view.playActive     = play_;
  view.reverseActive  = reverse_;

  if (reverse_)
  {
    std::fill(std::begin(view.reverseLedColors), std::end(view.reverseLedColors), LedRgbBrightness{0x0000ff, 1.0f});
  }

  constexpr uint8_t N = spotykach::Hardware::kNumLedsPerRing;
  Effect::RingSpan  ringSpan;

  switch (state_)
  {
    case ECHO:
    {
      // Factor the length of the echo into the LED ring (and zoom in twice for a better view)
      float positionFactor = 2.0f * position_ * (float)kEchoAudioDataSamples / (float)kLooperAudioDataSamples;

      // Compute spans in LED index space
      uint8_t start    = static_cast<uint8_t>((1.0f - positionFactor) * (N - 1));
      uint8_t spanSize = static_cast<uint8_t>(size_ * positionFactor * N);
      spanSize         = std::min(spanSize, (uint8_t)(Hardware::kNumLedsPerRing - start));

      // Yellow area indicating the position
      LedRgbBrightness ledColor = {0xffff00, 1.0f};
      populateLedRing(ringSpan, N, ledColor, start, N - start + 1);
      view.rings[view.layerCount++] = ringSpan;

      // Orange span indicating the size
      ledColor = {0xff8000, 1.0f};
      populateLedRing(ringSpan, N, ledColor, start, spanSize + 1, true);
      view.rings[view.layerCount++] = ringSpan;

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
      uint8_t readIxLed = std::min(static_cast<uint8_t>(start + positionFactor * spanSize), N);

      // Display read head position
      ledColor = {0xff00ff, 0.5f};
      populateLedRing(ringSpan, N, ledColor, readIxLed, 1);
      view.rings[view.layerCount++] = ringSpan;

      // Play LED colors
      view.playLedColors[0] = {0x00ff00, 1.0f};    // Green
      view.playLedColors[1] = {0x000000, 0.0f};    // Off

      break;
    }

    case RECORDING:
    {
      // Compute spans in LED index space
      uint8_t start    = 0;

      // Read head position
      uint8_t readIxLed  = static_cast<uint8_t>(readIx_ * (N - 1) / kLooperAudioDataSamples);

      // Write head position
      uint8_t writeIxLed = static_cast<uint8_t>(writeIx_ * (N - 1) / kLooperAudioDataSamples);

      // Yellow area indicating the writable area
      LedRgbBrightness ledColor = {0xffff00, 1.0f};
      populateLedRing(ringSpan, N, ledColor, start, N);
      view.rings[view.layerCount++] = ringSpan;

      // Orange area indicating the actively written area
      ledColor = {0xff8000, 1.0f};
      if (speed_ > 0)
      {
        populateLedRing(ringSpan, N, ledColor, readIxLed, N - readIxLed);
      }
      else
      {
        populateLedRing(ringSpan, N, ledColor, 0, writeIxLed);
      }
      view.rings[view.layerCount++] = ringSpan;

      // Display read head position
      ledColor = {0xff00ff, 0.5f};
      populateLedRing(ringSpan, N, ledColor, readIxLed, 1);
      view.rings[view.layerCount++] = ringSpan;

      // Display write head position
      ledColor = {0xff0000, 0.5f};
      populateLedRing(ringSpan, N, ledColor, writeIxLed, 1);
      view.rings[view.layerCount++] = ringSpan;

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
      uint8_t start    = static_cast<uint8_t>(position_ * (N - 1));
      uint8_t spanSize = static_cast<uint8_t>((1.0f - position_) * size_ * N);
      spanSize         = std::min(spanSize, (uint8_t)(Hardware::kNumLedsPerRing - start));

      // Read head position
      uint8_t readIxLed  = static_cast<uint8_t>(readIx_ * (N - 1) / kLooperAudioDataSamples);

      // Yellow area indicating the buffer
      LedRgbBrightness ledColor = {0xffff00, 1.0f};
      populateLedRing(ringSpan, N, ledColor, 0, N);
      view.rings[view.layerCount++] = ringSpan;

      // Orange span indicating the position and size
      ledColor = {0xff8000, 1.0f};
      populateLedRing(ringSpan, N, ledColor, start, spanSize, true);
      view.rings[view.layerCount++] = ringSpan;

      // Display read head position
      ledColor = {0xff00ff, 0.5f};
      populateLedRing(ringSpan, N, ledColor, readIxLed, 1);
      view.rings[view.layerCount++] = ringSpan;

      // Play LED colors
      if (play_)
      {
        // Solid Green
        std::fill(std::begin(view.playLedColors), std::end(view.playLedColors), LedRgbBrightness{0x00ff00, 1.0f});
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

float Spotykach::processEnvelope(bool gate)
{
  // gate indicates whether envelope should run this sample
  float a = envSquare_.Process(gate);
  float b = envFall_.Process(gate);
  float c = envTri_.Process(gate);
  float d = envRise_.Process(gate);
  // Interpolate across four shapes over shape_ in [0,1]
  float t = shape_;
  if (t < 0.33333334f)
  {
    float u = t / 0.33333334f;    // 0..1, Square->Falling
    return infrasonic::lerp(a, b, u);
  }
  else if (t < 0.6666667f)
  {
    float u = (t - 0.33333334f) / 0.33333334f;    // Falling->Triangle
    return infrasonic::lerp(b, c, u);
  }
  else
  {
    float u = (t - 0.6666667f) / 0.33333334f;    // Triangle->Rising
    return infrasonic::lerp(c, d, u);
  }
}

void Spotykach::processAudioSample (AudioHandle::InputBuffer  in,
                                    AudioHandle::OutputBuffer out,
                                    size_t sample,
                                    bool applyEnvelope,
                                    bool record)
{
  for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
  {
    // Only process if this channel is active in the current mode
    if (isChannelActive(ch))
    {
      float *currentLooperChannel = &looperAudioData[effectSide_][ch][0];

      // Neighbouring sample indices
      size_t rIdx0 = static_cast<size_t>(readIx_);
      size_t rIdx1 = (rIdx0 + 1) % kLooperAudioDataSamples;
      // Fractional part for interpolation
      float frac = readIx_ - rIdx0;
      // Neighbouring samples
      float s0 = currentLooperChannel[rIdx0];
      float s1 = currentLooperChannel[rIdx1];

      // If not playing, audition the input crossfaded with silence
      float loopOut = 0.0f;
      if (play_)
      {
        // Linear interpolation when between the two samples
        loopOut = infrasonic::lerp(s0, s1, frac);
      }
      // Mix the input with the loop output, with windowed envelope (if applied)
      float env  = applyEnvelope ? processEnvelope(play_) : 1.0f;
      float wet  = loopOut * env;
      out[ch][sample] = infrasonic::lerp(in[ch][sample], wet, mix_);

      if (record)
      {
        // Feedback/overdub
        size_t wIdx0                 = static_cast<size_t>(writeIx_);
        size_t wIdx1                 = static_cast<size_t>((wIdx0 + 1) % kLooperAudioDataSamples);
        float  old0                  = currentLooperChannel[wIdx0];
        float  old1                  = currentLooperChannel[wIdx1];
        currentLooperChannel[wIdx0] = in[ch][sample] + feedback_ * old0;
        currentLooperChannel[wIdx1] = in[ch][sample] + feedback_ * old1;
      }
    }
  }
}

void Spotykach::processAudio(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t blockSize)
{
  // Pass-through if mode is not REEL or channelConfig is not MONO_LEFT, MONO_RIGHT, or STEREO
  if ((mode_ != (Deck::DeckMode) REEL) ||
      (channelConfig_ == ChannelConfig::OFF) || (channelConfig_ >= ChannelConfig::CH_CONFIG_LAST))
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
      for (size_t i = 0; i < blockSize; ++i)
      {
        for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
        {
          // Only process if this channel is active in the current mode
          if (isChannelActive(ch))
          {
            out[ch][i] = infrasonic::lerp(in[ch][i], 0.0f, mix_);
          }
        }
      }
      return;
    }
    ///////////////
    case ECHO:
    {
      // Leave a margin to prevent the read index overtaking the write index
      float readWindowMargin = std::abs(speed_ * kBlockSize);
      // Using same span rule as display: (writeIx_ - position_, writeIx_ - position_*(1.0f-size_))
      float windowLen = std::max(1.0f, position_ * (float)kEchoAudioDataSamples * size_ - readWindowMargin);
      configureEnvelopeLength(windowLen * speed_);

      for (size_t i = 0; i < blockSize; ++i)
      {
        // Periodic envelope retrigger based on window length in samples
        if (envSampleCounter_ >= static_cast<uint32_t>(windowSamples_))
        {
          retriggerEnvelopes(false);
          envSampleCounter_ = 0;
        }
        envSampleCounter_++;

        // Process audio for the current sample
        processAudioSample(in, out, i, true, true);

        // Update the write index
        Span<float> writeSpan = {0.0f, kLooperAudioDataSamples};
        updateIndex(writeIx_, 1.0f, writeSpan);

        // Map position_ in (0,1) to (0, kEchoAudioDataSamples)
        float readWindowOffset = position_ * (float)kEchoAudioDataSamples;
        // Set the start of the read window to trail the write index (with wraparound)
        float readWindowStart = writeIx_;
        updateIndex(readWindowStart, -(readWindowOffset + readWindowMargin), writeSpan);
        // Set the end of the read window based on the size (with wraparound)
        float readWindowEnd = readWindowStart + (readWindowOffset * size_) - readWindowMargin;
        if (readWindowEnd > kLooperAudioDataSamples)
        {
          readWindowEnd -= kLooperAudioDataSamples;
        }

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
        // Process audio for the current sample
        processAudioSample(in, out, i, false, true);

        // Update the write index
        Span<float> writeSpan = {0.0f, kLooperAudioDataSamples};
        updateIndex(writeIx_, speed_, writeSpan);

        // Update the read index to follow the write index
        readIx_ = writeIx_;
      }
      return;
    }
    ///////////////
    case LOOP_PLAYBACK:
    {
      float windowLen = std::max(1.0f, (1.0f - position_) * size_ * kLooperAudioDataSamples);
      configureEnvelopeLength(windowLen * speed_);
      for (size_t i = 0; i < blockSize; ++i)
      {
        // Process audio for the current sample
        processAudioSample(in, out, i, true, false);

        // Map position_ relative to kLooperAudioDataSamples
        float pos = position_ * (float)kLooperAudioDataSamples;
        // Constrain readIx_ to span [spanStart, spanEnd)
        float spanStart = pos;
        float spanEnd   = pos + windowLen;
        if (spanEnd > kLooperAudioDataSamples)
        {
          spanEnd -= kLooperAudioDataSamples;
        }
        float prevReadIx = readIx_;

        if (play_)
        {
          // Advance read, write follows read
          Span<float> loopSpan = {spanStart, spanEnd};
          updateIndex(readIx_, speed_, loopSpan);
          writeIx_ = readIx_;
        }

        // Retrigger envelope on span wrap and constrain
        if (speed_ >= 0)
        {
          if (readIx_ < prevReadIx)
          {
            retriggerEnvelopes(false);
            readIx_ = spanStart;
          }
        }
        else
        {
          if (readIx_ >= prevReadIx)
          {
            retriggerEnvelopes(false);
            readIx_ = spanEnd - 1;
          }
        }
      }
      return;
    }
  }
}
