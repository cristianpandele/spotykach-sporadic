#include "app.h"
#include "Spotykach.h"

static constexpr size_t kLooperAudioDataSamples = 15.0f * kSampleRate * kNumberChannelsMono;
static constexpr size_t kEchoAudioDataSamples = 2.0f * kSampleRate;

// Reserve a buffer for the 15-second Spotykach looper (for each channel) - 16bit mono audio at 48khz (about 0.172 MB each)
static DSY_SDRAM_BSS float   looperAudioData[kNumberChannelsStereo][kLooperAudioDataSamples] {{0.0f}};

using namespace infrasonic;

void Spotykach::init ()
{
  // Initialize the Spotykach looper pointers
  readIx_  = 0;
  writeIx_ = 0;

  // Initialize ADSR envelopes
  initEnvelopes(sampleRate_);
  // Default lengths; will be reconfigured each block based on window size
  configureEnvelopeLength(blockSize_);
  prevReadIx_ = 0.0f;

  for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
  {
    // Initialize the input sculpt effect
    inputSculpt_[ch].init(sampleRate_);
  }
}

//////////
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

//////////
// Handle parameter changes
void Spotykach::setPosition (float p, bool gritLatch)
{
  bool positionChanged;
  bool positionChangedGrit;
  Deck::setPosition(p, gritLatch, positionChanged, positionChangedGrit);

  if (positionChangedGrit)
  {
    return;
  }
  else if (positionChanged && !getGritMenuOpen())
  {
    position_ = positionControl_;
  }
}

void Spotykach::setSize (float s, bool gritLatch)
{
  bool sizeChanged;
  bool sizeChangedGrit;
  Deck::setSize(s, gritLatch, sizeChanged, sizeChangedGrit);

  if (sizeChangedGrit)
  {
    return;
  }
  else if (sizeChanged  && !getGritMenuOpen())
  {
    // Ensure size is just a hair above 0 at all times
    size_ = daisysp::fmap(sizeControl_, 0.05f, 1.0f);
  }
}

void Spotykach::setShape (float s, bool gritLatch)
{
  bool shapeChanged;
  bool shapeChangedGrit;
  Deck::setShape(s, gritLatch, shapeChanged, shapeChangedGrit);

  if (shapeChangedGrit)
  {
    return;
  }
  else if (shapeChanged && !getGritMenuOpen())
  {
    shape_ = shapeControl_;
  }
}

void Spotykach::setPitch (float p, bool gritLatch)
{
  bool pitchChanged;
  bool pitchChangedGrit;
  Deck::setPitch(p, gritLatch, pitchChanged, pitchChangedGrit);

  if (pitchChangedGrit)
  {
    return;
  }
  else if (pitchChanged && !getGritMenuOpen())
  {
    // Map the pitch to 0..4
    speed_ = daisysp::fmap(pitchControl_, 0.0f, 4.0f);

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
        // Clear the audio buffers
        for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
        {
          if (isChannelActive(ch))
          {
            std::fill(std::begin(looperAudioData[ch]), std::end(looperAudioData[ch]), 0.0f);
          }
        }

        // Mark as not safe to play until we accumulate at least a buffer's worth of audio
        safeToPlay        = false;
        envSampleCounter_ = 0;

        // Log::PrintLine("Switching to ECHO state");
        state_ = ECHO;
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
        // Reset envelope sample counter
        envSampleCounter_ = 0;
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
            std::fill(std::begin(looperAudioData[ch]), std::end(looperAudioData[ch]), 0.0f);
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
  if (s)
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
  // Update the analog deck parameters based on the control frame
  // Use grit modifiers (pad latch or grit menu) to route to InputSculpt
  setMix(c.mix, c.mixAlt);
  setPitch(c.pitch, c.pitchGrit);
  setPosition(c.position, c.positionGrit);
  setSize(c.size, c.sizeGrit);
  setShape(c.shape, c.shapeGrit);
}

void Spotykach::updateDigitalControls (const DigitalControlFrame &c)
{
  // Update the digital deck parameters based on the control frame
  setSpotyPlay(c.spotyPlay);
  if (c.spotyPlay)
  {
    setReverse(false);
    setPlay(false);
    setAltPlay(false);
    return;
  }

  setReverse(c.reverse);
  setPlay(c.play);
  setAltPlay(c.altPlay);
  setFlux(c.flux);
  setGrit(c.grit);

  Deck::updateDigitalControlsEffects(c);
}

void Spotykach::getDigitalControls(DigitalControlFrame &c)
{
  c.reverse = reverse_;
  c.play = play_;
  c.altPlay = record_;
  c.spotyPlay = false;  // Reset Spotykach+Play state
  c.flux = flux_;
  c.altFlux = false; // Reset Alt+Flux state
  c.grit = grit_;
  c.altGrit = false; // Reset Alt+Grit state
}

void Spotykach::updateLooperDisplayState(DisplayState &view)
{
  constexpr uint8_t N = spotykach::Hardware::kNumLedsPerRing;
  Deck::RingSpan    ringSpan;

  // Yellow area indicating the buffer
  uint8_t start = 0;
  LedRgbBrightness ledColor = {0xffff00, kMidLedBrightness};
  populateLedRing(ringSpan, N, ledColor, start, N);
  view.rings[view.layerCount++] = ringSpan;

  switch (state_)
  {
    case ECHO:
    {
      // Factor the length of the echo into the LED ring (and zoom in twice for a better view)
      float positionFactor = 2.0f * position_ * (float)kEchoAudioDataSamples / (float)kLooperAudioDataSamples;

      // Compute spans in LED index space
      start = std::round(daisysp::fmap((1.0f - positionFactor), 0, N));
      start = daisysp::fclamp(start, 0, N - 1);

      // Orange span indicating the size
      ledColor         = {0xff8000, kMaxLedBrightness};
      uint8_t spanSize = std::round(daisysp::fmap(size_ * positionFactor, 0, N));
      spanSize         = daisysp::fclamp(spanSize, 0, (N - start));
      populateLedRing(ringSpan, N, ledColor, start, spanSize, true, reverse_);
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
      view.playLedColors[0] = {0x00ff00, kMaxLedBrightness};    // Green
      view.playLedColors[1] = {0x000000, kOffLedBrightness};    // Off

      break;
    }

    case RECORDING:
    {
      // Compute spans in LED index space
      // Read head position
      uint8_t readIxLed = static_cast<uint8_t>(
        infrasonic::map(readIx_, 0.0f, static_cast<float>(kLooperAudioDataSamples), 0.0f, static_cast<float>(N)));

      // Write head position
      uint8_t writeIxLed = static_cast<uint8_t>(
        infrasonic::map(writeIx_, 0.0f, static_cast<float>(kLooperAudioDataSamples), 0.0f, static_cast<float>(N)));

      // Orange area indicating the actively written area
      ledColor = {0xff8000, kMaxLedBrightness};
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

      // Play LED colors (always displayed in RECORDING state)
      view.playActive = true;
      if (play_)
      {
        view.playLedColors[0] = {0x00ff00, kMaxLedBrightness};    // Green
      }
      else
      {
        view.playLedColors[0] = {0x000000, kOffLedBrightness};    // Off
      }
      view.playLedColors[1] = {0xff0000, kMaxLedBrightness};    // Red

      break;
    }

    case LOOP_PLAYBACK:
    {
      // Compute spans in LED index space
      start = std::round(daisysp::fmap(position_, 0, N));
      start = daisysp::fclamp(start, 0, N);

      // Read head position
      uint8_t readIxLed = static_cast<uint8_t>(
        infrasonic::map(readIx_, 0.0f, static_cast<float>(kLooperAudioDataSamples), 0.0f, static_cast<float>(N)));

      // Orange span indicating the position and size
      ledColor         = {0xff8000, kMaxLedBrightness};
      uint8_t spanSize = std::round(daisysp::fmap((1.0f - position_) * size_, 0, N + 1));
      spanSize         = daisysp::fclamp(spanSize, 0, (N - start));
      populateLedRing(ringSpan, N, ledColor, start, spanSize, true, reverse_);
      view.rings[view.layerCount++] = ringSpan;

      // Display read head position
      ledColor = {0xff00ff, 0.5f};
      populateLedRing(ringSpan, N, ledColor, readIxLed, 1);
      view.rings[view.layerCount++] = ringSpan;

      // Play LED colors
      if (play_)
      {
        // Solid Green
        std::fill(std::begin(view.playLedColors), std::end(view.playLedColors), LedRgbBrightness{0x00ff00, kMaxLedBrightness});
      }

      break;
    }

    default:
    {
      // OFF state - no active rings
      // Play LED colors
      view.playLedColors[0] = {0x000000, kOffLedBrightness};    // Off
      view.playLedColors[1] = {0x000000, kOffLedBrightness};    // Off
      break;
    }
  }
}

void Spotykach::updateDisplayState()
{
  // Prepare a minimal display view without exposing internals
  DisplayState view{};
  view.playActive     = play_;
  view.reverseActive  = reverse_;

  // Reverse pad LED handling
  if (reverse_)
  {
    std::fill(std::begin(view.reverseLedColors),
              std::end(view.reverseLedColors),
              LedRgbBrightness{0x0000ff, kMaxLedBrightness});
  }

  // Check if there is an update to the held state of the effect pads
  detectEffectPadsHeld();

  if (isEffectPlaying())
  {
    // Flux/Grit pad LEDs and ring display
    updateEffectDisplayStates(view);
  }

  if (!isEffectDisplayed())
  {
    updateLooperDisplayState(view);
  }

  // Publish the state of the display
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
    while (index >= window.end)
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

void Spotykach::retriggerEnvelopesOnSpanWrap(float prevReadIx, Span<float> readSpan)
{
  // Retrigger envelope on span wrap and constrain
  if (speed_ >= 0)
  {
    if (readIx_ < prevReadIx)
    {
      retriggerEnvelopes(false);
      readIx_           = readSpan.start;
      envSampleCounter_ = 0;
    }
  }
  else
  {
    if (readIx_ >= prevReadIx)
    {
      retriggerEnvelopes(false);
      readIx_           = readSpan.end - 1;
      envSampleCounter_ = 0;
    }
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
      float *currentLooperChannel = &looperAudioData[ch][0];

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
        if ((mode_ == (Deck::DeckMode) ECHO && !safeToPlay)
          || (rIdx0 > kLooperAudioDataSamples - 2 && speed_ > 0))
          {
            // If not yet safe to play, do not output any looped audio
            loopOut = 0.0f;
          }
        else
        {
          // Normal playback
          // Linear interpolation when between the two samples
          loopOut = infrasonic::lerp(s0, s1, frac);
        }
      }
      // Mix the (sculpted) input with the loop output, with windowed envelope (if applied)
      float env  = applyEnvelope ? processEnvelope(play_) : 1.0f;
      float wet  = loopOut * env;
      // Process the input audio through the input sculpt effect
      float sculptedInput = inputSculpt_[ch].processSample(in[ch][sample]);
      out[ch][sample] = infrasonic::lerp(sculptedInput, wet, mix_);

      if (record)
      {
        // Feedback/overdub
        size_t wIdx0                 = static_cast<size_t>(writeIx_);
        size_t wIdx1                 = static_cast<size_t>((wIdx0 + 1) % kLooperAudioDataSamples);
        float  old0                  = currentLooperChannel[wIdx0];
        float  old1                  = currentLooperChannel[wIdx1];

        // Record the sculpted input (with potential feedback)
        currentLooperChannel[wIdx0] = sculptedInput + feedback_ * old0;
        currentLooperChannel[wIdx1] = sculptedInput + feedback_ * old1;
      }
    }
  }
}

void Spotykach::processAudio(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t blockSize)
{
  // Pass-through if channelConfig is not MONO_LEFT, MONO_RIGHT, or STEREO
  if ((channelConfig_ == ChannelConfig::OFF) || (channelConfig_ >= ChannelConfig::CH_CONFIG_LAST))
  {
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
            // Process the input audio through the input sculpt effect
            inputSculptBuf_[ch][i] = inputSculpt_[ch].processSample(in[ch][i]);
            out[ch][i] = infrasonic::lerp(inputSculptBuf_[ch][i], 0.0f, mix_);
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

      float writeIxSpeed = 1.0f;
      float relSpeed     = std::max(std::abs(speed_ - writeIxSpeed), 0.001f);
      configureEnvelopeLength(windowLen * relSpeed);

      for (size_t i = 0; i < blockSize; ++i)
      {
        if (!safeToPlay)
        {
          // When ECHO is first enabled, switch safeToPlay to true based on window length in samples
          if (envSampleCounter_ >= static_cast<uint32_t>(windowSamples_))
          {
            safeToPlay = true;
          }
          envSampleCounter_++;
        }

        // Process audio for the current sample
        processAudioSample(in, out, i, true, true);

        // Update the write index
        Span<float> writeSpan = {0.0f, kLooperAudioDataSamples};
        updateIndex(writeIx_, writeIxSpeed, writeSpan);

        // Map position_ in (0,1) to (0, kEchoAudioDataSamples)
        float readWindowOffset = position_ * (float)kEchoAudioDataSamples;
        // Set the start of the read window to trail the write index (with wraparound)
        float readWindowStart = writeIx_;
        updateIndex(readWindowStart, -(readWindowOffset), writeSpan);

        // Set the end of the read window based on the size (with wraparound)
        float readWindowEnd = readWindowStart + (readWindowOffset * size_) - readWindowMargin;
        while (readWindowEnd > kLooperAudioDataSamples - 2)
        {
          readWindowEnd = kLooperAudioDataSamples - 2;
        }

        // Save the read index before updating
        float prevReadIx = readIx_;

        // Update the read index
        Span<float> readSpan = {readWindowStart, readWindowEnd};
        updateIndex(readIx_, speed_, readSpan);

        // Retrigger envelopes on span wrap
        retriggerEnvelopesOnSpanWrap(prevReadIx, readSpan);
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

      float writeIxSpeed = speed_;

      configureEnvelopeLength(windowLen * writeIxSpeed);
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
        // Save the loop span
        Span<float> loopSpan = {spanStart, spanEnd};

        // Save the read index before updating
        float prevReadIx = readIx_;

        if (play_)
        {
          // Advance write, read follows write
          updateIndex(writeIx_, writeIxSpeed, loopSpan);
          readIx_ = writeIx_;
        }

        // Retrigger envelopes on span wrap
        retriggerEnvelopesOnSpanWrap(prevReadIx, loopSpan);
      }
      return;
    }
  }
}
